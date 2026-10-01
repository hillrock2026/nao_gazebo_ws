#!/usr/bin/env python3
"""Shared, single-definition Phase 4E / Path B per-run metrics.

Combines the legacy Phase 4C/4E recomputation (body, legs, contact, IMU,
command) with the frozen Gate B1 primitives in ``phase4e_analysis`` (windows,
yaw wrap, alternation, config consistency). No ROS, no Gazebo.
"""

import json
import math
import os
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import phase4c_recompute as p4c  # noqa: E402
import phase4e_analysis as A  # noqa: E402

_PRODUCER_PATH = os.path.join(_TOOLS, '..', 'src', 'nao_gz_sim', 'scripts',
                              'phase4b_producer.py')

# Narrow, provable exception sets. Each entry is an anticipated, data-driven
# failure mode of the call chain below; programming errors and unknown
# exceptions are deliberately NOT listed and propagate to the caller.
#   OSError        : missing/unreadable trace or producer file
#   ValueError     : json.JSONDecodeError / bad float() text / non-finite angle
#   KeyError       : malformed JSONL row missing a required field
#   TypeError      : wrong value type in a row (dimension/shape error)
#   IndexError     : short joint/position vector
#   ZeroDivisionError : degenerate time span
_EXPECTED_DATA_ERRORS = (OSError, ValueError, KeyError, TypeError, IndexError,
                         ZeroDivisionError)
# Module loading of the canonical producer additionally can fail with
# SyntaxError (bad source) or ImportError (unavailable dependency such as
# rclpy) or AttributeError (missing JOINT_ORDER/LIMITS or spec is None).
_EXPECTED_MODULE_ERRORS = (OSError, ImportError, SyntaxError, AttributeError,
                           TypeError, ValueError)


def structured_error(reason, stage, path=None, detail=None):
    """Uniform, machine-readable failure result.

    Always contains ``ok`` (False), ``reason`` and ``stage``; ``path`` and
    ``detail`` are added when applicable. Never used to represent a valid
    numeric value, so a real value and an error can never be confused.
    """
    err = {'ok': False, 'reason': reason, 'stage': stage}
    if path is not None:
        err['path'] = path
    if detail is not None:
        err['detail'] = detail
    return err


def is_structured_error(value):
    return (isinstance(value, dict) and value.get('ok') is False
            and isinstance(value.get('reason'), str)
            and isinstance(value.get('stage'), str))


def _finite_number(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(float(value)))


def _load_jsonl(path):
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path) as fh:
        for line in fh:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def trace_rows(trace_dir, name):
    return _load_jsonl(os.path.join(trace_dir, '%s_trace.jsonl' % name))


def _body_series(trace_dir):
    rows = trace_rows(trace_dir, 'body_pose')
    t = [r['t'] for r in rows]
    x = [r['x'] for r in rows]
    y = [r['y'] for r in rows]
    z = [r['z'] for r in rows]
    yaw = [r.get('yaw', 0.0) for r in rows]
    return t, x, y, z, yaw


def _arm_metrics(trace_dir, cfg, ready, mode):
    states = trace_rows(trace_dir, 'state')
    neutral = float(cfg.get('arm_neutral_pitch_rad', 0.0))
    sign = float(cfg.get('arm_swing_sign', 1.0))
    amp = float(cfg.get('arm_swing_rad', 0.0))
    elbow = float(cfg.get('elbow_bend_rad', 0.0))
    if mode == 'stand':
        lo, hi = A.stand_window(cfg, ready)
        min_samples = A.DEFAULT_MIN_STAND_SAMPLES
    else:
        lo, hi = A.steady_window(cfg, ready)
        min_samples = A.DEFAULT_MIN_STEADY_SAMPLES
    times = [s['t'] for s in states]
    idx = A.select_indices(times, lo, hi)
    out = {'window_lo': lo, 'window_hi': hi, 'n': len(idx),
           'min_samples': min_samples, 'available': len(idx) >= min_samples}
    if len(idx) < min_samples:
        out['reason'] = 'insufficient_samples'
        return out
    L = [states[i]['position'][8] for i in idx]
    R = [states[i]['position'][19] for i in idx]
    E = [states[i]['position'][11] for i in idx]
    out.update({
        'LShoulderPitch_mean': sum(L) / len(L),
        'RShoulderPitch_mean': sum(R) / len(R),
        'LShoulderPitch_ptp': max(L) - min(L),
        'RShoulderPitch_ptp': max(R) - min(R),
        'LElbowRoll_mean': sum(E) / len(E),
        'configured_swing': amp, 'configured_neutral': neutral,
        'configured_sign': sign, 'configured_elbow': elbow,
        'arms_down': (sum(L) / len(L)) > 1.0 and (sum(R) / len(R)) > 1.0,
        'amplitude_ok': (abs((max(L) - min(L)) / 2.0 - abs(amp)) <= 0.03
                         and abs((max(R) - min(R)) / 2.0 - abs(amp)) <= 0.03),
        'elbow_ok': abs(sum(E) / len(E) + elbow) <= 0.05,
    })
    # anti-phase + leg coordination on the walk window (left foot FK velocity)
    if mode == 'walk' and len(L) >= 10:
        dl = [v - out['LShoulderPitch_mean'] for v in L]
        dr = [v - out['RShoulderPitch_mean'] for v in R]
        denom = math.sqrt(sum(a * a for a in dl) * sum(b * b for b in dr)) or 1e-9
        out['antiphase_corr'] = sum(a * b for a, b in zip(dl, dr)) / denom
        lm = _leg_match_fk(trace_dir, cfg, times, idx, dl)
        if is_structured_error(lm):
            out['leg_match_fraction'] = None
            out['leg_match_status'] = lm
        elif lm is None:
            out['leg_match_fraction'] = None
            out['leg_match_status'] = structured_error(
                'insufficient_leg_match_samples', 'leg_match_fk',
                path=trace_dir)
        else:
            out['leg_match_fraction'] = lm
    else:
        out['antiphase_corr'] = None
        out['leg_match_fraction'] = None
    return out


def _leg_match_fk(trace_dir, cfg, times, idx, wave):
    """Fraction where left-arm wave sign matches left-foot forward velocity.

    Uses the observed-foot FK (phase4c foot_samples) exactly like the Phase 4E
    aggregator, so the metric is reproducible by an independent implementation.
    """
    try:
        run = p4c.load_run(trace_dir)
        bounds, _ = p4c.segment_bounds(cfg, run)
        obs = p4c.foot_samples(run, bounds, 0.05)
        lx = {s['t']: s['base'][0] for s in obs['observed']['l']}
    except _EXPECTED_DATA_ERRORS as exc:
        return structured_error(
            'leg_match_input_invalid', 'leg_match_fk', path=trace_dir,
            detail='%s: %s' % (type(exc).__name__, exc))
    return A.leg_match_fraction(times, idx, wave, lx)


def _command_checks(trace_dir):
    rows = trace_rows(trace_dir, 'command')
    ok = [r for r in rows if r.get('ok')]
    out = {'total': len(rows), 'accepted': len(ok), 'rejected': len(rows) - len(ok),
           'reject_reasons': {}}
    for r in rows:
        if not r.get('ok'):
            out['reject_reasons'][r.get('reason', 'unknown')] = \
                out['reject_reasons'].get(r.get('reason', 'unknown'), 0) + 1
    t = [r['t'] for r in ok]
    out['strictly_increasing'] = all(t[i] > t[i - 1] for i in range(1, len(t)))
    if len(t) >= 2:
        out['rate_hz'] = (len(t) - 1) / (t[-1] - t[0])
        out['max_gap'] = max(t[i] - t[i - 1] for i in range(1, len(t)))
    limits = _command_limits(ok)
    if is_structured_error(limits):
        out['finite_in_limits'] = None
        out['finite_in_limits_status'] = limits
    else:
        out['finite_in_limits'] = limits
    return out


def _command_limits(ok):
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            'phase4b_producer', _PRODUCER_PATH)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        order, limits = mod.JOINT_ORDER, mod.LIMITS
    except _EXPECTED_MODULE_ERRORS as exc:
        return structured_error(
            'command_limit_reference_unavailable', 'command_limits',
            path=_PRODUCER_PATH,
            detail='%s: %s' % (type(exc).__name__, exc))
    for r in ok:
        pos = r.get('position', [])
        if len(pos) != len(order):
            return False
        for n, v in zip(order, pos):
            if not math.isfinite(v) or v < limits[n][0] - 1e-9 or v > limits[n][1] + 1e-9:
                return False
    return True


def _alternation(trace_dir):
    rows = trace_rows(trace_dir, 'contact')
    events = [(r['t'], r['side'], r['value']) for r in rows]
    res = A.alternation(events)
    res['n_events'] = len(events)
    return res


def run_metrics(trace_dir, cfg, ready, frozen_cfg=None, mode='walk'):
    """Compute the frozen per-run metric set and gate results."""
    metrics = {'mode': mode, 'ready_sim': ready}
    metrics['config_diffs'] = (A.config_consistency(cfg, frozen_cfg)
                               if frozen_cfg is not None else None)
    metrics['config_ok'] = (metrics['config_diffs'] == []
                            if frozen_cfg is not None else None)

    t, x, y, z, yaw = _body_series(trace_dir)
    metrics['body_trace_n'] = len(t)
    if mode == 'walk':
        lo, hi = A.steady_window(cfg, ready)
    else:
        lo, hi = A.stand_window(cfg, ready)
    idx = A.select_indices(t, lo, hi)
    metrics['window'] = {'lo': lo, 'hi': hi, 'n': len(idx)}
    if len(idx) >= 2:
        metrics['body_dx'] = x[idx[-1]] - x[idx[0]]
        metrics['body_dy'] = y[idx[-1]] - y[idx[0]]
        metrics['body_min_z'] = min(z[i] for i in idx)
        metrics['body_max_z'] = max(z[i] for i in idx)
    yr = A.yaw_drift(t, yaw, lo, hi, A.DEFAULT_MIN_STEADY_SAMPLES)
    metrics['yaw'] = yr
    metrics['max_yaw_abs_from_start_deg'] = yr['max_abs_from_start_deg']
    metrics['yaw_net_deg'] = yr['net_deg']

    metrics['arm'] = _arm_metrics(trace_dir, cfg, ready, mode)
    metrics['alternation'] = _alternation(trace_dir)
    metrics['command'] = _command_checks(trace_dir)
    imu = trace_rows(trace_dir, 'imu')
    metrics['imu_n'] = len(imu)
    metrics['imu_nonfinite'] = _imu_nonfinite(imu)

    # legacy Phase 4C recomputation for body roll/pitch, foot swing, sliding
    try:
        summary = p4c.analyze(trace_dir, cfg)
    except _EXPECTED_DATA_ERRORS as exc:
        summary = None
        metrics['phase4c_status'] = structured_error(
            'phase4c_analyze_failed', 'phase4c_analyze', path=trace_dir,
            detail='%s: %s' % (type(exc).__name__, exc))
    metrics['phase4c'] = summary
    if summary:
        rp = summary.get('body', {}).get('roll_pitch_deg', {}).get('steady_gait', {})
        metrics['max_roll_deg'] = rp.get('max_roll')
        metrics['max_pitch_deg'] = rp.get('max_pitch')
        foot = summary.get('foot_fk', {}).get('observed', {})
        metrics['left_swing_m'] = foot.get('l', {}).get('steady_gait', {}).get('swing_height')
        metrics['right_swing_m'] = foot.get('r', {}).get('steady_gait', {}).get('swing_height')
        sliding = summary.get('sliding', {})
        metrics['support_drift_m'] = max(
            (sliding.get(s, {}).get('max_interval_x_drift', 0.0)
             for s in ('l', 'r')), default=0.0)
    return metrics


def _imu_nonfinite(rows):
    n = 0
    for r in rows:
        for k in ('wx', 'wy', 'wz', 'ax', 'ay', 'az'):
            v = r.get(k)
            if not isinstance(v, (int, float)) or not math.isfinite(float(v)):
                n += 1
    return n


def evaluate_gates(m, thresholds, group):
    """Apply the frozen hard gates to one run's metrics."""
    hard = thresholds['hard_gates']
    g = {}

    def add(name, passed, value=None, detail=None):
        g[name] = {'pass': bool(passed), 'value': value, 'detail': detail}

    add('config_matches_frozen_charter', m.get('config_ok') is True,
        m.get('config_diffs'))
    cmd = m.get('command', {})
    add('command_rate_100hz', cmd.get('rate_hz') is not None
        and abs(cmd['rate_hz'] - 100.0) <= 3.0, cmd.get('rate_hz'))
    add('command_reject_zero', cmd.get('rejected') == 0, cmd.get('rejected'))
    add('command_strictly_increasing', cmd.get('strictly_increasing') is True,
        cmd.get('strictly_increasing'))
    # A structured failure is an explicit FAIL, never masked by a comparison
    # against a defaulted 0.0.
    cmd_limits_ok = (cmd.get('finite_in_limits') is True
                     and not is_structured_error(cmd.get('finite_in_limits_status')))
    add('command_finite_in_limits', cmd_limits_ok, cmd.get('finite_in_limits'),
        cmd.get('finite_in_limits_status'))
    add('imu_no_nonfinite', m.get('imu_nonfinite') == 0, m.get('imu_nonfinite'))
    if m['mode'] == 'walk':
        # Phase 4C-derived metrics must be present and finite; a missing value
        # (e.g. an analyze failure) fails the gate instead of defaulting to 0.
        phase4c_ok = not is_structured_error(m.get('phase4c_status'))
        yr = m.get('yaw', {})
        add('yaw_drift_deg_max_abs_le_5',
            yr.get('ok') and abs(yr['max_abs_from_start_deg']) <= 5.0,
            yr.get('max_abs_from_start_deg'), yr.get('reason'))
        add('lateral_drift_le_0.05', _finite_number(m.get('body_dy'))
            and abs(m['body_dy']) <= 0.05, m.get('body_dy'))
        add('roll_le_15', phase4c_ok and _finite_number(m.get('max_roll_deg'))
            and m['max_roll_deg'] <= 15.0, m.get('max_roll_deg'),
            m.get('phase4c_status'))
        add('pitch_le_15', phase4c_ok and _finite_number(m.get('max_pitch_deg'))
            and m['max_pitch_deg'] <= 15.0, m.get('max_pitch_deg'),
            m.get('phase4c_status'))
        add('torso_min_height_gt_0.25', _finite_number(m.get('body_min_z'))
            and m['body_min_z'] > 0.25, m.get('body_min_z'))
        add('support_drift_le_0.03',
            phase4c_ok and _finite_number(m.get('support_drift_m'))
            and m['support_drift_m'] <= 0.03, m.get('support_drift_m'),
            m.get('phase4c_status'))
        alt = m.get('alternation', {})
        add('alternation_ok', alt.get('ok') is True,
            {'count': alt.get('alternation_count'),
             'fraction': alt.get('alternation_fraction'),
             'n_support': alt.get('n_support')}, alt.get('reason'))
        a = m.get('arm', {})
        swing = abs(a.get('configured_swing') or 0.0)
        base_ok = bool(a.get('available') and a.get('elbow_ok'))
        if group == 'CONTROL':
            arm_ok = base_ok and a.get('antiphase_corr') is not None
        elif swing < 1e-9:
            # NATURAL-STATIC: pose only (no swing by design); no anti-phase req.
            arm_ok = base_ok and bool(a.get('arms_down'))
        else:
            leg_match = a.get('leg_match_fraction')
            leg_match_ok = (not is_structured_error(a.get('leg_match_status'))
                            and _finite_number(leg_match)
                            and leg_match >= 0.7)
            arm_ok = (base_ok and a.get('arms_down') and a.get('amplitude_ok')
                      and _finite_number(a.get('antiphase_corr'))
                      and a['antiphase_corr'] <= -0.9
                      and leg_match_ok)
        add('arm_natural_down_and_antiphase' if group != 'CONTROL'
            else 'arm_control_reproducible', arm_ok,
            {'mean_L': a.get('LShoulderPitch_mean'),
             'mean_R': a.get('RShoulderPitch_mean'),
             'ptp_L': a.get('LShoulderPitch_ptp'),
             'corr': a.get('antiphase_corr'),
             'leg_match': a.get('leg_match_fraction')})
    return g
