#!/usr/bin/env python3
# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
P0-a offline measured-gait analyser (read-only, ROS-free, deterministic).

Computes, from an already-recorded run, the *measured* gait metrics required to
judge whether a gait produces real forward displacement. It reuses the frozen
window/series helpers in ``tools/phase4e_analysis.py`` (and therefore never
re-defines the steady window); it never re-implements trace parsing twice.

Definition (must match docs/task_p0a_gait_search/measured_contract.md):

  body_xy(t)          measured base_link world position (body_pose_trace) on the
                      ground plane; the forward axis is the window-start heading
                      projected from world +x using the measured yaw.
  window              [lo, hi] = steady_window(cfg, ready)  (frozen definition)
  effective_duration  t_end - t_start over the window samples
  forward_disp        (end - start) . forward_unit
  lateral_disp        (end - start) . left_unit
  net_disp            ||end - start||
  actual_speed_mean   forward_disp / effective_duration
  speed_std           population std of the instantaneous forward speed series
  path_speed_mean     path_length / effective_duration

Inputs are never modified. All output goes to ``--out``. Any missing/corrupt
input raises an explicit error (no silent default, no empty "success" shape).

Usage:
  python3 tools/analyze_measured_gait.py \
      --runs-root docs/task4_phase2b_baseline_a_evidence --label phase2b \
      --runs-root docs/task4_phase2d_active_failure_boundary_evidence --label phase2d \
      --out docs/task_p0a_gait_search/baseline_existing_measured.json
"""

import argparse
import hashlib
import json
import math
import os
import sys
from collections import defaultdict

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)

import phase4e_analysis as A  # noqa: E402


class MeasuredGaitError(Exception):
    """Explicit, non-silent failure of the measured-gait analysis."""


# A window shorter than this (simulation seconds of *actual* measured coverage)
# is not a valid measurement (see docs/task_p0a_gait_search/measured_contract.md).
DEFAULT_MIN_EFFECTIVE_DURATION_SEC = 5.0
# Fall evidence: a drop of the base_link height to/below this (frozen Phase 2B
# torso-height gate) or an absolute torso tilt at/above this angle.
FALL_Z_THRESHOLD_M = 0.25
FALL_TILT_THRESHOLD_DEG = 60.0


# --------------------------------------------------------------------------- io
def load_jsonl(path):
    if not os.path.isfile(path):
        raise MeasuredGaitError('trace file missing: %s' % path)
    rows = []
    with open(path, 'r', encoding='utf-8') as fh:
        for lineno, line in enumerate(fh, 1):
            if not line.strip():
                continue
            try:
                rows.append(json.loads(line))
            except ValueError as exc:
                raise MeasuredGaitError(
                    'bad JSON in %s:%d: %s' % (path, lineno, exc))
    return rows


def finite(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(float(value)))


def body_series(trace_dir):
    rows = load_jsonl(os.path.join(trace_dir, 'body_pose_trace.jsonl'))
    if not rows:
        raise MeasuredGaitError('empty body_pose trace in %s' % trace_dir)
    for key in ('t', 'x', 'y'):
        for r in rows:
            if not finite(r.get(key)):
                raise MeasuredGaitError(
                    'body_pose row missing finite %r in %s' % (key, trace_dir))
    return rows


def contact_events(trace_dir):
    path = os.path.join(trace_dir, 'contact_trace.jsonl')
    if not os.path.isfile(path):
        return []
    out = []
    for r in load_jsonl(path):
        if finite(r.get('t')) and r.get('side') in ('left', 'right') \
                and isinstance(r.get('value'), bool):
            out.append((float(r['t']), r['side'], r['value']))
    return out


def ready_time(run_dir, trace_dir):
    result = os.path.join(run_dir, 'result.json')
    if os.path.isfile(result):
        with open(result, 'r', encoding='utf-8') as fh:
            data = json.load(fh)
        ready = (data.get('labels') or {}).get('ready_sim')
        if finite(ready):
            return float(ready)
    state = os.path.join(trace_dir, 'state_trace.jsonl')
    if os.path.isfile(state):
        rows = load_jsonl(state)
        if rows and finite(rows[0].get('t')):
            return float(rows[0]['t'])
    raise MeasuredGaitError(
        'cannot determine ready time for %s (no result.json/ready_sim, no state trace)'
        % run_dir)


# ------------------------------------------------------------------------ core
def compute_run(run_dir, source_label):
    cfg_path = os.path.join(run_dir, 'run_config.json')
    if not os.path.isfile(cfg_path):
        raise MeasuredGaitError('run_config.json missing: %s' % run_dir)
    with open(cfg_path, 'r', encoding='utf-8') as fh:
        cfg = json.load(fh)
    for key in ('duration_sec', 'start_delay_sec', 'gait_blend_sec'):
        if not finite(cfg.get(key)):
            raise MeasuredGaitError(
                'run_config missing numeric %r in %s' % (key, run_dir))
    # config_id/seed are recorded in result.json, not run_config.json.
    result_path = os.path.join(run_dir, 'result.json')
    if os.path.isfile(result_path):
        with open(result_path, 'r', encoding='utf-8') as fh:
            result_meta = json.load(fh)
        for key in ('config_id', 'seed', 'step_length_multiplier'):
            if cfg.get(key) is None and result_meta.get(key) is not None:
                cfg[key] = result_meta[key]

    trace_dir = os.path.join(run_dir, 'trace')
    ready = ready_time(run_dir, trace_dir)
    rows = body_series(trace_dir)

    t = [float(r['t']) for r in rows]
    x = [float(r['x']) for r in rows]
    y = [float(r['y']) for r in rows]
    z = [float(r['z']) for r in rows] if all(finite(r.get('z')) for r in rows) else None

    result = {
        'run_dir': run_dir,
        'source': source_label,
        'config_id': cfg.get('config_id'),
        'seed': cfg.get('seed'),
        'ready_sim': ready,
        'gait_parameters': {k: cfg.get(k) for k in (
            'step_length_m', 'step_frequency_hz', 'swing_height_m',
            'double_support_ratio', 'leg_extension_m', 'sway_amplitude_rad',
            'sway_sign', 'foot_pitch_swing_rad', 'gait_method',
            'walking_direction')},
        'controller_parameters': {k: cfg.get(k) for k in (
            'actuator_p_gain', 'actuator_max_velocity', 'command_rate_hz')},
        'anomalies': [],
    }

    # Attitude is mandatory (see measured_contract.md). A missing yaw must never
    # silently fall back to world +x, and a missing roll/pitch would hide a fall;
    # all three are rejected explicitly with the offending field and run id.
    bad_attitude = None
    for index, row in enumerate(rows):
        for field in ('yaw', 'roll', 'pitch'):
            if field not in row:
                bad_attitude = {'type': 'attitude_field_missing',
                                'field': field, 'index': index}
            elif not finite(row.get(field)):
                bad_attitude = {'type': 'attitude_field_nonfinite',
                                'field': field, 'index': index,
                                'value_repr': repr(row.get(field))}
            if bad_attitude:
                break
        if bad_attitude:
            break
    if bad_attitude is not None:
        bad_attitude['run_dir'] = run_dir
        result['measurable'] = False
        result['reason'] = 'invalid_attitude_field'
        result['anomalies'] = [bad_attitude]
        return result

    rolls = [float(r['roll']) for r in rows]
    pitches = [float(r['pitch']) for r in rows]
    yaws = [float(r['yaw']) for r in rows]

    anomalies = []
    for i in range(1, len(t)):
        if t[i] <= t[i - 1]:
            anomalies.append({'type': 'non_monotonic_timestamp',
                              'index': i, 't_prev': t[i - 1], 't': t[i]})
            break
    dt = [t[i] - t[i - 1] for i in range(1, len(t))]
    if dt:
        mean_dt = sum(dt) / len(dt)
        for i, d in enumerate(dt, 1):
            if d <= 0 or d > 20.0 * mean_dt:
                anomalies.append({'type': 'irregular_sample_interval',
                                  'index': i, 'dt': d, 'mean_dt': mean_dt})
                break

    lo, hi = A.steady_window(cfg, ready)
    idx = A.select_indices(t, lo, hi)
    # Nominal vs actual window. If the trace ends before the nominal right edge,
    # the actual window is the last valid measured pose; this is disclosed and
    # never silently presented as a complete nominal window.
    actual_lo = t[idx[0]] if idx else None
    actual_hi = t[idx[-1]] if idx else None
    effective_duration = (actual_hi - actual_lo) if actual_lo is not None else 0.0
    truncated = bool(idx and actual_hi < hi - 1e-9)
    missing_duration = (max(0.0, hi - actual_hi) if idx else 0.0)
    if idx and actual_lo > lo + 1e-9:
        missing_duration += (actual_lo - lo)
    result['window'] = {
        'nominal_lo': lo, 'nominal_hi': hi,
        'actual_lo': actual_lo, 'actual_hi': actual_hi,
        'n': len(idx), 'min_steady_samples': A.DEFAULT_MIN_STEADY_SAMPLES,
        'effective_duration_sec': effective_duration,
        'truncated': truncated, 'missing_duration_sec': missing_duration,
        'min_effective_duration_sec': DEFAULT_MIN_EFFECTIVE_DURATION_SEC,
    }
    result['anomalies'] = anomalies
    if len(idx) < 2:
        result['measurable'] = False
        result['reason'] = 'insufficient_steady_samples'
        result['n_steady_samples'] = len(idx)
        result['trace_truncated'] = truncated
        return result
    if effective_duration < DEFAULT_MIN_EFFECTIVE_DURATION_SEC:
        result['measurable'] = False
        result['reason'] = 'insufficient_effective_duration'
        result['n_steady_samples'] = len(idx)
        result['trace_truncated'] = truncated
        return result

    ts, xs, ys, zs = t[idx[0]], x[idx[0]], y[idx[0]], (z[idx[0]] if z else None)
    te, xe, ye, ze = t[idx[-1]], x[idx[-1]], y[idx[-1]], (z[idx[-1]] if z else None)
    duration = te - ts
    if duration <= 0:
        raise MeasuredGaitError('non-positive steady window in %s' % run_dir)

    # forward axis = window-start heading from measured yaw (unwrap first)
    uw = A.unwrap([yaws[i] for i in idx])
    yaw0 = uw[0]
    yaw_change_deg = math.degrees(uw[-1] - uw[0])
    fx, fy = math.cos(yaw0), math.sin(yaw0)        # forward unit
    lx, ly = -math.sin(yaw0), math.cos(yaw0)       # left unit
    dx, dy = xe - xs, ye - ys
    forward = dx * fx + dy * fy
    lateral = dx * lx + dy * ly
    net = math.hypot(dx, dy)

    # instantaneous forward speed series (projected onto forward unit)
    speeds = []
    path_len = 0.0
    max_single_step = 0.0
    for a, b in zip(idx, idx[1:]):
        dt_i = t[b] - t[a]
        if dt_i <= 0:
            continue
        vx = (x[b] - x[a]) / dt_i
        vy = (y[b] - y[a]) / dt_i
        speeds.append(vx * fx + vy * fy)
        step = math.hypot(x[b] - x[a], y[b] - y[a])
        max_single_step = max(max_single_step, step)
        path_len += step
    speed_mean = forward / duration
    speed_std = _pstdev(speeds) if speeds else None
    path_speed = path_len / duration

    tilt = max(max(abs(rolls[i]), abs(pitches[i])) for i in idx)
    final_tilt = max(abs(rolls[idx[-1]]), abs(pitches[idx[-1]]))
    tilt_deg = math.degrees(tilt)

    # Fall detection from actual measured pose evidence (never from a stored
    # failure_reason): a height drop to/below the frozen torso gate, or an
    # absolute torso tilt at/above the fall angle. Both evidences are kept.
    fall_evidence = []
    min_z = None
    if z is not None:
        min_z = min(z[i] for i in idx)
        if min_z <= FALL_Z_THRESHOLD_M:
            fall_evidence.append({'type': 'height_drop', 'min_z_m': min_z,
                                  'threshold_m': FALL_Z_THRESHOLD_M})
    if tilt_deg >= FALL_TILT_THRESHOLD_DEG:
        fall_evidence.append({'type': 'torso_tilt', 'max_tilt_deg': tilt_deg,
                              'threshold_deg': FALL_TILT_THRESHOLD_DEG})
    fall = bool(fall_evidence)

    contacts = contact_events(trace_dir)
    alt = A.alternation(contacts) if contacts else {'ok': False,
                                                    'reason': 'no_contact_events'}

    result.update({
        'measurable': True,
        'n_steady_samples': len(idx),
        'effective_duration_sec': duration,
        'trace_truncated': truncated,
        'missing_duration_sec': missing_duration,
        'start_pose': {'x': xs, 'y': ys, 'z': zs},
        'end_pose': {'x': xe, 'y': ye, 'z': ze},
        'yaw_start_rad': yaw0,
        'yaw_change_deg': yaw_change_deg,
        'body_dx_m': dx,
        'body_dy_m': dy,
        'body_dz_m': (ze - zs) if (z is not None) else None,
        'net_displacement_m': net,
        'forward_displacement_m': forward,
        'lateral_displacement_m': lateral,
        'actual_speed_mean_mps': speed_mean,
        'actual_speed_std_mps': speed_std,
        'path_speed_mean_mps': path_speed,
        'path_over_net': (path_len / net) if net > 1e-9 else None,
        'max_single_step_m': max_single_step,
        'torso_tilt_max_deg': tilt_deg,
        'torso_tilt_final_deg': math.degrees(final_tilt),
        'body_min_z_m': min_z,
        'fall': fall,
        'fall_evidence': fall_evidence,
        'contact': {'alternation_count': alt.get('alternation_count'),
                    'alternation_fraction': alt.get('alternation_fraction'),
                    'n_support': alt.get('n_support'),
                    'ok': alt.get('ok'),
                    'reason': alt.get('reason')},
    })
    if net < 1e-6 and abs(speed_mean) < 1e-6:
        result['anomalies'].append({'type': 'zero_net_displacement'})
    if abs(lateral) > abs(forward):
        result['anomalies'].append({'type': 'lateral_dominates_forward'})
    return result


def _pstdev(values):
    n = len(values)
    if n == 0:
        return None
    mu = sum(values) / n
    return math.sqrt(sum((v - mu) ** 2 for v in values) / n)


# ------------------------------------------------------------------- discovery
def find_runs(root):
    runs = []
    for dirpath, dirnames, filenames in os.walk(root):
        if os.path.isfile(os.path.join(dirpath, 'run_config.json')) and \
                os.path.isfile(os.path.join(dirpath, 'trace', 'body_pose_trace.jsonl')):
            runs.append(dirpath)
    if not runs:
        raise MeasuredGaitError('no run directories found under %s' % root)
    return sorted(runs)


def group_stats(runs):
    groups = defaultdict(list)
    for r in runs:
        if r.get('measurable'):
            groups[(r['source'], r.get('config_id'))].append(r)
    out = {}
    for key, rows_all in sorted(groups.items(),
                                key=lambda kv: (kv[0][0], str(kv[0][1]))):
        name = '%s::%s' % (key[0], key[1])
        # Fall episodes are never dropped: they keep their full measured
        # metrics and are reported separately. Normal-motion statistics are
        # computed over the non-fall measurable episodes only, so a fall can
        # never be used as evidence of a candidate's speed stability.
        non_fall = [r for r in rows_all if not r.get('fall')]
        fall_rows = [r for r in rows_all if r.get('fall')]
        n_all = len(rows_all)
        n_fall = len(fall_rows)
        out[name] = {
            'source': key[0],
            'config_id': key[1],
            'total_episode_count': n_all,
            'non_fall_measurable_count': len(non_fall),
            'fall_count': n_fall,
            'fall_rate': (n_fall / n_all) if n_all else None,
            'seeds': sorted({r.get('seed') for r in rows_all
                             if r.get('seed') is not None}),
            'actual_speed_mean_mps': _stats(
                [r['actual_speed_mean_mps'] for r in non_fall]),
            'actual_speed_std_within_mps': _stats(
                [r['actual_speed_std_mps'] for r in non_fall
                 if r.get('actual_speed_std_mps') is not None]),
            'forward_displacement_m': _stats(
                [r['forward_displacement_m'] for r in non_fall]),
            'net_displacement_m': _stats(
                [r['net_displacement_m'] for r in non_fall]),
            'lateral_displacement_m': _stats(
                [r['lateral_displacement_m'] for r in non_fall]),
            'torso_tilt_max_deg': _stats(
                [r['torso_tilt_max_deg'] for r in non_fall]),
            'yaw_change_deg': _stats(
                [r['yaw_change_deg'] for r in non_fall]),
            'fall_runs': [{
                'run_dir': r['run_dir'], 'seed': r.get('seed'),
                'actual_speed_mean_mps': r['actual_speed_mean_mps'],
                'forward_displacement_m': r['forward_displacement_m'],
                'lateral_displacement_m': r['lateral_displacement_m'],
                'net_displacement_m': r['net_displacement_m'],
                'torso_tilt_max_deg': r['torso_tilt_max_deg'],
                'fall_evidence': r.get('fall_evidence'),
            } for r in fall_rows],
            'gait_parameters': rows_all[0].get('gait_parameters'),
        }
    return out


def _stats(values):
    vals = [float(v) for v in values if finite(v)]
    if not vals:
        return {'n': 0, 'mean': None, 'std': None, 'min': None, 'max': None}
    # A single sample has no dispersion; report std/None instead of a fake 0.0.
    std = _pstdev(vals) if len(vals) >= 2 else None
    return {'n': len(vals), 'mean': _mean(vals), 'std': std,
            'min': min(vals), 'max': max(vals)}


def _mean(values):
    return sum(values) / len(values)


def between_groups(groups):
    names = sorted(groups)
    pairs = []
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = groups[names[i]], groups[names[j]]
            ma = a['actual_speed_mean_mps']['mean']
            mb = b['actual_speed_mean_mps']['mean']
            if not (finite(ma) and finite(mb)):
                continue
            wa = a['actual_speed_mean_mps']['std'] or 0.0
            wb = b['actual_speed_mean_mps']['std'] or 0.0
            pooled = math.sqrt((wa * wa + wb * wb) / 2.0)
            diff = mb - ma
            pairs.append({
                'pair': [names[i], names[j]],
                'mean_diff_mps': diff,
                'within_a_std': wa,
                'within_b_std': wb,
                'pooled_within_std': pooled,
                'separation': (abs(diff) / pooled) if pooled > 1e-12 else None,
                'abs_diff_gt_pooled': abs(diff) > pooled,
            })
    return pairs


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


# ------------------------------------------------------------------------ main
def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs-root', action='append', default=[],
                        help='recursively analyse every run under this root')
    parser.add_argument('--run', action='append', default=[],
                        help='analyse one specific run directory')
    parser.add_argument('--label', action='append', default=[],
                        help='source label for the matching --runs-root, positionally; '
                             'defaults to the root basename')
    parser.add_argument('--out', required=True, help='output JSON path')
    args = parser.parse_args(argv)

    if not args.runs_root and not args.run:
        parser.error('at least one --runs-root or --run is required')

    if args.label and len(args.label) != len(args.runs_root):
        raise MeasuredGaitError(
            '--label must be given once per --runs-root (%d labels for %d roots)'
            % (len(args.label), len(args.runs_root)))

    run_dirs = []
    for index, root in enumerate(args.runs_root):
        if not os.path.isdir(root):
            raise MeasuredGaitError('runs root does not exist: %s' % root)
        label = (args.label[index] if args.label
                 else os.path.basename(os.path.normpath(root)))
        run_dirs.extend((d, label) for d in find_runs(root))
    for d in args.run:
        if not os.path.isdir(d):
            raise MeasuredGaitError('run dir does not exist: %s' % d)
        run_dirs.append((d, os.path.basename(os.path.normpath(os.path.dirname(d)))
                         or 'explicit'))
    if not run_dirs:
        raise MeasuredGaitError('no runs discovered')

    runs = [compute_run(d, label) for d, label in run_dirs]
    groups = group_stats(runs)
    report = {
        'artifact': 'P0-a measured gait analysis',
        'tool': 'tools/analyze_measured_gait.py',
        'forward_axis_definition': (
            "base_link world position projected on the window-start measured "
            "heading (world +x rotated by unwrapped yaw at window start)"),
        'window_definition': (
            "phase4e_analysis.steady_window(cfg, ready) = "
            "[ready+start_delay+gait_blend, ready+start_delay+duration]"),
        'displacement_definition': 'end_pose - start_pose over window samples',
        'speed_definition': 'forward_displacement / effective_duration',
        'speed_std_definition': (
            'population std of per-sample projected forward velocity over the window'),
        'trace_truncation_definition': (
            "nominal window = steady_window(cfg, ready); actual window = "
            "[first,last] valid measured pose inside it; trace_truncated is true "
            "when actual_hi < nominal_hi; a window whose effective duration is "
            "below min_effective_duration_sec (%.1f s) is measurable=false"
            % DEFAULT_MIN_EFFECTIVE_DURATION_SEC),
        'fall_definition': (
            "measured fall evidence: base_link height <= %.2f m or absolute "
            "torso tilt >= %.1f deg inside the window; fall episodes keep full "
            "metrics and are excluded from non-fall speed statistics"
            % (FALL_Z_THRESHOLD_M, FALL_TILT_THRESHOLD_DEG)),
        'min_effective_duration_sec': DEFAULT_MIN_EFFECTIVE_DURATION_SEC,
        'n_runs': len(runs),
        'n_measurable': sum(1 for r in runs if r.get('measurable')),
        'n_fall': sum(1 for r in runs if r.get('measurable') and r.get('fall')),
        'runs': runs,
        'groups': groups,
        'between_group': between_groups(groups),
        'anomalies': [{'run_dir': r['run_dir'], 'anomalies': r['anomalies']}
                      for r in runs if r.get('anomalies')],
    }
    out_dir = os.path.dirname(os.path.abspath(args.out))
    os.makedirs(out_dir, exist_ok=True)
    with open(args.out, 'w', encoding='utf-8') as fh:
        json.dump(report, fh, indent=2, sort_keys=True)
    print('wrote %s (%d runs, %d measurable, %d groups)'
          % (args.out, len(runs), report['n_measurable'], len(groups)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
