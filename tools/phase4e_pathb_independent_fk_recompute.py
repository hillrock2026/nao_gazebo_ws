#!/usr/bin/env python3
"""Gate B3.0 — independent FK-dependent recompute (hardened for B3 reuse).

Rebuilds left/right foot world positions from the canonical URDF kinematics and
the raw state/body traces, then recomputes:
  * per-side foot swing height,
  * support-foot drift,
  * arm-leg phase match,
and compares them with the stored run_metrics.json values against FROZEN,
unit-bearing tolerances.

Independence: does NOT import tools/phase4e_analysis.py,
tools/phase4e_metrics.py or tools/phase4e_pathb_independent_recompute.py.
Kinematics are parsed directly from src/nao/nao_description/urdf/nao_simple.urdf.

Failure policy (B3.0):
  * every missing/non-finite/dimension/time-alignment problem is an explicit,
    structured failure (ok/reason/stage/path/detail) — never a 0.0 sentinel and
    never a PASS;
  * state/body matching is bounded by a FROZEN maximum skew (see
    DEFAULT_MAX_STATE_BODY_SKEW_SEC and docs/task4_phase4e_pathb_b3_evidence/
    b3_0/fk_tolerances.json); a run whose nearest body pose is too far fails with
    reason ``body_time_skew_exceeded`` instead of using a stale pose;
  * ``all_ok`` means every run's data is valid AND every comparison is within the
    frozen tolerance; the final line is either
    ``PASS: all comparisons within frozen tolerances`` or a FAIL message.

Safe output (B3.0):
  * --output-dir defaults to a new B3.0 evidence directory and never overwrites
    the historical review_corrections_v1 outputs;
  * workspace-external paths, symlink/``..`` escapes, files, and historical B2
    evidence/run/snapshot directories are rejected;
  * a non-empty/existing output dir is refused unless --overwrite is explicit;
  * JSON/TXT are published atomically (temp file + os.replace) after all
    computation; no recursive deletion is ever performed.

Usage: python3 tools/phase4e_pathb_independent_fk_recompute.py \
           [--output-dir DIR] [--overwrite] [--max-state-body-skew-sec F]
"""

import argparse
import json
import math
import os
import sys
import tempfile
import xml.etree.ElementTree as ET

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EV = os.path.join(WS, 'docs', 'task4_phase4e_pathb_evidence')
URDF = os.path.join(WS, 'src', 'nao', 'nao_description', 'urdf', 'nao_simple.urdf')
DEFAULT_OUT_DIR = os.path.join(WS, 'docs', 'task4_phase4e_pathb_b3_evidence',
                               'b3_0', 'fk_recompute')

GROUND_REF_Z = 0.0025
CLEARANCE_M = 0.004
CONTACT_GAP_SEC = 0.03

# Frozen maximum |state sample time - nearest body pose time|.
# Basis (recorded before any B3 candidate run):
#   * the body_pose topic is recorded at ~62.5 Hz (measured inter-sample gap
#     0.016-0.018 s) while joint state is recorded at 100 Hz (0.010 s);
#   * for a nearest-sample match the worst-case skew is half the body period
#     (~0.008 s); the measured maximum skew inside the steady window over all
#     nine B2 runs is 0.009 s;
#   * the frozen default is 0.02 s = ~2.2x the observed maximum, which absorbs
#     scheduler jitter while rejecting a stale (mis-matched) body pose.
DEFAULT_MAX_STATE_BODY_SKEW_SEC = 0.02

# Frozen comparison tolerances (absolute, same units as the compared quantity).
# Basis: the two independent implementations (production Phase 4C FK and this
# URDF-chain FK) share the analytic model but differ in ground-reference and
# clearance bookkeeping; over the nine B2 runs the observed absolute differences
# are <= 3.7e-5 m (swing L), <= 5.6e-5 m (swing R), <= 9.3e-4 m (drift) and
# exactly 0.0 (arm/leg match fraction). Tolerances are set with ~2x headroom,
# far below the 0.03 m forward gate, and are frozen before any B3 result.
FK_TOLERANCES = {
    'swing_l_m': 1.0e-4,
    'swing_r_m': 1.0e-4,
    'support_drift_m': 2.0e-3,
    'arm_leg_match_fraction': 1.0e-6,
}

FORBIDDEN_OUTPUT_PREFIXES = (
    os.path.join(WS, 'docs', 'task4_phase4e_pathb_evidence'),
    os.path.join(WS, 'docs', 'task4_phase4e_visual_evidence'),
)

OUTPUT_JSON_NAME = 'independent_fk_recompute.json'
OUTPUT_TXT_NAME = 'independent_fk_recompute.txt'

JOINT_ORDER = [
    'HeadYaw', 'HeadPitch',
    'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll',
    'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw',
    'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll',
    'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw',
]


# ----------------------------------------------------------------- error model
def structured_error(reason, stage, path=None, detail=None):
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


def _finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(float(v))


# ----------------------------------------------------------------- kinematics
def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def mat_vec(m, v):
    return [sum(m[i][k] * v[k] for k in range(3)) for i in range(3)]


def axis_angle(axis, angle):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    if n == 0:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z = x / n, y / n, z / n
    c, s, C = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]


def rpy_to_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def quat_to_mat(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return [[1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]]


def parse_urdf(urdf_path=URDF):
    """Parse the link/joint chain from a URDF file.

    Raises xml.etree.ElementTree.ParseError on malformed XML and OSError on a
    missing/unreadable file; callers translate these into structured failures.
    """
    root = ET.parse(urdf_path).getroot()
    chain = {}
    for j in root.iter('joint'):
        name, jt = j.get('name'), j.get('type')
        if not name or jt is None:
            continue
        origin = j.find('origin')
        axis = j.find('axis')
        xyz = [float(v) for v in (origin.get('xyz') if origin is not None else '0 0 0').split()]
        rpy = [float(v) for v in (origin.get('rpy') if origin is not None else '0 0 0').split()]
        ax = [float(v) for v in (axis.get('xyz') if axis is not None else '0 0 0').split()]
        chain[j.find('child').get('link')] = {
            'joint': name, 'parent': j.find('parent').get('link'),
            'xyz': xyz, 'rpy': rpy, 'axis': ax, 'type': jt}
    return chain


def fk(chain, target, joints, base='base_link'):
    """Return (R, t) of ``target`` link frame in ``base`` using URDF transforms."""
    ops = []
    cur = target
    while cur != base:
        if cur not in chain:
            raise ValueError('no URDF chain from %s to %s' % (target, base))
        e = chain[cur]
        ops.append(e)
        cur = e['parent']
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    t = [0.0, 0.0, 0.0]
    for e in reversed(ops):
        R = mat_mul(R, rpy_to_mat(*e['rpy']))
        d = mat_vec(R, e['xyz'])
        t = [t[i] + d[i] for i in range(3)]
        if e['type'] != 'fixed':
            ang = joints.get(e['joint'])
            if ang is None:
                raise ValueError('missing joint %s' % e['joint'])
            R = mat_mul(R, axis_angle(e['axis'], ang))
    return R, t


def read_jsonl(path):
    rows = []
    with open(path) as fh:
        for line in fh:
            if line.strip():
                rows.append(json.loads(line))
    return rows


# ----------------------------------------------------------- pure computations
def compute_support_drift(zs, xs, ts, ground_ref=GROUND_REF_Z,
                          clearance=CLEARANCE_M, contact_gap=CONTACT_GAP_SEC):
    """Max per-grounded-interval x drift, or a structured failure.

    Fails (never returns 0.0) on: dimension mismatch, non-finite input,
    non-increasing time, fewer than two grounded samples, or no grounded
    interval containing at least two samples.
    """
    if not (len(zs) == len(xs) == len(ts)):
        return structured_error('time_dim_mismatch', 'support_drift',
                                detail={'n_z': len(zs), 'n_x': len(xs), 'n_t': len(ts)})
    for seq in (zs, xs, ts):
        for v in seq:
            if not _finite(v):
                return structured_error('non_finite_input', 'support_drift')
    for a, b in zip(ts, ts[1:]):
        if not (b > a):
            return structured_error('time_not_increasing', 'support_drift')
    grounded = [(ts[i], xs[i], zs[i]) for i in range(len(ts))
                if zs[i] <= ground_ref + clearance]
    if len(grounded) < 2:
        return structured_error('insufficient_grounded_samples', 'support_drift',
                                detail={'grounded_samples': len(grounded)})
    intervals, cur = [], [grounded[0]]
    for g in grounded[1:]:
        if g[0] - cur[-1][0] <= contact_gap:
            cur.append(g)
        else:
            intervals.append(cur)
            cur = [g]
    intervals.append(cur)
    drifts = [abs(iv[-1][1] - iv[0][1]) for iv in intervals if len(iv) >= 2]
    if not drifts:
        return structured_error('no_valid_support_interval', 'support_drift',
                                detail={'n_intervals': len(intervals)})
    return {'ok': True, 'value': max(drifts), 'grounded_samples': len(grounded),
            'n_intervals': len(intervals)}


def compare_item(name, value, reference, tolerance):
    """Frozen-tolerance comparison of one independent value vs the reference."""
    rec = {'value': value, 'reference': reference, 'abs_diff': None,
           'tolerance': tolerance, 'pass': False}
    if not _finite(value):
        rec.update({'reason': 'independent_value_missing_or_non_finite'})
        return rec
    if not _finite(reference):
        rec.update({'reason': 'reference_missing_or_non_finite'})
        return rec
    diff = abs(float(value) - float(reference))
    rec['abs_diff'] = diff
    if diff > tolerance:
        rec.update({'reason': 'abs_diff_exceeds_tolerance'})
        return rec
    rec['pass'] = True
    return rec


def _nearest_body(bodies, t):
    """(row, skew) of the body pose nearest in time to ``t``."""
    best, bd = None, None
    for b in bodies:
        d = abs(b['t'] - t)
        if bd is None or d < bd:
            bd, best = d, b
    return best, bd


def recompute_run(label, run_dir, chain, max_state_body_skew_sec=DEFAULT_MAX_STATE_BODY_SKEW_SEC,
                  tolerances=None):
    """Recompute one run's FK metrics and compare with the stored references."""
    tolerances = dict(FK_TOLERANCES if tolerances is None else tolerances)
    rec = {'run_id': label, 'path': run_dir, 'ok': False, 'reason': None,
           'stage': None, 'detail': None, 'max_state_body_skew_sec': None,
           'n_window_samples': 0, 'value': {}, 'reference': {},
           'comparisons': {}}

    try:
        cfg = json.load(open(os.path.join(run_dir, 'run_config.json')))
        res = json.load(open(os.path.join(run_dir, 'run_result.json')))
        stored = json.load(open(os.path.join(run_dir, 'run_metrics.json')))
        state_path = os.path.join(run_dir, 'trace', 'state_trace.jsonl')
        body_path = os.path.join(run_dir, 'trace', 'body_pose_trace.jsonl')
        states = read_jsonl(state_path)
        bodies = read_jsonl(body_path)
    except OSError as exc:
        rec.update(structured_error('trace_or_config_missing', 'load', path=run_dir,
                                    detail='%s: %s' % (type(exc).__name__, exc)))
        return rec
    except ValueError as exc:
        rec.update(structured_error('trace_json_invalid', 'load', path=run_dir,
                                    detail='%s: %s' % (type(exc).__name__, exc)))
        return rec
    except KeyError as exc:
        rec.update(structured_error('trace_field_missing', 'load', path=run_dir,
                                    detail='KeyError: %s' % (exc,)))
        return rec

    if not states:
        rec.update(structured_error('empty_state_trace', 'load', path=state_path))
        return rec
    if not bodies:
        rec.update(structured_error('empty_body_trace', 'load', path=body_path))
        return rec

    ready = res['ready_sim']
    lo = ready + cfg['start_delay_sec'] + cfg['gait_blend_sec']
    hi = ready + cfg['start_delay_sec'] + cfg['duration_sec']

    L = {'z': [], 'x': []}
    R = {'z': [], 'x': []}
    wave = []
    times = []
    max_skew = 0.0
    for st in states:
        t = st['t']
        if not (lo <= t <= hi):
            continue
        if len(st['position']) != len(JOINT_ORDER):
            rec.update(structured_error('state_dim', 'fk', path=state_path,
                                        detail={'expected': len(JOINT_ORDER),
                                                'got': len(st['position'])}))
            return rec
        joints = {n: st['position'][i] for i, n in enumerate(JOINT_ORDER)}
        if any(not _finite(v) for v in joints.values()):
            rec.update(structured_error('non_finite_input', 'fk', path=state_path))
            return rec
        b, skew = _nearest_body(bodies, t)
        if b is None:
            rec.update(structured_error('body_time_skew_exceeded', 'body_match',
                                        path=body_path, detail='no body samples'))
            return rec
        max_skew = max(max_skew, skew)
        if skew > max_state_body_skew_sec:
            rec['max_state_body_skew_sec'] = max_skew
            rec.update(structured_error(
                'body_time_skew_exceeded', 'body_match', path=body_path,
                detail={'max_skew_sec': skew, 'limit_sec': max_state_body_skew_sec}))
            return rec
        Rb = quat_to_mat(b['qx'], b['qy'], b['qz'], b['qw'])
        base = [b['x'], b['y'], b['z']]
        for side, tgt, out in (('L', 'l_sole', L), ('R', 'r_sole', R)):
            _, p = fk(chain, tgt, joints)
            pw = [base[i] + sum(Rb[i][j] * p[j] for j in range(3)) for i in range(3)]
            out['z'].append(pw[2])
            out['x'].append(pw[0])
            if side == 'L':
                out.setdefault('base_x', []).append(p[0])
        wave.append(joints['LShoulderPitch'])
        times.append(t)

    rec['max_state_body_skew_sec'] = max_skew
    n = len(L['z'])
    rec['n_window_samples'] = n
    if n < 50:
        rec.update(structured_error('insufficient_window', 'fk', path=run_dir,
                                    detail={'n': n}))
        return rec

    swing_l = max(L['z']) - min(L['z'])
    swing_r = max(R['z']) - min(R['z'])
    drift_l = compute_support_drift(L['z'], L['x'], times)
    if is_structured_error(drift_l):
        rec.update(drift_l)
        return rec
    drift_r = compute_support_drift(R['z'], R['x'], times)
    if is_structured_error(drift_r):
        rec.update(drift_r)
        return rec
    drift = max(drift_l['value'], drift_r['value'])

    mean_w = sum(wave) / len(wave)
    dw = [w - mean_w for w in wave]
    basex = L['base_x']
    num = den = 0
    for k in range(1, len(times)):
        dt = times[k] - times[k - 1]
        if dt <= 0:
            continue
        vel = (basex[k] - basex[k - 1]) / dt
        den += 1
        if dw[k - 1] * vel > 0:
            num += 1
    if den == 0:
        rec.update(structured_error('no_valid_velocity_interval', 'arm_leg_match',
                                    path=run_dir))
        return rec
    leg_match = num / den

    value = {'swing_l_m': swing_l, 'swing_r_m': swing_r,
             'support_drift_m': drift, 'arm_leg_match_fraction': leg_match}
    reference = {
        'swing_l_m': stored.get('left_swing_m'),
        'swing_r_m': stored.get('right_swing_m'),
        'support_drift_m': stored.get('support_drift_m'),
        'arm_leg_match_fraction': (stored.get('arm') or {}).get('leg_match_fraction'),
    }
    rec['value'] = value
    rec['reference'] = reference
    comparisons = {}
    for name in ('swing_l_m', 'swing_r_m', 'support_drift_m',
                 'arm_leg_match_fraction'):
        comparisons[name] = compare_item(name, value[name], reference[name],
                                         tolerances[name])
    rec['comparisons'] = comparisons
    failed = [k for k, c in comparisons.items() if not c['pass']]
    if failed:
        rec.update({'ok': False, 'reason': 'comparison_out_of_tolerance',
                    'stage': 'compare', 'detail': {'failed': failed}})
        return rec
    rec.update({'ok': True, 'reason': None, 'stage': None, 'detail': None})
    return rec


# ------------------------------------------------------------- output safety
def resolve_output_dir(requested, overwrite, workspace=WS,
                       forbidden_prefixes=FORBIDDEN_OUTPUT_PREFIXES):
    """Validate/return the resolved output dir or a structured error."""
    ws = os.path.realpath(workspace)
    out = os.path.realpath(requested)
    if out != ws and not out.startswith(ws + os.sep):
        return None, structured_error('output_outside_workspace', 'output_dir',
                                      path=requested)
    for pref in forbidden_prefixes:
        p = os.path.realpath(pref)
        if out == p or out.startswith(p + os.sep):
            return None, structured_error('output_forbidden_historical_dir',
                                          'output_dir', path=requested,
                                          detail=p)
    if os.path.exists(out) and os.path.isfile(out):
        return None, structured_error('output_path_is_file', 'output_dir',
                                      path=requested)
    if os.path.isdir(out) and os.listdir(out) and not overwrite:
        return None, structured_error('output_dir_not_empty', 'output_dir',
                                      path=requested)
    return out, None


def _atomic_write(path, text):
    directory = os.path.dirname(path)
    fd, tmp = tempfile.mkstemp(dir=directory, prefix='.tmp_fk_', suffix='.part')
    try:
        with os.fdopen(fd, 'w') as fh:
            fh.write(text)
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def render_text(records, all_ok):
    lines = []
    for label, rec in records.items():
        if not rec.get('ok'):
            lines.append('%-18s FAIL reason=%s stage=%s detail=%s'
                         % (label, rec.get('reason'), rec.get('stage'),
                            rec.get('detail')))
            continue
        c = rec['comparisons']
        lines.append(
            '%-18s skew=%.4fs n=%d  swingL %.6f/%.6f d=%.2e tol=%.1e %s  '
            'swingR %.6f/%.6f d=%.2e tol=%.1e %s  drift %.6f/%.6f d=%.2e '
            'tol=%.1e %s  legmatch %.6f/%.6f d=%.2e tol=%.1e %s' % (
                label, rec['max_state_body_skew_sec'], rec['n_window_samples'],
                c['swing_l_m']['value'], c['swing_l_m']['reference'],
                c['swing_l_m']['abs_diff'], c['swing_l_m']['tolerance'],
                'PASS' if c['swing_l_m']['pass'] else 'FAIL',
                c['swing_r_m']['value'], c['swing_r_m']['reference'],
                c['swing_r_m']['abs_diff'], c['swing_r_m']['tolerance'],
                'PASS' if c['swing_r_m']['pass'] else 'FAIL',
                c['support_drift_m']['value'], c['support_drift_m']['reference'],
                c['support_drift_m']['abs_diff'], c['support_drift_m']['tolerance'],
                'PASS' if c['support_drift_m']['pass'] else 'FAIL',
                c['arm_leg_match_fraction']['value'],
                c['arm_leg_match_fraction']['reference'],
                c['arm_leg_match_fraction']['abs_diff'],
                c['arm_leg_match_fraction']['tolerance'],
                'PASS' if c['arm_leg_match_fraction']['pass'] else 'FAIL'))
    summary = ('PASS: all comparisons within frozen tolerances' if all_ok
               else 'FAIL: invalid run or comparison outside frozen tolerance')
    lines.append('')
    lines.append('Independent FK recompute: ' + summary)
    return '\n'.join(lines) + '\n'


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--output-dir', default=DEFAULT_OUT_DIR)
    ap.add_argument('--overwrite', action='store_true',
                    help='allow writing into an existing non-empty output dir '
                         '(only this tool\'s named files are replaced)')
    ap.add_argument('--max-state-body-skew-sec', type=float,
                    default=DEFAULT_MAX_STATE_BODY_SKEW_SEC)
    ap.add_argument('--runs-root', default=EV,
                    help='root holding charter/run_order.json and run dirs '
                         '(default: the frozen B2 evidence tree)')
    ap.add_argument('--run-order-file', default=None,
                    help='override run_order.json path (default: '
                         '<runs-root>/charter/run_order.json)')
    args = ap.parse_args(argv)

    out_dir, err = resolve_output_dir(args.output_dir, args.overwrite)
    if err is not None:
        print('OUTPUT DIR REJECTED: %s' % json.dumps(err, sort_keys=True),
              file=sys.stderr)
        return 2

    try:
        chain = parse_urdf()
    except (OSError, ET.ParseError, ValueError) as exc:
        print('URDF PARSE FAILED: %s' % json.dumps(structured_error(
            'urdf_parse_failed', 'urdf', path=URDF,
            detail='%s: %s' % (type(exc).__name__, exc)), sort_keys=True),
            file=sys.stderr)
        return 2

    runs_root = args.runs_root
    order_path = args.run_order_file or os.path.join(runs_root, 'charter',
                                                     'run_order.json')
    order = json.load(open(order_path))
    records = {}
    all_ok = True
    for label in order['order']:
        run_dir = os.path.join(runs_root, order['run_dir'][label])
        rec = recompute_run(label, run_dir, chain,
                            max_state_body_skew_sec=args.max_state_body_skew_sec)
        records[label] = rec
        if not rec['ok']:
            all_ok = False

    text = render_text(records, all_ok)
    report = {
        'tool': 'independent URDF FK recompute (B3.0)',
        'ground_ref_z': GROUND_REF_Z,
        'clearance_m': CLEARANCE_M,
        'max_state_body_skew_sec': args.max_state_body_skew_sec,
        'tolerances': FK_TOLERANCES,
        'all_ok': all_ok,
        'runs': records,
    }
    print(text, end='')
    try:
        os.makedirs(out_dir, exist_ok=True)
        _atomic_write(os.path.join(out_dir, OUTPUT_JSON_NAME),
                      json.dumps(report, indent=2, sort_keys=True) + '\n')
        _atomic_write(os.path.join(out_dir, OUTPUT_TXT_NAME), text)
    except OSError as exc:
        print('OUTPUT WRITE FAILED: %s: %s' % (type(exc).__name__, exc),
              file=sys.stderr)
        return 2
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
