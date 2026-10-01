#!/usr/bin/env python3
"""Task 4 Phase 4C independent recomputation and gate evaluation.

This tool is read-only with respect to the simulation. It consumes the RAW
JSONL traces written by ``phase4b_trace.py`` (clock / command / state / contact
/ body_pose / imu) and recomputes every Phase 4C metric *without* trusting the
producer or the Phase 4B summary:

  * command/state response with startup / gait_blend / steady / stop_ramp /
    final_hold segmentation;
  * per-segment per-joint tracking error (max / mean / RMS) and limit margin;
  * forward kinematics of BOTH the commanded targets and the observed joint
    states, in the base frame and in the world frame (using the recorded body
    quaternion directly);
  * main-collision contact evidence (from the frozen Bool topics) and a
    *supplementary* foot-ground contact signal derived from the observed FK sole
    height, reported separately;
  * body-motion / height / attitude / IMU stability and per-window displacement;
  * sliding and fall indicators.

Usage:
    python3 tools/phase4c_recompute.py analyze <run_dir> --config <run_config.json>
    python3 tools/phase4c_recompute.py aggregate --baseline <dir> \
        --runs <d1> <d2> <d3> --pause <dir> [--out <summary.json>]
"""

import argparse
import json
import math
import os
import sys

JOINT_ORDER = [
    'HeadYaw', 'HeadPitch',
    'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll',
    'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw',
    'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll',
    'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw',
]
LIMITS = {
    'HeadYaw': (-2.08567, 2.08567),
    'HeadPitch': (-0.671952, 0.514872),
    'LHipYawPitch': (-1.14529, 0.740718),
    'LHipRoll': (-0.379435, 0.79046),
    'LHipPitch': (-1.53589, 0.48398),
    'LKneePitch': (-0.0923279, 2.11255),
    'LAnklePitch': (-1.18944, 0.922581),
    'LAnkleRoll': (-0.397761, 0.768992),
    'LShoulderPitch': (-2.08567, 2.08567),
    'LShoulderRoll': (-0.314159, 1.32645),
    'LElbowYaw': (-2.08567, 2.08567),
    'LElbowRoll': (-1.54462, -0.0349066),
    'LWristYaw': (-1.82387, 1.82387),
    'RHipYawPitch': (-1.14529, 0.740718),
    'RHipRoll': (-0.79046, 0.379435),
    'RHipPitch': (-1.53589, 0.48398),
    'RKneePitch': (-0.0923279, 2.11255),
    'RAnklePitch': (-1.1863, 0.932006),
    'RAnkleRoll': (-0.768992, 0.397761),
    'RShoulderPitch': (-2.08567, 2.08567),
    'RShoulderRoll': (-1.32645, 0.314159),
    'RElbowYaw': (-2.08567, 2.08567),
    'RElbowRoll': (0.0349066, 1.54462),
    'RWristYaw': (-1.82387, 1.82387),
}

# SDF main ankle collision bottom (pose z -0.013014, half size 0.032936) and
# FSR box bottom (pose z -0.04511, half 0.0025). Documented here so the
# supplementary contact threshold is explainable.
ANKLE_MAIN_BOTTOM = -0.013014 - 0.065872 / 2.0   # = -0.045950
FSR_BOTTOM = -0.04511 - 0.005 / 2.0              # = -0.047610

# Phase 4C review P2 #1: the supplementary stance reference is ANCHORED to the
# canonical SDF geometry, never derived from the measured run. The FSR box bottom
# is 0.0025 m below the l_sole frame (l_ankle->l_sole fixed offset 0.04511;
# FSR box pose z 0.04511 + half size 0.0025), so when the FSR box rests on
# ground_plane (z = 0) the l_sole frame is at z = +0.0025 m. This is an
# independent geometric constant, not `min(observed steady z)`.
GROUND_REF_SOLE_Z_M = (0.04511 + 0.005 / 2.0) - 0.04511  # == 0.0025


# --------------------------------------------------------------- small matrix
def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def axis_angle(axis, q):
    n = math.sqrt(sum(a * a for a in axis))
    if n < 1e-12:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z = (a / n for a in axis)
    c, s = math.cos(q), math.sin(q)
    C = 1 - c
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


LEG_CHAIN = {
    'L': ('LHipYawPitch', (0.0, 0.707106, -0.707106), (0.0, 0.05, -0.085),
          'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'),
    'R': ('RHipYawPitch', (0.0, 0.707106, 0.707106), (0.0, -0.05, -0.085),
          'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'),
}


def sole_position_base(q, side):
    """Exact serial-chain FK of the sole frame in base_link (Phase 4C fix).

    Each link offset is expressed in the current link frame, matching
    ``nao_simple.urdf``; see phase4b_trace.py for the documented defect.
    """
    yawp_name, yp_axis, yp_origin, roll_name, hp_name, kn_name, ap_name, ar_name = LEG_CHAIN[side]
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    t = [float(yp_origin[0]), float(yp_origin[1]), float(yp_origin[2])]
    R = mat_mul(R, axis_angle(yp_axis, q[yawp_name]))
    R = mat_mul(R, axis_angle((1, 0, 0), q[roll_name]))
    R = mat_mul(R, axis_angle((0, 1, 0), q[hp_name]))
    d = mat_vec(R, [0.0, 0.0, -0.100])
    t = [t[i] + d[i] for i in range(3)]
    R = mat_mul(R, axis_angle((0, 1, 0), q[kn_name]))
    d = mat_vec(R, [0.0, 0.0, -0.1029])
    t = [t[i] + d[i] for i in range(3)]
    R = mat_mul(R, axis_angle((0, 1, 0), q[ap_name]))
    R = mat_mul(R, axis_angle((1, 0, 0), q[ar_name]))
    d = mat_vec(R, [0.0, 0.0, -0.04511])
    t = [t[i] + d[i] for i in range(3)]
    return t


# ------------------------------------------------------------------- loading
def load_jsonl(path):
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path) as fh:
        for line in fh:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def load_run(run_dir):
    run = {}
    run['commands'] = load_jsonl(os.path.join(run_dir, 'command_trace.jsonl'))
    run['states'] = load_jsonl(os.path.join(run_dir, 'state_trace.jsonl'))
    run['bodies'] = load_jsonl(os.path.join(run_dir, 'body_pose_trace.jsonl'))
    run['contacts'] = load_jsonl(os.path.join(run_dir, 'contact_trace.jsonl'))
    run['imu'] = load_jsonl(os.path.join(run_dir, 'imu_trace.jsonl'))
    run['clock'] = load_jsonl(os.path.join(run_dir, 'clock_trace.jsonl'))
    return run


def load_config(run_dir):
    cfg_path = os.path.join(run_dir, 'run_config.json')
    if os.path.exists(cfg_path):
        with open(cfg_path) as fh:
            return json.load(fh)
    return {}


# ------------------------------------------------------------- segmentation
def segment_bounds(cfg, run):
    """Return (bounds dict, notes). All values in simulation seconds."""
    states = run['states']
    bodies = run['bodies']
    t_first_state = states[0]['t'] if states else None
    t_first_body = bodies[0]['t'] if bodies else None
    t_last = None
    for key in ('bodies', 'states', 'imu'):
        if run[key]:
            t_last = run[key][-1]['t'] if t_last is None else max(t_last, run[key][-1]['t'])

    mode = cfg.get('gait_method', 'walk')
    start_delay = float(cfg.get('start_delay_sec', 0.5))
    blend = float(cfg.get('gait_blend_sec', 1.5))
    duration = float(cfg.get('duration_sec', 22.0))
    ramp = float(cfg.get('stop_ramp_sec', 1.5))
    hold = float(cfg.get('hold_final_sec', 2.0))

    bounds = {}
    t_ready = t_first_state
    if t_ready is None:
        return {}, ['no joint state samples']
    if mode == 'walk':
        gait_start = t_ready + start_delay
        bounds = {
            'spawn': (t_first_body, t_ready),
            'settling': (t_ready, gait_start),
            'gait_blend': (gait_start, gait_start + blend),
            'steady_gait': (gait_start + blend, gait_start + duration),
            'stop_ramp': (gait_start + duration, gait_start + duration + ramp),
            'final_hold': (gait_start + duration + ramp,
                           gait_start + duration + ramp + hold),
        }
    else:
        # Static baseline: settle, then a long steady window and a final hold so
        # that the same settled/absolute height statistics are available.
        settle_end = t_ready + max(start_delay, 1.5)
        hold_lo = max(settle_end, (t_last or settle_end) - 2.0)
        bounds = {
            'spawn': (t_first_body, t_ready),
            'settling': (t_ready, settle_end),
            'steady_gait': (settle_end, hold_lo),
            'final_hold': (hold_lo, t_last),
        }
    return bounds, []


def in_segment(bounds, name, t):
    if name not in bounds or bounds[name][0] is None:
        return False
    lo, hi = bounds[name]
    return lo <= t < hi


# --------------------------------------------------------------- command side
def command_metrics(run, cfg):
    accepted = [c for c in run['commands'] if c.get('ok')]
    rejected = [c for c in run['commands'] if not c.get('ok')]
    res = {
        'accepted': len(accepted),
        'rejected': len(rejected),
        'reject_reasons': {},
    }
    for r in rejected:
        reason = str(r.get('reason'))
        res['reject_reasons'][reason] = res['reject_reasons'].get(reason, 0) + 1
    if not accepted:
        res['strictly_increasing'] = False
        return res, []
    stamps = [c['t'] for c in accepted]
    span = stamps[-1] - stamps[0]
    res['stamp_min'] = stamps[0]
    res['stamp_max'] = stamps[-1]
    res['strictly_increasing'] = all(b > a for a, b in zip(stamps, stamps[1:]))
    res['rate_hz'] = (len(stamps) - 1) / span if span > 0 else None
    res['max_stamp_gap_sec'] = max(
        (b - a for a, b in zip(stamps, stamps[1:])), default=0.0)
    res['dim_ok'] = all(len(c.get('position', [])) == 24 for c in accepted)
    res['velocity_effort_empty'] = True
    res['in_limit'] = all(
        LIMITS[n][0] - 1e-9 <= v <= LIMITS[n][1] + 1e-9
        for c in accepted for n, v in zip(JOINT_ORDER, c['position']))
    res['position_finite'] = all(
        math.isfinite(v) for c in accepted for v in c['position'])
    # bounded command step
    steps = []
    for a, b in zip(accepted, accepted[1:]):
        steps.append(max(abs(x - y) for x, y in zip(a['position'], b['position'])))
    res['max_command_step_rad'] = max(steps) if steps else 0.0
    return res, accepted


def hold_last_command(accepted, t, hold_max):
    """Largest command stamp <= t within hold_max; None if none."""
    lo, hi = 0, len(accepted)
    idx = -1
    while lo < hi:
        mid = (lo + hi) // 2
        if accepted[mid]['t'] <= t:
            idx = mid
            lo = mid + 1
        else:
            hi = mid
    if idx < 0:
        return None
    if t - accepted[idx]['t'] > hold_max + 1e-9:
        return None
    return accepted[idx]


def seg_name(bounds, t):
    for name in ('spawn', 'settling', 'gait_blend', 'steady_gait', 'stop_ramp', 'final_hold'):
        if in_segment(bounds, name, t):
            return name
    if bounds.get('steady_gait') and t >= bounds['steady_gait'][0]:
        return 'after'
    if bounds.get('spawn') and bounds['spawn'][0] is not None and t < bounds['spawn'][0]:
        return 'before'
    return 'other'


# ------------------------------------------------------------------ tracking
def tracking_metrics(run, accepted, bounds, hold_max):
    segs = ['spawn', 'settling', 'gait_blend', 'steady_gait', 'stop_ramp', 'final_hold']
    data = {s: {n: {'max': 0.0, 'sum': 0.0, 'sumsq': 0.0, 'count': 0} for n in JOINT_ORDER}
            for s in segs}
    overall = {n: {'max': 0.0} for n in JOINT_ORDER}
    margin = {n: 1e9 for n in JOINT_ORDER}
    coverage_hits = 0
    coverage_total = 0
    missing_hold = 0
    active_lo = accepted[0]['t'] if accepted else None
    active_hi = accepted[-1]['t'] if accepted else None
    active_hits = 0
    active_total = 0
    tail_samples = 0
    for st in run['states']:
        t = st['t']
        pos = st['position']
        cmd = hold_last_command(accepted, t, hold_max)
        coverage_total += 1
        if active_lo is not None and active_lo <= t <= active_hi:
            active_total += 1
            if cmd is not None:
                active_hits += 1
        elif active_hi is not None and t > active_hi:
            tail_samples += 1
        if cmd is None:
            missing_hold += 1
            continue
        coverage_hits += 1
        s = seg_name(bounds, t)
        for i, n in enumerate(JOINT_ORDER):
            e = abs(pos[i] - cmd['position'][i])
            overall[n]['max'] = max(overall[n]['max'], e)
            if s in data:
                d = data[s][n]
                d['max'] = max(d['max'], e)
                d['sum'] += e
                d['sumsq'] += e * e
                d['count'] += 1
            lo, hi = LIMITS[n]
            margin[n] = min(margin[n], pos[i] - lo, hi - pos[i])
    for s in segs:
        for n in JOINT_ORDER:
            d = data[s][n]
            if d['count']:
                d['mean'] = d['sum'] / d['count']
                d['rms'] = math.sqrt(d['sumsq'] / d['count'])
            else:
                d['mean'] = None
                d['rms'] = None
    res = {
        'per_segment': {s: {n: {'max': data[s][n]['max'],
                                'mean': data[s][n]['mean'],
                                'rms': data[s][n]['rms'],
                                'count': data[s][n]['count']} for n in JOINT_ORDER}
                        for s in segs},
        'per_joint_max': {n: overall[n]['max'] for n in JOINT_ORDER},
        'per_joint_min_limit_margin': margin,
        'min_limit_margin': min(margin.values()),
        'command_coverage': coverage_hits / coverage_total if coverage_total else 0.0,
        'command_coverage_active': active_hits / active_total if active_total else 0.0,
        'active_window': [active_lo, active_hi],
        'tail_samples_after_last_command': tail_samples,
        'state_samples_without_command': missing_hold,
    }
    for s in segs:
        vals = [v for n in JOINT_ORDER for v in [res['per_segment'][s][n]['max']]]
        res['per_segment_max_rad'] = res.get('per_segment_max_rad', {})
        res['per_segment_max_rad'][s] = max(vals) if vals else None
    return res


# -------------------------------------------------------------------- FK side
def _nearest_body(bodies, t):
    """Body row with largest stamp <= t (hold-last)."""
    if not bodies:
        return None
    lo, hi = 0, len(bodies)
    idx = -1
    while lo < hi:
        mid = (lo + hi) // 2
        if bodies[mid]['t'] <= t:
            idx = mid
            lo = mid + 1
        else:
            hi = mid
    if idx < 0:
        return None
    return bodies[idx]


def body_quat(row):
    if 'qx' in row:
        return row['qx'], row['qy'], row['qz'], row['qw']
    # fallback: rebuild from Euler (only for legacy traces)
    from math import cos, sin
    roll, pitch, yaw = row['roll'], row['pitch'], row['yaw']
    cr, sr = cos(roll / 2), sin(roll / 2)
    cp, sp = cos(pitch / 2), sin(pitch / 2)
    cy, sy = cos(yaw / 2), sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * cp * sp, cr * cp * cy + sr * cp * sy)


def foot_samples(run, bounds, hold_max):
    """Compute per-sample FK(observed) and FK(command) sole world positions."""
    out = {'observed': {'l': [], 'r': []}, 'commanded': {'l': [], 'r': []}}
    bodies = run['bodies']
    commands = run['commands']
    accepted = [c for c in commands if c.get('ok')]
    for st in run['states']:
        t = st['t']
        b = _nearest_body(bodies, t)
        if b is None:
            continue
        qx, qy, qz, qw = body_quat(b)
        R = quat_to_R(qx, qy, qz, qw)
        base = (b['x'], b['y'], b['z'])
        q_obs = {n: st['position'][i] for i, n in enumerate(JOINT_ORDER)}
        cmd = hold_last_command(accepted, t, hold_max)
        for side in ('l', 'r'):
            pb = sole_position_base(q_obs, side.upper())
            pw = [base[i] + R[i][0] * pb[0] + R[i][1] * pb[1] + R[i][2] * pb[2]
                  for i in range(3)]
            out['observed'][side].append({'t': t, 'x': pw[0], 'y': pw[1], 'z': pw[2],
                                          'base': pb, 'seg': seg_name(bounds, t)})
            if cmd is not None:
                q_cmd = {n: cmd['position'][i] for i, n in enumerate(JOINT_ORDER)}
                pbc = sole_position_base(q_cmd, side.upper())
                pwc = [base[i] + R[i][0] * pbc[0] + R[i][1] * pbc[1] + R[i][2] * pbc[2]
                       for i in range(3)]
                out['commanded'][side].append({'t': t, 'x': pwc[0], 'y': pwc[1], 'z': pwc[2],
                                               'base': pbc, 'seg': seg_name(bounds, t)})
    return out


def foot_segment_stats(samples, segs):
    res = {}
    for side in ('l', 'r'):
        res[side] = {}
        for seg in segs:
            xs = [s['z'] for s in samples[side] if s['seg'] == seg]
            if not xs:
                continue
            xw = [s['x'] for s in samples[side] if s['seg'] == seg]
            res[side][seg] = {
                'min_z': min(xs), 'max_z': max(xs), 'swing_height': max(xs) - min(xs),
                'x_range': max(xw) - min(xw),
                'x_start': xw[0], 'x_end': xw[-1], 'count': len(xs),
            }
    return res


def contact_metrics(run, ground_ref, clearance, bounds, observed, cfg):
    contacts = run['contacts']
    res = {}
    # reconstruct per-side step function over the contact event times
    per_side = {'left': [], 'right': []}
    for c in contacts:
        per_side[c['side']].append((c['t'], bool(c['value'])))
    for side in ('left', 'right'):
        per_side[side].sort()
    # freshness violations: gaps between consecutive same-side events
    freshness = float(0.05)
    violations = 0
    max_gap = 0.0
    for side in ('left', 'right'):
        ts = [t for t, _ in per_side[side]]
        for a, b in zip(ts, ts[1:]):
            max_gap = max(max_gap, b - a)
            if b - a > freshness + 1e-9:
                violations += 1
    res['freshness_violations'] = violations
    res['max_contact_gap_sec'] = max_gap
    res['left_samples'] = len(per_side['left'])
    res['right_samples'] = len(per_side['right'])
    res['left_any_true'] = any(v for _, v in per_side['left'])
    res['right_any_true'] = any(v for _, v in per_side['right'])
    res['left_any_false'] = any(not v for _, v in per_side['left'])
    res['right_any_false'] = any(not v for _, v in per_side['right'])

    # sample both feet on the union of event times -> alternation
    events = sorted(set([t for t, _ in per_side['left']] +
                        [t for t, _ in per_side['right']]))
    lv = rv = None
    li = ri = 0
    trans = {'left': 0, 'right': 0}
    alt = 0
    last_pattern = None
    prev_l = prev_r = None
    for t in events:
        while li < len(per_side['left']) and per_side['left'][li][0] <= t:
            lv = per_side['left'][li][1]
            li += 1
        while ri < len(per_side['right']) and per_side['right'][ri][0] <= t:
            rv = per_side['right'][ri][1]
            ri += 1
        if prev_l is not None and lv != prev_l:
            trans['left'] += 1
        if prev_r is not None and rv != prev_r:
            trans['right'] += 1
        pattern = (lv, rv)
        if pattern in ((True, False), (False, True)) and pattern != last_pattern:
            alt += 1
            last_pattern = pattern
        elif pattern != last_pattern:
            last_pattern = pattern
        prev_l, prev_r = lv, rv
    res['transitions'] = trans
    res['alternation_count'] = alt

    # gait-phase alignment of the Bool transitions (must not be spatially random)
    gait_start = bounds.get('gait_blend', [None])[0]
    freq = float(cfg.get('step_frequency_hz', 0.5) or 0.5)
    phase_info = {}
    if gait_start is not None:
        for side in ('left', 'right'):
            evs = per_side[side]
            phases = []
            for (ta, va), (tb, vb) in zip(evs, evs[1:]):
                if va != vb and tb >= gait_start:
                    phases.append((tb - gait_start) * freq % 1.0)
            mean, conc = _circ_mean([p * 2 * math.pi for p in phases])
            phase_info[side] = {
                'transition_count': len(phases),
                'circ_mean_phase': (mean / (2 * math.pi)) if mean is not None else None,
                'concentration': conc,
            }
    res['gait_phase_alignment'] = phase_info
    # NOTE (Phase 4C review P3 #2): no Bool-vs-supplementary agreement metric is
    # reported. The two signals measure different physical quantities (main ankle
    # collision box vs. FK sole height including the lower FSR box), so a single
    # agreement number is not informative. They are reported separately.
    return res


def _circ_mean(angles):
    if not angles:
        return None, None
    s = sum(math.sin(a) for a in angles) / len(angles)
    c = sum(math.cos(a) for a in angles) / len(angles)
    r = math.hypot(s, c)
    return math.atan2(s, c) % (2 * math.pi), r


def supplementary_metrics(observed, ground_ref, clearance, bounds, cfg):
    """Independent foot-ground signal from observed FK sole height."""
    rows_l, rows_r = observed['l'], observed['r']
    n = min(len(rows_l), len(rows_r))
    gl = [rows_l[i]['z'] <= ground_ref + clearance for i in range(n)]
    gr = [rows_r[i]['z'] <= ground_ref + clearance for i in range(n)]
    segs = [rows_l[i]['seg'] for i in range(n)]
    steady = [i for i in range(n) if segs[i] == 'steady_gait']
    alt = 0
    last = None
    for i in steady:
        pat = (gl[i], gr[i])
        if pat in ((True, False), (False, True)) and pat != last:
            alt += 1
            last = pat
        elif pat != last:
            last = pat
    grounded_l = sum(1 for i in steady if gl[i])
    grounded_r = sum(1 for i in steady if gr[i])
    # anti-phase: at each steady sample exactly one foot should be near ground
    anti = sum(1 for i in steady if gl[i] != gr[i])
    # per-side swing intervals (z above the grounded threshold)
    def intervals(g):
        out = []
        cur = None
        for i in steady:
            if not g[i]:
                if cur is None:
                    cur = [i, i]
                else:
                    cur[1] = i
            elif cur is not None:
                out.append(cur)
                cur = None
        if cur is not None:
            out.append(cur)
        return out
    il = intervals(gl)
    ir = intervals(gr)
    return {
        'alternation_count': alt,
        'left_grounded_fraction': grounded_l / len(steady) if steady else None,
        'right_grounded_fraction': grounded_r / len(steady) if steady else None,
        'anti_phase_fraction': anti / len(steady) if steady else None,
        'left_swing_interval_count': len(il),
        'right_swing_interval_count': len(ir),
        'left_any_grounded': bool(grounded_l > 0),
        'right_any_grounded': bool(grounded_r > 0),
        'left_any_swing': bool(grounded_l < len(steady)),
        'right_any_swing': bool(grounded_r < len(steady)),
        'ground_ref_z': ground_ref,
        'clearance_m': clearance,
    }


def sliding_metrics(observed, ground_ref, clearance, bounds):
    """Quantify stance-foot world sliding while the supplement says grounded."""
    res = {}
    for side in ('l', 'r'):
        rows = observed[side]
        seg_rows = [s for s in rows if s['seg'] in ('gait_blend', 'steady_gait')]
        grounded = [s for s in seg_rows if s['z'] <= ground_ref + clearance]
        res[side] = {'grounded_samples': len(grounded)}
        if len(grounded) < 2:
            continue
        xs = [s['x'] for s in grounded]
        res[side]['grounded_x_span'] = max(xs) - min(xs)
        # per grounded interval drift
        intervals = []
        cur = [grounded[0]]
        for s in grounded[1:]:
            if s['t'] - cur[-1]['t'] <= 0.03:
                cur.append(s)
            else:
                intervals.append(cur)
                cur = [s]
        intervals.append(cur)
        drifts = [abs(iv[-1]['x'] - iv[0]['x']) for iv in intervals if len(iv) >= 2]
        res[side]['n_grounded_intervals'] = len(intervals)
        res[side]['max_interval_x_drift'] = max(drifts) if drifts else 0.0
        res[side]['mean_interval_x_drift'] = (
            sum(drifts) / len(drifts) if drifts else 0.0)
    return res


def imu_metrics(run, bounds):
    segs = ['spawn', 'settling', 'gait_blend', 'steady_gait', 'stop_ramp', 'final_hold']
    rows = run['imu']
    res = {'nonfinite': 0, 'samples': len(rows)}
    per = {s: {'max_omega': 0.0, 'max_accel': 0.0} for s in segs}
    prev_mag = None
    max_spike_w = 0.0
    max_spike_a = 0.0
    steady_w = []
    steady_a = []
    for r in rows:
        vals = [r['wx'], r['wy'], r['wz'], r['ax'], r['ay'], r['az']]
        if any(not math.isfinite(v) for v in vals):
            res['nonfinite'] += 1
            continue
        w = math.sqrt(r['wx'] ** 2 + r['wy'] ** 2 + r['wz'] ** 2)
        a = math.sqrt(r['ax'] ** 2 + r['ay'] ** 2 + r['az'] ** 2)
        s = seg_name(bounds, r['t'])
        if s in per:
            per[s]['max_omega'] = max(per[s]['max_omega'], w)
            per[s]['max_accel'] = max(per[s]['max_accel'], a)
        if s == 'steady_gait':
            steady_w.append(w)
            steady_a.append(a)
        if prev_mag is not None:
            max_spike_w = max(max_spike_w, abs(w - prev_mag[0]))
            max_spike_a = max(max_spike_a, abs(a - prev_mag[1]))
        prev_mag = (w, a)
    res['per_segment'] = per
    res['max_omega'] = max((v['max_omega'] for v in per.values()), default=0.0)
    res['max_accel'] = max((v['max_accel'] for v in per.values()), default=0.0)
    res['max_spike_omega'] = max_spike_w
    res['max_spike_accel'] = max_spike_a
    if steady_w:
        res['steady_omega_mean'] = sum(steady_w) / len(steady_w)
        res['steady_omega_max'] = max(steady_w)
        res['steady_accel_mean'] = sum(steady_a) / len(steady_a)
        res['steady_accel_max'] = max(steady_a)
        res['steady_accel_min'] = min(steady_a)
    # divergence: compare rolling mean of acceleration in first vs last steady 20%
    if len(steady_a) >= 10:
        k = max(1, len(steady_a) // 5)
        res['steady_accel_first20_mean'] = sum(steady_a[:k]) / k
        res['steady_accel_last20_mean'] = sum(steady_a[-k:]) / k
    return res


def body_metrics(run, bounds):
    rows = run['bodies']
    res = {'samples': len(rows)}
    if not rows:
        return res
    t0, t1 = rows[0]['t'], rows[-1]['t']
    res['t_first'] = t0
    res['t_last'] = t1

    def window(lo, hi):
        return [b for b in rows if lo <= b['t'] < hi]

    spawn_w = window(*bounds['spawn']) if bounds.get('spawn', (None, None))[0] is not None else []
    settled_w = window(*bounds['final_hold']) if 'final_hold' in bounds else []
    steady_w = window(*bounds['steady_gait']) if 'steady_gait' in bounds else []
    res['spawn_z_max'] = max((b['z'] for b in spawn_w), default=None)
    res['spawn_z_min'] = min((b['z'] for b in spawn_w), default=None)
    res['absolute_min_z'] = min(b['z'] for b in rows)
    res['absolute_max_z'] = max(b['z'] for b in rows)
    if steady_w:
        res['steady_min_z'] = min(b['z'] for b in steady_w)
        res['steady_max_z'] = max(b['z'] for b in steady_w)
    if settled_w:
        res['settled_z'] = sum(b['z'] for b in settled_w) / len(settled_w)
        res['settled_final_roll_deg'] = math.degrees(
            sum(abs(b['roll']) for b in settled_w) / len(settled_w))
    # displacement windows
    res['displacement'] = {}
    res['displacement']['spawn_to_final'] = _disp(rows[0], rows[-1])
    for name in ('steady_gait', 'stop_ramp', 'final_hold'):
        w = window(*bounds[name]) if bounds.get(name) else []
        if len(w) >= 2:
            res['displacement'][name] = _disp(w[0], w[-1])
    if steady_w:
        res['displacement']['steady_to_final'] = _disp(steady_w[0], rows[-1])
    # per gait cycle displacement (forward x only), for walk runs
    res['per_cycle_forward_m'] = []
    res['per_cycle_notes'] = None
    # attitude per segment
    res['roll_pitch_deg'] = {}
    for name, (lo, hi) in bounds.items():
        w = window(lo, hi)
        if w:
            res['roll_pitch_deg'][name] = {
                'max_roll': math.degrees(max(abs(b['roll']) for b in w)),
                'max_pitch': math.degrees(max(abs(b['pitch']) for b in w)),
                'max_yaw_drift': math.degrees(max(abs(b['yaw']) for b in w)),
            }
    return res


def per_cycle_metrics(run, cfg, bounds):
    rows = run['bodies']
    freq = float(cfg.get('step_frequency_hz', 0.5))
    d = float(cfg.get('duration_sec', 22.0))
    if 'gait_blend' not in bounds or not rows:
        return []
    gait_start = bounds['gait_blend'][0]
    n = int(math.floor(d * freq))
    cycles = []
    for k in range(n):
        ta = gait_start + k / freq
        tb = gait_start + (k + 1) / freq
        wa = [b for b in rows if b['t'] >= ta]
        wb = [b for b in rows if b['t'] >= tb]
        if not wa or not wb:
            break
        cycles.append({
            'cycle': k,
            't_start': ta,
            't_end': tb,
            'forward_m': wb[0]['x'] - wa[0]['x'],
            'lateral_m': wb[0]['y'] - wa[0]['y'],
        })
    return cycles


def _disp(a, b):
    return {
        'dx': b['x'] - a['x'], 'dy': b['y'] - a['y'], 'dz': b['z'] - a['z'],
        'planar': math.hypot(b['x'] - a['x'], b['y'] - a['y']),
    }


# -------------------------------------------------------------------- analyze
def analyze(run_dir, config=None):
    run = load_run(run_dir)
    cfg = config or load_config(run_dir)
    bounds, notes = segment_bounds(cfg, run)
    hold_max = float(cfg.get('command_hold_max', 0.05))
    accepted = [c for c in run['commands'] if c.get('ok')]
    cm = command_metrics(run, cfg)
    result = {
        'run_dir': run_dir,
        'config': cfg,
        'state_count': len(run['states']),
        'body_count': len(run['bodies']),
        'imu_count': len(run['imu']),
        'segment_bounds': {k: list(v) for k, v in bounds.items()},
        'segment_notes': notes,
        'command': cm[0],
        'tracking': tracking_metrics(run, accepted, bounds, hold_max),
        'imu': imu_metrics(run, bounds),
        'body': body_metrics(run, bounds),
        'per_cycle': per_cycle_metrics(run, cfg, bounds),
    }
    if not run['states']:
        return result

    obs = foot_samples(run, bounds, hold_max)
    # SDF-anchored stance reference (Phase 4C review P2 #1), NOT derived from the
    # measured run.
    ground_ref = GROUND_REF_SOLE_Z_M
    clearance = float(cfg.get('supplementary_clearance_m', 0.004))
    result['foot_fk'] = {
        'ground_ref_z': ground_ref,
        'supplementary_clearance_m': clearance,
        'observed': foot_segment_stats(obs['observed'],
                                       ['gait_blend', 'steady_gait', 'stop_ramp']),
        'commanded': foot_segment_stats(obs['commanded'],
                                        ['gait_blend', 'steady_gait', 'stop_ramp']),
    }
    result['contact'] = contact_metrics(
        run, ground_ref, clearance, bounds, obs['observed'], cfg)
    result['supplementary'] = supplementary_metrics(
        obs['observed'], ground_ref, clearance, bounds, cfg)
    result['sliding'] = sliding_metrics(obs['observed'], ground_ref, clearance, bounds)
    return result


def evaluate_run(result, thresholds):
    cfg = result['config']
    mode = cfg.get('gait_method', 'walk')
    c = result['command']
    t = result['tracking']
    checks = {}

    # ---- command/state response
    checks['commands_present'] = c.get('accepted', 0) > 0
    checks['no_rejections'] = not c.get('reject_reasons')
    checks['stamps_increasing'] = bool(c.get('strictly_increasing'))
    checks['dim_ok'] = bool(c.get('dim_ok'))
    checks['in_limit'] = bool(c.get('in_limit'))
    checks['finite'] = bool(c.get('position_finite'))
    checks['rate_ok'] = (c.get('rate_hz') or 0.0) >= thresholds['min_rate_hz']
    checks['coverage_ok'] = (
        (t.get('command_coverage_active') or 0.0) >= thresholds['min_command_coverage'])
    checks['has_states'] = result.get('state_count', 0) > 0
    checks['segmented_tracking_available'] = all(
        result['tracking']['per_segment_max_rad'].get(s) is not None
        for s in ('settling', 'steady_gait'))
    response_pass = all(checks.values())

    foot = result.get('foot_fk')
    contact = result.get('contact', {})
    supp = result.get('supplementary', {})
    checks_foot = {}
    if foot:
        lo = foot['observed'].get('l', {}).get('steady_gait', {})
        ro = foot['observed'].get('r', {}).get('steady_gait', {})
        lc = foot['commanded'].get('l', {}).get('steady_gait', {})
        rc = foot['commanded'].get('r', {}).get('steady_gait', {})
        obs_swing = max(lo.get('swing_height', 0.0), ro.get('swing_height', 0.0))
        cmd_swing = max(lc.get('swing_height', 0.0), rc.get('swing_height', 0.0))
        checks_foot['observed_swing_ok'] = obs_swing >= thresholds['min_foot_swing_m']
        checks_foot['observed_min_z_ok'] = (
            min(lo.get('min_z', 1.0), ro.get('min_z', 1.0)) >= thresholds['min_foot_z'])
        checks_foot['commanded_fk_present'] = bool(lc) and bool(rc)
        checks_foot['obs_vs_cmd_swing_consistent'] = (
            abs(obs_swing - cmd_swing) <= thresholds['max_swing_mismatch_m'])
    checks_foot['bool_alternation_ok'] = (
        contact.get('alternation_count', 0) >= thresholds['min_alternation'])
    checks_foot['bool_both_feet_active'] = (
        contact.get('left_any_true') and contact.get('right_any_true')
        and contact.get('left_any_false') and contact.get('right_any_false'))
    checks_foot['contact_fresh'] = contact.get('freshness_violations', 0) == 0
    checks_foot['bool_phase_aligned'] = all(
        (contact.get('gait_phase_alignment', {}).get(s, {}).get('concentration') or 0.0)
        >= thresholds['min_phase_concentration'] for s in ('left', 'right'))
    checks_foot['supplementary_alternation_ok'] = (
        supp.get('alternation_count', 0) >= thresholds['min_alternation'])
    checks_foot['supplementary_antiphase_ok'] = (
        (supp.get('anti_phase_fraction') or 0.0) >= thresholds['min_antiphase_fraction'])
    checks_foot['supplementary_swing_ok'] = bool(
        supp.get('left_any_swing') and supp.get('right_any_swing'))
    foot_pass = all(checks_foot.values())

    b = result['body']
    checks_body = {}
    checks_body['height_not_collapsed'] = (
        b.get('absolute_min_z', 0.0) >= thresholds['min_abs_z'])
    if mode == 'walk':
        fwd = b.get('displacement', {}).get('spawn_to_final', {}).get('dx', 0.0)
        checks_body['forward_displacement_ok'] = fwd >= thresholds['min_forward_m']
        checks_body['lateral_small'] = abs(
            b.get('displacement', {}).get('spawn_to_final', {}).get('dy', 0.0)
        ) <= thresholds['max_lateral_m']
    rp = b.get('roll_pitch_deg', {}).get('steady_gait', {})
    checks_body['attitude_ok'] = (
        max(rp.get('max_roll', 999), rp.get('max_pitch', 999))
        <= thresholds['max_roll_pitch_deg'])
    imu = result['imu']
    checks_body['imu_nonfinite_zero'] = imu.get('nonfinite', 1) == 0
    checks_body['imu_bounded'] = (
        imu.get('max_omega', 1e9) <= thresholds['max_imu_omega']
        and imu.get('max_accel', 1e9) <= thresholds['max_imu_accel'])
    if mode == 'walk':
        sliding = result.get('sliding', {})
        max_drift = max((sliding.get(s, {}).get('max_interval_x_drift', 0.0)
                         for s in ('l', 'r')), default=0.0)
        checks_body['no_excessive_ground_slide'] = max_drift <= thresholds['max_stance_slide_m']
    body_pass = all(checks_body.values())

    return {
        'response_pass': response_pass,
        'response_checks': checks,
        'foot_pass': foot_pass,
        'foot_checks': checks_foot,
        'body_pass': body_pass,
        'body_checks': checks_body,
    }


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    a = sub.add_parser('analyze')
    a.add_argument('run_dir')
    a.add_argument('--config', default=None)
    a.add_argument('--out', default=None)
    a.add_argument('--thresholds', default=None)
    g = sub.add_parser('aggregate')
    g.add_argument('--baseline', required=True)
    g.add_argument('--runs', nargs=3, required=True)
    g.add_argument('--pause', default=None)
    g.add_argument('--out', required=True)
    g.add_argument('--thresholds', default=None)
    g.add_argument('--runs-config', default=None)
    g.add_argument('--baseline-config', default=None)
    g.add_argument('--pause-config', default=None)
    args = ap.parse_args()

    if args.cmd == 'analyze':
        cfg = json.load(open(args.config)) if args.config else None
        res = analyze(args.run_dir, cfg)
        thresholds = _load_thresholds(args.thresholds)
        res['evaluation'] = evaluate_run(res, thresholds)
        text = json.dumps(res, indent=2, sort_keys=True)
        if args.out:
            with open(args.out, 'w') as fh:
                fh.write(text)
        print(text)
        return

    thresholds = _load_thresholds(args.thresholds)
    base_cfg = json.load(open(args.baseline_config)) if args.baseline_config else None
    runs_cfg = json.load(open(args.runs_config)) if args.runs_config else None
    baseline = analyze(args.baseline, base_cfg)
    runs = [analyze(d, runs_cfg) for d in args.runs]
    summary = {
        'thresholds': thresholds,
        'baseline': {'summary': baseline['body'], 'command': baseline['command']},
        'runs': [],
    }
    for r in runs:
        r['evaluation'] = evaluate_run(r, thresholds)
        summary['runs'].append({
            'run_dir': r['run_dir'],
            'command': r['command'],
            'tracking': {
                'per_segment_max_rad': r['tracking']['per_segment_max_rad'],
                'command_coverage': r['tracking']['command_coverage'],
                'command_coverage_active': r['tracking']['command_coverage_active'],
                'tail_samples_after_last_command':
                    r['tracking']['tail_samples_after_last_command'],
                'min_limit_margin': r['tracking']['min_limit_margin'],
                'per_joint_min_limit_margin': r['tracking']['per_joint_min_limit_margin'],
                'per_joint_max': r['tracking']['per_joint_max'],
                'state_samples': r['state_count'],
            },
            'foot_fk': r['foot_fk'],
            'contact': r['contact'],
            'supplementary': r['supplementary'],
            'sliding': r['sliding'],
            'body': r['body'],
            'imu': r['imu'],
            'per_cycle': r['per_cycle'],
            'evaluation': r['evaluation'],
        })
    summary['repeatability'] = repeatability(runs, thresholds)
    if args.pause:
        pause_cfg = json.load(open(args.pause_config)) if args.pause_config else None
        pause = analyze(args.pause, pause_cfg)
        pause['evaluation'] = evaluate_run(pause, thresholds)
        summary['pause'] = _pause_metrics(pause)

    # ---- top-level gates (all evidence dirs required)
    required = ('command_trace.jsonl', 'state_trace.jsonl', 'body_pose_trace.jsonl',
                'contact_trace.jsonl', 'imu_trace.jsonl', 'clock_trace.jsonl',
                'run_config.json')
    evidence_ok = all(
        all(os.path.exists(os.path.join(d, f)) for f in required)
        for d in [args.baseline] + list(args.runs) + ([args.pause] if args.pause else []))
    baseline_body = baseline['body']
    baseline_ok = (baseline['command'].get('accepted', 0) > 0
                   and baseline_body.get('absolute_min_z', 0.0) >= 0.25
                   and abs(baseline_body.get('displacement', {})
                           .get('spawn_to_final', {}).get('planar', 1.0)) < 0.02)
    gates = {
        'command_state_response': all(r['evaluation']['response_pass'] for r in runs),
        'foot_contact_behavior': all(r['evaluation']['foot_pass'] for r in runs),
        'body_motion_stability': all(r['evaluation']['body_pass'] for r in runs),
        'repeatability': summary['repeatability']['pass'],
        'pause_resume_semantics': bool(summary.get('pause', {}).get('pass')),
        'baseline_valid': baseline_ok,
        'evidence_complete': evidence_ok,
    }
    summary['gates'] = gates
    summary['phase4c_status'] = 'PASS' if all(gates.values()) else 'BLOCKED'
    with open(args.out, 'w') as fh:
        json.dump(summary, fh, indent=2, sort_keys=True)
    print(json.dumps({'gates': gates, 'phase4c_status': summary['phase4c_status'],
                      'repeatability': summary['repeatability'],
                      'pause': summary.get('pause', {}).get('checks')},
                     indent=2, sort_keys=True))


def repeatability(runs, thresholds):
    fwds = []
    for r in runs:
        fwds.append(r['body'].get('displacement', {}).get('spawn_to_final', {}).get('dx'))
    valid = [f for f in fwds if f is not None]
    res = {'forward_m': fwds}
    if valid:
        mean = sum(valid) / len(valid)
        spread = max(valid) - min(valid)
        res['forward_mean'] = mean
        res['forward_range'] = spread
        res['forward_cv'] = (spread / mean) if mean else None
        res['all_forward_positive'] = all(f > 0 for f in valid)
        res['all_forward_above_min'] = all(f >= thresholds['min_forward_m'] for f in valid)
    res['alternation'] = [r['contact'].get('alternation_count') for r in runs]
    res['all_response_pass'] = all(r['evaluation']['response_pass'] for r in runs)
    res['all_foot_pass'] = all(r['evaluation']['foot_pass'] for r in runs)
    res['all_body_pass'] = all(r['evaluation']['body_pass'] for r in runs)
    res['pass'] = bool(res.get('all_forward_above_min')
                       and res['all_response_pass'] and res['all_foot_pass']
                       and res['all_body_pass'])
    return res


def _pause_metrics(pause):
    cmd = pause['command']
    checks = {
        'stamps_strictly_increasing': bool(cmd.get('strictly_increasing')),
        'no_rejections': not cmd.get('reject_reasons'),
        'no_sim_gap_gt_hold': (cmd.get('max_stamp_gap_sec') or 0.0)
        <= float(pause['config'].get('command_hold_max', 0.05)) + 1e-6,
        'body_stable': (pause['body'].get('absolute_min_z', 0.0) >= 0.25),
    }
    return {
        'stamps_strictly_increasing': cmd.get('strictly_increasing'),
        'rejections': cmd.get('reject_reasons'),
        'accepted': cmd.get('accepted'),
        'rate_hz': cmd.get('rate_hz'),
        'max_command_gap_sec': cmd.get('max_stamp_gap_sec'),
        'body_forward_m': pause['body'].get('displacement', {}).get('spawn_to_final', {}).get('dx'),
        'absolute_min_z': pause['body'].get('absolute_min_z'),
        'checks': checks,
        'pass': all(checks.values()),
        'evaluation': pause['evaluation'],
    }


def _max_gap(times):
    return max((b - a for a, b in zip(times, times[1:])), default=0.0)


def _load_thresholds(path):
    default = {
        'min_rate_hz': 50.0,
        'min_command_coverage': 0.95,
        'min_foot_swing_m': 0.002,
        'min_foot_z': -0.005,
        'max_swing_mismatch_m': 0.006,
        'min_alternation': 4,
        'min_phase_concentration': 0.30,
        'min_antiphase_fraction': 0.30,
        'min_abs_z': 0.25,
        'min_forward_m': 0.08,
        'max_lateral_m': 0.03,
        'max_roll_pitch_deg': 35.0,
        'max_imu_omega': 25.0,
        'max_imu_accel': 120.0,
        'max_stance_slide_m': 0.03,
    }
    if path:
        default.update(json.load(open(path)))
    return default


if __name__ == '__main__':
    sys.exit(main())
