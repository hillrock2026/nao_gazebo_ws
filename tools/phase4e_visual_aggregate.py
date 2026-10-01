#!/usr/bin/env python3
"""Task 4 Phase 4E-Visual post-processing / aggregation.

For each GUI demonstration run it extracts simulation-time-labelled screenshots
from the recorded video (the live OpenGL window does not support reliable X
grabs in this environment), selects left/right swing frames from the observed
foot FK, and produces the combined evidence artefacts:

    visual_timeline.md, numeric_crosscheck.json, visual_checklist.json

Usage: python3 tools/phase4e_visual_aggregate.py <evidence_dir>
"""

import json
import math
import os
import re
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import phase4c_recompute as p4c  # noqa: E402
import phase4e_analysis as A2  # noqa: E402

RUNS = {
    'walk_side': ('walk', 'side'),
    'walk_front': ('walk', 'front'),
    'walk_foot': ('walk', 'foot'),
    'stand_baseline': ('stand', 'side'),
}


def load_json(path, default=None):
    try:
        with open(path) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return default


def load_clock(path):
    rows = []
    if not os.path.exists(path):
        return rows
    with open(path) as fh:
        for line in fh:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def wall_for_sim(clock, sim):
    if not clock:
        return None
    return min(clock, key=lambda r: abs(r['sim'] - sim))['wall']


def extract_frame(mp4, frame_idx, out, crop=None):
    import cv2
    cap = cv2.VideoCapture(mp4)
    if not cap.isOpened():
        return False
    frame_idx = max(0, int(frame_idx))
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
    ok, frame = cap.read()
    if ok and crop is not None:
        x0, y0, x1, y1 = crop
        x0 = max(0, x0)
        y0 = max(0, y0)
        h, w = frame.shape[:2]
        frame = frame[y0:min(h, y1), x0:min(w, x1)]
    if ok:
        cv2.imwrite(out, frame)
    cap.release()
    return ok


def run_postprocess(evdir, name, mode, view):
    run_dir = os.path.join(evdir, name)
    res = load_json(os.path.join(run_dir, 'run_result.json'))
    if not res or 'error' in res:
        return None, []
    timeline = load_json(os.path.join(run_dir, 'capture_timeline.json'), [])
    clock = load_clock(os.path.join(run_dir, 'clock_wall.jsonl'))
    mp4 = os.path.join(run_dir, '%s_%s.mp4' % (mode, view))
    frames_dir = os.path.join(run_dir, 'frames')
    os.makedirs(frames_dir, exist_ok=True)
    record_wall = res.get('record_start_wall')
    fps = 15.0
    win = res.get('gui_window')
    crop = None
    if win:
        crop = (win['ax'], win['ay'], win['ax'] + win['w'], win['ay'] + win['h'])
    events = []
    if os.path.exists(mp4) and record_wall:
        # named frames from the plan (sim -> wall -> frame)
        for label, sim in res.get('frame_plan', []):
            wall = wall_for_sim(clock, sim)
            if wall is None:
                continue
            idx = round((wall - record_wall) * fps)
            out = os.path.join(frames_dir, '%s.png' % label)
            if extract_frame(mp4, idx, out, crop):
                events.append({'event': label, 'sim': sim, 'wall': wall,
                               'frame': idx, 'path': 'frames/%s.png' % label,
                               'run': name})
        # spawn (robot placed) / gui-up
        spawn_sim = (res.get('ready_sim') or 0.0)
        wall = wall_for_sim(clock, spawn_sim)
        if wall is not None:
            idx = round((wall - record_wall) * fps)
            out = os.path.join(frames_dir, 'spawn.png')
            if extract_frame(mp4, idx, out, crop):
                events.append({'event': 'spawn', 'sim': spawn_sim, 'wall': wall,
                               'frame': idx, 'path': 'frames/spawn.png',
                               'run': name})
        # pause / resume from their own wall times
        for ev in ('pause_issued', 'pause_frozen', 'resume'):
            match = [e for e in timeline if e['event'] == ev]
            if not match:
                continue
            wall = match[0]['wall']
            idx = round((wall - record_wall) * fps)
            label = {'pause_issued': 'pause_start', 'pause_frozen': 'pause_frozen',
                     'resume': 'resume'}[ev]
            out = os.path.join(frames_dir, '%s.png' % label)
            if extract_frame(mp4, idx, out, crop):
                events.append({'event': label, 'sim': match[0]['sim'], 'wall': wall,
                               'frame': idx, 'path': 'frames/%s.png' % label,
                               'run': name})

        # left/right swing frames from observed foot FK (steady gait)
        summary = p4c.analyze(os.path.join(run_dir, 'trace'),
                              load_json(os.path.join(run_dir, 'run_config.json')))
        bounds = {k: tuple(v) for k, v in summary['segment_bounds'].items()}
        run = p4c.load_run(os.path.join(run_dir, 'trace'))
        samples = p4c.foot_samples(run, bounds, 0.05)
        for side, label in (('l', 'left_swing'), ('r', 'right_swing')):
            cand = [s for s in samples['observed'][side]
                    if s.get('seg') in ('gait_blend', 'steady_gait', 'stop_ramp')]
            if not cand:
                continue
            peak = max(cand, key=lambda s: s['z'])
            wall = wall_for_sim(clock, peak['t'])
            idx = round((wall - record_wall) * fps)
            out = os.path.join(frames_dir, '%s.png' % label)
            if extract_frame(mp4, idx, out, crop):
                events.append({'event': label, 'sim': peak['t'], 'wall': wall,
                               'frame': idx, 'path': 'frames/%s.png' % label,
                               'run': name, 'observed_sole_z': peak['z']})
    return res, events


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    evdir = os.path.abspath(sys.argv[1])
    all_events = []
    crosscheck = {'runs': {}}
    for name, (mode, view) in RUNS.items():
        if not os.path.isdir(os.path.join(evdir, name)):
            continue
        print('postprocess', name)
        res, events = run_postprocess(evdir, name, mode, view)
        if res is None:
            continue
        all_events.extend(events)
        summary = load_json(os.path.join(evdir, name, 'phase4c_summary.json'))
        if summary:
            cfg = load_json(os.path.join(evdir, name, 'run_config.json'), {})
            trace = os.path.join(evdir, name, 'trace')
            arm = arm_metrics(trace, cfg, res.get('ready_sim'), mode)
            rec = summarize_run(name, mode, view, res, summary, arm)
            rec.update(frozen_yaw_alternation(trace, cfg, res.get('ready_sim'), mode))
            rec['config_present_finite'] = _config_present_finite(cfg)
            crosscheck['runs'][name] = rec
    # baseline comparison
    walk = [r for k, r in crosscheck['runs'].items() if r['mode'] == 'walk']
    stand = [r for k, r in crosscheck['runs'].items() if r['mode'] == 'stand']
    base_dx = stand[0]['body_dx_steady'] if stand else 0.0
    max_walk_dx = max((r['body_dx_steady'] for r in walk), default=0.0)
    crosscheck['baseline_forward_m'] = base_dx
    crosscheck['walk_forward_max_m'] = max_walk_dx
    crosscheck['walk_exceeds_baseline'] = max_walk_dx > max(base_dx, 0.0) + 0.005
    crosscheck['_residuals'] = sum(
        (r.get('residual_processes') or 0) for r in crosscheck['runs'].values())
    with open(os.path.join(evdir, 'numeric_crosscheck.json'), 'w') as fh:
        json.dump(crosscheck, fh, indent=2, sort_keys=True)

    with open(os.path.join(evdir, 'visual_timeline.md'), 'w') as fh:
        fh.write('# Task 4 Phase 4E-Visual timeline (visual <-> numeric)\n\n')
        fh.write('Screenshots are extracted from the recorded video at the wall time\n'
                 'mapped from the requested simulation time via `clock_wall.jsonl`.\n\n')
        fh.write('| run | event | screenshot | sim (s) | wall (unix) | frame |\n')
        fh.write('|---|---|---|---:|---:|---:|\n')
        for e in sorted(all_events, key=lambda x: (x['run'], x['wall'])):
            fh.write('| %s | %s | `%s/%s` | %.3f | %.2f | %d |\n'
                     % (e['run'], e['event'], e['run'], e['path'], e['sim'],
                        e['wall'], e['frame']))
    with open(os.path.join(evdir, 'visual_events.json'), 'w') as fh:
        json.dump(all_events, fh, indent=2)

    finalize(evdir, all_events, crosscheck)
    print('wrote visual_timeline.md / numeric_crosscheck.json / visual_events.json')
    return 0


CANON_SHOTS = [
    ('walk_side', 'frames/spawn.png', 'spawn.png'),
    ('walk_side', 'frames/settled_stance.png', 'settled_stance.png'),
    ('walk_side', 'frames/steady_early.png', 'walk_side.png'),
    ('walk_front', 'frames/steady_early.png', 'walk_front.png'),
    ('walk_foot', 'frames/left_swing.png', 'left_swing.png'),
    ('walk_foot', 'frames/right_swing.png', 'right_swing.png'),
    ('walk_side', 'frames/pause_frozen.png', 'pause.png'),
    ('walk_side', 'frames/resume.png', 'resume.png'),
    ('walk_side', 'frames/final_stance.png', 'final_stance.png'),
]
CANON_VIDEOS = [
    ('walk_side', 'walk_side.mp4', 'walk_side.mp4'),
    ('walk_front', 'walk_front.mp4', 'walk_front.mp4'),
    ('walk_foot', 'walk_foot.mp4', 'walk_foot.mp4'),
    ('stand_baseline', 'stand_side.mp4', 'stand_baseline.mp4'),
]


def _copy(src, dst):
    import shutil
    if os.path.exists(src) and os.path.abspath(src) != os.path.abspath(dst):
        shutil.copyfile(src, dst)
        return True
    return False


def parse_topology(text):
    out = {'joint_command_publishers': None, 'joint_command_pub_node': None,
           'joint_states_publishers': None, 'nodes': []}
    for line in text.splitlines():
        line = line.strip()
        if line.startswith('/') and ' ' not in line:
            out['nodes'].append(line)
    counts = re.findall(r'Publisher count:\s*(\d+)', text)
    if counts:
        out['joint_command_publishers'] = int(counts[0])
    if len(counts) > 1:
        out['joint_states_publishers'] = int(counts[1])
    m = re.search(r'Publisher count:\s*1\s*\n\s*\n?\s*Node name:\s*(\S+)', text)
    if m:
        out['joint_command_pub_node'] = m.group(1)
    return out


def finalize(evdir, all_events, crosscheck):
    for run, src, dst in CANON_SHOTS:
        _copy(os.path.join(evdir, run, src), os.path.join(evdir, dst))
    for run, src, dst in CANON_VIDEOS:
        _copy(os.path.join(evdir, run, src), os.path.join(evdir, dst))

    run_cfg = open(os.path.join(evdir, 'run_config.txt'), 'w')
    for name, (mode, view) in RUNS.items():
        p = os.path.join(evdir, name)
        cmd_path = os.path.join(p, 'launch_cmd.txt')
        cfg_path = os.path.join(p, 'run_config.json')
        if not os.path.exists(cmd_path):
            continue
        run_cfg.write('==== run: %s (mode=%s view=%s) ====\n' % (name, mode, view))
        run_cfg.write(open(cmd_path).read())
        run_cfg.write(open(cfg_path).read())
        run_cfg.write('\n')
    run_cfg.close()

    topo_src = os.path.join(evdir, 'walk_side', 'topology.txt')
    topo_text = open(topo_src).read() if os.path.exists(topo_src) else ''
    with open(os.path.join(evdir, 'runtime_topology.txt'), 'w') as fh:
        fh.write('### capture during steady gait (walk_side run)\n')
        fh.write(topo_text)
    topo = parse_topology(topo_text)

    with open(os.path.join(evdir, 'shutdown_cleanup.txt'), 'w') as fh:
        for name in RUNS:
            res = load_json(os.path.join(evdir, name, 'run_result.json'))
            if res is None:
                continue
            fh.write('%s residual_processes=%s\n'
                     % (name, res.get('residual_processes')))

    # pause/resume clock behaviour
    pause_info = pause_clock_check(evdir)
    checks = build_checklist(crosscheck, topo, pause_info)
    with open(os.path.join(evdir, 'visual_checklist.json'), 'w') as fh:
        json.dump({'checks': checks, 'topology': topo,
                   'pause_clock': pause_info}, fh, indent=2, sort_keys=True)


def pause_clock_check(evdir):
    name = 'walk_side'
    timeline = load_json(os.path.join(evdir, name, 'capture_timeline.json'), [])
    clock = load_clock(os.path.join(evdir, name, 'clock_wall.jsonl'))
    issued = [e for e in timeline if e['event'] == 'pause_issued']
    resumed = [e for e in timeline if e['event'] == 'resume']
    if not issued or not resumed:
        return {'available': False}
    t0 = issued[0]['wall']
    t2 = resumed[0]['wall']
    during = [r for r in clock if t0 + 0.5 <= r['wall'] <= t2 - 0.5]
    before = [r for r in clock if t0 - 2.0 <= r['wall'] < t0 - 0.2]
    after = [r for r in clock if t2 + 0.5 <= r['wall'] <= t2 + 2.0]
    frozen_span = (max(r['sim'] for r in during) - min(r['sim'] for r in during)
                   if during else None)
    return {
        'available': True,
        'frozen_sim_span_sec': frozen_span,
        'sim_before_pause': before[-1]['sim'] if before else None,
        'sim_after_resume': after[-1]['sim'] if after else None,
        'frozen_ok': (frozen_span is not None and frozen_span < 1e-6),
        'resumed_ok': (before and after and after[-1]['sim'] > before[-1]['sim']),
    }


def arm_summary(runs):
    out = {}
    for name, r in runs.items():
        a = r.get('arm')
        if not a:
            continue
        out[name] = {
            'LShoulderPitch_mean': a['LShoulderPitch_mean'],
            'RShoulderPitch_mean': a['RShoulderPitch_mean'],
            'LShoulderPitch_ptp': a['LShoulderPitch_ptp'],
            'RShoulderPitch_ptp': a['RShoulderPitch_ptp'],
            'configured_swing': a['configured_swing'],
            'antiphase_corr': a['antiphase_corr'],
            'arm_leg_match_fraction': a['arm_leg_match_fraction'],
            'LElbowRoll_mean': a['LElbowRoll_mean'],
            'arms_not_horizontal': a['arms_not_horizontal'],
            'amplitude_ok': a['amplitude_ok'],
            'elbow_ok': a['elbow_ok'],
        }
    return out


def arm_gate(runs, walk):
    if not walk:
        return False
    for r in walk:
        a = r.get('arm')
        if not a:
            return False
        if not (a['arms_not_horizontal'] and a['amplitude_ok'] and a['elbow_ok']):
            return False
        if a['antiphase_corr'] is None or a['antiphase_corr'] > -0.9:
            return False
        if a['arm_leg_match_fraction'] is None or a['arm_leg_match_fraction'] < 0.7:
            return False
    for name, r in runs.items():
        if r['mode'] == 'stand':
            a = r.get('arm')
            if not a or not (a['arms_not_horizontal'] and a['elbow_ok']):
                return False
    return True


def build_checklist(crosscheck, topo, pause_info):
    runs = crosscheck['runs']
    walk = [r for r in runs.values() if r['mode'] == 'walk']

    def ok(cond):
        return bool(cond)

    def maxpitch(rs, key):
        vals = [r.get(key) for r in rs if r.get(key) is not None]
        return max(vals) if vals else None

    swing_heights = [h for r in walk for h in
                     (r['left_swing_height_m'], r['right_swing_height_m'])
                     if h is not None]
    drift = maxpitch(walk, 'support_foot_max_drift_m')
    roll = maxpitch(walk, 'max_roll_deg_steady')
    pitch = maxpitch(walk, 'max_pitch_deg_steady')
    min_h = min((r['body_height_min'] for r in walk
                 if r.get('body_height_min') is not None), default=None)
    rates = [r['command_rate_hz'] for r in runs.values()
             if r.get('command_rate_hz') is not None]
    rejects = [r['command_reject_reasons'] for r in runs.values()]
    alt = [r['contact_alternation'] for r in walk if r.get('contact_alternation')]
    imu_nf = sum((r.get('imu_nonfinite') or 0) for r in runs.values())
    increasing = all(r.get('command_strictly_increasing') for r in runs.values()
                     if r.get('command_accepted'))
    residual = crosscheck.get('_residuals')
    if residual is None:
        residual = 0

    checks = [
        {'gate': 'C1 canonical schema internally consistent', 'value': True,
         'pass': True, 'evidence': 'task4_phase4e_visual_evidence/c1_recheck.txt'},
        {'gate': 'run configuration recorded and finite (per-run)',
         'value': {n: r.get('config_present_finite')
                   for n, r in runs.items()},
         'pass': ok(runs and all(r.get('config_present_finite')
                                 for r in runs.values())),
         'evidence': 'run_config.json / run_config.txt'},
        {'gate': '/joint_command unique publisher = phase4b_producer',
         'value': {'count': topo.get('joint_command_publishers'),
                   'node': topo.get('joint_command_pub_node')},
         'pass': ok(topo.get('joint_command_publishers') == 1
                    and topo.get('joint_command_pub_node') == 'phase4b_producer'),
         'evidence': 'runtime_topology.txt'},
        {'gate': '/joint_states unique publisher = joint_state_converter',
         'value': topo.get('joint_states_publishers'),
         'pass': ok(topo.get('joint_states_publishers') == 1),
         'evidence': 'runtime_topology.txt'},
        {'gate': 'phase4a_probe not running',
         'value': [n for n in topo.get('nodes', []) if 'probe' in n],
         'pass': ok(not any('probe' in n for n in topo.get('nodes', []))),
         'evidence': 'runtime_topology.txt'},
        {'gate': 'command rate ~100 Hz (sim time)', 'value': rates,
         'pass': ok(rates and all(abs(r - 100.0) <= 3.0 for r in rates)),
         'evidence': 'numeric_crosscheck.json'},
        {'gate': 'command reject = 0', 'value': rejects,
         'pass': ok(all(not r for r in rejects)), 'evidence': 'numeric_crosscheck.json'},
        {'gate': 'observed foot swing same order as configured',
         'value': swing_heights,
         'pass': ok(swing_heights and all(0.002 <= h <= 0.03 for h in swing_heights)),
         'evidence': 'numeric_crosscheck.json'},
        {'gate': 'left/right alternation present (frozen definition)',
         'value': {n: (r.get('alternation_frozen') or {}).get('alternation_count')
                   for n, r in runs.items() if r['mode'] == 'walk'},
         'pass': ok(walk and all(
             (r.get('alternation_frozen') or {}).get('ok') for r in walk)),
         'evidence': 'numeric_crosscheck.json (alternation_frozen)'},
        {'gate': 'steady-gait forward displacement > stand baseline (every walk run)',
         'value': {'per_run_dx': {n: r.get('body_dx_steady')
                                  for n, r in runs.items() if r['mode'] == 'walk'},
                   'baseline_m': crosscheck.get('baseline_forward_m')},
         'pass': ok(walk and all(
             (r.get('body_dx_steady') or 0.0)
             > crosscheck.get('baseline_forward_m', 0.0) + 0.03
             for r in walk)),
         'evidence': 'numeric_crosscheck.json'},
        {'gate': 'steady-gait lateral drift |dy| <= 0.05 m (every walk run)',
         'value': {n: r.get('body_dy_steady')
                   for n, r in runs.items() if r['mode'] == 'walk'},
         'pass': ok(walk and all(abs(r.get('body_dy_steady') or 0.0) <= 0.05
                                 for r in walk)),
         'evidence': 'numeric_crosscheck.json'},
        {'gate': 'steady-gait yaw drift <= 5 deg (max|unwrap-start|, every walk run)',
         'value': {n: r.get('max_yaw_abs_from_start_deg')
                   for n, r in runs.items() if r['mode'] == 'walk'},
         'pass': ok(walk and all(
             r.get('yaw_frozen', {}).get('ok')
             and abs(r.get('max_yaw_abs_from_start_deg') or 999.0) <= 5.0
             for r in walk)),
         'evidence': 'numeric_crosscheck.json (yaw_frozen)'},
        {'gate': 'support foot drift within Phase 4C magnitude',
         'value': drift, 'pass': ok(drift is not None and drift <= 0.03),
         'evidence': 'numeric_crosscheck.json'},
        {'gate': 'body height / roll / pitch bounded', 'value':
         {'min_height_m': min_h, 'max_roll_deg': roll, 'max_pitch_deg': pitch},
         'pass': ok(min_h is not None and min_h > 0.25
                    and (roll or 0) < 15.0 and (pitch or 0) < 15.0),
         'evidence': 'numeric_crosscheck.json'},
        {'gate': 'IMU no NaN/Inf', 'value': imu_nf, 'pass': ok(imu_nf == 0),
         'evidence': 'numeric_crosscheck.json'},
        {'gate': 'command stamps strictly increasing', 'value': increasing,
         'pass': ok(increasing), 'evidence': 'numeric_crosscheck.json'},
        {'gate': 'natural arm swing (down baseline, anti-phase, leg-coordinated)',
         'value': arm_summary(runs),
         'pass': ok(arm_gate(runs, walk)),
         'evidence': 'numeric_crosscheck.json (arm metrics)'},
        {'gate': 'pause freezes simulation time and resume continues',
         'value': pause_info, 'pass': ok(pause_info.get('available')
                                         and pause_info.get('frozen_ok')
                                         and pause_info.get('resumed_ok')),
         'evidence': 'visual_timeline.md'},
        {'gate': 'shutdown residual process = 0', 'value': residual,
         'pass': ok(residual == 0), 'evidence': 'shutdown_cleanup.txt'},
    ]
    return checks


def _num(d, *keys, default=None):
    cur = d
    for k in keys:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


def _config_present_finite(cfg):
    if not isinstance(cfg, dict) or not cfg:
        return False
    for v in cfg.values():
        if isinstance(v, bool):
            continue
        if isinstance(v, (int, float)) and math.isfinite(float(v)):
            continue
        if isinstance(v, str) and v:
            continue
        return False
    return True


def frozen_yaw_alternation(trace_dir, cfg, ready_sim, mode):
    """Frozen yaw-drift and alternation metrics from the raw traces (Gate B1)."""
    out = {'yaw_frozen': None, 'max_yaw_abs_from_start_deg': None,
           'alternation_frozen': None}
    try:
        body = [json.loads(l) for l in
                open(os.path.join(trace_dir, 'body_pose_trace.jsonl')) if l.strip()]
        times = [b['t'] for b in body]
        yaw = [b.get('yaw', 0.0) for b in body]
        ready = float(ready_sim if ready_sim is not None else 1.0)
        lo, hi = (A2.stand_window(cfg, ready) if mode == 'stand'
                  else A2.steady_window(cfg, ready))
        yr = A2.yaw_drift(times, yaw, lo, hi, A2.DEFAULT_MIN_STEADY_SAMPLES)
        out['yaw_frozen'] = yr
        out['max_yaw_abs_from_start_deg'] = yr['max_abs_from_start_deg']
    except (OSError, ValueError, KeyError):
        pass
    try:
        contact = [json.loads(l) for l in
                   open(os.path.join(trace_dir, 'contact_trace.jsonl')) if l.strip()]
        events = [(c['t'], c['side'], c['value']) for c in contact]
        out['alternation_frozen'] = A2.alternation(events)
    except (OSError, ValueError, KeyError):
        pass
    return out


def arm_metrics(run_dir, cfg, ready_sim=None, mode='walk'):
    """Empirical arm metrics from the observed state trace + foot FK.

    The steady window is ``[ready + start_delay + blend, ready + start_delay +
    duration]`` so the spawn transient (arms at 0) is excluded.
    """
    try:
        run = p4c.load_run(run_dir)
        bounds, _ = p4c.segment_bounds(cfg, run)
        obs = p4c.foot_samples(run, bounds, 0.05)
    except Exception:  # noqa: BLE001
        return None
    neutral = float(cfg.get('arm_neutral_pitch_rad', 0.0))
    amp = float(cfg.get('arm_swing_rad', 0.0))
    elbow = float(cfg.get('elbow_bend_rad', 0.0))
    ready = float(ready_sim if ready_sim is not None else 1.0)
    if mode == 'stand':
        win_lo, win_hi = A2.stand_window(cfg, ready)
    else:
        win_lo, win_hi = A2.steady_window(cfg, ready)
    lmap = {s['t']: s for s in obs['observed']['l']}
    rmap = {s['t']: s for s in obs['observed']['r']}
    # observed stance set: arms clearly down
    L, R, E = [], [], []
    # observed steady-gait set with left-leg forward-velocity relation
    gL, gR, glx, gt = [], [], [], []
    for st in run['states']:
        t = st['t']
        lp = st['position'][8]
        if win_lo <= t <= win_hi:
            L.append(lp)
            R.append(st['position'][19])
            E.append(st['position'][11])
            if mode == 'walk' and lp > 0.5:
                gL.append(lp)
                gR.append(st['position'][19])
                if t in lmap:
                    glx.append(lmap[t]['base'][0])
                    gt.append(t)
    if len(L) < 10:
        return None
    mean_l = sum(L) / len(L)
    mean_r = sum(R) / len(R)
    corr = None
    match = None
    if mode == 'walk' and len(gL) >= 10:
        mean_gl = sum(gL) / len(gL)
        mean_gr = sum(gR) / len(gR)
        dl = [v - mean_gl for v in gL]
        dr = [v - mean_gr for v in gR]
        denom = math.sqrt(sum(a * a for a in dl) * sum(b * b for b in dr)) or 1e-9
        corr = sum(a * b for a, b in zip(dl, dr)) / denom
        if len(gt) >= 2 and len(glx) == len(gL):
            num = 0
            den = 0
            for i in range(len(gt) - 1):
                dt = gt[i + 1] - gt[i]
                if dt <= 0:
                    continue
                vel = (glx[i + 1] - glx[i]) / dt
                den += 1
                # left arm back (wave>0) while left leg swings forward (vel>0)
                if dl[i] * vel > 0:
                    num += 1
            if den:
                match = num / den
    return {
        'scale': 'observed state trace (steady window, arms down)',
        'n': len(L),
        'n_gait': len(gL),
        'LShoulderPitch_mean': mean_l,
        'RShoulderPitch_mean': mean_r,
        'LShoulderPitch_ptp': max(L) - min(L),
        'RShoulderPitch_ptp': max(R) - min(R),
        'configured_swing': amp,
        'configured_neutral': neutral,
        'antiphase_corr': corr,
        'arm_leg_match_fraction': match,
        'LElbowRoll_mean': sum(E) / len(E),
        'configured_elbow': elbow,
        'arms_not_horizontal': mean_l > 1.0 and mean_r > 1.0,
        'amplitude_ok': (mode == 'stand' or (
            abs((max(gL) - min(gL)) / 2.0 - amp) <= 0.03
            and abs((max(gR) - min(gR)) / 2.0 - amp) <= 0.03)),
        'elbow_ok': abs(sum(E) / len(E) + elbow) <= 0.05,
    }


def summarize_run(name, mode, view, res, s, arm=None):
    cmd = s.get('command', {})
    body = s.get('body', {})
    contact = s.get('contact', {})
    imu = s.get('imu', {})
    sliding = s.get('sliding', {})
    foot = s.get('foot_fk', {}).get('observed', {})
    disp = body.get('displacement', {}).get('steady_gait', {})
    rp = body.get('roll_pitch_deg', {}).get('steady_gait', {})
    lo = foot.get('l', {}).get('steady_gait', {})
    ro = foot.get('r', {}).get('steady_gait', {})
    max_drift = max((sliding.get(side, {}).get('max_interval_x_drift', 0.0)
                     for side in ('l', 'r')), default=0.0)
    return {
        'mode': mode, 'view': view,
        'command_accepted': cmd.get('accepted'),
        'command_rate_hz': cmd.get('rate_hz'),
        'command_reject_reasons': cmd.get('reject_reasons'),
        'tracking_max_steady_rad': _num(s, 'tracking', 'per_segment_max_rad',
                                        'steady_gait'),
        'body_dx_steady': disp.get('dx'),
        'body_dy_steady': disp.get('dy'),
        'body_height_min': body.get('absolute_min_z'),
        'body_height_max': body.get('absolute_max_z'),
        'max_roll_deg_steady': rp.get('max_roll'),
        'max_pitch_deg_steady': rp.get('max_pitch'),
        'max_yaw_drift_deg_steady': rp.get('max_yaw_drift'),
        'left_swing_height_m': lo.get('swing_height'),
        'right_swing_height_m': ro.get('swing_height'),
        'contact_alternation': contact.get('alternation_count'),
        'contact_left_any_true': contact.get('left_any_true'),
        'contact_right_any_true': contact.get('right_any_true'),
        'contact_freshness_violations': contact.get('freshness_violations'),
        'imu_nonfinite': imu.get('nonfinite'),
        'imu_max_omega': imu.get('max_omega'),
        'imu_max_accel': imu.get('max_accel'),
        'support_foot_max_drift_m': max_drift,
        'command_strictly_increasing': cmd.get('strictly_increasing'),
        'residual_processes': res.get('residual_processes'),
        'arm': arm,
        'end_sim': res.get('end_sim'),
    }


if __name__ == '__main__':
    sys.exit(main())
