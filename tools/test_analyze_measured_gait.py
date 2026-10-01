#!/usr/bin/env python3
"""Offline fixture tests for tools/analyze_measured_gait.py (no Gazebo, no ROS).

Covers the P0-a lightweight robustness fixes:

  * attitude fields (yaw/roll/pitch) must be present and finite -- a missing or
    non-finite yaw/roll/pitch marks the episode measurable=false instead of
    silently defaulting to world +x (Fix 3);
  * trace truncation is disclosed (nominal vs actual window, effective
    duration, missing duration) and an under-length window is not measurable
    (Fix 2);
  * fall is detected from measured height/tilt evidence, kept in the run
    metrics, and excluded from non-fall group statistics (Fix 4);
  * yaw wrap is handled by the frozen unwrap;
  * non-monotonic timestamps are flagged.

Usage: python3 tools/test_analyze_measured_gait.py [--json-out F]
Exit 0 = all tests passed.
"""

import argparse
import json
import math
import os
import shutil
import sys
import tempfile

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import analyze_measured_gait as M  # noqa: E402

RESULTS = []
CFG = {'start_delay_sec': 0.5, 'gait_blend_sec': 1.5, 'duration_sec': 8.0}
YAW0 = 3.10  # near +pi, so a wrap test crosses the branch cut


def check(name, cond, detail=None):
    RESULTS.append({'name': name, 'pass': bool(cond), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                           '' if cond else ' :: %r' % (detail,)))


def _write_rows(trace_dir, rows):
    os.makedirs(trace_dir, exist_ok=True)
    with open(os.path.join(trace_dir, 'body_pose_trace.jsonl'), 'w',
              encoding='utf-8') as fh:
        for row in rows:
            fh.write(json.dumps(row) + '\n')


def make_run(root, name, rows, cfg=None):
    run_dir = os.path.join(root, name)
    trace_dir = os.path.join(run_dir, 'trace')
    _write_rows(trace_dir, rows)
    with open(os.path.join(trace_dir, 'state_trace.jsonl'), 'w',
              encoding='utf-8') as fh:
        fh.write(json.dumps({'t': 0.0, 'position': [0.0] * 24}) + '\n')
    run_cfg = dict(cfg or CFG)
    with open(os.path.join(run_dir, 'run_config.json'), 'w',
              encoding='utf-8') as fh:
        json.dump(run_cfg, fh)
    with open(os.path.join(run_dir, 'result.json'), 'w', encoding='utf-8') as fh:
        json.dump({'labels': {'ready_sim': 0.0}, 'config_id': name,
                   'seed': 101}, fh)
    return run_dir


def straight_rows(t_max=9.0, step=0.05, speed=0.05, z=0.33, pitch=0.0,
                  roll=0.0, yaw_fn=None, yaw=YAW0, mutate=None):
    rows = []
    n = int(round(t_max / step)) + 1
    for i in range(n):
        t = i * step
        s = speed * t
        row = {'t': t, 'x': s * math.cos(yaw), 'y': s * math.sin(yaw), 'z': z,
               'roll': roll, 'pitch': pitch,
               'yaw': (yaw_fn(t) if yaw_fn else yaw),
               'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}
        if mutate is not None:
            mutate(row, i)
        rows.append(row)
    return rows


def test_yaw_missing_fails():
    tmp = tempfile.mkdtemp(prefix='p0a_yaw_missing_')
    try:
        def drop_yaw(row, i):
            if i == 100:
                row.pop('yaw')
        rows = straight_rows(mutate=drop_yaw)
        rd = make_run(tmp, 'run', rows)
        res = M.compute_run(rd, 'test')
        check('yaw_missing_not_measurable', res.get('measurable') is False)
        check('yaw_missing_reason', res.get('reason') == 'invalid_attitude_field',
              res.get('reason'))
        check('yaw_missing_no_forward',
              'forward_displacement_m' not in res)
        an = (res.get('anomalies') or [{}])[0]
        check('yaw_missing_anomaly_field', an.get('field') == 'yaw', an)
        check('yaw_missing_anomaly_type', an.get('type') == 'attitude_field_missing',
              an)
        check('yaw_missing_run_identity', an.get('run_dir') == rd, an)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_yaw_nan_inf_fail():
    for label, value in (('nan', float('nan')), ('inf', float('inf'))):
        tmp = tempfile.mkdtemp(prefix='p0a_yaw_%s_' % label)
        try:
            def set_bad(row, i, value=value):
                if i == 100:
                    row['yaw'] = value
            rows = straight_rows(mutate=set_bad)
            rd = make_run(tmp, 'run', rows)
            res = M.compute_run(rd, 'test')
            check('yaw_%s_not_measurable' % label,
                  res.get('measurable') is False)
            check('yaw_%s_reason' % label,
                  res.get('reason') == 'invalid_attitude_field',
                  res.get('reason'))
            an = (res.get('anomalies') or [{}])[0]
            check('yaw_%s_anomaly_type' % label,
                  an.get('type') == 'attitude_field_nonfinite', an)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


def test_yaw_wrap_ok():
    tmp = tempfile.mkdtemp(prefix='p0a_yaw_wrap_')
    try:
        def wrap(t):
            return YAW0 if t < 4.5 else -YAW0
        rows = straight_rows(yaw_fn=wrap, yaw=YAW0)
        rd = make_run(tmp, 'run', rows)
        res = M.compute_run(rd, 'test')
        check('yaw_wrap_measurable', res.get('measurable') is True,
              res.get('reason'))
        check('yaw_wrap_start_unwrapped',
              abs(res.get('yaw_start_rad') - YAW0) < 1e-9,
              res.get('yaw_start_rad'))
        check('yaw_wrap_forward_positive',
              res.get('forward_displacement_m', 0.0) > 0.0,
              res.get('forward_displacement_m'))
        check('yaw_wrap_lateral_small',
              abs(res.get('lateral_displacement_m', 9.9)) < 1e-6,
              res.get('lateral_displacement_m'))
        check('yaw_wrap_change_finite',
              isinstance(res.get('yaw_change_deg'), float)
              and math.isfinite(res['yaw_change_deg']),
              res.get('yaw_change_deg'))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_non_monotonic_flagged():
    tmp = tempfile.mkdtemp(prefix='p0a_nonmono_')
    try:
        rows = straight_rows()
        rows[20]['t'] = rows[19]['t']  # duplicate timestamp
        rd = make_run(tmp, 'run', rows)
        res = M.compute_run(rd, 'test')
        types = [a['type'] for a in res.get('anomalies', [])]
        check('non_monotonic_flagged', 'non_monotonic_timestamp' in types, types)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_truncation_disclosed():
    tmp = tempfile.mkdtemp(prefix='p0a_trunc_')
    try:
        cfg = dict(CFG, duration_sec=12.0)
        rows = straight_rows(t_max=9.0)
        rd = make_run(tmp, 'run', rows, cfg=cfg)
        res = M.compute_run(rd, 'test')
        win = res.get('window', {})
        check('truncation_measurable', res.get('measurable') is True,
              res.get('reason'))
        check('truncation_true', win.get('truncated') is True, win)
        check('truncation_actual_hi', abs(win.get('actual_hi') - 9.0) < 1e-9, win)
        check('truncation_missing_duration',
              abs(win.get('missing_duration_sec') - 3.5) < 1e-6, win)
        check('truncation_effective_duration',
              abs(win.get('effective_duration_sec') - 7.0) < 1e-6, win)
        check('truncation_flag_on_run', res.get('trace_truncated') is True, res)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_short_effective_window_not_measurable():
    tmp = tempfile.mkdtemp(prefix='p0a_short_')
    try:
        cfg = dict(CFG, duration_sec=20.0)
        rows = straight_rows(t_max=6.5)  # window [2.0, 20.5] -> ~4.5 s actual
        rd = make_run(tmp, 'run', rows, cfg=cfg)
        res = M.compute_run(rd, 'test')
        check('short_window_not_measurable', res.get('measurable') is False,
              res.get('reason'))
        check('short_window_reason',
              res.get('reason') == 'insufficient_effective_duration',
              res.get('reason'))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_fall_detection_and_group_stats():
    tmp = tempfile.mkdtemp(prefix='p0a_fall_')
    try:
        normal = straight_rows(t_max=9.0)
        fall_height = straight_rows(t_max=9.0, mutate=lambda r, i: (
            r.__setitem__('z', 0.10) if 80 <= i <= 90 else None))
        fall_tilt = straight_rows(t_max=9.0, mutate=lambda r, i: (
            r.__setitem__('pitch', 1.2) if 80 <= i <= 90 else None))
        rd_n = make_run(tmp, 'normal', normal)
        rd_h = make_run(tmp, 'fall_height', fall_height)
        rd_t = make_run(tmp, 'fall_tilt', fall_tilt)
        res_n = M.compute_run(rd_n, 'src')
        res_h = M.compute_run(rd_h, 'src')
        res_t = M.compute_run(rd_t, 'src')
        check('no_fall_normal', res_n.get('fall') is False, res_n.get('fall'))
        check('fall_height_true', res_h.get('fall') is True, res_h.get('fall'))
        check('fall_height_evidence',
              any(e['type'] == 'height_drop' for e in res_h.get('fall_evidence', [])),
              res_h.get('fall_evidence'))
        check('fall_tilt_true', res_t.get('fall') is True, res_t.get('fall'))
        check('fall_tilt_evidence',
              any(e['type'] == 'torso_tilt' for e in res_t.get('fall_evidence', [])),
              res_t.get('fall_evidence'))
        check('fall_not_deleted',
              res_h.get('forward_displacement_m') is not None)

        # give all three the same source/config so they land in one group
        for r in (res_n, res_h, res_t):
            r['source'] = 'src'
            r['config_id'] = 'C'
        groups = M.group_stats([res_n, res_h, res_t])
        g = groups['src::C']
        check('group_total_count', g['total_episode_count'] == 3, g)
        check('group_fall_count', g['fall_count'] == 2, g)
        check('group_non_fall_count', g['non_fall_measurable_count'] == 1, g)
        check('group_fall_rate', abs(g['fall_rate'] - 2.0 / 3.0) < 1e-12, g)
        check('group_non_fall_stats_use_one', g['actual_speed_mean_mps']['n'] == 1,
              g['actual_speed_mean_mps'])
        check('group_fall_runs_kept', len(g['fall_runs']) == 2, g['fall_runs'])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_stats_single_sample_std_none():
    s = M._stats([0.1])
    check('stats_single_std_none', s['std'] is None, s)
    s2 = M._stats([0.1, 0.3])
    check('stats_two_std_present', abs(s2['std'] - 0.1) < 1e-12, s2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('P0-a measured gait analyser fixture tests')
    test_yaw_missing_fails()
    test_yaw_nan_inf_fail()
    test_yaw_wrap_ok()
    test_non_monotonic_flagged()
    test_truncation_disclosed()
    test_short_effective_window_not_measurable()
    test_fall_detection_and_group_stats()
    test_stats_single_sample_std_none()
    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\nanalyze_measured_gait fixture tests: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w') as fh:
            json.dump({'passed': passed, 'total': total, 'results': RESULTS},
                      fh, indent=2, sort_keys=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
