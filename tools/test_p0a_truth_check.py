#!/usr/bin/env python3
"""P0-a truth-check fixtures (offline, no Gazebo).

Usage: python3 tools/test_p0a_truth_check.py [--json-out F]
"""

import argparse
import json
import os
import shutil
import sys
import tempfile

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import p0a_truth_check as T  # noqa: E402
import analyze_measured_gait as M  # noqa: E402

RESULTS = []
CFG = {'start_delay_sec': 0.5, 'gait_blend_sec': 1.5, 'duration_sec': 8.0}
REPRO = [
    ('docs/task_p0a_gait_search/work/reproduce/fwdf08_seed101',
     {'forward_rel_fit_r2': 0.9990157364284215,
      'max_single_frame_forward_m': 0.001020409906034081,
      'monotonic_non_decreasing_fraction': 0.8073229291716687}),
    ('docs/task_p0a_gait_search/work/reproduce/fwdf08_seed202',
     {'forward_rel_fit_r2': 0.9992020872326086,
      'max_single_frame_forward_m': 0.0008702223975676354,
      'monotonic_non_decreasing_fraction': 0.8134805345729227}),
]


def check(name, cond, detail=None):
    RESULTS.append({'name': name, 'pass': bool(cond), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                           '' if cond else ' :: %r' % (detail,)))


def make_run(root, name, rows, cfg=None):
    run_dir = os.path.join(root, name)
    trace = os.path.join(run_dir, 'trace')
    os.makedirs(trace)
    with open(os.path.join(trace, 'body_pose_trace.jsonl'), 'w') as fh:
        for r in rows:
            fh.write(json.dumps(r) + '\n')
    with open(os.path.join(trace, 'state_trace.jsonl'), 'w') as fh:
        fh.write(json.dumps({'t': 0.0, 'position': [0.0] * 24}) + '\n')
    with open(os.path.join(run_dir, 'run_config.json'), 'w') as fh:
        json.dump(dict(cfg or CFG), fh)
    with open(os.path.join(run_dir, 'result.json'), 'w') as fh:
        json.dump({'labels': {'ready_sim': 0.0}}, fh)
    return run_dir


def rows_linear(a=0.02, t_max=9.0, step=0.05, perturb=None):
    out = []
    n = int(round(t_max / step)) + 1
    for i in range(n):
        t = i * step
        x = a * t
        if perturb is not None:
            x = perturb(i, x)
        out.append({'t': t, 'x': x, 'y': 0.0, 'z': 0.33, 'roll': 0.0,
                    'pitch': 0.0, 'yaw': 0.0,
                    'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0})
    return out


def test_linear():
    tmp = tempfile.mkdtemp(prefix='p0a_tc_lin_')
    try:
        rd = make_run(tmp, 'run', rows_linear())
        r = T.truth_check(rd)
        check('linear_r2_one', abs(r['forward_rel_fit_r2'] - 1.0) < 1e-9, r)
        check('linear_slope', abs(r['forward_rel_fit_slope_mps'] - 0.02) < 1e-12, r)
        check('linear_max_step', abs(r['max_single_frame_forward_m']
                                     - 0.02 * 0.05) < 1e-12, r)
        check('linear_monotonic_one',
              abs(r['monotonic_non_decreasing_fraction'] - 1.0) < 1e-12, r)
        check('linear_fraction_definition',
              abs(r['max_single_frame_fraction_of_forward']
                  - (0.02 * 0.05) / (0.02 * (8.5 - 2.0))) < 1e-9, r)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_monotonic_definition():
    tmp = tempfile.mkdtemp(prefix='p0a_tc_mono_')
    try:
        # one real decrease inside the window -> one bad adjacent pair
        def perturb(i, x):
            # sample 100 is t=5.0 (x=0.1); drop sample 101 below it
            if i == 101:
                return 0.02 * 5.0 - 0.01
            return x
        rd = make_run(tmp, 'run', rows_linear(perturb=perturb))
        r = T.truth_check(rd)
        pairs = r['n_adjacent_pairs']
        check('monotonic_counts_one_bad',
              abs(r['monotonic_non_decreasing_fraction']
                  - (pairs - 1) / pairs) < 1e-12, r)
        # a decrease smaller than tol is tolerated
        def tiny(i, x):
            if i == 101:
                return 0.02 * 5.0 - 5e-5
            return x
        rd2 = make_run(tmp, 'run2', rows_linear(perturb=tiny))
        r2 = T.truth_check(rd2)
        check('monotonic_tol_absorbs_tiny_decrease',
              abs(r2['monotonic_non_decreasing_fraction'] - 1.0) < 1e-12, r2)
        r3 = T.truth_check(rd2, monotonic_tol=0.0)
        check('monotonic_tol_zero_counts_tiny',
              r3['monotonic_non_decreasing_fraction'] < 1.0, r3)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_explicit_failures():
    tmp = tempfile.mkdtemp(prefix='p0a_tc_fail_')
    try:
        # only 1 sample inside window
        rows = [{'t': 2.0, 'x': 0.0, 'y': 0.0, 'z': 0.3, 'roll': 0.0,
                 'pitch': 0.0, 'yaw': 0.0, 'qx': 0, 'qy': 0, 'qz': 0, 'qw': 1}]
        rd = make_run(tmp, 'short', rows)
        try:
            T.truth_check(rd)
            check('short_window_raises', False)
        except M.MeasuredGaitError:
            check('short_window_raises', True)
        # missing yaw
        bad = rows_linear()
        bad[100].pop('yaw')
        rd2 = make_run(tmp, 'bad_yaw', bad)
        try:
            T.truth_check(rd2)
            check('missing_yaw_raises', False)
        except M.MeasuredGaitError:
            check('missing_yaw_raises', True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_reproduce_recorded():
    ws = os.path.dirname(_TOOLS)
    for rel, expected in REPRO:
        rd = os.path.join(ws, rel)
        if not os.path.isdir(rd):
            check('reproduce_%s_skip' % rel, True)
            continue
        r = T.truth_check(rd)
        ok = all(abs(r[k] - v) < 1e-12 for k, v in expected.items())
        check('reproduce_%s' % os.path.basename(rel), ok, r)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('P0-a truth-check fixtures')
    test_linear()
    test_monotonic_definition()
    test_explicit_failures()
    test_reproduce_recorded()
    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\ntruth-check fixtures: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w') as fh:
            json.dump({'passed': passed, 'total': total, 'results': RESULTS},
                      fh, indent=2, sort_keys=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
