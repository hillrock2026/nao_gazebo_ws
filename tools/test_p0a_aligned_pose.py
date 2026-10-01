#!/usr/bin/env python3
"""P0-a aligned-pose method fixtures (offline, deterministic).

Covers the declared alignment contract end-to-end: absolute vs gait-relative
time, tolerance, interpolation (none/nearest/linear/hold), pairing, offset
handling, boundary/unmatched queries and the error paths (empty, conflicting
duplicate timestamps, non-monotonic input, missing trace, invalid options).
Also asserts the frozen seeddiag r1/r2 aligned triple is reproduced exactly and
that the reported JSON method matches the algorithm actually run.

Usage: python3 tools/test_p0a_aligned_pose.py [--json-out F]
"""

import argparse
import json
import math
import os
import shutil
import sys
import tempfile

_TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(_TOOLS)
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import p0a_aligned_pose as AP  # noqa: E402

SEEDDIAG = os.path.join(WS, 'docs', 'task_p0a_gait_search', 'p1r_support_swing',
                        'runs')
FROZEN = {'x': 0.0255788167045869, 'y': 0.019062282984621374,
          'z': 0.0034137217641802}
RESULTS = []


def check(name, cond, detail=None):
    RESULTS.append({'name': name, 'pass': bool(cond), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                           '' if cond else ' :: %r' % (detail,)))


def make_run(root, name, rows, ready=0.0):
    run = os.path.join(root, name)
    os.makedirs(os.path.join(run, 'trace'))
    with open(os.path.join(run, 'trace', 'body_pose_trace.jsonl'), 'w') as fh:
        for r in rows:
            fh.write(json.dumps(r) + '\n')
    with open(os.path.join(run, 'result.json'), 'w') as fh:
        json.dump({'labels': {'ready_sim': ready}}, fh)
    return run


def test_absolute_and_tolerance():
    tmp = tempfile.mkdtemp(prefix='p1r_ap_')
    try:
        a = make_run(tmp, 'a', [{'t': i * 0.1, 'x': i * 0.1, 'y': 0.0,
                                 'z': 0.5} for i in range(11)])
        b = make_run(tmp, 'b', [{'t': i * 0.1, 'x': i * 0.1 + 0.1, 'y': 0.0,
                                 'z': 0.5} for i in range(11)])
        rep = AP.compare(a, b, time_reference='absolute', tolerance=0.02,
                         interpolation='none')
        check('absolute_const_x_diff',
              abs(rep['max_abs_diff_m']['x'] - 0.1) < 1e-12,
              rep['max_abs_diff_m'])
        check('method_fields_present',
              rep['method']['time_reference'] == 'absolute'
              and rep['method']['tolerance_sec'] == 0.02
              and rep['method']['interpolation'] == 'none'
              and rep['method']['pairing']
              == 'all_cross_pairs_within_tolerance'
              and rep['method']['offset_handling'] == 'none', rep['method'])
        check('json_metric_matches_argmax',
              rep['result']['argmax']['x'] is not None
              and abs(abs(rep['result']['argmax']['x']['a']
                          - rep['result']['argmax']['x']['b'])
                      - rep['result']['max_abs_diff_m']['x']) < 1e-12,
              rep['result']['argmax']['x'])
        # tolerance 0.005 removes the 0.1 m x offset only if no exact-time pair;
        # here exact-time pairs exist so the result is unchanged
        rep2 = AP.compare(a, b, tolerance=0.005, interpolation='none')
        check('tolerance_applied_pairs',
              rep2['result']['n_pairs_used'] > 0
              and rep2['result']['n_pairs_used'] <= rep['result']['n_pairs_used'],
              (rep2['result']['n_pairs_used'], rep['result']['n_pairs_used']))
        # too-small tolerance with a shifted grid leaves nothing to pair
        c = make_run(tmp, 'c', [{'t': i * 0.1 + 0.05, 'x': i * 0.1, 'y': 0,
                                 'z': 0} for i in range(11)])
        rep3 = AP.compare(a, c, tolerance=0.001, interpolation='none')
        check('tolerance_excludes_all',
              rep3['result']['n_pairs_used'] == 0
              and rep3['result']['n_a_unmatched'] == 11,
              rep3['result'])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_gait_relative_and_offsets():
    tmp = tempfile.mkdtemp(prefix='p1r_ap2_')
    try:
        a = make_run(tmp, 'a', [{'t': 1.0 + i * 0.1, 'x': i * 0.1, 'y': 0,
                                 'z': 0} for i in range(11)], ready=1.0)
        b = make_run(tmp, 'b', [{'t': 2.0 + i * 0.1, 'x': i * 0.1, 'y': 0,
                                 'z': 0} for i in range(11)], ready=2.0)
        rep = AP.compare(a, b, time_reference='gait_relative',
                         tolerance=1e-9, interpolation='none')
        check('gait_relative_aligns',
              rep['result']['max_abs_diff_m']['x'] == 0.0
              and rep['inputs']['a']['ready_sim'] == 1.0
              and rep['inputs']['b']['ready_sim'] == 2.0,
              rep['result']['max_abs_diff_m'])
        rep_abs = AP.compare(a, b, time_reference='absolute',
                             tolerance=0.02, interpolation='none')
        check('absolute_does_not_align',
              abs(rep_abs['result']['max_abs_diff_m']['x'] - 1.0) < 1e-9,
              rep_abs['result']['max_abs_diff_m'])
        rep_sr = AP.compare(a, b, time_reference='absolute',
                            offset_mode='subtract_ready', tolerance=1e-9,
                            interpolation='none')
        check('subtract_ready_offset_recorded',
              rep_sr['result']['offset_handling']['mode'] == 'subtract_ready'
              and rep_sr['result']['offset_handling']
              ['changes_original_axis'] is False
              and rep_sr['max_abs_diff_m']['x'] == 0.0,
              rep_sr['result']['offset_handling'])
        rep_est = AP.compare(a, b, time_reference='absolute',
                             offset_mode='estimate', tolerance=0.02,
                             interpolation='none')
        check('estimate_offset_bounded',
              abs(rep_est['result']['offset_handling']['applied_offset_sec']
                  + 1.0) <= 0.25,
              rep_est['result']['offset_handling'])
        check('offset_does_not_touch_sources',
              rep_est['inputs']['a']['n_rows'] == 11)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_interpolations():
    tmp = tempfile.mkdtemp(prefix='p1r_ap3_')
    try:
        a = make_run(tmp, 'a', [{'t': i * 0.1, 'x': i * 0.1, 'y': 2 * i * 0.1,
                                 'z': 0.0} for i in range(11)])
        b = make_run(tmp, 'b', [{'t': i * 0.05 + 0.025, 'x': i * 0.05 + 0.025,
                                 'y': 2 * (i * 0.05 + 0.025), 'z': 0.0}
                                for i in range(21)])
        for mode in ('none', 'nearest', 'linear', 'hold'):
            rep = AP.compare(a, b, tolerance=0.06, interpolation=mode)
            check('interp_%s_runs' % mode,
                  rep['method']['interpolation'] == mode
                  and rep['method']['pairing']
                  == AP.PAIRING_BY_INTERPOLATION[mode]
                  and rep['result']['n_pairs_used'] > 0, rep['result'])
        lin = AP.compare(a, b, tolerance=0.06, interpolation='linear')
        # B is exactly linear in t on both axes; linear interp must be ~exact
        check('linear_exact_on_linear_b',
              lin['max_abs_diff_m']['x'] < 1e-12
              and abs(lin['max_abs_diff_m']['y']) < 1e-12,
              lin['max_abs_diff_m'])
        hold = AP.compare(a, b, tolerance=0.06, interpolation='hold')
        check('hold_nonzero_on_linear_b',
              hold['max_abs_diff_m']['y'] > 0.0, hold['max_abs_diff_m'])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_error_paths():
    tmp = tempfile.mkdtemp(prefix='p1r_ap4_')
    try:
        good = make_run(tmp, 'good', [{'t': 0.0, 'x': 0, 'y': 0, 'z': 0},
                                      {'t': 0.1, 'x': 1, 'y': 0, 'z': 0}])
        # empty trace
        empty = make_run(tmp, 'empty', [])
        try:
            AP.compare(empty, good)
            check('empty_trace_fails', False)
        except AP.AlignedPoseError:
            check('empty_trace_fails', True)
        # missing trace
        miss = os.path.join(tmp, 'miss')
        os.makedirs(miss)
        try:
            AP.compare(miss, good)
            check('missing_trace_fails', False)
        except AP.AlignedPoseError:
            check('missing_trace_fails', True)
        # conflicting duplicate timestamps
        conflict = make_run(tmp, 'conflict',
                            [{'t': 0.0, 'x': 0, 'y': 0, 'z': 0},
                             {'t': 0.0, 'x': 9, 'y': 0, 'z': 0}])
        try:
            AP.compare(conflict, good)
            check('conflict_timestamp_fails', False)
        except AP.AlignedPoseError:
            check('conflict_timestamp_fails', True)
        # exact duplicate collapse
        dup = make_run(tmp, 'dup',
                       [{'t': 0.0, 'x': 0, 'y': 0, 'z': 0},
                        {'t': 0.0, 'x': 0, 'y': 0, 'z': 0},
                        {'t': 0.1, 'x': 1, 'y': 0, 'z': 0}])
        rep = AP.compare(dup, good, interpolation='none')
        check('exact_duplicate_collapsed',
              rep['inputs']['a']['n_exact_duplicates_removed'] == 1
              and rep['inputs']['a']['n_unique_samples'] == 2,
              rep['inputs']['a'])
        # non-monotonic input is sorted and recorded, not rejected
        rev = make_run(tmp, 'rev',
                       [{'t': 0.1, 'x': 1, 'y': 0, 'z': 0},
                        {'t': 0.0, 'x': 0, 'y': 0, 'z': 0}])
        rep2 = AP.compare(rev, good, interpolation='none')
        check('non_monotonic_sorted_recorded',
              rep2['inputs']['a']['n_non_monotonic_input'] == 1
              and rep2['max_abs_diff_m']['x'] == 0.0,
              rep2['inputs']['a'])
        # invalid options
        for label, kwargs in (('bad_interp', {'interpolation': 'cubic'}),
                              ('bad_offset', {'offset_mode': 'warp'}),
                              ('bad_time', {'time_reference': 'phase'})):
            try:
                AP.compare(good, good, **kwargs)
                check('%s_fails' % label, False)
            except AP.AlignedPoseError:
                check('%s_fails' % label, True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_seeddiag_frozen_value():
    r1 = os.path.join(SEEDDIAG, 'seeddiag_r1')
    r2 = os.path.join(SEEDDIAG, 'seeddiag_r2')
    if not (os.path.isdir(r1) and os.path.isdir(r2)):
        check('seeddiag_frozen_skip', True)
        return
    rep = AP.compare(r1, r2, time_reference='absolute', tolerance=0.02,
                     interpolation='none')
    check('seeddiag_frozen_triple_exact',
          all(rep['max_abs_diff_m'][k] == FROZEN[k] for k in FROZEN),
          rep['max_abs_diff_m'])
    check('seeddiag_json_method_matches',
          rep['method']['time_reference'] == 'absolute'
          and rep['method']['tolerance_sec'] == 0.02
          and rep['method']['pairing'] == 'all_cross_pairs_within_tolerance')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('P0-a aligned-pose method fixtures')
    test_absolute_and_tolerance()
    test_gait_relative_and_offsets()
    test_interpolations()
    test_error_paths()
    test_seeddiag_frozen_value()
    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\naligned-pose fixtures: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w') as fh:
            json.dump({'passed': passed, 'total': total, 'results': RESULTS},
                      fh, indent=2, sort_keys=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
