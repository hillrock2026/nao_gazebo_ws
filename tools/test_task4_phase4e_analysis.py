#!/usr/bin/env python3
"""Offline fixture tests for tools/phase4e_analysis.py (no Gazebo, no ROS).

Covers Gate B1 requirements: config consistency, stand-window isolation, yaw
wrap/unwrap, steady window boundaries, alternation semantics, and the frozen
forward/yaw gate behaviour.

Usage: python3 tools/test_task4_phase4e_analysis.py [--json-out F]
Exit 0 = all tests passed.
"""

import argparse
import json
import math
import os
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import phase4e_analysis as A  # noqa: E402

RESULTS = []


def check(name, cond, detail=None):
    RESULTS.append({'name': name, 'pass': bool(cond), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                           '' if cond else ' :: %s' % (detail,)))


CFG = {'start_delay_sec': 0.5, 'gait_blend_sec': 1.5, 'duration_sec': 18.0,
       'stand_sec': 24.0}
READY = 1.0


def test_config_consistency():
    expected = {'a': 1.0, 'b': 2, 'c': True, 'd': 'walk', 'e': 0.1}
    check('config_exact_pass',
          A.config_consistency(dict(expected), expected) == [])
    check('config_bool_vs_number_fail',
          any(d['error'] == 'type_mismatch'
              for d in A.config_consistency({**expected, 'a': True}, expected)))
    check('config_missing_fail',
          any(d['error'] == 'missing_in_run_config'
              for d in A.config_consistency({k: v for k, v in expected.items()
                                             if k != 'b'}, expected)))
    check('config_extra_fail',
          any(d['error'] == 'extra_unfrozen_key'
              for d in A.config_consistency({**expected, 'z': 1}, expected)))
    check('config_wrong_value_fail',
          any(d['error'] == 'value_mismatch'
              for d in A.config_consistency({**expected, 'a': 1.2}, expected)))
    check('config_nan_fail',
          any(d['error'] == 'non_finite_or_type'
              for d in A.config_consistency({**expected, 'a': float('nan')},
                                            expected)))
    check('config_string_mismatch_fail',
          any(d['error'] == 'value_mismatch'
              for d in A.config_consistency({**expected, 'd': 'stand'}, expected)))


def test_steady_window():
    lo, hi = A.steady_window(CFG, READY)
    check('steady_window_endpoints', abs(lo - 3.0) < 1e-9 and abs(hi - 19.5) < 1e-9,
          (lo, hi))
    times = [0.0, 2.9, 3.0, 3.001, 19.5, 19.501]
    idx = A.select_indices(times, lo, hi)
    check('steady_window_inclusive', idx == [2, 3, 4], idx)
    # insufficient samples
    few = A.yaw_drift([0, 1, 2], [0, 0, 0], 0, 10, min_samples=5)
    check('steady_window_insufficient', not few['ok']
          and few['reason'] == 'insufficient_samples', few)


def test_yaw():
    hi = 5.0
    # small drift, no wrap: +1 deg per sample over 100 samples
    times = [i * 0.1 for i in range(100)]
    yaws = [math.radians(0.05 * i) for i in range(100)]
    r = A.yaw_drift(times, yaws, 0, 10, min_samples=10)
    check('yaw_small_drift', r['ok'] and abs(r['net_deg'] - 4.95) < 1e-6
          and abs(r['max_abs_from_start_deg'] - 4.95) < 1e-3, r)
    # forward crossing +pi -> -pi (increase by ~3.6 deg steps through pi)
    yaws2 = []
    ang = math.pi - math.radians(18)
    for i in range(12):
        yaws2.append(ang)
        ang += math.radians(3.6)
        if ang > math.pi:
            ang -= 2 * math.pi
    r2 = A.yaw_drift([i * 0.1 for i in range(12)], yaws2, 0, 2, min_samples=10)
    check('yaw_forward_wrap', r2['ok'] and r2['net_deg'] > 0
          and abs(r2['net_deg'] - 39.6) < 0.2
          and abs(r2['max_abs_from_start_deg'] - 39.6) < 0.3, r2)
    # reverse crossing -pi -> +pi
    yaws3 = []
    ang = -math.pi + math.radians(18)
    for i in range(12):
        yaws3.append(ang)
        ang -= math.radians(3.6)
        if ang < -math.pi:
            ang += 2 * math.pi
    r3 = A.yaw_drift([i * 0.1 for i in range(12)], yaws3, 0, 2, min_samples=10)
    check('yaw_reverse_wrap', r3['ok'] and r3['net_deg'] < 0
          and abs(r3['net_deg'] + 39.6) < 0.2, r3)
    # multiple crossings
    yaws4 = [((i * 0.9) % (2 * math.pi)) - math.pi for i in range(60)]
    r4 = A.yaw_drift([i * 0.1 for i in range(60)], yaws4, 0, 6, min_samples=10)
    check('yaw_multiple_crossings_finite', r4['ok'] and math.isfinite(r4['net_deg']), r4)
    # non-finite
    bad = list(yaws)
    bad[5] = float('nan')
    r5 = A.yaw_drift(times, bad, 0, 10, min_samples=10)
    check('yaw_nonfinite_fail', not r5['ok'] and r5['reason'] == 'non_finite_yaw', r5)
    # gate behaviour: >5 deg must exceed threshold, <5 must not
    check('yaw_gate_threshold', abs(A.yaw_drift(
        [i * 0.1 for i in range(100)], [math.radians(0.05 * i) for i in range(100)],
        0, 10, 10)['max_abs_from_start_deg']) <= hi
        and abs(r2['max_abs_from_start_deg']) > hi)


def test_stand_window_no_transition_pollution():
    lo, hi = A.stand_window(CFG, READY)
    check('stand_window_endpoints', abs(lo - 3.0) < 1e-9 and abs(hi - 24.5) < 1e-9,
          (lo, hi))
    # ramp 0 -> 1.5 during [1.0, 3.0), flat 1.5 afterwards
    times = [i * 0.05 for i in range(600)]
    arms = []
    for t in times:
        if t < 3.0:
            arms.append(1.5 * min(1.0, max(0.0, (t - 1.0) / 2.0)))
        else:
            arms.append(1.5)
    idx = A.select_indices(times, lo, hi)
    vals = [arms[i] for i in idx]
    ptp = max(vals) - min(vals)
    check('stand_window_excludes_ramp', ptp < 1e-9 and len(vals) >= 200,
          {'ptp': ptp, 'n': len(vals)})
    # whole-series ptp would be polluted
    check('stand_whole_series_polluted', (max(arms) - min(arms)) > 1.4,
          max(arms) - min(arms))


def test_alternation():
    # correct strict alternation: L 0-1, R 1-2, L 2-3, R 3-4, L 4-5, R 5-6
    ev = []
    for k, side in enumerate(['left', 'right', 'left', 'right', 'left', 'right']):
        ev.append((k, side, True))
        ev.append((k + 1, side, False))
    r = A.alternation(ev)
    check('alternation_correct', r['ok'] and r['alternation_count'] == 5
          and abs(r['alternation_fraction'] - 1.0) < 1e-9, r)
    # single foot only (left repeats) -> fail
    ev2 = []
    for k in range(6):
        ev2.append((k, 'left', True))
        ev2.append((k + 1, 'left', False))
    r2 = A.alternation(ev2)
    check('alternation_single_foot_fail', not r2['ok'], r2)
    # sub-debounce pulse must be dropped, and pure jitter gives no valid support
    r3 = A.alternation([(0.0, 'left', True), (0.02, 'left', False)])
    check('alternation_subpulse_dropped', not r3['ok'] and r3['n_support'] == 0, r3)
    ev3 = []
    t = 0.0
    for k in range(20):
        ev3.append((t, 'left', True))
        ev3.append((t + 0.01, 'left', False))
        ev3.append((t, 'right', True))
        ev3.append((t + 0.01, 'right', False))
        t += 0.02
    r3b = A.alternation(ev3)
    check('alternation_jitter_fail', not r3b['ok'], r3b)
    # double support: overlapping L/R true -> both intervals, still alternate
    ev4 = []
    for k, side in enumerate(['left', 'right', 'left', 'right', 'left', 'right']):
        ev4.append((k, side, True))
        ev4.append((k + 1.05, side, False))
    r4 = A.alternation(ev4)
    check('alternation_double_support_ok', r4['ok']
          and r4['double_support_sec'] > 0, r4)
    # no contact at all
    r5 = A.alternation([])
    check('alternation_none_fail', not r5['ok'] and r5['n_support'] == 0, r5)


def test_leg_match_fraction():
    # steady window indices start at a NON-ZERO position (the original bug class)
    times = [0.0, 1.0, 2.0, 3.0, 4.0]
    idx = [2, 3, 4]
    wave = [1.0, -1.0, 1.0]          # local array aligned with idx, len 3 < 5
    lx = {2.0: 0.0, 3.0: 0.1, 4.0: 0.0}
    got = A.leg_match_fraction(times, idx, wave, lx)
    check('leg_match_nonzero_start_idx', got is not None and abs(got - 1.0) < 1e-12,
          got)
    # pre-fix absolute indexing would index wave[2..4] -> IndexError on len-3 wave
    try:
        _ = [wave[i] for i in idx]
        old_ok = False
    except IndexError:
        old_ok = True
    check('leg_match_prefix_absolute_index_fails', old_ok)

    # hand-derived mixed case: match, then zero-velocity (not >0) -> 0.5
    got2 = A.leg_match_fraction([0.0, 1.0, 2.0, 3.0], [1, 2, 3],
                                [1.0, -1.0, -1.0],
                                {1.0: 0.0, 2.0: 0.1, 3.0: 0.1})
    check('leg_match_hand_expected', got2 is not None and abs(got2 - 0.5) < 1e-12, got2)

    # explicit failures, no silent fallback
    check('leg_match_empty_none', A.leg_match_fraction(times, [], [], lx) is None)
    check('leg_match_len_mismatch_none',
          A.leg_match_fraction(times, [1, 2, 3], [1.0], lx) is None)
    check('leg_match_single_none', A.leg_match_fraction(times, [1], [1.0], lx) is None)
    check('leg_match_no_overlap_none',
          A.leg_match_fraction(times, [2, 3], [1.0, 1.0], {}) is None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('Task 4 Phase 4E Path B offline analysis fixture tests')
    test_config_consistency()
    test_steady_window()
    test_yaw()
    test_stand_window_no_transition_pollution()
    test_alternation()
    test_leg_match_fraction()
    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\nanalysis fixture tests: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w') as fh:
            json.dump({'passed': passed, 'total': total, 'results': RESULTS},
                      fh, indent=2, sort_keys=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
