#!/usr/bin/env python3
"""Task 4 Phase 4E offline tests for the natural arm swing of the producer.

Constructs the real ``Phase4bProducer`` node in-process (no simulation, no
publisher traffic) and checks the arm baseline/swing analytically. These tests
do NOT represent real collection.

Usage: python3 tools/test_task4_phase4b_arm_swing.py [--json-out F]
Exit 0 = all tests passed.
"""

import argparse
import importlib.util
import json
import math
import os
import sys

import rclpy

_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'nao_gz_sim', 'scripts'))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

_spec = importlib.util.spec_from_file_location(
    'phase4b_producer', os.path.join(_SCRIPTS, 'phase4b_producer.py'))
producer = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(producer)
JOINT_ORDER = producer.JOINT_ORDER
LIMITS = producer.LIMITS

RESULTS = []


def check(name, cond, detail=None):
    RESULTS.append({'name': name, 'pass': bool(cond), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                           '' if cond else ' :: %s' % (detail,)))


def make_node(overrides=None):
    args = ['--ros-args']
    for k, v in (overrides or {}).items():
        args += ['-p', '%s:=%s' % (k, v)]
    rclpy.init(args=args)
    return producer.Phase4bProducer()


def with_node(overrides, fn):
    node = None
    try:
        node = make_node(overrides)
        return fn(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def test_stance():
    def case(node):
        cmd = node.stance_cmd
        sp = {n: cmd[JOINT_ORDER.index(n)] for n in
              ('LShoulderPitch', 'RShoulderPitch', 'LShoulderRoll',
               'RShoulderRoll', 'LElbowRoll', 'RElbowRoll')}
        check('stance_not_horizontal',
              abs(sp['LShoulderPitch'] - 0.0) > 1.0
              and abs(sp['RShoulderPitch'] - 0.0) > 1.0, sp)
        check('stance_neutral_matches_param',
              abs(sp['LShoulderPitch'] - node.arm_neutral_pitch) < 1e-9
              and abs(sp['RShoulderPitch'] - node.arm_neutral_pitch) < 1e-9, sp)
        check('stance_roll_mirrored',
              abs(sp['LShoulderRoll'] - node.arm_neutral_roll) < 1e-9
              and abs(sp['RShoulderRoll'] + node.arm_neutral_roll) < 1e-9, sp)
        check('stance_elbows_bent_mirrored',
              abs(sp['LElbowRoll'] + node.elbow_bend) < 1e-9
              and abs(sp['RElbowRoll'] - node.elbow_bend) < 1e-9, sp)
    with_node({}, case)


def _arm(cmd, name):
    return cmd[JOINT_ORDER.index(name)]


def test_swing():
    def case(node):
        neutral = node.arm_neutral_pitch
        amp = node.arm_swing
        phases = [i / 200.0 for i in range(200)]
        L = [_arm(node.gait_target(p), 'LShoulderPitch') for p in phases]
        R = [_arm(node.gait_target(p), 'RShoulderPitch') for p in phases]
        check('arms_sum_constant',
              max(abs((L[i] + R[i]) - 2.0 * neutral) for i in range(200)) < 1e-9)
        # peak-to-peak amplitude == 2*arm_swing
        pp_l = max(L) - min(L)
        pp_r = max(R) - min(R)
        check('amplitude_matches_config',
              abs(pp_l - 2 * amp) < 1e-6 and abs(pp_r - 2 * amp) < 1e-6,
              (pp_l, pp_r, amp))
        # anti-phase: L(p) - neutral == -(R(p) - neutral)
        check('arms_antiphase',
              max(abs((L[i] - neutral) + (R[i] - neutral)) for i in range(200)) < 1e-9)
        # leg coordination: mid left swing -> left arm back (pitch>neutral),
        # right arm forward (pitch<neutral)
        swing_ratio = 0.5 - node.ds_ratio / 2.0
        mid = swing_ratio / 2.0
        cmd = node.gait_target(mid)
        check('left_leg_forward_left_arm_back',
              _arm(cmd, 'LShoulderPitch') > neutral + 1e-6
              and _arm(cmd, 'RShoulderPitch') < neutral - 1e-6,
              (_arm(cmd, 'LShoulderPitch'), _arm(cmd, 'RShoulderPitch'), neutral))
        # all joints within frozen limits across a full cycle
        bad = []
        for p in phases:
            t = node.gait_target(p)
            for n, v in zip(JOINT_ORDER, t):
                lo, hi = LIMITS[n]
                if not math.isfinite(v) or v < lo - 1e-9 or v > hi + 1e-9:
                    bad.append((p, n, v))
        check('all_targets_within_limits', not bad, bad[:3])
        # arm wave frequency equals gait frequency: period 1/freq
        p_a = node.gait_target(0.10)
        p_b = node.gait_target(0.10 + 1.0 / node.freq * node.freq)  # +1 period
        check('arm_wave_phase_periodic',
              abs(_arm(p_a, 'LShoulderPitch') - _arm(p_b, 'LShoulderPitch')) < 1e-9)
        # blend continuity at gait start (phase 0) with stance
        g0 = node.gait_target(0.0)
        blended0 = node.blend(node.stance_cmd, g0, 0.0)
        check('blend_start_equals_stance',
              max(abs(a - b) for a, b in zip(blended0, node.stance_cmd)) < 1e-9)
        check('gait_phase0_arm_continuous',
              abs(_arm(g0, 'LShoulderPitch') - neutral) < 1e-9)
        # stop ramp end equals stance
        ramp_from = node.gait_target((node.duration * node.freq) % 1.0)
        ramp_end = node.blend(ramp_from, node.stance_cmd, 1.0)
        check('stop_ramp_end_equals_stance',
              max(abs(a - b) for a, b in zip(ramp_end, node.stance_cmd)) < 1e-9)
    with_node({}, case)


def test_zero_swing_keeps_neutral():
    def case(node):
        g = node.gait_target(0.25)
        check('zero_swing_keeps_neutral',
              abs(_arm(g, 'LShoulderPitch') - node.arm_neutral_pitch) < 1e-9
              and abs(_arm(g, 'RShoulderPitch') - node.arm_neutral_pitch) < 1e-9
              and abs(node.arm_neutral_pitch) > 1.0,
              (_arm(g, 'LShoulderPitch'), node.arm_neutral_pitch))
    with_node({'arm_swing_rad': 0.0}, case)


def test_phase_offset():
    def case(node):
        g0 = node.gait_target(0.0)
        check('phase_offset_applied',
              abs(_arm(g0, 'LShoulderPitch')
                  - (node.arm_neutral_pitch + node.arm_swing * math.sin(node.arm_phase_offset)))
              < 1e-9)
    with_node({'arm_phase_offset_rad': math.pi / 2.0}, case)


def test_invalid_params():
    cases = {
        'neutral_below_range': {'arm_neutral_pitch_rad': -0.1},
        'neutral_above_range': {'arm_neutral_pitch_rad': 2.2},
        'swing_too_large': {'arm_swing_rad': 5.0},
        'elbow_zero': {'elbow_bend_rad': 0.0},
        'neutral_nan': {'arm_neutral_pitch_rad': 'nan'},
        'roll_out_of_range': {'arm_neutral_roll_rad': 0.9},
    }
    for name, ov in cases.items():
        ok = False
        try:
            with_node(ov, lambda n: None)
        except Exception as exc:  # noqa: BLE001
            ok = isinstance(exc, ValueError)
        check('rejects_%s' % name, ok)


def test_control_compat():
    """CONTROL (legacy zero-pose) must be exactly reproducible by config."""
    cfg = {'arm_neutral_pitch_rad': 0.0, 'arm_neutral_roll_rad': 0.0,
           'elbow_bend_rad': 0.0349066, 'arm_swing_rad': 0.08,
           'arm_swing_sign': -1, 'arm_phase_offset_rad': 0.0}

    def case(node):
        st = node.stance_cmd
        check('control_stance_arms_at_zero',
              abs(_arm(st, 'LShoulderPitch')) < 1e-9
              and abs(_arm(st, 'RShoulderPitch')) < 1e-9)
        check('control_stance_elbows_clamped',
              abs(_arm(st, 'LElbowRoll') + 0.0349066) < 1e-9
              and abs(_arm(st, 'RElbowRoll') - 0.0349066) < 1e-9)
        g = node.gait_target(0.25)  # sin(2*pi*0.25)=1
        check('control_wave_legacy_sign',
              abs(_arm(g, 'LShoulderPitch') + 0.08) < 1e-9
              and abs(_arm(g, 'RShoulderPitch') - 0.08) < 1e-9,
              (_arm(g, 'LShoulderPitch'), _arm(g, 'RShoulderPitch')))
        g2 = node.gait_target(0.75)  # sin=-1
        check('control_wave_legacy_sign_neg',
              abs(_arm(g2, 'LShoulderPitch') - 0.08) < 1e-9
              and abs(_arm(g2, 'RShoulderPitch') + 0.08) < 1e-9)
    with_node(cfg, case)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('Task 4 Phase 4E offline arm-swing tests (no simulation)')
    test_stance()
    test_swing()
    test_zero_swing_keeps_neutral()
    test_phase_offset()
    test_control_compat()
    test_invalid_params()
    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\narm-swing tests: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w') as fh:
            json.dump({'passed': passed, 'total': total, 'results': RESULTS},
                      fh, indent=2, sort_keys=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
