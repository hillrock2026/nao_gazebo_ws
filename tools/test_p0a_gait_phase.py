#!/usr/bin/env python3
"""P0-a P1 explicit support/swing phase model fixtures (offline, no ROS).

Usage: python3 tools/test_p0a_gait_phase.py [--json-out F]
"""

import argparse
import ast
import json
import math
import os
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(_TOOLS)
PHASE_DIR = os.path.join(WS, 'src', 'nao_gz_sim', 'scripts')
LAUNCH = os.path.join(WS, 'src', 'nao_gz_sim', 'launch', 'nao_sim.launch.py')
if PHASE_DIR not in sys.path:
    sys.path.insert(0, PHASE_DIR)
import nao_gait_phase as G  # noqa: E402

RESULTS = []
DS = 0.3
S = G.swing_ratio(DS)
H = DS / 2.0
EPS = 1e-9


def _safe(detail):
    if isinstance(detail, (str, int, float, bool, list, tuple, dict, type(None))):
        if isinstance(detail, tuple):
            return [_safe(x) for x in detail]
        if isinstance(detail, list):
            return [_safe(x) for x in detail]
        if isinstance(detail, dict):
            return {str(k): _safe(v) for k, v in detail.items()}
        return detail
    return repr(detail)


def check(name, cond, detail=None):
    RESULTS.append({'name': name, 'pass': bool(cond), 'detail': _safe(detail)})
    print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                           '' if cond else ' :: %r' % (detail,)))


def test_coverage_and_exclusion():
    n = 20000
    last = None
    boundaries = []
    for i in range(n):
        ph = i / n
        name, prog, lr, rr = G.classify(ph, DS)
        if not (0.0 <= prog < 1.0 + 1e-12):
            check('progress_in_unit_interval', False, (ph, prog))
            break
        if last is not None and name != last:
            boundaries.append((ph, last, name))
        last = name
    names = {G.classify(i / n, DS)[0] for i in range(n)}
    check('all_four_phases_present', names == set(G.PHASE_NAMES), names)
    check('exactly_three_internal_boundaries', len(boundaries) == 3, boundaries)
    expected = [S, 0.5, 1.0 - H]
    got = [round(b[0], 6) for b in boundaries]
    check('boundaries_match_geometry',
          all(abs(a - b) < 2.0 / n for a, b in zip(got, expected)),
          (got, expected))
    check('boundary_order_correct',
          [b[2] for b in boundaries] == ['DS_TO_LEFT_SUPPORT', 'RIGHT_SWING',
                                         'DS_TO_RIGHT_SUPPORT'],
          [b[2] for b in boundaries])


def test_roles_and_symmetry():
    for ph in (0.1, 0.4, 0.6, 0.9):
        _, _, lr, rr = G.classify(ph, DS)
        _, _, lr2, rr2 = G.classify((ph + 0.5) % 1.0, DS)
        check('roles_opposite_%s' % ph, (lr, rr) == (rr2, lr2), (ph, lr, rr, lr2, rr2))
    check('left_swing_left_role', G.classify(0.1, DS)[2] == 'swing')
    check('right_swing_right_role', G.classify(0.6, DS)[3] == 'swing')
    check('ds_both_support', G.classify(0.4, DS)[2:] == ('support', 'support'))


def test_ds_validation():
    for bad in (-0.1, 0.6, 0.7, float('nan'), float('inf'), '0.2', True, None):
        try:
            G.validate_ds_ratio(bad)
            check('ds_rejects_%r' % (bad,), False, bad)
        except (ValueError, TypeError):
            check('ds_rejects_%r' % (bad,), True)
    check('ds_accepts_zero', G.validate_ds_ratio(0.0) == 0.0)
    check('ds_accepts_0599', abs(G.validate_ds_ratio(0.599) - 0.599) < 1e-12)
    try:
        G.validate_phase_mode('bogus')
        check('mode_rejects_bogus', False)
    except ValueError:
        check('mode_rejects_bogus', True)
    check('mode_accepts_legacy',
          G.validate_phase_mode('legacy') == 'legacy')


def test_phase_continuity():
    check('classify_0_eq_1', G.classify(0.0, DS)[0] == G.classify(1.0, DS)[0])
    check('sway_0_eq_1',
          abs(G.sway_profile(0.0, DS) - G.sway_profile(1.0, DS)) < 1e-9,
          (G.sway_profile(0.0, DS), G.sway_profile(1.0, DS)))
    for b in (S, 0.5, 1.0 - H):
        lo = G.sway_profile((b - EPS) % 1.0, DS)
        hi = G.sway_profile(b % 1.0, DS)
        check('sway_continuous_at_%.3f' % b, abs(lo - hi) < 1e-4, (lo, hi))


def test_foot_target_continuity_and_lift():
    params = dict(ds_ratio=DS, step=0.02, leg_ext=0.24401, swing_h=0.008,
                  foot_pitch_swing=0.10)
    for b in (S, 0.5, 1.0 - H):
        for side in ('L', 'R'):
            lo = G.leg_foot_target((b - EPS) % 1.0, side, **params)
            hi = G.leg_foot_target(b % 1.0, side, **params)
            check('foot_%s_continuous_at_%.3f' % (side, b),
                  all(abs(a - c) < 1e-3 for a, c in zip(lo, hi)), (side, b, lo, hi))
    # swing foot lifted at mid-swing, support foot flat
    mid_swing = S / 2.0
    x, z, pitch = G.leg_foot_target(mid_swing, 'L', **params)
    check('left_swing_lifted', z > -params['leg_ext'] + 0.4 * params['swing_h'],
          z)
    xr, zr, _ = G.leg_foot_target(mid_swing, 'R', **params)
    check('right_support_flat', abs(zr + params['leg_ext']) < 1e-12, zr)
    # mid-stance flat for left
    xm, zm, _ = G.leg_foot_target(0.7, 'L', **params)
    check('left_stance_flat', abs(zm + params['leg_ext']) < 1e-12, zm)


def test_legacy_foot_trajectory_equivalence():
    # Independent re-derivation of the historical Phase 4B mapping.
    step, leg_ext, swing_h, fp = 0.02, 0.24401, 0.008, 0.10
    s = 0.5 - DS / 2.0

    def smoothstep(p):
        p = max(0.0, min(1.0, p))
        return p * p * (3.0 - 2.0 * p)

    def legacy(phase, side):
        offset = 0.0 if side == 'L' else 0.5
        phi = (phase + offset) % 1.0
        if phi < s:
            p = phi / s
            return (-step / 2 + step * smoothstep(p),
                    -(leg_ext - swing_h * math.sin(math.pi * p)),
                    fp * math.sin(math.pi * p))
        p = (phi - s) / (1.0 - s)
        return (step / 2 - step * p, -leg_ext, 0.0)
    worst = 0.0
    for i in range(2001):
        ph = i / 2000.0
        for side in ('L', 'R'):
            a = G.leg_foot_target(ph, side, DS, step, leg_ext, swing_h, fp)
            b = legacy(ph, side)
            worst = max(worst, max(abs(x - y) for x, y in zip(a, b)))
    check('legacy_foot_equivalence', worst < 1e-12, worst)


def test_sway_profile_shape():
    check('sway_left_swing_plus1', G.sway_profile(0.1, DS) == 1.0)
    check('sway_right_swing_minus1', G.sway_profile(0.6, DS) == -1.0)
    # transition during DS is monotonic +1 -> -1
    vals = [G.sway_profile(S + H * i / 50.0, DS) for i in range(51)]
    check('sway_ds_monotonic_down',
          all(a >= b - 1e-12 for a, b in zip(vals, vals[1:])), vals[:3])
    check('sway_ds_endpoints', abs(vals[0] - 1.0) < 1e-9 and abs(vals[-1] + 1.0) < 1e-9,
          (vals[0], vals[-1]))
    # transition completes before swing: at start of LEFT_SWING sway is already +1
    check('sway_complete_before_left_swing', G.sway_profile(0.0, DS) == 1.0)


def test_launch_wiring():
    tree = ast.parse(open(LAUNCH).read())
    declared = None
    wired = False
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            f = node.func
            name = f.id if isinstance(f, ast.Name) else (
                f.attr if isinstance(f, ast.Attribute) else None)
            if name == 'DeclareLaunchArgument':
                n = node.args[0].value if node.args else None
                if n == 'phase4b_gait_phase_mode':
                    for kw in node.keywords:
                        if kw.arg == 'default_value':
                            declared = kw.value.value
            if name == 'LaunchConfiguration':
                if node.args and getattr(node.args[0], 'value', None) == \
                        'phase4b_gait_phase_mode':
                    wired = True
    check('launch_phase_mode_default_legacy', declared == 'legacy', declared)
    check('launch_phase_mode_wired', wired)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('P0-a P1 gait phase fixtures')
    test_coverage_and_exclusion()
    test_roles_and_symmetry()
    test_ds_validation()
    test_phase_continuity()
    test_foot_target_continuity_and_lift()
    test_legacy_foot_trajectory_equivalence()
    test_sway_profile_shape()
    test_launch_wiring()
    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\ngait phase fixtures: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w') as fh:
            json.dump({'passed': passed, 'total': total, 'results': RESULTS},
                      fh, indent=2, sort_keys=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
