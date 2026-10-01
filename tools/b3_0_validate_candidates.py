#!/usr/bin/env python3
"""Gate B3.0 — independent validator for the frozen B3 candidate table.

Checks candidates.json / run_order.json programmatically:
  * exactly 12 candidates: 1 anchor, 10 single-factor, 1 conservative combo;
  * each single-factor changes exactly one authorized search field;
  * each authorized search field appears in exactly two single-factor entries;
  * no unauthorized field changes vs the anchor (arm/controller/physics fixed);
  * foot_pitch_swing_rad is explicit in every candidate config;
  * candidate IDs and run_order.json correspond one-to-one.

It then performs a no-Gazebo static reachability check: for every candidate it
evaluates the producer's *own* gait math (imported read-only; no ROS node is
created and rclpy is never initialised) across a phase grid and asserts that all
24 joint targets stay inside the canonical limits, and that all parameter values
lie in the producer's documented validation ranges.

Usage: python3.12 tools/b3_0_validate_candidates.py
Exit 0 = all checks pass.
"""

import importlib.util
import json
import os
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(_TOOLS)
B3 = os.path.join(WS, 'docs', 'task4_phase4e_pathb_b3_evidence', 'b3_0')
PRODUCER = os.path.join(WS, 'src', 'nao_gz_sim', 'scripts', 'phase4b_producer.py')

SEARCH_PARAMETERS = ['step_length_m', 'step_frequency_hz', 'leg_extension_m',
                     'sway_amplitude_rad', 'foot_pitch_swing_rad']
IMMUTABLE_ARM_FIELDS = ['arm_neutral_pitch_rad', 'arm_neutral_roll_rad',
                        'arm_swing_rad', 'arm_phase_offset_rad', 'arm_swing_sign',
                        'elbow_bend_rad']

RESULTS = []


def check(name, cond, detail=None):
    RESULTS.append({'name': name, 'pass': bool(cond), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                           '' if cond else ' :: %s' % (detail,)))


# B3.0 validates the historical (frozen) gait scheduling only. The producer's
# P0-a phase-mode switch defaults to 'legacy'; this validator pins that value
# explicitly so the stub and the production method contract stay unambiguous
# (no getattr/fallback in production code).
VALIDATOR_PHASE_MODE = 'legacy'


def load_producer():
    # Make the producer's sibling pure module (nao_gait_phase) importable when
    # the producer is loaded directly from its source path.
    scripts = os.path.dirname(PRODUCER)
    if scripts not in sys.path:
        sys.path.insert(0, scripts)
    spec = importlib.util.spec_from_file_location('phase4b_producer', PRODUCER)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)   # imports rclpy only; no rclpy.init()
    return mod


class _Dummy:
    pass


def static_reachability(mod, cfg):
    """Evaluate the producer's real gait_target over a phase grid."""
    d = _Dummy()
    d.leg_ext = float(cfg['leg_extension_m'])
    d.arm_neutral_pitch = float(cfg['arm_neutral_pitch_rad'])
    d.arm_neutral_roll = float(cfg['arm_neutral_roll_rad'])
    d.elbow_bend = float(cfg['elbow_bend_rad'])
    d.step = float(cfg['step_length_m'])
    d.freq = float(cfg['step_frequency_hz'])
    d.swing_h = float(cfg['swing_height_m'])
    d.ds_ratio = float(cfg['double_support_ratio'])
    d.sway_amp = float(cfg['sway_amplitude_rad'])
    d.arm_swing = float(cfg['arm_swing_rad'])
    d.arm_swing_sign = float(cfg['arm_swing_sign'])
    d.arm_phase_offset = float(cfg['arm_phase_offset_rad'])
    d.foot_pitch_swing = float(cfg['foot_pitch_swing_rad'])
    d.direction_sign = 1.0
    d.sway_sign = float(cfg.get('sway_sign', 1.0))
    d.phase_mode = VALIDATOR_PHASE_MODE
    d.stance_cmd = mod.Phase4bProducer._make_stance(d)
    violations = []
    for k in range(0, 200):
        phase = k / 200.0
        target = mod.Phase4bProducer.gait_target(d, phase)
        for name, value in zip(mod.JOINT_ORDER, target):
            lo, hi = mod.LIMITS[name]
            if value < lo - 1e-9 or value > hi + 1e-9:
                violations.append({'phase': phase, 'joint': name,
                                   'value': value, 'limit': [lo, hi]})
    return violations


PRODUCER_RANGES = {
    'step_length_m': (0.0, 0.08),
    'step_frequency_hz': (0.2, 2.0),
    'leg_extension_m': (0.15, 0.24801),
    'sway_amplitude_rad': (0.0, 0.35),
    'foot_pitch_swing_rad': (0.0, 0.4),
}


def main():
    cand = json.load(open(os.path.join(B3, 'candidates.json')))
    order = json.load(open(os.path.join(B3, 'run_order.json')))
    cs = cand['candidates']

    check('candidate_count_12', len(cs) == 12, len(cs))
    kinds = [c['kind'] for c in cs]
    check('one_anchor', kinds.count('anchor') == 1, kinds)
    check('ten_single_factor', kinds.count('single-factor') == 10, kinds)
    check('one_conservative', kinds.count('conservative-combination') == 1, kinds)

    anchor = [c for c in cs if c['kind'] == 'anchor'][0]
    anchor_cfg = anchor['config']
    check('anchor_foot_pitch_0.10',
          anchor_cfg.get('foot_pitch_swing_rad') == 0.10,
          anchor_cfg.get('foot_pitch_swing_rad'))

    # every candidate: only authorized fields differ from the anchor
    for c in cs:
        changed = {k for k in set(c['config']) | set(anchor_cfg)
                   if c['config'].get(k) != anchor_cfg.get(k)}
        if c['kind'] == 'anchor':
            check('anchor_no_change_%s' % c['id'], not changed, changed)
        elif c['kind'] == 'single-factor':
            check('single_factor_one_field_%s' % c['id'],
                  len(changed) == 1 and changed <= set(SEARCH_PARAMETERS),
                  changed)
        else:
            check('conservative_only_search_fields_%s' % c['id'],
                  changed <= set(SEARCH_PARAMETERS), changed)
        check('foot_pitch_explicit_%s' % c['id'],
              'foot_pitch_swing_rad' in c['config'],
              None)
        for f in IMMUTABLE_ARM_FIELDS:
            check('arm_field_frozen_%s_%s' % (c['id'], f),
                  c['config'].get(f) == anchor_cfg.get(f),
                  (c['config'].get(f), anchor_cfg.get(f)))

    # exactly two single-factor candidates per authorized field
    per_field = {p: 0 for p in SEARCH_PARAMETERS}
    for c in cs:
        if c['kind'] != 'single-factor':
            continue
        field = list(c['overrides'])[0]
        per_field[field] = per_field.get(field, 0) + 1
    check('two_single_factors_per_field',
          per_field == {p: 2 for p in SEARCH_PARAMETERS}, per_field)

    # IDs <-> run order one-to-one
    ids = [c['id'] for c in cs]
    check('ids_unique', len(set(ids)) == len(ids), ids)
    check('run_order_matches_ids_one_to_one',
          sorted(order['order']) == sorted(ids) and len(order['order']) == 12,
          order['order'])

    # parameter ranges
    for c in cs:
        for p, (lo, hi) in PRODUCER_RANGES.items():
            v = c['config'][p]
            check('range_%s_%s' % (c['id'], p),
                  isinstance(v, (int, float)) and lo <= v <= hi, (p, v))

    # explicit declaration: this validator exercises the frozen legacy schedule
    check('validator_phase_mode_legacy', VALIDATOR_PHASE_MODE == 'legacy',
          VALIDATOR_PHASE_MODE)

    # static joint-limit reachability using the producer's own gait math
    mod = load_producer()
    for c in cs:
        v = static_reachability(mod, c['config'])
        check('joint_limits_%s' % c['id'], not v, v[:3])

    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\ncandidate validation: %d/%d passed' % (passed, total))
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
