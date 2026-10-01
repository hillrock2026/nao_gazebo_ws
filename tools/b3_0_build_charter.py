#!/usr/bin/env python3
"""Gate B3.0 — build the frozen B3 charter (hard gates, candidates, run order,
selection rules). Pure Python; reads the frozen B2 charter read-only and writes
only into docs/task4_phase4e_pathb_b3_evidence/b3_0/.

Run order and candidate values are chosen from Phase 4B/4C design evidence only
(see docs/task4_phase4b_evidence/gait_design.md); no B3 result exists at build
time. After this runs, the four JSON files are frozen by source_test_freeze and
SHA256SUMS.
"""

import hashlib
import json
import os

_TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(_TOOLS)
B2 = os.path.join(WS, 'docs', 'task4_phase4e_pathb_evidence', 'charter')
OUT = os.path.join(WS, 'docs', 'task4_phase4e_pathb_b3_evidence', 'b3_0')

SEARCH_PARAMETERS = ['step_length_m', 'step_frequency_hz', 'leg_extension_m',
                     'sway_amplitude_rad', 'foot_pitch_swing_rad']
IMMUTABLE_ARM_FIELDS = ['arm_neutral_pitch_rad', 'arm_neutral_roll_rad',
                        'arm_swing_rad', 'arm_phase_offset_rad', 'arm_swing_sign',
                        'elbow_bend_rad']

# Frozen candidate values with pre-registered, evidence-based reasons. No B3
# result was known when these were chosen.
SINGLE_FACTOR = [
    ('B3-SL-0.03', 'step_length_m', 0.03,
     'Phase 4B design: producer conservative default 0.030 m; +50% of the B2 '
     '0.02 m anchor but far below max_step_length 0.08 m and the 0.03 m forward '
     'gate margin; stance excursion scales linearly with step length.'),
    ('B3-SL-0.04', 'step_length_m', 0.04,
     'Phase 4B design allows up to 0.08 m; 0.04 m doubles the anchor stride to '
     'more directly target the 0.08 m net-forward gate while staying in the '
     'validated low-speed regime.'),
    ('B3-SF-0.65', 'step_frequency_hz', 0.65,
     'Between B2 0.5 Hz and the producer default 0.8 Hz; increases the number '
     'of stance cycles inside the fixed 18 s duration by 30%.'),
    ('B3-SF-0.80', 'step_frequency_hz', 0.80,
     'Producer documented default 0.8 Hz, already exercised in Phase 4B '
     'validation; within [0.2, 2.0] Hz.'),
    ('B3-LE-0.24001', 'leg_extension_m', 0.24001,
     '4 mm more knee bend than the anchor (0.24401); lowers the CoM and keeps '
     'further margin from the straight-leg singularity (L_MAX=0.24801).'),
    ('B3-LE-0.24801', 'leg_extension_m', 0.24801,
     'Kinematic maximum hip-to-sole extension L_THIGH+L_SHIN+L_SOLE; highest '
     'CoM / straightest stance allowed by the model.'),
    ('B3-SW-0.05', 'sway_amplitude_rad', 0.05,
     'Slightly below the anchor 0.06; reduces lateral pelvis travel to test '
     'whether less weight shift improves tracking within [0, 0.35].'),
    ('B3-SW-0.09', 'sway_amplitude_rad', 0.09,
     '+50% of anchor lateral transfer; more decisive pelvis shift toward the '
     'stance foot, well within [0, 0.35].'),
    ('B3-FP-0.05', 'foot_pitch_swing_rad', 0.05,
     'Half the anchor swing-foot pitch; less ankle excursion in swing '
     '(psi = foot_pitch_swing*sin(pi*p)) within [0, 0.4].'),
    ('B3-FP-0.15', 'foot_pitch_swing_rad', 0.15,
     '+50% swing-foot pitch; more toe-clearance / roll-off in swing while '
     'within the validated [0, 0.4] range.'),
]

CONSERVATIVE = {
    'B3-CONS-1': {
        'step_length_m': 0.03,
        'step_frequency_hz': 0.65,
        'leg_extension_m': 0.24201,
        'sway_amplitude_rad': 0.08,
        'foot_pitch_swing_rad': 0.12,
    }
}
CONSERVATIVE_REASON = (
    'Pre-registered conservative combination: mild forward gain (step_length '
    '0.03, frequency 0.65) with stability hedges (leg_extension 0.24201, lower '
    'CoM), a modest increase in lateral transfer (sway 0.08) and swing-foot '
    'pitch (0.12) for clearance. All five values are within the producer '
    'validation ranges and only the five authorized fields change.')

RUN_ORDER = [
    'B3-ANCHOR',
    'B3-SL-0.03',
    'B3-SF-0.80',
    'B3-LE-0.24001',
    'B3-SW-0.09',
    'B3-FP-0.05',
    'B3-SF-0.65',
    'B3-SL-0.04',
    'B3-FP-0.15',
    'B3-LE-0.24801',
    'B3-SW-0.05',
    'B3-CONS-1',
]


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def param_args(cfg):
    args = []
    for k in sorted(cfg):
        v = cfg[k]
        if isinstance(v, bool):
            v = 'true' if v else 'false'
        if k.startswith('actuator_') or k.startswith('phase4b_'):
            name = k
        else:
            name = 'phase4b_' + k
        args.append('%s:=%s' % (name, v))
    return args


def main():
    frozen = json.load(open(os.path.join(B2, 'frozen_configs.json')))
    thresholds = json.load(open(os.path.join(B2, 'thresholds.json')))
    anchor = dict(frozen['NATURAL-SWING'])
    anchor['foot_pitch_swing_rad'] = 0.10

    candidates = []
    candidates.append({
        'id': 'B3-ANCHOR', 'kind': 'anchor', 'base': 'NATURAL-SWING',
        'overrides': {}, 'config': dict(anchor),
        'value_reasons': {}, 'launch_args': param_args(anchor)})

    for cid, field, value, reason in SINGLE_FACTOR:
        cfg = dict(anchor)
        cfg[field] = value
        candidates.append({
            'id': cid, 'kind': 'single-factor', 'base': 'NATURAL-SWING',
            'overrides': {field: value}, 'config': cfg,
            'value_reasons': {field: reason}, 'launch_args': param_args(cfg)})

    for cid, overrides in CONSERVATIVE.items():
        cfg = dict(anchor)
        cfg.update(overrides)
        candidates.append({
            'id': cid, 'kind': 'conservative-combination',
            'base': 'NATURAL-SWING', 'overrides': overrides, 'config': cfg,
            'value_reasons': {k: CONSERVATIVE_REASON for k in overrides},
            'launch_args': param_args(cfg)})

    cand_doc = {
        'note': ('Exactly 12 frozen B3.1 candidates. Values pre-registered from '
                 'Phase 4B/4C evidence only. Built ' +
                 'before any B3 run; must not change afterwards.'),
        'anchor_id': 'B3-ANCHOR',
        'anchor_source': 'B2 frozen_configs.json NATURAL-SWING + explicit '
                         'foot_pitch_swing_rad=0.10',
        'search_parameters': SEARCH_PARAMETERS,
        'immutable_arm_fields': IMMUTABLE_ARM_FIELDS,
        'foot_pitch_swing_rad_default': 0.10,
        'count': len(candidates),
        'candidates': candidates,
    }
    order_doc = {
        'note': 'Fixed B3.1 run order, frozen before any result is known; each '
                'of the 12 IDs appears exactly once.',
        'order': RUN_ORDER,
    }
    selection_doc = {
        'note': 'Frozen B3.2 selection rules; no winner pre-selected.',
        'eligibility': 'Only a candidate whose single B3.1 screening run passes '
                       'every frozen hard gate is eligible.',
        'max_selected': 2,
        'screening_runs_count_toward_repeats': False,
        'repeat_runs_per_selected': 3,
        'repeat_pass_rule': 'Each selected candidate must pass all hard gates in '
                            'all 3 fresh repeats (3/3).',
        'no_aggregate_override': 'Means, best values or any aggregate statistic '
                                 'never override a single FAIL.',
        'tie_break': [
            '1. smaller max |yaw drift| (deg) in the screening run',
            '2. larger body_net_forward_m (m) in the screening run',
            '3. smaller max support-foot x drift (m) in the screening run',
            '4. lexicographically smaller candidate id',
        ],
        'tie_break_note': 'Deterministic and pre-registered; must not change '
                          'after seeing results.',
        'stop_condition': 'If no candidate reaches 3/3 fresh passes, stop and '
                          'emit: Gate B3: NO SOLUTION WITHIN AUTHORIZED SEARCH '
                          'SPACE',
        'termination_text': 'Gate B3: NO SOLUTION WITHIN AUTHORIZED SEARCH SPACE',
    }
    gate_names = [
        'Phase 4B body_net_forward_m >= 0.08 m',
        'B2 steady dx > stand baseline + 0.03 m',
        'max |unwrap(yaw) - window-start yaw| <= 5 deg',
        '|steady dy| <= 0.05 m',
        'steady max |roll| <= 15 deg',
        'steady max |pitch| <= 15 deg',
        'steady min body z > 0.25 m',
        'max support-foot interval x drift <= 0.03 m',
        'contact alternation count >= 3',
        'strict-alternation fraction >= 0.6',
        'command rate |Hz - 100| <= 3',
        'command reject count == 0',
        'command timestamps strictly increasing',
        'command/state finite and within canonical joint limits',
        'IMU nonfinite == 0',
        'run config exactly matches the frozen candidate config',
        'natural arms down, correct amplitude, antiphase, leg coordination and '
        'elbow bend',
    ]
    hard_doc = {
        'note': 'Frozen B3 hard gates (verbatim list) plus machine mappings. '
                'Inherited exactly from the frozen B2 charter '
                '(charter/thresholds.json); the body_net_forward_m gate is new.',
        'source_b2_thresholds_sha256':
            sha256_file(os.path.join(B2, 'thresholds.json')),
        'gate_list': gate_names,
        'steady_window_rule': thresholds['steady_window_rule'],
        'stand_window_rule': thresholds['stand_window_rule'],
        'inherited_hard_gates': thresholds['hard_gates'],
        'new_hard_gates': {
            'body_net_forward_m': {
                'rule': 'body_net_forward_m >= 0.08 (inclusive)',
                'implementation_gate': 'body_net_forward_ge_0.08',
                'data_source': "run trace/body_pose_trace.jsonl field 'x'",
                'window': 'full recorded body trace, first sample to last sample',
                'positive_direction': '+x forward (world frame)',
                'endpoints': 'first and last sample both included',
                'min_samples': 2,
                'fail_policy': 'missing trace / non-finite / < 2 samples => FAIL',
                'source': 'Phase 4B walking threshold min_net_displacement_m '
                          '(phase4b_trace.py body_net_forward_m = xs[-1]-xs[0])',
            }
        },
        'no_relaxation': 'forward/yaw and all other frozen gates are unchanged; '
                         'no mean may override a single FAIL.',
    }

    os.makedirs(OUT, exist_ok=True)
    for name, doc in (('candidates.json', cand_doc),
                      ('run_order.json', order_doc),
                      ('selection_rules.json', selection_doc),
                      ('hard_gates.json', hard_doc)):
        with open(os.path.join(OUT, name), 'w') as fh:
            json.dump(doc, fh, indent=2, sort_keys=True)
    print('wrote candidates/run_order/selection_rules/hard_gates to %s' % OUT)
    print('candidate count = %d ; run order length = %d'
          % (len(candidates), len(RUN_ORDER)))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
