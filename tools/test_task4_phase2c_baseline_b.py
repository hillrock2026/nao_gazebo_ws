#!/usr/bin/env python3
# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Phase 2C Baseline B OFFLINE tests (no Gazebo, no ROS).

Covers the planner invariants and the safety gates:
  * absolute values derived from the validated anchor (multiplier kept only as
    provenance);
  * controller-unsupported parameters (torso_pitch_rad) refused, never clamped;
  * out-of-range candidates refused (no silent clamp);
  * univariate smoke plan shape and provenance;
  * mixed 40/40/20 ratio;
  * root-overwrite refusal and per-round root isolation.

Usage: python3 tools/test_task4_phase2c_baseline_b.py [--json-out F]
"""

import argparse
import json
import os
import shutil
import sys
import tempfile
import types

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import phase2c_baseline_b as C  # noqa: E402
import run_task4_phase2b_baseline_a as R  # noqa: E402
import run_task4_phase2c_baseline_b as RC  # noqa: E402

_RESULTS = []


def check(name, condition, detail=None):
    _RESULTS.append({'name': name, 'pass': bool(condition), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if condition else 'FAIL', name,
                           '' if condition else ' :: %s' % (detail,)))


def test_anchor_and_supported():
    anchor, source = C.load_anchor()
    check('anchor_step_default', anchor['step_length_m'] == 0.02, anchor)
    check('anchor_freq_default', anchor['step_frequency_hz'] == 0.5)
    check('anchor_source_has_sha',
          len(source['sha256']) == 64 and 'candidates.json' in source['path'],
          source)
    supported = C.controller_supported_params()
    for p in ('step_length_m', 'step_frequency_hz', 'swing_height_m',
              'double_support_ratio'):
        check('controller_supports_%s' % p, p in supported)
    check('controller_lacks_torso_pitch', 'torso_pitch_rad' not in supported,
          'torso_pitch_rad' in supported)


def test_validate_override():
    supported = C.controller_supported_params()
    check('override_ok',
          C.validate_override('step_length_m', 0.02, supported) == [])
    check('override_out_of_range',
          C.validate_override('step_length_m', 0.5, supported)[0]['reason']
          == 'out_of_controller_range')
    check('override_nonfinite',
          C.validate_override('step_length_m', float('nan'), supported)[0]
          ['reason'] == 'not_finite_number')
    check('override_bool_rejected',
          C.validate_override('step_length_m', True, supported)[0]['reason']
          == 'not_finite_number')
    check('override_unsupported',
          C.validate_override('torso_pitch_rad', 0.05, supported)[0]['reason']
          == 'unsupported_parameter')
    # double_support is strictly < 0.6
    check('override_ds_exclusive_bound',
          C.validate_override('double_support_ratio', 0.6, supported)[0]
          ['reason'] == 'out_of_controller_range')


def test_univariate_candidates():
    anchor, _ = C.load_anchor()
    supported = C.controller_supported_params()
    cands = C.build_univariate_candidates(anchor, supported)
    sl = {c['candidate_id']: c for c in cands if c['factor'] == 'step_length'}
    check('univariate_sl_count', len(sl) == 5, len(sl))
    check('univariate_sl_absolute',
          abs(sl['step_length_0.5x']['absolute']['step_length_m'] - 0.01) < 1e-12
          and abs(sl['step_length_1.4x']['absolute']['step_length_m']
                  - 0.028) < 1e-12, sl['step_length_1.4x']['absolute'])
    check('univariate_sl_multiplier_kept',
          sl['step_length_1.4x']['multipliers']['step_length_m'] == 1.4)
    fr = [c for c in cands if c['factor'] == 'step_frequency']
    check('univariate_freq_count', len(fr) == 5, len(fr))
    check('univariate_freq_absolute',
          any(abs(c['absolute']['step_frequency_hz'] - 0.65) < 1e-12 for c in fr))
    sw = [c for c in cands if c['factor'] == 'swing_height']
    check('univariate_swing_count', len(sw) == 3)
    ds = [c for c in cands if c['factor'] == 'double_support_ratio']
    check('univariate_ds_count', len(ds) == 3)
    check('univariate_ds_absolute',
          any(abs(c['absolute']['double_support_ratio'] - 0.36) < 1e-12
              for c in ds))
    tp = [c for c in cands if c['factor'] == 'torso_pitch']
    check('torso_pitch_candidates_all_refused',
          len(tp) == 3 and all(not c['supported'] for c in tp))
    check('torso_pitch_radians_recorded',
          abs([c for c in tp if c['angle_deg'] == 3.0][0]
              ['absolute']['torso_pitch_rad'] - 0.05235987755982989) < 1e-12)


def test_no_silent_clamp():
    anchor, _ = C.load_anchor()
    supported = C.controller_supported_params()
    cand = C._candidate('huge', 'univariate', 'step_length',
                        {'step_length_m': 5.0}, anchor, supported)
    check('no_silent_clamp_refused', not cand['supported']
          and cand['problems'][0]['reason'] == 'out_of_controller_range',
          cand)
    check('no_silent_clamp_value_unchanged',
          abs(cand['absolute']['step_length_m'] - 0.1) < 1e-12,
          cand['absolute'])


def test_smoke_plan():
    plan = C.build_univariate_smoke_plan()
    rows = plan['rows']
    check('smoke_row_count', len(rows) == 11, len(rows))
    anchor_rows = [r for r in rows if r['factor'] == 'anchor']
    check('smoke_anchor_repeats', len(anchor_rows) == 3)
    check('smoke_anchor_unique_seeds',
          len({r['seed'] for r in anchor_rows}) == 3)
    check('smoke_episode_ids_unique',
          len({r['episode_id'] for r in rows}) == len(rows))
    facs = {r['factor'] for r in rows}
    check('smoke_covers_factors',
          {'step_length', 'step_frequency', 'swing_height',
           'double_support_ratio'} <= facs)
    check('smoke_no_torso_rows', 'torso_pitch' not in facs)
    check('smoke_refused_torso_recorded',
          {r['candidate_id'] for r in plan['refused']} ==
          {'torso_pitch_-3deg', 'torso_pitch_+0deg', 'torso_pitch_+3deg'},
          plan['refused'])
    check('smoke_overrides_supported',
          all(r['overrides'] == {} or
              all(C.validate_override(p, v, C.controller_supported_params()) == []
                  for p, v in r['overrides'].items()) for r in rows))


def test_mixed_plan():
    plan = C.build_mixed_plan()
    ratio = plan['ratio']
    check('mixed_ratio_ok', ratio['ok'] is True, ratio)
    check('mixed_counts_8_8_4',
          ratio['counts'] == {'mixed_small': 8, 'mixed_joint': 8,
                              'mixed_boundary': 4}, ratio['counts'])
    check('mixed_all_supported', plan['counts']['refused'] == 0, plan['counts'])
    joint = [c for c in plan['candidates'] if c['kind'] == 'mixed_joint']
    check('mixed_joint_2to3_params',
          all(2 <= len(c['multipliers']) <= 3 for c in joint))
    check('mixed_absolute_recorded',
          all(c['absolute'] for c in plan['candidates']))


def test_runner_root_isolation_and_guard():
    args_u = types.SimpleNamespace(root=None, round='univariate')
    args_m = types.SimpleNamespace(root=None, round='mixed')
    check('runner_roots_distinct',
          RC._root_for(args_u) != RC._root_for(args_m)
          and RC._root_for(args_u).endswith('univariate')
          and RC._root_for(args_m).endswith('mixed'))
    check('runner_root_is_phase2c',
          'task4_phase2c_baseline_b_evidence' in RC._root_for(args_u))

    tmp = tempfile.mkdtemp(prefix='p2c_guard_')
    try:
        existing = os.path.join(tmp, 'univariate')
        os.makedirs(existing)
        marker = os.path.join(existing, 'keep.txt')
        with open(marker, 'w', encoding='utf-8') as fh:
            fh.write('KEEP')
        try:
            RC.main(['--mode', 'batch', '--round', 'univariate',
                     '--root', existing])
            refused = False
        except R.RunnerError as exc:
            refused = exc.reason == 'root_exists_refused'
        check('runner_refuses_existing_root', refused)
        check('runner_guard_untouched',
              open(marker, encoding='utf-8').read() == 'KEEP')
        check('phase2c_does_not_touch_2b_evidence',
              'task4_phase2b_baseline_a_evidence' not in RC.DEFAULT_ROOT_BASE
              and 'task4_phase2c' in RC.DEFAULT_ROOT_BASE)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('Phase 2C Baseline B OFFLINE tests (no simulation)')
    test_anchor_and_supported()
    test_validate_override()
    test_univariate_candidates()
    test_no_silent_clamp()
    test_smoke_plan()
    test_mixed_plan()
    test_runner_root_isolation_and_guard()
    passed = sum(1 for r in _RESULTS if r['pass'])
    total = len(_RESULTS)
    print('\nphase 2C baseline B offline tests: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as fh:
            json.dump({'offline': True, 'passed': passed, 'total': total,
                       'results': _RESULTS}, fh, indent=2, sort_keys=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
