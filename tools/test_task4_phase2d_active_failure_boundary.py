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
Phase 2D OFFLINE tests (no Gazebo, no ROS).

Covers the failure taxonomy, honest classification, external-push schema,
unsupported-parameter refusal, root guard and dataset integration rules.
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
import phase2d_active_failure_boundary as D  # noqa: E402
import phase2c_baseline_b as C  # noqa: E402
import phase2b_baseline_a as B  # noqa: E402
import phase2a_episode_dataset as ep  # noqa: E402
import run_task4_phase2b_baseline_a as R  # noqa: E402
import run_task4_phase2d_active_failure_boundary as RD  # noqa: E402

_RESULTS = []


def check(name, condition, detail=None):
    _RESULTS.append({'name': name, 'pass': bool(condition), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if condition else 'FAIL', name,
                           '' if condition else ' :: %s' % (detail,)))


def _base_meta(**over):
    m = {
        'episode_id': 'episode_000001',
        'controller_name': 'phase4b_producer',
        'seed': 1001,
        'param_set': {'step_length_m': 0.02, 'step_frequency_hz': 0.5,
                      'swing_height_m': 0.008, 'double_support_ratio': 0.3,
                      'sway_amplitude_rad': 0.06, 'torso_pitch_rad': 0.0},
        'duration_sec': 10.0,
        'fell': False,
        'fall_time': -1.0,
        'mean_speed_x_mps': 0.001,
        'max_torso_tilt_deg': 8.0,
        'energy_qdot': 1.0,
        'success': True,
        'usable': True,
        'failure_reason': None,
    }
    m.update(over)
    return m


def test_failure_taxonomy():
    check('ten_failure_reasons', len(D.FAILURE_REASONS) == 10)
    check('all_snake_case',
          all(r == r.lower() and ' ' not in r for r in D.FAILURE_REASONS))
    check('priority_covers_all',
          set(D.FAILURE_PRIORITY) == set(D.FAILURE_REASONS))
    check('startup_highest_priority',
          D.FAILURE_PRIORITY[0] == 'startup_instability')


def test_classification():
    # startup failure: no valid frozen run / no samples -> unusable, kept
    c = D.classify_failure(False, 0, False, {}, {})
    check('classify_startup_instability',
          c['failure_reason'] == 'startup_instability'
          and c['usable'] is False and c['success'] is False, c)
    # timeout
    c = D.classify_failure(True, 100, True,
                           {'duration_sec': 4.0, 'fell': False}, {})
    check('classify_timeout',
          c['failure_reason'] == 'timeout_or_no_progress'
          and c['usable'] is True, c)
    # fall: complete samples stay usable
    c = D.classify_failure(True, 100, False,
                           {'duration_sec': 10.0, 'fell': True,
                            'fall_time': 6.0}, {})
    check('classify_fall', c['failure_reason'] == 'fall'
          and c['usable'] is True and c['success'] is False, c)
    # lateral instability (no fall)
    c = D.classify_failure(True, 100, False,
                           {'duration_sec': 10.0, 'fell': False,
                            'max_torso_tilt_deg': 8.0}, {'body_dy': 0.4})
    check('classify_lateral', c['failure_reason'] == 'lateral_instability', c)
    # boundary mapping: lateral instability on a step-length boundary
    c = D.classify_failure(
        True, 100, False,
        {'duration_sec': 10.0, 'fell': False, 'max_torso_tilt_deg': 40.0},
        {}, {'step_length_m': {'expected_risk': 'excessive_step_length'}})
    check('classify_boundary_maps_excessive',
          c['failure_reason'] == 'excessive_step_length', c)
    # contact anomaly
    c = D.classify_failure(True, 100, False,
                           {'duration_sec': 10.0, 'fell': False},
                           {'alternation': {'ok': False, 'n_support': 0}})
    check('classify_contact_anomaly',
          c['failure_reason'] == 'contact_anomaly', c)
    # success
    c = D.classify_failure(True, 100, False,
                           {'duration_sec': 10.0, 'fell': False,
                            'max_torso_tilt_deg': 8.0}, {})
    check('classify_success',
          c['success'] is True and c['failure_reason'] is None
          and c['usable'] is True, c)
    # never emits a reason outside the enum
    check('classify_enum_only',
          all(c['failure_reason'] is None or
              c['failure_reason'] in D.FAILURE_REASONS for c in [c]))


def test_failure_metadata_rules():
    # success=false requires a non-blank reason (Phase 2A rule)
    check('fail_requires_reason',
          any(e['reason'] == 'failure_requires_reason'
              for e in ep.validate_metadata(
                  _base_meta(success=False, usable=True, failure_reason=None))))
    check('fail_blank_reason_rejected',
          any(e['field'] == 'failure_reason'
              for e in ep.validate_metadata(
                  _base_meta(success=False, usable=True, failure_reason='   '))))
    # failed but complete samples -> usable=true is legal
    check('failed_but_usable_ok',
          ep.validate_metadata(
              _base_meta(success=False, usable=True,
                         failure_reason='fall')) == [])
    # success cannot coexist with a fall
    check('success_forbids_fell',
          any(e['reason'] == 'success_forbids_fell'
              for e in ep.validate_metadata(
                  _base_meta(success=True, fell=True, fall_time=3.0,
                             failure_reason=None))))
    # success cannot carry a failure reason
    check('success_null_reason',
          any(e['reason'] == 'success_requires_null'
              for e in ep.validate_metadata(
                  _base_meta(success=True, failure_reason='fall'))))
    # failure_detail secondary reasons must not override the primary
    meta = _base_meta(success=False, usable=True, failure_reason='fall',
                      failure_detail={'secondary_reasons':
                                      ['lateral_instability'],
                                      'diagnostics': {'body_dy_m': 0.4}})
    check('failure_detail_recorded', ep.validate_metadata(meta) == [])


def test_boundary_candidates():
    anchor, _ = C.load_anchor()
    supported = C.controller_supported_params()
    cands = {c['candidate_id']: c for c in D.build_boundary_candidates(
        anchor, supported)}
    check('boundary_step_length_absolute',
          abs(cands['boundary_step_length_1p4x']['absolute']
              - 0.028) < 1e-12, cands['boundary_step_length_1p4x'])
    check('boundary_frequency_absolute',
          abs(cands['boundary_frequency_1p3x']['absolute']
              - 0.65) < 1e-12)
    check('boundary_swing_absolute',
          abs(cands['boundary_swing_0p7x']['absolute'] - 0.0056) < 1e-12)
    check('boundary_ds_low_absolute',
          abs(cands['boundary_ds_low']['absolute'] - 0.05) < 1e-12)
    check('boundary_probes_supported',
          cands['probe_step_length_max']['supported']
          and cands['probe_frequency_high']['supported']
          and cands['probe_swing_min']['supported'])
    check('boundary_probes_in_range',
          C.validate_override('step_length_m',
                              cands['probe_step_length_max']['absolute'],
                              supported) == []
          and C.validate_override('step_frequency_hz',
                                  cands['probe_frequency_high']['absolute'],
                                  supported) == [])
    # torso_pitch explicitly refused
    tp = cands['boundary_torso_pitch_p6deg']
    check('torso_pitch_refused',
          not tp['supported']
          and tp['problems'][0]['reason'] == 'unsupported_parameter', tp)


def test_plan_uniqueness_and_readonly():
    plan = D.boundary_plan()
    rows = plan['rows']
    check('plan_rows_8', len(rows) == 8, len(rows))
    check('plan_episode_ids_unique',
          len({r['episode_id'] for r in rows}) == len(rows))
    check('plan_candidate_seed_unique',
          len({(r['candidate_id'], r['seed']) for r in rows}) == len(rows))
    check('plan_refused_torso',
          [r['candidate_id'] for r in plan['refused']]
          == ['boundary_torso_pitch_p6deg'])
    check('plan_has_anchor_control',
          any(r['candidate_id'] == 'ANCHOR_CTRL' for r in rows))
    # plan mode must be read-only (no evidence created)
    tmp = tempfile.mkdtemp(prefix='p2d_plan_')
    try:
        target = os.path.join(tmp, 'root')
        rc = RD.main(['--mode', 'plan', '--root', target])
        check('plan_mode_exit_zero', rc == 0)
        check('plan_mode_read_only', not os.path.exists(target))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_external_push():
    check('push_none_ok', D.validate_external_push(None) == [])
    good = {'time_sec': 5.2, 'duration_sec': 0.2, 'force_n': 2.0,
            'direction': [0.0, 1.0, 0.0], 'link': 'base_link', 'frame': 'world'}
    check('push_good_ok', D.validate_external_push(good) == [])
    check('push_missing_key',
          any(p['reason'] == 'push_missing_key'
              for p in D.validate_external_push({'time_sec': 1.0})))
    check('push_bad_direction',
          any(p['reason'] == 'push_direction_invalid'
              for p in D.validate_external_push(
                  {'time_sec': 1.0, 'force_n': 1.0, 'direction': [0, 1]})))
    check('push_zero_direction',
          any(p['reason'] == 'push_direction_zero'
              for p in D.validate_external_push(
                  {'time_sec': 1.0, 'force_n': 1.0,
                   'direction': [0.0, 0.0, 0.0]})))
    plan = D.external_push_plan()
    check('push_not_supported_honestly',
          plan['supported'] is False
          and plan['status'] == 'not_covered'
          and plan['reason'] == 'no_approved_force_injection_interface', plan)


def test_coverage_matrix():
    plan = D.boundary_plan()
    matrix = D.failure_coverage_matrix(plan['rows'])
    covered = [r for r in D.FAILURE_REASONS if matrix[r]['covered_by_plan']]
    not_covered = [r for r in D.FAILURE_REASONS
                   if not matrix[r]['covered_by_plan']]
    check('coverage_has_planned', len(covered) >= 4, covered)
    check('coverage_reports_not_covered', len(not_covered) >= 1, not_covered)
    check('coverage_no_fake_trigger',
          all(matrix[r]['note'] for r in not_covered))
    check('coverage_external_push_present', 'external_push' in matrix)


def test_root_guard_and_temp():
    tmp = tempfile.mkdtemp(prefix='p2d_guard_')
    try:
        existing = os.path.join(tmp, 'boundary')
        os.makedirs(existing)
        marker = os.path.join(existing, 'keep.txt')
        with open(marker, 'w', encoding='utf-8') as fh:
            fh.write('KEEP')
        try:
            RD.main(['--mode', 'smoke', '--root', existing])
            refused = False
        except R.RunnerError as exc:
            refused = exc.reason == 'root_exists_refused'
        check('root_guard_refuses_existing', refused)
        check('root_guard_untouched',
              open(marker, encoding='utf-8').read() == 'KEEP')
        check('p2d_root_separate',
              'task4_phase2d' in RD.DEFAULT_ROOT
              and 'task4_phase2b_baseline_a_evidence' not in RD.DEFAULT_ROOT)
        # atomic json write leaves no temp file
        target = os.path.join(tmp, 'sub', 'r.json')
        R._write_json(target, {'ok': 1})
        leftovers = [n for n in os.listdir(os.path.dirname(target))
                     if n.startswith('.tmp_')]
        check('no_temp_leftover', not leftovers, leftovers)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_aggregate_consistency():
    labels = {'mean_speed_x_mps': 0.0, 'max_torso_tilt_deg': 5.0,
              'duration_sec': 10.0, 'energy_qdot': 1.0, 'fall_time': -1.0,
              'success': True, 'fell': False, 'usable': True,
              'failure_reason': None}
    ep_ok = {'episode_id': 'episode_000001', 'config_id': 'c1', 'seed': 1,
             'step_length_m': 0.02, 'frequency_hz': 0.5, 'labels': labels}
    labels_fail = dict(labels, success=False, usable=True,
                       failure_reason='fall', fell=True, fall_time=5.0)
    ep_fail = {'episode_id': 'episode_000002', 'config_id': 'c2', 'seed': 2,
               'step_length_m': 0.02, 'frequency_hz': 0.5, 'labels': labels_fail}
    quality = B.quality_report([ep_ok, ep_fail], {'train': 2}, [1, 2])
    per_episode = [
        {'episode_id': 'episode_000001', 'candidate_id': 'c1', 'usable': True,
         'actual_failure_reason': None},
        {'episode_id': 'episode_000002', 'candidate_id': 'c2', 'usable': True,
         'actual_failure_reason': 'fall'},
    ]
    check('aggregate_consistent',
          RD.check_aggregate_consistency(per_episode, quality) == [])
    check('aggregate_count_8_style',
          quality['overall']['usable_count'] == 2)
    # a deliberately inconsistent aggregate is detected
    bad = json.loads(json.dumps(quality))
    bad['overall']['usable_count'] = 1
    problems = RD.check_aggregate_consistency(per_episode, bad)
    check('aggregate_mismatch_detected',
          any(p['reason'] == 'overall_usable_mismatch' for p in problems),
          problems)

    counts = RD.failure_counts_from_per_episode(per_episode)
    check('failure_counts_none_bucket', counts == {'none': 1, 'fall': 1},
          counts)
    counts2 = RD.failure_counts_from_per_episode(
        [{'actual_failure_reason': None},
         {'actual_failure_reason': 'unknown_custom_reason'}])
    check('failure_counts_unknown_reason_no_crash',
          counts2 == {'none': 1, 'unknown_custom_reason': 1}, counts2)


def _result(eid, candidate, ok, usable, success, reason, labels):
    return {
        'episode_id': eid, 'ok': ok, 'seed': 1,
        'run_cfg': {'step_length_m': 0.02, 'step_frequency_hz': 0.5},
        'labels': labels,
        'phase2d': {
            'candidate_id': candidate, 'seed': 1,
            'requested_param_set': {}, 'applied_param_set': {},
            'expected_risk': None, 'external_push': None,
            'classification': {'success': success, 'usable': usable,
                               'failure_reason': reason,
                               'secondary_reasons': [], 'diagnostics': {}},
        },
    }


def test_write_report_reconciles():
    tmp = tempfile.mkdtemp(prefix='p2d_reconcile_')
    try:
        labels_ok = {'mean_speed_x_mps': 0.0, 'max_torso_tilt_deg': 5.0,
                     'duration_sec': 10.0, 'energy_qdot': 1.0,
                     'fall_time': -1.0, 'success': True, 'fell': False,
                     'usable': True, 'failure_reason': None}
        # stale Phase-2B label says usable=False for a failed-but-complete run;
        # the Phase 2D classification says usable=True and must win.
        labels_fail_stale = dict(labels_ok, success=False, fell=True,
                                 fall_time=5.0, usable=False,
                                 failure_reason='fall')
        results = [
            _result('episode_000001', 'c1', True, True, True, None, labels_ok),
            _result('episode_000002', 'c2', True, True, False, 'fall',
                    labels_fail_stale),
        ]
        report = RD.write_report(tmp, [], results, [], {}, None)
        check('reconcile_per_episode_usable',
              all(e['usable'] for e in report['per_episode']))
        check('reconcile_overall_usable',
              report['quality']['overall']['usable_count'] == 2,
              report['quality']['overall'])
        check('reconcile_failure_counts',
              report['failure_reason_counts'] == {'none': 1, 'fall': 1},
              report['failure_reason_counts'])
        # report file written atomically
        check('report_written',
              os.path.isfile(os.path.join(tmp, 'quality_report.json')))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('Phase 2D active-failure/boundary OFFLINE tests (no simulation)')
    test_failure_taxonomy()
    test_classification()
    test_failure_metadata_rules()
    test_boundary_candidates()
    test_plan_uniqueness_and_readonly()
    test_external_push()
    test_coverage_matrix()
    test_root_guard_and_temp()
    test_aggregate_consistency()
    test_write_report_reconciles()
    passed = sum(1 for r in _RESULTS if r['pass'])
    total = len(_RESULTS)
    print('\nphase 2D offline tests: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as fh:
            json.dump({'offline': True, 'passed': passed, 'total': total,
                       'results': _RESULTS}, fh, indent=2, sort_keys=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
