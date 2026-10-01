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
Phase 2B Baseline A OFFLINE tests (no Gazebo, no ROS launch).

Covers the pure planning/derivation logic and the honest-failure behaviour:
  * the authoritative default is read from the frozen B3.0 anchor;
  * A0/A1/A2 absolute step lengths and the fixed 1.0x frequency;
  * the fixed, reproducible seed list and sampling plan;
  * measured energy/fall/label derivation, including explicit "missing" instead
    of fabricated defaults;
  * quality report uniqueness/speed ordering.

Usage: python3 tools/test_task4_phase2b_baseline_a.py [--json-out F]
Exit 0 = all offline tests passed.
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
import phase2b_baseline_a as B  # noqa: E402
import run_task4_phase2b_baseline_a as R  # noqa: E402

_RESULTS = []


def check(name, condition, detail=None):
    _RESULTS.append({'name': name, 'pass': bool(condition), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if condition else 'FAIL', name,
                           '' if condition else ' :: %s' % (detail,)))


def test_defaults_and_plan():
    anchor, source = B.load_anchor_config()
    check('anchor_default_step', anchor['step_length_m'] == 0.02, anchor)
    check('anchor_default_freq', anchor['step_frequency_hz'] == 0.5, anchor)
    check('anchor_source_named', 'candidates.json' in source, source)
    plan = B.derive_config_plan(anchor)
    by_id = {p['config_id']: p for p in plan}
    check('plan_three_configs', sorted(by_id) == ['A0', 'A1', 'A2'])
    check('plan_A0', by_id['A0']['step_length_m'] == 0.01
          and by_id['A0']['frequency_hz'] == 0.5, by_id['A0'])
    check('plan_A1', by_id['A1']['step_length_m'] == 0.02, by_id['A1'])
    check('plan_A2', abs(by_id['A2']['step_length_m'] - 0.024) < 1e-12,
          by_id['A2'])
    check('plan_freq_all_1x',
          all(p['frequency_hz'] == 0.5 for p in plan))
    check('plan_multiplier_recorded',
          [by_id[c]['step_length_multiplier'] for c in ('A0', 'A1', 'A2')]
          == [0.5, 1.0, 1.2])
    check('plan_absolute_not_ratio',
          all(isinstance(p['step_length_m'], float) for p in plan))


def test_seeds_and_sampling():
    check('seed_list_fixed', B.fixed_seed_list(3) == [101, 202, 303])
    check('seed_list_repeatable', B.fixed_seed_list(3) == B.fixed_seed_list(3))
    check('seed_list_int', all(isinstance(s, int) for s in B.fixed_seed_list(5)))
    for bad in (0, len(B.SEED_LIST) + 1):
        try:
            B.fixed_seed_list(bad)
            ok = False
        except B.BaselineAError:
            ok = True
        check('seed_list_rejects_%d' % bad, ok)
    plan = B.sampling_plan(3)
    rows = plan['rows']
    check('sampling_count', len(rows) == 9, len(rows))
    check('sampling_episode_ids_unique',
          len({r['episode_id'] for r in rows}) == 9)
    check('sampling_config_seed_unique',
          len({(r['config_id'], r['seed']) for r in rows}) == 9)
    check('sampling_episode_id_format',
          all(r['episode_id'].startswith('episode_') for r in rows))
    check('sampling_each_config_3_seeds',
          all(sum(1 for r in rows if r['config_id'] == c) == 3
              for c in ('A0', 'A1', 'A2')))
    # documented determinism: same plan every call
    check('sampling_deterministic', B.sampling_plan(3)['rows'] == rows)


def test_energy_and_fall():
    tmp = tempfile.mkdtemp(prefix='p2b_energy_')
    try:
        path = os.path.join(tmp, 'samples.jsonl')
        with open(path, 'w', encoding='utf-8') as fh:
            for dq in ([1.0, 0.0], [0.0, 2.0]):
                fh.write(json.dumps({'joint_velocity': dq}) + '\n')
        energy, n = B.energy_qdot_from_samples(path)
        check('energy_measured', abs(energy - 2.5) < 1e-12, (energy, n))
        check('energy_count', n == 2)

        body = os.path.join(tmp, 'body_pose_trace.jsonl')
        with open(body, 'w', encoding='utf-8') as fh:
            for t, z in ((1.0, 0.33), (2.0, 0.32), (3.0, 0.31)):
                fh.write(json.dumps({'t': t, 'x': 0.0, 'y': 0.0, 'z': z}) + '\n')
        fell, fall_time, min_z = B.fall_from_body_trace(tmp, 0.5, 3.5)
        check('fall_none_detected', fell is False and fall_time is None
              and abs(min_z - 0.31) < 1e-12, (fell, fall_time, min_z))

        with open(body, 'w', encoding='utf-8') as fh:
            for t, z in ((1.0, 0.33), (2.0, 0.20), (3.0, 0.10)):
                fh.write(json.dumps({'t': t, 'x': 0.0, 'y': 0.0, 'z': z}) + '\n')
        fell, fall_time, min_z = B.fall_from_body_trace(tmp, 0.5, 3.5)
        check('fall_detected', fell is True and fall_time == 2.0
              and abs(min_z - 0.10) < 1e-12, (fell, fall_time, min_z))

        # window filter excludes out-of-window rows
        fell, fall_time, min_z = B.fall_from_body_trace(tmp, 2.5, 3.5)
        check('fall_window_filter', fell is True and fall_time == 3.0,
              (fell, fall_time, min_z))

        # missing trace -> undetermined, never a default
        empty = os.path.join(tmp, 'empty')
        os.makedirs(empty)
        fell, fall_time, min_z = B.fall_from_body_trace(empty, 0.0, 10.0)
        check('fall_missing_undetermined',
              fell is None and fall_time is None and min_z is None)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_labels_honest_failure():
    tmp = tempfile.mkdtemp(prefix='p2b_labels_')
    try:
        run = os.path.join(tmp, 'run')
        trace = os.path.join(tmp, 'trace')
        os.makedirs(run)
        os.makedirs(trace)
        # samples exist (energy measurable) but no traces -> metrics missing
        with open(os.path.join(run, 'samples.jsonl'), 'w', encoding='utf-8') as fh:
            for _ in range(3):
                fh.write(json.dumps({'joint_velocity': [0.1] * 24}) + '\n')
        summary = {
            'dataset_committed': True, 'finish_reason': 'duration_complete',
            'samples_written': 3,
            'first_written_timestamp': 2.0, 'last_written_timestamp': 11.0,
        }
        cfg = {'start_delay_sec': 0.5, 'gait_blend_sec': 1.5,
               'duration_sec': 16.0, 'stand_sec': 12.0, 'gait_method': 'walk',
               'arm_swing_rad': 0.12, 'arm_neutral_pitch_rad': 1.5,
               'elbow_bend_rad': 0.3, 'arm_swing_sign': 1}
        labels = B.derive_episode_labels(run, trace, cfg, ready=2.0,
                                         summary=summary)
        check('labels_energy_measured',
              abs(labels['energy_qdot'] - 24 * 0.01) < 1e-12,
              labels['energy_qdot'])
        check('labels_duration_measured', labels['duration_sec'] == 9.0)
        check('labels_fell_missing', 'fell' in labels['missing']
              and labels['fell'] is None)
        check('labels_speed_missing', labels['mean_speed_x_mps'] is None
              and 'mean_speed_x_mps' in labels['missing'])
        check('labels_not_success', labels['success'] is False)
        check('labels_not_usable', labels['usable'] is False)
        check('labels_failure_reason_nonempty',
              isinstance(labels['failure_reason'], str)
              and labels['failure_reason'].strip() != '')

        # no summary at all -> duration missing, not fabricated
        labels2 = B.derive_episode_labels(run, trace, cfg, ready=2.0,
                                          summary=None)
        check('labels_no_summary_duration_missing',
              labels2['duration_sec'] is None
              and 'duration_sec' in labels2['missing']
              and labels2['success'] is False
              and labels2['collector_committed'] is False)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_quality_report():
    def ep_row(cid, seed, eid, speed, success, fell, usable, duration):
        return {'episode_id': eid, 'config_id': cid, 'seed': seed,
                'step_length_m': 0.02, 'frequency_hz': 0.5,
                'labels': {'mean_speed_x_mps': speed,
                           'max_torso_tilt_deg': 2.0,
                           'duration_sec': duration,
                           'energy_qdot': 1.0, 'fall_time': -1.0,
                           'success': success, 'fell': fell, 'usable': usable,
                           'failure_reason': None if success else 'fall'}}
    episodes = [
        ep_row('A0', 101, 'episode_000001', 0.010, True, False, True, 10.0),
        ep_row('A0', 202, 'episode_000002', 0.012, True, False, True, 10.5),
        ep_row('A1', 101, 'episode_000003', 0.020, True, False, True, 11.0),
        ep_row('A1', 202, 'episode_000004', None, False, True, False, 6.0),
    ]
    splits = {'train': 2, 'val': 1, 'eval_fixed': 1}
    report = B.quality_report(episodes, splits, [1, 2, 3, 4])
    check('report_by_config', sorted(report['by_config']) == ['A0', 'A1'])
    check('report_A0_success', report['by_config']['A0']['success_count'] == 2)
    check('report_A1_fell', report['by_config']['A1']['fell_count'] == 1)
    check('report_overall_count', report['overall']['episode_count'] == 4)
    check('report_no_duplicates',
          report['overall']['duplicate_episode_ids'] == []
          and report['overall']['duplicate_config_seed_pairs'] == [])
    check('report_split_counts', report['overall']['split_counts'] == splits)
    order = report['speed_order']
    check('speed_order_measured_desc',
          [m['mean_speed_x_mps'] for m in order['measured']]
          == [0.020, 0.012, 0.010], order['measured'])
    check('speed_order_unmeasured_listed',
          len(order['unmeasured']) == 1
          and order['unmeasured'][0]['episode_id'] == 'episode_000004')
    check('speed_order_basis_no_cmd_vel', 'cmd_vel' in order['basis'])


def test_stats():
    s = B._stats([1.0, None, 3.0, float('nan'), 2.0])
    check('stats_ignores_nonfinite', s['n'] == 3 and s['min'] == 1.0
          and s['max'] == 3.0 and abs(s['mean'] - 2.0) < 1e-12, s)
    check('stats_empty', B._stats([]) == {'n': 0, 'min': None, 'max': None,
                                          'mean': None})


def test_root_guard_and_temp():
    check('default_root_not_old_evidence',
          R.DEFAULT_ROOT.endswith('task4_phase2b_baseline_a_runs')
          and 'smoke_evidence' not in R.DEFAULT_ROOT, R.DEFAULT_ROOT)
    tmp = tempfile.mkdtemp(prefix='p2b_guard_')
    try:
        fresh = os.path.join(tmp, 'fresh_root')
        got, replaced = R.guard_root(fresh, allow_overwrite=False)
        check('guard_fresh_ok', got == os.path.abspath(fresh)
              and replaced is False)
        check('guard_does_not_create', not os.path.exists(fresh))

        existing = os.path.join(tmp, 'existing_root')
        os.makedirs(existing)
        marker = os.path.join(existing, 'dataset.json')
        with open(marker, 'w', encoding='utf-8') as fh:
            fh.write('PRECIOUS')
        try:
            R.guard_root(existing, allow_overwrite=False)
            refused = False
        except R.RunnerError as exc:
            refused = exc.reason == 'root_exists_refused'
        check('guard_existing_refused', refused)
        check('guard_existing_untouched',
              open(marker, encoding='utf-8').read() == 'PRECIOUS')

        got, replaced = R.guard_root(existing, allow_overwrite=True)
        check('guard_allow_overwrite', replaced is True)
        check('guard_allow_does_not_delete',
              os.path.isfile(marker))

        # main() must refuse an existing root before any ROS check/launch
        try:
            R.main(['--mode', 'batch', '--root', existing,
                    '--seeds-per-config', '1'])
            main_refused = False
        except R.RunnerError as exc:
            main_refused = exc.reason == 'root_exists_refused'
        check('main_refuses_existing_root', main_refused)

        # unique temp files for json writes, cleaned up on success
        target = os.path.join(tmp, 'x', 'result.json')
        R._write_json(target, {'a': 1})
        leftover = [n for n in os.listdir(os.path.dirname(target))
                    if n.startswith('.tmp_p2b_')]
        check('write_json_no_temp_leftover', not leftover, leftover)
        check('write_json_content',
              json.load(open(target, encoding='utf-8')) == {'a': 1})
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('Phase 2B Baseline A OFFLINE tests (no simulation)')
    test_defaults_and_plan()
    test_seeds_and_sampling()
    test_energy_and_fall()
    test_labels_honest_failure()
    test_quality_report()
    test_stats()
    test_root_guard_and_temp()
    passed = sum(1 for r in _RESULTS if r['pass'])
    total = len(_RESULTS)
    print('\nphase 2B baseline A offline tests: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as fh:
            json.dump({'offline': True, 'passed': passed, 'total': total,
                       'results': _RESULTS}, fh, indent=2, sort_keys=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
