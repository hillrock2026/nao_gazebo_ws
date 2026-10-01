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
Phase 2E OFFLINE tests (no Gazebo, no ROS): unified labels, continuous metrics,
quality score and aggregate consistency.
"""

import argparse
import copy
import hashlib
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
_SCRIPTS = os.path.normpath(os.path.join(
    _TOOLS, '..', 'src', 'nao_gz_sim', 'scripts'))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)
import phase2e_labels_quality as Q  # noqa: E402
import phase2a_episode_dataset as ep  # noqa: E402
import nao_trajectory_schema as schema  # noqa: E402

_RESULTS = []
POS = [0.0] * 24
POS[11] = -0.0349066
POS[22] = 0.0349066


def check(name, condition, detail=None):
    _RESULTS.append({'name': name, 'pass': bool(condition), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if condition else 'FAIL', name,
                           '' if condition else ' :: %s' % (detail,)))


def sample(t, tilt_deg=0.0, left=False, right=True, dq=0.1, cmd=True,
           energy_dq=None):
    half = math.radians(tilt_deg) / 2.0
    q = [math.sin(half), 0.0, 0.0, math.cos(half)]  # rotation about x
    rec = {
        'timestamp': float(t),
        'joint_position': list(POS),
        'joint_velocity': [energy_dq if energy_dq is not None else dq] * 24,
        'imu': {'stamp': float(t), 'orientation': q,
                'angular_velocity': [0.0, 0.0, 0.0],
                'linear_acceleration': [0.0, 0.0, 9.81]},
        'left_contact': left, 'right_contact': right,
        'command': {'stamp': float(t), 'position': list(POS)} if cmd else None,
    }
    return rec


def make_run(root, name, samples):
    run = os.path.join(root, name)
    os.makedirs(run)
    with open(os.path.join(run, 'manifest.json'), 'w', encoding='utf-8') as fh:
        json.dump(schema.build_manifest(), fh, allow_nan=False)
    with open(os.path.join(run, 'samples.jsonl'), 'w', encoding='utf-8') as fh:
        for rec in samples:
            fh.write(schema.strict_dumps(rec) + '\n')
    return run


def meta(eid, **over):
    base = {
        'episode_id': eid, 'controller_name': 'phase4b_producer', 'seed': 1001,
        'param_set': {'step_length_m': 0.02, 'step_frequency_hz': 0.5,
                      'swing_height_m': 0.008, 'double_support_ratio': 0.3,
                      'sway_amplitude_rad': 0.06, 'torso_pitch_rad': 0.0},
        'duration_sec': 10.0, 'fell': False, 'fall_time': -1.0,
        'mean_speed_x_mps': 0.001, 'max_torso_tilt_deg': 8.0,
        'energy_qdot': 1.0, 'success': True, 'usable': True,
        'failure_reason': None,
    }
    base.update(over)
    return base


def build_dataset(root, entries):
    inputs = []
    for eid, samples, m in entries:
        inputs.append(ep.EpisodeInput(eid, make_run(root, 'r_' + eid, samples), m))
    ds = ep.EpisodeDataset(os.path.join(root, 'ds'))
    ds.build(inputs, dataset_name='p2e_test',
             sample_schema={'name': 'nao_gazebo_trajectory', 'version': '1.0.1'})
    return ds


def test_tilt_and_contact():
    check('tilt_identity_zero', Q.imu_tilt_deg([0.0, 0.0, 0.0, 1.0]) == 0.0)
    tilt = Q.imu_tilt_deg([math.sin(math.radians(30) / 2), 0.0, 0.0,
                           math.cos(math.radians(30) / 2)])
    check('tilt_thirty_deg', abs(tilt - 30.0) < 1e-6, tilt)
    check('tilt_bad_quat_none', Q.imu_tilt_deg([0.0, 0.0, 0.0]) is None)
    check('tilt_zero_norm_none', Q.imu_tilt_deg([0.0, 0.0, 0.0, 0.0]) is None)

    alt = Q.contact_alternation(['L', 'L', 'R', 'R', 'L', 'N', 'D'])
    check('alt_counts', alt['alternation_count'] == 2
          and alt['transitions'] == 4, alt)
    check('alt_fractions', 0.0 <= alt['alternation_fraction'] <= 1.0
          and 0.0 <= alt['double_support_fraction'] <= 1.0, alt)
    check('alt_empty_none', Q.contact_alternation([]) is None)


def test_sample_metrics():
    tmp = tempfile.mkdtemp(prefix='p2e_metrics_')
    try:
        path = os.path.join(tmp, 'samples.jsonl')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(schema.strict_dumps(sample(1.0, tilt_deg=10.0,
                                                energy_dq=1.0)) + '\n')
            fh.write(schema.strict_dumps(sample(1.01, tilt_deg=20.0,
                                                energy_dq=0.0)) + '\n')
        m = Q.compute_sample_metrics(path)
        check('metrics_max_tilt', m['imu_tilt_deg_max'] == 20.0, m)
        check('metrics_mean_tilt', m['imu_tilt_deg_mean'] == 15.0, m)
        check('metrics_internal_keys_are_imu_named',
              'max_torso_tilt_deg' not in m and 'mean_torso_tilt_deg' not in m,
              sorted(m))
        check('metrics_energy', abs(m['energy_qdot'] - 12.0) < 1e-6, m)
        check('metrics_command_coverage', m['command_coverage'] == 1.0, m)
        # missing fields -> null, never 0
        bad = os.path.join(tmp, 'bad.jsonl')
        rec = sample(1.0)
        del rec['imu']
        del rec['command']
        with open(bad, 'w', encoding='utf-8') as fh:
            fh.write(schema.strict_dumps(rec) + '\n')
        mb = Q.compute_sample_metrics(bad)
        check('metrics_missing_tilt_null', mb['imu_tilt_deg_max'] is None
              and mb['imu_tilt_deg_mean'] is None, mb)
        check('metrics_missing_command_null', mb['command_coverage'] is None, mb)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_quality_score():
    full = Q.compute_quality_score(False, -1.0, 10.0, 5.0, 0.2, None)
    check('score_complete_or_partial',
          full['quality_score_status'] == 'partial'
          and full['quality_score'] is not None, full)
    check('score_missing_speed', 'speed' in full['quality_score_missing'])
    check('score_weights_renormalized',
          abs(sum(full['quality_score_weights'].values()) - 1.0) < 1e-6,
          full['quality_score_weights'])
    check('score_version', full['quality_score_version'] == '1.0.0')
    # fall lowers survival
    fall = Q.compute_quality_score(True, 2.0, 10.0, 5.0, 0.2, None)
    check('score_fall_lower', fall['quality_score'] < full['quality_score'],
          (fall['quality_score'], full['quality_score']))
    check('score_fall_survival', abs(
        fall['quality_score_components']['survival'] - 0.2) < 1e-6, fall)
    # energy missing is not 0 and is recorded
    no_energy = Q.compute_quality_score(False, -1.0, 10.0, 5.0, None, None)
    check('score_energy_missing_not_zero',
          no_energy['quality_score_components']['energy'] is None
          and 'energy' in no_energy['quality_score_missing'], no_energy)
    check('score_energy_renormalized',
          abs(sum(no_energy['quality_score_weights'].values()) - 1.0) < 1e-6)
    # survival missing -> insufficient, score null
    insuff = Q.compute_quality_score(None, None, None, 5.0, 0.2, None)
    check('score_insufficient_null',
          insuff['quality_score'] is None
          and insuff['quality_score_status'] == 'insufficient'
          and 'survival' in insuff['quality_score_missing'], insuff)
    # deterministic + JSON-safe (no NaN/Infinity)
    check('score_deterministic',
          Q.compute_quality_score(False, -1.0, 10.0, 5.0, 0.2, None)
          == full)
    json.dumps({k: v for k, v in full.items()}, allow_nan=False)
    check('score_json_safe', True)


def test_evaluate_dataset_and_readonly():
    tmp = tempfile.mkdtemp(prefix='p2e_ds_')
    try:
        e1 = 'episode_000001'
        e2 = 'episode_000002'
        ds = build_dataset(tmp, [
            (e1, [sample(1.0 + 0.01 * i, tilt_deg=5.0) for i in range(4)],
             meta(e1)),
            (e2, [sample(2.0 + 0.01 * i, tilt_deg=40.0, left=True,
                         right=False) for i in range(4)],
             meta(e2, success=False, usable=True, fell=True, fall_time=3.0,
                  failure_reason='fall')),
        ])
        # hash the dataset before
        before = {}
        for root, _, files in os.walk(ds.root):
            for f in files:
                p = os.path.join(root, f)
                before[os.path.relpath(p, ds.root)] = hashlib.sha256(
                    open(p, 'rb').read()).hexdigest()
        _, report = Q.evaluate_dataset(ds.root)
        after = {}
        for root, _, files in os.walk(ds.root):
            for f in files:
                p = os.path.join(root, f)
                after[os.path.relpath(p, ds.root)] = hashlib.sha256(
                    open(p, 'rb').read()).hexdigest()
        check('evaluate_readonly', before == after)
        check('evaluate_episode_count', len(report['per_episode']) == 2)
        check('evaluate_usable_both',
              report['aggregate']['overall']['usable_count'] == 2)
        check('evaluate_failure_kept',
              report['aggregate']['overall']['failure_reason_counts']
              == {'none': 1, 'fall': 1}, report['aggregate']['overall'])
        # speed from metadata, never cmd_vel
        check('evaluate_speed_from_metadata',
              all(e['mean_speed_x_mps'] == 0.001
                  for e in report['per_episode']))
        check('evaluate_tracking_error_null',
              all(e['speed_tracking_error'] is None
                  for e in report['per_episode']))
        check('aggregate_consistent', Q.check_aggregate_consistency(report) == [])
        check('quality_by_config_counts',
              all(b['episode_count'] == 1
                  for b in report['aggregate']['by_config'].values()))
        # contact_alternation is retained as a diagnostic even though it is
        # no longer declared to participate in the score
        check('evaluate_contact_retained',
              all('contact_alternation' in e for e in report['per_episode'])
              and report['field_dictionary']['contact_alternation']['in_score']
              is False)
        check('evaluate_imu_tilt_fields',
              all('imu_tilt_deg_max' in e and 'imu_tilt_deg_mean' in e
                  for e in report['per_episode']))

        # derived index must not touch the original dataset
        derived = os.path.join(tmp, 'derived')
        Q.write_derived_index(report, derived)
        check('derived_created',
              os.path.isfile(os.path.join(derived, 'derived_index.jsonl')))
        after2 = {}
        for root, _, files in os.walk(ds.root):
            for f in files:
                p = os.path.join(root, f)
                after2[os.path.relpath(p, ds.root)] = hashlib.sha256(
                    open(p, 'rb').read()).hexdigest()
        check('derived_non_destructive', before == after2)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_old_metadata_compat_and_none_reason():
    # 1.0.0 legacy metadata (no usable_semantics) still validates
    legacy = meta('episode_000001')
    legacy['schema_version'] = '1.0.0'
    check('legacy_1_0_0_valid', ep.validate_metadata(legacy) == [])
    new = meta('episode_000001')
    new['schema_version'] = '1.1.0'
    new['usable_semantics'] = 'sample_completeness'
    check('new_1_1_0_valid', ep.validate_metadata(new) == [])
    bad = meta('episode_000001')
    bad['schema_version'] = '1.1.0'   # missing usable_semantics
    check('new_1_1_0_requires_semantics',
          any(e['reason'] == 'required_for_v1_1_0'
              for e in ep.validate_metadata(bad)))
    unknown = meta('episode_000001')
    unknown['schema_version'] = '9.9.9'
    check('unknown_version_rejected',
          any(e['reason'] == 'unknown_schema_version'
              for e in ep.validate_metadata(unknown)))
    # failure_reason None / unknown never crash aggregation
    per = [{'candidate_id': 'c', 'success': True, 'usable': True, 'fell': False,
            'mean_speed_x_mps': None, 'max_torso_tilt_deg': None,
            'quality_score': None, 'quality_score_status': 'insufficient',
            'failure_reason': None},
           {'candidate_id': 'c', 'success': False, 'usable': True, 'fell': True,
            'mean_speed_x_mps': 0.0, 'max_torso_tilt_deg': 5.0,
            'quality_score': 0.5, 'quality_score_status': 'partial',
            'failure_reason': 'weird_unknown_reason'}]
    agg = Q.aggregate(per)
    check('aggregate_none_and_unknown_reason',
          agg['overall']['failure_reason_counts']
          == {'none': 1, 'weird_unknown_reason': 1}
          and agg['overall']['usable_count'] == 2, agg['overall'])


_CLI = os.path.join(_TOOLS, 'phase2e_labels_quality.py')


def _reject_constant(name):
    raise ValueError('non-finite JSON constant: %s' % name)


def _raises_phase2e(fn, *args, **kwargs):
    """Return the Phase2EError message, a ``__WRONG__`` marker, or None."""
    try:
        fn(*args, **kwargs)
    except Q.Phase2EError as exc:
        return str(exc)
    except Exception as exc:  # pragma: no cover - wrong error type is a failure
        return '__WRONG__%s: %s' % (type(exc).__name__, exc)
    return None


def _nonfinite_error(path):
    try:
        Q.compute_sample_metrics(path)
    except Q.Phase2EError as exc:
        return str(exc)
    except Exception as exc:  # pragma: no cover
        return '__WRONG__%s: %s' % (type(exc).__name__, exc)
    return None


def _write_jsonl(path, records, allow_nan=False):
    with open(path, 'w', encoding='utf-8') as fh:
        for rec in records:
            fh.write(json.dumps(rec, allow_nan=allow_nan) + '\n')


def test_nonfinite_components():
    nan, inf, ninf = float('nan'), float('inf'), float('-inf')
    for label, value in (('nan', nan), ('inf', inf), ('ninf', ninf)):
        d = _raises_phase2e(Q._clamp, value)
        check('clamp_%s_rejected' % label,
              d is not None and not d.startswith('__WRONG__'), d)
    check('clamp_finite_ok', Q._clamp(0.5) == 0.5 and Q._clamp(2.0) == 1.0)

    # A non-finite quaternion must never look like a perfect (0deg) torso.
    for label, q in (('nan', [nan, 0.0, 0.0, 1.0]),
                     ('inf', [inf, 0.0, 0.0, 1.0]),
                     ('ninf', [0.0, ninf, 0.0, 1.0])):
        d = _raises_phase2e(Q.imu_tilt_deg, q)
        check('tilt_%s_rejected' % label,
              d is not None and not d.startswith('__WRONG__'), d)
    check('tilt_valid_still_works', abs(Q.imu_tilt_deg(
        [math.sin(math.radians(30) / 2), 0.0, 0.0,
         math.cos(math.radians(30) / 2)]) - 30.0) < 1e-6)

    # Components: NaN/Infinity must never become a perfect 1.0.
    for label, value in (('nan', nan), ('inf', inf)):
        d = _raises_phase2e(Q._stability_component, value, None)
        check('stability_%s_rejected' % label,
              d is not None and not d.startswith('__WRONG__'), d)
        d = _raises_phase2e(Q._energy_component, value)
        check('energy_%s_rejected' % label,
              d is not None and not d.startswith('__WRONG__'), d)
    d = _raises_phase2e(Q._survival_component, True, nan, 10.0)
    check('survival_fall_time_nan_rejected',
          d is not None and not d.startswith('__WRONG__'), d)
    d = _raises_phase2e(Q._survival_component, True, 2.0, nan)
    check('survival_duration_nan_rejected',
          d is not None and not d.startswith('__WRONG__'), d)
    d = _raises_phase2e(Q._survival_component, nan, -1.0, 10.0)
    check('survival_fell_nan_rejected',
          d is not None and not d.startswith('__WRONG__'), d)

    # API entry point: corrupt inputs cannot yield a finite, high score.
    cases = (
        ('mean_tilt_nan', (False, -1.0, 10.0, nan, 0.2, None)),
        ('mean_tilt_inf', (False, -1.0, 10.0, inf, 0.2, None)),
        ('energy_nan', (False, -1.0, 10.0, 5.0, nan, None)),
        ('energy_ninf', (False, -1.0, 10.0, 5.0, ninf, None)),
        ('fall_time_nan', (True, nan, 10.0, 5.0, 0.2, None)),
        ('duration_nan', (True, 2.0, nan, 5.0, 0.2, None)),
        ('fell_nan', (nan, -1.0, 10.0, 5.0, 0.2, None)),
    )
    for label, args in cases:
        d = _raises_phase2e(Q.compute_quality_score, *args)
        check('score_%s_rejected' % label,
              d is not None and not d.startswith('__WRONG__'), d)


def test_sample_metrics_nonfinite():
    tmp = tempfile.mkdtemp(prefix='p2e_nonfinite_')
    try:
        base = [sample(1.0 + 0.01 * i, tilt_deg=5.0) for i in range(3)]
        nan, inf, ninf = float('nan'), float('inf'), float('-inf')

        def case(name, mutate, allow_nan=True):
            recs = copy.deepcopy(base)
            mutate(recs[1])
            path = os.path.join(tmp, '%s.jsonl' % name)
            _write_jsonl(path, recs, allow_nan=allow_nan)
            return _nonfinite_error(path)

        # The non-standard NaN/Infinity literals are rejected at parse time, so
        # the error always carries the offending line (here the middle line).
        d = case('jv_nan', lambda r: r['joint_velocity'].__setitem__(0, nan))
        check('metrics_jv_nan_located',
              d is not None and not d.startswith('__WRONG__') and ':2' in d, d)
        d = case('jv_inf', lambda r: r['joint_velocity'].__setitem__(3, inf))
        check('metrics_jv_inf_rejected',
              d is not None and not d.startswith('__WRONG__') and ':2' in d, d)
        d = case('jv_ninf', lambda r: r['joint_velocity'].__setitem__(3, ninf))
        check('metrics_jv_ninf_rejected',
              d is not None and not d.startswith('__WRONG__') and ':2' in d, d)
        d = case('imu_nan',
                 lambda r: r['imu']['orientation'].__setitem__(0, nan))
        check('metrics_imu_nan_located',
              d is not None and not d.startswith('__WRONG__') and ':2' in d, d)
        d = case('ts_nan', lambda r: r.__setitem__('timestamp', nan))
        check('metrics_timestamp_nan_located',
              d is not None and not d.startswith('__WRONG__') and ':2' in d, d)
        d = case('cmd_stamp_nan',
                 lambda r: r['command'].__setitem__('stamp', nan))
        check('metrics_command_stamp_nan_located',
              d is not None and not d.startswith('__WRONG__') and ':2' in d, d)
        d = case('cmd_pos_nan',
                 lambda r: r['command']['position'].__setitem__(2, nan))
        check('metrics_command_position_nan_located',
              d is not None and not d.startswith('__WRONG__') and ':2' in d, d)

        # A finite value whose square overflows must also fail (not become inf);
        # this exercises the per-field gate with full field context.
        d = case('jv_overflow',
                 lambda r: r['joint_velocity'].__setitem__(0, 1e200))
        check('metrics_jv_square_overflow_located',
              d is not None and not d.startswith('__WRONG__')
              and ':2' in d and 'joint_velocity' in d and 'overflow' in d, d)

        # A literal 1e999 is valid JSON text but overflows to inf: reject it.
        recs = copy.deepcopy(base)
        recs[1]['joint_velocity'][0] = 123.456789
        line = json.dumps(recs[1])
        path = os.path.join(tmp, 'huge_exp.jsonl')
        with open(path, 'w', encoding='utf-8') as fh:
            fh.write(json.dumps(base[0]) + '\n')
            fh.write(line.replace('123.456789', '1e999', 1) + '\n')
            fh.write(json.dumps(base[2]) + '\n')
        d = _nonfinite_error(path)
        check('metrics_1e999_rejected',
              d is not None and not d.startswith('__WRONG__') and ':2' in d, d)

        # The clean file still yields finite, JSON-safe metrics.
        good = os.path.join(tmp, 'good.jsonl')
        _write_jsonl(good, base)
        m = Q.compute_sample_metrics(good)
        json.dumps(m, allow_nan=False)
        check('metrics_finite_json_safe', m['energy_qdot'] is not None)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_evaluate_episode_nonfinite_metadata():
    tmp = tempfile.mkdtemp(prefix='p2e_meta_')
    try:
        sp = os.path.join(tmp, 'samples.jsonl')
        _write_jsonl(sp, [sample(1.0 + 0.01 * i, tilt_deg=5.0)
                          for i in range(3)])
        ref = Q.evaluate_episode(meta('episode_000001'), sp)
        bad = meta('episode_000001', mean_speed_x_mps=float('inf'),
                   max_torso_tilt_deg=float('nan'),
                   energy_qdot=float('-inf'))
        ev = Q.evaluate_episode(bad, sp)
        check('meta_nonfinite_speed_null',
              ev['continuous']['mean_speed_x_mps'] is None)
        check('meta_nonfinite_tilt_null',
              ev['continuous']['max_torso_tilt_deg'] is None)
        reasons = sorted((w['field'], w['reason'])
                         for w in ev['data_quality_warnings'])
        check('meta_nonfinite_warned',
              ('mean_speed_x_mps', 'non_finite_metadata') in reasons
              and ('max_torso_tilt_deg', 'non_finite_metadata') in reasons
              and ('energy_qdot', 'non_finite_metadata') in reasons, reasons)
        check('meta_nonfinite_score_unchanged', ev['quality'] == ref['quality'])
        json.dumps(ev, allow_nan=False)
        check('meta_nonfinite_json_safe', True)

        d = _raises_phase2e(Q.evaluate_episode,
                            meta('episode_000001', fell=float('nan')), sp)
        check('meta_fell_nan_rejected',
              d is not None and not d.startswith('__WRONG__'), d)
        d = _raises_phase2e(Q.evaluate_episode,
                            meta('episode_000001', fell=True,
                                 fall_time=float('nan')), sp)
        check('meta_fall_time_nan_rejected',
              d is not None and not d.startswith('__WRONG__'), d)
        d = _raises_phase2e(Q.evaluate_episode,
                            meta('episode_000001', fell=True, fall_time=2.0,
                                 duration_sec=float('nan')), sp)
        check('meta_duration_nan_rejected',
              d is not None and not d.startswith('__WRONG__'), d)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_dataset_nonfinite_rejected():
    tmp = tempfile.mkdtemp(prefix='p2e_dsnan_')
    try:
        e1, e2 = 'episode_000001', 'episode_000002'
        ds = build_dataset(tmp, [
            (e1, [sample(1.0 + 0.01 * i, tilt_deg=5.0) for i in range(4)],
             meta(e1)),
            (e2, [sample(2.0 + 0.01 * i, tilt_deg=5.0) for i in range(4)],
             meta(e2)),
        ])
        Q.evaluate_dataset(ds.root)
        check('dataset_nonfinite_precondition_ok', True)

        # Inject a NaN on the middle line and drop the now-stale content hash so
        # the structural validator still admits the dataset.
        sp = ds.samples_path(e1)
        lines = open(sp, encoding='utf-8').read().splitlines()
        rec = json.loads(lines[1])
        rec['joint_velocity'][0] = float('nan')
        lines[1] = json.dumps(rec, allow_nan=True)
        with open(sp, 'w', encoding='utf-8') as fh:
            fh.write('\n'.join(lines) + '\n')
        mp = ds.metadata_path(e1)
        m = json.load(open(mp, encoding='utf-8'))
        m.pop('samples_sha256', None)
        with open(mp, 'w', encoding='utf-8') as fh:
            json.dump(m, fh, indent=2, sort_keys=True)
        check('dataset_nonfinite_structurally_valid',
              ep.EpisodeDataset(ds.root).validate()['valid'])

        err = None
        try:
            Q.evaluate_dataset(ds.root)
        except Q.Phase2EError as exc:
            err = str(exc)
        except Exception as exc:  # pragma: no cover
            err = '__WRONG__%s: %s' % (type(exc).__name__, exc)
        check('dataset_nonfinite_rejected_with_context',
              err is not None and not err.startswith('__WRONG__')
              and e1 in err and ':2' in err, err)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _run_cli(*args):
    return subprocess.run([sys.executable, _CLI, *args],
                          capture_output=True, text=True)


def _stderr_payload(r):
    try:
        return json.loads(r.stderr.strip())
    except ValueError:
        return None


def test_cli_error_paths():
    tmp = tempfile.mkdtemp(prefix='p2e_cli_')
    try:
        e1 = 'episode_000001'
        ds = build_dataset(tmp, [
            (e1, [sample(1.0 + 0.01 * i, tilt_deg=5.0) for i in range(4)],
             meta(e1))])
        before = _tree_hash(ds.root)

        # 1. normal path, no derived root -> exit 0, strict JSON report written.
        out = os.path.join(tmp, 'out.json')
        r = _run_cli('--dataset', ds.root, '--out', out)
        check('cli_normal_exit_zero', r.returncode == 0, r.stderr[-300:])
        check('cli_normal_no_traceback', 'Traceback' not in r.stderr,
              r.stderr[-300:])
        check('cli_normal_report_written', os.path.isfile(out))
        if os.path.isfile(out):
            with open(out, encoding='utf-8') as fh:
                json.load(fh, parse_constant=_reject_constant)
            check('cli_normal_report_strict_json', True)
        summary = _stderr_payload(r)
        if summary is None:
            try:
                summary = json.loads(r.stdout.strip())
            except ValueError:
                summary = None
        check('cli_normal_stdout_parseable',
              isinstance(summary, dict) and 'episodes' in summary,
              r.stdout[-300:])

        # 2. normal path, fresh derived root -> exit 0, both files written.
        derived_fresh = os.path.join(tmp, 'derived_fresh')
        r = _run_cli('--dataset', ds.root, '--derived-root', derived_fresh)
        check('cli_derived_fresh_exit_zero', r.returncode == 0, r.stderr[-300:])
        check('cli_derived_fresh_files',
              os.path.isfile(os.path.join(derived_fresh,
                                          'derived_index.jsonl'))
              and os.path.isfile(os.path.join(derived_fresh,
                                              'derived_manifest.json')),
              os.listdir(derived_fresh) if os.path.isdir(derived_fresh) else None)

        # 3. --out parent directory not yet present -> exit 0, atomic write.
        deep_out = os.path.join(tmp, 'new_parent', 'nested', 'report.json')
        r = _run_cli('--dataset', ds.root, '--out', deep_out)
        check('cli_out_parent_created_exit_zero', r.returncode == 0,
              r.stderr[-300:])
        check('cli_out_parent_created_file', os.path.isfile(deep_out))
        check('cli_valid_runs_non_destructive', _tree_hash(ds.root) == before)

        # 4. invalid dataset -> exit 2, structured stderr, no report.
        missing = os.path.join(tmp, 'does_not_exist')
        out2 = os.path.join(tmp, 'never_invalid.json')
        r = _run_cli('--dataset', missing, '--out', out2)
        check('cli_invalid_dataset_exit_2', r.returncode == 2, r.returncode)
        check('cli_invalid_dataset_no_traceback', 'Traceback' not in r.stderr,
              r.stderr[-300:])
        check('cli_invalid_dataset_structured',
              isinstance(_stderr_payload(r), dict)
              and _stderr_payload(r).get('ok') is False, r.stderr[-300:])
        check('cli_invalid_dataset_no_report', not os.path.isfile(out2))

        # 5. existing derived root -> exit 2, structured, no overwrite/pollution.
        existing = os.path.join(tmp, 'derived_exists')
        os.makedirs(existing)
        sentinel = os.path.join(existing, 'sentinel.txt')
        with open(sentinel, 'w', encoding='utf-8') as fh:
            fh.write('keep me\n')
        r = _run_cli('--dataset', ds.root, '--derived-root', existing,
                     '--out', os.path.join(tmp, 'never_derived.json'))
        check('cli_derived_existing_exit_2', r.returncode == 2, r.returncode)
        check('cli_derived_existing_no_traceback', 'Traceback' not in r.stderr,
              r.stderr[-300:])
        check('cli_derived_existing_structured',
              isinstance(_stderr_payload(r), dict)
              and _stderr_payload(r).get('ok') is False
              and 'derived root already exists' in _stderr_payload(r).get(
                  'detail', ''), r.stderr[-300:])
        check('cli_derived_existing_untouched',
              sorted(os.listdir(existing)) == ['sentinel.txt'],
              os.listdir(existing))
        check('cli_derived_existing_no_report',
              not os.path.isfile(os.path.join(tmp, 'never_derived.json')))
        check('cli_derived_existing_no_temp',
              not [f for f in os.listdir(tmp) if f.startswith('.tmp_p2e_')],
              os.listdir(tmp))

        # 6. non-finite sample -> exit 2, structured stderr, no report.
        sp = ds.samples_path(e1)
        lines = open(sp, encoding='utf-8').read().splitlines()
        rec = json.loads(lines[1])
        rec['joint_velocity'][0] = float('nan')
        lines[1] = json.dumps(rec, allow_nan=True)
        with open(sp, 'w', encoding='utf-8') as fh:
            fh.write('\n'.join(lines) + '\n')
        mp = ds.metadata_path(e1)
        m = json.load(open(mp, encoding='utf-8'))
        m.pop('samples_sha256', None)
        with open(mp, 'w', encoding='utf-8') as fh:
            json.dump(m, fh, indent=2, sort_keys=True)
        out3 = os.path.join(tmp, 'never_nonfinite.json')
        r = _run_cli('--dataset', ds.root, '--out', out3)
        check('cli_nonfinite_exit_2', r.returncode == 2,
              (r.returncode, r.stderr[-300:]))
        check('cli_nonfinite_no_traceback', 'Traceback' not in r.stderr,
              r.stderr[-300:])
        check('cli_nonfinite_structured',
              isinstance(_stderr_payload(r), dict)
              and _stderr_payload(r).get('ok') is False, r.stderr[-300:])
        check('cli_nonfinite_no_report', not os.path.isfile(out3))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _tree_hash(root):
    out = {}
    for r, _, files in os.walk(root):
        for f in files:
            p = os.path.join(r, f)
            out[os.path.relpath(p, root)] = hashlib.sha256(
                open(p, 'rb').read()).hexdigest()
    return out


def test_contact_contract():
    entry = Q.FIELD_DICTIONARY['contact_alternation']
    check('contract_contact_in_score_false', entry['in_score'] is False, entry)
    check('contract_contact_in_filter_false', entry['in_filter'] is False, entry)
    check('contract_contact_source_consistent',
          'left/right_contact' in entry['source'], entry)
    contact_a = {'n': 4, 'transitions': 3, 'alternation_count': 2,
                 'alternation_fraction': 0.66, 'double_support_fraction': 0.0,
                 'flight_fraction': 0.0}
    contact_b = {'n': 4, 'transitions': 0, 'alternation_count': 0,
                 'alternation_fraction': 0.0, 'double_support_fraction': 1.0,
                 'flight_fraction': 0.0}
    sa = Q.compute_quality_score(False, -1.0, 10.0, 5.0, 0.2, contact_a)
    sb = Q.compute_quality_score(False, -1.0, 10.0, 5.0, 0.2, contact_b)
    check('contract_contact_does_not_change_score', sa == sb, (sa, sb))
    comps = set(sa['quality_score_components'])
    check('contract_contact_not_a_component',
          comps == {'speed', 'survival', 'stability', 'energy'}, comps)
    check('contract_contact_not_in_weights',
          'contact_alternation' not in sa['quality_score_weights']
          and set(sa['quality_score_weights']) <= {'survival', 'stability',
                                                    'energy'},
          sa['quality_score_weights'])


def test_tilt_field_mapping():
    tmp = tempfile.mkdtemp(prefix='p2e_tilt_')
    try:
        sp = os.path.join(tmp, 'samples.jsonl')
        _write_jsonl(sp, [sample(1.0, tilt_deg=30.0),
                          sample(1.01, tilt_deg=30.0)])
        m = Q.compute_sample_metrics(sp)
        check('tilt_internal_max_key',
              abs(m['imu_tilt_deg_max'] - 30.0) < 1e-6, m)
        check('tilt_internal_mean_key',
              abs(m['imu_tilt_deg_mean'] - 30.0) < 1e-6, m)

        ev = Q.evaluate_episode(
            meta('episode_000001', max_torso_tilt_deg=99.0), sp)
        check('tilt_episode_imu_max',
              abs(ev['continuous']['imu_tilt_deg_max'] - 30.0) < 1e-6,
              ev['continuous'])
        check('tilt_episode_imu_mean',
              abs(ev['continuous']['imu_tilt_deg_mean'] - 30.0) < 1e-6,
              ev['continuous'])
        # body-attitude metadata is independent from the IMU z-axis tilt
        check('tilt_body_attitude_independent',
              ev['continuous']['max_torso_tilt_deg'] == 99.0, ev['continuous'])
        expected_stab = Q._round(
            Q._clamp(1.0 - 30.0 / Q.DEFAULTS['TILT_BAD_DEG']))
        check('tilt_stability_from_imu_mean',
              ev['quality']['quality_score_components']['stability']
              == expected_stab, ev['quality']['quality_score_components'])
        ev2 = Q.evaluate_episode(
            meta('episode_000001', max_torso_tilt_deg=1.0), sp)
        check('tilt_body_attitude_not_in_score', ev2['quality'] == ev['quality'])
        check('tilt_dictionary_imu_source',
              'IMU' in Q.FIELD_DICTIONARY['imu_tilt_deg_mean']['source']
              and 'mean_torso_tilt_deg'
              in Q.FIELD_DICTIONARY['imu_tilt_deg_mean'].get('note', ''))
        check('tilt_dictionary_body_attitude_distinct',
              'not interchangeable'
              in Q.FIELD_DICTIONARY['max_torso_tilt_deg'].get('note', ''))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _fake_report(dataset_dir, episodes):
    return {
        'dataset_dir': dataset_dir,
        'quality_score_version': Q.QUALITY_SCORE_VERSION,
        'per_episode': episodes,
    }


def _derived_ep(eid, usable, score, status='partial', fell=False, reason=None):
    return {'episode_id': eid, 'usable': usable, 'quality_score': score,
            'quality_score_status': status,
            'quality_score_version': Q.QUALITY_SCORE_VERSION,
            'fell': fell, 'failure_reason': reason}


def test_derived_index_order():
    tmp = tempfile.mkdtemp(prefix='p2e_order_')
    try:
        ds_dir = os.path.join(tmp, 'pseudo_dataset')
        os.makedirs(ds_dir)
        with open(os.path.join(ds_dir, 'keep.txt'), 'w', encoding='utf-8') as fh:
            fh.write('do not touch\n')
        before = _tree_hash(ds_dir)
        episodes = [
            _derived_ep('episode_000004', True, None, status='insufficient'),
            _derived_ep('episode_000003', True, 0.9),
            _derived_ep('episode_000002', True, 0.9),
            _derived_ep('episode_000001', True, 0.5),
            _derived_ep('episode_000006', True, 0.0, fell=True, reason='fall'),
            _derived_ep('episode_000005', False, 0.99),
        ]
        report = _fake_report(ds_dir, episodes)
        root1 = os.path.join(tmp, 'derived1')
        n = Q.write_derived_index(report, root1)
        check('order_kept_count', n == 5, n)
        rows = [json.loads(line)
                for line in open(os.path.join(root1, 'derived_index.jsonl'),
                                 encoding='utf-8')]
        order = [r['episode_id'] for r in rows]
        check('order_finite_desc_none_last',
              order == ['episode_000002', 'episode_000003', 'episode_000001',
                        'episode_000006', 'episode_000004'], order)
        check('order_usable_false_excluded',
              'episode_000005' not in order, order)
        check('order_failed_but_usable_included',
              'episode_000006' in order, order)
        man = json.load(open(os.path.join(root1, 'derived_manifest.json'),
                             encoding='utf-8'))
        check('order_manifest_fields',
              man['filter'] == 'usable == true'
              and 'desc' in man['sort'] and 'nulls last' in man['sort']
              and 'episode_id asc' in man['sort']
              and man['quality_score_version'] == Q.QUALITY_SCORE_VERSION
              and man['kept'] == 5 and man['total'] == 6
              and man['non_destructive'] is True, man)
        root2 = os.path.join(tmp, 'derived2')
        Q.write_derived_index(report, root2)
        check('order_deterministic_bytes',
              open(os.path.join(root1, 'derived_index.jsonl'), 'rb').read()
              == open(os.path.join(root2, 'derived_index.jsonl'), 'rb').read()
              and open(os.path.join(root1, 'derived_manifest.json'), 'rb').read()
              == open(os.path.join(root2, 'derived_manifest.json'), 'rb').read())
        check('order_dataset_non_destructive', _tree_hash(ds_dir) == before)
        d = _raises_phase2e(Q.write_derived_index, report, root1)
        check('order_refuses_existing_root',
              d is not None and not d.startswith('__WRONG__')
              and 'exists' in d, d)
        check('order_existing_root_untouched',
              sorted(os.listdir(root1))
              == ['derived_index.jsonl', 'derived_manifest.json'],
              os.listdir(root1))
        check('order_no_temp_files',
              not [f for f in os.listdir(tmp) if f.startswith('.tmp_p2e_')],
              os.listdir(tmp))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_legacy_usable_semantics():
    tmp = tempfile.mkdtemp(prefix='p2e_legacy_')
    try:
        sp = os.path.join(tmp, 'samples.jsonl')
        _write_jsonl(sp, [sample(1.0, tilt_deg=5.0)])

        legacy_missing = meta('episode_000001', schema_version='1.0.0')
        legacy_missing.pop('usable')
        check('legacy_missing_usable_rejected',
              any(e['field'] == 'usable' and e['reason'] == 'missing'
                  for e in ep.validate_metadata(legacy_missing)))
        legacy = meta('episode_000001', schema_version='1.0.0')
        check('legacy_with_usable_valid', ep.validate_metadata(legacy) == [])
        legacy_fail = meta('episode_000001', schema_version='1.0.0',
                           success=False, usable=True, fell=True, fall_time=2.0,
                           failure_reason='fall')
        check('legacy_failed_but_usable_valid',
              ep.validate_metadata(legacy_fail) == [])

        m = meta('episode_000001')
        m.pop('usable')
        ev = Q.evaluate_episode(m, sp)
        check('legacy_eval_usable_null',
              ev['discrete']['usable'] is None, ev['discrete'])
        reasons = [(w['field'], w['reason']) for w in ev['data_quality_warnings']]
        check('legacy_eval_warns',
              ('usable', 'legacy_usable_unknown') in reasons, reasons)
        per = [{'candidate_id': 'c', 'success': True, 'usable': None,
                'fell': False, 'mean_speed_x_mps': None,
                'max_torso_tilt_deg': None, 'quality_score': None,
                'quality_score_status': 'insufficient', 'failure_reason': None}]
        check('legacy_not_counted_usable',
              Q.aggregate(per)['overall']['usable_count'] == 0)

        check('v1_1_0_missing_semantics_rejected',
              any(e['reason'] == 'required_for_v1_1_0'
                  for e in ep.validate_metadata(
                      meta('episode_000001', schema_version='1.1.0'))))
        check('v1_1_0_valid',
              ep.validate_metadata(meta(
                  'episode_000001', schema_version='1.1.0',
                  usable_semantics='sample_completeness')) == [])
        check('legacy_unknown_version_rejected',
              any(e['reason'] == 'unknown_schema_version'
                  for e in ep.validate_metadata(
                      meta('episode_000001', schema_version='9.9.9'))))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _jsonable(value):
    """
    Deterministically normalise test details for strict JSON output.

    Only the known test-detail containers are converted (``set``/``frozenset``
    -> sorted list, ``tuple`` -> list, ``dict``/``list`` recursed); scalars are
    passed through and any unsupported type raises instead of being silently
    stringified, so a new detail type can never slip into ``--json-out``.
    """
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, (set, frozenset)):
        return sorted(_jsonable(v) for v in value)
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, dict):
        return {k: _jsonable(v) for k, v in value.items()}
    raise TypeError('non-serializable test detail type: %s'
                    % type(value).__name__)


def test_json_out_serialization():
    norm = _jsonable({'z': {'b', 'a'}, 'l': (1, 'x')})
    check('json_out_set_deterministic',
          norm == {'z': ['a', 'b'], 'l': [1, 'x']}
          and json.dumps(norm, sort_keys=True, allow_nan=False)
          == '{"l": [1, "x"], "z": ["a", "b"]}', norm)
    raised = False
    try:
        _jsonable(object())
    except TypeError:
        raised = True
    check('json_out_unknown_type_raises', raised)
    # A set detail inside the real results list must serialise strictly.
    probe = [{'name': 'probe', 'pass': True, 'detail': {'b', 'a'}}]
    json.dumps({'results': _jsonable(probe)}, sort_keys=True, allow_nan=False)
    check('json_out_results_strict_json', True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('Phase 2E labels/quality OFFLINE tests (no simulation)')
    test_tilt_and_contact()
    test_sample_metrics()
    test_quality_score()
    test_evaluate_dataset_and_readonly()
    test_old_metadata_compat_and_none_reason()
    test_nonfinite_components()
    test_sample_metrics_nonfinite()
    test_evaluate_episode_nonfinite_metadata()
    test_dataset_nonfinite_rejected()
    test_cli_error_paths()
    test_contact_contract()
    test_tilt_field_mapping()
    test_derived_index_order()
    test_legacy_usable_semantics()
    test_json_out_serialization()
    passed = sum(1 for r in _RESULTS if r['pass'])
    total = len(_RESULTS)
    print('\nphase 2E offline tests: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as fh:
            json.dump({'offline': True, 'passed': passed, 'total': total,
                       'results': _jsonable(_RESULTS)}, fh, indent=2,
                      sort_keys=True, allow_nan=False)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
