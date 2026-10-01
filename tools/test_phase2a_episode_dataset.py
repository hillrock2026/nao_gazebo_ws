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
Phase 2A episode dataset OFFLINE tests (no ROS, no Gazebo, no simulation).

Covers the Phase 2A acceptance criteria:

  1. a valid episode is written and read back;
  2. frozen sample fields are byte-identical; metadata never leaks into samples;
  3. missing/invalid metadata and illegal fall_time/status combinations fail;
  4. the index locates metadata and samples;
  5. missing files, duplicate episodes, duplicate/illegal splits and path
     escapes are detected;
  6. train/val/eval_fixed are deterministic, mutually exclusive and total;
  7. legacy frozen samples are still readable the old way;
  8. a failed build leaves no dataset and no staging directory;
  9. JSON/JSONL round-trips are exact.

Usage: python3 tools/test_phase2a_episode_dataset.py [--json-out F]
Exit 0 = all offline tests passed.
"""

import argparse
import copy
import importlib.util
import json
import os
import shutil
import sys
import tempfile

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
_SCRIPTS = os.path.normpath(os.path.join(
    _TOOLS, '..', 'src', 'nao_gz_sim', 'scripts'))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

import nao_trajectory_schema as schema  # noqa: E402
import phase2a_episode_dataset as ep  # noqa: E402

_RESULTS = []


def check(name, condition, detail=None):
    _RESULTS.append({'name': name, 'pass': bool(condition), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if condition else 'FAIL', name,
                           '' if condition else ' :: %s' % (detail,)))


# ---------------------------------------------------------------------------
# fixtures: a valid frozen run directory (manifest.json + samples.jsonl)
# ---------------------------------------------------------------------------
DEFAULT_POSITION = [0.0] * 24
DEFAULT_POSITION[11] = -0.0349066   # LElbowRoll lower limit
DEFAULT_POSITION[22] = 0.0349066    # RElbowRoll upper limit

_JOINT_ORDER = list(schema.JOINT_ORDER)
_LIMITS = schema.LIMITS
_HOLD_MAX = schema.COMMAND_HOLD_MAX


def valid_sample(t):
    return {
        'timestamp': float(t),
        'joint_position': [0.0] * 24,
        'joint_velocity': [0.0] * 24,
        'imu': {'stamp': float(t), 'orientation': [0.0, 0.0, 0.0, 1.0],
                'angular_velocity': [0.0, 0.0, 0.0],
                'linear_acceleration': [0.0, 0.0, 9.81]},
        'left_contact': False,
        'right_contact': False,
        'command': {'stamp': float(t), 'position': list(DEFAULT_POSITION)},
    }


def make_run(root, name='run', n=3, manifest=None, samples=None):
    run_dir = os.path.join(root, name)
    os.makedirs(run_dir, exist_ok=True)
    with open(os.path.join(run_dir, 'manifest.json'), 'w', encoding='utf-8') as fh:
        json.dump(schema.build_manifest() if manifest is None else manifest,
                  fh, indent=2, allow_nan=False)
    rows = samples if samples is not None else \
        [valid_sample(1.0 + 0.01 * i) for i in range(n)]
    with open(os.path.join(run_dir, 'samples.jsonl'), 'w', encoding='utf-8') as fh:
        for row in rows:
            fh.write(schema.strict_dumps(row) + '\n')
    return run_dir


def valid_metadata(episode_id, **overrides):
    base = {
        'episode_id': episode_id,
        'controller_name': 'phase4b_producer',
        'seed': 17,
        'param_set': {
            'step_length_m': 0.02,
            'step_frequency_hz': 0.5,
            'swing_height_m': 0.008,
            'double_support_ratio': 0.30,
            'sway_amplitude_rad': 0.06,
            'torso_pitch_rad': 0.0,
        },
        'duration_sec': 12.0,
        'fell': False,
        'fall_time': -1.0,
        'mean_speed_x_mps': 0.0046,
        'max_torso_tilt_deg': 1.5,
        'energy_qdot': 12.3,
        'success': True,
        'usable': True,
        'failure_reason': None,
    }
    base.update(overrides)
    return base


def build_dataset(tmp, ids, splits=None, overwrite=False, name='ds'):
    inputs = []
    for i, eid in enumerate(ids):
        inputs.append(ep.EpisodeInput(eid, make_run(tmp, 'run_%s' % eid),
                                      valid_metadata(eid),
                                      split=(splits[i] if splits else None)))
    ds = ep.EpisodeDataset(os.path.join(tmp, name))
    report = ds.build(inputs, dataset_name='unit_test',
                      sample_schema={'name': 'nao_gazebo_trajectory',
                                     'version': '1.0.1'},
                      split_seed=7, overwrite=overwrite)
    return ds, report


_episode_counter = [0]


def _next_id():
    _episode_counter[0] += 1
    return 'episode_%06d' % _episode_counter[0]


# ---------------------------------------------------------------------------
# 1 + 4: write, read, index location
# ---------------------------------------------------------------------------
def test_build_and_read():
    tmp = tempfile.mkdtemp(prefix='p2a_read_')
    try:
        eid = _next_id()
        run = make_run(tmp, 'run1', n=4)
        ds = ep.EpisodeDataset(os.path.join(tmp, 'ds'))
        ds.build([ep.EpisodeInput(eid, run, valid_metadata(eid))],
                 dataset_name='unit', sample_schema={'name': 'nao_gazebo_trajectory',
                                                     'version': '1.0.1'},
                 split_seed=0)
        manifest = ds.load_manifest()
        index = ds.load_index()
        check('build_manifest_counts',
              manifest['episode_count'] == 1 and manifest['sample_count'] == 4,
              manifest)
        check('build_index_one_row', len(index) == 1 and index[0]['episode_id'] == eid,
              index)
        check('index_locates_metadata',
              index[0]['metadata'] == 'episodes/%s/metadata.json' % eid
              and os.path.isfile(ds.metadata_path(eid)), index[0])
        check('index_locates_samples',
              index[0]['samples'] == 'episodes/%s/samples.jsonl' % eid
              and os.path.isfile(ds.samples_path(eid)), index[0])
        check('index_summary_fields',
              index[0]['sample_count'] == 4 and index[0]['usable'] is True
              and index[0]['success'] is True, index[0])
        meta = ds.load_metadata(eid)
        check('metadata_round_trip',
              meta['episode_id'] == eid and meta['param_set']['step_length_m'] == 0.02,
              meta)
        check('metadata_carries_sample_manifest',
              meta['sample_manifest']['schema_version'] == '1.0.1')
        check('self_validate_ok', ds.validate()['valid'], ds.validate()['errors'][:2])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ---------------------------------------------------------------------------
# 2: sample preservation and metadata isolation
# ---------------------------------------------------------------------------
def test_samples_preserved():
    tmp = tempfile.mkdtemp(prefix='p2a_samp_')
    try:
        eid = _next_id()
        src = make_run(tmp, 'run1', n=5)
        ds = ep.EpisodeDataset(os.path.join(tmp, 'ds'))
        ds.build([ep.EpisodeInput(eid, src, valid_metadata(eid))],
                 dataset_name='unit', sample_schema={'name': 'nao_gazebo_trajectory',
                                                     'version': '1.0.1'})
        src_bytes = open(os.path.join(src, 'samples.jsonl'), 'rb').read()
        dst_bytes = open(ds.samples_path(eid), 'rb').read()
        check('samples_byte_identical', src_bytes == dst_bytes)
        rows = [json.loads(l) for l in dst_bytes.decode().splitlines()]
        check('samples_keys_frozen',
              all(list(r.keys()) == schema.SAMPLE_KEYS for r in rows), rows[0].keys())
        leaked = [k for r in rows for k in r
                  if k in ('episode_id', 'success', 'usable', 'param_set',
                           'seed', 'controller_name', 'fell')]
        check('metadata_not_in_samples', not leaked, leaked)
        check('samples_values_unchanged',
              rows[0] == valid_sample(1.0), rows[0])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ---------------------------------------------------------------------------
# 3: metadata validation
# ---------------------------------------------------------------------------
def _errors_for(meta, eid=None):
    return ep.validate_metadata(meta, expected_episode_id=eid)


def test_metadata_validation():
    good = valid_metadata('episode_000001')
    check('meta_valid', _errors_for(good, 'episode_000001') == [])
    for key in ep.REQUIRED_METADATA_KEYS:
        bad = copy.deepcopy(good)
        del bad[key]
        reasons = {e['reason'] for e in _errors_for(bad)}
        check('meta_missing_%s' % key, 'missing' in reasons, sorted(reasons))
    check('meta_bad_id',
          any(e['reason'] == 'invalid_id'
              for e in _errors_for(valid_metadata('ep1'))))
    check('meta_id_mismatch',
          any(e['reason'] == 'id_mismatch'
              for e in _errors_for(good, 'episode_000002')))
    check('meta_seed_type',
          any(e['reason'] == 'not_an_integer'
              for e in _errors_for(valid_metadata('episode_000001', seed=True))))
    check('meta_seed_float',
          any(e['reason'] == 'not_an_integer'
              for e in _errors_for(valid_metadata('episode_000001', seed=1.5))))
    check('meta_param_missing_key',
          any(e['reason'] == 'missing' and e['field'] == 'param_set.torso_pitch_rad'
              for e in _errors_for(valid_metadata(
                  'episode_000001',
                  param_set={'step_length_m': 0.02}))))
    check('meta_param_stringified',
          any(e['reason'] == 'not_an_object'
              for e in _errors_for(valid_metadata('episode_000001',
                                                  param_set='0.02,0.5'))))
    check('meta_param_nonfinite',
          any(e['reason'] == 'non_finite'
              for e in _errors_for(valid_metadata(
                  'episode_000001',
                  param_set=dict(valid_metadata('episode_000001')['param_set'],
                                 step_length_m=float('nan'))))))
    check('meta_duration_nonpositive',
          any(e['reason'] == 'not_positive_finite'
              for e in _errors_for(valid_metadata('episode_000001', duration_sec=0.0))))
    check('meta_tilt_negative',
          any(e['reason'] == 'not_nonnegative_finite'
              for e in _errors_for(valid_metadata('episode_000001',
                                                  max_torso_tilt_deg=-1.0))))
    check('meta_energy_negative',
          any(e['reason'] == 'not_nonnegative_finite'
              for e in _errors_for(valid_metadata('episode_000001',
                                                  energy_qdot=-0.1))))

    # fall_time / state relationships
    check('meta_fell_true_neg_fall_time',
          any(e['reason'] == 'fell_true_requires_nonnegative'
              for e in _errors_for(valid_metadata(
                  'episode_000001', fell=True, fall_time=-1.0,
                  success=False, usable=False, failure_reason='fall'))))
    check('meta_not_fell_wrong_fall_time',
          any(e['reason'] == 'not_fell_requires_minus_one'
              for e in _errors_for(valid_metadata('episode_000001', fall_time=3.0))))
    check('meta_success_with_fell',
          any(e['reason'] == 'success_forbids_fell'
              for e in _errors_for(valid_metadata(
                  'episode_000001', fell=True, fall_time=2.0))))
    # Phase 2D: a failed-but-complete episode may be usable (sample usability is
    # independent of success).
    check('meta_usable_without_success_allowed',
          _errors_for(valid_metadata(
              'episode_000001', success=False, usable=True,
              failure_reason='x')) == [])
    check('meta_success_with_reason',
          any(e['reason'] == 'success_requires_null'
              for e in _errors_for(valid_metadata(
                  'episode_000001', failure_reason='oops'))))
    check('meta_failure_without_reason',
          any(e['reason'] == 'failure_requires_reason'
              for e in _errors_for(valid_metadata(
                  'episode_000001', success=False, usable=False,
                  failure_reason=None))))
    check('meta_fall_after_duration',
          any(e['reason'] == 'after_duration'
              for e in _errors_for(valid_metadata(
                  'episode_000001', fell=True, fall_time=99.0,
                  success=False, usable=False, failure_reason='fall'))))
    # a consistent failure episode is accepted
    check('meta_failure_consistent',
          _errors_for(valid_metadata(
              'episode_000001', fell=True, fall_time=4.2, success=False,
              usable=False, failure_reason='fall')) == [])

    try:
        ep.build_metadata('episode_000001', 'c', 0, {'step_length_m': 1.0},
                          1.0, False, -1.0, 0.0, 0.0, 0.0, True, True)
        built = True
    except ep.MetadataValidationError:
        built = False
    check('build_metadata_rejects', not built)


# ---------------------------------------------------------------------------
# 5: structural failures
# ---------------------------------------------------------------------------
def _write_jsonl(path, rows):
    with open(path, 'w', encoding='utf-8') as fh:
        for row in rows:
            fh.write(json.dumps(row, separators=(',', ':')) + '\n')


def test_structural_failures():
    tmp = tempfile.mkdtemp(prefix='p2a_struct_')
    try:
        eid = _next_id()
        ds, _ = build_dataset(tmp, [eid])
        check('baseline_valid', ds.validate()['valid'])

        # duplicate episode id at build time
        dup_ok = False
        try:
            ep.EpisodeDataset(os.path.join(tmp, 'dup')).build(
                [ep.EpisodeInput(eid, make_run(tmp, 'r1'), valid_metadata(eid)),
                 ep.EpisodeInput(eid, make_run(tmp, 'r2'), valid_metadata(eid))],
                dataset_name='d', sample_schema={'name': 'nao_gazebo_trajectory',
                                                 'version': '1.0.1'})
        except ep.EpisodeDatasetError as exc:
            dup_ok = 'duplicate' in str(exc)
        check('duplicate_episode_rejected', dup_ok)

        # illegal explicit split
        badsplit_ok = False
        try:
            ep.EpisodeDataset(os.path.join(tmp, 'badsplit')).build(
                [ep.EpisodeInput(eid, make_run(tmp, 'r3'), valid_metadata(eid),
                                 split='test')],
                dataset_name='d', sample_schema={'name': 'nao_gazebo_trajectory',
                                                 'version': '1.0.1'})
        except ep.EpisodeDatasetError:
            badsplit_ok = True
        check('illegal_explicit_split_rejected', badsplit_ok)

        # path escape in index
        idx_path = ds.index_path
        rows = [json.loads(l) for l in open(idx_path)]
        rows[0]['samples'] = '../outside.jsonl'
        _write_jsonl(idx_path, rows)
        rep = ds.validate()
        check('index_path_escape_detected',
              not rep['valid'] and 'path_unsafe' in {e['reason'] for e in rep['errors']},
              rep['errors'][:2])
        # restore
        build_dataset(tmp, [eid], overwrite=True)

        # missing referenced file
        os.unlink(ds.samples_path(eid))
        rep = ds.validate()
        check('missing_samples_detected',
              not rep['valid']
              and 'referenced_file_missing' in {e['reason'] for e in rep['errors']},
              rep['errors'][:2])
        build_dataset(tmp, [eid], overwrite=True)

        # illegal split in index
        rows = [json.loads(l) for l in open(idx_path)]
        rows[0]['split'] = 'test'
        _write_jsonl(idx_path, rows)
        rep = ds.validate()
        check('illegal_split_detected',
              not rep['valid'] and 'illegal_split' in {e['reason'] for e in rep['errors']},
              rep['errors'][:2])
        build_dataset(tmp, [eid], overwrite=True)

        # duplicate entry in a split file (the episode's own split)
        split = ds.load_index()[0]['split']
        split_path = ds.split_path(split)
        with open(split_path, 'a', encoding='utf-8') as fh:
            fh.write(json.dumps({'episode_id': eid}) + '\n')
        rep = ds.validate()
        dup_reasons = {e['reason'] for e in rep['errors']}
        check('duplicate_split_detected',
              not rep['valid']
              and ({'duplicate_in_split', 'episode_in_multiple_splits'}
                   & dup_reasons),
              rep['errors'][:3])
        build_dataset(tmp, [eid], overwrite=True)

        # episode missing from splits
        with open(split_path, 'w', encoding='utf-8') as fh:
            fh.write('')
        rep = ds.validate()
        check('missing_episode_in_splits_detected',
              not rep['valid']
              and 'episode_missing_from_splits' in {e['reason'] for e in rep['errors']},
              rep['errors'][:3])
        build_dataset(tmp, [eid], overwrite=True)

        # unindexed episode directory
        os.makedirs(ds.episode_dir('episode_999999'))
        rep = ds.validate()
        check('unindexed_episode_detected',
              not rep['valid'] and 'unindexed_episode' in {e['reason']
                                                           for e in rep['errors']},
              rep['errors'][:3])
        shutil.rmtree(ds.episode_dir('episode_999999'))

        # symlink escape is rejected by resolve_within
        outside = os.path.join(tmp, 'secret.txt')
        open(outside, 'w').close()
        link = os.path.join(ds.root, 'link.txt')
        os.symlink(outside, link)
        esc = False
        try:
            ep.resolve_within(ds.root, 'link.txt')
        except ep.PathSafetyError:
            esc = True
        check('symlink_escape_detected', esc)

        for bad in ('/abs/path', 'a/../../b', '\\win', '', 'a\x00b'):
            blocked = False
            try:
                ep.normalize_relative_path(bad)
            except ep.PathSafetyError:
                blocked = True
            check('path_reject_%r' % bad, blocked)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ---------------------------------------------------------------------------
# 6: deterministic, mutually exclusive, total splits
# ---------------------------------------------------------------------------
def test_splits():
    ids = ['episode_%06d' % i for i in range(1, 21)]
    plan_a = ep.plan_splits(ids, seed=3, train_ratio=0.7, val_ratio=0.1)
    plan_b = ep.plan_splits(list(reversed(ids)), seed=3, train_ratio=0.7,
                            val_ratio=0.1)
    check('split_order_independent', plan_a == plan_b,
          (plan_a, plan_b))
    counts = {s: sum(1 for v in plan_a.values() if v == s) for s in ep.SPLITS}
    check('split_total_coverage',
          sum(counts.values()) == len(ids) and counts['train'] == 14
          and counts['val'] == 2 and counts['eval_fixed'] == 4, counts)
    check('split_exclusive',
          len(set(plan_a.keys())) == len(ids)
          and all(v in ep.SPLITS for v in plan_a.values()))

    tmp = tempfile.mkdtemp(prefix='p2a_split_')
    try:
        small = ids[:6]
        ds, _ = build_dataset(tmp, small, name='ds1')
        first = {s: open(ds.split_path(s), 'rb').read() for s in ep.SPLITS}
        # rebuild identically and compare
        ds2, _ = build_dataset(tmp, small, name='ds2')
        second = {s: open(ds2.split_path(s), 'rb').read() for s in ep.SPLITS}
        check('split_files_deterministic', first == second, first)
        check('eval_fixed_deterministic',
              ep.plan_splits(small, seed=7) == ep.plan_splits(small, seed=7))
        # overlap check across files
        all_ids = []
        for s in ep.SPLITS:
            all_ids += [r['episode_id'] for r in ds.load_splits()[s]]
        check('split_files_disjoint_total',
              sorted(all_ids) == sorted(small), sorted(all_ids))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ---------------------------------------------------------------------------
# 7: legacy frozen data still readable the old way
# ---------------------------------------------------------------------------
def test_legacy_compat():
    tmp = tempfile.mkdtemp(prefix='p2a_legacy_')
    try:
        src = make_run(tmp, 'run_legacy', n=4)
        validator_path = os.path.join(_TOOLS, 'validate_task4_phase4d_dataset.py')
        spec = importlib.util.spec_from_file_location('golden_val', validator_path)
        golden = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(golden)
        report = golden.validate_dataset(src)
        check('legacy_golden_validator_still_accepts', report['valid'],
              report['errors'][:2])
        manifest, rows = ep.read_frozen_run(src)
        check('legacy_read_frozen_run',
              manifest['schema_version'] == '1.0.1' and len(rows) == 4)
        eid = _next_id()
        ds = ep.EpisodeDataset(os.path.join(tmp, 'ds'))
        ds.build([ep.EpisodeInput(eid, src, valid_metadata(eid))],
                 dataset_name='unit', sample_schema={'name': 'nao_gazebo_trajectory',
                                                     'version': '1.0.1'})
        # the original run is untouched
        check('legacy_source_untouched',
              open(os.path.join(src, 'samples.jsonl'), 'rb').read()
              == open(ds.samples_path(eid), 'rb').read())
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ---------------------------------------------------------------------------
# 8: failed build is atomic
# ---------------------------------------------------------------------------
def test_atomic_failure():
    tmp = tempfile.mkdtemp(prefix='p2a_atomic_')
    try:
        eid = _next_id()
        bad_manifest = schema.build_manifest()
        bad_manifest['schema_version'] = '9.9.9'
        bad_run = make_run(tmp, 'bad_run', manifest=bad_manifest)
        target = os.path.join(tmp, 'ds')
        raised = False
        try:
            ep.EpisodeDataset(target).build(
                [ep.EpisodeInput(eid, bad_run, valid_metadata(eid))],
                dataset_name='d', sample_schema={'name': 'nao_gazebo_trajectory',
                                                 'version': '1.0.1'})
        except ep.EpisodeDatasetError:
            raised = True
        check('invalid_run_build_raises', raised)
        check('failed_build_no_dataset', not os.path.exists(target))
        check('failed_build_no_staging',
              not any(n.startswith('.phase2a_staging_') for n in os.listdir(tmp)),
              os.listdir(tmp))

        # invalid metadata also fails atomically
        good_run = make_run(tmp, 'good_run')
        raised = False
        try:
            ep.EpisodeDataset(target).build(
                [ep.EpisodeInput(eid, good_run, valid_metadata(eid, fell=True))],
                dataset_name='d', sample_schema={'name': 'nao_gazebo_trajectory',
                                                 'version': '1.0.1'})
        except (ep.MetadataValidationError, ep.EpisodeDatasetError):
            raised = True
        check('invalid_metadata_build_raises', raised)
        check('failed_metadata_build_no_dataset', not os.path.exists(target))
        check('failed_metadata_build_no_staging',
              not any(n.startswith('.phase2a_staging_') for n in os.listdir(tmp)))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ---------------------------------------------------------------------------
# 9: exact JSON/JSONL round-trip
# ---------------------------------------------------------------------------
def test_round_trip():
    tmp = tempfile.mkdtemp(prefix='p2a_rt_')
    try:
        eid = _next_id()
        ds, _ = build_dataset(tmp, [eid])
        manifest = ds.load_manifest()
        index = ds.load_index()
        meta = ds.load_metadata(eid)
        # re-serialize and re-parse: values must be identical
        check('manifest_round_trip',
              json.loads(json.dumps(manifest, allow_nan=False)) == manifest)
        check('index_round_trip',
              json.loads(json.dumps(index, allow_nan=False)) == index)
        check('metadata_round_trip',
              json.loads(json.dumps(meta, allow_nan=False)) == meta)
        # manifest/index/splits are sorted and stable
        ids = [r['episode_id'] for r in index]
        check('index_sorted', ids == sorted(ids), ids)
        check('no_nan_in_manifest',
              'NaN' not in json.dumps(manifest, allow_nan=False))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ---------------------------------------------------------------------------
# P1: malformed index/metadata must fail structurally, never crash
# ---------------------------------------------------------------------------
def _canonical_params(**overrides):
    params = {
        'step_length_m': 0.02,
        'step_frequency_hz': 0.5,
        'swing_height_m': 0.008,
        'double_support_ratio': 0.30,
        'sway_amplitude_rad': 0.06,
        'torso_pitch_rad': 0.0,
    }
    params.update(overrides)
    return params


def test_param_set_types():
    check('canonical_valid_passes',
          ep.validate_param_set(_canonical_params()) == [])
    for value, label in (('fast', 'string'), (True, 'bool'), (False, 'bool_false'),
                         (float('nan'), 'nan'), (float('inf'), 'inf'),
                         (float('-inf'), 'neg_inf'), (None, 'none')):
        errors = ep.validate_param_set(_canonical_params(step_length_m=value))
        check('canonical_rejects_%s' % label,
              any(e['field'] == 'param_set.step_length_m' for e in errors),
              errors)
    # spot-check the stable reasons
    check('canonical_string_reason',
          ep.validate_param_set(
              _canonical_params(step_length_m='fast'))[0]['reason']
          == 'canonical_not_number')
    check('canonical_bool_reason',
          ep.validate_param_set(
              _canonical_params(step_length_m=True))[0]['reason']
          == 'canonical_not_number')
    check('canonical_nan_reason',
          ep.validate_param_set(
              _canonical_params(step_length_m=float('nan')))[0]['reason']
          == 'non_finite')
    check('canonical_inf_reason',
          ep.validate_param_set(
              _canonical_params(step_length_m=float('inf')))[0]['reason']
          == 'non_finite')
    # extension keys keep the duck-typed policy
    ext = _canonical_params(gait_method='walk', flag=True, extra_ratio=0.1)
    check('extension_keys_allowed', ep.validate_param_set(ext) == [], ext)
    check('extension_nested_rejected',
          any(e['reason'] == 'unsupported_type'
              for e in ep.validate_param_set(
                  _canonical_params(statistics={'a': 1}))))
    # torso_pitch_rad contract: explicit value is accepted, missing is rejected
    check('torso_pitch_explicit_ok',
          ep.validate_param_set(_canonical_params(torso_pitch_rad=0.0)) == [])
    check('torso_pitch_missing_rejected',
          any(e['field'] == 'param_set.torso_pitch_rad' and e['reason'] == 'missing'
              for e in ep.validate_param_set(
                  {k: v for k, v in _canonical_params().items()
                   if k != 'torso_pitch_rad'})))
    # validate_metadata path uses the same rule
    check('metadata_canonical_string_rejected',
          any(e['field'] == 'param_set.step_length_m'
              for e in _errors_for(valid_metadata(
                  'episode_000001',
                  param_set=_canonical_params(step_length_m='fast')))))


def test_failure_reason_whitespace():
    check('failure_reason_blank_rejected',
          any(e['field'] == 'failure_reason'
              for e in _errors_for(valid_metadata(
                  'episode_000001', success=False, usable=False,
                  failure_reason='   '))))
    check('failure_reason_tab_rejected',
          any(e['reason'] == 'failure_requires_reason'
              for e in _errors_for(valid_metadata(
                  'episode_000001', success=False, usable=False,
                  failure_reason='\t'))))
    check('failure_reason_text_ok',
          _errors_for(valid_metadata(
              'episode_000001', success=False, usable=False,
              failure_reason='fell_on_step')) == [])
    check('failure_reason_null_still_means_none',
          _errors_for(valid_metadata('episode_000001')) == [])


def _corrupt_index(ds, mutate):
    with open(ds.index_path, 'r', encoding='utf-8') as fh:
        rows = [json.loads(l) for l in fh if l.strip()]
    rows = mutate(rows)
    with open(ds.index_path, 'w', encoding='utf-8') as fh:
        for row in rows:
            fh.write(json.dumps(row, separators=(',', ':')) + '\n')


def test_malformed_validate():
    tmp = tempfile.mkdtemp(prefix='p2a_malformed_')
    try:
        eid = _next_id()
        ds, _ = build_dataset(tmp, [eid])

        # 1. index line is a JSON string
        with open(ds.index_path, 'w', encoding='utf-8') as fh:
            fh.write('"nonsense"\n')
        rep = _safe_validate(ds)
        check('malformed_index_string_no_crash', rep is not None)
        check('malformed_index_string_invalid',
              rep is not None and rep['valid'] is False)
        check('malformed_index_string_reason',
              rep is not None
              and 'row_not_object' in {e['reason'] for e in rep['errors']},
              rep['errors'][:2] if rep else None)
        build_dataset(tmp, [eid], overwrite=True)

        # 2. index line missing the metadata key
        def drop_metadata(rows):
            del rows[0]['metadata']
            return rows
        _corrupt_index(ds, drop_metadata)
        rep = _safe_validate(ds)
        check('malformed_index_missing_metadata_no_crash', rep is not None)
        check('malformed_index_missing_metadata_invalid',
              rep is not None and rep['valid'] is False)
        check('malformed_index_missing_metadata_reason',
              rep is not None
              and 'index_keys_mismatch' in {e['reason'] for e in rep['errors']},
              rep['errors'][:2] if rep else None)
        build_dataset(tmp, [eid], overwrite=True)

        # 3. metadata.json is a legal JSON array
        with open(ds.metadata_path(eid), 'w', encoding='utf-8') as fh:
            fh.write('[1, 2, 3]\n')
        rep = _safe_validate(ds)
        check('malformed_metadata_array_no_crash', rep is not None)
        check('malformed_metadata_array_invalid',
              rep is not None and rep['valid'] is False)
        check('malformed_metadata_array_reason',
              rep is not None
              and 'metadata_not_object' in {e['reason'] for e in rep['errors']},
              rep['errors'][:2] if rep else None)
        build_dataset(tmp, [eid], overwrite=True)

        # 4. metadata.json is a JSON string
        with open(ds.metadata_path(eid), 'w', encoding='utf-8') as fh:
            fh.write('"just a string"\n')
        rep = _safe_validate(ds)
        check('malformed_metadata_string_no_crash',
              rep is not None and rep['valid'] is False)
        check('malformed_metadata_string_reason',
              rep is not None
              and 'metadata_not_object' in {e['reason'] for e in rep['errors']})

        # 5. many malformed rows at once: all detected, no crash
        build_dataset(tmp, [eid], overwrite=True)
        with open(ds.index_path, 'w', encoding='utf-8') as fh:
            fh.write('"nonsense"\n')
            fh.write('{"episode_id": "episode_000001"}\n')
            fh.write('42\n')
        rep = _safe_validate(ds)
        reasons = {e['reason'] for e in rep['errors']} if rep else set()
        check('malformed_multiple_detected',
              rep is not None and rep['valid'] is False
              and 'row_not_object' in reasons and 'index_keys_mismatch' in reasons,
              sorted(reasons))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _safe_validate(ds):
    try:
        return ds.validate()
    except Exception as exc:  # noqa: BLE001 - the test asserts it never happens
        _RESULTS.append({'name': 'validate_no_exception', 'pass': False,
                         'detail': '%s: %s' % (type(exc).__name__, exc)})
        print('  [FAIL] validate raised %s: %s' % (type(exc).__name__, exc))
        return None


def test_cli_validate_exit_codes():
    import subprocess
    tmp = tempfile.mkdtemp(prefix='p2a_cli_')
    module = os.path.join(_TOOLS, 'phase2a_episode_dataset.py')

    def run_cli(dataset):
        return subprocess.run([sys.executable, module, 'validate',
                               '--dataset', dataset],
                              capture_output=True, text=True)

    try:
        eid = _next_id()
        ds, _ = build_dataset(tmp, [eid])
        good = run_cli(ds.root)
        check('cli_valid_exit_zero', good.returncode == 0, good.returncode)
        check('cli_valid_no_traceback', 'Traceback' not in good.stderr)

        # malformed: index string row
        with open(ds.index_path, 'w', encoding='utf-8') as fh:
            fh.write('"nonsense"\n')
        bad1 = run_cli(ds.root)
        check('cli_malformed_index_exit_one', bad1.returncode == 1, bad1.returncode)
        check('cli_malformed_index_no_traceback',
              'Traceback' not in bad1.stderr and 'Traceback' not in bad1.stdout,
              bad1.stderr[-300:])
        check('cli_malformed_index_json_error',
              json.loads(bad1.stdout)['valid'] is False)

        # malformed: index missing metadata key
        build_dataset(tmp, [eid], overwrite=True)
        _corrupt_index(ds, _drop_metadata_key)
        bad2 = run_cli(ds.root)
        check('cli_missing_metadata_exit_one', bad2.returncode == 1, bad2.returncode)
        check('cli_missing_metadata_no_traceback',
              'Traceback' not in bad2.stderr, bad2.stderr[-300:])

        # malformed: metadata is an array
        build_dataset(tmp, [eid], overwrite=True)
        with open(ds.metadata_path(eid), 'w', encoding='utf-8') as fh:
            fh.write('[1, 2, 3]\n')
        bad3 = run_cli(ds.root)
        check('cli_metadata_array_exit_one', bad3.returncode == 1, bad3.returncode)
        check('cli_metadata_array_no_traceback',
              'Traceback' not in bad3.stderr, bad3.stderr[-300:])

        # CLI build rejects a non-numeric canonical param structurally
        bad_run = make_run(tmp, 'bad_param_run')
        spec = {'dataset_name': 'd',
                'sample_schema': {'name': 'nao_gazebo_trajectory',
                                  'version': '1.0.1'},
                'episodes': [{'episode_id': 'episode_000777',
                              'run_dir': bad_run,
                              'metadata': valid_metadata(
                                  'episode_000777',
                                  param_set=_canonical_params(
                                      step_length_m='fast'))}]}
        spec_path = os.path.join(tmp, 'spec.json')
        with open(spec_path, 'w', encoding='utf-8') as fh:
            json.dump(spec, fh)
        built = subprocess.run(
            [sys.executable, module, 'build',
             '--dataset', os.path.join(tmp, 'bad_ds'), '--spec', spec_path],
            capture_output=True, text=True)
        check('cli_bad_param_build_exit_nonzero', built.returncode != 0,
              built.returncode)
        check('cli_bad_param_build_structured',
              'canonical_not_number' in (built.stdout + built.stderr),
              (built.stdout + built.stderr)[-300:])
        check('cli_bad_param_build_no_dataset',
              not os.path.exists(os.path.join(tmp, 'bad_ds')))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _drop_metadata_key(rows):
    for row in rows:
        if isinstance(row, dict):
            row.pop('metadata', None)
    return rows


# ---------------------------------------------------------------------------
# P3 (optional): read-side samples_sha256 content integrity
# ---------------------------------------------------------------------------
def test_samples_hash_read_side():
    tmp = tempfile.mkdtemp(prefix='p2a_hash_')
    try:
        eid_a = 'episode_000001'
        eid_b = 'episode_000002'
        inputs = [
            ep.EpisodeInput(eid_a, make_run(tmp, 'run_a', n=3),
                            valid_metadata(eid_a)),
            ep.EpisodeInput(eid_b, make_run(tmp, 'run_b', n=3),
                            valid_metadata(eid_b)),
        ]
        ds = ep.EpisodeDataset(os.path.join(tmp, 'ds'))
        ds.build(inputs, dataset_name='unit',
                 sample_schema={'name': 'nao_gazebo_trajectory',
                                'version': '1.0.1'})
        meta_a = ds.load_metadata(eid_a)
        check('writer_records_sha256',
              isinstance(meta_a.get('samples_sha256'), str)
              and len(meta_a['samples_sha256']) == 64, meta_a.get('samples_sha256'))
        check('valid_hash_passes', ds.validate()['valid'])

        # 1. content modified, line count unchanged
        with open(ds.samples_path(eid_a), 'r', encoding='utf-8') as fh:
            rows = [json.loads(l) for l in fh if l.strip()]
        rows[1]['joint_position'][0] = 0.5
        with open(ds.samples_path(eid_a), 'w', encoding='utf-8') as fh:
            for row in rows:
                fh.write(schema.strict_dumps(row) + '\n')
        rep = ds.validate()
        check('hash_mismatch_content_detected',
              not rep['valid']
              and 'samples_hash_mismatch' in {e['reason'] for e in rep['errors']},
              rep['errors'][:3])
        check('hash_mismatch_line_count_unchanged',
              ds.load_index()[0]['sample_count'] == len(rows))
        # Distinct content (same line count) so a path swap is detectable.
        run_a2 = make_run(tmp, 'run_a2',
                          samples=[valid_sample(3.0 + 0.01 * i) for i in range(3)])
        run_b2 = make_run(tmp, 'run_b2',
                          samples=[valid_sample(4.0 + 0.01 * i) for i in range(3)])
        ds.build([ep.EpisodeInput(eid_a, run_a2, valid_metadata(eid_a)),
                  ep.EpisodeInput(eid_b, run_b2, valid_metadata(eid_b))],
                 dataset_name='unit',
                 sample_schema={'name': 'nao_gazebo_trajectory',
                                'version': '1.0.1'},
                 overwrite=True)
        check('distinct_hash_valid', ds.validate()['valid'])

        # 2. index samples paths swapped between episodes
        idx = ds.load_index()
        idx[0]['samples'], idx[1]['samples'] = idx[1]['samples'], idx[0]['samples']
        with open(ds.index_path, 'w', encoding='utf-8') as fh:
            for row in idx:
                fh.write(json.dumps(row, separators=(',', ':')) + '\n')
        rep = ds.validate()
        check('hash_mismatch_swapped_paths_detected',
              not rep['valid']
              and 'samples_hash_mismatch' in {e['reason'] for e in rep['errors']},
              rep['errors'][:3])

        # 3. a legacy dataset without samples_sha256 still validates
        legacy = _next_id()
        ds2 = ep.EpisodeDataset(os.path.join(tmp, 'legacy'))
        run = make_run(tmp, 'run_legacy')
        ds2.build([ep.EpisodeInput(legacy, run, valid_metadata(legacy))],
                  dataset_name='unit',
                  sample_schema={'name': 'nao_gazebo_trajectory',
                                 'version': '1.0.1'})
        meta = ds2.load_metadata(legacy)
        meta.pop('samples_sha256')
        with open(ds2.metadata_path(legacy), 'w', encoding='utf-8') as fh:
            json.dump(meta, fh)
        check('legacy_without_hash_validates', ds2.validate()['valid'])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('Phase 2A episode dataset OFFLINE tests (no simulation)')
    test_build_and_read()
    test_samples_preserved()
    test_metadata_validation()
    test_param_set_types()
    test_failure_reason_whitespace()
    test_malformed_validate()
    test_cli_validate_exit_codes()
    test_samples_hash_read_side()
    test_structural_failures()
    test_splits()
    test_legacy_compat()
    test_atomic_failure()
    test_round_trip()
    passed = sum(1 for r in _RESULTS if r['pass'])
    total = len(_RESULTS)
    print('\nphase 2A offline tests: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as fh:
            json.dump({'offline': True, 'passed': passed, 'total': total,
                       'results': _RESULTS}, fh, indent=2, sort_keys=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
