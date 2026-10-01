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
Phase 2F split OFFLINE tests (no ROS, no Gazebo, no simulation).

Covers: deterministic episode-random and group-aware planning, exclusivity/
totality, eval_fixed stability, leakage audit (group/seed/failure/repeat),
preview-not-final semantics, gated freeze, atomic writes and read-only audit of
a real Phase 2A dataset.

Usage: python3 tools/test_phase2f_split.py [--json-out F]
Exit 0 = all offline tests passed.
"""

import argparse
import copy
import hashlib
import json
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
import nao_trajectory_schema as schema  # noqa: E402
import phase2a_episode_dataset as ep  # noqa: E402
import phase2f_split as F  # noqa: E402

_RESULTS = []
_CLI = os.path.join(_TOOLS, 'phase2f_split.py')
POS = [0.0] * 24
POS[11] = -0.0349066
POS[22] = 0.0349066


def check(name, condition, detail=None):
    _RESULTS.append({'name': name, 'pass': bool(condition), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if condition else 'FAIL', name,
                           '' if condition else ' :: %s' % (detail,)))


def _jsonable(value):
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


# ---------------------------------------------------------------------------
# synthetic episodes + real dataset fixture
# ---------------------------------------------------------------------------
def E(eid, param, seed, source='ds_a', split='train', failure=None,
      success=True, sha=None, count=10):
    return {
        'episode_id': eid,
        'param_set_key': param,
        'config_group_key': param,
        'seed': seed,
        'source_dataset': source,
        'split': split,
        'success': success,
        'usable': True,
        'fell': failure == 'fall',
        'failure_reason': failure,
        'sample_path': '/tmp/%s/samples.jsonl' % eid,
        'sample_sha256': sha or ('%064d' % (int(eid[-1]) + 1)),
        'sample_count': count,
        'baseline_config': None,
    }


def sample(t):
    q = [0.0, 0.0, 0.0, 1.0]
    return {
        'timestamp': float(t),
        'joint_position': list(POS),
        'joint_velocity': [0.1] * 24,
        'imu': {'stamp': float(t), 'orientation': q,
                'angular_velocity': [0.0, 0.0, 0.0],
                'linear_acceleration': [0.0, 0.0, 9.81]},
        'left_contact': False, 'right_contact': True,
        'command': {'stamp': float(t), 'position': list(POS)},
    }


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


def build_ds_at(root, rel, entries, name='p2f_test'):
    inputs = []
    for eid, samples, m in entries:
        inputs.append(ep.EpisodeInput(eid, make_run(root, 'r_' + eid, samples),
                                      m))
    ds = ep.EpisodeDataset(os.path.join(root, rel))
    ds.build(inputs, dataset_name=name,
             sample_schema={'name': 'nao_gazebo_trajectory',
                            'version': '1.0.1'})
    return ds


def _tree_hash(root):
    out = {}
    for r, _, fs in os.walk(root):
        for f in fs:
            p = os.path.join(r, f)
            out[os.path.relpath(p, root)] = hashlib.sha256(
                open(p, 'rb').read()).hexdigest()
    return out


def _run_cli(*args):
    return subprocess.run([sys.executable, _CLI, *args],
                          capture_output=True, text=True)


def _stderr_payload(r):
    try:
        return json.loads(r.stderr.strip())
    except ValueError:
        return None


# ---------------------------------------------------------------------------
# tests
# ---------------------------------------------------------------------------
def test_policy_validation():
    ok = F.merge_policy({})
    check('policy_defaults', ok['split_policy_version'] == F.SPLIT_POLICY_VERSION
          and F.DEFAULT_BATCH0_ROOT in ok['forbid_roots'], ok)
    for bad in ({'train_ratio': 0.9, 'val_ratio': 0.2},
                {'train_ratio': 1.5},
                {'split_seed': 1.5},
                {'eval_fixed_policy': 'nope'},
                {'group_holdout_enabled': True, 'group_holdout_key': 'bogus'},
                {'near_duplicate_rel_tol': -1}):
        raised = False
        try:
            F.merge_policy(bad)
        except F.Phase2FSplitError:
            raised = True
        check('policy_rejects:%s' % sorted(bad), raised)


_CANON = {'step_length_m': 0.02, 'step_frequency_hz': 0.5,
          'swing_height_m': 0.008, 'double_support_ratio': 0.3,
          'sway_amplitude_rad': 0.06, 'torso_pitch_rad': 0.0}


def test_canonical_key_normalization():
    keys = set()
    for zero in (0, 0.0, -0.0, -0):
        ps = dict(_CANON)
        ps['torso_pitch_rad'] = zero
        keys.add(F.canonical_param_set_key(ps))
    check('negative_zero_same_canonical_key', len(keys) == 1, keys)
    check('negative_zero_not_encoded_as_minus',
          all('=-0' not in key and 'torso_pitch_rad=0' in key for key in keys),
          keys)
    check('canonical_input_order_independent',
          F.canonical_param_set_key(_CANON)
          == F.canonical_param_set_key(dict(reversed(list(_CANON.items())))))
    ps = dict(_CANON)
    ps['step_length_m'] = 0.020
    ps['step_frequency_hz'] = 5e-1
    check('canonical_numeric_format_equivalent',
          F.canonical_param_set_key(_CANON) == F.canonical_param_set_key(ps))
    ps = dict(_CANON)
    ps['extra_field'] = 'ignored'
    check('canonical_extra_field_ignored',
          F.canonical_param_set_key(_CANON) == F.canonical_param_set_key(ps))
    ps = dict(_CANON)
    ps.pop('step_frequency_hz')
    raised = False
    try:
        F.canonical_param_set_key(ps)
    except F.Phase2FSplitError:
        raised = True
    check('canonical_missing_key_rejected', raised)


def _write_json(path, obj):
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump(obj, fh)
    return path


def _full_evidence(ds_root, policy_path, baseline_path, review_path):
    _, _, info = F.load_episodes(ds_root)
    ev = {k: True for k in F.GATE2A_EVIDENCE_KEYS}
    ev.update({
        'dataset_root': info['dataset_root'],
        'dataset_aggregate_sha256': info['aggregate_sha256'],
        'dataset_index_sha256': info['index_sha256'],
        'dataset_manifest_sha256': info['manifest_sha256'],
        'file_count': info['file_count'],
        'split_policy_sha256': F.sha256_file(policy_path),
        'evidence_files': {
            'baseline_report': {'path': baseline_path,
                                'sha256': F.sha256_file(baseline_path)},
            'independent_review': {'path': review_path,
                                   'sha256': F.sha256_file(review_path)},
        },
    })
    return ev, info


def _expect_gate_reject(ev, info, policy_sha, label):
    raised = False
    try:
        F.check_gate2a_evidence(ev, info, policy_sha)
    except F.Gate2AEvidenceError:
        raised = True
    check(label, raised)


def test_evidence_binding():
    tmp = tempfile.mkdtemp(prefix='p2f_bind_')
    try:
        ds = build_ds_at(tmp, 'ds', [
            ('episode_000001', [sample(1.0)], meta('episode_000001')),
            ('episode_000002', [sample(2.0)], meta('episode_000002'))])
        policy_path = _write_json(os.path.join(tmp, 'policy.json'), {
            'group_holdout_enabled': False, 'eval_fixed_policy': 'explicit',
            'eval_fixed_episodes': ['episode_000002']})
        baseline = _write_json(os.path.join(tmp, 'baseline.NOT_REAL.json'),
                               {'placeholder': 'baseline report stand-in'})
        review = _write_json(os.path.join(tmp, 'review.NOT_REAL.json'),
                             {'placeholder': 'review stand-in'})
        ev, info = _full_evidence(ds.root, policy_path, baseline, review)
        psha = F.sha256_file(policy_path)
        F.check_gate2a_evidence(ev, info, psha)
        check('binding_accepts_full_evidence', True)

        e = copy.deepcopy(ev)
        e.pop('dataset_aggregate_sha256')
        _expect_gate_reject(e, info, psha, 'binding_rejects_missing_hash_field')

        e = copy.deepcopy(ev)
        e['dataset_index_sha256'] = '0' * 64
        _expect_gate_reject(e, info, psha, 'binding_rejects_hash_mismatch')

        e = copy.deepcopy(ev)
        e['dataset_root'] = '/some/other/dataset'
        _expect_gate_reject(e, info, psha, 'binding_rejects_wrong_dataset_root')

        e = copy.deepcopy(ev)
        e['split_policy_sha256'] = '0' * 64
        _expect_gate_reject(e, info, psha, 'binding_rejects_policy_hash_mismatch')

        e = copy.deepcopy(ev)
        e['evidence_files'] = {'baseline_report':
                               ev['evidence_files']['baseline_report']}
        _expect_gate_reject(e, info, psha, 'binding_rejects_missing_role')

        e = copy.deepcopy(ev)
        e['evidence_files']['baseline_report']['sha256'] = '0' * 64
        _expect_gate_reject(e, info, psha, 'binding_rejects_file_hash_mismatch')

        e = copy.deepcopy(ev)
        e['evidence_files']['independent_review']['path'] = \
            os.path.join(tmp, 'nope.json')
        _expect_gate_reject(e, info, psha, 'binding_rejects_missing_file')

        e = copy.deepcopy(ev)
        e['tiers_ok'] = 1
        _expect_gate_reject(e, info, psha, 'binding_rejects_non_bool_true')

        # CLI freeze with the fully bound evidence writes a final manifest.
        ev_path = _write_json(os.path.join(tmp, 'evidence.json'), ev)
        out = os.path.join(tmp, 'final.json')
        r = _run_cli('freeze', '--dataset', ds.root, '--policy', policy_path,
                     '--gate2a-evidence', ev_path, '--manifest-out', out)
        check('cli_freeze_bound_evidence_exit0',
              r.returncode == 0 and os.path.isfile(out), r.stderr[-200:])
        if os.path.isfile(out):
            m = json.load(open(out))
            check('cli_freeze_bound_final_manifest',
                  m['status'] == 'final' and m['final'] is True)
        # tampered dataset hash at CLI level is rejected, no overwrite
        bad = os.path.join(tmp, 'tampered.json')
        e = copy.deepcopy(ev)
        e['dataset_manifest_sha256'] = '0' * 64
        _write_json(bad, e)
        out2 = os.path.join(tmp, 'final2.json')
        r = _run_cli('freeze', '--dataset', ds.root, '--policy', policy_path,
                     '--gate2a-evidence', bad, '--manifest-out', out2)
        check('cli_freeze_tampered_hash_exit2',
              r.returncode == 2 and not os.path.isfile(out2)
              and _stderr_payload(r).get('ok') is False, r.stderr[-200:])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_evaluation_views_labeled():
    episodes = [E('episode_%06d' % i, 'p%d' % (i % 3), 100 + i)
                for i in range(1, 13)]
    info = {'dataset_root': '/tmp/x', 'aggregate_sha256': '0' * 64,
            'index_sha256': '1' * 64, 'manifest_sha256': '2' * 64,
            'file_count': 1, 'dataset_under_forbidden_root': [],
            'batch0_contaminated_episodes': []}
    pa = F.merge_policy({'group_holdout_enabled': False})
    pb = F.merge_policy({'group_holdout_enabled': True,
                         'group_holdout_key': 'param_set_key'})
    ma = F.build_manifest('/tmp/x', episodes,
                          F.plan_episode_random(episodes, pa), pa, 'preview',
                          info)
    mb = F.build_manifest('/tmp/x', episodes,
                          F.plan_group_aware(episodes, pb), pb, 'preview',
                          info)
    check('primary_view_label', ma['evaluation_view'] == F.PRIMARY_VIEW)
    check('stress_view_label', mb['evaluation_view'] == F.STRESS_VIEW)
    check('views_distinguishable',
          ma['evaluation_view'] != mb['evaluation_view'])
    check('stress_view_no_param_cross_split',
          mb['leakage']['param_set_cross_split'] == {})
    check('both_previews_not_final',
          ma['final'] is False and mb['final'] is False)


def test_episode_random_determinism():
    episodes = [E('episode_%06d' % i, 'p%d' % (i % 3), 100 + i)
                for i in range(1, 13)]
    policy = F.merge_policy({'split_seed': 7, 'train_ratio': 0.6,
                             'val_ratio': 0.2, 'group_holdout_enabled': False})
    a = F.plan_episode_random(episodes, policy)
    b = F.plan_episode_random(list(reversed(episodes)), policy)
    check('episode_random_order_independent', a == b, (a, b))
    c = F.plan_episode_random(
        episodes, F.merge_policy({'split_seed': 8, 'train_ratio': 0.6,
                                  'val_ratio': 0.2}))
    check('episode_random_seed_changes_plan', a != c)
    # exclusivity and totality
    by = {s: {e['episode_id'] for e in episodes if a[e['episode_id']] == s}
          for s in F.SPLITS}
    all_ids = {e['episode_id'] for e in episodes}
    disjoint = (by['train'] & by['val']) | (by['train'] & by['eval_fixed']) \
        | (by['val'] & by['eval_fixed'])
    check('split_exclusive_total',
          not disjoint and by['train'] | by['val'] | by['eval_fixed'] == all_ids)


def test_group_aware_no_cross_split():
    episodes = []
    n = 0
    for g in range(4):
        for seed in (101, 202, 303):
            n += 1
            episodes.append(E('episode_%06d' % n, 'param_%d' % g, seed))
    policy = F.merge_policy({'split_seed': 11, 'train_ratio': 0.5,
                             'val_ratio': 0.25, 'group_holdout_enabled': True,
                             'group_holdout_key': 'param_set_key'})
    plan = F.plan_group_aware(episodes, policy)
    audit = F.leakage_audit(episodes, plan, policy)
    check('group_aware_no_param_cross_split',
          audit['param_set_cross_split'] == {}, audit['param_set_cross_split'])
    check('group_aware_total',
          sorted(plan) == sorted(e['episode_id'] for e in episodes))
    check('group_aware_episode_units_whole', audit['adjacent_frame_leakage'] == [])
    # episode-random baseline should expose the same key as leaky
    rnd = F.plan_episode_random(
        episodes, F.merge_policy({'split_seed': 11, 'train_ratio': 0.5,
                                  'val_ratio': 0.25}))
    ra = F.leakage_audit(episodes, rnd,
                         F.merge_policy({'group_holdout_enabled': False}))
    check('episode_random_exposes_group_leak',
          isinstance(ra['param_set_cross_split'], dict))


def test_eval_fixed_explicit_stability():
    base = [E('episode_%06d' % i, 'p%d' % (i % 3), 100 + i)
            for i in range(1, 13)]
    policy = F.merge_policy({'split_seed': 3, 'group_holdout_enabled': False,
                             'eval_fixed_policy': 'explicit',
                             'eval_fixed_episodes': ['episode_000002',
                                                     'episode_000005']})
    plan_one = F.plan_episode_random(base, policy)
    check('eval_fixed_explicit_honored',
          plan_one['episode_000002'] == 'eval_fixed'
          and plan_one['episode_000005'] == 'eval_fixed', plan_one)
    grown = base + [E('episode_000013', 'p9', 999)]
    plan_two = F.plan_episode_random(grown, policy)
    check('eval_fixed_stable_under_new_data',
          plan_two['episode_000002'] == 'eval_fixed'
          and plan_two['episode_000005'] == 'eval_fixed', plan_two)
    bad = F.merge_policy({'eval_fixed_policy': 'explicit',
                          'eval_fixed_episodes': ['episode_999999']})
    raised = False
    try:
        F.plan_episode_random(base, bad)
    except F.Phase2FSplitError:
        raised = True
    check('eval_fixed_missing_episode_rejected', raised)


def test_leakage_audit_fields():
    episodes = [E('episode_000001', 'pA', 1001, failure='fall'),
                E('episode_000002', 'pA', 1001, failure='fall'),
                E('episode_000003', 'pB', 1002),
                E('episode_000004', 'pB', 1003)]
    plan = {'episode_000001': 'train', 'episode_000002': 'val',
            'episode_000003': 'train', 'episode_000004': 'eval_fixed'}
    policy = F.merge_policy({'group_holdout_enabled': False})
    audit = F.leakage_audit(episodes, plan, policy)
    check('leak_reports_param_cross_split',
          set(audit['param_set_cross_split']) == {'pA', 'pB'},
          audit['param_set_cross_split'])
    check('leak_reports_seed_cross_split',
          '1001' in audit['seed_cross_split'], audit['seed_cross_split'])
    check('leak_reports_adjacent_none', audit['adjacent_frame_leakage'] == [])
    check('leak_reports_failure_classes',
          audit['observed_failure_classes'] == ['fall'],
          audit['observed_failure_classes'])
    check('leak_reports_counts',
          audit['counts_by_split'] == {'train': 2, 'val': 1, 'eval_fixed': 1},
          audit['counts_by_split'])


def test_manifest_preview_semantics():
    episodes = [E('episode_%06d' % i, 'p%d' % (i % 2), 100 + i)
                for i in range(1, 7)]
    policy = F.merge_policy({'split_seed': 5, 'group_holdout_enabled': False})
    plan = F.plan_episode_random(episodes, policy)
    info = {'dataset_root': '/tmp/x', 'aggregate_sha256': '0' * 64,
            'index_sha256': '1' * 64, 'manifest_sha256': '2' * 64,
            'file_count': 1, 'dataset_under_forbidden_root': [],
            'batch0_contaminated_episodes': []}
    m = F.build_manifest('/tmp/x', episodes, plan, policy, 'preview', info)
    check('manifest_preview_not_final',
          m['status'] == 'preview' and m['final'] is False)
    for key in ('split_manifest_version', 'split_policy_version', 'splits',
                'counts', 'leakage', 'generation_command', 'created_at'):
        check('manifest_has_%s' % key, key in m)
    json.dumps(m, sort_keys=True, allow_nan=False)
    check('manifest_strict_json', True)


def test_gate_booleans():
    good = {k: True for k in F.GATE2A_EVIDENCE_KEYS}
    F.check_gate2a_booleans(good)
    check('gate_accepts_complete_true', True)
    for mutate in ('drop', 'false'):
        ev = dict(good)
        if mutate == 'drop':
            ev.pop('tiers_ok')
        else:
            ev['failure_classes_ok'] = False
        raised = False
        try:
            F.check_gate2a_booleans(ev)
        except F.Gate2AEvidenceError:
            raised = True
        check('gate_rejects_%s' % mutate, raised)
    raised = False
    try:
        F.check_gate2a_evidence(good)   # no dataset binding info
    except F.Gate2AEvidenceError:
        raised = True
    check('gate_requires_binding_info', raised)


def test_cli_freeze_gate_without_binding():
    tmp = tempfile.mkdtemp(prefix='p2f_freeze_')
    try:
        ds = build_ds_at(tmp, 'ds', [
            ('episode_000001', [sample(1.0), sample(1.01)], meta('episode_000001')),
            ('episode_000002', [sample(2.0), sample(2.01)], meta('episode_000002'))])
        policy_path = _write_json(os.path.join(tmp, 'policy.json'), {
            'group_holdout_enabled': False, 'eval_fixed_policy': 'explicit',
            'eval_fixed_episodes': ['episode_000002']})
        out = os.path.join(tmp, 'final.json')
        ev_fail = _write_json(os.path.join(tmp, 'ev_fail.json'),
                              {'episodes_ok': True})
        r = _run_cli('freeze', '--dataset', ds.root, '--policy', policy_path,
                     '--gate2a-evidence', ev_fail, '--manifest-out', out)
        check('cli_freeze_missing_gate_exit2',
              r.returncode == 2 and _stderr_payload(r).get('ok') is False
              and not os.path.isfile(out), (r.returncode, r.stderr[-200:]))
        # all-true booleans but no hash binding -> still rejected, no file
        ev_unbound = _write_json(os.path.join(tmp, 'ev_unbound.json'),
                                 {k: True for k in F.GATE2A_EVIDENCE_KEYS})
        r = _run_cli('freeze', '--dataset', ds.root, '--policy', policy_path,
                     '--gate2a-evidence', ev_unbound, '--manifest-out', out)
        check('cli_freeze_unbound_evidence_exit2',
              r.returncode == 2 and not os.path.isfile(out),
              (r.returncode, r.stderr[-200:]))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_plan_guard_and_preview_cli():
    tmp = tempfile.mkdtemp(prefix='p2f_plan_')
    try:
        ds = build_ds_at(tmp, 'ds', [
            ('episode_000001', [sample(1.0), sample(1.01)], meta('episode_000001')),
            ('episode_000002', [sample(2.0), sample(2.01)], meta('episode_000002')),
            ('episode_000003', [sample(3.0), sample(3.01)], meta('episode_000003')),
            ('episode_000004', [sample(4.0), sample(4.01)], meta('episode_000004'))])
        before = _tree_hash(ds.root)
        policy_path = os.path.join(tmp, 'policy.json')
        with open(policy_path, 'w') as fh:
            json.dump({'split_seed': 1, 'train_ratio': 0.5, 'val_ratio': 0.25,
                       'group_holdout_enabled': True,
                       'group_holdout_key': 'param_set_key'}, fh)
        # guard: manifest into splits/ refused
        r = _run_cli('plan', '--dataset', ds.root, '--policy', policy_path,
                     '--mode', 'preview', '--manifest-out',
                     os.path.join(ds.root, 'splits', 'split_manifest.json'))
        check('plan_refuses_splits_dir', r.returncode == 2,
              (r.returncode, r.stderr[-200:]))
        manifest = os.path.join(tmp, 'preview.json')
        r = _run_cli('plan', '--dataset', ds.root, '--policy', policy_path,
                     '--mode', 'preview', '--manifest-out', manifest,
                     '--json-out', os.path.join(tmp, 'audit.json'))
        check('plan_preview_exit0', r.returncode == 0, r.stderr[-200:])
        m = json.load(open(manifest))
        check('plan_preview_marked_preview',
              m['status'] == 'preview' and m['final'] is False)
        check('plan_does_not_modify_dataset', _tree_hash(ds.root) == before)
        # preview must not overwrite an existing manifest without explicit flag
        r = _run_cli('plan', '--dataset', ds.root, '--policy', policy_path,
                     '--mode', 'preview', '--manifest-out', manifest)
        check('plan_refuses_overwrite', r.returncode == 2)
        # deterministic: same policy -> same manifest episode_to_split
        m2 = os.path.join(tmp, 'preview2.json')
        _run_cli('plan', '--dataset', ds.root, '--policy', policy_path,
                 '--mode', 'preview', '--manifest-out', m2)
        check('plan_deterministic',
              json.load(open(manifest))['episode_to_split']
              == json.load(open(m2))['episode_to_split'])
        check('plan_no_temp_left',
              not [f for f in os.listdir(tmp) if f.startswith('.tmp_p2f_')])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_audit_readonly_and_invalid():
    tmp = tempfile.mkdtemp(prefix='p2f_audit_')
    try:
        ds = build_ds_at(tmp, 'ds', [
            ('episode_000001', [sample(1.0)], meta('episode_000001')),
            ('episode_000002', [sample(2.0)], meta('episode_000002'))])
        before = _tree_hash(ds.root)
        out = os.path.join(tmp, 'audit.json')
        r = _run_cli('audit', '--dataset', ds.root, '--json-out', out)
        check('audit_exit0_and_readonly',
              r.returncode == 0 and os.path.isfile(out)
              and _tree_hash(ds.root) == before, r.stderr[-200:])
        report = json.load(open(out))
        check('audit_has_observed', 'observed' in report)
        for key in ('counts_by_split', 'param_set_cross_split',
                    'seed_cross_split', 'adjacent_frame_leakage'):
            check('audit_has_%s' % key, key in report['observed'])
        r = _run_cli('audit', '--dataset', os.path.join(tmp, 'missing'))
        check('audit_invalid_dataset_exit2',
              r.returncode == 2 and _stderr_payload(r).get('ok') is False,
              r.stderr[-200:])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_batch0_contamination_flagged():
    tmp = tempfile.mkdtemp(prefix='p2f_contam_')
    try:
        entries = [('episode_000001', [sample(1.0)], meta('episode_000001'))]
        cand = build_ds_at(os.path.join(tmp, 'cand'), 'work/dataset', entries,
                           name='cand')
        target = build_ds_at(os.path.join(tmp, 'target'), 'ds', entries,
                             name='target')
        _, episodes, info = F.load_episodes(target.root,
                                            [os.path.join(tmp, 'cand')])
        check('batch0_contamination_detected',
              info['batch0_contaminated_episodes'] == ['episode_000001'],
              info['batch0_contaminated_episodes'])
        # a dataset under the forbidden root is flagged too
        _, _, info2 = F.load_episodes(cand.root, [os.path.join(tmp, 'cand')])
        check('dataset_under_forbidden_root_flagged',
              bool(info2['dataset_under_forbidden_root']))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_json_out_strict():
    probe = {'results': [{'name': 'x', 'pass': True, 'detail': {'b', 'a'}}]}
    text = json.dumps({'results': _jsonable(probe['results'])},
                      sort_keys=True, allow_nan=False)
    check('json_out_strict_set', text == '{"results": [{"detail": ["a", "b"], '
          '"name": "x", "pass": true}]}', text)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('Phase 2F split OFFLINE tests (no simulation)')
    test_policy_validation()
    test_canonical_key_normalization()
    test_episode_random_determinism()
    test_group_aware_no_cross_split()
    test_eval_fixed_explicit_stability()
    test_leakage_audit_fields()
    test_manifest_preview_semantics()
    test_evaluation_views_labeled()
    test_gate_booleans()
    test_evidence_binding()
    test_cli_freeze_gate_without_binding()
    test_plan_guard_and_preview_cli()
    test_audit_readonly_and_invalid()
    test_batch0_contamination_flagged()
    test_json_out_strict()
    passed = sum(1 for r in _RESULTS if r['pass'])
    total = len(_RESULTS)
    print('\nphase 2F split offline tests: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as fh:
            json.dump({'offline': True, 'passed': passed, 'total': total,
                       'results': _jsonable(_RESULTS)}, fh, indent=2,
                      sort_keys=True, allow_nan=False)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
