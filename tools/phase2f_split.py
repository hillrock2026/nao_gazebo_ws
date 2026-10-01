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
Phase 2F split tool: read-only audit, group-aware preview and gated freeze.

This module deliberately separates the three stages:

  audit    read-only leakage/coverage audit of an existing Phase 2A dataset and
           its current ``splits/*.jsonl`` (never writes the dataset);
  plan     deterministic *preview* split (episode-random baseline or
           group-aware) written as a manifest marked ``status=preview`` /
           ``final=false`` -- it never writes ``splits/*.jsonl``;
  freeze   the only path that can emit a final Phase 2F manifest, and only with
           an explicit Gate 2A evidence file where every gate is satisfied.

Policy, ratios, group key and ``eval_fixed`` rules live in :data:`DEFAULT_POLICY`
and are documented in ``docs/Phase2F_Split_Policy.md``. Splitting is always at
``episode_id`` granularity: every sample of an episode belongs to exactly one
split. It reuses :func:`phase2a_episode_dataset.plan_splits` for the
episode-random baseline instead of duplicating split logic.

``quality_score`` (Phase 2E) is not used here; this tool only partitions data.
"""

import argparse
import hashlib
import json
import os
import platform
import random
import sys
import tempfile
import time

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import phase2a_episode_dataset as ep  # noqa: E402

SPLIT_POLICY_VERSION = '2f-policy-1.1.0'
SPLIT_MANIFEST_VERSION = '2f-manifest-1.1.0'
SPLITS = ('train', 'val', 'eval_fixed')
# Phase 2F evaluates two *non-comparable* generalization goals (see
# docs/Phase2F_Evaluation_Goals.md): the main split and the stress split.
PRIMARY_VIEW = 'seen_group_generalization'
STRESS_VIEW = 'unseen_group_stress'
EVALUATION_VIEWS = (PRIMARY_VIEW, STRESS_VIEW)
GROUP_KEY_CHOICES = ('param_set_key', 'config_group_key', 'seed',
                     'source_dataset')
DEFAULT_BATCH0_ROOT = os.path.join(
    os.path.dirname(_TOOLS), 'docs', 'task5_gate2a_batch0_candidate')

# Gate 2A post-conditions required before ``freeze`` is allowed (see
# docs/Phase2F_Split_Policy.md section 7). Every key must be present and True.
GATE2A_EVIDENCE_KEYS = (
    'episode_count_in_range',      # 100-200 valid episodes
    'seeds_ok',                    # >= 3 auditable seeds
    'tiers_ok',                    # >= 3 measured body-pose distinguishable tiers
    'failure_classes_ok',          # >= 3 actually observed failure classes
    'provenance_ok',               # provenance/schema/Phase2E/hash complete
    'baseline_report',             # docs/Baseline_Report.md generated
    'independent_review_passed',   # Gate 2A independent review passed
    'policy_frozen',               # split policy/group key/eval_fixed frozen
    'evaluation_goal_decided',     # episode-random / group-holdout decided
    'leakage_tests_passed',        # all leakage + regression tests passed
)

DEFAULT_POLICY = {
    'split_policy_version': SPLIT_POLICY_VERSION,
    # None -> inferred: group holdout => stress view, else primary view.
    'evaluation_view': None,
    'split_seed': 20260,
    'train_ratio': 0.7,
    'val_ratio': 0.2,
    'eval_fixed_policy': 'remainder',      # 'remainder' | 'explicit'
    'eval_fixed_episodes': [],
    'group_holdout_enabled': False,
    'group_holdout_key': 'param_set_key',
    'failure_balance_policy': 'report_only',   # 'report_only'|'require_min_classes'
    'min_failure_classes': 3,
    'seed_balance_policy': 'report_only',
    'tier_balance_policy': 'report_only',
    'near_duplicate_rel_tol': 0.05,
    'forbid_roots': [DEFAULT_BATCH0_ROOT],
    'exclude_episode_ids': [],
}


class Phase2FSplitError(Exception):
    """Known, explicit Phase 2F split failure (non-silent)."""


class Gate2AEvidenceError(Phase2FSplitError):
    """Freeze was attempted without a satisfied Gate 2A evidence file."""


# ---------------------------------------------------------------------------
# small helpers
# ---------------------------------------------------------------------------
def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def aggregate_root_hash(root):
    """Return (aggregate_sha256, file_count) over sorted ``relpath:sha256``."""
    lines = []
    for base, _, files in os.walk(root):
        for name in files:
            path = os.path.join(base, name)
            lines.append('%s:%s' % (os.path.relpath(path, root),
                                    sha256_file(path)))
    return hashlib.sha256('\n'.join(sorted(lines)).encode()).hexdigest(), len(lines)


def canonical_param_set_key(param_set):
    """Stable canonical key over the six frozen ``param_set`` numbers."""
    if not isinstance(param_set, dict):
        raise Phase2FSplitError('param_set is not an object: %r' % (param_set,))
    items = []
    for key in ep.CANONICAL_PARAM_KEYS:
        value = param_set.get(key)
        if not ep.is_number(value):
            raise Phase2FSplitError(
                'param_set.%s is missing or not a finite number: %r'
                % (key, value))
        rounded = round(float(value), 6)
        # Normalise all numeric zeros to +0.0 so 0, 0.0, -0.0 and -0 share one
        # canonical key (mathematically equal params must not split a group).
        if rounded == 0:
            rounded = 0.0
        items.append((key, rounded))
    return ';'.join('%s=%g' % (k, v) for k, v in items)


def is_finite_number(value):
    return ep.is_number(value) and ep.is_finite(value)


# ---------------------------------------------------------------------------
# policy
# ---------------------------------------------------------------------------
def merge_policy(user):
    policy = json.loads(json.dumps(DEFAULT_POLICY))
    if user:
        policy.update(user)
    validate_policy(policy)
    return policy


def validate_policy(policy):
    problems = []
    for key in ('train_ratio', 'val_ratio'):
        value = policy.get(key)
        if not is_finite_number(value) or not (0.0 <= float(value) <= 1.0):
            problems.append('%s must be in [0,1]' % key)
    if not problems and float(policy['train_ratio']) + float(policy['val_ratio']) \
            > 1.0 + 1e-9:
        problems.append('train_ratio + val_ratio must be <= 1')
    if not ep.is_int(policy.get('split_seed')):
        problems.append('split_seed must be an integer')
    if policy.get('eval_fixed_policy') not in ('remainder', 'explicit'):
        problems.append('eval_fixed_policy must be remainder|explicit')
    if not isinstance(policy.get('eval_fixed_episodes'), list):
        problems.append('eval_fixed_episodes must be a list')
    if not isinstance(policy.get('group_holdout_enabled'), bool):
        problems.append('group_holdout_enabled must be boolean')
    if policy.get('group_holdout_enabled') and \
            policy.get('group_holdout_key') not in GROUP_KEY_CHOICES:
        problems.append('group_holdout_key must be one of %s'
                        % (GROUP_KEY_CHOICES,))
    if policy.get('evaluation_view') not in (None,) + EVALUATION_VIEWS:
        problems.append('evaluation_view must be null or one of %s'
                        % (EVALUATION_VIEWS,))
    if not is_finite_number(policy.get('near_duplicate_rel_tol')) \
            or float(policy['near_duplicate_rel_tol']) < 0.0:
        problems.append('near_duplicate_rel_tol must be >= 0')
    if not isinstance(policy.get('forbid_roots'), list):
        problems.append('forbid_roots must be a list')
    if not isinstance(policy.get('exclude_episode_ids'), list):
        problems.append('exclude_episode_ids must be a list')
    if problems:
        raise Phase2FSplitError('invalid split policy: %s' % problems)


def load_policy(path):
    if not path:
        return merge_policy({})
    try:
        with open(path, 'r', encoding='utf-8') as fh:
            return merge_policy(json.load(fh))
    except OSError as exc:
        raise Phase2FSplitError('cannot read policy %s: %s' % (path, exc))
    except ValueError as exc:
        raise Phase2FSplitError('policy is not valid JSON %s: %s'
                                % (path, exc))


# ---------------------------------------------------------------------------
# dataset loading (read-only)
# ---------------------------------------------------------------------------
def _reference_sample_shas(roots):
    """samples_sha256 from forbidden roots' dataset dirs (batch-0 guard)."""
    shas = set()
    for root in roots:
        for candidate in (os.path.join(root, 'work', 'dataset'), root):
            index = os.path.join(candidate, 'index.jsonl')
            if not os.path.isfile(index):
                continue
            try:
                ds = ep.EpisodeDataset(candidate)
                for row in ds.load_index():
                    meta = ds.load_metadata(row['episode_id'])
                    sha = meta.get('samples_sha256')
                    if isinstance(sha, str) and len(sha) == 64:
                        shas.add(sha)
            except (OSError, ValueError, ep.EpisodeDatasetError):
                continue
    return shas


def load_episodes(dataset_root, forbid_roots=None):
    """Validate and read one dataset; returns (ds, episodes, info)."""
    dataset_root = os.path.abspath(dataset_root)
    ds = ep.EpisodeDataset(dataset_root)
    check = ds.validate()
    if not check['valid']:
        raise Phase2FSplitError('dataset invalid: %s' % check['errors'][:3])
    forbid = [os.path.abspath(r) for r in (forbid_roots or [])]
    under_forbidden = [r for r in forbid
                       if dataset_root == r
                       or dataset_root.startswith(r + os.sep)]
    ref_shas = _reference_sample_shas(forbid)
    index = ds.load_index()
    episodes = []
    contaminated = []
    for row in index:
        eid = row['episode_id']
        meta = ds.load_metadata(eid)
        sp = ds.samples_path(eid)
        if not os.path.isfile(sp):
            raise Phase2FSplitError('missing samples file: %s' % sp)
        sha = meta.get('samples_sha256')
        if not (isinstance(sha, str) and len(sha) == 64):
            sha = sha256_file(sp)
        if sha in ref_shas:
            contaminated.append(eid)
        param_key = canonical_param_set_key(meta.get('param_set'))
        # config_group_key defaults to the exact canonical param set; a policy
        # may group by a coarser key via group_holdout_key.
        config_key = param_key
        episodes.append({
            'episode_id': eid,
            'split': row['split'],
            'source_dataset': dataset_root,
            'sample_path': sp,
            'sample_sha256': sha,
            'sample_count': row.get('sample_count'),
            'param_set_key': param_key,
            'config_group_key': config_key,
            'seed': meta.get('seed'),
            'success': meta.get('success'),
            'usable': meta.get('usable'),
            'fell': meta.get('fell'),
            'failure_reason': meta.get('failure_reason'),
            'baseline_config': meta.get('baseline_config'),
        })
    agg, count = aggregate_root_hash(dataset_root)
    info = {
        'dataset_root': dataset_root,
        'aggregate_sha256': agg,
        'file_count': count,
        'index_sha256': sha256_file(os.path.join(dataset_root,
                                                 ep.INDEX_FILE)),
        'manifest_sha256': sha256_file(os.path.join(dataset_root,
                                                    ep.MANIFEST_FILE)),
        'episode_count': len(episodes),
        'forbid_roots': forbid,
        'dataset_under_forbidden_root': under_forbidden,
        'batch0_contaminated_episodes': sorted(contaminated),
    }
    return ds, episodes, info


# ---------------------------------------------------------------------------
# plan
# ---------------------------------------------------------------------------
def _targets(n, train_ratio, val_ratio):
    n_train = int(n * float(train_ratio))
    n_val = int(n * float(val_ratio))
    return n_train, n_val, n - n_train - n_val


def _apply_eval_fixed(plan, policy):
    if policy['eval_fixed_policy'] == 'explicit':
        for eid in policy['eval_fixed_episodes']:
            if eid not in plan:
                raise Phase2FSplitError(
                    'eval_fixed episode not present in dataset: %s' % eid)
            plan[eid] = 'eval_fixed'
    return plan


def plan_episode_random(episodes, policy):
    ids = [e['episode_id'] for e in episodes]
    plan = ep.plan_splits(ids, seed=policy['split_seed'],
                          train_ratio=float(policy['train_ratio']),
                          val_ratio=float(policy['val_ratio']))
    return _apply_eval_fixed(plan, policy)


def _group_value(episode, key):
    if key == 'param_set_key':
        return episode['param_set_key']
    if key == 'config_group_key':
        return episode['config_group_key']
    if key == 'seed':
        return str(episode['seed'])
    if key == 'source_dataset':
        return episode['source_dataset']
    raise Phase2FSplitError('unknown group key: %r' % (key,))


def plan_group_aware(episodes, policy):
    """Assign whole groups to splits; no group is ever split across splits.

    Groups are walked in a deterministic shuffled order and each whole group is
    assigned to the split that is currently furthest *behind* its target
    fraction (normalized deficit). This keeps splits non-empty and approximates
    the requested ratios even when groups are few/large -- exact ratios are
    impossible with whole groups, which is reported as a warning rather than
    silently collapsing everything into ``train``.
    """
    key = policy['group_holdout_key']
    groups = {}
    for episode in episodes:
        groups.setdefault(_group_value(episode, key), []).append(
            episode['episode_id'])
    n = len(episodes)
    n_train, n_val, n_eval = _targets(n, policy['train_ratio'],
                                      policy['val_ratio'])
    targets = {'train': n_train, 'val': n_val, 'eval_fixed': n_eval}
    counts = {s: 0 for s in SPLITS}
    keys = sorted(groups, key=lambda value: str(value))
    random.Random(policy['split_seed']).shuffle(keys)
    plan = {}
    for group in keys:
        members = groups[group]

        def deficit(split):
            target = targets[split]
            if target <= 0:
                return -1.0 - counts[split]
            return (target - counts[split]) / float(target)

        chosen = max(SPLITS, key=lambda s: (deficit(s), -SPLITS.index(s)))
        for eid in members:
            plan[eid] = chosen
        counts[chosen] += len(members)
    return _apply_eval_fixed(plan, policy)


def plan_split(episodes, policy):
    excluded = set(policy.get('exclude_episode_ids') or [])
    kept = [e for e in episodes if e['episode_id'] not in excluded]
    if not kept:
        raise Phase2FSplitError('no episodes left after exclusion')
    if policy['group_holdout_enabled']:
        return plan_group_aware(kept, policy)
    return plan_episode_random(kept, policy)


# ---------------------------------------------------------------------------
# leakage audit
# ---------------------------------------------------------------------------
def _by_split(episodes, plan):
    result = {s: [] for s in SPLITS}
    for episode in episodes:
        split = plan[episode['episode_id']]
        if split not in SPLITS:
            raise Phase2FSplitError('illegal split for %s: %r'
                                    % (episode['episode_id'], split))
        result[split].append(episode)
    return result


def _cross_split(by_split, field):
    mapping = {}
    for split in SPLITS:
        for episode in by_split[split]:
            mapping.setdefault(episode[field], set()).add(split)
    return {str(k): sorted(v) for k, v in mapping.items() if len(v) > 1}


def _pairwise_overlap(by_split, attr):
    out = {}
    for i in range(len(SPLITS)):
        for j in range(i + 1, len(SPLITS)):
            a = {e[attr] for e in by_split[SPLITS[i]]}
            b = {e[attr] for e in by_split[SPLITS[j]]}
            if a & b:
                out['%s__%s' % (SPLITS[i], SPLITS[j])] = sorted(a & b)
    return out


def _parse_param_key(name):
    """Parse a canonical 'k=v;k=v' key; return None when not well-formed."""
    parsed = {}
    try:
        for pair in name.split(';'):
            key, _, value = pair.partition('=')
            if not key or not value:
                return None
            parsed[key] = float(value)
    except (ValueError, AttributeError):
        return None
    if any(key not in parsed for key in ep.CANONICAL_PARAM_KEYS):
        return None
    return parsed


def near_duplicate_groups(episodes, rel_tol):
    """Group param_set_keys whose canonical numbers are within ``rel_tol``.

    Non-canonical keys (e.g. synthetic labels) are ignored: near-duplicate
    detection only applies to well-formed canonical param sets.
    """
    numeric = {}
    for episode in episodes:
        name = episode['param_set_key']
        if name in numeric:
            continue
        parsed = _parse_param_key(name)
        if parsed is not None:
            numeric[name] = parsed
    names = sorted(numeric)
    groups = []
    seen = set()
    for i, a in enumerate(names):
        if a in seen:
            continue
        group = [a]
        for b in names[i + 1:]:
            if b in seen:
                continue
            close = True
            for key in ep.CANONICAL_PARAM_KEYS:
                va, vb = numeric[a][key], numeric[b][key]
                scale = max(abs(va), abs(vb), 1e-9)
                if abs(va - vb) / scale > rel_tol:
                    close = False
                    break
            if close:
                group.append(b)
                seen.add(b)
        if len(group) > 1:
            groups.append(group)
    return groups


def leakage_audit(episodes, plan, policy):
    by_split = _by_split(episodes, plan)
    counts = {s: len(by_split[s]) for s in SPLITS}
    sample_counts = {s: sum(int(e['sample_count'] or 0) for e in by_split[s])
                     for s in SPLITS}
    episode_to_split = {}
    for split in SPLITS:
        for episode in by_split[split]:
            episode_to_split.setdefault(episode['episode_id'], set()).add(split)
    adjacent = sorted(eid for eid, ss in episode_to_split.items()
                      if len(ss) > 1)
    failure_by_split = {
        s: _counts([e['failure_reason'] if e['failure_reason'] is not None
                    else 'none' for e in by_split[s]])
        for s in SPLITS}
    seed_by_split = {s: sorted({str(e['seed']) for e in by_split[s]})
                     for s in SPLITS}
    params_by_split = {s: sorted({e['param_set_key'] for e in by_split[s]})
                       for s in SPLITS}
    observed_failures = sorted({e['failure_reason'] for e in episodes
                                if e['failure_reason'] is not None})
    near_dups = near_duplicate_groups(episodes,
                                      float(policy['near_duplicate_rel_tol']))
    audit = {
        'counts_by_split': counts,
        'sample_counts_by_split': sample_counts,
        'episode_overlap': _pairwise_overlap(by_split, 'episode_id'),
        'sample_path_overlap': _pairwise_overlap(by_split, 'sample_path'),
        'sample_sha256_overlap': _pairwise_overlap(by_split, 'sample_sha256'),
        'adjacent_frame_leakage': adjacent,   # [] == structurally impossible
        'param_set_cross_split': _cross_split(by_split, 'param_set_key'),
        'config_group_cross_split': _cross_split(by_split, 'config_group_key'),
        'seed_cross_split': _cross_split(by_split, 'seed'),
        'failure_reason_by_split': failure_by_split,
        'seed_by_split': seed_by_split,
        'param_set_by_split': params_by_split,
        'observed_failure_classes': observed_failures,
        'near_duplicate_groups': near_dups,
        'n_distinct_seeds': len({str(e['seed']) for e in episodes}),
        'n_distinct_param_sets': len({e['param_set_key'] for e in episodes}),
    }
    return audit


def _counts(values):
    out = {}
    for value in values:
        out[str(value)] = out.get(str(value), 0) + 1
    return out


# ---------------------------------------------------------------------------
# manifest
# ---------------------------------------------------------------------------
def build_manifest(dataset_root, episodes, plan, policy, status, info):
    if status not in ('preview', 'final'):
        raise Phase2FSplitError('manifest status must be preview|final')
    splits = {s: sorted(e['episode_id'] for e in episodes
                        if plan[e['episode_id']] == s) for s in SPLITS}
    n = len(episodes)
    counts = {s: len(splits[s]) for s in SPLITS}
    ratios_actual = {s: (round(counts[s] / n, 6) if n else 0.0)
                     for s in SPLITS}
    audit = leakage_audit(episodes, plan, policy)
    targets = dict(zip(SPLITS, _targets(n, policy['train_ratio'],
                                        policy['val_ratio'])))
    warnings = []
    if counts['train'] == 0 or counts['val'] == 0 or counts['eval_fixed'] == 0:
        warnings.append('one_or_more_splits_empty')
    for split in SPLITS:
        target = targets[split]
        if abs(counts[split] - target) > max(1, 0.15 * target):
            warnings.append('ratios_not_met:%s(%d!=%d)'
                            % (split, counts[split], target))
            break
    if audit['param_set_cross_split']:
        warnings.append('param_set_cross_split_present')
    if audit['seed_cross_split']:
        warnings.append('seed_cross_split_present')
    if audit['adjacent_frame_leakage']:
        warnings.append('adjacent_frame_leakage_present')
    if info['dataset_under_forbidden_root']:
        warnings.append('dataset_under_forbidden_root')
    if info['batch0_contaminated_episodes']:
        warnings.append('batch0_contamination')
    view = policy.get('evaluation_view') or (
        STRESS_VIEW if policy['group_holdout_enabled'] else PRIMARY_VIEW)
    return {
        'split_manifest_version': SPLIT_MANIFEST_VERSION,
        'split_policy_version': SPLIT_POLICY_VERSION,
        'evaluation_view': view,
        'status': status,
        'final': status == 'final',
        'dataset_root': info['dataset_root'],
        'source_aggregate_sha256': info['aggregate_sha256'],
        'source_index_sha256': info['index_sha256'],
        'source_manifest_sha256': info['manifest_sha256'],
        'source_file_count': info['file_count'],
        'split_seed': policy['split_seed'],
        'train_ratio': policy['train_ratio'],
        'val_ratio': policy['val_ratio'],
        'eval_ratio': round(1.0 - float(policy['train_ratio'])
                            - float(policy['val_ratio']), 6),
        'eval_fixed_policy': policy['eval_fixed_policy'],
        'eval_fixed_episodes': list(policy['eval_fixed_episodes']),
        'group_holdout_enabled': policy['group_holdout_enabled'],
        'group_holdout_key': policy['group_holdout_key'],
        'episode_to_split': {e['episode_id']: plan[e['episode_id']]
                             for e in episodes},
        'splits': splits,
        'counts': counts,
        'ratios_actual': ratios_actual,
        'leakage': audit,
        'warnings': warnings,
        'generation_command': ' '.join(sys.argv),
        'environment': {
            'python': platform.python_version(),
            'platform': platform.platform(),
        },
        'created_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
    }


# ---------------------------------------------------------------------------
# atomic write
# ---------------------------------------------------------------------------
def write_json_atomic(path, obj, allow_overwrite=False):
    if os.path.exists(path) and not allow_overwrite:
        raise Phase2FSplitError('refusing to overwrite existing file: %s'
                                % path)
    directory = os.path.dirname(path) or '.'
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.tmp_p2f_', dir=directory)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as fh:
            json.dump(obj, fh, indent=2, sort_keys=True, allow_nan=False)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def _guard_manifest_path(manifest_out, dataset_root):
    """A preview/final manifest must never be written into splits/*.jsonl."""
    splits_dir = os.path.join(os.path.abspath(dataset_root), ep.SPLITS_DIR)
    target = os.path.abspath(manifest_out)
    if target == splits_dir or target.startswith(splits_dir + os.sep):
        raise Phase2FSplitError(
            'refusing to write a Ph2F manifest into the dataset splits dir: %s'
            % target)


# ---------------------------------------------------------------------------
# audit report
# ---------------------------------------------------------------------------
def build_audit_report(episodes, plan, policy, info, status):
    audit = leakage_audit(episodes, plan, policy)
    warnings = []
    if audit['param_set_cross_split']:
        warnings.append('param_set_cross_split_present')
    if audit['seed_cross_split']:
        warnings.append('seed_cross_split_present')
    if audit['adjacent_frame_leakage']:
        warnings.append('adjacent_frame_leakage_present')
    if info['batch0_contaminated_episodes']:
        warnings.append('batch0_contamination')
    if info['dataset_under_forbidden_root']:
        warnings.append('dataset_under_forbidden_root')
    if policy['failure_balance_policy'] == 'require_min_classes' and \
            len(audit['observed_failure_classes']) < int(policy['min_failure_classes']):
        warnings.append('failure_classes_below_min:%d<%d'
                        % (len(audit['observed_failure_classes']),
                           int(policy['min_failure_classes'])))
    view = policy.get('evaluation_view') or (
        STRESS_VIEW if policy['group_holdout_enabled'] else PRIMARY_VIEW)
    return {
        'report_version': SPLIT_MANIFEST_VERSION,
        'split_policy_version': SPLIT_POLICY_VERSION,
        'evaluation_view': view,
        'status': status,
        'final': False,
        'dataset_root': info['dataset_root'],
        'aggregate_sha256': info['aggregate_sha256'],
        'episode_count': info['episode_count'],
        'observed': audit,
        'planned': {'split_seed': policy['split_seed'],
                    'train_ratio': policy['train_ratio'],
                    'val_ratio': policy['val_ratio'],
                    'eval_fixed_policy': policy['eval_fixed_policy']},
        'balance': {
            'failure_balance_policy': policy['failure_balance_policy'],
            'min_failure_classes': policy['min_failure_classes'],
            'seed_balance_policy': policy['seed_balance_policy'],
            'tier_balance_policy': policy['tier_balance_policy'],
        },
        'warnings': warnings,
        'errors': [],
    }


# ---------------------------------------------------------------------------
# CLI commands
# ---------------------------------------------------------------------------
def _print(obj):
    print(json.dumps(obj, indent=2, sort_keys=True, allow_nan=False))


def cmd_audit(args):
    policy = load_policy(args.policy)
    if args.forbid_root:
        policy['forbid_roots'] = list(policy['forbid_roots']) + args.forbid_root
        validate_policy(policy)
    _, episodes, info = load_episodes(args.dataset, policy['forbid_roots'])
    plan = {e['episode_id']: e['split'] for e in episodes}
    report = build_audit_report(episodes, plan, policy, info, 'audit_existing')
    if args.json_out:
        write_json_atomic(args.json_out, report, allow_overwrite=True)
    _print({'dataset': info['dataset_root'],
            'episode_count': info['episode_count'],
            'counts_by_split': report['observed']['counts_by_split'],
            'param_set_cross_split': len(report['observed']['param_set_cross_split']),
            'seed_cross_split': len(report['observed']['seed_cross_split']),
            'adjacent_frame_leakage': report['observed']['adjacent_frame_leakage'],
            'batch0_contaminated_episodes': info['batch0_contaminated_episodes'],
            'warnings': report['warnings'],
            'json_out': args.json_out})
    return 0


def cmd_plan(args):
    if args.mode != 'preview':
        raise Phase2FSplitError(
            'only --mode preview is permitted while Gate 2A is not passed')
    policy = load_policy(args.policy)
    _guard_manifest_path(args.manifest_out, args.dataset)
    _, episodes, info = load_episodes(args.dataset, policy['forbid_roots'])
    plan = plan_split(episodes, policy)
    manifest = build_manifest(args.dataset, episodes, plan, policy, 'preview',
                              info)
    if args.json_out:
        report = build_audit_report(episodes, plan, policy, info,
                                    'preview')
        write_json_atomic(args.json_out, report, allow_overwrite=True)
    write_json_atomic(args.manifest_out, manifest,
                      allow_overwrite=args.allow_overwrite)
    _print({'status': manifest['status'], 'final': manifest['final'],
            'counts': manifest['counts'],
            'group_holdout_enabled': manifest['group_holdout_enabled'],
            'warnings': manifest['warnings'],
            'manifest_out': args.manifest_out})
    return 0


# Gate 2A evidence must additionally bind to the *actual* dataset and to the
# frozen artifacts, so a hand-written all-true JSON cannot pass review.
EVIDENCE_BINDING_KEYS = (
    'dataset_root',
    'dataset_aggregate_sha256',
    'dataset_index_sha256',
    'dataset_manifest_sha256',
    'file_count',
    'split_policy_sha256',
    'evidence_files',
)
# Roles that must be present under ``evidence_files`` with a matching sha256.
EVIDENCE_FILE_ROLES = ('baseline_report', 'independent_review')


def check_gate2a_booleans(evidence):
    if not isinstance(evidence, dict):
        raise Gate2AEvidenceError('Gate 2A evidence must be a JSON object')
    missing = [k for k in GATE2A_EVIDENCE_KEYS if k not in evidence]
    failed = [k for k in GATE2A_EVIDENCE_KEYS if evidence.get(k) is not True]
    if missing or failed:
        raise Gate2AEvidenceError(
            'Gate 2A post-conditions not satisfied: missing=%s failed=%s'
            % (missing, failed))


def check_gate2a_binding(evidence, info, policy_sha256):
    """Bind the evidence to the real dataset, policy and evidence files."""
    missing = [k for k in EVIDENCE_BINDING_KEYS if k not in evidence]
    if missing:
        raise Gate2AEvidenceError(
            'Gate 2A evidence missing hash-binding fields: %s' % missing)
    expected = {
        'dataset_root': info['dataset_root'],
        'dataset_aggregate_sha256': info['aggregate_sha256'],
        'dataset_index_sha256': info['index_sha256'],
        'dataset_manifest_sha256': info['manifest_sha256'],
        'file_count': info['file_count'],
    }
    for key, want in expected.items():
        got = evidence.get(key)
        if got != want:
            raise Gate2AEvidenceError(
                'Gate 2A evidence %s mismatch: evidence=%r actual=%r'
                % (key, got, want))
    if policy_sha256 is not None and \
            evidence.get('split_policy_sha256') != policy_sha256:
        raise Gate2AEvidenceError(
            'Gate 2A evidence split_policy_sha256 mismatch: evidence=%r '
            'actual=%r' % (evidence.get('split_policy_sha256'),
                           policy_sha256))
    files = evidence.get('evidence_files')
    if not isinstance(files, dict):
        raise Gate2AEvidenceError('evidence_files must be an object')
    for role in EVIDENCE_FILE_ROLES:
        entry = files.get(role)
        if not isinstance(entry, dict) or not entry.get('path') \
                or not entry.get('sha256'):
            raise Gate2AEvidenceError(
                'evidence_files.%s must record path and sha256' % role)
        path = entry['path']
        if not os.path.isfile(path):
            raise Gate2AEvidenceError(
                'evidence_files.%s path missing: %s' % (role, path))
        actual = sha256_file(path)
        if actual != entry['sha256']:
            raise Gate2AEvidenceError(
                'evidence_files.%s sha256 mismatch: recorded=%r actual=%r'
                % (role, entry['sha256'], actual))


def check_gate2a_evidence(evidence, info=None, policy_sha256=None):
    check_gate2a_booleans(evidence)
    if info is None:
        raise Gate2AEvidenceError(
            'Gate 2A evidence binding requires dataset info')
    check_gate2a_binding(evidence, info, policy_sha256)


def cmd_freeze(args):
    policy = load_policy(args.policy)
    policy_sha256 = sha256_file(args.policy)
    try:
        with open(args.gate2a_evidence, 'r', encoding='utf-8') as fh:
            evidence = json.load(fh)
    except OSError as exc:
        raise Gate2AEvidenceError(
            'cannot read Gate 2A evidence %s: %s'
            % (args.gate2a_evidence, exc))
    check_gate2a_booleans(evidence)   # cheap fail-fast before touching data
    if policy['eval_fixed_policy'] != 'explicit' or \
            not policy['eval_fixed_episodes']:
        raise Gate2AEvidenceError(
            'freeze requires eval_fixed_policy=explicit with a frozen episode '
            'list')
    _guard_manifest_path(args.manifest_out, args.dataset)
    _, episodes, info = load_episodes(args.dataset, policy['forbid_roots'])
    check_gate2a_binding(evidence, info, policy_sha256)  # raises before write
    plan = plan_split(episodes, policy)
    manifest = build_manifest(args.dataset, episodes, plan, policy, 'final',
                              info)
    write_json_atomic(args.manifest_out, manifest,
                      allow_overwrite=args.allow_overwrite)
    _print({'status': manifest['status'], 'final': manifest['final'],
            'counts': manifest['counts'], 'manifest_out': args.manifest_out})
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description='Phase 2F split tool')
    sub = parser.add_subparsers(dest='command', required=True)

    p_audit = sub.add_parser('audit', help='read-only leakage/coverage audit')
    p_audit.add_argument('--dataset', required=True)
    p_audit.add_argument('--policy', default=None)
    p_audit.add_argument('--forbid-root', action='append', default=None)
    p_audit.add_argument('--json-out', default=None)
    p_audit.set_defaults(func=cmd_audit)

    p_plan = sub.add_parser('plan', help='deterministic preview split')
    p_plan.add_argument('--dataset', required=True)
    p_plan.add_argument('--policy', required=True)
    p_plan.add_argument('--mode', default='preview', choices=['preview'])
    p_plan.add_argument('--manifest-out', required=True)
    p_plan.add_argument('--json-out', default=None)
    p_plan.add_argument('--allow-overwrite', action='store_true')
    p_plan.set_defaults(func=cmd_plan)

    p_freeze = sub.add_parser('freeze', help='gated final split manifest')
    p_freeze.add_argument('--dataset', required=True)
    p_freeze.add_argument('--policy', required=True)
    p_freeze.add_argument('--gate2a-evidence', required=True)
    p_freeze.add_argument('--manifest-out', required=True)
    p_freeze.add_argument('--allow-overwrite', action='store_true')
    p_freeze.set_defaults(func=cmd_freeze)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ep.EpisodeDatasetError, Phase2FSplitError) as exc:
        print(json.dumps({'ok': False, 'reason': type(exc).__name__,
                          'detail': str(exc)}), file=sys.stderr)
        sys.exit(2)
