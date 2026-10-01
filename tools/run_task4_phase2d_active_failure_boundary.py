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
Phase 2D runner — active failure & boundary samples.

Reuses the validated Phase 2B launch/collect/validate pipeline and the Phase 2C
absolute-parameter handling, then applies the Phase 2D honest failure
classifier. Failed episodes that still have a valid frozen run are KEPT in the
dataset (``success=false``, ``usable=true``); startup failures with no samples
are preserved on disk (run/log/metadata) and recorded in the report, never
deleted and never faked into success.

Modes: plan (read-only), dry-run (read-only), smoke (small directed batch).
"""

import argparse
import json
import os
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import phase2d_active_failure_boundary as D  # noqa: E402
import phase2b_baseline_a as B  # noqa: E402
import phase2a_episode_dataset as ep  # noqa: E402
import run_task4_phase2b_baseline_a as R  # noqa: E402

WS = os.path.dirname(_TOOLS)
DEFAULT_ROOT = os.path.join(
    WS, 'docs', 'task4_phase2d_active_failure_boundary_evidence', 'boundary')


def _load_json(path, default=None):
    if not os.path.isfile(path):
        return default
    with open(path, 'r', encoding='utf-8') as fh:
        return json.load(fh)


def run_candidate(row, work_root, timeout, log_path, domain_id,
                  allow_overwrite):
    shared = {
        'episode_id': row['episode_id'],
        'config_id': row['candidate_id'],
        'seed': row['seed'],
        'step_length_multiplier': row.get('multiplier'),
        'overrides': dict(row['overrides']),
    }
    result = R.run_one(shared, work_root, timeout, log_path, domain_id,
                       allow_overwrite=allow_overwrite)
    base = os.path.join(work_root, 'runs', row['episode_id'])
    run_dir = os.path.join(base, 'run')
    trace_dir = os.path.join(base, 'trace')
    summary = _load_json(os.path.join(base, 'summary.json'))
    provenance = {
        'candidate_id': row['candidate_id'],
        'kind': row.get('kind'),
        'param': row.get('param'),
        'multiplier': row.get('multiplier'),
        'boundary_label': row.get('boundary_label'),
        'expected_risk': row.get('expected_risk'),
        'requested_param_set': row.get('requested_param_set', {}),
        'external_push': row.get('external_push'),
        'seed': row['seed'],
    }

    if result.get('ok'):
        states = _load_jsonl(os.path.join(trace_dir, 'state_trace.jsonl'))
        ready = states[0]['t'] if states else None
        full = B.derive_episode_labels(run_dir, trace_dir, result['run_cfg'],
                                       ready, summary)
        boundaries = {}
        if row.get('param'):
            boundaries[row['param']] = {
                'boundary_label': row.get('boundary_label'),
                'expected_risk': row.get('expected_risk')}
        cls = D.classify_failure(
            True, full.get('sample_count'), result.get('timed_out'), full,
            full.get('metrics'), boundaries)
        # verify requested == applied for the perturbed parameter
        applied = {}
        problems = []
        if row.get('param'):
            requested = row['requested_param_set'][row['param']]
            applied_value = result['run_cfg'].get(row['param'])
            applied = {row['param']: applied_value}
            if applied_value is None or abs(float(applied_value)
                                            - float(requested)) > 1e-9:
                problems.append({'reason': 'requested_applied_mismatch',
                                 'requested': requested,
                                 'applied': applied_value})
        provenance['applied_param_set'] = applied
        provenance['application_problems'] = problems
        provenance['classification'] = cls
        provenance['metrics_subset'] = {k: (full.get('metrics') or {}).get(k)
                                        for k in ('body_dx', 'body_dy',
                                                  'body_min_z', 'max_roll_deg',
                                                  'max_pitch_deg', 'window')}
        result['phase2d'] = provenance
        result['labels'] = _reconciled_labels(
            {'labels': {k: v for k, v in full.items() if k != 'metrics'},
             'phase2d': provenance})
    else:
        result['phase2d'] = provenance
        result['phase2d']['classification'] = {
            'success': False,
            'failure_reason': ('timeout_or_no_progress'
                               if result.get('reason') == 'launch_timeout'
                               else 'startup_instability'),
            'secondary_reasons': [], 'usable': False,
            'diagnostics': {'runner_reason': result.get('reason')}}
    R._write_json(os.path.join(base, 'result.json'), result)
    return result


def _load_jsonl(path):
    rows = []
    if not os.path.isfile(path):
        return rows
    with open(path, 'r', encoding='utf-8') as fh:
        for line in fh:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def _reconciled_labels(result):
    """
    Make the Phase 2D discrete labels consistent with the Phase 2D
    classification (single source of truth), so per-episode and aggregate
    ``usable`` can never disagree (Phase 2D P1).
    """
    labels = dict(result.get('labels') or {})
    cls = (result.get('phase2d') or {}).get('classification')
    if cls:
        labels['success'] = cls.get('success', labels.get('success'))
        labels['usable'] = cls.get('usable', labels.get('usable'))
        labels['failure_reason'] = cls.get('failure_reason')
    return labels


def failure_counts_from_per_episode(per_episode):
    counts = {}
    for e in per_episode:
        reason = e.get('actual_failure_reason')
        key = reason if reason is not None else 'none'
        counts[key] = counts.get(key, 0) + 1
    return counts


def check_aggregate_consistency(per_episode, quality):
    """Return a list of consistency problems (empty = consistent)."""
    problems = []
    usable = [e for e in per_episode if e.get('usable') is True]
    overall = quality.get('overall', {})
    if overall.get('usable_count') != len(usable):
        problems.append({'reason': 'overall_usable_mismatch',
                         'aggregate': overall.get('usable_count'),
                         'per_episode': len(usable)})
    for cfg, block in quality.get('by_config', {}).items():
        subset = [e for e in per_episode if e.get('candidate_id') == cfg
                  and e.get('usable') is True]
        if block.get('usable_count') != len(subset):
            problems.append({'reason': 'by_config_usable_mismatch',
                             'config': cfg, 'aggregate': block.get('usable_count'),
                             'per_episode': len(subset)})
    total = sum(failure_counts_from_per_episode(per_episode).values())
    if total != len(per_episode):
        problems.append({'reason': 'failure_counts_total_mismatch',
                         'counts_total': total,
                         'per_episode': len(per_episode)})
    return problems


def build_dataset(work_root, results, train_ratio, val_ratio, split_seed):
    inputs = []
    excluded = []
    for result in results:
        if not result.get('ok'):
            excluded.append({'episode_id': result['episode_id'],
                             'reason': result['phase2d']['classification']
                             ['failure_reason'],
                             'kept_on_disk': True,
                             'path': 'runs/%s' % result['episode_id']})
            continue
        labels = result['labels']
        cls = result['phase2d']['classification']
        if any(labels.get(k) is None for k in (
                'duration_sec', 'mean_speed_x_mps', 'max_torso_tilt_deg',
                'energy_qdot', 'fell')):
            excluded.append({'episode_id': result['episode_id'],
                             'reason': 'missing_required_metric',
                             'missing': labels.get('missing'),
                             'kept_on_disk': True,
                             'path': 'runs/%s' % result['episode_id']})
            continue
        param_set = ep.param_set_from_producer(result['run_cfg'])
        p = result['phase2d']
        failure_detail = {
            'secondary_reasons': cls['secondary_reasons'],
            'diagnostics': cls['diagnostics'],
        }
        try:
            meta = ep.build_metadata(
                episode_id=result['episode_id'],
                controller_name=B.CONTROLLER_NAME,
                seed=result['seed'],
                param_set=param_set,
                duration_sec=labels['duration_sec'],
                fell=labels['fell'],
                fall_time=labels['fall_time'],
                mean_speed_x_mps=labels['mean_speed_x_mps'],
                max_torso_tilt_deg=labels['max_torso_tilt_deg'],
                energy_qdot=labels['energy_qdot'],
                success=cls['success'],
                usable=cls['usable'],
                failure_reason=cls['failure_reason'],
                controller_version=B.CONTROLLER_VERSION,
                baseline_config=p['candidate_id'],
                baseline_round='phase2d_boundary',
                boundary_label=p['boundary_label'],
                expected_risk=p['expected_risk'],
                requested_param_set=p['requested_param_set'],
                applied_param_set=p['applied_param_set'],
                failure_detail=failure_detail,
                source_run_id=result['episode_id'],
                target_duration_sec=labels['target_duration_sec'],
                body_min_z_m=labels['body_min_z_m'],
                energy_qdot_definition=labels.get('energy_qdot_definition'),
            )
        except ep.MetadataValidationError as exc:
            excluded.append({'episode_id': result['episode_id'],
                             'reason': 'metadata_invalid',
                             'detail': exc.errors, 'kept_on_disk': True})
            continue
        run_dir = os.path.join(work_root, 'runs', result['episode_id'], 'run')
        inputs.append(ep.EpisodeInput(result['episode_id'], run_dir, meta))
    if not inputs:
        return None, excluded, {}
    ds = ep.EpisodeDataset(os.path.join(work_root, 'dataset'))
    report = ds.build(inputs, dataset_name='nao_gait_baseline_phase2d',
                      sample_schema={'name': 'nao_gazebo_trajectory',
                                     'version': '1.0.1'},
                      split_seed=split_seed, train_ratio=train_ratio,
                      val_ratio=val_ratio, overwrite=True)
    return ds, excluded, report


def write_report(work_root, rows, results, excluded, dataset_report, ds):
    episodes = []
    per_episode = []
    for result in results:
        p = result['phase2d']
        cls = p['classification']
        reason = cls['failure_reason']
        per_episode.append({
            'episode_id': result['episode_id'],
            'candidate_id': p['candidate_id'],
            'seed': p['seed'],
            'requested_param_set': p['requested_param_set'],
            'applied_param_set': p.get('applied_param_set'),
            'expected_risk': p['expected_risk'],
            'actual_failure_reason': reason,
            'usable': cls['usable'],
            'success': cls['success'],
            'external_push': p['external_push'],
            'ok': result.get('ok'),
            'path': 'runs/%s' % result['episode_id'],
        })
        if result.get('ok'):
            episodes.append({
                'episode_id': result['episode_id'],
                'config_id': p['candidate_id'],
                'seed': result['seed'],
                'step_length_m': result['run_cfg']['step_length_m'],
                'frequency_hz': result['run_cfg']['step_frequency_hz'],
                'labels': _reconciled_labels(result),
            })
    split_counts = {}
    index_rows = []
    if ds is not None and os.path.isdir(ds.root):
        index_rows = ds.load_index()
        for r in index_rows:
            split_counts[r['split']] = split_counts.get(r['split'], 0) + 1
    quality = B.quality_report(episodes, split_counts, index_rows)
    consistency = check_aggregate_consistency(per_episode, quality)
    if consistency:
        raise D.Phase2DError(
            'aggregate inconsistency (P1): %s' % consistency)
    report = {
        'round': 'phase2d_boundary',
        'plan_rows': rows,
        'execution': {
            'runs_total': len(results),
            'runs_ok': sum(1 for r in results if r.get('ok')),
            'excluded_from_dataset': excluded,
            'dataset_build': dataset_report,
        },
        'per_episode': per_episode,
        'failure_reason_counts': failure_counts_from_per_episode(per_episode),
        'coverage': D.failure_coverage_matrix(rows),
        'external_push': D.external_push_plan(),
        'quality': quality,
        'speed_basis': 'measured body_pose; cmd_vel unused',
    }
    R._write_json(os.path.join(work_root, 'quality_report.json'), report)
    return report


def mode_regenerate(args):
    """
    Rebuild quality_report.json from existing run artifacts only.

    Reads runs/*/result.json + validates the existing dataset; never writes
    samples/metadata/index. Atomic report write; non-zero exit on failure.
    """
    root_abs = os.path.abspath(args.root)
    work_root = os.path.join(root_abs, 'work')
    rows = D.boundary_plan()['rows']
    results = []
    for row in rows:
        path = os.path.join(work_root, 'runs', row['episode_id'], 'result.json')
        if not os.path.isfile(path):
            raise D.Phase2DError('missing run artifact: %s' % path)
        with open(path, 'r', encoding='utf-8') as fh:
            results.append(json.load(fh))
    ds = ep.EpisodeDataset(os.path.join(work_root, 'dataset'))
    if not os.path.isdir(ds.root):
        raise D.Phase2DError('dataset missing: %s' % ds.root)
    dataset_report = ds.validate()
    indexed = {r['episode_id'] for r in ds.load_index()}
    excluded = []
    for result in results:
        if not result.get('ok'):
            excluded.append({'episode_id': result['episode_id'],
                             'reason': result['phase2d']['classification']
                             ['failure_reason'], 'kept_on_disk': True})
        elif result['episode_id'] not in indexed:
            excluded.append({'episode_id': result['episode_id'],
                             'reason': 'not_indexed', 'kept_on_disk': True})
    report = write_report(work_root, rows, results, excluded, dataset_report, ds)
    R._log(os.path.join(root_abs, 'runner_log.txt'),
           'regenerated quality_report.json from %d run artifacts' % len(results))
    print(json.dumps({
        'regenerated': True,
        'failure_reason_counts': report['failure_reason_counts'],
        'quality_overall': report['quality']['overall'],
        'dataset': {k: dataset_report.get(k) for k in
                    ('valid', 'episode_count', 'sample_count')},
    }, indent=2, sort_keys=True, allow_nan=False))
    return 0


def mode_plan(args):
    print(json.dumps(D.full_plan(), indent=2, sort_keys=True, allow_nan=False))
    return 0


def mode_dry_run(args):
    plan = D.boundary_plan()
    print('round: boundary smoke')
    print('anchor source: %s' % plan['anchor_source']['path'])
    print('criteria version: %s' % plan['criteria']['version'])
    print('refused candidates (never launched):')
    for ref in plan['refused']:
        print('  %s :: %s' % (ref['candidate_id'],
                              [x['reason'] for x in ref['problems']]))
    print('external_push: %s' % D.external_push_plan()['reason'])
    print('planned episodes: %d (~%ds)' % (
        len(plan['rows']), plan['estimated_total_seconds']))
    for row in plan['rows']:
        print('  %s %s kind=%s param=%s abs=%s seed=%s risk=%s'
              % (row['episode_id'], row['candidate_id'], row['kind'],
                 row['param'], row['absolute'], row['seed'],
                 row['expected_risk']))
    return 0


def _execute(rows, args):
    root_abs, replaced = R.guard_root(args.root, args.allow_overwrite)
    work_root = os.path.join(root_abs, 'work')
    os.makedirs(work_root, exist_ok=True)
    log_path = os.path.join(root_abs, 'runner_log.txt')
    R._log(log_path, '=== phase2d boundary start root=%s rows=%d replaced=%s ==='
           % (root_abs, len(rows), replaced))
    results = []
    for i, row in enumerate(rows):
        domain = 220 + i
        results.append(run_candidate(row, work_root, args.timeout, log_path,
                                     domain, args.allow_overwrite))
    ds, excluded, dataset_report = build_dataset(
        work_root, results, args.train_ratio, args.val_ratio, args.split_seed)
    report = write_report(work_root, rows, results, excluded, dataset_report, ds)
    print(json.dumps({
        'runs_total': len(results),
        'runs_ok': sum(1 for r in results if r.get('ok')),
        'failure_reason_counts': report['failure_reason_counts'],
        'excluded': excluded,
        'dataset': {k: dataset_report.get(k) for k in
                    ('valid', 'episode_count', 'sample_count')},
    }, indent=2, sort_keys=True, allow_nan=False))
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description='Phase 2D failure/boundary runner')
    parser.add_argument('--mode', required=True,
                        choices=['plan', 'dry-run', 'smoke', 'regenerate'])
    parser.add_argument('--root', default=DEFAULT_ROOT)
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--train-ratio', type=float, default=0.7)
    parser.add_argument('--val-ratio', type=float, default=0.2)
    parser.add_argument('--split-seed', type=int, default=2028)
    parser.add_argument('--allow-overwrite', action='store_true')
    args = parser.parse_args(argv)
    if args.mode == 'plan':
        return mode_plan(args)
    if args.mode == 'dry-run':
        return mode_dry_run(args)
    if args.mode == 'regenerate':
        if not os.path.isdir(args.root):
            raise D.Phase2DError('root does not exist: %s' % args.root)
        return mode_regenerate(args)
    R.guard_root(args.root, args.allow_overwrite)
    if args.mode == 'smoke':
        return _execute(D.boundary_plan()['rows'], args)
    return 0


if __name__ == '__main__':
    sys.exit(main())
