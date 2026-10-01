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
Phase 2C Baseline B runner (parameter-perturbation study).

Reuses the already-validated Phase 2B launch/collect/validate/label pipeline
(``run_task4_phase2b_baseline_a``) but drives Phase 2C candidates with absolute
parameters and full provenance. It never touches the frozen sample schema, never
uses ``cmd_vel``, and never silently drops or clamps a parameter.

Modes:
  plan      read-only: print the full round-1/round-2 plan, write nothing.
  dry-run   read-only: validate candidates/roots and print launch commands.
  smoke     run the round-1 univariate smoke (2 extreme candidates per factor +
            default-anchor controls; refused candidates are never launched).
  batch     run a full round (``--round univariate|mixed``); gated separately.

Every root is guarded: an existing non-empty root is refused unless
``--allow-overwrite`` is given.
"""

import argparse
import hashlib
import json
import os
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import phase2c_baseline_b as C  # noqa: E402
import phase2b_baseline_a as B  # noqa: E402
import phase2a_episode_dataset as ep  # noqa: E402
import run_task4_phase2b_baseline_a as R  # noqa: E402

WS = os.path.dirname(_TOOLS)
DEFAULT_ROOT_BASE = os.path.join(WS, 'docs', 'task4_phase2c_baseline_b_evidence')


def _root_for(args):
    if args.root:
        return args.root
    return os.path.join(DEFAULT_ROOT_BASE, args.round)


def _row_for_runner(row):
    """Adapt a Phase 2C plan row to the shared Phase 2B runner row schema."""
    param = row.get('param')
    multiplier = row.get('multiplier')
    return {
        'episode_id': row['episode_id'],
        'config_id': row['candidate_id'],
        'seed': row['seed'],
        'step_length_multiplier': (multiplier if param == 'step_length_m'
                                   else None),
        'overrides': dict(row['overrides']),
    }


def run_candidate(row, work_root, timeout, log_path, domain_id,
                  allow_overwrite):
    shared = _row_for_runner(row)
    result = R.run_one(shared, work_root, timeout, log_path, domain_id,
                       allow_overwrite=allow_overwrite)
    result['phase2c'] = {
        'candidate_id': row['candidate_id'],
        'factor': row['factor'],
        'param': row.get('param'),
        'multiplier': row.get('multiplier'),
        'absolute': row.get('absolute', {}),
        'angle_deg': row.get('angle_deg'),
    }
    result_path = os.path.join(work_root, 'runs', row['episode_id'],
                               'result.json')
    R._write_json(result_path, result)
    return result


def build_dataset(work_root, results, round_name, train_ratio, val_ratio,
                  split_seed):
    inputs = []
    excluded = []
    for result in results:
        if not result.get('ok'):
            excluded.append({'episode_id': result.get('episode_id'),
                             'reason': result.get('reason')})
            continue
        labels = result['labels']
        if any(labels.get(k) is None for k in (
                'duration_sec', 'mean_speed_x_mps', 'max_torso_tilt_deg',
                'energy_qdot', 'fell')):
            excluded.append({'episode_id': result['episode_id'],
                             'reason': 'missing_required_metric',
                             'missing': labels.get('missing')})
            continue
        param_set = ep.param_set_from_producer(result['run_cfg'])
        p2c = result['phase2c']
        multiplier = p2c['multiplier']
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
                success=labels['success'],
                usable=labels['usable'],
                failure_reason=labels['failure_reason'],
                controller_version=B.CONTROLLER_VERSION,
                baseline_config=p2c['candidate_id'],
                baseline_round=round_name,
                perturbation_factor=p2c['factor'],
                perturbation_param=p2c['param'],
                perturbation_multiplier=(multiplier
                                         if multiplier is not None else 1.0),
                perturbation_angle_deg=p2c['angle_deg'],
                source_run_id=result['episode_id'],
                target_duration_sec=labels['target_duration_sec'],
                body_min_z_m=labels['body_min_z_m'],
                energy_qdot_definition=labels.get('energy_qdot_definition'),
            )
        except ep.MetadataValidationError as exc:
            excluded.append({'episode_id': result['episode_id'],
                             'reason': 'metadata_invalid',
                             'detail': exc.errors})
            continue
        run_dir = os.path.join(work_root, 'runs', result['episode_id'], 'run')
        inputs.append(ep.EpisodeInput(result['episode_id'], run_dir, meta))
    if not inputs:
        return None, excluded, {}
    ds = ep.EpisodeDataset(os.path.join(work_root, 'dataset'))
    report = ds.build(inputs, dataset_name='nao_gait_baseline_b_%s' % round_name,
                      sample_schema={'name': 'nao_gazebo_trajectory',
                                     'version': '1.0.1'},
                      split_seed=split_seed, train_ratio=train_ratio,
                      val_ratio=val_ratio, overwrite=True)
    return ds, excluded, report


def write_report(work_root, round_name, plan_or_rows, results, excluded,
                 dataset_report, ds):
    episodes = []
    provenance = []
    for result in results:
        if not result.get('ok'):
            continue
        labels = result['labels']
        p2c = result['phase2c']
        episodes.append({
            'episode_id': result['episode_id'],
            'config_id': p2c['candidate_id'],
            'seed': result['seed'],
            'step_length_m': result['run_cfg']['step_length_m'],
            'frequency_hz': result['run_cfg']['step_frequency_hz'],
            'labels': labels,
        })
        run_config_path = os.path.join(work_root, 'runs',
                                       result['episode_id'], 'run_config.json')
        run_config_sha = None
        if os.path.isfile(run_config_path):
            h = hashlib.sha256()
            with open(run_config_path, 'rb') as fh:
                for chunk in iter(lambda: fh.read(65536), b''):
                    h.update(chunk)
            run_config_sha = h.hexdigest()
        provenance.append({
            'episode_id': result['episode_id'],
            'candidate_id': p2c['candidate_id'],
            'factor': p2c['factor'],
            'param': p2c['param'],
            'multiplier': p2c['multiplier'],
            'angle_deg': p2c['angle_deg'],
            'absolute_overrides': p2c['absolute'],
            'seed': result['seed'],
            'run_config_sha': run_config_sha,
            'frozen_valid': result.get('frozen_valid'),
            'residual_processes': result.get('residual_processes'),
        })
    split_counts = {}
    index_rows = []
    if ds is not None and os.path.isdir(ds.root):
        index_rows = ds.load_index()
        for row in index_rows:
            split_counts[row['split']] = split_counts.get(row['split'], 0) + 1
    report = {
        'round': round_name,
        'plan': plan_or_rows,
        'execution': {
            'runs_ok': sum(1 for r in results if r.get('ok')),
            'runs_total': len(results),
            'failed': [{'episode_id': r.get('episode_id'),
                        'reason': r.get('reason')}
                       for r in results if not r.get('ok')],
            'excluded_from_dataset': excluded,
            'dataset_build': dataset_report,
        },
        'provenance': provenance,
        'quality': B.quality_report(episodes, split_counts, index_rows),
        'speed_basis': ('mean_speed_x_mps measured from body_pose trace; '
                        'cmd_vel unused'),
        'seed_semantics': ('fixed integer provenance id; the frozen controller '
                           'has no RNG'),
    }
    R._write_json(os.path.join(work_root, 'quality_report.json'), report)
    return report


# ---------------------------------------------------------------------------
# modes
# ---------------------------------------------------------------------------
def mode_plan(args):
    print(json.dumps(C.full_plan(), indent=2, sort_keys=True, allow_nan=False))
    return 0


def mode_dry_run(args):
    plan = C.build_univariate_smoke_plan()
    print('round: univariate smoke')
    print('anchor source: %s' % plan['anchor_source']['path'])
    print('refused candidates (never launched):')
    for ref in plan['refused']:
        print('  %s :: %s' % (ref['candidate_id'],
                              [p['reason'] for p in ref['problems']]))
    print('planned episodes: %d (~%ds)' % (
        len(plan['rows']), plan['estimated_total_seconds']))
    for row in plan['rows']:
        print('  %s %s factor=%s param=%s mult=%s seed=%s overrides=%s'
              % (row['episode_id'], row['candidate_id'], row['factor'],
                 row['param'], row['multiplier'], row['seed'],
                 row['overrides']))
    return 0


def _execute(rows, args):
    root = _root_for(args)
    root_abs, replaced = R.guard_root(root, args.allow_overwrite)
    work_root = os.path.join(root_abs, 'work')
    os.makedirs(work_root, exist_ok=True)
    log_path = os.path.join(root_abs, 'runner_log.txt')
    R._log(log_path, '=== phase2c %s start root=%s rows=%d replaced=%s ==='
           % (args.round, root_abs, len(rows), replaced))
    results = []
    for i, row in enumerate(rows):
        domain = 200 + i
        results.append(run_candidate(row, work_root, args.timeout, log_path,
                                     domain, args.allow_overwrite))
    ds, excluded, dataset_report = build_dataset(
        work_root, results, args.round, args.train_ratio, args.val_ratio,
        args.split_seed)
    report = write_report(work_root, args.round, rows, results, excluded,
                          dataset_report, ds)
    print(json.dumps({
        'round': args.round,
        'runs_ok': sum(1 for r in results if r.get('ok')),
        'runs_total': len(results),
        'excluded': excluded,
        'dataset': {k: dataset_report.get(k) for k in
                    ('valid', 'episode_count', 'sample_count')},
        'quality_overall': report['quality']['overall'],
    }, indent=2, sort_keys=True, allow_nan=False))
    return 0 if all(r.get('ok') for r in results) else 1


def mode_smoke(args):
    plan = C.build_univariate_smoke_plan()
    return _execute(plan['rows'], args)


def mode_batch(args):
    if args.round == 'univariate':
        plan = C.build_univariate_smoke_plan(
            seeds_per_candidate=args.seeds_per_candidate,
            anchor_repeats=max(3, args.seeds_per_candidate))
        rows = plan['rows']
    else:
        mixed = C.build_mixed_candidates()
        supported = [c for c in mixed if c['supported']]
        rows = []
        seeds = list(C.SEED_LIST[:args.seeds_per_candidate])
        index = 0
        for cand in supported:
            for seed in seeds:
                index += 1
                rows.append({
                    'episode_id': 'episode_%06d' % index,
                    'candidate_id': cand['candidate_id'],
                    'factor': 'mixed',
                    'param': None,
                    'multiplier': None,
                    'absolute': cand['absolute'],
                    'angle_deg': None,
                    'overrides': dict(cand['absolute']),
                    'seed': seed,
                })
    return _execute(rows, args)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Phase 2C Baseline B runner')
    parser.add_argument('--mode', required=True,
                        choices=['plan', 'dry-run', 'smoke', 'batch'])
    parser.add_argument('--round', default='univariate',
                        choices=['univariate', 'mixed'])
    parser.add_argument('--root', default=None)
    parser.add_argument('--seeds-per-candidate', type=int, default=1)
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--train-ratio', type=float, default=0.7)
    parser.add_argument('--val-ratio', type=float, default=0.2)
    parser.add_argument('--split-seed', type=int, default=2027)
    parser.add_argument('--allow-overwrite', action='store_true')
    args = parser.parse_args(argv)
    if args.mode == 'plan':
        return mode_plan(args)
    if args.mode == 'dry-run':
        return mode_dry_run(args)
    # Refuse an existing root before any ROS/Gazebo check or launch.
    R.guard_root(_root_for(args), args.allow_overwrite)
    if args.mode == 'smoke':
        return mode_smoke(args)
    return mode_batch(args)


if __name__ == '__main__':
    sys.exit(main())
