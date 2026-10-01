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
Phase 2B Baseline A collection runner (real headless Gazebo + ROS).

For each fixed (config, seed) row it:
  1. launches the approved Phase 4B gait producer + read-only Phase 4B trace and
     a Phase 4D frozen-schema collector in an isolated ROS_DOMAIN_ID/GZ_PARTITION;
  2. waits for the atomic collector summary, then shuts the group down;
  3. validates the frozen run with the independent golden validator;
  4. derives honest labels from measured samples/traces (never cmd_vel, never a
     fabricated default) via ``phase2b_baseline_a``;
  5. after all rows, builds the Phase 2A episode dataset and writes a quality
     report.

Modes:
  plan       read-only: print the frozen sampling plan (configs/seeds).
  dry-run    read-only: validate sources/paths and print the launch commands.
  smoke      run the first seed of every config (real collection).
  batch      run every row of the frozen plan (real collection).

The frozen Phase 4D sample schema, the frozen collector and the frozen golden
validator are never modified. The controller supports no ``seed`` input (no RNG,
no sensor noise in nao_simple.sdf); seeds are recorded honestly as replicate
identifiers -- never claimed to inject randomness.
"""

import argparse
import importlib.util
import json
import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import phase2b_baseline_a as B  # noqa: E402
import phase2a_episode_dataset as ep  # noqa: E402

WS = os.path.dirname(_TOOLS)
# Dedicated, phase-scoped run root. The historical 30-episode evidence root
# (docs/task4_phase2b_baseline_a_evidence) is NEVER written by default.
DEFAULT_ROOT = os.path.join(WS, 'docs', 'task4_phase2b_baseline_a_runs')

LAUNCH_BASE = ['ros2', 'launch', 'nao_gz_sim', 'nao_sim.launch.py',
               'gui:=false', 'use_rsp:=true',
               'enable_phase4a:=true', 'phase4a_run_probe:=false',
               'phase4a_run_trace:=false', 'enable_phase4b:=true',
               'phase4b_run_gait:=true', 'phase4b_run_trace:=true',
               'phase4b_record_duration:=0.0']

# Collection timing: the producer must outlive the 12 s collector window.
COLLECT_OVERRIDES = {'duration_sec': 16.0, 'stop_ramp_sec': 1.5,
                     'hold_final_sec': 1.0}


class RunnerError(Exception):
    def __init__(self, reason, stage, detail=None):
        super().__init__(reason)
        self.reason = reason
        self.stage = stage
        self.detail = detail


def guard_root(root, allow_overwrite):
    """
    Refuse to write into an existing non-empty root unless explicitly allowed.

    Returns the absolute root path. This is the single overwrite gate: without
    ``allow_overwrite`` an existing dataset/evidence directory is never touched,
    and no silent delete/append happens.
    """
    root_abs = os.path.abspath(root)
    if os.path.exists(root_abs) and not os.path.isdir(root_abs):
        raise RunnerError('root_not_a_directory', 'guard_root', root_abs)
    if os.path.isdir(root_abs) and os.listdir(root_abs):
        if not allow_overwrite:
            raise RunnerError(
                'root_exists_refused',
                'guard_root',
                {'root': root_abs,
                 'hint': 'pass --allow-overwrite to replace this root'})
        print('[guard_root] --allow-overwrite: replacing existing root %s'
              % root_abs)
        return root_abs, True
    return root_abs, False


def _load_json(path, default=None):
    if not os.path.isfile(path):
        return default
    with open(path, 'r', encoding='utf-8') as fh:
        return json.load(fh)


def _write_json(path, obj):
    directory = os.path.dirname(path) or '.'
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.tmp_p2b_', dir=directory)
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


def _log(path, message):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'a', encoding='utf-8') as fh:
        fh.write('[%s] %s\n' % (time.strftime('%H:%M:%S'), message))


def param_args(cfg):
    args = []
    for key in sorted(cfg):
        value = cfg[key]
        if isinstance(value, bool):
            value = 'true' if value else 'false'
        name = key if key.startswith('actuator_') else 'phase4b_' + key
        args.append('%s:=%s' % (name, value))
    return args


def build_launch_cmd(run_cfg, run_dir, trace_dir, summary_path):
    cmd = list(LAUNCH_BASE)
    cmd += param_args(run_cfg)
    cmd += ['phase4b_evidence_dir:=%s' % trace_dir,
            'enable_phase4d:=true', 'phase4d_run_collector:=true',
            'phase4d_output_dir:=%s' % run_dir,
            'phase4d_summary_path:=%s' % summary_path,
            'phase4d_duration_sec:=%s' % B.TARGET_COLLECT_SEC,
            'phase4d_overwrite:=true']
    return cmd


def _reap_group(pgid, timeout=25):
    try:
        os.killpg(pgid, signal.SIGINT)
    except ProcessLookupError:
        return 0
    for _ in range(6):
        time.sleep(1)
        if not _group_alive(pgid):
            return 0
    try:
        os.killpg(pgid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    for _ in range(timeout):
        time.sleep(1)
        if not _group_alive(pgid):
            break
    return _residual_count()


def _group_alive(pgid):
    try:
        os.killpg(pgid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def _residual_count():
    try:
        out = subprocess.run(
            ['bash', '-lc',
             "ps -eo cmd | grep -E 'gz sim|nao_sim.launch|"
             "install/nao_gz_sim/lib/nao_gz_sim|ros_gz_bridge/parameter_bridge'"
             " | grep -v grep | wc -l"],
            stdout=subprocess.PIPE, text=True).stdout.strip()
        return int(out or '0')
    except (OSError, ValueError):
        return -1


def validate_frozen_run(run_dir):
    path = os.path.join(_TOOLS, 'validate_task4_phase4d_dataset.py')
    spec = importlib.util.spec_from_file_location('p2b_golden', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.validate_dataset(os.path.abspath(run_dir))


def run_one(row, work_root, timeout, log_path, domain_id,
            allow_overwrite=False):
    episode_id = row['episode_id']
    base = os.path.join(work_root, 'runs', episode_id)
    run_dir = os.path.join(base, 'run')
    trace_dir = os.path.join(base, 'trace')
    summary_path = os.path.join(base, 'summary.json')
    launch_log = os.path.join(base, 'launch.log')
    result_path = os.path.join(base, 'result.json')
    if os.path.isdir(base):
        if not allow_overwrite:
            raise RunnerError('episode_dir_exists_refused', 'run_one', base)
        shutil.rmtree(base)
    os.makedirs(trace_dir)
    os.makedirs(run_dir)

    anchor = B.load_anchor_config()[0]
    run_cfg = B.run_config_for(anchor, row['overrides'])
    run_cfg.update(COLLECT_OVERRIDES)
    cmd = build_launch_cmd(run_cfg, run_dir, trace_dir, summary_path)
    _write_json(os.path.join(base, 'run_config.json'), run_cfg)
    with open(os.path.join(base, 'launch_cmd.txt'), 'w', encoding='utf-8') as fh:
        fh.write(' '.join(cmd) + '\n')

    env = dict(os.environ)
    env['GZ_PARTITION'] = 'p2b_%s' % episode_id
    env['ROS_DOMAIN_ID'] = str(domain_id)
    _log(log_path, '%s launching (config=%s seed=%s domain=%d)'
         % (episode_id, row['config_id'], row['seed'], domain_id))

    with open(launch_log, 'w', encoding='utf-8') as log:
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                env=env, start_new_session=True)
    pgid = os.getpgid(proc.pid)
    started = time.time()
    timed_out = True
    while time.time() - started < timeout:
        if os.path.isfile(summary_path) and os.path.getsize(summary_path) > 0:
            timed_out = False
            break
        if proc.poll() is not None:
            break
        time.sleep(1)
    residual = _reap_group(pgid)
    summary = _load_json(summary_path)
    _log(log_path, '%s finished timed_out=%s residual=%d wall=%.1fs'
         % (episode_id, timed_out, residual, time.time() - started))

    result = {'episode_id': episode_id, 'config_id': row['config_id'],
              'seed': row['seed'],
              'step_length_multiplier': row['step_length_multiplier'],
              'timed_out': timed_out,
              'residual_processes': residual, 'run_cfg': run_cfg}
    if summary is None:
        result['ok'] = False
        result['reason'] = 'launch_timeout' if timed_out else 'no_summary'
        _write_json(result_path, result)
        return result

    report = validate_frozen_run(run_dir)
    result['frozen_valid'] = report['valid']
    result['frozen_errors'] = report['errors'][:3]
    if not report['valid']:
        result['ok'] = False
        result['reason'] = 'frozen_invalid'
        _write_json(result_path, result)
        return result

    states = _load_jsonl(os.path.join(trace_dir, 'state_trace.jsonl'))
    if not states:
        result['ok'] = False
        result['reason'] = 'no_state_trace'
        _write_json(result_path, result)
        return result
    ready = states[0]['t']

    labels = B.derive_episode_labels(run_dir, trace_dir, run_cfg, ready, summary)
    result['ok'] = True
    result['labels'] = {k: v for k, v in labels.items() if k != 'metrics'}
    # keep a compact metrics subset for the report
    m = labels.get('metrics', {})
    result['metrics_subset'] = {k: m.get(k) for k in (
        'body_dx', 'body_dy', 'body_min_z', 'body_max_z', 'max_roll_deg',
        'max_pitch_deg', 'max_yaw_abs_from_start_deg', 'window', 'imu_nonfinite',
        'command')}
    _write_json(result_path, result)
    _log(log_path, '%s labels success=%s fell=%s usable=%s speed=%s missing=%s'
         % (episode_id, labels['success'], labels['fell'], labels['usable'],
            labels['mean_speed_x_mps'], labels['missing']))
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


# ---------------------------------------------------------------------------
# dataset assembly + report
# ---------------------------------------------------------------------------
def build_dataset(work_root, results, dataset_name='nao_gait_baseline_a',
                  train_ratio=0.7, val_ratio=0.2, split_seed=2026):
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
                baseline_config=result['config_id'],
                step_length_multiplier=result.get('step_length_multiplier'),
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
    report = ds.build(inputs, dataset_name=dataset_name,
                      sample_schema={'name': 'nao_gazebo_trajectory',
                                     'version': '1.0.1'},
                      split_seed=split_seed, train_ratio=train_ratio,
                      val_ratio=val_ratio, overwrite=True)
    return ds, excluded, report


def write_report(work_root, plan, results, dataset_report, excluded, ds):
    episodes = []
    for result in results:
        if not result.get('ok'):
            continue
        labels = result['labels']
        episodes.append({
            'episode_id': result['episode_id'],
            'config_id': result['config_id'],
            'seed': result['seed'],
            'step_length_m': result['run_cfg']['step_length_m'],
            'frequency_hz': result['run_cfg']['step_frequency_hz'],
            'labels': labels,
        })
    split_counts = {}
    index_rows = []
    if ds is not None and os.path.isdir(ds.root):
        index_rows = ds.load_index()
        for row in index_rows:
            split_counts[row['split']] = split_counts.get(row['split'], 0) + 1
    report = {
        'plan': plan,
        'execution': {
            'rows_total': len(plan['rows']),
            'runs_ok': sum(1 for r in results if r.get('ok')),
            'runs_failed': [{'episode_id': r.get('episode_id'),
                             'reason': r.get('reason')}
                            for r in results if not r.get('ok')],
            'excluded_from_dataset': excluded,
            'dataset_build': dataset_report,
        },
        'quality': B.quality_report(episodes, split_counts, index_rows),
        'seed_semantics': ('replicate/provenance identifier only; the frozen '
                           'controller is deterministic and consumes no seed'),
        'speed_basis': 'mean_speed_x_mps measured from body_pose trace; cmd_vel unused',
    }
    _write_json(os.path.join(work_root, 'quality_report.json'), report)
    return report


# ---------------------------------------------------------------------------
# modes
# ---------------------------------------------------------------------------
def mode_plan(args):
    plan = B.sampling_plan(args.seeds_per_config)
    print(json.dumps(plan, indent=2, sort_keys=True, allow_nan=False))
    return 0


def mode_dry_run(args):
    plan = B.sampling_plan(args.seeds_per_config)
    print('anchor source: %s' % plan['anchor_source'])
    print('default step_length_m=%s frequency_hz=%s'
          % (plan['anchor_config']['step_length_m'],
             plan['anchor_config']['step_frequency_hz']))
    print('ros2 on PATH   : %s' % bool(shutil.which('ros2')))
    print('golden validator: %s' % os.path.isfile(
        os.path.join(_TOOLS, 'validate_task4_phase4d_dataset.py')))
    for row in plan['rows']:
        anchor = plan['anchor_config']
        run_cfg = B.run_config_for(anchor, row['overrides'])
        run_cfg.update(COLLECT_OVERRIDES)
        run_dir = '<root>/runs/%s/run' % row['episode_id']
        trace_dir = '<root>/runs/%s/trace' % row['episode_id']
        summary = '<root>/runs/%s/summary.json' % row['episode_id']
        cmd = build_launch_cmd(run_cfg, run_dir, trace_dir, summary)
        print('\n# %s config=%s seed=%s step_length_m=%s'
              % (row['episode_id'], row['config_id'], row['seed'],
                 row['step_length_m']))
        print(' '.join(cmd))
    return 0


def _execute(rows, args):
    root_abs, replaced = guard_root(args.root, args.allow_overwrite)
    work_root = os.path.join(root_abs, 'work')
    os.makedirs(work_root, exist_ok=True)
    log_path = os.path.join(root_abs, 'runner_log.txt')
    _log(log_path, '=== batch start root=%s mode=%s rows=%d replaced=%s ==='
         % (root_abs, args.mode, len(rows), replaced))
    results = []
    for i, row in enumerate(rows):
        domain = 160 + i
        result = run_one(row, work_root, args.timeout, log_path, domain,
                         allow_overwrite=args.allow_overwrite)
        results.append(result)
    ds, excluded, dataset_report = build_dataset(
        work_root, results, train_ratio=args.train_ratio,
        val_ratio=args.val_ratio, split_seed=args.split_seed)
    plan = B.sampling_plan(args.seeds_per_config)
    plan = dict(plan, rows=rows)
    report = write_report(work_root, plan, results, dataset_report,
                          excluded, ds)
    print(json.dumps({
        'runs_ok': sum(1 for r in results if r.get('ok')),
        'runs_total': len(results),
        'excluded': excluded,
        'dataset': {k: dataset_report.get(k) for k in
                    ('valid', 'episode_count', 'sample_count')},
        'quality_overall': report['quality']['overall'],
    }, indent=2, sort_keys=True, allow_nan=False))
    return 0 if all(r.get('ok') for r in results) else 1


def mode_smoke(args):
    plan = B.sampling_plan(args.seeds_per_config)
    rows = [plan['rows'][i] for i in (0, args.seeds_per_config,
                                     2 * args.seeds_per_config)
            if i < len(plan['rows'])]
    return _execute(rows, args)


def mode_batch(args):
    plan = B.sampling_plan(args.seeds_per_config)
    return _execute(plan['rows'], args)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Phase 2B Baseline A runner')
    parser.add_argument('--mode', required=True,
                        choices=['plan', 'dry-run', 'smoke', 'batch'])
    parser.add_argument('--root', default=DEFAULT_ROOT)
    parser.add_argument('--seeds-per-config', type=int, default=3)
    parser.add_argument('--timeout', type=float, default=120.0)
    parser.add_argument('--train-ratio', type=float, default=0.7)
    parser.add_argument('--val-ratio', type=float, default=0.2)
    parser.add_argument('--split-seed', type=int, default=2026)
    parser.add_argument('--allow-overwrite', action='store_true',
                        help='explicitly allow replacing an existing --root')
    args = parser.parse_args(argv)
    if args.mode == 'plan':
        return mode_plan(args)
    if args.mode == 'dry-run':
        return mode_dry_run(args)
    # Fail fast on an existing root BEFORE any ROS/Gazebo check or launch.
    guard_root(args.root, args.allow_overwrite)
    if not shutil.which('ros2'):
        raise RunnerError('ros2 not on PATH; source ROS before running',
                          'environment')
    if args.mode == 'smoke':
        return mode_smoke(args)
    return mode_batch(args)


if __name__ == '__main__':
    sys.exit(main())
