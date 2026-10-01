#!/usr/bin/env python3
"""Phase A — NAO Gazebo data-acquisition minimal closed loop (one episode).

Runs exactly ONE B-only episode end to end and publishes a self-describing,
atomic episode directory:

    Frozen gait parameters + seed + duration
        -> ros2 launch (Gazebo + gait producer + phase4b trace + phase4d collector)
        -> NAO gait execution
        -> raw trajectory collection (frozen schema v1.0.1 samples + raw traces)
        -> atomic episode publication (staging dir -> os.replace)
        -> manifest.json + provenance.json
        -> episode_integrity (nao_episode_integrity)

The runner only prepares / launches / monitors / stops / cleans up / records
provenance. It never judges gait quality and never computes a contract status
(P1-R v3 gates are out of scope for Phase A).

Episode layout (<out-root>/):

    manifest.json            episode manifest (episode_id, timing, signals, status)
    provenance.json          who/when/seed/gait/gazebo/code/duration/termination
    episode_integrity.json   episode_integrity + failure class (+ reserved fields)
    launch_cmd.txt           exact `ros2 launch` argv actually executed
    gz_sim_cmd.actual.txt    real `gz sim` argv observed in /proc (proves --seed)
    gz_sim_cmd.expected.txt  deterministically reconstructed `gz sim` argv
    run_config.json          the resolved frozen gait config
    launch.log               combined launch stdout/stderr
    collector_summary.json   phase4d collector accounting (outside dataset/)
    dataset/                 frozen schema v1.0.1: manifest.json + samples.jsonl
    trace/                   raw per-topic JSONL (clock/command/state/contact/body_pose/imu)

Usage:
    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 tools/run_phase_a_episode.py \
        --episode-id smoke_01 --seed 12345 --duration 8.0 \
        --out-root docs/task_p0a_gait_search/phase_a_evidence/smoke_01
"""

import argparse
import json
import os
import shlex
import shutil
import signal
import subprocess
import sys
import time

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import run_p0a_gait_candidate as R  # noqa: E402  (pure builders + reap helpers)
import nao_episode_integrity as EI  # noqa: E402

WS = os.path.dirname(_TOOLS)
VALIDATOR = os.path.join(_TOOLS, 'validate_task4_phase4d_dataset.py')

# Frozen B-only gait parameters for Phase A acquisition (§11: values are frozen;
# the gait *mathematics* is never modified here). Parity with the reviewed B
# snapshot (explicit_support_swing / 0.8 Hz / sway_sign -1). Overridable per run.
FROZEN_GAIT = {
    'gait_method': 'walk',
    'gait_phase_mode': 'explicit_support_swing',
    'step_length_m': 0.02,
    'step_frequency_hz': 0.8,
    'swing_height_m': 0.008,
    'double_support_ratio': 0.3,
    'leg_extension_m': 0.24401,
    'gait_blend_sec': 1.5,
    'sway_amplitude_rad': 0.06,
    'sway_sign': -1,
    'arm_swing_rad': 0.12,
    'arm_neutral_pitch_rad': 1.5,
    'arm_neutral_roll_rad': 0.0,
    'elbow_bend_rad': 0.3,
    'arm_phase_offset_rad': 0.0,
    'arm_swing_sign': 1,
    'foot_pitch_swing_rad': 0.1,
    'walking_direction': 'forward',
    'command_rate_hz': 100.0,
    'start_delay_sec': 0.5,
    'stop_ramp_sec': 1.5,
    'hold_final_sec': 1.0,
    'actuator_p_gain': 15.0,
    'actuator_max_velocity': 4.0,
    'actuator_hold_on_start': True,
}

# Never write an episode into the surviving/archived frozen evidence.
FORBIDDEN_OUT_ROOT_MARKERS = (
    os.path.join('docs', 'task_p0a_gait_search',
                 'stage7_b_only_holdout_reentry_v4_evidence'),
    os.path.join(os.path.dirname(WS), 'backup'),
)


class PhaseAError(Exception):
    pass


def resolve_config(overrides):
    """Lowest -> highest precedence: FROZEN_GAIT < caller overrides (§9)."""
    cfg = dict(FROZEN_GAIT)
    cfg.update(overrides or {})
    return cfg


def _sha256_file(path):
    if not os.path.isfile(path):
        return None
    import hashlib
    return hashlib.sha256(open(path, 'rb').read()).hexdigest()


def _capture_gz_argv():
    """Read the live `gz sim` argv from /proc (real execution path, §10).

    ``gz`` is a ruby wrapper whose /proc/<pid>/cmdline reports the entire
    command as ONE argv element (e.g. ``"gz sim -s -r -v 4 --seed 12345 ..."``),
    so the joined form is split back into an argv list. The bash wrapper that
    launched it is skipped by name.
    """
    for pid in os.listdir('/proc'):
        if not pid.isdigit():
            continue
        try:
            comm = open('/proc/%s/comm' % pid, encoding='utf-8').read().strip()
            with open('/proc/%s/cmdline' % pid, 'rb') as fh:
                raw = fh.read().split(b'\0')
        except OSError:
            continue
        args = [a.decode('utf-8', 'replace') for a in raw if a]
        if not args or os.path.basename(args[0]) == 'bash':
            continue
        joined = ' '.join(args)
        if 'gz sim' not in joined:
            continue
        if not (os.path.basename(args[0]) in ('gz', 'gz-sim', 'ruby')
                or comm in ('gz', 'gz-sim', 'ruby')):
            continue
        return shlex.split(joined) if len(args) == 1 else args
    return None


def _seed_from_argv(argv):
    if argv and '--seed' in argv:
        i = argv.index('--seed')
        if i + 1 < len(argv):
            try:
                return int(argv[i + 1])
            except ValueError:
                return None
    return None


def _git_revision():
    try:
        p = subprocess.run(['git', '-C', WS, 'rev-parse', 'HEAD'],
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                           text=True, timeout=10)
        rev = p.stdout.strip()
        return (rev or None) if p.returncode == 0 else None
    except (OSError, subprocess.SubprocessError):
        return None


def _build_launch_cmd(cfg, trace_dir, dataset_dir, summary_path, record, gz_seed):
    cmd = list(R.LAUNCH_BASE)
    cmd += R.param_args(cfg)
    cmd += ['phase4b_evidence_dir:=%s' % trace_dir,
            'phase4b_record_duration:=%s' % record,
            'enable_phase4d:=true', 'phase4d_run_collector:=true',
            'phase4d_output_dir:=%s' % dataset_dir,
            'phase4d_summary_path:=%s' % summary_path,
            'phase4d_duration_sec:=%s' % record,
            'phase4d_overwrite:=true',
            'phase4d_validator_script:=%s' % VALIDATOR]
    if gz_seed is not None:
        cmd += ['gz_seed:=%d' % int(gz_seed)]
    return cmd


def run_episode(episode_id, seed, duration, out_root, overrides, timeout,
                domain_id, gz_seed, allow_overwrite):
    if not shutil.which('ros2'):
        raise PhaseAError('ros2 not on PATH; source ROS before running')
    real_out = os.path.realpath(out_root)
    for marker in FORBIDDEN_OUT_ROOT_MARKERS:
        m = os.path.realpath(marker)
        if real_out == m or real_out.startswith(m + os.sep):
            raise PhaseAError('refusing out_root inside frozen evidence: %s' % out_root)
    if os.path.exists(out_root) and not allow_overwrite:
        raise PhaseAError('out_root exists (use --allow-overwrite): %s' % out_root)

    cfg = resolve_config(overrides)
    cfg['duration_sec'] = float(duration)
    record = (cfg['start_delay_sec'] + float(duration) + cfg['stop_ramp_sec']
              + cfg['hold_final_sec'] + 1.0)

    staging = os.path.abspath(out_root) + '.staging.%d' % os.getpid()
    if os.path.exists(staging):
        shutil.rmtree(staging)
    dataset_dir = os.path.join(staging, 'dataset')
    trace_dir = os.path.join(staging, 'trace')
    summary_path = os.path.join(staging, 'collector_summary.json')
    os.makedirs(trace_dir)

    cmd = _build_launch_cmd(cfg, trace_dir, dataset_dir, summary_path, record, gz_seed)
    gz_cmd_expected = R.build_gz_sim_command(gz_seed)

    launch_log = os.path.join(staging, 'launch.log')
    with open(os.path.join(staging, 'launch_cmd.txt'), 'w', encoding='utf-8') as fh:
        fh.write(' '.join(cmd) + '\n')
    with open(os.path.join(staging, 'gz_sim_cmd.expected.txt'), 'w',
              encoding='utf-8') as fh:
        fh.write(' '.join(gz_cmd_expected) + '\n')
    with open(os.path.join(staging, 'run_config.json'), 'w', encoding='utf-8') as fh:
        json.dump(cfg, fh, indent=2, sort_keys=True, allow_nan=False)
    # Placeholder for the *observed* gz sim argv; overwritten after launch with
    # whatever was actually read from /proc (never left as the expected cmd).
    with open(os.path.join(staging, 'gz_sim_cmd.actual.txt'), 'w',
              encoding='utf-8') as fh:
        fh.write('UNVERIFIED: gz sim argv not yet observed\n')

    env = dict(os.environ)
    env['PATH'] = os.pathsep.join(
        e for e in os.environ.get('PATH', '').split(os.pathsep)
        if 'anaconda' not in e and 'condabin' not in e)
    env.pop('CONDA_PREFIX', None)
    env.pop('CONDA_DEFAULT_ENV', None)
    env['GZ_PARTITION'] = R.partition_id('phase_a', episode_id)
    env['ROS_DOMAIN_ID'] = str(domain_id)

    start_wall = time.strftime('%Y-%m-%dT%H:%M:%S%z')
    started = time.time()
    timed_out = True
    proc = None
    capture = {'gz_argv': None, 'residual': None, 'pgid': None}
    try:
        with open(launch_log, 'w', encoding='utf-8') as log:
            proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                    env=env, start_new_session=True)
        pgid = os.getpgid(proc.pid)
        capture['pgid'] = pgid
        while time.time() - started < timeout:
            if os.path.isfile(summary_path) and os.path.getsize(summary_path) > 0:
                timed_out = False
                break
            if proc.poll() is not None:
                # Launch exited; give the collector a moment to flush its summary.
                time.sleep(1.0)
                timed_out = not (os.path.isfile(summary_path)
                                 and os.path.getsize(summary_path) > 0)
                break
            time.sleep(0.5)
        capture['gz_argv'] = _capture_gz_argv()
        capture['residual'] = R._reap_group(pgid)
    finally:
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, OSError):
                pass

    end_wall = time.strftime('%Y-%m-%dT%H:%M:%S%z')
    residual = capture['residual']
    if residual is None:
        residual = R._residual_count()

    collector_summary = None
    if os.path.isfile(summary_path):
        try:
            collector_summary = json.load(open(summary_path, encoding='utf-8'))
        except (OSError, ValueError):
            collector_summary = None

    # Persist the *observed* gz sim invocation (real execution path, §10).
    with open(os.path.join(staging, 'gz_sim_cmd.actual.txt'), 'w',
              encoding='utf-8') as fh:
        fh.write((' '.join(capture['gz_argv']) if capture['gz_argv']
                  else 'UNVERIFIED: gz sim argv not observed in /proc') + '\n')

    # Authoritative gait-window start (C2): producer ready + start_delay.
    state_first, _ = EI._trace_t_span(os.path.join(trace_dir, 'state_trace.jsonl'))
    gait_start = (state_first + float(cfg['start_delay_sec'])
                  if state_first is not None else None)

    verdict = EI.evaluate(staging, requested_duration_sec=float(duration),
                          collector_summary=collector_summary,
                          episode_id=episode_id, gait_start_sec=gait_start,
                          record_window_duration_sec=record)
    # A hard launch failure (never produced a dataset) is an infrastructure
    # failure even if the dataset directory happens to exist but is empty.
    if not os.path.isdir(dataset_dir):
        verdict['failure_class'] = 'infrastructure_failure'
        verdict['episode_integrity'] = 'FAIL'
    if residual:
        verdict.setdefault('checks', {})['residual_processes_zero'] = {
            'pass': False, 'detail': residual}
        verdict['episode_integrity'] = 'FAIL'
        verdict['failure_class'] = verdict.get('failure_class') or 'infrastructure_failure'

    dur = verdict.get('duration', {})
    cov = verdict.get('coverage') or {}
    accepted_span = dur.get('accepted_sample_span_sec')
    execution_duration = dur.get('execution_duration_sec')
    gz_seed_actual = _seed_from_argv(capture['gz_argv'])
    seed_verified = (gz_seed is None and gz_seed_actual is None) or \
        (gz_seed is not None and gz_seed_actual == int(gz_seed))

    source_files = {
        'tools/run_phase_a_episode.py': os.path.abspath(__file__),
        'tools/nao_episode_integrity.py': os.path.join(_TOOLS, 'nao_episode_integrity.py'),
        'src/nao_gz_sim/scripts/phase4b_producer.py': os.path.join(
            WS, 'src/nao_gz_sim/scripts/phase4b_producer.py'),
        'src/nao_gz_sim/scripts/phase4d_collector.py': os.path.join(
            WS, 'src/nao_gz_sim/scripts/phase4d_collector.py'),
        'src/nao_gz_sim/scripts/nao_trajectory_schema.py': os.path.join(
            WS, 'src/nao_gz_sim/scripts/nao_trajectory_schema.py'),
        'src/nao_gz_sim/launch/nao_sim.launch.py': os.path.join(
            WS, 'src/nao_gz_sim/launch/nao_sim.launch.py'),
    }

    provenance = {
        'artifact': 'Phase A episode provenance',
        'episode_id': episode_id,
        'seed': {'requested': gz_seed, 'actual_gz_sim': gz_seed_actual,
                 'verified': seed_verified, 'partition': env['GZ_PARTITION'],
                 'ros_domain_id': domain_id,
                 'semantics': ('physics RNG seed forwarded to gz sim --seed; the '
                               'partition/domain id is NOT a physics seed')},
        'gait_parameters': cfg,
        'gazebo': {
            'command_actual': capture['gz_argv'],
            'command_expected': gz_cmd_expected,
            'world': os.path.join(WS, 'install', 'nao_gz_sim', 'share',
                                  'nao_gz_sim', 'worlds', 'nao_world.sdf'),
            'physics_engine': 'gz-physics-dartsim-plugin',
            'robot_model': 'nao (nao_simple.sdf runtime instance)',
            'gz_sim_version': (subprocess.run(['gz', 'sim', '--version'],
                                              stdout=subprocess.PIPE, text=True)
                               .stdout.splitlines()[0]
                               if shutil.which('gz') else None),
        },
        'launch_configuration': {'argv': cmd},
        'collector_configuration': {
            'dataset_dir': 'dataset', 'trace_dir': 'trace',
            'duration_sec': record, 'schema_version': '1.0.1',
            'validator_script': VALIDATOR},
        'code_revision': {'git_commit': _git_revision(),
                          'source_sha256': {k: _sha256_file(v)
                                            for k, v in source_files.items()}},
        'timing': {'start_timestamp': start_wall, 'end_timestamp': end_wall,
                   'wall_sec': time.time() - started},
        'duration': {
            'requested_gait_duration_sec': float(duration),
            'record_window_duration_sec': record,
            'execution_duration_sec': execution_duration,
            'execution_duration_source': dur.get('execution_duration_source'),
            'accepted_sample_span_sec': accepted_span,
            'gait_start_sec': gait_start,
            'gait_start_source': ('trace/state_trace.jsonl first t + '
                                  'gait_parameters.start_delay_sec'),
            'legacy_actual_duration_sec': accepted_span,
            'legacy_note': ('actual_duration_sec/duration_sec are DEPRECATED aliases '
                            'of accepted_sample_span_sec (not execution duration)')},
        'coverage': cov,
        'termination_reason': verdict.get('termination'),
        'seed_invocation_verified': seed_verified,
        'residual_processes': residual,
        'environment': {'ros_distro': os.environ.get('ROS_DISTRO'),
                        'python': sys.version},
        'outputs': {
            'dataset_manifest_sha256': _sha256_file(
                os.path.join(dataset_dir, 'manifest.json')),
            'dataset_samples_sha256': _sha256_file(
                os.path.join(dataset_dir, 'samples.jsonl')),
        },
        'timed_out': timed_out,
    }

    start_time = None
    end_time = None
    first_last = _samples_span(dataset_dir)
    if first_last:
        start_time, end_time = first_last

    manifest = {
        'artifact': 'Phase A episode manifest',
        'schema_name': 'nao_gazebo_episode',
        'schema_version': 'phase_a_1.0.0',
        'dataset_schema_version': '1.0.1',
        'episode_id': episode_id,
        'start_time': start_time,
        'end_time': end_time,
        'sample_count': verdict.get('sample_count'),
        'duration': {
            'requested_gait_duration_sec': float(duration),
            'record_window_duration_sec': record,
            'execution_duration_sec': execution_duration,
            'accepted_sample_span_sec': accepted_span,
            'legacy_duration_sec': accepted_span,
            'legacy_note': ('duration_sec/requested_duration_sec kept for backward '
                            'compatibility; duration_sec == accepted_sample_span_sec '
                            'and does NOT mean execution/gait/record duration')},
        'gait_relative_coverage': cov.get('gait_relative_coverage'),
        'coverage': cov,
        # Legacy aliases (explicitly labelled deprecated).
        'duration_sec': accepted_span,
        'requested_duration_sec': float(duration),
        'joint_order': _joint_order(),
        'signals': ['clock', 'joint_state', 'joint_command', 'imu',
                    'left_contact', 'right_contact', 'body_pose'],
        'source_topics': ['/clock', '/joint_states', '/joint_command',
                          '/imu/data', '/left_foot/contact',
                          '/right_foot/contact', '/body_pose'],
        'simulation_time': {'time_source': 'gazebo_simulation_time',
                            'first_ts': start_time, 'last_ts': end_time},
        'collection_status': verdict.get('termination'),
        'episode_integrity': verdict.get('episode_integrity'),
        'failure_class': verdict.get('failure_class'),
        'dataset': {'dir': 'dataset', 'manifest': 'dataset/manifest.json',
                    'samples': 'dataset/samples.jsonl', 'trace_dir': 'trace'},
        'provenance': 'provenance.json',
        'integrity_report': 'episode_integrity.json',
    }

    with open(os.path.join(staging, 'provenance.json'), 'w', encoding='utf-8') as fh:
        json.dump(provenance, fh, indent=2, sort_keys=True, allow_nan=False)
    with open(os.path.join(staging, 'manifest.json'), 'w', encoding='utf-8') as fh:
        json.dump(manifest, fh, indent=2, sort_keys=True, allow_nan=False)
    with open(os.path.join(staging, 'episode_integrity.json'), 'w',
              encoding='utf-8') as fh:
        json.dump(verdict, fh, indent=2, sort_keys=True, allow_nan=False)

    _publish(staging, out_root)
    return {'episode_dir': os.path.abspath(out_root), 'manifest': manifest,
            'provenance': provenance, 'integrity': verdict}


def _joint_order():
    import nao_trajectory_schema  # installed alongside the collector
    return list(nao_trajectory_schema.JOINT_ORDER)


def _samples_span(dataset_dir):
    path = os.path.join(dataset_dir, 'samples.jsonl')
    if not os.path.isfile(path):
        return None
    first = last = None
    with open(path, 'r', encoding='utf-8') as fh:
        for line in fh:
            if not line.strip():
                continue
            try:
                ts = json.loads(line)['timestamp']
            except (ValueError, KeyError):
                continue
            if first is None:
                first = ts
            last = ts
    return (first, last) if first is not None else None


def _publish(staging, out_root):
    if os.path.isdir(out_root):
        backup = out_root + '.replaced.%d' % os.getpid()
        os.replace(out_root, backup)
        try:
            os.replace(staging, out_root)
        except OSError:
            os.replace(backup, out_root)
            raise
        shutil.rmtree(backup, ignore_errors=True)
    else:
        os.replace(staging, out_root)


def main(argv=None):
    ap = argparse.ArgumentParser(description='Phase A single-episode acquisition')
    ap.add_argument('--episode-id', required=True)
    ap.add_argument('--seed', type=int, required=True, help='physics seed (gz sim --seed)')
    ap.add_argument('--duration', type=float, required=True,
                    help='requested walking duration [sim s]')
    ap.add_argument('--out-root', required=True)
    ap.add_argument('--override', action='append', default=[],
                    help='repeatable key=value frozen-gait override')
    ap.add_argument('--timeout', type=float, default=300.0)
    ap.add_argument('--domain-id', type=int, default=88)
    ap.add_argument('--allow-overwrite', action='store_true')
    args = ap.parse_args(argv)

    overrides = {}
    for item in args.override:
        if '=' not in item:
            sys.stderr.write('override must be key=value: %r\n' % item)
            return 2
        k, raw = item.split('=', 1)
        try:
            overrides[k] = json.loads(raw)
        except ValueError:
            overrides[k] = raw

    try:
        out = run_episode(args.episode_id, args.seed, args.duration, args.out_root,
                          overrides, args.timeout, args.domain_id,
                          gz_seed=args.seed, allow_overwrite=args.allow_overwrite)
    except PhaseAError as exc:
        sys.stderr.write('run_phase_a_episode: error: %s\n' % exc)
        return 2

    print(EI.format_smoke_block(out['integrity'], seed=args.seed,
                                requested_duration=args.duration))
    print('\nepisode_dir: %s' % out['episode_dir'])
    return 0 if out['integrity']['episode_integrity'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
