#!/usr/bin/env python3
"""Stage 7 — real single-episode B-only execution wrapper (authorized phase).

Runs exactly one headless Gazebo episode for one preregistered physics seed by
wrapping the frozen ``tools/run_p0a_gait_candidate.py`` (never modifying it).
Records the full per-episode evidence contract.  This script only runs when the
recorded authorization is GRANTED.
"""

import argparse
import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
EVID = os.path.dirname(_HERE)
WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(_HERE))))
_TOOLS = os.path.join(WS, 'tools')
for _p in (_TOOLS, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import run_p0a_gait_candidate as R  # noqa: E402

CANDIDATE_ID = 'stage7b'
PROC_START_WALL = time.strftime('%Y-%m-%dT%H:%M:%S%z')


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def atomic_json(path, obj):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        json.dump(obj, fh, indent=2, sort_keys=True, allow_nan=False)
    os.replace(tmp, path)


def authorization_granted():
    p = os.path.join(EVID, 'Stage7_authorization_record.json')
    try:
        rec = json.load(open(p, encoding='utf-8'))
    except (OSError, ValueError):
        return False
    return (rec.get('execution_authorization') == 'GRANTED'
            and bool((rec.get('authorization_text') or '').strip()))


def _cpu_model():
    try:
        with open('/proc/cpuinfo', 'r', encoding='utf-8') as fh:
            for line in fh:
                if line.startswith('model name'):
                    return line.split(':', 1)[1].strip()
    except OSError:
        pass
    return None


def _gazebo_version():
    try:
        out = subprocess.run(['gz', 'sim', '--version'], stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True, timeout=20)
        return out.stdout.splitlines()[0] if out.stdout else None
    except (OSError, subprocess.SubprocessError):
        return None


def environment_snapshot():
    keys = ['ROS_DISTRO', 'ROS_VERSION', 'ROS_PYTHON_VERSION', 'ROS_DOMAIN_ID',
            'GZ_PARTITION', 'RMW_IMPLEMENTATION']
    return {
        'captured_at_local': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'proc_start_wall': PROC_START_WALL,
        'host': platform.node(), 'cwd': os.getcwd(), 'workspace_abs': WS,
        'env': {k: os.environ.get(k) for k in keys if os.environ.get(k) is not None},
        'python': {'executable': sys.executable, 'version': sys.version},
        'platform': {'os': platform.platform(), 'machine': platform.machine(),
                     'release': platform.release()},
        'cpu': {'model_name': _cpu_model(), 'n_logical_cpus': os.cpu_count()},
        'gazebo_version': _gazebo_version(),
        'runner_source_sha256': sha256_file(os.path.join(
            _TOOLS, 'run_p0a_gait_candidate.py')),
        'wrapper_source_sha256': sha256_file(os.path.abspath(__file__)),
    }


def write_sums(root):
    lines = []
    for base, dirs, files in os.walk(root):
        dirs.sort()
        for name in sorted(files):
            if name == 'SHA256SUMS':
                continue
            full = os.path.join(base, name)
            lines.append('%s  %s' % (sha256_file(full), os.path.relpath(full, root)))
    with open(os.path.join(root, 'SHA256SUMS'), 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(lines) + ('\n' if lines else ''))
    return len(lines)


def fail(reason, stage, extra=None, out_root=None, code=4):
    rec = {'status': 'FAILED', 'reason': reason, 'stage': stage,
           'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
    if extra:
        rec.update(extra)
    if out_root and os.path.isdir(out_root):
        atomic_json(os.path.join(out_root, 'failure_reason.json'), rec)
        atomic_json(os.path.join(out_root, 'run_result.json'), rec)
    print(json.dumps(rec, indent=2, sort_keys=True))
    return code


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--episode-id', default='B')
    ap.add_argument('--run-order', type=int, required=True)
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--domain-id', type=int, default=210)
    ap.add_argument('--out-root', required=True)
    ap.add_argument('--gates', required=True)
    ap.add_argument('--frozen-config', required=True)
    ap.add_argument('--timeout', type=float, default=240.0)
    args = ap.parse_args(argv)

    if not authorization_granted():
        sys.stderr.write('stage7_exec_runner: authorization NOT_GRANTED; refusing\n')
        return 3

    out_root = args.out_root if os.path.isabs(args.out_root) \
        else os.path.join(WS, args.out_root)
    if os.path.exists(out_root):
        return fail('output directory exists (never reuse/overwrite): %s' % out_root,
                    'output_guard', out_root=out_root, code=3)

    frozen = json.load(open(args.frozen_config, encoding='utf-8'))
    if frozen.get('expected_config', {}).get('gait_phase_mode') != 'explicit_support_swing':
        return fail('frozen snapshot is not B explicit_support_swing', 'config_reproduce',
                    out_root=out_root, code=5)

    cfg = R.load_anchor()
    cfg.update(dict(frozen['overrides']))
    cfg.update(R.EXPERIMENT_DEFAULTS)
    if cfg != frozen['expected_config']:
        return fail('reproduced config != frozen expected_config',
                    'config_freeze_mismatch',
                    {'reproduced': cfg, 'expected': frozen['expected_config']},
                    out_root=out_root, code=5)

    started = time.time()
    try:
        result = R.run_candidate(
            CANDIDATE_ID, args.seed, out_root, dict(frozen['overrides']),
            args.timeout, args.domain_id, allow_overwrite=False,
            gz_seed=args.seed, p1r_gates_path=args.gates)
    except Exception as exc:  # noqa: BLE001
        return fail('runner raised %s: %s' % (type(exc).__name__, exc),
                    'runner_exception', {'episode_id': args.episode_id},
                    out_root=out_root, code=4)

    try:
        written = json.load(open(os.path.join(out_root, 'run_config.json'),
                                 encoding='utf-8'))
    except (OSError, ValueError) as exc:
        return fail('run_config.json unreadable after run: %s' % exc,
                    'config_roundtrip', out_root=out_root, code=5)
    if written != frozen['expected_config']:
        return fail('config round-trip mismatch', 'config_roundtrip',
                    {'written': written}, out_root=out_root, code=5)

    residual = result.get('residual_processes')
    if residual != 0:
        return fail('residual processes after run: %r' % residual,
                    'residual_processes', out_root=out_root, code=4)
    if result.get('ok') is not True:
        return fail('run not ok: %s' % result.get('reason'), 'run_not_ok',
                    {'reason': result.get('reason'),
                     'timed_out': result.get('timed_out')},
                    out_root=out_root, code=4)

    analysis = result.get('p1r_analysis')
    gates = result.get('p1r_gates')
    if not isinstance(analysis, dict) or not isinstance(gates, dict):
        return fail('missing p1r_analysis/p1r_gates', 'analysis_missing',
                    out_root=out_root, code=4)

    atomic_json(os.path.join(out_root, 'metrics.json'), analysis)
    atomic_json(os.path.join(out_root, 'gates.json'), gates)
    atomic_json(os.path.join(out_root, 'environment_snapshot.json'),
                environment_snapshot())
    atomic_json(os.path.join(out_root, 'config_snapshot.json'), frozen)
    atomic_json(os.path.join(out_root, 'pgid.json'), {
        'pgid': result.get('pgid'), 'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z')})
    atomic_json(os.path.join(out_root, 'residual_process.json'), {
        'residual_processes': residual,
        'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z')})
    atomic_json(os.path.join(out_root, 'stop_reason.json'), {
        'stop_reason': 'episode_complete', 'timed_out': result.get('timed_out'),
        'exit_reason': result.get('exit_reason')})
    # stdout/stderr capture (launch.log is the merged process log)
    for name in ('stdout.log', 'stderr.log'):
        p = os.path.join(out_root, name)
        if not os.path.exists(p):
            open(p, 'w').close()

    run_result = {
        'stage': 7, 'episode_id': args.episode_id, 'run_order': args.run_order,
        'mode': 'B_ONLY', 'phase_mode': 'explicit_support_swing', 'seed': args.seed,
        'configured_physics_seed': args.seed, 'wrapper_seed_field': args.seed,
        'repetition': args.seed, 'candidate_id': CANDIDATE_ID,
        'domain_id': args.domain_id,
        'gz_partition': R.partition_id(CANDIDATE_ID, args.seed),
        'ok': True, 'exit_reason': result.get('exit_reason'),
        'run_dir_rel': os.path.relpath(out_root, WS), 'run_dir_abs': out_root,
        'cwd': WS,
        'launch_cmd': result.get('launch_cmd'),
        'gz_sim_cmd_expected': result.get('gz_sim_cmd_expected'),
        'trace_dir': os.path.join(out_root, 'trace'),
        'pgid': result.get('pgid'), 'physics_seed': result.get('physics_seed'),
        'seed_metadata': {k: result.get(k) for k in (
            'replicate_id', 'physics_seed', 'physics_seed_controlled',
            'communication_partition_id', 'seed_semantics')},
        'residual_processes': residual, 'timed_out': result.get('timed_out'),
        'start_wall': result.get('start_wall'), 'end_wall': result.get('end_wall'),
        'wall_sec': result.get('wall_sec'), 'wall_sec_total': time.time() - started,
        'config_roundtrip_ok': True, 'gate_pass': gates.get('pass'),
        'gate_fails': [k for k, v in (gates.get('checks') or {}).items() if not v],
    }
    atomic_json(os.path.join(out_root, 'run_result.json'), run_result)
    n = write_sums(out_root)
    print(json.dumps({
        'episode_id': args.episode_id, 'run_order': args.run_order,
        'seed': args.seed, 'ok': True, 'gate_pass': gates.get('pass'),
        'gate_fails': run_result['gate_fails'], 'residual_processes': residual,
        'n_files_hashed': n, 'run_dir': os.path.relpath(out_root, WS),
        'launch_cmd': run_result['launch_cmd'],
    }, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
