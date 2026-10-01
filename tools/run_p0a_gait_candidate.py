#!/usr/bin/env python3
"""P0-a single-candidate gait experiment runner (real headless Gazebo + ROS).

Runs ONE gait candidate for ONE replicate seed and writes the raw, read-only
Phase 4B trace plus a measured-gait analysis produced by
``tools/analyze_measured_gait.py`` (measured body pose only; never cmd_vel).

This is a *development* runner for the P0-a forward-gait search. It does NOT
touch the four historical datasets, the Phase 2E/2F artifacts or batch-0. All
output goes under the caller's ``--out-root``.

Usage:
  python3 tools/run_p0a_gait_candidate.py \
      --candidate-id cand_01 --seed 101 \
      --out-root docs/task_p0a_gait_search/work/screen/cand_01_seed101 \
      --override step_frequency_hz=1.2 \
      --override com_forward_shift_m=0.02
"""

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import p0a_phase_analysis as PA  # noqa: E402

WS = os.path.dirname(_TOOLS)
ANCHOR = os.path.join(WS, 'docs', 'task4_phase4e_pathb_b3_evidence', 'b3_0',
                      'candidates.json')
# Frozen P1R evidence tree: never deleted or written by the runner.
FROZEN_EVIDENCE_ROOT = os.path.join(WS, 'docs', 'task_p0a_gait_search',
                                    'p1r_support_swing')

# Gait durations for P0-a experiments: full nominal steady window coverage.
EXPERIMENT_DEFAULTS = {
    'duration_sec': 16.0,
    'stop_ramp_sec': 1.5,
    'hold_final_sec': 1.0,
}
# record_duration_sec (relative to first /joint_states) must exceed
# start_delay + duration; leave 1 s margin so the nominal window is complete.
RECORD_MARGIN_SEC = 1.0

LAUNCH_BASE = ['ros2', 'launch', 'nao_gz_sim', 'nao_sim.launch.py',
               'gui:=false', 'use_rsp:=true',
               'enable_phase4a:=true', 'phase4a_run_probe:=false',
               'phase4a_run_trace:=false', 'enable_phase4b:=true',
               'phase4b_run_gait:=true', 'phase4b_run_trace:=true']


class RunnerError(Exception):
    pass


def load_p1r_gates(path):
    """Canonical frozen-gate loader (wrapper or flat); raises GateContractError."""
    return PA.load_gates(path)


def load_anchor():
    with open(ANCHOR, 'r', encoding='utf-8') as fh:
        data = json.load(fh)
    anchors = [c for c in data.get('candidates', []) if c.get('kind') == 'anchor']
    if len(anchors) != 1:
        raise RunnerError('expected 1 anchor, got %d' % len(anchors))
    return dict(anchors[0]['config'])


def parse_overrides(pairs):
    out = {}
    for item in pairs:
        if '=' not in item:
            raise RunnerError('override must be key=value: %r' % item)
        key, raw = item.split('=', 1)
        try:
            value = json.loads(raw)
        except ValueError:
            value = raw
        out[key] = value
    return out


def param_args(cfg):
    args = []
    for key in sorted(cfg):
        value = cfg[key]
        if isinstance(value, bool):
            value = 'true' if value else 'false'
        name = key if key.startswith('actuator_') else 'phase4b_' + key
        args.append('%s:=%s' % (name, value))
    return args


def _load_jsonl(path):
    rows = []
    if not os.path.isfile(path):
        return rows
    with open(path, 'r', encoding='utf-8') as fh:
        for line in fh:
            if line.strip():
                rows.append(json.loads(line))
    return rows


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


def _reap_group(pgid, timeout=25):
    try:
        os.killpg(pgid, signal.SIGINT)
    except ProcessLookupError:
        return _residual_count()
    for _ in range(5):
        time.sleep(1)
        if not _group_alive(pgid):
            return _residual_count()
    try:
        os.killpg(pgid, signal.SIGTERM)
    except ProcessLookupError:
        return _residual_count()
    for _ in range(timeout):
        time.sleep(1)
        if not _group_alive(pgid):
            return _residual_count()
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    time.sleep(1)
    return _residual_count()


def partition_id(candidate_id, replicate_id):
    """Communication-isolation partition id (NOT a physics seed)."""
    return 'p0a_%s_%s' % (candidate_id, replicate_id)


def seed_metadata(candidate_id, replicate_id, gz_seed):
    """Honest seed semantics recorded with every run."""
    return {
        'replicate_id': replicate_id,
        'physics_seed': gz_seed,
        'physics_seed_controlled': gz_seed is not None,
        'communication_partition_id': partition_id(candidate_id, replicate_id),
        'seed_semantics': ('replicate label; physics RNG seed forwarded to '
                           'gz sim --seed' if gz_seed is not None
                           else 'replicate label only; no physics seed wired'),
    }


def build_launch_command(cfg, trace_dir, record, gz_seed=None):
    """Pure builder for the ``ros2 launch`` argv (unit-testable, no side fx)."""
    cmd = list(LAUNCH_BASE)
    cmd += param_args(cfg)
    cmd += ['phase4b_evidence_dir:=%s' % trace_dir,
            'phase4b_record_duration:=%s' % record]
    if gz_seed is not None:
        # Real physics RNG seed, forwarded to gz sim --seed (not a partition /
        # domain id). Persisted verbatim via launch_cmd.txt.
        cmd += ['gz_seed:=%d' % int(gz_seed)]
    return cmd


def build_gz_sim_command(gz_seed, world_path=None,
                         engine='gz-physics-dartsim-plugin'):
    """Expected final ``gz sim`` argv mirrored from the launch OpaqueFunction.

    Saved per run so the *engine-level* command (not just ``ros2 launch``) is
    auditable. It is a deterministic reconstruction of `_gz_sim_actions`.
    """
    gz = shutil.which('gz') or 'gz'
    if world_path is None:
        world_path = os.path.join(WS, 'install', 'nao_gz_sim', 'share',
                                  'nao_gz_sim', 'worlds', 'nao_world.sdf')
    cmd = [gz, 'sim', '-v', '4', '-r', '-s']
    if gz_seed is not None:
        cmd += ['--seed', str(int(gz_seed))]
    cmd += ['--physics-engine', engine, world_path]
    return cmd


def prepare_out_root(out_root, allow_overwrite=False):
    """Guard, then create the run output root.

    Refuses any ``out_root`` whose realpath is the frozen P1R evidence tree or
    lies inside it (``realpath`` resolves symlinks first, so symlink escapes are
    covered).  Outside that tree the existing behaviour is unchanged: a new root
    is created, and an existing root is removed only with ``allow_overwrite``.
    """
    real_out = os.path.realpath(out_root)
    frozen = os.path.realpath(FROZEN_EVIDENCE_ROOT)
    if real_out == frozen or real_out.startswith(frozen + os.sep):
        raise RunnerError(
            'refusing to create/delete an out_root inside the frozen P1R '
            'evidence tree (%s): %s' % (frozen, out_root))
    if os.path.isdir(out_root):
        if not allow_overwrite:
            raise RunnerError('out_root exists (use --allow-overwrite): %s'
                              % out_root)
        shutil.rmtree(out_root)
    trace_dir = os.path.join(out_root, 'trace')
    os.makedirs(trace_dir)
    return trace_dir


def resolve_config(anchor, overrides):
    """Resolve the effective config with explicit precedence (Phase A §9).

    Lowest -> highest priority: ``anchor`` < ``EXPERIMENT_DEFAULTS`` <
    ``overrides``. Applying ``EXPERIMENT_DEFAULTS`` before ``overrides`` means a
    caller's explicit value (``duration_sec``, ``stop_ramp_sec``,
    ``hold_final_sec``, gait parameters, ...) always wins; the earlier order
    silently clobbered user overrides with the defaults.
    """
    cfg = dict(anchor)
    cfg.update(EXPERIMENT_DEFAULTS)
    cfg.update(overrides)
    return cfg


def run_candidate(candidate_id, seed, out_root, overrides, timeout,
                  domain_id, allow_overwrite=False, gz_seed=None,
                  p1r_gates_path=None):
    trace_dir = prepare_out_root(out_root, allow_overwrite)

    # Precedence (lowest -> highest): anchor config < EXPERIMENT_DEFAULTS <
    # user overrides. Phase A §9 fix (see ``resolve_config``).
    cfg = resolve_config(load_anchor(), overrides)
    record = (cfg['start_delay_sec'] + cfg['duration_sec']
              + RECORD_MARGIN_SEC)

    cmd = build_launch_command(cfg, trace_dir, record, gz_seed)
    gz_cmd = build_gz_sim_command(gz_seed)

    # strict JSON: no NaN/Inf may enter a config/evidence file
    with open(os.path.join(out_root, 'run_config.json'), 'w',
              encoding='utf-8') as fh:
        json.dump(cfg, fh, indent=2, sort_keys=True, allow_nan=False)
    with open(os.path.join(out_root, 'launch_cmd.txt'), 'w',
              encoding='utf-8') as fh:
        fh.write(' '.join(cmd) + '\n')
    with open(os.path.join(out_root, 'gz_sim_cmd_expected.txt'), 'w',
              encoding='utf-8') as fh:
        fh.write(' '.join(gz_cmd) + '\n')

    env = dict(os.environ)
    # The ROS 2 (Jazzy) Python entry points must use the system Python 3.12;
    # an active conda Python on PATH would win ``/usr/bin/env python3`` and break
    # rclpy. Drop conda entries from the child PATH.
    env['PATH'] = os.pathsep.join(
        entry for entry in os.environ.get('PATH', '').split(os.pathsep)
        if 'anaconda' not in entry and 'condabin' not in entry)
    env.pop('CONDA_PREFIX', None)
    env.pop('CONDA_DEFAULT_ENV', None)
    env['GZ_PARTITION'] = partition_id(candidate_id, seed)
    env['ROS_DOMAIN_ID'] = str(domain_id)
    launch_log = os.path.join(out_root, 'launch.log')
    summary = os.path.join(trace_dir, 'phase4b_summary.json')
    start_wall = time.strftime('%Y-%m-%dT%H:%M:%S%z')
    started = time.time()
    with open(launch_log, 'w', encoding='utf-8') as log:
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                env=env, start_new_session=True)
    pgid = os.getpgid(proc.pid)
    timed_out = True
    while time.time() - started < timeout:
        if os.path.isfile(summary) and os.path.getsize(summary) > 0:
            timed_out = False
            break
        if proc.poll() is not None:
            break
        time.sleep(1)
    residual = _reap_group(pgid)
    end_wall = time.strftime('%Y-%m-%dT%H:%M:%S%z')

    states = _load_jsonl(os.path.join(trace_dir, 'state_trace.jsonl'))
    ready = states[0]['t'] if states else None
    provenance = {
        'candidate_id': candidate_id, 'replicate_id': seed,
        'physics_seed': gz_seed, 'ros_domain_id': domain_id,
        'gz_partition': partition_id(candidate_id, seed),
        'python': sys.version, 'ros_distro': os.environ.get('ROS_DISTRO'),
        'start_wall': start_wall, 'end_wall': end_wall,
        'overrides': overrides, 'timeout_sec': timeout,
        'launch_cmd': ' '.join(cmd), 'gz_sim_cmd_expected': ' '.join(gz_cmd),
    }
    with open(os.path.join(out_root, 'provenance.json'), 'w',
              encoding='utf-8') as fh:
        json.dump(provenance, fh, indent=2, sort_keys=True, allow_nan=False)

    result = {
        'candidate_id': candidate_id,
        'seed': seed,
        'run_dir': out_root,
        'timed_out': timed_out,
        'exit_reason': 'summary_written' if not timed_out else 'launch_timeout',
        'pgid': pgid,
        'residual_processes': residual,
        'start_wall': start_wall,
        'end_wall': end_wall,
        'wall_sec': time.time() - started,
        'ready_sim': ready,
        **seed_metadata(candidate_id, seed, gz_seed),
        'run_config': cfg,
        'phase4b_summary': (json.load(open(summary, 'r', encoding='utf-8'))
                            if os.path.isfile(summary) else None),
    }
    if not states:
        result['ok'] = False
        result['reason'] = 'launch_timeout' if timed_out else 'no_state_trace'
    else:
        with open(os.path.join(out_root, 'result.json'), 'w',
                  encoding='utf-8') as fh:
            json.dump({'labels': {'ready_sim': ready},
                       'config_id': candidate_id, 'seed': seed}, fh,
                      indent=2, sort_keys=True, allow_nan=False)
        import analyze_measured_gait as M
        try:
            result['measured'] = M.compute_run(out_root, 'p0a_%s' % candidate_id)
            result['ok'] = True
        except M.MeasuredGaitError as exc:
            # Structured failure; never a fabricated success.
            result['measured'] = None
            result['ok'] = False
            result['reason'] = 'measured_failed:%s' % type(exc).__name__
            result['measured_error'] = str(exc)
    if p1r_gates_path:
        # Canonical loader: accepts the v2 wrapper (hard_gates nested) and the
        # legacy flat form, and fails explicitly on any contract violation.
        # Stage 7' runtime-failure repair: a missing trace must not surface as a
        # bare analysis exception escaping run_candidate; record it structurally.
        if not states:
            result['p1r_analysis'] = None
            result['p1r_gates'] = None
            result['p1r_error'] = 'analysis_skipped_no_state_trace'
        else:
            import analyze_measured_gait as M
            try:
                gates = PA.load_gates(p1r_gates_path)
                analysis = PA.analyse(out_root)
                result['p1r_analysis'] = analysis
                result['p1r_gates'] = PA.evaluate_gates(analysis, gates)
            except (M.MeasuredGaitError, PA.GateContractError,
                    FileNotFoundError, OSError) as exc:
                result['ok'] = False
                result['reason'] = 'analysis_failed:%s' % type(exc).__name__
                result['p1r_analysis'] = None
                result['p1r_gates'] = None
                result['p1r_error'] = '%s: %s' % (type(exc).__name__, exc)
    with open(os.path.join(out_root, 'result_p0a.json'), 'w',
              encoding='utf-8') as fh:
        json.dump(result, fh, indent=2, sort_keys=True, allow_nan=False)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description='P0-a candidate runner')
    parser.add_argument('--candidate-id', default=None)
    parser.add_argument('--seed', type=int, default=None)
    parser.add_argument('--out-root', default=None)
    parser.add_argument('--override', action='append', default=[],
                        help='repeatable key=value gait/actuator override')
    parser.add_argument('--timeout', type=float, default=150.0)
    parser.add_argument('--domain-id', type=int, default=77)
    parser.add_argument('--gz-seed', type=int, default=None,
                        help='real physics seed forwarded to gz sim --seed '
                             '(default: none, same as historical)')
    parser.add_argument('--p1r-gates', default=None,
                        help='frozen_gates_v2.json; if given, the run also stores '
                             'the P1-R phase analysis and gate verdict')
    parser.add_argument('--allow-overwrite', action='store_true')
    parser.add_argument('--validate-gates', nargs='?', const='', default=None,
                        metavar='GATE_FILE',
                        help='validate the given gate file (or --p1r-gates) and '
                             'exit without Gazebo; exit 2 on any contract '
                             'violation')
    args = parser.parse_args(argv)

    if args.validate_gates is not None:
        gate_file = args.validate_gates or args.p1r_gates
        if not gate_file:
            sys.stderr.write('run_p0a_gait_candidate: --validate-gates needs a '
                             'gate file (value or --p1r-gates)\n')
            return 2
        try:
            gates = load_p1r_gates(gate_file)
        except PA.GateContractError as exc:
            sys.stderr.write('run_p0a_gait_candidate: gate contract error: %s\n'
                             % exc)
            return 2
        print(json.dumps({'validated': True, 'gate_file': gate_file,
                          'gate_form': gates.get('_gate_form'),
                          'thresholds': {k: gates[k] for k in
                                         PA.REQUIRED_HARD_GATES},
                          'epsilon_sec': gates.get('epsilon_sec')},
                         indent=2, sort_keys=True))
        return 0

    missing = [name for name in ('candidate_id', 'seed', 'out_root')
               if getattr(args, name) is None]
    if missing:
        sys.stderr.write('run_p0a_gait_candidate: missing required '
                         'arguments: %s\n'
                         % ', '.join('--' + m.replace('_', '-') for m in missing))
        return 2

    # Fail fast on an invalid gate file: never launch Gazebo with a bad contract.
    if args.p1r_gates:
        try:
            load_p1r_gates(args.p1r_gates)
        except PA.GateContractError as exc:
            sys.stderr.write('run_p0a_gait_candidate: gate contract error: %s\n'
                             % exc)
            return 2

    if not shutil.which('ros2'):
        sys.stderr.write('run_p0a_gait_candidate: ros2 not on PATH; '
                         'source ROS before running\n')
        return 2
    overrides = parse_overrides(args.override)
    try:
        result = run_candidate(args.candidate_id, args.seed, args.out_root,
                               overrides, args.timeout, args.domain_id,
                               args.allow_overwrite, gz_seed=args.gz_seed,
                               p1r_gates_path=args.p1r_gates)
    except RunnerError as exc:
        sys.stderr.write('run_p0a_gait_candidate: error: %s\n' % exc)
        return 2
    measured = result.get('measured') or {}
    print(json.dumps({
        'ok': result.get('ok'),
        'reason': result.get('reason'),
        'seed': result['seed'],
        'physics_seed': result.get('physics_seed'),
        'residual_processes': result.get('residual_processes'),
        'p1r_gates_pass': (result.get('p1r_gates') or {}).get('pass'),
        'p1r_gate_fails': [k for k, v in
                           ((result.get('p1r_gates') or {}).get('checks') or {}).items()
                           if not v],
        'measured': {k: measured.get(k) for k in (
            'measurable', 'reason', 'forward_displacement_m',
            'lateral_displacement_m', 'net_displacement_m',
            'actual_speed_mean_mps', 'actual_speed_std_mps', 'yaw_change_deg',
            'torso_tilt_max_deg', 'fall', 'trace_truncated',
            'effective_duration_sec', 'path_over_net', 'max_single_step_m')},
    }, indent=2, sort_keys=True))
    return 0 if result.get('ok') else 1


if __name__ == '__main__':
    sys.exit(main())
