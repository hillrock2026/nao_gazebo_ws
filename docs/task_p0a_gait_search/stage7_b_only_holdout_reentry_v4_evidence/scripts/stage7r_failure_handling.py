#!/usr/bin/env python3
"""Stage 7' — runtime failure handling (PHASE 2 repair, offline).

Pure, testable helpers used by ``stage7r_execd.py``:

  * ``environment_ready``  — preflight overlay/package readiness (prevents the
    exact Stage 7' 909101 failure: unsourced workspace overlay => package not found);
  * ``trace_readiness``    — required-trace presence/readiness check before any
    analysis, so a missing trace yields a structured fatal failure rather than an
    unauditable bare exception;
  * ``classify_exception`` — map known runtime/analysis exceptions to a frozen
    attempt type + failure reason;
  * ``residual_scan``      — explicit residual-process scan (never silently zero).

Never starts Gazebo/ROS.
"""

import os
import shutil
import subprocess

REQUIRED_TRACE_FILES = ['state_trace.jsonl']
OPTIONAL_TRACE_FILES = ['body_pose_trace.jsonl', 'contact_trace.jsonl',
                        'imu_trace.jsonl', 'clock_trace.jsonl',
                        'command_trace.jsonl', 'phase4b_summary.json']

# Known environment/analysis exception class names (no broad swallow).
ENV_EXCEPTIONS = ('MeasuredGaitError',)
INTEGRITY_EXCEPTIONS = ('GateContractError', 'PhaseAnalysisError',
                        'FileNotFoundError', 'PermissionError', 'IsADirectoryError')


def environment_ready(ros2=None, ament_prefix=None):
    """Return (ok, reason).  ok=False means do NOT launch."""
    ros2 = ros2 if ros2 is not None else shutil.which('ros2')
    if not ros2:
        return False, 'ros2_not_on_path'
    ament = ament_prefix if ament_prefix is not None else os.environ.get('AMENT_PREFIX_PATH', '')
    if 'install' not in ament.replace('\\', '/'):
        return False, 'workspace_overlay_not_sourced'
    try:
        p = subprocess.run(['ros2', 'pkg', 'prefix', 'nao_gz_sim'],
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           text=True, timeout=30)
    except (OSError, subprocess.SubprocessError):
        return False, 'ros2_pkg_probe_failed'
    if p.returncode != 0:
        return False, 'nao_gz_sim_package_not_found'
    if not (p.stdout or '').strip():
        return False, 'nao_gz_sim_prefix_empty'
    return True, None


def trace_readiness(trace_dir):
    info = {'trace_dir': trace_dir, 'exists': os.path.isdir(trace_dir),
            'files': [], 'missing': [], 'empty': [], 'readable': False, 'valid': False}
    if not info['exists']:
        return info
    info['files'] = sorted(os.listdir(trace_dir))
    for f in REQUIRED_TRACE_FILES:
        p = os.path.join(trace_dir, f)
        if not os.path.isfile(p):
            info['missing'].append(f)
        elif os.path.getsize(p) == 0:
            info['empty'].append(f)
    if not info['missing'] and not info['empty']:
        p = os.path.join(trace_dir, REQUIRED_TRACE_FILES[0])
        try:
            with open(p, 'r', encoding='utf-8', errors='ignore') as fh:
                info['readable'] = bool(fh.readline().strip())
        except OSError:
            info['readable'] = False
        info['monotonic'] = _advisory_monotonic(p)
        info['schema_note'] = ('advisory only: schema / time-monotonicity are NOT part of '
                               '.valid; they are enforced by the analysis-layer escalation '
                               'rule (hard FAIL on invalid trace), never converted to success')
    info['valid'] = (info['exists'] and not info['missing']
                     and not info['empty'] and info['readable'])
    return info


def _advisory_monotonic(path, max_lines=20000):
    """Advisory non-decreasing check on the 't' field; True/False/None."""
    import json as _json
    last = None
    n = 0
    try:
        with open(path, 'r', encoding='utf-8', errors='ignore') as fh:
            for line in fh:
                line = line.strip()
                if not line:
                    continue
                n += 1
                if n > max_lines:
                    break
                try:
                    t = _json.loads(line).get('t')
                except ValueError:
                    return None
                if t is None:
                    return None
                if last is not None and t < last:
                    return False
                last = t
    except OSError:
        return None
    return True if last is not None else None


def classify_exception(exc):
    name = type(exc).__name__
    msg = str(exc)
    if name in ENV_EXCEPTIONS:
        return {'attempt_type': 'environment_failure',
                'failure_reason': 'analysis_failed_no_ready_or_state_trace',
                'analysis_exception_type': name, 'analysis_exception_message': msg}
    if name in INTEGRITY_EXCEPTIONS:
        return {'attempt_type': 'integrity_failure',
                'failure_reason': 'io_or_contract_error:%s' % name,
                'analysis_exception_type': name, 'analysis_exception_message': msg}
    return {'attempt_type': 'integrity_failure',
            'failure_reason': 'unexpected_analysis_exception:%s' % name,
            'analysis_exception_type': name, 'analysis_exception_message': msg}


def residual_scan():
    try:
        out = subprocess.run(
            ['bash', '-lc',
             "ps -eo pid,pgid,cmd | grep -E 'gz sim|nao_sim.launch|"
             "parameter_bridge|ros2 launch' | grep -v grep | wc -l"],
            stdout=subprocess.PIPE, text=True).stdout.strip()
        return int(out or '0')
    except (OSError, ValueError):
        return -1


def build_failure_record(attempt_id, seed, phase_mode, step_frequency_hz, sway_sign,
                         attempt_type, failure_reason, launch_started, launch_cmd,
                         gz_sim_cmd_expected, pgid, residual_processes, trace_info,
                         analysis_exception_type=None, analysis_exception_message=None):
    """Structured failure record; never claims success or a gate verdict."""
    return {
        'artifact': "Stage 7' structured runtime failure",
        'stage': "7'",
        'attempt_id': attempt_id,
        'seed': seed,
        'attempt_type': attempt_type,
        'failure_reason': failure_reason,
        'episode_verdict': 'NOT_VALID',
        'hard_gate_verdict': 'NOT_EVALUATED',
        'trace_valid': bool(trace_info.get('valid')) if trace_info else False,
        'trace_readiness': trace_info,
        'analysis_exception_type': analysis_exception_type,
        'analysis_exception_message': analysis_exception_message,
        'launch_started': bool(launch_started),
        'launch_cmd': launch_cmd,
        'gz_sim_cmd_expected': gz_sim_cmd_expected,
        'configured_physics_seed': seed,
        'configured_phase_mode': phase_mode,
        'configured_step_frequency_hz': step_frequency_hz,
        'configured_sway_sign': sway_sign,
        'pgid': pgid,
        'residual_processes': residual_processes,
        'ok': False,
        'automatic_retry': False,
        'seed_replacement': False,
        'recovery_started': False,
        'hard_gate_fabricated': False,
        'metrics_fabricated': False,
    }


def self_test():
    import tempfile
    checks = {}
    # trace readiness fixtures
    with tempfile.TemporaryDirectory() as d:
        checks['empty_dir_invalid'] = trace_readiness(d)['valid'] is False
        checks['empty_dir_missing_state'] = 'state_trace.jsonl' in trace_readiness(d)['missing']
        td = os.path.join(d, 'partial')
        os.makedirs(td)
        open(os.path.join(td, 'state_trace.jsonl'), 'w').close()
        r = trace_readiness(td)
        checks['empty_file_invalid'] = r['valid'] is False and 'state_trace.jsonl' in r['empty']
        td2 = os.path.join(d, 'valid')
        os.makedirs(td2)
        with open(os.path.join(td2, 'state_trace.jsonl'), 'w') as fh:
            fh.write('{"t": 0.0}\n')
        r2 = trace_readiness(td2)
        checks['valid_trace_ok'] = r2['valid'] is True and r2['readable'] is True
    # exception classification
    class MeasuredGaitError(Exception):
        pass

    class GateContractError(Exception):
        pass
    c1 = classify_exception(MeasuredGaitError('cannot determine ready time'))
    checks['measured_maps_environment'] = c1['attempt_type'] == 'environment_failure'
    checks['measured_reason'] = c1['failure_reason'] == 'analysis_failed_no_ready_or_state_trace'
    c2 = classify_exception(GateContractError('bad gate'))
    checks['gate_maps_integrity'] = c2['attempt_type'] == 'integrity_failure'
    c3 = classify_exception(FileNotFoundError('x'))
    checks['file_maps_integrity'] = c3['attempt_type'] == 'integrity_failure'
    # environment readiness pure checks
    checks['env_missing_ros2'] = environment_ready(ros2=None, ament_prefix='x')[0] is False
    checks['env_no_overlay'] = environment_ready(ros2='/bin/true', ament_prefix='/opt/ros/jazzy')[1] \
        == 'workspace_overlay_not_sourced'
    # residual scan returns int
    checks['residual_scan_int'] = isinstance(residual_scan(), int)
    # failure record shape
    rec = build_failure_record(1, 909101, 'explicit_support_swing', 0.8, -1,
                               'environment_failure', 'analysis_failed_no_ready_or_state_trace',
                               False, 'cmd', 'gzcmd', None, 0, {'valid': False})
    checks['record_not_valid'] = rec['episode_verdict'] == 'NOT_VALID'
    checks['record_no_gate_verdict'] = rec['hard_gate_verdict'] == 'NOT_EVALUATED'
    checks['record_ok_false'] = rec['ok'] is False
    checks['record_no_fabrication'] = (rec['hard_gate_fabricated'] is False
                                       and rec['metrics_fabricated'] is False)
    return checks


if __name__ == '__main__':
    import json
    import sys
    if '--self-test' in sys.argv:
        st = self_test()
        print(json.dumps({'self_test': st, 'n_pass': sum(1 for v in st.values() if v),
                          'n_total': len(st)}, indent=2, sort_keys=True))
    else:
        raise SystemExit('use --self-test')
