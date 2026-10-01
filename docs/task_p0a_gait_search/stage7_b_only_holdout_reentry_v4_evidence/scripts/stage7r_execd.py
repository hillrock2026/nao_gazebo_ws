#!/usr/bin/env python3
"""Stage 7' — repaired single-attempt B-only execution wrapper (offline prepared).

Runtime-failure repair (PHASE 2):
  * per-attempt, single-consumption authorization with strict reason matching;
  * environment preflight (overlay/package readiness) BEFORE launch — this is the
    direct fix for the 909101 failure (`nao_gz_sim` package not found because the
    workspace overlay was not sourced);
  * known runtime/analysis exceptions are converted into a structured fatal
    failure record (never a bare traceback, never a fabricated success);
  * trace-readiness is checked explicitly; success path writes metrics.json and
    gates.json (the orchestrator reads gates.json).

This build refuses to launch Gazebo/ROS in the current (offline repair) task: it
returns 3 unless a valid unconsumed authorization exists AND the caller passes the
explicit ``--i-have-authorization`` flag.
"""

import argparse
import hashlib
import json
import os
import sys
import time
import traceback

_HERE = os.path.dirname(os.path.abspath(__file__))
EVID = os.path.dirname(_HERE)
WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(_HERE))))
for _p in (os.path.join(WS, 'tools'), _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import run_p0a_gait_candidate as R  # noqa: E402
import stage7r_authorization as AUTH  # noqa: E402
import stage7r_failure_handling as FH  # noqa: E402
import stage7r_env as ENV  # noqa: E402

CANDIDATE_ID = 'stage7rb'
KNOWN_ANALYSIS_ERRORS = (FileNotFoundError, OSError)


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


def write_sums(root):
    # R2-05: never hash interpreter caches / temp files (F-02 recurrence guard).
    exclude_dirs = {'__pycache__'}
    exclude_suffix = ('.pyc', '.pyo', '.tmp')
    lines = []
    for base, dirs, files in os.walk(root):
        dirs[:] = sorted(d for d in dirs if d not in exclude_dirs)
        for name in sorted(files):
            if name == 'SHA256SUMS' or name.endswith(exclude_suffix):
                continue
            full = os.path.join(base, name)
            lines.append('%s  %s' % (sha256_file(full), os.path.relpath(full, root)))
    with open(os.path.join(root, 'SHA256SUMS'), 'w', encoding='utf-8') as fh:
        fh.write('\n'.join(lines) + ('\n' if lines else ''))
    return len(lines)


def write_failure(out_root, rec):
    os.makedirs(out_root, exist_ok=True)
    atomic_json(os.path.join(out_root, 'failure_reason.json'), rec)
    atomic_json(os.path.join(out_root, 'run_result.json'), rec)
    atomic_json(os.path.join(out_root, 'residual_process.json'),
                {'residual_processes': rec['residual_processes'],
                 'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z')})
    atomic_json(os.path.join(out_root, 'pgid.json'),
                {'pgid': rec['pgid'], 'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z')})
    n = write_sums(out_root)
    return n


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--attempt-id', type=int, required=True)
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--reason', required=True)
    ap.add_argument('--run-order', type=int, required=True)
    ap.add_argument('--domain-id', type=int, default=210)
    ap.add_argument('--out-root', required=True)
    ap.add_argument('--gates', required=True)
    ap.add_argument('--frozen-config', required=True)
    ap.add_argument('--timeout', type=float, default=240.0)
    ap.add_argument('--i-have-authorization', action='store_true')
    ap.add_argument('--wrapper-change', action='store_true')
    ap.add_argument('--expected-fingerprint-hash', default=None,
                    help='preregistered environment fingerprint hash (fail closed on mismatch)')
    args = ap.parse_args(argv)

    # F3: strict reason match, no initial_episode fallback.
    ok, info = AUTH.authorize_with_reason(args.attempt_id, args.seed, args.reason,
                                          require_wrapper_change=args.wrapper_change)
    if not ok:
        sys.stderr.write("stage7r_execd: authorization rejected for "
                         "attempt=%s seed=%s reason=%s: %s\n"
                         % (args.attempt_id, args.seed, args.reason, info.get('reason')))
        return 3
    if not args.i_have_authorization:
        sys.stderr.write('stage7r_execd: refusing to launch without the explicit '
                         '--i-have-authorization acknowledgement\n')
        return 3
    # R2-01: an authorized launch must be fingerprint-bound (protocol violation otherwise).
    if not args.expected_fingerprint_hash:
        sys.stderr.write('stage7r_execd: authorized launch must be fingerprint-bound '
                         '(--expected-fingerprint-hash required); refusing as protocol '
                         'violation\n')
        return 3

    frozen = json.load(open(args.frozen_config, encoding='utf-8'))
    if frozen.get('expected_config', {}).get('gait_phase_mode') != 'explicit_support_swing':
        raise SystemExit('frozen snapshot is not B explicit_support_swing')
    b = frozen['expected_config']
    out_root = args.out_root if os.path.isabs(args.out_root) else os.path.join(WS, args.out_root)
    if os.path.exists(out_root):
        raise SystemExit('out_root exists (never reuse): %s' % out_root)

    # --- v4 PHASE 1: unified bootstrap; preflight env == launch env (F-01) ---
    env_context = None
    try:
        boot_env = ENV.bootstrap_environment(WS)
        env_checks = ENV.verify_environment(WS, boot_env)
    except ENV.EnvironmentError2 as exc:
        reason = str(exc).split(':', 1)[0]
        rec = FH.build_failure_record(
            args.attempt_id, args.seed, 'explicit_support_swing',
            b.get('step_frequency_hz'), b.get('sway_sign'),
            'environment_failure', 'bootstrap_%s' % reason,
            launch_started=False, launch_cmd=None, gz_sim_cmd_expected=None,
            pgid=None, residual_processes=FH.residual_scan(),
            trace_info={'valid': False})
        rec['environment_preflight'] = {'ok': False, 'reason': str(exc)}
        n = write_failure(out_root, rec)
        print(json.dumps({'attempt_id': args.attempt_id, 'seed': args.seed, 'ok': False,
                          'attempt_type': 'environment_failure',
                          'failure_reason': rec['failure_reason'], 'launch_started': False,
                          'n_files_hashed': n}, indent=2))
        return 4
    fingerprint = ENV.environment_fingerprint(boot_env)
    fp_hash = ENV.fingerprint_hash(fingerprint)
    if args.expected_fingerprint_hash and fp_hash != args.expected_fingerprint_hash:
        rec = FH.build_failure_record(
            args.attempt_id, args.seed, 'explicit_support_swing',
            b.get('step_frequency_hz'), b.get('sway_sign'),
            'environment_failure', 'environment_fingerprint_mismatch',
            launch_started=False, launch_cmd=None, gz_sim_cmd_expected=None,
            pgid=None, residual_processes=FH.residual_scan(),
            trace_info={'valid': False})
        rec['environment_preflight'] = {'ok': False, 'reason': 'fingerprint_mismatch',
                                        'expected': args.expected_fingerprint_hash,
                                        'actual': fp_hash}
        n = write_failure(out_root, rec)
        print(json.dumps({'attempt_id': args.attempt_id, 'seed': args.seed, 'ok': False,
                          'attempt_type': 'environment_failure',
                          'failure_reason': rec['failure_reason'], 'n_files_hashed': n},
                         indent=2))
        return 4
    env_context = {'fingerprint': fingerprint, 'fingerprint_hash': fp_hash,
                   'checks': env_checks, 'source': 'stage7r_env.bootstrap_environment'}
    # Install the SAME bootstrapped env for the delegated launch (F-01 fix).
    os.environ.clear()
    os.environ.update(boot_env)

    # Authorization consumed only immediately before launch.
    AUTH.consume_attempt(args.attempt_id, args.seed)
    try:
        result = R.run_candidate(CANDIDATE_ID, args.seed, out_root, dict(frozen['overrides']),
                                 args.timeout, args.domain_id, allow_overwrite=False,
                                 gz_seed=args.seed, p1r_gates_path=args.gates)
    except Exception as exc:  # noqa: BLE001 - classified explicitly below, never success
        cls = FH.classify_exception(exc)
        trace_dir = os.path.join(out_root, 'trace')
        rec = FH.build_failure_record(
            args.attempt_id, args.seed, 'explicit_support_swing',
            b.get('step_frequency_hz'), b.get('sway_sign'),
            cls['attempt_type'], cls['failure_reason'], launch_started=True,
            launch_cmd=_read_first_line(os.path.join(out_root, 'launch_cmd.txt')),
            gz_sim_cmd_expected=_read_first_line(os.path.join(out_root, 'gz_sim_cmd_expected.txt')),
            pgid=None, residual_processes=FH.residual_scan(),
            trace_info=FH.trace_readiness(trace_dir),
            analysis_exception_type=cls['analysis_exception_type'],
            analysis_exception_message=cls['analysis_exception_message'])
        rec['traceback_summary'] = traceback.format_exc()[-2000:]
        rec['pgid_unavailable_reason'] = ('runner/analysis exception before PGID capture '
                                          '(delegate did not return a PGID)')
        rec['environment'] = env_context
        n = write_failure(out_root, rec)
        print(json.dumps({'attempt_id': args.attempt_id, 'seed': args.seed, 'ok': False,
                          'attempt_type': cls['attempt_type'],
                          'analysis_exception_type': cls['analysis_exception_type'],
                          'n_files_hashed': n}, indent=2))
        return 4

    prov = _read_json(os.path.join(out_root, 'provenance.json'))
    trace_dir = os.path.join(out_root, 'trace')
    tinfo = FH.trace_readiness(trace_dir)
    launch = result.get('launch_cmd') or prov.get('launch_cmd') \
        or _read_first_line(os.path.join(out_root, 'launch_cmd.txt'))
    gz = result.get('gz_sim_cmd_expected') or prov.get('gz_sim_cmd_expected') \
        or _read_first_line(os.path.join(out_root, 'gz_sim_cmd_expected.txt'))

    if result.get('ok') is not True or not launch or not gz or not tinfo['valid']:
        if not tinfo['valid']:
            atype, freason = 'environment_failure', 'no_state_trace_after_launch'
        else:
            atype, freason = 'integrity_failure', 'analysis_incomplete_or_missing_command'
        rec = FH.build_failure_record(
            args.attempt_id, args.seed, 'explicit_support_swing',
            b.get('step_frequency_hz'), b.get('sway_sign'), atype, freason,
            launch_started=True, launch_cmd=launch, gz_sim_cmd_expected=gz,
            pgid=result.get('pgid'), residual_processes=result.get('residual_processes'),
            trace_info=tinfo, analysis_exception_type=None,
            analysis_exception_message=result.get('p1r_error') or result.get('reason'))
        rec['runner_reason'] = result.get('reason')
        if rec.get('pgid') is None:
            rec['pgid_unavailable_reason'] = ('delegate did not return a PGID on the '
                                              'failure path')
        rec['environment'] = env_context
        n = write_failure(out_root, rec)
        print(json.dumps({'attempt_id': args.attempt_id, 'seed': args.seed, 'ok': False,
                          'attempt_type': atype, 'failure_reason': freason,
                          'n_files_hashed': n}, indent=2))
        return 4

    # Success path: persist metrics/gates (never fabricated) + structured result.
    atomic_json(os.path.join(out_root, 'metrics.json'), result.get('p1r_analysis'))
    atomic_json(os.path.join(out_root, 'gates.json'), result.get('p1r_gates'))
    atomic_json(os.path.join(out_root, 'residual_process.json'),
                {'residual_processes': result.get('residual_processes'),
                 'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z')})
    atomic_json(os.path.join(out_root, 'pgid.json'),
                {'pgid': result.get('pgid'),
                 'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z')})
    rr = {'stage': "7'", 'attempt_id': args.attempt_id, 'seed': args.seed,
          'reason': args.reason, 'phase_mode': 'explicit_support_swing',
          'ok': True, 'exit_reason': result.get('exit_reason'),
          'configured_physics_seed': args.seed, 'launch_cmd': launch,
          'gz_sim_cmd_expected': gz, 'pgid': result.get('pgid'),
          'residual_processes': result.get('residual_processes'),
          'episode_verdict': 'VALID', 'hard_gate_verdict':
          ('PASS' if (result.get('p1r_gates') or {}).get('pass') else 'FAIL'),
          'trace_valid': True,
          'gate_pass': (result.get('p1r_gates') or {}).get('pass'),
          'gate_fails': [k for k, v in ((result.get('p1r_gates') or {}).get('checks') or {}).items()
                         if not v],
          'environment': env_context}
    atomic_json(os.path.join(out_root, 'run_result.json'), rr)
    n = write_sums(out_root)
    print(json.dumps({'attempt_id': args.attempt_id, 'seed': args.seed, 'ok': True,
                      'gate_pass': rr['gate_pass'], 'n_files_hashed': n}, indent=2))
    return 0


def _read_json(p):
    try:
        return json.load(open(p, encoding='utf-8'))
    except (OSError, ValueError):
        return {}


def _read_first_line(p):
    try:
        with open(p, encoding='utf-8') as fh:
            return fh.readline().strip()
    except OSError:
        return None


def self_test():
    """Sandbox + monkeypatch tests of the fingerprint binding invariant (R2-01/R2-02)."""
    import tempfile
    checks = {}
    calls = {'consume': 0, 'run': 0}
    saved_env = dict(os.environ)
    orig = (ENV.bootstrap_environment, ENV.verify_environment, ENV.environment_fingerprint,
            ENV.fingerprint_hash, AUTH.authorize_with_reason, AUTH.consume_attempt,
            R.run_candidate)
    try:
        ENV.bootstrap_environment = lambda *a, **k: {'PATH': '/usr/bin',
                                                     'AMENT_PREFIX_PATH': '/ws/install'}
        ENV.verify_environment = lambda *a, **k: {'ok': True}
        ENV.environment_fingerprint = lambda env: {'PATH': '/usr/bin',
                                                   'python_executable': '/usr/bin/python3'}
        ENV.fingerprint_hash = lambda fp: 'GOOD'
        AUTH.authorize_with_reason = lambda *a, **k: (True, {'reason': 'initial_episode'})

        def _consume(*a, **k):
            calls['consume'] += 1
        AUTH.consume_attempt = _consume

        def _run(*a, **k):
            calls['run'] += 1
            raise FileNotFoundError('mock runner not executed')
        R.run_candidate = _run

        base = tempfile.mkdtemp(prefix='stage7r_execd_')
        frozen = os.path.join(base, 'frozen.json')
        json.dump({'expected_config': {'gait_phase_mode': 'explicit_support_swing',
                                       'step_frequency_hz': 0.8, 'sway_sign': -1},
                   'overrides': {'gait_phase_mode': 'explicit_support_swing',
                                 'step_frequency_hz': 0.8, 'sway_sign': -1}},
                  open(frozen, 'w'))
        common = ['--attempt-id', '1', '--seed', '909301', '--reason', 'initial_episode',
                  '--run-order', '1', '--domain-id', '210', '--gates', os.path.join(base, 'gates.json'),
                  '--frozen-config', frozen]

        # A) mismatch -> return 4, authorization not consumed, runner not called
        out_a = os.path.join(base, 'out_a')
        rc = main(common + ['--out-root', out_a, '--i-have-authorization',
                            '--expected-fingerprint-hash', 'BAD'])
        rec = _read_json(os.path.join(out_a, 'failure_reason.json'))
        checks['mismatch_return4'] = rc == 4
        checks['mismatch_reason'] = rec.get('failure_reason') == 'environment_fingerprint_mismatch'
        checks['mismatch_launch_not_started'] = rec.get('launch_started') is False
        checks['mismatch_authorization_not_consumed'] = calls['consume'] == 0
        checks['mismatch_runner_not_called'] = calls['run'] == 0

        # B) authorized without fingerprint -> return 3, nothing consumed
        out_b = os.path.join(base, 'out_b')
        rc = main(common + ['--out-root', out_b, '--i-have-authorization'])
        checks['authorized_without_hash_return3'] = (rc == 3 and calls['consume'] == 0
                                                     and calls['run'] == 0)

        # C) match -> passes binding, consumes once, then runner exception recorded
        out_c = os.path.join(base, 'out_c')
        rc = main(common + ['--out-root', out_c, '--i-have-authorization',
                            '--expected-fingerprint-hash', 'GOOD'])
        checks['match_consumes_once'] = (rc == 4 and calls['consume'] == 1 and calls['run'] == 1)
        os.environ.clear()
        os.environ.update(saved_env)
    finally:
        (ENV.bootstrap_environment, ENV.verify_environment, ENV.environment_fingerprint,
         ENV.fingerprint_hash, AUTH.authorize_with_reason, AUTH.consume_attempt,
         R.run_candidate) = orig
        os.environ.clear()
        os.environ.update(saved_env)
    return checks


if __name__ == '__main__':
    if '--self-test' in sys.argv:
        st = self_test()
        print(json.dumps({'self_test': st, 'n_pass': sum(1 for v in st.values() if v),
                          'n_total': len(st)}, indent=2, sort_keys=True))
        sys.exit(0 if all(st.values()) else 1)
    sys.exit(main())
