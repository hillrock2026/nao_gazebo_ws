#!/usr/bin/env python3
"""Stage 7'' — strict-serial B-only orchestrator with per-attempt authorization.

Entry-point repair (round 2):
  * F-01: reads ``Stage7r_preregistration.json`` (the actual v2 directory name),
    fail-closed if the preregistration / run order is missing;
  * F-02: ``trace_ok`` is defined fail-closed from ``run_result.trace_valid``;
  * backstop: any internal exception after a failure writes ``Stage7r_stop.json``
    (attempt_type=integrity_failure) instead of crashing with a bare traceback.

Never runs automatically; no retry, no seed replacement, no recovery.  Executing
Gazebo/ROS is delegated to ``stage7r_execd.py`` (which requires a real
per-attempt authorization and explicit acknowledgement).
"""

import json
import os
import subprocess
import sys
import tempfile
import time
import traceback

_HERE = os.path.dirname(os.path.abspath(__file__))
EVID = os.path.dirname(_HERE)
WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(_HERE))))
for _p in (_HERE,):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import stage7r_authorization as AUTH  # noqa: E402
import stage7r_attempt_policy as POLICY  # noqa: E402

RUNNER = os.path.join(_HERE, 'stage7r_execd.py')
DOMAIN_ID = 210
PREREG_NAME = 'Stage7r_preregistration.json'
RUN_ORDER_NAME = 'Stage7r_run_order.json'


def atomic_json(path, obj):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        json.dump(obj, fh, indent=2, sort_keys=True, allow_nan=False)
    os.replace(tmp, path)


def read_json(path):
    try:
        return json.load(open(path, encoding='utf-8'))
    except (OSError, ValueError):
        return None


def classify_failure(rr, gates, returncode):
    """Pure, fail-closed failure classification (used by tests and run)."""
    trace_ok = bool(rr and rr.get('trace_valid'))
    if rr is None:
        return {'attempt_type': 'integrity_failure',
                'failure_reason': 'run_result_missing',
                'episode_verdict': 'NOT_VALID',
                'hard_gate_verdict': 'NOT_EVALUATED',
                'trace_valid': trace_ok}
    if rr.get('ok') is True:
        return {'attempt_type': 'hard_gate_failure',
                'failure_reason': 'hard_gate_failure',
                'episode_verdict': 'VALID',
                'hard_gate_verdict': 'FAIL',
                'trace_valid': bool(rr.get('trace_valid', trace_ok))}
    return {'attempt_type': rr.get('attempt_type', 'integrity_failure'),
            'failure_reason': rr.get('failure_reason', 'unstructured_failure'),
            'episode_verdict': rr.get('episode_verdict', 'NOT_VALID'),
            'hard_gate_verdict': rr.get('hard_gate_verdict', 'NOT_EVALUATED'),
            'trace_valid': bool(rr.get('trace_valid', trace_ok)),
            'analysis_exception_type': rr.get('analysis_exception_type'),
            'analysis_exception_message': rr.get('analysis_exception_message')}


def write_stop(evid, transitions, executed, status='FAIL_STOPPED', attempt_type=None,
               stop_reason=None, seeds=None, failed_seed=None, extra=None):
    stop = {'artifact': "Stage 7' stop record", 'stage': "7''", 'status': status,
            'status_machine': transitions, 'executed_seeds': executed,
            'automatic_retry': False, 'seed_replacement': False}
    if attempt_type:
        stop['attempt_type'] = attempt_type
    if stop_reason:
        stop['stop_reason'] = stop_reason
    if failed_seed is not None:
        stop['failed_seed'] = failed_seed
    if seeds:
        stop['seeds_not_run'] = [s for s in seeds if s not in
                                 [e.get('seed') for e in executed]]
    if extra:
        stop.update(extra)
    atomic_json(os.path.join(evid, 'Stage7r_stop.json'), stop)
    return stop


def run(evid, runner, timeout=240.0, auth_ledger=None):
    auth_ledger = auth_ledger or AUTH.LEDGER
    prereg_path = os.path.join(evid, PREREG_NAME)
    order_path = os.path.join(evid, RUN_ORDER_NAME)
    prereg = read_json(prereg_path)
    order = read_json(order_path)
    if not prereg or not order:
        sys.stderr.write('stage7r_orchestrator: missing prereg/run order '
                         '(expected %s and %s)\n' % (prereg_path, order_path))
        return 2
    seeds = [r['seed'] for r in order['order']]
    if seeds != prereg['preregistered_seeds']:
        sys.stderr.write('stage7r_orchestrator: order != prereg seeds\n')
        return 2
    # R2-01: a preregistration must be environment-fingerprint-bound.  A missing
    # field is a hard fail-close, never silently omitted.
    if not prereg.get('environment_fingerprint_hash'):
        sys.stderr.write('stage7r_orchestrator: preregistration missing '
                         'environment_fingerprint_hash; refusing to plan (fail closed)\n')
        return 2

    events = []
    transitions = ['INIT', 'PRECHECK']
    executed = []

    def emit(o):
        o = dict(o)
        o['time'] = time.strftime('%Y-%m-%dT%H:%M:%S%z')
        events.append(o)
        print(json.dumps(o, sort_keys=True), flush=True)

    def flush():
        atomic_json(os.path.join(evid, 'Stage7r_execution_log.json'),
                    {'artifact': "Stage 7' execution log", 'events': events})
        atomic_json(os.path.join(evid, 'Stage7r_execution_status.json'),
                    {'artifact': "Stage 7' execution status", 'planned_seeds': seeds,
                     'executed_seeds': executed, 'status_machine': transitions,
                     'automatic_retry': False, 'seed_replacement': False,
                     'updated_at': time.strftime('%Y-%m-%dT%H:%M:%S%z')})

    try:
        for i, seed in enumerate(seeds):
            attempt_id = i + 1
            ok, auth_info = AUTH.authorize_attempt(attempt_id, seed, auth_ledger)
            if not ok:
                pv = POLICY.classification('protocol_violation')
                transitions.append('PROTOCOL_VIOLATION_attempt_%d' % attempt_id)
                stop = {'artifact': "Stage 7' stop record", 'stage': "7''",
                        'status': 'STOPPED_PROTOCOL_VIOLATION',
                        'attempt_type': 'protocol_violation',
                        'protocol_violation': 'unauthorized_attempt',
                        'counts_as_episode': pv['counts_as_episode'],
                        'recovery_allowed': pv['recovery_allowed'],
                        'stop_reason': 'unauthorized_attempt',
                        'attempt_id': attempt_id, 'seed': seed,
                        'authorization_reason': auth_info.get('reason'),
                        'status_machine': transitions, 'automatic_retry': False,
                        'seed_replacement': False}
                atomic_json(os.path.join(evid, 'Stage7r_stop.json'), stop)
                flush()
                emit({'event': 'STOP_PROTOCOL_VIOLATION',
                      'protocol_violation': 'unauthorized_attempt',
                      'attempt_id': attempt_id, 'seed': seed,
                      'reason': auth_info.get('reason')})
                return 3
            transitions.append('RUN_B_%d' % seed)
            out_root = os.path.join(evid, 'runs', 'seed-%d' % seed)
            cmd = [sys.executable, runner, '--attempt-id', str(attempt_id), '--seed', str(seed),
                   '--reason', auth_info.get('reason', 'initial_episode'),
                   '--run-order', str(attempt_id), '--domain-id', str(DOMAIN_ID),
                   '--out-root', out_root,
                   '--gates', os.path.join(WS, 'docs/task_p0a_gait_search', 'p1r_support_swing',
                                           'frozen_gates_v2.json'),
                   '--frozen-config', os.path.join(evid, 'Stage7r_B_config_snapshot.json'),
                   '--i-have-authorization', '--timeout', str(timeout)]
            expected_fp = prereg.get('environment_fingerprint_hash')
            if expected_fp:
                cmd += ['--expected-fingerprint-hash', expected_fp]
            emit({'event': 'attempt_start', 'attempt_id': attempt_id, 'seed': seed, 'argv': cmd})
            p = subprocess.run(cmd, cwd=WS, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True)
            emit({'event': 'attempt_end', 'attempt_id': attempt_id, 'seed': seed,
                  'returncode': p.returncode, 'stdout_tail': p.stdout.strip().splitlines()[-30:]})
            rr = read_json(os.path.join(out_root, 'run_result.json'))
            gates = read_json(os.path.join(out_root, 'gates.json'))
            passed = bool(p.returncode == 0 and rr and rr.get('ok') and gates
                          and gates.get('pass') and rr.get('residual_processes') == 0
                          and rr.get('launch_cmd'))
            executed.append({'attempt_id': attempt_id, 'seed': seed,
                             'returncode': p.returncode, 'passed': passed})
            flush()
            if not passed:
                transitions.append('FAIL_STOP_%d' % seed)
                cls = classify_failure(rr, gates, p.returncode)
                classification = {
                    'artifact': "Stage 7' failure classification",
                    'stage': "7''", 'status': 'STAGE_7R_FAIL_STOPPED',
                    'attempt_id': attempt_id, 'seed': seed,
                    'returncode': p.returncode, **cls,
                    'configured_physics_seed': seed,
                    'residual_processes': (rr or {}).get('residual_processes'),
                    'pgid': (rr or {}).get('pgid'),
                    'automatic_retry': False, 'seed_replacement': False,
                    'recovery_started': False,
                    'seeds_not_run': [s for s in seeds[i + 1:]],
                    'created_at': time.strftime('%Y-%m-%dT%H:%M:%S%z')}
                atomic_json(os.path.join(evid, 'Stage7r_failure_classification.json'),
                            classification)
                write_stop(evid, transitions, executed, status='FAIL_STOPPED',
                           attempt_type=cls['attempt_type'], stop_reason=cls['failure_reason'],
                           seeds=seeds, failed_seed=seed,
                           extra={'stage7r_status': 'STAGE_7R_FAIL_STOPPED'})
                flush()
                emit({'event': 'STOP', 'seed': seed, 'reason': cls['failure_reason'],
                      'attempt_type': cls['attempt_type']})
                return 1
            transitions.append('B_%d_PASS' % seed)
        transitions.append('COMPLETE')
        write_stop(evid, transitions, executed, status='COMPLETE',
                   stop_reason='all_preregistered_attempts_passed',
                   extra={'stage7r_status': 'STAGE_7R_PASS_PROVISIONAL_HOLDOUT'})
        flush()
        emit({'event': 'STOP', 'reason': 'all_preregistered_attempts_passed'})
        return 0
    except Exception as exc:  # noqa: BLE001 - backstop: record, never succeed
        transitions.append('INTERNAL_ERROR_STOP')
        write_stop(evid, transitions, executed, status='FAIL_STOPPED',
                   attempt_type='integrity_failure', stop_reason='orchestrator_internal_error',
                   seeds=seeds,
                   extra={'stage7r_status': 'STAGE_7R_FAIL_STOPPED',
                          'internal_error_type': type(exc).__name__,
                          'internal_error_message': str(exc),
                          'traceback_summary': traceback.format_exc()[-2000:]})
        try:
            flush()
        except Exception:  # noqa: BLE001
            pass
        emit({'event': 'INTERNAL_ERROR_STOP', 'error_type': type(exc).__name__})
        return 4


def self_test():
    """Sandbox (tempfile) + mock-runner tests of the failure-classification paths."""
    checks = {}

    MOCK = r'''#!/usr/bin/env python3
import json, os, sys
out = sys.argv[sys.argv.index('--out-root') + 1]
scen = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'scenario.txt')).read().strip()
open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'last_argv.json'), 'w').write(json.dumps(sys.argv[1:]))
os.makedirs(out, exist_ok=True)
open(os.path.join(out, 'launch_cmd.txt'), 'w').write('mock launch\n')
open(os.path.join(out, 'gz_sim_cmd_expected.txt'), 'w').write('mock gz\n')
def wj(name, obj):
    json.dump(obj, open(os.path.join(out, name), 'w'))
if scen == 'missing_rr':
    sys.exit(1)
if scen == 'hard_gate_fail':
    wj('run_result.json', {'ok': True, 'launch_cmd': 'mock', 'residual_processes': 0, 'trace_valid': True})
    wj('gates.json', {'pass': False, 'checks': {'lateral_ratio': False}})
    sys.exit(0)
if scen == 'structured_env_fail':
    wj('run_result.json', {'ok': False, 'attempt_type': 'environment_failure',
       'failure_reason': 'analysis_failed_no_ready_or_state_trace',
       'episode_verdict': 'NOT_VALID', 'hard_gate_verdict': 'NOT_EVALUATED',
       'trace_valid': False, 'residual_processes': 0, 'launch_cmd': None, 'pgid': None})
    sys.exit(4)
if scen == 'structured_no_tracevalid':
    wj('run_result.json', {'ok': False})
    sys.exit(4)
if scen == 'success':
    wj('run_result.json', {'ok': True, 'launch_cmd': 'mock', 'residual_processes': 0, 'trace_valid': True})
    wj('gates.json', {'pass': True, 'checks': {}})
    sys.exit(0)
sys.exit(9)
'''

    def sandbox(scen, seed=900001, with_ledger=True, prereg=True, order=True,
                fingerprint=True):
        d = tempfile.mkdtemp(prefix='stage7r_orch_')
        mock = os.path.join(d, 'mock_execd.py')
        with open(mock, 'w', encoding='utf-8') as fh:
            fh.write(MOCK)
        with open(os.path.join(d, 'scenario.txt'), 'w', encoding='utf-8') as fh:
            fh.write(scen)
        prereg_doc = {'artifact': 'test', 'stage': "7''", 'preregistered_seeds': [seed]}
        if fingerprint:
            prereg_doc['environment_fingerprint_hash'] = 'fp_test_hash'
        json.dump(prereg_doc, open(os.path.join(d, PREREG_NAME), 'w')) if prereg else None
        json.dump({'order': [{'seed': seed}]},
                  open(os.path.join(d, RUN_ORDER_NAME), 'w')) if order else None
        ledger = os.path.join(d, 'Stage7r_authorization_ledger.json')
        if with_ledger:
            AUTH.issue(1, seed, 'initial_episode', path=ledger)
        return d, mock, ledger

    def read(p):
        return json.load(open(p, encoding='utf-8')) if os.path.isfile(p) else None

    # F-01: prereg/run-order missing -> return 2, no crash
    d, mock, led = sandbox('success', prereg=False)
    checks['prereg_missing_return2'] = run(d, mock, auth_ledger=led) == 2

    # unauthorized attempt -> protocol_violation, return 3
    d, mock, led = sandbox('success', with_ledger=False)
    rc = run(d, mock, auth_ledger=led)
    stop = read(os.path.join(d, 'Stage7r_stop.json')) or {}
    checks['unauthorized_protocol_violation_return3'] = (
        rc == 3 and stop.get('attempt_type') == 'protocol_violation')

    # missing run_result -> integrity_failure, classification + stop written, return 1
    d, mock, led = sandbox('missing_rr')
    rc = run(d, mock, auth_ledger=led)
    cls = read(os.path.join(d, 'Stage7r_failure_classification.json')) or {}
    stop = read(os.path.join(d, 'Stage7r_stop.json')) or {}
    checks['missing_rr_classified_no_crash'] = (
        rc == 1 and cls.get('attempt_type') == 'integrity_failure'
        and stop.get('status') == 'FAIL_STOPPED'
        and cls.get('trace_valid') is False)

    # hard-gate FAIL (rr.ok True, gates pass False)
    d, mock, led = sandbox('hard_gate_fail')
    rc = run(d, mock, auth_ledger=led)
    cls = read(os.path.join(d, 'Stage7r_failure_classification.json')) or {}
    checks['hard_gate_fail_classified'] = (
        rc == 1 and cls.get('attempt_type') == 'hard_gate_failure'
        and cls.get('hard_gate_verdict') == 'FAIL')

    # structured environment failure
    d, mock, led = sandbox('structured_env_fail')
    rc = run(d, mock, auth_ledger=led)
    cls = read(os.path.join(d, 'Stage7r_failure_classification.json')) or {}
    checks['structured_env_failure_honoured'] = (
        rc == 1 and cls.get('attempt_type') == 'environment_failure'
        and cls.get('trace_valid') is False)

    # structured failure with no trace_valid field -> defaults False
    d, mock, led = sandbox('structured_no_tracevalid')
    rc = run(d, mock, auth_ledger=led)
    cls = read(os.path.join(d, 'Stage7r_failure_classification.json')) or {}
    checks['missing_trace_valid_defaults_false'] = (
        rc == 1 and cls.get('trace_valid') is False)

    # success path
    d, mock, led = sandbox('success')
    rc = run(d, mock, auth_ledger=led)
    stop = read(os.path.join(d, 'Stage7r_stop.json')) or {}
    checks['success_returns_0'] = rc == 0 and stop.get('status') == 'COMPLETE'

    # R2-01: prereg fingerprint hash is passed through to the executor cmd
    argv = read(os.path.join(d, 'last_argv.json')) or []
    checks['orchestrator_passes_fingerprint'] = (
        '--expected-fingerprint-hash' in argv
        and argv[argv.index('--expected-fingerprint-hash') + 1] == 'fp_test_hash')
    # R2-01: prereg missing the fingerprint field -> plan-phase fail-close (return 2)
    d2, mock2, led2 = sandbox('success', fingerprint=False)
    checks['orchestrator_missing_fingerprint_failclose'] = (
        run(d2, mock2, auth_ledger=led2) == 2
        and not os.path.isfile(os.path.join(d2, 'last_argv.json')))

    # classify_failure pure cases
    checks['classify_none_integrity'] = classify_failure(None, None, 1)['attempt_type'] == 'integrity_failure'
    checks['classify_ok_true_hard_gate'] = classify_failure(
        {'ok': True, 'trace_valid': True}, {'pass': False}, 0)['attempt_type'] == 'hard_gate_failure'
    checks['classify_structured_default'] = classify_failure(
        {'ok': False}, None, 4)['attempt_type'] == 'integrity_failure'
    return checks


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    if '--self-test' in argv:
        st = self_test()
        print(json.dumps({'self_test': st, 'n_pass': sum(1 for v in st.values() if v),
                          'n_total': len(st)}, indent=2, sort_keys=True))
        return 0 if all(st.values()) else 1
    timeout = 240.0
    evid = EVID
    runner = RUNNER
    if '--timeout' in argv:
        timeout = float(argv[argv.index('--timeout') + 1])
    if '--evid' in argv:
        evid = argv[argv.index('--evid') + 1]
    if '--runner' in argv:
        runner = argv[argv.index('--runner') + 1]
    return run(evid, runner, timeout)


if __name__ == '__main__':
    sys.exit(main())
