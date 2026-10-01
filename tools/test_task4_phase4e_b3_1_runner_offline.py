#!/usr/bin/env python3
"""Gate B3.1 runner — offline tests (no ROS, no Gazebo, no launch).

Covers the runner-authoring prompt §7 checklist:
  1 order read correctly; 2 12 candidates exactly once; 3 13th rejected;
  4 per-field run_config write; 5 foot_pitch explicit; 6 bad configs rejected;
  7 B3 gates used (not B2); 8 body_net_forward 0.08 boundary; 9 single FAIL not
  overridden by aggregate; 10 missing/corrupt trace, timeout, non-zero/finish,
  residual structured failures; 11 output safety; 12 process-group cleanup only
  own pgid; 13 atomic output success/failure; 14 screening cannot trigger repeat;
  15 existing B3.0 offline tests + validator still pass; 16 py_compile +
  joint-limit/contract checks.

Expected values are hand-built fixtures; the runner is never used to generate an
expected value. Usage: python3.12 tools/test_task4_phase4e_b3_1_runner_offline.py
"""

import copy
import json
import os
import shutil
import subprocess
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(_TOOLS)
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import run_task4_phase4e_pathb_b3_1 as R  # noqa: E402

TMP = os.path.join(WS, '.b3_1_test_tmp')
RESULTS = []


def check(name, cond, detail=None):
    RESULTS.append({'name': name, 'pass': bool(cond), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                           '' if cond else ' :: %s' % (detail,)))


def _write_jsonl(path, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as fh:
        for r in rows:
            fh.write(json.dumps(r) + '\n')


def write_metrics_trace(trace, net_forward, finish_ok=True, corrupt=None,
                        reject_command=False, missing=None):
    n = 1300
    times = [0.02 * i for i in range(n)]
    if missing != 'state':
        _write_jsonl(os.path.join(trace, 'state_trace.jsonl'),
                     [{'t': t, 'position': [0.0] * 24} for t in times])
    if missing != 'body':
        _write_jsonl(os.path.join(trace, 'body_pose_trace.jsonl'),
                     [{'t': t, 'x': net_forward * i / (n - 1), 'y': 0.0,
                       'z': 0.3, 'yaw': 0.0} for i, t in enumerate(times)])
    if missing != 'command':
        _write_jsonl(os.path.join(trace, 'command_trace.jsonl'),
                     [{'t': t, 'ok': not (reject_command and i == 5),
                       'position': [0.0] * 24} for i, t in enumerate(times)])
    if missing != 'contact':
        ev = []
        for k in range(8):
            side = 'left' if k % 2 == 0 else 'right'
            ev.append({'t': k * 0.5, 'side': side, 'value': True})
            ev.append({'t': k * 0.5 + 0.4, 'side': side, 'value': False})
        _write_jsonl(os.path.join(trace, 'contact_trace.jsonl'), ev)
    if missing != 'imu':
        _write_jsonl(os.path.join(trace, 'imu_trace.jsonl'),
                     [{'t': t, 'wx': 0.0, 'wy': 0.0, 'wz': 0.0, 'ax': 0.0,
                       'ay': 0.0, 'az': 9.8} for t in times])
    if corrupt == 'state':
        with open(os.path.join(trace, 'state_trace.jsonl'), 'w') as fh:
            fh.write('{not valid json\n')
    if corrupt == 'clock':
        _write_jsonl(os.path.join(trace, 'clock_trace.jsonl'), [{'t': 0.0}])
    elif missing != 'clock':
        _write_jsonl(os.path.join(trace, 'clock_trace.jsonl'), [{'t': 0.0}])


def make_launcher(net_forward=0.2, finish='gait_duration_complete', ready=1.0,
                  residual=0, fail_reason=None, fail_stage='launch',
                  corrupt=None, reject_command=False, missing=None):
    def launcher(candidate_id, cmd, run_dir, trace, timeout):
        if fail_reason:
            return {'fail_reason': fail_reason, 'fail_stage': fail_stage,
                    'ready_sim': None, 'residual_processes': residual,
                    'start_wall': 0.0, 'end_wall': 1.0}
        write_metrics_trace(trace, net_forward, corrupt=corrupt,
                            reject_command=reject_command, missing=missing)
        return {'ready_sim': ready, 'finish_reason': finish,
                'residual_processes': residual, 'exit_code': 0,
                'start_wall': 0.0, 'end_wall': 1.0, 'process_group_exited': True}
    return launcher


def _frozen():
    frozen = R.load_frozen()
    valid = R.validate_candidate_table(frozen)
    return frozen, valid


def _anchor(frozen):
    return [c for c in frozen['candidates']['candidates']
            if c['kind'] == 'anchor'][0]['config']


# 1/2
def test_order_read_and_counts():
    frozen = R.load_frozen()
    order = frozen['run_order']['order']
    expected = ['B3-ANCHOR', 'B3-SL-0.03', 'B3-SF-0.80', 'B3-LE-0.24001',
                'B3-SW-0.09', 'B3-FP-0.05', 'B3-SF-0.65', 'B3-SL-0.04',
                'B3-FP-0.15', 'B3-LE-0.24801', 'B3-SW-0.05', 'B3-CONS-1']
    check('order_read_exact', order == expected, order)
    check('order_12_unique', len(order) == 12 and len(set(order)) == 12)
    valid = R.validate_candidate_table(frozen)
    check('candidate_table_valid', valid.get('ok') is True, valid)


# 3
def test_13th_candidate_rejected():
    frozen = R.load_frozen()
    bad = copy.deepcopy(frozen)
    extra = copy.deepcopy(bad['candidates']['candidates'][0])
    extra['id'] = 'B3-EXTRA-13'
    bad['candidates']['candidates'].append(extra)
    out = R.validate_candidate_table(bad)
    check('thirteenth_rejected', R.is_structured_error(out)
          and out['reason'] == 'candidate_table_invalid', out.get('reason'))


# 4/5
def test_run_config_and_foot_pitch():
    frozen, _ = _frozen()
    anchor = _anchor(frozen)
    runs_root = os.path.join(TMP, 'runs_ok')
    res = R.execute_candidate('B3-ANCHOR', anchor, anchor, runs_root,
                              {'hard_gates': {}}, 0.0,
                              launcher=make_launcher())
    check('execute_ok', res.get('ok') is True, res)
    run_dir = os.path.join(runs_root, 'B3-ANCHOR')
    persisted = R.load_json(os.path.join(run_dir, 'run_config.json'))
    check('run_config_field_by_field', persisted == anchor)
    check('foot_pitch_explicit_present', 'foot_pitch_swing_rad' in persisted
          and persisted['foot_pitch_swing_rad'] == 0.10)
    cmd = open(os.path.join(run_dir, 'launch_cmd.txt')).read()
    check('foot_pitch_on_launch_cmd', 'phase4b_foot_pitch_swing_rad:=0.1' in cmd)
    check('run_manifest_written',
          os.path.isfile(os.path.join(run_dir, 'SHA256SUMS')))


# 6
def test_bad_configs_rejected():
    frozen, _ = _frozen()
    anchor = _anchor(frozen)
    check('missing_field_rejected',
          R.validate_candidate_config({k: v for k, v in anchor.items()
                                       if k != 'step_length_m'}, anchor) != [])
    check('extra_field_rejected',
          R.validate_candidate_config({**anchor, 'evil': 1}, anchor) != [])
    check('wrong_nonfinite_rejected',
          R.validate_candidate_config({**anchor, 'step_length_m': float('nan')},
                                      anchor) != [])
    check('unauthorized_change_rejected',
          R.validate_candidate_config({**anchor, 'duration_sec': 99.0}, anchor) != [])
    check('foot_pitch_removed_rejected',
          R.validate_candidate_config({k: v for k, v in anchor.items()
                                       if k != 'foot_pitch_swing_rad'}, anchor) != [])
    ref = R.load_json(os.path.join(
        WS, 'docs', 'task4_phase4e_pathb_b3_evidence', 'b3_0', 'hard_gates.json'))
    check('loads_without_b2_frozen_configs', isinstance(ref, dict))


# 7/8/9
def test_b3_gates_boundary_and_no_aggregate_override():
    frozen, _ = _frozen()
    anchor = _anchor(frozen)
    thresholds = R.build_thresholds(frozen['hard_gates'])

    def run_with_net(net, reject=False):
        root = os.path.join(TMP, 'gates_%s_%s' % (net, reject))
        r = R.execute_candidate('C-%s-%s' % (net, reject), anchor, anchor, root,
                                thresholds, 0.0,
                                launcher=make_launcher(net_forward=net,
                                                       reject_command=reject))
        return r, os.path.join(root, 'C-%s-%s' % (net, reject))

    r_pass, d_pass = run_with_net(0.09)
    gates_pass = R.load_json(os.path.join(d_pass, 'run_gates.json'))
    check('uses_b3_gate_name', 'body_net_forward_ge_0.08' in gates_pass,
          sorted(gates_pass))
    check('body_net_forward_pass_0.09',
          gates_pass['body_net_forward_ge_0.08']['pass'] is True)

    r_b, d_b = run_with_net(0.08)
    gates_b = R.load_json(os.path.join(d_b, 'run_gates.json'))
    check('body_net_forward_boundary_0.08_pass',
          gates_b['body_net_forward_ge_0.08']['pass'] is True,
          gates_b['body_net_forward_ge_0.08'])

    r_low, d_low = run_with_net(0.079)
    gates_low = R.load_json(os.path.join(d_low, 'run_gates.json'))
    check('body_net_forward_below_0.08_fail',
          gates_low['body_net_forward_ge_0.08']['pass'] is False)
    check('single_fail_not_overridden',
          r_low.get('all_gates_pass') is False)

    r_rej, d_rej = run_with_net(0.2, reject=True)
    gates_rej = R.load_json(os.path.join(d_rej, 'run_gates.json'))
    check('command_reject_gate_fails',
          gates_rej['command_reject_zero']['pass'] is False)
    check('aggregate_does_not_override_reject',
          r_rej.get('all_gates_pass') is False)

    # eligibility uses all_gates_pass only
    results = {'pass': {'ok': True, 'all_gates_pass': True, 'metrics': {},
                        'gates': {'body_net_forward_ge_0.08': {'value': 0.1}}},
               'fail': {'ok': True, 'all_gates_pass': False, 'metrics': {},
                        'gates': {}}}
    elig = R.compute_eligibility(frozen, results)
    check('eligibility_only_all_pass', elig['eligible'] == ['pass']
          and elig['selected'] == ['pass'], elig)


# 10
def test_infra_structured_failures():
    frozen, _ = _frozen()
    anchor = _anchor(frozen)
    thresholds = R.build_thresholds(frozen['hard_gates'])

    def run(tag, **kw):
        root = os.path.join(TMP, 'infra')
        cid = 'T-%s' % tag
        return R.execute_candidate(cid, anchor, anchor, root, thresholds, 0.0,
                                   launcher=make_launcher(**kw)), \
            os.path.join(root, cid)

    r, d = run('missing', missing='state')
    check('missing_trace_structured',
          R.is_structured_error(r) and r['reason'] == 'raw_trace_invalid', r)
    r, d = run('corrupt', corrupt='state')
    check('corrupt_trace_structured',
          R.is_structured_error(r) and r['reason'] == 'raw_trace_invalid', r)
    r, d = run('timeout', fail_reason='launch_timeout')
    check('timeout_structured',
          R.is_structured_error(r) and r['reason'] == 'launch_timeout', r)
    r, d = run('nofinish', finish=None)
    check('missing_finish_structured',
          R.is_structured_error(r) and r['reason'] == 'missing_finish_reason', r)
    r, d = run('safety', finish='safety_stop')
    check('safety_stop_structured',
          R.is_structured_error(r) and r['reason'] == 'producer_safety_stop', r)
    r, d = run('residual', residual=3)
    check('residual_structured',
          R.is_structured_error(r) and r['reason'] == 'residual_processes', r)

    # config roundtrip mismatch
    rd = os.path.join(TMP, 'roundtrip')
    R.write_json_atomic(os.path.join(rd, 'run_config.json'), {'a': 1})
    check('config_roundtrip_mismatch',
          R.is_structured_error(R.check_config_roundtrip({'a': 2}, rd)))
    # launch/config mismatch
    cmd = R.build_launch_cmd(anchor, os.path.join(TMP, 'x', 'trace'))
    bad_cfg = dict(anchor)
    bad_cfg['step_length_m'] = 0.07
    check('config_launch_mismatch',
          R.check_config_matches_launch(bad_cfg, cmd) != [])


# 11
def test_output_safety():
    base = os.path.join(TMP, 'safety')
    ws = os.path.join(base, 'ws')
    outside = os.path.join(base, 'outside')
    hist = os.path.join(ws, 'hist')
    os.makedirs(ws, exist_ok=True)
    os.makedirs(outside, exist_ok=True)
    os.makedirs(hist, exist_ok=True)

    _, err = R.resolve_runs_root(os.path.join(outside, 'runs'), workspace=ws)
    check('outside_workspace_rejected',
          R.is_structured_error(err) and err['reason'] == 'output_outside_workspace')
    _, err = R.resolve_runs_root(os.path.join(ws, '..', 'outside', 'runs'),
                                 workspace=ws)
    check('dotdot_escape_rejected', R.is_structured_error(err))
    link = os.path.join(ws, 'link')
    if os.path.lexists(link):
        os.remove(link)
    os.symlink(outside, link)
    _, err = R.resolve_runs_root(os.path.join(link, 'runs'), workspace=ws)
    check('symlink_escape_rejected', R.is_structured_error(err))
    _, err = R.resolve_runs_root(os.path.join(hist, 'runs'), workspace=ws,
                                 forbidden_prefixes=(hist,))
    check('historical_dir_rejected',
          R.is_structured_error(err)
          and err['reason'] == 'output_forbidden_historical_dir')
    f = os.path.join(ws, 'f')
    open(f, 'w').write('x')
    _, err = R.resolve_runs_root(f, workspace=ws)
    check('file_root_rejected',
          R.is_structured_error(err) and err['reason'] == 'output_root_is_file')

    root = os.path.join(TMP, 'runs_nonempty')
    rd = os.path.join(root, 'B3-ANCHOR')
    os.makedirs(rd, exist_ok=True)
    open(os.path.join(rd, 'keep'), 'w').write('keep')
    _, err = R.resolve_run_dir(root, 'B3-ANCHOR', overwrite=False)
    check('nonempty_run_dir_rejected',
          R.is_structured_error(err) and err['reason'] == 'run_dir_not_empty')
    got, err = R.resolve_run_dir(root, 'B3-ANCHOR', overwrite=True)
    check('overwrite_flag_allows', err is None)


# 12
def test_kill_group_only_own_pgid():
    calls = []
    original = R.os.killpg

    def fake_killpg(pgid, sig):
        calls.append((pgid, sig))
        raise ProcessLookupError()

    R.os.killpg = fake_killpg
    try:
        R.kill_group(4321)
    finally:
        R.os.killpg = original
    check('killpg_only_own_pgid', calls and all(c[0] == 4321 for c in calls), calls)
    src = open(os.path.join(_TOOLS, 'run_task4_phase4e_pathb_b3_1.py')).read()
    check('no_pkill_or_killall', 'pkill' not in src and 'killall' not in src)


# 13
def test_atomic_output():
    d = os.path.join(TMP, 'atomic')
    p = os.path.join(d, 'x.json')
    R.write_json_atomic(p, {'a': 1})
    check('atomic_success', R.load_json(p) == {'a': 1})
    before = set(os.listdir(d))
    original = R.os.replace

    def boom(a, b):
        raise OSError('injected')

    R.os.replace = boom
    raised = False
    try:
        R.write_json_atomic(os.path.join(d, 'y.json'), {'b': 2})
    except OSError:
        raised = True
    finally:
        R.os.replace = original
    after = set(os.listdir(d))
    check('atomic_failure_no_partial', raised and after == before
          and not os.path.exists(os.path.join(d, 'y.json')), (before, after))


# 14
def test_screening_no_repeat():
    frozen, _ = _frozen()
    # no --yes-run -> refuse, and create no run dirs
    runs = os.path.join(TMP, 'screen_runs')
    rc = R.main(['--mode', 'screening', '--runs-root', runs])
    check('screening_requires_yes_run', rc == 2)
    check('screening_created_no_runs', not os.path.isdir(runs))
    rc = R.main(['--mode', 'repeat'])
    check('repeat_requires_explicit', rc == 2)
    # run_screening is screening-only: results contain single runs, no repeats
    root = os.path.join(TMP, 'screen2')
    out = R.run_screening(frozen, root, launcher=make_launcher())
    check('screening_runs_once_per_id',
          out.get('ok') and len(out['results']) == 12
          and out['order'] == frozen['run_order']['order'])
    check('no_repeat_key', 'repeat' not in out)


# 15
def test_existing_tests_still_pass():
    for name in ['tools/test_task4_phase4e_analysis.py',
                 'tools/test_task4_phase4e_b3_offline.py',
                 'tools/b3_0_validate_candidates.py']:
        p = subprocess.run([sys.executable, name], cwd=WS,
                           capture_output=True, text=True)
        check('existing_test_%s' % os.path.basename(name), p.returncode == 0,
              p.stdout.strip().splitlines()[-1:] or p.stderr[-200:])


# 16
def test_compile_and_checks():
    p = subprocess.run([sys.executable, '-m', 'py_compile',
                        'tools/run_task4_phase4e_pathb_b3_1.py',
                        'tools/test_task4_phase4e_b3_1_runner_offline.py'],
                       cwd=WS, capture_output=True, text=True)
    check('py_compile', p.returncode == 0, p.stderr[-200:])
    for name in ['tools/check_joint_limits_consistency.py',
                 'tools/check_task3a_contract.py']:
        p = subprocess.run([sys.executable, name], cwd=WS,
                           capture_output=True, text=True)
        check('check_%s' % os.path.basename(name), p.returncode == 0,
              p.stdout.strip().splitlines()[-1:] or p.stderr[-200:])


def main():
    if os.path.isdir(TMP):
        shutil.rmtree(TMP)
    os.makedirs(TMP, exist_ok=True)
    print('Gate B3.1 runner offline tests')
    test_order_read_and_counts()
    test_13th_candidate_rejected()
    test_run_config_and_foot_pitch()
    test_bad_configs_rejected()
    test_b3_gates_boundary_and_no_aggregate_override()
    test_infra_structured_failures()
    test_output_safety()
    test_kill_group_only_own_pgid()
    test_atomic_output()
    test_screening_no_repeat()
    test_existing_tests_still_pass()
    test_compile_and_checks()
    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\nb3_1 runner offline tests: %d/%d passed' % (passed, total))
    shutil.rmtree(TMP, ignore_errors=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
