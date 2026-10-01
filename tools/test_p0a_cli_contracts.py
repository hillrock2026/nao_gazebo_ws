#!/usr/bin/env python3
"""Stage-0 CLI subprocess regression fixtures (offline, real subprocesses).

Exercises the command-line contract of the Stage-0 tools exactly as an operator
would invoke them (not just their Python APIs):

  * ``p0a_seeddiag_replay.py``   generate / compare / replay, BLOCKED exit 1,
                                 bad input exit != 0, safe vs unsafe output;
  * ``p0a_aligned_pose.py``      all alignment options + error paths;
  * ``p0a_env_capture.py``       schema, redaction, output protection;
  * ``p0a_stage0_offline_gate.py`` output-path protection;
  * ``p0a_stage0_deliverables.py`` temp generation + idempotency;
  * ``run_p0a_gait_candidate.py`` gate-contract path (exit 2, no traceback).

All outputs go to allow-listed temp dirs.  Authority interpreter:
``/usr/bin/python3.12`` (+ ROS Jazzy sourced for the runner import path).

Usage: python3 tools/test_p0a_cli_contracts.py [--json-out F]
"""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile

_TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(_TOOLS)
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
PY = '/usr/bin/python3.12' if os.path.exists('/usr/bin/python3.12') else sys.executable
P1R = os.path.join(WS, 'docs', 'task_p0a_gait_search', 'p1r_support_swing')
RUNS = os.path.join(P1R, 'runs')
GATES = os.path.join(P1R, 'frozen_gates_v2.json')
PRECHANGE = os.path.join(WS, 'docs', 'task_p0a_gait_search', 'p1bd_repair',
                         'seed_repeatability_result.prechange.json')
RESULTS = []


def check(name, cond, detail=None):
    RESULTS.append({'name': name, 'pass': bool(cond), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                           '' if cond else ' :: %r' % (detail,)))


def run(args, env=None, cwd=WS):
    return subprocess.run([PY] + args, cwd=cwd, capture_output=True, text=True,
                          env=env)


def write_json(path, obj):
    with open(path, 'w') as fh:
        json.dump(obj, fh)


def test_seeddiag_cli(tmp):
    out = os.path.join(tmp, 'gen.json')
    p = run(['tools/p0a_seeddiag_replay.py', 'generate',
             '--runs-root', RUNS, '--gates', GATES, '--out', out])
    check('seeddiag_generate_exit0', p.returncode == 0, p.stderr[-300:])
    check('seeddiag_generate_output', os.path.isfile(out))

    # compare prechange (original) vs generated -> PASS
    cmp_out = os.path.join(tmp, 'cmp.json')
    p = run(['tools/p0a_seeddiag_replay.py', 'compare',
             '--old', PRECHANGE, '--new', out, '--out', cmp_out])
    check('seeddiag_compare_pass', p.returncode == 0, p.stderr[-300:])
    diff = json.load(open(cmp_out))
    check('seeddiag_compare_verdict_pass', diff['verdict'] == 'PASS',
          diff['verdict'])
    check('seeddiag_compare_no_semantic', diff['n_semantic_diffs'] == 0)
    check('seeddiag_compare_gate_unchanged', diff['gate_changed'] is False)

    # semantic change -> BLOCKED exit 1
    old = os.path.join(tmp, 'old.json')
    new = os.path.join(tmp, 'new.json')
    write_json(old, {'forward_displacement_m': 0.1})
    write_json(new, {'forward_displacement_m': 0.2})
    p = run(['tools/p0a_seeddiag_replay.py', 'compare',
             '--old', old, '--new', new, '--out', os.path.join(tmp, 'd1.json')])
    check('seeddiag_semantic_blocked_exit1', p.returncode == 1, p.returncode)

    write_json(new, {'runs': {'seeddiag_r1': {'gate_pass': True}}})
    write_json(old, {'runs': {'seeddiag_r1': {'gate_pass': False}}})
    out_g = os.path.join(tmp, 'd2.json')
    p = run(['tools/p0a_seeddiag_replay.py', 'compare',
             '--old', old, '--new', new, '--out', out_g])
    check('seeddiag_gate_blocked_exit1', p.returncode == 1, p.returncode)
    check('seeddiag_gate_blocked_flag',
          json.load(open(out_g))['gate_changed'] is True)

    # bad input -> non-zero, no traceback for missing file
    p = run(['tools/p0a_seeddiag_replay.py', 'compare',
             '--old', os.path.join(tmp, 'nope.json'), '--new', new,
             '--out', os.path.join(tmp, 'd3.json')])
    check('seeddiag_missing_input_nonzero', p.returncode != 0, p.returncode)
    check('seeddiag_missing_input_no_traceback',
          'Traceback' not in p.stderr, p.stderr[-200:])

    # existing output refused (exit 3)
    p = run(['tools/p0a_seeddiag_replay.py', 'generate',
             '--runs-root', RUNS, '--gates', GATES, '--out', out])
    check('seeddiag_existing_output_refused', p.returncode == 3, p.returncode)

    # traversal / unsafe path refused (exit 3)
    p = run(['tools/p0a_seeddiag_replay.py', 'generate',
             '--runs-root', RUNS, '--gates', GATES,
             '--out', os.path.join(WS, 'docs', 'evil.json')])
    check('seeddiag_unsafe_path_refused', p.returncode == 3, p.returncode)

    # symlink output refused
    tgt = os.path.join(tmp, 'real_g.json')
    link = os.path.join(tmp, 'link_g.json')
    write_json(tgt, {'x': 1})
    if not os.path.islink(link):
        os.symlink(tgt, link)
    p = run(['tools/p0a_seeddiag_replay.py', 'generate',
             '--runs-root', RUNS, '--gates', GATES, '--out', link])
    check('seeddiag_symlink_output_refused', p.returncode == 3, p.returncode)

    # replay command happy path
    replay_out = os.path.join(tmp, 'replay.json')
    replay_diff = os.path.join(tmp, 'replay_diff.json')
    p = run(['tools/p0a_seeddiag_replay.py', 'replay',
             '--runs-root', RUNS, '--gates', GATES, '--frozen', PRECHANGE,
             '--out', replay_out, '--diff-out', replay_diff])
    check('seeddiag_replay_exit0', p.returncode == 0, p.stderr[-300:])


def test_aligned_cli(tmp):
    a = os.path.join(RUNS, 'seeddiag_r1')
    b = os.path.join(RUNS, 'seeddiag_r2')
    base = ['tools/p0a_aligned_pose.py', '--run-a', a, '--run-b', b]

    out = os.path.join(tmp, 'aligned.json')
    p = run(base + ['--time-reference', 'absolute', '--tolerance', '0.02',
                    '--interpolation', 'none', '--out', out])
    check('aligned_success', p.returncode == 0, p.stderr[-300:])
    rep = json.load(open(out))
    check('aligned_triple_exact',
          rep['max_abs_diff_m']['x'] == 0.0255788167045869
          and rep['max_abs_diff_m']['y'] == 0.019062282984621374
          and rep['max_abs_diff_m']['z'] == 0.0034137217641802,
          rep['max_abs_diff_m'])
    check('aligned_method_json',
          rep['method']['time_reference'] == 'absolute'
          and rep['method']['interpolation'] == 'none'
          and rep['method']['pairing'] == 'all_cross_pairs_within_tolerance'
          and rep['method']['offset_handling'] == 'none')

    for label, extra in (
            ('gait_relative', ['--time-reference', 'gait_relative']),
            ('nearest', ['--interpolation', 'nearest']),
            ('linear', ['--interpolation', 'linear']),
            ('hold', ['--interpolation', 'hold']),
            ('tolerance', ['--tolerance', '0.05']),
            ('offset_estimate', ['--offset-mode', 'estimate']),
            ('offset_subtract_ready', ['--offset-mode', 'subtract_ready'])):
        o = os.path.join(tmp, 'a_%s.json' % label)
        p = run(base + extra + ['--out', o])
        check('aligned_option_%s' % label, p.returncode == 0, p.stderr[-200:])

    # empty input
    empty = os.path.join(tmp, 'empty_run')
    os.makedirs(os.path.join(empty, 'trace'))
    open(os.path.join(empty, 'trace', 'body_pose_trace.jsonl'), 'w').close()
    p = run(['tools/p0a_aligned_pose.py', '--run-a', empty, '--run-b', b,
             '--out', os.path.join(tmp, 'e.json')])
    check('aligned_empty_input_nonzero', p.returncode != 0, p.returncode)
    check('aligned_empty_input_no_traceback', 'Traceback' not in p.stderr)

    # duplicate + non-monotonic tolerated
    dup = os.path.join(tmp, 'dup_run')
    os.makedirs(os.path.join(dup, 'trace'))
    with open(os.path.join(dup, 'trace', 'body_pose_trace.jsonl'), 'w') as fh:
        for row in ({'t': 0.2, 'x': 1, 'y': 0, 'z': 0},
                    {'t': 0.2, 'x': 1, 'y': 0, 'z': 0},
                    {'t': 0.1, 'x': 0, 'y': 0, 'z': 0}):
            fh.write(json.dumps(row) + '\n')
    p = run(['tools/p0a_aligned_pose.py', '--run-a', dup, '--run-b', dup,
             '--out', os.path.join(tmp, 'dup.json')])
    check('aligned_dup_nonmono_ok', p.returncode == 0, p.stderr[-200:])

    # existing output refused
    p = run(base + ['--out', out])
    check('aligned_existing_refused', p.returncode == 3, p.returncode)

    # unsafe path refused
    p = run(base + ['--out', os.path.join(WS, 'docs', 'evil_aligned.json')])
    check('aligned_unsafe_refused', p.returncode == 3, p.returncode)

    # symlink output refused
    tgt = os.path.join(tmp, 'aligned_real.json')
    link = os.path.join(tmp, 'aligned_link.json')
    write_json(tgt, {'x': 1})
    if not os.path.islink(link):
        os.symlink(tgt, link)
    p = run(base + ['--out', link])
    check('aligned_symlink_refused', p.returncode == 3, p.returncode)


def test_env_cli(tmp):
    out = os.path.join(tmp, 'env.json')
    env = dict(os.environ)
    env['STAGE0_TEST_TOKEN'] = 'leak-me-please'
    p = run(['tools/p0a_env_capture.py', '--out', out], env=env)
    check('env_success', p.returncode == 0, p.stderr[-300:])
    data = json.load(open(out))
    for key in ('python', 'ros', 'gazebo', 'platform', 'workspace', 'run',
                'env', 'timezone'):
        check('env_has_%s' % key, key in data)
    check('env_python_executable', bool(data['python']['executable']))
    check('env_gazebo_structured', 'available' in data['gazebo'])
    check('env_colcon_structured', 'available' in data['ros']['colcon'])
    check('env_secret_redacted',
          'leak-me-please' not in json.dumps(data)
          and '<redacted>' in json.dumps(data))
    check('env_ros_section', isinstance(data['ros'], dict))

    p = run(['tools/p0a_env_capture.py', '--out', out], env=env)
    check('env_existing_refused', p.returncode == 3, p.returncode)
    p = run(['tools/p0a_env_capture.py',
             '--out', os.path.join(WS, 'docs', 'evil_env.json')], env=env)
    check('env_unsafe_refused', p.returncode == 3, p.returncode)


def test_offline_gate_cli(tmp):
    p = run(['tools/p0a_stage0_offline_gate.py',
             '--out', os.path.join(WS, 'docs', 'evil_gate.txt')])
    check('offline_gate_unsafe_refused', p.returncode == 3, p.returncode)
    check('offline_gate_error_on_stderr',
          'outside the allowed roots' in p.stderr, p.stderr[-200:])


def test_deliverables_cli(tmp):
    out_dir = os.path.join(tmp, 'deliverables')
    os.mkdir(out_dir)
    # an already-extended frozen file requires the pre-change backup in out_dir
    shutil.copyfile(PRECHANGE, os.path.join(
        out_dir, 'seed_repeatability_result.prechange.json'))
    p = run(['tools/p0a_stage0_deliverables.py', '--out-dir', out_dir])
    check('deliverables_exit0', p.returncode == 0, p.stderr[-400:])
    for name in ('seeddiag_replay.json', 'seeddiag_replay_diff.json',
                 'analysis_source_hashes.json', 'environment.json',
                 'test_file_hashes.json',
                 'seed_repeatability_result_extension.json',
                 'remediation_summary.json'):
        check('deliverables_has_%s' % name, os.path.isfile(
            os.path.join(out_dir, name)))
    side = json.load(open(os.path.join(
        out_dir, 'seed_repeatability_result_extension.json')))
    check('deliverables_sidecar_complete',
          'schema_version' in side and 'prechange_sha256' in side
          and 'historical_provenance' in side, sorted(side))
    side_bytes = open(os.path.join(
        out_dir, 'seed_repeatability_result_extension.json')).read()
    # second run must keep the complete sidecar (no downgrade) and succeed
    p = run(['tools/p0a_stage0_deliverables.py', '--out-dir', out_dir])
    check('deliverables_idempotent_exit0', p.returncode == 0, p.stderr[-300:])
    check('deliverables_sidecar_not_downgraded',
          open(os.path.join(
              out_dir, 'seed_repeatability_result_extension.json')).read()
          == side_bytes)
    # unsafe out-dir refused
    p = run(['tools/p0a_stage0_deliverables.py',
             '--out-dir', os.path.join(WS, 'docs', 'evil_dir')])
    check('deliverables_unsafe_refused', p.returncode == 3, p.returncode)


def test_runner_gate_contract(tmp):
    good = GATES
    flat = os.path.join(tmp, 'flat_good.json')
    wrap = json.load(open(GATES))
    write_json(flat, wrap['hard_gates'])
    missing = os.path.join(tmp, 'missing.json')
    malformed = os.path.join(tmp, 'malformed.json')
    open(malformed, 'w').write('{not json')
    wrap_bad = os.path.join(tmp, 'wrap_bad.json')
    write_json(wrap_bad, {'version': 'x', 'hard_gates': 'not-an-object'})
    wrap_unknown = os.path.join(tmp, 'wrap_unknown.json')
    write_json(wrap_unknown, {'version': 'x', 'hard_gates': wrap['hard_gates'],
                              'extra_section': 1})
    flat_unknown = os.path.join(tmp, 'flat_unknown.json')
    write_json(flat_unknown, dict(wrap['hard_gates'], min_foward=0.1))
    missing_field = os.path.join(tmp, 'missing_field.json')
    d = dict(wrap['hard_gates'])
    d.pop('min_forward')
    write_json(missing_field, d)
    # F-2: caller-supplied gate_form must not bypass validation
    form_flat_typo = os.path.join(tmp, 'form_flat_typo.json')
    write_json(form_flat_typo, dict(wrap['hard_gates'], _gate_form='flat',
                                    typo_key=1))
    form_flat_only = os.path.join(tmp, 'form_flat_only.json')
    write_json(form_flat_only, {'_gate_form': 'flat'})
    form_wrap_typo = os.path.join(tmp, 'form_wrap_typo.json')
    write_json(form_wrap_typo, {'_gate_form': 'wrapper',
                                'hard_gates': dict(wrap['hard_gates'],
                                                   typo_key=1)})
    form_flat_legal = os.path.join(tmp, 'form_flat_legal.json')
    write_json(form_flat_legal, dict(wrap['hard_gates'], _gate_form='flat'))

    required = ['--candidate-id', 'x', '--seed', '1',
                '--out-root', os.path.join(tmp, 'runroot')]

    def validate(gatefile, expect_rc, label):
        p = run(['tools/run_p0a_gait_candidate.py', '--validate-gates',
                 '--p1r-gates', gatefile] + required)
        check('runner_%s_rc%d' % (label, expect_rc), p.returncode == expect_rc,
              (p.returncode, p.stderr[-200:]))
        if expect_rc == 0:
            check('runner_%s_validated' % label,
                  json.loads(p.stdout)['validated'] is True)
        else:
            check('runner_%s_stderr' % label,
                  'gate contract error' in p.stderr, p.stderr[-200:])
            check('runner_%s_no_pseudo_success' % label,
                  '"validated": true' not in p.stdout.lower(), p.stdout[:200])
            check('runner_%s_no_traceback' % label,
                  'Traceback' not in p.stderr, p.stderr[-200:])

    validate(good, 0, 'wrapper_good')
    validate(flat, 0, 'flat_good')
    validate(missing, 2, 'missing')
    validate(malformed, 2, 'malformed')
    validate(wrap_bad, 2, 'wrapper_schema_error')
    validate(wrap_unknown, 2, 'wrapper_unknown_key')
    validate(flat_unknown, 2, 'flat_unknown_key')
    validate(missing_field, 2, 'missing_field')
    validate(form_flat_typo, 2, 'gate_form_flat_typo')
    validate(form_flat_only, 2, 'gate_form_flat_only')
    validate(form_wrap_typo, 2, 'gate_form_wrapper_typo')
    validate(form_flat_legal, 0, 'gate_form_flat_legal')

    # positional --validate-gates GATE_FILE (no required run args) form
    p = run(['tools/run_p0a_gait_candidate.py', '--validate-gates', good])
    check('runner_positional_validate_good_rc0', p.returncode == 0,
          (p.returncode, p.stderr[-200:]))
    check('runner_positional_validate_good_out',
          p.returncode == 0 and json.loads(p.stdout)['validated'] is True)
    p = run(['tools/run_p0a_gait_candidate.py', '--validate-gates',
             form_flat_typo])
    check('runner_positional_validate_bad_rc2', p.returncode == 2,
          (p.returncode, p.stderr[-200:]))
    check('runner_positional_validate_bad_stderr',
          'gate contract error' in p.stderr and 'Traceback' not in p.stderr)
    # real-run mode still requires the run arguments (explicit, exit 2)
    p = run(['tools/run_p0a_gait_candidate.py', '--p1r-gates', good])
    check('runner_missing_run_args_rc2', p.returncode == 2, p.returncode)
    check('runner_missing_run_args_stderr',
          'missing required arguments' in p.stderr, p.stderr[-200:])


def _p1r_snapshot():
    snap = {}
    for base, dirs, names in os.walk(P1R):
        dirs.sort()
        for n in sorted(names):
            p = os.path.join(base, n)
            snap[os.path.relpath(p, P1R)] = hashlib.sha256(
                open(p, 'rb').read()).hexdigest()
    return snap


def test_runner_out_root_guard(tmp):
    """F-1: --allow-overwrite must never delete the frozen P1R evidence tree."""
    before = _p1r_snapshot()
    required = ['--candidate-id', 'x', '--seed', '1', '--allow-overwrite']
    for label, out_root in (
            ('p1r_subdir',
             os.path.join(P1R, 'runs', 'probe')),
            ('p1r_root', P1R)):
        p = run(['tools/run_p0a_gait_candidate.py', '--out-root', out_root]
                + required)
        check('runner_out_root_%s_rejected' % label, p.returncode != 0,
              (p.returncode, p.stderr[-200:]))
        check('runner_out_root_%s_stderr' % label,
              'frozen P1R evidence' in p.stderr, p.stderr[-200:])
        check('runner_out_root_%s_no_traceback' % label,
              'Traceback' not in p.stderr, p.stderr[-200:])
    after = _p1r_snapshot()
    check('p1r_unchanged_after_runner_guard',
          before == after,
          {'n_before': len(before), 'n_after': len(after)})
    check('p1r_probe_not_created',
          not os.path.exists(os.path.join(P1R, 'runs', 'probe')))

    # normal overwrite regression on a sandbox dir (function-level, no Gazebo)
    sandbox = os.path.join(tmp, 'overwrite_sandbox')
    os.makedirs(os.path.join(sandbox, 'trace'))
    with open(os.path.join(sandbox, 'result_p0a.json'), 'w') as fh:
        fh.write('{"stale": true}')
    try:
        import run_p0a_gait_candidate as R
        trace = R.prepare_out_root(sandbox, allow_overwrite=True)
        check('runner_normal_overwrite_ok',
              os.path.isdir(trace)
              and not os.path.exists(os.path.join(sandbox, 'result_p0a.json')))
    except Exception as exc:  # noqa: BLE001 - test must record the real error
        check('runner_normal_overwrite_ok', False, repr(exc))
    # without --allow-overwrite an existing root is still refused
    try:
        import run_p0a_gait_candidate as R
        try:
            R.prepare_out_root(sandbox, allow_overwrite=False)
            check('runner_existing_without_allow_refused', False)
        except R.RunnerError:
            check('runner_existing_without_allow_refused', True)
    except Exception as exc:  # noqa: BLE001
        check('runner_existing_without_allow_refused', False, repr(exc))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('Stage-0 CLI subprocess fixtures')
    tmp = tempfile.mkdtemp(prefix='p1r_cli_')
    try:
        test_seeddiag_cli(tmp)
        test_aligned_cli(tmp)
        test_env_cli(tmp)
        test_offline_gate_cli(tmp)
        test_deliverables_cli(tmp)
        test_runner_gate_contract(tmp)
        test_runner_out_root_guard(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\ncli-contract fixtures: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w') as fh:
            json.dump({'passed': passed, 'total': total, 'results': RESULTS},
                      fh, indent=2, sort_keys=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
