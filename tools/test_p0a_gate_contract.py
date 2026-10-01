#!/usr/bin/env python3
"""P0-a P1-R gate CLI contract fixtures (offline, no Gazebo).

Covers the v2 *wrapper* gate (``frozen_gates_v2.json``, thresholds nested under
``hard_gates``) and the legacy *flat* gate (thresholds at top level) across the
CLI and the runner call path:

  * normal CLI invocation with ``--gates``;
  * wrapper/runner calling the flat ``evaluate_gates`` consumer;
  * wrapper vs flat producing identical verdicts for the same analysis;
  * missing / illegal / unknown arguments and unknown thresholds;
  * a missing or malformed gate file (explicit error, non-zero exit);
  * output-path creation and stdout/stderr responsibilities;
  * the runner no longer unwraps ``hard_gates`` itself.

Usage: python3 tools/test_p0a_gate_contract.py [--json-out F]
"""

import argparse
import io
import json
import os
import shutil
import sys
import tempfile
from contextlib import redirect_stderr, redirect_stdout

_TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(_TOOLS)
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import p0a_phase_analysis as PA  # noqa: E402

WRAPPER = os.path.join(WS, 'docs', 'task_p0a_gait_search', 'p1r_support_swing',
                       'frozen_gates_v2.json')
FWDF08 = os.path.join(WS, 'docs', 'task_p0a_gait_search', 'work', 'reproduce',
                      'fwdf08_seed101')
RESULTS = []


def check(name, cond, detail=None):
    RESULTS.append({'name': name, 'pass': bool(cond), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                           '' if cond else ' :: %r' % (detail,)))


def _flat_from_wrapper(wrapper_path, out_path):
    with open(wrapper_path) as fh:
        data = json.load(fh)
    flat = dict(data['hard_gates'])
    with open(out_path, 'w') as fh:
        json.dump(flat, fh)
    return flat


def test_loader_contract():
    resolved = PA.load_gates(WRAPPER)
    with open(WRAPPER) as fh:
        wrapper = json.load(fh)
    check('wrapper_form_detected', resolved['_gate_form'] == 'wrapper')
    check('wrapper_version_recorded',
          resolved['_gate_version'] == wrapper.get('version'))
    check('wrapper_thresholds_flat_equal',
          all(resolved[k] == wrapper['hard_gates'][k]
              for k in PA.REQUIRED_HARD_GATES))

    tmp = tempfile.mkdtemp(prefix='p1r_gate_')
    try:
        flat_path = os.path.join(tmp, 'flat.json')
        _flat_from_wrapper(WRAPPER, flat_path)
        flat = PA.load_gates(flat_path)
        check('flat_form_detected', flat['_gate_form'] == 'flat')
        check('wrapper_flat_same_thresholds',
              all(resolved[k] == flat[k] for k in PA.REQUIRED_HARD_GATES))
        # normalize_gates is idempotent (returns thresholds without metadata)
        norm = PA.normalize_gates(resolved)
        check('normalize_idempotent',
              all(norm[k] == resolved[k] for k in PA.REQUIRED_HARD_GATES)
              and norm['epsilon_sec'] == resolved['epsilon_sec']
              and all(k not in norm for k in PA.GATE_META_KEYS))

        # F-2: a caller-supplied _gate_form must not bypass any validation.
        valid13 = dict(wrapper['hard_gates'])
        bad_form_flat = dict(valid13)
        bad_form_flat['_gate_form'] = 'flat'
        bad_form_flat['typo_key'] = 1
        try:
            PA.normalize_gates(bad_form_flat, source='form-bypass-flat')
            check('gate_form_flat_does_not_bypass_typo', False)
        except PA.GateContractError as exc:
            check('gate_form_flat_does_not_bypass_typo',
                  'typo_key' in str(exc), str(exc))
        try:
            PA.normalize_gates({'_gate_form': 'flat'},
                               source='form-bypass-missing')
            check('gate_form_flat_missing_keys_fails', False)
        except PA.GateContractError as exc:
            check('gate_form_flat_missing_keys_fails',
                  'missing required' in str(exc), str(exc))
        bad_wrapper_form = {'_gate_form': 'wrapper',
                            'hard_gates': dict(valid13, typo_key=1)}
        try:
            PA.normalize_gates(bad_wrapper_form, source='form-bypass-wrapper')
            check('gate_form_wrapper_does_not_bypass_typo', False)
        except PA.GateContractError as exc:
            check('gate_form_wrapper_does_not_bypass_typo',
                  'typo_key' in str(exc), str(exc))
        legal_form = dict(valid13)
        legal_form['_gate_form'] = 'flat'
        norm_legal = PA.normalize_gates(legal_form)
        check('gate_form_legal_thresholds_ok',
              all(norm_legal[k] == valid13[k]
                  for k in PA.REQUIRED_HARD_GATES))
        check('gate_form_not_in_result',
              all(k not in norm_legal for k in PA.GATE_META_KEYS),
              sorted(norm_legal))

        # missing required threshold -> explicit failure naming the key
        bad = json.load(open(flat_path))
        bad.pop('min_forward')
        bad_path = os.path.join(tmp, 'bad.json')
        json.dump(bad, open(bad_path, 'w'))
        try:
            PA.load_gates(bad_path)
            check('missing_threshold_fails', False)
        except PA.GateContractError as exc:
            check('missing_threshold_fails', 'min_forward' in str(exc), str(exc))

        # unknown wrapper top-level key -> explicit failure
        wrap_bad = {'version': 'x', 'hard_gates': dict(wrapper['hard_gates']),
                    'thresolds': {}}
        wrap_bad_path = os.path.join(tmp, 'wrap_bad.json')
        json.dump(wrap_bad, open(wrap_bad_path, 'w'))
        try:
            PA.load_gates(wrap_bad_path)
            check('unknown_wrapper_key_fails', False)
        except PA.GateContractError as exc:
            check('unknown_wrapper_key_fails', 'thresolds' in str(exc), str(exc))

        # non-numeric threshold -> explicit failure
        nonnum = json.load(open(flat_path))
        nonnum['min_airborne'] = 'high'
        nonnum_path = os.path.join(tmp, 'nonnum.json')
        json.dump(nonnum, open(nonnum_path, 'w'))
        try:
            PA.load_gates(nonnum_path)
            check('nonnumeric_threshold_fails', False)
        except PA.GateContractError as exc:
            check('nonnumeric_threshold_fails', 'min_airborne' in str(exc),
                  str(exc))

        # malformed JSON -> explicit failure
        junk_path = os.path.join(tmp, 'junk.json')
        open(junk_path, 'w').write('{not json')
        try:
            PA.load_gates(junk_path)
            check('malformed_gate_fails', False)
        except PA.GateContractError as exc:
            check('malformed_gate_fails', 'not valid JSON' in str(exc), str(exc))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # missing file -> explicit failure
    try:
        PA.load_gates(os.path.join(WS, 'no_such_gates.json'))
        check('missing_gate_file_fails', False)
    except PA.GateContractError as exc:
        check('missing_gate_file_fails', 'missing' in str(exc), str(exc))


def test_wrapper_calls_flat_consumer():
    analysis = PA.analyse(FWDF08)
    wrapper = PA.load_gates(WRAPPER)
    tmp = tempfile.mkdtemp(prefix='p1r_gate2_')
    try:
        flat_path = os.path.join(tmp, 'flat.json')
        _flat_from_wrapper(WRAPPER, flat_path)
        flat = PA.load_gates(flat_path)
        v_wrapper = PA.evaluate_gates(analysis, WRAPPER)      # raw wrapper dict
        v_wrapped = PA.evaluate_gates(analysis, wrapper)      # normalised wrapper
        v_flat = PA.evaluate_gates(analysis, flat)            # normalised flat
        check('wrapper_path_matches_flat_path',
              v_wrapper['checks'] == v_flat['checks']
              and v_wrapper['pass'] == v_flat['pass'],
              (v_wrapper['pass'], v_flat['pass']))
        check('normalized_wrapper_matches',
              v_wrapped['checks'] == v_wrapper['checks'])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_cli_behavior():
    tmp = tempfile.mkdtemp(prefix='p1r_gate3_')
    try:
        # normal invocation, wrapper gate file, nested output dir
        out = os.path.join(tmp, 'nested', 'deep', 'analysis.json')
        buf, err = io.StringIO(), io.StringIO()
        with redirect_stdout(buf), redirect_stderr(err):
            rc = PA.main(['--run', FWDF08, '--gates', WRAPPER, '--out', out])
        check('cli_normal_exit_zero', rc == 0, (rc, err.getvalue()))
        check('cli_out_created', os.path.isfile(out))
        check('cli_stdout_reports_path', 'wrote %s' % out in buf.getvalue(),
              buf.getvalue())
        check('cli_stderr_empty_on_success', err.getvalue() == '',
              err.getvalue())
        payload = json.load(open(out))
        check('cli_output_has_gates',
              payload['runs'][0].get('gates', {}).get('pass') is not None)

        # wrapper vs flat CLI output verdict identical
        flat_path = os.path.join(tmp, 'flat.json')
        _flat_from_wrapper(WRAPPER, flat_path)
        out_flat = os.path.join(tmp, 'analysis_flat.json')
        rc2 = PA.main(['--run', FWDF08, '--gates', flat_path, '--out', out_flat])
        check('cli_flat_exit_zero', rc2 == 0)
        a = json.load(open(out))['runs'][0]['gates']
        b = json.load(open(out_flat))['runs'][0]['gates']
        check('cli_wrapper_flat_verdict_identical', a['checks'] == b['checks'])

        # missing gate file -> non-zero, message on stderr, no traceback
        err = io.StringIO()
        with redirect_stderr(err):
            rc = PA.main(['--run', FWDF08, '--gates',
                          os.path.join(tmp, 'nope.json'),
                          '--out', os.path.join(tmp, 'x.json')])
        check('cli_missing_gates_exit_nonzero', rc != 0, rc)
        check('cli_missing_gates_stderr',
              'gate file missing' in err.getvalue(), err.getvalue())
        check('cli_missing_gates_no_output',
              not os.path.isfile(os.path.join(tmp, 'x.json')))

        # missing run dir -> non-zero, explicit stderr
        err = io.StringIO()
        with redirect_stderr(err):
            rc = PA.main(['--run', os.path.join(tmp, 'no_run'),
                          '--out', os.path.join(tmp, 'y.json')])
        check('cli_missing_run_exit_nonzero', rc != 0, rc)
        check('cli_missing_run_stderr', 'run_config' in err.getvalue(),
              err.getvalue())

        # missing required --out -> argparse exit 2
        try:
            with redirect_stderr(io.StringIO()):
                PA.main(['--run', FWDF08])
            check('cli_missing_out_arg_fails', False)
        except SystemExit as exc:
            check('cli_missing_out_arg_fails', exc.code == 2, exc.code)

        # unknown argument -> argparse exit 2
        try:
            with redirect_stderr(io.StringIO()):
                PA.main(['--run', FWDF08, '--out', 'z.json', '--bogus'])
            check('cli_unknown_arg_fails', False)
        except SystemExit as exc:
            check('cli_unknown_arg_fails', exc.code == 2, exc.code)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_runner_uses_canonical_loader():
    src = open(os.path.join(_TOOLS, 'run_p0a_gait_candidate.py')).read()
    check('runner_uses_canonical_loader', 'PA.load_gates(' in src)
    check('runner_no_manual_unwrap', "gates.get('hard_gates'" not in src)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('P0-a P1-R gate CLI contract fixtures')
    test_loader_contract()
    test_wrapper_calls_flat_consumer()
    test_cli_behavior()
    test_runner_uses_canonical_loader()
    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\ngate contract fixtures: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w') as fh:
            json.dump({'passed': passed, 'total': total, 'results': RESULTS},
                      fh, indent=2, sort_keys=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
