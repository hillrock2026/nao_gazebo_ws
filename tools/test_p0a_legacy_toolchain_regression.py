#!/usr/bin/env python3
"""P0-a P1-R regression fixtures for the legacy B3 toolchain (offline).

Guards the regression introduced by the P1 phase-mode change: the B3.0
candidate validator stubbed the producer without ``phase_mode`` (AttributeError)
and the producer's sibling ``nao_gait_phase`` import failed under direct file
loading. This test proves both are fixed and that the validator explicitly
declares it exercises the legacy schedule.

Usage: python3 tools/test_p0a_legacy_toolchain_regression.py [--json-out F]
"""

import argparse
import ast
import json
import os
import subprocess
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(_TOOLS)
B3_0 = os.path.join(_TOOLS, 'b3_0_validate_candidates.py')
B3_1 = os.path.join(_TOOLS, 'test_task4_phase4e_b3_1_runner_offline.py')
RESULTS = []


def check(name, cond, detail=None):
    RESULTS.append({'name': name, 'pass': bool(cond), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                           '' if cond else ' :: %r' % (detail,)))


def static_checks():
    src = open(B3_0).read()
    check('validator_declares_legacy', "VALIDATOR_PHASE_MODE = 'legacy'" in src)
    check('stub_sets_phase_mode', 'd.phase_mode = VALIDATOR_PHASE_MODE' in src)
    check('stub_sets_sway_sign', 'd.sway_sign = ' in src)
    check('load_producer_adds_scripts_path',
          "scripts not in sys.path" in src or 'sys.path.insert' in src)
    # producer resolves its sibling module explicitly
    prod = open(os.path.join(WS, 'src', 'nao_gz_sim', 'scripts',
                             'phase4b_producer.py')).read()
    check('producer_sibling_path_guard',
          '_HERE' in prod and 'sys.path.insert' in prod)


def subprocess_checks():
    for label, path, marker in (
            ('b3_0_validate_candidates', B3_0, 'candidate validation:'),
            ('b3_1_runner_offline', B3_1, '53/53 passed')):
        proc = subprocess.run([sys.executable, path], cwd=WS,
                              capture_output=True, text=True)
        out = proc.stdout + proc.stderr
        check('%s_exit_zero' % label, proc.returncode == 0,
              {'rc': proc.returncode, 'tail': out.strip().splitlines()[-1:]})
        check('%s_marker' % label, marker in out,
              out.strip().splitlines()[-1:])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('P0-a P1-R legacy toolchain regression fixtures')
    static_checks()
    subprocess_checks()
    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\nlegacy toolchain fixtures: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w') as fh:
            json.dump({'passed': passed, 'total': total, 'results': RESULTS},
                      fh, indent=2, sort_keys=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
