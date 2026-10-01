#!/usr/bin/env python3
"""Stage-0 environment-capture fixtures (offline, read-only).

Usage: python3 tools/test_p0a_env_capture.py [--json-out F]
"""

import argparse
import json
import os
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import p0a_env_capture as E  # noqa: E402

RESULTS = []


def check(name, cond, detail=None):
    RESULTS.append({'name': name, 'pass': bool(cond), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                           '' if cond else ' :: %r' % (detail,)))


def test_capture_contract():
    data = E.capture(command='unit-test')
    for key in ('python', 'ros', 'platform', 'workspace', 'run', 'env',
                'captured_at_utc', 'captured_at_local', 'timezone'):
        check('has_%s' % key, key in data)
    check('python_version_recorded', bool(data['python']['version']))
    check('python_executable_recorded', bool(data['python']['executable']))
    check('pip_version_key', 'pip' in data['python'])
    check('packages_recorded', isinstance(data['python']['packages'], dict))
    check('platform_recorded',
          data['platform']['architecture'] and data['platform']['system'])
    check('git_fallback_recorded',
          data['workspace']['git'].get('is_git_repo') is not None)
    check('run_command_recorded', data['run']['command'] == 'unit-test')
    check('json_serialisable', bool(json.dumps(data, sort_keys=True)))
    return data


def test_redaction():
    os.environ['MY_TOKEN'] = 'super-secret-value'
    os.environ['AWS_SECRET_ACCESS_KEY'] = 'aws-secret'
    os.environ['ROS_DOMAIN_ID'] = '77'
    try:
        data = E.capture()
    finally:
        del os.environ['MY_TOKEN']
        del os.environ['AWS_SECRET_ACCESS_KEY']
        del os.environ['ROS_DOMAIN_ID']
    dumped = json.dumps(data)
    check('secret_token_not_leaked', 'super-secret-value' not in dumped)
    check('secret_aws_not_leaked', 'aws-secret' not in dumped)
    check('redaction_marker_present', '<redacted>' in dumped)
    check('ros_var_captured', data['env'].get('ROS_DOMAIN_ID') == '77')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('Stage-0 environment-capture fixtures')
    test_capture_contract()
    test_redaction()
    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\nenvironment-capture fixtures: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w') as fh:
            json.dump({'passed': passed, 'total': total, 'results': RESULTS},
                      fh, indent=2, sort_keys=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
