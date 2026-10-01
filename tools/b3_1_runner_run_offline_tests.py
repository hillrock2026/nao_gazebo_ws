#!/usr/bin/env python3
"""Gate B3.1 runner authoring — execute and log the offline test commands.

Runs each command verbatim in the workspace and records command, cwd,
interpreter, start/end time, stdout+stderr and exit code into
docs/task4_phase4e_pathb_b3_evidence/b3_1_runner/offline_tests.txt.

Invoke from a shell with /opt/ros/jazzy and install/setup.bash sourced so the
rclpy-dependent analysis used by the runner can import its bindings. No Gazebo
or ROS launch is started by any command.
"""

import datetime
import os
import subprocess
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(WS, 'docs', 'task4_phase4e_pathb_b3_evidence', 'b3_1_runner',
                   'offline_tests.txt')
PY = sys.executable

COMMANDS = [
    ('runner offline tests',
     [PY, 'tools/test_task4_phase4e_b3_1_runner_offline.py']),
    ('runner validate mode (read-only)',
     [PY, 'tools/run_task4_phase4e_pathb_b3_1.py', '--mode', 'validate']),
    ('runner plan mode (read-only, 12 commands)',
     [PY, 'tools/run_task4_phase4e_pathb_b3_1.py', '--mode', 'plan']),
    ('existing analysis fixtures',
     [PY, 'tools/test_task4_phase4e_analysis.py']),
    ('existing B3.0 offline fixtures',
     [PY, 'tools/test_task4_phase4e_b3_offline.py']),
    ('B3 candidate table validator',
     [PY, 'tools/b3_0_validate_candidates.py']),
    ('joint-order/limits consistency',
     [PY, 'tools/check_joint_limits_consistency.py']),
    ('Task 3A contract check',
     [PY, 'tools/check_task3a_contract.py']),
    ('B2 9-run keywise regression compare (read-only)',
     [PY, 'tools/b3_0_b2_keywise_regression.py', 'compare',
      '--before', 'docs/task4_phase4e_pathb_b3_evidence/b3_0/b2_metrics_before.json',
      '--after', 'docs/task4_phase4e_pathb_b3_evidence/b3_0/b2_metrics_after.json',
      '--out', 'docs/task4_phase4e_pathb_b3_evidence/b3_1_runner/b2_keywise_recheck.json']),
    ('frozen B3.0 source/test chain verification (from workspace root)',
     ['sha256sum', '-c',
      'docs/task4_phase4e_pathb_b3_evidence/b3_0/source_test_freeze.sha256']),
    ('Python syntax compile (runner + tests + frozen analysis)',
     [PY, '-m', 'py_compile',
      'tools/run_task4_phase4e_pathb_b3_1.py',
      'tools/test_task4_phase4e_b3_1_runner_offline.py',
      'tools/b3_1_runner_run_offline_tests.py',
      'tools/phase4e_metrics.py', 'tools/phase4e_b3_gates.py',
      'src/nao_gz_sim/launch/nao_sim.launch.py']),
]


def main():
    lines = ['Gate B3.1 runner authoring — offline test execution log',
             'workspace: %s' % WS,
             'interpreter: %s' % PY,
             'note: no Gazebo / gz sim / ROS node is started by any command.',
             '']
    overall = 0
    for label, argv in COMMANDS:
        start = datetime.datetime.now().astimezone().isoformat()
        proc = subprocess.run(argv, cwd=WS, capture_output=True, text=True)
        end = datetime.datetime.now().astimezone().isoformat()
        overall = overall or proc.returncode
        lines.append('=' * 78)
        lines.append('TEST: %s' % label)
        lines.append('command: %s' % ' '.join(argv))
        lines.append('cwd: %s' % WS)
        lines.append('interpreter: %s' % argv[0])
        lines.append('start: %s' % start)
        lines.append('end:   %s' % end)
        lines.append('exit_code: %d' % proc.returncode)
        lines.append('--- stdout ---')
        lines.append(proc.stdout.rstrip())
        lines.append('--- stderr ---')
        lines.append(proc.stderr.rstrip())
        lines.append('')
    lines.append('=' * 78)
    lines.append('ALL OFFLINE TESTS EXIT 0' if overall == 0
                 else 'SOME OFFLINE TESTS FAILED (see above)')
    with open(OUT, 'w') as fh:
        fh.write('\n'.join(lines) + '\n')
    print('wrote %s (overall exit %d)' % (OUT, overall))
    return overall


if __name__ == '__main__':
    sys.exit(main())
