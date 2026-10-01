#!/usr/bin/env python3
"""Gate B3.0 — run and log the offline test commands.

Executes each command verbatim in the workspace, capturing command, working
directory, interpreter, start/end times, stdout+stderr and exit code into
docs/task4_phase4e_pathb_b3_evidence/b3_0/offline_tests.txt.

Must be invoked from a shell that has /opt/ros/jazzy and install/setup.bash
sourced so the rclpy-dependent tests can import their bindings.
"""

import datetime
import os
import subprocess
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(WS, 'docs', 'task4_phase4e_pathb_b3_evidence', 'b3_0',
                   'offline_tests.txt')
PY = sys.executable

COMMANDS = [
    ('F-4/F-5/F-7/body_net_forward offline fixtures',
     PY, [PY, 'tools/test_task4_phase4e_b3_offline.py']),
    ('F-7 launch wiring fixtures',
     PY, [PY, 'tools/test_task4_phase4e_b3_launch.py']),
    ('existing analysis fixtures',
     PY, [PY, 'tools/test_task4_phase4e_analysis.py']),
    ('existing arm-swing fixtures',
     PY, [PY, 'tools/test_task4_phase4b_arm_swing.py']),
    ('existing Phase 4D offline fixtures',
     PY, [PY, 'tools/test_task4_phase4d_offline.py']),
    ('B3 candidate table validation',
     PY, [PY, 'tools/b3_0_validate_candidates.py']),
    ('joint-order/limits consistency check',
     PY, [PY, 'tools/check_joint_limits_consistency.py']),
    ('Task 3A contract check',
     PY, [PY, 'tools/check_task3a_contract.py']),
    ('B2 9-run keywise regression compare',
     PY, [PY, 'tools/b3_0_b2_keywise_regression.py', 'compare',
          '--before', 'docs/task4_phase4e_pathb_b3_evidence/b3_0/b2_metrics_before.json',
          '--after', 'docs/task4_phase4e_pathb_b3_evidence/b3_0/b2_metrics_after.json',
          '--out', 'docs/task4_phase4e_pathb_b3_evidence/b3_0/b2_metrics_keywise_diff.json']),
    ('independent FK recompute on historical 9-run trace',
     PY, [PY, 'tools/phase4e_pathb_independent_fk_recompute.py', '--overwrite']),
    ('Python syntax compile (changed/new files)',
     PY, [PY, '-m', 'py_compile',
          'tools/phase4e_metrics.py', 'tools/phase4e_analysis.py',
          'tools/phase4e_pathb_independent_fk_recompute.py',
          'tools/phase4e_b3_gates.py', 'tools/b3_0_b2_keywise_regression.py',
          'tools/b3_0_build_charter.py', 'tools/b3_0_validate_candidates.py',
          'tools/test_task4_phase4e_b3_offline.py',
          'tools/test_task4_phase4e_b3_launch.py',
          'src/nao_gz_sim/launch/nao_sim.launch.py',
          'src/nao_gz_sim/scripts/phase4b_producer.py']),
]


def main():
    lines = ['Gate B3.0 — offline test execution log',
             'workspace: %s' % WS,
             'interpreter: %s' % PY,
             'note: no Gazebo / gz sim / ROS node is started by any command.',
             '']
    overall = 0
    for label, cwd, argv in COMMANDS:
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
