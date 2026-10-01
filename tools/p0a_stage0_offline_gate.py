#!/usr/bin/env python3
"""Stage-0 offline hard-gate runner (offline; no Gazebo / ROS launch).

Executes, verbatim, the 13 authoritative offline commands recorded in
``docs/task_p0a_gait_search/p1r_support_swing/tests/offline_results.txt`` plus
the new Stage-0 test commands, capturing command text, cwd, resolved
interpreter, start/end time, exit code, full stdout/stderr and the repository
files each command changed (by before/after mtime+sha256 over tools/ + docs/task_p0a_gait_search/).

Documented environment recipe (see docs/Task4_Phase4E_PathB_Gate_B3_1_*.md):
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  /usr/bin/python3.12 ...        # python3 (conda 3.14) lacks rclpy

Usage:
  python3 tools/p0a_stage0_offline_gate.py --out docs/.../offline_results_full.txt
"""

import argparse
import datetime
import glob
import hashlib
import os
import subprocess
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import p0a_output_guard as OG  # noqa: E402

WS = os.path.dirname(_TOOLS)
PY = '/usr/bin/python3.12' if os.path.exists('/usr/bin/python3.12') else sys.executable

# (label, original command text, argv) -- command text preserved verbatim from
# the authoritative offline_results.txt (python3 resolved to PY per recipe).
ORIGINAL_13 = [
    ('test_p0a_phase_analysis', 'python3 tools/test_p0a_phase_analysis.py',
     [PY, 'tools/test_p0a_phase_analysis.py']),
    ('test_p0a_gait_phase', 'python3 tools/test_p0a_gait_phase.py',
     [PY, 'tools/test_p0a_gait_phase.py']),
    ('test_p0a_seed_semantics', 'python3 tools/test_p0a_seed_semantics.py',
     [PY, 'tools/test_p0a_seed_semantics.py']),
    ('test_p0a_truth_check', 'python3 tools/test_p0a_truth_check.py',
     [PY, 'tools/test_p0a_truth_check.py']),
    ('test_analyze_measured_gait', 'python3 tools/test_analyze_measured_gait.py',
     [PY, 'tools/test_analyze_measured_gait.py']),
    ('test_task4_phase4e_analysis', 'python3 tools/test_task4_phase4e_analysis.py',
     [PY, 'tools/test_task4_phase4e_analysis.py']),
    ('test_task4_phase4e_b3_offline', 'python3 tools/test_task4_phase4e_b3_offline.py',
     [PY, 'tools/test_task4_phase4e_b3_offline.py']),
    ('test_task4_phase4e_b3_launch', 'python3 tools/test_task4_phase4e_b3_launch.py',
     [PY, 'tools/test_task4_phase4e_b3_launch.py']),
    ('test_task4_phase4e_b3_1_runner_offline',
     'python3 tools/test_task4_phase4e_b3_1_runner_offline.py',
     [PY, 'tools/test_task4_phase4e_b3_1_runner_offline.py']),
    ('b3_0_validate_candidates', 'python3.12 tools/b3_0_validate_candidates.py',
     [PY, 'tools/b3_0_validate_candidates.py']),
    ('check_joint_limits_consistency', 'python3 tools/check_joint_limits_consistency.py',
     [PY, 'tools/check_joint_limits_consistency.py']),
    ('check_task3a_contract', 'python3 tools/check_task3a_contract.py',
     [PY, 'tools/check_task3a_contract.py']),
    ('test_p0a_legacy_toolchain_regression',
     'python3 tools/test_p0a_legacy_toolchain_regression.py',
     [PY, 'tools/test_p0a_legacy_toolchain_regression.py']),
]
NEW_TESTS = [
    ('test_p0a_gate_contract', 'python3 tools/test_p0a_gate_contract.py',
     [PY, 'tools/test_p0a_gate_contract.py']),
    ('test_p0a_aligned_pose', 'python3 tools/test_p0a_aligned_pose.py',
     [PY, 'tools/test_p0a_aligned_pose.py']),
    ('test_p0a_seeddiag_replay', 'python3 tools/test_p0a_seeddiag_replay.py',
     [PY, 'tools/test_p0a_seeddiag_replay.py']),
    ('test_p0a_env_capture', 'python3 tools/test_p0a_env_capture.py',
     [PY, 'tools/test_p0a_env_capture.py']),
    ('test_p0a_no_source_write', 'python3 tools/test_p0a_no_source_write.py',
     [PY, 'tools/test_p0a_no_source_write.py']),
    ('test_p0a_output_guard', 'python3 tools/test_p0a_output_guard.py',
     [PY, 'tools/test_p0a_output_guard.py']),
    ('test_p0a_review_remediation',
     'python3 tools/test_p0a_review_remediation.py',
     [PY, 'tools/test_p0a_review_remediation.py']),
    ('test_p0a_cli_contracts', 'python3 tools/test_p0a_cli_contracts.py',
     [PY, 'tools/test_p0a_cli_contracts.py']),
]


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def snapshot():
    files = glob.glob(os.path.join(_TOOLS, '*.py'))
    files += glob.glob(os.path.join(WS, 'src', 'nao_gz_sim', 'scripts', '*.py'))
    root = os.path.join(WS, 'docs', 'task_p0a_gait_search', 'p1r_support_swing')
    for base, _, names in os.walk(root):
        for n in names:
            files.append(os.path.join(base, n))
    return {f: sha256_file(f) for f in files if os.path.isfile(f)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    try:
        out_path = OG.guard_output(args.out, allow_existing=True)
    except OG.OutputPathError as exc:
        sys.stderr.write('p0a_stage0_offline_gate: error: %s\n' % exc)
        return OG.EXIT_OUTPUT_PATH
    previous_sha = sha256_file(out_path) if os.path.isfile(out_path) else None
    lines = [
        'Stage-0 offline hard gate — full execution log',
        'workspace: %s' % WS,
        'cwd: %s' % os.getcwd(),
        'interpreter: %s' % PY,
        'environment recipe: source /opt/ros/jazzy/setup.bash && '
        'source install/setup.bash',
        'note: no Gazebo / gz sim / ROS launch is started by any command.',
        'generated_at: %s' % datetime.datetime.now().astimezone().isoformat(),
        'previous_output_sha256: %s' % (previous_sha or 'none (new file)'),
        'controlled_replace: staged + validated + atomic replace; the previous '
        'hash above is the replacement audit trail.',
        '=' * 78, '',
    ]
    overall = 0
    summary = []
    for group, commands in (('ORIGINAL-13', ORIGINAL_13), ('NEW', NEW_TESTS)):
        lines.append('##### %s #####' % group)
        lines.append('')
        for label, text, argv in commands:
            before = snapshot()
            start = datetime.datetime.now().astimezone().isoformat()
            proc = subprocess.run(argv, cwd=WS, capture_output=True, text=True)
            end = datetime.datetime.now().astimezone().isoformat()
            after = snapshot()
            changed = [f for f in before if after.get(f) != before[f]]
            new_files = [f for f in after if f not in before]
            overall = overall or proc.returncode
            lines.append('=' * 78)
            lines.append('TEST: %s' % label)
            lines.append('cmd: %s' % text)
            lines.append('argv: %s' % ' '.join(argv))
            lines.append('cwd: %s' % WS)
            lines.append('interpreter_resolved: %s' % argv[0])
            lines.append('start: %s' % start)
            lines.append('end:   %s' % end)
            lines.append('exit_code: %d' % proc.returncode)
            lines.append('repo_files_changed: %s' % (changed or 'none'))
            lines.append('repo_files_created: %s' % (new_files or 'none'))
            lines.append('--- stdout ---')
            lines.append(proc.stdout.rstrip())
            lines.append('--- stderr ---')
            lines.append(proc.stderr.rstrip())
            lines.append('')
            summary.append((group, label, proc.returncode))
        lines.append('')
    lines.append('=' * 78)
    lines.append('SUMMARY (exit codes)')
    for group, label, rc in summary:
        lines.append('  [%s] %-40s exit=%d' % (group, label, rc))
    lines.append('')
    lines.append('ALL COMMANDS EXIT 0' if overall == 0
                 else 'SOME COMMANDS FAILED (see above)')
    try:
        OG.stage_and_replace_text(os.path.dirname(out_path),
                                  os.path.basename(out_path),
                                  '\n'.join(lines) + '\n',
                                  replacement_log=[])
    except OG.OutputPathError as exc:
        sys.stderr.write('p0a_stage0_offline_gate: error: %s\n' % exc)
        return OG.EXIT_OUTPUT_PATH
    print('wrote %s (overall exit %d)' % (args.out, overall))
    return overall


if __name__ == '__main__':
    sys.exit(main())
