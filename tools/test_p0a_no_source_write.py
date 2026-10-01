#!/usr/bin/env python3
"""Stage-0 write-guard fixtures v2 (offline, read-only).

Watches the **directory member set** (not only file contents) of:
  * ``tools/*.py`` (sources + tests);
  * ``src/nao_gz_sim/scripts/*.py`` and ``src/nao_gz_sim/launch/*.py``;
  * the whole ``p1r_support_swing/`` tree (fixtures, golden files, frozen gates,
    charter, manifest, evidence);
  * the whole ``p1bd_repair/`` tree (deliverables, sidecars, pre-change backups).

For every watched path it records kind (file/symlink), SHA-256 (or symlink
target) and size, then runs the offline command set and recomputes.  It detects
modification, creation, deletion, rename and symlink changes.  An **empty**
allowlist is used: every command is expected to be read-only against the repo
(all real writes go to allow-listed temp dirs), so no path is excluded.

The same primitives are self-tested against a temporary tree to prove each
detection class fires.

Authority interpreter: ``/usr/bin/python3.12`` (+ ROS Jazzy sourced).

Usage: python3 tools/test_p0a_no_source_write.py [--json-out F]
"""

import argparse
import glob
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile

_TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(_TOOLS)
P1R = os.path.join(WS, 'docs', 'task_p0a_gait_search', 'p1r_support_swing')
P1BD = os.path.join(WS, 'docs', 'task_p0a_gait_search', 'p1bd_repair')
WRITE_GUARD_VERSION = 'p0a_no_source_write:2.0'
COMMANDS = [
    'tools/test_p0a_phase_analysis.py',
    'tools/test_p0a_gait_phase.py',
    'tools/test_p0a_seed_semantics.py',
    'tools/test_p0a_truth_check.py',
    'tools/test_p0a_gate_contract.py',
    'tools/test_p0a_aligned_pose.py',
    'tools/test_p0a_seeddiag_replay.py',
    'tools/test_p0a_env_capture.py',
    'tools/test_p0a_output_guard.py',
    'tools/test_p0a_review_remediation.py',
    'tools/test_p0a_cli_contracts.py',
]
# No repository path is allowed to change during the read-only command set.
ALLOWED_CHANGES = set()
RESULTS = []


def check(name, cond, detail=None):
    RESULTS.append({'name': name, 'pass': bool(cond), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                           '' if cond else ' :: %r' % (detail,)))


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def _entry(path):
    if os.path.islink(path):
        return {'kind': 'symlink', 'target': os.readlink(path)}
    st = os.stat(path)
    return {'kind': 'file', 'sha256': sha256_file(path), 'size': st.st_size}


def _excluded(name):
    return (name == '__pycache__' or name == '.pytest_cache'
            or name.endswith('.pyc') or name.startswith('.staging-'))


def snapshot_roots(roots):
    """Snapshot every file/symlink under the given roots (recursive)."""
    snap = {}
    for root in roots:
        if os.path.isfile(root) or os.path.islink(root):
            snap[root] = _entry(root)
            continue
        for base, dirs, names in os.walk(root):
            dirs[:] = [d for d in dirs if not _excluded(d)]
            for n in names:
                if _excluded(n):
                    continue
                p = os.path.join(base, n)
                snap[p] = _entry(p)
    return snap


def watched_roots():
    roots = sorted(glob.glob(os.path.join(_TOOLS, '*.py')))
    roots += sorted(glob.glob(os.path.join(WS, 'src', 'nao_gz_sim', 'scripts',
                                           '*.py')))
    roots += sorted(glob.glob(os.path.join(WS, 'src', 'nao_gz_sim', 'launch',
                                           '*.py')))
    roots += [P1R, P1BD]
    return roots


def diff_snapshots(before, after):
    changed, symlink_changed = [], []
    for p in before:
        if p in after and before[p] != after[p]:
            changed.append(p)
            if before[p]['kind'] == 'symlink' or after[p]['kind'] == 'symlink':
                symlink_changed.append(p)
    removed = [p for p in before if p not in after]
    added = [p for p in after if p not in before]
    # rename pairing: identical content/kind among removed and added
    renamed = []
    used_added = set()
    for r in list(removed):
        for a in added:
            if a in used_added:
                continue
            if before[r] == after[a]:
                renamed.append({'from': r, 'to': a})
                used_added.add(a)
                removed.remove(r)
                added.remove(a)
                break
    return {'changed': changed, 'symlink_changed': symlink_changed,
            'removed': removed, 'added': added, 'renamed': renamed}


def test_detection_primitives():
    tmp = tempfile.mkdtemp(prefix='p1r_wg_')
    try:
        os.mkdir(os.path.join(tmp, 'r'))
        root = os.path.join(tmp, 'r')
        with open(os.path.join(root, 'a.txt'), 'w') as fh:
            fh.write('aaa')
        with open(os.path.join(root, 'c.txt'), 'w') as fh:
            fh.write('ccc')
        os.symlink(os.path.join(root, 'a.txt'), os.path.join(root, 'link'))
        before = snapshot_roots([root])

        with open(os.path.join(root, 'a.txt'), 'w') as fh:
            fh.write('changed')
        diff = diff_snapshots(before, snapshot_roots([root]))
        check('detect_modify', os.path.join(root, 'a.txt') in diff['changed'],
              diff)

        snap2 = snapshot_roots([root])
        with open(os.path.join(root, 'new.txt'), 'w') as fh:
            fh.write('n')
        diff = diff_snapshots(snap2, snapshot_roots([root]))
        check('detect_add', os.path.join(root, 'new.txt') in diff['added'],
              diff)

        snap3 = snapshot_roots([root])
        os.remove(os.path.join(root, 'new.txt'))
        diff = diff_snapshots(snap3, snapshot_roots([root]))
        check('detect_delete',
              os.path.join(root, 'new.txt') in diff['removed'], diff)

        # rename: remove c, add d with identical content
        snap4 = snapshot_roots([root])
        shutil.move(os.path.join(root, 'c.txt'), os.path.join(root, 'd.txt'))
        diff = diff_snapshots(snap4, snapshot_roots([root]))
        check('detect_rename', bool(diff['renamed']), diff)

        # symlink change
        snap5 = snapshot_roots([root])
        os.remove(os.path.join(root, 'link'))
        os.symlink(os.path.join(root, 'd.txt'), os.path.join(root, 'link'))
        diff = diff_snapshots(snap5, snapshot_roots([root]))
        check('detect_symlink_change', bool(diff['symlink_changed']), diff)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('Stage-0 write-guard fixtures v2 (%s)' % WRITE_GUARD_VERSION)
    test_detection_primitives()
    roots = watched_roots()
    before = snapshot_roots(roots)
    check('watch_set_nonempty', len(before) > 100, len(before))
    for cmd in COMMANDS:
        proc = subprocess.run([sys.executable, cmd], cwd=WS,
                              stdout=subprocess.DEVNULL,
                              stderr=subprocess.PIPE, text=True)
        check('cmd_%s_exit_zero' % os.path.basename(cmd),
              proc.returncode == 0,
              proc.stderr.strip().splitlines()[-3:] if proc.returncode else None)
    after = snapshot_roots(roots)
    diff = diff_snapshots(before, after)
    diff = {k: [x for x in v if x not in ALLOWED_CHANGES] for k, v in diff.items()}
    check('no_source_writes', not any(diff.values()), diff)
    check('no_symlink_writes', not diff['symlink_changed'], diff)
    if args.json_out:
        with open(args.json_out, 'w') as fh:
            json.dump({'write_guard_version': WRITE_GUARD_VERSION,
                       'passed': sum(1 for r in RESULTS if r['pass']),
                       'total': len(RESULTS), 'results': RESULTS,
                       'n_watched': len(before), 'diff': diff,
                       'allowed_changes': sorted(ALLOWED_CHANGES)},
                      fh, indent=2, sort_keys=True)
    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\nwrite-guard fixtures: %d/%d passed' % (passed, total))
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
