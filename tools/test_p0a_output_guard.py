#!/usr/bin/env python3
"""Stage-0 output-path guard fixtures (offline, read-only).

Covers normalisation/realpath, allowed-root enforcement, ``..`` traversal
escape, absolute paths outside the roots, symlinked targets and symlinked
parents, default overwrite refusal, controlled replacement, exclusive
creation and the stable exit code.

Usage: python3 tools/test_p0a_output_guard.py [--json-out F]
"""

import argparse
import json
import os
import shutil
import sys
import tempfile

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import p0a_output_guard as OG  # noqa: E402

RESULTS = []


def check(name, cond, detail=None):
    RESULTS.append({'name': name, 'pass': bool(cond), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                           '' if cond else ' :: %r' % (detail,)))


def _expect_error(fn):
    try:
        fn()
    except OG.OutputPathError as exc:
        return str(exc)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('Stage-0 output-path guard fixtures')

    tmp = tempfile.mkdtemp(prefix='p1r_og_')
    sub = os.path.join(tmp, 'sub')
    os.mkdir(sub)
    try:
        # 1. normal allowed write inside a temp root
        out = os.path.join(sub, 'a.json')
        OG.write_json(out, {'k': 1})
        check('allowed_temp_write', os.path.isfile(out))

        # 2. default refuses overwrite
        err = _expect_error(lambda: OG.write_json(out, {'k': 2}))
        check('default_refuses_overwrite', err is not None
              and 'overwrite' in err, err)
        check('no_truncation_on_refusal',
              json.load(open(out)) == {'k': 1})

        # 3. controlled replacement updates and is atomic
        OG.write_json(out, {'k': 2}, allow_existing=True)
        check('controlled_replace', json.load(open(out)) == {'k': 2})

        # 4. exclusive creation helper
        p = os.path.join(sub, 'excl.txt')
        OG.write_text(p, 'x')
        err = _expect_error(lambda: OG.write_text(p, 'y'))
        check('exclusive_create', err is not None)

        # 5. nested parent creation inside the root
        deep = os.path.join(sub, 'x', 'y', 'z', 'n.json')
        OG.write_json(deep, {'ok': True})
        check('nested_dirs_created', os.path.isfile(deep))

        # 6. .. traversal escaping a restricted root is rejected
        restricted = [sub]
        escape = os.path.join(sub, '..', 'escape.json')
        err = _expect_error(lambda: OG.guard_output(escape, roots=restricted))
        check('dotdot_escape_rejected', err is not None, err)

        # 7. absolute path outside allowed roots is rejected
        err = _expect_error(lambda: OG.guard_output('/etc/passwd',
                                                     roots=restricted))
        check('absolute_outside_rejected', err is not None, err)

        # 8. symlinked target is rejected
        realfile = os.path.join(sub, 'real.json')
        OG.write_json(realfile, {'real': True})
        link = os.path.join(sub, 'link.json')
        os.symlink(realfile, link)
        err = _expect_error(lambda: OG.guard_output(link, roots=restricted))
        check('symlink_target_rejected', err is not None
              and 'symlink' in err, err)

        # 9. symlinked parent escaping the root is rejected
        if not os.path.exists(os.path.join(tmp, 'outside')):
            os.mkdir(os.path.join(tmp, 'outside'))
        linkdir = os.path.join(sub, 'leak')
        os.symlink(os.path.join(tmp, 'outside'), linkdir)
        err = _expect_error(lambda: OG.guard_output(
            os.path.join(linkdir, 'z.json'), roots=restricted))
        check('symlink_parent_escape_rejected', err is not None, err)

        # 10. guard_dir rejects a symlinked directory
        err = _expect_error(lambda: OG.guard_dir(linkdir, roots=restricted))
        check('guard_dir_symlink_rejected', err is not None, err)

        # 11. stable exit code
        check('stable_exit_code', OG.EXIT_OUTPUT_PATH == 3,
              OG.EXIT_OUTPUT_PATH)

        # 12. empty path rejected
        err = _expect_error(lambda: OG.guard_output('', roots=restricted))
        check('empty_path_rejected', err is not None, err)

        # 13. F-5: a pre-planted symlinked staging dir must be rejected
        pid = os.getpid()
        staging = os.path.join(sub, '.staging-%d' % pid)
        leak = os.path.join(tmp, 'outside', 'leak-target')
        os.makedirs(leak, exist_ok=True)
        os.symlink(leak, staging)
        err = _expect_error(lambda: OG.stage_and_replace(
            sub, 'x.json', json.dumps({'k': 1}), roots=[sub]))
        check('symlinked_staging_rejected', err is not None
              and 'staging' in err, err)
        os.remove(staging)
        # normal staging path still works and leaves no staging dir
        OG.stage_and_replace(sub, 'x.json', json.dumps({'k': 2}), roots=[sub])
        check('normal_staging_regression',
              json.load(open(os.path.join(sub, 'x.json'))) == {'k': 2}
              and not os.path.lexists(os.path.join(sub, '.staging-%d' % pid)))

        # 14. F-6: TMPDIR redirect must not remove /tmp from the allowed roots
        old_tmpdir = os.environ.get('TMPDIR')
        old_tfdir14 = tempfile.tempdir
        os.environ['TMPDIR'] = os.path.join(tmp, 'altroot')
        os.makedirs(os.path.join(tmp, 'altroot'), exist_ok=True)
        tempfile.tempdir = None
        try:
            roots = OG.default_allowed_roots()
            roots_real = [os.path.realpath(r) for r in roots]
            check('tmp_pinned_in_roots',
                  os.path.realpath('/tmp') in roots_real, roots_real)
            check('outside_sandbox_merged',
                  os.path.realpath(os.path.join(tmp, 'altroot')) in roots_real,
                  roots_real)
            try:
                OG.guard_output('/tmp/r3_guard_probe_%d.json' % os.getpid(),
                                allow_existing=True)
                check('tmp_still_allowed_under_tmpdir', True)
            except OG.OutputPathError as exc:
                check('tmp_still_allowed_under_tmpdir', False, str(exc))
        finally:
            tempfile.tempdir = old_tfdir14
            if old_tmpdir is None:
                os.environ.pop('TMPDIR', None)
            else:
                os.environ['TMPDIR'] = old_tmpdir

        # 15. G-4: a workspace-internal TMPDIR must NOT become an allowed root.
        # Stand-in lives outside /tmp and outside the real repo/P1R so that the
        # always-pinned '/tmp' root cannot mask the exclusion.
        old_tfdir = tempfile.tempdir
        orig_ws = OG.workspace_root
        ws_link = None
        try:
            ws_sim = tempfile.mkdtemp(prefix='r4_ws_sim_', dir='/var/tmp')
            outside_tmp = True
        except OSError:
            ws_sim = os.path.join(tmp, 'ws_sim')
            os.makedirs(ws_sim, exist_ok=True)
            outside_tmp = False
        try:
            os.environ['TMPDIR'] = ws_sim
            tempfile.tempdir = None
            OG.workspace_root = lambda: ws_sim
            roots_real = [os.path.realpath(r)
                          for r in OG.default_allowed_roots()]
            check('ws_internal_tmpdir_not_in_roots',
                  os.path.realpath(ws_sim) not in roots_real, roots_real)
            check('tmp_pinned_ws_internal',
                  os.path.realpath('/tmp') in roots_real, roots_real)
            if outside_tmp:
                err = _expect_error(lambda: OG.guard_output(
                    os.path.join(ws_sim, 'x.json'), allow_existing=True))
                check('ws_internal_tmpdir_write_rejected', err is not None, err)
            else:
                check('ws_internal_tmpdir_write_rejected_env_limited', True)
            try:
                OG.guard_output('/tmp/r4_probe_%d.json' % os.getpid(),
                                allow_existing=True)
                check('tmp_allowed_ws_internal', True)
            except OG.OutputPathError as exc:
                check('tmp_allowed_ws_internal', False, str(exc))

            # symlinked workspace-internal TMPDIR is resolved then rejected
            ws_link = ws_sim + '_link'
            if not os.path.lexists(ws_link):
                os.symlink(ws_sim, ws_link)
            os.environ['TMPDIR'] = ws_link
            tempfile.tempdir = None
            roots_real = [os.path.realpath(r)
                          for r in OG.default_allowed_roots()]
            check('ws_symlink_tmpdir_not_in_roots',
                  os.path.realpath(ws_link) not in roots_real, roots_real)
            if outside_tmp:
                err = _expect_error(lambda: OG.guard_output(
                    os.path.join(ws_link, 'y.json'), allow_existing=True))
                check('ws_symlink_tmpdir_write_rejected', err is not None, err)
            else:
                check('ws_symlink_tmpdir_write_rejected_env_limited', True)
        finally:
            OG.workspace_root = orig_ws
            tempfile.tempdir = old_tfdir
            os.environ.pop('TMPDIR', None)
            shutil.rmtree(ws_sim, ignore_errors=True)
            if ws_link and os.path.islink(ws_link):
                os.remove(ws_link)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\noutput-guard fixtures: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w') as fh:
            json.dump({'passed': passed, 'total': total, 'results': RESULTS},
                      fh, indent=2, sort_keys=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
