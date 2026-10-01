#!/usr/bin/env python3
"""Shared, tested output-path guard for Stage-0 tools (offline, read-only).

One implementation is reused by every Stage-0 writer instead of per-tool copies.
It enforces:

  * path normalisation and ``realpath`` resolution;
  * the target must resolve inside an *allowed root* (never the whole repo);
  * ``..`` traversal that escapes an allowed root is rejected;
  * absolute paths outside the allowed roots are rejected;
  * the target itself may not be a symlink;
  * a symlinked parent that escapes the allowed root is rejected;
  * existing outputs are refused by default (no silent truncation);
  * creation uses ``O_EXCL`` / staging+replace, never following symlinks.

Allowed roots are the Stage-0 generated-artifact directory
``docs/task_p0a_gait_search/p1bd_repair`` and the temporary directory.  ``/tmp``
is always pinned; ``tempfile.gettempdir()`` is merged **only** when its realpath
is outside the workspace root, so a ``TMPDIR`` redirect cannot grant write
access to a repository directory (e.g. P1R).  Duplicate roots are removed in
:func:`_real_roots` by realpath.  Callers may add explicit extra roots for tests.

Errors raise :class:`OutputPathError`; the CLIs map it to
:data:`EXIT_OUTPUT_PATH` with an explicit stderr message.
"""

import hashlib
import json
import os
import tempfile

TOOL_VERSION = 'p0a_output_guard:1.0'
# Stable, documented non-zero exit code for output-path/overwrite violations.
EXIT_OUTPUT_PATH = 3
_STAGE0_REL = ('docs', 'task_p0a_gait_search', 'p1bd_repair')


class OutputPathError(Exception):
    """Raised when an output path violates the Stage-0 safety policy."""


def workspace_root():
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def default_allowed_roots():
    """Allowed output roots: the Stage-0 generated dir plus ``/tmp``.

    ``/tmp`` is always pinned.  ``tempfile.gettempdir()`` (which ``TMPDIR``
    controls) is merged only when its realpath is **outside** the workspace root,
    so a ``TMPDIR`` pointing into the repository cannot grant write access to
    e.g. the frozen P1R tree.  Duplicates are removed by realpath in
    :func:`_real_roots`.
    """
    ws = workspace_root()
    roots = [os.path.join(ws, *_STAGE0_REL), '/tmp']
    tmp = os.path.realpath(tempfile.gettempdir())
    ws_real = os.path.realpath(ws)
    inside_ws = tmp == ws_real or tmp.startswith(ws_real + os.sep)
    if not inside_ws:
        roots.append(tempfile.gettempdir())
    return roots


def _real_roots(roots):
    out = []
    for r in (roots or default_allowed_roots()):
        rr = os.path.realpath(r)
        if rr not in out:
            out.append(rr)
    return out


def _under(path, root):
    return path == root or path.startswith(root + os.sep)


def _root_for(real_path, roots):
    for root in roots:
        if _under(real_path, root):
            return root
    return None


def guard_output(target, allow_existing=False, roots=None):
    """Validate ``target`` and return its absolute path.

    Raises :class:`OutputPathError` on any policy violation.  Does not create
    directories or truncate anything.
    """
    if target is None or (isinstance(target, str) and not target.strip()):
        raise OutputPathError('empty output path')
    roots_r = _real_roots(roots)
    ap = os.path.abspath(os.fspath(target))
    if os.path.islink(ap):
        raise OutputPathError('refusing to write through a symlink: %s' % ap)
    real = os.path.realpath(ap)
    if _root_for(real, roots_r) is None:
        raise OutputPathError(
            'output path is outside the allowed roots: %s (resolves to %s)'
            % (ap, real))
    if os.path.lexists(ap):
        if not allow_existing:
            raise OutputPathError('refusing to overwrite existing output: %s' % ap)
        if os.path.islink(ap) or not os.path.isfile(ap):
            raise OutputPathError(
                'existing output is not a regular file: %s' % ap)
        if _root_for(os.path.realpath(ap), roots_r) is None:
            raise OutputPathError(
                'existing output escapes the allowed roots: %s' % ap)
    return ap


def ensure_parent(target, roots=None):
    """Create missing parent directories, each verified inside an allowed root."""
    roots_r = _real_roots(roots)
    parent = os.path.dirname(os.path.abspath(target))
    missing = []
    probe = parent
    while probe and not os.path.exists(probe):
        missing.append(probe)
        nxt = os.path.dirname(probe)
        if nxt == probe:
            break
        probe = nxt
    if probe:
        if os.path.islink(probe):
            raise OutputPathError('symlink in output path: %s' % probe)
        if _root_for(os.path.realpath(probe), roots_r) is None:
            raise OutputPathError(
                'output parent escapes the allowed roots: %s' % probe)
    for d in reversed(missing):
        try:
            os.mkdir(d)
        except FileExistsError:
            pass
        if os.path.islink(d):
            raise OutputPathError('symlink in output path: %s' % d)
        if _root_for(os.path.realpath(d), roots_r) is None:
            raise OutputPathError(
                'created output parent escapes the allowed roots: %s' % d)
    return parent


def guard_dir(path, allow_existing=True, roots=None):
    """Validate an output *directory* path (created on demand)."""
    roots_r = _real_roots(roots)
    ap = os.path.abspath(os.fspath(path))
    if os.path.lexists(ap) and not os.path.isdir(ap):
        raise OutputPathError('output directory is not a directory: %s' % ap)
    if os.path.islink(ap):
        raise OutputPathError('refusing to use a symlinked output dir: %s' % ap)
    real = os.path.realpath(ap)
    if _root_for(real, roots_r) is None:
        raise OutputPathError(
            'output directory is outside the allowed roots: %s' % ap)
    return ap


def write_text(target, text, allow_existing=False, roots=None):
    """Write text under the guard; exclusive create or staging+replace."""
    ap = guard_output(target, allow_existing=allow_existing, roots=roots)
    ensure_parent(ap, roots=roots)
    if os.path.lexists(ap):
        # controlled replacement: stage, validate, atomic replace
        tmp = '%s.stage-%d' % (ap, os.getpid())
        if os.path.lexists(tmp):
            os.remove(tmp)
        with open(tmp, 'x', encoding='utf-8') as fh:
            fh.write(text)
        os.replace(tmp, ap)
    else:
        fd = os.open(ap, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o644)
        with os.fdopen(fd, 'w', encoding='utf-8') as fh:
            fh.write(text)
    return ap


def write_json(target, obj, allow_existing=False, roots=None):
    """Serialise ``obj`` as strict JSON under the guard."""
    text = json.dumps(obj, indent=2, sort_keys=True, allow_nan=False)
    # validate the serialisation round-trips before touching the target
    json.loads(text)
    return write_text(target, text + '\n', allow_existing=allow_existing,
                      roots=roots)


def stage_and_replace(out_dir, name, text, replacement_log=None, roots=None):
    """Generate to a staging file, validate JSON, hash old, atomically replace.

    Records ``{name, old_sha256, new_sha256}`` in ``replacement_log`` (a list)
    so an existing Stage-0 deliverable is never replaced without a hash trail.
    """
    def validate(payload):
        json.loads(payload)
    return _stage_and_replace(out_dir, name, text, validate, replacement_log,
                              roots)


def stage_and_replace_text(out_dir, name, text, replacement_log=None,
                           roots=None):
    """Text variant of :func:`stage_and_replace` (no JSON validation)."""
    return _stage_and_replace(out_dir, name, text, None, replacement_log,
                              roots)


def _stage_and_replace(out_dir, name, text, validate, replacement_log, roots):
    out_dir = guard_dir(out_dir, roots=roots)
    final = os.path.join(out_dir, name)
    guard_output(final, allow_existing=True, roots=roots)
    staging = os.path.join(out_dir, '.staging-%d' % os.getpid())
    # Refuse a symlinked staging dir (it would let writes escape the root) and
    # any pre-existing entry: create exclusively with os.mkdir.
    if os.path.lexists(staging):
        if os.path.islink(staging):
            raise OutputPathError(
                'refusing symlinked staging directory: %s' % staging)
        raise OutputPathError('staging path already exists: %s' % staging)
    os.mkdir(staging)
    stage_path = os.path.join(staging, name)
    if os.path.lexists(stage_path):
        os.remove(stage_path)
    with open(stage_path, 'x', encoding='utf-8') as fh:
        fh.write(text)
    if validate is not None:
        validate(text)
    old_hash = _sha256(final) if os.path.isfile(final) else None
    os.replace(stage_path, final)
    new_hash = _sha256(final)
    try:
        os.rmdir(staging)  # leave no staging directory behind
    except OSError:
        pass
    if replacement_log is not None:
        replacement_log.append({'name': name, 'old_sha256': old_hash,
                                'new_sha256': new_hash,
                                'replaced': old_hash is not None})
    return final


def _sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


if __name__ == '__main__':
    import sys
    # self-check: never used as a script in the pipeline
    sys.stderr.write('p0a_output_guard is a library, not a CLI\n')
    raise SystemExit(2)
