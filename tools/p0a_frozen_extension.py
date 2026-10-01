#!/usr/bin/env python3
"""Frozen-evidence extension sidecar state machine (offline, no in-place edits).

The historical P1-R ``seed_repeatability_result.json`` was schema-extended *in
place* during the original Stage-0 run (an ``aligned_body_pose_method`` block +
``_stage0_schema_extension`` marker).  That in-place change is a permanent
historical fact disclosed by the report erratum; this module never repeats it.

Policy:
  * the frozen file is **read-only** here; it is never opened for writing;
  * any metadata extension is recorded in a **sidecar**
    (``seed_repeatability_result_extension.json``);
  * a complete sidecar is never downgraded by a later run;
  * a missing / mismatched pre-change backup for an already-extended frozen file
    is a hard error (never reconstructed from the extended file);
  * the pre-change hash has **no independent historical freeze source** (no git,
    no freeze manifest) — this is recorded, not hidden.

States:
  frozen : ``pristine`` | ``historically_extended`` | ``extended_without_marker``
  backup : ``valid`` | ``missing`` | ``mismatched``
  sidecar: ``complete`` | ``legacy`` | ``missing``
"""

import hashlib
import json
import os

import p0a_output_guard as OG

SCHEMA_VERSION = 'stage0-extension-sidecar:1.0'
SIDECAR_NAME = 'seed_repeatability_result_extension.json'
PRECHANGE_NAME = 'seed_repeatability_result.prechange.json'
ERATUM_REFERENCE = 'P1BD_repair_report.md#erratum'
ARTIFACT = 'Stage-0 frozen-evidence extension sidecar'
EXTENSION_FIELDS = ('aligned_body_pose_method', '_stage0_schema_extension')

REQUIRED_SIDECAR_KEYS = (
    'artifact', 'schema_version', 'extension_type', 'source_frozen_file',
    'prechange_backup', 'prechange_sha256', 'postchange_sha256',
    'extension_fields', 'aligned_body_pose_method', 'values_changed',
    'n_semantic_diffs', 'n_nonsemantic_diffs', 'gate_changed',
    'verdict_changed', 'replay_verdict', 'created_at', 'recovered_at',
    'historical_provenance', 'erratum_reference', 'frozen_state',
    'backup_state',
)

HISTORICAL_PROVENANCE = (
    'The pre-change SHA-256 of seed_repeatability_result.json has NO independent '
    'historical freeze source: the workspace is not a git repository and the '
    'P1R freeze manifest covers only P1R_charter.md and frozen_gates_v2.json. '
    'It is recorded from the Stage-0 in-place extension marker and the local '
    'prechange backup; byte-level historical authenticity is therefore a limited '
    'assurance, not an independent proof. The frozen numeric fields were '
    'independently re-derived from the raw traces by the replay comparator '
    '(0 semantic diffs, gate/verdict unchanged).'
)


class ExtensionError(Exception):
    """Raised when the frozen-extension state machine cannot proceed safely."""


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def read_json(path):
    with open(path, 'r', encoding='utf-8') as fh:
        return json.load(fh)


def _rel(path):
    ws = OG.workspace_root()
    try:
        rel = os.path.relpath(path, ws)
    except ValueError:
        return path
    return rel if not rel.startswith('..') else path


def detect_frozen_state(frozen):
    has_marker = '_stage0_schema_extension' in frozen
    has_method = 'aligned_body_pose_method' in frozen
    if has_marker:
        return 'historically_extended'
    if has_method:
        return 'extended_without_marker'
    return 'pristine'


def detect_backup_state(backup_path, recorded_sha):
    if not os.path.isfile(backup_path):
        return 'missing'
    actual = sha256_file(backup_path)
    if recorded_sha is None:
        return 'unknown'
    return 'valid' if actual == recorded_sha else 'mismatched'


def detect_sidecar_state(sidecar_path):
    if not os.path.isfile(sidecar_path):
        return 'missing'
    try:
        data = read_json(sidecar_path)
    except ValueError:
        return 'legacy'
    if not isinstance(data, dict):
        return 'legacy'
    missing = [k for k in REQUIRED_SIDECAR_KEYS if k not in data]
    return 'complete' if not missing else 'legacy'


def is_complete(sidecar):
    return (isinstance(sidecar, dict)
            and all(k in sidecar for k in REQUIRED_SIDECAR_KEYS))


def build_sidecar(*, frozen_path, backup_path, prechange_sha, postchange_sha,
                  extension_fields, aligned_body_pose_method, diff,
                  frozen_state, backup_state, created_at, recovered_at):
    return {
        'artifact': ARTIFACT,
        'schema_version': SCHEMA_VERSION,
        'extension_type': 'aligned_body_pose_method + _stage0_schema_extension',
        'source_frozen_file': _rel(frozen_path),
        'prechange_backup': _rel(backup_path),
        'prechange_sha256': prechange_sha,
        'postchange_sha256': postchange_sha,
        'extension_fields': list(extension_fields),
        'aligned_body_pose_method': aligned_body_pose_method,
        'values_changed': False,
        'n_semantic_diffs': diff.get('n_semantic_diffs'),
        'n_nonsemantic_diffs': diff.get('n_nonsemantic_diffs'),
        'gate_changed': diff.get('gate_changed'),
        'verdict_changed': diff.get('verdict_changed'),
        'replay_verdict': diff.get('verdict'),
        'created_at': created_at,
        'recovered_at': recovered_at,
        'historical_provenance': HISTORICAL_PROVENANCE,
        'erratum_reference': ERATUM_REFERENCE,
        'frozen_state': frozen_state,
        'backup_state': backup_state,
        'review_status_before': 'PASS_WITH_ERRATUM (待准入整改)',
        'review_status_after': 'PASS_WITH_ERRATUM (整改完成，可进入 P1-BD)',
        'frozen_evidence_modified_by_this_tool': False,
        'admission_ready': True,
    }


def resolve_old_evidence(out_dir, frozen_path):
    """Return the path holding the *pre-change* original evidence.

    For an already-extended frozen file the only admissible source is the
    pre-change backup; if it is missing this raises (never falls back to the
    extended file).  A pristine frozen file is its own original.
    """
    out_dir = OG.guard_dir(out_dir)
    backup_path = os.path.join(out_dir, PRECHANGE_NAME)
    if os.path.isfile(backup_path):
        return backup_path
    frozen = read_json(frozen_path)
    if detect_frozen_state(frozen) != 'pristine':
        raise ExtensionError(
            'pre-change backup missing for already-extended frozen evidence: '
            '%s' % backup_path)
    return frozen_path


def ensure_sidecar(out_dir, frozen_path, diff, aligned_body_pose_method=None,
                   now=None, replacement_log=None):
    """Return ``(sidecar, states)``; write the sidecar only, never the frozen file.

    Raises :class:`ExtensionError` on missing/mismatched backup for an
    already-extended frozen file, or on an unverifiable pre-change hash.
    """
    import time
    now = now or time.strftime('%Y-%m-%dT%H:%M:%S%z')
    out_dir = OG.guard_dir(out_dir)
    frozen = read_json(frozen_path)
    fstate = detect_frozen_state(frozen)
    backup_path = os.path.join(out_dir, PRECHANGE_NAME)
    sidecar_path = os.path.join(out_dir, SIDECAR_NAME)

    marker = frozen.get('_stage0_schema_extension') or {}
    recorded_pre = marker.get('prechange_sha256')
    sstate = detect_sidecar_state(sidecar_path)
    if recorded_pre is None and sstate == 'complete':
        recorded_pre = read_json(sidecar_path).get('prechange_sha256')

    if fstate == 'pristine':
        if not os.path.isfile(backup_path):
            with open(frozen_path, 'r', encoding='utf-8') as fh:
                frozen_text = fh.read()
            OG.write_text(backup_path, frozen_text, allow_existing=False)
        prechange_sha = sha256_file(backup_path)
        if sha256_file(frozen_path) != prechange_sha:
            raise ExtensionError(
                'pristine frozen file differs from its pre-change backup: %s'
                % frozen_path)
        postchange_sha = None
        bstate = 'valid'
    elif fstate in ('historically_extended', 'extended_without_marker'):
        if recorded_pre is None:
            raise ExtensionError(
                'historically extended frozen evidence has no recorded '
                'pre-change hash; cannot verify the backup')
        bstate = detect_backup_state(backup_path, recorded_pre)
        if bstate == 'missing':
            raise ExtensionError(
                'pre-change backup missing for already-extended frozen '
                'evidence: %s (refusing to reconstruct it from the extended '
                'file)' % backup_path)
        if bstate == 'mismatched':
            raise ExtensionError(
                'pre-change backup hash mismatch: recorded=%s actual=%s (%s)'
                % (recorded_pre, sha256_file(backup_path), backup_path))
        if bstate == 'unknown':
            raise ExtensionError(
                'pre-change backup present but no recorded hash to verify it')
        prechange_sha = recorded_pre
        postchange_sha = sha256_file(frozen_path)
    else:  # pragma: no cover - detect_frozen_state has no other value
        raise ExtensionError('unknown frozen state: %r' % fstate)

    if sstate == 'complete':
        sidecar = read_json(sidecar_path)
        recorded_post = sidecar.get('postchange_sha256')
        if recorded_post is None:
            # complete and "no postchange recorded" is self-contradictory: a
            # complete sidecar must pin the hash of the file it was built from.
            raise ExtensionError(
                'complete sidecar missing postchange_sha256: %s'
                % sidecar_path)
        actual_post = sha256_file(frozen_path)
        if recorded_post != actual_post:
            raise ExtensionError(
                'complete sidecar postchange hash mismatch: recorded=%s '
                'actual=%s (%s)' % (recorded_post, actual_post, frozen_path))
        return sidecar, {
            'frozen_state': fstate, 'backup_state': bstate,
            'sidecar_state': sstate, 'wrote_sidecar': False,
            'frozen_modified': False}

    method = (frozen.get('aligned_body_pose_method')
              or aligned_body_pose_method)
    if method is None:
        raise ExtensionError(
            'no aligned_body_pose_method available for the sidecar')
    created_at = now
    try:
        created_at = time.strftime(
            '%Y-%m-%dT%H:%M:%S%z', time.localtime(os.path.getmtime(frozen_path)))
    except OSError:
        pass
    sidecar = build_sidecar(
        frozen_path=frozen_path, backup_path=backup_path,
        prechange_sha=prechange_sha, postchange_sha=postchange_sha,
        extension_fields=EXTENSION_FIELDS, aligned_body_pose_method=method,
        diff=diff, frozen_state=fstate, backup_state=bstate,
        created_at=created_at, recovered_at=now)
    text = json.dumps(sidecar, indent=2, sort_keys=True, allow_nan=False) + '\n'
    OG.stage_and_replace(out_dir, SIDECAR_NAME, text,
                         replacement_log=replacement_log)
    return sidecar, {'frozen_state': fstate, 'backup_state': bstate,
                     'sidecar_state': 'complete', 'wrote_sidecar': True,
                     'frozen_modified': False}
