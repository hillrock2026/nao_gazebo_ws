#!/usr/bin/env python3
"""Stage 1 fail-closed contract enforcement checker (R11-R15, REV1.5).

Read-only contract instrumentation (Level 1/2 tooling): publishes nothing,
subscribes to nothing, launches nothing, modifies no frozen component. It
is NOT a new V-gate and does NOT change probe methodology; R6-R10 and the
R11 rules are enforced, never redefined.

Normative schema source (R14): the checker OWNS NO INDEPENDENT SCHEMA.
All field lists, statuses, shapes, provenance semantics, raw-retention
requiredness, and the frame-artifact count rule are loaded at run time
from the normative machine-readable JSON block embedded in the RFC
(Appendix D); the Collector Contract embeds an identical copy (§ 8.4) and
the checker requires the two blocks to be canonically equal. Prose
keyword parsing has been removed entirely (R14/B2): semantics come from
the explicit required/nullable/type declarations of the block, and the
prose tables are checked for FIELD-NAME consistency only.

R14 closures (audit B1-B4):
  B1 Option-A shapes (pose position[3]/orientation[4], vec3, zmp vec2)
     come from the normative block — an RFC schema mutation makes
     previously-valid artifacts fail --smoke.
  B2 explicit required/nullable/type semantics; "non-null" prose can no
     longer be misread as nullable; requiredness is an explicit flag.
  B3 raw-retention requiredness fully bound per the block (Contact
     header, body_1_id/body_2_id, JointWrench members incl. both body
     sides with force/torque/force_offset, raw pose orientation,
     contact-point arrays, 24-vector command). Unknown native keys are
     preserved and never rejected (lossless retention).
  B4 the smoke frame artifact is an ARRAY of frame records and
     frame_count (provenance) must equal the actual record count; an
     empty episode (frame_count=0, frames=[]) is valid; frame_count
     stays int_nonneg, never forced > 0.

R15 closures (audit B5-B6):
  B5 (R15-A integrity pin): the authorized schema is pinned by version
     AND by the SHA-256 of the normative block's exact fenced byte span.
     BOTH the RFC Appendix D block and the Contract § 8.4 block must
     match the pin — so a SYNCHRONIZED unauthorized change to both
     documents fails even though the two copies still agree. Unknown
     schema versions are rejected, never auto-accepted. Changing the
     schema is a governed version upgrade: update the block in both
     documents, bump schema_version, recompute the pin, and update
     _AUTHORIZED_SCHEMA_* here plus the tests.
  B6 (R15-B uniqueness + strict JSON): exactly ONE normative ```json
     fenced block per document (a duplicate, a missing block, or
     malformed/duplicated fence markers fail — never a silent first/last
     match), and the block is parsed STRICTLY: duplicate JSON object
     keys at ANY depth and non-standard constants (NaN/Infinity) are
     rejections, not last-wins ambiguity. Artifacts load under the same
     strict rules.
  B5 (R15-C runtime invariant): frame_count == len(frames) is enforced
     UNCONDITIONALLY by check_frame_artifact — a checker runtime
     invariant, not a check conditional on the schema-declared rule
     (the authorized schema must still declare it; see
     _validate_schema_shape).

CLI modes (R13 § 19 / R14):
  --smoke  THE artifact enforcement entry point (actual RFC + actual
           Collector Contract + actual artifacts -> single fail-closed
           chain -> PASS exit 0 / FAIL exit != 0). The acquisition
           workflow MUST use --smoke.
  default  informational only — document bindings + status maps
           (derived from the loaded normative schema); NOT artifact
           enforcement.

Exit codes: 0 = PASS; 1 = FAIL; 2 = CLI usage error (argparse).
"""

import argparse
import hashlib
import json
import math
import os
import re
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(TOOLS)
RFC_PATH = os.path.join(WS, 'docs', 'STAGE1_DYNAMICS_DATASET_RFC.md')
CONTRACT_PATH = os.path.join(WS, 'docs', 'STAGE1_COLLECTOR_CONTRACT.md')

_SHA256_RE = re.compile(r'\A[0-9a-f]{64}\Z')
_GIT_COMMIT_RE = re.compile(r'\A[0-9a-f]{40}\Z')
_SCHEMA_VERSION_PREFIX = 'stage1-normative'


# ---------------------------------------------------------------------------
# R14/R15: normative schema loading (the checker's ONLY schema source)
# ---------------------------------------------------------------------------

# R15-A: the authorized normative schema. The pin is the SHA-256 of the
# normative block's exact fenced byte span (the lines strictly between
# the opening ```json fence line and the closing ``` fence line, read
# with universal-newline normalization and joined with '\n'). It covers
# the schema authorized at RFC REV1.5 (Appendix D) / Collector Contract
# v1.6 (§ 8.4) — both documents embed the byte-identical block.
# A schema change is a GOVERNED version upgrade, never auto-accepted:
# update the block in BOTH documents byte-identically, bump
# schema_version, recompute this SHA-256, and update these constants and
# the test expectations in the same review. Unknown versions or pin
# mismatches fail closed (R15-A/B5), even when the two copies agree.
_AUTHORIZED_SCHEMA_VERSION = 'stage1-normative-schema/1.0'
_AUTHORIZED_SCHEMA_SHA256 = (
    'eb620b233b3a4499cceaf23a1c1054a7c2693415e018874501b773d713b25241')


class _AmbiguousJsonError(ValueError):
    """Duplicate object key or non-standard constant in JSON text."""


def _reject_duplicate_keys(pairs):
    obj = {}
    for key, value in pairs:
        if key in obj:
            raise _AmbiguousJsonError(
                'duplicate JSON object key %r — last-wins (or first-wins) '
                'resolution is not an acceptable norm (R15-B)' % key)
        obj[key] = value
    return obj


def _reject_nonstandard_constant(name):
    raise _AmbiguousJsonError(
        'non-standard JSON constant %r (NaN/Infinity are not JSON, '
        'R15-B)' % name)


def _strict_json_loads(text):
    """Strict JSON (R15-B): duplicate object keys at ANY nesting depth and
    non-standard constants (NaN/Infinity/-Infinity) are parse failures
    instead of silently resolved values."""
    return json.loads(text, object_pairs_hook=_reject_duplicate_keys,
                      parse_constant=_reject_nonstandard_constant)


def _scan_fenced_regions(text):
    """CommonMark-style ```-fence pairing, line-based and deterministic.

    A fence line is a line whose stripped form starts with '```'. While a
    fence is open, only a BARE '```' line (no info string) closes it; a
    fence line carrying an info string (e.g. '```json') is CONTENT and
    cannot close. Returns (regions, errors); regions =
    [(info, content, open_line, close_line)] where content is the exact
    newline-normalized byte span between the two fence lines. A fence
    left open at EOF is a structural error (missing end marker,
    R15-B) — it is never silently ignored."""
    lines = text.split('\n')
    regions = []
    errors = []
    in_fence = False
    info = None
    content_start = None
    for index, raw in enumerate(lines):
        stripped = raw.strip()
        if not stripped.startswith('```'):
            continue
        if in_fence:
            if stripped == '```':
                regions.append((info, '\n'.join(lines[content_start:index]),
                                content_start + 1, index + 1))
                in_fence = False
                info = None
            # a fence line with an info string inside an open fence is
            # content — it cannot close (duplicate opening marker case)
        else:
            in_fence = True
            info = stripped[3:].strip()
            content_start = index + 1
    if in_fence:
        errors.append('unterminated code fence (missing closing "```") '
                      'opened at line %d (R15-B: marker integrity)'
                      % content_start)
    return regions, errors


def _find_normative_block(text):
    """Locate the UNIQUE normative schema block (R15-B: exactly one).

    Normative-relevant region = a fenced ```json region whose content
    text carries the stage1-normative version prefix. Ordinary example
    JSON blocks never carry the prefix and are never mistaken for the
    schema; a prefix found inside a non-json fence is a marker-order /
    fence-type error. Exactly one normative region must exist — a
    duplicate is a hard failure, never a silent first/last-match pick.
    The block text is STRICT-parsed: duplicate keys and non-standard
    constants are rejections (R15-B), and it must be a JSON object
    declaring a stage1-normative schema_version.

    Returns (schema, block_text, None) or (None, None, error)."""
    regions, fence_errors = _scan_fenced_regions(text)
    problems = list(fence_errors)
    candidates = []
    for info, content, open_line, close_line in regions:
        if _SCHEMA_VERSION_PREFIX not in content:
            continue
        if info == 'json':
            candidates.append(content)
        else:
            problems.append(
                'normative version prefix found inside a %s-fenced region '
                '(lines %d-%d) — the normative block must be a ```json '
                'fence; marker order or fence-type error (R15-B)'
                % ('```' + info if info else 'bare', open_line, close_line))
    if problems:
        return None, None, '; '.join(problems)
    if not candidates:
        return None, None, (
            'normative schema block (%s…) not found — exactly one fenced '
            '```json block carrying the version prefix is required '
            '(R15-B)' % _SCHEMA_VERSION_PREFIX)
    if len(candidates) > 1:
        return None, None, (
            '%d duplicate normative schema blocks found — exactly one is '
            'allowed; no silent first/last-match selection (R15-B)'
            % len(candidates))
    block_text = candidates[0]
    try:
        schema = _strict_json_loads(block_text)
    except _AmbiguousJsonError as exc:
        return None, None, ('normative schema block is ambiguous JSON: %s '
                            '(R15-B: duplicate keys / non-standard '
                            'constants are rejected, not resolved)'
                            % (exc,))
    except ValueError as exc:
        return None, None, ('normative schema block is not parseable '
                            'JSON: %s' % (exc,))
    if not isinstance(schema, dict) \
            or not str(schema.get('schema_version', '')).startswith(
                _SCHEMA_VERSION_PREFIX):
        return None, None, ('normative block does not declare a %s… '
                            'schema_version (R15-B)' % _SCHEMA_VERSION_PREFIX)
    return schema, block_text, None


def _validate_schema_shape(schema):
    """Structural sanity of the normative block itself (fail-closed: any
    error makes the schema unusable)."""
    errors = []
    for key in ('schema_version', 'status_semantics', 'frame_fields',
                'provenance_fields', 'raw_structures', 'raw_categories',
                'required_raw_categories', 'frame_artifact'):
        if key not in schema:
            errors.append('normative schema: %r section missing' % key)
    if errors:
        return errors
    sem = schema['status_semantics']
    for key in ('composition_status', 'wrench_frame_status',
                'wrench_blocked_statuses', 'valid_field_statuses'):
        if key not in sem:
            errors.append('status_semantics.%s missing' % key)
    names = []
    for spec in schema['provenance_fields']:
        if 'name' not in spec:
            errors.append('provenance field without a name: %r' % (spec,))
            continue
        names.append(spec['name'])
        for key in ('required', 'nullable', 'type'):
            if key not in spec:
                errors.append('provenance %s: %r missing (R14/B2 explicit '
                              'semantics)' % (spec['name'], key))
    if len(names) != len(set(names)):
        errors.append('provenance field names are not unique')
    for group in ('measured', 'ordinary_derived', 'option_a', 'option_b'):
        g = schema['frame_fields'].get(group)
        if not isinstance(g, dict) or 'status' not in g \
                or not isinstance(g.get('fields'), list):
            errors.append('frame_fields.%s malformed' % group)
            continue
        gnames = []
        for spec in g['fields']:
            if not isinstance(spec, dict) or 'name' not in spec:
                errors.append('frame_fields.%s: field entry without a '
                              'name: %r' % (group, spec))
                continue
            gnames.append(spec['name'])
        if len(gnames) != len(set(gnames)):
            errors.append('frame_fields.%s: duplicate field names '
                          '(R15-B — a duplicate field spec is ambiguous, '
                          'not a silently-deduplicated list)' % group)
    frame_artifact = schema['frame_artifact']
    if not isinstance(frame_artifact, dict) \
            or frame_artifact.get('count_must_equal') != 'frame_count':
        errors.append('frame_artifact.count_must_equal must declare '
                      '"frame_count" — the checker enforces frame_count == '
                      'len(frames) as an unconditional runtime invariant '
                      '(R15-C), and the authorized schema must declare the '
                      'same rule (consistency of code invariant and '
                      'pinned declaration)')
    for cat, struct in schema['raw_categories'].items():
        if struct not in schema['raw_structures']:
            errors.append('raw category %r references unknown structure %r'
                          % (cat, struct))
    return errors


def load_normative_schema(path):
    """Load + AUTHORIZE the normative machine-readable schema block from a
    document (R14 load + R15-A integrity pin).

    Returns (schema, errors); schema is None when the block is missing,
    non-unique, ambiguous/unparseable JSON, structurally invalid, of an
    unauthorized version, or its fenced byte span does not match the
    authorized integrity pin (fail-closed — a synchronized RFC+Contract
    change cannot bypass it, R15-A/B5)."""
    if not path or not os.path.isfile(path):
        return None, ['%s: document not found — fail-closed (R14)' % (path,)]
    with open(path, 'r', encoding='utf-8') as fh:
        text = fh.read()
    schema, block_text, err = _find_normative_block(text)
    if err is not None:
        return None, ['%s: %s' % (path, err)]
    errors = []
    version = schema.get('schema_version')
    if version != _AUTHORIZED_SCHEMA_VERSION:
        errors.append(
            '%s: schema_version %r is not the authorized %r (RFC REV1.5 '
            'Appendix D / Contract v1.6 § 8.4) — unknown/unauthorized '
            'schema versions are rejected, never auto-accepted (R15-A '
            'version policy)' % (path, version, _AUTHORIZED_SCHEMA_VERSION))
    block_sha = hashlib.sha256(block_text.encode('utf-8')).hexdigest()
    if block_sha != _AUTHORIZED_SCHEMA_SHA256:
        errors.append(
            '%s: normative block integrity pin mismatch — sha256(block '
            'bytes)=%s != authorized %s. An unauthorized schema change '
            'fails EVEN when the RFC and Contract copies agree (R15-A, '
            'audit B5); a schema change is a governed version upgrade '
            '(bump schema_version, update the pin and tests together)'
            % (path, block_sha, _AUTHORIZED_SCHEMA_SHA256))
    if errors:
        return None, errors
    errors = _validate_schema_shape(schema)
    if errors:
        return None, errors
    return schema, []


_SCHEMA, _SCHEMA_ERRORS = load_normative_schema(RFC_PATH)
if _SCHEMA is None:
    raise RuntimeError('cannot load the normative schema from %s: %s'
                       % (RFC_PATH, _SCHEMA_ERRORS))


def _schema_or_default(schema):
    return _SCHEMA if schema is None else schema


# --- Constants DERIVED from the normative schema (no independent list) ------

def _group_fields(schema, group):
    return tuple(f['name'] for f in
                 schema['frame_fields'][group]['fields'])


MANDATORY_PROVENANCE_FIELDS = tuple(sorted(
    f['name'] for f in _SCHEMA['provenance_fields']))
NULLABLE_PROVENANCE_FIELDS = tuple(
    f['name'] for f in _SCHEMA['provenance_fields'] if f.get('nullable'))
PROVENANCE_SHA256_FIELDS = tuple(
    f['name'] for f in _SCHEMA['provenance_fields']
    if f.get('type') == 'sha256')
PROVENANCE_STRING_FIELDS = tuple(
    f['name'] for f in _SCHEMA['provenance_fields']
    if f.get('type') == 'string')
PROVENANCE_INT_FIELDS = tuple(
    f['name'] for f in _SCHEMA['provenance_fields']
    if f.get('type') == 'int')
PROVENANCE_INT_NONNEG_FIELDS = tuple(
    f['name'] for f in _SCHEMA['provenance_fields']
    if f.get('type') == 'int_nonneg')
PROVENANCE_NUMBER_FIELDS = tuple(
    f['name'] for f in _SCHEMA['provenance_fields']
    if f.get('type') == 'number_finite')

STATUS_MEASURED = _SCHEMA['status_semantics']['valid_field_statuses'][0]
STATUS_DERIVED = _SCHEMA['status_semantics']['valid_field_statuses'][1]
STATUS_DERIVED_PENDING = \
    _SCHEMA['status_semantics']['valid_field_statuses'][2]
VALID_FIELD_STATUSES = tuple(
    _SCHEMA['status_semantics']['valid_field_statuses'])

COMPOSITION_STATUS_UNVERIFIED = \
    _SCHEMA['status_semantics']['composition_status']
WRENCH_FRAME_STATUS_UNKNOWN = \
    _SCHEMA['status_semantics']['wrench_frame_status']
WRENCH_BLOCKED_STATUSES = tuple(
    _SCHEMA['status_semantics']['wrench_blocked_statuses'])

MEASURED_FIELDS = _group_fields(_SCHEMA, 'measured')
ORDINARY_DERIVED_FIELDS = _group_fields(_SCHEMA, 'ordinary_derived')
COMPOSITION_DEPENDENT_FIELDS = _group_fields(_SCHEMA, 'option_a')
WRENCH_FRAME_DEPENDENT_FIELDS = _group_fields(_SCHEMA, 'option_b')
OPTION_A_FIELD_SHAPES = {f['name']: f
                         for f in _SCHEMA['frame_fields']['option_a'][
                             'fields']}

RAW_STREAM_STRUCT = {
    cat: tuple(
        _SCHEMA['raw_structures'][struct]['required_record_keys'])
    for cat, struct in _SCHEMA['raw_categories'].items()}
REQUIRED_RAW_CATEGORIES = tuple(
    _SCHEMA['required_raw_categories'])
COMMAND_VECTOR_DIM = _SCHEMA['raw_structures']['joint_command'][
    'command_length']

# R12-D legacy binding: required raw categories bound to Collector
# Contract § 6 text tokens (name-level binding, not schema semantics).
RAW_CATEGORY_CONTRACT_TOKENS = {
    'contacts_left': '/nao/left_foot_contact_raw',
    'contacts_right': '/nao/right_foot_contact_raw',
    'pose_info': '/world/nao_world/pose/info',
    'joint_command': 'controller_command',
}


# ---------------------------------------------------------------------------
# Small value helpers (R13: int-not-bool, finite numbers, vector shapes)
# ---------------------------------------------------------------------------

def _is_int_not_bool(v):
    return isinstance(v, int) and not isinstance(v, bool)


def _is_finite_number(v):
    return (isinstance(v, (int, float)) and not isinstance(v, bool)
            and math.isfinite(v))


def _vec_errors(value, n, label):
    if not isinstance(value, list):
        return ['%s: must be a list of %d finite numbers (got %s)'
                % (label, n, type(value).__name__)]
    if len(value) != n:
        return ['%s: must have %d elements (got %d)'
                % (label, n, len(value))]
    bad = [v for v in value if not _is_finite_number(v)]
    if bad:
        return ['%s: non-numeric or non-finite elements (e.g. %r)'
                % (label, bad[0])]
    return []


def _pose_errors(value, pos_n, ori_n, label):
    if not isinstance(value, dict):
        return ['%s: pose must be an object {position, orientation} (got %s)'
                % (label, type(value).__name__)]
    errs = []
    if 'position' not in value:
        errs.append('%s: position missing' % label)
    else:
        errs.extend(_vec_errors(value['position'], pos_n,
                                label + '.position'))
    if 'orientation' not in value:
        errs.append('%s: orientation missing' % label)
    else:
        errs.extend(_vec_errors(value['orientation'], ori_n,
                                label + '.orientation'))
    return errs


def _spec_shape_errors(value, spec, label):
    """Shape validation driven by a normative Option-A field spec."""
    ftype = spec.get('type')
    if ftype == 'pose':
        return _pose_errors(value, spec.get('position', 3),
                            spec.get('orientation', 4), label)
    if ftype == 'vector':
        return _vec_errors(value, spec.get('length', 3), label)
    return ['%s: unknown normative type %r' % (label, ftype)]


# ---------------------------------------------------------------------------
# Provenance-table NAME parsing (prose, name-level only — R14/B2: semantics
# are NEVER parsed from prose; they come from the normative block)
# ---------------------------------------------------------------------------

def _read_provenance_rows(doc_path, doc_kind):
    """Parse (fields, rule_text) per table row of the provenance tables.

    doc_kind 'rfc': rows | `field` | rule |  (RFC § 3.10).
    doc_kind 'contract': rows | N | `field` | rule |  (Contract § 8)."""
    with open(doc_path, 'r', encoding='utf-8') as fh:
        lines = fh.readlines()
    if doc_kind == 'rfc':
        head = '## 3.10 Provenance'
        field_cell, rule_cell = 1, 2
        numbered = False
    elif doc_kind == 'contract':
        head = '## 8.'
        field_cell, rule_cell = 2, 3
        numbered = True
    else:
        raise ValueError('doc_kind must be rfc or contract')
    start = None
    for i, line in enumerate(lines):
        if line.startswith(head):
            start = i
            break
    if start is None:
        raise ValueError('%s: heading %r not found' % (doc_path, head))
    rows = []
    seen_field = False
    for line in lines[start + 1:]:
        stripped = line.strip()
        if doc_kind == 'rfc':
            if stripped.startswith('---') and seen_field:
                break
        else:
            if stripped.startswith('## '):
                break
        if stripped.startswith('#'):
            break
        if not stripped.startswith('|'):
            continue
        cells = stripped.split('|')
        need = (rule_cell + 2) if numbered else (rule_cell + 1)
        if len(cells) < need:
            continue
        if numbered:
            if not re.fullmatch(r'\d+', cells[1].strip()):
                continue
        else:
            first = cells[1].strip()
            if first == 'Field' or re.fullmatch(r'-+', first):
                continue
        fields = re.findall(r'`([^`]+)`', cells[field_cell])
        if not fields:
            continue
        seen_field = True
        rows.append((fields, cells[rule_cell]))
    return rows


def parse_rfc_provenance_fields(rfc_path=RFC_PATH):
    """Field names of the RFC § 3.10 prose table (name-level only)."""
    fields = []
    for row_fields, _rule in _read_provenance_rows(rfc_path, 'rfc'):
        fields.extend(row_fields)
    return tuple(sorted(set(fields)))


def parse_contract_provenance_fields(contract_path=CONTRACT_PATH):
    """Field names of the Collector Contract § 8 prose checklist table."""
    fields = []
    for row_fields, _rule in _read_provenance_rows(contract_path,
                                                    'contract'):
        fields.extend(row_fields)
    return tuple(sorted(set(fields)))


# ---------------------------------------------------------------------------
# R12-C/R13-B/R14 bindings
# ---------------------------------------------------------------------------

def check_rfc_binding(rfc_path=RFC_PATH):
    """RFC internal consistency: § 3.10 prose table field names == the RFC's
    normative block provenance field names."""
    schema, errs = load_normative_schema(rfc_path)
    if schema is None:
        return {
            'ok': False,
            'check': 'RFC § 3.10 prose names == RFC Appendix D '
                     'provenance names',
            'rfc_path': rfc_path,
            'declared': [],
            'parsed_from_rfc': [],
            'missing_in_checker': [],
            'extra_in_checker': [],
            'errors': errs,
        }
    block_names = tuple(sorted(f['name'] for f in
                               schema['provenance_fields']))
    try:
        prose_names = parse_rfc_provenance_fields(rfc_path)
    except Exception as exc:  # noqa: BLE001 — reported
        return {
            'ok': False, 'check': 'RFC § 3.10 prose names == RFC Appendix D '
                                  'provenance names', 'rfc_path': rfc_path,
            'declared': list(block_names), 'parsed_from_rfc': [],
            'missing_in_checker': [], 'extra_in_checker': [],
            'errors': ['%s: %s' % (type(exc).__name__, exc)],
        }
    return {
        'ok': prose_names == block_names,
        'check': 'RFC § 3.10 prose names == RFC Appendix D provenance '
                 'names (name-level; semantics live in Appendix D)',
        'rfc_path': rfc_path,
        'declared': list(block_names),
        'parsed_from_rfc': list(prose_names),
        'missing_in_checker': sorted(set(prose_names) - set(block_names)),
        'extra_in_checker': sorted(set(block_names) - set(prose_names)),
    }


def check_contract_binding(contract_path=CONTRACT_PATH):
    """Contract internal consistency: § 8 prose checklist names == the
    Contract's normative block provenance field names."""
    schema, errs = load_normative_schema(contract_path)
    if schema is None:
        return {
            'ok': False,
            'check': 'Contract § 8 prose names == Contract § 8.4 normative '
                     'block names',
            'contract_path': contract_path,
            'declared': [], 'parsed_from_contract': [],
            'missing_in_checker': [], 'extra_in_checker': [],
            'errors': errs,
        }
    block_names = tuple(sorted(f['name'] for f in
                               schema['provenance_fields']))
    try:
        prose_names = parse_contract_provenance_fields(contract_path)
    except Exception as exc:  # noqa: BLE001 — reported
        return {
            'ok': False, 'check': 'Contract § 8 prose names == Contract '
                                  '§ 8.4 block names',
            'contract_path': contract_path, 'declared': list(block_names),
            'parsed_from_contract': [], 'missing_in_checker': [],
            'extra_in_checker': [],
            'errors': ['%s: %s' % (type(exc).__name__, exc)],
        }
    return {
        'ok': prose_names == block_names,
        'check': 'Contract § 8 prose names == Contract § 8.4 normative '
                 'block names (name-level)',
        'contract_path': contract_path,
        'declared': list(block_names),
        'parsed_from_contract': list(prose_names),
        'missing_in_checker': sorted(set(prose_names) - set(block_names)),
        'extra_in_checker': sorted(set(block_names) - set(prose_names)),
    }


def check_semantic_binding(rfc_path=RFC_PATH, contract_path=CONTRACT_PATH):
    """R14/B2 (replaces R13 prose-keyword binding): the RFC's Appendix D
    normative block and the Collector Contract's § 8.4 block must be
    CANONICALLY EQUAL. Any schema/semantic mutation (field add/remove,
    nullable, required/optional, type, shape, raw requiredness, count
    rule) in EITHER document is a validation failure. No prose is parsed
    for semantics."""
    rfc_schema, rfc_errs = load_normative_schema(rfc_path)
    contract_schema, contract_errs = load_normative_schema(contract_path)
    errors = list(rfc_errs) + list(contract_errs)
    if rfc_schema is None or contract_schema is None:
        return {
            'ok': False,
            'check': 'R14/B2: RFC Appendix D block == Contract § 8.4 block '
                     '(canonical equality; explicit machine semantics)',
            'errors': errors or ['normative schema block missing'],
        }
    rfc_canon = json.dumps(rfc_schema, sort_keys=True)
    contract_canon = json.dumps(contract_schema, sort_keys=True)
    if rfc_canon != contract_canon:
        errors.append('RFC Appendix D normative block != Collector Contract '
                      '§ 8.4 block (schema/semantic drift)')
        drift = {}
        for key in sorted(set(rfc_schema) | set(contract_schema)):
            r = json.dumps(rfc_schema.get(key), sort_keys=True)
            c = json.dumps(contract_schema.get(key), sort_keys=True)
            if r != c:
                drift[key] = {'rfc': rfc_schema.get(key),
                              'contract': contract_schema.get(key)}
        return {
            'ok': False,
            'check': 'R14/B2: RFC Appendix D block == Contract § 8.4 block '
                     '(canonical equality; explicit machine semantics)',
            'errors': errors,
            'drifting_sections': drift,
        }
    return {
        'ok': True,
        'check': 'R14/B2: RFC Appendix D block == Contract § 8.4 block '
                 '(canonical equality; explicit machine semantics)',
        'errors': [],
    }


def check_provenance_binding(rfc_path=RFC_PATH, contract_path=CONTRACT_PATH):
    """R12-C + R14: prose name bindings + normative block equality."""
    rfc = check_rfc_binding(rfc_path)
    contract = check_contract_binding(contract_path)
    semantic = check_semantic_binding(rfc_path, contract_path)
    return {
        'ok': rfc['ok'] and contract['ok'] and semantic['ok'],
        'check': 'RFC § 3.10 == Contract § 8 == normative block '
                 '(names) + RFC Appendix D == Contract § 8.4 (semantics, '
                 'R14)',
        'rfc_binding': rfc,
        'contract_binding': contract,
        'semantic_binding': semantic,
        'rfc_vs_contract': {
            'equal': rfc['parsed_from_rfc'] == contract[
                'parsed_from_contract'],
            'in_rfc_not_contract': sorted(set(rfc['parsed_from_rfc'])
                                          - set(contract[
                                              'parsed_from_contract'])),
            'in_contract_not_rfc': sorted(set(contract[
                'parsed_from_contract']) - set(rfc['parsed_from_rfc'])),
        },
    }


def check_raw_category_binding(contract_path=CONTRACT_PATH):
    """R12-D: every required raw category's contract token must exist."""
    with open(contract_path, 'r', encoding='utf-8') as fh:
        text = fh.read()
    start = text.find('## 6.')
    end = text.find('## 7.')
    section = text[start:end] if start >= 0 and end > start else ''
    missing = []
    for category, token in sorted(RAW_CATEGORY_CONTRACT_TOKENS.items()):
        if token not in section:
            missing.append({'category': category, 'token': token})
    return {
        'ok': not missing and bool(section),
        'check': 'required raw categories bound to Collector Contract § 6 '
                 'tokens (no invented streams)',
        'contract_path': contract_path,
        'required_categories': list(REQUIRED_RAW_CATEGORIES),
        'missing_bindings': missing,
    }


# ---------------------------------------------------------------------------
# R11-B/R12/R13-D/R14: provenance completeness + type/range (fail-closed)
# ---------------------------------------------------------------------------

def check_provenance(episode_record, schema=None):
    """R11-B/R12/R13-D/R14: every normative provenance field present,
    nullability per the block, and type/range-valid. Semantics are read
    from the normative schema (R14/B2 — no prose parsing)."""
    schema = _schema_or_default(schema)
    if not isinstance(episode_record, dict):
        return {
            'ok': False,
            'check': 'normative provenance completeness (R11-B/R14)',
            'missing': list(MANDATORY_PROVENANCE_FIELDS),
            'invalid_nonnull': [],
            'format_errors': ['provenance record is not a JSON object'],
            'note_unknown_fields_ignored': [],
        }
    missing = []
    invalid = []
    errors = []
    for spec in schema['provenance_fields']:
        name = spec['name']
        required = bool(spec.get('required', True))
        nullable = bool(spec.get('nullable', False))
        ftype = spec.get('type')
        if name not in episode_record:
            if required:
                missing.append(name)
            continue
        v = episode_record[name]
        if v is None:
            if not nullable:
                invalid.append(name)
            continue
        if ftype == 'object':
            if not isinstance(v, dict) or not v:
                errors.append('%s: %r is not a non-empty object'
                              % (name, v))
        elif ftype == 'string':
            if not isinstance(v, str) or not v.strip():
                errors.append('%s: %r is not a non-empty string' % (name, v))
        elif ftype == 'sha256':
            if not isinstance(v, str) or not _SHA256_RE.fullmatch(v):
                errors.append('%s: %r is not a 64-hex lowercase sha256 '
                              'string' % (name, v))
        elif ftype == 'commit40':
            if not isinstance(v, str) or not _GIT_COMMIT_RE.fullmatch(v):
                errors.append('%s: %r is not a 40-hex commit string'
                              % (name, v))
        elif ftype == 'int':
            if not _is_int_not_bool(v):
                errors.append('%s: %r is not an integer (booleans '
                              'rejected, R13-D)' % (name, v))
        elif ftype == 'int_nonneg':
            if not _is_int_not_bool(v):
                errors.append('%s: %r is not an integer (booleans '
                              'rejected, R13-D)' % (name, v))
            elif v < 0:
                errors.append('%s: %r is negative (R13-D)' % (name, v))
        elif ftype == 'number_finite':
            if not _is_finite_number(v):
                errors.append('%s: %r is not a finite number (NaN/Infinity '
                              'rejected, R13-D)' % (name, v))
        else:
            errors.append('%s: unknown normative type %r' % (name, ftype))
    # Cross-field extent rule (episode extents are ordered).
    ts, te = episode_record.get('t_start'), episode_record.get('t_end')
    if _is_finite_number(ts) and _is_finite_number(te) and te < ts:
        errors.append('t_end %r < t_start %r (episode extents are ordered, '
                      'R13-D)' % (te, ts))
    unknown = sorted(set(episode_record) - set(MANDATORY_PROVENANCE_FIELDS))
    return {
        'ok': not missing and not invalid and not errors,
        'check': 'normative provenance completeness + type/range '
                 '(R11-B/R12/R13-D/R14; semantics from Appendix D)',
        'missing': missing,
        'invalid_nonnull': invalid,
        'format_errors': errors,
        'note_unknown_fields_ignored': unknown,
    }


# ---------------------------------------------------------------------------
# R11-A/R12-B: pending-validation propagation (allowlist-only, fail-closed)
# ---------------------------------------------------------------------------

def check_pending_propagation(manifest,
                              composition_status=None,
                              wrench_frame_status=None,
                              schema=None):
    """R11-A rule + R12-B allowlists; groups and statuses from the
    normative schema."""
    schema = _schema_or_default(schema)
    sem = schema['status_semantics']
    if composition_status is None:
        composition_status = sem['composition_status']
    if wrench_frame_status is None:
        wrench_frame_status = sem['wrench_frame_status']
    groups = schema['frame_fields']
    errors = []
    if composition_status != sem['composition_status']:
        return {
            'ok': None,
            'check': 'R11-A pending-validation propagation',
            'errors': ['composition_status %r is not the current contract '
                       'state (%r); this checker does not certify any other '
                       'state' % (composition_status,
                                  sem['composition_status'])],
        }
    option_a_status = groups['option_a']['status']
    for f in _group_fields(schema, 'option_a'):
        status = manifest.get(f)
        if status is None:
            errors.append('%s: status missing/not emitted '
                          '(composition-dependent field) — fail-closed, '
                          'not "nothing to check" (R12-B)' % f)
        elif status != option_a_status:
            errors.append(
                '%s: status %r is not %r while composition_status is '
                'UNVERIFIED — a field depending on unverified composition '
                'cannot be emitted as validated/ordinary derived data (R11-A)'
                % (f, status, option_a_status))
    if wrench_frame_status == sem['wrench_frame_status']:
        blocked = tuple(sem['wrench_blocked_statuses'])
        for f in _group_fields(schema, 'option_b'):
            status = manifest.get(f)
            if status is None:
                errors.append('%s: status missing/not emitted '
                              '(wrench-frame-dependent field) — fail-closed, '
                              'not "nothing to check" (R12-B)' % f)
            elif status not in blocked:
                errors.append(
                    '%s: status %r is not an explicit blocked status (%s) '
                    'while the wrench frame/reference is UNKNOWN — unknown '
                    'or unrecognized statuses FAIL (R12-B Option-B bypass '
                    'closed)' % (f, status, ' / '.join(blocked)))
    for f in _group_fields(schema, 'measured'):
        status = manifest.get(f)
        if status is None:
            errors.append('%s: status missing/not emitted (measured field) '
                          '— fail-closed (R12-B)' % f)
        elif status != groups['measured']['status']:
            errors.append('%s: measured field status %r != %r'
                          % (f, status, groups['measured']['status']))
    ordinary_ok = (sem['valid_field_statuses'][1],
                   sem['valid_field_statuses'][2])
    for f in _group_fields(schema, 'ordinary_derived'):
        status = manifest.get(f)
        if status is None:
            errors.append('%s: status missing/not emitted (ordinary derived '
                          'field) — fail-closed (R12-B)' % f)
        elif status not in ordinary_ok:
            errors.append('%s: ordinary derived field status %r invalid'
                          % (f, status))
    return {
        'ok': not errors,
        'check': 'R11-A pending-validation propagation + R12-B allowlists '
                 '(composition_status=%s, wrench_frame=%s; groups from '
                 'Appendix D)'
                 % (composition_status, wrench_frame_status),
        'errors': errors,
    }


def check_status_manifest(artifact, schema=None):
    """R12-B: validate a structured status-manifest artifact (schema-driven
    groups, statuses, and root declarations)."""
    schema = _schema_or_default(schema)
    sem = schema['status_semantics']
    errors = []
    if not isinstance(artifact, dict):
        return {
            'ok': False,
            'check': 'R12-B status manifest artifact',
            'errors': ['manifest artifact is not a JSON object'],
        }
    comp = artifact.get('composition_status')
    if comp is None:
        errors.append('manifest: composition_status root declaration '
                      'missing — fail-closed (R12-B)')
    elif comp != sem['composition_status']:
        errors.append('manifest: composition_status %r is not %r; this '
                      'checker certifies no other state (R12-B)'
                      % (comp, sem['composition_status']))
    wfs = artifact.get('wrench_frame_status')
    if wfs is None:
        errors.append('manifest: wrench_frame_status root declaration '
                      'missing — fail-closed (R12-B)')
    elif wfs != sem['wrench_frame_status']:
        errors.append('manifest: wrench_frame_status %r is not %r; this '
                      'checker certifies no other state (R12-B)'
                      % (wfs, sem['wrench_frame_status']))
    field_status = artifact.get('field_status')
    if not isinstance(field_status, dict):
        errors.append('manifest: field_status map missing or not an object '
                      '— fail-closed (R12-B)')
        field_status = {}
    known = set()
    for group in ('measured', 'ordinary_derived', 'option_a', 'option_b'):
        known.update(_group_fields(schema, group))
    allowed_statuses = tuple(sem['valid_field_statuses']) + tuple(
        sem['wrench_blocked_statuses'])
    for key in sorted(field_status):
        if key not in known:
            errors.append('manifest field_status: unknown field %r — the '
                          'manifest is a status map over the schema fields '
                          '(R12-B fail-closed)' % key)
        elif field_status[key] not in allowed_statuses:
            errors.append('manifest field_status[%s]: unrecognized status '
                          '%r (R12-B fail-closed)' % (key, field_status[key]))
    prop = check_pending_propagation(field_status, schema=schema)
    if prop['ok'] is not True:
        errors.extend(prop['errors'])
    return {
        'ok': not errors,
        'check': 'R12-B status manifest artifact (roots + field_status '
                 'allowlists; groups from Appendix D)',
        'errors': errors,
    }


# ---------------------------------------------------------------------------
# R13-A/R14: frame emission (Option-A schema + Option-B nulls)
# ---------------------------------------------------------------------------

def check_frame_emission(frame, manifest=None,
                         composition_status=None,
                         wrench_frame_status=None,
                         schema=None):
    """R11-A/R12-B/R13-A/R14 emission check on ONE frame record.

    Option-A fields: present, non-null, shape per the NORMATIVE spec
    (R14/B1: shapes come from Appendix D, not checker hardcoding).
    Option-B fields: present and null while the wrench frame/reference is
    UNKNOWN."""
    schema = _schema_or_default(schema)
    sem = schema['status_semantics']
    if composition_status is None:
        composition_status = sem['composition_status']
    if wrench_frame_status is None:
        wrench_frame_status = sem['wrench_frame_status']
    errors = []
    if not isinstance(frame, dict):
        return {
            'ok': False,
            'check': 'R13-A/R14 frame emission (Option-A schema + '
                     'Option-B nulls)',
            'errors': ['frame record is not a JSON object'],
        }
    for spec in schema['frame_fields']['option_a']['fields']:
        f = spec['name']
        if f not in frame:
            errors.append(
                '%s: Option-A field missing from the frame — the manifest '
                'declaration must match the actual emitted artifact (R13-A)'
                % f)
            continue
        v = frame[f]
        if v is None:
            errors.append(
                '%s: Option-A value is null — Option A retains the value '
                'while composition_status is UNVERIFIED (R13-A)' % f)
            continue
        errors.extend(_spec_shape_errors(v, spec, '%s (Option-A)' % f))
    if wrench_frame_status == sem['wrench_frame_status']:
        for f in _group_fields(schema, 'option_b'):
            if f not in frame:
                errors.append(
                    '%s: not emitted — Option-B field must be PRESENT and '
                    'null while the wrench frame/reference is UNKNOWN '
                    '(R12-B fail-closed)' % f)
            elif frame[f] is not None:
                errors.append(
                    '%s: emitted a non-null value %r while the wrench '
                    'frame/reference is UNKNOWN — must be null + blocked '
                    '(Option B, R11-A/R12-B)' % (f, frame[f]))
    if manifest is not None:
        prop = check_pending_propagation(
            manifest, composition_status=composition_status,
            wrench_frame_status=wrench_frame_status, schema=schema)
        if prop['ok'] is not True:
            errors.extend(prop['errors'])
    return {
        'ok': not errors,
        'check': 'R13-A/R14 frame emission (Option-A presence/type/shape '
                 'from Appendix D + Option-B nulls; composition_status=%s, '
                 'wrench_frame=%s)'
                 % (composition_status, wrench_frame_status),
        'errors': errors,
    }


# ---------------------------------------------------------------------------
# R12-D/R13-C/R14: raw storage structural contract (schema-driven)
# ---------------------------------------------------------------------------

def _validate_contacts_record(rec, idx, stream, errors, schema):
    struct = schema['raw_structures']['contacts']
    contacts = rec.get('contacts')
    if not isinstance(contacts, list):
        errors.append('raw_records[%d] (%s): contacts must be a list (an '
                      'explicit EMPTY Contacts message is allowed; a '
                      'missing key is not) (R13-C)' % (idx, stream))
        return
    n = rec.get('n_contacts')
    if n is not None:
        if not _is_int_not_bool(n) or n < 0:
            errors.append('raw_records[%d] (%s): n_contacts %r must be an '
                          'integer >= 0 (R13-D)' % (idx, stream, n))
        elif n != len(contacts):
            errors.append('raw_records[%d] (%s): n_contacts %r != '
                          'len(contacts) %d (R13-C)'
                          % (idx, stream, n, len(contacts)))
    for j, contact in enumerate(contacts):
        label = 'raw_records[%d] (%s).contacts[%d]' % (idx, stream, j)
        if not isinstance(contact, dict):
            errors.append('%s: contact entry must be an object (R13-C)'
                          % label)
            continue
        for key in struct['contact_required_keys']:
            if key not in contact:
                errors.append('%s.%s missing (required raw-retention key, '
                              'R14/B3 — RFC § 3.6.1)' % (label, key))
        for key in ('collision1', 'collision2'):
            v = contact.get(key)
            if key in contact and (not isinstance(v, str) or not v.strip()):
                errors.append('%s.%s must be a non-empty string (collision '
                              'identity, RFC § 3.6.1)' % (label, key))
        header = contact.get('header')
        if 'header' in contact and not isinstance(header, dict):
            errors.append('%s.header must be an object (per-contact header '
                          'verbatim, RFC § 3.6.1)' % label)
        arrays = {}
        for key in ('position', 'normal'):
            arr = contact.get(key)
            arrays[key] = arr
            if key in contact:
                if not isinstance(arr, list) or not arr:
                    errors.append('%s.%s must be a non-empty list of '
                                  'contact points (RFC § 3.6.1)'
                                  % (label, key))
                    continue
                for k, pt in enumerate(arr):
                    errors.extend(_vec_errors(pt, 3, '%s.%s[%d]'
                                              % (label, key, k)))
        depths = contact.get('depth')
        arrays['depth'] = depths
        if 'depth' in contact:
            if not isinstance(depths, list) or not depths \
                    or not all(_is_finite_number(d) for d in depths):
                errors.append('%s.depth must be a non-empty list of finite '
                              'numbers (RFC § 3.6.1)' % label)
                arrays['depth'] = None
        if all(isinstance(arrays.get(k), list) and arrays[k]
               for k in ('position', 'normal', 'depth')):
            if not (len(arrays['position']) == len(arrays['normal'])
                    == len(arrays['depth'])):
                errors.append('%s: position/normal/depth per-contact-point '
                              'array lengths differ (R13-C shape)' % label)
        wrenches = contact.get('wrenches')
        if 'wrenches' in contact:
            if not isinstance(wrenches, list) or not wrenches:
                errors.append('%s.wrenches must be a non-empty list of '
                              'JointWrench objects (RFC § 3.6.1)' % label)
                continue
            for k, w in enumerate(wrenches):
                wlabel = '%s.wrenches[%d]' % (label, k)
                if not isinstance(w, dict):
                    errors.append('%s: must be an object, not %r (a '
                                  '"not-a-wrench" entry is rejected, R13-C)'
                                  % (wlabel, w))
                    continue
                for key in struct['wrench_required_keys']:
                    if key not in w:
                        errors.append('%s.%s missing (required JointWrench '
                                      'member, R14/B3 — RFC § 3.6.1)'
                                      % (wlabel, key))
                for key in ('body_1_name', 'body_2_name'):
                    if key in w and not isinstance(w[key], str):
                        errors.append('%s.%s must be a string (verbatim — '
                                      'an empty string is a valid measured '
                                      'value)' % (wlabel, key))
                for key in ('body_1_id', 'body_2_id'):
                    if key in w and not _is_int_not_bool(w[key]):
                        errors.append('%s.%s must be an integer (entity id)'
                                      % (wlabel, key))
                for side in ('body_1_wrench', 'body_2_wrench'):
                    wr = w.get(side)
                    if not isinstance(wr, dict):
                        if side in w:
                            errors.append('%s.%s must be an object (both '
                                          'body sides, RFC § 3.6.1)'
                                          % (wlabel, side))
                        continue
                    for key in struct['wrench_member_required_keys']:
                        if key not in wr:
                            errors.append('%s.%s.%s missing (R14/B3 — RFC '
                                          '§ 3.6.1)' % (wlabel, side, key))
                        else:
                            errors.extend(_vec_errors(
                                wr[key], 3, '%s.%s.%s' % (wlabel, side,
                                                          key)))


def _validate_pose_record(rec, idx, stream, errors, schema):
    struct = schema['raw_structures']['pose_info']
    entries = rec.get('entries')
    if not isinstance(entries, list) or not entries:
        errors.append('raw_records[%d] (%s): entries must be a non-empty '
                      'list of pose entries (R13-C)' % (idx, stream))
        return
    for j, ent in enumerate(entries):
        label = 'raw_records[%d] (%s).entries[%d]' % (idx, stream, j)
        if not isinstance(ent, dict):
            errors.append('%s: pose entry must be an object (a '
                          '"not-a-pose-entry" is rejected, R13-C)' % label)
            continue
        for key in struct['entry_required_keys']:
            if key not in ent:
                errors.append('%s.%s missing (required raw pose key, '
                              'R14/B3 — raw pose events retain the verbatim '
                              'Pose_V entry)' % (label, key))
        name = ent.get('name')
        if 'name' in ent and (not isinstance(name, str) or not name.strip()):
            errors.append('%s.name must be a non-empty string (entry '
                          'identity)' % label)
        if 'position' in ent:
            errors.extend(_vec_errors(ent['position'], 3,
                                      label + '.position'))
        if 'orientation' in ent:
            errors.extend(_vec_errors(ent['orientation'], 4,
                                      label + '.orientation'))
        if 'id' in ent and not _is_int_not_bool(ent['id']):
            errors.append('%s.id must be an integer' % label)


def _validate_joint_command_record(rec, idx, stream, errors, schema):
    label = 'raw_records[%d] (%s)' % (idx, stream)
    cmd = rec.get('command')
    if not isinstance(cmd, list):
        errors.append('%s.command must be a list (R13-C)' % label)
        return
    dim = schema['raw_structures']['joint_command']['command_length']
    if len(cmd) != dim:
        errors.append('%s.command must have exactly %d elements (got %d) '
                      '— the NAO command vector is float[%d] (RFC § 3.9 / '
                      'Appendix D)' % (label, dim, len(cmd), dim))
        return
    for j, v in enumerate(cmd):
        if not _is_finite_number(v):
            errors.append('%s.command[%d] = %r is not a finite number '
                          '(R13-C)' % (label, j, v))
            break


def _validate_joint_states_record(rec, idx, stream, errors, schema):
    label = 'raw_records[%d] (%s)' % (idx, stream)
    names = rec.get('name')
    if not isinstance(names, list) or not names \
            or not all(isinstance(n, str) and n for n in names):
        errors.append('%s.name must be a non-empty list of joint names'
                      % label)
        names = None
    for key in ('position', 'velocity'):
        if key in rec:
            arr = rec[key]
            if not isinstance(arr, list) \
                    or not all(_is_finite_number(v) for v in arr):
                errors.append('%s.%s must be a list of finite numbers'
                              % (label, key))
            elif names is not None and len(arr) != len(names):
                errors.append('%s.%s length %d != len(name) %d'
                              % (label, key, len(arr), len(names)))


def _validate_body_pose_record(rec, idx, stream, errors, schema):
    label = 'raw_records[%d] (%s)' % (idx, stream)
    if 'position' not in rec:
        errors.append('%s.position missing' % label)
    else:
        errors.extend(_vec_errors(rec['position'], 3, label + '.position'))
    if 'orientation' in rec:
        errors.extend(_vec_errors(rec['orientation'], 4,
                                  label + '.orientation'))


def check_raw_storage(raw_records, composition_status=None,
                      required_categories=None, schema=None):
    """R12-D + R13-C + R14/B3: raw records must belong to a KNOWN raw
    stream category (from Appendix D), carry the structure's required keys,
    and satisfy the nested retention structure. Required-ness comes from
    the normative block (Contact header, body ids, JointWrench members,
    raw pose orientation, ...); unknown/optional native keys are preserved
    and never rejected (lossless retention). Structural validity only —
    no physics sanity checks."""
    schema = _schema_or_default(schema)
    sem = schema['status_semantics']
    if composition_status is None:
        composition_status = sem['composition_status']
    if required_categories is None:
        required_categories = tuple(schema['required_raw_categories'])
    errors = []
    if not isinstance(raw_records, list):
        return {
            'ok': False,
            'check': 'R12-D/R13-C/R14 raw storage structural contract',
            'n_records': 0,
            'errors': ['raw artifact is not a JSON array'],
            'present_categories': [],
        }
    present = set()
    validators = {
        'contacts': _validate_contacts_record,
        'pose_info': _validate_pose_record,
        'joint_command': _validate_joint_command_record,
        'joint_states': _validate_joint_states_record,
        'body_pose': _validate_body_pose_record,
    }
    for i, rec in enumerate(raw_records):
        if not isinstance(rec, dict):
            errors.append('raw_records[%d]: not an object' % i)
            continue
        stream = rec.get('stream')
        if stream not in schema['raw_categories']:
            errors.append(
                'raw_records[%d]: unknown raw stream category %r — known '
                'categories: %s (arbitrary unrelated records are NOT raw '
                'storage, R12-D)' % (i, stream, ', '.join(sorted(
                    schema['raw_categories']))))
            continue
        present.add(stream)
        struct_name = schema['raw_categories'][stream]
        struct = schema['raw_structures'][struct_name]
        for key in struct['required_record_keys']:
            if key not in rec:
                errors.append('raw_records[%d] (%s): required key %r '
                              'missing (R12-D/R14 structural contract)'
                              % (i, stream, key))
        stamp = rec.get('stamp_s')
        if not _is_finite_number(stamp):
            errors.append('raw_records[%d] (%s): stamp_s %r is not a finite '
                          'number' % (i, stream, stamp))
        validator = validators.get(struct_name)
        if validator is not None:
            validator(rec, i, stream, errors, schema)
    for category in required_categories:
        if category not in present:
            errors.append('required raw stream category %r missing from the '
                          'raw artifact (R12-D)' % category)
    return {
        'ok': not errors,
        'check': 'R12-D/R13-C/R14 raw storage structural contract '
                 '(composition_status=%s; raw data unaffected per R11-A; '
                 'requiredness from Appendix D; structural validity only)'
                 % composition_status,
        'n_records': len(raw_records),
        'present_categories': sorted(present),
        'required_categories': list(required_categories),
        'errors': errors,
    }


# ---------------------------------------------------------------------------
# R12-A/R14: the smoke path (actual artifacts -> all checks -> PASS/FAIL)
# ---------------------------------------------------------------------------

def load_json_artifact(path, label):
    """Load a JSON artifact STRICTLY (R15-B): duplicate object keys at any
    nesting depth and non-standard constants (NaN/Infinity) are load
    failures, so an artifact can never rely on last-wins key resolution
    to smuggle a second value past a check. A missing/unreadable/invalid
    artifact is a FAILURE (never "nothing to check")."""
    if not path:
        return None, ['%s: artifact path not provided — fail-closed (R12-A)'
                      % label]
    if not os.path.isfile(path):
        return None, ['%s: artifact file %r not found — fail-closed (R12-A)'
                      % (label, path)]
    try:
        with open(path, 'r', encoding='utf-8') as fh:
            return _strict_json_loads(fh.read()), []
    except Exception as exc:  # noqa: BLE001 — reported, never fatal
        return None, ['%s: cannot load %r strictly (%s: %s)'
                      % (label, path, type(exc).__name__, exc)]


def check_frame_artifact(frames, provenance, manifest=None, schema=None,
                          composition_status=None,
                          wrench_frame_status=None):
    """R14/B4 + R15-C: the frame artifact is an ARRAY of frame records.
    Every record passes the R13-A/R14 emission check, and the provenance
    `frame_count` must equal the ACTUAL record count as an UNCONDITIONAL
    RUNTIME INVARIANT of this checker — the check is NOT conditional on
    any schema-declared rule, so a synchronized RFC+Contract schema
    mutation cannot remove it (audit B5; R15-C). An empty episode
    `frame_count=0` + `frames=[]` is valid — frame_count is never forced
    > 0; it must MATCH. frame_count must be an int (booleans/floats
    rejected) and non-negative."""
    schema = _schema_or_default(schema)
    errors = []
    if not isinstance(frames, list):
        return {
            'ok': False,
            'check': 'R14/B4 + R15-C frame artifact (array of frames + '
                     'count consistency)',
            'errors': ['frame artifact must be a JSON ARRAY of frame '
                       'records (R14/B4), got %s'
                       % type(frames).__name__],
        }
    field_status = manifest if isinstance(manifest, dict) else None
    for i, frame in enumerate(frames):
        rep = check_frame_emission(
            frame, manifest=field_status, schema=schema,
            composition_status=composition_status,
            wrench_frame_status=wrench_frame_status)
        for err in rep['errors']:
            errors.append('frame[%d]: %s' % (i, err))
    # R15-C (audit B5): frame_count == len(frames) is enforced by the
    # checker itself, unconditionally — never gated on the schema's
    # frame_artifact.count_must_equal declaration (the authorized schema
    # must declare the same rule; _validate_schema_shape cross-checks it).
    fc = provenance.get('frame_count') \
        if isinstance(provenance, dict) else None
    if not _is_int_not_bool(fc):
        errors.append('frame_count not available/valid in the provenance '
                      'artifact for the R15-C runtime count invariant (the '
                      'provenance check reports details)')
    elif fc < 0:
        errors.append('frame_count %d is negative — int_nonneg '
                      '(R13-D/R15-C)' % fc)
    elif len(frames) != fc:
        errors.append('frame_count %d != actual frame record count %d '
                      '(R14/B4 artifact consistency, enforced '
                      'unconditionally by the checker per R15-C)'
                      % (fc, len(frames)))
    return {
        'ok': not errors,
        'check': 'R14/B4 + R15-C frame artifact (array of frames; each '
                 'frame per R13-A/R14 emission rules; frame_count == '
                 'record count as an unconditional checker invariant; '
                 'empty episode allowed)',
        'n_frames': len(frames),
        'errors': errors,
    }


def run_smoke(rfc_path=RFC_PATH, contract_path=CONTRACT_PATH,
              manifest_path=None, provenance_path=None, frame_path=None,
              raw_path=None):
    """R12-A/R13/R14/R15: the single fail-closed smoke validation path.

    Chain: normative schema load + R15-A authorization (version + pin,
    from the given RFC) -> bindings (prose names + block equality) ->
    raw-category binding -> provenance -> status manifest -> frame
    artifact (array + UNCONDITIONAL count consistency, R15-C) -> raw
    storage. Any failure (or any missing artifact/schema, any unauthorized
    schema version, any pin mismatch) => ok=False; the caller exits
    non-zero. If the normative schema cannot be authorized from the given
    RFC, every schema-driven check FAILS (fail-closed — never a silent
    fallback to the default schema). Integrity errors are never caught
    and converted into a PASS."""
    checks = {}
    schema, schema_errs = load_normative_schema(rfc_path)
    if schema is None:
        checks['normative_schema'] = {
            'ok': False,
            'check': 'R14 normative schema load (RFC Appendix D)',
            'errors': schema_errs,
        }
        unavailable = ['normative schema unavailable from %r — fail-closed '
                       '(R14); no checks may silently pass' % rfc_path]
        for name in ('provenance_binding', 'raw_category_binding',
                     'provenance', 'status_manifest', 'frame_emission',
                     'raw_storage'):
            checks[name] = {'ok': False,
                            'check': name + ' (schema unavailable)',
                            'errors': list(unavailable)}
        failed = [name for name, rep in checks.items()
                  if rep.get('ok') is not True]
        return {
            'smoke': {
                'ok': False,
                'mode': 'R12-A/R13/R14 fail-closed smoke over actual '
                        'artifacts',
                'inputs': {'rfc': rfc_path, 'contract': contract_path,
                           'manifest': manifest_path,
                           'provenance': provenance_path,
                           'frame': frame_path, 'raw': raw_path},
                'chain': ['normative_schema (RFC Appendix D load)'],
                'checks': checks,
                'failed_checks': failed,
            },
        }
    checks['normative_schema'] = {
        'ok': True,
        'check': 'R14 normative schema load + R15-A authorization '
                 '(version + integrity pin) (RFC Appendix D)',
        'schema_version': schema.get('schema_version'),
        'authorized_version': _AUTHORIZED_SCHEMA_VERSION,
        'authorized_block_sha256': _AUTHORIZED_SCHEMA_SHA256,
    }
    load_errors = []

    checks['provenance_binding'] = check_provenance_binding(
        rfc_path, contract_path)
    checks['raw_category_binding'] = check_raw_category_binding(contract_path)

    manifest, errs = load_json_artifact(manifest_path, 'manifest')
    if errs:
        load_errors.extend(errs)
        checks['status_manifest'] = {'ok': False, 'check':
                                     'R12-B status manifest artifact',
                                     'errors': errs}
    else:
        checks['status_manifest'] = check_status_manifest(manifest,
                                                           schema=schema)

    provenance, errs = load_json_artifact(provenance_path, 'provenance')
    if errs:
        load_errors.extend(errs)
        checks['provenance'] = {'ok': False, 'check':
                                'R11-B provenance completeness', 'errors':
                                errs}
    else:
        checks['provenance'] = check_provenance(provenance, schema=schema)

    frame, errs = load_json_artifact(frame_path, 'frame')
    if errs:
        load_errors.extend(errs)
        checks['frame_emission'] = {'ok': False, 'check':
                                    'R13-A/R14 frame artifact', 'errors':
                                    errs}
    else:
        field_status = {}
        if isinstance(manifest, dict) \
                and isinstance(manifest.get('field_status'), dict):
            field_status = manifest['field_status']
        checks['frame_emission'] = check_frame_artifact(
            frame, provenance, manifest=field_status, schema=schema)

    raw, errs = load_json_artifact(raw_path, 'raw')
    if errs:
        load_errors.extend(errs)
        checks['raw_storage'] = {'ok': False, 'check':
                                 'R12-D/R13-C/R14 raw storage structural '
                                 'contract', 'errors': errs}
    else:
        checks['raw_storage'] = check_raw_storage(raw, schema=schema)

    failed_checks = [name for name, rep in checks.items()
                     if rep.get('ok') is not True]
    return {
        'smoke': {
            'ok': not failed_checks and not load_errors,
            'mode': 'R12-A/R13/R14 fail-closed smoke over actual artifacts',
            'inputs': {
                'rfc': rfc_path, 'contract': contract_path,
                'manifest': manifest_path, 'provenance': provenance_path,
                'frame': frame_path, 'raw': raw_path,
            },
            'chain': ['normative_schema (RFC Appendix D load + R15-A '
                      'version/pin authorization)',
                      'provenance_binding (prose names + RFC block == '
                      'Contract block + both blocks pin-authorized, R14/R15)',
                      'raw_category_binding (Contract § 6 tokens)',
                      'provenance completeness + type/range (schema-driven)',
                      'status manifest (schema-driven allowlists)',
                      'frame artifact (array; per-frame Option-A schema; '
                      'frame_count == record count UNCONDITIONALLY, '
                      'R14/B4 + R15-C)',
                      'raw storage nested structure (requiredness from '
                      'Appendix D, R14/B3)'],
            'checks': checks,
            'failed_checks': failed_checks,
        },
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Stage 1 fail-closed contract enforcement checker '
                    '(R11-R15). Modes: --smoke is THE artifact enforcement '
                    'entry point (actual RFC + actual Collector Contract + '
                    'actual artifacts -> single fail-closed chain -> exit 0 '
                    'only on PASS; the acquisition workflow MUST use '
                    '--smoke). The default mode (no --smoke) is '
                    'INFORMATIONAL ONLY — document bindings and the '
                    'normative-schema-derived status maps; it is NOT '
                    'artifact enforcement.',
        epilog='Acquisition workflow note: only a --smoke PASS (exit 0) '
               'constitutes contract enforcement; the informational mode '
               'never validates artifacts. The frame artifact is a JSON '
               'array of frame records; frame_count == len(frames) is an '
               'UNCONDITIONAL checker runtime invariant (R14/B4 + R15-C). '
               'The normative schema is only usable when it is the '
               'authorized version AND matches the integrity pin '
               '(R15-A) — a synchronized RFC/Contract schema change is '
               'not a bypass.')
    parser.add_argument('--smoke', action='store_true',
                        help='R12-A/R13/R14: run the full fail-closed '
                             'smoke chain over actual artifacts; exit 0 '
                             'only on PASS. THE enforcement entry point.')
    parser.add_argument('--rfc', default=RFC_PATH,
                        help='path to STAGE1_DYNAMICS_DATASET_RFC.md '
                             '(default: the repository RFC; the normative '
                             'schema is loaded from its Appendix D)')
    parser.add_argument('--contract', default=CONTRACT_PATH,
                        help='path to STAGE1_COLLECTOR_CONTRACT.md '
                             '(default: the repository contract; its § 8.4 '
                             'block must equal the RFC block)')
    parser.add_argument('--manifest', default=None,
                        help='status manifest artifact JSON (required for '
                             '--smoke): {composition_status, '
                             'wrench_frame_status, field_status}')
    parser.add_argument('--provenance', default=None,
                        help='episode provenance artifact JSON (required '
                             'for --smoke): every normative provenance '
                             'field; frame_count must equal the frame '
                             'artifact record count (R14/B4)')
    parser.add_argument('--frame', default=None,
                        help='frame artifact JSON (required for --smoke): '
                             'a JSON ARRAY of frame records, each with '
                             'every Option-A field present in its '
                             'Appendix-D shape and Option-B fields present '
                             'and null')
    parser.add_argument('--raw', default=None,
                        help='raw records artifact JSON (required for '
                             '--smoke): [{stream, stamp_s, ...}, ...] with '
                             'the Appendix-D nested retention structure')
    parser.add_argument('--check-rfc-binding', action='store_true',
                        help='informational: RFC § 3.10 prose names == RFC '
                             'Appendix D provenance names')
    parser.add_argument('--check-contract-binding', action='store_true',
                        help='informational: Contract § 8 prose names == '
                             'Contract § 8.4 block names')
    parser.add_argument('--check-semantic-binding', action='store_true',
                        help='informational: RFC Appendix D block == '
                             'Contract § 8.4 block (R14 canonical equality)')
    parser.add_argument('--json', action='store_true',
                        help='emit machine-readable JSON (default output '
                             'is JSON)')
    args = parser.parse_args(argv)

    if args.smoke:
        result = run_smoke(rfc_path=args.rfc, contract_path=args.contract,
                           manifest_path=args.manifest,
                           provenance_path=args.provenance,
                           frame_path=args.frame, raw_path=args.raw)
        print(json.dumps(result, indent=2))
        return 0 if result['smoke']['ok'] else 1

    out = {}
    binding_failed = False
    if args.check_rfc_binding:
        out['rfc_binding'] = check_rfc_binding(args.rfc)
        binding_failed = binding_failed or not out['rfc_binding']['ok']
    if args.check_contract_binding:
        out['contract_binding'] = check_contract_binding(args.contract)
        binding_failed = binding_failed or not out['contract_binding']['ok']
    if args.check_semantic_binding:
        out['semantic_binding'] = check_semantic_binding(
            args.rfc, args.contract)
        binding_failed = binding_failed or not out[
            'semantic_binding']['ok']
    if args.check_rfc_binding and args.check_contract_binding:
        out['provenance_binding'] = check_provenance_binding(
            args.rfc, args.contract)
        binding_failed = binding_failed or not out[
            'provenance_binding']['ok']
    out['mode'] = 'informational (NOT artifact enforcement — use --smoke)'
    out['normative_schema'] = {
        'source': RFC_PATH + ' Appendix D',
        'schema_version': _SCHEMA.get('schema_version'),
        'authorized_version': _AUTHORIZED_SCHEMA_VERSION,
        'authorized_block_sha256': _AUTHORIZED_SCHEMA_SHA256,
        'load_errors': _SCHEMA_ERRORS,
    }
    out['status_maps'] = {
        'measured': list(MEASURED_FIELDS),
        'ordinary_derived': list(ORDINARY_DERIVED_FIELDS),
        'composition_dependent_derived_pending': list(
            COMPOSITION_DEPENDENT_FIELDS),
        'wrench_frame_dependent_null_blocked': list(
            WRENCH_FRAME_DEPENDENT_FIELDS),
        'option_a_field_specs': {
            f['name']: {k: v for k, v in f.items() if k != 'name'}
            for f in _SCHEMA['frame_fields']['option_a']['fields']},
        'wrench_blocked_statuses': list(WRENCH_BLOCKED_STATUSES),
        'required_raw_categories': list(REQUIRED_RAW_CATEGORIES),
        'command_vector_dim': COMMAND_VECTOR_DIM,
        'composition_status': COMPOSITION_STATUS_UNVERIFIED,
        'wrench_frame_status': WRENCH_FRAME_STATUS_UNKNOWN,
    }
    print(json.dumps(out, indent=2))
    return 1 if binding_failed else 0


if __name__ == '__main__':
    sys.exit(main())
