#!/usr/bin/env python3
"""Stage 1 fail-closed contract enforcement checker (R11 + R12 + R13, REV1.4).

Read-only contract instrumentation (Level 1/2 tooling): publishes nothing,
subscribes to nothing, launches nothing, modifies no frozen component. It
is NOT a new V-gate and does NOT change probe methodology; R6-R10 and the
R11 rules are enforced, never redefined.

R11 (machine-actionable contract):
  R11-A pending-validation propagation (Option A: value retained +
      derived_pending_validation; Option B: null + blocked).
  R11-B provenance completeness (every RFC § 3.10 mandatory field).

R12 (fail-closed enforcement):
  R12-A --smoke CLI over actual artifacts; missing artifact = FAIL.
  R12-B allowlist-only statuses (unknown status / missing status FAIL).
  R12-C RFC § 3.10 == Contract § 8 == checker list (field-set binding).
  R12-D raw storage belongs to a known stream category.

R13 (fail-closed schema hardening — closes the R12 audit bypasses B1-B4):
  B1 Option-A frame bypass: every composition-dependent field must be
      PRESENT in the emitted frame with a non-null value of the RFC
      shape (pose {position[3], orientation[4]}, vec3, zmp vec2) —
      manifest declaration must match the actual emitted artifact.
  B2 RFC <-> Contract semantic drift: per-field semantic markers
      (nullable / optional / type) are extracted from BOTH documents'
      provenance tables and must equal the canonical expectation
      (PROVENANCE_SEMANTICS); any add/remove/substitute of a semantic
      marker in either document FAILS the smoke path.
  B3 raw nested bypass: nested payload structure per the RFC/Contract
      retention contract (per-contact collision identity, contact-point
      arrays, JointWrench with both body sides incl. force/torque/
      force_offset; pose entries with name+position; 24-vector command);
      "not-a-wrench" / "not-a-pose-entry" FAIL. Unknown/optional native
      keys are preserved (lossless retention) and never rejected.
  B4 provenance type/range bypass: int-not-bool (gazebo_seed,
      frame_count), frame_count >= 0, finite t_start/t_end (no NaN/
      Infinity), t_end >= t_start.

CLI modes (R13 § 19):
  --smoke  THE artifact enforcement entry point: actual RFC + actual
           Collector Contract + actual artifacts -> single fail-closed
           chain -> PASS (exit 0) / FAIL (exit != 0). The acquisition
           workflow MUST use --smoke; the default mode is informational
           only and is NOT artifact enforcement.
  default  informational: document bindings + status maps.

Exit codes: 0 = PASS; 1 = FAIL; 2 = CLI usage error (argparse).
"""

import argparse
import json
import math
import os
import re
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(TOOLS)
RFC_PATH = os.path.join(WS, 'docs', 'STAGE1_DYNAMICS_DATASET_RFC.md')
CONTRACT_PATH = os.path.join(WS, 'docs', 'STAGE1_COLLECTOR_CONTRACT.md')

# --- R11-B/R12-C: mandatory provenance (MUST equal RFC § 3.10 AND Contract
# --- § 8; both are parsed — see check_provenance_binding) -------------------
MANDATORY_PROVENANCE_FIELDS = (
    'theta', 'theta_id', 'theta_hash', 'theta_list_hash', 'git_commit',
    'config_hash', 'cop_derivation_hash', 'gazebo_seed',
    'robot_model_version', 'world_model_version', 'gz_sim_version',
    'stage1_rfc_hash', 'acquisition_wallclock_utc', 'ring',
    'frame_count', 't_start', 't_end', 'episode_integrity_hash',
)

NULLABLE_PROVENANCE_FIELDS = ('cop_derivation_hash',)

# --- R13-B: canonical semantic expectations per provenance field -------------
# type markers per document side, extracted with _TYPE_MARKER_RULES from the
# CURRENT documents' own table text (None = the document carries no
# machine-readable type marker for that field; multi-field rows carry no
# per-field type). 'nullable': the row's rule text mentions null.
# 'optional': False for every field (RFC § 3.10 "mandatory, no exceptions").
# This constant is the checker's machine interpretation of the two documents;
# check_semantic_binding verifies both documents still declare exactly this.
PROVENANCE_SEMANTICS = {
    'theta':                {'rfc_type': 'object',  'contract_type': 'object',      'nullable': False},
    'theta_id':             {'rfc_type': None,      'contract_type': 'string',      'nullable': False},
    'theta_hash':           {'rfc_type': None,      'contract_type': 'hash',        'nullable': False},
    'theta_list_hash':      {'rfc_type': None,      'contract_type': None,          'nullable': False},
    'git_commit':           {'rfc_type': 'commit',  'contract_type': 'commit',      'nullable': False},
    'config_hash':          {'rfc_type': 'sha256',  'contract_type': None,          'nullable': False},
    'cop_derivation_hash':  {'rfc_type': 'sha256',  'contract_type': None,          'nullable': True},
    'gazebo_seed':          {'rfc_type': 'int',     'contract_type': 'int',         'nullable': False},
    'robot_model_version':  {'rfc_type': 'sha256',  'contract_type': 'sha256',      'nullable': False},
    'world_model_version':  {'rfc_type': 'sha256',  'contract_type': 'sha256',      'nullable': False},
    'gz_sim_version':       {'rfc_type': None,      'contract_type': None,          'nullable': False},
    'stage1_rfc_hash':      {'rfc_type': 'sha256',  'contract_type': 'sha256',      'nullable': False},
    'acquisition_wallclock_utc': {'rfc_type': 'string', 'contract_type': 'string',  'nullable': False},
    'ring':                 {'rfc_type': 'string',  'contract_type': 'string',      'nullable': False},
    'frame_count':          {'rfc_type': None,      'contract_type': 'int_nonneg',  'nullable': False},
    't_start':              {'rfc_type': None,      'contract_type': 'number',      'nullable': False},
    't_end':                {'rfc_type': None,      'contract_type': 'number',      'nullable': False},
    'episode_integrity_hash': {'rfc_type': 'sha256', 'contract_type': 'sha256',      'nullable': False},
}

# Type-marker extraction rules, ordered; applied to the rule/约束 cell of a
# single-field row. Marker names mirror the artifact validation types
# ('hash' = a canonical-JSON sha256 hash).
_TYPE_MARKER_RULES = (
    (re.compile(r'sha[-\s]?512', re.IGNORECASE), 'sha512'),
    (re.compile(r'sha[-\s]?256', re.IGNORECASE), 'sha256'),
    (re.compile(r'\bHEAD\b'), 'commit'),
    (re.compile(r'wall[-\s]?clock|UTC', re.IGNORECASE), 'string'),
    (re.compile(r'\bdict\b', re.IGNORECASE), 'object'),
    (re.compile(r'\bseed\b', re.IGNORECASE), 'int'),
    (re.compile('帧数'), 'int_nonneg'),
    (re.compile(r'\blabel\b', re.IGNORECASE), 'string'),
    (re.compile(r'\bid\b', re.IGNORECASE), 'string'),
    (re.compile(r'sim[ _]time', re.IGNORECASE), 'number'),
    (re.compile(r'\bhash\b', re.IGNORECASE), 'hash'),
)

# --- R13-D/R12: provenance artifact validation types (format/range only —
# --- no hash computation rules are defined or changed here) ------------------
PROVENANCE_SHA256_FIELDS = (
    'theta_hash', 'theta_list_hash', 'config_hash', 'cop_derivation_hash',
    'robot_model_version', 'world_model_version', 'stage1_rfc_hash',
    'episode_integrity_hash',
)
PROVENANCE_STRING_FIELDS = (
    'theta_id', 'gz_sim_version', 'acquisition_wallclock_utc', 'ring',
)
PROVENANCE_INT_FIELDS = ('gazebo_seed',)
PROVENANCE_INT_NONNEG_FIELDS = ('frame_count',)
PROVENANCE_NUMBER_FIELDS = ('t_start', 't_end')

_SHA256_RE = re.compile(r'\A[0-9a-f]{64}\Z')
_GIT_COMMIT_RE = re.compile(r'\A[0-9a-f]{40}\Z')

# --- R11-A/R12-B: validation-status vocabulary (no new enums) ---------------
STATUS_MEASURED = 'measured'
STATUS_DERIVED = 'derived'
STATUS_DERIVED_PENDING = 'derived_pending_validation'

COMPOSITION_STATUS_UNVERIFIED = 'UNVERIFIED'
WRENCH_FRAME_STATUS_UNKNOWN = 'UNKNOWN'

WRENCH_BLOCKED_STATUSES = ('null_blocked', 'blocked')

MEASURED_FIELDS = (
    'joint_position', 'joint_velocity', 'base_pose', 'pose_source_stamp',
    'left_contact', 'right_contact', 'imu_orientation',
    'imu_linear_acceleration', 'imu_angular_velocity', 'controller_command',
)

ORDINARY_DERIVED_FIELDS = (
    'joint_acceleration', 'base_velocity', 'base_angular_velocity',
    'gait_phase', 'desired_velocity',
)

COMPOSITION_DEPENDENT_FIELDS = (
    'left_foot_pose', 'right_foot_pose',
    'left_foot_velocity', 'right_foot_velocity',
    'com_position', 'com_velocity', 'com_acceleration', 'zmp_position',
)

WRENCH_FRAME_DEPENDENT_FIELDS = (
    'left_contact_force', 'right_contact_force',
    'left_contact_moment', 'right_contact_moment', 'cop_position',
)

VALID_FIELD_STATUSES = (STATUS_MEASURED, STATUS_DERIVED,
                        STATUS_DERIVED_PENDING)

# --- R13-A: Option-A frame shapes (RFC § 3.3-3.5, § 3.7) ---------------------
# pose = {position: float[3], orientation: float[4] (xyzw)};
# foot/com velocities, com position/velocity/acceleration = float[3];
# zmp_position = float[2].
OPTION_A_FIELD_SHAPES = {
    'left_foot_pose': 'pose',
    'right_foot_pose': 'pose',
    'left_foot_velocity': 'vec3',
    'right_foot_velocity': 'vec3',
    'com_position': 'vec3',
    'com_velocity': 'vec3',
    'com_acceleration': 'vec3',
    'zmp_position': 'vec2',
}

# --- R12-D/R13-C: raw storage structural contract ----------------------------
# Categories bound to Collector Contract § 6.1/§ 6.3 tokens. Required record
# keys; nested payloads are validated per the RFC retention contract
# (§ 3.6.1 contacts, § 3.3-3.5 pose entries, § 3.9 command vector).
# Unknown/optional native keys are preserved (lossless retention) and are
# NEVER rejected — only the required structure is enforced (R13 § 15).
RAW_STREAM_STRUCT = {
    'contacts_left': ('stamp_s', 'contacts'),
    'contacts_right': ('stamp_s', 'contacts'),
    'pose_info': ('stamp_s', 'entries'),
    'joint_command': ('stamp_s', 'command'),
    'joint_states': ('stamp_s', 'name'),
    'body_pose': ('stamp_s', 'position'),
}
REQUIRED_RAW_CATEGORIES = ('contacts_left', 'contacts_right', 'pose_info',
                           'joint_command')
RAW_CATEGORY_CONTRACT_TOKENS = {
    'contacts_left': '/nao/left_foot_contact_raw',
    'contacts_right': '/nao/right_foot_contact_raw',
    'pose_info': '/world/nao_world/pose/info',
    'joint_command': 'controller_command',
}
# RFC § 3.9: the command vector is float[24] (canonical JOINT_ORDER).
COMMAND_VECTOR_DIM = 24


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


def _pose_errors(value, label):
    if not isinstance(value, dict):
        return ['%s: pose must be an object {position, orientation} (got %s)'
                % (label, type(value).__name__)]
    errs = []
    if 'position' not in value:
        errs.append('%s: position missing' % label)
    else:
        errs.extend(_vec_errors(value['position'], 3, label + '.position'))
    if 'orientation' not in value:
        errs.append('%s: orientation missing' % label)
    else:
        errs.extend(_vec_errors(value['orientation'], 4,
                                label + '.orientation'))
    return errs


def _shape_errors(value, shape, label):
    if shape == 'pose':
        return _pose_errors(value, label)
    if shape == 'vec3':
        return _vec_errors(value, 3, label)
    if shape == 'vec2':
        return _vec_errors(value, 2, label)
    return ['%s: unknown shape %r' % (label, shape)]


# ---------------------------------------------------------------------------
# R12-C/R13-B: document parsing (field sets + semantic markers)
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
    """Parse the field names of the RFC § 3.10 provenance table."""
    fields = []
    for row_fields, _rule in _read_provenance_rows(rfc_path, 'rfc'):
        fields.extend(row_fields)
    return tuple(sorted(set(fields)))


def parse_contract_provenance_fields(contract_path=CONTRACT_PATH):
    """Parse the field names of the Collector Contract § 8 checklist table."""
    fields = []
    for row_fields, _rule in _read_provenance_rows(contract_path, 'contract'):
        fields.extend(row_fields)
    return tuple(sorted(set(fields)))


def extract_doc_provenance_semantics(doc_path, doc_kind):
    """R13-B: extract per-field semantic markers from a provenance table.

    Per row: nullable = the rule text mentions 'null'; optional = the rule
    text mentions 'optional'/'可选'; type = the first _TYPE_MARKER_RULES
    match in the rule text — ONLY for single-field rows (multi-field rows
    carry no per-field type in the current documents)."""
    semantics = {}
    for fields, rule in _read_provenance_rows(doc_path, doc_kind):
        nullable = 'null' in rule.lower()
        optional = bool(re.search(r'optional|可选', rule, re.IGNORECASE))
        type_marker = None
        if len(fields) == 1:
            for pattern, marker in _TYPE_MARKER_RULES:
                if pattern.search(rule):
                    type_marker = marker
                    break
        for f in fields:
            semantics[f] = {'nullable': nullable, 'optional': optional,
                            'type': type_marker}
    return semantics


def check_rfc_binding(rfc_path=RFC_PATH):
    """R11-B/R12-C: checker list == RFC § 3.10 table."""
    parsed = parse_rfc_provenance_fields(rfc_path)
    declared = tuple(sorted(MANDATORY_PROVENANCE_FIELDS))
    return {
        'ok': parsed == declared,
        'check': 'MANDATORY_PROVENANCE_FIELDS == RFC § 3.10 table',
        'rfc_path': rfc_path,
        'declared': declared,
        'parsed_from_rfc': parsed,
        'missing_in_checker': sorted(set(parsed) - set(declared)),
        'extra_in_checker': sorted(set(declared) - set(parsed)),
    }


def check_contract_binding(contract_path=CONTRACT_PATH):
    """R12-C: checker list == Collector Contract § 8 checklist table."""
    parsed = parse_contract_provenance_fields(contract_path)
    declared = tuple(sorted(MANDATORY_PROVENANCE_FIELDS))
    return {
        'ok': parsed == declared,
        'check': 'MANDATORY_PROVENANCE_FIELDS == Collector Contract § 8 '
                 'checklist table',
        'contract_path': contract_path,
        'declared': declared,
        'parsed_from_contract': parsed,
        'missing_in_checker': sorted(set(parsed) - set(declared)),
        'extra_in_checker': sorted(set(declared) - set(parsed)),
    }


def check_semantic_binding(rfc_path=RFC_PATH, contract_path=CONTRACT_PATH):
    """R13-B (closes audit B2): RFC § 3.10 <-> Contract § 8 SEMANTIC binding.

    For every mandatory field, the (nullable, optional, type-marker) tuple
    extracted from EACH document's table must equal the canonical
    expectation in PROVENANCE_SEMANTICS — a nullable, type, or
    required/optional mutation in either document (add, remove, or
    substitute) FAILS. Field-set drift is also reported here (and enforced
    by the R12-C name binding)."""
    rfc_sem = extract_doc_provenance_semantics(rfc_path, 'rfc')
    contract_sem = extract_doc_provenance_semantics(contract_path,
                                                    'contract')
    errors = []
    for field in MANDATORY_PROVENANCE_FIELDS:
        expected = PROVENANCE_SEMANTICS.get(field)
        if expected is None:
            errors.append('%s: no canonical semantic expectation '
                          '(checker constant drift)' % field)
            continue
        for side, sem in (('rfc', rfc_sem), ('contract', contract_sem)):
            got = sem.get(field)
            if got is None:
                errors.append('%s: %s document no longer carries the field '
                              '(field-set drift)' % (field, side))
                continue
            type_key = side + '_type'
            if got['type'] != expected[type_key]:
                errors.append(
                    '%s: %s type marker %r != expected %r (semantic drift)'
                    % (field, side, got['type'], expected[type_key]))
            if got['nullable'] != expected['nullable']:
                errors.append(
                    '%s: %s nullable %r != expected %r (semantic drift)'
                    % (field, side, got['nullable'], expected['nullable']))
            if got['optional'] != bool(expected.get('optional', False)):
                errors.append(
                    '%s: %s optional %r != expected %r (required/optional '
                    'drift — RFC § 3.10 is mandatory, no exceptions)'
                    % (field, side, got['optional'],
                       bool(expected.get('optional', False))))
    for side, sem in (('rfc', rfc_sem), ('contract', contract_sem)):
        for f in sorted(set(sem) - set(MANDATORY_PROVENANCE_FIELDS)):
            errors.append('%s: %s document carries a field outside the '
                          'mandatory set (field-set drift)' % (f, side))
    return {
        'ok': not errors,
        'check': 'R13-B semantic binding: RFC § 3.10 <-> Contract § 8 <-> '
                 'canonical (nullable / optional / type markers)',
        'errors': errors,
        'extracted': {'rfc': rfc_sem, 'contract': contract_sem},
    }


def check_provenance_binding(rfc_path=RFC_PATH, contract_path=CONTRACT_PATH):
    """R12-C + R13-B: field-set binding + semantic binding."""
    rfc = check_rfc_binding(rfc_path)
    contract = check_contract_binding(contract_path)
    semantic = check_semantic_binding(rfc_path, contract_path)
    equal = rfc['parsed_from_rfc'] == contract['parsed_from_contract']
    return {
        'ok': rfc['ok'] and contract['ok'] and equal and semantic['ok'],
        'check': 'RFC § 3.10 == Collector Contract § 8 == checker list '
                 '(field set) + semantic markers (R12-C/R13-B)',
        'rfc_binding': rfc,
        'contract_binding': contract,
        'semantic_binding': semantic,
        'rfc_vs_contract': {
            'equal': equal,
            'in_rfc_not_contract': sorted(set(rfc['parsed_from_rfc'])
                                          - set(contract[
                                              'parsed_from_contract'])),
            'in_contract_not_rfc': sorted(set(contract['parsed_from_contract'])
                                          - set(rfc['parsed_from_rfc'])),
        },
    }


def check_raw_category_binding(contract_path=CONTRACT_PATH):
    """R12-D/R12-C: every required raw category's contract token must exist."""
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
# R11-B/R12/R13-D: provenance completeness + type/range (fail-closed)
# ---------------------------------------------------------------------------

def check_provenance(episode_record):
    """R11-B/R12/R13-D: every RFC § 3.10 mandatory field present, non-null
    (except cop_derivation_hash), and type/range-valid per the Collector
    Contract § 8 constraints. R13-D closes the B4 bypass: booleans are not
    integers (gazebo_seed = true FAILS), frame_count >= 0, t_start/t_end
    are finite numbers (NaN/Infinity FAIL), and t_end >= t_start."""
    if not isinstance(episode_record, dict):
        return {
            'ok': False,
            'check': 'RFC § 3.10 mandatory provenance completeness '
                     '(R11-B/R13-D)',
            'missing': list(MANDATORY_PROVENANCE_FIELDS),
            'invalid_nonnull': [],
            'format_errors': ['provenance record is not a JSON object'],
            'note_unknown_fields_ignored': [],
        }
    missing = [f for f in MANDATORY_PROVENANCE_FIELDS
               if f not in episode_record]
    invalid = []
    for f in MANDATORY_PROVENANCE_FIELDS:
        if f in episode_record and f not in NULLABLE_PROVENANCE_FIELDS:
            v = episode_record[f]
            if v is None or v == '' or v == {}:
                invalid.append(f)
    errors = []
    for f in PROVENANCE_SHA256_FIELDS:
        if f not in episode_record:
            continue
        v = episode_record[f]
        if v is None and f in NULLABLE_PROVENANCE_FIELDS:
            continue
        if not isinstance(v, str) or not _SHA256_RE.fullmatch(v):
            errors.append('%s: %r is not a 64-hex lowercase sha256 string'
                          % (f, v))
    if 'git_commit' in episode_record:
        v = episode_record['git_commit']
        if not isinstance(v, str) or not _GIT_COMMIT_RE.fullmatch(v):
            errors.append('git_commit: %r is not a 40-hex commit string'
                          % (v,))
    if 'theta' in episode_record \
            and not isinstance(episode_record.get('theta'), dict):
        errors.append('theta: %r is not an object'
                      % (episode_record.get('theta'),))
    for f in PROVENANCE_STRING_FIELDS:
        if f in episode_record:
            v = episode_record[f]
            if not isinstance(v, str) or not v.strip():
                errors.append('%s: %r is not a non-empty string' % (f, v))
    # R13-D: booleans are NOT integers (bool is an int subclass in Python).
    for f in PROVENANCE_INT_FIELDS:
        if f in episode_record and not _is_int_not_bool(episode_record[f]):
            errors.append('%s: %r is not an integer (booleans rejected, '
                          'R13-D)' % (f, episode_record[f]))
    for f in PROVENANCE_INT_NONNEG_FIELDS:
        if f not in episode_record:
            continue
        v = episode_record[f]
        if not _is_int_not_bool(v):
            errors.append('%s: %r is not an integer (booleans rejected, '
                          'R13-D)' % (f, v))
        elif v < 0:
            errors.append('%s: %r is negative (R13-D)' % (f, v))
    for f in PROVENANCE_NUMBER_FIELDS:
        if f in episode_record:
            v = episode_record[f]
            if not _is_finite_number(v):
                errors.append('%s: %r is not a finite number (NaN/Infinity '
                              'rejected, R13-D)' % (f, v))
    if 't_start' in episode_record and 't_end' in episode_record:
        ts, te = episode_record['t_start'], episode_record['t_end']
        if _is_finite_number(ts) and _is_finite_number(te) and te < ts:
            errors.append('t_end %r < t_start %r (episode extents are '
                          'ordered, R13-D)' % (te, ts))
    unknown = sorted(set(episode_record) - set(MANDATORY_PROVENANCE_FIELDS))
    return {
        'ok': not missing and not invalid and not errors,
        'check': 'RFC § 3.10 mandatory provenance completeness + type/range '
                 '(R11-B/R12/R13-D)',
        'missing': missing,
        'invalid_nonnull': invalid,
        'format_errors': errors,
        'note_unknown_fields_ignored': unknown,
    }


# ---------------------------------------------------------------------------
# R11-A/R12-B: pending-validation propagation (allowlist-only, fail-closed)
# ---------------------------------------------------------------------------

def check_pending_propagation(manifest,
                              composition_status=COMPOSITION_STATUS_UNVERIFIED,
                              wrench_frame_status=WRENCH_FRAME_STATUS_UNKNOWN):
    """R11-A rule + R12-B fail-closed allowlists on a field-status manifest."""
    errors = []
    if composition_status != COMPOSITION_STATUS_UNVERIFIED:
        return {
            'ok': None,
            'check': 'R11-A pending-validation propagation',
            'errors': ['composition_status %r is not the current contract '
                       'state (UNVERIFIED); this checker does not certify '
                       'any other state' % composition_status],
        }
    for f in COMPOSITION_DEPENDENT_FIELDS:
        status = manifest.get(f)
        if status is None:
            errors.append('%s: status missing/not emitted '
                          '(composition-dependent field) — fail-closed, '
                          'not "nothing to check" (R12-B)' % f)
        elif status != STATUS_DERIVED_PENDING:
            errors.append(
                '%s: status %r is not %r while composition_status is '
                'UNVERIFIED — a field depending on unverified composition '
                'cannot be emitted as validated/ordinary derived data (R11-A)'
                % (f, status, STATUS_DERIVED_PENDING))
    if wrench_frame_status == WRENCH_FRAME_STATUS_UNKNOWN:
        for f in WRENCH_FRAME_DEPENDENT_FIELDS:
            status = manifest.get(f)
            if status is None:
                errors.append('%s: status missing/not emitted '
                              '(wrench-frame-dependent field) — fail-closed, '
                              'not "nothing to check" (R12-B)' % f)
            elif status not in WRENCH_BLOCKED_STATUSES:
                errors.append(
                    '%s: status %r is not an explicit blocked status (%s) '
                    'while the wrench frame/reference is UNKNOWN — unknown '
                    'or unrecognized statuses FAIL (R12-B Option-B bypass '
                    'closed)' % (f, status,
                                 ' / '.join(WRENCH_BLOCKED_STATUSES)))
    for f in MEASURED_FIELDS:
        status = manifest.get(f)
        if status is None:
            errors.append('%s: status missing/not emitted (measured field) '
                          '— fail-closed (R12-B)' % f)
        elif status != STATUS_MEASURED:
            errors.append('%s: measured field status %r != %r'
                          % (f, status, STATUS_MEASURED))
    for f in ORDINARY_DERIVED_FIELDS:
        status = manifest.get(f)
        if status is None:
            errors.append('%s: status missing/not emitted (ordinary derived '
                          'field) — fail-closed (R12-B)' % f)
        elif status not in (STATUS_DERIVED, STATUS_DERIVED_PENDING):
            errors.append('%s: ordinary derived field status %r invalid'
                          % (f, status))
    return {
        'ok': not errors,
        'check': 'R11-A pending-validation propagation + R12-B allowlists '
                 '(composition_status=%s, wrench_frame=%s)'
                 % (composition_status, wrench_frame_status),
        'errors': errors,
    }


def check_status_manifest(artifact):
    """R12-B: validate a structured status-manifest artifact."""
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
    elif comp != COMPOSITION_STATUS_UNVERIFIED:
        errors.append('manifest: composition_status %r is not %r; this '
                      'checker certifies no other state (R12-B)'
                      % (comp, COMPOSITION_STATUS_UNVERIFIED))
    wfs = artifact.get('wrench_frame_status')
    if wfs is None:
        errors.append('manifest: wrench_frame_status root declaration '
                      'missing — fail-closed (R12-B)')
    elif wfs != WRENCH_FRAME_STATUS_UNKNOWN:
        errors.append('manifest: wrench_frame_status %r is not %r; this '
                      'checker certifies no other state (R12-B)'
                      % (wfs, WRENCH_FRAME_STATUS_UNKNOWN))
    field_status = artifact.get('field_status')
    if not isinstance(field_status, dict):
        errors.append('manifest: field_status map missing or not an object '
                      '— fail-closed (R12-B)')
        field_status = {}
    known = (MEASURED_FIELDS + ORDINARY_DERIVED_FIELDS
             + COMPOSITION_DEPENDENT_FIELDS + WRENCH_FRAME_DEPENDENT_FIELDS)
    for key in sorted(field_status):
        if key not in known:
            errors.append('manifest field_status: unknown field %r — the '
                          'manifest is a status map over the schema fields '
                          '(R12-B fail-closed)' % key)
        elif field_status[key] not in VALID_FIELD_STATUSES + \
                WRENCH_BLOCKED_STATUSES:
            errors.append('manifest field_status[%s]: unrecognized status '
                          '%r (R12-B fail-closed)' % (key, field_status[key]))
    prop = check_pending_propagation(field_status)
    if prop['ok'] is not True:
        errors.extend(prop['errors'])
    return {
        'ok': not errors,
        'check': 'R12-B status manifest artifact (roots + field_status '
                 'allowlists)',
        'errors': errors,
    }


# ---------------------------------------------------------------------------
# R13-A/R12-B: frame emission (Option-A value/schema + Option-B nulls)
# ---------------------------------------------------------------------------

def check_frame_emission(frame, manifest=None,
                         composition_status=COMPOSITION_STATUS_UNVERIFIED,
                         wrench_frame_status=WRENCH_FRAME_STATUS_UNKNOWN):
    """R11-A/R12-B/R13-A emission check on one emitted sample frame.

    Option-A (composition-dependent) fields — R13-A closes audit B1: each
    field must be PRESENT with a NON-NULL value of the RFC shape (pose
    {position[3], orientation[4]}; vec3; zmp vec2). A manifest declaring
    derived_pending_validation while the frame is missing/null/malformed
    is a status/value mismatch and FAILS. Option-B fields must be PRESENT
    and null while the wrench frame/reference is UNKNOWN."""
    errors = []
    if not isinstance(frame, dict):
        return {
            'ok': False,
            'check': 'R13-A frame emission (Option-A schema + Option-B '
                     'nulls)',
            'errors': ['frame artifact is not a JSON object'],
        }
    for f, shape in sorted(OPTION_A_FIELD_SHAPES.items()):
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
        errors.extend(_shape_errors(v, shape, '%s (Option-A)' % f))
    if wrench_frame_status == WRENCH_FRAME_STATUS_UNKNOWN:
        for f in WRENCH_FRAME_DEPENDENT_FIELDS:
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
            wrench_frame_status=wrench_frame_status)
        if prop['ok'] is not True:
            errors.extend(prop['errors'])
    return {
        'ok': not errors,
        'check': 'R13-A frame emission (Option-A presence/type/shape + '
                 'Option-B nulls; composition_status=%s, wrench_frame=%s)'
                 % (composition_status, wrench_frame_status),
        'errors': errors,
    }


# ---------------------------------------------------------------------------
# R12-D/R13-C: raw storage structural contract (nested, fail-closed)
# ---------------------------------------------------------------------------

def _validate_contacts_record(rec, idx, stream, errors):
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
        for key in ('collision1', 'collision2'):
            v = contact.get(key)
            if not isinstance(v, str) or not v.strip():
                errors.append('%s.%s must be a non-empty string (collision '
                              'identity, RFC § 3.6.1)' % (label, key))
        arrays = {}
        for key in ('position', 'normal'):
            arr = contact.get(key)
            arrays[key] = arr
            if not isinstance(arr, list) or not arr:
                errors.append('%s.%s must be a non-empty list of contact '
                              'points (RFC § 3.6.1)' % (label, key))
                continue
            for k, pt in enumerate(arr):
                errors.extend(_vec_errors(pt, 3, '%s.%s[%d]'
                                          % (label, key, k)))
        depths = contact.get('depth')
        if not isinstance(depths, list) or not depths \
                or not all(_is_finite_number(d) for d in depths):
            errors.append('%s.depth must be a non-empty list of finite '
                          'numbers (RFC § 3.6.1)' % label)
            arrays['depth'] = None
        else:
            arrays['depth'] = depths
        if all(isinstance(arrays.get(k), list) and arrays[k]
               for k in ('position', 'normal', 'depth')):
            if not (len(arrays['position']) == len(arrays['normal'])
                    == len(arrays['depth'])):
                errors.append('%s: position/normal/depth per-contact-point '
                              'array lengths differ (R13-C shape)' % label)
        wrenches = contact.get('wrenches')
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
            for key in ('body_1_name', 'body_2_name'):
                if key not in w:
                    errors.append('%s.%s missing (body identity, RFC '
                                  '§ 3.6.1; verbatim — an empty string is '
                                  'a valid measured value)' % (wlabel, key))
                elif not isinstance(w[key], str):
                    errors.append('%s.%s must be a string' % (wlabel, key))
            for side in ('body_1_wrench', 'body_2_wrench'):
                wr = w.get(side)
                if not isinstance(wr, dict):
                    errors.append('%s.%s missing or not an object (both '
                                  'body sides are mandatory, RFC § 3.6.1)'
                                  % (wlabel, side))
                    continue
                for key in ('force', 'torque', 'force_offset'):
                    if key not in wr:
                        errors.append('%s.%s.%s missing (RFC § 3.6.1)'
                                      % (wlabel, side, key))
                    else:
                        errors.extend(_vec_errors(
                            wr[key], 3, '%s.%s.%s' % (wlabel, side, key)))


def _validate_pose_record(rec, idx, stream, errors):
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
        name = ent.get('name')
        if not isinstance(name, str) or not name.strip():
            errors.append('%s.name must be a non-empty string (entry '
                          'identity)' % label)
        if 'position' not in ent:
            errors.append('%s.position missing (pose fields)' % label)
        else:
            errors.extend(_vec_errors(ent['position'], 3,
                                      label + '.position'))
        if 'orientation' in ent:
            errors.extend(_vec_errors(ent['orientation'], 4,
                                      label + '.orientation'))
        if 'id' in ent and not _is_int_not_bool(ent['id']):
            errors.append('%s.id must be an integer' % label)


def _validate_joint_command_record(rec, idx, stream, errors):
    label = 'raw_records[%d] (%s)' % (idx, stream)
    cmd = rec.get('command')
    if not isinstance(cmd, list):
        errors.append('%s.command must be a list (R13-C)' % label)
        return
    if len(cmd) != COMMAND_VECTOR_DIM:
        errors.append('%s.command must have exactly %d elements (got %d) '
                      '— the NAO command vector is float[24] (RFC § 3.9)'
                      % (label, COMMAND_VECTOR_DIM, len(cmd)))
        return
    for j, v in enumerate(cmd):
        if not _is_finite_number(v):
            errors.append('%s.command[%d] = %r is not a finite number '
                          '(R13-C)' % (label, j, v))
            break


def _validate_joint_states_record(rec, idx, stream, errors):
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


def _validate_body_pose_record(rec, idx, stream, errors):
    label = 'raw_records[%d] (%s)' % (idx, stream)
    if 'position' not in rec:
        errors.append('%s.position missing' % label)
    else:
        errors.extend(_vec_errors(rec['position'], 3, label + '.position'))
    if 'orientation' in rec:
        errors.extend(_vec_errors(rec['orientation'], 4,
                                  label + '.orientation'))


_RAW_RECORD_VALIDATORS = {
    'contacts_left': _validate_contacts_record,
    'contacts_right': _validate_contacts_record,
    'pose_info': _validate_pose_record,
    'joint_command': _validate_joint_command_record,
    'joint_states': _validate_joint_states_record,
    'body_pose': _validate_body_pose_record,
}


def check_raw_storage(raw_records, composition_status=COMPOSITION_STATUS_UNVERIFIED,
                      required_categories=REQUIRED_RAW_CATEGORIES):
    """R12-D + R13-C: raw records must belong to a KNOWN raw stream category,
    carry the category's required keys, and satisfy the RFC/Contract nested
    retention structure (per-contact collision identity + contact-point
    arrays + both-side JointWrenches with force/torque/force_offset; pose
    entries with name+position; 24-vector command). Arbitrary unrelated
    JSON objects, "not-a-wrench"/"not-a-pose-entry" entries, and missing
    required streams FAIL. Unknown/optional native keys are preserved
    (lossless retention, R13 § 15) and never rejected. Structural validity
    only — no physics sanity checks (R13 § 14)."""
    errors = []
    if not isinstance(raw_records, list):
        return {
            'ok': False,
            'check': 'R12-D/R13-C raw storage structural contract',
            'n_records': 0,
            'errors': ['raw artifact is not a JSON array'],
            'present_categories': [],
        }
    present = set()
    for i, rec in enumerate(raw_records):
        if not isinstance(rec, dict):
            errors.append('raw_records[%d]: not an object' % i)
            continue
        stream = rec.get('stream')
        if stream not in RAW_STREAM_STRUCT:
            errors.append(
                'raw_records[%d]: unknown raw stream category %r — known '
                'categories: %s (arbitrary unrelated records are NOT raw '
                'storage, R12-D)' % (i, stream,
                                     ', '.join(sorted(RAW_STREAM_STRUCT))))
            continue
        present.add(stream)
        for key in RAW_STREAM_STRUCT[stream]:
            if key not in rec:
                errors.append('raw_records[%d] (%s): required key %r '
                              'missing (R12-D structural contract)'
                              % (i, stream, key))
        stamp = rec.get('stamp_s')
        if not _is_finite_number(stamp):
            errors.append('raw_records[%d] (%s): stamp_s %r is not a finite '
                          'number' % (i, stream, stamp))
        validator = _RAW_RECORD_VALIDATORS.get(stream)
        if validator is not None:
            validator(rec, i, stream, errors)
    for category in required_categories:
        if category not in present:
            errors.append('required raw stream category %r missing from the '
                          'raw artifact (R12-D)' % category)
    return {
        'ok': not errors,
        'check': 'R12-D/R13-C raw storage structural contract '
                 '(composition_status=%s; raw data unaffected per R11-A; '
                 'structural validity only, no physics checks)'
                 % composition_status,
        'n_records': len(raw_records),
        'present_categories': sorted(present),
        'required_categories': list(required_categories),
        'errors': errors,
    }


# ---------------------------------------------------------------------------
# R12-A/R13: the smoke path (actual artifacts -> all checks -> PASS/FAIL)
# ---------------------------------------------------------------------------

def load_json_artifact(path, label):
    """Load a JSON artifact; a missing/unreadable/invalid artifact is a
    FAILURE (never "nothing to check")."""
    if not path:
        return None, ['%s: artifact path not provided — fail-closed (R12-A)'
                      % label]
    if not os.path.isfile(path):
        return None, ['%s: artifact file %r not found — fail-closed (R12-A)'
                      % (label, path)]
    try:
        with open(path, 'r', encoding='utf-8') as fh:
            return json.load(fh), []
    except Exception as exc:  # noqa: BLE001 — reported, never fatal
        return None, ['%s: cannot load %r (%s: %s)'
                      % (label, path, type(exc).__name__, exc)]


def run_smoke(rfc_path=RFC_PATH, contract_path=CONTRACT_PATH,
              manifest_path=None, provenance_path=None, frame_path=None,
              raw_path=None):
    """R12-A/R13: the single fail-closed smoke validation path.

    Chain: field-set binding -> semantic binding (R13-B) -> raw-category
    binding -> provenance completeness/type/range (R13-D) -> status
    manifest (R12-B) -> frame emission (R13-A Option-A schema + Option-B
    nulls) -> raw storage nested structure (R13-C) -> overall PASS/FAIL.
    Any failure (or any missing artifact) => ok=False; the caller exits
    non-zero."""
    checks = {}
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
        checks['status_manifest'] = check_status_manifest(manifest)

    provenance, errs = load_json_artifact(provenance_path, 'provenance')
    if errs:
        load_errors.extend(errs)
        checks['provenance'] = {'ok': False, 'check':
                                'R11-B provenance completeness', 'errors':
                                errs}
    else:
        checks['provenance'] = check_provenance(provenance)

    frame, errs = load_json_artifact(frame_path, 'frame')
    if errs:
        load_errors.extend(errs)
        checks['frame_emission'] = {'ok': False, 'check':
                                    'R13-A frame emission', 'errors': errs}
    else:
        field_status = {}
        if isinstance(manifest, dict) \
                and isinstance(manifest.get('field_status'), dict):
            field_status = manifest['field_status']
        comp = manifest.get('composition_status') \
            if isinstance(manifest, dict) else None
        wfs = manifest.get('wrench_frame_status') \
            if isinstance(manifest, dict) else None
        checks['frame_emission'] = check_frame_emission(
            frame, manifest=field_status,
            composition_status=comp if comp is not None
            else COMPOSITION_STATUS_UNVERIFIED,
            wrench_frame_status=wfs if wfs is not None
            else WRENCH_FRAME_STATUS_UNKNOWN)

    raw, errs = load_json_artifact(raw_path, 'raw')
    if errs:
        load_errors.extend(errs)
        checks['raw_storage'] = {'ok': False, 'check':
                                 'R12-D/R13-C raw storage structural '
                                 'contract', 'errors': errs}
    else:
        checks['raw_storage'] = check_raw_storage(raw)

    failed_checks = [name for name, rep in checks.items()
                     if rep.get('ok') is not True]
    return {
        'smoke': {
            'ok': not failed_checks and not load_errors,
            'mode': 'R12-A/R13 fail-closed smoke over actual artifacts',
            'inputs': {
                'rfc': rfc_path, 'contract': contract_path,
                'manifest': manifest_path, 'provenance': provenance_path,
                'frame': frame_path, 'raw': raw_path,
            },
            'chain': ['provenance_binding (RFC § 3.10 == Contract § 8 == '
                      'checker, field set)',
                      'semantic_binding (R13-B: nullable/type/optional '
                      'markers, both documents)',
                      'raw_category_binding (Contract § 6 tokens)',
                      'provenance completeness + type/range (R13-D)',
                      'status manifest (R11-A/R12-B allowlists)',
                      'frame emission (R13-A Option-A schema + Option-B '
                      'nulls)',
                      'raw storage nested structure (R13-C)'],
            'checks': checks,
            'failed_checks': failed_checks,
        },
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Stage 1 fail-closed contract enforcement checker '
                    '(R11 + R12 + R13). Modes: --smoke is THE artifact '
                    'enforcement entry point (actual RFC + actual Collector '
                    'Contract + actual artifacts -> single fail-closed '
                    'chain -> exit 0 only on PASS; the acquisition workflow '
                    'MUST use --smoke). The default mode (no --smoke) is '
                    'INFORMATIONAL ONLY — document bindings and status '
                    'maps; it is NOT artifact enforcement.',
        epilog='Acquisition workflow note: only a --smoke PASS (exit 0) '
               'constitutes contract enforcement; the informational mode '
               'never validates artifacts.')
    parser.add_argument('--smoke', action='store_true',
                        help='R12-A/R13: run the full fail-closed smoke '
                             'chain over actual artifacts; exit 0 only on '
                             'PASS. THE enforcement entry point.')
    parser.add_argument('--rfc', default=RFC_PATH,
                        help='path to STAGE1_DYNAMICS_DATASET_RFC.md '
                             '(default: the repository RFC)')
    parser.add_argument('--contract', default=CONTRACT_PATH,
                        help='path to STAGE1_COLLECTOR_CONTRACT.md '
                             '(default: the repository contract)')
    parser.add_argument('--manifest', default=None,
                        help='status manifest artifact JSON (required for '
                             '--smoke): {composition_status, '
                             'wrench_frame_status, field_status}')
    parser.add_argument('--provenance', default=None,
                        help='episode provenance artifact JSON (required '
                             'for --smoke): every RFC § 3.10 field')
    parser.add_argument('--frame', default=None,
                        help='sample frame artifact JSON (required for '
                             '--smoke): every Option-A field present with '
                             'its RFC shape; Option-B fields present and '
                             'null')
    parser.add_argument('--raw', default=None,
                        help='raw records artifact JSON (required for '
                             '--smoke): [{stream, stamp_s, ...}, ...] with '
                             'RFC-shaped nested payloads')
    parser.add_argument('--check-rfc-binding', action='store_true',
                        help='informational: verify MANDATORY_PROVENANCE_'
                             'FIELDS equals the RFC § 3.10 table')
    parser.add_argument('--check-contract-binding', action='store_true',
                        help='informational: verify MANDATORY_PROVENANCE_'
                             'FIELDS equals the Collector Contract § 8 '
                             'table')
    parser.add_argument('--check-semantic-binding', action='store_true',
                        help='informational: verify the RFC § 3.10 <-> '
                             'Contract § 8 semantic markers (R13-B: '
                             'nullable/type/optional)')
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
    out['status_maps'] = {
        'measured': list(MEASURED_FIELDS),
        'ordinary_derived': list(ORDINARY_DERIVED_FIELDS),
        'composition_dependent_derived_pending': list(
            COMPOSITION_DEPENDENT_FIELDS),
        'wrench_frame_dependent_null_blocked': list(
            WRENCH_FRAME_DEPENDENT_FIELDS),
        'wrench_blocked_statuses': list(WRENCH_BLOCKED_STATUSES),
        'option_a_field_shapes': dict(OPTION_A_FIELD_SHAPES),
        'required_raw_categories': list(REQUIRED_RAW_CATEGORIES),
        'command_vector_dim': COMMAND_VECTOR_DIM,
        'composition_status': COMPOSITION_STATUS_UNVERIFIED,
        'wrench_frame_status': WRENCH_FRAME_STATUS_UNKNOWN,
    }
    print(json.dumps(out, indent=2))
    return 1 if binding_failed else 0


if __name__ == '__main__':
    sys.exit(main())
