#!/usr/bin/env python3
"""Stage 1 fail-closed contract enforcement checker (R11 + R12, REV1.4).

Read-only contract instrumentation (Level 1/2 tooling): publishes nothing,
subscribes to nothing, launches nothing, modifies no frozen component. It
is NOT a new V-gate and does NOT change probe methodology; R6-R10 and the
R11 rules are enforced, never redefined.

R11 (machine-actionable contract):
  R11-A pending-validation propagation: derived fields depending on the
      UNVERIFIED composition must be derived_pending_validation (Option A:
      value retained); wrench-frame-dependent fields must be null + blocked
      (Option B). Raw measured fields are unaffected.
  R11-B provenance completeness: every RFC § 3.10 mandatory field must be
      emitted; MANDATORY_PROVENANCE_FIELDS is bound to the RFC § 3.10 table
      by parsing.

R12 (fail-closed enforcement — this revision):
  R12-A actual artifact enforcement: a real smoke CLI consumes the actual
      RFC, the actual Collector Contract, and actual artifact files
      (status manifest, provenance record, sample frame, raw records) and
      exits non-zero on ANY failure. Missing artifacts are FAILURES, not
      "nothing to check".
  R12-B fail-closed pending propagation: status checking is allowlist-only
      — unknown/unrecognized statuses (e.g. cop_position = null with an
      arbitrary status string) FAIL; a missing manifest, missing root
      status declaration, or missing field status FAILS.
  R12-C RFC <-> Collector Contract binding: the Collector Contract § 8
      provenance checklist is machine-parsed and must EQUAL the RFC § 3.10
      table and the checker list; drift in either document is a failure.
      The required raw-storage categories are bound to Collector Contract
      § 6 tokens by text presence.
  R12-D raw storage structural contract: raw records must belong to a
      known raw stream category and carry the category's required keys —
      an arbitrary unrelated JSON object is NOT valid raw storage.

Smoke CLI (single fail-closed path; R12-A):

    python3 tools/stage1_contract_check.py --smoke \\
        --manifest <status_manifest.json> \\
        --provenance <episode_provenance.json> \\
        --frame <sample_frame.json> \\
        --raw <raw_records.json> \\
        [--rfc <RFC.md>] [--contract <CONTRACT.md>] [--json]

Artifact formats (synthetic fixtures; NOT acquisition data):

  status manifest: {"composition_status": "UNVERIFIED",
                    "wrench_frame_status": "UNKNOWN",
                    "field_status": {<schema field>: <status>, ...}}
  provenance:      {<every RFC § 3.10 field>: <value, ...>}
  sample frame:    {<field>: <value>} — every wrench-frame-dependent field
                   MUST be present and null (Option B); composition fields
                   may carry values (Option A; statuses live in the
                   manifest).
  raw records:     [{"stream": <category>, "stamp_s": <float>, ...}, ...]
                   — required categories: contacts_left, contacts_right,
                   pose_info, joint_command (bound to Collector Contract
                   § 6.1/§ 6.3); joint_states/body_pose are valid optional
                   categories.

Exit codes: 0 = smoke PASS (or binding-only checks PASS); 1 = FAIL;
2 = CLI usage error (argparse).
"""

import argparse
import json
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

# Fields whose value may legitimately be null (RFC § 3.10: "null until
# then" — the FIELD must still be present).
NULLABLE_PROVENANCE_FIELDS = ('cop_derivation_hash',)

# Provenance format contract (Collector Contract § 8 checklist constraints;
# format-only — no hash computation rules are defined or changed here).
PROVENANCE_SHA256_FIELDS = (
    'theta_hash', 'theta_list_hash', 'config_hash', 'cop_derivation_hash',
    'robot_model_version', 'world_model_version', 'stage1_rfc_hash',
    'episode_integrity_hash',
)
PROVENANCE_STRING_FIELDS = (
    'theta_id', 'gz_sim_version', 'acquisition_wallclock_utc', 'ring',
)
PROVENANCE_INT_FIELDS = ('gazebo_seed', 'frame_count')
PROVENANCE_NUMBER_FIELDS = ('t_start', 't_end')

_SHA256_RE = re.compile(r'\A[0-9a-f]{64}\Z')
_GIT_COMMIT_RE = re.compile(r'\A[0-9a-f]{40}\Z')

# --- R11-A/R12-B: validation-status vocabulary (no new enums) ---------------
STATUS_MEASURED = 'measured'
STATUS_DERIVED = 'derived'
STATUS_DERIVED_PENDING = 'derived_pending_validation'

COMPOSITION_STATUS_UNVERIFIED = 'UNVERIFIED'
WRENCH_FRAME_STATUS_UNKNOWN = 'UNKNOWN'

# R12-B: allowlist-only. A wrench-frame-dependent field may ONLY carry an
# explicit blocked status (Option B); anything else — including unknown or
# unrecognized strings — FAILS.
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

# --- R12-D: raw storage structural contract ---------------------------------
# Categories bound to Collector Contract § 6.1/§ 6.3 tokens (see
# RAW_CATEGORY_CONTRACT_TOKENS + check_raw_category_binding). No new raw
# stream is invented; joint_states/body_pose are valid optional categories
# (Collector Contract § 2 stream inventory).
RAW_STREAM_STRUCT = {
    'contacts_left': ('stamp_s', 'collision1', 'collision2', 'wrenches'),
    'contacts_right': ('stamp_s', 'collision1', 'collision2', 'wrenches'),
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


# ---------------------------------------------------------------------------
# R12-C: document parsing (RFC § 3.10 table, Collector Contract § 8 table)
# ---------------------------------------------------------------------------

def parse_rfc_provenance_fields(rfc_path=RFC_PATH):
    """Parse the field names of the RFC § 3.10 provenance table."""
    with open(rfc_path, 'r', encoding='utf-8') as fh:
        lines = fh.readlines()
    start = None
    for i, line in enumerate(lines):
        if line.startswith('## 3.10 Provenance'):
            start = i
            break
    if start is None:
        raise ValueError('RFC %s: § 3.10 heading not found' % rfc_path)
    fields = []
    for line in lines[start + 1:]:
        stripped = line.strip()
        if stripped.startswith('---') and fields:
            break
        if stripped.startswith('#'):
            break
        if not stripped.startswith('|'):
            continue
        cells = stripped.split('|')
        if len(cells) < 3:
            continue
        first_cell = cells[1]
        if first_cell.strip() == 'Field' \
                or re.fullmatch(r'-+', first_cell.strip()):
            continue
        fields.extend(re.findall(r'`([^`]+)`', first_cell))
    return tuple(sorted(set(fields)))


def parse_contract_provenance_fields(contract_path=CONTRACT_PATH):
    """Parse the field names of the Collector Contract § 8 checklist table.

    § 8 rows look like:  | 1 | `theta` | 约束 |  — the '#' cell must be a
    number; field names are the backticked tokens of the Field cell."""
    with open(contract_path, 'r', encoding='utf-8') as fh:
        lines = fh.readlines()
    start = None
    for i, line in enumerate(lines):
        if line.startswith('## 8.'):
            start = i
            break
    if start is None:
        raise ValueError('Contract %s: § 8 heading not found' % contract_path)
    fields = []
    for line in lines[start + 1:]:
        stripped = line.strip()
        if stripped.startswith('## '):
            break
        if not stripped.startswith('|'):
            continue
        cells = stripped.split('|')
        if len(cells) < 4:
            continue
        num_cell = cells[1].strip()
        if not re.fullmatch(r'\d+', num_cell):
            continue
        fields.extend(re.findall(r'`([^`]+)`', cells[2]))
    return tuple(sorted(set(fields)))


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


def check_provenance_binding(rfc_path=RFC_PATH, contract_path=CONTRACT_PATH):
    """R12-C: RFC § 3.10 == Collector Contract § 8 == checker list.

    Proves the two documents are not two independent declarations."""
    rfc = check_rfc_binding(rfc_path)
    contract = check_contract_binding(contract_path)
    equal = rfc['parsed_from_rfc'] == contract['parsed_from_contract']
    return {
        'ok': rfc['ok'] and contract['ok'] and equal,
        'check': 'RFC § 3.10 == Collector Contract § 8 == checker list '
                 '(R12-C binding)',
        'rfc_binding': rfc,
        'contract_binding': contract,
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
    """R12-D/R12-C: every required raw category's contract token must exist.

    Binds the required raw stream categories to Collector Contract § 6
    text: if the contract drops a stream, the checker's requirement is
    detectably unbound and this check FAILS (no invented raw streams)."""
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
# R11-B/R12: provenance completeness + format (fail-closed)
# ---------------------------------------------------------------------------

def check_provenance(episode_record):
    """R11-B/R12 smoke check: every RFC § 3.10 mandatory field present,
    non-null (except cop_derivation_hash), and format-valid per the
    Collector Contract § 8 constraints (format only — no hash computation
    rules are defined here)."""
    if not isinstance(episode_record, dict):
        return {
            'ok': False,
            'check': 'RFC § 3.10 mandatory provenance completeness (R11-B)',
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
    v = episode_record.get('git_commit')
    if 'git_commit' in episode_record \
            and (not isinstance(v, str)
                 or not _GIT_COMMIT_RE.fullmatch(v)):
        errors.append('git_commit: %r is not a 40-hex commit string' % (v,))
    if 'theta' in episode_record \
            and not isinstance(episode_record.get('theta'), dict):
        errors.append('theta: %r is not an object' % (episode_record.get(
            'theta'),))
    for f in PROVENANCE_STRING_FIELDS:
        if f in episode_record:
            v = episode_record[f]
            if not isinstance(v, str) or not v.strip():
                errors.append('%s: %r is not a non-empty string' % (f, v))
    for f in PROVENANCE_INT_FIELDS:
        if f in episode_record and not isinstance(episode_record[f], int):
            errors.append('%s: %r is not an integer' % (f, episode_record[f]))
    for f in PROVENANCE_NUMBER_FIELDS:
        if f in episode_record \
                and not isinstance(episode_record[f], (int, float)):
            errors.append('%s: %r is not a number' % (f, episode_record[f]))
    unknown = sorted(set(episode_record) - set(MANDATORY_PROVENANCE_FIELDS))
    return {
        'ok': not missing and not invalid and not errors,
        'check': 'RFC § 3.10 mandatory provenance completeness + contract '
                 '§ 8 formats (R11-B/R12)',
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
    """R11-A rule + R12-B fail-closed allowlists on a field-status manifest.

    R12-B changes vs R11: wrench-frame-dependent fields accept ONLY the
    explicit blocked statuses (null_blocked / blocked) — unknown or
    unrecognized status strings FAIL (the R11 audit bypass is closed)."""
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
    """R12-B: validate a structured status-manifest artifact.

    Required: {"composition_status": "UNVERIFIED",
               "wrench_frame_status": "UNKNOWN",
               "field_status": {field: status}}. Missing root declarations,
    a non-UNVERIFIED composition status, a non-UNKNOWN wrench status, an
    unknown field_status key, or any allowlist violation FAILS. A missing
    manifest must be handled by the caller as a failure (never PASS)."""
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
                WRENCH_BLOCKED_STATUSES + ('blocked',):
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


def check_frame_emission(frame, manifest=None,
                         composition_status=COMPOSITION_STATUS_UNVERIFIED,
                         wrench_frame_status=WRENCH_FRAME_STATUS_UNKNOWN):
    """R11-A/R12-B emission check on one emitted sample frame.

    Every wrench-frame-dependent field MUST be present and null while the
    wrench frame/reference is UNKNOWN (Option B) — a missing field is a
    failure (R12-B fail-closed), not "not emitted, therefore fine".
    Composition-dependent fields may carry values (Option A); their status
    enforcement happens via the manifest."""
    errors = []
    if not isinstance(frame, dict):
        return {
            'ok': False,
            'check': 'R12-B frame emission (Option-B nulls, presence '
                     'required)',
            'errors': ['frame artifact is not a JSON object'],
        }
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
        'check': 'R12-B frame emission (Option-B nulls + presence, '
                 'composition_status=%s, wrench_frame=%s)'
                 % (composition_status, wrench_frame_status),
        'errors': errors,
    }


# ---------------------------------------------------------------------------
# R12-D: raw storage structural contract (fail-closed)
# ---------------------------------------------------------------------------

def check_raw_storage(raw_records, composition_status=COMPOSITION_STATUS_UNVERIFIED,
                      required_categories=REQUIRED_RAW_CATEGORIES):
    """R12-D: raw records must belong to a KNOWN raw stream category, carry
    the category's required keys, and every required category must be
    present. Arbitrary unrelated JSON objects and missing required streams
    FAIL. Raw measured data remains collectible regardless of the pending
    composition status (R11-A non-propagation rule — unchanged)."""
    errors = []
    if not isinstance(raw_records, list):
        return {
            'ok': False,
            'check': 'R12-D raw storage structural contract',
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
        if not isinstance(stamp, (int, float)) or isinstance(stamp, bool):
            errors.append('raw_records[%d] (%s): stamp_s %r is not a '
                          'number' % (i, stream, stamp))
        for key in ('wrenches', 'entries', 'command', 'name'):
            if key in rec and not (isinstance(rec[key], list)
                                   and rec[key]):
                errors.append('raw_records[%d] (%s): %s must be a '
                              'non-empty list' % (i, stream, key))
        for key in ('collision1', 'collision2'):
            if key in rec and not (isinstance(rec[key], str)
                                   and rec[key].strip()):
                errors.append('raw_records[%d] (%s): %s must be a '
                              'non-empty string' % (i, stream, key))
    for category in required_categories:
        if category not in present:
            errors.append('required raw stream category %r missing from the '
                          'raw artifact (R12-D)' % category)
    return {
        'ok': not errors,
        'check': 'R12-D raw storage structural contract '
                 '(composition_status=%s; raw data unaffected per R11-A)'
                 % composition_status,
        'n_records': len(raw_records),
        'present_categories': sorted(present),
        'required_categories': list(required_categories),
        'errors': errors,
    }


# ---------------------------------------------------------------------------
# R12-A: the smoke path (actual artifacts -> all checks -> PASS/FAIL)
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
    """R12-A: the single fail-closed smoke validation path.

    Chain: RFC binding -> Collector Contract binding -> raw-category
    binding -> provenance completeness -> status manifest (pending
    propagation) -> frame emission (wrench Option-B) -> raw storage
    structure -> overall PASS/FAIL. Any failure => ok=False (caller exits
    non-zero). Missing artifacts are failures."""
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
                                    'R12-B frame emission', 'errors': errs}
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
                                 'R12-D raw storage structural contract',
                                 'errors': errs}
    else:
        checks['raw_storage'] = check_raw_storage(raw)

    failed_checks = [name for name, rep in checks.items()
                     if rep.get('ok') is not True]
    return {
        'smoke': {
            'ok': not failed_checks and not load_errors,
            'mode': 'R12-A fail-closed smoke over actual artifacts',
            'inputs': {
                'rfc': rfc_path, 'contract': contract_path,
                'manifest': manifest_path, 'provenance': provenance_path,
                'frame': frame_path, 'raw': raw_path,
            },
            'chain': ['provenance_binding (RFC § 3.10 == Contract § 8 == '
                      'checker)', 'raw_category_binding (Contract § 6 '
                      'tokens)', 'provenance completeness + formats',
                      'status manifest (R11-A/R12-B allowlists)',
                      'frame emission (wrench Option-B, presence required)',
                      'raw storage structure (R12-D)'],
            'checks': checks,
            'failed_checks': failed_checks,
        },
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Stage 1 fail-closed contract enforcement checker '
                    '(R11 + R12). Modes: --smoke (single fail-closed path '
                    'over actual artifacts, non-zero exit on failure), '
                    '--check-rfc-binding / --check-contract-binding '
                    '(document drift guards), default (print bindings + '
                    'status maps).')
    parser.add_argument('--smoke', action='store_true',
                        help='R12-A: run the full fail-closed smoke chain '
                             'over actual artifacts; exit 0 only on PASS')
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
                             '--smoke): wrench Option-B fields present and '
                             'null')
    parser.add_argument('--raw', default=None,
                        help='raw records artifact JSON (required for '
                             '--smoke): [{stream, stamp_s, ...}, ...]')
    parser.add_argument('--check-rfc-binding', action='store_true',
                        help='verify MANDATORY_PROVENANCE_FIELDS equals the '
                             'RFC § 3.10 table (R11-B drift guard)')
    parser.add_argument('--check-contract-binding', action='store_true',
                        help='verify MANDATORY_PROVENANCE_FIELDS equals the '
                             'Collector Contract § 8 table (R12-C drift '
                             'guard)')
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
    if args.check_rfc_binding and args.check_contract_binding:
        out['provenance_binding'] = check_provenance_binding(
            args.rfc, args.contract)
        binding_failed = binding_failed or not out[
            'provenance_binding']['ok']
    out['status_maps'] = {
        'measured': list(MEASURED_FIELDS),
        'ordinary_derived': list(ORDINARY_DERIVED_FIELDS),
        'composition_dependent_derived_pending': list(
            COMPOSITION_DEPENDENT_FIELDS),
        'wrench_frame_dependent_null_blocked': list(
            WRENCH_FRAME_DEPENDENT_FIELDS),
        'wrench_blocked_statuses': list(WRENCH_BLOCKED_STATUSES),
        'required_raw_categories': list(REQUIRED_RAW_CATEGORIES),
        'composition_status': COMPOSITION_STATUS_UNVERIFIED,
        'wrench_frame_status': WRENCH_FRAME_STATUS_UNKNOWN,
    }
    print(json.dumps(out, indent=2))
    return 1 if binding_failed else 0


if __name__ == '__main__':
    sys.exit(main())
