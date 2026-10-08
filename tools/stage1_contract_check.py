#!/usr/bin/env python3
"""Stage 1 contract checker: R11 machine-actionable enforcement (REV1.4).

Read-only validator for the two REV1.3-audit blockers hardened by R11:

  R11-A — pending-validation propagation:
      any derived field whose computation depends (directly or indirectly)
      on an input whose validation status is UNVERIFIED (composition) or
      pending/blocked (wrench frame/reference) MUST be emitted as
      derived_pending_validation (Option A: value retained) or remain null
      + blocked (Option B: wrench-dependent fields) — never as validated
      derived data. Raw measured fields are NOT invalidated.

  R11-B — provenance completeness:
      every mandatory provenance field of RFC § 3.10 MUST be emitted by
      the collector. MANDATORY_PROVENANCE_FIELDS is bound to the RFC § 3.10
      table by parsing it (check_rfc_binding) — drift between this list
      and the RFC is a validation failure, not a silent simplification.

This module is contract instrumentation only (Level 1/2 tooling): it
publishes nothing, subscribes to nothing, launches nothing, and modifies
no frozen component. It is NOT a new V-gate and does NOT change probe
methodology. composition_status remains UNVERIFIED and the wrench
frame/reference remains UNKNOWN until independent evidence exists — this
checker enforces that those facts cannot be silently consumed as
validated; it never upgrades them.

Field-to-status map source of truth: RFC REV1.4 § 3.2–§ 3.7 schema class
columns (R11-A annotations). Provenance source of truth: RFC § 3.10.

Usage:
    python3 tools/stage1_contract_check.py --check-rfc-binding
"""

import argparse
import json
import os
import re
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(TOOLS)
RFC_PATH = os.path.join(WS, 'docs', 'STAGE1_DYNAMICS_DATASET_RFC.md')

# --- R11-B: mandatory provenance (MUST equal RFC § 3.10) --------------------
# Bound to the RFC table by parsing (see parse_rfc_provenance_fields and
# check_rfc_binding). Do not extend this list without an RFC revision.
MANDATORY_PROVENANCE_FIELDS = (
    'theta', 'theta_id', 'theta_hash', 'theta_list_hash', 'git_commit',
    'config_hash', 'cop_derivation_hash', 'gazebo_seed',
    'robot_model_version', 'world_model_version', 'gz_sim_version',
    'stage1_rfc_hash', 'acquisition_wallclock_utc', 'ring',
    'frame_count', 't_start', 't_end', 'episode_integrity_hash',
)

# Fields whose value may legitimately be null (per RFC § 3.10: "null until
# then" — the FIELD must still be present).
NULLABLE_PROVENANCE_FIELDS = ('cop_derivation_hash',)

# --- R11-A: validation-status vocabulary (no new enums; RFC § 3.0) ----------
STATUS_MEASURED = 'measured'
STATUS_DERIVED = 'derived'
STATUS_DERIVED_PENDING = 'derived_pending_validation'

COMPOSITION_STATUS_UNVERIFIED = 'UNVERIFIED'
WRENCH_FRAME_STATUS_UNKNOWN = 'UNKNOWN'

# Raw measured fields (RFC § 3.0 classification summary): unaffected by the
# pending composition status; stored verbatim (non-propagation rule).
MEASURED_FIELDS = (
    'joint_position', 'joint_velocity', 'base_pose', 'pose_source_stamp',
    'left_contact', 'right_contact', 'imu_orientation',
    'imu_linear_acceleration', 'imu_angular_velocity', 'controller_command',
)

# Ordinary derived fields consuming NO unverified input (RFC § 3.3–§ 3.9):
# joint_acceleration (from joint_velocity), base velocities (base_pose is
# the /body_pose cross-checked model entry), deterministic gait channels.
ORDINARY_DERIVED_FIELDS = (
    'joint_acceleration', 'base_velocity', 'base_angular_velocity',
    'gait_phase', 'desired_velocity',
)

# Option A fields (value retained, status derived_pending_validation):
# depend directly or indirectly on the UNVERIFIED pose composition.
COMPOSITION_DEPENDENT_FIELDS = (
    'left_foot_pose', 'right_foot_pose',
    'left_foot_velocity', 'right_foot_velocity',
    'com_position', 'com_velocity', 'com_acceleration', 'zmp_position',
)

# Option B fields (value null + derivation blocked while the wrench
# frame/reference is undetermined — RFC § 3.6.2/§ 3.6.4/§ 3.7).
WRENCH_FRAME_DEPENDENT_FIELDS = (
    'left_contact_force', 'right_contact_force',
    'left_contact_moment', 'right_contact_moment', 'cop_position',
)

VALID_FIELD_STATUSES = (STATUS_MEASURED, STATUS_DERIVED,
                        STATUS_DERIVED_PENDING)


def parse_rfc_provenance_fields(rfc_path=RFC_PATH):
    """Parse the field names of the RFC § 3.10 provenance table.

    Returns a sorted tuple of field names. This is the machine binding of
    MANDATORY_PROVENANCE_FIELDS to the RFC text: if the RFC table changes,
    check_rfc_binding detects the drift."""
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
        if first_cell.strip() == 'Field' or re.fullmatch(r'-+', first_cell.strip()):
            continue
        fields.extend(re.findall(r'`([^`]+)`', first_cell))
    return tuple(sorted(set(fields)))


def check_rfc_binding(rfc_path=RFC_PATH):
    """R11-B drift guard: the checker list MUST equal the RFC § 3.10 table."""
    parsed = parse_rfc_provenance_fields(rfc_path)
    declared = tuple(sorted(MANDATORY_PROVENANCE_FIELDS))
    ok = parsed == declared
    return {
        'ok': ok,
        'check': 'MANDATORY_PROVENANCE_FIELDS == RFC § 3.10 table '
                 '(R11-B single source of truth)',
        'rfc_path': rfc_path,
        'declared': declared,
        'parsed_from_rfc': parsed,
        'missing_in_checker': sorted(set(parsed) - set(declared)),
        'extra_in_checker': sorted(set(declared) - set(parsed)),
    }


def check_provenance(episode_record):
    """R11-B smoke check: every RFC § 3.10 mandatory field must be present.

    Non-nullable fields must carry a non-null, non-empty value; nullable
    fields (cop_derivation_hash) may be None per the RFC. Returns a report
    dict with ok / missing / invalid; a False ok is a validation failure."""
    missing = [f for f in MANDATORY_PROVENANCE_FIELDS
               if f not in episode_record]
    invalid = []
    for f in MANDATORY_PROVENANCE_FIELDS:
        if f in episode_record and f not in NULLABLE_PROVENANCE_FIELDS:
            v = episode_record[f]
            if v is None or v == '' or v == {}:
                invalid.append(f)
    unknown = sorted(set(episode_record) - set(MANDATORY_PROVENANCE_FIELDS))
    return {
        'ok': not missing and not invalid,
        'check': 'RFC § 3.10 mandatory provenance completeness (R11-B)',
        'missing': missing,
        'invalid_nonnull': invalid,
        'note_unknown_fields_ignored': unknown,
    }


def check_pending_propagation(manifest,
                              composition_status=COMPOSITION_STATUS_UNVERIFIED,
                              wrench_frame_status=WRENCH_FRAME_STATUS_UNKNOWN):
    """R11-A smoke check on a field-status manifest (field -> status).

    Rules (RFC REV1.4 § 3.0 / collector contract § 4.7):
      * while composition_status is UNVERIFIED, every
        COMPOSITION_DEPENDENT field MUST be derived_pending_validation —
        plain derived / measured / validated annotations are failures;
      * while the wrench frame/reference is UNKNOWN, every
        WRENCH_FRAME_DEPENDENT field MUST be blocked/pending (never plain
        derived or validated) and its emitted VALUE must be null (Option B);
      * MEASURED fields are unaffected (non-propagation) — they must remain
        'measured' regardless of the composition status;
      * ORDINARY_DERIVED fields are unaffected (no unverified input).

    manifest values: status string, or None meaning "field not emitted".
    Fields not known to the checker are ignored (time/meta schema fields
    carry no validation status)."""
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
            errors.append('%s: not emitted (composition-dependent field '
                          'missing from manifest)' % f)
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
                errors.append('%s: not emitted (wrench-frame-dependent '
                              'field missing from manifest)' % f)
            elif status in (STATUS_DERIVED, STATUS_MEASURED, 'validated'):
                errors.append(
                    '%s: status %r is not blocked/pending while the wrench '
                    'frame/reference is UNKNOWN — must remain null + '
                    'blocked (Option B, R11-A)' % (f, status))
    for f in MEASURED_FIELDS:
        status = manifest.get(f)
        if status is None:
            errors.append('%s: not emitted (measured field missing from '
                          'manifest)' % f)
        elif status != STATUS_MEASURED:
            errors.append('%s: measured field status %r != %r'
                          % (f, status, STATUS_MEASURED))
    for f in ORDINARY_DERIVED_FIELDS:
        status = manifest.get(f)
        if status is None:
            errors.append('%s: not emitted (ordinary derived field missing '
                          'from manifest)' % f)
        elif status not in (STATUS_DERIVED, STATUS_DERIVED_PENDING):
            errors.append('%s: ordinary derived field status %r invalid'
                          % (f, status))
    return {
        'ok': not errors,
        'check': 'R11-A pending-validation propagation '
                 '(composition_status=%s, wrench_frame=%s)'
                 % (composition_status, wrench_frame_status),
        'errors': errors,
    }


def check_frame_emission(frame, manifest=None,
                         composition_status=COMPOSITION_STATUS_UNVERIFIED,
                         wrench_frame_status=WRENCH_FRAME_STATUS_UNKNOWN):
    """R11-A emission check on one emitted frame record.

    Option A fields: a numeric value MAY be present, but the record must
    not mark the field validated (manifest/annotation check). Option B
    fields: the emitted VALUE must be null while the wrench frame is
    UNKNOWN. Raw measured fields may always be present with values."""
    errors = []
    if wrench_frame_status == WRENCH_FRAME_STATUS_UNKNOWN:
        for f in WRENCH_FRAME_DEPENDENT_FIELDS:
            if f in frame and frame[f] is not None:
                errors.append(
                    '%s: emitted a non-null value while the wrench '
                    'frame/reference is UNKNOWN — must be null + blocked '
                    '(Option B, R11-A); value=%r' % (f, frame[f]))
    if manifest is not None:
        prop = check_pending_propagation(
            manifest, composition_status=composition_status,
            wrench_frame_status=wrench_frame_status)
        if prop['ok'] is not True:
            errors.extend(prop['errors'])
    return {
        'ok': not errors,
        'check': 'R11-A frame emission (Option B nulls + manifest statuses)',
        'errors': errors,
    }


def check_raw_storage(raw_records, composition_status=COMPOSITION_STATUS_UNVERIFIED):
    """R11-A non-propagation check: raw measured records stay collectible.

    raw_records: list of raw stream events (pose events, Contacts messages,
    joint-state messages). The pending composition status never invalidates
    them; this check verifies they are present and carry their raw payload,
    and explicitly reports that storage is allowed."""
    errors = []
    if not isinstance(raw_records, list) or not raw_records:
        errors.append('raw_records empty — raw measured streams must remain '
                      'collectible regardless of composition status (R11-A '
                      'non-propagation rule)')
    for i, rec in enumerate(raw_records):
        if not isinstance(rec, dict) or not rec:
            errors.append('raw_records[%d]: not a non-empty record' % i)
    return {
        'ok': not errors,
        'check': 'R11-A non-propagation: raw measured data unaffected '
                 '(composition_status=%s)' % composition_status,
        'n_records': len(raw_records) if isinstance(raw_records, list) else 0,
        'errors': errors,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Stage 1 R11 contract checker (read-only). '
                    'Validates the checker-RFC provenance binding and '
                    'prints the field-status maps.')
    parser.add_argument('--check-rfc-binding', action='store_true',
                        help='verify MANDATORY_PROVENANCE_FIELDS equals the '
                             'RFC § 3.10 table (R11-B drift guard)')
    parser.add_argument('--json', action='store_true',
                        help='emit machine-readable JSON')
    args = parser.parse_args(argv)
    out = {}
    if args.check_rfc_binding:
        out['rfc_binding'] = check_rfc_binding()
    out['status_maps'] = {
        'measured': list(MEASURED_FIELDS),
        'ordinary_derived': list(ORDINARY_DERIVED_FIELDS),
        'composition_dependent_derived_pending': list(
            COMPOSITION_DEPENDENT_FIELDS),
        'wrench_frame_dependent_null_blocked': list(
            WRENCH_FRAME_DEPENDENT_FIELDS),
        'composition_status': COMPOSITION_STATUS_UNVERIFIED,
        'wrench_frame_status': WRENCH_FRAME_STATUS_UNKNOWN,
    }
    if args.json:
        print(json.dumps(out, indent=2))
    else:
        print(json.dumps(out, indent=2))
    binding = out.get('rfc_binding')
    if binding is not None and not binding['ok']:
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
