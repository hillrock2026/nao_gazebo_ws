#!/usr/bin/env python3
"""R11 minimal-hardening tests (REV1.4): Tests A–D.

Run:  python3 tools/test_stage1_r11.py

  Test A — pending propagation: with composition_status = UNVERIFIED, every
           composition-dependent derived field cannot be emitted as
           validated/ordinary derived data; wrench-frame-dependent fields
           cannot be emitted non-null or validated while the wrench
           frame/reference is UNKNOWN.
  Test B — raw data unaffected: raw pose / raw model pose / raw contacts /
           raw joint data remain collectible (stored verbatim, measured)
           regardless of the pending composition status.
  Test C — provenance completeness (negative): removing ANY single RFC
           § 3.10 mandatory provenance field makes validation FAIL.
  Test D — provenance completeness (positive): a complete RFC § 3.10
           provenance record PASSES, and the checker's field list is bound
           to the RFC § 3.10 table (R11-B drift guard).

These tests are contract instrumentation (Level 1/2 tooling), not
acquisition: no simulation, no episode collection, no probe execution.
"""

import copy
import unittest

import stage1_contract_check as c


def complete_manifest():
    """Field-status manifest exactly as the REV1.4 schema declares it."""
    m = {}
    for f in c.MEASURED_FIELDS:
        m[f] = c.STATUS_MEASURED
    for f in c.ORDINARY_DERIVED_FIELDS:
        m[f] = c.STATUS_DERIVED
    for f in c.COMPOSITION_DEPENDENT_FIELDS:
        m[f] = c.STATUS_DERIVED_PENDING
    for f in c.WRENCH_FRAME_DEPENDENT_FIELDS:
        m[f] = 'null_blocked'  # Option B: derivation blocked (R11-A)
    return m


def complete_provenance():
    """A provenance record carrying every RFC § 3.10 mandatory field."""
    theta = {'gait_method': 'walk', 'step_length_m': 0.02}
    return {
        'theta': theta,
        'theta_id': 'ring1-0007',
        'theta_hash': 'a' * 64,
        'theta_list_hash': '844646f5' + '0' * 56,
        'git_commit': '0' * 40,
        'config_hash': 'b' * 64,
        'cop_derivation_hash': None,   # nullable until first derivation
        'gazebo_seed': 1,
        'robot_model_version': '44ed8285' + '0' * 56,
        'world_model_version': '383341d1' + '0' * 56,
        'gz_sim_version': '8.15.0',
        'stage1_rfc_hash': 'c' * 64,
        'acquisition_wallclock_utc': '2026-10-08T00:00:00+00:00',
        'ring': 'ring1',
        'frame_count': 6000,
        't_start': 0.5,
        't_end': 60.5,
        'episode_integrity_hash': 'd' * 64,
    }


def raw_records():
    """Raw measured stream events (verbatim storage, non-propagation)."""
    return [
        {'stream': 'pose_info', 'stamp_s': 1.012,
         'entries': [{'name': 'nao', 'position': [0.5, -0.28, 0.33]},
                     {'name': 'l_ankle', 'position': [0.0, 0.05, -0.28]}]},
        {'stream': 'contacts_left', 'stamp_s': 1.013,
         'n_contacts': 1, 'collision1': 'nao::l_ankle::l_ankle_collision',
         'wrenches': [{'body_1_force': [0.01, 0.02, 25.5]}]},
        {'stream': 'joint_states', 'stamp_s': 1.010,
         'name': ['LHipPitch'], 'position': [0.1], 'velocity': [0.0]},
    ]


class TestAPendingPropagation(unittest.TestCase):
    """Test A: unverified composition cannot appear as validated."""

    def test_correct_manifest_passes(self):
        rep = c.check_pending_propagation(complete_manifest())
        self.assertTrue(rep['ok'], rep['errors'])

    def test_every_composition_dependent_field_rejects_plain_derived(self):
        for f in c.COMPOSITION_DEPENDENT_FIELDS:
            for bad in (c.STATUS_DERIVED, 'validated', c.STATUS_MEASURED):
                m = complete_manifest()
                m[f] = bad
                rep = c.check_pending_propagation(m)
                self.assertFalse(
                    rep['ok'],
                    '%s=%r was accepted while composition_status is '
                    'UNVERIFIED (R11-A violation not detected)' % (f, bad))

    def test_missing_composition_dependent_field_fails(self):
        for f in c.COMPOSITION_DEPENDENT_FIELDS:
            m = complete_manifest()
            del m[f]
            rep = c.check_pending_propagation(m)
            self.assertFalse(rep['ok'], '%s missing was accepted' % f)

    def test_wrench_fields_reject_validated_or_derived_status(self):
        for f in c.WRENCH_FRAME_DEPENDENT_FIELDS:
            for bad in (c.STATUS_DERIVED, 'validated'):
                m = complete_manifest()
                m[f] = bad
                rep = c.check_pending_propagation(m)
                self.assertFalse(
                    rep['ok'],
                    '%s=%r accepted while wrench frame/reference is '
                    'UNKNOWN (Option B violation not detected)' % (f, bad))

    def test_wrench_fields_reject_non_null_emission(self):
        frame = {'cop_position': [0.31, -0.02]}
        rep = c.check_frame_emission(frame)
        self.assertFalse(rep['ok'], rep['errors'])
        frame_ok = {'cop_position': None, 'left_contact_force': None}
        rep2 = c.check_frame_emission(frame_ok)
        self.assertTrue(rep2['ok'], rep2['errors'])

    def test_unverified_composition_never_certified(self):
        rep = c.check_pending_propagation(
            complete_manifest(), composition_status='VERIFIED')
        self.assertIsNone(rep['ok'])
        self.assertTrue(rep['errors'])


class TestBRawDataUnaffected(unittest.TestCase):
    """Test B: raw measured data remains collectible."""

    def test_raw_records_storage_passes(self):
        rep = c.check_raw_storage(raw_records())
        self.assertTrue(rep['ok'], rep['errors'])
        self.assertEqual(rep['n_records'], 3)

    def test_measured_fields_pass_under_unverified_composition(self):
        m = {f: c.STATUS_MEASURED for f in c.MEASURED_FIELDS}
        rep = c.check_pending_propagation(m)
        # measured fields are validated (no pending propagation onto them);
        # only the *unemitted* dependent fields are flagged, none of which
        # is measured — so no measured field may appear in the errors.
        errored = {e.split(':')[0] for e in rep['errors']}
        for f in c.MEASURED_FIELDS:
            self.assertNotIn(f, errored)

    def test_raw_storage_independent_of_composition_status(self):
        for status in (c.COMPOSITION_STATUS_UNVERIFIED, 'VERIFIED'):
            rep = c.check_raw_storage(raw_records(),
                                      composition_status=status)
            self.assertTrue(rep['ok'], rep['errors'])


class TestCProvenanceMissing(unittest.TestCase):
    """Test C (negative): removing any one § 3.10 field FAILS validation."""

    def test_each_missing_field_fails(self):
        base = complete_provenance()
        for f in c.MANDATORY_PROVENANCE_FIELDS:
            rec = copy.deepcopy(base)
            del rec[f]
            rep = c.check_provenance(rec)
            self.assertFalse(
                rep['ok'],
                'missing %r was accepted (R11-B violation not detected)' % f)
            self.assertIn(f, rep['missing'])

    def test_null_non_nullable_field_fails(self):
        for f in c.MANDATORY_PROVENANCE_FIELDS:
            if f in c.NULLABLE_PROVENANCE_FIELDS:
                continue
            rec = complete_provenance()
            rec[f] = None
            rep = c.check_provenance(rec)
            self.assertFalse(
                rep['ok'], 'null %r was accepted' % f)
            self.assertIn(f, rep['invalid_nonnull'])

    def test_empty_record_fails(self):
        rep = c.check_provenance({})
        self.assertFalse(rep['ok'])
        self.assertEqual(len(rep['missing']),
                         len(c.MANDATORY_PROVENANCE_FIELDS))


class TestDProvenanceComplete(unittest.TestCase):
    """Test D (positive): complete § 3.10 provenance PASSES."""

    def test_complete_record_passes(self):
        rep = c.check_provenance(complete_provenance())
        self.assertTrue(rep['ok'], rep)
        self.assertEqual(rep['missing'], [])
        self.assertEqual(rep['invalid_nonnull'], [])

    def test_nullable_cop_derivation_hash_allowed_null(self):
        self.assertIn('cop_derivation_hash', c.NULLABLE_PROVENANCE_FIELDS)
        rep = c.check_provenance(complete_provenance())
        self.assertTrue(rep['ok'])

    def test_rfc_binding_no_drift(self):
        rep = c.check_rfc_binding()
        self.assertTrue(rep['ok'], rep)
        self.assertEqual(rep['missing_in_checker'], [])
        self.assertEqual(rep['extra_in_checker'], [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
