#!/usr/bin/env python3
"""R12 fail-closed contract enforcement tests (REV1.4).

Run:  python3 tools/test_stage1_r12.py

Covers the R12 task § 11 negative matrix, § 7.2 mutation tests, § 12
anti-circularity, § 10 CLI exit-code behavior, and the R6-R10 regression
invariants (§ 13).

Anti-circularity: fixtures are LITERALS (field names written out) or are
built from the INDEPENDENT document parsers (parse_rfc_provenance_fields /
parse_contract_provenance_fields), never from the checker's
MANDATORY_PROVENANCE_FIELDS constant. The literal fixture is additionally
asserted equal to the parsed RFC field set, so the tests can detect
checker-contract disagreement instead of only proving the checker accepts
its own output.

These tests are contract instrumentation only: no simulation, no episode
collection, no probe execution, no acquisition data.
"""

import copy
import json
import os
import subprocess
import sys
import tempfile
import unittest

import stage1_contract_check as c

TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(TOOLS)
CHECKER = os.path.join(TOOLS, 'stage1_contract_check.py')
PROBE = os.path.join(TOOLS, 'stage1_probe.py')


# ---------------------------------------------------------------------------
# Anti-circular fixtures: explicit literals (NOT built from checker lists).
# ---------------------------------------------------------------------------

# LITERAL provenance fixture: every field written out by hand. The test
# asserts this literal set == the independently parsed RFC § 3.10 table.
LITERAL_PROVENANCE_FIELDS = [
    'theta', 'theta_id', 'theta_hash', 'theta_list_hash',
    'git_commit', 'config_hash', 'cop_derivation_hash', 'gazebo_seed',
    'robot_model_version', 'world_model_version', 'gz_sim_version',
    'stage1_rfc_hash', 'acquisition_wallclock_utc', 'ring',
    'frame_count', 't_start', 't_end', 'episode_integrity_hash',
]

# LITERAL schema status fixture: field -> status, written out by hand.
LITERAL_FIELD_STATUS = {
    'joint_position': 'measured',
    'joint_velocity': 'measured',
    'base_pose': 'measured',
    'pose_source_stamp': 'measured',
    'left_contact': 'measured',
    'right_contact': 'measured',
    'imu_orientation': 'measured',
    'imu_linear_acceleration': 'measured',
    'imu_angular_velocity': 'measured',
    'controller_command': 'measured',
    'joint_acceleration': 'derived',
    'base_velocity': 'derived',
    'base_angular_velocity': 'derived',
    'gait_phase': 'derived',
    'desired_velocity': 'derived',
    'left_foot_pose': 'derived_pending_validation',
    'right_foot_pose': 'derived_pending_validation',
    'left_foot_velocity': 'derived_pending_validation',
    'right_foot_velocity': 'derived_pending_validation',
    'com_position': 'derived_pending_validation',
    'com_velocity': 'derived_pending_validation',
    'com_acceleration': 'derived_pending_validation',
    'zmp_position': 'derived_pending_validation',
    'left_contact_force': 'null_blocked',
    'right_contact_force': 'null_blocked',
    'left_contact_moment': 'null_blocked',
    'right_contact_moment': 'null_blocked',
    'cop_position': 'null_blocked',
}


def literal_manifest():
    return {
        'composition_status': 'UNVERIFIED',
        'wrench_frame_status': 'UNKNOWN',
        'field_status': dict(LITERAL_FIELD_STATUS),
    }


def literal_provenance():
    # Synthetic fixture values (NOT acquisition data). The three pinned
    # hashes are the repository's real § 0.1 pins, written out literally;
    # the rest are placeholder sha256-shaped strings.
    return {
        'theta': {'gait_method': 'walk', 'step_length_m': 0.02},
        'theta_id': 'ring1-0007',
        'theta_hash': 'a' * 64,
        'theta_list_hash':
            '844646f55076f67647c9159a6c1f53018af89276154ff3a4ad3bdce28df95dbf',
        'git_commit': '0' * 40,
        'config_hash': 'b' * 64,
        'cop_derivation_hash': None,
        'gazebo_seed': 1,
        'robot_model_version':
            '44ed828509db9e869bfa701b491233862dcd1448183bb2ec0a1c3b497aa46400',
        'world_model_version':
            '383341d11d831d0376cae074b1205e5ad7784841a6670bcf0db33813065709cc',
        'gz_sim_version': '8.15.0',
        'stage1_rfc_hash': 'c' * 64,
        'acquisition_wallclock_utc': '2026-10-08T00:00:00+00:00',
        'ring': 'ring1',
        'frame_count': 6000,
        't_start': 0.5,
        't_end': 60.5,
        'episode_integrity_hash': 'd' * 64,
    }


# Fixture self-check: the pinned literals must be exactly 64 hex chars.
for _f in ('theta_hash', 'theta_list_hash', 'config_hash',
           'robot_model_version', 'world_model_version',
           'stage1_rfc_hash', 'episode_integrity_hash'):
    _v = literal_provenance()[_f]
    assert isinstance(_v, str) and len(_v) == 64, _f


def literal_frame():
    return {
        'left_foot_pose': {'position': [0.31, 0.05, 0.05]},
        'right_foot_pose': {'position': [0.30, -0.05, 0.05]},
        'com_position': [0.31, 0.0, 0.33],
        # Option B: present and null while wrench frame/reference UNKNOWN.
        'left_contact_force': None,
        'right_contact_force': None,
        'left_contact_moment': None,
        'right_contact_moment': None,
        'cop_position': None,
    }


def literal_raw():
    return [
        {'stream': 'contacts_left', 'stamp_s': 1.013, 'n_contacts': 1,
         'collision1': 'nao::l_ankle::l_ankle_collision',
         'collision2': 'ground_plane::link::collision',
         'wrenches': [{'body_1_force': [0.01, 0.02, 25.5]}]},
        {'stream': 'contacts_right', 'stamp_s': 1.017, 'n_contacts': 1,
         'collision1': 'nao::r_ankle::r_ankle_collision',
         'collision2': 'ground_plane::link::collision',
         'wrenches': [{'body_1_force': [0.0, -0.01, 24.8]}]},
        {'stream': 'pose_info', 'stamp_s': 1.012,
         'entries': [{'name': 'nao', 'position': [0.5, -0.28, 0.33]},
                     {'name': 'l_ankle', 'position': [0.0, 0.05, -0.28]}]},
        {'stream': 'joint_command', 'stamp_s': 1.010, 'command': [0.1] * 24},
    ]


def write_fixture(dirname, manifest=None, provenance=None, frame=None,
                  raw=None):
    paths = {}
    for name, payload in (('manifest', manifest), ('provenance',
                                                   provenance),
                          ('frame', frame), ('raw', raw)):
        if payload is None:
            continue
        path = os.path.join(dirname, '%s.json' % name)
        with open(path, 'w', encoding='utf-8') as fh:
            json.dump(payload, fh)
        paths[name] = path
    return paths


def run_cli(paths, extra=()):
    cmd = [sys.executable, CHECKER, '--smoke',
           '--rfc', c.RFC_PATH, '--contract', c.CONTRACT_PATH]
    for name in ('manifest', 'provenance', 'frame', 'raw'):
        if name in paths:
            cmd.extend(['--%s' % name, paths[name]])
    cmd.extend(extra)
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    return proc


def mutate_rfc_add_field(src, dst):
    """Insert one extra table row into RFC § 3.10 (+1 field mutation)."""
    with open(src, 'r', encoding='utf-8') as fh:
        text = fh.read()
    anchor = '| `episode_integrity_hash` |'
    idx = text.find(anchor)
    assert idx >= 0, 'RFC § 3.10 anchor not found'
    line_end = text.find('\n', idx)
    mutated = (text[:line_end + 1]
               + '| `r12_mutant_field` | mutation test row |\n'
               + text[line_end + 1:])
    with open(dst, 'w', encoding='utf-8') as fh:
        fh.write(mutated)


def mutate_rfc_remove_field(src, dst):
    """Remove the `ring` row from RFC § 3.10 (-1 field mutation)."""
    with open(src, 'r', encoding='utf-8') as fh:
        lines = fh.readlines()
    out, removed = [], False
    for line in lines:
        if not removed and line.startswith('| `ring` |'):
            removed = True
            continue
        out.append(line)
    assert removed, 'RFC § 3.10 ring row not found'
    with open(dst, 'w', encoding='utf-8') as fh:
        fh.writelines(out)


def mutate_contract_add_field(src, dst):
    """Insert one extra checklist row into Contract § 8 (+1 field)."""
    with open(src, 'r', encoding='utf-8') as fh:
        text = fh.read()
    anchor = '| 18 | `episode_integrity_hash` |'
    idx = text.find(anchor)
    assert idx >= 0, 'Contract § 8 anchor not found'
    line_end = text.find('\n', idx)
    mutated = (text[:line_end + 1]
               + '| 19 | `r12_mutant_field` | mutation test row |\n'
               + text[line_end + 1:])
    with open(dst, 'w', encoding='utf-8') as fh:
        fh.write(mutated)


def mutate_contract_remove_field(src, dst):
    """Remove one checklist row from Contract § 8 (-1 field)."""
    with open(src, 'r', encoding='utf-8') as fh:
        lines = fh.readlines()
    out, removed = [], False
    for line in lines:
        if not removed and line.startswith('| 14 | `ring` |'):
            removed = True
            continue
        out.append(line)
    assert removed, 'Contract § 8 ring row not found'
    with open(dst, 'w', encoding='utf-8') as fh:
        fh.writelines(out)


class TestR12BAntiBypass(unittest.TestCase):
    """R12-B: no status can bypass the fail-closed allowlists."""

    def test_composition_field_rejects_derived_validated_measured(self):
        bad = ('derived', 'validated', 'measured')
        for f in c.COMPOSITION_DEPENDENT_FIELDS:
            for status in bad:
                m = dict(LITERAL_FIELD_STATUS)
                m[f] = status
                rep = c.check_pending_propagation(m)
                self.assertFalse(rep['ok'],
                                 '%s=%r bypassed (R12-B)' % (f, status))

    def test_composition_field_rejects_unknown_and_missing(self):
        for f in c.COMPOSITION_DEPENDENT_FIELDS:
            for status in ('UNRECOGNIZED_STATUS', 'unknown', 'pending?',
                           '', None):
                m = dict(LITERAL_FIELD_STATUS)
                if status is None:
                    del m[f]
                else:
                    m[f] = status
                rep = c.check_pending_propagation(m)
                self.assertFalse(rep['ok'],
                                 '%s=%r bypassed (R12-B)' % (f, status))

    def test_wrench_field_rejects_unknown_status(self):
        # The exact R11-audit bypass: cop_position = null with an unknown
        # status string must FAIL.
        for f in c.WRENCH_FRAME_DEPENDENT_FIELDS:
            for status in ('UNRECOGNIZED_STATUS', 'banana', 'derived',
                           'validated', 'measured', 'null_pending?', ''):
                m = dict(LITERAL_FIELD_STATUS)
                m[f] = status
                rep = c.check_pending_propagation(m)
                self.assertFalse(rep['ok'],
                                 '%s=%r bypassed (R12-B Option-B bypass)'
                                 % (f, status))

    def test_wrench_field_accepts_only_explicit_blocked_statuses(self):
        for status in c.WRENCH_BLOCKED_STATUSES:
            m = dict(LITERAL_FIELD_STATUS)
            m['cop_position'] = status
            rep = c.check_pending_propagation(m)
            self.assertTrue(rep['ok'], rep['errors'])

    def test_manifest_root_declarations_required(self):
        rep = c.check_status_manifest({'field_status':
                                       dict(LITERAL_FIELD_STATUS)})
        self.assertFalse(rep['ok'])
        rep = c.check_status_manifest({})
        self.assertFalse(rep['ok'])
        rep = c.check_status_manifest(literal_manifest())
        self.assertTrue(rep['ok'], rep['errors'])

    def test_manifest_rejects_certified_composition(self):
        m = literal_manifest()
        m['composition_status'] = 'VERIFIED'
        rep = c.check_status_manifest(m)
        self.assertFalse(rep['ok'])
        m = literal_manifest()
        m['wrench_frame_status'] = 'DETERMINED'
        rep = c.check_status_manifest(m)
        self.assertFalse(rep['ok'])

    def test_manifest_rejects_unknown_field_key(self):
        m = literal_manifest()
        m['field_status']['not_a_schema_field'] = 'derived'
        rep = c.check_status_manifest(m)
        self.assertFalse(rep['ok'])

    def test_frame_option_b_presence_required(self):
        frame = {'cop_position': None}  # other Option-B fields missing
        rep = c.check_frame_emission(frame)
        self.assertFalse(rep['ok'], rep['errors'])
        frame = literal_frame()
        rep = c.check_frame_emission(frame)
        self.assertTrue(rep['ok'], rep['errors'])

    def test_frame_option_b_non_null_fails(self):
        frame = literal_frame()
        frame['cop_position'] = [0.31, -0.02]
        rep = c.check_frame_emission(frame)
        self.assertFalse(rep['ok'])


class TestR12CRfcContractBinding(unittest.TestCase):
    """R12-C: RFC § 3.10 == Contract § 8 == checker, with mutation tests."""

    def test_binding_holds_on_repository_documents(self):
        rep = c.check_provenance_binding()
        self.assertTrue(rep['ok'], rep)
        self.assertTrue(rep['rfc_binding']['ok'])
        self.assertTrue(rep['contract_binding']['ok'])
        self.assertTrue(rep['rfc_vs_contract']['equal'])

    def test_literal_fixture_matches_parsed_rfc(self):
        # Anti-circularity: the hand-written literal set must equal the
        # independently parsed RFC table (and the contract table).
        parsed_rfc = c.parse_rfc_provenance_fields()
        parsed_contract = c.parse_contract_provenance_fields()
        self.assertEqual(sorted(LITERAL_PROVENANCE_FIELDS),
                         list(parsed_rfc))
        self.assertEqual(list(parsed_rfc), list(parsed_contract))

    def test_rfc_add_field_mutation_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            mutated = os.path.join(tmp, 'rfc_mutated.md')
            mutate_rfc_add_field(c.RFC_PATH, mutated)
            rep = c.check_rfc_binding(mutated)
            self.assertFalse(rep['ok'])
            self.assertIn('r12_mutant_field', rep['extra_in_checker'] or
                          rep['missing_in_checker'] + rep['extra_in_checker'])
            full = c.check_provenance_binding(mutated, c.CONTRACT_PATH)
            self.assertFalse(full['ok'])

    def test_rfc_remove_field_mutation_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            mutated = os.path.join(tmp, 'rfc_mutated.md')
            mutate_rfc_remove_field(c.RFC_PATH, mutated)
            rep = c.check_rfc_binding(mutated)
            self.assertFalse(rep['ok'])

    def test_contract_add_field_mutation_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            mutated = os.path.join(tmp, 'contract_mutated.md')
            mutate_contract_add_field(c.CONTRACT_PATH, mutated)
            rep = c.check_contract_binding(mutated)
            self.assertFalse(rep['ok'])
            full = c.check_provenance_binding(c.RFC_PATH, mutated)
            self.assertFalse(full['ok'])

    def test_contract_remove_field_mutation_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            mutated = os.path.join(tmp, 'contract_mutated.md')
            mutate_contract_remove_field(c.CONTRACT_PATH, mutated)
            rep = c.check_contract_binding(mutated)
            self.assertFalse(rep['ok'])

    def test_raw_category_binding_holds_and_detects_drift(self):
        rep = c.check_raw_category_binding()
        self.assertTrue(rep['ok'], rep)
        with tempfile.TemporaryDirectory() as tmp:
            mutated = os.path.join(tmp, 'contract_mutated.md')
            with open(c.CONTRACT_PATH, 'r', encoding='utf-8') as fh:
                text = fh.read()
            # remove the pose topic token from § 6.1
            start = text.find('## 6.')
            end = text.find('## 7.')
            section = text[start:end].replace('/world/nao_world/pose/info',
                                              '<removed>')
            mutated_text = text[:start] + section + text[end:]
            with open(mutated, 'w', encoding='utf-8') as fh:
                fh.write(mutated_text)
            rep2 = c.check_raw_category_binding(mutated)
            self.assertFalse(rep2['ok'])


class TestR12DRawStructural(unittest.TestCase):
    """R12-D: raw storage structural contract."""

    def test_valid_raw_passes(self):
        rep = c.check_raw_storage(literal_raw())
        self.assertTrue(rep['ok'], rep['errors'])
        self.assertEqual(rep['present_categories'],
                         ['contacts_left', 'contacts_right', 'joint_command',
                          'pose_info'])

    def test_arbitrary_unrelated_record_fails(self):
        rep = c.check_raw_storage([{'unrelated': 1}])
        self.assertFalse(rep['ok'])

    def test_required_stream_missing_fails(self):
        for drop in ('contacts_left', 'contacts_right', 'pose_info',
                     'joint_command'):
            raw = [r for r in literal_raw() if r['stream'] != drop]
            rep = c.check_raw_storage(raw)
            self.assertFalse(rep['ok'],
                             'missing required stream %r accepted' % drop)

    def test_unknown_stream_fails(self):
        raw = literal_raw() + [{'stream': 'zzz_unrelated', 'stamp_s': 1.0,
                                'payload': {'x': 1}}]
        rep = c.check_raw_storage(raw)
        self.assertFalse(rep['ok'])

    def test_structurally_invalid_record_fails(self):
        raw = literal_raw() + [{'stream': 'pose_info', 'entries': []}]
        rep = c.check_raw_storage(raw)
        self.assertFalse(rep['ok'])
        raw = literal_raw() + [{'stream': 'contacts_left',
                                'collision1': '', 'collision2': '',
                                'wrenches': []}]
        rep = c.check_raw_storage(raw)
        self.assertFalse(rep['ok'])
        raw = literal_raw() + [{'stream': 'joint_command', 'command': []}]
        rep = c.check_raw_storage(raw)
        self.assertFalse(rep['ok'])

    def test_raw_storage_not_a_list_fails(self):
        rep = c.check_raw_storage({'not': 'a list'})
        self.assertFalse(rep['ok'])


class TestR12ProvenanceFailClosed(unittest.TestCase):
    """R12 provenance: completeness + contract formats, fail-closed."""

    def test_literal_complete_provenance_passes(self):
        rep = c.check_provenance(literal_provenance())
        self.assertTrue(rep['ok'], rep)

    def test_every_missing_field_fails(self):
        for f in LITERAL_PROVENANCE_FIELDS:
            rec = copy.deepcopy(literal_provenance())
            del rec[f]
            rep = c.check_provenance(rec)
            self.assertFalse(rep['ok'], 'missing %r accepted' % f)

    def test_bad_hash_format_fails(self):
        for f in ('theta_hash', 'config_hash', 'stage1_rfc_hash',
                  'robot_model_version', 'episode_integrity_hash',
                  'theta_list_hash'):
            rec = copy.deepcopy(literal_provenance())
            rec[f] = 'not-a-sha256'
            rep = c.check_provenance(rec)
            self.assertFalse(rep['ok'], 'bad %r accepted' % f)

    def test_bad_git_commit_and_types_fail(self):
        rec = literal_provenance()
        rec['git_commit'] = 'xyz'
        self.assertFalse(c.check_provenance(rec)['ok'])
        rec = literal_provenance()
        rec['gazebo_seed'] = 'one'
        self.assertFalse(c.check_provenance(rec)['ok'])
        rec = literal_provenance()
        rec['theta'] = 'not-a-dict'
        self.assertFalse(c.check_provenance(rec)['ok'])
        rec = literal_provenance()
        rec['cop_derivation_hash'] = 'not-a-sha256'
        self.assertFalse(c.check_provenance(rec)['ok'])

    def test_non_dict_record_fails(self):
        self.assertFalse(c.check_provenance(['list'])['ok'])
        self.assertFalse(c.check_provenance(None)['ok'])


class TestR12ACliSmoke(unittest.TestCase):
    """R12-A: actual CLI over actual artifacts; exit codes are binding."""

    def test_valid_fixture_exit_zero(self):
        with tempfile.TemporaryDirectory() as tmp:
            paths = write_fixture(tmp, manifest=literal_manifest(),
                                  provenance=literal_provenance(),
                                  frame=literal_frame(),
                                  raw=literal_raw())
            proc = run_cli(paths)
            self.assertEqual(proc.returncode, 0, proc.stdout[-2000:])
            payload = json.loads(proc.stdout)
            self.assertTrue(payload['smoke']['ok'])
            self.assertEqual(payload['smoke']['failed_checks'], [])

    def test_invalid_manifest_exit_nonzero(self):
        with tempfile.TemporaryDirectory() as tmp:
            m = literal_manifest()
            m['field_status']['left_foot_velocity'] = 'derived'
            paths = write_fixture(tmp, manifest=m,
                                  provenance=literal_provenance(),
                                  frame=literal_frame(),
                                  raw=literal_raw())
            proc = run_cli(paths)
            self.assertNotEqual(proc.returncode, 0)
            payload = json.loads(proc.stdout)
            self.assertFalse(payload['smoke']['ok'])
            self.assertIn('status_manifest', payload['smoke'][
                'failed_checks'])

    def test_invalid_frame_exit_nonzero(self):
        with tempfile.TemporaryDirectory() as tmp:
            f = literal_frame()
            f['cop_position'] = [0.31, -0.02]   # Option-B bypass attempt
            paths = write_fixture(tmp, manifest=literal_manifest(),
                                  provenance=literal_provenance(),
                                  frame=f, raw=literal_raw())
            proc = run_cli(paths)
            self.assertNotEqual(proc.returncode, 0)
            payload = json.loads(proc.stdout)
            self.assertFalse(payload['smoke']['ok'])

    def test_missing_provenance_field_exit_nonzero(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = literal_provenance()
            del p['theta_list_hash']
            paths = write_fixture(tmp, manifest=literal_manifest(),
                                  provenance=p, frame=literal_frame(),
                                  raw=literal_raw())
            proc = run_cli(paths)
            self.assertNotEqual(proc.returncode, 0)

    def test_missing_artifact_file_exit_nonzero(self):
        with tempfile.TemporaryDirectory() as tmp:
            paths = write_fixture(tmp, manifest=literal_manifest(),
                                  provenance=literal_provenance(),
                                  frame=literal_frame())
            paths['raw'] = os.path.join(tmp, 'does_not_exist.json')
            proc = run_cli(paths)
            self.assertNotEqual(proc.returncode, 0)
            payload = json.loads(proc.stdout)
            self.assertFalse(payload['smoke']['ok'])

    def test_missing_manifest_artifact_exit_nonzero(self):
        with tempfile.TemporaryDirectory() as tmp:
            paths = write_fixture(tmp, provenance=literal_provenance(),
                                  frame=literal_frame(),
                                  raw=literal_raw())
            proc = run_cli(paths)   # no --manifest at all
            self.assertNotEqual(proc.returncode, 0)
            payload = json.loads(proc.stdout)
            self.assertFalse(payload['smoke']['ok'])
            self.assertIn('status_manifest',
                          payload['smoke']['failed_checks'])

    def test_invalid_raw_exit_nonzero(self):
        with tempfile.TemporaryDirectory() as tmp:
            paths = write_fixture(tmp, manifest=literal_manifest(),
                                  provenance=literal_provenance(),
                                  frame=literal_frame(),
                                  raw=[{'unrelated': 1}])
            proc = run_cli(paths)
            self.assertNotEqual(proc.returncode, 0)
            payload = json.loads(proc.stdout)
            self.assertIn('raw_storage', payload['smoke']['failed_checks'])

    def test_mutated_rfc_document_exit_nonzero(self):
        with tempfile.TemporaryDirectory() as tmp:
            mutated = os.path.join(tmp, 'rfc_mutated.md')
            mutate_rfc_add_field(c.RFC_PATH, mutated)
            paths = write_fixture(tmp, manifest=literal_manifest(),
                                  provenance=literal_provenance(),
                                  frame=literal_frame(),
                                  raw=literal_raw())
            proc = run_cli(paths, extra=['--rfc', mutated])
            self.assertNotEqual(proc.returncode, 0)
            payload = json.loads(proc.stdout)
            self.assertIn('provenance_binding',
                          payload['smoke']['failed_checks'])


class TestR6toR10Regression(unittest.TestCase):
    """§ 13: R6-R10 semantics must not be reintroduced/changed by R12."""

    def test_r6_no_fixed_cadence_band_in_probe(self):
        with open(PROBE, 'r', encoding='utf-8') as fh:
            text = fh.read()
        self.assertNotIn('CADENCE_MIN_HZ', text)
        self.assertNotIn('CADENCE_MAX_HZ', text)
        self.assertIn('event_publication_stats', text)
        self.assertIn('contact_publication_events', text)

    def test_r7_two_state_vocabulary_preserved(self):
        with open(c.RFC_PATH, 'r', encoding='utf-8') as fh:
            text = fh.read()
        self.assertIn('UNKNOWN_NO_MESSAGE', text)
        self.assertIn('VALID_CONTACT', text)
        # two-state contract wording present; VALID_NO_CONTACT only as a
        # future extension
        self.assertIn('future extension', text)
        # R7 no-fabrication rules live in the governing documents
        self.assertIn('no fabricated flight labels', text)
        with open(c.CONTRACT_PATH, 'r', encoding='utf-8') as fh:
            contract = fh.read()
        self.assertIn('两态', contract)
        self.assertIn('不得伪造', contract)

    def test_r8_composition_remains_unverified(self):
        self.assertEqual(c.COMPOSITION_STATUS_UNVERIFIED, 'UNVERIFIED')
        with open(c.RFC_PATH, 'r', encoding='utf-8') as fh:
            text = fh.read()
        self.assertIn('composition_status', text)
        self.assertIn('UNVERIFIED', text)
        # the checker must refuse to certify anything else
        rep = c.check_pending_propagation(
            dict(LITERAL_FIELD_STATUS), composition_status='VERIFIED')
        self.assertNotEqual(rep['ok'], True)

    def test_r9_measured_vs_derived_semantics_preserved(self):
        with open(c.RFC_PATH, 'r', encoding='utf-8') as fh:
            text = fh.read()
        self.assertIn('derived_pending_validation', text)
        self.assertIn('measured', text)
        self.assertIn('derived', text)

    def test_r10_association_compatibility_not_clock_sync(self):
        with open(PROBE, 'r', encoding='utf-8') as fh:
            text = fh.read()
        self.assertIn('timestamp_association_compatibility', text)
        self.assertNotIn('timestamp_semantics_confirmed', text)
        with open(c.RFC_PATH, 'r', encoding='utf-8') as fh:
            text = fh.read()
        self.assertIn('association compatibility', text)
        self.assertIn('universal clock identity', text)


if __name__ == '__main__':
    unittest.main(verbosity=2)
