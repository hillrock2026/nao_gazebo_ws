#!/usr/bin/env python3
"""R13 fail-closed schema hardening tests (REV1.4).

Run:  python3 tools/test_stage1_r13.py

Attack-driven (R13 task § 20-§ 23): every case is a malformed, drifted, or
corrupted artifact; the assertion is that the ACTUAL --smoke CLI path exits
non-zero (not merely that a helper returns False). The § 23 end-to-end
smoke matrix is executed via subprocess. Fixtures are hand-written literals
(anti-circularity, § 21): field names and values are written out, not
generated from the checker's constants; the literal provenance field list
is separately asserted equal to the independently parsed RFC § 3.10 table.

No Gazebo acquisition, no network, no external service; synthetic fixtures
and temporary mutation copies only.
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
# Hand-written literal fixtures (§ 21 anti-circularity)
# ---------------------------------------------------------------------------

LITERAL_PROVENANCE_FIELDS = [
    'theta', 'theta_id', 'theta_hash', 'theta_list_hash',
    'git_commit', 'config_hash', 'cop_derivation_hash', 'gazebo_seed',
    'robot_model_version', 'world_model_version', 'gz_sim_version',
    'stage1_rfc_hash', 'acquisition_wallclock_utc', 'ring',
    'frame_count', 't_start', 't_end', 'episode_integrity_hash',
]

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
    # Synthetic values (NOT acquisition data); pinned hashes written out
    # literally as the repository's real § 0.1 pins.
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
        'frame_count': 1,
        't_start': 0.5,
        't_end': 60.5,
        'episode_integrity_hash': 'd' * 64,
    }


def literal_frame():
    # R13-A Option-A shapes written out literally: pose {position[3],
    # orientation[4]}; vec3; zmp vec2; Option-B fields present and null.
    return {
        'left_foot_pose': {'position': [0.31, 0.05, 0.05],
                           'orientation': [0.0, 0.0, 0.0, 1.0]},
        'right_foot_pose': {'position': [0.30, -0.05, 0.05],
                            'orientation': [0.0, 0.0, 0.0, 1.0]},
        'left_foot_velocity': [0.01, 0.0, 0.0],
        'right_foot_velocity': [0.0, 0.0, 0.0],
        'com_position': [0.31, 0.0, 0.33],
        'com_velocity': [0.0, 0.0, 0.0],
        'com_acceleration': [0.0, 0.0, 0.0],
        'zmp_position': [0.31, 0.0],
        'left_contact_force': None,
        'right_contact_force': None,
        'left_contact_moment': None,
        'right_contact_moment': None,
        'cop_position': None,
    }


def literal_raw():
    # RFC-shaped nested payloads, written out literally.
    return [
        {'stream': 'contacts_left', 'stamp_s': 1.013, 'n_contacts': 1,
         'contacts': [{
             'header': {'stamp_s': 1.013},
             'collision1': 'nao::l_ankle::l_ankle_collision',
             'collision2': 'ground_plane::link::collision',
             'position': [[0.1, 0.05, 0.0]],
             'normal': [[0.0, 0.0, 1.0]],
             'depth': [0.001],
             'wrenches': [{
                 'body_1_name': '', 'body_1_id': 0,
                 'body_2_name': '', 'body_2_id': 0,
                 'body_1_wrench': {'force': [0.01, 0.02, 25.5],
                                   'torque': [0.0, 0.001, 0.0],
                                   'force_offset': [0.1, 0.05, 0.0]},
                 'body_2_wrench': {'force': [-0.01, -0.02, -25.5],
                                   'torque': [0.0, -0.001, 0.0],
                                   'force_offset': [0.1, 0.05, 0.0]}}]}]},
        {'stream': 'contacts_right', 'stamp_s': 1.017, 'n_contacts': 1,
         'contacts': [{
             'header': {'stamp_s': 1.017},
             'collision1': 'nao::r_ankle::r_ankle_collision',
             'collision2': 'ground_plane::link::collision',
             'position': [[0.1, -0.05, 0.0]],
             'normal': [[0.0, 0.0, 1.0]],
             'depth': [0.001],
             'wrenches': [{
                 'body_1_name': '', 'body_1_id': 0,
                 'body_2_name': '', 'body_2_id': 0,
                 'body_1_wrench': {'force': [0.0, -0.01, 24.8],
                                   'torque': [0.0, 0.0, 0.0],
                                   'force_offset': [0.1, -0.05, 0.0]},
                 'body_2_wrench': {'force': [0.0, 0.01, -24.8],
                                   'torque': [0.0, 0.0, 0.0],
                                   'force_offset': [0.1, -0.05, 0.0]}}]}]},
        {'stream': 'pose_info', 'stamp_s': 1.012,
         'entries': [{'name': 'nao', 'position': [0.5, -0.28, 0.33],
                      'orientation': [0.0, 0.0, 0.0, 1.0]},
                     {'name': 'l_ankle', 'position': [0.0, 0.05, -0.28],
                      'orientation': [0.0, 0.0, 0.0, 1.0]}]},
        {'stream': 'joint_command', 'stamp_s': 1.010, 'command': [0.1] * 24},
    ]


# ---------------------------------------------------------------------------
# CLI harness
# ---------------------------------------------------------------------------

def write_case(dirname, manifest=None, provenance=None, frame=None, raw=None,
               rfc_text=None, contract_text=None):
    """Write a full fixture set; returns the CLI argument list."""
    args = []
    if rfc_text is not None:
        p = os.path.join(dirname, 'rfc.md')
        with open(p, 'w', encoding='utf-8') as fh:
            fh.write(rfc_text)
        args.extend(['--rfc', p])
    if contract_text is not None:
        p = os.path.join(dirname, 'contract.md')
        with open(p, 'w', encoding='utf-8') as fh:
            fh.write(contract_text)
        args.extend(['--contract', p])
    for name, payload in (('manifest', manifest), ('provenance',
                                                   provenance),
                          ('frame', frame), ('raw', raw)):
        if payload is None:
            continue
        p = os.path.join(dirname, '%s.json' % name)
        with open(p, 'w', encoding='utf-8') as fh:
            json.dump(payload, fh)
        args.extend(['--%s' % name, p])
    return args


def run_cli(args):
    cmd = [sys.executable, CHECKER, '--smoke'] + args
    return subprocess.run(cmd, capture_output=True, text=True, timeout=60)


def expect_fail(name, proc):
    assert proc.returncode != 0, \
        '%s: expected exit != 0, got 0\n%s' % (name, proc.stdout[-3000:])
    payload = json.loads(proc.stdout)
    assert payload['smoke']['ok'] is False, \
        '%s: exit non-zero but smoke ok=true' % name


# ---------------------------------------------------------------------------
# Document mutation helpers (§ 8.4 / § 20 semantic-binding attacks)
# ---------------------------------------------------------------------------

def read(path):
    with open(path, 'r', encoding='utf-8') as fh:
        return fh.read()


def write_text(path, text):
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return text


def mutate_insert_row(text, anchor, row):
    idx = text.find(anchor)
    assert idx >= 0, 'anchor %r not found' % anchor[:40]
    end = text.find('\n', idx)
    return text[:end + 1] + row + '\n' + text[end + 1:]


def mutate_remove_line(text, prefix):
    lines = text.split('\n')
    out, removed = [], False
    for line in lines:
        if not removed and line.startswith(prefix):
            removed = True
            continue
        out.append(line)
    assert removed, 'line %r not found' % prefix[:40]
    return '\n'.join(out)


def mutate_rfc_add_field(src):
    return mutate_insert_row(read(src), '| `episode_integrity_hash` |',
                             '| `r13_mutant_field` | mutation test row |')


def mutate_rfc_remove_field(src):
    return mutate_remove_line(read(src), '| `ring` |')


def mutate_contract_add_field(src):
    return mutate_insert_row(read(src), '| 18 | `episode_integrity_hash` |',
                             '| 19 | `r13_mutant_field` | mutation test row |')


def mutate_contract_remove_field(src):
    return mutate_remove_line(read(src), '| 14 | `ring` |')


def mutate_rfc_nullable(src):
    # R14: semantic mutations target the normative JSON block (Appendix D),
    # not prose. cop_derivation_hash nullable true -> false.
    text = read(src)
    old = ('{"name": "cop_derivation_hash", "required": true, '
           '"nullable": true, "type": "sha256"}')
    assert text.count(old) == 1, 'cop_derivation_hash block row not unique'
    return text.replace(
        old, '{"name": "cop_derivation_hash", "required": true, '
        '"nullable": false, "type": "sha256"}', 1)


def mutate_contract_nullable(src):
    # R14: same mutation in the Contract § 8.4 block copy.
    text = read(src)
    old = ('{"name": "cop_derivation_hash", "required": true, '
           '"nullable": true, "type": "sha256"}')
    assert text.count(old) == 1, 'cop_derivation_hash block row not unique'
    return text.replace(
        old, '{"name": "cop_derivation_hash", "required": true, '
        '"nullable": false, "type": "sha256"}', 1)


def mutate_rfc_type(src):
    # R14: type mutation in the normative block (theta_hash sha256 -> string).
    text = read(src)
    old = ('{"name": "theta_hash", "required": true, "nullable": false, '
           '"type": "sha256"}')
    assert text.count(old) == 1, 'theta_hash block row not unique'
    return text.replace(
        old, '{"name": "theta_hash", "required": true, "nullable": false, '
        '"type": "string"}', 1)


def mutate_rfc_type_removal(src):
    # R14: removing a semantic key from the block makes the schema itself
    # structurally invalid (fail-closed load failure).
    text = read(src)
    old = ('{"name": "theta_hash", "required": true, "nullable": false, '
           '"type": "sha256"}')
    assert text.count(old) == 1
    return text.replace(
        old, '{"name": "theta_hash", "required": true, "nullable": false}',
        1)


def mutate_contract_type(src):
    # R14: type mutation in the Contract § 8.4 block copy (theta object ->
    # string) -> block inequality.
    text = read(src)
    old = ('{"name": "theta", "required": true, "nullable": false, '
           '"type": "object"}')
    assert text.count(old) == 1, 'theta block row not unique'
    return text.replace(
        old, '{"name": "theta", "required": true, "nullable": false, '
        '"type": "string"}', 1)


def mutate_rfc_requiredness(src):
    # R14: requiredness mutation in the block (ring required -> optional).
    text = read(src)
    old = ('{"name": "ring", "required": true, "nullable": false, '
           '"type": "string"}')
    assert text.count(old) == 1, 'ring block row not unique'
    return text.replace(
        old, '{"name": "ring", "required": false, "nullable": false, '
        '"type": "string"}', 1)


def mutate_contract_requiredness(src):
    # R14: requiredness mutation in the Contract § 8.4 block copy
    # (gazebo_seed required -> optional).
    text = read(src)
    old = ('{"name": "gazebo_seed", "required": true, "nullable": false, '
           '"type": "int"}')
    assert text.count(old) == 1, 'gazebo_seed block row not unique'
    return text.replace(
        old, '{"name": "gazebo_seed", "required": false, "nullable": false, '
        '"type": "int"}', 1)


# ---------------------------------------------------------------------------
# R13-A unit attacks: Option-A frame bypass (audit B1)
# ---------------------------------------------------------------------------

class TestR13AFrameAttacks(unittest.TestCase):

    def test_valid_frame_passes(self):
        rep = c.check_frame_emission(literal_frame())
        self.assertTrue(rep['ok'], rep['errors'])

    def test_option_a_missing_fails(self):
        for f in ('left_foot_pose', 'right_foot_velocity', 'com_position',
                  'zmp_position', 'com_acceleration'):
            frame = copy.deepcopy(literal_frame())
            del frame[f]
            rep = c.check_frame_emission(frame)
            self.assertFalse(rep['ok'], '%s missing accepted (B1)' % f)

    def test_option_a_null_fails(self):
        for f in ('left_foot_pose', 'com_velocity', 'zmp_position'):
            frame = copy.deepcopy(literal_frame())
            frame[f] = None
            rep = c.check_frame_emission(frame)
            self.assertFalse(rep['ok'], '%s null accepted (B1)' % f)

    def test_option_a_wrong_type_fails(self):
        cases = {
            'com_position': 'not-a-vector',
            'left_foot_velocity': 3,
            'zmp_position': {'x': 1, 'y': 2},
            'left_foot_pose': [0.31, 0.05, 0.05],
        }
        for f, v in cases.items():
            frame = copy.deepcopy(literal_frame())
            frame[f] = v
            rep = c.check_frame_emission(frame)
            self.assertFalse(rep['ok'], '%s type %r accepted (B1)'
                             % (f, type(v).__name__))

    def test_option_a_wrong_shape_fails(self):
        cases = {
            'com_position': [0.1, 0.2],                 # vec3 -> len 2
            'zmp_position': [0.1, 0.2, 0.3],            # vec2 -> len 3
            'left_foot_pose': {'position': [0.1, 0.2, 0.3]},       # no orientation
            'right_foot_pose': {'position': [0.1, 0.2, 0.3],
                                'orientation': [0.0, 0.0, 1.0]},   # quat len 3
            'left_foot_velocity': [0.1, 0.2, float('nan')],        # non-finite
        }
        for f, v in cases.items():
            frame = copy.deepcopy(literal_frame())
            frame[f] = v
            rep = c.check_frame_emission(frame)
            self.assertFalse(rep['ok'], '%s shape %r accepted (B1)' % (f, v))

    def test_option_b_non_null_and_missing_fail(self):
        frame = copy.deepcopy(literal_frame())
        frame['cop_position'] = [0.31, 0.0]
        self.assertFalse(c.check_frame_emission(frame)['ok'])
        frame = copy.deepcopy(literal_frame())
        del frame['left_contact_force']
        self.assertFalse(c.check_frame_emission(frame)['ok'])

    def test_status_value_mismatch_fails(self):
        # manifest declares pending, frame value malformed/null -> FAIL
        frame = copy.deepcopy(literal_frame())
        frame['com_position'] = None
        rep = c.check_frame_emission(
            frame, manifest=dict(LITERAL_FIELD_STATUS))
        self.assertFalse(rep['ok'])
        frame = copy.deepcopy(literal_frame())
        frame['com_position'] = 'not-a-vector'
        rep = c.check_frame_emission(
            frame, manifest=dict(LITERAL_FIELD_STATUS))
        self.assertFalse(rep['ok'])


# ---------------------------------------------------------------------------
# R13-B unit attacks: semantic binding (audit B2)
# ---------------------------------------------------------------------------

class TestR13BSemanticBinding(unittest.TestCase):

    def test_positive_binding_holds_on_repository_documents(self):
        rep = c.check_semantic_binding()
        self.assertTrue(rep['ok'], rep['errors'])

    def test_literal_field_list_matches_parsed_rfc(self):
        self.assertEqual(sorted(LITERAL_PROVENANCE_FIELDS),
                         list(c.parse_rfc_provenance_fields()))
        self.assertEqual(list(c.parse_rfc_provenance_fields()),
                         list(c.parse_contract_provenance_fields()))

    def test_rfc_nullable_mutation_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, 'rfc.md')
            write_text(p, mutate_rfc_nullable(c.RFC_PATH))
            rep = c.check_semantic_binding(p, c.CONTRACT_PATH)
            self.assertFalse(rep['ok'], rep['errors'])

    def test_contract_nullable_mutation_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, 'contract.md')
            write_text(p, mutate_contract_nullable(c.CONTRACT_PATH))
            rep = c.check_semantic_binding(c.RFC_PATH, p)
            self.assertFalse(rep['ok'])

    def test_rfc_type_mutation_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, 'rfc.md')
            write_text(p, mutate_rfc_type(c.RFC_PATH))
            rep = c.check_semantic_binding(p, c.CONTRACT_PATH)
            self.assertFalse(rep['ok'], rep['errors'])

    def test_rfc_type_removal_fails(self):
        # R14: removing a semantic key from the normative block makes the
        # schema structurally invalid (fail-closed load failure)
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, 'rfc.md')
            write_text(p, mutate_rfc_type_removal(c.RFC_PATH))
            rep = c.check_semantic_binding(p, c.CONTRACT_PATH)
            self.assertFalse(rep['ok'])
            schema, errs = c.load_normative_schema(p)
            self.assertIsNone(schema)
            self.assertTrue(errs)

    def test_contract_type_mutation_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, 'contract.md')
            write_text(p, mutate_contract_type(c.CONTRACT_PATH))
            rep = c.check_semantic_binding(c.RFC_PATH, p)
            self.assertFalse(rep['ok'])

    def test_rfc_requiredness_mutation_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, 'rfc.md')
            write_text(p, mutate_rfc_requiredness(c.RFC_PATH))
            rep = c.check_semantic_binding(p, c.CONTRACT_PATH)
            self.assertFalse(rep['ok'])

    def test_contract_requiredness_mutation_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, 'contract.md')
            write_text(p, mutate_contract_requiredness(c.CONTRACT_PATH))
            rep = c.check_semantic_binding(c.RFC_PATH, p)
            self.assertFalse(rep['ok'])

    def test_field_add_remove_mutations_fail(self):
        # R14: prose field add/remove is caught by the NAME binding
        # (prose table vs normative block), reported via the umbrella
        # provenance_binding; block mutations are caught by the
        # semantic binding.
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, 'rfc.md')
            write_text(p, mutate_rfc_add_field(c.RFC_PATH))
            self.assertFalse(c.check_provenance_binding(p,
                                                         c.CONTRACT_PATH)[
                                                             'ok'])
            write_text(p, mutate_rfc_remove_field(c.RFC_PATH))
            self.assertFalse(c.check_provenance_binding(p,
                                                         c.CONTRACT_PATH)[
                                                             'ok'])
            q = os.path.join(tmp, 'contract.md')
            write_text(q, mutate_contract_add_field(c.CONTRACT_PATH))
            self.assertFalse(c.check_provenance_binding(c.RFC_PATH, q)[
                'ok'])
            write_text(q, mutate_contract_remove_field(c.CONTRACT_PATH))
            self.assertFalse(c.check_provenance_binding(c.RFC_PATH, q)[
                'ok'])


# ---------------------------------------------------------------------------
# R13-C unit attacks: raw nested structure (audit B3)
# ---------------------------------------------------------------------------

class TestR13CRawNestedAttacks(unittest.TestCase):

    def test_valid_raw_passes(self):
        rep = c.check_raw_storage(literal_raw())
        self.assertTrue(rep['ok'], rep['errors'])

    def test_not_a_wrench_fails(self):
        raw = copy.deepcopy(literal_raw())
        raw[0]['contacts'][0]['wrenches'] = ['not-a-wrench']
        rep = c.check_raw_storage(raw)
        self.assertFalse(rep['ok'])

    def test_wrench_required_field_missing_fails(self):
        raw = copy.deepcopy(literal_raw())
        del raw[0]['contacts'][0]['wrenches'][0]['body_1_wrench']
        self.assertFalse(c.check_raw_storage(raw)['ok'])
        raw = copy.deepcopy(literal_raw())
        del raw[0]['contacts'][0]['wrenches'][0]['body_1_wrench']['force']
        self.assertFalse(c.check_raw_storage(raw)['ok'])
        raw = copy.deepcopy(literal_raw())
        del raw[0]['contacts'][0]['wrenches'][0]['body_1_wrench'][
            'force_offset']
        self.assertFalse(c.check_raw_storage(raw)['ok'])
        raw = copy.deepcopy(literal_raw())
        del raw[0]['contacts'][0]['wrenches'][0]['body_2_name']
        self.assertFalse(c.check_raw_storage(raw)['ok'])

    def test_wrench_vector_wrong_shape_fails(self):
        raw = copy.deepcopy(literal_raw())
        raw[0]['contacts'][0]['wrenches'][0]['body_1_wrench']['force'] = \
            [0.0, 25.5]
        self.assertFalse(c.check_raw_storage(raw)['ok'])
        raw = copy.deepcopy(literal_raw())
        w = raw[0]['contacts'][0]['wrenches'][0]['body_2_wrench']
        w['torque'] = [0.0, 0.0, float('inf')]
        self.assertFalse(c.check_raw_storage(raw)['ok'])
        raw = copy.deepcopy(literal_raw())
        raw[0]['contacts'][0]['wrenches'][0]['body_1_wrench']['force'] = \
            'not-a-vec3'
        self.assertFalse(c.check_raw_storage(raw)['ok'])

    def test_not_a_pose_entry_fails(self):
        raw = copy.deepcopy(literal_raw())
        raw[2]['entries'] = ['not-a-pose-entry']
        self.assertFalse(c.check_raw_storage(raw)['ok'])

    def test_pose_required_field_missing_fails(self):
        raw = copy.deepcopy(literal_raw())
        raw[2]['entries'][0] = {'name': 'nao'}     # position missing
        self.assertFalse(c.check_raw_storage(raw)['ok'])
        raw = copy.deepcopy(literal_raw())
        raw[2]['entries'][0] = {'position': [0.5, -0.28, 0.33]}  # name missing
        self.assertFalse(c.check_raw_storage(raw)['ok'])
        raw = copy.deepcopy(literal_raw())
        raw[2]['entries'][0]['orientation'] = [0.0, 0.0, 1.0]  # quat len 3
        self.assertFalse(c.check_raw_storage(raw)['ok'])

    def test_contact_identity_missing_fails(self):
        raw = copy.deepcopy(literal_raw())
        del raw[0]['contacts'][0]['collision1']
        self.assertFalse(c.check_raw_storage(raw)['ok'])
        raw = copy.deepcopy(literal_raw())
        raw[0]['contacts'][0]['collision2'] = ''
        self.assertFalse(c.check_raw_storage(raw)['ok'])

    def test_contact_point_arrays_drift_fails(self):
        raw = copy.deepcopy(literal_raw())
        raw[0]['contacts'][0]['position'] = [[0.1, 0.05]]
        self.assertFalse(c.check_raw_storage(raw)['ok'])
        raw = copy.deepcopy(literal_raw())
        raw[0]['contacts'][0]['normal'] = []
        self.assertFalse(c.check_raw_storage(raw)['ok'])
        raw = copy.deepcopy(literal_raw())
        raw[0]['contacts'][0]['depth'] = [0.001, 0.002]  # length mismatch
        self.assertFalse(c.check_raw_storage(raw)['ok'])

    def test_joint_command_attacks_fail(self):
        raw = copy.deepcopy(literal_raw())
        raw[3]['command'] = [0.1] * 23            # wrong dimension
        self.assertFalse(c.check_raw_storage(raw)['ok'])
        raw = copy.deepcopy(literal_raw())
        raw[3]['command'] = [0.1] * 24 + ['x']    # 25, non-numeric tail
        self.assertFalse(c.check_raw_storage(raw)['ok'])
        raw = copy.deepcopy(literal_raw())
        raw[3]['command'] = ['not-a-24-vector']
        self.assertFalse(c.check_raw_storage(raw)['ok'])
        raw = copy.deepcopy(literal_raw())
        raw[3]['command'] = [0.1] * 23 + [True]   # boolean element
        self.assertFalse(c.check_raw_storage(raw)['ok'])

    def test_empty_contacts_list_is_structurally_valid(self):
        # an explicit EMPTY Contacts message is valid structure (silence
        # semantics are a different, non-structural concern)
        raw = copy.deepcopy(literal_raw())
        raw[0]['contacts'] = []
        raw[0]['n_contacts'] = 0
        rep = c.check_raw_storage(raw)
        self.assertTrue(rep['ok'], rep['errors'])

    def test_unknown_native_keys_preserved_not_rejected(self):
        # lossless retention (§ 15): unknown/optional native keys are fine
        raw = copy.deepcopy(literal_raw())
        raw[0]['header'] = {'stamp': 1.013}
        raw[0]['contacts'][0]['world'] = {'name': 'world'}
        raw[0]['contacts'][0]['wrenches'][0]['body_1_id'] = 8
        raw[2]['entries'][0]['id'] = 26
        raw[3]['extra_native_field'] = [1, 2, 3]
        rep = c.check_raw_storage(raw)
        self.assertTrue(rep['ok'], rep['errors'])


# ---------------------------------------------------------------------------
# R13-D unit attacks: provenance type/range (audit B4)
# ---------------------------------------------------------------------------

class TestR13DProvenanceAttacks(unittest.TestCase):

    def test_valid_provenance_passes(self):
        self.assertTrue(c.check_provenance(literal_provenance())['ok'])

    def test_boolean_gazebo_seed_fails(self):
        for v in (True, False):
            rec = literal_provenance()
            rec['gazebo_seed'] = v
            rep = c.check_provenance(rec)
            self.assertFalse(rep['ok'], 'gazebo_seed=%r accepted' % v)

    def test_negative_and_boolean_frame_count_fail(self):
        for v in (-1, True, 1.5):
            rec = literal_provenance()
            rec['frame_count'] = v
            rep = c.check_provenance(rec)
            self.assertFalse(rep['ok'], 'frame_count=%r accepted' % v)
        rec = literal_provenance()
        rec['frame_count'] = 0
        self.assertTrue(c.check_provenance(rec)['ok'])

    def test_non_finite_timestamps_fail(self):
        for field, v in (('t_start', float('nan')),
                         ('t_start', float('inf')),
                         ('t_end', float('-inf'))):
            rec = literal_provenance()
            rec[field] = v
            rep = c.check_provenance(rec)
            self.assertFalse(rep['ok'], '%s=%r accepted' % (field, v))

    def test_invalid_time_ordering_fails(self):
        rec = literal_provenance()
        rec['t_start'] = 60.5
        rec['t_end'] = 0.5
        rep = c.check_provenance(rec)
        self.assertFalse(rep['ok'])

    def test_boolean_numeric_elsewhere_rejected(self):
        rec = literal_provenance()
        rec['t_start'] = True     # bool is not a finite number
        self.assertFalse(c.check_provenance(rec)['ok'])

    def test_json_nan_roundtrip_reachable(self):
        # The parser (json) accepts non-standard NaN; the checker must
        # reject it — the attack is reachable, not an unreachable test.
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, 'provenance.json')
            rec = literal_provenance()
            rec['t_start'] = float('nan')
            with open(p, 'w', encoding='utf-8') as fh:
                json.dump(rec, fh)
            with open(p, 'r', encoding='utf-8') as fh:
                loaded = json.load(fh)
            self.assertNotEqual(loaded['t_start'], loaded['t_start'])  # NaN
            self.assertFalse(c.check_provenance(loaded)['ok'])


# ---------------------------------------------------------------------------
# R6-R10 regression invariants (§ 24)
# ---------------------------------------------------------------------------

class TestR6toR10Regression(unittest.TestCase):

    def test_rfc_hash_pinned(self):
        # R14: the RFC was modified exactly once (REV1.5, Appendix D
        # normative schema block) — pinned to the R14-committed hash.
        import hashlib
        h = hashlib.sha256(
            open(c.RFC_PATH, 'rb').read()).hexdigest()
        self.assertEqual(
            h, '4f92c9bf2c6496d9cbf092775d795854ec7b98e535fd52124709da7d'
               '307458d7')

    def test_r6_no_cadence_band_in_probe(self):
        text = read(PROBE)
        self.assertNotIn('CADENCE_MIN_HZ', text)
        self.assertNotIn('CADENCE_MAX_HZ', text)
        self.assertIn('contact_publication_events', text)

    def test_r7_two_state_vocabulary(self):
        text = read(c.RFC_PATH)
        self.assertIn('UNKNOWN_NO_MESSAGE', text)
        self.assertIn('VALID_CONTACT', text)
        self.assertIn('future extension', text)
        self.assertIn('no fabricated flight labels', text)

    def test_r8_composition_remains_unverified(self):
        self.assertEqual(c.COMPOSITION_STATUS_UNVERIFIED, 'UNVERIFIED')
        rep = c.check_pending_propagation(
            dict(LITERAL_FIELD_STATUS), composition_status='VERIFIED')
        self.assertNotEqual(rep['ok'], True)

    def test_r10_association_compatibility(self):
        text = read(PROBE)
        self.assertIn('timestamp_association_compatibility', text)
        rfc = read(c.RFC_PATH)
        self.assertIn('universal clock identity', rfc)


# ---------------------------------------------------------------------------
# § 23 end-to-end smoke matrix — every FAIL row via the ACTUAL CLI path
# ---------------------------------------------------------------------------

class TestR13SmokeMatrixCLI(unittest.TestCase):
    """The mandatory § 23 matrix. Every FAIL case must exit != 0 through
    the real --smoke CLI (never a helper-only assertion)."""

    def _run_case(self, name, manifest=None, provenance=None, frame=None,
                  raw=None, rfc_text=None, contract_text=None):
        with tempfile.TemporaryDirectory() as tmp:
            # callers pass the frame artifact as an ARRAY of frames (R14)
            args = write_case(tmp, manifest=manifest, provenance=provenance,
                              frame=frame, raw=raw, rfc_text=rfc_text,
                              contract_text=contract_text)
            proc = run_cli(args)
            return proc

    def test_matrix_valid_passes_exit_0(self):
        proc = self._run_case('valid', manifest=literal_manifest(),
                              provenance=literal_provenance(),
                              frame=[literal_frame()], raw=literal_raw())
        self.assertEqual(proc.returncode, 0, proc.stdout[-3000:])
        payload = json.loads(proc.stdout)
        self.assertTrue(payload['smoke']['ok'])
        self.assertEqual(payload['smoke']['failed_checks'], [])

    def test_matrix_missing_artifacts_fail(self):
        for name, kwargs in (
                ('missing manifest', dict(provenance=literal_provenance(),
                                          frame=[literal_frame()],
                                          raw=literal_raw())),
                ('missing provenance', dict(manifest=literal_manifest(),
                                            frame=[literal_frame()],
                                            raw=literal_raw())),
                ('missing frame', dict(manifest=literal_manifest(),
                                       provenance=literal_provenance(),
                                       raw=literal_raw())),
                ('missing raw', dict(manifest=literal_manifest(),
                                     provenance=literal_provenance(),
                                     frame=[literal_frame()]))):
            proc = self._run_case(name, **kwargs)
            expect_fail(name, proc)

    def test_matrix_option_a_attacks_fail(self):
        cases = {}
        f = copy.deepcopy(literal_frame())
        del f['left_foot_pose']
        cases['Option-A missing'] = f
        f = copy.deepcopy(literal_frame())
        f['com_position'] = None
        cases['Option-A null'] = f
        f = copy.deepcopy(literal_frame())
        f['com_position'] = 'not-a-vector'
        cases['Option-A wrong type'] = f
        f = copy.deepcopy(literal_frame())
        f['com_position'] = [0.1, 0.2]
        cases['Option-A wrong shape'] = f
        f = copy.deepcopy(literal_frame())
        f['cop_position'] = [0.31, 0.0]
        cases['Option-B non-null'] = f
        for name, frame in cases.items():
            proc = self._run_case(name, manifest=literal_manifest(),
                                  provenance=literal_provenance(),
                                  frame=[frame], raw=literal_raw())
            expect_fail(name, proc)
            payload = json.loads(proc.stdout)
            self.assertIn('frame_emission', payload['smoke']['failed_checks'])

    def test_matrix_wrong_option_a_status_fails(self):
        m = literal_manifest()
        m['field_status']['left_foot_velocity'] = 'derived'
        proc = self._run_case('wrong Option-A status', manifest=m,
                              provenance=literal_provenance(),
                              frame=[literal_frame()], raw=literal_raw())
        expect_fail('wrong Option-A status', proc)

    def test_matrix_unknown_status_fails(self):
        m = literal_manifest()
        m['field_status']['cop_position'] = 'UNRECOGNIZED_STATUS'
        proc = self._run_case('unknown status', manifest=m,
                              provenance=literal_provenance(),
                              frame=[literal_frame()], raw=literal_raw())
        expect_fail('unknown status', proc)

    def test_matrix_field_mutations_fail(self):
        for name, kw in (
                ('RFC field add', dict(rfc_text=mutate_rfc_add_field(
                    c.RFC_PATH))),
                ('Contract field add', dict(contract_text=mutate_contract_add_field(
                    c.CONTRACT_PATH))),
                ('RFC field remove', dict(rfc_text=mutate_rfc_remove_field(
                    c.RFC_PATH))),
                ('Contract field remove',
                 dict(contract_text=mutate_contract_remove_field(
                     c.CONTRACT_PATH)))):
            proc = self._run_case(name, manifest=literal_manifest(),
                                  provenance=literal_provenance(),
                                  frame=[literal_frame()], raw=literal_raw(),
                                  **kw)
            expect_fail(name, proc)
            payload = json.loads(proc.stdout)
            self.assertIn('provenance_binding', payload['smoke'][
                'failed_checks'])

    def test_matrix_semantic_mutations_fail(self):
        for name, kw in (
                ('RFC nullable mutation',
                 dict(rfc_text=mutate_rfc_nullable(c.RFC_PATH))),
                ('Contract nullable mutation',
                 dict(contract_text=mutate_contract_nullable(
                     c.CONTRACT_PATH))),
                ('RFC type mutation',
                 dict(rfc_text=mutate_rfc_type(c.RFC_PATH))),
                ('Contract type mutation',
                 dict(contract_text=mutate_contract_type(
                     c.CONTRACT_PATH))),
                ('RFC requiredness mutation',
                 dict(rfc_text=mutate_rfc_requiredness(c.RFC_PATH))),
                ('Contract requiredness mutation',
                 dict(contract_text=mutate_contract_requiredness(
                     c.CONTRACT_PATH)))):
            proc = self._run_case(name, manifest=literal_manifest(),
                                  provenance=literal_provenance(),
                                  frame=[literal_frame()], raw=literal_raw(),
                                  **kw)
            expect_fail(name, proc)
            payload = json.loads(proc.stdout)
            self.assertIn('provenance_binding', payload['smoke'][
                'failed_checks'])

    def test_matrix_raw_nested_corruption_fails(self):
        raw = copy.deepcopy(literal_raw())
        raw[0]['contacts'][0]['wrenches'] = ['not-a-wrench']
        proc = self._run_case('malformed wrench', manifest=literal_manifest(),
                              provenance=literal_provenance(),
                              frame=[literal_frame()], raw=raw)
        expect_fail('malformed wrench', proc)
        raw = copy.deepcopy(literal_raw())
        raw[2]['entries'] = ['not-a-pose-entry']
        proc = self._run_case('malformed pose entry',
                              manifest=literal_manifest(),
                              provenance=literal_provenance(),
                              frame=[literal_frame()], raw=raw)
        expect_fail('malformed pose entry', proc)
        raw = copy.deepcopy(literal_raw())
        raw[3]['command'] = [0.1] * 23
        proc = self._run_case('malformed joint command',
                              manifest=literal_manifest(),
                              provenance=literal_provenance(),
                              frame=[literal_frame()], raw=raw)
        expect_fail('malformed joint command', proc)

    def test_matrix_provenance_corruption_fails(self):
        for name, mutate in (
                ('boolean gazebo_seed',
                 lambda r: r.update(gazebo_seed=True)),
                ('negative frame_count',
                 lambda r: r.update(frame_count=-1)),
                ('non-finite timestamp',
                 lambda r: r.update(t_start=float('nan'))),
                ('invalid time ordering',
                 lambda r: r.update(t_start=60.5, t_end=0.5))):
            rec = literal_provenance()
            mutate(rec)
            proc = self._run_case(name, manifest=literal_manifest(),
                                  provenance=rec, frame=[literal_frame()],
                                  raw=literal_raw())
            expect_fail(name, proc)
            payload = json.loads(proc.stdout)
            self.assertIn('provenance', payload['smoke']['failed_checks'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
