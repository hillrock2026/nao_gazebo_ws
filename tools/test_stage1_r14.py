#!/usr/bin/env python3
"""R14 normative-schema-binding and artifact-consistency tests (REV1.5).

Run:  python3 tools/test_stage1_r14.py

Attack-driven (R14 task § 10-§ 12): the M1-M14 mutation matrix runs
through the REAL --smoke CLI. The core independence evidence (§ 11) is
"mutated RFC (normative block) -> UNCHANGED checker -> smoke -> FAIL":
the checker loads its schema from the RFC's Appendix D at run time, so an
RFC schema mutation changes the enforced contract and the old artifact no
longer passes. Fixtures are hand-written literals (no checker-constant
generation).

No Gazebo acquisition, no network, no external service; synthetic
fixtures and temporary mutation copies only.
"""

import copy
import json
import os
import subprocess
import sys
import tempfile
import unittest

import stage1_contract_check as c
import stage1_git_guard as git_guard

TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(TOOLS)
CHECKER = os.path.join(TOOLS, 'stage1_contract_check.py')
PROBE = os.path.join(TOOLS, 'stage1_probe.py')


# ---------------------------------------------------------------------------
# Hand-written literal fixtures (§ 11 independence)
# ---------------------------------------------------------------------------

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


def literal_provenance(frame_count=1):
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
        'frame_count': frame_count,
        't_start': 0.5,
        't_end': 60.5,
        'episode_integrity_hash': 'd' * 64,
    }


def literal_frame():
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


def write_case(dirname, manifest=None, provenance=None, frame=None, raw=None,
               rfc_text=None, contract_text=None):
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


def read(path):
    with open(path, 'r', encoding='utf-8') as fh:
        return fh.read()


def write_text(path, text):
    with open(path, 'w', encoding='utf-8') as fh:
        fh.write(text)
    return text


def expect_fail(name, proc):
    assert proc.returncode != 0, \
        '%s: expected exit != 0, got 0\n%s' % (name, proc.stdout[-3000:])
    payload = json.loads(proc.stdout)
    assert payload['smoke']['ok'] is False, \
        '%s: exit non-zero but smoke ok=true' % name
    return payload


# ---------------------------------------------------------------------------
# Normative-block mutations (§ 10 M1-M7; the checker is UNCHANGED — the
# mutated RFC Appendix D IS the new enforced contract)
# ---------------------------------------------------------------------------

def _mutate_doc(src, old, new, count=1):
    text = read(src)
    assert text.count(old) == count, \
        'mutation anchor %r not unique (count=%d)' % (old[:60],
                                                      text.count(old))
    return text.replace(old, new, count)


def mutate_schema_com_position_len(src, length=4):
    # M1: com_position vector length 3 -> N (RFC Appendix D)
    return _mutate_doc(
        src, '{"name": "com_position", "type": "vector", "length": 3},',
        '{"name": "com_position", "type": "vector", "length": %d},' % length)


def mutate_schema_pose_orientation_len(src, length=5):
    # M2: left_foot_pose orientation 4 -> N (RFC Appendix D)
    return _mutate_doc(
        src, '{"name": "left_foot_pose", "type": "pose", "position": 3, '
             '"orientation": 4},',
        '{"name": "left_foot_pose", "type": "pose", "position": 3, '
        '"orientation": %d},' % length)


def mutate_schema_zmp_len(src, length=4):
    # M3: zmp_position vector length 2 -> N (RFC Appendix D)
    return _mutate_doc(
        src, '{"name": "zmp_position", "type": "vector", "length": 2}',
        '{"name": "zmp_position", "type": "vector", "length": %d}'
        % length)


def mutate_schema_cop_nullable_false(src):
    # M4: cop_derivation_hash nullable true -> false ("non-null" semantics
    # as an explicit flag — never a prose substring)
    return _mutate_doc(
        src, '{"name": "cop_derivation_hash", "required": true, '
             '"nullable": true, "type": "sha256"}',
        '{"name": "cop_derivation_hash", "required": true, '
        '"nullable": false, "type": "sha256"}')


def mutate_schema_ring_optional(src):
    # M5: required -> optional (explicit flag)
    return _mutate_doc(
        src, '{"name": "ring", "required": true, "nullable": false, '
             '"type": "string"}',
        '{"name": "ring", "required": false, "nullable": false, '
        '"type": "string"}')


def mutate_schema_world_required(src):
    # M6: optional -> required (contact_optional_keys 'world' moved into
    # contact_required_keys — artifacts lacking 'world' must fail)
    text = read(src)
    old_req = ('"contact_required_keys": ["header", "collision1", '
               '"collision2", "position", "normal", "depth", "wrenches"],')
    new_req = ('"contact_required_keys": ["header", "collision1", '
               '"collision2", "position", "normal", "depth", "wrenches", '
               '"world"],')
    old_opt = '"contact_optional_keys": ["world"],'
    new_opt = '"contact_optional_keys": [],'
    assert text.count(old_req) == 1 and text.count(old_opt) == 1
    return text.replace(old_req, new_req, 1).replace(old_opt, new_opt, 1)


def mutate_schema_theta_type(src):
    # M7: type mutation (theta object -> string)
    return _mutate_doc(
        src, '{"name": "theta", "required": true, "nullable": false, '
             '"type": "object"}',
        '{"name": "theta", "required": true, "nullable": false, '
        '"type": "string"}')


# ---------------------------------------------------------------------------
# § 23-equivalent CLI matrix (R14): every FAIL via the REAL --smoke path
# ---------------------------------------------------------------------------

class TestR14SmokeMatrixCLI(unittest.TestCase):

    def _case(self, name, manifest=None, provenance=None, frame=None,
              raw=None, rfc_text=None, contract_text=None):
        with tempfile.TemporaryDirectory() as tmp:
            args = write_case(tmp, manifest=manifest,
                              provenance=provenance, frame=frame, raw=raw,
                              rfc_text=rfc_text, contract_text=contract_text)
            return run_cli(args)

    # --- positive --------------------------------------------------------

    def test_valid_fixture_passes_exit_0(self):
        proc = self._case('valid', manifest=literal_manifest(),
                          provenance=literal_provenance(frame_count=1),
                          frame=[literal_frame()], raw=literal_raw())
        self.assertEqual(proc.returncode, 0, proc.stdout[-3000:])
        payload = json.loads(proc.stdout)
        self.assertTrue(payload['smoke']['ok'])
        self.assertEqual(payload['smoke']['failed_checks'], [])

    def test_unknown_extra_native_field_still_passes(self):
        raw = copy.deepcopy(literal_raw())
        raw[0]['contacts'][0]['world'] = {'name': 'world'}
        raw[0]['contacts'][0]['native_extension'] = [1, 2, 3]
        raw[3]['extra_native_field'] = {'anything': True}
        proc = self._case('unknown native fields', manifest=literal_manifest(),
                          provenance=literal_provenance(frame_count=1),
                          frame=[literal_frame()], raw=raw)
        self.assertEqual(proc.returncode, 0, proc.stdout[-3000:])

    # --- M1-M3: RFC normative schema shape mutations ----------------------

    def test_m1_rfc_com_position_shape_mutation_fails(self):
        proc = self._case('M1', manifest=literal_manifest(),
                          provenance=literal_provenance(frame_count=1),
                          frame=[literal_frame()], raw=literal_raw(),
                          rfc_text=mutate_schema_com_position_len(
                              c.RFC_PATH, length=4))
        payload = expect_fail('M1', proc)
        self.assertIn('frame_emission', payload['smoke']['failed_checks'])

    def test_m2_rfc_pose_orientation_shape_mutation_fails(self):
        proc = self._case('M2', manifest=literal_manifest(),
                          provenance=literal_provenance(frame_count=1),
                          frame=[literal_frame()], raw=literal_raw(),
                          rfc_text=mutate_schema_pose_orientation_len(
                              c.RFC_PATH, length=5))
        payload = expect_fail('M2', proc)
        self.assertIn('frame_emission', payload['smoke']['failed_checks'])

    def test_m3_rfc_zmp_shape_mutation_fails(self):
        proc = self._case('M3', manifest=literal_manifest(),
                          provenance=literal_provenance(frame_count=1),
                          frame=[literal_frame()], raw=literal_raw(),
                          rfc_text=mutate_schema_zmp_len(c.RFC_PATH,
                                                         length=4))
        payload = expect_fail('M3', proc)
        self.assertIn('frame_emission', payload['smoke']['failed_checks'])

    # --- M4-M7: RFC normative semantic mutations --------------------------

    def test_m4_rfc_nullable_mutation_fails(self):
        # nullable -> non-null: the artifact's cop_derivation_hash=null now
        # violates the mutated non-nullable schema (B2: explicit flag, not
        # a prose substring — "non-null" prose could never be misread)
        proc = self._case('M4', manifest=literal_manifest(),
                          provenance=literal_provenance(frame_count=1),
                          frame=[literal_frame()], raw=literal_raw(),
                          rfc_text=mutate_schema_cop_nullable_false(
                              c.RFC_PATH))
        payload = expect_fail('M4', proc)
        self.assertIn('provenance', payload['smoke']['failed_checks'])
        self.assertIn('provenance_binding',
                      payload['smoke']['failed_checks'])

    def test_m5_rfc_required_to_optional_mutation_fails(self):
        proc = self._case('M5', manifest=literal_manifest(),
                          provenance=literal_provenance(frame_count=1),
                          frame=[literal_frame()], raw=literal_raw(),
                          rfc_text=mutate_schema_ring_optional(c.RFC_PATH))
        payload = expect_fail('M5', proc)
        self.assertIn('provenance_binding', payload['smoke']['failed_checks'])

    def test_m6_rfc_optional_to_required_mutation_fails(self):
        # 'world' becomes a required contact key: artifacts without it fail
        proc = self._case('M6', manifest=literal_manifest(),
                          provenance=literal_provenance(frame_count=1),
                          frame=[literal_frame()], raw=literal_raw(),
                          rfc_text=mutate_schema_world_required(c.RFC_PATH))
        payload = expect_fail('M6', proc)
        self.assertIn('raw_storage', payload['smoke']['failed_checks'])
        self.assertIn('provenance_binding', payload['smoke']['failed_checks'])

    def test_m7_rfc_type_mutation_fails(self):
        proc = self._case('M7', manifest=literal_manifest(),
                          provenance=literal_provenance(frame_count=1),
                          frame=[literal_frame()], raw=literal_raw(),
                          rfc_text=mutate_schema_theta_type(c.RFC_PATH))
        payload = expect_fail('M7', proc)
        self.assertIn('provenance', payload['smoke']['failed_checks'])

    def test_contract_block_mutation_fails(self):
        # mutating the CONTRACT's § 8.4 block copy (any of the same
        # mutations) -> block inequality -> FAIL
        for name, mutate in (
                ('contract nullable', mutate_schema_cop_nullable_false),
                ('contract required->optional', mutate_schema_ring_optional),
                ('contract type', mutate_schema_theta_type)):
            proc = self._case(name, manifest=literal_manifest(),
                              provenance=literal_provenance(frame_count=1),
                              frame=[literal_frame()], raw=literal_raw(),
                              contract_text=mutate(c.CONTRACT_PATH))
            payload = expect_fail(name, proc)
            self.assertIn('provenance_binding',
                          payload['smoke']['failed_checks'])

    # --- M8-M11: RFC-required raw field deletion from the artifact -------

    def test_m8_delete_body_1_id_fails(self):
        raw = copy.deepcopy(literal_raw())
        del raw[0]['contacts'][0]['wrenches'][0]['body_1_id']
        proc = self._case('M8', manifest=literal_manifest(),
                          provenance=literal_provenance(frame_count=1),
                          frame=[literal_frame()], raw=raw)
        payload = expect_fail('M8', proc)
        self.assertIn('raw_storage', payload['smoke']['failed_checks'])

    def test_m9_delete_body_2_id_fails(self):
        raw = copy.deepcopy(literal_raw())
        del raw[1]['contacts'][0]['wrenches'][0]['body_2_id']
        proc = self._case('M9', manifest=literal_manifest(),
                          provenance=literal_provenance(frame_count=1),
                          frame=[literal_frame()], raw=raw)
        payload = expect_fail('M9', proc)
        self.assertIn('raw_storage', payload['smoke']['failed_checks'])

    def test_m10_delete_contact_header_fails(self):
        raw = copy.deepcopy(literal_raw())
        del raw[0]['contacts'][0]['header']
        proc = self._case('M10', manifest=literal_manifest(),
                          provenance=literal_provenance(frame_count=1),
                          frame=[literal_frame()], raw=raw)
        payload = expect_fail('M10', proc)
        self.assertIn('raw_storage', payload['smoke']['failed_checks'])

    def test_m11_delete_raw_pose_orientation_fails(self):
        raw = copy.deepcopy(literal_raw())
        del raw[2]['entries'][0]['orientation']
        proc = self._case('M11', manifest=literal_manifest(),
                          provenance=literal_provenance(frame_count=1),
                          frame=[literal_frame()], raw=raw)
        payload = expect_fail('M11', proc)
        self.assertIn('raw_storage', payload['smoke']['failed_checks'])

    # --- M12-M14: artifact count consistency (B4) --------------------------

    def test_m12_frame_count_zero_with_frame_fails(self):
        proc = self._case('M12', manifest=literal_manifest(),
                          provenance=literal_provenance(frame_count=0),
                          frame=[literal_frame()], raw=literal_raw())
        payload = expect_fail('M12', proc)
        self.assertIn('frame_emission', payload['smoke']['failed_checks'])

    def test_m13_frame_count_two_with_one_frame_fails(self):
        proc = self._case('M13', manifest=literal_manifest(),
                          provenance=literal_provenance(frame_count=2),
                          frame=[literal_frame()], raw=literal_raw())
        payload = expect_fail('M13', proc)
        self.assertIn('frame_emission', payload['smoke']['failed_checks'])

    def test_m14_empty_episode_passes(self):
        # frame_count=0 + frames=[] is a legal empty episode (never force
        # frame_count > 0 — the count must MATCH, § 15 error 2)
        proc = self._case('M14', manifest=literal_manifest(),
                          provenance=literal_provenance(frame_count=0),
                          frame=[], raw=literal_raw())
        self.assertEqual(proc.returncode, 0, proc.stdout[-3000:])
        payload = json.loads(proc.stdout)
        self.assertTrue(payload['smoke']['ok'])

    def test_frame_count_matches_two_frames_passes(self):
        proc = self._case('count=2', manifest=literal_manifest(),
                          provenance=literal_provenance(frame_count=2),
                          frame=[literal_frame(), literal_frame()],
                          raw=literal_raw())
        self.assertEqual(proc.returncode, 0, proc.stdout[-3000:])

    def test_frame_artifact_not_an_array_fails(self):
        proc = self._case('frame dict', manifest=literal_manifest(),
                          provenance=literal_provenance(frame_count=1),
                          frame=literal_frame(), raw=literal_raw())
        payload = expect_fail('frame dict', proc)
        self.assertIn('frame_emission', payload['smoke']['failed_checks'])

    # --- schema load failures (fail-closed, no silent fallback) -----------

    def test_missing_normative_block_fails_closed(self):
        with tempfile.TemporaryDirectory() as tmp:
            rfc_no_block = os.path.join(tmp, 'rfc_no_block.md')
            text = read(c.RFC_PATH)
            start = text.find('```json')
            end = text.find('```', start + 8)
            write_text(rfc_no_block, text[:start] + text[end + 3:])
            args = write_case(tmp, manifest=literal_manifest(),
                              provenance=literal_provenance(frame_count=1),
                              frame=[literal_frame()], raw=literal_raw(),
                              rfc_text=read(rfc_no_block))
            proc = run_cli(args)
            payload = expect_fail('no normative block', proc)
            self.assertIn('normative_schema',
                          payload['smoke']['failed_checks'])
            # every schema-driven check fails — no silent fallback
            self.assertIn('provenance', payload['smoke']['failed_checks'])
            self.assertIn('raw_storage', payload['smoke']['failed_checks'])


# ---------------------------------------------------------------------------
# Unit-level checks of the normative binding
# ---------------------------------------------------------------------------

class TestR14NormativeBinding(unittest.TestCase):

    def test_schema_loads_from_both_documents(self):
        schema, errs = c.load_normative_schema(c.RFC_PATH)
        self.assertFalse(errs)
        self.assertTrue(schema)
        schema2, errs2 = c.load_normative_schema(c.CONTRACT_PATH)
        self.assertFalse(errs2)
        self.assertEqual(json.dumps(schema, sort_keys=True),
                         json.dumps(schema2, sort_keys=True))

    def test_bindings_hold_on_repository_documents(self):
        self.assertTrue(c.check_rfc_binding()['ok'])
        self.assertTrue(c.check_contract_binding()['ok'])
        self.assertTrue(c.check_semantic_binding()['ok'])
        self.assertTrue(c.check_provenance_binding()['ok'])

    def test_checker_constants_derived_from_schema(self):
        # the checker owns no independent list: constants equal the block
        schema, _ = c.load_normative_schema(c.RFC_PATH)
        self.assertEqual(
            sorted(c.MANDATORY_PROVENANCE_FIELDS),
            sorted(f['name'] for f in schema['provenance_fields']))
        self.assertEqual(
            sorted(c.COMPOSITION_DEPENDENT_FIELDS),
            sorted(f['name'] for f in
                   schema['frame_fields']['option_a']['fields']))
        self.assertEqual(
            sorted(c.WRENCH_FRAME_DEPENDENT_FIELDS),
            sorted(f['name'] for f in
                   schema['frame_fields']['option_b']['fields']))
        self.assertEqual(c.COMMAND_VECTOR_DIM,
                         schema['raw_structures']['joint_command'][
                             'command_length'])

    def test_option_a_shapes_come_from_schema(self):
        # B1: the enforced shapes equal the normative block (not hardcoded)
        frame = literal_frame()
        frame['com_position'] = [0.1, 0.2, 0.3]
        rep = c.check_frame_emission(frame)
        self.assertTrue(rep['ok'], rep['errors'])
        frame['com_position'] = [0.1, 0.2]
        rep = c.check_frame_emission(frame)
        self.assertFalse(rep['ok'])
        # with a MUTATED schema (com_position length 4), the same [3]
        # value fails — proving the shape comes from the schema object.
        # R15-A: a mutated DOCUMENT can no longer be loaded (integrity
        # pin), so the mutation is applied to an in-memory copy of the
        # authorized schema; the schema-driven evidence is unchanged.
        schema, _ = c.load_normative_schema(c.RFC_PATH)
        schema = copy.deepcopy(schema)
        for f in schema['frame_fields']['option_a']['fields']:
            if f['name'] == 'com_position':
                f['length'] = 4
        frame = literal_frame()
        rep = c.check_frame_emission(frame, schema=schema)
        self.assertFalse(rep['ok'])
        self.assertTrue(any('com_position' in e for e in rep['errors']))

    def test_prose_semantics_never_parsed(self):
        # B2: "non-null" prose must not affect the machine semantics —
        # only the explicit block flag does.
        text = read(c.RFC_PATH)
        mutated = text.replace(
            'fixed **before first derivation**; `null` until then',
            'fixed **before first derivation**; must be non-null', 1)
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, 'rfc.md')
            write_text(p, mutated)
            schema, errs = c.load_normative_schema(p)
            self.assertFalse(errs)
            spec = [f for f in schema['provenance_fields']
                    if f['name'] == 'cop_derivation_hash'][0]
            self.assertTrue(spec['nullable'])  # prose change ignored

    def test_m6_world_required_enforced_by_mutated_schema(self):
        # R15-A: a mutated DOCUMENT can no longer be loaded (integrity
        # pin), so the world-required mutation is applied to an in-memory
        # copy of the authorized schema; the schema-driven requiredness
        # evidence is unchanged (raw structure requiredness comes from the
        # schema, not checker hardcoding).
        schema, _ = c.load_normative_schema(c.RFC_PATH)
        schema = copy.deepcopy(schema)
        schema['raw_structures']['contacts']['contact_required_keys'] = list(
            schema['raw_structures']['contacts']['contact_required_keys']) \
            + ['world']
        rep = c.check_raw_storage(literal_raw(), schema=schema)
        self.assertFalse(rep['ok'])
        self.assertTrue(any('world' in e for e in rep['errors']))


# ---------------------------------------------------------------------------
# R6-R10 regression invariants (§ 12)
# ---------------------------------------------------------------------------

class TestR6toR10Regression(unittest.TestCase):

    def test_rfc_hash_pinned_rev15(self):
        import hashlib
        h = hashlib.sha256(open(c.RFC_PATH, 'rb').read()).hexdigest()
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
        self.assertIn('no fabricated flight labels', text)

    def test_r8_composition_remains_unverified(self):
        schema, _ = c.load_normative_schema(c.RFC_PATH)
        self.assertEqual(schema['status_semantics']['composition_status'],
                         'UNVERIFIED')
        rep = c.check_pending_propagation(
            dict(LITERAL_FIELD_STATUS), composition_status='VERIFIED')
        self.assertNotEqual(rep['ok'], True)

    def test_r10_association_compatibility(self):
        text = read(PROBE)
        self.assertIn('timestamp_association_compatibility', text)
        rfc = read(c.RFC_PATH)
        self.assertIn('universal clock identity', rfc)

    def test_frozen_files_untouched(self):
        # scope guard: the frozen set and probe are byte-identical to the
        # R13 baseline commit (FULL sha). Shared fail-closed guard, called
        # once per protected path: a git subprocess failure / missing
        # baseline / timeout is a FAILURE, not a silent pass.
        for rel in git_guard.PROTECTED_PATHS:
            git_guard.assert_path_unchanged(
                WS, git_guard.FULL_BASELINE_SHA, rel)


if __name__ == '__main__':
    unittest.main(verbosity=2)
