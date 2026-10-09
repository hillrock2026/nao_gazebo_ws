#!/usr/bin/env python3
"""R15 minimal-hardening tests (audit B5/B6, REV1.5 + R15).

Run:  python3 tools/test_stage1_r15.py

Closes the two R14-audit Blockers:

  B5 (R15-A pin + R15-C invariant) — a SYNCHRONIZED unauthorized schema
     change to the RFC Appendix D block and the Contract § 8.4 block must
     FAIL even though the two copies still agree; frame_count ==
     len(frames) must be enforced unconditionally (runtime invariant, not
     a schema-declared rule).
  B6 (R15-B uniqueness + strict JSON) — a duplicate or missing normative
     block, malformed/duplicated fence markers, and duplicate JSON object
     keys (ANY depth, schema documents AND artifacts) must FAIL; ordinary
     example JSON and unknown native extra fields stay non-blocking.

All A/B/C enforcement evidence runs through the REAL --smoke CLI over
temporary mutated copies of the ACTUAL repository documents (the real
RFC/Contract are never modified). Duplicate-JSON-key fixtures are built
by RAW-TEXT surgery — never via Python dicts, which cannot even
represent duplicate keys (a dict-built "duplicate key" fixture would
silently collapse and test nothing).

Hand-written literal fixtures (no checker-constant generation).
No Gazebo acquisition, no network, no external service.
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

RFC_TEXT = open(c.RFC_PATH, 'r', encoding='utf-8').read()
CONTRACT_TEXT = open(c.CONTRACT_PATH, 'r', encoding='utf-8').read()

REAL_RFC = c.RFC_PATH
REAL_CONTRACT = c.CONTRACT_PATH


# ---------------------------------------------------------------------------
# Hand-written literal fixtures (independence: nothing derived from the
# checker's constants)
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
        'acquisition_wallclock_utc': '2026-10-09T00:00:00+00:00',
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


# ---------------------------------------------------------------------------
# Document mutation helpers (raw-text surgery on the ACTUAL documents; the
# repository files are never modified)
# ---------------------------------------------------------------------------

def replace_once(text, old, new):
    count = text.count(old)
    assert count == 1, \
        'mutation anchor %r not unique (count=%d)' % (old[:60], count)
    return text.replace(old, new, 1)


def sync_mutate(old, new):
    """Synchronized mutation of the RFC block AND the Contract block —
    the two copies remain equal (the B5 attack shape)."""
    return (replace_once(RFC_TEXT, old, new),
            replace_once(CONTRACT_TEXT, old, new))


def fenced_normative_block(text):
    start = text.find('```json')
    end = text.find('```', start + 8) + 3
    assert start >= 0 and end > start + 8, 'normative block not found'
    return text[start:end]


# Mutation anchors (all unique in the authorized documents at R15 time)
COUNT_RULE = '"count_must_equal": "frame_count",'
VERSION_LINE = '  "schema_version": "stage1-normative-schema/1.0",'
COMPOSITION_LINE = \
    '    "composition_status": "UNVERIFIED",'
FRAME_ARTIFACT_ANCHOR = '''  "frame_artifact": {
    "type": "array_of_frames",
    "count_must_equal": "frame_count",
    "empty_episode_allowed": true
  }
}'''


# ---------------------------------------------------------------------------
# CLI harness
# ---------------------------------------------------------------------------

def write_case(dirname, manifest=None, provenance=None, frame=None,
               raw=None, rfc_text=None, contract_text=None,
               frame_text=None, raw_text=None, missing_frame=False):
    """Prepare a smoke CLI case. frame_text/raw_text write RAW JSON text
    (used for duplicate-key artifacts); the object variants are dumped."""
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
    for name, text in (('frame', frame_text), ('raw', raw_text)):
        if text is None:
            continue
        p = os.path.join(dirname, '%s.json' % name)
        with open(p, 'w', encoding='utf-8') as fh:
            fh.write(text)
        args.extend(['--%s' % name, p])
    if missing_frame:
        args.extend(['--frame', os.path.join(dirname, 'does_not_exist.json')])
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
    return payload


def expect_pass(name, proc):
    assert proc.returncode == 0, \
        '%s: expected exit 0\n%s' % (name, proc.stdout[-3000:])
    payload = json.loads(proc.stdout)
    assert payload['smoke']['ok'] is True
    return payload


def collect_errors(obj, out=None):
    """All error strings anywhere in the smoke report (checks, sub-binds)."""
    if out is None:
        out = []
    if isinstance(obj, dict):
        for key, value in obj.items():
            if key == 'errors' and isinstance(value, list):
                out.extend(str(e) for e in value)
            else:
                collect_errors(value, out)
    elif isinstance(obj, list):
        for value in obj:
            collect_errors(value, out)
    return out


class SmokeCase(unittest.TestCase):
    """Shared CLI case runner: real docs by default; mutated copies when
    rfc_text/contract_text are given."""

    def _case(self, name, manifest=None, provenance=None, frame=None,
              raw=None, rfc_text=None, contract_text=None,
              frame_text=None, raw_text=None, missing_frame=False):
        with tempfile.TemporaryDirectory() as tmp:
            args = write_case(tmp, manifest=manifest,
                              provenance=provenance, frame=frame, raw=raw,
                              rfc_text=rfc_text, contract_text=contract_text,
                              frame_text=frame_text, raw_text=raw_text,
                              missing_frame=missing_frame)
            return run_cli(args)

    def _valid_case(self, frame_count=1, n_frames=1, rfc_text=None,
                    contract_text=None, frame_text=None, raw_text=None,
                    missing_frame=False, name='case'):
        frame = None if frame_text is not None \
            else [literal_frame() for _ in range(n_frames)]
        return self._case(
            name, manifest=literal_manifest(),
            provenance=literal_provenance(frame_count=frame_count),
            frame=frame, raw=None if raw_text is not None
            else literal_raw(),
            rfc_text=rfc_text, contract_text=contract_text,
            frame_text=frame_text, raw_text=raw_text,
            missing_frame=missing_frame)


# ---------------------------------------------------------------------------
# R15-A: authorized-schema integrity pin (audit B5)
# ---------------------------------------------------------------------------

class TestR15AIntegrityPin(SmokeCase):

    def test_authorized_documents_and_valid_artifacts_pass(self):
        proc = self._valid_case()
        expect_pass('authorized baseline', proc)

    def test_authorized_zero_frame_episode_passes(self):
        proc = self._valid_case(frame_count=0, n_frames=0)
        expect_pass('authorized empty episode', proc)

    def test_rfc_block_alone_mutated_fails(self):
        rfc, = (replace_once(RFC_TEXT, COUNT_RULE,
                             '"count_must_equal": "frame_records",'),)
        proc = self._valid_case(rfc_text=rfc)
        payload = expect_fail('RFC-only mutation', proc)
        errors = collect_errors(payload)
        self.assertTrue(any('integrity pin mismatch' in e for e in errors),
                        errors)

    def test_contract_block_alone_mutated_fails(self):
        contract = replace_once(
            CONTRACT_TEXT, COUNT_RULE, '"count_must_equal": "frame_records",')
        proc = self._valid_case(contract_text=contract)
        payload = expect_fail('Contract-only mutation', proc)
        errors = collect_errors(payload)
        self.assertTrue(any('integrity pin mismatch' in e for e in errors),
                        errors)
        # the error must LOCATE the contract document
        self.assertTrue(any('contract.md' in e for e in errors), errors)

    def test_synced_identical_unauthorized_schema_fails(self):
        # THE B5 attack: both copies mutated identically — they remain
        # canonically equal, so R14's block-equality check passes; the
        # R15-A pin must still reject them.
        rfc, contract = sync_mutate(
            COUNT_RULE, '"count_must_equal": "frame_records",')
        proc = self._valid_case(rfc_text=rfc, contract_text=contract)
        payload = expect_fail('synced unauthorized schema', proc)
        errors = collect_errors(payload)
        self.assertTrue(any('integrity pin mismatch' in e for e in errors),
                        errors)
        self.assertTrue(any('EVEN when the RFC and Contract copies agree'
                            in e for e in errors), errors)

    def test_synced_rule_removal_fails(self):
        # B5 variant: the count rule is deleted from BOTH documents
        rfc, contract = sync_mutate('"count_must_equal": "frame_count",\n',
                                    '')
        proc = self._valid_case(rfc_text=rfc, contract_text=contract,
                                frame_count=2, n_frames=1)
        payload = expect_fail('synced rule removal', proc)
        self.assertTrue(any('integrity pin mismatch' in e
                            for e in collect_errors(payload)))

    def test_synced_semantic_field_mutation_fails(self):
        # B5 generalization: any unauthorized schema semantic (a type
        # change), applied to both copies, fails
        rfc, contract = sync_mutate(
            '{"name": "theta", "required": true, "nullable": false, '
            '"type": "object"}',
            '{"name": "theta", "required": true, "nullable": false, '
            '"type": "string"}')
        proc = self._valid_case(rfc_text=rfc, contract_text=contract)
        payload = expect_fail('synced type mutation', proc)
        self.assertTrue(any('integrity pin mismatch' in e
                            for e in collect_errors(payload)))

    def test_unknown_schema_version_fails(self):
        # version policy: an unknown/unauthorized schema version is
        # rejected, never auto-accepted (even synced across both docs)
        rfc, contract = sync_mutate(
            'stage1-normative-schema/1.0', 'stage1-normative-schema/9.9')
        proc = self._valid_case(rfc_text=rfc, contract_text=contract)
        payload = expect_fail('unknown schema version', proc)
        errors = collect_errors(payload)
        self.assertTrue(any('is not the authorized' in e for e in errors),
                        errors)

    def test_pin_mismatch_vs_version_mismatch_distinguished(self):
        # the error messages must LOCATE the failure class. A version bump
        # changes the bytes too, so both facts are reported — the version
        # message identifies the class. A content tamper with the SAME
        # authorized version yields pin errors ONLY.
        rfc, contract = sync_mutate(
            'stage1-normative-schema/1.0', 'stage1-normative-schema/9.9')
        payload = expect_fail(
            'version', self._valid_case(rfc_text=rfc,
                                        contract_text=contract))
        errors = collect_errors(payload)
        self.assertTrue(any('is not the authorized' in e for e in errors))

        rfc, contract = sync_mutate(
            '{"name": "ring", "required": true, "nullable": false, '
            '"type": "string"}',
            '{"name": "ring", "required": false, "nullable": false, '
            '"type": "string"}')
        payload = expect_fail(
            'pin', self._valid_case(rfc_text=rfc, contract_text=contract))
        errors = collect_errors(payload)
        self.assertTrue(any('integrity pin mismatch' in e for e in errors))
        self.assertFalse(any('is not the authorized' in e for e in errors))

    def test_prose_only_edit_passes(self):
        # an unrelated prose edit that violates no existing prose/schema
        # consistency rule stays a PASS (documents are not hash-frozen
        # beyond the normative block; existing hash-pin tests govern
        # the repository state separately)
        rfc = RFC_TEXT + \
            '\n\nUnrelated prose appendix sentence (R15 test fixture).\n'
        contract = CONTRACT_TEXT + \
            '\n\nUnrelated prose appendix sentence (R15 test fixture).\n'
        proc = self._valid_case(rfc_text=rfc, contract_text=contract)
        expect_pass('prose-only edit', proc)


# ---------------------------------------------------------------------------
# R15-B: unique normative block + strict JSON (audit B6)
# ---------------------------------------------------------------------------

class TestR15BUniquenessAndStrictJson(SmokeCase):

    def test_unique_block_normal_rules_pass(self):
        # a normal, unique RFC schema block is handled by the normal rules
        proc = self._valid_case()
        expect_pass('unique block', proc)

    def test_rfc_duplicate_block_fails(self):
        rfc = RFC_TEXT + '\n' + fenced_normative_block(RFC_TEXT) + '\n'
        proc = self._valid_case(rfc_text=rfc)
        payload = expect_fail('RFC duplicate block', proc)
        errors = collect_errors(payload)
        self.assertTrue(any('duplicate normative schema blocks' in e
                            for e in errors), errors)

    def test_contract_duplicate_block_fails(self):
        contract = CONTRACT_TEXT + '\n' + \
            fenced_normative_block(CONTRACT_TEXT) + '\n'
        proc = self._valid_case(contract_text=contract)
        payload = expect_fail('Contract duplicate block', proc)
        errors = collect_errors(payload)
        self.assertTrue(any('duplicate normative schema blocks' in e
                            for e in errors), errors)

    def test_rfc_missing_block_fails(self):
        block = fenced_normative_block(RFC_TEXT)
        rfc = RFC_TEXT.replace(block, '', 1)
        proc = self._valid_case(rfc_text=rfc)
        payload = expect_fail('RFC missing block', proc)
        errors = collect_errors(payload)
        self.assertTrue(any('not found' in e for e in errors), errors)

    def test_contract_missing_block_fails(self):
        block = fenced_normative_block(CONTRACT_TEXT)
        contract = CONTRACT_TEXT.replace(block, '', 1)
        proc = self._valid_case(contract_text=contract)
        payload = expect_fail('Contract missing block', proc)
        errors = collect_errors(payload)
        self.assertTrue(any('not found' in e for e in errors), errors)

    def test_duplicate_opening_marker_fails(self):
        rfc = replace_once(RFC_TEXT, '```json\n{', '```json\n```json\n{')
        proc = self._valid_case(rfc_text=rfc)
        expect_fail('duplicate opening marker', proc)

    def test_duplicate_closing_marker_fails(self):
        block = fenced_normative_block(RFC_TEXT)
        rfc = RFC_TEXT.replace(block, block + '\n```', 1)
        proc = self._valid_case(rfc_text=rfc)
        payload = expect_fail('duplicate closing marker', proc)
        errors = collect_errors(payload)
        self.assertTrue(any('unterminated code fence' in e for e in errors),
                        errors)

    def test_marker_order_close_before_open_fails(self):
        rfc = replace_once(RFC_TEXT, '```json\n{', '```\n```json\n{')
        proc = self._valid_case(rfc_text=rfc)
        payload = expect_fail('close before open', proc)
        errors = collect_errors(payload)
        self.assertTrue(any('inside a bare-fenced region' in e
                            for e in errors), errors)

    def test_top_level_duplicate_json_key_fails(self):
        # RAW-TEXT duplicate key (never dict-built): both documents carry
        # the same ambiguous block; strict parsing must reject it BEFORE
        # any equality/pin comparison could make it look authorized
        rfc, contract = sync_mutate(
            VERSION_LINE, VERSION_LINE + '\n' + VERSION_LINE)
        proc = self._valid_case(rfc_text=rfc, contract_text=contract)
        payload = expect_fail('top-level duplicate key', proc)
        errors = collect_errors(payload)
        self.assertTrue(any('duplicate JSON object key' in e for e in errors),
                        errors)

    def test_nested_duplicate_json_key_fails(self):
        rfc, contract = sync_mutate(
            COMPOSITION_LINE,
            COMPOSITION_LINE + '\n' + COMPOSITION_LINE)
        proc = self._valid_case(rfc_text=rfc, contract_text=contract)
        payload = expect_fail('nested duplicate key', proc)
        errors = collect_errors(payload)
        self.assertTrue(any('duplicate JSON object key' in e for e in errors),
                        errors)

    def test_duplicate_key_weakening_count_rule_fails(self):
        # the B6b attack reproduced at the R14 baseline: a duplicate
        # "frame_artifact" key whose last-wins value drops the count rule,
        # applied to BOTH documents (blocks stay canonically equal)
        attack = FRAME_ARTIFACT_ANCHOR.replace(
            '  }\n}', '  },\n  "frame_artifact": {\n'
            '    "count_must_equal": "no_check"\n  }\n}')
        rfc, contract = sync_mutate(FRAME_ARTIFACT_ANCHOR, attack)
        proc = self._valid_case(rfc_text=rfc, contract_text=contract,
                                frame_count=2, n_frames=1)
        payload = expect_fail('duplicate key weakens count rule', proc)
        errors = collect_errors(payload)
        self.assertTrue(any('duplicate JSON object key' in e for e in errors),
                        errors)

    def test_schema_block_nan_rejected(self):
        # non-standard JSON constants are ambiguous — rejected, not parsed
        rfc, contract = sync_mutate('"command_length": 24',
                                    '"command_length": NaN')
        proc = self._valid_case(rfc_text=rfc, contract_text=contract)
        payload = expect_fail('NaN in schema block', proc)
        errors = collect_errors(payload)
        self.assertTrue(any('non-standard JSON constant' in e for e in errors),
                        errors)

    def test_unknown_native_extra_fields_still_pass(self):
        # lossless retention: unknown native extra keys in artifacts are
        # preserved, not rejected
        raw = copy.deepcopy(literal_raw())
        raw[0]['contacts'][0]['native_extension'] = [1, 2, 3]
        raw[3]['extra_native_field'] = {'anything': True}
        frame = literal_frame()
        frame['extra_native_field'] = {'kept': True}
        proc = self._case('unknown extra fields', manifest=literal_manifest(),
                          provenance=literal_provenance(frame_count=1),
                          frame=[frame], raw=raw)
        expect_pass('unknown native extra fields', proc)

    def test_artifact_duplicate_key_fails(self):
        # RAW-TEXT duplicate key in the frame artifact: last-wins could
        # smuggle a second value past the Option-B null check; strict
        # artifact loading rejects the ambiguity
        text = json.dumps([literal_frame()], indent=1)
        assert text.count('"cop_position": null') == 1
        text = text.replace(
            '"cop_position": null',
            '"cop_position": null, "cop_position": [0.1, 0.2]', 1)
        proc = self._valid_case(frame_text=text)
        payload = expect_fail('artifact duplicate key', proc)
        errors = collect_errors(payload)
        self.assertTrue(any('duplicate JSON object key' in e for e in errors)
                        or any('cop_position' in e for e in errors), errors)

    def test_raw_artifact_duplicate_key_fails(self):
        text = json.dumps(literal_raw(), indent=1)
        assert text.count('"stream": "joint_command"') == 1
        text = text.replace(
            '"stream": "joint_command"',
            '"stream": "joint_command", "stream": "pose_info"', 1)
        proc = self._valid_case(raw_text=text)
        payload = expect_fail('raw artifact duplicate key', proc)
        errors = collect_errors(payload)
        self.assertTrue(any('duplicate JSON object key' in e for e in errors),
                        errors)


# ---------------------------------------------------------------------------
# R15-C: frame_count runtime invariant (audit B5)
# ---------------------------------------------------------------------------

class TestR15CFrameCountInvariant(SmokeCase):

    def test_count_matrix(self):
        for frame_count, n_frames, should_pass in (
                (0, 0, True), (1, 1, True), (2, 2, True),
                (0, 1, False), (2, 1, False), (1, 0, False)):
            proc = self._valid_case(frame_count=frame_count,
                                    n_frames=n_frames)
            label = 'count=%d frames=%d' % (frame_count, n_frames)
            if should_pass:
                expect_pass(label, proc)
            else:
                payload = expect_fail(label, proc)
                self.assertTrue(any('frame_count' in e for e in
                                    collect_errors(payload)))

    def test_negative_count_fails(self):
        provenance = literal_provenance(frame_count=-1)
        proc = self._case('negative', manifest=literal_manifest(),
                          provenance=provenance,
                          frame=[literal_frame()], raw=literal_raw())
        payload = expect_fail('negative frame_count', proc)
        self.assertTrue(any('negative' in e for e in collect_errors(payload))
                        or any('frame_count' in e
                               for e in collect_errors(payload)))

    def test_bool_and_float_count_fail(self):
        for bad in (True, 2.0):
            provenance = literal_provenance(frame_count=1)
            provenance['frame_count'] = bad
            proc = self._case('bool/float', manifest=literal_manifest(),
                              provenance=provenance,
                              frame=[literal_frame()], raw=literal_raw())
            expect_fail('frame_count=%r' % (bad,), proc)

    def test_frames_null_fails(self):
        proc = self._valid_case(frame_text='null')
        payload = expect_fail('frames=null', proc)
        self.assertTrue(any('must be a JSON ARRAY' in e
                            for e in collect_errors(payload)))

    def test_frames_object_fails(self):
        proc = self._valid_case(frame_text=json.dumps(literal_frame()))
        payload = expect_fail('frames=object', proc)
        self.assertTrue(any('must be a JSON ARRAY' in e
                            for e in collect_errors(payload)))

    def test_frames_missing_file_fails(self):
        proc = self._valid_case(missing_frame=True)
        payload = expect_fail('frames missing', proc)
        self.assertTrue(any('not found' in e for e in collect_errors(payload)))

    def test_invalid_frame_with_equal_count_fails(self):
        # count equality alone must not pass an internally-invalid frame
        frame = literal_frame()
        frame['zmp_position'] = [0.1, 0.2, 0.3]
        proc = self._case('bad frame', manifest=literal_manifest(),
                          provenance=literal_provenance(frame_count=1),
                          frame=[frame], raw=literal_raw())
        payload = expect_fail('invalid frame, equal count', proc)
        self.assertTrue(any('zmp_position' in e
                            for e in collect_errors(payload)))

    def test_synced_count_rule_deletion_still_enforced(self):
        # § 6.3: delete the frame-count consistency rule from BOTH
        # documents (copies stay equal) — without an authorized version
        # bump + pin update the smoke CLI must reject the changed norm
        rfc, contract = sync_mutate('"count_must_equal": "frame_count",\n',
                                    '')
        proc = self._valid_case(rfc_text=rfc, contract_text=contract,
                                frame_count=2, n_frames=1)
        payload = expect_fail('synced count-rule deletion', proc)
        self.assertTrue(any('integrity pin mismatch' in e
                            for e in collect_errors(payload)))

    def test_synced_count_rule_optionalized_rejected(self):
        # § 6.3: make the rule "optional" in both copies — rejected
        rfc, contract = sync_mutate(
            '"count_must_equal": "frame_count",',
            '"count_must_equal": "optional",')
        proc = self._valid_case(rfc_text=rfc, contract_text=contract,
                                frame_count=2, n_frames=1)
        payload = expect_fail('synced count-rule optionalized', proc)
        self.assertTrue(any('integrity pin mismatch' in e
                            for e in collect_errors(payload)))

    def test_synced_metadata_change_rejected(self):
        # § 6.3: mutate frame_artifact metadata in both copies — the two
        # blocks remain equal; still rejected without authorized upgrade
        rfc, contract = sync_mutate(
            '"type": "array_of_frames",', '"type": "array_of_records",')
        proc = self._valid_case(rfc_text=rfc, contract_text=contract)
        payload = expect_fail('synced metadata change', proc)
        self.assertTrue(any('integrity pin mismatch' in e
                            for e in collect_errors(payload)))

    def test_authorized_zero_frame_episode_passes(self):
        proc = self._valid_case(frame_count=0, n_frames=0)
        expect_pass('empty episode under authorized schema', proc)


# ---------------------------------------------------------------------------
# Unit-level: strict JSON, fence scanning, schema shape validation
# ---------------------------------------------------------------------------

class TestR15UnitStrict(unittest.TestCase):

    def test_strict_loads_rejects_duplicate_keys_at_any_depth(self):
        for text in ('{"a": 1, "a": 2}',
                     '{"x": {"a": 1, "a": 2}}',
                     '{"schema_version": "s", "schema_version": "t"}',
                     '[{"a": 1, "a": 2}]'):
            with self.assertRaises(c._AmbiguousJsonError):
                c._strict_json_loads(text)

    def test_strict_loads_rejects_nonstandard_constants(self):
        for text in ('{"a": NaN}', '{"a": Infinity}', '{"a": -Infinity}'):
            with self.assertRaises(c._AmbiguousJsonError):
                c._strict_json_loads(text)

    def test_strict_loads_accepts_normal_json(self):
        obj = c._strict_json_loads('{"a": 1, "b": [true, null, 0.5]}')
        self.assertEqual(obj, {'a': 1, 'b': [True, None, 0.5]})

    def test_scan_fences_flags_unterminated(self):
        regions, errors = c._scan_fenced_regions('```json\n{"a": 1}')
        self.assertEqual(regions, [])
        self.assertEqual(len(errors), 1)
        self.assertIn('unterminated code fence', errors[0])

    def test_scan_fences_info_line_cannot_close(self):
        # a ```json line inside an open fence is CONTENT, not a closer
        text = '```json\n{"a": 1}\n```json\n```'
        regions, errors = c._scan_fenced_regions(text)
        self.assertEqual(errors, [])
        self.assertEqual(len(regions), 1)
        self.assertIn('```json', regions[0][1])

    def test_shape_validation_rejects_duplicate_frame_field_names(self):
        schema, _ = c.load_normative_schema(c.RFC_PATH)
        schema = copy.deepcopy(schema)
        fields = schema['frame_fields']['measured']['fields']
        schema['frame_fields']['measured']['fields'] = \
            list(fields) + [dict(fields[0])]
        errors = c._validate_schema_shape(schema)
        self.assertTrue(any('duplicate field names' in e for e in errors),
                        errors)

    def test_shape_validation_requires_count_rule_declaration(self):
        schema, _ = c.load_normative_schema(c.RFC_PATH)
        schema = copy.deepcopy(schema)
        schema['frame_artifact']['count_must_equal'] = 'optional'
        errors = c._validate_schema_shape(schema)
        self.assertTrue(any('count_must_equal' in e for e in errors), errors)


# ---------------------------------------------------------------------------
# R6-R10 + frozen-state regression
# ---------------------------------------------------------------------------

class TestR6toR10Regression(unittest.TestCase):

    def test_rfc_and_contract_files_unchanged_by_r15(self):
        # R15 changes no rule and no document byte: the R14-committed
        # hashes still pin both documents (RFC REV1.5 / Contract v1.6)
        import hashlib
        for path, expected in (
                (c.RFC_PATH,
                 'f8662d1673f1a30e9cfb2d8156214d7124eb45c08301087d85c4fae'
                 '1ca894efe'),
                (c.CONTRACT_PATH,
                 '0384aad2d1f4db92c7fc02729c71fa9daf4d3542a26df8e72d2682ec'
                 'fdab0295')):
            h = hashlib.sha256(open(path, 'rb').read()).hexdigest()
            self.assertEqual(h, expected, path)

    def test_pin_constants_match_both_documents(self):
        for path in (c.RFC_PATH, c.CONTRACT_PATH):
            with open(path, 'r', encoding='utf-8') as fh:
                text = fh.read()
            schema, block_text, err = c._find_normative_block(text)
            self.assertIsNone(err, (path, err))
            import hashlib
            sha = hashlib.sha256(block_text.encode('utf-8')).hexdigest()
            self.assertEqual(sha, c._AUTHORIZED_SCHEMA_SHA256, path)
            self.assertEqual(schema['schema_version'],
                             c._AUTHORIZED_SCHEMA_VERSION)

    def test_r6_no_cadence_band_in_probe(self):
        text = open(PROBE, 'r', encoding='utf-8').read()
        self.assertNotIn('CADENCE_MIN_HZ', text)
        self.assertNotIn('CADENCE_MAX_HZ', text)
        self.assertIn('contact_publication_events', text)

    def test_r7_two_state_vocabulary(self):
        text = open(c.RFC_PATH, 'r', encoding='utf-8').read()
        self.assertIn('UNKNOWN_NO_MESSAGE', text)
        self.assertIn('VALID_CONTACT', text)
        self.assertIn('no fabricated flight labels', text)

    def test_r8_composition_remains_unverified(self):
        schema, _ = c.load_normative_schema(c.RFC_PATH)
        self.assertEqual(schema['status_semantics']['composition_status'],
                         'UNVERIFIED')

    def test_r9_measured_derived_separation_unchanged(self):
        schema, _ = c.load_normative_schema(c.RFC_PATH)
        measured = schema['frame_fields']['measured']
        self.assertEqual(measured['status'], 'measured')
        self.assertIn('joint_position',
                      [f['name'] for f in measured['fields']])
        self.assertNotIn('left_foot_pose',
                         [f['name'] for f in measured['fields']])
        derived = schema['frame_fields']['ordinary_derived']
        self.assertEqual(derived['status'], 'derived')
        self.assertIn('base_velocity',
                      [f['name'] for f in derived['fields']])

    def test_r10_association_compatibility(self):
        text = open(PROBE, 'r', encoding='utf-8').read()
        self.assertIn('timestamp_association_compatibility', text)
        rfc = open(c.RFC_PATH, 'r', encoding='utf-8').read()
        self.assertIn('universal clock identity', rfc)

    def test_frozen_files_untouched(self):
        # scope guard: the frozen set and probe are byte-identical to the
        # R13 baseline commit 88aa3fa (same guard as the R14 suite)
        for rel in ('tools/run_phase_a_episode.py', 'tools/stage1_probe.py',
                    'src/nao/nao_description/urdf/nao_simple.sdf',
                    'src/nao_gz_sim/worlds/nao_world.sdf'):
            path = os.path.join(WS, rel)
            proc = subprocess.run(
                ['git', 'diff', '88aa3fa', '--', rel], cwd=WS,
                capture_output=True, text=True, timeout=30)
            self.assertEqual(proc.stdout.strip(), '',
                             'frozen file modified: %s' % rel)


if __name__ == '__main__':
    unittest.main(verbosity=2)
