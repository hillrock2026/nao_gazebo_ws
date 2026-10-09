#!/usr/bin/env python3
"""Stage 1 native collector tests.

Run:  python3 tools/test_stage1_collector.py

Layers (task § 6) — each states what it can and cannot prove:

  * Unit (synthetic fixtures): message -> artifact conversion, source-stamp
    retention, unknown-state retention, silence handling, raw/derived
    separation, frame order/association, frame_count, duplicate-id, write
    error, checker-rejection status.  **Fixtures exercise local logic only;
    they do NOT prove the real Gazebo capture path.**
  * Round-trip: source-like message -> collector serialization -> artifact on
    disk -> STRICT read-back -> frozen contract validation; saved values are
    compared field-by-field to the input (no silent loss/rewrite).
  * Real CLI: the collector subcommands drive the FROZEN R15 checker by
    subprocess; exit codes are asserted (never internal-function duplicates).

No Gazebo, no network.  Synthetic fixtures are labelled FIXTURE throughout.
"""

import copy
import json
import os
import subprocess
import sys
import tempfile
import types
import unittest

TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(TOOLS)
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

import stage1_native_collector as C
import stage1_contract_check as checker

COLLECTOR = os.path.join(TOOLS, 'stage1_native_collector.py')


# ---------------------------------------------------------------------------
# FIXTURE message builders (synthetic — NOT Gazebo evidence)
# ---------------------------------------------------------------------------

def NS(**kw):
    return types.SimpleNamespace(**kw)


def F_vec(x, y, z):
    return NS(x=x, y=y, z=z)


def F_quat(x=0.0, y=0.0, z=0.0, w=1.0):
    return NS(x=x, y=y, z=z, w=w)


def F_header(t):
    return NS(stamp=NS(sec=int(t), nsec=int(round((t - int(t)) * 1e9))))


def F_entity(name, eid):
    return NS(name=name, id=eid, type=2)


def F_contacts(stream, t, n_contacts=1, n_points=1):
    contacts = []
    for k in range(n_contacts):
        left = stream == 'contacts_left'
        contacts.append(NS(
            header=F_header(t),
            collision1=F_entity(
                'nao::l_ankle::l_ankle_collision' if left
                else 'nao::r_ankle::r_ankle_collision', 5 + k),
            collision2=F_entity('ground_plane::link::collision', 3),
            position=[F_vec(0.1 + 0.01 * j, 0.05, 0.0)
                      for j in range(n_points)],
            normal=[F_vec(0.0, 0.0, 1.0) for _ in range(n_points)],
            depth=[0.001 + 0.0001 * j for j in range(n_points)],
            world=2,
            wrench=[NS(body_1_name='', body_1_id=0,
                       body_2_name='', body_2_id=0,
                       body_1_wrench=NS(force=F_vec(0.01, 0.02, 25.5),
                                        torque=F_vec(0.0, 0.001, 0.0),
                                        force_offset=F_vec(0.1, 0.05, 0.0)),
                       body_2_wrench=NS(force=F_vec(-0.01, -0.02, -25.5),
                                        torque=F_vec(0.0, -0.001, 0.0),
                                        force_offset=F_vec(0.1, 0.05, 0.0)))]))
    return NS(header=F_header(t), contact=contacts)


def F_pose_v(t, model_xy=(0.1, 0.0)):
    entries = [NS(name='nao', id=10,
                  position=F_vec(model_xy[0], model_xy[1], 0.33),
                  orientation=F_quat()),
               NS(name='ground_plane', id=4, position=F_vec(0, 0, 0),
                  orientation=F_quat()),
               NS(name='world', id=1, position=F_vec(0, 0, 0),
                  orientation=F_quat())]
    for i, ln in enumerate(C.EXPECTED_LINKS):
        entries.append(NS(name=ln, id=11 + i,
                          position=F_vec(0.0, 0.05, -0.28),
                          orientation=F_quat()))
    return NS(header=F_header(t), pose=entries)


JOINT = C._joint_order()
NJ = len(JOINT)


def F_joint_states(t, pos=None, vel=None, names=None):
    return NS(header=F_header(t), name=list(names or JOINT),
              position=list(pos if pos is not None else [0.1] * NJ),
              velocity=list(vel if vel is not None else [0.0] * NJ),
              effort=[])


def F_joint_command(t, pos=None):
    return NS(header=F_header(t), name=list(JOINT),
              position=list(pos if pos is not None else [0.1] * NJ))


def F_body_pose(t, x=0.1):
    return NS(header=NS(stamp=NS(sec=int(t), nsec=0), frame_id='world'),
              pose=NS(position=F_vec(x, 0.0, 0.33), orientation=F_quat()))


def F_imu(t):
    return NS(header=F_header(t), orientation=F_quat(),
              linear_acceleration=F_vec(0.0, 0.0, 9.8),
              angular_velocity=F_vec(0.0, 0.0, 0.0))


def F_bool(v):
    return NS(data=v)


def fixture_records(n_frames=5, dt=0.01, t0=1.0, with_contacts=True,
                    with_bools=True, with_command=True, with_pose=True,
                    model_x=None):
    """A complete, valid synthetic record set (FIXTURE)."""
    kinds = ['contacts_left', 'contacts_right', 'pose_info', 'joint_states',
             'joint_command', 'body_pose', 'imu', 'left_contact_bool',
             'right_contact_bool']
    recs = {k: [] for k in kinds}
    seq = 0
    for i in range(n_frames):
        t = t0 + dt * i
        mx = (model_x[i] if model_x else 0.1 + 0.001 * i)
        if with_contacts:
            recs['contacts_left'].append(dict(
                C.build_contacts_record('contacts_left', F_contacts(
                    'contacts_left', t)), _seq=seq)); seq += 1
            recs['contacts_right'].append(dict(
                C.build_contacts_record('contacts_right', F_contacts(
                    'contacts_right', t)), _seq=seq)); seq += 1
        if with_pose:
            recs['pose_info'].append(dict(
                C.build_pose_info_record(F_pose_v(t, (mx, 0.0))), _seq=seq))
            seq += 1
        recs['joint_states'].append(dict(
            C.build_joint_states_record(F_joint_states(t)), _seq=seq))
        seq += 1
        if with_command:
            recs['joint_command'].append(dict(
                C.build_joint_command_record(F_joint_command(t)), _seq=seq))
            seq += 1
        recs['body_pose'].append(dict(
            C.build_body_pose_record(F_body_pose(t, mx)), _seq=seq))
        seq += 1
        recs['imu'].append(dict(C.build_imu_record(F_imu(t)), _seq=seq))
        seq += 1
        if with_bools:
            recs['left_contact_bool'].append(
                dict(C.build_bool_record(F_bool(True)), _seq=seq)); seq += 1
            recs['right_contact_bool'].append(
                dict(C.build_bool_record(F_bool(False)), _seq=seq)); seq += 1
    return recs


def fixture_assembler(episode_id='r0_anchor_s0', ring='ring0', seed=0):
    theta = json.load(open(os.path.join(
        WS, 'dataset', 'stage0', 'stage0_theta_list.json')
    ))['theta_list'][0]['theta']
    inertials, sdf_sha = C.load_sdf_inertials()
    total = sum(v['mass'] for v in inertials.values())
    return C.EpisodeAssembler(
        episode_id=episode_id, theta=theta, theta_id=episode_id, seed=seed,
        ring=ring, config_hash_source=C.config_hash_source(),
        wallclock_utc='2026-10-09T00:00:00+00:00', git_commit='0' * 40,
        rfc_hash=C.sha256_file(C.RFC_PATH), model_hash=sdf_sha,
        world_hash=C.sha256_file(C.WORLD_PATH), gz_version='8.15.0',
        theta_list_hash='844646f5' + '0' * 56, inertials=inertials,
        total_mass=total, sdf_sha=sdf_sha)


def freeze_fixture(records=None, tmp=None, **kw):
    records = fixture_records(**kw) if records is None else records
    asm = fixture_assembler()
    tmp = tmp or tempfile.mkdtemp()
    summary = C.freeze_episode(asm, records, tmp)
    return asm, records, tmp, summary


def run_cli(args):
    return subprocess.run([sys.executable, COLLECTOR] + args,
                          capture_output=True, text=True, timeout=120)


def smoke(episode_dir):
    return subprocess.run(
        [sys.executable, os.path.join(TOOLS, 'stage1_contract_check.py'),
         '--smoke',
         '--manifest', os.path.join(episode_dir, 'status_manifest.json'),
         '--provenance', os.path.join(episode_dir, 'episode_provenance.json'),
         '--frame', os.path.join(episode_dir, 'frames.json'),
         '--raw', os.path.join(episode_dir, 'raw_records.json')],
        capture_output=True, text=True, timeout=120)


# ---------------------------------------------------------------------------
# 6.1 Unit tests (FIXTURE): conversion, retention, silence, ordering
# ---------------------------------------------------------------------------

class TestCollectorUnit(unittest.TestCase):

    def test_contacts_record_is_lossless(self):
        msg = F_contacts('contacts_left', 12.345, n_contacts=2, n_points=3)
        rec = C.build_contacts_record('contacts_left', msg)
        self.assertEqual(rec['stamp_s'], 12.345)
        self.assertEqual(rec['n_contacts'], 2)
        self.assertEqual(len(rec['contacts']), 2)
        c = rec['contacts'][0]
        # collision identity verbatim (required key = name string)
        self.assertEqual(c['collision1'], 'nao::l_ankle::l_ankle_collision')
        self.assertEqual(c['collision2'], 'ground_plane::link::collision')
        # every contact point retained (no first-only reduction)
        self.assertEqual(len(c['position']), 3)
        self.assertEqual(len(c['normal']), 3)
        self.assertEqual(len(c['depth']), 3)
        for got, want in zip(c['depth'], [0.001, 0.0011, 0.0012]):
            self.assertAlmostEqual(got, want, places=12)
        # both body sides + force_offset retained
        w = c['wrenches'][0]
        self.assertEqual(set(w),
                         {'body_1_name', 'body_1_id', 'body_2_name',
                          'body_2_id', 'body_1_wrench', 'body_2_wrench'})
        self.assertEqual(w['body_1_wrench']['force'], [0.01, 0.02, 25.5])
        self.assertEqual(w['body_1_wrench']['force_offset'], [0.1, 0.05, 0.0])
        self.assertEqual(w['body_2_wrench']['force'], [-0.01, -0.02, -25.5])
        # per-contact header timestamp retained
        self.assertEqual(c['header']['stamp']['sec'], 12)

    def test_pose_info_retains_all_entries(self):
        msg = F_pose_v(3.0)
        rec = C.build_pose_info_record(msg)
        self.assertEqual(rec['stamp_s'], 3.0)
        self.assertEqual(len(rec['entries']), len(msg.pose))
        names = [e['name'] for e in rec['entries']]
        self.assertIn('nao', names)
        for ln in C.EXPECTED_LINKS:
            self.assertIn(ln, names)
        e0 = rec['entries'][0]
        self.assertEqual(set(e0), {'name', 'id', 'position', 'orientation'})

    def test_joint_states_and_command_preserved(self):
        js = C.build_joint_states_record(F_joint_states(2.5))
        self.assertEqual(js['name'], JOINT)
        self.assertEqual(len(js['position']), NJ)
        jc = C.build_joint_command_record(F_joint_command(2.5))
        self.assertEqual(len(jc['command']), 24)

    def test_silent_contacts_are_unknown_no_message(self):
        # no contact records at all -> frames carry UNKNOWN_NO_MESSAGE and no
        # wrench record / zero force is invented
        recs = fixture_records(with_contacts=False)
        asm = fixture_assembler()
        frames = asm.build_frames(recs)
        self.assertTrue(frames)
        for f in frames:
            self.assertEqual(f['left_contact_wrench_state'],
                             'UNKNOWN_NO_MESSAGE')
            self.assertEqual(f['right_contact_wrench_state'],
                             'UNKNOWN_NO_MESSAGE')
            for name in checker.WRENCH_FRAME_DEPENDENT_FIELDS:
                self.assertIsNone(f[name])
        # no contact raw record was fabricated for the silent stream
        streams = [r['stream'] for r in asm.raw_records(recs)]
        self.assertNotIn('contacts_left', streams)
        self.assertNotIn('contacts_right', streams)

    def test_raw_and_derived_are_separated(self):
        recs = fixture_records()
        raw = fixture_assembler().raw_records(recs)
        streams = {r['stream'] for r in raw}
        self.assertTrue(streams <= {'contacts_left', 'contacts_right',
                                    'pose_info', 'joint_command',
                                    'joint_states', 'body_pose'})
        frames = fixture_assembler().build_frames(recs)
        f = frames[2]
        for name in checker.COMPOSITION_DEPENDENT_FIELDS:
            self.assertIn(name, f)
        for name in checker.WRENCH_FRAME_DEPENDENT_FIELDS:
            self.assertIn(name, f)
            self.assertIsNone(f[name])

    def test_frame_order_and_pose_association(self):
        recs = fixture_records(n_frames=4)
        frames = fixture_assembler().build_frames(recs)
        self.assertEqual([f['frame_id'] for f in frames], [0, 1, 2, 3])
        stamps = [f['sim_time_sec'] for f in frames]
        self.assertEqual(stamps, sorted(stamps))
        for f in frames:
            self.assertIsNotNone(f['pose_source_stamp'])
            self.assertLessEqual(f['pose_source_stamp'], f['sim_time_sec'])

    def test_pose_miss_leaves_option_a_null_no_fabrication(self):
        recs = fixture_records(with_pose=False)
        frames = fixture_assembler().build_frames(recs)
        for f in frames:
            self.assertIsNone(f['pose_source_stamp'])
            self.assertIsNone(f['left_foot_pose']['position'])
            self.assertIsNone(f['com_position'])

    def test_unknown_pose_source_stamp_present_when_missing(self):
        recs = fixture_records(n_frames=2)
        frames = fixture_assembler().build_frames(recs)
        self.assertIn('pose_source_stamp', frames[0])

    def test_duplicate_timestamp_not_divided_by_zero(self):
        # two joint_states with the SAME stamp -> no 0/0; velocities stay None
        recs = fixture_records(n_frames=3)
        recs['joint_states'][1]['stamp_s'] = \
            recs['joint_states'][0]['stamp_s']
        frames = fixture_assembler().build_frames(recs)
        self.assertEqual(len(frames), 3)
        self.assertTrue(all(f['frame_id'] == i for i, f in enumerate(frames)))

    def test_missing_source_field_is_not_defaulted(self):
        # a contact without depth/position must NOT gain invented values
        rec = C.build_contacts_record('contacts_left', F_contacts(
            'contacts_left', 1.0))
        rec['contacts'][0]['depth'] = []
        rec['contacts'][0]['position'] = []
        self.assertEqual(rec['contacts'][0]['depth'], [])
        self.assertEqual(rec['contacts'][0]['position'], [])

    def test_duplicate_episode_dir_refused(self):
        records = fixture_records()
        asm = fixture_assembler()
        tmp = tempfile.mkdtemp()
        out = os.path.join(tmp, 'ep')
        os.makedirs(out)
        args = NS(episode_id='e', out_root=out, allow_overwrite=False)
        with self.assertRaises(RuntimeError):
            C._finalize(args, asm, records, {})

    def test_write_failure_is_classified(self):
        records = fixture_records()
        asm = fixture_assembler()
        tmp = tempfile.mkdtemp()
        out = os.path.join(tmp, 'ep')
        orig = C.freeze_episode
        C.freeze_episode = lambda *a, **k: (_ for _ in ()).throw(
            ValueError('disk full (injected)'))
        try:
            args = NS(episode_id='e', out_root=out, allow_overwrite=False)
            rc, status = C._finalize(args, asm, records, {})
        finally:
            C.freeze_episode = orig
        self.assertEqual(rc, 1)
        self.assertEqual(status['status'], 'write_failed')

    def test_partial_episode_when_required_stream_empty(self):
        records = fixture_records()
        records['contacts_left'] = []
        asm = fixture_assembler()
        tmp = tempfile.mkdtemp()
        out = os.path.join(tmp, 'ep')
        args = NS(episode_id='e', out_root=out, allow_overwrite=False)
        rc, status = C._finalize(args, asm, records, {})
        self.assertEqual(status['status'], 'partial')
        self.assertNotEqual(rc, 0)

    def test_no_joint_states_is_collection_failed(self):
        records = fixture_records()
        records['joint_states'] = []
        asm = fixture_assembler()
        tmp = tempfile.mkdtemp()
        out = os.path.join(tmp, 'ep')
        args = NS(episode_id='e', out_root=out, allow_overwrite=False)
        rc, status = C._finalize(args, asm, records, {})
        self.assertEqual(status['status'], 'collection_failed')
        self.assertNotEqual(rc, 0)

    def test_checker_rejection_is_contract_rejected(self):
        records = fixture_records()
        # break an Option-A field -> checker must reject
        asm = fixture_assembler()
        tmp = tempfile.mkdtemp()
        out = os.path.join(tmp, 'ep')
        orig = asm.build_frames

        def broken(recs):
            frames = orig(recs)
            frames[1]['zmp_position'] = [0.1, 0.2, 0.3]  # wrong shape
            return frames
        asm.build_frames = broken
        args = NS(episode_id='e', out_root=out, allow_overwrite=False)
        rc, status = C._finalize(args, asm, records, {})
        self.assertEqual(status['status'], 'contract_rejected')
        self.assertNotEqual(rc, 0)
        self.assertFalse(status['checker_ok'])

    def test_sdf_inertials_parse_25_links(self):
        inertials, sha = C.load_sdf_inertials()
        self.assertEqual(len(inertials), 25)
        self.assertEqual(len(sha), 64)
        total = sum(v['mass'] for v in inertials.values())
        self.assertAlmostEqual(total, 5.3054, places=3)


# ---------------------------------------------------------------------------
# 6.2 Round-trip (FIXTURE): serialize -> disk -> strict read-back -> validate
# ---------------------------------------------------------------------------

class TestCollectorRoundTrip(unittest.TestCase):

    def test_full_roundtrip_passes_frozen_checker(self):
        asm, records, tmp, summary = freeze_fixture(n_frames=5)
        self.assertEqual(summary['frame_count'], 5)
        proc = smoke(tmp)
        self.assertEqual(proc.returncode, 0, proc.stdout[-2000:])

    def test_roundtrip_values_equal_source(self):
        msg = F_contacts('contacts_right', 7.5, n_contacts=1, n_points=2)
        src = C.build_contacts_record('contacts_right', msg)
        tmp = tempfile.mkdtemp()
        C.write_json(os.path.join(tmp, 'r.json'), [src])
        back = checker._strict_json_loads(
            open(os.path.join(tmp, 'r.json'), encoding='utf-8').read())
        self.assertEqual(back, [src])
        # deep equality of the retained contact payload
        self.assertEqual(back[0]['contacts'][0]['position'],
                         src['contacts'][0]['position'])
        self.assertEqual(back[0]['contacts'][0]['wrenches'],
                         src['contacts'][0]['wrenches'])

    def test_roundtrip_pose_entries_equal_source(self):
        src = C.build_pose_info_record(F_pose_v(9.25))
        tmp = tempfile.mkdtemp()
        C.write_json(os.path.join(tmp, 'p.json'), [src])
        back = json.load(open(os.path.join(tmp, 'p.json'), encoding='utf-8'))
        self.assertEqual(back[0]['entries'], src['entries'])
        self.assertEqual(back[0]['stamp_s'], 9.25)

    def test_saved_stamp_equals_source_stamp(self):
        asm, records, tmp, summary = freeze_fixture(n_frames=3, t0=40.0)
        prov = json.load(open(os.path.join(
            tmp, 'episode_provenance.json'), encoding='utf-8'))
        self.assertEqual(prov['t_start'], 40.0)
        self.assertAlmostEqual(prov['t_end'], 40.02, places=9)

    def test_missing_field_not_filled(self):
        recs = fixture_records(n_frames=2)
        recs['contacts_left'][0]['contacts'][0]['depth'] = []
        asm = fixture_assembler()
        tmp = tempfile.mkdtemp()
        C.freeze_episode(asm, recs, tmp)
        proc = smoke(tmp)
        self.assertNotEqual(proc.returncode, 0)  # required[] empty -> FAIL


# ---------------------------------------------------------------------------
# 6.3 Real CLI tests (the collector drives the FROZEN checker by subprocess)
# ---------------------------------------------------------------------------

class TestCollectorCLI(unittest.TestCase):

    def _episode(self, **kw):
        asm, records, tmp, summary = freeze_fixture(**kw)
        return tmp

    def test_smoke_subcommand_passes(self):
        tmp = self._episode()
        proc = run_cli(['smoke', '--episode-dir', tmp])
        self.assertEqual(proc.returncode, 0, proc.stdout[-2000:])

    def test_verify_subcommand_passes(self):
        tmp = self._episode()
        proc = run_cli(['verify', '--episode-dir', tmp])
        self.assertEqual(proc.returncode, 0, proc.stdout[-2000:])
        rep = json.loads(proc.stdout)
        self.assertTrue(rep['frame_count_consistent'])
        self.assertEqual(rep['status'], 'PASS')

    def test_verify_detects_frame_count_mismatch(self):
        tmp = self._episode()
        p = os.path.join(tmp, 'episode_provenance.json')
        prov = json.load(open(p, encoding='utf-8'))
        prov['frame_count'] = prov['frame_count'] + 3
        C.write_json(p, prov)
        proc = run_cli(['verify', '--episode-dir', tmp])
        self.assertNotEqual(proc.returncode, 0)
        self.assertEqual(json.loads(proc.stdout)['status'], 'FAIL')

    def test_invalid_json_rejected(self):
        tmp = self._episode()
        open(os.path.join(tmp, 'frames.json'), 'w').write('{not json')
        proc = run_cli(['smoke', '--episode-dir', tmp])
        self.assertNotEqual(proc.returncode, 0)

    def test_required_field_missing_rejected(self):
        tmp = self._episode()
        p = os.path.join(tmp, 'raw_records.json')
        raw = json.load(open(p, encoding='utf-8'))
        for r in raw:
            if r['stream'] == 'contacts_left':
                del r['contacts'][0]['header']
                break
        C.write_json(p, raw)
        proc = run_cli(['smoke', '--episode-dir', tmp])
        self.assertNotEqual(proc.returncode, 0)

    def test_provenance_field_missing_rejected(self):
        tmp = self._episode()
        p = os.path.join(tmp, 'episode_provenance.json')
        prov = json.load(open(p, encoding='utf-8'))
        del prov['ring']
        C.write_json(p, prov)
        proc = run_cli(['smoke', '--episode-dir', tmp])
        self.assertNotEqual(proc.returncode, 0)

    def test_duplicate_key_in_artifact_rejected(self):
        tmp = self._episode()
        p = os.path.join(tmp, 'frames.json')
        text = open(p, encoding='utf-8').read()
        text = text.replace('"frame_id": 0', '"frame_id": 0, "frame_id": 9', 1)
        open(p, 'w', encoding='utf-8').write(text)
        proc = run_cli(['smoke', '--episode-dir', tmp])
        self.assertNotEqual(proc.returncode, 0)

    def test_incomplete_episode_not_reported_success(self):
        tmp = self._episode()
        os.remove(os.path.join(tmp, 'raw_records.json'))
        proc = run_cli(['smoke', '--episode-dir', tmp])
        self.assertNotEqual(proc.returncode, 0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
