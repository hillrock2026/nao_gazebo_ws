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
import itertools
import json
import math
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


def F_clock(t):
    return NS(clock=NS(sec=int(t), nanosec=int(round((t - int(t)) * 1e9))))


def fixture_records(n_frames=5, dt=0.01, t0=1.0, with_contacts=True,
                    with_bools=True, with_command=True, with_pose=True,
                    model_x=None):
    """A complete, valid synthetic record set (FIXTURE)."""
    kinds = ['contacts_left', 'contacts_right', 'pose_info', 'joint_states',
             'joint_command', 'body_pose', 'imu', 'left_contact_bool',
             'right_contact_bool', 'clock']
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
        recs['clock'].append(dict(C.build_clock_record(F_clock(t)), _seq=seq))
        seq += 1
        if with_bools:
            # Bool receipt sim time attached at callback time (R-05); fixture
            # uses the frame sim time => fresh, matched, current clock valid
            recs['left_contact_bool'].append(dict(
                C.build_bool_record(F_bool(True)), _recv_sim_s=t,
                _recv_epoch=0, _recv_clock_state='valid', _seq=seq)); seq += 1
            recs['right_contact_bool'].append(dict(
                C.build_bool_record(F_bool(False)), _recv_sim_s=t,
                _recv_epoch=0, _recv_clock_state='valid', _seq=seq)); seq += 1
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

    def test_missing_required_stream_is_collection_failed(self):
        records = fixture_records()
        records['contacts_left'] = []
        asm = fixture_assembler()
        tmp = tempfile.mkdtemp()
        out = os.path.join(tmp, 'ep')
        args = NS(episode_id='e', out_root=out, allow_overwrite=False)
        rc, status = C._finalize(args, asm, records, {})
        self.assertEqual(status['terminal_status'], 'collection_failed')
        self.assertEqual(status['acquisition_completeness'], 'missing_required')
        self.assertNotEqual(rc, 0)

    def test_no_joint_states_is_collection_failed(self):
        records = fixture_records()
        records['joint_states'] = []
        asm = fixture_assembler()
        tmp = tempfile.mkdtemp()
        out = os.path.join(tmp, 'ep')
        args = NS(episode_id='e', out_root=out, allow_overwrite=False)
        rc, status = C._finalize(args, asm, records, {})
        self.assertEqual(status['terminal_status'], 'collection_failed')
        self.assertNotEqual(rc, 0)

    def test_callback_error_prevents_complete_validated(self):
        # a non-empty callback_errors must not yield complete_validated (F1/R-01)
        records = fixture_records()
        asm = fixture_assembler()
        tmp = tempfile.mkdtemp()
        out = os.path.join(tmp, 'ep')
        args = NS(episode_id='e', out_root=out, allow_overwrite=False)
        rc, status = C._finalize(args, asm, records,
                                 {'callback_errors': [{'kind': 'imu'}]})
        self.assertNotEqual(status['terminal_status'], 'complete_validated')
        self.assertNotEqual(rc, 0)

    def test_subscribe_not_ok_prevents_complete_validated(self):
        records = fixture_records()
        asm = fixture_assembler()
        tmp = tempfile.mkdtemp()
        out = os.path.join(tmp, 'ep')
        args = NS(episode_id='e', out_root=out, allow_overwrite=False)
        streams = {'joint_command': {'subscribe_ok': False}}
        rc, status = C._finalize(args, asm, records, {'streams': streams})
        self.assertNotEqual(status['terminal_status'], 'complete_validated')
        self.assertNotEqual(rc, 0)

    def test_checker_rejection_records_contract_failed(self):
        # the contract dimension must record 'failed' and the run must not be
        # complete_validated (integrity is gated => terminal integrity_failed
        # by precedence; the contract result is still recorded)
        records = fixture_records()
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
        self.assertEqual(status['contract_validation'], 'failed')
        self.assertNotEqual(status['terminal_status'], 'complete_validated')
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
# 6.2b L0 -> L1: REAL native gz protobuf message -> builder (not a fixture)
#
# Exercises the collector's contact conversion boundary with the ACTUAL native
# message type the callback receives (gz.msgs.Contacts over gz-transport), not
# the SimpleNamespace fixtures. This validates the L0 (native message) -> L1
# (builder record) leg only; it does NOT exercise the live gz-transport
# subscription/discovery or the full end-to-end capture path.
# ---------------------------------------------------------------------------

_NATIVE_CONTACTS_PB2 = None
try:
    _NATIVE_CONTACTS_PB2 = C.probe.gz_packages()[0]
except Exception:  # noqa: BLE001 - environment without gz protobuf bindings
    _NATIVE_CONTACTS_PB2 = None


def native_contacts_message(t=12.25, n_points=2):
    """A REAL gz.msgs.Contacts protobuf (built through the native bindings)."""
    pb = _NATIVE_CONTACTS_PB2
    sec, nsec = int(t), int(round((t - int(t)) * 1e9))
    m = pb.Contacts()
    m.header.stamp.sec = sec
    m.header.stamp.nsec = nsec
    c = m.contact.add()
    c.header.stamp.sec = sec
    c.header.stamp.nsec = nsec
    c.collision1.name = 'nao::l_ankle::l_ankle_collision'
    c.collision1.id = 5
    c.collision1.type = 2
    c.collision2.name = 'ground_plane::link::collision'
    c.collision2.id = 3
    c.collision2.type = 2
    for j in range(n_points):
        p = c.position.add(); p.x = 0.1 + 0.01 * j; p.y = 0.05; p.z = 0.0
        n = c.normal.add(); n.x = 0.0; n.y = 0.0; n.z = 1.0
        c.depth.append(0.001 + 0.0001 * j)
    w = c.wrench.add()
    w.body_1_name = 'nao::l_ankle'; w.body_1_id = 7
    w.body_2_name = 'ground_plane'; w.body_2_id = 3
    for side, sgn in (('body_1_wrench', 1.0), ('body_2_wrench', -1.0)):
        wr = getattr(w, side)
        wr.force.x = 0.01 * sgn; wr.force.y = 0.02 * sgn; wr.force.z = 25.5 * sgn
        wr.torque.x = 0.0; wr.torque.y = 0.001 * sgn; wr.torque.z = 0.0
        wr.force_offset.x = 0.1; wr.force_offset.y = 0.05
        wr.force_offset.z = 0.0
    c.world.id = 2; c.world.name = 'default'; c.world.type = 2
    return m


@unittest.skipUnless(_NATIVE_CONTACTS_PB2 is not None,
                     'native gz.msgs.Contacts protobuf bindings unavailable')
class TestNativeL0L1(unittest.TestCase):

    def test_type_identity_and_source_stamp(self):
        rec = C.build_contacts_record('contacts_left',
                                      native_contacts_message(12.25))
        self.assertEqual(rec['stream'], 'contacts_left')
        self.assertEqual(rec['stamp_s'], 12.25)
        self.assertEqual(rec['header']['stamp'], {'sec': 12, 'nsec': 250000000})
        self.assertEqual(rec['n_contacts'], 1)

    def test_contact_fields_verbatim(self):
        msg = native_contacts_message()
        rec = C.build_contacts_record('contacts_left', msg)
        c0, c = rec['contacts'][0], msg.contact[0]
        self.assertEqual(c0['collision1'], c.collision1.name)
        self.assertEqual(c0['collision2'], c.collision2.name)
        self.assertEqual(c0['collision1_id'], c.collision1.id)
        self.assertEqual(c0['collision2_id'], c.collision2.id)
        self.assertEqual(c0['position'], [[p.x, p.y, p.z] for p in c.position])
        self.assertEqual(c0['normal'], [[p.x, p.y, p.z] for p in c.normal])
        self.assertEqual(c0['depth'], list(c.depth))
        self.assertEqual(c0['header']['stamp'], {'sec': 12, 'nsec': 250000000})
        self.assertEqual(c0['world'], {'id': c.world.id, 'name': c.world.name,
                                       'type': str(c.world.type)})

    def test_wrench_and_force_offset_verbatim(self):
        msg = native_contacts_message()
        rec = C.build_contacts_record('contacts_left', msg)
        w0, w = rec['contacts'][0]['wrenches'][0], msg.contact[0].wrench[0]
        self.assertEqual(w0['body_1_name'], w.body_1_name)
        self.assertEqual(w0['body_1_id'], w.body_1_id)
        self.assertEqual(w0['body_2_name'], w.body_2_name)
        self.assertEqual(w0['body_2_id'], w.body_2_id)
        for key in ('body_1_wrench', 'body_2_wrench'):
            src = getattr(w, key)
            self.assertEqual(w0[key]['force'],
                             [src.force.x, src.force.y, src.force.z])
            self.assertEqual(w0[key]['torque'],
                             [src.torque.x, src.torque.y, src.torque.z])
            self.assertEqual(w0[key]['force_offset'],
                             [src.force_offset.x, src.force_offset.y,
                              src.force_offset.z])
        self.assertEqual(w0['body_1_wrench']['force_offset'], [0.1, 0.05, 0.0])

    def test_absent_repeated_fields_not_fabricated(self):
        pb = _NATIVE_CONTACTS_PB2
        m = pb.Contacts()
        m.header.stamp.sec = 1
        c = m.contact.add()
        c.collision1.name = 'nao::l_ankle::l_ankle_collision'
        c.collision1.id = 5; c.collision1.type = 2
        c0 = C.build_contacts_record('contacts_left', m)['contacts'][0]
        self.assertEqual(c0['wrenches'], [])
        self.assertEqual(c0['position'], [])
        self.assertEqual(c0['normal'], [])
        self.assertEqual(c0['depth'], [])

    def test_raw_only_and_l1_to_l2_roundtrip(self):
        rec = C.build_contacts_record('contacts_left', native_contacts_message())
        # raw/derived separation: only raw fields are emitted for a contact
        self.assertEqual(set(rec.keys()),
                         {'stream', 'stamp_s', 'header', 'n_contacts',
                          'contacts'})
        tmp = tempfile.mkdtemp()
        C.write_json(os.path.join(tmp, 'r.json'), [rec])
        back = checker._strict_json_loads(
            open(os.path.join(tmp, 'r.json'), encoding='utf-8').read())
        self.assertEqual(back, [rec])


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


# ---------------------------------------------------------------------------
# 6.4 v0.3 repair rules (FIXTURE unit tests; NOT RUN under this task)
# ---------------------------------------------------------------------------

class TestV03RepairRules(unittest.TestCase):
    """Unit tests for the v0.3 determinate rules (R-01/R-02/R-03/R-05/R-06/
    R-09/R-10/R-13/R-14) and the gated-input fail-closed behavior.

    Protected roots are replaced by a temporary directory in setUp so these
    tests NEVER touch the real historical smoke dirs."""

    def setUp(self):
        self._orig_roots = C.PROTECTED_ROOTS
        self._prot_tmp = tempfile.mkdtemp()
        self._prot = os.path.join(self._prot_tmp, 'hist_root')
        os.makedirs(self._prot)
        C.PROTECTED_ROOTS = (self._prot,)

    def tearDown(self):
        C.PROTECTED_ROOTS = self._orig_roots

    # --- R-13 Header ---------------------------------------------------
    def test_header_retains_stamp_and_data(self):
        h = NS(stamp=NS(sec=7, nsec=123456789),
               data=[NS(key='k1', value='v1'), NS(key='k2', value='v2')])
        out = C._header_verbatim(h)
        self.assertEqual(out['stamp'], {'sec': 7, 'nsec': 123456789})
        self.assertEqual(out['data'], [{'key': 'k1', 'value': 'v1'},
                                       {'key': 'k2', 'value': 'v2'}])

    def test_header_empty_data_and_absent_stamp(self):
        self.assertEqual(C._header_verbatim(NS(stamp=NS(sec=1, nsec=0),
                                               data=[]))['data'], [])
        out = C._header_verbatim(NS(data=[NS(key='a', value='b')]))
        self.assertNotIn('stamp', out)
        self.assertEqual(out['data'], [{'key': 'a', 'value': 'b'}])
        self.assertIsInstance(out, dict)

    def test_contact_header_data_survives_builder(self):
        msg = F_contacts('contacts_left', 3.5)
        msg.contact[0].header = NS(stamp=NS(sec=3, nsec=500000000),
                                   data=[NS(key='x', value='y')])
        h = C.build_contacts_record('contacts_left', msg)['contacts'][0][
            'header']
        self.assertEqual(h['stamp'], {'sec': 3, 'nsec': 500000000})
        self.assertEqual(h['data'], [{'key': 'x', 'value': 'y'}])

    # --- R-05 Bool association ----------------------------------------
    def _bools(self, entries):
        return [dict(value=v, _recv_sim_s=t, _recv_epoch=e, _seq=i)
                for i, (t, v, e) in enumerate(entries)]

    def test_bool_fresh_and_boundary(self):
        v, _t, st, why = C.assoc_contact_bool(
            self._bools([(1.0, True, 0)]), 1.02, 0)
        self.assertTrue(v); self.assertEqual(st, 'matched'); self.assertIsNone(why)
        v2, _t2, st2, _w2 = C.assoc_contact_bool(
            self._bools([(1.0, False, 0)]), 1.05, 0)   # age == 0.05 => fresh
        self.assertFalse(v2); self.assertEqual(st2, 'matched')

    def test_bool_stale(self):
        v, _t, st, why = C.assoc_contact_bool(
            self._bools([(1.0, True, 0)]), 1.06, 0)
        self.assertIsNone(v); self.assertEqual(st, 'unknown'); self.assertEqual(why, 'stale')

    def test_bool_future(self):
        v, _t, _st, why = C.assoc_contact_bool(
            self._bools([(2.0, True, 0)]), 1.0, 0)
        self.assertEqual(why, 'future'); self.assertIsNone(v)

    def test_bool_clock_missing_rewind_conflict_ambiguous(self):
        self.assertEqual(C.assoc_contact_bool(
            [dict(value=True, _recv_sim_s=None, _seq=0)], 1.0, 0)[3],
            'clock_missing')
        self.assertEqual(C.assoc_contact_bool(
            self._bools([(1.0, True, 0)]), 1.0, 1)[3], 'clock_rewind')
        self.assertEqual(C.assoc_contact_bool(
            self._bools([(1.0, True, 0), (1.0, False, 0)]), 1.0, 0)[3],
            'duplicate_conflict')
        self.assertEqual(C.assoc_contact_bool(
            [dict(value=True, _recv_sim_s=1.0, _recv_epoch=0, _seq=0),
             dict(value=True, _recv_sim_s=None, _recv_epoch=0, _seq=1)],
            1.0, 0)[3], 'order_ambiguous')
        self.assertEqual(C.assoc_contact_bool([], 1.0, 0)[3], 'no_message')

    def test_bool_identical_duplicate_matched(self):
        v, _t, st, _why = C.assoc_contact_bool(
            self._bools([(1.0, True, 0), (1.0, True, 0)]), 1.0, 0)
        self.assertTrue(v); self.assertEqual(st, 'matched')

    # --- R-06 IMU ------------------------------------------------------
    def test_imu_symmetric_nearest_future_and_tie(self):
        self.assertEqual(C.symmetric_nearest_index([1.005], 1.000, 0.01), 0)
        self.assertEqual(C.symmetric_nearest_index([0.99, 1.01], 1.0, 0.01), 0)
        self.assertIsNone(C.symmetric_nearest_index([1.02], 1.0, 0.01))

    def test_imu_missing_and_invalid_stamp(self):
        self.assertEqual(C.assoc_imu([{'stamp_s': None}], [], [], [],
                                     1.0, 0, 2.0)[2], 'source_stamp_missing')
        self.assertEqual(C.assoc_imu([{'stamp_s': float('inf')}],
                                     [{'stamp_s': float('inf')}],
                                     [float('inf')], [0], 1.0, 0, 2.0)[2],
                         'source_stamp_invalid')

    def test_imu_immature_and_matched(self):
        imu = [{'stamp_s': 1.0}]
        j, st, why, _d = C.assoc_imu(imu, imu, [1.0], [0], 1.0, 0, 1.0)
        self.assertIsNone(j); self.assertEqual(st, 'pending')
        self.assertEqual(why, 'immature_window')
        imu2 = [{'stamp_s': 1.0}, {'stamp_s': 1.005}]
        j2, st2, why2, d2 = C.assoc_imu(imu2, imu2, [1.0, 1.005], [0, 0],
                                        1.0, 0, 2.0)
        self.assertEqual(j2, 0); self.assertEqual(st2, 'matched')
        self.assertEqual(d2, 0.0)

    # --- classification ------------------------------------------------
    def test_classify_contract(self):
        self.assertEqual(C.classify_contract(0, None), ('error', 'checker_error'))
        self.assertEqual(C.classify_contract(0, {'smoke': {'ok': True}}),
                         ('passed', None))
        self.assertEqual(C.classify_contract(0, {'smoke': {'ok': False}}),
                         ('error', 'checker_error'))
        self.assertEqual(C.classify_contract(1, {'smoke': {'ok': False}}),
                         ('failed', 'checker_failed'))

    def test_classify_terminal_precedence(self):
        self.assertEqual(C.classify_terminal('missing_required', 'failed',
                                             'not_run', [], []),
                         'collection_failed')
        self.assertEqual(C.classify_terminal('complete', 'failed', 'passed',
                                             [], []), 'contract_rejected')
        self.assertEqual(C.classify_terminal('complete', 'passed', 'passed',
                                             [], []), 'complete_validated')
        self.assertEqual(C.classify_terminal('partial', 'passed', 'passed',
                                             [{'k': 1}], []), 'partial')
        self.assertEqual(C.classify_terminal('complete', 'passed', 'passed',
                                             [], [{'e': 1}]),
                         'collection_failed')
        self.assertEqual(C.classify_terminal('complete', 'passed', 'not_run',
                                             [], []), 'integrity_failed')
        self.assertEqual(C.classify_terminal('complete', 'passed', 'passed',
                                             [], [], aborted=True), 'aborted')

    def test_verify_integrity_missing_summary_and_eligibility(self):
        # F-04: the required set is fixed, so a missing summary FAILS (no longer
        # a 'not_run' gate) — F-04 must never yield a passing integrity state.
        st, why, _d = C.verify_integrity('/nonexistent')
        self.assertEqual(st, 'failed')
        self.assertNotEqual(st, 'not_run')
        base = {'artifact': 'x', 'episode_id': 'e'}
        ok = C.build_status_doc(base, None, 'complete', ('passed', None),
                                ('passed', None), 'complete_validated')
        self.assertEqual(ok['dataset_eligibility'], 'undetermined')
        self.assertEqual(ok['process_exit_result'], 'success')
        bad = C.build_status_doc(base, None, 'complete', ('passed', None),
                                 ('not_run', 'integrity_not_run'),
                                 'integrity_failed')
        self.assertEqual(bad['dataset_eligibility'], 'ineligible')
        self.assertEqual(bad['process_exit_result'], 'failure')

    # --- R-02 publish --------------------------------------------------
    def _mk(self):
        tmp = tempfile.mkdtemp()
        staging = os.path.join(tmp, 'stg'); os.makedirs(staging)
        return tmp, staging, os.path.join(tmp, 'out')

    def _patch_raises(self, calls_ok, exc=OSError('boom')):
        orig = C._os_replace
        state = {'n': 0}

        def fake(a, b):
            state['n'] += 1
            if state['n'] in calls_ok:
                return orig(a, b)
            raise exc
        C._os_replace = fake
        return orig

    def test_publish_no_old_target_success(self):
        _tmp, staging, out = self._mk()
        res = C._publish(staging, out)
        self.assertTrue(res['ok']); self.assertTrue(os.path.isdir(out))

    def test_publish_no_old_target_failure_keeps_staging(self):
        _tmp, staging, out = self._mk()
        orig = self._patch_raises(calls_ok=())
        try:
            res = C._publish(staging, out)
        finally:
            C._os_replace = orig
        self.assertFalse(res['ok']); self.assertTrue(os.path.isdir(staging))
        self.assertFalse(os.path.exists(out))

    def test_publish_backup_name_conflict_fail_closed(self):
        _tmp, staging, out = self._mk()
        os.makedirs(out); open(os.path.join(out, 'marker'), 'w').write('old')
        os.makedirs(C._backup_path(out))
        res = C._publish(staging, out)
        self.assertFalse(res['ok'])
        self.assertIn('backup_name_conflict', res['detail'])
        self.assertTrue(os.path.isfile(os.path.join(out, 'marker')))

    def test_publish_backup_move_failure_leaves_target(self):
        _tmp, staging, out = self._mk()
        os.makedirs(out); open(os.path.join(out, 'marker'), 'w').write('old')
        orig = self._patch_raises(calls_ok=())
        try:
            res = C._publish(staging, out)
        finally:
            C._os_replace = orig
        self.assertFalse(res['ok'])
        self.assertTrue(os.path.isfile(os.path.join(out, 'marker')))
        self.assertTrue(os.path.isdir(staging))

    def test_publish_staging_move_failure_restores(self):
        _tmp, staging, out = self._mk()
        os.makedirs(out); open(os.path.join(out, 'marker'), 'w').write('old')
        orig = self._patch_raises(calls_ok=(1, 3))   # 1:->backup, 3:restore
        try:
            res = C._publish(staging, out)
        finally:
            C._os_replace = orig
        self.assertFalse(res['ok']); self.assertTrue(res['restored'])
        self.assertTrue(os.path.isfile(os.path.join(out, 'marker')))

    def test_publish_restore_failure_keeps_backup(self):
        _tmp, staging, out = self._mk()
        os.makedirs(out); open(os.path.join(out, 'marker'), 'w').write('old')
        orig = self._patch_raises(calls_ok=(1,))     # only target->backup ok
        try:
            res = C._publish(staging, out)
        finally:
            C._os_replace = orig
        self.assertFalse(res['ok']); self.assertFalse(res['restored'])
        self.assertTrue(res['backup']
                        and os.path.isfile(os.path.join(res['backup'],
                                                        'marker')))

    # --- R-14 protection (temp protected root; never the real dirs) -----
    def test_protected_path_logic(self):
        self.assertTrue(C.is_protected_path(self._prot))
        self.assertTrue(C.is_protected_path(os.path.join(self._prot, 'sub', 'x')))
        self.assertTrue(C.is_protected_path(
            os.path.join(self._prot_tmp, 'hist_root', '..', 'hist_root', 'y')))
        self.assertTrue(C.is_protected_path(self._prot_tmp))  # ancestor
        self.assertFalse(C.is_protected_path(os.path.join(self._prot_tmp, 'other')))
        with self.assertRaises(PermissionError):
            C.guard_path(self._prot)

    def test_finalize_refuses_protected_out_root(self):
        asm = fixture_assembler()
        records = fixture_records()
        args = NS(episode_id='e', out_root=self._prot, allow_overwrite=False)
        with self.assertRaises(PermissionError):
            C._finalize(args, asm, records, {})

    # --- gated inputs / config separation ------------------------------
    def test_theta_id_field_bound_fails_closed_on_bad_id(self):
        # Owner-authorized binding: theta_id is read from the matched frozen
        # entry's `theta_id`. A non-string / blank / missing id still fails
        # closed (never an unverified identity).
        self.assertEqual(C.THETA_ID_FIELD, 'theta_id')
        self.assertEqual(C.theta_id_from_entry({'theta_id': 'r0_anchor'}),
                         'r0_anchor')
        with self.assertRaises(RuntimeError):
            C.theta_id_from_entry({'theta_id': ''})
        with self.assertRaises(RuntimeError):
            C.theta_id_from_entry({})

    def test_source_integrity_coverage_unverified(self):
        self.assertIsNone(C.SOURCE_MODULE_LIST)
        prov = C.source_integrity_provenance()
        self.assertEqual(prov['coverage'], 'unverified')
        self.assertTrue(prov['files'])

    def test_meta_requested_vs_effective_config(self):
        asm = fixture_assembler()
        asm.requested_config = {'theta': {'x': 1}}
        meta = asm.meta(fixture_records(n_frames=2))
        self.assertEqual(meta['requested_config'], {'theta': {'x': 1}})
        self.assertEqual(meta['effective_config']['state'], 'unverified')

    def test_published_unvalidated_phase_never_validated(self):
        # F-04 note: with OD-4=A the integrity gate is fixed, so a COMPLETE
        # episode can validate. This test keeps the original invariant by using
        # a NON-validated episode (checker rejection) and asserting the
        # published directory is never marked validated.
        records = fixture_records()
        asm = fixture_assembler()
        orig = asm.build_frames

        def broken(recs):
            frames = orig(recs)
            frames[1]['zmp_position'] = [0.1, 0.2, 0.3]   # wrong shape
            return frames
        asm.build_frames = broken
        out = os.path.join(tempfile.mkdtemp(), 'ep')
        args = NS(episode_id='e', out_root=out, allow_overwrite=False)
        rc, status = C._finalize(args, asm, records, {})
        self.assertNotEqual(rc, 0)
        self.assertEqual(status['phase'], 'published_unvalidated')
        self.assertNotEqual(status['terminal_status'], 'complete_validated')
        on_disk = json.load(open(os.path.join(out, 'status.json'),
                                 encoding='utf-8'))
        self.assertNotEqual(on_disk.get('phase'), 'validated')
        self.assertNotEqual(on_disk.get('terminal_status'),
                            'complete_validated')


# ---------------------------------------------------------------------------
# 6.5 Known-defect minimal repair (F-01a / F-02 / F-03a) — FIXTURE; NOT RUN
# ---------------------------------------------------------------------------

class TestKnownDefectsRepair(unittest.TestCase):
    """Regression tests for the three targeted defects (invalid-clock
    propagation, conflicting IMU timestamps, pre-side-effect θ-list
    verification). None of these tests is executed under this task."""

    # --- F-01a: clock validity / no stale borrow ----------------------
    def test_clockstate_validity_transitions(self):
        cs = C.ClockState()
        self.assertFalse(cs.valid)
        self.assertIsNone(cs.receipt_sim_s())
        self.assertEqual(cs.state(), 'missing')
        cs.update(5.0)
        self.assertTrue(cs.valid)
        self.assertEqual(cs.receipt_sim_s(), 5.0)
        self.assertEqual(cs.state(), 'valid')
        cs.update(None)                       # current clock invalid
        self.assertFalse(cs.valid)
        self.assertIsNone(cs.receipt_sim_s())  # never borrows the cache
        self.assertEqual(cs.sim_s, 5.0)        # cache retained for reference
        self.assertEqual(cs.state(), 'invalid')

    def test_bool_receipt_never_borrows_stale_clock(self):
        cs = C.ClockState()
        store, errors, seq = [], [], itertools.count()
        cb = C._make_callback('left_contact_bool', store, errors, seq,
                              clock=cs)
        cb(F_bool(True))                       # clock never valid yet
        self.assertIsNone(store[0]['_recv_sim_s'])
        self.assertEqual(store[0]['_recv_clock_state'], 'missing')
        cs.update(5.0)                         # valid clock
        cb(F_bool(True))
        self.assertEqual(store[1]['_recv_sim_s'], 5.0)
        self.assertEqual(store[1]['_recv_clock_state'], 'valid')
        cs.update(None)                        # clock goes invalid
        cb(F_bool(True))
        self.assertIsNone(store[2]['_recv_sim_s'])   # no stale borrow (F-01a)
        self.assertEqual(store[2]['_recv_clock_state'], 'invalid')

    def test_assoc_bool_clock_invalid_is_unknown(self):
        recs = [dict(value=True, _recv_sim_s=None, _recv_epoch=0,
                     _recv_clock_state='invalid', _seq=0)]
        v, t, st, why = C.assoc_contact_bool(recs, 5.0, 0)
        self.assertIsNone(v)
        self.assertEqual(st, 'unknown')
        self.assertEqual(why, 'clock_invalid')
        self.assertNotEqual(v, False)          # never coerced to false

    def test_assoc_bool_valid_clock_path_unchanged(self):
        recs = [dict(value=True, _recv_sim_s=5.0, _recv_epoch=0,
                     _recv_clock_state='valid', _seq=0)]
        v, _t, st, why = C.assoc_contact_bool(recs, 5.02, 0)
        self.assertTrue(v)
        self.assertEqual(st, 'matched')
        self.assertIsNone(why)

    # --- F-02: conflicting same-timestamp IMU ------------------------
    def _imu(self, t, acc_z):
        return {'stamp_s': t, 'orientation': [0.0, 0.0, 0.0, 1.0],
                'linear_acceleration': [0.0, 0.0, acc_z],
                'angular_velocity': [0.0, 0.0, 0.0]}

    def test_imu_same_stamp_identical_content_matched(self):
        a = self._imu(1.0, 9.8)
        b = dict(a)
        imu = [a, b]
        j, st, why, _d = C.assoc_imu(imu, imu, [1.0, 1.0], [0, 0], 1.0, 0, 2.0)
        self.assertEqual(j, 0)
        self.assertEqual(st, 'matched')

    def test_imu_same_stamp_conflicting_content_ambiguous(self):
        imu = [self._imu(1.0, 9.8), self._imu(1.0, 9.9)]   # same stamp, diff data
        j, st, why, _d = C.assoc_imu(imu, imu, [1.0, 1.0], [0, 0], 1.0, 0, 2.0)
        self.assertIsNone(j)
        self.assertEqual(st, 'unknown')
        self.assertEqual(why, 'duplicate_ambiguous')

    def test_imu_unique_nearest_matched(self):
        imu = [self._imu(1.0, 9.8), self._imu(1.005, 9.8)]
        j, st, why, d = C.assoc_imu(imu, imu, [1.0, 1.005], [0, 0],
                                    1.0, 0, 2.0)
        self.assertEqual(j, 0)
        self.assertEqual(st, 'matched')
        self.assertEqual(d, 0.0)

    def test_imu_no_match_outside_window(self):
        imu = [self._imu(1.0, 9.8)]
        j, st, why, _d = C.assoc_imu(imu, imu, [1.0], [0], 1.5, 0, 3.0)
        self.assertIsNone(j)
        self.assertEqual(why, 'no_candidate')

    # --- F-03a: θ-list verification before side effects --------------
    def test_compute_theta_list_hash_matches_frozen_anchor(self):
        data = C.load_theta_list()
        self.assertEqual(data.get('theta_list_hash'), C.THETA_LIST_FIXED_HASH)
        self.assertEqual(C.compute_theta_list_hash(data),
                         C.THETA_LIST_FIXED_HASH)
        res = C.verify_theta_list(data)        # real list passes
        self.assertEqual(res['computed'], C.THETA_LIST_FIXED_HASH)
        self.assertTrue(res['declared_matches'])

    def test_verify_theta_list_detects_tampered_content(self):
        data = C.load_theta_list()
        data['theta_list'][0]['theta']['step_length_m'] = 999.0  # tamper
        # self-declared hash unchanged -> must still fail on the independent pin
        with self.assertRaises(RuntimeError):
            C.verify_theta_list(data)

    def test_verify_theta_list_rejects_missing_array(self):
        with self.assertRaises(RuntimeError):
            C.verify_theta_list({'theta_list_hash': C.THETA_LIST_FIXED_HASH})

    def test_cmd_collect_verifies_theta_before_collection(self):
        tampered = {'theta_list': [{'theta': {'x': 1}}],
                    'theta_list_hash': C.THETA_LIST_FIXED_HASH}
        called = {'collect': False}

        def spy_collect(*_a, **_k):
            called['collect'] = True
            raise AssertionError('collect_live must not run')

        orig_load, orig_collect = C.load_theta_list, C.collect_live
        C.load_theta_list = lambda: tampered
        C.collect_live = spy_collect
        try:
            args = NS(episode_id='e', theta='/nonexistent.json',
                      domain_id=None, gz_partition=None)
            rc = C.cmd_collect(args)
        finally:
            C.load_theta_list, C.collect_live = orig_load, orig_collect
        self.assertEqual(rc, 1)
        self.assertFalse(called['collect'])    # no acquisition side effect


# ---------------------------------------------------------------------------
# 6.6 F-04 per-file integrity (OD-4=A / OD-5=A) — FIXTURE; NOT RUN
# ---------------------------------------------------------------------------

class TestF04Integrity(unittest.TestCase):
    """T1–T14 regression coverage for F-04 (all 12 files frozen + hashed; the
    summary self-hash is non-circular; verify_integrity checks the fixed set
    fail-closed). Uses only isolated temp dirs; never touches real episodes."""

    def _frozen(self):
        _asm, _records, tmp, _summary = freeze_fixture(n_frames=3)
        return tmp

    def _meta_path(self, tmp):
        return os.path.join(tmp, C.SUMMARY_ARTIFACT)

    def _read_meta(self, tmp):
        return json.load(open(self._meta_path(tmp), encoding='utf-8'))

    def _write_meta(self, tmp, meta):
        C.write_json(self._meta_path(tmp), meta)

    # T1
    def test_T1_complete_artifacts_pass(self):
        tmp = self._frozen()
        self.assertEqual(C.verify_integrity(tmp), ('passed', None, None))

    # T2
    def test_T2_missing_required_file_fails(self):
        tmp = self._frozen()
        os.remove(os.path.join(tmp, 'raw', 'pose_info.json'))
        st, _why, _d = C.verify_integrity(tmp)
        self.assertEqual(st, 'failed')

    # T3
    def test_T3_required_file_without_hash_declaration_fails(self):
        tmp = self._frozen()
        meta = self._read_meta(tmp)
        del meta['artifact_hashes']['frames.jsonl']       # file present, no hash
        self._write_meta(tmp, meta)
        st, _why, detail = C.verify_integrity(tmp)
        self.assertEqual(st, 'failed')
        self.assertIn('without a hash declaration', detail)

    # T4
    def test_T4_partial_hash_map_fails(self):
        tmp = self._frozen()
        meta = self._read_meta(tmp)
        meta['artifact_hashes'] = {
            k: meta['artifact_hashes'][k]
            for k in ('frames.json', 'frames.jsonl')}
        self._write_meta(tmp, meta)
        self.assertEqual(C.verify_integrity(tmp)[0], 'failed')

    # T5
    def test_T5_modified_file_content_fails(self):
        tmp = self._frozen()
        with open(os.path.join(tmp, 'frames.jsonl'), 'a', encoding='utf-8') as fh:
            fh.write('\n')                        # tamper a required file
        self.assertEqual(C.verify_integrity(tmp)[0], 'failed')

    # T6
    def test_T6_hash_map_missing_or_empty_fails(self):
        tmp = self._frozen()
        meta = self._read_meta(tmp)
        del meta['artifact_hashes']
        self._write_meta(tmp, meta)
        self.assertEqual(C.verify_integrity(tmp)[0], 'failed')

        tmp2 = self._frozen()
        m2 = self._read_meta(tmp2)
        m2['artifact_hashes'] = {}
        self._write_meta(tmp2, m2)
        self.assertEqual(C.verify_integrity(tmp2)[0], 'failed')

    # T7
    def test_T7_invalid_hash_map_type_fails(self):
        tmp = self._frozen()
        meta = self._read_meta(tmp)
        meta['artifact_hashes'] = ['not', 'a', 'map']
        self._write_meta(tmp, meta)
        self.assertEqual(C.verify_integrity(tmp)[0], 'failed')

    # T8
    def test_T8_summary_self_hash_recomputable_and_detects_change(self):
        tmp = self._frozen()
        meta = self._read_meta(tmp)
        # recomputes deterministically from the unmodified content
        self.assertEqual(C.compute_summary_self_hash(meta),
                         meta['summary_self_hash'])
        # mutating the summary's final content (without fixing the self-hash)
        # must be detected
        meta['notes'] = ['tampered']
        self._write_meta(tmp, meta)
        self.assertEqual(C.verify_integrity(tmp)[0], 'failed')

    # T9
    def test_T9_extra_or_unsafe_declaration_fails(self):
        tmp = self._frozen()
        meta = self._read_meta(tmp)
        meta['artifact_hashes']['../evil'] = '0' * 64
        meta['artifact_hashes']['/etc/passwd'] = '0' * 64
        self._write_meta(tmp, meta)
        st, _why, detail = C.verify_integrity(tmp)
        self.assertEqual(st, 'failed')
        self.assertIn('unauthorized', detail)

    # T10
    def test_T10_path_escaping_symlink_fails_without_reading(self):
        tmp = self._frozen()
        target = os.path.join(tmp, 'raw', 'pose_info.json')
        os.remove(target)
        try:
            os.symlink('/etc/hostname', target)      # escape the episode root
        except (OSError, NotImplementedError):
            self.skipTest('symlinks unavailable')
        st, _why, detail = C.verify_integrity(tmp)
        self.assertEqual(st, 'failed')
        self.assertIn('unsafe artifact path', detail)

    # T11
    def test_T11_not_run_and_failed_never_complete_validated(self):
        self.assertEqual(
            C.classify_terminal('complete', 'passed', 'not_run', [], []),
            'integrity_failed')
        base = {'artifact': 'x', 'episode_id': 'e'}
        doc = C.build_status_doc(base, None, 'complete', ('passed', None),
                                 ('failed', 'integrity_failed'),
                                 'integrity_failed')
        self.assertNotEqual(doc['terminal_status'], 'complete_validated')
        self.assertEqual(doc['process_exit_result'], 'failure')

    # T12
    def test_T12_freeze_hashes_all_12_files(self):
        tmp = self._frozen()
        meta = self._read_meta(tmp)
        self.assertEqual(set(meta['artifact_hashes']),
                         set(C.EPISODE_ARTIFACT_FILES))
        self.assertEqual(len(C.EPISODE_ARTIFACT_FILES), 12)
        for rel in C.EPISODE_ARTIFACT_FILES:
            self.assertTrue(C._valid_sha256(meta['artifact_hashes'][rel]))

    # T13
    def test_T13_episode_integrity_hash_semantics_preserved(self):
        tmp = self._frozen()
        prov = json.load(open(os.path.join(tmp, 'episode_provenance.json'),
                              encoding='utf-8'))
        self.assertEqual(prov['episode_integrity_hash'],
                         C.sha256_file(os.path.join(tmp, 'frames.jsonl')))
        meta = self._read_meta(tmp)
        self.assertEqual(meta['artifact_hashes']['frames.jsonl'],
                         prov['episode_integrity_hash'])
        # the single frames.jsonl hash is NOT the whole protection: tampering a
        # different required file is still detected
        with open(os.path.join(tmp, 'raw_records.json'), 'a',
                  encoding='utf-8') as fh:
            fh.write('\n')
        self.assertEqual(C.verify_integrity(tmp)[0], 'failed')

    # T14
    def test_T14_tampered_summary_hash_declaration_detected(self):
        tmp = self._frozen()
        meta = self._read_meta(tmp)
        meta['artifact_hashes']['frames.json'] = '0' * 64     # wrong declared hash
        self._write_meta(tmp, meta)
        self.assertEqual(C.verify_integrity(tmp)[0], 'failed')
        tmp2 = self._frozen()
        m2 = self._read_meta(tmp2)
        m2['summary_self_hash'] = '0' * 64                    # wrong self hash
        self._write_meta(tmp2, m2)
        self.assertEqual(C.verify_integrity(tmp2)[0], 'failed')

    # C-1 (F-04): a summary carrying a non-finite value (NaN / Infinity) makes
    # the canonical self-hash serialization (allow_nan=False) un-computable.
    # verify_integrity MUST return the contract failure tuple ('failed',
    # 'integrity_failed', <reason>) — the exception must never escape. The
    # injected value sits in the non-hash `notes` field, so every earlier gate
    # (declared hashes, file existence, per-file hash match, path safety)
    # passes and the ONLY reachable stop is the summary self-hash computation.
    def test_C1_nonfinite_summary_selfhash_fails_without_raising(self):
        for bad in (float('nan'), float('inf')):
            tmp = self._frozen()
            # unmodified episode passes: the fixture is otherwise fully valid
            self.assertEqual(C.verify_integrity(tmp), ('passed', None, None))
            meta = self._read_meta(tmp)
            meta['notes'] = [bad]
            with open(self._meta_path(tmp), 'w', encoding='utf-8') as fh:
                json.dump(meta, fh, allow_nan=True)   # write NaN/Infinity literal
            # the non-finite value survives the plain read the verifier uses
            back = json.load(open(self._meta_path(tmp), encoding='utf-8'))
            self.assertFalse(math.isfinite(back['notes'][0]))
            try:
                state, why, detail = C.verify_integrity(tmp)
            except Exception as exc:  # noqa: BLE001
                self.fail('verify_integrity raised %s instead of returning the '
                          'failure tuple: %s' % (type(exc).__name__, exc))
            self.assertEqual(state, 'failed')
            self.assertEqual(why, 'integrity_failed')
            # the failure is the self-hash COMPUTATION stop, not an earlier
            # gate (declared hashes / file existence / per-file mismatch)
            self.assertIn('self-hash computation failed', detail)
            self.assertNotIn('mismatch', detail)


if __name__ == '__main__':
    unittest.main(verbosity=2)
