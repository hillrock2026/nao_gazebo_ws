#!/usr/bin/env python3
"""Stage 1 native collector — minimal, contract-bound (Level 2).

Implements the frozen data flow (task § 0):

    Frozen gait baseline -> existing execution path (running simulation)
      -> Gazebo native measurements -> native collector -> raw artifacts +
      provenance + frame records -> frozen R15 contract checker

Design (task § 5; archaeology report):
  * It is a **passive subscriber**, never a runner: it subscribes to existing
    topics on a simulation started by the frozen runner/launch path and writes
    Stage 1 artifacts. It launches nothing, publishes nothing, and modifies no
    frozen component.
  * Native contacts are captured on the **gz-transport native** path
    (`/nao/{left,right}_foot_contact_raw`, gz.msgs.Contacts) — the canonical
    lossless raw source (RFC § 3.6.1, contract § 6.1). The ROS bridge copy is
    not used as raw evidence (it loses per-contact header / world /
    force_offset). Pose comes from the gz-only `/world/nao_world/pose/info`.
  * Raw records are retained **losslessly** (no drop / round / aggregate): the
    full per-contact-point arrays, both body-side wrenches incl. force_offset,
    every Pose_V entry, the 24-vector command, joint states, body pose.
  * Silence is never converted to data: a frame with no associated contact
    message records contact state `UNKNOWN_NO_MESSAGE` (no zero wrench, no
    fabricated flight/no-contact label). No interpolation / hold / zero-fill.
  * Derived fields follow RFC § 3.3-3.7 and the R11-A propagation rule:
    composed foot/CoM/zmp fields are `derived_pending_validation` (Option A:
    value retained); wrench-frame-dependent fields are `null` + blocked
    (Option B). `composition_status` is never upgraded from `UNVERIFIED`.
  * The 18 RFC § 3.10 provenance fields are emitted; the status manifest is
    schema-driven (field groups read from the frozen R15 checker so the
    collector cannot drift from Appendix D).
  * Artifacts are written atomically (staging dir -> os.replace); an episode
    directory is never silently overwritten; the outcome is classified in a
    NON-normative `status.json`.
  * Validation calls the frozen `tools/stage1_contract_check.py --smoke`
    (called, never modified). Its exit code decides `complete_validated` vs
    `contract_rejected`.

Subcommands:
  collect --episode-id ID --out-root DIR --duration S --theta JSON --seed N
          --ring R   (subscribe to a running sim; write artifacts + run smoke)
  verify  --episode-dir DIR   (strict re-read + re-run the frozen checker)
  smoke   --episode-dir DIR   (only re-run the frozen checker)

Run the collector with the ROS 2 system python so rclpy/gz transport import:

    source /opt/ros/jazzy/setup.bash
    /usr/bin/python3 tools/stage1_native_collector.py collect ...
"""

import argparse
import hashlib
import itertools
import json
import math
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone

TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(TOOLS)
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

import stage1_probe as probe  # frozen instrumentation helpers (reused)
import stage1_contract_check as checker  # frozen R15 checker (schema source)

RFC_PATH = os.path.join(WS, 'docs', 'STAGE1_DYNAMICS_DATASET_RFC.md')
CONTRACT_PATH = os.path.join(WS, 'docs', 'STAGE1_COLLECTOR_CONTRACT.md')
SDF_PATH = os.path.join(WS, 'src', 'nao', 'nao_description', 'urdf',
                        'nao_simple.sdf')
WORLD_PATH = os.path.join(WS, 'src', 'nao_gz_sim', 'worlds', 'nao_world.sdf')
THETA_LIST_PATH = os.path.join(WS, 'dataset', 'stage0',
                               'stage0_theta_list.json')

LEFT_CONTACT_TOPIC = '/nao/left_foot_contact_raw'
RIGHT_CONTACT_TOPIC = '/nao/right_foot_contact_raw'
POSE_INFO_TOPIC = '/world/nao_world/pose/info'
JOINT_STATES_TOPIC = '/joint_states'
JOINT_COMMAND_TOPIC = '/joint_command'
BODY_POSE_TOPIC = '/body_pose'
IMU_TOPIC = '/imu/data'
LEFT_BOOL_TOPIC = '/left_foot/contact'
RIGHT_BOOL_TOPIC = '/right_foot/contact'

STREAMS = (  # kind, transport, topic
    ('contacts_left', 'gz', LEFT_CONTACT_TOPIC),
    ('contacts_right', 'gz', RIGHT_CONTACT_TOPIC),
    ('pose_info', 'gz', POSE_INFO_TOPIC),
    ('joint_states', 'ros', JOINT_STATES_TOPIC),
    ('joint_command', 'ros', JOINT_COMMAND_TOPIC),
    ('body_pose', 'ros', BODY_POSE_TOPIC),
)
FRAME_AUX_STREAMS = (  # not raw categories; feed measured frame fields only
    ('imu', 'ros', IMU_TOPIC),
    ('left_contact_bool', 'ros', LEFT_BOOL_TOPIC),
    ('right_contact_bool', 'ros', RIGHT_BOOL_TOPIC),
)

# Association windows (RFC § 3.6.3 / contract § 5.2); causal last-not-after.
CONTACT_WINDOW_SEC = 0.05
POSE_WINDOW_SEC = 0.02
BODY_POSE_WINDOW_SEC = 0.02
BOOL_WINDOW_SEC = 0.05
COMMAND_WINDOW_SEC = 0.05
IMU_WINDOW_SEC = 0.01

G_PINNED = 9.81          # RFC § 3.7 kinematics formula constant (pinned)
FLOOR_Z = 0.0            # nao_world.sdf ground plane z (pinned constant)

EXPECTED_LINKS = list(probe.EXPECTED_LINKS)  # 25 links (RFC § 0.1)


def _joint_order():
    """Canonical 24-joint order (frozen nao_trajectory_schema; RFC § 3.2)."""
    try:
        import nao_trajectory_schema
        return list(nao_trajectory_schema.JOINT_ORDER)
    except Exception:  # noqa: BLE001 — fall back to the frozen source file
        pass
    import re
    src = os.path.join(WS, 'src', 'nao_gz_sim', 'scripts',
                       'nao_trajectory_schema.py')
    with open(src, 'r', encoding='utf-8') as fh:
        text = fh.read()
    m = re.search(r'JOINT_ORDER\s*=\s*\[(.*?)\]', text, re.DOTALL)
    if not m:
        raise RuntimeError('cannot resolve JOINT_ORDER')
    return re.findall(r"['\"]([^'\"]+)['\"]", m.group(1))


_gait_module = None


def _gait_phase_module():
    global _gait_module
    if _gait_module is None:
        try:
            import nao_gait_phase as gp
            _gait_module = gp
        except Exception:  # noqa: BLE001
            _gait_module = False
    return _gait_module or None


# ---------------------------------------------------------------------------
# Small field helpers (verbatim, defensive) — mirror the probe's accessors.
# ---------------------------------------------------------------------------

def _field(msg, name, default=None):
    return getattr(msg, name, default)


def _vec3(v):
    if v is None:
        return None
    try:
        return [float(v.x), float(v.y), float(v.z)]
    except AttributeError:
        return None


def _quat(q):
    if q is None:
        return None
    try:
        return [float(q.x), float(q.y), float(q.z), float(q.w)]
    except AttributeError:
        return None


def _stamp_seconds(header, default=None):
    if header is None:
        return default
    stamp = getattr(header, 'stamp', None)
    if stamp is None:
        stamp = header
    sec = getattr(stamp, 'sec', None)
    nsec = getattr(stamp, 'nanosec', None)
    if nsec is None:
        nsec = getattr(stamp, 'nsec', None)
    if sec is None:
        return default
    return float(sec) + float(nsec or 0) * 1e-9


def _header_verbatim(header):
    """gz Header -> {stamp:{sec,nsec}} verbatim (timestamp retained)."""
    if header is None:
        return None
    stamp = getattr(header, 'stamp', None)
    if stamp is None:
        return {}
    return {'stamp': {'sec': int(getattr(stamp, 'sec', 0)),
                      'nsec': int(getattr(stamp, 'nsec', 0))}}


def _entity_name(ent):
    name = _field(ent, 'name')
    return name if isinstance(name, str) else None


def _entity_id(ent):
    v = _field(ent, 'id')
    return int(v) if isinstance(v, int) and not isinstance(v, bool) else None


def _entity_record(ent):
    """Verbatim entity identity {id, name, type} for nested gz Entity fields
    (e.g. Contact.world); None when the field is absent."""
    if ent is None:
        return None
    return {'id': _entity_id(ent), 'name': _entity_name(ent),
            'type': str(_field(ent, 'type'))}


def _wrench_members(w):
    """gz Wrench -> {force, torque, force_offset} verbatim (3-vectors)."""
    if w is None:
        return None
    return {
        'force': _vec3(_field(w, 'force')),
        'torque': _vec3(_field(w, 'torque')),
        'force_offset': _vec3(_field(w, 'force_offset')),
    }


# ---------------------------------------------------------------------------
# Lossless record builders (Appendix D raw shapes). Return None on a message
# that cannot be faithfully represented — never a partially invented record.
# ---------------------------------------------------------------------------

def build_contacts_record(stream, msg):
    header = _field(msg, 'header')
    stamp = _stamp_seconds(header)
    raw = _field(msg, 'contact') or []
    contacts = []
    for c in raw:
        c1, c2 = _field(c, 'collision1'), _field(c, 'collision2')
        positions = [_vec3(p) for p in (_field(c, 'position') or [])]
        normals = [_vec3(n) for n in (_field(c, 'normal') or [])]
        depths = [float(d) for d in (_field(c, 'depth') or [])]
        wrenches = []
        for w in (_field(c, 'wrench') or []):
            wrenches.append({
                'body_1_name': _field(w, 'body_1_name'),
                'body_1_id': _field(w, 'body_1_id'),
                'body_2_name': _field(w, 'body_2_name'),
                'body_2_id': _field(w, 'body_2_id'),
                'body_1_wrench': _wrench_members(_field(w, 'body_1_wrench')),
                'body_2_wrench': _wrench_members(_field(w, 'body_2_wrench')),
            })
        contacts.append({
            'header': _header_verbatim(_field(c, 'header')),
            'collision1': _entity_name(c1),
            'collision2': _entity_name(c2),
            # lossless identity extras (unknown native keys are preserved by
            # the checker; the required keys above stay strings/objects)
            'collision1_id': _entity_id(c1),
            'collision2_id': _entity_id(c2),
            'collision1_type': str(_field(c1, 'type')) if c1 is not None
            else None,
            'collision2_type': str(_field(c2, 'type')) if c2 is not None
            else None,
            'world': _entity_record(_field(c, 'world')),
            'position': positions,
            'normal': normals,
            'depth': depths,
            'wrenches': wrenches,
        })
    return {
        'stream': stream,
        'stamp_s': stamp,
        'header': _header_verbatim(header),
        'n_contacts': len(contacts),
        'contacts': contacts,
    }


def build_pose_info_record(msg):
    poses = _field(msg, 'pose') or []
    entries = []
    for p in poses:
        entries.append({
            'name': _field(p, 'name'),
            'id': _entity_id(p),
            'position': _vec3(_field(p, 'position')),
            'orientation': _quat(_field(p, 'orientation')),
        })
    return {
        'stream': 'pose_info',
        'stamp_s': _stamp_seconds(_field(msg, 'header')),
        'header': _header_verbatim(_field(msg, 'header')),
        'n_entries': len(entries),
        'entries': entries,
    }


def build_joint_states_record(msg):
    names = list(_field(msg, 'name') or [])
    return {
        'stream': 'joint_states',
        'stamp_s': _stamp_seconds(_field(msg, 'header')),
        'name': [str(n) for n in names],
        'position': [float(v) for v in (_field(msg, 'position') or [])],
        'velocity': [float(v) for v in (_field(msg, 'velocity') or [])],
        'effort': [float(v) for v in (_field(msg, 'effort') or [])],
    }


def build_joint_command_record(msg):
    """`/joint_command` is published as sensor_msgs/JointState in the live
    producer (the RFC prose names JointTrajectory — recorded discrepancy);
    the actual command reference is `.position` (24 floats)."""
    positions = _field(msg, 'position')
    if positions is None:
        points = _field(msg, 'points') or []
        positions = _field(points[0], 'positions') if points else None
    return {
        'stream': 'joint_command',
        'stamp_s': _stamp_seconds(_field(msg, 'header')),
        'name': [str(n) for n in (_field(msg, 'name') or [])],
        'command': [float(v) for v in (positions or [])],
    }


def build_body_pose_record(msg):
    pose = _field(msg, 'pose')
    return {
        'stream': 'body_pose',
        'stamp_s': _stamp_seconds(_field(msg, 'header')),
        'frame_id': _field(_field(msg, 'header'), 'frame_id'),
        'position': _vec3(_field(pose, 'position')) if pose is not None
        else None,
        'orientation': _quat(_field(pose, 'orientation')) if pose is not None
        else None,
    }


def build_imu_record(msg):
    return {
        'stamp_s': _stamp_seconds(_field(msg, 'header')),
        'orientation': _quat(_field(msg, 'orientation')),
        'linear_acceleration': _vec3(_field(msg, 'linear_acceleration')),
        'angular_velocity': _vec3(_field(msg, 'angular_velocity')),
    }


def build_bool_record(msg):
    v = _field(msg, 'data')
    return {'stamp_s': None, 'value': bool(v) if v is not None else None}


BUILDERS = {
    'contacts_left': lambda m: build_contacts_record('contacts_left', m),
    'contacts_right': lambda m: build_contacts_record('contacts_right', m),
    'pose_info': build_pose_info_record,
    'joint_states': build_joint_states_record,
    'joint_command': build_joint_command_record,
    'body_pose': build_body_pose_record,
    'imu': build_imu_record,
    'left_contact_bool': build_bool_record,
    'right_contact_bool': build_bool_record,
}


# ---------------------------------------------------------------------------
# Quaternion / composition geometry (RFC § 3.3 / contract § 4.3).
# ---------------------------------------------------------------------------

def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


def quat_to_R(q):
    x, y, z, w = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def rotate(R, v):
    return [R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2]
            for i in range(3)]


def compose_pose(p_model, q_model, p_rel, q_rel):
    """T_world_link = T_world_model . T_model_link (Hamilton)."""
    R = quat_to_R(q_model)
    t = [p_model[i] + rotate(R, p_rel)[i] for i in range(3)]
    return t, quat_mul(q_model, q_rel)


def _quat_log_rotation_vector(q):
    """Axis-angle vector of a unit quaternion [x,y,z,w] (world frame)."""
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z)
    if n < 1e-12:
        return [0.0, 0.0, 0.0]
    angle = 2.0 * math.atan2(n, w)
    return [angle * x / n, angle * y / n, angle * z / n]


# ---------------------------------------------------------------------------
# Association: causal last-not-after within a window (index-sorted streams).
# ---------------------------------------------------------------------------

def causal_index(stamps, t, window):
    """Largest i with stamps[i] <= t and t - stamps[i] <= window, else None.

    `stamps` must be sorted ascending with no None (callers filter+sort)."""
    import bisect
    i = bisect.bisect_right(stamps, t) - 1
    if i < 0:
        return None
    return i if (t - stamps[i]) <= window else None


# ---------------------------------------------------------------------------
# Finite differences (central; one-sided at boundaries). None-aware.
# ---------------------------------------------------------------------------

def finite_difference(times, values):
    """values: list of vector lists or None; returns list of vectors or None."""
    out = []
    n = len(times)
    for i in range(n):
        v = values[i]
        if v is None:
            out.append(None)
            continue
        if i > 0 and i < n - 1 and times[i + 1] is not None \
                and times[i - 1] is not None \
                and times[i + 1] != times[i - 1]:
            dt = times[i + 1] - times[i - 1]
            a, b = values[i + 1], values[i - 1]
            if a is None or b is None:
                out.append(None)
                continue
            out.append([(a[k] - b[k]) / dt for k in range(len(v))])
        elif n > 1 and i == 0 and times[1] is not None \
                and times[0] is not None and times[1] != times[0] \
                and values[1] is not None:
            dt = times[1] - times[0]
            out.append([(values[1][k] - v[k]) / dt for k in range(len(v))])
        elif n > 1 and i == n - 1 and times[n - 1] is not None \
                and times[n - 2] is not None \
                and times[n - 1] != times[n - 2] \
                and values[n - 2] is not None:
            dt = times[n - 1] - times[n - 2]
            out.append([(v[k] - values[n - 2][k]) / dt
                        for k in range(len(v))])
        else:
            out.append(None)
        # second-difference form for acceleration handled by caller
    return out


def second_difference(times, values):
    """Second derivative via successive finite differences."""
    first = finite_difference(times, values)
    return finite_difference(times, first)


def quat_angular_velocity(times, quats):
    out = []
    n = len(times)
    for i in range(n):
        q = quats[i]
        if q is None:
            out.append(None)
            continue
        if i > 0 and i < n - 1 and quats[i + 1] is not None \
                and quats[i - 1] is not None and times[i + 1] is not None \
                and times[i - 1] is not None and times[i + 1] != times[i - 1]:
            q0, q1 = quats[i - 1], quats[i + 1]
            dt = times[i + 1] - times[i - 1]
            inv = [-q0[0], -q0[1], -q0[2], q0[3]]
            dq = quat_mul(q1, inv)
            rv = _quat_log_rotation_vector(dq)
            out.append([c / dt for c in rv])
        else:
            out.append(None)
    return out


# ---------------------------------------------------------------------------
# SDF inertial constants (pinned model → frozen CoM constants, RFC § 3.4).
# ---------------------------------------------------------------------------

def load_sdf_inertials(path=SDF_PATH):
    """Return {link: {'mass': m, 'com': [x,y,z]}} for the 25 links + sha256."""
    import re
    text = open(path, 'r', encoding='utf-8').read()
    sha = hashlib.sha256(text.encode('utf-8')).hexdigest()
    out = {}
    for m in re.finditer(r"<link\s+name=['\"]([^'\"]+)['\"]>(.*?)</link>",
                         text, re.DOTALL):
        name, body = m.group(1), m.group(2)
        im = re.search(r'<inertial>(.*?)</inertial>', body, re.DOTALL)
        if not im:
            continue
        ib = im.group(1)
        mass_m = re.search(r'<mass>([-\d.eE+]+)</mass>', ib)
        pose_m = re.search(r'<pose>([^<]+)</pose>', ib)
        if not mass_m:
            continue
        com = [0.0, 0.0, 0.0]
        if pose_m:
            vals = [float(v) for v in pose_m.group(1).split()]
            if len(vals) >= 3:
                com = vals[:3]
        out[name] = {'mass': float(mass_m.group(1)), 'com': com}
    return out, sha


# ---------------------------------------------------------------------------
# Episode assembler: raw records -> frames -> artifacts (pure; no I/O).
# ---------------------------------------------------------------------------

def _status_map():
    """field_status map straight from the frozen normative schema."""
    schema = checker._SCHEMA
    out = {}
    for group in ('measured', 'ordinary_derived', 'option_a', 'option_b'):
        st = schema['frame_fields'][group]['status']
        for f in schema['frame_fields'][group]['fields']:
            out[f['name']] = st
    return out


def build_status_manifest():
    schema = checker._SCHEMA
    return {
        'composition_status': schema['status_semantics']['composition_status'],
        'wrench_frame_status': schema['status_semantics']['wrench_frame_status'],
        'field_status': _status_map(),
    }


class EpisodeAssembler:
    """Assemble one episode's artifacts from collected raw records.

    All inputs are plain dicts already produced by the builders (or, in tests,
    synthetic fixtures that mimic the same shapes). Pure: no I/O, no ROS.
    """

    def __init__(self, episode_id, theta, theta_id, seed, ring,
                 config_hash_source, wallclock_utc, git_commit,
                 rfc_hash, model_hash, world_hash, gz_version,
                 theta_list_hash, inertials, total_mass, sdf_sha,
                 joint_order=None):
        self.episode_id = episode_id
        self.theta = theta
        self.theta_id = theta_id
        self.seed = seed
        self.ring = ring
        self.config_hash_source = config_hash_source
        self.wallclock_utc = wallclock_utc
        self.git_commit = git_commit
        self.rfc_hash = rfc_hash
        self.model_hash = model_hash
        self.world_hash = world_hash
        self.gz_version = gz_version
        self.theta_list_hash = theta_list_hash
        self.inertials = inertials
        self.total_mass = total_mass
        self.sdf_sha = sdf_sha
        self.joint_order = list(joint_order) if joint_order else _joint_order()
        self.notes = []

    # -- raw ------------------------------------------------------------
    def raw_records(self, records):
        """records: {kind: [records]}. Returns the raw artifact array."""
        out = []
        for kind in ('contacts_left', 'contacts_right', 'pose_info',
                     'joint_command', 'joint_states', 'body_pose'):
            for rec in records.get(kind, []):
                out.append(_strip_private(rec))
        return out

    # -- frames ---------------------------------------------------------
    def build_frames(self, records):
        js = _by_stamp(records, 'joint_states')
        pose = _by_stamp(records, 'pose_info')
        body = _by_stamp(records, 'body_pose')
        cmd = _by_stamp(records, 'joint_command')
        imu = _by_stamp(records, 'imu')
        lcl = _by_stamp(records, 'contacts_left')
        rcl = _by_stamp(records, 'contacts_right')
        bools = {k: sorted([r for r in records.get(k, [])],
                           key=lambda r: r.get('_seq', -1))
                 for k in ('left_contact_bool', 'right_contact_bool')}

        pose_stamps = [r['stamp_s'] for r in pose]
        body_stamps = [r['stamp_s'] for r in body]
        cmd_stamps = [r['stamp_s'] for r in cmd]
        imu_stamps = [r['stamp_s'] for r in imu]
        lcl_stamps = [r['stamp_s'] for r in lcl]
        rcl_stamps = [r['stamp_s'] for r in rcl]

        frames = []
        for i, st in enumerate(js):
            t = st['stamp_s']
            if list(st.get('name', [])) != self.joint_order:
                self.notes.append('frame %d: joint_states name != JOINT_ORDER'
                                  % i)
            p_idx = causal_index(pose_stamps, t, POSE_WINDOW_SEC)
            b_idx = causal_index(body_stamps, t, BODY_POSE_WINDOW_SEC)
            c_idx = causal_index(cmd_stamps, t, COMMAND_WINDOW_SEC)
            i_idx = causal_index(imu_stamps, t, IMU_WINDOW_SEC)
            l_idx = causal_index(lcl_stamps, t, CONTACT_WINDOW_SEC)
            r_idx = causal_index(rcl_stamps, t, CONTACT_WINDOW_SEC)
            frame = {
                'episode_id': self.episode_id,
                'frame_id': i,
                'sim_time_sec': t,
                'joint_position': st.get('position'),
                'joint_velocity': st.get('velocity'),
                'base_pose': ({
                    'position': body[b_idx].get('position'),
                    'orientation': body[b_idx].get('orientation'),
                } if b_idx is not None else None),
                'pose_source_stamp': pose[p_idx]['stamp_s']
                if p_idx is not None else None,
                # contact booleans: /left_foot/contact is std_msgs/Bool (no
                # header stamp) -> associated by RECEIPT ORDER only, never a
                # time-based freshness claim (RFC § 3.6.2 keeps this Stage-0
                # comparability signal; it is not independent evidence)
                'left_contact': self._latest_bool(bools['left_contact_bool'],
                                                  st.get('_seq')),
                'right_contact': self._latest_bool(
                    bools['right_contact_bool'], st.get('_seq')),
                'imu_orientation': imu[i_idx].get('orientation')
                if i_idx is not None else None,
                'imu_linear_acceleration': imu[i_idx].get(
                    'linear_acceleration') if i_idx is not None else None,
                'imu_angular_velocity': imu[i_idx].get('angular_velocity')
                if i_idx is not None else None,
                'controller_command': cmd[c_idx].get('command')
                if c_idx is not None else None,
                # non-normative extras (honest silence semantics, RFC § 3.6.4)
                'left_contact_wrench_state': 'VALID_CONTACT'
                if l_idx is not None else 'UNKNOWN_NO_MESSAGE',
                'right_contact_wrench_state': 'VALID_CONTACT'
                if r_idx is not None else 'UNKNOWN_NO_MESSAGE',
                'left_contact_assoc_stamp': lcl[l_idx]['stamp_s']
                if l_idx is not None else None,
                'right_contact_assoc_stamp': rcl[r_idx]['stamp_s']
                if r_idx is not None else None,
                '_pose_index': p_idx,
            }
            frames.append(frame)

        self._add_derived(frames, pose)
        for f in frames:
            f.pop('_pose_index', None)
            _strip_private_inplace(f)
        return frames

    def _latest_bool(self, recs, frame_seq):
        if frame_seq is None:
            return None
        best = None
        for r in recs:
            if r.get('_seq') is not None and r['_seq'] < frame_seq:
                best = r
            else:
                break
        return None if best is None else best.get('value')

    def _add_derived(self, frames, pose):
        times = [f['sim_time_sec'] for f in frames]

        # ordinary derived (no unverified input consumed)
        jvel = [f.get('joint_velocity') for f in frames]
        jacc = finite_difference(times, jvel) if any(
            v is not None for v in jvel) else [None] * len(frames)
        base_pos = [(f['base_pose'] or {}).get('position')
                    if f['base_pose'] else None for f in frames]
        base_ori = [(f['base_pose'] or {}).get('orientation')
                    if f['base_pose'] else None for f in frames]
        bvel = finite_difference(times, base_pos)
        bang = quat_angular_velocity(times, base_ori)

        # option A: composition-dependent (derived_pending_validation)
        foot_l, foot_r, com = self._compose_per_frame(frames, pose)
        fl_vel = finite_difference(times, [p for p, _q in foot_l])
        fr_vel = finite_difference(times, [p for p, _q in foot_r])
        com_vel = finite_difference(times, com)
        com_acc = second_difference(times, com)

        for i, f in enumerate(frames):
            f['joint_acceleration'] = jacc[i]
            f['base_velocity'] = bvel[i]
            f['base_angular_velocity'] = bang[i]
            f['gait_phase'] = self._gait_phase(times[i], times[0])
            f['desired_velocity'] = self._desired_velocity()
            f['left_foot_pose'] = _pose_dict(foot_l[i])
            f['right_foot_pose'] = _pose_dict(foot_r[i])
            f['left_foot_velocity'] = fl_vel[i]
            f['right_foot_velocity'] = fr_vel[i]
            f['com_position'] = com[i]
            f['com_velocity'] = com_vel[i]
            f['com_acceleration'] = com_acc[i]
            f['zmp_position'] = _zmp(com[i], com_acc[i])
            # option B: wrench-frame-dependent -> present + null (blocked)
            for name in checker.WRENCH_FRAME_DEPENDENT_FIELDS:
                f[name] = None

    def _compose_per_frame(self, frames, pose):
        foot_l, foot_r, com = [], [], []
        for f in frames:
            p_idx = f['_pose_index']
            if p_idx is None:
                foot_l.append((None, None))
                foot_r.append((None, None))
                com.append(None)
                continue
            msg = pose[p_idx]
            by_name = {}
            for e in msg.get('entries', []):
                n = e.get('name')
                if n is not None and n not in by_name:
                    by_name[n] = e
            model = by_name.get('nao')
            if model is None or model.get('position') is None \
                    or model.get('orientation') is None:
                foot_l.append((None, None))
                foot_r.append((None, None))
                com.append(None)
                continue

            def composed(link):
                e = by_name.get(link)
                if e is None or e.get('position') is None \
                        or e.get('orientation') is None:
                    return None, None
                return compose_pose(model['position'], model['orientation'],
                                    e['position'], e['orientation'])

            pl, ql = composed('l_ankle')
            pr, qr = composed('r_ankle')
            foot_l.append((pl, ql))
            foot_r.append((pr, qr))
            # CoM: sum_i m_i (t_i + R_i . c_i) / sum_i m_i (RFC § 3.4)
            if self.total_mass and all(
                    k in by_name and by_name[k].get('position')
                    and by_name[k].get('orientation')
                    for k in self.inertials):
                acc = [0.0, 0.0, 0.0]
                for link, spec in self.inertials.items():
                    e = by_name[link]
                    t_i, q_i = compose_pose(model['position'],
                                            model['orientation'],
                                            e['position'], e['orientation'])
                    c = rotate(quat_to_R(q_i), spec['com'])
                    for k in range(3):
                        acc[k] += spec['mass'] * (t_i[k] + c[k])
                com.append([a / self.total_mass for a in acc])
            else:
                com.append(None)
        return foot_l, foot_r, com

    def _gait_phase(self, t, t_epoch):
        gp = _gait_phase_module()
        if gp is None:
            return None
        freq = float(self.theta.get('step_frequency_hz', 0.0) or 0.0)
        ds = float(self.theta.get('double_support_ratio', 0.3) or 0.3)
        start = float(self.theta.get('start_delay_sec', 0.0) or 0.0)
        elapsed = (t - t_epoch) - start
        if freq <= 0 or elapsed < 0:
            return None
        phase = (elapsed * freq) % 1.0
        name, frac, _a, _b = gp.classify(phase, ds)
        return {'phase': name, 'fraction': float(frac)}

    def _desired_velocity(self):
        sl = float(self.theta.get('step_length_m', 0.0) or 0.0)
        sf = float(self.theta.get('step_frequency_hz', 0.0) or 0.0)
        return [sl * sf, 0.0]

    # -- provenance / manifest / meta ----------------------------------
    def provenance(self, frame_count, t_start, t_end, integrity_hash):
        return {
            'theta': self.theta,
            'theta_id': self.theta_id,
            'theta_hash': _canonical_sha256(self.theta),
            'theta_list_hash': self.theta_list_hash,
            'git_commit': self.git_commit,
            'config_hash': _canonical_sha256(self.config_hash_source),
            'cop_derivation_hash': None,
            'gazebo_seed': self.seed,
            'robot_model_version': self.model_hash,
            'world_model_version': self.world_hash,
            'gz_sim_version': self.gz_version,
            'stage1_rfc_hash': self.rfc_hash,
            'acquisition_wallclock_utc': self.wallclock_utc,
            'ring': self.ring,
            'frame_count': frame_count,
            't_start': t_start,
            't_end': t_end,
            'episode_integrity_hash': integrity_hash,
        }

    def meta(self, records):
        return {
            'artifact': 'Stage 1 collector meta (NON-NORMATIVE)',
            'episode_id': self.episode_id,
            'stream_message_counts': {
                k: len(v) for k, v in sorted(records.items())},
            'association_windows_sec': {
                'contact': CONTACT_WINDOW_SEC, 'pose': POSE_WINDOW_SEC,
                'body_pose': BODY_POSE_WINDOW_SEC, 'bool': BOOL_WINDOW_SEC,
                'command': COMMAND_WINDOW_SEC, 'imu': IMU_WINDOW_SEC},
            'sdf_sha256': self.sdf_sha,
            'g_pinned': G_PINNED, 'floor_z': FLOOR_Z,
            'gait_phase_epoch_rule':
                't_epoch = first frame sim_time; elapsed = t - t_epoch - '
                'theta.start_delay_sec; phase = (elapsed*freq) % 1',
            'notes': list(self.notes),
        }


def _pose_dict(pt):
    p, q = pt
    return {'position': p, 'orientation': q}


def _by_stamp(records, kind):
    """Records of `kind` with a finite source stamp, sorted ascending."""
    rows = [r for r in records.get(kind, [])
            if isinstance(r.get('stamp_s'), (int, float))
            and not isinstance(r.get('stamp_s'), bool)]
    rows.sort(key=lambda r: r['stamp_s'])
    return rows


def _strip_private(rec):
    return {k: v for k, v in rec.items() if not str(k).startswith('_')}


def _strip_private_inplace(rec):
    for k in [k for k in rec if str(k).startswith('_')]:
        rec.pop(k, None)


def _zmp(com, com_acc):
    if com is None or com_acc is None:
        return None
    return [com[0] - ((com[2] - FLOOR_Z) / G_PINNED) * com_acc[0],
            com[1] - ((com[2] - FLOOR_Z) / G_PINNED) * com_acc[1]]


def _canonical_sha256(obj):
    return hashlib.sha256(
        json.dumps(obj, indent=2, sort_keys=True, allow_nan=False)
        .encode('utf-8')).hexdigest()


# ---------------------------------------------------------------------------
# Writing (atomic) + strict re-read + frozen checker invocation.
# ---------------------------------------------------------------------------

def write_json(path, payload):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        json.dump(payload, fh, indent=2, sort_keys=False, allow_nan=False)
        fh.write('\n')
    os.replace(tmp, path)


def write_jsonl(path, rows):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        for row in rows:
            fh.write(json.dumps(row, sort_keys=False, allow_nan=False))
            fh.write('\n')
    os.replace(tmp, path)


def sha256_file(path):
    return hashlib.sha256(open(path, 'rb').read()).hexdigest()


def _publish(staging, out_root):
    """Atomic publish: move any existing target aside, then os.replace the
    staging dir into place (frozen-runner `_publish` pattern). An existing
    episode is never silently overwritten in place."""
    if os.path.isdir(out_root):
        backup = out_root + '.replaced.%d' % os.getpid()
        try:
            os.replace(out_root, backup)
        except OSError:
            shutil.rmtree(out_root, ignore_errors=True)
            backup = None
        try:
            os.replace(staging, out_root)
        finally:
            if backup:
                shutil.rmtree(backup, ignore_errors=True)
    else:
        os.replace(staging, out_root)


def freeze_episode(assembler, records, staging):
    """Write all artifacts into `staging` (an existing empty dir)."""
    os.makedirs(os.path.join(staging, 'raw'), exist_ok=True)
    raw = assembler.raw_records(records)
    frames = assembler.build_frames(records)
    frame_count = len(frames)
    t_start = frames[0]['sim_time_sec'] if frames else None
    t_end = frames[-1]['sim_time_sec'] if frames else None

    write_jsonl(os.path.join(staging, 'frames.jsonl'), frames)
    integrity = sha256_file(os.path.join(staging, 'frames.jsonl'))
    frames_path = os.path.join(staging, 'frames.json')
    write_json(frames_path, frames)
    write_json(os.path.join(staging, 'raw_records.json'), raw)
    write_json(os.path.join(staging, 'status_manifest.json'),
               build_status_manifest())
    write_json(os.path.join(staging, 'episode_provenance.json'),
               assembler.provenance(frame_count, t_start, t_end, integrity))
    write_json(os.path.join(staging, 'collector_meta.json'),
               assembler.meta(records))
    for kind in ('contacts_left', 'contacts_right', 'pose_info',
                 'joint_command', 'joint_states', 'body_pose'):
        write_json(os.path.join(staging, 'raw', '%s.json' % kind),
                   records.get(kind, []))
    return {'frame_count': frame_count, 'n_raw': len(raw),
            't_start': t_start, 't_end': t_end,
            'episode_integrity_hash': integrity}


def run_frozen_checker(episode_dir, rfc=RFC_PATH, contract=CONTRACT_PATH):
    """Invoke the frozen R15 checker --smoke over an episode directory.
    Returns (exit_code, report_dict_or_None, stdout, stderr)."""
    cmd = [sys.executable, os.path.join(TOOLS, 'stage1_contract_check.py'),
           '--smoke', '--rfc', rfc, '--contract', contract,
           '--manifest', os.path.join(episode_dir, 'status_manifest.json'),
           '--provenance', os.path.join(
               episode_dir, 'episode_provenance.json'),
           '--frame', os.path.join(episode_dir, 'frames.json'),
           '--raw', os.path.join(episode_dir, 'raw_records.json')]
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    report = None
    try:
        report = json.loads(proc.stdout)
    except Exception:
        report = None
    return proc.returncode, report, proc.stdout, proc.stderr


# ---------------------------------------------------------------------------
# Live collection (needs rclpy + gz transport; a simulation must be running).
# ---------------------------------------------------------------------------

def _make_callback(kind, store, errors, seq):
    builder = BUILDERS[kind]

    def cb(msg):
        try:
            rec = builder(msg)
            rec['_seq'] = next(seq)  # receipt order (NOT a time claim)
            store.append(rec)
        except Exception as exc:  # noqa: BLE001
            errors.append({'kind': kind, 'index': next(seq),
                           'error': '%s: %s' % (type(exc).__name__, exc)})
    return cb


def collect_live(spec, duration, warmup_sec, transport, msg_types):
    """Subscribe every (kind, transport, topic) in spec; warm up; collect for
    `duration`; return {kind: [records]} + subscribe/error info."""
    session = probe.ProbeSession('stage1_native_collector',
                                 transport_module=transport)
    stores, subscribe_ok, errors = {}, {}, []
    ds = {}
    seq = itertools.count()
    missing = []
    try:
        for kind, tr, topic in spec:
            store = probe.LockedRecords()
            cb = _make_callback(kind, store, errors, seq)
            if tr == 'ros':
                ok = session.ros_subscribe(topic, msg_types[kind], cb)
            else:
                ok = session.gz_subscribe(msg_types[kind], topic, cb)
            stores[kind] = store
            subscribe_ok[kind] = ok
            ds[kind] = {'transport': tr, 'topic': topic, 'subscribe_ok': ok}
        missing = probe.wait_for_streams(stores, warmup_sec)
        if len(missing) < len(stores):
            time.sleep(duration)
        records = {k: s.snapshot() for k, s in stores.items()}
    finally:
        session.close()
    for k, s in stores.items():
        ds[k]['n_messages'] = len(records[k])
        ds[k]['empty_after_warmup'] = k in missing
    return records, {'streams': ds, 'callback_errors': errors,
                     'session_errors': list(session.session_errors)}


def _message_types():
    contacts_pb2, pose_v_pb2, _t = probe.gz_packages()
    import rclpy  # noqa: F401
    from sensor_msgs.msg import JointState, Imu
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Bool
    return {
        'contacts_left': contacts_pb2.Contacts,
        'contacts_right': contacts_pb2.Contacts,
        'pose_info': pose_v_pb2.Pose_V,
        'joint_states': JointState,
        'joint_command': JointState,
        'body_pose': PoseStamped,
        'imu': Imu,
        'left_contact_bool': Bool,
        'right_contact_bool': Bool,
    }


# ---------------------------------------------------------------------------
# Episode context (provenance inputs) from the repository.
# ---------------------------------------------------------------------------

def _git_commit():
    out = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=WS,
                         capture_output=True, text=True)
    commit = out.stdout.strip()
    if len(commit) != 40:
        raise RuntimeError('cannot resolve git HEAD (commit40 required)')
    return commit


def _gz_version():
    if not shutil.which('gz'):
        return 'unknown'
    out = subprocess.run(['gz', 'sim', '--version'], capture_output=True,
                         text=True)
    line = (out.stdout or '').splitlines()
    return line[0] if line else 'unknown'


def load_theta_list():
    with open(THETA_LIST_PATH, 'r', encoding='utf-8') as fh:
        return json.load(fh)


def resolve_theta(theta_json_path=None, theta_id_hint=None):
    """Load a θ dict; fail-closed cross-check membership in the frozen list."""
    data = load_theta_list()
    list_hash = data.get('theta_list_hash')
    entries = data.get('theta_list', [])
    if theta_json_path:
        with open(theta_json_path, 'r', encoding='utf-8') as fh:
            theta = json.load(fh)
        if isinstance(theta, dict) and 'theta' in theta \
                and isinstance(theta['theta'], dict):
            theta = theta['theta']
    else:
        raise RuntimeError('--theta <theta.json> is required')
    h = _canonical_sha256(theta)
    match = [e for e in entries if _canonical_sha256(e.get('theta', {})) == h]
    if not match:
        raise RuntimeError('theta_hash %s is not in the frozen Stage 0 list '
                           '(theta_list_hash=%s) — fail-closed'
                           % (h[:16], (list_hash or '?')[:16]))
    return theta, list_hash, {'computed': h}


def config_hash_source():
    return {
        'collector': 'stage1_native_collector/1.0',
        'streams': [list(s) for s in STREAMS],
        'windows_sec': {
            'contact': CONTACT_WINDOW_SEC, 'pose': POSE_WINDOW_SEC,
            'body_pose': BODY_POSE_WINDOW_SEC, 'bool': BOOL_WINDOW_SEC,
            'command': COMMAND_WINDOW_SEC, 'imu': IMU_WINDOW_SEC},
        'native_contact_path': 'gz-transport',
        'composition': 'T_world_link = T_world_model . T_model_link',
        'g_pinned': G_PINNED, 'floor_z': FLOOR_Z,
        'serialization': 'json indent=2 allow_nan=False; frames.jsonl',
    }


# ---------------------------------------------------------------------------
# High-level flows.
# ---------------------------------------------------------------------------

def cmd_collect(args):
    from_ros = args.domain_id
    if from_ros is not None:
        os.environ['ROS_DOMAIN_ID'] = str(from_ros)
    if args.gz_partition:
        os.environ['GZ_PARTITION'] = args.gz_partition
    import rclpy
    rclpy.init()
    try:
        transport = probe.gz_packages()[2]
        msg_types = _message_types()
        records, info = collect_live(
            list(STREAMS) + list(FRAME_AUX_STREAMS), args.duration,
            args.warmup_sec, transport, msg_types)
    finally:
        rclpy.shutdown()

    theta, theta_list_hash, _ = resolve_theta(args.theta)
    inertials, sdf_sha = load_sdf_inertials()
    total_mass = sum(v['mass'] for v in inertials.values())
    assembler = EpisodeAssembler(
        episode_id=args.episode_id, theta=theta, theta_id=args.episode_id,
        seed=args.seed, ring=args.ring,
        config_hash_source=config_hash_source(),
        wallclock_utc=datetime.now(timezone.utc).isoformat(),
        git_commit=_git_commit(), rfc_hash=sha256_file(RFC_PATH),
        model_hash=sdf_sha, world_hash=sha256_file(WORLD_PATH),
        gz_version=_gz_version(), theta_list_hash=theta_list_hash,
        inertials=inertials, total_mass=total_mass, sdf_sha=sdf_sha)
    rc, status = _finalize(args, assembler, records, info)
    print(json.dumps(status, indent=2))
    return rc


def _finalize(args, assembler, records, info):
    out_root = os.path.abspath(args.out_root)
    if os.path.exists(out_root) and not args.allow_overwrite:
        raise RuntimeError('out_root exists (use --allow-overwrite): %s'
                           % out_root)
    staging = out_root + '.staging.%d' % os.getpid()
    if os.path.exists(staging):
        shutil.rmtree(staging)
    os.makedirs(staging)
    status = {'artifact': 'Stage 1 collector status (NON-NORMATIVE)',
              'episode_id': assembler.episode_id,
              'streams': info.get('streams', {}),
              'callback_errors': info.get('callback_errors', []),
              'session_errors': info.get('session_errors', [])}
    try:
        summary = freeze_episode(assembler, records, staging)
    except Exception as exc:
        status['status'] = 'write_failed'
        status['error'] = '%s: %s' % (type(exc).__name__, exc)
        try:
            write_json(os.path.join(staging, 'status.json'), status)
            _publish(staging, out_root)
        finally:
            pass
        return 1, status
    status.update(summary)
    rc, report, _out, _err = run_frozen_checker(staging)
    status['checker_exit_code'] = rc
    status['checker_ok'] = bool(report and report.get('smoke', {}).get('ok'))
    if report is not None:
        write_json(os.path.join(staging, 'smoke_report.json'), report)
    if not records.get('joint_states'):
        status['status'] = 'collection_failed'
    elif not all(records.get(k) for k in
                 ('contacts_left', 'contacts_right', 'pose_info',
                  'joint_command')):
        status['status'] = 'partial'
    elif rc == 0:
        status['status'] = 'complete_validated'
    else:
        status['status'] = 'contract_rejected'
    write_json(os.path.join(staging, 'status.json'), status)
    _publish(staging, out_root)
    return (0 if status['status'] == 'complete_validated' else 1), status


def cmd_smoke(args):
    rc, report, out, err = run_frozen_checker(args.episode_dir)
    sys.stdout.write(out)
    if err:
        sys.stderr.write(err)
    return rc


def cmd_verify(args):
    """Strict re-read of an episode dir + re-run the frozen checker."""
    report = {'artifact': 'Stage 1 collector verify (round-trip + smoke)'}
    required = ['raw_records.json', 'frames.json', 'frames.jsonl',
                'status_manifest.json', 'episode_provenance.json']
    missing = [f for f in required
               if not os.path.isfile(os.path.join(args.episode_dir, f))]
    if missing:
        report['status'] = 'FAIL'
        report['missing'] = missing
        print(json.dumps(report, indent=2))
        return 1
    strict = checker._strict_json_loads
    try:
        with open(os.path.join(args.episode_dir, 'frames.json'),
                  encoding='utf-8') as fh:
            frames = strict(fh.read())
        with open(os.path.join(args.episode_dir, 'episode_provenance.json'),
                  encoding='utf-8') as fh:
            prov = strict(fh.read())
    except Exception as exc:
        report['status'] = 'FAIL'
        report['error'] = 'strict read failed: %s: %s' % (
            type(exc).__name__, exc)
        print(json.dumps(report, indent=2))
        return 1
    report['frame_count_frames_json'] = len(frames) \
        if isinstance(frames, list) else None
    report['frame_count_provenance'] = prov.get('frame_count') \
        if isinstance(prov, dict) else None
    report['frame_count_consistent'] = (
        isinstance(frames, list) and isinstance(prov, dict)
        and prov.get('frame_count') == len(frames))
    rc, smoke, out, _err = run_frozen_checker(args.episode_dir)
    report['checker_exit_code'] = rc
    report['checker_ok'] = bool(smoke and smoke.get('smoke', {}).get('ok'))
    report['status'] = 'PASS' if (rc == 0 and report['frame_count_consistent']) \
        else 'FAIL'
    print(json.dumps(report, indent=2))
    return 0 if report['status'] == 'PASS' else 1


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Stage 1 native collector (minimal, contract-bound).')
    sub = parser.add_subparsers(dest='cmd', required=True)

    p = sub.add_parser('collect', help='subscribe to a running sim and write '
                                       'artifacts + run the frozen checker')
    p.add_argument('--episode-id', required=True)
    p.add_argument('--out-root', required=True)
    p.add_argument('--duration', type=float, default=15.0)
    p.add_argument('--warmup-sec', type=float, default=30.0)
    p.add_argument('--theta', required=True,
                   help='path to a theta dict JSON (or an entry with a '
                        '"theta" key)')
    p.add_argument('--seed', type=int, required=True)
    p.add_argument('--ring', default='ring0')
    p.add_argument('--domain-id', type=int, default=None)
    p.add_argument('--gz-partition', default=None)
    p.add_argument('--allow-overwrite', action='store_true')
    p.set_defaults(func=cmd_collect)

    p = sub.add_parser('verify', help='strict re-read + re-run the checker')
    p.add_argument('--episode-dir', required=True)
    p.set_defaults(func=cmd_verify)

    p = sub.add_parser('smoke', help='re-run the frozen checker on a dir')
    p.add_argument('--episode-dir', required=True)
    p.set_defaults(func=cmd_smoke)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
