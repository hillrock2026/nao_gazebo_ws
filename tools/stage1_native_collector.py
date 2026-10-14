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
CLOCK_TOPIC = '/clock'

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
    ('clock', 'ros', CLOCK_TOPIC),
)

# Association windows (RFC § 3.6.3 / contract § 5.2); causal last-not-after.
CONTACT_WINDOW_SEC = 0.05
POSE_WINDOW_SEC = 0.02
BODY_POSE_WINDOW_SEC = 0.02
BOOL_WINDOW_SEC = 0.05
COMMAND_WINDOW_SEC = 0.05
IMU_WINDOW_SEC = 0.01

# The configured windows are inclusive at their nominal boundary, but the binary
# representation of a decimal time difference can exceed the window by ~1e-16 at
# exactly that boundary (e.g. 1.05 - 1.0 > 0.05). Absorb only that representation
# error, and only in the two window comparisons below — never a general tolerance.
_WINDOW_BOUNDARY_EPS = 1e-9

G_PINNED = 9.81          # RFC § 3.7 kinematics formula constant (pinned)
FLOOR_Z = 0.0            # nao_world.sdf ground plane z (pinned constant)

EXPECTED_LINKS = list(probe.EXPECTED_LINKS)  # 25 links (RFC § 0.1)

# ---------------------------------------------------------------------------
# F-04 (OD-4=A / OD-5=A): the frozen Episode artifact set. `freeze_episode()`
# writes exactly these 12 files; the integrity verifier checks this FIXED set
# (not merely the hash-map entries that happen to exist). The hash map lives in
# the summary file `collector_meta.json` (OD-5=A — no separate manifest).
# ---------------------------------------------------------------------------
EPISODE_ARTIFACT_FILES = (
    'frames.jsonl',
    'frames.json',
    'raw_records.json',
    'status_manifest.json',
    'episode_provenance.json',
    'collector_meta.json',
    'raw/contacts_left.json',
    'raw/contacts_right.json',
    'raw/pose_info.json',
    'raw/joint_command.json',
    'raw/joint_states.json',
    'raw/body_pose.json',
)
SUMMARY_ARTIFACT = 'collector_meta.json'   # the summary; holds artifact_hashes
ARTIFACT_HASH_METHOD = (
    'sha256 over file bytes for every artifact; the summary itself '
    '(collector_meta.json) is hashed over the canonical json (sort_keys, '
    'compact separators) of its content with its own artifact_hashes entry and '
    'the summary_self_hash field removed (non-circular, F-04-2)')
REQUIRED_ARTIFACT_LIST = EPISODE_ARTIFACT_FILES   # OD-4=A fixes the R-04 gate

# Pre-implementation input gates (task v0.1 §4). Unset inputs MUST NOT be
# guessed: while unset the dependent success paths stay disabled / fail closed
# (no fabricated success, no guessed identity). THETA_ID_FIELD is now fixed to
# the frozen θ-list entry's `theta_id` (Owner decision O-04 + explicit binding
# authorization in STAGE1_THETA_ID_BINDING_READONLY_PREFLIGHT); SOURCE_MODULE_LIST
# and DATASET_ELIGIBILITY_POLICY remain unset.
THETA_ID_FIELD = 'theta_id'        # R-07 gate (θ-id field name, Owner-bound)
SOURCE_MODULE_LIST = None          # R-08 gate (source-hash module coverage)
DATASET_ELIGIBILITY_POLICY = None  # §4.9 gate (explicit eligibility policy)

# Independent pinned θ-list hash (F-03a). Source: the FROZEN Stage 0 evidence
# — `docs/STAGE0_CONTRACT_MANIFEST.md` and RFC §0.1 pin
# `dataset/stage0/stage0_theta_list.json` to this value; RFC §3.10 requires a
# fail-closed recompute before every episode. This is an INDEPENDENT anchor and
# is never taken from the θ-list file's own self-declared `theta_list_hash`.
THETA_LIST_FIXED_HASH = (
    '844646f55076f67647c9159a6c1f53018af89276154ff3a4ad3bdce28df95dbf')
# Stage 0 canonical method (RFC §0.1): SHA-256 over
# json.dumps(theta_list, indent=2, sort_keys=True, allow_nan=False), UTF-8,
# no trailing newline, over the `theta_list` entries array.
THETA_LIST_HASH_METHOD = ('sha256(json.dumps(theta_list, indent=2, '
                          'sort_keys=True, allow_nan=False), utf-8)')

# R-14 protected historical roots. These paths are NEVER opened/enumerated;
# they are only ever compared against resolved candidate paths.
PROTECTED_ROOTS = (
    os.path.join(WS, 'dataset', 'stage1', 'episodes', 'stage1_smoke0'),
    os.path.join(WS, 'dataset', 'stage1', 'episodes',
                 'stage1_smoke0.staging.4362'),
)


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


def _kv_lossless(v):
    """Lossless JSON-safe representation of a KeyValue scalar (R-13)."""
    if v is None or isinstance(v, (str, bool, int)):
        return v
    if isinstance(v, float):
        return v if math.isfinite(v) else None
    return str(v)


def _header_verbatim(header):
    """gz Header -> verbatim field-level object (R-13).

    Retains BOTH the `stamp` (sec/nsec) and the `data` repeated KeyValue list
    (order preserved) so no header field is silently dropped. Returns a JSON
    object (possibly empty) — never a non-object — so the frozen checker's
    "header must be an object" rule holds. This is field-level semantic
    equivalence, NOT raw protobuf byte preservation.
    """
    if header is None:
        return None
    out = {}
    stamp = getattr(header, 'stamp', None)
    if stamp is not None:
        sec = getattr(stamp, 'sec', None)
        if sec is None:
            sec = getattr(stamp, 'secs', None)
        nsec = getattr(stamp, 'nsec', None)
        if nsec is None:
            nsec = getattr(stamp, 'nanosec', None)
        if sec is not None:
            out['stamp'] = {'sec': int(sec), 'nsec': int(nsec or 0)}
    data = getattr(header, 'data', None)
    if data is not None:
        try:
            items = list(data)
        except TypeError:
            items = []
        out['data'] = [{'key': _kv_lossless(getattr(kv, 'key', None)),
                        'value': _kv_lossless(getattr(kv, 'value', None))}
                       for kv in items]
    return out


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
    """std_msgs/Bool carries NO header/source stamp (RFC § 3.6.2); the only
    time available is the simulation `receipt` time, attached by the callback
    as `_recv_sim_s` (never a source stamp)."""
    v = _field(msg, 'data')
    return {'stamp_s': None, 'value': bool(v) if v is not None else None}


def build_clock_record(msg):
    """rosgraph_msgs/Clock -> {'stamp_s': sim seconds} (the sim clock)."""
    c = _field(msg, 'clock')
    sec = getattr(c, 'sec', None) if c is not None else None
    if sec is None and c is not None:
        sec = getattr(c, 'secs', None)
    nsec = getattr(c, 'nanosec', None) if c is not None else None
    if nsec is None and c is not None:
        nsec = getattr(c, 'nsec', None)
    if sec is None:
        return {'stamp_s': None}
    return {'stamp_s': float(sec) + float(nsec or 0) * 1e-9}


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
    'clock': build_clock_record,
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


def symmetric_nearest_index(stamps, t, window):
    """Index of the symmetric-nearest sample: min |stamp - t| with
    |stamp - t| <= window (inclusive), tie -> the EARLIER stamp (R-06).

    `stamps` must be sorted ascending with no None. Returns None if no
    candidate is inside the window."""
    best = None
    best_d = None
    for idx, s in enumerate(stamps):
        d = abs(s - t)
        if d > window + _WINDOW_BOUNDARY_EPS:
            continue
        if best_d is None or d < best_d:
            best, best_d = idx, d
        # equal distance: keep the earlier (already stored, ascending order)
    return best


def _monotonic_epochs(stamps):
    """Assign a clock epoch to a time-ordered sequence: the epoch increments
    whenever sim time decreases (a clock rewind). None -> None (R-05/R-06)."""
    epoch = 0
    prev = None
    out = []
    for s in stamps:
        if s is None:
            out.append(None)
            continue
        if prev is not None and s < prev:
            epoch += 1
        out.append(epoch)
        prev = s
    return out


# ---------------------------------------------------------------------------
# R-14: historical-path write protection (path semantics, not string compare).
# The protected roots are NEVER accessed — only resolved candidate paths are
# compared against them. Any resolution failure/ambiguity fails closed.
# ---------------------------------------------------------------------------

def _resolve_for_protection(path):
    """Resolve `path` to a canonical absolute form for comparison, without
    touching protected directories. The longest existing ancestor is realpath'd
    and the remaining (not-yet-existing) components appended lexically."""
    p = os.path.abspath(path)
    head = p
    suffix = []
    while head and not os.path.exists(head):
        parent, tail = os.path.split(head)
        if tail == '' or parent == head:
            break
        suffix.insert(0, tail)
        head = parent
    real = os.path.realpath(head) if head and os.path.exists(head) \
        else os.path.realpath(p)
    for comp in suffix:
        real = os.path.join(real, comp)
    return os.path.normpath(real)


def is_protected_path(path):
    """True if `path` equals, is inside, or is an ancestor of a protected
    root (canonical/resolved comparison; symlinks resolved). (R-14)

    The protected roots are compared in their lexical absolute form so that
    no filesystem access (stat/enumerate) is performed on them."""
    try:
        rp = _resolve_for_protection(path)
    except Exception:  # noqa: BLE001 — unresolvable ⇒ fail closed
        return True
    for root in PROTECTED_ROOTS:
        rr = os.path.normpath(os.path.abspath(root))
        if rp == rr or rp.startswith(rr + os.sep):
            return True            # candidate is (inside) the protected root
        if rr.startswith(rp + os.sep):
            return True            # candidate is an ancestor of a protected root
    return False


def guard_path(path):
    """Fail closed (raise) if operating on `path` could affect a protected
    historical root (R-14)."""
    if is_protected_path(path):
        raise PermissionError(
            'refusing to operate on a protected historical path (R-14): %s'
            % path)
    return path


# ---------------------------------------------------------------------------
# R-05 / R-06 / R-10: association with explicit UNKNOWN reason codes.
# ---------------------------------------------------------------------------

def _bool_value(r):
    v = r.get('value')
    return None if v is None else bool(v)


def assoc_contact_bool(recs, t, frame_epoch):
    """Associate a headerless Bool to a frame at sim time `t` (R-05).

    Returns (value, recv_sim_s, state, reason). `state` is 'matched' or
    'unknown'; `reason` is a single UNKNOWN token (or None). Never coerces to
    false: unknown stays unknown with a reason."""
    if not recs:
        return None, None, 'unknown', 'no_message'
    recv_recs = [r for r in recs if r.get('_recv_sim_s') is not None]
    no_recv = [r for r in recs if r.get('_recv_sim_s') is None]
    if not recv_recs:
        # no usable receipt time: distinguish a clock that became invalid from
        # one never seen (F-01a) — never borrow a stale cached value
        if any(r.get('_recv_clock_state') == 'invalid' for r in no_recv):
            return None, None, 'unknown', 'clock_invalid'
        return None, None, 'unknown', 'clock_missing'
    if no_recv:
        # some Bool records carry a receipt clock while others do not; if the
        # remainder are invalid-clock receipts the reason is the clock (F-01a),
        # otherwise the receipt order cannot be reconciled (R-05 ambiguity)
        if any(r.get('_recv_clock_state') == 'invalid' for r in no_recv):
            return None, None, 'unknown', 'clock_invalid'
        return None, None, 'unknown', 'order_ambiguous'
    same = [r for r in recv_recs if r.get('_recv_epoch', 0) == frame_epoch]
    if not same:
        return None, None, 'unknown', 'clock_rewind'
    past = [r for r in same if r['_recv_sim_s'] <= t]
    if not past:
        if any(r['_recv_sim_s'] > t for r in same):
            return None, None, 'unknown', 'future'
        return None, None, 'unknown', 'no_message'
    t_sel = max(r['_recv_sim_s'] for r in past)
    sel = [r for r in past if r['_recv_sim_s'] == t_sel]
    values = {_bool_value(r) for r in sel}
    if len(values) > 1:
        return None, t_sel, 'unknown', 'duplicate_conflict'
    value = _bool_value(sel[-1])
    if (t - t_sel) > BOOL_WINDOW_SEC + _WINDOW_BOUNDARY_EPS:
        return None, t_sel, 'unknown', 'stale'
    if value is None:
        return None, t_sel, 'unknown', 'no_message'
    return value, t_sel, 'matched', None


def _imu_content(rec):
    """Hashable content signature of an IMU sample (measured fields only)."""
    return (tuple(rec.get('orientation') or ()),
            tuple(rec.get('linear_acceleration') or ()),
            tuple(rec.get('angular_velocity') or ()))


def assoc_imu(imu_all, imu, stamps, imu_epochs, t, frame_epoch, t_observed_max):
    """Symmetric-nearest IMU association with maturity + epoch rules (R-06).

    Returns (index_into_imu, match_state, reason, delta_s). `match_state` is
    'matched', 'pending' (immature/end) or 'unknown'."""
    if not imu_all:
        return None, 'unknown', 'no_candidate', None
    if not stamps:
        return None, 'unknown', 'source_stamp_missing', None
    valid = [j for j in range(len(stamps))
             if imu_epochs[j] == frame_epoch and math.isfinite(stamps[j])]
    if not valid:
        if any(not math.isfinite(s) for s in stamps):
            return None, 'unknown', 'source_stamp_invalid', None
        return None, 'unknown', 'clock_rewind', None
    if t_observed_max is not None and (t + IMU_WINDOW_SEC) > t_observed_max:
        # the future half of the symmetric window has not materialised yet —
        # do not finalise early (R-06 immature window / end of collection)
        return None, 'pending', 'immature_window', None
    window = [stamps[j] for j in valid]
    k = symmetric_nearest_index(window, t, IMU_WINDOW_SEC)
    if k is None:
        return None, 'unknown', 'no_candidate', None
    j = valid[k]
    # F-02: an identical source timestamp whose samples CONFLICT in content
    # must not be resolved by an arbitrary pick — report ambiguity, never
    # 'matched'. Identical content may use the deterministic pick.
    sel_stamp = stamps[j]
    same = [x for x in valid if stamps[x] == sel_stamp]
    if len({_imu_content(imu[x]) for x in same}) > 1:
        return None, 'unknown', 'duplicate_ambiguous', None
    return j, 'matched', None, (stamps[j] - t)


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
                 joint_order=None, requested_config=None):
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
        self.requested_config = requested_config
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

        imu_all = records.get('imu', [])
        frame_epochs = _monotonic_epochs([r['stamp_s'] for r in js])
        _js_times = [x for x in (r['stamp_s'] for r in js) if x is not None]
        t_observed_max = max(_js_times) if _js_times else None
        imu_epochs = _monotonic_epochs(imu_stamps)

        frames = []
        for i, st in enumerate(js):
            t = st['stamp_s']
            if list(st.get('name', [])) != self.joint_order:
                self.notes.append('frame %d: joint_states name != JOINT_ORDER'
                                  % i)
            p_idx = causal_index(pose_stamps, t, POSE_WINDOW_SEC)
            b_idx = causal_index(body_stamps, t, BODY_POSE_WINDOW_SEC)
            c_idx = causal_index(cmd_stamps, t, COMMAND_WINDOW_SEC)
            l_idx = causal_index(lcl_stamps, t, CONTACT_WINDOW_SEC)
            r_idx = causal_index(rcl_stamps, t, CONTACT_WINDOW_SEC)
            i_idx, i_state, i_reason, i_delta = assoc_imu(
                imu_all, imu, imu_stamps, imu_epochs, t, frame_epochs[i],
                t_observed_max)
            l_val, l_recv, l_state, l_reason = assoc_contact_bool(
                bools['left_contact_bool'], t, frame_epochs[i])
            r_val, r_recv, r_state, r_reason = assoc_contact_bool(
                bools['right_contact_bool'], t, frame_epochs[i])
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
                # header stamp) -> associated by SIM RECEIPT TIME with an
                # inclusive freshness bound; unknown/stale/future/rewind stay
                # UNKNOWN with a reason (R-05, never coerced to false)
                'left_contact': l_val,
                'right_contact': r_val,
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
                # non-normative association evidence (R-05/R-06/R-10): which
                # sample was used (or explicit unknown) and why
                'imu_source_stamp': imu[i_idx]['stamp_s']
                if i_idx is not None else None,
                'imu_delta_s': i_delta,
                'imu_match_state': i_state,
                'imu_unknown_reason': i_reason,
                'left_contact_recv_sim_s': l_recv,
                'left_contact_assoc_state': l_state,
                'left_contact_unknown_reason': l_reason,
                'right_contact_recv_sim_s': r_recv,
                'right_contact_assoc_state': r_state,
                'right_contact_unknown_reason': r_reason,
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
            # R-07: requested vs effective configuration kept strictly separate
            'requested_config': self.requested_config,
            'effective_config': {
                'state': 'unverified', 'value': None,
                'reason': 'no episode-linked producer/runtime evidence '
                          '(R-07 gate)'},
            # R-08: bounded source binding (coverage is a pre-implementation
            # input; never claimed as complete while the module list is unset)
            'source_integrity': source_integrity_provenance(),
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


_HEX = '0123456789abcdef'


def _valid_sha256(v):
    return (isinstance(v, str) and len(v) == 64
            and all(c in _HEX for c in v))


def _canonical_json_bytes(obj):
    """Deterministic canonical serialization used for the summary self-hash:
    sorted keys, compact separators, no NaN/Inf. Produced and consumed by the
    same rule (F-04-2)."""
    return json.dumps(obj, sort_keys=True, separators=(',', ':'),
                      allow_nan=False).encode('utf-8')


def compute_summary_self_hash(meta):
    """F-04-2: deterministic, non-circular hash of the summary file.

    Rule: sha256 over the canonical json of the summary object with its own
    `summary_self_hash` field removed AND its own entry in `artifact_hashes`
    removed. Mutating any other field (or any other file's declared hash)
    changes this value; mutating only the hash field is caught by comparing
    the stored `summary_self_hash` against the recomputation."""
    clone = {k: v for k, v in (meta or {}).items() if k != 'summary_self_hash'}
    ah = dict(clone.get('artifact_hashes') or {})
    ah.pop(SUMMARY_ARTIFACT, None)
    clone['artifact_hashes'] = ah
    return hashlib.sha256(_canonical_json_bytes(clone)).hexdigest()


def _safe_artifact_path(root, rel):
    """Resolve `rel` under `root`; return the joined path or None if unsafe
    (absolute, escaping, or symlink-escaping). Never reads the target file."""
    if not isinstance(rel, str) or not rel or os.path.isabs(rel):
        return None
    norm = os.path.normpath(rel)
    if norm in ('.', '') or norm.startswith('..' + os.sep) or norm == '..':
        return None
    full = os.path.join(root, norm)
    real_root = os.path.realpath(root)
    real_full = os.path.realpath(full)   # resolves symlinks (incl. non-existent tail)
    if real_full != real_root and not real_full.startswith(real_root + os.sep):
        return None
    return full


def source_integrity_provenance():
    """R-08: bounded, non-normative source-hash provenance.

    Always binds the executing collector file itself. Module coverage comes
    ONLY from an explicitly fixed, finite module list; while that list is unset
    (pre-implementation input) the coverage is reported 'unverified' and full
    coverage is never claimed. This is not source authenticity."""
    cf = os.path.abspath(__file__)
    files = {os.path.relpath(cf, WS): sha256_file(cf)}
    coverage = 'unverified'
    if SOURCE_MODULE_LIST is not None:
        for rel in SOURCE_MODULE_LIST:
            p = os.path.join(WS, rel)
            if os.path.isfile(p):
                files[rel] = sha256_file(p)
        coverage = 'listed-only'
    return {'algorithm': 'sha256', 'coverage': coverage,
            'files': dict(sorted(files.items())),
            'note': 'covers only the enumerated specified files; not all '
                    'executed code / dependencies / runtime; not source '
                    'authenticity'}


def _backup_path(out_root):
    """Deterministic backup name (no undefined/random overwrite) (R-02 S6)."""
    return out_root + '.replaced.%d' % os.getpid()


_os_replace = os.replace   # test seam for the publish rename (R-02)


def _publish(staging, out_root):
    """Publish `staging` into `out_root` (R-02 state machine).

    Never deletes/overwrites the last copy of existing data. Returns a result
    dict: {'ok': bool, 'status': str, 'backup': path|None, 'restored': bool|None,
    'detail': str|None}. 'ok' is True only when the staging dir was moved into
    a target and no old data was lost. Failures are explicit, never success.
    """
    result = {'ok': False, 'status': 'publish_failed', 'backup': None,
              'restored': None, 'detail': None}
    guard_path(out_root)
    guard_path(staging)
    if os.path.isdir(out_root):
        backup = _backup_path(out_root)
        guard_path(backup)
        if os.path.exists(backup):
            # S6: backup name conflict — fail closed, do not touch the target
            result['detail'] = 'backup_name_conflict: %s' % backup
            return result
        try:
            _os_replace(out_root, backup)          # S3: target -> backup
        except OSError as exc:
            result['detail'] = 'backup_move_failed: %s' % exc
            return result                          # target left untouched
        result['backup'] = backup
        try:
            _os_replace(staging, out_root)          # S1: staging -> target
        except OSError as exc:
            # S4/S5: attempt restore; never delete the only old copy
            try:
                _os_replace(backup, out_root)
                result['restored'] = True
                result['detail'] = 'publish_move_failed_restored: %s' % exc
            except OSError as exc2:
                result['restored'] = False
                result['detail'] = ('publish_move_failed_restore_failed: '
                                    '%s / %s' % (exc, exc2))
                # backup is retained, recognizable and recoverable (R-02 S5)
            return result
        # success: the old data is retained as a recoverable backup (a cleanup
        # step is separate and must never delete the only copy — R-02 S9)
        result['ok'] = True
        result['status'] = 'published'
        return result
    try:
        _os_replace(staging, out_root)              # S1 (no old target)
    except OSError as exc:
        result['detail'] = 'publish_move_failed_no_old: %s' % exc
        return result                              # S2: staging retained
    result['ok'] = True
    result['status'] = 'published'
    return result


def freeze_episode(assembler, records, staging):
    """Write all artifacts into `staging` (an existing empty dir).

    F-04-1: record the SHA-256 of all 12 frozen episode files in the summary's
    `artifact_hashes` (OD-4=A / OD-5=A). The summary file itself
    (`collector_meta.json`) is hashed with the non-circular rule in F-04-2."""
    os.makedirs(os.path.join(staging, 'raw'), exist_ok=True)
    raw = assembler.raw_records(records)
    frames = assembler.build_frames(records)
    frame_count = len(frames)
    t_start = frames[0]['sim_time_sec'] if frames else None
    t_end = frames[-1]['sim_time_sec'] if frames else None

    # 11 non-summary files first (write order must not drop any required file)
    write_jsonl(os.path.join(staging, 'frames.jsonl'), frames)
    integrity = sha256_file(os.path.join(staging, 'frames.jsonl'))
    write_json(os.path.join(staging, 'frames.json'), frames)
    write_json(os.path.join(staging, 'raw_records.json'), raw)
    write_json(os.path.join(staging, 'status_manifest.json'),
               build_status_manifest())
    write_json(os.path.join(staging, 'episode_provenance.json'),
               assembler.provenance(frame_count, t_start, t_end, integrity))
    for kind in ('contacts_left', 'contacts_right', 'pose_info',
                 'joint_command', 'joint_states', 'body_pose'):
        write_json(os.path.join(staging, 'raw', '%s.json' % kind),
                   records.get(kind, []))

    # F-04-1: hash every required file except the summary; fail-closed if any
    # required file could not be written.
    hashes = {}
    for rel in EPISODE_ARTIFACT_FILES:
        if rel == SUMMARY_ARTIFACT:
            continue
        p = os.path.join(staging, rel)
        if not os.path.isfile(p):
            raise RuntimeError(
                'required artifact not written at freeze: %s (F-04)' % rel)
        hashes[rel] = sha256_file(p)

    # F-04-2: build the summary, then add its own non-circular self-hash.
    meta = assembler.meta(records)
    meta['artifact_hash_method'] = ARTIFACT_HASH_METHOD
    meta['artifact_hashes'] = dict(hashes)
    self_hash = compute_summary_self_hash(meta)   # over content without the self entry
    meta['summary_self_hash'] = self_hash
    meta['artifact_hashes'][SUMMARY_ARTIFACT] = self_hash
    write_json(os.path.join(staging, SUMMARY_ARTIFACT), meta)

    return {'frame_count': frame_count, 'n_raw': len(raw),
            't_start': t_start, 't_end': t_end,
            'episode_integrity_hash': integrity,
            'artifact_hashes': dict(meta['artifact_hashes'])}


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

class ClockState:
    """Latest simulation clock value + validity + epoch.

    The clock is the ONLY sim-time source for headerless Bools (RFC § 3.6.2).
    A rewind starts a new epoch; a clock message that carries no valid sim time
    marks the CURRENT clock invalid so a new receipt never borrows a stale
    cached value (F-01a)."""

    def __init__(self):
        self.sim_s = None       # last known value (cache; NOT usable when invalid)
        self.epoch = 0
        self.valid = False

    def update(self, sim_s):
        if sim_s is None:
            # current clock invalid/missing — retain the cache for reference,
            # but never treat it as a usable receipt time
            self.valid = False
            return
        if self.sim_s is not None and sim_s < self.sim_s:
            self.epoch += 1
        self.sim_s = sim_s
        self.valid = True

    def receipt_sim_s(self):
        """Sim time usable for a NEW receipt event, or None when the current
        clock is invalid/missing (never borrows a stale cached value)."""
        return self.sim_s if self.valid else None

    def state(self):
        """'valid' | 'invalid' (had a clock, now unusable) | 'missing'."""
        if self.valid:
            return 'valid'
        return 'missing' if self.sim_s is None else 'invalid'


def _make_callback(kind, store, errors, seq, clock=None):
    builder = BUILDERS[kind]

    def cb(msg):
        try:
            rec = builder(msg)
            if kind == 'clock':
                if clock is not None:
                    clock.update(rec.get('stamp_s'))
            elif kind in ('left_contact_bool', 'right_contact_bool'):
                # receipt simulation time (NOT a source stamp) + clock epoch;
                # an invalid/missing current clock yields NO receipt time
                # (never a stale cached value) — F-01a
                rec['_recv_sim_s'] = (clock.receipt_sim_s()
                                      if clock is not None else None)
                rec['_recv_epoch'] = clock.epoch if clock is not None else 0
                rec['_recv_clock_state'] = (clock.state()
                                            if clock is not None else 'missing')
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
    clock = ClockState()
    missing = []
    try:
        for kind, tr, topic in spec:
            store = probe.LockedRecords()
            cb = _make_callback(kind, store, errors, seq, clock=clock)
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
    from rosgraph_msgs.msg import Clock
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
        'clock': Clock,
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


def compute_theta_list_hash(data):
    """Recompute the frozen θ-list hash over its `theta_list` entries array
    using the Stage 0 canonical method (RFC §0.1/§3.10), independent of any
    value declared inside the file."""
    entries = (data or {}).get('theta_list')
    if not isinstance(entries, list):
        raise RuntimeError('θ list has no `theta_list` array — fail closed')
    canonical = json.dumps(entries, indent=2, sort_keys=True, allow_nan=False)
    return hashlib.sha256(canonical.encode('utf-8')).hexdigest()


def verify_theta_list(data):
    """F-03a: independently verify the θ list BEFORE any acquisition side
    effect. Recompute the hash from the actual file content and compare it to
    the FROZEN anchor `THETA_LIST_FIXED_HASH` — never to the file's own
    self-declared `theta_list_hash`. Fail-closed on mismatch/missing/invalid.

    Returns a small evidence dict; raises RuntimeError on any failure."""
    if THETA_LIST_FIXED_HASH is None:
        raise RuntimeError('θ-list fixed hash not configured — fail closed')
    computed = compute_theta_list_hash(data)
    if computed != THETA_LIST_FIXED_HASH:
        raise RuntimeError(
            'θ-list tamper check FAILED: recomputed %s != pinned %s '
            '(fail-closed)' % (computed[:16], THETA_LIST_FIXED_HASH[:16]))
    declared = (data or {}).get('theta_list_hash')
    return {'method': THETA_LIST_HASH_METHOD, 'computed': computed,
            'pinned': THETA_LIST_FIXED_HASH, 'declared': declared,
            'declared_matches': declared == computed}


def resolve_theta(theta_json_path=None, theta_id_hint=None, data=None):
    """Load a θ dict; fail-closed cross-check membership in the frozen list.

    `data` may be a pre-loaded θ list (avoids a duplicate read); when None the
    list is loaded here. Returns (theta, list_hash, matched_entry, info). The
    matched entry is returned so the caller can derive the θ identity — but the
    identity FIELD NAME is a pre-implementation input (R-07) and is not guessed.
    """
    if data is None:
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
    if len(match) > 1:
        raise RuntimeError('theta_hash %s matches %d frozen entries — '
                           'fail-closed (R-07)' % (h[:16], len(match)))
    return theta, list_hash, match[0], {'computed': h}


def theta_id_from_entry(entry):
    """θ identity from the matched frozen entry (R-07).

    GATED: the unique `theta_id` field name is a pre-implementation input and
    is NOT guessed. While unset, this fails closed so no run can produce an
    unverified θ binding."""
    if THETA_ID_FIELD is None:
        raise RuntimeError('theta_id field name not confirmed (R-07 gate) — '
                           'fail closed')
    tid = (entry or {}).get(THETA_ID_FIELD)
    if not isinstance(tid, str) or not tid:
        raise RuntimeError('matched theta entry has no usable %r (R-07)'
                           % THETA_ID_FIELD)
    return tid


def prepare_collection_context(theta_json_path, data=None):
    """F-03a ordering: independently verify the θ list and resolve the θ
    identity BEFORE any acquisition side effect. Returns
    (theta, theta_id, theta_list_hash, theta_info); raises (fail-closed) on any
    hash / membership / identity failure so the caller must invoke this before
    starting collection."""
    if data is None:
        data = load_theta_list()
    check = verify_theta_list(data)                 # F-03a independent anchor
    theta, _declared, entry, theta_info = resolve_theta(theta_json_path,
                                                        data=data)
    theta_id = theta_id_from_entry(entry)           # R-07 gate (fail closed)
    return theta, theta_id, check['computed'], theta_info


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
    # F-03a / R-07: independently verify the θ list and resolve the θ identity
    # BEFORE any acquisition side-effect (no episode dir, no subscription, no
    # rclpy). Fail-closed on any hash / membership / identity failure.
    try:
        theta, theta_id, theta_list_hash, theta_info = \
            prepare_collection_context(args.theta)
    except RuntimeError as exc:
        status = {'artifact': 'Stage 1 collector status (NON-NORMATIVE)',
                  'episode_id': args.episode_id, 'phase': 'aborted',
                  'terminal_status': 'aborted',
                  'terminal_reason': 'theta_binding_failed',
                  'process_exit_result': 'failure',
                  'dataset_eligibility': 'ineligible', 'status': 'aborted',
                  'detail': str(exc)}
        print(json.dumps(status, indent=2))
        return 1

    from_ros = args.domain_id
    if from_ros is not None:
        os.environ['ROS_DOMAIN_ID'] = str(from_ros)
    if args.gz_partition:
        os.environ['GZ_PARTITION'] = args.gz_partition
    import rclpy
    rclpy.init()
    aborted = False
    try:
        transport = probe.gz_packages()[2]
        msg_types = _message_types()
        records, info = collect_live(
            list(STREAMS) + list(FRAME_AUX_STREAMS), args.duration,
            args.warmup_sec, transport, msg_types)
    except KeyboardInterrupt:
        aborted = True
        records, info = {}, {}
    finally:
        rclpy.shutdown()
    info['aborted'] = aborted
    if aborted:
        sys.stderr.write('collector interrupted — aborting without publish\n')

    inertials, sdf_sha = load_sdf_inertials()
    total_mass = sum(v['mass'] for v in inertials.values())
    requested_config = {
        'theta': theta, 'theta_id': theta_id, 'seed': args.seed,
        'ring': args.ring, 'duration_sec': args.duration,
        'warmup_sec': args.warmup_sec,
        'association_windows_sec': {
            'contact': CONTACT_WINDOW_SEC, 'pose': POSE_WINDOW_SEC,
            'body_pose': BODY_POSE_WINDOW_SEC, 'bool': BOOL_WINDOW_SEC,
            'command': COMMAND_WINDOW_SEC, 'imu': IMU_WINDOW_SEC},
        'theta_hash': theta_info.get('computed'),
    }
    assembler = EpisodeAssembler(
        episode_id=args.episode_id, theta=theta, theta_id=theta_id,
        seed=args.seed, ring=args.ring,
        config_hash_source=config_hash_source(),
        wallclock_utc=datetime.now(timezone.utc).isoformat(),
        git_commit=_git_commit(), rfc_hash=sha256_file(RFC_PATH),
        model_hash=sdf_sha, world_hash=sha256_file(WORLD_PATH),
        gz_version=_gz_version(), theta_list_hash=theta_list_hash,
        inertials=inertials, total_mass=total_mass, sdf_sha=sdf_sha,
        requested_config=requested_config)
    try:
        rc, status = _finalize(args, assembler, records, info)
    except PermissionError as exc:      # R-14 protected path — fail closed
        status = {'artifact': 'Stage 1 collector status (NON-NORMATIVE)',
                  'episode_id': args.episode_id, 'phase': 'aborted',
                  'terminal_status': 'aborted', 'terminal_reason': 'aborted',
                  'process_exit_result': 'failure',
                  'dataset_eligibility': 'ineligible', 'status': 'aborted',
                  'detail': str(exc)}
        rc = 1
    print(json.dumps(status, indent=2))
    return rc


# ---------------------------------------------------------------------------
# R-01 / R-09: six-dimension terminal status model (task v0.3 §4).
# ---------------------------------------------------------------------------

TERMINAL_PRECEDENCE = ('aborted', 'write_failed', 'status_write_failed',
                       'publish_failed', 'collection_failed',
                       'integrity_failed', 'contract_rejected', 'partial',
                       'complete_validated')

REASON_FOR_TERMINAL = {
    'aborted': 'aborted',
    'write_failed': 'write_failed',
    'status_write_failed': 'status_write_failed',
    'publish_failed': 'publish_failed',
    'collection_failed': 'collection_failed',
    'integrity_failed': 'integrity_failed',
    'contract_rejected': 'checker_failed',
    'partial': 'partial',
    'complete_validated': 'complete_validated',
}

REQUIRED_RAW_CATEGORIES = ('contacts_left', 'contacts_right', 'pose_info',
                           'joint_command')
FRAME_CLOCK_CATEGORY = 'joint_states'


def classify_contract(rc, report):
    """(state, reason) for contract_validation (§4.7). A bare exit code 0 is
    NEVER sufficient — the report must exist, parse, and report smoke ok."""
    if report is None:
        return 'error', 'checker_error'
    if not isinstance(report, dict):
        return 'error', 'checker_error'
    ok = bool(report.get('smoke', {}).get('ok'))
    if rc == 0 and ok:
        return 'passed', None
    if rc == 0 and not ok:
        return 'error', 'checker_error'   # zero exit but not passing
    return 'failed', 'checker_failed'


def verify_integrity(root, summary=None):
    """(state, reason, detail) for integrity_validation (R-04 / F-04).

    F-04-3: check the FIXED required set `EPISODE_ARTIFACT_FILES`, not merely
    the entries that happen to be declared. The hash map (`artifact_hashes`) is
    read from the persisted summary file (OD-5=A). Fail-closed on: missing
    summary, missing/empty/ill-typed hash map, undeclared or extra entries,
    missing files, invalid hash values, unreadable files, hash mismatch,
    unsafe/escaping paths, and summary self-hash inconsistency."""
    if REQUIRED_ARTIFACT_LIST is None:
        return ('not_run', 'integrity_not_run',
                'required artifact list not fixed (R-04 gate)')
    if not isinstance(root, str) or not root:
        return ('failed', 'integrity_failed', 'invalid episode root')
    root = os.path.abspath(root)

    # The hash map lives in the summary file itself (OD-5=A).
    summary_path = os.path.join(root, SUMMARY_ARTIFACT)
    if not os.path.isfile(summary_path):
        return ('failed', 'integrity_failed',
                'summary %s missing (F-04)' % SUMMARY_ARTIFACT)
    try:
        with open(summary_path, 'r', encoding='utf-8') as fh:
            meta = json.load(fh)
    except Exception as exc:  # noqa: BLE001
        return ('failed', 'integrity_failed',
                'summary unreadable: %s: %s' % (type(exc).__name__, exc))
    if not isinstance(meta, dict):
        return ('failed', 'integrity_failed', 'summary is not a JSON object')
    ah = meta.get('artifact_hashes')
    if ah is None:
        return ('failed', 'integrity_failed', 'artifact_hashes missing')
    if not isinstance(ah, dict):
        return ('failed', 'integrity_failed', 'artifact_hashes not an object')
    if not ah:
        return ('failed', 'integrity_failed', 'artifact_hashes empty')

    required = tuple(EPISODE_ARTIFACT_FILES)
    allowed = set(required)
    extra = sorted(k for k in ah if k not in allowed)
    if extra:
        return ('failed', 'integrity_failed',
                'unauthorized artifact_hashes entries: %s' % extra)
    undeclared = sorted(allowed - set(ah))
    if undeclared:
        return ('failed', 'integrity_failed',
                'required files without a hash declaration: %s' % undeclared)

    for rel in required:
        if not _valid_sha256(ah.get(rel)):
            return ('failed', 'integrity_failed',
                    'invalid hash declaration for %s' % rel)
        full = _safe_artifact_path(root, rel)
        if full is None:
            return ('failed', 'integrity_failed',
                    'unsafe artifact path (escapes episode root): %s' % rel)
        if not os.path.isfile(full):
            return ('failed', 'integrity_failed',
                    'required artifact missing: %s' % rel)
        if rel == SUMMARY_ARTIFACT:
            # F-04-2: the summary is verified by the canonical self-hash rule.
            try:
                recomputed = compute_summary_self_hash(meta)
            except Exception as exc:  # noqa: BLE001 — non-finite/ill-typed
                # content makes the canonical (allow_nan=False) serialization
                # un-computable; fail closed like the ordinary hash path above
                # instead of letting the exception escape the verifier.
                return ('failed', 'integrity_failed',
                        'summary self-hash computation failed: %s: %s'
                        % (type(exc).__name__, exc))
            if meta.get('summary_self_hash') != recomputed \
                    or ah.get(SUMMARY_ARTIFACT) != recomputed:
                return ('failed', 'integrity_failed',
                        'summary self-hash mismatch: %s' % SUMMARY_ARTIFACT)
            continue
        try:
            got = sha256_file(full)
        except Exception as exc:  # noqa: BLE001
            return ('failed', 'integrity_failed',
                    'unreadable artifact %s: %s' % (rel, exc))
        if got != ah.get(rel):
            return ('failed', 'integrity_failed', 'hash mismatch: %s' % rel)
    return ('passed', None, None)


def _stream_ok(kind, records, streams):
    """A required stream counts as acquired only if it has records AND (when
    subscribe info is known) subscribe_ok is True (R-09; F1)."""
    if not records.get(kind):
        return False
    info = streams.get(kind)
    if info is not None and info.get('subscribe_ok') is not True:
        return False
    return True


def acquisition_completeness(records, streams, cb_errors):
    missing = [k for k in REQUIRED_RAW_CATEGORIES
               if not _stream_ok(k, records, streams)]
    if not _stream_ok(FRAME_CLOCK_CATEGORY, records, streams):
        missing.append(FRAME_CLOCK_CATEGORY)
    if missing:
        return 'missing_required', missing
    if cb_errors:
        return 'partial', []
    return 'complete', []


def classify_terminal(acq, contract_state, integrity_state, cb_errors,
                      sess_errors, aborted=False):
    """Unique primary terminal status by fixed precedence (§4.3/§4.4)."""
    conditions = {
        'aborted': bool(aborted),
        'write_failed': False,            # handled by the freeze path
        'status_write_failed': False,     # handled post-publish
        'publish_failed': False,          # handled post-publish
        'collection_failed': bool(sess_errors) or acq == 'missing_required',
        'integrity_failed': integrity_state != 'passed',
        'contract_rejected': contract_state != 'passed',
        'partial': bool(cb_errors),
        'complete_validated': True,
    }
    for st in TERMINAL_PRECEDENCE:
        if conditions.get(st):
            return st
    return 'complete_validated'


def build_status_doc(base, summary, acquisition, contract, integrity,
                     terminal, reason=None, phase='staging', detail=None,
                     publish=None):
    # `contract` is (state, reason); `integrity` is (state, reason[, detail])
    contract_state, contract_reason = contract[0], contract[1]
    integrity_state, integrity_reason = integrity[0], integrity[1]
    if terminal == 'integrity_failed':
        reason = integrity_reason or 'integrity_failed'
    elif terminal == 'contract_rejected':
        reason = contract_reason or 'checker_failed'
    elif reason is None:
        reason = REASON_FOR_TERMINAL.get(terminal)
    exit_result = 'success' if terminal == 'complete_validated' else 'failure'
    if terminal == 'complete_validated' and integrity_state == 'passed':
        eligibility = ('eligible' if DATASET_ELIGIBILITY_POLICY is not None
                       else 'undetermined')
    else:
        eligibility = 'ineligible'
    doc = dict(base)
    doc.update({
        'phase': phase,
        'acquisition_completeness': acquisition,
        'contract_validation': contract_state,
        'integrity_validation': integrity_state,
        'terminal_status': terminal,
        'terminal_reason': reason,
        'process_exit_result': exit_result,
        'dataset_eligibility': eligibility,
        'status': terminal,   # legacy alias of terminal_status
    })
    if summary:
        for k in ('frame_count', 'n_raw', 't_start', 't_end',
                  'episode_integrity_hash'):
            if k in summary:
                doc[k] = summary[k]
    if detail is not None:
        doc['detail'] = detail
    if publish is not None:
        doc['publish_status'] = publish
    return doc


def _finalize(args, assembler, records, info):
    out_root = os.path.abspath(args.out_root)
    guard_path(out_root)                       # R-14 fail-closed protection
    if os.path.exists(out_root) and not args.allow_overwrite:
        raise RuntimeError('out_root exists (use --allow-overwrite): %s'
                           % out_root)
    staging = out_root + '.staging.%d' % os.getpid()
    guard_path(staging)
    if os.path.exists(staging):
        shutil.rmtree(staging)
    os.makedirs(staging)

    streams = info.get('streams', {}) or {}
    cb_errors = list(info.get('callback_errors', []) or [])
    sess_errors = list(info.get('session_errors', []) or [])
    base = {'artifact': 'Stage 1 collector status (NON-NORMATIVE)',
            'episode_id': assembler.episode_id,
            'phase': 'staging',
            'streams': streams,
            'callback_errors': cb_errors,
            'session_errors': sess_errors}

    # --- freeze --------------------------------------------------------
    try:
        summary = freeze_episode(assembler, records, staging)
    except Exception as exc:  # noqa: BLE001
        status = build_status_doc(
            base, None, acquisition='not_started',
            contract=('not_run', 'checker_not_run'),
            integrity=('not_run', 'integrity_not_run'),
            terminal='write_failed', reason='write_failed',
            detail='%s: %s' % (type(exc).__name__, exc))
        write_json(os.path.join(staging, 'status.json'), status)
        return 1, status

    # --- provisional status inside staging (never formally validated) ---
    acq, _missing = acquisition_completeness(records, streams, cb_errors)
    write_json(os.path.join(staging, 'status.json'),
               build_status_doc(
                   base, summary, acquisition=acq,
                   contract=('not_run', 'checker_not_run'),
                   integrity=('not_run', 'integrity_not_run'),
                   terminal='in_progress', reason=None, phase='staging'))

    # --- integrity verification (R-04, gated) --------------------------
    integrity = verify_integrity(staging, summary)

    # --- frozen checker -------------------------------------------------
    try:
        rc, report, _out, _err = run_frozen_checker(staging)
        contract = classify_contract(rc, report)
    except Exception:  # noqa: BLE001 — launch failure / timeout
        rc, report = None, None
        contract = ('error', 'checker_error')
    base['checker_exit_code'] = rc
    base['checker_ok'] = bool(isinstance(report, dict)
                              and report.get('smoke', {}).get('ok'))
    if isinstance(report, dict):
        write_json(os.path.join(staging, 'smoke_report.json'), report)

    # --- classify the pre-publish terminal ------------------------------
    terminal = classify_terminal(acq, contract[0], integrity[0], cb_errors,
                                 sess_errors, aborted=bool(info.get('aborted')))
    reason = REASON_FOR_TERMINAL.get(terminal)

    # --- interrupt: abort without publishing -----------------------------
    if terminal == 'aborted':
        status = build_status_doc(base, summary, acquisition=acq,
                                  contract=contract, integrity=integrity,
                                  terminal='aborted', reason='aborted',
                                  phase='staging')
        write_json(os.path.join(staging, 'status.json'), status)
        return 1, status

    # --- publish (R-02) --------------------------------------------------
    pub = _publish(staging, out_root)
    if not pub['ok']:
        status = build_status_doc(base, summary, acquisition=acq,
                                  contract=contract, integrity=integrity,
                                  terminal='publish_failed',
                                  reason='publish_failed',
                                  phase='publish_failed',
                                  detail=pub.get('detail'),
                                  publish=pub['status'])
        if os.path.isdir(staging):
            write_json(os.path.join(staging, 'status.json'), status)
        return 1, status

    # --- commit the final status atomically into the published target ---
    phase = ('validated' if terminal == 'complete_validated'
             else 'published_unvalidated')
    status = build_status_doc(base, summary, acquisition=acq, contract=contract,
                              integrity=integrity, terminal=terminal,
                              reason=reason, phase=phase, publish='published')
    try:
        write_json(os.path.join(out_root, 'status.json'), status)  # atomic
    except Exception as exc:  # noqa: BLE001 — final-status write failure
        status = build_status_doc(
            base, summary, acquisition=acq, contract=contract,
            integrity=integrity, terminal='status_write_failed',
            reason='status_write_failed', phase='published_unvalidated',
            detail='%s: %s' % (type(exc).__name__, exc))
        return 1, status
    return (0 if terminal == 'complete_validated' else 1), status


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
    # R-04 (artifact-list-independent failure paths): recompute the recorded
    # integrity hash and require frames.json ↔ frames.jsonl content equality.
    # (The full required-artifact-list pass remains gated — see verify_integrity.)
    integrity_errors = []
    try:
        recomputed = sha256_file(os.path.join(args.episode_dir, 'frames.jsonl'))
    except Exception as exc:  # noqa: BLE001
        recomputed = None
        integrity_errors.append('frames.jsonl unreadable: %s' % exc)
    recorded = prov.get('episode_integrity_hash') \
        if isinstance(prov, dict) else None
    report['recorded_integrity_hash'] = recorded
    report['recomputed_integrity_hash'] = recomputed
    if recorded is not None and recomputed is not None \
            and recorded != recomputed:
        integrity_errors.append(
            'episode_integrity_hash mismatch (recorded != recomputed)')
    try:
        with open(os.path.join(args.episode_dir, 'frames.jsonl'),
                  encoding='utf-8') as fh:
            jsonl = [strict(ln) for ln in fh.read().splitlines() if ln.strip()]
    except Exception as exc:  # noqa: BLE001
        jsonl = None
        integrity_errors.append('frames.jsonl parse failed: %s' % exc)
    report['frames_json_matches_jsonl'] = (
        isinstance(frames, list) and jsonl is not None and frames == jsonl)
    if isinstance(frames, list) and jsonl is not None and frames != jsonl:
        integrity_errors.append('frames.json != frames.jsonl content')
    report['integrity_errors'] = integrity_errors
    rc, smoke, out, _err = run_frozen_checker(args.episode_dir)
    report['checker_exit_code'] = rc
    report['checker_ok'] = bool(smoke and smoke.get('smoke', {}).get('ok'))
    report['status'] = 'PASS' if (rc == 0 and report['frame_count_consistent']
                                  and not integrity_errors) else 'FAIL'
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
