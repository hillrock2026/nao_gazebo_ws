#!/usr/bin/env python3
"""Stage 1 pre-acquisition probes: V-1 (contact raw path) and V-2 (pose path).

Contract: docs/STAGE1_DYNAMICS_DATASET_RFC.md (REV1.2-CORRECTED, pending
re-audit), Appendix B. This script is Level 1 preparation instrumentation
(RFC § 0.0):

  * read-only — it only subscribes to existing topics and writes JSON reports
    under dataset/stage1/probe/;
  * it publishes nothing, launches nothing, and modifies no frozen component
    (runner, controller/gait generator, converters, model, world, physics,
    launch files);
  * it never starts acquisition — the 96 episodes are Level 3 and stay gated.

V-1 determines (never assumes) the contact raw-path facts:
  1. runtime population & cadence of both foot contact topics on BOTH the ROS
     bridge path and the gz-transport native path (REV1.2/R1: the native
     Contacts stream is the canonical raw source; the bridge is an optional
     derived stream — its field loss is documented, a dead bridge is
     recorded but does not block the native path);
  2. body identity — verbatim collision / body entity names observed;
  3. publication mode — which payload fields are populated (positions,
     normals, depths, wrenches; force_offset and world on the native path;
     the ROS bridge schema is introspected and recorded for field loss);
  4. timestamp semantics (REV1.2/R2) — timestamp-domain validation on SOURCE
     stamps only: the same simulation clock domain as /joint_states is
     established via nearest-neighbor statistics (median / p95 / max of
     |contact stamp - nearest /joint_states stamp|); monotonicity and
     measured dt; exact-stamp preservation across the ROS bridge. The
     REV1.1 envelope-overlap criterion is DELETED — subscriber capture
     window overlap is a discovery-skew artifact, not timestamp-domain
     validation; callback arrival time is never used as measurement time;
  5. frame/reference interpretation — N3L / standing equilibrium is used ONLY
     as a plausibility check (GPT-5.6 correction 3). It never substitutes for
     a common-frame/common-origin comparison, so it can never mark this
     criterion more than "plausible".

V-2 determines the /world/nao_world/pose/info facts:
  1. unique entities — all 25 links of nao_simple.sdf as unique, exactly-named
     entries;
  2. pose frame semantics (REV1.2/R4) — DETERMINED from evidence, never
     assumed, and REPORTED as relative/model-relative, never as world pose:
     run 1 falsified the RFC REV1.1 App. B V-2.2 literal reading — link
     entries are parent-frame / model-relative poses while the model entry
     carries the world pose. The model entry is cross-checked against
     /body_pose (exact-stamp), world poses are recovered by composing the
     nao model entry with each link entry
     (p_w = p_nao + R(q_nao).p_rel, q_w = q_nao (x) q_rel). REV1.2/R5: the
     base_link (identity-link) comparison is a consistency check ONLY — the
     composition method requires verification using selected non-identity
     links before composed poses are treated as validated;
  3. source stamps & measured cadence (SceneBroadcaster has no configured
     publish rate — measured, not assumed);
  4. alignment hit rate (the operative check) — the fraction of joint-clock
     frames whose causal last-not-after 0.02 s window contains at least one
     pose message must be >= 95 %.

Usage (ROS 2 Jazzy sourced, workspace overlay sourced, simulation already
running on the same ROS_DOMAIN_ID / GZ_PARTITION):

    python3 tools/stage1_probe.py --probe both --duration 20.0 \
        --warmup-sec 5.0

REV1.2 evidence run (report suffix; writes V1_report_REV12.json /
V2_report_REV12.json):

    python3 tools/stage1_probe.py --probe both --duration 20.0 \
        --warmup-sec 30.0 --ros-domain-id 88 \
        --gz-partition stage1probe4 --report-suffix REV12

Standing-scenario launch used with this probe (runtime args only, no file
edits — RFC § 5.4). It mirrors the acquisition LAUNCH_BASE
(tools/run_phase_a_episode.py): phase4a loads the actuator with command
publishing left to the gait producer, phase4b gait keeps the robot
standing. Run 1 evidence: with enable_phase4b:=false there is no
/joint_command publisher at all — the robot collapses within ~6 s, both
feet leave the ground, and V-1 observes zero contact messages in the
window (fallen-robot artifact, archived as *.run1_fallen.json):

    ros2 launch nao_gz_sim nao_sim.launch.py gui:=false use_rsp:=true \
        enable_phase4a:=true phase4a_run_probe:=false phase4a_run_trace:=false \
        enable_phase4b:=true phase4b_run_gait:=true phase4b_run_trace:=false \
        phase4b_gait_method:=stand phase4b_duration_sec:=70 \
        phase4b_stand_sec:=70
"""

import argparse
import bisect
import collections
import hashlib
import json
import math
import os
import re
import statistics
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone

TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(TOOLS)

RFC_PATH = os.path.join(WS, 'docs', 'STAGE1_DYNAMICS_DATASET_RFC.md')
SDF_PATH = os.path.join(WS, 'src', 'nao', 'nao_description', 'urdf',
                        'nao_simple.sdf')

LEFT_CONTACT_TOPIC = '/nao/left_foot_contact_raw'
RIGHT_CONTACT_TOPIC = '/nao/right_foot_contact_raw'
CONTACT_TOPICS = (LEFT_CONTACT_TOPIC, RIGHT_CONTACT_TOPIC)
POSE_INFO_TOPIC = '/world/nao_world/pose/info'
JOINT_STATES_TOPIC = '/joint_states'
BODY_POSE_TOPIC = '/body_pose'

# The 25 links of the frozen nao_simple.sdf (RFC § 0.1).
EXPECTED_LINKS = [
    'base_link', 'Neck', 'Head', 'LPelvis', 'LHip', 'LThigh', 'LTibia',
    'LAnklePitch_link', 'l_ankle', 'LShoulder', 'LBicep', 'LElbow',
    'LForeArm', 'l_wrist', 'RPelvis', 'RHip', 'RThigh', 'RTibia',
    'RAnklePitch_link', 'r_ankle', 'RShoulder', 'RBicep', 'RElbow',
    'RForeArm', 'r_wrist',
]

# Entities tracked per pose/info message (model + key links + ground).
TRACKED_ENTITIES = ('nao', 'base_link', 'l_ankle', 'r_ankle',
                    'ground_plane')

# Collision local names inside nao_simple.sdf (expected, verified at runtime).
EXPECTED_FOOT_COLLISION = {
    LEFT_CONTACT_TOPIC: 'l_ankle_collision',
    RIGHT_CONTACT_TOPIC: 'r_ankle_collision',
}

G_VALUES = (9.8, 9.81)  # m/s^2 — sanity envelope only (plausibility, GPT-5.6 #3).
N3L_RATIO_MIN, N3L_RATIO_MAX = 0.5, 1.5
N3L_PAIRWISE_MAX_FRACTION = 0.2  # |fz_L + fz_R| / (m*g) upper bound.

V2_HIT_RATE_MIN = 0.95           # RFC App. B V-2 criterion 4 (>= 95 %).
CADENCE_MIN_HZ, CADENCE_MAX_HZ = 50.0, 150.0
POPULATED_FRACTION_MIN = 0.95
STAMP_ENVELOPE_FRACTION_MIN = 0.95
BODY_POSE_MATCH_FRACTION_MIN = 0.99
BODY_POSE_MATCH_MAX_DIFF_M = 1e-6
BODY_POSE_MATCH_MAX_Q_DIFF = 1e-9
ALL_LINKS_PER_MESSAGE_MIN = 0.99  # RFC: one message carries all 25 link poses.
STANDING_XY_MAX_M = 0.5
STANDING_BASE_Z_RANGE = (0.05, 1.0)
STANDING_ANKLE_Z_RANGE = (0.0, 0.15)
GROUND_Z_MAX_ABS_M = 0.01
MIN_MESSAGES_FOR_CADENCE = 50
MAX_CALLBACK_ERRORS = 200

# RFC § 3.6.3 causal association windows (seconds).
CONTACT_ASSOC_WINDOW_SEC = 0.05
POSE_ASSOC_WINDOW_SEC = 0.02

# Frame-semantics determination (V-2 criterion 2 — determined, not assumed).
DYNAMIC_POSE_TOPIC = '/world/nao_world/dynamic_pose/info'
FRAME_IDENTITY_TOL = 1e-6        # m / quat component: base_link == model frame.
FRAME_SEMANTICS_FRACTION_MIN = 0.99
DYNAMIC_MATCH_MAX_DIFF_M = 1e-3  # classification tolerance, see analyze_v2.

CRITERION_STATUS = ('confirmed', 'plausible', 'contradicted', 'unknown')
V1_BLOCKING = ('population_cadence', 'body_identity', 'publication_mode',
               'timestamp_semantics')
V2_BLOCKING = ('unique_entities', 'pose_frame_semantics',
               'source_stamps_cadence', 'alignment_hit_rate')


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def git_commit():
    """Best-effort HEAD commit of the frozen worktree (probe pinning)."""
    try:
        out = subprocess.run(
            ['git', 'rev-parse', 'HEAD'], cwd=WS, capture_output=True,
            text=True, timeout=10)
        return out.stdout.strip() or None
    except Exception:
        return None


def sdf_mass_summary(path):
    """Sum <mass> in nao_simple.sdf; report m*g for both g values."""
    masses = []
    try:
        text = open(path, 'r', encoding='utf-8', errors='replace').read()
        masses = [float(m) for m in re.findall(
            r'<mass>([0-9.eE+-]+)</mass>', text)]
    except Exception:
        pass
    total = sum(masses) if masses else None
    return {
        'sdf_path': path,
        'n_mass_elements': len(masses),
        'total_mass_kg': total,
        'm_g_newtons': [round(total * g, 4) for g in G_VALUES]
        if total is not None else None,
    }


def dt_stats(stamps):
    """Measured dt statistics over strictly-ordered second floats."""
    if len(stamps) < 2:
        return {'n_dt': 0}
    dts = [b - a for a, b in zip(stamps, stamps[1:])]
    pos = [d for d in dts if d > 0]
    hz = 1.0 / (statistics.mean(pos)) if pos else None
    return {
        'n_dt': len(dts),
        'dt_min_s': round(min(dts), 9),
        'dt_max_s': round(max(dts), 9),
        'dt_mean_s': round(statistics.mean(dts), 9),
        'dt_median_s': round(statistics.median(dts), 9),
        'dt_stdev_s': round(statistics.stdev(dts), 9) if len(dts) > 1 else 0.0,
        'mean_rate_hz': round(hz, 6) if hz else None,
        'n_nonpositive_dt': sum(1 for d in dts if d <= 0),
    }


def percentile(sorted_vals, p):
    """Nearest-rank percentile of an ascending list; None when empty."""
    if not sorted_vals:
        return None
    k = max(0, min(len(sorted_vals) - 1,
                   int(math.ceil(p * len(sorted_vals))) - 1))
    return sorted_vals[k]


def round_floats(obj, ndigits=9):
    """Recursively round floats; reject non-finite values (JSON-safe)."""
    if isinstance(obj, float):
        if not math.isfinite(obj):
            raise ValueError('non-finite float in report: %r' % obj)
        return round(obj, ndigits)
    if isinstance(obj, dict):
        return {k: round_floats(v, ndigits) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [round_floats(v, ndigits) for v in obj]
    return obj


def atomic_write_json(path, payload):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(payload, f, indent=2, allow_nan=False, sort_keys=False)
        f.write('\n')
    os.replace(tmp, path)


def local_name(entity_name):
    """'nao::base_link' -> 'base_link' (unchanged when unscoped)."""
    if isinstance(entity_name, str) and '::' in entity_name:
        return entity_name.split('::')[-1]
    return entity_name


# ---------------------------------------------------------------------------
# Record builders (defensive: every field read through _field / _unwrap_ros).
# ---------------------------------------------------------------------------

def _field(msg, name, default=None):
    try:
        return getattr(msg, name)
    except AttributeError:
        return default


def _unwrap_ros(value):
    """std_msgs wrappers (String/UInt32) expose the payload as .data."""
    if value is None:
        return None
    data = getattr(value, 'data', None)
    if data is not None and not hasattr(data, 'x'):
        return data
    return value


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
    """gz Header (sec/nsec attrs) or ROS Header (stamp sec/nanosec attrs)."""
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


def _entity_record(ent):
    if ent is None:
        return None
    return {
        'id': _field(ent, 'id'),
        'name': _field(ent, 'name'),
        'type': str(_field(ent, 'type')),
    }


def _gz_wrench_record(w):
    """gz.msgs JointWrench — verbatim both body sides incl. force_offset."""
    if w is None:
        return None
    rec = {}
    for side in ('body_1', 'body_2'):
        rec[side + '_name'] = _field(w, side + '_name')
        rec[side + '_id'] = _field(w, side + '_id')
        wr = _field(w, side + '_wrench')
        rec[side + '_force'] = _vec3(_field(wr, 'force')) if wr else None
        rec[side + '_torque'] = _vec3(_field(wr, 'torque')) if wr else None
        rec[side + '_force_offset'] = _vec3(
            _field(wr, 'force_offset')) if wr else None
    return rec


def _ros_wrench_record(w):
    """ros_gz JointWrench — std_msgs wrappers unwrapped; schema has no
    force_offset (bridge field loss recorded in the report)."""
    if w is None:
        return None
    rec = {}
    for side in ('body_1', 'body_2'):
        rec[side + '_name'] = _unwrap_ros(_field(w, side + '_name'))
        rec[side + '_id'] = _unwrap_ros(_field(w, side + '_id'))
        wr = _field(w, side + '_wrench')
        if wr is None and side == 'body_1':
            wr = _field(w, 'body_1_wrench')
        rec[side + '_force'] = _vec3(_field(wr, 'force')) if wr else None
        rec[side + '_torque'] = _vec3(_field(wr, 'torque')) if wr else None
    return rec


def gz_contact_record(msg):
    """Compact per-message record of a gz.msgs Contacts message."""
    rec = {
        'stamp_s': _stamp_seconds(_field(msg, 'header')),
        'n_contacts': len(_field(msg, 'contact') or []),
        'contacts': [],
    }
    for c in (_field(msg, 'contact') or []):
        positions = list(_field(c, 'position') or [])
        normals = list(_field(c, 'normal') or [])
        depths = [float(d) for d in (_field(c, 'depth') or [])]
        crec = {
            'collision1': _entity_record(_field(c, 'collision1')),
            'collision2': _entity_record(_field(c, 'collision2')),
            'world': _entity_record(_field(c, 'world')),
            'contact_stamp_s': _stamp_seconds(_field(c, 'header')),
            'n_positions': len(positions),
            'first_position': _vec3(positions[0]) if positions else None,
            'n_normals': len(normals),
            'first_normal': _vec3(normals[0]) if normals else None,
            'depths': depths,
            'wrenches': [_gz_wrench_record(w)
                         for w in (_field(c, 'wrench') or [])],
        }
        rec['contacts'].append(crec)
    return rec


def ros_contact_record(msg):
    """Compact per-message record of a ros_gz_interfaces/msg/Contacts."""
    contacts = _field(msg, 'contacts')
    if contacts is None:
        contacts = _field(msg, 'contact') or []
    rec = {
        'stamp_s': _stamp_seconds(_field(msg, 'header')),
        'n_contacts': len(list(contacts or [])),
        'contacts': [],
    }
    for c in (contacts or []):
        positions = list(_field(c, 'positions') or [])
        normals = list(_field(c, 'normals') or [])
        depths = [float(d) for d in (_field(c, 'depths') or [])]
        crec = {
            'collision1': _entity_record(_field(c, 'collision1')),
            'collision2': _entity_record(_field(c, 'collision2')),
            'contact_stamp_s': _stamp_seconds(_field(c, 'header')),
            'n_positions': len(positions),
            'first_position': _vec3(positions[0]) if positions else None,
            'n_normals': len(normals),
            'first_normal': _vec3(normals[0]) if normals else None,
            'depths': depths,
            'wrenches': [_ros_wrench_record(w)
                         for w in (_field(c, 'wrenches') or [])],
        }
        rec['contacts'].append(crec)
    return rec


def joint_state_record(msg):
    """Compact /joint_states record — stamps drive the analysis."""
    return {
        'stamp_s': _stamp_seconds(_field(msg, 'header')),
        'n_name': len(_field(msg, 'name') or []),
        'n_position': len(_field(msg, 'position') or []),
        'n_velocity': len(_field(msg, 'velocity') or []),
        'n_effort': len(_field(msg, 'effort') or []),
    }


def body_pose_record(msg):
    """geometry_msgs/PoseStamped from body_pose_converter (stamp = source)."""
    p = _field(msg, 'pose')
    return {
        'stamp_s': _stamp_seconds(_field(msg, 'header')),
        'frame_id': _field(_field(msg, 'header'), 'frame_id'),
        'position': _vec3(_field(p, 'position')) if p else None,
        'orientation': _quat(_field(p, 'orientation')) if p else None,
    }


def gz_pose_record(msg):
    """gz.msgs Pose_V — all entries (name, id) + tracked entity poses."""
    poses = list(_field(msg, 'pose') or [])
    entries = []
    for p in poses:
        name = _field(p, 'name')
        if isinstance(name, str):
            name = sys.intern(name)
        entries.append((name, _field(p, 'id')))
    tracked = {}
    for p in poses:
        name = _field(p, 'name')
        if not isinstance(name, str):
            continue
        key = local_name(name)
        if key in TRACKED_ENTITIES and key not in tracked:
            tracked[key] = {
                'id': _field(p, 'id'),
                'position': _vec3(_field(p, 'position')),
                'orientation': _quat(_field(p, 'orientation')),
                'verbatim_name': name,
            }
    return {
        'stamp_s': _stamp_seconds(_field(msg, 'header')),
        'entries': entries,
        'tracked': tracked,
    }


# ---------------------------------------------------------------------------
# Thread-safe record store + callback error capture.
# ---------------------------------------------------------------------------

_CALLBACK_ERRORS = []


def record_callback_error(kind, exc):
    if len(_CALLBACK_ERRORS) < MAX_CALLBACK_ERRORS:
        _CALLBACK_ERRORS.append({
            'kind': kind,
            'error': '%s: %s' % (type(exc).__name__, exc),
            'wallclock': utc_now(),
        })


class LockedRecords:
    """Append-only, lock-guarded record list for one stream."""

    def __init__(self):
        self._lock = threading.Lock()
        self._records = []
        self.first_msg_wall = None
        self.last_msg_wall = None

    def append(self, rec):
        with self._lock:
            now = time.time()
            if self.first_msg_wall is None:
                self.first_msg_wall = now
            self.last_msg_wall = now
            self._records.append(rec)

    def snapshot(self):
        with self._lock:
            return list(self._records)

    def __len__(self):
        with self._lock:
            return len(self._records)


def guarded(kind, store, builder):
    """Wrap a callback: build a record, capture any exception."""
    def cb(msg):
        try:
            store.append(builder(msg))
        except Exception as exc:  # noqa: BLE001 — recorded, never fatal
            record_callback_error(kind, exc)
    return cb


# ---------------------------------------------------------------------------
# Lazy importers (gz msgs/transport variants; rclpy + message classes).
# ---------------------------------------------------------------------------

def gz_packages():
    """Return (contacts_pb2, pose_v_pb2, transport) or Nones on failure."""
    contacts = pose_v = None
    for msgs_mod in ('gz.msgs11', 'gz.msgs10', 'gz.msgs9'):
        try:
            m = __import__(msgs_mod, fromlist=[
                'contacts_pb2', 'pose_v_pb2'])
            contacts = m.contacts_pb2
            pose_v = m.pose_v_pb2
            break
        except ImportError:
            continue
    transport = None
    for transport_mod in ('gz.transport13', 'gz.transport12'):
        try:
            transport = __import__(transport_mod, fromlist=['Node'])
            break
        except ImportError:
            continue
    return contacts, pose_v, transport


def ros_packages():
    """Return (rclpy, Contacts, Contact, JointWrench, JointState,
    PoseStamped) — raises ImportError with a clear message when missing."""
    import rclpy  # noqa: F401
    from ros_gz_interfaces.msg import Contacts, Contact, JointWrench
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import PoseStamped
    return rclpy, Contacts, Contact, JointWrench, JointState, PoseStamped


# ---------------------------------------------------------------------------
# Message-schema introspection (gz protobuf descriptors + ROS field types).
# ---------------------------------------------------------------------------

_PB_FIELD_TYPES = {
    1: 'double', 2: 'float', 3: 'int64', 4: 'uint64', 5: 'int32',
    6: 'fixed64', 7: 'fixed32', 8: 'bool', 9: 'string', 10: 'group',
    11: 'message', 12: 'bytes', 13: 'uint32', 14: 'enum', 15: 'sfixed32',
    16: 'sfixed64', 17: 'sint32', 18: 'sint64',
}


def _pb_tree(desc, depth, max_depth):
    out = {}
    for fname, fdesc in sorted(desc.fields_by_name.items()):
        kind = _PB_FIELD_TYPES.get(fdesc.type, 'type_%d' % fdesc.type)
        if fdesc.label == 3:
            kind = 'repeated ' + kind
        if fdesc.type == 11 and depth < max_depth:
            out[fname] = _pb_tree(fdesc.message_type, depth + 1, max_depth)
        else:
            out[fname] = kind
    return out


def introspect_pb(msg_class, max_depth=3):
    if not hasattr(msg_class, 'DESCRIPTOR'):
        return None
    return _pb_tree(msg_class.DESCRIPTOR, 0, max_depth)


def introspect_ros(msg_class):
    try:
        fat = msg_class.get_fields_and_field_types()
    except Exception:
        return None
    return {name: str(ftype) for name, ftype in fat.items()}


def message_schemas(contacts_pb2, pose_v_pb2, ros_contact, ros_joint_wrench,
                    ros_contacts, joint_state_cls, pose_stamped_cls):
    """Determined (introspected) schemas of every message involved."""
    schemas = {}
    try:
        contacts_cls = contacts_pb2.Contacts
        schemas['gz.msgs.Contacts'] = introspect_pb(contacts_cls)
        cd = contacts_cls.DESCRIPTOR.fields_by_name['contact'].message_type
        schemas['gz.msgs.Contact'] = _pb_tree(cd, 0, 2)
        schemas['gz.msgs.Entity'] = _pb_tree(
            cd.fields_by_name['collision1'].message_type, 0, 1)
        jw = cd.fields_by_name['wrench'].message_type
        schemas['gz.msgs.JointWrench'] = _pb_tree(jw, 0, 2)
        schemas['gz.msgs.Wrench'] = _pb_tree(
            jw.fields_by_name['body_1_wrench'].message_type, 0, 1)
        schemas['gz.msgs.Vector3d'] = _pb_tree(
            cd.fields_by_name['position'].message_type, 0, 1)
    except Exception as exc:
        record_callback_error('schema:gz_contacts', exc)
    try:
        pose_v_cls = pose_v_pb2.Pose_V
        schemas['gz.msgs.Pose_V'] = introspect_pb(pose_v_cls)
        pd = pose_v_cls.DESCRIPTOR.fields_by_name['pose'].message_type
        schemas['gz.msgs.Pose'] = _pb_tree(pd, 0, 2)
    except Exception as exc:
        record_callback_error('schema:gz_pose', exc)
    schemas['ros_gz_interfaces/msg/Contacts'] = introspect_ros(ros_contacts)
    schemas['ros_gz_interfaces/msg/Contact'] = introspect_ros(ros_contact)
    schemas['ros_gz_interfaces/msg/JointWrench'] = introspect_ros(
        ros_joint_wrench)
    schemas['sensor_msgs/msg/JointState'] = introspect_ros(joint_state_cls)
    schemas['geometry_msgs/msg/PoseStamped'] = introspect_ros(
        pose_stamped_cls)
    return schemas


# gz -> ros_gz_interfaces semantic field mapping (schema-level).
_BRIDGE_FIELD_MAP = {
    'Contact': {
        'header': None, 'collision1': 'collision1', 'collision2':
        'collision2', 'position': 'positions', 'normal': 'normals',
        'depth': 'depths', 'wrench': 'wrenches', 'world': None,
    },
    'JointWrench': {
        'body_1_name': 'body_1_name', 'body_1_id': 'body_1_id',
        'body_2_name': 'body_2_name', 'body_2_id': 'body_2_id',
        'body_1_wrench': 'body_1_wrench', 'body_2_wrench': 'body_2_wrench',
    },
    'Wrench': {'force': 'force', 'torque': 'torque', 'force_offset': None},
}


def bridge_field_loss(ros_contact=None, ros_joint_wrench=None):
    """gz fields with no ROS-bridge counterpart, per message level.

    Cross-checked against the introspected ROS schemas when the classes are
    available; a mapping to a field absent from the ROS schema also counts
    as gz_only_unmapped."""
    ros_fields = {}
    for cls in (ros_contact, ros_joint_wrench):
        if cls is None:
            continue
        try:
            fat = cls.get_fields_and_field_types()
        except Exception:
            continue
        if fat:
            ros_fields[cls.__name__] = set(fat.keys())
    result = {}
    for level, mapping in _BRIDGE_FIELD_MAP.items():
        level_ros = ros_fields.get(level)
        gz_only = []
        for gz_field, ros_field in mapping.items():
            if ros_field is None:
                gz_only.append(gz_field)
            elif level_ros is not None and ros_field not in level_ros:
                gz_only.append(gz_field)
        result[level] = {
            'gz_fields': sorted(mapping.keys()),
            'mapped_ros_fields': sorted(v for v in mapping.values() if v),
            'gz_only_unmapped': sorted(gz_only),
            'ros_schema_cross_checked': level_ros is not None,
        }
    return result


# ---------------------------------------------------------------------------
# Probe session: one rclpy node + one gz-transport Node, background spin.
# ---------------------------------------------------------------------------

class ProbeSession:
    """Assumes rclpy.init() has already been called (main does it once)."""

    def __init__(self, node_name, transport_module=None):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        self.start_wall = time.time()
        self.node = rclpy.create_node(node_name)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._spin_thread = threading.Thread(
            target=self._spin, name='probe-spin', daemon=True)
        self._spin_thread.start()
        self._transport = transport_module
        self.gz_node = None
        self.session_errors = []

    def _spin(self):
        try:
            self.executor.spin()
        except Exception as exc:  # noqa: BLE001 — recorded, never fatal
            self.session_errors.append(
                'executor.spin: %s: %s' % (type(exc).__name__, exc))

    def ros_subscribe(self, topic, msg_type, callback):
        try:
            from rclpy.qos import (DurabilityPolicy, HistoryPolicy,
                                   QoSProfile, ReliabilityPolicy)
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=50,
                durability=DurabilityPolicy.VOLATILE)
            self.node.create_subscription(msg_type, topic, callback, qos)
            return True
        except Exception as exc:  # noqa: BLE001
            self.session_errors.append(
                'ros_subscribe %s: %s: %s' % (topic,
                                              type(exc).__name__, exc))
            return False

    def gz_subscribe(self, msg_type, topic, callback):
        try:
            if self.gz_node is None:
                Node = self._transport.Node
                self.gz_node = Node()
            ok = self.gz_node.subscribe(msg_type, topic, callback)
            return bool(ok)
        except Exception as exc:  # noqa: BLE001
            self.session_errors.append(
                'gz_subscribe %s: %s: %s' % (topic,
                                             type(exc).__name__, exc))
            return False

    def close(self):
        for closer in (self.executor.shutdown, self.node.destroy_node):
            try:
                closer()
            except Exception:
                pass


def wait_for_streams(streams, warmup_sec, poll_sec=0.5):
    """Block until every stream received >= 1 message or warmup expires.

    Returns the list of stream kinds still empty after the warmup."""
    deadline = time.time() + warmup_sec
    while True:
        missing = [kind for kind, s in streams.items() if len(s) == 0]
        if not missing or time.time() >= deadline:
            return missing
        time.sleep(poll_sec)


def run_collection(label, spec, duration, warmup_sec, transport_module=None):
    """Subscribe every stream in spec, warm up, collect for duration.

    spec items: (kind, transport, topic, msg_type, builder). Returns
    {'records': {kind: [...]}, 'streams': {kind: {...}}, 'errors': {...}}."""
    _CALLBACK_ERRORS.clear()
    session = ProbeSession('stage1_probe_%s' % label,
                           transport_module=transport_module)
    streams = {}
    subscribe_ok = {}
    topics = {}
    transports = {}
    for kind, transport_kind, topic, msg_type, builder in spec:
        store = LockedRecords()
        cb = guarded(kind, store, builder)
        if transport_kind == 'ros':
            ok = session.ros_subscribe(topic, msg_type, cb)
        else:
            ok = session.gz_subscribe(msg_type, topic, cb)
        streams[kind] = store
        subscribe_ok[kind] = ok
        topics[kind] = topic
        transports[kind] = transport_kind
    missing = wait_for_streams(streams, warmup_sec)
    if len(missing) < len(streams):
        time.sleep(duration)
    records = {kind: store.snapshot() for kind, store in streams.items()}
    session.close()
    stream_info = {}
    for kind, store in streams.items():
        n = len(records[kind])
        n_valid = sum(1 for r in records[kind]
                      if isinstance(r.get('stamp_s'), float))
        first_after = (store.first_msg_wall - session.start_wall
                       if store.first_msg_wall is not None else None)
        wall_span = (store.last_msg_wall - store.first_msg_wall
                     if store.first_msg_wall is not None
                     and store.last_msg_wall is not None else None)
        stream_info[kind] = {
            'transport': transports[kind],
            'topic': topics[kind],
            'subscribe_ok': subscribe_ok[kind],
            'first_msg_after_s': round(first_after, 3)
            if first_after is not None else None,
            'wall_span_s': round(wall_span, 3) if wall_span else None,
            'n_messages': n,
            'n_valid_stamp': n_valid,
            'empty_after_warmup': kind in missing,
        }
    return {
        'records': records,
        'streams': stream_info,
        'session_errors': list(session.session_errors),
        'callback_errors': list(_CALLBACK_ERRORS),
    }


# ---------------------------------------------------------------------------
# V-1: contact raw-path probe (gz native + ROS bridge + joint clock).
# ---------------------------------------------------------------------------

def gz_kind(topic):
    return 'gz_' + ('left' if topic == LEFT_CONTACT_TOPIC else 'right')


def ros_kind(topic):
    return 'ros_' + ('left' if topic == LEFT_CONTACT_TOPIC else 'right')


def _criterion(cid, title, blocking, status, evidence, notes=''):
    return {
        'id': cid,
        'title': title,
        'blocking': blocking,
        'status': status,
        'evidence': evidence,
        'notes': notes,
    }


def _topic_facts_v1(records, topic):
    gz_recs = records.get(gz_kind(topic), [])
    ros_recs = records.get(ros_kind(topic), [])
    stamps = [r['stamp_s'] for r in gz_recs
              if isinstance(r.get('stamp_s'), float)]
    nonempty = [r for r in gz_recs if r.get('n_contacts', 0) > 0]
    return {
        'gz_n': len(gz_recs),
        'gz_nonempty': len(nonempty),
        'gz_empty': len(gz_recs) - len(nonempty),
        'ros_n': len(ros_recs),
        'stamps': stamps,
        'dt': dt_stats(sorted(stamps)) if len(stamps) >= 2 else {'n_dt': 0},
        'gz_recs': gz_recs,
        'ros_recs': ros_recs,
    }


def analyze_v1(collected, args, provenance):
    records = collected['records']
    streams = collected['streams']
    criteria = []
    tf = {topic: _topic_facts_v1(records, topic) for topic in CONTACT_TOPICS}

    # --- criterion 1: population & cadence, native path canonical (blocking)
    pop_ev, pop_problems, pop_notes = {}, [], []
    pop_status = 'confirmed'
    for topic in CONTACT_TOPICS:
        t = tf[topic]
        hz = t['dt'].get('mean_rate_hz')
        ev = {
            'gz_n_messages': t['gz_n'],
            'gz_mean_rate_hz': hz,
            'gz_dt': t['dt'],
            'ros_n_messages': t['ros_n'],
            'native_path_canonical': True,
        }
        gz_stream = streams.get(gz_kind(topic), {})
        wall = gz_stream.get('wall_span_s')
        if t['stamps'] and wall:
            sim_span = max(t['stamps']) - min(t['stamps'])
            ev['gz_sim_span_s'] = round(sim_span, 6)
            ev['gz_wall_span_s'] = wall
            ev['gz_sim_over_wall_rate'] = round(sim_span / wall, 6)
        pop_ev[topic] = ev
        if t['gz_n'] < MIN_MESSAGES_FOR_CADENCE:
            pop_status = 'contradicted'
            pop_problems.append(
                '%s: gz native path received %d messages (< %d)' % (
                    topic, t['gz_n'], MIN_MESSAGES_FOR_CADENCE))
        elif hz is None or not (CADENCE_MIN_HZ <= hz <= CADENCE_MAX_HZ):
            pop_status = 'contradicted'
            pop_problems.append(
                '%s: measured gz cadence %s Hz outside expected band '
                '[%s, %s]' % (topic, hz, CADENCE_MIN_HZ, CADENCE_MAX_HZ))
        if t['ros_n'] == 0:
            # REV1.2/R1: the ROS bridge is an optional derived stream — a
            # dead bridge is recorded, but does not block the native path.
            ev['ros_bridge_status'] = 'no_messages_received'
            pop_notes.append(
                '%s: ROS bridge path received 0 messages — optional derived '
                'stream (REV1.2/R1), recorded, non-blocking for the native '
                'canonical path' % topic)
    criteria.append(_criterion(
        'population_cadence',
        'Runtime population & cadence of both foot contact topics on the '
        'gz-transport native path (canonical raw source, REV1.2/R1); the ROS '
        'bridge path is an optional derived stream, recorded but '
        'non-blocking', True,
        pop_status, pop_ev, '; '.join(pop_problems + pop_notes)))

    # --- criterion 2: body identity (blocking) ---------------------------
    body_ev, body_problems = {}, []
    body_status = 'unknown'
    for topic in CONTACT_TOPICS:
        expected_foot = EXPECTED_FOOT_COLLISION[topic]
        other_foot = EXPECTED_FOOT_COLLISION[
            RIGHT_CONTACT_TOPIC if topic == LEFT_CONTACT_TOPIC
            else LEFT_CONTACT_TOPIC]
        pairs = collections.Counter()
        wnames = collections.Counter()
        n_contacts = 0
        n_matching = 0
        n_cross = 0
        for r in tf[topic]['gz_recs']:
            for c in r['contacts']:
                n_contacts += 1
                c1 = (c.get('collision1') or {}).get('name')
                c2 = (c.get('collision2') or {}).get('name')
                pairs['%s | %s' % (c1, c2)] += 1
                l1 = local_name(c1) if isinstance(c1, str) else None
                l2 = local_name(c2) if isinstance(c2, str) else None
                foot_ok = expected_foot in (l1, l2)
                # Ground match must search the FULL scoped names: the ground
                # collision's local name is just 'collision' — the 'ground'
                # token lives in the model scope
                # ('ground_plane::link::collision'). Matching local names
                # only made ground_ok false for every real foot-ground pair.
                full = ('%s %s' % (c1 or '', c2 or '')).lower()
                ground_ok = 'ground' in full
                if foot_ok and ground_ok:
                    n_matching += 1
                if other_foot in (l1, l2):
                    n_cross += 1
                for w in c['wrenches']:
                    for side in ('body_1', 'body_2'):
                        nm = w.get(side + '_name')
                        if isinstance(nm, str):
                            wnames[nm] += 1
        match_frac = (n_matching / n_contacts) if n_contacts else None
        body_ev[topic] = {
            'expected_foot_collision_local': expected_foot,
            'n_nonempty_contacts': n_contacts,
            'fraction_expected_pair': round(match_frac, 6)
            if match_frac is not None else None,
            'n_cross_contamination': n_cross,
            'collision_pairs_top': [
                {'pair': p, 'count': n} for p, n in pairs.most_common(4)],
            'wrench_body_names_top': [
                {'name': nm, 'count': n} for nm, n in wnames.most_common(6)],
        }
        if n_contacts == 0:
            body_problems.append(
                '%s: no non-empty contact messages — body identity not '
                'determinable in this window' % topic)
        elif match_frac is None or match_frac < POPULATED_FRACTION_MIN \
                or n_cross > 0:
            body_status = 'contradicted'
            body_problems.append(
                '%s: expected collision pair fraction %s, cross '
                'contamination %d' % (topic, match_frac, n_cross))
        elif body_status != 'contradicted':
            body_status = 'confirmed'
    if any('not determinable' in p for p in body_problems) \
            and body_status != 'contradicted':
        body_status = 'unknown'
    criteria.append(_criterion(
        'body_identity',
        'Verbatim collision / body entity names map to the expected '
        'ankle/ground pair, left/right distinguishable, no cross '
        'contamination', True, body_status, body_ev,
        '; '.join(body_problems)))

    # --- criterion 3: publication mode (blocking) -------------------------
    pub_ev, pub_problems = {}, []
    pub_status = 'confirmed'
    no_contact_modes = {}
    for topic in CONTACT_TOPICS:
        contacts = [c for r in tf[topic]['gz_recs'] for c in r['contacts']]
        wrenches = [w for c in contacts for w in (c.get('wrenches') or [])]
        n, nw = len(contacts), len(wrenches)

        def frac(items, pred):
            if not items:
                return None
            return round(sum(1 for x in items if pred(x)) / len(items), 6)

        ev = {
            'n_messages': tf[topic]['gz_n'],
            'n_empty_messages': tf[topic]['gz_empty'],
            'n_nonempty_contacts': n,
            'n_wrenches': nw,
            'fraction_positions_populated': frac(
                contacts, lambda c: c.get('n_positions', 0) > 0),
            'fraction_normals_populated': frac(
                contacts, lambda c: c.get('n_normals', 0) > 0),
            'fraction_depths_populated': frac(
                contacts, lambda c: len(c.get('depths') or []) > 0),
            'fraction_world_populated': frac(
                contacts, lambda c: c.get('world') is not None),
            'fraction_wrenches_present': frac(
                contacts, lambda c: len(c.get('wrenches') or []) > 0),
            'fraction_body1_force_populated': frac(
                wrenches, lambda w: w.get('body_1_force') is not None),
            'fraction_body2_force_populated': frac(
                wrenches, lambda w: w.get('body_2_force') is not None),
            'fraction_body1_force_offset_populated': frac(
                wrenches, lambda w: w.get('body_1_force_offset') is not None),
            'fraction_body2_force_offset_populated': frac(
                wrenches, lambda w: w.get('body_2_force_offset') is not None),
        }
        dt = tf[topic]['dt']
        if dt.get('n_dt', 0) >= 2 and dt.get('dt_mean_s'):
            ev['dt_jitter_fraction'] = round(
                (dt.get('dt_stdev_s') or 0.0) / dt['dt_mean_s'], 6)
        jitter = ev.get('dt_jitter_fraction')
        if tf[topic]['gz_empty'] > 0:
            mode = 'periodic_with_empty_messages'
        elif jitter is not None and jitter <= 0.2:
            mode = 'periodic_no_empty_messages_in_window'
        else:
            mode = 'irregular_or_event_only'
        ev['publication_mode'] = mode
        if tf[topic]['gz_empty'] > 0:
            nc = 'empty_messages_observed'
        elif mode == 'periodic_no_empty_messages_in_window':
            nc = 'not_observed_in_window_periodic_cadence'
        else:
            nc = 'not_observable'
        ev['no_contact_observability'] = nc
        no_contact_modes[topic] = nc
        checks = [
            ('positions', ev['fraction_positions_populated']),
            ('normals', ev['fraction_normals_populated']),
            ('depths', ev['fraction_depths_populated']),
            ('world', ev['fraction_world_populated']),
            ('wrenches', ev['fraction_wrenches_present']),
            ('body_1_force', ev['fraction_body1_force_populated']),
            ('body_2_force', ev['fraction_body2_force_populated']),
            ('body_1_force_offset',
             ev['fraction_body1_force_offset_populated']),
            ('body_2_force_offset',
             ev['fraction_body2_force_offset_populated']),
        ]
        for fname, f in checks:
            if f is not None and f < POPULATED_FRACTION_MIN:
                pub_status = 'contradicted'
                pub_problems.append(
                    '%s: %s populated fraction %s < %s' % (
                        topic, fname, f, POPULATED_FRACTION_MIN))
        if n == 0:
            if pub_status != 'contradicted':
                pub_status = 'unknown'
            pub_problems.append(
                '%s: no non-empty contacts — publication mode not '
                'determinable in this window' % topic)
        elif nw == 0:
            pub_status = 'contradicted'
            pub_problems.append(
                '%s: contacts present but zero JointWrench entries — '
                'wrench unpopulated (RFC App. B V-1 failure condition)'
                % topic)
        if nc == 'not_observable':
            pub_status = 'contradicted'
            pub_problems.append(
                '%s: event-only publication — no-contact is unobservable '
                'with no independent signal (RFC App. B V-1 failure '
                'condition)' % topic)
        pub_ev[topic] = ev
    criteria.append(_criterion(
        'publication_mode',
        'Which payload fields are populated per non-empty contact '
        '(positions, normals, depths, wrenches, world, force_offset) and '
        'whether publication is periodic or event-only', True, pub_status,
        pub_ev, '; '.join(pub_problems)))

    # --- criterion 4: timestamp semantics (blocking) ----------------------
    # REV1.2/R2: timestamp-domain validation on SOURCE stamps only. The
    # REV1.1 envelope-overlap criterion is deleted: subscriber capture
    # window overlap measures discovery skew (a probe instrumentation
    # artifact), not stamp-domain agreement. Callback arrival time is not
    # valid measurement time; only preserved source stamps are used.
    def arrival_backwards(seq):
        back = dup = 0
        for a, b in zip(seq, seq[1:]):
            if b < a:
                back += 1
            elif b == a:
                dup += 1
        return back, dup

    def nearest_dist(sorted_stamps, t):
        i = bisect.bisect_left(sorted_stamps, t)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(sorted_stamps):
                d = abs(sorted_stamps[j] - t)
                if best is None or d < best:
                    best = d
        return best

    joint_recs = records.get('joint_states', [])
    joint_arr = [r['stamp_s'] for r in joint_recs
                 if isinstance(r.get('stamp_s'), float)]
    jstamps = sorted(joint_arr)
    joint_dt = dt_stats(jstamps)
    jback, jdup = arrival_backwards(joint_arr)
    ts_ev = {'joint_states': {
        'n_messages': len(joint_recs),
        'n_valid_stamp': len(joint_arr),
        'dt': joint_dt,
        'backwards': jback,
        'duplicates': jdup,
    }}
    ts_status = 'confirmed'
    ts_problems = []
    if len(jstamps) < 2:
        ts_status = 'contradicted'
        ts_problems.append(
            '/joint_states: %d valid stamps — joint clock unusable for the '
            'sim-time cross-check' % len(jstamps))
    if jback > 0:
        ts_status = 'contradicted'
        ts_problems.append(
            '/joint_states: %d backwards stamps (non-monotonic)' % jback)
    for topic in CONTACT_TOPICS:
        arr = tf[topic]['stamps']
        back, dup = arrival_backwards(arr)
        gz_set = set(arr)
        ros_arr = [r['stamp_s'] for r in tf[topic]['ros_recs']
                   if isinstance(r.get('stamp_s'), float)]
        ros_back, ros_dup = arrival_backwards(ros_arr)
        match = (sum(1 for s in ros_arr if s in gz_set) / len(ros_arr)
                 if ros_arr else None)
        ev = {
            'gz_n_valid_stamp': len(arr),
            'gz_backwards': back,
            'gz_duplicates': dup,
            'ros_n_valid_stamp': len(ros_arr),
            'ros_backwards': ros_back,
            'ros_duplicates': ros_dup,
            'fraction_ros_stamps_exact_in_gz': round(match, 6)
            if match is not None else None,
        }
        # Nearest-neighbor statistics on source stamps: for every contact
        # message stamp, |stamp - nearest /joint_states stamp|. Same clock
        # domain check: p95 <= contact association window (pre-registered
        # constant; a different clock domain would show second-scale
        # distances, not millisecond-scale).
        if jstamps and arr:
            dists = sorted(nearest_dist(jstamps, s) for s in arr)
            nn = {
                'n_samples': len(dists),
                'median_s': round(statistics.median(dists), 9),
                'p95_s': round(percentile(dists, 0.95), 9),
                'max_s': round(dists[-1], 9),
                'direction': 'contact stamp -> nearest /joint_states stamp',
            }
            ev['nearest_joint_stamp_dt'] = nn
            same_domain = nn['p95_s'] <= args.contact_window_sec
            ev['clock_domain_check'] = {
                'method': 'p95 of |contact source stamp - nearest '
                          '/joint_states source stamp| <= contact '
                          'association window (REV1.2/R2; source stamps '
                          'only)',
                'threshold_s': args.contact_window_sec,
                'statistic_s': nn['p95_s'],
                'same_clock_domain': same_domain,
            }
            if not same_domain:
                ts_status = 'contradicted'
                ts_problems.append(
                    '%s: p95 nearest-stamp difference %s s > %s s — contact '
                    'stamps are not in the /joint_states simulation clock '
                    'domain' % (topic, nn['p95_s'],
                                args.contact_window_sec))
        else:
            ev['nearest_joint_stamp_dt'] = None
            ev['clock_domain_check'] = {
                'method': 'nearest-neighbor on source stamps (REV1.2/R2)',
                'same_clock_domain': None,
                'note': 'not measurable: contact stamps = %d, joint stamps '
                        '= %d' % (len(arr), len(jstamps)),
            }
        if back > 0:
            ts_status = 'contradicted'
            ts_problems.append(
                '%s: %d backwards stamps on the gz native path' % (
                    topic, back))
        if ros_arr and ros_back > 0:
            ts_status = 'contradicted'
            ts_problems.append(
                '%s: %d backwards stamps on the ROS bridge path' % (
                    topic, ros_back))
        if match is not None and match < STAMP_ENVELOPE_FRACTION_MIN:
            ts_status = 'contradicted'
            ts_problems.append(
                '%s: only %s of ROS bridge stamps exactly match a gz native '
                'stamp — bridge does not preserve source stamps' % (
                    topic, round(match, 6)))
        ts_ev[topic] = ev
    criteria.append(_criterion(
        'timestamp_semantics',
        'Timestamp-domain validation on source stamps (REV1.2/R2): same '
        'simulation clock domain as /joint_states via nearest-neighbor '
        'statistics (median / p95 / max), monotonicity, measured dt, and '
        'exact-stamp preservation across the ROS bridge; callback arrival '
        'time is not valid measurement time and subscriber capture-window '
        'envelope overlap is not a criterion', True, ts_status, ts_ev,
        '; '.join(ts_problems)))

    # --- criterion 5: frame/reference (plausibility only, not blocking) ---
    def ankle_fz_list(topic):
        out = []
        for r in tf[topic]['gz_recs']:
            stamp = r.get('stamp_s')
            got = None
            for c in r['contacts']:
                for w in (c.get('wrenches') or []):
                    b1 = str(w.get('body_1_name') or '')
                    b2 = str(w.get('body_2_name') or '')
                    if 'ankle' in b1.lower():
                        f = w.get('body_1_force')
                        if f:
                            got = (f[2], b1)
                            break
                    if 'ankle' in b2.lower():
                        f = w.get('body_2_force')
                        if f:
                            got = (f[2], b2)
                            break
                if got:
                    break
            if got and stamp is not None:
                out.append((stamp, got[0], got[1]))
        return out

    left_fz = sorted(ankle_fz_list(LEFT_CONTACT_TOPIC))
    right_fz = sorted(ankle_fz_list(RIGHT_CONTACT_TOPIC))
    right_stamps = [x[0] for x in right_fz]
    pairs = []
    for t, fl, _bn in left_fz:
        i = bisect.bisect_right(right_stamps, t)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(right_fz):
                d = abs(right_fz[j][0] - t)
                if d <= POSE_ASSOC_WINDOW_SEC and (
                        best is None or d < best[0]):
                    best = (d, right_fz[j][1])
        if best is not None:
            pairs.append((t, fl, best[1]))
    fr_ev = {
        'n_left_ankle_wrench_samples': len(left_fz),
        'n_right_ankle_wrench_samples': len(right_fz),
        'n_matched_pairs': len(pairs),
        'pair_match_window_s': POSE_ASSOC_WINDOW_SEC,
        'left_body_names': sorted({bn for _, _, bn in left_fz}),
        'right_body_names': sorted({bn for _, _, bn in right_fz}),
        'median_fz_left_newtons': round(
            statistics.median([f for _, f, _ in left_fz]), 6)
        if left_fz else None,
        'median_fz_right_newtons': round(
            statistics.median([f for _, f, _ in right_fz]), 6)
        if right_fz else None,
    }
    mass = (provenance.get('sdf_mass') or {}).get('total_mass_kg')
    fr_status = 'unknown'
    if mass and pairs:
        mg = [mass * g for g in G_VALUES]
        sums = [fl + fr for _, fl, fr in pairs]
        ratios = [abs(s) / m for s in sums for m in mg]
        med_ratio = statistics.median(ratios)
        fr_ev['m_g_newtons'] = [round(m, 4) for m in mg]
        fr_ev['median_signed_sum_newtons'] = round(
            statistics.median(sums), 6)
        fr_ev['median_abs_ratio_to_mg'] = round(med_ratio, 6)
        fr_ev['fraction_pairs_ratio_in_0.5_1.5'] = round(
            sum(1 for r in ratios
                if N3L_RATIO_MIN <= r <= N3L_RATIO_MAX) / len(ratios), 6)
        if len(pairs) >= 20 and N3L_RATIO_MIN <= med_ratio <= N3L_RATIO_MAX:
            fr_status = 'plausible'
    criteria.append(_criterion(
        'frame_reference',
        'Frame/reference interpretation of the contact wrench (N3L / '
        'standing equilibrium — plausibility only)', False, fr_status,
        fr_ev,
        'N3L and standing load are plausibility checks only (GPT-5.6 '
        'correction 3): this criterion is never confirmable by this probe; '
        'CoP/derived-moment derivation stays blocked until a '
        'common-frame/common-origin comparison is performed.'))

    # --- determined facts & verdict ---------------------------------------
    determined = [
        {
            'fact': 'contact publication mode per topic (gz native path)',
            'value': {topic: pub_ev[topic]['publication_mode']
                      for topic in CONTACT_TOPICS},
            'consequence': 'periodic cadence implies messages continue '
                           'during no-contact; empty-message behavior must '
                           'still be re-confirmed during the acquisition '
                           'pilot (fail-closed three-state, RFC § 3.6.4)',
        },
        {
            'fact': 'no-contact observability within the standing window',
            'value': no_contact_modes,
            'consequence': 'VALID_NO_CONTACT may only be emitted when empty '
                           'Contacts messages are confirmed on the native '
                           'path; otherwise the collector must emit UNKNOWN '
                           '(fail-closed)',
        },
        {
            'fact': 'verbatim collision entity names per topic',
            'value': {topic: [p['pair'] for p in
                              body_ev[topic]['collision_pairs_top']]
                      for topic in CONTACT_TOPICS},
            'consequence': 'collector association filters use these verbatim '
                           'names, not the scoped-name guess',
        },
        {
            'fact': 'wrench body entity names per topic',
            'value': {topic: [w['name'] for w in
                              body_ev[topic]['wrench_body_names_top']]
                      for topic in CONTACT_TOPICS},
            'consequence': 'ankle-side identification for CoP uses the body '
                           'name containing "ankle"',
        },
        {
            'fact': 'ROS bridge exact-stamp preservation fraction',
            'value': {topic: ts_ev[topic].get(
                'fraction_ros_stamps_exact_in_gz')
                for topic in CONTACT_TOPICS},
            'consequence': 'both paths share the gz source sim-time domain '
                           'when this fraction is high',
        },
        {
            'fact': 'contact/joint clock-domain agreement '
                    '(nearest-neighbor source-stamp statistics, REV1.2/R2)',
            'value': {topic: {
                'median_s': (ts_ev[topic].get('nearest_joint_stamp_dt')
                             or {}).get('median_s'),
                'p95_s': (ts_ev[topic].get('nearest_joint_stamp_dt')
                          or {}).get('p95_s'),
                'max_s': (ts_ev[topic].get('nearest_joint_stamp_dt')
                          or {}).get('max_s'),
            } for topic in CONTACT_TOPICS},
            'consequence': 'millisecond-scale nearest-stamp distances '
                           'establish the same simulation clock domain on '
                           'source stamps; the collector aligns on source '
                           'stamps only, never on callback arrival time',
        },
    ]
    if pairs and fr_ev.get('median_signed_sum_newtons') is not None:
        determined.append({
            'fact': 'contact wrench force sign convention '
                    '(median signed foot fz sum vs m*g)',
            'value': fr_ev['median_signed_sum_newtons'],
            'consequence': 'close to +m*g indicates upward ground reaction '
                           'on the foot; close to -m*g indicates inverted '
                           'sign; the collector records signed values '
                           'verbatim and never flips signs',
        })
    blocking_failures = [c['id'] for c in criteria
                         if c['blocking'] and c['status'] != 'confirmed']
    consequences = []
    if blocking_failures:
        consequences.append(
            'acquisition blocked until the RFC is revised (RFC App. B V-1 '
            'failure conditions)')
    else:
        consequences.append(
            'raw contact acquisition feasible on the gz-transport native '
            'path (lossless); see bridge_field_loss for the ROS bridge '
            'field loss')
    consequences.append(
        'frame/reference determination stays at plausibility level; '
        'CoP/derived-moment derivation remains blocked pending a '
        'common-frame comparison (RFC App. B V-1)')
    verdict = {
        'probe': 'V-1',
        'status': 'FAIL' if blocking_failures else 'PASS',
        'blocking_failures': blocking_failures,
        'non_blocking_statuses': {c['id']: c['status'] for c in criteria
                                  if not c['blocking']},
        'consequences': consequences,
    }
    return {'criteria': criteria, 'determined_facts': determined,
            'verdict': verdict}


# ---------------------------------------------------------------------------
# V-1 entry point.
# ---------------------------------------------------------------------------

def run_v1(args, env, provenance):
    """Run the V-1 collection and analysis; return the full report dict.

    Raises RuntimeError when the gz-transport native path is unavailable:
    the lossless-path facts (world, force_offset, per-contact header) cannot
    be determined without it, so V-1 fails closed instead of reporting
    bridge-only facts (GPT-5.6 corrections 2 and 5).
    """
    contacts_pb2, pose_v_pb2, transport = gz_packages()
    (_rclpy, RosContacts, RosContact, RosJointWrench, RosJointState,
     RosPoseStamped) = ros_packages()
    if contacts_pb2 is None or transport is None:
        raise RuntimeError(
            'gz python bindings (gz.msgs* / gz.transport*) not importable — '
            'the gz-transport native contact path cannot be probed; V-1 '
            'fails closed instead of assuming bridge-only facts')
    spec = [
        (gz_kind(LEFT_CONTACT_TOPIC), 'gz', LEFT_CONTACT_TOPIC,
         contacts_pb2.Contacts, gz_contact_record),
        (gz_kind(RIGHT_CONTACT_TOPIC), 'gz', RIGHT_CONTACT_TOPIC,
         contacts_pb2.Contacts, gz_contact_record),
        (ros_kind(LEFT_CONTACT_TOPIC), 'ros', LEFT_CONTACT_TOPIC,
         RosContacts, ros_contact_record),
        (ros_kind(RIGHT_CONTACT_TOPIC), 'ros', RIGHT_CONTACT_TOPIC,
         RosContacts, ros_contact_record),
        ('joint_states', 'ros', JOINT_STATES_TOPIC, RosJointState,
         joint_state_record),
    ]
    collected = run_collection('v1', spec, args.duration, args.warmup_sec,
                               transport_module=transport)
    analysis = analyze_v1(collected, args, provenance)
    schemas = message_schemas(
        contacts_pb2, pose_v_pb2, RosContact, RosJointWrench, RosContacts,
        RosJointState, RosPoseStamped)
    loss = bridge_field_loss(RosContact, RosJointWrench)
    samples = build_samples(collected['records'], args.max_samples)
    return build_report('V-1', args, env, provenance, collected, analysis,
                        samples, schemas=schemas, bridge_loss=loss)


# ---------------------------------------------------------------------------
# V-2: /world/nao_world/pose/info probe (gz native pose path + joint clock +
# derived /body_pose cross-check).
# ---------------------------------------------------------------------------

def arrival_backwards(seq):
    """(backwards, duplicates) counts over an arrival-ordered stamp list."""
    back = dup = 0
    for a, b in zip(seq, seq[1:]):
        if b < a:
            back += 1
        elif b == a:
            dup += 1
    return back, dup


def _quat_normalized(q):
    if not q:
        return None
    norm = math.sqrt(sum(v * v for v in q))
    if not norm or not math.isfinite(norm):
        return None
    return [v / norm for v in q]


def _quat_mul(a, b):
    """Hamilton product a (x) b for [x, y, z, w] quaternions."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def _quat_rot(q, v):
    """Rotate vector v by [x, y, z, w] quaternion q (q assumed unit)."""
    x, y, z, w = q
    # t = 2 (q_vec x v); v' = v + w t + q_vec x t
    tx = 2.0 * (y * v[2] - z * v[1])
    ty = 2.0 * (z * v[0] - x * v[2])
    tz = 2.0 * (x * v[1] - y * v[0])
    return [
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx),
    ]


def compose_world_pose(model, link):
    """World pose of a link entry composed with the model entry of the same
    Pose_V message: p_w = p_model + R(q_model) . p_rel,
    q_w = q_model (x) q_rel. Quaternions are [x, y, z, w] (gz.msgs.Pose)."""
    pm, pl = model['position'], link['position']
    qm = _quat_normalized(model.get('orientation'))
    ql = _quat_normalized(link.get('orientation'))
    if qm is None:
        p_w = [pm[k] + pl[k] for k in range(3)]
    else:
        rp = _quat_rot(qm, pl)
        p_w = [pm[k] + rp[k] for k in range(3)]
    if qm is not None and ql is not None:
        q_w = _quat_mul(qm, ql)
    else:
        q_w = qm if qm is not None else ql
    return {'position': p_w, 'orientation': q_w}


def _identity_deviation(tr):
    """Max deviation of a tracked pose from identity (position [0,0,0],
    quaternion [0,0,0,1]; -q counts as identity too). None if no pose."""
    if not tr or not tr.get('position'):
        return None
    d = max(abs(v) for v in tr['position'])
    qn = _quat_normalized(tr.get('orientation'))
    if qn is not None:
        dq = min(max(abs(qn[0]), abs(qn[1]), abs(qn[2]), abs(qn[3] - 1.0)),
                 max(abs(qn[0]), abs(qn[1]), abs(qn[2]), abs(qn[3] + 1.0)))
        d = max(d, dq)
    return d


def _quat_component_diff(q1, q2):
    """Max component diff of two quaternions after normalization
    (q and -q are the same rotation: min over the sign choice)."""
    a, b = _quat_normalized(q1), _quat_normalized(q2)
    if a is None or b is None:
        return None
    return min(max(abs(a[k] - b[k]) for k in range(4)),
               max(abs(a[k] + b[k]) for k in range(4)))


def analyze_v2(collected, args, provenance):
    records = collected['records']
    streams = collected['streams']
    criteria = []

    pose_recs = [r for r in records.get('pose_info', [])
                 if isinstance(r.get('stamp_s'), float)]
    joint_recs = records.get('joint_states', [])
    joint_arr = [r['stamp_s'] for r in joint_recs
                 if isinstance(r.get('stamp_s'), float)]
    body_recs = [r for r in records.get('body_pose', [])
                 if isinstance(r.get('stamp_s'), float)]

    # --- criterion 1: unique entities (blocking) --------------------------
    name_counts = collections.Counter()
    name_ids = collections.defaultdict(set)
    n_entries_total = 0
    coverage_per_message = []
    for r in pose_recs:
        entries = r.get('entries') or []
        n_entries_total += len(entries)
        locals_in_msg = set()
        for nm, eid in entries:
            if isinstance(nm, str):
                name_counts[nm] += 1
                name_ids[nm].add(eid)
                locals_in_msg.add(local_name(nm))
        coverage_per_message.append(
            sum(1 for l in EXPECTED_LINKS if l in locals_in_msg))
    n_msgs = len(pose_recs)
    verbatim_names = sorted(name_counts)
    scoped = [n for n in verbatim_names if '::' in n]
    if not verbatim_names:
        naming = 'not_observable'
    elif not scoped:
        naming = 'bare_names'
    elif len(scoped) == len(verbatim_names):
        naming = 'scoped_names'
    else:
        naming = 'mixed'
    link_candidates = {link: [n for n in verbatim_names
                              if local_name(n) == link]
                       for link in EXPECTED_LINKS}
    missing_links = [l for l in EXPECTED_LINKS if not link_candidates[l]]
    ambiguous_links = {l: c for l, c in link_candidates.items() if len(c) > 1}
    ids_unstable = {n: sorted(i for i in v if i is not None)
                    for n, v in name_ids.items() if len(v) > 1}
    frac_all = (sum(1 for c in coverage_per_message
                    if c == len(EXPECTED_LINKS)) / n_msgs
                if n_msgs else None)
    ue_ev = {
        'n_messages': n_msgs,
        'n_entries_total': n_entries_total,
        'n_unique_verbatim_names': len(verbatim_names),
        'verbatim_names': verbatim_names,
        'local_name_union': sorted({local_name(n)
                                    for n in verbatim_names}),
        'naming_convention': naming,
        'model_entry_nao_present': 'nao' in name_counts,
        'ground_entry_present': any(
            'ground' in local_name(n).lower() for n in verbatim_names),
        'missing_expected_links': missing_links,
        'ambiguous_expected_links': ambiguous_links,
        'ids_unstable': ids_unstable,
        'fraction_messages_with_all_25_links': round(frac_all, 6)
        if frac_all is not None else None,
        'per_message_link_coverage_min': min(coverage_per_message)
        if coverage_per_message else None,
        'rfc_reference': 'RFC App. B V-2 criterion 1 — "25 strings '
                         'occurring in Pose_V is not sufficient"; '
                         'exactly-named unique entries required',
    }
    ue_status = 'confirmed'
    ue_problems = []
    if n_msgs == 0:
        ue_status = 'unknown'
        ue_problems.append(
            'pose/info received 0 valid messages — entity inventory not '
            'determinable')
    else:
        if missing_links:
            ue_status = 'contradicted'
            ue_problems.append('missing expected links: %s'
                               % ', '.join(missing_links))
        if ambiguous_links:
            ue_status = 'contradicted'
            ue_problems.append(
                'ambiguous (non-unique) expected links: %s'
                % ', '.join(sorted(ambiguous_links)))
        if ids_unstable:
            ue_status = 'contradicted'
            ue_problems.append('%d entity names with unstable ids'
                               % len(ids_unstable))
        if frac_all is not None and frac_all < ALL_LINKS_PER_MESSAGE_MIN:
            ue_status = 'contradicted'
            ue_problems.append(
                'fraction of messages carrying all 25 link poses = %s < %s '
                '(RFC: one message must carry the full 25-link snapshot)'
                % (round(frac_all, 6), ALL_LINKS_PER_MESSAGE_MIN))
        if 'nao' not in name_counts:
            ue_status = 'contradicted'
            ue_problems.append(
                "model entry 'nao' not found in pose/info "
                '(body_pose_converter selects pose.name() == "nao")')
    criteria.append(_criterion(
        'unique_entities',
        'All 25 nao_simple.sdf links appear as unique, exactly-named '
        'entities in /world/nao_world/pose/info (model entry "nao" '
        'included)', True, ue_status, ue_ev, '; '.join(ue_problems)))

    # --- criterion 2: pose frame semantics (blocking) --------------------
    # REV1.2/R4: RFC App. B V-2.2 is corrected — Pose_V link entries are
    # parent-frame (model-relative) poses, never world poses; world poses
    # are derived by composition with the nao model entry. The criterion
    # REPORTS the frame semantics as model-relative, never as world pose,
    # and is determined, never assumed:
    #   (a) link-entry frame semantics from in-band evidence;
    #   (b) /body_pose exact-stamp cross-check — validates the nao model
    #       entry is the world pose (the composition anchor);
    #   (c) identity-link composition consistency — base_link IS the model
    #       frame, so composing the nao entry with the base_link entry
    #       reproduces the nao entry. REV1.2/R5: this is a CONSISTENCY
    #       CHECK ONLY, not an end-to-end validation — the composition
    #       method requires verification using selected non-identity links;
    #   (d) independent cross-check against dynamic_pose/info;
    #   (e) fixed standing-geometry sanity — informational only (REV1.2:
    #       applies in standing scenarios; walk-scenario swing feet are
    #       expected outside the envelope).
    pose_by_stamp = {}
    for r in pose_recs:
        tr = (r.get('tracked') or {}).get('nao')
        if tr is not None and tr.get('position') is not None:
            pose_by_stamp[r['stamp_s']] = tr
    frame_ids = collections.Counter(r.get('frame_id') for r in body_recs)
    matched = unmatched = 0
    pos_diffs, quat_diffs = [], []
    for r in body_recs:
        tr = pose_by_stamp.get(r['stamp_s'])
        p, q = r.get('position'), r.get('orientation')
        if tr is None or not p:
            unmatched += 1
            continue
        matched += 1
        pos_diffs.append(max(abs(p[k] - tr['position'][k]) for k in range(3)))
        qz = _quat_normalized(q)
        qt = _quat_normalized(tr.get('orientation'))
        if qz and qt:
            quat_diffs.append(max(abs(qz[k] - qt[k]) for k in range(4)))

    def tracked_rows(key):
        rows = [(r.get('tracked') or {}).get(key) for r in pose_recs]
        return [t for t in rows if t and t.get('position')]

    def composed_rows(key):
        """World poses of `key` composed with the same message's nao entry."""
        rows = []
        for r in pose_recs:
            tr = r.get('tracked') or {}
            m, l = tr.get('nao'), tr.get(key)
            if m and l and m.get('position') and l.get('position'):
                w = compose_world_pose(m, l)
                w['stamp_s'] = r.get('stamp_s')
                rows.append(w)
        return rows

    def median_pos(rows):
        if not rows:
            return None
        return [statistics.median([t['position'][k] for t in rows])
                for k in range(3)]

    def standing_rows_check(rows, xy_max, z_lo, z_hi, source):
        if not rows:
            return {'n': 0, 'source': source,
                    'fraction_within_standing_envelope': None}
        ok = sum(1 for t in rows
                 if z_lo <= t['position'][2] <= z_hi
                 and max(abs(t['position'][0]),
                         abs(t['position'][1])) <= xy_max)
        return {
            'n': len(rows),
            'source': source,
            'median_position_m': median_pos(rows),
            'fraction_within_standing_envelope': round(ok / len(rows), 6),
        }

    # (a) link-entry frame semantics, from in-band evidence only. base_link
    # is the model frame (nao_simple.sdf gives base_link no <pose> tag), so
    # a world-frame base_link entry would have to carry the nao model-entry
    # world pose; an identity base_link entry alongside a moving nao entry
    # is only consistent with parent-frame link entries.
    n_sem = base_identity = nao_identity = base_matches_nao = 0
    ankle_rows_n = ankle_below_ground = 0
    for r in pose_recs:
        tr = r.get('tracked') or {}
        b, m, la = tr.get('base_link'), tr.get('nao'), tr.get('l_ankle')
        if not (b and m and b.get('position') and m.get('position')):
            continue
        n_sem += 1
        # Explicit None checks: _identity_deviation returns exactly 0.0 for
        # an identity pose, which is falsy — the historical `(... or 1.0)`
        # form counted every identity pose as non-identity. None (missing
        # pose) is the only non-identity-indicating sentinel.
        d_b = _identity_deviation(b)
        if d_b is not None and d_b <= FRAME_IDENTITY_TOL:
            base_identity += 1
        d_m = _identity_deviation(m)
        if d_m is not None and d_m <= FRAME_IDENTITY_TOL:
            nao_identity += 1
        d_q = _quat_component_diff(b.get('orientation'),
                                   m.get('orientation'))
        if max(abs(b['position'][k] - m['position'][k])
               for k in range(3)) <= FRAME_IDENTITY_TOL \
                and d_q is not None and d_q <= FRAME_IDENTITY_TOL:
            base_matches_nao += 1
        if la and la.get('position'):
            ankle_rows_n += 1
            if la['position'][2] < -0.10:
                # a world-pose ankle below the ground plane is impossible
                # while the robot stands on the ground — parent-frame proof
                ankle_below_ground += 1
    base_identity_frac = (base_identity / n_sem) if n_sem else None
    nao_identity_frac = (nao_identity / n_sem) if n_sem else None
    base_matches_frac = (base_matches_nao / n_sem) if n_sem else None
    ankle_below_frac = (ankle_below_ground / ankle_rows_n) \
        if ankle_rows_n else None
    if n_sem == 0:
        frame_sem = 'not_observable'
    elif (base_identity_frac or 0.0) >= FRAME_SEMANTICS_FRACTION_MIN \
            and nao_identity_frac is not None \
            and nao_identity_frac <= 0.5:
        # nao_identity_frac == 0.0 means the model entry moves, which is
        # the normal state; the historical `(nao_identity_frac or 1.0)`
        # form made this branch unreachable whenever the nao entry moved.
        frame_sem = 'model_relative_for_links'
    elif (base_matches_frac or 0.0) >= FRAME_SEMANTICS_FRACTION_MIN \
            and nao_identity_frac is not None \
            and nao_identity_frac <= 0.5:
        frame_sem = 'world_frame_for_links'
    elif (base_identity_frac or 0.0) >= FRAME_SEMANTICS_FRACTION_MIN \
            and (ankle_below_frac or 0.0) >= 0.5:
        # model entry parked at the world origin; the below-ground raw
        # ankle still discriminates parent-relative vs world semantics.
        frame_sem = 'model_relative_for_links'
    else:
        frame_sem = 'indeterminate'
    frame_sem_ev = {
        'n_messages_evaluated': n_sem,
        'base_link_identity_fraction': round(base_identity_frac, 6)
        if base_identity_frac is not None else None,
        'nao_entry_identity_fraction': round(nao_identity_frac, 6)
        if nao_identity_frac is not None else None,
        'base_link_matches_nao_entry_fraction': round(base_matches_frac, 6)
        if base_matches_frac is not None else None,
        'l_ankle_raw_below_ground_fraction': round(ankle_below_frac, 6)
        if ankle_below_frac is not None else None,
        'identity_tolerance': FRAME_IDENTITY_TOL,
        'fraction_min': FRAME_SEMANTICS_FRACTION_MIN,
        'link_entry_frame_semantics': frame_sem,
        'rfc_expectation': 'model_relative_for_links (RFC REV1.2/R4) — '
                           'entries are reported as parent/model-relative, '
                           'never as world pose',
        'discriminator': 'base_link is the model frame (no <pose> in '
                         'nao_simple.sdf), so a world-frame base_link entry '
                         'must equal the nao model-entry world pose; an '
                         'identity base_link entry with a moving nao entry '
                         'proves parent-frame link entries',
        'run1_evidence': 'run 1 (fallen robot, *.run1_fallen.json): '
                         'base_link == [0, 0, 0] exactly on every message '
                         'while the nao entry was [0.2616, 0, 0.1141]; '
                         'l_ankle kept the SDF hip y-offset 0.05 and the '
                         'fully-extended leg geometry — parent-frame FK, '
                         'not a world pose',
    }

    # (d) independent cross-check: dynamic_pose/info (the SceneBroadcaster
    # dynamic-entity pose stream) vs the raw link entries and vs the
    # composed world poses. Informational and non-blocking — it determines
    # whether gz offers a native world-pose stream the collector could use
    # instead of composing. The quasi-static stand makes small stamp
    # mismatches harmless.
    dyn_recs = [r for r in records.get('dynamic_pose_info', [])
                if isinstance(r.get('stamp_s'), float)]
    dyn_by_stamp = {}
    for r in dyn_recs:
        dyn_by_stamp.setdefault(r['stamp_s'], r)
    dyn_sorted = sorted(dyn_by_stamp)
    pose_stamps = sorted(r['stamp_s'] for r in pose_recs)
    pose_dts = [b - a for a, b in zip(pose_stamps, pose_stamps[1:])]
    dyn_tol = (0.5 * statistics.median(pose_dts)) if pose_dts else 0.01
    dyn_matched = 0
    dyn_stamp_diffs = []
    dyn_raw_diffs = collections.defaultdict(list)
    dyn_comp_diffs = collections.defaultdict(list)

    def dyn_nearest(t):
        if not dyn_sorted:
            return None
        i = bisect.bisect_left(dyn_sorted, t)
        best, best_d = None, None
        for j in (i - 1, i):
            if 0 <= j < len(dyn_sorted):
                d = abs(dyn_sorted[j] - t)
                if best_d is None or d < best_d:
                    best, best_d = dyn_sorted[j], d
        return best, best_d

    for r in pose_recs:
        tr = r.get('tracked') or {}
        m = tr.get('nao')
        if not (m and m.get('position')):
            continue
        hit = dyn_by_stamp.get(r['stamp_s'])
        if hit is None:
            near = dyn_nearest(r['stamp_s'])
            if near is None or near[1] > dyn_tol:
                continue
            hit = dyn_by_stamp[near[0]]
            dyn_stamp_diffs.append(near[1])
        else:
            dyn_stamp_diffs.append(0.0)
        dyn_matched += 1
        dtr = hit.get('tracked') or {}
        for key in ('base_link', 'l_ankle', 'r_ankle'):
            l, dl = tr.get(key), dtr.get(key)
            if not (l and dl and l.get('position') and dl.get('position')):
                continue
            dyn_raw_diffs[key].append(max(
                abs(l['position'][k] - dl['position'][k])
                for k in range(3)))
            w = compose_world_pose(m, l)
            dyn_comp_diffs[key].append(max(
                abs(w['position'][k] - dl['position'][k])
                for k in range(3)))

    def _maxv(d):
        return max(d) if d else None
    comp_max = max((_maxv(v) for v in dyn_comp_diffs.values()), default=None)
    raw_max = max((_maxv(v) for v in dyn_raw_diffs.values()), default=None)
    if dyn_matched and comp_max is not None:
        if comp_max <= DYNAMIC_MATCH_MAX_DIFF_M \
                and (raw_max is None
                     or raw_max > 10 * DYNAMIC_MATCH_MAX_DIFF_M):
            dyn_sem = 'matches_composed_world_pose'
        elif raw_max is not None and raw_max <= DYNAMIC_MATCH_MAX_DIFF_M \
                and comp_max > 10 * DYNAMIC_MATCH_MAX_DIFF_M:
            dyn_sem = 'matches_raw_parent_relative'
        else:
            dyn_sem = 'ambiguous'
    elif dyn_matched:
        dyn_sem = 'not_comparable'
    else:
        dyn_sem = 'not_observable'
    dyn_ev = {
        'topic': DYNAMIC_POSE_TOPIC,
        'n_messages': len(dyn_recs),
        'dt': dt_stats(sorted(r['stamp_s'] for r in dyn_recs))
        if len(dyn_recs) >= 2 else {'n_dt': 0},
        'n_stamp_matched': dyn_matched,
        'stamp_match_tolerance_s': round(dyn_tol, 9),
        'max_abs_stamp_diff_s': max(dyn_stamp_diffs)
        if dyn_stamp_diffs else None,
        'max_position_diff_vs_raw_link_entry_m': {
            k: _maxv(v) for k, v in sorted(dyn_raw_diffs.items())},
        'max_position_diff_vs_composed_world_m': {
            k: _maxv(v) for k, v in sorted(dyn_comp_diffs.items())},
        'dynamic_entry_semantics': dyn_sem,
        'role': 'informational cross-check (non-blocking) — determines '
                'whether dynamic_pose/info is a native world-pose source '
                'the collector could use instead of composing pose/info',
    }

    model_link_deltas = []
    for r in pose_recs:
        tr = r.get('tracked') or {}
        a, b = tr.get('nao'), tr.get('base_link')
        if a and b and a.get('position') and b.get('position'):
            model_link_deltas.append(
                [a['position'][k] - b['position'][k] for k in range(3)])
    ground_rows = tracked_rows('ground_plane')
    ground_ev = {
        'n': len(ground_rows),
        'median_z_m': statistics.median(
            [t['position'][2] for t in ground_rows])
        if ground_rows else None,
        'fraction_abs_z_within_tolerance': round(
            sum(1 for t in ground_rows
                if abs(t['position'][2]) <= GROUND_Z_MAX_ABS_M)
            / len(ground_rows), 6) if ground_rows else None,
        'tolerance_abs_z_m': GROUND_Z_MAX_ABS_M,
    }
    match_frac = (matched / len(body_recs)) if body_recs else None
    max_pos = max(pos_diffs) if pos_diffs else None
    max_quat = max(quat_diffs) if quat_diffs else None

    # (c) composition evidence. base_link IS the model frame (nao_simple.sdf
    # gives base_link no <pose>), so composing the nao entry with the
    # base_link entry must reproduce the nao entry exactly. REV1.2/R5: this
    # identity-link comparison is a CONSISTENCY CHECK ONLY — it cannot
    # exercise the rotation term (identity link), so it is not an
    # end-to-end validation; the composition method requires verification
    # using selected non-identity links.
    base_comp_vs_nao = []
    for r in pose_recs:
        tr = r.get('tracked') or {}
        m, b = tr.get('nao'), tr.get('base_link')
        if m and b and m.get('position') and b.get('position'):
            w = compose_world_pose(m, b)
            base_comp_vs_nao.append(max(
                abs(w['position'][k] - m['position'][k]) for k in range(3)))
    composition_ev = {
        'formula': 'p_world = p_nao + R(q_nao) . p_link_entry; '
                   'q_world = q_nao (x) q_link_entry; quaternions are '
                   '[x, y, z, w] (gz.msgs.Pose convention)',
        'n_messages_with_model_and_base_link': len(base_comp_vs_nao),
        'composed_base_link_vs_nao_entry_max_diff_m':
            max(base_comp_vs_nao) if base_comp_vs_nao else None,
        'base_link_composition_tolerance_m': BODY_POSE_MATCH_MAX_DIFF_M,
        'l_ankle_raw_median_position_m': median_pos(tracked_rows('l_ankle')),
        'l_ankle_composed_median_position_m':
            median_pos(composed_rows('l_ankle')),
        'r_ankle_composed_median_position_m':
            median_pos(composed_rows('r_ankle')),
        'note': 'the raw l_ankle median documents parent-frame values (SDF '
                'hip y-offset 0.05, leg FK); the composed medians are the '
                'recovered world poses; REV1.2/R5: the base_link comparison '
                'is an identity-link CONSISTENCY CHECK ONLY (base_link IS '
                'the model frame, so the rotation term is never exercised) '
                '— not an end-to-end validation; the composition method '
                'requires verification using selected non-identity links '
                '(e.g. l_ankle / r_ankle) before composed poses are treated '
                'as validated',
    }

    # (e) fixed standing-geometry sanity, evaluated on world poses under
    # the DETERMINED frame semantics — raw link entries only if they were
    # proven world-frame; composed (nao entry (x) link entry) otherwise.
    def _standing_range(key):
        if key == 'base_link':
            return STANDING_BASE_Z_RANGE
        return STANDING_ANKLE_Z_RANGE

    if frame_sem == 'world_frame_for_links':
        standing_geometry = {
            key: standing_rows_check(
                tracked_rows(key), STANDING_XY_MAX_M, *_standing_range(key),
                'raw link entry (frame semantics: world_frame_for_links)')
            for key in ('base_link', 'l_ankle', 'r_ankle')}
    elif frame_sem == 'model_relative_for_links':
        standing_geometry = {
            key: standing_rows_check(
                composed_rows(key), STANDING_XY_MAX_M, *_standing_range(key),
                'composed world pose (nao entry (x) %s entry)' % key)
            for key in ('base_link', 'l_ankle', 'r_ankle')}
    else:
        standing_geometry = {
            key: {'n': 0,
                  'source': 'not evaluated — link-entry frame semantics '
                            'is %s' % frame_sem,
                  'fraction_within_standing_envelope': None}
            for key in ('base_link', 'l_ankle', 'r_ankle')}

    wp_ev = {
        'body_pose_n_messages': len(body_recs),
        'body_pose_frame_ids': {str(k): v for k, v in frame_ids.items()},
        'pose_info_messages_with_nao_entry': len(pose_by_stamp),
        'n_exact_stamp_matched': matched,
        'n_unmatched': unmatched,
        'fraction_exact_stamp_matched': round(match_frac, 6)
        if match_frac is not None else None,
        'max_position_diff_m': max_pos,
        'median_position_diff_m': statistics.median(pos_diffs)
        if pos_diffs else None,
        'n_quat_compared': len(quat_diffs),
        'max_normalized_quat_component_diff': max_quat,
        'tolerances': {
            'match_fraction_min': BODY_POSE_MATCH_FRACTION_MIN,
            'position_m': BODY_POSE_MATCH_MAX_DIFF_M,
            'quat_component': BODY_POSE_MATCH_MAX_Q_DIFF,
        },
        'body_pose_cross_check_scope': 'the /body_pose exact-stamp check '
            'validates that the nao MODEL entry (the composition anchor) '
            'is the world pose — link entries have no /body_pose '
            'counterpart',
        'frame_semantics': frame_sem_ev,
        'composition': composition_ev,
        'model_entry_nao_vs_base_link_median_delta_m': [
            statistics.median([d[k] for d in model_link_deltas])
            for k in range(3)] if model_link_deltas else None,
        'standing_geometry': standing_geometry,
        'ground_plane': ground_ev,
        'dynamic_pose_info_cross_check': dyn_ev,
    }
    wp_status = 'confirmed'
    wp_problems = []
    # (b) /body_pose exact-stamp cross-check of the nao model entry.
    if not body_recs or not pose_by_stamp:
        if wp_status == 'confirmed':
            wp_status = 'unknown'
        wp_problems.append(
            'cross-check not observable: body_pose = %d messages, '
            'pose/info nao entries = %d' % (len(body_recs),
                                            len(pose_by_stamp)))
    else:
        if match_frac < BODY_POSE_MATCH_FRACTION_MIN:
            wp_status = 'contradicted'
            wp_problems.append(
                'exact-stamp match fraction %s < %s — /body_pose is not '
                'the same-stamp projection of pose/info'
                % (round(match_frac, 6), BODY_POSE_MATCH_FRACTION_MIN))
        if max_pos is None or max_pos > BODY_POSE_MATCH_MAX_DIFF_M:
            wp_status = 'contradicted'
            wp_problems.append(
                'matched-pair max position difference %s m exceeds %s m'
                % (max_pos, BODY_POSE_MATCH_MAX_DIFF_M))
        if quat_diffs and max_quat > BODY_POSE_MATCH_MAX_Q_DIFF:
            wp_status = 'contradicted'
            wp_problems.append(
                'matched-pair max normalized quaternion component '
                'difference %s exceeds %s'
                % (max_quat, BODY_POSE_MATCH_MAX_Q_DIFF))
        if body_recs and 'world' not in frame_ids:
            wp_status = 'contradicted'
            wp_problems.append(
                '/body_pose frame_id is not "world" (observed: %s)'
                % sorted(str(k) for k in frame_ids))
    # (a) frame semantics per RFC REV1.2/R4, determined from evidence. The
    # corrected contract expects parent/model-relative link entries; they
    # are reported as model-relative, never as world pose. Fail-closed
    # semantics: world-frame link entries contradict the REV1.2 contract
    # (the composition recovery path assumes model-relative entries).
    if frame_sem == 'model_relative_for_links':
        wp_problems.append(
            'link entries are parent-frame (model-relative) poses — the '
            'REV1.2 contract semantics (reported as model-relative, never '
            'as world pose); world poses are derived by composing the nao '
            'model entry with each link entry (same Pose_V message)')
    elif frame_sem == 'world_frame_for_links':
        wp_status = 'contradicted'
        wp_problems.append(
            'link entries carry world-frame poses — this falsifies the '
            'REV1.2 contract (entries must be parent/model-relative; the '
            'composition recovery path would be invalid)')
    else:
        if wp_status == 'confirmed':
            wp_status = 'unknown'
        wp_problems.append(
            'link-entry frame semantics is %s — the REV1.2 frame-semantics '
            'criterion cannot be confirmed or refuted' % frame_sem)
    # (c) identity-link composition consistency (only meaningful under
    # parent-frame semantics, where composition is the recovery path).
    # REV1.2/R5: consistency check, NOT a validation.
    if frame_sem == 'model_relative_for_links' \
            and base_comp_vs_nao \
            and max(base_comp_vs_nao) > BODY_POSE_MATCH_MAX_DIFF_M:
        wp_status = 'contradicted'
        wp_problems.append(
            'composed base_link world pose deviates from the nao entry by '
            '%s m (> %s m) — identity-link composition consistency fails '
            'on this run' % (max(base_comp_vs_nao),
                             BODY_POSE_MATCH_MAX_DIFF_M))
    # (e) fixed standing-geometry / ground sanity — informational only
    # (REV1.2): the envelope applies to standing scenarios; walk-scenario
    # swing feet are expected outside the ankle envelope by design. Values
    # are recorded as evidence, not as blocking status.
    standing_notes = []
    for key, chk in standing_geometry.items():
        frac = chk.get('fraction_within_standing_envelope')
        if frac is None:
            standing_notes.append(
                '%s pose not observable in tracked pose/info entries'
                % key)
        elif frac < POPULATED_FRACTION_MIN:
            standing_notes.append(
                'informational (non-blocking, REV1.2): %s '
                'standing-geometry envelope fraction %s < %s — expected '
                'outside standing scenarios (source: %s)'
                % (key, frac, POPULATED_FRACTION_MIN, chk.get('source')))
    if ground_rows and \
            ground_ev['fraction_abs_z_within_tolerance'] \
            < POPULATED_FRACTION_MIN:
        standing_notes.append(
            'informational (non-blocking, REV1.2): ground_plane |z| '
            'fraction %s < %s' % (
                ground_ev['fraction_abs_z_within_tolerance'],
                POPULATED_FRACTION_MIN))
    wp_ev['informational_checks'] = {
        'standing_geometry': 'informational (REV1.2): standing-scenario '
                             'sanity on composed world poses; walk-scenario '
                             'swing feet are expected outside the ankle '
                             'envelope — not a blocking check',
        'ground_plane': 'informational (REV1.2): static-entity sanity on '
                        'the world frame',
        'notes': standing_notes,
    }
    criteria.append(_criterion(
        'pose_frame_semantics',
        'RFC REV1.2 App. B V-2.2 (R4): pose/info link entries are parent/'
        'model-relative poses — reported as model-relative, never as world '
        'pose; world poses are derived by composition '
        'T_world_link = T_world_model . T_model_link; the nao model entry '
        '(composition anchor) is cross-checked against /body_pose '
        '(exact-stamp); identity-link composition consistency is recorded '
        '(R5: consistency check only — the composition method requires '
        'verification using selected non-identity links)',
        True, wp_status, wp_ev,
        '; '.join(wp_problems + standing_notes)))

    # --- criterion 3: source stamps & measured cadence (blocking) --------
    sc_ev, sc_problems = {}, []
    sc_status = 'confirmed'
    stamp_lists = {
        'pose_info': [r['stamp_s'] for r in pose_recs],
        'joint_states': joint_arr,
        'body_pose': [r['stamp_s'] for r in body_recs],
    }
    for kind, arr in stamp_lists.items():
        back, dup = arrival_backwards(arr)
        ev = {
            'n_valid_stamp': len(arr),
            'dt': dt_stats(sorted(arr)) if len(arr) >= 2 else {'n_dt': 0},
            'backwards': back,
            'duplicates': dup,
        }
        wall = (streams.get(kind) or {}).get('wall_span_s')
        if arr and wall:
            ev['sim_span_s'] = max(arr) - min(arr)
            ev['wall_span_s'] = wall
            ev['sim_over_wall_rate'] = (max(arr) - min(arr)) / wall
        sc_ev[kind] = ev
        if len(arr) < MIN_MESSAGES_FOR_CADENCE:
            sc_status = 'contradicted'
            sc_problems.append(
                '%s: only %d valid stamps (< %d)'
                % (kind, len(arr), MIN_MESSAGES_FOR_CADENCE))
        if back > 0:
            sc_status = 'contradicted'
            sc_problems.append(
                '%s: %d backwards stamps (non-monotonic source stamps)'
                % (kind, back))
    criteria.append(_criterion(
        'source_stamps_cadence',
        'Source stamps and measured cadence of pose/info, /joint_states and '
        '/body_pose (SceneBroadcaster rate is not configured — measured, '
        'never assumed)', True, sc_status, sc_ev,
        '; '.join(sc_problems)))

    # --- criterion 4: alignment hit rate (blocking, operative) -----------
    pose_sorted = sorted(r['stamp_s'] for r in pose_recs)
    window = args.pose_window_sec
    hits = 0
    misses = []
    for t in joint_arr:
        i = bisect.bisect_right(pose_sorted, t)
        last = pose_sorted[i - 1] if i > 0 else None
        if last is not None and t - last <= window:
            hits += 1
        elif len(misses) < 20:
            misses.append({
                'joint_stamp_s': t,
                'last_pose_stamp_s': last,
                'gap_s': t - last if last is not None else None,
            })
    hit_rate = (hits / len(joint_arr)) if joint_arr else None
    al_ev = {
        'n_joint_frames': len(joint_arr),
        'n_pose_messages': len(pose_sorted),
        'causal_window_s': window,
        'window_semantics': 'last-not-after: last pose stamp p with p <= t '
                            'and t - p <= window (RFC § 3.6.3, App. B V-2 '
                            'criterion 4)',
        'n_hits': hits,
        'hit_rate': round(hit_rate, 6) if hit_rate is not None else None,
        'hit_rate_min': V2_HIT_RATE_MIN,
        'first_misses': misses,
        'pose_info_dt_mean_s': (sc_ev['pose_info']['dt'] or {}).get(
            'dt_mean_s'),
    }
    if not joint_arr or not pose_sorted:
        al_status = 'unknown'
        al_problems = [
            'alignment not measurable: joint frames = %d, pose messages = '
            '%d' % (len(joint_arr), len(pose_sorted))]
    elif len(joint_arr) < MIN_MESSAGES_FOR_CADENCE:
        al_status = 'contradicted'
        al_problems = [
            'only %d joint frames (< %d) — hit rate not trustworthy'
            % (len(joint_arr), MIN_MESSAGES_FOR_CADENCE)]
    elif hit_rate < V2_HIT_RATE_MIN:
        al_status = 'contradicted'
        al_problems = [
            'joint-clock causal-window hit rate %s < %s — pose path cannot '
            'be associated to >= 95 %% of joint frames'
            % (round(hit_rate, 6), V2_HIT_RATE_MIN)]
    else:
        al_status = 'confirmed'
        al_problems = []
    criteria.append(_criterion(
        'alignment_hit_rate',
        'Fraction of joint-clock frames whose causal last-not-after %s s '
        'window contains at least one pose/info message (>= 95 %%)'
        % window, True, al_status, al_ev, '; '.join(al_problems)))

    # --- determined facts & verdict ---------------------------------------
    determined = [
        {
            'fact': 'pose/info entity naming convention',
            'value': {'convention': naming,
                      'verbatim_names': verbatim_names},
            'consequence': 'the collector must associate poses by these '
                           'verbatim names (local-name matching is a probe '
                           'convenience, not the collector contract)',
        },
        {
            'fact': 'fraction of pose/info messages carrying all 25 links',
            'value': round(frac_all, 6) if frac_all is not None else None,
            'consequence': 'a single-message 25-link snapshot is usable '
                           'only when this fraction is high; otherwise the '
                           'collector must stitch across messages',
        },
        {
            'fact': 'measured pose/info cadence (source stamps)',
            'value': sc_ev['pose_info']['dt'],
            'consequence': 'SceneBroadcaster rate is determined at runtime '
                           'and pinned in this report, not assumed',
        },
        {
            'fact': '/body_pose exact-stamp agreement with pose/info',
            'value': {
                'fraction_exact_stamp_matched': round(match_frac, 6)
                if match_frac is not None else None,
                'frame_ids': {str(k): v for k, v in frame_ids.items()},
            },
            'consequence': 'body_pose_converter is confirmed to re-publish '
                           'the model-entry pose with the source stamp',
        },
        {
            'fact': 'pose/info link-entry frame semantics (REV1.2/R4)',
            'value': {
                'link_entry_frame_semantics': frame_sem,
                'base_link_identity_fraction':
                    frame_sem_ev['base_link_identity_fraction'],
                'base_link_matches_nao_entry_fraction':
                    frame_sem_ev['base_link_matches_nao_entry_fraction'],
            },
            'consequence': 'link entries are parent-frame (model-relative) '
                           'poses, reported as model-relative and NEVER as '
                           'world pose: the collector must NOT record them '
                           'as world poses; world poses are derived by '
                           'composing the nao model entry with each link '
                           'entry (same Pose_V message) — '
                           'T_world_link = T_world_model . T_model_link '
                           '(dynamic_pose/info is also parent-relative, '
                           'not a native world-pose alternative)',
        },
        {
            'fact': 'world-pose composition consistency (identity link '
                    'only, REV1.2/R5)',
            'value': {
                'composed_base_link_vs_nao_entry_max_diff_m':
                    composition_ev[
                        'composed_base_link_vs_nao_entry_max_diff_m'],
                'dynamic_entry_semantics': dyn_ev[
                    'dynamic_entry_semantics'],
                'dynamic_pose_info_n_messages': dyn_ev['n_messages'],
            },
            'consequence': 'identity-link consistency (base_link IS the '
                           'model frame, rotation term never exercised) — '
                           'NOT an end-to-end validation: the composition '
                           'method is the declared collector recovery path '
                           'but REQUIRES verification using selected '
                           'non-identity links (e.g. l_ankle/r_ankle) '
                           'before composed poses are treated as validated',
        },
        {
            'fact': 'model entry "nao" vs base_link pose offset',
            'value': wp_ev['model_entry_nao_vs_base_link_median_delta_m'],
            'consequence': 'under model-relative link entries the base_link '
                           'entry is the model-frame identity, so this '
                           'delta IS the nao model-entry world pose itself — '
                           'not a constant kinematic offset between two '
                           'world poses',
        },
        {
            'fact': 'joint-clock causal-window hit rate (%s s)'
                    % window,
            'value': round(hit_rate, 6) if hit_rate is not None else None,
            'consequence': 'per-frame miss flags are required in the '
                           'collector when frames fall outside the window',
        },
    ]
    blocking_failures = [c['id'] for c in criteria
                         if c['blocking'] and c['status'] != 'confirmed']
    consequences = []
    if blocking_failures:
        consequences.append(
            'acquisition blocked until the pose path is fixed (RFC REV1.2 '
            'App. B V-2 failure conditions): %s' % ', '.join(blocking_failures))
    else:
        consequences.append(
            'pose/info link entries confirmed parent/model-relative '
            '(REV1.2/R4): reported as model-relative, never as world pose; '
            'world poses are derived by composition with the nao model '
            'entry (same message) and still REQUIRE non-identity-link '
            'verification (REV1.2/R5) before being treated as validated')
        consequences.append(
            'pose/info may serve as the raw pose source (model-relative '
            'entries + model entry retained verbatim) with the %s s '
            'causal last-not-after association window (RFC § 3.6.3)'
            % window)
        consequences.append(
            'joint-clock frames are pose-associatable at >= 95 %%; the '
            'collector still records per-frame miss flags')
    verdict = {
        'probe': 'V-2',
        'status': 'FAIL' if blocking_failures else 'PASS',
        'blocking_failures': blocking_failures,
        'non_blocking_statuses': {c['id']: c['status'] for c in criteria
                                  if not c['blocking']},
        'consequences': consequences,
    }
    return {'criteria': criteria, 'determined_facts': determined,
            'verdict': verdict}


# ---------------------------------------------------------------------------
# V-2 entry point.
# ---------------------------------------------------------------------------

def run_v2(args, env, provenance):
    """Run the V-2 collection and analysis; return the full report dict.

    Raises RuntimeError when the gz-transport native path is unavailable:
    /world/nao_world/pose/info is a gz-transport topic with no ROS bridge
    counterpart, so the probe fails closed without it.
    """
    _contacts_pb2, pose_v_pb2, transport = gz_packages()
    (_rclpy, _RosContacts, _RosContact, _RosJointWrench, RosJointState,
     RosPoseStamped) = ros_packages()
    if pose_v_pb2 is None or transport is None:
        raise RuntimeError(
            'gz python bindings (gz.msgs* / gz.transport*) not importable — '
            '/world/nao_world/pose/info has no ROS bridge counterpart; V-2 '
            'fails closed instead of assuming /body_pose-only facts')
    spec = [
        ('pose_info', 'gz', POSE_INFO_TOPIC, pose_v_pb2.Pose_V,
         gz_pose_record),
        ('dynamic_pose_info', 'gz', DYNAMIC_POSE_TOPIC, pose_v_pb2.Pose_V,
         gz_pose_record),
        ('joint_states', 'ros', JOINT_STATES_TOPIC, RosJointState,
         joint_state_record),
        ('body_pose', 'ros', BODY_POSE_TOPIC, RosPoseStamped,
         body_pose_record),
    ]
    collected = run_collection('v2', spec, args.duration, args.warmup_sec,
                               transport_module=transport)
    analysis = analyze_v2(collected, args, provenance)
    schemas = None
    try:
        schemas = {
            'gz.msgs.Pose_V': introspect_pb(pose_v_pb2.Pose_V),
            'sensor_msgs/msg/JointState': introspect_ros(RosJointState),
            'geometry_msgs/msg/PoseStamped': introspect_ros(
                RosPoseStamped),
        }
    except Exception as exc:  # noqa: BLE001 — recorded, never fatal
        record_callback_error('schema:v2', exc)
    samples = build_samples(collected['records'], args.max_samples)
    return build_report('V-2', args, env, provenance, collected, analysis,
                        samples, schemas=schemas, bridge_loss=None)


# ---------------------------------------------------------------------------
# Report assembly: samples, environment, provenance, build, write.
# ---------------------------------------------------------------------------

def _contact_sample(rec, max_contacts=2, max_depths=8, max_wrenches=2):
    """Truncate one contact record for embedding in the report."""
    out = {
        'stamp_s': rec.get('stamp_s'),
        'n_contacts': rec.get('n_contacts'),
        'contacts': [],
    }
    for c in (rec.get('contacts') or []):
        c = dict(c)
        c['depths'] = list(c.get('depths') or [])[:max_depths]
        c['wrenches'] = list(c.get('wrenches') or [])[:max_wrenches]
        out['contacts'].append(c)
        if len(out['contacts']) >= max_contacts:
            break
    return out


def _pose_sample(rec, max_entries=30):
    """Truncate one Pose_V record: leading (name, id) pairs + totals."""
    entries = rec.get('entries') or []
    return {
        'stamp_s': rec.get('stamp_s'),
        'n_entries_total': len(entries),
        'first_entries': [{'name': nm, 'id': i}
                           for nm, i in entries[:max_entries]],
        'tracked': rec.get('tracked'),
    }


def build_samples(records, max_samples):
    """First (and first non-empty) message per stream kind, truncated.

    max_samples >= 2 additionally embeds the first message that actually
    carries contact points / pose entries (empty messages stay visible via
    n_available vs n_contacts/n_entries_total)."""
    out = {}
    for kind, recs in sorted(records.items()):
        if not recs:
            out[kind] = {'n_available': 0, 'first_message': None}
            continue
        first = recs[0]
        entry = {'n_available': len(recs)}
        if 'contacts' in first:
            entry['first_message'] = _contact_sample(first)
            if max_samples >= 2:
                nonempty = next((r for r in recs if r.get('n_contacts')),
                                None)
                entry['first_nonempty_message'] = (
                    _contact_sample(nonempty) if nonempty is not None
                    else None)
        elif 'entries' in first:
            entry['first_message'] = _pose_sample(first)
        else:
            entry['first_message'] = first
        out[kind] = entry
    return out


def environment_facts(args):
    return {
        'wallclock_utc': utc_now(),
        'python_version': sys.version.split()[0],
        'ROS_DISTRO': os.environ.get('ROS_DISTRO'),
        'RMW_IMPLEMENTATION': os.environ.get('RMW_IMPLEMENTATION'),
        'ROS_DOMAIN_ID': {
            'effective': os.environ.get('ROS_DOMAIN_ID'),
            'cli': (str(args.ros_domain_id)
                    if args.ros_domain_id is not None else None),
        },
        'GZ_PARTITION': {
            'effective': os.environ.get('GZ_PARTITION'),
            'cli': args.gz_partition,
        },
    }


def provenance_facts():
    """Probe pinning (RFC Appendix B): commit + frozen-artifact hashes."""
    return {
        'git_commit': git_commit(),
        'rfc': {'path': RFC_PATH, 'sha256': sha256_file(RFC_PATH)},
        'sdf': {'path': SDF_PATH, 'sha256': sha256_file(SDF_PATH)},
        'sdf_mass': sdf_mass_summary(SDF_PATH),
        'mode': {
            'read_only': True,
            'level': 'Level 1 preparation instrumentation (RFC § 0.0)',
            'published_topics': [],
            'frozen_components_modified': 'none',
            'outputs': 'dataset/stage1/probe/V1_report[_suffix].json, '
                       'V2_report[_suffix].json (--report-suffix)',
        },
    }


REPORT_SCHEMA_VERSION = 'stage1-probe/1.2'


def build_report(probe_id, args, env, provenance, collected, analysis,
                 samples, schemas=None, bridge_loss=None):
    """Assemble the full probe report; floats rounded to 12 digits.

    12 (not 9) digits: V-2 quaternion-component diffs are bounded by 1e-9
    and must survive rounding; positions by 1e-6."""
    config = {
        'probe': args.probe,
        'duration_s': args.duration,
        'warmup_sec': args.warmup_sec,
        'contact_window_sec': args.contact_window_sec,
        'pose_window_sec': args.pose_window_sec,
        'max_samples': args.max_samples,
        'report_suffix': args.report_suffix,
        'ros_qos': 'BEST_EFFORT / KEEP_LAST(50) / VOLATILE',
        'gz_transport': 'gz-transport native subscribe (default QoS)',
    }
    report = {
        'schema_version': REPORT_SCHEMA_VERSION,
        'probe': probe_id,
        'wallclock_utc': utc_now(),
        'argv': list(sys.argv),
        'environment': env,
        'provenance': provenance,
        'config': config,
        'config_sha256': hashlib.sha256(json.dumps(
            round_floats(config, 12), sort_keys=True,
            separators=(',', ':')).encode('utf-8')).hexdigest(),
        'measurement': {
            'streams': collected['streams'],
            'session_errors': collected['session_errors'],
            'callback_errors': collected['callback_errors'],
        },
        'analysis': analysis,
        'samples': samples,
    }
    if schemas is not None:
        report['message_schemas'] = schemas
    if bridge_loss is not None:
        report['bridge_field_loss'] = bridge_loss
    return round_floats(report, 12)


def write_report(path, report, args):
    if os.path.exists(path) and not args.allow_overwrite:
        raise RuntimeError(
            'refusing to overwrite existing report %s without '
            '--allow-overwrite' % path)
    atomic_write_json(path, report)
    print('[stage1_probe] wrote %s' % path, flush=True)


# ---------------------------------------------------------------------------
# CLI entry point.
# ---------------------------------------------------------------------------

def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Stage 1 pre-acquisition probes (V-1 contacts, V-2 pose). '
                    'Read-only Level 1 instrumentation; the simulation must '
                    'already be running on the same ROS_DOMAIN_ID / '
                    'GZ_PARTITION.')
    parser.add_argument('--probe', choices=('v1', 'v2', 'both'),
                        default='both',
                        help='which probe to run (default: both)')
    parser.add_argument('--duration', type=float, default=15.0,
                        help='collection window in seconds after warmup '
                             '(default: 15.0)')
    parser.add_argument('--warmup-sec', type=float, default=45.0,
                        help='max seconds to wait for the first message on '
                             'every stream (default: 45.0)')
    parser.add_argument('--out-root',
                        default=os.path.join('dataset', 'stage1', 'probe'),
                        help='report directory; relative paths resolve '
                             'against the workspace root (default: '
                             'dataset/stage1/probe)')
    parser.add_argument('--ros-domain-id', type=int, default=None,
                        help='ROS_DOMAIN_ID set before rclpy.init (default: '
                             'keep the inherited environment)')
    parser.add_argument('--gz-partition', default=None,
                        help='GZ_PARTITION set before creating the '
                             'gz-transport node (default: keep the '
                             'inherited environment)')
    parser.add_argument('--contact-window-sec', type=float,
                        default=CONTACT_ASSOC_WINDOW_SEC,
                        help='V-1 joint-state causal association window '
                             '(default: 0.05)')
    parser.add_argument('--pose-window-sec', type=float,
                        default=POSE_ASSOC_WINDOW_SEC,
                        help='V-2 pose/joint causal association window '
                             '(default: 0.02)')
    parser.add_argument('--max-samples', type=int, default=2,
                        help='messages embedded per stream in the report '
                             '(default: 2)')
    parser.add_argument('--report-suffix', default='',
                        help='suffix appended to report filenames, e.g. '
                             'REV12 writes V1_report_REV12.json / '
                             'V2_report_REV12.json (default: none — '
                             'V1_report.json / V2_report.json)')
    parser.add_argument('--allow-overwrite', action='store_true',
                        help='allow overwriting existing report files')
    return parser.parse_args(argv)


def main(argv=None):
    """Run the selected probe(s); exit codes: 0 = all PASS, 1 = any FAIL,
    2 = fatal (no report written for at least one requested probe)."""
    args = parse_args(argv)
    # Environment must be finalized BEFORE rclpy.init()/gz node creation —
    # both select domain/partition at init time.
    if args.ros_domain_id is not None:
        os.environ['ROS_DOMAIN_ID'] = str(args.ros_domain_id)
    if args.gz_partition is not None:
        os.environ['GZ_PARTITION'] = args.gz_partition

    out_root = args.out_root
    if not os.path.isabs(out_root):
        out_root = os.path.join(WS, out_root)
    os.makedirs(out_root, exist_ok=True)

    fatals = []
    statuses = {}
    rclpy_inited = False
    try:
        rclpy, _rc, _rco, _rcw, _rjs, _rps = ros_packages()
        rclpy.init()
        rclpy_inited = True
        env = environment_facts(args)
        provenance = provenance_facts()
        order = ('v1', 'v2') if args.probe == 'both' else (args.probe,)
        runners = {'v1': run_v1, 'v2': run_v2}
        for probe_id in order:
            suffix = ('_%s' % args.report_suffix) if args.report_suffix \
                else ''
            path = os.path.join(out_root, '%s_report%s.json'
                                % (probe_id.upper(), suffix))
            try:
                report = runners[probe_id](args, env, provenance)
                write_report(path, report, args)
                statuses[probe_id] = report['analysis']['verdict']['status']
            except Exception as exc:  # noqa: BLE001 — fatal, reported
                fatals.append((probe_id, '%s: %s'
                               % (type(exc).__name__, exc)))
                print('[stage1_probe] FATAL %s: %s: %s'
                      % (probe_id, type(exc).__name__, exc),
                      file=sys.stderr, flush=True)
    except Exception as exc:  # noqa: BLE001 — rclpy init failed
        fatals.append(('init', '%s: %s' % (type(exc).__name__, exc)))
        print('[stage1_probe] FATAL init: %s: %s'
              % (type(exc).__name__, exc), file=sys.stderr, flush=True)
    finally:
        if rclpy_inited:
            try:
                import rclpy
                rclpy.shutdown()
            except Exception:
                pass

    for probe_id, status in sorted(statuses.items()):
        print('[stage1_probe] %s verdict: %s' % (probe_id, status),
              flush=True)
    if fatals:
        return 2
    if any(s != 'PASS' for s in statuses.values()):
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
