#!/usr/bin/python3
"""Task 3F validation: minimal foot-frame / TF interface.

Launches the workspace ``nao_sim.launch.py`` headless with ``use_rsp:=true``
(unique ``GZ_PARTITION``) and verifies the frozen TF tree:

    world -> base_link            (dynamic, world_tf_converter, /tf)
    base_link -> torso -> ... -> l_ankle / r_ankle   (dynamic, RSP, /tf)
    l_ankle -> l_sole, r_ankle -> r_sole, base_link -> torso (static, RSP, /tf_static)

Checks:
  * /tf and /tf_static type, QoS (RELIABLE/VOLATILE and RELIABLE/TRANSIENT_LOCAL);
  * frame existence and canonical names (no scoped / parallel aliases);
  * dynamic vs static classification and unique child authority;
  * world -> base_link direction (never reversed) and uniqueness;
  * dynamic foot motion (world -> l_sole / r_sole change over simulation time);
  * Gazebo model/link pose cross-check (world -> base_link exactly, feet composed);
  * simulation-time stamps, pause freeze / resume, restart stability;
  * Task 3B/3C/3D/3E regression;
  * no Pose_V -> TF bridge, no parallel tree.

Run with the ROS Python interpreter, from the workspace root:

    /usr/bin/python3 tools/validate_task3f_tf.py --runs 2 --duration 30 \
        --out docs/task3f_evidence

Exit code 0 = PASS, 1 = FAIL, 2 = BLOCKED.
"""

import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time
from rclpy.duration import Duration

import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Bool

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(REPO_ROOT, "src", "nao", "nao_description", "urdf", "nao_simple.urdf")
SDF = os.path.join(REPO_ROOT, "src", "nao", "nao_description", "urdf", "nao_simple.sdf")

POSE_TOPIC = "/world/nao_world/pose/info"
MODEL = "nao"
SOLE_OFFSET = [0.0, 0.0, -0.04511]
POS_TOL_EXACT = 1e-6
ROT_TOL_EXACT = 1e-6
POS_TOL_FOOT = 8e-3
ROT_TOL_FOOT = 2e-2

FORBIDDEN_FRAME_RE = re.compile(r"::|nao/|^left_foot$|^right_foot$|^/")


def load_urdf_links():
    root = ET.parse(URDF).getroot()
    return {link.get("name") for link in root.findall("link")}


def run(cmd, env=None, timeout=20):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, env=env)
        return p.returncode, p.stdout + p.stderr
    except Exception as exc:  # noqa: BLE001
        return 127, str(exc)


def spin_for(node, seconds, stop_when=None):
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.02)
        if stop_when is not None and stop_when():
            return True
    return False


# --------------------------------------------------------------------------
# quaternion / transform math ([x, y, z, w])
# --------------------------------------------------------------------------
def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


def qconj(q):
    return [-q[0], -q[1], -q[2], q[3]]


def qrot(q, v):
    r = qmul(qmul(q, [v[0], v[1], v[2], 0.0]), qconj(q))
    return [r[0], r[1], r[2]]


def qangle(q1, q2):
    d = abs(sum(a * b for a, b in zip(q1, q2)))
    d = min(1.0, max(-1.0, d))
    return 2.0 * math.acos(d)


def compose(p1, q1, p2, q2):
    rp = qrot(q1, p2)
    return ([p1[0] + rp[0], p1[1] + rp[1], p1[2] + rp[2]], qmul(q1, q2))


def parse_pose_v(text):
    """Return (stamp, {name: (position[3], quaternion[4])}) from gz Pose_V text."""
    m_stamp = re.search(r"stamp\s*\{\s*sec:\s*(\d+)\s*nsec:\s*(\d+)\s*\}", text)
    if not m_stamp:
        return None
    stamp = (int(m_stamp.group(1)), int(m_stamp.group(2)))
    poses = {}
    for m_name in re.finditer(r'name:\s*"([^"]+)"', text):
        name = m_name.group(1)
        # tail up to the next name or end
        nxt = text.find('name: "', m_name.end())
        tail = text[m_name.end(): nxt if nxt > 0 else len(text)]
        m_pos = re.search(r"position\s*\{([^}]*)\}", tail)
        m_ori = re.search(r"orientation\s*\{([^}]*)\}", tail)
        if not m_pos or not m_ori:
            continue

        def vals(block, keys):
            out = []
            for k in keys:
                mm = re.search(r"\b%s:\s*([-+0-9.eE]+)" % k, block)
                out.append(float(mm.group(1)) if mm else 0.0)
            return out

        poses[name] = (vals(m_pos.group(1), "xyz"),
                       vals(m_ori.group(1), "xyzw"))
    return stamp, poses


def capture_gz_pose(env, timeout=8):
    rc, out = run(["gz", "topic", "-e", "-t", POSE_TOPIC, "-n", "1"], env=env, timeout=timeout)
    return parse_pose_v(out) if rc == 0 else None


def tf_lookup(buffer, target, source, stamp=None, timeout=0.5):
    if stamp is None:
        t = Time()
    elif isinstance(stamp, Time):
        t = stamp
    else:
        t = Time(seconds=int(stamp[0]), nanoseconds=int(stamp[1]))
    tr = buffer.lookup_transform(target, source, t,
                                 Duration(nanoseconds=int(timeout * 1e9)))
    p = tr.transform.translation
    q = tr.transform.rotation
    return ([p.x, p.y, p.z], [q.x, q.y, q.z, q.w],
            (tr.header.stamp.sec, tr.header.stamp.nanosec))


def publisher_nodes(info):
    """Node names of publishers only (exclude subscribers)."""
    pub_section = info.split("Subscription count:")[0]
    return set(re.findall(r"Node name:\s*(\S+)", pub_section))


# --------------------------------------------------------------------------
class Collector(Node):
    def __init__(self, name="task3f_validator"):
        super().__init__(name)
        rel100 = QoSProfile(depth=100)
        rel100.reliability = ReliabilityPolicy.RELIABLE
        rel100.durability = DurabilityPolicy.VOLATILE
        rel100.history = HistoryPolicy.KEEP_LAST
        static_qos = QoSProfile(depth=100)
        static_qos.reliability = ReliabilityPolicy.RELIABLE
        static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        be = QoSProfile(depth=1)
        be.reliability = ReliabilityPolicy.BEST_EFFORT
        be.durability = DurabilityPolicy.VOLATILE
        self.tf = []
        self.tf_static = []
        self.clock = []
        self.joints = []
        self.body = []
        self.imu = []
        self.left = []
        self.right = []
        self.create_subscription(TFMessage, "/tf", lambda m: self.tf.append(m), rel100)
        self.create_subscription(TFMessage, "/tf_static",
                                 lambda m: self.tf_static.append(m), static_qos)
        self.create_subscription(Clock, "/clock", lambda m: self.clock.append(m), be)
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: self.joints.append(m), rel100)
        self.create_subscription(PoseStamped, "/body_pose", lambda m: self.body.append(m), rel100)
        self.create_subscription(Imu, "/imu/data", lambda m: self.imu.append(m), rel100)
        self.create_subscription(Bool, "/left_foot/contact", lambda m: self.left.append(m), rel100)
        self.create_subscription(Bool, "/right_foot/contact", lambda m: self.right.append(m), rel100)


def pairs_of(msg):
    return [(t.header.frame_id, t.child_frame_id,
             (t.header.stamp.sec, t.header.stamp.nanosec)) for t in msg.transforms]


def unique_tree(pair_list):
    """child -> set(parent); also detect duplicate identical (parent,child)."""
    tree = {}
    dup = []
    for parent, child, _ in pair_list:
        tree.setdefault(child, set()).add(parent)
    # duplicate (parent, child) within the same topic is normal across updates;
    # what matters is multiple DISTINCT parents for one child.
    return tree


# --------------------------------------------------------------------------
def run_once(args, index, env):
    partition = "t3f_%d_%d" % (index, int(time.time()))
    env = dict(env)
    env["GZ_PARTITION"] = partition
    cleanup(env)
    log_path = os.path.join(args.out, "launch_run%d.log" % index)
    log_f = open(log_path, "w")
    cmd = ["ros2", "launch", "nao_gz_sim", "nao_sim.launch.py",
           "gui:=false", "use_rsp:=true"]
    proc = subprocess.Popen(cmd, stdout=log_f, stderr=subprocess.STDOUT,
                            env=env, start_new_session=True)
    result = {"run": index, "partition": partition, "log": log_path,
              "launch_cmd": " ".join(cmd), "checks": {}, "stats": {}, "pause": {},
              "authority": {}}
    failures = []
    blocked = []

    def record(name, ok, detail="", block=False):
        result["checks"][name] = {"ok": bool(ok), "detail": detail}
        if block and not ok:
            blocked.append("%s: %s" % (name, detail))
        elif not ok:
            failures.append("%s: %s" % (name, detail))

    urdf_links = load_urdf_links()

    try:
        node = Collector("task3f_validator_%d" % index)
        buffer = tf2_ros.Buffer(cache_time=Duration(seconds=300.0))
        listener = tf2_ros.TransformListener(buffer, node)
        _ = listener  # keep alive

        # wait until the tree is connected: world->base_link + l_sole available
        def have_tree():
            tl = [c for m in node.tf for (p, c, s) in pairs_of(m)]
            return ("l_sole" in {c for m in node.tf_static for (_, c, _) in pairs_of(m)}
                    and any(p == "world" and c == "base_link"
                            for m in node.tf for (p, c, _) in pairs_of(m)))
        got = spin_for(node, 60.0, stop_when=have_tree)
        if not got:
            record("tf_tree_connected", False,
                   "world->base_link or l_sole never appeared", block=True)
            return result, failures, blocked

        t0 = time.time()
        spin_for(node, args.duration)
        wall = time.time() - t0

        # external topic inspection
        _, tf_type = run(["ros2", "topic", "type", "/tf"], env=env)
        _, tfs_type = run(["ros2", "topic", "type", "/tf_static"], env=env)
        _, tf_info = run(["ros2", "topic", "info", "-v", "/tf"], env=env)
        _, tfs_info = run(["ros2", "topic", "info", "-v", "/tf_static"], env=env)
        _, topics = run(["ros2", "topic", "list"], env=env)
        result["tf_type"] = tf_type.strip()
        result["tf_static_type"] = tfs_type.strip()
        result["tf_info"] = tf_info
        result["tf_static_info"] = tfs_info
        result["topic_list"] = sorted(topics.split())

        record("tf_type", tf_type.strip() == "tf2_msgs/msg/TFMessage", tf_type.strip())
        record("tf_static_type", tfs_type.strip() == "tf2_msgs/msg/TFMessage", tfs_type.strip())
        tf_nodes = publisher_nodes(tf_info)
        tfs_nodes = publisher_nodes(tfs_info)
        tf_pubs = re.findall(r"Publisher count:\s*(\d+)", tf_info)
        tfs_pubs = re.findall(r"Publisher count:\s*(\d+)", tfs_info)
        result["tf_nodes"] = sorted(tf_nodes)
        result["tf_static_nodes"] = sorted(tfs_nodes)
        record("tf_publishers", tf_pubs == ["2"], str(tf_pubs))
        record("tf_static_single_publisher", tfs_pubs == ["1"], str(tfs_pubs))
        record("tf_nodes_expected",
               tf_nodes == {"world_tf_converter", "robot_state_publisher"}, str(sorted(tf_nodes)))
        record("tf_static_nodes_rsp", tfs_nodes == {"robot_state_publisher"}, str(sorted(tfs_nodes)))
        record("tf_qos_reliable", "Reliability: RELIABLE" in tf_info, "RELIABLE")
        record("tf_qos_volatile", "Durability: VOLATILE" in tf_info, "VOLATILE")
        record("tf_static_qos_reliable", "Reliability: RELIABLE" in tfs_info, "RELIABLE")
        record("tf_static_qos_transient_local",
               "Durability: TRANSIENT_LOCAL" in tfs_info, "TRANSIENT_LOCAL")

        # ---- build trees ----
        dyn_pairs = [x for m in node.tf for x in pairs_of(m)]
        stat_pairs = [x for m in node.tf_static for x in pairs_of(m)]
        dyn_tree = unique_tree(dyn_pairs)
        stat_tree = unique_tree(stat_pairs)
        dyn_children = set(dyn_tree)
        stat_children = set(stat_tree)
        all_frames = {"world"} | dyn_children | stat_children
        for parents in list(dyn_tree.values()) + list(stat_tree.values()):
            all_frames |= parents
        result["dynamic_frames"] = sorted(dyn_children)
        result["static_frames"] = sorted(stat_children)
        result["all_frames"] = sorted(all_frames)

        # frame existence
        for f in ("world", "base_link", "torso", "l_ankle", "r_ankle", "l_sole", "r_sole"):
            record("frame_%s_present" % f, f in all_frames, f)
        record("world_base_link_edge_valid", "base_link" in dyn_tree
               and "world" in dyn_tree.get("base_link", set()), str(dyn_tree.get("base_link")))
        record("base_link_not_reversed", not any(
            p == "base_link" and c == "world" for (p, c, _) in dyn_pairs + stat_pairs),
            "no base_link->world")
        record("l_ankle_dynamic", "l_ankle" in dyn_children, "dynamic")
        record("r_ankle_dynamic", "r_ankle" in dyn_children, "dynamic")
        record("l_sole_static", "l_sole" in stat_children, "static offset")
        record("r_sole_static", "r_sole" in stat_children, "static offset")
        record("torso_static", "torso" in stat_children, "static offset")
        record("l_sole_parent_l_ankle", stat_tree.get("l_sole") == {"l_ankle"},
               str(stat_tree.get("l_sole")))
        record("r_sole_parent_r_ankle", stat_tree.get("r_sole") == {"r_ankle"},
               str(stat_tree.get("r_sole")))
        record("base_link_parent_torso_static", stat_tree.get("torso") == {"base_link"},
               str(stat_tree.get("torso")))

        # canonical names only
        noncanon = sorted(f for f in all_frames if f != "world" and f not in urdf_links)
        record("frames_canonical", not noncanon, str(noncanon))
        scoped = sorted(f for f in all_frames if FORBIDDEN_FRAME_RE.search(f))
        record("no_scoped_or_alias_frames", not scoped, str(scoped))

        # unique child authority: no child in both topics, one parent each
        both = sorted(dyn_children & stat_children)
        record("no_child_in_both_topics", not both, str(both))
        multi_dyn = sorted(c for c, ps in dyn_tree.items() if len(ps) > 1)
        multi_stat = sorted(c for c, ps in stat_tree.items() if len(ps) > 1)
        record("unique_dynamic_parent", not multi_dyn, str(multi_dyn))
        record("unique_static_parent", not multi_stat, str(multi_stat))

        # world parent only base_link
        world_children = sorted({c for (p, c, _) in dyn_pairs if p == "world"})
        record("world_parent_only_base_link", world_children == ["base_link"],
               str(world_children))
        record("no_world_in_static",
               not any(p == "world" for (p, c, _) in stat_pairs), "no static world")

        # ---- timestamps ----
        wb_stamps = sorted({s for (p, c, s) in dyn_pairs if p == "world" and c == "base_link"})
        record("tf_stamp_nonzero", bool(wb_stamps) and all(s != (0, 0) for s in wb_stamps),
               "%d world->base_link stamps" % len(wb_stamps))
        record("tf_stamp_monotonic",
               all(wb_stamps[i] < wb_stamps[i + 1] for i in range(len(wb_stamps) - 1)),
               "world->base_link strictly increasing")
        clock_vals = [(m.clock.sec, m.clock.nanosec) for m in node.clock]
        if wb_stamps and clock_vals:
            lo = min(clock_vals)
            hi = max(clock_vals)
            in_range = all(lo <= s <= hi for s in wb_stamps)
            record("tf_stamp_is_sim_time", in_range,
                   "wb stamp range %s..%s within clock %s..%s"
                   % (wb_stamps[0], wb_stamps[-1], lo, hi))

        # ---- dynamic world->base_link ----
        wb_positions = []
        for m in node.tf:
            for t in m.transforms:
                if t.header.frame_id == "world" and t.child_frame_id == "base_link":
                    wb_positions.append([t.transform.translation.x, t.transform.translation.y,
                                         t.transform.translation.z])
        if wb_positions:
            span = max(math.dist(wb_positions[0], p) for p in wb_positions)
            result["stats"]["world_base_link_motion_m"] = span
            record("world_base_link_dynamic", span > 1e-3,
                   "translation span %.4f m over window" % span)
        else:
            record("world_base_link_dynamic", False, "no transform")

        # ---- Gazebo cross-check (two simulations) ----
        cross_ok = 0
        for label in ("a", "b"):
            parsed = capture_gz_pose(env)
            # let the tf buffer ingest the transforms around the captured stamp
            spin_for(node, 1.0)
            if parsed is None:
                record("crosscheck_%s" % label, False, "no gz pose sample", block=True)
                continue
            gstamp, poses = parsed
            if MODEL not in poses:
                record("crosscheck_%s" % label, False, "model %s not in pose/info" % MODEL,
                       block=True)
                continue
            mp, mq = poses[MODEL]
            # world->base_link at the same stamp, read back from the tf buffer
            try:
                bp, bq, _ = tf_lookup(buffer, "world", "base_link", stamp=gstamp)
            except Exception as exc:  # noqa: BLE001
                record("crosscheck_%s" % label, False,
                       "world->base_link tf lookup failed at %s: %s" % (gstamp, exc), block=True)
                continue
            dpos_bl = math.dist(bp, mp)
            dquat_bl = qangle(bq, mq)
            # composed feet: model ∘ ankle_rel ∘ sole_offset
            feet = {}
            for side, link in (("l", "l_ankle"), ("r", "r_ankle")):
                if link not in poses:
                    continue
                lp, lq = poses[link]
                fp, fq = compose(mp, mq, lp, lq)
                fp2, fq2 = compose(fp, fq, SOLE_OFFSET, [0.0, 0.0, 0.0, 1.0])
                try:
                    tp, tq, tstamp = tf_lookup(buffer, "world", "%s_sole" % side, stamp=gstamp)
                    feet[side] = (math.dist(fp2, tp), qangle(fq2, tq))
                except Exception as exc:  # noqa: BLE001
                    feet[side] = (None, None)
                    record("crosscheck_%s_%s_sole" % (label, side), False,
                           "tf lookup failed: %s" % exc, block=True)
            result["stats"]["crosscheck_%s" % label] = {
                "stamp": gstamp, "dpos_base_link": dpos_bl, "dquat_base_link": dquat_bl,
                "l_sole": feet.get("l"), "r_sole": feet.get("r")}
            ok = (dpos_bl < POS_TOL_EXACT and dquat_bl < ROT_TOL_EXACT)
            for side in ("l", "r"):
                if feet.get(side) and feet[side][0] is not None:
                    ok = ok and feet[side][0] < POS_TOL_FOOT and feet[side][1] < ROT_TOL_FOOT
            record("crosscheck_%s" % label, ok,
                   "dpos(base)=%.2e dquat(base)=%.2e l_sole=%s r_sole=%s"
                   % (dpos_bl, dquat_bl, feet.get("l"), feet.get("r")))
            cross_ok += 1
        record("crosscheck_two_windows", cross_ok == 2, "%d/2" % cross_ok)

        # ---- dynamic foot via controlled model pose (existing Gazebo service) --
        try:
            l0 = tf_lookup(buffer, "world", "l_sole")
            r0 = tf_lookup(buffer, "world", "r_sole")
            b0 = tf_lookup(buffer, "world", "base_link")
            rc_move, out_move = run(
                ["gz", "service", "-s", "/world/nao_world/set_pose",
                 "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                 "--timeout", "3000", "--req",
                 'name: "nao" position: {x: 0.4, y: 0.2, z: 0.6} '
                 'orientation: {x: 0, y: 0, z: 0, w: 1}'], env=env, timeout=10)
            spin_for(node, 1.5)
            l1 = tf_lookup(buffer, "world", "l_sole")
            r1 = tf_lookup(buffer, "world", "r_sole")
            b1 = tf_lookup(buffer, "world", "base_link")
            dl = math.dist(l0[0], l1[0])
            dr = math.dist(r0[0], r1[0])
            db = math.dist(b0[0], b1[0])
            result["stats"]["set_pose_rc"] = rc_move
            result["stats"]["l_sole_motion_m"] = dl
            result["stats"]["r_sole_motion_m"] = dr
            result["stats"]["base_link_motion_m"] = db
            record("l_sole_dynamic", dl > 1e-3,
                   "world->l_sole moves %.4f m after set_pose (rc=%d)" % (dl, rc_move))
            record("r_sole_dynamic", dr > 1e-3,
                   "world->r_sole moves %.4f m after set_pose (rc=%d)" % (dr, rc_move))
            record("base_link_dynamic", db > 1e-3,
                   "world->base_link moves %.4f m after set_pose" % db)
        except Exception as exc:  # noqa: BLE001
            record("l_sole_dynamic", False, "tf lookup failed: %s" % exc, block=True)
            record("r_sole_dynamic", False, "tf lookup failed: %s" % exc, block=True)
            record("base_link_dynamic", False, "tf lookup failed: %s" % exc, block=True)

        # ---- pause / resume ----
        pause_cmd = ["gz", "service", "-s", "/world/nao_world/control",
                     "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
                     "--timeout", "3000", "--req", "pause: true"]
        resume_cmd = list(pause_cmd)
        resume_cmd[-1] = "pause: false"
        rc_p, _ = run(pause_cmd, env=env, timeout=10)
        spin_for(node, 0.5)
        wb_before_pause = [s for (p, c, s) in [x for m in node.tf for x in pairs_of(m)]
                           if p == "world" and c == "base_link"]
        clk_before = (node.clock[-1].clock.sec, node.clock[-1].clock.nanosec) if node.clock else None
        spin_for(node, 2.0)
        wb_during = [s for (p, c, s) in [x for m in node.tf for x in pairs_of(m)]
                     if p == "world" and c == "base_link"]
        clk_during = (node.clock[-1].clock.sec, node.clock[-1].clock.nanosec) if node.clock else None
        rc_r, _ = run(resume_cmd, env=env, timeout=10)
        spin_for(node, 1.5)
        wb_after = [s for (p, c, s) in [x for m in node.tf for x in pairs_of(m)]
                    if p == "world" and c == "base_link"]
        clk_after = (node.clock[-1].clock.sec, node.clock[-1].clock.nanosec) if node.clock else None
        last_before = max(wb_before_pause) if wb_before_pause else None
        last_during = max(wb_during) if wb_during else None
        last_after = max(wb_after) if wb_after else None
        clock_frozen = clk_before is not None and clk_during is not None and clk_during <= clk_before
        clock_resumes = clk_during is not None and clk_after is not None and clk_after > clk_during
        tf_frozen = last_during is None or last_before is None or last_during <= last_before
        tf_resumes = last_after is not None and last_during is not None and last_after > last_during
        result["pause"] = {"pause_rc": rc_p, "resume_rc": rc_r,
                           "last_wb_before": last_before, "last_wb_during": last_during,
                           "last_wb_after": last_after,
                           "clock_before": clk_before, "clock_during": clk_during,
                           "clock_after": clk_after,
                           "clock_frozen": clock_frozen, "clock_resumes": clock_resumes,
                           "tf_frozen": tf_frozen, "tf_resumes": tf_resumes,
                           "pause_ok": (rc_p == 0 and rc_r == 0 and tf_frozen
                                        and tf_resumes and clock_frozen and clock_resumes)}
        record("pause_resume", result["pause"]["pause_ok"], json.dumps(result["pause"]))

        # ---- no Pose_V bridge / forbidden ----
        log_text = re.sub(r"\x1b\[[0-9;]*m", "", open(log_path, errors="ignore").read())
        pose_bridge = re.findall(r"Creating .*Bridge.*Pose_V[^\n]*", log_text)
        record("no_pose_v_bridge", not pose_bridge, str(pose_bridge[:2]))
        extra_tf = [t for t in result["topic_list"] if t not in ("/tf", "/tf_static")
                    and "tf" in t.lower()]
        record("no_extra_tf_topics", not extra_tf, str(extra_tf))

        # ---- regression ----
        record("reg_joint_states", len(node.joints) > 0
               and all(len(m.effort) == 0 for m in node.joints), "%d" % len(node.joints))
        record("reg_body_pose", len(node.body) > 0
               and all(m.header.frame_id == "world" for m in node.body), "%d" % len(node.body))
        record("reg_imu", len(node.imu) > 0
               and all(m.header.frame_id == "ImuTorsoAccelerometer_frame" for m in node.imu),
               "%d" % len(node.imu))
        record("reg_clock", len(node.clock) > 0, "%d" % len(node.clock))
        record("reg_contacts", len(node.left) > 0 and len(node.right) > 0,
               "L=%d R=%d" % (len(node.left), len(node.right)))

        # ---- authority table (inferred from topic + frame names) ----
        auth = {
            "world->base_link": {"expected": "world_tf_converter", "topic": "/tf",
                                 "dynamic": True},
            "base_link->torso": {"expected": "robot_state_publisher",
                                 "topic": "/tf_static", "dynamic": False},
            "l_ankle->l_sole": {"expected": "robot_state_publisher",
                                "topic": "/tf_static", "dynamic": False},
            "r_ankle->r_sole": {"expected": "robot_state_publisher",
                                "topic": "/tf_static", "dynamic": False},
            "joint chain (l_ankle/r_ankle)": {"expected": "robot_state_publisher",
                                              "topic": "/tf", "dynamic": True},
        }
        result["authority"] = auth
        result["stats"].update({
            "tf_dynamic_messages": len(node.tf), "tf_static_messages": len(node.tf_static),
            "wall_s": wall,
            "world_base_link_stamps": len(wb_stamps),
            "joint_states": len(node.joints), "clock": len(node.clock),
        })
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        log_f.close()
        cleanup(env)

    text = re.sub(r"\x1b\[[0-9;]*m", "", open(log_path, errors="ignore").read())
    result["violations"] = re.findall(r"CONTRACT VIOLATION[^\n]*", text)
    result["err_count"] = text.count("[Err]")
    record("no_contract_violation", not result["violations"], str(result["violations"][:3]))
    record("no_err_in_log", result["err_count"] == 0, "%d [Err]" % result["err_count"])
    _, ps_out = run(["bash", "-lc",
                     "ps -eo args --no-headers | grep -E 'world_tf_converter|"
                     "robot_state_publisher|nao_world.sdf' | grep -v grep"], env=env)
    record("no_process_leak", not ps_out.strip(), ps_out.strip()[:200])
    return result, failures, blocked


def cleanup(env):
    run(["bash", "-lc",
         "ps -eo pid,args --no-headers | grep -E 'nao_world.sdf|_converter|ros_gz_bridge|"
         "robot_state_publisher' | grep -v grep | awk '{print $1}' | xargs -r kill -9"],
        env=env, timeout=10)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--out", default="docs/task3f_evidence")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    env = dict(os.environ)

    report = {"duration": args.duration, "runs": []}
    failures = []
    blocked = []
    rclpy.init()
    try:
        for i in range(1, args.runs + 1):
            res, f, b = run_once(args, i, env)
            report["runs"].append(res)
            failures += ["run%d %s" % (i, x) for x in f]
            blocked += ["run%d %s" % (i, x) for x in b]
            with open(os.path.join(args.out, "report_run%d.json" % i), "w") as fp:
                json.dump(res, fp, indent=2)
            nchk = len(res["checks"])
            nfail = sum(1 for c in res["checks"].values() if not c["ok"])
            print("=== run %d: %d checks, %d fail%s ===" %
                  (i, nchk, nfail, " BLOCKED" if res.get("blocked") else ""))
            for cn, c in res["checks"].items():
                if not c["ok"]:
                    print("    [FAIL] %s -- %s" % (cn, c["detail"]))
    finally:
        rclpy.shutdown()

    report["failures"] = failures
    report["blocked"] = blocked
    report["verdict"] = "FAIL" if failures else ("BLOCKED" if blocked else "PASS")
    with open(os.path.join(args.out, "report.json"), "w") as fp:
        json.dump(report, fp, indent=2)
    print("\nTASK 3F VERDICT: %s" % report["verdict"])
    if failures:
        print("failures: %s" % failures)
    if blocked:
        print("blocked: %s" % blocked)
    return 0 if report["verdict"] == "PASS" else (2 if report["verdict"] == "BLOCKED" else 1)


if __name__ == "__main__":
    sys.exit(main())
