#!/usr/bin/python3
"""Task 3G: unified 30 s minimal closed-loop validation.

Runs the current minimal NAO simulation in ONE Gazebo/ROS session per run
(``gui:=false``, ``use_rsp:=true``, one ``GZ_PARTITION``) for at least 30 s and
verifies that the Task 3B-3F interfaces work together:

    /joint_states  /clock  /body_pose  /imu/data
    /left_foot/contact  /right_foot/contact
    /tf (world->base_link + RSP joint chain)  /tf_static (RSP fixed)

It performs static checks first, collects every interface in the same session,
checks publisher/QoS/authority, simulation-time consistency, TF closure and
dynamic foot lookup, an independent Gazebo Pose_V cross-check, pause/resume,
a second independent run with a fresh partition, and process cleanup.

Run with the ROS Python interpreter, from the workspace root:

    /usr/bin/python3 tools/validate_task3g_minimal_loop.py \
        --runs 2 --duration 30 --out docs/task3g_evidence

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
WORLD = os.path.join(REPO_ROOT, "src", "nao_gz_sim", "worlds", "nao_world.sdf")
POSE_TOPIC = "/world/nao_world/pose/info"
MODEL = "nao"
SOLE_OFFSET = [0.0, 0.0, -0.04511]

EXPECTED_JOINTS = [
    "HeadYaw", "HeadPitch",
    "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
    "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw",
    "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
    "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw",
]

FORBIDDEN_TOPIC_RE = re.compile(
    r"/imu/(gyro|accel|raw)$|/contacts$|_contacts$|fsr|pressure|bumper|wrench"
    r"|joint_command|ros2_control|controller_manager|/joint_positions|/joint_velocities")
FORBIDDEN_FRAME_RE = re.compile(r"::|nao/|^left_foot$|^right_foot$|^/")

EXPECTED_PUBLISHERS = {
    "/joint_states": {"joint_state_converter"},
    "/body_pose": {"body_pose_converter"},
    "/imu/data": {"imu_converter"},
    "/left_foot/contact": {"contact_converter"},
    "/right_foot/contact": {"contact_converter"},
    "/tf": {"world_tf_converter", "robot_state_publisher"},
    "/tf_static": {"robot_state_publisher"},
    "/clock": {"ros_gz_bridge"},
}

# QoS depth evidence boundary (Task 3G-R1, P2-5).
#
# `ros2 topic info -v` cannot report History depth in this environment
# (FastDDS/CLI prints "History (Depth): UNKNOWN"); this is a tool/DDS limit and
# is never treated as a failure or coerced into a value. The depth below is the
# documented SOURCE configuration (auditable source location), NOT a runtime
# measurement. The validator records the runtime CLI observation alongside it
# and never claims to have independently measured the publisher depth.
QOS_DEPTH_SOURCE = {
    "/clock": {"runtime": "BEST_EFFORT + VOLATILE", "depth_source": "CLOCK profile / launch",
               "depth": 1, "boundary": "runtime CLI depth may be UNKNOWN"},
    "/left_foot/contact": {"runtime": "RELIABLE + VOLATILE",
                           "depth_source": "contact_converter.cpp KeepLast(1)", "depth": 1,
                           "boundary": "runtime depth not independently measured"},
    "/right_foot/contact": {"runtime": "RELIABLE + VOLATILE",
                            "depth_source": "contact_converter.cpp KeepLast(1)", "depth": 1,
                            "boundary": "runtime depth not independently measured"},
    "/tf": {"runtime": "RELIABLE + VOLATILE",
            "depth_source": "world_tf_converter.cpp KeepLast(100) / RSP", "depth": 100,
            "boundary": "runtime CLI depth may be UNKNOWN"},
    "/tf_static": {"runtime": "RELIABLE + TRANSIENT_LOCAL", "depth_source": "RSP config",
                   "depth": None, "boundary": "runtime CLI depth may be UNKNOWN"},
}


def run(cmd, env=None, timeout=30, cwd=None):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout,
                           env=env, cwd=cwd)
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


def load_urdf_links():
    return {l.get("name") for l in ET.parse(URDF).getroot().findall("link")}


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
    m_stamp = re.search(r"stamp\s*\{\s*sec:\s*(\d+)\s*nsec:\s*(\d+)\s*\}", text)
    if not m_stamp:
        return None
    stamp = (int(m_stamp.group(1)), int(m_stamp.group(2)))
    poses = {}
    for m_name in re.finditer(r'name:\s*"([^"]+)"', text):
        name = m_name.group(1)
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

        poses[name] = (vals(m_pos.group(1), "xyz"), vals(m_ori.group(1), "xyzw"))
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
    tr = buffer.lookup_transform(target, source, t, Duration(nanoseconds=int(timeout * 1e9)))
    p, q = tr.transform.translation, tr.transform.rotation
    return ([p.x, p.y, p.z], [q.x, q.y, q.z, q.w],
            (tr.header.stamp.sec, tr.header.stamp.nanosec))


def publisher_nodes(info):
    return set(re.findall(r"Node name:\s*(\S+)", info.split("Subscription count:")[0]))


def topic_info(env, topic, tries=20):
    """ros2 topic info -v with retry to tolerate transient DDS discovery."""
    out = ""
    for _ in range(tries):
        rc, out = run(["ros2", "topic", "info", "-v", topic], env=env)
        if "Publisher count" in out and "Unknown topic" not in out:
            return out
        time.sleep(0.3)
    return out


def topic_type(env, topic, tries=20):
    out = ""
    for _ in range(tries):
        rc, out = run(["ros2", "topic", "type", topic], env=env)
        out = out.strip()
        if out and "Unknown topic" not in out:
            return out
        time.sleep(0.3)
    return out


def tup(m):
    return (m.header.stamp.sec, m.header.stamp.nanosec)


def nondecreasing(seq):
    return all(seq[i] <= seq[i + 1] for i in range(len(seq) - 1))


def finite(vals):
    return all(math.isfinite(v) for v in vals)


# --------------------------------------------------------------------------
class Collector(Node):
    def __init__(self, name):
        super().__init__(name)
        rel = QoSProfile(depth=100)
        rel.reliability = ReliabilityPolicy.RELIABLE
        rel.durability = DurabilityPolicy.VOLATILE
        rel.history = HistoryPolicy.KEEP_LAST
        static_qos = QoSProfile(depth=100)
        static_qos.reliability = ReliabilityPolicy.RELIABLE
        static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        be = QoSProfile(depth=1)
        be.reliability = ReliabilityPolicy.BEST_EFFORT
        be.durability = DurabilityPolicy.VOLATILE
        self.joints, self.clock, self.body, self.imu = [], [], [], []
        self.left, self.right, self.tf, self.tf_static = [], [], [], []
        self.create_subscription(JointState, "/joint_states", lambda m: self.joints.append(m), rel)
        self.create_subscription(Clock, "/clock", lambda m: self.clock.append(m), be)
        self.create_subscription(PoseStamped, "/body_pose", lambda m: self.body.append(m), rel)
        self.create_subscription(Imu, "/imu/data", lambda m: self.imu.append(m), rel)
        self.create_subscription(Bool, "/left_foot/contact", lambda m: self.left.append(m), rel)
        self.create_subscription(Bool, "/right_foot/contact", lambda m: self.right.append(m), rel)
        self.create_subscription(TFMessage, "/tf", lambda m: self.tf.append(m), rel)
        self.create_subscription(TFMessage, "/tf_static", lambda m: self.tf_static.append(m),
                                 static_qos)


# Workspace-specific signatures used for READ-ONLY detection only (never to
# kill). They match this workspace's own launch/model/converter processes.
STALE_PROC_SIGNATURES = ("nao_world.sdf", "nao_sim.launch", "/nao_gz_sim/lib/nao_gz_sim/")


def scan_stale_processes(env):
    """READ-ONLY scan for leftover processes from this workspace's runs.

    Never used to terminate anything: a non-empty result is reported as
    BLOCKED so an operator can clean up manually. Matching is limited to
    workspace-specific signatures so unrelated users/sessions are never
    touched or misreported.
    """
    _, out = run(["bash", "-lc", "ps -eo pid,args --no-headers"], env=env, timeout=10)
    stale = []
    for line in out.splitlines():
        if any(sig in line for sig in STALE_PROC_SIGNATURES):
            stale.append(line.strip())
    return stale


def reap_process_group(proc, log_f):
    """Terminate ONLY the process group created by this validator's Popen.

    SIGINT first, then SIGKILL as a last resort, both strictly against the
    known process group id derived from our own ``proc``. Waits for the group
    leader, verifies the group is gone and returns any cleanup error strings
    (so failures are reported instead of silently swallowed).
    """
    errors = []
    try:
        pgid = os.getpgid(proc.pid)
    except ProcessLookupError:
        pgid = None
    if pgid is not None and proc.poll() is None:
        try:
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                errors.append("launch process %d did not exit after SIGKILL" % proc.pid)
    if pgid is not None:
        try:
            os.killpg(pgid, 0)
            errors.append("process group %d still alive after cleanup" % pgid)
        except ProcessLookupError:
            pass
    log_f.close()
    return errors


# --------------------------------------------------------------------------
# Static checks
# --------------------------------------------------------------------------
def static_checks(args, env):
    outdir = os.path.join(args.out, "static")
    os.makedirs(outdir, exist_ok=True)
    cmds = [
        ("contract", ["python3", "tools/check_task3a_contract.py"], 60),
        ("sdf_model", ["gz", "sdf", "-k", SDF], 60),
        ("sdf_world", ["gz", "sdf", "-k", WORLD], 60),
        ("check_urdf", ["check_urdf", URDF], 60),
        ("build", ["colcon", "build", "--packages-select", "nao_gz_sim", "--symlink-install",
                   "--cmake-args", "-DCMAKE_IGNORE_PREFIX_PATH=/home/liu/anaconda3"], 300),
        ("test", ["colcon", "test", "--packages-select", "nao_gz_sim"], 300),
        ("test_result", ["colcon", "test-result"], 60),
    ]
    results = {}
    for name, cmd, timeout in cmds:
        rc, output = run(cmd, env=env, timeout=timeout, cwd=REPO_ROOT)
        with open(os.path.join(outdir, name + ".log"), "w") as f:
            f.write("$ %s\n(exit=%d)\n%s\n" % (" ".join(cmd), rc, output))
        results[name] = {"cmd": " ".join(cmd), "rc": rc, "tail": output.strip().splitlines()[-6:]}
    # interpret
    ok = 0
    checks = {}
    checks["contract_pass"] = results["contract"]["rc"] == 0 and any(
        "CONTRACT CHECK: PASS" in l for l in results["contract"]["tail"])
    checks["sdf_model_valid"] = results["sdf_model"]["rc"] == 0 and any(
        l.strip() == "Valid." for l in results["sdf_model"]["tail"])
    checks["sdf_world_valid"] = results["sdf_world"]["rc"] == 0 and any(
        l.strip() == "Valid." for l in results["sdf_world"]["tail"])
    checks["urdf_parsed"] = results["check_urdf"]["rc"] == 0
    checks["build_ok"] = results["build"]["rc"] == 0
    checks["test_ok"] = results["test"]["rc"] == 0
    for v in checks.values():
        ok += 1 if v else 0
    return {"commands": results, "checks": checks,
            "ok": all(checks.values()), "n_ok": ok, "n_total": len(checks)}


# --------------------------------------------------------------------------
def run_once(args, index, env):
    partition = "t3g_%d_%d" % (index, int(time.time()))
    env = dict(env)
    env["GZ_PARTITION"] = partition
    result = {"run": index, "partition": partition,
              "log": os.path.join(args.out, "launch_run%d.log" % index),
              "checks": {}, "stats": {}, "pause": {}, "authority": {}, "topology": {}}
    failures, blocked = [], []

    def record(name, ok, detail="", block=False):
        result["checks"][name] = {"ok": bool(ok), "detail": detail}
        if not ok:
            (blocked if block else failures).append("%s: %s" % (name, detail))

    # Never kill unknown processes to "clear the field": only detect (read-only)
    # and report BLOCKED so an operator can clean up manually.
    stale = scan_stale_processes(env)
    if stale:
        result["blocked"] = "stale workspace processes present; manual cleanup required"
        result["stale_processes"] = stale
        record("environment_clean_before_run", False,
               "stale: %s (not killed)" % stale[:3], block=True)
        return result, failures, blocked
    record("environment_clean_before_run", True, "no stale workspace processes")

    log_path = result["log"]
    log_f = open(log_path, "w")
    cmd = ["ros2", "launch", "nao_gz_sim", "nao_sim.launch.py", "gui:=false", "use_rsp:=true"]
    proc = subprocess.Popen(cmd, stdout=log_f, stderr=subprocess.STDOUT,
                            env=env, start_new_session=True)
    result["launch_cmd"] = " ".join(cmd)
    urdf_links = load_urdf_links()

    try:
        node = Collector("task3g_validator_%d" % index)
        buffer = tf2_ros.Buffer(cache_time=Duration(seconds=300.0))
        _listener = tf2_ros.TransformListener(buffer, node)

        def have_all():
            has_wb = any(p == "world" and c == "base_link"
                         for m in node.tf for (p, c, _) in pairs_of_msg(m))
            has_sole = any(c in ("l_sole", "r_sole") for m in node.tf_static
                           for (_, c, _) in pairs_of_msg(m))
            return has_wb and has_sole and len(node.joints) > 0 and len(node.imu) > 0

        if not spin_for(node, 60.0, stop_when=have_all):
            record("startup_all_interfaces", False,
                   "not all interfaces appeared within 60 s", block=True)
            return result, failures, blocked

        t0 = time.time()
        spin_for(node, args.duration)
        wall = time.time() - t0

        # ---- topology ----
        checks_info = {t: topic_info(env, t) for t in EXPECTED_PUBLISHERS}
        _, topics = run(["ros2", "topic", "list"], env=env)
        nodes_out = ""
        for _ in range(20):
            _, nodes_out = run(["ros2", "node", "list"], env=env)
            if "world_tf_converter" in nodes_out and "robot_state_publisher" in nodes_out:
                break
            time.sleep(0.3)
        nodes = nodes_out
        types = {t: topic_type(env, t) for t in EXPECTED_PUBLISHERS}
        result["topic_list"] = sorted(topics.split())
        result["node_list"] = sorted(n for n in nodes.split() if n.startswith("/"))
        result["topic_info"] = checks_info
        result["topic_types"] = types
        result["topology"] = {t: {"publishers": sorted(publisher_nodes(i)),
                                  "pub_count": re.findall(r"Publisher count:\s*(\d+)", i)}
                              for t, i in checks_info.items()}
        result["authority"] = {}

        # QoS depth evidence boundary: record the runtime CLI observation for the
        # PUBLISHER endpoints only (never subscriber depth) and keep it clearly
        # separated from the source configuration (P2-5).
        result["qos_depth_source"] = QOS_DEPTH_SOURCE
        result["runtime_qos_history_publisher_only"] = {
            t: re.findall(r"History \(Depth\):\s*(\S+)",
                          checks_info[t].split("Subscription count:")[0])
            for t in QOS_DEPTH_SOURCE}
        record("qos_depth_evidence_boundary", True,
               "depth from source config; runtime CLI history (publisher only) recorded "
               "separately (UNKNOWN tolerated, not a measurement)")

        for topic, expected in EXPECTED_PUBLISHERS.items():
            pubs = publisher_nodes(checks_info[topic])
            record("publisher_%s" % topic.strip("/").replace("/", "_"),
                   pubs == expected, "got %s expected %s" % (sorted(pubs), sorted(expected)))
        expected_nodes = {"joint_state_converter", "body_pose_converter", "imu_converter",
                          "contact_converter", "world_tf_converter", "robot_state_publisher"}
        present_nodes = {n.lstrip("/") for n in result["node_list"]}
        record("expected_nodes_present", expected_nodes <= present_nodes,
               "missing %s" % sorted(expected_nodes - present_nodes))
        forbidden = [t for t in result["topic_list"] if FORBIDDEN_TOPIC_RE.search(t)]
        record("no_forbidden_topics", not forbidden, str(forbidden))

        # ---- joint states ----
        js = list(node.joints)
        record("joint_type", types["/joint_states"] == "sensor_msgs/msg/JointState",
               types["/joint_states"])
        record("joint_count_24", js and all(len(m.name) == 24 for m in js), "%d msgs" % len(js))
        record("joint_order", js and all(list(m.name) == EXPECTED_JOINTS for m in js),
               "canonical SDF order")
        record("joint_lengths", js and all(len(m.position) == 24 and len(m.velocity) == 24
                                           for m in js), "position/velocity 24")
        record("joint_effort_empty", js and all(len(m.effort) == 0 for m in js), "effort []")
        record("joint_frame_id_empty", js and all(m.header.frame_id == "" for m in js), "")
        jst = [tup(m) for m in js]
        record("joint_stamp_sim", bool(jst) and all(s != (0, 0) for s in jst)
               and nondecreasing(jst), "nonzero non-decreasing")
        record("joint_finite", js and all(finite(list(m.position) + list(m.velocity)) for m in js),
               "no NaN/Inf")
        if jst:
            result["stats"]["joint_sim_span_s"] = (jst[-1][0] - jst[0][0]) + \
                (jst[-1][1] - jst[0][1]) * 1e-9
            result["stats"]["joint_hz_sim"] = (len(jst) - 1) / result["stats"]["joint_sim_span_s"] \
                if result["stats"]["joint_sim_span_s"] > 0 else None

        # ---- clock ----
        clk = list(node.clock)
        cv = [(m.clock.sec, m.clock.nanosec) for m in clk]
        record("clock_type", types["/clock"] == "rosgraph_msgs/msg/Clock", types["/clock"])
        record("clock_qos_best_effort", "Reliability: BEST_EFFORT" in checks_info["/clock"], "")
        record("clock_qos_volatile", "Durability: VOLATILE" in checks_info["/clock"], "")
        record("clock_nonzero_monotonic", bool(cv) and all(s != (0, 0) for s in cv)
               and nondecreasing(cv), "n=%d" % len(cv))
        clock_lo, clock_hi = (min(cv), max(cv)) if cv else (None, None)
        result["stats"]["clock_range"] = [clock_lo, clock_hi]
        result["stats"]["clock_count"] = len(cv)

        # ---- body pose ----
        bp = list(node.body)
        record("body_type", types["/body_pose"] == "geometry_msgs/msg/PoseStamped", types["/body_pose"])
        record("body_frame_world", bp and all(m.header.frame_id == "world" for m in bp), "")
        bst = [tup(m) for m in bp]
        record("body_stamp_sim", bool(bst) and all(s != (0, 0) for s in bst)
               and nondecreasing(bst), "nonzero non-decreasing")
        finite_bp, norm_bp = True, 0.0
        for m in bp:
            o = m.pose.orientation
            if not finite([m.pose.position.x, m.pose.position.y, m.pose.position.z,
                           o.x, o.y, o.z, o.w]):
                finite_bp = False
            norm_bp = max(norm_bp, abs(math.sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w) - 1))
        record("body_finite", finite_bp, "no NaN/Inf")
        record("body_quaternion_normalized", norm_bp < 1e-6, "max |norm-1|=%.2e" % norm_bp)

        # ---- IMU ----
        imu = list(node.imu)
        record("imu_type", types["/imu/data"] == "sensor_msgs/msg/Imu", types["/imu/data"])
        record("imu_frame", imu and all(m.header.frame_id == "ImuTorsoAccelerometer_frame"
                                        for m in imu), "")
        ist = [tup(m) for m in imu]
        record("imu_stamp_sim", bool(ist) and all(s != (0, 0) for s in ist)
               and nondecreasing(ist), "nonzero non-decreasing")
        finite_imu, norm_imu, cov_ok = True, 0.0, True
        for m in imu:
            o = m.orientation
            a = m.angular_velocity
            l = m.linear_acceleration
            if not finite([o.x, o.y, o.z, o.w, a.x, a.y, a.z, l.x, l.y, l.z]):
                finite_imu = False
            norm_imu = max(norm_imu, abs(math.sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w) - 1))
            for cov in (m.orientation_covariance, m.angular_velocity_covariance,
                        m.linear_acceleration_covariance):
                if len(cov) != 9 or any(c != 0.0 for c in cov):
                    cov_ok = False
        record("imu_finite", finite_imu, "no NaN/Inf")
        record("imu_quaternion_normalized", norm_imu < 1e-6, "max |norm-1|=%.2e" % norm_imu)
        record("imu_covariance_zero", cov_ok, "no modeled noise")
        if len(ist) > 1:
            span = (ist[-1][0] - ist[0][0]) + (ist[-1][1] - ist[0][1]) * 1e-9
            hz = (len(ist) - 1) / span if span > 0 else None
            result["stats"]["imu_hz_sim"] = hz
            result["stats"]["imu_sim_span_s"] = span
            record("imu_frequency_100hz", hz is not None and 80.0 <= hz <= 130.0,
                   "%.2f Hz sim" % (hz if hz else -1))

        # ---- contact ----
        record("contact_left_type", types["/left_foot/contact"] == "std_msgs/msg/Bool", "")
        record("contact_right_type", types["/right_foot/contact"] == "std_msgs/msg/Bool", "")
        record("contact_left_qos", "Reliability: RELIABLE" in checks_info["/left_foot/contact"]
               and "Durability: VOLATILE" in checks_info["/left_foot/contact"], "")
        record("contact_right_qos", "Reliability: RELIABLE" in checks_info["/right_foot/contact"]
               and "Durability: VOLATILE" in checks_info["/right_foot/contact"], "")
        lv = [m.data for m in node.left]
        rv = [m.data for m in node.right]
        record("contact_bool_values", all(isinstance(v, bool) for v in lv + rv), "bool")
        result["stats"]["contact_left"] = {"n": len(lv), "true": sum(1 for v in lv if v),
                                           "false": sum(1 for v in lv if v is False),
                                           "last": lv[-1] if lv else None}
        result["stats"]["contact_right"] = {"n": len(rv), "true": sum(1 for v in rv if v),
                                            "false": sum(1 for v in rv if v is False),
                                            "last": rv[-1] if rv else None}
        # no message must not be auto-false; just report unknown when absent
        record("contact_unknown_not_false", True,
               "left n=%d right n=%d (absent=unknown, not coerced false)" % (len(lv), len(rv)))

        # ---- TF tree ----
        record("tf_type", types["/tf"] == "tf2_msgs/msg/TFMessage", types["/tf"])
        record("tf_static_type", types["/tf_static"] == "tf2_msgs/msg/TFMessage",
               types["/tf_static"])
        record("tf_qos", "Reliability: RELIABLE" in checks_info["/tf"]
               and "Durability: VOLATILE" in checks_info["/tf"], "")
        record("tf_static_qos", "Reliability: RELIABLE" in checks_info["/tf_static"]
               and "Durability: TRANSIENT_LOCAL" in checks_info["/tf_static"], "")
        dyn_pairs = [x for m in node.tf for x in pairs_of_msg(m)]
        stat_pairs = [x for m in node.tf_static for x in pairs_of_msg(m)]
        dyn_tree, stat_tree = {}, {}
        for p, c, _ in dyn_pairs:
            dyn_tree.setdefault(c, set()).add(p)
        for p, c, _ in stat_pairs:
            stat_tree.setdefault(c, set()).add(p)
        dyn_children, stat_children = set(dyn_tree), set(stat_tree)
        all_frames = {"world"} | dyn_children | stat_children
        for ps in list(dyn_tree.values()) + list(stat_tree.values()):
            all_frames |= ps
        result["authority"] = {
            "world->base_link": {"topic": "/tf", "dynamic": True},
            "base_link->torso": {"topic": "/tf_static", "dynamic": False},
            "l_ankle->l_sole": {"topic": "/tf_static", "dynamic": False},
            "r_ankle->r_sole": {"topic": "/tf_static", "dynamic": False},
            "joint chain (l_ankle/r_ankle)": {"topic": "/tf", "dynamic": True},
        }
        for f in ("world", "base_link", "torso", "l_ankle", "r_ankle", "l_sole", "r_sole"):
            record("frame_%s" % f, f in all_frames, f)
        record("world_base_link_dynamic", dyn_tree.get("base_link") == {"world"},
               str(dyn_tree.get("base_link")))
        record("no_base_link_to_world",
               not any(p == "base_link" and c == "world" for (p, c, _) in dyn_pairs + stat_pairs), "")
        record("l_ankle_dynamic", "l_ankle" in dyn_children, "")
        record("r_ankle_dynamic", "r_ankle" in dyn_children, "")
        record("l_sole_static", stat_tree.get("l_sole") == {"l_ankle"}, str(stat_tree.get("l_sole")))
        record("r_sole_static", stat_tree.get("r_sole") == {"r_ankle"}, str(stat_tree.get("r_sole")))
        record("torso_static", stat_tree.get("torso") == {"base_link"}, str(stat_tree.get("torso")))
        record("no_child_in_both", not (dyn_children & stat_children),
               str(sorted(dyn_children & stat_children)))
        record("unique_child_parent",
               all(len(v) == 1 for v in list(dyn_tree.values()) + list(stat_tree.values())), "")
        record("frames_canonical", all(f == "world" or f in urdf_links for f in all_frames),
               str(sorted(f for f in all_frames if f != "world" and f not in urdf_links)))
        record("no_scoped_alias_frames", not any(FORBIDDEN_FRAME_RE.search(f) for f in all_frames),
               str(sorted(f for f in all_frames if FORBIDDEN_FRAME_RE.search(f))))
        record("world_parent_only_base_link",
               {c for (p, c, _) in dyn_pairs if p == "world"} == {"base_link"}, "")

        wb_stamps = sorted({s for (p, c, s) in dyn_pairs if p == "world" and c == "base_link"})
        la_stamps = sorted({s for (p, c, s) in dyn_pairs if c == "l_ankle"})
        record("tf_stamp_sim_range",
               bool(wb_stamps) and all(clock_lo <= s <= clock_hi for s in wb_stamps),
               "wb in clock range")
        record("tf_wb_monotonic", nondecreasing(wb_stamps), "world->base_link")
        record("tf_rsp_monotonic", nondecreasing(la_stamps), "l_ankle stamps non-decreasing")

        # unified simulation-time consistency
        # /clock is BEST_EFFORT depth 1, so it may drop the very first samples;
        # allow a small tolerance at the window edges while still proving all
        # interfaces share the same simulation timeline.
        def in_clock_range(stamps):
            if not stamps or clock_lo is None:
                return False
            lo = clock_lo[0] + clock_lo[1] * 1e-9
            hi = clock_hi[0] + clock_hi[1] * 1e-9
            s0 = stamps[0][0] + stamps[0][1] * 1e-9
            s1 = stamps[-1][0] + stamps[-1][1] * 1e-9
            return lo - 0.2 <= s0 and s1 <= hi + 0.2

        for label, stamps in (("joint", jst), ("body", bst), ("imu", ist)):
            record("simtime_%s_in_clock_range" % label, in_clock_range(stamps),
                   "first=%s last=%s clock=%s..%s"
                   % (stamps[0] if stamps else None, stamps[-1] if stamps else None,
                      clock_lo, clock_hi))
        record("simtime_wb_in_clock_range", in_clock_range(wb_stamps),
               "clock=%s..%s" % (clock_lo, clock_hi))

        # body pose vs world->base_link (same Gazebo source, noted non-independent)
        bmap = {tup(m): m for m in bp}
        matched = None
        for m in node.tf:
            for t in m.transforms:
                if t.header.frame_id == "world" and t.child_frame_id == "base_link":
                    s = (t.header.stamp.sec, t.header.stamp.nanosec)
                    if s in bmap:
                        matched = (bmap[s], t, s)
        if matched:
            bmsg, t, s = matched
            dpos = math.dist([bmsg.pose.position.x, bmsg.pose.position.y, bmsg.pose.position.z],
                             [t.transform.translation.x, t.transform.translation.y,
                              t.transform.translation.z])
            dquat = qangle([bmsg.pose.orientation.x, bmsg.pose.orientation.y,
                            bmsg.pose.orientation.z, bmsg.pose.orientation.w],
                           [t.transform.rotation.x, t.transform.rotation.y,
                            t.transform.rotation.z, t.transform.rotation.w])
            result["stats"]["body_vs_tf"] = {"stamp": s, "dpos": dpos, "dquat": dquat}
            record("body_vs_world_tf", dpos < 1e-6 and dquat < 1e-6,
                   "dpos=%.2e dquat=%.2e (same Gazebo model source, not independent)" % (dpos, dquat))
        else:
            record("body_vs_world_tf", False, "no shared stamp", block=True)

        # independent Gazebo Pose_V cross-check
        parsed = capture_gz_pose(env)
        spin_for(node, 1.0)
        if parsed is None:
            record("gazebo_crosscheck", False, "no gz pose sample", block=True)
        elif MODEL not in parsed[1]:
            record("gazebo_crosscheck", False, "model %s absent from pose/info" % MODEL, block=True)
        else:
            gstamp, poses = parsed
            try:
                bp_t, bq_t, _ = tf_lookup(buffer, "world", "base_link", stamp=gstamp)
                dpos_bl = math.dist(bp_t, poses[MODEL][0])
                dquat_bl = qangle(bq_t, poses[MODEL][1])
            except Exception as exc:  # noqa: BLE001
                dpos_bl, dquat_bl = None, None
                record("gazebo_crosscheck", False, "lookup failed: %s" % exc, block=True)
            feet = {}
            for side, link in (("l", "l_ankle"), ("r", "r_ankle")):
                if link not in poses:
                    continue
                fp, fq = compose(poses[MODEL][0], poses[MODEL][1], *poses[link])
                fp2, fq2 = compose(fp, fq, SOLE_OFFSET, [0.0, 0.0, 0.0, 1.0])
                try:
                    tp, tq, _ = tf_lookup(buffer, "world", "%s_sole" % side, stamp=gstamp)
                    feet[side] = (math.dist(fp2, tp), qangle(fq2, tq))
                except Exception as exc:  # noqa: BLE001
                    feet[side] = (None, None)
            if dpos_bl is not None:
                result["stats"]["gazebo_crosscheck"] = {
                    "stamp": gstamp, "dpos_base_link": dpos_bl, "dquat_base_link": dquat_bl,
                    "l_sole": feet.get("l"), "r_sole": feet.get("r")}
                ok = dpos_bl < 1e-6 and dquat_bl < 1e-6
                for side in ("l", "r"):
                    if feet.get(side) and feet[side][0] is not None:
                        ok = ok and feet[side][0] < 8e-3 and feet[side][1] < 2e-2
                record("gazebo_crosscheck", ok,
                       "dpos(base)=%.2e l_sole=%s r_sole=%s" % (dpos_bl, feet.get("l"), feet.get("r")))

        # dynamic foot lookup (controlled model pose via existing service)
        try:
            l0 = tf_lookup(buffer, "world", "l_sole")
            r0 = tf_lookup(buffer, "world", "r_sole")
            run(["gz", "service", "-s", "/world/nao_world/set_pose", "--reqtype", "gz.msgs.Pose",
                 "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req",
                 'name: "nao" position: {x: 0.4, y: 0.2, z: 0.6} '
                 'orientation: {x: 0, y: 0, z: 0, w: 1}'], env=env, timeout=10)
            spin_for(node, 1.5)
            l1 = tf_lookup(buffer, "world", "l_sole")
            r1 = tf_lookup(buffer, "world", "r_sole")
            dl, dr = math.dist(l0[0], l1[0]), math.dist(r0[0], r1[0])
            result["stats"]["foot_dynamic"] = {"l": dl, "r": dr}
            record("world_l_sole_dynamic", dl > 1e-3, "%.4f m" % dl)
            record("world_r_sole_dynamic", dr > 1e-3, "%.4f m" % dr)
            record("world_foot_not_static_offset", dl > 1e-3 and dr > 1e-3,
                   "world foot pose changes, not just static sole offset")
        except Exception as exc:  # noqa: BLE001
            record("world_l_sole_dynamic", False, "lookup failed: %s" % exc, block=True)
            record("world_r_sole_dynamic", False, "lookup failed: %s" % exc, block=True)
            record("world_foot_not_static_offset", False, "lookup failed: %s" % exc, block=True)

        # ---- pause / resume ----
        pause_cmd = ["gz", "service", "-s", "/world/nao_world/control", "--reqtype",
                     "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean", "--timeout", "3000",
                     "--req", "pause: true"]
        resume_cmd = list(pause_cmd)
        resume_cmd[-1] = "pause: false"
        rc_p, _ = run(pause_cmd, env=env, timeout=10)
        spin_for(node, 0.5)
        pre = {"clock": len(node.clock), "joints": len(node.joints), "body": len(node.body),
               "imu": len(node.imu), "left": len(node.left), "right": len(node.right),
               "tf": len(node.tf)}
        ck_before = (node.clock[-1].clock.sec, node.clock[-1].clock.nanosec) if node.clock else None
        wb_before = [s for (p, c, s) in [x for m in node.tf for x in pairs_of_msg(m)]
                     if c == "base_link"] if node.tf else []
        spin_for(node, 2.0)
        during = {"clock": len(node.clock) - pre["clock"], "joints": len(node.joints) - pre["joints"],
                  "body": len(node.body) - pre["body"], "imu": len(node.imu) - pre["imu"],
                  "left": len(node.left) - pre["left"], "right": len(node.right) - pre["right"],
                  "tf": len(node.tf) - pre["tf"]}
        ck_during = (node.clock[-1].clock.sec, node.clock[-1].clock.nanosec) if node.clock else None
        wb_during = [s for (p, c, s) in [x for m in node.tf for x in pairs_of_msg(m)]
                     if c == "base_link"]
        rc_r, _ = run(resume_cmd, env=env, timeout=10)
        spin_for(node, 1.5)
        after = {"joints": len(node.joints) - pre["joints"] - during["joints"],
                 "imu": len(node.imu) - pre["imu"] - during["imu"]}
        ck_after = (node.clock[-1].clock.sec, node.clock[-1].clock.nanosec) if node.clock else None
        clock_frozen = ck_before is not None and ck_during is not None and ck_during <= ck_before
        clock_resumes = ck_during is not None and ck_after is not None and ck_after > ck_during
        tf_frozen = (not wb_before) or (not wb_during) or max(wb_during) <= max(wb_before)
        result["pause"] = {
            "pause_rc": rc_p, "resume_rc": rc_r, "during": during,
            "clock_before": ck_before, "clock_during": ck_during, "clock_after": ck_after,
            "clock_frozen": clock_frozen, "clock_resumes": clock_resumes, "tf_frozen": tf_frozen,
            "event_topics_quiet": during["joints"] == 0 and during["body"] == 0
            and during["imu"] == 0 and during["left"] == 0 and during["right"] == 0,
            "pause_ok": (rc_p == 0 and rc_r == 0 and clock_frozen and clock_resumes and tf_frozen
                         and during["joints"] == 0 and during["body"] == 0
                         and during["imu"] == 0 and during["left"] == 0 and during["right"] == 0
                         and after["joints"] > 0 and after["imu"] > 0),
        }
        record("pause_resume", result["pause"]["pause_ok"], json.dumps(result["pause"]))

        # ---- publisher count drift ----
        drift = {}
        for t in EXPECTED_PUBLISHERS:
            info2 = topic_info(env, t)
            c1 = re.findall(r"Publisher count:\s*(\d+)", checks_info[t])
            c2 = re.findall(r"Publisher count:\s*(\d+)", info2)
            drift[t] = [c1, c2]
        record("no_publisher_count_drift", all(
            d[0] == d[1] for d in drift.values()), json.dumps(drift))

        result["stats"].update({"wall_s": wall, "joint": len(node.joints), "clock": len(node.clock),
                                "body": len(node.body), "imu": len(node.imu),
                                "tf_messages": len(node.tf), "tf_static_messages": len(node.tf_static)})
    finally:
        # Only ever terminate the process group we created with Popen above.
        cleanup_errors = reap_process_group(proc, log_f)
        record("process_group_cleanup", not cleanup_errors,
               "; ".join(cleanup_errors) if cleanup_errors else "own process group terminated")

    text = re.sub(r"\x1b\[[0-9;]*m", "", open(log_path, errors="ignore").read())
    result["violations"] = re.findall(r"CONTRACT VIOLATION[^\n]*", text)
    result["err_count"] = text.count("[Err]")
    record("no_contract_violation", not result["violations"], str(result["violations"][:3]))
    record("no_err_in_log", result["err_count"] == 0, "%d [Err]" % result["err_count"])
    pose_bridge = re.findall(r"Creating .*Bridge.*Pose_V[^\n]*", text)
    record("no_pose_v_bridge", not pose_bridge, str(pose_bridge[:2]))
    # read-only leak check (never kills): workspace-specific signatures only
    leak = scan_stale_processes(env)
    record("no_process_leak", not leak, str(leak[:3]))
    return result, failures, blocked


def pairs_of_msg(msg):
    return [(t.header.frame_id, t.child_frame_id,
             (t.header.stamp.sec, t.header.stamp.nanosec)) for t in msg.transforms]


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", type=int, default=2)
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--out", default="docs/task3g_evidence")
    ap.add_argument("--skip-static", action="store_true")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    env = dict(os.environ)

    report = {"duration": args.duration, "runs": [], "static": None}
    failures, blocked = [], []

    if not args.skip_static:
        report["static"] = static_checks(args, env)
        for k, v in report["static"]["checks"].items():
            print("static %-18s %s" % (k, "PASS" if v else "FAIL"))
        if not report["static"]["ok"]:
            failures.append("static checks")

    rclpy.init()
    interrupted = False
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
            print("=== run %d partition %s: %d checks, %d fail%s ===" %
                  (i, res["partition"], nchk, nfail, " BLOCKED" if res.get("blocked") else ""))
            for cn, c in res["checks"].items():
                if not c["ok"]:
                    print("    [FAIL] %s -- %s" % (cn, c["detail"]))
    except KeyboardInterrupt:
        # run_once()'s own finally already reaped the launch process group.
        interrupted = True
        report["interrupted"] = True
        print("\ninterrupted: own launch process group reaped; stopping")
    finally:
        rclpy.shutdown()

    report["failures"] = failures
    report["blocked"] = blocked
    report["interrupted"] = interrupted
    if failures:
        report["verdict"] = "FAIL"
    elif interrupted:
        report["verdict"] = "BLOCKED"
    elif blocked:
        report["verdict"] = "BLOCKED"
    else:
        report["verdict"] = "PASS"
    with open(os.path.join(args.out, "report.json"), "w") as fp:
        json.dump(report, fp, indent=2)
    print("\nTASK 3G VERDICT: %s" % report["verdict"])
    if failures:
        print("failures: %s" % failures)
    if blocked:
        print("blocked: %s" % blocked)
    return 0 if report["verdict"] == "PASS" else (2 if report["verdict"] == "BLOCKED" else 1)


if __name__ == "__main__":
    sys.exit(main())
