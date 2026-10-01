#!/usr/bin/python3
"""Task 3E validation: minimal left/right foot contact interface.

Part 1 (actual launch): runs ``nao_sim.launch.py`` headless and verifies the
frozen public Bool topics, the internal raw ``Contacts`` topics, QoS,
publisher ownership, publish rate, pause/resume (no wall timer), forbidden
topics, contract violations and Task 3B/3C/3D regressions over a 30 s window.

Part 2 (controlled fixture): starts a Gazebo world + bridge + converter and
spawns a validation-only fixture built from the *canonical* SDF (optionally
with a fixed base->world joint and strongly damped joints so a pose can be
held deterministically). It then exercises:
  * double-foot contact            -> both true
  * left-only contact              -> left true, right false
  * right-only contact             -> right true, left false
  * off-ground (ground moved away) -> both false, restore -> both true
  * non-foot contact (free settle) -> head/tibia contact, both feet false
Every scenario is checked with the raw scoped collision names and the public
Bool values.

Exit code 0 = PASS, 1 = FAIL, 2 = BLOCKED.

Run with the ROS Python interpreter, from the workspace root:

    /usr/bin/python3 tools/validate_task3e_foot_contact.py \
        --out docs/task3e_evidence --duration 30

This tool never modifies the canonical SDF/URDF; fixtures are written under
the output directory only.
"""

import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import Contacts

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CANONICAL_SDF = os.path.join(
    REPO_ROOT, "src", "nao", "nao_description", "urdf", "nao_simple.sdf")

LEFT_TOPIC = "/left_foot/contact"
RIGHT_TOPIC = "/right_foot/contact"
LEFT_RAW = "/nao/left_foot_contact_raw"
RIGHT_RAW = "/nao/right_foot_contact_raw"
BODY_RAW = "/nao/body_contact_raw"

FORBIDDEN_TOPIC_RE = re.compile(
    r"/contacts$|_contacts$|fsr|pressure|bumper|wrench"
    r"|joint_command|ros2_control|controller_manager|/imu/(gyro|accel)")
# NOTE (Task 3F): /tf and /tf_static were forbidden for Task 3E (no TF yet) and
# are now the frozen Task 3F outputs, so they are no longer excluded here. The
# contact interface (/left_foot/contact, /right_foot/contact) is unchanged.

# Deterministic fixture geometry (measured from the canonical model).
DOUBLE_Z = 0.333
SINGLE_Z = 0.315
SINGLE_ROLL = 0.2
GROUND_DROP = -0.012
OFF_GROUND_Z = -5.0

BODY_DIAG = [
    ("base_link", "base_link_fixed_joint_lump__torso_collision"),
    ("Head", "Head_collision"),
    ("LPelvis", "LPelvis_collision"), ("RPelvis", "RPelvis_collision"),
    ("LHip", "LHip_collision"), ("RHip", "RHip_collision"),
    ("LThigh", "LThigh_collision"), ("RThigh", "RThigh_collision"),
    ("LTibia", "LTibia_collision"), ("RTibia", "RTibia_collision"),
    ("LAnklePitch_link", "LAnklePitch_link_collision"),
    ("RAnklePitch_link", "RAnklePitch_link_collision"),
    ("LBicep", "LBicep_collision"), ("RBicep", "RBicep_collision"),
    ("LElbow", "LElbow_collision"), ("RElbow", "RElbow_collision"),
    ("LForeArm", "LForeArm_collision"), ("RForeArm", "RForeArm_collision"),
]


# --------------------------------------------------------------------------
# fixture generation (validation only; never touches the canonical file)
# --------------------------------------------------------------------------
def insert_before_link_end(text, link, payload):
    i = text.index("<link name='%s'>" % link)
    j = text.index("</link>", i)
    return text[:j] + payload + text[j:]


def make_fixture(canonical, out, fixed=False, damp=False, diag=False, freeze=False):
    text = open(canonical).read()
    if fixed:
        joint = ("    <joint name='task3e_world_hold' type='fixed'>\n"
                 "      <parent>world</parent>\n"
                 "      <child>base_link</child>\n"
                 "    </joint>\n")
        idx = text.index("<model name=")
        gt = text.index(">", idx) + 1
        text = text[:gt] + "\n" + joint + text[gt:]
    if freeze:
        text = text.replace("type='revolute'", "type='fixed'")
    if damp:
        text = text.replace("<damping>0.2</damping>", "<damping>100000</damping>")
    if diag:
        for k, (link, col) in enumerate(BODY_DIAG):
            sensor = ("      <sensor name='task3e_diag_%d' type='contact'>\n"
                      "        <always_on>1</always_on>\n"
                      "        <update_rate>100</update_rate>\n"
                      "        <contact>\n"
                      "          <collision>%s</collision>\n"
                      "          <topic>%s</topic>\n"
                      "        </contact>\n"
                      "      </sensor>\n" % (k, col, BODY_RAW))
            text = insert_before_link_end(text, link, sensor)
    with open(out, "w") as f:
        f.write(text)
    return out


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------
def run(cmd, env=None, timeout=30):
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


class Collector(Node):
    def __init__(self, name="task3e_validator", enable_body=False, enable_regression=True):
        super().__init__(name)
        rel = QoSProfile(depth=50)
        rel.reliability = ReliabilityPolicy.RELIABLE
        rel.durability = DurabilityPolicy.VOLATILE
        rel.history = HistoryPolicy.KEEP_LAST
        be = QoSProfile(depth=1)
        be.reliability = ReliabilityPolicy.BEST_EFFORT
        be.durability = DurabilityPolicy.VOLATILE
        self.left, self.right = [], []
        self.left_raw, self.right_raw, self.body_raw = [], [], []
        self.clock = []
        self.joints, self.body, self.imu = [], [], []
        self.create_subscription(Bool, LEFT_TOPIC, lambda m: self.left.append(m), rel)
        self.create_subscription(Bool, RIGHT_TOPIC, lambda m: self.right.append(m), rel)
        self.create_subscription(Contacts, LEFT_RAW, lambda m: self.left_raw.append(m), rel)
        self.create_subscription(Contacts, RIGHT_RAW, lambda m: self.right_raw.append(m), rel)
        if enable_body:
            self.create_subscription(Contacts, BODY_RAW, lambda m: self.body_raw.append(m), rel)
        self.create_subscription(Clock, "/clock", lambda m: self.clock.append(m), be)
        if enable_regression:
            self.create_subscription(JointState, "/joint_states",
                                     lambda m: self.joints.append(m), rel)
            self.create_subscription(PoseStamped, "/body_pose",
                                     lambda m: self.body.append(m), rel)
            self.create_subscription(Imu, "/imu/data", lambda m: self.imu.append(m), rel)

    def clear(self):
        self.left.clear()
        self.right.clear()
        self.left_raw.clear()
        self.right_raw.clear()
        self.body_raw.clear()
        self.clock.clear()
        self.joints.clear()
        self.body.clear()
        self.imu.clear()


def sim_ts(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def last_data(msgs):
    return msgs[-1].data if msgs else None


def collision_names(msg):
    out = []
    for c in msg.contacts:
        out.append((c.collision1.name, c.collision2.name))
    return out


def has_pair(msg, foot_collision, ground="ground_plane"):
    for n1, n2 in collision_names(msg):
        for a, b in ((n1, n2), (n2, n1)):
            if a.endswith("::%s" % foot_collision) and b.startswith(ground + "::"):
                return True
    return False


# --------------------------------------------------------------------------
# Part 1: the real launch
# --------------------------------------------------------------------------
def run_launch(args, env):
    out = {"part": "launch", "checks": {}, "stats": {}, "pause": {}}
    failures = []
    blocked = []

    def record(name, ok, detail=""):
        out["checks"][name] = {"ok": bool(ok), "detail": detail}
        if not ok:
            failures.append("%s: %s" % (name, detail))

    partition = "t3e_launch_%d" % int(time.time())
    env = dict(env)
    env["GZ_PARTITION"] = partition
    # remove any stale processes from a previous run before measuring publishers
    cleanup_processes(env)
    log_path = os.path.join(args.out, "launch_30s.log")
    log_f = open(log_path, "w")
    cmd = ["ros2", "launch", "nao_gz_sim", "nao_sim.launch.py",
           "gui:=false", "use_rsp:=false"]
    proc = subprocess.Popen(cmd, stdout=log_f, stderr=subprocess.STDOUT,
                            env=env, start_new_session=True)
    out["partition"] = partition
    out["log"] = log_path
    out["launch_cmd"] = " ".join(cmd)

    try:
        node = Collector(name="task3e_validator_launch", enable_body=False,
                         enable_regression=True)
        got = spin_for(node, 60.0,
                       stop_when=lambda: len(node.left) > 0 and len(node.right) > 0)
        if not got:
            blocked.append("no public Bool message within timeout")
            out["blocked"] = "no /left_foot/contact or /right_foot/contact message"
            record("bool_receives", False, "no message")
            return out, failures, blocked

        # 30 s stability window
        t0 = time.time()
        spin_for(node, args.duration)
        wall = time.time() - t0

        # topic metadata
        _, ltype = run(["ros2", "topic", "type", LEFT_TOPIC], env=env)
        _, rtype = run(["ros2", "topic", "type", RIGHT_TOPIC], env=env)
        _, linfo = run(["ros2", "topic", "info", "-v", LEFT_TOPIC], env=env)
        _, rinfo = run(["ros2", "topic", "info", "-v", RIGHT_TOPIC], env=env)
        _, lraw_info = run(["ros2", "topic", "info", "-v", LEFT_RAW], env=env)
        _, rraw_info = run(["ros2", "topic", "info", "-v", RIGHT_RAW], env=env)
        _, lraw_type = run(["ros2", "topic", "type", LEFT_RAW], env=env)
        _, topics = run(["ros2", "topic", "list"], env=env)
        out["left_type"] = ltype.strip()
        out["right_type"] = rtype.strip()
        out["left_info"] = linfo
        out["right_info"] = rinfo
        out["left_raw_info"] = lraw_info
        out["right_raw_info"] = rraw_info
        out["left_raw_type"] = lraw_type.strip()
        out["topic_list"] = sorted(topics.split())

        record("left_type_bool", ltype.strip() == "std_msgs/msg/Bool", ltype.strip())
        record("right_type_bool", rtype.strip() == "std_msgs/msg/Bool", rtype.strip())
        lpub = re.findall(r"Publisher count:\s*(\d+)", linfo)
        rpub = re.findall(r"Publisher count:\s*(\d+)", rinfo)
        record("left_single_publisher", lpub == ["1"], str(lpub))
        record("right_single_publisher", rpub == ["1"], str(rpub))
        record("left_publisher_converter", "contact_converter" in linfo, "node")
        record("right_publisher_converter", "contact_converter" in rinfo, "node")
        record("left_qos_reliable", "Reliability: RELIABLE" in linfo, "RELIABLE")
        record("right_qos_reliable", "Reliability: RELIABLE" in rinfo, "RELIABLE")
        record("left_qos_volatile", "Durability: VOLATILE" in linfo, "VOLATILE")
        record("right_qos_volatile", "Durability: VOLATILE" in rinfo, "VOLATILE")
        record("left_qos_depth_unknown_noted",
               "History (Depth): UNKNOWN" in linfo or "History (Depth): 1" in linfo,
               "source KeepLast(1); CLI may report UNKNOWN")
        # raw topics are internal: exactly one publisher, owned by ros_gz_bridge
        lrpub = re.findall(r"Publisher count:\s*(\d+)", lraw_info)
        rrpub = re.findall(r"Publisher count:\s*(\d+)", rraw_info)
        record("left_raw_single_publisher", lrpub == ["1"], str(lrpub))
        record("right_raw_single_publisher", rrpub == ["1"], str(rrpub))
        record("left_raw_publisher_bridge", "ros_gz_bridge" in lraw_info
               or "parameter_bridge" in lraw_info, "bridge")
        record("right_raw_publisher_bridge", "ros_gz_bridge" in rraw_info
               or "parameter_bridge" in rraw_info, "bridge")
        record("left_raw_type_contacts",
               lraw_type.strip() == "ros_gz_interfaces/msg/Contacts", lraw_type.strip())
        forbidden = [t for t in out["topic_list"] if FORBIDDEN_TOPIC_RE.search(t)]
        record("no_forbidden_topics", not forbidden, str(forbidden))

        msgs_l = list(node.left)
        msgs_r = list(node.right)
        record("left_messages", len(msgs_l) > 100, "%d" % len(msgs_l))
        record("right_messages", len(msgs_r) > 100, "%d" % len(msgs_r))
        record("left_values_bool", all(isinstance(m.data, bool) for m in msgs_l), "bool")
        record("right_values_bool", all(isinstance(m.data, bool) for m in msgs_r), "bool")
        record("bool_has_no_header", not hasattr(msgs_l[0], "header"), "std_msgs/Bool")

        # raw scoped names / stamps (from the internal raw topics)
        rawl = list(node.left_raw)
        rawr = list(node.right_raw)
        raw_ok_l = all(not has_pair(m, "r_ankle_collision") for m in rawl) if rawl else True
        raw_ok_r = all(not has_pair(m, "l_ankle_collision") for m in rawr) if rawr else True
        record("raw_left_no_right_foot", raw_ok_l, "no r_ankle pair in left raw")
        record("raw_right_no_left_foot", raw_ok_r, "no l_ankle pair in right raw")
        raw_stamps = [sim_ts(m.header.stamp) for m in (rawl + rawr)]
        record("raw_stamp_nonzero", all(s > 0 for s in raw_stamps) if raw_stamps else True,
               "%d raw msgs" % len(raw_stamps))
        raw_l_stamps = [sim_ts(m.header.stamp) for m in rawl]
        raw_r_stamps = [sim_ts(m.header.stamp) for m in rawr]

        def nondecreasing(s):
            return all(s[i] <= s[i + 1] for i in range(len(s) - 1))

        record("raw_left_stamp_nondecreasing", nondecreasing(raw_l_stamps), "per-topic")
        record("raw_right_stamp_nondecreasing", nondecreasing(raw_r_stamps), "per-topic")

        # rate
        def rate(msgs):
            if len(msgs) < 2:
                return None
            # Bool has no stamp: use wall arrival proxy only for a coarse rate
            return None
        # publish rate via a short hz measurement on the running system
        rc, hz_out = run(["bash", "-lc",
                          "timeout 5 ros2 topic hz %s 2>/dev/null | grep 'average rate' | tail -1"
                          % LEFT_TOPIC], env=env, timeout=12)
        out["left_hz"] = hz_out.strip()
        m = re.search(r"average rate:\s*([0-9.]+)", hz_out)
        record("left_publish_rate", bool(m) and 60.0 <= float(m.group(1)) <= 140.0,
               hz_out.strip())

        # pause / resume: no wall timer may fabricate samples
        pause_cmd = ["gz", "service", "-s", "/world/nao_world/control",
                     "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
                     "--timeout", "3000", "--req", "pause: true"]
        resume_cmd = list(pause_cmd)
        resume_cmd[-1] = "pause: false"
        rc_p, _ = run(pause_cmd, env=env, timeout=10)
        spin_for(node, 0.5)
        n0l, n0r = len(node.left), len(node.right)
        spin_for(node, 2.0)
        paused_l = len(node.left) - n0l
        paused_r = len(node.right) - n0r
        rc_r, _ = run(resume_cmd, env=env, timeout=10)
        spin_for(node, 1.5)
        resumed_l = len(node.left) - n0l - paused_l
        out["pause"] = {"pause_rc": rc_p, "resume_rc": rc_r,
                        "new_left_during_pause": paused_l, "new_right_during_pause": paused_r,
                        "new_left_after_resume": resumed_l,
                        "pause_ok": (rc_p == 0 and rc_r == 0 and paused_l == 0
                                     and paused_r == 0 and resumed_l > 0)}
        record("pause_resume_no_wall_timer", out["pause"]["pause_ok"], json.dumps(out["pause"]))

        # regressions
        record("reg_joint_states", len(node.joints) > 0
               and all(len(m.effort) == 0 for m in node.joints),
               "%d msgs" % len(node.joints))
        record("reg_body_pose", len(node.body) > 0
               and all(m.header.frame_id == "world" for m in node.body),
               "%d msgs" % len(node.body))
        record("reg_imu", len(node.imu) > 0
               and all(m.header.frame_id == "ImuTorsoAccelerometer_frame" for m in node.imu),
               "%d msgs" % len(node.imu))
        record("reg_clock", len(node.clock) > 0, "%d msgs" % len(node.clock))

        out["stats"] = {"left_count": len(node.left), "right_count": len(node.right),
                        "wall_s": wall, "raw_left_count": len(rawl),
                        "raw_right_count": len(rawr),
                        "left_last": last_data(node.left), "right_last": last_data(node.right),
                        "left_true": sum(1 for m in node.left if m.data),
                        "right_true": sum(1 for m in node.right if m.data)}
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
        cleanup_processes(env)

    text = re.sub(r"\x1b\[[0-9;]*m", "", open(log_path, errors="ignore").read())
    out["violations"] = re.findall(r"CONTRACT VIOLATION[^\n]*", text)
    out["err_count"] = text.count("[Err]")
    record("no_contract_violation", not out["violations"], str(out["violations"][:3]))
    record("no_err_in_log", out["err_count"] == 0, "%d [Err]" % out["err_count"])
    _, ps_out = run(["bash", "-lc",
                     "ps -eo args --no-headers | grep -E 'contact_converter|"
                     "joint_state_converter|body_pose_converter|imu_converter|nao_world.sdf' "
                     "| grep -v grep"], env=env)
    record("no_process_leak", not ps_out.strip(), ps_out.strip()[:200])
    return out, failures, blocked


# --------------------------------------------------------------------------
# Part 2: controlled fixtures
# --------------------------------------------------------------------------
class FixturePipeline:
    def __init__(self, args, env, name, body=False):
        self.args = args
        self.env = dict(env)
        self.env["GZ_PARTITION"] = "t3e_%s_%d" % (name, int(time.time() * 1000) % 1000000)
        ws_share = os.path.join(REPO_ROOT, "install", "nao_meshes", "share")
        self.env["GZ_SIM_RESOURCE_PATH"] = ws_share
        self.name = name
        self.body = body
        self.procs = []
        self.gzsim = None
        self.logdir = args.out
        self.log = os.path.join(args.out, "fixture_%s.log" % name)
        self.gz_log = os.path.join(args.out, "fixture_%s_gz.log" % name)

    def _popen(self, cmd, logfile, append=False):
        f = open(logfile, "a" if append else "w")
        p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT,
                             env=self.env, start_new_session=True)
        self.procs.append((p, f))
        return p

    def start(self):
        cleanup_processes(self.env)
        pkg = os.path.join(REPO_ROOT, "install", "nao_gz_sim", "share", "nao_gz_sim")
        world = os.path.join(pkg, "worlds", "nao_world.sdf")
        clock_cfg = os.path.join(pkg, "config", "clock_bridge.yaml")
        self.gzsim = self._popen(["gz", "sim", "-v", "2", "-s", "-r", world], self.gz_log)
        self._popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge", "--ros-args",
                     "-p", "config_file:=" + clock_cfg, "-p", "use_sim_time:=true"],
                    self.log)
        topics = [LEFT_RAW + "@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts",
                  RIGHT_RAW + "@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"]
        if self.body:
            topics.append(BODY_RAW + "@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts")
        self._popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge"] + topics
                    + ["--ros-args", "-p", "use_sim_time:=true"], self.log)
        self._popen(["ros2", "run", "nao_gz_sim", "contact_converter", "--ros-args",
                     "-p", "use_sim_time:=true"], self.log, append=True)
        return self

    def spawn(self, fixture_file, z, roll=0.0, pitch=0.0):
        rc, o = run(["ros2", "run", "ros_gz_sim", "create", "-world", "nao_world",
                     "-file", fixture_file, "-name", "nao", "-x", "0", "-y", "0",
                     "-z", str(z), "-R", str(roll), "-P", str(pitch), "-Y", "0"],
                    env=self.env, timeout=30)
        return rc, o

    def ground(self, z):
        return run(["gz", "service", "-s", "/world/nao_world/set_pose",
                    "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
                    "--timeout", "3000", "--req",
                    'name: "ground_plane" position: {x: 0, y: 0, z: %s} '
                    'orientation: {x: 0, y: 0, z: 0, w: 1}' % z], env=self.env, timeout=10)

    def stop(self):
        for p, f in self.procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
        time.sleep(1.5)
        for p, f in self.procs:
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
            f.close()
        cleanup_processes(self.env)


def cleanup_processes(env):
    run(["bash", "-lc",
         "ps -eo pid,args --no-headers | grep -E 'nao_world.sdf|contact_converter|"
         "ros_gz_bridge' | grep -v grep | awk '{print $1}' | xargs -r kill -9"],
        env=env, timeout=10)


def run_fixture_scenarios(args, env):
    out = {"part": "fixtures", "scenarios": {}, "failures": [], "blocked": []}
    fixtures_dir = os.path.join(args.out, "fixtures")
    os.makedirs(fixtures_dir, exist_ok=True)
    locked = make_fixture(CANONICAL_SDF, os.path.join(fixtures_dir, "locked.sdf"),
                          fixed=True, damp=True, freeze=True, diag=False)
    free = make_fixture(CANONICAL_SDF, os.path.join(fixtures_dir, "free_diag.sdf"),
                        fixed=False, damp=False, diag=True)

    def scenario(name, body=False):
        sc = {"name": name, "checks": {}, "blocked": None, "samples": []}
        out["scenarios"][name] = sc
        return sc

    def record(sc, name, ok, detail=""):
        sc["checks"][name] = {"ok": bool(ok), "detail": detail}
        if not ok:
            out["failures"].append("%s/%s: %s" % (sc["name"], name, detail))

    node = Collector(name="task3e_validator_fixtures", enable_body=True,
                     enable_regression=False)

    # --- double foot -------------------------------------------------------
    sc = scenario("double_foot")
    pipe = FixturePipeline(args, env, "double", body=False).start()
    try:
        spin_for(node, 6.0)
        node.clear()
        rc, o = pipe.spawn(locked, DOUBLE_Z, roll=0.0)
        if rc != 0:
            sc["blocked"] = "spawn failed: %s" % o
        else:
            spin_for(node, 30.0)
            lvals = [m.data for m in node.left]
            rvals = [m.data for m in node.right]
            sc["samples"] = {"left": _summary(node.left), "right": _summary(node.right),
                             "raw_left": _raw_summary(node.left_raw),
                             "raw_right": _raw_summary(node.right_raw)}
            record(sc, "left_true", lvals and lvals[-1] is True, str(lvals[:5]))
            record(sc, "right_true", rvals and rvals[-1] is True, str(rvals[:5]))
            record(sc, "left_raw_pair", any(has_pair(m, "l_ankle_collision")
                                            for m in node.left_raw), "l_ankle<->ground")
            record(sc, "right_raw_pair", any(has_pair(m, "r_ankle_collision")
                                             for m in node.right_raw), "r_ankle<->ground")
            record(sc, "left_raw_only_left",
                   all(not has_pair(m, "r_ankle_collision") for m in node.left_raw), "no right")
            record(sc, "right_raw_only_right",
                   all(not has_pair(m, "l_ankle_collision") for m in node.right_raw), "no left")
            sc["stability"] = _stability(node, 30.0)
            maxc = max(sc["stability"]["max_contacts_left"],
                       sc["stability"]["max_contacts_right"])
            record(sc, "no_contact_explosion", maxc < 50, "max contacts per raw msg = %d" % maxc)
            record(sc, "raw_stamps_nonzero_and_increasing",
                   sc["samples"]["raw_left"]["nonzero"] and sc["samples"]["raw_right"]["nonzero"]
                   and sc["samples"]["raw_left"]["increasing"]
                   and sc["samples"]["raw_right"]["increasing"],
                   "sim-time raw stamps")
    finally:
        pipe.stop()

    # --- single foot left / right -----------------------------------------
    for side, roll, expect_l, expect_r in (("left", -SINGLE_ROLL, True, False),
                                           ("right", SINGLE_ROLL, False, True)):
        sc = scenario("single_%s" % side)
        pipe = FixturePipeline(args, env, "single_%s" % side, body=False).start()
        try:
            spin_for(node, 6.0)
            node.clear()
            rc, o = pipe.spawn(locked, SINGLE_Z, roll=roll)
            if rc != 0:
                sc["blocked"] = "spawn failed: %s" % o
            else:
                spin_for(node, 4.0)
                both_seen = (len(node.left) > 0 and len(node.right) > 0
                             and last_data(node.left) is True and last_data(node.right) is True)
                record(sc, "both_contact_seen", bool(both_seen),
                       "L=%s R=%s" % (last_data(node.left), last_data(node.right)))
                pipe.ground(GROUND_DROP)
                spin_for(node, 26.0)
                lv, rv = last_data(node.left), last_data(node.right)
                sc["samples"] = {"left": _summary(node.left), "right": _summary(node.right),
                                 "raw_left": _raw_summary(node.left_raw),
                                 "raw_right": _raw_summary(node.right_raw)}
                record(sc, "left_value", lv is expect_l, "%s (expect %s)" % (lv, expect_l))
                record(sc, "right_value", rv is expect_r, "%s (expect %s)" % (rv, expect_r))
                sc["stability"] = _stability(node, 30.0)
        finally:
            pipe.stop()

    # --- off-ground --------------------------------------------------------
    sc = scenario("off_ground")
    pipe = FixturePipeline(args, env, "offground", body=False).start()
    try:
        spin_for(node, 6.0)
        node.clear()
        rc, o = pipe.spawn(locked, DOUBLE_Z, roll=0.0)
        if rc != 0:
            sc["blocked"] = "spawn failed: %s" % o
        else:
            spin_for(node, 4.0)
            record(sc, "initial_contact", last_data(node.left) is True
                   and last_data(node.right) is True,
                   "L=%s R=%s" % (last_data(node.left), last_data(node.right)))
            pipe.ground(OFF_GROUND_Z)
            spin_for(node, 20.0)
            lv, rv = last_data(node.left), last_data(node.right)
            sc["off_values"] = {"left": lv, "right": rv}
            record(sc, "both_false_off_ground", lv is False and rv is False, "L=%s R=%s" % (lv, rv))
            pipe.ground(0.0)
            spin_for(node, 6.0)
            lv2, rv2 = last_data(node.left), last_data(node.right)
            sc["restored_values"] = {"left": lv2, "right": rv2}
            record(sc, "both_true_restored", lv2 is True and rv2 is True, "L=%s R=%s" % (lv2, rv2))
    finally:
        pipe.stop()

    # --- non-foot contact --------------------------------------------------
    sc = scenario("non_foot")
    pipe = FixturePipeline(args, env, "nonfoot", body=True).start()
    try:
        spin_for(node, 6.0)
        node.clear()
        rc, o = pipe.spawn(free, 0.35, roll=0.0)
        if rc != 0:
            sc["blocked"] = "spawn failed: %s" % o
        else:
            # let the un-controlled model land on its feet and collapse
            spin_for(node, 12.0)
            # inspect only the settled window (after the landing transient)
            node.clear()
            spin_for(node, 30.0)
            body_names = set()
            for m in node.body_raw:
                for n1, n2 in collision_names(m):
                    body_names.add(n1)
                    body_names.add(n2)
            body_only = [n for n in body_names
                         if n.startswith("nao::") and "ankle" not in n.lower()]
            lv, rv = last_data(node.left), last_data(node.right)
            sc["body_collisions"] = sorted(body_only)
            sc["foot_values"] = {"left": lv, "right": rv}
            sc["settled_window"] = {"raw_left": len(node.left_raw),
                                    "raw_right": len(node.right_raw),
                                    "body": len(node.body_raw)}
            record(sc, "non_foot_contact_observed", bool(body_only),
                   "body collisions: %s" % sorted(body_only)[:5])
            record(sc, "feet_not_true", lv is not True and rv is not True, "L=%s R=%s" % (lv, rv))
            record(sc, "feet_not_both_true", not (lv is True and rv is True), "L=%s R=%s" % (lv, rv))
            record(sc, "foot_raw_silent",
                   not any(has_pair(m, "l_ankle_collision") for m in node.left_raw)
                   and not any(has_pair(m, "r_ankle_collision") for m in node.right_raw),
                   "no foot-ground pair in the settled window")
    finally:
        pipe.stop()

    # --- no valid sample => must NOT publish false ------------------------
    sc = scenario("no_sample_unknown")
    pipe = FixturePipeline(args, env, "nosample", body=False).start()
    try:
        spin_for(node, 6.0)
        node.clear()
        rc, o = pipe.spawn(locked, 0.6, roll=0.0)  # airborne, no contact at all
        if rc != 0:
            sc["blocked"] = "spawn failed: %s" % o
        else:
            spin_for(node, 30.0)
            sc["published"] = {"left": len(node.left), "right": len(node.right),
                               "raw_left": len(node.left_raw), "raw_right": len(node.right_raw)}
            record(sc, "no_false_before_first_sample",
                   len(node.left) == 0 and len(node.right) == 0,
                   "published L=%d R=%d (expected 0: unknown, not false)"
                   % (len(node.left), len(node.right)))
            record(sc, "no_raw_when_airborne",
                   len(node.left_raw) == 0 and len(node.right_raw) == 0,
                   "gz contact sensor is contact-triggered")
    finally:
        pipe.stop()

    return out


def _summary(msgs):
    vals = [m.data for m in msgs]
    return {"count": len(vals), "true": sum(1 for v in vals if v),
            "false": sum(1 for v in vals if v is False), "last": vals[-1] if vals else None}


def _raw_summary(msgs):
    stamps = [sim_ts(m.header.stamp) for m in msgs]
    return {"count": len(stamps), "nonzero": all(s > 0 for s in stamps),
            "increasing": all(stamps[i] <= stamps[i + 1] for i in range(len(stamps) - 1)),
            "first": stamps[0] if stamps else None, "last": stamps[-1] if stamps else None}


def _stability(node, wall_s):
    lv = [m.data for m in node.left]
    rv = [m.data for m in node.right]
    transitions_l = sum(1 for i in range(1, len(lv)) if lv[i] != lv[i - 1])
    transitions_r = sum(1 for i in range(1, len(rv)) if rv[i] != rv[i - 1])
    return {"left_count": len(lv), "right_count": len(rv),
            "left_true": sum(1 for v in lv if v), "right_true": sum(1 for v in rv if v),
            "left_transitions": transitions_l, "right_transitions": transitions_r,
            "left_hz_wall": len(lv) / wall_s if wall_s else None,
            "raw_left": len(node.left_raw), "raw_right": len(node.right_raw),
            "max_contacts_left": max((len(m.contacts) for m in node.left_raw), default=0),
            "max_contacts_right": max((len(m.contacts) for m in node.right_raw), default=0)}


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--out", default="docs/task3e_evidence")
    ap.add_argument("--skip-launch", action="store_true")
    ap.add_argument("--skip-fixtures", action="store_true")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    env = dict(os.environ)

    report = {"duration": args.duration}
    failures = []
    blocked = []

    rclpy.init()
    try:
        if not args.skip_launch:
            res, f, b = run_launch(args, env)
            report["launch"] = res
            failures += f
            blocked += b
            with open(os.path.join(args.out, "launch_30s.json"), "w") as fp:
                json.dump(res, fp, indent=2)
            print("=== launch run: %d checks, %d failures, %d blocked ==="
                  % (len(res["checks"]), len(f), len(b)))

        if not args.skip_fixtures:
            res = run_fixture_scenarios(args, env)
            report["fixtures"] = res
            failures += res["failures"]
            blocked += res["blocked"]
            with open(os.path.join(args.out, "fixtures.json"), "w") as fp:
                json.dump(res, fp, indent=2)
            for name, sc in res["scenarios"].items():
                nchk = len(sc["checks"])
                nfail = sum(1 for c in sc["checks"].values() if not c["ok"])
                print("=== fixture %-14s: %d checks, %d fail%s ==="
                      % (name, nchk, nfail, " BLOCKED" if sc.get("blocked") else ""))
                for cn, c in sc["checks"].items():
                    if not c["ok"]:
                        print("    [FAIL] %s -- %s" % (cn, c["detail"]))
    finally:
        rclpy.shutdown()

    report["failures"] = failures
    report["blocked"] = blocked
    if failures:
        report["verdict"] = "FAIL"
    elif blocked:
        report["verdict"] = "BLOCKED"
    else:
        report["verdict"] = "PASS"
    with open(os.path.join(args.out, "report.json"), "w") as fp:
        json.dump(report, fp, indent=2)

    print("\nTASK 3E VERDICT: %s" % report["verdict"])
    if failures:
        print("failures: %s" % failures)
    if blocked:
        print("blocked: %s" % blocked)
    return 0 if report["verdict"] == "PASS" else (2 if report["verdict"] == "BLOCKED" else 1)


if __name__ == "__main__":
    sys.exit(main())
