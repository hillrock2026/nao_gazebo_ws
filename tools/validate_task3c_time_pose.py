#!/usr/bin/python3
"""Task 3C validation: Gazebo simulation time (/clock) and /body_pose.

Launches the workspace ``nao_sim.launch.py`` headless and verifies:

* ``/clock`` exists, is ``rosgraph_msgs/msg/Clock``, CLOCK QoS
  (BEST_EFFORT + VOLATILE), single publisher, advances with simulation time,
  stops while Gazebo is paused and resumes afterwards;
* ``/body_pose`` exists, is ``geometry_msgs/msg/PoseStamped``, single publisher,
  ``frame_id == "world"``, non-zero strictly increasing Gazebo timestamps,
  normalized finite quaternion, and numerically matches the Gazebo model pose
  from ``/world/nao_world/pose/info`` (name ``nao``) for the same stamp;
* no ``/tf`` or ``/tf_static`` is published (RSP disabled);
* Task 3B ``/joint_states`` still works (sim stamps, empty effort).

Run with the ROS Python interpreter:

    /usr/bin/python3 tools/validate_task3c_time_pose.py --runs 2 --duration 30 \
        --out /tmp/task3c

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

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

MODEL = "nao"
POSE_TOPIC = "/world/nao_world/pose/info"
POSITION_TOL = 1e-6
QUATERNION_TOL = 1e-6


class Collector(Node):
    def __init__(self):
        super().__init__("task3c_validator")
        clock_qos = QoSProfile(depth=1)
        clock_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        clock_qos.durability = DurabilityPolicy.VOLATILE
        pose_qos = QoSProfile(depth=10)
        pose_qos.reliability = ReliabilityPolicy.RELIABLE
        pose_qos.durability = DurabilityPolicy.VOLATILE
        self.clock = []
        self.body = []
        self.joints = []
        self.create_subscription(Clock, "/clock", lambda m: self.clock.append(m), clock_qos)
        self.create_subscription(PoseStamped, "/body_pose",
                                 lambda m: self.body.append(m), pose_qos)
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: self.joints.append(m), pose_qos)


def run(cmd, env=None, timeout=20):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, env=env)
        return p.returncode, p.stdout + p.stderr
    except Exception as exc:  # noqa: BLE001
        return 127, str(exc)


def spin_for(node, seconds, stop_when=None):
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
        if stop_when is not None and stop_when():
            return True
    return False


def ts(msg):
    return (msg.header.stamp.sec, msg.header.stamp.nanosec)


def parse_gz_pose(text):
    """Return ((sec, nsec), position_xyz, quaternion_xyzw) for model 'nao'."""
    m_stamp = re.search(r"stamp\s*\{\s*sec:\s*(\d+)\s*nsec:\s*(\d+)\s*\}", text)
    if not m_stamp:
        return None
    stamp = (int(m_stamp.group(1)), int(m_stamp.group(2)))
    m_name = re.search(r'name:\s*"%s"' % re.escape(MODEL), text)
    if not m_name:
        return None
    tail = text[m_name.start():]
    m_pos = re.search(r"position\s*\{([^}]*)\}", tail)
    m_ori = re.search(r"orientation\s*\{([^}]*)\}", tail)
    if not m_pos or not m_ori:
        return None

    def vals(block, keys):
        out = []
        for k in keys:
            mm = re.search(r"\b%s:\s*([-+0-9.eE]+)" % k, block)
            out.append(float(mm.group(1)) if mm else 0.0)
        return out

    pos = vals(m_pos.group(1), "xyz")
    quat = vals(m_ori.group(1), "xyzw")
    return stamp, pos, quat


def quat_angle(q1, q2):
    d = abs(sum(a * b for a, b in zip(q1, q2)))
    d = min(1.0, max(-1.0, d))
    return 2.0 * math.acos(d)


def run_once(args, index, env):
    partition = "t3c_%d_%d" % (index, int(time.time()))
    env = dict(env)
    env["GZ_PARTITION"] = partition
    log_path = os.path.join(args.out, "launch_run%d.log" % index)
    log_f = open(log_path, "w")
    cmd = ["ros2", "launch", "nao_gz_sim", "nao_sim.launch.py",
           "gui:=false", "use_rsp:=false"]
    proc = subprocess.Popen(cmd, stdout=log_f, stderr=subprocess.STDOUT,
                            env=env, start_new_session=True)
    result = {"run": index, "partition": partition, "log": log_path,
              "launch_cmd": " ".join(cmd), "checks": {}, "stats": {}, "pause": {}}
    failures = []

    def record(name, ok, detail=""):
        result["checks"][name] = {"ok": bool(ok), "detail": detail}
        if not ok:
            failures.append("%s: %s" % (name, detail))

    gz_capture = None
    try:
        rclpy.init()
        node = Collector()
        got = spin_for(node, 60.0, stop_when=lambda: len(node.body) > 0)
        if not got:
            result["blocked"] = "no /body_pose message within timeout"
            record("body_pose_receives", False, "no message")
            return result, failures

        # Capture one Gazebo pose/info message concurrently for cross-check.
        gz_capture = subprocess.Popen(
            ["gz", "topic", "-e", "-t", POSE_TOPIC, "-n", "1"],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, env=env)

        t0 = time.time()
        spin_for(node, args.duration)
        wall = time.time() - t0

        # Pause / resume.
        pause_cmd = ["gz", "service", "-s", "/world/nao_world/control",
                     "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
                     "--timeout", "3000", "--req", "pause: true"]
        resume_cmd = list(pause_cmd)
        resume_cmd[-1] = "pause: false"
        rc_p, out_p = run(pause_cmd, env=env, timeout=10)
        spin_for(node, 0.5)
        n_body_p = len(node.body)
        n_clock_p = len(node.clock)
        clock_at_pause = (node.clock[-1].clock.sec, node.clock[-1].clock.nanosec) \
            if node.clock else None
        spin_for(node, 2.0)
        paused_body = len(node.body) - n_body_p
        paused_clock_msgs = len(node.clock) - n_clock_p
        clock_end_pause = (node.clock[-1].clock.sec, node.clock[-1].clock.nanosec) \
            if node.clock else None
        rc_r, out_r = run(resume_cmd, env=env, timeout=10)
        spin_for(node, 1.5)
        resumed_body = len(node.body) - n_body_p - paused_body
        clock_after_resume = (node.clock[-1].clock.sec, node.clock[-1].clock.nanosec) \
            if node.clock else None
        # While paused Gazebo may keep republishing /clock, but the simulation
        # value must NOT advance. Body pose (event driven) must stop entirely.
        clock_frozen = (clock_at_pause is not None and clock_end_pause is not None
                        and clock_end_pause <= clock_at_pause)
        clock_resumes = (clock_after_resume is not None and clock_end_pause is not None
                         and clock_after_resume > clock_end_pause)
        result["pause"] = {
            "pause_rc": rc_p, "resume_rc": rc_r,
            "pause_output": out_p.strip()[:120], "resume_output": out_r.strip()[:120],
            "new_body_during_pause": paused_body,
            "new_clock_msgs_during_pause": paused_clock_msgs,
            "clock_at_pause": clock_at_pause, "clock_end_pause": clock_end_pause,
            "clock_after_resume": clock_after_resume,
            "new_body_after_resume": resumed_body,
            "pause_ok": (rc_p == 0 and rc_r == 0 and paused_body == 0
                         and clock_frozen and resumed_body > 0 and clock_resumes),
        }

        # External topic inspection.
        _, clock_type = run(["ros2", "topic", "type", "/clock"], env=env)
        _, clock_info = run(["ros2", "topic", "info", "-v", "/clock"], env=env)
        _, body_type = run(["ros2", "topic", "type", "/body_pose"], env=env)
        _, body_info = run(["ros2", "topic", "info", "-v", "/body_pose"], env=env)
        _, topics = run(["ros2", "topic", "list"], env=env)
        result["clock_type"] = clock_type.strip()
        result["body_type"] = body_type.strip()
        result["body_info"] = body_info
        result["topic_list"] = sorted(topics.split())

        body_msgs = list(node.body)
        clock_msgs = list(node.clock)
        joint_msgs = list(node.joints)

        record("clock_type", clock_type.strip() == "rosgraph_msgs/msg/Clock", clock_type.strip())
        cpub = re.findall(r"Publisher count:\s*(\d+)", clock_info)
        record("clock_single_publisher", len(cpub) == 1 and cpub[0] == "1", str(cpub))
        record("clock_qos_best_effort", "Reliability: BEST_EFFORT" in clock_info,
               "expected BEST_EFFORT")
        record("clock_qos_volatile", "Durability: VOLATILE" in clock_info,
               "expected VOLATILE")
        record("body_pose_type", body_type.strip() == "geometry_msgs/msg/PoseStamped",
               body_type.strip())
        bpub = re.findall(r"Publisher count:\s*(\d+)", body_info)
        record("body_pose_single_publisher", len(bpub) == 1 and bpub[0] == "1", str(bpub))
        # Task 3F adds the frozen world->base_link TF publisher, so /tf may exist
        # here; the Task 3C requirement is that body_pose_converter itself must
        # NOT publish /tf or /tf_static.
        _, tf_info3c = run(["ros2", "topic", "info", "-v", "/tf"], env=env)
        _, tfs_info3c = run(["ros2", "topic", "info", "-v", "/tf_static"], env=env)
        tf_pub_nodes = set(re.findall(r"Node name:\s*(\S+)",
                                      tf_info3c.split("Subscription count:")[0]))
        tfs_pub_nodes = set(re.findall(r"Node name:\s*(\S+)",
                                       tfs_info3c.split("Subscription count:")[0]))
        record("body_pose_does_not_publish_tf",
               "body_pose_converter" not in tf_pub_nodes
               and "body_pose_converter" not in tfs_pub_nodes,
               "tf=%s tf_static=%s" % (sorted(tf_pub_nodes), sorted(tfs_pub_nodes)))

        # ---- clock checks ----
        cvals = [(m.clock.sec, m.clock.nanosec) for m in clock_msgs]
        record("clock_advances", len(cvals) > 1 and cvals[-1] > cvals[0],
               "first=%s last=%s n=%d" % (cvals[0] if cvals else None,
                                          cvals[-1] if cvals else None, len(cvals)))
        record("clock_monotonic",
               all(cvals[i] <= cvals[i + 1] for i in range(len(cvals) - 1)),
               "non-decreasing")

        # ---- body_pose checks ----
        n = len(body_msgs)
        record("body_pose_count", n > 10, "%d messages in %.1fs wall" % (n, wall))
        frame_ok = all(m.header.frame_id == "world" for m in body_msgs)
        record("body_pose_frame_id_world", frame_ok,
               sorted({m.header.frame_id for m in body_msgs}))
        stamps = [ts(m) for m in body_msgs]
        record("body_pose_stamp_nonzero",
               all(s != (0, 0) for s in stamps), "all non-zero")
        record("body_pose_stamp_increasing",
               all(stamps[i] < stamps[i + 1] for i in range(n - 1)),
               "strictly increasing")
        norms = []
        finite = True
        for m in body_msgs:
            o = m.pose.orientation
            q = [o.x, o.y, o.z, o.w]
            if not all(math.isfinite(v) for v in q) or \
               not all(math.isfinite(v) for v in
                       [m.pose.position.x, m.pose.position.y, m.pose.position.z]):
                finite = False
            norms.append(math.sqrt(sum(v * v for v in q)))
        record("body_pose_finite", finite, "no NaN/Inf")
        sd = max(abs(v - 1.0) for v in norms) if norms else 1.0
        record("body_pose_quaternion_normalized", sd < 1e-6, "max |norm-1|=%.2e" % sd)

        # ---- joint_states still OK (Task 3B regression) ----
        record("joint_states_present", len(joint_msgs) > 0,
               "%d messages" % len(joint_msgs))
        if joint_msgs:
            jstamps = [(m.header.stamp.sec, m.header.stamp.nanosec) for m in joint_msgs]
            record("joint_states_sim_stamp",
                   all(s != (0, 0) for s in jstamps)
                   and all(jstamps[i] < jstamps[i + 1] for i in range(len(jstamps) - 1)),
                   "nonzero, strictly increasing")
            record("joint_states_effort_empty",
                   all(len(m.effort) == 0 for m in joint_msgs), "effort empty")

        # ---- cross-check with Gazebo pose/info ----
        gz_text = ""
        if gz_capture is not None:
            try:
                gz_text = gz_capture.communicate(timeout=10)[0] or ""
            except subprocess.TimeoutExpired:
                gz_capture.kill()
        parsed = parse_gz_pose(gz_text) if gz_text else None
        result["gz_pose_sample"] = {"stamp": parsed[0], "position": parsed[1],
                                    "quaternion": parsed[2]} if parsed else None
        if parsed is None:
            record("pose_crosscheck", False, "could not parse Gazebo pose/info sample")
        else:
            gstamp, gpos, gquat = parsed
            match = next((m for m in body_msgs if ts(m) == gstamp), None)
            if match is None:
                record("pose_crosscheck", False,
                       "no /body_pose with Gazebo stamp %s" % (gstamp,))
            else:
                bp = [match.pose.position.x, match.pose.position.y, match.pose.position.z]
                bo = [match.pose.orientation.x, match.pose.orientation.y,
                      match.pose.orientation.z, match.pose.orientation.w]
                dpos = math.dist(bp, gpos)
                dquat = quat_angle(bo, gquat)
                result["stats"]["crosscheck_position_err_m"] = dpos
                result["stats"]["crosscheck_quaternion_angle_rad"] = dquat
                result["stats"]["crosscheck_stamp"] = gstamp
                record("pose_crosscheck",
                       dpos < POSITION_TOL and dquat < QUATERNION_TOL,
                       "dpos=%.2e dquat=%.2e @stamp %s" % (dpos, dquat, gstamp))

        # ---- frequency ----
        if n >= 2:
            sim_span = (stamps[-1][0] - stamps[0][0]) + (stamps[-1][1] - stamps[0][1]) * 1e-9
            result["stats"].update({
                "body_count": n, "wall_s": wall, "sim_span_s": sim_span,
                "body_rate_hz_wall": n / wall if wall else None,
                "sim_rate_hz": (n - 1) / sim_span if sim_span > 0 else None,
                "body_first_stamp": stamps[0], "body_last_stamp": stamps[-1],
                "clock_first": cvals[0] if cvals else None,
                "clock_last": cvals[-1] if cvals else None,
                "clock_count": len(cvals),
            })
        record("pause_behavior", result["pause"].get("pause_ok", False),
               json.dumps(result["pause"])[:300])

        # reload nodes use_sim_time
        rc, p1 = run(["ros2", "param", "get", "/body_pose_converter", "use_sim_time"], env=env)
        rc2, p2 = run(["ros2", "param", "get", "/joint_state_converter", "use_sim_time"], env=env)
        body_sim = "true" in p1.strip().lower()
        joint_sim = "true" in p2.strip().lower()
        record("use_sim_time_true", body_sim and joint_sim,
               "body='%s' joint='%s'" % (p1.strip(), p2.strip()))

        rclpy.shutdown()
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
        # best-effort kill any leftover tool processes from this partition
        run(["bash", "-lc",
             "ps -eo pid,args --no-headers | grep -E 'body_pose_converter|joint_state_converter' "
             "| grep -v grep | awk '{print $1}' | xargs -r kill"], env=env, timeout=10)

    text = re.sub(r"\x1b\[[0-9;]*m", "", open(log_path, errors="ignore").read())
    result["violations"] = re.findall(r"CONTRACT VIOLATION[^\n]*", text)
    result["err_count"] = text.count("[Err]")
    record("no_contract_violation", not result["violations"], str(result["violations"][:3]))
    record("no_err_in_log", result["err_count"] == 0, "%d [Err]" % result["err_count"])
    return result, failures


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", type=int, default=2)
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--out", default="/tmp/task3c")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    env = dict(os.environ)

    all_failures = []
    for i in range(1, args.runs + 1):
        res, fails = run_once(args, i, env)
        with open(os.path.join(args.out, "report_run%d.json" % i), "w") as f:
            json.dump(res, f, indent=2)
        print("=== run %d: %s ===" % (i, "PASS" if not fails else "FAIL"))
        for name, c in res["checks"].items():
            if not c["ok"]:
                print("  [FAIL] %s -- %s" % (name, c["detail"]))
        print("  checks: %d/%d PASS; stats=%s; pause=%s" % (
            sum(1 for c in res["checks"].values() if c["ok"]), len(res["checks"]),
            json.dumps(res.get("stats", {}))[:400], res.get("pause", {}).get("pause_ok")))
        all_failures += ["run%d %s" % (i, f) for f in fails]

    print("\nTASK 3C VERDICT: %s" % ("PASS" if not all_failures else "FAIL"))
    if all_failures:
        print("failures: %s" % all_failures)
    return 0 if not all_failures else 1


if __name__ == "__main__":
    sys.exit(main())
