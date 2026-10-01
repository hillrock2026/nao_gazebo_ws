#!/usr/bin/python3
"""Task 3B validation: the frozen /joint_states interface.

Launches the workspace ``nao_sim.launch.py`` headless, subscribes to
``/joint_states`` with the frozen QoS, collects at least 30 seconds of
messages and asserts the Task 3A joint-state contract:

* type ``sensor_msgs/msg/JointState``;
* exactly one public publisher on ``/joint_states``;
* 24 canonical joint names in the frozen SDF document order;
* position/velocity length == name length; ``effort`` empty; ``frame_id == ""``;
* non-zero, strictly increasing Gazebo simulation timestamps; no NaN/Inf;
* no controller / ros2_control / joint command / LoLa / hip-coupling topics.

Uses rclpy, so run it with the ROS Python interpreter:

    /usr/bin/python3 tools/validate_task3b_joint_states.py --duration 30 --out /tmp/t3b.json

Exit code 0 = all checks pass, 1 = a check failed, 2 = BLOCKED (could not run).
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

from sensor_msgs.msg import JointState

EXPECTED_NAMES = [
    "HeadYaw", "HeadPitch",
    "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
    "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw",
    "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
    "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw",
]
FORBIDDEN_TOPIC_RE = re.compile(
    r"joint_command|joint_positions|joint_velocities|ros2_control|controller_manager|"
    r"lola|Lola|LoLa|hip_sync|HIPYAWPITCH",
)
FORBIDDEN_LOG_RE = re.compile(
    r"controller_manager|ros2_control|CONTRACT VIOLATION|HeadYaw command|hip.?sync|LoLa",
    re.IGNORECASE,
)


class Collector(Node):
    def __init__(self):
        super().__init__("task3b_validator")
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self.messages = []
        self.create_subscription(JointState, "/joint_states", self._cb, qos)

    def _cb(self, msg):
        self.messages.append(msg)


def run(cmd, env=None, timeout=15):
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, env=env)
        return p.returncode, p.stdout + p.stderr
    except Exception as exc:  # noqa: BLE001
        return 127, str(exc)


def spin_for(node, seconds, stop_when=None):
    """Spin node for wall seconds, optionally stopping when stop_when() is true."""
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
        if stop_when is not None and stop_when():
            return True
    return False


def stamp_lt(a, b):
    return (a.sec, a.nanosec) < (b.sec, b.nanosec)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--gui", default="false")
    ap.add_argument("--use-rsp", default="false")
    ap.add_argument("--out", default="/tmp/task3b_joint_states.json")
    ap.add_argument("--log", default="/tmp/task3b_launch.log")
    args = ap.parse_args()

    partition = "t3b_val_%d" % int(time.time())
    env = dict(os.environ)
    env["GZ_PARTITION"] = partition

    log_f = open(args.log, "w")
    launch_cmd = ["ros2", "launch", "nao_gz_sim", "nao_sim.launch.py",
                  "gui:=%s" % args.gui, "use_rsp:=%s" % args.use_rsp]
    proc = subprocess.Popen(launch_cmd, stdout=log_f, stderr=subprocess.STDOUT,
                            env=env, start_new_session=True)

    result = {"launch_cmd": " ".join(launch_cmd), "log": args.log,
              "partition": partition, "duration_target_s": args.duration,
              "checks": {}, "stats": {}, "pause": {}}
    failures = []

    def record(name, ok, detail=""):
        result["checks"][name] = {"ok": bool(ok), "detail": detail}
        if not ok:
            failures.append("%s: %s" % (name, detail))

    try:
        rclpy.init()
        node = Collector()

        # Wait for the first message.
        got = spin_for(node, min(60.0, args.duration + 30.0),
                       stop_when=lambda: len(node.messages) > 0)
        if not got:
            result["blocked"] = "no /joint_states message within timeout"
            record("topic_receives_messages", False, "no message received")
            raise SystemExit(2)

        # Collect for the required duration.
        t0 = time.time()
        spin_for(node, args.duration)
        wall_elapsed = time.time() - t0
        msgs = list(node.messages)

        # Pause / resume behavior via the Gazebo world control service.
        pause_cmd = ["gz", "service", "-s", "/world/nao_world/control",
                     "--reqtype", "gz.msgs.WorldControl",
                     "--reptype", "gz.msgs.Boolean", "--timeout", "3000",
                     "--req", "pause: true"]
        resume_cmd = list(pause_cmd)
        resume_cmd[-1] = "pause: false"
        rc_p, out_p = run(pause_cmd, env=env, timeout=10)
        spin_for(node, 0.5)  # drain in-flight messages after pause is applied
        n_start = len(node.messages)
        spin_for(node, 2.0)
        n_end = len(node.messages)
        rc_r, out_r = run(resume_cmd, env=env, timeout=10)
        spin_for(node, 1.5)
        n_resumed = len(node.messages)
        result["pause"] = {
            "pause_cmd": " ".join(pause_cmd), "pause_rc": rc_p,
            "pause_output": out_p.strip()[:300],
            "resume_rc": rc_r, "resume_output": out_r.strip()[:300],
            "msgs_at_pause_start": n_start, "msgs_after_2s_pause": n_end,
            "msgs_after_resume": n_resumed,
            "new_msgs_during_pause": n_end - n_start,
            "pause_ok": (rc_p == 0 and rc_r == 0 and n_end == n_start
                         and n_resumed > n_end),
        }

        # External topic inspection.
        _, type_out = run(["ros2", "topic", "type", "/joint_states"], env=env)
        _, info_out = run(["ros2", "topic", "info", "-v", "/joint_states"], env=env)
        _, list_out = run(["ros2", "topic", "list"], env=env)
        result["topic_type"] = type_out.strip()
        result["topic_info"] = info_out
        result["topic_list"] = sorted(list_out.split())

        # ---- checks -----------------------------------------------------------
        record("topic_type", type_out.strip() == "sensor_msgs/msg/JointState",
               type_out.strip())
        pub_matches = re.findall(r"Publisher count:\s*(\d+)", info_out)
        record("single_publisher", len(pub_matches) == 1 and pub_matches[0] == "1",
               "publisher count: %s" % pub_matches)
        record("qos_reliable", "Reliability: RELIABLE" in info_out,
               "expected RELIABLE in topic info")
        record("qos_volatile", "Durability: VOLATILE" in info_out,
               "expected VOLATILE in topic info")
        forbidden_topics = [t for t in result["topic_list"] if FORBIDDEN_TOPIC_RE.search(t)]
        record("no_forbidden_topics", not forbidden_topics, str(forbidden_topics))

        # ---- message-level checks --------------------------------------------
        n = len(msgs)
        result["stats"]["message_count"] = n
        record("message_count", n >= 10, "collected %d messages in %.1fs wall" % (n, wall_elapsed))

        names_ok = all(list(m.name) == EXPECTED_NAMES for m in msgs)
        record("names_exact_order", names_ok,
               "all messages have the frozen 24-name order" if names_ok else "order/set drift")
        record("name_count_24", all(len(m.name) == 24 for m in msgs),
               "name lengths: %s" % sorted({len(m.name) for m in msgs}))
        pos_len_ok = all(len(m.position) == len(m.name) for m in msgs)
        vel_len_ok = all(len(m.velocity) == len(m.name) for m in msgs)
        record("position_len", pos_len_ok, "position length == name length for all")
        record("velocity_len", vel_len_ok, "velocity length == name length for all")
        effort_empty = all(len(m.effort) == 0 for m in msgs)
        record("effort_empty", effort_empty,
               "effort lengths: %s" % sorted({len(m.effort) for m in msgs}))
        frame_empty = all(m.header.frame_id == "" for m in msgs)
        record("frame_id_empty", frame_empty,
               "frame_ids: %s" % sorted({m.header.frame_id for m in msgs}))

        finite_ok = True
        for m in msgs:
            for v in list(m.position) + list(m.velocity):
                if not math.isfinite(v):
                    finite_ok = False
        record("values_finite", finite_ok, "no NaN/Inf in position/velocity")

        nonzero = all((m.header.stamp.sec != 0 or m.header.stamp.nanosec != 0) for m in msgs)
        record("stamp_nonzero", nonzero, "all stamps non-zero (simulation time)")
        strictly_inc = all(stamp_lt(msgs[i].header.stamp, msgs[i + 1].header.stamp)
                           for i in range(n - 1))
        regressions = sum(1 for i in range(n - 1)
                          if stamp_lt(msgs[i + 1].header.stamp, msgs[i].header.stamp))
        record("stamp_strictly_increasing", strictly_inc and regressions == 0,
               "regressions=%d" % regressions)

        # ---- frequency stats --------------------------------------------------
        if n >= 2:
            sim_periods = []
            for i in range(n - 1):
                a = msgs[i].header.stamp
                b = msgs[i + 1].header.stamp
                sim_periods.append((b.sec - a.sec) + (b.nanosec - a.nanosec) * 1e-9)
            sim_periods = [p for p in sim_periods if p > 0]
            result["stats"]["sim_period_min_s"] = min(sim_periods) if sim_periods else None
            result["stats"]["sim_period_max_s"] = max(sim_periods) if sim_periods else None
            result["stats"]["sim_period_mean_s"] = (
                sum(sim_periods) / len(sim_periods) if sim_periods else None)
            if sim_periods:
                result["stats"]["sim_frequency_hz"] = 1.0 / result["stats"]["sim_period_mean_s"]
            span = (msgs[-1].header.stamp.sec - msgs[0].header.stamp.sec) + \
                   (msgs[-1].header.stamp.nanosec - msgs[0].header.stamp.nanosec) * 1e-9
            result["stats"]["sim_span_s"] = span
            result["stats"]["message_rate_hz_wall"] = n / wall_elapsed if wall_elapsed else None
            result["stats"]["sim_time_min"] = {"sec": msgs[0].header.stamp.sec,
                                               "nanosec": msgs[0].header.stamp.nanosec}
            result["stats"]["sim_time_max"] = {"sec": msgs[-1].header.stamp.sec,
                                               "nanosec": msgs[-1].header.stamp.nanosec}
            record("sim_time_advanced", span > 0.9 * wall_elapsed * 0.5,
                   "sim span %.2fs over %.2fs wall" % (span, wall_elapsed))

        record("pause_behavior", result["pause"].get("pause_ok", False),
               json.dumps(result["pause"])[:400])

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

    text = re.sub(r"\x1b\[[0-9;]*m", "", open(args.log, errors="ignore").read())
    result["log_forbidden_hits"] = FORBIDDEN_LOG_RE.findall(text)
    result["log_err_count"] = text.count("[Err]")
    record("no_forbidden_in_log", not result["log_forbidden_hits"],
           str(result["log_forbidden_hits"][:5]))
    record("no_err_in_log", result["log_err_count"] == 0,
           "%d [Err]" % result["log_err_count"])

    result["verdict"] = "PASS" if not failures else "FAIL"
    result["failures"] = failures
    with open(args.out, "w") as f:
        json.dump(result, f, indent=2)

    print(json.dumps({k: result[k] for k in
                      ("verdict", "failures", "stats", "pause", "log_err_count")}, indent=2))
    for name, c in result["checks"].items():
        print("  [%s] %s%s" % ("PASS" if c["ok"] else "FAIL", name,
                               "" if c["ok"] else " -- " + c["detail"]))
    return 0 if not failures else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SystemExit:
        raise
