#!/usr/bin/python3
"""Task 3D validation: minimal native Gazebo IMU on /imu/data.

Launches the workspace ``nao_sim.launch.py`` headless and verifies:

* ``/imu/data`` exists, is ``sensor_msgs/msg/Imu``, single publisher, RELIABLE +
  VOLATILE, ``frame_id == "ImuTorsoAccelerometer_frame"``, non-zero strictly
  increasing Gazebo timestamps;
* orientation is finite and normalized; angular velocity / linear acceleration
  are finite; covariance is all-zero (no noise model);
* a static window (after the un-controlled model has settled) has near-zero
  angular velocity, |linear_acceleration| ~= 9.8 m/s^2 and direction matching
  ``R_sensor_world^T * (0, 0, +9.8)`` (Gazebo reports specific force incl. gravity);
* pause/resume behavior; no forbidden topics; Task 3B/3C regressions.

Run with the ROS Python interpreter:

    /usr/bin/python3 tools/validate_task3d_imu.py --runs 2 --duration 30 \
        --out /tmp/task3d

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

from sensor_msgs.msg import Imu, JointState
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped

FRAME = "ImuTorsoAccelerometer_frame"
G = 9.8
ANG_VEL_TOL = 1e-3       # rad/s, generous vs measured ~1e-7
ACC_DIR_COS_MIN = 0.999
ACC_MAG_TOL = 0.05       # m/s^2
QUAT_NORM_TOL = 1e-6
SETTLE_S = 6.0           # ignore the initial collapse transient
FORBIDDEN_TOPIC_RE = re.compile(
    r"/imu/(gyro|accel|raw)|joint_command|ros2_control|controller_manager")
# NOTE (Task 3F): /tf and /tf_static are the frozen Task 3F outputs and are no
# longer forbidden; the IMU interface itself is unchanged.


class Collector(Node):
    def __init__(self):
        super().__init__("task3d_validator")
        rel = QoSProfile(depth=10)
        rel.reliability = ReliabilityPolicy.RELIABLE
        rel.durability = DurabilityPolicy.VOLATILE
        be = QoSProfile(depth=1)
        be.reliability = ReliabilityPolicy.BEST_EFFORT
        be.durability = DurabilityPolicy.VOLATILE
        self.imu = []
        self.clock = []
        self.joints = []
        self.body = []
        self.create_subscription(Imu, "/imu/data", lambda m: self.imu.append(m), rel)
        self.create_subscription(Clock, "/clock", lambda m: self.clock.append(m), be)
        self.create_subscription(JointState, "/joint_states",
                                 lambda m: self.joints.append(m), rel)
        self.create_subscription(PoseStamped, "/body_pose",
                                 lambda m: self.body.append(m), rel)


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


def ts(m):
    return (m.header.stamp.sec, m.header.stamp.nanosec)


def quat_to_R(q):
    x, y, z, w = q
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def Rt_times(R, v):
    # R^T * v
    return [sum(R[i][j] * v[i] for i in range(3)) for j in range(3)]


def run_once(args, index, env):
    partition = "t3d_%d_%d" % (index, int(time.time()))
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

    try:
        rclpy.init()
        node = Collector()
        got = spin_for(node, 60.0, stop_when=lambda: len(node.imu) > 0)
        if not got:
            result["blocked"] = "no /imu/data message within timeout"
            record("imu_receives", False, "no message")
            return result, failures

        t0 = time.time()
        spin_for(node, args.duration)
        wall = time.time() - t0

        # pause / resume
        pause_cmd = ["gz", "service", "-s", "/world/nao_world/control",
                     "--reqtype", "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean",
                     "--timeout", "3000", "--req", "pause: true"]
        resume_cmd = list(pause_cmd)
        resume_cmd[-1] = "pause: false"
        rc_p, out_p = run(pause_cmd, env=env, timeout=10)
        spin_for(node, 0.5)
        n0 = len(node.imu)
        spin_for(node, 2.0)
        paused = len(node.imu) - n0
        rc_r, out_r = run(resume_cmd, env=env, timeout=10)
        spin_for(node, 1.5)
        resumed = len(node.imu) - n0 - paused
        result["pause"] = {"pause_rc": rc_p, "resume_rc": rc_r,
                           "new_imu_during_pause": paused,
                           "new_imu_after_resume": resumed,
                           "pause_ok": (rc_p == 0 and rc_r == 0 and paused == 0
                                        and resumed > 0)}
        record("pause_behavior", result["pause"]["pause_ok"], json.dumps(result["pause"]))

        _, imu_type = run(["ros2", "topic", "type", "/imu/data"], env=env)
        _, imu_info = run(["ros2", "topic", "info", "-v", "/imu/data"], env=env)
        _, topics = run(["ros2", "topic", "list"], env=env)
        result["imu_type"] = imu_type.strip()
        result["imu_info"] = imu_info
        result["topic_list"] = sorted(topics.split())

        msgs = list(node.imu)
        record("imu_type", imu_type.strip() == "sensor_msgs/msg/Imu", imu_type.strip())
        pub = re.findall(r"Publisher count:\s*(\d+)", imu_info)
        record("imu_single_publisher", len(pub) == 1 and pub[0] == "1", str(pub))
        record("imu_qos_reliable", "Reliability: RELIABLE" in imu_info, "RELIABLE")
        record("imu_qos_volatile", "Durability: VOLATILE" in imu_info, "VOLATILE")
        forbidden = [t for t in result["topic_list"] if FORBIDDEN_TOPIC_RE.search(t)]
        record("no_forbidden_topics", not forbidden, str(forbidden))

        n = len(msgs)
        record("imu_count", n > 100, "%d messages / %.1fs wall" % (n, wall))
        frame_ok = all(m.header.frame_id == FRAME for m in msgs)
        record("frame_id_canonical", frame_ok,
               sorted({m.header.frame_id for m in msgs}))
        stamps = [ts(m) for m in msgs]
        record("stamp_nonzero", all(s != (0, 0) for s in stamps), "all non-zero")
        record("stamp_increasing",
               all(stamps[i] < stamps[i + 1] for i in range(n - 1)), "strictly increasing")

        # finite + quaternion norm + covariance
        finite = True
        norm_err = 0.0
        cov_ok = True
        for m in msgs:
            vals = [m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w,
                    m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                    m.linear_acceleration.x, m.linear_acceleration.y,
                    m.linear_acceleration.z]
            if not all(math.isfinite(v) for v in vals):
                finite = False
            q = [m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w]
            norm_err = max(norm_err, abs(math.sqrt(sum(v * v for v in q)) - 1.0))
            for cov in (m.orientation_covariance, m.angular_velocity_covariance,
                        m.linear_acceleration_covariance):
                if len(cov) != 9 or any(c != 0.0 for c in cov):
                    cov_ok = False
        record("values_finite", finite, "no NaN/Inf")
        record("quaternion_normalized", norm_err < QUAT_NORM_TOL,
               "max |norm-1|=%.2e" % norm_err)
        record("covariance_all_zero", cov_ok,
               "all three 9-element covariances are 0 (no modeled noise)")

        # static window stats
        t_first = stamps[0]

        def sim_t(s):
            return (s[0] - t_first[0]) + (s[1] - t_first[1]) * 1e-9

        static = [m for m in msgs if sim_t(ts(m)) >= SETTLE_S]
        record("static_window", len(static) > 50, "%d samples after %.1fs settle"
               % (len(static), SETTLE_S))
        if static:
            av = [[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]
                  for m in static]
            mean_av = [sum(v[i] for v in av) / len(av) for i in range(3)]
            rms_av = [math.sqrt(sum(v[i] ** 2 for v in av) / len(av)) for i in range(3)]
            max_av = max(math.sqrt(sum(x * x for x in v)) for v in av)
            mags, cosines = [], []
            for m in static:
                a = [m.linear_acceleration.x, m.linear_acceleration.y,
                     m.linear_acceleration.z]
                q = [m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w]
                nq = math.sqrt(sum(v * v for v in q))
                q = [v / nq for v in q]
                exp = Rt_times(quat_to_R(q), [0.0, 0.0, G])
                mag = math.sqrt(sum(v * v for v in a))
                mags.append(mag)
                denom = mag * G
                cosines.append(sum(a[i] * exp[i] for i in range(3)) / denom if denom else 0.0)
            result["stats"].update({
                "static_samples": len(static),
                "ang_vel_mean": mean_av, "ang_vel_rms": rms_av, "ang_vel_max_mag": max_av,
                "accel_mag_mean": sum(mags) / len(mags), "accel_mag_min": min(mags),
                "accel_mag_max": max(mags),
                "accel_dir_cos_mean": sum(cosines) / len(cosines),
                "accel_dir_cos_min": min(cosines),
            })
            record("static_ang_velocity_near_zero", max_av < ANG_VEL_TOL,
                   "max|w|=%.3e rad/s (tol %.0e)" % (max_av, ANG_VEL_TOL))
            record("accel_magnitude_9p8",
                   abs(sum(mags) / len(mags) - G) < ACC_MAG_TOL,
                   "mean|a|=%.4f m/s^2" % (sum(mags) / len(mags)))
            record("accel_gravity_direction",
                   min(cosines) > ACC_DIR_COS_MIN,
                   "min cos(a, R^T*(0,0,+g))=%.6f (tol %.3f)"
                   % (min(cosines), ACC_DIR_COS_MIN))

        # regressions
        record("joint_states_present", len(node.joints) > 0, "%d" % len(node.joints))
        if node.joints:
            js = [(m.header.stamp.sec, m.header.stamp.nanosec) for m in node.joints]
            record("joint_states_ok",
                   all(s != (0, 0) for s in js)
                   and all(len(m.effort) == 0 for m in node.joints), "sim stamp, effort empty")
        record("body_pose_present",
               len(node.body) > 0 and all(m.header.frame_id == "world" for m in node.body),
               "%d messages" % len(node.body))
        record("clock_present", len(node.clock) > 0, "%d" % len(node.clock))

        if n >= 2:
            span = (stamps[-1][0] - stamps[0][0]) + (stamps[-1][1] - stamps[0][1]) * 1e-9
            result["stats"].update({
                "imu_count": n, "wall_s": wall, "sim_span_s": span,
                "rate_hz_sim": (n - 1) / span if span > 0 else None,
                "rate_hz_wall": n / wall if wall else None,
                "first_stamp": stamps[0], "last_stamp": stamps[-1],
            })

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
        run(["bash", "-lc",
             "ps -eo pid,args --no-headers | grep -E 'imu_converter|joint_state_converter|"
             "body_pose_converter' | grep -v grep | awk '{print $1}' | xargs -r kill"],
            env=env, timeout=10)

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
    ap.add_argument("--out", default="/tmp/task3d")
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
        print("  checks %d/%d; stats=%s; pause=%s" % (
            sum(1 for c in res["checks"].values() if c["ok"]), len(res["checks"]),
            json.dumps(res.get("stats", {}))[:420], res.get("pause", {}).get("pause_ok")))
        all_failures += ["run%d %s" % (i, f) for f in fails]

    print("\nTASK 3D VERDICT: %s" % ("PASS" if not all_failures else "FAIL"))
    if all_failures:
        print("failures: %s" % all_failures)
    return 0 if not all_failures else 1


if __name__ == "__main__":
    sys.exit(main())
