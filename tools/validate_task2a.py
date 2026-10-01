#!/usr/bin/env python3
"""Task 2A headless validation driver for the canonical NAO model.

Runs the existing ``nao_sim.launch.py`` (server-only, no controller, no bridge,
no robot_state_publisher), spawns the canonical ``nao_simple.sdf`` and samples
world poses and foot-sole probe points from the standard ``pose/info`` topic.

The script only observes the running simulation; it never publishes joint
commands and never starts a controller.

Usage:
    python3 tools/validate_task2a.py --runs 2 --duration 30 --out /tmp/task2a
"""

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import physics_baseline as pb  # noqa: E402

REPO_ROOT = pb.REPO_ROOT

ALL_LINKS = [
    "base_link", "Neck", "Head", "LPelvis", "LHip", "LThigh", "LTibia",
    "LAnklePitch_link", "l_ankle", "LShoulder", "LBicep", "LElbow", "LForeArm",
    "l_wrist", "RPelvis", "RHip", "RThigh", "RTibia", "RAnklePitch_link",
    "r_ankle", "RShoulder", "RBicep", "RElbow", "RForeArm", "r_wrist",
]
SOLE_PROBES = [("r_sole", "r_ankle", [0, 0, -0.04511]),
               ("l_sole", "l_ankle", [0, 0, -0.04511])]


def strip_ansi(text):
    return re.sub(r"\x1b\[[0-9;]*m", "", text)


def run_once(args, index):
    partition = "task2a_run%d_%d" % (index, int(time.time()))
    env = dict(os.environ)
    env["GZ_PARTITION"] = partition
    log_path = os.path.join(args.out, "launch_run%d.log" % index)
    log_f = open(log_path, "w")

    cmd = ["ros2", "launch", "nao_gz_sim", "nao_sim.launch.py",
           "gui:=false", "use_rsp:=false",
           "spawn_z:=%s" % args.spawn_z,
           "physics_engine:=%s" % args.physics_engine]
    proc = subprocess.Popen(cmd, stdout=log_f, stderr=subprocess.STDOUT,
                            env=env, start_new_session=True)

    result = {"run": index, "log": log_path, "samples": []}
    links = ["base_link", "r_ankle", "l_ankle"] + ALL_LINKS
    try:
        time.sleep(args.warmup)
        t0 = time.time()
        period = 1.0 / args.hz
        while time.time() - t0 < args.duration:
            t = time.time() - t0
            poses = pb.sample_poses(env, "nao_world")
            world = pb.resolve_world(poses, "nao", links)
            entry = {"t": round(t, 3)}
            if "nao" in world:
                entry["model"] = {"xyz": world["nao"][0], "quat": world["nao"][1]}
            for lk in set(links):
                if lk in world:
                    entry[lk] = {"xyz": world[lk][0], "quat": world[lk][1]}
            for pname, ref, offset in SOLE_PROBES:
                if ref in world:
                    pos, quat = world[ref]
                    r = pb._quat_rotate(quat, offset)
                    entry[pname] = {"xyz": [pos[i] + r[i] for i in range(3)]}
            result["samples"].append(entry)
            time.sleep(max(0.0, period - (time.time() - t0 - t)))
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        log_f.close()

    text = strip_ansi(open(log_path, errors="ignore").read())
    result["spawn_ok"] = "Entity creation successful" in text
    result["engine"] = (re.search(r"Loaded \[[^\]]*\]", text) or [None])[0]
    result["errors"] = {
        "mesh_not_implemented": text.count("has not been implemented yet for dartsim"),
        "geometry_could_not_create": text.count("couldn't be created"),
        "failed_load_mesh": len(re.findall(
            r"Failed to load mesh|Unable to find file with URI", text)),
        "Err": len(re.findall(r"\[Err\]", text)),
    }
    result["forbidden"] = {
        "controller": len(re.findall(r"controller_manager|ros2_control", text, re.I)),
        "bridge": len(re.findall(r"ros_gz_bridge|parameter_bridge", text, re.I)),
        "joint_command": len(re.findall(
            r"joint_command|JointTrajectory|command_topic", text, re.I)),
    }
    return result


def summarize(result):
    s = result["samples"]
    if not s:
        return "NO SAMPLES"
    last = s[-1]
    base = last.get("model", {}).get("xyz", [0, 0, 0])
    rs = [e["r_sole"]["xyz"][2] for e in s if "r_sole" in e]
    ls = [e["l_sole"]["xyz"][2] for e in s if "l_sole" in e]
    zs = [v["xyz"][2] for v in last.values()
          if isinstance(v, dict) and "xyz" in v and "quat" in v]
    return ("spawn=%s engine=%s errors=%s forbidden=%s\n"
            "  final base=%s  min_sole_z=%.4f  min_link_origin_z=%.4f"
            % (result["spawn_ok"], result["engine"], result["errors"],
               result["forbidden"], [round(v, 4) for v in base],
               min(rs + ls) if rs and ls else float("nan"),
               min(zs) if zs else float("nan")))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--runs", type=int, default=2)
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--hz", type=float, default=4.0)
    ap.add_argument("--warmup", type=float, default=11.0)
    ap.add_argument("--spawn-z", default="0.40")
    ap.add_argument("--physics-engine", default="gz-physics-dartsim-plugin")
    ap.add_argument("--out", default="/tmp/task2a_validation")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    results = []
    for i in range(1, args.runs + 1):
        r = run_once(args, i)
        results.append(r)
        print("=== run %d ===" % i)
        print(summarize(r))
        reports = open(os.path.join(args.out, "report_run%d.json" % i), "w")
        json.dump(r, reports, indent=2)
        reports.close()

    print("\nreproducibility:")
    keys = ("model", "r_sole", "l_sole")
    for k in keys:
        vals = []
        for r in results:
            if r["samples"]:
                v = r["samples"][-1].get(k, {}).get("xyz")
                if v:
                    vals.append([round(x, 4) for x in v])
        print("  final %-8s %s" % (k, vals))
    return 0


if __name__ == "__main__":
    sys.exit(main())
