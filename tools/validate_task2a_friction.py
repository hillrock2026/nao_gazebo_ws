#!/usr/bin/env python3
"""Task 2A P0-1: reproducible DART friction probe on a tilted-gravity box.

The probe is intentionally independent of the canonical NAO model. It creates a
temporary world (static ground plane) and a temporary single-box model, both
with explicit ``<surface><friction><ode><mu>/<mu2>`` values, runs Gazebo Sim
server-only with the DART engine and records the box's world trajectory.

A tilted gravity vector ``g = (-g*sin(theta), 0, -g*cos(theta))`` is used so a
flat ground is equivalent to an inclined plane: with zero friction the box
accelerates down-hill with ``a = g*sin(theta)``; when the effective friction
coefficient exceeds ``tan(theta)`` the box stays static. Only the SDF friction
values are varied between runs.

No controller, bridge, joint command or sensor is used. The canonical NAO SDF,
world and launch are never modified.

Usage:
    python3 tools/validate_task2a_friction.py --mu 0 --out /tmp/fric_mu0.json
    python3 tools/validate_task2a_friction.py --mu 1 --out /tmp/fric_mu1.json
"""

import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import physics_baseline as pb  # noqa: E402

ENGINE = "gz-physics-dartsim-plugin"
WORLD = "friction_world"


def gz_version():
    try:
        out = subprocess.run(["gz", "sim", "--version"], capture_output=True,
                             text=True, timeout=10)
        return (out.stdout + out.stderr).strip().splitlines()[0]
    except Exception as exc:  # noqa: BLE001
        return "unknown: %s" % exc


def make_world(gravity, mu, mu2):
    return """<?xml version="1.0" ?>
<sdf version="1.11">
  <world name="{world}">
    <gravity>{gx} {gy} {gz}</gravity>
    <physics name="1ms" type="dartsim">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface>
            <friction><ode><mu>{mu}</mu><mu2>{mu2}</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
""".format(world=WORLD, gx=gravity[0], gy=gravity[1], gz=gravity[2], mu=mu, mu2=mu2)


def make_box(size, mass, mu, mu2):
    i = mass * (size * size) / 6.0
    return """<?xml version="1.0" ?>
<sdf version='1.11'>
  <model name='slider'>
    <link name='box'>
      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{i}</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>{i}</iyy><iyz>0</iyz><izz>{i}</izz>
        </inertia>
      </inertial>
      <collision name='box_collision'>
        <pose>0 0 0 0 0 0</pose>
        <geometry><box><size>{s} {s} {s}</size></box></geometry>
        <surface>
          <friction><ode><mu>{mu}</mu><mu2>{mu2}</mu2></ode></friction>
        </surface>
      </collision>
    </link>
  </model>
</sdf>
""".format(mass=mass, i=i, s=size, mu=mu, mu2=mu2)


def run(args):
    gravity = (-9.8 * math.sin(math.radians(args.tilt_deg)), 0.0,
               -9.8 * math.cos(math.radians(args.tilt_deg)))
    workdir = tempfile.mkdtemp(prefix="task2a_friction_")
    world_path = os.path.join(workdir, "friction_world.sdf")
    box_path = os.path.join(workdir, "box.sdf")
    open(world_path, "w").write(make_world(gravity, args.ground_mu, args.ground_mu2))
    open(box_path, "w").write(make_box(args.box_size, args.mass, args.box_mu, args.box_mu2))

    env = dict(os.environ)
    env["GZ_PARTITION"] = "fric_%d" % int(time.time() * 1000)
    log_path = args.log or os.path.join(workdir, "friction.log")
    log_f = open(log_path, "w")
    cmd = ["gz", "sim", "-s", "-r", "-v", "4",
           "--physics-engine", ENGINE, world_path]
    proc = subprocess.Popen(cmd, stdout=log_f, stderr=subprocess.STDOUT,
                            env=env, start_new_session=True)

    result = {
        "engine_requested": ENGINE,
        "gz_version": gz_version(),
        "command": " ".join(cmd),
        "world": WORLD,
        "world_file": world_path,
        "box_file": box_path,
        "log": log_path,
        "gravity_vector": list(gravity),
        "tilt_deg": args.tilt_deg,
        "tan_tilt": math.tan(math.radians(args.tilt_deg)),
        "ground_friction": {"mu": args.ground_mu, "mu2": args.ground_mu2},
        "box_friction": {"mu": args.box_mu, "mu2": args.box_mu2},
        "box": {"size": args.box_size, "mass": args.mass,
                "spawn_xyz": [0.0, 0.0, args.spawn_z], "initial_velocity": [0, 0, 0]},
        "sliding_threshold_m": args.slide_threshold,
        "samples": [],
    }
    try:
        time.sleep(args.warmup)
        if proc.poll() is not None:
            result["server_died"] = True
            return result
        create = subprocess.run(
            ["ros2", "run", "ros_gz_sim", "create", "-world", WORLD,
             "-file", box_path, "-name", "slider",
             "-x", "0", "-y", "0", "-z", str(args.spawn_z)],
            capture_output=True, text=True, env=env, timeout=30)
        result["spawn_output"] = (create.stdout + create.stderr).strip()
        result["spawn_ok"] = "successful" in result["spawn_output"].lower()

        t0 = time.time()
        period = 1.0 / args.hz
        while time.time() - t0 < args.duration:
            t = time.time() - t0
            poses = pb.sample_poses(env, WORLD)
            if "slider" in poses:
                result["samples"].append({"t": round(t, 4), "xyz": poses["slider"][0]})
            time.sleep(max(0.0, period - (time.time() - t0 - t)))
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        log_f.close()

    text = re.sub(r"\x1b\[[0-9;]*m", "", open(log_path, errors="ignore").read())
    m = re.search(r"Loaded \[[^\]]*\]", text)
    result["loaded_engine"] = m.group(0) if m else ""
    result["engine_ok"] = "dartsim" in result["loaded_engine"]

    xs = [s["xyz"][0] for s in result["samples"]]
    zs = [s["xyz"][2] for s in result["samples"]]
    speeds = []
    for a, b in zip(result["samples"], result["samples"][1:]):
        dt = b["t"] - a["t"]
        if dt > 0:
            speeds.append(math.dist(a["xyz"], b["xyz"]) / dt)
    if xs:
        result["x0"] = xs[0]
        result["final_x"] = xs[-1]
        result["final_z"] = zs[-1]
        result["final_dx"] = xs[-1] - xs[0]
        result["max_abs_dx"] = max(abs(x - xs[0]) for x in xs)
        result["max_speed"] = max(speeds) if speeds else 0.0
        result["slid"] = result["max_abs_dx"] > args.slide_threshold
    result["sample_count"] = len(result["samples"])

    ok = (result.get("spawn_ok") and result["engine_ok"] and result["sample_count"] > 0
          and not result.get("server_died"))
    result["probe_ok"] = bool(ok)
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mu", type=float, default=None,
                    help="Sets both ground and box mu/mu2 unless overridden.")
    ap.add_argument("--ground-mu", type=float, default=None)
    ap.add_argument("--ground-mu2", type=float, default=None)
    ap.add_argument("--box-mu", type=float, default=None)
    ap.add_argument("--box-mu2", type=float, default=None)
    ap.add_argument("--tilt-deg", type=float, default=15.0)
    ap.add_argument("--box-size", type=float, default=0.1)
    ap.add_argument("--mass", type=float, default=1.0)
    ap.add_argument("--spawn-z", type=float, default=0.05)
    ap.add_argument("--duration", type=float, default=6.0)
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--warmup", type=float, default=7.0)
    ap.add_argument("--slide-threshold", type=float, default=0.02)
    ap.add_argument("--log", default="")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    base = args.mu if args.mu is not None else 1.0
    if args.ground_mu is None:
        args.ground_mu = base
    if args.ground_mu2 is None:
        args.ground_mu2 = args.ground_mu
    if args.box_mu is None:
        args.box_mu = base
    if args.box_mu2 is None:
        args.box_mu2 = args.box_mu

    result = run(args)
    text = json.dumps(result, indent=2)
    if args.out:
        open(args.out, "w").write(text)
    print(text)
    return 0 if result.get("probe_ok") else 1


if __name__ == "__main__":
    sys.exit(main())
