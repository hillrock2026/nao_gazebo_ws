#!/usr/bin/env python3
"""Headless physics-baseline harness for the NAO Gazebo Sim model (Task 2A).

This tool only *observes* the simulation. It never writes to the canonical
model, never injects controllers, never publishes joint commands and never
starts a ROS-Gazebo bridge. It runs ``gz sim`` server-only, spawns a model with
``ros_gz_sim create`` and samples world poses of a model and selected links
through the standard ``gz model`` CLI.

Example:
    python3 tools/physics_baseline.py \
        --engine bullet --model src/nao/nao_description/urdf/nao_simple.sdf \
        --name nao --spawn-z 0.35 --duration 15 --hz 4 \
        --links base_link,r_ankle,l_ankle \
        --out /tmp/nao_bullet.json
"""

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time

# gz-sim 8 selects the physics engine through the --physics-engine CLI option.
ENGINE_PLUGINS = {
    "dartsim": "gz-physics-dartsim-plugin",
    "bullet": "gz-physics-bullet-plugin",
    "bullet-featherstone": "gz-physics-bullet-featherstone-plugin",
    "tpe": "gz-physics-tpe-plugin",
}

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_WORLD = os.path.join(REPO_ROOT, "src", "nao_gz_sim", "worlds", "nao_world.sdf")

NUM_TRIPLE = r"\[\s*([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s*\]"


def _bracket_lines(text):
    out = []
    for line in text.splitlines():
        s = line.strip()
        m = re.fullmatch(r"\[([-+0-9.eE\s]+)\]", s)
        if m and len(m.group(1).split()) == 3:
            out.append([float(v) for v in m.group(1).split()])
    return out


def parse_entity_pose(text):
    """Return (xyz, rpy) for either ``gz model -m NAME -p`` or ``-l LINK``."""
    # Match the *link* "Pose [ XYZ ... ]" header, not "Inertial Pose".
    blocks = re.findall(
        r"-\s+Pose\s+\[[^\]]*XYZ[^\]]*\][^\n]*\n((?:\s*\[[^\]]*\]\s*\n?)+)", text)
    if not blocks:
        return None
    nums = _bracket_lines(blocks[-1])
    if len(nums) >= 2:
        return nums[0], nums[1]
    return None


def run_gz(args, env=None, timeout=10):
    try:
        p = subprocess.run(["gz"] + args, capture_output=True, text=True,
                           timeout=timeout, env=env)
        return p.stdout + p.stderr
    except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
        return "ERROR: %s" % exc


POSE_RE = re.compile(r'pose\s*\{\s*name:\s*"([^"]+)"(.*?)\n\}', re.S)


def _vec(block, key, names):
    m = re.search(r"%s\s*\{([^}]*)\}" % key, block)
    vals = [0.0] * len(names)
    if m:
        for i, k in enumerate(names):
            km = re.search(r"\b%s:\s*([-+0-9.eE]+)" % k, m.group(1))
            if km:
                vals[i] = float(km.group(1))
    return vals


def _quat_rotate(q, v):
    """Rotate vector v by quaternion q = (x, y, z, w)."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * cross(q.xyz, v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return [vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx)]


def sample_poses(env, world_name):
    """Return name -> (xyz, quat_xyzw) from one pose/info message.

    ``gz sim`` publishes pose/info poses relative to the parent entity, so a
    model is relative to the world and a link is relative to its model.
    """
    out = {}
    text = run_gz(["topic", "-e", "-t",
                   "/world/%s/pose/info" % world_name, "-n", "1"], env=env)
    for m in POSE_RE.finditer(text):
        out[m.group(1)] = (_vec(m.group(2), "position", "xyz"),
                           _vec(m.group(2), "orientation", "xyzw"))
    return out


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


def resolve_world(poses, model_name, link_names):
    """Compose model world transform with parent-relative link poses.

    Returns name -> ((x, y, z), quat_xyzw) in world coordinates.
    """
    result = {}
    if model_name not in poses:
        return result
    mpos, mquat = poses[model_name]
    result[model_name] = (mpos, mquat)
    for name in link_names:
        if name not in poses:
            continue
        lpos, lquat = poses[name]
        r = _quat_rotate(mquat, lpos)
        result[name] = ([mpos[i] + r[i] for i in range(3)],
                        _quat_mul(mquat, lquat))
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--engine", default="dartsim", choices=sorted(ENGINE_PLUGINS))
    ap.add_argument("--world", default=DEFAULT_WORLD)
    ap.add_argument("--model", required=True)
    ap.add_argument("--name", default="nao")
    ap.add_argument("--world-name", default="nao_world")
    ap.add_argument("--spawn-z", type=float, default=0.35)
    ap.add_argument("--spawn-x", type=float, default=0.0)
    ap.add_argument("--spawn-y", type=float, default=0.0)
    ap.add_argument("--duration", type=float, default=15.0)
    ap.add_argument("--hz", type=float, default=4.0)
    ap.add_argument("--warmup", type=float, default=7.0)
    ap.add_argument("--links", default="base_link,r_ankle,l_ankle")
    ap.add_argument(
        "--probes", default="",
        help="Semicolon-separated probe points fixed in a link frame, "
             "formatted as NAME:LINK:ox,oy,oz (world position is recorded).")
    ap.add_argument("--out", default="")
    ap.add_argument("--keep-log", default="")
    args = ap.parse_args()

    links = [s for s in args.links.split(",") if s]
    probes = []
    for spec in [s for s in args.probes.split(";") if s]:
        pname, ref, offset = spec.split(":")
        probes.append((pname, ref, [float(v) for v in offset.split(",")]))
        if ref not in links:
            links.append(ref)
    plugin = ENGINE_PLUGINS[args.engine]
    partition = "pb_%s_%d" % (args.engine, int(time.time()))

    env = dict(os.environ)
    env["GZ_PARTITION"] = partition
    # Resource path for model://nao_meshes if the caller did not provide one.
    if "GZ_SIM_RESOURCE_PATH" not in env:
        for cand in (
            os.path.join(REPO_ROOT, "install", "nao_meshes", "share"),
            os.path.join(REPO_ROOT, "src", "nao_meshes2", "share"),
        ):
            if os.path.isdir(cand):
                env["GZ_SIM_RESOURCE_PATH"] = cand
                break

    log_path = args.keep_log or os.path.join("/tmp", "pb_%s.log" % args.engine)
    log_f = open(log_path, "w")
    cmd = ["gz", "sim", "-s", "-r", "-v", "4", "--physics-engine", plugin, args.world]
    proc = subprocess.Popen(cmd, stdout=log_f, stderr=subprocess.STDOUT,
                            env=env, start_new_session=True)

    result = {"engine": args.engine, "plugin": plugin, "model": args.model,
              "spawn_z": args.spawn_z, "duration": args.duration,
              "samples": [], "errors": {}, "log": log_path}
    try:
        time.sleep(args.warmup)
        if proc.poll() is not None:
            result["startup_failed"] = True
        else:
            create = subprocess.run(
                ["ros2", "run", "ros_gz_sim", "create",
                 "-world", args.world_name, "-file", args.model,
                 "-name", args.name,
                 "-x", str(args.spawn_x), "-y", str(args.spawn_y), "-z", str(args.spawn_z)],
                capture_output=True, text=True, env=env, timeout=30)
            result["spawn_output"] = (create.stdout + create.stderr).strip()
            result["spawn_ok"] = "successful" in result["spawn_output"].lower()

            t0 = time.time()
            period = 1.0 / args.hz
            while time.time() - t0 < args.duration:
                t = time.time() - t0
                entry = {"t": round(t, 3)}
                poses = sample_poses(env, args.world_name)
                world = resolve_world(poses, args.name, links)
                if args.name in world:
                    entry["model"] = {"xyz": world[args.name][0],
                                      "quat": world[args.name][1]}
                for lk in links:
                    if lk in world:
                        entry[lk] = {"xyz": world[lk][0], "quat": world[lk][1]}
                for pname, ref, offset in probes:
                    if ref in world:
                        pos, quat = world[ref]
                        r = _quat_rotate(quat, offset)
                        entry[pname] = {"xyz": [pos[i] + r[i] for i in range(3)]}
                result["samples"].append(entry)
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

    text = open(log_path, errors="ignore").read()
    result["errors"] = {
        "mesh_not_implemented": text.count("has not been implemented yet for dartsim"),
        "geometry_could_not_create": text.count("couldn't be created"),
        "failed_load_mesh": len(re.findall(
            r"Failed to load mesh|Unable to find file with URI", text)),
        "sdf_error": len(re.findall(r"\[Err\]", text)),
    }
    result["loaded_engine"] = ""
    m = re.search(r"Loaded \[[^\]]*\]", text)
    if m:
        result["loaded_engine"] = re.sub(r"\x1b\[[0-9;]*m", "", m.group(0))

    out = json.dumps(result, indent=2)
    if args.out:
        with open(args.out, "w") as f:
            f.write(out)
    print(out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
