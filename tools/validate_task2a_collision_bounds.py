#!/usr/bin/env python3
"""Task 2A P0-2: compute the minimum collision-geometry z of a NAO run.

Reads the canonical ``nao_simple.sdf`` (all 33 collisions: 31 box, 2 sphere)
and one or more ``validate_task2a.py`` report JSONs. For every sample it
composes, for every collision:

    world_point = link_world_pos
                + R(link_world_quat) * (collision_pose_xyz
                                        + R(collision_pose_rpy) * corner)

where ``link_world_pos`` / ``link_world_quat`` come from the run report (which
itself composes the model world pose from ``/world/<world>/pose/info`` with the
parent-relative link pose published by Gazebo). Joint state is therefore
included through the reported link transforms; no separate joint query exists
or is needed.

For a sphere the lowest point is ``world_center_z - radius``.

Usage:
    python3 tools/validate_task2a_collision_bounds.py \
        --report docs/task2a_evidence/report_run1.json \
        --out docs/task2a_evidence/collision_bounds_run1.json
    python3 tools/validate_task2a_collision_bounds.py --selftest
"""

import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SDF = os.path.join(REPO_ROOT, "src", "nao", "nao_description", "urdf", "nao_simple.sdf")


# ---------------------------------------------------------------- quaternion
def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    # q = qz(yaw) * qy(pitch) * qx(roll)  == Rz*Ry*Rx
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


def quat_rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return [vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx)]


def quat_to_matrix(q):
    x, y, z, w = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def mat_rotate(m, v):
    return [sum(m[i][j] * v[j] for j in range(3)) for i in range(3)]


# ------------------------------------------------------------------- parsing
def parse_collisions(sdf_path):
    model = ET.parse(sdf_path).getroot().find("model")
    collisions = []
    for link in model.findall("link"):
        link_name = link.get("name")
        for col in link.findall("collision"):
            pose_el = col.find("pose")
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
            if pose_el is not None and pose_el.text:
                vals = [float(v) for v in pose_el.text.split()]
                xyz, rpy = vals[0:3], vals[3:6]
            geom = col.find("geometry")
            child = list(geom)[0] if geom is not None and len(list(geom)) else None
            if child is None:
                raise RuntimeError("collision %s has no geometry" % col.get("name"))
            kind = child.tag
            if kind == "box":
                size = [float(v) for v in child.find("size").text.split()]
            elif kind == "sphere":
                radius = float(child.find("radius").text)
                size = [radius]
            else:
                raise RuntimeError("unexpected geometry %s in %s" % (kind, col.get("name")))
            collisions.append({
                "name": col.get("name"), "link": link_name, "type": kind,
                "pose_xyz": xyz, "pose_rpy": rpy,
                "size": size,
            })
    return collisions


# --------------------------------------------------------------- computation
def collision_min_z(col, link_pos, link_quat):
    cq = quat_from_rpy(*col["pose_rpy"])
    if col["type"] == "sphere":
        world_center = [link_pos[i] + quat_rotate(link_quat, col["pose_xyz"])[i]
                        for i in range(3)]
        return world_center[2] - col["size"][0]
    half = [s * 0.5 for s in col["size"]]
    min_z = float("inf")
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                corner = [sx * half[0], sy * half[1], sz * half[2]]
                local = [col["pose_xyz"][i] + quat_rotate(cq, corner)[i] for i in range(3)]
                world = [link_pos[i] + quat_rotate(link_quat, local)[i] for i in range(3)]
                min_z = min(min_z, world[2])
    return min_z


def compute(report, collisions, tolerance, stable_fraction):
    samples = report["samples"]
    per_collision = {c["name"]: {"min_z": float("inf"), "link": c["link"],
                                 "type": c["type"], "time": None} for c in collisions}
    global_min = {"z": float("inf"), "collision": None, "link": None, "time": None}
    sample_mins = []
    for entry in samples:
        t = entry.get("t")
        sample_min = float("inf")
        for col in collisions:
            link_pose = entry.get(col["link"])
            if not link_pose or "xyz" not in link_pose or "quat" not in link_pose:
                continue
            z = collision_min_z(col, link_pose["xyz"], link_pose["quat"])
            sample_min = min(sample_min, z)
            if z < per_collision[col["name"]]["min_z"]:
                per_collision[col["name"]]["min_z"] = z
                per_collision[col["name"]]["time"] = t
            if z < global_min["z"]:
                global_min.update(z=z, collision=col["name"], link=col["link"], time=t)
        sample_mins.append(sample_min)
    n_stable = max(1, int(math.ceil(len(samples) * stable_fraction)))
    tail = [m for m in sample_mins[-n_stable:] if m != float("inf")]
    return {
        "per_collision": per_collision,
        "global_min_z": global_min["z"],
        "global_min_collision": global_min["collision"],
        "global_min_link": global_min["link"],
        "global_min_time": global_min["time"],
        "stable_tail_min_z": min(tail) if tail else None,
        "stable_tail_samples": n_stable,
        "penetration_m": max(0.0, -global_min["z"]),
        "below_ground": global_min["z"] < 0.0,
        "tolerance_m": tolerance,
        "within_tolerance": global_min["z"] >= -tolerance,
    }


# ------------------------------------------------------------------ selftest
def selftest():
    ok = True
    # 1) quaternion rotation vs rotation matrix on a set of vectors.
    for rpy in [(0.3, -0.5, 1.1), (0.0, 0.0, math.pi / 2), (1.2, 0.4, -0.7)]:
        q = quat_from_rpy(*rpy)
        m = quat_to_matrix(q)
        for v in [(0.1, -0.2, 0.3), (1, 0, 0), (0, 0, 1)]:
            a = quat_rotate(q, v)
            b = mat_rotate(m, v)
            if max(abs(a[i] - b[i]) for i in range(3)) > 1e-12:
                print("SELFTEST FAIL quat/matrix", rpy, v, a, b)
                ok = False
    # 2) box at identity, pose z=0.05, size 0.1 -> min z = 0.0
    col = {"type": "box", "pose_xyz": [0, 0, 0.05], "pose_rpy": [0, 0, 0],
           "size": [0.1, 0.1, 0.1]}
    z = collision_min_z(col, [0, 0, 0], [0, 0, 0, 1])
    if abs(z) > 1e-12:
        print("SELFTEST FAIL identity box", z)
        ok = False
    # 3) box centre (0.1,0,0), yaw 90 deg -> world centre (0,0.1,0), min z=-0.05
    col = {"type": "box", "pose_xyz": [0.1, 0, 0], "pose_rpy": [0, 0, math.pi / 2],
           "size": [0.1, 0.1, 0.1]}
    z = collision_min_z(col, [0, 0, 0], [0, 0, 0, 1])
    if abs(z - (-0.05)) > 1e-12:
        print("SELFTEST FAIL rotated box", z)
        ok = False
    # 4) sphere centre z=0.1 r=0.03 -> 0.07
    col = {"type": "sphere", "pose_xyz": [0, 0, 0.1], "pose_rpy": [0, 0, 0],
           "size": [0.03]}
    z = collision_min_z(col, [1, 2, 3], [0, 0, 0, 1])
    if abs(z - 3.07) > 1e-12:
        print("SELFTEST FAIL sphere", z)
        ok = False
    print("SELFTEST %s" % ("PASS" if ok else "FAIL"))
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sdf", default=DEFAULT_SDF)
    ap.add_argument("--report", action="append", default=[])
    ap.add_argument("--out", default="")
    ap.add_argument("--ground-z", type=float, default=0.0)
    ap.add_argument("--tolerance", type=float, default=0.005,
                    help="Declared acceptable penetration depth in metres.")
    ap.add_argument("--stable-fraction", type=float, default=0.25)
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return 0 if selftest() else 1

    if not args.report:
        print("no --report given", file=sys.stderr)
        return 2

    collisions = parse_collisions(args.sdf)
    type_counts = {}
    for c in collisions:
        type_counts[c["type"]] = type_counts.get(c["type"], 0) + 1
    print("collisions parsed: %d %s" % (len(collisions), type_counts))

    outputs = []
    exit_ok = True
    for rep_path in args.report:
        report = json.load(open(rep_path))
        res = compute(report, collisions, args.tolerance, args.stable_fraction)
        out = {
            "engine": report.get("engine", ""),
            "world": "nao_world",
            "model": "nao",
            "ground_z": args.ground_z,
            "source_sdf": os.path.abspath(args.sdf),
            "source_report": os.path.abspath(rep_path),
            "sample_count": len(report["samples"]),
            "collision_count": len(collisions),
            "collision_types": type_counts,
            "transform_method": (
                "world = link_world_pos + R(link_world_quat) * "
                "(collision_pose_xyz + R(collision_pose_rpy) * corner); "
                "link_world pose taken from pose/info composed with model world pose"),
            "threshold": {"ground_z": args.ground_z, "tolerance_m": args.tolerance,
                          "stable_fraction": args.stable_fraction},
            **res,
        }
        outputs.append(out)
        print("  %s: global_min_z=%.6f @ t=%s (%s / %s)  tail_min_z=%.6f  below_ground=%s"
              % (os.path.basename(rep_path), out["global_min_z"], out["global_min_time"],
                 out["global_min_link"], out["global_min_collision"],
                 out["stable_tail_min_z"], out["below_ground"]))
        if not out["within_tolerance"]:
            exit_ok = False

    if args.out:
        open(args.out, "w").write(json.dumps(outputs, indent=2))
    return 0 if exit_ok else 1


if __name__ == "__main__":
    sys.exit(main())
