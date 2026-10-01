#!/usr/bin/env python3
"""Read-only inventory and SDF/URDF comparison for the NAO model files.

This script only parses XML. It never writes to the model files and never
launches Gazebo. It is the reproducible generator for the tables in
docs/Task0B_Model_and_Gazebo_Architecture_Decision.md.

Usage:
    python3 tools/model_inventory.py [path/to/nao_simple.sdf] [path/to/nao_simple.urdf]
"""

import sys
import os
import xml.etree.ElementTree as ET

DEFAULT_SDF = "src/nao/nao_description/urdf/nao_simple.sdf"
DEFAULT_URDF = "src/nao/nao_description/urdf/nao_simple.urdf"


def parse_sdf(path):
    model = ET.parse(path).getroot().find("model")
    links, joints, frames = {}, {}, {}
    for link in model.findall("link"):
        links[link.get("name")] = link
    for joint in model.findall("joint"):
        axis = joint.find("axis")
        limit = axis.find("limit") if axis is not None else None
        joints[joint.get("name")] = {
            "type": joint.get("type"),
            "parent": joint.find("parent").text.strip(),
            "child": joint.find("child").text.strip(),
            "axis": axis.find("xyz").text.strip() if axis is not None and axis.find("xyz") is not None else "",
            "lower": limit.find("lower").text if limit is not None and limit.find("lower") is not None else "",
            "upper": limit.find("upper").text if limit is not None and limit.find("upper") is not None else "",
            "effort": limit.find("effort").text if limit is not None and limit.find("effort") is not None else "",
            "velocity": limit.find("velocity").text if limit is not None and limit.find("velocity") is not None else "",
        }
    for frame in model.findall("frame"):
        frames[frame.get("name")] = frame
    return {"model": model.get("name"), "model_element": model,
            "links": links, "joints": joints, "frames": frames}


def parse_urdf(path):
    root = ET.parse(path).getroot()
    links, joints = {}, {}
    for link in root.findall("link"):
        links[link.get("name")] = link
    for joint in root.findall("joint"):
        axis = joint.find("axis")
        limit = joint.find("limit")
        origin = joint.find("origin")
        joints[joint.get("name")] = {
            "type": joint.get("type"),
            "parent": joint.find("parent").get("link"),
            "child": joint.find("child").get("link"),
            "axis": axis.get("xyz") if axis is not None else "",
            "lower": limit.get("lower") if limit is not None else "",
            "upper": limit.get("upper") if limit is not None else "",
            "effort": limit.get("effort") if limit is not None else "",
            "velocity": limit.get("velocity") if limit is not None else "",
            "origin": origin.get("xyz") if origin is not None else "",
            "mimic": joint.find("mimic") is not None,
        }
    return {"name": root.get("name"), "links": links, "joints": joints}


def bar(title):
    print()
    print("=" * 78)
    print(title)
    print("=" * 78)


def main():
    sdf_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SDF
    urdf_path = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_URDF
    if not os.path.isfile(sdf_path) or not os.path.isfile(urdf_path):
        print("model files not found; run from the workspace root", file=sys.stderr)
        return 1
    sdf = parse_sdf(sdf_path)
    urdf = parse_urdf(urdf_path)

    bar("SDF: %s" % sdf_path)
    print("model name :", sdf["model"])
    print("links      :", len(sdf["links"]))
    print("joints     :", len(sdf["joints"]))
    print("frames     :", len(sdf["frames"]))
    print("plugins    :", len(sdf["model_element"].findall("plugin")))
    print("sensors    :", len(list(sdf["model_element"].iter("sensor"))))
    print("transmission:", len(list(sdf["model_element"].iter("transmission"))))

    bar("URDF: %s" % urdf_path)
    revolute = [j for j in urdf["joints"].values() if j["type"] != "fixed"]
    print("robot name :", urdf["name"])
    print("links      :", len(urdf["links"]))
    print("joints     :", len(urdf["joints"]))
    print("actuated   :", len(revolute))
    print("mimic      :", [k for k, v in urdf["joints"].items() if v["mimic"]])
    print("transmission:", len(urdf["joints"]) and sum(1 for _ in ET.parse(urdf_path).getroot().findall("transmission")))

    bar("LINK SET COMPARISON")
    sdf_links, urdf_links = set(sdf["links"]), set(urdf["links"])
    print("SDF links not in URDF :", sorted(sdf_links - urdf_links))
    print("URDF links not in SDF :", sorted(urdf_links - sdf_links))

    bar("JOINT COMPARISON (actuated)")
    hdr = "%-18s %-8s %-14s %-18s %-18s %-10s %-10s %-9s %-9s" % (
        "joint", "type", "in URDF", "SDF parent/child", "URDF parent/child", "axis", "lower", "upper", "vel")
    print(hdr)
    for name, sj in sdf["joints"].items():
        uj = urdf["joints"].get(name)
        if uj is None:
            print("%-18s MISSING IN URDF" % name)
            continue
        def same(a, b):
            try:
                return abs(float(a) - float(b)) < 1e-6
            except ValueError:
                return False
        axis_ok = "OK" if same(sj["axis"].split()[0], uj["axis"].split()[0]) and \
                         same(sj["axis"].split()[1], uj["axis"].split()[1]) and \
                         same(sj["axis"].split()[2], uj["axis"].split()[2]) else "DIFF"
        print("%-18s %-8s %-14s %-18s %-18s %-10s %-10s %-9s %-9s" % (
            name, sj["type"], "yes", "%s/%s" % (sj["parent"], sj["child"]),
            "%s/%s" % (uj["parent"], uj["child"]), axis_ok,
            "OK" if same(sj["lower"], uj["lower"]) else "DIFF",
            "OK" if same(sj["upper"], uj["upper"]) else "DIFF",
            "OK" if same(sj["velocity"], uj["velocity"]) else "DIFF"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
