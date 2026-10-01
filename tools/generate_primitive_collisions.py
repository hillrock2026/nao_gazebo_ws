#!/usr/bin/env python3
"""Generate the canonical runtime SDF from the mesh reference.

Reads the mesh reference SDF (``nao_simple_raw.sdf``), then:

* replaces every ``<mesh>`` collision geometry with an axis-aligned ``<box>``
  derived from the mesh's binary/ascii STL bounding box (mesh scale applied);
* adds an explicit friction surface to every collision (including FSR boxes);
* adds two elbow sphere collisions;
* adds joint ``<damping>``/``<friction>`` dynamics;
* enables the Gazebo joint-state publisher system (Task 3B internal raw topic);
* adds one native IMU sensor on ``base_link`` (Task 3D);
* adds one native contact sensor on each foot link ``l_ankle``/``r_ankle``
  monitoring the frozen main collision ``l_ankle_collision``/``r_ankle_collision``
  (Task 3E internal raw topics).

Visuals, inertials, joints, frames and the joint topology are copied verbatim.
The STL is resolved from the merged ``nao_meshes`` share directory. Running this
on ``nao_simple_raw.sdf`` reproduces ``nao_simple.sdf`` byte-for-byte.

Usage:
    python3 tools/generate_primitive_collisions.py \
        --sdf src/nao/nao_description/urdf/nao_simple_raw.sdf \
        --out src/nao/nao_description/urdf/nao_simple.sdf \
        --mesh-root install/nao_meshes/share/nao_meshes \
        [--damping 0.2] [--joint-friction 0.1] [--collision-friction 1.0] \
        [--joint-state-topic /nao/joint_states_raw] [--joint-state-rate 100] \
        [--imu-link base_link] [--imu-topic /nao/imu_raw] [--imu-rate 100] \
        [--left-contact-topic /nao/left_foot_contact_raw] \
        [--right-contact-topic /nao/right_foot_contact_raw] [--contact-rate 100]
"""

import argparse
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_SDF = os.path.join(REPO_ROOT, "src", "nao", "nao_description", "urdf", "nao_simple.sdf")
DEFAULT_MESH_ROOT = os.path.join(REPO_ROOT, "install", "nao_meshes", "share", "nao_meshes")

URDF_VEC = r"[-+0-9.eE]+"


def stl_bbox(path):
    """Return (min xyz, max xyz) of a binary or ascii STL, or None."""
    with open(path, "rb") as f:
        data = f.read()
    if len(data) < 84:
        return None
    n = struct.unpack("<I", data[80:84])[0]
    if 84 + n * 50 == len(data) and n > 0:
        xs, ys, zs = [], [], []
        off = 84
        for _ in range(n):
            vals = struct.unpack("<12f", data[off:off + 48])
            off += 50
            for j in range(3, 12, 3):
                xs.append(vals[j])
                ys.append(vals[j + 1])
                zs.append(vals[j + 2])
        return ((min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs)))
    # ascii fallback
    xs, ys, zs = [], [], []
    for line in data.decode("utf-8", "ignore").splitlines():
        s = line.strip()
        if s.startswith("vertex"):
            p = s.split()
            if len(p) >= 4:
                try:
                    xs.append(float(p[1]))
                    ys.append(float(p[2]))
                    zs.append(float(p[3]))
                except ValueError:
                    pass
    if not xs:
        return None
    return ((min(xs), min(ys), min(zs)), (max(xs), max(ys), max(zs)))


def mesh_filename(uri):
    return uri.split("model://")[-1].split("/")[-1]


def model_summary(model):
    return {
        "links": [link.get("name") for link in model.findall("link")],
        "joints": [(j.get("name"), j.get("type")) for j in model.findall("joint")],
        "frames": [f.get("name") for f in model.findall("frame")],
    }


def replace_collision_geometry(sdf_text, mesh_root, mu):
    """Replace <mesh> collision geometry with a bbox <box>."""
    table = []

    def repl(match):
        block = match.group(0)
        mm = re.search(r"<mesh>.*?<scale>([^<]+)</scale>.*?<uri>([^<]+)</uri>",
                       block, re.S)
        if not mm:
            return block
        scale = [float(v) for v in mm.group(1).split()]
        uri = mm.group(2).strip()
        stl = os.path.join(mesh_root, "meshes", "V40", mesh_filename(uri))
        bbox = stl_bbox(stl)
        if bbox is None:
            raise RuntimeError("cannot read STL bbox for %s" % stl)
        size = [(bbox[1][i] - bbox[0][i]) * scale[i] for i in range(3)]
        center = [(bbox[1][i] + bbox[0][i]) * 0.5 * scale[i] for i in range(3)]
        name_m = re.search(r"<collision name='([^']*)'", block)
        table.append({
            "collision": name_m.group(1) if name_m else "?",
            "mesh": mesh_filename(uri),
            "scale": scale,
            "size": [round(v, 6) for v in size],
            "center": [round(v, 6) for v in center],
        })
        new_block = block
        new_block = re.sub(r"<geometry>.*?</geometry>",
                           "<geometry><box><size>%s</size></box></geometry>"
                           % " ".join("%.6f" % v for v in size),
                           new_block, count=1, flags=re.S)
        new_block = re.sub(r"<pose>[^<]*</pose>",
                           "<pose>%s 0 0 0</pose>" % " ".join("%.6f" % v for v in center),
                           new_block, count=1)
        surface = ("<surface><friction><ode><mu>%g</mu><mu2>%g</mu2></ode></friction></surface>"
                   % (mu, mu))
        new_block = new_block.replace("</geometry>", "</geometry>" + surface, 1)
        return new_block

    sdf_text = re.sub(r"<collision name='[^']*'>.*?</collision>", repl, sdf_text, flags=re.S)
    return sdf_text, table


def ensure_collision_friction(sdf_text, mu):
    """Add an explicit friction surface to any collision that lacks one."""
    surface = ("<surface><friction><ode><mu>%g</mu><mu2>%g</mu2></ode></friction></surface>"
               % (mu, mu))

    def repl(match):
        block = match.group(0)
        if "<surface>" in block:
            return block
        return block.replace("</geometry>", "</geometry>" + surface, 1)

    return re.sub(r"<collision name='[^']*'>.*?</collision>", repl, sdf_text, flags=re.S)


def add_joint_state_publisher(sdf_text, topic, update_rate):
    """Enable the Gazebo joint-state publisher as a model plugin."""
    if update_rate is None or update_rate <= 0:
        return sdf_text
    plugin = (
        "    <plugin filename=\"gz-sim-joint-state-publisher-system\" "
        "name=\"gz::sim::systems::JointStatePublisher\">\n"
        "      <topic>%s</topic>\n"
        "      <update_rate>%g</update_rate>\n"
        "    </plugin>\n" % (topic, update_rate))
    return re.sub(r"(<model name='[^']*'>\s*\n)",
                  r"\1" + plugin, sdf_text, count=1)


def add_imu_sensor(sdf_text, link, pose, topic, update_rate, name="imu_sensor"):
    """Insert one native Gazebo IMU sensor inside the given link."""
    if update_rate is None or update_rate <= 0:
        return sdf_text
    sensor = (
        "      <sensor name='%s' type='imu'>\n"
        "        <always_on>1</always_on>\n"
        "        <update_rate>%g</update_rate>\n"
        "        <topic>%s</topic>\n"
        "        <pose>%s</pose>\n"
        "        <imu>\n"
        "          <angular_velocity><x><noise type='none'/></x>"
        "<y><noise type='none'/></y><z><noise type='none'/></z></angular_velocity>\n"
        "          <linear_acceleration><x><noise type='none'/></x>"
        "<y><noise type='none'/></y><z><noise type='none'/></z></linear_acceleration>\n"
        "        </imu>\n"
        "      </sensor>\n" % (name, update_rate, topic, pose))
    i = sdf_text.index("<link name='%s'>" % link)
    j = sdf_text.index("</link>", i)
    return sdf_text[:j] + sensor + sdf_text[j:]


def add_contact_sensors(sdf_text, sensors, update_rate):
    """Insert one native Gazebo contact sensor per foot link.

    ``sensors`` is a list of ``(link, collision, name, topic)`` tuples. The
    contact sensor only reports contacts involving the named main collision of
    that link (FSR boxes are deliberately NOT monitored, per the Task 3A/3E
    contract). The topic is placed inside ``<contact>`` because that is where
    the Gazebo Contact system reads it from.
    """
    if update_rate is None or update_rate <= 0:
        return sdf_text
    for link, collision, name, topic in sensors:
        marker = "<link name='%s'>" % link
        i = sdf_text.index(marker)
        j = sdf_text.index("</link>", i)
        sensor = (
            "      <sensor name='%s' type='contact'>\n"
            "        <always_on>1</always_on>\n"
            "        <update_rate>%g</update_rate>\n"
            "        <contact>\n"
            "          <collision>%s</collision>\n"
            "          <topic>%s</topic>\n"
            "        </contact>\n"
            "      </sensor>\n" % (name, update_rate, collision, topic))
        sdf_text = sdf_text[:j] + sensor + sdf_text[j:]
    return sdf_text


def add_joint_dynamics(sdf_text, damping, friction):
    if damping is None and friction is None:
        return sdf_text
    extra = ""
    if damping is not None:
        extra += "\n          <damping>%g</damping>" % damping
    if friction is not None:
        extra += "\n          <friction>%g</friction>" % friction
    return sdf_text.replace(
        "<spring_stiffness>0</spring_stiffness>",
        "<spring_stiffness>0</spring_stiffness>" + extra)


def add_elbow_collisions(sdf_text, radius, mu):
    surface = ("<surface><friction><ode><mu>%g</mu><mu2>%g</mu2></ode></friction></surface>"
               % (mu, mu))
    for link in ("LElbow", "RElbow"):
        marker = "<link name='%s'>" % link
        idx = sdf_text.find(marker)
        if idx < 0:
            raise RuntimeError("link %s not found" % link)
        end = sdf_text.find("</link>", idx)
        col = ("\n        <collision name='%s_collision'>"
               "\n          <pose>0 0 0 0 0 0</pose>"
               "\n          <geometry><sphere><radius>%g</radius></sphere></geometry>"
               "\n          %s\n        </collision>\n      " % (link, radius, surface))
        sdf_text = sdf_text[:end] + col + sdf_text[end:]
    return sdf_text


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sdf", default=DEFAULT_SDF)
    ap.add_argument("--out", required=True)
    ap.add_argument("--mesh-root", default=DEFAULT_MESH_ROOT)
    ap.add_argument("--damping", type=float, default=0.2)
    ap.add_argument("--joint-friction", type=float, default=0.1)
    ap.add_argument("--collision-friction", type=float, default=1.0)
    ap.add_argument("--elbow-radius", type=float, default=0.025)
    ap.add_argument("--joint-state-topic", default="/nao/joint_states_raw")
    ap.add_argument("--joint-state-rate", type=float, default=100.0)
    ap.add_argument("--imu-link", default="base_link")
    ap.add_argument("--imu-pose", default="-0.008 0.00606 0.027 0 0 0")
    ap.add_argument("--imu-topic", default="/nao/imu_raw")
    ap.add_argument("--imu-rate", type=float, default=100.0)
    ap.add_argument("--left-contact-topic", default="/nao/left_foot_contact_raw")
    ap.add_argument("--right-contact-topic", default="/nao/right_foot_contact_raw")
    ap.add_argument("--contact-rate", type=float, default=100.0)
    ap.add_argument("--table", default="")
    args = ap.parse_args()

    text = open(args.sdf).read()
    text, table = replace_collision_geometry(text, args.mesh_root, args.collision_friction)
    text = ensure_collision_friction(text, args.collision_friction)
    text = add_elbow_collisions(text, args.elbow_radius, args.collision_friction)
    text = add_joint_dynamics(text, args.damping, args.joint_friction)
    text = add_joint_state_publisher(text, args.joint_state_topic, args.joint_state_rate)
    text = add_imu_sensor(text, args.imu_link, args.imu_pose, args.imu_topic, args.imu_rate)
    text = add_contact_sensors(
        text,
        [("l_ankle", "l_ankle_collision", "left_foot_contact", args.left_contact_topic),
         ("r_ankle", "r_ankle_collision", "right_foot_contact", args.right_contact_topic)],
        args.contact_rate)
    with open(args.out, "w") as f:
        f.write(text)

    if args.table:
        import json
        json.dump(table, open(args.table, "w"), indent=2)

    # Self-check: the output must be valid XML and still declare the same links.
    ET.parse(args.out)
    old = ET.parse(args.sdf).getroot().find("model")
    new = ET.parse(args.out).getroot().find("model")
    assert model_summary(old) == model_summary(new), "links/joints/frames changed!"
    print("replaced %d mesh collisions; wrote %s" % (len(table), args.out))
    print("links/joints/frames unchanged: OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
