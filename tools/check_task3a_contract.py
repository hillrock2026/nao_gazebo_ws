#!/usr/bin/env python3
"""Task 3A contract-vs-model consistency check (read-only, no bridge).

Verifies that the frozen interface contract in
``docs/Task3A_Interface_and_Naming_Contract.md`` is still consistent with the
canonical model files:

* the 24 canonical joint names, in the exact canonical SDF document order;
* the SDF and URDF agree on the joint name set;
* the frozen body / foot / IMU frame names exist and are wired as documented;
* left and right foot links/frames are distinct and not swapped;
* the canonical world and model currently declare no sensor plugin (documented
  Task 3B prerequisite), and no controller/bridge is present.

Exit code 0 = all checks pass, 1 = at least one check failed.
"""

import os
import sys
import xml.etree.ElementTree as ET

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SDF = os.path.join(REPO_ROOT, "src", "nao", "nao_description", "urdf", "nao_simple.sdf")
URDF = os.path.join(REPO_ROOT, "src", "nao", "nao_description", "urdf", "nao_simple.urdf")
WORLD = os.path.join(REPO_ROOT, "src", "nao_gz_sim", "worlds", "nao_world.sdf")

# Frozen canonical joint order = SDF document order (see contract section 7).
EXPECTED_JOINT_ORDER = [
    "HeadYaw", "HeadPitch",
    "LHipYawPitch", "LHipRoll", "LHipPitch", "LKneePitch", "LAnklePitch", "LAnkleRoll",
    "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw",
    "RHipYawPitch", "RHipRoll", "RHipPitch", "RKneePitch", "RAnklePitch", "RAnkleRoll",
    "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw",
]
LEFT_FOOT_LINK, RIGHT_FOOT_LINK = "l_ankle", "r_ankle"
LEFT_FOOT_FRAME, RIGHT_FOOT_FRAME = "l_sole", "r_sole"
SOLE_OFFSET_Z = -0.04511
IMU_FRAME = "ImuTorsoAccelerometer_frame"
IMU_ALT_FRAME = "ImuTorsoGyrometer_frame"
BODY_FRAME = "base_link"
BODY_ALIAS = "torso"

failures = []
warnings = []


def check(cond, msg):
    if cond:
        print("  [PASS] %s" % msg)
    else:
        print("  [FAIL] %s" % msg)
        failures.append(msg)


def warn(cond, msg):
    if not cond:
        print("  [WARN] %s" % msg)
        warnings.append(msg)
    else:
        print("  [PASS] %s" % msg)


def main():
    sdf_model = ET.parse(SDF).getroot().find("model")
    urdf = ET.parse(URDF).getroot()

    sdf_joints = sdf_model.findall("joint")
    sdf_joint_order = [j.get("name") for j in sdf_joints]
    sdf_links = {link_el.get("name") for link_el in sdf_model.findall("link")}
    sdf_frames = {f.get("name"): f for f in sdf_model.findall("frame")}
    urdf_links = {link_el.get("name") for link_el in urdf.findall("link")}
    urdf_joints = urdf.findall("joint")
    urdf_revolute = [j.get("name") for j in urdf_joints if j.get("type") == "revolute"]

    print("== canonical joint order (SDF document order) ==")
    check(sdf_joint_order == EXPECTED_JOINT_ORDER,
          "SDF joint order equals frozen contract order (%d joints)" % len(EXPECTED_JOINT_ORDER))
    check(set(sdf_joint_order) == set(urdf_revolute),
          "SDF and URDF have the same 24 revolute joint names")
    check(all(j.get("type") == "revolute" for j in sdf_joints),
          "all canonical SDF joints are revolute")

    print("== body / foot / IMU frames ==")
    check(BODY_FRAME in sdf_links, "SDF has body link '%s'" % BODY_FRAME)
    check(BODY_FRAME in urdf_links and BODY_ALIAS in urdf_links,
          "URDF has '%s' and '%s'" % (BODY_FRAME, BODY_ALIAS))
    bfj = [j for j in urdf_joints if j.get("name") == "base_link_fixedjoint"]
    check(bool(bfj) and bfj[0].find("origin").get("xyz") in ("0 0 0", "0.0 0.0 0.0"),
          "URDF base_link -> torso is identity (base_link == torso)")
    for link, frame in ((LEFT_FOOT_LINK, LEFT_FOOT_FRAME), (RIGHT_FOOT_LINK, RIGHT_FOOT_FRAME)):
        check(link in sdf_links and link in urdf_links, "link '%s' exists in SDF and URDF" % link)
        check(frame in sdf_frames and frame in urdf_links,
              "frame '%s' exists in SDF and as a URDF link" % frame)
        fixed = [j for j in urdf_joints if j.find("child").get("link") == frame]
        check(bool(fixed) and fixed[0].get("type") == "fixed"
              and fixed[0].find("parent").get("link") == link,
              "URDF fixed joint %s -> %s" % (link, frame))
    check(IMU_FRAME in sdf_frames and IMU_FRAME in urdf_links,
          "frozen IMU frame '%s' exists in SDF frame and URDF link" % IMU_FRAME)
    check(IMU_ALT_FRAME in sdf_frames and IMU_ALT_FRAME in urdf_links,
          "alternative IMU frame '%s' exists (documented, not used as frame_id)" % IMU_ALT_FRAME)

    print("== left/right foot mapping is not swapped ==")

    def sdf_frame_root(frame_name):
        """Follow SDF frame attached_to links until a real link is reached."""
        seen = set()
        cur = frame_name
        while cur in sdf_frames and cur not in seen:
            seen.add(cur)
            cur = sdf_frames[cur].get("attached_to")
        return cur

    check(sdf_frame_root(LEFT_FOOT_FRAME) == LEFT_FOOT_LINK,
          "SDF %s attaches (transitively) to %s" % (LEFT_FOOT_FRAME, LEFT_FOOT_LINK))
    check(sdf_frame_root(RIGHT_FOOT_FRAME) == RIGHT_FOOT_LINK,
          "SDF %s attaches (transitively) to %s" % (RIGHT_FOOT_FRAME, RIGHT_FOOT_LINK))
    # A robust non-swap check: the 'l_'/'r_' prefixes must stay on the same side.
    check(LEFT_FOOT_LINK.startswith("l_") and RIGHT_FOOT_LINK.startswith("r_")
          and LEFT_FOOT_FRAME.startswith("l_") and RIGHT_FOOT_FRAME.startswith("r_")
          and LEFT_FOOT_LINK != RIGHT_FOOT_LINK and LEFT_FOOT_FRAME != RIGHT_FOOT_FRAME,
          "left uses l_ankle/l_sole, right uses r_ankle/r_sole (distinct)")
    # The sole frames must be at the documented offset from the ankle.
    for frame, link in ((LEFT_FOOT_FRAME, LEFT_FOOT_LINK), (RIGHT_FOOT_FRAME, RIGHT_FOOT_LINK)):
        fixed = [j for j in urdf_joints if j.find("child").get("link") == frame][0]
        z = float(fixed.find("origin").get("xyz").split()[2])
        check(abs(z - SOLE_OFFSET_Z) < 1e-6,
              "%s offset z=%.5f from %s" % (frame, z, link))

    print("== sensor / controller / bridge declarations ==")
    sdf_sensors = list(sdf_model.iter("sensor"))
    imu_sensors = [s for s in sdf_sensors if s.get("type") == "imu"]
    contact_sensors = [s for s in sdf_sensors if s.get("type") == "contact"]
    check(len(imu_sensors) == 1 and len(sdf_sensors) == 3,
          "canonical SDF declares the single Task 3D IMU sensor and the two "
          "Task 3E foot contact sensors: %s"
          % [(s.get("name"), s.get("type")) for s in sdf_sensors])
    foot_contact = {}
    for link_el in sdf_model.findall("link"):
        for s in link_el.findall("sensor"):
            if s.get("type") == "contact":
                col = s.find("contact/collision")
                topic = s.find("contact/topic")
                foot_contact[link_el.get("name")] = (
                    col.text if col is not None else None,
                    topic.text if topic is not None else None)
    check(foot_contact.get("l_ankle", (None,))[0] == "l_ankle_collision",
          "Task 3E left contact sensor on l_ankle monitors l_ankle_collision: %s"
          % (foot_contact.get("l_ankle"),))
    check(foot_contact.get("r_ankle", (None,))[0] == "r_ankle_collision",
          "Task 3E right contact sensor on r_ankle monitors r_ankle_collision: %s"
          % (foot_contact.get("r_ankle"),))
    check(len(foot_contact) == 2,
          "Task 3E contact sensors are exactly on l_ankle and r_ankle")
    sdf_plugins = sdf_model.findall("plugin")
    sdf_plugin_names = [p.get("name", "") for p in sdf_plugins]
    check(len(sdf_plugin_names) >= 1
          and all("JointStatePublisher" in n for n in sdf_plugin_names),
          "canonical SDF model plugins are only JointStatePublisher (Task 3B): %s"
          % sdf_plugin_names)
    check(not any("Controller" in n or "ros2_control" in n for n in sdf_plugin_names),
          "canonical SDF declares no controller plugin")
    world = ET.parse(WORLD).getroot().find("world")
    world_plugins = [p.get("name", "") for p in world.findall("plugin")]
    check(all("Controller" not in n and "ros2_control" not in n for n in world_plugins),
          "world declares no controller plugin")
    check(any("Imu" in n for n in world_plugins),
          "world declares the Task 3D Imu system: %s" % world_plugins)
    check(any(n.endswith("systems::Contact") for n in world_plugins),
          "world declares the Task 3E Contact system: %s" % world_plugins)
    check(not any("Sensors" in n for n in world_plugins),
          "world declares no other Sensors system")

    print()
    if failures:
        print("CONTRACT CHECK: FAIL (%d failures)" % len(failures))
        return 1
    print("CONTRACT CHECK: PASS (warnings: %d)" % len(warnings))
    return 0


if __name__ == "__main__":
    sys.exit(main())
