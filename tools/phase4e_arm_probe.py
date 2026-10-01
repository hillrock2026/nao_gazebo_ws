#!/usr/bin/env python3
"""Read-only arm-pose measurement probe for Task 4 Phase 4E.

Subscribes to ``/joint_states`` and looks up ``base_link -> l_wrist`` via TF,
printing the observed shoulder/elbow joint values and the left wrist position
(relative to base_link) once per second. Publishes nothing; changes no state.

Usage: python3 tools/phase4e_arm_probe.py [seconds]
"""

import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import tf2_ros

JOINT_ORDER = [
    'HeadYaw', 'HeadPitch',
    'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll',
    'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw',
    'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll',
    'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw',
]


class ArmProbe(Node):
    def __init__(self):
        super().__init__('phase4e_arm_probe')
        self.latest = None
        self.create_subscription(JointState, '/joint_states', self.on_state, 10)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

    def on_state(self, msg):
        self.latest = list(msg.position)


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 8.0
    rclpy.init()
    node = ArmProbe()
    deadline = time.time() + seconds
    last = 0.0
    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.latest is None or len(node.latest) != 24:
                continue
            if time.time() - last < 1.0:
                continue
            last = time.time()
            p = node.latest
            try:
                tr = node.buffer.lookup_transform('base_link', 'l_wrist',
                                                  rclpy.time.Time())
                t = tr.transform.translation
                hand = '(%.4f, %.4f, %.4f)' % (t.x, t.y, t.z)
            except Exception as exc:  # noqa: BLE001
                hand = 'TF unavailable: %s' % type(exc).__name__
            print('LShoulderPitch=%+.4f LShoulderRoll=%+.4f LElbowRoll=%+.4f '
                  'RShoulderPitch=%+.4f RElbowRoll=%+.4f  l_wrist(base_link)=%s'
                  % (p[8], p[9], p[11], p[19], p[22], hand), flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
