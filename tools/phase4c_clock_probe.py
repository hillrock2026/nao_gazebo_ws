#!/usr/bin/env python3
"""Task 4 Phase 4C minimal read-only /clock probe.

Subscribes to the frozen ``/clock`` topic for a bounded wall duration and prints
the first simulation timestamp observed (or ``None`` if no message arrived). It
publishes nothing and is used only to demonstrate simulation-time freeze during
a Gazebo pause.

Usage: python3 tools/phase4c_clock_probe.py <wall_seconds>
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    rclpy.init()
    node = Node('phase4c_clock_probe')
    state = {'t': None}

    def on_clock(msg):
        state['t'] = msg.clock.sec + msg.clock.nanosec * 1e-9

    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=1)
    node.create_subscription(Clock, '/clock', on_clock, qos)
    end = time.time() + seconds
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    print(state['t'])
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
