#!/usr/bin/env python3
"""Read-only wall-time / simulation-time logger for the Phase 4E-Visual demo.

Subscribes to ``/clock`` and appends ``{"wall": <unix s>, "sim": <sim s>}`` JSONL
rows (flushed immediately) so that GUI screenshots/video (timestamped in wall
time) can be aligned with simulation time, and the pause/resume behaviour can be
proven (wall advances while ``sim`` is frozen).

It publishes nothing and never touches the model. Usage::

    python3 tools/phase4e_visual_clock_logger.py --out /path/clock_wall.jsonl
"""

import argparse
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


class ClockLogger(Node):
    def __init__(self, out_path):
        super().__init__('phase4e_visual_clock_logger')
        self.out_path = out_path
        self.fh = open(out_path, 'w', encoding='utf-8')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Clock, '/clock', self.on_clock, qos)

    def on_clock(self, msg):
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.fh.write(json.dumps({'wall': time.time(), 'sim': sim}) + '\n')
        self.fh.flush()

    def close(self):
        try:
            self.fh.close()
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    rclpy.init()
    node = ClockLogger(args.out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
