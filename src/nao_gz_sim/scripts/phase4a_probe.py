#!/usr/bin/env python3
# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
Task 4 Phase 4A probe: a finite, explicitly-launched test command publisher.

This is NOT a walking command producer and NOT the Phase 4B producer. It exists
only to exercise the Phase 4A execution path:

    valid 24-dim /joint_command -> actuator -> Gazebo joint -> /joint_states

It is started only when the launch argument ``enable_phase4a:=true`` is set. It
subscribes to /joint_states to learn the initial state, waits for a valid Gazebo
simulation time, then publishes a finite number of complete 24-dimensional
position targets on /joint_command and exits. It never publishes in an endless
background loop.

Each message is a complete frozen-order 24-vector:

  * names/order     = the frozen Task 3A / schema order (never a single joint);
  * positions       = documented safe home (clamp(0, lower, upper)) for every
                      joint except the tested joint, which is commanded to the
                      requested safe target;
  * velocity/effort = empty; frame_id = ''; stamp = Gazebo simulation time.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

JOINT_ORDER = [
    'HeadYaw', 'HeadPitch',
    'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll',
    'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw',
    'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll',
    'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw',
]

# Frozen joint limits from the canonical SDF (docs/task4_phase3_evidence/joint_limits.json).
LIMITS = {
    'HeadYaw': (-2.08567, 2.08567),
    'HeadPitch': (-0.671952, 0.514872),
    'LHipYawPitch': (-1.14529, 0.740718),
    'LHipRoll': (-0.379435, 0.79046),
    'LHipPitch': (-1.53589, 0.48398),
    'LKneePitch': (-0.0923279, 2.11255),
    'LAnklePitch': (-1.18944, 0.922581),
    'LAnkleRoll': (-0.397761, 0.768992),
    'LShoulderPitch': (-2.08567, 2.08567),
    'LShoulderRoll': (-0.314159, 1.32645),
    'LElbowYaw': (-2.08567, 2.08567),
    'LElbowRoll': (-1.54462, -0.0349066),
    'LWristYaw': (-1.82387, 1.82387),
    'RHipYawPitch': (-1.14529, 0.740718),
    'RHipRoll': (-0.79046, 0.379435),
    'RHipPitch': (-1.53589, 0.48398),
    'RKneePitch': (-0.0923279, 2.11255),
    'RAnklePitch': (-1.1863, 0.932006),
    'RAnkleRoll': (-0.768992, 0.397761),
    'RShoulderPitch': (-2.08567, 2.08567),
    'RShoulderRoll': (-1.32645, 0.314159),
    'RElbowYaw': (-2.08567, 2.08567),
    'RElbowRoll': (0.0349066, 1.54462),
    'RWristYaw': (-1.82387, 1.82387),
}


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


# Documented safe home: nearest in-limit value to 0 (all joints are 0 except the
# elbow rolls whose 0 rad is outside the frozen limit).
HOME = [clamp(0.0, *LIMITS[name]) for name in JOINT_ORDER]


def qos_profile():
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


class Phase4aProbe(Node):
    def __init__(self):
        super().__init__('phase4a_probe')
        self.declare_parameter('command_topic', '/joint_command')
        self.declare_parameter('state_topic', '/joint_states')
        self.declare_parameter('joint_name', 'HeadYaw')
        self.declare_parameter('target_position_rad', 0.3)
        self.declare_parameter('publish_count', 10)
        self.declare_parameter('publish_period_sec', 0.2)
        self.declare_parameter('start_delay_sec', 2.0)
        self.declare_parameter('settle_sec', 3.0)

        self.command_topic = self.get_parameter('command_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.joint_name = self.get_parameter('joint_name').value
        self.target = float(self.get_parameter('target_position_rad').value)
        self.publish_count = int(self.get_parameter('publish_count').value)
        self.period = float(self.get_parameter('publish_period_sec').value)
        self.start_delay = float(self.get_parameter('start_delay_sec').value)
        self.settle = float(self.get_parameter('settle_sec').value)

        if self.joint_name not in JOINT_ORDER:
            raise ValueError('unknown joint_name %r' % self.joint_name)
        if not math.isfinite(self.target) or not (
            LIMITS[self.joint_name][0] <= self.target <= LIMITS[self.joint_name][1]
        ):
            raise ValueError(
                'target_position_rad=%r is outside frozen limit %r for %s'
                % (self.target, LIMITS[self.joint_name], self.joint_name)
            )

        self.joint_index = JOINT_ORDER.index(self.joint_name)
        self.positions_template = list(HOME)
        self.positions_template[self.joint_index] = self.target

        self.pub = self.create_publisher(
            JointState, self.command_topic, qos_profile())
        self.sub = self.create_subscription(
            JointState, self.state_topic, self.on_state, qos_profile())

        self.initial_position = None
        self.first_state_time = None
        self.last_publish_time = None
        self.published = 0
        self.finished = False
        self.timer = self.create_timer(0.05, self.on_timer)

        self.get_logger().info(
            'phase4a_probe: joint=%s target=%.4f rad count=%d period=%.2fs '
            'command_topic=%s' % (
                self.joint_name, self.target, self.publish_count, self.period,
                self.command_topic))

    def on_state(self, msg):
        if self.initial_position is not None:
            return
        # Wait for a valid Gazebo simulation timestamp.
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return
        if list(msg.name) != JOINT_ORDER:
            self.get_logger().error('state joint order mismatch; probe idle')
            return
        self.initial_position = list(msg.position)
        self.first_state_time = self.get_clock().now()
        self.get_logger().info(
            'initial %s position = %.5f rad (sim stamp %d.%09u)'
            % (self.joint_name, self.initial_position[self.joint_index],
               msg.header.stamp.sec, msg.header.stamp.nanosec))

    def on_timer(self):
        if self.finished or self.initial_position is None:
            return
        now = self.get_clock().now()
        if (now - self.first_state_time).nanoseconds * 1e-9 < self.start_delay:
            return
        if self.published >= self.publish_count:
            if (now - self.last_publish_time).nanoseconds * 1e-9 >= self.settle:
                self.finished = True
                self.get_logger().info(
                    'phase4a_probe: published %d finite commands; exiting'
                    % self.published)
                rclpy.shutdown()
            return
        if self.last_publish_time is not None:
            elapsed = (now - self.last_publish_time).nanoseconds * 1e-9
            if elapsed < self.period:
                return

        stamp = now.to_msg()
        if stamp.sec == 0 and stamp.nanosec == 0:
            return
        # Never publish a non-increasing timestamp.
        if self.last_publish_time is not None and now <= self.last_publish_time:
            return

        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = ''
        msg.name = list(JOINT_ORDER)
        msg.position = list(self.positions_template)
        msg.velocity = []
        msg.effort = []
        self.pub.publish(msg)
        self.published += 1
        self.last_publish_time = now
        self.get_logger().info(
            'published command %d/%d stamp=%d.%09u %s=%.4f'
            % (self.published, self.publish_count, stamp.sec, stamp.nanosec,
               self.joint_name, self.target))


def main(args=None):
    rclpy.init(args=args)
    node = Phase4aProbe()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # rclpy shutdown race (e.g. RCLError while spinning)
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
