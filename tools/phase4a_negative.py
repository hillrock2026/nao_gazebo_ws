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
Task 4 Phase 4A negative test publisher.

Runs against a live simulator started with ``enable_phase4a:=true
phase4a_run_probe:=false`` (actuator loaded, no built-in probe). It publishes a
sequence of deliberately illegal commands on /joint_command, then one valid
command. Expected behaviour:

  * every illegal command is rejected by the actuator (recorded in its log, and
    by the phase4a_trace as ``command_rejected``);
  * the tested joint never moves toward the out-of-limit target;
  * the final valid command is still executed, proving the actuator survived the
    invalid input and never silently corrected it.

This is a test-only tool; it is not a command producer.
"""

import math
import sys

import rclpy
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

HOME = [0.0] * 24
HOME[JOINT_ORDER.index('LElbowRoll')] = -0.0349066
HOME[JOINT_ORDER.index('RElbowRoll')] = 0.0349066


def qos_profile():
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
    )


def home_with(joint_name, value):
    positions = list(HOME)
    positions[JOINT_ORDER.index(joint_name)] = value
    return positions


class NegativeTest(Node):
    def __init__(self):
        super().__init__('phase4a_negative_test')
        self.command_topic = self.declare_parameter(
            'command_topic', '/joint_command').value
        self.state_topic = self.declare_parameter(
            'state_topic', '/joint_states').value
        self.joint_name = self.declare_parameter('joint_name', 'HeadYaw').value
        self.target = float(self.declare_parameter('valid_target_rad', 0.4).value)

        self.pub = self.create_publisher(
            JointState, self.command_topic, qos_profile())
        self.create_subscription(
            JointState, self.state_topic, self.on_state, qos_profile())

        self.joint_index = JOINT_ORDER.index(self.joint_name)
        self.have_state = False
        self.last_action_time = None
        self.observed = []
        self.valid_stamp = None
        self.b_wait_start = None
        self.reasons = []
        self.step = 0
        self.finished = False
        self.timer = self.create_timer(0.05, self.on_timer)

    def on_state(self, msg):
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return
        if len(msg.position) == 24:
            self.have_state = True
            self.observed.append(float(msg.position[self.joint_index]))

    def make_msg(self, stamp, positions, names=None, velocity=None, effort=None,
                 frame_id=''):
        msg = JointState()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.name = list(names) if names is not None else list(JOINT_ORDER)
        msg.position = list(positions)
        msg.velocity = list(velocity) if velocity is not None else []
        msg.effort = list(effort) if effort is not None else []
        return msg

    def publish(self, msg, reason):
        self.pub.publish(msg)
        self.reasons.append(reason)
        self.get_logger().info('sent invalid command: %s' % reason)

    def on_timer(self):
        if self.finished or not self.have_state:
            return
        # Wait until the actuator subscription is actually matched, otherwise
        # the first commands are lost to DDS discovery and never reach it.
        if self.pub.get_subscription_count() < 1:
            self.last_action_time = None
            return
        if self.last_action_time is None:
            self.last_action_time = self.get_clock().now()
            return
        now = self.get_clock().now()
        if (now - self.last_action_time).nanoseconds * 1e-9 < 0.2:
            return
        if now.nanoseconds == 0:
            return
        self.run_step(now)

    def run_step(self, now):
        stamp = now.to_msg()
        step = self.step
        self.step += 1
        self.last_action_time = now

        if step == 0:
            # 1. wrong dimension (23).
            self.publish(
                self.make_msg(stamp, HOME[:23], names=JOINT_ORDER[:23]), 'dim23')
        elif step == 1:
            # 2. non-empty velocity.
            self.publish(
                self.make_msg(stamp, HOME, velocity=[0.0]), 'velocity_not_empty')
        elif step == 2:
            # 3. non-empty effort.
            self.publish(
                self.make_msg(stamp, HOME, effort=[0.0]), 'effort_not_empty')
        elif step == 3:
            # 4. non-empty frame_id.
            self.publish(
                self.make_msg(stamp, HOME, frame_id='world'), 'frame_not_empty')
        elif step == 4:
            # 5. zero timestamp.
            from builtin_interfaces.msg import Time
            self.publish(self.make_msg(Time(), HOME), 'zero_stamp')
        elif step == 5:
            # 6. out-of-limit target (HeadYaw limit is +-2.08567).
            self.publish(
                self.make_msg(stamp, home_with(self.joint_name, 5.0)),
                'out_of_limit')
        elif step == 6:
            # 7. valid command A (also establishes the last accepted stamp).
            self.valid_stamp = stamp
            self.pub.publish(self.make_msg(stamp, home_with(self.joint_name, 0.2)))
            self.get_logger().info('sent VALID command A: %s=0.2' % self.joint_name)
        elif step == 7:
            # 8. duplicate of A (same stamp).
            self.publish(
                self.make_msg(self.valid_stamp, home_with(self.joint_name, 0.2)),
                'duplicate_stamp')
        elif step == 8:
            # 9. backward stamp.
            back = self.valid_stamp
            back.nanosec -= 100000000
            if back.nanosec < 0:
                back.nanosec += 1000000000
                back.sec -= 1
            self.publish(
                self.make_msg(back, home_with(self.joint_name, 0.2)), 'backward_stamp')
        elif step == 9:
            # 10. final valid command B.
            self.pub.publish(
                self.make_msg(stamp, home_with(self.joint_name, self.target)))
            self.b_wait_start = now
            self.get_logger().info(
                'sent VALID command B: %s=%.3f' % (self.joint_name, self.target))
        elif step >= 10:
            reached = self.observed and self.observed[-1] > self.target - 0.05
            timed_out = (now - self.b_wait_start).nanoseconds * 1e-9 > 6.0
            if reached or timed_out:
                self.finish()

    def finish(self):
        self.finished = True
        if not self.observed:
            self.get_logger().error('no state observed')
            self.result = False
        else:
            max_observed = max(self.observed)
            final = self.observed[-1]
            out_of_limit_ignored = max_observed < 0.5
            valid_executed = final > self.target - 0.05
            self.result = bool(out_of_limit_ignored and valid_executed)
            print('=== PHASE4A NEGATIVE TEST RESULT ===')
            print('invalid_commands_sent : %d' % len(self.reasons))
            print('reasons               : %s' % ','.join(self.reasons))
            print('max_observed_position : %.6f (must stay well below 5.0)' % max_observed)
            print('final_position        : %.6f (target %.3f)' % (final, self.target))
            print('out_of_limit_ignored  : %s' % out_of_limit_ignored)
            print('valid_executed        : %s' % valid_executed)
            print('NEGATIVE TEST: %s' % ('PASS' if self.result else 'FAIL'))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = NegativeTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
