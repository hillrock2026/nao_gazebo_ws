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
Task 4 Phase 4A closed-loop trace recorder and summariser.

Read-only observer. It subscribes to /clock, /joint_command and /joint_states,
writes the raw samples to a JSONL trace and computes the Phase 4A response
summary (initial / target / min / max / final position and the absolute
response delta of the tested joint), plus command-contract counters.

It publishes nothing and never touches the simulation.
"""

import json
import math
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

import rosgraph_msgs.msg
from sensor_msgs.msg import JointState

JOINT_ORDER = [
    'HeadYaw', 'HeadPitch',
    'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll',
    'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw',
    'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll',
    'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw',
]

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


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def qos_profile():
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=200,
    )


def clock_qos_profile():
    # /clock is published BEST_EFFORT/VOLATILE/KEEP_LAST(1) (Task 3A section 9).
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )


class Phase4aTrace(Node):
    def __init__(self):
        super().__init__('phase4a_trace')
        self.declare_parameter('clock_topic', '/clock')
        self.declare_parameter('command_topic', '/joint_command')
        self.declare_parameter('state_topic', '/joint_states')
        self.declare_parameter('target_joint', 'HeadYaw')
        self.declare_parameter('output_jsonl', '')
        self.declare_parameter('output_summary', '')
        self.declare_parameter('record_duration_sec', 0.0)
        self.declare_parameter('clock_log_period_sec', 0.05)

        self.clock_topic = self.get_parameter('clock_topic').value
        self.command_topic = self.get_parameter('command_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.target_joint = self.get_parameter('target_joint').value
        self.output_jsonl = self.get_parameter('output_jsonl').value
        self.output_summary = self.get_parameter('output_summary').value
        self.record_duration = float(self.get_parameter('record_duration_sec').value)
        self.clock_log_period = float(self.get_parameter('clock_log_period_sec').value)

        if self.target_joint not in JOINT_ORDER:
            raise ValueError('unknown target_joint %r' % self.target_joint)
        self.target_index = JOINT_ORDER.index(self.target_joint)

        self._jsonl = None
        if self.output_jsonl:
            os.makedirs(os.path.dirname(self.output_jsonl), exist_ok=True)
            self._jsonl = open(self.output_jsonl, 'w')

        self.latest_clock = None
        self.last_clock_logged = None
        self.first_state_stamp = None

        self.state_count = 0
        self.command_count = 0
        self.state_positions = []
        self.state_stamps = []
        self.command_targets = []
        self.command_stamps = []

        self.command_reject_counts = {}
        self.last_command_stamp = None
        self.nonfinite_state = 0

        self._write({
            'kind': 'meta', 'joint_order': JOINT_ORDER,
            'target_joint': self.target_joint,
            'command_topic': self.command_topic,
            'state_topic': self.state_topic})

        self.create_subscription(
            rosgraph_msgs.msg.Clock, self.clock_topic, self.on_clock, clock_qos_profile())
        self.create_subscription(
            JointState, self.command_topic, self.on_command, qos_profile())
        self.create_subscription(
            JointState, self.state_topic, self.on_state, qos_profile())

        self.get_logger().info(
            'phase4a_trace: clock=%s command=%s state=%s target_joint=%s '
            'duration=%.1fs jsonl=%s summary=%s' % (
                self.clock_topic, self.command_topic, self.state_topic,
                self.target_joint, self.record_duration,
                self.output_jsonl, self.output_summary))

    def _write(self, record):
        if self._jsonl is not None:
            self._jsonl.write(json.dumps(record, separators=(',', ':')) + '\n')

    def on_clock(self, msg):
        t = stamp_sec(msg.clock)
        self.latest_clock = t
        if (self.last_clock_logged is None
                or t - self.last_clock_logged >= self.clock_log_period):
            self.last_clock_logged = t
            self._write({'kind': 'clock', 't': t})

    def _count_command_reject(self, reason):
        self.command_reject_counts[reason] = \
            self.command_reject_counts.get(reason, 0) + 1

    def on_command(self, msg):
        self.command_count += 1
        reason = None
        name = list(msg.name)
        position = list(msg.position)
        if len(name) != 24 or len(position) != 24:
            reason = 'command_dim_mismatch'
        elif name != JOINT_ORDER:
            reason = 'command_joint_order_mismatch'
        elif len(msg.velocity) != 0:
            reason = 'command_velocity_not_empty'
        elif len(msg.effort) != 0:
            reason = 'command_effort_not_empty'
        elif msg.header.frame_id != '':
            reason = 'command_frame_mismatch'
        elif msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            reason = 'command_zero_timestamp'
        else:
            for value in position:
                if not math.isfinite(value):
                    reason = 'command_non_finite'
                    break
            if reason is None:
                for jname, value in zip(JOINT_ORDER, position):
                    lo, hi = LIMITS[jname]
                    if value < lo - 1e-9 or value > hi + 1e-9:
                        reason = 'command_out_of_limit'
                        break
            if reason is None and self.last_command_stamp is not None:
                if stamp_sec(msg.header.stamp) == self.last_command_stamp:
                    reason = 'command_duplicate_timestamp'
                elif stamp_sec(msg.header.stamp) < self.last_command_stamp:
                    reason = 'command_backward_timestamp'

        if reason is not None:
            self._count_command_reject(reason)
            self._write({
                'kind': 'command_rejected', 'reason': reason,
                't': stamp_sec(msg.header.stamp)})
            return

        self.last_command_stamp = stamp_sec(msg.header.stamp)
        self.command_targets.append(position)
        self.command_stamps.append(self.last_command_stamp)
        self._write({
            'kind': 'command', 't': self.last_command_stamp,
            'clock_at_recv': self.latest_clock,
            'position': position})

    def on_state(self, msg):
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return
        t = stamp_sec(msg.header.stamp)
        if self.first_state_stamp is None:
            self.first_state_stamp = t
        self.state_count += 1
        positions = [float(v) for v in msg.position]
        if len(positions) == 24:
            self.state_positions.append(positions)
        if any(not math.isfinite(v) for v in positions):
            self.nonfinite_state += 1
        self.state_stamps.append(t)
        self._write({'kind': 'state', 't': t, 'position': positions})

        if self.record_duration > 0:
            elapsed = t - self.first_state_stamp
            if elapsed >= self.record_duration:
                self.get_logger().info(
                    'phase4a_trace: record duration reached (%.2f sim s); finalising'
                    % elapsed)
                rclpy.shutdown()

    def finalize(self):
        summary = {
            'target_joint': self.target_joint,
            'state_count': self.state_count,
            'command_count': self.command_count,
            'command_reject_counts': self.command_reject_counts,
            'nonfinite_state_samples': self.nonfinite_state,
        }
        if self.command_stamps:
            summary['command_stamp_min'] = min(self.command_stamps)
            summary['command_stamp_max'] = max(self.command_stamps)
            summary['command_stamps_strictly_increasing'] = all(
                b > a for a, b in zip(self.command_stamps, self.command_stamps[1:]))
            summary['command_dim_ok'] = all(
                len(c) == 24 for c in self.command_targets)
            summary['command_target_position'] = \
                self.command_targets[-1][self.target_index]
        if self.state_positions:
            series = [p[self.target_index] for p in self.state_positions]
            summary['initial_position'] = series[0]
            summary['minimum_observed_position'] = min(series)
            summary['maximum_observed_position'] = max(series)
            summary['final_position'] = series[-1]
            summary['absolute_response_delta'] = abs(series[-1] - series[0])
            summary['state_stamp_min'] = self.state_stamps[0]
            summary['state_stamp_max'] = self.state_stamps[-1]
            summary['state_stamp_strictly_increasing'] = all(
                b > a for a, b in zip(self.state_stamps, self.state_stamps[1:]))
        checks = self._evaluate(summary)
        summary['checks'] = checks
        summary['passed'] = bool(checks) and all(checks.values())

        if self.output_summary:
            os.makedirs(os.path.dirname(self.output_summary), exist_ok=True)
            with open(self.output_summary, 'w') as fh:
                json.dump(summary, fh, indent=2, sort_keys=True)
        if rclpy.ok():
            self.get_logger().info('phase4a_trace summary: %s' % json.dumps(summary))
        if self._jsonl is not None:
            self._jsonl.flush()
            self._jsonl.close()
            self._jsonl = None
        return summary

    def _evaluate(self, summary):
        checks = {}
        checks['has_commands'] = summary['command_count'] > 0
        checks['has_states'] = summary['state_count'] > 0
        checks['commands_accepted'] = (
            summary['command_count'] > 0
            and not summary['command_reject_counts'])
        checks['command_stamps_positive'] = summary.get('command_stamp_min', 0) > 0
        checks['command_stamps_increasing'] = summary.get(
            'command_stamps_strictly_increasing', False)
        checks['command_dim_ok'] = summary.get('command_dim_ok', False)
        checks['no_nonfinite_state'] = summary['nonfinite_state_samples'] == 0
        if 'initial_position' in summary:
            checks['target_differs_from_initial'] = abs(
                summary.get('command_target_position', 0.0)
                - summary['initial_position']) > 1e-3
            checks['joint_actually_moved'] = summary['absolute_response_delta'] > 1e-3
            checks['move_direction_consistent'] = self._direction_ok(summary)
        return checks

    def _direction_ok(self, summary):
        if 'command_target_position' not in summary:
            return False
        delta_cmd = summary['command_target_position'] - summary['initial_position']
        delta_obs = summary['final_position'] - summary['initial_position']
        if abs(delta_cmd) < 1e-3:
            return False
        # The joint must have moved toward the command (same sign), allowing the
        # controller to still be settling.
        return delta_cmd * delta_obs > 0


def main(args=None):
    rclpy.init(args=args)
    node = Phase4aTrace()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # rclpy shutdown race (e.g. RCLError while spinning)
        if rclpy.ok():
            raise
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
