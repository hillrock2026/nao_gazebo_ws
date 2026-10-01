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
Task 4 Phase 4B read-only closed-loop trace and summmariser.

Subscribes to /clock, /joint_command, /joint_states, /left_foot/contact,
/right_foot/contact, /body_pose and /imu/data, writes raw per-topic JSONL traces
(simulation seconds; source stamp kept distinct from receipt time) and computes
the producer-contract and walking-behaviour summary.

It publishes nothing and never touches the simulation. The raw traces are
sufficient to recompute the summary with tools/phase4b_recompute_summary.py.
"""

import json
import math
import os

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

import rosgraph_msgs.msg
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool

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


# ---------------------------------------------------------------- small matrix
def mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def mat_vec(a, v):
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def axis_angle(axis, q):
    n = math.sqrt(sum(a * a for a in axis))
    if n < 1e-12:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z = (a / n for a in axis)
    c, s = math.cos(q), math.sin(q)
    C = 1 - c
    return [[c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C]]


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def rot_to_rpy(R):
    pitch = math.asin(max(-1.0, min(1.0, -R[2][0])))
    roll = math.atan2(R[2][1], R[2][2])
    yaw = math.atan2(R[1][0], R[0][0])
    return roll, pitch, yaw


# Leg forward kinematics: sole position in the base_link frame.
LEG_CHAIN = {
    'L': ('LHipYawPitch', (0.0, 0.707106, -0.707106), (0.0, 0.05, -0.085),
          'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'),
    'R': ('RHipYawPitch', (0.0, 0.707106, 0.707106), (0.0, -0.05, -0.085),
          'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'),
}


def sole_position_base(q, side):
    """
    Exact serial-chain FK of the ``l_sole``/``r_sole`` frame in base_link.

    Phase 4C correctness fix: each joint origin translation must be expressed in
    the *current* link frame (``R @ (0,0,-L)``), not added as a bare ``z`` offset.
    The previous un-rotated subtraction understated the swing-foot clearance and
    fabricated a small ground penetration. The chain and origins match
    ``nao_simple.urdf`` (LHipYawPitch -> LHipRoll -> LHipPitch -> LKneePitch ->
    LAnklePitch -> LAnkleRoll -> fixed sole offset 0.04511).
    """
    yawp_name, yp_axis, yp_origin, roll_name, hp_name, kn_name, ap_name, ar_name = LEG_CHAIN[side]
    R = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    t = [float(yp_origin[0]), float(yp_origin[1]), float(yp_origin[2])]
    R = mat_mul(R, axis_angle(yp_axis, q[yawp_name]))
    R = mat_mul(R, axis_angle((1, 0, 0), q[roll_name]))
    R = mat_mul(R, axis_angle((0, 1, 0), q[hp_name]))
    d = mat_vec(R, [0.0, 0.0, -0.100])
    t = [t[i] + d[i] for i in range(3)]
    R = mat_mul(R, axis_angle((0, 1, 0), q[kn_name]))
    d = mat_vec(R, [0.0, 0.0, -0.1029])
    t = [t[i] + d[i] for i in range(3)]
    R = mat_mul(R, axis_angle((0, 1, 0), q[ap_name]))
    R = mat_mul(R, axis_angle((1, 0, 0), q[ar_name]))
    d = mat_vec(R, [0.0, 0.0, -0.04511])
    t = [t[i] + d[i] for i in range(3)]
    return t


def clock_qos():
    return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=1)


def rel_qos(depth=200):
    return QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


def join(out_dir, name):
    os.makedirs(out_dir, exist_ok=True)
    return open(os.path.join(out_dir, name), 'w')


class Phase4bTrace(Node):
    def __init__(self):
        super().__init__('phase4b_trace')
        for p, d in (('output_dir', ''), ('record_duration_sec', 0.0),
                     ('gait_method', 'walk'),
                     ('min_net_displacement_m', 0.08),
                     ('max_torso_height_loss_m', 0.06),
                     ('max_roll_pitch_deg', 35.0),
                     ('max_imu_angular_velocity', 25.0),
                     ('max_imu_linear_acceleration', 120.0),
                     ('min_alternation_count', 4),
                     ('min_command_coverage', 0.95)):
            self.declare_parameter(p, d)
        self.out_dir = self.get_parameter('output_dir').value
        self.record_duration = float(self.get_parameter('record_duration_sec').value)
        self.th = {k: self.get_parameter(k).value for k in (
            'min_net_displacement_m', 'max_torso_height_loss_m',
            'max_roll_pitch_deg', 'max_imu_angular_velocity',
            'max_imu_linear_acceleration', 'min_alternation_count',
            'min_command_coverage')}

        self.f_clock = join(self.out_dir, 'clock_trace.jsonl')
        self.f_cmd = join(self.out_dir, 'command_trace.jsonl')
        self.f_state = join(self.out_dir, 'state_trace.jsonl')
        self.f_contact = join(self.out_dir, 'contact_trace.jsonl')
        self.f_body = join(self.out_dir, 'body_pose_trace.jsonl')
        self.f_imu = join(self.out_dir, 'imu_trace.jsonl')

        self.latest_clock = None
        self.first_state_stamp = None
        self.commands = []          # (stamp, position, ok)
        self.states = []            # (stamp, position)
        self.contacts = []          # (recv_sim, left, right)
        self.bodies = []            # (stamp, x,y,z, roll,pitch,yaw, qx,qy,qz,qw)
        self.imu = []               # (stamp, wx,wy,wz, ax,ay,az)
        self.command_reject = {}
        self.last_cmd_stamp = None
        self.count = {'clock': 0, 'command': 0, 'state': 0, 'contact': 0,
                      'body': 0, 'imu': 0}

        self.create_subscription(rosgraph_msgs.msg.Clock, '/clock', self.on_clock, clock_qos())
        self.create_subscription(JointState, '/joint_command', self.on_command, rel_qos())
        self.create_subscription(JointState, '/joint_states', self.on_state, rel_qos())
        self.create_subscription(Bool, '/left_foot/contact', self.on_left, rel_qos(1))
        self.create_subscription(Bool, '/right_foot/contact', self.on_right, rel_qos(1))
        self.create_subscription(PoseStamped, '/body_pose', self.on_body, rel_qos())
        self.create_subscription(Imu, '/imu/data', self.on_imu, rel_qos())

        self.get_logger().info('phase4b_trace writing to %s' % self.out_dir)

    def dump(self, fh, obj):
        fh.write(json.dumps(obj, separators=(',', ':')) + '\n')

    def on_clock(self, msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.latest_clock = t
        self.count['clock'] += 1
        if self.count['clock'] % 20 == 0:
            self.dump(self.f_clock, {'t': t})

    def on_command(self, msg):
        self.count['command'] += 1
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        reason = None
        if len(msg.name) != 24 or len(msg.position) != 24:
            reason = 'dim'
        elif list(msg.name) != JOINT_ORDER:
            reason = 'order'
        elif len(msg.velocity) != 0:
            reason = 'velocity'
        elif len(msg.effort) != 0:
            reason = 'effort'
        elif msg.header.frame_id != '':
            reason = 'frame'
        elif stamp <= 0:
            reason = 'stamp'
        else:
            for v in msg.position:
                if not math.isfinite(v):
                    reason = 'nonfinite'
                    break
            if reason is None:
                for n, v in zip(JOINT_ORDER, msg.position):
                    lo, hi = LIMITS[n]
                    if v < lo - 1e-9 or v > hi + 1e-9:
                        reason = 'limit'
                        break
            if reason is None and self.last_cmd_stamp is not None and stamp <= self.last_cmd_stamp:
                reason = 'stamp_order'

        if reason is not None:
            self.command_reject[reason] = self.command_reject.get(reason, 0) + 1
            self.dump(self.f_cmd, {'t': stamp, 'ok': False, 'reason': reason})
            return
        self.last_cmd_stamp = stamp
        self.commands.append([stamp, [float(v) for v in msg.position]])
        self.dump(self.f_cmd, {'t': stamp, 'recv_sim': self.latest_clock, 'ok': True,
                               'position': [float(v) for v in msg.position]})

    def on_state(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0 or len(msg.position) != 24:
            return
        self.count['state'] += 1
        pos = [float(v) for v in msg.position]
        if self.first_state_stamp is None:
            self.first_state_stamp = stamp
        self.states.append([stamp, pos])
        self.dump(self.f_state, {'t': stamp, 'position': pos})
        if self.record_duration > 0 and stamp - self.first_state_stamp >= self.record_duration:
            self.get_logger().info('record duration reached; finalising')
            rclpy.shutdown()

    def on_left(self, msg):
        self.count['contact'] += 1
        self._contact('left', bool(msg.data))

    def on_right(self, msg):
        self._contact('right', bool(msg.data))

    def _contact(self, side, value):
        rec = {'t': self.latest_clock, 'side': side, 'value': value}
        self.dump(self.f_contact, rec)
        # maintain a merged timeline of (t, left, right)
        if side == 'left':
            left, right = value, (self.contacts[-1][2] if self.contacts else None)
        else:
            left, right = (self.contacts[-1][1] if self.contacts else None), value
        self.contacts.append([self.latest_clock, left, right])

    def on_body(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0:
            return
        self.count['body'] += 1
        p = msg.pose.position
        o = msg.pose.orientation
        R = quat_to_R(o.x, o.y, o.z, o.w)
        roll, pitch, yaw = rot_to_rpy(R)
        # Phase 4C: persist the normalized body quaternion directly so that foot
        # FK does not have to rebuild the rotation from Euler angles.
        n = math.sqrt(o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w)
        qx, qy, qz, qw = o.x / n, o.y / n, o.z / n, o.w / n
        self.bodies.append([stamp, p.x, p.y, p.z, roll, pitch, yaw,
                            qx, qy, qz, qw])
        self.dump(self.f_body, {'t': stamp, 'x': p.x, 'y': p.y, 'z': p.z,
                                'roll': roll, 'pitch': pitch, 'yaw': yaw,
                                'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw})

    def on_imu(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0:
            return
        self.count['imu'] += 1
        a = msg.linear_acceleration
        w = msg.angular_velocity
        self.imu.append([stamp, w.x, w.y, w.z, a.x, a.y, a.z])
        self.dump(self.f_imu, {'t': stamp, 'wx': w.x, 'wy': w.y, 'wz': w.z,
                               'ax': a.x, 'ay': a.y, 'az': a.z})

    # ------------------------------------------------------------- summary
    def _nearest(self, series, t):
        best = None
        for row in series:
            if row[0] <= t:
                best = row
            else:
                break
        return best

    def finalize(self):
        s = {'counts': dict(self.count),
             'command_reject_counts': self.command_reject}

        # ---- producer contract
        s['command_accepted'] = len(self.commands)
        if self.commands:
            stamps = [c[0] for c in self.commands]
            s['command_stamp_min'] = stamps[0]
            s['command_stamp_max'] = stamps[-1]
            s['command_stamps_strictly_increasing'] = all(
                b > a for a, b in zip(stamps, stamps[1:]))
            span = stamps[-1] - stamps[0]
            s['command_rate_hz'] = (len(stamps) - 1) / span if span > 0 else None
            dims_ok = all(len(c[1]) == 24 for c in self.commands)
            s['command_dim_ok'] = dims_ok
            in_lim = True
            for _, pos in self.commands:
                for n, v in zip(JOINT_ORDER, pos):
                    lo, hi = LIMITS[n]
                    if v < lo - 1e-9 or v > hi + 1e-9:
                        in_lim = False
            s['command_in_limit'] = in_lim
            steps = []
            for (_, a), (_, b) in zip(self.commands, self.commands[1:]):
                steps.append(max(abs(x - y) for x, y in zip(a, b)))
            s['max_command_step_rad'] = max(steps) if steps else 0.0
            first = self.commands[0][0]
            s['command_coverage'] = None
            if self.states:
                covered = sum(1 for st, _ in self.states if st >= first)
                s['command_coverage'] = covered / len(self.states)
        else:
            s['command_stamps_strictly_increasing'] = False
            s['command_dim_ok'] = False
            s['command_in_limit'] = False
            s['max_command_step_rad'] = None
            s['command_coverage'] = 0.0

        # ---- joint tracking error (hold-last command)
        max_err = {n: 0.0 for n in JOINT_ORDER}
        sum_err = {n: 0.0 for n in JOINT_ORDER}
        margin = {n: 1e9 for n in JOINT_ORDER}
        if self.commands and self.states:
            cmds = self.commands
            ci = 0
            for st, pos in self.states:
                while ci + 1 < len(cmds) and cmds[ci + 1][0] <= st:
                    ci += 1
                if cmds[ci][0] > st:
                    continue
                for i, n in enumerate(JOINT_ORDER):
                    e = abs(pos[i] - cmds[ci][1][i])
                    max_err[n] = max(max_err[n], e)
                    sum_err[n] += e
                    lo, hi = LIMITS[n]
                    margin[n] = min(margin[n], pos[i] - lo, hi - pos[i])
            s['per_joint_max_tracking_error'] = max_err
            s['per_joint_mean_tracking_error'] = {
                n: sum_err[n] / len(self.states) for n in JOINT_ORDER}
            s['per_joint_min_limit_margin'] = margin
            s['max_tracking_error'] = max(max_err.values())
            s['min_limit_margin'] = min(margin.values())

        # ---- foot motion (world)
        foot = {'l': [], 'r': []}
        if self.states and self.bodies:
            for st, pos in self.states:
                q = {n: pos[i] for i, n in enumerate(JOINT_ORDER)}
                body = self._nearest(self.bodies, st)
                if body is None:
                    continue
                _, bx, by, bz, roll, pitch, yaw = body[:7]
                if len(body) >= 11:
                    # Phase 4C: use the recorded body quaternion directly.
                    R = quat_to_R(body[7], body[8], body[9], body[10])
                else:
                    R = quat_to_R(*_euler_to_quat(roll, pitch, yaw))
                for side in ('l', 'r'):
                    pb = sole_position_base(q, side.upper())
                    base = (bx, by, bz)
                    pw = [base[i] + R[i][0] * pb[0] + R[i][1] * pb[1] + R[i][2] * pb[2]
                          for i in range(3)]
                    foot[side].append([st, pw[0], pw[1], pw[2]])
        for side in ('l', 'r'):
            if foot[side]:
                zs = [f[3] for f in foot[side]]
                s[f'{side}_foot_min_z'] = min(zs)
                s[f'{side}_foot_max_z'] = max(zs)
                s[f'{side}_foot_swing_height'] = max(zs) - min(zs)
                s[f'{side}_foot_x_range'] = (
                    max(f[1] for f in foot[side]) - min(f[1] for f in foot[side]))
                s[f'{side}_foot_world_start'] = foot[side][0][1:4]
                s[f'{side}_foot_world_end'] = foot[side][-1][1:4]

        # ---- contact alternation
        contacts = [c for c in self.contacts if c[1] is not None and c[2] is not None]
        s['contact_samples'] = len(contacts)
        trans_l = trans_r = 0
        alt = 0
        last_pattern = None
        for i in range(1, len(contacts)):
            if contacts[i][1] != contacts[i - 1][1]:
                trans_l += 1
            if contacts[i][2] != contacts[i - 1][2]:
                trans_r += 1
            pattern = (contacts[i][1], contacts[i][2])
            if pattern != last_pattern and pattern in ((True, False), (False, True)):
                alt += 1
                last_pattern = pattern
            elif pattern != last_pattern:
                last_pattern = pattern
        s['left_contact_transition_count'] = trans_l
        s['right_contact_transition_count'] = trans_r
        s['alternation_count'] = alt

        # ---- body
        if self.bodies:
            xs = [b[1] for b in self.bodies]
            ys = [b[2] for b in self.bodies]
            zs = [b[3] for b in self.bodies]
            rolls = [abs(b[4]) for b in self.bodies]
            pitches = [abs(b[5]) for b in self.bodies]
            s['body_net_displacement_m'] = math.hypot(xs[-1] - xs[0], ys[-1] - ys[0])
            s['body_net_forward_m'] = xs[-1] - xs[0]
            s['body_net_lateral_m'] = ys[-1] - ys[0]
            s['body_initial_z'] = zs[0]
            s['body_min_z'] = min(zs)
            s['body_final_z'] = zs[-1]
            s['body_torso_height_loss'] = zs[0] - min(zs)
            s['body_max_roll_deg'] = math.degrees(max(rolls))
            s['body_max_pitch_deg'] = math.degrees(max(pitches))
            s['body_final_roll_deg'] = math.degrees(self.bodies[-1][4])
            s['body_final_pitch_deg'] = math.degrees(self.bodies[-1][5])

        # ---- imu
        if self.imu:
            wmax = 0.0
            amax = 0.0
            nonfin = 0
            for row in self.imu:
                vals = row[1:]
                if any(not math.isfinite(v) for v in vals):
                    nonfin += 1
                    continue
                wmax = max(wmax, math.sqrt(row[1] ** 2 + row[2] ** 2 + row[3] ** 2))
                amax = max(amax, math.sqrt(row[4] ** 2 + row[5] ** 2 + row[6] ** 2))
            s['imu_max_angular_velocity'] = wmax
            s['imu_max_linear_acceleration'] = amax
            s['imu_nonfinite_samples'] = nonfin

        s['thresholds'] = self.th
        s['producer_checks'] = self._producer_checks(s)
        s['producer_contract_pass'] = bool(s['producer_checks']) and all(
            s['producer_checks'].values())
        s['walking_checks'] = self._walking_checks(s)
        s['walking_pass'] = bool(s['walking_checks']) and all(
            s['walking_checks'].values())
        with open(os.path.join(self.out_dir, 'phase4b_summary.json'), 'w') as fh:
            json.dump(s, fh, indent=2, sort_keys=True)
        for fh in (self.f_clock, self.f_cmd, self.f_state, self.f_contact,
                   self.f_body, self.f_imu):
            fh.flush()
            fh.close()
        if rclpy.ok():
            self.get_logger().info('phase4b_trace summary: %s' % json.dumps(s))
        return s

    def _producer_checks(self, s):
        checks = {}
        checks['commands_present'] = s.get('command_accepted', 0) > 0
        checks['commands_accepted'] = (s.get('command_accepted', 0) > 0
                                       and not s.get('command_reject_counts'))
        checks['command_stamps_increasing'] = s.get('command_stamps_strictly_increasing', False)
        checks['command_dim_ok'] = s.get('command_dim_ok', False)
        checks['command_in_limit'] = s.get('command_in_limit', False)
        checks['command_rate_ok'] = (s.get('command_rate_hz') or 0) >= 50.0
        checks['states_present'] = s.get('counts', {}).get('state', 0) > 0
        checks['command_coverage_ok'] = (
            (s.get('command_coverage') or 0) >= self.th['min_command_coverage'])
        return checks

    def _walking_checks(self, s):
        checks = {}
        checks['net_displacement_ok'] = (
            s.get('body_net_displacement_m', 0.0) >= self.th['min_net_displacement_m'])
        checks['torso_height_ok'] = (
            s.get('body_torso_height_loss', 1e9) <= self.th['max_torso_height_loss_m'])
        checks['attitude_ok'] = (
            max(s.get('body_max_roll_deg', 999.0), s.get('body_max_pitch_deg', 999.0))
            <= self.th['max_roll_pitch_deg'])
        checks['imu_ok'] = (
            s.get('imu_max_angular_velocity', 1e9) <= self.th['max_imu_angular_velocity']
            and s.get('imu_max_linear_acceleration', 1e9)
            <= self.th['max_imu_linear_acceleration'])
        checks['contact_alternation_ok'] = (
            s.get('alternation_count', 0) >= self.th['min_alternation_count'])
        checks['foot_swing_ok'] = (
            max(s.get('l_foot_swing_height', 0.0), s.get('r_foot_swing_height', 0.0)) > 0.002)
        return checks


def _euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def main(args=None):
    rclpy.init(args=args)
    node = Phase4bTrace()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
