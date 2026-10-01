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
Task 4 Phase 4B minimal, explainable periodic gait command producer.

This is the first real ``/joint_command`` producer of the workspace. It is NOT a
random/current-state/zero publisher and it is NOT a collector. It generates a
periodic 24-dimensional physical position target from a *sagittal-plane foot
trajectory plus a small analytic leg inverse kinematics*, phase-locked to the
Gazebo simulation clock, and publishes it on the frozen interface:

    topic: /joint_command
    type:  sensor_msgs/msg/JointState
    name:  frozen 24-joint order (Task 3A / schema section 4)
    position: 24 physical position targets [rad]
    velocity/effort: empty; header.frame_id: ''; header.stamp: Gazebo sim time

Gait model (see docs/task4_phase4b_evidence/gait_design.md):

  * Each leg follows a foot-sole trajectory expressed relative to its hip in the
    sagittal (x-z) plane: a stance line that moves from +step/2 to -step/2 and a
    swing arc from -step/2 back to +step/2 lifted by ``swing_height``.
  * A 2-link analytic IK (thigh 0.100 m, shin 0.1029 m, ankle-to-sole 0.04511 m)
    converts the sole target into HipPitch/KneePitch/AnklePitch so the sole stays
    flat (pitch 0) and on the intended height.
  * A lateral weight-shift is produced by a common HipRoll / AnkleRoll sway that
    moves the pelvis over the stance foot; arms swing opposite to the legs.
  * The two legs are 180 deg out of phase with a configurable double-support
    overlap.

Every emitted message is validated before publication (finite, frozen order,
within the canonical SDF joint limits, bounded step from the previous target).
A violation stops the producer safely instead of publishing an unsafe target.

``gait_method:=stand`` holds a static in-limit stance and is used as the
pre-gait baseline. No random values, no copying of /joint_states, no teleport.
"""

import math
import os
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

# Resolve the sibling pure phase module both when installed side by side (normal
# ROS execution) and when this file is loaded directly by an offline loader
# (spec_from_file_location), which does not add the script directory to path.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import nao_gait_phase as gait_phase

JOINT_ORDER = [
    'HeadYaw', 'HeadPitch',
    'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll',
    'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw',
    'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll',
    'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw',
]

# Frozen canonical joint limits (must match nao_simple.sdf; consistency is
# checked by tools/check_joint_limits_consistency.py).
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

# Link lengths from canonical URDF (sagittal leg chain).
L_THIGH = 0.100
L_SHIN = 0.1029
L_SOLE = 0.04511
L_MAX = L_THIGH + L_SHIN          # hip-to-ankle at full extension
LEG_EXT = L_MAX + L_SOLE          # hip-to-sole at full extension = 0.24801


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


# Documented static stance: nearest in-limit value to zero (elbow rolls cannot
# be zero), a small knee bend for numeric margin.
STANCE = {name: _clamp(0.0, *LIMITS[name]) for name in JOINT_ORDER}


def _set(cmd, name, value):
    cmd[JOINT_ORDER.index(name)] = value


def _smoothstep(p):
    p = _clamp(p, 0.0, 1.0)
    return p * p * (3.0 - 2.0 * p)


def leg_ik(sole_x, sole_z, foot_pitch):
    """
    Analytic 2-link leg IK.

    ``sole_x``/``sole_z`` are the sole position relative to the hip in the
    sagittal plane (z down negative). Returns (HipPitch, KneePitch, AnklePitch)
    for a user-specified foot pitch (0 = sole parallel to the ground).
    """
    # ankle position = sole + R_y(pitch) * (0, 0, L_SOLE)
    ankle_x = sole_x + L_SOLE * math.sin(foot_pitch)
    ankle_z = sole_z + L_SOLE * math.cos(foot_pitch)
    r = math.hypot(ankle_x, ankle_z)
    r = _clamp(r, abs(L_THIGH - L_SHIN) + 1e-4, L_MAX - 1e-4)
    alpha = math.atan2(ankle_x, -ankle_z)
    c_knee = (r * r - L_THIGH * L_THIGH - L_SHIN * L_SHIN) / (2.0 * L_THIGH * L_SHIN)
    q_knee = math.acos(_clamp(c_knee, -1.0, 1.0))
    a_abs = alpha + math.atan2(
        L_SHIN * math.sin(q_knee), L_THIGH + L_SHIN * math.cos(q_knee))
    q_hip = -a_abs
    q_ankle = foot_pitch - q_hip - q_knee
    return q_hip, q_knee, q_ankle


class Phase4bProducer(Node):
    def __init__(self):
        super().__init__('phase4b_producer')
        # Interface
        self.declare_parameter('command_topic', '/joint_command')
        self.declare_parameter('state_topic', '/joint_states')
        # Method and gait geometry
        self.declare_parameter('gait_method', 'walk')  # 'stand' | 'walk'
        self.declare_parameter('step_length_m', 0.030)
        self.declare_parameter('step_frequency_hz', 0.8)
        self.declare_parameter('swing_height_m', 0.015)
        self.declare_parameter('double_support_ratio', 0.20)
        self.declare_parameter('leg_extension_m', LEG_EXT - 0.004)
        self.declare_parameter('sway_amplitude_rad', 0.15)
        # P0-a: lateral weight-shift phase. +1 keeps the historical (frozen)
        # behaviour; -1 moves the pelvis toward the *stance* foot (the physically
        # correct weight transfer: pelvis over the support foot before the other
        # foot swings). See docs/task_p0a_gait_search/Forward_Gait_Execution_Report.md.
        self.declare_parameter('sway_sign', 1.0)
        # P0-a P1: explicit support/swing phase scheduling. 'legacy' preserves
        # the historical (frozen) behaviour; 'explicit_support_swing' keeps the
        # same phase boundaries/roles but makes the lateral weight transfer
        # complete during double support (phase-aligned sway) instead of the
        # legacy free-running sinusoid.
        self.declare_parameter('gait_phase_mode', 'legacy')
        self.declare_parameter('arm_swing_rad', 0.20)
        # Natural arm baseline (Phase 4E): the shoulder pitch zero is the
        # arms-forward-horizontal pose, so a natural hanging arm needs a
        # non-zero neutral pitch (~1.5 rad). Arms swing around this baseline.
        self.declare_parameter('arm_neutral_pitch_rad', 1.50)
        self.declare_parameter('arm_neutral_roll_rad', 0.0)
        self.declare_parameter('elbow_bend_rad', 0.30)
        self.declare_parameter('arm_phase_offset_rad', 0.0)
        # +1 = natural (left arm back while left leg forward); -1 reproduces the
        # pre-Phase-4E legacy zero-pose mapping (L=-A*sin, R=+A*sin) for the
        # Path B CONTROL configuration. Default +1 leaves the natural behaviour
        # unchanged.
        self.declare_parameter('arm_swing_sign', 1)
        self.declare_parameter('foot_pitch_swing_rad', 0.10)
        self.declare_parameter('walking_direction', 'forward')
        # Timing (all simulation seconds)
        self.declare_parameter('command_rate_hz', 100.0)
        self.declare_parameter('start_delay_sec', 0.5)
        self.declare_parameter('gait_blend_sec', 1.5)
        self.declare_parameter('duration_sec', 12.0)
        self.declare_parameter('stop_ramp_sec', 1.5)
        self.declare_parameter('hold_final_sec', 2.0)
        self.declare_parameter('stand_sec', 12.0)
        # Safety
        self.declare_parameter('max_delta_per_cycle', 0.15)
        self.declare_parameter('max_step_length_m', 0.08)
        self.declare_parameter('max_swing_height_m', 0.05)

        self.command_topic = self.get_parameter('command_topic').value
        self.state_topic = self.get_parameter('state_topic').value
        self.method = self.get_parameter('gait_method').value
        self.step = float(self.get_parameter('step_length_m').value)
        self.freq = float(self.get_parameter('step_frequency_hz').value)
        self.swing_h = float(self.get_parameter('swing_height_m').value)
        self.ds_ratio = float(self.get_parameter('double_support_ratio').value)
        self.leg_ext = float(self.get_parameter('leg_extension_m').value)
        self.sway_amp = float(self.get_parameter('sway_amplitude_rad').value)
        self.sway_sign = float(self.get_parameter('sway_sign').value)
        self.phase_mode = self.get_parameter('gait_phase_mode').value
        self.arm_swing = float(self.get_parameter('arm_swing_rad').value)
        self.arm_neutral_pitch = float(
            self.get_parameter('arm_neutral_pitch_rad').value)
        self.arm_neutral_roll = float(
            self.get_parameter('arm_neutral_roll_rad').value)
        self.elbow_bend = float(self.get_parameter('elbow_bend_rad').value)
        self.arm_phase_offset = float(
            self.get_parameter('arm_phase_offset_rad').value)
        self.arm_swing_sign = float(self.get_parameter('arm_swing_sign').value)
        self.foot_pitch_swing = float(self.get_parameter('foot_pitch_swing_rad').value)
        self.direction = self.get_parameter('walking_direction').value
        self.rate = float(self.get_parameter('command_rate_hz').value)
        self.start_delay = float(self.get_parameter('start_delay_sec').value)
        self.gait_blend = float(self.get_parameter('gait_blend_sec').value)
        self.duration = float(self.get_parameter('duration_sec').value)
        self.stop_ramp = float(self.get_parameter('stop_ramp_sec').value)
        self.hold_final = float(self.get_parameter('hold_final_sec').value)
        self.stand_sec = float(self.get_parameter('stand_sec').value)
        self.max_delta = float(self.get_parameter('max_delta_per_cycle').value)
        self.max_step = float(self.get_parameter('max_step_length_m').value)
        self.max_swing_h = float(self.get_parameter('max_swing_height_m').value)

        if not self._validate_parameters():
            raise ValueError('invalid Phase 4B gait parameters')

        self.direction_sign = -1.0 if self.direction == 'backward' else 1.0
        # Static stance consistent with the gait's leg extension, so the
        # stand->walk and walk->stand transitions are continuous.
        self.stance_cmd = self._make_stance()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)
        self.pub = self.create_publisher(JointState, self.command_topic, qos)
        self.create_subscription(
            JointState, self.state_topic, self.on_state, qos)

        self.ready_stamp = None
        self.gait_start = None
        self.ramp_from = None
        self.last_stamp = None
        self.prev_target = list(self.stance_cmd)
        self.published = 0
        self.stopped = False
        self.stop_reason = 'running'
        self.period = 1.0 / self.rate
        self.timer = self.create_timer(self.period, self.on_timer)

        self.get_logger().info(
            'phase4b_producer: method=%s step=%.3fm freq=%.2fHz swing=%.3fm '
            'ds=%.2f leg_ext=%.3f sway=%.2f sway_sign=%+.0f phase_mode=%s '
            'rate=%.1fHz dir=%s '
            'arm_neutral=%.3f arm_roll=%.3f elbow=%.3f arm_swing=%.3f '
            'arm_phase_off=%.3f' % (
                self.method, self.step, self.freq, self.swing_h, self.ds_ratio,
                self.leg_ext, self.sway_amp, self.sway_sign, self.phase_mode,
                self.rate, self.direction,
                self.arm_neutral_pitch, self.arm_neutral_roll, self.elbow_bend,
                self.arm_swing, self.arm_phase_offset))

    # ------------------------------------------------------------------ checks
    def _finite(self, value):
        return isinstance(value, (int, float)) and math.isfinite(float(value))

    def _make_stance(self):
        cmd = list(STANCE.values())
        for side in ('L', 'R'):
            q_hip, q_knee, q_ankle = leg_ik(0.0, -self.leg_ext, 0.0)
            _set(cmd, side + 'HipPitch', q_hip)
            _set(cmd, side + 'KneePitch', q_knee)
            _set(cmd, side + 'AnklePitch', q_ankle)
        # Natural hanging-arm baseline, shared by spawn hold, stand baseline,
        # gait blend, stop ramp and final hold so all transitions are
        # continuous. Left/right roll and elbow signs are mirrored by the
        # canonical SDF/URDF limits.
        for side in ('L', 'R'):
            _set(cmd, side + 'ShoulderPitch', self.arm_neutral_pitch)
            _set(cmd, side + 'ElbowYaw', 0.0)
            _set(cmd, side + 'WristYaw', 0.0)
        _set(cmd, 'LShoulderRoll', self.arm_neutral_roll)
        _set(cmd, 'RShoulderRoll', -self.arm_neutral_roll)
        _set(cmd, 'LElbowRoll', -self.elbow_bend)
        _set(cmd, 'RElbowRoll', self.elbow_bend)
        return cmd

    def _validate_parameters(self):
        log = self.get_logger()
        checks = [
            ('gait_method', self.method in ('stand', 'walk')),
            ('walking_direction', self.direction in ('forward', 'backward')),
            ('step_length_m', self._finite(self.step) and 0.0 <= self.step <= self.max_step),
            ('step_frequency_hz', self._finite(self.freq) and 0.2 <= self.freq <= 2.0),
            ('swing_height_m',
             self._finite(self.swing_h) and 0.0 <= self.swing_h <= self.max_swing_h),
            ('double_support_ratio', self._finite(self.ds_ratio) and 0.0 <= self.ds_ratio < 0.6),
            ('leg_extension_m', self._finite(self.leg_ext)
             and 0.15 <= self.leg_ext <= LEG_EXT),
            ('sway_amplitude_rad', self._finite(self.sway_amp) and 0.0 <= self.sway_amp <= 0.35),
            ('sway_sign',
             self._finite(self.sway_sign) and self.sway_sign in (-1.0, 1.0)),
            ('gait_phase_mode',
             self.phase_mode in gait_phase.GAIT_PHASE_MODES),
            ('arm_swing_rad', self._finite(self.arm_swing) and 0.0 <= self.arm_swing <= 0.8),
            ('arm_neutral_pitch_rad',
             self._finite(self.arm_neutral_pitch)
             and 0.0 <= self.arm_neutral_pitch <= 1.9
             and self.arm_neutral_pitch - self.arm_swing >= LIMITS['LShoulderPitch'][0]
             and self.arm_neutral_pitch + self.arm_swing <= LIMITS['LShoulderPitch'][1]),
            ('arm_neutral_roll_rad',
             self._finite(self.arm_neutral_roll)
             and 0.0 <= self.arm_neutral_roll <= 0.30
             and self.arm_neutral_roll <= LIMITS['LShoulderRoll'][1]
             and -self.arm_neutral_roll >= LIMITS['RShoulderRoll'][0]),
            ('elbow_bend_rad',
             self._finite(self.elbow_bend)
             and 0.0349066 <= self.elbow_bend <= 1.5
             and -self.elbow_bend >= LIMITS['LElbowRoll'][0]
             and -self.elbow_bend <= LIMITS['LElbowRoll'][1]
             and self.elbow_bend <= LIMITS['RElbowRoll'][1]
             and self.elbow_bend >= LIMITS['RElbowRoll'][0]),
            ('arm_phase_offset_rad',
             self._finite(self.arm_phase_offset)
             and abs(self.arm_phase_offset) <= 2.0 * math.pi),
            ('arm_swing_sign',
             self._finite(self.arm_swing_sign)
             and self.arm_swing_sign in (-1.0, 1.0)),
            ('foot_pitch_swing_rad', self._finite(self.foot_pitch_swing)
             and 0.0 <= self.foot_pitch_swing <= 0.4),
            ('command_rate_hz', self._finite(self.rate) and 10.0 <= self.rate <= 500.0),
            ('start_delay_sec',
             self._finite(self.start_delay) and 0.0 <= self.start_delay <= 10.0),
            ('gait_blend_sec',
             self._finite(self.gait_blend) and 0.0 <= self.gait_blend <= 5.0),
            ('duration_sec', self._finite(self.duration) and 0.0 < self.duration <= 120.0),
            ('stop_ramp_sec', self._finite(self.stop_ramp) and 0.0 <= self.stop_ramp <= 10.0),
            ('hold_final_sec', self._finite(self.hold_final) and 0.0 <= self.hold_final <= 30.0),
            ('stand_sec', self._finite(self.stand_sec) and 0.0 < self.stand_sec <= 120.0),
            ('max_delta_per_cycle',
             self._finite(self.max_delta) and 0.01 <= self.max_delta <= 1.0),
        ]
        ok = True
        for name, good in checks:
            if not good:
                log.error('invalid parameter %s=%r' % (name, getattr(self, name, None)))
                ok = False
        return ok

    # ------------------------------------------------------------------ callbacks
    def on_state(self, msg):
        if self.ready_stamp is not None:
            return
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            return
        if len(msg.name) == len(JOINT_ORDER) and list(msg.name) == JOINT_ORDER:
            self.ready_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.get_logger().info(
                'model ready at sim %.3f s; gait starts at +%.2f s'
                % (self.ready_stamp, self.start_delay))

    def on_timer(self):
        if self.stopped:
            return
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return
        if self.last_stamp is not None and now <= self.last_stamp:
            return
        t = now.nanoseconds * 1e-9

        if self.ready_stamp is None:
            # Publish the stance before the model is ready, so that the actuator
            # locks the joints immediately at spawn and the robot never falls.
            target = list(self.stance_cmd)
        elif self.method == 'stand':
            target = list(self.stance_cmd)
            if t - self.ready_stamp >= self.stand_sec:
                self.finish('stand_duration_complete', t)
                return
        else:
            if self.gait_start is None:
                self.gait_start = self.ready_stamp + self.start_delay
            if t < self.gait_start:
                target = list(self.stance_cmd)
            else:
                elapsed = t - self.gait_start
                if elapsed < self.duration:
                    phase = (elapsed * self.freq) % 1.0
                    target = self.gait_target(phase)
                    if elapsed < self.gait_blend:
                        # Blend in from the static stance so the first gait
                        # command is continuous regardless of the sim-time phase.
                        p = _smoothstep(elapsed / max(self.gait_blend, 1e-6))
                        target = self.blend(list(self.stance_cmd), target, p)
                elif elapsed < self.duration + self.stop_ramp:
                    if self.ramp_from is None:
                        self.ramp_from = self.gait_target((self.duration * self.freq) % 1.0)
                    p = (elapsed - self.duration) / max(self.stop_ramp, 1e-6)
                    target = self.blend(self.ramp_from, list(self.stance_cmd), _smoothstep(p))
                elif elapsed < self.duration + self.stop_ramp + self.hold_final:
                    target = list(self.stance_cmd)
                else:
                    self.finish('gait_duration_complete', t)
                    return

        if not self._check_safety(target):
            self.finish('safety_stop', t)
            return
        self.publish_target(target, now)

    # ------------------------------------------------------------------ gait
    def gait_target(self, phase):
        cmd = list(self.stance_cmd)
        # Shared phase/foot definition (nao_gait_phase): the per-leg sagittal
        # trajectory is unchanged; the mode only changes how sway is coupled to
        # the support/swing schedule.
        for side in ('L', 'R'):
            x, z, pitch = gait_phase.leg_foot_target(
                phase, side, self.ds_ratio, self.step, self.leg_ext,
                self.swing_h, self.foot_pitch_swing, self.direction_sign)
            q_hip, q_knee, q_ankle = leg_ik(x, z, pitch)
            _set(cmd, side + 'HipPitch', q_hip)
            _set(cmd, side + 'KneePitch', q_knee)
            _set(cmd, side + 'AnklePitch', q_ankle)
        # Lateral weight shift: swing the pelvis toward the stance foot.
        # Left leg swings for phase in [0, swing_ratio); then the pelvis must be
        # over the right support foot. The measured P0-a audit showed the
        # historical sign instead moved the pelvis onto the *swinging* foot
        # (wrong), so ``sway_sign`` is exposed: +1 = historical (frozen)
        # behaviour, -1 = pelvis over the stance foot.
        if self.phase_mode == 'explicit_support_swing':
            # Phase-aligned transfer: hold the support-side sway through the
            # whole swing and cross over smoothly during double support, so the
            # weight shift is complete *before* the swing foot lifts.
            profile = gait_phase.sway_profile(phase, self.ds_ratio)
            sway = (-self.sway_sign) * self.sway_amp * profile
        else:
            sway = self.sway_sign * (
                -self.sway_amp * math.sin(2.0 * math.pi * phase))
        for side in ('L', 'R'):
            _set(cmd, side + 'HipRoll', sway)
            _set(cmd, side + 'AnkleRoll', -sway)
        # Natural arm swing around the hanging-arm baseline. Left and right
        # arms are anti-phase; the leg coordination is verified from the trace
        # and visually (at phase ~0.2 the left leg swings forward, so the left
        # arm is back and the right arm forward).
        arm_wave = self.arm_swing_sign * self.arm_swing * math.sin(
            2.0 * math.pi * phase + self.arm_phase_offset)
        _set(cmd, 'LShoulderPitch', self.arm_neutral_pitch + arm_wave)
        _set(cmd, 'RShoulderPitch', self.arm_neutral_pitch - arm_wave)
        return cmd

    def blend(self, a, b, p):
        return [x + (y - x) * p for x, y in zip(a, b)]

    def _check_safety(self, target):
        if len(target) != len(JOINT_ORDER):
            self.get_logger().error('internal: target dimension %d' % len(target))
            return False
        for name, value in zip(JOINT_ORDER, target):
            if not self._finite(value):
                self.get_logger().error('non-finite target for %s' % name)
                return False
            lo, hi = LIMITS[name]
            if value < lo - 1e-9 or value > hi + 1e-9:
                self.get_logger().error(
                    'target out of limit for %s: %.4f not in [%.4f, %.4f]'
                    % (name, value, lo, hi))
                return False
        for i, (name, value) in enumerate(zip(JOINT_ORDER, target)):
            if abs(value - self.prev_target[i]) > self.max_delta:
                self.get_logger().error(
                    'target step too large for %s: %.4f rad'
                    % (name, abs(value - self.prev_target[i])))
                return False
        return True

    def publish_target(self, target, now):
        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = ''
        msg.name = list(JOINT_ORDER)
        msg.position = [float(v) for v in target]
        msg.velocity = []
        msg.effort = []
        self.pub.publish(msg)
        self.prev_target = [float(v) for v in target]
        self.last_stamp = now
        self.published += 1

    def finish(self, reason, t):
        self.stopped = True
        self.stop_reason = reason
        self.get_logger().info(
            'phase4b_producer stopping: reason=%s last_stamp=%.3f published=%d'
            % (reason, t, self.published))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Phase4bProducer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
