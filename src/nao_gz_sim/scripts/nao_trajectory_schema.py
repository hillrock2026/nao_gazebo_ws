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
Frozen Task 4 trajectory schema v1.0.1 and the shared command contract.

This module is the single source of truth for:

  * the frozen 24-joint order and the canonical SDF joint limits;
  * the exact manifest object (Task4_Trajectory_Schema.md v1.0.1 section 2);
  * the exact sample key sets (sections 3, 3.1, 3.2);
  * strict (non-finite-forbidding) JSON serialization;
  * the frozen ``/joint_command`` validation rules (previously the executable
    specification in ``tools/joint_command_contract.py``) plus a bounded
    causal command history used by the Phase 4D collector.

It has no ROS dependency so it can be imported by the installed collector and by
offline tests alike.

``sync_policy`` note: v1.0.1 freezes the 4-subkey serialization of
``sync_policy`` (``main_clock`` / ``imu`` / ``contact`` / ``command``) that
v1.0.0 had left undefined. This is a non-semantic erratum: the synchronization
*thresholds* remain frozen and live in ``sync_tolerance_sec`` (see
``docs/Task4_Trajectory_Schema_v1.0.1_Erratum.md``).
"""

from collections import deque, namedtuple
import json
import math

JOINT_ORDER = [
    'HeadYaw', 'HeadPitch',
    'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll',
    'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw',
    'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll',
    'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw',
]

# Frozen canonical joint limits (must match nao_simple.sdf; checked by
# tools/check_joint_limits_consistency.py).
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

SAMPLE_KEYS = ['timestamp', 'joint_position', 'joint_velocity', 'imu',
               'left_contact', 'right_contact', 'command']
IMU_KEYS = ['stamp', 'orientation', 'angular_velocity', 'linear_acceleration']
COMMAND_KEYS = ['stamp', 'position']

IMU_FRAME_ID = 'ImuTorsoAccelerometer_frame'
IMU_NEAREST_MAX = 0.01
CONTACT_FRESHNESS_MAX = 0.05
COMMAND_HOLD_MAX = 0.05

# ----------------------------------------------------------------- command
REASON_DIM = 'command_dim_mismatch'
REASON_ORDER = 'command_joint_order_mismatch'
REASON_VEL = 'command_velocity_not_empty'
REASON_EFF = 'command_effort_not_empty'
REASON_FRAME = 'command_frame_mismatch'
REASON_ZERO = 'command_zero_timestamp'
REASON_NEGATIVE = 'command_negative_timestamp'
REASON_NONFINITE = 'command_non_finite'
REASON_LIMIT = 'command_out_of_limit'
REASON_DUP = 'command_duplicate_timestamp'
REASON_BACK = 'command_backward_timestamp'

REASONS = (
    REASON_DIM, REASON_ORDER, REASON_VEL, REASON_EFF, REASON_FRAME,
    REASON_ZERO, REASON_NEGATIVE, REASON_NONFINITE, REASON_LIMIT,
    REASON_DUP, REASON_BACK,
)

# ------------------------------------------------------------- state/imu/etc
REASON_STATE_DIM = 'joint_dim_mismatch'
REASON_STATE_ORDER = 'joint_order_mismatch'
REASON_STATE_NONFINITE = 'non_finite'
REASON_STATE_ZERO = 'zero_timestamp'
REASON_STATE_NEGATIVE = 'state_negative_timestamp'
REASON_STATE_DUP = 'duplicate_timestamp'
REASON_STATE_BACK = 'backward_timestamp'
REASON_IMU_NO_MATCH = 'imu_no_match'
REASON_IMU_QUAT_NORM = 'imu_quat_norm'
REASON_IMU_NONFINITE = 'imu_non_finite'
REASON_IMU_FRAME = 'imu_frame_mismatch'
REASON_IMU_ZERO = 'imu_zero_timestamp'
REASON_IMU_NEGATIVE = 'imu_negative_timestamp'
REASON_CONTACT = 'contact_not_fresh'
REASON_CONTACT_MISSING = 'contact_not_received'
REASON_CONTACT_STALE = 'contact_stale'
REASON_CONTACT_FUTURE = 'contact_future'
REASON_COMMAND_NOT_AVAILABLE = 'command_not_available'
REASON_COMMAND_FUTURE = 'command_future'
REASON_COMMAND_STALE = 'command_stale'
REASON_CLOCK_BACKWARD = 'clock_backward'
REASON_CLOCK_REWIND = 'clock_rewind'
REASON_CLOCK_NEGATIVE = 'clock_negative_timestamp'
REASON_CLOCK_ZERO = 'clock_zero_timestamp'
REASON_JSON_NONFINITE = 'json_non_finite'

# Accounting: terminal categories for main-clock (/joint_states) messages and
# collection boundaries. Every message counted by ``joint_messages_seen`` must
# end in exactly one of these (or remain ``pending_current``).
DISCARD_DURATION_CUTOFF = 'discarded_duration_cutoff'
DISCARD_MAX_SAMPLES_CUTOFF = 'discarded_max_samples_cutoff'
DISCARD_SHUTDOWN = 'discarded_shutdown'
DISCARD_REWIND = 'discarded_rewind'
DISCARD_PENDING_OVERFLOW = 'discarded_pending_overflow'

REASON_DURATION_CUTOFF = 'duration_cutoff'
REASON_MAX_SAMPLES_CUTOFF = 'max_samples_cutoff'
REASON_SHUTDOWN = 'shutdown'
REASON_PENDING_OVERFLOW = 'pending_overflow'

ValidationResult = namedtuple('ValidationResult', 'ok reason stamp_sec stamp_pair')


def stamp_pair(header):
    """Return (sec, nanosec) int pair from a message header stamp."""
    return (int(header.stamp.sec), int(header.stamp.nanosec))


def stamp_to_sec(pair):
    return pair[0] + pair[1] * 1e-9


def stamp_is_negative(pair):
    """
    Return True for any (sec, nanosec) pair denoting a negative sim stamp.

    ``nanosec`` is compared as an exact integer (never a float approximation) so
    a negative ``sec`` is never masked by a positive ``nanosec``.
    """
    return pair[0] < 0 or pair[1] < 0


def stamp_is_zero(pair):
    """Return True only for the all-zero (0, 0) simulation stamp."""
    return pair[0] == 0 and pair[1] == 0


def validate_command(msg, limits, prev_stamp_pair=None):
    """
    Validate one command message-like object.

    ``limits`` maps joint name -> (lower, upper) in rad.
    ``prev_stamp_pair`` is the last *accepted* command stamp pair or None.

    Returns ValidationResult(ok, reason, stamp_sec, stamp_pair).
    On failure ``reason`` is one of ``REASONS``; on success ``reason`` is None.
    """
    name = list(getattr(msg, 'name', []) or [])
    position = list(getattr(msg, 'position', []) or [])
    velocity = list(getattr(msg, 'velocity', []) or [])
    effort = list(getattr(msg, 'effort', []) or [])

    if len(name) != len(JOINT_ORDER) or len(position) != len(JOINT_ORDER):
        return ValidationResult(False, REASON_DIM, None, None)
    if name != JOINT_ORDER:
        return ValidationResult(False, REASON_ORDER, None, None)
    if len(velocity) != 0:
        return ValidationResult(False, REASON_VEL, None, None)
    if len(effort) != 0:
        return ValidationResult(False, REASON_EFF, None, None)
    if getattr(msg.header, 'frame_id', '') != '':
        return ValidationResult(False, REASON_FRAME, None, None)

    pair = stamp_pair(msg.header)
    if stamp_is_negative(pair):
        return ValidationResult(False, REASON_NEGATIVE, None, None)
    if stamp_is_zero(pair):
        return ValidationResult(False, REASON_ZERO, None, None)

    for value in position:
        if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            return ValidationResult(False, REASON_NONFINITE, None, None)

    for joint_name, value in zip(JOINT_ORDER, position):
        lower, upper = limits[joint_name]
        value = float(value)
        if value < lower or value > upper:
            return ValidationResult(False, REASON_LIMIT, None, None)

    if prev_stamp_pair is not None:
        if pair == prev_stamp_pair:
            return ValidationResult(False, REASON_DUP, None, None)
        if pair < prev_stamp_pair:
            return ValidationResult(False, REASON_BACK, None, None)

    return ValidationResult(True, None, stamp_to_sec(pair), pair)


class CommandCache:
    """
    Bounded last-command cache implementing the frozen hold-last rule.

    Only a valid, monotonically increasing command updates the cache. The cache
    is read by ``get_for(sample_time_sec)`` which returns the held 24-position
    vector only when ``0 <= sample_time - command.stamp <= hold_max_sec``.
    A future command (stamp > sample time) and a stale command
    (stamp too old) both return None.
    """

    def __init__(self, hold_max_sec=COMMAND_HOLD_MAX):
        self.hold_max_sec = float(hold_max_sec)
        self._stamp_pair = None
        self._position = None
        self.accepted = 0
        self.reject_counts = {}

    def offer(self, msg, limits):
        result = validate_command(msg, limits, self._stamp_pair)
        if result.ok:
            self._stamp_pair = result.stamp_pair
            self._position = [float(v) for v in msg.position]
            self.accepted += 1
        else:
            self.reject_counts[result.reason] = self.reject_counts.get(result.reason, 0) + 1
        return result

    def get_for(self, sample_time_sec):
        if self._stamp_pair is None:
            return None
        delta = float(sample_time_sec) - stamp_to_sec(self._stamp_pair)
        if 0.0 <= delta <= self.hold_max_sec:
            return list(self._position)
        return None

    @property
    def last_stamp_pair(self):
        return self._stamp_pair


class CommandHistory:
    """
    Bounded causal command history for exact hold-last sample selection.

    Unlike ``CommandCache`` (which only keeps the last command), this keeps a
    short bounded history so a sample can select the most recent command whose
    stamp is <= the sample time even when newer future commands are already in
    the cache. Validation still goes through the single ``validate_command``
    rule; an invalid command never updates the history.
    """

    def __init__(self, hold_max_sec=COMMAND_HOLD_MAX, max_items=200):
        self.hold_max_sec = float(hold_max_sec)
        self.max_items = int(max_items)
        self._items = deque()          # (stamp_sec, stamp_pair, position)
        self._last_pair = None
        self.accepted = 0
        self.reject_counts = {}

    def offer(self, msg, limits):
        result = validate_command(msg, limits, self._last_pair)
        if result.ok:
            self._last_pair = result.stamp_pair
            self._items.append(
                (result.stamp_sec, result.stamp_pair,
                 [float(v) for v in msg.position]))
            while len(self._items) > self.max_items:
                self._items.popleft()
            self.accepted += 1
        else:
            self.reject_counts[result.reason] = self.reject_counts.get(result.reason, 0) + 1
        return result

    def select_for(self, sample_time_sec):
        """Return (stamp_sec | None, position | None, reason | None)."""
        if not self._items:
            return None, None, REASON_COMMAND_NOT_AVAILABLE
        chosen = None
        for item in self._items:
            if item[0] <= sample_time_sec + 1e-12:
                chosen = item
            else:
                break
        if chosen is None:
            return None, None, REASON_COMMAND_FUTURE
        if sample_time_sec - chosen[0] <= self.hold_max_sec + 1e-9:
            return chosen[0], list(chosen[2]), None
        return None, None, REASON_COMMAND_STALE

    def status_for(self, sample_time_sec):
        """Return (position | None, reason | None) for hold-last selection."""
        _stamp, position, reason = self.select_for(sample_time_sec)
        return position, reason

    def prune(self, before_sec):
        while self._items and self._items[0][0] < before_sec:
            self._items.popleft()

    @property
    def last_stamp_pair(self):
        return self._last_pair


# ---------------------------------------------------------------- manifest
def build_manifest():
    return {
        'schema_name': 'nao_gazebo_trajectory',
        'schema_version': '1.0.1',
        'serialization_format': 'JSONL + manifest JSON',
        'robot': 'nao',
        'time_source': 'gazebo_simulation_time',
        'time_unit': 's',
        'joint_order': list(JOINT_ORDER),
        'joint_position_unit': 'rad',
        'joint_velocity_unit': 'rad/s',
        'imu': {
            'frame_id': IMU_FRAME_ID,
            'orientation_representation': 'quaternion_xyzw',
            'orientation_frame': IMU_FRAME_ID,
            'orientation_unit_norm': True,
            'angular_velocity_unit': 'rad/s',
            'angular_velocity_frame': IMU_FRAME_ID,
            'linear_acceleration_unit': 'm/s^2',
            'linear_acceleration_frame': IMU_FRAME_ID,
            'linear_acceleration_includes_gravity': True,
            'covariance_semantics': 'all_zero_no_noise_model',
        },
        'contact': {
            'left_topic': '/left_foot/contact',
            'right_topic': '/right_foot/contact',
            'message_type': 'std_msgs/msg/Bool',
            'has_header': False,
            'semantics': ('true = matching foot main collision pair with ground_plane '
                          'is fresh within freshness_window_sec'),
            'ground_model': 'ground_plane',
            'left_link': 'l_ankle',
            'left_collision': 'l_ankle_collision',
            'right_link': 'r_ankle',
            'right_collision': 'r_ankle_collision',
            'freshness_window_sec': CONTACT_FRESHNESS_MAX,
        },
        'command': {
            'topic': '/joint_command',
            'message_type': 'sensor_msgs/msg/JointState',
            'semantics': 'position_target',
            'dimension': 24,
            'order': 'same_as_joint_order',
            'position_unit': 'rad',
            'has_header': True,
            'header_stamp_source': 'gazebo_simulation_time',
            'velocity_array': 'must_be_empty',
            'effort_array': 'must_be_empty',
            'frame_id': '',
        },
        'sync_policy': {
            'main_clock': 'new_joint_states_message',
            'imu': 'nearest_neighbor_sim_time',
            'contact': 'clock_freshness_zero_order_hold',
            'command': 'hold_last_causal_bounded',
        },
        'sync_tolerance_sec': {
            'imu_nearest_max': IMU_NEAREST_MAX,
            'contact_freshness_max': CONTACT_FRESHNESS_MAX,
            'command_hold_max': COMMAND_HOLD_MAX,
        },
        'source_topics': {
            'joint_state': '/joint_states',
            'imu': '/imu/data',
            'left_contact': '/left_foot/contact',
            'right_contact': '/right_foot/contact',
            'command': '/joint_command',
            'clock': '/clock',
        },
        'source_message_types': {
            'joint_state': 'sensor_msgs/msg/JointState',
            'imu': 'sensor_msgs/msg/Imu',
            'left_contact': 'std_msgs/msg/Bool',
            'right_contact': 'std_msgs/msg/Bool',
            'command': 'sensor_msgs/msg/JointState',
            'clock': 'rosgraph_msgs/msg/Clock',
        },
        'missing_data_policy': 'reject_sample',
        'invalid_data_policy': 'drop_sample_and_count_reason',
    }


def strict_dumps(obj):
    """Strict JSON with no NaN/Infinity and no whitespace."""
    return json.dumps(obj, ensure_ascii=False, separators=(',', ':'), allow_nan=False)
