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
Task 4 Phase 4D/4E-0 trajectory collector (frozen schema v1.0.1).

Subscribes only (never publishes, never controls the robot) to the frozen
sources and writes a strict JSONL dataset + manifest:

    /joint_states       (main sampling event; timestamp = header.stamp)
    /joint_command      (causal hold-last, <= 0.05 s)
    /imu/data           (nearest-neighbor on simulation time, <= 0.01 s)
    /left_foot/contact  (Bool; /clock freshness, <= 0.05 s)
    /right_foot/contact (Bool; same)
    /clock              (simulation-time watermark and contact recv_sim)

Rules (Task4_Trajectory_Schema.md v1.0.1 sections 3/5/6):

  * a sample is written only when EVERY field is valid; otherwise it is dropped
    and the reject reason is counted;
  * every source stamp must be strictly positive: a negative (or zero)
    simulation stamp is rejected at the source and never enters a cache,
    pending queue or history;
  * missing / stale / future contact is NEVER converted to ``false``;
  * command and IMU are never zero-filled, interpolated, extrapolated or
    inferred from state;
  * timestamps are strictly increasing simulation time;
  * JSON forbids NaN/Infinity; the final dataset directory contains exactly
    ``manifest.json`` and ``samples.jsonl`` (atomic directory rename on success).

Accounting (Task 4 Phase 4E-0 condition C3): every ``/joint_states`` message is
classified into exactly one terminal category (written / rejected / a collection
boundary discard) or remains ``pending_current`` at the end of the run. The
summary exposes the counters and the derived ``accounted_total`` /
``unaccounted`` so that ``joint_messages_seen == accounted_total +
unaccounted`` holds for every run.

The command validation is the single shared rule in
``nao_trajectory_schema.py`` (same executable specification used by Phase 3
tooling).
"""

from collections import deque
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

from nao_trajectory_schema import (
    build_manifest,
    COMMAND_HOLD_MAX,
    CommandHistory,
    CONTACT_FRESHNESS_MAX,
    DISCARD_DURATION_CUTOFF,
    DISCARD_MAX_SAMPLES_CUTOFF,
    DISCARD_PENDING_OVERFLOW,
    DISCARD_REWIND,
    DISCARD_SHUTDOWN,
    IMU_FRAME_ID,
    IMU_NEAREST_MAX,
    JOINT_ORDER,
    LIMITS,
    REASON_CLOCK_BACKWARD,
    REASON_CLOCK_NEGATIVE,
    REASON_CLOCK_REWIND,
    REASON_CLOCK_ZERO,
    REASON_CONTACT_FUTURE,
    REASON_CONTACT_MISSING,
    REASON_CONTACT_STALE,
    REASON_DURATION_CUTOFF,
    REASON_IMU_FRAME,
    REASON_IMU_NEGATIVE,
    REASON_IMU_NO_MATCH,
    REASON_IMU_NONFINITE,
    REASON_IMU_QUAT_NORM,
    REASON_IMU_ZERO,
    REASON_JSON_NONFINITE,
    REASON_MAX_SAMPLES_CUTOFF,
    REASON_PENDING_OVERFLOW,
    REASON_SHUTDOWN,
    REASON_STATE_BACK,
    REASON_STATE_DIM,
    REASON_STATE_DUP,
    REASON_STATE_NEGATIVE,
    REASON_STATE_NONFINITE,
    REASON_STATE_ORDER,
    REASON_STATE_ZERO,
    stamp_is_negative,
    stamp_is_zero,
    strict_dumps,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool

# Terminal categories (besides ``samples_written``) for a main-clock message.
STATE_TERMINAL_FIELDS = (
    'joint_messages_rejected',
    'sync_samples_rejected',
    'discarded_duration_cutoff',
    'discarded_max_samples_cutoff',
    'discarded_shutdown',
    'discarded_rewind',
    'discarded_pending_overflow',
)


def _finite(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


class Phase4dCollector(Node):
    def __init__(self):
        super().__init__('phase4d_collector')
        self.declare_parameter('output_dir', '')
        self.declare_parameter('summary_path', '')
        self.declare_parameter('duration_sec', 8.0)
        self.declare_parameter('max_samples', 0)
        self.declare_parameter('overwrite', False)
        self.declare_parameter('imu_nearest_max', IMU_NEAREST_MAX)
        self.declare_parameter('contact_freshness_max', CONTACT_FRESHNESS_MAX)
        self.declare_parameter('command_hold_max', COMMAND_HOLD_MAX)
        self.declare_parameter('watermark_margin_sec', 0.02)
        self.declare_parameter('imu_buffer_sec', 1.0)
        self.declare_parameter('max_pending', 500)
        self.declare_parameter('validator_script', '')

        self.output_dir = str(self.get_parameter('output_dir').value)
        self.summary_path = str(self.get_parameter('summary_path').value)
        self.duration = float(self.get_parameter('duration_sec').value)
        self.max_samples = int(self.get_parameter('max_samples').value)
        self.overwrite = bool(self.get_parameter('overwrite').value)
        self.imu_max = float(self.get_parameter('imu_nearest_max').value)
        self.contact_max = float(self.get_parameter('contact_freshness_max').value)
        self.cmd_hold = float(self.get_parameter('command_hold_max').value)
        self.watermark_margin = float(self.get_parameter('watermark_margin_sec').value)
        self.imu_buffer = float(self.get_parameter('imu_buffer_sec').value)
        self.max_pending = int(self.get_parameter('max_pending').value)
        self.validator_script = str(self.get_parameter('validator_script').value)

        if not self.output_dir:
            raise ValueError('output_dir is required')
        self.output_dir = os.path.abspath(self.output_dir)
        self.manifest_path = os.path.join(self.output_dir, 'manifest.json')
        self.samples_path = os.path.join(self.output_dir, 'samples.jsonl')
        self.staging_dir = None
        self.tmp_samples = None
        self.tmp_manifest = None
        if self.summary_path:
            self.summary_path = os.path.abspath(self.summary_path)
            if self.summary_path == self.output_dir or \
                    self.summary_path.startswith(self.output_dir + os.sep):
                raise ValueError('summary_path must NOT be inside output_dir')
            if os.path.exists(self.summary_path):
                raise ValueError('summary_path already exists: %s' % self.summary_path)
        self._prepare_output_dir()

        # synchronization state
        self.latest_clock = None
        self.last_clock = None
        self.pending = deque()          # (t, position, velocity)
        self.imu_buf = deque()          # (stamp, orientation4, ang3, lin3)
        self.contacts = {'left': deque(), 'right': deque()}  # (recv_sim, value)
        self.cmd = CommandHistory(self.cmd_hold)
        self.last_state_pair = None
        self.start_sim = None
        self.last_written_ts = None
        self.rewinding = False
        self.stopped = False
        self.finishing = False
        self.success = False

        self.stats = {
            'success': False,
            'process_completed': False,
            'dataset_committed': False,
            'has_samples': False,
            'finish_reason': None,
            'duration_sec': self.duration,
            'max_samples': self.max_samples,
            'joint_messages_seen': 0,
            'imu_messages_seen': 0,
            'imu_invalid': 0,
            'contact_messages_seen': 0,
            'joint_messages_rejected': 0,
            'samples_written': 0,
            'sync_samples_rejected': 0,
            'discarded_duration_cutoff': 0,
            'discarded_max_samples_cutoff': 0,
            'discarded_shutdown': 0,
            'discarded_rewind': 0,
            'discarded_pending_overflow': 0,
            'pending_current': 0,
            'accounted_total': 0,
            'unaccounted': 0,
            'reject_total': 0,
            'reject_counts_by_reason': {},
            'first_written_timestamp': None,
            'last_written_timestamp': None,
            'command_accepted': 0,
            'command_rejected': 0,
            'max_imu_delta': None,
            'max_contact_age_left': None,
            'max_contact_age_right': None,
            'max_command_age': None,
        }

        qos10 = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE,
                           history=HistoryPolicy.KEEP_LAST, depth=10)
        qos1 = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE,
                          history=HistoryPolicy.KEEP_LAST, depth=1)
        clock_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                               durability=DurabilityPolicy.VOLATILE,
                               history=HistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(Clock, '/clock', self.on_clock, clock_qos)
        self.create_subscription(JointState, '/joint_states', self.on_state, qos10)
        self.create_subscription(JointState, '/joint_command', self.on_command, qos10)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos10)
        self.create_subscription(Bool, '/left_foot/contact',
                                 lambda m: self.on_contact('left', m), qos1)
        self.create_subscription(Bool, '/right_foot/contact',
                                 lambda m: self.on_contact('right', m), qos1)

        self.get_logger().info(
            'phase4d_collector ready: output_dir=%s duration=%.3fs max_samples=%d '
            'overwrite=%s' % (self.output_dir, self.duration, self.max_samples,
                              self.overwrite))

    # ------------------------------------------------------------- lifecycle
    def _prepare_output_dir(self):
        """
        Create an instance-unique staging directory next to ``output_dir``.

        The final ``output_dir`` is never created or modified here; it only
        appears (or is swapped in) after the dataset is complete and validated.
        """
        parent = os.path.dirname(self.output_dir) or '.'
        if os.path.exists(self.output_dir) and not os.path.isdir(self.output_dir):
            raise ValueError('output_dir exists and is not a directory: %s'
                             % self.output_dir)
        if os.path.isdir(self.output_dir):
            entries = os.listdir(self.output_dir)
            if entries and not self.overwrite:
                raise ValueError(
                    'output_dir already exists and is not empty: %s '
                    '(pass overwrite:=true to replace)' % self.output_dir)
        os.makedirs(parent, exist_ok=True)
        self.staging_dir = tempfile.mkdtemp(
            prefix='.phase4d_staging_', dir=parent)
        self.tmp_samples = os.path.join(self.staging_dir, 'samples.jsonl')
        self.tmp_manifest = os.path.join(self.staging_dir, 'manifest.json')
        self.f_samples = open(self.tmp_samples, 'x', encoding='utf-8')

    @staticmethod
    def _fsync_dir(path):
        try:
            fd = os.open(path, os.O_RDONLY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
        except OSError:
            pass

    def _inc_reject(self, reason):
        """Diagnostic per-reason counter (does not affect the conservation)."""
        counts = self.stats['reject_counts_by_reason']
        counts[reason] = counts.get(reason, 0) + 1

    def _record_terminal(self, field, reason):
        """Count one main-clock message into exactly one terminal category."""
        self.stats[field] = self.stats.get(field, 0) + 1
        if reason is not None:
            self._inc_reject(reason)

    def _update_accounting(self):
        written = self.stats['samples_written']
        terminal = sum(self.stats[field] for field in STATE_TERMINAL_FIELDS)
        pending_current = len(self.pending)
        self.stats['pending_current'] = pending_current
        self.stats['accounted_total'] = written + terminal + pending_current
        self.stats['unaccounted'] = \
            self.stats['joint_messages_seen'] - self.stats['accounted_total']
        self.stats['reject_total'] = terminal
        self.stats['has_samples'] = written > 0

    def _write_summary(self):
        if not self.summary_path:
            return
        self._update_accounting()
        try:
            with open(self.summary_path, 'w', encoding='utf-8') as fh:
                fh.write(json.dumps(self.stats, indent=2, sort_keys=True,
                                    allow_nan=False))
        except OSError as exc:  # pragma: no cover - evidence only
            self.get_logger().error('failed to write summary: %s' % exc)

    def _swap_into_place(self):
        """Publish the staging dataset with a single directory rename."""
        parent = os.path.dirname(self.output_dir) or '.'
        if os.path.isdir(self.output_dir):
            backup = self.output_dir + '.replaced.%d' % os.getpid()
            os.replace(self.output_dir, backup)
            try:
                os.replace(self.staging_dir, self.output_dir)
            except OSError:
                os.replace(backup, self.output_dir)
                raise
            shutil.rmtree(backup, ignore_errors=True)
        else:
            os.replace(self.staging_dir, self.output_dir)
        self._fsync_dir(parent)

    def commit(self, reason):
        """Atomically publish the finished dataset (idempotent, no shutdown)."""
        if self.success:
            return
        self.stats['finish_reason'] = reason
        self._update_accounting()
        try:
            self.f_samples.flush()
            os.fsync(self.f_samples.fileno())
        finally:
            try:
                self.f_samples.close()
            except OSError:
                pass
        with open(self.tmp_manifest, 'x', encoding='utf-8') as fh:
            fh.write(json.dumps(build_manifest(), ensure_ascii=False, indent=2,
                                allow_nan=False))
            fh.flush()
            os.fsync(fh.fileno())
        self._fsync_dir(self.staging_dir)
        # Optional independent pre-commit validation (C4 golden validator).
        if self.validator_script and os.path.isfile(self.validator_script):
            proc = subprocess.run(
                [sys.executable, self.validator_script, self.staging_dir],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            if proc.returncode != 0:
                raise RuntimeError(
                    'pre-commit dataset validation failed (exit=%d):\n%s'
                    % (proc.returncode, proc.stdout))
        self._swap_into_place()
        self.success = True
        self.stats['success'] = True
        self.stats['dataset_committed'] = True
        self.stats['process_completed'] = True
        self._update_accounting()
        self._write_summary()

    def finish(self, reason):
        if self.stopped:
            return
        self.stopped = True
        self.finishing = True
        if reason == 'duration_complete':
            category, cat_reason = DISCARD_DURATION_CUTOFF, REASON_DURATION_CUTOFF
        elif reason == 'max_samples':
            category, cat_reason = DISCARD_MAX_SAMPLES_CUTOFF, REASON_MAX_SAMPLES_CUTOFF
        else:
            category, cat_reason = DISCARD_SHUTDOWN, REASON_SHUTDOWN
        if category == DISCARD_SHUTDOWN:
            # Watermark-safe shutdown: only finalise pending samples that are
            # already past the IMU watermark; never choose a best-effort nearest
            # neighbour that a later IMU message could still improve.
            while self.pending:
                if self.latest_clock is None:
                    break
                watermark = self.latest_clock - self.imu_max - self.watermark_margin
                if self.pending[0][0] <= watermark + 1e-12:
                    self._finalize(self.pending.popleft())
                else:
                    break
        # Remaining pending are an explicit collection-boundary discard.
        while self.pending:
            self.pending.popleft()
            self._record_terminal(category, cat_reason)
        self.commit(reason)
        self.finishing = False
        self.get_logger().info(
            'phase4d_collector finished: reason=%s samples=%d reject_total=%d '
            'pending=%d unaccounted=%d'
            % (reason, self.stats['samples_written'], self.stats['reject_total'],
               self.stats['pending_current'], self.stats['unaccounted']))
        if rclpy.ok():
            rclpy.shutdown()

    def abort(self, reason):
        if self.success or self.stats['process_completed']:
            return
        self.stats['process_completed'] = True
        self.stats['dataset_committed'] = False
        self.stats['success'] = False
        self.stats['finish_reason'] = reason
        try:
            self.f_samples.close()
        except OSError:
            pass
        if self.staging_dir and os.path.isdir(self.staging_dir):
            shutil.rmtree(self.staging_dir, ignore_errors=True)
        self._write_summary()

    # ------------------------------------------------------------- callbacks
    def on_clock(self, msg):
        if self.stopped:
            return
        pair = (int(msg.clock.sec), int(msg.clock.nanosec))
        if stamp_is_negative(pair):
            self._inc_reject(REASON_CLOCK_NEGATIVE)
            return
        if stamp_is_zero(pair):
            self._inc_reject(REASON_CLOCK_ZERO)
            return
        sim = pair[0] + pair[1] * 1e-9
        if self.last_clock is not None and sim < self.last_clock - 1e-9:
            self._handle_rewind()
        self.last_clock = sim
        self.latest_clock = sim
        self._drain()

    def _handle_rewind(self):
        self.get_logger().warn('simulation clock went backwards; clearing caches')
        self.rewinding = True
        dropped = len(self.pending)
        self.pending.clear()
        for _ in range(dropped):
            self._record_terminal(DISCARD_REWIND, REASON_CLOCK_REWIND)
        self._inc_reject(REASON_CLOCK_BACKWARD)
        self.imu_buf.clear()
        self.contacts['left'].clear()
        self.contacts['right'].clear()
        self.cmd = CommandHistory(self.cmd_hold)
        self.last_state_pair = None

    def on_state(self, msg):
        if self.stopped:
            return
        self.stats['joint_messages_seen'] += 1
        position = list(getattr(msg, 'position', []) or [])
        velocity = list(getattr(msg, 'velocity', []) or [])
        name = list(getattr(msg, 'name', []) or [])
        reason = self._validate_state(msg, name, position, velocity)
        if reason is not None:
            self._record_terminal('joint_messages_rejected', reason)
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.last_state_pair = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        if self.start_sim is None:
            self.start_sim = t
        if len(self.pending) >= self.max_pending:
            self.pending.popleft()
            self._record_terminal(DISCARD_PENDING_OVERFLOW, REASON_PENDING_OVERFLOW)
        self.pending.append((t, [float(v) for v in position],
                             [float(v) for v in velocity]))
        self._drain()

    def _validate_state(self, msg, name, position, velocity):
        if len(name) != len(JOINT_ORDER) or len(position) != len(JOINT_ORDER) \
                or len(velocity) != len(JOINT_ORDER):
            return REASON_STATE_DIM
        if name != JOINT_ORDER:
            return REASON_STATE_ORDER
        pair = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        if stamp_is_negative(pair):
            return REASON_STATE_NEGATIVE
        if stamp_is_zero(pair):
            return REASON_STATE_ZERO
        for value in position + velocity:
            if not _finite(value):
                return REASON_STATE_NONFINITE
        if self.last_state_pair is not None:
            if pair == self.last_state_pair:
                return REASON_STATE_DUP
            if pair < self.last_state_pair:
                return REASON_STATE_BACK
        return None

    def on_command(self, msg):
        if self.stopped:
            return
        result = self.cmd.offer(msg, LIMITS)
        if result.ok:
            self.stats['command_accepted'] = self.cmd.accepted
        else:
            self.stats['command_rejected'] += 1

    def on_imu(self, msg):
        if self.stopped:
            return
        self.stats['imu_messages_seen'] += 1
        reason = self._validate_imu(msg)
        if reason is not None:
            self.stats['imu_invalid'] += 1
            self._inc_reject(reason)
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.imu_buf.append((
            stamp,
            [float(msg.orientation.x), float(msg.orientation.y),
             float(msg.orientation.z), float(msg.orientation.w)],
            [float(msg.angular_velocity.x), float(msg.angular_velocity.y),
             float(msg.angular_velocity.z)],
            [float(msg.linear_acceleration.x), float(msg.linear_acceleration.y),
             float(msg.linear_acceleration.z)]))
        if self.latest_clock is not None:
            self._prune_imu()

    def _validate_imu(self, msg):
        if msg.header.frame_id != IMU_FRAME_ID:
            return REASON_IMU_FRAME
        pair = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        if stamp_is_negative(pair):
            return REASON_IMU_NEGATIVE
        if stamp_is_zero(pair):
            return REASON_IMU_ZERO
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        values = q + [msg.angular_velocity.x, msg.angular_velocity.y,
                      msg.angular_velocity.z, msg.linear_acceleration.x,
                      msg.linear_acceleration.y, msg.linear_acceleration.z]
        if any(not _finite(v) for v in values):
            return REASON_IMU_NONFINITE
        norm = math.sqrt(sum(v * v for v in q))
        if not math.isfinite(norm) or norm < 1e-9 or abs(norm - 1.0) > 1e-6:
            return REASON_IMU_QUAT_NORM
        return None

    def on_contact(self, foot, msg):
        if self.stopped:
            return
        self.stats['contact_messages_seen'] += 1
        if self.latest_clock is None:
            return
        self.contacts[foot].append((self.latest_clock, bool(msg.data)))
        while self.contacts[foot] and \
                self.latest_clock - self.contacts[foot][0][0] > self.contact_max + 0.5:
            self.contacts[foot].popleft()

    # ----------------------------------------------------------- synchronization
    def _prune_imu(self):
        lo = self.latest_clock - self.imu_buffer
        while self.imu_buf and self.imu_buf[0][0] < lo:
            self.imu_buf.popleft()

    def _nearest_imu(self, t):
        best = None
        best_delta = None
        for rec in self.imu_buf:
            delta = abs(rec[0] - t)
            if best_delta is None or delta < best_delta - 1e-12 or \
                    (abs(delta - best_delta) <= 1e-12 and rec[0] < best[0]):
                best = rec
                best_delta = delta
        if best is None or best_delta is None or best_delta > self.imu_max + 1e-9:
            return None, None
        return best, best_delta

    def _contact_value(self, foot, t):
        dq = self.contacts[foot]
        chosen = None
        for rec in dq:
            if rec[0] <= t + 1e-12:
                chosen = rec
            else:
                break
        if chosen is None:
            return None, (REASON_CONTACT_MISSING if not dq else REASON_CONTACT_FUTURE)
        age = t - chosen[0]
        if age < -1e-9:
            return None, REASON_CONTACT_FUTURE
        if age > self.contact_max + 1e-9:
            return None, REASON_CONTACT_STALE
        return bool(chosen[1]), age

    def _drain(self):
        if self.stopped or self.latest_clock is None:
            return
        watermark = self.latest_clock - self.imu_max - self.watermark_margin
        while self.pending and self.pending[0][0] <= watermark + 1e-12:
            self._finalize(self.pending.popleft())
            if self.stopped:
                return

    def _finalize(self, sample):
        if self.stopped and not self.finishing:
            return
        t, position, velocity = sample
        if self.rewinding:
            if self.last_written_ts is not None and t <= self.last_written_ts:
                self._record_terminal(DISCARD_REWIND, REASON_CLOCK_REWIND)
                return
            self.rewinding = False
        if self.last_written_ts is not None and t <= self.last_written_ts + 1e-9:
            self._record_terminal('sync_samples_rejected', REASON_STATE_BACK)
            return
        if not self.finishing and self.duration > 0 and self.start_sim is not None \
                and t - self.start_sim > self.duration + 1e-9:
            self._record_terminal(DISCARD_DURATION_CUTOFF, REASON_DURATION_CUTOFF)
            self.finish('duration_complete')
            return

        imu, imu_delta = self._nearest_imu(t)
        if imu is None:
            self._record_terminal('sync_samples_rejected', REASON_IMU_NO_MATCH)
            self._after_reject(t)
            return
        cmd_sec, cmd_pos, cmd_reason = self.cmd.select_for(t)
        if cmd_pos is None:
            self._record_terminal('sync_samples_rejected', cmd_reason)
            self._after_reject(t)
            return
        left, left_age = self._contact_value('left', t)
        if left is None:
            self._record_terminal('sync_samples_rejected', left_age)
            self._after_reject(t)
            return
        right, right_age = self._contact_value('right', t)
        if right is None:
            self._record_terminal('sync_samples_rejected', right_age)
            self._after_reject(t)
            return
        cmd_age = t - cmd_sec

        record = {
            'timestamp': t,
            'joint_position': position,
            'joint_velocity': velocity,
            'imu': {
                'stamp': imu[0],
                'orientation': imu[1],
                'angular_velocity': imu[2],
                'linear_acceleration': imu[3],
            },
            'left_contact': left,
            'right_contact': right,
            'command': {'stamp': cmd_sec, 'position': cmd_pos},
        }
        try:
            line = strict_dumps(record)
        except ValueError:
            self._record_terminal('sync_samples_rejected', REASON_JSON_NONFINITE)
            self._after_reject(t)
            return
        self.f_samples.write(line + '\n')
        self.stats['samples_written'] += 1
        self.last_written_ts = t
        if self.stats['first_written_timestamp'] is None:
            self.stats['first_written_timestamp'] = t
        self.stats['last_written_timestamp'] = t
        self.stats['max_imu_delta'] = _max_opt(self.stats['max_imu_delta'], imu_delta)
        self.stats['max_command_age'] = _max_opt(self.stats['max_command_age'], cmd_age)
        self.stats['max_contact_age_left'] = _max_opt(
            self.stats['max_contact_age_left'], left_age)
        self.stats['max_contact_age_right'] = _max_opt(
            self.stats['max_contact_age_right'], right_age)
        self._prune_buffers(t)

        if not self.finishing:
            if self.max_samples > 0 and self.stats['samples_written'] >= self.max_samples:
                self.finish('max_samples')
                return
            if self.duration > 0 and self.start_sim is not None and \
                    t - self.start_sim >= self.duration - 1e-9:
                self.finish('duration_complete')

    def _after_reject(self, t):
        self._prune_buffers(t)

    def _prune_buffers(self, t):
        if self.latest_clock is not None:
            self._prune_imu()
        self.cmd.prune(t - self.cmd_hold - 0.5)
        cutoff = t - self.contact_max - 0.5
        for foot in ('left', 'right'):
            while self.contacts[foot] and self.contacts[foot][0][0] < cutoff:
                self.contacts[foot].popleft()


def _max_opt(current, value):
    if value is None:
        return current
    if current is None:
        return value
    return max(current, value)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Phase4dCollector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None:
            node.abort('interrupted')
    except Exception:
        if node is not None:
            node.abort('exception')
        raise
    finally:
        if node is not None:
            if not node.success:
                node.abort('not_finished')
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
