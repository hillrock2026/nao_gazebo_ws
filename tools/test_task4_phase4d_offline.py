#!/usr/bin/env python3
"""Task 4 Phase 4D/4E-0 OFFLINE unit/fixture tests (no running simulation).

These tests are explicitly offline: they construct message objects in-process
and exercise the collector's callbacks, the shared command rules and the
independent golden dataset validator directly. They do NOT represent real
collection and must never be presented as such.

Covers Task 4 Phase 4E-0 conditions:
    C2  negative simulation timestamps rejected at the source
    C3  accounting conservation (every /joint_states message classified)
    C4  independent golden validator (no writer import) + mutation tests

Usage:
    python3 tools/test_task4_phase4d_offline.py [--json-out F]

Exit 0 = all offline tests passed.
"""

import argparse
import copy
import importlib.util
import json
import os
import shutil
import sys
import tempfile
import types

import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool

_SCRIPTS = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'nao_gz_sim', 'scripts'))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from nao_trajectory_schema import (  # noqa: E402
    DISCARD_DURATION_CUTOFF,
    DISCARD_MAX_SAMPLES_CUTOFF,
    DISCARD_PENDING_OVERFLOW,
    DISCARD_REWIND,
    DISCARD_SHUTDOWN,
    JOINT_ORDER,
    LIMITS,
    REASON_COMMAND_FUTURE,
    REASON_COMMAND_NOT_AVAILABLE,
    REASON_COMMAND_STALE,
    REASON_DIM,
    REASON_IMU_NEGATIVE,
    REASON_LIMIT,
    REASON_NEGATIVE,
    REASON_NONFINITE,
    REASON_STATE_NEGATIVE,
    REASON_ZERO,
    CommandHistory,
    build_manifest,
    stamp_is_negative,
    stamp_is_zero,
    strict_dumps,
    validate_command,
)

_VALIDATOR_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               'validate_task4_phase4d_dataset.py')

_spec = importlib.util.spec_from_file_location(
    'phase4d_collector', os.path.join(_SCRIPTS, 'phase4d_collector.py'))
collector_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(collector_mod)

_spec_v = importlib.util.spec_from_file_location(
    'validate_task4_phase4d_dataset', _VALIDATOR_PATH)
validator_mod = importlib.util.module_from_spec(_spec_v)
_spec_v.loader.exec_module(validator_mod)

RESULTS = []


def check(name, condition, detail=None):
    RESULTS.append({'name': name, 'pass': bool(condition), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if condition else 'FAIL', name,
                           '' if condition else ' :: %s' % (detail,)))


# --------------------------------------------------------------- msg helpers
# A position that is inside every frozen SDF limit (elbow roll cannot be zero).
DEFAULT_POSITION = [0.0] * 24
DEFAULT_POSITION[11] = -0.0349066   # LElbowRoll lower limit
DEFAULT_POSITION[22] = 0.0349066    # RElbowRoll upper limit


def state_msg(t, position=None, velocity=None, name=None, stamp_zero=False):
    m = JointState()
    m.header.stamp.sec = 0 if stamp_zero else int(t)
    m.header.stamp.nanosec = 0 if stamp_zero else int(round((t - int(t)) * 1e9))
    m.name = list(JOINT_ORDER) if name is None else name
    m.position = [0.0] * 24 if position is None else position
    m.velocity = [0.0] * 24 if velocity is None else velocity
    m.effort = []
    return m


def state_msg_stamp(sec, nsec, position=None):
    m = JointState()
    m.header.stamp.sec = sec
    m.header.stamp.nanosec = nsec
    m.name = list(JOINT_ORDER)
    m.position = list(DEFAULT_POSITION) if position is None else position
    m.velocity = [0.0] * 24
    m.effort = []
    return m


def command_msg(t, position=None, velocity=None, frame_id='', name=None):
    m = JointState()
    m.header.stamp.sec = int(t)
    m.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    m.header.frame_id = frame_id
    m.name = list(JOINT_ORDER) if name is None else name
    m.position = list(DEFAULT_POSITION) if position is None else position
    m.velocity = [] if velocity is None else velocity
    m.effort = []
    return m


def fake_command(sec, nsec, position=None):
    return types.SimpleNamespace(
        name=list(JOINT_ORDER),
        position=list(DEFAULT_POSITION) if position is None else position,
        velocity=[], effort=[],
        header=types.SimpleNamespace(
            frame_id='', stamp=types.SimpleNamespace(sec=sec, nanosec=nsec)))


def imu_msg(t, frame_id='ImuTorsoAccelerometer_frame', quat=(0.0, 0.0, 0.0, 1.0)):
    m = Imu()
    m.header.stamp.sec = int(t)
    m.header.stamp.nanosec = int(round((t - int(t)) * 1e9))
    m.header.frame_id = frame_id
    m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = quat
    m.angular_velocity.x = 0.0
    m.angular_velocity.y = 0.0
    m.angular_velocity.z = 0.0
    m.linear_acceleration.x = 0.0
    m.linear_acceleration.y = 0.0
    m.linear_acceleration.z = 9.81
    return m


def imu_msg_stamp(sec, nsec):
    m = Imu()
    m.header.stamp.sec = sec
    m.header.stamp.nanosec = nsec
    m.header.frame_id = 'ImuTorsoAccelerometer_frame'
    m.orientation.w = 1.0
    m.linear_acceleration.z = 9.81
    return m


def clock_msg(t):
    m = Clock()
    m.clock.sec = int(t)
    m.clock.nanosec = int(round((t - int(t)) * 1e9))
    return m


def clock_msg_stamp(sec, nsec):
    m = Clock()
    m.clock.sec = sec
    m.clock.nanosec = nsec
    return m


def bool_msg(value):
    m = Bool()
    m.data = value
    return m


def make_args(output_dir, duration_sec=0.0, max_samples=0, overwrite=False,
              watermark_margin_sec=0.0):
    return ['--ros-args',
            '-p', 'output_dir:=%s' % output_dir,
            '-p', 'duration_sec:=%s' % duration_sec,
            '-p', 'max_samples:=%s' % max_samples,
            '-p', 'overwrite:=%s' % ('true' if overwrite else 'false'),
            '-p', 'watermark_margin_sec:=%s' % watermark_margin_sec]


# ---------------------------------------------------------- schema/contract
def test_command_rules():
    limits = LIMITS
    check('cmd_valid', validate_command(command_msg(1.0), limits).ok)
    check('cmd_dim', validate_command(command_msg(1.0, position=[0.0] * 23), limits).reason
          == REASON_DIM)
    check('cmd_order', validate_command(
        command_msg(1.0, name=list(reversed(JOINT_ORDER))), limits).reason
        == 'command_joint_order_mismatch')
    check('cmd_velocity_not_empty', validate_command(
        command_msg(1.0, velocity=[0.0] * 24), limits).reason == 'command_velocity_not_empty')
    bad_eff = command_msg(1.0)
    bad_eff.effort = [0.0] * 24
    check('cmd_effort_not_empty',
          validate_command(bad_eff, limits).reason == 'command_effort_not_empty')
    check('cmd_frame', validate_command(
        command_msg(1.0, frame_id='world'), limits).reason == 'command_frame_mismatch')
    check('cmd_zero', validate_command(
        command_msg(0.0), limits).reason == REASON_ZERO)
    bad_nf = command_msg(1.0)
    bad_nf.position[0] = float('nan')
    check('cmd_nonfinite', validate_command(bad_nf, limits).reason == REASON_NONFINITE)
    bad_lim = command_msg(1.0)
    bad_lim.position[0] = 99.0
    check('cmd_out_of_limit', validate_command(bad_lim, limits).reason == REASON_LIMIT)
    check('cmd_duplicate', validate_command(
        command_msg(1.0), limits, prev_stamp_pair=(1, 0)).reason
        == 'command_duplicate_timestamp')
    check('cmd_backward', validate_command(
        command_msg(0.5), limits, prev_stamp_pair=(1, 0)).reason
        == 'command_backward_timestamp')

    hist = CommandHistory(hold_max_sec=0.05)
    check('hist_invalid_does_not_update',
          not hist.offer(bad_lim, limits).ok and hist.last_stamp_pair is None)
    hist.offer(command_msg(1.00), limits)
    hist.offer(command_msg(1.10), limits)  # future for a sample at 1.05
    stamp, pos, reason = hist.select_for(1.05)
    check('hist_selects_causal', pos is not None and abs(stamp - 1.00) < 1e-9,
          (stamp, reason))
    stamp, pos, reason = hist.select_for(1.20)
    check('hist_stale', pos is None and reason == REASON_COMMAND_STALE, reason)
    stamp, pos, reason = hist.select_for(0.90)
    check('hist_future_only', pos is None and reason == REASON_COMMAND_FUTURE, reason)
    empty = CommandHistory(hold_max_sec=0.05)
    stamp, pos, reason = empty.select_for(1.0)
    check('hist_not_available',
          pos is None and reason == REASON_COMMAND_NOT_AVAILABLE, reason)


def test_strict_json():
    for bad in (float('nan'), float('inf'), float('-inf')):
        try:
            strict_dumps({'x': bad})
            ok = False
        except ValueError:
            ok = True
        check('strict_json_rejects_%s' % ('nan' if bad != bad else str(bad)), ok)
    check('strict_json_no_whitespace', strict_dumps({'a': 1, 'b': 2}) == '{"a":1,"b":2}')


def test_manifest():
    m = build_manifest()
    check('manifest_keys_count', len(m) == 18, len(m))
    check('manifest_schema_version', m['schema_version'] == '1.0.1', m['schema_version'])
    check('manifest_sync_policy', m['sync_policy'] == {
        'main_clock': 'new_joint_states_message',
        'imu': 'nearest_neighbor_sim_time',
        'contact': 'clock_freshness_zero_order_hold',
        'command': 'hold_last_causal_bounded'}, m['sync_policy'])
    check('manifest_no_forbidden',
          not ({'sample_count', 'episode_id', 'created_at', 'git_hash', 'statistics',
                'success_label', 'supplementary_contact', 'body_pose'} & set(m)))
    check('manifest_sync_tolerance',
          m['sync_tolerance_sec'] == {'imu_nearest_max': 0.01,
                                      'contact_freshness_max': 0.05,
                                      'command_hold_max': 0.05})


# ---------------------------------------------------- C2 negative timestamps
def test_negative_timestamps():
    limits = LIMITS
    # command: first message negative is rejected with the negative reason
    check('negative_command_first_message',
          validate_command(fake_command(-1, 0), limits).reason == REASON_NEGATIVE)
    check('negative_command_negative_fractional_sec',
          validate_command(fake_command(-5, 500000000), limits).reason == REASON_NEGATIVE)
    check('negative_command_pair_helper',
          stamp_is_negative((-1, 0)) and stamp_is_negative((-5, 500000000))
          and not stamp_is_negative((0, 1)) and not stamp_is_negative((1, 0)))
    check('zero_stamp_pair_helper', stamp_is_zero((0, 0)) and not stamp_is_zero((0, 1)))

    hist = CommandHistory(hold_max_sec=0.05)
    res = hist.offer(fake_command(-1, 0), limits)
    check('negative_command_does_not_update_history',
          not res.ok and res.reason == REASON_NEGATIVE
          and hist.last_stamp_pair is None and hist.accepted == 0
          and hist.select_for(0.5)[2] == REASON_COMMAND_NOT_AVAILABLE,
          (res.reason, hist.last_stamp_pair, hist.accepted))
    # control: (0,0) still uses the zero reason (not misreported as negative)
    check('zero_command_control_reason',
          validate_command(fake_command(0, 0), limits).reason == REASON_ZERO)

    tmp = tempfile.mkdtemp(prefix='p4d_neg_')
    try:
        def case(node):
            node.on_clock(clock_msg_stamp(-1, 0))
            check('negative_clock_not_applied',
                  node.latest_clock is None
                  and node.stats['reject_counts_by_reason'].get(
                      'clock_negative_timestamp', 0) == 1, node.latest_clock)
            node.on_clock(clock_msg_stamp(0, 0))
            check('zero_clock_not_applied',
                  node.latest_clock is None
                  and node.stats['reject_counts_by_reason'].get(
                      'clock_zero_timestamp', 0) == 1, node.latest_clock)

            node.on_imu(imu_msg_stamp(-1, 0))
            check('negative_imu_not_buffered',
                  len(node.imu_buf) == 0 and node.stats['imu_invalid'] == 1
                  and node.stats['reject_counts_by_reason'].get(
                      REASON_IMU_NEGATIVE, 0) == 1, len(node.imu_buf))
            node.on_imu(imu_msg_stamp(2, 0))
            check('positive_imu_buffered', len(node.imu_buf) == 1, len(node.imu_buf))

            node.on_state(state_msg_stamp(-1, 0))
            node.on_state(state_msg_stamp(-5, 500000000))
            check('negative_state_not_pending',
                  len(node.pending) == 0 and node.last_state_pair is None
                  and node.stats['reject_counts_by_reason'].get(
                      REASON_STATE_NEGATIVE, 0) == 2, len(node.pending))
            node.on_state(state_msg_stamp(0, 0))
            check('zero_state_reason',
                  len(node.pending) == 0
                  and node.stats['reject_counts_by_reason'].get('zero_timestamp', 0) == 1)
            node.on_state(state_msg_stamp(2, 0))
            check('positive_state_pending', len(node.pending) == 1, len(node.pending))
            node.commit('negative_test')
        with_collector(tmp, case)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------- C3 accounting
def with_collector(tmp, fn, **params):
    output_dir = os.path.join(tmp, 'ds')
    rclpy.init(args=make_args(output_dir, **params))
    node = None
    try:
        node = collector_mod.Phase4dCollector()
        return fn(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _accounted(node):
    node._update_accounting()
    s = node.stats
    return (s['unaccounted'] == 0
            and s['joint_messages_seen'] == s['accounted_total']), s


def _conserved(node):
    ok, s = _accounted(node)
    return ok and s['pending_current'] == 0, s


def _queue_samples(node, n, t0=0.5):
    """Queue n pending samples without advancing the clock past the watermark."""
    node.on_clock(clock_msg(t0))
    node.on_contact('left', bool_msg(False))
    node.on_contact('right', bool_msg(False))
    for k in range(n):
        t = t0 + k * 0.01
        node.on_imu(imu_msg(t))
        node.on_command(command_msg(t))
        node.on_state(state_msg(t))
        node.on_contact('left', bool_msg(False))
        node.on_contact('right', bool_msg(False))
    return t0


def drive(node, n=6, t0=0.5, malformed=None, imu_offset=0.0, contact=False,
          command_offset=0.0):
    dt = 0.01
    node.on_clock(clock_msg(t0))
    if contact:
        node.on_contact('left', bool_msg(False))
        node.on_contact('right', bool_msg(False))
    for k in range(n):
        t = t0 + k * dt
        node.on_imu(imu_msg(t + imu_offset))
        if malformed != 'no_command':
            if malformed == 'invalid_command':
                node.on_command(command_msg(t, position=[99.0] + [0.0] * 23))
            else:
                node.on_command(command_msg(t + command_offset))
        node.on_state(state_msg(t))
        node.on_clock(clock_msg(t + 0.02))
        if contact:
            node.on_contact('left', bool_msg(False))
            node.on_contact('right', bool_msg(False))


def test_collector_offline():
    # 1. full valid stream writes samples
    tmp = tempfile.mkdtemp(prefix='p4d_offline_')
    try:
        def case1(node):
            drive(node, n=6, contact=True)
            node.commit('offline_test')
            lines = [json.loads(l) for l in
                     open(os.path.join(node.output_dir, 'samples.jsonl'))]
            check('collector_writes_valid_samples', len(lines) == 6, len(lines))
            check('collector_dataset_dir_exact',
                  sorted(os.listdir(node.output_dir)) == ['manifest.json', 'samples.jsonl'])
            ok, s = _conserved(node)
            check('collector_conservation_positive', ok, s)
        with_collector(tmp, case1)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # 2. no command -> zero samples
    tmp = tempfile.mkdtemp(prefix='p4d_offline_')
    try:
        def case2(node):
            drive(node, n=6, malformed='no_command', contact=True)
            node.commit('offline_test')
            check('collector_no_command_zero_samples',
                  node.stats['samples_written'] == 0, node.stats['samples_written'])
            check('collector_no_command_reason',
                  node.stats['reject_counts_by_reason'].get(
                      REASON_COMMAND_NOT_AVAILABLE, 0) >= 6,
                  node.stats['reject_counts_by_reason'])
            ok, s = _conserved(node)
            check('collector_conservation_no_command', ok, s)
        with_collector(tmp, case2)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # 3. invalid command never updates cache
    tmp = tempfile.mkdtemp(prefix='p4d_offline_')
    try:
        def case3(node):
            drive(node, n=3, malformed='invalid_command', contact=True)
            check('collector_invalid_command_no_cache',
                  node.stats['samples_written'] == 0 and node.cmd.accepted == 0
                  and node.stats['command_rejected'] >= 3,
                  (node.stats['samples_written'], node.cmd.accepted,
                   node.stats['command_rejected']))
        with_collector(tmp, case3)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # 4. contact missing -> zero samples + contact reason
    tmp = tempfile.mkdtemp(prefix='p4d_offline_')
    try:
        def case4(node):
            drive(node, n=4, contact=False)
            check('collector_contact_missing_zero',
                  node.stats['samples_written'] == 0
                  and node.stats['reject_counts_by_reason'].get(
                      'contact_not_received', 0) >= 1,
                  node.stats['reject_counts_by_reason'])
        with_collector(tmp, case4)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # 5. IMU out of window -> reject
    tmp = tempfile.mkdtemp(prefix='p4d_offline_')
    try:
        def case5(node):
            drive(node, n=3, contact=True, imu_offset=0.5)
            check('collector_imu_no_match',
                  node.stats['samples_written'] == 0
                  and node.stats['reject_counts_by_reason'].get('imu_no_match', 0) >= 1,
                  node.stats['reject_counts_by_reason'])
        with_collector(tmp, case5)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # 6. output collision default refuses
    tmp = tempfile.mkdtemp(prefix='p4d_offline_')
    try:
        os.makedirs(os.path.join(tmp, 'ds'))
        with open(os.path.join(tmp, 'ds', 'manifest.json'), 'w') as fh:
            fh.write('{}')
        collision_ok = False
        try:
            rclpy.init(args=make_args(os.path.join(tmp, 'ds'), overwrite=False))
            coll = collector_mod.Phase4dCollector()
            coll.destroy_node()
        except Exception as exc:  # noqa: BLE001
            collision_ok = 'already exists' in str(exc)
        finally:
            if rclpy.ok():
                rclpy.shutdown()
        check('collector_output_collision_refused', collision_ok)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # 7. interrupt/abort leaves no success-shaped dataset and no staging dir
    tmp = tempfile.mkdtemp(prefix='p4d_offline_')
    try:
        def case7(node):
            drive(node, n=3, contact=True)
            node.abort('interrupt')
            check('collector_abort_no_final_dataset',
                  not os.path.exists(node.output_dir)
                  and not any(n.startswith('.phase4d_staging_')
                              for n in os.listdir(tmp)),
                  os.listdir(tmp))
        with_collector(tmp, case7)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # 8. simulation clock rewind rejects until past the last written timestamp
    tmp = tempfile.mkdtemp(prefix='p4d_offline_')
    try:
        def case8(node):
            drive(node, n=2, contact=True)
            last = node.last_written_ts
            node.on_clock(clock_msg(0.4))            # backward
            node.on_state(state_msg(last - 0.005))   # below last written
            node.on_clock(clock_msg(last + 0.2))     # drain
            reasons = node.stats['reject_counts_by_reason']
            check('collector_clock_rewind_rejected',
                  reasons.get('clock_backward', 0) >= 1
                  and reasons.get('clock_rewind', 0) >= 1, reasons)
        with_collector(tmp, case8)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_accounting():
    # A. exact max_samples = 1 with pending remainder discarded explicitly
    tmp = tempfile.mkdtemp(prefix='p4d_acc_')
    try:
        def case_n1(node):
            _queue_samples(node, 3)
            node.on_clock(clock_msg(0.51))     # watermark 0.50 -> finalize first
            node.commit('t')
            s = node.stats
            check('accounting_max1_exact', s['samples_written'] == 1,
                  s['samples_written'])
            check('accounting_max1_remainder',
                  s[DISCARD_MAX_SAMPLES_CUTOFF] == 2, s[DISCARD_MAX_SAMPLES_CUTOFF])
            ok, s = _conserved(node)
            check('accounting_max1_conserved', ok, s)
        with_collector(tmp, case_n1, max_samples=1)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # B. exact max_samples = 2 with pending remainder
    tmp = tempfile.mkdtemp(prefix='p4d_acc_')
    try:
        def case_n2(node):
            _queue_samples(node, 4)
            node.on_clock(clock_msg(0.51))     # finalize #1
            node.on_clock(clock_msg(0.52))     # finalize #2 -> cap
            node.commit('t')
            s = node.stats
            check('accounting_max2_exact', s['samples_written'] == 2, s['samples_written'])
            check('accounting_max2_remainder',
                  s[DISCARD_MAX_SAMPLES_CUTOFF] == 2, s[DISCARD_MAX_SAMPLES_CUTOFF])
            ok, s = _conserved(node)
            check('accounting_max2_conserved', ok, s)
        with_collector(tmp, case_n2, max_samples=2)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # C. N < pending (already covered by A) and duration+max both configured
    tmp = tempfile.mkdtemp(prefix='p4d_acc_')
    try:
        def case_both(node):
            drive(node, n=5, contact=True)
            s = node.stats
            check('accounting_both_cap', s['samples_written'] == 2, s['samples_written'])
            ok, s = _conserved(node)
            check('accounting_both_conserved', ok, s)
        with_collector(tmp, case_both, max_samples=2, duration_sec=0.02)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # D. duration cutoff discards the over-duration split explicitly
    tmp = tempfile.mkdtemp(prefix='p4d_acc_')
    try:
        def case_dur(node):
            drive(node, n=5, contact=True)
            s = node.stats
            check('accounting_duration_exact', s['samples_written'] == 3,
                  s['samples_written'])
            check('accounting_duration_discard',
                  s[DISCARD_DURATION_CUTOFF] >= 1, s[DISCARD_DURATION_CUTOFF])
            ok, s = _conserved(node)
            check('accounting_duration_conserved', ok, s)
        with_collector(tmp, case_dur, duration_sec=0.025)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # E. pending overflow is a terminal discard, never an unaccounted loss
    tmp = tempfile.mkdtemp(prefix='p4d_acc_')
    try:
        def case_ovf(node):
            node.max_pending = 2
            _queue_samples(node, 4)
            node.commit('t')
            s = node.stats
            check('accounting_pending_overflow',
                  s[DISCARD_PENDING_OVERFLOW] >= 1, s[DISCARD_PENDING_OVERFLOW])
            ok, s = _accounted(node)
            check('accounting_overflow_conserved', ok, s)
        with_collector(tmp, case_ovf, watermark_margin_sec=0.0)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # F. watermark-safe finish: not-watermark-ready pending -> discarded_shutdown
    tmp = tempfile.mkdtemp(prefix='p4d_acc_')
    try:
        def case_wm(node):
            node.on_clock(clock_msg(1.0))
            node.on_imu(imu_msg(5.0))
            node.on_command(command_msg(5.0))
            node.on_state(state_msg(5.0))
            node.finish('shutdown')
            s = node.stats
            check('accounting_watermark_safe_finish',
                  s['samples_written'] == 0 and s[DISCARD_SHUTDOWN] == 1,
                  (s['samples_written'], s[DISCARD_SHUTDOWN]))
            ok, s = _conserved(node)
            check('accounting_shutdown_conserved', ok, s)
        with_collector(tmp, case_wm)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # G. rewind clears queued pending into discarded_rewind
    tmp = tempfile.mkdtemp(prefix='p4d_acc_')
    try:
        def case_rw(node):
            _queue_samples(node, 3)
            node.on_clock(clock_msg(0.4))  # backward
            s = node.stats
            check('accounting_rewind_pending',
                  s[DISCARD_REWIND] >= 1 and len(node.pending) == 0,
                  (s[DISCARD_REWIND], len(node.pending)))
            ok, s = _conserved(node)
            check('accounting_rewind_conserved', ok, s)
        with_collector(tmp, case_rw)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


# ------------------------------------------------------------ dataset validator
def write_dataset(root, manifest=None, samples=None, raw_samples=None):
    os.makedirs(root, exist_ok=True)
    with open(os.path.join(root, 'manifest.json'), 'w') as fh:
        json.dump(build_manifest() if manifest is None else manifest, fh, indent=2)
    with open(os.path.join(root, 'samples.jsonl'), 'w') as fh:
        for s in (samples or []):
            fh.write(json.dumps(s, separators=(',', ':')) + '\n')
        if raw_samples:
            fh.write(raw_samples)


def valid_sample(t):
    return {
        'timestamp': t,
        'joint_position': [0.0] * 24,
        'joint_velocity': [0.0] * 24,
        'imu': {'stamp': t, 'orientation': [0.0, 0.0, 0.0, 1.0],
                'angular_velocity': [0.0, 0.0, 0.0],
                'linear_acceleration': [0.0, 0.0, 9.81]},
        'left_contact': False,
        'right_contact': False,
        'command': {'stamp': t, 'position': list(DEFAULT_POSITION)},
    }


def _expect_reject(tmp, name, samples=None, manifest=None, raw_samples=None,
                   expect=None):
    d = os.path.join(tmp, name)
    write_dataset(d, manifest=manifest, samples=samples, raw_samples=raw_samples)
    rep = validator_mod.validate_dataset(d)
    reasons = {e.get('reason') for e in rep['errors']}
    ok = not rep['valid'] and (expect is None or expect in reasons)
    check('validator_rejects_%s' % name, ok, (rep['valid'], sorted(reasons)))


def test_dataset_validator():
    tmp = tempfile.mkdtemp(prefix='p4d_val_')
    try:
        good = os.path.join(tmp, 'good')
        write_dataset(good, samples=[valid_sample(1.00), valid_sample(1.01)])
        rep = validator_mod.validate_dataset(good)
        check('validator_accepts_valid', rep['valid'], rep['errors'][:1])

        extra_dir = os.path.join(tmp, 'extra_file')
        write_dataset(extra_dir, samples=[valid_sample(1.00)])
        open(os.path.join(extra_dir, 'notes.txt'), 'w').close()
        rep_extra = validator_mod.validate_dataset(extra_dir)
        check('validator_rejects_extra_file',
              not rep_extra['valid']
              and 'unexpected_directory_contents' in {e.get('reason')
                                                      for e in rep_extra['errors']},
              rep_extra['errors'][:1])

        missing_field = dict(valid_sample(1.00))
        del missing_field['imu']
        _expect_reject(tmp, 'sample_missing_field', samples=[missing_field],
                       expect='sample_keys_mismatch')

        extra_field = dict(valid_sample(1.00))
        extra_field['body_pose'] = 1
        _expect_reject(tmp, 'sample_extra_field', samples=[extra_field],
                       expect='sample_keys_mismatch')

        bad_contact = valid_sample(1.00)
        bad_contact['left_contact'] = 0
        _expect_reject(tmp, 'int_contact', samples=[bad_contact],
                       expect='left_contact_not_boolean')

        bad_contact_str = valid_sample(1.00)
        bad_contact_str['right_contact'] = 'false'
        _expect_reject(tmp, 'str_contact', samples=[bad_contact_str],
                       expect='right_contact_not_boolean')

        bad_ts = valid_sample(1.00)
        bad_ts['timestamp'] = float('nan')
        _expect_reject(tmp, 'nan_timestamp', samples=[bad_ts], expect='invalid_json')

        _expect_reject(tmp, 'non_monotonic',
                       samples=[valid_sample(1.02), valid_sample(1.01)],
                       expect='timestamp_not_increasing')
        _expect_reject(tmp, 'duplicate_timestamp',
                       samples=[valid_sample(1.00), valid_sample(1.00)],
                       expect='timestamp_not_increasing')
        _expect_reject(tmp, 'zero_timestamp', samples=[valid_sample(0.0)],
                       expect='timestamp_invalid')
        _expect_reject(tmp, 'negative_timestamp', samples=[valid_sample(-1.5)],
                       expect='timestamp_invalid')

        bad_imu = valid_sample(1.00)
        bad_imu['imu']['stamp'] = 1.5
        _expect_reject(tmp, 'imu_delta', samples=[bad_imu],
                       expect='imu_delta_exceeds_window')
        bad_imu_neg = valid_sample(1.00)
        bad_imu_neg['imu']['stamp'] = -0.5
        _expect_reject(tmp, 'imu_negative_stamp', samples=[bad_imu_neg],
                       expect='imu_stamp_invalid')
        bad_imu_keys = valid_sample(1.00)
        bad_imu_keys['imu']['extra'] = 1
        _expect_reject(tmp, 'imu_extra_key', samples=[bad_imu_keys],
                       expect='imu_keys_mismatch')

        bad_cmd = valid_sample(1.00)
        bad_cmd['command']['stamp'] = 0.5
        _expect_reject(tmp, 'command_age', samples=[bad_cmd],
                       expect='command_age_out_of_range')
        bad_cmd_neg = valid_sample(1.00)
        bad_cmd_neg['command']['stamp'] = -0.5
        _expect_reject(tmp, 'command_negative_stamp', samples=[bad_cmd_neg],
                       expect='command_stamp_invalid')
        bad_cmd_dim = valid_sample(1.00)
        bad_cmd_dim['command']['position'] = [0.0] * 23
        _expect_reject(tmp, 'command_dim', samples=[bad_cmd_dim],
                       expect='command_position_invalid')
        bad_cmd_missing = valid_sample(1.00)
        del bad_cmd_missing['command']['stamp']
        _expect_reject(tmp, 'command_missing_key', samples=[bad_cmd_missing],
                       expect='command_keys_mismatch')
        bad_cmd_extra = valid_sample(1.00)
        bad_cmd_extra['command']['extra'] = 1
        _expect_reject(tmp, 'command_extra_key', samples=[bad_cmd_extra],
                       expect='command_keys_mismatch')
        bad_cmd_limit = valid_sample(1.00)
        bad_cmd_limit['command']['position'][0] = 99.0
        _expect_reject(tmp, 'command_limit', samples=[bad_cmd_limit],
                       expect='command_out_of_limit')

        bad_jp = valid_sample(1.00)
        bad_jp['joint_position'] = [0.0] * 23
        _expect_reject(tmp, 'joint_position_dim', samples=[bad_jp],
                       expect='joint_position_invalid')
        bad_jv = valid_sample(1.00)
        bad_jv['joint_velocity'] = [0.0] * 23
        _expect_reject(tmp, 'joint_velocity_dim', samples=[bad_jv],
                       expect='joint_velocity_invalid')
        bad_ori = valid_sample(1.00)
        bad_ori['imu']['orientation'] = [0.0, 0.0, 1.0]
        _expect_reject(tmp, 'imu_orientation_dim', samples=[bad_ori],
                       expect='imu_orientation_invalid')

        _expect_reject(tmp, 'nan_raw', expect='invalid_json',
                       raw_samples='{"timestamp":NaN}\n')
        _expect_reject(tmp, 'inf_raw', expect='invalid_json',
                       raw_samples='{"timestamp":Infinity}\n')
        _expect_reject(tmp, 'empty_line', samples=[valid_sample(1.00)],
                       raw_samples='\n')
        _expect_reject(tmp, 'truncated_line', samples=[valid_sample(1.00)],
                       raw_samples='{"timestamp":1.02}')

        # ---- manifest mutations ----
        for key in list(build_manifest().keys()):
            m = build_manifest()
            del m[key]
            _expect_reject(tmp, 'manifest_missing_%s' % key, samples=[valid_sample(1.0)],
                           manifest=m, expect='manifest_missing_key')
        m = build_manifest()
        m['episode_id'] = 'x'
        _expect_reject(tmp, 'manifest_extra_key', samples=[valid_sample(1.0)],
                       manifest=m, expect='manifest_extra_key')
        m = build_manifest()
        m['schema_version'] = '1.0.0'
        _expect_reject(tmp, 'manifest_schema_version', samples=[valid_sample(1.0)],
                       manifest=m, expect='manifest_value_mismatch')
        for key in ('main_clock', 'imu', 'contact', 'command'):
            m = build_manifest()
            m['sync_policy'][key] = 'wrong'
            _expect_reject(tmp, 'sync_policy_value_%s' % key,
                           samples=[valid_sample(1.0)], manifest=m,
                           expect='manifest_value_mismatch')
            m = build_manifest()
            del m['sync_policy'][key]
            _expect_reject(tmp, 'sync_policy_missing_%s' % key,
                           samples=[valid_sample(1.0)], manifest=m,
                           expect='manifest_missing_key')
        m = build_manifest()
        m['sync_policy']['extra'] = 'x'
        _expect_reject(tmp, 'sync_policy_extra', samples=[valid_sample(1.0)],
                       manifest=m, expect='manifest_extra_key')
        for key in ('imu_nearest_max', 'contact_freshness_max', 'command_hold_max'):
            m = build_manifest()
            m['sync_tolerance_sec'][key] = 9.9
            _expect_reject(tmp, 'tolerance_%s' % key, samples=[valid_sample(1.0)],
                           manifest=m, expect='manifest_value_mismatch')
        m = build_manifest()
        m['source_topics']['imu'] = '/wrong'
        _expect_reject(tmp, 'source_topic', samples=[valid_sample(1.0)],
                       manifest=m, expect='manifest_value_mismatch')
        m = build_manifest()
        m['source_message_types']['command'] = 'std_msgs/msg/Int'
        _expect_reject(tmp, 'source_type', samples=[valid_sample(1.0)],
                       manifest=m, expect='manifest_value_mismatch')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_validator_independence():
    with open(_VALIDATOR_PATH, 'r', encoding='utf-8') as fh:
        src = fh.read()
    check('validator_no_writer_import',
          'nao_trajectory_schema' not in src and 'build_manifest' not in src)
    check('validator_module_has_no_writer_symbols',
          not hasattr(validator_mod, 'build_manifest')
          and not hasattr(validator_mod, 'SAMPLE_KEYS_WRITER'))
    check('validator_golden_schema_version',
          validator_mod.GOLDEN_MANIFEST['schema_version'] == '1.0.1')
    check('validator_golden_sync_policy',
          validator_mod.GOLDEN_MANIFEST['sync_policy'] == {
              'main_clock': 'new_joint_states_message',
              'imu': 'nearest_neighbor_sim_time',
              'contact': 'clock_freshness_zero_order_hold',
              'command': 'hold_last_causal_bounded'})
    # A writer-side manifest change must not silently move the validator target.
    tmp = tempfile.mkdtemp(prefix='p4d_indep_')
    try:
        mutated = build_manifest()
        mutated['schema_version'] = '1.0.0'
        d = os.path.join(tmp, 'writer_10')
        write_dataset(d, manifest=mutated, samples=[valid_sample(1.0)])
        check('validator_not_auto_synced_with_writer',
              not validator_mod.validate_dataset(d)['valid'])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('Task 4 Phase 4D/4E-0 OFFLINE tests (no simulation)')
    test_command_rules()
    test_strict_json()
    test_manifest()
    test_negative_timestamps()
    test_collector_offline()
    test_accounting()
    test_dataset_validator()
    test_validator_independence()
    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\noffline tests: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w') as fh:
            json.dump({'offline': True, 'passed': passed, 'total': total,
                       'results': RESULTS}, fh, indent=2, sort_keys=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
