#!/usr/bin/env python3
"""Task 4 Phase 4E-0 INDEPENDENT golden dataset validator (schema v1.0.1).

Reads the FINAL ``manifest.json`` and ``samples.jsonl`` of a trajectory dataset
directory and validates them against the frozen schema **without importing any
writer/collector module**. The expected manifest and all key sets / thresholds
below are a deliberate, hand-derived duplication of
``docs/Task4_Trajectory_Schema.md`` (v1.0.1) so that a bug in the writer cannot
be silently accepted by the validator. This duplication is the independent
verification boundary; do NOT refactor it back into a shared module.

Usage:
    python3 tools/validate_task4_phase4d_dataset.py <dataset_dir> [--json-out F]

Exit code 0 = valid; 1 = invalid (with stable reason codes and line numbers).
"""

import argparse
import json
import math
import os
import sys

MANIFEST_FILE = 'manifest.json'
SAMPLES_FILE = 'samples.jsonl'

# ---------------------------------------------------------------------------
# Golden contract (derived from docs/Task4_Trajectory_Schema.md v1.0.1 by hand)
# ---------------------------------------------------------------------------
SCHEMA_VERSION = '1.0.1'

JOINT_ORDER = [
    'HeadYaw', 'HeadPitch',
    'LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll',
    'LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw',
    'RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll',
    'RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw',
]

# Frozen canonical SDF joint limits in rad (nao_simple.sdf).
JOINT_LIMITS = {
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

IMU_FRAME = 'ImuTorsoAccelerometer_frame'
IMU_NEAREST_MAX = 0.01
CONTACT_FRESHNESS_MAX = 0.05
COMMAND_HOLD_MAX = 0.05

SAMPLE_KEYS = ['timestamp', 'joint_position', 'joint_velocity', 'imu',
               'left_contact', 'right_contact', 'command']
IMU_KEYS = ['stamp', 'orientation', 'angular_velocity', 'linear_acceleration']
COMMAND_KEYS = ['stamp', 'position']

GOLDEN_MANIFEST = {
    'schema_name': 'nao_gazebo_trajectory',
    'schema_version': SCHEMA_VERSION,
    'serialization_format': 'JSONL + manifest JSON',
    'robot': 'nao',
    'time_source': 'gazebo_simulation_time',
    'time_unit': 's',
    'joint_order': list(JOINT_ORDER),
    'joint_position_unit': 'rad',
    'joint_velocity_unit': 'rad/s',
    'imu': {
        'frame_id': IMU_FRAME,
        'orientation_representation': 'quaternion_xyzw',
        'orientation_frame': IMU_FRAME,
        'orientation_unit_norm': True,
        'angular_velocity_unit': 'rad/s',
        'angular_velocity_frame': IMU_FRAME,
        'linear_acceleration_unit': 'm/s^2',
        'linear_acceleration_frame': IMU_FRAME,
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


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------
def _is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _finite(value):
    return _is_number(value) and math.isfinite(float(value))


def _reject_constant(name):
    raise ValueError('non-finite JSON constant: %s' % name)


def _deep_diff(want, got, path=''):
    """Yield (reason, path, expected, actual) for the first structural diff."""
    if isinstance(want, dict):
        if not isinstance(got, dict):
            yield ('manifest_not_object', path or '<root>', 'object', type(got).__name__)
            return
        for key in got:
            if key not in want:
                yield ('manifest_extra_key', '%s/%s' % (path, key), None, 'present')
                return
        for key in want:
            if key not in got:
                yield ('manifest_missing_key', '%s/%s' % (path, key), 'present', None)
                return
            for diff in _deep_diff(want[key], got[key], '%s/%s' % (path, key)):
                yield diff
                return
        return
    if isinstance(want, list):
        if not isinstance(got, list) or len(want) != len(got):
            yield ('manifest_value_mismatch', path, want, got)
            return
        for i, (w, g) in enumerate(zip(want, got)):
            for diff in _deep_diff(w, g, '%s[%d]' % (path, i)):
                yield diff
                return
        return
    if want != got:
        yield ('manifest_value_mismatch', path, want, got)


def validate_manifest(path, errors):
    try:
        with open(path, 'r', encoding='utf-8') as fh:
            got = json.load(fh, parse_constant=_reject_constant)
    except (OSError, ValueError) as exc:
        errors.append({'where': 'manifest', 'reason': 'manifest_unreadable',
                       'detail': str(exc)})
        return
    if got == GOLDEN_MANIFEST:
        return
    for reason, where, expected, actual in _deep_diff(GOLDEN_MANIFEST, got):
        errors.append({'where': 'manifest', 'reason': reason, 'path': where,
                       'expected': expected, 'got': actual})
        return


def validate_sample(obj, lineno, errors, prev_ts):
    def fail(reason, detail=None):
        errors.append({'where': 'sample', 'line': lineno, 'reason': reason,
                       'detail': detail})

    if not isinstance(obj, dict):
        fail('sample_not_object')
        return prev_ts
    if list(obj.keys()) != SAMPLE_KEYS:
        fail('sample_keys_mismatch',
             {'extra': [k for k in obj if k not in SAMPLE_KEYS],
              'missing': [k for k in SAMPLE_KEYS if k not in obj]})
        return prev_ts

    ts = obj.get('timestamp')
    if not _finite(ts) or float(ts) <= 0.0:
        fail('timestamp_invalid', ts)
        return prev_ts
    ts = float(ts)
    if prev_ts is not None and ts <= prev_ts:
        fail('timestamp_not_increasing', {'prev': prev_ts, 'got': ts})
        return ts

    jp = obj.get('joint_position')
    if not isinstance(jp, list) or len(jp) != 24 or not all(_finite(v) for v in jp):
        fail('joint_position_invalid')
        return ts
    jv = obj.get('joint_velocity')
    if not isinstance(jv, list) or len(jv) != 24 or not all(_finite(v) for v in jv):
        fail('joint_velocity_invalid')
        return ts

    imu = obj.get('imu')
    if not isinstance(imu, dict) or list(imu.keys()) != IMU_KEYS:
        fail('imu_keys_mismatch', None if not isinstance(imu, dict) else list(imu.keys()))
        return ts
    istamp = imu.get('stamp')
    if not _finite(istamp) or float(istamp) <= 0.0:
        fail('imu_stamp_invalid', istamp)
        return ts
    if abs(ts - float(istamp)) > IMU_NEAREST_MAX + 1e-9:
        fail('imu_delta_exceeds_window', {'timestamp': ts, 'imu_stamp': istamp})
        return ts
    ori = imu.get('orientation')
    if not isinstance(ori, list) or len(ori) != 4 or not all(_finite(v) for v in ori):
        fail('imu_orientation_invalid')
        return ts
    norm = math.sqrt(sum(float(v) * float(v) for v in ori))
    if abs(norm - 1.0) > 1e-6:
        fail('imu_quat_norm', norm)
        return ts
    for key, dim in (('angular_velocity', 3), ('linear_acceleration', 3)):
        val = imu.get(key)
        if not isinstance(val, list) or len(val) != dim \
                or not all(_finite(v) for v in val):
            fail('imu_%s_invalid' % key)
            return ts

    for key in ('left_contact', 'right_contact'):
        if not isinstance(obj.get(key), bool):
            fail('%s_not_boolean' % key, obj.get(key))
            return ts

    cmd = obj.get('command')
    if not isinstance(cmd, dict) or list(cmd.keys()) != COMMAND_KEYS:
        fail('command_keys_mismatch', None if not isinstance(cmd, dict) else list(cmd.keys()))
        return ts
    cstamp = cmd.get('stamp')
    if not _finite(cstamp) or float(cstamp) <= 0.0:
        fail('command_stamp_invalid', cstamp)
        return ts
    age = ts - float(cstamp)
    if age < -1e-9 or age > COMMAND_HOLD_MAX + 1e-9:
        fail('command_age_out_of_range', {'age': age, 'command_stamp': cstamp})
        return ts
    cpos = cmd.get('position')
    if not isinstance(cpos, list) or len(cpos) != 24 or not all(_finite(v) for v in cpos):
        fail('command_position_invalid')
        return ts
    for name, value in zip(JOINT_ORDER, cpos):
        lower, upper = JOINT_LIMITS[name]
        if float(value) < lower or float(value) > upper:
            fail('command_out_of_limit', {'joint': name, 'value': value,
                                          'limit': [lower, upper]})
            return ts
    return ts


def validate_dataset(dataset_dir):
    report = {'dataset_dir': dataset_dir, 'valid': True, 'errors': [],
              'samples': 0, 'manifest_ok': False, 'samples_ok': False}
    errors = report['errors']
    if not os.path.isdir(dataset_dir):
        errors.append({'where': 'dataset', 'reason': 'not_a_directory'})
        report['valid'] = False
        return report
    entries = sorted(os.listdir(dataset_dir))
    if entries != [MANIFEST_FILE, SAMPLES_FILE]:
        errors.append({'where': 'dataset', 'reason': 'unexpected_directory_contents',
                       'entries': entries})
        report['valid'] = False
        return report

    validate_manifest(os.path.join(dataset_dir, MANIFEST_FILE), errors)
    report['manifest_ok'] = not errors

    samples_path = os.path.join(dataset_dir, SAMPLES_FILE)
    prev_ts = None
    n = 0
    try:
        fh = open(samples_path, 'r', encoding='utf-8')
    except OSError as exc:
        errors.append({'where': 'samples', 'reason': 'samples_unreadable',
                       'detail': str(exc)})
        report['valid'] = False
        return report
    with fh:
        for lineno, line in enumerate(fh, start=1):
            if not line.endswith('\n'):
                errors.append({'where': 'sample', 'line': lineno,
                               'reason': 'truncated_line'})
                continue
            if not line.strip():
                errors.append({'where': 'sample', 'line': lineno, 'reason': 'empty_line'})
                continue
            try:
                obj = json.loads(line, parse_constant=_reject_constant)
            except ValueError as exc:
                errors.append({'where': 'sample', 'line': lineno,
                               'reason': 'invalid_json', 'detail': str(exc)})
                continue
            try:
                prev_ts = validate_sample(obj, lineno, errors, prev_ts)
            except ValueError as exc:
                errors.append({'where': 'sample', 'line': lineno,
                               'reason': 'non_finite_value', 'detail': str(exc)})
            n += 1
    report['samples'] = n
    report['samples_ok'] = not any(
        e.get('where') in ('sample', 'samples') for e in errors)
    report['valid'] = report['manifest_ok'] and report['samples_ok'] and not errors
    return report


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dataset_dir')
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    report = validate_dataset(os.path.abspath(args.dataset_dir))
    text = json.dumps(report, indent=2, sort_keys=True, allow_nan=False)
    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as fh:
            fh.write(text)
    print(text)
    return 0 if report['valid'] else 1


if __name__ == '__main__':
    sys.exit(main())
