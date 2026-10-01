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
Phase 2E — unified discrete labels, continuous metrics and an explainable
quality score.

Pure, ROS-free, deterministic, read-only with respect to the dataset. It reads
a Phase 2A dataset (index.jsonl + per-episode metadata.json + frozen
samples.jsonl) and produces a quality report. It NEVER writes samples/metadata/
index; it never uses ``cmd_vel``; missing metrics stay ``null`` with an explicit
missing reason -- never 0 or a fabricated default.

``quality_score`` is for ranking / cleaning / statistics / replay sampling ONLY.
It is explicitly NOT a Dreamer reward and does not replace success/failure.
"""

import argparse
import json
import math
import os
import sys
import tempfile

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
_SCRIPTS = os.path.normpath(os.path.join(_TOOLS, '..', 'src', 'nao_gz_sim',
                                         'scripts'))
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)
import phase2a_episode_dataset as ep  # noqa: E402

try:
    from nao_trajectory_schema import COMMAND_HOLD_MAX as _COMMAND_HOLD_MAX
except Exception:  # pragma: no cover - fallback constant
    _COMMAND_HOLD_MAX = 0.05

QUALITY_SCORE_VERSION = '1.0.0'
QUALITY_FAILURE_REASONS = (
    'startup_instability', 'excessive_step_length', 'excessive_frequency',
    'insufficient_swing_height', 'excessive_torso_pitch', 'lateral_instability',
    'contact_anomaly', 'tracking_error', 'fall', 'timeout_or_no_progress')

# Documented, fixed, reproducible normalization constants (no dynamic thresholds).
DEFAULTS = {
    'TILT_BAD_DEG': 30.0,          # >= this mean tilt -> stability 0
    'ENERGY_REF_QDOT': 2.0,        # rad^2/s^2 reference (provisional)
    'COMMAND_HOLD_MAX': float(_COMMAND_HOLD_MAX),
    'SPEED_REF_MPS': 0.05,         # documented, but speed is NOT scored (see policy)
}
BASE_WEIGHTS = {'speed': 0.30, 'survival': 0.35, 'stability': 0.25,
                'energy': 0.10}
# Speed is excluded from the score by policy: the frozen system has no target
# speed / tracking semantics, so a speed score would be meaningless (and letting
# a near-zero speed be "good" or "bad" would hide the current in-place gait).
SPEED_POLICY = 'not_scored_no_target_speed_semantics'


class Phase2EError(Exception):
    pass


# ---------------------------------------------------------------------------
# data dictionary
# ---------------------------------------------------------------------------
FIELD_DICTIONARY = {
    'success': {'type': 'bool', 'unit': None,
                'source': 'Phase 2D classification', 'window': 'episode',
                'missing': 'not allowed', 'in_score': False, 'in_filter': True},
    'usable': {'type': 'bool', 'unit': None,
               'source': 'sample completeness (metadata schema >=1.1.0)',
               'window': 'episode', 'missing': 'not allowed',
               'in_score': False, 'in_filter': True},
    'fell': {'type': 'bool', 'unit': None, 'source': 'body trace z<=0.25',
             'window': 'steady window', 'missing': 'null (undetermined)',
             'in_score': True, 'in_filter': True},
    'fall_time': {'type': 'float|null', 'unit': 's',
                  'source': 'body trace first z<=0.25', 'window': 'steady window',
                  'missing': '-1.0 means no fall', 'in_score': True,
                  'in_filter': True},
    'failure_reason': {'type': 'str|null', 'unit': None,
                       'source': 'Phase 2D classification', 'window': 'episode',
                       'missing': 'null on success', 'in_score': False,
                       'in_filter': True},
    'mean_speed_x_mps': {'type': 'float|null', 'unit': 'm/s',
                         'source': 'body_pose trace body_dx/window',
                         'window': 'steady gait window',
                         'missing': 'metadata null -> not scored',
                         'in_score': False, 'in_filter': False},
    'speed_tracking_error': {'type': 'float|null', 'unit': 'm/s',
                             'source': 'none (no target-speed semantics)',
                             'window': 'n/a',
                             'missing': 'always null in current system',
                             'in_score': False, 'in_filter': False},
    'max_torso_tilt_deg': {'type': 'float|null', 'unit': 'deg',
                           'source': 'metadata (Phase 4E body-attitude: '
                                     'max(|roll|,|pitch|) from body_pose trace)',
                           'window': 'steady gait window',
                           'missing': 'null if absent (different definition '
                                      'from imu_tilt_deg_*)',
                           'note': 'body-attitude definition; NOT the IMU '
                                   'z-axis tilt and not interchangeable with '
                                   'imu_tilt_deg_*',
                           'in_score': False, 'in_filter': True},
    'imu_tilt_deg_max': {'type': 'float|null', 'unit': 'deg',
                         'source': 'IMU orientation recomputed from samples '
                                   '(z-axis tilt)',
                         'window': 'all samples',
                         'missing': 'null if no valid quaternion',
                         'note': 'IMU z-axis tilt; distinct from the '
                                 'body-attitude max_torso_tilt_deg',
                         'in_score': False, 'in_filter': False},
    'imu_tilt_deg_mean': {'type': 'float|null', 'unit': 'deg',
                          'source': 'IMU orientation recomputed from samples '
                                    '(z-axis tilt)',
                          'window': 'all samples',
                          'missing': 'null if no valid quaternion',
                          'note': 'the concept "mean_torso_tilt_deg" maps to '
                                  'this field, but its definition is the IMU '
                                  'z-axis tilt (not body-attitude)',
                          'in_score': True, 'in_filter': False},
    'energy_qdot': {'type': 'float|null', 'unit': 'rad^2/s^2',
                    'source': 'samples joint_velocity mean sum(dq^2)',
                    'window': 'all samples', 'missing': 'null (never 0)',
                    'in_score': True, 'in_filter': False},
    'contact_alternation': {'type': 'object|null', 'unit': None,
                            'source': 'samples left/right_contact transitions',
                            'window': 'all samples',
                            'missing': 'null if no contact field',
                            'note': 'diagnostic only: no agreed anomaly '
                                    'threshold yet, so it is NOT scored',
                            'in_score': False, 'in_filter': False},
    'command_coverage': {'type': 'float|null', 'unit': 'fraction',
                         'source': 'samples command stamp within hold',
                         'window': 'all samples',
                         'missing': 'null if no command field',
                         'in_score': False, 'in_filter': False},
}


def _finite(v):
    """True only for a finite ``int``/``float`` (``bool`` is never a number)."""
    if not isinstance(v, (int, float)) or isinstance(v, bool):
        return False
    try:
        return math.isfinite(float(v))
    except (OverflowError, ValueError):
        return False


def _clamp(v, lo=0.0, hi=1.0):
    """Clamp a **finite** value; non-finite input is an explicit failure.

    ``min``/``max`` have a NaN ordering trap (``min(hi, nan) -> hi``), so a
    silent clamp of NaN/Infinity would fabricate a "perfect" component. We
    refuse to compute instead of masking corrupt data.
    """
    if not _finite(v):
        raise Phase2EError('clamp received a non-finite value: %r' % (v,))
    return max(lo, min(hi, v))


def _round(v, nd=6):
    if v is None:
        return None
    if not _finite(v):
        raise Phase2EError('round received a non-finite value: %r' % (v,))
    return round(float(v), nd)


# ---------------------------------------------------------------------------
# per-sample metrics from the frozen samples.jsonl
# ---------------------------------------------------------------------------
def _reject_json_constant(name):
    """Reject the non-standard JSON ``NaN``/``Infinity`` extension literals."""
    raise ValueError('non-finite JSON constant %r' % (name,))


def _strict_parse_float(text):
    """Reject JSON numbers that overflow to ``inf`` (e.g. ``1e999``)."""
    value = float(text)
    if not math.isfinite(value):
        raise ValueError('non-finite JSON number %r' % (text,))
    return value


def iter_samples(samples_path):
    """
    Yield sample objects, rejecting malformed lines and **any** non-finite
    numeric literal. ``json.loads`` accepts ``NaN``/``Infinity`` by default, so
    we pass strict ``parse_constant``/``parse_float`` hooks; errors carry the
    file and line number for auditing.
    """
    with open(samples_path, 'r', encoding='utf-8') as fh:
        for lineno, line in enumerate(fh, start=1):
            if not line.endswith('\n'):
                raise Phase2EError('%s:%d truncated line' % (samples_path, lineno))
            if not line.strip():
                raise Phase2EError('%s:%d empty line' % (samples_path, lineno))
            try:
                rec = json.loads(line, parse_constant=_reject_json_constant,
                                 parse_float=_strict_parse_float)
            except Phase2EError:
                raise
            except ValueError as exc:
                raise Phase2EError('%s:%d invalid sample JSON: %s'
                                   % (samples_path, lineno, exc))
            if not isinstance(rec, dict):
                raise Phase2EError('%s:%d sample is not a JSON object'
                                   % (samples_path, lineno))
            yield rec


def imu_tilt_deg(orientation):
    """Torso tilt (deg) = angle between IMU z-axis and world z, from quaternion.

    A malformed (wrong length / zero norm) quaternion yields ``None``
    (missing), but a non-finite or non-numeric component is a hard error --
    otherwise ``NaN`` silently looks like a perfectly upright torso.
    """
    if not isinstance(orientation, list) or len(orientation) != 4:
        return None
    if not all(_finite(v) for v in orientation):
        raise Phase2EError(
            'imu orientation has a non-finite or non-numeric component: %r'
            % (orientation,))
    x, y, z, w = (float(v) for v in orientation)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm):
        raise Phase2EError(
            'imu orientation magnitude overflow: %r' % (orientation,))
    if norm < 1e-9:
        return None
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    # third column z-component of the rotation matrix = 1 - 2(x^2 + y^2)
    r33 = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(_clamp(r33, -1.0, 1.0)))


def contact_alternation(states, transition_debounce=0):
    """Return a structured alternation summary from a list of support states."""
    n = len(states)
    if n == 0:
        return None
    transitions = 0
    alternations = 0
    double = 0
    flight = 0
    for i, s in enumerate(states):
        if s == 'D':
            double += 1
        elif s == 'N':
            flight += 1
        if i > 0 and states[i] != states[i - 1]:
            transitions += 1
            if {states[i - 1], states[i]} == {'L', 'R'}:
                alternations += 1
    return {
        'n': n,
        'transitions': transitions,
        'alternation_count': alternations,
        'alternation_fraction': _round(alternations / max(1, n - 1)),
        'double_support_fraction': _round(double / n),
        'flight_fraction': _round(flight / n),
    }


def compute_sample_metrics(samples_path):
    """Stream the frozen samples once and return measured metrics."""
    tilts = []
    energy_sum = 0.0
    n = 0
    states = []
    command_valid = 0
    contacts_present = True
    command_present = True
    for lineno, rec in enumerate(iter_samples(samples_path), start=1):
        ctx = '%s:%d' % (samples_path, lineno)
        n += 1
        dq = rec.get('joint_velocity')
        if not isinstance(dq, list):
            raise Phase2EError('%s sample missing joint_velocity' % ctx)
        for i, v in enumerate(dq):
            if not _finite(v):
                raise Phase2EError(
                    '%s joint_velocity[%d] is not finite: %r' % (ctx, i, v))
            term = float(v) * float(v)
            if not math.isfinite(term):
                raise Phase2EError(
                    '%s joint_velocity[%d] square overflows to a non-finite '
                    'value' % (ctx, i))
            energy_sum += term
        if not math.isfinite(energy_sum):
            raise Phase2EError('%s cumulative energy is non-finite' % ctx)
        imu = rec.get('imu') or {}
        if not isinstance(imu, dict):
            raise Phase2EError('%s imu is not an object' % ctx)
        orientation = imu.get('orientation')
        if orientation is not None:
            try:
                tilt = imu_tilt_deg(orientation)
            except Phase2EError as exc:
                raise Phase2EError('%s imu.orientation: %s' % (ctx, exc))
            if tilt is not None:
                tilts.append(tilt)
        if 'left_contact' in rec and 'right_contact' in rec:
            left, right = rec['left_contact'], rec['right_contact']
            if not isinstance(left, bool) or not isinstance(right, bool):
                raise Phase2EError(
                    '%s left/right_contact must be booleans' % ctx)
            states.append('L' if left and not right else 'R' if right and not left
                          else 'D' if left and right else 'N')
        else:
            contacts_present = False
        ts = rec.get('timestamp')
        if ts is not None and not _finite(ts):
            raise Phase2EError('%s timestamp is not finite: %r' % (ctx, ts))
        cmd = rec.get('command')
        if cmd is None:
            command_present = False
        elif not isinstance(cmd, dict):
            raise Phase2EError('%s command is not an object' % ctx)
        else:
            stamp = cmd.get('stamp')
            if not _finite(stamp):
                raise Phase2EError(
                    '%s command.stamp is not finite: %r' % (ctx, stamp))
            position = cmd.get('position')
            if not isinstance(position, list) or len(position) != 24:
                command_present = False
            else:
                for i, v in enumerate(position):
                    if not _finite(v):
                        raise Phase2EError(
                            '%s command.position[%d] is not finite: %r'
                            % (ctx, i, v))
                if not _finite(ts):
                    command_present = False
                else:
                    age = float(ts) - float(stamp)
                    if -1e-9 <= age <= DEFAULTS['COMMAND_HOLD_MAX'] + 1e-9:
                        command_valid += 1
    out = {
        'sample_count': n,
        'energy_qdot': _round(energy_sum / n) if n else None,
        # These are the IMU z-axis tilts recomputed from the samples; the field
        # names say so explicitly (they are NOT the body-attitude
        # ``max_torso_tilt_deg`` taken from metadata).
        'imu_tilt_deg_max': _round(max(tilts)) if tilts else None,
        'imu_tilt_deg_mean': _round(sum(tilts) / len(tilts)) if tilts else None,
        'contact_alternation': contact_alternation(states) if contacts_present
        else None,
        'command_coverage': _round(command_valid / n) if (n and command_present)
        else None,
    }
    return out


# ---------------------------------------------------------------------------
# quality score
# ---------------------------------------------------------------------------
def _survival_component(fell, fall_time, duration):
    if fell is None:
        return None
    if not isinstance(fell, bool):
        raise Phase2EError('fell must be a boolean or null: %r' % (fell,))
    if not fell:
        return 1.0
    if not _finite(fall_time):
        raise Phase2EError(
            'fall_time must be finite when fell=true: %r' % (fall_time,))
    if not _finite(duration) or float(duration) <= 0:
        raise Phase2EError(
            'duration must be a positive finite number when fell=true: %r'
            % (duration,))
    return _round(_clamp(float(fall_time) / float(duration)))


def _stability_component(mean_tilt, contact):
    # Contact alternation is reported as a diagnostic but is NOT scored: no
    # contact-anomaly threshold is agreed yet, so scoring it would be arbitrary.
    if mean_tilt is None:
        return None
    if not _finite(mean_tilt):
        raise Phase2EError(
            'mean_tilt must be finite or null: %r' % (mean_tilt,))
    return _round(_clamp(1.0 - float(mean_tilt) / DEFAULTS['TILT_BAD_DEG']))


def _energy_component(energy):
    if energy is None:
        return None
    if not _finite(energy):
        raise Phase2EError('energy must be finite or null: %r' % (energy,))
    return _round(_clamp(1.0 - float(energy) / DEFAULTS['ENERGY_REF_QDOT']))


def compute_quality_score(fell, fall_time, duration, mean_tilt, energy,
                          contact):
    """
    Deterministic score with renormalized weights + status + missing list.

    speed is intentionally not scored (no target-speed semantics); when
    survival/stability/energy are missing the weights are renormalized over the
    available components and ``quality_score_status='partial'``.

    Non-finite inputs are rejected here (the API entry point) so that a corrupt
    episode can never receive a finite, high score or a success-shaped output.
    """
    if fell is not None and not isinstance(fell, bool):
        raise Phase2EError('fell must be a boolean or null: %r' % (fell,))
    for name, value in (('fall_time', fall_time), ('duration', duration),
                        ('mean_tilt', mean_tilt), ('energy', energy)):
        if value is not None and not _finite(value):
            raise Phase2EError(
                '%s must be finite or null: %r' % (name, value))
    components = {
        'speed': None,
        'survival': _survival_component(fell, fall_time, duration),
        'stability': _stability_component(mean_tilt, contact),
        'energy': _energy_component(energy),
    }
    missing = ['speed']
    active = {k: v for k, v in components.items()
              if k != 'speed' and v is not None}
    for k in ('survival', 'stability', 'energy'):
        if components[k] is None:
            missing.append(k)
    if 'survival' not in active:
        return {
            'quality_score': None,
            'quality_score_version': QUALITY_SCORE_VERSION,
            'quality_score_status': 'insufficient',
            'quality_score_components': {k: _round(v) for k, v in components.items()},
            'quality_score_weights': {},
            'quality_score_missing': sorted(missing),
            'quality_score_notes': {'speed': SPEED_POLICY},
        }
    base = {k: BASE_WEIGHTS[k] for k in active}
    total = sum(base.values())
    weights = {k: _round(v / total) for k, v in base.items()}
    score = sum(active[k] * (base[k] / total) for k in active)
    status = 'complete' if not missing else 'partial'
    return {
        'quality_score': _round(score),
        'quality_score_version': QUALITY_SCORE_VERSION,
        'quality_score_status': status,
        'quality_score_components': {k: _round(v) for k, v in components.items()},
        'quality_score_weights': weights,
        'quality_score_missing': sorted(missing),
        'quality_score_notes': {'speed': SPEED_POLICY},
    }


# ---------------------------------------------------------------------------
# episode + dataset
# ---------------------------------------------------------------------------
def _finite_metadata_field(meta, field, warnings):
    """Return a finite metadata scalar or ``None`` + an explicit warning.

    A present-but-non-finite optional metric is recorded as invalid (never
    silently dropped, and never leaked into JSON where it would corrupt the
    report).
    """
    value = meta.get(field)
    if value is None:
        return None
    if not _finite(value):
        warnings.append({'field': field, 'reason': 'non_finite_metadata'})
        return None
    return value


def evaluate_episode(meta, samples_path):
    """Return the Phase 2E fields for one episode (read-only)."""
    sample_metrics = compute_sample_metrics(samples_path)
    discrete = {
        'success': meta.get('success'),
        'usable': meta.get('usable'),
        'fell': meta.get('fell'),
        'fall_time': meta.get('fall_time'),
        'failure_reason': meta.get('failure_reason'),
    }
    if discrete['fell'] is not None and not isinstance(discrete['fell'], bool):
        raise Phase2EError('fell must be a boolean or null: %r'
                           % (discrete['fell'],))
    if discrete['fall_time'] is not None \
            and not _finite(discrete['fall_time']):
        raise Phase2EError('fall_time must be finite or null: %r'
                           % (discrete['fall_time'],))
    warnings = []
    if discrete['usable'] is None:
        # Legacy (schema 1.0.0) metadata may not carry ``usable``. We never
        # backfill it from success or guess a value: it stays null (so it is
        # not counted as usable) and the ambiguity is recorded explicitly.
        warnings.append({'field': 'usable',
                         'reason': 'legacy_usable_unknown'})
    continuous = {
        'mean_speed_x_mps': _finite_metadata_field(
            meta, 'mean_speed_x_mps', warnings),
        'speed_tracking_error': None,
        # body-attitude definition, taken from metadata (different from IMU z-tilt)
        'max_torso_tilt_deg': _finite_metadata_field(
            meta, 'max_torso_tilt_deg', warnings),
        # IMU z-axis tilt, recomputed from the frozen samples
        'imu_tilt_deg_max': sample_metrics['imu_tilt_deg_max'],
        'imu_tilt_deg_mean': sample_metrics['imu_tilt_deg_mean'],
        'energy_qdot': sample_metrics['energy_qdot'],
        'contact_alternation': sample_metrics['contact_alternation'],
        'command_coverage': sample_metrics['command_coverage'],
    }
    meta_energy = meta.get('energy_qdot')
    if meta_energy is not None and not _finite(meta_energy):
        warnings.append({'field': 'energy_qdot',
                         'reason': 'non_finite_metadata'})
    elif _finite(meta_energy) and continuous['energy_qdot'] is not None \
            and abs(float(meta_energy) - continuous['energy_qdot']) > 1e-6:
        warnings.append({
            'field': 'energy_qdot', 'reason': 'metadata_vs_sample_mismatch',
            'metadata': float(meta_energy),
            'recomputed': continuous['energy_qdot']})
    if meta.get('max_torso_tilt_deg') is None:
        warnings.append({'field': 'max_torso_tilt_deg',
                         'reason': 'metadata_missing'})
    score = compute_quality_score(
        discrete['fell'], discrete['fall_time'], meta.get('duration_sec'),
        continuous['imu_tilt_deg_mean'], continuous['energy_qdot'],
        continuous['contact_alternation'])
    return {
        'discrete': discrete,
        'continuous': continuous,
        'quality': score,
        'sample_metrics': {'sample_count': sample_metrics['sample_count']},
        'data_quality_warnings': warnings,
    }


def evaluate_dataset(dataset_dir, phases=None):
    """Evaluate every episode of a dataset; returns (episodes, report)."""
    ds = ep.EpisodeDataset(dataset_dir)
    check = ds.validate()
    if not check['valid']:
        raise Phase2EError('dataset invalid: %s' % check['errors'][:3])
    index = ds.load_index()
    per_episode = []
    warnings = []
    for row in index:
        eid = row['episode_id']
        meta = ds.load_metadata(eid)
        try:
            ev = evaluate_episode(meta, ds.samples_path(eid))
        except Phase2EError as exc:
            raise Phase2EError('episode %s: %s' % (eid, exc))
        entry = {
            'episode_id': eid,
            'split': row['split'],
            'candidate_id': meta.get('baseline_config'),
            'seed': meta.get('seed'),
            **ev['discrete'],
            **ev['continuous'],
            'quality_score': ev['quality']['quality_score'],
            'quality_score_version': ev['quality']['quality_score_version'],
            'quality_score_status': ev['quality']['quality_score_status'],
            'quality_score_components': ev['quality']['quality_score_components'],
            'quality_score_weights': ev['quality']['quality_score_weights'],
            'quality_score_missing': ev['quality']['quality_score_missing'],
            'quality_score_notes': ev['quality']['quality_score_notes'],
            'sample_count': ev['sample_metrics']['sample_count'],
            'data_quality_warnings': ev['data_quality_warnings'],
        }
        per_episode.append(entry)
        for w in ev['data_quality_warnings']:
            warnings.append(dict(w, episode_id=eid))
    report = {
        'dataset_dir': os.path.abspath(dataset_dir),
        'quality_score_version': QUALITY_SCORE_VERSION,
        'field_dictionary': FIELD_DICTIONARY,
        'defaults': DEFAULTS,
        'base_weights': BASE_WEIGHTS,
        'per_episode': per_episode,
        'aggregate': aggregate(per_episode),
        'data_quality_warnings': warnings,
        'usage': {
            'allowed': ['ranking', 'cleaning', 'statistics', 'replay_sampling'],
            'forbidden': ['sole_dreamer_reward', 'override_success_or_failure',
                          'delete_failed_samples', 'forward_walk_probability'],
        },
        'missing_policy': 'renormalize + quality_score_status + components',
    }
    problems = check_aggregate_consistency(report)
    if problems:
        raise Phase2EError('aggregate inconsistency: %s' % problems)
    return ds, report


def aggregate(per_episode):
    def stats(values):
        xs = [v for v in values if _finite(v)]
        if not xs:
            return {'n': 0, 'min': None, 'max': None, 'mean': None}
        return {'n': len(xs), 'min': _round(min(xs)), 'max': _round(max(xs)),
                'mean': _round(sum(xs) / len(xs))}

    def block(entries):
        return {
            'episode_count': len(entries),
            'success_count': sum(1 for e in entries if e['success'] is True),
            'usable_count': sum(1 for e in entries if e['usable'] is True),
            'fell_count': sum(1 for e in entries if e['fell'] is True),
            'mean_speed_x_mps': stats([e['mean_speed_x_mps'] for e in entries]),
            'max_torso_tilt_deg': stats([e['max_torso_tilt_deg'] for e in entries]),
            'quality_score': stats([e['quality_score'] for e in entries]),
            'quality_status_counts': _count([e['quality_score_status']
                                             for e in entries]),
            'failure_reason_counts': _count(
                [e['failure_reason'] if e['failure_reason'] is not None
                 else 'none' for e in entries]),
        }
    by_config = {}
    for cid in sorted({e['candidate_id'] for e in per_episode
                       if e['candidate_id'] is not None}):
        by_config[cid] = block([e for e in per_episode
                                if e['candidate_id'] == cid])
    return {'overall': block(per_episode), 'by_config': by_config,
            'score_stats_by_status': _score_stats_by_status(per_episode)}


def _count(values):
    counts = {}
    for v in values:
        counts[v] = counts.get(v, 0) + 1
    return counts


def _score_stats_by_status(per_episode):
    out = {}
    for e in per_episode:
        out.setdefault(e['quality_score_status'], []).append(e['quality_score'])
    return {k: {'n': len(v)} for k, v in out.items()}


def check_aggregate_consistency(report):
    per_episode = report['per_episode']
    agg = report['aggregate']
    problems = []
    usable = sum(1 for e in per_episode if e['usable'] is True)
    if agg['overall']['usable_count'] != usable:
        problems.append('overall_usable_mismatch')
    if agg['overall']['episode_count'] != len(per_episode):
        problems.append('overall_count_mismatch')
    for cid, blk in agg['by_config'].items():
        subset = [e for e in per_episode if e['candidate_id'] == cid]
        if blk['usable_count'] != sum(1 for e in subset if e['usable'] is True):
            problems.append('by_config_usable_mismatch:%s' % cid)
        if blk['episode_count'] != len(subset):
            problems.append('by_config_count_mismatch:%s' % cid)
    total = sum(agg['overall']['failure_reason_counts'].values())
    if total != len(per_episode):
        problems.append('failure_counts_total_mismatch')
    return problems


# ---------------------------------------------------------------------------
# CLI + atomic report write
# ---------------------------------------------------------------------------
def _write_json_atomic(path, obj):
    directory = os.path.dirname(path) or '.'
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix='.tmp_p2e_', dir=directory)
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as fh:
            json.dump(obj, fh, indent=2, sort_keys=True, allow_nan=False)
            fh.flush()
            os.fsync(fh.fileno())
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def check_derived_root_available(derived_root):
    """Refuse to write derived output over an existing path (never overwrite)."""
    if os.path.exists(derived_root):
        raise Phase2EError(
            'derived root already exists, refusing to overwrite: %s'
            % derived_root)


def write_derived_index(report, derived_root):
    """
    Non-destructive derived index/splits under a NEW root (never touches the
    original dataset). Records score version and the filter/sort basis.
    """
    check_derived_root_available(derived_root)
    os.makedirs(derived_root)
    rows = sorted(
        [e for e in report['per_episode'] if e['usable'] is True],
        key=lambda e: (e['quality_score'] is None, -(e['quality_score'] or 0.0),
                       e['episode_id']))
    with open(os.path.join(derived_root, 'derived_index.jsonl'), 'w',
              encoding='utf-8') as fh:
        for e in rows:
            fh.write(json.dumps({
                'episode_id': e['episode_id'],
                'source_dataset': report['dataset_dir'],
                'quality_score': e['quality_score'],
                'quality_score_status': e['quality_score_status'],
                'quality_score_version': e['quality_score_version'],
            }, sort_keys=True, allow_nan=False) + '\n')
    _write_json_atomic(os.path.join(derived_root, 'derived_manifest.json'), {
        'source_dataset': report['dataset_dir'],
        'quality_score_version': report['quality_score_version'],
        'derived_from': 'phase2e_labels_quality',
        'filter': 'usable == true',
        'sort': 'quality_score desc (nulls last, episode_id asc tie-break)',
        'kept': len(rows),
        'total': len(report['per_episode']),
        'non_destructive': True,
    })
    return len(rows)


def main(argv=None):
    parser = argparse.ArgumentParser(description='Phase 2E labels + quality score')
    parser.add_argument('--dataset', required=True)
    parser.add_argument('--out', default=None)
    parser.add_argument('--derived-root', default=None)
    args = parser.parse_args(argv)
    # Validate output destinations before writing anything, so a refused
    # derived root never leaves behind a success-shaped report or temp files.
    if args.derived_root:
        check_derived_root_available(args.derived_root)
    _, report = evaluate_dataset(args.dataset)
    if args.out:
        _write_json_atomic(args.out, report)
    if args.derived_root:
        write_derived_index(report, args.derived_root)
    summary = {
        'episodes': len(report['per_episode']),
        'overall': report['aggregate']['overall'],
        'data_quality_warnings': len(report['data_quality_warnings']),
    }
    print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False))
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (ep.EpisodeDatasetError, Phase2EError) as exc:
        print(json.dumps({'ok': False, 'reason': type(exc).__name__,
                          'detail': str(exc)}), file=sys.stderr)
        sys.exit(2)
