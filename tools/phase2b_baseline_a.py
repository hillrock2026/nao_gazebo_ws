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
Phase 2B Baseline A planning, honest label derivation and quality reporting.

This module is intentionally ROS-free and side-effect free (except reading
files): the launch/collection runner
(``tools/run_task4_phase2b_baseline_a.py``) and the offline tests
(``tools/test_task4_phase2b_baseline_a.py``) both import it.

Scope (Phase 2B Baseline A only):
  * read the AUTHORITATIVE, already-validated gait configuration from the frozen
    Phase 4E/B3.0 charter (the config that actually walks) -- never assume;
  * derive the three step-length tiers A0/A1/A2 and a fixed, recorded seed list;
  * derive honest per-episode labels from the frozen samples and the read-only
    traces, with no fabricated defaults.

Field definitions introduced here (documented, measured -- never invented):

  mean_speed_x_mps   = body_dx / (steady_window_hi - steady_window_lo)
                       (body_pose trace; NOT cmd_vel, which is not used at all)
  max_torso_tilt_deg = max(|max_roll_deg|, |max_pitch_deg|) from the shared
                       Phase 4E recomputation
  fell               = min body z inside the steady window <= 0.25 m
                       (same 0.25 m threshold as the frozen
                       ``torso_min_height_gt_0.25`` gate)
  fall_time          = first simulation time inside the steady window where
                       body z <= 0.25 m (absolute sim seconds)
  energy_qdot        = mean over samples of sum_i dq_i(t)^2  [rad^2/s^2],
                       computed from the frozen samples' ``joint_velocity``
                       (a measured proxy; the evaluation layer has no canonical
                       energy metric yet -- see the Phase 2B report)
"""

import json
import math
import os

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Frozen, validated charter anchor: the single config that most recently walked
# in the Phase 4E/B3 evidence. This is the "current actually-running gait".
ANCHOR_CANDIDATES = os.path.join(
    WS, 'docs', 'task4_phase4e_pathb_b3_evidence', 'b3_0', 'candidates.json')

CONFIG_MULTIPLIERS = (('A0', 0.5), ('A1', 1.0), ('A2', 1.2))

# Fixed, reproducible integer seeds. Recorded as provenance/replicate identity.
# NOTE: the frozen Phase 4B producer is deterministic and does NOT consume a
# seed (no RNG, no noise model in nao_simple.sdf), so these seeds cannot change
# the trajectory. They are recorded honestly as replicate identifiers.
SEED_LIST = (101, 202, 303, 404, 505, 606, 707, 808, 909, 1010, 1111, 1212)

TARGET_COLLECT_SEC = 12.0          # Phase 4D collector window
MIN_DURATION_SEC = 8.0
MAX_DURATION_SEC = 15.0
FALL_Z_THRESHOLD_M = 0.25

CONTROLLER_NAME = 'phase4b_producer'
CONTROLLER_VERSION = 'phase4b_producer:frozen-2026'

# Canonical Phase 2A param keys whose absolute value the frozen gait defines.
# ``torso_pitch_rad`` is a reserved Phase 2A key that the frozen gait never
# varies; it must be passed explicitly (0.0) per the Phase 2A contract.
RESERVED_TORSO_PITCH_RAD = 0.0


class BaselineAError(Exception):
    """Explicit, non-silent Baseline A failure."""


# ---------------------------------------------------------------------------
# authoritative default / plan
# ---------------------------------------------------------------------------
def load_anchor_config(path=ANCHOR_CANDIDATES):
    """
    Read the frozen B3.0 charter anchor config (the validated walking gait).

    Returns ``(config_dict, source_description)``. Raises if the source is
    missing or does not contain exactly one anchor.
    """
    if not os.path.isfile(path):
        raise BaselineAError('authoritative config source missing: %s' % path)
    with open(path, 'r', encoding='utf-8') as fh:
        data = json.load(fh)
    anchors = [c for c in data.get('candidates', []) if c.get('kind') == 'anchor']
    if len(anchors) != 1:
        raise BaselineAError('expected exactly 1 anchor, got %d' % len(anchors))
    cfg = anchors[0].get('config')
    if not isinstance(cfg, dict):
        raise BaselineAError('anchor config is not an object')
    for key in ('step_length_m', 'step_frequency_hz', 'swing_height_m',
                'double_support_ratio', 'sway_amplitude_rad'):
        value = cfg.get(key)
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise BaselineAError('anchor config %s is not numeric' % key)
    source = 'docs/task4_phase4e_pathb_b3_evidence/b3_0/candidates.json' \
             '#candidates[kind=anchor].config (frozen Phase 4E/B3.0 anchor)'
    return dict(cfg), source


def derive_config_plan(anchor_cfg):
    """
    Derive the A0/A1/A2 plan from the authoritative default.

    ``step_length_m`` is scaled; ``step_frequency_hz`` is held at 1.0x default
    (Baseline A isolates step length; the validated cadence 0.5 Hz is kept).
    Returns a list of dicts with absolute values and the multiplier recorded.
    """
    base_step = float(anchor_cfg['step_length_m'])
    base_freq = float(anchor_cfg['step_frequency_hz'])
    plan = []
    for config_id, multiplier in CONFIG_MULTIPLIERS:
        overrides = {'step_length_m': base_step * multiplier,
                     'step_frequency_hz': base_freq * 1.0}
        plan.append({
            'config_id': config_id,
            'step_length_multiplier': multiplier,
            'frequency_multiplier': 1.0,
            'default_step_length_m': base_step,
            'default_frequency_hz': base_freq,
            'step_length_m': overrides['step_length_m'],
            'frequency_hz': overrides['step_frequency_hz'],
            'overrides': overrides,
            'rationale': 'Baseline A isolates step length at the validated cadence',
        })
    return plan


def fixed_seed_list(n):
    if n < 1 or n > len(SEED_LIST):
        raise BaselineAError('seed count %d out of range [1, %d]'
                             % (n, len(SEED_LIST)))
    return list(SEED_LIST[:n])


def episode_id_for(index):
    """Deterministic episode id for the sorted plan (1-based)."""
    if index < 1 or index > 999999:
        raise BaselineAError('episode index out of range: %d' % index)
    return 'episode_%06d' % index


def run_config_for(anchor_cfg, override):
    """Full producer config sent on the launch command: anchor + override."""
    cfg = dict(anchor_cfg)
    cfg.update(override)
    return cfg


def sampling_plan(seeds_per_config=3):
    """
    Frozen sampling plan: one row per (config_id, seed).

    Row keys: config_id, index, episode_id, seed, step_length_m, frequency_hz,
    overrides. This is fixed before execution and recorded in the report.
    """
    anchor, source = load_anchor_config()
    configs = derive_config_plan(anchor)
    seeds = fixed_seed_list(seeds_per_config)
    rows = []
    index = 0
    for config in configs:
        for seed in seeds:
            index += 1
            rows.append({
                'config_id': config['config_id'],
                'index': index,
                'episode_id': episode_id_for(index),
                'seed': seed,
                'step_length_m': config['step_length_m'],
                'frequency_hz': config['frequency_hz'],
                'step_length_multiplier': config['step_length_multiplier'],
                'overrides': dict(config['overrides']),
            })
    return {'anchor_source': source, 'anchor_config': anchor, 'rows': rows}


# ---------------------------------------------------------------------------
# honest label derivation
# ---------------------------------------------------------------------------
def _finite_number(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(float(value)))


def energy_qdot_from_samples(samples_path):
    """Mean over samples of sum_i dq_i^2 [rad^2/s^2] from joint_velocity."""
    total = 0.0
    n = 0
    with open(samples_path, 'r', encoding='utf-8') as fh:
        for line in fh:
            if not line.strip():
                raise BaselineAError('blank sample line in %s' % samples_path)
            rec = json.loads(line)
            dq = rec.get('joint_velocity')
            if not isinstance(dq, list) or not dq:
                raise BaselineAError('sample missing joint_velocity')
            total += sum(float(v) * float(v) for v in dq)
            n += 1
    if n == 0:
        return None, 0
    return total / n, n


def _load_jsonl(path):
    rows = []
    if not os.path.isfile(path):
        return rows
    with open(path, 'r', encoding='utf-8') as fh:
        for line in fh:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def fall_from_body_trace(trace_dir, lo, hi, threshold=FALL_Z_THRESHOLD_M):
    """
    Return (fell, fall_time, min_z) over the steady window [lo, hi].

    A fall is declared when the body height drops to/below ``threshold`` (the
    frozen torso-height gate uses 0.25 m); ``fall_time`` is the first sim time
    at/under the threshold. Returns ``(None, None, None)`` if the trace is
    unavailable, so the caller can mark the field undetermined instead of
    guessing.
    """
    rows = _load_jsonl(os.path.join(trace_dir, 'body_pose_trace.jsonl'))
    if not rows:
        return None, None, None
    min_z = None
    fall_time = None
    for row in rows:
        t = row.get('t')
        z = row.get('z')
        if not _finite_number(t) or not _finite_number(z):
            continue
        if t < lo - 1e-9 or t > hi + 1e-9:
            continue
        if min_z is None or z < min_z:
            min_z = z
        if fall_time is None and z <= threshold:
            fall_time = float(t)
    if min_z is None:
        return None, None, None
    return (fall_time is not None), fall_time, float(min_z)


def derive_episode_labels(run_dir, trace_dir, run_cfg, ready, summary,
                          samples_path=None):
    """
    Derive the Phase 2A metadata fields from measured data only.

    ``summary`` is the Phase 4D collector statistics dict (may be None if the
    run failed). Returns a dict with the metadata fields plus ``missing`` (a
    list of required metrics that could not be determined) and ``metrics`` (the
    raw Phase 4E metric dict for the report). Any required field that cannot be
    measured is reported in ``missing`` -- never defaulted.
    """
    import sys
    tools = os.path.dirname(os.path.abspath(__file__))
    if tools not in sys.path:
        sys.path.insert(0, tools)
    import phase4e_metrics as M  # local import: ROS-free but heavy

    samples_path = samples_path or os.path.join(run_dir, 'samples.jsonl')
    missing = []
    out = {
        'target_duration_sec': TARGET_COLLECT_SEC,
        'collector_committed': bool(summary and summary.get('dataset_committed')),
        'collector_finish_reason': (summary or {}).get('finish_reason'),
        'sample_count': (summary or {}).get('samples_written'),
    }

    metrics = M.run_metrics(trace_dir, run_cfg, ready, mode='walk') \
        if trace_dir and os.path.isdir(trace_dir) else {}
    out['metrics'] = metrics
    window = metrics.get('window') or {}
    lo = window.get('lo')
    hi = window.get('hi')

    # actual collected span (authoritative duration)
    first = (summary or {}).get('first_written_timestamp')
    last = (summary or {}).get('last_written_timestamp')
    if _finite_number(first) and _finite_number(last) and last > first:
        duration = float(last) - float(first)
    else:
        duration = None
        missing.append('duration_sec')
    out['duration_sec'] = duration

    # mean forward speed (measured from body pose, never cmd_vel)
    if _finite_number(metrics.get('body_dx')) and _finite_number(lo) \
            and _finite_number(hi) and hi > lo:
        out['mean_speed_x_mps'] = float(metrics['body_dx']) / (hi - lo)
    else:
        out['mean_speed_x_mps'] = None
        missing.append('mean_speed_x_mps')

    # torso tilt
    roll = metrics.get('max_roll_deg')
    pitch = metrics.get('max_pitch_deg')
    if _finite_number(roll) and _finite_number(pitch):
        out['max_torso_tilt_deg'] = max(abs(float(roll)), abs(float(pitch)))
    else:
        out['max_torso_tilt_deg'] = None
        missing.append('max_torso_tilt_deg')

    # energy proxy from the frozen samples
    try:
        energy, _n = energy_qdot_from_samples(samples_path)
    except (OSError, ValueError, BaselineAError):
        energy = None
    if energy is not None:
        out['energy_qdot'] = float(energy)
        out['energy_qdot_definition'] = 'mean_sum_joint_velocity_squared_rad2_s2'
    else:
        out['energy_qdot'] = None
        missing.append('energy_qdot')

    # fall state from the body trace over the steady window
    if _finite_number(lo) and _finite_number(hi):
        fell, fall_time, min_z = fall_from_body_trace(trace_dir, lo, hi)
    else:
        fell, fall_time, min_z = None, None, None
    out['body_min_z_m'] = min_z
    if fell is None:
        missing.append('fell')
        out['fell'] = None
        out['fall_time'] = None
        missing.append('fall_time')
    else:
        out['fell'] = bool(fell)
        out['fall_time'] = float(fall_time) if fall_time is not None else -1.0

    # explicit success / usable rules (never silently default to success)
    success = True
    reasons = []
    if out['fell'] is None:
        success = False
        reasons.append('fall_not_determined')
    elif out['fell']:
        success = False
        reasons.append('fall')
    if duration is None:
        success = False
        reasons.append('duration_not_measured')
    elif duration < MIN_DURATION_SEC:
        success = False
        reasons.append('duration_below_min')
    elif duration > MAX_DURATION_SEC:
        success = False
        reasons.append('duration_above_max')
    if not out['collector_committed']:
        success = False
        reasons.append('collector_not_committed')
    if out['mean_speed_x_mps'] is None or out['max_torso_tilt_deg'] is None:
        success = False
        reasons.append('metric_missing')

    out['success'] = success
    # Phase 2D/2E semantics: ``usable`` means the episode's samples are complete,
    # committed and auditable -- it is INDEPENDENT of ``success``, so a failed
    # but complete episode is usable. ``success`` keeps its own meaning.
    out['usable'] = bool(out['collector_committed'] and not missing)
    if success:
        out['failure_reason'] = None
    else:
        out['failure_reason'] = '_'.join(reasons) if reasons else 'unspecified_failure'
    out['missing'] = missing
    out['ready_sim'] = ready
    return out


# ---------------------------------------------------------------------------
# sampling plan / quality statistics (pure)
# ---------------------------------------------------------------------------
def _stats(values):
    xs = [float(v) for v in values if _finite_number(v)]
    if not xs:
        return {'n': 0, 'min': None, 'max': None, 'mean': None}
    return {'n': len(xs), 'min': min(xs), 'max': max(xs),
            'mean': sum(xs) / len(xs)}


def quality_report(episodes, split_counts, index_rows):
    """
    Aggregate honest quality statistics.

    ``episodes`` is a list of per-episode dicts: config_id, seed, episode_id,
    labels (derive_episode_labels output), actual duration, split. Returns a
    JSON-serialisable report dict.
    """
    by_config = {}
    for ep in episodes:
        by_config.setdefault(ep['config_id'], []).append(ep)
    report = {'by_config': {}, 'overall': {}}
    all_eps = []
    for config_id in sorted(by_config):
        eps = by_config[config_id]
        all_eps.extend(eps)
        labels = [e['labels'] for e in eps]
        speeds = [l.get('mean_speed_x_mps') for l in labels]
        tilts = [l.get('max_torso_tilt_deg') for l in labels]
        durations = [l.get('duration_sec') for l in labels]
        report['by_config'][config_id] = {
            'step_length_m': eps[0]['step_length_m'],
            'frequency_hz': eps[0]['frequency_hz'],
            'seeds': sorted(e['seed'] for e in eps),
            'episode_count': len(eps),
            'success_count': sum(1 for l in labels if l.get('success')),
            'fell_count': sum(1 for l in labels if l.get('fell')),
            'usable_count': sum(1 for l in labels if l.get('usable')),
            'duration_sec': _stats(durations),
            'mean_speed_x_mps': _stats(speeds),
            'max_torso_tilt_deg': _stats(tilts),
            'energy_qdot_coverage': sum(
                1 for l in labels if _finite_number(l.get('energy_qdot'))),
            'fall_time_coverage': sum(
                1 for l in labels if l.get('fall_time') is not None),
            'failure_reasons': _count_reasons(labels),
        }
    # uniqueness / duplicates
    ids = [e['episode_id'] for e in all_eps]
    pairs = ['%s:%s' % (e['config_id'], e['seed']) for e in all_eps]
    report['overall'] = {
        'episode_count': len(all_eps),
        'success_count': sum(1 for e in all_eps if e['labels'].get('success')),
        'fell_count': sum(1 for e in all_eps if e['labels'].get('fell')),
        'usable_count': sum(1 for e in all_eps if e['labels'].get('usable')),
        'duration_sec': _stats([e['labels'].get('duration_sec') for e in all_eps]),
        'mean_speed_x_mps': _stats(
            [e['labels'].get('mean_speed_x_mps') for e in all_eps]),
        'max_torso_tilt_deg': _stats(
            [e['labels'].get('max_torso_tilt_deg') for e in all_eps]),
        'duplicate_episode_ids': sorted(
            {i for i in ids if ids.count(i) > 1}),
        'duplicate_config_seed_pairs': sorted(
            {p for p in pairs if pairs.count(p) > 1}),
        'split_counts': dict(split_counts),
        'index_rows': len(index_rows),
    }
    report['speed_order'] = speed_order(episodes)
    return report


def _count_reasons(labels):
    counts = {}
    for l in labels:
        reason = l.get('failure_reason')
        if reason:
            counts[reason] = counts.get(reason, 0) + 1
    return counts


def speed_order(episodes):
    """
    Order (config_id, mean measured speed) by measured speed, descending.

    Only measured ``mean_speed_x_mps`` is used -- never cmd_vel or a theoretical
    value. Episodes without a measured speed are listed separately.
    """
    measured = [(e['config_id'], e['episode_id'],
                 e['labels'].get('mean_speed_x_mps')) for e in episodes]
    good = [m for m in measured if _finite_number(m[2])]
    bad = [m for m in measured if not _finite_number(m[2])]
    good.sort(key=lambda m: m[2], reverse=True)
    return {
        'measured': [{'config_id': c, 'episode_id': e, 'mean_speed_x_mps': s}
                     for c, e, s in good],
        'unmeasured': [{'config_id': c, 'episode_id': e} for c, e, _ in bad],
        'basis': 'measured mean_speed_x_mps from body_pose trace (not cmd_vel)',
    }
