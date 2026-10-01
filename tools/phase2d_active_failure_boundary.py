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
Phase 2D — active failure & boundary-sample planning and honest classification.

Pure, ROS-free. It:
  * defines the stable ``failure_reason`` enum and a stable priority order;
  * defines versioned, explicit failure criteria (no "extreme parameter" labels);
  * derives directed boundary candidates from the validated anchor default;
  * refuses parameters the controller does not support (e.g. ``torso_pitch_rad``);
  * validates the external-push schema and reports that no approved force
    injection interface exists (so push is planned but never faked);
  * classifies an observed episode into (primary, secondary) failure reasons from
    measured evidence only.

``expected_risk`` and ``actual_failure_reason`` are deliberately separate: the
former is a plan hypothesis, the latter comes from evidence via
``classify_failure``.
"""

import hashlib
import math
import os

import phase2c_baseline_b as C

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_ROOT_BASE = os.path.join(
    WS, 'docs', 'task4_phase2d_active_failure_boundary_evidence')

# ---------------------------------------------------------------------------
# failure taxonomy
# ---------------------------------------------------------------------------
FAILURE_REASONS = (
    'startup_instability',
    'excessive_step_length',
    'excessive_frequency',
    'insufficient_swing_height',
    'excessive_torso_pitch',
    'lateral_instability',
    'contact_anomaly',
    'tracking_error',
    'fall',
    'timeout_or_no_progress',
)

# Stable primary-cause priority (first match wins when several are true).
FAILURE_PRIORITY = (
    'startup_instability',
    'timeout_or_no_progress',
    'fall',
    'excessive_step_length',
    'excessive_frequency',
    'insufficient_swing_height',
    'excessive_torso_pitch',
    'lateral_instability',
    'contact_anomaly',
    'tracking_error',
)

# Versioned, explicit criteria. A reason is only ever assigned when its
# criterion is actually met by measured evidence.
CRITERIA = {
    'version': '2D-1.0.0',
    'fall_body_min_z_m': 0.25,          # same threshold as frozen torso gate
    'lateral_body_dy_m': 0.15,
    'lateral_tilt_deg': 30.0,
    'tracking_max_rad': 0.60,           # per-segment max tracking error
    'contact_min_support': 3,
    'contact_min_alternation_fraction': 0.60,
    'min_duration_sec': 8.0,
    'max_duration_sec': 15.0,
}

MIN_DURATION_SEC = CRITERIA['min_duration_sec']
MAX_DURATION_SEC = CRITERIA['max_duration_sec']


class Phase2DError(Exception):
    pass


# ---------------------------------------------------------------------------
# directed boundary candidates
# ---------------------------------------------------------------------------
# (candidate_id, param, absolute-or-multiplier, boundary_label, expected_risk,
#  kind). Values are computed from the anchor default; multipliers are kept as
# provenance. ``probe_*`` candidates are within the controller range but are
# deliberately extreme to try to *observe* a failure -- they never force a label.
def _boundary_specs():
    return [
        # --- required directed boundaries (section 4) ---
        {'candidate_id': 'boundary_step_length_1p4x', 'param': 'step_length_m',
         'multiplier': 1.4, 'boundary_label': 'upper_step_length',
         'expected_risk': 'excessive_step_length', 'kind': 'boundary'},
        {'candidate_id': 'boundary_frequency_1p3x', 'param': 'step_frequency_hz',
         'multiplier': 1.3, 'boundary_label': 'upper_frequency',
         'expected_risk': 'excessive_frequency', 'kind': 'boundary'},
        {'candidate_id': 'boundary_swing_0p7x', 'param': 'swing_height_m',
         'multiplier': 0.7, 'boundary_label': 'lower_swing_height',
         'expected_risk': 'insufficient_swing_height', 'kind': 'boundary'},
        {'candidate_id': 'boundary_ds_low', 'param': 'double_support_ratio',
         'absolute': 0.05, 'boundary_label': 'low_double_support',
         'expected_risk': 'contact_anomaly', 'kind': 'boundary'},
        # torso_pitch is not a controller input -> refused below
        {'candidate_id': 'boundary_torso_pitch_p6deg', 'param': 'torso_pitch_rad',
         'absolute': math.radians(6.0), 'boundary_label': 'upper_torso_pitch',
         'expected_risk': 'excessive_torso_pitch', 'kind': 'boundary'},
        # --- diagnostic probes (within controller range, attempt observation) ---
        {'candidate_id': 'probe_step_length_max', 'param': 'step_length_m',
         'absolute': C.CONTROLLER_RANGES['step_length_m'][1],
         'boundary_label': 'max_step_length',
         'expected_risk': 'excessive_step_length', 'kind': 'probe'},
        {'candidate_id': 'probe_frequency_high', 'param': 'step_frequency_hz',
         'absolute': 1.2, 'boundary_label': 'high_frequency',
         'expected_risk': 'excessive_frequency', 'kind': 'probe'},
        {'candidate_id': 'probe_swing_min', 'param': 'swing_height_m',
         'absolute': 0.0, 'boundary_label': 'min_swing_height',
         'expected_risk': 'insufficient_swing_height', 'kind': 'probe'},
    ]


def _compute_absolute(anchor, spec):
    if 'absolute' in spec:
        return float(spec['absolute'])
    base = float(anchor[spec['param']])
    return base * float(spec['multiplier'])


def build_boundary_candidates(anchor=None, supported=None):
    if anchor is None:
        anchor, _ = C.load_anchor()
    if supported is None:
        supported = C.controller_supported_params()
    candidates = []
    for spec in _boundary_specs():
        absolute = _compute_absolute(anchor, spec)
        problems = []
        if spec['param'] not in supported:
            problems.append({'reason': 'unsupported_parameter',
                             'param': spec['param'],
                             'detail': 'controller declares no such parameter'})
        else:
            problems.extend(C.validate_override(spec['param'], absolute,
                                                supported))
        candidates.append({
            'candidate_id': spec['candidate_id'],
            'param': spec['param'],
            'kind': spec['kind'],
            'multiplier': spec.get('multiplier'),
            'absolute': absolute,
            'boundary_label': spec['boundary_label'],
            'expected_risk': spec['expected_risk'],
            'supported': not problems,
            'problems': problems,
        })
    return candidates


def boundary_plan(seeds_per_candidate=1, anchor_repeats=1):
    anchor, source = C.load_anchor()
    supported = C.controller_supported_params()
    candidates = build_boundary_candidates(anchor, supported)
    seeds = list(C.SEED_LIST[:max(seeds_per_candidate, 1)])
    rows = []
    index = 0
    refused = []
    for r in range(anchor_repeats):
        index += 1
        rows.append({
            'episode_id': 'episode_%06d' % index,
            'candidate_id': 'ANCHOR_CTRL',
            'kind': 'anchor',
            'param': None,
            'multiplier': 1.0,
            'absolute': {},
            'requested_param_set': {},
            'boundary_label': 'default_anchor',
            'expected_risk': None,
            'overrides': {},
            'seed': C.SEED_LIST[r],
            'external_push': None,
        })
    for cand in candidates:
        if not cand['supported']:
            refused.append(cand)
            continue
        for s in range(seeds_per_candidate):
            index += 1
            rows.append({
                'episode_id': 'episode_%06d' % index,
                'candidate_id': cand['candidate_id'],
                'kind': cand['kind'],
                'param': cand['param'],
                'multiplier': cand['multiplier'],
                'absolute': {cand['param']: cand['absolute']},
                'requested_param_set': {cand['param']: cand['absolute']},
                'boundary_label': cand['boundary_label'],
                'expected_risk': cand['expected_risk'],
                'overrides': {cand['param']: cand['absolute']},
                'seed': seeds[s if s < len(seeds) else 0],
                'external_push': None,
            })
    return {
        'round': 'boundary_smoke',
        'anchor_source': source,
        'anchor_config': anchor,
        'controller_supported_params': sorted(supported),
        'criteria': CRITERIA,
        'rows': rows,
        'refused': refused,
        'estimated_seconds_per_episode': 55,
        'estimated_total_seconds': len(rows) * 55,
    }


# ---------------------------------------------------------------------------
# external push (plan only; not supported yet)
# ---------------------------------------------------------------------------
EXTERNAL_PUSH_SCHEMA_KEYS = (
    'time_sec', 'duration_sec', 'force_n', 'direction', 'link', 'frame')


def validate_external_push(push):
    """Return a list of problem dicts for one external_push object."""
    if push is None:
        return []
    if not isinstance(push, dict):
        return [{'reason': 'push_not_object'}]
    problems = []
    for key in ('time_sec', 'force_n', 'direction'):
        if key not in push:
            problems.append({'reason': 'push_missing_key', 'key': key})
    for key in ('time_sec', 'duration_sec', 'force_n'):
        if key in push and (not isinstance(push[key], (int, float))
                            or isinstance(push[key], bool)
                            or not math.isfinite(float(push[key]))):
            problems.append({'reason': 'push_value_not_finite', 'key': key})
    direction = push.get('direction')
    if direction is not None:
        if not isinstance(direction, list) or len(direction) != 3 \
                or not all(isinstance(v, (int, float))
                           and not isinstance(v, bool)
                           and math.isfinite(float(v)) for v in direction):
            problems.append({'reason': 'push_direction_invalid'})
        else:
            norm = math.sqrt(sum(float(v) ** 2 for v in direction))
            if norm < 1e-9:
                problems.append({'reason': 'push_direction_zero'})
    return problems


def external_push_plan():
    """
    External push is planned but NOT executed: the frozen Task-1 world does not
    load an ApplyLinkWrench system, so there is no approved force-injection
    interface wired into the approved launch. Faking a push is forbidden.
    """
    return {
        'supported': False,
        'reason': 'no_approved_force_injection_interface',
        'detail': ('worlds/nao_world.sdf loads Physics/UserCommands/'
                   'SceneBroadcaster/Imu/Contact only; the gz-sim '
                   'ApplyLinkWrench system is not loaded and the frozen launch '
                   'exposes no wrench service. Wiring one would modify frozen '
                   'Task-1 world/launch assets, which is out of Phase 2D scope.'),
        'schema_keys': list(EXTERNAL_PUSH_SCHEMA_KEYS),
        'planned_configs': [],
        'status': 'not_covered',
    }


# ---------------------------------------------------------------------------
# failure coverage matrix
# ---------------------------------------------------------------------------
def failure_coverage_matrix(rows):
    target = {r: [] for r in FAILURE_REASONS}
    for row in rows:
        risk = row.get('expected_risk')
        if risk in target:
            target[risk].append(row['candidate_id'])
    matrix = {}
    for reason in FAILURE_REASONS:
        cands = sorted(set(target[reason]))
        matrix[reason] = {
            'planned_candidates': cands,
            'covered_by_plan': bool(cands),
            'status': 'planned' if cands else 'not_covered',
            'note': None if cands else
            'no safe/reproducible directed trigger in current capability',
        }
    # reasons that have no candidate at all
    matrix['external_push'] = external_push_plan()
    return matrix


# ---------------------------------------------------------------------------
# honest classification
# ---------------------------------------------------------------------------
def _finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) \
        and math.isfinite(float(v))


def classify_failure(frozen_valid, sample_count, timed_out, labels, metrics,
                     boundaries=None):
    """
    Classify one observed episode from measured evidence only.

    Returns a dict: ``success``, ``failure_reason`` (primary or None),
    ``secondary_reasons``, ``usable``, ``diagnostics``.
    ``boundaries`` maps param -> {'boundary_label','expected_risk'} for mapping
    an observed non-fall instability onto the excessive_* reason.
    """
    boundaries = boundaries or {}
    metrics = metrics or {}
    secondary = []
    diagnostics = {}

    # 1. no usable samples at all
    if not frozen_valid or not sample_count:
        return {'success': False, 'failure_reason': 'startup_instability',
                'secondary_reasons': [], 'usable': False,
                'diagnostics': {'frozen_valid': bool(frozen_valid),
                                'sample_count': sample_count}}

    duration = labels.get('duration_sec')
    fell = labels.get('fell')
    tilt = labels.get('max_torso_tilt_deg')
    body_dy = metrics.get('body_dy')
    contact = metrics.get('alternation') or {}
    phase4c = metrics.get('phase4c') or {}

    primary = None
    if timed_out or (not _finite(duration)) or duration < MIN_DURATION_SEC:
        primary = 'timeout_or_no_progress'
        diagnostics['duration_sec'] = duration
        diagnostics['timed_out'] = bool(timed_out)
    elif fell:
        primary = 'fall'
    else:
        if _finite(body_dy) and abs(float(body_dy)) > CRITERIA['lateral_body_dy_m']:
            secondary.append('lateral_instability')
            diagnostics['body_dy_m'] = float(body_dy)
        if _finite(tilt) and float(tilt) > CRITERIA['lateral_tilt_deg']:
            secondary.append('lateral_instability')
            diagnostics['max_torso_tilt_deg'] = float(tilt)
        if contact and contact.get('ok') is False:
            secondary.append('contact_anomaly')
            diagnostics['contact'] = {
                'n_support': contact.get('n_support'),
                'alternation_fraction': contact.get('alternation_fraction')}
        trek = _tracking_max(phase4c)
        if trek is not None and trek > CRITERIA['tracking_max_rad']:
            secondary.append('tracking_error')
            diagnostics['tracking_max_rad'] = trek

        # boundary hypothesis: map an observed instability (not a fall) on a
        # perturbed parameter to its excessive_/insufficient_ reason.
        if secondary:
            primary = _boundary_reason(boundaries) or secondary[0]
            secondary = [s for s in secondary if s != primary]

    success = primary is None
    return {
        'success': success,
        'failure_reason': primary,
        'secondary_reasons': sorted(set(secondary)),
        # complete/committed samples are usable regardless of success (the
        # authoritative usable flag comes from the shared label derivation).
        'usable': bool(labels.get('usable', True)),
        'diagnostics': diagnostics,
    }


def _boundary_reason(boundaries):
    for param, info in boundaries.items():
        risk = info.get('expected_risk')
        if risk in ('excessive_step_length', 'excessive_frequency',
                    'insufficient_swing_height', 'excessive_torso_pitch'):
            return risk
    return None


def _tracking_max(phase4c):
    """Best-effort extraction of a max per-segment tracking error (rad)."""
    try:
        tracking = phase4c.get('tracking') or {}
        values = []
        for seg in tracking.values():
            for v in (seg.get('max') or {}).values() if isinstance(seg, dict) else []:
                if _finite(v):
                    values.append(abs(float(v)))
        return max(values) if values else None
    except (AttributeError, TypeError):
        return None


# ---------------------------------------------------------------------------
# full plan
# ---------------------------------------------------------------------------
def full_plan():
    anchor, source = C.load_anchor()
    supported = C.controller_supported_params()
    boundary = boundary_plan()
    return {
        'default_source': source,
        'controller_supported_params': sorted(supported),
        'failure_reasons': list(FAILURE_REASONS),
        'failure_priority': list(FAILURE_PRIORITY),
        'criteria': CRITERIA,
        'boundary_candidates': build_boundary_candidates(anchor, supported),
        'boundary_smoke': boundary,
        'external_push': external_push_plan(),
        'coverage': failure_coverage_matrix(boundary['rows']),
        'notes': [
            'expected_risk is a hypothesis; actual failure comes from evidence',
            'torso_pitch_rad is refused (unsupported), never faked',
            'external push is planned but not executable (no injection interface)',
            'seeds are fixed provenance ids; the frozen controller has no RNG',
        ],
    }


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()
