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
Phase 2C Baseline B planning (parameter-perturbation study).

Pure, ROS-free, side-effect-free: reads the validated Phase 4E/B3.0 anchor and
the frozen producer source to derive absolute candidate parameters, and refuses
candidates the controller cannot honour (never silently ignoring/clamping).

Round 1  univariate : one factor at a time
Round 2  mixed      : 40% small perturbations, 40% 2-3-parameter combinations,
                      20% boundary samples

Design rules enforced here:
  * absolute values are computed from the anchor default and recorded, with the
    multiplier kept only as an additional provenance field;
  * every override is type/finite/range checked against the controller's own
    documented limits (read from the producer source);
  * a parameter the controller does not declare (e.g. ``torso_pitch_rad``) is
    refused explicitly, not dropped;
  * no silent clamping: an out-of-range candidate is refused.
"""

import hashlib
import json
import math
import os
import re

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ANCHOR_CANDIDATES = os.path.join(
    WS, 'docs', 'task4_phase4e_pathb_b3_evidence', 'b3_0', 'candidates.json')
PRODUCER_SOURCE = os.path.join(
    WS, 'src', 'nao_gz_sim', 'scripts', 'phase4b_producer.py')

# Fixed, reproducible integer seeds (provenance only; the frozen controller has
# no RNG). Never system time, never duplicated.
SEED_LIST = tuple(range(1001, 1001 + 200))

FACTORS = ('step_length', 'step_frequency', 'swing_height',
           'double_support_ratio', 'torso_pitch')

# Controller parameter -> (min, max, max_inclusive) limits, mirroring the frozen
# producer's ``_validate_parameters`` (phase4b_producer.py). Used to refuse
# out-of-range candidates instead of clamping.
CONTROLLER_RANGES = {
    'step_length_m': (0.0, 0.08, True),
    'step_frequency_hz': (0.2, 2.0, True),
    'swing_height_m': (0.0, 0.05, True),
    'double_support_ratio': (0.0, 0.6, False),   # strictly < 0.6
    'leg_extension_m': (0.15, 0.24801, True),
    'sway_amplitude_rad': (0.0, 0.35, True),
}

# Univariate candidate table: factor -> (controller param, [multipliers]).
# torso_pitch is expressed in DEGREES and converted to rad below; it is not a
# controller parameter, so its candidates are refused.
UNIVARIATE = {
    'step_length': ('step_length_m', [0.5, 0.75, 1.0, 1.2, 1.4]),
    'step_frequency': ('step_frequency_hz', [0.7, 0.85, 1.0, 1.15, 1.3]),
    'swing_height': ('swing_height_m', [0.7, 1.0, 1.3]),
    'double_support_ratio': ('double_support_ratio', [0.8, 1.0, 1.2]),
}
TORSO_PITCH_DEG = (-3.0, 0.0, 3.0)


class BaselineBError(Exception):
    pass


# ---------------------------------------------------------------------------
# sources
# ---------------------------------------------------------------------------
def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def load_anchor():
    if not os.path.isfile(ANCHOR_CANDIDATES):
        raise BaselineBError('anchor source missing: %s' % ANCHOR_CANDIDATES)
    with open(ANCHOR_CANDIDATES, 'r', encoding='utf-8') as fh:
        data = json.load(fh)
    anchors = [c for c in data.get('candidates', []) if c.get('kind') == 'anchor']
    if len(anchors) != 1:
        raise BaselineBError('expected exactly 1 anchor, got %d' % len(anchors))
    cfg = anchors[0]['config']
    return dict(cfg), {
        'path': os.path.relpath(ANCHOR_CANDIDATES, WS),
        'sha256': sha256_file(ANCHOR_CANDIDATES),
        'field': 'candidates[kind=anchor].config',
    }


def controller_supported_params():
    """Parse ``declare_parameter('name', ...)`` from the frozen producer."""
    if not os.path.isfile(PRODUCER_SOURCE):
        raise BaselineBError('producer source missing: %s' % PRODUCER_SOURCE)
    with open(PRODUCER_SOURCE, 'r', encoding='utf-8') as fh:
        text = fh.read()
    return set(re.findall(r"declare_parameter\('([A-Za-z0-9_]+)'", text))


# ---------------------------------------------------------------------------
# validation
# ---------------------------------------------------------------------------
def _is_finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) \
        and math.isfinite(float(v))


def validate_override(param, value, supported):
    """
    Return a list of problem dicts for one override ([] = acceptable).

    Unsupported parameters are refused with reason ``unsupported_parameter``.
    """
    problems = []
    if param not in supported:
        problems.append({'reason': 'unsupported_parameter', 'param': param})
        return problems
    if not _is_finite(value):
        problems.append({'reason': 'not_finite_number', 'param': param,
                         'value': value})
        return problems
    rng = CONTROLLER_RANGES.get(param)
    if rng is not None:
        lo, hi, inclusive = rng
        v = float(value)
        if v < lo or (v > hi if inclusive else v >= hi):
            problems.append({'reason': 'out_of_controller_range', 'param': param,
                             'value': v, 'range': [lo, hi], 'inclusive': inclusive})
    return problems


# ---------------------------------------------------------------------------
# candidate construction
# ---------------------------------------------------------------------------
def _abs_cfg(anchor, multiplier_map):
    """Absolute config: anchor value * multiplier for each param."""
    out = {}
    for param, mult in multiplier_map.items():
        if param not in anchor:
            raise BaselineBError('anchor has no %s' % param)
        out[param] = float(anchor[param]) * float(mult)
    return out


def _candidate(candidate_id, kind, factor, multipliers, anchor, supported,
               angle_deg=None):
    absolute = _abs_cfg(anchor, multipliers)
    problems = []
    for param, value in absolute.items():
        problems.extend(validate_override(param, value, supported))
    return {
        'candidate_id': candidate_id,
        'kind': kind,
        'factor': factor,
        'multipliers': {k: float(v) for k, v in multipliers.items()},
        'absolute': absolute,
        'angle_deg': angle_deg,
        'supported': not problems,
        'problems': problems,
    }


def build_univariate_candidates(anchor=None, supported=None):
    if anchor is None:
        anchor, _ = load_anchor()
    if supported is None:
        supported = controller_supported_params()
    candidates = []
    for factor, (param, multipliers) in UNIVARIATE.items():
        for mult in multipliers:
            cid = '%s_%gx' % (factor, mult)
            candidates.append(_candidate(cid, 'univariate', factor,
                                         {param: mult}, anchor, supported))
    # torso_pitch: degrees -> rad, but the controller declares no such input,
    # so every candidate is refused (not clamped, not silently dropped).
    for deg in TORSO_PITCH_DEG:
        cid = 'torso_pitch_%+gdeg' % deg
        candidates.append({
            'candidate_id': cid,
            'kind': 'univariate',
            'factor': 'torso_pitch',
            'multipliers': {'torso_pitch_rad': 1.0},
            'absolute': {'torso_pitch_rad': math.radians(deg)},
            'angle_deg': deg,
            'supported': False,
            'problems': [{'reason': 'unsupported_parameter',
                          'param': 'torso_pitch_rad',
                          'detail': 'phase4b producer has no torso_pitch input'}],
        })
    return candidates


# Mixed-perturbation table: (id, changed params -> multipliers). The 40/40/20
# split is asserted by build_mixed_candidates.
MIXED_SMALL = [
    ('m_small_sl_090', {'step_length_m': 0.90}),
    ('m_small_sl_110', {'step_length_m': 1.10}),
    ('m_small_fr_090', {'step_frequency_hz': 0.90}),
    ('m_small_fr_110', {'step_frequency_hz': 1.10}),
    ('m_small_sw_090', {'swing_height_m': 0.90}),
    ('m_small_sw_110', {'swing_height_m': 1.10}),
    ('m_small_ds_090', {'double_support_ratio': 0.90}),
    ('m_small_ds_110', {'double_support_ratio': 1.10}),
]
MIXED_JOINT = [
    ('m_joint_sl090_fr110', {'step_length_m': 0.90, 'step_frequency_hz': 1.10}),
    ('m_joint_sl110_fr090', {'step_length_m': 1.10, 'step_frequency_hz': 0.90}),
    ('m_joint_sl110_sw110', {'step_length_m': 1.10, 'swing_height_m': 1.10}),
    ('m_joint_fr110_sw110', {'step_frequency_hz': 1.10, 'swing_height_m': 1.10}),
    ('m_joint_sl090_ds110', {'step_length_m': 0.90, 'double_support_ratio': 1.10}),
    ('m_joint_sl110_fr110_sw110', {'step_length_m': 1.10,
                                   'step_frequency_hz': 1.10,
                                   'swing_height_m': 1.10}),
    ('m_joint_sl090_fr090_ds090', {'step_length_m': 0.90,
                                   'step_frequency_hz': 0.90,
                                   'double_support_ratio': 0.90}),
    ('m_joint_sl120_fr110_sw120', {'step_length_m': 1.20,
                                   'step_frequency_hz': 1.10,
                                   'swing_height_m': 1.20}),
]
MIXED_BOUNDARY = [
    ('m_bound_sl140', {'step_length_m': 1.40}),
    ('m_bound_fr130', {'step_frequency_hz': 1.30}),
    ('m_bound_sw130', {'swing_height_m': 1.30}),
    ('m_bound_ds120_fr130', {'double_support_ratio': 1.20,
                             'step_frequency_hz': 1.30}),
]


def build_mixed_candidates(anchor=None, supported=None):
    if anchor is None:
        anchor, _ = load_anchor()
    if supported is None:
        supported = controller_supported_params()
    groups = (('small', MIXED_SMALL), ('joint', MIXED_JOINT),
              ('boundary', MIXED_BOUNDARY))
    candidates = []
    for kind, table in groups:
        for cid, mult in table:
            candidates.append(_candidate(cid, 'mixed_' + kind, 'mixed',
                                         mult, anchor, supported))
    return candidates


def _ratio(counts, total):
    return {k: (v / total if total else 0.0) for k, v in counts.items()}


def mixed_ratio_check(candidates):
    """Return the actual 40/40/20 kind ratio and whether it matches the plan."""
    counts = {'mixed_small': 0, 'mixed_joint': 0, 'mixed_boundary': 0}
    for c in candidates:
        if c['kind'] in counts:
            counts[c['kind']] += 1
    total = sum(counts.values())
    ratios = _ratio(counts, total)
    target = {'mixed_small': 0.4, 'mixed_joint': 0.4, 'mixed_boundary': 0.2}
    ok = all(abs(ratios[k] - target[k]) <= 0.001 for k in target)
    return {'counts': counts, 'ratios': ratios, 'target': target, 'ok': ok}


# ---------------------------------------------------------------------------
# plan assembly
# ---------------------------------------------------------------------------
def _episode_id(index):
    return 'episode_%06d' % index


def build_univariate_smoke_plan(seeds_per_candidate=1, anchor_repeats=3):
    """
    Small first-round smoke: for each supported factor, the two extreme
    perturbed candidates (2-3 required) plus dedicated default-anchor controls.
    ``torso_pitch`` candidates are refused and never scheduled.
    """
    supported = controller_supported_params()
    anchor, source = load_anchor()
    all_candidates = build_univariate_candidates(anchor, supported)
    seeds = list(SEED_LIST[:max(seeds_per_candidate, anchor_repeats)])
    rows = []
    index = 0
    refused = []
    # default-anchor controls first
    for r in range(anchor_repeats):
        index += 1
        rows.append({'episode_id': _episode_id(index),
                     'candidate_id': 'ANCHOR_R%d' % (r + 1),
                     'factor': 'anchor', 'param': None, 'multiplier': 1.0,
                     'absolute': {}, 'angle_deg': None,
                     'overrides': {}, 'seed': seeds[r if r < len(seeds) else 0]})
    for factor in ('step_length', 'step_frequency', 'swing_height',
                   'double_support_ratio'):
        cands = [c for c in all_candidates if c['factor'] == factor]
        supported_cands = [c for c in cands if c['supported']
                           and abs(c['multipliers'][FACTOR_PARAM[factor]] - 1.0) > 1e-9]
        # pick first (low) and last (high) perturbed candidates
        picked = [supported_cands[0], supported_cands[-1]]
        for cand in picked[:2]:
            for s in range(seeds_per_candidate):
                index += 1
                param = FACTOR_PARAM[factor]
                rows.append({'episode_id': _episode_id(index),
                             'candidate_id': cand['candidate_id'],
                             'factor': factor, 'param': param,
                             'multiplier': cand['multipliers'][param],
                             'absolute': cand['absolute'],
                             'angle_deg': None,
                             'overrides': {param: cand['absolute'][param]},
                             'seed': seeds[s if s < len(seeds) else 0]})
    for cand in all_candidates:
        if not cand['supported']:
            refused.append({'candidate_id': cand['candidate_id'],
                            'factor': cand['factor'],
                            'problems': cand['problems']})
    return {
        'round': 'univariate_smoke',
        'anchor_source': source,
        'anchor_config': anchor,
        'controller_supported_params': sorted(supported),
        'rows': rows,
        'refused': refused,
        'estimated_seconds_per_episode': 55,
        'estimated_total_seconds': len(rows) * 55,
    }


FACTOR_PARAM = {
    'step_length': 'step_length_m',
    'step_frequency': 'step_frequency_hz',
    'swing_height': 'swing_height_m',
    'double_support_ratio': 'double_support_ratio',
}


def build_univariate_full_plan():
    """Full first-round candidate table (planned, not necessarily executed)."""
    anchor, source = load_anchor()
    supported = controller_supported_params()
    cands = build_univariate_candidates(anchor, supported)
    return {'round': 'univariate_full', 'anchor_source': source,
            'candidates': cands,
            'counts': {'supported': sum(1 for c in cands if c['supported']),
                       'refused': sum(1 for c in cands if not c['supported'])},
            'seeds_per_candidate_planned': 3,
            'estimated_seconds_per_episode': 55}


def build_mixed_plan():
    anchor, source = load_anchor()
    supported = controller_supported_params()
    cands = build_mixed_candidates(anchor, supported)
    ratio = mixed_ratio_check(cands)
    return {'round': 'mixed', 'anchor_source': source,
            'ratio': ratio, 'candidates': cands,
            'counts': {'supported': sum(1 for c in cands if c['supported']),
                       'refused': sum(1 for c in cands if not c['supported'])},
            'seeds_per_candidate_planned': 2,
            'estimated_seconds_per_episode': 55}


def full_plan():
    return {
        'default_source': load_anchor()[1],
        'controller_supported_params': sorted(controller_supported_params()),
        'univariate_full': build_univariate_full_plan(),
        'univariate_smoke': build_univariate_smoke_plan(),
        'mixed': build_mixed_plan(),
        'notes': [
            'multipliers are additional provenance only; absolute values are authoritative',
            'torso_pitch_rad is not a controller parameter and is refused, not clamped',
            'seeds are fixed integers; the frozen controller has no RNG',
            'round 2 (mixed) runs only after round 1 smoke passes',
        ],
    }
