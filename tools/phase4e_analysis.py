#!/usr/bin/env python3
"""Single, reusable, testable definitions for Phase 4E / Path B metrics.

Pure Python (no ROS, no Gazebo). The Phase 4E aggregator and the offline
fixture tests both import these functions so that window selection, yaw drift,
contact alternation and config-consistency checks have exactly one definition.

Frozen rules (see docs/task4_phase4e_pathb_evidence/charter/thresholds.json):

* steady window   : (ready + start_delay + gait_blend, ready + start_delay + duration],
                    inclusive endpoints, >= DEFAULT_MIN_STEADY_SAMPLES samples
* stand window    : [ready + STAND_SETTLE_SEC, ready + stand_sec - STAND_TAIL_SEC],
                    inclusive, >= DEFAULT_MIN_STAND_SAMPLES samples
* yaw drift       : unwrap the yaw series by shortest arc; report the signed net
                    change and max |unwrapped - unwrapped[start]|. The gate uses
                    the latter (degrees). Endpoints are never subtracted after
                    wrapping.
* alternation     : per-side transitions debounced by ALTERNATION_DEBOUNCE_SEC,
                    support intervals shorter than MIN_SUPPORT_SEC dropped;
                    alternation_count = number of consecutive support intervals
                    with a different side; fraction = count/(n_support-1).
"""

import math

DEFAULT_MIN_STEADY_SAMPLES = 50
DEFAULT_MIN_STAND_SAMPLES = 200
STAND_SETTLE_SEC = 2.0
STAND_TAIL_SEC = 0.5
ALTERNATION_DEBOUNCE_SEC = 0.08
MIN_SUPPORT_SEC = 0.05
MIN_ALTERNATION_COUNT = 3
MIN_ALTERNATION_FRACTION = 0.6
CONFIG_NUM_TOL = 1e-9


def is_finite_number(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(float(value)))


def steady_window(cfg, ready):
    """Return (lo, hi) of the frozen steady-gait window, endpoints inclusive."""
    start = float(ready) + float(cfg['start_delay_sec'])
    lo = start + float(cfg['gait_blend_sec'])
    hi = start + float(cfg['duration_sec'])
    return lo, hi


def stand_window(cfg, ready):
    """Return (lo, hi) of the frozen stand steady window, endpoints inclusive."""
    lo = float(ready) + STAND_SETTLE_SEC
    hi = float(ready) + float(cfg['stand_sec']) - STAND_TAIL_SEC
    return lo, hi


def select_indices(times, lo, hi):
    return [i for i, t in enumerate(times) if lo <= t <= hi]


def unwrap(values):
    """Unwrap an angle series by the shortest arc (radians)."""
    out = []
    offset = 0.0
    last = None
    for v in values:
        if not is_finite_number(v):
            raise ValueError('non-finite angle in unwrap: %r' % (v,))
        v = float(v)
        if last is not None:
            d = v - last
            if d > math.pi:
                offset -= 2.0 * math.pi
            elif d < -math.pi:
                offset += 2.0 * math.pi
        out.append(v + offset)
        last = v
    return out


def yaw_drift(times, yaws, lo, hi, min_samples=DEFAULT_MIN_STEADY_SAMPLES):
    """Signed net yaw change and max absolute yaw excursion within [lo, hi]."""
    idx = select_indices(times, lo, hi)
    res = {'window_lo': lo, 'window_hi': hi, 'n': len(idx),
           'min_samples': min_samples, 'ok': False, 'reason': None,
           'net_deg': None, 'max_abs_from_start_deg': None}
    if len(idx) < min_samples:
        res['reason'] = 'insufficient_samples'
        return res
    raw = [yaws[i] for i in idx]
    if any(not is_finite_number(v) for v in raw):
        res['reason'] = 'non_finite_yaw'
        return res
    uw = unwrap(raw)
    res['net_deg'] = math.degrees(uw[-1] - uw[0])
    res['max_abs_from_start_deg'] = max(abs(y - uw[0]) for y in uw) * 180.0 / math.pi
    res['ok'] = True
    return res


def alternation(events, debounce=ALTERNATION_DEBOUNCE_SEC,
                min_support=MIN_SUPPORT_SEC):
    """
    Frozen contact-alternation metric.

    ``events`` is an iterable of ``(t, side, value)``; ``side`` in {'left','right'};
    ``value`` truthy means the main-collision contact Bool is fresh/true.
    Returns a dict with per-side support intervals, alternation count/fraction,
    double-support and flight time, and an ``ok`` flag.
    """
    ev = sorted((float(t), side, bool(v)) for t, side, v in events)
    supports = []
    per_side = {'left': [], 'right': []}
    min_interval = max(min_support, debounce)
    for side in ('left', 'right'):
        seq = [(t, v) for (t, s, v) in ev if s == side]
        raw = []
        cur = None
        for t, v in seq:
            if v and cur is None:
                cur = [t, None]
            elif (not v) and cur is not None:
                cur[1] = t
                raw.append((cur[0], cur[1]))
                cur = None
        if cur is not None and seq:
            raw.append((cur[0], seq[-1][0]))
        intervals = [(a, b) for a, b in raw if (b - a) >= min_interval]
        per_side[side] = intervals
        supports.extend((a, b, side) for a, b in intervals)

    supports.sort(key=lambda s: (s[0], s[1]))
    n = len(supports)
    alt = 0
    for i in range(1, n):
        if supports[i][2] != supports[i - 1][2]:
            alt += 1
    fraction = (alt / (n - 1)) if n >= 2 else 0.0

    def _overlap(a, b):
        lo = max(a[0], b[0])
        hi = min(a[1], b[1])
        return max(0.0, hi - lo)

    double_support = 0.0
    for l in per_side['left']:
        for r in per_side['right']:
            double_support += _overlap(l, r)

    # flight time inside the support span
    flight = 0.0
    if supports:
        span_lo = supports[0][0]
        span_hi = max(s[1] for s in supports)
        cover = _merge_cover([(a, b) for a, b, _ in supports])
        flight = (span_hi - span_lo) - sum(b - a for a, b in cover)

    ok = (n >= 4 and alt >= MIN_ALTERNATION_COUNT
          and fraction >= MIN_ALTERNATION_FRACTION)
    reason = None
    if n < 4:
        reason = 'too_few_support_intervals'
    elif alt < MIN_ALTERNATION_COUNT:
        reason = 'too_few_alternations'
    elif fraction < MIN_ALTERNATION_FRACTION:
        reason = 'low_alternation_fraction'
    return {
        'n_support': n,
        'alternation_count': alt,
        'alternation_fraction': fraction,
        'double_support_sec': double_support,
        'flight_sec': flight,
        'per_side_intervals': per_side,
        'ok': ok, 'reason': reason,
        'definition': ('debounce=%.3fs min_support=%.3fs min_alt=%d min_frac=%.2f'
                       % (debounce, min_support, MIN_ALTERNATION_COUNT,
                          MIN_ALTERNATION_FRACTION)),
    }


def _merge_cover(intervals):
    out = []
    for a, b in sorted(intervals):
        if out and a <= out[-1][1]:
            out[-1] = (out[-1][0], max(out[-1][1], b))
        else:
            out.append((a, b))
    return out


def leg_match_fraction(times, idx, wave, lx_by_t):
    """Fraction where the left-arm wave sign matches left-foot forward velocity.

    ``times`` are absolute sample times, ``idx`` the selected (possibly
    non-zero-start) indices into ``times``, ``wave`` the local list aligned with
    ``idx`` (i.e. ``len(wave) == len(idx)``), and ``lx_by_t`` maps absolute time
    to the left-foot forward offset. Returns ``None`` on empty / mismatched /
    insufficient input (explicit failure, no silent fallback).
    """
    if not isinstance(idx, (list, tuple)) or not isinstance(wave, (list, tuple)):
        return None
    if len(idx) != len(wave) or len(idx) < 2:
        return None
    keys = [times[i] for i in idx]
    num = den = 0
    for k in range(1, len(keys)):
        t0, t1 = keys[k - 1], keys[k]
        if t0 in lx_by_t and t1 in lx_by_t and t1 > t0:
            vel = (lx_by_t[t1] - lx_by_t[t0]) / (t1 - t0)
            den += 1
            if wave[k - 1] * vel > 0:
                num += 1
    return (num / den) if den else None


def config_consistency(actual, expected, tol=CONFIG_NUM_TOL):
    """Per-field comparison of a run config against a frozen expected config."""
    diffs = []
    for key in sorted(set(actual) | set(expected)):
        if key not in actual:
            diffs.append({'field': key, 'error': 'missing_in_run_config'})
            continue
        if key not in expected:
            diffs.append({'field': key, 'error': 'extra_unfrozen_key',
                          'actual': actual[key]})
            continue
        a = actual[key]
        e = expected[key]
        if isinstance(a, bool) != isinstance(e, bool):
            diffs.append({'field': key, 'error': 'type_mismatch',
                          'expected': e, 'actual': a})
            continue
        if isinstance(e, bool):
            if a != e:
                diffs.append({'field': key, 'error': 'value_mismatch',
                              'expected': e, 'actual': a})
            continue
        if isinstance(e, (int, float)):
            if not is_finite_number(a):
                diffs.append({'field': key, 'error': 'non_finite_or_type',
                              'expected': e, 'actual': a})
                continue
            if abs(float(a) - float(e)) > tol:
                diffs.append({'field': key, 'error': 'value_mismatch',
                              'expected': e, 'actual': a})
            continue
        if a != e:
            diffs.append({'field': key, 'error': 'value_mismatch',
                          'expected': e, 'actual': a})
    return diffs
