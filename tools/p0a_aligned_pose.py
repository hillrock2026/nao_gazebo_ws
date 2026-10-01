#!/usr/bin/env python3
"""P0-a aligned body-pose comparison (offline, deterministic, read-only).

Compares two recorded runs' measured ``body_pose_trace.jsonl`` after explicitly
declared time alignment.  Every output JSON records the full alignment *method*
so the numbers cannot be read without their condition:

  * ``time_reference`` : ``absolute`` (raw simulation time) or ``gait_relative``
    (``t - ready_sim``);
  * ``tolerance``      : maximum matching time gap in seconds;
  * ``interpolation``  : ``none`` | ``nearest`` | ``linear`` | ``hold``;
  * ``pairing``        : how the two series are matched;
  * ``offset_handling``: ``none`` | ``subtract_ready`` | ``estimate`` and whether
    the estimate changes the original time axis (it never does).

Duplicate timestamps are collapsed when identical and rejected when they
conflict; non-monotonic timestamps are sorted and recorded; boundary/out-of-range
and empty inputs are handled explicitly (never a silent empty result).

Usage:
  python3 tools/p0a_aligned_pose.py --run-a <a> --run-b <b> --out <json> \
      [--time-reference absolute] [--tolerance 0.02] \
      [--interpolation none] [--offset-mode none]
"""

import argparse
import bisect
import hashlib
import json
import math
import os
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import p0a_output_guard as OG  # noqa: E402

TOOL_VERSION = 'p0a_aligned_pose:1.0'
AXES = ('x', 'y', 'z')
DEFAULT_TOLERANCE_SEC = 0.02
TIME_REFERENCES = ('absolute', 'gait_relative')
INTERPOLATIONS = ('none', 'nearest', 'linear', 'hold')
OFFSET_MODES = ('none', 'subtract_ready', 'estimate')
PAIRING_BY_INTERPOLATION = {
    'none': 'all_cross_pairs_within_tolerance',
    'nearest': 'nearest_sample_within_tolerance',
    'linear': 'linear_interpolation_at_query_time',
    'hold': 'zero_order_hold_within_tolerance',
}


class AlignedPoseError(Exception):
    pass


def _finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) \
        and math.isfinite(float(v))


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def _load_jsonl(path):
    rows = []
    with open(path, 'r', encoding='utf-8') as fh:
        for line in fh:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def ready_time(run_dir):
    """Ready simulation time from result_p0a.json or result.json labels."""
    for name, getter in (
            ('result_p0a.json', lambda d: d.get('ready_sim')),
            ('result.json', lambda d: (d.get('labels') or {}).get('ready_sim'))):
        path = os.path.join(run_dir, name)
        if os.path.isfile(path):
            with open(path, 'r', encoding='utf-8') as fh:
                value = getter(json.load(fh))
            if _finite(value):
                return float(value)
    raise AlignedPoseError('cannot determine ready_sim in %s' % run_dir)


def load_series(run_dir, time_reference='absolute'):
    """Load, validate, de-duplicate and sort a body_pose series."""
    if time_reference not in TIME_REFERENCES:
        raise AlignedPoseError('invalid time_reference %r' % time_reference)
    path = os.path.join(run_dir, 'trace', 'body_pose_trace.jsonl')
    if not os.path.isfile(path):
        raise AlignedPoseError('body pose trace missing: %s' % path)
    rows = _load_jsonl(path)
    if not rows:
        raise AlignedPoseError('empty body pose trace: %s' % path)
    raw = []
    for i, r in enumerate(rows):
        for field in ('t',) + AXES:
            if not _finite(r.get(field)):
                raise AlignedPoseError(
                    'body row %d invalid %s in %s' % (i, field, path))
        raw.append((float(r['t']), float(r['x']), float(r['y']), float(r['z'])))

    # exact-duplicate collapse; conflicting same-time samples fail explicitly.
    seen = {}
    exact_dups = 0
    conflicts = []
    for t, x, y, z in raw:
        if t in seen:
            if seen[t] == (x, y, z):
                exact_dups += 1
            else:
                conflicts.append(t)
        else:
            seen[t] = (x, y, z)
    if conflicts:
        raise AlignedPoseError(
            'conflicting body pose at identical time in %s: %r'
            % (path, sorted(set(conflicts))[:5]))
    non_monotonic = sum(1 for a, b in zip(raw, raw[1:]) if b[0] < a[0])
    items = sorted(((t, v[0], v[1], v[2]) for t, v in seen.items()),
                   key=lambda e: e[0])
    offset = 0.0
    ready = None
    if time_reference == 'gait_relative':
        ready = ready_time(run_dir)
        offset = ready
    series = [{'t': t - offset, 'x': x, 'y': y, 'z': z}
              for (t, x, y, z) in items]
    diagnostics = {
        'run_dir': run_dir,
        'trace': path,
        'sha256': sha256_file(path),
        'time_reference': time_reference,
        'ready_sim': ready,
        'time_offset_applied_sec': offset,
        'n_rows': len(raw),
        'n_unique_samples': len(series),
        'n_exact_duplicates_removed': exact_dups,
        'n_non_monotonic_input': non_monotonic,
        't_min': series[0]['t'] if series else None,
        't_max': series[-1]['t'] if series else None,
    }
    return series, diagnostics


def _estimate_offset(series_a, series_b, tol):
    """Estimate the constant shift to apply to B that best aligns it with A.

    Coarse deterministic search over the overlapping time span on a 5 ms grid
    (bounded to +/-3 s), using the mean per-axis absolute residual of matched
    nearest samples as the cost.  Comparison-only: source axes are untouched.
    """
    if not series_a or not series_b:
        return 0.0
    tb = [p['t'] for p in series_b]

    def cost(lag):
        total, n = 0.0, 0
        for p in series_a:
            tq = p['t'] - lag
            i = bisect.bisect_left(tb, tq)
            for j in (i - 1, i, i + 1):
                if 0 <= j < len(series_b) and abs(tb[j] - tq) <= tol:
                    q = series_b[j]
                    total += abs(p['x'] - q['x']) + abs(p['y'] - q['y']) \
                        + abs(p['z'] - q['z'])
                    n += 1
                    break
        return total / n if n else None

    best_lag, best_cost = 0.0, None
    for k in range(-600, 601):
        lag = k * 0.005
        c = cost(lag)
        if c is not None and (best_cost is None or c < best_cost):
            best_cost, best_lag = c, lag
    return best_lag


def _interp_at(series, t, field):
    ts = [p['t'] for p in series]
    i = bisect.bisect_right(ts, t) - 1
    if i < 0:
        return None
    if i >= len(series) - 1:
        return series[-1][field]
    a, b = series[i], series[i + 1]
    if b['t'] == a['t']:
        return a[field]
    f = (t - a['t']) / (b['t'] - a['t'])
    return a[field] + f * (b[field] - a[field])


def align(series_a, series_b, tolerance=DEFAULT_TOLERANCE_SEC,
          interpolation='none', offset_info=None):
    """Return per-axis max abs diff plus pairing/omission diagnostics."""
    if interpolation not in INTERPOLATIONS:
        raise AlignedPoseError('invalid interpolation %r' % interpolation)
    if not series_a or not series_b:
        raise AlignedPoseError('cannot align an empty series')

    tb = [p['t'] for p in series_b]

    result = {k: 0.0 for k in AXES}
    argmax = {k: None for k in AXES}
    n_pairs = 0
    unmatched_a = 0
    n_b_matched = set()

    for pa in series_a:
        ta = pa['t']
        if interpolation == 'none':
            lo = bisect.bisect_left(tb, ta - tolerance)
            hi = bisect.bisect_right(tb, ta + tolerance)
            if lo >= hi:
                unmatched_a += 1
                continue
            for j in range(lo, hi):
                n_pairs += 1
                n_b_matched.add(j)
                for k in AXES:
                    d = abs(pa[k] - series_b[j][k])
                    if d > result[k]:
                        result[k] = d
                        argmax[k] = {'t_a': ta, 't_b': series_b[j]['t'],
                                     'a': pa[k], 'b': series_b[j][k]}
        elif interpolation == 'nearest':
            i = bisect.bisect_left(tb, ta)
            cand = [j for j in (i - 1, i, i + 1) if 0 <= j < len(series_b)]
            if not cand:
                unmatched_a += 1
                continue
            j = min(cand, key=lambda j: abs(tb[j] - ta))
            if abs(tb[j] - ta) > tolerance:
                unmatched_a += 1
                continue
            n_pairs += 1
            n_b_matched.add(j)
            for k in AXES:
                d = abs(pa[k] - series_b[j][k])
                if d > result[k]:
                    result[k] = d
                    argmax[k] = {'t_a': ta, 't_b': series_b[j]['t'],
                                 'a': pa[k], 'b': series_b[j][k]}
        elif interpolation == 'linear':
            if ta < tb[0] or ta > tb[-1]:
                unmatched_a += 1
                continue
            n_pairs += 1
            for k in AXES:
                v = _interp_at(series_b, ta, k)
                d = abs(pa[k] - v)
                if d > result[k]:
                    result[k] = d
                    argmax[k] = {'t_a': ta, 't_b': None, 'a': pa[k], 'b': v}
        else:  # hold
            i = bisect.bisect_right(tb, ta) - 1
            if i < 0 or abs(tb[i] - ta) > tolerance:
                unmatched_a += 1
                continue
            n_pairs += 1
            n_b_matched.add(i)
            for k in AXES:
                d = abs(pa[k] - series_b[i][k])
                if d > result[k]:
                    result[k] = d
                    argmax[k] = {'t_a': ta, 't_b': series_b[i]['t'],
                                 'a': pa[k], 'b': series_b[i][k]}
    return {
        'max_abs_diff_m': result,
        'argmax': argmax,
        'n_a': len(series_a),
        'n_b': len(series_b),
        'n_pairs_used': n_pairs,
        'n_a_unmatched': unmatched_a,
        'n_b_unmatched': max(0, len(series_b) - len(n_b_matched))
        if interpolation in ('none', 'nearest', 'hold') else None,
        'n_b_unmatched_note': (
            'n_b_unmatched is null for interpolation=linear (B is sampled, not '
            'indexed)' if interpolation == 'linear' else None),
        'offset_handling': offset_info or {
            'mode': 'none', 'applied_offset_sec': 0.0,
            'changes_original_axis': False},
    }


def compare(run_a, run_b, time_reference='absolute',
            tolerance=DEFAULT_TOLERANCE_SEC, interpolation='none',
            offset_mode='none'):
    if offset_mode not in OFFSET_MODES:
        raise AlignedPoseError('invalid offset_mode %r' % offset_mode)
    if interpolation not in INTERPOLATIONS:
        raise AlignedPoseError('invalid interpolation %r' % interpolation)
    if time_reference not in TIME_REFERENCES:
        raise AlignedPoseError('invalid time_reference %r' % time_reference)
    series_a, diag_a = load_series(run_a, time_reference)
    series_b, diag_b = load_series(run_b, time_reference)
    pairing = PAIRING_BY_INTERPOLATION[interpolation]
    if offset_mode == 'subtract_ready':
        ra, rb = ready_time(run_a), ready_time(run_b)
        series_a = [dict(p, t=p['t'] - ra) for p in series_a]
        series_b = [dict(p, t=p['t'] - rb) for p in series_b]
        offset_info = {'mode': offset_mode, 'a_offset_sec': ra,
                       'b_offset_sec': rb, 'changes_original_axis': False,
                       'note': 'subtract each run ready_sim (comparison only)'}
    elif offset_mode == 'estimate':
        lag = _estimate_offset(series_a, series_b, tolerance)
        series_b = [dict(p, t=p['t'] + lag) for p in series_b]
        offset_info = {'mode': offset_mode, 'applied_offset_sec': lag,
                       'changes_original_axis': False,
                       'note': 'single constant lag on a 5 ms grid, applied to '
                               'B for comparison only'}
    else:
        offset_info = {'mode': 'none', 'applied_offset_sec': 0.0,
                       'changes_original_axis': False,
                       'note': 'no time shift'}
    method = {
        'time_reference': time_reference,
        'tolerance_sec': tolerance,
        'interpolation': interpolation,
        'pairing': pairing,
        'offset_handling': offset_mode,
        'definitions': {
            'time_reference': {
                'absolute': 'raw simulation time t',
                'gait_relative': 't - ready_sim per run',
            }[time_reference],
            'pairing': pairing,
            'offset': ('no time shift' if offset_mode == 'none'
                       else 'subtract each ready_sim' if offset_mode == 'subtract_ready'
                       else 'single constant lag estimated on a 5 ms grid'),
            'boundary': 'query times with no match inside tolerance are counted '
                        'in n_a_unmatched and omitted from the max',
            'empty_input': 'explicit AlignedPoseError (never an empty result)',
            'duplicate_timestamps': 'identical rows collapsed; conflicting '
                                    'same-time rows raise',
            'non_monotonic_timestamps': 'sorted by time; count recorded as '
                                        'n_non_monotonic_input',
            'metric': 'per-axis maximum absolute difference between matched '
                      'poses (metres); not a COM or trajectory error',
        },
    }
    aligned = align(series_a, series_b, tolerance, interpolation, offset_info)
    return {
        'artifact': 'P0-a aligned body-pose comparison',
        'tool': TOOL_VERSION,
        'method': method,
        'inputs': {'a': diag_a, 'b': diag_b},
        'result': aligned,
        'max_abs_diff_m': aligned['max_abs_diff_m'],
    }


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run-a', required=True)
    ap.add_argument('--run-b', required=True)
    ap.add_argument('--time-reference', choices=TIME_REFERENCES,
                    default='absolute')
    ap.add_argument('--tolerance', type=float, default=DEFAULT_TOLERANCE_SEC)
    ap.add_argument('--interpolation', choices=INTERPOLATIONS, default='none')
    ap.add_argument('--offset-mode', choices=OFFSET_MODES, default='none')
    ap.add_argument('--out', required=True)
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv)
    if not (args.tolerance > 0):
        sys.stderr.write('p0a_aligned_pose: --tolerance must be > 0\n')
        return 2
    try:
        report = compare(args.run_a, args.run_b, args.time_reference,
                         args.tolerance, args.interpolation, args.offset_mode)
    except AlignedPoseError as exc:
        sys.stderr.write('p0a_aligned_pose: error: %s\n' % exc)
        return 2
    try:
        OG.write_json(args.out, report, allow_existing=False)
    except OG.OutputPathError as exc:
        sys.stderr.write('p0a_aligned_pose: error: %s\n' % exc)
        return OG.EXIT_OUTPUT_PATH
    print('wrote %s' % args.out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
