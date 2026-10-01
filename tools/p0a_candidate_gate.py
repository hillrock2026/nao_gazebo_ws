#!/usr/bin/env python3
"""P0-a forward-gait candidate gate (frozen charter, offline, deterministic).

Evaluates one or more ``result_p0a.json`` files produced by
``tools/run_p0a_gait_candidate.py`` against the pre-registered rules in
``docs/task_p0a_gait_search/forward_candidate_charter.md``. Uses measured body
pose only (never cmd_vel / parameter names).

Usage:
  python3 tools/p0a_candidate_gate.py --seed 101 --result <r101.json> \
      --seed 202 --result <r202.json> --seed 303 --result <r303.json>
"""

import argparse
import json
import math
import os
import sys

# Frozen charter thresholds (forward_candidate_charter.md section 5).
MIN_FORWARD_M = 0.08
MAX_LATERAL_TO_FORWARD = 0.5
MAX_ABS_YAW_DEG = 30.0
MAX_TORSO_TILT_DEG = 35.0
MIN_EFFECTIVE_DURATION_S = 8.0
MAX_MISSING_DURATION_S = 1.0
MAX_PATH_OVER_NET = 8.0
MAX_SINGLE_STEP_M = 0.02
MIN_CONTACT_ALTERNATIONS = 3
MIN_SWING_CLEARANCE_M = 0.002
REQUIRED_SEEDS = (101, 202, 303)


def _finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) \
        and math.isfinite(float(v))


def evaluate(records):
    per_seed = []
    for rec in records:
        seed = rec['seed']
        d = rec.get('data') or {}
        m = d.get('measured') or {}
        s = d.get('phase4b_summary') or {}
        ann = [a.get('type') for a in (m.get('anomalies') or [])]
        fwd = m.get('forward_displacement_m')
        lat = m.get('lateral_displacement_m')
        yaw = m.get('yaw_change_deg')
        tilt = m.get('torso_tilt_max_deg')
        path_net = m.get('path_over_net')
        step = m.get('max_single_step_m')
        eff = m.get('effective_duration_sec')
        miss = m.get('missing_duration_sec')
        contact = m.get('contact') or {}
        alt = contact.get('alternation_count')
        lh = s.get('l_foot_swing_height')
        rh = s.get('r_foot_swing_height')
        swing = max([v for v in (lh, rh) if _finite(v)] or [0.0])
        checks = {
            'ok': bool(d.get('ok')),
            'measurable': m.get('measurable') is True,
            'no_heavy_anomaly': not any(t in (
                'non_monotonic_timestamp', 'irregular_sample_interval',
                'attitude_field_missing', 'attitude_field_nonfinite') for t in ann),
            'forward_gt_min': _finite(fwd) and fwd > MIN_FORWARD_M,
            'forward_positive': _finite(fwd) and fwd > 0.0,
            'lateral_not_dominant': (_finite(fwd) and _finite(lat) and fwd > 0
                                     and abs(lat) / fwd <= MAX_LATERAL_TO_FORWARD),
            'yaw_ok': _finite(yaw) and abs(yaw) <= MAX_ABS_YAW_DEG,
            'tilt_ok': _finite(tilt) and tilt <= MAX_TORSO_TILT_DEG,
            'no_fall': m.get('fall') is False,
            'duration_ok': _finite(eff) and eff >= MIN_EFFECTIVE_DURATION_S,
            'not_severely_truncated': _finite(miss) and miss <= MAX_MISSING_DURATION_S,
            'path_over_net_ok': _finite(path_net) and path_net <= MAX_PATH_OVER_NET,
            'single_step_ok': _finite(step) and step <= MAX_SINGLE_STEP_M,
            'contact_ok': _finite(alt) and alt >= MIN_CONTACT_ALTERNATIONS,
            'swing_clearance_ok': swing > MIN_SWING_CLEARANCE_M,
        }
        per_seed.append({'seed': seed, 'checks': checks,
                         'forward': fwd, 'lateral': lat, 'yaw': yaw,
                         'tilt': tilt, 'path_over_net': path_net,
                         'effective_duration': eff, 'swing_clearance': swing,
                         'contact_alternations': alt})
    seeds = [p['seed'] for p in per_seed]
    all_pass = all(all(p['checks'].values()) for p in per_seed)
    fwds = [p['forward'] for p in per_seed if _finite(p['forward'])]
    directions_consistent = bool(fwds) and all(f > 0 for f in fwds)
    mean_f = sum(fwds) / len(fwds) if fwds else None
    std_f = (math.sqrt(sum((f - mean_f) ** 2 for f in fwds) / len(fwds))
             if len(fwds) >= 2 else None)
    return {
        'required_seeds': list(REQUIRED_SEEDS),
        'seeds_present': seeds,
        'seed_count_ok': len(seeds) == len(REQUIRED_SEEDS)
        and sorted(seeds) == sorted(REQUIRED_SEEDS),
        'directions_consistent': directions_consistent,
        'all_seeds_pass': all_pass,
        'pass': bool(all_pass and directions_consistent
                     and len(seeds) == len(REQUIRED_SEEDS)
                     and sorted(seeds) == sorted(REQUIRED_SEEDS)),
        'mean_forward_m': mean_f,
        'std_forward_m': std_f,
        'per_seed': per_seed,
    }


def _load_result(path):
    if not os.path.isfile(path):
        raise SystemExit('p0a_candidate_gate: result file missing: %s' % path)
    try:
        with open(path, 'r', encoding='utf-8') as fh:
            return json.load(fh)
    except ValueError as exc:
        raise SystemExit('p0a_candidate_gate: result file is not valid JSON: '
                         '%s (%s)' % (path, exc))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--result', action='append', required=True)
    ap.add_argument('--seed', action='append', type=int, required=True)
    ap.add_argument('--out', default=None)
    args = ap.parse_args(argv)
    if len(args.result) != len(args.seed):
        raise SystemExit('--result and --seed must be given in equal counts')
    records = []
    for path, seed in zip(args.result, args.seed):
        records.append({'seed': seed, 'data': _load_result(path)})
    verdict = evaluate(records)
    print(json.dumps(verdict, indent=2, sort_keys=True))
    if args.out:
        with open(args.out, 'w', encoding='utf-8') as fh:
            json.dump(verdict, fh, indent=2, sort_keys=True)
    return 0 if verdict['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
