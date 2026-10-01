#!/usr/bin/env python3
"""P0-a forward-motion truth-check tool (offline, deterministic).

Replaces the one-off /tmp script that produced ``truth_check.json``. Every
quantity has one precise, documented definition (see ``metric_definitions`` in
the output) and is covered by ``tools/test_p0a_truth_check.py``.

Inputs (never modified):
  * raw ``trace/body_pose_trace.jsonl`` (measured body pose only);
  * ``run_config.json`` + ``result.json`` (window / ready time).

Usage:
  python3 tools/p0a_truth_check.py --run <run_dir> [--run ...] --out <json>
"""

import argparse
import json
import math
import os
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import analyze_measured_gait as M  # noqa: E402
import phase4e_analysis as A  # noqa: E402

TOOL_VERSION = 'p0a_truth_check:1.0'
DEFAULT_MONOTONIC_TOL_M = 1e-4

METRIC_DEFINITIONS = {
    'window': "phase4e_analysis.steady_window(cfg, ready) = "
              "[ready+start_delay+gait_blend, ready+start_delay+duration]",
    'forward_axis': "window-start measured yaw, unwrapped: "
                    "unit=(cos yaw0, sin yaw0)",
    'projected_forward_series': "p(t) = x(t)*cos(yaw0) + y(t)*sin(yaw0) for "
                                "every body_pose sample inside the window",
    'forward_rel_fit_slope_mps': "ordinary least squares slope of p(t) vs t",
    'forward_rel_fit_r2': "1 - SS_res/SS_tot of that linear fit",
    'max_single_frame_forward_m': "max |p[k+1]-p[k]| over adjacent window pairs",
    'max_single_frame_fraction_of_forward': "max_single_frame_forward_m / "
                                            "forward_displacement_m",
    'monotonic_non_decreasing_fraction': "count(p[k+1] >= p[k] - tol) / "
                                         "(n_window_samples - 1); tol is "
                                         "explicit (default 1e-4 m); null if "
                                         "fewer than 2 samples",
}


def finite(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) \
        and math.isfinite(float(v))


def truth_check(run_dir, monotonic_tol=DEFAULT_MONOTONIC_TOL_M):
    cfg_path = os.path.join(run_dir, 'run_config.json')
    if not os.path.isfile(cfg_path):
        raise M.MeasuredGaitError('run_config.json missing: %s' % run_dir)
    with open(cfg_path, 'r', encoding='utf-8') as fh:
        cfg = json.load(fh)
    for key in ('duration_sec', 'start_delay_sec', 'gait_blend_sec'):
        if not finite(cfg.get(key)):
            raise M.MeasuredGaitError('run_config missing numeric %r' % key)
    trace_dir = os.path.join(run_dir, 'trace')
    ready = M.ready_time(run_dir, trace_dir)
    rows = M.body_series(trace_dir)
    for r in rows:
        for field in ('yaw',):
            if not M.finite(r.get(field)):
                raise M.MeasuredGaitError(
                    'missing/non-finite %s in %s' % (field, trace_dir))
    t = [float(r['t']) for r in rows]
    x = [float(r['x']) for r in rows]
    y = [float(r['y']) for r in rows]
    yaw = [float(r['yaw']) for r in rows]
    lo, hi = A.steady_window(cfg, ready)
    idx = A.select_indices(t, lo, hi)
    if len(idx) < 2:
        raise M.MeasuredGaitError('fewer than 2 window samples: %s' % run_dir)
    yaw0 = A.unwrap([yaw[i] for i in idx])[0]
    fx, fy = math.cos(yaw0), math.sin(yaw0)
    ts = [t[i] for i in idx]
    proj = [x[i] * fx + y[i] * fy for i in idx]
    n = len(ts)
    mt = sum(ts) / n
    mp = sum(proj) / n
    sxx = sum((v - mt) ** 2 for v in ts)
    sxy = sum((v - mt) * (p - mp) for v, p in zip(ts, proj))
    slope = (sxy / sxx) if sxx > 0 else None
    inter = (mp - slope * mt) if slope is not None else None
    ss_res = (sum((p - (inter + slope * v)) ** 2 for v, p in zip(ts, proj))
              if slope is not None else None)
    ss_tot = sum((p - mp) ** 2 for p in proj)
    r2 = (1.0 - ss_res / ss_tot) if (ss_res is not None and ss_tot > 0) else None
    diffs = [abs(b - a) for a, b in zip(proj, proj[1:])]
    max_step = max(diffs) if diffs else None
    forward = proj[-1] - proj[0]
    mono = (sum(1 for a, b in zip(proj, proj[1:])
                if b >= a - monotonic_tol) / (n - 1))
    return {
        'tool': TOOL_VERSION,
        'run_dir': run_dir,
        'ready_sim': ready,
        'forward_axis_rad': yaw0,
        'window': {'lo': lo, 'hi': hi, 'n': n},
        'forward_displacement_m': forward,
        'forward_rel_fit_slope_mps': slope,
        'forward_rel_fit_r2': r2,
        'max_single_frame_forward_m': max_step,
        'max_single_frame_fraction_of_forward':
            (max_step / forward) if (max_step is not None and abs(forward) > 1e-12)
            else None,
        'monotonic_non_decreasing_fraction': mono,
        'monotonic_tol_m': monotonic_tol,
        'n_adjacent_pairs': n - 1,
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--run', action='append', required=True)
    ap.add_argument('--monotonic-tol', type=float, default=DEFAULT_MONOTONIC_TOL_M)
    ap.add_argument('--out', required=True)
    args = ap.parse_args(argv)
    out = {
        'artifact': 'P0-a truth check',
        'tool': TOOL_VERSION,
        'metric_definitions': METRIC_DEFINITIONS,
        'runs': [truth_check(d, args.monotonic_tol) for d in args.run],
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w', encoding='utf-8') as fh:
        json.dump(out, fh, indent=2, sort_keys=True)
    print('wrote %s (%d runs)' % (args.out, len(out['runs'])))
    return 0


if __name__ == '__main__':
    sys.exit(main())
