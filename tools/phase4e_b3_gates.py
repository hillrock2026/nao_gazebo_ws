#!/usr/bin/env python3
"""Gate B3 — frozen per-run hard-gate evaluation.

Extends the frozen Gate B2 gate set (tools/phase4e_metrics.evaluate_gates) with
the new Phase 4B ``body_net_forward_m`` gate. Pure Python; no ROS, no Gazebo.

``body_net_forward_m`` definition (frozen in
docs/task4_phase4e_pathb_b3_evidence/b3_0/hard_gates.json):
  * data source : the run's raw trace/body_pose_trace.jsonl (field ``x``);
  * window      : the full recorded body trace, first sample to last sample
                  (matches phase4b_trace.py ``xs[-1] - xs[0]``);
  * positive    : +x is forward (world frame);
  * endpoints   : first and last sample both included;
  * min samples : 2;
  * missing / non-finite / sample-shortfall => FAIL;
  * pass when body_net_forward_m >= 0.08 (inclusive).
"""

import json
import math
import os
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import phase4e_metrics as M  # noqa: E402

BODY_NET_FORWARD_MIN_M = 0.08
BODY_NET_FORWARD_MIN_SAMPLES = 2


def _finite(v):
    return (isinstance(v, (int, float)) and not isinstance(v, bool)
            and math.isfinite(float(v)))


def body_net_forward(body_rows):
    """Structured net forward displacement over the full body trace."""
    if not isinstance(body_rows, list) or len(body_rows) < BODY_NET_FORWARD_MIN_SAMPLES:
        return M.structured_error('body_trace_insufficient', 'body_net_forward',
                                  detail={'n': len(body_rows) if isinstance(
                                      body_rows, list) else None,
                                      'min_samples': BODY_NET_FORWARD_MIN_SAMPLES})
    xs = [r.get('x') for r in body_rows]
    if any(not _finite(x) for x in xs):
        return M.structured_error('body_net_forward_non_finite', 'body_net_forward')
    value = float(xs[-1]) - float(xs[0])
    return {'ok': True, 'value': value, 'n': len(xs)}


def evaluate_b3_gates(metrics, trace_dir, thresholds, group):
    """Frozen B2 gates + the body_net_forward_m gate for one run."""
    gates = M.evaluate_gates(metrics, thresholds, group)
    body_rows = M.trace_rows(trace_dir, 'body_pose')
    bf = body_net_forward(body_rows)
    if bf.get('ok'):
        add = {'pass': bf['value'] >= BODY_NET_FORWARD_MIN_M,
               'value': bf['value'],
               'detail': {'min_m': BODY_NET_FORWARD_MIN_M,
                          'inclusive': True, 'n': bf['n']}}
    else:
        add = {'pass': False, 'value': None, 'detail': bf}
    gates['body_net_forward_ge_0.08'] = add
    return gates, bf


if __name__ == '__main__':
    print(json.dumps({'module': 'phase4e_b3_gates',
                      'body_net_forward_min_m': BODY_NET_FORWARD_MIN_M}))
