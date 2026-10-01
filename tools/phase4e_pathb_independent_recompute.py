#!/usr/bin/env python3
"""Independent recompute of the Gate B2 headline metrics from raw traces.

Deliberately does NOT import phase4e_analysis/phase4e_metrics: it re-derives
steady dx, unwrapped yaw excursion and config equality with its own short code,
then diffs against the stored run_metrics.json / run_gates.json.

Usage: python3 tools/phase4e_pathb_independent_recompute.py
"""

import json
import math
import os

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EV = os.path.join(WS, 'docs', 'task4_phase4e_pathb_evidence')


def load(path, default=None):
    try:
        with open(path) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return default


def read_jsonl(path):
    rows = []
    with open(path) as fh:
        for line in fh:
            if line.strip():
                rows.append(json.loads(line))
    return rows


def unwrap(seq):
    out = []
    off = 0.0
    last = None
    for v in seq:
        if last is not None:
            d = v - last
            if d > math.pi:
                off -= 2 * math.pi
            elif d < -math.pi:
                off += 2 * math.pi
        out.append(v + off)
        last = v
    return out


def main():
    order = load(os.path.join(EV, 'charter', 'run_order.json'))
    frozen = load(os.path.join(EV, 'charter', 'frozen_configs.json'))
    lines = []
    ok_all = True
    for label in order['order']:
        group = order['group_of'][label]
        rd = os.path.join(EV, order['run_dir'][label])
        res = load(os.path.join(rd, 'run_result.json'))
        cfg = load(os.path.join(rd, 'run_config.json'))
        stored = load(os.path.join(rd, 'run_metrics.json'))
        ready = res['ready_sim']
        lo = ready + cfg['start_delay_sec'] + cfg['gait_blend_sec']
        hi = ready + cfg['start_delay_sec'] + cfg['duration_sec']
        body = read_jsonl(os.path.join(rd, 'trace', 'body_pose_trace.jsonl'))
        win = [b for b in body if lo <= b['t'] <= hi]
        dx = win[-1]['x'] - win[0]['x']
        uw = unwrap([b.get('yaw', 0.0) for b in win])
        yaw = max(abs(y - uw[0]) for y in uw) * 180.0 / math.pi
        cfg_eq = (cfg == frozen[group])
        d_dx = abs(dx - stored['body_dx'])
        d_yaw = abs(yaw - stored['max_yaw_abs_from_start_deg'])
        row_ok = cfg_eq and d_dx < 1e-9 and d_yaw < 1e-6
        ok_all = ok_all and row_ok
        lines.append('%-18s dx %.9f (stored %.9f, d=%.2e)  yaw %.6f '
                     '(stored %.6f, d=%.2e)  cfg_eq=%s  n=%d  %s'
                     % (label, dx, stored['body_dx'], d_dx, yaw,
                        stored['max_yaw_abs_from_start_deg'], d_yaw, cfg_eq,
                        len(win), 'OK' if row_ok else 'MISMATCH'))
    lines.append('')
    lines.append('independent recompute: %s' % ('PASS' if ok_all else 'MISMATCH'))
    text = '\n'.join(lines)
    print(text)
    with open(os.path.join(EV, 'independent_recompute.txt'), 'w') as fh:
        fh.write(text + '\n')
    return 0 if ok_all else 1


if __name__ == '__main__':
    raise SystemExit(main())
