#!/usr/bin/env python3
"""Task 4 Phase 4B independent summary recomputation from the raw JSONL traces.

Reads ``command_trace.jsonl``, ``body_pose_trace.jsonl`` and
``contact_trace.jsonl`` (and optionally ``state_trace.jsonl``) from a run
directory and recomputes the headline producer-contract and walking metrics
*without* trusting ``phase4b_summary.json``. This demonstrates that the raw
traces are sufficient to derive the result.

Usage: python3 tools/phase4b_recompute_summary.py <run_dir>
"""

import json
import math
import os
import sys


def load(path, kind=None):
    rows = []
    if not os.path.exists(path):
        return rows
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        r = json.loads(line)
        if kind is None or r.get('kind') == kind:
            rows.append(r)
    return rows


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    d = sys.argv[1]

    commands = [json.loads(l) for l in open(os.path.join(d, 'command_trace.jsonl'))
                if l.strip()]
    accepted = [c for c in commands if c.get('ok')]
    rejected = [c for c in commands if not c.get('ok')]
    stamps = [c['t'] for c in accepted]

    bodies = [json.loads(l) for l in open(os.path.join(d, 'body_pose_trace.jsonl'))
              if l.strip()]
    contacts = [json.loads(l) for l in open(os.path.join(d, 'contact_trace.jsonl'))
                if l.strip()] if os.path.exists(os.path.join(d, 'contact_trace.jsonl')) else []

    print('run_dir:', d)
    print('command accepted:', len(accepted), 'rejected:', len(rejected))
    if stamps:
        span = stamps[-1] - stamps[0]
        rate = (len(stamps) - 1) / span if span > 0 else 0.0
        print('command stamps strictly increasing:', all(b > a for a, b in zip(stamps, stamps[1:])))
        print('command stamp min/max: %.3f / %.3f' % (stamps[0], stamps[-1]))
        print('command rate (accepted/span): %.2f Hz' % rate)
    if bodies:
        xs = [b['x'] for b in bodies]
        ys = [b['y'] for b in bodies]
        zs = [b['z'] for b in bodies]
        print('body net forward: %.4f m' % (xs[-1] - xs[0]))
        print('body net displacement: %.4f m' % math.hypot(xs[-1] - xs[0], ys[-1] - ys[0]))
        print('body torso height loss: %.4f m' % (zs[0] - min(zs)))
        print('body max roll/pitch: %.2f / %.2f deg'
              % (math.degrees(max(abs(b['roll']) for b in bodies)),
                 math.degrees(max(abs(b['pitch']) for b in bodies))))
    # contact alternation: merge the interleaved left/right Bool events.
    last = {'left': None, 'right': None}
    alt = 0
    last_pattern = None
    trans = {'left': 0, 'right': 0}
    for c in contacts:
        side = c['side']
        if last[side] is not None and c['value'] != last[side]:
            trans[side] += 1
        last[side] = c['value']
        pattern = (last['left'], last['right'])
        if pattern in ((True, False), (False, True)) and pattern != last_pattern:
            alt += 1
        last_pattern = pattern
    print('contact transitions left/right:', trans['left'], trans['right'])
    print('contact alternation count:', alt)
    return 0


if __name__ == '__main__':
    sys.exit(main())
