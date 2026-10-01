#!/usr/bin/env python3
"""Gate B3.0 — read-only B2 keywise metrics regression helper.

Recomputes the frozen per-run metrics for the nine persisted Gate B2 runs from
their raw traces (never writes to, or overwrites, the B2 evidence tree), and can
recursively compare two such captures key by key.

This tool is intentionally separate from the production analysis so that a
"before" capture (pre-change source) and an "after" capture (post-change source)
can be compared without touching any frozen artifact.

Usage:
  python3 tools/b3_0_b2_keywise_regression.py capture --out CAPTURE.json
  python3 tools/b3_0_b2_keywise_regression.py compare --before BEFORE.json \
        --after AFTER.json --out DIFF.json
Exit 0 = capture ok / comparison has no unexplained differences.
"""

import argparse
import json
import os
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import phase4e_metrics as M  # noqa: E402

WS = os.path.dirname(_TOOLS)
EV = os.path.join(WS, 'docs', 'task4_phase4e_pathb_evidence')


def _load(path):
    with open(path) as fh:
        return json.load(fh)


def capture(args):
    order = _load(os.path.join(EV, 'charter', 'run_order.json'))
    frozen = _load(os.path.join(EV, 'charter', 'frozen_configs.json'))
    runs = {}
    for label in order['order']:
        run_dir = os.path.join(EV, order['run_dir'][label])
        cfg = frozen[order['group_of'][label]]
        res = _load(os.path.join(run_dir, 'run_result.json'))
        ready = res['ready_sim']
        m = M.run_metrics(os.path.join(run_dir, 'trace'), cfg, ready, cfg,
                          mode='walk')
        m['run_id'] = label
        m['group'] = order['group_of'][label]
        runs[label] = m
    out = {
        'tool': 'b3_0_b2_keywise_regression.capture',
        'runs_root': EV,
        'order': order['order'],
        'runs': runs,
    }
    with open(args.out, 'w') as fh:
        json.dump(out, fh, indent=2, sort_keys=True)
    print('captured %d runs -> %s' % (len(runs), args.out))
    return 0


def _compare(before, after, path, new_keys, diffs):
    if isinstance(before, dict) and isinstance(after, dict):
        for key in sorted(set(before) | set(after)):
            if key not in before:
                new_keys.append({'path': path + '/' + str(key),
                                 'value': after[key]})
                continue
            if key not in after:
                diffs.append({'path': path + '/' + str(key),
                              'error': 'missing_in_after',
                              'before': before[key]})
                continue
            _compare(before[key], after[key], path + '/' + str(key),
                     new_keys, diffs)
        return
    if isinstance(before, list) and isinstance(after, list):
        if len(before) != len(after):
            diffs.append({'path': path, 'error': 'list_length',
                          'before_len': len(before), 'after_len': len(after)})
            return
        for i, (b, a) in enumerate(zip(before, after)):
            _compare(b, a, '%s[%d]' % (path, i), new_keys, diffs)
        return
    # scalar: value and type must match exactly; floats compare exactly
    if type(before) is not type(after):
        diffs.append({'path': path, 'error': 'type_changed',
                      'before': before, 'after': after,
                      'before_type': type(before).__name__,
                      'after_type': type(after).__name__})
        return
    if before != after:
        diffs.append({'path': path, 'error': 'value_changed',
                      'before': before, 'after': after})


def compare(args):
    before = _load(args.before)
    after = _load(args.after)
    new_keys = []
    diffs = []
    _compare(before, after, '', new_keys, diffs)
    per_run = {}
    for label in before.get('runs', {}):
        b = before['runs'][label]
        a = after.get('runs', {}).get(label)
        if a is None:
            per_run[label] = {'present_after': False}
            continue
        nk, df = [], []
        _compare(b, a, label, nk, df)
        per_run[label] = {'present_after': True, 'new_keys': len(nk),
                          'differences': len(df)}
    out = {
        'before': args.before, 'after': args.after,
        'new_key_count': len(new_keys),
        'difference_count': len(diffs),
        'new_keys': new_keys,
        'differences': diffs,
        'per_run': per_run,
    }
    with open(args.out, 'w') as fh:
        json.dump(out, fh, indent=2, sort_keys=True)
    print('new keys: %d ; differences: %d ; written %s'
          % (len(new_keys), len(diffs), args.out))
    return 0 if not diffs else 1


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('capture')
    c.add_argument('--out', required=True)
    c.set_defaults(func=capture)
    d = sub.add_parser('compare')
    d.add_argument('--before', required=True)
    d.add_argument('--after', required=True)
    d.add_argument('--out', required=True)
    d.set_defaults(func=compare)
    args = ap.parse_args()
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
