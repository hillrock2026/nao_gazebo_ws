#!/usr/bin/env python3
"""P0-a seed-diagnostic replay fixtures (offline, deterministic, read-only).

Proves the replay/regenerate/compare tooling:
  * regenerates the seeddiag r1/r2 repeatability result from the raw traces;
  * reproduces every frozen field value (gate/verdict unchanged);
  * is deterministic across repeated runs (excluding the timestamp);
  * detects semantic / gate / verdict changes and float-boundary differences;
  * fails explicitly when a run directory is missing.

Usage: python3 tools/test_p0a_seeddiag_replay.py [--json-out F]
"""

import argparse
import copy
import json
import os
import shutil
import sys
import tempfile

_TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(_TOOLS)
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import p0a_seeddiag_replay as SR  # noqa: E402

P1R = os.path.join(WS, 'docs', 'task_p0a_gait_search', 'p1r_support_swing')
RUNS_ROOT = os.path.join(P1R, 'runs')
GATES = os.path.join(P1R, 'frozen_gates_v2.json')
# Compare against the pre-Stage-0 original: the in-tree result was extended with
# the aligned-pose method block (Stage-0 0.1.3), so the frozen numeric baseline
# is preserved as the prechange copy.
PRECHANGE = os.path.join(WS, 'docs', 'task_p0a_gait_search', 'p1bd_repair',
                         'seed_repeatability_result.prechange.json')
FROZEN = PRECHANGE if os.path.isfile(PRECHANGE) else \
    os.path.join(P1R, 'seed_repeatability_result.json')
RESULTS = []


def check(name, cond, detail=None):
    RESULTS.append({'name': name, 'pass': bool(cond), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                           '' if cond else ' :: %r' % (detail,)))


def _strip_time(obj):
    obj = copy.deepcopy(obj)
    obj.pop('generated_at', None)
    obj.get('notes', {}).pop('generated_at', None)
    return obj


def test_regenerate_and_replay():
    new = SR.generate(RUNS_ROOT, GATES)
    old = json.load(open(FROZEN))
    check('regen_physics_seed', new['physics_seed'] == old['physics_seed'],
          new['physics_seed'])
    check('regen_r1_gate_fail',
          new['runs']['seeddiag_r1']['gate_pass'] is False
          and new['runs']['seeddiag_r1']['gate_fails'] == ['swing_to_support_onset_measured'],
          new['runs']['seeddiag_r1']['gate_fails'])
    check('regen_r2_gate_pass',
          new['runs']['seeddiag_r2']['gate_pass'] is True
          and new['runs']['seeddiag_r2']['gate_fails'] == [],
          new['runs']['seeddiag_r2']['gate_fails'])
    check('regen_method_b_missing',
          new['runs']['seeddiag_r1']['method_b_onset']['n_missing'] == 1
          and new['runs']['seeddiag_r2']['method_b_onset']['n_missing'] == 0)
    check('regen_aligned_triple',
          new['aligned_body_pose_max_abs_diff_m']
          == old['aligned_body_pose_max_abs_diff_m'],
          new['aligned_body_pose_max_abs_diff_m'])
    check('regen_byte_equality_nondeterministic',
          all(not v['identical'] for v in new['byte_equality'].values()))

    diff = SR.compare(old, new)
    check('replay_verdict_pass', diff['verdict'] == 'PASS', diff['verdict'])
    check('replay_no_semantic_diffs', diff['n_semantic_diffs'] == 0, diff)
    check('replay_gate_unchanged', diff['gate_changed'] is False)
    check('replay_verdict_unchanged', diff['verdict_changed'] is False)
    check('replay_only_added_metadata',
          all(d['kind'] == 'added' for d in diff['diffs']), diff['diffs'])

    # determinism ignoring the timestamp
    new2 = SR.generate(RUNS_ROOT, GATES)
    check('regen_deterministic',
          _strip_time(new) == _strip_time(new2))
    return new, diff


def test_diff_detection():
    old = json.load(open(FROZEN))

    tiny = copy.deepcopy(old)
    tiny['runs']['seeddiag_r1']['forward_displacement_m'] += 1e-15
    check('tiny_float_within_tolerance',
          SR.compare(old, tiny)['n_semantic_diffs'] == 0)

    changed = copy.deepcopy(old)
    changed['runs']['seeddiag_r1']['forward_displacement_m'] += 1e-3
    d = SR.compare(old, changed)
    check('numeric_change_blocked',
          d['verdict'] == 'BLOCKED' and d['n_semantic_diffs'] > 0
          and any(x['affects_gate'] for x in d['diffs']), d['reason'])

    verdict = copy.deepcopy(old)
    verdict['runs']['seeddiag_r1']['gate_pass'] = True
    d2 = SR.compare(old, verdict)
    check('verdict_change_blocked',
          d2['verdict'] == 'BLOCKED' and d2['verdict_changed'] is True,
          d2['reason'])

    missing = copy.deepcopy(old)
    del missing['runs']['seeddiag_r1']['method_b_onset']
    d3 = SR.compare(old, missing)
    check('removed_field_blocked', d3['verdict'] == 'BLOCKED', d3['reason'])


def test_missing_run_fails():
    tmp = tempfile.mkdtemp(prefix='p1r_sd_')
    try:
        try:
            SR.generate(tmp, GATES)
            check('missing_run_fails', False)
        except SR.SeedDiagError:
            check('missing_run_fails', True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('P0-a seed-diagnostic replay fixtures')
    new, diff = test_regenerate_and_replay()
    test_diff_detection()
    test_missing_run_fails()
    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\nseed-diagnostic replay fixtures: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w') as fh:
            json.dump({'passed': passed, 'total': total, 'results': RESULTS},
                      fh, indent=2, sort_keys=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
