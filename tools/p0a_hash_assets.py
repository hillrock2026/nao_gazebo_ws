#!/usr/bin/env python3
"""P0-a protected-asset SHA-256 baseline/compare tool (read-only, offline).

Computes a deterministic hash snapshot of the assets P0-a must never modify:

  * the four historical Phase 2A datasets (aggregate ``relpath:sha256``);
  * the four formal Phase 2E reports (per-file sha256 + size);
  * the two Phase 2F policy documents (per-file sha256 + size);
  * the batch-0 candidate tree (aggregate + file count);
  * the recorded fwdf08 P0-a reproduce run directories.

Usage:
  python3 tools/p0a_hash_assets.py write  --out <baseline.json>
  python3 tools/p0a_hash_assets.py compare --before <a.json> --after <b.json>
"""

import argparse
import hashlib
import json
import os
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(_TOOLS)
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)

DATASETS = {
    'phase2b_preliminary': 'docs/task4_phase2b_baseline_a_evidence/work/dataset',
    'phase2b_smoke': 'docs/task4_phase2b_baseline_a_smoke_evidence/work/dataset',
    'phase2c_univariate':
        'docs/task4_phase2c_baseline_b_evidence/univariate/work/dataset',
    'phase2d_boundary':
        'docs/task4_phase2d_active_failure_boundary_evidence/boundary/work/dataset',
}
REPORTS_DIR = 'docs/task4_phase2e_labels_quality_evidence/reports'
POLICY_DOCS = ['docs/Phase2F_Split_Policy.md', 'docs/Phase2F_Evaluation_Goals.md']
BATCH0 = 'docs/task5_gate2a_batch0_candidate'
FWDF08_RUNS = [
    'docs/task_p0a_gait_search/work/reproduce/fwdf08_seed101',
    'docs/task_p0a_gait_search/work/reproduce/fwdf08_seed202',
    'docs/task_p0a_gait_search/work/reproduce/fwdf08_seed303',
]
# P1-R addition: protected old-P1 and legacy B3 artifacts.
OLD_P1_DIR = 'docs/task_p0a_gait_search/p1_support_swing'
B3_EVIDENCE = 'docs/task4_phase4e_pathb_b3_evidence'
OLD_P0A_DOCS = [
    'docs/task_p0a_gait_search/forward_candidate_charter.md',
    'docs/task_p0a_gait_search/forward_candidate_charter_erratum.md',
    'docs/task_p0a_gait_search/measured_contract.md',
    'docs/task_p0a_gait_search/P0-Audit.md',
    'docs/task_p0a_gait_search/Forward_Gait_Execution_Report.md',
]


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def aggregate_root(root):
    """(aggregate_sha256, file_count) over sorted ``relpath:sha256``."""
    lines = []
    for base, _, files in os.walk(root):
        for name in files:
            path = os.path.join(base, name)
            lines.append('%s:%s' % (os.path.relpath(path, root),
                                    sha256_file(path)))
    return hashlib.sha256('\n'.join(sorted(lines)).encode()).hexdigest(), len(lines)


def file_entry(rel):
    p = os.path.join(WS, rel)
    return {'path': rel, 'size_bytes': os.path.getsize(p),
            'sha256': sha256_file(p)}


def snapshot():
    out = {'workspace': WS, 'datasets': {}, 'reports': [], 'policy_docs': [],
           'batch0': {}, 'fwdf08_runs': [], 'old_p1': {}, 'b3_evidence': {},
           'old_p0a_docs': []}
    for name, rel in DATASETS.items():
        agg, count = aggregate_root(os.path.join(WS, rel))
        out['datasets'][name] = {'root': rel, 'file_count': count,
                                 'aggregate_sha256': agg}
    for name in sorted(os.listdir(os.path.join(WS, REPORTS_DIR))):
        if name.endswith('.json'):
            out['reports'].append(file_entry(os.path.join(REPORTS_DIR, name)))
    for rel in POLICY_DOCS:
        out['policy_docs'].append(file_entry(rel))
    agg, count = aggregate_root(os.path.join(WS, BATCH0))
    out['batch0'] = {'root': BATCH0, 'file_count': count,
                     'aggregate_sha256': agg}
    for rel in FWDF08_RUNS:
        agg, count = aggregate_root(os.path.join(WS, rel))
        out['fwdf08_runs'].append({'root': rel, 'file_count': count,
                                   'aggregate_sha256': agg})
    agg, count = aggregate_root(os.path.join(WS, OLD_P1_DIR))
    out['old_p1'] = {'root': OLD_P1_DIR, 'file_count': count,
                     'aggregate_sha256': agg}
    agg, count = aggregate_root(os.path.join(WS, B3_EVIDENCE))
    out['b3_evidence'] = {'root': B3_EVIDENCE, 'file_count': count,
                          'aggregate_sha256': agg}
    for rel in OLD_P0A_DOCS:
        out['old_p0a_docs'].append(file_entry(rel))
    return out


def _flatten(snap):
    flat = {}
    for name, d in snap['datasets'].items():
        flat['dataset:%s:aggregate' % name] = d['aggregate_sha256']
        flat['dataset:%s:file_count' % name] = d['file_count']
    for e in snap['reports']:
        flat['report:%s' % os.path.basename(e['path'])] = e['sha256']
    for e in snap['policy_docs']:
        flat['policy:%s' % os.path.basename(e['path'])] = e['sha256']
    flat['batch0:aggregate'] = snap['batch0']['aggregate_sha256']
    flat['batch0:file_count'] = snap['batch0']['file_count']
    for e in snap['fwdf08_runs']:
        flat['fwdf08:%s' % os.path.basename(e['root'])] = e['aggregate_sha256']
    if snap.get('old_p1'):
        flat['old_p1:aggregate'] = snap['old_p1']['aggregate_sha256']
        flat['old_p1:file_count'] = snap['old_p1']['file_count']
    if snap.get('b3_evidence'):
        flat['b3_evidence:aggregate'] = snap['b3_evidence']['aggregate_sha256']
        flat['b3_evidence:file_count'] = snap['b3_evidence']['file_count']
    for e in snap.get('old_p0a_docs', []):
        flat['old_p0a:%s' % os.path.basename(e['path'])] = e['sha256']
    return flat


def compare(before, after):
    fb, fa = _flatten(before), _flatten(after)
    keys = sorted(set(fb) | set(fa))
    diffs = []
    for k in keys:
        if fb.get(k) != fa.get(k):
            diffs.append({'key': k, 'before': fb.get(k), 'after': fa.get(k)})
    return {'match': not diffs, 'n_keys': len(keys), 'diffs': diffs}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest='cmd', required=True)
    w = sub.add_parser('write')
    w.add_argument('--out', required=True)
    c = sub.add_parser('compare')
    c.add_argument('--before', required=True)
    c.add_argument('--after', required=True)
    args = ap.parse_args(argv)
    if args.cmd == 'write':
        snap = snapshot()
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, 'w', encoding='utf-8') as fh:
            json.dump(snap, fh, indent=2, sort_keys=True)
        print('wrote %s' % args.out)
        return 0
    with open(args.before) as fh:
        before = json.load(fh)
    with open(args.after) as fh:
        after = json.load(fh)
    result = compare(before, after)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result['match'] else 1


if __name__ == '__main__':
    sys.exit(main())
