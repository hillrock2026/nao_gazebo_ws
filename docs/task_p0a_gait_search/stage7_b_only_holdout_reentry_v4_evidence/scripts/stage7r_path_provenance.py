#!/usr/bin/env python3
"""Stage 7' — path/provenance repair document (offline, read-only over history).

Detects and records the Stage 7 attempt-2 directory-move absolute-path defect:
``runs/attempt2_defective_launchcmd/seed-909001/run_result.json::run_dir_abs``
still points at the pre-move location ``runs/seed-909001``.

Writes Stage7r_path_provenance_repair.json.  It does NOT modify any historical
evidence; it records the mapping and the normalization rule for future runs.
"""

import hashlib
import json
import os
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
EVID = os.path.dirname(_HERE)
WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(_HERE))))
OLD = os.path.join(WS, 'docs', 'task_p0a_gait_search',
                   'stage7_b_only_holdout_confirmation_evidence')


def atomic_json(path, obj):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        json.dump(obj, fh, indent=2, sort_keys=True, allow_nan=False)
    os.replace(tmp, path)


def load(p):
    try:
        return json.load(open(p, encoding='utf-8'))
    except (OSError, ValueError):
        return None


def main():
    a2 = load(os.path.join(OLD, 'runs', 'attempt2_defective_launchcmd', 'seed-909001',
                           'run_result.json'))
    a3 = load(os.path.join(OLD, 'runs', 'seed-909001', 'run_result.json'))
    a2_abs = (a2 or {}).get('run_dir_abs')
    a3_abs = (a3 or {}).get('run_dir_abs')
    a2_expected = os.path.join(OLD, 'runs', 'attempt2_defective_launchcmd', 'seed-909001')
    out = {
        'artifact': "Stage 7' path/provenance repair record",
        'created_at_local': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'defect': {
            'id': 'attempt2_run_dir_abs_stale_after_move',
            'description': ('attempt-2 run directory was moved from runs/seed-909001 to '
                            'runs/attempt2_defective_launchcmd/seed-909001, but its '
                            'run_result.json::run_dir_abs still references the old path.'),
            'run_result_json_sha256': hashlib.sha256(
                open(os.path.join(OLD, 'runs', 'attempt2_defective_launchcmd', 'seed-909001',
                                  'run_result.json'), 'rb').read()).hexdigest(),
            'recorded_run_dir_abs': a2_abs,
            'actual_current_abs': a2_expected,
            'stale': a2_abs != a2_expected,
        },
        'authoritative_attempt3': {
            'recorded_run_dir_abs': a3_abs,
            'actual_current_abs': os.path.join(OLD, 'runs', 'seed-909001'),
            'stale': a3_abs != os.path.join(OLD, 'runs', 'seed-909001'),
        },
        'move_mapping': [
            {'from': os.path.join(OLD, 'runs', 'seed-909001'),
             'to': os.path.join(OLD, 'runs', 'attempt2_defective_launchcmd', 'seed-909001'),
             'attempt': 2, 'seed': 909001},
        ],
        'normalization_rule': ('Future runs record BOTH run_dir_rel (workspace-relative, '
                               'canonical) and run_dir_abs (= os.path.realpath(out_root)). '
                               'Evidence archives must ship a move_mapping; run_result files '
                               'may not silently retain pre-move absolute paths. A relocated '
                               'run must state old->new in an erratum.'),
        'remediation': ('Historical attempt-2 evidence is NOT modified. The mapping in this '
                        'file is the authoritative resolution of the stale reference.'),
        'evidence_modified': False,
    }
    p = os.path.join(EVID, 'Stage7r_path_provenance_repair.json')
    atomic_json(p, out)
    print(json.dumps({'defect_stale': out['defect']['stale'],
                      'attempt3_stale': out['authoritative_attempt3']['stale'],
                      'path': os.path.basename(p)}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
