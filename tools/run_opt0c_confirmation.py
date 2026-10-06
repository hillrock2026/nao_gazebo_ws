#!/usr/bin/env python3
"""OPT-0C — best-vs-baseline confirmation experiment (no optimization).

Runs a paired (Common Random Numbers) comparison of two FIXED θ vectors:

    baseline θ = ParameterSpace.anchor_normalized()      (the frozen anchor)
    best θ     = optimization_results/best_theta.yaml    (the OPT-0 best)

for SEEDS = [101, 202, 303] -> 6 Gazebo episodes. This driver does not optimize
anything; it only reuses the audited frozen runner and the audited evaluator.

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    python3 -u tools/run_opt0c_confirmation.py

Outputs (does NOT touch optimization_results/episodes or best_*.json):
    optimization_results/confirmation/episodes/conf_{group}_s{seed}/
    optimization_results/confirmation/rows.json
    optimization_results/confirmation/summary.json
"""

import hashlib
import json
import os
import statistics
import sys
import time

import yaml

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS = os.path.join(WS, 'tools')
for _p in (WS, TOOLS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_p0a_gait_candidate as R  # noqa: E402  (FROZEN — import only)
from optimization.parameter_space import ParameterSpace  # noqa: E402
from optimization.fitness import FitnessEvaluator  # noqa: E402

SEEDS = [101, 202, 303]
TIMEOUT = 200.0
DOMAIN_ID = 77
BASELINE_YAML = os.path.join(WS, 'config', 'gait_opt_baseline.yaml')
BEST_THETA_YAML = os.path.join(WS, 'optimization_results', 'best_theta.yaml')
OUT = os.path.join(WS, 'optimization_results', 'confirmation')

FROZEN = {
    'src/nao_gz_sim/scripts/phase4b_producer.py':
        'aab60f6fbb18b657736ebf9d7633e65b3d0df021c564cddb537bb806875bd5eb',
    'tools/run_p0a_gait_candidate.py':
        '00aef7129d65ad78b1bbf219bcec9e9354547c6c0bd8d1ba4fc5948936b820af',
    'src/nao_gz_sim/scripts/nao_gait_phase.py':
        '4cee76de5db06f7276b9c9803738caa857f0da2d16fbe526d2d86eeb1b6b10fc',
}
ANCHOR = 'docs/task4_phase4e_pathb_b3_evidence/b3_0/candidates.json'
ANCHOR_SHA = '116dde704ba2d4bbf3f5afaaefc0c5a77a478541d341a86536e38ac0426bc13b'


def sha256(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def preflight():
    problems = []
    for rel, expect in FROZEN.items():
        got = sha256(os.path.join(WS, rel))
        if got != expect:
            problems.append('frozen hash mismatch: %s' % rel)
    if sha256(os.path.join(WS, ANCHOR)) != ANCHOR_SHA:
        problems.append('anchor hash mismatch')
    if not os.path.isfile(BEST_THETA_YAML):
        problems.append('best_theta.yaml missing')
    if os.path.exists(OUT):
        problems.append('confirmation output dir already exists: %s' % OUT)
    if not R.shutil.which('ros2'):
        problems.append('ros2 not on PATH (source ROS first)')
    return problems


def build_evaluator():
    cfg = yaml.safe_load(open(BASELINE_YAML, 'r', encoding='utf-8')) or {}
    fit = cfg.get('fitness', {}) or {}
    refs = {k: v for k, v in fit.items()
            if k.startswith(('V_ref', 'tilt_ref', 'yaw_ref', 'pon_ref', 'step_ref'))}
    return FitnessEvaluator(refs=refs, weights=fit.get('weights'),
                            penalty_fall=fit.get('penalty_fall', -1.0),
                            penalty_invalid=fit.get('penalty_invalid', -2.0),
                            score_scale=fit.get('score_scale', 0.3))


def load_thetas():
    sp = ParameterSpace()
    baseline = sp.from_normalized(sp.anchor_normalized())
    best = {k: float(v) for k, v in
            yaml.safe_load(open(BEST_THETA_YAML, 'r', encoding='utf-8'))['best']['overrides'].items()}
    return sp, {'baseline': baseline, 'best': best}


def run_episode(group, seed, overrides, ev, sp):
    cand_id = 'conf_%s_s%d' % (group, seed)
    ep = os.path.join(OUT, 'episodes', cand_id)
    R.run_candidate(cand_id, seed, ep, overrides, TIMEOUT, DOMAIN_ID,
                    allow_overwrite=True, gz_seed=seed)
    fr = ev.evaluate_result(ep)
    m = fr.detail.get('raw_metrics', {})
    # raw measured block for the provenance row
    try:
        raw = json.load(open(os.path.join(ep, 'result_p0a.json')))['measured'] or {}
    except Exception:  # noqa: BLE001
        raw = {}
    row = {
        'group': group, 'seed': seed, 'candidate_id': cand_id,
        'fitness': fr.fitness, 'status': fr.status,
        'speed_mps': raw.get('actual_speed_mean_mps'),
        'forward_m': raw.get('forward_displacement_m'),
        'tilt_deg': raw.get('torso_tilt_max_deg'),
        'yaw_deg': raw.get('yaw_change_deg'),
        'fall': bool(raw.get('fall')),
        'actual_speed_std_mps': raw.get('actual_speed_std_mps'),
        'path_over_net': raw.get('path_over_net'),
        'max_single_step_m': raw.get('max_single_step_m'),
        'lateral_displacement_m': raw.get('lateral_displacement_m'),
        'net_displacement_m': raw.get('net_displacement_m'),
        'effective_duration_sec': raw.get('effective_duration_sec'),
        'trace_truncated': raw.get('trace_truncated'),
        'sim_result_path': os.path.relpath(os.path.join(ep, 'result_p0a.json'), WS),
    }
    return row


def _stats(vals):
    vals = [v for v in vals if isinstance(v, (int, float))]
    if not vals:
        return {'n': 0, 'mean': None, 'std': None, 'min': None, 'max': None}
    return {'n': len(vals), 'mean': statistics.fmean(vals),
            'std': statistics.pstdev(vals) if len(vals) > 1 else 0.0,
            'min': min(vals), 'max': max(vals)}


def summarize(rows):
    by = {'baseline': [r for r in rows if r['group'] == 'baseline'],
          'best': [r for r in rows if r['group'] == 'best']}
    metrics = ['fitness', 'speed_mps', 'forward_m', 'tilt_deg', 'yaw_deg',
               'path_over_net', 'max_single_step_m']
    agg = {g: {m: _stats([r.get(m) for r in rs]) for m in metrics}
           for g, rs in by.items()}
    paired = []
    for seed in SEEDS:
        b = next(r for r in by['baseline'] if r['seed'] == seed)
        t = next(r for r in by['best'] if r['seed'] == seed)
        paired.append({
            'seed': seed,
            'delta_fitness': t['fitness'] - b['fitness'],
            'delta_speed_mps': _d(t.get('speed_mps'), b.get('speed_mps')),
            'delta_forward_m': _d(t.get('forward_m'), b.get('forward_m')),
        })
    return {'aggregate': agg, 'paired': paired}


def _d(a, b):
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return a - b
    return None


def decide(rows, summary):
    best = [r for r in rows if r['group'] == 'best']
    base = [r for r in rows if r['group'] == 'baseline']
    cond1 = all(r['status'] == 'ok' and not r['fall'] for r in best)
    cond2 = all(p['delta_fitness'] > 0 for p in summary['paired'])
    cond3 = (_stats([r['speed_mps'] for r in best])['mean']
             > _stats([r['speed_mps'] for r in base])['mean'])
    cond4 = (_stats([r['forward_m'] for r in best])['mean']
             > _stats([r['forward_m'] for r in base])['mean'])
    if not cond1:
        result = 'FAIL'                      # a best episode fell / was invalid
    elif cond2 and cond3 and cond4:
        result = 'PASS'
    elif cond2:
        result = 'INCONCLUSIVE'              # fitness better, locomotion not
    else:
        result = 'FAIL'
    return {'OPT0C_RESULT': result, 'conditions': {
        'cond1_best_3of3_ok_nofall': cond1,
        'cond2_delta_fitness_3of3_gt0': cond2,
        'cond3_mean_best_speed_gt_baseline': cond3,
        'cond4_mean_best_forward_gt_baseline': cond4},
        'LEVEL_3': 'ACHIEVED' if result == 'PASS' else 'NOT_ACHIEVED'}


def main():
    problems = preflight()
    if problems:
        for p in problems:
            sys.stderr.write('PREFLIGHT FAIL: %s\n' % p)
        return 2
    print('[opt0c] preflight PASS')

    sp, thetas = load_thetas()
    ev = build_evaluator()
    os.makedirs(os.path.join(OUT, 'episodes'), exist_ok=True)

    rows = []
    order = []
    for seed in SEEDS:
        order.append(('baseline', seed))
        order.append(('best', seed))

    for group, seed in order:
        row = run_episode(group, seed, thetas[group], ev, sp)
        rows.append(row)
        print('[opt0c] %-8s s%d fit=%+.4f status=%-7s speed=%+.4f fwd=%+.4f fall=%s' % (
            group, seed, row['fitness'], row['status'],
            row['speed_mps'] or 0, row['forward_m'] or 0, row['fall']))

    summary = summarize(rows)
    decision = decide(rows, summary)
    provenance = {
        'git_commit': _git(),
        'frozen_sha256': {k: sha256(os.path.join(WS, k)) for k in FROZEN},
        'anchor_sha256': sha256(os.path.join(WS, ANCHOR)),
        'best_theta_sha256': sha256(BEST_THETA_YAML),
        'best_theta_overrides': thetas['best'],
        'baseline_overrides': thetas['baseline'],
        'seeds': SEEDS,
        'evaluator': {'refs': ev.refs, 'weights': ev.weights,
                      'score_scale': ev.score_scale,
                      'penalty_fall': ev.penalty_fall,
                      'penalty_invalid': ev.penalty_invalid},
        'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
    }
    with open(os.path.join(OUT, 'rows.json'), 'w', encoding='utf-8') as fh:
        json.dump({'order': [{'group': g, 'seed': s} for g, s in order],
                   'rows': rows, 'provenance': provenance}, fh,
                  indent=2, sort_keys=True)
    with open(os.path.join(OUT, 'summary.json'), 'w', encoding='utf-8') as fh:
        json.dump({'aggregate': summary['aggregate'], 'paired': summary['paired'],
                   'decision': decision}, fh, indent=2, sort_keys=True)

    print('[opt0c] === DECISION ===')
    print(json.dumps(decision, indent=2, sort_keys=True))
    return 0 if decision['OPT0C_RESULT'] == 'PASS' else 1


def _git():
    import subprocess
    try:
        return subprocess.run(['git', '-C', WS, 'rev-parse', 'HEAD'],
                              capture_output=True, text=True).stdout.strip() or None
    except Exception:  # noqa: BLE001
        return None


if __name__ == '__main__':
    sys.exit(main())
