#!/usr/bin/env python3
"""Phase OPT-0 minimum-viable automatic gait optimization.

Closed loop (uses pycma; the frozen runner is imported, never modified):

    CMA ask -> candidate x in [0,1]^5 -> override dict
            -> run_p0a_gait_candidate.run_candidate(...)   (FROZEN: launches Gazebo)
            -> FitnessEvaluator.evaluate(result_p0a.json)
            -> CMA tell  (minimize -fitness)

Run with ROS sourced (the child `ros2 launch` inherits the environment):

    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 tools/run_gait_optimization.py --pop 4 --gens 3 --out-root optimization_results

Outputs (under --out-root):
    baseline.json          anchor θ fitness (the frozen baseline)
    generation_log.json    per-generation records (θ, fitness, terms, result path)
    best_theta.yaml        best θ found (named + normalized + overrides)
    best_fitness.json      best fitness + provenance
    episodes/<id>/         raw run dirs produced by the frozen runner
"""

import argparse
import json
import os
import shutil
import subprocess
import sys
import time

import cma
import yaml

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOLS = os.path.join(WS, 'tools')
for _p in (WS, TOOLS):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import run_p0a_gait_candidate as R  # noqa: E402  (FROZEN — import only)
from optimization.parameter_space import ParameterSpace  # noqa: E402
from optimization.fitness import FitnessEvaluator  # noqa: E402

BASELINE_YAML = os.path.join(WS, 'config', 'gait_opt_baseline.yaml')


def git_commit():
    try:
        out = subprocess.run(['git', '-C', WS, 'rev-parse', 'HEAD'],
                             capture_output=True, text=True)
        return out.stdout.strip() or None
    except Exception:
        return None


def load_baseline_cfg():
    if os.path.isfile(BASELINE_YAML):
        with open(BASELINE_YAML, 'r', encoding='utf-8') as fh:
            return yaml.safe_load(fh) or {}
    return {}


def build_evaluator(baseline_cfg):
    fit = baseline_cfg.get('fitness', {}) or {}
    refs = {k: v for k, v in fit.items()
            if k.startswith(('V_ref', 'tilt_ref', 'yaw_ref', 'pon_ref', 'step_ref'))}
    return FitnessEvaluator(refs=refs, weights=fit.get('weights'),
                            penalty_fall=fit.get('penalty_fall', -1.0),
                            penalty_invalid=fit.get('penalty_invalid', -2.0),
                            score_scale=fit.get('score_scale', 0.3))


def run_candidate(cand_id, overrides, args, out_root):
    """Run one candidate through the FROZEN runner. Returns the episode dir."""
    ep = os.path.join(out_root, 'episodes', cand_id)
    R.run_candidate(cand_id, args.seed, ep, overrides, args.timeout,
                    args.domain_id, allow_overwrite=True, gz_seed=args.gz_seed)
    return ep


def evaluate_candidate(cand_id, x, sp, ev, args, out_root, gen):
    """θ -> episode -> FitnessResult. Never raises; records failures."""
    overrides = sp.from_normalized(x)
    record = {
        'generation': gen, 'candidate_id': cand_id,
        'x': [float(v) for v in x],
        'theta_named': sp.to_named(x), 'overrides': overrides,
        'seed': args.seed, 'gz_seed': args.gz_seed, 'git_commit': git_commit(),
        'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
    }
    try:
        ep = run_candidate(cand_id, overrides, args, out_root)
    except Exception as exc:  # noqa: BLE001 - MVP: record, never crash the loop
        record.update({'fitness': ev.penalty_invalid, 'status': 'error',
                       'error': '%s: %s' % (type(exc).__name__, exc),
                       'sim_result_path': None})
        return record
    fr = ev.evaluate_result(ep)
    record.update({'fitness': fr.fitness, 'status': fr.status,
                   'terms': fr.terms, 'detail_reason': fr.detail.get('reason'),
                   'sim_result_path': os.path.relpath(
                       os.path.join(ep, 'result_p0a.json'), WS)})
    return record


def write_outputs(out_root, sp, baseline_rec, gen_log, best_rec, args, cfg):
    with open(os.path.join(out_root, 'generation_log.json'), 'w',
              encoding='utf-8') as fh:
        json.dump({'generations': gen_log}, fh, indent=2, sort_keys=True)
    with open(os.path.join(out_root, 'baseline.json'), 'w', encoding='utf-8') as fh:
        json.dump(baseline_rec, fh, indent=2, sort_keys=True)
    if best_rec is None:
        return
    best_fit = {
        'fitness': best_rec['fitness'], 'status': best_rec['status'],
        'candidate_id': best_rec['candidate_id'],
        'generation': best_rec['generation'],
        'theta_named': best_rec['theta_named'],
        'x_normalized': best_rec['x'], 'overrides': best_rec['overrides'],
        'git_commit': best_rec['git_commit'], 'timestamp': best_rec['timestamp'],
        'seed': best_rec['seed'], 'gz_seed': best_rec['gz_seed'],
        'sim_result_path': best_rec.get('sim_result_path'),
        'baseline_fitness': baseline_rec['fitness'],
    }
    with open(os.path.join(out_root, 'best_fitness.json'), 'w',
              encoding='utf-8') as fh:
        json.dump(best_fit, fh, indent=2, sort_keys=True)
    with open(os.path.join(out_root, 'best_theta.yaml'), 'w',
              encoding='utf-8') as fh:
        yaml.safe_dump({'best': best_fit, 'space': sp.describe()}, fh,
                       sort_keys=False, default_flow_style=False)


def main(argv=None):
    ap = argparse.ArgumentParser(description='Phase OPT-0 gait optimization (pycma)')
    ap.add_argument('--pop', type=int, default=4, help='CMA-ES population (4-8)')
    ap.add_argument('--gens', type=int, default=3, help='generations (3-5)')
    ap.add_argument('--sigma', type=float, default=0.2, help='CMA-ES initial sigma')
    ap.add_argument('--seed', type=int, default=101, help='replicate label / CMA seed')
    ap.add_argument('--gz-seed', type=int, default=101, help='physics seed -> gz sim --seed')
    ap.add_argument('--timeout', type=float, default=200.0, help='per-run launch timeout [s]')
    ap.add_argument('--domain-id', type=int, default=77)
    ap.add_argument('--out-root', default=os.path.join(WS, 'optimization_results'))
    ap.add_argument('--max-candidates', type=int, default=40, help='hard safety cap')
    ap.add_argument('--baseline-only', action='store_true')
    args = ap.parse_args(argv)

    if not shutil.which('ros2'):
        sys.stderr.write('run_gait_optimization: ros2 not on PATH; source ROS first\n')
        return 2

    os.makedirs(args.out_root, exist_ok=True)
    cfg = load_baseline_cfg()
    sp = ParameterSpace()
    ev = build_evaluator(cfg)
    gen_log = []
    all_records = []

    # --- baseline (anchor θ) -------------------------------------------------
    anchor_x = sp.anchor_normalized()
    print('[opt0] baseline anchor_normalized=%s' % (tuple(round(v, 4) for v in anchor_x),))
    baseline_rec = evaluate_candidate('opt0_baseline', anchor_x, sp, ev, args,
                                      args.out_root, gen=-1)
    all_records.append(baseline_rec)
    print('[opt0] baseline fitness=%+.4f status=%s' % (
        baseline_rec['fitness'], baseline_rec['status']))
    if args.baseline_only:
        write_outputs(args.out_root, sp, baseline_rec, gen_log, None, args, cfg)
        return 0

    # --- CMA-ES --------------------------------------------------------------
    x0 = [min(0.9, max(0.1, v)) for v in anchor_x]
    lo, hi = sp.bounds()
    es = cma.CMAEvolutionStrategy(
        x0, args.sigma,
        {'popsize': args.pop, 'bounds': [lo, hi], 'seed': args.seed, 'verbose': -9})

    total = 0
    for gen in range(args.gens):
        if total >= args.max_candidates:
            print('[opt0] max-candidates reached; stopping')
            break
        xs = es.ask()
        records = []
        for idx, x in enumerate(xs):
            cand_id = 'opt0_g%02d_c%02d' % (gen, idx)
            rec = evaluate_candidate(cand_id, x, sp, ev, args, args.out_root, gen)
            records.append(rec)
            all_records.append(rec)
            total += 1
            print('[opt0] g%d c%d fit=%+.4f status=%-7s named=%s' % (
                gen, idx, rec['fitness'], rec['status'],
                {k: round(v, 4) for k, v in rec['theta_named'].items()}))
            if total >= args.max_candidates:
                break
        fits = [r['fitness'] for r in records]
        es.tell([list(r['x']) for r in records], [-f for f in fits])
        gen_log.append({
            'generation': gen, 'records': records,
            'best_fitness': max(fits), 'mean_fitness': sum(fits) / len(fits),
        })
        print('[opt0] generation %d: best=%+.4f mean=%+.4f' % (
            gen, max(fits), sum(fits) / len(fits)))
        write_outputs(args.out_root, sp, baseline_rec, gen_log, None, args, cfg)

        if es.stop():
            print('[opt0] cma stop() condition met')
            break

    best_rec = max(all_records, key=lambda r: r['fitness'])
    write_outputs(args.out_root, sp, baseline_rec, gen_log, best_rec, args, cfg)
    print('[opt0] BEST fit=%+.4f status=%s (baseline %+.4f) named=%s' % (
        best_rec['fitness'], best_rec['status'], baseline_rec['fitness'],
        {k: round(v, 4) for k, v in best_rec['theta_named'].items()}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
