#!/usr/bin/env python3
"""Task 4 / Phase 4E Path B — Gate B2 persisted 3x3 causal comparison.

Runs exactly the nine frozen experiments in the frozen order, persists every
raw trace and evidence file, computes the frozen per-run metrics/gates, and
writes the group analysis. No parameter search; no gait/physics changes; no
Pilot/100k; no video. Only the process group started by this script is stopped
(never pkill/killall).

Usage: python3 tools/run_task4_phase4e_pathb_b2.py [--only LABEL]
"""

import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
import time

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import phase4e_metrics as M  # noqa: E402
import phase4e_analysis as A  # noqa: E402

WS = os.path.dirname(_TOOLS)
EV = os.path.join(WS, 'docs', 'task4_phase4e_pathb_evidence')
CHARTER = os.path.join(EV, 'charter')
LAUNCH = ['ros2', 'launch', 'nao_gz_sim', 'nao_sim.launch.py',
          'gui:=false', 'use_rsp:=false',
          'enable_phase4a:=true', 'phase4a_run_probe:=false', 'phase4a_run_trace:=false',
          'enable_phase4b:=true', 'phase4b_run_gait:=true', 'phase4b_run_trace:=true',
          'phase4b_record_duration:=0.0', 'enable_phase4d:=false']


def log(msg):
    print('[%s] %s' % (time.strftime('%H:%M:%S'), msg), flush=True)


def load_json(path, default=None):
    try:
        with open(path) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return default


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def param_args(cfg):
    args = []
    for k in sorted(cfg):
        v = cfg[k]
        if isinstance(v, bool):
            v = 'true' if v else 'false'
        if k.startswith('actuator_'):
            name = k
        elif k.startswith('phase4b_'):
            name = k
        else:
            name = 'phase4b_' + k
        args.append('%s:=%s' % (name, v))
    return args


def kill_group(pgid):
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            os.killpg(pgid, sig)
        except ProcessLookupError:
            return
        for _ in range(50):
            time.sleep(0.2)
            try:
                os.killpg(pgid, 0)
            except ProcessLookupError:
                return
    try:
        os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass


def residual_count():
    proc = subprocess.run(['bash', '-c',
                           "ps -eo cmd | grep -E 'gz sim|nao_sim.launch|"
                           "install/nao_gz_sim/lib/nao_gz_sim|"
                           "ros_gz_bridge/parameter_bridge|robot_state_publisher' "
                           "| grep -v grep | wc -l"],
                          capture_output=True, text=True)
    try:
        return int(proc.stdout.strip())
    except (ValueError, AttributeError):
        return None


def run_one(label, group, cfg, run_dir, domain):
    if os.path.isdir(run_dir) and os.listdir(run_dir):
        raise SystemExit('refusing to overwrite existing run dir: %s' % run_dir)
    trace = os.path.join(run_dir, 'trace')
    os.makedirs(trace, exist_ok=True)
    with open(os.path.join(run_dir, 'run_config.json'), 'w') as fh:
        json.dump(cfg, fh, indent=2, sort_keys=True)
    cmd = LAUNCH + param_args(cfg) + ['phase4b_evidence_dir:=%s' % trace]
    with open(os.path.join(run_dir, 'launch_cmd.txt'), 'w') as fh:
        fh.write(' '.join(cmd) + '\n')
    env = dict(os.environ)
    env['GZ_PARTITION'] = 't4pb_%s' % label.lower().replace('-', '_')
    env['ROS_DOMAIN_ID'] = str(domain)
    start = time.time()
    logf = open(os.path.join(run_dir, 'launch.log'), 'w')
    proc = subprocess.Popen(cmd, stdout=logf, stderr=subprocess.STDOUT, cwd=WS,
                            env=env, start_new_session=True)
    pgid = proc.pid
    ready = None
    finish_reason = None
    deadline = time.time() + 180
    while time.time() < deadline:
        try:
            text = open(os.path.join(run_dir, 'launch.log')).read()
        except OSError:
            text = ''
        import re
        m = re.search(r'model ready at sim ([\d.]+)', text)
        if m and ready is None:
            ready = float(m.group(1))
        m2 = re.search(r'phase4b_producer stopping: reason=(\S+)', text)
        if m2:
            finish_reason = m2.group(1)
            break
        if proc.poll() is not None:
            break
        time.sleep(0.5)
    time.sleep(1.0)
    kill_group(pgid)
    logf.close()
    for _ in range(30):
        if residual_count() == 0:
            break
        time.sleep(1.0)
    res = {
        'run_id': label, 'group': group,
        'start_wall': start, 'end_wall': time.time(),
        'ready_sim': ready, 'finish_reason': finish_reason,
        'residual_processes': residual_count(),
        'launch_cmd': cmd,
    }
    with open(os.path.join(run_dir, 'run_result.json'), 'w') as fh:
        json.dump(res, fh, indent=2, sort_keys=True)
    return res


def analyze_one(run_dir, cfg, group, frozen, thresholds):
    res = load_json(os.path.join(run_dir, 'run_result.json'), {})
    ready = res.get('ready_sim')
    if ready is None:
        raise SystemExit('missing ready_sim in %s' % run_dir)
    m = M.run_metrics(os.path.join(run_dir, 'trace'), cfg, ready, frozen,
                      mode='walk')
    m['run_id'] = res.get('run_id')
    m['group'] = group
    gates = M.evaluate_gates(m, thresholds, group)
    with open(os.path.join(run_dir, 'run_metrics.json'), 'w') as fh:
        json.dump(m, fh, indent=2, sort_keys=True)
    with open(os.path.join(run_dir, 'run_gates.json'), 'w') as fh:
        json.dump(gates, fh, indent=2, sort_keys=True)
    return m, gates


def write_manifest(run_dir):
    entries = []
    for root, _dirs, files in os.walk(run_dir):
        for f in sorted(files):
            if f == 'SHA256SUMS':
                continue
            p = os.path.join(root, f)
            entries.append('%s  %s' % (sha256_file(p),
                                       os.path.relpath(p, run_dir)))
    with open(os.path.join(run_dir, 'SHA256SUMS'), 'w') as fh:
        fh.write('\n'.join(entries) + '\n')
    return entries


def mean(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else None


def std(xs):
    xs = [x for x in xs if x is not None]
    if len(xs) < 2:
        return None
    mu = sum(xs) / len(xs)
    return (sum((x - mu) ** 2 for x in xs) / (len(xs) - 1)) ** 0.5


def group_stats(results, key, group):
    vals = [r['metrics'].get(key) for r in results if r['group'] == group]
    return {'n': len(vals), 'values': vals, 'mean': mean(vals), 'std': std(vals),
            'min': min([v for v in vals if v is not None], default=None),
            'max': max([v for v in vals if v is not None], default=None)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--only', default=None, help='run a single frozen label')
    args = ap.parse_args()
    frozen = load_json(os.path.join(CHARTER, 'frozen_configs.json'))
    order = load_json(os.path.join(CHARTER, 'run_order.json'))
    thresholds = load_json(os.path.join(CHARTER, 'thresholds.json'))
    labels = order['order']
    if args.only:
        labels = [args.only]
    runs_root = os.path.join(EV, 'runs')
    os.makedirs(runs_root, exist_ok=True)

    # frozen forward baseline: prior persisted Phase 4E stand run dx (read-only)
    base_dx = None
    prior = os.path.join(WS, 'docs', 'task4_phase4e_visual_evidence',
                         'stand_baseline', 'phase4c_summary.json')
    ps = load_json(prior)
    if ps:
        base_dx = ps.get('body', {}).get('displacement', {}).get('steady_gait', {}).get('dx')
    log('frozen forward baseline (prior stand dx) = %r' % base_dx)

    results = []
    domain_base = 200
    for i, label in enumerate(labels):
        group = order['group_of'][label]
        cfg = frozen[group]
        run_dir = os.path.join(EV, order['run_dir'][label])
        log('=== %s (%s) -> %s ===' % (label, group, run_dir))
        if (os.path.exists(os.path.join(run_dir, 'run_result.json'))
                and os.path.exists(os.path.join(run_dir, 'trace',
                                                'state_trace.jsonl'))):
            log('reusing previously persisted run (infrastructure resume)')
            res = load_json(os.path.join(run_dir, 'run_result.json'))
        else:
            res = run_one(label, group, cfg, run_dir, domain_base + i + 1)
        m, gates = analyze_one(run_dir, cfg, group, cfg, thresholds)
        # forward gate (needs global baseline)
        if m['mode'] == 'walk':
            gates['forward_displacement_gt_baseline_plus_0.03'] = {
                'pass': (m.get('body_dx') is not None and base_dx is not None
                         and m['body_dx'] > base_dx + 0.03),
                'value': m.get('body_dx'), 'baseline': base_dx}
        entries = write_manifest(run_dir)
        results.append({'run_id': label, 'group': group,
                        'metrics': m, 'gates': gates,
                        'manifest_files': len(entries),
                        'residual': res.get('residual_processes')})
        log('%s dx=%s yaw=%s residual=%s gates_fail=%d'
            % (label, m.get('body_dx'), m.get('max_yaw_abs_from_start_deg'),
               res.get('residual_processes'),
               sum(1 for g in gates.values() if not g['pass'])))

    if args.only:
        with open(os.path.join(EV, 'runs', '%s_summary.json' % args.only), 'w') as fh:
            json.dump(results[0], fh, indent=2, sort_keys=True)
        return 0

    groups = ['CONTROL', 'NATURAL-STATIC', 'NATURAL-SWING']
    metrics_keys = ['body_dx', 'body_dy', 'max_yaw_abs_from_start_deg',
                    'max_roll_deg', 'max_pitch_deg', 'body_min_z',
                    'support_drift_m', 'left_swing_m', 'right_swing_m',
                    'imu_nonfinite']
    group_analysis = {'baseline_dx': base_dx, 'groups': {}}
    for g in groups:
        group_analysis['groups'][g] = {
            k: group_stats(results, k, g) for k in metrics_keys}
        rs = [r for r in results if r['group'] == g]
        group_analysis['groups'][g]['all_runs_pass_all_gates'] = all(
            all(v['pass'] for v in r['gates'].values()) for r in rs)

    with open(os.path.join(EV, 'pathb_results.json'), 'w') as fh:
        json.dump(results, fh, indent=2, sort_keys=True)
    with open(os.path.join(EV, 'pathb_group_analysis.json'), 'w') as fh:
        json.dump(group_analysis, fh, indent=2, sort_keys=True)
    # chart-wide manifest
    with open(os.path.join(EV, 'SHA256SUMS'), 'w') as fh:
        for root, _dirs, files in os.walk(EV):
            for f in sorted(files):
                if f == 'SHA256SUMS':
                    continue
                p = os.path.join(root, f)
                fh.write('%s  %s\n' % (sha256_file(p), os.path.relpath(p, EV)))
    log('B2 complete: 9 runs analysed')
    return 0


if __name__ == '__main__':
    sys.exit(main())
