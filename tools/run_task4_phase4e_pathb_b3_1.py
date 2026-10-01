#!/usr/bin/env python3
"""Gate B3.1 — frozen 12-candidate single headless screening runner.

This runner reads the frozen Gate B3 charter and, in ``screening`` mode, runs
each of the 12 frozen candidates exactly once headless. It only ever drives its
own candidate runs; it never re-runs B2, never touches B2/B3.0 evidence, never
adds a 13th candidate, and never performs B3.2 repeats (those are a separate,
explicit mode).

Pipeline stages (candidate IDs, run order, and gate semantics come from the
frozen B3.0 charter, never from this file):
  1. verify the frozen B3.0 source/test chain is unchanged;
  2. load and validate candidates.json / run_order.json / hard_gates.json /
     selection_rules.json;
  3. for each candidate in the frozen order, run one headless launch with an
     isolated ROS_DOMAIN_ID / GZ_PARTITION, persist config/command/log/result/
     trace, evaluate the frozen B3 gates (B2 gates + body_net_forward_m >= 0.08),
     and publish an atomic per-run manifest;
  4. report per-run gates and (in a separate mode) B3.2 eligibility.

Modes:
  validate   (default) read-only: verify frozen chain + candidate table, exit.
  plan       read-only: print the 12 launch commands without writing/running.
  screening  run the 12 candidates once each (requires --yes-run).
  resume     read-only: verify already-persisted run evidence; never counts an
             old run as a new run and never launches.
  repeat     separate explicit B3.2 mode: requires --candidate IDs and --yes-run;
             never triggered by screening.

This phase (authoring) does not call screening/repeat/resume launch paths.
"""

import argparse
import hashlib
import json
import math
import os
import re
import signal
import subprocess
import sys
import tempfile
import time

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import phase4e_metrics as M  # noqa: E402
import phase4e_b3_gates as G  # noqa: E402

WS = os.path.dirname(_TOOLS)
B3_EV = os.path.join(WS, 'docs', 'task4_phase4e_pathb_b3_evidence')
B3_0 = os.path.join(B3_EV, 'b3_0')
B3_1 = os.path.join(B3_EV, 'b3_1')
DEFAULT_RUNS_ROOT = os.path.join(B3_1, 'runs')
B2_EV = os.path.join(WS, 'docs', 'task4_phase4e_pathb_evidence')
VIS_EV = os.path.join(WS, 'docs', 'task4_phase4e_visual_evidence')

# Output roots that must never be written by this runner.
FORBIDDEN_OUTPUT_PREFIXES = (B2_EV, B3_0, VIS_EV)

SOURCE_FREEZE = os.path.join(B3_0, 'source_test_freeze.sha256')

SEARCH_PARAMETERS = ['step_length_m', 'step_frequency_hz', 'leg_extension_m',
                     'sway_amplitude_rad', 'foot_pitch_swing_rad']
IMMUTABLE_ARM_FIELDS = ['arm_neutral_pitch_rad', 'arm_neutral_roll_rad',
                        'arm_swing_rad', 'arm_phase_offset_rad', 'arm_swing_sign',
                        'elbow_bend_rad']
GROUP = 'NATURAL-SWING'

# Frozen launch base (headless; enables the Phase 4B producer + trace only).
LAUNCH_BASE = ['ros2', 'launch', 'nao_gz_sim', 'nao_sim.launch.py',
               'gui:=false', 'use_rsp:=false',
               'enable_phase4a:=true', 'phase4a_run_probe:=false',
               'phase4a_run_trace:=false', 'enable_phase4b:=true',
               'phase4b_run_gait:=true', 'phase4b_run_trace:=true',
               'phase4b_record_duration:=0.0', 'enable_phase4d:=false']

REQUIRED_TRACE_FILES = ['clock_trace.jsonl', 'command_trace.jsonl',
                        'state_trace.jsonl', 'contact_trace.jsonl',
                        'body_pose_trace.jsonl', 'imu_trace.jsonl']
PER_RUN_FILES = ['run_config.json', 'launch_cmd.txt', 'launch.log',
                 'run_result.json', 'run_metrics.json', 'run_gates.json',
                 'SHA256SUMS']


class RunnerError(Exception):
    """Structured infrastructure failure (launch/timeout/unknown)."""

    def __init__(self, reason, stage, detail=None):
        super().__init__(reason)
        self.reason = reason
        self.stage = stage
        self.detail = detail


# ------------------------------------------------------------------ utilities
def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def load_json(path):
    with open(path) as fh:
        return json.load(fh)


def write_json_atomic(path, obj):
    _atomic_write(path, json.dumps(obj, indent=2, sort_keys=True) + '\n')


def write_jsonl_atomic(path, rows):
    _atomic_write(path, ''.join(json.dumps(r) + '\n' for r in rows))


def _atomic_write(path, text):
    directory = os.path.dirname(path)
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=directory, prefix='.tmp_b31_', suffix='.part')
    try:
        with os.fdopen(fd, 'w') as fh:
            fh.write(text)
        os.replace(tmp, path)
    except BaseException:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def structured_error(reason, stage, candidate_id, run_dir, detail=None):
    err = {'ok': False, 'reason': reason, 'stage': stage,
           'candidate_id': candidate_id, 'run_dir': run_dir}
    if detail is not None:
        err['detail'] = detail
    return err


def is_structured_error(value):
    return (isinstance(value, dict) and value.get('ok') is False
            and isinstance(value.get('reason'), str)
            and isinstance(value.get('stage'), str))


def _finite(v):
    return (isinstance(v, (int, float)) and not isinstance(v, bool)
            and math.isfinite(float(v)))


# --------------------------------------------------------------- frozen chain
def parse_manifest(path):
    """Parse a sha256sum manifest into [(relpath, hash)] (ignoring comments)."""
    entries = []
    with open(path) as fh:
        for line in fh:
            line = line.rstrip('\n')
            if not line or line.startswith('#'):
                continue
            parts = line.split(None, 1)
            if len(parts) != 2:
                continue
            entries.append((parts[1].strip(), parts[0].strip()))
    return entries


def verify_source_freeze(manifest=SOURCE_FREEZE, workspace=WS):
    """Verify every entry of the frozen B3.0 source/test manifest."""
    missing, mismatched = [], []
    for rel, want in parse_manifest(manifest):
        p = os.path.join(workspace, rel)
        if not os.path.isfile(p):
            missing.append(rel)
            continue
        if sha256_file(p) != want:
            mismatched.append(rel)
    if missing or mismatched:
        return structured_error(
            'source_freeze_mismatch', 'verify_source_freeze', None, B3_0,
            {'missing': missing, 'mismatched': mismatched})
    return {'ok': True, 'entries': len(parse_manifest(manifest))}


def load_frozen(charter_dir=B3_0):
    return {
        'candidates': load_json(os.path.join(charter_dir, 'candidates.json')),
        'run_order': load_json(os.path.join(charter_dir, 'run_order.json')),
        'hard_gates': load_json(os.path.join(charter_dir, 'hard_gates.json')),
        'selection_rules': load_json(os.path.join(charter_dir, 'selection_rules.json')),
    }


def validate_candidate_table(frozen):
    """Programmatic validation of the frozen candidate table and run order."""
    cand = frozen['candidates']
    order = frozen['run_order']
    cs = cand.get('candidates', [])
    problems = []
    if len(cs) != 12:
        problems.append('candidate_count=%d' % len(cs))
    kinds = [c.get('kind') for c in cs]
    if kinds.count('anchor') != 1:
        problems.append('anchor_count=%d' % kinds.count('anchor'))
    if kinds.count('single-factor') != 10:
        problems.append('single_factor_count=%d' % kinds.count('single-factor'))
    if kinds.count('conservative-combination') != 1:
        problems.append('conservative_count=%d'
                        % kinds.count('conservative-combination'))
    ids = [c.get('id') for c in cs]
    if len(set(ids)) != len(ids):
        problems.append('duplicate_ids')
    if len(order.get('order', [])) != 12 or len(set(order.get('order', []))) != 12:
        problems.append('run_order_not_12_unique')
    if sorted(order.get('order', [])) != sorted(ids):
        problems.append('run_order_ids_mismatch')

    anchors = [c for c in cs if c.get('kind') == 'anchor']
    anchor_cfg = anchors[0].get('config', {}) if anchors else {}
    if anchor_cfg.get('foot_pitch_swing_rad') != 0.10:
        problems.append('anchor_foot_pitch=%r'
                        % anchor_cfg.get('foot_pitch_swing_rad'))

    per_field = {p: 0 for p in SEARCH_PARAMETERS}
    for c in cs:
        cfg = c.get('config', {})
        if 'foot_pitch_swing_rad' not in cfg:
            problems.append('%s_missing_foot_pitch' % c.get('id'))
        if set(cfg) != set(anchor_cfg):
            problems.append('%s_key_set_mismatch' % c.get('id'))
        for f in IMMUTABLE_ARM_FIELDS:
            if cfg.get(f) != anchor_cfg.get(f):
                problems.append('%s_arm_field_changed_%s' % (c.get('id'), f))
        changed = {k for k in set(cfg) | set(anchor_cfg)
                   if cfg.get(k) != anchor_cfg.get(k)}
        if c.get('kind') == 'anchor':
            if changed:
                problems.append('anchor_changed')
        elif c.get('kind') == 'single-factor':
            if len(changed) != 1 or not changed <= set(SEARCH_PARAMETERS):
                problems.append('%s_bad_single_factor_%s'
                                % (c.get('id'), sorted(changed)))
            else:
                per_field[list(changed)[0]] += 1
            overrides = set(c.get('overrides', {}))
            if overrides != changed:
                problems.append('%s_overrides_config_mismatch' % c.get('id'))
        else:
            if not changed <= set(SEARCH_PARAMETERS):
                problems.append('%s_unauthorized_change_%s'
                                % (c.get('id'), sorted(changed - set(SEARCH_PARAMETERS))))
    if per_field != {p: 2 for p in SEARCH_PARAMETERS}:
        problems.append('per_field=%r' % per_field)

    if problems:
        return structured_error('candidate_table_invalid', 'validate_candidates',
                                None, B3_0, problems)
    return {'ok': True, 'count': len(cs), 'anchor_cfg': anchor_cfg}


def validate_candidate_config(cfg, anchor_cfg):
    """Reject missing/extra/unknown/non-finite candidate config fields."""
    problems = []
    if set(cfg) != set(anchor_cfg):
        missing = sorted(set(anchor_cfg) - set(cfg))
        extra = sorted(set(cfg) - set(anchor_cfg))
        problems.append({'missing': missing, 'extra': extra})
    for k, v in cfg.items():
        if isinstance(v, bool):
            continue
        if isinstance(v, (int, float)):
            if not _finite(v):
                problems.append('non_finite:%s' % k)
        elif not isinstance(v, str):
            problems.append('bad_type:%s' % k)
    changed = {k for k in set(cfg) | set(anchor_cfg)
               if cfg.get(k) != anchor_cfg.get(k)}
    if not changed <= set(SEARCH_PARAMETERS):
        problems.append('unauthorized_change:%s'
                        % sorted(changed - set(SEARCH_PARAMETERS)))
    if 'foot_pitch_swing_rad' not in cfg:
        problems.append('missing_foot_pitch_swing_rad')
    return problems


# ----------------------------------------------------------- launch / config
def param_args(cfg):
    args = []
    for k in sorted(cfg):
        v = cfg[k]
        if isinstance(v, bool):
            v = 'true' if v else 'false'
        if k.startswith('actuator_') or k.startswith('phase4b_'):
            name = k
        else:
            name = 'phase4b_' + k
        args.append('%s:=%s' % (name, v))
    return args


def build_launch_cmd(cfg, trace_dir):
    return LAUNCH_BASE + param_args(cfg) + \
        ['phase4b_evidence_dir:=%s' % trace_dir]


def check_config_matches_launch(cfg, cmd):
    """Every candidate field must be provably present on the launch command."""
    args = set(cmd)
    problems = []
    for k, v in cfg.items():
        if isinstance(v, bool):
            v = 'true' if v else 'false'
        name = k if (k.startswith('actuator_') or k.startswith('phase4b_')) \
            else 'phase4b_' + k
        token = '%s:=%s' % (name, v)
        if token not in args:
            problems.append(token)
    return problems


def check_config_roundtrip(cfg, run_dir):
    """run_config.json must equal the candidate config field by field."""
    persisted = load_json(os.path.join(run_dir, 'run_config.json'))
    if persisted != cfg:
        return structured_error(
            'run_config_mismatch', 'check_config', None, run_dir,
            {'diff': sorted(set(persisted) ^ set(cfg))})
    return {'ok': True}


# ------------------------------------------------------------- output safety
def resolve_runs_root(requested, workspace=WS,
                      forbidden_prefixes=FORBIDDEN_OUTPUT_PREFIXES):
    ws = os.path.realpath(workspace)
    out = os.path.realpath(requested)
    if out != ws and not out.startswith(ws + os.sep):
        return None, structured_error('output_outside_workspace', 'output_root',
                                      None, requested)
    for pref in forbidden_prefixes:
        p = os.path.realpath(pref)
        if out == p or out.startswith(p + os.sep):
            return None, structured_error('output_forbidden_historical_dir',
                                          'output_root', None, requested, p)
    if os.path.exists(out) and os.path.isfile(out):
        return None, structured_error('output_root_is_file', 'output_root',
                                      None, requested)
    return out, None


def resolve_run_dir(runs_root, candidate_id, overwrite=False):
    run_dir = os.path.join(runs_root, candidate_id)
    if os.path.isdir(run_dir) and os.listdir(run_dir) and not overwrite:
        return None, structured_error('run_dir_not_empty', 'run_dir',
                                      candidate_id, run_dir)
    return run_dir, None


# --------------------------------------------------------------- trace checks
def check_trace(run_dir, candidate_id):
    trace = os.path.join(run_dir, 'trace')
    if not os.path.isdir(trace):
        return structured_error('missing_trace_dir', 'check_trace',
                                candidate_id, run_dir)
    empty = []
    corrupt = []
    for name in REQUIRED_TRACE_FILES:
        p = os.path.join(trace, name)
        if not os.path.isfile(p) or os.path.getsize(p) == 0:
            empty.append(name)
            continue
        try:
            with open(p) as fh:
                for line in fh:
                    if line.strip():
                        json.loads(line)
                        break
                else:
                    empty.append(name)
        except ValueError:
            corrupt.append(name)
    if empty or corrupt:
        return structured_error('raw_trace_invalid', 'check_trace',
                                candidate_id, run_dir,
                                {'empty': empty, 'corrupt': corrupt})
    return {'ok': True}


# ------------------------------------------------------------- gates / eval
def load_base_dx(prior=os.path.join(VIS_EV, 'stand_baseline', 'phase4c_summary.json')):
    try:
        ps = load_json(prior)
    except (OSError, ValueError):
        return None
    return ps.get('body', {}).get('displacement', {}).get('steady_gait', {}).get('dx')


def evaluate_run(run_dir, cfg, thresholds, base_dx, candidate_id):
    """Evaluate frozen B3 gates for one persisted run (no exception hiding)."""
    res = load_json(os.path.join(run_dir, 'run_result.json'))
    ready = res.get('ready_sim')
    if ready is None:
        return structured_error('missing_ready_marker', 'evaluate', candidate_id,
                                run_dir)
    trace = os.path.join(run_dir, 'trace')
    m = M.run_metrics(trace, cfg, ready, cfg, mode='walk')
    gates, bf = G.evaluate_b3_gates(m, trace, thresholds, GROUP)
    gates['forward_displacement_gt_baseline_plus_0.03'] = {
        'pass': (m.get('body_dx') is not None and base_dx is not None
                 and m['body_dx'] > base_dx + 0.03),
        'value': m.get('body_dx'), 'baseline': base_dx}
    return {'ok': True, 'metrics': m, 'gates': gates, 'body_net_forward': bf}


def run_gates_pass(gates):
    return bool(gates) and all(g.get('pass') is True for g in gates.values())


def write_run_manifest(run_dir):
    entries = []
    for root, _dirs, files in os.walk(run_dir):
        for f in sorted(files):
            if f == 'SHA256SUMS':
                continue
            p = os.path.join(root, f)
            entries.append('%s  %s' % (sha256_file(p),
                                       os.path.relpath(p, run_dir)))
    _atomic_write(os.path.join(run_dir, 'SHA256SUMS'),
                  '\n'.join(entries) + '\n')
    return entries


# -------------------------------------------------------------- process group
def residual_count():
    proc = subprocess.run(
        ['bash', '-c',
         "ps -eo cmd | grep -E 'gz sim|nao_sim.launch|"
         "install/nao_gz_sim/lib|ros_gz_bridge/parameter_bridge|"
         "robot_state_publisher' | grep -v grep | wc -l"],
        capture_output=True, text=True)
    try:
        return int(proc.stdout.strip())
    except (ValueError, AttributeError):
        return None


def kill_group(pgid):
    """Stop only the process group this runner created; never by process name."""
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


def default_launcher(candidate_id, cmd, run_dir, trace_dir, timeout):
    """Run one headless launch in its own process group; wait for completion."""
    env = dict(os.environ)
    env['GZ_PARTITION'] = 't4pb31_%s' % candidate_id.lower().replace('.', '_').replace('-', '_')
    stable = int(hashlib.sha256(candidate_id.encode()).hexdigest(), 16)
    env['ROS_DOMAIN_ID'] = str(210 + (stable % 40))
    call = dict(cmd)
    for i, a in enumerate(call):
        if a.startswith('phase4b_evidence_dir:='):
            call[i] = 'phase4b_evidence_dir:=%s' % trace_dir
    launch_log = os.path.join(run_dir, 'launch.log')
    start = time.time()
    logf = open(launch_log, 'w')
    proc = subprocess.Popen(call, stdout=logf, stderr=subprocess.STDOUT, cwd=WS,
                            env=env, start_new_session=True)
    pgid = proc.pid
    ready = None
    finish = None
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            text = open(launch_log).read()
        except OSError:
            text = ''
        m = re.search(r'model ready at sim ([\d.]+)', text)
        if m and ready is None:
            ready = float(m.group(1))
        m2 = re.search(r'phase4b_producer stopping: reason=(\S+)', text)
        if m2:
            finish = m2.group(1)
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
    result = {'ready_sim': ready, 'finish_reason': finish,
              'residual_processes': residual_count(), 'exit_code': proc.returncode,
              'start_wall': start, 'end_wall': time.time(),
              'process_group_exited': residual_count() == 0}
    if time.time() >= deadline and ready is None:
        result['fail_reason'] = 'launch_timeout'
        result['fail_stage'] = 'launch'
    return result


# ------------------------------------------------------------- one candidate
def execute_candidate(candidate_id, cfg, anchor_cfg, runs_root, thresholds,
                      base_dx, launcher=None, timeout=180.0, overwrite=False):
    """Run (or inject-run) one candidate and persist/ evaluate its evidence."""
    launcher = launcher or default_launcher
    problems = validate_candidate_config(cfg, anchor_cfg)
    if problems:
        return structured_error('invalid_candidate_config', 'validate', candidate_id,
                                os.path.join(runs_root, candidate_id), problems)
    run_dir, err = resolve_run_dir(runs_root, candidate_id, overwrite)
    if err is not None:
        err['candidate_id'] = candidate_id
        return err
    trace = os.path.join(run_dir, 'trace')
    os.makedirs(trace, exist_ok=True)
    cmd = build_launch_cmd(cfg, trace)
    mismatch = check_config_matches_launch(cfg, cmd)
    if mismatch:
        return structured_error('config_launch_mismatch', 'build_launch',
                                candidate_id, run_dir, mismatch)
    write_json_atomic(os.path.join(run_dir, 'run_config.json'), cfg)
    _atomic_write(os.path.join(run_dir, 'launch_cmd.txt'), ' '.join(cmd) + '\n')
    rt = check_config_roundtrip(cfg, run_dir)
    if is_structured_error(rt):
        rt['candidate_id'] = candidate_id
        return rt
    try:
        launch_res = launcher(candidate_id, cmd, run_dir, trace, timeout)
    except RunnerError as exc:
        launch_res = {'fail_reason': exc.reason, 'fail_stage': exc.stage,
                      'detail': exc.detail, 'residual_processes': residual_count()}
    if launch_res.get('fail_reason'):
        result = structured_error(launch_res['fail_reason'],
                                  launch_res.get('fail_stage', 'launch'),
                                  candidate_id, run_dir, launch_res.get('detail'))
        result.update({'start_wall': launch_res.get('start_wall'),
                       'end_wall': launch_res.get('end_wall'),
                       'residual_processes': launch_res.get('residual_processes')})
        write_json_atomic(os.path.join(run_dir, 'run_result.json'), result)
        return result
    result = {'run_id': candidate_id, 'group': GROUP,
              'start_wall': launch_res.get('start_wall'),
              'end_wall': launch_res.get('end_wall'),
              'ready_sim': launch_res.get('ready_sim'),
              'finish_reason': launch_res.get('finish_reason'),
              'residual_processes': launch_res.get('residual_processes'),
              'process_group_exited': launch_res.get('process_group_exited'),
              'launch_cmd': cmd}
    write_json_atomic(os.path.join(run_dir, 'run_result.json'), result)
    if result['ready_sim'] is None:
        result.update(structured_error('missing_ready_marker', 'launch',
                                       candidate_id, run_dir))
        write_json_atomic(os.path.join(run_dir, 'run_result.json'), result)
        return result
    if result.get('finish_reason') is None:
        result.update(structured_error('missing_finish_reason', 'launch',
                                       candidate_id, run_dir))
        write_json_atomic(os.path.join(run_dir, 'run_result.json'), result)
        return result
    if result['finish_reason'] == 'safety_stop':
        result.update(structured_error('producer_safety_stop', 'launch',
                                       candidate_id, run_dir))
        write_json_atomic(os.path.join(run_dir, 'run_result.json'), result)
        return result
    if result['finish_reason'] != 'gait_duration_complete':
        result.update(structured_error('unexpected_finish_reason', 'launch',
                                       candidate_id, run_dir,
                                       result['finish_reason']))
        write_json_atomic(os.path.join(run_dir, 'run_result.json'), result)
        return result
    if result.get('residual_processes') not in (0, None):
        result.update(structured_error('residual_processes', 'post_run',
                                       candidate_id, run_dir,
                                       result['residual_processes']))
        write_json_atomic(os.path.join(run_dir, 'run_result.json'), result)
        return result
    tr = check_trace(run_dir, candidate_id)
    if is_structured_error(tr):
        result.update(tr)
        write_json_atomic(os.path.join(run_dir, 'run_result.json'), result)
        return result
    ev = evaluate_run(run_dir, cfg, thresholds, base_dx, candidate_id)
    if is_structured_error(ev):
        result.update(ev)
        write_json_atomic(os.path.join(run_dir, 'run_result.json'), result)
        return result
    write_json_atomic(os.path.join(run_dir, 'run_metrics.json'), ev['metrics'])
    write_json_atomic(os.path.join(run_dir, 'run_gates.json'), ev['gates'])
    result['ok'] = True
    result['all_gates_pass'] = run_gates_pass(ev['gates'])
    result['body_net_forward'] = ev['body_net_forward']
    write_json_atomic(os.path.join(run_dir, 'run_result.json'), result)
    write_run_manifest(run_dir)
    return result


# ---------------------------------------------------------------- screening
def build_thresholds(hard_gates_doc):
    return {'hard_gates': hard_gates_doc.get('inherited_hard_gates', {})}


def run_screening(frozen, runs_root, launcher=None, timeout=180.0,
                  overwrite=False):
    """Run the frozen 12 candidates once each, strictly in frozen order."""
    order = frozen['run_order']['order']
    by_id = {c['id']: c for c in frozen['candidates']['candidates']}
    valid = validate_candidate_table(frozen)
    if is_structured_error(valid):
        return {'ok': False, 'error': valid, 'results': {}}
    anchor_cfg = valid['anchor_cfg']
    thresholds = build_thresholds(frozen['hard_gates'])
    base_dx = load_base_dx()
    results = {}
    for candidate_id in order:
        cfg = by_id[candidate_id]['config']
        results[candidate_id] = execute_candidate(
            candidate_id, cfg, anchor_cfg, runs_root, thresholds, base_dx,
            launcher=launcher, timeout=timeout, overwrite=overwrite)
    passing = [cid for cid, r in results.items()
               if r.get('ok') and r.get('all_gates_pass')]
    return {'ok': True, 'order': order, 'results': results,
            'passing': passing}


def compute_eligibility(frozen, results):
    """Apply the frozen B3.2 selection rules (no repeats are run)."""
    rules = frozen['selection_rules']
    passing = [cid for cid, r in results.items()
               if r.get('ok') and r.get('all_gates_pass')]
    if not passing:
        return {'eligible': [], 'selected': [],
                'termination_text': rules.get('termination_text'),
                'no_solution': True}
    ranked = sorted(passing, key=lambda cid: _rank_key(results[cid], cid))
    selected = ranked[:int(rules.get('max_selected', 2))]
    return {'eligible': passing, 'selected': selected,
            'termination_text': rules.get('termination_text'),
            'no_solution': False}


def _rank_key(result, candidate_id):
    m = result.get('metrics') or {}
    gates = result.get('gates') or {}
    yaw = m.get('max_yaw_abs_from_start_deg')
    fwd = None
    if 'body_net_forward_ge_0.08' in gates:
        fwd = gates['body_net_forward_ge_0.08'].get('value')
    drift = m.get('support_drift_m')
    return (
        abs(yaw) if isinstance(yaw, (int, float)) else float('inf'),
        -(fwd if isinstance(fwd, (int, float)) else float('-inf')),
        drift if isinstance(drift, (int, float)) else float('inf'),
        candidate_id,
    )


# ---------------------------------------------------------------- CLI modes
def mode_validate(frozen, runs_root):
    src = verify_source_freeze()
    if is_structured_error(src):
        print(json.dumps(src, sort_keys=True))
        return 1
    valid = validate_candidate_table(frozen)
    if is_structured_error(valid):
        print(json.dumps(valid, sort_keys=True))
        return 1
    root, err = resolve_runs_root(runs_root)
    if err is not None:
        print(json.dumps(err, sort_keys=True))
        return 1
    print(json.dumps({'ok': True, 'source_freeze_entries': src['entries'],
                      'candidate_count': valid['count'],
                      'runs_root': root}, sort_keys=True))
    return 0


def mode_plan(frozen, runs_root):
    valid = validate_candidate_table(frozen)
    if is_structured_error(valid):
        print(json.dumps(valid, sort_keys=True))
        return 1
    by_id = {c['id']: c for c in frozen['candidates']['candidates']}
    for candidate_id in frozen['run_order']['order']:
        cfg = by_id[candidate_id]['config']
        trace = os.path.join(runs_root, candidate_id, 'trace')
        print(' '.join(build_launch_cmd(cfg, trace)))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', choices=['validate', 'plan', 'screening',
                                       'resume', 'repeat'], default='validate')
    ap.add_argument('--runs-root', default=DEFAULT_RUNS_ROOT)
    ap.add_argument('--candidate', action='append', default=None,
                    help='explicit candidate id (plan/validate/repeat only)')
    ap.add_argument('--yes-run', action='store_true',
                    help='required to actually launch anything')
    args = ap.parse_args(argv)

    frozen = load_frozen()

    if args.mode == 'validate':
        return mode_validate(frozen, args.runs_root)
    if args.mode == 'plan':
        return mode_plan(frozen, args.runs_root)
    if args.mode == 'resume':
        # read-only: never launches, never counts an old run as new.
        order = frozen['run_order']['order']
        present = [cid for cid in order
                   if os.path.isfile(os.path.join(args.runs_root, cid,
                                                  'run_result.json'))]
        print(json.dumps({'mode': 'resume', 'read_only': True,
                          'already_persisted': present,
                          'launched': 0}, sort_keys=True))
        return 0
    if args.mode == 'repeat':
        # Separate explicit B3.2 mode; never triggered by screening.
        if not args.yes_run or not args.candidate:
            print(json.dumps(structured_error(
                'repeat_requires_explicit_candidates_and_yes_run', 'repeat',
                None, args.runs_root), sort_keys=True))
            return 2
        print(json.dumps({'mode': 'repeat', 'candidates': args.candidate,
                          'note': 'B3.2 repeat is a separate authorized phase; '
                                  'this authoring build does not run it.'},
                         sort_keys=True))
        return 0
    # screening
    if not args.yes_run:
        print(json.dumps(structured_error(
            'screening_requires_yes_run', 'screening', None, args.runs_root),
            sort_keys=True))
        return 2
    root, err = resolve_runs_root(args.runs_root)
    if err is not None:
        print(json.dumps(err, sort_keys=True))
        return 2
    src = verify_source_freeze()
    if is_structured_error(src):
        print(json.dumps(src, sort_keys=True))
        return 2
    out = run_screening(frozen, root)
    if not out.get('ok'):
        print(json.dumps(out['error'], sort_keys=True))
        return 1
    print(json.dumps({'order': out['order'], 'passing': out['passing']},
                     sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
