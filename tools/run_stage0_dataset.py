#!/usr/bin/env python3
"""Stage 0 dataset driver — orchestration only.

Contract: docs/STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md (REV1.1,
sha256 093459461125b56f71c8c69ad59f218f0a60ce8233fa42a848b371c357274c2c).

This driver ONLY orchestrates. It never generates gait, never collects, never
analyses, and never derives a metric. It invokes the CANONICAL runner
``tools/run_phase_a_episode.py`` (unmodified) and, optionally, the frozen
analyzer ``tools/analyze_measured_gait.py`` (unmodified) as-is.

Responsibilities (RFC §8.1):
  1. theta validation          (RFC §4.2 / §4.3 / §4.4 / §4.5)
  2. seed scheduling           (RFC §5)
  3. canonical runner invocation
  4. frozen-tool orchestration (analyzer only; read-only)
  5. provenance generation     (RFC §4.6 / §6.5)
  6. corpus index emission     (delegated to tools/build_stage0_index.py)

Sampling (RFC §7): the driver consumes a **pre-registered, static theta list**
(`dataset/stage0/stage0_theta_list.json`) that is fixed and hashed BEFORE any
episode executes. It executes exactly that list: no result-dependent expansion,
no early stopping, no reordering, no optimization, no failure-driven selection.
Failure episodes are preserved.

Usage:
  # Offline contract check (no Gazebo)
  python3 tools/run_stage0_dataset.py --check-contract

  # Execute a ring (or several) of the pre-registered list
  source /opt/ros/jazzy/setup.bash && source install/setup.bash
  python3 tools/run_stage0_dataset.py --ring 0
  python3 tools/run_stage0_dataset.py --ring 1 2 3 --allow-overwrite --build-index
"""

import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import time

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
WS = os.path.dirname(_TOOLS)

CANONICAL_RUNNER = os.path.join(_TOOLS, 'run_phase_a_episode.py')
ANALYZER = os.path.join(_TOOLS, 'analyze_measured_gait.py')
INDEX_BUILDER = os.path.join(_TOOLS, 'build_stage0_index.py')
LAUNCH_SRC = os.path.join(WS, 'src', 'nao_gz_sim', 'launch', 'nao_sim.launch.py')

DEFAULT_OUT_ROOT = os.path.join(WS, 'dataset', 'stage0')
DEFAULT_THETA_LIST = os.path.join(DEFAULT_OUT_ROOT, 'stage0_theta_list.json')

# Never write a dataset into surviving/archived frozen evidence (RFC §8.4).
FORBIDDEN_OUT_ROOT_MARKERS = (
    os.path.join('docs', 'task_p0a_gait_search',
                 'stage7_b_only_holdout_reentry_v4_evidence'),
    os.path.join('docs', 'task4_phase4e_pathb_b3_evidence'),
    os.path.join('docs', 'task_p0a_gait_search', 'phase_a_evidence'),
    os.path.join('docs', 'task_p0a_gait_search', 'phase_a_closure_evidence'),
    'backup',
)

# --------------------------------------------------------------------- anchor
# RFC §3.3 — the 27-key canonical Stage 0 anchor. Frozen; do not edit here.
STAGE0_ANCHOR_THETA = {
    'gait_method': 'walk',
    'gait_phase_mode': 'legacy',
    'step_length_m': 0.02,
    'step_frequency_hz': 0.8,
    'swing_height_m': 0.008,
    'double_support_ratio': 0.3,
    'leg_extension_m': 0.24401,
    'sway_amplitude_rad': 0.06,
    'sway_sign': -1,
    'foot_pitch_swing_rad': 0.1,
    'walking_direction': 'forward',
    'command_rate_hz': 100.0,
    'start_delay_sec': 0.5,
    'gait_blend_sec': 1.5,
    'duration_sec': 16.0,
    'stop_ramp_sec': 1.5,
    'hold_final_sec': 1.0,
    'stand_sec': 24.0,
    'arm_swing_rad': 0.12,
    'arm_neutral_pitch_rad': 1.5,
    'arm_neutral_roll_rad': 0.0,
    'arm_phase_offset_rad': 0.0,
    'arm_swing_sign': 1,
    'elbow_bend_rad': 0.3,
    'actuator_p_gain': 15.0,
    'actuator_max_velocity': 4.0,
    'actuator_hold_on_start': True,
}
REQUIRED_KEYS = frozenset(STAGE0_ANCHOR_THETA)
EXPECTED_ANCHOR_THETA_HASH = \
    '28e795b29921a9061c910b53aaccddbf88ecd5f300ad24ea93079cb5c4e94998'

# RFC §3.2 — duration is supplied by the canonical runner's --duration argument,
# not by a theta override (RFC §4.6: source_category = runner_cli_argument).
DURATION_FROM_CLI = 'duration_sec'

# RFC §3.3 — behavior-critical, anchor-defining; MUST NOT be varied in Stage 0.
ANCHOR_DEFINING = ('sway_sign', 'gait_phase_mode')

# RFC §4.4 — validated against phase4b_producer._validate_parameters.
RANGES = {
    'step_length_m': (0.0, 0.08),
    'step_frequency_hz': (0.2, 2.0),
    'swing_height_m': (0.0, 0.05),
    'double_support_ratio': (0.0, 0.6),      # upper exclusive (producer: < 0.6)
    'leg_extension_m': (0.15, 0.24801),
    'sway_amplitude_rad': (0.0, 0.35),
    'foot_pitch_swing_rad': (0.0, 0.4),
}
ENUMS = {
    'sway_sign': (-1, 1),
    'gait_phase_mode': ('legacy', 'explicit_support_swing'),
    'gait_method': ('stand', 'walk'),
    'walking_direction': ('forward', 'backward'),
    'arm_swing_sign': (-1, 1),
}
# RFC §4.5 — producer-internal / not launch-exposed. MUST NOT appear in theta.
NON_EXPOSED = ('max_delta_per_cycle', 'max_step_length_m', 'max_swing_height_m')

BASE_POSE_TOLERANCE_SEC = 0.02  # RFC §6.4 (frozen constant)


class ContractError(Exception):
    """A theta/provenance contract violation. Aborts before launch."""


# ------------------------------------------------------------------- hashing
def canonical_bytes(obj):
    """RFC §3.2 canonical serialization (no trailing newline)."""
    return json.dumps(obj, indent=2, sort_keys=True, allow_nan=False).encode('utf-8')


def sha256_hex(obj):
    return hashlib.sha256(canonical_bytes(obj)).hexdigest()


theta_hash = sha256_hex


def _launch_base():
    """Frozen launch template, read from the frozen runner (not re-typed)."""
    import run_phase_a_episode as canon  # noqa: E402  (import only)
    return list(canon.R.LAUNCH_BASE)


def config_hash(resolved_theta):
    """RFC §6.6 — canonical theta + the frozen launch template."""
    material = canonical_bytes(resolved_theta) + b'\n' + \
        ' '.join(sorted(_launch_base())).encode('utf-8')
    return hashlib.sha256(material).hexdigest()


def sha256_file(path):
    if not os.path.isfile(path):
        return None
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


# ---------------------------------------------------------------- validation
def declared_launch_args():
    """Names declared by the frozen launch file (parsed, never modified)."""
    with open(LAUNCH_SRC, 'r', encoding='utf-8') as fh:
        text = fh.read()
    return set(re.findall(r"DeclareLaunchArgument\(\s*'([A-Za-z0-9_]+)'", text))


def launch_name_for(key):
    return key if key.startswith('actuator_') else 'phase4b_' + key


def validate_theta(theta, allowed_keys=REQUIRED_KEYS):
    """Return a list of contract violations (RFC §4.2–§4.5)."""
    errors = []
    keys = set(theta)

    for k in NON_EXPOSED:
        if k in keys:
            errors.append('theta contains non-exposed/internal key %r (RFC §4.5)' % k)

    missing = sorted(allowed_keys - keys)
    if missing:
        errors.append('theta missing required keys: %s (RFC §4.2)' % ', '.join(missing))
    unknown = sorted(keys - allowed_keys)
    if unknown:
        errors.append('theta has unknown keys: %s (RFC §4.3, fail-closed)'
                      % ', '.join(unknown))

    declared = declared_launch_args()
    for k in sorted(keys):
        name = launch_name_for(k)
        if name not in declared:
            errors.append('theta key %r maps to launch arg %r which is NOT '
                          'launch-declared (RFC §4.3)' % (k, name))

    for k, (lo, hi) in RANGES.items():
        if k not in theta:
            continue
        v = theta[k]
        if not isinstance(v, (int, float)) or isinstance(v, bool):
            errors.append('%s must be numeric, got %r' % (k, v))
            continue
        if v < lo or v > hi:
            errors.append('%s=%r out of producer range [%s, %s] (RFC §4.4)'
                          % (k, v, lo, hi))
    if 'double_support_ratio' in theta and theta['double_support_ratio'] >= 0.6:
        errors.append('double_support_ratio must be < 0.6 (producer range)')

    for k, allowed in ENUMS.items():
        if k in theta and theta[k] not in allowed:
            errors.append('%s=%r not in %s (RFC §4.4)' % (k, theta[k], allowed))
    return errors


def resolve_like_canonical(theta):
    """Reproduce tools/run_phase_a_episode.resolve_config + duration assignment."""
    import run_phase_a_episode as canon  # noqa: E402
    overrides = {k: v for k, v in theta.items() if k != DURATION_FROM_CLI}
    resolved = dict(canon.FROZEN_GAIT)
    resolved.update(overrides)
    resolved[DURATION_FROM_CLI] = float(theta[DURATION_FROM_CLI])
    return resolved


def overrides_for(theta):
    """theta -> canonical runner `--override k=v` argument list (RFC §3.6)."""
    args = []
    for k in sorted(theta):
        if k == DURATION_FROM_CLI:
            continue  # supplied via --duration (RFC §4.6)
        args += ['--override', '%s=%s' % (k, json.dumps(theta[k]))]
    return args


# --------------------------------------------------------------- theta list
def load_theta_list(path):
    """Load and verify the pre-registered static theta list (RFC §7)."""
    if not os.path.isfile(path):
        raise ContractError('theta list not found: %s' % path)
    with open(path, 'r', encoding='utf-8') as fh:
        doc = json.load(fh)
    entries = doc.get('theta_list')
    if not isinstance(entries, list) or not entries:
        raise ContractError('theta list is empty or malformed: %s' % path)
    recomputed = sha256_hex(entries)
    declared = doc.get('theta_list_hash')
    if recomputed != declared:
        raise ContractError(
            'theta_list_hash mismatch: file declares %s, recomputed %s — the list '
            'was modified after declaration. STOP.' % (declared, recomputed))
    ids = [e.get('theta_id') for e in entries]
    if len(set(ids)) != len(ids):
        raise ContractError('theta_id values are not unique')
    return doc, entries, recomputed


# ------------------------------------------------------------------ plumbing
def check_out_root(out_root):
    real = os.path.realpath(out_root)
    for marker in FORBIDDEN_OUT_ROOT_MARKERS:
        m = os.path.realpath(os.path.join(WS, marker))
        if real == m or real.startswith(m + os.sep):
            raise ContractError('refusing output root inside frozen evidence: %s' % out_root)


def require_ros():
    import shutil
    if not shutil.which('ros2'):
        raise ContractError('ros2 not on PATH; source ROS before running '
                            '(source /opt/ros/jazzy/setup.bash && source install/setup.bash)')


def read_json(path):
    if not os.path.isfile(path):
        return None
    with open(path, 'r', encoding='utf-8') as fh:
        return json.load(fh)


def git_commit():
    try:
        p = subprocess.run(['git', '-C', WS, 'rev-parse', 'HEAD'],
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
        return p.stdout.strip() or None
    except OSError:
        return None


# --------------------------------------------------------- episode invocation
def invoke_canonical_runner(episode_id, seed, theta, out_root, timeout, domain_id,
                            allow_overwrite):
    """Invoke tools/run_phase_a_episode.py (unmodified) as a subprocess."""
    cmd = [sys.executable, CANONICAL_RUNNER,
           '--episode-id', episode_id,
           '--seed', str(seed),
           '--duration', str(theta[DURATION_FROM_CLI]),
           '--out-root', out_root,
           '--timeout', str(timeout),
           '--domain-id', str(domain_id)]
    cmd += overrides_for(theta)
    if allow_overwrite:
        cmd += ['--allow-overwrite']
    log_path = os.path.join(os.path.dirname(out_root), episode_id + '.runner.log')
    started = time.time()
    with open(log_path, 'w', encoding='utf-8') as log:
        proc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT,
                              env=dict(os.environ))
    return {'argv': cmd, 'exit_code': proc.returncode, 'log': log_path,
            'wall_sec': time.time() - started}


def run_analyzer(episode_dir, out_path):
    """Invoke the frozen analyzer as-is (read-only). Never derives metrics."""
    cmd = [sys.executable, ANALYZER, '--run', episode_dir, '--out', out_path]
    proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    report = read_json(out_path)
    measured = None
    if report and report.get('runs'):
        measured = report['runs'][0]
    return {'argv': cmd, 'exit_code': proc.returncode,
            'output': out_path, 'measured': measured,
            'stdout_tail': (proc.stdout or '')[-500:]}


def vs_source_hash(provenance):
    """RFC §6.5 source_hash: frozen sources + this driver."""
    out = {}
    src = ((provenance or {}).get('code_revision') or {}).get('source_sha256') or {}
    out.update(src)
    out['tools/run_stage0_dataset.py'] = sha256_file(os.path.abspath(__file__))
    return out


def build_record(entry, ring_label, ring_index, seed, run, ep_dir, analyzer_enabled,
                 record_dir, theta_list_hash, theta_list_declared_at):
    """Assemble the per-episode record from the episode's OWN artifacts."""
    theta = entry['theta']
    theta_id = entry['theta_id']
    episode_id = os.path.basename(ep_dir)

    provenance = read_json(os.path.join(ep_dir, 'provenance.json'))
    manifest = read_json(os.path.join(ep_dir, 'manifest.json'))
    integrity = read_json(os.path.join(ep_dir, 'episode_integrity.json'))
    resolved = read_json(os.path.join(ep_dir, 'run_config.json'))

    rec = {
        'artifact': 'Stage 0 episode record',
        'schema': 'stage0_episode_record_1.0.0',
        'episode_id': episode_id,
        'theta_id': theta_id,
        'ring': ring_label,
        'ring_id': entry['ring_id'],
        'ring_index': ring_index,
        'sampling_reason': entry.get('sampling_reason'),
        'declared_theta': theta,
        'resolved_theta': resolved,
        'episode_dir': os.path.relpath(ep_dir, WS),
        'theta_list_hash': theta_list_hash,
        'theta_list_declared_at': theta_list_declared_at,
        'runner': {
            'canonical': os.path.relpath(CANONICAL_RUNNER, WS),
            'exit_code': run['exit_code'],
            'wall_sec': run['wall_sec'],
            'log': os.path.relpath(run['log'], WS),
        },
        'base_pose_tolerance_sec': BASE_POSE_TOLERANCE_SEC,
        'experiment_seed': seed,
        'gazebo_seed': (provenance or {}).get('seed', {}).get('requested'),
        'seed_verified': (provenance or {}).get('seed', {}).get('verified'),
        'seed_semantics': (provenance or {}).get('seed', {}).get('semantics'),
        'seed_partition': (provenance or {}).get('seed', {}).get('partition'),
        'seed_ros_domain_id': (provenance or {}).get('seed', {}).get('ros_domain_id'),
        'episode_wall_start': ((provenance or {}).get('timing') or {}).get('start_timestamp'),
        'git_commit': (provenance or {}).get('code_revision', {}).get('git_commit')
                      or git_commit(),
        'source_sha256': vs_source_hash(provenance),
        'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'episode_present': os.path.isdir(ep_dir),
    }

    if resolved is None:
        rec['theta_contract'] = {'ok': False, 'reason': 'run_config.json missing'}
        rec['theta_hash'] = None
        rec['config_hash'] = None
        rec['metrics'] = None
        rec['execution_success'] = False
        rec['behavior_success'] = None
        rec['episode_integrity'] = None
        rec['termination_status'] = None
        rec['metrics_error'] = 'episode artifacts missing'
        return rec

    keys = set(resolved)
    rec['theta_contract'] = {
        'ok': keys == REQUIRED_KEYS,
        'key_count': len(resolved),
        'expected_key_count': len(REQUIRED_KEYS),
        'unexpected_keys': sorted(keys - REQUIRED_KEYS),
        'missing_keys': sorted(REQUIRED_KEYS - keys),
    }
    rec['theta_hash'] = theta_hash(resolved)
    rec['config_hash'] = config_hash(resolved)

    if theta == STAGE0_ANCHOR_THETA:
        rec['anchor_theta_hash_expected'] = EXPECTED_ANCHOR_THETA_HASH
        rec['anchor_theta_hash_match'] = \
            (rec['theta_hash'] == EXPECTED_ANCHOR_THETA_HASH)

    rec['episode_integrity'] = (integrity or {}).get('episode_integrity')
    rec['failure_class'] = (integrity or {}).get('failure_class')
    rec['execution_success'] = rec['episode_integrity'] == 'PASS'
    go = (integrity or {}).get('gait_outcome') or {}
    rec['behavior_success'] = go.get('walking_pass') if go else None
    rec['gait_outcome'] = go or None
    rec['termination_status'] = {
        'collection_status': (manifest or {}).get('collection_status'),
        'termination': (integrity or {}).get('termination'),
        'finish_reason': ((integrity or {}).get('termination_detail') or {}).get('finish_reason'),
    }

    rec['metrics'] = None
    rec['metrics_source'] = None
    rec['metrics_error'] = None
    if analyzer_enabled:
        metrics_path = os.path.join(record_dir, '%s.metrics.json' % episode_id)
        try:
            res = run_analyzer(ep_dir, metrics_path)
            if res['measured'] is not None:
                rec['metrics'] = res['measured']
                rec['metrics_source'] = os.path.relpath(ANALYZER, WS)
            else:
                rec['metrics_error'] = 'analyzer produced no run record (exit=%s)' \
                    % res['exit_code']
        except (OSError, ValueError) as exc:
            rec['metrics_error'] = '%s: %s' % (type(exc).__name__, exc)
    return rec


# ------------------------------------------------------------------- commands
def cmd_check_contract(args):
    errors = validate_theta(STAGE0_ANCHOR_THETA)
    resolved = resolve_like_canonical(STAGE0_ANCHOR_THETA)
    th = theta_hash(resolved)
    report = {
        'mode': 'check-contract',
        'theta_key_count': len(STAGE0_ANCHOR_THETA),
        'validation_errors': errors,
        'resolved_key_count': len(resolved),
        'resolved_keys_match_required': set(resolved) == REQUIRED_KEYS,
        'predicted_theta_hash': th,
        'expected_anchor_theta_hash': EXPECTED_ANCHOR_THETA_HASH,
        'theta_hash_match': th == EXPECTED_ANCHOR_THETA_HASH,
        'anchor_defining_held': {k: STAGE0_ANCHOR_THETA[k] for k in ANCHOR_DEFINING},
        'duration_source': 'runner_cli_argument (--duration)',
        'config_hash': config_hash(resolved),
        'launch_declared_arg_count': len(declared_launch_args()),
        'ok': (not errors) and set(resolved) == REQUIRED_KEYS
              and th == EXPECTED_ANCHOR_THETA_HASH,
    }
    # theta-list integrity (if present)
    try:
        doc, entries, recomputed = load_theta_list(args.theta_list)
        counts = {}
        for e in entries:
            counts[e['ring_id']] = counts.get(e['ring_id'], 0) + 1
        report['theta_list'] = {
            'path': os.path.relpath(args.theta_list, WS),
            'theta_count': len(entries),
            'ring_counts': {str(k): counts[k] for k in sorted(counts)},
            'theta_list_hash': recomputed,
            'declared_at': doc.get('declared_at'),
            'hash_verified': True,
        }
    except ContractError as exc:
        report['theta_list'] = {'error': str(exc)}
        report['ok'] = False
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report['ok'] else 2


def cmd_run(args):
    require_ros()
    out_root = os.path.abspath(args.out_root)
    check_out_root(out_root)

    doc, entries, list_hash = load_theta_list(args.theta_list)
    declared_at = doc.get('declared_at')
    list_mtime = os.path.getmtime(args.theta_list)
    if list_mtime > time.time() + 1:
        raise ContractError('theta list mtime is in the future; refusing')

    # Validate EVERY theta up-front, before launching anything.
    all_errors = {}
    for e in entries:
        errs = validate_theta(e['theta'])
        if errs:
            all_errors[e['theta_id']] = errs
    if all_errors:
        raise ContractError('theta list contains contract violations:\n%s'
                            % json.dumps(all_errors, indent=2, sort_keys=True))

    # Pre-flight: the anchor theta must resolve to the contract hash.
    for e in entries:
        if e['theta'] == STAGE0_ANCHOR_THETA:
            pred = theta_hash(resolve_like_canonical(e['theta']))
            if pred != EXPECTED_ANCHOR_THETA_HASH:
                raise ContractError('PREDICTED anchor theta_hash %s != contract %s — STOP'
                                    % (pred, EXPECTED_ANCHOR_THETA_HASH))

    rings = args.ring if args.ring else sorted({e['ring_id'] for e in entries})
    selected = [e for e in entries if e['ring_id'] in rings]
    if not selected:
        raise ContractError('no theta in rings %s' % rings)

    print('[stage0] theta_list_hash=%s theta=%d selected=%d rings=%s seeds=%s'
          % (list_hash, len(entries), len(selected), sorted(rings), args.seeds))
    print('[stage0] theta list declared_at=%s (mtime=%s) — declared BEFORE execution'
          % (declared_at, time.strftime('%Y-%m-%dT%H:%M:%S%z',
                                        time.localtime(list_mtime))))

    record_dir = os.path.join(out_root, 'records')
    os.makedirs(record_dir, exist_ok=True)

    records, blockers = [], []
    for ring_id in sorted(rings):
        ring_dir = os.path.join(out_root, 'ring%d' % ring_id)
        os.makedirs(ring_dir, exist_ok=True)
        ring_entries = [e for e in selected if e['ring_id'] == ring_id]
        eff_cfg = {
            'artifact': 'Stage 0 sampling config (effective)',
            'ring_id': ring_id,
            'theta_ids': [e['theta_id'] for e in ring_entries],
            'theta_list_path': os.path.relpath(args.theta_list, WS),
            'theta_list_hash': list_hash,
            'theta_list_declared_at': declared_at,
            'seeds': args.seeds,
            'output_root': os.path.relpath(out_root, WS),
            'canonical_runner': os.path.relpath(CANONICAL_RUNNER, WS),
            'git_commit': git_commit(),
        }
        with open(os.path.join(ring_dir, 'stage0_sampling_config.json'), 'w',
                  encoding='utf-8') as fh:
            json.dump(eff_cfg, fh, indent=2, sort_keys=True, allow_nan=False)

        for idx, entry in enumerate(ring_entries):
            for seed in args.seeds:
                episode_id = '%s_s%d' % (entry['theta_id'], seed)
                ep_dir = os.path.join(ring_dir, episode_id)
                print('[stage0] ring%d %s seed=%s' % (ring_id, entry['theta_id'], seed))
                run = invoke_canonical_runner(episode_id, seed, entry['theta'], ep_dir,
                                              args.timeout, args.domain_id,
                                              args.allow_overwrite)
                rec = build_record(entry, 'ring%d' % ring_id, idx, seed, run, ep_dir,
                                   not args.no_metrics, record_dir, list_hash,
                                   declared_at)
                with open(os.path.join(record_dir, episode_id + '.record.json'), 'w',
                          encoding='utf-8') as fh:
                    json.dump(rec, fh, indent=2, sort_keys=True, allow_nan=False)
                records.append(rec)
                print('[stage0]   exit=%s integrity=%s theta_hash=%s'
                      % (run['exit_code'], rec['episode_integrity'],
                         (rec['theta_hash'] or '')[:16]))

                # Contract guard only (RFC §3.2/§9.2): STOP on a hash mismatch for
                # the anchor. This is not sampling adaptivity.
                if entry['theta'] == STAGE0_ANCHOR_THETA and \
                        rec.get('anchor_theta_hash_match') is False:
                    blockers.append({'episode_id': episode_id,
                                     'theta_hash': rec['theta_hash'],
                                     'expected': EXPECTED_ANCHOR_THETA_HASH,
                                     'kind': 'theta_hash_mismatch'})

    if blockers:
        bp = os.path.join(out_root, 'IMPLEMENTATION_BLOCKER_REPORT.json')
        with open(bp, 'w', encoding='utf-8') as fh:
            json.dump({'blockers': blockers, 'records': records}, fh, indent=2,
                      sort_keys=True, allow_nan=False)
        sys.stderr.write('stage0: BLOCKER written to %s\n' % bp)
        return 2

    if args.build_index:
        subprocess.run([sys.executable, INDEX_BUILDER, '--root', out_root],
                       env=dict(os.environ))
    print('[stage0] complete: %d episode(s) over rings %s' % (len(records), sorted(rings)))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description='Stage 0 dataset driver (orchestration only)')
    ap.add_argument('--check-contract', action='store_true',
                    help='offline: validate theta + assert the anchor hash (no Gazebo)')
    ap.add_argument('--ring', type=int, nargs='+', default=None,
                    help='ring id(s) to execute; default = all rings in the theta list')
    ap.add_argument('--seeds', type=int, nargs='+', default=[0, 1, 2])
    ap.add_argument('--out-root', default=DEFAULT_OUT_ROOT)
    ap.add_argument('--theta-list', default=DEFAULT_THETA_LIST)
    ap.add_argument('--timeout', type=float, default=300.0)
    ap.add_argument('--domain-id', type=int, default=88)
    ap.add_argument('--allow-overwrite', action='store_true')
    ap.add_argument('--no-metrics', action='store_true',
                    help='skip frozen-analyzer orchestration')
    ap.add_argument('--build-index', action='store_true',
                    help='invoke tools/build_stage0_index.py after the run')
    args = ap.parse_args(argv)

    try:
        if args.check_contract:
            return cmd_check_contract(args)
        return cmd_run(args)
    except ContractError as exc:
        sys.stderr.write('run_stage0_dataset: CONTRACT ERROR: %s\n' % exc)
        return 2


if __name__ == '__main__':
    sys.exit(main())
