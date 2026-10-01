#!/usr/bin/env python3
"""Stage 7' — preregistration, gate policy, attempt policy and manifests (offline)."""

import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
EVID = os.path.dirname(_HERE)
WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(_HERE))))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
import stage7r_env as ENV  # noqa: E402

CANONICAL = os.path.join(WS, 'docs', 'task_p0a_gait_search', 'p1bd_evidence', 'P1BD_config_snapshot.json')
V2_GATES = os.path.join(WS, 'docs', 'task_p0a_gait_search', 'p1r_support_swing', 'frozen_gates_v2.json')
V3_SCHEMA = os.path.join(WS, 'docs', 'task_p0a_gait_search',
                         'phase5_p1r_v3_contract_provenance_erratum_evidence',
                         'Phase5_reentry_contract_schema_corrected.json')
PHASE5_EVAL = os.path.join(WS, 'docs', 'task_p0a_gait_search',
                           'phase5_p1r_v3_contract_reentry_evidence',
                           'Phase5_reentry_reference_evaluator.py')

SOURCE_FILES = [
    'docs/task_p0a_gait_search/p1bd_evidence/P1BD_config_snapshot.json',
    'docs/task_p0a_gait_search/p1r_support_swing/frozen_gates_v2.json',
    'docs/task_p0a_gait_search/p1r_support_swing/P1R_charter.md',
    'docs/task_p0a_gait_search/phase5_p1r_v3_contract_provenance_erratum_evidence/Phase5_reentry_contract_schema_corrected.json',
    'docs/task_p0a_gait_search/phase5_p1r_v3_contract_reentry_evidence/Phase5_reentry_reference_evaluator.py',
    'tools/run_p0a_gait_candidate.py',
    'tools/p0a_phase_analysis.py',
    'tools/analyze_measured_gait.py',
    'tools/phase4e_analysis.py',
    'src/nao_gz_sim/scripts/nao_gait_phase.py',
    'src/nao_gz_sim/scripts/phase4b_producer.py',
    'src/nao_gz_sim/launch/nao_sim.launch.py',
    'src/nao_gz_sim/worlds/nao_world.sdf',
]
EXEC_SCRIPTS = [
    'scripts/stage7r_setup.py',
    'scripts/stage7r_seed_allocate.py',
    'scripts/stage7r_authorization.py',
    'scripts/stage7r_attempt_policy.py',
    'scripts/stage7r_preregister.py',
    'scripts/stage7r_precheck.py',
    'scripts/stage7r_execd.py',
    'scripts/stage7r_orchestrator.py',
    'scripts/stage7r_episode_runner.py',
    'scripts/stage7r_finalize.py',
    'scripts/stage7r_path_provenance.py',
    'scripts/stage7r_failure_handling.py',
    'scripts/stage7r_env.py',
]
LEGACY_FILES = ['legacy/stage7_exec_runner_attempt2_old.py']
ASSET_DIRS = ['src/nao_gz_sim/urdf', 'src/nao_description/urdf',
              'install/nao_gz_sim/share/nao_gz_sim/worlds']


def _iso(epoch):
    return time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(epoch)) + \
        '.%06d' % int((epoch % 1) * 1e6) + time.strftime('%z', time.localtime(epoch))


def sha256_file(path):
    if not os.path.isfile(path):
        return None
    return hashlib.sha256(open(path, 'rb').read()).hexdigest()


def atomic_json(path, obj):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        json.dump(obj, fh, indent=2, sort_keys=True, allow_nan=False)
    os.replace(tmp, path)


def dir_manifest(rel):
    root = os.path.join(WS, rel)
    entries = []
    if not os.path.isdir(root):
        return {'path': rel, 'exists': False, 'aggregate_sha256': None}
    for base, dirs, files in os.walk(root):
        dirs.sort()
        for name in sorted(files):
            full = os.path.join(base, name)
            entries.append((os.path.relpath(full, WS), sha256_file(full)))
    lines = ''.join('%s\0%s\n' % (r, h) for r, h in sorted(entries))
    return {'path': rel, 'exists': True, 'n_files': len(entries),
            'aggregate_sha256': hashlib.sha256(lines.encode()).hexdigest(),
            'entries': [{'path': r, 'sha256': h} for r, h in sorted(entries)]}


def cmd(a):
    try:
        return subprocess.run(a, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, timeout=20).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def environment_binding(workspace=None):
    """Compute the prereg-level environment fingerprint binding at freeze time.

    Uses the SAME functions the executor uses (no hard-coded hash).  Raises
    ``ENV.EnvironmentError2`` (fail-closed) when the environment cannot be
    bootstrapped/verified, so a preregistration is never frozen against an
    unverifiable environment.
    """
    workspace = workspace or WS
    boot_env = ENV.bootstrap_environment(workspace)
    ENV.verify_environment(workspace, boot_env)
    fingerprint = ENV.environment_fingerprint(boot_env)
    return {
        'environment_fingerprint_hash': ENV.fingerprint_hash(fingerprint),
        'environment_fingerprint_version': ENV.FINGERPRINT_VERSION,
        'environment_fingerprint_keys': sorted(fingerprint.keys()),
        'environment_fingerprint': fingerprint,
    }


def self_test():
    import tempfile
    checks = {}
    orig = (ENV.bootstrap_environment, ENV.verify_environment,
            ENV.environment_fingerprint, ENV.fingerprint_hash)
    try:
        ENV.bootstrap_environment = lambda *a, **k: {'PATH': '/x', 'AMENT_PREFIX_PATH': '/ws/install'}
        ENV.verify_environment = lambda *a, **k: {'ok': True}
        ENV.environment_fingerprint = lambda env: {'PATH': '/x', 'python_executable': '/usr/bin/python3'}
        ENV.fingerprint_hash = lambda fp: 'deadbeef'
        binding = environment_binding('/tmp/ws')
        checks['embed_hash'] = binding['environment_fingerprint_hash'] == 'deadbeef'
        checks['embed_version'] = binding['environment_fingerprint_version'] == ENV.FINGERPRINT_VERSION
        checks['embed_keys'] = binding['environment_fingerprint_keys'] == ['PATH', 'python_executable']
        checks['embed_fingerprint'] = binding['environment_fingerprint']['PATH'] == '/x'

        def _fail(*a, **k):
            raise ENV.EnvironmentError2('bootstrap_source_failed:test')
        ENV.bootstrap_environment = _fail
        try:
            environment_binding('/tmp/ws')
            checks['bootstrap_failure_refuses'] = False
        except ENV.EnvironmentError2:
            checks['bootstrap_failure_refuses'] = True
    finally:
        (ENV.bootstrap_environment, ENV.verify_environment,
         ENV.environment_fingerprint, ENV.fingerprint_hash) = orig
    return checks


def main():
    reg_path = os.path.join(EVID, 'Stage7r_preregistration.json')
    if os.path.exists(reg_path):
        sys.stderr.write('stage7r_preregister: preregistration exists (freeze-once)\n')
        return 2
    snap = json.load(open(os.path.join(EVID, 'Stage7r_B_config_snapshot.json'), encoding='utf-8'))
    alloc = json.load(open(os.path.join(EVID, 'Stage7r_seed_allocation.json'), encoding='utf-8'))
    order = json.load(open(os.path.join(EVID, 'Stage7r_run_order.json'), encoding='utf-8'))
    policy = json.load(open(os.path.join(EVID, 'Stage7r_attempt_policy.json'), encoding='utf-8'))
    b = snap['expected_config']
    if not (b['step_frequency_hz'] == 0.8 and b['sway_sign'] == -1
            and b['gait_phase_mode'] == 'explicit_support_swing'):
        raise SystemExit('stage7r_preregister: B config not frozen')
    if not alloc.get('all_seeds_fresh') or alloc.get('freshness_scan_hits'):
        raise SystemExit('stage7r_preregister: seed freshness not clean')
    seeds = alloc['seeds']

    gate_policy = {
        'artifact': "Stage 7' executable gate policy (provisional)",
        'created_at_local': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'policy': 'KEEP_PROVISIONAL',
        'v3_gate_frozen': False, 'strict_v3_compliance': 'UNVERIFIED',
        'fk_landing_valid': {'status': 'UNVERIFIED', 'is_pass': False,
                             'is_not_applicable': False,
                             'reason': "no validated Stage 7' raw-trace evaluator"},
        'executable_hard_gates': {'source': os.path.relpath(V2_GATES, WS),
                                  'sha256': sha256_file(V2_GATES), 'n_hard_gates': 18,
                                  'excludes': ['fk_landing_valid'],
                                  'role': 'frozen executable subset of the v3 hard-gate hierarchy'},
        'evaluator': {'source': os.path.relpath(PHASE5_EVAL, WS), 'sha256': sha256_file(PHASE5_EVAL)},
        'gate_schema': {'source': os.path.relpath(V3_SCHEMA, WS), 'sha256': sha256_file(V3_SCHEMA)},
        'diagnostic_metrics': ['zero_contact_max_sec', 'm0_legacy_contact_delay_max_sec',
                               'swing_to_support_onset_delay_sec', 'ds_both_contact_rate',
                               'left_swing_right_support_contact_rate',
                               'right_swing_left_support_contact_rate'],
        'diagnostic_failure_policy': {'allowed': True, 'affects_episode_pass': False,
                                      'affects_stage_pass': False,
                                      'escalation_rule': ('Diagnostics are reported, not hard '
                                                          'verdicts. Hard exceptions: contact trace '
                                                          'missing/empty/non-monotonic/schema-invalid '
                                                          '=> measurable=false; onset n_missing>0 '
                                                          '=> hard FAIL.')},
        'recovery_policy': {
            'hard_gate_failure': {'stop_stage': True, 'recovery': False,
                                  'same_seed_repeat': False, 'seed_replacement': False},
            'fatal_integrity_failure': {'stop_stage': True,
                                        'recovery': 'explicit_preexisting_per_attempt_authorization_only',
                                        'max_recovery_per_seed': policy['max_recovery_per_seed'],
                                        'automatic': False},
            'environment_failure': {'stop_stage': True,
                                    'recovery': 'explicit_preexisting_per_attempt_authorization_only',
                                    'max_recovery_per_seed': policy['max_recovery_per_seed'],
                                    'automatic': False},
            'protocol_violation': {'stop_stage': True, 'recovery': False},
        },
        'claim_wording_required': ("satisfies the P1-R v3 provisional engineering contract under "
                                   "the frozen executable gate subset, with fk_landing_valid=UNVERIFIED"),
        'forbidden_wording': ['strict v3 compliant', 'fully v3 validated', 'final v3 pass'],
    }

    prereg = {
        'artifact': "Stage 7r-next preregistration (new independent B-only holdout batch)",
        'created_at_local': None,  # set equal to created_at at freeze
        'stage': "7r-next", 'batch_id': 'stage7r_batch_v3',
        'title': 'new independent B-only holdout batch', 'mode': 'B_ONLY',
        'gait_phase_mode': 'explicit_support_swing',
        'episode_count': len(seeds), 'seed_count': len(seeds), 'serial_execution': True,
        'historical_stage7_status': 'STAGE_7_FAIL_STOPPED',
        'historical_stage7_authoritative_valid_episode_count': 1,
        'historical_stage7_claim_level': 'single_seed_observation',
        'historical_holdout_confirmation': 'NOT_ESTABLISHED',
        'historical_evidence_modified': False,
        'previous_failed_batches': {
            'stage7_confirmation': {'status': 'STAGE_7_FAIL_STOPPED', 'executed': [909001],
                                    'seeds_not_run': [909002, 909003, 909004, 909005]},
            'stage7p_reentry': {'status': 'STAGE_7R_FAIL_STOPPED', 'executed': [909101],
                                'attempt_type': 'environment_failure', 'seeds_not_run':
                                [909102, 909103, 909104, 909105]},
            'stage7r_v2': {'status': 'STAGE_7R_FAIL_STOPPED', 'executed': [909201],
                           'failed_gate': 'path_over_net', 'seeds_not_run':
                           [909202, 909203, 909204, 909205]}},
        'new_batch_independent': True,
        'failed_batch_seeds_excluded': [909001, 909002, 909003, 909004, 909005,
                                        909101, 909102, 909103, 909104, 909105,
                                        909201, 909202, 909203, 909204, 909205],
        'failed_batch_not_continuation': ('909201 already failed (hard-gate) and 909202-909205 '
                                          'are not resumed; this is a new preregistered batch'),
        'execution_authorization': 'NOT_GRANTED',
        'execution_status': 'NOT_STARTED',
        'v3_gate_frozen': False, 'strict_v3_compliance': 'UNVERIFIED',
        'fk_landing_valid': 'UNVERIFIED',
        'executable_gate_file': os.path.relpath(V2_GATES, WS),
        'executable_gate_file_sha256': sha256_file(V2_GATES),
        'evaluator_source_hash': sha256_file(PHASE5_EVAL),
        'gate_schema_hash': sha256_file(V3_SCHEMA),
        'gate_policy_artifact': 'Stage7r_gate_policy.json',
        'attempt_policy_artifact': 'Stage7r_attempt_policy.json',
        'diagnostic_failure_policy': gate_policy['diagnostic_failure_policy'],
        'recovery_policy': gate_policy['recovery_policy'],
        'per_attempt_authorization_required': True,
        'authorization_ledger_artifact': 'Stage7r_authorization_ledger.json',
        'frozen_config': {'source': os.path.relpath(CANONICAL, WS),
                          'source_sha256': sha256_file(CANONICAL),
                          'gait_phase_mode': 'explicit_support_swing',
                          'step_frequency_hz': 0.8, 'sway_sign': -1},
        'preregistered_seeds': seeds, 'executed_seeds': [],
        'seed_replacement_allowed': False, 'automatic_retry': False,
        'seed_allocation_sha256': alloc['seed_allocation_sha256'],
        'run_order': [r['seed'] for r in order['order']],
        'stop_rules': {
            'any_fatal_failure': {'stop_stage_immediately': True, 'no_automatic_retry': True,
                                  'no_seed_swap': True, 'recovery_requires_preexisting_authorization': True},
            'hard_gate_failure': {'stop_stage_immediately': True, 'recovery': False,
                                  'no_seed_swap': True},
            'complete': {'episodes': len(seeds), 'per_seed': 1}},
        'cannot_claim': ['deterministic_paired_evidence', 'statistical_equivalence',
                         'statistical_significance', 'population_probability_guarantee',
                         'P0-a completion', 'final tolerance', 'P1 final closure',
                         'P2 authorization', 'strict v3 compliance'],
        'claim_level': 'initial_holdout_confirmation',
        'attempt_taxonomy': policy['attempt_types'],
        'protocol_violation': policy['protocol_violation'],
        'reason_match_strict': True,
        'initial_episode_requires_authorization': True,
        'wrapper_change_requires_authorization': True,
        'runtime_failure_repair': ('stage7r_failure_handling.py: environment preflight + '
                                   'structured fatal failure; guard in run_p0a_gait_candidate.py'),
        'status_machine': ['INIT', 'PRECHECK', 'PREREGISTRATION_READY', 'AWAITING_USER_AUTHORIZATION'],
    }

    # R2-01/R2-03: bind the preregistration to the verified environment fingerprint
    # (computed at freeze time with the executor's own functions).  Fail-closed.
    try:
        prereg.update(environment_binding())
    except ENV.EnvironmentError2 as exc:
        sys.stderr.write('stage7r_preregister: environment not verifiable; refusing to '
                         'freeze preregistration: %s\n' % exc)
        return 3

    source_manifest = {
        'artifact': "Stage 7' source manifest",
        'created_at_local': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'algorithm': 'SHA-256',
        'sources': [{'path': p, 'exists': os.path.isfile(os.path.join(WS, p)),
                     'sha256': sha256_file(os.path.join(WS, p))} for p in SOURCE_FILES],
        'execution_scripts': [{'path': p, 'sha256': sha256_file(os.path.join(EVID, p))}
                              for p in EXEC_SCRIPTS],
        'legacy_sources': [{'path': p, 'sha256': sha256_file(os.path.join(EVID, p))}
                           for p in LEGACY_FILES],
        'underlying_runner': {'path': 'tools/run_p0a_gait_candidate.py',
                              'sha256': sha256_file(os.path.join(WS, 'tools/run_p0a_gait_candidate.py'))},
        'charter': {'path': 'Stage7r_charter.md',
                    'sha256': sha256_file(os.path.join(EVID, 'Stage7r_charter.md'))},
        'note': 'Includes ALL execution scripts; freeze-once before any Stage 7 run.',
    }
    asset_manifest = {'artifact': "Stage 7' asset manifest",
                      'created_at_local': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                      'algorithm': 'SHA-256',
                      'asset_dirs': [dir_manifest(d) for d in ASSET_DIRS]}
    env_manifest = {'artifact': "Stage 7' environment manifest",
                    'created_at_local': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                    'host': platform.node(), 'cwd': WS, 'kernel': platform.release(),
                    'platform': platform.platform(), 'machine': platform.machine(),
                    'python': {'executable': sys.executable, 'version': sys.version},
                    'ros_distro_env': os.environ.get('ROS_DISTRO'),
                    'gazebo_version': (cmd(['gz', 'sim', '--version']) or '').splitlines()[0]
                                      if cmd(['gz', 'sim', '--version']) else None,
                    'gz_which': shutil.which('gz'), 'ros2_which': shutil.which('ros2'),
                    'note': 'gazebo_version is a version probe, not a simulation start'}
    ledger = {'artifact': "Stage 7' per-attempt authorization ledger",
              'created_at_local': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
              'authorizations': [],
              'note': ('Empty at preregistration. Each attempt requires an independently issued, '
                       'single-consumption authorization before it starts. No top-level GRANTED '
                       'flag is used.')}

    atomic_json(os.path.join(EVID, 'Stage7r_gate_policy.json'), gate_policy)
    # F2: create the source manifest first, then bind its hash into the prereg.
    sm_epoch = time.time()
    source_manifest['created_at'] = _iso(sm_epoch)
    source_manifest['created_at_epoch'] = sm_epoch
    atomic_json(os.path.join(EVID, 'Stage7r_source_manifest.json'), source_manifest)
    time.sleep(1.2)  # prove manifest strictly precedes preregistration freeze
    sm_hash = sha256_file(os.path.join(EVID, 'Stage7r_source_manifest.json'))
    pre_epoch = time.time()
    prereg['source_manifest'] = {'path': 'Stage7r_source_manifest.json', 'sha256': sm_hash,
                                 'created_at': source_manifest['created_at'],
                                 'created_at_epoch': sm_epoch, 'freeze_status': 'FROZEN'}
    prereg['created_at'] = _iso(pre_epoch)
    prereg['created_at_epoch'] = pre_epoch
    prereg['created_at_local'] = prereg['created_at']  # F-07: keep the two consistent
    prereg['supersedes_note'] = ("Stage 7'' B-only reentry; supersedes the failed Stage 7' "
                                 "batch seeds 909101-909105 (909101 executed, NOT_VALID)")
    atomic_json(os.path.join(EVID, 'Stage7r_asset_manifest.json'), asset_manifest)
    atomic_json(os.path.join(EVID, 'Stage7r_environment_manifest.json'), env_manifest)
    atomic_json(os.path.join(EVID, 'Stage7r_authorization_ledger.json'), ledger)
    atomic_json(reg_path, prereg)
    print(json.dumps({'preregistered': True, 'seeds': seeds,
                      'execution_authorization': 'NOT_GRANTED',
                      'v3_gate_frozen': False}, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    if '--self-test' in sys.argv:
        st = self_test()
        print(json.dumps({'self_test': st, 'n_pass': sum(1 for v in st.values() if v),
                          'n_total': len(st)}, indent=2, sort_keys=True))
        raise SystemExit(0 if all(st.values()) else 1)
    raise SystemExit(main())
