#!/usr/bin/env python3
"""Stage 7'' — static PRECHECK (offline). Never starts Gazebo/ROS.

Covers the PHASE 6 requirements: directory isolation, seed freshness, B config,
source-manifest binding + ordering, gate hashes, recovery/no-retry/no-seed-swap,
protocol-violation taxonomy, strict reason match, runtime-failure handling,
empty runs/ledger, no active processes, and historical/failure evidence hashes.
"""

import hashlib
import json
import os
import subprocess
import sys
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
EVID = os.path.dirname(_HERE)
WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(_HERE))))
for _p in (_HERE, os.path.join(WS, 'tools')):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import stage7r_authorization as AUTH  # noqa: E402
import stage7r_attempt_policy as POLICY  # noqa: E402
import stage7r_failure_handling as FH  # noqa: E402


def load(name):
    p = os.path.join(EVID, name)
    return json.load(open(p, encoding='utf-8')) if os.path.isfile(p) else None


def sh(p):
    return hashlib.sha256(open(p, 'rb').read()).hexdigest() if os.path.isfile(p) else None


def sha_check(d):
    r = subprocess.run(['sha256sum', '-c', 'SHA256SUMS'], cwd=d, stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, text=True).stdout
    return (sum(1 for l in r.splitlines() if l.endswith(': OK')),
            sum(1 for l in r.splitlines() if 'FAILED' in l))


def main():
    snap = load('Stage7r_B_config_snapshot.json')
    cd = load('Stage7r_config_diff.json')
    gp = load('Stage7r_gate_policy.json')
    prereg = load('Stage7r_preregistration.json')
    alloc = load('Stage7r_seed_allocation.json')
    src = load('Stage7r_source_manifest.json')
    asset = load('Stage7r_asset_manifest.json')
    env = load('Stage7r_environment_manifest.json')
    policy = load('Stage7r_attempt_policy.json')
    ledger = load('Stage7r_authorization_ledger.json')
    pathrep = load('Stage7r_path_provenance_repair.json')
    checks = {}

    def c(n, ok, d=None):
        checks[n] = {'pass': bool(ok), 'detail': d}

    bc = (snap or {}).get('expected_config', {})
    seeds = (alloc or {}).get('seeds', [])
    hist_old = os.path.join(WS, 'docs', 'task_p0a_gait_search',
                            'stage7_b_only_holdout_confirmation_evidence')
    hist_prev = os.path.join(WS, 'docs', 'task_p0a_gait_search',
                             'stage7_b_only_holdout_reentry_evidence')
    hist_failed_batch = os.path.join(WS, 'docs', 'task_p0a_gait_search',
                                     'stage7_b_only_holdout_reentry_v2_evidence')

    c('1_new_dir_not_overwrite_history',
      os.path.realpath(EVID) not in (os.path.realpath(hist_old), os.path.realpath(hist_prev),
                                     os.path.realpath(hist_failed_batch))
      and all(os.path.isdir(x) for x in (hist_old, hist_prev, hist_failed_batch)),
      {'old': os.path.relpath(hist_old, WS), 'prev': os.path.relpath(hist_prev, WS),
       'failed_batch': os.path.relpath(hist_failed_batch, WS)})
    c('2_new_charter_frozen', os.path.isfile(os.path.join(EVID, 'Stage7r_charter.md'))
      and os.path.isfile(os.path.join(EVID, 'Stage7r_charter_amendment_F1_F3.md')),
      {'charter': sh(os.path.join(EVID, 'Stage7r_charter.md'))})
    c('3_seeds_published', seeds and seeds == (prereg or {}).get('preregistered_seeds'), seeds)
    scan = (alloc or {}).get('freshness_scan_scope', {})
    c('4_seed_freshness_scanned', bool(scan) and scan.get('n_files_scanned', 0) > 0,
      {'n_files_scanned': scan.get('n_files_scanned')})
    forbidden = set((alloc or {}).get('forbidden_or_historical_seeds', []))
    c('5_seeds_not_forbidden', bool(seeds) and not (set(seeds) & forbidden),
      {'intersection': sorted(set(seeds) & forbidden)})
    c('6_b_config_explicit', bc.get('gait_phase_mode') == 'explicit_support_swing'
      and (cd or {}).get('B_vs_canonical', {}).get('pass') is True
      and all(x['field'] == 'gait_phase_mode' for x in (cd or {}).get('B_vs_canonical', {}).get('diff', [])),
      bc.get('gait_phase_mode'))
    c('7_step_frequency_0.8', bc.get('step_frequency_hz') == 0.8, bc.get('step_frequency_hz'))
    c('8_sway_sign_-1', bc.get('sway_sign') == -1, bc.get('sway_sign'))
    c('9_gate_file_hash_frozen',
      (gp or {}).get('executable_hard_gates', {}).get('sha256') == (prereg or {}).get('executable_gate_file_sha256')
      == 'bf6e75e7d5f479b2aaba6143a2b3370a602bf643f4f5df401ab47e81f94d38c5',
      (gp or {}).get('executable_hard_gates', {}).get('sha256'))
    c('10_v3_provisional_clear',
      (gp or {}).get('v3_gate_frozen') is False
      and (gp or {}).get('strict_v3_compliance') == 'UNVERIFIED'
      and (gp or {}).get('fk_landing_valid', {}).get('status') == 'UNVERIFIED',
      {'v3_gate_frozen': (gp or {}).get('v3_gate_frozen')})
    c('11_diagnostic_policy_preregistered',
      (gp or {}).get('diagnostic_failure_policy', {}).get('affects_episode_pass') is False,
      (gp or {}).get('diagnostic_failure_policy'))
    c('12_recovery_policy_preregistered',
      (gp or {}).get('recovery_policy', {}).get('fatal_integrity_failure', {}).get('automatic') is False
      and (gp or {}).get('recovery_policy', {}).get('environment_failure', {}).get('automatic') is False
      and (gp or {}).get('recovery_policy', {}).get('protocol_violation', {}).get('stop_stage') is True,
      (gp or {}).get('recovery_policy'))
    c('13_per_attempt_authorization',
      (prereg or {}).get('per_attempt_authorization_required') is True
      and isinstance(ledger, dict) and ledger.get('authorizations') == [],
      {'ledger_empty': True})
    c('14_source_manifest_all_scripts',
      len((src or {}).get('execution_scripts', [])) >= 12
      and all(s.get('sha256') for s in (src or {}).get('execution_scripts', [])),
      {'n_execution_scripts': len((src or {}).get('execution_scripts', []))})
    mismatch = [s['path'] for s in (src or {}).get('execution_scripts', [])
                if sh(os.path.join(EVID, s['path'])) != s.get('sha256')]
    c('15_scripts_hash_locked_no_silent_change', not mismatch, {'mismatch': mismatch})
    c('16_path_provenance_rules', pathrep is not None and 'normalization_rule' in (pathrep or {}),
      {'defect': (pathrep or {}).get('defect', {}).get('id')})
    c('17_no_retry_no_seed_swap_encoded',
      (prereg or {}).get('automatic_retry') is False
      and (prereg or {}).get('seed_replacement_allowed') is False
      and (prereg or {}).get('stop_rules', {}).get('any_fatal_failure', {}).get('no_automatic_retry') is True,
      (prereg or {}).get('stop_rules'))
    c('18_no_active_gazebo_ros_controller', FH.residual_scan() == 0,
      {'residual_processes': FH.residual_scan()})
    ok_auth, info = AUTH.authorize_attempt(1, seeds[0] if seeds else -1)
    c('19_executor_fail_closed_without_authorization',
      ok_auth is False and info.get('reason') == 'no_single_unconsumed_authorization_for_attempt',
      info)
    c('20_no_new_episode_started',
      os.path.isdir(os.path.join(EVID, 'runs')) and not os.listdir(os.path.join(EVID, 'runs')),
      {'runs_empty': True})

    # F1 / F2 / F3 / runtime
    types = (policy or {}).get('attempt_types', [])
    c('21_protocol_violation_taxonomy_present', 'protocol_violation' in types and len(types) == 9, types)
    pv = (policy or {}).get('protocol_violation', {})
    c('22_protocol_violation_stop_policy_consistent',
      pv.get('counts_as_episode') is False and pv.get('recovery_allowed') is False
      and pv.get('stop_stage') is True and pv.get('automatic_retry') is False
      and pv.get('seed_replacement') is False, pv)
    sm = (prereg or {}).get('source_manifest', {})
    c('23_source_manifest_hash_embedded_in_preregistration',
      sm.get('path') == 'Stage7r_source_manifest.json' and bool(sm.get('sha256')), sm)
    c('24_source_manifest_hash_matches', sh(os.path.join(EVID, 'Stage7r_source_manifest.json')) == sm.get('sha256'),
      {'embedded': sm.get('sha256'), 'actual': sh(os.path.join(EVID, 'Stage7r_source_manifest.json'))})
    sm_e = (src or {}).get('created_at_epoch')
    pre_e = (prereg or {}).get('created_at_epoch')
    c('25_source_manifest_created_before_preregistration_freeze',
      isinstance(sm_e, (int, float)) and isinstance(pre_e, (int, float)) and sm_e < pre_e,
      {'manifest_epoch': sm_e, 'prereg_epoch': pre_e})
    c('26_reason_match_strict',
      (policy or {}).get('reason_match_strict') is True
      and (prereg or {}).get('reason_match_strict') is True
      and (policy or {}).get('initial_episode_requires_authorization') is True,
      {'reason_match_strict': (policy or {}).get('reason_match_strict')})
    rt = AUTH.reason_self_test()
    c('27_initial_episode_reason_cannot_bypass_recovery',
      rt.get('2_recovery_auth_cannot_take_initial_request') is True
      and rt.get('3_env_recovery_auth_cannot_take_initial_request') is True
      and rt.get('9_recovery_cannot_borrow_initial_auth') is True, rt)
    at = AUTH.self_test()
    c('28_authorization_tests_pass', all(at.values()) and all(rt.values()),
      {'authorization_self_test': at, 'reason_self_test': rt})
    fh = FH.self_test()
    execd_src = open(os.path.join(EVID, 'scripts/stage7r_execd.py'), encoding='utf-8').read()
    c('29_runtime_failure_handling_present',
      all(fh.values()) and 'stage7r_env' in execd_src
      and 'expected_fingerprint_hash' in execd_src
      and 'build_failure_record' in execd_src and 'write_failure' in execd_src,
      {'failure_handling_self_test': fh})
    hist = {
        'stage7_confirmation': ('docs/task_p0a_gait_search/stage7_b_only_holdout_confirmation_evidence', 90),
        'stage6_screening': ('docs/task_p0a_gait_search/stage6_v3_minimal_ab_screening_evidence', 36),
        'stage6_reentry': ('docs/task_p0a_gait_search/stage6_v3_minimal_ab_screening_reentry_evidence', 17),
        'stage7p_reentry_failure': ('docs/task_p0a_gait_search/stage7_b_only_holdout_reentry_evidence', 56),
        'stage7r_current_failed_batch': ('docs/task_p0a_gait_search/stage7_b_only_holdout_reentry_v2_evidence', 78),
        'stage7r_v3': ('docs/task_p0a_gait_search/stage7_b_only_holdout_reentry_v3_evidence', 42),
        'stage0_qualification': ('docs/task_p0a_gait_search/stage0_infrastructure_qualification', 32),
    }
    hist_res = {}
    for k, (d, _n) in hist.items():
        ok, bad = sha_check(os.path.join(WS, d))
        hist_res[k] = {'ok': ok, 'failed': bad}
    fail_ok = all(os.path.isfile(os.path.join(EVID, '..', 'stage7_b_only_holdout_reentry_evidence',
                                              'runs/seed-909101', f))
                  for f in ('launch.log', 'launch_cmd.txt', 'provenance.json'))
    runner_ok = sh(os.path.join(WS, 'tools/run_p0a_gait_candidate.py')) == \
        '6e4d927472f06deba9b1c97432bfc8ac3701905a3d7591e47102873ae21f3491'
    c('30_historical_and_failure_evidence_hashes_intact',
      all(v['failed'] == 0 for v in hist_res.values()) and fail_ok and runner_ok,
      {'dirs': hist_res, 'runner_unchanged': runner_ok})

    # F-01 prevention: orchestrator's referenced EVID filenames must exist and be
    # listed in SHA256SUMS (no v1/v2/v3 filename drift).
    import re as _re
    orch_src = open(os.path.join(EVID, 'scripts/stage7r_orchestrator.py'), encoding='utf-8').read()
    names = _re.findall(r"(?:PREREG_NAME|RUN_ORDER_NAME)\s*=\s*'([^']+)'", orch_src)
    sums_path = os.path.join(EVID, 'SHA256SUMS')
    sums = open(sums_path, encoding='utf-8').read() if os.path.isfile(sums_path) else ''
    binding = (len(names) >= 2
               and all(os.path.isfile(os.path.join(EVID, n)) and n in sums for n in names)
               and 'Stage7r_preregistration_v2.json' not in orch_src)
    c('31_orchestrator_filename_binding', binding,
      {'referenced': names, 'exists': [os.path.isfile(os.path.join(EVID, n)) for n in names],
       'in_sha256sums': [n in sums for n in names]})

    # F-01/R2-01: prereg environment fingerprint binds the live environment.
    import stage7r_env as _ENV
    live = _ENV.fingerprint_hash(_ENV.environment_fingerprint(_ENV.bootstrap_environment(WS)))
    c('32_environment_fingerprint_binds_preregistration',
      live == (prereg or {}).get('environment_fingerprint_hash')
      and (prereg or {}).get('environment_fingerprint_version') == _ENV.FINGERPRINT_VERSION,
      {'live': live, 'prereg': (prereg or {}).get('environment_fingerprint_hash')})

    n_pass = sum(1 for v in checks.values() if v['pass'])
    all_pass = n_pass == len(checks)
    out = {'artifact': "Stage 7'' static PRECHECK",
           'created_at_local': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
           'n_checks': len(checks), 'n_pass': n_pass, 'all_pass': all_pass,
           'terminal': 'PRECHECK_PASS_PENDING_AUTHORIZATION' if all_pass else 'PRECHECK_FAIL_STOP',
           'execution_authorization': 'NOT_GRANTED', 'checks': checks}
    tmp = os.path.join(EVID, 'Stage7r_precheck.json.tmp')
    with open(tmp, 'w', encoding='utf-8') as fh:
        json.dump(out, fh, indent=2, sort_keys=True, allow_nan=False)
    os.replace(tmp, os.path.join(EVID, 'Stage7r_precheck.json'))
    print(json.dumps({'n_pass': n_pass, 'n_checks': len(checks), 'terminal': out['terminal'],
                      'failed': [k for k, v in checks.items() if not v['pass']]},
                     indent=2, sort_keys=True))
    return 0 if all_pass else 1


if __name__ == '__main__':
    raise SystemExit(main())
