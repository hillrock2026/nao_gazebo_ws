#!/usr/bin/env python3
"""Stage 7' — attempt taxonomy and recovery policy (F1-repaired, offline).

Resolves the Stage 7 contradiction between
``retry -> fatal_integrity_failure -> stop`` and
``recovery_after_integrity_failure -> same-seed rerun``.

F1 repair: the formal taxonomy now includes ``protocol_violation`` (9 types),
matching the charter / preregistration / gate-policy frozen documents.  A
protocol violation is always terminal: not an episode, no recovery, no automatic
retry, no seed replacement.

Never starts Gazebo/ROS.
"""

import json
import os
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
EVID = os.path.dirname(_HERE)

ATTEMPT_TYPES = [
    'initial_episode',
    'environment_failure',
    'integrity_failure',
    'recovery_after_environment_failure',
    'recovery_after_integrity_failure',
    'repeat_same_seed',
    'valid_protocol_episode',
    'hard_gate_failure',
    'protocol_violation',
]

# Behaviors that MUST be recognized as protocol violations (F1 §3.2).
PROTOCOL_VIOLATIONS = [
    'unauthorized_attempt',
    'unauthorized_same_seed_repeat',
    'unauthorized_recovery',
    'seed_replacement',
    'run_A',
    'continue_after_hard_gate_failure',
    'out_of_order_seed',
    'modify_frozen_gates',
    'modify_diagnostic_policy',
    'silent_script_change_after_freeze',
    'script_not_in_source_manifest',
    'config_or_source_hash_mismatch',
    'forged_or_missing_provenance',
    'episode_count_exceeds_preregistration',
]

POLICY = {
    'artifact': "Stage 7' attempt policy (F1-repaired)",
    'attempt_types': ATTEMPT_TYPES,
    'counts_as_episode': {
        'initial_episode': 'only if trace valid and not integrity/environment failure',
        'recovery_after_environment_failure': 'only if trace valid',
        'recovery_after_integrity_failure': 'only if trace valid',
        'repeat_same_seed': 'only if trace valid',
        'valid_protocol_episode': True,
        'environment_failure': False,
        'integrity_failure': False,
        'hard_gate_failure': True,
        'protocol_violation': False,
    },
    'may_request_recovery': ['environment_failure', 'integrity_failure'],
    'recovery_requires_preexisting_authorization': True,
    'recovery_is_automatic': False,
    'recovery_is_retry': True,
    'recovery_same_seed_allowed': 'only when authorization.same_seed_repeat_allowed is true',
    'max_recovery_per_seed': 1,
    'hard_gate_failure_stop_is_absolute': True,
    'integrity_failure_stop': True,
    'environment_failure_stop': True,
    'protocol_violation': {
        'counts_as_episode': False,
        'recovery_allowed': False,
        'stop_stage': True,
        'automatic_retry': False,
        'seed_replacement': False,
    },
    'protocol_violation_behaviors': PROTOCOL_VIOLATIONS,
    'who_may_authorize_recovery': 'user (explicit, scope-clear, recorded per attempt)',
    'per_attempt_authorization_required_before_start': True,
    'reason_match_strict': True,
    'initial_episode_requires_authorization': True,
    'initial_episode_reason_cannot_bypass_recovery': True,
    'resolved_contradiction': ('integrity_failure always stops the stage; a same-seed '
                               'rerun is a recovery that is permitted only with a '
                               'pre-issued, single-consumption per-attempt authorization. '
                               'It is not an automatic retry.'),
    'forbidden': ['automatic_retry', 'implicit_recovery', 'same_seed_repeat_without_authorization',
                  'hard_gate_failure_continue', 'seed_replacement', 'reason_fallback'],
}


def classification(attempt_type):
    """Return the normative classification for an attempt type (fail closed)."""
    if attempt_type not in ATTEMPT_TYPES:
        raise ValueError('unknown_attempt_type: %r' % attempt_type)
    if attempt_type == 'protocol_violation':
        out = dict(POLICY['protocol_violation'])
        out['attempt_type'] = 'protocol_violation'
        return out
    return {
        'attempt_type': attempt_type,
        'counts_as_episode': POLICY['counts_as_episode'].get(attempt_type),
        'stop_stage': attempt_type != 'valid_protocol_episode' and attempt_type != 'initial_episode'
                     and not attempt_type.startswith('recovery_') and attempt_type != 'repeat_same_seed',
        'recovery_allowed': attempt_type in POLICY['may_request_recovery'],
        'automatic_retry': False,
        'seed_replacement': False,
    }


def is_protocol_violation(behavior):
    return behavior in PROTOCOL_VIOLATIONS


def write(path=None):
    p = path or os.path.join(EVID, 'Stage7r_attempt_policy.json')
    tmp = p + '.tmp'
    payload = dict(POLICY)
    payload['generated_at'] = time.strftime('%Y-%m-%dT%H:%M:%S.%f%z')
    with open(tmp, 'w', encoding='utf-8') as fh:
        json.dump(payload, fh, indent=2, sort_keys=True, allow_nan=False)
    os.replace(tmp, p)
    return p


def self_test():
    checks = {}
    checks['protocol_violation_enum_present'] = 'protocol_violation' in ATTEMPT_TYPES
    checks['taxonomy_has_9_types'] = len(ATTEMPT_TYPES) == 9
    pv = classification('protocol_violation')
    checks['1_protocol_violation_recognized'] = (
        pv['counts_as_episode'] is False and pv['stop_stage'] is True)
    checks['2_protocol_violation_not_valid_episode'] = pv['counts_as_episode'] is False
    checks['3_protocol_violation_no_recovery'] = pv['recovery_allowed'] is False
    checks['4_protocol_violation_no_seed_replacement'] = pv['seed_replacement'] is False
    checks['5_protocol_violation_blocks_orchestrator'] = pv['stop_stage'] is True
    checks['6_protocol_violation_has_stop_reason'] = (
        pv['attempt_type'] == 'protocol_violation' and pv['stop_stage'] is True)
    checks['7_unauthorized_attempt_is_violation'] = is_protocol_violation('unauthorized_attempt')
    checks['hard_gate_stop_absolute'] = POLICY['hard_gate_failure_stop_is_absolute'] is True
    checks['recovery_not_automatic'] = POLICY['recovery_is_automatic'] is False
    checks['recovery_needs_preauth'] = POLICY['recovery_requires_preexisting_authorization'] is True
    checks['no_seed_replacement'] = 'seed_replacement' in POLICY['forbidden']
    checks['reason_match_strict'] = POLICY['reason_match_strict'] is True
    checks['initial_episode_needs_auth'] = POLICY['initial_episode_requires_authorization'] is True
    checks['all_14_violation_behaviors'] = len(PROTOCOL_VIOLATIONS) == 14
    return checks


if __name__ == '__main__':
    import sys
    if '--self-test' in sys.argv:
        st = self_test()
        print(json.dumps({'self_test': st,
                          'n_pass': sum(1 for v in st.values() if v),
                          'n_total': len(st)}, indent=2, sort_keys=True))
    else:
        print(write(sys.argv[1] if len(sys.argv) > 1 else None))
