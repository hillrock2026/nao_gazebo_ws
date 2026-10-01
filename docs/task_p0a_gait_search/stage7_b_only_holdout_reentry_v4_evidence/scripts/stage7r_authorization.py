#!/usr/bin/env python3
"""Stage 7' — per-attempt authorization ledger (repaired mechanism, offline).

Fixes the Stage 7 defect where ``authorization_granted()`` only checked a single
top-level ``GRANTED`` flag, allowing one authorization to be reused across
attempts and leaving attempt 3 without an independent authorization entry.

Model:
  * every attempt before it starts must consume exactly one unconsumed
    authorization entry matching (attempt_id, seed);
  * an entry is consumed once (``status`` ISSUED -> CONSUMED, ``consumed_at``);
  * fail closed: no matching unconsumed entry => refuse to launch;
  * a top-level GRANTED flag is never sufficient by itself.

This module never launches Gazebo/ROS.
"""

import hashlib
import json
import os
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
EVID = os.path.dirname(_HERE)
LEDGER = os.path.join(EVID, 'Stage7r_authorization_ledger.json')

REQUIRED = ('authorization_id', 'attempt_id', 'seed', 'reason',
            'same_seed_repeat_allowed', 'wrapper_change_allowed',
            'issued_at', 'consumed_at', 'status')
VALID_STATUS = ('ISSUED', 'CONSUMED', 'REVOKED', 'EXPIRED')


class AuthorizationError(Exception):
    """Fail-closed authorization error."""


def _sha256_text(text):
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def load_ledger(path=LEDGER):
    if not os.path.isfile(path):
        return {'artifact': "Stage 7' per-attempt authorization ledger",
                'authorizations': []}
    with open(path, 'r', encoding='utf-8') as fh:
        return json.load(fh)


def atomic_write(path, obj):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        json.dump(obj, fh, indent=2, sort_keys=True, allow_nan=False)
    os.replace(tmp, path)


def validate_entry(entry):
    missing = [k for k in REQUIRED if k not in entry]
    if missing:
        raise AuthorizationError('authorization_missing_fields: %s' % sorted(missing))
    if entry['status'] not in VALID_STATUS:
        raise AuthorizationError('authorization_bad_status: %r' % entry['status'])
    if not isinstance(entry['same_seed_repeat_allowed'], bool):
        raise AuthorizationError('same_seed_repeat_allowed must be bool')
    if not isinstance(entry['wrapper_change_allowed'], bool):
        raise AuthorizationError('wrapper_change_allowed must be bool')
    return True


def match_unconsumed(ledger, attempt_id, seed):
    hits = [a for a in ledger.get('authorizations', [])
            if a.get('attempt_id') == attempt_id
            and a.get('seed') == seed
            and a.get('status') == 'ISSUED'
            and not a.get('consumed_at')]
    return hits


def authorize_attempt(attempt_id, seed, path=LEDGER):
    """Return (ok, info).  Does NOT consume; caller must consume atomically."""
    ledger = load_ledger(path)
    hits = match_unconsumed(ledger, attempt_id, seed)
    if len(hits) != 1:
        return False, {'reason': 'no_single_unconsumed_authorization_for_attempt',
                       'attempt_id': attempt_id, 'seed': seed,
                       'n_matches': len(hits)}
    try:
        validate_entry(hits[0])
    except AuthorizationError as exc:
        return False, {'reason': str(exc)}
    return True, hits[0]


_SAME_SEED_REASONS = {'recovery_after_environment_failure',
                      'recovery_after_integrity_failure', 'repeat_same_seed'}


def authorize_with_reason(attempt_id, seed, requested_reason, path=LEDGER,
                          require_wrapper_change=False):
    """Strict (attempt_id, seed, reason) authorization.

    F3 repair: ``requested_reason`` must equal the authorization entry's reason
    exactly.  There is no ``initial_episode`` fallback, so a recovery
    authorization can never be consumed under an ``initial_episode`` request.
    Same-seed recovery requires explicit ``same_seed_repeat_allowed``; a wrapper
    change requires explicit ``wrapper_change_allowed``.
    """
    if not isinstance(requested_reason, str) or not requested_reason:
        return False, {'reason': 'requested_reason_missing'}
    ok, info = authorize_attempt(attempt_id, seed, path)
    if not ok:
        return False, info
    if requested_reason != info.get('reason'):
        return False, {'reason': 'reason_mismatch', 'requested': requested_reason,
                       'authorized': info.get('reason'),
                       'authorization_id': info.get('authorization_id')}
    if requested_reason in _SAME_SEED_REASONS and not info.get('same_seed_repeat_allowed'):
        return False, {'reason': 'same_seed_repeat_not_allowed',
                       'authorization_id': info.get('authorization_id')}
    if require_wrapper_change and not info.get('wrapper_change_allowed'):
        return False, {'reason': 'wrapper_change_not_allowed',
                       'authorization_id': info.get('authorization_id')}
    return True, info


def consume_attempt(attempt_id, seed, path=LEDGER):
    """Atomically consume the matching unconsumed authorization, then return it."""
    ok, info = authorize_attempt(attempt_id, seed, path)
    if not ok:
        raise AuthorizationError(info['reason'])
    ledger = load_ledger(path)
    for a in ledger['authorizations']:
        if (a['authorization_id'] == info['authorization_id']
                and a['status'] == 'ISSUED' and not a.get('consumed_at')):
            a['consumed_at'] = time.strftime('%Y-%m-%dT%H:%M:%S%z')
            a['status'] = 'CONSUMED'
            break
    atomic_write(path, ledger)
    return info


def issue(attempt_id, seed, reason, same_seed_repeat_allowed=False,
          wrapper_change_allowed=False, authorization_id=None, path=LEDGER):
    ledger = load_ledger(path)
    n = len(ledger.get('authorizations', [])) + 1
    entry = {
        'authorization_id': authorization_id or ('auth-%03d' % n),
        'attempt_id': attempt_id, 'seed': seed, 'reason': reason,
        'same_seed_repeat_allowed': bool(same_seed_repeat_allowed),
        'wrapper_change_allowed': bool(wrapper_change_allowed),
        'issued_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'consumed_at': None, 'status': 'ISSUED',
    }
    validate_entry(entry)
    ledger.setdefault('authorizations', []).append(entry)
    ledger['updated_at'] = entry['issued_at']
    atomic_write(path, ledger)
    return entry


def self_test():
    import tempfile
    checks = {}
    fd, tmp = tempfile.mkstemp(suffix='.json')
    os.close(fd)
    os.unlink(tmp)
    try:
        checks['no_ledger_fails_closed'] = authorize_attempt(1, 909101, tmp)[0] is False
        issue(1, 909101, 'initial_episode', path=tmp)
        ok, info = authorize_attempt(1, 909101, tmp)
        checks['issued_authorizes'] = ok is True
        checks['wrong_seed_refused'] = authorize_attempt(1, 909102, tmp)[0] is False
        checks['wrong_attempt_refused'] = authorize_attempt(2, 909101, tmp)[0] is False
        consume_attempt(1, 909101, tmp)
        checks['double_consume_refused'] = authorize_attempt(1, 909101, tmp)[0] is False
        # recovery authority must carry the flag
        issue(2, 909101, 'recovery_after_integrity_failure',
              same_seed_repeat_allowed=True, wrapper_change_allowed=True, path=tmp)
        ok, info = authorize_attempt(2, 909101, tmp)
        checks['recovery_authorized'] = ok and info['same_seed_repeat_allowed'] is True
        checks['top_level_flag_not_used'] = 'execution_authorization' not in info
    finally:
        if os.path.isfile(tmp):
            os.unlink(tmp)
    return checks


def reason_self_test():
    """F3 tests: strict reason matching and no initial_episode bypass."""
    import tempfile
    checks = {}
    fd, tmp = tempfile.mkstemp(suffix='.json')
    os.close(fd)
    os.unlink(tmp)
    try:
        # 1) auth reason=initial_episode, request initial_episode -> pass
        issue(1, 909101, 'initial_episode', path=tmp)
        checks['1_initial_episode_match_passes'] = \
            authorize_with_reason(1, 909101, 'initial_episode', path=tmp)[0] is True
        # 2) auth reason=recovery_after_integrity_failure, request initial_episode -> reject
        issue(2, 909101, 'recovery_after_integrity_failure',
              same_seed_repeat_allowed=True, path=tmp)
        ok, info = authorize_with_reason(2, 909101, 'initial_episode', path=tmp)
        checks['2_recovery_auth_cannot_take_initial_request'] = (
            ok is False and info['reason'] == 'reason_mismatch')
        # 3) auth reason=recovery_after_environment_failure, request initial_episode -> reject
        issue(3, 909101, 'recovery_after_environment_failure',
              same_seed_repeat_allowed=True, path=tmp)
        checks['3_env_recovery_auth_cannot_take_initial_request'] = \
            authorize_with_reason(3, 909101, 'initial_episode', path=tmp)[0] is False
        # 4) requested != authorized reason -> reject
        issue(4, 909101, 'recovery_after_integrity_failure',
              same_seed_repeat_allowed=True, path=tmp)
        checks['4_reason_mismatch_rejected'] = \
            authorize_with_reason(4, 909101, 'valid_protocol_episode', path=tmp)[0] is False
        # 5) same-seed repeat not explicitly authorized -> reject
        issue(5, 909101, 'recovery_after_integrity_failure', same_seed_repeat_allowed=False, path=tmp)
        ok, info = authorize_with_reason(5, 909101, 'recovery_after_integrity_failure', path=tmp)
        checks['5_same_seed_not_authorized_rejected'] = (
            ok is False and info['reason'] == 'same_seed_repeat_not_allowed')
        # 6) wrapper change not authorized -> reject
        issue(6, 909101, 'initial_episode', wrapper_change_allowed=False, path=tmp)
        ok, info = authorize_with_reason(6, 909101, 'initial_episode',
                                         require_wrapper_change=True, path=tmp)
        checks['6_wrapper_change_not_authorized_rejected'] = (
            ok is False and info['reason'] == 'wrapper_change_not_allowed')
        # 7) attempt_id/seed mismatch -> reject
        issue(7, 909102, 'initial_episode', path=tmp)
        checks['7_attempt_seed_mismatch_rejected'] = (
            authorize_with_reason(7, 909101, 'initial_episode', path=tmp)[0] is False
            and authorize_with_reason(8, 909102, 'initial_episode', path=tmp)[0] is False)
        # 8) consumed then reused -> reject
        issue(9, 909103, 'initial_episode', path=tmp)
        consume_attempt(9, 909103, tmp)
        checks['8_consumed_reuse_rejected'] = \
            authorize_with_reason(9, 909103, 'initial_episode', path=tmp)[0] is False
        # 9) recovery cannot borrow initial-episode authorization
        issue(10, 909104, 'initial_episode', path=tmp)
        ok, info = authorize_with_reason(10, 909104, 'recovery_after_integrity_failure', path=tmp)
        checks['9_recovery_cannot_borrow_initial_auth'] = (
            ok is False and info['reason'] == 'reason_mismatch')
    finally:
        if os.path.isfile(tmp):
            os.unlink(tmp)
    return checks


if __name__ == '__main__':
    import sys
    if '--reason-self-test' in sys.argv:
        st = reason_self_test()
        print(json.dumps({'reason_self_test': st,
                          'n_pass': sum(1 for v in st.values() if v),
                          'n_total': len(st)}, indent=2, sort_keys=True))
    elif '--self-test' in sys.argv:
        st = self_test()
        st.update(reason_self_test())
        print(json.dumps({'self_test': st,
                          'n_pass': sum(1 for v in st.values() if v),
                          'n_total': len(st)}, indent=2, sort_keys=True))
    else:
        raise SystemExit('use --self-test | --reason-self-test')
