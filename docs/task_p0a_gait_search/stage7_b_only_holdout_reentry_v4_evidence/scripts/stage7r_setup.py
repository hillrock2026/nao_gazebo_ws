#!/usr/bin/env python3
"""Stage 7' — B-only frozen configuration generator (offline only).

Same frozen baseline and fail-closed rules as the Stage 7 corrected B config:
``explicit_support_swing``, ``step_frequency_hz=0.8``, ``sway_sign=-1``.
Never starts Gazebo/ROS.
"""

import copy
import hashlib
import json
import os
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
EVID = os.path.dirname(_HERE)
WS = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(_HERE))))
_TOOLS = os.path.join(WS, 'tools')
if _TOOLS not in os.sys.path:
    os.sys.path.insert(0, _TOOLS)

CANONICAL = os.path.join(WS, 'docs', 'task_p0a_gait_search', 'p1bd_evidence',
                         'P1BD_config_snapshot.json')
REQUIRED_FROZEN_OVERRIDES = {'step_frequency_hz': 0.8, 'sway_sign': -1}
REQUIRED_PHASE_FIELDS = ('step_frequency_hz', 'sway_sign', 'gait_phase_mode')
B_MODE = 'explicit_support_swing'
ALLOWED_B_DIFF = {'gait_phase_mode'}


class ConfigFreezeError(Exception):
    pass


def atomic_json(path, obj):
    tmp = path + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as fh:
        json.dump(obj, fh, indent=2, sort_keys=True, allow_nan=False)
    os.replace(tmp, path)


def require_fields(cfg, fields=REQUIRED_PHASE_FIELDS):
    missing = [f for f in fields if f not in cfg or cfg[f] is None]
    if missing:
        raise ConfigFreezeError('configuration_field_missing: %s' % sorted(missing))


def load_canonical(path=CANONICAL):
    if not os.path.isfile(path):
        raise ConfigFreezeError('canonical baseline missing: %s' % path)
    snap = json.load(open(path, encoding='utf-8'))
    overrides = dict(snap.get('overrides') or {})
    expected = dict(snap.get('expected_config') or {})
    for k, v in REQUIRED_FROZEN_OVERRIDES.items():
        if overrides.get(k) != v:
            raise ConfigFreezeError('canonical override %s = %r, expected %r'
                                    % (k, overrides.get(k), v))
    require_fields(expected)
    for k, v in REQUIRED_FROZEN_OVERRIDES.items():
        if expected.get(k) != v:
            raise ConfigFreezeError('canonical expected_config %s = %r, expected %r'
                                    % (k, expected.get(k), v))
    return expected, overrides


def build_b(canonical_expected):
    b = copy.deepcopy(canonical_expected)
    b['gait_phase_mode'] = B_MODE
    require_fields(b)
    if b['step_frequency_hz'] != 0.8 or b['sway_sign'] != -1:
        raise ConfigFreezeError('B overrides not frozen')
    if b['gait_phase_mode'] != B_MODE:
        raise ConfigFreezeError('B gait_phase_mode != explicit_support_swing')
    return b


def check_b_vs_canonical(b, canonical_expected):
    d = [{'field': k, 'B': b.get(k), 'canonical': canonical_expected.get(k)}
         for k in sorted(set(b) | set(canonical_expected))
         if b.get(k) != canonical_expected.get(k)]
    extra = [x for x in d if x['field'] not in ALLOWED_B_DIFF]
    if extra:
        raise ConfigFreezeError('B differs from canonical beyond phase mode: %s' % extra)
    return d


def launch_arg_map(cfg):
    import run_p0a_gait_candidate as R
    return dict(a.split(':=', 1) for a in R.param_args(cfg))


def check_launch_consistency(cfg, label):
    m = launch_arg_map(cfg)
    if m.get('phase4b_step_frequency_hz') != str(cfg['step_frequency_hz']):
        raise ConfigFreezeError('%s launch step_frequency mismatch' % label)
    if m.get('phase4b_sway_sign') != str(cfg['sway_sign']):
        raise ConfigFreezeError('%s launch sway_sign mismatch' % label)
    if m.get('phase4b_gait_phase_mode') != cfg['gait_phase_mode']:
        raise ConfigFreezeError('%s launch gait_phase_mode mismatch' % label)
    return m


def self_test():
    r = {}
    try:
        require_fields({'step_frequency_hz': 0.8, 'gait_phase_mode': B_MODE})
        r['missing_sway_sign_raises'] = False
    except ConfigFreezeError as exc:
        r['missing_sway_sign_raises'] = 'configuration_field_missing' in str(exc)
    canonical, _ = load_canonical()
    b = build_b(canonical)
    r['built_sway_sign'] = b['sway_sign']
    r['built_step_frequency_hz'] = b['step_frequency_hz']
    r['built_gait_phase_mode'] = b['gait_phase_mode']
    check_launch_consistency(b, 'B')
    r['launch_consistency'] = True
    return r


def main():
    created = time.strftime('%Y-%m-%dT%H:%M:%S%z')
    canonical_expected, _ = load_canonical()
    b = build_b(canonical_expected)
    d_canon = check_b_vs_canonical(b, canonical_expected)
    launch_b = check_launch_consistency(b, 'B')
    st = self_test()
    snap = {
        'artifact': "Stage 7' B-only holdout frozen config snapshot",
        'created_at_local': created,
        'canonical_source': os.path.relpath(CANONICAL, WS),
        'canonical_source_sha256': hashlib.sha256(open(CANONICAL, 'rb').read()).hexdigest(),
        'expected_config': b,
        'overrides': {'gait_phase_mode': B_MODE, 'step_frequency_hz': 0.8, 'sway_sign': -1},
        'note': ('fwdf08 complete fixed config; step_frequency_hz=0.8, sway_sign=-1, '
                 'gait_phase_mode=explicit_support_swing. No producer default. B-only.'),
    }
    diff = {
        'artifact': "Stage 7' B-only config diff audit",
        'created_at_local': created,
        'canonical_source': os.path.relpath(CANONICAL, WS),
        'B_vs_canonical': {'allowed_fields': sorted(ALLOWED_B_DIFF), 'diff': d_canon,
                           'pass': all(x['field'] in ALLOWED_B_DIFF for x in d_canon)},
        'launch_consistency': {'B': {'pass': True,
                                     'step_frequency_hz': launch_b.get('phase4b_step_frequency_hz'),
                                     'sway_sign': launch_b.get('phase4b_sway_sign'),
                                     'gait_phase_mode': launch_b.get('phase4b_gait_phase_mode')}},
        'fail_closed_self_test': st,
    }
    atomic_json(os.path.join(EVID, 'Stage7r_B_config_snapshot.json'), snap)
    atomic_json(os.path.join(EVID, 'Stage7r_config_diff.json'), diff)
    print(json.dumps({'B_vs_canonical_pass': diff['B_vs_canonical']['pass'],
                      'B': {'step_frequency_hz': b['step_frequency_hz'],
                            'sway_sign': b['sway_sign'],
                            'gait_phase_mode': b['gait_phase_mode']},
                      'self_test': st}, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
