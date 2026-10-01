#!/usr/bin/env python3
"""Gate B3.0 offline fixtures (no ROS node, no Gazebo).

Covers:
  * F-4 : narrow-exception / structured-failure behaviour of phase4e_metrics;
  * F-5 : independent FK tool error paths, frozen tolerances, output-dir safety;
  * F-7 : config consistency for the explicit foot_pitch_swing_rad field;
  * the new body_net_forward_m gate (PASS / ==0.08 / <0.08 / missing /
    non-finite / sample-shortfall).

Expected values are simple, hand-computable fixture values; the production
analysis functions are never used to generate an expected value.

Usage: python3.12 tools/test_task4_phase4e_b3_offline.py
Exit 0 = all tests passed.
"""

import json
import math
import os
import shutil
import sys
import tempfile

_TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(_TOOLS)
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import phase4e_metrics as M  # noqa: E402
import phase4e_analysis as A  # noqa: E402
import phase4e_b3_gates as G  # noqa: E402
import phase4e_pathb_independent_fk_recompute as FK  # noqa: E402

RESULTS = []
TMP = os.path.join(WS, '.b3_0_test_tmp')


def check(name, cond, detail=None):
    RESULTS.append({'name': name, 'pass': bool(cond), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                           '' if cond else ' :: %s' % (detail,)))


def _write_jsonl(path, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as fh:
        for r in rows:
            fh.write(json.dumps(r) + '\n')


def _write_json(path, obj):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as fh:
        json.dump(obj, fh, indent=2, sort_keys=True)


def const_state(times, pos=None):
    pos = pos if pos is not None else [0.0] * 24
    return [{'t': t, 'position': list(pos)} for t in times]


def const_body(times, x=0.0, y=0.0, z=-10.0):
    return [{'t': t, 'x': x, 'y': y, 'z': z, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0,
             'qw': 1.0} for t in times]


def make_fk_run(root, rel, cfg, ready, states, bodies, stored,
                commands=None, under=None):
    rd = os.path.join(root, rel)
    _write_json(os.path.join(rd, 'run_config.json'), cfg)
    _write_json(os.path.join(rd, 'run_result.json'), {'ready_sim': ready})
    _write_json(os.path.join(rd, 'run_metrics.json'), stored)
    _write_jsonl(os.path.join(rd, 'trace', 'state_trace.jsonl'), states)
    _write_jsonl(os.path.join(rd, 'trace', 'body_pose_trace.jsonl'), bodies)
    if commands is not None:
        _write_jsonl(os.path.join(rd, 'trace', 'command_trace.jsonl'), commands)
    return rd


FK_CFG = {'start_delay_sec': 0.5, 'gait_blend_sec': 0.1, 'duration_sec': 1.0}
FK_STORED = {'left_swing_m': 0.0, 'right_swing_m': 0.0, 'support_drift_m': 0.0,
             'arm': {'leg_match_fraction': 0.0}}


# ------------------------------------------------------------------------ F-5
def test_fk_happy_path():
    times = [1.60 + 0.01 * i for i in range(90)]
    states = const_state(times)
    bodies = const_body(times)
    rd = make_fk_run(TMP + '/fk', 'runs/happy', FK_CFG, 1.0, states, bodies,
                     FK_STORED)
    chain = FK.parse_urdf()
    rec = FK.recompute_run('HAPPY', rd, chain)
    check('fk_happy_ok', rec['ok'] is True, rec.get('reason'))
    check('fk_happy_zero_swing',
          rec['value']['swing_l_m'] == 0.0 and rec['value']['swing_r_m'] == 0.0,
          rec['value'])
    check('fk_happy_zero_drift', rec['value']['support_drift_m'] == 0.0,
          rec['value'])
    check('fk_happy_zero_legmatch',
          rec['value']['arm_leg_match_fraction'] == 0.0, rec['value'])
    check('fk_happy_max_skew_recorded',
          rec['max_state_body_skew_sec'] is not None
          and rec['max_state_body_skew_sec'] <= 0.02,
          rec['max_state_body_skew_sec'])
    check('fk_happy_all_comparisons_pass',
          all(c['pass'] for c in rec['comparisons'].values()))


def test_fk_errors():
    chain = FK.parse_urdf()
    times = [1.60 + 0.01 * i for i in range(60)]

    # missing state trace
    rd = make_fk_run(TMP + '/fk', 'runs/missing_state', FK_CFG, 1.0,
                     const_state(times), const_body(times), FK_STORED)
    os.remove(os.path.join(rd, 'trace', 'state_trace.jsonl'))
    rec = FK.recompute_run('X', rd, chain)
    check('fk_missing_state', rec['reason'] in ('trace_or_config_missing',
                                                'missing_state_trace'), rec['reason'])

    # missing body trace
    rd = make_fk_run(TMP + '/fk', 'runs/missing_body', FK_CFG, 1.0,
                     const_state(times), const_body(times), FK_STORED)
    os.remove(os.path.join(rd, 'trace', 'body_pose_trace.jsonl'))
    rec = FK.recompute_run('X', rd, chain)
    check('fk_missing_body', rec['reason'] in ('trace_or_config_missing',
                                               'missing_body_trace'), rec['reason'])

    # empty traces
    rd = make_fk_run(TMP + '/fk', 'runs/empty_state', FK_CFG, 1.0, [],
                     const_body(times), FK_STORED)
    rec = FK.recompute_run('X', rd, chain)
    check('fk_empty_state', rec['reason'] == 'empty_state_trace'
          and rec['stage'] == 'load', rec['reason'])

    rd = make_fk_run(TMP + '/fk', 'runs/empty_body', FK_CFG, 1.0,
                     const_state(times), [], FK_STORED)
    rec = FK.recompute_run('X', rd, chain)
    check('fk_empty_body', rec['reason'] == 'empty_body_trace', rec['reason'])

    # state dimension error
    rd = make_fk_run(TMP + '/fk', 'runs/dim', FK_CFG, 1.0,
                     [{'t': t, 'position': [0.0] * 3} for t in times],
                     const_body(times), FK_STORED)
    rec = FK.recompute_run('X', rd, chain)
    check('fk_state_dim', rec['reason'] == 'state_dim', rec['reason'])

    # body skew exceeded (body 5 s away)
    rd = make_fk_run(TMP + '/fk', 'runs/skew', FK_CFG, 1.0, const_state(times),
                     [{'t': t + 5.0, 'x': 0.0, 'y': 0.0, 'z': -10.0, 'qx': 0.0,
                       'qy': 0.0, 'qz': 0.0, 'qw': 1.0} for t in times],
                     FK_STORED)
    rec = FK.recompute_run('X', rd, chain)
    check('fk_body_skew', rec['reason'] == 'body_time_skew_exceeded'
          and rec['stage'] == 'body_match', rec['reason'])

    # non-finite joint
    bad = const_state(times)
    bad[10]['position'][0] = float('nan')
    rd = make_fk_run(TMP + '/fk', 'runs/nan', FK_CFG, 1.0, bad,
                     const_body(times), FK_STORED)
    rec = FK.recompute_run('X', rd, chain)
    check('fk_nan', rec['reason'] == 'non_finite_input', rec['reason'])

    # grounded insufficient (body high -> feet above ground)
    rd = make_fk_run(TMP + '/fk', 'runs/noground', FK_CFG, 1.0,
                     const_state(times), const_body(times, z=10.0), FK_STORED)
    rec = FK.recompute_run('X', rd, chain)
    check('fk_grounded_insufficient',
          rec['reason'] == 'insufficient_grounded_samples', rec['reason'])

    # no valid support interval: 2 grounded samples far apart
    bodies = const_body(times, z=10.0)
    bodies[0]['z'] = -10.0
    bodies[-1]['z'] = -10.0
    rd = make_fk_run(TMP + '/fk', 'runs/nointerval', FK_CFG, 1.0,
                     const_state(times), bodies, FK_STORED)
    rec = FK.recompute_run('X', rd, chain)
    check('fk_no_valid_interval',
          rec['reason'] == 'no_valid_support_interval', rec['reason'])

    # reference missing
    stored = {'left_swing_m': 0.0, 'right_swing_m': 0.0,
              'support_drift_m': 0.0, 'arm': {}}
    rd = make_fk_run(TMP + '/fk', 'runs/noref', FK_CFG, 1.0,
                     const_state(times), const_body(times), stored)
    rec = FK.recompute_run('X', rd, chain)
    check('fk_reference_missing',
          rec['reason'] == 'comparison_out_of_tolerance'
          and not rec['comparisons']['arm_leg_match_fraction']['pass'],
          rec['reason'])

    # comparison beyond tolerance: reference offset by 0.01 m
    stored = dict(FK_STORED)
    stored['left_swing_m'] = 0.01
    rd = make_fk_run(TMP + '/fk', 'runs/outtol', FK_CFG, 1.0,
                     const_state(times), const_body(times), stored)
    rec = FK.recompute_run('X', rd, chain)
    check('fk_out_of_tolerance',
          rec['reason'] == 'comparison_out_of_tolerance'
          and rec['comparisons']['swing_l_m']['abs_diff'] == 0.01
          and not rec['comparisons']['swing_l_m']['pass'], rec['comparisons']['swing_l_m'])


def test_compute_support_drift_errors():
    base = FK.compute_support_drift([-1, -1, -1], [0, 0, 0], [0, 0.01, 0.02])
    check('drift_ok', base.get('ok') is True and base['value'] == 0.0, base)
    check('drift_dim_mismatch',
          FK.compute_support_drift([-1, -1], [0, 0], [0, 1, 2])['reason']
          == 'time_dim_mismatch')
    check('drift_non_finite',
          FK.compute_support_drift([-1, float('nan'), -1], [0, 0, 0], [0, 1, 2])['reason']
          == 'non_finite_input')
    check('drift_non_increasing',
          FK.compute_support_drift([-1, -1, -1], [0, 0, 0], [0, 2, 1])['reason']
          == 'time_not_increasing')
    check('drift_insufficient_grounded',
          FK.compute_support_drift([10, 10, 10], [0, 0, 0], [0, 1, 2])['reason']
          == 'insufficient_grounded_samples')


def test_fk_output_dir_safety():
    base = os.path.join(TMP, 'od')
    os.makedirs(base, exist_ok=True)
    inside = os.path.join(base, 'wsA')
    outside = os.path.join(base, 'wsB')
    os.makedirs(inside, exist_ok=True)
    os.makedirs(outside, exist_ok=True)

    _, err = FK.resolve_output_dir(os.path.join(outside, 'out'), False,
                                   workspace=inside)
    check('od_outside_workspace', FK.is_structured_error(err)
          and err['reason'] == 'output_outside_workspace', err)

    _, err = FK.resolve_output_dir(os.path.join(inside, '..', 'wsB', 'out'),
                                   False, workspace=inside)
    check('od_dotdot_escape', FK.is_structured_error(err), err)

    link = os.path.join(inside, 'link')
    if os.path.lexists(link):
        os.remove(link)
    os.symlink(outside, link)
    _, err = FK.resolve_output_dir(os.path.join(link, 'out'), False,
                                   workspace=inside)
    check('od_symlink_escape', FK.is_structured_error(err), err)

    hist = os.path.join(inside, 'historical')
    os.makedirs(hist, exist_ok=True)
    _, err = FK.resolve_output_dir(os.path.join(hist, 'out'), False,
                                   workspace=inside, forbidden_prefixes=(hist,))
    check('od_historical', FK.is_structured_error(err)
          and err['reason'] == 'output_forbidden_historical_dir', err)

    f = os.path.join(inside, 'afile')
    open(f, 'w').write('x')
    _, err = FK.resolve_output_dir(f, False, workspace=inside)
    check('od_is_file', FK.is_structured_error(err)
          and err['reason'] == 'output_path_is_file', err)

    nonempty = os.path.join(inside, 'nonempty')
    os.makedirs(nonempty, exist_ok=True)
    open(os.path.join(nonempty, 'keep.txt'), 'w').write('keep')
    _, err = FK.resolve_output_dir(nonempty, False, workspace=inside)
    check('od_nonempty_refused', FK.is_structured_error(err)
          and err['reason'] == 'output_dir_not_empty', err)
    got, err = FK.resolve_output_dir(nonempty, True, workspace=inside)
    check('od_nonempty_overwrite_ok', err is None and got == os.path.realpath(nonempty))

    ok, err = FK.resolve_output_dir(os.path.join(inside, 'fresh'), False,
                                    workspace=inside)
    check('od_fresh_ok', err is None, err)


def test_fk_main_exit_codes():
    runs_root = os.path.join(TMP, 'runs_root')
    times = [1.60 + 0.01 * i for i in range(90)]
    make_fk_run(runs_root, 'runs/fix_1', FK_CFG, 1.0, const_state(times),
                const_body(times), FK_STORED)
    os.makedirs(os.path.join(runs_root, 'charter'), exist_ok=True)
    _write_json(os.path.join(runs_root, 'charter', 'run_order.json'),
                {'order': ['FIX-1'], 'run_dir': {'FIX-1': 'runs/fix_1'}})
    out = os.path.join(TMP, 'main_out')
    rc = FK.main(['--runs-root', runs_root, '--output-dir', out])
    check('fk_main_happy_exit0', rc == 0, rc)
    txt = open(os.path.join(out, 'independent_fk_recompute.txt')).read()
    check('fk_main_pass_sentence',
          'PASS: all comparisons within frozen tolerances' in txt)

    # bad output dir -> exit 2
    rc = FK.main(['--runs-root', runs_root, '--output-dir', '/tmp/not_in_ws'])
    check('fk_main_bad_outdir_exit2', rc == 2, rc)

    # failing run -> exit 1
    make_fk_run(runs_root, 'runs/bad_1', FK_CFG, 1.0, const_state(times),
                const_body(times), dict(FK_STORED, left_swing_m=0.5))
    _write_json(os.path.join(runs_root, 'charter', 'run_order.json'),
                {'order': ['BAD-1'], 'run_dir': {'BAD-1': 'runs/bad_1'}})
    rc = FK.main(['--runs-root', runs_root,
                  '--output-dir', os.path.join(TMP, 'main_out2')])
    check('fk_main_fail_exit1', rc == 1, rc)


# ------------------------------------------------------------------------ F-4
def test_f4_leg_match_structured():
    rd = os.path.join(TMP, 'f4', 'legmatch')
    _write_jsonl(os.path.join(rd, 'state_trace.jsonl'), [123, 456])
    err = M._leg_match_fk(rd, {'start_delay_sec': 0.5, 'gait_blend_sec': 1.5,
                               'duration_sec': 18.0}, [0.0, 1.0], [0, 1],
                          [1.0, -1.0])
    check('f4_leg_match_structured',
          M.is_structured_error(err) and err['stage'] == 'leg_match_fk'
          and err['reason'] == 'leg_match_input_invalid', err)


def test_f4_command_limits_structured():
    original = M._PRODUCER_PATH
    try:
        M._PRODUCER_PATH = os.path.join(TMP, 'f4', 'does_not_exist.py')
        err = M._command_limits([{'position': [0.0] * 24}])
        check('f4_command_limits_unavailable',
              M.is_structured_error(err)
              and err['stage'] == 'command_limits'
              and err['reason'] == 'command_limit_reference_unavailable', err)
        bad = os.path.join(TMP, 'f4', 'bad_producer.py')
        os.makedirs(os.path.dirname(bad), exist_ok=True)
        open(bad, 'w').write('def (:\n')
        M._PRODUCER_PATH = bad
        err2 = M._command_limits([])
        check('f4_command_limits_syntax',
              M.is_structured_error(err2)
              and err2['reason'] == 'command_limit_reference_unavailable', err2)
    finally:
        M._PRODUCER_PATH = original


def _valid_metrics_run(root, rel):
    # run_metrics takes the directory that directly contains *_trace.jsonl.
    rd = os.path.join(root, rel)
    times = [0.02 * i for i in range(200)]
    _write_jsonl(os.path.join(rd, 'body_pose_trace.jsonl'),
                 [{'t': t, 'x': 0.001 * i, 'y': 0.0, 'z': 0.3, 'yaw': 0.0}
                  for i, t in enumerate(times)])
    _write_jsonl(os.path.join(rd, 'state_trace.jsonl'),
                 [{'t': t, 'position': [0.0] * 24} for t in times])
    _write_jsonl(os.path.join(rd, 'command_trace.jsonl'),
                 [{'t': t, 'ok': True, 'position': [0.0] * 24} for t in times])
    _write_jsonl(os.path.join(rd, 'contact_trace.jsonl'),
                 [{'t': 0.0, 'side': 'left', 'value': True}])
    _write_jsonl(os.path.join(rd, 'imu_trace.jsonl'),
                 [{'t': t, 'wx': 0.0, 'wy': 0.0, 'wz': 0.0, 'ax': 0.0,
                   'ay': 0.0, 'az': 9.8} for t in times])
    return rd


def test_f4_unknown_exception_propagates():
    rd = _valid_metrics_run(os.path.join(TMP, 'f4'), 'unknown')
    cfg = {'gait_method': 'walk', 'start_delay_sec': 0.5, 'gait_blend_sec': 1.5,
           'duration_sec': 3.0, 'stand_sec': 24.0}
    original = M.p4c.analyze

    def boom(*a, **k):
        raise RuntimeError('injected programming error')

    M.p4c.analyze = boom
    raised = False
    try:
        M.run_metrics(rd, cfg, 0.0, cfg, mode='walk')
    except RuntimeError:
        raised = True
    finally:
        M.p4c.analyze = original
    check('f4_unknown_exception_not_swallowed', raised)

    chain = FK.parse_urdf()
    original = M.p4c.load_run

    def boom2(*a, **k):
        raise RuntimeError('injected')

    M.p4c.load_run = boom2
    raised2 = False
    try:
        M._leg_match_fk(rd, cfg, [0.0, 1.0], [0, 1], [1.0, -1.0])
    except RuntimeError:
        raised2 = True
    finally:
        M.p4c.load_run = original
    check('f4_leg_match_unknown_exception_propagates', raised2)


def test_f4_gate_fails_on_structured_failure():
    rd = os.path.join(TMP, 'f4', 'analyze_fail')
    times = [0.01 * i for i in range(10)]
    _write_jsonl(os.path.join(rd, 'body_pose_trace.jsonl'),
                 [{'t': t, 'x': 0.0, 'y': 0.0, 'z': 0.3, 'yaw': 0.0}
                  for t in times])
    _write_jsonl(os.path.join(rd, 'state_trace.jsonl'),
                 [{'t': t, 'position': [0.0] * 3} for t in times[:5]])
    _write_jsonl(os.path.join(rd, 'command_trace.jsonl'),
                 [{'t': 0.0, 'ok': True, 'position': [0.0] * 24}])
    _write_jsonl(os.path.join(rd, 'contact_trace.jsonl'), [])
    _write_jsonl(os.path.join(rd, 'imu_trace.jsonl'), [])
    cfg = {'gait_method': 'walk', 'start_delay_sec': 0.0, 'gait_blend_sec': 0.0,
           'duration_sec': 1.0, 'stand_sec': 24.0}
    m = M.run_metrics(rd, cfg, 0.0, cfg, mode='walk')
    check('f4_analyze_structured', M.is_structured_error(m.get('phase4c_status')),
          m.get('phase4c_status'))
    gates = M.evaluate_gates(m, {'hard_gates': {}}, 'NATURAL-SWING')
    check('f4_roll_gate_fails', gates['roll_le_15']['pass'] is False)
    check('f4_support_drift_gate_fails',
          gates['support_drift_le_0.03']['pass'] is False)


# ------------------------------------------------------------------------ F-7
def test_f7_config_consistency():
    anchor = {'foot_pitch_swing_rad': 0.10, 'step_length_m': 0.02,
              'step_frequency_hz': 0.5, 'leg_extension_m': 0.24401,
              'sway_amplitude_rad': 0.06, 'arm_swing_rad': 0.12,
              'arm_swing_sign': 1}
    check('f7_exact_pass', A.config_consistency(dict(anchor), anchor) == [])
    check('f7_missing_fail',
          any(d['error'] == 'missing_in_run_config'
              for d in A.config_consistency({k: v for k, v in anchor.items()
                                             if k != 'foot_pitch_swing_rad'}, anchor)))
    check('f7_extra_fail',
          any(d['error'] == 'extra_unfrozen_key'
              for d in A.config_consistency({**anchor, 'extra': 1}, anchor)))
    check('f7_wrong_value_fail',
          any(d['error'] == 'value_mismatch'
              for d in A.config_consistency({**anchor, 'foot_pitch_swing_rad': 0.2}, anchor)))
    check('f7_bool_as_number_fail',
          any(d['error'] == 'type_mismatch'
              for d in A.config_consistency({**anchor, 'foot_pitch_swing_rad': True}, anchor)))
    check('f7_nan_fail',
          any(d['error'] == 'non_finite_or_type'
              for d in A.config_consistency({**anchor, 'foot_pitch_swing_rad': float('nan')}, anchor)))
    check('f7_inf_fail',
          any(d['error'] == 'non_finite_or_type'
              for d in A.config_consistency({**anchor, 'foot_pitch_swing_rad': float('inf')}, anchor)))


# ----------------------------------------------------------- body_net_forward
def test_body_net_forward():
    good = [{'t': 0.0, 'x': 0.0}, {'t': 1.0, 'x': 0.09}]
    r = G.body_net_forward(good)
    check('bnf_pass', r.get('ok') and abs(r['value'] - 0.09) < 1e-12, r)
    boundary = [{'t': 0.0, 'x': 0.0}, {'t': 1.0, 'x': 0.08}]
    res = G.body_net_forward(boundary)
    check('bnf_boundary_0.08', res['value'] == 0.08)
    low = [{'t': 0.0, 'x': 0.0}, {'t': 1.0, 'x': 0.079}]
    check('bnf_low_value', G.body_net_forward(low)['value'] < 0.08)
    check('bnf_missing_x',
          M.is_structured_error(G.body_net_forward([{'t': 0.0}, {'t': 1.0, 'x': 0.1}])))
    check('bnf_nonfinite',
          M.is_structured_error(G.body_net_forward(
              [{'t': 0.0, 'x': 0.0}, {'t': 1.0, 'x': float('nan')}])))
    check('bnf_insufficient',
          M.is_structured_error(G.body_net_forward([{'t': 0.0, 'x': 0.0}])))

    # gate wiring: pass/fail
    rd = os.path.join(TMP, 'bnf', 'run')
    times = [0.02 * i for i in range(200)]
    _write_jsonl(os.path.join(rd, 'trace', 'body_pose_trace.jsonl'),
                 [{'t': t, 'x': 0.001 * i, 'y': 0.0, 'z': 0.3, 'yaw': 0.0}
                  for i, t in enumerate(times)])
    _write_jsonl(os.path.join(rd, 'trace', 'state_trace.jsonl'),
                 [{'t': t, 'position': [0.0] * 24} for t in times])
    _write_jsonl(os.path.join(rd, 'trace', 'command_trace.jsonl'),
                 [{'t': t, 'ok': True, 'position': [0.0] * 24} for t in times])
    _write_jsonl(os.path.join(rd, 'trace', 'contact_trace.jsonl'), [])
    _write_jsonl(os.path.join(rd, 'trace', 'imu_trace.jsonl'), [])
    cfg = {'gait_method': 'walk', 'start_delay_sec': 0.5, 'gait_blend_sec': 1.5,
           'duration_sec': 3.0, 'stand_sec': 24.0}
    m = M.run_metrics(rd, cfg, 0.0, cfg, mode='walk')
    gates, bf = G.evaluate_b3_gates(m, os.path.join(rd, 'trace'),
                                    {'hard_gates': {}}, 'NATURAL-SWING')
    check('bnf_gate_pass', gates['body_net_forward_ge_0.08']['pass'] is True,
          gates['body_net_forward_ge_0.08'])

    rd2 = os.path.join(TMP, 'bnf', 'run_short')
    _write_jsonl(os.path.join(rd2, 'trace', 'body_pose_trace.jsonl'),
                 [{'t': 0.0, 'x': 0.1, 'y': 0.0, 'z': 0.3, 'yaw': 0.0}])
    gates2, _ = G.evaluate_b3_gates(m, os.path.join(rd2, 'trace'),
                                    {'hard_gates': {}}, 'NATURAL-SWING')
    check('bnf_gate_short_fails', gates2['body_net_forward_ge_0.08']['pass'] is False)


def main():
    if os.path.isdir(TMP):
        shutil.rmtree(TMP)
    print('Gate B3.0 offline fixtures')
    test_fk_happy_path()
    test_fk_errors()
    test_compute_support_drift_errors()
    test_fk_output_dir_safety()
    test_fk_main_exit_codes()
    test_f4_leg_match_structured()
    test_f4_command_limits_structured()
    test_f4_unknown_exception_propagates()
    test_f4_gate_fails_on_structured_failure()
    test_f7_config_consistency()
    test_body_net_forward()
    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\nb3_0 offline fixtures: %d/%d passed' % (passed, total))
    shutil.rmtree(TMP, ignore_errors=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
