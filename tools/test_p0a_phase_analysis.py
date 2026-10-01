#!/usr/bin/env python3
"""P0-a P1-R offline phase-analysis fixtures (offline, no Gazebo).

Usage: python3 tools/test_p0a_phase_analysis.py [--json-out F]
"""

import argparse
import json
import math
import os
import shutil
import sys
import tempfile

_TOOLS = os.path.dirname(os.path.abspath(__file__))
_SCRIPTS = os.path.join(os.path.dirname(_TOOLS), 'src', 'nao_gz_sim', 'scripts')
for _p in (_TOOLS, _SCRIPTS):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import p0a_phase_analysis as PA  # noqa: E402
import nao_gait_phase as G  # noqa: E402

RESULTS = []
CFG = {'start_delay_sec': 0.5, 'gait_blend_sec': 1.5, 'duration_sec': 8.0,
       'step_frequency_hz': 0.8, 'double_support_ratio': 0.3,
       'gait_phase_mode': 'explicit_support_swing'}
# P1-R frozen gates v2 (hard).
GATES = {'min_forward': 0.08, 'max_lateral_ratio': 0.5, 'max_yaw_deg': 30.0,
         'max_tilt_deg': 35.0, 'min_duration': 8.0, 'max_missing': 1.0,
         'max_path_over_net': 8.0, 'min_contact_alt': 3,
         'min_support_ground': 0.80, 'min_ds_both_ground': 0.50,
         'min_airborne': 0.40, 'min_clearance': 0.002,
         'max_support_drift_m': 0.010, 'epsilon_sec': 1e-6}


def check(name, cond, detail=None):
    RESULTS.append({'name': name, 'pass': bool(cond), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                           '' if cond else ' :: %r' % (detail,)))


def make_run(root, name, cfg=None, body_z=0.33, contact_mode='normal',
             drop_state=False, nan_joint=False, nan_body=False,
             state_names=False, t0=2.0, t1=8.5, n_states=300,
             recorded_mode=None, omit_mode=False, names_mode=None,
             legacy_key=True):
    run_dir = os.path.join(root, name)
    trace = os.path.join(run_dir, 'trace')
    os.makedirs(trace)
    cfg = dict(cfg or CFG)
    if recorded_mode is None:
        recorded_mode = cfg.get('gait_phase_mode', 'legacy')
    if omit_mode:
        cfg.pop('gait_phase_mode', None)
        if legacy_key:
            cfg['gait_method'] = 'walk'
        else:
            cfg.pop('gait_method', None)
    else:
        cfg['gait_phase_mode'] = recorded_mode
    if names_mode is None:
        names_mode = 'canonical' if state_names else 'absent'
    with open(os.path.join(run_dir, 'run_config.json'), 'w') as fh:
        json.dump(cfg, fh)
    with open(os.path.join(run_dir, 'result.json'), 'w') as fh:
        json.dump({'labels': {'ready_sim': 0.0}}, fh)
    freq = cfg['step_frequency_hz']
    if not drop_state:
        with open(os.path.join(trace, 'state_trace.jsonl'), 'w') as fh:
            for i in range(n_states):
                t = t0 + i * (t1 - t0) / (n_states - 1)
                pos = [0.0] * 24
                if nan_joint and i == 10:
                    pos[3] = float('nan')
                row = {'t': t, 'position': pos}
                if names_mode == 'canonical':
                    row['name'] = list(PA.JOINT_ORDER)
                elif names_mode == 'reversed':
                    row['name'] = list(reversed(PA.JOINT_ORDER))
                elif names_mode == 'empty':
                    row['name'] = []
                elif names_mode == 'short':
                    row['name'] = list(PA.JOINT_ORDER[:23])
                fh.write(json.dumps(row) + '\n')

    with open(os.path.join(trace, 'body_pose_trace.jsonl'), 'w') as fh:
        for i in range(n_states):
            t = t0 + i * (t1 - t0) / (n_states - 1)
            z = float('nan') if nan_body else body_z
            fh.write(json.dumps({'t': t, 'x': 0.01 * t, 'y': 0.0, 'z': z,
                                 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                                 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}) + '\n')
    if contact_mode == 'missing':
        pass
    else:
        with open(os.path.join(trace, 'contact_trace.jsonl'), 'w') as fh:
            if contact_mode != 'empty':
                for i in range(n_states):
                    t = t0 + i * (t1 - t0) / (n_states - 1)
                    if contact_mode == 'nonmono' and i == 50:
                        t = t0  # backwards in time
                    ph = ((t - 0.5) * freq) % 1.0
                    nm = G.classify(ph, cfg['double_support_ratio'])[0]
                    lv = nm == 'LEFT_SWING'
                    side_l, side_r = 'left', 'right'
                    if contact_mode == 'badside':
                        side_l = 'LEFT'
                    val_l = bool(lv)
                    if contact_mode == 'badvalue':
                        val_l = 1
                    fh.write(json.dumps({'t': t, 'side': side_l, 'value': val_l}) + '\n')
                    fh.write(json.dumps({'t': t, 'side': side_r, 'value': bool(not lv)}) + '\n')
                    if contact_mode == 'conflict' and i == 0:
                        fh.write(json.dumps({'t': t, 'side': side_l,
                                             'value': bool(not val_l)}) + '\n')
    return run_dir


def test_phase_stats():
    tmp = tempfile.mkdtemp(prefix='p1rpa_')
    try:
        r = PA.analyse(make_run(tmp, 'run'))
        check('phase_measurable', r['measurable'] is True, r.get('measurable'))
        check('phase_mode_from_config',
              r['phase_mode_analysis'] == 'explicit_support_swing',
              r['phase_mode_analysis'])
        fr = {k: v['fraction'] for k, v in r['phase_stats'].items()}
        check('phase_fraction_left_swing', abs(fr['LEFT_SWING'] - 0.35) < 0.05, fr)
        check('phase_fraction_right_swing', abs(fr['RIGHT_SWING'] - 0.35) < 0.05, fr)
        check('phase_fraction_ds_half',
              abs(fr['DS_TO_LEFT_SUPPORT'] - 0.15) < 0.05
              and abs(fr['DS_TO_RIGHT_SUPPORT'] - 0.15) < 0.05, fr)
        check('phase_zero_names_empty', r['phase_zero_sample_names'] == [],
              r['phase_zero_sample_names'])
        check('onset_present',
              isinstance(r['swing_to_support_onset_delay_sec'], dict)
              and r['swing_to_support_onset_delay_sec']['n_boundaries'] >= 1,
              r['swing_to_support_onset_delay_sec'])
        check('m0_diagnostic_present',
              isinstance(r['m0_legacy_contact_delay_sec'], dict))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_contact_and_mode_failures():
    cases = [
        ('missing_contact', dict(contact_mode='missing')),
        ('empty_contact', dict(contact_mode='empty')),
        ('conflict_contact', dict(contact_mode='conflict')),
        ('badside_contact', dict(contact_mode='badside')),
        ('badvalue_contact', dict(contact_mode='badvalue')),
        ('no_state', dict(drop_state=True)),
        ('nan_joint', dict(nan_joint=True)),
        ('nan_body', dict(nan_body=True)),
    ]
    for name, kw in cases:
        tmp = tempfile.mkdtemp(prefix='p1rpa_f_')
        try:
            rd = make_run(tmp, 'run', **kw)
            try:
                PA.analyse(rd)
                check('fail_%s' % name, False)
            except PA.PhaseAnalysisError:
                check('fail_%s' % name, True)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)
    # missing run_config
    tmp = tempfile.mkdtemp(prefix='p1rpa_m_')
    try:
        try:
            PA.analyse(os.path.join(tmp, 'nope'))
            check('fail_missing_run_config', False)
        except PA.PhaseAnalysisError:
            check('fail_missing_run_config', True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    # mode mismatch (recorded legacy, requested explicit)
    tmp = tempfile.mkdtemp(prefix='p1rpa_mm_')
    try:
        rd = make_run(tmp, 'run', recorded_mode='legacy')
        try:
            PA.analyse(rd, phase_mode='explicit_support_swing')
            check('fail_mode_mismatch', False)
        except PA.PhaseAnalysisError:
            check('fail_mode_mismatch', True)
        # default (None) uses recorded legacy -> ok
        r = PA.analyse(rd)
        check('mode_default_uses_recorded', r['phase_mode_analysis'] == 'legacy')
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    # state joint names wrong order
    tmp = tempfile.mkdtemp(prefix='p1rpa_j_')
    try:
        rd = make_run(tmp, 'run', state_names=True)
        # corrupt the names in the file
        p = os.path.join(rd, 'trace', 'state_trace.jsonl')
        rows = [json.loads(l) for l in open(p) if l.strip()]
        rows[0]['name'] = list(reversed(PA.JOINT_ORDER))
        with open(p, 'w') as fh:
            for r in rows:
                fh.write(json.dumps(r) + '\n')
        try:
            PA.analyse(rd)
            check('fail_joint_order', False)
        except PA.PhaseAnalysisError:
            check('fail_joint_order', True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    # zero-sample phase -> not measurable
    tmp = tempfile.mkdtemp(prefix='p1rpa_z_')
    try:
        rd = make_run(tmp, 'run', t0=2.0, t1=2.6, n_states=80)
        r = PA.analyse(rd)
        check('zero_phase_not_measurable',
              r['measurable'] is False and len(r['phase_zero_sample_names']) >= 1,
              (r['measurable'], r['phase_zero_sample_names']))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def _contact_metrics(result):
    return {k: result[k] for k in (
        'ds_both_contact_rate', 'left_swing_right_support_contact_rate',
        'right_swing_left_support_contact_rate', 'zero_contact')}


def test_contact_ordering_and_duplicates():
    import random
    tmp = tempfile.mkdtemp(prefix='p1rpa_ord_')
    try:
        run = make_run(tmp, 'run')
        td = os.path.join(run, 'trace')
        path = os.path.join(td, 'contact_trace.jsonl')
        original = [json.loads(l) for l in open(path) if l.strip()]
        events, diag0 = PA.validate_contacts(td)
        check('ordering_input_already_canonical',
              diag0['input_already_canonical'] is True
              and diag0['n_input_order_violations'] == 0, diag0)
        base = PA.analyse(run)
        base_m = _contact_metrics(base)

        # exact duplicate + full shuffle must not change the contact metrics or
        # the canonical event sequence.
        shuffled = list(original) + [dict(original[3])]
        random.Random(1234).shuffle(shuffled)
        with open(path, 'w') as fh:
            for row in shuffled:
                fh.write(json.dumps(row) + '\n')
        events2, diag = PA.validate_contacts(td)
        check('exact_duplicate_removed',
              diag['n_exact_duplicates_removed'] == 1, diag)
        check('duplicate_not_silent', diag['n_unique_events'] == len(original),
              (diag['n_unique_events'], len(original)))
        check('shuffled_input_flagged',
              diag['input_already_canonical'] is False
              and diag['n_input_order_violations'] > 0, diag)
        check('global_sort_matches_original', events2 == events)
        check('sorted_by_global_time',
              all(a[0] <= b[0] for a, b in zip(events2, events2[1:])))
        check('contact_metrics_order_invariant',
              _contact_metrics(PA.analyse(run)) == base_m)
        check('_validated_contacts_backcompat',
              PA._validated_contacts(td) == events)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    # conflicting values at an identical time must fail explicitly
    tmp = tempfile.mkdtemp(prefix='p1rpa_conf_')
    try:
        run = make_run(tmp, 'run', contact_mode='conflict')
        try:
            PA.analyse(run)
            check('conflict_contact_fails', False)
        except PA.PhaseAnalysisError as exc:
            check('conflict_contact_fails', 'conflict' in str(exc), str(exc))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_mode_contract():
    # explicit mode
    tmp = tempfile.mkdtemp(prefix='p1rpa_mode_')
    try:
        r = PA.analyse(make_run(tmp, 'run'))
        check('mode_explicit_source', r['phase_mode_source'] == 'explicit')
        check('mode_not_assumed', r['phase_mode_assumed'] is False)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    # missing mode but legacy schema -> inferred legacy, explicitly recorded
    tmp = tempfile.mkdtemp(prefix='p1rpa_mode2_')
    try:
        rd = make_run(tmp, 'run', omit_mode=True)
        r = PA.analyse(rd)
        check('mode_missing_legacy_inferred',
              r['phase_mode_recorded'] == 'legacy'
              and r['phase_mode_source'] == 'historical_legacy_schema'
              and r['phase_mode_assumed'] is True, r['phase_mode_source'])
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    # missing mode and no legacy key -> explicit failure
    tmp = tempfile.mkdtemp(prefix='p1rpa_mode3_')
    try:
        rd = make_run(tmp, 'run', omit_mode=True, legacy_key=False)
        try:
            PA.analyse(rd)
            check('mode_missing_no_schema_fails', False)
        except PA.PhaseAnalysisError:
            check('mode_missing_no_schema_fails', True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    # empty / wrong-type mode -> explicit failure
    for label, mode in (('empty_mode', ''), ('int_mode', 123)):
        tmp = tempfile.mkdtemp(prefix='p1rpa_mode4_')
        try:
            rd = make_run(tmp, 'run', recorded_mode=mode)
            try:
                PA.analyse(rd)
                check('%s_fails' % label, False)
            except PA.PhaseAnalysisError:
                check('%s_fails' % label, True)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


def test_joint_order_contract():
    tmp = tempfile.mkdtemp(prefix='p1rpa_jc_')
    try:
        r = PA.analyse(make_run(tmp, 'run', names_mode='canonical'))
        jo = r['joint_order']
        check('names_provided_verified',
              jo['names_provided'] is True
              and jo['canonical_order_verified'] is True
              and jo['order_assumed'] is False, jo)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    tmp = tempfile.mkdtemp(prefix='p1rpa_jc2_')
    try:
        r = PA.analyse(make_run(tmp, 'run', names_mode='absent'))
        jo = r['joint_order']
        check('names_absent_assumed_not_verified',
              jo['names_provided'] is False
              and jo['order_assumed'] is True
              and jo['canonical_order_verified'] is False
              and jo['order_unverifiable'] is False
              and bool(jo['order_source']), jo)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    for label, nm in (('reversed', 'reversed'), ('empty', 'empty'),
                      ('short', 'short')):
        tmp = tempfile.mkdtemp(prefix='p1rpa_jc3_')
        try:
            rd = make_run(tmp, 'run', names_mode=nm)
            try:
                PA.analyse(rd)
                check('joint_names_%s_fails' % label, False)
            except PA.PhaseAnalysisError:
                check('joint_names_%s_fails' % label, True)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


def test_gate_evaluation():
    base = {'locomotion': {'forward_displacement_m': 0.12,
                           'lateral_displacement_m': 0.01,
                           'yaw_change_deg': 1.0, 'torso_tilt_max_deg': 1.0,
                           'fall': False, 'effective_duration_sec': 14.0,
                           'missing_duration_sec': 0.01, 'path_over_net': 5.0,
                           'contact': {'alternation_count': 20}},
            'measurable': True,
            'left_support_ground_fraction': 0.95,
            'right_support_ground_fraction': 0.95,
            'ds_both_ground_fraction': 0.9,
            'left_swing_airborne_fraction': 0.7,
            'right_swing_airborne_fraction': 0.7,
            'swing_clearance_max_m': {'L': 0.008, 'R': 0.008},
            'support_foot_drift': {'L': {'max_m': 0.003}, 'R': {'max_m': 0.003}},
            'swing_to_support_onset_delay_sec': {'n_boundaries': 21,
                                                 'n_missing': 0,
                                                 'max_sec': 1.02},
            'zero_contact': {'max_duration_sec': 0.9}}
    verdict = PA.evaluate_gates(base, GATES)
    check('gate_all_pass', verdict['pass'] is True, verdict)
    # onset delay magnitude is diagnostic -> large value must NOT fail
    big_onset = json.loads(json.dumps(base))
    big_onset['swing_to_support_onset_delay_sec']['max_sec'] = 99.0
    check('onset_delay_is_diagnostic',
          PA.evaluate_gates(big_onset, GATES)['pass'] is True)
    check('onset_delay_reported_in_diagnostics',
          PA.evaluate_gates(big_onset, GATES)['diagnostics'][
              'swing_to_support_onset_delay_sec']['max_sec'] == 99.0)
    # zero_contact is diagnostic -> large value must NOT fail
    big_zc = json.loads(json.dumps(base))
    big_zc['zero_contact']['max_duration_sec'] = 99.0
    check('zero_contact_is_diagnostic',
          PA.evaluate_gates(big_zc, GATES)['pass'] is True)
    # missing onset is a HARD fail
    miss = json.loads(json.dumps(base))
    miss['swing_to_support_onset_delay_sec']['n_missing'] = 1
    check('missing_onset_hard_fail',
          PA.evaluate_gates(miss, GATES)['pass'] is False)
    # onset None (not measurable) is a HARD fail
    none_onset = json.loads(json.dumps(base))
    none_onset['swing_to_support_onset_delay_sec'] = None
    check('none_onset_hard_fail',
          PA.evaluate_gates(none_onset, GATES)['pass'] is False)
    # zero boundaries is a HARD fail
    zero_b = json.loads(json.dumps(base))
    zero_b['swing_to_support_onset_delay_sec'] = {'n_boundaries': 0,
                                                  'n_missing': 0, 'max_sec': None}
    check('zero_boundaries_hard_fail',
          PA.evaluate_gates(zero_b, GATES)['pass'] is False)
    # individual hard failures
    for key, mutate in (
            ('forward', lambda b: b['locomotion'].__setitem__('forward_displacement_m', -0.05)),
            ('lateral', lambda b: b['locomotion'].__setitem__('lateral_displacement_m', 0.2)),
            ('yaw', lambda b: b['locomotion'].__setitem__('yaw_change_deg', 40.0)),
            ('tilt', lambda b: b['locomotion'].__setitem__('torso_tilt_max_deg', 40.0)),
            ('fall', lambda b: b['locomotion'].__setitem__('fall', True)),
            ('pathnet', lambda b: b['locomotion'].__setitem__('path_over_net', 8.04)),
            ('support_ground', lambda b: b.__setitem__('left_support_ground_fraction', 0.4)),
            ('airborne', lambda b: b.__setitem__('left_swing_airborne_fraction', 0.1)),
            ('drift', lambda b: b['support_foot_drift']['L'].__setitem__('max_m', 0.02)),
            ('clearance', lambda b: b['swing_clearance_max_m'].__setitem__('L', 0.001)),
            ('ds_ground', lambda b: b.__setitem__('ds_both_ground_fraction', 0.1))):
        b = json.loads(json.dumps(base))
        mutate(b)
        check('gate_%s_fail' % key,
              PA.evaluate_gates(b, GATES)['pass'] is False)
    # epsilon: duration just below threshold within epsilon passes
    eps = json.loads(json.dumps(base))
    eps['locomotion']['effective_duration_sec'] = 8.0 - 5e-7
    check('epsilon_tolerates_sub_microsecond',
          PA.evaluate_gates(eps, GATES)['pass'] is True)
    below = json.loads(json.dumps(base))
    below['locomotion']['effective_duration_sec'] = 8.0 - 0.02
    check('epsilon_does_not_absorb_20ms',
          PA.evaluate_gates(below, GATES)['pass'] is False)


def test_strict_json_output():
    tmp = tempfile.mkdtemp(prefix='p1rpa_j2_')
    try:
        out = os.path.join(tmp, 'out.json')
        rd = make_run(tmp, 'run')
        rc = PA.main(['--run', rd, '--out', out])
        check('strict_json_exit_zero', rc == 0)
        json.load(open(out))
        check('strict_json_parses', True)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def test_regression_existing():
    ws = os.path.dirname(_TOOLS)
    rd = os.path.join(ws, 'docs/task_p0a_gait_search/work/reproduce/fwdf08_seed101')
    if not os.path.isdir(rd):
        check('regression_fwdf08_skip', True)
        return
    r = PA.analyse(rd)
    check('regression_fwdf08_mode', r['phase_mode_recorded'] == 'legacy')
    check('regression_fwdf08_support_ground',
          r['left_support_ground_fraction'] > 0.9
          and r['right_support_ground_fraction'] > 0.9,
          (r['left_support_ground_fraction'], r['right_support_ground_fraction']))
    check('regression_fwdf08_m0_matches_old',
          abs(r['m0_legacy_contact_delay_sec']['max_sec'] - 0.39) < 0.02,
          r['m0_legacy_contact_delay_sec'])
    # old P1 exploratory explicit run: read-only recompute must match recorded mode
    old = os.path.join(ws, 'docs/task_p0a_gait_search/p1_support_swing/runs/B_explicit')
    if os.path.isdir(old):
        ro = PA.analyse(old)
        check('regression_old_p1_explicit_mode',
              ro['phase_mode_recorded'] == 'explicit_support_swing')
        check('regression_old_p1_explicit_support_ground',
              ro['left_support_ground_fraction'] > 0.9
              and ro['right_support_ground_fraction'] > 0.9)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('P0-a P1-R phase-analysis fixtures')
    test_phase_stats()
    test_contact_and_mode_failures()
    test_contact_ordering_and_duplicates()
    test_mode_contract()
    test_joint_order_contract()
    test_gate_evaluation()
    test_strict_json_output()
    test_regression_existing()
    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\nphase-analysis fixtures: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w') as fh:
            json.dump({'passed': passed, 'total': total, 'results': RESULTS},
                      fh, indent=2, sort_keys=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
