#!/usr/bin/env python3
"""Offline Phase A tests: episode_integrity verdicts + failure paths (no Gazebo).

Covers Phase A spec §5/§12/§13/§14/§16, the §9 config-precedence fix, and the
Phase A Closure hardening cases (§9 CASE A–D + explicit duration semantics).

Run:
    python3 tools/test_phase_a_episode_integrity.py [--json-out F]
Exit 0 = all checks passed; 1 = at least one failed.
"""

import argparse
import json
import os
import shutil
import sys
import tempfile

_TOOLS = os.path.dirname(os.path.abspath(__file__))
_WS = os.path.dirname(_TOOLS)
for _p in (_TOOLS, os.path.join(_WS, 'src', 'nao_gz_sim', 'scripts')):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import nao_episode_integrity as EI  # noqa: E402
import nao_trajectory_schema as SCH  # noqa: E402
import run_p0a_gait_candidate as R  # noqa: E402

RESULTS = []


def check(name, ok, detail=None):
    RESULTS.append({'name': name, 'pass': bool(ok), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if ok else 'FAIL', name,
                           '' if ok else ' :: %r' % (detail,)))


_MID = {n: (lo + hi) / 2.0 for n, (lo, hi) in SCH.LIMITS.items()}
_MID_POS = [round(_MID[n], 6) for n in SCH.JOINT_ORDER]


def _valid_sample(ts):
    return {
        'timestamp': round(float(ts), 6),
        'joint_position': list(_MID_POS),
        'joint_velocity': [0.0] * 24,
        'imu': {'stamp': round(float(ts), 6),
                'orientation': [0.0, 0.0, 0.0, 1.0],
                'angular_velocity': [0.0, 0.0, 0.0],
                'linear_acceleration': [0.0, 0.0, 9.81]},
        'left_contact': True,
        'right_contact': False,
        'command': {'stamp': round(float(ts) - 0.01, 6), 'position': list(_MID_POS)},
    }


def make_episode(root, samples=None, summary=None, body_pose=True,
                 manifest=True, raw_sample_lines=None, phase4b=None,
                 state_first=None):
    """Create a synthetic Phase A episode dir; returns its path."""
    ds = os.path.join(root, 'dataset')
    tr = os.path.join(root, 'trace')
    os.makedirs(ds, exist_ok=True)
    os.makedirs(tr, exist_ok=True)
    if manifest:
        with open(os.path.join(ds, 'manifest.json'), 'w', encoding='utf-8') as fh:
            json.dump(SCH.build_manifest(), fh, indent=2, allow_nan=False)
    if raw_sample_lines is not None:
        with open(os.path.join(ds, 'samples.jsonl'), 'w', encoding='utf-8') as fh:
            fh.write(raw_sample_lines)
    elif samples is not None:
        with open(os.path.join(ds, 'samples.jsonl'), 'w', encoding='utf-8') as fh:
            for s in samples:
                fh.write(json.dumps(s, allow_nan=False) + '\n')
    if body_pose:
        with open(os.path.join(tr, 'body_pose_trace.jsonl'), 'w', encoding='utf-8') as fh:
            fh.write(json.dumps({'t': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.30,
                                 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                                 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}) + '\n')
    if state_first is not None:
        with open(os.path.join(tr, 'state_trace.jsonl'), 'w', encoding='utf-8') as fh:
            fh.write(json.dumps({'t': state_first, 'position': list(_MID_POS)}) + '\n')
    if phase4b is not None:
        with open(os.path.join(tr, 'phase4b_summary.json'), 'w', encoding='utf-8') as fh:
            json.dump(phase4b, fh, allow_nan=False)
    if summary is not None:
        with open(os.path.join(root, 'collector_summary.json'), 'w',
                  encoding='utf-8') as fh:
            json.dump(summary, fh, indent=2, allow_nan=False)
    return root


def _series(n=81, step=0.1, start=1.0):
    return [_valid_sample(start + i * step) for i in range(n)]


def _ok_summary():
    return {'success': True, 'process_completed': True, 'dataset_committed': True,
            'has_samples': True, 'finish_reason': 'duration_complete',
            'duration_sec': 12.0, 'samples_written': 81, 'unaccounted': 0}


def _cov_episode(tmp, ts, requested, gait_start, finish='duration_complete'):
    """Episode whose accepted-sample timestamps are exactly ``ts``."""
    d = tempfile.mkdtemp(dir=tmp)
    make_episode(d, samples=[_valid_sample(t) for t in ts],
                 summary=dict(_ok_summary(), finish_reason=finish))
    return d, EI.evaluate(d, requested_duration_sec=requested, gait_start_sec=gait_start,
                          record_window_duration_sec=12.0)


def _frange(a, b, step=0.1):
    out, x = [], a
    while x <= b + 1e-9:
        out.append(round(x, 6))
        x += step
    return out


def test_episode_integrity(tmp):
    # 1. valid episode -> PASS
    d = tempfile.mkdtemp(dir=tmp)
    make_episode(d, samples=_series(), summary=_ok_summary())
    v = EI.evaluate(d, requested_duration_sec=8.0)
    check('valid_episode_pass', v['episode_integrity'] == 'PASS'
          and v['failure_class'] is None, v['failure_class'])
    check('valid_signals_all_pass',
          all(s == 'PASS' for s in v['signals'].values()), v['signals'])

    # 2. gazebo start failure: nothing produced
    d = tempfile.mkdtemp(dir=tmp)
    v = EI.evaluate(d, requested_duration_sec=8.0)
    check('gazebo_start_failure_infra',
          v['episode_integrity'] == 'FAIL'
          and v['failure_class'] == 'infrastructure_failure', v['failure_class'])

    # 3. collector produced no dataset (summary present, no dataset dir)
    d = tempfile.mkdtemp(dir=tmp)
    make_episode(d, samples=None, manifest=False, body_pose=False)
    with open(os.path.join(d, 'collector_summary.json'), 'w') as fh:
        json.dump({'success': False, 'dataset_committed': False,
                   'finish_reason': None}, fh)
    v = EI.evaluate(d, requested_duration_sec=8.0)
    check('collector_failure_infra',
          v['episode_integrity'] == 'FAIL'
          and v['failure_class'] == 'infrastructure_failure', v['failure_class'])

    # 4. early termination: valid data but finish_reason=shutdown + short
    d = tempfile.mkdtemp(dir=tmp)
    make_episode(d, samples=_series(n=31), summary={'success': True,
                 'process_completed': True, 'dataset_committed': True,
                 'finish_reason': 'shutdown', 'samples_written': 31})
    v = EI.evaluate(d, requested_duration_sec=8.0)
    check('early_termination_infra',
          v['episode_integrity'] == 'FAIL'
          and v['failure_class'] == 'infrastructure_failure'
          and v['termination'] == 'early_termination', v['termination'])

    # 5. missing command in one sample -> data integrity
    d = tempfile.mkdtemp(dir=tmp)
    s = _series()
    s[5]['command'] = None
    make_episode(d, samples=s, summary=_ok_summary())
    v = EI.evaluate(d, requested_duration_sec=8.0)
    check('missing_command_data_integrity',
          v['episode_integrity'] == 'FAIL'
          and v['failure_class'] == 'data_integrity_failure'
          and v['signals']['command'] == 'FAIL', v['signals']['command'])

    # 6. invalid timestamp (non-monotonic) -> data integrity
    d = tempfile.mkdtemp(dir=tmp)
    s = _series()
    s[10]['timestamp'] = s[9]['timestamp']
    make_episode(d, samples=s, summary=_ok_summary())
    v = EI.evaluate(d, requested_duration_sec=8.0)
    check('invalid_timestamp_data_integrity',
          v['episode_integrity'] == 'FAIL'
          and v['failure_class'] == 'data_integrity_failure'
          and v['signals']['clock'] == 'FAIL', v['signals']['clock'])

    # 7. missing joint state (wrong dim) -> data integrity
    d = tempfile.mkdtemp(dir=tmp)
    s = _series()
    s[3]['joint_position'] = [0.0] * 23
    make_episode(d, samples=s, summary=_ok_summary())
    v = EI.evaluate(d, requested_duration_sec=8.0)
    check('missing_joint_state_data_integrity',
          v['episode_integrity'] == 'FAIL'
          and v['failure_class'] == 'data_integrity_failure'
          and v['signals']['joint_state'] == 'FAIL', v['signals']['joint_state'])

    # 8. malformed sample line -> data integrity
    d = tempfile.mkdtemp(dir=tmp)
    lines = ''.join(json.dumps(x, allow_nan=False) + '\n' for x in _series())
    lines += 'this is not json\n'
    make_episode(d, raw_sample_lines=lines, summary=_ok_summary())
    v = EI.evaluate(d, requested_duration_sec=8.0)
    check('malformed_sample_data_integrity',
          v['episode_integrity'] == 'FAIL'
          and v['failure_class'] == 'data_integrity_failure', v['failure_class'])

    # 9. missing body pose trace -> data integrity
    d = tempfile.mkdtemp(dir=tmp)
    make_episode(d, samples=_series(), summary=_ok_summary(), body_pose=False)
    v = EI.evaluate(d, requested_duration_sec=8.0)
    check('missing_body_pose_data_integrity',
          v['episode_integrity'] == 'FAIL'
          and v['signals']['body_pose'] == 'FAIL', v['signals']['body_pose'])

    # 10. zero samples -> infrastructure
    d = tempfile.mkdtemp(dir=tmp)
    make_episode(d, raw_sample_lines='', summary=_ok_summary())
    v = EI.evaluate(d, requested_duration_sec=8.0)
    check('zero_samples_infra',
          v['episode_integrity'] == 'FAIL'
          and v['failure_class'] == 'infrastructure_failure', v['failure_class'])

    # 11. §13: a fall must NOT be an integrity failure
    d = tempfile.mkdtemp(dir=tmp)
    make_episode(d, samples=_series(), summary=_ok_summary())
    with open(os.path.join(d, 'trace', 'phase4b_summary.json'), 'w') as fh:
        json.dump({'body_initial_z': 0.32, 'body_min_z': 0.15, 'body_final_z': 0.16,
                   'body_torso_height_loss': 0.17, 'body_net_displacement_m': 0.05,
                   'body_max_roll_deg': 60.0}, fh)
    v = EI.evaluate(d, requested_duration_sec=8.0)
    check('fall_is_not_integrity_failure',
          v['episode_integrity'] == 'PASS'
          and v['gait_outcome']['fall_detected'] is True,
          v['gait_outcome'])
    check('fall_reported_separately', v['reserved'] == {'measurement_quality': None,
                                                        'contract_status': None},
          v['reserved'])


def test_duration_semantics(tmp):
    # C1: the four duration concepts are distinct and explicit.
    d, v = _cov_episode(tmp, _frange(0.5, 9.5), 8.0, 1.0)
    dur = v['duration']
    check('requested_gait_duration_sec', dur['requested_gait_duration_sec'] == 8.0,
          dur['requested_gait_duration_sec'])
    check('record_window_duration_sec', dur['record_window_duration_sec'] == 12.0,
          dur['record_window_duration_sec'])
    check('accepted_sample_span_sec', abs(dur['accepted_sample_span_sec'] - 9.0) < 1e-6,
          dur['accepted_sample_span_sec'])
    check('legacy_alias_is_span',
          dur['actual_duration_sec_LEGACY'] == dur['accepted_sample_span_sec'],
          dur['actual_duration_sec_LEGACY'])
    check('execution_duration_absent_without_clock_trace',
          dur['execution_duration_sec'] is None, dur['execution_duration_sec'])

    # execution_duration_sec sourced from clock_trace when present
    d = tempfile.mkdtemp(dir=tmp)
    make_episode(d, samples=_series(), summary=_ok_summary())
    with open(os.path.join(d, 'trace', 'clock_trace.jsonl'), 'w') as fh:
        for t in (0.0, 5.0, 12.5):
            fh.write(json.dumps({'t': t}) + '\n')
    v = EI.evaluate(d, requested_duration_sec=8.0)
    check('execution_duration_from_clock_trace',
          v['duration']['execution_duration_sec'] == 12.5
          and 'clock_trace' in v['duration']['execution_duration_source'],
          v['duration']['execution_duration_sec'])


def test_coverage(tmp):
    # CASE A: full gait window covered -> coverage ~1.0
    _d, v = _cov_episode(tmp, _frange(0.5, 9.5), 8.0, 1.0)
    cov = v['coverage']
    check('CASE_A_coverage_full', cov['gait_relative_coverage'] >= 0.999,
          cov['gait_relative_coverage'])
    check('CASE_A_status_full', cov['coverage_status'] == 'FULL', cov['coverage_status'])
    check('CASE_A_head_gap_zero', cov['gait_head_gap_sec'] < 0.05,
          cov['gait_head_gap_sec'])

    # CASE B: first 2 s of the gait window have no accepted samples -> ~0.75
    _d, v = _cov_episode(tmp, _frange(3.0, 9.0), 8.0, 1.0)
    cov = v['coverage']
    check('CASE_B_coverage_075', abs(cov['gait_relative_coverage'] - 0.75) < 0.02,
          cov['gait_relative_coverage'])
    check('CASE_B_head_gap_2', abs(cov['gait_head_gap_sec'] - 2.0) < 0.05,
          cov['gait_head_gap_sec'])
    check('CASE_B_tail_zero', cov['gait_tail_truncation_sec'] < 0.05,
          cov['gait_tail_truncation_sec'])
    check('CASE_B_status_partial', cov['coverage_status'] == 'PARTIAL',
          cov['coverage_status'])
    # head-only gap is disclosed, NOT an integrity failure (§8: no behaviour change)
    check('CASE_B_head_gap_not_integrity_failure',
          v['episode_integrity'] == 'PASS' and v['termination'] == 'normal_completion',
          (v['episode_integrity'], v['termination']))

    # CASE C: gait stops 3 s in -> coverage ~0.375 + early termination FAIL
    _d, v = _cov_episode(tmp, _frange(1.0, 4.0), 8.0, 1.0)
    cov = v['coverage']
    check('CASE_C_coverage_0375', abs(cov['gait_relative_coverage'] - 0.375) < 0.02,
          cov['gait_relative_coverage'])
    check('CASE_C_tail_truncation', cov['gait_tail_truncation_sec'] > 4.0,
          cov['gait_tail_truncation_sec'])
    check('CASE_C_early_termination',
          v['termination'] == 'early_termination'
          and v['episode_integrity'] == 'FAIL'
          and v['failure_class'] == 'infrastructure_failure',
          (v['termination'], v['episode_integrity'], v['failure_class']))

    # early termination BEFORE gait onset: no accepted samples in the window
    _d, v = _cov_episode(tmp, _frange(1.0, 5.0), 8.0, 10.0)
    check('early_termination_before_onset',
          v['termination'] == 'early_termination' and v['episode_integrity'] == 'FAIL',
          (v['termination'], v['episode_integrity']))
    check('early_termination_before_onset_coverage_zero',
          v['coverage']['gait_relative_coverage'] == 0.0,
          v['coverage']['gait_relative_coverage'])

    # early termination shortly after onset (1 s of 8)
    _d, v = _cov_episode(tmp, _frange(1.0, 2.0), 8.0, 1.0)
    check('early_termination_shortly_after_onset',
          v['termination'] == 'early_termination' and v['episode_integrity'] == 'FAIL',
          (v['termination'], v['episode_integrity']))

    # normal completion (full window, finish_reason normal)
    _d, v = _cov_episode(tmp, _frange(0.5, 9.5), 8.0, 1.0)
    check('normal_completion', v['termination'] == 'normal_completion'
          and v['episode_integrity'] == 'PASS', v['termination'])

    # CASE D: gait outcome FALL / poor walking must NOT fail integrity
    d = tempfile.mkdtemp(dir=tmp)
    make_episode(d, samples=[_valid_sample(t) for t in _frange(0.5, 9.5)],
                 summary=dict(_ok_summary(), finish_reason='duration_complete'),
                 phase4b={'walking_pass': False, 'producer_contract_pass': True,
                          'body_initial_z': 0.32, 'body_min_z': 0.14,
                          'body_final_z': 0.15, 'body_torso_height_loss': 0.18,
                          'body_net_displacement_m': 0.02})
    v = EI.evaluate(d, requested_duration_sec=8.0, gait_start_sec=1.0)
    check('CASE_D_fall_not_integrity_failure',
          v['episode_integrity'] == 'PASS'
          and v['gait_outcome']['fall_detected'] is True
          and v['gait_outcome']['walking_pass'] is False,
          (v['episode_integrity'], v['gait_outcome'].get('fall_detected'),
           v['gait_outcome'].get('walking_pass')))

    # gait_start derived from trace + provenance when not passed explicitly
    d = tempfile.mkdtemp(dir=tmp)
    make_episode(d, samples=[_valid_sample(t) for t in _frange(0.5, 9.5)],
                 summary=dict(_ok_summary(), finish_reason='duration_complete'),
                 state_first=1.019)
    with open(os.path.join(d, 'provenance.json'), 'w') as fh:
        json.dump({'gait_parameters': {'start_delay_sec': 0.5}}, fh)
    v = EI.evaluate(d, requested_duration_sec=8.0)
    check('gait_start_derived_from_trace_and_provenance',
          abs(v['coverage']['gait_start_sec'] - 1.519) < 1e-6
          and 'state_trace' in v['coverage']['gait_start_source'],
          v['coverage']['gait_start_sec'])


def test_config_precedence():
    anchor = {'duration_sec': 16.0, 'start_delay_sec': 0.5, 'step_frequency_hz': 0.8}
    cfg = R.resolve_config(anchor, {'duration_sec': 5.0})
    check('runner_defaults_then_overrides',
          cfg['duration_sec'] == 5.0 and cfg['stop_ramp_sec'] == 1.5
          and cfg['hold_final_sec'] == 1.0, cfg.get('duration_sec'))


def test_smoke_block(tmp):
    d, v = _cov_episode(tmp, _frange(0.5, 9.5), 8.0, 1.0)
    block = EI.format_smoke_block(v, seed=12345, requested_duration=8.0)
    for token in ('EPISODE_ID', 'SEED', 'REQUESTED_GAIT_DURATION_SEC',
                  'RECORD_WINDOW_DURATION_SEC', 'EXECUTION_DURATION_SEC',
                  'ACCEPTED_SAMPLE_SPAN_SEC', 'GAIT_RELATIVE_COVERAGE',
                  'GAIT_HEAD_GAP_SEC', 'GAIT_TAIL_TRUNCATION_SEC', 'COVERAGE_STATUS',
                  'SAMPLE_COUNT', 'JOINT_STATE:', 'COMMAND:', 'IMU:', 'CONTACT:',
                  'BODY_POSE:', 'CLOCK:', 'EPISODE_INTEGRITY:', 'TERMINATION:'):
        check('smoke_block_has_%s' % token.rstrip(':'), token in block, block)
    check('smoke_block_no_ambiguous_actual_duration',
          'ACTUAL_DURATION:' not in block, block)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    tmp = tempfile.mkdtemp(prefix='phase_a_test_')
    try:
        print('Phase A episode-integrity offline tests')
        test_episode_integrity(tmp)
        test_duration_semantics(tmp)
        test_coverage(tmp)
        test_config_precedence()
        test_smoke_block(tmp)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    n_pass = sum(1 for r in RESULTS if r['pass'])
    report = {'n_pass': n_pass, 'n_total': len(RESULTS), 'results': RESULTS}
    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as fh:
            json.dump(report, fh, indent=2, sort_keys=True)
    print('\nphase A integrity tests: %d/%d passed' % (n_pass, len(RESULTS)))
    return 0 if n_pass == len(RESULTS) else 1


if __name__ == '__main__':
    sys.exit(main())
