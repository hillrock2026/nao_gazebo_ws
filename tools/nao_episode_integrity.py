#!/usr/bin/env python3
"""Phase A episode integrity validator (offline, read-only).

This module establishes exactly ONE verdict: ``episode_integrity`` (PASS/FAIL).
It is deliberately kept separate from the two *future* axes, which are only
reserved here as ``null`` placeholders and never computed:

    episode_integrity    <- this module (data was acquired coherently)
    measurement_quality  <- reserved (is the data scientifically usable?)
    contract_status      <- reserved (P1-R v3 gate verdict; NOT Phase A)

Design invariants (Phase A spec §4, §12, §13, §14):
  * A gait that runs and then FALLS is ``episode_integrity = PASS`` as long as
    the raw data is complete. The fall is reported under ``gait_outcome`` and
    never flips the integrity verdict.
  * Only data/plumbing problems fail integrity, split into two classes:
      - ``infrastructure_failure`` : Gazebo/collector never produced a dataset,
        zero samples, or the requested gait window was truncated (early
        termination).
      - ``data_integrity_failure`` : a dataset exists but signals are missing /
        corrupt / schema-invalid.
  * Gait outcomes (fall, large yaw, drift, poor clearance) are NOT integrity
    failures and are never emitted as the failure class.

Phase A Closure (P1-1) hardening — explicit, non-ambiguous duration semantics:

  * ``requested_gait_duration_sec``  : user-requested gait duration (the window
    the dataset must cover).
  * ``record_window_duration_sec``   : collector/runner allowed acquisition
    window (= start_delay + gait + stop_ramp + final_hold + margin).
  * ``execution_duration_sec``       : observed Gazebo sim-time span
    (trace/clock_trace.jsonl first->last). ``None`` if no clock trace exists
    (never fabricated).
  * ``accepted_sample_span_sec``     : last - first accepted sample timestamp.
    This is NOT execution duration and NOT requested gait duration.
  * legacy ``actual_duration_sec`` / manifest ``duration_sec`` : DEPRECATED
    aliases of ``accepted_sample_span_sec`` (kept for backward compatibility,
    explicitly labelled, and no longer allowed to mean "how long the episode
    ran").
  * ``gait_relative_coverage``       : fraction of the requested gait window
    [gait_start, gait_start + requested] covered by accepted samples, computed
    as the union of consecutive-sample intervals clipped to the window, with any
    gap longer than ``GAP_CAP_SEC`` treated as uncovered (data gap). See the
    ``coverage`` block in the verdict for numerator/denominator/source/policy.
  * ``gait_head_gap_sec`` / ``gait_tail_truncation_sec`` : where coverage is
    missing. The early-termination guard keys on ``gait_tail_truncation_sec``
    (the gait ran short), which removes the old span-based head blind spot.

The deep frozen-schema contract (v1.0.1) is verified by the independent golden
validator ``tools/validate_task4_phase4d_dataset.py`` (imported, not
re-implemented) so this module never becomes a second source of truth.

CLI:
    python3 tools/nao_episode_integrity.py <episode_dir> [--json-out F]
Exit code 0 = episode_integrity PASS, 1 = FAIL (report still printed).
"""

import argparse
import json
import math
import os
import sys

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import validate_task4_phase4d_dataset as GV  # noqa: E402  (independent golden validator)

MANIFEST_FILE = 'manifest.json'
SAMPLES_FILE = 'samples.jsonl'

# Order used for the smoke printout.
SIGNALS = ('joint_state', 'command', 'imu', 'contact', 'body_pose', 'clock')

# Bounds used only to *report* a fall; never used to fail integrity.
FALL_HEIGHT_LOSS_M = 0.10

# Coverage policy (Phase A Closure C2): a consecutive-sample interval longer
# than this is a data gap and is NOT counted as covered.
GAP_CAP_SEC = 0.10
# Early-termination guard (C3): the gait window end missed by more than this
# many seconds of accepted samples means the gait did not complete.
TAIL_TRUNCATION_TOL_SEC = 0.5
# Coverage is labelled FULL at/above this fraction; disclosure only, never a gate.
FULL_COVERAGE_FRACTION = 0.999


def _finite(value):
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _finite_list(values, length):
    return (isinstance(values, list) and len(values) == length
            and all(_finite(v) for v in values))


def _read_jsonl(path):
    """Yield (lineno, obj_or_None, raw_line) for a JSONL file; None => malformed."""
    if not os.path.isfile(path):
        return
    with open(path, 'r', encoding='utf-8') as fh:
        for lineno, line in enumerate(fh, start=1):
            if not line.strip():
                yield lineno, None, line
                continue
            try:
                yield lineno, json.loads(line), line
            except ValueError:
                yield lineno, None, line


def _trace_t_span(path, key='t'):
    """Return (first, last) of a numeric ``key`` column in a JSONL trace."""
    first = last = None
    for _lineno, obj, _raw in _read_jsonl(path):
        if obj is None or not _finite(obj.get(key)):
            continue
        v = float(obj[key])
        if first is None:
            first = v
        last = v
    return first, last


def _check_samples(samples_path):
    """Strict per-signal scan of samples.jsonl.

    Returns (per_signal, info) where ``per_signal`` maps a signal to
    ``{'pass', 'n_ok', 'n_bad', 'examples'}`` and ``info`` carries the
    sample accounting (count, timestamps, first malformed line).
    """
    per_signal = {s: {'pass': False, 'n_ok': 0, 'n_bad': 0, 'examples': []}
                  for s in SIGNALS}
    info = {'samples': 0, 'first_ts': None, 'last_ts': None, 'prev_ts': None,
            'timestamps': [], 'timestamp_monotonic': True, 'malformed_lines': [],
            'missing_keys': {}}

    def bad(signal, lineno, reason):
        rec = per_signal[signal]
        rec['n_bad'] += 1
        if len(rec['examples']) < 5:
            rec['examples'].append({'line': lineno, 'reason': reason})

    saw_any = False
    for lineno, obj, _raw in _read_jsonl(samples_path):
        if obj is None:
            info['malformed_lines'].append(lineno)
            continue
        saw_any = True
        info['samples'] += 1
        for key in ('timestamp', 'joint_position', 'joint_velocity', 'imu',
                    'left_contact', 'right_contact', 'command'):
            if key not in obj:
                info['missing_keys'][key] = info['missing_keys'].get(key, 0) + 1

        ts = obj.get('timestamp')
        if not _finite(ts) or float(ts) <= 0:
            bad('clock', lineno, 'timestamp_not_positive_finite')
        else:
            ts = float(ts)
            if info['prev_ts'] is not None and ts <= info['prev_ts']:
                info['timestamp_monotonic'] = False
                bad('clock', lineno, 'timestamp_not_strictly_increasing')
            info['prev_ts'] = ts
            if info['first_ts'] is None:
                info['first_ts'] = ts
            info['last_ts'] = ts
            info['timestamps'].append(ts)
            per_signal['clock']['n_ok'] += 1

        if _finite_list(obj.get('joint_position'), 24) and \
                _finite_list(obj.get('joint_velocity'), 24):
            per_signal['joint_state']['n_ok'] += 1
        else:
            bad('joint_state', lineno, 'joint_position/velocity_not_24_finite')

        cmd = obj.get('command')
        if isinstance(cmd, dict) and _finite_list(cmd.get('position'), 24):
            per_signal['command']['n_ok'] += 1
        else:
            bad('command', lineno, 'command_position_not_24_finite')

        imu = obj.get('imu')
        if isinstance(imu, dict) and _finite_list(imu.get('orientation'), 4) \
                and _finite_list(imu.get('angular_velocity'), 3) \
                and _finite_list(imu.get('linear_acceleration'), 3):
            per_signal['imu']['n_ok'] += 1
        else:
            bad('imu', lineno, 'imu_vectors_not_finite')

        if isinstance(obj.get('left_contact'), bool) and \
                isinstance(obj.get('right_contact'), bool):
            per_signal['contact']['n_ok'] += 1
        else:
            bad('contact', lineno, 'contact_not_bool')

    for signal, rec in per_signal.items():
        rec['pass'] = saw_any and rec['n_ok'] > 0 and rec['n_bad'] == 0
    return per_signal, info


def _check_body_pose(trace_dir):
    """Body pose lives only in the raw phase4b trace (not the frozen samples)."""
    rec = {'pass': False, 'count': 0, 'examples': []}
    if not trace_dir:
        return rec
    path = os.path.join(trace_dir, 'body_pose_trace.jsonl')
    for lineno, obj, _raw in _read_jsonl(path):
        if obj is None:
            rec['examples'].append({'line': lineno, 'reason': 'malformed'})
            continue
        ok = (_finite(obj.get('x')) and _finite(obj.get('y')) and _finite(obj.get('z'))
              and _finite(obj.get('roll')) and _finite(obj.get('pitch'))
              and _finite(obj.get('yaw')) and _finite_list(
                  [obj.get('qx'), obj.get('qy'), obj.get('qz'), obj.get('qw')], 4))
        if ok:
            rec['count'] += 1
        elif len(rec['examples']) < 5:
            rec['examples'].append({'line': lineno, 'reason': 'non_finite_pose'})
    rec['pass'] = rec['count'] > 0 and not rec['examples']
    return rec


def derive_gait_start(episode_dir, trace_dir):
    """Authoritative gait-window start: producer ``ready_stamp + start_delay``.

    ``ready_stamp`` is the first valid /joint_states stamp (the producer treats
    the first full 24-joint message as the model-ready instant); ``start_delay``
    comes from ``provenance.gait_parameters``. Returns ``(gait_start|None, src)``.
    """
    first_state = None
    st = os.path.join(trace_dir or '', 'state_trace.jsonl')
    if os.path.isfile(st):
        first_state, _ = _trace_t_span(st)
    if first_state is None:
        return None, 'unavailable:no_state_trace'
    start_delay = None
    prov_path = os.path.join(episode_dir, 'provenance.json')
    if os.path.isfile(prov_path):
        try:
            prov = json.load(open(prov_path, encoding='utf-8'))
            start_delay = (prov.get('gait_parameters') or {}).get('start_delay_sec')
        except (OSError, ValueError):
            start_delay = None
    if start_delay is None:
        start_delay = 0.5  # producer default; matches FROZEN_GAIT
        src = 'state_trace.first + default(start_delay=0.5)'
    else:
        src = 'provenance.gait_parameters.start_delay_sec'
    return first_state + float(start_delay), 'state_trace.first+start_delay(%s)' % src


def _coverage(timestamps, gait_start, gait_end):
    """Union-of-sample-intervals coverage of the gait window (C2).

    numerator   = measure of the union of consecutive-sample intervals, clipped
                  to [gait_start, gait_end]; a consecutive interval longer than
                  GAP_CAP_SEC is a data gap and contributes 0.
    denominator = requested gait duration (gait_end - gait_start).
    timestamp   = dataset/samples.jsonl[].timestamp (gazebo sim time).
    rejected prefix / terminal tail are excluded automatically by the window clip.
    """
    requested = gait_end - gait_start
    covered = 0.0
    for a, b in zip(timestamps, timestamps[1:]):
        if (b - a) > GAP_CAP_SEC + 1e-6:
            continue
        lo, hi = max(a, gait_start), min(b, gait_end)
        if hi > lo:
            covered += hi - lo
    in_window = [t for t in timestamps if gait_start <= t <= gait_end]
    after = [t for t in timestamps if t >= gait_start]
    before = [t for t in timestamps if t <= gait_end]
    head_gap = (after[0] - gait_start) if after else requested
    head_gap = max(0.0, min(head_gap, requested))
    tail_truncation = (gait_end - before[-1]) if before else requested
    tail_truncation = max(0.0, min(tail_truncation, requested))
    coverage = (covered / requested) if requested > 0 else None
    if coverage is not None:
        coverage = min(1.0, coverage)
    return {
        'requested_gait_duration_sec': requested,
        'gait_start_sec': gait_start,
        'gait_end_sec': gait_end,
        'numerator_covered_sec': covered,
        'denominator_sec': requested,
        'gait_relative_coverage': coverage,
        'gait_head_gap_sec': head_gap,
        'gait_tail_truncation_sec': tail_truncation,
        'n_samples_in_window': len(in_window),
        'coverage_status': (None if coverage is None else
                            ('FULL' if coverage >= FULL_COVERAGE_FRACTION else 'PARTIAL')),
        'timestamp_source': 'dataset/samples.jsonl[].timestamp',
        'gap_policy': ('union of consecutive-sample intervals clipped to the gait '
                       'window; intervals > %.2fs treated as uncovered gaps' % GAP_CAP_SEC),
    }


def _classify_termination(finish_reason, requested, span, has_dataset, tail_truncation):
    if not has_dataset:
        return 'collector_failure' if finish_reason is None else 'no_dataset'
    if finish_reason in ('duration_complete', 'max_samples'):
        classification = 'normal_completion'
    elif finish_reason == 'shutdown':
        classification = 'early_termination'
    elif finish_reason is None:
        classification = 'collector_failure'
    else:
        classification = 'early_termination'
    if tail_truncation is not None:
        # C3: guard on the gait window itself (removes the span-based head blind
        # spot). A window end missed by > tolerance means the gait ran short.
        if tail_truncation > TAIL_TRUNCATION_TOL_SEC and \
                classification == 'normal_completion':
            classification = 'early_termination'
    elif requested is not None and span is not None and span < requested - 1.0:
        # Legacy fallback only when no gait window is available (kept for
        # backward compatibility with episodes lacking a state trace).
        if classification == 'normal_completion':
            classification = 'early_termination'
    return classification


def _gait_outcome(trace_dir):
    """Informational only: gait quality is NEVER an integrity failure (§13)."""
    outcome = {'available': False, 'fall_detected': None, 'note':
               'gait outcome is reported, not an integrity verdict'}
    if not trace_dir:
        return outcome
    path = os.path.join(trace_dir, 'phase4b_summary.json')
    if not os.path.isfile(path):
        return outcome
    try:
        s = json.load(open(path, encoding='utf-8'))
    except (OSError, ValueError):
        return outcome
    outcome['available'] = True
    for key in ('body_net_displacement_m', 'body_net_forward_m', 'body_net_lateral_m',
                'body_initial_z', 'body_min_z', 'body_final_z',
                'body_torso_height_loss', 'body_max_roll_deg', 'body_max_pitch_deg',
                'producer_contract_pass', 'walking_pass'):
        if key in s:
            outcome[key] = s[key]
    loss = s.get('body_torso_height_loss')
    init_z = s.get('body_initial_z')
    final_z = s.get('body_final_z')
    if loss is not None and init_z is not None and final_z is not None:
        outcome['fall_detected'] = bool(
            loss > FALL_HEIGHT_LOSS_M or final_z < init_z - FALL_HEIGHT_LOSS_M)
    return outcome


def evaluate(episode_dir, requested_duration_sec=None, collector_summary=None,
             episode_id=None, gait_start_sec=None, record_window_duration_sec=None):
    """Evaluate one episode directory and return the integrity verdict dict.

    ``episode_dir`` layout (Phase A):
        dataset/{manifest.json,samples.jsonl}   frozen schema (raw trajectory)
        trace/...                               raw per-topic traces (incl. body_pose)
        provenance.json                         (optional; not required to judge)
    """
    episode_dir = os.path.abspath(episode_dir)
    dataset_dir = os.path.join(episode_dir, 'dataset')
    trace_dir = os.path.join(episode_dir, 'trace')
    samples_path = os.path.join(dataset_dir, SAMPLES_FILE)
    has_dataset = os.path.isdir(dataset_dir)

    summary = collector_summary
    if summary is None:
        p = os.path.join(episode_dir, 'collector_summary.json')
        if os.path.isfile(p):
            try:
                summary = json.load(open(p, encoding='utf-8'))
            except (OSError, ValueError):
                summary = None
    finish_reason = (summary or {}).get('finish_reason')

    per_signal, info = _check_samples(samples_path)
    body_pose = _check_body_pose(trace_dir)
    signals = {s: ('PASS' if per_signal[s]['pass'] else 'FAIL') for s in SIGNALS}
    signals['body_pose'] = 'PASS' if body_pose['pass'] else 'FAIL'

    # Deep frozen-schema contract (independent golden validator) on dataset/.
    gv_report = GV.validate_dataset(dataset_dir) if has_dataset else {
        'valid': False, 'manifest_ok': False, 'samples_ok': False, 'samples': 0,
        'errors': [{'where': 'dataset', 'reason': 'dataset_missing'}]}

    # ---- C1: explicit duration semantics ----
    requested_gait = requested_duration_sec
    if requested_gait is None and os.path.isfile(os.path.join(episode_dir, 'manifest.json')):
        try:
            m = json.load(open(os.path.join(episode_dir, 'manifest.json'), encoding='utf-8'))
            requested_gait = (m.get('duration') or {}).get(
                'requested_gait_duration_sec', m.get('requested_duration_sec'))
        except (OSError, ValueError):
            requested_gait = None
    record_window = record_window_duration_sec
    if record_window is None:
        record_window = (summary or {}).get('duration_sec')
    exec_first, exec_last = _trace_t_span(os.path.join(trace_dir, 'clock_trace.jsonl'))
    execution_duration = (exec_last - exec_first) if (exec_first is not None
                                                      and exec_last is not None) else None
    accepted_span = (None if info['first_ts'] is None or info['last_ts'] is None
                     else info['last_ts'] - info['first_ts'])

    # ---- C2: gait window + coverage ----
    if gait_start_sec is None:
        gait_start_sec, gait_start_src = derive_gait_start(episode_dir, trace_dir)
    else:
        gait_start_src = 'caller (producer ready + start_delay)'
    coverage = None
    tail_truncation = None
    if gait_start_sec is not None and requested_gait:
        coverage = _coverage(info['timestamps'], float(gait_start_sec),
                             float(gait_start_sec) + float(requested_gait))
        coverage['gait_start_source'] = gait_start_src
        tail_truncation = coverage['gait_tail_truncation_sec']

    classification = _classify_termination(
        finish_reason, requested_gait, accepted_span, has_dataset, tail_truncation)

    checks = {}
    checks['dataset_contract_v1_0_1'] = {
        'pass': bool(gv_report.get('valid')), 'detail': {
            'manifest_ok': gv_report.get('manifest_ok'),
            'samples_ok': gv_report.get('samples_ok'),
            'errors': gv_report.get('errors', [])[:5]}}
    checks['sample_count_gt_zero'] = {
        'pass': info['samples'] > 0, 'detail': info['samples']}
    checks['sim_clock_monotonic'] = {
        'pass': bool(info['timestamp_monotonic'] and info['samples'] > 0),
        'detail': {'monotonic': info['timestamp_monotonic'],
                   'malformed_lines': info['malformed_lines'][:5]}}
    checks['joint_state_present_finite'] = {
        'pass': per_signal['joint_state']['pass'],
        'detail': {k: per_signal['joint_state'][k]
                   for k in ('n_ok', 'n_bad', 'examples')}}
    checks['command_present_finite'] = {
        'pass': per_signal['command']['pass'],
        'detail': {k: per_signal['command'][k]
                   for k in ('n_ok', 'n_bad', 'examples')}}
    checks['imu_present_finite'] = {
        'pass': per_signal['imu']['pass'],
        'detail': {k: per_signal['imu'][k] for k in ('n_ok', 'n_bad', 'examples')}}
    checks['contact_present'] = {
        'pass': per_signal['contact']['pass'],
        'detail': {k: per_signal['contact'][k]
                   for k in ('n_ok', 'n_bad', 'examples')}}
    checks['body_pose_present_finite'] = {
        'pass': body_pose['pass'],
        'detail': {'count': body_pose['count'], 'examples': body_pose['examples']}}
    checks['completeness'] = {
        'pass': classification == 'normal_completion',
        'detail': {'termination': classification, 'finish_reason': finish_reason,
                   'requested_gait_duration_sec': requested_gait,
                   'accepted_sample_span_sec': accepted_span,
                   'gait_tail_truncation_sec': tail_truncation,
                   'gait_relative_coverage': (coverage or {}).get(
                       'gait_relative_coverage')}}

    data_ok = all(c['pass'] for c in checks.values())
    non_completeness_fail = any(
        not c['pass'] for name, c in checks.items() if name != 'completeness')

    # Failure classification (§14). Only two classes are integrity-relevant.
    if not has_dataset or info['samples'] == 0:
        failure_class = 'infrastructure_failure'
    elif not data_ok:
        failure_class = ('infrastructure_failure' if not non_completeness_fail
                         else 'data_integrity_failure')
    else:
        failure_class = None

    verdict = {
        'artifact': 'Phase A episode integrity',
        'episode_id': episode_id or os.path.basename(episode_dir),
        'episode_integrity': 'PASS' if data_ok else 'FAIL',
        'failure_class': failure_class,
        'termination': classification,
        'duration': {
            'requested_gait_duration_sec': requested_gait,
            'record_window_duration_sec': record_window,
            'execution_duration_sec': execution_duration,
            'execution_duration_source': ('trace/clock_trace.jsonl first->last sim time'
                                          if execution_duration is not None else
                                          'unavailable:no_clock_trace'),
            'accepted_sample_span_sec': accepted_span,
            # DEPRECATED alias of accepted_sample_span_sec; NOT execution duration.
            'actual_duration_sec_LEGACY': accepted_span,
            'legacy_note': ("actual_duration_sec/duration_sec are deprecated aliases "
                            "of accepted_sample_span_sec and were ambiguous; use the "
                            "explicit *_sec fields above"),
        },
        'termination_detail': {
            'finish_reason': finish_reason,
            'requested_gait_duration_sec': requested_gait,
            'accepted_sample_span_sec': accepted_span,
            'actual_duration_sec': accepted_span,  # DEPRECATED alias (compat)
            'gait_tail_truncation_sec': tail_truncation},
        'coverage': coverage,
        'sample_count': info['samples'],
        'signals': signals,
        'checks': checks,
        'gait_outcome': _gait_outcome(trace_dir),
        # Reserved for later phases; intentionally NOT computed here.
        'reserved': {'measurement_quality': None, 'contract_status': None},
    }
    return verdict


def format_smoke_block(verdict, seed=None, requested_duration=None):
    """Render the Phase A smoke summary block (explicit duration semantics)."""
    d = verdict.get('duration', {})
    cov = verdict.get('coverage') or {}

    def val(x, fmt='%s'):
        return fmt % x if x is not None else None

    lines = [
        'EPISODE_ID:                 %s' % verdict.get('episode_id'),
        'SEED:                       %s' % seed,
        'REQUESTED_GAIT_DURATION_SEC: %s' % val(
            d.get('requested_gait_duration_sec', requested_duration)),
        'RECORD_WINDOW_DURATION_SEC:  %s' % val(d.get('record_window_duration_sec')),
        'EXECUTION_DURATION_SEC:      %s' % val(
            d.get('execution_duration_sec'), '%.3f'),
        'ACCEPTED_SAMPLE_SPAN_SEC:    %s' % val(
            d.get('accepted_sample_span_sec'), '%.3f'),
        'GAIT_RELATIVE_COVERAGE:      %s' % val(
            cov.get('gait_relative_coverage'), '%.4f'),
        'GAIT_HEAD_GAP_SEC:           %s' % val(cov.get('gait_head_gap_sec'), '%.3f'),
        'GAIT_TAIL_TRUNCATION_SEC:    %s' % val(
            cov.get('gait_tail_truncation_sec'), '%.3f'),
        'COVERAGE_STATUS:             %s' % cov.get('coverage_status'),
        'SAMPLE_COUNT:                %s' % verdict.get('sample_count'),
        '',
    ]
    for signal in SIGNALS:
        lines.append('%s: %s' % (signal.upper(), verdict['signals'].get(signal)))
    lines.append('')
    lines.append('EPISODE_INTEGRITY:   %s' % verdict.get('episode_integrity'))
    if verdict.get('failure_class'):
        lines.append('FAILURE_CLASS:       %s' % verdict['failure_class'])
    lines.append('TERMINATION:         %s' % verdict.get('termination'))
    return '\n'.join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description='Phase A episode integrity validator')
    ap.add_argument('episode_dir')
    ap.add_argument('--requested-duration', type=float, default=None)
    ap.add_argument('--gait-start', type=float, default=None)
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args(argv)
    verdict = evaluate(args.episode_dir, requested_duration_sec=args.requested_duration,
                       gait_start_sec=args.gait_start)
    text = json.dumps(verdict, indent=2, sort_keys=True, allow_nan=False)
    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as fh:
            fh.write(text)
    print(text)
    return 0 if verdict['episode_integrity'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
