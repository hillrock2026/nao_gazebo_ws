#!/usr/bin/env python3
"""GLM-5.3 review remediation fixtures (offline, read-only).

Covers the remediation items that are pure logic/state-machine tests:
  * R2/R3 frozen-extension sidecar state machine (pristine / extended / missing
    backup / mismatched backup / complete-vs-legacy sidecar, no-downgrade);
  * R7 flat + wrapper unknown gate-key rejection (symmetric);
  * R9 diff classification state machine (all branches reachable);
  * R10 unknown/malformed gait_phase_mode values;
  * R11 structured Gazebo/colcon capture;
  * R1 erratum formalisation in the report.

Usage: python3 tools/test_p0a_review_remediation.py [--json-out F]
"""

import argparse
import copy
import hashlib
import io
import json
import os
import re
import shutil
import sys
import tempfile
from contextlib import redirect_stderr

_TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(_TOOLS)
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import p0a_phase_analysis as PA  # noqa: E402
import p0a_seeddiag_replay as SR  # noqa: E402
import p0a_frozen_extension as FE  # noqa: E402
import p0a_env_capture as EC  # noqa: E402
import p0a_output_guard as OG  # noqa: E402

GATES = {'min_forward': 0.08, 'max_lateral_ratio': 0.5, 'max_yaw_deg': 30.0,
         'max_tilt_deg': 35.0, 'min_duration': 8.0, 'max_missing': 1.0,
         'max_path_over_net': 8.0, 'min_contact_alt': 3,
         'min_support_ground': 0.80, 'min_ds_both_ground': 0.50,
         'min_airborne': 0.40, 'min_clearance': 0.002,
         'max_support_drift_m': 0.010, 'epsilon_sec': 1e-6}
REPORT = os.path.join(WS, 'docs', 'task_p0a_gait_search', 'p1bd_repair',
                      'P1BD_repair_report.md')
RESULTS = []


def check(name, cond, detail=None):
    RESULTS.append({'name': name, 'pass': bool(cond), 'detail': detail})
    print('  [%s] %s%s' % ('PASS' if cond else 'FAIL', name,
                           '' if cond else ' :: %r' % (detail,)))


def sha256_text(text):
    return hashlib.sha256(text.encode()).hexdigest()


def _write(path, obj):
    with open(path, 'w', encoding='utf-8') as fh:
        json.dump(obj, fh, indent=2, sort_keys=True)


DIFF_OK = {'verdict': 'PASS', 'n_semantic_diffs': 0,
           'n_nonsemantic_diffs': 25, 'gate_changed': False,
           'verdict_changed': False}
METHOD = {'time_reference': 'absolute', 'tolerance_sec': 0.02,
          'interpolation': 'none', 'pairing': 'all_cross_pairs_within_tolerance',
          'offset_handling': 'none'}


def test_frozen_extension_states():
    tmp = tempfile.mkdtemp(prefix='p1r_fe_')
    try:
        out_dir = os.path.join(tmp, 'out')
        os.mkdir(out_dir)
        pristine = {'artifact': 'x', 'runs': {'seeddiag_r1': {'gate_pass': False}}}
        frozen_path = os.path.join(tmp, 'frozen.json')
        _write(frozen_path, pristine)
        pristine_text = open(frozen_path).read()
        p_sha = sha256_text(pristine_text)

        # pristine -> backup created, complete sidecar, frozen untouched
        sidecar, states = FE.ensure_sidecar(out_dir, frozen_path, DIFF_OK,
                                            METHOD, now='T0')
        check('pristine_creates_backup',
              os.path.isfile(os.path.join(out_dir, FE.PRECHANGE_NAME)))
        check('pristine_backup_is_original_bytes',
              open(os.path.join(out_dir, FE.PRECHANGE_NAME)).read()
              == pristine_text)
        check('pristine_complete_sidecar', FE.is_complete(sidecar), sidecar)
        check('pristine_frozen_not_modified',
              open(frozen_path).read() == pristine_text)
        check('pristine_semantic_counts',
              sidecar['n_semantic_diffs'] == 0
              and sidecar['values_changed'] is False)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # extended + valid backup
    tmp = tempfile.mkdtemp(prefix='p1r_fe2_')
    try:
        out_dir = os.path.join(tmp, 'out')
        os.mkdir(out_dir)
        pristine_text = '{"runs": {"seeddiag_r1": {"gate_pass": false}}}\n'
        p_sha = sha256_text(pristine_text)
        with open(os.path.join(out_dir, FE.PRECHANGE_NAME), 'w') as fh:
            fh.write(pristine_text)
        extended = {'runs': {'seeddiag_r1': {'gate_pass': False}},
                    'aligned_body_pose_method': METHOD,
                    '_stage0_schema_extension': {
                        'field': 'aligned_body_pose_method',
                        'prechange_sha256': p_sha, 'values_changed': False}}
        frozen_path = os.path.join(tmp, 'frozen.json')
        _write(frozen_path, extended)
        frozen_text = open(frozen_path).read()
        sidecar, states = FE.ensure_sidecar(out_dir, frozen_path, DIFF_OK,
                                            METHOD, now='T1')
        check('extended_valid_backup_state',
              states['frozen_state'] == 'historically_extended'
              and states['backup_state'] == 'valid'
              and states['frozen_modified'] is False, states)
        check('extended_prechange_recorded',
              sidecar['prechange_sha256'] == p_sha, sidecar)
        check('extended_postchange_recorded',
              sidecar['postchange_sha256'] == sha256_text(frozen_text))
        check('extended_frozen_not_modified',
              open(frozen_path).read() == frozen_text)
        # second run must not downgrade the complete sidecar
        side1 = open(os.path.join(out_dir, FE.SIDECAR_NAME)).read()
        sidecar2, states2 = FE.ensure_sidecar(out_dir, frozen_path, DIFF_OK,
                                              METHOD, now='T2')
        check('complete_sidecar_not_downgraded',
              open(os.path.join(out_dir, FE.SIDECAR_NAME)).read() == side1
              and states2['wrote_sidecar'] is False, states2)
        check('complete_sidecar_keeps_fields',
              FE.is_complete(sidecar2))

        # F-4: complete sidecar must re-verify postchange against the frozen file
        side_bytes = open(os.path.join(out_dir, FE.SIDECAR_NAME)).read()
        replaced = dict(extended)
        replaced['unexpected_extra'] = 1
        _write(frozen_path, replaced)
        replaced_text = open(frozen_path).read()
        try:
            FE.ensure_sidecar(out_dir, frozen_path, DIFF_OK, METHOD)
            check('complete_sidecar_postchange_mismatch_fails', False)
        except FE.ExtensionError as exc:
            check('complete_sidecar_postchange_mismatch_fails',
                  'postchange hash mismatch' in str(exc)
                  and 'recorded=' in str(exc) and 'actual=' in str(exc),
                  str(exc))
        check('postchange_mismatch_sidecar_unchanged',
              open(os.path.join(out_dir, FE.SIDECAR_NAME)).read() == side_bytes)
        check('postchange_mismatch_frozen_unchanged',
              open(frozen_path).read() == replaced_text)
        check('postchange_mismatch_zero_write',
              FE.detect_sidecar_state(os.path.join(out_dir, FE.SIDECAR_NAME))
              == 'complete')
        with open(frozen_path, 'w') as fh:
            fh.write(frozen_text)  # restore for the remaining cases

        # G-3: complete sidecar with postchange_sha256=null is self-contradictory
        null_side = json.loads(side_bytes)
        null_side['postchange_sha256'] = None
        _write(os.path.join(out_dir, FE.SIDECAR_NAME), null_side)
        null_side_bytes = open(os.path.join(out_dir, FE.SIDECAR_NAME)).read()
        frozen_now = open(frozen_path).read()
        try:
            FE.ensure_sidecar(out_dir, frozen_path, DIFF_OK, METHOD)
            check('complete_sidecar_null_postchange_fails', False)
        except FE.ExtensionError as exc:
            check('complete_sidecar_null_postchange_fails',
                  'missing postchange_sha256' in str(exc), str(exc))
        check('null_postchange_sidecar_unchanged',
              open(os.path.join(out_dir, FE.SIDECAR_NAME)).read()
              == null_side_bytes)
        check('null_postchange_frozen_unchanged',
              open(frozen_path).read() == frozen_now)
        check('null_postchange_zero_write',
              FE.detect_sidecar_state(os.path.join(out_dir, FE.SIDECAR_NAME))
              == 'complete')

        # legacy sidecar is upgraded to complete
        _write(os.path.join(out_dir, FE.SIDECAR_NAME),
               {'already_extended': True, 'field_added': 'x'})
        check('legacy_detected',
              FE.detect_sidecar_state(os.path.join(out_dir, FE.SIDECAR_NAME))
              == 'legacy')
        sidecar3, _ = FE.ensure_sidecar(out_dir, frozen_path, DIFF_OK,
                                        METHOD, now='T3')
        check('legacy_upgraded_complete', FE.is_complete(sidecar3))

        # missing backup -> hard error
        os.remove(os.path.join(out_dir, FE.PRECHANGE_NAME))
        try:
            FE.ensure_sidecar(out_dir, frozen_path, DIFF_OK, METHOD)
            check('missing_backup_fails', False)
        except FE.ExtensionError as exc:
            check('missing_backup_fails', 'backup missing' in str(exc), str(exc))
        check('missing_backup_frozen_unchanged',
              open(frozen_path).read() == frozen_text)

        # mismatched backup -> hard error
        with open(os.path.join(out_dir, FE.PRECHANGE_NAME), 'w') as fh:
            fh.write('{"different": true}\n')
        try:
            FE.ensure_sidecar(out_dir, frozen_path, DIFF_OK, METHOD)
            check('mismatched_backup_fails', False)
        except FE.ExtensionError as exc:
            check('mismatched_backup_fails', 'mismatch' in str(exc), str(exc))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    # real repo: this tool must NOT modify the frozen file
    real_frozen = os.path.join(WS, 'docs', 'task_p0a_gait_search',
                               'p1r_support_swing',
                               'seed_repeatability_result.json')
    before = FE.sha256_file(real_frozen)
    check('real_frozen_readable', os.path.isfile(real_frozen))
    # sanity: no writer in the deliverables tool references FROZEN_RESULT open(...,'w')
    src = open(os.path.join(_TOOLS, 'p0a_stage0_deliverables.py')).read()
    check('deliverables_has_no_frozen_write',
          "FROZEN_RESULT, 'w'" not in src and 'shutil.copyfile(FROZEN_RESULT' not in src)
    check('real_frozen_unchanged_after_reads', FE.sha256_file(real_frozen) == before)


def test_gate_unknown_keys():
    bad_flat = dict(GATES)
    bad_flat['min_foward'] = 0.1
    try:
        PA.normalize_gates(bad_flat, source='flat-test')
        check('flat_unknown_key_rejected', False)
    except PA.GateContractError as exc:
        check('flat_unknown_key_rejected',
              'min_foward' in str(exc) and 'flat' in str(exc), str(exc))
    bad_wrap = {'version': 'x', 'hard_gates': dict(GATES, bogus=1)}
    try:
        PA.normalize_gates(bad_wrap, source='wrap-test')
        check('wrapper_inner_unknown_key_rejected', False)
    except PA.GateContractError as exc:
        check('wrapper_inner_unknown_key_rejected', 'bogus' in str(exc), str(exc))
    good_wrap = {'version': 'x', 'hard_gates': dict(GATES)}
    norm_wrap = PA.normalize_gates(good_wrap)
    check('wrapper_good_ok',
          all(norm_wrap[k] == GATES[k] for k in PA.REQUIRED_HARD_GATES)
          and all(k not in norm_wrap for k in PA.GATE_META_KEYS))
    # F-2: _gate_form metadata never bypasses validation
    try:
        PA.normalize_gates({'_gate_form': 'flat', 'min_forward': 0.08},
                           source='form-only')
        check('gate_form_only_missing_keys_fails', False)
    except PA.GateContractError:
        check('gate_form_only_missing_keys_fails', True)
    try:
        PA.normalize_gates(dict(GATES, _gate_form='flat', typo=1),
                           source='form-typo')
        check('gate_form_typo_still_rejected', False)
    except PA.GateContractError as exc:
        check('gate_form_typo_still_rejected', 'typo' in str(exc), str(exc))


def test_diff_state_machine():
    check('sm_no_differences',
          SR.compare({'a': 1}, {'a': 1})['verdict'] == 'PASS')
    check('sm_nonsemantic_added',
          SR.compare({'a': 1}, {'a': 1, 'tool': 'x'})['verdict'] == 'PASS')
    d = SR.compare({'generated_at': 'x'}, {'generated_at': 'y'})
    check('sm_nonsemantic_changed',
          d['verdict'] == 'PASS_WITH_ERRATUM'
          and d['branch'] == 'nonsemantic_metadata_changed', d['branch'])
    d = SR.compare({'generated_at': 'x'}, {})
    check('sm_nonsemantic_removed',
          d['verdict'] == 'PASS_WITH_ERRATUM', d['verdict'])
    d = SR.compare({'n_rows': 1}, {'n_rows': 2})
    check('sm_semantic_changed',
          d['verdict'] == 'BLOCKED'
          and d['branch'] == 'semantic_value_changed', d['branch'])
    d = SR.compare({'forward_displacement_m': 0.1},
                   {'forward_displacement_m': 0.2})
    check('sm_gate_leaf_changed',
          d['verdict'] == 'BLOCKED' and d['gate_changed'] is True, d)
    d = SR.compare({'runs': {'seeddiag_r1': {'gate_pass': False}}},
                   {'runs': {'seeddiag_r1': {'gate_pass': True}}})
    check('sm_gate_changed',
          d['verdict'] == 'BLOCKED' and d['gate_changed'] is True, d)
    d = SR.compare({'stop': 'A'}, {'stop': 'B'})
    check('sm_verdict_changed',
          d['verdict'] == 'BLOCKED' and d['verdict_changed'] is True, d)
    d = SR.compare({'zzz_unknown': 1}, {'zzz_unknown': 2})
    check('sm_unknown_path_blocked', d['verdict'] == 'BLOCKED', d['verdict'])
    check('sm_tiny_float_equal',
          SR.compare({'x': 1.0}, {'x': 1.0 + 1e-15})['verdict'] == 'PASS')
    # dict key order is a pure format difference -> no diff
    a, b = {}, {}
    for k in ('b', 'a', 'c'):
        a[k] = 1
    for k in ('c', 'a', 'b'):
        b[k] = 1
    check('sm_key_order_ignored',
          SR.compare(a, b)['verdict'] == 'PASS')


def test_mode_strings():
    def outcome(cfg):
        try:
            return PA.resolve_phase_mode(cfg)
        except PA.PhaseAnalysisError as exc:
            return ('error', str(exc))
    check('mode_bogus', outcome({'gait_phase_mode': 'bogus'})[0] == 'error')
    check('mode_case', outcome({'gait_phase_mode': 'LEGACY'})[0] == 'error')
    check('mode_ws', outcome({'gait_phase_mode': ' legacy '})[0] == 'error')
    check('mode_null', outcome({'gait_phase_mode': None})[0] == 'error')
    check('mode_empty', outcome({'gait_phase_mode': ''})[0] == 'error')
    check('mode_number', outcome({'gait_phase_mode': 1})[0] == 'error')
    check('mode_list', outcome({'gait_phase_mode': ['legacy']})[0] == 'error')
    check('mode_object', outcome({'gait_phase_mode': {'m': 'legacy'}})[0]
          == 'error')
    check('mode_legacy_missing',
          outcome({'gait_method': 'walk'}) == ('legacy',
                                               'historical_legacy_schema'))
    check('mode_nonlegacy_missing',
          outcome({'duration_sec': 1.0})[0] == 'error')
    check('mode_explicit_ok',
          outcome({'gait_phase_mode': 'explicit_support_swing'})
          == ('explicit_support_swing', 'explicit'))


def test_gazebo_capture():
    missing = EC.probe_tool([['definitely_not_a_real_tool_xyz', '--version']])
    check('probe_missing_structured',
          missing['available'] is False and missing['version'] is None
          and missing['attempts'], missing)
    failing = EC.probe_tool([['false']])
    check('probe_failure_structured',
          failing['available'] is True and failing['version'] is None
          and 'exit_code' in failing['attempts'][0], failing)
    data = EC.capture()
    check('capture_has_gazebo', 'gazebo' in data
          and 'available' in data['gazebo'], data.get('gazebo'))
    check('capture_colcon_structured',
          isinstance(data['ros']['colcon'], dict)
          and 'available' in data['ros']['colcon'], data['ros'].get('colcon'))
    check('capture_catkin_structured',
          isinstance(data['ros']['catkin'], dict)
          and 'available' in data['ros']['catkin'])


# --- G-1 / G-2 ---
R3_AUTHORITATIVE_12 = {
    # Phase A §9: the runner's config-merge order was fixed (EXPERIMENT_DEFAULTS
    # is now applied before user overrides), so its authoritative hash moved from
    # 52c790113d80 to 00aef7129d65.
    'tools/run_p0a_gait_candidate.py': '00aef7129d65',
    'tools/test_p0a_cli_contracts.py': 'fd9edae88175',
    'analysis_source_hashes.json': '6dc49b5c112d',
    'test_file_hashes.json': '34492a71b569',
    'offline_results_full.txt': '999e8a6ce09b',
}
P1BD = os.path.join(WS, 'docs', 'task_p0a_gait_search', 'p1bd_repair')
OFFLINE_LOG = os.path.join(P1BD, 'offline_results_full.txt')


def _resolve_report_path(fname):
    if fname.startswith(('tools/', 'src/', 'docs/')):
        return os.path.join(WS, fname)
    return os.path.join(P1BD, fname)


def _parse_hash_table(section):
    rows = {}
    for line in section.splitlines():
        s = line.strip()
        if not s.startswith('|') or not s.endswith('|'):
            continue
        cells = [c.strip().strip('`').strip()
                 for c in s.strip('|').split('|')]
        if len(cells) != 3:
            continue
        if cells[0] == 'file' or cells[0].startswith('-'):
            continue
        rows[cells[0]] = {'old': cells[1], 'new': cells[2]}
    return rows


def _section(text, heading, next_heading):
    start = text.find(heading)
    if start < 0:
        return ''
    end = text.find(next_heading, start + 1) if next_heading else -1
    return text[start:end] if end > 0 else text[start:]


def _header_previous_hash(path):
    with open(path, encoding='utf-8') as fh:
        for line in fh:
            if line.startswith('previous_output_sha256:'):
                return line.split(':', 1)[1].strip()
    return None


def _hash_subsection(section):
    m = re.search(r'^### Hash changes\s*$', section, re.MULTILINE)
    if not m:
        return ''
    rest = section[m.end():]
    ends = [x for x in (rest.find('\n### '), rest.find('\n## ')) if x > 0]
    return rest[:min(ends)] if ends else rest


def test_report_hash_tables():
    if not os.path.isfile(REPORT):
        check('report_hash_tables_present', False, REPORT)
        return
    text = open(REPORT, encoding='utf-8').read()
    r3 = _section(text, '## Round-3 minimal remediation',
                  '## Round-4 minimal remediation')
    r4 = _section(text, '## Round-4 minimal remediation', None)
    r3_rows = _parse_hash_table(_hash_subsection(r3))
    r4_rows = _parse_hash_table(_hash_subsection(r4))
    # G-1: the 5 previously-stale Round-3 rows now equal the authoritative value
    for fname, auth12 in R3_AUTHORITATIVE_12.items():
        row = r3_rows.get(fname)
        check('r3_hash_row_%s' % fname.replace('/', '_'),
              row is not None and row['new'].startswith(auth12),
              (row, auth12))
    # G-1: every measurable Round-4 row new == current file sha256[:12].
    # ``offline_results_full.txt`` is rewritten by the gate that runs this test
    # (finish-time log), so its row is recorded in the r4 ledger but not asserted
    # against the live file here; the report itself is a placeholder row.
    self_referential = {'offline_results_full.txt', 'P1BD_repair_report.md'}
    for fname, row in r4_rows.items():
        if fname in self_referential:
            continue
        new = row['new'].replace('…', '').strip()
        if not re.match(r'^[0-9a-f]{12}$', new):
            continue
        path = _resolve_report_path(fname)
        if not os.path.isfile(path):
            check('r4_hash_row_exists_%s' % fname, False, path)
            continue
        actual = hashlib.sha256(open(path, 'rb').read()).hexdigest()[:12]
        check('r4_hash_row_%s' % fname.replace('/', '_'),
              new == actual, {'report': new, 'actual': actual, 'file': fname})


def test_r3_offline_oldhash():
    r3_path = os.path.join(P1BD, 'remediation_summary_r3.json')
    r4_path = os.path.join(P1BD, 'remediation_summary_r4.json')
    r3 = json.load(open(r3_path))
    entry = [c for c in r3['changed_files']
             if c['name'].endswith('offline_results_full.txt')][0]
    check('r3_offline_old_not_null', entry['old_sha256'] is not None, entry)
    check('r3_offline_old_value',
          entry['old_sha256']
          == 'f41d16f0cee40cfcf5b7769a2ca55bde6c85015a7a0858375233a8e8992dbbda',
          entry['old_sha256'])
    hdr = _header_previous_hash(OFFLINE_LOG)
    check('r3_offline_note_sources_header',
          'previous_output_sha256' in entry.get('note', ''), entry.get('note'))
    # the round-4 entry (if present) must track the current header, no null.
    if os.path.isfile(r4_path):
        r4 = json.load(open(r4_path))
        e4 = [c for c in r4['changed_files']
              if c['name'].endswith('offline_results_full.txt')][0]
        check('r4_offline_old_not_null', e4['old_sha256'] is not None)
        # stable across a gate re-run: either the current file is the round-4
        # output (header == r4 old) or it was regenerated from it (header == r4 new)
        check('r4_offline_old_tracks_header',
              hdr in (e4['old_sha256'], e4['new_sha256']),
              {'r4_old': e4['old_sha256'], 'r4_new': e4['new_sha256'],
               'header': hdr})
        # three-way: report Round-4 table row == r4 summary new (both recorded,
        # stable); the live file matches unless it was just regenerated by a gate
        # re-run (then the header chain proves it)
        text = open(REPORT, encoding='utf-8').read()
        r4_sec = _section(text, '## Round-4 minimal remediation', None)
        rows = _parse_hash_table(_hash_subsection(r4_sec))
        row = rows.get('offline_results_full.txt')
        actual12 = hashlib.sha256(open(OFFLINE_LOG, 'rb').read()).hexdigest()[:12]
        check('r4_offline_table_matches_summary',
              row is not None
              and row['new'].replace('…', '').strip() == e4['new_sha256'][:12],
              {'table': row['new'] if row else None,
               'summary': e4['new_sha256'][:12]})
        check('r4_offline_actual_matches_summary',
              actual12 == e4['new_sha256'][:12] or hdr == e4['new_sha256'],
              {'actual': actual12, 'summary': e4['new_sha256'][:12],
               'header': hdr})


def test_report_erratum():
    if not os.path.isfile(REPORT):
        check('report_present', False, REPORT)
        return
    text = open(REPORT, encoding='utf-8').read()
    check('report_present', bool(text))
    check('erratum_section_present',
          'Erratum' in text and 'in-place' in text.lower())
    for needle in ('pre-change SHA-256', 'post-change SHA-256',
                   'values_changed', 'aligned_body_pose_method',
                   '_stage0_schema_extension', 'sidecar',
                   'independent historical freeze', 'PASS_WITH_ERRATUM',
                   '允许进入 P1-BD'):
        check('erratum_mentions_%s' % needle.replace(' ', '_'),
              needle in text, needle)
    check('erratum_negative_no_longer_claims',
          'No erratum is required' not in text
          and '无需 erratum' not in text)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--json-out', default=None)
    args = ap.parse_args()
    print('GLM-5.3 review remediation fixtures')
    test_frozen_extension_states()
    test_gate_unknown_keys()
    test_diff_state_machine()
    test_mode_strings()
    test_gazebo_capture()
    test_report_hash_tables()
    test_r3_offline_oldhash()
    test_report_erratum()
    passed = sum(1 for r in RESULTS if r['pass'])
    total = len(RESULTS)
    print('\nreview-remediation fixtures: %d/%d passed' % (passed, total))
    if args.json_out:
        with open(args.json_out, 'w') as fh:
            json.dump({'passed': passed, 'total': total, 'results': RESULTS},
                      fh, indent=2, sort_keys=True)
    return 0 if passed == total else 1


if __name__ == '__main__':
    sys.exit(main())
