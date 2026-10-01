#!/usr/bin/env python3
"""P0-a P1-R seed-diagnostic replay / regenerate / value-by-value compare.

Deterministically recomputes the ``seed_repeatability_result.json`` content from
the frozen raw traces of the two serial seed-diagnostic runs and compares it,
field by field, against the frozen evidence.  No Gazebo, no ROS, read-only.

Subcommands (also available as ``replay``):

  generate  rebuild the repeatability result from raw traces;
  compare   field-level diff of an old vs new result (float rules, sorting,
            ignored-field reasons, gate/verdict impact);
  replay    generate + compare in one step.

Usage:
  python3 tools/p0a_seeddiag_replay.py generate --runs-root DIR --out NEW.json
  python3 tools/p0a_seeddiag_replay.py compare --old OLD.json --new NEW.json \
      --out DIFF.json
  python3 tools/p0a_seeddiag_replay.py replay --runs-root DIR \
      --frozen OLD.json --out NEW.json --diff-out DIFF.json
"""

import argparse
import hashlib
import json
import math
import os
import re
import subprocess
import sys
import time

_TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(_TOOLS)
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)
import p0a_phase_analysis as PA  # noqa: E402
import p0a_aligned_pose as AP  # noqa: E402
import p0a_output_guard as OG  # noqa: E402
import nao_gait_phase as G  # noqa: E402

TOOL_VERSION = 'p0a_seeddiag_replay:1.0'
DEFAULT_RUNS = ('seeddiag_r1', 'seeddiag_r2')
FLOAT_ABS_TOL = 1e-12
FLOAT_REL_TOL = 1e-9
TRACE_FILES = ('body_pose_trace.jsonl', 'contact_trace.jsonl',
               'state_trace.jsonl')
# Metadata/provenance field names that are NOT frozen numeric evidence.  Matched
# by exact path segment (never substring), so a new/unknown field defaults to
# semantic and cannot be silently downgraded.
NONSEMANTIC_FIELD_NAMES = {
    'artifact', 'tool', 'tool_version', 'generated_at', 'environment',
    'source_hashes', 'source_hashes_meta', 'input_files', 'notes',
    'schema_extension', '_stage0_schema_extension',
    'aligned_body_pose_method', 'method_b_onset_rule',
    'run_config_sha256', 'provenance_sha256', 'manifest_derived',
    'contact_ordering', 'joint_order', 'phase_mode_source',
    'phase_mode_assumed', 'contact_diagnostics', 'review_status_before',
    'review_status_after', 'erratum_formalized', 'admission_ready',
}
# Backwards-compatible alias.
NONSEMANTIC_KEYS = NONSEMANTIC_FIELD_NAMES
GATE_LEAF_HINTS = ('gate', 'onset', 'phase_stats', 'forward_displacement_m',
                   'lateral_displacement_m', 'yaw_change_deg',
                   'torso_tilt_max_deg', 'path_over_net',
                   'missing_duration_sec', 'zero_contact_max_sec',
                   'n_state_samples', 'replicate_id', 'physics_seed',
                   'ready_sim', 'trace_truncated')
VERDICT_HINTS = ('gate_pass', 'gate_fails', 'stop')


class SeedDiagError(Exception):
    pass


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def _read_json(path):
    with open(path, 'r', encoding='utf-8') as fh:
        return json.load(fh)


def _seed_log_line(run_dir):
    path = os.path.join(run_dir, 'launch.log')
    if not os.path.isfile(path):
        return None
    with open(path, 'r', encoding='utf-8', errors='replace') as fh:
        text = fh.read()
    text = re.sub(r'\x1b\[[0-9;]*m', '', text)
    m = re.search(r'Setting seed value:\s*([0-9]+)', text)
    return 'Setting seed value: %s' % m.group(1) if m else None


def _n_state_samples(run_dir):
    path = os.path.join(run_dir, 'trace', 'state_trace.jsonl')
    with open(path, 'r', encoding='utf-8') as fh:
        return sum(1 for line in fh if line.strip())


def load_run(run_dir, gates):
    """One run's repeatability record, recomputed from the raw traces."""
    result_path = os.path.join(run_dir, 'result_p0a.json')
    prov_path = os.path.join(run_dir, 'provenance.json')
    if not os.path.isfile(result_path):
        raise SeedDiagError('result_p0a.json missing: %s' % run_dir)
    result = _read_json(result_path)
    provenance = _read_json(prov_path) if os.path.isfile(prov_path) else {}
    analysis = PA.analyse(run_dir)
    loco = analysis['locomotion']
    gate = PA.evaluate_gates(analysis, gates)
    phase_stats = {name: analysis['phase_stats'][name]['fraction']
                   for name in G.PHASE_NAMES}
    return {
        'forward_displacement_m': loco.get('forward_displacement_m'),
        'lateral_displacement_m': loco.get('lateral_displacement_m'),
        'yaw_change_deg': loco.get('yaw_change_deg'),
        'torso_tilt_max_deg': loco.get('torso_tilt_max_deg'),
        'path_over_net': loco.get('path_over_net'),
        'missing_duration_sec': loco.get('missing_duration_sec'),
        'trace_truncated': loco.get('trace_truncated'),
        'zero_contact_max_sec':
            (analysis.get('zero_contact') or {}).get('max_duration_sec'),
        'phase_stats': phase_stats,
        'method_b_onset': analysis['swing_to_support_onset_delay_sec'],
        'm0_onset': analysis['m0_legacy_contact_delay_sec'],
        'gate_fails': [k for k, v in gate['checks'].items() if not v],
        'gate_pass': gate['pass'],
        'ready_sim': result.get('ready_sim'),
        'replicate_id': result.get('replicate_id'),
        'physics_seed': result.get('physics_seed'),
        'n_state_samples': _n_state_samples(run_dir),
        'timed_out': result.get('timed_out'),
        'gz_sim_cmd_expected': provenance.get('gz_sim_cmd_expected'),
        'seed_log_line': _seed_log_line(run_dir),
        'run_config_sha256': sha256_file(os.path.join(run_dir,
                                                      'run_config.json')),
        'provenance_sha256': (sha256_file(prov_path)
                              if os.path.isfile(prov_path) else None),
    }


def byte_equality(run_a, run_b):
    out = {}
    for name in TRACE_FILES:
        pa = os.path.join(run_a, 'trace', name)
        pb = os.path.join(run_b, 'trace', name)
        ha = sha256_file(pa) if os.path.isfile(pa) else 'missing'
        hb = sha256_file(pb) if os.path.isfile(pb) else 'missing'
        out[name] = {'r1_sha256': ha, 'r2_sha256': hb,
                     'identical': ha == hb and ha != 'missing'}
    return out


def generate(runs_root, gates_path, config_label='fwdf08 + legacy',
             old_result=None):
    gates = PA.load_gates(gates_path) if gates_path else None
    r1_dir = os.path.join(runs_root, DEFAULT_RUNS[0])
    r2_dir = os.path.join(runs_root, DEFAULT_RUNS[1])
    for d in (r1_dir, r2_dir):
        if not os.path.isdir(d):
            raise SeedDiagError('run directory missing: %s' % d)
    if gates is None:
        raise SeedDiagError('a gate file is required (--gates)')
    runs = {DEFAULT_RUNS[0]: load_run(r1_dir, gates),
            DEFAULT_RUNS[1]: load_run(r2_dir, gates)}
    physics_seeds = {runs[r]['physics_seed'] for r in runs}
    physics_seed = physics_seeds.pop() if len(physics_seeds) == 1 else None
    failed = [r for r in runs if not runs[r]['gate_pass']]
    # NOTE: the historical stop label is retained verbatim.  Correcting the
    # BASELINE/INFRA naming (ISS-07) is out of Stage 0 scope and would itself be
    # a verdict-string change; it is recorded only as a note.
    stop = ('A/B NOT STARTED' if failed
            else 'BASELINE STABLE: A/B PERMITTED')
    aligned = AP.compare(r1_dir, r2_dir, time_reference='absolute',
                         tolerance=AP.DEFAULT_TOLERANCE_SEC,
                         interpolation='none')
    beq = byte_equality(r1_dir, r2_dir)
    nondeterministic = not all(v['identical'] for v in beq.values())
    if failed and nondeterministic:
        failing = (DEFAULT_RUNS[0] if not runs[DEFAULT_RUNS[0]]['gate_pass']
                   else DEFAULT_RUNS[1]).replace('seeddiag_', '')
        conclusion = ('seed wiring/acceptance confirmed (gz sim --seed + Gazebo '
                      'log); run remains nondeterministic (traces differ, %s '
                      'fails baseline qualification) -> baseline/infra NOT stable'
                      % failing)
    elif failed:
        failing = (DEFAULT_RUNS[0] if not runs[DEFAULT_RUNS[0]]['gate_pass']
                   else DEFAULT_RUNS[1]).replace('seeddiag_', '')
        conclusion = ('seed wiring/acceptance confirmed; run traces identical '
                      'but %s fails baseline qualification' % failing)
    else:
        conclusion = 'seed wiring/acceptance confirmed; baseline stable'
    manifest = {
        'run_config_sha256_r1': runs[DEFAULT_RUNS[0]]['run_config_sha256'],
        'run_config_sha256_r2': runs[DEFAULT_RUNS[1]]['run_config_sha256'],
        'provenance_sha256_r1': runs[DEFAULT_RUNS[0]]['provenance_sha256'],
        'provenance_sha256_r2': runs[DEFAULT_RUNS[1]]['provenance_sha256'],
    }
    return {
        'artifact': 'P1-R seed repeatability diagnostic',
        'tool': TOOL_VERSION,
        'generated_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'config': config_label,
        'physics_seed': physics_seed,
        'serial': True,
        'stop': ('BASELINE/INFRA BLOCKED: ' + stop if failed
                 else stop),
        'conclusion': conclusion,
        'runs': runs,
        'byte_equality': beq,
        'aligned_body_pose_max_abs_diff_m': aligned['max_abs_diff_m'],
        'aligned_body_pose_method': aligned['method'],
        'manifest_derived': manifest,
        'notes': {
            'classification': ('historical stop label kept verbatim; ISS-07 '
                               'naming correction is out of Stage 0 scope'),
            'method_b_onset_rule': analysis_rule(),
        },
    }


def analysis_rule():
    return ('Method B: first Bool contact=true of the now-support foot within '
            'one cycle after the commanded swing-end boundary '
            '(L=0.5-ds/2, R=1-ds/2); no onset inside the cycle -> n_missing')


# ----------------------------------------------------------------- comparator
def _path_segments(path):
    return [seg for seg in re.split(r'[.\[]', path) if seg]


def _is_nonsemantic(path):
    """Exact-segment match against the metadata allowlist (never substring)."""
    return any(seg in NONSEMANTIC_FIELD_NAMES for seg in _path_segments(path))


def _affects_gate(path):
    return (path in VERDICT_HINTS or any(h in path for h in GATE_LEAF_HINTS))


def _affects_verdict(path):
    return any(h in path for h in VERDICT_HINTS)


def _num_equal(a, b):
    if a == b:
        return True
    if not (isinstance(a, (int, float)) and not isinstance(a, bool)
            and isinstance(b, (int, float)) and not isinstance(b, bool)):
        return False
    if not (math.isfinite(float(a)) and math.isfinite(float(b))):
        return False
    return abs(a - b) <= max(FLOAT_ABS_TOL, FLOAT_REL_TOL * max(abs(a), abs(b)))


def _flatten(obj, prefix=''):
    out = {}
    if isinstance(obj, dict):
        for k in sorted(obj):
            out.update(_flatten(obj[k], '%s.%s' % (prefix, k) if prefix else k))
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            out.update(_flatten(v, '%s[%d]' % (prefix, i)))
    else:
        out[prefix] = obj
    return out


def compare(old, new):
    fo, fn = _flatten(old), _flatten(new)
    keys = sorted(set(fo) | set(fn))
    diffs = []
    for k in keys:
        if k not in fo:
            diffs.append({'path': k, 'old': None, 'new': fn[k],
                          'kind': 'added', 'nonsemantic': _is_nonsemantic(k),
                          'rule': 'field added in new result'})
        elif k not in fn:
            diffs.append({'path': k, 'old': fo[k], 'new': None,
                          'kind': 'removed', 'nonsemantic': _is_nonsemantic(k),
                          'rule': 'field missing in new result'})
        elif not _num_equal(fo[k], fn[k]):
            rule = ('float abs<=%g or rel<=%g' % (FLOAT_ABS_TOL, FLOAT_REL_TOL)
                    if isinstance(fo[k], float) or isinstance(fn[k], float)
                    else 'exact')
            diffs.append({'path': k, 'old': fo[k], 'new': fn[k],
                          'kind': 'changed', 'nonsemantic': _is_nonsemantic(k),
                          'rule': rule})
    for d in diffs:
        # unknown/unmatched paths are semantic by default: never downgraded.
        d['classification'] = ('nonsemantic_metadata' if d['nonsemantic']
                               else 'semantic')
        d['affects_gate'] = _affects_gate(d['path']) and not d['nonsemantic']
        d['affects_verdict'] = _affects_verdict(d['path']) and not d['nonsemantic']
    semantic = [d for d in diffs if not d['nonsemantic']]
    gate_changes = [d for d in semantic if d['affects_gate']]
    verdict_changes = [d for d in semantic if d['affects_verdict']]
    nonsemantic_value = [d for d in diffs if d['nonsemantic']
                         and d['kind'] in ('changed', 'removed')]
    added = [d for d in diffs if d['kind'] == 'added' and d['nonsemantic']]
    # explicit verdict state machine (unknown paths -> semantic -> BLOCKED)
    if verdict_changes or gate_changes:
        verdict = 'BLOCKED'
        branch = 'gate_or_verdict_changed'
        reason = ('gate/verdict-affecting difference: %s'
                  % ', '.join(d['path'] for d in (verdict_changes
                                                  or gate_changes)))
    elif semantic:
        verdict = 'BLOCKED'
        branch = 'semantic_value_changed'
        reason = 'semantic difference(s): %s' % ', '.join(
            d['path'] for d in semantic)
    elif nonsemantic_value:
        verdict = 'PASS_WITH_ERRATUM'
        branch = 'nonsemantic_metadata_changed'
        reason = ('only non-semantic metadata changed (added=%d changed/removed=%d)'
                  % (len(added), len(nonsemantic_value)))
    else:
        verdict = 'PASS'
        branch = 'no_differences'
        reason = ('all frozen fields reproduced exactly'
                  + ('' if not added else
                     '; %d non-semantic metadata field(s) added '
                     '(schema extension)' % len(added)))
    return {
        'artifact': 'P0-a seed-diagnostic replay diff',
        'tool': TOOL_VERSION,
        'generated_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'classification_state_machine': {
            'no_differences': 'PASS',
            'nonsemantic_added_changed_removed': 'PASS_WITH_ERRATUM',
            'semantic_value_changed': 'BLOCKED',
            'gate_changed': 'BLOCKED',
            'verdict_changed': 'BLOCKED',
            'unknown_path_policy': 'unmatched paths are semantic -> BLOCKED',
            'branch_taken': branch,
        },
        'comparison_rules': {
            'float_abs_tol': FLOAT_ABS_TOL,
            'float_rel_tol': FLOAT_REL_TOL,
            'sorting': 'paths sorted lexicographically; lists compared by index',
            'key_order': 'dict key order is irrelevant (flatten sorts keys)',
            'ignored_field_names': sorted(NONSEMANTIC_FIELD_NAMES),
            'ignored_fields': {k: 'provenance/metadata only (not frozen '
                                  'numeric evidence)'
                               for k in sorted(NONSEMANTIC_FIELD_NAMES)},
            'added_fields': 'non-semantic schema extensions are ignored and '
                            'never treated as a value change',
            'unknown_fields': 'any path whose segment is not in '
                              'ignored_field_names is semantic',
        },
        'n_fields_old': len(fo),
        'n_fields_new': len(fn),
        'diffs': diffs,
        'n_semantic_diffs': len(semantic),
        'n_nonsemantic_diffs': len([d for d in diffs if d['nonsemantic']]),
        'n_nonsemantic_added': len(added),
        'n_nonsemantic_value_diffs': len(nonsemantic_value),
        'gate_changed': bool(gate_changes),
        'verdict_changed': bool(verdict_changes),
        'erratum': ({'applies': bool(nonsemantic_value),
                     'added_schema_extensions': [d['path'] for d in added]}
                    if (nonsemantic_value or added) else None),
        'branch': branch,
        'verdict': verdict,
        'reason': reason,
    }


def cmd_generate(args):
    result = generate(args.runs_root, args.gates, args.config_label)
    try:
        OG.write_json(args.out, result, allow_existing=False)
    except OG.OutputPathError as exc:
        sys.stderr.write('p0a_seeddiag_replay: error: %s\n' % exc)
        return OG.EXIT_OUTPUT_PATH
    print('wrote %s' % args.out)
    return 0


def cmd_compare(args):
    diff = compare(_read_json(args.old), _read_json(args.new))
    try:
        OG.write_json(args.out, diff, allow_existing=False)
    except OG.OutputPathError as exc:
        sys.stderr.write('p0a_seeddiag_replay: error: %s\n' % exc)
        return OG.EXIT_OUTPUT_PATH
    print(json.dumps({'verdict': diff['verdict'], 'reason': diff['reason'],
                      'n_semantic_diffs': diff['n_semantic_diffs'],
                      'n_nonsemantic_diffs': diff['n_nonsemantic_diffs'],
                      'branch': diff['branch']},
                     indent=2, sort_keys=True))
    return 0 if diff['verdict'] != 'BLOCKED' else 1


def cmd_replay(args):
    result = generate(args.runs_root, args.gates, args.config_label)
    diff = compare(_read_json(args.frozen), result)
    try:
        OG.write_json(args.out, result, allow_existing=False)
        OG.write_json(args.diff_out, diff, allow_existing=False)
    except OG.OutputPathError as exc:
        sys.stderr.write('p0a_seeddiag_replay: error: %s\n' % exc)
        return OG.EXIT_OUTPUT_PATH
    print(json.dumps({'wrote': args.out, 'diff': args.diff_out,
                      'verdict': diff['verdict'], 'reason': diff['reason'],
                      'n_semantic_diffs': diff['n_semantic_diffs'],
                      'n_nonsemantic_diffs': diff['n_nonsemantic_diffs'],
                      'branch': diff['branch']},
                     indent=2, sort_keys=True))
    return 0 if diff['verdict'] != 'BLOCKED' else 1


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__)
    sub = ap.add_subparsers(dest='cmd', required=True)
    g = sub.add_parser('generate')
    g.add_argument('--runs-root', required=True)
    g.add_argument('--gates', required=True)
    g.add_argument('--config-label', default='fwdf08 + legacy')
    g.add_argument('--out', required=True)
    g.set_defaults(func=cmd_generate)
    c = sub.add_parser('compare')
    c.add_argument('--old', required=True)
    c.add_argument('--new', required=True)
    c.add_argument('--out', required=True)
    c.set_defaults(func=cmd_compare)
    r = sub.add_parser('replay')
    r.add_argument('--runs-root', required=True)
    r.add_argument('--gates', required=True)
    r.add_argument('--config-label', default='fwdf08 + legacy')
    r.add_argument('--frozen', required=True)
    r.add_argument('--out', required=True)
    r.add_argument('--diff-out', required=True)
    r.set_defaults(func=cmd_replay)
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except (SeedDiagError, PA.PhaseAnalysisError, AP.AlignedPoseError,
            OG.OutputPathError) as exc:
        sys.stderr.write('p0a_seeddiag_replay: error: %s\n' % exc)
        return 2
    except (OSError, ValueError) as exc:
        # missing/unreadable/invalid JSON inputs: explicit error, no traceback
        sys.stderr.write('p0a_seeddiag_replay: input error: %s\n' % exc)
        return 2


if __name__ == '__main__':
    sys.exit(main())
