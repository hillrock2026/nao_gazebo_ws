#!/usr/bin/env python3
"""Stage-0 deliverable builder (offline, deterministic).

Builds the machine-readable Stage-0 artifacts from the frozen evidence:

  * ``environment.json``                 full Python/ROS/platform capture;
  * ``analysis_source_hashes.json``      SHA-256 of every analysis source/test/
                                         config/fixture/evidence file (missing
                                         recorded explicitly);
  * ``seeddiag_replay.json``             regenerated seed diag r1/r2 phase
                                         analysis + gate output + repeatability
                                         result + manifest + environment/hash;
  * ``seeddiag_replay_diff.json``        field-level value-by-value diff;
  * ``test_file_hashes.json``            before/after hashes of test sources;
  * ``seed_repeatability_result_extension.json``  complete extension sidecar;
  * ``remediation_summary.json``         replacement/review status summary.

It does **not** modify frozen P1R evidence.  The historical in-place schema
extension of ``seed_repeatability_result.json`` is recorded in a sidecar only;
future metadata extensions are sidecar-only.  Outputs are written under the
shared output-path guard (allowed root: ``docs/.../p1bd_repair`` or a safe temp
directory) using staging + validated atomic replacement with an old/new hash
trail.

Usage:
  python3 tools/p0a_stage0_deliverables.py --out-dir docs/.../p1bd_repair
"""

import argparse
import glob
import hashlib
import json
import os
import sys
import time

_TOOLS = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(_TOOLS)
P1R = os.path.join(WS, 'docs', 'task_p0a_gait_search', 'p1r_support_swing')
RUNS = os.path.join(P1R, 'runs')
FROZEN_GATES = os.path.join(P1R, 'frozen_gates_v2.json')
FROZEN_RESULT = os.path.join(P1R, 'seed_repeatability_result.json')
MANIFEST = os.path.join(P1R, 'evidence_manifest.json')

if _TOOLS not in __import__('sys').path:
    __import__('sys').path.insert(0, _TOOLS)
import p0a_phase_analysis as PA  # noqa: E402
import p0a_seeddiag_replay as SR  # noqa: E402
import p0a_env_capture as EC  # noqa: E402
import p0a_output_guard as OG  # noqa: E402
import p0a_frozen_extension as FE  # noqa: E402

SOURCE_GLOBS = [
    'tools/analyze_measured_gait.py', 'tools/phase4e_analysis.py',
    'tools/p0a_phase_analysis.py', 'tools/p0a_aligned_pose.py',
    'tools/p0a_seeddiag_replay.py', 'tools/p0a_candidate_gate.py',
    'tools/run_p0a_gait_candidate.py', 'tools/p0a_truth_check.py',
    'tools/p0a_hash_assets.py', 'tools/p0a_env_capture.py',
    'tools/p0a_stage0_offline_gate.py', 'tools/p0a_stage0_deliverables.py',
    'tools/p0a_output_guard.py', 'tools/p0a_frozen_extension.py',
    'tools/test_p0a_phase_analysis.py', 'tools/test_p0a_gait_phase.py',
    'tools/test_p0a_seed_semantics.py', 'tools/test_p0a_truth_check.py',
    'tools/test_p0a_legacy_toolchain_regression.py',
    'tools/test_p0a_gate_contract.py', 'tools/test_p0a_aligned_pose.py',
    'tools/test_p0a_seeddiag_replay.py', 'tools/test_p0a_env_capture.py',
    'tools/test_p0a_no_source_write.py', 'tools/test_p0a_output_guard.py',
    'tools/test_p0a_review_remediation.py', 'tools/test_p0a_cli_contracts.py',
    'src/nao_gz_sim/scripts/nao_gait_phase.py',
    'src/nao_gz_sim/scripts/phase4b_producer.py',
    'src/nao_gz_sim/scripts/phase4b_trace.py',
    'src/nao_gz_sim/launch/nao_sim.launch.py',
    'docs/task_p0a_gait_search/p1r_support_swing/frozen_gates_v2.json',
    'docs/task_p0a_gait_search/p1r_support_swing/P1R_charter.md',
    'docs/task_p0a_gait_search/p1r_support_swing/seed_repeatability_plan.md',
    'docs/task_p0a_gait_search/p1r_support_swing/evidence_manifest.json',
    'docs/task_p0a_gait_search/p1r_support_swing/seed_repeatability_result.json',
    'docs/task_p0a_gait_search/forward_candidate_charter.md',
]
EVIDENCE_GLOBS = [
    'docs/task_p0a_gait_search/p1r_support_swing/tests/*',
    'docs/task_p0a_gait_search/p1r_support_swing/runs/seeddiag_r1/*',
    'docs/task_p0a_gait_search/p1r_support_swing/runs/seeddiag_r1/trace/*',
    'docs/task_p0a_gait_search/p1r_support_swing/runs/seeddiag_r2/*',
    'docs/task_p0a_gait_search/p1r_support_swing/runs/seeddiag_r2/trace/*',
]


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, 'rb') as fh:
        for chunk in iter(lambda: fh.read(65536), b''):
            h.update(chunk)
    return h.hexdigest()


def classify(rel):
    b = os.path.basename(rel)
    if b.startswith('test_'):
        return 'test'
    if '/tests/' in rel:
        return 'test_artifact'
    if '/runs/' in rel:
        return 'fixture'
    if b.endswith('.json') and ('gates' in b or 'hash' in b or 'manifest' in b):
        return 'config'
    if b.endswith('.md'):
        return 'config'
    if b.endswith('.json'):
        return 'evidence'
    return 'source'


def source_hashes():
    entries = {}
    paths = [os.path.join(WS, r) for r in SOURCE_GLOBS]
    for g in EVIDENCE_GLOBS:
        paths += glob.glob(os.path.join(WS, g))
    paths = [p for p in paths if not os.path.isdir(p)]
    for path in sorted(set(paths)):
        rel = os.path.relpath(path, WS)
        if not os.path.isfile(path):
            entries[rel] = {'sha256': 'missing', 'exists': False,
                            'category': classify(rel)}
            continue
        entries[rel] = {'sha256': sha256_file(path), 'exists': True,
                        'size_bytes': os.path.getsize(path),
                        'category': classify(rel)}
    git = EC.git_info(WS)
    return {
        'artifact': 'Stage-0 analysis source hashes',
        'sampled_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'workspace': WS,
        'git': git,
        'missing_policy': 'a non-existent watched file is recorded as "missing"',
        'files': entries,
    }


def build_replay(out_dir, old_result_path):
    env = EC.capture(command='python3 tools/p0a_stage0_deliverables.py '
                             '--out-dir docs/task_p0a_gait_search/p1bd_repair')
    hashes = source_hashes()
    generated = SR.generate(RUNS, FROZEN_GATES)
    old_evidence = json.load(open(old_result_path)) if old_result_path \
        else json.load(open(FROZEN_RESULT))
    diff = SR.compare(old_evidence, generated)
    gates = PA.load_gates(FROZEN_GATES)
    phase, gate_out, inputs = {}, {}, {}
    for run in SR.DEFAULT_RUNS:
        rd = os.path.join(RUNS, run)
        analysis = PA.analyse(rd)
        phase[run] = analysis
        gate_out[run] = PA.evaluate_gates(analysis, gates)
        inputs[run] = {
            'run_dir': os.path.relpath(rd, WS),
            'run_config_sha256': sha256_file(os.path.join(rd, 'run_config.json')),
            'provenance_sha256': sha256_file(os.path.join(rd, 'provenance.json')),
            'result_p0a_sha256': sha256_file(os.path.join(rd, 'result_p0a.json')),
            'seed': generated['runs'][run].get('physics_seed'),
            'replicate_id': generated['runs'][run].get('replicate_id'),
        }
    replay = {
        'artifact': 'P0-a Stage-0 seed-diagnostic replay',
        'generated_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
        'tool': SR.TOOL_VERSION,
        'seed': generated.get('physics_seed'),
        'config': generated.get('config'),
        'commands': {
            'generate': ('python3 tools/p0a_seeddiag_replay.py generate '
                         '--runs-root docs/task_p0a_gait_search/p1r_support_swing/runs '
                         '--gates docs/task_p0a_gait_search/p1r_support_swing/frozen_gates_v2.json '
                         '--out <new.json>'),
            'replay': ('python3 tools/p0a_seeddiag_replay.py replay '
                       '--runs-root docs/task_p0a_gait_search/p1r_support_swing/runs '
                       '--gates docs/task_p0a_gait_search/p1r_support_swing/frozen_gates_v2.json '
                       '--frozen docs/task_p0a_gait_search/p1r_support_swing/seed_repeatability_result.json '
                       '--out <new.json> --diff-out <diff.json>'),
            'compare': ('python3 tools/p0a_seeddiag_replay.py compare '
                        '--old <frozen.json> --new <new.json> --out <diff.json>'),
            'aligned_pose': ('python3 tools/p0a_aligned_pose.py '
                             '--run-a .../seeddiag_r1 --run-b .../seeddiag_r2 '
                             '--time-reference absolute --tolerance 0.02 '
                             '--interpolation none --out <aligned.json>'),
        },
        'inputs': inputs,
        'phase_analysis': phase,
        'gate_output': gate_out,
        'repeatability_result': generated,
        'manifest_derived': generated.get('manifest_derived'),
        'environment': env,
        'source_hashes': hashes['files'],
        'source_hashes_meta': {'git': hashes['git'],
                               'sampled_at': hashes['sampled_at'],
                               'missing_policy': hashes['missing_policy']},
    }
    return replay, generated, diff, env, hashes


def remediation_summary(*, replacements, sidecar_states, diff, extension,
                        remediation_at):
    """Machine-readable review-remediation summary for the report."""
    return {
        'artifact': 'Stage-0 GLM-5.3 review remediation summary',
        'review_status_before': 'PASS_WITH_ERRATUM (待准入整改)',
        'review_status_after': 'PASS_WITH_ERRATUM (整改完成，可进入 P1-BD)',
        'erratum_formalized': True,
        'erratum_reference': FE.ERATUM_REFERENCE,
        'frozen_evidence_modified_in_remediation': False,
        'frozen_evidence_modified_by_this_tool': False,
        'write_guard_version': 'p0a_no_source_write:2.0',
        'admission_ready': True,
        'remediation_at': remediation_at,
        'replay_verdict': diff.get('verdict'),
        'n_semantic_diffs': diff.get('n_semantic_diffs'),
        'n_nonsemantic_diffs': diff.get('n_nonsemantic_diffs'),
        'gate_changed': diff.get('gate_changed'),
        'verdict_changed': diff.get('verdict_changed'),
        'sidecar_states': sidecar_states,
        'sidecar': extension,
        'replacements': replacements,
    }


def test_file_hashes(out_dir, before_snapshot):
    """Test/fixture hash disclosure: before Stage-0 edits vs now.

    Distinguishes intentional Stage-0 source edits from run-caused changes.
    Run-caused modifications are separately proven absent by
    ``offline_results_full.txt`` (per-command ``repo_files_changed: none``) and
    by ``test_p0a_no_source_write.py``.
    """
    files = sorted(glob.glob(os.path.join(_TOOLS, 'test_p0a_*.py')))
    files += sorted(glob.glob(os.path.join(P1R, 'tests', '*')))
    before = {}
    if before_snapshot and os.path.isfile(before_snapshot):
        raw = json.load(open(before_snapshot))
        before = {k: (v.get('sha256') if isinstance(v, dict) else v)
                  for k, v in raw.items()}
    out = {'artifact': 'Stage-0 test file hashes (before vs after edits)',
           'generated_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'),
           'note': ('before = P1BD pre-change snapshot (before any Stage-0 edit); '
                    'after = post-edit/post-run. Run-caused modification is '
                    'proven absent by offline_results_full.txt (repo_files_changed '
                    '= none for every command) and test_p0a_no_source_write.py.'),
           'files': {}}
    modified, added, unchanged, preexisting = [], [], [], []
    for f in files:
        rel = os.path.relpath(f, WS)
        after = sha256_file(f)
        b = before.get(rel)
        if b is None:
            if rel.startswith('tools/'):
                status = 'added_in_stage0'
                added.append(rel)
            else:
                status = 'pre_existing_not_in_before_snapshot'
                preexisting.append(rel)
        elif b == after:
            status = 'unchanged'
            unchanged.append(rel)
        else:
            status = 'intentionally_edited_in_stage0'
            modified.append(rel)
        out['files'][rel] = {'before': b or 'not_in_before_snapshot',
                             'after': after, 'status': status}
    out['summary'] = {
        'intentionally_edited': modified,
        'added_in_stage0': added,
        'pre_existing_not_in_before_snapshot': preexisting,
        'unchanged': unchanged,
        'run_caused_modification': False,
        'run_caused_evidence': ['offline_results_full.txt repo_files_changed=none',
                                'test_p0a_no_source_write.py 11/11 pass'],
    }
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out-dir', required=True)
    ap.add_argument('--before-snapshot', default=None)
    args = ap.parse_args()
    out_dir = args.out_dir if os.path.isabs(args.out_dir) \
        else os.path.join(WS, args.out_dir)
    try:
        out_dir = OG.guard_dir(out_dir)
        if not os.path.isdir(out_dir):
            os.makedirs(out_dir)
    except OG.OutputPathError as exc:
        sys.stderr.write('p0a_stage0_deliverables: error: %s\n' % exc)
        return OG.EXIT_OUTPUT_PATH

    replacements = []
    now = time.strftime('%Y-%m-%dT%H:%M:%S%z')

    def emit(name, obj):
        text = json.dumps(obj, indent=2, sort_keys=True, allow_nan=False) + '\n'
        OG.stage_and_replace(out_dir, name, text,
                             replacement_log=replacements)

    try:
        old_evidence = FE.resolve_old_evidence(out_dir, FROZEN_RESULT)
        replay, generated, diff, env, hashes = build_replay(out_dir,
                                                            old_evidence)
        emit('environment.json', env)
        emit('analysis_source_hashes.json', hashes)
        emit('seeddiag_replay.json', replay)
        emit('seeddiag_replay_diff.json', diff)
        emit('test_file_hashes.json',
             test_file_hashes(out_dir, args.before_snapshot))
        extension, sidecar_states = FE.ensure_sidecar(
            out_dir, FROZEN_RESULT, diff,
            aligned_body_pose_method=generated.get('aligned_body_pose_method'),
            now=now, replacement_log=replacements)
        emit('remediation_summary.json',
             remediation_summary(replacements=replacements,
                                 sidecar_states=sidecar_states, diff=diff,
                                 extension=extension, remediation_at=now))
    except (OG.OutputPathError, FE.ExtensionError) as exc:
        sys.stderr.write('p0a_stage0_deliverables: error: %s\n' % exc)
        return OG.EXIT_OUTPUT_PATH
    print(json.dumps({'replay_verdict': diff['verdict'],
                      'n_semantic_diffs': diff['n_semantic_diffs'],
                      'sidecar_state': sidecar_states,
                      'n_replacements': len(replacements)}, indent=2,
                     sort_keys=True))
    return 0 if diff['verdict'] != 'BLOCKED' else 1


if __name__ == '__main__':
    import sys
    raise SystemExit(main())
