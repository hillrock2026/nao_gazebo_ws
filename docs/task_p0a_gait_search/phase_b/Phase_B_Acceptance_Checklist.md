# Phase B — Acceptance Checklist & Self-Audit

- Document: `docs/task_p0a_gait_search/phase_b/Phase_B_Acceptance_Checklist.md`
- Stage: **Phase B-0/B-1 self-audit** (Implementation Agent self-check before hand-off)
- Purpose: (1) define the Gate B1-1…B1-14 checklist; (2) record the self-audit result;
  (3) give the Independent Audit Agent a reproducible verification procedure.
- Rule: this is a **self-audit**, not acceptance. Acceptance belongs to the Independent
  Audit Agent.

---

## 1. Gate checklist + self-audit

| Gate | Requirement | Evidence | Self-audit |
|---|---|---|---|
| **B1-1** | Phase A baseline is traceable | `Phase_B0_Freeze_Record.md` §3 (source SHA-256) + §4 (episode SHA-256) + §7 (closure conclusion) | **PASS** |
| **B1-2** | Freeze content is explicit | `Phase_B0_Freeze_Record.md` §2 (components) + §5 (semantics) + §6 (schemas) | **PASS** |
| **B1-3** | Episode contract is explicit | RFC §2 (Episode first-class; EpisodeSpec/Execution/Artifact/EvaluationResult) | **PASS** |
| **B1-4** | Runner/Collector/Evaluator ownership unambiguous | RFC §4 (ownership matrix + integrity-row resolution) | **PASS** |
| **B1-5** | Episode lifecycle explicit | RFC §5 (CREATED→…→COMMITTED + failure states + permissions + idempotency/retry/cleanup/crash) | **PASS** |
| **B1-6** | Failure taxonomy explicit | RFC §6 (7 classes + retention policy + integrity-mapping rule) | **PASS** |
| **B1-7** | duration/coverage semantics no implicit change | RFC §8 (inherited verbatim; freeze anchors in Freeze Record §5) | **PASS** |
| **B1-8** | seed provenance chain explicit | RFC §9 (spec→requested→argv→actual→provenance→verify) | **PASS** |
| **B1-9** | atomic publication explicit | RFC §7 (staging→finalize→os.replace; crash-safe) | **PASS** |
| **B1-10** | integrity/quality/contract/outcome separated | RFC §2.4 + §6 (forbidden couplings stated) | **PASS** |
| **B1-11** | carry-forward findings explicit | `Phase_B_Carry_Forward_Register.md` (P2-A…E, CF-6…9, UQ-0…7) | **PASS** |
| **B1-12** | non-goals explicit | RFC §14 (+ register §4 scope guard) | **PASS** |
| **B1-13** | future implementation boundary explicit | RFC §11 (interfaces) + §12 (files/order) + §4 (import acyclicity) | **PASS** |
| **B1-14** | RFC auditable by an independent agent | RFC is self-contained with hashes, frozen semantics, and this checklist | **PASS** |

**Self-audit result: 14/14 PASS.**

Caveats (do not fail any gate; disclosed for the auditor):

- **UQ-0:** the brief's `audit_closure_12346` does not exist in the workspace; this RFC
  proceeded from the available closure evidence.
- The closure baseline is **single-episode per configuration**; no repeatability is claimed
  (protocol defined, not run).

---

## 2. Independent verification procedure (for the Audit Agent)

Read-only, reproducible steps:

1. **Re-hash the freeze:** run `sha256sum` on the §3 source list and the §4 episode file
   lists in `Phase_B0_Freeze_Record.md`; all must match.
2. **Re-derive the two episodes:** recompute duration/coverage from `smoke_01` and
   `closure_01` datasets/traces and confirm the frozen values (§5 of the Freeze Record).
3. **Confirm no code changed during B-0/B-1:** verify that only the four `phase_b/*.md`
   documents are new and that every §3/§4 artifact is untouched.
4. **Check the acyclicity claim** against RFC §4/§12 (import direction).
5. **Check scope compliance:** no Runner/Collector/Evaluator implementation exists; no gait
   math change; no schema change; no batch run.

---

## 3. Planned verification per sub-phase (implementation stage, not now)

Offline (fast):

- Existing suites: `test_phase_a_episode_integrity.py` (57), `test_p0a_seed_semantics.py`
  (14), `test_task4_phase4d_offline.py` (133), plus the `test_p0a_*` gate/cli/review/
  no-source-write suites (record the exact failing set as baseline).
- New: internal-gap regression (P2-B); evaluator purity/idempotency (B4); import
  acyclicity guard; duration/coverage freeze test against both episodes.

Runtime (per sub-phase):

- ≥1 real Gazebo episode per sub-phase → `EPISODE_INTEGRITY=PASS`, residual 0, seed
  verified, golden validator valid.
- `smoke_01` read-only re-evaluation with before/after directory hashing (B-G12).

---

## 4. Required final report block (this stage)

```text
PHASE_B0_FREEZE_STATUS = COMPLETE

PHASE_B1_RFC_STATUS = READY

RUNNER_IMPLEMENTATION_STARTED = NO

PHASE_A_BEHAVIOR_CHANGED = NO

RFC_ARTIFACTS:
- docs/task_p0a_gait_search/phase_b/Phase_B_RFC.md
- docs/task_p0a_gait_search/phase_b/Phase_B0_Freeze_Record.md
- docs/task_p0a_gait_search/phase_b/Phase_B_Carry_Forward_Register.md
- docs/task_p0a_gait_search/phase_b/Phase_B_Acceptance_Checklist.md

CARRY_FORWARD_FINDINGS:
- P2-A schema version / provenance coverage shape consistency
- P2-B internal-gap regression test missing
- P2-C systematic low gait coverage (~0.74), instrumentation only
- P2-D gait never started vs delayed onset vs head-only loss
- P2-E residual-process scan environment-limited
- CF-6 staging-path provenance, CF-7 body_pose not in samples,
  CF-8 pre-existing test FAILs, CF-9 single-episode baseline

OPEN_ARCHITECTURAL_QUESTIONS:
- UQ-0 missing audit_closure_12346
- UQ-1 episode bundle schema versioning
- UQ-2 staging-path provenance polish
- UQ-3 research_contract placeholder shape
- UQ-4 P2-C instrumentation-only scope
- UQ-5 P2-D evaluator sub-label
- UQ-6 P2-E partition-scoped residual count
- UQ-7 batch runner in/out of Phase B core

SELF_AUDIT:
- Gate B1-1:  PASS
- Gate B1-2:  PASS
- Gate B1-3:  PASS
- Gate B1-4:  PASS
- Gate B1-5:  PASS
- Gate B1-6:  PASS
- Gate B1-7:  PASS
- Gate B1-8:  PASS
- Gate B1-9:  PASS
- Gate B1-10: PASS
- Gate B1-11: PASS
- Gate B1-12: PASS
- Gate B1-13: PASS
- Gate B1-14: PASS

RECOMMENDATION:
READY_FOR_INDEPENDENT_AUDIT
```

End of Acceptance Checklist.
