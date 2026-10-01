# Phase B — Carry-Forward Register

- Document: `docs/task_p0a_gait_search/phase_b/Phase_B_Carry_Forward_Register.md`
- Stage: **Phase B-0/B-1**
- Purpose: record every open Phase A closure finding (P2-A…P2-E) plus RFC open questions,
  with owner, target sub-phase, and an explicit scope guard so none of them silently
  expands the Phase B-1 RFC scope.
- Rule: **none of these findings authorizes a gait-algorithm change or a change to Phase A
  acceptance semantics.** Any such change requires its own RFC + version decision.

---

## 1. Phase A closure findings carried forward

### P2-A — Schema version / provenance coverage shape consistency

- **Finding:** after the duration/coverage fields were added, the episode bundle schema
  version remained `phase_a_1.0.0`. There is no written schema-evolution policy. The
  `coverage` block also appears duplicated across `manifest.json` / `provenance.json` /
  `episode_integrity.json`.
- **Impact:** low (all copies are identical today), but a drift risk for batch production.
- **Owner:** Collector (integrity/manifest) + Runner (provenance) → resolved in B3.
- **Target:** B3.
- **Options (audit decides — UQ-1):**
  1. Keep `phase_a_1.0.0` and add an additive `episode_schema_version` field (RFC
     recommendation: least churn on a freshly-frozen baseline).
  2. Bump to `phase_b_1.0.0` with a written policy.
- **Scope guard:** additive only; no dataset schema v1.0.1 change; no field rename.

### P2-B — Missing internal-gap regression test

- **Finding:** Phase A added head/tail gap tests (CASE A–D) but no test for an **internal**
  gap (a data gap in the middle of the gait window). The `_coverage()` gap-cap policy is
  therefore under-tested.
- **Impact:** a middle-of-window gap could be under- or over-counted silently.
- **Owner:** Collector finalize (coverage).
- **Target:** B3 — `tools/test_phase_b_internal_gap.py`.
- **Required assertions:** internal interval > 0.10 s contributes 0; coverage reflects the
  internal loss; guard does **not** misclassify an internal gap as head/tail truncation
  unless the *tail* is genuinely lost.
- **Scope guard:** test only; no coverage-formula change.

### P2-C — Systematic low gait coverage (~0.74)

- **Finding:** real episodes show `gait_relative_coverage ≈ 0.74–0.76`, `gait_head_gap ≈
  1.95–2.06 s`, driven by `contact_not_received` (239 in closure_01) and command
  readiness (`command_not_available` 55, `command_stale` 51). This is a **strict-sync
  production-efficiency property**, not a coverage-definition bug.
- **Impact:** the head of the requested gait window is not captured; disclosed, not hidden.
- **Owner:** Collector (instrumentation) + later phase (acceptance alignment).
- **Target:** B3 = **instrumentation only** (additive `collector_summary.json` fields):
  `first_accepted_offset_from_gait_start_sec`, time-to-first-accepted, and a
  coverage-attribution view of the rejected prefix.
- **Explicit prohibition:** MUST NOT change acceptance semantics, MUST NOT redefine
  coverage, MUST NOT touch gait math or the gait phase model.
- **Deferred:** any acceptance-semantics change (earlier contact/command alignment) is a
  **separate future RFC** with its own regression evidence.
- **Open question:** UQ-4.

### P2-D — "gait never started" vs "delayed onset" vs "head-only loss"

- **Finding:** acquisition data alone cannot positively distinguish "the gait never
  executed" from "the head of the window was not captured". The C3 guard protects the
  window *end* and always discloses the head gap, so low coverage is never silent, but the
  positive distinction needs gait-content analysis.
- **Impact:** a "walking_pass=False with low coverage" episode is ambiguous about *why*.
- **Owner:** Evaluator.
- **Target:** B4 — optional `gait_activity` sub-label derived from gait-content evidence
  (command trace / joint motion), **not** a Phase B gate.
- **Scope guard:** evaluator enrichment only; integrity semantics unchanged.
- **Open question:** UQ-5.

### P2-E — Residual-process detection is environment-limited

- **Finding:** `_residual_count()` in the Runner performs a **global** scan (any concurrent
  `gz sim` counts), so a parallel job can cause a false residual (>0) and a spurious
  integrity FAIL.
- **Impact:** medium under concurrency; low on a controlled single-instance host.
- **Owner:** Runner (cleanup/residual).
- **Target:** B2 — partition-scoped residual detection (filter by the episode's
  `GZ_PARTITION`), behind a test. Behavior preserved otherwise.
- **Scope guard:** detection scoping only; no change to the residual==0 gate semantics.
- **Open question:** UQ-6.

---

## 2. Additional carried items (from the Phase A reports)

| ID | Item | Target | Note |
|---|---|---|---|
| CF-6 | Provenance/`launch_cmd.txt` record the **pre-publish staging path**; after atomic publish the staging path no longer exists | B2 | Additive `run_dir_publish_path` mapping or relative-path normalization; must NOT fabricate the real command |
| CF-7 | `body_pose` lives only in the raw trace, not in frozen `samples.jsonl` | later | Any inclusion is a schema v1.0.1 evolution → separate RFC |
| CF-8 | Four pre-existing test FAILs (`p0a_gate_contract`, `p0a_cli_contracts`, `p0a_review_remediation`, `p0a_no_source_write` sub-fails) caused by archived inputs (`p1r_support_swing/`, `p1bd_repair/`) | every sub-phase | Reported as **"no demonstrated regression"**, never "no regression proven"; compare failing sets byte-for-byte |
| CF-9 | Single-episode closure → no repeatability claim | Phase B repeatability probe (§10 RFC) | design only in B-1 |

---

## 3. RFC open questions (for the Independent Audit Agent)

| ID | Question | RFC stance |
|---|---|---|
| UQ-0 | `audit_closure_12346` not found in the workspace | Confirm the intended artifact or that the reference is stale |
| UQ-1 | Episode bundle schema versioning (P2-A) | Additive `episode_schema_version`; no bump |
| UQ-2 | Provenance staging-path (CF-6) | Add additive `run_dir_publish_path` in B2 |
| UQ-3 | `research_contract` shape in Phase B | Keep `null`/`NOT_EVALUATED` placeholder; no new contract logic |
| UQ-4 | P2-C scope | Instrumentation only; acceptance change deferred to a separate RFC |
| UQ-5 | P2-D distinction | Optional Evaluator sub-label in B4; not a gate |
| UQ-6 | P2-E residual scoping | Implement in B2 behind a test |
| UQ-7 | Batch runner in Phase B core? | Defer (`run_phase_b_batch.py` design only) |

---

## 4. Scope guard (non-negotiable)

```text
None of P2-A..P2-E, CF-6..CF-9, or UQ-0..UQ-7 authorizes:
  - a gait algorithm change
  - a gait phase model change
  - a change to Phase A acceptance semantics
  - re-coupling gait FAIL with episode invalid
  - a dataset schema v1.0.1 change
  - starting Runner/Collector/Evaluator implementation during B-1
```

Any change in the list above requires its own RFC and an explicit version decision.

End of Carry-Forward Register.
