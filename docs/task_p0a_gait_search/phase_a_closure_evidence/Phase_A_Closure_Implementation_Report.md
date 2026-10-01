# Phase A Closure — Duration & Coverage Hardening — Implementation Report

- Task: Phase A Closure / Duration & Coverage Hardening (close audit finding **P1-1** only)
- Baseline: `PHASE_A_ACCEPTED` (GLM-5.3 independent acceptance). Phase A is **not** redesigned.
- Author: opencode / DeepSeek
- Date: 2026-09-30 (Asia/Shanghai)
- Workspace: `/home/liu/workspace/Nao_Program/nao_gazebo_ws`
- Scope: C1 duration semantics · C2 gait-relative coverage · C3 early-termination guard head blind spot.
  Not Phase B, not gait optimization, no producer/collector/schema redesign.
- Final: **`PHASE_A_CLOSURE_STATUS: PHASE_A_CLOSURE_READY`**

---

## 0. Audit-first — PHASE_A_CLOSURE_BASELINE

Independently recomputed from `phase_a_evidence/smoke_01` (read-only):

```text
requested_gait_duration_sec   = 8.0
record_window_duration_sec    = 12.0   (start_delay .5 + gait 8 + ramp 1.5 + hold 1.0 + margin 1.0)
accepted sample count         = 849
first accepted sample ts      = 3.579
last  accepted sample ts      = 12.059
accepted_sample_span_sec      = 8.48
model-ready (state_trace[0])  = 1.019  (launch.log producer: "ready 1.019, starts +0.50")
derived gait window           = [1.519, 9.519]
gait HEAD gap                 = 2.06 s
clock_trace span (sim run)    = 0.024 -> 13.003 (12.979 s)
gait_relative_coverage        = 5.94 / 8.0 = 0.7425   <-- was NOT disclosed
old guard verdict             = normal_completion        <-- span 8.48 > requested-1.0 = 7.0
```

Field semantics at baseline (the ambiguity):

| Field | Value | True meaning | Problem |
|---|---|---|---|
| `requested_duration_sec` (cfg/prov/manifest) | 8.0 | requested **gait** duration | ok, but named ambiguously |
| `duration_sec` (collector summary) | 12.0 | collector **record window** | ok |
| `duration_sec` (manifest) / `actual_duration_sec` (termination_detail) | 8.48 | **accepted-sample timestamp span** | **implies "episode/execution duration" — wrong** |
| `record_window_duration_sec` | — | absent | — |
| `execution_duration_sec` | — | absent | — |
| `gait_relative_coverage` | — | absent | **head gap hidden** |

Root cause of the head blind spot: the guard compared the **accepted-sample span** (8.48) to
`requested − 1.0` (7.0). The 2.48 s of samples *after* `gait_end` (9.519) inflate the span and mask
the 2.06 s head gap. (Corroborates GLM: ~2.5 s blind spot / ~1.48 s margin.)

---

## 1. Changes (reuse > fix > add; minimal)

| File | Change | Reason |
|---|---|---|
| `tools/nao_episode_integrity.py` | C1/C2/C3: explicit `duration` block, `coverage` block, gait-window tail-truncation guard, `derive_gait_start()`, `_coverage()`, `_trace_t_span()`, new smoke block | close P1-1 |
| `tools/run_phase_a_episode.py` | compute authoritative `gait_start` (state_trace[0] + start_delay) and pass it; persist `duration`/`coverage` into manifest + provenance; keep legacy aliases labelled | close P1-1 |
| `tools/test_phase_a_episode_integrity.py` | +§9 coverage/early-termination cases (A–D), duration-semantics tests, new smoke-block assertions | prove the semantics |

No change to: `phase4b_producer.py` (gait math), `phase4d_collector.py`, `nao_trajectory_schema.py` (schema
v1.0.1 untouched), `validate_task4_phase4d_dataset.py`, `nao_sim.launch.py`, `run_p0a_gait_candidate.py`,
or any historical/archived evidence.

Hashes:

```text
tools/nao_episode_integrity.py         cc3e7e95bc900da37c90f14fd38a23226ea45cea0e19f70989c1b4470dd88a3a
tools/run_phase_a_episode.py           a024140f294a293b0fb8503e6f53960028d8919c516f1cb640891d9fc297d6fb
tools/test_phase_a_episode_integrity.py c761fd1cbefd72f3d6fc99be69c84259e1a0d92472fd3cf9e19dba5f32485d79
```

### C1 — explicit duration semantics

`episode_integrity.json.duration` and `manifest.json.duration` / `provenance.json.duration` now carry:

```text
requested_gait_duration_sec   user-requested gait duration
record_window_duration_sec    collector/runner allowed acquisition window
execution_duration_sec        observed gz sim-time span (clock_trace first->last); null if absent
accepted_sample_span_sec      last - first accepted sample timestamp
*_LEGACY / legacy_note        actual_duration_sec / duration_sec kept as DEPRECATED aliases of the span
```

Legacy `manifest.duration_sec` and `manifest.requested_duration_sec` are **kept** for backward
compatibility but explicitly labelled; the smoke block no longer prints an ambiguous `ACTUAL_DURATION`.

### C2 — gait-relative coverage

Definition (documented in the verdict's `coverage` block):

```text
window        = [gait_start, gait_start + requested_gait_duration_sec]
gait_start    = producer ready_stamp + start_delay_sec
                (ready_stamp = first valid /joint_states stamp; cross-checked vs launch.log producer line)
numerator     = measure of the UNION of consecutive accepted-sample intervals [t_i, t_{i+1}],
                clipped to `window`; an interval longer than GAP_CAP_SEC (0.10 s) is a data gap -> 0
denominator   = requested_gait_duration_sec
timestamp     = dataset/samples.jsonl[].timestamp  (gazebo sim time)
gaps          = interval > 0.10 s treated as uncovered (reason: 100 Hz stream; real dt ~0.01 s)
rejected prefix = excluded by the window clip; surfaced as `gait_head_gap_sec`
terminal tail   = excluded by the window clip; surfaced as `gait_tail_truncation_sec`
early termination = finish_reason abnormal OR gait_tail_truncation_sec > 0.5 s
coverage_status = FULL (>=0.999) | PARTIAL | null
```

Rationale for union-of-intervals (not `last − first`): `last − first` includes the tail beyond
`gait_end` and ignores the head gap; for smoke_01 it yields 8.48/8.0 = 1.06 (>1, meaningless) and hides
the 2.06 s head gap. The union-of-intervals metric is bounded by the window and gap-aware.

### C3 — early-termination guard

`_classify_termination()` now keys on **`gait_tail_truncation_sec`** (the gait window end missed by
accepted samples, tolerance 0.5 s) instead of the raw span. This removes the head blind spot: the head
gap no longer inflates the margin. The legacy span check is retained **only** when no gait window can be
derived (episodes without a state trace), preserving backward compatibility.

---

## 2. Real Gazebo regression — closure_01 (§10)

```text
command: python3 tools/run_phase_a_episode.py --episode-id closure_01 --seed 12345 --duration 8.0 \
           --out-root docs/task_p0a_gait_search/phase_a_closure_evidence/closure_01 --timeout 240
```

```text
REQUESTED_GAIT_DURATION_SEC: 8.0     RECORD_WINDOW_DURATION_SEC: 12.0
EXECUTION_DURATION_SEC:      12.921  ACCEPTED_SAMPLE_SPAN_SEC:   8.490
GAIT_RELATIVE_COVERAGE:      0.7563  GAIT_HEAD_GAP_SEC: 1.950   GAIT_TAIL_TRUNCATION_SEC: 0.000
COVERAGE_STATUS: PARTIAL             SAMPLE_COUNT: 850
JOINT_STATE/COMMAND/IMU/CONTACT/BODY_POSE/CLOCK: PASS
EPISODE_INTEGRITY: PASS   TERMINATION: normal_completion
```

Independent verification of closure_01:

```text
gz sim argv (from /proc) : gz sim -v 4 -r -s --seed 12345 --physics-engine gz-physics-dartsim-plugin <world>
seed                     : requested 12345 == actual 12345  (verified = true)
golden validator         : valid=True manifest_ok=True samples_ok=True samples=850 errors=0
accounting               : seen 1204 == accounted 1204, unaccounted 0, finish duration_complete
coverage (recomputed)    : independently recomputed gait_start 1.466, coverage 0.7563 == reported
residual processes       : 0
episode_integrity        : PASS, failure_class null, gait_outcome.walking_pass=False (separate)
```

---

## 3. Backward compatibility (§11)

```text
smoke_01 re-evaluated (read-only):
  episode_integrity      : PASS  (termination normal_completion)   <- unchanged verdict
  golden validator       : valid=True, 849 samples
  duration block         : requested 8.0 / record 12.0 / execution 12.979 / accepted span 8.48
  coverage               : 0.7425, head gap 2.06, tail 0.0, status PARTIAL   <- now disclosed
  legacy fields          : manifest.duration_sec / requested_duration_sec still present
  files modified         : none (find -newermt returns nothing)
```

No schema v1.0.1 field was changed; new fields live only in the episode-level manifest/provenance/integrity.

---

## 4. Tests (§9)

```text
PASS  test_task4_phase4d_offline.py        133/133   (existing, unchanged)
PASS  test_p0a_seed_semantics.py            14/14    (existing, unchanged)
PASS  test_phase_a_episode_integrity.py     57/57    (26 original + 31 new closure checks)
PASS  real Gazebo regression (closure_01)           EPISODE_INTEGRITY=PASS, residual 0
FAIL  test_p0a_gate_contract.py                      PRE-EXISTING (frozen_gates_v2.json archived)
FAIL  test_p0a_cli_contracts.py                      PRE-EXISTING (p1r_support_swing/p1bd archived)
FAIL  test_p0a_review_remediation.py                 PRE-EXISTING (p1r_support_swing/*.json archived)
FAIL  test_p0a_no_source_write.py (14/19)            PRE-EXISTING (same 5 sub-fails as Phase A baseline)
```

New closure checks (all PASS) include: duration semantics (requested/record/span/legacy alias/execution
from clock trace); CASE A coverage≈1.0; CASE B prefix gap coverage≈0.75 + head-only gap not a failure;
CASE C truncation coverage≈0.375 + early-termination FAIL; early termination before onset; early
termination shortly after onset; normal completion; CASE D fall/poor-walking not an integrity failure.

---

## 5. Answers to the 15 required questions (§13)

1. **requested_gait_duration_sec** — user-requested gait duration (`FROZEN_GAIT['duration_sec']` / `--duration`); closure_01 = 8.0.
2. **record_window_duration_sec** — allowed acquisition window = start_delay + gait + stop_ramp + final_hold + margin (`record`); closure_01 = 12.0.
3. **execution_duration_sec** — observed Gazebo sim-time span = `trace/clock_trace.jsonl` first→last; closure_01 = 12.921; `null` when no clock trace (never fabricated).
4. **accepted_sample_span_sec** — `last − first` accepted sample timestamp; closure_01 = 8.490. Explicitly NOT execution/gait/record duration.
5. **ACTUAL_DURATION_SEC** — retained only as a DEPRECATED alias of `accepted_sample_span_sec` (`duration.actual_duration_sec_LEGACY`, `termination_detail.actual_duration_sec`, `manifest.duration_sec`), labelled with `legacy_note`; removed from the smoke output.
6. **gait_relative_coverage** — `covered / requested_gait_duration_sec`, where `covered` = union of consecutive-sample intervals clipped to `[gait_start, gait_end]`, gap > 0.10 s counted as uncovered.
7. **Why not sample-span / requested** — the span includes samples past `gait_end` and ignores the head gap; it gave 1.06 (nonsensical) for smoke_01 and hid the 2.06 s head gap. Coverage uses the real gait window and is bounded.
8. **Rejected prefix** — excluded by the window clip; surfaced as `gait_head_gap_sec` (smoke_01 2.06 s; closure_01 1.95 s).
9. **Terminal truncation** — samples past `gait_end` excluded; missing tail surfaced as `gait_tail_truncation_sec` (>0.5 s ⇒ early termination).
10. **Early termination** — `finish_reason ∉ {duration_complete, max_samples}` OR `gait_tail_truncation_sec > 0.5`; legacy span fallback only when no gait window is derivable.
11. **Gait outcome vs integrity** — still fully separated: `gait_outcome` (incl. `walking_pass`, `fall_detected`) never sets `failure_class`; CASE D asserts PASS with `walking_pass=False` + `fall_detected=True`.
12. **smoke_01 compatible** — re-evaluates to PASS/normal_completion; golden validator valid/849; no file modified; legacy manifest fields kept.
13. **New episode independently verified** — closure_01: seed verified, golden validator valid/850, accounting closed, coverage independently recomputed = 0.7563, residual 0.
14. **All test results** — see §4 (133/133, 14/14, 57/57 PASS; 4 pre-existing FAIL from archived inputs).
15. **New P0/P1/P2** — see §6.

---

## 6. Findings

**P0** — none.

**Blocking P1** — none.

**P2 (disclosed, not blocking)**
1. **Gait-window coverage is PARTIAL (~0.74–0.76) for this gait/env.** The head of the requested gait
   window is not captured because the collector rejects until command+contact+IMU synchronize
   (contact arrives only after the feet settle). This is a property of the *strict sync* collector, now
   **explicitly disclosed** (`gait_relative_coverage`, `gait_head_gap_sec`, `coverage_status=PARTIAL`).
   Reducing it (e.g., earlier contact/settle alignment) is a collector/Phase-B concern and is out of scope
   per §3 (no collector redesign).
2. **"Gait never started" is not distinguishable from "head not captured" by acquisition data alone.**
   The new guard protects the gait window *end* and always discloses head gap/coverage, so a low-coverage
   episode is never *silent*; but positively detecting "gait did not execute" would require gait-content
   analysis, which is outside Phase A Closure. Recorded as a residual for Phase B.
3. `provenance.launch_configuration.argv` / `launch_cmd.txt` record the pre-publish **staging** path
   (accurate real command; staging path no longer exists after atomic publish). Unchanged from Phase A;
   candidate for a Phase-B provenance polish.
4. `_residual_count()` is a global scan (any concurrent `gz sim` counts) — inherited from the Phase A runner.

---

## 7. PHASE_A_CLOSURE_STATUS

```text
PHASE_A_CLOSURE_STATUS: PHASE_A_CLOSURE_READY
```

Justification (all gate conditions met):

```text
P0                                    = 0
blocking P1                           = 0
duration semantics                    = explicit (requested/record/execution/accepted-span; legacy labelled)
gait-relative coverage                = defined (window union-of-intervals, gap-capped, documented)
early-termination guard               = verified (head blind spot removed; CASE A-D + onset tests pass)
real Gazebo regression                = PASS (closure_01; seed verified; golden valid; residual 0)
old smoke compatibility               = PASS (smoke_01 unchanged, still PASS, golden valid)
no gait math change                   = true (producer/schema/collector untouched)
no Stage 7 change                     = true
```

Recorded but non-blocking: the PARTIAL head coverage (P2-1) is **disclosed**, not hidden; per §8 no
behaviour change was made for the head gap beyond disclosure + the guard fix.

Do not proceed to Phase B from this task.

End of report.
