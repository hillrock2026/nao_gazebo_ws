# Phase B-1 — Production Kernel RFC

- Document: `docs/task_p0a_gait_search/phase_b/Phase_B_RFC.md`
- Stage: **Phase B-1 (architecture / RFC only — NO implementation)**
- Baseline: `PHASE_B_BASELINE_FREEZE` (see `Phase_B0_Freeze_Record.md`)
- Author: Implementation Agent (Qoder + DeepSeek = one agent)
- Date: 2026-09-30 (Asia/Shanghai)
- Workspace: `/home/liu/workspace/Nao_Program/nao_gazebo_ws`
- Companion documents:
  - `Phase_B0_Freeze_Record.md` — frozen baseline + hashes
  - `Phase_B_Carry_Forward_Register.md` — P2-A…P2-E + open questions
  - `Phase_B_Acceptance_Checklist.md` — Gates B1-1…B1-14 self-audit
- Status at end: **`PHASE_B1_RFC_STATUS = READY`** (awaiting Independent Audit Agent)

> This document defines the architecture only. It writes **no** Runner, **no** Collector
> rewrite, **no** Evaluator, runs **no** batch, and changes **no** Phase A semantics.
> Implementation may only start after an independent auditor emits `PHASE_B_RFC_ACCEPTED`.

---

## 0. Meta — inputs read, and one missing input

### 0.1 Inputs read and re-verified for this RFC

Phase A implementation report, Phase A closure report, the GLM-5.3 audit prompt,
the Phase A task spec, both real episodes (`smoke_01`, `closure_01`), and the source:
`run_phase_a_episode.py`, `run_p0a_gait_candidate.py`, `nao_episode_integrity.py`,
`test_phase_a_episode_integrity.py`, `test_p0a_seed_semantics.py`, `phase4d_collector.py`,
`phase4b_producer.py`, `phase4b_trace.py`, `nao_trajectory_schema.py`, `nao_sim.launch.py`,
`validate_task4_phase4d_dataset.py`, `measured_contract.md`.

### 0.2 Missing input (disclosed, non-blocking)

The brief references **`audit_closure_12346`**. **No such file (or any file containing
`12346`) exists under `docs/` or `tools/`.** The only closure evidence present is
`phase_a_closure_evidence/` (report + `closure_01`, seed `12345`). Recorded as **UQ-0**.
This RFC proceeds from the available closure evidence.

---

## 1. Baseline — the three defects Phase B must remove

Phase A works and is accepted-with-findings, but three structural defects block a
production kernel (all re-derived from source, §1 of the Freeze Record):

1. **Runner calls the evaluator inline.** `run_phase_a_episode.py` computes
   `EI.evaluate(...)` and bakes the verdict into the published manifest, so the verdict
   cannot be re-derived without re-running, and the Runner owns evaluation semantics.
2. **Integrity and gait outcome share one module.** `nao_episode_integrity.py` emits both
   `episode_integrity` **and** `gait_outcome`; the two axes are not separable.
3. **No canonical-episode reader.** Duration/coverage derivations are private to the
   evaluator (`_coverage`, `_trace_t_span`, `derive_gait_start`), while the Runner
   duplicates a state-trace read to pass `gait_start` in.

Phase B removes these **without** touching gait math, the gait phase model, Stage 7,
P1-R, dataset schema v1.0.1, the golden validator, or any frozen Phase A field semantics.

---

## 2. Episode as a first-class citizen

An **Episode** is the atomic unit of production, and the thing future batch runs,
repeatability probes, and audits consume. It is defined as:

```text
Episode = identity
        + configuration       (EpisodeSpec)
        + execution           (EpisodeExecution)
        + raw streams         (trace/*.jsonl)
        + canonical samples   (dataset/samples.jsonl)
        + integrity           (episode_integrity.json)
        + evaluation          (evaluation sidecar)
        + provenance          (provenance.json)
```

An episode is **never** "good" or "bad" as a whole. It is a *valid data container* with
four **independent** verdict axes (see §2.4). A gait that falls, or fails the research
contract, is still a **valid episode**.

### 2.1 EpisodeSpec — what we intend to run

| Field | Meaning | Owner |
|---|---|---|
| `episode_id` | unique identity within a run/batch | caller |
| `gait_method`, `gait_phase_mode` | gait selector (no gait math here) | caller |
| `theta` (gait parameters) | frozen θ: step_length, frequency, swing_height, leg_extension, double_support_ratio, sway_amplitude/sign, arm params, foot_pitch, command_rate, blend, start_delay, stop_ramp, hold_final, actuator gains | caller |
| `requested_gait_duration_sec` | requested gait window | caller |
| `record_window_duration_sec` | computed acquisition window = start_delay + gait + stop_ramp + hold_final + margin | Runner |
| `seed` | physics RNG seed intended for `gz sim --seed` | caller |
| `world`, `physics_engine`, `robot_model` | environment selectors | Runner (defaults frozen) |
| `collection_config` | sync tolerances (imu/contact/command), duration, max_samples | Collector contract (frozen v1.0.1) |
| `provenance_metadata` | operator, git revision, tags, batch id | caller/Runner |

EpisodeSpec is **immutable** once a run starts.

### 2.2 EpisodeExecution — what actually happened

| Field | Meaning |
|---|---|
| `requested_seed` / `actual_gz_sim_seed` / `seed_verified` | seed chain result |
| `launch_cmd` (argv), `gz_sim_cmd.actual.txt`, `gz_sim_cmd.expected.txt` | real vs reconstructed command |
| `process_lifecycle` | pgid, signals sent, exit status |
| `start_timestamp` / `end_timestamp` / `wall_sec` | wall clock |
| `execution_duration_sec` | observed gz sim-time span (clock trace) |
| `finish_reason` | collector terminal reason |
| `termination` | `normal_completion` / `early_termination` / … |
| `exit_status` | launch process exit code |
| `early_termination` | bool (tail truncation / abnormal finish) |
| `infrastructure_failure` | bool (no dataset, zero samples, residual, crash) |

### 2.3 EpisodeArtifact — what is stored

| Field | Meaning |
|---|---|
| `raw_streams` | `trace/{clock,command,state,contact,body_pose,imu}_trace.jsonl` + `phase4b_summary.json` |
| `canonical_samples` | `dataset/{manifest.json,samples.jsonl}` (frozen schema v1.0.1) |
| `manifest` | episode manifest (`manifest.json`) |
| `provenance` | `provenance.json` |
| `integrity` | `episode_integrity.json` |
| `evaluation` | sidecar `evaluation.json` (outside the episode dir) |
| `publication_state` | `staging` / `committed` / `aborted` / `quarantined` |

### 2.4 EvaluationResult — four strictly separated axes

```text
episode_integrity     PASS | FAIL      -> is the acquired DATA coherent?   (acquisition)
measurement_quality   VALID|UNUSABLE   -> is the data SCIENTIFICALLY usable?
research_contract     PASS | FAIL | NOT_EVALUATED   -> Stage 7 / P1-R gates (reserved in Phase B)
gait_outcome          e.g. {walking_pass, fall_detected, yaw_change, drift}
```

Invariants (constraint #8, must never be re-coupled):

```text
episode_integrity = PASS  AND  gait_outcome = FALL        -> allowed, valid episode
episode_integrity = PASS  AND  research_contract = FAIL   -> allowed, valid episode
gait FAIL  =>  episode invalid                            -> FORBIDDEN
research_contract FAIL => discard episode                 -> FORBIDDEN
```

---

## 3. Target pipeline (data flow)

```text
            ┌───────────────────────────────┐
 Gait θ  ──▶ │  EpisodeSpec                   │
 + seed      │  (id, θ, durations, seed, …)   │
 + episode_id└───────────────┬───────────────┘
                             │
                             ▼
     ╔════════════════════════════════════════════════════════════╗
     ║  RUNNER  (live: process + seed + provenance)                ║
     ║  prepare -> launch (gz sim + producer + trace + collector)  ║
     ║  -> monitor -> capture gz argv -> verify seed -> terminate  ║
     ║  -> reap residuals -> provenance -> episode atomic publish  ║
     ╚═══════════════════════════┬════════════════════════════════╝
                                 │ RAW EPISODE (staging/)
                                 │  dataset/ trace/ collector_summary.json
                                 │  launch_cmd.txt gz_sim_cmd.*.txt run_config.json launch.log
                                 ▼
     ╔════════════════════════════════════════════════════════════╗
     ║  COLLECTOR  (canonicalization + acquisition facts)          ║
     ║  accounting closure -> duration -> gait-relative coverage   ║
     ║  -> EPISODE INTEGRITY verdict -> assemble manifest          ║
     ║  -> dataset atomic publish (inner) ; Runner does episode    ║
     ║     atomic publish (outer)                                  ║
     ╚═══════════════════════════┬════════════════════════════════╝
                                 │ CANONICAL EPISODE (immutable)
                                 ▼
     ╔════════════════════════════════════════════════════════════╗
     ║  EVALUATOR  (offline, pure, idempotent, read-only)          ║
     ║  measurement_metrics -> measurement_quality -> gait_outcome ║
     ║  -> research_contract   + independent integrity VERIFY      ║
     ╚═══════════════════════════┬════════════════════════════════╝
                                 ▼
                       evaluation.json (sidecar, OUTSIDE episode)
```

Clock-time reality (compressed in the diagram): the **live collector node**
(`phase4d_collector.py`) runs *inside* the Runner's Gazebo session as the acquisition
driver; the **COLLECTOR boundary** is the *finalize/canonicalize* responsibility that also
owns the acquisition facts. The **EVALUATOR** runs strictly after publication, read-only.

---

## 4. Ownership matrix (authoritative — disputes resolved here)

| Capability | Runner | Collector | Evaluator |
|---|---|---|---|
| seed | **Owner** | observe | verify |
| Gazebo lifecycle | **Owner** | observe | none |
| process lifecycle | **Owner** | observe | none |
| raw ROS/Gazebo stream | trigger | **Owner** | none |
| canonical samples | none | **Owner** | read |
| **episode_integrity verdict** | execution-level **input** | data-level **input** | **verify** (independent, read-only) |
| measurement quality | none | none | **Owner** |
| gait outcome | none | none | **Owner** |
| research contract | none | none | **Owner** |
| provenance | **Owner** | contribute | verify |
| metrics | none | none | **Owner** |

**Disputed row resolved (integrity).** The matrix deliberately lists no single "Owner"
for `episode_integrity`. Resolution adopted by this RFC:

- The **Collector owns the canonical integrity verdict** (it owns acquisition coherence
  and the canonical episode, per Phase B-3).
- The **Runner supplies execution-level inputs** (termination, residual, seed
  verification) and the **Collector supplies data-level inputs** (accounting closure,
  schema validity via the golden validator, duration/coverage).
- The **Evaluator independently *verifies*** the integrity verdict by re-deriving it
  read-only; it never overwrites it. This keeps integrity verifiable by a second party
  (satisfying the "verify" column) while keeping a single writer (Collector).

Mechanical form (see `Phase_B_Acceptance_Checklist.md`, Gate B1-13):

```text
Runner   -> imports Collector (calls finalize) ;  never imports Evaluator
Collector-> imports golden validator + pure model ;  never imports Runner/Evaluator
Evaluator-> imports pure model only ;  never imports Runner/Collector
```

---

## 5. Episode state machine

```text
   CREATED
      │  (staging dir created, config resolved, guards passed)
      ▼
   STARTING        ── launch; new session/pgid; env sanitized
      │
      ▼
   RUNNING         ── gz sim + producer + trace + live collector alive
      │  (first /joint_states => model ready)
      ▼
   COLLECTING      ── live collector writing dataset/ + summary
      │  (collector_summary.json non-empty; finish_reason set)
      ▼
   FINALIZING      ── capture gz argv; SIGINT->TERM->KILL; reap; residual check;
      │               Collector finalize: closure+duration+coverage+integrity;
      │               write manifest/provenance/integrity; pre-publish validation
      ▼
   COMMITTED       ── RENAME staging -> out_root (atomic); immutable

Failure states (terminal):
   FAILED                 generic unrecoverable error (no usable episode)
   ABORTED                operator/Ctrl-C or explicit abort
   EARLY_TERMINATED       finish_reason abnormal OR tail truncation > 0.5 s
   INFRASTRUCTURE_FAILURE no dataset / zero samples / residual>0 / crash
```

### 5.1 State permissions

| State | May write raw streams | May write canonical samples | May publish episode | Usable episode? |
|---|---|---|---|---|
| CREATED | no | no | no | no |
| STARTING | yes | (staging only) | no | no |
| RUNNING | yes | (staging only) | no | no |
| COLLECTING | yes | (staging only) | no | no |
| FINALIZING | no new samples | writes manifest/integrity (staging) | no | no |
| COMMITTED | no | no (frozen) | **yes (once)** | **yes** (integrity PASS or disclosed FAIL) |
| FAILED/ABORTED | — | — | no | no (staging preserved) |
| EARLY_TERMINATED | — | — | yes, **marked** integrity FAIL | retained for audit |
| INFRASTRUCTURE_FAILURE | — | — | yes, **marked** integrity FAIL | retained for audit |

Trigger ownership: `CREATED→STARTING→RUNNING→COLLECTING` = Runner; `COLLECTING→FINALIZING`
= Runner (lifecycle) + Collector (finalize); `FINALIZING→COMMITTED` = Collector+Runner
(publish); failure states = whichever boundary detects them.

### 5.2 Idempotency, retry, cleanup

- **Idempotency:** an episode_id + out_root pair is published at most once. Re-running
  requires `--allow-overwrite`; otherwise the run is refused. The Evaluator is idempotent
  (same input → identical output; episode unchanged).
- **Retry:** no automatic retry inside the Runner. Retry = a **new** `episode_id` (or an
  explicit overwrite). Retry **policy** belongs to the future batch layer.
- **Cleanup:** process group terminated with escalation (SIGINT→SIGTERM→SIGKILL);
  `residual_processes == 0` is a hard gate (else integrity FAIL). Staging is replaced on
  success; on abort, staging is preserved for post-mortem (never published).
- **Crash recovery:** because publish is a single atomic `os.replace`, a crash can never
  leave a half-written episode at the final path. A leftover `<out_root>.staging.<pid>`
  is explicitly **not** a committed episode; a recovery tool MAY re-finalize from it
  (design note), otherwise it is discarded.

---

## 6. Failure taxonomy (7 classes — not everything is "failure")

| Class | Detected by | Maps to `episode_integrity` | Episode retained? | Feeds which axis |
|---|---|---|---|---|
| `INFRASTRUCTURE_FAILURE` | Runner | **FAIL** | yes (audit) | integrity |
| `COLLECTION_FAILURE` | Collector | **FAIL** | yes (audit) | integrity (subclass of infra) |
| `DATA_INTEGRITY_FAILURE` | Collector (golden validator) | **FAIL** | yes (audit) | integrity |
| `MEASUREMENT_QUALITY_FAILURE` | Evaluator | PASS | **yes** | measurement_quality |
| `RESEARCH_CONTRACT_FAILURE` | Evaluator | PASS | **yes** | research_contract |
| `GAIT_OUTCOME_FAILURE` | Evaluator | PASS | **yes** | gait_outcome |
| `EARLY_TERMINATION` | Collector | **FAIL** | yes (audit) | integrity (mapped) |

Hard rules:

```text
Only INFRASTRUCTURE / COLLECTION / DATA_INTEGRITY / EARLY_TERMINATION may set
episode_integrity = FAIL.
MEASUREMENT_QUALITY / RESEARCH_CONTRACT / GAIT_OUTCOME failures NEVER set
episode_integrity = FAIL.
NO failure class ever deletes an episode.
GAIT_OUTCOME_FAILURE  !=  EPISODE_INTEGRITY_FAILURE   (invariant)
```

### 6.1 Integrity failure classes (frozen from Phase A)

- `infrastructure_failure`: no dataset dir, zero samples, residual > 0, dataset not
  committed.
- `data_integrity_failure`: dataset exists but signals missing / corrupt / schema-invalid
  (golden validator) / timestamps non-monotonic.
- Early termination is detected from `finish_reason ∉ {duration_complete, max_samples}`
  **or** `gait_tail_truncation_sec > 0.5`.

---

## 7. Data lifecycle (staging → validation → atomic publish)

```text
staging/                                  (NOT a valid episode)
  dataset/{manifest.json,samples.jsonl}   written by the live collector (dataset-level
                                           atomic commit inside staging)
  trace/*.jsonl + phase4b_summary.json    written by phase4b_trace
  collector_summary.json                  written by the live collector
  launch_cmd.txt / gz_sim_cmd.{actual,expected}.txt / run_config.json / launch.log

        │  FINALIZE
        │  1. Collector: accounting closure + duration + coverage + integrity verdict
        │  2. Collector: assemble manifest
        │  3. Runner: write provenance.json
        │  4. pre-publish validation (golden validator already inside collector commit;
        │     any additional checks are read-only)
        ▼
   os.replace(staging -> out_root)         (Runner; single atomic rename)
        ▼
out_root/                                  FINAL, IMMUTABLE CANONICAL EPISODE
  manifest.json  provenance.json  episode_integrity.json
  dataset/  trace/  launch artifacts

        │  EVALUATE (after publish, read-only)
        ▼
<out_root>.evaluation.json                (sidecar, OUTSIDE the episode dir)
```

Answers to the required lifecycle questions:

- **When is manifest written?** FINALIZING (staging), by the Collector; transported to
  final by the atomic rename.
- **When is provenance written?** FINALIZING (staging), by the Runner.
- **When does the validator run?** Inside the collector's dataset commit (pre-commit
  golden validation), and again read-only as the integrity cross-check.
- **When is integrity computed?** FINALIZING, by the Collector; verified read-only by the
  Evaluator post-publish.
- **When does evaluation run?** After COMMITTED, by the Evaluator; output lands outside
  the episode.
- **When is COMMITTED allowed?** Only after every staging artifact is written, validation
  passes (or the failure is explicitly classified and disclosed), and the rename succeeds.
- **Crash recovery?** See §5.2. Publish is atomic; partial states are never visible at the
  final path.
- **Partial episode storage?** A non-committed staging dir is preserved for audit but is
  never presented as a valid episode; a committed FAIL episode is retained and marked.

---

## 8. Duration & coverage semantics (INHERITED — frozen, no implicit change)

Inherited verbatim from Phase A closure (see Freeze Record §4). Any change would require
old semantics, new semantics, rationale, and schema/version impact — none are proposed.

```text
requested_gait_duration_sec   user-requested gait window                     (denominator)
record_window_duration_sec    start_delay + gait + stop_ramp + hold_final + margin
execution_duration_sec        observed gz sim-time span (clock trace); null if absent
accepted_sample_span_sec      last - first accepted sample timestamp (NOT execution)
gait_relative_coverage        union-of-intervals clipped to [gait_start, gait_end],
                              gap > 0.10 s => uncovered; denominator = requested
gait_head_gap_sec             window head not covered (rejected prefix)
internal gaps                 any consecutive interval > 0.10 s => uncovered (gap-capped)
gait_tail_truncation_sec      window end missed by accepted samples
early termination             finish_reason abnormal OR tail_truncation > 0.5 s
coverage_status               FULL (>= 0.999) | PARTIAL | null
legacy actual_duration_sec /  DEPRECATED aliases of accepted_sample_span_sec (labelled)
manifest duration_sec
```

Division of labour: the **Collector owns** these blocks; the **Evaluator reads** them
(for `measurement_quality`) without recomputing the acquisition semantics.

---

## 9. Seed provenance chain (must be provable end-to-end)

```text
EpisodeSpec.seed
      │
      ▼
Runner:  gz_seed:=<seed>   on the ros2 launch argv
      │
      ▼
launch:  gz sim ... --seed <seed>
      │
      ▼
Runner:  /proc capture of the REAL gz sim argv  (not the expected command)
      │
      ▼
provenance.seed = {requested, actual_gz_sim, verified, partition, ros_domain_id, semantics}
      │
      ▼
Evaluator: independently verifies (requested == actual) and that verification flag is honest
```

Forbidden: writing only `requested_seed = 12345` without proving Gazebo used 12345.
`GZ_PARTITION` / `ROS_DOMAIN_ID` are **communication isolation, not physics seeds**.
`seed_verified = (requested == actual_gz_sim)`.

---

## 10. Repeatability protocol (DESIGN ONLY — not run in this stage)

Phase B must eventually answer: *given fixed θ and a fixed seed, how reproducible is an
episode?* The RFC reserves a **repeatability probe**:

```text
inputs:   fixed θ, fixed seed, N episodes (N chosen later; e.g. 5..20), same world/engine
method:   run N episodes via the Kernel; no parameter variation
report:   per-metric mean / variance / std / min / max
          sample_count, gait_relative_coverage, gait_head_gap
          outcome rate (walking_pass, fall rate)
          infrastructure_failure rate, measurement_quality stability
          seed-reproducibility check (does the same seed reproduce identical physics?)
```

Constraints:

- **Do NOT run 10–20 episodes now.** This stage defines the protocol only.
- No gait optimization; no parameter sweep; no new research claims.
- Batch-level statistics (episode-to-episode variance) are strictly distinct from
  per-episode per-frame std (`measured_contract.md` §4) — must never be mixed.

---

## 11. Interfaces (proposed — concrete, implementation-deferred)

```python
# tools/nao_episode_model.py  (PURE shared kernel; stdlib only; no ROS; no side effects)

@dataclass(frozen=True)
class EpisodeFacts:
    episode_dir: str; sample_count: int
    timestamps: tuple[float, ...]
    first_ts: float | None; last_ts: float | None
    clock_first: float | None; clock_last: float | None
    state_first: float | None
    joint_order: tuple[str, ...]

def read_episode(episode_dir) -> EpisodeFacts: ...
def coverage_block(timestamps, gait_start_sec, gait_end_sec, gap_cap_sec=0.10) -> dict: ...
def derive_gait_start(episode_dir, trace_dir) -> tuple[float | None, str]: ...
def duration_block(*, requested, record_window, execution, accepted_span) -> dict: ...
```

```python
# tools/nao_episode_runner.py  (Runner)

@dataclass(frozen=True)
class EpisodeSpec: ...          # see §2.1

@dataclass(frozen=True)
class EpisodeExecution: ...     # see §2.2

class EpisodeRunner:
    def run(self, spec: EpisodeSpec) -> EpisodeExecution: ...   # ends at RUNNING/SESSION done
def publish(staging_dir, out_root) -> None: ...                 # atomic os.replace
```

```python
# tools/nao_episode_collector.py  (Collector)

@dataclass(frozen=True)
class CanonicalVerdict:
    episode_integrity: str; failure_class: str | None; termination: str
    duration: Mapping[str, Any]; coverage: Mapping[str, Any] | None
    accounting: Mapping[str, Any]; signals: Mapping[str, str]
    checks: Mapping[str, Any]; sample_count: int

class EpisodeCollector:
    def finalize(self, execution, *, requested_gait_duration_sec,
                 record_window_duration_sec) -> CanonicalVerdict: ...
    def manifest(self, spec, execution, verdict) -> Mapping[str, Any]: ...
```

```python
# tools/nao_episode_evaluator.py  (Evaluator)

@dataclass(frozen=True)
class Evaluation:
    episode_id: str
    measurement_metrics: Mapping[str, Any]
    measurement_quality: Mapping[str, Any]
    gait_outcome: Mapping[str, Any]
    research_contract: Mapping[str, Any]      # reserved (NOT_EVALUATED) in Phase B
    integrity_verification: Mapping[str, Any] # independent re-derivation of the verdict
    evaluator_version: str

def evaluate(canonical_episode_dir, eval_out=None) -> Evaluation: ...
    # eval_out default "<episode>.evaluation.json"; NEVER writes inside the episode
```

Compatibility: `tools/run_phase_a_episode.py` keeps its exact CLI and smoke output
(delegates to the Kernel); `tools/nao_episode_integrity.py` stays importable as a facade.

---

## 12. Proposed files by sub-phase (no implementation in this stage)

| Sub-phase | File | Kind | Purpose |
|---|---|---|---|
| B2 | `tools/nao_episode_runner.py` | add | Runner: live lifecycle + provenance + publish |
| B2 | `tools/run_phase_a_episode.py` | modify | Kernel/CLI wrapper (behavior identical) |
| B2 | `tools/run_p0a_gait_candidate.py` | modify (thin) | helpers re-exported / relocated |
| B3 | `tools/nao_episode_collector.py` | add | finalize: closure/duration/coverage/integrity |
| B3 | `tools/nao_episode_model.py` | add | pure canonical-episode reader/derivations |
| B3 | `tools/nao_episode_integrity.py` | modify | facade delegating to Collector/Evaluator |
| B3 | `tools/test_phase_b_internal_gap.py` | add | P2-B internal-gap regression |
| B3 | `phase4d_collector.py` | modify (additive, audit-gated) | P2-C instrumentation (summary fields) |
| B4 | `tools/nao_episode_evaluator.py` | add | metrics/quality/outcome/contract/verify |
| B4 | `tools/test_phase_b_evaluator_purity.py` | add | idempotency + read-only proof |
| B? | `tools/run_phase_b_batch.py` | add (deferred) | declared-list batch runner (design only) |
| — | `Phase_B_Implementation_Report.md` | add (later) | final implementation report |

Ordering: **B2 → audit → B3 → audit → B4 → audit** (never all at once).

---

## 13. Migration & regression risk

| Risk | Severity | Mitigation |
|---|---|---|
| Coverage/duration semantics drift during extraction | HIGH | freeze tests against `smoke_01`/`closure_01`; exact-value assertions |
| Facade drift (EI facade vs new modules) | MED | facade delegates; single-source derivations in `model.py` |
| Runner→Evaluator coupling leaks into B4 | MED | acyclicity import test enforced at the end state |
| P2-C instrumentation becomes a gate | HIGH | additive summary fields only; no verdict reads them |
| Launch argv / seed chain silently changes in B2 | HIGH | reproduce `launch_cmd.txt` / `gz_sim_cmd.actual.txt` semantics; seed test |
| Historical evidence rewritten | HIGH | read-only hash check before/after each sub-phase (Freeze Record §5) |
| Pre-existing 4 test FAILs misattributed | MED | compare failing set byte-for-byte; report **"no demonstrated regression"**, not "proven" |

---

## 14. Non-goals (explicit)

```text
NOT IN SCOPE (Phase B-1):
- Runner implementation
- Collector rewrite
- Evaluator implementation
- gait algorithm redesign
- gait optimization / RL / parameter search
- Stage 7 rerun / large-scale dataset generation
- new research claims
- replacing or weakening Phase A acceptance semantics
- changing dataset schema v1.0.1 or the golden validator
- running a multi-episode batch

Phase B-1 is an architecture / RFC phase, not an implementation phase.
```

Forbidden in this stage: touching gait math; writing Runner/Collector/Evaluator; starting
batch collection or Stage 7; re-coupling gait FAIL with data invalid;
changing schema semantics without an explicit version decision; deferring interface
questions with "define later".

---

## 15. Responsibilities Q&A index (prior brief, 20 questions)

Q1 Runner responsibilities → §4; Q2 Collector → §4; Q3 Evaluator → §4; Q4 interfaces →
§11; Q5 lifecycle → §5; Q6 artifact ownership → §2.3/§7; Q7 episode-dir ownership → §7;
Q8 provenance ownership → §9; Q9 schema ownership → Freeze Record §6; Q10 failure
ownership → §6; Q11 termination ownership → §6; Q12 seed propagation → §9; Q13 duration
semantics → §8; Q14 coverage semantics → §8; Q15 P2-C strategy →
`Phase_B_Carry_Forward_Register.md`; Q16 backward compatibility → §12 + Freeze Record;
Q17 testing strategy → `Phase_B_Acceptance_Checklist.md`; Q18 migration → §12; Q19 batch
production → §10 + §12; Q20 non-goals → §14.

---

## 16. Status

```text
PHASE_B1_RFC_STATUS = READY
```

Self-audit against Gates B1-1…B1-14 is in `Phase_B_Acceptance_Checklist.md`.
**Awaiting the Independent Audit Agent.** Implementation may only begin after
`PHASE_B_RFC_ACCEPTED`; the final implementation state may only be
`PHASE_B_IMPLEMENTATION_READY` or `PHASE_B_IMPLEMENTATION_BLOCKED` — never self-declared
`PHASE_B_ACCEPTED`.

End of RFC.
