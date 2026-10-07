# STAGE0_STAGE1_TRANSITION_REPORT

Stage 0 → Stage 1 transition report — closure decision, current dataset capability assessment,
Stage 1 RFC summary, and the next implementation boundary.

| Field | Value |
|---|---|
| Role | Implementation Agent (DeepSeek / Qoder role) |
| Date | 2026-10-07 |
| Basis | Stage 0 closure commit `045a21fdcf7d8d7261a0ad2fa0ab5e8acf041d73`; `docs/STAGE0_CLOSURE_FINAL_STATUS.md`; `docs/STAGE1_DYNAMICS_DATASET_RFC.md` (REV1.0-DRAFT) |

---

## Section 1 — Stage 0 Closure

```
Stage0 Closure:

PASS
```

Evidence (details in `docs/STAGE0_CLOSURE_FINAL_STATUS.md`):

- **BF-1 resolved:** all Stage 0 artifacts (governing RFC REV1.1, contract manifest, closure docs,
  θ list, corpus index, per-episode records/metrics/provenance, drivers) are git-tracked at commit
  `045a21f` (1263 files). A fresh `git clone` at that commit recovers the full contract identity,
  and all pinned hashes match inside the clone.
- **BF-2 resolved:** corpus complete per the pre-registered rings — 32 θ × 3 seeds = 96 episodes
  (ring0 = 3, ring1 = 42, ring2 = 30, ring3 = 21), failures preserved (`theta_list_hash`
  `844646f5…` verified fail-closed).
- **BF-3 resolved:** `corpus_index.json` deterministic (byte hash `260fd9e7…`) and self-verifying
  (`content_hash` `eaf879…` recomputed OK).
- **No frozen component touched:** `tools/run_phase_a_episode.py` unmodified; the Stage 0 RFC, the
  Stage 0 dataset, and the sampling contract are untouched; the closure only performed
  `git add` + `git commit`.
- Standing caveats (non-blocking, unchanged): GLM-5.3 independent audit still absent; 9/96 episodes
  `behavior_success = unknown`; raw traces excluded from the commit by owner decision
  (regenerable); no best-θ selection exists or is implied.

---

## Section 2 — Current Dataset Capability

**What exists: a behavior dataset.** The frozen Stage 0 corpus answers *"what did the gait do?"*
per (θ, seed): joint positions/velocities, commands, IMU, contact booleans, base pose, episode
behavior metrics, full provenance.

**What is missing: a dynamics dataset.** The program target
`F_NAO(θ,t) = F_LIPM(θ,t) + R_φ(θ,t)` requires observing forces and full-body dynamics. Measured
gaps in Stage 0 (facts, not assumptions):

| Missing observable | Root cause in the Stage 0 pipeline |
|---|---|
| CoM position / velocity / acceleration | link poses were available on `/world/nao_world/pose/info` but only `base_link` was consumed; nothing else persisted |
| Base / foot velocities | no twist stream ever recorded |
| Foot poses | `l_ankle` / `r_ankle` poses never recorded as streams |
| Contact force / moment | `contact_converter.cpp` reduces the raw `Contacts` messages (which carry `JointWrench`) to `Bool` — the wrench is destroyed at conversion time; the Stage 0 trace subscribes only the `Bool` |
| ZMP / CoP | never computed; underivable without measured wrenches |

**Conclusion:** the gap is irrecoverable post hoc — no re-processing of the frozen Stage 0 corpus
can produce dynamics observables. A fresh acquisition pass with extended (read-only)
instrumentation is required. Stage 0 answers behavior questions and remains the frozen baseline;
Stage 1 must supply the dynamics observation.

---

## Section 3 — Stage 1 RFC Summary

`docs/STAGE1_DYNAMICS_DATASET_RFC.md` (REV1.0-DRAFT, **specification only — no implementation
authorized**). In one line: *re-run the identical pre-registered 32 θ × 3 seeds grid with
read-only extended instrumentation, recording full-body dynamics and contact wrenches.*

- **Objective (§ 2):** physics identification, LIPM validation, residual-learning preparation,
  stability analysis. Explicitly not optimization, not best-gait search.
- **Schema (§ 3):** ten field groups — Time, Joint (24, canonical order), Base, CoM (from all 25
  link poses + pinned SDF inertials), Foot (ankle frames), Contact (**wrench + moment + validity**,
  booleans kept for Stage 0 comparability), ZMP/CoP (kinematics-derived `zmp` vs
  wrench-derived `cop`, deliberately independent), IMU, Command (+ deterministic `gait_phase`,
  `desired_velocity`), Provenance (θ/hash chain, git commit, config hash, seed, model versions,
  RFC hash).
- **Sampling (§ 4):** pre-registered now — verbatim Stage 0 θ list + same seeds (96 episodes);
  inherited prohibitions (no optimization-driven sampling, no discarding bad episodes, no early
  stopping); fail-closed tamper checks; new storage root `dataset/stage1/` (`dataset/stage0/`
  never written).
- **Measurement (§ 5):** per-frame `(s_t, u_t, s_{t+1})` tuples, self-contained frames,
  consecutivity-only pairing, no interpolation/fabrication of measured quantities, mark-don't-discard
  quality flags (5 % degraded threshold), read-only instrumentation.
- **LIPM compatibility (§ 6):** every quantity of `X = [c_x, c_y, ċ_x, ċ_y]`,
  `c̈ = ω²(c − p)` maps to a schema field; deviation `r(t) = c̈ − ω²(c − p_cop)` computable
  field-to-field; no LIPM implementation, no `ω` fit, no residual training in Stage 1.
- **Out of scope (§ 7):** CMA-ES / CEM / RL / MPC / PINN / any fitness or optimizer / any NN
  training / LIPM implementation / any frozen-artifact or pipeline modification / collector
  implementation itself.
- **Acceptance (§ 8):** RFC criteria R1–R7 (schema / sampling / provenance / sources / no
  optimization leakage / no implementation / Stage 0 immutability) all satisfied by the document;
  dataset criteria D1–D5 deferred to the implementation phase.
- **Gates (Appendix B):** V-1 (blocking) contact-wrench population; V-2 (blocking) pose-info
  completeness; V-3 (informational) native twist availability. V-1 failure blocks collection —
  no force-free fallback.
- **BF-1 prevention:** the RFC is un-ignored in `.gitignore` by an explicit negation rule
  (`!docs/STAGE1_DYNAMICS_DATASET_RFC.md`) and committed — the same durability guarantee as the
  Stage 0 RFC.

---

## Section 4 — Next Implementation Boundary

The only authorized next step, and only after this RFC is accepted (owner + Independent Auditor
TRAE / GLM-5.3):

> **Stage 1 instrumentation implementation** — a read-only collector that subscribes to the
> already-existing streams (Appendix A of the RFC), passes gates V-1/V-2 first, then acquires the
> 96 pre-registered episodes into `dataset/stage1/` under the § 3 schema and § 4 sampling contract.

Boundary conditions for that step:

1. Gates V-1 and V-2 pass **before** any episode is collected; V-1 failure blocks Stage 1 until the
   RFC is revised (no partial pivot, no force-free fallback).
2. The collector adds subscriptions only — no pipeline modification, no frozen-artifact
   modification (`tools/run_phase_a_episode.py`, Stage 0 dataset, Stage 0 RFC all untouched).
3. Everything forbidden by RFC § 7 stays forbidden during implementation: no optimizer, no
   learning, no LIPM fitting, no best-θ selection.
4. Later stages (LIPM prior, residual network, optimization, sim2real) each require their own
   authorization — the roadmap order is fixed:

```text
Baseline Dataset   (Stage 0 — frozen, PASS)
        ↓
Dynamics Dataset   (Stage 1 — RFC drafted, implementation pending authorization)
        ↓
LIPM Prior         (not started)
        ↓
Residual Network   (not started)
        ↓
Optimization       (not started)
        ↓
Sim2Real           (not started)
```

**Do not skip stages. Optimization happens last.**

---

Status: implementation work complete; awaiting owner decision and Independent Auditor
(TRAE + GLM-5.3) audit of the Stage 0 closure and the Stage 1 RFC.
