# STAGE0_PARAMETERIZED_GAIT_RFC_REV1

| Field | Value |
|---|---|
| RFC id | `STAGE0_PARAMETERIZED_GAIT_RFC` |
| Revision | **REV1.1-DRAFT** (REV1 + R-2 wording closure) |
| Supersedes | `STAGE0_PARAMETERIZED_GAIT_RFC.md` (REV0-DRAFT) |
| Status | **awaiting independent audit** — NOT accepted |
| Date | 2026-10-06 |
| Author role | Implementation Agent |
| Task type | **CONTRACT REPAIR ONLY** (contradiction removal) |
| Code changes | **0** (document only) |
| Repository | `nao_gazebo_ws`, branch `main`, HEAD `6d00f90b7d80060f5b42b7d64499699ec3100911` |

Stage 0 philosophy is preserved unchanged:

```
theta  →  gait generator  →  Gazebo  →  episode  →  measured behavior
```

This revision removes the internal contradictions and two defects identified for repair (F-1…F-7).
No capability is added, no scope is expanded, no frozen component is modified.

---

## 0. Preamble

### 0.1 Inputs, and one input gap (disclosed)

| Input | Availability | Use |
|---|---|---|
| `STAGE0_PARAMETERIZED_GAIT_RFC.md` REV0-DRAFT | **PRESENT** | basis of this revision |
| GLM-5.3 Independent Adversarial Audit Report (Stage 0) | **NOT FOUND** (re-searched: only `docs/GLM-5.3 Independent Acceptance - Audit Phase A.txt` and `docs/task_p0a_gait_search/GLM-5.3_Stage6_review_prompt.md` exist; no Stage 0 adversarial report) | **not read** |
| Repair directive F-1…F-7 | **SUPPLIED IN THE TASK BRIEF** | the operative change driver for REV1 |

Because the adversarial report itself is not present in the repository, REV1 is driven by the
**enumerated repair directive (F-1…F-7)** rather than by the report text. The findings cited here are
re-derived from repository state where possible (§0.3 lists what was independently re-verified).
REV1 does **not** claim to incorporate findings it has not seen; unresolved audit inputs are carried
in §11.

Independently re-verified for this revision (not taken on trust):

| Claim | Method | Result |
|---|---|---|
| fwdf08 `run_config.json` has 26 keys, no `gait_phase_mode` | parsed the file | confirmed (26 keys) |
| `70855de1…` is that artifact's hash | re-serialized with `json.dumps(indent=2, sort_keys=True, allow_nan=False)` | reproduces byte-for-byte |
| the 27-key anchor hash | same serializer on the 26 keys + `gait_phase_mode="legacy"` | `28e795b2…` (§3.2) |
| canonical sample period = 0.010 s | measured `state_trace`/`imu_trace` from two episodes | confirmed (min = median = max = 0.010 s) |
| body_pose spacing | measured `body_pose_trace` from two episodes | median 0.008 / 0.015 s; max gap 0.017 / 0.018 s |
| OPT-1A same-θ same-seed Δ | `grep` `OPT1A_OBJECTIVE_DIAGNOSIS.md` | `0.128` at lines 103, 149; caveat at line 440 (§5.3) |

### 0.2 Normative language

`MUST` / `MUST NOT` / `SHALL` = binding. `SHOULD` = strong default, deviation needs recorded rationale.
`MAY` = permitted.

### 0.3 Revision changelog (REV0 → REV1)

| F | Defect in REV0 | REV1 change | Section |
|---|---|---|---|
| **F-1** | REV0 mixed the *historical* 26-key fwdf08 artifact hash with the *Stage 0* 27-key anchor θ under a single `theta_hash`. The two objects differ (26 vs 27 keys) and cannot share one hash. | Split into **two** named identities: `historical_fwdf08_artifact_hash` (provenance/reference only) and `stage0_anchor_theta_hash` (the Stage 0 θ identity). Stage 0 `theta_hash` now **always** means the 27-key resolved canonical θ. `gait_phase_mode` **retained** in the explicit anchor. | §3.2, §4.6 |
| **F-2** | REV0 permitted multiple runners while the dataset contract requires one reproducible production path. | Exactly **one** canonical Stage 0 runner: `tools/run_phase_a_episode.py`, with rationale, artifact list, and the frozen-analyzer invocation contract. Wording corrected to **"driver MUST NOT derive metrics"** (not "must not invoke analyzer"); the driver **MAY orchestrate** frozen tools; the frozen analyzer stays authoritative. | §6.1, §8.1, §8.2 |
| **F-3** | Ring 3 was **adaptive** ("push until a boundary; stop at the first boundary"), which violates static pre-registration. | Ring 3 replaced by a **pre-declared failure-boundary coverage grid**: fully declared before execution, fully executed, no result-dependent expansion, **no early stopping**. Failure-oriented samples are allowed only if declared up front. | §7 |
| **R-2** | REV1 still described Ring 3 as spanning "on the axes that Rings 1–2 motivated" — a residual dependency on prior-ring results. | Ring 3 wording closed: θ list fixed **before any episode of any ring executes**; explicitly independent of Ring 1/Ring 2 outcomes and of failure episodes; adaptive-search language prohibited. | §7.1 |
| **F-4** | REV0 did not disclose the *mixed* temporal semantics of `s_t` (some components causal, one not). | Added a complete **temporal alignment table** (source / rule / causal? / tolerance) covering every state component, incl. `base_pose` causal last-not-after, `command` causal hold-last, `contact` causal last-not-after, and `imu` **non-causal symmetric nearest ±0.01 s, may include one future sample**. Disclosure only; collector and schema untouched. | §6.3 |
| **F-5** | REV0 attributed `yaw std ≈ 25.6°` to *same-seed* instability; it is a **cross-seed** dispersion. | Corrected label to cross-seed dispersion; same-seed instability now evidenced by **OPT-1A same-θ same-seed Δfitness = 0.128**, with its own caveat. | §5.3 |
| **F-6** | REV0 left the base-pose tolerance "declared once" — unspecified. | Tolerance **frozen at 0.02 s**, with value, measured reason, and no-per-episode-tuning rule. | §6.4 |
| **F-7** | REV0's provenance vocabulary had 3 categories. | Extended to **4**: `anchor_declaration`, `sampling_dimension`, **`runner_cli_argument`** (new), `runner_default`; plus a `transport` field. Explicitly applied to `duration_sec` and `hold_final_sec`. | §4.6 |

### 0.4 Changed sections index

| Section | REV1 status |
|---|---|
| §1 Objective and Scope | unchanged (non-goals restated for completeness) |
| §2 Frozen Repository Boundary | unchanged; §2.4 added (canonical runner pointer) |
| §3 Canonical Anchor Contract | **CHANGED (F-1)** |
| §4 Parameter Contract | **CHANGED (F-7)** |
| §5 Seed and Repeatability Contract | **CHANGED (F-5)** |
| §6 Dataset Contract | **CHANGED (F-2, F-4, F-6)** |
| §7 Sampling Strategy | **CHANGED (F-3)** |
| §8 Minimal Implementation Boundary | **CHANGED (F-2 wording)** |
| §9 Acceptance Criteria | **CHANGED** (criteria tightened to the repaired contracts) |
| §10 No-Code-Change Declaration | new (REV1 deliverable) |
| §11 Remaining Open Issues | new (REV1 deliverable) |

Preserved without change (explicitly confirmed in §9.1): 27-parameter anchor, `sway_sign = -1`,
`gait_phase_mode = legacy`, no optimization, no learning, frozen repository boundary, only thin driver
+ corpus index added, no schema upgrade.

---

## 1. Objective and Scope

### 1.1 Objective

Stage 0 SHALL establish an **auditable data-production closed loop** for a parameterized NAO gait
baseline:

```
theta  →  gait generator  →  Gazebo  →  episode  →  measured behavior
```

For every produced episode, a third party MUST be able to answer from the artifacts alone: **which θ,
which seed, which code, which configuration, and what was measured.**

Stage 0 produces **data about the existing gait system**. It does not improve the gait.

### 1.2 In scope

1. An explicit, frozen **anchor** θ (§3).
2. An explicit **θ schema** with validation, rejection, and per-key provenance (§4).
3. An explicit **seed and repeatability** contract (§5).
4. A **dataset contract** at episode level and trajectory level, on one canonical runner (§6).
5. A **pre-declared, static** sampling strategy (§7).
6. A strictly bounded **implementation surface** (§8).

### 1.3 Non-goals (MUST NOT appear in Stage 0 work)

- **Gait optimization**: no objective, no fitness, no search/optimizer, no best-θ selection.
- **Learning**: no fitting, no learned component, no training.
- **Physics model changes**: world / robot model / engine / contact / friction / mass are frozen (§2).
- **Controller redesign**: the actuator servo and its command contract are frozen (§2).
- Any future program (planner / estimator / deployment) — MUST NOT be designed in this RFC.

### 1.4 The single question Stage 0 answers

> Given an explicit θ, what behavior does the current, unmodified NAO gait system actually produce, and
> can that be proven from the stored artifacts?

---

## 2. Frozen Repository Boundary

### 2.1 Components that MUST NOT be modified

| Layer | Frozen path(s) | Reason |
|---|---|---|
| Gait producer (gait math) | `src/nao_gz_sim/scripts/phase4b_producer.py`, `src/nao_gz_sim/scripts/nao_gait_phase.py` | the object under measurement |
| Collector | `src/nao_gz_sim/scripts/phase4d_collector.py` | canonicalization + accounting closure |
| Trajectory schema | `src/nao_gz_sim/scripts/nao_trajectory_schema.py` (v1.0.1) | frozen dataset contract |
| Trajectory validator | `tools/validate_task4_phase4d_dataset.py` | golden validator |
| Analyzer + window contract | `tools/analyze_measured_gait.py`, `tools/phase4e_analysis.py` | redefining it silently redefines every metric |
| Controller | `src/nao_gz_sim/src/phase4a_actuator.cpp` | frozen command contract + servo |
| Physics / world / model | `src/nao_gz_sim/worlds/nao_world.sdf`, `src/nao_description/urdf/nao_simple.sdf`, `nao_simple.urdf` | canonical model (actuator is injected into a temp runtime SDF, never the file) |
| Launch | `src/nao_gz_sim/launch/nao_sim.launch.py` | parameter fan-out contract |
| Existing runners | `tools/run_p0a_gait_candidate.py`, `tools/run_phase_a_episode.py` | hash-pinned by `Phase_B0_Freeze_Record.md` §3 |
| Integrity | `tools/nao_episode_integrity.py` | four uncoupled verdict axes |
| Historical evidence | `docs/task4_phase4e_pathb_b3_evidence/**`, `docs/task_p0a_gait_search/phase_a_*`, `.../work/reproduce/fwdf08_*`, `.../stage7_*_evidence/**`, `backup/**` | frozen artifacts |

### 2.2 What Stage 0 MAY add (only this)

1. **Thin sampling driver** — a new file that *invokes* the canonical runner unmodified (§8).
2. **Corpus index** — a new file recording episode-level provenance across the run (§6.7).

### 2.3 Boundary invariants

- Dependency direction is strictly `driver → frozen components`, never the reverse. No frozen file
  SHALL import or require a Stage 0 artifact.
- No frozen file's behavior SHALL depend on the presence of Stage 0 artifacts.

### 2.4 Canonical runner (new cross-reference)

Exactly one runner is the Stage 0 production path: **`tools/run_phase_a_episode.py`** (§6.1).
The non-canonical runner `tools/run_p0a_gait_candidate.py` MAY be used **read-only for cross-checks**
and MUST NOT be the production path (§6.2).

---

## 3. Canonical Anchor Contract

### 3.1 Definition

The canonical Stage 0 anchor is:

> **`fwdf08-stage0-anchor`** — the θ of the validated forward-gait candidate `fwdf08`
> (`docs/task_p0a_gait_search/work/reproduce/fwdf08_seed{101,202,303}/run_config.json`), extended with
> the anchor's own explicitly-declared timing **and** an explicit `gait_phase_mode`.

`fwdf08` is the only configuration in the repository with a 3-seed validated forward result
(mean forward 0.1235 m, std 0.0047 m; fall = 0; `work/reproduce/fwdf08_gate.json`).

### 3.2 Two distinct identities (F-1 fix)

REV0 conflated two different objects. They are now **separate, named, and non-interchangeable**.

| Identity | Meaning | Value | Permitted use |
|---|---|---|---|
| **`historical_fwdf08_artifact_hash`** | SHA-256 of the **original 26-key** fwdf08 `run_config.json` artifact | `70855de120a1d99faa53be713886b90a5a336ff6e651b5830b97815871fa2182` | **provenance / reference only.** MUST NOT be used as a Stage 0 θ identity or compared against a Stage 0 θ. |
| **`stage0_anchor_theta_hash`** | SHA-256 of the **27-key** resolved Stage 0 anchor θ (the 26 keys + `gait_phase_mode="legacy"`) | `28e795b29921a9061c910b53aaccddbf88ecd5f300ad24ea93079cb5c4e94998` | **the Stage 0 anchor identity.** This is what `theta_hash` means (§4.6). |

Why the 26 ≠ 27: the historical fwdf08 artifact **did not carry** `gait_phase_mode` (it inherited the
producer default `legacy`), whereas the Stage 0 anchor **declares it explicitly** (§3.4). The two
serialized objects therefore differ in length and digests.

**Hash method (frozen).** Canonical serialization of a θ object is
`json.dumps(obj, indent=2, sort_keys=True, allow_nan=False)` as UTF-8 **with no trailing newline** —
byte-identical to the serialization the runners already write for `run_config.json`
(`tools/run_p0a_gait_candidate.py:263-265`). This method was validated in this revision: re-serializing
the historical artifact reproduces `70855de1…` byte-for-byte, therefore the 27-key digest `28e795b2…`
is trustworthy.

**Consistency rules (normative).**

- A Stage 0 `theta_hash` **MUST** be computed over the **27-key resolved canonical θ** and nothing else.
- An episode whose resolved θ has 26 keys (i.e. `gait_phase_mode` absent) is **out of contract**.
- `historical_fwdf08_artifact_hash` **MUST** be recorded, when present, under a field name that is
  visually and semantically distinct (e.g. `reference.historical_fwdf08_artifact_hash`) and **MUST NOT**
  occupy the `theta_hash` field.

### 3.3 Explicit parameter declaration (27 keys — all MUST be present)

| # | Parameter | Anchor value | Note |
|---|---|---|---|
| 1 | `gait_method` | `walk` | |
| 2 | `gait_phase_mode` | `legacy` | **explicit** — historical fwdf08 ran legacy (key absent); REV1 keeps it explicit |
| 3 | `step_length_m` | `0.02` | |
| 4 | `step_frequency_hz` | `0.8` | |
| 5 | `swing_height_m` | `0.008` | |
| 6 | `double_support_ratio` | `0.3` | |
| 7 | `leg_extension_m` | `0.24401` | |
| 8 | `sway_amplitude_rad` | `0.06` | |
| 9 | `sway_sign` | `-1` | **explicit** — behavior-critical |
| 10 | `foot_pitch_swing_rad` | `0.1` | |
| 11 | `walking_direction` | `forward` | |
| 12 | `command_rate_hz` | `100.0` | |
| 13 | `start_delay_sec` | `0.5` | |
| 14 | `gait_blend_sec` | `1.5` | |
| 15 | `duration_sec` | `16.0` | **explicit** — see §3.5 |
| 16 | `stop_ramp_sec` | `1.5` | |
| 17 | `hold_final_sec` | `1.0` | **explicit** — see §3.5 |
| 18 | `stand_sec` | `24.0` | behaviorally inert for `walk`, but part of the 27-key identity (§3.6) |
| 19 | `arm_swing_rad` | `0.12` | |
| 20 | `arm_neutral_pitch_rad` | `1.5` | |
| 21 | `arm_neutral_roll_rad` | `0.0` | |
| 22 | `arm_phase_offset_rad` | `0.0` | |
| 23 | `arm_swing_sign` | `1` | |
| 24 | `elbow_bend_rad` | `0.3` | |
| 25 | `actuator_p_gain` | `15.0` | |
| 26 | `actuator_max_velocity` | `4.0` | |
| 27 | `actuator_hold_on_start` | `true` | |

### 3.4 Forbidden inheritance (normative)

- The anchor MUST NOT rely on `EXPERIMENT_DEFAULTS` (`tools/run_p0a_gait_candidate.py:42-46`) as the
  **source** of any value. Rows 15–17 carry the same numbers those defaults would have supplied
  **deliberately**, but they MUST arrive as explicit declarations. (The canonical runner does not apply
  `EXPERIMENT_DEFAULTS` at all — §6.2 — so this is reinforced, not merely asserted.)
- The anchor MUST NOT inherit **`B3-ANCHOR`**
  (`docs/task4_phase4e_pathb_b3_evidence/b3_0/candidates.json`, sha256 `116dde70…bc13b`):

  | Property | `B3-ANCHOR` (NOT the anchor) | `fwdf08-stage0-anchor` |
  |---|---|---|
  | `sway_sign` | absent → launch default `+1` | `-1` (explicit) |
  | `gait_phase_mode` | absent → default `legacy` | `legacy` (explicit) |
  | `step_frequency_hz` | `0.5` | `0.8` |
  | observed mean forward | ≈ 0.0022 m (in-place shuffle) | 0.1235 m |
  | role | optimizer baseline only | **Stage 0 anchor** |

- The anchor MUST NOT be defined by the `FROZEN_GAIT` dict in `tools/run_phase_a_episode.py:62-88`
  (that dict uses `gait_phase_mode=explicit_support_swing`, a different gait family). Reusing the
  **runner** is required (§6.1); reusing that **dict as the anchor definition** is forbidden.

### 3.5 Anchor execution vs anchor θ

Stage 0 execution SHALL wire an explicit physics seed (§5), which the historical fwdf08 runs did not
(their `launch_cmd.txt` contains no `gz_seed:=`). Therefore the anchor **θ** is frozen exactly as
§3.2/§3.3, while the Stage 0 anchor **run** is a *re-baseline* of that θ under explicit seed and full
provenance, and MUST NOT be presented as a byte-identical reproduction of the historical runs.

### 3.6 Resolution requirement

To make `theta_hash` equal to `stage0_anchor_theta_hash`, the canonical runner MUST resolve **exactly
the 27 keys of §3.3** — no more, no fewer. Concretely the driver MUST pass all 27 as explicit overrides
(including `stand_sec`, which the runner's `FROZEN_GAIT` background does not contain). A resolved θ with
any additional, missing, or renamed key yields a different digest and is out of contract.

---

## 4. Parameter Contract (θ schema)

### 4.1 θ object shape

```
theta = {
  "frozen_background":   { ... },   # constant across the Stage 0 run
  "sampling_dimensions": [ ... ]    # declared subset of eligible axes; the only keys that vary
}
```

The **resolved** value of every key MUST be recorded per episode (§6.1), not just the deltas.

### 4.2 Required keys

A θ object MUST contain all 27 keys of §3.3. A θ missing any required key MUST be **rejected before any
runner is invoked**.

### 4.3 Unknown-key rejection (fail-closed)

- The driver MUST reject any key not in the declared θ schema, and MUST abort on the first occurrence.
- Rationale (verified): overrides become launch arguments by mechanically prefixing
  `phase4b_`/`actuator_` (`tools/run_p0a_gait_candidate.py:90-98`). A key with no matching launch
  declaration (e.g. the stale `com_forward_shift_m` example at `:17`) is accepted by the driver and can
  never influence the gait. The `ros2 launch` behavior for such a key is **unverified** (audit item
  `UNKNOWN`); Stage 0 MUST NOT depend on it and rejects unknown keys itself.
- The driver MUST additionally assert that every retained key is a **launch-declared** argument. A key
  that is producer-declared but not launch-exposed (§4.5) is **not** eligible.

### 4.4 Range validation

Each axis MUST be validated against the **producer's own** acceptance ranges (do not invent bounds).
Source: `phase4b_producer.py` `_validate_parameters()` (`:306-366`).

| Axis | Validated range |
|---|---|
| `step_length_m` | `[0, 0.08]` (`max_step_length_m`) |
| `step_frequency_hz` | `[0.2, 2.0]` |
| `swing_height_m` | `[0, 0.05]` (`max_swing_height_m`) |
| `double_support_ratio` | `[0, 0.6)` |
| `leg_extension_m` | `[0.15, 0.24801]` |
| `sway_amplitude_rad` | `[0, 0.35]` |
| `sway_sign` | `{-1, +1}` |
| `gait_phase_mode` | `{legacy, explicit_support_swing}` |
| `foot_pitch_swing_rad` | `[0, 0.4]` |

Out-of-range θ MUST be rejected before launch. **Eligible sampling dimensions** are the launch-exposed,
producer-validated continuous geometry axes: `step_length_m`, `step_frequency_hz`, `swing_height_m`,
`double_support_ratio`, `leg_extension_m`, `sway_amplitude_rad`, `foot_pitch_swing_rad`.

`sway_sign` and `gait_phase_mode` are **anchor-defining** and MUST NOT be varied in Stage 0: changing
either changes the gait *family*, not a neighborhood.

### 4.5 Non-sampling, non-exposed bounds (explicit)

| Parameter | Producer default | Status |
|---|---|---|
| **`max_delta_per_cycle`** | `0.15` (range `[0.01, 1.0]`) | **Producer-internal safety bound.** Not forwarded by the launch file ⇒ not settable from any runner ⇒ **MUST NOT be a sampling dimension and MUST NOT appear in θ.** Acts only as an abort guard in `_check_safety()` (`:490`). |
| `max_step_length_m` | `0.08` | Declared by the producer, **not** launch-exposed (validation cap only). Not a sampling dimension. |
| `max_swing_height_m` | `0.05` | Declared by the producer, **not** launch-exposed (validation cap only). Not a sampling dimension. |

### 4.6 Per-key parameter provenance (F-7 fix)

For each episode the driver MUST persist, for **every** key in the resolved θ:

```
source_category ∈ { anchor_declaration | sampling_dimension | runner_cli_argument | runner_default }
transport       ∈ { anchor_file | cli_override | cli_run_control | launch_default | internal_default }
```

| Category | Meaning |
|---|---|
| `anchor_declaration` | value originates from the declared anchor θ (§3.3) |
| `sampling_dimension` | value originates from the sampling plan (§7) |
| **`runner_cli_argument`** | value originates from a runner CLI argument **outside** the θ object (run-control) |
| `runner_default` | value originates only from a runner/internal default — **a defect for Stage 0** |

Normative rules:

- **`runner_default` is forbidden.** Every resolved key MUST have an explicit origin. Any key whose
  provenance is `runner_default` invalidates the episode's provenance record.
- `duration_sec`, `hold_final_sec` — the two keys with the REV0 default-override hazard — MUST be
  recorded with their true origin. On the canonical runner (§6.1) the expected origins are:

  | Key | `source_category` | `transport` | Reason |
  |---|---|---|---|
  | `duration_sec` | `runner_cli_argument` | `cli_run_control` | supplied by the runner's `--duration` argument |
  | `hold_final_sec` | `anchor_declaration` | `cli_override` | declared in the 27-key anchor, transported as an explicit override |

  Neither may resolve to `runner_default` on any path.

- the driver MUST also record the **declared** θ, the **resolved** θ, and
  `theta_hash` = SHA-256 over the resolved 27-key θ using the §3.2 method.

---

## 5. Seed and Repeatability Contract

### 5.1 Three distinct concepts (MUST NOT be conflated)

| Concept | Meaning | Recorded as |
|---|---|---|
| **experiment seed** (`--seed`) | replicate **label**; a grouping key, not a physics input | `seed.semantics` |
| **gz seed** (`--gz-seed` → `gz sim --seed`) | the **real physics RNG** seed | `seed.actual_gz_sim` |
| **partition / domain** (`GZ_PARTITION`, `ROS_DOMAIN_ID`) | process isolation only; **not** a physics seed | `seed.partition`, `seed.ros_domain_id` |

The runner documents this separation (`tools/run_p0a_gait_candidate.py:160-175`, `:178-188`) and the
canonical runner records it (`run_phase_a_episode.py:325-332`). Stage 0 MUST preserve it.

### 5.2 Per-episode seed record (MUST)

```
seed.requested        # intended physics seed
seed.actual_gz_sim    # seed observed on the real gz sim argv
seed.verified         # requested == actual (or both absent)
seed.partition        # GZ_PARTITION
seed.ros_domain_id    # ROS_DOMAIN_ID
seed.semantics        # the label-vs-physics distinction, stated in words
```

- Every Stage 0 episode MUST wire an explicit gz seed. An episode with `seed.requested = null` MUST be
  marked `seed.verified = false` and MUST NOT be used as repeatability evidence.
- Where the live `gz sim` argv cannot be observed, the episode MUST fall back to
  `gz_sim_cmd.expected.txt` and MUST label the value **reconstructed**, not observed.

### 5.3 Repeatability requirement (F-5 correction)

- **≥ 3 seeds per θ.** A θ with fewer than 3 completed seeds MUST be reported `INSUFFICIENT`; no
  statistic SHALL be computed from it.
- The goal is **statistical coverage, not bit reproducibility.**

**Evidence, correctly attributed:**

- **Same-seed instability** (same θ, same seed, separate run): OPT-1A records that a repeated run at the
  same θ and the same seed **moved fitness by `0.128`** — the same order as the entire baseline→best
  gap (`docs/gait_optimization/OPT1A_OBJECTIVE_DIAGNOSIS.md:103`, `:149`). Caveat, recorded by OPT-1A
  itself (`:440`, item EI-3): that `0.128` divergence has **not** been decomposed into RNG vs bridge vs
  real-time scheduling vs window truncation. It therefore evidences *instability*, not its cause.
- **Cross-seed dispersion**: the `yaw_deg.std ≈ 25.6°` figure is a **cross-seed** dispersion across the
  3 confirmation seeds (`optimization_results/confirmation/summary.json`,
  `aggregate.baseline.yaw_deg.std`). It is **not** a same-seed instability measure. REV0 mislabelled it;
  REV1 corrects the label.

- Stage 0 MUST report dispersions **across seeds** and MUST NOT claim determinism, reproducibility, or
  "no regression". The strongest permitted claim is a numeric dispersion plus its seed count.

### 5.4 Failure retention

A run whose gait fails (fall, `walking_pass=false`) but whose data is coherent
(`episode_integrity = PASS`) is a **valid episode** and MUST be kept and indexed. Stage 0 MUST NOT drop
failure episodes, and MUST NOT equate "gait failed" with "episode invalid" (inherited from
`tools/nao_episode_integrity.py:4-16`).

---

## 6. Dataset Contract

Stage 0 reuses the existing episode artifact format. **No schema upgrade** (frozen schema v1.0.1).

### 6.1 Canonical runner (F-2 fix)

**Exactly one** runner is the Stage 0 production path:

> **`tools/run_phase_a_episode.py`**

Why it is canonical (all verified in `tools/run_phase_a_episode.py`):

1. It publishes an **atomic, self-describing episode** (staging dir → `os.replace`, `:466-477`).
2. It writes **full provenance including `code_revision.source_sha256` for six frozen source files**
   (`:312-323`, `:351-353`) — no other runner records code hashes.
3. It **verifies the physics seed against the live `gz sim` argv read from `/proc`**
   (`:116-143`, `:257`, `:278-282`) and records `seed.verified` (`:308-310`).
4. It computes the episode verdict via the frozen integrity tool (`nao_episode_integrity.evaluate`,
   `:289-292`), keeping the four axes uncoupled.
5. It runs the collector **and** the golden validator in-loop (`:173-178`, validated pre-commit inside
   the collector).
6. It does **not** apply `EXPERIMENT_DEFAULTS`, removing the REV0 default-override hazard at the source.

Artifacts it produces (per episode directory):

| Artifact | Content |
|---|---|
| `manifest.json` | episode manifest (schema `phase_a_1.0.0`, dataset schema `1.0.1`) |
| `provenance.json` | θ, seed triple, gazebo argv actual/expected, `code_revision.git_commit` + `source_sha256`, outputs SHA-256 |
| `episode_integrity.json` | 9 checks + coverage + duration + `gait_outcome` + `reserved{}` |
| `run_config.json` | the **resolved** θ (this is what `theta_hash` hashes) |
| `launch_cmd.txt`, `gz_sim_cmd.actual.txt`, `gz_sim_cmd.expected.txt` | real vs reconstructed invocation |
| `launch.log` | combined stdout/stderr |
| `collector_summary.json` | collector accounting closure |
| `dataset/manifest.json`, `dataset/samples.jsonl` | canonical trajectory dataset (v1.0.1) |
| `trace/{clock,command,state,contact,body_pose,imu}_trace.jsonl`, `trace/phase4b_summary.json` | raw streams |

**How the frozen analyzer fits (authoritative, offline).** The metric layer
(`tools/analyze_measured_gait.py`) runs **after publication, read-only, offline**. It is verified to
consume a canonical episode directory **unmodified**: it needs `run_config.json` (present),
`trace/body_pose_trace.jsonl` (present), `trace/state_trace.jsonl` (present; used as the `ready_time`
fallback when `result.json` is absent — which is the case for Phase A episodes), and
`trace/contact_trace.jsonl` (present). It therefore requires **no change** and remains the single
authority for the `measured` metric block.

### 6.2 Non-canonical runner (restriction)

`tools/run_p0a_gait_candidate.py` MAY be used **read-only** (e.g. to cross-check a metric against the
`result_p0a.json` schema), but MUST NOT be the Stage 0 production path. Reasons: its provenance
records no code hash and no git commit (`:305-316`), and it is the path on which `EXPERIMENT_DEFAULTS`
silently rewrites `duration_sec`/`hold_final_sec` (`:42-46`, `:233-245`).

### 6.3 Temporal alignment table (F-4 fix — disclosure only)

The canonical per-sample state is:

```
s_t = ( timestamp,
        joint_position[24],
        joint_velocity[24],
        imu{ stamp, orientation[4], angular_velocity[3], linear_acceleration[3] },
        left_contact,
        right_contact,
        command{ stamp, position[24] },
        * base_pose )            # attached by join, NOT stored in the sample
```

The first seven components are exactly the frozen schema v1.0.1 sample keys
(`nao_trajectory_schema.py:79-87`) and live in `dataset/samples.jsonl` unchanged.

**The alignment semantics are heterogeneous and MUST NOT be described as uniform:**

| State component | Source | Alignment rule | Causal? | Tolerance |
|---|---|---|---|---|
| `timestamp` (sample event) | `/joint_states` | a new message **is** the sample (main clock) | — | — |
| `joint_position[24]` | `/joint_states` | same message as the sample event | — | — |
| `joint_velocity[24]` | `/joint_states` | same message as the sample event | — | — |
| `imu` | `/imu/data` | **symmetric nearest** on sim time (`nearest_neighbor_sim_time`); tie → earlier | **NON-causal — may include one future sample** | `±0.01 s` (`IMU_NEAREST_MAX`) |
| `command` | `/joint_command` | **hold-last**: last command with `stamp ≤ t` (`hold_last_causal_bounded`) | causal | `≤0.05 s` (`COMMAND_HOLD_MAX`) |
| `left_contact`, `right_contact` | `/left_foot/contact`, `/right_foot/contact` | **last-not-after** on `/clock` receipt (`clock_freshness_zero_order_hold`) | causal | `≤0.05 s` (`CONTACT_FRESHNESS_MAX`) |
| `* base_pose` | `/body_pose` → `trace/body_pose_trace.jsonl` | **causal last-not-after join** (last pose with `t_pose ≤ t`) | causal | `0.02 s` (§6.4) |
| `/clock` | `/clock` | simulation-time watermark only; never a sample field | causal | — |

Sources: constants `IMU_NEAREST_MAX`, `CONTACT_FRESHNESS_MAX`, `COMMAND_HOLD_MAX`
(`nao_trajectory_schema.py:85-87`); policy strings `sync_policy` (`:382-392`); collection rules
`phase4d_collector.py` `_nearest_imu` / `_contact_value` / `CommandHistory.select_for`.

**Explicit statements required by this contract:**

- `base_pose` is attached by a **causal last-not-after join** (§6.4) — never symmetric, never
  interpolated.
- `command` is **causal hold-last**.
- `contact` is **causal last-not-after**.
- `imu` is the **one non-causal** component: the frozen collector's symmetric nearest neighbour with
  maximum `±0.01 s`, and MAY therefore include **one future sample**.

This is **disclosure only**: the collector and the schema are NOT modified, and the frozen
`phase4b_trace` internal foot-FK join (an unbounded last-not-after `_nearest`, `:329-336`) is untouched.
The tolerance below governs only the Stage 0 `s_t.base_pose` attachment.

### 6.4 Base-pose tolerance — frozen (F-6 fix)

> **Frozen value: 0.02 s.**

| Aspect | Value / statement |
|---|---|
| Value | `0.02 s` |
| Rule | for sample time `t`, `base_pose(t)` = last `body_pose_trace` row with `t_pose ≤ t`; if none, or if `t − t_pose > 0.02 s`, `base_pose(t)` is **unavailable** |
| Reason (measured) | canonical sample period = **0.010 s** (min = median = max, both episodes measured); `body_pose` spacing median **0.008 / 0.015 s** with **max observed gap 0.017 / 0.018 s**. `0.02 s` = 2× the sample period and exceeds the maximum observed gap, so normal jitter never drops a pose, while a stale pose (>2 sample periods) is rejected. |
| Rejected alternative | `0.05 s` (matching the collector's contact freshness bound) was rejected: it would silently attach a pose up to 5× the sample period old, blurring the join. |
| No tuning | the value is a constant; it MUST NOT be tuned per episode, per θ, or per ring, and MUST be recorded as a constant in each episode's provenance |
| On absence | `base_pose(t)` unavailable MUST be surfaced as unavailable — **no interpolation, extrapolation, or zero-fill**, consistent with the schema's `missing_data_policy = reject_sample` |

### 6.5 Episode-level required fields

| Field | Meaning | Requirement |
|---|---|---|
| `episode_id` | unique identity within the run | MUST |
| `theta` | the resolved θ (27 keys) | MUST |
| `theta_hash` | §3.2 digest of the resolved θ — **never** the historical artifact hash | MUST |
| `reference.historical_fwdf08_artifact_hash` | the 26-key reference digest | MUST when applicable; reference-only |
| `seed.requested` / `.actual_gz_sim` / `.verified` | seed triple (§5.2) | MUST |
| `git_commit` | repository HEAD at run time | MUST |
| `source_hash` | SHA-256 of each frozen source file involved (driver + canonical runner + producer + collector + schema + launch) | MUST |
| `config_hash` | SHA-256 of the frozen configuration inputs (§6.6) | MUST |
| `provenance[*].source_category` / `.transport` | per-key origin (§4.6), **no `runner_default`** | MUST |
| `base_pose_tolerance_sec` | frozen `0.02` | MUST |
| `artifact_path` | episode directory | MUST |
| `status.episode_integrity` | `PASS` / `FAIL` | MUST |
| `status.collection_status` / `.termination` | e.g. `normal_completion` | MUST |
| `metrics` | §6.7 | MUST when `episode_integrity = PASS` |
| `failure_class` | from the integrity verdict, when applicable | MUST |

These fields MUST be sourced from the episode's own artifacts; the driver MUST NOT re-derive them.

### 6.6 `config_hash`

`config_hash` MUST be the SHA-256 of the frozen configuration inputs — at minimum the canonical θ
serialization (§3.2) plus the frozen launch command template — so two episodes claiming the same
configuration can be proven to share it without re-reading the gait code.

### 6.7 Metrics

Metrics MUST reuse the existing definitions unchanged:
`forward_displacement_m`, `lateral_displacement_m`, `net_displacement_m`, `actual_speed_mean_mps`,
`actual_speed_std_mps`, `yaw_change_deg`, `torso_tilt_max_deg`, `fall`, `path_over_net`,
`max_single_step_m`, `effective_duration_sec`, `trace_truncated`
(`tools/analyze_measured_gait.py:319-350`), and episode-level `gait_outcome`
(`tools/nao_episode_integrity.py:324`).

Metrics MUST be **sourced from frozen tool outputs** — the analyzer's output and/or the integrity
verdict's `gait_outcome`. The driver MUST NOT compute them (§8.2). Stage 0 MUST NOT redefine a metric,
add a derived score, or rank episodes.

### 6.8 Corpus index (the one new dataset artifact)

A single machine-readable file, one record per episode, carrying the §6.5 fields, sufficient to answer
§1.4 without opening the episode directory. Rewritten atomically per run; MUST NOT be written inside any
frozen evidence tree.

---

## 7. Sampling Strategy

Stage 0 SHALL use a **pre-declared, static θ list**. The list MUST be fixed and hashed
(`theta_list_hash`) **before any episode is executed**. No interactive expansion, and **no optimization
algorithm** (no objective, no fitness, no search).

```
Ring 0  ANCHOR RE-BASELINE
        fwdf08-stage0-anchor exactly as §3.3            × ≥3 seeds
        purpose: Stage 0 baseline under explicit seed wiring + full provenance
            │
            ▼
Ring 1  LOCAL PERTURBATION
        one sampling dimension at a time, both directions (+Δ / −Δ);
        Δ declared in the θ list; NO combinations                × ≥3 seeds per θ
            │
            ▼
Ring 2  COVERAGE
        wider single-axis spans, plus pre-declared 2-axis combinations;
        all strictly inside §4.4 ranges                          × ≥3 seeds per θ
            │
            ▼
Ring 3  PRE-DECLARED FAILURE-BOUNDARY COVERAGE GRID
        a FIXED grid of θ declared in the SAME pre-registered θ list as Rings 0–2
        and FIXED BEFORE ANY EPISODE OF ANY RING EXECUTES; it spans outward from
        the anchor toward the expected failure boundary. It is NOT derived from,
        selected by, extended by, narrowed by, or reordered by the results of
        Ring 1 or Ring 2. The ENTIRE grid is executed regardless of outcomes.
                                                          × ≥3 seeds per θ
```

### 7.1 Ring 3 rules (F-3 + R-2 fix — normative)

Ring 3 is a **pre-declared coverage grid**, not an adaptive probe. Its parameters MUST satisfy all of:

- **Fixed before execution:** the entire Ring 3 θ list MUST be determined and written **before any
  episode of any ring is executed**, and MUST be part of the single pre-registered `theta_list_hash`.
- **Independent of prior rings:** Ring 3 MUST NOT be determined, selected, extended, narrowed, or
  reordered on the basis of Ring 1 or Ring 2 experimental results. It is fixed at declaration time,
  not after those rings run.
- **Independent of failures:** Ring 3 MUST NOT be expanded, adjusted, retargeted, or re-seeded because
  of any observed failure episode (fall, `walking_pass = false`, `episode_integrity = FAIL`, or
  `measurable = false`). A failure outcome is recorded, never acted upon.
- **No result-dependent expansion** — the number of samples cannot grow from observed outcomes.
- **No early stopping** — every declared grid point is executed.
- **No hidden optimization** — no direction selection, no "best so far", no search.

Ring 3 **MAY** contain failure-oriented samples (θ expected to fall or fail the walking check), but they
MUST be declared **before any episode runs**. Observing a failure MUST NOT change what is executed next.

**Prohibited language.** No wording in this section may be interpreted as adaptive search or as
result-driven control flow. Terms such as "push until", "stop at the first boundary", "continue in the
direction that…", or "expand based on…" MUST NOT appear in, or be applied to, Ring 3.

### 7.2 Boundary classification (reporting only)

The failure condition may be *labelled* post hoc for the record — the first occurrence of any of
`episode_integrity = FAIL`, `gait_outcome.fall_detected = true`, `walking_pass = false`, or analyzer
`measurable = false`. This label is a **reporting annotation only** and MUST NOT influence control flow,
grid size, or any subsequent θ.

### 7.3 Standing rules

- **Ring 0 MUST be executed first** and recorded before Rings 1–3 are interpreted.
- Δ (Ring 1) and the spans/grid (Rings 2–3) are **data** in the θ list, not code. The RFC fixes the
  rule, not the numbers.
- Arms, head, and upper-body parameters MUST remain frozen across all rings (§3.3).
- No ring may be extended, truncated, or re-ordered after its results are seen.
- **Ring 3 is fixed at declaration time** — before any ring executes — and MUST NOT be revised after
  Ring 1 or Ring 2 run (R-2).
- Failure episodes MUST be kept (§5.4).

---

## 8. Minimal Implementation Boundary

Future implementation is authorized to add **only** the two artifacts of §2.2.

### 8.1 Thin sampling driver — permitted responsibilities (exhaustive)

1. **θ validation** — §4.2 (required keys), §4.3 (unknown-key rejection), §4.4 (range), §4.5.
2. **Seed scheduling** — experiment seed and explicit gz seed per §5; distinct isolation ids for
   concurrent runs.
3. **Runner invocation** — invoke the canonical runner `tools/run_phase_a_episode.py` **unmodified**
   with the resolved θ and `--duration`; capture exit status.
4. **Frozen-tool orchestration** — the driver **MAY** invoke frozen tools **as-is** (e.g. the analyzer
   `tools/analyze_measured_gait.py`, the integrity tool, the dataset validator) to populate metrics and
   status. Orchestration is permitted; reimplementation is not.
5. **Provenance generation** — collect `git_commit`, `source_hash`, `theta_hash`, `config_hash`, the
   seed triple, per-key `source_category`/`transport`, and `base_pose_tolerance_sec`.
6. **Corpus index** — emit/update the index of §6.8.

### 8.2 Thin sampling driver — MUST NOT

- MUST NOT modify any file in §2.1.
- MUST NOT **derive metrics** — it MUST NOT calculate, recompute, adjust, or re-derive any metric or the
  gait outcome. It reads them from frozen tool outputs. *(Corrected wording: the prohibition is on
  deriving metrics, **not** on invoking the analyzer. Invoking frozen tools is permitted under §8.1.4.)*
- MUST NOT replace, reimplement, or bypass the frozen analyzer — the analyzer remains **authoritative**
  for the metric layer.
- MUST NOT compute gait math or any trajectory/joint quantity.
- MUST NOT publish or subscribe to any ROS topic; MUST NOT interact with Gazebo directly.
- MUST NOT score, rank, select, or filter episodes.
- MUST NOT drop, overwrite, or mutate an episode after publication.
- MUST NOT write into any frozen evidence tree (§2.1) or `backup/`.

### 8.3 Explicitly NOT required

- **No new parameter-injection interface.** All 27 θ keys are already launch-exposed (including
  `phase4b_sway_sign` and `phase4b_gait_phase_mode`); the existing `param_args()` mapping suffices.
- **No changes** to schema, analyzer, evaluator, controller, physics, or the existing runners.

### 8.4 Output location

The output root MUST be named in the run declaration and MUST NOT be inside any frozen evidence tree. The
driver MUST refuse, before writing, an output root resolving inside a protected tree (the canonical
runner already implements such a guard, `run_phase_a_episode.py:188-194`).

---

## 9. Acceptance Criteria

### 9.1 RFC completion criteria

| # | Criterion | Where | Status |
|---|---|---|---|
| A1 | **Anchor explicit** — all 27 keys declared, incl. `sway_sign=-1`, `gait_phase_mode=legacy`, `duration_sec`, `hold_final_sec`; no reliance on `EXPERIMENT_DEFAULTS`; no inheritance from `B3-ANCHOR` | §3.3–§3.6 | MET (contract) |
| A2 | **θ contract complete** — required keys, unknown-key rejection, range validation tied to the producer, 4-category per-key provenance incl. `runner_cli_argument`, no `runner_default` | §4 | MET |
| A3 | **Dataset contract complete** — one canonical runner; episode-level fields; `s_t` incl. the heterogeneous temporal table and the frozen base-pose tolerance; no schema upgrade | §6 | MET |
| A4 | **Provenance complete** — two distinct hashes (26-key historical reference vs 27-key Stage 0 identity), `config_hash`, `git_commit`, `source_hash`, seed triple, `theta_list_hash` | §3.2, §4.6, §5.2, §6.5–§6.6, §7 | MET |
| A5 | **No frozen component modification** — only the thin driver and corpus index are added | §2.1–§2.2 | MET (contract) |
| A6 | **Static pre-registration** — the full θ list is fixed and hashed before execution; Ring 3 is a coverage grid with no adaptivity, no early stopping, no optimization | §7 | MET |
| A7 | **Preservation** — 27-parameter anchor, `sway_sign=-1`, `gait_phase_mode=legacy`, no optimization, no learning, frozen boundary, no schema upgrade all intact | §0.4, §1.3, §3, §7 | MET |

### 9.2 Implementation acceptance (only after this RFC is accepted)

1. A1–A7 hold in the delivered code.
2. Every frozen file in §2.1 is byte-identical after the run (SHA-256 compared to the pinned set).
3. The anchor episode's `run_config.json` hashes to `stage0_anchor_theta_hash`
   (`28e795b2…94998`) and has exactly 27 keys.
4. Ring 0 produces ≥3 seeds of the anchor with `seed.verified = true`.
5. No episode's per-key provenance contains `runner_default`.
6. No failure episode was dropped.
7. The corpus index answers §1.4 for every episode without opening the episode directory.

`MUST NOT` be self-certified. Acceptance is the auditor's to emit.

---

## 10. No-Code-Change Declaration

This revision is a **document-only contract repair**. Verified on the working tree:

```
git status --porcelain   →  (empty)
```

Therefore:

- **No repository code was modified.** No source file, config, launch file, schema, or script was
  edited, added, or deleted by this revision.
- The only files created by this Stage 0 work are documents (`STAGE0_PARAMETERIZED_GAIT_RFC.md`,
  `STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md`, `STAGE0_REPOSITORY_BASELINE_AUDIT.md`), all at the repository
  root.
- No runner, sampling driver, collector, analyzer, evaluator, fitness, controller, or gait generator was
  implemented or altered.
- The two frozen runners remain unmodified; only their **existing** behavior was read and cited.
- No sampling was executed. No Gazebo process was launched. `gz` was not on `PATH` during this work.

---

## 11. Remaining Open Issues

| # | Issue | Impact | Owner |
|---|---|---|---|
| **O1** | **The GLM-5.3 Stage 0 adversarial audit is absent from the repository.** Re-searched: only `docs/GLM-5.3 Independent Acceptance - Audit Phase A.txt` and `docs/task_p0a_gait_search/GLM-5.3_Stage6_review_prompt.md` exist. | REV1 is driven by the enumerated F-1…F-7 directive, **not** by the report text. Any finding in that report not covered by F-1…F-7 is **not** addressed here. | Contract owner / auditor |
| **O2** | `STAGE0_REPOSITORY_BASELINE_INDEPENDENT_AUDIT` remains absent. | No independent reading of the ground truth has been reconciled against this contract. | Independent auditor |
| **O3** | **REV1 has not been audited.** | Status is **awaiting independent audit**; no acceptance is claimed. F-1…F-7 repairs are self-declared as applied, not verified. | Independent auditor |
| **O4** | Baseline-audit item **R9** (behavior of an undeclared override key at `ros2 launch`) is still `UNKNOWN`; not verified here. | §4.3 rejects unknown keys defensively, so Stage 0 does not depend on it. Verification still recommended. | Engineer |
| **O5** | `stage0_anchor_theta_hash` assumes the canonical runner resolves **exactly** the 27 declared keys. If the driver omits `stand_sec` (inert for `walk` but part of the identity), the digest differs. | §3.6 makes this checkable; the implementation MUST assert 27 keys. **Not yet empirically run** (no Gazebo available during this work). | Engineer |
| **O6** | The base-pose tolerance `0.02 s` is derived from two episodes' measured spacing (max gap 0.017/0.018 s). A run with slower `/body_pose` publishing could exceed it. | Samples beyond the tolerance lose `base_pose` and are marked unavailable (honest, not silent). If this occurs frequently, the constant should be revisited by a **new revision**, not tuned per episode. | Engineer / auditor |
| **O7** | `STAGE0_PARAMETERIZED_GAIT_RFC_REV1.md` is **git-ignored** (`.gitignore:27`, `**/*RFC*.md`), as is the audit (`.gitignore:24`). | These documents are local-only and not reproducible from a clone; a future stage citing them must record their SHA-256 out-of-band. | Contract owner |

**Status: REV1-DRAFT — awaiting independent audit. Not accepted.** No implementation is authorized by
this document, no part of Stage 0 is implemented here, and no sampling was performed.
