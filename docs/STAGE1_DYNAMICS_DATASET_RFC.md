# STAGE1_DYNAMICS_DATASET_RFC

Stage 1 Dynamics Dataset RFC — specification of the dynamics-observation dataset required to
advance from the frozen Stage 0 behavior baseline to physics identification, LIPM validation, and
residual-learning preparation.

| Field | Value |
|---|---|
| Revision | REV1.6-PROVENANCE-CLARIFICATION |
| Status | **ACQUISITION-CONTRACT-READY** — R14 normative machine-readable schema binding + artifact consistency per the GPT-5.6 independent closure audit of R13 (Decision C — BLOCKED); **pending final independent audit acceptance**; three-tier authorization, see § 0.0 |
| Author | Implementation Agent (DeepSeek / Qoder role); REV1.2 corrections: Implementation Agent (GLM-5.3); REV1.3 hardening: Implementation Agent (GLM-5.3); REV1.4 (R11) hardening: Implementation Agent (GLM-5.3); REV1.5 (R14) normative schema: Implementation Agent (GLM-5.3); REV1.6 (provenance evidential-boundary clarification): Implementation Agent (DeepSeek) |
| Date | 2026-10-08 (REV1.1: 2026-10-07; REV1.2–1.5: 2026-10-08; REV1.6: 2026-10-13) |
| Roadmap position | Baseline Dataset (Stage 0, frozen) → **Dynamics Dataset (this RFC)** → LIPM Prior → Residual Network → Optimization → Sim2Real |

## 0. Scope declaration (read first)

REV1.1 turned this document from a draft into the **frozen acquisition contract** for Stage 1.
"FINAL" means exactly this: the contract for raw data acquisition is frozen, and any later
change to population, seeds, rings, schema, formulas, or validity rules requires a REV increment
(§ 4.4). It does **not** mean the program, the analysis, or any downstream stage is finalized.

**REV1.2 (superseded by this revision) applied the five semantic corrections (R1–R5) identified
by the GPT-5.6 independent audit of the REV1.1 preparation artifacts** (2026-10-08): raw contact
source (R1), V-1 timestamp validation method (R2), contact validity semantics naming (R3),
V-2 pose frame semantics (R4), and composition validation wording (R5).

**REV1.3 (superseded by this revision) applied the five minimal-hardening corrections (R6–R10)
identified by the GPT-5.6 independent re-audit of REV1.2** (2026-10-08): event-aware contact
publication validation replacing the absolute effective-frequency band (R6), Stage 1 two-state
contact validity (R7), V-2 composition evidence with an explicit status (R8), measured/derived
label correction (R9), and timestamp association-compatibility wording (R10).

**REV1.4 (superseded by this revision) applied the R11 minimal hardening for the two blockers
of the GPT-5.6 independent adversarial audit of REV1.3 (Decision B — READY AFTER MINOR
FIXES):** R11-A pending-validation propagation (§ 3.0 rule; schema class annotations extended
to every derived field that transitively depends on the unverified composition or the unknown
wrench frame/reference) and R11-B provenance completeness binding (§ 3.10 declared the single
source of truth for collector provenance). No new V-gate, no field added/removed, no
population/seeds/rings change, no reopening of any R6–R10 decision.

**REV1.5 (superseded by this revision) applies the R14 normative-schema binding for the four bypasses of the
GPT-5.6 independent closure audit of R13 (Decision C — BLOCKED):** B1 (Option-A shapes were
hardcoded in the checker), B2 (prose-keyword semantic parsing was fragile — "non-null" counted
as nullable), B3 (raw-retention requiredness not fully bound), B4 (frame_count vs actual frame
records). The change is **Appendix D**: a normative machine-readable JSON schema block that is
the single machine source of truth — the Collector Contract embeds an identical copy and the
smoke checker loads its semantics from this block at run time (the checker owns no independent
schema); explicit `required`/`nullable`/`type`/shape semantics replace all prose-keyword
parsing; the raw-retention requiredness (Contact headers, `body_1_id`/`body_2_id`, pose
orientation, JointWrench members) and the frame-artifact count rule (`frame_count` == actual
frame records; empty episodes allowed) are machine-enforced. No research-objective, R6–R10,
acquisition-requirement, or raw/derived scientific-meaning change. **REV1.5 is pending final
independent audit acceptance** — no acceptance criterion is claimed satisfied by this revision,
and no acquisition authorization is claimed.

**REV1.6 (this revision) adds a § 3.10 provenance evidential-boundary clarification** (Owner-approved
D4-EPC decision; Option B — RFC-only prose). `theta`/`theta_id`/`theta_hash` describe the requested
(pre-registered) configuration; `config_hash` identifies the Collector configuration;
`cop_derivation_hash` identifies the CoP/derived-wrench derivation configuration; none of these
fields — by existence, equality, or recomputability — constitutes evidence that the producer
applied the corresponding gait parameters at runtime. No population, seeds, rings, formula, or
Appendix D schema-block change; no field added or removed; no runtime behavior or automatic
verification; **REV1.6 is pending final independent audit acceptance, and no acquisition
authorization is claimed.**

### 0.0 Authorization levels (three-tier, explicit — REV1.1)

| Level | Scope | Status under this revision |
|---|---|---|
| 1 — Preparation | Completion of this RFC revision; read-only V-1/V-2 probes (`tools/stage1_probe.py`, new file); collector contract specification (`docs/STAGE1_COLLECTOR_CONTRACT.md`, new file) | **authorized** |
| 2 — Collector implementation | Implementing the collector per `docs/STAGE1_COLLECTOR_CONTRACT.md` as a **read-only instrumentation process**: new subscribers, a new writer process, storage under `dataset/stage1/` only — no modification to any frozen component (runner, controller/gait generator, converters, model, world, physics, launch files) | **authorized within the § 7 boundary** |
| 3 — Bulk raw acquisition | The 96 pre-registered episodes (§ 4.1) | **NOT authorized by this document alone.** Requires: acceptance of this revision by the **final independent audit** (REV1.6 is pending), V-1 pass **and** V-2 pass, probe results pinned (Appendix B), and remote publication of the Stage 0 closure commit and this contract (owner action, GPT-5.6 correction 8). |

Analysis, LIPM fitting, residual learning, training, and optimization are **never** authorized by
this document and require separate authorization (§ 7).

This document is a **contract**, not itself an implementation. Specifically, this RFC does not
include, authorize, or imply:

- any modification to the simulation pipeline (launch files, converters, world, model),
- any modification to the Stage 0 frozen artifacts,
- any modification to the frozen runner `tools/run_phase_a_episode.py`.

The probe and the collector are read-only instrumentation processes; their authorization and
boundary rules are stated in § 5.4, § 7, and `docs/STAGE1_COLLECTOR_CONTRACT.md`.

### 0.1 Pinned upstream artifacts (inheritance anchors)

| Artifact | Pin | Value |
|---|---|---|
| Stage 0 governing RFC | SHA-256 over file bytes | `093459461125b56f71c8c69ad59f218f0a60ce8233fa42a848b371c357274c2c` |
| Stage 0 θ list (`dataset/stage0/stage0_theta_list.json`) | `theta_list_hash` (Stage 0 method: SHA-256 over `json.dumps(theta_list, indent=2, sort_keys=True, allow_nan=False)`, UTF-8, no trailing newline, over the entries array) | `844646f55076f67647c9159a6c1f53018af89276154ff3a4ad3bdce28df95dbf` |
| Stage 0 θ list (file bytes) | SHA-256 over file bytes | `dd52135eed21f3b20c330a2a61e90e89a1f9c13690bddd58330762dc45b9c04a` |
| Stage 0 corpus index (`dataset/stage0/corpus_index.json`) | SHA-256 over file bytes | `260fd9e7888839f01abf640e9773542b8e6afc331c8e4be64b528c871e7a0544` |
| Stage 0 closure commit | git commit | `045a21fdcf7d8d7261a0ad2fa0ab5e8acf041d73` |
| Robot model (`src/nao/nao_description/urdf/nao_simple.sdf`) | SHA-256 over file bytes | `44ed828509db9e869bfa701b491233862dcd1448183bb2ec0a1c3b497aa46400` |
| World (`src/nao_gz_sim/worlds/nao_world.sdf`) | SHA-256 over file bytes | `383341d11d831d0376cae074b1205e5ad7784841a6670bcf0db33813065709cc` |
| Simulator | version | Gazebo Harmonic, gz-sim `8.15.0` |
| Frozen episode runner | path | `tools/run_phase_a_episode.py` — **must remain unmodified** |

Stage 1 inherits every Stage 0 principle that is not explicitly superseded by this document.

---

# 1. Motivation

The program target is

```math
F_NAO(\theta,t) \;=\; F_LIPM(\theta,t) \;+\; R_\phi(\theta,t)
```

where `F_LIPM` is the physics prior (linear inverted pendulum) and `R_\phi` is a residual
correction learned from data. Estimating, validating, and eventually learning that decomposition
requires observing the **actual dynamical state and the actual contact wrenches** of the robot,
not only its gait-level behavior.

## 1.1 What Stage 0 already provides

Per the frozen Stage 0 contract (REV1.1, §6), each Stage 0 episode records:

- joint position (24 joints, `/joint_states`),
- joint velocity (24 joints, `/joint_states`),
- joint command (24 joints, `/joint_command`),
- IMU orientation / linear acceleration / angular velocity (`/imu/data`),
- left/right contact **boolean** (`/left_foot/contact`, `/right_foot/contact`),
- base pose (6-DoF world pose of `base_link`, `/body_pose`),
- θ parameters and full provenance,
- episode-level behavior metrics (walking pass, falls, displacement).

This is a **behavior dataset**: it answers *"what did the gait do?"*

## 1.2 What Stage 0 lacks: robot dynamics observation

| Missing observable | Why it is missing (measured fact, not an assumption) |
|---|---|
| CoM position / velocity / acceleration | Never computed or recorded in Stage 0. The full-body link poses needed to compute it were available on the wire (`/world/nao_world/pose/info`, published by SceneBroadcaster) but only the `base_link` entry was consumed (`body_pose_converter`); all other link poses were discarded on receipt. |
| Base linear / angular velocity | Only `base_pose` (position + orientation) was recorded; no twist stream exists in the Stage 0 trace. |
| Foot pose / foot velocity | The `l_ankle` / `r_ankle` link poses were never recorded as streams (same single-link consumption as above). |
| Contact force / contact moment | The bridge already delivers the raw gz contact messages (`/nao/left_foot_contact_raw`, `/nao/right_foot_contact_raw`, type `ros_gz_interfaces/msg/Contacts`, whose gz `msgs.Contact` carries `repeated JointWrench`), **but `contact_converter.cpp` reduces each message to a `Bool` and discards the wrench at conversion time**, and `phase4b_trace.py` subscribes only to the `Bool` topics. Stage 0 traces therefore never contained a single wrench sample. |
| ZMP / CoP | Never computed. Without measured contact wrenches it is not derivable from Stage 0 data. |

Two consequences follow, and both are design drivers for Stage 1:

1. **The gap is irrecoverable post hoc.** The contact wrench is destroyed at collection time
   (`contact_converter.cpp`), and the discarded link poses were never persisted. No re-processing
   of the frozen Stage 0 corpus can produce dynamics observables.
2. **The gap is exactly the modeling target.** The LIPM residual `R_\phi` is, by construction, a
   statement about forces, CoM dynamics, and contact — quantities Stage 0 cannot see. Training or
   validating any component of `F_LIPM + R_\phi` on Stage 0 data alone would silently reduce the
   residual to whatever can be inferred from kinematics + IMU, i.e. to a different (weaker, biased)
   problem than the stated target.

**Stage 0 is a behavior dataset. Stage 1 must be a dynamics dataset.** A new acquisition pass
with extended instrumentation is therefore required; it is a fresh observation of the same
pre-registered θ population, not an extension of Stage 0 data.

---

# 2. Dataset Objective

The Stage 1 dataset is for **observation**, not selection. Explicitly, it is **not** for
optimization, not for finding the fastest gait, and not for ranking θ. It has four declared uses:

1. **Physics identification.** Establish the empirical CoM / contact-wrench / foot behavior of the
   parameterized gait family in simulation: ground-reaction magnitudes and timing, CoM excursion,
   support timing, flight phases, fall dynamics. This provides the physical plausibility baseline
   against which all later models are judged.
2. **LIPM validation.** Test the LIPM prediction
   `\ddot{c}_x = \omega^2 (c_x - p_x)` (and the y-axis counterpart) against *measured* CoM
   kinematics and *measured* CoP, per episode, per ring, per gait phase; quantify the deviation
   distribution and the constant-CoM-height assumption. This decides whether the LIPM prior is
   usable and how large a residual it leaves.
3. **Residual-learning preparation.** Provide aligned `(s_t, u_t, s_{t+1})` transition tuples with
   full provenance so that a future `R_\phi` can be trained on well-defined deviation targets
   without re-collection. This RFC defines only the data contract; no training occurs in Stage 1.
4. **Stability analysis.** Contact timing, ZMP/CoP excursion relative to the support polygon,
   measured fall dynamics at the failure boundary (ring 3), enabling stability characterization
   independent of any controller.

Success criteria for Stage 1 are **observability and data quality** (completeness, alignment,
provenance, integrity), never gait performance.

---

# 3. Dataset Schema

## 3.0 Frame clock and file format

- One record ("frame") per `/joint_states` message — the Stage 0 master clock is inherited
  unchanged: nominal period `0.010 s` (100 Hz), simulation time. The nominal period is a
  labeling convention only; the **operative transition interval is measured** per § 5.2 (REV1.1).
- Frames are strictly ordered within an episode; `frame_id` is the 0-based index of the
  `/joint_states` message within the episode.
- Per-episode storage: newline-delimited JSON (`.jsonl`), one frame per line, plus episode-level
  metadata records (§ 3.10) in separate files. Layout: Appendix C.
- Every field below carries, in its definition table: name, type/shape, unit, source, alignment
  rule, and **measurement class** — `measured` (from a topic) or `derived` (deterministic
  post-processing with a pinned rule). Derived fields must never be presented as sensor data.
- **Classification summary (REV1.3/R9):** `measured` = raw gz pose messages (`Pose_V` link
  entries + the model pose), raw `Contacts` messages, joint states (plus IMU, command Bool
  streams). `derived` = foot pose (composition), CoM, CoP, ZMP, velocities/accelerations
  (finite differences), and transforms (composition). Composed foot/CoM fields additionally
  carry **`derived_pending_validation`** status until V-2 composition evidence exists (§ 3.3
  note, Appendix B V-2, REV1.3/R8).
- **Validation-status propagation (REV1.4/R11-A — binding rule).** Status vocabulary is
  exactly: `measured`, `derived`, `derived_pending_validation` (no new enums). Rule: **any
  derived field whose computation depends — directly or indirectly — on an input whose
  validation status is `UNVERIFIED` (composition) or pending/blocked (wrench
  frame/reference) MUST carry `derived_pending_validation` (or remain `null` + blocked where
  § 3.6.4/§ 3.7 already block derivation) and MUST NOT be represented as validated derived
  data.** Pending-validation status propagates along every dependency chain; it is lifted
  only when the root input's status changes (a future independent world-pose reference for
  the composition; the V-1 common-frame/common-origin determination for wrenches).
  - **Null-vs-pending implementation (as adopted by this schema):** composition-dependent
    fields use **Option A** — the numeric value is retained, the field is marked
    `derived_pending_validation`. Wrench-frame-dependent fields (`left/right_contact_force`,
    `left/right_contact_moment`, `cop_position`) use **Option B** — `null` while the
    frame/reference is undetermined (§ 3.6.2, § 3.6.4, § 3.7; derivation blocked). In both
    cases a downstream consumer cannot mistake the value for validated physical ground
    truth.
  - **Propagation chains (this schema's actual fields):**
    1. raw model pose + raw relative link pose → `left/right_foot_pose`
       (`derived_pending_validation`) → `left/right_foot_velocity`
       (`derived_pending_validation`);
    2. raw model/link poses → `com_position` (`derived_pending_validation`) →
       `com_velocity` (`derived_pending_validation`) → `com_acceleration`
       (`derived_pending_validation`); `zmp_position` depends on the CoM chain →
       `derived_pending_validation`;
    3. pose + wrench → `cop_position` / world-frame moment: **blocked + `null`** while the
       wrench frame/reference is undetermined (§ 3.6.4) — never treated as validated.
  - **Non-propagation rule:** raw measured data — raw `Pose_V` entries, the model pose, raw
    `Contacts` messages, joint states, IMU, command — are **not** invalidated by the pending
    composition status; they are stored verbatim as `measured`. Only derived quantities that
    *consume* the unverified composition (or the unknown wrench interpretation) carry the
    pending/blocked status.
  - Ordinary derived fields that consume **no** unverified input (e.g. `joint_acceleration`,
    `base_velocity`, `base_angular_velocity` — the base pose anchor is the `/body_pose`
    cross-checked model entry, § 3.3; `gait_phase`, `desired_velocity`) remain plain
    `derived` and are unaffected by R11-A.

## 3.1 Time

| Field | Type | Unit | Source / rule |
|---|---|---|---|
| `sim_time_sec` | float | s (sim time) | stamp of the `/joint_states` message defining the frame (Stage 0 semantics; renamed in REV1.1 to make the time domain explicit) |
| `episode_id` | string | — | globally unique; inherits the Stage 0 naming convention (θ id + seed) |
| `frame_id` | int | — | 0-based, strictly increasing, consecutive within an episode |

## 3.2 Joint (24 entries, canonical order pinned)

Canonical `JOINT_ORDER` (from `src/nao_gz_sim/scripts/nao_trajectory_schema.py`, must match
`nao_simple.sdf`):

```text
HeadYaw, HeadPitch,
LHipYawPitch, LHipRoll, LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll,
LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, LWristYaw,
RHipYawPitch, RHipRoll, RHipPitch, RKneePitch, RAnklePitch, RAnkleRoll,
RShoulderPitch, RShoulderRoll, RElbowYaw, RElbowRoll, RWristYaw
```

| Field | Type | Unit | Source / rule | Class |
|---|---|---|---|---|
| `joint_position` | float[24] | rad | `/joint_states` (frame clock) | measured |
| `joint_velocity` | float[24] | rad/s | `/joint_states` (frame clock) | measured |
| `joint_acceleration` | float[24], **optional** | rad/s² | central finite difference of `joint_velocity` on the frame grid; one-sided at episode boundaries | derived |

## 3.3 Base

| Field | Type | Unit | Source / rule | Class |
|---|---|---|---|---|
| `base_pose` | {`position`: float[3], `orientation`: float[4] (xyzw)} | m, – | `/body_pose` (frozen topic), which re-publishes the `nao` **model entry** of `/world/nao_world/pose/info` with its source stamp. **REV1.2 (R4):** the `base_link` entry itself is a parent/model-relative pose (identity — `base_link` is the model frame), **not** a world pose; the world pose of `base_link` equals the model entry (pose frame semantics note below). Alignment: causal last-not-after, tolerance `0.02 s` (Stage 0 §6.4) | measured |
| `pose_source_stamp` | float | s (sim time) | stamp of the source `/world/nao_world/pose/info` message from which the link-pose fields of this frame were taken. One message carries all 25 link poses, so a single stamp covers base/CoM/foot pose fields (§ 3.3–3.5). Preserved verbatim; alignment misses flagged per frame (REV1.1, per GPT-5.6 correction 5) | measured |
| `base_velocity` | float[3] | m/s | central finite difference of `base_pose.position` on the frame grid; one-sided at boundaries | derived |
| `base_angular_velocity` | float[3] | rad/s | quaternion finite difference of `base_pose.orientation` (pinned: `2 * log(q(t+1) * q(t)^{-1}) / dt`), expressed in the **world** frame | derived |

*Derived twist is the default contract. If verification gate V-3 (Appendix B) finds a native
ground-truth twist stream, the source may be upgraded by RFC revision only — silent substitution
is forbidden.*

**Pose frame semantics (REV1.2, R4/R5 corrections).** Entries of
`/world/nao_world/pose/info` are **parent/model-relative poses**, not world poses (probe
evidence: the `base_link` entry is pinned at the model-frame identity on every message while
the `nao` model entry moves; the REV1.1 "each entry is the world pose of the corresponding link
entity" assumption is falsified). The world pose of a link is **derived by composition** with
the `nao` model entry of the *same* `Pose_V` message:

```text
T_world_link = T_world_model · T_model_link
```

- `base_link` world pose = the model entry itself (identity link), cross-checkable against
  `/body_pose` (exact-stamp).
- The model entry (composition anchor) is validated against `/body_pose` by gate V-2.
- **R5:** the composition method itself is **not** "end-to-end validated" — only the identity
  link (`base_link`) has been compared. The composition method **requires verification using
  selected non-identity links** (e.g. `l_ankle` / `r_ankle`) before composed poses are treated
  as validated world poses; identity-link agreement alone is a consistency check, not a
  validation.
- **R8 (REV1.3):** gate V-2 reports explicit **composition evidence** for at least two
  non-identity links (`l_ankle`, `r_ankle`) and a **`composition_status`** — `UNVERIFIED`
  while no independent non-identity-link world-pose source is available. Validation is never
  claimed from the composition inputs themselves. While `composition_status` is `UNVERIFIED`,
  composed foot/CoM fields remain **`derived_pending_validation`**; the collector may still
  store the raw `Pose_V` entries and the model pose.
- All pose fields of this contract (§ 3.3–3.5) are world poses obtained through this
  composition; the raw model-relative entries and the model entry are retained verbatim in the
  raw capture (Appendix C).

## 3.4 CoM

Computed from full-body link poses + pinned inertial parameters (the model is pinned by hash, § 0.1,
so the parameters are frozen constants of this contract):

```math
c \;=\; \frac{1}{\sum_i m_i} \sum_{i} m_i \,\big(R_i\, \hat{c}_i + t_i\big)
```

where `i` runs over all 25 links of `nao_simple.sdf`, `(R_i, t_i)` is the **composed world pose**
of link `i` (REV1.2, R4: `T_world_link = T_world_model · T_model_link`, both factors taken from
the same `/world/nao_world/pose/info` message — model entry + link entry; parent-relative link
entries are never world poses and are never recorded as such; see the § 3.3 pose frame semantics
note), association causal last-not-after, tolerance `0.02 s`, and
`(m_i, ĉ_i)` are the mass and CoM offset-in-link-frame pinned from the SDF.

| Field | Type | Unit | Source / rule | Class |
|---|---|---|---|---|
| `com_position` | float[3] | m | formula above (REV1.3/R9: **derived** — inputs are composed world poses, `derived_pending_validation` while V-2 composition_status is UNVERIFIED, plus pinned constants) | derived |
| `com_velocity` | float[3] | m/s | central finite difference of `com_position`; **`derived_pending_validation` — inherits from `com_position` while composition_status is UNVERIFIED (R11-A)** | derived |
| `com_acceleration` | float[3] | m/s² | second finite difference of `com_position` (pinned stencil); **`derived_pending_validation` — inherits from the CoM chain while composition_status is UNVERIFIED (R11-A)** | derived |

## 3.5 Foot

**Foot frame declaration:** `nao_simple.sdf` has no sole link. The foot reference frame of this
contract is the **ankle link frame** (`l_ankle` / `r_ankle`) — the links that carry the contact
sensors. If a sole link is ever added to the model, this section requires an RFC revision (model
hash would change).

| Field | Type | Unit | Source / rule | Class |
|---|---|---|---|---|
| `left_foot_pose` | {`position`: float[3], `orientation`: float[4]} | m, – | composed world pose of `l_ankle` (REV1.2, R4: model entry · `l_ankle` entry, same `Pose_V` message, § 3.3 pose frame semantics; the raw `l_ankle` entry is model-relative, never a world pose), causal last-not-after ≤ `0.02 s` | derived (composition of measured pose/info entries; **derived_pending_validation** while V-2 composition_status is UNVERIFIED — R8/R9) |
| `right_foot_pose` | {`position`: float[3], `orientation`: float[4]} | m, – | composed world pose of `r_ankle` (REV1.3/R9 correction: model entry · `r_ankle` entry, same `Pose_V` message, same rule as `left_foot_pose`; the raw `r_ankle` entry is model-relative, never a world pose), causal last-not-after ≤ `0.02 s` | derived (composition of measured pose/info entries; **derived_pending_validation** while V-2 composition_status is UNVERIFIED — R8/R9) |
| `left_foot_velocity` | float[3] | m/s | central finite difference of `left_foot_pose.position`; **`derived_pending_validation` — inherits from `left_foot_pose` while composition_status is UNVERIFIED (R11-A)** | derived |
| `right_foot_velocity` | float[3] | m/s | central finite difference of `right_foot_pose.position`; **`derived_pending_validation` — inherits from `right_foot_pose` while composition_status is UNVERIFIED (R11-A)** | derived |

## 3.6 Contact (raw-first, lossless; wrench, not boolean)

Stage 0 recorded booleans only. Stage 1 records the contact evidence **raw-first** (REV1.1): the
primary contract artifact is the lossless raw payload; per-foot aggregates and moments are
derived additions, never replacements.

### 3.6.1 Raw payload retention (mandatory, lossless — REV1.1; source corrected REV1.2/R1)

**Source (REV1.2, R1 correction): the Gazebo native `Contacts` stream is the canonical raw
evidence source.** The collector subscribes, via gz-transport, to the native topics
`/nao/left_foot_contact_raw` and `/nao/right_foot_contact_raw` (gz `msgs.Contacts`; each
`msgs.Contact` entry carries the full evidence below) and retains the native payload losslessly.
The two topics are stored **separately** — never summed or merged at write time.

The ROS bridge stream on the same names (`ros_gz_interfaces/msg/Contacts`) is an **optional
derived stream only**: it may be recorded as an additional cross-validation copy but is never
the raw evidence source and never a substitute for the native capture. The bridge is lossy, and
its field loss is a **measured, documented fact** (probe `bridge_field_loss`, run evidence):
the per-contact `header` stamp, the per-contact `world` field, and `Wrench.force_offset` have
no ROS-bridge counterpart — all three are mandatory raw-retention fields below, which is why
native gz capture is mandatory.

For each `Contacts` message, the collector retains, without reduction:

| gz message field | Retained content |
|---|---|
| `Contacts.header` | header (incl. timestamp) verbatim |
| `Contact.header` | per-contact header (incl. timestamp) verbatim |
| `Contact.collision1`, `collision2` | collision names verbatim (collision identity) |
| `Contact.wrench[]` (`JointWrench`) | every entry verbatim: `body_1_name`, `body_1_id`, `body_2_name`, `body_2_id` (body identity), `body_1_wrench`, `body_2_wrench` (both body sides) |
| `Wrench` (each body side) | `force` (Vector3d), `torque` (Vector3d), `force_offset` (Vector3d) verbatim |
| `Contact.position[]`, `normal[]`, `depth[]` | per-contact-point position / normal / depth arrays verbatim |
| `Contact.world` | per-contact world field verbatim, if populated |
| stream metadata | per-topic capture metadata: topic name, message type, transport (gz native), per-message arrival order, and the § 3.6.3 association record. If the optional ROS bridge copy is also captured, its metadata must record that it is a derived stream with the field loss documented above |

**Prohibitions (fail-closed):**

- Per-foot resultant force/moment fields (§ 3.6.2) are **derived additions**; writing them in
  place of the raw payload is a contract violation. Derived per-foot force/moment data may
  **never replace** the raw native evidence.
- The ROS bridge stream is an optional derived stream only (REV1.2, R1); bridge data may never
  be presented as, or persisted in place of, the native `Contacts` payload.
- Bilateral summation (`body_1_wrench + body_2_wrench`) may be computed downstream, never
  recorded as a substitute for both body-side wrenches.
- Contacts within one foot are never merged if merging discards contact-level records.
- No source field may be dropped, rounded, aggregated, or re-expressed before persistence.

### 3.6.2 Per-frame derived contact fields (additions, not replacements)

| Field | Type | Unit | Source / rule | Class |
|---|---|---|---|---|
| `left_contact` | bool | – | `/left_foot/contact`, freshness last-not-after ≤ `0.05 s` (Stage 0 rule, unchanged) | measured |
| `right_contact` | bool | – | `/right_foot/contact`, same rule | measured |
| `left_contact_force` | float[3] ∪ null | N | sum of the left sensor's reported wrench force components, expressed in the **world** frame — published **only after** the frame/reference interpretation is **determined** by gate V-1 and the pinned transform recorded; `null` until then | derived (from raw) |
| `right_contact_force` | float[3] ∪ null | N | same, right sensor | derived (from raw) |
| `left_contact_moment` | float[3] ∪ null | N·m | net moment about the **left foot (ankle) frame origin** — same V-1 precondition (moment reference point must be determined, not assumed); `null` until then | derived (from raw) |
| `right_contact_moment` | float[3] ∪ null | N·m | same, right foot frame origin | derived (from raw) |
| `left_contact_wrench_state` | `VALID_CONTACT` ∪ `UNKNOWN_NO_MESSAGE` | – | Stage 1 two-state contract (REV1.3/R7); `VALID_NO_CONTACT` is a **future extension only**, not part of the Stage 1 acquisition contract — see § 3.6.4 | meta |
| `right_contact_wrench_state` | `VALID_CONTACT` ∪ `UNKNOWN_NO_MESSAGE` | – | Stage 1 two-state contract (REV1.3/R7); see § 3.6.4 | meta |

The frozen `Bool` topics (`/left_foot/contact`, `/right_foot/contact`) are **kept** so the boolean
definition remains byte-identical to Stage 0 (cross-stage comparability). The Bool is produced by
`contact_converter` from the **same underlying contact stream**; it is therefore **not
independent evidence** of no-contact and can never upgrade a silence frame — under the
REV1.3/R7 two-state contract it is not used for wrench validity at all (silence is
`UNKNOWN_NO_MESSAGE`; a future `VALID_NO_CONTACT` extension would require an independent
validated source, GPT-5.6 correction 4).

### 3.6.3 Frame association of contact messages

Raw contact messages are stored as events with their own stamps (§ 3.6.1). Their association with
joint-clock frames uses the declared rule: for frame `t`, the associated raw message set is all
left/right-topic messages with stamp in `(sim_time_sec[t] − 0.05 s, sim_time_sec[t]]` (causal
last-not-after; `0.05 s` = Stage 0 freshness value, pre-registered). The association (which raw
messages, at what stamps) is recorded per frame. No interpolation, no hold, no zero-fill across
frames. A frame with no associated message records an empty association set and wrench state
`UNKNOWN_NO_MESSAGE` (§ 3.6.4; the Stage 1 contract has no other silence state). Streams are
never aligned by callback arrival time; callback arrival time is not valid measurement time
(REV1.2/R2).

### 3.6.4 Wrench validity semantics — Stage 1 two-state contract (REV1.3/R7)

**REV1.0's rule "`contact = false` and no message → force = `[0,0,0]`, `valid = true`" is
deleted.** For an event-only or lossy stream, silence is uninterpretable, and a Bool derived from
the same stream is not independent evidence that a zero wrench was observed. The Stage 1
acquisition contract therefore operates in exactly **two states** (fail-closed):

- `VALID_CONTACT` — at least one associated raw message within the window with relevant fields
  populated; the wrench evidence is observed.
- `UNKNOWN_NO_MESSAGE` — **no contact message** within the window (event-only stream, dropped
  delivery, or no update). **No message never equals a zero wrench.** Derived force/moment are
  `null`; **no interpolation, no hold, no zero-fill; no fabricated flight labels; no fabricated
  no-contact state.** The frame records missing contact evidence, nothing else. Downstream
  analyses must treat these frames as missing contact evidence.

**`VALID_NO_CONTACT` — future extension only, NOT part of the Stage 1 acquisition contract
(REV1.3/R7).** If ever activated by a future RFC revision, it would require exactly one of:
(a) an **explicit empty contact message** — an associated `Contacts` message within the window
whose contact list is empty — or (b) an **independent validated no-contact source** (a
pre-registered signal from a different code path, verified by a probe gate). Nothing in Stage 1
may emit it; a same-stream-derived Bool never suffices.

- Wrenches are never averaged, extrapolated, or gap-filled across frames.
- **Frame/reference interpretation status:** whether body-side wrench force/torque are expressed
  in world or link axes, and about which reference point torque is taken, is **determined by
  V-1** (Appendix B) — via documented Gazebo message semantics plus common-frame/common-origin
  comparison — never assumed, and never inferred solely from Newton's-third-law summation or
  standing-load magnitudes. Until determined and pinned, derived moment/CoP fields remain `null`
  (**derivation blocked**), while **raw acquisition remains authorized** — the retained raw
  evidence allows later reinterpretation; discarded aggregates would not.

## 3.7 ZMP / CoP

Two deliberately independent quantities (this is a design decision, not redundancy):

- `cop_position` — **kinetics** (from measured contact wrenches): the point on the ground contact
  plane at which the net measured ground-reaction wrench produces zero horizontal moment.
  Single support: computed from that foot's measured force/moment. Double support: computed from
the sum of both feet's wrenches about the world origin, then projected. The pinned formula:
with net force `F = (F_x, F_y, F_z)` and net moment `τ` about the world origin,
`cop = ( (−τ_y·F_z − ... )/... )` — the exact closed-form expression is fixed **before the first
CoP derivation**, which may occur after raw acquisition (REV1.1: the derivation contract is
decoupled from the collection contract), and is recorded in a dedicated `cop_derivation_hash`
(§ 3.10) **separate from** the collector `config_hash`; once fixed it is immutable. CoP derivation
is additionally **blocked until V-1 determines the wrench frame/reference interpretation**
(§ 3.6.4) — raw acquisition is not blocked by this.
- `zmp_position` — **kinematics** (LIPM formula, model side):

```math
p_x \;=\; c_x \;-\; \frac{c_z - p_z^{floor}}{g}\,\ddot{c}_x
\qquad
p_y \;=\; c_y \;-\; \frac{c_z - p_z^{floor}}{g}\,\ddot{c}_y
```

with `g = 9.81 m/s²` (pinned for this kinematics formula), `p_z^{floor}` = world z of the contact
plane (pinned constant from `nao_world.sdf`), and `c`, `c̈` from § 3.4. **Recorded discrepancy
(REV1.1, P2, non-blocking):** `nao_world.sdf` sets world gravity to `-9.8 m/s²`, not `-9.81`. This
affects derived-ZMP consistency only, never raw observation. The derivation stage must pin the
`g` choice (recorded world gravity or the formula constant) together with the
`cop_derivation_hash` before first derivation.

| Field | Type | Unit | Source / rule | Class |
|---|---|---|---|---|
| `cop_position` | float[2] ∪ null | m | measured-wrench formula above; `null` when total vertical force < `1.0 N` (flight — pre-registered threshold); **R11-A: Option B — `null` + derivation blocked while the wrench frame/reference is undetermined (§ 3.6.4, § 3.7); never validated until the V-1 common-frame/common-origin determination** | derived (from measured wrenches) |
| `zmp_position` | float[2] ∪ null | m | LIPM kinematics formula above; `null` only when required inputs are invalid; **`derived_pending_validation` — inherits from the CoM chain while composition_status is UNVERIFIED (R11-A)** | derived (from derived CoM) |

The discrepancy `zmp − cop` is **the** LIPM-consistency signal: for a perfectly LIPM-consistent
rigid flat-contact system the two coincide. Recording both independently is what makes § 6
validation possible without circularity (neither quantity is used to compute the other).

## 3.8 IMU (inherited, unchanged from Stage 0)

| Field | Type | Unit | Source / rule | Class |
|---|---|---|---|---|
| `imu_orientation` | float[4] | – | `/imu/data`, symmetric nearest within ±`0.01 s` (non-causal, Stage 0 §6.3) | measured |
| `imu_linear_acceleration` | float[3] | m/s² | same | measured |
| `imu_angular_velocity` | float[3] | rad/s | same | measured |

## 3.9 Command (inherited + deterministic extensions)

| Field | Type | Unit | Source / rule | Class |
|---|---|---|---|---|
| `controller_command` | float[24] | rad | the **actual command/reference published on `/joint_command`** — what the controller/gait generator sent; hold-last ≤ `0.05 s` (Stage 0 rule, unchanged). It is **not** measured joint state, **not** torque, and **not** the realized actuator output applied by the physics engine; its source message stamp is retained (REV1.1 clarification, GPT-5.6 P0-R4) | measured |
| `gait_phase` | {`phase`: string, `fraction`: float ∈ [0,1)} | – | deterministic derivation from `(θ, t)` via the frozen ROS-free model `src/nao_gz_sim/scripts/nao_gait_phase.py` (classify + normalized phase time); no simulation feedback is involved, hence provenance-clean | derived (deterministic, replayable) |
| `desired_velocity` | float[2] | m/s | deterministic from θ per the frozen Stage 0 parameterization: `v_x = step_length × step_frequency`, `v_y = 0`; exact definitions pinned to the Stage 0 GaitGenerator semantics | derived (deterministic) |

## 3.10 Provenance (episode-level, mandatory, no exceptions)

Every episode record must carry:

> **R11-B (REV1.4) / R14: this table is the single source of truth for collector provenance
> completeness.** The collector MUST emit every field below. Collector-contract checklists
> and smoke validations are *derived from* and *must equal* this list — a simplified or
> paraphrased provenance subset is contract drift and is a validation failure. **R14: the
> machine-readable semantics (required / nullable / type) live in Appendix D**; this prose
> table's field names must equal the Appendix D `provenance_fields` names (machine-checked),
> and the checker reads semantics from Appendix D, never from prose keywords.

| Field | Source / rule |
|---|---|
| `theta` | full parameter dict (verbatim from the frozen θ list entry) |
| `theta_id` | the frozen θ-list entry's identity label (e.g. `r0_anchor`) — an identity, not a hash |
| `theta_hash` | per-θ canonical JSON hash (Stage 0 method) |
| `theta_list_hash` | `844646f5…` — cross-checked fail-closed before every episode (inherited Stage 0 tamper check) |
| `git_commit` | HEAD of the collector at acquisition time |
| `config_hash` | SHA-256 of the collector configuration (subscriptions, alignment constants, association window, serialization rules) — **excludes** any CoP derivation formula (REV1.1 decoupling, per GPT-5.6 P0-R5) |
| `cop_derivation_hash` | SHA-256 of the CoP/derived-wrench derivation configuration (closed-form formula, `g` choice, frame/reference transforms as determined by V-1), fixed **before first derivation**; `null` until then. Separate field, separate freeze time (§ 3.7) |
| `gazebo_seed` | the seed of the run (inherited from the Stage 0 per-episode `run_config` value for the same (θ, seed) pair) |
| `robot_model_version` | `nao_simple.sdf` sha256 `44ed8285…` (§ 0.1) |
| `world_model_version` | `nao_world.sdf` sha256 `383341d1…` (§ 0.1) |
| `gz_sim_version` | `8.15.0` (recorded, not pinned — environment fact) |
| `stage1_rfc_hash` | sha256 of this RFC at the revision under which the episode was collected |
| `acquisition_wallclock_utc` | wall-clock acquisition date (UTC) — distinct time domain from `sim_time_sec`; no cross-domain arithmetic |
| `ring` | sampling ring label (§ 4) |
| `frame_count`, `t_start`, `t_end` | episode extents |
| `episode_integrity_hash` | sha256 over the episode's canonical sample file (self-verification) |

`theta`, `theta_id`, and `theta_hash` describe the requested (pre-registered) configuration. `config_hash` identifies the Collector configuration, and `cop_derivation_hash` identifies the CoP/derived-wrench derivation configuration. None of these fields—by existence, equality, or recomputability—constitutes evidence that the producer applied the corresponding gait parameters at runtime; verifying actual application requires trustworthy, readable producer-side applied-configuration evidence uniquely linked to the episode. (`cop_derivation_hash` is not an episode-data integrity hash.)

---

# 4. Sampling Contract

## 4.1 Pre-registration (before any Stage 1 data exists)

The entire Stage 1 population is fixed **now**, in this RFC:

- **θ population:** verbatim the frozen Stage 0 pre-registered list — 32 θ
  (`dataset/stage0/stage0_theta_list.json`, `theta_list_hash = 844646f5…`), rings inherited:
  anchor (ring 0), local perturbation (ring 1), parameter coverage (ring 2), failure boundary
  (ring 3).
- **Seeds:** the same 3 seeds per θ as Stage 0 (values inherited from the Stage 0 per-episode
  `run_config`; identical (θ, seed) pairs → 96 episodes).
- **Strategy:** re-execute the identical pre-registered 32 × 3 grid with extended instrumentation.

Rationale (pre-declared, not post hoc): dynamics observables are new physical measurements that
cannot be attached to Stage 0 retroactively (§ 1.2). Re-running the **same** pre-registered grid
(a) introduces zero optimization pressure — the grid was fixed by Stage 0 coverage logic before
any dynamics data existed and is re-fixed here before any Stage 1 data exists; (b) yields
behavior↔dynamics correspondence per (θ, seed) pair, since Stage 0 behavior labels for the same
pairs are already frozen.

## 4.2 Inherited prohibitions (Stage 0 principles, unchanged)

- **No optimization-driven sampling.** No θ may be added, dropped, duplicated, or re-weighted
  because of measured performance, fit quality, residual magnitude, or any downstream metric.
- **No discarding bad episodes.** Falls, non-walking episodes, `behavior_success = unknown`
  episodes, degraded episodes (§ 5) — all are retained and labeled. The ring 3 failure boundary is
  explicitly re-observed, not avoided.
- **No early stopping** based on model fit or dataset "quality" trends.

## 4.3 Tamper and integrity checks (fail-closed, inherited BF-3 pattern)

- Before every episode: recompute `theta_list_hash` over the frozen list; mismatch → abort
  collection entirely (no skip-and-continue).
- Corpus index: self-verifying (`content_hash` over own content, excluding the field itself),
  atomically written, with a volatile-field sidecar — the Stage 0 `corpus_index.json` /
  `corpus_index.meta.json` pattern is inherited verbatim.
- Storage root: `dataset/stage1/` (new). **Writing into `dataset/stage0/` is forbidden under all
  circumstances** — Stage 0 is frozen.

## 4.4 Modification clause

Any change to the population, seeds, rings, schema, or formulas after the first Stage 1 episode is
collected requires an RFC revision (REV increment) with the change recorded — the dataset records
the RFC hash under which each episode was collected (§ 3.10). No silent drift.

---

# 5. Measurement Requirement

## 5.1 Per-timestep tuple

For every frame `t` (except the last frame of an episode), the dataset must support constructing:

```text
(s_t, u_t, s_{t+1})
```

with

- **state `s`:** joint (position + velocity), base (pose + derived velocities), CoM (position +
  derived velocity/acceleration), feet (pose + derived velocity), contact (boolean + force +
  moment + validity), IMU;
- **control `u`:** `controller_command` (24), with `gait_phase` and `desired_velocity` as
  deterministic context channels (part of the record, not part of `u`'s physical semantics).

A frame must be **self-contained**: constructing the tuple may not require joining across episodes
or consulting files outside the episode's own records + the pinned model constants.

## 5.2 Consecutivity, Δt, and validity rules

- `(t, t+1)` pairs are valid **only** for consecutive `frame_id`s within the same episode.
  Cross-episode pairs are never formed; episode boundaries are hard boundaries.
- **Transition Δt contract (REV1.1):** the transition interval is
  `Δt = sim_time_sec[t+1] − sim_time_sec[t]` — the **measured** adjacent simulation timestamps,
  never the nominal `0.010 s`. A transition is **invalid** when `Δt ≤ 0` (reset / non-monotonic
  clock) or `|Δt − 0.010| > 0.005` (pre-registered data-quality band around the nominal 100 Hz
  grid — a chosen policy, not a physical constant). **Transition invalidity is not frame
  invalidity:** each of the two frames retains its own per-group validity flags; only the
  `(s_t, u_t, s_{t+1})` tuple use and the finite-difference stencils that span the transition are
  rejected. No separate Δt field is stored (recomputable from adjacent `sim_time_sec`).
- Clock gaps (dropped `/joint_states` messages, resets) must be flagged in a per-frame
  `clock_valid` mask; they are never silently interpolated.
- **Source stamps preserved (REV1.1, per GPT-5.6 correction 5):** the source message stamps of
  all measured streams — `/joint_states`, each raw contact message (§ 3.6.1), each
  `pose/info` message (§ 3.3 `pose_source_stamp`) — are preserved losslessly. Streams are never
  aligned by callback arrival time and never assumed exactly synchronous; alignment uses only
  the declared rules of § 3. Reset/non-positive Δt invalidates the adjacent transitions and marks
  the clock discontinuity; it does not silently discard the frames.
- Measured quantities are never interpolated or gap-filled. Derived quantities use only the pinned
  finite-difference stencils of § 3, which skip invalid frames and invalid transitions rather
  than bridging them.

## 5.3 Per-episode data-quality marking (mark, never discard)

- Every frame carries validity flags for each mandatory measured group
  (`joint`, `base_pose`, `foot_pose`, `contact`, `imu`, `command`).
- An episode with systematic dropout — pre-registered threshold: **more than 5 % of frames** with
  any mandatory measured field invalid — is retained but marked `degraded = true` in its episode
  record, with the per-group invalid fractions recorded.
- Filtering of degraded episodes for downstream analyses is an **analyst decision** made after
  collection, recorded in analysis documents — never a collector decision.

## 5.4 Measurement-source constraint (read-only instrumentation)

The collector may only **subscribe** to streams that already exist in the current pipeline
(Appendix A). No new publishers, no parameter changes, no model/world/launch modifications, no
converter changes are authorized by this RFC. Existing launch arguments that enable
already-defined behavior (e.g. bridge/converter enable flags) may be used **at runtime**; editing
the launch or converter files themselves remains forbidden (REV1.1 boundary clarification, per
GPT-5.6 §2.8). Two sources are already on the wire but unverified
for this purpose (wrench population; pose-info completeness) — these carry blocking gates V-1/V-2
(Appendix B).

---

# 6. Compatibility with LIPM

The future LIPM state is

```math
X \;=\; [\,c_x,\; c_y,\; \dot{c}_x,\; \dot{c}_y\,]
```

with dynamics

```math
\ddot{c}_x \;=\; \omega^2 (c_x - p_x),
\qquad
\omega^2 = g / c_z
```

(and the y-axis counterpart), under the constant-CoM-height assumption. The Stage 1 schema
supplies every quantity in these equations, and nothing in the equations is computed at collection
time:

| LIPM quantity | Stage 1 field | Notes |
|---|---|---|
| `c_x, c_y` | `com_position[0:2]` | derived from measured link poses |
| `ċ_x, ċ_y` | `com_velocity[0:2]` | pinned finite difference |
| `c̈_x, c̈_y` | `com_acceleration[0:2]` | pinned stencil — deviation target |
| `c_z` | `com_position[2]` | height-constancy testable per episode |
| `p_x, p_y` (realized) | `cop_position` | from **measured** wrenches |
| `p_x, p_y` (LIPM-predicted) | `zmp_position` | from kinematics formula |
| `θ`, phase context | provenance + `gait_phase`, `desired_velocity` | residual model inputs |

The three declared uses map directly onto the schema:

1. **Estimate deviation:** `r(t) = c̈_measured − ω²(c − p_cop)` — computable field-to-field, no
   extra collection.
2. **Validate the model:** distributions of `r`, of `zmp − cop`, and of `c_z` variance over the
   corpus (per ring, per phase) decide LIPM usability — a Stage 2 decision supported, not made,
   here.
3. **Train the residual (future):** `(s_t, u_t, s_{t+1}, θ, phase)` tuples with provenance allow
   `R_φ` training without re-collection.

**Explicit non-commitment:** this RFC does not implement LIPM, does not fit `ω`, does not train
any residual, and does not pre-compute any deviation statistics. Those belong to later stages and
require their own authorization. The schema is designed so that no future stage forces
re-collection.

---

# 7. Out of Scope

Stage 1 **forbids** (this list is exhaustive at the contract level; anything not explicitly
authorized by an accepted RFC revision is out of scope):

- **Optimization of any kind:** CMA-ES, CEM, RL (policy or parameter learning), MPC, any fitness
  function design or optimization, any gait optimizer.
- **Learning of any kind:** PINN training, neural network training, residual training, system
  identification fitting beyond recording raw observables.
- **LIPM implementation or fitting** (including `ω` estimation) — Stage 1 records data only.
- **Any modification to frozen Stage 0 artifacts:** the Stage 0 RFC, the Stage 0 dataset
  (`dataset/stage0/**`), the frozen runner `tools/run_phase_a_episode.py`, the Stage 0 closure
  documents.
- **Any modification to the simulation pipeline** (launch files, converters, `nao_world.sdf`,
  `nao_simple.sdf`, gait generator semantics). Instrumentation is read-only (§ 5.4).
- **Best-θ selection, ranking, or recommendation** — no Stage 1 output may express or imply a gait
  preference.
- **Collector implementation boundary (REV1.1):** the collector may be implemented under Level 2
  authorization (§ 0.0) as a **read-only instrumentation process** — new subscribers, a new writer
  process, `dataset/stage1/` storage only, no modification to any frozen component (runner,
  controller/gait generator, converters, model, world, physics, launch files). **Bulk raw
  acquisition of the 96 episodes is Level 3 and remains gated** on V-1/V-2 pass + pinned probe
  results (Appendix B).

---

# 8. Acceptance Criteria

## 8.1 RFC acceptance (criteria for *this document* — all must hold)

| # | Criterion | Where satisfied |
|---|---|---|
| R1 | **Schema defined** — all ten field groups (Time, Joint, Base, CoM, Foot, Contact, ZMP/COP, IMU, Command, Provenance) with type, shape, unit, source, alignment rule, and measured/derived class | § 3 |
| R2 | **Sampling defined** — pre-registered θ population + seeds + rings fixed before any data; no optimization-driven sampling; no discarding; fail-closed tamper checks | § 4 |
| R3 | **Provenance defined** — θ/hash chain, git commit, config hash, seed, model versions, RFC hash per episode | § 3.10 |
| R4 | **Measurement source identified** — every field mapped to an existing topic or a pinned derivation rule; sources not yet verified carry explicit gates | § 5.4, Appendix A, Appendix B |
| R5 | **No optimization leakage** — no fitness, no ranking, no adaptive sampling; explicit prohibitions | § 4.2, § 7 |
| R6 | **Authorization explicit and tiered** — preparation + collector implementation authorized within the read-only boundary; bulk acquisition gated on V-1/V-2 + pinned results (REV1.1) | § 0.0, § 5.4, § 7 |
| R7 | **Stage 0 immutability respected** — no frozen artifact touched; new storage root `dataset/stage1/` | § 4.3, § 7 |

## 8.2 Dataset acceptance (deferred — criteria for the *future* Stage 1 dataset, satisfiable only by the separately authorized implementation)

| # | Criterion |
|---|---|
| D1 | All 96 pre-registered episodes acquired under gates V-1/V-2 passed **and probe results pinned**; a V-1 failure on wrench population / body identity / contact-event observation blocks acquisition (no partial pivot; event-only publication itself is two-state-consistent, REV1.3/R7); an unresolved frame/reference interpretation blocks derived moment/CoP fields but not raw acquisition (§ 3.6.4) |
| D2 | Provenance fields present for 100 % of episodes; `theta_list_hash` fail-closed checks all passed |
| D3 | Per-frame validity flags present for 100 % of frames; degraded episodes marked, none discarded |
| D4 | Corpus index self-verification (content_hash) passes; commit scope decided by owner at implementation time |
| D5 | `(s_t, u_t, s_{t+1})` tuples constructible for 100 % of consecutive frame pairs without cross-episode joins |

D-criteria are **not** claimed satisfied by this document. They bind the implementation phase.

---

# Appendix A — Measurement source inventory

All sources exist in the current pipeline; the collector adds subscriptions only.

| # | Field groups | Source | Type | Status |
|---|---|---|---|---|
| A1 | frame clock, joint pos/vel | `/joint_states` | `sensor_msgs/JointState` | frozen (Stage 0) |
| A2 | controller_command | `/joint_command` | `trajectory_msgs/JointTrajectory` (reduced) | frozen (Stage 0) |
| A3 | IMU | `/imu/data` | `sensor_msgs/Imu` | frozen (Stage 0) |
| A4 | contact boolean | `/left_foot/contact`, `/right_foot/contact` | `std_msgs/Bool` | frozen (Stage 0) — definition must remain byte-identical |
| A5 | base_pose | `/body_pose` | `geometry_msgs/PoseStamped` | frozen (Stage 0) |
| A6 | **contact wrench** | `/nao/left_foot_contact_raw`, `/nao/right_foot_contact_raw` — **gz-transport native, the canonical raw source (REV1.2/R1)** | gz `msgs.Contacts` (each `Contact` carries `repeated JointWrench`) | on the wire — **wrench population unverified → gate V-1**. The ROS bridge counterpart (`ros_gz_interfaces/Contacts`) is an **optional derived stream only** (measured field loss: per-contact header stamp, `world`, `force_offset`) |
| A7 | **all 25 link poses + `nao` model entry** (base pose cross-check, CoM, foot poses) | `/world/nao_world/pose/info` | gz `msgs.Pose_V` (SceneBroadcaster) | on the wire — **completeness/rate unverified → gate V-2**. Entries are parent/model-relative (REV1.2/R4); world poses are **derived by composition** with the model entry (§ 3.3 pose frame semantics) |
| A8 | gait_phase, desired_velocity | deterministic derivation (§ 3.9) | `nao_gait_phase.py` + pinned formulas | derived |
| A9 | joint/base/foot/CoM velocities & accelerations | deterministic finite differences (§ 3.2–3.5) | pinned stencils | derived |

# Appendix B — Verification gates (must pass before Stage 1 collection starts)

- **V-1 (blocking): contact wrench availability and semantics.** Executed by the read-only probe
  `tools/stage1_probe.py` (subscribes to both `*_foot_contact_raw` topics — on the gz-transport
  **native** path (canonical, R1) and on the ROS bridge path (derived, field-loss documentation)
  — and `/joint_states`; report contract: `dataset/stage1/probe/V1_report_REV13.json`, REV1.3
  evidence run; prior evidence retained: REV1.2 `V1_report_REV12.json` + run-4 end-hold archive,
  REV1.1 run-3 `V1_report.json` + archived fallen-run reports). The probe must **determine**
  (never assume; report each finding as `confirmed` / `plausible` / `unknown`):
  1. **Event-aware contact publication validation (REV1.3/R6 — replaces the removed absolute
     effective-contact-frequency [50, 150] Hz band).** Only intervals containing contact
     messages are evaluated; reported metrics: `active_contact_event_rate`, `median_event_dt`,
     `p95_event_dt`, `max_event_dt` (within contact bursts), and burst duration. **Silence
     intervals are not cadence failures**; silence semantics remain `UNKNOWN_NO_MESSAGE`. The
     `JointWrench` fields of both topics must be actually populated at runtime with per-contact,
     per-body-side force, torque, and `force_offset` values (schema presence alone proves
     nothing).
  2. **Body identity** — `body_1_name` / `body_2_name` map to the expected ankle/ground collision
     pairs, left/right distinguishable, no cross-foot contamination.
  3. **Publication mode (REV1.3/R7 — under the Stage 1 two-state contract).** Periodic vs.
     event-only publication and whether empty-contact messages are ever sent are **determined
     facts**: event-only publication is **contract-consistent** (silence is recorded as
     `UNKNOWN_NO_MESSAGE`; no zero wrench from silence, no fabricated flight labels, no
     fabricated no-contact state) and is **not a blocking failure**. Empty-message behavior is
     recorded for the future `VALID_NO_CONTACT` extension only.
  4. **Timestamp association compatibility (REV1.3/R10 — formerly "timestamp semantics").**
     Validation uses **source timestamps only** and proves that contact source stamps are
     **associable** to the `/joint_states` grid: **nearest-neighbor statistics** — for every
     contact message source stamp, the absolute difference to the nearest `/joint_states`
     source stamp, reported as **median, p95, and max**; plus monotonicity and measured dt of
     the source stamps. The statistics are **evaluated on the common subscription window**:
     contact stamps received before the `/joint_states` subscription became live (rclpy/DDS
     discovery skew — the documented instrumentation boundary, collector contract § 5.3) are
     excluded from the statistics and reported as boundary-excluded samples, never silently
     dropped. Source stamps are preserved verbatim. **The evidence proves association
     compatibility; it does NOT prove universal clock identity.** **Callback arrival time is
     not valid measurement time.** Envelope-overlap of subscriber capture windows is explicitly
     **not** a criterion (the REV1.1 envelope-fraction check is deleted: it measured subscriber
     discovery skew — a probe instrumentation artifact).
  5. **Frame/reference interpretation** — established from documented Gazebo message semantics
     plus a **controlled comparison after transforming both sides to a common frame and a common
     moment origin**. Newton's-third-law summation and standing-load magnitudes are **plausibility
     checks only**: they do not by themselves establish world-vs-body axes or the torque reference
     point, and standing torque magnitude must never be used to reverse-infer the moment origin
     (GPT-5.6 P1-R1). Per-foot static support near `≈ 26 N` (total `m·g ≈ 52 N`, `m = 5.305 kg`,
     `g = 9.8 m/s²`) is a gross sanity range, not unit proof.

  If the wrenches are not populated, body identity is wrong, or contact events are not observed,
  **Stage 1 acquisition is blocked until this
  RFC is revised. No force-free fallback is pre-authorized** (estimating forces from kinematics
  would defeat the purpose of § 1). Event-only publication is not a blocking condition under the
  REV1.3/R7 two-state contract. If frame/reference remains unresolved, **raw acquisition
  proceeds but CoP/derived-moment derivation is blocked** (§ 3.6.4, § 3.7).

- **V-2 (blocking): pose-info completeness, semantics, and rate.** Executed by the same probe
  via gz-transport subscription to `/world/nao_world/pose/info` (report:
  `dataset/stage1/probe/V2_report_REV13.json`, REV1.3 evidence run; prior evidence retained:
  REV1.2 `V2_report_REV12.json`, REV1.1 run-3 `V2_report.json`). Verify:
  1. **Unique entities** — all 25 links of `nao_simple.sdf` (incl. `base_link`, `l_ankle`,
     `r_ankle`) appear as **unique, exactly-named** entries (25 strings occurring in `Pose_V` is
     not sufficient).
  2. **Pose frame semantics (REV1.2/R4 correction).** `Pose_V` link entries are
     **parent/model-relative poses** — the REV1.1 assumption "each entry is the world pose of the
     corresponding link entity" is **falsified by probe evidence** (identity-pinned `base_link`
     entry while the `nao` model entry moves) and entries must **never** be recorded as world
     poses. The world pose of a link requires **composition**:
     `T_world_link = T_world_model · T_model_link`, with both factors from the same `Pose_V`
     message. The model entry (composition anchor) is cross-checked against `/body_pose` (which
     `body_pose_converter` identifies from the model entry; exact-stamp comparison). The frame
     semantics is **determined from in-band evidence, never assumed**. **REV1.2/R5:** the
     composition method **requires verification using selected non-identity links** (e.g.
     `l_ankle` / `r_ankle`) before composed poses are treated as validated world poses —
     identity-link (`base_link`) agreement alone is a consistency check, not a validation.
     **REV1.3/R8 — composition evidence (an extension of this criterion, not a new gate):** the
     probe must report `composition_evidence` for **at least two non-identity links**
     (`l_ankle`, `r_ankle`, optionally `Head`) composed via `T_world_model × T_model_link`, and
     compare against an **independent available pose source** if one exists. If no independent
     non-identity-link world-pose source is available, the report must state
     `composition_status: UNVERIFIED` — **validation must not be claimed**. While
     `composition_status` is `UNVERIFIED`, composed foot/CoM fields remain
     `derived_pending_validation` (§ 3.3 note, § 3.5); the collector may still store the raw
     `Pose_V` entries and the model pose.
     Fixed standing geometry remains an informational sanity check (standing scenarios only).
     An existing subscriber proves consumption, not semantics.
  3. **Source stamps and cadence** — per-message stamps are recorded; the observed publication
     cadence is measured (SceneBroadcaster has no explicit publish_rate configured — measured,
     not assumed).
  4. **Alignment hit rate (the operative check)** — the fraction of joint-clock frames whose
     causal last-not-after `0.02 s` window contains at least one pose message is **≥ 95 %**;
     misses are flagged per frame (a nominal average frequency alone is not proof).

  Failure → RFC revision required (no silent widening of tolerances). Passing V-2 validates pose
  coverage and frame semantics only; the mass/CoM computation itself is a § 3.4 derivation, not
  validated here.

- **Probe result pinning (REV1.1; file naming per REV1.3):** V-1/V-2 reports record their git
  commit, the RFC hash under which they ran, the invocation command (argv), and a config hash.
  **Level 3 (bulk acquisition, § 0.0) requires both reports to exist, pass
  their blocking criteria, and be committed** before the first production episode.
- **V-3 (non-blocking, informational):** presence of a native ground-truth twist stream; if
  absent, the derived finite-difference twist of § 3.3 is the contract (no revision needed).

# Appendix C — Storage layout (contract-level)

```text
dataset/stage1/
  probe/                      # pre-acquisition probe evidence (REV1.3)
                              #   V1_report_REV13.json, V2_report_REV13.json
                              #   (REV1.2: V1/V2_report_REV12.json + run-4
                              #    end-hold archive; REV1.1 run-3:
                              #    V1/V2_report.json + archived fallen-run
                              #    reports) — pinned before Level 3
  stage1_theta_list.json      # verbatim copy of the frozen Stage 0 list
                              # + cross-checked theta_list_hash (fail-closed)
  raw/                        # lossless raw stream captures per episode
                              #   (§ 3.6.1 contact payload; source-stamped pose
                              #    events) — immutable once written
  derived/                    # recomputable derived artifacts (per-foot
                              #   aggregates, aligned .jsonl frames, CoP once
                              #   unblocked) — never replacements for raw/
  ring0/ ring1/ ring2/ ring3/ # per-episode directories, Stage 0 conventions
  records/                    # *.record.json + *.metrics.json per episode
  provenance/                 # collector config, hashes, episode provenance
  corpus_index.json           # self-verifying (content_hash), atomic write
  corpus_index.meta.json      # volatile sidecar (BF-3 pattern)
```

- `dataset/stage0/**` is never written (§ 4.3).
- The per-frame aligned samples (`.jsonl`) **and** the lossless raw payload captures (§ 3.6.1) are
  both contract artifacts — REV1.1 makes raw retention mandatory (not an owner option); only the
  git commit scope for `dataset/stage1/` remains an owner decision at implementation time.
- The exact raw/derived/provenance layout, immutability and recomputation rules are specified in
  `docs/STAGE1_COLLECTOR_CONTRACT.md` (Level 1 deliverable, referenced by this contract).

# Appendix D — Normative machine-readable schema (REV1.5 / R14)

The JSON block below is the **single machine-readable normative schema** of this contract
(R14): the Collector Contract embeds an identical copy (its § 8.4), and the smoke checker
(`tools/stage1_contract_check.py`) loads its field lists, statuses, shapes, provenance
semantics, raw-retention requirements, and the frame-artifact count rule **from this block at
run time — the checker owns no independent schema**. The prose sections of this RFC remain the
human-readable definition; **in case of any discrepancy, this block governs**. Provenance field
names must additionally equal the § 3.10 table and the Collector Contract § 8 checklist
(machine-checked). Modifying this block is an RFC revision (§ 4.4): any schema/semantic
mutation makes previously-valid artifacts fail `--smoke` (fail-closed), and any drift between
this block and the Collector Contract's copy is a validation failure. Explicit machine-readable
semantics (`required`, `nullable`, `type`, `shape`) replace all prose-keyword parsing (R14/B2).

```json
{
  "schema_version": "stage1-normative-schema/1.0",
  "status_semantics": {
    "composition_status": "UNVERIFIED",
    "wrench_frame_status": "UNKNOWN",
    "wrench_blocked_statuses": ["null_blocked", "blocked"],
    "valid_field_statuses": ["measured", "derived", "derived_pending_validation"]
  },
  "frame_fields": {
    "measured": {"status": "measured", "fields": [
      {"name": "joint_position"},
      {"name": "joint_velocity"},
      {"name": "base_pose"},
      {"name": "pose_source_stamp"},
      {"name": "left_contact"},
      {"name": "right_contact"},
      {"name": "imu_orientation"},
      {"name": "imu_linear_acceleration"},
      {"name": "imu_angular_velocity"},
      {"name": "controller_command"}
    ]},
    "ordinary_derived": {"status": "derived", "fields": [
      {"name": "joint_acceleration"},
      {"name": "base_velocity"},
      {"name": "base_angular_velocity"},
      {"name": "gait_phase"},
      {"name": "desired_velocity"}
    ]},
    "option_a": {"status": "derived_pending_validation", "fields": [
      {"name": "left_foot_pose", "type": "pose", "position": 3, "orientation": 4},
      {"name": "right_foot_pose", "type": "pose", "position": 3, "orientation": 4},
      {"name": "left_foot_velocity", "type": "vector", "length": 3},
      {"name": "right_foot_velocity", "type": "vector", "length": 3},
      {"name": "com_position", "type": "vector", "length": 3},
      {"name": "com_velocity", "type": "vector", "length": 3},
      {"name": "com_acceleration", "type": "vector", "length": 3},
      {"name": "zmp_position", "type": "vector", "length": 2}
    ]},
    "option_b": {"status": "null_blocked", "fields": [
      {"name": "left_contact_force", "type": "vector", "length": 3},
      {"name": "right_contact_force", "type": "vector", "length": 3},
      {"name": "left_contact_moment", "type": "vector", "length": 3},
      {"name": "right_contact_moment", "type": "vector", "length": 3},
      {"name": "cop_position", "type": "vector", "length": 2}
    ]}
  },
  "provenance_fields": [
    {"name": "theta", "required": true, "nullable": false, "type": "object"},
    {"name": "theta_id", "required": true, "nullable": false, "type": "string"},
    {"name": "theta_hash", "required": true, "nullable": false, "type": "sha256"},
    {"name": "theta_list_hash", "required": true, "nullable": false, "type": "sha256"},
    {"name": "git_commit", "required": true, "nullable": false, "type": "commit40"},
    {"name": "config_hash", "required": true, "nullable": false, "type": "sha256"},
    {"name": "cop_derivation_hash", "required": true, "nullable": true, "type": "sha256"},
    {"name": "gazebo_seed", "required": true, "nullable": false, "type": "int"},
    {"name": "robot_model_version", "required": true, "nullable": false, "type": "sha256"},
    {"name": "world_model_version", "required": true, "nullable": false, "type": "sha256"},
    {"name": "gz_sim_version", "required": true, "nullable": false, "type": "string"},
    {"name": "stage1_rfc_hash", "required": true, "nullable": false, "type": "sha256"},
    {"name": "acquisition_wallclock_utc", "required": true, "nullable": false, "type": "string"},
    {"name": "ring", "required": true, "nullable": false, "type": "string"},
    {"name": "frame_count", "required": true, "nullable": false, "type": "int_nonneg"},
    {"name": "t_start", "required": true, "nullable": false, "type": "number_finite"},
    {"name": "t_end", "required": true, "nullable": false, "type": "number_finite"},
    {"name": "episode_integrity_hash", "required": true, "nullable": false, "type": "sha256"}
  ],
  "raw_structures": {
    "contacts": {
      "required_record_keys": ["stamp_s", "contacts"],
      "optional_record_keys": ["n_contacts"],
      "contact_required_keys": ["header", "collision1", "collision2", "position", "normal", "depth", "wrenches"],
      "contact_optional_keys": ["world"],
      "wrench_required_keys": ["body_1_name", "body_1_id", "body_2_name", "body_2_id", "body_1_wrench", "body_2_wrench"],
      "wrench_member_required_keys": ["force", "torque", "force_offset"]
    },
    "pose_info": {
      "required_record_keys": ["stamp_s", "entries"],
      "entry_required_keys": ["name", "position", "orientation"],
      "entry_optional_keys": ["id"]
    },
    "joint_command": {
      "required_record_keys": ["stamp_s", "command"],
      "command_length": 24
    },
    "joint_states": {
      "required_record_keys": ["stamp_s", "name"],
      "optional_record_keys": ["position", "velocity"]
    },
    "body_pose": {
      "required_record_keys": ["stamp_s", "position"],
      "optional_record_keys": ["orientation"]
    }
  },
  "raw_categories": {
    "contacts_left": "contacts",
    "contacts_right": "contacts",
    "pose_info": "pose_info",
    "joint_command": "joint_command",
    "joint_states": "joint_states",
    "body_pose": "body_pose"
  },
  "required_raw_categories": ["contacts_left", "contacts_right", "pose_info", "joint_command"],
  "frame_artifact": {
    "type": "array_of_frames",
    "count_must_equal": "frame_count",
    "empty_episode_allowed": true
  }
}
```

Notes (binding semantics):

- `optional_record_keys` / `contact_optional_keys` / `entry_optional_keys` declare fields that
  **may** be present (validated only for basic type when present); `required_*` keys MUST be
  present. **Unknown native/extension keys are always preserved and never rejected**
  (lossless retention, § 3.6.1) — the checker enforces only the required structure.
- Raw vector shapes (contact `position`/`normal` = vec3, `depth` = finite numbers, wrench
  members = vec3, pose entry `position` = vec3 / `orientation` = quat4) are the checker's
  structural interpretation of the gz message types; their **requiredness** comes from this
  block.
- The smoke frame artifact is an **array of frame records**; `frame_count` (provenance) must
  equal the actual record count; an empty episode (`frame_count = 0`, `frames = []`) is valid
  (R14/B4 — `frame_count` itself remains `int_nonneg`, never forced `> 0`).

---

## Document status

**REV1.6-PROVENANCE-CLARIFICATION — ACQUISITION-CONTRACT-READY (pending final independent audit).**

Revision history:

- **REV1.0-DRAFT** (2026-10-07): original specification, submitted for owner authorization and
  Independent Auditor review. "No implementation authorized."
- **REV1.1-FINAL** (2026-10-07): minimal revision per the GLM-5.3 adversarial audit
  (`STAGE1_RFC_REV1.1_MINIMAL_REVISION_AND_PROBE_DESIGN_AUDIT_REPORT.md`, verdict A) and the
  GPT-5.6 engineering review (Decision B — READY AFTER MINOR FIXES). Changes, in order of the
  GPT-5.6 required corrections:
  1. three-tier authorization; "FINAL" = frozen acquisition contract, not blanket permission
     (§ 0, § 0.0);
  2. lossless raw contact payload retention mandatory; per-foot aggregates demoted to derived
     additions (§ 3.6.1–3.6.2);
  3. V-1 semantics rewritten: determine-not-assume; N3L/standing values are plausibility checks
     only; common-frame/common-origin comparison required (Appendix B);
  4. flight validity fail-closed: three-state wrench semantics replace the zero-fill rule; Bool
     from the same stream is not independent evidence (§ 3.6.4);
  5. source stamps preserved for all three streams; measured-Δt transition contract;
     transition-invalid ≠ frame-invalid (§ 3.3, § 5.2);
  6. V-2 semantics: unique entities, world-pose semantics, source stamps, ≥ 95 % causal-window
     hit rate (Appendix B);
  7. boundary frozen: launch args runtime-only, no file edits; `dataset/stage0/**`,
     runner, controller/generator, model, world, physics untouched (§ 5.4, § 7);
  8. `config_hash` / `cop_derivation_hash` decoupled; remote push of the Stage 0 closure commit +
     this contract required before production (§ 3.10, § 0.0).
  Also: `sim_time_sec` / `acquisition_wallclock_utc` renames (§ 3.1, § 3.10);
  `controller_command` actual-published semantics (§ 3.9); gravity 9.8/9.81 discrepancy recorded
  (§ 3.7); probe report pinning required before Level 3 (Appendix B, Appendix C).
- **REV1.2-CORRECTED** (2026-10-08): semantic corrections per the GPT-5.6 independent audit of
  the REV1.1 preparation artifacts (corrections R1–R5; correction task
  `STAGE1_REV1.2_CORRECTION_TASK`). Semantic only — population, seeds, rings, schema shape,
  formulas, and all § 0.1 frozen pins unchanged:
  1. **R1 — raw contact source:** the Gazebo native `Contacts` stream (gz-transport) is the
     canonical raw evidence source; native payload retention mandatory; the ROS bridge is an
     optional derived stream only, with its measured field loss (per-contact header stamp,
     `world`, `force_offset`) documented in the contract; derived per-foot force/moment data
     may never replace raw evidence (§ 3.6.1, Appendix A6);
  2. **R2 — V-1 timestamp validation:** the envelope-overlap criterion is deleted — subscriber
     capture window overlap is not timestamp-domain validation; replaced by source-stamp
     validation: same simulation clock domain via nearest-neighbor statistics (median / p95 /
     max); callback arrival time is not valid measurement time (§ 3.6.3, Appendix B V-1.4);
  3. **R3 — contact validity semantics:** fail-closed rule kept; the three states are
     `VALID_CONTACT` / `UNKNOWN_NO_MESSAGE` / `VALID_NO_CONTACT`; no contact message →
     `UNKNOWN_NO_MESSAGE`, no zero wrench from silence; `VALID_NO_CONTACT` requires an explicit
     empty contact message or an independent validated no-contact source (§ 3.6.2–3.6.4);
  4. **R4 — V-2 pose frame semantics:** `Pose_V` entries are parent/model-relative poses, never
     world poses; world poses require composition `T_world_link = T_world_model · T_model_link`;
     pose fields reclassified accordingly (§ 3.3 note, § 3.4, § 3.5, Appendix A7, Appendix B
     V-2.2);
  5. **R5 — validation wording:** "end-to-end validated" claims for the composition method are
     removed — only an identity-link (`base_link`) comparison exists; the composition method
     requires verification using selected non-identity links before composed poses are treated
     as validated world poses (§ 3.3 note, Appendix B V-2.2).
  Probe re-run evidence under this revision: `dataset/stage1/probe/V1_report_REV12.json`,
  `V2_report_REV12.json`. **REV1.2 does not re-freeze itself** — it is pending re-audit, and no
  acceptance criterion is claimed satisfied by this revision.
- **REV1.3-MINIMAL-HARDENING** (2026-10-08): minimal corrections per the GPT-5.6 independent
  re-audit of REV1.2 (corrections R6–R10; correction task
  `STAGE1_REV13_MINIMAL_HARDENING_TASK`). Hardening only — no new V-gate, no schema change,
  no population/seeds/rings change, no controller/gait/Stage 0 change, no optimization:
  1. **R6 — event-aware contact publication validation:** the absolute effective-contact-
     frequency band [50, 150] Hz is removed; the V-1 publication criterion now reports
     `active_contact_event_rate`, `median_event_dt`, `p95_event_dt`, `max_event_dt` (within
     contact bursts), and burst duration; only intervals containing contact messages are
     evaluated; silence intervals are not cadence failures; silence semantics remain
     `UNKNOWN_NO_MESSAGE` (Appendix B V-1.1);
  2. **R7 — Stage 1 two-state contact validity:** the Stage 1 acquisition contract operates
     exactly `VALID_CONTACT` / `UNKNOWN_NO_MESSAGE`; no message never equals a zero wrench; no
     fabricated flight labels; no fabricated no-contact state. `VALID_NO_CONTACT` is kept as a
     future extension only (explicit empty contact message or independent validated source),
     not required for Stage 1 acquisition; event-only publication is contract-consistent and
     no longer a V-1 blocking condition (§ 3.6.2–3.6.4, Appendix B V-1.3);
  3. **R8 — V-2 composition evidence (extension of the existing criterion, not a new gate):**
     `composition_evidence` validates at least two non-identity links (e.g. `l_ankle`,
     `r_ankle`) via `T_world_model × T_model_link`, compared against an independent pose
     source if available; otherwise `composition_status: UNVERIFIED` and no validation claim;
     composed foot/CoM fields remain `derived_pending_validation` until evidence exists
     (§ 3.3 note, § 3.5, Appendix B V-2.2);
  4. **R9 — measured/derived labels:** `right_foot_pose` (and `left_foot_pose`) are `derived`
     (not measured); measured = raw gz pose messages, model pose, raw contacts, joint states;
     derived = foot pose, CoM, CoP, ZMP, velocities, transforms (§ 3.0, § 3.4, § 3.5);
  5. **R10 — timestamp wording:** the confirmed V-1 timestamp criterion is renamed/stated as
     **timestamp association compatibility** — the evidence proves the source stamps are
     associable; it does not prove universal clock identity (Appendix B V-1.4).
  Probe evidence under this revision: `dataset/stage1/probe/V1_report_REV13.json`,
  `V2_report_REV13.json`. **REV1.3 is pending final independent audit** — status
  ACQUISITION-CONTRACT-READY is a readiness claim only, not acceptance; no acquisition
  authorization is claimed.
- **REV1.4-HARDENING** (2026-10-08): R11 minimal hardening for the two blockers of the
  GPT-5.6 independent adversarial audit of REV1.3 (Decision B — READY AFTER MINOR FIXES;
  correction task `STAGE1_REV11_MINIMAL_HARDENING_TASK`). No field added/removed, no new
  V-gate, no reopening of R6–R10:
  1. **R11-A — pending-validation propagation:** § 3.0 gains the binding
     validation-status propagation rule (any derived field depending directly or
     indirectly on an input whose validation status is `UNVERIFIED`/pending/blocked must
     carry `derived_pending_validation` or remain `null`+blocked, never validated derived
     data); the adopted null-vs-pending implementation is codified (Option A — value
     retained — for composition-dependent fields; Option B — `null`+blocked — for
     wrench-frame-dependent fields); the three propagation chains are declared (foot
     pose→foot velocity; CoM→CoM velocity→CoM acceleration, ZMP; pose+wrench→CoP/world
     moment blocked); the non-propagation rule protects raw measured data; the schema
     tables annotate `com_velocity`, `com_acceleration`, `left/right_foot_velocity`,
     `zmp_position` as `derived_pending_validation`, and `cop_position` as Option-B blocked;
  2. **R11-B — provenance completeness binding:** § 3.10 is declared the single source of
     truth for collector provenance completeness; the collector must emit every field and
     checklists/smoke validation must equal it; machine-readable enforcement in
     `tools/stage1_contract_check.py` (Tests A–D in `tools/test_stage1_r11.py`).
  `composition_status` remains `UNVERIFIED` and the wrench frame/reference remains
  undetermined — this revision makes those facts machine-actionable, it does not resolve
  them. **REV1.4 is pending final independent audit acceptance; no acquisition
  authorization is claimed.**
- **REV1.5-NORMATIVE-SCHEMA** (2026-10-08): R14 normative-schema binding + artifact
  consistency for the four bypasses of the GPT-5.6 independent closure audit of R13
  (Decision C — BLOCKED; correction task
  `STAGE1_REV14_NORMATIVE_SCHEMA_BINDING_AND_ARTIFACT_CONSISTENCY_TASK`). **Adds Appendix D** —
  the normative machine-readable JSON schema block (single machine source of truth; the
  Collector Contract embeds an identical copy and the checker loads it at run time):
  1. **B1 closure:** Option-A field shapes (pose position[3]/orientation[4], vec3, zmp vec2)
     come from the normative block, not checker hardcoding — an RFC schema mutation makes old
     artifacts fail `--smoke`;
  2. **B2 closure:** explicit `required`/`nullable`/`type` semantics replace all
     prose-keyword parsing ("non-null" can no longer be read as nullable; "may be omitted"
     is an explicit `required: false`);
  3. **B3 closure:** raw-retention requiredness is bound per the block (Contact `header`,
     `body_1_id`/`body_2_id`, JointWrench members, raw pose `orientation`, contact-point
     arrays, 24-vector command); unknown native keys remain preserved;
  4. **B4 closure:** the smoke frame artifact is an array of frame records and
     `frame_count` must equal the actual record count (empty episode `0/[]` is valid;
     `frame_count` stays `int_nonneg`, never forced `> 0`).
  No research-objective / R6–R10 / acquisition-requirement / raw-vs-derived scientific-meaning
  change. **REV1.5 is pending final independent audit acceptance; no acquisition
  authorization is claimed.**
- **REV1.6-PROVENANCE-CLARIFICATION** (2026-10-13): § 3.10 provenance **evidential-boundary
  clarification** per the Owner-approved D4-EPC decision (Owner Decision Freeze O-05; Option B
  — RFC-only prose; Collector Contract unchanged). Adds one prose paragraph to § 3.10 stating
  that `theta`/`theta_id`/`theta_hash` describe the requested (pre-registered) configuration,
  `config_hash` identifies the Collector configuration, `cop_derivation_hash` identifies the
  CoP/derived-wrench derivation configuration, and none of these fields — by existence,
  equality, or recomputability — constitutes evidence that the producer applied the
  corresponding gait parameters at runtime (verifying actual application requires trustworthy,
  readable producer-side applied-configuration evidence uniquely linked to the episode;
  `cop_derivation_hash` is not an episode-data integrity hash). No field added/removed, no
  Appendix D schema-block / `schema_version` change, no population/seeds/rings/formula change,
  no runtime behavior or automatic verification. **D-C (2026-10-13, Owner-authorized): the § 3.10
  table lists `theta_id` and `theta_hash` as separate rows — `theta_id` is the frozen θ-list
  entry's identity label, not a hash (prose-only clarification, no field added/removed).**
  Existing records are unaffected. **REV1.6 is
  pending final independent audit acceptance; no acquisition authorization is claimed.**

**This document authorizes preparation and collector implementation (Levels 1–2, § 0.0) only.
Bulk acquisition of the 96 episodes (Level 3) is NOT authorized by this document alone — it
requires V-1/V-2 pass + pinned probe results + remote publication of the Stage 0 closure commit
and this contract. No analysis, LIPM fitting, residual learning, training, or optimization is
authorized (§ 7).**
