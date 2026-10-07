# STAGE1_DYNAMICS_DATASET_RFC

Stage 1 Dynamics Dataset RFC — specification of the dynamics-observation dataset required to
advance from the frozen Stage 0 behavior baseline to physics identification, LIPM validation, and
residual-learning preparation.

| Field | Value |
|---|---|
| Revision | REV1.0-DRAFT |
| Status | **RFC — specification only. Not authorized for implementation.** |
| Author | Implementation Agent (DeepSeek / Qoder role) |
| Date | 2026-10-07 |
| Roadmap position | Baseline Dataset (Stage 0, frozen) → **Dynamics Dataset (this RFC)** → LIPM Prior → Residual Network → Optimization → Sim2Real |

## 0. Scope declaration (read first)

This document is a **Request For Comments / contract draft**. It contains:

- a schema specification,
- a sampling contract,
- a measurement-source inventory,
- verification gates and acceptance criteria.

It contains **no implementation**. Specifically, this RFC does not include, authorize, or imply:

- a collector / instrumentation script,
- any modification to the simulation pipeline (launch files, converters, world, model),
- any modification to the Stage 0 frozen artifacts.

Stage 1 **implementation** (the collector) is a separate, later step that requires its own
authorization after this RFC is accepted. Until then, nothing in this document may be executed.

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
  unchanged: nominal period `0.010 s` (100 Hz), simulation time.
- Frames are strictly ordered within an episode; `frame_id` is the 0-based index of the
  `/joint_states` message within the episode.
- Per-episode storage: newline-delimited JSON (`.jsonl`), one frame per line, plus episode-level
  metadata records (§ 3.10) in separate files. Layout: Appendix C.
- Every field below carries, in its definition table: name, type/shape, unit, source, alignment
  rule, and **measurement class** — `measured` (from a topic) or `derived` (deterministic
  post-processing with a pinned rule). Derived fields must never be presented as sensor data.

## 3.1 Time

| Field | Type | Unit | Source / rule |
|---|---|---|---|
| `timestamp` | float | s (sim time) | stamp of the `/joint_states` message defining the frame (Stage 0 semantics) |
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
| `base_pose` | {`position`: float[3], `orientation`: float[4] (xyzw)} | m, – | `/body_pose` (frozen topic); equivalently the `base_link` entry of `/world/nao_world/pose/info`. Alignment: causal last-not-after, tolerance `0.02 s` (Stage 0 §6.4) | measured |
| `base_velocity` | float[3] | m/s | central finite difference of `base_pose.position` on the frame grid; one-sided at boundaries | derived |
| `base_angular_velocity` | float[3] | rad/s | quaternion finite difference of `base_pose.orientation` (pinned: `2 * log(q(t+1) * q(t)^{-1}) / dt`), expressed in the **world** frame | derived |

*Derived twist is the default contract. If verification gate V-3 (Appendix B) finds a native
ground-truth twist stream, the source may be upgraded by RFC revision only — silent substitution
is forbidden.*

## 3.4 CoM

Computed from full-body link poses + pinned inertial parameters (the model is pinned by hash, § 0.1,
so the parameters are frozen constants of this contract):

```math
c \;=\; \frac{1}{\sum_i m_i} \sum_{i} m_i \,\big(R_i\, \hat{c}_i + t_i\big)
```

where `i` runs over all 25 links of `nao_simple.sdf`, `(R_i, t_i)` is the world pose of link `i`
(source: `/world/nao_world/pose/info`, causal last-not-after, tolerance `0.02 s`), and
`(m_i, ĉ_i)` are the mass and CoM offset-in-link-frame pinned from the SDF.

| Field | Type | Unit | Source / rule | Class |
|---|---|---|---|---|
| `com_position` | float[3] | m | formula above | derived (from measured poses + pinned constants) |
| `com_velocity` | float[3] | m/s | central finite difference of `com_position` | derived |
| `com_acceleration` | float[3] | m/s² | second finite difference of `com_position` (pinned stencil) | derived |

## 3.5 Foot

**Foot frame declaration:** `nao_simple.sdf` has no sole link. The foot reference frame of this
contract is the **ankle link frame** (`l_ankle` / `r_ankle`) — the links that carry the contact
sensors. If a sole link is ever added to the model, this section requires an RFC revision (model
hash would change).

| Field | Type | Unit | Source / rule | Class |
|---|---|---|---|---|
| `left_foot_pose` | {`position`: float[3], `orientation`: float[4]} | m, – | `l_ankle` entry of `/world/nao_world/pose/info`, causal last-not-after ≤ `0.02 s` | measured |
| `right_foot_pose` | {`position`: float[3], `orientation`: float[4]} | m, – | `r_ankle` entry, same rule | measured |
| `left_foot_velocity` | float[3] | m/s | central finite difference of `left_foot_pose.position` | derived |
| `right_foot_velocity` | float[3] | m/s | central finite difference of `right_foot_pose.position` | derived |

## 3.6 Contact (wrench, not boolean)

Stage 0 recorded booleans only. Stage 1 records the wrench. Source: the raw bridged topics
`/nao/left_foot_contact_raw` and `/nao/right_foot_contact_raw`
(`ros_gz_interfaces/msg/Contacts`; gz `msgs.Contact` carries `repeated JointWrench` with
force/torque fields). The frozen `Bool` topics (`/left_foot/contact`, `/right_foot/contact`) are
**kept** so the boolean definition remains byte-identical to Stage 0 (cross-stage comparability).

| Field | Type | Unit | Source / rule | Class |
|---|---|---|---|---|
| `left_contact` | bool | – | `/left_foot/contact`, freshness last-not-after ≤ `0.05 s` (Stage 0 rule, unchanged) | measured |
| `right_contact` | bool | – | `/right_foot/contact`, same rule | measured |
| `left_contact_force` | float[3] ∪ null | N | sum of reported wrench force components for the left sensor's collisions, expressed in the **world** frame after the transform validated by gate V-1 | measured |
| `right_contact_force` | float[3] ∪ null | N | same, right sensor | measured |
| `left_contact_moment` | float[3] ∪ null | N·m | net moment about the **left foot (ankle) frame origin**, after the V-1-validated transform | measured |
| `right_contact_moment` | float[3] ∪ null | N·m | same, right foot frame origin | measured |
| `left_contact_wrench_valid` | bool | – | see semantics below | meta |
| `right_contact_wrench_valid` | bool | – | see semantics below | meta |

**Wrench validity semantics (pre-registered, fail-closed against fabrication):**

- `contact = false` and no `Contacts` message within the Stage 0 freshness window (`0.05 s`) →
  force = `[0,0,0]`, moment = `[0,0,0]`, `valid = true` (true flight; zero is the physical value).
- `contact = true` but no `Contacts` message within the window → force = `null`, moment = `null`,
  `valid = false` (**no interpolation, no hold, no zero-fill** — the frame is flagged, never
  fabricated).
- Wrenches are never averaged, extrapolated, or gap-filled across frames.

## 3.7 ZMP / CoP

Two deliberately independent quantities (this is a design decision, not redundancy):

- `cop_position` — **kinetics** (from measured contact wrenches): the point on the ground contact
  plane at which the net measured ground-reaction wrench produces zero horizontal moment.
  Single support: computed from that foot's measured force/moment. Double support: computed from
  the sum of both feet's wrenches about the world origin, then projected. The pinned formula:
  with net force `F = (F_x, F_y, F_z)` and net moment `τ` about the world origin,
  `cop = ( (−τ_y·F_z − ... )/... )` — the exact closed-form expression is fixed at implementation
  time **before the first collected episode** and recorded in the collector config hash (§ 3.10);
  once fixed it is immutable.
- `zmp_position` — **kinematics** (LIPM formula, model side):

```math
p_x \;=\; c_x \;-\; \frac{c_z - p_z^{floor}}{g}\,\ddot{c}_x
\qquad
p_y \;=\; c_y \;-\; \frac{c_z - p_z^{floor}}{g}\,\ddot{c}_y
```

with `g = 9.81 m/s²` (pinned), `p_z^{floor}` = world z of the contact plane (pinned constant from
`nao_world.sdf`), and `c`, `c̈` from § 3.4.

| Field | Type | Unit | Source / rule | Class |
|---|---|---|---|---|
| `cop_position` | float[2] ∪ null | m | measured-wrench formula above; `null` when total vertical force < `1.0 N` (flight — pre-registered threshold) | derived (from measured wrenches) |
| `zmp_position` | float[2] ∪ null | m | LIPM kinematics formula above; `null` only when required inputs are invalid | derived (from derived CoM) |

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
| `controller_command` | float[24] | rad | `/joint_command`, hold-last ≤ `0.05 s` (Stage 0 rule, unchanged) | measured |
| `gait_phase` | {`phase`: string, `fraction`: float ∈ [0,1)} | – | deterministic derivation from `(θ, t)` via the frozen ROS-free model `src/nao_gz_sim/scripts/nao_gait_phase.py` (classify + normalized phase time); no simulation feedback is involved, hence provenance-clean | derived (deterministic, replayable) |
| `desired_velocity` | float[2] | m/s | deterministic from θ per the frozen Stage 0 parameterization: `v_x = step_length × step_frequency`, `v_y = 0`; exact definitions pinned to the Stage 0 GaitGenerator semantics | derived (deterministic) |

## 3.10 Provenance (episode-level, mandatory, no exceptions)

Every episode record must carry:

| Field | Source / rule |
|---|---|
| `theta` | full parameter dict (verbatim from the frozen θ list entry) |
| `theta_id`, `theta_hash` | per-θ canonical JSON hash (Stage 0 method) |
| `theta_list_hash` | `844646f5…` — cross-checked fail-closed before every episode (inherited Stage 0 tamper check) |
| `git_commit` | HEAD of the collector at acquisition time |
| `config_hash` | SHA-256 of the collector configuration (includes the pinned CoP formula and all alignment constants) |
| `gazebo_seed` | the seed of the run (inherited from the Stage 0 per-episode `run_config` value for the same (θ, seed) pair) |
| `robot_model_version` | `nao_simple.sdf` sha256 `44ed8285…` (§ 0.1) |
| `world_model_version` | `nao_world.sdf` sha256 `383341d1…` (§ 0.1) |
| `gz_sim_version` | `8.15.0` (recorded, not pinned — environment fact) |
| `stage1_rfc_hash` | sha256 of this RFC at the revision under which the episode was collected |
| `timestamp` | wall-clock acquisition date |
| `ring` | sampling ring label (§ 4) |
| `frame_count`, `t_start`, `t_end` | episode extents |
| `episode_integrity_hash` | sha256 over the episode's canonical sample file (self-verification) |

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

## 5.2 Consecutivity and validity rules

- `(t, t+1)` pairs are valid **only** for consecutive `frame_id`s within the same episode.
  Cross-episode pairs are never formed; episode boundaries are hard boundaries.
- Clock gaps (dropped `/joint_states` messages, resets) must be flagged in a per-frame
  `clock_valid` mask; they are never silently interpolated.
- Measured quantities are never interpolated or gap-filled. Derived quantities use only the pinned
  finite-difference stencils of § 3, which skip invalid frames rather than bridging them.

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
converter changes are authorized by this RFC. Two sources are already on the wire but unverified
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
- **Implementation of the collector itself** — separate authorization required after this RFC is
  accepted.

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
| R6 | **No implementation** — specification only; collector explicitly out of scope and unauthorized | § 0, § 7 |
| R7 | **Stage 0 immutability respected** — no frozen artifact touched; new storage root `dataset/stage1/` | § 4.3, § 7 |

## 8.2 Dataset acceptance (deferred — criteria for the *future* Stage 1 dataset, satisfiable only by the separately authorized implementation)

| # | Criterion |
|---|---|
| D1 | All 96 pre-registered episodes acquired under gates V-1/V-2 passed; V-1 failure blocks collection (no partial pivot) |
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
| A6 | **contact wrench** | `/nao/left_foot_contact_raw`, `/nao/right_foot_contact_raw` | `ros_gz_interfaces/Contacts` (gz `msgs.Contact` with `repeated JointWrench`) | on the wire, bridged — **wrench population unverified → gate V-1** |
| A7 | **all 25 link world poses** (base pose cross-check, CoM, foot poses) | `/world/nao_world/pose/info` | gz `msgs.Pose_V` (SceneBroadcaster) | on the wire — **completeness/rate unverified → gate V-2** |
| A8 | gait_phase, desired_velocity | deterministic derivation (§ 3.9) | `nao_gait_phase.py` + pinned formulas | derived |
| A9 | joint/base/foot/CoM velocities & accelerations | deterministic finite differences (§ 3.2–3.5) | pinned stencils | derived |

# Appendix B — Verification gates (must pass before Stage 1 collection starts)

- **V-1 (blocking): contact wrench availability.** During a scripted probe episode, verify that
  the `JointWrench` fields of both `*_foot_contact_raw` topics are populated with physically
  plausible values (e.g., stance-foot vertical force > 0 while the robot is supported), that the
  frame convention of the wrench is established, and that flight periods are distinguishable per
  the § 3.6 semantics (event-only publication vs. periodic). If the physics engine does not
  populate the wrenches or flight is ambiguous, **Stage 1 collection is blocked until this RFC is
  revised. No force-free fallback is pre-authorized** (estimating forces from kinematics would
  defeat the purpose of § 1).
- **V-2 (blocking): pose-info completeness and rate.** Verify that `/world/nao_world/pose/info`
  enumerates all 25 links of `nao_simple.sdf` and arrives at a rate sufficient for the `0.02 s`
  causal last-not-after alignment (§ 3.3–3.5). Failure → RFC revision required (no silent
  widening of tolerances).
- **V-3 (non-blocking, informational):** presence of a native ground-truth twist stream; if
  absent, the derived finite-difference twist of § 3.3 is the contract (no revision needed).

# Appendix C — Storage layout (contract-level)

```text
dataset/stage1/
  stage1_theta_list.json      # verbatim copy of the frozen Stage 0 list
                              # + cross-checked theta_list_hash (fail-closed)
  ring0/ ring1/ ring2/ ring3/ # per-episode directories, Stage 0 conventions
  records/                    # *.record.json + *.metrics.json per episode
  corpus_index.json           # self-verifying (content_hash), atomic write
  corpus_index.meta.json      # volatile sidecar (BF-3 pattern)
```

- `dataset/stage0/**` is never written (§ 4.3).
- The per-frame aligned samples (`.jsonl`) are the contract artifact. Raw stream dumps and the
  git commit scope for `dataset/stage1/` are an **owner decision at implementation time**, not
  pre-declared here.

---

## Document status

REV1.0-DRAFT — submitted for owner authorization and Independent Auditor (TRAE + GLM-5.3) review.
**No implementation is authorized by this document.** Stage 1 instrumentation requires a separate
authorization after this RFC is accepted.
