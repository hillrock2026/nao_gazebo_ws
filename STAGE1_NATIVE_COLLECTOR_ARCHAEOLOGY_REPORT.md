# STAGE1_NATIVE_COLLECTOR_ARCHAEOLOGY_REPORT

Phase 0 deliverable for `STAGE1_NATIVE_COLLECTOR_MINIMAL_IMPLEMENTATION_TASK`.
**Read-only** archaeology (no repository file was modified to produce this
report; git status was clean apart from one pre-existing untracked dir).
Every interface below is confirmed from **code and real-run evidence**, not
from file names (task § 3.2). Status tags: `CONFIRMED` (evidence cited) /
`BLOCKED` / `UNVERIFIED`.

---

## 1. Actual repository and Git baseline

| Item | Value |
|---|---|
| Repo path | `/home/liu/workspace/Nao_Program/nao_gazebo_ws` |
| Branch | `main` |
| HEAD | `bcbdcdcda13fbb42f08da051c37380d515fa7bd6` (`stage1: issue R15 minimal hardening report (records b097ba6)`) |
| Parent | `b097ba6` (R15 implementation: checker + tests) |
| origin | `git@github.com:hillrock2026/nao_gazebo_ws.git` |
| `origin/main` | `6d00f90b7d80060f5b42b7d64499699ec3100911` |
| Unpushed local commits | **16** (`045a21f` stage0 freeze → `bcbdcdc`); RF/R11–R15 all local-only |
| Working tree | clean; one pre-existing untracked dir `docs/task4_phase4b_evidence/` (existed before R15; **must not be deleted/overwritten**) |
| Stage 1 spec files present | RFC `docs/STAGE1_DYNAMICS_DATASET_RFC.md`; Contract `docs/STAGE1_COLLECTOR_CONTRACT.md`; checker `tools/stage1_contract_check.py`; tests `tools/test_stage1_r11..r15.py`; report `STAGE1_REV15_MINIMAL_HARDENING_REPORT.md` (all at the paths named in the task) |

Stage 1 artifact hashes (sha256, first 16 hex — recomputed this session):

| File | sha256 (16) |
|---|---|
| `docs/STAGE1_DYNAMICS_DATASET_RFC.md` | `f8662d1673f1a30e` |
| `docs/STAGE1_COLLECTOR_CONTRACT.md` | `0384aad2d1f4db92` |
| `tools/stage1_contract_check.py` | `0b29b26f21a4c06f` |
| `tools/stage1_probe.py` (frozen) | `9db101d94a6f66e1` |
| `tools/test_stage1_r11.py` | `238c9108ff2276bd` |
| `tools/test_stage1_r12.py` | `a7a7d7f2262893f0` |
| `tools/test_stage1_r13.py` | `e6c0c5ccdce5deae` |
| `tools/test_stage1_r14.py` | `1136fd4f08e5286b` |
| `tools/test_stage1_r15.py` | `2be45674d3465b05` |

Normative schema block pin (RFC Appendix D == Contract § 8.4, byte span):
sha256 `eb620b233b3a4499cceaf23a1c1054a7c2693415e018874501b773d713b25241`,
version `stage1-normative-schema/1.0` — `CONFIRMED` (checker loads it with the
R15-A pin; `--info` mode reports `load_errors: []`).

Phase B0 freeze record: `docs/task_p0a_gait_search/phase_b/Phase_B0_Freeze_Record.md`
§ 3 (13 protected SHA-256 anchors) + § 4 (protected episode artifacts).
`CONFIRMED` — 13/13 anchors match the live files (verified in the R15 round and
re-verified by the R15 test `test_frozen_files_untouched` vs `88aa3fa`).

### 1.1 R15 Decision A status (precondition) — `CONFIRMED with note`

The R15 implementation and report commits **exist in this checkout**
(`b097ba6`, `bcbdcdc`), i.e. the object that the independent audit reviewed is
present. There is **no standalone audit-result file on disk**: the directory
contains the audit *prompt* `prompt/STAGE1_REV15_INDEPENDENT_ADVERSARIAL_AUDIT
prompt.txt`, and every `*AUDIT*`/`*REVIEW*` document is git-ignored by design
(`.gitignore`), so the audit verdict is delivered out-of-band. The **Decision A
/ PASS / ACCEPT grant is carried by this task directive** (identical delivery
pattern to R14→R15). No on-disk R15 audit verdict file is claimed here.

**Do not assume `origin` contains R15** — it does not (`origin/main` = `6d00f90`).
Push remains the owner's decision; this task performs no push.

---

## 2. Existing Runner execution flow (`CONFIRMED`)

`tools/run_phase_a_episode.py` (FROZEN — must not be modified), `run_episode()`:

1. `resolve_config(overrides)` → the 27-key frozen gait config; sets
   `duration_sec`.
2. Computes `record = start_delay + duration + stop_ramp + hold_final + 1.0`.
3. Creates a **staging** dir `<out_root>.staging.<pid>/` with `dataset/`,
   `trace/`, and writes `run_config.json`, `launch_cmd.txt`,
   `gz_sim_cmd.expected.txt`.
4. `env['GZ_PARTITION'] = R.partition_id('phase_a', episode_id)`,
   `env['ROS_DOMAIN_ID'] = str(domain_id)`; strips conda from PATH.
5. `ros2 launch nao_gz_sim nao_sim.launch.py <args>` in a new session; waits for
   `collector_summary.json` (written by the phase-4d collector node) or a timeout.
6. Reads the **actual** gz sim argv from `/proc` into `gz_sim_cmd.actual.txt`
   (never the expected cmd) → physics seed is *proven*, not assumed.
7. `EI.evaluate(...)` (integrity) → `verdict`; `_joint_order()` from
   `nao_trajectory_schema.JOINT_ORDER` (24 names).
8. Writes `provenance.json`, `manifest.json`, `episode_integrity.json` into
   staging.
9. `_publish(staging, out_root)` — **atomic**: `os.replace(staging, out_root)`
   (with a `.replaced.<pid>` backup if the target already exists).

Episode layout produced by the runner (one dir per episode):
`dataset/` (`manifest.json`, `samples.jsonl`), `trace/`,
`collector_summary.json`, `provenance.json`, `manifest.json`,
`episode_integrity.json`, `run_config.json`, `launch_cmd.txt`,
`gz_sim_cmd.{expected,actual}.txt`, `launch.log`.

**Collector boundary:** the collector must **not** become a second runner
(contract § 9 `NO`). It attaches as a passive subscriber to a running
simulation, exactly as `tools/stage1_probe.py` does today (its documented
launch uses *runtime args only, no file edits*).

---

## 3. Reusable data sources and writers (`CONFIRMED`)

| Reusable asset | Location | Reuse mode |
|---|---|---|
| gz-transport + rclpy subscription session (guarded callbacks, `LockedRecords`, background spin) | `tools/stage1_probe.py` (`ProbeSession`, `guarded`, `LockedRecords`) | **import** (probe is frozen → read-only import, no edits) |
| gz message classes + transport loader | `tools/stage1_probe.py::gz_packages()` (`gz.msgs10/11`, `gz.transport13`) | import |
| Atomic JSON writer (`tmp` + `os.replace`, `allow_nan=False`) | `tools/stage1_probe.py::atomic_write_json`; `run_phase_a_episode.py::_publish` | import / mirror |
| Canonical bytes + `theta_hash` method | `tools/run_stage0_dataset.py::canonical_bytes` = `json.dumps(obj, indent=2, sort_keys=True, allow_nan=False)`; `sha256` over it | **reproduced**: anchor θ → `28e795b2…94998` = frozen `anchor_theta_hash` |
| θ population + `theta_list_hash` | `dataset/stage0/stage0_theta_list.json` (`theta_list_hash=844646f5…`, 32 θ, seeds [0,1,2], 27-key θ) | read-only reference |
| Contact raw + pose-in-memory shapes | `dataset/stage1/probe/V{1,2}_report_REV13.json` (14 tracked files) | evidence of real field population |

---

## 4. Native Gazebo data read interface (`CONFIRMED`)

From `src/nao_gz_sim/launch/nao_sim.launch.py` + `nao_simple.sdf` +
`tools/stage1_probe.py` + probe evidence:

| Stream | Topic | Transport | Type | Rate (measured) |
|---|---|---|---|---|
| Left foot contact raw | `/nao/left_foot_contact_raw` | **gz native** (canonical) + ROS bridge (same name, optional/lossy) | gz `msgs.Contacts` | 136.05 Hz effective; 1 kHz burst (walk) |
| Right foot contact raw | `/nao/right_foot_contact_raw` | gz native + ROS bridge | gz `msgs.Contacts` | 128.90 Hz; 1 kHz burst |
| Pose_V | `/world/nao_world/pose/info` | **gz only** (SceneBroadcaster) | gz `msgs.Pose_V` | 81.33 Hz |
| Joint clock | `/joint_states` | ROS | `sensor_msgs/JointState` | **100.0 Hz exact** |
| Base pose | `/body_pose` | ROS | `geometry_msgs/PoseStamped` | 82.05 Hz |
| Command | `/joint_command` | ROS | **`sensor_msgs/JointState`** (24 `position`) — **doc says `trajectory_msgs/JointTrajectory`** | 100 Hz |
| IMU | `/imu/data` | ROS | `sensor_msgs/Imu` | (Stage 0) |
| Contact bools | `/left_foot/contact`, `/right_foot/contact` | ROS | `std_msgs/Bool` | (Stage 0) |

Contact sensor source: `nao_simple.sdf` `<sensor type='contact'>` on
`l_ankle_collision` / `r_ankle_collision` → `<topic>/nao/{left,right}_foot_contact_raw`.
Native gz must be used (contract § 6.1, RFC § 3.6.1/R1): the ROS bridge loses
per-contact `header`, `world`, and `Wrench.force_offset` (probe
`bridge_field_loss`, sha-pinned evidence). **`CONFIRMED`.**

**Discrepancy to record (task § 2.1):** RFC Appendix A2 declares
`/joint_command` as `trajectory_msgs/JointTrajectory (reduced)`; the live
producer (`src/nao_gz_sim/scripts/phase4b_producer.py`, `publish_target`)
publishes `sensor_msgs/JointState` (`.name = JOINT_ORDER`, `.position = 24
floats`, empty velocity/effort). The collector captures what is actually
published and maps `.position` → `controller_command` float[24] (RFC § 3.9
defines `controller_command` type-agnostically as "the actual command/reference
published on `/joint_command`"). **RFC/Contract are NOT edited**; the
discrepancy is recorded as an open item for the auditor.

---

## 5. Raw message fields, timestamps, coordinates (`CONFIRMED`)

gz schema (from probe `message_schemas`, `V1_report_REV13.json`):

- `gz.msgs.Contacts`: `header{stamp{sec,nsec}}`, `contact` = repeated `Contact`.
- `gz.msgs.Contact`: `header{stamp{sec,nsec}}`, `collision1/collision2` =
  `Entity{header,id(uint64),name(string),type(enum)}`, `position`/`normal` =
  repeated `Vector3d{x,y,z}`, `depth` = repeated `double`, `wrench` = repeated
  `JointWrench`, `world` (enum), plus `Frame*` fields.
- `gz.msgs.JointWrench`: `body_1_id(uint32)`, `body_1_name(string)`,
  `body_1_wrench`, and `body_2_*` — each `Wrench{force,torque,force_offset:
  Vector3d, header}`. Measured: `body_*_name` are **empty strings** (contract
  § 3.5 → foot-side identification must use the *collision* entity names).
- `gz.msgs.Pose_V`: `header`, `pose` = repeated
  `Pose{name,id,position{x,y,z},orientation{x,y,z,w}}`. Measured ~54 entries
  (model `nao` + 25 links + `*_visual` + `world` + `ground_plane`); the 25
  non-visual links appear by exact bare name; link entries are
  **model-relative** (V-2: `base_link` identity 1.0), the `nao` model entry is
  the world anchor (cross-checked vs `/body_pose`, exact-stamp).

Timestamps: all streams carry **simulation** header stamps (sec/nsec); single
frame clock = `/joint_states` stub; association windows contact 0.05 s /
pose 0.02 s causal last-not-after (contract § 5). `acquisition_wallclock_utc`
is a separate time domain. `CONFIRMED`.

Coordinate semantics: `T_world_link = T_world_model · T_model_link`, Hamilton
quaternion product, `[x,y,z,w]` order (contract § 4.3); `composition_status =
UNVERIFIED` (no independent non-identity-link source exists → composed fields
stay `derived_pending_validation`). Wrench `frame_reference = unknown` → CoP
blocked (Option B nulls). `CONFIRMED`.

---

## 6. Existing episode / provenance format (`CONFIRMED`)

- Stage 0 episode dirs: `dataset/stage0/ring{0..3}/<episode_id>/` with
  `manifest.json`, `provenance.json` (Phase A provenance: `episode_id`, `seed`
  {requested,actual_gz_sim,partition,ros_domain_id,verified},
  `code_revision.source_sha256`, `gazebo.command_actual`), `run_config.json`
  (resolved 27-key θ), `episode_integrity.json`.
- Naming: `episode_id` = θ-id + seed (e.g. `r0_anchor_s0`); record files
  `dataset/stage0/records/<episode_id>.record.json` / `.metrics.json`.
- `theta_hash` method reproduced exactly (anchor `28e795b2…94998`).
- Stage 1 storage layout is specified at contract level in **RFC Appendix C**
  (`dataset/stage1/{probe,raw,derived,ring0..3,records,provenance,corpus_index.json}`)
  — **no Stage 1 episode directory exists yet** (only `dataset/stage1/probe/`).

---

## 7. R15 checker invocation (`CONFIRMED`)

The frozen acceptance boundary (must be **called, never modified**):

```
python3 tools/stage1_contract_check.py --smoke \
  --manifest <status_manifest.json> --provenance <episode_provenance.json> \
  --frame <frames_array.json> --raw <raw_records_array.json> \
  [--rfc <path>] [--contract <path>]
```

Chain: (R14) normative schema load **+ R15-A version/pin authorization** → RFC
∁ Contract bindings → raw-category binding → provenance completeness+type/range
→ status-manifest allowlists → frame array + **unconditional
`frame_count == len(frames)`** → raw nested structure. Any failure → `ok=false`,
exit ≠ 0. `--smoke` is the only enforcement entry point; default mode is
informational.

Artifact shapes the collector must emit (derived from the pinned schema, read
this session):

- **manifest**: `{composition_status:"UNVERIFIED", wrench_frame_status:"UNKNOWN",
  field_status:{<all 28 schema fields>: <status>}}` — measured→`measured`,
  ordinary_derived→`derived`, option_a→`derived_pending_validation`,
  option_b→`null_blocked`.
- **provenance**: the 18 RFC § 3.10 fields (`cop_derivation_hash` may be null).
- **frame**: **array** of frame records; each must carry **all 8 Option-A
  fields present/non-null with the declared shape** (`pose{position[3],
  orientation[4]}`, vec3, zmp vec2) and **all 5 Option-B fields present and
  null**; array length == `frame_count`.
- **raw**: array of `{stream, stamp_s, …}` records; `contacts_left/right`,
  `pose_info`, `joint_command` **required**; `joint_states`/`body_pose` optional;
  nested required keys per Appendix D (contact `header`, `collision1/2` as
  non-empty strings, `position/normal` non-empty 3-vec lists, `depth` finite
  list, `wrenches[]` with both body sides + `force/torque/force_offset`;
  pose entries `name/position/orientation`; 24-vector command). Unknown native
  keys are preserved and never rejected.

---

## 8. Current implementation gaps

1. **No collector exists.** Nothing subscribes to the raw streams per RFC
   § 3.6.1 and no code writes Stage 1 raw/frame/manifest/provenance artifacts.
2. **Probe builders are compact, not lossless.** `gz_contact_record` /
   `gz_pose_record` in the probe keep counts and first-values (`first_position`,
   `n_positions`, `tracked`), not the full per-contact-point arrays / all
   entries the contract mandates. A **new full-retention builder** is required
   (reusing the probe's transport, not its compact shapes).
3. **Derived fields are not computed anywhere.** `left/right_foot_pose`,
   `left/right_foot_velocity`, `com_position/velocity/acceleration`,
   `zmp_position` (Option A) must be computed to satisfy the smoke frame
   contract; CoM needs the SDF inertial constants (present: 25 `<inertial>`
   blocks with `<mass>` + `<inertial><pose>`).
4. **No frame/manifest/provenance writer** for Stage 1, no episode-status
   classification (complete / partial / failed / rejected).
5. **Doc↔code discrepancy** on `/joint_command` message type (§ 4).
6. **`dynamic_pose/info`** is an alternative parent-relative pose stream
   (contract § 4.5) — not needed; `pose/info` alone suffices.

---

## 9. Minimal implementation plan

Build **one new module** `tools/stage1_native_collector.py` (plus tests). It is
a **passive subscriber** (probe-pattern), never a runner:

- Subscribe (gz native) `/nao/{left,right}_foot_contact_raw`
  (`gz.msgs.Contacts`) and `/world/nao_world/pose/info` (`gz.msgs.Pose_V`);
  (ROS) `/joint_states`, `/joint_command`, `/body_pose`, `/imu/data`,
  `/left_foot/contact`, `/right_foot/contact`.
- Retain **lossless** raw records (Appendix D shapes) into
  `raw_records.json` (+ per-category files under `raw/`); no field dropped /
  rounded / aggregated.
- Build **frames** (one per `/joint_states`), computing the RFC-pinned derived
  fields (composition, CoM from SDF constants, finite differences, LIPM zmp,
  gait_phase via `nao_gait_phase.py`); Option-B wrench fields null; statuses
  per R11-A. Write `frames.json` (array, for `--frame`) **and** `frames.jsonl`
  (RFC § 3.0 layout).
- Silence → `UNKNOWN_NO_MESSAGE` (never zero wrench / flight); pose miss flag
  per frame; no interpolation/hold/zero-fill.
- Emit `status_manifest.json` (schema-driven) and `episode_provenance.json`
  (18 RFC § 3.10 fields; `theta`/`theta_hash`/`theta_list_hash` via the Stage 0
  method; model/world hashes; `stage1_rfc_hash`; seed; ring).
- **Atomic** staging → `os.replace`; never silently overwrite; classify the
  outcome (complete+validated / partial / failed / write-failure /
  contract-rejected) in an episode `status.json` **kept separate from the
  normative provenance**.
- Invoke the **frozen R15 checker** `--smoke` over the emitted artifacts and
  record its exit code; a rejection is a distinct, recorded status (never
  "success").
- A `--verify` round-trip mode: re-read artifacts **strictly** and compare
  saved values to the in-memory source (defined conversion rules only).

---

## 10. Files to be modified / added

| File | Action |
|---|---|
| `tools/stage1_native_collector.py` | **add** (core collector) |
| `tools/test_stage1_collector.py` | **add** (unit + round-trip + real-CLI tests) |
| `STAGE1_NATIVE_COLLECTOR_ARCHAEOLOGY_REPORT.md` | **add** (this report) |
| `STAGE1_NATIVE_COLLECTOR_CONTRACT_MAPPING.md` | **add** (Phase 1) |
| `STAGE1_NATIVE_COLLECTOR_MINIMAL_IMPLEMENTATION_REPORT.md` | **add** (Phase 5) |
| Stage 1 smoke artifacts under `dataset/stage1/` | **add** (evidence) |

No existing tracked file is edited by the plan.

## 11. Frozen files that must NOT be modified

`docs/STAGE1_DYNAMICS_DATASET_RFC.md`, `docs/STAGE1_COLLECTOR_CONTRACT.md`,
`tools/stage1_contract_check.py` (+ its R15 pin/strict-JSON/invariant),
`tools/stage1_probe.py`, `tools/run_phase_a_episode.py`,
`tools/run_p0a_gait_candidate.py`, `tools/nao_episode_integrity.py`,
`tools/validate_task4_phase4d_dataset.py`, `src/nao_gz_sim/**` (controller, gait
producer, launch, converters), `src/nao/nao_description/urdf/nao_simple.sdf`,
`src/nao_gz_sim/worlds/nao_world.sdf`, all `dataset/stage0/**` and the 13
freeze anchors + protected episode artifacts, and the pre-existing untracked
`docs/task4_phase4b_evidence/`.

## 12. Environment conditions for a real Gazebo smoke

`CONFIRMED available`: Gazebo Sim **8.15.0** (matches RFC pin), ROS 2 **Jazzy**,
system python **3.12.3** with `rclpy` after `source /opt/ros/jazzy/setup.bash`,
`gz.transport13` + `gz.msgs10` importable on `/usr/bin/python3`, workspace
already built (`install/setup.bash` exists), launch file
`src/nao_gz_sim/launch/nao_sim.launch.py` present. **No simulation is running
now** (`gz topic -l` empty). A real smoke = launch the sim headless with the
documented runtime-arg command (probe docstring) + run the collector against it
(`ROS_DOMAIN_ID`/`GZ_PARTITION` aligned) + run the R15 checker on the artifacts.
This is one short in-gait window — **not** a pilot, not a parameter sweep.

## 13. Assumptions currently unverifiable / blockers

- **R15 audit verdict file** is not on disk (git-ignored by design); the
  Decision A grant is taken from this task directive only (§ 1.1).
- Real Gazebo smoke depends on the machine being able to launch gz sim +
  DDS discovery within the run; if launch/RTF/topics fail, the smoke is
  recorded `BLOCKED`/`UNVERIFIED` with the missing conditions — **no fabricated
  Gazebo evidence** (synthetic fixtures are labelled as fixtures only).
- `/joint_command` type discrepancy (§ 4) is recorded; the collector adapts to
  the live type without touching the contract.
- The exact numeric CoM/derived values depend on the SDF inertial parse; a
  parse/hash guard will pin the SDF source sha256 into provenance.
