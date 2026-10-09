# STAGE1_NATIVE_COLLECTOR_MINIMAL_IMPLEMENTATION_REPORT

## 1. Implementation Identity

| Item | Value |
|---|---|
| Task | `STAGE1_NATIVE_COLLECTOR_MINIMAL_IMPLEMENTATION_TASK` (minimal native collector) |
| Role | GLM-5.3 Implementation Agent |
| Date | 2026-10-09 |
| Prerequisite | R15 Decision A / PASS / ACCEPT (carried by the task directive — see § 2) |
| Deliverables | `tools/stage1_native_collector.py` (1266 lines); `tools/test_stage1_collector.py` (551 lines, 29 tests); `STAGE1_NATIVE_COLLECTOR_ARCHAEOLOGY_REPORT.md`; `STAGE1_NATIVE_COLLECTOR_CONTRACT_MAPPING.md`; real-Gazebo smoke evidence `dataset/stage1/smoke/stage1_smoke0_evidence.json` |
| Overall status | **IMPLEMENTATION COMPLETE for the authorized scope; ready for `STAGE1_NATIVE_COLLECTOR_INDEPENDENT_ADVERSARIAL_AUDIT`. NOT accepted, NOT Level-3-authorized.** |
| Real Gazebo smoke | **PASS** (checker exit 0; `complete_validated`) — reproducible; § 12 |

Every claim below carries a code location, the executed command, the real exit
code, the result, and an artifact/log path. Status tags:
`PASS`/`FAIL`/`BLOCKED`/`UNVERIFIED`/`NOT APPLICABLE`.

## 2. Repository and Commit Baseline

| Item | Value |
|---|---|
| Repo | `/home/liu/workspace/Nao_Program/nao_gazebo_ws` |
| Branch / HEAD at start | `main` / `bcbdcdcda13fbb42f08da051c37380d515fa7bd6` |
| `origin/main` | `6d00f90b7d80060f5b42b7d64499699ec3100911` — **16 commits ahead (R11–R15 + this work); nothing pushed** |
| Working tree at start | clean except pre-existing untracked `docs/task4_phase4b_evidence/` (untouched) |
| R15 object under audit | present locally: `b097ba6` (checker pin + tests), `bcbdcdc` (report) |

**R15 Decision A — `CONFIRMED with note` (`UNVERIFIED` on disk):** the audited
commits exist in this checkout. There is **no standalone R15 audit-verdict file
in the repo** — only the audit *prompt* (`prompt/STAGE1_REV15_INDEPENDENT_
ADVERSARIAL_AUDIT prompt.txt`); every `*AUDIT*` document is git-ignored by
design. The Decision A grant is taken from the task directive (same delivery
pattern as R14→R15). This is recorded, not assumed.

No push and no history rewrite was performed (task § 3.1/§ 8.1).

## 3. Contract and Freeze References

| Reference | sha256 (16) | Status |
|---|---|---|
| `docs/STAGE1_DYNAMICS_DATASET_RFC.md` | `f8662d1673f1a30e` | unchanged |
| `docs/STAGE1_COLLECTOR_CONTRACT.md` | `0384aad2d1f4db92` | unchanged |
| `tools/stage1_contract_check.py` (frozen R15 checker) | `0b29b26f21a4c06f` | unchanged |
| normative schema block pin (RFC App. D == Contract § 8.4) | `eb620b23…b25241`, version `stage1-normative-schema/1.0` | unchanged |
| `tools/stage1_probe.py` (frozen) | `9db101d94a6f66e1` | unchanged |
| Phase B0 freeze record § 3 anchors | 13/13 intact | PASS |

## 4. Existing Architecture Archaeology

`PASS` — full read-only archaeology in `STAGE1_NATIVE_COLLECTOR_ARCHAEOLOGY_REPORT.md` (13 required sections). Highlights: the runner is `tools/run_phase_a_episode.py` (staging → launch → atomic publish; FROZEN); the only existing live-capture code is the frozen probe; the gz-native contact topics `/nao/{left,right}_foot_contact_raw` and the gz-only `/world/nao_world/pose/info` are the canonical raw sources; `/joint_states` (24) is the frame clock. One recorded discrepancy: RFC App. A2 names `/joint_command` as `trajectory_msgs/JointTrajectory`, but the live producer publishes `sensor_msgs/JointState` — the collector adapts to the live type without editing the contract.

## 5. Contract-to-Implementation Matrix

`PASS` — `STAGE1_NATIVE_COLLECTOR_CONTRACT_MAPPING.md` (15 domains: episode identity, provenance, native contacts, pose_V, joint/state, timestamps, raw artifacts, frames, frame_count, unknown/missing, derived fields, trajectory, failure handling, validation, output integrity). Field names / required-ness / status enums are taken from the frozen Appendix D; implementation-added metadata is kept in **non-normative** files.

## 6. Modified Files and Rationale

| File | Change | Rationale |
|---|---|---|
| `tools/stage1_native_collector.py` | **new** | the collector (§ 7–§ 10) |
| `tools/test_stage1_collector.py` | **new** | unit + round-trip + real-CLI tests (§ 11) |
| `dataset/stage1/smoke/stage1_smoke0_evidence.json` | **new** | compact pinned smoke evidence (§ 12) |
| `STAGE1_NATIVE_COLLECTOR_ARCHAEOLOGY_REPORT.md` / `_CONTRACT_MAPPING.md` | **new** | Phase 0 / Phase 1 deliverables |
| `STAGE1_NATIVE_COLLECTOR_MINIMAL_IMPLEMENTATION_REPORT.md` | **new** | this report |

**No existing tracked file was modified** (`git diff` vs `bcbdcdc` is empty for
all tracked files). No frozen file touched.

## 7. Native Contacts Capture Path — `PASS`

`build_contacts_record` (`stage1_native_collector.py`) + gz-native subscribe
(`collect_live`). For every gz `msgs.Contacts` message it retains, without
reduction: the message `stamp_s` + verbatim `header`; per contact the verbatim
`header`, `collision1/2` names **plus** their ids/types, **all** `position[]/
normal[]/depth[]` points, the `world` entity, and **every** `JointWrench` with
both `body_1_wrench`/`body_2_wrench` (`force`/`torque`/`force_offset`).
`UNKNOWN_NO_MESSAGE` is a per-frame state when a contact stream is silent; no
zero-wrench, no fabricated flight/no-contact label, no interpolation/hold.

Evidence (real run): 910 left / 959 right native contact messages;
`collision1='nao::l_ankle::l_ankle_collision'`,
`collision2='ground_plane::link::collision'`; sample wrench
`force=[-31.14, -31.14, 48.62]` — in
`dataset/stage1/smoke/stage1_smoke0_evidence.json`.

## 8. Pose_V, Wrench and Timestamp Semantics — `PASS` (with declared limits)

- **Pose_V**: all entries retained verbatim (`name,id,position,orientation`).
  World poses are obtained **only** by composition
  `T_world_link = T_world_model · T_model_link` from the same message
  (`compose_pose`); link entries are never written as world poses.
  `composition_status` stays `UNVERIFIED`; the collector never upgrades it.
- **Wrench**: `frame_reference` stays `unknown`; Option-B frame fields
  (`left/right_contact_force/moment`, `cop_position`) are **present and null**;
  raw wrenches are retained verbatim with **no** interpretive transform.
- **Derived (R11-A)**: composed foot/CoM/zmp fields carry the Option-A value and
  are marked `derived_pending_validation` via the schema-driven manifest;
  `zmp_position` is the pinned LIPM kinematics (`g=9.81`, floor z=0).
- **Timestamps**: all source stamps preserved verbatim; the frame clock is
  `/joint_states`; `acquisition_wallclock_utc` is a separate field. Association
  windows: contact 0.05 s, pose 0.02 s (causal last-not-after). No claim of
  clock synchronization / universal clock identity is made anywhere.
- **Declared limitation**: `/left_foot/contact` is `std_msgs/Bool` (no header) →
  the boolean is associated by **receipt order only** (documented in
  `collector_meta.json`); it is not used as contact-wrench evidence, and the
  authoritative silence semantics come from the native stream.

## 9. Episode Provenance and Artifact Layout — `PASS`

`episode_provenance.json` emits all **18** RFC § 3.10 fields: `theta`,
`theta_id`, `theta_hash` (Stage 0 canonical method, reproduced exactly),
`theta_list_hash` (fail-closed list-membership check), `git_commit`,
`config_hash` (collector config: subscriptions/windows/association/
serialization, decoupled from any CoP formula), `cop_derivation_hash` = null,
`gazebo_seed`, `robot_model_version`/`world_model_version` (SDF/world sha256),
`gz_sim_version`, `stage1_rfc_hash`, `acquisition_wallclock_utc`, `ring`,
`frame_count`, `t_start`, `t_end`, `episode_integrity_hash`
(= sha256 of `frames.jsonl`). `status_manifest.json` is generated from the
frozen schema field groups. Layout:
`dataset/stage1/episodes/<episode_id>/{raw_records.json, raw/<cat>.json,
frames.json, frames.jsonl, status_manifest.json, episode_provenance.json,
collector_meta.json (non-normative), status.json (non-normative),
smoke_report.json}`.

**Declared deviation**: this per-episode layout is not an instance of RFC
Appendix C's corpus layout (`raw/ derived/ ringN/ records/ provenance/
corpus_index.json`) — the corpus layout belongs to the (unauthorized) Level-3
acquisition stage. Recorded for the auditor.

## 10. Trajectory Record Construction — `PASS`

One frame per `/joint_states` message, `frame_id` 0-based and strictly
consecutive (RFC § 3.0); the per-timestep tuple `(s_t, u_t, s_{t+1})` is the
consecutive frame sequence with `u_t = controller_command` recorded verbatim
from `/joint_command` (never an actuator output). No control input is invented
(absent ⇒ null); a frame whose pose window misses keeps Option-A fields null
(derived from the unverified composition is never fabricated); rows are never
dropped to manufacture complete transitions. `frame_count == len(frames)` is
invariant (checked by the frozen R15-C).

## 11. Raw Round-trip Test Results — `PASS`

| Command | Exit | Result |
|---|---|---|
| `PYTHONDONTWRITEBYTECODE=1 python3 tools/test_stage1_collector.py` | 0 | Ran 29 tests — OK |
| `/usr/bin/python3 tools/stage1_native_collector.py verify --episode-dir dataset/stage1/episodes/stage1_smoke0` | 0 | `frame_count_consistent=true`, `checker_exit_code=0`, `status=PASS` |

Round-trip chain exercised: source-like message → builder → artifact on disk →
**strict** read-back (`checker._strict_json_loads`) → field-by-field equality
(contacts: all positions/normals/depths/wrenches/headers; pose: all entries;
non-default stamps) → frozen contract validation. Missing-field tests prove no
default is invented (an empty `depth`/`position` stays empty and the checker
**rejects** it). Fixtures are synthetic and labelled; they prove local logic
only — the real path is proven separately by the Gazebo smoke (§ 12).

## 12. Real CLI and Gazebo Smoke Results — `PASS`

Environment: Gazebo Sim 8.15.0, ROS 2 Jazzy, system python 3.12.3; launch
`ros2 launch nao_gz_sim nao_sim.launch.py` headless (`gui:=false`), θ-derived
runtime args, `ROS_DOMAIN_ID=98`, `GZ_PARTITION=stage1collector3`, PATH
sanitized (anaconda/condabin removed).

| Attempt | Command / condition | Exit | Result |
|---|---|---|---|
| 1 | collect, unsanitized env | 1 | `partial` — producer **died**: anaconda python 3.14 picked up (`rclpy._rclpy_pybind11` mismatch) → 0 contacts/`joint_command` |
| 2 | same, stale domain | 1 | same producer death (launcher edit mis-applied) |
| 3 | sanitized env | 1 | `write_failed` — real bug: gz `Contact.world` is an `Entity`, not JSON-serializable (**fixed**) |
| 4 | overwrite existing dir | 1 | `ENOTEMPTY` on `rmtree`+replace (**fixed**: runner-style `_publish` backup) |
| 5 | **final** | **0** | **`complete_validated`, checker exit 0** |

Final run (real native data): frames 1233; raw 5824 records
(`contacts_left` 910, `contacts_right` 959, `pose_info` 791, `joint_command`
1204, `joint_states` 1233, `body_pose` 727); imu 1204; bool contacts 1150/1150;
t 7.072→19.392 s; `episode_integrity_hash` `7117a49b…`.
`verify` → exit 0. Commands and results in
`dataset/stage1/smoke/stage1_smoke0_evidence.json`. The three failure attempts
are reported deliberately: they are direct evidence that collection errors are
**not** converted into success (the failure classes `partial`/`write_failed`
were emitted).

Full artifacts (~50 MB) are kept **local only** (not committed); the compact
evidence file is committed.

## 13. R11–R15 and Project Regression Results — `PASS`

| Command | Exit | Result |
|---|---|---|
| `PYTHONDONTWRITEBYTECODE=1 python3 tools/test_stage1_r11.py` | 0 | Ran 15 — OK |
| `... r12.py` | 0 | Ran 40 — OK |
| `... r13.py` | 0 | Ran 49 — OK |
| `... r14.py` | 0 | Ran 32 — OK |
| `... r15.py` | 0 | Ran 51 — OK |
| `... test_stage1_collector.py` | 0 | Ran 29 — OK |

187 historical + 29 new = **216**, all exit 0, no skips, no hidden failures.

## 14. Frozen File Integrity and Git Status — `PASS`

- RFC / Contract / checker / probe hashes unchanged (§ 3); 13/13 Phase B0 freeze
  anchors intact; `run_phase_a_episode.py`, `stage1_probe.py`, `nao_simple.sdf`,
  `nao_world.sdf` byte-identical to `88aa3fa` (`git diff` empty).
- `git diff` vs `bcbdcdc`: **no tracked file modified**; working tree clean
  except new untracked files (`dataset/stage1/episodes/` = local evidence,
  `docs/task4_phase4b_evidence/` = pre-existing, untouched).
- Commits (LOCAL, **not pushed**): `b10300c` (implementation + Phase 0/1
  reports), parent `bcbdcdc`; this report commit follows.

## 15. Known Limitations, Unverified Items and Handover

1. **Not accepted** — needs `STAGE1_NATIVE_COLLECTOR_INDEPENDENT_ADVERSARIAL_AUDIT`.
2. **R15 audit verdict** has no on-disk file (git-ignored); grant carried by the directive.
3. **Appendix C corpus layout** (ring dirs, `corpus_index.json`) is deferred to Level 3 (unauthorized); per-episode layout used instead.
4. **`/joint_command` type** differs from RFC App. A2 prose (`JointState` vs `JointTrajectory`); collector adapts, contract unedited.
5. **Contact boolean** (`std_msgs/Bool`) has no source stamp → receipt-order association only.
6. **Composition `UNVERIFIED`**; composed fields `derived_pending_validation`; the collector computes them for implementation consistency only and never upgrades the status.
7. **Wrench frame/reference `unknown`** → CoP/moment Option-B nulls.
8. **`gait_phase` epoch** is pinned as `first_frame_sim_time + theta.start_delay_sec` (the producer's `ready_stamp`-based epoch is not observable at the collector); documented in `collector_meta.json`.
9. **`base_angular_velocity`** uses the central quaternion-difference with rotation-vector/dt (`ordinary_derived`, not smoke-validated); boundaries are null (not one-sided).
10. **Real smoke** was a single short in-gait window (not a pilot, not a sweep); full artifacts are local-only.
11. Unverified by this task: physical correctness of composition/CoM/zmp values; Level-3 dataset acceptance — all explicitly out of scope.
