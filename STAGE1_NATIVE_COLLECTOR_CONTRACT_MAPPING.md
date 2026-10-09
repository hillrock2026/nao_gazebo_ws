# STAGE1_NATIVE_COLLECTOR_CONTRACT_MAPPING

Phase 1 deliverable: **Contract → Implementation Matrix**. Every row pins the
frozen-contract basis, the planned implementation location, the test method, the
current status, and open issues. All field names / required-ness / status enums
come from the **frozen** RFC (`docs/STAGE1_DYNAMICS_DATASET_RFC.md` § 3.x /
Appendix D) and Contract (`docs/STAGE1_COLLECTOR_CONTRACT.md` § 3–10). Implementation
target: new file `tools/stage1_native_collector.py` (no frozen file edited).
Status legend: `PLANNED` / `IMPLEMENTED` / `PASS` / `FAIL` / `BLOCKED` /
`UNVERIFIED` / `NOT APPLICABLE`.

| # | Contract domain | Contract basis | Implementation location | Test method | Status | Open issues |
|---|---|---|---|---|---|---|
| 1 | **Episode identity** | RFC § 3.1 (`episode_id` inherits Stage 0 naming); contract § 8 (provenance) | `episode_id` CLI arg; propagated into frame records, provenance, status; refuse duplicate episode dirs | unit: id echo into every frame/provenance; duplicate-id refusal | PLANNED | Stage 0 uses `<θ-id>_s<seed>`; the exact θ-id string is not stored in the θ list → collector takes `--episode-id` explicitly |
| 2 | **Provenance** | RFC § 3.10 (single source of truth, 18 fields); Appendix D `provenance_fields`; contract § 8 | `build_provenance()`; `theta`/`theta_hash` via Stage 0 canonical method; `theta_list_hash`, model/world hashes, `git_commit`, `config_hash`, `stage1_rfc_hash`, `gz_sim_version`, `gazebo_seed`, `ring`, `frame_count`, `t_start/t_end`, `episode_integrity_hash` | unit: field-set == Appendix D names; real CLI: `--smoke` provenance check PASS | PLANNED | `git_commit` = `git rev-parse HEAD`; `cop_derivation_hash` stays `null` (allowed) |
| 3 | **Native Contacts** | RFC § 3.6.1 (lossless retention, gz native mandatory); contract § 3.4/§ 3.5/§ 6.1/§ 6.2 | gz-native subscribe `gz.msgs.Contacts`; `contact_record()` full-retention builder → `contacts_left`/`contacts_right` raw records | round-trip: every gz field (header/collision ids+names/position[]/normal[]/depth[]/wrench[] both body sides + force_offset) equals source; real smoke: topic non-empty | PLANNED | wrench `body_*_name` are empty at runtime (contract § 3.5) → foot side via collision name; native path mandatory |
| 4 | **Pose_V** | RFC § 3.3/§ 3.4/§ 3.5 (model-relative entries; composition); contract § 4.2–4.4 | gz subscribe `gz.msgs.Pose_V`; retain **all** entries (`name,id,position,orientation`) as `pose_info`; select 25 links + `nao` model by exact name for composition | round-trip: every entry retained; real smoke: 25 link names present | PLANNED | `*_visual` entries retained as raw but excluded from composition (contract § 4.1) |
| 5 | **Joint / state** | RFC § 3.2 (24 joints, canonical order); Appendix D `joint_states` | ROS subscribe `/joint_states`; retain `name,position,velocity`(+`effort`) verbatim | round-trip: names/positions equal; real CLI: raw `joint_states` PASS | PLANNED | measured 24 names (probe V1) |
| 6 | **Timestamp** | RFC § 3.1/§ 3.6.3; contract § 5.1–5.3 | frame clock = `/joint_states` header stamp (`sim_time_sec`); all source stamps preserved verbatim; `acquisition_wallclock_utc` separate | unit: source stamp preserved ≠ wallclock; per-frame association windows | PLANNED | never describe association as clock synchronization (§ 5.5 prohibition) |
| 7 | **Raw artifacts** | RFC § 3.6.1 + Appendix C; Contract § 6.3 | `raw_records.json` (array, categories `contacts_left/right`,`pose_info`,`joint_command` required; `joint_states`,`body_pose` optional) + `raw/<cat>.json`; values verbatim, no rounding/aggregation | round-trip + real CLI raw check PASS | PLANNED | order within a stream preserved (arrival order recorded) |
| 8 | **Frame records** | RFC § 3.0 (one per `/joint_states`, `frame_id` 0-based ordered) | `build_frames()`; `frames.json` (array) + `frames.jsonl` | unit: `frame_id` strictly consecutive; ordering = joint clock | PLANNED | `.jsonl` per RFC § 3.0; array per R15 `--frame` |
| 9 | **`frame_count`** | RFC Appendix D `frame_artifact.count_must_equal`; R15-C invariant | `frame_count = len(frames)` at write time (never forced > 0; empty episode legal) | real CLI: matrix (0,0)P (1,1)P (2,2)P (0,1)F | PLANNED | R15-C enforces unconditionally in the frozen checker |
| 10 | **Unknown / missing data** | RFC § 3.6.3/§ 3.6.4 (two-state, no zero-fill); § 3.3 miss flag | contact silence → `UNKNOWN_NO_MESSAGE` (no wrench record fabricated, no zero force, no flight); pose window miss → per-frame flag; Option-B wrench fields `null`; incomplete trajectory rows kept with validity status | unit: silence → no record + state=UNKNOWN_NO_MESSAGE; missing fields stay absent (no default) | PLANNED | wrench frame/reference `unknown` (CoP blocked) |
| 11 | **Derived fields (R11-A)** | RFC § 3.0/§ 3.3–3.7; contract § 4.7 | compute Option-A fields (composition, CoM from SDF inertials, finite differences, LIPM zmp), mark `derived_pending_validation`; Option-B `null`; measured/derived split per Appendix D | real CLI: frame emission + propagation PASS; composition self-check ≤ 1e-6 | PLANNED | `composition_status` must stay `UNVERIFIED` (never upgraded by the collector) |
| 12 | **Trajectory records** | RFC § 5.1/§ 5.2 (`(s_t,u_t,s_{t+1})`) | frames are the per-timestep records; explicit association rule documented; no fabricated control/next-state | unit: transition rule = consecutive frame_id; contact/pose validity carried per row | PLANNED | task scope ends at records, not LIPM/residual |
| 13 | **Failure handling** | Contract § 5.8-style integrity; task § 5.8 | staging → atomic `os.replace`; distinct `status.json` classes: `complete_validated` / `partial` / `collection_failed` / `write_failed` / `contract_rejected`; never silently overwrite | unit: each failure class; injected write error; duplicate episode refusal | PLANNED | status kept separate from normative provenance |
| 14 | **Validation** | Contract § 10; R15 CLI | invoke frozen `tools/stage1_contract_check.py --smoke` over emitted artifacts; record exit code + report | real CLI: valid → exit 0; each mutation → exit ≠ 0 | PLANNED | checker is called, never modified |
| 15 | **Output integrity** | BF-3 pattern (RFC § 4.3); contract § 8 | atomic tmp+replace; post-write hash; `episode_integrity_hash` = sha256 of canonical sample file | unit: truncation/corruption detected; re-read hash match | PLANNED | — |

## Normative field-set snapshot (from Appendix D, used as the implementation target)

- **Option-A (present, non-null, `derived_pending_validation`)**: `left_foot_pose`,
  `right_foot_pose`, `left_foot_velocity`, `right_foot_velocity`, `com_position`,
  `com_velocity`, `com_acceleration`, `zmp_position`.
- **Option-B (present, `null`)**: `left_contact_force`, `right_contact_force`,
  `left_contact_moment`, `right_contact_moment`, `cop_position`.
- **measured**: `joint_position`, `joint_velocity`, `base_pose`,
  `pose_source_stamp`, `left_contact`, `right_contact`, `imu_orientation`,
  `imu_linear_acceleration`, `imu_angular_velocity`, `controller_command`.
- **ordinary_derived**: `joint_acceleration`, `base_velocity`,
  `base_angular_velocity`, `gait_phase`, `desired_velocity`.
- **raw categories**: `contacts_left`, `contacts_right`, `pose_info`,
  `joint_command` (required) + `joint_states`, `body_pose` (optional).

## Deviations / recorded discrepancies (no contract edit)

1. `/joint_command` is `sensor_msgs/JointState` in the live producer, not
   `trajectory_msgs/JointTrajectory` (RFC App. A2 prose). Collector adapts;
   discrepancy reported to the auditor.
2. Collector-internal metadata (capture stats, per-stream arrival order, status
   class) is written to **non-normative** files (`collector_meta.json`,
   `status.json`) and labelled as implementation-added, never promoted to
   normative fields.
