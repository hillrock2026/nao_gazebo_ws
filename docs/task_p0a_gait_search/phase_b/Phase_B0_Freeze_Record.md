# Phase B-0 — Baseline Freeze Record

- Document: `docs/task_p0a_gait_search/phase_b/Phase_B0_Freeze_Record.md`
- Stage: **Phase B-0 (Freeze only)**
- Status: **`PHASE_B_BASELINE_FREEZE`**
- Date: 2026-09-30 (Asia/Shanghai)
- Author: Implementation Agent (Qoder + DeepSeek = one agent)
- Purpose: pin the exact Phase A baseline that Phase B must preserve, with hashes, so any
  future drift is detectable and the independent audit is reproducible.

---

## 1. Repository state

```text
workspace            /home/liu/workspace/Nao_Program/nao_gazebo_ws
git                  NOT A GIT REPOSITORY  (no .git; repo state is pinned by SHA-256 below)
os                   Ubuntu 24.04.5 LTS
kernel               7.0.0-31-generic
ros_distro           jazzy  (/opt/ros/jazzy)
gz sim               Gazebo Sim, version 8.15.0
nproc                6
colcon packages       nao_description, nao_gz_sim, nao_meshes, nao_state_publisher
archived-out evidence /home/liu/workspace/Nao_Program/backup  (moved out before Phase A;
                      NOT restored, NOT modified)
```

> Because there is no git, **the SHA-256 tables in §3 and §4 are the freeze**. Any change
> to a listed file is a baseline violation and must be justified in the Phase B
> Implementation Report.

Note on environment drift: the interactive shell reports `python3 = 3.14.6`; the Phase A
ROS runtime used `/usr/bin/python3 = 3.12.3` (recorded in `closure_01/provenance.json`).
All Phase A runs used 3.12.3.

---

## 2. Frozen components (identity + responsibility)

| Component | Path | Role | Phase A decision |
|---|---|---|---|
| Runner / orchestrator | `tools/run_phase_a_episode.py` | prepare/launch/monitor/stop/cleanup/provenance/+episode atomic publish | baseline |
| Runner helpers | `tools/run_p0a_gait_candidate.py` | LAUNCH_BASE, param_args, build_gz_sim_command, reap, residual, partition, resolve_config | baseline |
| Integrity checker / (evaluator) | `tools/nao_episode_integrity.py` | `episode_integrity` + `gait_outcome` + duration/coverage | baseline (to be split in B3/B4) |
| Golden validator | `tools/validate_task4_phase4d_dataset.py` | **independent** hand-derived schema v1.0.1 check | frozen |
| Gait producer | `src/nao_gz_sim/scripts/phase4b_producer.py` | B-only gait command producer (**gait math**) | FROZEN |
| Gait phase model | `src/nao_gz_sim/scripts/nao_gait_phase.py` | pure phase module | FROZEN |
| Trace recorder | `src/nao_gz_sim/scripts/phase4b_trace.py` | raw per-topic JSONL + `phase4b_summary.json` | baseline |
| Collector (live) | `src/nao_gz_sim/scripts/phase4d_collector.py` | sync, accounting, staging→atomic dataset publish | baseline (acceptance semantics frozen) |
| Trajectory schema | `src/nao_gz_sim/scripts/nao_trajectory_schema.py` | frozen schema v1.0.1 + command contract | FROZEN |
| Launch wiring | `src/nao_gz_sim/launch/nao_sim.launch.py` | gz sim + producer + trace + collector | baseline |
| Offline tests | `tools/test_phase_a_episode_integrity.py` | 57 Phase A closure checks | baseline |
| Seed tests | `tools/test_p0a_seed_semantics.py` | 14 seed-semantics checks | baseline |
| Collector offline tests | `tools/test_task4_phase4d_offline.py` | 133 collector/schema/validator checks | baseline |

---

## 3. Protected source SHA-256 (freeze anchors)

```text
a024140f294a293b0fb8503e6f53960028d8919c516f1cb640891d9fc297d6fb  tools/run_phase_a_episode.py
cc3e7e95bc900da37c90f14fd38a23226ea45cea0e19f70989c1b4470dd88a3a  tools/nao_episode_integrity.py
00aef7129d65ad78b1bbf219bcec9e9354547c6c0bd8d1ba4fc5948936b820af  tools/run_p0a_gait_candidate.py
9d0dca078f8d07aba577b103382bc4cfbf4be19e2d918352e6005398eee2c55d  tools/validate_task4_phase4d_dataset.py
c761fd1cbefd72f3d6fc99be69c84259e1a0d92472fd3cf9e19dba5f32485d79  tools/test_phase_a_episode_integrity.py
e5a76c1ec4de5997f212e88007c90c7363f6cb83d9da9e02236a2fbbb8730418  tools/test_p0a_seed_semantics.py
c042d47b591c5c37beee70cdb9f32b18457eee63e94d8472a277c6df9c28a7d1  tools/test_task4_phase4d_offline.py
aab60f6fbb18b657736ebf9d7633e65b3d0df021c564cddb537bb806875bd5eb  src/nao_gz_sim/scripts/phase4b_producer.py
4cee76de5db06f7276b9c9803738caa857f0da2d16fbe526d2d86eeb1b6b10fc  src/nao_gz_sim/scripts/nao_gait_phase.py
af3da50cd807922a54bd9ae7cba831d67e46988cba10145759c08d7435a44f68  src/nao_gz_sim/scripts/phase4d_collector.py
81fd94bfe9eb7c85e3c1160432da5d5dfc9b420c787bfda6bb23e531606e629d  src/nao_gz_sim/scripts/phase4b_trace.py
bd4b2bcdb7314ad96420649e4e24d8d3afc97d6b622e3741eadef7de1fa45f3d  src/nao_gz_sim/scripts/nao_trajectory_schema.py
2ed565145441df601455e14a9ab4209333f95264a0e6bb65944373201f2ad787  src/nao_gz_sim/launch/nao_sim.launch.py
```

Cross-check: the four SHA-256 in the Phase A / closure reports
(`run_phase_a_episode.py a024140f…`, `nao_episode_integrity.py cc3e7e95…`,
`test_phase_a_episode_integrity.py c761fd1c…`, `run_p0a_gait_candidate.py 00aef712…`)
match the live files — the freeze is consistent with the accepted evidence.

---

## 4. Protected episode artifacts (must never be rewritten)

### 4.1 smoke_01 — `docs/task_p0a_gait_search/phase_a_evidence/smoke_01/`

```text
17edca0cd34498bd713a3950a0a812dc5e04fec87870346565ef522979071023  collector_summary.json
8805e090d0cb7ca5c4afa1ef2b22878ef70b9a202a44ba008fb531ad6212fe95  dataset/manifest.json
8c64cb9e8584d66a1ce236d6aa57059bde7b1f154d28ff215092074de0d6b300  dataset/samples.jsonl
c1800b0b3f3c32c276cdbdfb5216589aa7b1dd9553dd2c413e6d8b7fb621bb51  episode_integrity.json
6835b1a907f0dc01f0e31bb129ffb14894f4c420aab03d6fc9180b5367b5764c  gz_sim_cmd.actual.txt
6ffaa8d9fc1c1e24872fabce437841b64a969876f53b6e2a9252a82bcc3b76ea  gz_sim_cmd.expected.txt
0815376d6f348688260d883c64a60eca3f1b1786f73216381959d4616dfb4721  launch_cmd.txt
bde78c5a22c1893a8d94992b14008348fc1c0acf0ff6d0608261c361eafae0b9  launch.log
37800739b61f31808c9ed1e5c210caf21fc556b142104952c5961bd62de6619d  manifest.json
0fae07ce7d74b2e7ca5c0bc211e13f03967a91642185cb27a742d5dd2810d155  provenance.json
29a4e0192895e231849919d1022bc30f4cc0b74b23e96d3f8e5bc956bb9c80e2  run_config.json
30badb981f26e878e6efab6a07de66f1ece5d058ec56098ea7029c69df602ca6  trace/body_pose_trace.jsonl
263998337181c6ad9734b189a6b9b5146d5eb21f8677ac73bff1aa463c0e0283  trace/clock_trace.jsonl
ead2702e1bc24515544e8a129b9b3aaf164b2b10fdab398cf9af4ee88a2cbc70  trace/command_trace.jsonl
cc614a4f1670113e3fd999b1c06fb5281028e79ad9ca73501b23fdc0d51c5448  trace/contact_trace.jsonl
552b604646939907f7e5f5b4154534d012eb5159fb374b1428512d6b7f9f7c3b  trace/imu_trace.jsonl
d7fdd11edca82f0dbdf7fefcdae5b9cff86fcbe763d9d54978743f6d16dbe833  trace/phase4b_summary.json
06feed8edb6da825d3f3c0139fe0fb5b8930702410c36a6076b80ffda0062719  trace/state_trace.jsonl
```

### 4.2 closure_01 — `docs/task_p0a_gait_search/phase_a_closure_evidence/closure_01/`

```text
30391a75daf51ab22b6906607590356e4c0092a296c01b507e624c66d59f5716  collector_summary.json
8805e090d0cb7ca5c4afa1ef2b22878ef70b9a202a44ba008fb531ad6212fe95  dataset/manifest.json
8bfb5acf019e8766a53dd5585ed7cfb75c312e43d19a6d6ad6d67f846f41ac64  dataset/samples.jsonl
ee5f1056020ff2582c0de28933d37759b8c071cb3408b580a18fbd3d57b3664a  episode_integrity.json
6835b1a907f0dc01f0e31bb129ffb14894f4c420aab03d6fc9180b5367b5764c  gz_sim_cmd.actual.txt
6ffaa8d9fc1c1e24872fabce437841b64a969876f53b6e2a9252a82bcc3b76ea  gz_sim_cmd.expected.txt
6be62741cc99b8b6a0ce631e76d77fcfcff5923b61df40ff8e2e7c7f0bcc669e  launch_cmd.txt
2773c8ace57027f9ced525dd4291fa3f4e7f9d2d9e5d7fa4a7e0dd21c4c0aead  launch.log
6a0e2263f1e5162e895dacbec61d74382e6345e80bcfcc849bce34854755825a  manifest.json
a6cc303c80d6ebc9a9b2ed3976b74fddc5deae1315501c076ae9eaf1e44d81da  provenance.json
29a4e0192895e231849919d1022bc30f4cc0b74b23e96d3f8e5bc956bb9c80e2  run_config.json
7ce714270638445780c5011eee3155691f6658e99a6e4b4969d0edd6ae7e7d1e  trace/body_pose_trace.jsonl
cc39575b97a776143ebc2c4f32751850ce0b440af573cac79bcee2043f292273  trace/clock_trace.jsonl
490c261ad9a813e389fb053a5b889b00fa8455e0a58aceed0adc1ee8e38cd098  trace/command_trace.jsonl
9f652254914a65c0b95efcd594a4fc2c9a21de52adc6315249cadc44895a97a3  trace/contact_trace.jsonl
0cf7bafa95d0dbd68d5e03953d3b85461f4047bbcb14efc60b8b6159a02e30f1  trace/imu_trace.jsonl
c6e0afb41937a6a71d72b9c4bb45743952951338afd9eef765bf02b5cd73342d  trace/phase4b_summary.json
4c14133a5c40e13529c3d60a5afc4de55bed82cb5553d8f6a183d70edb674362  trace/state_trace.jsonl
```

Note: both `dataset/manifest.json` are byte-identical
(`8805e090d0cb7ca5c4afa1ef2b22878ef70b9a202a44ba008fb531ad6212fe95`) — the dataset
manifest is schema-only (no episode-specific values), as designed.

Freeze rule: **any change to a file in §3 or §4 invalidates the freeze** and must be
justified, or the bug must first be approved as a documented fix.

---

## 5. Frozen Phase A semantics (must be preserved)

### 5.1 Duration semantics

| Field | Frozen meaning |
|---|---|
| `requested_gait_duration_sec` | user-requested gait window (denominator for coverage) |
| `record_window_duration_sec` | start_delay + gait + stop_ramp + hold_final + 1.0 margin |
| `execution_duration_sec` | observed gz sim-time span (clock trace first→last); `null` if absent |
| `accepted_sample_span_sec` | `last - first` accepted sample timestamp; NOT execution duration |
| legacy `actual_duration_sec` / `duration_sec` | DEPRECATED aliases of `accepted_sample_span_sec`, labelled |

### 5.2 Coverage semantics

```text
window        = [gait_start, gait_start + requested_gait_duration_sec]
gait_start    = producer ready_stamp + start_delay_sec
numerator     = union of consecutive accepted-sample intervals clipped to the window;
                interval > 0.10 s (GAP_CAP_SEC) => uncovered (internal gap policy)
denominator   = requested_gait_duration_sec
gait_head_gap_sec / gait_tail_truncation_sec = where coverage is missing
early termination = finish_reason abnormal OR gait_tail_truncation_sec > 0.5
coverage_status   = FULL (>= 0.999) | PARTIAL | null
```

### 5.3 Seed / provenance

- `seed.requested` → `gz_seed:=<n>` → real `gz sim --seed <n>` captured from `/proc`.
- `seed.verified = (requested == actual_gz_sim)`; `GZ_PARTITION`/`ROS_DOMAIN_ID` are
  **not** physics seeds.

### 5.4 Publication / accounting / verdicts

- **Atomic publication**: collector `staging → rename` (dataset); Runner
  `os.replace(staging → out_root)` (episode).
- **Accounting closure**: `joint_messages_seen == accounted_total + unaccounted`
  (closure_01: 1204 == 1204, unaccounted 0).
- **Episode integrity**: PASS/FAIL over acquisition coherence only; classes
  `infrastructure_failure` / `data_integrity_failure`.
- **Gait outcome**: reported, never an integrity failure (`walking_pass=False`,
  `fall_detected=False` in both episodes).
- **Measurement quality / research contract**: `null` placeholders (reserved).

### 5.5 Non-negotiable separation (re-stated)

```text
walking failure            -> episode INVALID                               FORBIDDEN
research contract FAIL     -> discard episode                              FORBIDDEN
episode_integrity  !=  gait_outcome  !=  measurement_quality  !=  contract  REQUIRED
```

---

## 6. Schema freeze

| Schema | Version | Owner | Freeze |
|---|---|---|---|
| Dataset samples/manifest | **1.0.1** | `nao_trajectory_schema.py` | FROZEN — no field change |
| Golden validator contract | derived by hand from v1.0.1 | `validate_task4_phase4d_dataset.py` | FROZEN |
| Episode bundle (manifest/provenance/integrity) | `phase_a_1.0.0` | `run_phase_a_episode.py` | baseline; evolution policy is P2-A (Carry-Forward Register) |

---

## 7. Phase A Closure Audit conclusion (inherited)

```text
PHASE_A_CLOSURE_DECISION = PHASE_A_CLOSURE_ACCEPTED_WITH_FINDINGS
P1-1 = CLOSED
P0 = 0
BLOCKING_P1 = 0
NEXT_PHASE_AUTHORIZATION = ENTER_PHASE_B
```

Accepted capabilities: real Gazebo episode runs; seed chain verifiable; accounting
closure holds; golden validator PASS; episode integrity PASS; duration semantics
disambiguated; gait-relative coverage defined; rejected prefix quantified; internal gap
math handled; early-termination guard verified; walking outcome separated from integrity;
smoke_01 backward compatibility holds.

Two caveats carried forward: closure evidence is single-episode per configuration (no
repeatability claimed), and the closure audit artifact `audit_closure_12346` referenced in
the Phase B brief could not be located (UQ-0).

---

## 8. Freeze verdict

```text
PHASE_B_BASELINE_FREEZE = COMPLETE
```

The baseline is pinned by SHA-256 (§3, §4) and by the frozen semantics (§5, §6). No file
in this record was modified during Phase B-0/B-1. This document and the RFC are the only
new artifacts.

End of Freeze Record.
