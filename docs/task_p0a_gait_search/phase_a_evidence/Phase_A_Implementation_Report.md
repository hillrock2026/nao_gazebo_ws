# Phase A Implementation Report — NAO Gazebo 数据采集最小闭环

- Task: `docs/Phase A - NAO Gazebo 数据采集最小闭环.txt`
- Author: opencode / DeepSeek
- Date: 2026-09-30 (Asia/Shanghai)
- Workspace: `/home/liu/workspace/Nao_Program/nao_gazebo_ws`
- Scope: 建立并冻结“一个可复现、可审计的最小 Episode 数据生产闭环”。不是 gait optimization，
  不是 Stage 7 contract validation，未修改 P1-R v3，未新增 gait algorithm。
- Final: **`PHASE_A_READY`**

> 执行前环境变更（已向用户确认）：任务开始前，历史 `docs/` evidence（含 `p1r_support_swing/`、
> `p1bd_repair/`、`task4_phase4e_pathb_b3_evidence/` 等）已被移出工作区到
> `/home/liu/workspace/Nao_Program/backup/`（673 MB），工作区被 zip 快照，`.git` 不存在。
> 经用户决定，Phase A 采用 **self-contained** 方案：**不恢复、不修改任何归档 evidence**，
> 自建冻结 gait 配置 + 新 orchestrator，仅复用 launch / producer / collector / schema。

---

## 1. 当前实现审计（component / status / reuse|modify|add / reason）

| Component | Current status | Decision | Reason |
|---|---|---|---|
| `src/nao_gz_sim/launch/nao_sim.launch.py` | 完整：Gazebo + producer + trace + **phase4d collector** 均可由参数启用；`gz_seed` 已转发 `gz sim --seed` | **reuse** | 已满足 A1/A2/§10；不改一行 |
| `src/nao_gz_sim/scripts/phase4b_producer.py` | 完整 B-only gait 生产器 | **reuse** | §11 禁止改 gait math；仅通过 launch 参数选择冻结值 |
| `src/nao_gz_sim/scripts/phase4d_collector.py` | 成熟：staging → 独立 validator → 原子目录 rename；样本核算；reject reason；时间/接触/IMU/command 新鲜度；schema 校验 | **reuse** | §1 明确规定“不允许重新设计 collector” |
| `src/nao_gz_sim/scripts/nao_trajectory_schema.py` | 冻结 schema v1.0.1（manifest/sample 契约 + 命令契约） | **reuse** | 冻结契约，禁止改字段 |
| `tools/validate_task4_phase4d_dataset.py` | 独立 golden validator（精确匹配 manifest + 逐样本校验） | **reuse（import）** | 作为 `episode_integrity` 的深契约校验，避免第二套 schema |
| `tools/run_p0a_gait_candidate.py` | 存在 **config merge 顺序 bug**（§9）：`EXPERIMENT_DEFAULTS` 覆盖用户 override | **modify** | §9 要求修复；已加纯函数 `resolve_config` + 单测 |
| episode→manifest/provenance | 只有 P0-a runner 的 provenance；无 episode 级 manifest；无 `episode_integrity` | **add** | A4/A5/§6/§7 缺口 |
| Episode Integrity validator | 不存在 | **add** | A5/§12/§13/§14 |
| Phase A 单命令入口（smoke） | 不存在（仅有裸 `ros2 launch` 脚本） | **add** | §15/§18 |

`GAP` 汇总：① §9 merge bug；② 无 episode 级 manifest/provenance（§6/§7）；③ 无 `episode_integrity`
（A5/§12–14）；④ 无单命令 smoke（§15）；⑤ 无失败路径测试（§16）。均以 **minimal add** 解决，未重写 collector。

---

## 2. 修改 / 新增文件

| File | Change | Reason |
|---|---|---|
| `tools/run_p0a_gait_candidate.py` | 新增纯函数 `resolve_config(anchor, overrides)`（precedence: anchor < defaults < overrides）；`run_candidate` 改用它 | §9：用户 override 必须胜过 `EXPERIMENT_DEFAULTS` |
| `tools/test_p0a_review_remediation.py` | `R3_AUTHORITATIVE_12['tools/run_p0a_gait_candidate.py']` 由 `52c790113d80` → `00aef7129d65` | 保持 runner SHA 固化与实现一致（用户批准的 Fix-in-place 范围） |
| `tools/nao_episode_integrity.py` | **新增**：离线 `episode_integrity` 判定器 + CLI | A5/§12/§13/§14；与 `contract_status`/`measurement_quality` 严格分离 |
| `tools/run_phase_a_episode.py` | **新增**：Phase A 单 episode orchestrator（self-contained 冻结 gait；staging→原子发布；manifest/provenance/integrity；smoke 打印） | A1–A4/§5–§10/§15 |
| `tools/test_phase_a_episode_integrity.py` | **新增**：26 项离线测试（失败路径 + §13 fall + §9 precedence + smoke 格式） | §16 |

未改动：`nao_sim.launch.py`、`phase4b_producer.py`（含 gait math）、`phase4d_collector.py`、
`nao_trajectory_schema.py`、`validate_task4_phase4d_dataset.py`、任何历史 evidence。

新/改文件 SHA-256：

```text
tools/run_phase_a_episode.py           8b7fa5edef7ba8dcb48839c4420f17c2e20fa69b5101432231245611fc9073b2
tools/nao_episode_integrity.py         f039bc6cd777d5fbb3629faf7bd46d5ab6d96749d34bbd0c2ff2c699907847bc
tools/test_phase_a_episode_integrity.py 4857561e607616daf8fa8ae8661b70cd4a9641509c9903def56b2d41b0e872ed
tools/run_p0a_gait_candidate.py        00aef7129d65ad78b1bbf219bcec9e9354547c6c0bd8d1ba4fc5948936b820af
tools/test_p0a_review_remediation.py   9489ed58eb040315d70f870d2dc574a6bdad864e23700720a21ab8059dba3d54
```

---

## 3. 数据闭环

```text
Frozen gait parameters θ (FROZEN_GAIT) + seed + duration + episode_id
   │  resolve_config(): FROZEN_GAIT < user overrides          (tools/run_phase_a_episode.py)
   ▼
ros2 launch nao_gz_sim nao_sim.launch.py  (headless, one process group)
   ├─ gz sim -s -r --seed <seed> --physics-engine dartsim <world>   ← §10 真实 argv 从 /proc 捕获
   ├─ phase4b_producer.py  (explicit_support_swing, 0.8 Hz, sway_sign -1)  ← gait math 未改
   ├─ phase4b_trace.py     → trace/*.jsonl (clock/command/state/contact/body_pose/imu)
   └─ phase4d_collector.py → dataset/ (staging → golden validator → 原子 rename)
   ▼
episode staging dir  (<out-root>.staging.<pid>/)
   ├─ dataset/{manifest.json, samples.jsonl}   frozen schema v1.0.1
   ├─ trace/…                                  raw per-topic traces (含 body_pose)
   ├─ provenance.json  manifest.json  episode_integrity.json
   ├─ launch_cmd.txt  gz_sim_cmd.actual.txt  gz_sim_cmd.expected.txt  run_config.json  launch.log
   ▼
episode_integrity  (tools/nao_episode_integrity.py)  —— 与 contract_status / measurement_quality 分离
   ▼
os.replace(staging → <out-root>)   ← 原子发布；绝无半写入 episode 被当作完整 episode
```

**原子发布（A3）**：collector 自身 `episode.tmp → validator → rename`；Phase A 再把整个 episode
staging 目录用一次 `os.replace` 落地。失败（infra/data-integrity）也**如实发布**为
`episode_integrity=FAIL`，绝不伪装成正常 episode（§16/§17）。

**三类失败分离（§14）**：`infrastructure_failure`（未产出 dataset / 0 样本 / 采集被提前截断）、
`data_integrity_failure`（schema 非法、时间轴损坏、信号缺失/非有限）、`gait_outcome`（fall / 大 yaw /
漂移 / clearance —— **永不**计入 integrity，§13）。

---

## 4. 实际执行命令

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 tools/run_phase_a_episode.py \
    --episode-id smoke_01 --seed 12345 --duration 8.0 \
    --out-root docs/task_p0a_gait_search/phase_a_evidence/smoke_01 \
    --timeout 240 --allow-overwrite
```

实际 `ros2 launch` argv（逐字记录于 `smoke_01/launch_cmd.txt`）：

```text
ros2 launch nao_gz_sim nao_sim.launch.py gui:=false use_rsp:=true enable_phase4a:=true
  phase4a_run_probe:=false phase4a_run_trace:=false enable_phase4b:=true phase4b_run_gait:=true
  phase4b_run_trace:=true actuator_hold_on_start:=true actuator_max_velocity:=4.0 actuator_p_gain:=15.0
  phase4b_arm_neutral_pitch_rad:=1.5 phase4b_arm_neutral_roll_rad:=0.0 phase4b_arm_phase_offset_rad:=0.0
  phase4b_arm_swing_rad:=0.12 phase4b_arm_swing_sign:=1 phase4b_command_rate_hz:=100.0
  phase4b_double_support_ratio:=0.3 phase4b_duration_sec:=8.0 phase4b_elbow_bend_rad:=0.3
  phase4b_foot_pitch_swing_rad:=0.1 phase4b_gait_blend_sec:=1.5 phase4b_gait_method:=walk
  phase4b_gait_phase_mode:=explicit_support_swing phase4b_hold_final_sec:=1.0 phase4b_leg_extension_m:=0.24401
  phase4b_start_delay_sec:=0.5 phase4b_step_frequency_hz:=0.8 phase4b_step_length_m:=0.02
  phase4b_stop_ramp_sec:=1.5 phase4b_sway_amplitude_rad:=0.06 phase4b_sway_sign:=-1
  phase4b_swing_height_m:=0.008 phase4b_walking_direction:=forward
  phase4b_evidence_dir:=<staging>/trace phase4b_record_duration:=12.0
  enable_phase4d:=true phase4d_run_collector:=true phase4d_output_dir:=<staging>/dataset
  phase4d_summary_path:=<staging>/collector_summary.json phase4d_duration_sec:=12.0 phase4d_overwrite:=true
  phase4d_validator_script:=<ws>/tools/validate_task4_phase4d_dataset.py gz_seed:=12345
```

**seed provenance chain（§10，真实执行路径）**：请求 `seed=12345` → `gz_seed:=12345` → 实际
`gz sim` argv（从 `/proc/<pid>/cmdline` 捕获，写入 `gz_sim_cmd.actual.txt`）：

```text
gz sim -v 4 -r -s --seed 12345 --physics-engine gz-physics-dartsim-plugin
      <ws>/install/nao_gz_sim/share/nao_gz_sim/worlds/nao_world.sdf
```

`provenance.seed.verified = true`（requested == actual_gz_sim == 12345）。**不是**只写 JSON 的假 seed。

---

## 5. Episode 实际结果

```text
episode id        : smoke_01
seed              : 12345 (requested == actual gz sim invocation)
duration          : requested 8.0 s, actual 8.48 s (sample span)
sample count      : 849 (synchronized schema samples)
raw trace counts  : clock 12648, state 1201, command 1201, imu 1200, contact 938, body 784
signals           : JOINT_STATE PASS / COMMAND PASS / IMU PASS / CONTACT PASS / BODY_POSE PASS / CLOCK PASS
integrity         : episode_integrity = PASS   (failure_class = null)
termination       : normal_completion (collector finish_reason = duration_complete)
residual procs    : 0
dataset sha256    : manifest 8805e090d0cb7ca5c4afa1ef2b22878ef70b9a202a44ba008fb531ad6212fe95
                    samples  8c64cb9e8584d66a1ce236d6aa57059bde7b1f154d28ff215092074de0d6b300
golden validator  : valid=True, manifest_ok=True, samples_ok=True, samples=849, errors=0
```

Gait outcome（**信息性，非 integrity**，§13）：`producer_contract_pass=True`、
`walking_pass=False`（净前进 0.053 m < 0.08 m 阈值）、`fall_detected=False`、lateral −0.0004 m、
torso height loss 0.0044 m、alternation 17。即“采集完整但 gait 质量未达标”——正是
`episode_integrity = PASS` 且 `gait 表现不佳` 的预期分离（§4）。

Episode 目录：

```text
docs/task_p0a_gait_search/phase_a_evidence/smoke_01/
    manifest.json  provenance.json  episode_integrity.json
    launch_cmd.txt  gz_sim_cmd.actual.txt  gz_sim_cmd.expected.txt  run_config.json  launch.log
    collector_summary.json
    dataset/manifest.json  dataset/samples.jsonl        (frozen schema v1.0.1)
    trace/{clock,command,state,contact,body_pose,imu}_trace.jsonl  trace/phase4b_summary.json
```

---

## 6. 测试结果

```text
PASS  test_task4_phase4d_offline.py            133/133   (existing collector/schema/validator offline)
PASS  test_p0a_seed_semantics.py                14/14    (existing seed semantics)
PASS  test_phase_a_episode_integrity.py         26/26    (NEW: failure paths §16 + fall §13 + §9 + smoke)
PASS  ONE REAL GAZEBO EPISODE (smoke_01)                 (EPISODE_INTEGRITY = PASS, residual 0)

FAIL  test_p0a_gate_contract.py                          PRE-EXISTING: frozen_gates_v2.json archived
FAIL  test_p0a_cli_contracts.py                          PRE-EXISTING: p1r_support_swing/p1bd_repair archived
FAIL  test_p0a_review_remediation.py                     PRE-EXISTING: p1r_support_swing/*.json archived
FAIL  test_p0a_no_source_write.py  (14/19; 5 sub-fails)  PRE-EXISTING: same gate-file/p1bd deps
NOT RUN  Stage 7 / P1-R whole-episode suite              OUT OF SCOPE (Phase A 不做 contract validation)
```

`test_p0a_no_source_write.py` 的 5 个失败子项与任务开始前 **baseline 完全一致**（同一集合）；
本任务新增的 3 个 `tools/*.py` 未被 write-guard 标记为任何写入。四个 `FAIL` 的根因**相同**：
`docs/task_p0a_gait_search/p1r_support_swing/frozen_gates_v2.json` 与 `p1bd_repair/*` 在任务开始前
已被移出工作区，与 Phase A 改动无关（任务开始前即为 FAIL，已记录）。

---

## 7. Remaining Issues

**P0**：无。

**P1**
1. （环境，pre-existing，非 Phase A 引入）三个回归测试因归档输入缺失而 FAIL：
   `test_p0a_gate_contract.py`、`test_p0a_cli_contracts.py`、`test_p0a_review_remediation.py`
   及 `test_p0a_no_source_write.py` 的 4 个 sub-fail。修复方式：从
   `/home/liu/workspace/Nao_Program/backup/` 恢复 `p1r_support_swing/`（含 `frozen_gates_v2.json`）
   与 `p1bd_repair/` 到 `docs/`；Phase A 按用户决定未恢复。
2. `provenance.launch_configuration.argv` 与 `launch_cmd.txt` 记录的是**发布前 staging 路径**
   （真实执行命令，真实性优先）。原子发布后 staging 路径已不存在。建议 Phase B 在 provenance 中
   额外记录 `run_dir_publish_path` 映射，或采用“相对路径 + run_dir_rel”规范化（不改变真实性）。

**P2**
3. `body_pose` 目前只存在于 raw trace（`body_pose_trace.jsonl`），未进入冻结 `samples.jsonl`
   （frozen schema 禁止加字段，Phase A 刻意未扩展）。若后续 dataset 需要**同步后的** body pose，
   应在 Phase B 做 schema 版本化演进，而非改动 v1.0.1。
4. `tools/run_p0a_gait_candidate.py` 的 `_residual_count()` 为**全机**扫描（任何并发 `gz sim` 都会
   计入）。Phase A 沿用同一语义；受控单机环境下可接受，多任务并发时需按 `GZ_PARTITION` 过滤。
5. gait 结果：本次 B 配置 `walking_pass=False`（前进 0.053 m < 0.08 m）。**这是 gait 表现问题，不是
   Phase A 缺陷**，记录备查，按 §11/§17 不在本阶段修复。

---

## 8. Phase A 建议结论

```text
PHASE_A_READY
```

依据（仅当全部满足才可宣布 READY）：

```text
[OK] 真实 Gazebo Episode 成功产生        (smoke_01, 849 samples, gz sim 8.15.0 headless)
[OK] 数据完整                             (golden validator valid=True, errors=0, unaccounted=0)
[OK] provenance 完整                      (seed 真实 argv 已验证 true；gazebo/launch/collector/code 均记录)
[OK] integrity validator 成功判定         (episode_integrity=PASS；失败路径 26/26)
[OK] 清理完成                             (residual_processes = 0)
```

不表示：gait 质量 PASS、Stage 7 contract validation、P1 closure 或 P2 authorization。

完成后停止，不进入 Phase B。

End of report.
