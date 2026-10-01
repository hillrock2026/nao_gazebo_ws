# P0-a Forward-Gait Execution Report

工作区：`/home/liu/workspace/Nao_Program/nao_gazebo_ws`
日期：2026-09-25
范围：完成四项轻量修复 → 冻结判定契约 → 修改 gait → 单 seed 快筛 → 3 seed 复现。

```text
Gate 2A：未通过   正式 split：未生成   eval_fixed：未永久冻结   Gate 2B：未进入
P0-a：未解决      P0-b：未解决
```

---

## 1. 任务范围

- 本轮**只**寻找一个 forward-gait candidate。
- **未**设计三个速度档位；**未**声称 slow/medium/fast。
- **未**进入 P0-b；**未**实现 external_push。
- **未**进行 100–200 条正式数据扩充；**未**生成 Baseline_Report.md。
- **未**生成正式 Phase 2F split；**未**执行 freeze；**未**永久冻结 eval_fixed。
- 所有新输出写入 `docs/task_p0a_gait_search/work/`，未写入任何正式 dataset。

## 2. 四项轻量修复

| 修复项 | 文件 | 修改内容 | 测试结果 |
|---|---|---|---|
| 1. net/lateral 表格错位 | `docs/task_p0a_gait_search/P0-Audit.md` §6.1 | 从 baseline JSON 重算，net 与 lateral 分列并标单位/mean±std：A0/A1/A2 net = 0.0459/0.0491/0.0511 m；lateral = 0.0437/0.0451/0.0496 m | 数值复核一致；结论“三组无可区分前向速度”不变 |
| 2. trace 截断规则 | `measured_contract.md` §3；`tools/analyze_measured_gait.py` | 显式输出 nominal/actual window、effective duration、`trace_truncated`、`missing_duration_sec`；`effective<5.0s` → `measurable=false` | `test_analyze_measured_gait.py` 截断用例 PASS |
| 3. yaw 缺失显式失败 | `tools/analyze_measured_gait.py` | 删除 `r.get('yaw',0.0)` 静默 fallback；yaw/roll/pitch 缺失或 NaN/Inf → `measurable=false` + `attitude_field_missing/nonfinite`（含 field/index/run_dir） | yaw 正常/缺失/NaN/Inf/wrap 用例 PASS |
| 4. fall 统计规则 | `measured_contract.md` §5；`tools/analyze_measured_gait.py` | fall = `body_min_z<=0.25m` 或 `torso_tilt>=60°`（实测证据，不读 failure_reason）；fall 保留完整 metrics、单列 `fall_runs`；组统计报告 total/non-fall/fall/fall_rate，非 fall 才进入正常速度统计 | `test_analyze_measured_gait.py` fall 用例 PASS |

针对性测试：

```text
python3 tools/test_analyze_measured_gait.py    -> 40/40 passed
python3 tools/test_task4_phase4e_analysis.py   -> 32/32 passed
```

`baseline_existing_52_measured.json` 已用修复后的工具重算（52 runs / 52 measurable / 25 groups；
fall=1，为 phase2d probe_step_length_max/episode_000006，tilt≈180°、min_z≈0.06 m）。

## 3. Forward candidate 判定契约

冻结文件：`docs/task_p0a_gait_search/forward_candidate_charter.md`（在 3-seed 复现**之前**写入）。

- measured pose 来源：`/world/nao_world/pose/info → body_pose_converter → /body_pose → body_pose_trace`。
- 坐标系：窗口起点 measured `yaw` 的固定前向轴；世界位移投影到前向/侧向轴。
- 窗口：`[ready+start_delay+gait_blend, ready+start_delay+duration]`；trace 截断显式记录。
- 指标：forward/lateral/net、path_length、effective duration、actual_speed_mean、
  逐帧 speed std、yaw_change、torso tilt、fall、contact alternation、truncation、anomalies。
- 冻结阈值：`min_forward=0.08 m`、`|lateral|/forward<=0.5`、`|yaw_change|<=30°`、
  `tilt<=35°`、`eff>=8 s`、`missing<=1 s`、`path/net<=8`、`max_single_step<=0.02 m`、
  `contact_alt>=3`、`swing_clearance>0.002 m`、`required_seeds=3`、`allowed_fall=0`。
- 预登记 seed：`101, 202, 303`；三 seed 配置完全一致。
- 阈值依据：既有 52 条非 fall 数据 `|forward|` p95=0.054 m、`|yaw|` max=24.6°、
  `path/net` median=33（近原地）。阈值取自既有分布上界，未按新结果调整。

## 4. Gait 修改

根因（实测）：原 `sway = -A·sin(2πφ)` 的**符号反了**。对窗口内 body pose 分相位统计显示：
左腿摆动期（φ∈[0,0.35)）body y=+0.02~+0.028 m（向**左**/摆动侧），即重心被移向**摆动脚**，
而非支撑脚；双脚同时受力 → 原地抖动（`path_over_net≈45`），无净前移。修复为
`sway = sway_sign·(-A·sin(2πφ))`，`sway_sign=-1` 时重心移向支撑脚。

| Iteration | 文件/参数 | 修改理由 | 预期效果 | 实际结果（seed101） |
|---|---|---|---|---|
| 0 | 冻结 anchor（`sway_sign=+1`） | 复现旧行为（参考） | 近原地 | fwd 0.049 m，path/net 44.9，yaw 27.3° |
| 1 | 仅 `sway_amplitude_rad=0.15`（符号未改） | 增大摆幅测试单支撑 | 若重心仍错则更差 | fwd −0.012 m，lateral 0.038，path/net 63.4 → 失败 |
| 2 | **`phase4b_producer.py` + launch 新增 `sway_sign`，`sway_sign=-1`** | 把重心移到支撑脚 | 真正单支撑，摩擦推进 | fwd **0.105 m**，lateral 0.005，path/net **5.5**，tilt 1.1° → 通过 |
| 3 | `sway_sign=-1` + `step_frequency_hz=0.8` | 单支撑成立后提高步频 | 前移增加 | fwd **0.138 m**，path/net 6.7，yaw −5.6° → 选定 |
| 4 | `sway_sign=-1` + `step_length_m=0.04` | 增大步幅 | 前移增加 | fwd 0.203 m，但 yaw 26.9°、lateral 0.079（比 0.39）→ 保留观察 |
| 5 | `sway_sign=-1` + `step=0.04` + `freq=0.8` | 两者叠加 | 更快 | fwd 0.229 m，但 **yaw −43.1°** → 拒绝（自旋） |

修改文件：`src/nao_gz_sim/scripts/phase4b_producer.py`（新增 `sway_sign` 参数，默认 +1 保持历史行为）、
`src/nao_gz_sim/launch/nao_sim.launch.py`（暴露 `phase4b_sway_sign`）。
未修改 controller/physics/friction；未修改历史数据与正式 artifact。

## 5. 快速探索结果

所有产生数据的候选（单 seed=101；`ok=false` 的为基础设施无效，非 gait 失败）：

| Candidate | Seed | Forward (m) | Lateral (m) | Net (m) | Speed (m/s) | Yaw (°) | Tilt (°) | Fall | path/net | Contact alt | Result |
|---|---:|---:|---:|---:|---:|---:|---:|---|---:|---:|---|
| baseline_anchor | 101 | 0.0486 | 0.0078 | 0.0492 | 0.00335 | 27.31 | 8.26 | no | 44.9 | 32 | 近原地 |
| c_sway015 (sign+1,sway.15) | 101 | -0.0124 | 0.0380 | 0.0400 | -0.00086 | -0.82 | 9.91 | no | 63.4 | 31 | lateral/path 失败 |
| c_swing030 (swing.03) | 101 | — | — | — | — | — | — | — | — | — | 基础设施无效（子 shell 未 source ROS，无 trace） |
| c_swayneg006 (**sign−1**) | 101 | 0.1049 | 0.0049 | 0.1050 | 0.00723 | 7.72 | 1.08 | no | 5.5 | 14 | 通过快筛 |
| c_swayneg012 (sign−1,sway.12) | 101 | 0.0873 | -0.0203 | 0.0897 | 0.00602 | -2.73 | 0.53 | no | 8.68 | 14 | path/net 略超 |
| c_swayneg018 (sign−1,sway.18) | 101 | 0.0767 | -0.0155 | 0.0782 | 0.00529 | 10.12 | 0.52 | no | 13.7 | 14 | 低于 min_forward |
| **c_neg_f08 (sign−1,freq.8)** | 101 | **0.1382** | 0.0108 | 0.1386 | 0.00954 | -5.64 | 4.51 | no | 6.69 | 19 | **选定** |
| c_neg_sl04 (sign−1,step.04) | 101 | 0.2029 | 0.0787 | 0.2176 | 0.01400 | 26.92 | 0.94 | no | 2.99 | 14 | 通过但 yaw 接近上限 |
| c_neg_f08_sl04 | 101 | 0.2288 | -0.0754 | 0.2409 | 0.01579 | -43.09 | 4.29 | no | 6.00 | 15 | yaw>30 拒绝 |

说明：`c_swing030` 与首次 `baseline_anchor`（timeout）均为运行环境问题（后者为 conda Python 抢占
`/usr/bin/env python3` 导致 rclpy 导入失败；前者为并行子 shell 未 source 安装空间）。
修复运行器环境后无 gait 失败被删除或替换。

## 6. 三 seed 复现结果

候选 `fwdf08`：`sway_sign=-1`、`step_frequency_hz=0.8`，其余等于 anchor。
三 seed `run_config.json` 字节一致（`md5=ba032f49d6e1f9b982f26e9fab70816b`）。

| Candidate | Seed | Forward (m) | Lateral (m) | Actual speed mean (m/s) | Yaw change (°) | Max tilt (°) | Fall | Trace valid | Pass |
|---|---:|---:|---:|---:|---:|---:|---|---|---|
| fwdf08 | 101 | 0.1195 | 0.0193 | 0.00825 | -0.63 | 0.58 | no | yes (eff 14.49 s) | ✓ |
| fwdf08 | 202 | 0.1210 | 0.0164 | 0.00835 | -1.74 | 0.58 | no | yes (eff 14.49 s) | ✓ |
| fwdf08 | 303 | 0.1301 | 0.0155 | 0.00898 | 2.02 | 0.83 | no | yes (eff 14.49 s) | ✓ |

聚合（`work/reproduce/fwdf08_gate.json`，全部通过冻结规则）：

```text
mean forward displacement      = 0.1235 m
std forward displacement       = 0.0047 m
mean actual speed              = 0.00852 m/s
episode-level actual speed std = 0.00032 m/s
mean / max lateral-to-forward  = 0.139 / 0.161
mean / max |yaw change|        = 1.46° / 2.02°
mean torso tilt                = 0.66°
fall count                     = 0
```

raw evidence：
`docs/task_p0a_gait_search/work/reproduce/fwdf08_seed{101,202,303}/`
（`trace/body_pose_trace.jsonl`、`state_trace.jsonl`、`contact_trace.jsonl`、
`phase4b_summary.json`、`result_p0a.json`、`launch.log`）。
离线复算：`fwdf08_recompute.json` 与各 episode 内嵌 measured 逐字段一致。

## 7. 真实性检查

measured forward displacement **不是**由下列因素造成：

- **cmd_vel / 参数名**：分析器只读 body pose，从未读取 cmd_vel 或参数名（契约 §7）。
- **世界坐标方向误用**：前向轴 = 窗口起点 measured yaw 的固定投影；
  `forward_rel_fit` 三 seed 的 R² = 0.998–0.999，即沿该轴近似匀速直线累积。
- **reset / pose jump**：`max_single_step` 三 seed 0.00087–0.00106 m（占总前移 <0.9%），
  无 `non_monotonic_timestamp` / `irregular_sample_interval` anomaly。
- **整体侧滑**：lateral/forward ≤ 0.161；`path_over_net` 5.1–5.7（旧 gait 为 33–45）。
- **原地自旋**：|yaw change| ≤ 2.02°（阈值 30°）。
- **单 seed 异常**：三 seed 同号且同量级（0.1195/0.1210/0.1301 m，std 0.0047 m）。
- **真实迈步**：按相位统计世界系足底高度，左摆动相（φ≈0.12–0.38）左脚高于右脚
  4–6 mm，右摆动相（φ≈0.62–0.88）右脚高于左脚 4–6 mm；最小足底 z≈0.0026 m（无穿地）；
  contact alternation 22–23，support 23–25。

限制：接触信号仅为主碰撞 bool，未直接测得切向接触力；上述“未打滑”结论来自
body pose 线性累积 + 足底离地 + 低 path/net 的联合推断，**不是**接触力测量。

## 8. 最终结论

**结论 A：找到一个通过初步复现验证的 forward-gait candidate。**

```text
已找到一个通过初步复现验证的 forward-gait candidate：
- 3 个预登记 seed（101/202/303）；
- 每个 seed 均产生超过冻结阈值（0.08 m）的正向 measured forward displacement
  （0.1195 / 0.1210 / 0.1301 m）；
- 侧向漂移没有主导（|lateral|/forward ≤ 0.161）；
- yaw/self-spin 受控（|yaw| ≤ 2.02°）；
- 无 fall；
- measured pose 与 contact 证据完整；
- 三 seed 配置完全一致。
```

**但必须同时声明：**

```text
这只是一个 forward-gait candidate；
尚未形成三个可区分速度档位；
P0-a 总体尚未完成；
尚不能进入 P0-b；
尚不能进入正式 Gate 2A 扩充。
```

## 9. 后续状态

```text
三个 measured 速度档位：尚未完成
P0-a 总体：尚未完成
P0-b：尚未进入
Gate 2A：未通过
正式数据扩充：未开始
Phase 2F final split：未生成
Phase 2F freeze：未执行
eval_fixed：未永久冻结
```

隔离核验（阶段末一次性）：

```text
4 个历史 dataset 聚合 SHA-256：65/11/27/21 文件，全部 MATCH
4 份 Phase 2E report SHA-256：全部 MATCH
2 份 Phase 2F policy 文档 SHA-256：全部 MATCH
batch-0 hashes.json：145 文件全部 MATCH
新目录中无 status=final；无正式 task4_phase2f* split 目录；
eval_fixed 仅历史 preview，未新增冻结
新输出全部位于 docs/task_p0a_gait_search/work/，未写入正式 dataset
```

## 10. 下一轮最小改动建议

1. 以 `fwdf08` 为基线，仅调整一个因素（如 `step_length_m`）并把 `|yaw_change|` 作为约束，
   寻找更低旋转、更高速度的配置；`step_length=0.04` 已显示前移 0.20 m 但 yaw 达 26.9°，
   需先解决 yaw（例如对称化摆动/落地或加入 yaw 稳定项，但需单独记录）。
2. 待出现 ≥3 个稳定候选后，再讨论三个可区分速度档位（本轮明确不做）。
3. 采集侧建议覆盖完整名义窗口（本轮已覆盖，missing ≤0.013 s）。
