# P0-a Forward-Gait Candidate Charter（先冻结，后实验）

工作区：`/home/liu/workspace/Nao_Program/nao_gazebo_ws`
状态：**判定规则在此冻结**。本文件在运行任何 3-seed 复现实验**之前**写入；
阈值只依据**既有 52 条** measured 数据、NAO 尺寸、窗口时长与采样率确定，
**不依据本轮新筛选结果**。实验后不得修改。

```text
Gate 2A：未通过   P0-a：未解决   P0-b：未解决
正式 split：未生成   eval_fixed：未冻结
```

---

## 1. Measured pose 来源（唯一合法来源）

```text
/world/nao_world/pose/info
  → body_pose_converter
  → /body_pose
  → state_trace / body_pose_trace 中的 base_link 世界位姿
```

- 实际速度/位移**只能**来自上述 measured body pose。
- **禁止**用 `cmd_vel`、理论速度、期望速度、`step_length_m` 等参数名、
  或 candidate 名称作为速度/位移依据。

## 2. 坐标与前向方向

- 前向轴 = 分析窗口**起点**的 measured `yaw`（先 unwrap）；
  窗口内该轴固定，不逐帧旋转。
- 世界位移投影到初始前向轴 / 侧向轴：
  `forward_unit=(cos yaw0, sin yaw0)`，`left_unit=(-sin yaw0, cos yaw0)`。
- `yaw` 必须存在、有限、正确 unwrap；缺失/NaN/Inf 直接 `measurable=false`。
- 不允许逐帧旋转坐标后把曲线运动误计为直线前进。

## 3. 分析窗口与 trace 截断

- 名义窗口 `[lo, hi] = [ready+start_delay+gait_blend, ready+start_delay+duration]`。
- 实际窗口以窗口内**最后一个有效 measured pose** 为右端；截断必须显式记录
  （`nominal/actual window`、`effective duration`、`truncated`、`missing duration`）。
- 最小有效时长 `minimum_effective_duration_s = 8.0`（低于此不可测，不得作通过证据）；
  且 `missing_duration_sec <= 1.0`（严重截断拒绝）。
- 新实验采集时长覆盖完整名义窗口。

## 4. 必须输出的指标（每 episode）

```text
forward_displacement_m   lateral_displacement_m   net_displacement_m
path_length_m            effective_duration_s
actual_speed_mean_mps    actual_speed_std_mps     （逐帧瞬时速度总体 std）
yaw_change_deg           max_torso_tilt_deg
fall                     body_min_z_m
contact alternation summary（n_support / alternation_count / alternation_fraction）
trace_truncated / missing_duration_sec
anomalies
```

- **两个 std 严格区分**：
  - `actual_speed_std_mps` = 单 episode 逐帧瞬时速度 std（平滑性）；
  - **episode-level 稳定性** = 各 seed 的 `actual_speed_mean_mps` 之间的 std。

## 5. 预冻结阈值（依据既有 52 条 + 模型尺寸）

既有 52 条（非 fall，n=51）实测分布（`baseline_existing_52_measured.json`）：

```text
|forward_displacement| : median 0.011 m, p95 0.054 m
|lateral_displacement| : median 0.044 m, p95 0.079 m
|yaw_change|           : median 8.1°, p95 18.6°, max 24.6°
torso_tilt_max         : median 8.0°, p95 8.7°
path_over_net          : median 33.4（近原地抖动）
max_single_step        : median 0.0067 m
名义窗口 ≈ 14.5 s，采样 ≈ 62.5 Hz，leg_ext=0.248 m
```

冻结阈值：

```text
min_forward_displacement_m      = 0.08    （> 既有 p95=0.054）
max_abs_lateral_to_forward_ratio = 0.5    （侧向不得主导）
max_abs_yaw_change_deg          = 30.0    （> 既有非 fall max 24.6；禁止主要靠旋转）
max_torso_tilt_deg              = 35.0    （沿用 phase4b walking attitude gate）
minimum_effective_duration_s    = 8.0
max_missing_duration_s          = 1.0
max_path_over_net               = 8.0     （远离既有近原地 median 33）
max_single_step_m               = 0.02    （防 reset/pose jump；≈1.25 m/s @62.5Hz）
min_contact_alternations        = 3       （沿用 phase4e MIN_ALTERNATION_COUNT）
min_swing_foot_clearance_m      = 0.002   （沿用 phase4b foot_swing_ok）
required_seed_count             = 3
allowed_fall_count              = 0
seeds                           = 101, 202, 303  （预先固定）
```

理由：既有三档 forward 均值在 ±0.012 m 内、std ≤0.019 m，`min_forward=0.08 m`
高于全部非 fall episode 的 p95；侧向/偏航/路径阈值均取自既有分布上界或既有冻结
gate，不在看到新结果后调整。

## 6. Forward candidate 通过规则（全部满足才通过）

1. ≥3 个不同 seed，且为预登记 `101/202/303`；
2. 每个 seed `forward_displacement_m > min_forward_displacement_m`（严格正）；
3. 每个 seed 都超过最小有效前向位移阈值（同上）；
4. 三 seed 方向一致（forward 同为正）；
5. 不能只有一个 seed 前进、其余原地/后退（由 2+4 保证）；
6. `|lateral|/forward <= 0.5`（lateral 不主导）；
7. `|yaw_change_deg| <= 30`（不主要靠自旋）；
8. `fall == 0`（任一 seed fall → 不通过）；
9. measured pose 有效（`measurable=true`，无 attitude/anomaly 硬错误）；
10. `effective_duration_sec >= 8.0` 且 `missing_duration_sec <= 1.0`（无严重截断）；
11. contact：`alternation_count >= 3` 且至少一脚 swing clearance > 0.002 m；
12. 前向位移不由整体侧滑/坐标错误/单次异常跳变造成：
    `path_over_net <= 8.0` 且 `max_single_step_m <= 0.02`；
13. 三 seed 使用**完全相同**的 gait/controller/physics 配置，不为单 seed 调参。

## 7. fall 规则

- fall 由实际 measured 高度/tilt 判定：`body_min_z_m <= 0.25 m` 或
  `max_torso_tilt_deg >= 60°`。
- fall episode **保留**完整 measured metrics，单独计入 fall count/rate，
  **不得**用于证明 candidate 的正常速度稳定性。
- 任一 seed fall → 本轮 candidate **不通过**。

## 8. 本轮唯一目标与停止点

- 只找**一个** forward-gait candidate；不设计三个速度档；不进入 P0-b；不做正式扩充。
- 停止点：一个 candidate 在 ≥3 个预登记 seed 上均满足第 6 节全部规则。
- 成功后只能声称：“找到一个通过初步复现验证的 forward-gait candidate”，
  仍须声明：三个 measured 速度档位未完成、P0-a 未完成、未进入 P0-b/Gate 2A。
