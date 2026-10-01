# P0-a 第二阶段：Measured 速度指标契约（冻结）

工作区：`/home/liu/workspace/Nao_Program/nao_gazebo_ws`
状态：离线、只读；**只使用 measured body pose**，禁止 cmd_vel / 理论速度 / 参数名。
工具：`tools/analyze_measured_gait.py`（复用 `tools/phase4e_analysis.py` 的稳态窗口）。

```text
Gate 2A：未通过   正式 split：未生成   eval_fixed：未冻结   Gate 2B：未进入
P0-a：未解决      P0-b：未解决
```

---

## 1. body pose 来源与坐标系

- 来源：`/body_pose`（Gazebo 真值 model `nao` 的 world pose，frame_id=`world`），由
  `src/nao_gz_sim/src/body_pose_converter.cpp` 从 `/world/nao_world/pose/info` 选取。
- model `nao` == `base_link`（`base_link` 与 `torso` 为 identity fixed joint）。
- 记录：`body_pose_trace.jsonl` 的 `t, x, y, z, roll, pitch, yaw, qx..qw`。
- 世界系；机器人 spawn 于原点、无初始旋转 → 初始前向轴 = 世界 **+x**。

## 2. 前向轴定义

```text
forward_unit = (cos(yaw0), sin(yaw0))
left_unit    = (-sin(yaw0), cos(yaw0))
yaw0         = unwrap(yaw) at the first sample inside the steady window
```

用窗口起始实测 yaw 作为朝向，避免把初始偏航误差计入前向；前向与侧向互为正交投影。

## 3. 时间窗口与 trace 截断

名义窗口（nominal window）：

```text
[nominal_lo, nominal_hi] = phase4e_analysis.steady_window(cfg, ready)
         = [ready + start_delay_sec + gait_blend_sec, ready + start_delay_sec + duration_sec]
```

- 排除 blend-in 与 stop ramp/hold，只取稳态步态段。
- `ready` 取 `result.json.labels.ready_sim`；缺失时回退到 `state_trace` 首样本时间。

实际窗口（actual window）：

```text
[actual_lo, actual_hi] = [首个, 末个] 落在名义窗口内的有效 measured pose 时间
effective_duration_sec = actual_hi - actual_lo
trace_truncated        = actual_hi < nominal_hi   （name 下显式布尔）
missing_duration_sec   = max(0, nominal_hi - actual_hi) + max(0, actual_lo - nominal_lo)
```

- 若 state trace 早于名义窗口右端结束，**实际窗口以最后一个有效 measured pose 为准**；
  工具必须记录并输出 `nominal window`、`actual window`、`effective duration`、
  `truncated=true/false`、`missing duration`，**不得把被截断的 trace 静默表现为完整名义窗口**。
- 新实验应尽量保证采集覆盖完整名义窗口（采集时长 ≥
  `start_delay_sec + duration_sec + margin`），截断仅作为可披露的数据属性，不得事后补造。
- 窗口内样本数 < `DEFAULT_MIN_STEADY_SAMPLES(50)` → `measurable=false`，不静默丢样。
- **最小有效时长门槛**：`effective_duration_sec < DEFAULT_MIN_EFFECTIVE_DURATION_SEC(5.0)` →
  `measurable=false`（`reason=insufficient_effective_duration`），该 episode **不得**作为
  gait candidate 通过证据，也不得据其推算速度。
- 时间戳非单调 / 采样间隔异常（> 20×均值）→ 记入 `anomalies`。

## 4. 位移与速度定义

```text
start_pose, end_pose   窗口首/末 body_pose
body_dx, body_dy, body_dz   end - start
net_displacement_m      ||end - start||_xy
forward_displacement_m  (end - start) · forward_unit
lateral_displacement_m  (end - start) · left_unit
effective_duration_sec  t_end - t_start
actual_speed_mean_mps   forward_displacement / effective_duration
actual_speed_std_mps    population std of per-sample projected forward velocity
path_speed_mean_mps     path_length / effective_duration
yaw_change_deg          unwrap(yaw)[last] - unwrap(yaw)[first] over the window
path_over_net           path_length / net_displacement_m
max_single_step_m       max ||pose[k+1]-pose[k]||_xy over the window（防跳变）
```

- 用**净前向位移/时长**作为档位主指标（非路径长度），防止原地摆腿/绕圈被误计为前进。
- `path_over_net` 监控“路径长但净位移小”（原地摆动/侧漂）风险。
- **两个不同量纲的 std 必须区分**：
  - `actual_speed_std_mps` = 单 episode **逐帧瞬时速度**的总体标准差（步态平滑性指标）；
  - `episode-level actual_speed_mean std` = 多个 episode 的 `actual_speed_mean_mps` 之间的
    标准差（复现稳定性指标，见 §6/§8）。不得混用。

## 5. 质量、异常与 fall 指标

```text
torso_tilt_max_deg = max_window( max(|roll|, |pitch|) )   [deg]
body_min_z_m       = min_window( body z )                 [m]
contact.*          alternation_count / alternation_fraction（复用 phase4e_analysis.alternation）
anomalies          non_monotonic_timestamp / irregular_sample_interval /
                   zero_net_displacement / lateral_dominates_forward
```

### 5.1 姿态字段（强制）

- `yaw`、`roll`、`pitch` 必须存在且为有限数值。
- yaw 缺失 / 非数值 / NaN / Inf → 该 episode `measurable=false`
  （`reason=invalid_attitude_field`），并在 `anomalies` 记录
  `attitude_field_missing` / `attitude_field_nonfinite`（含 `field`、`index`、`run_dir`）；
  **不得**退化为世界 x 轴或默认 0.0；**不得**用宽泛 try/except 吞掉该错误。
- roll/pitch 同样强制（否则会掩盖 fall）。

### 5.2 fall 判定与统计（强制）

fall 由**实际 measured 姿态/高度证据**判定（不得只读取已有 `failure_reason`）：

```text
fall = (body_min_z_m <= 0.25 m)  OR  (torso_tilt_max_deg >= 60.0 deg)
```

- `fall_evidence` 保留每一类命中的具体证据（`height_drop` / `torso_tilt`），
  多个异常同时出现时全部保留。
- **fall episode 不能被删除**：必须保留完整 measured metrics。
- fall episode 必须**单独计入** fall count / fall rate，**不得**用于证明某个
  gait candidate 的正常速度稳定性。
- 组统计必须同时报告：`total_episode_count`、`non_fall_measurable_count`、
  `fall_count`、`fall_rate`，以及**仅基于 non-fall** 的 speed/displacement statistics；
  fall 明细列于 `fall_runs`。
- 若候选的**任一复现 seed 发生 fall**，则该 candidate 本轮**不通过**。

## 6. 档间判定（待小批数据）

仅当 ≥3 档每档 ≥5 条、≥3 seed 时才判定。阈值必须在看到结果前冻结（见
`P0-a search plan` 章节/后续 `batch_validation` 报告），不得事后挑选。报告项：

```text
tier pair | mean_diff | within-tier std | pooled within std | separation=diff/pooled | overlap
```

## 7. 禁止项

用 `cmd_vel`、理论速度、参数名、参数排序或 gait preset 名称作为档位判定；用路径长度冒充
净前移；用异常值制造档间差异；删除失败/零位移 episode；把 batch 结果并入正式 Gate 2A。
