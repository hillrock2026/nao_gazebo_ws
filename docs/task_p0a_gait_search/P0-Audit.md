# P0-a 第一阶段：NAO 可测前向步态只读链路审计

工作区：`/home/liu/workspace/Nao_Program/nao_gazebo_ws`
状态：**只读审计**（本文件不修改任何代码、数据或配置）。
关联：`docs/Gate2A_Gap_Inventory.md`、`docs/task5_gate2a_batch0_candidate/STATUS.md`、
`docs/task4_phase4b_evidence/gait_design.md`、`docs/Phase2F_Freeze_Readiness.md`。

```text
Gate 2A：未通过
正式 Phase 2F final split：未生成
eval_fixed：未永久冻结
Gate 2B：未进入
batch-0：候选，不计入正式 Gate 2A
P0-a（可测前向步态）：未解决
P0-b（≥3 类真实失败）：未解决
```

---

## 1. Gait 生成调用链

```text
nao_sim.launch.py
  ├─ gz sim (nao_world.sdf, dartsim, max_step_size=0.001, gravity -9.8)
  ├─ spawn nao_simple.sdf (model name "nao")，运行时插入且仅插入一个插件：
  │     libnao_phase4a_actuator.so  (NaoPhase4aActuator)
  │        p_gain 默认 8.0 / max_velocity 默认 2.0 （Phase 4B/4E charter 用 15.0 / 4.0）
  │        hold_on_start 默认 false（charter true）
  ├─ ros_gz_bridge /clock（冻结 QoS）
  ├─ joint_state_bridge → joint_state_converter → /joint_states
  ├─ body_pose_converter  ← gz `/world/nao_world/pose/info`  → /body_pose (frame_id=world)
  ├─ imu_bridge → imu_converter → /imu/data
  ├─ contact_bridge → contact_converter → /left_foot/contact, /right_foot/contact
  ├─ world_tf_converter → /tf (world→base_link)
  ├─ robot_state_publisher (canonical URDF TF)
  ├─ phase4b_producer.py  → 发布 /joint_command (24 维物理位置目标)
  └─ phase4b_trace.py     → 订阅全部话题，写 JSONL traces + summary（只读）
```

控制回路（唯一写仿真的组件）：

```text
/joint_command (24 位置目标, sensor_msgs/JointState)
   → NaoPhase4aActuator::PreUpdate (每个物理步)
        velocity = clamp(p_gain * (target - current), ±max_velocity)
        → gz::sim::components::JointVelocityCmd
   → dartsim 物理积分
```

**关键结构性事实：这是纯开环关节位置伺服播放，没有平衡/ZMP/CoM 反馈控制器。**
`phase4b_producer.py` 只按内部相位生成髋/膝/踝/手臂目标角，从不读取 `/body_pose`、`/imu/data`
或接触状态作为反馈。`NaoPhase4aActuator` 只做“目标角 → 关节速度”的 P 伺服，不读取姿态。

---

## 2. Gait 参数定义与默认值

来源：`src/nao_gz_sim/scripts/phase4b_producer.py`（`Phase4bProducer.__init__`）与
`nao_sim.launch.py` 的 launch 默认值。冻结 charter（实际走路配置）在
`docs/task4_phase4e_pathb_b3_evidence/b3_0/candidates.json#[kind=anchor].config`。

| 参数 | launch/代码默认 | B3.0 charter 实测配置 | 物理含义 |
|---|---:|---:|---|
| `gait_method` | `walk` | `walk` | stand | walk |
| `step_length_m` | 0.03 | **0.02** | 矢状面足端步长（半幅 ±step/2） |
| `step_frequency_hz` | 0.8 | **0.5** | 步态周期频率 |
| `swing_height_m` | 0.015 | **0.008** | 摆动足抬升高度 |
| `double_support_ratio` | 0.20 | **0.30** | 双支撑占周期比例 |
| `leg_extension_m` | 0.24401 | 0.24401 | 髋到足底伸展长度（屈膝站位） |
| `sway_amplitude_rad` | 0.15 | **0.06** | 侧向髋/踝 roll 重心移动幅值 |
| `arm_swing_rad` | 0.20 | 0.12 | 肩 pitch 摆幅 |
| `arm_neutral_pitch_rad` | 1.50 | 1.5 | 自然垂臂基准 |
| `arm_swing_sign` | 1 | 1 | 手臂相位符号 |
| `foot_pitch_swing_rad` | 0.10 | 0.10 | 摆动足 pitch 幅值 |
| `command_rate_hz` | 100.0 | 100.0 | 命令发布率 |
| `start_delay_sec` | 0.5 | 0.5 | 模型 ready 后延迟 |
| `gait_blend_sec` | 1.5 | 1.5 | stance→gait 混合 |
| `duration_sec` | 12.0 | 18.0（采集用 16.0） | 步态持续仿真秒 |
| `stop_ramp_sec` / `hold_final_sec` | 1.5 / 2.0 | 1.5 / 2.0 | 收尾 |
| `swing_ratio` | 由 ds 推出 `0.5 - ds/2` | 0.35 | 单腿摆动占周期比 |
| `p_gain` / `max_velocity` | 8.0 / 2.0 | **15.0 / 4.0** | 关节速度伺服增益/限速 |
| `hold_on_start` | false | true | 首条命令前保持起始位姿 |

关节限位在 `phase4b_producer.LIMITS` 与 `nao_simple.sdf` 双份维护，由
`tools/check_joint_limits_consistency.py` 校验。步态几何：每条腿在矢状面 (x-z) 相对髋做
“支撑直线 + 摆动弧”，解析 2-link IK（大腿 0.100、小腿 0.1029、踝到足底 0.04511），
两腿相位差 180°，`sway = -sway_amp*sin(2πphase)` 同步到髋/踝 roll。

**没有参数表示目标线速度或 cmd_vel**：速度是关节轨迹的隐式结果，这正是问题所在。

---

## 3. Measured body pose 的真实来源

```text
gz `/world/nao_world/pose/info` (Pose_V)
  → body_pose_converter.cpp (按 model name "nao" 选取)
  → /body_pose (geometry_msgs/PoseStamped, frame_id = "world")
```

- 来源为 **Gazebo 真值 model world pose**（不是 odom、不是 TF 外推、不是 cmd_vel）。
- model `nao` 即 `base_link`（`base_link` 与 `torso` 之间为 identity fixed joint，
  `tools/check_task3a_contract.py` 校验）。
- 记录字段（`phase4b_trace.on_body`）：`t, x, y, z, roll, pitch, yaw, qx, qy, qz, qw`。
- 时间戳：Gazebo 仿真时间 `header.stamp`，converter 强制**严格递增**，backwards/duplicate 会
  fail/跳过（`body_pose_converter.cpp`）。因此轨迹内**不应有 reset 跳变或时间回退**；
  分析器仍须显式检查非单调时间戳。
- 采样：实测约 62.5 Hz（`phase4e_pathb_independent_fk_recompute.py` 记录），间隔可略有抖动。
- 坐标：世界系；机器人 spawn 于原点、无初始旋转，因此初始前向轴 = 世界 **+x**。
  但 body_pose 含 yaw，前向应投影到窗口起始朝向（见 §4 / 分析工具）。
- 能区分本体运动（model origin）与局部足端运动（足端由 `phase4b_trace` 的
  `sole_position_base` FK 计算后变换到世界系）。
- 不含 base 线速度字段；速度必须由位置序列差分得到。

---

## 4. 速度计算所需字段（已存在的真实字段）

来自 `body_pose_trace.jsonl`（每行）：

```text
t                     仿真秒
x, y, z               世界系 base_link 位置 [m]
roll, pitch, yaw      由四元数导出的姿态 [rad]
qx,qy,qz,qw           归一化四元数
```

来自 `state_trace.jsonl`：24 关节位置（可用于足端 FK、跟踪误差）。
来自 `contact_trace.jsonl`：`{t, side, value}`（ground_plane-only 布尔接触）。
来自 `command_trace.jsonl`：`{t, recv_sim, ok, position|reason}`。
来自 `run_config.json` / `result.json`：完整 gait/controller 参数、`ready_sim`、`seed`。

**当前链路并不缺失速度所需的最小字段**：仅用 `body_pose_trace` 的 `t,x,y` 即可计算
净/前向/侧向位移与实际速度。真正缺失的是**能产生前向位移的步态本身**，不是测量能力。

---

## 5. 当前数据链路缺失项

1. **无 balance/ZMP/CoM 反馈**：producer 不订阅姿态/IMU/接触；执行器不读姿态。
2. **无躯干前倾 / CoM 前移参数**：目标角里没有让质心越过支撑面的机制。
3. **seed 无语义**：`phase2b_baseline_a.SEED_LIST` 注释明确“producer 无 RNG、无噪声模型，
   seed 不改变轨迹”，seed 仅作 replicate 标识（`seed_semantics`）。
4. **无足端力/滑移反馈**：只能从 `contact_trace` 与 FK 事后推断。
5. **接触仅 bool**：`contact_converter` 只判定“主足碰撞 vs ground_plane”，无接触力/位置。
6. **命令为隐式速度**：没有“期望前进速度”字段可与实测速度对照（但这符合本任务要求）。

---

## 6. 零净位移的最可能根因（按现有测量证据排序）

现有 52 条正式 episode 的**实测**证据（见 §7 与本目录 baseline 报告）：

- 前向速度 `mean_speed_x_mps`：mean `+0.00022` m/s，std `0.00380`，即 **噪声量级，无一致前向运动**。
- 平均 `|body_dx|`（前向）= 0.0242 m vs 平均 `|body_dy|`（侧向）= 0.0494 m →
  **侧向位移约为前向的 2 倍**（侧漂主导）。
- 稳态窗口最大 `|yaw|` 偏移：mean 20.5°，max **326.5°** → 部分 episode 发生明显自旋。
- 最大 roll/pitch 常达 180°/90°（跌倒），tilt mean 11.4°。

### 6.1 A0/A1/A2 组统计（更正：net 与 lateral 分列）

> **更正说明**：先前版本摘要表把 `lateral_displacement` 的组均值写进了“净位移”列。
> 下表直接从 `docs/task_p0a_gait_search/baseline_existing_52_measured.json`
> （工具 `tools/analyze_measured_gait.py`）重新计算：net 与 lateral 分列，单位 **m**，
> 数值为 **group mean ± group std（population，n=10）**。
> net 组均值为 0.0459 / 0.0491 / 0.0511 m（与 GLM-5.3 独立复核一致）；
> lateral 组均值为 0.0437 / 0.0451 / 0.0496 m。更正**不改变**
> “现有三组没有可区分前向速度”的结论。

| 档 (config_id) | n | forward_displacement (m) | lateral_displacement (m) | net_displacement (m) | actual_speed_mean (m/s) |
|---|---:|---:|---:|---:|---:|
| A0 (`step_length_m`=0.010) | 10 | -0.0003 ± 0.0114 | +0.0437 ± 0.0227 | 0.0459 ± 0.0212 | -0.00001 ± 0.00106 |
| A1 (`step_length_m`=0.020) | 10 | -0.0119 ± 0.0190 | +0.0451 ± 0.0195 | 0.0491 ± 0.0226 | -0.00110 ± 0.00175 |
| A2 (`step_length_m`=0.024) | 10 | +0.0019 ± 0.0104 | +0.0496 ± 0.0197 | 0.0511 ± 0.0184 | +0.00019 ± 0.00096 |

三组 forward 均值均在 ±0.012 m 内且符号不一致，确认为“近零、不可区分”；
net 数值主要由 lateral 分量贡献（net 略大于 |lateral|，forward 近零）。

根因假设（H1 证据最强）：

- **H1（主因）开环无力学推进**：足端相对髋的轨迹摆动靠摩擦反推，但因无 ZMP/CoM 控制，
  反作用主要转化为机身侧漂/自旋而非净前移。证据：侧向位移 > 前向位移、yaw 大幅漂移。
- **H2 sway 非零均值残余**：`sway = -amp*sin(2πφ)` 理论零均值，但双腿相位/双支撑与足端
  不对称使每周期侧向冲量不抵消，累积成侧漂。
- **H3 步长/摆高过小**：step 0.02 m、swing 0.008 m 相对腿长 0.248 m 极小，摩擦可轻易打滑。
- **H4 支撑足打滑 / 无脚底力反馈**：μ=1 但 P 速度伺服无扭矩限制，支撑腿可能在原地滑。
- **H5 无躯干前倾/CoM 前移**：姿态目标只含臂/腿/踝，没有让质心前移越过支撑多边形。
- **H6 接触判定仅 ground_plane bool**：无法区分真实支撑与轻触/穿透。

**结论（审计层）**：当前冻结步态不是“速度档不可区分”，而是**根本不产生稳定前向净位移**；
必须先让步态产生可测前向位移，才谈得上 ≥3 个可区分速度档。

---

## 7. 计划修改的最小文件集合（尚未修改）

按优先级（后续阶段实施，且改动必须逐项记录前后值与理由）：

| 优先级 | 文件 | 拟改内容 | 预期对 measured body pose 的影响 |
|---|---|---|---|
| 1 | `src/nao_gz_sim/scripts/phase4b_producer.py` | 增加少量**前向推进相关**参数（躯干/髋前倾偏置、CoM 前移、支撑相足端后移比例、非对称 sway 偏置）；不改基本相位结构 | 若有效，`body_dx` 出现单调、跨 seed 稳定的正位移 |
| 2 | `src/nao_gz_sim/launch/nao_sim.launch.py` | 仅当新增 producer 参数需要时，镜像为 launch 参数 | 无（纯接口暴露） |
| 3 | 世界/足底摩擦 | `nao_world.sdf` 地面 `mu`、`nao_simple.sdf` 足碰撞 `mu`（默认 1.0） | 影响打滑；仅在 H4 被证实需要时才动 |
| 4 | 执行器参数（已有 launch 参数，无需改代码） | `actuator_p_gain` / `actuator_max_velocity` | 影响关节跟踪与支撑刚度 |

**本阶段（只读审计）不修改上述任何文件。**
如需修改 gait/controller/physics，报告将列出：文件、修改前后参数、理由、对 measured body pose
的预期影响、是否影响正式数据复现、如何恢复/复现。

---

## 8. 明确不修改的正式数据与历史 artifacts

```text
四个正式 dataset（task4_phase2b/2b_smoke/2c/2d evidence/work/dataset）：不修改
四份 Phase 2E report（task4_phase2e_labels_quality_evidence/reports/*.json）：不修改
Phase2F_Split_Policy.md / Phase2F_Evaluation_Goals.md：不修改
Phase2F_Freeze_Readiness.md：不修改（除非确有与 P0-a 直接相关且明确说明）
batch-0 候选（task5_gate2a_batch0_candidate）：不纳入正式数据，不修改
正式 dataset 的 splits/：不写入
Gate 2A / Phase 2F 状态结论：不变
```

P0-a 的所有新输出只写入 `docs/task_p0a_gait_search/`（及后续
`docs/task_p0a_batch_validation/`），不写入任何正式 dataset。

---

## 9. 下一阶段（输出给第二阶段）

1. 冻结可复核的 measured 速度契约（前向轴、起止窗口、净/前向/侧向位移、实际速度、
   标准差、异常处理），实现/复用离线分析器。
2. 用现有 52 条 episode 复算，建立“当前步态” measured 基线（证明零前向位移）。
3. 仅在小规模候选层搜索前向推进相关参数，每档 ≥5 条、≥3 seed，保留全部失败/零位移。
4. 只有在 ≥3 档 measured 前向速度稳定可区分时，才宣布 P0-a 小批通过，
   且**不**代表 Gate 2A 通过。
