# STAGE1_COLLECTOR_CONTRACT — Level 2 collector implementation contract (probe-pinned)

| 项 | 值 |
|---|---|
| Document | STAGE1_COLLECTOR_CONTRACT |
| Status | **Level 1 preparation artifact — collector implementation contract, evidence-pinned（v1.1，随 REV1.2 升版）** |
| Governing RFC | [STAGE1_DYNAMICS_DATASET_RFC.md](file:///home/liu/workspace/Nao_Program/nao_gazebo_ws/docs/STAGE1_DYNAMICS_DATASET_RFC.md)（**REV1.2-CORRECTED，pending re-audit**；App. B V-1/V-2 probe 判据） |
| Evidence base | REV1.2 证据（probe run 6，walk + FROZEN_GAIT 环境，in-gait 窗口）：[V1_report_REV12.json](file:///home/liu/workspace/Nao_Program/nao_gazebo_ws/dataset/stage1/probe/V1_report_REV12.json)、[V2_report_REV12.json](file:///home/liu/workspace/Nao_Program/nao_gazebo_ws/dataset/stage1/probe/V2_report_REV12.json)，schema `stage1-probe/1.2`；REV1.1 历史（run 3）：`V1_report.json` / `V2_report.json`，schema `stage1-probe/1.1` |
| Authorization | Level 2（collector implementation）合同依据。**Level 3（bulk acquisition）未授权**：V-1 仍含 blocking failure（见 §12），且 RFC REV1.2 尚待 re-audit |

本合同把 probe 实测结论固化为 collector 实现的**绑定规则**。规则分三类：

- **[PINNED]** 实测确定的事实（collector 必须按此实现，不得假设其他语义）；
- **[RULE]** 从事实导出的实现规则（fail-closed）；
- **[BLOCKED]** 因 blocking failure 尚不能实现/必须按受限方式实现的部分（RFC 修订前置）。

---

## 1. Evidence base 与 provenance

### 1.1 Probe 运行

**REV1.2 证据（run 6，成品，in-gait 窗口）：**

```
python3 tools/stage1_probe.py --probe both --duration 20.0 --warmup-sec 30.0 \
    --ros-domain-id 88 --gz-partition stage1probe6 --report-suffix REV12
```

- 环境：ROS 2 Jazzy、`ROS_DOMAIN_ID=88`、`GZ_PARTITION=stage1probe6`；probe 经 `--report-suffix REV12` 写入 `V1_report_REV12.json` / `V2_report_REV12.json`（schema `stage1-probe/1.2`；报告含 argv / wallclock / git commit / `config_sha256`）
- 报告 pinning：`git_commit = 9f64d012d9ea…`（REV1.2 修订 commit）；`rfc.sha256 = f006c0f22370…`（RFC REV1.2 文件 hash）
- 终态：**`V-1 verdict = FAIL`**（blocking: `population_cadence`、`publication_mode`；`timestamp_semantics` 在 REV1.2/R2 判据下 **confirmed**：nearest-neighbor 源 stamp 统计 median 0.003 s / p95 0.005 s，同钟域判定成立）；**`V-2 verdict = PASS`**（四判据全 confirmed；`pose_frame_semantics = model_relative_for_links` 与 REV1.2/R4 合同一致）

**REV1.1 历史（run 3，成品）：**

```
python3 tools/stage1_probe.py --probe both --duration 20.0 --warmup-sec 30.0 \
    --ros-domain-id 88 --gz-partition stage1probe3 --allow-overwrite
```

- 环境：ROS 2 Jazzy、Python 3.12.3、gz Harmonic、`ROS_DOMAIN_ID=88`、`GZ_PARTITION=stage1probe3`、`ROS_LOG_DIR=/tmp/stage1_ros_log`
- Probe QoS：ROS 侧 `BEST_EFFORT / KEEP_LAST(50) / VOLATILE`；contact/pose 原始路径用 **gz-transport native subscribe**（默认 QoS，无损）
- 关联窗口：contact 0.05 s、pose 因果窗口 0.02 s（last-not-after）
- 采集期间 RTF（sim/wall）：0.70–0.83（V-2 实测 pose_info 0.707、joint 0.702；V-1 实测 contact 0.827）
- 终态：`V-1 verdict = FAIL`（blocking: `publication_mode`、`timestamp_semantics`——后者为 REV1.1 envelope 判据的 instrumentation artifact，REV1.2/R2 已删除该判据）；`V-2 verdict = FAIL`（blocking: `world_pose_semantics`——REV1.1 字面 world-pose 假设被 falsified，REV1.2/R4 已按实测事实改写合同）。两份报告的 `analysis.criteria[].status` 三态判定为当时终态
- 历史运行归档：`V1_report.run1_fallen.json` / `V2_report.run1_fallen.json`（run 1，倒地）；`V1_report.run2_stand_fallen.json` / `V2_report.run2_stand_fallen.json`（run 2，stand 环境倒地）；`V1_report_REV12.run4_endhold.json` / `V2_report_REV12.run4_endhold.json`（run 4，REV1.2 判据但窗口落在 gait 结束后的 end-hold 相位——两足零接触、gz 侧零消息，V-1 population/body 判据不可测；该归档额外实证：**gait 末段（stop ramp + hold）两足 contact=false 且 contact 话题完全静默**，与 run 3 末段行为一致）。run 2 报告中的三处错误计数源于 probe 三个 bug，已在 run 3 前修复（见 §1.3），**以 run 3 为准**

### 1.2 采集环境（run 6 / run 3 仿真，与冻结 runner 等价）

probe 直接 launch `nao_sim.launch.py`，参数与冻结工件 [run_phase_a_episode.py](file:///home/liu/workspace/Nao_Program/nao_gazebo_ws/tools/run_phase_a_episode.py)（FROZEN_GAIT，L62-88）逐值一致（walk、gait_phase_mode 'explicit_support_swing'、step_length_m 0.02、step_frequency_hz 0.8、swing_height_m 0.008、double_support_ratio 0.3、leg_extension_m 0.24401、gait_blend_sec 1.5、sway_amplitude_rad 0.06、sway_sign -1、arm_* 全套、foot_pitch_swing_rad 0.1、command_rate_hz 100.0、start_delay_sec 0.5、stop_ramp_sec 1.5、hold_final_sec 1.0、actuator_p_gain 15.0、actuator_max_velocity 4.0、actuator_hold_on_start true），`phase4b_duration_sec:=60.0`、`phase4b_record_duration:=65.0`、`phase4b_evidence_dir:=/tmp/stage1_probe_run6_evidence`（run 3: `/tmp/stage1_probe_run3_evidence`）。run 6 仿真日志 `/tmp/stage1_probe_sim6.log`；run 6 时序：模型就绪后 gait 启动，contact 交替 ≥8 次时启动 probe（in-gait 窗口）；run 6 phase4b_summary：142 次交替、净位移 0.531 m、body z 全程 ≥0.330（无跌倒）、gait 末段两足 contact=false。

### 1.3 Probe 修复记录（run 2 → run 3）

三个 bug（run 2 错误结论的根因，run 3 修复并实证）：

1. **V-2 falsy-zero 计数**：`(_identity_deviation(b) or 1.0)` 把恒等位姿（deviation 恰为 0.0，falsy）计为非恒等 → 改为显式 `None` 检查（0.0 = 精确恒等，None = 无位姿）
2. **V-2 falsy-zero 分支**：`(nao_identity_frac or 1.0) <= 0.5` 使 `model_relative_for_links` 分支永不可达 → 改为 `is not None and <= 0.5`
3. **V-1 ground 匹配**：`'ground' in local_name` 只搜 local name（实际 local name 是 `collision`，'ground' 在 model scope）→ 改为全名搜索

---

## 2. Stream inventory（实测）

| Topic | 路径 | 实测速率 | 时钟域 | 用途 |
|---|---|---|---|---|
| `/joint_states` | ROS | **100.0 Hz 精确**（dt 恒 0.01 s，0 backwards，0 duplicates） | sim time | 帧时钟（RFC §3.1 `sim_time_sec`） |
| `/nao/left_foot_contact_raw` | gz native + ROS bridge（双路径实测） | 有效 136.05 Hz（行走窗口）；接触期 1 kHz burst | sim time | 左足 raw contact |
| `/nao/right_foot_contact_raw` | gz native + ROS bridge | 有效 128.90 Hz；接触期 1 kHz burst | sim time | 右足 raw contact |
| `/world/nao_world/pose/info` | **gz-only**（SceneBroadcaster） | 81.33 Hz（dt median 0.012 s，范围 0.004–0.018） | sim time | 25-link pose 源 |
| `/world/nao_world/dynamic_pose_info` | gz-only | 与 pose/info 同批（n=1292） | sim time | 见 §5.5 |
| `/body_pose` | ROS（converter 转发） | 82.05 Hz | sim time（保源 stamp） | 模型条目世界位姿交叉验证 |

**[RULE]** 帧时钟唯一取 `/joint_states` 的 header stamp（`sim_time_sec`，RFC P0-R2 改名）。所有跨流对齐以源 stamp 为准，**禁止**用 callback 到达时间对齐（GPT-5.6 §2.4-2）。

---

## 3. Contact stream contract（V-1 pinned）

### 3.1 [PINNED] 发布模式：门控型（event-only）

- 实测：行走窗口内 116 次抬脚（sim 日志 contact=false）期间 gz 路径 **`n_empty_messages = 0`**——无接触时 gz **不发布消息**（既非空消息也非零值消息）
- 分类：`publication_mode = 'irregular_or_event_only'`，`no_contact_observability = 'not_observable'`
- 附带：消息内字段填充率全 1.0（positions/normals/depths/world/wrenches/body1&2 force/force_offset）——**有消息即有完整 contact**

### 3.2 [BLOCKED→RULE] no-contact 有效性语义（REV1.2/R3 命名）

RFC App. B V-1 失败条件成立："publication mode makes no-contact unobservable with no independent no-contact signal"。

**[RULE]** collector 在沉默窗口（该 topic 无消息）**必须发 `UNKNOWN_NO_MESSAGE`（三态，REV1.2/R3）**，禁止把沉默序列化为 `valid=true` 的零 wrench（RFC §3.6.4 三态语义；GPT-5.6 correction P1-R2）。沉默（no contact message）不得生成零 wrench——"no zero wrench from silence" 是合同条款。

**[BLOCKED]** `VALID_NO_CONTACT` 目前**不可实现**：候选独立信号是 contact_converter 的 ROS boolean（不同代码路径），但其独立性/可靠性未经 probe 验证。RFC 修订必须二选一：(a) 增设该独立信号并单独验证后允许 `VALID_NO_CONTACT`；(b) 明确 Stage1 数据集 contact 状态只有 `VALID_CONTACT / UNKNOWN_NO_MESSAGE` 两态可用。**No force-free fallback**（RFC App. B 原文）。`VALID_NO_CONTACT` 的两种且仅两种成立形式（REV1.2/R3）：显式空 Contacts 消息，或经验证的独立 no-contact 源。

### 3.3 [PINNED] 节律（cadence）

- 接触期 burst dt：median 0.001 s（1 kHz 物理步突发），dt max 1.042/1.062 s（摆动相 gap），duty ≈21%（行走窗口）
- 行走窗口有效速率：run 3（窗口偏 gait 后段）实测左 136.05 Hz、右 128.90 Hz（带内 → confirmed）；**run 6 REV1.2 复跑（in-gait 早期窗口）实测左 181.85 Hz、右 157.72 Hz（超 [50,150] 带 → contradicted）**——有效速率 = duty × burst 率，**随窗口所处 gait 相位变化**（接触占比高 → 有效速率高）
- 注意：run 2（持续接触特例）曾实测 1000 Hz 超带。**[RULE]** REV1.2 未改写本判据（修正范围 R1–R5 不含 cadence）；判据的门控语义改写（"接触活跃期有效速率带 + 沉默期不发消息"）仍为 RFC 修订前置项（§11 R2）。collector 不得以固定速率假设做丢包检测（沉默 ≠ 丢包）

### 3.4 [PINNED] 实体命名（verbatim，collector 关联过滤必须用这些名字）

| Topic | 唯一 collision pair（verbatim 全名） | 计数 |
|---|---|---|
| `/nao/left_foot_contact_raw` | `nao::l_ankle::l_ankle_collision \| ground_plane::link::collision` | 2568/2568（fraction 1.0，0 cross-contamination） |
| `/nao/right_foot_contact_raw` | `nao::r_ankle::r_ankle_collision \| ground_plane::link::collision` | 2288/2288（fraction 1.0，0 cross-contamination） |

**[RULE]** 关联过滤用**全 scoped 名**精确匹配；不得用 local name 或猜测名（'ground' 在 model scope，local name 只是 `collision`——run 2 bug 的教训）。

### 3.5 [PINNED] wrench 体侧识别

- gz `JointWrench` 的 `body_1_name`/`body_2_name` 实测为**空串**（左 5424/5424、右 4996/4996）
- **[RULE]** ankle 侧识别必须用 **Contact 条目的 collision 实体名**（含 `ankle` 的 `nao::*_ankle::*_ankle_collision`），另一侧为 ground；不得依赖 wrench body name 字段

### 3.6 [PINNED] N3L 与 wrench 参考系状态

- 样本级证据：`body_2_force = -body_1_force` **逐分量精确相反**（合力 [0,0,0]）——双侧 wrench 数值自洽
- **[BLOCKED]** `frame_reference` criterion 终态 = **unknown**（非 blocking）：N3L 求和与站立载荷只能作 plausibility check（GPT-5.6 correction 3），**不能**确立 wrench 参考点/表达轴。站立平衡检查因 wrench body name 为空（无法匹配 'ankle'）得 0 样本 → unknown
- **[RULE]** RFC §3.6 的 wrench frame convention（P1-R1）**保持未 pin 状态**：CoP/derived-moment 推导在完成"公共坐标系 + 公共矩心"对比前保持 blocked；collector 只采集 raw wrench（§3.6.1 lossless），不做任何 wrench 参考系解释性变换

### 3.7 [PINNED] 时间戳语义（REV1.2/R2 判据下 confirmed）

- contact 源 stamp 与 `/joint_states` **同一 sim 时钟域**，判据为**源 stamp 的 nearest-neighbor 统计**（REV1.2/R2）：run 6 实测 median 0.003 s / p95 0.005 s（左右足同值；max 1.259 s 为窗口边界伪影——快照截止时 joint 流先止，p95 判定不受影响）；stamp 单调（0 backwards、0 duplicates）
- ROS bridge **精确保 stamp**：`fraction_ros_stamps_exact_in_gz = 1.0`（双足，run 3 与 run 6 一致）
- **REV1.2/R2 更正**：envelope-overlap（订阅捕获窗口重叠）**不再是判据**（REV1.1 的 `fraction_within_joint_envelope` 检查已从 probe 与 RFC 删除）——它是 probe 侧订阅发现 skew 的 instrumentation artifact（见 §5.3），不构成 timestamp-domain validation。REV1.2 复跑（run 6）`timestamp_semantics = confirmed`，无需改写判定

---

## 4. Pose stream contract（V-2 pinned）

### 4.1 [PINNED] 实体清单与命名

- 命名约定 `bare_names`：条目名 = SDF 中的裸 link 名（无 scope 前缀）
- 25 link 全部出现（`fraction_all_25_links = 1.0`，逐条 exact string match），含 `base_link`、`l_ankle`、`r_ankle`
- 消息还含 `*_visual` 条目（SDF visual 元素也出现在 pose/info）——**[RULE]** collector 按 25-link pinned 清单精确选名，忽略 visual 条目
- 单消息 25-link 完整率 1.0 → 无需跨消息拼接

### 4.2 [PINNED] 帧语义（REV1.2/R4 已入 RFC 合同）

- **link entries = parent-frame（model-relative）位姿**，**不是**世界位姿：`link_entry_frame_semantics = 'model_relative_for_links'`（run 3 与 run 6 REV1.2 复跑一致：`base_link_identity_fraction = 1.0`、`nao_entry_identity_fraction = 0.0`、`l_ankle_raw_below_ground_fraction = 1.0`）
- ~~RFC App. B V-2.2 字面假设被 falsified~~ → **REV1.2/R4 已按实测事实改写合同**：RFC 现声明 Pose_V 条目为 parent/model-relative、禁止当世界位姿入库、世界位姿必须经 composition（RFC §3.3 pose frame semantics note / App. B V-2.2）。run 6 `pose_frame_semantics = confirmed`（`V-2 verdict = PASS`）
- `/body_pose` 交叉验证确认：**nao 模型条目 = 世界位姿锚点**（run 6：exact-stamp 匹配 fraction 1.0、位置/四元数偏差 0.0、frame_id 全为 `world`；body_pose_converter 确认按源 stamp 转发模型条目位姿）

### 4.3 [PINNED] 世界位姿恢复公式（composition；R5 措辞更正）

```
p_world = p_model + R(q_model) · p_rel
q_world = q_model ⊗ q_rel          # Hamilton 积；四元数分量序 [x, y, z, w]
```

其中 `p_model/q_model` 取**同一 Pose_V 消息**的 `nao` 模型条目，`p_rel/q_rel` 取该 link 条目。

- 一致性检查：`composed_base_link_vs_nao_entry_max_diff_m = 0.0`（run 3: 1293/1293；run 6: 1336/1336）——**但这是 identity-link 一致性检查，不是端到端验证**（REV1.2/R5：base_link IS 模型帧，合成公式的旋转项从未被该比较检验）
- **[RULE]** collector 记录世界位姿**必须**走此合成路径（RFC REV1.2 §3.3/§3.4/§3.5 已按此改写），**禁止**把 link 条目裸值当世界位姿写库
- **[BLOCKED→RULE]** ~~合成路径写入 RFC 前需修订 App. B V-2.2 措辞~~ → REV1.2/R4 已完成该修订；剩余前置（R5）：**composition 方法需经 selected non-identity links（如 l_ankle/r_ankle）验证**后，合成位姿方可视为 validated——此前合成的 foot/CoM 位姿按 derived-and-pending-verification 对待

### 4.4 [PINNED] 节律与对齐

| 流 | n | 速率 | dt | 单调性 |
|---|---|---|---|---|
| pose/info | 1293 | 81.33 Hz | median 0.012 s（0.004–0.018） | 0 backwards / 0 dup |
| joint_states | 1504 | 100.0 Hz 精确 | 恒 0.01 s | 0 backwards / 0 dup |
| body_pose | 1172 | 82.05 Hz | median 0.012 s | 0 backwards / 0 dup |

- **[PINNED] 因果窗口命中率 = 1.0**（1504/1504，`last-not-after`：pose stamp p 满足 p ≤ t 且 t−p ≤ 0.02 s，RFC §3.6.3 / App. B V-2 criterion 4）
- **[RULE]** collector 逐帧记录 miss flag（窗口未命中 pose 的 joint 帧标记），不插值、不静默 hold（GPT-5.6 §2.4-4）

### 4.5 [PINNED] dynamic_pose/info

- `dynamic_entry_semantics = 'matches_raw_parent_relative'`（n=1292）：**也是 parent-relative**（与 pose/info 一致），不是原生世界位姿流
- 证据：raw 与 pose/info 同名条目差 ~0.0004（同 parent 帧数值噪声级），与合成世界位姿差 ~0.5
- **[RULE]** collector 不把 dynamic_pose/info 当世界位姿源；它与 pose/info 二选一即可，语义相同

### 4.6 站立几何判据的实测注记

行走窗口内 `standing_geometry` 机械伪影：base z 0.186（带内）、l_ankle 0.0（带内）、r_ankle 1.0（超带——右足处于摆动相抬起）。该 criterion 终态本就因 (a) 帧语义 contradicted；此注记仅说明 (e) 项数值在行走环境下不适用，常量不改（RFC criterion 措辞含 "fixed standing geometry"，fail-closed 保留判定）。

---

## 5. Timestamp 与关联合同

### 5.1 [RULE] 时钟域

- 唯一帧时钟：`/joint_states` stamp（`sim_time_sec`）
- 所有源 stamp **原样保留**（contact、pose/info、body_pose、joint 各自的 header stamp），不做重采样时间戳（GPT-5.6 §2.4-2）
- Δt 规则（RFC P0-R3）：`Δt = sim_time_sec[t+1] − sim_time_sec[t]`；`|Δt − 0.010| > 0.005 → clock_valid = false`
- episode 级 `acquisition_wallclock_utc` 与帧级 `sim_time_sec` 分离（RFC P0-R2）

### 5.2 [RULE] 关联窗口（实测支持）

| 关联 | 窗口 | 语义 |
|---|---|---|
| contact → joint 帧 | 0.05 s | last-not-after（RFC §3.6.3）；沉默 → contact 状态 `UNKNOWN_NO_MESSAGE`（§3.2），**禁止**把窗口内无消息写成 valid 零值 |
| pose → joint 帧 | 0.02 s | last-not-after；实测命中率 1.0；逐帧 miss flag |

### 5.3 [PINNED] 包络重叠检查的方法学限制（REV1.2/R2 已删除该判据）

- 实测（run 3，REV1.1 判据下）：V-1 中 contact sim span 18.868 s vs joint 包络 [40.065, 56.615]（16.55 s）→ fraction 0.861/0.868；V-2 中 joint sim span 15.03 s vs pose span 15.885 s（0.855 s 偏差）
- 根因（已判定，非猜测）：**probe 侧订阅启动 skew**。gz 订阅即时生效；rclpy/DDS 订阅发现延迟 0.86–2.3 sim s（逐次运行变化）。两条流同端点截止（快照时刻），ROS 侧包络是 gz 侧包络的严格后缀。算术自洽：wall skew 2.796 s × RTF 0.827 ≈ 2.31 sim s = 包络起点差
- joint 流本身无间隙（dt 恒 0.01 s、0 backwards）；nearest-stamp 偏差实测 3 ms（run 3）/ p95 5 ms（run 6）证明同域
- **[RULE→已执行]** REV1.2/R2：envelope-overlap **不得**用作时钟域判定（probe 与 RFC 均已删除该判据）；时钟域判定用 nearest-stamp 统计（median/p95/max，REV12 probe 已实现）或先裁剪到共同订阅窗口再比较。run 6 实测 `timestamp_semantics = confirmed`

---

## 6. Raw payload retention（lossless，绑定）

### 6.1 [RULE] 采集路径（native gz raw capture 强制；RFC source path 一致）

**contact raw 采集必须走 gz-transport native subscribe**（`/nao/left_foot_contact_raw`、`/nao/right_foot_contact_raw` 的 gz 侧）——这是**强制（mandatory）**要求，非可选项。**RFC source path 与本合同 source path 一致**：RFC REV1.2 §3.6.1（R1）声明同一对 gz-transport native topic 为 canonical raw evidence source，与 collector 的采集路径逐字一致。ROS 桥接路径仅作交叉验证（stamp 精确保留 1.0，但字段有损，见 §6.2；REV1.2/R1：bridge 是 optional derived stream，桥死不死不阻塞 native 路径）。pose 采集走 `/world/nao_world/pose/info`（gz-only topic）。

### 6.2 [PINNED] ROS bridge 字段损失（实测 `bridge_field_loss`）

| gz 消息级 | gz 字段 | ROS 桥字段 | **gz-only 丢失字段** |
|---|---|---|---|
| Contact | collision1, collision2, depth, header, normal, position, world, wrench | collision1, collision2, depths, normals, positions, wrenches | **header（逐 contact stamp）、world** |
| JointWrench | body_1_id/name/wrench, body_2_id/name/wrench | 同名全映射 | 无 |
| Wrench | force, force_offset, torque | force, torque | **force_offset** |

→ 走 ROS 桥会丢失 per-contact header stamp、world 位置、force_offset——三者均为 RFC §3.6.1 lossless raw retention 的必留字段。这是 native 路径强制的实测依据。

### 6.3 [RULE] 序列化要求

- gz.msgs `Contacts` 消息**整包原样序列化**（含 header/world/force_offset/双侧 wrench），另存解析视图（per-foot 力/矩字段，RFC §3.6.2）
- `controller_command` 记 `/joint_command` 上实际发布的目标（保留其源 stamp），不冒充执行器输出或实现关节状态（RFC P0-R4）
- 保留 `nao` 模型条目 + 全部 25 link 条目原始值（composition 的输入），世界位姿作为 derived 字段写入（公式 hash 进 provenance）

---

## 7. Validity 语义（三态，绑定；REV1.2/R3 命名）

| contact 状态 | 触发条件 | 采集期可用性 |
|---|---|---|
| `VALID_CONTACT` | 该足 topic 有非空 Contacts 消息且关联进 0.05 s 窗口 | 可用（字段填充率 1.0） |
| `VALID_NO_CONTACT` | 显式空 Contacts 消息（窗口内）**或**经验证的独立 no-contact 源（REV1.2/R3 仅此两种形式） | **[BLOCKED]** 见 §3.2——当前不可发 |
| `UNKNOWN_NO_MESSAGE` | 窗口内无消息（沉默） | **默认态**（门控发布 + 无独立信号）；**沉默不得生成零 wrench** |

flight 检测、1.0 N 阈值等 REV1.0 条款在 `VALID_NO_CONTACT` 不可发期间对 `UNKNOWN_NO_MESSAGE` 帧一律不判定为 flight（fail-closed）。

---

## 8. Provenance 与完整性（绑定，RFC §3.10 / §4.3）

**[RULE]** 每 episode 记录：

- `git_commit`（含冻结工件 hash 锚）
- `config_hash`：collector 配置 + **对齐常数**（contact 0.05 s / pose 0.02 s 窗口、composition 公式标识）——**与 CoP 公式解耦**（RFC P0-R5 / C4）
- probe 证据 pin：V1/V2 报告文件路径 + 内容 hash（RFC App. B pinning 条款：Level 3 授权要求两份报告 **committed**）
- 环境：ROS_DISTRO、gz 版本、RMW、ROS_DOMAIN_ID、GZ_PARTITION、实测 RTF 区间
- 数据完整性：输出文件 hash（BF-3 模式，写后校验）
- `sim_time_sec` 帧时钟 + `acquisition_wallclock_utc`（P0-R2 两处改名落地）

---

## 9. Collector implementation boundary（GLM §6，重申）

**YES**：新增 subscriber（gz native contact/pose + ROS 话题）、新增 writer 进程、新增 `dataset/stage1/` 布局、新增 tools/ 脚本、计算 RFC 已 pin 的 derived 字段、运行时新增 ros_gz_bridge 实例、向现有 launch 文件传参。

**NO**：修改 `tools/run_phase_a_episode.py`、`tools/run_p0a_gait_candidate.py`、`src/nao_gz_sim/**`（controller/gait generator/launch/converter 内容）、`nao_simple.sdf`、`nao_world.sdf`、physics/simulation behavior、写入 `dataset/stage0/`、重跑 Stage0、训练/优化/LIPM fitting。

---

## 10. Collector smoke validation（实现验收检查，绑定）

collector 实现完成后、任何 pilot episode 前必须通过（GPT-5.6 minimum gate set 第 3 条）：

1. **无损往返**：序列化的 raw Contacts/JointWrench 反序列化后与订阅回调内存中的消息逐字段一致（含 header、world、force_offset、双侧 wrench）
2. **stamp 保真**：落盘 stamp 与源 stamp 精确相等（对照 probe 实测 bridge 保真率 1.0 的基线）
3. **三态行为**：构造沉默窗口（无 contact 消息）→ 落盘必须为 `UNKNOWN_NO_MESSAGE`，不得出现 `valid=true` 零 wrench
4. **合成验证**：落盘世界位姿与同消息 composition 计算值偏差 ≤ 1e-6 m；link 原始值（parent-relative）独立列存
5. **命名匹配**：关联过滤对 verbatim pair（§3.4）命中 fraction = 1.0
6. **provenance 完整**：§8 字段全量存在 + 输出 hash 校验通过
7. **逐帧 flag**：pose 窗口 miss flag 与 contact `UNKNOWN_NO_MESSAGE` flag 按帧落盘

---

## 11. RFC 修订前置清单（Level 3 unblock 条件）

以下修订源自 probe 实测（不 widening、不 silent pass）。**REV1.2 已完成其中 R3、R4 及 R1 的语义部分**（新增状态列）；完成项与剩余项均需经 re-audit 后 Level 3 方可重新评估：

| # | 源 | 修订内容 | REV1.2 状态 |
|---|---|---|---|
| R1 | V-1 publication_mode（blocking） | no-contact 不可观测：增设经验证的独立 no-contact 信号（候选：contact_converter boolean），或明确 contact 状态限两态（`VALID_CONTACT`/`UNKNOWN_NO_MESSAGE`）。App. B V-1 失败条款已预设此路径 | **部分完成**：REV1.2/R3 已定义三态语义与 `VALID_NO_CONTACT` 的两种成立形式；独立信号仍缺 → publication_mode 仍 blocking（run 6 复跑复证） |
| R2 | V-1 population_cadence 语义 | cadence 判据改写为门控语义（接触活跃期有效速率带 + 沉默不发消息），并记录 run 2 持续接触 1000 Hz 特例 | **未完成**（不在本次修正范围 R1–R5 内）：run 6 in-gait 早期窗口实测 181.9/157.7 Hz 超带，再次实证有效速率随 gait 相位变化 |
| R3 | V-1 timestamp_semantics 判据方法 | envelope-overlap 换为 nearest-stamp 域检查（实测 3 ms）或共同窗口裁剪；记录 §5.3 instrumentation limitation | **完成**（REV1.2/R2）：判据已换为源 stamp nearest-neighbor 统计（median/p95/max + p95 域判定），run 6 `confirmed` |
| R4 | V-2 world_pose_semantics（blocking） | App. B V-2.2 "each entry is the world pose" 改为 parent-relative 事实 + 强制 composition（公式与验证数值入 RFC） | **完成**（REV1.2/R4）：RFC 已按实测事实改写（含 R5 的 non-identity-link 验证前置）；run 6 `pose_frame_semantics = confirmed`，V-2 PASS |
| R5 | V-1 frame_reference = unknown | §3.6 wrench 参考点/轴维持"未 pin"状态声明；CoP/derived moment 推导前置条件 = 公共坐标系 + 公共矩心对比（新增小 probe 或文档化检查），在此之前 CoP 后处理标记 blocked | **未完成**（wrench frame/reference 仍 unknown，非 blocking；composition 的 non-identity-link 验证同为前置项） |
| R6 | App. B pinning | 两份 probe 报告 path + hash 写入 RFC pinning 条款（Level 3 要求 committed——commit 动作待用户明确指示） | **报告完成、pinning 部分**：REV12 报告（run 6）已存在并记录 git commit + config hash；remote publication（GPT-5.6 correction 8）仍为 owner 动作 |

---

## 12. 终态判定引用

**REV1.2 复跑（run 6，in-gait 窗口，REV12 报告，schema `stage1-probe/1.2`）：**

- V-1：`verdict = FAIL`，blocking = `['population_cadence', 'publication_mode']`；confirmed = body_identity、**timestamp_semantics（REV1.2/R2 判据：nearest-neighbor 源 stamp 统计 median 0.003 s / p95 0.005 s）**；unknown（非 blocking）= frame_reference
- V-2：`verdict = PASS`，blocking = `[]`；confirmed = unique_entities、**pose_frame_semantics（`model_relative_for_links`，REV1.2/R4 合同语义）**、source_stamps_cadence、alignment_hit_rate
- `PROBE_EXIT = 1`（any FAIL，fail-closed）
- **结论：Level 3 acquisition 仍未授权**（V-1 两项 blocking + RFC REV1.2 pending re-audit + probe 报告 commit/remote publication 未完成）；collector implementation（Level 2）可在本合同约束下进行（§3.2 一处 BLOCKED 项按受限规则实现，§4.3 R5 验证前置）

**REV1.1 历史（run 3，schema `stage1-probe/1.1`，判定保留存档）：**

- V-1：`verdict = FAIL`，blocking = `['publication_mode', 'timestamp_semantics']`（后者为 envelope 判据 instrumentation artifact，REV1.2/R2 已删除该判据）；confirmed = population_cadence、body_identity；unknown（非 blocking）= frame_reference
- V-2：`verdict = FAIL`，blocking = `['world_pose_semantics']`（REV1.1 字面假设被 falsified——REV1.2/R4 已按实测事实改写合同）；confirmed = unique_entities、source_stamps_cadence、alignment_hit_rate

---

## Document status

| 版本 | 状态 |
|---|---|
| 1.0 | Level 1 preparation artifact，基于 probe run 3（schema `stage1-probe/1.1`）实测证据签发。随 RFC 修订（§11）升版 |
| 1.1 | REV1.2 升版：Governing RFC → REV1.2-CORRECTED（pending re-audit）；证据基新增 run 6 REV12 报告（`stage1-probe/1.2`）与 run 4 end-hold 归档；R1 语义命名（`UNKNOWN_NO_MESSAGE`）与 R3（timestamp 判据）、R4（frame semantics 入合同）落合同；R5 措辞更正（composition = identity-link 一致性检查，非端到端验证，需 non-identity-link 验证）；§6.1 声明 native gz raw capture 强制且 RFC source path 一致；§11 标注各修订项 REV1.2 状态；§12 终态换为 run 6 REV12 判定 |
