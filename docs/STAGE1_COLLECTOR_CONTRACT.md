# STAGE1_COLLECTOR_CONTRACT — Level 2 collector implementation contract (probe-pinned)

| 项 | 值 |
|---|---|
| Document | STAGE1_COLLECTOR_CONTRACT |
| Status | **Level 1 preparation artifact — collector implementation contract, evidence-pinned（v1.4，随 R12 fail-closed enforcement 升版）** |
| Governing RFC | [STAGE1_DYNAMICS_DATASET_RFC.md](file:///home/liu/workspace/Nao_Program/nao_gazebo_ws/docs/STAGE1_DYNAMICS_DATASET_RFC.md)（**REV1.4-HARDENING，ACQUISITION-CONTRACT-READY，pending final independent audit acceptance**；App. B V-1/V-2 probe 判据） |
| Evidence base | REV1.3 证据（probe run 8，walk + FROZEN_GAIT 环境，in-gait 窗口）：[V1_report_REV13.json](file:///home/liu/workspace/Nao_Program/nao_gazebo_ws/dataset/stage1/probe/V1_report_REV13.json)、[V2_report_REV13.json](file:///home/liu/workspace/Nao_Program/nao_gazebo_ws/dataset/stage1/probe/V2_report_REV13.json)，schema `stage1-probe/1.3`，**V-1 与 V-2 双 PASS**；REV1.2 历史（run 6）：`V1/V2_report_REV12.json`（schema 1.2）；REV1.1 历史（run 3）：`V1/V2_report.json`（schema 1.1） |
| Authorization | Level 2（collector implementation）合同依据。**Level 3（bulk acquisition）未授权**：probe 双 PASS 仅为判据证据，Level 3 仍需 RFC REV1.3 final independent audit 接受 + 报告 pinning + remote publication（owner 动作，见 §12） |

本合同把 probe 实测结论固化为 collector 实现的**绑定规则**。规则分三类：

- **[PINNED]** 实测确定的事实（collector 必须按此实现，不得假设其他语义）；
- **[RULE]** 从事实导出的实现规则（fail-closed）；
- **[BLOCKED]** 因 blocking failure 尚不能实现/必须按受限方式实现的部分（RFC 修订前置）。

---

## 1. Evidence base 与 provenance

### 1.1 Probe 运行

**REV1.3 证据（run 8，成品，in-gait 窗口，V-1/V-2 双 PASS）：**

```
python3 tools/stage1_probe.py --probe both --duration 20.0 --warmup-sec 30.0 \
    --ros-domain-id 88 --gz-partition stage1probe8 --report-suffix REV13
```

- 环境：ROS 2 Jazzy、`ROS_DOMAIN_ID=88`、`GZ_PARTITION=stage1probe8`；schema `stage1-probe/1.3`（报告含 argv / wallclock / git commit / `config_sha256`）
- 报告 pinning：`git_commit = 559c24374064…`（REV1.3 common-window 修订 commit）；`rfc.sha256 = d6a5e02286bb…`（RFC REV1.3 文件 hash）
- 终态：**`V-1 verdict = PASS`**（四 blocking 判据全 confirmed：`contact_publication_events`（R6 事件语义：左 28 bursts / 右 24 bursts、median_event_dt 0.001 s、active_contact_event_rate 898/937 Hz、silence 27/23 段均计为非失败）、`body_identity`（verbatim pair fraction 1.0）、`publication_mode`（R7 两态合同下 event-only 为合同一致，非失败）、`timestamp_association_compatibility`（R10：共同订阅窗口 nearest-neighbor median 0.003 s / p95 0.005 s；边界排除 86/159 样本已报告）；`frame_reference` = unknown（非 blocking））；**`V-2 verdict = PASS`**（四判据全 confirmed；`pose_frame_semantics = model_relative_for_links`；`composition_evidence`（R8）显式 `composition_status = UNVERIFIED`，验证未声称）
- **`PROBE_EXIT = 0`**（REV1.3 判据下首次双 PASS）
- run 7 归档：`V1/V2_report_REV13.run7_discovery_skew.json`——REV1.3 判据首跑中 `timestamp_association_compatibility` 因 rclpy/DDS 订阅发现延迟（run 6 实测 1.8 s、run 7 实测 4.5 s，非确定）对 joint 订阅生效前的 contact stamp 计算 nearest-neighbor（p95 3.495 s）而误判 contradicted；该 run 促成 common-window 裁剪实现（见 §5.3）后复跑通过

**REV1.2 历史（run 6，成品）：**

```
python3 tools/stage1_probe.py --probe both --duration 20.0 --warmup-sec 30.0 \
    --ros-domain-id 88 --gz-partition stage1probe6 --report-suffix REV12
```

- 环境：ROS 2 Jazzy、`ROS_DOMAIN_ID=88`、`GZ_PARTITION=stage1probe6`；schema `stage1-probe/1.2`
- 报告 pinning：`git_commit = 9f64d012d9ea…`；`rfc.sha256 = f006c0f22370…`（REV1.2）
- 终态：`V-1 verdict = FAIL`（blocking: `population_cadence`（[50,150] 有效速率带随 gait 相位变化——REV1.3/R6 已改事件语义）、`publication_mode`（event-only 无 no-contact 独立信号——REV1.3/R7 两态合同下不再失败））；`V-2 verdict = PASS`

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

### 1.2 采集环境（run 8 / run 6 / run 3 仿真，与冻结 runner 等价）

probe 直接 launch `nao_sim.launch.py`，参数与冻结工件 [run_phase_a_episode.py](file:///home/liu/workspace/Nao_Program/nao_gazebo_ws/tools/run_phase_a_episode.py)（FROZEN_GAIT，L62-88）逐值一致（walk、gait_phase_mode 'explicit_support_swing'、step_length_m 0.02、step_frequency_hz 0.8、swing_height_m 0.008、double_support_ratio 0.3、leg_extension_m 0.24401、gait_blend_sec 1.5、sway_amplitude_rad 0.06、sway_sign -1、arm_* 全套、foot_pitch_swing_rad 0.1、command_rate_hz 100.0、start_delay_sec 0.5、stop_ramp_sec 1.5、hold_final_sec 1.0、actuator_p_gain 15.0、actuator_max_velocity 4.0、actuator_hold_on_start true），`phase4b_duration_sec:=60.0`、`phase4b_record_duration:=65.0`、`phase4b_evidence_dir:=/tmp/stage1_probe_run8_evidence`（run 6: `/tmp/stage1_probe_run6_evidence`；run 3: `/tmp/stage1_probe_run3_evidence`）。run 8 仿真日志 `/tmp/stage1_probe_sim8.log`；各 run 时序：模型就绪后 gait 启动，contact 交替 ≥8 次时启动 probe（in-gait 窗口）；run 8 phase4b_summary：114 次交替、净位移 0.442 m、body z 全程 ≥0.330（无跌倒）。run 6 phase4b_summary：142 次交替、净位移 0.531 m、gait 末段两足 contact=false。

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

### 3.1 [PINNED] 发布模式：门控型（event-only；REV1.3/R7 下合同一致，非失败）

- 实测：行走窗口内抬脚期间 gz 路径 **`n_empty_messages = 0`**——无接触时 gz **不发布消息**（既非空消息也非零值消息）；gait 末段（stop ramp + hold）同样完全静默（run 3/4 复证）
- 分类：`publication_mode = 'irregular_or_event_only'`，`no_contact_observability = 'not_observable'`
- **REV1.3/R7**：Stage 1 合同为**两态**（`VALID_CONTACT / UNKNOWN_NO_MESSAGE`），event-only 发布是**合同一致**行为，不再是 blocking 失败——run 8 `publication_mode = confirmed`
- 附带：消息内字段填充率全 1.0（positions/normals/depths/world/wrenches/body1&2 force/force_offset）——**有消息即有完整 contact**

### 3.2 [RULE] no-contact 有效性语义（REV1.3/R7 两态合同；原 [BLOCKED] 已解除）

**[RULE]** collector 在沉默窗口（该 topic 无消息）**必须发 `UNKNOWN_NO_MESSAGE`（两态合同，REV1.3/R7）**：**无消息永远不等于零 wrench**（no zero wrench from silence）、**不得伪造 flight 标签**（no fabricated flight labels）、**不得伪造 no-contact 状态**（no fabricated no-contact state）。禁止把沉默序列化为 `valid=true` 的零 wrench（RFC §3.6.4；GPT-5.6 correction P1-R2）。

**[RULE]** `VALID_NO_CONTACT` 仅为**未来扩展**（future extension），**Stage 1 acquisition 不需要、也不得发**：若未来 RFC 激活，其成立形式有且仅有两种（显式空 Contacts 消息 / 经验证的独立 no-contact 源）。原 [BLOCKED] 项由 REV1.3/R7 解除（RFC §3.6.4 已按两态改写）。**No force-free fallback**（RFC App. B 原文）。

### 3.3 [PINNED] 节律（event-aware，REV1.3/R6）

- **[RULE→已执行]** REV1.3/R6：绝对有效速率带 [50,150] Hz **已删除**（有效速率 = duty × burst 率，随 gait 相位变化：run 3 实测 136.05/128.90 Hz、run 6 实测 181.85/157.72 Hz、run 2 站立特例 1000 Hz——同参数不同窗口，判据不可用）。判据改为**事件语义**：只评估含 contact 消息的区间；沉默区间计为非失败
- run 8 实测（REV1.3 判据）：左 28 bursts / 右 24 bursts；`median_event_dt` 0.001 s（1 kHz 物理步突发，两足同值）；`p95_event_dt` 0.001 s；`max_event_dt` 0.040/0.038 s；`active_contact_event_rate` 898.4/937.4 Hz；burst 时长 median 0.0595/0.0585 s；silence 段 27/23（**计为非失败**）
- **[RULE]** silence 语义仍为 `UNKNOWN_NO_MESSAGE`（§3.2）；collector 不得以固定速率假设做丢包检测（沉默 ≠ 丢包）

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

### 3.7 [PINNED] 时间戳关联兼容性（REV1.3/R10 判据下 confirmed）

- contact 源 stamp 与 `/joint_states` **可关联（association-compatible）**：判据为**源 stamp 的 nearest-neighbor 统计，在共同订阅窗口上评估**（REV1.3/R10）：run 8 实测 median 0.003 s / p95 0.005 s / max 0.005 s（两足同值）；边界排除样本（joint 订阅生效前收到的 contact stamp：左 86 / 右 159）计数并报告，从不静默丢弃；stamp 单调（0 backwards、0 duplicates）
- **措辞更正（REV1.3/R10）**：证据证明的是 **source stamps 可关联**（associable），**不证明** universal clock identity——判据名由 `timestamp_semantics` 改为 `timestamp_association_compatibility`
- ROS bridge **精确保 stamp**：`fraction_ros_stamps_exact_in_gz = 1.0`（双足，run 3/6/8 一致）
- **REV1.2/R2 更正（保留）**：envelope-overlap（订阅捕获窗口重叠）**不是判据**（REV1.1 的 `fraction_within_joint_envelope` 检查已从 probe 与 RFC 删除）——它是 probe 侧订阅发现 skew 的 instrumentation artifact（见 §5.3）

---

## 4. Pose stream contract（V-2 pinned）

### 4.1 [PINNED] 实体清单与命名

- 命名约定 `bare_names`：条目名 = SDF 中的裸 link 名（无 scope 前缀）
- 25 link 全部出现（`fraction_all_25_links = 1.0`，逐条 exact string match），含 `base_link`、`l_ankle`、`r_ankle`
- 消息还含 `*_visual` 条目（SDF visual 元素也出现在 pose/info）——**[RULE]** collector 按 25-link pinned 清单精确选名，忽略 visual 条目
- 单消息 25-link 完整率 1.0 → 无需跨消息拼接

### 4.2 [PINNED] 帧语义（REV1.2/R4 已入 RFC 合同；REV1.3/R8 组合证据显式化）

- **link entries = parent-frame（model-relative）位姿**，**不是**世界位姿：`link_entry_frame_semantics = 'model_relative_for_links'`（run 3/6/8 一致：`base_link_identity_fraction = 1.0`、`nao_entry_identity_fraction = 0.0`、`l_ankle_raw_below_ground_fraction = 1.0`）
- RFC REV1.2/R4 已按实测事实改写合同：Pose_V 条目为 parent/model-relative、禁止当世界位姿入库、世界位姿必须经 composition。run 6/8 `pose_frame_semantics = confirmed`（`V-2 verdict = PASS`）
- `/body_pose` 交叉验证确认：**nao 模型条目 = 世界位姿锚点**（run 8：exact-stamp 匹配 fraction 1.0、位置偏差 0.0、frame_id 全为 `world`）

### 4.3 [PINNED] 世界位姿恢复公式（composition；REV1.3/R8 composition_status = UNVERIFIED）

```
p_world = p_model + R(q_model) · p_rel
q_world = q_model ⊗ q_rel          # Hamilton 积；四元数分量序 [x, y, z, w]
```

其中 `p_model/q_model` 取**同一 Pose_V 消息**的 `nao` 模型条目，`p_rel/q_rel` 取该 link 条目。

- 一致性检查：`composed_base_link_vs_nao_entry_max_diff_m = 0.0`（run 3: 1293/1293；run 6: 1336/1336；run 8: 全部消息）——**但这是 identity-link 一致性检查，不是端到端验证**（REV1.2/R5：base_link IS 模型帧，合成公式的旋转项从未被该比较检验）
- **REV1.3/R8 composition_evidence**：probe 对 ≥2 个 non-identity link（`l_ankle`、`r_ankle`）按 `T_world_model × T_model_link` 报告组合证据，并给出显式 **`composition_status`**——当前 pipeline 无独立 non-identity-link 世界位姿源（`/body_pose` 仅覆盖 identity link；`dynamic_pose/info` 实测为 parent-relative，非世界位姿源；站立几何仅为包络 sanity）→ **`composition_status = UNVERIFIED`，验证未被声称**
- **[RULE]** collector 记录世界位姿**必须**走此合成路径（RFC REV1.3 §3.3/§3.4/§3.5），**禁止**把 link 条目裸值当世界位姿写库；raw Pose_V 条目与模型位姿照常存储
- **[RULE]** 合成的 foot/CoM 字段在 composition_status = UNVERIFIED 期间一律标 **`derived_pending_validation`**（R8/R9），不得呈现为 validated 世界位姿；待独立 non-identity-link 位姿源提供验证证据后方可升格

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

### 4.7 [RULE] pending-validation 传播（REV1.4/R11-A）

**依赖规则（binding，与 RFC §3.0 R11-A 逐字一致）：** 任何 derived field，其计算**直接或间接**依赖于 validation status 为 `UNVERIFIED`（composition）或 pending/blocked（wrench frame/reference）的输入，**必须**标记 `derived_pending_validation`（或按 §3.6.4/§3.7 保持 `null`+blocked），**禁止**表示为已验证的 ordinary derived data。pending 状态沿依赖链传播，仅当根输入状态改变时解除（未来独立 world-pose reference / V-1 公共坐标系+矩心 determination）。

**Null-vs-Pending 实现选择（与当前 schema 一致，R11-A codified）：**

- **Option A（取值保留）**：composition 依赖字段——`left/right_foot_pose`、`left/right_foot_velocity`、`com_position`、`com_velocity`、`com_acceleration`、`zmp_position`——数值照常计算落盘，字段标 `derived_pending_validation`
- **Option B（取值 null + blocked）**：wrench frame/reference 依赖字段——`left/right_contact_force`、`left/right_contact_moment`、`cop_position`——`null` 直至 V-1 determination（§3.6.4）

**传播链（collector 必须按此落盘状态）：**

```
raw model pose + raw relative link pose
        ↓  (composition, status = UNVERIFIED)
left/right_foot_pose        derived_pending_validation
        ↓
left/right_foot_velocity    derived_pending_validation

raw model/link poses
        ↓  (composition, status = UNVERIFIED)
com_position                derived_pending_validation
        ↓
com_velocity                derived_pending_validation
        ↓
com_acceleration            derived_pending_validation
zmp_position                derived_pending_validation   (CoM chain)

pose + wrench
        ↓  (wrench frame/reference = UNKNOWN)
cop_position / world-frame moment   null + blocked (Option B)
```

**非传播规则：** raw measured data（raw `Pose_V` 条目、model pose、raw `Contacts`、joint states、IMU、command）**不因** composition 未验证而失效——照常 verbatim 存储为 `measured`。禁止的只是：把依赖未验证 composition（或未知 wrench 解释）的 derived quantity 当成 validated quantity。

**[RULE] 机器可执行 enforcement：** collector schema manifest 与 smoke validation 必须通过 `tools/stage1_contract_check.py` 检查（字段→状态映射、传播规则、raw 不受影响）；collector 自行重算 `T_world_link = T_world_model × T_model_link` 只验证**实现一致性**，**不得**据此把 `composition_status` 从 `UNVERIFIED` 升级为 `VALIDATED`（物理正确性未被证明）。

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

### 5.3 [PINNED] 订阅发现 skew 与共同窗口裁剪（REV1.3 已实现）

- 实测（run 3，REV1.1 判据下）：V-1 中 contact sim span 18.868 s vs joint 包络 [40.065, 56.615]（16.55 s）→ fraction 0.861/0.868；V-2 中 joint sim span 15.03 s vs pose span 15.885 s（0.855 s 偏差）
- 根因（已判定，非猜测）：**probe 侧订阅启动 skew**。gz 订阅即时生效；rclpy/DDS 订阅发现延迟**非确定**（run 6 实测 1.798 s、run 7 实测 4.493 s）。两条流同端点截止（快照时刻），各自起点不同——run 7 中 joint 订阅生效前的 contact stamp 使 nearest-neighbor p95 达 3.495 s（归档 `*.run7_discovery_skew.json`），**该量是 discovery skew，不是不可关联证据**
- joint 流本身无间隙（dt 恒 0.01 s、0 backwards）；共同窗口内 nearest-stamp 偏差实测 3 ms（run 3）/ p95 5 ms（run 6/8）证明可关联
- **[RULE→已执行]** REV1.3：envelope-overlap **不得**用作判定（probe 与 RFC 均已删除）；nearest-neighbor 统计**在共同订阅窗口上评估**，joint 窗口外的 contact stamp **排除并计数报告**（run 8：左 86 / 右 159），从不静默丢弃。时钟域判定用 nearest-stamp 统计（median/p95/max）或先裁剪到共同订阅窗口再比较——REV1.3 probe 已按后者实现，run 8 `timestamp_association_compatibility = confirmed`

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

## 7. Validity 语义（两态合同，绑定；REV1.3/R7）

| contact 状态 | 触发条件 | 采集期可用性 |
|---|---|---|
| `VALID_CONTACT` | 该足 topic 有非空 Contacts 消息且关联进 0.05 s 窗口 | 可用（字段填充率 1.0） |
| `UNKNOWN_NO_MESSAGE` | 窗口内无消息（沉默） | **默认态**（门控发布）；**无消息永远不等于零 wrench、不得伪造 flight 标签、不得伪造 no-contact 状态** |
| `VALID_NO_CONTACT` | （**future extension，非 Stage 1 合同**）显式空 Contacts 消息或经验证的独立 no-contact 源 | **不发**（REV1.3/R7：Stage 1 acquisition 不需要） |

flight 检测、1.0 N 阈值等 REV1.0 条款在两态合同下**只能**作用于 `VALID_CONTACT` 帧的实测 wrench（作为推导规则）；沉默帧（`UNKNOWN_NO_MESSAGE`）一律不判定为 flight、不得伪造任何 no-contact/零 wrench 状态（fail-closed）。

---

## 8. Provenance 与完整性（绑定，R11-B：RFC §3.10 为唯一 source of truth）

**[RULE] Source of truth（REV1.4/R11-B；R12-C 机器双绑定）：** [RFC §3.10](file:///home/liu/workspace/Nao_Program/nao_gazebo_ws/docs/STAGE1_DYNAMICS_DATASET_RFC.md) 的 mandatory provenance 表是 collector provenance 完整性的**唯一权威定义**。**collector MUST 发射 §3.10 的每一个 field**；本节 checklist 与 §10 smoke validation 均**derived from / must equal** RFC §3.10——任何"简化版 provenance list"（RFC says A+B+C+D…、collector 只实现 A+B+C）都是 contract drift，判定为验证失败。机器可执行 enforcement（R12-C）：`tools/stage1_contract_check.py` 的 `MANDATORY_PROVENANCE_FIELDS` 通过**同时解析 RFC §3.10 表与本 §8 checklist 表**绑定，并要求三者优先级相等（RFC == Contract §8 == checker list）——**任一文档增删 field 均为 FAIL**（mutation 测试见 `tools/test_stage1_r12.py`），不再仅是两份独立声明的一致性口头声明。

**[RULE] RFC §3.10 完整 checklist（derived，必须与 §3.10 相等；以 RFC 为准）：**

| # | Field | 约束 |
|---|---|---|
| 1 | `theta` | 完整参数 dict（冻结 θ list 条目 verbatim） |
| 2 | `theta_id` | per-θ id |
| 3 | `theta_hash` | per-θ canonical JSON hash（Stage 0 method） |
| 4 | `theta_list_hash` | `844646f5…`（每 episode 前 fail-closed 复验） |
| 5 | `git_commit` | 采集时 collector HEAD（含冻结工件 hash 锚） |
| 6 | `config_hash` | collector 配置 + 对齐常数（contact 0.05 s / pose 0.02 s、composition 公式标识）——**与 CoP 公式解耦**（RFC P0-R5/C4） |
| 7 | `cop_derivation_hash` | `null` 直至 first derivation 前固定（允许 null；field 必须存在） |
| 8 | `gazebo_seed` | 该 (θ, seed) 的 seed（继承 Stage 0 run_config） |
| 9 | `robot_model_version` | `nao_simple.sdf` sha256 `44ed8285…` |
| 10 | `world_model_version` | `nao_world.sdf` sha256 `383341d1…` |
| 11 | `gz_sim_version` | `8.15.0`（记录，不 pin） |
| 12 | `stage1_rfc_hash` | 采集时 RFC revision 的 sha256 |
| 13 | `acquisition_wallclock_utc` | wall-clock UTC（与 `sim_time_sec` 时域分离） |
| 14 | `ring` | sampling ring label（§4） |
| 15 | `frame_count` | episode 帧数 |
| 16 | `t_start` | episode 起点 sim time |
| 17 | `t_end` | episode 终点 sim time |
| 18 | `episode_integrity_hash` | episode canonical sample file 的 sha256（**hash scope 与算法维持 RFC 原定义，R11 不改**） |

**[RULE] 实施附加项（非 RFC §3.10 替代、不与之冲突）：** probe 证据 pin（V1/V2 报告路径 + 内容 hash，RFC App. B pinning 条款：Level 3 要求两份报告 committed）；环境记录（ROS_DISTRO、gz 版本、RMW、ROS_DOMAIN_ID、GZ_PARTITION、实测 RTF 区间）；输出文件 hash 校验（BF-3 模式，写后校验）。

**[RULE] 完整性（§11 原则，R11 不重设计）：** `episode_integrity_hash` 的 hash scope/算法按 RFC §3.10 原定义执行（sha256 over episode canonical sample file）；R11 只要求 provenance 字段全量存在并通过 smoke 检查，**不改变** hash 定义。

---

## 9. Collector implementation boundary（GLM §6，重申）

**YES**：新增 subscriber（gz native contact/pose + ROS 话题）、新增 writer 进程、新增 `dataset/stage1/` 布局、新增 tools/ 脚本、计算 RFC 已 pin 的 derived 字段、运行时新增 ros_gz_bridge 实例、向现有 launch 文件传参。

**NO**：修改 `tools/run_phase_a_episode.py`、`tools/run_p0a_gait_candidate.py`、`src/nao_gz_sim/**`（controller/gait generator/launch/converter 内容）、`nao_simple.sdf`、`nao_world.sdf`、physics/simulation behavior、写入 `dataset/stage0/`、重跑 Stage0、训练/优化/LIPM fitting。

---

## 10. Collector smoke validation（实现验收检查，绑定；R11 升级）

collector 实现完成后、任何 pilot episode 前必须通过（GPT-5.6 minimum gate set 第 3 条）：

1. **无损往返**：序列化的 raw Contacts/JointWrench 反序列化后与订阅回调内存中的消息逐字段一致（含 header、world、force_offset、双侧 wrench）
2. **stamp 保真**：落盘 stamp 与源 stamp 精确相等（对照 probe 实测 bridge 保真率 1.0 的基线）
3. **两态行为（REV1.3/R7）**：构造沉默窗口（无 contact 消息）→ 落盘必须为 `UNKNOWN_NO_MESSAGE`，不得出现 `valid=true` 零 wrench、不得伪造 flight/no-contact 标签
4. **合成实现一致性**：落盘世界位姿与同消息 composition 计算值偏差 ≤ 1e-6 m；link 原始值（parent-relative）独立列存。**R11-A 限定：此项只验证实现一致性，不得据此把 `composition_status` 从 `UNVERIFIED` 升级**
5. **命名匹配**：关联过滤对 verbatim pair（§3.4）命中 fraction = 1.0
6. **provenance 完整（R11-B 升级）**：**RFC §3.10 的每一个 mandatory field** 逐项存在并通过 `tools/stage1_contract_check.py` 的 provenance check（缺失任一 field → FAIL）+ 输出 hash 校验通过——不再以 §8 的简化摘录为准
7. **逐帧 flag**：pose 窗口 miss flag 与 contact `UNKNOWN_NO_MESSAGE` flag 按帧落盘
8. **pending-validation 传播（R11-A）**：schema manifest 与样本帧通过 `tools/stage1_contract_check.py` 的 propagation check——依赖 `composition_status = UNVERIFIED` 的每个 derived field（`left/right_foot_pose/velocity`、`com_position/velocity/acceleration`、`zmp_position`）必须标 `derived_pending_validation`（不得为 plain `derived` 或 validated）；wrench 依赖字段（`left/right_contact_force/moment`、`cop_position`）必须 `null`+blocked；raw measured 字段不受影响照常存储

对应最小测试：`tools/test_stage1_r11.py`（Test A pending 传播、Test B raw 不受影响、Test C 缺失 provenance → FAIL、Test D 完整 provenance → PASS）。

**R12-A smoke 入口（fail-closed，单一链路）：**

```
python3 tools/stage1_contract_check.py --smoke \
    --manifest <status_manifest.json> \
    --provenance <episode_provenance.json> \
    --frame <sample_frame.json> \
    --raw <raw_records.json>
```

链路：RFC §3.10 == Contract §8 == checker（R12-C 双向解析绑定）→ raw 类别绑定（Contract §6 tokens）→ provenance 完整性 + 格式 → status manifest allowlists（R12-B）→ frame Option-B（存在且为 null）→ raw 结构契约（R12-D）→ 总 PASS/FAIL。**任一项失败 → `ok=false` 且进程 exit != 0**；缺失 artifact 文件/缺失 manifest 是 FAIL，不是"没有东西可检查"。对抗性负测 + RFC/Contract mutation 测试 + CLI exit-code 测试：`tools/test_stage1_r12.py`（R12-D）。

---

## 11. RFC 修订前置清单（Level 3 unblock 条件）

以下修订源自 probe 实测（不 widening、不 silent pass）。**REV1.3 已完成 R1（两态合同，R7）、R2（事件语义 cadence，R6）、R3（association compatibility + common-window 裁剪，R10）、R4（frame semantics 入合同，REV1.2/R4），R5 部分（composition 显式 UNVERIFIED，R8）**；完成项与剩余项均需经 final independent audit 后 Level 3 方可重新评估：

| # | 源 | 修订内容 | REV1.3 状态 |
|---|---|---|---|
| R1 | V-1 publication_mode（blocking） | no-contact 不可观测：增设经验证的独立 no-contact 信号（候选：contact_converter boolean），或明确 contact 状态限两态（`VALID_CONTACT`/`UNKNOWN_NO_MESSAGE`）。App. B V-1 失败条款已预设此路径 | **完成**（REV1.3/R7 选择两态路径）：Stage 1 合同限 `VALID_CONTACT / UNKNOWN_NO_MESSAGE`；`VALID_NO_CONTACT` 为 future extension；event-only 发布合同一致、非失败。run 8 `publication_mode = confirmed` |
| R2 | V-1 population_cadence 语义 | cadence 判据改写为门控语义（接触活跃期有效速率带 + 沉默不发消息），并记录 run 2 持续接触 1000 Hz 特例 | **完成**（REV1.3/R6）：判据改为 event-aware publication validation（burst 事件语义、silence 非失败、五指标报告）；run 8 `contact_publication_events = confirmed` |
| R3 | V-1 timestamp_semantics 判据方法 | envelope-overlap 换为 nearest-stamp 域检查（实测 3 ms）或共同窗口裁剪；记录 §5.3 instrumentation limitation | **完成**（REV1.2/R2 + REV1.3/R10 + common-window 裁剪）：判据为 `timestamp_association_compatibility`，共同窗口 nearest-neighbor 统计（run 8 median 0.003 / p95 0.005 s）；run 7 discovery-skew 边界已由裁剪解决 |
| R4 | V-2 world_pose_semantics（blocking） | App. B V-2.2 "each entry is the world pose" 改为 parent-relative 事实 + 强制 composition（公式与验证数值入 RFC） | **完成**（REV1.2/R4）：RFC 已按实测事实改写；run 6/8 `pose_frame_semantics = confirmed`，V-2 PASS |
| R5 | V-1 frame_reference = unknown | §3.6 wrench 参考点/轴维持"未 pin"状态声明；CoP/derived moment 推导前置条件 = 公共坐标系 + 公共矩心对比（新增小 probe 或文档化检查），在此之前 CoP 后处理标记 blocked | **部分完成**：wrench frame/reference 仍 unknown（非 blocking，CoP/derived-moment 推导维持 blocked）；composition 侧已由 REV1.3/R8 显式化为 `composition_status = UNVERIFIED`（foot/CoM 字段 `derived_pending_validation`），待独立 non-identity-link 位姿源 |
| R6 | App. B pinning | 两份 probe 报告 path + hash 写入 RFC pinning 条款（Level 3 要求 committed——commit 动作待用户明确指示） | **报告完成、pinning 部分**：REV13 报告（run 8）已存在并记录 git commit + config hash（本 session 已 commit）；remote publication（GPT-5.6 correction 8）仍为 owner 动作 |

---

## 12. 终态判定引用

**REV1.3 证据（run 8，in-gait 窗口，REV13 报告，schema `stage1-probe/1.3`）：**

- V-1：`verdict = PASS`，blocking = `[]`；confirmed = **contact_publication_events（R6 事件语义）**、body_identity、**publication_mode（R7 两态合同，event-only 合同一致）**、**timestamp_association_compatibility（R10：共同窗口 nearest-neighbor median 0.003 s / p95 0.005 s，边界排除样本已报告）**；unknown（非 blocking）= frame_reference
- V-2：`verdict = PASS`，blocking = `[]`；confirmed = unique_entities、**pose_frame_semantics（`model_relative_for_links`）**、source_stamps_cadence、alignment_hit_rate（1.0）；**composition_evidence（R8）显式 `composition_status = UNVERIFIED`——验证未声称，foot/CoM 字段 `derived_pending_validation`**
- **`PROBE_EXIT = 0`（REV1.3 判据下双 PASS）**
- **结论：Level 3 acquisition 仍未授权**——probe 双 PASS 仅为判据证据；Level 3 仍需 RFC REV1.3 **final independent audit 接受**、报告 pinning 完整化（commit 已完成）与 remote publication（owner 动作）。collector implementation（Level 2）可在本合同约束下进行（§4.3 composition `derived_pending_validation` 约束生效）

**REV1.2 历史（run 6，REV12 报告，schema `stage1-probe/1.2`）：**

- V-1：`verdict = FAIL`，blocking = `['population_cadence', 'publication_mode']`（两项分别由 REV1.3/R6 与 REV1.3/R7 解决）；confirmed = body_identity、timestamp_semantics（REV1.2/R2 判据）；unknown（非 blocking）= frame_reference
- V-2：`verdict = PASS`；run 7 归档 `*.run7_discovery_skew.json`（DDS 发现延迟边界，§5.3，已由 common-window 裁剪解决）

**REV1.1 历史（run 3，schema `stage1-probe/1.1`，判定保留存档）：**

- V-1：`verdict = FAIL`，blocking = `['publication_mode', 'timestamp_semantics']`（后者为 envelope 判据 instrumentation artifact，REV1.2/R2 已删除该判据）；confirmed = population_cadence、body_identity；unknown（非 blocking）= frame_reference
- V-2：`verdict = FAIL`，blocking = `['world_pose_semantics']`（REV1.1 字面假设被 falsified——REV1.2/R4 已按实测事实改写合同）；confirmed = unique_entities、source_stamps_cadence、alignment_hit_rate

---

## Document status

| 版本 | 状态 |
|---|---|
| 1.0 | Level 1 preparation artifact，基于 probe run 3（schema `stage1-probe/1.1`）实测证据签发。随 RFC 修订（§11）升版 |
| 1.1 | REV1.2 升版：Governing RFC → REV1.2-CORRECTED（pending re-audit）；证据基新增 run 6 REV12 报告（`stage1-probe/1.2`）与 run 4 end-hold 归档；R1 语义命名（`UNKNOWN_NO_MESSAGE`）与 R3（timestamp 判据）、R4（frame semantics 入合同）落合同；R5 措辞更正（composition = identity-link 一致性检查，非端到端验证，需 non-identity-link 验证）；§6.1 声明 native gz raw capture 强制且 RFC source path 一致；§11 标注各修订项 REV1.2 状态；§12 终态换为 run 6 REV12 判定 |
| 1.2 | REV1.3 升版：Governing RFC → REV1.3-MINIMAL-HARDENING（ACQUISITION-CONTRACT-READY，pending final independent audit）；证据基新增 run 8 REV13 报告（`stage1-probe/1.3`，**V-1/V-2 双 PASS，PROBE_EXIT=0**）与 run 7 discovery-skew 归档；R6 事件语义 cadence、R7 两态 validity（§3.1/3.2/7，原 [BLOCKED] 解除）、R8 composition_evidence 显式 `composition_status = UNVERIFIED`（foot/CoM = `derived_pending_validation`，§4.3）、R10 association-compatibility 措辞 + common-window 裁剪（§3.7/5.3）落合同；§11 修订项 REV1.3 状态；§12 终态换为 run 8 REV13 判定 |
| 1.3 | REV1.4/R11 升版（GPT-5.6 audit of REV1.3，Decision B 的两个 blocker）：Governing RFC → REV1.4-HARDENING；**R11-A** pending-validation 传播规则 + null-vs-pending 实现选择 + 传播链 + 非传播规则（§4.7，机器可执行 via `tools/stage1_contract_check.py`）；**R11-B** RFC §3.10 为 provenance 唯一 source of truth，§8 重写为完整 18-field checklist（derived/must-equal），§10-6 升级为逐 field 检查 + §10-8 新增传播检查（Tests A–D：`tools/test_stage1_r11.py`）。无新 V-gate、无字段增删、R6–R10 不重开 |
| 1.4 | R12 fail-closed enforcement 升版（R11 Closure Audit Decision C 的执行层 blocker）：§8 声明 R12-C 双向机器绑定（同时解析 RFC §3.10 与本 §8 表，任一增删 field → FAIL）；§10 增补 R12-A 单一 fail-closed smoke CLI（`--smoke`，任一项失败 → exit != 0，缺失 artifact = FAIL）；R12-B status allowlist 化（unknown/unrecognized status、缺失 status/manifest/root declaration 均 FAIL）；R12-D raw 结构契约（已知类别 + 必需键 + required streams 绑定 Contract §6 tokens）。测试：`tools/test_stage1_r12.py`（40 cases：负测矩阵、mutation 测试、anti-circular fixtures、CLI exit codes、R6–R10 回归）+ `tools/test_stage1_r11.py`（fixture 按 R12 契约加强，15 cases 全过）。无 RFC 规则变化、无新 V-gate、probe 未动 |
