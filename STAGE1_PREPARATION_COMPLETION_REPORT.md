# STAGE1_PREPARATION_COMPLETION_REPORT

| Field | Value |
|---|---|
| Role | Independent Implementation Agent + Adversarial Engineering Executor |
| Inputs | [GLM-5.3 审计报告](file:///home/liu/workspace/Nao_Program/nao_gazebo_ws/STAGE1_RFC_REV1.1_MINIMAL_REVISION_AND_PROBE_DESIGN_AUDIT_REPORT.md)（Verdict A）；[GPT-5.6 工程复审](file:///home/liu/workspace/Nao_Program/nao_gazebo_ws.worktrees/nao-gazebo-gait-review-stage0-stage1/STAGE1_GLM5.3_REVIEW_GPT5.6_REPORT.md)（Decision B — READY AFTER MINOR FIXES，8 项修正） |
| Scope executed | Stage 1 **preparation**（Level 1）：RFC 修订 + V-1/V-2 probe + collector contract。**非 acquisition**：未开始 96 episode 采集，未触碰任何冻结工件，未训练/优化 |
| Date | 2026-10-07 |

## Executive Summary

四项 Task 全部完成，Probe run 3（walk + FROZEN_GAIT 环境，与冻结 runner 逐参数一致）产出两份 fail-closed 报告：**V-1 FAIL**（blocking: `publication_mode`、`timestamp_semantics`）、**V-2 FAIL**（blocking: `world_pose_semantics`）。按 RFC App. B 与 GPT-5.6 §5 授权序列，失败不触发 silent widening，而是触发 RFC 修订前置。

> **最终结论：NOT AUTHORIZED FOR ACQUISITION。** Level 3（96 episodes）被 blocking failure 阻断，unblock 路径 = RFC 修订（6 项，见 §7）+ 修订后 probe 复跑通过 + 报告 pinning（含 commit，需用户明确指示）。

---

## Q1 — 授权范围内的四项 Task 是否全部完成？

**是（4/4），全部工件在位：**

| Task | 工件 | SHA-256（交付时点） |
|---|---|---|
| 1. RFC REV1.1 修订 | [docs/STAGE1_DYNAMICS_DATASET_RFC.md](file:///home/liu/workspace/Nao_Program/nao_gazebo_ws/docs/STAGE1_DYNAMICS_DATASET_RFC.md)（REV1.1-FINAL） | `6bd2e1ce05190de0384fa0f818f858ebc6f51b99f338fe6066dc90753621cb86` |
| 2/3. V-1/V-2 probe | [tools/stage1_probe.py](file:///home/liu/workspace/Nao_Program/nao_gazebo_ws/tools/stage1_probe.py)（schema `stage1-probe/1.1`） | `511ebada24072331f6f2f4cb68138e459d52f61978c5d9011eb6bfbc22bf16ce` |
| — run 3 证据 | [dataset/stage1/probe/V1_report.json](file:///home/liu/workspace/Nao_Program/nao_gazebo_ws/dataset/stage1/probe/V1_report.json) | `56885bcb2ba2fbd2061a71447cba5db2180200ea5a584703ac9bfde6027572cc` |
| — run 3 证据 | [dataset/stage1/probe/V2_report.json](file:///home/liu/workspace/Nao_Program/nao_gazebo_ws/dataset/stage1/probe/V2_report.json) | `69f5d484982a4c524dc23392a1ebaa08cab4945cc397f66ec9fb13f3478bfcf9` |
| 4. Collector contract | [docs/STAGE1_COLLECTOR_CONTRACT.md](file:///home/liu/workspace/Nao_Program/nao_gazebo_ws/docs/STAGE1_COLLECTOR_CONTRACT.md) | `889d82f090de4175471c4805dd962f6dc26d57b8607e9c119d0fd0bcb0c8bda2` |

历史 probe 运行归档（不作为判定依据）：`V1/V2_report.run1_fallen.json`（run 1 倒地）、`V1/V2_report.run2_stand_fallen.json`（run 2 stand 环境倒地）。

范围合规声明（未做清单）：未修改 `tools/run_phase_a_episode.py`、`tools/run_p0a_gait_candidate.py`、`src/nao_gz_sim/**`、`nao_simple.sdf`、`nao_world.sdf`、`dataset/stage0/`（只读）；未开始任何 episode 采集；未做 LIPM fitting / residual learning / training / optimization；未 commit / 未 push（RFC pinning 与 GPT-5.6 correction 8 均要求 commit/push 为 owner 动作，需用户明确指示）。

## Q2 — RFC REV1.1 是否落实 GPT-5.6 的 8 项修正？

**是——8 项中 6 项已直接落实入 RFC 文本，2 项按设计转为 probe 判定/合同条款（其 probe 依赖部分已由实测回填至 collector contract，RFC 侧修订因 blocking failure 列为前置项）：**

| # | GPT-5.6 修正 | 处置 |
|---|---|---|
| 1 | Authorization wording（FINAL = acquisition contract frozen；三级授权分离；V-1/V-2 结果 pin 后方可 bulk） | ✅ RFC §0.0 三级授权表 + REV1.1-FINAL 状态行 |
| 2 | Raw contact capture（无损 per-topic Contacts 保留；left/right 分开；derived 只增不换） | ✅ RFC §3.6.1（mandatory, lossless — REV1.1）+ 合同 §6（含实测 bridge 字段损失：header/world/force_offset） |
| 3 | V-1 semantics（运行时字段填充实测；公共坐标系/矩心后才可比 N3L；站立载荷仅 plausibility） | ✅ probe 判据按此实现（frame_reference = unknown 终态即此修正的 fail-closed 体现）；wrench 参考点维持未 pin → RFC 修订前置 R5 |
| 4 | Flight validity（event-only 沉默 ≠ 零 wrench；非独立信号不得确立 no-contact） | ✅ RFC §3.6.4 三态（REV1.1）+ 实测证实 publication 为门控型 → 修订前置 R1 |
| 5 | Time semantics（双时钟字段改名；actual-Δt 规则；源 stamp 保留；不插值） | ✅ RFC §3.1/§3.10（P0-R2/P0-R3）+ 合同 §5 |
| 6 | V-2 semantics（unique entities、world-pose 语义、源 stamp、逐帧命中率；/body_pose 仅 base 交叉验证） | ✅ probe 判据按此实现；**实测 falsify 字面 world-pose 假设** → 修订前置 R4（composition 公式已验证回填合同 §4.3） |
| 7 | Boundary（Stage0/runner/controller/model/world/physics 冻结；probe/collector 只读进程） | ✅ RFC §5.4/§7 + 合同 §9；执行期零违规（Q1 未做清单） |
| 8 | Remote durability（`045a21f` 与 RFC 发布先于首个 production episode） | ⏸ 未执行（push 为 owner 动作；不阻塞本地 preparation，与 GPT-5.6 判定一致） |

## Q3 — probe 是否按 RFC App. B 判据实现并执行？

**是。** `tools/stage1_probe.py` 按 determine-not-assume 原则实现五/四 criterion 判定，三态输出（confirmed/contradicted/unknown）、N3L 仅 plausibility（非 blocking）、退出码 0/1/2、报告 schema `stage1-probe/1.1`。执行序列（对抗性自检披露）：

| Run | 环境 | 结果 | 处置 |
|---|---|---|---|
| 1 | 默认（stand） | 机器人倒地 | 报告归档 `*.run1_fallen.json`，判不可用 |
| 2 | stand + producer 默认 leg_ext | 倒地 + **暴露 probe 三个 bug** | 报告归档 `*.run2_stand_fallen.json`；定位并修复三 bug（见下） |
| 3 | **walk + FROZEN_GAIT 全参数**（与冻结 runner 逐值一致）+ duration 60 sim s + GZ_PARTITION/ROS_DOMAIN_ID 隔离 | 机器人全程行走（116 次抬脚、sim 日志左右足交替）→ 采集 20 s 窗口 | **成品**：`V1_report.json` / `V2_report.json` |

Run 2 → run 3 修复的三个 probe bug（run 2 报告中的错误计数根因；修复后经 run 3 实证）：

1. V-2 计数 falsy-zero：`(_identity_deviation(b) or 1.0)` 把恒等位姿（deviation=0.0）误计为非恒等 → 显式 `None` 检查（修复后 `base_link_identity_fraction` 0.0→1.0）
2. V-2 分支 falsy-zero：`(nao_identity_frac or 1.0) <= 0.5` 使 `model_relative_for_links` 分支不可达 → `is not None` 检查（修复后 frame_semantics `'indeterminate'`→`'model_relative_for_links'`）
3. V-1 ground 匹配：`'ground' in local_name` 只搜局部名（实际局部名是 `collision`）→ 全 scoped 名搜索（修复后 `fraction_expected_pair` 0.0→1.0）

## Q4 — V-1/V-2 实测终态是什么？

**V-1 = FAIL**（`PROBE_EXIT=1`；blocking = `publication_mode`、`timestamp_semantics`）：

| Criterion | 状态 | 实测 |
|---|---|---|
| population_cadence | **confirmed** | 行走窗口有效速率 136.05/128.90 Hz（[50,150] 带内）；接触期 1 kHz burst、摆动 gap ≤1.06 s、duty ≈21% |
| body_identity | **confirmed** | fraction_expected_pair = 1.0（双足）；verbatim pair `nao::l_ankle::l_ankle_collision \| ground_plane::link::collision` 等；0 cross-contamination |
| publication_mode | **contradicted（blocking）** | **门控型**：116 次抬脚期间 gz 路径 `n_empty_messages = 0`——无接触时不发消息 → no-contact 不可观测，命中 RFC App. B V-1 失败条件（无独立 no-contact 信号） |
| timestamp_semantics | **contradicted（blocking）** | envelope overlap 0.861/0.868 < 0.95。**根因判定**（determined）：probe 侧 rclpy/DDS 订阅发现延迟（0.86–2.3 sim s，wall 2.796 s × RTF 0.827 ≈ 2.31 sim s，与包络起点差精确吻合；V-2 同签名 0.855 s）——**probe instrumentation limitation，非流属性**；时钟域本身一致（`median_abs_nearest_joint_dt_s = 0.003`、joint dt 恒 0.01 s 无间隙、bridge 保 stamp 1.0）。判定按 fail-closed 保留，归因更正写入合同 §5.3 与修订项 R3 |
| frame_reference | unknown（非 blocking） | N3L 样本级成立（body_2 = −body_1 逐分量精确），但按 GPT-5.6 correction 3 仅为 plausibility；wrench body name 实测空串 → 站立平衡检查 0 样本。CoP/derived-moment 维持 blocked |

**V-2 = FAIL**（blocking = `world_pose_semantics`）：

| Criterion | 状态 | 实测 |
|---|---|---|
| unique_entities | **confirmed** | bare_names 约定；25 link 全出现 fraction 1.0（含 base_link/l_ankle/r_ankle）；单消息完整率 1.0 |
| world_pose_semantics | **contradicted（blocking）** | **link entries = parent-relative（model-relative）**，RFC App. B V-2.2 字面假设被 falsified：base_link_identity_fraction 1.0（1293/1293）、nao_entry_identity_fraction 0.0（模型条目移动）、l_ankle 裸值 z<0 fraction 1.0。**恢复路径已端到端验证**：`p_world = p_model + R(q_model)·p_rel; q_world = q_model ⊗ q_rel`（[x,y,z,w]），composed vs `/body_pose`（nao 条目=世界锚点，1172/1172 exact-stamp）偏差 0.0 |
| source_stamps_cadence | **confirmed** | pose/info 81.33 Hz（dt median 0.012 s）、joint 100.0 Hz 精确、body_pose 82.05 Hz；全流 0 backwards/0 duplicates |
| alignment_hit_rate | **confirmed** | 0.02 s 因果 last-not-after 命中 1504/1504 = 1.0 |

附带发现：`dynamic_pose/info` 亦 parent-relative（`matches_raw_parent_relative`，n=1292）；站立几何判据在行走窗口呈机械伪影（r_ankle z=1.0 摆动相）——常量未改（RFC criterion 措辞含 "fixed standing geometry"，fail-closed）。

## Q5 — collector contract 是否固化全部实测语义？

**是。** [docs/STAGE1_COLLECTOR_CONTRACT.md](file:///home/liu/workspace/Nao_Program/nao_gazebo_ws/docs/STAGE1_COLLECTOR_CONTRACT.md) 以 [PINNED]/[RULE]/[BLOCKED] 三级条款固化：门控发布与三态有效性（`VALID_NO_CONTACT` 不可发 → 沉默必发 `UNKNOWN`）、verbatim collision 名关联过滤、wrench 体侧按 collision 实体名识别（body name 空）、N3L 仅样本级、双时钟字段与关联窗口（0.05 s/0.02 s last-not-after）、envelope 检查禁用（改 nearest-stamp）、gz native 无损采集（bridge 丢 header/world/force_offset 实测表）、composition 公式（0.0 偏差验证）、provenance/hash 要求、collector boundary（YES/NO）、smoke validation 7 项绑定检查。

## Q6 — Stage 1 acquisition（Level 3）是否授权？

**否。NOT AUTHORIZED FOR ACQUISITION。**

依据链：

1. RFC §0.0 Level 3 条款：需 V-1 pass **且** V-2 pass + 结果 pinned + remote publication；
2. RFC App. B 失败条款：V-1 失败 → "acquisition blocked until the RFC is revised. No force-free fallback"；V-2 失败 → RFC 修订前置；
3. 实测：V-1 FAIL（2 blocking）+ V-2 FAIL（1 blocking）→ GPT-5.6 §5 "Required before production acquisition" 的 4 条 bounded issues 中 3 条（frame/reference、event-only fail-closed、raw retention 路径）已由实测+合同闭合，但 no-contact 独立信号与 world-pose 假设两条**必须经 RFC 修订**方可闭合；
4. 未执行项：probe 报告 commit（RFC pinning 条款要求 committed）、remote push（GPT-5.6 correction 8）——均为 owner 动作，需用户明确指示。

**同时确认**：Level 2（collector implementation）在合同约束下可进行（RFC §0.0 授权范围内；两处 BLOCKED 条款按受限规则实现），Level 1 preparation 至此完成。

---

## 交付清单与 pinning

| 工件 | 位置 | SHA-256 |
|---|---|---|
| RFC REV1.1-FINAL | `docs/STAGE1_DYNAMICS_DATASET_RFC.md` | `6bd2e1ce05190de0384fa0f818f858ebc6f51b99f338fe6066dc90753621cb86` |
| Probe 脚本 | `tools/stage1_probe.py` | `511ebada24072331f6f2f4cb68138e459d52f61978c5d9011eb6bfbc22bf16ce` |
| V-1 报告（run 3 成品） | `dataset/stage1/probe/V1_report.json` | `56885bcb2ba2fbd2061a71447cba5db2180200ea5a584703ac9bfde6027572cc` |
| V-2 报告（run 3 成品） | `dataset/stage1/probe/V2_report.json` | `69f5d484982a4c524dc23392a1ebaa08cab4945cc397f66ec9fb13f3478bfcf9` |
| Collector contract | `docs/STAGE1_COLLECTOR_CONTRACT.md` | `889d82f090de4175471c4805dd962f6dc26d57b8607e9c119d0fd0bcb0c8bda2` |
| 归档（非判定依据） | `dataset/stage1/probe/*.run1_fallen.json`、`*.run2_stand_fallen.json` | —（报告内含自身 argv/provenance） |

## RFC 修订前置清单（Level 3 unblock 路径，详见合同 §11）

| # | 源 | 修订 |
|---|---|---|
| R1 | V-1 publication_mode | no-contact 不可观测 → 增设经验证的独立信号（候选 contact_converter boolean）或明确 contact 限两态（`VALID_CONTACT`/`UNKNOWN`） |
| R2 | V-1 cadence 语义 | 判据改门控语义（接触活跃期速率带 + 沉默不发消息）；记录持续接触 1000 Hz 特例 |
| R3 | V-1 timestamp 判据方法 | envelope-overlap → nearest-stamp（实测 3 ms）或共同窗口裁剪；记录 instrumentation limitation |
| R4 | V-2 world_pose_semantics | V-2.2 字面假设 → parent-relative 事实 + 强制 composition（公式+验证数值入 RFC） |
| R5 | V-1 frame_reference unknown | §3.6 wrench 参考点/轴维持未 pin；CoP 推导前置 = 公共坐标系+矩心对比 |
| R6 | App. B pinning | 两份报告 path+hash 入 RFC；Level 3 要求 committed（commit 待用户指示） |

修订完成后：复跑 probe（预计判据按修订版）→ V-1/V-2 pass → pinning（commit + push，owner 动作）→ Level 3 重新评估。
