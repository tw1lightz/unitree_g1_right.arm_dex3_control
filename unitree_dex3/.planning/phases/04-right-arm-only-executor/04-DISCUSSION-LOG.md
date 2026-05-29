# Phase 4: Right-Arm-Only Executor - Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-05-13
**Phase:** 04-right-arm-only-executor
**Areas discussed:** A. Hand 代码删除范围, B. 收到非右臂轨迹时如何处理

---

## A. Hand 代码删除范围

| Option | Description | Selected |
|--------|-------------|----------|
| 1. 彻底删除（推荐） | 与 Phase 1 删除左臂代码一致：删除 `left_hand_pub_` / `right_hand_pub_` 成员、构造函数里的 `create_publisher`、`is_left_hand` 探测代码块、开手 `sleep_for(1s)`、关手 `publish` 调用。后续需要 hand 控制时，重新打开一个 phase 添加。 | |
| 2. 最小删除 | 仅删除两个 `hand_cmd_pub->publish()` 调用（开手、关手），保留 publishers/探测/sleep 作为惰性代码。代码量变化小、可逆。但 `sleep_for(1s)` 在轨迹开始前空走 1 秒。 | ✓ |
| 3. 你定 | Claude 自主决定（会遵循 Phase 1 的"完全删除"风格，默认选项 1）。 | |

**User's choice:** 2. 最小删除
**Notes:** 用户明确接受空走的 1 s sleep；保留 publishers + 探测 + sleep 作为未来 DEX3 grasping phase 的入口。不加 `[[maybe_unused]]` 抑制警告。

---

## B. 收到非右臂轨迹时如何处理

### 问题 1：含非右臂关节时

| Option | Description | Selected |
|--------|-------------|----------|
| 1. 严格拒绝（推荐） | 检查 `joint_names`：如果任何关节名不在 `{right_shoulder_pitch, _roll, _yaw, right_elbow, right_wrist_roll, _pitch, _yaw}` 中→`RCLCPP_ERROR` 并 return，不发 `LowCmd`。防御深入，避免误写左臂。 | |
| 2. 静默过滤 | 循环里跳过非右臂关节，仅处理右臂那几个。不报错、不拒绝，上下游不中断。 | |
| 3. 不校验 | 完全信任 planner，轨迹有什么就写什么（依赖现有 `joint_name_to_index.at()` 在未知关节名时抛出 `std::out_of_range`）。 | |
| 4. 你定 | Claude 选推荐项 1（防御错误低成本，与 Phase 1「只管右臂」一致）。 | |

**User's choice:** Free-text — "跳过非右臂关节，仅处理右臂，同时警告含有非右臂关节"
**Notes:** 用户给出混合方案 — 静默过滤的行为 + WARN 警告。即不阻塞上下游，但留下日志痕迹。Claude 在 CONTEXT.md D-03 step 1 中记录为"WARN + strip"。

### 问题 2：右臂关节不全时

| Option | Description | Selected |
|--------|-------------|----------|
| 1. 拒绝，要求 7 关节都在 | `RCLCPP_ERROR` + return。与 Phase 1 planner 总是输出完整 7 轴一致，任何部分轨迹都是错误。 | ✓ |
| 2. 警告 + 执行包含部分 | `RCLCPP_WARN` 列出缺的关节名，但仍按轨迹里含有的部分右臂关节执行。未出现的右臂关节闷 `latest_joint_positions_`。 | |
| 3. 静默接受 | 不报警，那些未在轨迹中的右臂关节位置从 `latest_joint_positions_` 填充，和现有所有关节填充逻辑一致。 | |
| 4. 你定 | Claude 选推荐（项 1 拒绝，防止 partial trajectory 陆扣崩潰；planner 理应输出 7 轴完整）。 | |

**User's choice:** 1. 拒绝，要求 7 关节都在
**Notes:** 严格的完整性检查 — 7 个右臂关节缺任何一个都 `RCLCPP_ERROR + return`。配合问题 1 的"WARN + strip foreign"，整体验证逻辑：先剔除非右臂关节并 WARN，再检查右臂全集，缺失则 ERROR。

---

## Areas Considered But Not Discussed

- **C. master 开关在右臂模式下的语义** — 未被用户选择。CONTEXT.md D-06 标记为 Claude's discretion，要求 planner agent 研究 Unitree `arm_sdk` 文档以确认非右臂槽位（`mode=0`、`q=0` 默认）在 `kNotUsedJoint.q = 0.5` 时不会被混合到 body controller 输出里；如发现混合则需要提出修复方案并回到讨论。
- **D. Pre/post-trajectory 时序结构** — 未被用户选择。CONTEXT.md "Carried Forward" 段落明确：1 s hold + 3 s ramp + 250 Hz + master 0.5→0 曲线**不动**，原样保留 Plan 01-09 / 01-11 / 01-12 调出来的参数。

## Claude's Discretion

- **D-06**：master-switch 在右臂-only 写入下的语义需要由 planner agent 确认（Unitree firmware blending behavior with default-zero / mode=0 slots）。
- 三个 publish 循环（waypoint / hold / ramp）写入收窄到 7 关节的具体实现策略（hardcoded indices vs `joint_name_to_index` 查找 vs 新建 `kRightArmIndices[]` 数组）。
- WARN/ERROR 日志的具体格式与详细度。

## Deferred Ideas

- **未来重启 DEX3 hand 控制** — 因 D-01 选择"最小删除"，publishers / 探测 / sleep 仍在；future phase 直接重新加 `publish()` 调用即可。
- **Phase-1 风格的彻底删除** — 显式考虑后被否，留待未来清理 phase。
- **Master-switch 重新调参** — 若 D-06 研究发现现行 0.5/ramp 在 default-zero 槽位下不安全，可能要单独立项修复。
