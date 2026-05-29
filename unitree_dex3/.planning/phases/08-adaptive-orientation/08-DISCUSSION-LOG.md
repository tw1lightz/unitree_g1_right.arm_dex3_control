# Phase 8: 自适应末端位姿 - Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-05-18
**Phase:** 8-自适应末端位姿
**Areas discussed:** 姿态策略, 肩部参考点, 输入覆盖规则, 验证方式

---

## 姿态策略

| Question | Options considered | User's choice |
|----------|--------------------|---------------|
| 自适应 orientation 中，TCP 哪个轴应该指向目标？ | +X 指向目标（推荐）; -Z 指向目标; 由你决定 | +X 指向目标 |
| 当 +X 已经指向目标时，绕 +X 的 roll 怎么定？ | 尽量保持 torso +Z 向上（推荐）; 尽量保留当前手腕 roll; 不强约束 roll | 尽量保持 torso +Z 向上 |
| 目标在肩部正上方/正下方时，up reference 退化怎么办？ | 切换到 torso +Y fallback（推荐）; 使用当前 TCP 姿态 fallback; 只记录为边界风险 | 切换到 torso +Y fallback |
| Phase 08 是否只生成一个确定性 orientation？ | 只生成单一 orientation（推荐）; 生成少量 roll 候选; 由 planner 自行决定 | 只生成单一 orientation |

**Notes:** User selected a deterministic `right_tcp_link +X` approach-axis strategy with `torso +Z` up reference and `torso +Y` fallback.

---

## 肩部参考点

| Question | Options considered | User's choice |
|----------|--------------------|---------------|
| “右肩参考点”应该怎么定义？ | 使用 right_shoulder_pitch_link（推荐）; 硬编码 torso 固定偏移; 使用 KDL 链首段自动推导 | 使用 right_shoulder_pitch_link 原点 |
| planner 应该怎样获取 right_shoulder_pitch_link 在 torso_link 下的位置？ | 从 KDL/URDF 链计算（推荐）; 从 TF lookup 获取; 启动时解析固定 origin | 从 KDL/URDF 链计算 |
| right_shoulder_pitch_link 是否需要做成 ROS 参数？ | 做成参数，默认 right_shoulder_pitch_link（推荐）; 直接硬编码 link 名; 由 planner 计划阶段决定 | 直接硬编码 link 名 |
| 如果目标点几乎等于肩部点，方向长度太小，怎么处理？ | 报错并拒绝该 goal（推荐）; 沿用输入 orientation; 使用默认前向方向 | 报错并拒绝该 goal |

**Notes:** User asked what the shoulder reference point does. It was explained as the origin used to compute `normalize(target - shoulder)`, which then determines the adaptive TCP approach direction.

---

## 输入覆盖规则

| Question | Options considered | User's choice |
|----------|--------------------|---------------|
| Phase 08 默认是否覆盖 /goal_pose 里的 orientation？ | 总是自动覆盖（推荐）; 仅当 orientation 无效/默认时覆盖; 由参数开关控制默认关闭 | 总是自动覆盖 |
| 是否需要保留参数来关闭自适应 orientation？ | 需要，默认开启（推荐）; 不需要，永远开启; 由实现者决定 | 需要，默认开启 |
| 当 adaptive_orientation_enabled=false 时，应该怎么处理 /goal_pose.orientation？ | 完全保留旧逻辑（推荐）; 只跳过覆盖但检查四元数有效性; 关闭时拒绝 goal | 完全保留旧逻辑 |
| 自动覆盖 orientation 时，需要打印哪些日志？ | 打印目标、肩部、方向和 quaternion（推荐）; 只打印启用/禁用状态; DEBUG 级别详细日志 | 打印目标、肩部、方向和 quaternion |

**Notes:** User asked what overwrite means and why it can happen. It was explained that `PoseStamped` always carries an orientation, old upstream sources may fill fixed/default quaternions, and Phase 08 needs the planner to replace those fixed poses by default.

---

## 验证方式

| Question | Options considered | User's choice |
|----------|--------------------|---------------|
| Phase 08 应该如何对比固定姿态 vs 自适应姿态？ | 固定测试点集 A/B 对比（推荐）; 现场随手测几个 AprilTag 点; 只做编译和日志验证 | 固定测试点集 A/B 对比 |
| 固定测试点集应覆盖哪些区域？ | 典型 + 边界 + 肩部正上方（推荐）; 只覆盖 AprilTag 常见桌面区域; 由实现者现场选择 | 只覆盖 AprilTag 常见桌面区域 |
| Phase 08 验证范围如何处理 AprilTag 常见区域与 roadmap 困难区域的冲突？ | 桌面必测 + 困难冒烟（推荐）; 只测桌面区域; 完整覆盖所有区域 | 只测桌面区域 |
| A/B 对比中，什么算“成功率明显提升”？ | 自适应成功数 > 固定姿态（推荐）; 自适应全部桌面点成功; 只要求日志显示姿态变化 | 自适应全部桌面点成功 |
| Phase 08 的验证要跑到哪一层？ | planner-only，不执行机器人（推荐）; planner + executor 实机执行; 先 planner-only，最后选 1 点实机执行 | planner-only，不执行机器人 |

**Notes:** User explicitly chose tabletop-only UAT despite roadmap wording about workspace boundary and shoulder-overhead areas. CONTEXT records this as a known verification gap/deferred item while keeping vertical fallback in implementation scope.

---

## Claude's Discretion

- Exact numeric thresholds for near-zero direction and near-parallel fallback.
- Exact C++ orthonormal basis construction.
- Exact A/B verification format, as long as planner-only tabletop success is clear.

## Deferred Ideas

- Multi-candidate orientation fallback belongs to Future ORI-02.
- Tag-normal-based approach direction belongs to Future ORI-03.
- Full workspace-boundary and shoulder-overhead UAT is deferred / recorded as a roadmap verification gap for this user-selected tabletop-only validation.
