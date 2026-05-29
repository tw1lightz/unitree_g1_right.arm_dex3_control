# Phase 1: Right-Arm-Only Planner - Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2025-04-28
**Phase:** 01-right-arm-only-planner
**Areas discussed:** 左臂代码处理, 碰撞检测的全身链接变换, URDF模型选择, 调试日志清理

---

## 左臂代码处理

| Option | Description | Selected |
|--------|-------------|----------|
| 完全删除 | 删除所有左臂代码（KDL链、IK/FK求解器、left_tip参数、左右选择逻辑）。代码更简洁，约减少60行。 | ✓ |
| 保留但禁用 | 保留左臂代码但通过参数或编译标志禁用。保留未来快速启用左臂的可能性。 | |
| 你来决定 | Claude根据项目约束和最佳实践自行判断。 | |

**User's choice:** 完全删除，根据y轴选择手臂的逻辑也要去掉
**Notes:** 用户明确要求删除基于y坐标的左右臂选择逻辑，始终使用右臂。

---

## 碰撞检测的全身链接变换

| Option | Description | Selected |
|--------|-------------|----------|
| 从TF树查询 | 通过robot_state_publisher已发布的TF树获取各链接变换。 | |
| KDL全树FK + 关节状态 | 用/joint_states的最新数据对KDL树做全身正运动学。 | |
| 静态URDF默认变换 | 对不动的链接使用URDF中的默认位置。 | |
| 你来决定 | Claude根据性能、简单性和安全性约束自行判断。 | ✓ |

**User's choice:** 你来决定
**Notes:** 用户将此技术实现决策委托给Claude。

---

## URDF模型选择

| Option | Description | Selected |
|--------|-------------|----------|
| collision_primitives版本 | g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf — 碰撞体用简单几何基元。FCL检查最快。 | ✓ |
| 默认mesh版本 | g1_29dof_lock_waist_with_hand_rev_1_0.urdf — 碰撞体用完整网格模型。更精确但更慢。 | |
| 你来决定 | Claude根据性能和精度需求自行判断。 | |

**User's choice:** collision_primitives版本
**Notes:** 与初始化阶段的推荐一致，现正式锁定。

---

## 调试日志清理

| Option | Description | Selected |
|--------|-------------|----------|
| 本阶段清理 | 删除冗余调试日志，保留关键信息日志（初始化成功、规划开始/完成、错误）。 | ✓ |
| 降级为DEBUG | 保留所有日志但把详细输出降为RCLCPP_DEBUG。运行时默认不显示。 | |
| 推迟到后续 | 本阶段只关注功能修改，日志清理留给将来。 | |
| 你来决定 | Claude自行判断哪些该留、哪些该删、哪些该降级。 | |

**User's choice:** 本阶段清理
**Notes:** 用户查看了日志分布位置后决定在本阶段清理。

## Claude's Discretion

- 碰撞检测中非规划链接的世界坐标变换获取方式

## Deferred Ideas

None
