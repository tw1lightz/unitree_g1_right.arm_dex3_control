# Phase 3: Trajectory Smoothing & Validation - Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-05-13
**Phase:** 03-trajectory-smoothing-validation
**Areas discussed:** Time Parameterization, Validation Location & Behavior, Velocity Limit Source

---

## Time Parameterization Method

| Option | Description | Selected |
|--------|-------------|----------|
| 简单最大速度缩放 | dt = max(\|Δq_i\| / vel_limit_i) / velocity_scale. 无最小步长。 | |
| 简单最大速度 + 最小步长 | 同上，加 min_time_step 下限防止微小移动产生过短 dt。 | ✓ |
| 梯形速度曲线 | 加速-匀速-减速。需要加速度限制（URDF 没提供）。 | |
| 你来决定 | Claude 自行选择 | |

**User's choice:** 简单最大速度 + 最小步长
**Notes:** None

### velocity_scale 默认值

| Option | Description | Selected |
|--------|-------------|----------|
| 0.1 (10%) | 肩/肘 3.7 rad/s，腕 2.2 rad/s。较慢但安全。 | |
| 0.2 (20%) | 肩/肘 7.4 rad/s，腕 4.4 rad/s。运动适中。 | ✓ |
| 0.3 (30%) | 肩/肘 11.1 rad/s。需物理测试确认。 | |

**User's choice:** 0.2 (20%)

### min_time_step 默认值

| Option | Description | Selected |
|--------|-------------|----------|
| 0.02s (20ms) | executor 发布周期 5 倍余量。 | ✓ |
| 0.05s (50ms) | 与当前固定步长相同。 | |
| 你来决定 | Claude 选择合理值 | |

**User's choice:** 0.02s (20ms)

---

## Validation Location & Behavior

| Option | Description | Selected |
|--------|-------------|----------|
| 只在 planner 端 | planner 发布前验证，executor 保持现有 clamp。 | ✓ |
| 两端都验 | planner + executor 双重验证。 | |
| 只在 executor 端 | executor 收到后验证，planner 只生成。 | |

**User's choice:** 只在 planner 端

### 验证失败行为

| Option | Description | Selected |
|--------|-------------|----------|
| 硬拒绝 | RCLCPP_ERROR 日志，不发布。 | |
| 尝试修复 + 重验 | clamp 位置，拉伸 dt，重验后仍失败则拒绝。 | ✓ |

**User's choice:** 尝试修复 + 重验

---

## Velocity Limit Source

| Option | Description | Selected |
|--------|-------------|----------|
| 扩展 planner 的 joint_limits_ | 改为含 position + velocity 的结构体。 | |
| 新增单独的 velocity_limits_ map | 保持现有不变，另加 map。 | |
| 你来决定 | Claude 根据代码结构选择。 | ✓ |

**User's choice:** 你来决定

---

## Claude's Discretion

- Velocity limits storage approach in planner: extend existing struct vs. add separate map

## Deferred Ideas

None — discussion stayed within phase scope.
