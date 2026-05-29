# Phase 2: Path Simplification & Quality - Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-05-11
**Phase:** 02-path-simplification-quality
**Areas discussed:** 简化 API 选型, interpolate() 的去留, 简化超时限制, 简化无效时的日志行为, 路径可复现性

---

## 简化 API 选型

| Option | Description | Selected |
|--------|-------------|----------|
| `ss->simplifySolution()` | 一行调用，SimpleSetup 自动复用已有的状态检查器和空间信息，无需额外参数 | |
| `og::PathSimplifier` 手动构造 | 显式控制 maxSteps / maxEmptySteps，灵活性更高，代码多 5~10 行 | |
| 两种方案通过参数切换 | `simplify_method` 字符串参数，"simple" 或 "manual"，运行时可选 | ✓ |

**User's choice:** 两种方案都实现，通过 `simplify_method` 参数切换。
**Notes:** 用户希望在代码里改参数来决定用哪种，而不是只选一种。后续跟进选用 `simplify_method` 字符串参数（"simple"/"manual"），配套 `simplify_max_steps`（默认 100）和 `simplify_max_empty_steps`（默认 50）。

---

## interpolate() 的去留

| Option | Description | Selected |
|--------|-------------|----------|
| simplify 后保留 interpolate() | solve→simplify→interpolate→转换，执行器收到密集路径点，平滑运动 | ✓ |
| 只 simplify，不 interpolate | 执行器收到稀疏节点，Phase 3 加速度插补时自然处理 | |
| 两个参数都加 | bool 参数 `interpolate_after_simplify` | |

**User's choice:** simplify 后保留 interpolate()。
**Notes:** 用户先询问了 interpolate() 的作用（在相邻状态间插入中间状态让路径变密集），理解后选择保留。路径点计数日志在 simplify 后、interpolate 前拍快照，以反映真实的简化效果。

---

## 简化超时限制

| Option | Description | Selected |
|--------|-------------|----------|
| 独立参数 `simplify_timeout` | 新增独立参数（默认 0.5s），与 planning_timeout 分离 | ✓ |
| 按比例关联 `planning_timeout` | simplify 时间 = planning_timeout × ratio（默认 0.3） | |
| 不加超时（依赖 max_steps） | simplify_max_steps 已限制迭代次数，不引入时间墙 | |

**User's choice:** 独立参数 `simplify_timeout`（默认 0.5s）。

---

## 简化无效时的日志行为

| Option | Description | Selected |
|--------|-------------|----------|
| DEBUG 级别静默处理 | 无效简化是正常现象，INFO 仅写 "Simplified: N → N waypoints" | |
| INFO 正常输出，记录减少比例 | 统一格式 "Simplified: N → M waypoints (-X%)"，N==M 时为 (-0%) | ✓ |

**User's choice:** INFO 统一格式，记录百分比，N==M 时输出 (-0%)，不特殊处理。

---

## 路径可复现性（从测试反馈引出）

| Option | Description | Selected |
|--------|-------------|----------|
| 加 `planner_seed` 参数 | 设固定值可让调试时路径可复现 | |
| 不加 seed，信任简化就够 | 路径质量改善后，随机性可接受 | ✓ |

**User's choice:** 不加随机种子，接受每次路径不同，信任 shortcutting 能消除先退后伸问题。
**Notes:** 用户在 Phase 1 完成后测试发现两个现象：(1) 相同目标点每次路径不同；(2) 有时手臂先往后退再往前伸到目标。Claude 解释这是 RRTConnect 的正常概率行为，simplifySolution() 的 shortcutting 操作会消除绕弯路径，但路径每次不同是采样算法本质。

---

## Claude's Discretion

无——所有决策均由用户明确选择。

## Deferred Ideas

- **`simplify_method = "none"` 选项** — 允许完全禁用简化用于对比调试，用户未请求，如需可作为 quick task 添加。
- **固定随机种子 `planner_seed`** — 决定不加，路径随机性可接受，简化质量足够。
