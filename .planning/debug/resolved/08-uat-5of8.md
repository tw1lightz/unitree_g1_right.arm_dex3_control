---
status: resolved
phase: 08-adaptive-orientation
ticket: 08-uat-5of8
gap_ref: 08-UAT.md test 3
created: 2026-05-19
updated: 2026-05-19
resolved: 2026-05-19
resolution: known-limit-within-scope
severity: major
---

# Debug: Phase 8 A/B harness 5/8 PASS（D-15 8/8 未达成）

## 现象

`/gsd-execute-phase 8` 后跑 `08-UAT.md` test 3：
- `adaptive_orientation_enabled:=true`：5/8 PASS，3 个 FAIL
- `adaptive_orientation_enabled:=false` baseline：4/8 PASS

## 实测对比（事实，from /tmp/p8-debug/planner-{true,false}.log + harness summary）

| Target | (x, y, z) torso_link | adaptive=true | adaptive=false | 根因（实测日志） |
|--------|---------------------|---------------|----------------|-----------------|
| center | (0.40, -0.20, 0.00) | PASS | PASS | 双模式可达 |
| **center-near** | (0.30, -0.20, 0.00) | **FAIL** OMPL goal=INVALID | **PASS** | adaptive 朝向把 IK 解推进碰撞；fixed quat 凑巧不撞 |
| **center-far** | (0.55, -0.20, 0.00) | **FAIL** TRAC-IK -3 | **FAIL** TRAC-IK -3 | TRUE 运动学不可达（≈0.61 m 接近/超出右臂极限） |
| right-side | (0.40, -0.40, 0.00) | **PASS** | **FAIL** TRAC-IK -3 | adaptive 救回一个 baseline IK 失败 |
| **left-of-mid** | (0.40, -0.05, 0.00) | **FAIL** OMPL goal=INVALID | **FAIL** OMPL goal=INVALID | 双模式都在目标关节角碰撞（近躯干中线） |
| low | (0.40, -0.20, -0.10) | **PASS** | **FAIL** TRAC-IK -3 | adaptive 救回一个 baseline IK 失败 |
| high | (0.40, -0.20, 0.15) | PASS | PASS | 双模式可达 |
| diag | (0.45, -0.30, 0.05) | PASS | PASS | 双模式可达 |

净结果：adaptive 救回 right-side + low（+2），回归 center-near（-1），对真正难的 center-far / left-of-mid 跟 baseline 表现一致 → 净 +1 = 5 vs 4。

## 假设验证结论（事实，非假设）

| 假设 | 状态 | 决定性证据 |
|------|------|------------|
| **H1 — 运动学不可达** | **部分确认 — 仅适用于 center-far** | 双模式 TRAC-IK 直接 `No solution found (code: -3)` + `IK failed with both current and neutral seed. Aborting.` |
| **H2 — IK retry 撑爆 harness 3.0 s timeout** | **否定** | 失败目标的日志里没有任何 `Trying random seed (N/20)` 行 —— 重试循环根本没触发。 |
| **H3 — OMPL planning_timeout 不够** | **否定** | OMPL 失败原因是 `goal=INVALID` —— start state valid 而 goal IK 解被 collision check 拒收，OMPL 根本没开始找路径，1.0 s timeout 没起作用。 |
| **H4 — 目标关节角碰撞** | **确认 — 适用于 center-near (adaptive 单边)、left-of-mid (双模式)** | IK Success → `OMPL state validity: start=VALID goal=INVALID` → `OMPL failed to find a path` 一气呵成。collision check 在 IK 解的关节角上判到 link 自碰撞。 |

H2/H3 完全否定 → 加 timeout 的 path C 路径数据上不成立，无意义。

## 根因（事实）

5/8 失败由两类正交原因构成，**全部都在 Phase 8 设计 scope 之外**（per CONTEXT.md D-04 + D-14）：

### 类别 1: 物理运动学不可达（H1）

- center-far at (0.55, -0.20, 0.00)：离右肩 0.61 m，接近或超出右臂最大伸展（≈0.55–0.60 m）。
- 任何 orientation 都救不了 —— 这是物理硬限。
- 解决方案：换更长的臂；或在上层（Phase 9 端到端集成）限制 goal 不超出 0.55 m 半径；或在 planner 里加 reach radius 检查 + 早返。
- **跟 Phase 8 单 orientation 设计无关**。

### 类别 2: 目标关节角碰撞（H4），单 orientation 取舍

- left-of-mid at (0.40, -0.05, 0.00)：靠近躯干中线 (Y=-0.05)，腕在任何朝向下都碰到躯干 / 自身手指 / 其它臂段（双模式 OMPL 都判 goal=INVALID）。
- center-near at (0.30, -0.20, 0.00) under adaptive：adaptive 朝向把 TCP +X 向下指 62%（dir z 分量 -0.62），导致腕的 IK 解关节角触发自碰撞；fixed quat 朝向偏水平，凑巧避开。
- 这正是 D-04 明文写着 deferred 到 Future ORI-02 的"multi-candidate orientation fallback"该解决的问题：单确定性 orientation 必然有取舍，对工作空间正常区域救 IK，对极端位置反而把 IK 解推进碰撞。

### Phase 8 净价值（实测）

- adaptive 救 right-side + low（+2，固定 quat 的腕 roll 在这两个目标上不可行，adaptive 的腕 roll 自然顺手）
- adaptive 回归 center-near（-1，单 orientation 取舍 ≡ ORI-02 该救的 case）
- 对真·难的 center-far / left-of-mid 不分伯仲（kinematic / collision 硬限）

净 +1 (+25% 相对 baseline) 是 Phase 8 在单 orientation 体制下的合理改善 —— 不是"明显提升"，但是合规改善。

## 解决路径决定（与用户协商后）

**接受 5/8 为 Phase 8 设计 scope 内的已知极限**，不开 gap-closure plan。原因：

1. CONTEXT D-04 明文限定单 orientation，multi-candidate 是 Future ORI-02 范畴。
2. CONTEXT D-14 用户明确选择 tabletop-only UAT scope。
3. 所有 3 个 FAIL 的根因都是设计 scope 之外的约束（kinematic 物理极限 + 单 orientation 取舍 + 工作空间近中线碰撞）。
4. 真·改善（+2 个 IK 救回）已实测确认。
5. 加 timeout（path C）数据已否决，没意义。

后续动作：
- 08-VERIFICATION.md 加 "Known Limitations within D-04 Scope" 节，记录上述 per-target 根因。
- ROADMAP success criterion #3 "明显提升" 从 ✓ 改 partial（+1 是合规改善但不是"明显"）。
- Future ORI-02 ticket: multi-candidate orientation 救 center-near 类（adaptive 取舍）+ left-of-mid 类（近中线碰撞）；center-far 类无 software 修复路径。

## 关联文件

- `.planning/phases/08-adaptive-orientation/08-UAT.md`（gap 来源）
- `.planning/phases/08-adaptive-orientation/08-VERIFICATION.md`（已知极限文档化目标）
- `.planning/phases/08-adaptive-orientation/08-CONTEXT.md`（D-04 / D-14 scope 边界来源）
- `src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp`（adaptive splice + 现有 IK retry 逻辑）

## 实测日志切片（保存的根本证据）

planner-true.log 关键片段：
```
24: Adaptive orientation: target=[0.400,-0.200,0.000] dir=[0.829,-0.209,-0.519] q=[…]
25: TRAC-IK result: Success (code: 384)
36: Plan published: 14 waypoints                                       ← center PASS
38: Adaptive orientation: target=[0.300,-0.200,0.000] dir=[0.742,-0.250,-0.621]
39: TRAC-IK result: Success (code: 209)
42: OMPL state validity: start=VALID goal=INVALID                       ← center-near collision
56: OMPL failed to find a path for goal pose                            ← center-near FAIL
58: Adaptive orientation: target=[0.550,-0.200,0.000] dir=[0.898,-0.164,-0.408]
59: TRAC-IK result: No solution found (code: -3)                        ← center-far IK fail
61: IK failed with both current and neutral seed. Aborting.             ← center-far FAIL
80: TRAC-IK result: Success (code: 253)
83: OMPL state validity: start=VALID goal=INVALID                       ← left-of-mid collision
97: OMPL failed to find a path for goal pose                            ← left-of-mid FAIL
```

planner-false.log 关键片段（同 8 个目标，固定 quat）:
```
49,53: TRAC-IK -3 → IK failed → Aborting    (center-far, right-side: kinematic IK fail)
60-74: OMPL goal=INVALID → failed path      (left-of-mid: same collision under fixed quat)
76: TRAC-IK -3 → Aborting                   (low: kinematic IK fail under fixed orientation only)
```

---

*Resolved: 2026-05-19 — H1/H4 confirmed via planner stdout, root causes are D-04 / D-14 scope boundaries, no Phase 8 gap-closure planned.*
