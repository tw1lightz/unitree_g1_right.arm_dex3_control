---
status: open
phase: 08-adaptive-orientation
ticket: 08-uat-5of8
gap_ref: 08-UAT.md test 3
created: 2026-05-19
updated: 2026-05-19
severity: major
---

# Debug: Phase 8 A/B harness 5/8 PASS（D-15 8/8 未达成）

## 现象

`/gsd-execute-phase 8` 后跑 `08-UAT.md` test 3：
- `adaptive_orientation_enabled:=true`：5/8 PASS，3 个 FAIL（每个 FAIL = harness 3.0 s 超时未收到 `/joint_trajectory_targets`）
- `adaptive_orientation_enabled:=false` baseline：4/8 PASS

| 目标 label | (x, y, z) torso_link | adaptive=true | adaptive=false |
|------------|---------------------|--------------|----------------|
| center | (0.40, -0.20, 0.00) | PASS (~2.2s) | ? |
| center-near | (0.30, -0.20, 0.00) | **FAIL (timeout)** | ? |
| center-far | (0.55, -0.20, 0.00) | **FAIL (timeout)** | ? |
| right-side | (0.40, -0.40, 0.00) | PASS (~2.1s) | ? |
| left-of-mid | (0.40, -0.05, 0.00) | **FAIL (timeout)** | ? |
| low | (0.40, -0.20, -0.10) | PASS (~2.1s) | ? |
| high | (0.40, -0.20, 0.15) | PASS (~2.1s) | ? |
| diag | (0.45, -0.30, 0.05) | PASS (~2.2s) | ? |

baseline 模式下哪 4 个 PASS / 哪 4 个 FAIL 没有现场记录 —— 是诊断 missing 数据。

## 几何粗算（基于 URDF + KDL 链）

右肩缓存原点（init log）= (0.0040, -0.1002, 0.2478)。
右臂从 `right_shoulder_pitch_link` 经 `right_tcp_link` 总长（伸直）≈ 0.55–0.60 m。

| 目标 | 离右肩距离 (m) | TCP +X 方向（adaptive） | 备注 |
|------|---------------|------------------------|------|
| center | 0.408 | (+0.97, -0.24, -0.61) → ‖.‖=1 | 在臂展中段，方向自然 |
| center-near | 0.400 | (+0.74, -0.25, -0.62) | 距离 OK，但 TCP 必须显著向下指 |
| center-far | 0.608 | (+0.90, -0.16, -0.41) | **接近或超出最大臂展** |
| right-side | 0.413 | (+0.96, -0.73, -0.60) | 方向远离躯干，腕关节舒展 |
| left-of-mid | 0.470 | (+0.84, +0.11, -0.53) | **TCP +X 有正 Y 分量 → 必须穿过身体中线** |
| low | 0.503 | (+0.79, -0.20, -0.69) | 距离 OK，下倾自然 |
| high | 0.474 | (+0.84, -0.21, -0.20) | 距离 OK，方向接近水平 |
| diag | 0.567 | (+0.79, -0.35, -0.32) | 距离接近极限，但方向自然 |

3 个 FAIL 目标的几何特征：
- **center-far** ≈ 0.61 m 接近或超出右臂最大可达半径
- **center-near** TCP +X 必须显著向下指（z 分量 -0.62）—— 在 adaptive 模式下需要腕 pitch / roll 弯到极限
- **left-of-mid** TCP +X 有正 Y 分量 0.11 —— 右臂要把腕指向左前方（穿过躯干中线），右臂关节限制大概率拒绝

PASS 的 5 个目标共同特征：距离适中（0.40–0.57 m）且 TCP 方向远离躯干、不过度俯仰。

## 假设（按可能性排序）

### H1（最可能）：底层运动学限制，跟 Phase 8 自适应逻辑没直接关系

baseline 模式（固定四元数）也只有 4/8 PASS —— 说明这 4 个失败的目标里，至少有几个**无论用什么 orientation 都到不了**：要么超出臂展（center-far），要么 TCP 朝向冲突 wrist 关节限制。Phase 8 自适应位姿把 +X 指向目标，反而在 center-near / left-of-mid 把 wrist 推到更极端方向 —— 这就是 +1 改善的来源（adaptive 让 1 个原本失败的几何 + 失败 orientation 案例变为成功），但**无法拯救本来就不可达的位置**。

如果 H1 成立：
- 这不是 Phase 8 的 bug，是 ORI-01 单一确定性 orientation 的固有局限（CONTEXT D-04 明确把 multi-candidate fallback 列为 Future ORI-02）。
- 对应 ROADMAP success criterion #4 "在工作空间边界…" 的 partial 状态正是这个事实。
- **正确处理**：把 5/8 接受为 Phase 8 的已知极限，记录 baseline n=4 vs adaptive n=5 的相对改善，将 8/8 目标推迟到 Future ORI-02 的 multi-candidate orientation。

### H2：harness 3.0 s timeout 太紧 —— 真有 trajectory 但晚到

planner 内 IK 链：
- TRAC-IK 单次 timeout = 1.0 s
- "too-close-to-seed" 重试上限 = 20 次随机种子（原代码已有，非 Phase 8 引入），最坏 21 × 1.0 = 21 s
- OMPL `planning_timeout` = 1.0 s

harness 3.0 s 容忍 = 1×TRAC-IK + 1×OMPL + 余量。如果首次 IK 失败、走 neutral seed 重试，再走 random-seed 循环，时间可能超过 3 s。

如果 H2 成立：
- harness 把 timeout 加到 5–8 s，重新跑就会观察到 PASS_COUNT 提升或者 timeout 消除。
- **正确处理**：bump 默认 timeout，记录现场实际收到 trajectory 的耗时。

### H3：planner 内 OMPL planning_timeout 不够，单纯加时间能解决

启动时 INFO 行 `Planning timeout: 1.00 seconds`。对距离极限的 center-far / 朝向较难的 case，OMPL 1.0 s 找路径可能不够。

如果 H3 成立：
- 把 `planning_timeout:=2.0` 或 `:=3.0` 重新跑能直接救回部分。
- **正确处理**：加 launch arg 调高 timeout，文档化。

### H4：collision-skip-pairs 没覆盖某些 adaptive 朝向才会接触的 link 对

新 orientation 把 TCP +X 指向身体中线时（left-of-mid），右臂大臂可能与躯干碰撞 —— 但 ik_fcl_ompl_planner 的 collision check 仅在 OMPL 阶段做，IK 阶段不做，所以 IK 结果会先被 OMPL state-validity-check 拒绝。OMPL 找不到合法 start state → 输出 `start=INVALID` warning 但仍然超时退出，无 trajectory。

如果 H4 成立：
- 启动时 INFO 行的 collision skip pairs 列表不全，需要追加。
- **正确处理**：现场跑测试 3 时把 planner stdout 抓到文件，看是不是有 `OMPL state validity: start=INVALID` 行。

## 缺失数据（决定走哪个假设的关键）

为了把诊断从假设变事实，我需要 **planner 那一侧的 stdout/stderr**。最便捷的做法是在重跑测试 3 时把 planner 输出导到文件：

```bash
# 终端 A：
ros2 launch unitree_g1_dex3_stack planner.launch.py 2>&1 | tee /tmp/p8-planner.log
# 终端 B（重跑 harness）：
ros2 run unitree_g1_dex3_stack adaptive_orientation_ab.py
```

然后取这一段：
```bash
grep -E 'Adaptive orientation|TRAC-IK result|IK failed|OMPL state validity|OMPL failed|Aborting goal' /tmp/p8-planner.log | head -100
```

关键信号：
- 三个 FAIL 目标各对应几行 `Adaptive orientation: target=…` —— 验证 D-12 日志确实在打。
- 紧跟着是 `TRAC-IK result: No solution found (code: -3)` → 假设 H1（IK 几何不可达）。
- 或者 `IK succeeded` 但接着 `OMPL state validity: start=INVALID` 或 `OMPL failed to find a path` → 假设 H3 或 H4。
- 或者一长串 `Trying random seed (N/20)` 然后 `Failed to find a sufficiently different IK solution` → 老 IK-too-close-to-seed 重试逻辑（非 Phase 8 引入），可能走 H2。

baseline 模式（test 4）下哪 4 个 PASS / 哪 4 个 FAIL 也是关键 —— 如果 baseline 失败的 4 个里有 3 个跟 adaptive 失败的 3 个完全重合，H1 立判成立。

## 决策树

| 路径 | 触发条件 | Phase 8 处理 |
|------|----------|-------------|
| **A. 接受为已知极限**（推荐） | H1 成立（grep 显示 IK / OMPL 在 FAIL 目标上失败） | 修文档：08-VERIFICATION.md 记录 5/8 + baseline 4/8 的对比，明确写"+1 改善 = Phase 8 价值，剩余 3/8 等 Future ORI-02 multi-candidate orientation 解决"。Phase 8 不开 gap-closure plan。 |
| **B. 加 harness/planner timeout** | H2 或 H3 成立 | 开 1 个简短 gap-closure plan：(a) `adaptive_orientation_ab.py` `timeout_sec` 默认 3.0 → 6.0；(b) `planner.launch.py` 加 `planning_timeout` 默认 1.0 → 2.0。重跑 test 3 验收。 |
| **C. 修 collision skip / planner 别的 bug** | H4 或其它意外信号 | 开正式 gap-closure plan，按 stdout 揭示的具体故障点改代码。 |

未拿到 planner stdout 之前，A 是默认最稳妥的路径（A/B baseline 数据已经支持 H1）。

## 关联文件

- `.planning/phases/08-adaptive-orientation/08-UAT.md`（本 issue 来源）
- `.planning/phases/08-adaptive-orientation/08-CONTEXT.md`（D-04 multi-candidate 是 Future ORI-02）
- `.planning/phases/08-adaptive-orientation/08-RESEARCH.md`（PIT-08 右手系断言提到 "如果 paranoid 加 debug-level assertion"）
- `src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` 360 行起的 IK 重试 + 940 行起的 OMPL setup
- `src/unitree_g1_dex3_stack-main/scripts/adaptive_orientation_ab.py:39` `timeout_sec` 默认值

---
*Created: 2026-05-19 — pending stdout-capture re-run before root cause is fact rather than hypothesis*
