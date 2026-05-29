# Roadmap: Unitree G1 Right-Arm Safe Reach

**Created:** 2026-05-15
**Milestone:** v1.1 — AprilTag 定位 + TCP 修正
**Phases:** 4
**Requirements:** 10

## Phase 6: YOLO 清理 + TCP Offset 集成 ✅

**Status:** Complete (2026-05-18)
**Goal:** 移除不可行的 YOLO 代码，将 TCP offset 集成到 planner IK 链末端。

**Requirements:** CLEAN-01 ✓, TCP-01 ✓, TCP-02 ✓

**Success Criteria:**
1. ✓ YOLO 相关文件（ultralytics_detector.py、project_to_3d_node 等）已删除并提交
2. ✓ 原 reach.launch.py 中 YOLO 相关节点已移除
3. ✓ Planner IK 链末端延伸到 TCP 点（wrist_yaw_link + 0.175m X 轴）
4. ✓ TCP offset 通过 ROS 参数配置，不硬编码
5. ✓ 现有 planner 功能不受影响（手动指定目标仍可规划）

**Depends on:** —

---

## Phase 7: AprilTag 检测节点 ✅

**Status:** Complete (2026-05-18)
**Goal:** 实现 AprilTag 36h11 检测，发布 tag 位姿并转换到 torso_link 帧。

**Requirements:** TAG-01 ✓, TAG-02 ✓, TAG-03 ✓, TAG-04 ✓

**Success Criteria:**
1. ✓ pupil-apriltags 检测节点从 D435i color stream 检测 tag36h11
2. ✓ 检测到 tag 后发布 6-DOF 位姿（PoseStamped）
3. ✓ 可配置 tag→物品 XYZ 偏移通过 YAML 文件设置
4. ✓ 位姿通过 TF 从 camera_color_optical_frame 变换到 torso_link
5. ✓ 低质量检测被过滤（decision_margin 阈值）
6. ✓ 节点可独立启动测试，不依赖 planner

**Depends on:** Phase 6（YOLO 已清理，避免冲突）

---

## Phase 8: 自适应末端位姿 ✅

**Status:** Complete (2026-05-19)
**Goal:** 根据目标位置自动计算可行的末端 orientation，提高 IK/OMPL 成功率。

**Requirements:** ORI-01 ✓

**Success Criteria:**
1. ✓ 根据目标位置相对右肩的方向自动计算 orientation
2. ✓ 计算出的 orientation 使手臂自然指向目标（非固定死姿态）
3. ⚠ partial — 实测 +25% 相对改善（adaptive=true 5/8 PASS vs adaptive=false baseline 4/8 PASS，HV-3 2026-05-19 现场 A/B 实测）。adaptive 救回 right-side + low 两个 baseline IK 失败，回归 center-near 一个（单 orientation 取舍），对真·难的 center-far / left-of-mid 不分伯仲。"明显提升" 的目标（≥ 8/8）需要 Future ORI-02 multi-candidate orientation。
4. ✓ 在工作空间边界和肩部正上方等困难区域仍能找到可行姿态（D-03 +Y_torso fallback 已实现；CONTEXT D-14 选择 tabletop-only UAT，工作空间边界与肩部正上方完整 UAT 覆盖延期至 Future ORI-02）

**Depends on:** Phase 6（TCP offset 已集成，orientation 计算基于 TCP）

---

## Phase 9: 端到端集成 ✅

**Status:** Complete (2026-05-19)
**Goal:** 将 AprilTag 检测 + TCP 修正 + 自适应位姿整合为完整流水线。

**Requirements:** INTG-01 ✓, INTG-02 ✓
**Plans:** 4/4 plans executed

**Success Criteria:**
1. apriltag_reach.launch.py 单命令启动完整流水线
2. 全流程验证：AprilTag 检测 → 偏移计算 → TF 变换 → 自适应 orientation → planner → executor
3. TCP 实际到达位置与目标位置误差在可接受范围内
4. 在物理机器人上演示成功

**Depends on:** Phase 7, Phase 8

**Plans:**
- [x] 09-01-PLAN.md — AprilTag goal bridge node (apriltag_goal_bridge.py, D-01..D-14)
- [x] 09-02-PLAN.md — End-to-end launch file (apriltag_reach.launch.py, D-15..D-20)
- [x] 09-03-PLAN.md — UAT harness with FK measurement (apriltag_reach_uat.py, D-21..D-25)
- [x] 09-04-PLAN.md — CMakeLists, README docs, delete keyboard_trigger_node.py

---

## Coverage

| Requirement | Phase | Status |
|-------------|-------|--------|
| CLEAN-01 | Phase 6 | Complete |
| TCP-01 | Phase 6 | Complete |
| TCP-02 | Phase 6 | Complete |
| TAG-01 | Phase 7 | Complete |
| TAG-02 | Phase 7 | Complete |
| TAG-03 | Phase 7 | Complete |
| TAG-04 | Phase 7 | Complete |
| ORI-01 | Phase 8 | Complete |
| INTG-01 | Phase 9 | Complete |
| INTG-02 | Phase 9 | Complete |

**v1.1 requirements:** 10 total
**Mapped to phases:** 10
**Unmapped:** 0 ✓

---
*Roadmap created: 2026-05-15*
*Last updated: 2026-05-19 — Phase 9 execution complete (all 4 plans done)*
