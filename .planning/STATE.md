---
gsd_state_version: 1.0
milestone: v1.1
milestone_name: AprilTag 定位 + TCP 修正
status: ready_to_plan
last_updated: 2026-05-19T02:04:18.707Z
last_activity: 2026-05-19 -- Phase 8 execution started
progress:
  total_phases: 4
  completed_phases: 2
  total_plans: 8
  completed_plans: 28
  percent: 50
stopped_at: Phase 8 complete (2/2) — ready to discuss Phase 9
---

# Project State

## Project Reference

See: .planning/PROJECT.md (updated 2026-05-15)

**Core value:** The right arm TCP moves safely to the target position without colliding with the robot's own body or the environment, and without exceeding joint limits.
**Current focus:** Phase 9 — 端到端集成

## Current Position

Phase: 9
Plan: Not started
Status: Ready to plan
Last activity: 2026-05-19

## Current Milestone

**v1.1 — AprilTag 定位 + TCP 修正**

| Phase | Name | Status | Requirements |
|-------|------|--------|--------------|
| 6 | YOLO 清理 + TCP Offset 集成 | ✓ Complete (2026-05-18) | CLEAN-01 ✓, TCP-01 ✓, TCP-02 ✓ |
| 7 | AprilTag 检测节点 | ✓ Complete (2026-05-18) | TAG-01 ✓, TAG-02 ✓, TAG-03 ✓, TAG-04 ✓ |
| 8 | 自适应末端位姿 | ○ Pending | ORI-01 |
| 9 | 端到端集成 | ○ Pending | INTG-01~02 |

Progress: █████░░░░░ 50% (2/4 phases) — Phase 7 done

## Active Context

- v1.0 milestone complete — full right-arm reaching pipeline working
- Phase 6 complete: YOLO 已彻底移除；TCP offset 通过 URDF `right_tcp_link` 集成到 planner IK 链
- Phase 7 complete: `apriltag_detector_node.py` 已上线 — 订阅 RGB + CameraInfo、pupil-apriltags 检测 tag36h11、按 D-07/D-10/D-11 三重过滤、PnP 求 6-DOF、按 D-01 应用 tag-local XYZ offset、tf2 变换到 `torso_link`、双 topic（`/apriltag/tag_pose`、`/apriltag/target_pose`）发布；OpenCV 可视化（绿/红框 + 三轴 + HUD + quit-on-q）支持 `imshow:=false`；`apriltag.launch.py` 单命令启动（robot + RealSense 640×480×15 + d435 静态 TF + 检测节点）。验证通过：colcon build rc=0、ros2 pkg executables 列出节点、ros2 launch ... --print rc=0。Live UAT 4 项（topic echo、tf2_echo、参数覆盖、过滤可视化）等待硬件回归。
- Phase 7 verifier: 67/67 must_haves PASS, 0 fail; 4 human_verification 项已记录在 07-VERIFICATION.md
- D435i 全局分辨率为 640×480 @ 15fps，align_depth 关闭
- Phase 8 准备工作：可订阅 `/apriltag/target_pose` 直接接入；ORI-01 需要根据目标相对右肩的方向计算 orientation
- Phase 9 才整合到端到端 launch 并桥接到 `/goal_pose`

## Decisions Log

| Decision | Phase | Rationale |
|----------|-------|-----------|
| 移除 YOLO，改用 AprilTag 36h11 | Init | YOLO 检测经测试不可行；AprilTag 轻量可靠 |
| TCP offset 集成到 planner | Init | 当前 IK 目标不含 TCP offset |
| 自适应末端位姿 | Init | 固定位姿导致 IK/OMPL 失败率高 |
| Phase 7 检测节点单独 launch | Phase 7 | 隔离 detection 与 manipulation pipeline；Phase 9 再合 |
| imshow 作为 launch arg 而非 YAML param | Phase 7 (D-15) | 部署侧切换 (headless / SSH) 需要灵活覆盖 |
| pupil-apriltags 不进 package.xml | Phase 7 | 仅 pip 提供，rosdep 没有；改为 README 提示 + pip install |

---
Last activity: 2026-05-18 -- Phase 07 complete (TAG-01..04)
