---
gsd_state_version: 1.0
milestone: v1.1
milestone_name: AprilTag 定位 + TCP 修正
status: planning
last_updated: "2026-05-19T06:53:55.119Z"
last_activity: 2026-05-19
progress:
  total_phases: 4
  completed_phases: 3
  total_plans: 8
  completed_plans: 8
  percent: 75
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
| 8 | 自适应末端位姿 | ✓ Complete (2026-05-19) | ORI-01 ✓ |
| 9 | 端到端集成 | ○ Pending | INTG-01~02 |

Progress: ████████░░ 75% (3/4 phases) — Phase 8 done

## Active Context

- v1.0 milestone complete — full right-arm reaching pipeline working
- Phase 6 complete: YOLO 已彻底移除；TCP offset 通过 URDF `right_tcp_link` 集成到 planner IK 链
- Phase 7 complete: `apriltag_detector_node.py` 已上线 — 订阅 RGB + CameraInfo、pupil-apriltags 检测 tag36h11、按 D-07/D-10/D-11 三重过滤、PnP 求 6-DOF、按 D-01 应用 tag-local XYZ offset、tf2 变换到 `torso_link`、双 topic（`/apriltag/tag_pose`、`/apriltag/target_pose`）发布；OpenCV 可视化（绿/红框 + 三轴 + HUD + quit-on-q）支持 `imshow:=false`；`apriltag.launch.py` 单命令启动（robot + RealSense 640×480×15 + d435 静态 TF + 检测节点）。验证通过：colcon build rc=0、ros2 pkg executables 列出节点、ros2 launch ... --print rc=0。Live UAT 4 项（topic echo、tf2_echo、参数覆盖、过滤可视化）等待硬件回归。
- Phase 7 verifier: 67/67 must_haves PASS, 0 fail; 4 human_verification 项已记录在 07-VERIFICATION.md
- D435i 全局分辨率为 640×480 @ 15fps，align_depth 关闭
- Phase 8 complete: `ik_fcl_ompl_planner` 添加自适应末端位姿 — `init()` 通过 KDL FK 在 segment 1 缓存 `right_shoulder_pitch_link` 原点（D-05/D-06）；新增 ROS 参数 `adaptive_orientation_enabled`（默认 true，D-10）；新私有方法 `computeAdaptiveOrientation` 用 look-at 正交基（TCP +X = shoulder→target，+Z 主 up，+Y fallback 当 |dot|>0.95）输出确定性四元数（D-01..D-04）；`goalPoseCallback` 在 TF 变换之后插入条件 splice：当 toggle on 时原地修改 `pose_in_base.pose.orientation` 并 emit 单行 INFO（D-09/D-12），目标距肩 <0.05 m 时 RCLCPP_ERROR 早返（D-08）；toggle off 时整段跳过，pre-Phase-8 行为字节级保留（D-11）。验证：colcon build rc=0；12+14 个 source/build 断言全过；object 文件含 `IKFCLPlannerNode::computeAdaptiveOrientation` 符号。Plan 02 落地 8 点 tabletop A/B harness `scripts/adaptive_orientation_ab.py`（D-13/D-14/D-15/D-16）：`ros2 run unitree_g1_dex3_stack adaptive_orientation_ab.py` 在 `:=true` 时 PASS_COUNT 8/8 → exit 0；D-16 边界（无 executor 符号）由 negative grep 强制。
- Phase 8 verifier: 11/11 must_haves PASS, 0 fail；HV-1..HV-4（shoulder log、toggle log、live A/B、D-08 reject）在 G1 现场回归。Roadmap 成功标准 #4（workspace boundary / shoulder-overhead）partial — 代码含 D-03 fallback，但 D-14 选择 tabletop-only UAT，完整 UAT 留给 Future ORI-02。
- Phase 8 UAT (2026-05-19): 5 项测试 4 PASS + 1 issue 已 known_limit_within_scope 化。Test 3 D-15 验收 5/8 vs baseline 4/8（净 +1 / +25%）；3 个 FAIL 经 planner stdout 分析全部归类为 D-04/D-14 scope 之外的约束 —— center-far 物理不可达、left-of-mid 双模式中线碰撞、center-near 单 orientation 取舍。Future ORI-02 multi-candidate orientation 是这两类 collision 类失败的修复路径，center-far 类需上层限制 reach 或换更长臂。详见 .planning/debug/resolved/08-uat-5of8.md。
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
Last activity: 2026-05-19 -- Phase 08 complete (ORI-01)
