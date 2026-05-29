# Unitree G1 Right-Arm Safe Reach

## What This Is

A safe, stable right-arm reaching system for the Unitree G1 humanoid robot. The robot stands in place (official running mode maintains balance), detects AprilTag 36h11 markers via the head-mounted RealSense D435i, computes target position with configurable offset, and uses OMPL + FCL + TRAC-IK to plan and execute a collision-free trajectory for the right arm TCP (wrist + 0.175m offset) to reach the target. No dexterous hand control — the task is complete when the TCP arrives at the target.

## Core Value

The right arm TCP moves safely to the target position without colliding with the robot's own body or the environment, and without exceeding joint limits.

## Current Milestone: v1.1 AprilTag 定位 + TCP 修正

**Goal:** 用 AprilTag 36h11 替代 YOLO 检测，修正 TCP offset 和末端位姿策略，使到达成功率大幅提升。

**Target features:**
- 移除 YOLO 检测代码（已验证不可行）
- AprilTag 36h11 检测 + 可配置相对偏移定位物品
- TCP offset (0.175m) 集成到 planner IK 目标
- 自适应末端位姿（根据目标位置自动选择可行 orientation）

## Requirements

### Validated

- ✓ Right-arm-only OMPL+FCL+TRAC-IK motion planning — v1.0 Phase 1
- ✓ Self-collision detection (right arm vs all body links) — v1.0 Phase 1
- ✓ OMPL path simplification — v1.0 Phase 2
- ✓ Velocity-based trajectory smoothing + validation — v1.0 Phase 3
- ✓ Right-arm-only executor coexisting with running mode — v1.0 Phase 4
- ✓ End-to-end integration with single launch command — v1.0 Phase 5
- ✓ Camera-to-robot TF relationship — already calibrated
- ✓ URDF models for G1 + DEX3 hands — existing in `robots/`
- ✓ Joint state publishing from robot — existing

### Active

- [ ] 移除 YOLO 检测代码（已验证不可行）
- [ ] AprilTag 36h11 检测 + 可配置相对偏移定位物品
- [ ] TCP offset (0.175m) 集成到 planner IK 目标
- [ ] 自适应末端位姿（根据目标位置自动选择可行 orientation）

### Out of Scope

- Left arm control — not needed for this milestone
- DEX3 hand control — task is reach only, no grasping
- Walking/locomotion — robot stands in place, official running mode handles balance
- Simulation (Gazebo) — testing on physical robot only
- YOLO 检测 — 已验证不可行，彻底移除

## Context

- Running on Unitree G1 onboard computer (Ubuntu Linux, `/home/unitree/`)
- Existing ROS 2 workspace with all dependencies built (`colcon build`)
- v1.0 pipeline fully functional: planner + executor + launch files
- YOLO 检测经测试不可行，需彻底移除相关代码
- 头部 RealSense D435i 相机继续使用
- TCP offset = 0.175m 沿 right_wrist_yaw_link 局部 X 轴（参见 `tcp_torso_pose.py`）
- Planner 当前 IK 目标是 right_wrist_yaw_link，未考虑 TCP offset
- 固定末端位姿导致 IK/OMPL 失败率高，需自适应策略
- Official Unitree running mode controls legs, waist, and overall balance via `/arm_sdk` topic
- Conda environment `grab` required for Python perception nodes

## Constraints

- **Hardware**: Must run on G1 onboard computer (limited compute)
- **Safety**: All four safety aspects required — self-collision, environment collision, joint limits, trajectory smoothness
- **Coexistence**: Right arm control must coexist with official running mode without interference
- **Real-time**: Planning and execution must be fast enough for practical use (planner timeout configurable, default 1.0s)
- **Right arm only**: Only joints from `right_shoulder_pitch` to `right_wrist_yaw` (7 DOF) are controlled

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| Right arm only, no hand | Simplify scope; reach task doesn't need grasping | ✓ Good |
| Reuse existing perception pipeline | Already working; focus effort on planning and control | ⚠️ Revisit — YOLO 不可行 |
| FCL for collision checking | Already integrated and compiled in workspace | ✓ Good |
| OMPL RRTConnect for planning | Already implemented in `ik_fcl_ompl_planner` | ✓ Good |
| Official running mode for balance | Avoids implementing full-body control; proven stable | ✓ Good |
| 移除 YOLO，改用 AprilTag 36h11 | YOLO 检测经测试不可行；AprilTag 轻量可靠 | — Pending |
| TCP offset 集成到 planner | 当前 IK 目标不含 TCP offset，导致实际到达位置偏差 | — Pending |
| 自适应末端位姿 | 固定位姿导致 IK/OMPL 失败率高 | — Pending |

## Evolution

This document evolves at phase transitions and milestone boundaries.

**After each phase transition** (via `/gsd-transition`):
1. Requirements invalidated? → Move to Out of Scope with reason
2. Requirements validated? → Move to Validated with phase reference
3. New requirements emerged? → Add to Active
4. Decisions to log? → Add to Key Decisions
5. "What This Is" still accurate? → Update if drifted

**After each milestone** (via `/gsd-complete-milestone`):
1. Full review of all sections
2. Core Value check — still the right priority?
3. Audit Out of Scope — reasons still valid?
4. Update Context with current state

---
*Last updated: 2026-05-15 after milestone v1.1 start*
