# Unitree G1 Right-Arm Safe Reach

## What This Is

A safe, stable right-arm reaching system for the Unitree G1 humanoid robot. The robot stands in place (official running mode maintains balance), detects objects via the existing YOLO + RealSense perception pipeline, transforms coordinates from camera frame to robot frame, and uses OMPL + FCL + TRAC-IK to plan and execute a collision-free trajectory for the right arm only to reach the target position. No dexterous hand control — the task is complete when the right arm end-effector arrives at the target.

## Core Value

The right arm moves safely to the target position without colliding with the robot's own body or the environment, and without exceeding joint limits.

## Requirements

### Validated

- ✓ YOLO object detection on RGB images — existing (`ultralytics_detector.py`)
- ✓ 2D-to-3D projection from bounding boxes + depth images — existing (`project_to_3d_node`)
- ✓ Camera-to-robot TF relationship — already calibrated
- ✓ URDF models for G1 + DEX3 hands — existing in `robots/`
- ✓ OMPL + FCL + TRAC-IK planner compiled — existing (`ik_fcl_ompl_planner`)
- ✓ Joint state publishing from robot — existing (`joint_state_publisher`)
- ✓ Robot state publisher with URDF — existing (`robot.launch.py`)

### Active

- [ ] Right-arm-only motion planning (planner configured for right arm chain only)
- [ ] Self-collision detection (right arm vs torso, legs, left arm)
- [ ] Environment collision avoidance (table and other obstacles)
- [ ] Joint limit protection (position and velocity limits enforced in planner and executor)
- [ ] Smooth trajectory execution (interpolated, no sudden large motions)
- [ ] Coordinate transform pipeline: camera 3D detection → robot base frame → planner goal pose
- [ ] Right-arm-only trajectory executor (sends commands only for right arm joints, leaves rest to running mode)
- [ ] End-to-end integration: perception → coordinate transform → planning → execution
- [ ] Safe startup and error handling (graceful behavior when services/topics unavailable)

### Out of Scope

- Left arm control — not needed for this milestone
- DEX3 hand control — task is reach only, no grasping
- Walking/locomotion — robot stands in place, official running mode handles balance
- YOLO model training — using existing best.pt model
- Camera calibration — already done
- Simulation (Gazebo) — testing on physical robot only

## Context

- Running on Unitree G1 onboard computer (Ubuntu Linux, `/home/unitree/`)
- Existing ROS 2 workspace with all dependencies built (`colcon build`)
- Perception pipeline fully functional (YOLO + RealSense + 3D projection)
- Planner node (`ik_fcl_ompl_planner`) already compiled but currently supports both arms — needs to be focused on right arm only
- Trajectory executor (`joint_trajectory_executor`) currently sends commands to both arms and hands — needs to be restricted to right arm joints only
- Official Unitree running mode controls legs, waist, and overall balance via `/arm_sdk` topic
- Robot communicates via `unitree_hg` custom messages
- Conda environment `grab` required for Python perception node

## Constraints

- **Hardware**: Must run on G1 onboard computer (limited compute)
- **Safety**: All four safety aspects required — self-collision, environment collision, joint limits, trajectory smoothness
- **Coexistence**: Right arm control must coexist with official running mode without interference
- **Real-time**: Planning and execution must be fast enough for practical use (planner timeout configurable, default 1.0s)
- **Right arm only**: Only joints from `right_shoulder_pitch` to `right_wrist_yaw` (7 DOF) are controlled

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| Right arm only, no hand | Simplify scope; reach task doesn't need grasping | — Pending |
| Reuse existing perception pipeline | Already working; focus effort on planning and control | — Pending |
| FCL for collision checking | Already integrated and compiled in workspace | — Pending |
| OMPL RRTConnect for planning | Already implemented in `ik_fcl_ompl_planner` | — Pending |
| Official running mode for balance | Avoids implementing full-body control; proven stable | — Pending |

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
*Last updated: 2025-04-27 after initialization*
