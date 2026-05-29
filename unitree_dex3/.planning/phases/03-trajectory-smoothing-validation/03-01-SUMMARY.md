# Plan 03-01 Summary: Velocity-based time parameterization and trajectory validation

**Phase:** 03 — Trajectory Smoothing & Validation
**Plan:** 03-01
**Status:** Verified ✅
**Commit:** fa4712a (impl) · 5e6224c (velocity_scale 0.1) · 6e0c9b3 (velocity_scale 0.05)
**UAT:** 验收通过 2026-05-13

## What Was Built

Replaced the fixed 50ms time-step trajectory construction in `ik_fcl_ompl_planner.cpp` with velocity-based time parameterization, and added pre-publish trajectory validation with auto-fix. All changes confined to the single file `src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp`.

## Tasks Completed

### Task 1 — velocity_limits_ storage and URDF parsing
- Added `std::map<std::string, double> velocity_limits_` member alongside `joint_limits_`
- Extended the existing URDF joint parsing loop to also populate `velocity_limits_` from `joint->limits->velocity` (guarded: REVOLUTE/PRISMATIC only, velocity > 0)

### Task 2 — Parameter replacement
- Removed `trajectory_time_step` parameter declaration, fetch, and `time_step_` member
- Added `velocity_scale` (final default **0.05**) and `min_time_step` (default 0.02s) ROS 2 parameters
- Tuned down from initial 0.2 → 0.1 → 0.05 based on on-robot observation
- Updated init log to print `velocity_scale` and `min_time_step`

### Task 3 — Velocity-based trajectory construction
- Replaced `point.time_from_start = rclcpp::Duration::from_seconds(time_step_ * (idx+1))` fixed loop with per-segment dt computation
- Per segment: `dt = max(max_over_joints(|Δq_i| / (vel_limit_i × velocity_scale)), min_time_step)`
- Fallback velocity 10.0 rad/s for joints not in `velocity_limits_`
- Accumulates cumulative timestamp; logs total trajectory duration

### Task 4 — Pre-publish validation with auto-fix
- **Position validation:** clamps out-of-range positions to URDF joint limits, logs `RCLCPP_WARN`
- **Velocity validation:** stretches dt for segments exceeding `vel_limit × velocity_scale`, shifts all subsequent timestamps, logs `RCLCPP_WARN`
- **Re-validation:** if still failing after auto-fix, `RCLCPP_ERROR` + `return` (trajectory not published)
- **Pass log:** `"Trajectory validation passed"` or `"Trajectory validation passed (with auto-fix)"`

## Files Modified

- `src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` — 97 insertions, 6 deletions

## Verification

- `colcon build --packages-select unitree_g1_dex3_stack` → **exit code 0** (warnings pre-existing)
- Added `#include <cmath>` and `#include <algorithm>` for `std::abs` / `std::max`
- On-robot motion verified; velocity_scale tuned to 0.05 for safe operation

## Requirements Satisfied

- **EXEC-01:** Trajectory timing computed from URDF joint velocity limits (not fixed 50ms)
- **EXEC-02:** Trajectory validated before publication with position and velocity checks
