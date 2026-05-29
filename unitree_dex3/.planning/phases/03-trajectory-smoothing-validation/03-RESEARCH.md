# Phase 3: Trajectory Smoothing & Validation — Research

**Researched:** 2026-05-13
**Status:** Complete

## 1. Current Implementation Analysis

### 1.1 URDF Joint Limits Parsing (ik_fcl_ompl_planner.cpp:129-135)

Current code extracts **position limits only**:
```cpp
for (const auto& joint_pair : urdf_model.joints_) {
    const auto& joint = joint_pair.second;
    if (joint->type != urdf::Joint::REVOLUTE && ...) continue;
    if (!joint->limits) continue;
    joint_limits_[joint->name] = std::make_pair(joint->limits->lower, joint->limits->upper);
}
```

`joint_limits_` is typed `std::map<std::string, std::pair<double, double>>` (line 232). Velocity data (`joint->limits->velocity`) is available in the URDF model but not currently extracted.

### 1.2 Right Arm URDF Velocity Limits

From `g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf`:

| Joint | velocity (rad/s) |
|-------|-----------------|
| right_shoulder_pitch_joint | 37 |
| right_shoulder_roll_joint | 37 |
| right_shoulder_yaw_joint | 37 |
| right_elbow_joint | 37 |
| right_wrist_roll_joint | 37 |
| right_wrist_pitch_joint | 22 |
| right_wrist_yaw_joint | 22 |

At `velocity_scale=0.2` (D-02 default), effective max velocities: 7.4 rad/s (shoulder/elbow/wrist_roll), 4.4 rad/s (wrist_pitch/wrist_yaw).

### 1.3 Fixed Time Step Trajectory Construction (lines 633-644)

```cpp
path.interpolate();
const auto& states = path.getStates();
trajectory_msgs::msg::JointTrajectory traj_msg;
traj_msg.joint_names = planning_joints;
for (size_t idx = 0; idx < states.size(); ++idx) {
    const auto& state = states[idx];
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (size_t i = 0; i < planning_joints.size(); ++i) {
        point.positions.push_back(state->as<ob::RealVectorStateSpace::StateType>()->values[i]);
    }
    point.time_from_start = rclcpp::Duration::from_seconds(time_step_ * (idx + 1));
    traj_msg.points.push_back(point);
}
```

Key observations:
- `path.interpolate()` densifies the path (OMPL default: adds states at resolution-step intervals)
- Time is `time_step_ * (idx + 1)` — **cumulative from zero**, not per-segment
- No velocity consideration — large joint movements get the same 50ms as tiny ones

### 1.4 Parameter Declaration and Member Variables

- Line 59: `this->declare_parameter("trajectory_time_step", 0.05);`
- Line 100: `this->get_parameter("trajectory_time_step", time_step_);`
- Line 235: `double time_step_ = 0.05;` (member)
- Line 197-198: Logged in init as `"Planning timeout: %.2f seconds, time step: %.2f seconds"`

### 1.5 Publish Location (line 652-653)

```cpp
traj_msg.header.stamp = this->now();
traj_pub_->publish(traj_msg);
```

No validation between construction and publish. This is where validation should be inserted.

### 1.6 Executor's Position Clamp (joint_trajectory_executor.cpp:219-220)

```cpp
auto lim = joint_limits_.at(target_joint_name);
target_position = std::min(std::max(target_position, lim.lower), lim.upper);
```

Defense-in-depth only — not modified in this phase (per D-05).

## 2. Implementation Approach

### 2.1 Velocity Limits Storage (Claude's Discretion from CONTEXT.md)

**Decision: Add a separate `velocity_limits_` map** rather than changing `joint_limits_` to a struct.

Rationale: `joint_limits_` (pair<double,double>) is used in 5+ locations (OMPL bounds setup, IK random seed generation, fallback defaults). Changing its type to a struct would require modifying all consumers. A separate `std::map<std::string, double> velocity_limits_` is zero-disruption — only the new velocity-based code reads it.

### 2.2 Velocity-Based Time Parameterization Algorithm

Per D-01, for consecutive waypoints `q[k]` and `q[k+1]`:
```
dt[k] = max_over_joints(|q[k+1][i] - q[k][i]| / (velocity_limits_[i] * velocity_scale))
dt[k] = max(dt[k], min_time_step)
time_from_start[k+1] = time_from_start[k] + dt[k]
```

First point: `time_from_start[0] = dt[0]` (computed from start state to first waypoint, or `min_time_step` if start state isn't available in the trajectory — but OMPL path includes start state as first entry).

Note: OMPL `path.getStates()` includes the start state as `states[0]`. The trajectory should begin from `states[1]` (first motion point), or alternatively include all states but compute dt from consecutive pairs starting at (0→1).

**Current code starts trajectory from `states[0]`** (idx=0), so `time_from_start[0]` is non-zero (was `time_step_ * 1`). With velocity-based timing, `dt[0]` between start and `states[0]` is zero if they're identical. The `min_time_step` clamp handles this.

### 2.3 Validation Checks (D-07)

Two checks, run after trajectory construction, before publish:

1. **Position limits**: For each point, each joint position within `[lower, upper]` from `joint_limits_`.
2. **Velocity limits**: For consecutive points, `|q[k+1][i] - q[k][i]| / dt[k] <= velocity_limits_[i] * velocity_scale`.

### 2.4 Auto-Fix Strategy (D-06)

On validation failure:
1. **Position fix**: Clamp out-of-range positions to URDF limits. Log with `RCLCPP_WARN`.
2. **Velocity fix**: Stretch `dt` for the offending segment: `dt_new = max_over_joints(|Δq_i| / (vel_limit_i * velocity_scale))`. Recompute cumulative `time_from_start` from that point onward.
3. **Re-validate** after fix. If still failing (shouldn't happen with correct fix), reject with `RCLCPP_ERROR`.

## 3. Code Change Map

All changes in `ik_fcl_ompl_planner.cpp`:

| Location | Change | Lines |
|----------|--------|-------|
| Parameter declaration | Replace `trajectory_time_step` with `velocity_scale` (0.2) and `min_time_step` (0.02) | ~59 |
| Parameter fetch | Replace `get_parameter("trajectory_time_step", time_step_)` with new params | ~100 |
| Member variables | Replace `time_step_` with `velocity_scale_`, `min_time_step_`; add `velocity_limits_` map | ~232-235 |
| URDF parsing | Add `velocity_limits_[joint->name] = joint->limits->velocity` in existing loop | ~130-135 |
| Init log | Update log line to show velocity_scale and min_time_step instead of time_step | ~197-198 |
| Trajectory construction | Replace fixed `time_step_ * (idx + 1)` with velocity-based dt computation | ~637-644 |
| New: validation block | Insert between trajectory construction and publish | ~645-653 |

## 4. Risk Assessment

- **Low risk**: Only one file modified. Executor unchanged.
- **Testable**: Log output shows computed dt values and validation results. Compare motion smoothness visually.
- **Rollback**: Revert single file if issues. Or set `velocity_scale=1.0` + `min_time_step=0.05` to approximate old behavior.

## Validation Architecture

### Verification Strategy
1. **Compile test**: `colcon build` succeeds
2. **Log inspection**: Launch planner, send goal, verify logs show:
   - `velocity_scale` and `min_time_step` params logged at startup
   - Per-trajectory: computed dt values, total duration, validation pass/fail
3. **Behavior test**: Visual confirmation that arm motion is smoother than fixed-time-step
4. **Edge case**: Send goal very close to current position → verify `min_time_step` prevents micro-dt values

## RESEARCH COMPLETE
