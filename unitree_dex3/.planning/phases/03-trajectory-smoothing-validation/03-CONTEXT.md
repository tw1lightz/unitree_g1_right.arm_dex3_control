# Phase 3: Trajectory Smoothing & Validation - Context

**Gathered:** 2026-05-13
**Status:** Ready for planning

<domain>
## Phase Boundary

Replace the fixed 50ms time step in `ik_fcl_ompl_planner.cpp` with velocity-based time parameterization that respects URDF joint velocity limits, and add pre-publish trajectory validation (position limits + velocity limits) with auto-fix capability. Only the planner file is modified; the executor keeps its existing position clamp as a safety net.

</domain>

<decisions>
## Implementation Decisions

### Time Parameterization
- **D-01:** Use simple max-velocity scaling with minimum step: for each consecutive waypoint pair, `dt = max(|Δq_i| / vel_limit_i) / velocity_scale`, clamped to at least `min_time_step`. No trapezoidal or TOPP-RA profiles — keep it simple.
- **D-02:** Add `velocity_scale` ROS 2 parameter (double, default 0.2). This means the arm runs at 20% of URDF motor velocity limits by default (shoulder/elbow effective max ~7.4 rad/s, wrist pitch/yaw ~4.4 rad/s).
- **D-03:** Add `min_time_step` ROS 2 parameter (double, default 0.02s). Prevents micro-movements from producing impractically short time intervals.
- **D-04:** Remove the existing fixed `trajectory_time_step` parameter (currently 0.05s) — it is replaced by velocity-based timing.

### Trajectory Validation
- **D-05:** Validation runs in the planner only, after trajectory construction and before publishing. The executor's existing position clamp (line 219-220 of `joint_trajectory_executor.cpp`) stays as-is for defense in depth.
- **D-06:** On validation failure, attempt auto-fix first: clamp positions to URDF limits, stretch `dt` for velocity violations. Re-validate after fix. If still failing, reject with `RCLCPP_ERROR` and do not publish.
- **D-07:** Validation checks: (1) all joint positions within URDF limits, (2) implied velocity between consecutive waypoints does not exceed `vel_limit_i * velocity_scale`.

### Claude's Discretion
- **Velocity limits storage in planner:** Claude decides whether to extend the existing `joint_limits_` (`pair<double,double>`) to a struct with velocity, or add a separate `velocity_limits_` map. Choose based on minimal disruption to existing position-limit usage.

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Source Code
- `src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` — File being modified. Key lines: 59 (`trajectory_time_step` param), 235 (`time_step_` member), 633-644 (fixed time step trajectory construction), 611-656 (solve/simplify/publish block)
- `src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp` — NOT modified in this phase, but reference for understanding trajectory consumption: line 219-220 (position clamp), line 225-229 (time_from_start scheduling)

### Robot Model
- `src/unitree_g1_dex3_stack-main/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf` — URDF with velocity limits. Right arm velocity limits: shoulder/elbow/wrist_roll = 37 rad/s, wrist_pitch/wrist_yaw = 22 rad/s

### Supporting Files
- `src/unitree_g1_dex3_stack-main/include/g1_dex3_joint_defs.hpp` — Joint enums and name→index maps

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- `joint_limits_` (planner) — Already populated from URDF for all joints (position only). Needs velocity data added.
- URDF parsing block (planner lines 95-110) — Already fetches `robot_description` and parses joint limits. Extend to also read `joint->limits->velocity`.
- `path.interpolate()` at line 633 — Produces densified waypoints. Time parameterization runs on these interpolated states.

### Established Patterns
- ROS 2 `declare_parameter` / `get_parameter` — All new params (`velocity_scale`, `min_time_step`) follow this pattern (see Phase 2 params as example)
- RCLCPP logging macros — Use `RCLCPP_INFO` for timing summary, `RCLCPP_WARN` for auto-fix, `RCLCPP_ERROR` for rejection

### Integration Points
- Trajectory timing computation inserts between `path.interpolate()` (line 633) and the trajectory construction loop (lines 637-644)
- Validation runs after trajectory construction, before `traj_pub_->publish()` (line 653)
- The existing `time_step_` parameter and its usage at line 643 is replaced by per-waypoint computed dt

</code_context>

<specifics>
## Specific Ideas

No specific requirements — open to standard approaches within the decisions above.

</specifics>

<deferred>
## Deferred Ideas

None — discussion stayed within phase scope.

</deferred>

---

*Phase: 03-trajectory-smoothing-validation*
*Context gathered: 2026-05-13*
