# Phase 1: Right-Arm-Only Planner - Context

**Gathered:** 2025-04-28
**Status:** ✅ COMPLETE — all plans verified by user

<domain>
## Phase Boundary

Modify `ik_fcl_ompl_planner` to only plan for the right arm (7 DOF: right_shoulder_pitch → right_wrist_yaw), with correct self-collision checking of the right arm against ALL other body links. Remove all left-arm planning code. Fix the `isInCollision()` bug. Enforce URDF joint limits as OMPL bounds.

</domain>

<decisions>
## Implementation Decisions

### Left Arm Code Removal
- **D-01:** Completely delete all left arm code — KDL chain extraction, IK solver (`ik_left`), FK solver (`fk_left_solver`), `left_tip` parameter declaration, left arm joint limits arrays, and left arm debug logging. No disable/guard mechanism.
- **D-02:** Remove the y-coordinate arm selection logic (`bool use_right = pose_in_base.pose.position.y < 0.0`). Always use right arm regardless of goal position.

### Collision Checking
- **D-03:** Fix `isInCollision()` to check right arm links against ALL other body links (torso, legs, left arm), not just pairs where both links are in the planning chain. The current `||` condition must become `&&` (skip only if NEITHER link is in planning set).

### Claude's Discretion
- **Collision body-link transforms:** Claude decides the best approach for obtaining world-frame transforms of non-planning links (TF tree, full-tree FK, or static URDF defaults). Should balance performance, correctness, and implementation simplicity.

### URDF Model
- **D-04:** Use `g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf` for faster FCL collision checks. Ensure launch file and/or robot.launch.py is configured accordingly.

### Debug Logging Cleanup
- **D-05:** Clean up verbose debug logging in this phase. Remove redundant dumps (KDL chain structure, per-joint limits, per-state collision check strings, joint order comparisons). Keep essential logs: initialization success, planning start/completion, IK success/failure, collision detection results, errors.

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Source Code
- `src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` — The file being modified (903 lines, both-arm planner with collision bug)

### Robot Model
- `src/unitree_g1_dex3_stack-main/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf` — Selected URDF with collision primitives

### Supporting Files
- `src/unitree_g1_dex3_stack-main/include/g1_dex3_joint_defs.hpp` — Joint enums and name→index maps
- `src/unitree_g1_dex3_stack-main/launch/planner.launch.py` — Planner launch configuration (may need URDF path update)
- `src/unitree_g1_dex3_stack-main/launch/robot.launch.py` — Robot model launch (publishes URDF to robot_state_publisher)

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- `buildCollisionObjects()` — Already loads ALL URDF links' collision geometry into `link_collisions` map. No change needed for full-body collision support.
- `tf_buffer_` / `tf_listener_` — TF2 infrastructure already initialized, can be used for body-link transforms.
- `joint_limits_` map — Already populated from URDF for all joints, used for OMPL bounds.
- `latest_joint_positions_` — Updated from `/joint_states` callback, has all robot joint positions.

### Established Patterns
- URDF-at-runtime: Node fetches URDF from `/robot_state_publisher` service at startup.
- QoS best-effort for hardware topics.
- RCLCPP logging macros (INFO, WARN, ERROR, FATAL).

### Integration Points
- Subscribes to `/goal_pose` (PoseStamped) — no change needed.
- Publishes to `/joint_trajectory_targets` (JointTrajectory) — output will contain only right arm joints.
- Subscribes to `/joint_states` — already receives all robot joints.

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

*Phase: 01-right-arm-only-planner*
*Context gathered: 2025-04-28*
