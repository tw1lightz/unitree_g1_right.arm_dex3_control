# Phase 8: 自适应末端位姿 - Context

**Gathered:** 2026-05-18
**Status:** Ready for planning

<domain>
## Phase Boundary

Phase 08 delivers adaptive end-effector orientation inside the existing planner. The planner keeps the `/goal_pose` input interface, uses the goal position as the target TCP position, and computes a feasible `right_tcp_link` orientation from the target position relative to the right shoulder before calling TRAC-IK/OMPL.

This phase does not modify AprilTag detection, does not bridge `/apriltag/target_pose` to `/goal_pose`, does not create the end-to-end `apriltag_reach.launch.py`, and does not execute the robot. Phase 09 owns end-to-end integration.

</domain>

<decisions>
## Implementation Decisions

### A — Adaptive orientation strategy

- **D-01:** `right_tcp_link` local `+X` is the approach axis and must point from the right shoulder reference point toward the target position. This matches Phase 6 TCP semantics: `right_tcp_link` is offset by +0.175 m along `right_wrist_yaw_link` local X.
- **D-02:** Roll around the generated `+X` approach axis should be stabilized using `torso_link` `+Z` as the up reference. The generated frame should keep the TCP orientation visually/numerically stable rather than allowing arbitrary wrist roll.
- **D-03:** If the approach direction is near-parallel to `torso_link` `+Z`, use `torso_link` `+Y` as the fallback reference axis for orthonormal frame construction. This keeps the orientation deterministic near vertical targets.
- **D-04:** Phase 08 generates exactly one deterministic orientation per goal. Multi-candidate roll/orientation fallback is out of scope and remains Future ORI-02.

### B — Right shoulder reference point

- **D-05:** The shoulder reference point is the origin of `right_shoulder_pitch_link`.
- **D-06:** The planner should compute the `right_shoulder_pitch_link` origin in `base_link_`/`torso_link` using the already-loaded URDF/KDL tree, not by hardcoding coordinates and not by relying on runtime TF lookup.
- **D-07:** The shoulder link name is hardcoded as `right_shoulder_pitch_link`. Do not add a ROS parameter for the shoulder link in Phase 08.
- **D-08:** If the target position is too close to the shoulder reference point for a stable direction vector, reject the goal with a clear error and do not publish a trajectory.

### C — `/goal_pose.orientation` overwrite behavior

- **D-09:** By default, planner ignores/overwrites incoming `/goal_pose.pose.orientation`. It keeps the incoming position and replaces orientation with the adaptive orientation generated from the shoulder-to-target direction.
- **D-10:** Add `adaptive_orientation_enabled` as a ROS parameter, default `true`.
- **D-11:** When `adaptive_orientation_enabled=false`, preserve the old planner behavior exactly: use the incoming `/goal_pose.pose.orientation` directly as the TRAC-IK target orientation.
- **D-12:** When adaptive orientation is enabled, log one INFO line per goal with the target xyz, shoulder xyz, normalized direction, and generated quaternion so field debugging can confirm the pose is reasonable.

### D — Verification scope

- **D-13:** Verify fixed-orientation vs adaptive-orientation using a fixed A/B test set of AprilTag-common tabletop target positions in `torso_link`.
- **D-14:** The user selected tabletop-only UAT. Do not require workspace-boundary or shoulder-overhead target coverage for Phase 08 human verification, even though the implementation still includes the vertical-direction fallback from D-03.
- **D-15:** Success criterion for the selected tabletop UAT: with `adaptive_orientation_enabled=true`, every target in the chosen tabletop test set must produce a successful planner result and publish `/joint_trajectory_targets`.
- **D-16:** Verification is planner-only. Start robot model + planner, publish `/goal_pose`, observe TRAC-IK/OMPL logs and `/joint_trajectory_targets`; do not run the executor or move the physical robot for this phase.

### Claude's Discretion

- Pick exact numeric thresholds for near-zero shoulder-to-target distance and near-parallel up-reference detection, as long as they are conservative and logged clearly.
- Choose the exact orthonormal basis construction implementation in C++ as long as D-01 through D-04 are preserved.
- Choose whether the tabletop A/B check is manual commands or a small helper script during planning, as long as the verification output clearly reports per-target success/failure.

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Project requirements and phase scope

- `.planning/PROJECT.md` — Current milestone goal and key decision that fixed end-effector poses are causing IK/OMPL failures.
- `.planning/REQUIREMENTS.md` — `ORI-01`: compute feasible end-effector orientation from target position relative to shoulder.
- `.planning/ROADMAP.md` — Phase 8 goal and success criteria.
- `.planning/STATE.md` — Current state: Phase 8 pending; Phase 7 complete; Phase 9 owns end-to-end integration.

### Prior phase constraints

- `.planning/phases/06-yolo-tcp-offset/06-CONTEXT.md` — Phase 6 integrated TCP offset through `right_tcp_link`; planner `/goal_pose` interface remains unchanged.
- `.planning/phases/07-apriltag/07-CONTEXT.md` — Phase 7 publishes `/apriltag/target_pose` in `torso_link`, but Phase 8 does not bridge it to `/goal_pose`; Phase 9 owns integration.
- `.planning/phases/05-end-to-end-integration/05-CONTEXT.md` — Historical `/goal_pose` planner input and keyboard trigger context; old YOLO path has since been superseded by Phase 6/7.

### Planner source and launch entry points

- `src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` — Main implementation target. Current flow transforms `/goal_pose` into `base_link_`, constructs `KDL::Frame` from `pose_in_base.pose.orientation`, then calls TRAC-IK `CartToJnt`.
- `src/unitree_g1_dex3_stack-main/launch/planner.launch.py` — Add/pass `adaptive_orientation_enabled` launch parameter if planning chooses launch-level exposure.
- `src/unitree_g1_dex3_stack-main/scripts/tcp_torso_pose.py` — Reference for TCP offset semantics and KDL/FK style; TCP offset is along local +X.

### URDF references

- `src/unitree_g1_dex3_stack-main/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0.urdf` — Defines `right_shoulder_pitch_link`, `right_shoulder_pitch_joint`, `right_wrist_yaw_link`, `right_tcp_joint`, and `right_tcp_link`.
- `src/unitree_g1_dex3_stack-main/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf` — Collision-primitives planner URDF with the same right-arm/TCP link semantics.

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets

- `ik_fcl_ompl_planner.cpp` already loads URDF into `urdf_model`, builds `kdl_tree`, extracts `kdl_chain_right`, and owns `base_link_`/`right_tip_`; adaptive orientation should live in this node rather than a new node.
- `ik_fcl_ompl_planner.cpp` currently receives `/goal_pose` in `goalPoseCallback`, transforms it to `base_link_`, then creates `KDL::Frame target_frame` from position + orientation. The adaptive overwrite should happen after transform to base frame and before `target_frame` construction.
- `planner.launch.py` already follows the existing `DeclareLaunchArgument` + `OpaqueFunction` + parameter dict pattern for planner parameters.
- `tcp_torso_pose.py` documents and demonstrates the `+X` TCP offset semantics that Phase 08 orientation must respect.

### Established Patterns

- Planner parameters use `declare_parameter` and `get_parameter` in `ik_fcl_ompl_planner.cpp`.
- Planner logs use `RCLCPP_INFO/WARN/ERROR`, with fatal errors reserved for startup failures.
- Planning remains right-arm-only and expects 7 right-arm joints in the KDL chain.
- The project favors surgical planner changes over adding new packages or nodes for planner-internal behavior.

### Integration Points

- `/goal_pose` remains the planner input topic. Phase 08 should not require upstream nodes to change what they publish.
- `/joint_trajectory_targets` remains the planner output topic and the planner-only verification signal.
- `right_tip` default is `right_tcp_link`; adaptive orientation targets that TCP frame, not `right_wrist_yaw_link`.
- `adaptive_orientation_enabled=false` is the A/B baseline and must restore old behavior for comparison.

</code_context>

<specifics>
## Specific Ideas

- Generate an orthonormal frame in `torso_link` where generated x-axis equals `normalize(target - shoulder)`.
- Use `torso +Z` as the primary up reference; if `abs(dot(direction, up))` is near 1, switch to `torso +Y`.
- The generated quaternion should be written back into `pose_in_base.pose.orientation` before the existing `KDL::Frame target_frame` is built.
- Verification can use two planner runs over the same tabletop target set: one with `adaptive_orientation_enabled=false`, one with `true`. The user-selected pass requirement is that the adaptive run succeeds for all tabletop targets.
- The fixed-orientation baseline can use existing old behavior with whatever orientation the test `/goal_pose` messages contain.

</specifics>

<deferred>
## Deferred Ideas

- **Multi-candidate orientation fallback (Future ORI-02)** — Trying several roll angles or approach orientations after IK failure is out of scope for Phase 08.
- **Tag-normal-based approach direction (Future ORI-03)** — Using AprilTag pose normal to determine approach direction is out of scope; Phase 08 uses target position relative to shoulder only.
- **Workspace boundary / shoulder-overhead UAT gap** — The implementation keeps vertical fallback, but the user selected tabletop-only human verification. Full UAT coverage for workspace boundary and shoulder-overhead cases is deferred or should be treated as a known verification gap against the original roadmap wording.

</deferred>

---

*Phase: 08-adaptive-orientation*
*Context gathered: 2026-05-18*
