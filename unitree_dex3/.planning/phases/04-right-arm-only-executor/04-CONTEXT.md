# Phase 4: Right-Arm-Only Executor - Context

**Gathered:** 2026-05-13
**Status:** Ready for planning

<domain>
## Phase Boundary

Modify `joint_trajectory_executor.cpp` so that `LowCmd` only populates the 7 right-arm joints (`kRightShoulderPitch` through `kRightWristYaw`) in all three publish loops (waypoint, hold, exit ramp). Non-right-arm `motor_cmd[]` slots are left at default (zero/untouched) so the official running mode keeps authority on legs/waist/left-arm. Hand open/close `publish()` calls are removed (DEX3 control is out of scope for this phase). Trajectory side detection, hand publishers, the hand-open `sleep_for(1s)`, and the master-switch ramp pattern (`kNotUsedJoint.q` 0.5 → 0 over 3 s at 250 Hz) established by Plans 01-09 / 01-11 / 01-12 are preserved as inert/legacy code.

</domain>

<decisions>
## Implementation Decisions

### Hand Control Removal Scope
- **D-01:** Minimal removal — delete only the two `hand_cmd_pub->publish(hand_cmd)` calls (the `hand_cmd.data = false` open at L181-182 and the `hand_cmd.data = true` close at L269-270). Keep all surrounding infrastructure as inert/legacy code:
  - `left_hand_pub_` and `right_hand_pub_` member declarations (L134-135)
  - The `create_publisher<std_msgs::msg::Bool>` calls in the constructor (L58-59)
  - The `is_left_hand` detection block at L158-168 (still selects `hand_cmd_pub` even though it's no longer used to publish)
  - The `sleep_for(1s)` at L184 (now functions as an inert settling delay before trajectory begins)
- **D-02:** No `[[maybe_unused]]` annotations or compiler-warning suppression added. If the unused `hand_cmd_pub` local variable triggers a warning, leave it; future re-enabling of hand control just re-introduces a `publish()` call.

### Trajectory Side Validation
- **D-03:** Two-stage validation runs first thing in `trajectoryCallback`, before any motion logic:
  1. **Foreign-joint warning + strip:** If `joint_names` contains any name outside the right-arm set `{right_shoulder_pitch_joint, right_shoulder_roll_joint, right_shoulder_yaw_joint, right_elbow_joint, right_wrist_roll_joint, right_wrist_pitch_joint, right_wrist_yaw_joint}`, log `RCLCPP_WARN` listing the offending names and **strip** them (along with their column in every `point.positions` / `point.velocities` / `point.accelerations`) from the working copy. Continue execution with the stripped trajectory.
  2. **Completeness check:** After stripping, if the remaining right-arm joint set does not contain all 7 right-arm joint names, log `RCLCPP_ERROR` listing the missing names and `return` without publishing any `LowCmd`. Phase 1's planner always emits the complete 7-axis trajectory, so any partial trajectory is a misconfiguration.
- **D-04:** The existing empty-`joint_names` early-error at L154-157 (`RCLCPP_ERROR + return`) stays unchanged. The new completeness check at D-03 step 2 is a separate, stricter check; the empty case is naturally subsumed (an empty trajectory will fail the "7 right-arm joints present" check).
- **D-05:** Stripping happens on a local working copy of `joint_names` and a per-point index list, not by mutating `msg`. The original message stays read-only as in current code.

### Carried Forward (no re-decision needed)
- The 1 s post-trajectory hold loop (L272-294) and 3 s @ 250 Hz exit ramp (L307-340) — both narrow to the 7 right-arm joints during this phase, but the timing structure (1 s hold, 3 s ramp, 250 Hz, master `kNotUsedJoint.q` = 0.5 during waypoint+hold then linearly faded to 0.0 during ramp) stays exactly as tuned in Plans 01-09 / 01-11 / 01-12. Do NOT alter durations, frequencies, or the master ramp curve.
- The `latest_joint_positions_` / `standing_pose` / `trajectory_endpoint` baselines (Plan 01-09 / 01-12) are still tracked across all 28 indices because `lowstateCallback` continues to capture full state. Only the *write* loops are narrowed to the 7 right-arm joints.

### Claude's Discretion
- **D-06 (master-switch semantics under right-arm-only writes):** Phase 4 changes the prior assumption that all 28 `motor_cmd[]` slots are populated each frame. With non-right-arm slots left default-constructed (`q=0`, `dq=0`, `mode=0`, `kp=0`, `kd=0`, `tau=0`) and `kNotUsedJoint.q = 0.5` during the trajectory/hold window, the key question is whether firmware blends those default-zero slots into body controller output.
  - **Empirical evidence (2026-05-13):** User confirmed that with the current code (`master=0.5`, all 28 joints written with `mode=1` + current positions as baseline), only the right arm moved and the rest of the robot held the standing posture correctly throughout all prior phase testing. This indicates arm_sdk already coexists properly with the running mode body controller.
  - **Hypothesis:** `mode=0` slots are likely treated as "do not control" by arm_sdk firmware (consistent with Unitree HG series convention where mode=0 = PWM/voltage pass-through that arm_sdk does not command). Body controller retains full authority on those joints.
  - **Recommended approach for planner agent:** Implement Phase 4 with non-right-arm slots left at default (untouched). As the **first verification step**, execute a zero-displacement right-arm trajectory (start = current positions, end = current positions) and observe whether legs/waist exhibit any tremor or drift. If clean: proceed. If interference is observed: add an explicit write of `q=latest_joint_positions_[idx]` with `mode=0` for non-right-arm slots as a fallback. Do not pre-emptively add this unless the test fails.
  [RESOLVED 2026-05-14: Option A chosen — preserve 28-joint fill with kp=60 unchanged. Rationale: consistent with xr_teleoperate/free_arm_demo.py canonical coexistence pattern; 1+ year standing-mode evidence; eliminates D-06 firmware-default-zero risk without requiring bench-test gate. ROADMAP success criteria 1-2 updated accordingly in Plan 04-03 Task 2.]
- **Implementation strategy for narrowing the 3 publish loops:** Whether to filter via `joint_name_to_index` lookup of the 7 right-arm names, hardcode the 7 `JointIndex` enum values, or define a small `kRightArmIndices[]` array — Claude picks the cleanest option consistent with `g1_dex3_joint_defs.hpp` style.
- **Logging detail for stripped/missing joints:** Format and verbosity of the WARN/ERROR messages.

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Source Code (file being modified)
- `src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp` — The executor (368 lines). Key landmarks:
  - L58-59: hand publisher creation
  - L134-135: hand publisher members
  - L154-157: existing empty-joint_names guard (kept)
  - L158-168: `is_left_hand` detection (kept inert per D-01)
  - L181-185: hand-open publish + 1 s sleep (delete publish call only, keep sleep per D-01)
  - L193-232: waypoint loop — narrow writes to right arm
  - L269-270: hand-close publish (delete per D-01)
  - L272-294: 1 s hold loop — narrow writes to right arm, preserve timing
  - L307-340: 3 s exit ramp — narrow writes to right arm, preserve timing/master curve

### Joint Definitions
- `src/unitree_g1_dex3_stack-main/include/g1_dex3_joint_defs.hpp` — `JointIndex` enum and `joint_name_to_index` map. Right-arm range: `kRightShoulderPitch` (idx 22) through `kRightWristYaw` (idx 28). `kNotUsedJoint` (idx 29) is the `arm_sdk` master-switch slot.

### Robot Model
- `src/unitree_g1_dex3_stack-main/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf` — URDF carrying joint position/velocity limits used by the existing `joint_limits_` clamp at L219-220 (no changes needed in this phase, but planner agent should be aware).

### Prior Phase Decisions That Constrain This Phase
- `.planning/phases/01-right-arm-only-planner/01-CONTEXT.md` — Phase 1 D-01/D-02 (delete-not-disable left arm), D-05 (logging cleanup style)
- `.planning/phases/01-right-arm-only-planner/01-09-SUMMARY.md` — `standing_pose` snapshot pattern + q-interpolation in exit ramp
- `.planning/phases/01-right-arm-only-planner/01-11-SUMMARY.md` — 1 s hold loop + 250 Hz hold/ramp + master ramp; "Compound-Bug Ladder" of issues this fixed
- `.planning/phases/01-right-arm-only-planner/01-12-SUMMARY.md` — `trajectory_endpoint` capture pattern (single-threaded executor cannot rely on `latest_joint_positions_` being fresh during callback)
- `.planning/phases/03-trajectory-smoothing-validation/03-CONTEXT.md` — Phase 3 left the executor's L219-220 position clamp as defense-in-depth; not changed here

### External (firmware semantics — to be confirmed in research)
- Unitree `unitree_hg::msg::LowCmd` / `arm_sdk` topic semantics — specifically how `kNotUsedJoint.q` (master switch) blends `motor_cmd[]` slots that have `mode=0` and `q=0`. Per D-06, the planner agent must locate and cite the canonical Unitree reference (likely under Unitree SDK docs or the `unitree_sdk2` examples in the workspace).

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- `joint_name_to_index` map (header) — already maps `right_*_joint` names to their enum indices; can be used to derive the 7-element right-arm index set without hardcoding.
- `joint_limits_` map (executor) — already populated for all joints; the existing position clamp at L219-220 inside the waypoint loop continues to function for the right-arm joints we now write.
- `lowstateCallback` (L141-150) — continues to populate `latest_joint_positions_` for all 28 joints; no change needed. Provides the snapshot used by `standing_pose` / `trajectory_endpoint`.
- `g_shutdown_requested` SIGINT handling (Plan 01-04 / 01-06) — unchanged; the loop break-out semantics still apply to the narrowed waypoint loop.

### Established Patterns
- **RCLCPP logging** — `RCLCPP_INFO` for normal flow events, `RCLCPP_WARN` for recoverable anomalies (use for D-03 step 1), `RCLCPP_ERROR` for rejected trajectories (use for D-03 step 2 and existing empty-joint_names guard).
- **Master-switch authority pattern** — `kNotUsedJoint.q` is the single arm_sdk blend knob; it is published in every frame of all three loops at the same value as before. Do not introduce per-joint authority knobs.
- **Joint limits clamp inside write loop** — Existing pattern at L219-220 stays inside the narrowed waypoint loop.

### Integration Points
- **Subscribers unchanged:** `/joint_trajectory_targets` (input from planner), `/lf/lowstate` (input from robot).
- **Publishers:** `cmd_pub_` (`/arm_sdk`) — unchanged in topic and QoS, only the per-frame payload is narrowed. `left_hand_pub_` / `right_hand_pub_` — kept as inert publishers per D-01.
- **No CMakeLists.txt change** — the executor target stays.
- **No launch file change** — `control.launch.py` still launches the same executable with the same parameters.

</code_context>

<specifics>
## Specific Ideas

- The user is comfortable with the executor doing two-stage validation (warn-and-strip foreign joints; reject incomplete right-arm sets) — this matches the project's existing defense-in-depth posture (executor's pre-publish position clamp from Phase 3, planner's post-plan validation from Phase 3).
- The user explicitly chose minimal-removal for hand code rather than the Phase-1-style full deletion; rationale (paraphrased from selection): keep the diff small and revertible since DEX3 grasping is plausibly a future phase.

</specifics>

<deferred>
## Deferred Ideas

- **Future re-enabling of DEX3 hand control** — when added, simply re-introduce `hand_cmd_pub->publish(...)` calls; the publishers and detection logic still exist. Belongs in a future phase (out of v1.0 scope per PROJECT.md).
- **Phase-1-style full hand-code deletion** — explicitly considered and rejected per D-01. If a future cleanup phase removes inert code, this is on the list.
- **Master-switch retuning under right-arm-only writes** — if D-06 research finds the current 0.5/ramp pattern is unsafe with default-zero non-right-arm slots, the fix may merit its own remediation plan rather than being folded into Phase 4.

</deferred>

---

*Phase: 04-right-arm-only-executor*
*Context gathered: 2026-05-13*
