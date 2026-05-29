# Pitfalls Research: v1.1

**Scope:** Adding AprilTag 36h11 detection, TCP offset integration, and adaptive end-effector orientation to existing OMPL+FCL+TRAC-IK right-arm planner on Unitree G1.

**Date:** 2026-05-15

---

## AprilTag + RealSense Pitfalls

### P1: IR Projector Interference with Tag Detection

**What goes wrong:** The D435i's active IR projector creates a dot pattern that overlays the AprilTag in the IR image. If using the infrared stream for detection (or if the IR dots bleed into the RGB image under certain lighting), detection rate drops or pose estimation becomes noisy.

**Warning signs:** Intermittent detection failures, tag detected at close range but not at medium range, noisy pose oscillation.

**Prevention:**
- Use the **color (RGB) stream** for AprilTag detection, not the IR stream.
- If depth is needed for the tag's 3D position, use the aligned depth-to-color stream (`align_depth_to_color:=true` in realsense-ros).
- Consider disabling the IR emitter (`enable_infra:=false`) if it causes visible dot interference in the color image under low light. Test empirically.

**Phase:** Detection implementation (first).

---

### P2: Tag Size vs Detection Distance Mismatch

**What goes wrong:** AprilTag pose estimation accuracy degrades rapidly with distance. A tag that's too small becomes undetectable beyond ~0.5m. The G1's head camera is ~1.0-1.5m from the workspace, so a small tag (e.g., 3cm) may be at the edge of reliable detection.

**Warning signs:** Tag detected only when arm is already near the target; pose estimate has >2cm error; detection rate <80% at operating distance.

**Prevention:**
- **Minimum tag size rule:** Tag side length should be ≥ (detection_distance / 20). For 1.0m distance, use ≥5cm tags. For 1.5m, use ≥7.5cm.
- Measure actual detection distance from G1 head camera to workspace and size tag accordingly.
- Use `tag36h11` family (already chosen) — it has good error correction for partial occlusion.
- Log detection rate and pose variance during integration testing.

**Phase:** Hardware setup / detection implementation.

---

### P3: Camera Yaw Angle Degrades Pose Accuracy

**What goes wrong:** Research shows that the primary source of AprilTag pose error is the angular rotation (yaw) of the camera relative to the tag center. When the camera optical axis doesn't point directly at the tag center, position error increases dramatically (from ~1cm to >10cm at 70cm distance with 20° offset). The G1 head camera has a fixed orientation — if the tag is at the workspace edge, the viewing angle may be oblique.

**Warning signs:** Consistent position offset that varies with tag placement; X-axis error much larger than Z-axis error; error increases when tag is placed to the side.

**Prevention:**
- Place the AprilTag in the **central field of view** of the head camera where possible.
- Apply the configurable offset from tag center to actual target position (already planned) — but be aware this offset itself is subject to the same angular error.
- Consider temporal averaging (low-pass filter) on pose estimates to reduce noise.
- Validate pose accuracy empirically at the actual operating position before trusting it for IK goals.

**Phase:** Detection implementation + integration testing.

---

### P4: Incorrect Camera Intrinsics for Pose Estimation

**What goes wrong:** AprilTag pose estimation requires accurate camera intrinsics (fx, fy, cx, cy). If using wrong intrinsics (e.g., from a different resolution mode, or default values), the 3D pose will be systematically wrong — especially depth (Z).

**Warning signs:** Tag detected but reported position is consistently wrong in depth; position scales incorrectly with distance.

**Prevention:**
- Use the camera_info topic from the RealSense driver (it provides calibrated intrinsics for the active resolution).
- Ensure the AprilTag detector receives intrinsics matching the actual image resolution being processed.
- If downscaling images for performance, scale intrinsics accordingly (fx, fy, cx, cy all scale linearly with resolution).

**Phase:** Detection implementation.

---

### P5: Frame ID Mismatch Between Detection and TF Tree

**What goes wrong:** The AprilTag detector publishes poses in `camera_color_optical_frame` (or similar), but the planner expects goals in `torso_link`. If the frame_id in the published PoseStamped doesn't match what TF2 can resolve, the transform fails silently or produces garbage.

**Warning signs:** TF lookup exceptions in planner logs; goal position wildly wrong (meters off); planner receives goal but IK always fails.

**Prevention:**
- Verify the exact frame_id the AprilTag detector publishes (it should be the camera's optical frame).
- Ensure the static TF from `camera_color_optical_frame` → `torso_link` is published (via the existing camera-to-robot calibration).
- The existing planner already handles frame transformation via `tf_buffer_.transform()` — just ensure the detection node sets the correct `header.frame_id`.

**Phase:** Detection implementation + integration.

---

### P6: Tag Pose Ambiguity (Flip)

**What goes wrong:** AprilTag pose estimation can produce a "flipped" solution where the tag appears to be behind the camera or rotated 180°. This happens especially at oblique viewing angles or when the tag is small relative to image resolution.

**Warning signs:** Occasional wild jumps in reported tag Z position (e.g., suddenly negative); orientation flips by 180°.

**Prevention:**
- Filter detections: reject any pose where Z (depth) is negative or outside expected workspace bounds.
- Use the `decision_margin` field from the AprilTag detector — reject detections with low margin (<30).
- If using `apriltag_ros`, enable pose refinement and set appropriate tag size.

**Phase:** Detection implementation.

---

## TCP Offset Pitfalls

### P7: Offset Applied in Wrong Frame

**What goes wrong:** The TCP offset (0.175m along local X of `right_wrist_yaw_link`) must be applied in the **link-local frame**, not the world/base frame. If applied in `torso_link` frame, the offset direction changes with arm configuration, causing the actual TCP to miss the target.

**Warning signs:** TCP arrives at correct position only when arm is in one specific configuration; error varies with arm pose; offset seems to "rotate" with the wrist.

**Prevention:**
- The IK target frame must be computed as: `T_target = T_desired_tcp * T_tcp_to_wrist_inv`, where `T_tcp_to_wrist` is the fixed transform from `right_wrist_yaw_link` to TCP (0.175m along local X, no rotation).
- In KDL terms: create the target frame by "backing off" the TCP offset from the desired TCP position, in the TCP's own frame.
- **Correct formula:** `target_for_IK = desired_tcp_pose * KDL::Frame(KDL::Vector(-0.175, 0, 0))` — this moves the IK target 0.175m back along the TCP's X axis.

**Phase:** TCP offset integration (critical to get right first time).

---

### P8: Offset Direction Confusion (X vs Z, positive vs negative)

**What goes wrong:** The existing `tcp_torso_pose.py` script uses `tcp_offset_x = 0.145m` along +X of `right_wrist_yaw_link`. The PROJECT.md states 0.175m. If the offset magnitude or axis is wrong, the arm will consistently over/undershoot.

**Warning signs:** Arm reaches but TCP is consistently offset from target by a fixed amount; the offset is always in the same direction relative to the wrist.

**Prevention:**
- **Verify the actual TCP offset empirically** before coding: move arm to a known position, measure physical TCP position vs `right_wrist_yaw_link` TF.
- Reconcile the 0.145m (from script) vs 0.175m (from PROJECT.md) discrepancy — one is likely outdated.
- Make the offset a ROS parameter so it can be tuned without recompilation.
- The offset axis is confirmed as local +X of `right_wrist_yaw_link` (from the TCP context doc).

**Phase:** TCP offset integration.

---

### P9: TRAC-IK Chain Doesn't Include TCP — Two Approaches, Each with Traps

**What goes wrong:** TRAC-IK solves for the tip link of the KDL chain (`right_wrist_yaw_link`). The TCP is a virtual point beyond this link. Two approaches exist:

1. **Pre-compute IK target** (back off TCP offset from desired pose) — simpler but requires correct frame math.
2. **Extend KDL chain** with a virtual fixed segment for TCP — cleaner but requires modifying URDF or chain construction.

If approach 1 is used incorrectly (wrong frame for offset), IK solves for wrong pose. If approach 2 is used, the FK solver, collision checker, and OMPL state validity all need updating.

**Warning signs:** IK succeeds but FK of solution doesn't match desired TCP pose; collision checker flags false positives on the virtual link.

**Prevention:**
- **Recommended: Approach 1** (pre-compute target) — it's less invasive to the existing planner.
- Validate by: solving IK for the computed target, then running FK + applying TCP offset, and checking it matches the original desired pose (within tolerance).
- Add a unit test: `desired_tcp → compute_ik_target → IK → FK → apply_offset → compare with desired_tcp`.

**Phase:** TCP offset integration.

---

### P10: TCP Offset Breaks Collision Checking

**What goes wrong:** The collision checker validates the `right_wrist_yaw_link` position but doesn't know about the TCP extension. The arm could plan a path where the wrist is collision-free but the TCP (0.175m further out) collides with the environment or body.

**Warning signs:** Planned paths appear valid but physical TCP hits obstacles; near-misses that should be flagged aren't.

**Prevention:**
- Add a collision geometry (small sphere or cylinder, ~3cm radius, 17.5cm length) attached to `right_wrist_yaw_link` representing the TCP extension.
- Or: add the TCP as a virtual link in the URDF with a collision element.
- At minimum: add a workspace boundary check that the TCP position (computed via FK + offset) is within safe bounds for every waypoint.

**Phase:** TCP offset integration + collision checking update.

---

## Adaptive Orientation Pitfalls

### P11: Fixed Orientation Causes IK Failure at Workspace Boundaries

**What goes wrong:** The current system uses a hardcoded orientation quaternion (`[-0.682, 0.068, -0.078, 0.724]` in `detection_to_goal_node.cpp`). This orientation is only achievable in a subset of the workspace. When the target is at the edge of reachable space, IK fails because the 7-DOF arm cannot simultaneously reach the position AND match the fixed orientation.

**Warning signs:** IK failure rate >30%; failures cluster at specific workspace regions; planner logs "IK failed with both current and neutral seed."

**Prevention:**
- Implement adaptive orientation: compute a feasible orientation based on target position relative to shoulder.
- Strategy: point the TCP's approach axis (local X) toward the target from the shoulder, with gravity-aligned roll.
- Allow orientation tolerance in IK (TRAC-IK supports bounded orientation tolerance via the `bounds` parameter).

**Phase:** Adaptive orientation implementation.

---

### P12: Orientation Singularity Near Shoulder Overhead

**What goes wrong:** When the target is directly above or very close to the shoulder, the "point toward target" heuristic produces a degenerate orientation (gimbal lock in Euler angles, or the approach vector becomes parallel to the arm's first joint axis). IK may find a solution but it requires extreme joint angles.

**Warning signs:** Wild joint configurations for overhead targets; arm takes a very circuitous path; joint velocity limits exceeded.

**Prevention:**
- Define workspace boundaries: reject targets that are too close to the shoulder (<0.15m) or directly overhead.
- For targets near singularity zones, use a "default safe orientation" rather than the computed adaptive one.
- Add a reachability check before planning: compute distance from shoulder to target, reject if outside [0.15m, 0.65m] range (approximate for G1 right arm).

**Phase:** Adaptive orientation implementation.

---

### P13: Orientation Discontinuity Between Adjacent Targets

**What goes wrong:** If the adaptive orientation changes abruptly between two nearby targets (e.g., crossing a workspace boundary where the heuristic switches strategy), the arm may need to completely reconfigure between consecutive reaches, causing large unnecessary motions.

**Warning signs:** Arm makes large reconfigurations for small target position changes; planning time spikes for certain target transitions.

**Prevention:**
- Use smooth interpolation for orientation as a function of target position (e.g., SLERP between boundary orientations).
- Avoid hard if/else boundaries in orientation selection — use continuous functions.
- Consider the previous arm configuration as a hint for orientation selection (prefer orientations that are close to current).

**Phase:** Adaptive orientation implementation.

---

### P14: Orientation Tolerance Too Loose — Grasp Axis Misaligned

**What goes wrong:** If orientation tolerance is set too wide to improve IK success rate, the TCP may arrive at the target with an orientation that's physically useless (e.g., approaching from below when the object is on a table).

**Warning signs:** IK always succeeds but the physical approach direction is wrong; TCP arrives but can't interact with the target meaningfully.

**Prevention:**
- Define acceptable approach cone (e.g., ±30° from desired approach direction).
- Use TRAC-IK's `Manip1` or `Manip2` solve type to prefer solutions with better manipulability.
- Validate the final orientation of the IK solution against the acceptable cone before sending to OMPL.

**Phase:** Adaptive orientation implementation.

---

## Integration Pitfalls

### P15: Race Condition Between Detection and Planning

**What goes wrong:** AprilTag detection publishes a pose, the planner receives it and starts planning. During planning (~1s), the robot may have shifted slightly (running mode micro-adjustments), making the TF snapshot stale. The planned trajectory arrives at where the target *was*, not where it *is*.

**Warning signs:** Consistent small offset between planned and actual reach position; offset varies between trials; worse when robot is less stable.

**Prevention:**
- Snapshot the TF tree at detection time (use the detection message timestamp for TF lookups, not `tf2::TimePointZero`).
- Consider re-validating the target position just before execution starts.
- For the G1 in running mode, body sway is typically <1cm — this may be acceptable.

**Phase:** Integration.

---

### P16: Removing YOLO Breaks Existing Launch Files

**What goes wrong:** The existing pipeline has YOLO → project_to_3d → detection_to_goal → planner. Removing YOLO and replacing with AprilTag requires updating all launch files, topic remappings, and message types. If done partially, nodes subscribe to non-existent topics and hang silently.

**Warning signs:** Nodes start but never receive messages; `ros2 topic hz` shows 0 messages on expected topics; system appears to work but never triggers planning.

**Prevention:**
- Map out the complete topic graph before and after the change.
- Create a new launch file for the v1.1 pipeline rather than modifying existing ones (keep v1.0 as fallback).
- Use `ros2 topic list` and `ros2 node info` to verify all connections after launch.

**Phase:** Integration (first step).

---

### P17: AprilTag Detector Output Format Mismatch

**What goes wrong:** The existing planner subscribes to `/goal_pose` (PoseStamped). The `apriltag_ros` package publishes to `/detections` (AprilTagDetectionArray). If the new detection node doesn't convert to PoseStamped with the correct fields, the planner never receives goals.

**Warning signs:** AprilTag detected (visible in detector logs) but planner never triggers; topic type mismatch errors.

**Prevention:**
- Write a bridge node (or modify detection_to_goal_node) that:
  1. Subscribes to AprilTag detections
  2. Extracts pose for the configured tag ID
  3. Applies configurable offset (tag center → actual target)
  4. Computes adaptive orientation
  5. Publishes PoseStamped on `/goal_pose`
- This node replaces both `ultralytics_detector` + `project_to_3d_node` + `detection_to_goal_node`.

**Phase:** Detection → planner bridge implementation.

---

### P18: TCP Offset + Adaptive Orientation Interaction

**What goes wrong:** The TCP offset and adaptive orientation are not independent. The IK target computation is:
1. Start with desired TCP pose (position from detection + adaptive orientation)
2. Back off TCP offset to get wrist target

If the adaptive orientation is computed *after* the offset is applied (wrong order), or if the offset is applied in world frame instead of the oriented TCP frame, the result is wrong.

**Warning signs:** IK solutions that look correct in isolation but the physical TCP doesn't reach the target; error depends on orientation.

**Prevention:**
- **Correct order:**
  1. Compute desired TCP position (from AprilTag + configurable offset)
  2. Compute desired TCP orientation (adaptive, based on position)
  3. Combine into desired TCP pose (KDL::Frame)
  4. Apply TCP-to-wrist inverse transform to get IK target
- Validate the full chain: `detection → offset → orientation → TCP pose → IK target → IK → FK → TCP offset → verify matches step 3`.

**Phase:** Integration of all three features.

---

### P19: OMPL Planning Time Increases with Tighter Orientation Constraints

**What goes wrong:** When the IK goal has a specific orientation (not just position), the goal state in joint space is more constrained. OMPL may take longer to find a valid path, or fail within the timeout, because the goal configuration is in a narrow region of C-space.

**Warning signs:** Planning timeout failures increase after adding orientation constraints; planner succeeds for some orientations but not others at the same position.

**Prevention:**
- Increase planning timeout for initial testing (2-3s instead of 1s).
- Use multiple IK seeds (already implemented — 20 random tries) to find diverse goal configurations.
- If IK finds a goal but OMPL can't reach it, try alternative orientations within the acceptable cone.
- Consider a two-phase approach: first try preferred orientation, if OMPL fails within timeout, relax orientation and retry.

**Phase:** Integration testing + tuning.

---

### P20: Existing Planner Has Redundant IK Calls

**What goes wrong:** Looking at the current `ik_fcl_ompl_planner.cpp` (lines ~450-470), there are TWO IK calls in sequence — one with error handling and one without (`if (!solver->CartToJnt(seed, target_frame, goal))`). The second call overwrites the first result. This is likely a bug that will interact badly with the new TCP offset logic.

**Warning signs:** IK appears to succeed (first call) but then the result is overwritten by a failed second call; planner aborts unexpectedly.

**Prevention:**
- Fix this bug before adding TCP offset logic: remove the redundant second `CartToJnt` call.
- The correct flow should be: try current seed → if fail, try neutral seed → if fail, abort.
- This is existing technical debt that will cause confusion when modifying the IK target computation.

**Phase:** Pre-work before TCP offset integration.

---

## Prevention Checklist

### Before Implementation

- [ ] Measure actual TCP offset on physical robot (resolve 0.145m vs 0.175m discrepancy)
- [ ] Measure detection distance from G1 head camera to workspace
- [ ] Choose AprilTag size based on detection distance (≥ distance/20)
- [ ] Print and mount test AprilTag, verify detection rate >95% at operating distance
- [ ] Verify camera intrinsics match the resolution used for detection
- [ ] Verify TF chain: `camera_color_optical_frame` → `torso_link` is published and correct
- [ ] Fix redundant IK call bug in existing planner (P20)

### During Detection Implementation

- [ ] Use RGB stream (not IR) for AprilTag detection
- [ ] Set correct `tag_size` parameter in detector
- [ ] Publish detections with correct `frame_id` (camera optical frame)
- [ ] Filter detections: reject negative Z, low decision_margin, out-of-bounds
- [ ] Log detection rate and pose variance at operating distance
- [ ] Temporal filtering (low-pass) on pose estimates

### During TCP Offset Implementation

- [ ] Apply offset in link-local frame, not world frame
- [ ] Validate: FK(IK_solution) + TCP_offset ≈ desired_TCP_pose
- [ ] Add TCP collision geometry to FCL checker
- [ ] Make offset a ROS parameter (tunable without recompile)
- [ ] Unit test the offset math independently

### During Adaptive Orientation Implementation

- [ ] Define workspace boundaries (min/max reach distance)
- [ ] Implement smooth orientation function (no discontinuities)
- [ ] Handle singularity zones (overhead, too close)
- [ ] Validate orientation is within acceptable approach cone
- [ ] Test at workspace boundaries specifically

### During Integration

- [ ] Create new launch file (don't break v1.0)
- [ ] Verify complete topic graph with `ros2 topic list` / `ros2 node info`
- [ ] Correct computation order: position → orientation → TCP pose → IK target
- [ ] Increase planning timeout for initial testing (2-3s)
- [ ] End-to-end validation: place tag → detect → plan → execute → measure TCP position error
- [ ] Test with tag at multiple workspace positions (center, edges, near/far)

---

## Summary of Critical Path

The highest-risk pitfalls that are most likely to cause multi-day debugging:

1. **P7 (Wrong frame for TCP offset)** — Silent error, arm reaches wrong position consistently
2. **P11 (Fixed orientation → IK failure)** — Already observed in current system, primary motivation for v1.1
3. **P18 (TCP + orientation interaction)** — Subtle ordering bug, hard to diagnose
4. **P20 (Redundant IK call)** — Existing bug that will confuse new development
5. **P2 (Tag size vs distance)** — Hardware constraint that can't be fixed in software

Address P20 first (cleanup), then P7+P9 (TCP offset math), then P11-P14 (orientation), then P15-P19 (integration).
