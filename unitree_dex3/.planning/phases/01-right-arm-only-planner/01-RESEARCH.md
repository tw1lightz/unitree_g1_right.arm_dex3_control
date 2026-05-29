# Phase 1: Right-Arm-Only Planner — Research

**Researched:** 2026-04-29
**Domain:** ROS 2 motion planning (OMPL + FCL + TRAC-IK) on Unitree G1 humanoid
**Confidence:** HIGH

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions (NON-NEGOTIABLE)
- **D-01:** Completely delete all left arm code — KDL chain extraction, IK solver (`ik_left`), FK solver (`fk_left_solver`), `left_tip` parameter declaration, left arm joint limits arrays, and left arm debug logging. No disable/guard mechanism.
- **D-02:** Remove the y-coordinate arm selection logic (`bool use_right = pose_in_base.pose.position.y < 0.0`). Always use right arm regardless of goal position.
- **D-03:** Fix `isInCollision()` to check right arm links against ALL other body links (torso, legs, left arm), not just pairs where both links are in the planning chain. The current `||` condition must become `&&` (skip only if NEITHER link is in planning set).
- **D-04:** Use `g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf` for faster FCL collision checks. Ensure launch file and/or robot.launch.py is configured accordingly.
- **D-05:** Clean up verbose debug logging in this phase. Remove redundant dumps (KDL chain structure, per-joint limits, per-state collision check strings, joint order comparisons). Keep essential logs: initialization success, planning start/completion, IK success/failure, collision detection results, errors.

### Claude's Discretion
- **Body-link world-frame transforms** for non-planning links (torso, legs, left arm) used during collision checking. Choices: TF buffer lookup, full-tree KDL FK, or static URDF defaults. Must balance correctness, robustness, and implementation simplicity.

### Deferred Ideas (OUT OF SCOPE)
- None — discussion stayed within phase scope. Note: OMPL `setStateValidityCheckingResolution` and path simplification belong to Phase 2 (PLAN-03), not this phase.
</user_constraints>

<bug_root_cause_analysis>
## Bug Root Cause Analysis

### Bug 1: `isInCollision()` filter inverts the intent
**Location:** `src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp:847-851`

**Current code:**
```cpp
if (!planning_links.empty() &&
    (planning_links.find(it1->first) == planning_links.end() ||
     planning_links.find(it2->first) == planning_links.end())) {
    continue;
}
```

**Semantics:** "Skip this pair if planning_links is non-empty AND (link1 OR link2 is NOT in planning_links)." This means a pair is checked only when BOTH links are in `planning_links` — i.e., only arm-internal collisions (e.g., right_elbow vs right_wrist). Right-arm-vs-torso, right-arm-vs-left-arm, and right-arm-vs-leg collisions are silently skipped.

**Fix (D-03):** Change `||` to `&&`. Skip only when NEITHER link is in `planning_links`. A pair is then checked when AT LEAST ONE of (link1, link2) is in the right-arm planning chain. This includes:
- Arm-internal pairs (both in chain — still checked)
- Right-arm-vs-everything-else (one in chain — newly checked, this is the goal)
- Body-vs-body pairs where neither is in the chain (skipped — irrelevant since static body parts don't collide with each other during arm planning)

### Bug 2: Non-planning collision objects sit at world origin
**Location:** Same function, lines 808-843

**Observation:** `isInCollision()` calls `fk_solver->JntToCart(joints, out, i+1)` over the **right-arm KDL chain only**. This produces world frames for arm segments (e.g., `right_shoulder_pitch_link` … `right_wrist_yaw_link`). For all other links (torso, legs, left arm, head), `segment_frames.find(lc.segment_name)` returns `end()` and the `setTransform()` branch is skipped. Their `fcl::CollisionObjectd` retains its default identity transform — i.e., they sit at the BASE-LINK origin.

**Consequence:** Even after fixing Bug 1, the new collision checks will use wrong world poses for non-arm links. For example, `pelvis` and `left_knee_link` both end up overlapping at `torso_link`'s origin, producing both false negatives (real collisions missed) and false positives (phantom collisions in body-on-body pairs that wouldn't exist in reality — though those are now filtered out by Bug 1's fix).

**Fix:** Populate world-frame transforms for non-planning links once per goal, before OMPL solving. Strategy decided below.

### Bug 3: D-04 launch file mismatch
**Location:** `src/unitree_g1_dex3_stack-main/launch/robot.launch.py:48`

**Current:** Default `urdf_name` is `g1_29dof_lock_waist_with_hand_rev_1_0.urdf`.
**Required (D-04):** Default to `g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf`.

**Actual difference between the two URDFs (verified by parsing both files):**

| Property | `..._rev_1_0.urdf` | `..._collision_primitives.urdf` |
|----------|---------------------|----------------------------------|
| Link count | 54 | 54 (identical) |
| Joint count | 53 | 53 (identical) |
| `<visual>` geometry | 49 mesh | 49 mesh (identical) |
| `<collision>` geometry per link | **1 primitive** (axis-aligned bbox) | **3 primitives** (rotated, tighter-fitting boxes) |
| Total `<collision>` shapes | 52 (37 box + 7 cyl + 8 sphere) | **128** (108 box + 12 cyl + 8 sphere) |

**Both URDFs already use primitives for collision** — neither uses mesh BVH for FCL. The difference is granularity: the original wraps each link in one big axis-aligned box; the primitives variant wraps each link in three smaller, rotated boxes (with non-zero `rpy` on each `<origin>`) that hug the true link geometry more tightly.

Example `right_wrist_yaw_link`:
- Original: 1 box of 60×54×60 mm, axis-aligned at the link origin.
- Primitives: 3 rotated boxes (20×50×60 mm, 17×49×54 mm, 9×38×48 mm) at different rpy/xyz, jointly approximating the wrist's actual shape.

38 links (torso, both arms, both legs, all fingers, head, pelvis) get this 1→3 refinement — including every right-arm chain segment and every body link the right arm could collide with.

**Impact (corrected):**
- **Collision accuracy improves** — single coarse bbox produces many false positives (e.g., the wrist's bbox overlaps the torso's bbox even when the actual wrist is several cm clear). The primitives variant's tighter wrapping reduces those false positives → OMPL finds more feasible paths → higher planning success rate.
- **FCL performance: minor cost** — each pairwise check is now 3×3 = 9 box-box tests instead of 1×1 = 1. Total `<collision>` count rises from 52 to 128 (~2.5×). This is a small constant factor — both URDFs are dramatically faster than mesh BVH would be (which is what's relevant for the comparison; FCL handles primitives in O(1)).
- The real motivation behind D-04 is therefore **accuracy/feasibility**, not raw speed. The CONTEXT.md wording "faster FCL collision checks" is approximate — the precise framing is "tighter collision wrapping yields fewer false-positive collisions, which lets OMPL find paths that the coarse bbox would have rejected."
</bug_root_cause_analysis>

<body_link_transforms_decision>
## Decision: Body-Link Transforms via TF Buffer Lookup

### Options Evaluated

| Option | Pros | Cons | Verdict |
|--------|------|------|---------|
| **(A) TF lookup at goal start** | Uses existing `tf_buffer_` (already initialized at line 329-330). `robot_state_publisher` publishes the full TF tree from `/joint_states`. ~10-15 LOC. Matches existing project pattern (see `project_to_3d_node.cpp:213` for analog usage). One-shot per goal — no per-state overhead. | Depends on TF tree being populated. If `robot_state_publisher` hasn't received `/joint_states` yet, lookup fails. Mitigation: try/catch + 0.5s timeout (consistent with existing pattern at line 429). | **Selected** |
| (B) Full-tree KDL FK | Deterministic, no TF dependency. Self-contained. | Requires building name→tree-index mapping for all 29 joints, extra ~30-40 LOC. KDL `TreeFkSolverPos_recursive` JntArray ordering is non-trivial. | Rejected — more complexity for marginal robustness gain. By the time a goal arrives, joint_states + TF are guaranteed to be flowing (else the user can't see the robot at all). |
| (C) Static URDF defaults | Simplest. | Wrong: in running mode, the legs are bent (~30° squat); zero-position defaults put feet inside the floor and the torso lower than reality. Knees/hips would not collide-check correctly with the right hand. | Rejected — incorrect for the standing/running pose. |

### Selected Approach: (A) TF lookup with snapshot

**Algorithm:**
1. At top of `goalPoseCallback` (after the goal frame is in `base_link_`), enumerate non-planning links: `non_planning_links = link_collisions.keys() - planning_links` (where `planning_links` is the set of right-arm chain segment names).
2. For each non-planning link, call `tf_buffer_.lookupTransform(base_link_, link_name, tf2::TimePointZero, tf2::durationFromSec(0.2))`. Convert result to `Eigen::Isometry3d` manually (mirror existing pattern in `isInCollision`, lines 819-826).
3. Combine with `lc.local_transform` (URDF `<collision><origin>`) and call `lc.object->setTransform(...)`. Store result in a per-call cache so OMPL state validity checks don't redo the work.
4. Inside `isInCollision()`, only update transforms for links present in the right-arm chain (arm segments move with the OMPL state). Non-arm transforms remain valid for the whole plan.

**Why this is correct:**
- Non-arm links don't move during OMPL planning (running mode keeps the body stable; the OMPL state vector only spans 7 right-arm joints).
- A snapshot taken at goal arrival is fresh enough for the planning duration (≤1 s typical timeout).
- Failures are non-fatal: a failed TF lookup logs a warning and leaves that link's collision object unchanged (best-effort), but it does not abort the goal. We choose this over abort-on-fail because partial collision data is better than refusing to plan.

**Anti-pattern avoided:** Don't add a new dependency on `tf2_eigen` just for `tf2::transformToEigen` — the existing manual quaternion-to-rotation-matrix conversion is already used in this file and only needs a small adapter for `geometry_msgs::msg::TransformStamped`.
</body_link_transforms_decision>

<ompl_bounds_review>
## OMPL State Space Bounds Review

**Current code (lines 700-713):** Already correct — iterates `planning_joints` (which after Phase 1 will be just the 7 right-arm joints), looks each up in `joint_limits_` (populated from URDF at init), calls `bounds.setLow(i, lower)` / `bounds.setHigh(i, upper)`. Falls back to `[-3.14, 3.14]` if not found.

**Verification:** The 7 right-arm joints in `g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf` are revolute with explicit limits (verified for `right_shoulder_pitch_joint` [-3.0892, 2.6704], `right_shoulder_roll_joint` [-2.2515, 1.5882], `right_wrist_yaw_joint` [-1.6144, 1.6144]; all 7 follow the same pattern). The fallback path should never trigger in practice — but keep it as defensive code with a `RCLCPP_WARN` so any URDF regression is loud.

**No code change required for PLAN-04 logic.** The work is:
1. Verify the bounds path still works after `planning_joints` becomes right-arm-only.
2. Add a single concise INFO log at OMPL setup: "OMPL bounds set: 7 joints, [name=lo,hi] ... ".
3. Replace the existing redundant per-joint log dump (line 535-542) with this concise version.
</ompl_bounds_review>

<existing_code_assets>
## Existing Code Assets (No Change Needed)

These already work correctly and are reused unchanged:

- **`buildCollisionObjects()`** (lines 332-414) — Loads ALL URDF links' collision geometry into `link_collisions` map. Already supports box/cylinder/sphere/mesh. `local_transform` already captures the `<collision><origin>` from URDF. No change.
- **`tf_buffer_` / `tf_listener_`** (lines 329-330) — Already initialized in constructor init list. Used as-is for new body-link lookups.
- **`joint_limits_`** map — Already populated from URDF for all joints. Used as-is for OMPL bounds.
- **`latest_joint_positions_`** — Updated from `/joint_states` callback. Used as-is for IK seed.
- **URDF-at-runtime fetch** (lines 60-86) — Service-based fetch from `/robot_state_publisher`. No change.
- **TF goal-pose transform** (lines 422-438) — Already transforms incoming goal to `base_link_`. No change.
- **TRAC-IK random-seed retry loop** (lines 621-685) — Independent of left/right; works as-is for right arm only.
- **OMPL solve loop + trajectory publish** (lines 759-799) — `traj_msg.joint_names = planning_joints` will naturally publish only right-arm joints once `planning_joints` is right-arm-only. No change.
</existing_code_assets>

<debug_log_inventory>
## Debug Log Cleanup Inventory (D-05)

**Logs to REMOVE (verbose, redundant, or dev-only):**
| Lines | Content | Reason |
|-------|---------|--------|
| 113, 120 | `printKDLChainInfo(kdl_chain_right/left, ...)` calls — and the entire helper function definition (871-893) | Per-segment chain dump. Only useful during initial development. |
| 164-191 | Right & left arm joint limits listing | Redundant with the `OMPL bounds set` log in `goalPoseCallback`. |
| 234-240 | "URDF links (N): ..." dump of all link names | Unused after init, noisy. |
| 242-263, 265-286 | Right & left arm KDL chain joint type listing | Redundant with chain info already printed. |
| 288-291 | All joints' limit dump | Redundant with init INFO. |
| 444-456 | "Planning chain segments: ..." per-goal dump | Redundant — chain doesn't change. |
| 458-489 | Joint order check verbose output | Keep ONLY a single ERROR if mismatch detected; remove the verbose info dumps. |
| 501-507 | "Start state: name=val, ..." per-goal | Verbose. |
| 519-531 | "Target frame details ... RPY ..." | Verbose; the goal pose was already logged at line 430. |
| 534-542 | Per-joint limits dump per-goal | Redundant with init log. |
| 552-576 | IK seed/solution per-call dumps | Demote: Keep one INFO ("IK ok / IK failed: code N"), drop the multi-line seed/solution dumps. |
| 591-594, 600-607, 690-698, 780-794 | Current/Final EE pose, target pose dumps | Verbose. |
| 638-674 | "Found better solution (diff=...)" per-retry | Demote to DEBUG. |
| 720-728 (state_oss + RCLCPP_DEBUG) | Per-state debug formatting | Keep DEBUG only — already at DEBUG level, but the string-building cost is paid even when DEBUG is off. **Wrap in conditional or keep — low priority.** |
| 736-749 | Start/Goal state validity verbose dump | Demote: keep one WARN if invalid, drop value dumps. |
| 832-841 | Per-collision-object translation/rotation/AABB dumps (already commented out) | Already disabled — leave as-is OR remove for cleanliness. |

**Logs to KEEP (essential):**
- Init success: "IKFCLPlannerNode initialized with N joints" (line 217) ✓
- Init: chosen base link, right tip, planning timeout, time step (lines 218-221) ✓
- Init: collision skip pairs count (line 222-225) ✓
- Goal received in base frame (line 426/430) ✓
- IK success/failure result with code (line 570) ✓
- Collision detected pairs in `isInCollision` (line 863) ✓
- Plan-found summary (replace verbose final-state check with: "Plan published: N waypoints") ✓
- All RCLCPP_ERROR / RCLCPP_FATAL / RCLCPP_WARN throttled messages ✓

**Net effect:** ~150 lines of debug logging code removed. The file shrinks from 903 → ~700 lines.
</debug_log_inventory>

<validation_strategy>
## Validation Strategy

Multi-level verification approach. No Nyquist-style sampling concerns apply at this phase (state-validity-checking resolution is Phase 2's territory).

### Level 1: Static Source Verification (grep)
After implementation, the following must hold:

| Check | Command | Expected |
|-------|---------|----------|
| No left chain refs | `grep -nE 'kdl_chain_left|ik_left|fk_left_solver|left_tip_' src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` | No matches |
| No left arm chain methods | `grep -nE 'getChain.*left_tip' src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` | No matches |
| No y-axis arm selection | `grep -n 'use_right' src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` | No matches |
| Collision filter uses && | `grep -n 'planning_links.find.*== planning_links.end()' src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` | Expression joined by `&&` (or use a single combined check) |
| TF lookup for body links | `grep -n 'lookupTransform.*base_link_' src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` | At least one match in `goalPoseCallback` |
| Default URDF is collision_primitives | `grep -n "default_value=.*collision_primitives" src/unitree_g1_dex3_stack-main/launch/robot.launch.py` | One match |
| left_tip param declaration removed | `grep -n 'declare_parameter.*left_tip' src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` | No matches |
| left_tip declaration removed from planner.launch.py | `grep -n 'left_tip' src/unitree_g1_dex3_stack-main/launch/planner.launch.py` | No matches |

### Level 2: Build Verification
```bash
colcon build --packages-select unitree_g1_dex3_stack \
  --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON \
  -DCMAKE_BUILD_TYPE=Release
```
**Pass criteria:** Exit code 0, no warnings about removed symbols.

### Level 3: Init Smoke Test (launch & inspect logs)
```bash
ros2 launch unitree_g1_dex3_stack robot.launch.py &
ros2 launch unitree_g1_dex3_stack planner.launch.py &
```
**Pass criteria:**
- No FATAL/ERROR in planner output
- Init log lists exactly 7 right-arm joints with URDF-derived limits
- Robot model URI matches `..._collision_primitives.urdf`
- KDL chain "right" extracted with 7 joints
- No mention of `kdl_chain_left` / `left_tip` / left arm

### Level 4: Behavioral Verification (runtime, requires hardware)
Send goal poses with `ros2 topic pub /goal_pose ...`:

| Test | Goal pose (in `torso_link`) | Expected outcome |
|------|----------------------------|------------------|
| T1 — right side reach | x=0.30, y=-0.20, z=0.20 | Plan published; joint_names contains the 7 right-arm joints; trajectory has ≥2 waypoints |
| T2 — left-side goal still uses right arm (D-02 regression) | x=0.30, y=+0.20, z=0.20 | Plan published with the 7 right-arm joints (NOT left). Robot may twist torso-side or fail IK gracefully — but it does NOT silently switch to the left arm. |
| T3 — collision regression (D-03) | EE target embedded inside torso volume, e.g., x=-0.10, y=0.0, z=0.30 | OMPL fails to find a path OR `Collision detected between ... and torso_link` log line appears; planner publishes no trajectory. |
| T4 — feasibility regression | A goal that worked before the change (any clear-of-body pose) | Still produces a valid plan post-change. |

T1, T2, T4 confirm the right-arm-only and y-axis-removal behaviors. T3 confirms the collision-filter fix actually catches body collisions that were previously missed.

### Level 5: Code Volume Sanity Check
- Source file should shrink from 903 lines to roughly 680-720 lines (left arm code removal ~80 lines + log cleanup ~150 lines, partly offset by ~30 lines of new TF-lookup code).
- A delta near or above this range is expected; significantly less suggests cleanup was incomplete.
</validation_strategy>

<implementation_pitfalls>
## Common Pitfalls

### Pitfall 1: Forgetting to remove `use_right` from the `isInCollision` signature
**Symptom:** Compiler error or unused parameter warning.
**Fix:** After removing the y-axis selector, `isInCollision` callers no longer pass `use_right`. Remove the parameter and tighten the function — it always operates on the right arm chain.

### Pitfall 2: `planning_links` becomes empty if computed before chain is filtered
**Symptom:** With an empty `planning_links` set, the `if (!planning_links.empty() && ...)` guard at the top of the filter is bypassed, falling back to "check all pairs" — including body-vs-body pairs that yield false collision detections at static body geometries that overlap in URDF (legs and pelvis collision boxes often overlap slightly).
**Fix:** Ensure `planning_links` is always populated from the right-arm chain segments. If the chain extraction failed, abort the goal early — don't proceed with empty `planning_links`.

### Pitfall 3: Forgetting to remove `left_tip` from `planner.launch.py`
**Symptom:** Launch file declares an undeclared planner parameter; planner ignores it (silent), but later commits/reviews flag the dead arg.
**Fix:** Remove `DeclareLaunchArgument('left_tip', ...)` and the corresponding `LaunchConfiguration('left_tip').perform(...)` line, and the `'left_tip': left_tip` entry in the parameters dict.

### Pitfall 4: TF lookup races with first goal
**Symptom:** First goal after launch fails because `/tf` hasn't propagated yet; subsequent goals succeed.
**Fix:** Use a 0.2-0.5 s timeout in `lookupTransform` (consistent with existing pattern at line 429). On exception, log a single WARN per link and continue — the failure is recoverable; the affected link's collision object simply uses its previous (or default) transform until the next goal.

### Pitfall 5: Dropping the joint-order check entirely loses a useful safety net
**Symptom:** A future URDF rev that reorders joints could silently miswire IK seeds.
**Fix:** Keep the order check, but compress its output. Replace 30+ lines of INFO dumps with a single ERROR-only path: if mismatch, log one ERROR with the differing joint names; on success, log nothing (or a single one-line INFO).
</implementation_pitfalls>

<file_change_summary>
## File Change Summary

| File | Change | LOC delta |
|------|--------|-----------|
| `src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` | Remove all left-arm code, fix `isInCollision` filter (`||` → `&&`), populate non-planning link transforms via TF, remove y-axis selection, clean debug logs | -200 to -250 net (-280 removed, +30 added for TF block) |
| `src/unitree_g1_dex3_stack-main/launch/robot.launch.py` | Default `urdf_name` → `g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf` | ±0 (one-line edit) |
| `src/unitree_g1_dex3_stack-main/launch/planner.launch.py` | Remove `left_tip` declaration and parameter | -3 |
| `src/unitree_g1_dex3_stack-main/CMakeLists.txt` | No change — `tf2`, `tf2_ros`, `tf2_geometry_msgs` already in `ament_target_dependencies` for the planner | 0 |
| `src/unitree_g1_dex3_stack-main/package.xml` | No change — same reason as CMakeLists.txt | 0 |

**No new dependencies.** All TF / Eigen / FCL / KDL primitives are already linked.
</file_change_summary>

---

*Researched: 2026-04-29*
*Confidence: HIGH — root cause for both bugs identified by reading the code; chosen approach uses only patterns already present in the codebase.*
