---
phase: 01-right-arm-only-planner
plan: 02
subsystem: motion-planning
tags: [fcl, tf2, collision-checking, ros2, eigen]

requires:
  - phase: 01-right-arm-only-planner
    provides: "Right-arm-only goal handler with simplified isInCollision(joints, skip_pairs, planning_links) signature."
provides:
  - Self-collision check now considers right-arm links against ALL other body links (D-03 satisfied).
  - Non-planning collision objects (torso, legs, left arm, head, hands) carry world-frame transforms snapshotted from the TF tree once per goal.
  - tf_lookup_failures counter + per-link WARN emit a clear diagnostic when TF is missing for a body link.
affects:
  - 01-03 OMPL bounds verification + debug log cleanup + smoke test will operate on the now-correct collision check.

tech-stack:
  added: []
  patterns:
    - "Per-goal TF snapshot for static body links: tf_buffer_.lookupTransform(base, link, TimePointZero, 0.2s) → Eigen::Isometry3d → fcl::Transform3d. Mirrors the existing in-file pattern in isInCollision (no tf2_eigen dep)."
    - "isInCollision filter semantics: skip a pair only when NEITHER link is in the planning chain — checks arm-vs-arm and arm-vs-body, skips body-vs-body."

key-files:
  created:
    - .planning/phases/01-right-arm-only-planner/01-02-SUMMARY.md
  modified:
    - src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp

key-decisions:
  - "D-03 implemented: isInCollision filter changed from `||` to `&&` between the two planning_links.end() checks."
  - "Claude's discretion item (body-link transforms): chose TF lookup over full-tree FK or static URDF defaults — TF tree is already running, gives live data, and avoids implementing a new FK pass."

patterns-established:
  - "Body-link transform snapshot block lives inside goalPoseCallback, immediately after the planning_links population loop and before any OMPL setup. Failures fall back to the previously-set FCL transform with a WARN."

requirements-completed: [PLAN-02]

duration: ~10 min
completed: 2026-04-29
---

# Phase 01 Plan 02: Collision Filter Fix + Non-Arm TF Snapshot Summary

**Right-arm self-collision check now correctly considers arm-vs-body pairs (D-03 fix), and non-planning collision objects carry live world-frame transforms via a per-goal TF snapshot.**

## Performance

- **Tasks:** 2 / 2
- **Files modified:** 1
- **LOC delta on `ik_fcl_ompl_planner.cpp`:** 830 → 867 (+37 lines: 34 from the TF snapshot block, 3 from the filter comment).
- **Build:** `colcon build --packages-select unitree_g1_dex3_stack --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON` — exit 0.

## Accomplishments

- **TF snapshot for non-arm links** (Task 1): a single per-goal pass over `link_collisions` looks up `base_link_ → link_name` via `tf_buffer_.lookupTransform(..., TimePointZero, 0.2s)`, composes `Eigen::Isometry3d`, multiplies by the link's `local_transform`, and pushes via `setTransform`. Arm-chain links are skipped (they are updated per OMPL state inside `isInCollision`). Failures bump `tf_lookup_failures` and emit a per-link `RCLCPP_WARN`; a summary `WARN` fires once if the counter is non-zero.
- **Filter fix** (Task 2): the inverted `||` in the `isInCollision` planning-links filter is now `&&`, so a pair is skipped only when NEITHER link is in the planning chain. The `!planning_links.empty()` gate is preserved.

## Exact `lookupTransform` block (as inserted at line 386)

```cpp
        // Snapshot world-frame transforms of all non-planning links from the TF tree.
        // Right-arm chain segments will have their transforms updated per OMPL state
        // inside isInCollision(); everything else (torso, legs, left arm, head, hands)
        // is static during planning, so we look it up once and reuse.
        size_t tf_lookup_failures = 0;
        for (auto& [link_name, lc] : link_collisions) {
            if (planning_links.find(link_name) != planning_links.end()) {
                continue; // arm-chain links are handled per-state in isInCollision
            }
            try {
                auto tf_msg = tf_buffer_.lookupTransform(
                    base_link_, link_name, tf2::TimePointZero,
                    tf2::durationFromSec(0.2));
                const auto& t = tf_msg.transform.translation;
                const auto& q = tf_msg.transform.rotation;
                Eigen::Isometry3d world_tf = Eigen::Isometry3d::Identity();
                world_tf.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
                world_tf.translation() = Eigen::Vector3d(t.x, t.y, t.z);
                Eigen::Isometry3d final_tf = world_tf * lc.local_transform;
                lc.object->setTransform(fcl::Transform3d(final_tf.matrix()));
            } catch (const tf2::TransformException& ex) {
                ++tf_lookup_failures;
                RCLCPP_WARN(this->get_logger(),
                    "TF lookup failed for non-planning link '%s' (base='%s'): %s. "
                    "Using previous transform; collision check may be inaccurate.",
                    link_name.c_str(), base_link_.c_str(), ex.what());
            }
        }
        if (tf_lookup_failures > 0) {
            RCLCPP_WARN(this->get_logger(),
                "Body-link TF snapshot completed with %zu failures.",
                tf_lookup_failures);
        }
```

## Task Commits

1. **Task 1: TF snapshot of non-planning link transforms** — `678cb61` (feat)
2. **Task 2: fix `isInCollision` filter (`||` → `&&`)** — `071b416` (fix)

## Decisions Made

- D-03 implemented exactly as decided in `01-CONTEXT.md`.
- For the "Claude's discretion" item on body-link transforms: chose **per-goal TF lookup** rather than full-tree FK or static URDF defaults. Reasons:
  - The TF tree is already published by `robot_state_publisher` (started in `robot.launch.py`) and reflects the live `joint_states`, so we get accurate body-link poses for free.
  - Per-goal cost is bounded (number of non-arm links is small, fixed by URDF), and the snapshot avoids per-state TF lookups inside the OMPL validity checker.
  - No new dependency (`tf2_eigen` not needed); the manual quaternion-to-matrix conversion mirrors the in-file pattern already used inside `isInCollision`.

## Deviations from Plan

None — plan executed exactly as written. The pre-existing comment `// Only check if at least one link is in planning_links` on the line just before the new comment block (line 808) was deliberately left untouched per the plan's "Do not modify any other logic in `isInCollision`" instruction.

## Issues Encountered

- No smoke-launch was run as part of this plan — the optional V4 verification (launch + observe no FATAL/ERROR before goal arrival) is owned by Plan 01-03's smoke-test task. Recommend wiring it in Plan 03.

## Pitfalls flagged for Plan 03

1. **`tf2::TimePointZero` vs `rclcpp::Time(0, 0, RCL_ROS_TIME)`:** `TimePointZero` returns the latest available transform, which is correct for "snapshot once at goal arrival" but means the snapshot's freshness depends on TF buffer fill. If `robot_state_publisher` hasn't published a recent transform when the first goal arrives, the 0.2s timeout may expire and we will fall back to whatever transform was last set on the FCL object — which after construction is identity. Plan 03's smoke test should explicitly check the WARN output for `TF lookup failed for non-planning link` on first goal; if seen, raise the timeout or wait for TF readiness in `__init__`.
2. **Stale snapshot during long-running OMPL solves:** the snapshot is taken once per goal and reused for the duration of the solve. If the robot's torso/legs are commanded to move during the solve (e.g., locomotion), the body collision objects stay frozen at their pre-solve poses. This is acceptable for the v1 "static environment" decision in `STATE.md`, but Plan 03 should document this assumption near the snapshot block as a comment for downstream phases.
3. **`tf_lookup_failures` shadowing:** the counter is declared as a local in `goalPoseCallback`. If a future phase wants per-callback metrics, they should be promoted to a member. Not in scope for Plan 03.
4. **Verbose debug logs still present:** `seg_oss`, joint-order check, per-state collision string in the lambda, joint-limits dumps. Plan 03 should remove these per D-05 — they spam the log during planning and obscure the new TF lookup warnings if they ever fire.

## Next Phase Readiness

- `isInCollision` now correctly checks arm-vs-arm and arm-vs-body pairs with live body-link poses. Phase 1's safety goal is functionally satisfied.
- Plan 03 will (a) verify the OMPL state-space bounds path satisfies PLAN-04, (b) clean up the verbose debug logs per D-05, and (c) run a build + launch smoke test confirming Phase 1 success criteria #1 (single-arm planner), #4 (collision-primitives URDF), and #5 (no FATAL/ERROR at startup).

---
*Phase: 01-right-arm-only-planner*
*Completed: 2026-04-29*
