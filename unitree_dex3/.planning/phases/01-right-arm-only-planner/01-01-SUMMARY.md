---
phase: 01-right-arm-only-planner
plan: 01
subsystem: motion-planning
tags: [ompl, fcl, trac-ik, kdl, ros2, urdf, right-arm-only]

requires: []
provides:
  - Right-arm-only planner source: all left-arm members, params, KDL chain, IK/FK solvers, debug logs removed.
  - y-axis arm selector eliminated; planner unconditionally targets the right arm regardless of goal pose y.
  - `isInCollision()` signature simplified to right-only (drops `bool use_right` parameter).
  - Default URDF for `robot.launch.py` is now the collision-primitives variant (D-04 satisfied).
  - `planner.launch.py` no longer declares or forwards `left_tip`.
affects:
  - 01-02 collision filter and transform snapshot (will edit the simplified `isInCollision` body).
  - 01-03 OMPL bounds verification + debug log cleanup.

tech-stack:
  added: []
  patterns:
    - "Single-arm planner: hard-coded right_tip, no runtime arm selection."
    - "Collision-primitives URDF default: faster FCL checks for the same kinematic tree."

key-files:
  created:
    - .planning/phases/01-right-arm-only-planner/01-01-SUMMARY.md
  modified:
    - src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp
    - src/unitree_g1_dex3_stack-main/launch/robot.launch.py
    - src/unitree_g1_dex3_stack-main/launch/planner.launch.py

key-decisions:
  - "D-01 (left-arm removal): all left-arm infrastructure deleted with no compile-time guard or runtime flag."
  - "D-02 (no y-axis selector): planner always uses the right arm regardless of goal pose y-coordinate."
  - "D-04 (URDF model): robot.launch.py now defaults to the collision-primitives variant."

patterns-established:
  - "isInCollision now takes only `(joints, skip_pairs, planning_links)` — fk_solver and kdl_chain are hard-coded to right-arm members."

requirements-completed: [PLAN-01]

duration: ~25 min
completed: 2026-04-29
---

# Phase 01 Plan 01: Right-Arm-Only Refactor + URDF Switch Summary

**Stripped all left-arm planner infrastructure (members, IK/FK, KDL chain, debug logs), removed the `pose.position.y < 0.0` arm selector so the planner unconditionally targets the right arm, and switched `robot.launch.py` to default to the collision-primitives URDF.**

## Performance

- **Tasks:** 3 / 3
- **Files modified:** 3
- **LOC delta on `ik_fcl_ompl_planner.cpp`:** 903 → 830 (-73 lines).
- **Build:** `colcon build --packages-select unitree_g1_dex3_stack --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON` — exit 0.

## Accomplishments

- `robot.launch.py` default `urdf_name` switched to `g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf` (D-04).
- `planner.launch.py`: `left_tip` removed in all three places (perform line, params dict, DeclareLaunchArgument). AST parses cleanly. `right_tip` and the rest of the parameter set are unchanged.
- `ik_fcl_ompl_planner.cpp`:
  - Constructor: dropped `left_tip` declare/get, the left KDL chain extraction block, the left joint-limit arrays + population loop, the left-arm joint-limits debug oss block, the `ik_left` TRAC-IK init + asserts, the `fk_left_solver` init, the left-arm chain-segments INFO logs, and the kdl_left_oss debug dump.
  - Members: removed `kdl_chain_left`, `ik_left`, `fk_left_solver`, `left_tip_`.
  - `goalPoseCallback`: removed `bool use_right = pose_in_base.pose.position.y < 0.0;` (D-02). Replaced all `use_right ? right : left` ternaries (KDL::Chain ref, base/tip log, IK solver ref, FK solver ref) with right-only references.
  - State validity checker lambda: dropped `use_right` capture.
  - `isInCollision`: signature reduced to `(joints, skip_pairs, planning_links)` — `bool use_right` parameter dropped; body fk_solver/kdl_chain wired straight to right-arm members. The single call site in the lambda updated.

## Task Commits

1. **Task 1: switch robot.launch.py default URDF** — `70aca5d` (feat)
2. **Task 2: remove left_tip from planner.launch.py** — `411687f` (refactor)
3. **Task 3: strip all left-arm code + use_right selector from planner source** — `0390846` (refactor)

## Decisions Made

- D-01, D-02, D-04 implemented exactly as decided in `01-CONTEXT.md`. No new decisions in this plan.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1-3 — Plan AC over-specified] Task 3 acceptance criterion 3 (zero `pose_in_base.pose.position.y` matches) is impossible to satisfy.**

- **Found during:** Task 3 verification.
- **Issue:** The plan's `<acceptance_criteria>` for Task 3 included `grep -n 'pose_in_base\.pose\.position\.y' ... returns zero matches`. But the y-coordinate is legitimately used in:
  - The TF-transform success log (line 361 in the post-refactor file): `pose_in_base.pose.position.x, pose_in_base.pose.position.y, pose_in_base.pose.position.z`.
  - The KDL `target_frame` construction (line 444): `pose_in_base.pose.position.y` is needed because the IK target is a 3D position.
  - The "Input goal pose" RCLCPP_INFO (line 521): same x/y/z log triple.
  None of these are arm-selection logic.
- **Fix:** The `<action>` block of Task 3 only required deleting the selector line `bool use_right = pose_in_base.pose.position.y < 0.0;`, which is removed (verified: `grep -n use_right` returns zero). D-02 in `01-CONTEXT.md` ("Remove the y-coordinate arm selection logic. Always use right arm regardless of goal position.") is fully satisfied.
- **Files modified:** none (this is a documentation-of-AC-mismatch deviation, not a code change).
- **Verification:** All other Task 3 acceptance criteria pass (zero left-arm tokens, zero `use_right`, zero `Left arm:` logs, `kdl_chain_right` ≥ 4, `ik_right` ≥ 2, `isInCollision` exactly 2 occurrences with the simplified 3-parameter signature, build exit 0).
- **Committed in:** `0390846` (Task 3 commit).

---

**Total deviations:** 1 auto-fixed (1 plan-AC-mismatch — Rule 1-3, no scope change).
**Impact on plan:** No scope creep. The intent of D-02 is satisfied; only the AC wording needs to be relaxed in future replans.

## Issues Encountered

- The first `multi_edit` attempt on Task 3 had one chunk fail: the line `auto& fk_solver = use_right ? fk_right_solver : fk_left_solver;` appeared in both `goalPoseCallback` and `isInCollision`. The other chunks succeeded, and the duplicate was fixed with a single follow-up `edit` that included surrounding context to disambiguate. Final state verified clean.
- The `colcon build` produced PCL-related stderr warnings (`io features related to pcap/png/libusb-1.0 will be disabled`). These are pre-existing PCL configuration warnings unrelated to this plan and the build still exits 0.

## Next Phase Readiness

- `isInCollision` signature is now `(joints, skip_pairs, planning_links)` — Plan 01-02 will modify the body of this function to fix the `||` filter (D-03) and snapshot non-arm transforms.
- The planner currently still has the broken collision filter (`||` should be `&&`) and missing world transforms for non-planning links — this is intentional, and is Plan 01-02's scope.
- Verbose debug logging (joint order check, per-state collision dumps, joint-limits dumps) is still present — Plan 01-03 will clean it up per D-05.

---
*Phase: 01-right-arm-only-planner*
*Completed: 2026-04-29*
