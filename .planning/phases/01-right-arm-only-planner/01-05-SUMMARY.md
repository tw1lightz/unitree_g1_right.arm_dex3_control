---
phase: 01-right-arm-only-planner
plan: 05
subsystem: motion-planning
tags: [hot-fix, collision-checking, kdl, urdf, log-cleanup, live-verification]

requires:
  - phase: 01-right-arm-only-planner
    provides: "Right-arm-only planner with verified initialization (Plan 01-03) + executor safety net (Plan 01-04)."
provides:
  - On every planner startup, the right-arm KDL chain's adjacent-link pairs (parent_link:child_link, including base_link → first segment) are auto-appended to `collision_skip_pairs_`. With `base_link_=torso_link` and the standard 7-segment right arm chain this is 7 entries, eliminating the pre-existing `start=INVALID goal=INVALID` failure caused by adjacent-link self-collision.
  - The 28-line per-goal joint-order false-positive `RCLCPP_ERROR` block is removed; the immediately-following `planning_positions` lookup is by-name and order-independent so the check served no functional purpose.
affects:
  - Phase 1 live-verification path — operators can now send `/goal_pose` to a freshly-started planner (without launch parameters) and OMPL will actually run.
  - Future phases that may revise the KDL chain or `collision_skip_pairs_` semantics.

tech-stack:
  added: []
  patterns:
    - "Auto-derive adjacent-link skip pairs from a KDL::Chain by iterating segments and chaining `(base_link, segment[0]) → (segment[i], segment[i+1])`. Robust against URDF changes; no per-link hardcoding."

key-files:
  created:
    - .planning/phases/01-right-arm-only-planner/01-05-PLAN.md
    - .planning/phases/01-right-arm-only-planner/01-05-SUMMARY.md
  modified:
    - src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp

key-decisions:
  - "Append auto-derived adjacent-link pairs into the existing `collision_skip_pairs_` member (not a new member). Reuses the existing `isInCollision` matching path AND the existing init INFO log block at lines 168-171, so operators can read off all 7 skip pairs at startup without any new logging code."
  - "Seed the auto-derive loop with `prev = base_link_` because `KDL::Chain::getChain(base, tip, ...)` excludes the base from the segment list, so segment[0]'s parent is `base_link_` itself. Without this seed, the (torso_link, right_shoulder_pitch_link) pair would be missing and shoulder vs torso self-collision would re-bork OMPL."
  - "Delete the 28-line joint-order check rather than demote it to DEBUG. The check's invariant is not functionally required; downgrading would just trade visible false-positives for hidden noise. Documented the intentional absence in a 4-line comment."
  - "Treat as a Phase-1 hot-fix (Plan 01-05) rather than a Phase 2 task. The defects block the Plan 01-03 + 01-04 human-verify checkpoint, and Phase 2 (Path Simplification & Quality) is scoped to trajectory smoothing / re-planning, not collision-skip configuration."

patterns-established:
  - "When a parameter has a 'reasonable default that depends on URDF topology', derive that default in code from the URDF/KDL representation. The launch parameter is then for **non-default extras** (e.g. mesh-overlap pairs that aren't topologically adjacent), not for the baseline."
  - "When a sanity check's failure mode is a no-op (i.e. functional code is robust to the case being checked), the check should be deleted, not demoted. Hidden warnings are worse than no warnings."

requirements-completed: []

duration: ~25 min
completed: 2026-04-29
---

# Phase 01 Plan 05: Unblock live verification — Summary

**Hot-fix on top of Plans 01-03 + 01-04 to remove two pre-existing defects (present since `83b34a5 project_init`, NOT introduced by Phase 1) that surfaced at the user's first `/goal_pose` during the human-verify checkpoint.**

## Performance

- **Tasks:** 3 / 3 auto complete + 1 human-verify checkpoint pending
- **Files modified:** 1 (`ik_fcl_ompl_planner.cpp`)
- **LOC delta:** 698 → 687 (-11 lines net: +13 auto-derive block, +5 replacement comment, -28 deleted check, -1 trailing blank)
- **Build:** `colcon build --packages-select unitree_g1_dex3_stack` exit 0 (49.3 s).

## Implementation summary

Two surgical edits on `@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` via a single `multi_edit` call:

### Edit 1 — auto-derive adjacent-link skip pairs

Inserted right after the joint-limits parse (post-line 142, pre-TRAC-IK construction):

```@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp:144-156
        // Auto-derive adjacent-link skip pairs from the KDL chain so that
        // physically-connected (and therefore geometrically-overlapping)
        // links are not flagged as self-collisions. Order in the pair string
        // does not matter -- isInCollision matches both "a:b" and "b:a".
        // See Plan 01-05.
        {
            std::string prev = base_link_;
            for (unsigned int i = 0; i < kdl_chain_right.getNrOfSegments(); ++i) {
                const std::string& cur = kdl_chain_right.getSegment(i).getName();
                collision_skip_pairs_.push_back(prev + ":" + cur);
                prev = cur;
            }
        }
```

For the standard `base_link_ = "torso_link"` + `right_tip_ = "right_wrist_yaw_link"` configuration this appends:

```
torso_link:right_shoulder_pitch_link
right_shoulder_pitch_link:right_shoulder_roll_link
right_shoulder_roll_link:right_shoulder_yaw_link
right_shoulder_yaw_link:right_elbow_link
right_elbow_link:right_wrist_roll_link        <- the one in the user's failing log
right_wrist_roll_link:right_wrist_pitch_link
right_wrist_pitch_link:right_wrist_yaw_link
```

The existing init INFO block (`@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp:168-171`) reports them automatically — no new logging code.

### Edit 2 — delete the joint-order false-positive check

The 28-line `bool order_ok` block (which previously occupied lines 375-402) is gone, replaced by a 5-line comment:

```@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp:389-393
        // Note: KDL chain joint order does NOT need to match /joint_states
        // ordering -- the planning_positions lookup below is by-name. The
        // pre-existing joint-order ERROR check that lived here was removed
        // in Plan 01-05 because it was a false-positive on every goal.
        std::vector<double> planning_positions;
```

The directly-following `planning_positions` lookup is by-name (`std::find(joint_names_.begin(), joint_names_.end(), jname)`), so KDL chain order vs `/joint_states` order are independent.

## Acceptance criteria — static checks all PASS

| # | Check | Result |
|---|-------|--------|
| 1 | `colcon build --packages-select unitree_g1_dex3_stack` exit 0 | PASS (49.3 s) |
| 2 | `grep -c "out of order at idx"` ik_fcl_ompl_planner.cpp | 0 ✓ |
| 3 | `grep -c "Joint-order mismatch"` ik_fcl_ompl_planner.cpp | 0 ✓ |
| 4 | `grep -c "Auto-derive adjacent-link skip pairs"` ik_fcl_ompl_planner.cpp | 1 ✓ |
| 5 | `wc -l` ik_fcl_ompl_planner.cpp | 687 (was 698) ✓ |

## Acceptance criteria — runtime (deferred to user-driven re-verification)

| # | Check | How |
|---|-------|-----|
| 6 | Init INFO `Collision skip pairs: 7` (was `0`) | User restarts planner: `pkill -f ik_fcl_ompl_planner` then `ros2 run unitree_g1_dex3_stack ik_fcl_ompl_planner` and reads the init log. |
| 7 | No `Joint-order mismatch ... out of order at idx ...` ERROR on `/goal_pose` | User sends Plan 01-03 testA goal_pose. |
| 8 | No `Collision detected between right_<i>_link and right_<i+1>_link` WARN at start state | User sends Plan 01-03 testA goal_pose; OMPL should reach RRTConnect or fail for non-self-collision reasons. |

## Behavior contract

After Plan 01-05, `/goal_pose` callback does NOT emit:
- `[ERROR] Joint-order mismatch between KDL chain and /joint_states: ...`
- `[WARN] Collision detected between right_elbow_link and right_wrist_roll_link` (or any other adjacent right-arm link pair, when the arm is in or near its standing pose)

It MAY still emit collision WARNs for legitimate cases:
- Right arm vs body (e.g. `right_*_link` vs `torso_link` when not adjacent — that's the Plan 01-02 fix at work).
- Non-adjacent right-arm link pairs that happen to overlap at extreme joint configurations (rare; goes via the launch parameter if it ever shows up in practice — out of scope here).

It MAY emit `OMPL failed to find a path` for goals that are genuinely unreachable. That is OMPL working as designed, not a bug.

## Out of scope (NOT done in this plan)

- `planner.launch.py` empty-list parameter bug — the auto-derive makes the launch param's default-empty-string mostly cosmetic; the only operational use of the launch parameter now is for **extra** non-adjacent skip pairs. Still flagged for Phase 2 / 1.1 follow-up.
- Non-adjacent self-pair skips (e.g. shoulder_pitch vs wrist_yaw if their meshes happen to overlap at extreme configurations) — these go via the launch parameter alongside the auto-pairs, or via a future plan if practice surfaces them.
- Any change to `joint_trajectory_executor` (Plan 01-04 territory) or to URDF / CMakeLists.txt / package.xml.

## Deviations from Plan

None. Plan as written, executed as written.

## Issues Encountered

- The user-running planner instance (PID `110786`) was the pre-Plan-01-05 build. Smoke test for AC #6 (init log shows 7 skip pairs) is deferred to user re-running `ros2 run unitree_g1_dex3_stack ik_fcl_ompl_planner` after they're ready, rather than killing their interactive session.

## Next Steps

- **User action required:** restart the planner terminal (Ctrl+C → re-run `ros2 run unitree_g1_dex3_stack ik_fcl_ompl_planner`) and confirm:
  1. Init INFO line `Collision skip pairs: 7` (followed by 7 `Skipping collision check for pair: ...` lines).
  2. Re-run Plan 01-03 testA / testB / testC: no joint-order ERROR; no adjacent-link self-collision WARN.
  3. Re-run Plan 01-04 Ctrl+C / SIGTERM tests on the executor: still works as before (Plan 01-05 didn't touch the executor).
- After user-confirmed PASS, Phase 1 can be archived and we move on to Phase 2 planning.

---
*Phase: 01-right-arm-only-planner*
*Completed: 2026-04-29*
