---
phase: 01-right-arm-only-planner
plan: 03
subsystem: motion-planning
tags: [ompl, ros2, rclcpp, debug-cleanup, smoke-test, trac-ik]

requires:
  - phase: 01-right-arm-only-planner
    provides: "Right-arm-only goal handler with corrected collision filter and TF-snapshot body transforms."
provides:
  - OMPL bounds path is now defensive (WARN-on-fallback, summary INFO, 7-joint guard).
  - Verbose debug logging cleaned up per D-05; only essential init + per-goal milestone logs remain.
  - Latent bad_weak_ptr crash in node construction fixed (TRAC-IK init deferred out of constructor).
  - Smoke launch verified at runtime: planner reaches all 6 init INFO logs with no FATAL/ERROR.
affects:
  - Phase 02 — operates on a stable, runtime-verified Phase 1 planner.

tech-stack:
  added: []
  patterns:
    - "Two-phase rclcpp Node construction: cheap base init in ctor, real init in public init() called after make_shared. Avoids bad_weak_ptr from shared_from_this()."

key-files:
  created:
    - .planning/phases/01-right-arm-only-planner/01-03-SUMMARY.md
  modified:
    - src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp

key-decisions:
  - "PLAN-04 implemented: OMPL bounds derived from URDF joint_limits_ for all 7 right-arm joints; missing limits trigger explicit WARN; 7-joint guard fires before bounds setup."
  - "D-05 implemented: 19 verbose log items removed per inventory; non-essential per-state/per-goal dumps gone or demoted to DEBUG."
  - "Architectural fix (user-approved deviation, Rule 4): TRAC-IK construction moved out of constructor into init() method called from main after make_shared. Standard rclcpp pattern for shared_from_this()."

requirements-completed: [PLAN-04]

duration: ~50 min
completed: 2026-04-29
---

# Phase 01 Plan 03: Bounds Verification + Debug Cleanup + Smoke Test Summary

**OMPL bounds path tightened (WARN-on-fallback, summary INFO, 7-joint guard); verbose debug logging stripped per D-05 inventory; latent bad_weak_ptr construction crash fixed; runtime smoke launch verified.**

## Performance

- **Tasks:** 3 / 3 auto complete + 1 human-verify checkpoint pending
- **Files modified:** 1 (planner source)
- **LOC delta on planner.cpp:** 867 -> 697 lines (-170 in this plan)
- **Phase total:** 903 -> 697 lines (-206 lines, -22.8%)
- **Build:** `colcon build --packages-select unitree_g1_dex3_stack --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON` exits 0 after every task.
- **RCLCPP_INFO count:** 30+ -> 14 (in plan's [8, 14] target band).

## Final source size and surviving INFOs

The 14 surviving `RCLCPP_INFO` calls:

1. line 62 — "Waiting for /robot_state_publisher service..." (service wait loop, fires once per second until `robot_state_publisher` is up)
2. line 157 — "IKFCLPlannerNode initialized with N joints" (init banner)
3. line 158 — "Using base link: %s, right tip: %s" (init param summary)
4. line 160 — "Planning timeout / time step" (init param summary)
5. line 162 — "Collision skip pairs: N" (init param summary)
6. line 164 — "Skipping collision check for pair: %s" (per-skip-pair, only when non-empty)
7. line 166 — "Planner type: %s" (init param summary)
8. line 167 — "Right arm: base_link = %s, tip_link = %s" (init confirmation, plan explicitly preserves)
9. line 309 — "goal_pose already in base frame" (per-goal received)
10. line 313 — "Transformed goal_pose from frame X to Y" (per-goal received)
11. line 439 — "TRAC-IK result: %s (code: %d)" (per-goal IK result)
12. line 449 — "IK succeeded with neutral seed" (per-goal IK retry success)
13. line 552 — "OMPL bounds set for N right-arm joints: ..." (per-goal bounds, NEW in Plan 01-03)
14. line 621 — "Plan published: N waypoints over M right-arm joints" (per-goal publish, NEW in Plan 01-03)

## Task summary

### Task 1: OMPL bounds tightening — `2352370` (feat)
- WARN on URDF-limit fallback (joint name in message).
- INFO summary line listing all 7 right-arm joints with their final bounds.
- 7-joint guard inserted between planning_joints population and bounds setup.

### Task 2: Strip verbose debug logging per D-05 — `5075db6` (refactor)
- Constructor: removed `printKDLChainInfo` helper + call site, `right_chain_oss` joint-limits dump, `urdf_links_oss` link-list dump, `kdl_right_oss` chain-type dump, per-joint `joint_limits_` for-loop dump, redundant chain-segments/joints count INFOs.
- goalPoseCallback: removed `seg_oss` dump (kept loop body), compacted joint-order check (3 INFOs + 1 ERROR -> single ERROR-on-mismatch), removed start-state oss, target-frame-details RPY block, joint_oss order dump, KDL/TRAC-IK base-link log, per-joint limits dump, IK seed/solution multi-line dumps, second "Input goal pose" log, "Current EE pose" FK log, ikoss IK-solution dump, demoted random-seed retry INFOs to DEBUG, removed "Final EE pose" log, compacted start/goal validity dump (4 lines -> single conditional WARN), replaced verbose final-trajectory-vs-goal max-diff dump with concise "Plan published: N waypoints" INFO.
- isInCollision: removed 4 already-commented-out per-collision-object dumps, removed stale "Only check if at least one link" comment that the Plan 01-02 fix superseded.

### Task 3: Smoke test (build + launch) — verified
- Build: exit 0.
- Launch: `ros2 launch unitree_g1_dex3_stack robot.launch.py` brings up `robot_state_publisher` and parses the URDF cleanly (43 links).
- Planner: smoke run via `ros2 run unitree_g1_dex3_stack ik_fcl_ompl_planner` (see "Workaround" below) reached all 6 init INFO lines:

```
[INFO] [ik_fcl_ompl_planner]: IKFCLPlannerNode initialized with 43 joints
[INFO] [ik_fcl_ompl_planner]: Using base link: torso_link, right tip: right_wrist_yaw_link
[INFO] [ik_fcl_ompl_planner]: Planning timeout: 1.00 seconds, time step: 0.05 seconds
[INFO] [ik_fcl_ompl_planner]: Collision skip pairs: 0
[INFO] [ik_fcl_ompl_planner]: Planner type: RRTConnect
[INFO] [ik_fcl_ompl_planner]: Right arm: base_link = torso_link, tip_link = right_wrist_yaw_link
```

- No FATAL, no ERROR, no `left_tip` / `Left arm` / `kdl_chain_left`, no `bad_weak_ptr`.
- The `OMPL bounds set for ...` per-goal INFO did not fire because no `/goal_pose` message was sent during the 6-second smoke window — that line will appear on the first goal during the human-verify checkpoint.

## Bug fix (Rule 4 deviation, user-approved): bad_weak_ptr in constructor

### Cause

The original `IKFCLPlannerNode` constructor called `shared_from_this()` inside its body via the TRAC-IK constructor's first argument:

```cpp
ik_right = std::make_shared<TRAC_IK::TRAC_IK>(shared_from_this(), kdl_chain_right, ...);
```

By the C++ standard, `shared_from_this()` requires the object to already be managed by a `std::shared_ptr` — but inside the constructor, the `shared_ptr` doesn't exist yet. The result is `std::bad_weak_ptr` -> `std::terminate`.

This bug was latent: it pre-existed Phase 1 (commit `83b34a54` by `iMaxwel`, 2026-04-27, before any of my Phase 1 commits). The `colcon build` step has always passed because the failure is purely runtime. Plan 01-03's smoke test surfaced it for the first time.

### Fix (in `447dcf1`)

- Split construction into two phases: cheap base-class init in the constructor (Node name, `tf_buffer_`/`tf_listener_` members), and the real init (parameter declaration, service call, KDL/TRAC-IK/FK setup, subscribers, init logs) in a new public `init()` method.
- `main()` now does:

```cpp
auto node = std::make_shared<IKFCLPlannerNode>();
node->init();
rclcpp::spin(node);
```

`shared_from_this()` is now valid by the time the TRAC-IK constructor runs. Verified at runtime — see Task 3 smoke test output above.

### Approval trail

Per the user behavioral guidelines and execute-plan deviation rules, this scope expansion (architectural change to construction pattern, beyond Plan 01-03's `<action>` inventory) was a Rule 4 STOP point. I presented three options to the user (fix now / document and defer / pause execution). User selected "fix now (minimal)". The fix is minimal: split into ctor + `init()`, two-line change in `main()`.

## Workaround for pre-existing planner.launch.py bug

`ros2 launch unitree_g1_dex3_stack planner.launch.py` fails on Foxy with:

```
TypeError: Expected 'value' to be one of [<class 'float'>, <class 'int'>, <class 'str'>, <class 'bool'>, <class 'bytes'>], but got '()' of type '<class 'tuple'>'
```

at `launch_ros/utilities/evaluate_parameters.py`. This is a known launch_ros Foxy quirk with empty list parameters: the launch file passes `collision_skip_pairs: []` (the parsed result of an empty `default_value=''` arg), which Foxy's `evaluate_parameter_dict` doesn't accept.

Verified pre-existing: `git show HEAD~5:src/unitree_g1_dex3_stack-main/launch/planner.launch.py` shows the same `default_value=''` setup. Not caused by Plan 01 changes.

### Workaround used

Smoke test invokes the planner binary directly via `ros2 run unitree_g1_dex3_stack ik_fcl_ompl_planner`, which uses the C++ defaults (empty `std::vector<std::string>`) and bypasses the launch_ros parameter evaluation. The C++ binary handles empty `collision_skip_pairs` correctly — the smoke test confirms init succeeds.

### Pitfall flagged for future phases

A small follow-up plan (or Phase 1.1 gap-closure) should fix `planner.launch.py` so it can be invoked end-to-end. Options:
- Pass `collision_skip_pairs` as a comma-separated string and have the C++ side split it (matches the existing `LaunchConfiguration('collision_skip_pairs').perform(context)` pattern).
- Always pass at least one placeholder element (e.g., `['__none__']`) and have the C++ side filter it.
- Switch to ROS 2 Humble (where this is fixed in launch_ros) — but that's a system-level change.

I did not fix this in Plan 01-03 because (a) it's out of inventory scope, (b) the plan explicitly says "Do not modify CMakeLists.txt or package.xml" and the launch fix would similarly be off-scope, and (c) the user-approved bad_weak_ptr fix was the more critical of the two pre-existing issues.

## Source-level verification (RESEARCH.md Level 1, all PASS)

| # | Check | Result |
|---|-------|--------|
| 1 | No left chain refs (`kdl_chain_left|ik_left|fk_left_solver|left_tip_`) | PASS, no match |
| 2 | No `getChain.*left_tip` | PASS, no match |
| 3 | No `use_right` | PASS, no match |
| 4 | Collision filter uses `&&` | line 664 |
| 5 | TF lookup for body links (`tf_buffer_.lookupTransform`) | line 351 |
| 6 | Default URDF is `collision_primitives` in `robot.launch.py` | line 48 |
| 7 | `declare_parameter("left_tip", ...)` removed | PASS, no match |
| 8 | `left_tip` removed from `planner.launch.py` | PASS, no match |

## Deviations from Plan

### 1. [Rule 4 — Architectural change, user-approved] bad_weak_ptr fix

- **Found during:** Task 3 (smoke test) — pre-existing bug surfaced for the first time.
- **Issue:** `shared_from_this()` called from inside the constructor body throws `std::bad_weak_ptr` and terminates the planner before init completes.
- **Fix:** Split construction into ctor + `init()`, call `init()` from `main` after `make_shared`. Committed in `447dcf1`.
- **Files modified:** `src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` (ctor + new `init()` method + `main` change).
- **Verification:** Smoke run produces all 6 init INFOs with no `bad_weak_ptr` / `terminate`.
- **User approval:** Yes — explicit "fix now (minimal)" choice in interactive checkpoint.

### 2. [Rule 1-3 — Plan AC just outside band before tweak] Trimmed two redundant init INFOs

- **Found during:** Task 2 final AC check — `RCLCPP_INFO` count was 16, plan's target band is [8, 14].
- **Issue:** Two init INFOs ("Right arm KDL chain segments: %d" and "Right arm KDL chain joints: %d") were not in the plan's "do NOT remove" preservation list, but I had left them in the first pass.
- **Fix:** Removed both lines (they are redundant with line 167 "Right arm: base_link, tip_link"). INFO count -> 14, in band.
- **Files modified:** planner constructor section.
- **Verification:** `grep -c 'RCLCPP_INFO' ... = 14`, build still exit 0.
- **Committed in:** `5075db6` (part of Task 2 commit).

---

**Total deviations:** 2 (1 architectural with user approval, 1 minor over-cut tweak per Rule 1-3).
**Impact:** Both deviations align Phase 1 with its declared goal. The bad_weak_ptr fix unblocks runtime initialization; the INFO-count trim brings the source within the plan's verbosity target.

## Issues Encountered

- `ros2 launch unitree_g1_dex3_stack planner.launch.py` fails on Foxy due to a pre-existing empty-list parameter bug in the launch file (see "Workaround" section above). Not caused by Plan 01 changes — confirmed via `git show HEAD~5`.
- Initial multi_edit attempt for Task 3 (Plan 01-01) had one chunk fail because a duplicate line existed in two functions; resolved with a follow-up disambiguating edit. Did not affect final state.

## Next Phase Readiness

- Phase 1 mechanically complete: build OK, all 8 RESEARCH.md Level 1 grep checks pass, 6 init INFOs verified at runtime.
- Phase 2 (Path Simplification & Quality) can plan and execute against a stable, runtime-verified Phase 1 planner.
- Future small fix needed for `planner.launch.py` empty-list bug (see "Pitfall flagged for future phases" above).
- **Safety hook in place:** Before this SUMMARY was committed, the user requested an additional safety feature for live verification (smooth arm release on executor terminal exit). Implemented as **Plan 01-04** (`@/home/unitree/Desktop/unitree_dex3/.planning/phases/01-right-arm-only-planner/01-04-PLAN.md`, commit `c6ef43f`). The Plan 01-03 human-verify checkpoint below can therefore be exercised on the live robot with a 3-second graceful arm-release ramp protecting against mid-trajectory Ctrl+C jerks.

## Awaiting user verification

The remaining task is the Plan 01-03 `checkpoint:human-verify` block. Specifically, the user is asked to:

1. Confirm the source-level grep checks above (all 8 PASS).
2. Confirm the init log shows exactly the 6 expected INFOs with no left-arm references (smoke test output above).
3. (Operator-driven) On the live robot in running mode, send a `/goal_pose` in `torso_link` frame:
   - `y: -0.20` -> trajectory contains the 7 right-arm joints, no `left_*_joint`.
   - `y: +0.20` -> trajectory still contains only the 7 right-arm joints (planner did NOT switch to the left arm — D-02 regression check).
   - Optional: a goal close to the torso -> verify `Collision detected between right_*_link and torso_link` fires (D-03 regression check that the old `||` -> new `&&` filter actually catches body collisions).

Reply with "approved" once at least the static + init-log checks pass, or describe any failures.

**Safety net:** Plan 01-04's `gracefulRelease()` is now active. If you Ctrl+C the executor mid-trajectory during the live tests above, expect to see `Graceful release: smoothly transferring arm control to robot body (3s).` in the log followed by a smooth deceleration before the standing pose resumes. See `01-04-SUMMARY.md` for the full behavior contract.

---
*Phase: 01-right-arm-only-planner*
*Completed: 2026-04-29*
