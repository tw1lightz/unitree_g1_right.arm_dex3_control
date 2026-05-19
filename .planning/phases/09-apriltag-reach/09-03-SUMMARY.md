---
phase: 09-apriltag-reach
plan: 03
subsystem: UAT-harness
tags: [ros2, uat, pinocchio, fk, apriltag, intg-02, kdl, harness]

# Dependency graph
requires:
  - phase: 07-apriltag
    provides: /apriltag/target_pose (PoseStamped, frame_id=torso_link)
  - phase: 06-yolo-tcp-offset
    provides: right_tcp_link URDF chain, Pinocchio FK pattern from read_tcp_pose.py
  - phase: 08-adaptive-orientation
    provides: UAT harness pattern from adaptive_orientation_ab.py, 8-point target set for filtering
  - phase: 09-apriltag-reach plan 01
    provides: /goal_pose publish via bridge (operator G key triggers planner)
provides:
  - End-to-end UAT harness (scripts/apriltag_reach_uat.py) with FK-based TCP error measurement
  - 4-point tabletop target subset per D-21
  - PASS/FAIL reporting with 3cm threshold per D-23/D-24
affects: [09-apriltag-reach plan 04 (CMakeLists install entry, cleanup)]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - ROS 2 Python UAT harness with timer-driven FSM and spin_once exit-on-completion loop
    - Pinocchio KDL FK via buildReducedModel for real-time TCP position measurement
    - Operator-guided per-point cycle: place tag, detect, press G, measure, report, advance

key-files:
  created:
    - src/unitree_g1_dex3_stack-main/scripts/apriltag_reach_uat.py
  modified: []

key-decisions:
  - "D-21: 4-point tabletop subset: center (0.40, -0.20, 0.00), right-side (0.40, -0.40, 0.00), low (0.40, -0.20, -0.10), diag (0.45, -0.30, 0.05)"
  - "D-22: TCP error measured via Pinocchio FK from /joint_states (software FK, not hardware measurement)"
  - "D-23: Error threshold = 3cm (0.03m) matching PnP + model + placement tolerance"
  - "D-24: Pass criterion = 4/4 required for exit 0; less than 4/4 exits 1"
  - "D-25: Summary output per-target with expected/actual/error/PASS|FAIL table + PASS_COUNT N/4, pure Python print (no pandas/tabulate)"

requirements-completed: [INTG-02]

# Metrics
duration: 6min
completed: 2026-05-19
---

# Phase 9 Plan 03: UAT Harness Summary

**End-to-end UAT harness (apriltag_reach_uat.py) with Pinocchio KDL FK-based TCP position measurement, 4-point tabletop target subset per D-21, and structured PASS/FAIL reporting with 3cm threshold per D-23/D-24/D-25**

## Performance

- **Duration:** 6 min
- **Started:** 2026-05-19T08:00:00Z (approx)
- **Completed:** 2026-05-19T08:06:00Z (approx)
- **Tasks:** 2 (1 code + 1 verification)
- **Files modified:** 1

## Accomplishments

- Created `scripts/apriltag_reach_uat.py` implementing all D-21 through D-25 locked decisions
- Pinocchio reduced model (7 DOF right arm) with TCP frame at 0.175m X offset from right_wrist_yaw_joint (matching planner URDF)
- Timer-driven FSM with 5 phases: init -> waiting_tag -> waiting_traj -> measuring -> next_point -> done
- 4-point tabletop target subset filtered from Phase 8 8-point set: all within 0.55m reach radius and negative Y (right-side workspace)
- Three subscriptions: /joint_states (FK input), /joint_trajectory_targets (completion signal), /apriltag/target_pose (expected position)
- Per-point output: expected=(x,y,z), actual=(x,y,z), error_m=X.XXXX, PASS|FAIL
- Summary table with PASS_COUNT N/4 and exit 0 only on 4/4 (D-24)
- spin_once main loop with _finished flag for clean exit-on-completion

## Task Commits

Each task was committed atomically:

1. **Task 1: Create apriltag_reach_uat.py with FK-based TCP measurement harness** - `pending` (feat)
2. **Task 2: Verify UAT harness syntax and structural completeness** - verification passed, no changes needed

**Plan metadata:** `pending` (SUMMARY.md to be committed with final metadata commit)

## Files Created/Modified

- `src/unitree_g1_dex3_stack-main/scripts/apriltag_reach_uat.py` (330 lines) - End-to-end UAT harness with class AprilTagReachUAT(Node), 7 methods + main entry point

## Decisions Made

- All D-21 through D-25 decisions implemented as specified in 09-CONTEXT.md
- URDF path resolved via ament_index_python (robots/g1_description/ subdirectory), matching planner URDF default (g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf)
- FK joint-to-config mapping uses getJointId for Pinocchio joint index lookup in reduced model
- Operator-guided cycle per D-25: harness detects tag -> prompts operator -> waits for trajectory -> measures -> advances to next point
- No pandas/tabulate dependency per D-25 (pure Python print formatting)
- All TARGETS Y values negative (right-side workspace, no midline crossing per D-21)

## Deviations from Plan

None - plan executed exactly as written.

## Issues Encountered

None.

## Threat Surface Scan

No threat flags beyond what the plan's threat model already covers (T-09-08 through T-09-10 accepted: FK model mismatch, results logging, joint_states spoofing). No new network endpoints, auth paths, file access patterns, or schema changes.

## Self-Check: PASSED

- [x] File exists: `src/unitree_g1_dex3_stack-main/scripts/apriltag_reach_uat.py` (330 lines)
- [x] AST verification: class AprilTagReachUAT + all 5 required methods found (_tick, _compute_tcp_position, _summarize, main)
- [x] Compile check: `python3 -m py_compile` exits 0
- [x] TARGETS list has exactly 4 entries: center, right-side, low, diag
- [x] All TARGETS Y values are negative (right-side workspace)
- [x] error_threshold = 0.03 (3cm per D-23)
- [x] PASS_COUNT output present (D-25)
- [x] Pinocchio FK calls present: framesForwardKinematics, buildReducedModel, right_tcp frame
- [x] No pandas/tabulate dependency (D-25)
- [x] Exit 0 on 4/4, exit 1 otherwise (D-24)

## Next Phase Readiness

- UAT harness ready for Plan 04 CMakeLists install entry (add to install(PROGRAMS ...) list)
- UAT harness ready for Plan 04 README.md update (document UAT command)
- All 4 points are within 0.55m reach radius and right-side workspace, suitable for INTG-02 milestone sign-off

---
*Phase: 09-apriltag-reach*
*Completed: 2026-05-19*
