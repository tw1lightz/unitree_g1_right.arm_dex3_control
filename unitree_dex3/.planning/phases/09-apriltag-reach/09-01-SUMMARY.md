---
phase: 09-apriltag-reach
plan: 01
subsystem: integration (bridge)
tags: [apriltag, bridge, keyboard-trigger, ros2, tf2, deque, sliding-window]

# Dependency graph
requires:
  - phase: 07-apriltag
    provides: /apriltag/target_pose (PoseStamped, frame_id=torso_link, D-06)
  - phase: 08-adaptive-orientation
    provides: shoulder origin reference (right_shoulder_pitch_link), planner orientation overwrite (D-08)
provides:
  - AprilTag goal bridge node (scripts/apriltag_goal_bridge.py)
  - D-01 through D-14 business logic: cache, 5 guards, keyboard trigger, TF shoulder lookup
affects: [09-apriltag-reach plan 02 (launch), plan 03 (UAT), plan 04 (cleanup)]

# Tech tracking
tech-stack:
  added: []
  patterns:
    - ROS 2 Python bridge node with raw-terminal keyboard reading
    - Sliding-window deque cache with pure-Python averaging
    - Guard-chain pattern (WARN-and-return on each failure)
    - One-shot timer for trajectory completion signal (D-03)
    - TF2 shoulder origin lookup with retry counter (D-13)

key-files:
  created:
    - src/unitree_g1_dex3_stack-main/scripts/apriltag_goal_bridge.py
  modified: []

key-decisions:
  - "D-01: Trigger model = keypress G, single-shot /goal_pose publish (not continuous)"
  - "D-02: Trigger key = G (not K from YOLO era), configurable via ROS parameter"
  - "D-03: In-flight guard via waiting_for_completion flag + trajectory duration timer"
  - "D-07: Sliding window of 5 positions via collections.deque (maxlen=5)"
  - "D-08: Position-only average; orientation copied raw from latest frame (planner overwrites)"
  - "D-09: Stale threshold = 1.0s; age computed via rclpy.time.Time subtraction"
  - "D-10: Empty cache guard rejects G before first detection"
  - "D-11/D-14: Reachability pre-check with 0.55m threshold (Euclidean distance to shoulder)"
  - "D-13: Shoulder origin via TF2 lookup_transform(torso_link, right_shoulder_pitch_link) with 0.5s retry, max 10 attempts"

requirements-completed: [INTG-01]

# Metrics
duration: 4min
completed: 2026-05-19
---

# Phase 9 Plan 01: Bridge Node Summary

**AprilTag goal bridge with sliding-window cache, 5 guard conditions, and G-key trigger for safe human-in-the-loop /goal_pose publishing**

## Performance

- **Duration:** 4 min
- **Started:** 2026-05-19T07:58:00Z
- **Completed:** 2026-05-19T08:02:00Z
- **Tasks:** 2 (1 code + 1 verification)
- **Files modified:** 1

## Accomplishments
- Created `scripts/apriltag_goal_bridge.py` implementing all D-01 through D-14 locked decisions
- Five guard conditions: empty cache (D-10), shoulder unavailable, goal in-flight (D-03), stale data (D-09, 1.0s), reachability exceeded (D-11/D-14, 0.55m)
- Keyboard trigger on configurable G key via raw-terminal os.read (pattern from keyboard_trigger_node.py)
- Sliding-window position cache via collections.deque(maxlen=5) with pure-Python averaging
- TF2 shoulder origin lookup with 0.5s retry timer, fatal after 10 failed attempts (D-13)
- Trajectory completion one-shot timer via /joint_trajectory_targets subscription (D-03)
- destroy_node override restores terminal settings on shutdown

## Task Commits

Each task was committed atomically:

1. **Task 1: Create apriltag_goal_bridge.py with full business logic** - `87b8b12` (feat)
2. **Task 2: Verify bridge node syntax and structural completeness** - verification passed, no changes needed

**Plan metadata:** `pending` (SUMMARY.md to be committed with final metadata commit)

## Files Created/Modified
- `src/unitree_g1_dex3_stack-main/scripts/apriltag_goal_bridge.py` (303 lines) - AprilTag goal bridge node with class AprilTagGoalBridge(Node), 7 methods + main entry point

## Decisions Made
- All D-01 through D-14 decisions implemented as specified in 09-CONTEXT.md
- Trajectory completion signal uses /joint_trajectory_targets (planner output) with one-shot timer following points[-1].time_from_start + 1.0s margin
- Shoulder retry limit = 10 attempts (5s total window) before SystemExit(1)
- Default trigger_key = "g" with case-insensitive comparison
- No extra publish topics, services, or feedback topics (per D-14 deferred ideas)

## Deviations from Plan

None - plan executed exactly as written.

## Issues Encountered

None.

## Threat Surface Scan

No new threat surface beyond what the plan's threat model already covers (T-09-01 through T-09-04 mitigated by guards; T-09-SC accepted no new packages).

## Self-Check: PASSED

- [x] File exists: `src/unitree_g1_dex3_stack-main/scripts/apriltag_goal_bridge.py` (303 lines)
- [x] Commit exists: `87b8b12`
- [x] AST verification: class AprilTagGoalBridge + all 7 required methods found
- [x] Compile check: `python3 -m py_compile` exits 0
- [x] All 6 guard strings present (D-03, D-09, D-10, D-11/D-14, shoulder, success log)
- [x] Positive assertions: deque, lookup_transform, waiting_for_completion, torso_link, right_shoulder_pitch_link
- [x] Negative assertions: no numpy import, no orientation averaging, no extra publish topics

## Next Phase Readiness
- Bridge node is syntactically valid with all required guard logic per D-01 through D-14
- Ready for Plan 02 launch file integration (apriltag_reach.launch.py)
- Ready for Plan 03 UAT harness integration (apriltag_reach_uat.py)

---
*Phase: 09-apriltag-reach*
*Completed: 2026-05-19*
