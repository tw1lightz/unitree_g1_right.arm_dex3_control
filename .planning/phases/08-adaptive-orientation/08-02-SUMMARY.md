---
phase: 8
plan: 02
status: complete
completed: 2026-05-19
requirements:
  - ORI-01
commits:
  - cd15f13
  - dc3b50f
key-files:
  created:
    - src/unitree_g1_dex3_stack-main/scripts/adaptive_orientation_ab.py
  modified:
    - src/unitree_g1_dex3_stack-main/CMakeLists.txt
---

# Plan 08-02 Summary: A/B verification harness for adaptive orientation

## What was built

A non-interactive ROS 2 Python harness that turns Plan 08-01's
adaptive-orientation implementation into a deterministic A/B signal.
One new file + one-line CMakeLists.txt edit. No edits to any other
file. No new dependencies.

### 1. `scripts/adaptive_orientation_ab.py` (new, 194 lines, executable)

Single-file ROS 2 Python node `AdaptiveOrientationAB(Node)`:

- **Module-level constants:**
  - `TARGETS` — exact 8-tuple list `(label, x, y, z)` in `torso_link`,
    matching CONTEXT D-13/D-14 verbatim:
    `center`, `center-near`, `center-far`, `right-side`, `left-of-mid`,
    `low`, `high`, `diag`. Coordinates span +X 0.30..0.55 m, Y -0.40..-0.05 m,
    Z -0.10..+0.15 m — all well outside the 0.05 m shoulder-rejection radius.
  - `BASELINE_QUAT = (-0.68194788, 0.06844694, -0.07816853, 0.72398328)` —
    matches `keyboard_trigger_node.py`'s historical baseline so the only
    variable across A/B runs is the planner's `adaptive_orientation_enabled`
    parameter.
  - `_SETTLE_SEC = 1.0` — publisher↔subscriber connection-warm-up delay
    before the first goal publish.

- **`__init__`:** declares two informational ROS parameters
  (`adaptive: bool = True`, `timeout_sec: float = 3.0`), creates the
  `/goal_pose` publisher and `/joint_trajectory_targets` subscriber,
  initializes a small driver state machine, and starts a 0.1 s timer.

- **`_traj_cb`:** presence-only — sets a flag. The planner already
  validates trajectory contents in its pre-publish auto-fix block.

- **`_publish_goal`:** builds a `PoseStamped` with `frame_id=torso_link`,
  position from the target tuple, orientation = `BASELINE_QUAT`. Resets
  the per-target receive flag *before* publish to avoid race.

- **`_tick` state machine:** `settle → publishing → waiting → publishing → … → done`.
  In `waiting` phase, transitions to next target on either trajectory
  receipt (record PASS) or timeout (record FAIL). After all 8 targets
  processed, logs the per-target table + a `PASS_COUNT n/8 — adaptive=<label>`
  summary line, then sets `_finished=True` and `_exit_status` to 0
  (all PASS) or 1 (any FAIL).

- **`main`:** standard `rclpy.init` → manual `spin_once` loop checking
  `node._finished` (so the harness terminates rather than blocking),
  catches `KeyboardInterrupt` → exit 130, `try/finally` calls
  `node.destroy_node()` + `rclpy.try_shutdown()`, then `sys.exit(exit_status)`.

The script imports only `rclpy`, `rclpy.node`, `geometry_msgs.msg.PoseStamped`,
`trajectory_msgs.msg.JointTrajectory`, and `sys` — nothing executor-related.

### 2. `CMakeLists.txt` (+1 line)

The new script is appended to the existing `install(PROGRAMS …)`
block, alongside `tcp_torso_pose.py`, `keyboard_trigger_node.py`,
and `apriltag_detector_node.py`. No second `install(PROGRAMS)`
block, no new `<depend>`, no new `find_package`.

## Deviations from plan

None. The harness was implemented exactly as specified. The 8 target
labels, the 8-target ordering, the baseline quaternion, the parameter
names + defaults, the topic names, the exit-code contract, and the
single-source-no-imports D-16 boundary all match the plan verbatim.

## Self-Check: PASSED

**Source assertions (Task 1 — harness script):**

| # | Assertion | Result |
|---|-----------|--------|
| 1 | first line `#!/usr/bin/env python3` | ✓ |
| 2 | `test -x scripts/adaptive_orientation_ab.py` | PASS ✓ |
| 3 | line count ≥ 80 | 194 ✓ |
| 4 | `class AdaptiveOrientationAB(Node)` count | 1 ✓ (expect 1) |
| 5 | 8 target-label string occurrences | 8 ✓ (expect ≥8) |
| 6 | `'/goal_pose'` literal | 1 ✓ (expect 1) |
| 7 | `'/joint_trajectory_targets'` literal | 1 ✓ (expect 1) |
| 8 | `BASELINE_QUAT = (-0.68194788, 0.06844694, -0.07816853, 0.72398328)` | 1 ✓ (expect 1) |
| 9 | D-16 boundary: `unitree_hg\|joint_trajectory_executor\|JointTrajectoryExecutor\|Dex3Controller` | 0 ✓ (expect 0) |
| 10 | `python3 -m py_compile` rc | 0 ✓ |
| 11 | `python3 -c 'import ast; ast.parse(...)'` | OK ✓ |

**Source assertions (Task 2 — CMakeLists.txt):**

| # | Assertion | Result |
|---|-----------|--------|
| 12 | `scripts/adaptive_orientation_ab.py` count | 1 ✓ (expect 1) |
| 13 | line lives inside `install(PROGRAMS … DESTINATION)` block | 1 ✓ (expect 1) |
| 14 | only ONE `install(PROGRAMS` directive | 1 ✓ (expect 1) |

**Build assertions:**

- `colcon build --packages-select unitree_g1_dex3_stack --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON` rc=0 (incremental, 3.5 s).
- `install/unitree_g1_dex3_stack/lib/unitree_g1_dex3_stack/adaptive_orientation_ab.py` is `-rwxr-xr-x` (executable bit preserved through install).
- `ros2 pkg executables unitree_g1_dex3_stack | grep adaptive_orientation_ab.py` returns the new entry.

**Behavior assertions (deferred to live hardware):**

- D-15 acceptance — with the planner running on the live G1 with
  `adaptive_orientation_enabled:=true`, `ros2 run unitree_g1_dex3_stack
  adaptive_orientation_ab.py` should exit 0 and report `PASS_COUNT 8/8`
  within ~30 s. With `:=false`, the run is informational baseline.
- Recorded as human_verification in `08-VERIFICATION.md`.

## What this enables

- ORI-01 verification has a measurable, reproducible exit-code signal
  that operators can re-run after planner regressions.
- Phase 9 end-to-end integration can reuse the harness as a smoke
  test before bridging `/apriltag/target_pose` → `/goal_pose`.

## Commits

- `cd15f13 feat(08-02): add adaptive_orientation_ab.py A/B harness`
- `dc3b50f build(08-02): install adaptive_orientation_ab.py`

---
*Phase: 08-adaptive-orientation*
*Completed: 2026-05-19*
