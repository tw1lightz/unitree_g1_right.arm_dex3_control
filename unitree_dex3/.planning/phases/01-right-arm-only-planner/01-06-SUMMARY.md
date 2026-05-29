---
phase: 01-right-arm-only-planner
plan: 06
subsystem: motion-control
tags: [bug-fix, regression-fix, control-authority, ros2, signal-handling, arm-sdk]

requires:
  - phase: 01-right-arm-only-planner
    provides: "Executor with Plan 01-04 graceful-release scaffolding (signal handler, atomic flag, polling loop) — Plan 01-06 inherits the scaffolding and removes the broken ramp."
provides:
  - On Ctrl+C / SIGTERM, the executor no longer jerks every joint to its `q = 0` default-constructed pose. Mid-trajectory Ctrl+C breaks out of the waypoint loop at the next iteration and falls through to the existing 1-second end-of-trajectory ramp, producing a smooth in-flight release. Idle-state Ctrl+C exits the spin loop in ~50 ms without publishing any extra LowCmd; the destructor's instantaneous `q = 0` publish becomes a redundant-but-harmless reaffirmation since the body controller is already in charge.
affects:
  - Plan 01-04 design — the originally-planned separate `gracefulRelease()` method called from `main()` is fully removed; only its signal-handler scaffolding (atomic flag + uninstall + polling loop) survives.
  - Phase 1 verification path — the human-verify checkpoint can now exercise the safety net without the regression.

tech-stack:
  added: []
  patterns:
    - "ROS 2 graceful shutdown for hardware-control nodes: one ramp, in the data-producing callback, immediately after a fully-populated frame. Never run a second ramp from `main()` once the callback has already released authority."

key-files:
  created:
    - .planning/phases/01-right-arm-only-planner/01-06-PLAN.md
    - .planning/phases/01-right-arm-only-planner/01-06-SUMMARY.md
  modified:
    - src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp
    - .planning/phases/01-right-arm-only-planner/01-04-SUMMARY.md (regression cross-reference added)

key-decisions:
  - "**Never publish a LowCmd with `kNotUsedJoint.q = 1.0` from a state where the body controller is already in charge.** The master switch is a snap toggle, not a fade. Re-asserting `q = 1.0` even for one frame causes arm_sdk to immediately re-grab planner authority and apply whatever the rest of the (default-constructed, all-zero) `motor_cmd` array says. This is the root cause Plan 01-04 missed and Plan 01-06 fixes."
  - "Delete `gracefulRelease()` entirely instead of patching it. The trajectoryCallback end-of-trajectory ramp (line ~240) is the ONLY graceful release the system ever needed — it works correctly because it is preceded by a fully-populated `cmd_msg` (kp=60, kd=1.5, q=current trajectory point), so the ramp's first kp=0+q=0 message just slackens the joints, then the master switch fades. Reusing the existing ramp requires zero new logic; trying to make `gracefulRelease()` 'safe' would require tracking last-published `kNotUsedJoint.q` value, populating `motor_cmd[i].q` from `latest_joint_positions_`, and replicating kp/kd — and would still risk the steal whenever `gracefulRelease()` runs from idle state."
  - "Achieve mid-trajectory responsiveness with a 1-condition for-loop change instead of a separate cleanup path. Adding `&& !g_shutdown_requested.load()` to the waypoint loop lets the existing trailing code (hand close + ramp) act as the cleanup path. No new branches, no new code paths, no new state."
  - "Keep the destructor's instantaneous `q = 0.0` publish as-is. In the new code paths it is always redundant (body controller is already in charge by the time the destructor runs), but on a forced exit where the trajectoryCallback was somehow holding authority, it is a harmless final reaffirmation. Cost: 3 lines and one published LowCmd. Safety: never wrong."

patterns-established:
  - "When a hardware-control plan introduces a new authority-fade ramp, audit ALL preceding LowCmd-publishing paths: any one of them that already does (or naturally ends with) the same fade is the right place to add the shutdown hook, not main()."
  - "Whenever a 'cleanup' callable is called from `main()` after a spin loop exit, ask: is the callable's first published frame consistent with the LAST frame published by the producer (the callbacks)? If the producer ends with `master=0` and the cleanup starts with `master=1`, that single-frame transition is a discontinuity — the OS scheduler does not interpolate between published messages."

requirements-completed: []

duration: ~20 min
completed: 2026-04-29
---

# Phase 01 Plan 06: Fix gracefulRelease control-authority steal jerk — Summary

**Bug fix for the Plan 01-04 regression where pressing Ctrl+C on the executor jerked the arm to q=0 ("hands forward") before the smooth release.**

## Performance

- **Tasks:** 2 / 2 auto complete + 1 human-verify pending
- **Files modified:** 1 (`joint_trajectory_executor.cpp`)
- **LOC delta:** 286 → 272 lines (-14 net: -24 deleted gracefulRelease method, -3 deleted call from main, +5 expanded comments, +1 modified for-loop condition, +7 new comments tying to Plan 01-06)
- **Build:** `colcon build --packages-select unitree_g1_dex3_stack` exit 0 (20.9 s).

## Implementation summary

Three surgical edits + two comment refreshes, applied via two `multi_edit` calls on `@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp`:

### Edit 1 — delete the entire `gracefulRelease()` method

The 24-line block that previously occupied lines 126-149 is gone. The class now goes straight from the destructor closing brace to the `private:` keyword. No new helpers introduced.

### Edit 2 — add shutdown check to `trajectoryCallback` waypoint loop

```@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp:174-178
    // Plan 01-06: honor SIGINT/SIGTERM mid-trajectory by breaking out of
    // the waypoint loop at the next iteration. The trailing hand-close +
    // 1s end-of-trajectory ramp then runs from the current trajectory
    // point, producing a smooth release without re-grabbing authority.
    for (size_t i = 0; i < msg->points.size() && !g_shutdown_requested.load(); ++i) {
```

Single-condition addition. The existing trailing block (hand close at line ~228 + end-of-trajectory ramp at line ~240) is unchanged and acts as the cleanup path.

### Edit 3 — remove `gracefulRelease()` call from `main()`

```@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp:259-264
  // SIGINT/SIGTERM received (or rclcpp::ok went false). The trajectory-end
  // ramp inside trajectoryCallback (line ~249) is the only graceful release
  // we need; if a callback was running, its ramp has already executed by
  // the time we get here. The destructor's instantaneous q=0.0 publish is
  // a harmless reaffirmation in idle state. See Plan 01-06.
  rclcpp::shutdown();
```

The 3-line `node->gracefulRelease()` call (and the comment that introduced it) is replaced with the explanatory comment above. The polling loop, signal handler installation, and `uninstall_signal_handlers()` are all preserved — they are still required for clean main-loop exit and mid-trajectory break-out.

### Stale-comment refresh

Two comments that previously referenced the now-deleted `gracefulRelease()` were rewritten to reflect the new behavior:

- File-scope `g_shutdown_requested` header (lines 27-33) now describes the dual purpose of the flag (main-loop exit + trajectoryCallback break-out) and explicitly names Plan 01-06 as the ramp's deletion point.
- Destructor comment (lines 121-126) now describes the destructor's role as a redundant-but-harmless final reaffirmation of body-controller authority, with the SIGKILL caveat preserved.

## Acceptance criteria — all PASS

| # | Check | Result |
|---|-------|--------|
| 1 | `colcon build --packages-select unitree_g1_dex3_stack` exit 0 | PASS (20.9 s) |
| 2 | `grep -c "gracefulRelease"` | 0 ✓ (was 7) |
| 3 | `grep -c "void gracefulRelease"` | 0 ✓ |
| 4 | `grep -c "g_shutdown_requested.load"` | 2 ✓ (main loop + trajectoryCallback for-loop) |
| 5 | For-loop with shutdown check | line 184 ✓ |
| 6 | Destructor q=0.0 fallback preserved | line 128 ✓ |
| 7 | Trajectory-end ramp preserved | line 240 ✓ |
| 8 | `uninstall_signal_handlers` preserved | line 254 ✓ |
| 9 | `wc -l` joint_trajectory_executor.cpp | 272 (was 286) ✓ |

## Behavior contract after Plan 01-06

**Idle Ctrl+C** (no trajectory in flight, body controller in charge):
1. Signal handler sets `g_shutdown_requested = true` (signal-safe atomic store).
2. Main polling loop sees the flag at next `spin_some(50ms)` tick (worst-case ~50 ms latency).
3. Loop exits, `rclcpp::shutdown()` runs, node destructor runs.
4. Destructor publishes one LowCmd with `kNotUsedJoint.q = 0.0` — redundant but harmless because the body is already in charge.
5. Process exits.

**Mid-trajectory Ctrl+C** (waypoint loop is iterating, planner has authority):
1. Signal handler sets the flag.
2. `trajectoryCallback`'s next loop iteration evaluates `i < msg->points.size() && !g_shutdown_requested.load()` and breaks out.
3. Trailing code runs: hand close (~1 s) + end-of-trajectory ramp (1 s). The ramp is preceded by the last fully-populated `cmd_msg`, so its first kp=0+q=0 frame correctly slackens the joints, then the master switch fades to 0.0.
4. `trajectoryCallback` returns. Main polling loop sees the flag, exits.
5. Destructor publishes the redundant `q = 0.0` (harmless).
6. Process exits.

**SIGTERM**: same as Ctrl+C.

**SIGKILL**: degraded path. Destructor may not run at all; arm_sdk's own timeout takes over. Unchanged from Plan 01-04 and explicitly out of scope.

## Why the trajectory-end ramp works correctly even though `gracefulRelease()` did not

The trajectory-end ramp at line ~240 is byte-for-byte equivalent to what `gracefulRelease()` used to do. The difference is **what precedes it**:

- Before the trajectory-end ramp, the most recent published LowCmd is a fully-populated `cmd_msg` with `kp = 60.0, kd = 1.5, q = current_trajectory_point, kNotUsedJoint.q = 0.5`. arm_sdk is holding the joints stiffly at that target.
- The ramp's first frame (`step = 0` → `kNotUsedJoint.q = 1.0`, all else default zero) **does** also re-assert planner authority for one frame, but because the immediately previous frame was the populated `cmd_msg`, arm_sdk's "use the latest LowCmd" semantics interpolate between the two: the joints just go limp (kp=0+kd=0) at the current trajectory point. Then the master switch fades from 1.0 toward 0.0 over the next 50 frames, and the body controller smoothly takes over.
- Before `gracefulRelease()`, the most recent published LowCmd was the LAST frame of the trajectory-end ramp (`kNotUsedJoint.q = 0.0`, all else default zero) — body controller is in charge, joints are following the standing-pose servo. `gracefulRelease()`'s first frame (`kNotUsedJoint.q = 1.0`) is a discontinuity: the master switch flips, planner takes over, joints suddenly receive `q = 0` targets with arm_sdk's default kp/kd, and they jerk to the q=0 configuration ("hands forward"). The 150-step fade then walks the master switch back to 0.0 over 3 seconds, but the damage was done in the first 20 ms.

This is why deleting `gracefulRelease()` (rather than trying to patch it) is the correct minimal fix.

## Out of scope (NOT done in this plan)

- Any change to the trajectory-end ramp (line ~240). It works correctly when reached after a populated `cmd_msg`, which is exactly how the new break-out path uses it.
- Watchdog: detecting "planner has died from outside" inside executor and proactively releasing.
- Same hooks for `ik_fcl_ompl_planner` (planner is upstream; killing it stops trajectory generation but doesn't strand the arm).
- Same hooks for `dex3_controller`.
- The pre-existing `planner.launch.py` empty-list parameter bug.

## Deviations from Plan

**1. [Surgical scope expansion] — refresh two stale comments that referenced `gracefulRelease()`**

- **Found during:** post-edit review of remaining mentions of "gracefulRelease" in the file.
- **Issue:** The file-scope `g_shutdown_requested` comment block and the destructor's last-resort comment both still pointed at `gracefulRelease()` (which no longer exists), creating two stale documentation references.
- **Fix:** rewrote both comments in the same edit batch to describe the actual current behavior (dual flag purpose; destructor as harmless reaffirmation). No code lines changed, only comment lines.
- **Justification under Rule 3 (Surgical Changes):** the comments are a direct artifact of Edits 1+2, not unrelated cleanup. Leaving them stale would mislead future maintainers.
- **Approval:** mechanical staleness fix, no behavioral change.

**Total deviations:** 1 (mechanical comment refresh).

## Issues Encountered

None during implementation. The bug analysis was the bulk of the work; the edits themselves were 3 small operations, all applied successfully on the first `multi_edit` call.

## Next Steps

- **User action required:** rebuild already done, but `joint_trajectory_executor` process needs to be restarted (the running PID `111504` is the pre-Plan-01-06 binary). Then re-run the Ctrl+C / SIGTERM scenarios:
  - Idle Ctrl+C: process exits in <100 ms, no jerk, no "hands forward".
  - Mid-trajectory Ctrl+C: arm completes at most one more waypoint, then runs the 1 s end-of-trajectory ramp from the current point. No "hands forward". Total stop-and-release ≤ ~2 seconds.
  - `kill -SIGTERM`: same as Ctrl+C.
  - `kill -9`: degraded fallback (arm_sdk timeout), unchanged.
- After user-confirmed PASS, Phase 1 can be archived.

---
*Phase: 01-right-arm-only-planner*
*Completed: 2026-04-29*
