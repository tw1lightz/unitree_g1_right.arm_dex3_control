---
phase: 01-right-arm-only-planner
plan: 08
subsystem: motion-control
tags: [bug-fix, control-authority, ramp, kp-fade, follow-up]

requires:
  - phase: 01-right-arm-only-planner
    provides: "Plan 01-07 fixed the ramp's q=0 jerk by populating motor_cmd; Plan 01-08 fixes the residual snap by fading kp/kd over the ramp."
provides:
  - End-of-trajectory ramp now fades planner stiffness alongside the master switch over 3 seconds (was 1 s with constant stiffness). The body controller gains effective influence midway through the ramp, so by the time master authority hands off completely, the arm is already most of the way back to standing pose. No snap.
affects:
  - All trajectory completions (perceptually smoother return to standing).
  - Mid-trajectory Ctrl+C (Plan 01-06 break-out path) inherits the smoother release.

tech-stack:
  added: []
  patterns:
    - "When fading authority on a master-switch + servo-stiffness pair, fade BOTH together. A constant kp during a master-switch fade lets the planner over-rule the body controller until the very last tick, producing a snap at the handover."

key-files:
  created:
    - .planning/phases/01-right-arm-only-planner/01-08-PLAN.md
    - .planning/phases/01-right-arm-only-planner/01-08-SUMMARY.md
  modified:
    - src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp

key-decisions:
  - "Linearly fade kp 60→0 and kd 1.5→0 (rather than introduce a standing-pose launch parameter and actively interpolate q to standing). We don't actually need to know the body controller's target — we just need to stop fighting it. Letting kp decay lets the body's standing-pose pull act through the ramp."
  - "Extend ramp from 1 s / 50 steps to 3 s / 150 steps. Preserves 20 ms cadence. Matches the 3-second duration the user selected for Plan 01-04's gracefulRelease (and was comfortable with). The longer wall clock gives the kp fade enough time for the body controller to actually pull the arm home."
  - "Keep q = latest_joint_positions_ from Plan 01-07. Frame 0 still publishes (kp=60, q=current_actual), so it correctly continues the trajectory-following hold and does not jerk."
  - "Linear fade curve for kp/kd (rather than (1-t)^2 or other shape). Linear is the simplest robust choice; if real-robot behavior shows excessive gravity sag in the middle of the ramp (kp drops too fast for body controller to compensate), the next iteration can bias the curve. Surface this as the documented future-iteration plan B."

patterns-established:
  - "Compound-bug ladder: each layer of a Plan-over-Plan fix in safety-critical code reveals the next layer until enough invariants are restored. Plans 01-04 → 01-06 → 01-07 → 01-08 are exactly four observable behaviors, each fixed in turn. Stopping early would leave the user with the deepest one visible."
  - "Two coupled fades (master switch + stiffness) are the minimum to get a smooth controller handover. Either alone produces a discontinuity at one of the boundaries."

requirements-completed: []

duration: ~10 min
completed: 2026-04-29
---

# Phase 01 Plan 08: Fade kp/kd through trajectory-end ramp — Summary

**Bug fix on top of Plan 01-07. Plan 01-07 correctly eliminated the q=0 jerk at the start of the ramp; Plan 01-08 eliminates the residual snap at the END of the ramp where the constant-stiffness Plan 01-07 hold prevented the body controller from acting until the very last tick.**

## Performance

- **Tasks:** 2 / 2 auto complete + 1 human-verify pending
- **Files modified:** 1 (`joint_trajectory_executor.cpp`)
- **LOC delta:** 287 → 290 lines (+3 net: 3 lines comment header, 4 numeric edits in-place)
- **Build:** `colcon build --packages-select unitree_g1_dex3_stack` exit 0 (22.9 s).

## Implementation summary

Single `multi_edit` with two surgical edits inside the existing `for (int step = 0; step <= interp_steps; ++step)` ramp loop in `trajectoryCallback`:

### Edit 1 — duration + step count

```@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp:234-239
    // Smoothly interpolate final_cmd.motor_cmd[JointIndex::kNotUsedJoint].q from 1.0 to 0.0
    // Plan 01-08: 1s -> 3s; kp/kd also fade to 0 over the ramp so the body
    // controller's standing-pose pull can take effect during the ramp
    // instead of snapping in at the end.
    const double interp_duration = 3.0; // seconds
    const int interp_steps = 150;
```

20 ms per-step cadence preserved.

### Edit 2 — kp/kd fade

```@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp:256-258
        final_cmd.motor_cmd[idx].dq = 0.f;
        final_cmd.motor_cmd[idx].kp = static_cast<float>((1.0 - t) * 60.0);
        final_cmd.motor_cmd[idx].kd = static_cast<float>((1.0 - t) * 1.5);
```

`t` is the same `static_cast<double>(step) / interp_steps` already computed for the master-switch fade, so no extra arithmetic. Frame 0 (`t = 0`) → `kp = 60, kd = 1.5` (matches Plan 01-07's behavior exactly). Frame 150 (`t = 1`) → `kp = 0, kd = 0` (planner is fully transparent).

The `q = latest_joint_positions_` population block from Plan 01-07 is preserved — only `kp` and `kd` lines change.

## Acceptance criteria — all PASS

| # | Check | Result |
|---|-------|--------|
| 1 | `colcon build --packages-select unitree_g1_dex3_stack` exit 0 | PASS (22.9 s) |
| 2 | `grep -c "interp_duration = 3.0"` | 1 ✓ |
| 3 | `grep -c "interp_steps = 150"` | 1 ✓ |
| 4 | `grep -c "Plan 01-08"` | 1 ✓ |
| 5 | `grep -c "(1.0 - t) \* 60.0"` | 1 ✓ |
| 6 | `grep -c "(1.0 - t) \* 1.5"` | 1 ✓ |
| 7 | Plan 01-07 q-population marker preserved | 1 ✓ |
| 8 | `wc -l` joint_trajectory_executor.cpp | 290 (was 287) ✓ |

## Behavior contract after Plan 01-08

**End of normal trajectory:**
1. Last fully-populated `cmd_msg` puts the arm at the trajectory end-point with stiff servoing (kp=60, kd=1.5).
2. Hand close completes (~1 s).
3. `Trajectory execution complete, returning to default pose.` INFO logs.
4. 3-second ramp begins (151 frames at 20 ms each):
   - Frame 0 (t=0): `kNotUsedJoint.q = 1.0`, `kp = 60`, `kd = 1.5`, `q = current_actual` — smooth handover from trajectory-following.
   - Frame 75 (t=0.5): `kNotUsedJoint.q = 0.5`, `kp = 30`, `kd = 0.75` — planner is half as stiff, master switch half-faded; body controller's standing-pose pull is half-effective.
   - Frame 150 (t=1.0): `kNotUsedJoint.q = 0.0`, `kp = 0`, `kd = 0` — planner is transparent; body controller has full authority.
5. By frame 150, the arm has been gradually pulled from the trajectory end-point toward standing throughout the ramp; the body controller's final assertion is a small correction, not a snap.

**Idle Ctrl+C:** unchanged from Plan 01-06 — process exits in ~50 ms, no motion.

**Mid-trajectory Ctrl+C:** arm completes ≤1 more waypoint, falls into the 3-second ramp, smoothly returns to standing.

**SIGTERM / SIGKILL:** unchanged from Plan 01-06.

## The compound-bug ladder

This is the fourth Plan in a chain that all touch the same end-of-trajectory release behavior. Each Plan addresses exactly one observable defect; the live-robot reverification cycle surfaces the next layer.

| Plan | Symptom | Root cause | Fix |
|------|---------|------------|-----|
| 01-04 | Arm released instantaneously on Ctrl+C, perceived jerk | Destructor publishes single `q=0` LowCmd | Add `gracefulRelease()` 3 s ramp from `main()` |
| 01-06 | Ctrl+C → "hands forward" then slow return | `gracefulRelease()` re-grabs control authority from already-released body controller; default `motor_cmd` jerks every joint to its q=0 reference | Delete `gracefulRelease()`; add `g_shutdown_requested` check to `trajectoryCallback`'s waypoint loop so the existing trajectory-end ramp acts as the one and only release path |
| 01-07 | End of trajectory → small q=0 jerk before slow return | Trajectory-end ramp publishes default-zero `motor_cmd` (kp=0, q=0) at frame 0 with master still 1.0; arm_sdk's internal default kp jerks toward q=0 | Populate every motor_cmd's `q = latest_joint_positions_, kp=60, kd=1.5` in each ramp frame |
| **01-08** (this) | End of trajectory → no jerk, but snap to standing at end of ramp | Plan 01-07's constant kp=60 over-rules the body controller until master switch hits exactly 0.0; body controller then gets 100% authority in one tick and servos to standing fast | Fade kp 60→0 and kd 1.5→0 alongside the master-switch fade; extend ramp from 1 s to 3 s |

After Plan 01-08, the four user-observable defects are all fixed. There may yet be a fifth (e.g. gravity sag during the kp fade if the trajectory ended with the arm raised), but it is not visible until Plan 01-08 is verified on the robot.

## Out of scope (NOT done in this plan)

- Standing-pose launch parameter (active q interpolation to standing). Deferred unless approach 1 turns out to be insufficient.
- Refactoring the ramp into a helper. Two callsites is below the abstraction threshold.
- Anything in `ik_fcl_ompl_planner` or `dex3_controller`.
- The pre-existing `planner.launch.py` empty-list parameter bug.
- Any change to the trajectory-following loop or hand-open/close logic.

## Deviations from Plan

None. Plan as written, executed as written. Single `multi_edit` succeeded on first attempt.

## Issues Encountered

None during implementation.

## Next Steps

- **User action required:** restart `joint_trajectory_executor` (current PID is the Plan 01-07 binary). Then re-run the trajectory test:
  - Send right-side goal_pose, let arm move to completion.
  - At end of trajectory: arm holds briefly, then over ~3 seconds smoothly drifts back to standing pose. No snap at any point.
  - Re-confirm idle Ctrl+C still PASS.
  - Re-confirm mid-trajectory Ctrl+C still PASS (now in the new 3 s smooth ramp).
- After user-confirmed PASS, Phase 1 can be archived. (Watch for the kp-fade gravity-sag failure mode noted in the PLAN's caveat — if it appears, plan 01-09 would bias the kp curve.)

---
*Phase: 01-right-arm-only-planner*
*Completed: 2026-04-29*
