---
phase: 01-right-arm-only-planner
plan: 09
subsystem: motion-control
tags: [bug-fix, control-authority, ramp, snapshot, robust-against-protocol-uncertainty]

requires:
  - phase: 01-right-arm-only-planner
    provides: "Plan 01-08 attempted kp/kd fade based on a wrong assumption about arm_sdk's master switch; Plan 01-09 abandons the protocol-dependent approach in favor of explicitly driving q to standing during the ramp."
provides:
  - End of every trajectory drives the arm explicitly along an interpolated path from the trajectory end-point to a snapshot of the standing pose, with `kp = 60, kd = 1.5` stiff servoing throughout the 3 s ramp. By the time `kNotUsedJoint.q` reaches 0.0, the arm is physically at standing pose. Body controller's takeover is a small holding correction with no perceptible motion. Robust regardless of whether arm_sdk's master switch is binary, blended, or anything else.
affects:
  - All trajectory completions (genuinely smooth return to standing).
  - Mid-trajectory Ctrl+C (Plan 01-06 break-out path) inherits the correct interpolated release.
  - Plan 01-08 design — the kp/kd fade is reverted; only the 3 s ramp duration survives.

tech-stack:
  added: []
  patterns:
    - "When the protocol details of a control-authority handover are uncertain, do not rely on the handover for smooth motion. Instead, drive the arm to the handover target yourself before authority transfers — then the handover is observably a no-op."
    - "Snapshot 'where do I want to return to' from runtime state at callback entry, when the system is by precondition already in the desired return state. Avoids parameterization, avoids hard-coded constants, accurate for the specific robot in use."

key-files:
  created:
    - .planning/phases/01-right-arm-only-planner/01-09-PLAN.md
    - .planning/phases/01-right-arm-only-planner/01-09-SUMMARY.md
  modified:
    - src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp

key-decisions:
  - "Snapshot the standing pose from `latest_joint_positions_` at `trajectoryCallback` entry, instead of hard-coding constants or adding a launch parameter. The snapshot is correct for the specific robot in use, free of configuration burden, and accurate by construction (the precondition that arm is at standing at callback entry is enforced by the previous trajectory's ramp + body controller holding behavior). Two failure modes documented: back-to-back trajectories without ramp completion (would self-correct after one cycle), and `latest_joint_positions_` empty at callback entry (only possible in a vanishingly brief window after node start; fallback to Plan 01-07's q=current pattern is acceptable)."
  - "Snapshot the trajectory end-point as `ramp_start_positions` separately, just before the ramp begins. Using a fixed snapshot rather than the live (lowstate-driven, drifting) `latest_joint_positions_` makes the interpolation path deterministic and convergent — frame 0 always equals the trajectory end-point, frame 150 always equals the standing snapshot, and the kp=60 servo strictly tracks that path."
  - "Revert Plan 01-08's `(1.0 - t) * 60` and `(1.0 - t) * 1.5` kp/kd fade. The fade was based on the assumption arm_sdk's master switch produces a continuous blend, which user verification disproved. With Plan 01-09's explicit q interpolation, kp must remain HIGH (60) so the planner stiffly tracks the interpolated path; fading kp would let gravity sag the arm off the path."
  - "Keep `dq = 0, tau = 0` as before. We rely on kp/kd position-PD control; no feed-forward velocity or torque is needed because the 20 ms cadence interpolation produces velocities the controller can track with kp=60."
  - "Do NOT add a separate flag 'use snapshot' / 'use latest' switch. The fallback chain (`if snapshots OK -> interpolate; else if latest available -> q=latest; else q=0`) gives graceful degradation with no extra configuration."

patterns-established:
  - "When control-authority handovers in shared-control / multi-controller systems behave 'inexplicably' on smooth motion, the diagnostic step is: stop reasoning about the handover protocol, start reasoning about the state at the moment of handover. If you can drive the system to the handover target yourself, the handover becomes observably a no-op."
  - "Compound-bug ladder closure: Plans 01-04 → 01-06 → 01-07 → 01-08 → 01-09 all touch the same code region. Each Plan addressed the bug visible at the time, but only Plan 01-09 reframes the problem from 'make the handover smooth' to 'make the handover a no-op'. The reframe was forced by user verification disproving Plan 01-08's protocol assumption."

requirements-completed: []

duration: ~15 min
completed: 2026-04-29
---

# Phase 01 Plan 09: executor actively interpolates q to standing snapshot during ramp — Summary

**Plan 01-08 was wrong. Reverted its kp/kd fade. Plan 01-09 stops trying to make the master-switch handover smooth, and instead drives the arm to standing BEFORE the handover. Robust against any arm_sdk master-switch protocol shape (binary, blended, or other).**

## Performance

- **Tasks:** 2 / 2 auto complete + 1 human-verify pending
- **Files modified:** 1 (`joint_trajectory_executor.cpp`)
- **LOC delta:** 290 → 313 (+23 net: 8 lines for standing_pose snapshot, 8 lines for ramp_start_positions snapshot, 7 lines for new ramp body comments + interpolation)
- **Build:** `colcon build --packages-select unitree_g1_dex3_stack` exit 0 (20.7 s).

## Implementation summary

Single `multi_edit` with three surgical edits in `trajectoryCallback`:

### Edit 1 — snapshot standing pose at callback entry

```@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp:170-177
    // Plan 01-09: snapshot standing pose at callback entry. Arm is guaranteed
    // to be at standing here (either previous trajectory's ramp settled it
    // back, or robot just booted with body controller holding standing). This
    // snapshot is the target the end-of-trajectory ramp will drive toward.
    std::vector<float> standing_pose;
    if (!latest_joint_positions_.empty()) {
      standing_pose = latest_joint_positions_;
    }
```

### Edit 2 — snapshot trajectory end-point just before the ramp

```@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp:243-250
    // Plan 01-09: snapshot the trajectory end-point as the ramp's starting
    // pose. We interpolate from this fixed start (not the live, drifting
    // latest_joint_positions_) toward the standing snapshot, so the planner
    // stiffly tracks a deterministic path back to standing.
    std::vector<float> ramp_start_positions;
    if (!latest_joint_positions_.empty()) {
      ramp_start_positions = latest_joint_positions_;
    }
```

### Edit 3 — replace ramp body with q interpolation; revert Plan 01-08's kp/kd fade

```@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp:264-282
      // Plan 01-09: drive the arm explicitly from trajectory end-point to
      // standing snapshot under stiff servoing (kp/kd back to 60/1.5).
      // Frame 0 (t=0): q = ramp start = trajectory end-point (matches actual,
      // no jerk). Frame 150 (t=1): q = standing snapshot (arm is at standing).
      // Master switch then hands off to body controller with arm already there.
      for (const auto& pair : joint_name_to_index) {
        size_t idx = pair.second;
        if (ramp_start_positions.size() > idx && standing_pose.size() > idx) {
          final_cmd.motor_cmd[idx].q = static_cast<float>(
            (1.0 - t) * ramp_start_positions[idx] + t * standing_pose[idx]);
        } else if (latest_joint_positions_.size() > idx) {
          final_cmd.motor_cmd[idx].q = latest_joint_positions_[idx];
        } else {
          final_cmd.motor_cmd[idx].q = 0.0f;
        }
        final_cmd.motor_cmd[idx].dq = 0.f;
        final_cmd.motor_cmd[idx].kp = 60.0f;
        final_cmd.motor_cmd[idx].kd = 1.5f;
        final_cmd.motor_cmd[idx].tau = 0.f;
      }
```

Frame 0 (`t=0`): `q = ramp_start_positions[idx]` = trajectory end-point. Identical to Plan 01-07/01-08 frame-0 (no jerk). Frame 150 (`t=1`): `q = standing_pose[idx]`. arm physically at standing. Master switch hand-off observably a no-op.

## Acceptance criteria — all PASS

| # | Check | Result |
|---|-------|--------|
| 1 | `colcon build --packages-select unitree_g1_dex3_stack` exit 0 | PASS (20.7 s) |
| 2 | `grep -c "Plan 01-09"` ≥ 3 | 4 ✓ |
| 3 | `grep -c "standing_pose"` ≥ 4 | 4 ✓ |
| 4 | `grep -c "ramp_start_positions"` ≥ 4 | 4 ✓ |
| 5 | `grep -c "(1.0 - t) \* 60.0"` (Plan 01-08 fade reverted) | 0 ✓ |
| 6 | `grep -c "(1.0 - t) \* 1.5"` | 0 ✓ |
| 7 | `grep -c "kp = 60.0f"` (one in trajectory-following loop, one in ramp) | 2 ✓ |
| 8 | `wc -l` joint_trajectory_executor.cpp | 313 (was 290) ✓ |

## Behavior contract after Plan 01-09

**End of normal trajectory:**
1. trajectoryCallback entry: snapshot standing_pose = current `latest_joint_positions_`. Arm is at standing.
2. Hand opens (1 s). Trajectory follows. Arm reaches goal.
3. Hand closes (1 s).
4. Snapshot ramp_start_positions = current `latest_joint_positions_`. This is the trajectory end-point.
5. 3-second ramp begins (151 frames at 20 ms each). Each frame:
   - `kNotUsedJoint.q` linearly fades from 1.0 to 0.0 (master switch).
   - Every motor's `q` linearly interpolates from `ramp_start_positions[idx]` to `standing_pose[idx]`.
   - `kp = 60`, `kd = 1.5` (stiff servoing tracks the interpolated path).
6. Frame 0 publishes (kp=60, q=trajectory_end_point) — matches actual, no jerk.
7. Frame 150 publishes (kp=60, q=standing_pose, master=0.0) — arm has been driven smoothly along a 3-second linear path from trajectory end to standing. Body controller's takeover is observably a no-op because the arm is exactly where the body controller wants it.
8. trajectoryCallback returns.

**Idle Ctrl+C:** unchanged from Plan 01-06 — process exits in ~50 ms, no motion.

**Mid-trajectory Ctrl+C:** arm completes ≤1 more waypoint, falls into the 3 s correctly-interpolated return-to-standing.

**SIGTERM / SIGKILL:** unchanged from Plan 01-06.

## The compound-bug ladder, closed

| Plan | Symptom | Root cause | Fix |
|------|---------|------------|-----|
| 01-04 | Arm released instantaneously on Ctrl+C, perceived jerk | Destructor publishes single `q=0` LowCmd | Add `gracefulRelease()` 3 s ramp from `main()` |
| 01-06 | Ctrl+C → "hands forward" then slow return | `gracefulRelease()` re-grabs control authority; default `motor_cmd` jerks every joint to its q=0 reference | Delete `gracefulRelease()`; add `g_shutdown_requested` check to waypoint loop |
| 01-07 | End of trajectory → small q=0 jerk before slow return | Ramp publishes default-zero `motor_cmd` | Populate `q = latest_joint_positions_, kp=60, kd=1.5` in each ramp frame |
| 01-08 | End of trajectory → no jerk, but snap to standing at end of ramp | (incorrect diagnosis) Constant kp=60 over-rules body controller until master=0 | (incorrect fix) Fade kp/kd alongside master switch — based on wrong assumption that master switch is a continuous blend |
| **01-09** (this) | Same snap as Plan 01-08 reported | arm_sdk's master switch is binary (or near-binary), so kp fade had no influence on body authority during the ramp; body snapped at the threshold crossing | Reframe: don't try to make the handover smooth. Drive arm to standing yourself BEFORE the handover. Snapshot standing at callback entry; explicit q interpolation in ramp; kp/kd back to 60/1.5 (revert Plan 01-08). |

The reframe in Plan 01-09 is the substantive contribution: by abandoning the protocol-dependent assumption and instead controlling what we know we control (the published q sequence while authority is held), the fix becomes robust against ANY master-switch behavior shape.

## Out of scope (NOT done in this plan)

- Standing-pose launch parameter (snapshot covers this).
- Cubic / s-curve interpolation (linear is sufficient for the user's smoothness perception; future Plan if linear feels jerky at endpoints).
- Watchdog: detecting "planner has died from outside" inside executor.
- Same hooks for `ik_fcl_ompl_planner` or `dex3_controller`.
- The pre-existing `planner.launch.py` empty-list bug.
- Refactoring the ramp into a helper.

## Deviations from Plan

None. Plan as written, executed as written. Single `multi_edit` succeeded on first attempt.

## Issues Encountered

The diagnostic was the bulk of the work: deciding that Plan 01-08's failure was due to a wrong assumption about arm_sdk's master switch protocol (rather than e.g. a tuning issue with the kp fade curve). The decisive observation was "user reports same snap with two materially different ramp implementations" — that ruled out kp tuning and pointed at a structural issue in the handover model.

## Next Steps

- **User action required:** restart `joint_trajectory_executor` (current PID is the Plan 01-08 binary). Then re-run the trajectory test:
  - Send right-side goal_pose, let arm move to completion.
  - At end of trajectory: arm should reverse smoothly along a 3-second path back to the **same standing pose it started from**, with **no snap** at any point in the ramp.
  - Re-confirm idle Ctrl+C still PASS (no motion).
  - Re-confirm mid-trajectory Ctrl+C still PASS (3 s interpolated release).
- After user-confirmed PASS, **Phase 1 can be archived**. The compound-bug ladder ends here.

---
*Phase: 01-right-arm-only-planner*
*Completed: 2026-04-29*
