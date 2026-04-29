---
phase: 01-right-arm-only-planner
plan: 07
subsystem: motion-control
tags: [bug-fix, pre-existing, control-authority, ros2, arm-sdk]

requires:
  - phase: 01-right-arm-only-planner
    provides: "Plan 01-06 deleted gracefulRelease(), exposing the smaller trajectory-end ramp jerk that had been masked by the gracefulRelease one."
provides:
  - At the end of every trajectory, the executor's 1-second master-switch fade no longer produces a visible jerk. Each ramp frame holds every joint at its actual current position (`latest_joint_positions_`) with the same `kp = 60`, `kd = 1.5` as the trajectory-following frames; only `kNotUsedJoint.q` is faded from 1.0 to 0.0.
affects:
  - User experience on every trajectory completion (no longer feels jerky).
  - Mid-trajectory Ctrl+C path from Plan 01-06 (it falls into this same ramp; now the smooth release is genuinely smooth from any break-point).

tech-stack:
  added: []
  patterns:
    - "When fading authority on `kNotUsedJoint.q`, populate the rest of the LowCmd (`kp`, `kd`, `q` from current joint state) so arm_sdk holds the joints stiffly during the fade. Never publish a default-constructed LowCmd while `kNotUsedJoint.q > 0`."

key-files:
  created:
    - .planning/phases/01-right-arm-only-planner/01-07-PLAN.md
    - .planning/phases/01-right-arm-only-planner/01-07-SUMMARY.md
  modified:
    - src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp

key-decisions:
  - "Apply the same population block the trajectory-following loop uses (line 184-192). It is the proof-of-correctness: those frames don't jerk, so this pattern is exactly what arm_sdk wants. Copying byte-for-byte (rather than abstracting a helper) keeps the diff trivially auditable and avoids introducing a new abstraction for a 2-callsite pattern."
  - "Use `latest_joint_positions_` (lowstate-driven) rather than the trajectory's last point. The lowstate values reflect the **actual** physical pose including servo-tracking error; that's exactly what the body controller would smoothly absorb when it takes over. Using the trajectory's last point would be slightly off whenever the controller hadn't fully converged, introducing a small position step that would itself cause a micro-jerk."
  - "Do not refactor into a `publishHoldFrame(value)` helper. Two callsites (this ramp + main loop), small saving, and the trajectory-following frames overwrite `q` with trajectory targets after the population — threading that distinction through a helper is more code than the duplication."

patterns-established:
  - "Pre-existing bug surfaced by removing a masking bug: when the larger of two compounding bugs is fixed, the smaller one becomes visible. Plan a verification round AFTER each Plan that removes a major code path."
  - "On compound bugs in safety-critical code paths, fix one bug at a time and re-verify on hardware between fixes. Each Plan in 01-04 -> 01-06 -> 01-07 represented exactly one observable defect; trying to bundle them would have made root-causing this jerk much harder."

requirements-completed: []

duration: ~15 min
completed: 2026-04-29
---

# Phase 01 Plan 07: Fix trajectory-end ramp q=0 jerk — Summary

**Bug fix for a pre-existing defect in `trajectoryCallback`'s end-of-trajectory 1-second ramp. The ramp default-constructed each LowCmd and only set `kNotUsedJoint.q`; on its first frame (still master=1.0) arm_sdk's internal default kp jerked every joint toward its q=0 reference pose. Bug present since `83b34a5 project_init`; masked by Plan 01-04's `gracefulRelease()` jerk; surfaced after Plan 01-06 deleted `gracefulRelease()`.**

## Performance

- **Tasks:** 2 / 2 auto complete + 1 human-verify pending
- **Files modified:** 1 (`joint_trajectory_executor.cpp`)
- **LOC delta:** 272 → 287 (+15 lines: 3 comment lines + 12 lines of population block)
- **Build:** `colcon build --packages-select unitree_g1_dex3_stack` exit 0 (24.5 s).

## Implementation summary

Single surgical edit at `@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp:238-260` (the end-of-trajectory ramp's for-loop body):

```@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp:241-258
      unitree_hg::msg::LowCmd final_cmd;
      final_cmd.motor_cmd[JointIndex::kNotUsedJoint].q = static_cast<float>(value);
      // Plan 01-07: hold every joint at its current actual position with the
      // same kp/kd as the trajectory-following frames, so arm_sdk never sees
      // the default kp=0+q=0 fields and never jerks toward the q=0 reference.
      for (const auto& pair : joint_name_to_index) {
        size_t idx = pair.second;
        if (latest_joint_positions_.size() > idx) {
          final_cmd.motor_cmd[idx].q = latest_joint_positions_[idx];
        } else {
          final_cmd.motor_cmd[idx].q = 0.0f;
        }
        final_cmd.motor_cmd[idx].dq = 0.f;
        final_cmd.motor_cmd[idx].kp = 60.0f;
        final_cmd.motor_cmd[idx].kd = 1.5f;
        final_cmd.motor_cmd[idx].tau = 0.f;
      }
      cmd_pub_->publish(final_cmd);
```

The added 13-line block is byte-for-byte the same as the trajectory-following pattern at `@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp:185-195`. The ramp's frame cadence (50 frames × 20 ms = 1 s), the linear fade of `kNotUsedJoint.q` from 1.0 to 0.0, and the surrounding hand-close logic are all unchanged.

## Acceptance criteria — all PASS

| # | Check | Result |
|---|-------|--------|
| 1 | `colcon build --packages-select unitree_g1_dex3_stack` exit 0 | PASS (24.5 s) |
| 2 | `grep -c "Plan 01-07: hold every joint"` joint_trajectory_executor.cpp | 1 ✓ |
| 3 | trajectory-following population block at line 185-195 unchanged | ✓ (verified by `sed -n '183,193p'`) |
| 4 | destructor q=0.0 fallback (line 128) preserved | ✓ |
| 5 | Plan 01-06 signal-handler scaffolding preserved | ✓ |
| 6 | `wc -l` joint_trajectory_executor.cpp | 287 (was 272) ✓ |

## Behavior contract after Plan 01-07

**End of normal trajectory:**
1. Last fully-populated `cmd_msg` puts the arm at the trajectory end-point with stiff servoing.
2. `Hand closed after trajectory execution` INFO logs.
3. `Trajectory execution complete, returning to default pose.` INFO logs.
4. Ramp begins. Each of 51 frames publishes:
   - `kNotUsedJoint.q` linearly fading from 1.0 to 0.0
   - All other motors: `q = latest_joint_positions_[i]` (actual pose), `kp = 60`, `kd = 1.5`
5. arm_sdk holds the arm stiffly at the current pose during the fade. Body controller progressively gains influence as `kNotUsedJoint.q` decreases.
6. By frame 50, `kNotUsedJoint.q = 0.0`; body controller has full authority and starts its own smooth return to standing.
7. `trajectoryCallback` returns. **No jerk anywhere along this path.**

**Idle Ctrl+C:** unchanged from Plan 01-06 — process exits in ~50 ms, no motion.

**Mid-trajectory Ctrl+C:** arm completes ≤1 more waypoint, falls into the now-fixed end-of-trajectory ramp, smooth release.

**SIGTERM / SIGKILL:** unchanged from Plan 01-06.

## Why this bug took three plans to fix

The compound-bug timeline:

1. **`83b34a5 project_init`** — trajectoryCallback's end-of-trajectory ramp ships with the default-constructed LowCmd. Bug present but small.
2. **Plan 01-04** adds `gracefulRelease()` with the same default-constructed-LowCmd pattern. From idle state (where the body controller is already in charge after the trajectory-end ramp), `gracefulRelease()`'s step-0 frame slams `kNotUsedJoint.q` from 0.0 back to 1.0 → **large** authority-steal jerk to q=0. User reports "两手向前伸,然后才缓慢回到 running mode". Both jerks happen but the gracefulRelease one (5x larger, more visible) dominates the perception.
3. **Plan 01-06** deletes `gracefulRelease()`. The large jerk is gone. The trajectory-end ramp's small jerk surfaces on its own. User reports "结束后它会突然抖动,然后回到原点".
4. **Plan 01-07 (this plan)** fixes the original ramp the same way the trajectory-following frames already get it right.

The lesson: a masking bug can hide a smaller pre-existing one indefinitely. Plan 01-06 was correct as written, but the verification cycle had to repeat to surface the bug it unmasked. This is now recorded as a Phase 1 pattern: **fix one bug at a time and re-verify on hardware between fixes**.

## Out of scope (NOT done in this plan)

- Refactoring the population block into a `publishHoldFrame(value)` helper (two callsites, small saving, behavioral risk).
- Watchdog: detecting "planner has died from outside" inside executor.
- Same hooks for `ik_fcl_ompl_planner` or `dex3_controller`.
- The pre-existing `planner.launch.py` empty-list parameter bug.
- Any change to the trajectory-following loop or hand open/close logic.

## Deviations from Plan

None. Plan as written, executed as written. Single edit applied successfully on first attempt.

## Issues Encountered

None during implementation. The diagnostic step (re-reading the ramp code with the right hypothesis after Plan 01-06's user feedback) was the only non-trivial part.

## Next Steps

- **User action required:** restart `joint_trajectory_executor` (current PID is the Plan 01-06 binary). Then re-run the trajectory test:
  - Send right-side goal_pose, let arm move to completion.
  - Observe end of trajectory: **no perceptible jerk**. Arm holds final position briefly while master switch fades, then body controller smoothly resumes standing.
  - Re-confirm idle Ctrl+C still PASS (no motion).
  - Re-confirm mid-trajectory Ctrl+C still PASS (arm breaks at next waypoint, smooth release).
- After user-confirmed PASS, Phase 1 can be archived.

---
*Phase: 01-right-arm-only-planner*
*Completed: 2026-04-29*
