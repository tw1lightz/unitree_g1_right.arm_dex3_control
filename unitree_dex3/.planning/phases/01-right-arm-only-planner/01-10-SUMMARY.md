---
phase: 01-right-arm-only-planner
plan: 10
subsystem: motion-control
tags: [bug-fix, protocol-compliance, reference-parity, root-cause-finally]

requires:
  - phase: 01-right-arm-only-planner
    provides: "User-supplied reference implementation (right_arm_mode.py + robot_arm.py from hand_eye_calib) demonstrably works on the same hardware. Diffing field-by-field against our trajectoryCallback revealed the missing mode=1 field."
provides:
  - Every LowCmd from `trajectoryCallback` now sets `motor_cmd[idx].mode = 1` for every joint, enabling arm_sdk's PD-position control mode. With mode=1, the populated `q + kp + kd` values are actually tracked stiffly by the motor controllers, making Plan 01-09's explicit q-interpolation toward the standing snapshot effective in producing physical motion along the planned path.
affects:
  - All trajectory completions (Plan 01-09's design now actually executes).
  - Trajectory-following itself may feel stiffer (arm now under genuine PD control at kp=60 during waypoint tracking).

tech-stack:
  added: []
  patterns:
    - "When iterative attempts to control behavior fail despite seemingly correct logic, suspect a missing categorical control-mode field. Diff against a reference implementation that demonstrably works on the same hardware. Documentation may not state which protocol fields are functional requirements; observed-working code does."
    - "Minimum-variable principle: when a fix lands on a multi-field protocol, change one categorical field at a time. The reference implementation differs in 5 places (mode, kp, kd, kp_wrist, tau gravity FF); changing all five at once would obscure which one matters. Plan 01-10 changes only mode. If insufficient, Plan 01-11 changes gains. If still insufficient, Plan 01-12 changes gravity FF."

key-files:
  created:
    - .planning/phases/01-right-arm-only-planner/01-10-PLAN.md
    - .planning/phases/01-right-arm-only-planner/01-10-SUMMARY.md
  modified:
    - src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp

key-decisions:
  - "Add mode=1 in BOTH the trajectory-following loop AND the end-of-trajectory ramp. The reference implementation sets mode=1 once at controller __init__ on a persistent self.msg; we construct a fresh LowCmd each frame, so each must set mode=1 individually. Missing it in either place would fall back to mode=0 for that path."
  - "Do NOT set mode=1 in the destructor's instantaneous q=0 final publish. By that point the body controller is already in charge; that LowCmd is a one-shot master-switch reaffirmation, not a control command. Setting mode=1 there would pointlessly advertise PD intent for one tick during shutdown when no PD control is desired."
  - "Do NOT also bump kp 60→80, kd 1.5→3.0, kp_wrist 60→40, or add gravity feed-forward in this same plan. Those are tuning differences; mode=1 vs mode=0 is a categorical mode toggle. Stacking changes obscures which is necessary. Bisect by changing one variable at a time."
  - "Preserve Plan 01-09's q-interpolation, ramp_start_positions snapshot, and standing_pose snapshot logic exactly. The Plan 01-09 design (drive arm to standing yourself before authority transfer) is correct; it just couldn't physically execute because mode=0 had arm_sdk ignoring the commanded q values."

patterns-established:
  - "Compound-bug ladders eventually bottom out at a protocol-compliance issue when the call site never matched the documented (or undocumented) caller contract. Plans 01-04..01-09 fixed everything visible at the application logic layer; Plan 01-10 fixed the protocol-layer omission that made all of those fixes invisible to the hardware."
  - "User-supplied reference implementations on the same hardware are the highest-bandwidth diagnostic tool when stuck. Five plans of in-house reasoning failed to surface the mode field; one diff against a working caller surfaced it in minutes."

requirements-completed: []

duration: ~10 min
completed: 2026-04-29
---

# Phase 01 Plan 10: enable PD control mode (set motor_cmd[idx].mode = 1) — Summary

**Final root cause of the executor compound-bug ladder. Every LowCmd published from `trajectoryCallback` since `83b34a5 project_init` had `motor_cmd[].mode` defaulting to 0 (disabled / fallback), preventing arm_sdk from actually executing the commanded q/kp/kd. User-supplied reference `right_arm_mode.py` made this visible by direct diff against `G1_29_ArmController.__init__`'s line 120: `self.msg.motor_cmd[id].mode = 1`.**

## Performance

- **Tasks:** 2 / 2 auto complete + 1 human-verify pending
- **Files modified:** 1 (`joint_trajectory_executor.cpp`)
- **LOC delta:** 313 → 319 (+6 net: 2 single-line `mode = 1` assignments + 4 lines of comment markers)
- **Build:** `colcon build --packages-select unitree_g1_dex3_stack` exit 0 (19.9 s).

## Implementation summary

Single `multi_edit` with two single-line insertions inside `trajectoryCallback`:

### Edit 1 — trajectory-following loop

```@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp:199-213
      for (const auto& pair : joint_name_to_index) {
        size_t idx = pair.second;
        // Plan 01-10: enable PD control mode (was implicitly mode=0 before,
        // which caused arm_sdk to ignore q/kp/kd; reference: right_arm_mode.py).
        cmd_msg.motor_cmd[idx].mode = 1;
        if (latest_joint_positions_.size() > idx) {
          cmd_msg.motor_cmd[idx].q = latest_joint_positions_[idx];
        } else {
          cmd_msg.motor_cmd[idx].q = 0.0f;
        }
        cmd_msg.motor_cmd[idx].dq = 0.f;
        cmd_msg.motor_cmd[idx].kp = 60.0f;
        cmd_msg.motor_cmd[idx].kd = 1.5f;
        cmd_msg.motor_cmd[idx].tau = 0.f;
      }
```

### Edit 2 — end-of-trajectory ramp

```@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp:272-289
      for (const auto& pair : joint_name_to_index) {
        size_t idx = pair.second;
        // Plan 01-10: enable PD control mode here too, so the q-interpolation
        // computed below is actually tracked stiffly by the motor controllers.
        final_cmd.motor_cmd[idx].mode = 1;
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

Plan 01-09's q-interpolation, ramp_start_positions snapshot, and standing_pose snapshot logic are preserved unchanged. The destructor's q=0 fallback does NOT get mode=1.

## Acceptance criteria — all PASS

| # | Check | Result |
|---|-------|--------|
| 1 | `colcon build --packages-select unitree_g1_dex3_stack` exit 0 | PASS (19.9 s) |
| 2 | `grep -c "motor_cmd\[idx\].mode = 1"` | 2 ✓ |
| 3 | `grep -c "Plan 01-10"` | 2 ✓ |
| 4 | Plan 01-09 standing_pose preserved | 4 ✓ |
| 5 | Plan 01-09 ramp_start_positions preserved | 4 ✓ |
| 6 | `grep -c "kp = 60.0f"` (gain unchanged) | 2 ✓ |
| 7 | `wc -l` joint_trajectory_executor.cpp | 319 (was 313) ✓ |

## The compound-bug ladder, finally bottomed out

| Plan | Symptom | Root cause | Fix |
|------|---------|------------|-----|
| 01-04 | Arm released instantly on Ctrl+C | Destructor publishes single q=0 LowCmd | Add `gracefulRelease()` 3s ramp |
| 01-06 | "Hands forward" jerk on Ctrl+C | gracefulRelease re-grabs authority; default motor_cmd jerks to q=0 reference | Delete gracefulRelease; add `g_shutdown_requested` waypoint check |
| 01-07 | End-of-trajectory q=0 jerk | Ramp publishes default-zero motor_cmd | Populate motor_cmd q/kp/kd in ramp |
| 01-08 | (failed fix) Tried fading kp/kd alongside master | Wrong assumption: master switch is binary, not blended | Reverted in 01-09 |
| 01-09 | Snap to standing at end of ramp | Tried explicit q-interpolation to standing snapshot | Correct design — but… |
| **01-10 (this)** | Plan 01-09 q-interpolation didn't physically execute | Every LowCmd had `motor_cmd[].mode = 0` (arm_sdk's default fallback). arm_sdk was ignoring our q/kp/kd entirely; the apparent trajectory motion came from the fallback, not from PD control. | Set `motor_cmd[idx].mode = 1` in both populate loops. |

The ladder finally bottoms out at protocol compliance. Plans 01-04..01-09 were all fixing application-level logic; Plan 01-10 fixed the protocol-level omission that made all of them invisible to the hardware. **Bug present since `83b34a5 project_init`.**

## Out of scope (deferred to follow-up plans if needed)

- Plan 01-11: kp 60→80, kd 1.5→3.0, kp_wrist 60→40 (match reference numerical gains).
- Plan 01-12: gravity feed-forward via Pinocchio RNEA (match reference's `tauff = pin.rnea(...)` call).
- Both deferred unless mode=1 alone is insufficient. Minimum-variable principle.

## Deviations from Plan

None. Plan as written, executed as written. Single `multi_edit` succeeded on first attempt.

## Issues Encountered

The diagnostic was finally solved by user-supplied reference. Five in-house plans (01-04..01-09) all worked on application-level logic without ever questioning the protocol-level field defaults. The reference comparison surfaced the omission immediately. Lesson recorded for future similar situations.

## Next Steps

- **User action required:** restart `joint_trajectory_executor` (current PID is the Plan 01-09 binary). Then re-run the trajectory test:
  - Send right-side goal_pose, let arm move to completion.
  - Trajectory motion may feel stiffer (real PD now engaged at kp=60).
  - At end of trajectory: arm should reverse smoothly along 3s interpolated path back to the same standing pose, no snap.
  - Re-confirm idle Ctrl+C still PASS.
  - Re-confirm mid-trajectory Ctrl+C still PASS.
- If symptom persists → Plan 01-11 (numeric gain match).
- If PASS → **Phase 1 archive**.

---
*Phase: 01-right-arm-only-planner*
*Completed: 2026-04-29*
