# Plan 01-11 SUMMARY — Close Post-Trajectory Publish Gap + 250 Hz Ramp

## Status: ✅ Build-verified — awaiting human verification

## Root Cause (diagnosed this plan)

After Plans 01-04 through 01-10 the arm still snapped back to standing at
trajectory completion. The payload (mode=1, kp/kd, q interpolation) was
correct, but the **publish timeline** had a critical gap:

```
 last waypoint publish
       ↓
       |-- sleep_for(1s) --|-- hand_close publish --|-- sleep_for(1s) --|-- ramp begins -->
       |<----- 2 s silent: firmware drags arm toward standing on its own ----->|
```

During the 2 s gap, firmware received no master/q from the executor and
started overriding with its own standing-pose commands. By the time the
ramp started the arm had already snapped. Additionally, the ramp itself
ran at 50 Hz (150 steps / 3 s), whereas the reference `smooth_exit`
publishes at 250 Hz (500 steps / 2 s).

## Edits (2 coordinated changes in `joint_trajectory_executor.cpp`)

### Edit A — Hold-publish loop replaces two `sleep_for(1s)` calls

| Metric | Before | After |
|--------|--------|-------|
| Post-trajectory silent gap | 2.0 s | 0 s |
| Hand-close timing | Sequential with explicit 1 s wait | Issued once at frame 0; runs in parallel |
| Hold duration | N/A | 1.0 s (250 steps × 4 ms) |
| Hold publish rate | 0 Hz | 250 Hz |
| Hold payload | N/A | master=0.5, mode=1, kp=60, kd=1.5, q=latest |

### Edit B — Ramp frequency bump

| Metric | Before | After |
|--------|--------|-------|
| `interp_steps` | 150 | 750 |
| `interp_duration` | 3.0 s | 3.0 s (unchanged) |
| Ramp publish rate | 50 Hz | 250 Hz |
| Per-frame sleep | 20 ms | 4 ms |

## Static Acceptance Criteria

| AC | Expected | Actual | Status |
|----|----------|--------|--------|
| Post-trajectory code-only `sleep_for(1s)` calls | 0 | 0 | ✅ |
| `grep -c 'Plan 01-11'` | ≥2 | 2 | ✅ |
| `grep -c 'interp_steps = 750'` | 1 | 1 | ✅ |
| `grep -c 'interp_steps = 150'` | 0 | 0 | ✅ |
| `grep -c 'standing_pose'` (Plan 01-09 preserved) | ≥4 | 4 | ✅ |
| `grep -c 'motor_cmd[idx].mode = 1'` | 3 (traj+hold+ramp) | 3 | ✅ |
| `grep -c 'hold_steps = 250'` | 1 | 1 | ✅ |
| `colcon build` exit 0 | 0 | 0 | ✅ |

## Behavior Contract (expected, pending human verification)

1. Trajectory loop runs as before.
2. Log: `Trajectory point N executed; holding stiff at end-point while hand closes.`
3. Hand close issued once; executor publishes 250 Hz hold for 1 s (arm stiff at end-point).
4. Log: `Hand closed; trajectory execution complete, returning to default pose.`
5. 3 s × 250 Hz ramp: arm drives smoothly from end-point to standing snapshot; master fades 1.0→0.0.
6. Total continuous-publish window after trajectory: **4 s** with zero gaps.
7. Post-ramp: master hands off to firmware with arm already at standing — no snap.

## Human Verification Steps

```bash
# Terminal 1: launch arm_sdk + planner + executor
ros2 launch unitree_g1_dex3_stack planner.launch.py

# Terminal 2: send a test trajectory
ros2 topic pub --once /joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{ ... test trajectory ... }"

# Observe:
# - Arm executes trajectory normally
# - After last waypoint: arm stays stiff at end-point for ~1 s (hand closes)
# - Then arm reverses smoothly back to standing over ~3 s
# - NO abrupt snap at any point
# - Logs show the two Plan 01-11 info messages in sequence
```

**PASS criteria:** smooth, continuous motion from trajectory end-point back to standing.
**FAIL criteria:** any visible jerk/snap during the 4 s post-trajectory window → escalate to Plan 01-12 (gain tuning / reference-parity kp=80 kd=3).

## Compound-Bug Ladder (complete)

| Plan | Bug | Fix |
|------|-----|-----|
| 01-04 | No graceful shutdown | Custom SIGINT handler + gracefulRelease ramp |
| 01-06 | gracefulRelease re-grabs authority → jerk | Delete gracefulRelease; shutdown check in callback |
| 01-07 | Ramp q=0 → arm snaps to zero | Populate q from latest_joint_positions_ |
| 01-08 | Body controller overrides faded kp/kd | Extended ramp to 3 s, faded gains (reverted) |
| 01-09 | No explicit path to standing | q-interpolation from end-point to standing snapshot |
| 01-10 | PD mode not enabled | Set motor_cmd[idx].mode = 1 |
| **01-11** | **2 s publish gap + 50 Hz ramp** | **Hold-publish loop + 250 Hz ramp** |
