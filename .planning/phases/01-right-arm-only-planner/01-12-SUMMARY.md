# Plan 01-12 SUMMARY — Fix Stale `latest_joint_positions_` + Follow-up 01-12b Master Continuity

## Status: ✅ Verified by user — arm returns smoothly to standing after trajectory

## Root Cause (01-12)

Single-threaded `rclcpp::executors::SingleThreadedExecutor` only runs one callback at a time. `trajectoryCallback` is a long-running callback (trajectory loop + hold loop + ramp loop = several seconds). During that entire window, `lowstateCallback` never fires. Therefore `latest_joint_positions_` stays frozen at the standing pose from callback entry.

Using this stale value:
- In the **hold loop** → commanded the arm back to standing for 1 s → snap.
- In the **ramp start** → ramp started from standing instead of trajectory endpoint → no interpolation, just a fade of master with arm already at standing.

## Root Cause (01-12b follow-up)

After 01-12 fixed the stale-position bug, the user reported: "到达点位后冲击一段距离，然后缓慢回归" (overshoot then slow return).

| Stage | master (kNotUsedJoint.q) |
|-------|--------------------------|
| Trajectory loop | 0.5 |
| Hold loop | 0.5 |
| **Ramp step 0 (before fix)** | **1.0** ← discontinuity! |
| Ramp step 750 | 0.0 |

The 0.5→1.0 jump at the hold-to-ramp transition instantly doubled planner authority, causing arm_sdk to overshoot the target before the fade brought it back.

## Edits

### 01-12 (multi-line)

**`joint_trajectory_executor.cpp`:**

1. Added `trajectory_endpoint` vector extraction from `msg->points.back()` right after trajectory loop (lines 236-260):
   - Baseline from `standing_pose` for non-trajectory joints.
   - Overwrite with `last_point.positions[j]` (URDF limit clamped) for trajectory joints.

2. Hold loop `q` source: `latest_joint_positions_[idx]` → `trajectory_endpoint[idx]` (line 281-282).

3. Ramp start: `ramp_start_positions = latest_joint_positions_` → `ramp_start_positions = trajectory_endpoint` (line 300).

### 01-12b (single-line)

**`joint_trajectory_executor.cpp` line 312:**

```cpp
// Before:
double value = (1.0 - t) * 1.0 + t * 0.0;
// After:
double value = (1.0 - t) * 0.5 + t * 0.0; // Linear interpolation from 0.5 (matching trajectory/hold) to 0.0
```

Ramp now interpolates master 0.5→0.0, continuous with trajectory and hold phases.

## Build & Static ACs

| AC | Expected | Actual | Status |
|----|----------|--------|--------|
| `grep -c "trajectory_endpoint"` | ≥ 3 | 4 | ✅ |
| `grep "ramp_start_positions = trajectory_endpoint"` | 1 | 1 | ✅ |
| `grep "0.5 + t * 0.0"` | 1 | 1 | ✅ |
| `colcon build` exit 0 | 0 | 0 | ✅ |

## Behavior Contract (verified)

1. Trajectory executes normally.
2. After last waypoint: arm **stays stiff at end-point** for ~1 s (hand closes).
3. Ramp: arm smoothly reverses along q-interpolated path from trajectory endpoint to standing over ~3 s.
4. Master fades 0.5→0.0 continuously.
5. **No snap. No overshoot.** Firmware takes over with arm already at standing.

## Human Verification Steps (as performed by user)

```bash
# Launch arm_sdk + planner + executor
ros2 launch unitree_g1_dex3_stack planner.launch.py

# Send test trajectory and observe:
# - Arm reaches target
# - Stays at target for ~1 s
# - Smoothly returns to standing over ~3 s
# - No jerk or bounce at any point
```

**PASS criteria:** smooth motion from trajectory end-point back to standing. ✅

## Compound-Bug Ladder (complete)

| Plan | Bug | Fix |
|------|-----|-----|
| 01-04 | No graceful shutdown | Custom SIGINT + gracefulRelease ramp |
| 01-06 | gracefulRelease re-grabs authority | Delete gracefulRelease; shutdown check in callback |
| 01-07 | Ramp q=0 → snap to zero | Populate q from latest positions |
| 01-08 | Body controller override | kp/kd fade (reverted in 01-09) |
| 01-09 | No explicit standing path | q-interpolation from endpoint to standing |
| 01-10 | PD mode not enabled | motor_cmd[idx].mode = 1 |
| 01-11 | 2 s publish gap + 50 Hz ramp | 250 Hz hold loop + 750-step ramp |
| **01-12** | **latest_joint_positions_ stale** | **Extract trajectory_endpoint from waypoint** |
| **01-12b** | **master 0.5→1.0 jump** | **Ramp 0.5→0.0 continuous fade** |

## Lessons

1. **Single-threaded executor blocks all other callbacks** — any long-running callback must not depend on live subscriber data.
2. **Publish gaps are silent failures** — firmware has its own control loop and will fill any silence.
3. **Reference implementation frequency is part of the contract** — 250 Hz is the minimum, not a target.
4. **Signal continuity matters** — master, q, mode, kp, kd: any discontinuity is a mechanical impulse.
5. **Nine iterations to fix one symptom** — each iteration exposed the next hidden bug in the chain. Compound bugs require systematic isolation.

---

*Corresponding commits:*
- `6b63463` fix(01-12): use trajectory endpoint, not stale latest_joint_positions_
- `5a29e0f` fix(01-12b): ramp master 0.5->0.0 to match trajectory/hold, eliminate overshoot

*User verification: 2025-04-30 — "终于！成功了！"*
