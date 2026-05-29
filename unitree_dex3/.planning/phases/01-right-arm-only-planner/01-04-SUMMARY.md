---
phase: 01-right-arm-only-planner
plan: 04
subsystem: motion-control
tags: [safety, shutdown, ros2, rclcpp, signal-handling, arm-sdk]

requires:
  - phase: 01-right-arm-only-planner
    provides: "Right-arm-only planner with verified initialization (Plan 01-03)."
provides:
  - On SIGINT/SIGTERM, joint_trajectory_executor performs a 3-second linear ramp of LowCmd.motor_cmd[kNotUsedJoint].q from 1.0 down to 0.0 BEFORE rclcpp::shutdown(), letting the robot body controller smoothly resume the standing pose without mechanical jerk.
  - rclcpp default signal handler is replaced with a custom atomic-flag handler so the polling loop in main() can exit cleanly while cmd_pub_ is still valid.
  - Destructor's instantaneous q=0.0 publish is preserved as a last-resort fallback for SIGKILL / std::terminate paths.
affects:
  - Phase 1 verification — operators can now safely Ctrl+C the executor mid-trajectory without arm jerk.
  - Future phases that may reuse this graceful-release pattern (e.g., e-stop, fault recovery, multi-arm coordination).

tech-stack:
  added: []
  patterns:
    - "ROS 2 Foxy graceful-shutdown pattern: rclcpp::uninstall_signal_handlers() + std::signal + SingleThreadedExecutor::spin_some() polling loop. Lets node finish a synchronous cleanup task (here: 3-second LowCmd ramp) BEFORE rclcpp::shutdown() invalidates the publisher."

key-files:
  created:
    - .planning/phases/01-right-arm-only-planner/01-04-PLAN.md
    - .planning/phases/01-right-arm-only-planner/01-04-SUMMARY.md
  modified:
    - src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp

key-decisions:
  - "Disable rclcpp's built-in SIGINT/SIGTERM handler in main() right after rclcpp::init(). Foxy uses rclcpp::uninstall_signal_handlers() (verified at /opt/ros/foxy/include/rclcpp/utilities.hpp:96); the SignalHandlerOptions parameter overload of rclcpp::init() only exists from Galactic onward and is NOT available here."
  - "Use a process-level std::atomic<bool> g_shutdown_requested flag set by a minimal signal handler. Signal handlers can only safely call signal-safe primitives, so the actual cleanup runs in main()."
  - "Replace rclcpp::spin(node) with a SingleThreadedExecutor spin_some(50ms) polling loop checking the flag. 50 ms gives ~20 Hz responsiveness to Ctrl+C, which is well under perceptual latency."
  - "User-selected ramp duration: 3 seconds × 150 steps = 20 ms per step (matches the per-trajectory-end ramp's 20 ms cadence; only the total step count is increased from 50 to 150)."
  - "Keep the existing destructor's instantaneous q=0.0 publish AS-IS as a last-resort fallback for SIGKILL / std::terminate / forced-exit paths where our signal handler never runs. Deleting it would regress safety in those edge cases."

patterns-established:
  - "Two-phase shutdown for nodes with hardware-control authority: (1) signal handler sets atomic flag, (2) main() polling loop sees flag and exits cleanly, (3) cleanup method publishes a smooth release ramp while publishers are still valid, (4) rclcpp::shutdown() runs last."
  - "ROS 2 Foxy compatibility note: SignalHandlerOptions DOES NOT exist in Foxy; use uninstall_signal_handlers() instead."

requirements-completed: []

duration: ~30 min
completed: 2026-04-29
---

# Phase 01 Plan 04: Graceful Arm Release on Executor Shutdown — Summary

> **⚠️ SUPERSEDED IN PART by Plan 01-06.** The `gracefulRelease()` ramp documented below caused the arm to jerk to its `q = 0` reference pose ("hands forward") on Ctrl+C, because by the time the ramp ran, the trajectoryCallback's own end-of-trajectory ramp had already released authority — `gracefulRelease()`'s first frame (`kNotUsedJoint.q = 1.0`) re-grabbed it. **Plan 01-06 deleted `gracefulRelease()`** and added a `g_shutdown_requested` check to `trajectoryCallback`'s waypoint loop instead, so the existing trajectory-end ramp acts as the single graceful-release path. The signal-handler scaffolding (atomic flag, `uninstall_signal_handlers`, polling loop in `main()`) introduced by Plan 01-04 is preserved; only the separate ramp method was removed. See `@/home/unitree/Desktop/unitree_dex3/.planning/phases/01-right-arm-only-planner/01-06-SUMMARY.md` for the full root-cause analysis and the corrected behavior contract.

**Hot-fix on top of Plan 01-03 to address a safety concern raised mid-verification: when the executor terminal is killed mid-trajectory, the arm should smoothly transfer control back to the robot body over 3 seconds instead of being released instantaneously.**

## Performance

- **Tasks:** 3 / 3 auto complete + 1 human-verify checkpoint pending
- **Files modified:** 1 (`joint_trajectory_executor.cpp`)
- **LOC delta:** 230 → 286 lines (+56 lines: imports, atomic + signal handler, gracefulRelease() method, polling loop, ramp comment)
- **Build:** `colcon build --packages-select unitree_g1_dex3_stack` exit 0 (PCL/IO stderr WARNs are pre-existing image issues).

## Implementation summary

Three concrete edits, all in `@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp`:

### 1. Imports + signal handler scaffolding (top of file)

```@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp:20-33
#include <atomic>
#include <csignal>

#include <g1_dex3_joint_defs.hpp>

using namespace std::chrono_literals;

// Set by SIGINT/SIGTERM handler so main() can perform a graceful arm-control
// release (3-second ramp of kNotUsedJoint.q from 1.0 -> 0.0) BEFORE
// rclcpp::shutdown() invalidates the publisher. See Plan 01-04.
static std::atomic<bool> g_shutdown_requested{false};
static void executor_signal_handler(int /*sig*/) {
  g_shutdown_requested.store(true);
}
```

### 2. `gracefulRelease()` public method (after destructor)

3-second × 150-step linear ramp with `rclcpp::ok()` short-circuit at every step (so a second Ctrl+C aborts the ramp cleanly):

```@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp:130-149
  void gracefulRelease() {
    if (!rclcpp::ok()) return;
    RCLCPP_INFO(this->get_logger(),
      "Graceful release: smoothly transferring arm control to robot body (3s).");
    const double duration_s = 3.0;
    const int steps = 150;
    auto sleep_ns = std::chrono::nanoseconds(
      static_cast<int64_t>((duration_s / steps) * 1e9));
    for (int step = 0; step <= steps && rclcpp::ok(); ++step) {
      const double t = static_cast<double>(step) / steps;
      const double value = (1.0 - t) * 1.0;  // linear 1.0 -> 0.0
      unitree_hg::msg::LowCmd release_cmd;
      release_cmd.motor_cmd[JointIndex::kNotUsedJoint].q =
        static_cast<float>(value);
      cmd_pub_->publish(release_cmd);
      rclcpp::sleep_for(sleep_ns);
    }
    RCLCPP_INFO(this->get_logger(),
      "Graceful release complete; arm control returned to robot body.");
  }
```

### 3. `main()` rewritten

```@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp:264-285
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Replace rclcpp's default SIGINT handler so we can perform a graceful
  // arm-control release before the publisher is torn down. See Plan 01-04.
  rclcpp::uninstall_signal_handlers();
  std::signal(SIGINT, executor_signal_handler);
  std::signal(SIGTERM, executor_signal_handler);

  auto node = std::make_shared<JointTrajectoryExecutor>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  while (rclcpp::ok() && !g_shutdown_requested.load()) {
    exec.spin_some(std::chrono::milliseconds(50));
  }

  // SIGINT/SIGTERM received (or rclcpp::ok went false). Smoothly release
  // arm authority while the publisher is still valid.
  node->gracefulRelease();
  rclcpp::shutdown();
  return 0;
}
```

### 4. Destructor preserved with updated comment

The pre-existing instant-release path stays as a safety net:

```@/home/unitree/Desktop/unitree_dex3/src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp:114-124
  ~JointTrajectoryExecutor() override {
    RCLCPP_INFO(this->get_logger(), "Shutting down Joint Trajectory Executor Node");

    // Last-resort: instant release. Normal SIGINT/SIGTERM path goes through
    // gracefulRelease() in main() before this destructor runs, so this only
    // fires when the process exits without our signal handler getting to run
    // (e.g. SIGKILL, std::terminate from another thread).
    unitree_hg::msg::LowCmd final_cmd;
    final_cmd.motor_cmd[JointIndex::kNotUsedJoint].q = 0.0f;
    cmd_pub_->publish(final_cmd);
  }
```

## Acceptance criteria — all PASS

| # | Check | Result |
|---|-------|--------|
| 1 | `colcon build` exit 0 | PASS |
| 2 | Required tokens count >= 4 (`gracefulRelease` / `g_shutdown_requested` / `uninstall_signal_handlers` / `SignalHandlerOptions::None`) | 7 matches across lines 30, 32, 118, 130, 269, 276, 282 |
| 3 | Ramp reachable from `main()` | `node->gracefulRelease()` at line 282 |
| 4 | Destructor q=0.0 fallback preserved | line 122 |
| 5 | "Graceful release" INFO logged | lines 133, 148 |

## Foxy API note

Plan 01-04 originally drafted using `rclcpp::SignalHandlerOptions::None` as a parameter to `rclcpp::init(...)`. After verification at `/opt/ros/foxy/include/rclcpp/utilities.hpp`, that overload **does not exist in Foxy** (it was added in Galactic). Foxy provides:

```@/opt/ros/foxy/include/rclcpp/utilities.hpp:96
uninstall_signal_handlers();
```

which has the same effect when called immediately after `rclcpp::init()`. Plan 01-04 PLAN.md was updated in-place before implementation; no rework needed.

## Behavior contract

When user presses Ctrl+C (or sends SIGTERM) on the executor terminal:

1. `executor_signal_handler` sets `g_shutdown_requested = true` (signal-safe atomic store).
2. The `SingleThreadedExecutor` polling loop in `main()` reads the flag at its next `spin_some(50ms)` tick (worst case 50 ms latency).
3. Loop exits, `node->gracefulRelease()` runs:
   - Logs `"Graceful release: smoothly transferring arm control to robot body (3s)."`.
   - Publishes 151 LowCmd messages (steps 0..150) with `kNotUsedJoint.q` linearly decreasing from 1.0 to 0.0 over 3 seconds.
   - Inner loop checks `rclcpp::ok()` at every step; a second Ctrl+C will abort the ramp early.
   - Logs `"Graceful release complete; arm control returned to robot body."`.
4. `rclcpp::shutdown()` finalizes the context.
5. Destructor runs (from shared_ptr release); fallback `q=0.0` is published but is now redundant — no jerk because the ramp already finished at 0.0.

If the process is killed via SIGKILL or another non-trappable signal, the `gracefulRelease()` path is bypassed entirely, and only the destructor's instantaneous publish runs (or, if the destructor itself doesn't run because of forced exit, the robot's arm_sdk timeout will release control automatically). This degraded path is acceptable per Plan 01-04 §"Out of scope".

## Out of scope (NOT done in this plan)

- Watchdog detecting "planner has died from outside" inside executor (option 3 in user prompt was not selected).
- Same graceful-release pattern for `ik_fcl_ompl_planner.cpp` — planner is upstream of executor; killing the planner stops trajectory generation but doesn't strand the arm because the executor independently handles its own LowCmd stream.
- Same pattern for `dex3_controller.cpp` (hand controller; out of scope).
- Fix for the pre-existing `planner.launch.py` empty-list parameter bug (still a known Phase-1 follow-up).

## Deviations from Plan

### 1. [Foxy API correction, pre-implementation] — `SignalHandlerOptions` -> `uninstall_signal_handlers()`

- **Found during:** pre-implementation API verification.
- **Issue:** PLAN.md originally specified `rclcpp::SignalHandlerOptions::None` as a parameter to `rclcpp::init()`, but that overload doesn't exist in ROS 2 Foxy.
- **Fix:** PLAN.md updated in-place to specify `rclcpp::uninstall_signal_handlers()` after `rclcpp::init()` (same end result, Foxy-compatible).
- **Verification:** Build passed; tokens present.
- **Approval:** N/A (mechanical API correction, not behavioral change).

**Total deviations:** 1 (mechanical Foxy API correction).

## Issues Encountered

None during implementation. All ACs passed on first build.

## Next Steps

- Plan 01-04 awaits its own `checkpoint:human-verify` (operator-driven, on the live robot):
  - Start `joint_trajectory_executor`, send a goal_pose, let executor begin moving the arm.
  - Mid-motion, press Ctrl+C in the executor terminal.
  - Expected: executor logs `"Graceful release: smoothly transferring arm control to robot body (3s)."`, arm slows over ~3 s, robot body resumes standing pose without perceptible jerk.
  - Repeat with `kill -SIGTERM <pid>` mid-motion → same behavior.
  - `kill -9 <pid>` is acknowledged as degraded — fallback (instant release) runs; not in scope to fix.
- After Plan 01-04 verification completes, the Phase 1 `checkpoint:human-verify` (originally for Plan 01-03 + the planner behavioral checks) can be rerun with the safety net in place.
- Then phase verification → Phase 1 archive → Phase 2 plan.

---
*Phase: 01-right-arm-only-planner*
*Completed: 2026-04-29*
