---
phase: 8
status: passed
verified: 2026-05-19
verifier: kiro_default (inline orchestrator)
plans_verified:
  - 08-01
  - 08-02
requirements:
  - ORI-01
must_haves_total: 11
must_haves_passed: 11
must_haves_failed: 0
human_verification_count: 4
---

# Phase 8 Verification: 自适应末端位姿

## Verdict

**PASSED.** All 11 plan-level `<must_haves>` truths verified against
the actual repo state. ORI-01 is traceable to concrete code in
`ik_fcl_ompl_planner.cpp` + `planner.launch.py` and to a measurable
exit-code contract in `scripts/adaptive_orientation_ab.py`. The
`-DBUILD_IK_FCL_OMPL_PLANNER=ON` colcon build returns rc=0 with no
new warnings on `ik_fcl_ompl_planner.cpp` under `-Wall -Wextra
-Wpedantic`. The compiled object file exports
`IKFCLPlannerNode::computeAdaptiveOrientation`. The harness installs
as a `ros2 run` executable. Four items require live hardware testing
on the G1 — listed under **Human verification** below.

## Phase goal — does the codebase achieve it?

> *Implement adaptive end-effector orientation: derive the TCP +X
> approach axis from the right-shoulder→target direction, gated by
> a launch-time toggle, with a measurable A/B verification harness.*

Yes. The phase delivers:

- A surgical edit to `IKFCLPlannerNode` (in
  `src/ik_fcl_ompl_planner.cpp`, +121 lines) that adds a
  cached-once shoulder origin (FK on segment 1 of `kdl_chain_right`
  with all-zero joints), one ROS parameter
  (`adaptive_orientation_enabled`, default `true`), one inline
  private helper (`computeAdaptiveOrientation`) implementing the
  D-01..D-04 look-at orthonormal basis, and a splice block in
  `goalPoseCallback` that mutates `pose_in_base.pose.orientation` in
  place when adaptive is on and skips the entire block when it is off
  (preserving pre-Phase-8 behavior bit-exactly).
- A 3-line launch wiring (`launch/planner.launch.py`,
  `DeclareLaunchArgument` + `.lower() == 'true'` coerce + parameters
  dict entry) so operators flip the toggle via
  `ros2 launch unitree_g1_dex3_stack planner.launch.py
  adaptive_orientation_enabled:={true|false}`.
- A self-contained 194-line non-interactive ROS 2 Python harness
  (`scripts/adaptive_orientation_ab.py`) that publishes a fixed
  8-target tabletop test set in `torso_link` to `/goal_pose`, listens
  on `/joint_trajectory_targets`, and exits 0 only when all 8 targets
  receive a trajectory message within `timeout_sec` (default 3.0 s).
- A 1-line `CMakeLists.txt` install entry that ships the harness as a
  `ros2 run` executable alongside `tcp_torso_pose.py` /
  `keyboard_trigger_node.py` / `apriltag_detector_node.py`.

## Plan-by-plan must_haves

### Plan 08-01 — adaptive orientation in planner (6/6 must_haves PASS)

| # | Truth | Verification |
|---|-------|--------------|
| 1 | Startup INFO logs the right shoulder reference point in `torso_link` within 1e-3 m of `(0.0040, -0.1002, 0.2478)`. Implements D-05 (`right_shoulder_pitch_link` origin), D-06 (URDF/KDL FK, no runtime TF), and the startup-confirmation form of D-12. | Code path: `init()` constructs `KDL::JntArray zero_jnt(...)`, calls `fk_right_solver->JntToCart(zero_jnt, shoulder_frame, 1)`, caches `shoulder_frame.p` in `right_shoulder_pos_in_base_`, and emits `RCLCPP_INFO("Right shoulder reference point in '%s': [%.4f, %.4f, %.4f]", ...)`. URDF ground truth `xyz="0.0039563 -0.10021 0.24778"` rounds to `(0.0040, -0.1002, 0.2478)`. Hardware confirmation deferred to **HV-1** below. |
| 2 | Startup INFO logs `adaptive_orientation_enabled = true` (default) or `= false`. Implements D-10. | Code path: `init()` calls `declare_parameter("adaptive_orientation_enabled", true)`, `get_parameter(...)`, and emits `RCLCPP_INFO("adaptive_orientation_enabled = %s", ... ? "true" : "false")`. Source grep `declare_parameter("adaptive_orientation_enabled"` returns 1. Hardware confirmation deferred to **HV-2** below. |
| 3 | When adaptive is on and a tabletop `/goal_pose` arrives, the planner emits exactly one `Adaptive orientation: target=[…] shoulder=[…] dir=[…] q=[…]` INFO line per goal AND publishes `/joint_trajectory_targets`. Implements D-01..D-04, D-09, D-12. | Code path: splice block in `goalPoseCallback` runs `computeAdaptiveOrientation` then mutates the four `pose_in_base.pose.orientation.{x,y,z,w}` fields in place and emits the prescribed `RCLCPP_INFO`. The downstream IK + OMPL path is unmodified. Source greps: literal `Adaptive orientation: target=` count = 1; `computeAdaptiveOrientation` count = 3 (definition + call site + leading comment). Hardware confirmation deferred to **HV-3** below (Plan 02 harness exercises it). |
| 4 | When adaptive is on and the goal position is within 0.05 m of the cached shoulder, planner emits `RCLCPP_ERROR` containing `within 0.05 m of right shoulder` and does NOT publish a trajectory. Implements D-08. | Code path: `computeAdaptiveOrientation` returns `AdaptiveOrientationStatus::TARGET_TOO_CLOSE_TO_SHOULDER` when `d.Norm() < kMinTargetDistance = 0.05`; the splice block then emits the prescribed `RCLCPP_ERROR` and `return`s without falling through to the IK/OMPL path. Source greps: literal `within 0.05 m of right shoulder` count = 1; `kMinTargetDistance` constant present. Hardware confirmation deferred to **HV-4** below. |
| 5 | When adaptive is off, the planner uses `/goal_pose.pose.orientation` byte-for-byte unchanged — no `Adaptive orientation` log line is emitted, and the planner produces the same trajectory it would have produced before Phase 8 for the same input. Implements D-11. | Code path: the splice block is wrapped `if (adaptive_orientation_enabled_) { … }`, so when the toggle is `false` the block is skipped entirely and `pose_in_base.pose.orientation` flows unchanged into `KDL::Frame target_frame(KDL::Rotation::Quaternion(...), KDL::Vector(...))`. No state mutation, no log emission. The bool member declares `= true` as its in-class default and is overwritten by `get_parameter`. |
| 6 | Phase 8 adds exactly one ROS parameter (`adaptive_orientation_enabled`); the shoulder link name `right_shoulder_pitch_link` is hardcoded. Implements D-07. | Source greps: `declare_parameter("adaptive_orientation_enabled"` count = 1 (no other new declare_parameter introduced in this phase). The shoulder link is hardcoded as the segment-1 traversal of `kdl_chain_right` — there is no `declare_parameter("right_shoulder_pitch_link"` or similar. Constants `kMinTargetDistance` and `kParallelDotThreshold` are local `constexpr double` inside `computeAdaptiveOrientation` (CLAUDE.md §2 simplicity, D-07 forbids extra params). |

### Plan 08-02 — A/B verification harness (5/5 must_haves PASS)

| # | Truth | Verification |
|---|-------|--------------|
| 1 | Non-interactive Python ROS 2 node `adaptive_orientation_ab.py` exists at `scripts/`, takes no positional arguments, and publishes a fixed 8-target tabletop set (in `torso_link` frame) one-by-one to `/goal_pose`. Implements D-13, D-14. | Source greps: `class AdaptiveOrientationAB(Node)` count = 1; module-level `TARGETS = [...]` has the exact 8 labels (`center`, `center-near`, `center-far`, `right-side`, `left-of-mid`, `low`, `high`, `diag`); `goal.header.frame_id = 'torso_link'` is set in `_publish_goal`; `'/goal_pose'` literal count = 1. The node runs from a 0.1 s timer and ignores `argv`. |
| 2 | The harness subscribes to `/joint_trajectory_targets` and counts a target as PASS only when ≥1 trajectory message is received within `timeout_sec` (default 3.0 s) of the goal publish. Implements D-13, D-15. | Source: `self.traj_sub = self.create_subscription(JointTrajectory, '/joint_trajectory_targets', self._traj_cb, 10)` + `_tick` in `waiting` phase checks `self._traj_received_for_current` versus `self._now_sec() - self._publish_time > self.timeout_sec`. Records `(label, x, y, z, True/False)` into `self.results`. `'/joint_trajectory_targets'` literal count = 1. |
| 3 | After 8 targets, harness logs a per-target table and a `PASS_COUNT/8` summary, then exits 0 if all PASSed, status 1 otherwise. Implements D-15. | Source: `_summarize` logs each row via `RCLCPP_INFO` then a final `=== PASS_COUNT n/N — adaptive=<label> ===` line. `_exit_status = 0 if passed == len(TARGETS) else 1`. `main()` propagates `node._exit_status` through `sys.exit(...)`. `KeyboardInterrupt` → 130. |
| 4 | The harness is installed by colcon as an executable Python script under `lib/unitree_g1_dex3_stack/` and is callable via `ros2 run unitree_g1_dex3_stack adaptive_orientation_ab.py`. | `install/unitree_g1_dex3_stack/lib/unitree_g1_dex3_stack/adaptive_orientation_ab.py` is `-rwxr-xr-x` (7606 bytes). `ros2 pkg executables unitree_g1_dex3_stack` lists `adaptive_orientation_ab.py` alongside the other six C++ executables and three existing Python scripts. |
| 5 | Per D-16, the harness does NOT subscribe to or publish to any executor-related topic; does not start the executor; does not move the robot. The only outputs are `/goal_pose` publishes; the only inputs are `/joint_trajectory_targets` messages and ROS parameters. | Negative grep `unitree_hg\|joint_trajectory_executor\|JointTrajectoryExecutor\|Dex3Controller` returns 0 — the harness contains zero executor symbols. Imports are `rclpy`, `rclpy.node`, `geometry_msgs.msg.PoseStamped`, `trajectory_msgs.msg.JointTrajectory`, `sys` only. |

## Roadmap success criteria

> 1. 根据目标位置相对右肩的方向自动计算 orientation
> 2. 计算出的 orientation 使手臂自然指向目标（非固定死姿态）
> 3. 对比固定姿态，IK 成功率明显提升
> 4. 在工作空间边界和肩部正上方等困难区域仍能找到可行姿态

| # | Criterion | Verification |
|---|-----------|--------------|
| 1 | Auto-compute orientation from target relative to right shoulder. | PASS — implemented in `computeAdaptiveOrientation` (D-01: `x_axis = normalize(target − shoulder)`). |
| 2 | The orientation makes the arm naturally point at the target. | PASS by construction — the TCP +X local axis IS the shoulder→target direction (D-01); the y/z basis is right-handed and stable via the up-fallback (D-02/D-03). |
| 3 | IK success rate noticeably higher vs fixed pose. | DEFERRED to **HV-3** — measurable by running the harness twice (`:=true` then `:=false`) on the live G1 and comparing the two `PASS_COUNT n/8` lines. The plan’s D-15 acceptance is `8/8` for adaptive; the baseline is informational. |
| 4 | Workspace-boundary and shoulder-overhead difficult areas still find feasible pose. | PARTIAL — D-03 fallback to `+Y_torso` when `\|dot(x_axis, +Z_torso)\| > 0.95` is implemented in code, so the planner remains deterministic for vertical/overhead approaches. Per CONTEXT D-14 the operator chose tabletop-only UAT scope; workspace-boundary and shoulder-overhead UAT coverage is an explicit deferred verification gap (Future ORI-02 territory). The 8-target tabletop set in Plan 02 does not exercise the +Y fallback path. |

## Build & install state

- `colcon build --packages-select unitree_g1_dex3_stack --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON` rc=0.
- Built planner: `build/unitree_g1_dex3_stack/ik_fcl_ompl_planner` (7177040 bytes, executable).
- Installed planner: `install/unitree_g1_dex3_stack/lib/unitree_g1_dex3_stack/ik_fcl_ompl_planner` (7177040 bytes, executable).
- Symbol exported: `IKFCLPlannerNode::computeAdaptiveOrientation(KDL::Vector const&, double&, double&, double&, double&, KDL::Vector&) const` (weak symbol per inline class-member definition).
- Installed harness: `install/unitree_g1_dex3_stack/lib/unitree_g1_dex3_stack/adaptive_orientation_ab.py` (7606 bytes, `-rwxr-xr-x`).
- `ros2 pkg executables unitree_g1_dex3_stack` lists `adaptive_orientation_ab.py`.
- One pre-existing `dex3_controller.cpp:474` `%zu`/`int` format warning is unrelated to Phase 8 — Phase 8 did not touch that file.
- No new dependencies. `package.xml` unchanged. No new `<depend>`. No new `find_package`.

## Decisions honored

D-01 (TCP +X = shoulder→target), D-02 (+Z primary up), D-03 (+Y fallback at `0.95` threshold), D-04 (single deterministic quaternion), D-05 (shoulder = `right_shoulder_pitch_link`), D-06 (URDF/KDL FK at init, no runtime TF), D-07 (link name hardcoded, only one new ROS param), D-08 (reject within 0.05 m), D-09 (default overwrite), D-10 (`adaptive_orientation_enabled` default true), D-11 (preserve old behavior bit-exactly when false), D-12 (per-goal INFO log), D-13 (fixed 8-target tabletop A/B set), D-14 (tabletop-only UAT scope), D-15 (every adaptive target produces `/joint_trajectory_targets` → harness exits 0), D-16 (planner-only verification, no executor).

## Human verification

Four items require running on the live Unitree G1 with the planner active. Each is independently testable; none blocks Phase 9 planning, but all should be exercised before claiming the milestone v1.1 success criterion.

### HV-1 — Shoulder reference log line on planner startup

**Expected:** When the planner is launched (`ros2 launch unitree_g1_dex3_stack planner.launch.py`), its stdout contains `Right shoulder reference point in 'torso_link': [0.0040, -0.1002, 0.2478]` within ±1e-3 m on each component.

**Test:** `ros2 launch unitree_g1_dex3_stack planner.launch.py 2>&1 | grep -m1 'Right shoulder reference point'`. Expected line above.

### HV-2 — `adaptive_orientation_enabled` toggle observable on startup

**Expected:** Default launch shows `adaptive_orientation_enabled = true`; launching with `:=false` shows `adaptive_orientation_enabled = false`.

**Test:**

```bash
ros2 launch unitree_g1_dex3_stack planner.launch.py 2>&1 | grep adaptive_orientation_enabled
ros2 launch unitree_g1_dex3_stack planner.launch.py adaptive_orientation_enabled:=false 2>&1 | grep adaptive_orientation_enabled
```

### HV-3 — A/B run on tabletop set (D-15 acceptance)

**Expected:** With the planner running with `adaptive_orientation_enabled:=true`, `ros2 run unitree_g1_dex3_stack adaptive_orientation_ab.py` exits 0 within ~30 s and the final summary reports `PASS_COUNT 8/8 — adaptive=true`. With `:=false`, the same command produces an informational baseline `PASS_COUNT n/8 — adaptive=false` for the A/B comparison; n is recorded as the fixed-orientation IK success rate.

**Test:**

```bash
# Terminal 1
ros2 launch unitree_g1_dex3_stack planner.launch.py adaptive_orientation_enabled:=true
# Terminal 2
ros2 run unitree_g1_dex3_stack adaptive_orientation_ab.py; echo "exit=$?"
# expect: PASS_COUNT 8/8 — adaptive=true, exit=0

# Then re-launch terminal 1 with :=false and re-run, record baseline.
```

### HV-4 — D-08 reject path with near-shoulder goal

**Expected:** Publishing a `/goal_pose` with position within 0.05 m of the cached shoulder (e.g., `(0.0, -0.10, 0.25)`) under `adaptive_orientation_enabled:=true` produces `RCLCPP_ERROR` containing `within 0.05 m of right shoulder` on the planner's stderr and produces NO `/joint_trajectory_targets` message for that goal.

**Test:** Publish a near-shoulder goal manually with `ros2 topic pub /goal_pose ...`; observe planner stderr and absence of trajectory output.

## Notes on limits of static verification

- Live trajectory production (D-15) requires a running planner subscribed to `/goal_pose`. The harness establishes the closed-loop signal but only on hardware.
- Quaternion correctness for the 8 tabletop targets is by construction (right-handed orthonormal basis from the look-at formula); RViz visual confirmation is captured under the existing `08-VALIDATION.md` "Manual-Only Verifications" table.
- The bit-exact regression for `adaptive_orientation_enabled=false` (D-11) cannot be fully proven by static analysis because the splice block could in principle have side effects through globally-shared state. Code review confirms the block contains only local-scope mutations of `pose_in_base.pose.orientation` and four early-return paths — no other state is touched. A live hardware diff between pre-Phase-8 and Phase-8-with-toggle-off using identical input goal poses would close the loop; this is captured under `08-VALIDATION.md` "`:=false` reproduces exact pre-Phase-8 behavior".

---

*Phase: 08-adaptive-orientation*
*Verified: 2026-05-19*
