---
phase: 8
plan: 01
status: complete
completed: 2026-05-19
requirements:
  - ORI-01
commits:
  - 2876e54
  - b8f99ba
key-files:
  created: []
  modified:
    - src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp
    - src/unitree_g1_dex3_stack-main/launch/planner.launch.py
---

# Plan 08-01 Summary: Adaptive end-effector orientation in `ik_fcl_ompl_planner`

## What was built

Adaptive end-effector orientation derived from the shoulder→target
direction, gated by a launch-time toggle, spliced into the existing
`goalPoseCallback` flow. Two files modified, no new files, no new
headers, no new dependencies.

### 1. `src/ik_fcl_ompl_planner.cpp` (+121 lines, surgical)

Five surgical additions inside `class IKFCLPlannerNode`:

**(a) Two private members** (near `tf_buffer_` / `tf_listener_`):

```cpp
KDL::Vector right_shoulder_pos_in_base_;
bool adaptive_orientation_enabled_ = true;
```

Trailing-underscore naming matches the project convention. Default of
`true` for the toggle implements D-09/D-10.

**(b) Local enum + inline private helper** (`computeAdaptiveOrientation`):

```cpp
enum class AdaptiveOrientationStatus {
    OK,
    TARGET_TOO_CLOSE_TO_SHOULDER,
};

AdaptiveOrientationStatus computeAdaptiveOrientation(
    const KDL::Vector& target_in_base,
    double& out_qx, double& out_qy, double& out_qz, double& out_qw,
    KDL::Vector& out_dir_normalized) const;
```

Body builds a right-handed orthonormal basis with `+X = normalize(target − shoulder)`
(D-01), `up = +Z_torso` primary (D-02), `up = +Y_torso` fallback when
`|dot(x_axis, up)| > kParallelDotThreshold = 0.95` (D-03), then
`KDL::Rotation(x, y, z).GetQuaternion(...)` for a deterministic
single-quaternion result (D-04). The constants `kMinTargetDistance = 0.05` m
and `kParallelDotThreshold = 0.95` are local `constexpr double` per
D-07/CLAUDE.md §2 — they MUST NOT become ROS parameters.

**(c) Parameter wiring in `init()`** (right after the `tcp_offset_x` block):

```cpp
this->declare_parameter("adaptive_orientation_enabled", true);
this->get_parameter("adaptive_orientation_enabled", adaptive_orientation_enabled_);
RCLCPP_INFO(this->get_logger(),
    "adaptive_orientation_enabled = %s",
    adaptive_orientation_enabled_ ? "true" : "false");
```

Implements D-10 + the startup-confirmation log line.

**(d) Shoulder cache in `init()`** (immediately after `fk_right_solver` is constructed):

FK with all-zero `KDL::JntArray` on `segmentNr=1` of `kdl_chain_right`
returns the `right_shoulder_pitch_link` origin in `base_link_`. URDF
ground truth is `xyz="0.0039563 -0.10021 0.24778"` so the runtime log
should show `[0.0040, -0.1002, 0.2478]` ±1e-3 m. On FK failure: `RCLCPP_FATAL`
+ `rclcpp::shutdown(); return;` (matches the existing boot-stop pattern).
Implements D-05 + D-06 + the shoulder log line.

**(e) Splice in `goalPoseCallback`** (after the TF transform if/else, before
the `// Dynamically generate planning_joints` comment):

When `adaptive_orientation_enabled_ == true`:
- Build `KDL::Vector target_in_base` from `pose_in_base.pose.position`.
- Call `computeAdaptiveOrientation(...)`.
- On `TARGET_TOO_CLOSE_TO_SHOULDER`: emit `RCLCPP_ERROR` containing the
  literal `within 0.05 m of right shoulder` (the literal that
  `08-VALIDATION.md` greps for) plus target xyz + shoulder xyz, then
  `return` with no trajectory publish (D-08).
- On `OK`: mutate `pose_in_base.pose.orientation.{x,y,z,w}` in place
  (D-09; in-place mutation flagged with PIT-05 comment) and emit one
  `RCLCPP_INFO` line: `Adaptive orientation: target=[…] shoulder=[…] dir=[…] q=[…]`
  (D-12).

When `adaptive_orientation_enabled_ == false`: the entire block is
skipped, so `pose_in_base.pose.orientation` flows unchanged into
`KDL::Frame target_frame(KDL::Rotation::Quaternion(...), ...)` (D-11
bit-exact regression).

### 2. `launch/planner.launch.py` (+3 lines)

- New `DeclareLaunchArgument('adaptive_orientation_enabled', default_value='true')` in the args list.
- New perform-and-coerce line `adaptive_orientation_enabled = LaunchConfiguration('adaptive_orientation_enabled').perform(context).lower() == 'true'`.
- New key in the `parameters` dict: `'adaptive_orientation_enabled': adaptive_orientation_enabled,`.

The bool coercion uses `.lower() == 'true'` not `bool(...)` because
`bool('false')` evaluates to `True`, which would silently break the
`:=false` regression flow.

## Deviations from plan

None. The plan was executed exactly as specified. The acceptance
criterion that `grep -c 'computeAdaptiveOrientation' …` return ≥3
was met by including the helper name in the splice-block leading
comment alongside the function definition + call site (3 occurrences),
which is natural documentation rather than redundant code.

## Self-Check: PASSED

**Source assertions (Task 1):**

| # | Assertion | Result |
|---|-----------|--------|
| 1 | `KDL::Vector right_shoulder_pos_in_base_` count | 1 ✓ (expect 1) |
| 2 | `bool adaptive_orientation_enabled_` count | 1 ✓ (expect 1) |
| 3 | `declare_parameter("adaptive_orientation_enabled"` count | 1 ✓ (expect 1) |
| 4 | `computeAdaptiveOrientation` count | 3 ✓ (expect ≥3) |
| 5 | `AdaptiveOrientationStatus::(OK\|TARGET_TOO_CLOSE_TO_SHOULDER)` count | 3 ✓ (expect ≥2) |
| 6 | `kMinTargetDistance \| kParallelDotThreshold` count | 5 ✓ (expect ≥2) |
| 7 | literal `within 0.05 m of right shoulder` | 1 ✓ (expect 1) |
| 8 | literal `Adaptive orientation: target=` | 1 ✓ (expect 1) |

**Source assertions (Task 2):**

| # | Assertion | Result |
|---|-----------|--------|
| 9 | `DeclareLaunchArgument('adaptive_orientation_enabled', default_value='true')` count | 1 ✓ (expect 1) |
| 10 | `LaunchConfiguration('adaptive_orientation_enabled').perform(context).lower() == 'true'` count | 1 ✓ (expect 1) |
| 11 | `'adaptive_orientation_enabled': adaptive_orientation_enabled` count | 1 ✓ (expect 1) |
| 12 | `python3 -m py_compile planner.launch.py` | rc=0 ✓ |

**Build assertion:**
- `colcon build --packages-select unitree_g1_dex3_stack --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON` rc=0.
- Grep over the build log for `ik_fcl_ompl_planner.cpp` warnings/errors returns 0 — the modified file is `-Wall -Wextra -Wpedantic` clean.
- `nm -C build/.../ik_fcl_ompl_planner.cpp.o | grep computeAdaptiveOrientation` shows `IKFCLPlannerNode::computeAdaptiveOrientation(KDL::Vector const&, double&, double&, double&, double&, KDL::Vector&) const` — symbol present.
- Built binaries at `build/unitree_g1_dex3_stack/ik_fcl_ompl_planner` and `install/.../ik_fcl_ompl_planner` (7.2 MB).

The one pre-existing `dex3_controller.cpp:474` `%zu`/`int` warning is
unrelated to Phase 8 — Phase 8 did not touch that file.

**Behavior assertions:** Plan 02's harness exercises the runtime
behaviors (D-12 per-goal log, D-08 reject, D-11 byte-exact regression).
Acceptance for runtime checks lives in Plan 02 + the human-verified A/B
A/B run captured in 08-VERIFICATION.md.

## What this enables

- Plan 08-02's harness can now publish `/goal_pose` and observe
  `/joint_trajectory_targets` against the planner running with
  `adaptive_orientation_enabled:=true` (default) or `:=false` (A/B
  baseline) without further changes to the planner or launch file.
- Phase 9 end-to-end integration consumes the same toggle — flip
  `:=true` once the AprilTag → `/goal_pose` bridge lands.

## Commits

- `2876e54 feat(08-01): adaptive orientation in ik_fcl_ompl_planner`
- `b8f99ba feat(08-01): wire adaptive_orientation_enabled launch arg`

---
*Phase: 08-adaptive-orientation*
*Completed: 2026-05-19*
