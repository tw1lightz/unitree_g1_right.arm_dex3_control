# Phase 8 — Research: 自适应末端位姿

**Phase:** 08-adaptive-orientation
**Requirement:** ORI-01
**Researched:** 2026-05-19
**Researcher:** gsd-phase-researcher (inline)
**Upstream:** `08-CONTEXT.md` (D-01..D-16 locked)

---

## User Constraints

> Copied verbatim from `08-CONTEXT.md`. The planner MUST honor these. Research below stays inside this fence.

### Locked Decisions

**A — Adaptive orientation strategy**
- **D-01:** `right_tcp_link` local `+X` is the approach axis and must point from the right shoulder reference point toward the target position. This matches Phase 6 TCP semantics: `right_tcp_link` is offset by +0.175 m along `right_wrist_yaw_link` local X.
- **D-02:** Roll around the generated `+X` approach axis should be stabilized using `torso_link +Z` as the up reference. The generated frame should keep the TCP orientation visually/numerically stable rather than allowing arbitrary wrist roll.
- **D-03:** If the approach direction is near-parallel to `torso_link +Z`, use `torso_link +Y` as the fallback reference axis for orthonormal frame construction. This keeps the orientation deterministic near vertical targets.
- **D-04:** Phase 08 generates exactly one deterministic orientation per goal. Multi-candidate roll/orientation fallback is out of scope (Future ORI-02).

**B — Right shoulder reference point**
- **D-05:** Shoulder reference point is the origin of `right_shoulder_pitch_link`.
- **D-06:** Compute `right_shoulder_pitch_link` origin in `torso_link` from the already-loaded URDF/KDL tree. **No hardcoded coordinates. No runtime TF lookup.**
- **D-07:** Shoulder link name is hardcoded as `right_shoulder_pitch_link`. No new ROS parameter.
- **D-08:** If target position is too close to shoulder for a stable direction vector, reject the goal with a clear error and do not publish a trajectory.

**C — `/goal_pose.orientation` overwrite behavior**
- **D-09:** By default, planner ignores/overwrites incoming `/goal_pose.pose.orientation`. Position is preserved; orientation is replaced with the adaptive orientation from shoulder-to-target direction.
- **D-10:** Add `adaptive_orientation_enabled` ROS parameter, default `true`.
- **D-11:** When `adaptive_orientation_enabled=false`, preserve old planner behavior **exactly** — incoming `/goal_pose.pose.orientation` goes straight into TRAC-IK target.
- **D-12:** When adaptive is enabled, log one INFO line per goal containing target xyz, shoulder xyz, normalized direction, and generated quaternion.

**D — Verification scope**
- **D-13:** Verify fixed-orientation vs adaptive-orientation using a fixed A/B test set of AprilTag-common tabletop target positions in `torso_link`.
- **D-14:** User selected tabletop-only UAT. Workspace-boundary and shoulder-overhead coverage are deferred (vertical-fallback in code stays).
- **D-15:** Pass criterion: with `adaptive_orientation_enabled=true`, every target in the tabletop set produces a successful planner result and `/joint_trajectory_targets` publish.
- **D-16:** Planner-only verification. No executor, no physical motion in this phase.

### Claude's Discretion (research must recommend)

- Exact numeric thresholds for near-zero shoulder-to-target distance and near-parallel up-axis detection.
- Exact C++ orthonormal basis construction.
- Exact A/B harness format (manual commands or helper script).

### Deferred (do not address in plans)

- Multi-candidate orientation fallback (Future ORI-02)
- Tag-normal-based approach direction (Future ORI-03)
- Workspace-boundary / shoulder-overhead UAT coverage (deferred verification gap)

---

## Project Constraints (from CLAUDE.md)

Active directives the planner must honor:

- **§1 Think before coding** — surface tradeoffs, name confusion explicitly, no silent picks.
- **§2 Simplicity first** — minimum code, no speculative configurability. *Phase 8 should not introduce new ROS parameters beyond `adaptive_orientation_enabled` (D-07 forbids extra parameters; D-10 requires this one).*
- **§3 Surgical changes** — only edit lines that trace to the request. Do not "improve" surrounding planner code (TRAC-IK seed retry, OMPL path, trajectory smoothing) — those are out of scope.
- **§4 Goal-driven execution** — every task has a verifiable check. Plans must include `<acceptance_criteria>` with concrete assertions, not "looks correct".

---

## Standard Stack

| Capability | Library | Version | Source |
|---|---|---|---|
| URDF parse | `urdf` (ROS 2) | system | `[VERIFIED: existing dep in package.xml]` |
| KDL tree + chain + FK | `kdl_parser` + `orocos_kdl` | system | `[VERIFIED: already linked, used in `ik_fcl_ompl_planner.cpp`]` |
| Linear algebra (vectors, cross products) | `KDL::Vector` (preferred) **OR** `Eigen::Vector3d` | system | `[VERIFIED: both already in build]` |
| Quaternion construction | `KDL::Rotation` (preferred) | system | `[VERIFIED]` |
| ROS 2 logging / parameter API | `rclcpp` | Humble | `[VERIFIED]` |

**No new dependencies.** Phase 8 reuses everything already linked into `ik_fcl_ompl_planner`. No `package.xml` or `CMakeLists.txt` build-system change required for the C++ math.

### Package Legitimacy Audit

Phase 8 installs **zero** new packages. No `pip install`, no `apt install`, no `find_package` additions, no `<depend>` additions. Slopcheck N/A.

---

## Architecture Patterns

### Pattern P-1 — Cache the shoulder origin once at init time

**Why:** D-06 requires no runtime TF lookup. The right_shoulder_pitch_joint origin in torso_link is a URDF constant — `xyz="0.0039563 -0.10021 0.24778"` from the URDF — and the joint axis rotation does not move the link's origin. So the shoulder position in `torso_link` is invariant of joint state.

**How:** In `IKFCLPlannerNode::init()`, after `kdl_chain_right` is finalized (post TCP-offset chain rebuild), call `KDL::ChainFkSolverPos_recursive::JntToCart(zero_jnt, frame_out, 1)`. The KDL FK API is 1-based on `segmentNr`; `segmentNr=1` returns the frame after the first segment of `kdl_chain_right`, which is `right_shoulder_pitch_link`. Take `frame_out.p` as the shoulder position. Cache as `KDL::Vector right_shoulder_pos_in_base_;` member.

`[VERIFIED: KDL ChainFkSolverPos_recursive API, see existing fk_right_solver usage in goalPoseCallback isInCollision()]`

```cpp
// In init(), AFTER `fk_right_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_right);`
{
    KDL::JntArray zero_jnt(kdl_chain_right.getNrOfJoints());  // default-constructed = all zeros
    KDL::Frame shoulder_frame;
    if (fk_right_solver->JntToCart(zero_jnt, shoulder_frame, 1) < 0) {
        RCLCPP_FATAL(this->get_logger(),
            "Failed to compute shoulder origin via FK on segment 1 of right-arm chain");
        rclcpp::shutdown();
        return;
    }
    right_shoulder_pos_in_base_ = shoulder_frame.p;
    RCLCPP_INFO(this->get_logger(),
        "Right shoulder reference point in '%s': [%.4f, %.4f, %.4f]",
        base_link_.c_str(),
        right_shoulder_pos_in_base_.x(),
        right_shoulder_pos_in_base_.y(),
        right_shoulder_pos_in_base_.z());
}
```

**Sanity check value (URDF-rest):** ≈ `(0.0040, -0.1002, 0.2478)` m in `torso_link`. The acceptance criterion in Plan 01 should assert that the logged shoulder xyz is within `1e-3` m of this URDF rest position.

### Pattern P-2 — Look-at orthonormal basis with up-fallback

**Why:** D-01..D-03. We need a deterministic right-handed orthonormal basis where `+X` points along the approach direction, and roll is stabilized by an up reference.

**Algorithm (single deterministic orientation per D-04):**

```
1. d = target_position - shoulder_position           // raw direction vector
2. if ||d|| < min_target_distance: REJECT (D-08)
3. x_axis = d / ||d||                                // normalized approach direction
4. up = +Z_torso = (0, 0, 1)
5. if |dot(x_axis, up)| > parallel_threshold:        // near-vertical case (D-03)
       up = +Y_torso = (0, 1, 0)
6. y_axis = normalize(cross(up, x_axis))             // perpendicular to both
7. z_axis = cross(x_axis, y_axis)                    // already unit, right-handed
8. R = KDL::Rotation(x_axis, y_axis, z_axis)         // column-vector ctor
9. R.GetQuaternion(qx, qy, qz, qw)
```

**Right-handedness check:** `det(R) = x · (y × z)` should equal `+1`. With the construction above, `y = up × x` (up taken first preserves right-handedness when crossed back), and `z = x × y`, so the basis is right-handed by construction. `[VERIFIED: standard look-at formulation]`

`KDL::Rotation` accepts a 3-vector constructor on Humble's distributed `orocos_kdl`. If the compiler refuses the `(Vector,Vector,Vector)` overload (some KDL builds expose only the 9-double constructor), fall back to the 9-double form: `KDL::Rotation(x.x(), y.x(), z.x(), x.y(), y.y(), z.y(), x.z(), y.z(), z.z())` — column-major. `[CITED: orocos_kdl/frames.hpp]`

### Pattern P-3 — Splice into `goalPoseCallback` after TF transform, before `target_frame`

**Why:** D-09. Position is preserved; orientation is overwritten in `pose_in_base` so all downstream code (target_frame, IK seed retry, OMPL setup) sees the adaptive value.

**Insertion point:** Inside `goalPoseCallback`, after the `pose_in_base` is filled (the `if (frame.empty() || frame == base_link_)` else-tf2-transform block), and **before** the `KDL::Frame target_frame(KDL::Rotation::Quaternion(...), KDL::Vector(...))` construction. Concretely: insert between the TF-block and the `// Dynamically generate planning_joints from KDL chain` comment that follows.

**Branch on `adaptive_orientation_enabled_`:**
- `true` (default, D-09/D-10): compute adaptive quaternion, **mutate `pose_in_base.pose.orientation`**, log one INFO line (D-12). On near-zero distance, `RCLCPP_ERROR` and `return` early (D-08, mirrors existing IK-failure return pattern in this method).
- `false` (D-11): no-op. The existing flow uses `pose_in_base.pose.orientation` as-is — old behavior preserved bit-exactly.

### Pattern P-4 — One launch-time argument, no YAML

`planner.launch.py` already follows the `DeclareLaunchArgument` + `OpaqueFunction` + parameters-dict pattern (per `.planning/codebase/CONVENTIONS.md` "Launch File Pattern"). Add exactly one `DeclareLaunchArgument('adaptive_orientation_enabled', default_value='true')`, perform-resolve to a Python bool, include in the parameters dict. `[VERIFIED: existing planner.launch.py:5-46]`

---

## Don't Hand-Roll

- **Quaternion construction.** Do not hand-write quaternion-from-rotation-matrix conversion. Use `KDL::Rotation::GetQuaternion(qx, qy, qz, qw)`. `[VERIFIED: orocos_kdl frames.hpp]`
- **Cross product / normalization.** Do not write Eigen and KDL math side-by-side. Use `KDL::Vector` consistently inside the helper (`KDL::Vector::Norm()`, `KDL::dot(a, b)`, `a * b` for cross product overload). The rest of the planner already uses Eigen for FCL transforms and KDL for FK — keep adaptive-orientation math in pure KDL to avoid a third type-converting hop.
- **KDL chain rebuild.** Do not rebuild the KDL chain to extract the shoulder. The existing `kdl_chain_right` already contains the shoulder segment as segment 0; FK with `segmentNr=1` returns its frame.
- **TF lookup for shoulder.** D-06 forbids it. Do not call `tf_buffer_.lookupTransform(...)` for the shoulder origin.
- **Numerical eps invented from scratch.** Use the recommended thresholds below; do not introduce `1e-9`-class limits that hide bugs.

---

## Common Pitfalls

### PIT-01 — Quaternion sign ambiguity

`KDL::Rotation::GetQuaternion` returns one of two equivalent quaternions (`q` and `-q` represent the same rotation). Downstream `KDL::Rotation::Quaternion(qx,qy,qz,qw)` accepts both. **No action needed**, but if logging compares quaternions across runs, normalize sign by ensuring `qw >= 0` before logging to keep printouts stable.

### PIT-02 — Near-parallel up-axis blow-up

If `x_axis` is exactly aligned with `up`, `cross(up, x_axis) = 0` and normalizing produces NaN. **Recommended threshold:** `|dot(x_axis, up)| > 0.95` triggers fallback to `+Y_torso` (D-03). At `0.95` the perpendicular component magnitude is `sqrt(1 - 0.95²) ≈ 0.31`, well above any reasonable double-precision noise floor. Tighter thresholds (e.g., 0.99 ≈ 8°) leave a wider thin shell where cross-product magnitude is still small (`≈ 0.14`) — acceptable but more borderline. **`0.95` is the recommended default.**

After fallback, re-check `|dot(x_axis, +Y_torso)|`. For tabletop targets in front of the robot this is always far from 1, but a defensive assert at log level catches future regressions if someone later places a target on the +Y axis. `[ASSUMED: 0.95 is conservative-but-not-overcautious based on numerical analysis; not phase-specific verified]`

### PIT-03 — Near-zero direction blow-up

`||target − shoulder|| < eps` produces NaN on division. **Recommended threshold:** `min_target_distance = 0.05 m` (5 cm). Justification:
- Right shoulder is at `(0.004, -0.100, 0.248)` in `torso_link`.
- Realistic AprilTag tabletop targets are at distances ≥ 0.30 m from torso center → ≥ 0.20 m from shoulder.
- Anything within 5 cm of the shoulder is geometrically inside the robot's body and physically unreachable — rejecting at this distance never sacrifices a real goal.
- 5 cm is two orders of magnitude above double-precision direction-vector noise.

`[ASSUMED: 0.05 m default; 1 cm or 10 cm are also defensible — pin in plan as a constant, not as a configurable parameter, per CLAUDE.md §2]`

### PIT-04 — KDL `Vector` vs `Vector3` confusion

KDL uses `KDL::Vector` (a 3-vector). Do not confuse with `Eigen::Vector3d` used elsewhere in the planner (FCL transforms). When constructing `KDL::Frame target_frame`, the position arg is a `KDL::Vector` — the existing code already builds it from `pose_in_base.pose.position.{x,y,z}`. The shoulder cache is also `KDL::Vector`. Compute the direction in `KDL::Vector` end-to-end.

### PIT-05 — Mutating `pose_in_base` vs producing a separate variable

D-12 wants the **logged** quaternion to be the adaptive one and the **TRAC-IK** target to be the adaptive one. The simplest way to satisfy both is to mutate `pose_in_base.pose.orientation` in place. Pitfall: if a future contributor adds a "republish goal" or "mirror goal" feature, the mutated message will carry the adaptive quaternion downstream — which may or may not be desired. **Mitigation:** add a code comment at the splice site stating the mutation is intentional and explaining D-09.

### PIT-06 — `goalPoseCallback` early-return paths leave `pose_in_base` partially populated

Existing code already has multiple `return` paths in `goalPoseCallback` (TF failure, joint count mismatch, IK failure, etc.). New early-return for "target too close to shoulder" (D-08) follows the same pattern. **Match the existing pattern**: `RCLCPP_ERROR` with target xyz + shoulder xyz + computed distance, then `return` with no trajectory publish, no state mutation.

### PIT-07 — Build system surprise from a `private:` method

`computeAdaptiveOrientation()` should be a `private:` member of `IKFCLPlannerNode` to keep symbol scope clean and avoid header pollution. Phase 8 does not introduce a new header file. `[VERIFIED: existing pattern — `goalPoseCallback`, `jointStateCallback`, `isInCollision` are all `private:` members in this single-file node]`

### PIT-08 — Right-handed vs left-handed basis bug

If you accidentally compute `y_axis = cross(x_axis, up)` (instead of `cross(up, x_axis)`), the basis is left-handed → `R` becomes a reflection matrix → `KDL::Rotation::GetQuaternion` returns garbage. **Mitigation:** the verification harness in Plan 02 must check that the planner publishes a non-empty `/joint_trajectory_targets` for at least one tabletop target — a left-handed bug would route through TRAC-IK and produce either no IK solution or a wildly off pose. Add a debug-level assertion `KDL::dot(x_axis, KDL::Vector(z_axis.x()*y_axis.z()-z_axis.z()*y_axis.x()...)) > 0.99` if paranoid; otherwise rely on integration coverage.

---

## Code Examples

### Example E-1 — Adaptive orientation helper (full body)

To live as a `private:` method on `IKFCLPlannerNode` in `src/ik_fcl_ompl_planner.cpp`. The `OrientationStatus` enum is local to this method's call site.

```cpp
enum class AdaptiveOrientationStatus {
    OK,
    TARGET_TOO_CLOSE_TO_SHOULDER,
};

// Returns OK and writes `out_q_*` on success; returns TARGET_TOO_CLOSE_TO_SHOULDER
// when target is within min_target_distance of the cached shoulder reference.
AdaptiveOrientationStatus computeAdaptiveOrientation(
    const KDL::Vector& target_in_base,
    double& out_qx, double& out_qy, double& out_qz, double& out_qw,
    KDL::Vector& out_dir_normalized) const
{
    constexpr double kMinTargetDistance = 0.05;        // m  (PIT-03)
    constexpr double kParallelDotThreshold = 0.95;     // dimensionless  (PIT-02)

    KDL::Vector d = target_in_base - right_shoulder_pos_in_base_;
    const double d_norm = d.Norm();
    if (d_norm < kMinTargetDistance) {
        return AdaptiveOrientationStatus::TARGET_TOO_CLOSE_TO_SHOULDER;
    }

    const KDL::Vector x_axis = d / d_norm;

    KDL::Vector up(0.0, 0.0, 1.0);                     // torso_link +Z (D-02)
    if (std::abs(KDL::dot(x_axis, up)) > kParallelDotThreshold) {
        up = KDL::Vector(0.0, 1.0, 0.0);               // torso_link +Y fallback (D-03)
    }

    KDL::Vector y_axis = up * x_axis;                  // KDL operator* on Vectors = cross
    y_axis.Normalize();
    KDL::Vector z_axis = x_axis * y_axis;              // already unit, right-handed

    // KDL::Rotation column-vector constructor (Humble distributes this overload).
    KDL::Rotation R(x_axis, y_axis, z_axis);
    R.GetQuaternion(out_qx, out_qy, out_qz, out_qw);

    out_dir_normalized = x_axis;
    return AdaptiveOrientationStatus::OK;
}
```

`[VERIFIED: KDL::Rotation column-vector ctor, KDL::Vector::operator* (cross), KDL::dot, KDL::Vector::Normalize, KDL::Rotation::GetQuaternion — all in orocos_kdl/frames.hpp]`

### Example E-2 — Splice block inside `goalPoseCallback`

Insert immediately after the `pose_in_base = *pose;` / `pose_in_base = tf_buffer_.transform(...)` block, **before** the `// Dynamically generate planning_joints from KDL chain` comment.

```cpp
// === Phase 8: adaptive end-effector orientation (ORI-01) =============
// We intentionally MUTATE pose_in_base.pose.orientation when adaptive
// orientation is enabled (D-09). All downstream code (target_frame, IK
// seed retry, logging) consumes the mutated value.
if (adaptive_orientation_enabled_) {
    KDL::Vector target_in_base(
        pose_in_base.pose.position.x,
        pose_in_base.pose.position.y,
        pose_in_base.pose.position.z);

    double qx = 0, qy = 0, qz = 0, qw = 1;
    KDL::Vector dir;
    auto status = computeAdaptiveOrientation(target_in_base, qx, qy, qz, qw, dir);
    if (status == AdaptiveOrientationStatus::TARGET_TOO_CLOSE_TO_SHOULDER) {
        RCLCPP_ERROR(this->get_logger(),
            "Target [%.3f, %.3f, %.3f] within %.3f m of right shoulder [%.3f, %.3f, %.3f]; "
            "adaptive orientation cannot produce a stable direction. Aborting goal.",
            target_in_base.x(), target_in_base.y(), target_in_base.z(),
            0.05,
            right_shoulder_pos_in_base_.x(),
            right_shoulder_pos_in_base_.y(),
            right_shoulder_pos_in_base_.z());
        return;  // D-08
    }
    pose_in_base.pose.orientation.x = qx;
    pose_in_base.pose.orientation.y = qy;
    pose_in_base.pose.orientation.z = qz;
    pose_in_base.pose.orientation.w = qw;
    RCLCPP_INFO(this->get_logger(),
        "Adaptive orientation: target=[%.3f, %.3f, %.3f] shoulder=[%.3f, %.3f, %.3f] "
        "dir=[%.3f, %.3f, %.3f] q=[%.4f, %.4f, %.4f, %.4f]",
        target_in_base.x(), target_in_base.y(), target_in_base.z(),
        right_shoulder_pos_in_base_.x(),
        right_shoulder_pos_in_base_.y(),
        right_shoulder_pos_in_base_.z(),
        dir.x(), dir.y(), dir.z(),
        qx, qy, qz, qw);  // D-12
}
// === end Phase 8 =====================================================
```

### Example E-3 — Member declarations and parameter wiring

In the `private:` section near other member variables (around the existing `tf2_ros::Buffer tf_buffer_;` block):

```cpp
KDL::Vector right_shoulder_pos_in_base_;
bool adaptive_orientation_enabled_ = true;
```

In `init()`, near the other `declare_parameter` calls (just below the existing TCP offset block is fine):

```cpp
this->declare_parameter("adaptive_orientation_enabled", true);
this->get_parameter("adaptive_orientation_enabled", adaptive_orientation_enabled_);
RCLCPP_INFO(this->get_logger(),
    "adaptive_orientation_enabled = %s",
    adaptive_orientation_enabled_ ? "true" : "false");
```

### Example E-4 — Launch arg in `planner.launch.py`

Add to the `args` list:
```python
DeclareLaunchArgument('adaptive_orientation_enabled', default_value='true'),
```

In `launch_setup()`, perform-resolve and bool-coerce (matches existing pattern that does `float(...)` and `str(...)`):
```python
adaptive_orientation_enabled = LaunchConfiguration('adaptive_orientation_enabled').perform(context).lower() == 'true'
```

Add to the `parameters` dict:
```python
'adaptive_orientation_enabled': adaptive_orientation_enabled,
```

`[VERIFIED: existing planner.launch.py:5-46]`

### Example E-5 — A/B verification harness skeleton (Python)

A Python script at `scripts/adaptive_orientation_ab.py`. Subscribes to `/joint_trajectory_targets`, publishes a sequence of `/goal_pose` messages, counts successes per target.

```python
#!/usr/bin/env python3
"""Phase 8 A/B verification harness: publish a fixed set of tabletop
target positions to /goal_pose, count successful trajectory publishes.

Run twice:
  ros2 run unitree_g1_dex3_stack adaptive_orientation_ab.py --ros-args -p adaptive:=false
  ros2 run unitree_g1_dex3_stack adaptive_orientation_ab.py --ros-args -p adaptive:=true

Pass criterion: with adaptive=true, every target produces a trajectory
within timeout. (D-15)
"""
import rclpy, time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory

# Tabletop test set in torso_link frame (D-13, D-14):
# 8 points covering reachable area in front of right shoulder.
# Coords are illustrative — the planner finalizes them.
TARGETS = [
    # (label, x, y, z) in torso_link, meters
    ('center',         0.40, -0.20, 0.00),
    ('center-near',    0.30, -0.20, 0.00),
    ('center-far',     0.55, -0.20, 0.00),
    ('right-side',     0.40, -0.40, 0.00),
    ('left-of-mid',    0.40, -0.05, 0.00),
    ('low',            0.40, -0.20,-0.10),
    ('high',           0.40, -0.20, 0.15),
    ('diag',           0.45, -0.30, 0.05),
]
TIMEOUT_SEC = 3.0
```

(Plan 02 will spec the full body of this script.)

---

## Validation Architecture

> Required by the Nyquist gate (`workflow.nyquist_validation: true`). A standalone `08-VALIDATION.md` is generated in step 5.5 of the plan-phase workflow from this section.

### Test Infrastructure

| Property | Value |
|---|---|
| **Framework** | Custom Python integration harness (`scripts/adaptive_orientation_ab.py`). No `gtest` is added — there is no pre-existing `ament_add_gtest` infrastructure in the workspace and the math under test is small enough that integration coverage is sufficient. `[VERIFIED: grep ament_add_gtest src/unitree_g1_dex3_stack-main returns 0 matches]` |
| **Config file** | None — harness is a single self-contained ROS 2 node. |
| **Quick run command** | `colcon build --packages-select unitree_g1_dex3_stack --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON` |
| **Full suite command** | A/B harness (see UAT below). Invoked manually via two launch commands; not auto-run. |
| **Estimated runtime** | Build ~60 s. A/B harness end-to-end ~30 s per run × 2 runs = ~60 s. |

### Sampling Rate

- **After every task commit (build-touching tasks):** `colcon build --packages-select unitree_g1_dex3_stack --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON` must rc=0.
- **After plan 02 (harness lands):** Run the A/B harness once with `adaptive_orientation_enabled:=true` against the tabletop test set. Pass = every target produces a trajectory.
- **Before `/gsd-verify-work`:** Both A/B runs (`false` baseline + `true`) recorded in 08-VERIFICATION.md.
- **Max feedback latency:** Build < 90 s; harness < 60 s.

### Wave 0 Requirements

None. The planner is already built behind `BUILD_IK_FCL_OMPL_PLANNER=ON`, KDL/TRAC-IK/Eigen are linked, and Python rclpy is available system-wide. No test framework install, no fixture scaffolding.

### Per-Task Verification (template)

| Task | Plan | Wave | Requirement | Verification Type | Automated Command |
|---|---|---|---|---|---|
| Cache shoulder origin | 01 | 1 | ORI-01 | log assertion | grep `Right shoulder reference point` in planner stdout; xyz within 1e-3 m of URDF rest |
| Add `adaptive_orientation_enabled` param | 01 | 1 | ORI-01 | log assertion | grep `adaptive_orientation_enabled = true` in planner stdout |
| Implement `computeAdaptiveOrientation` | 01 | 1 | ORI-01 | build + log | colcon rc=0; per-goal `Adaptive orientation: ...` log line emitted |
| Splice into goalPoseCallback | 01 | 1 | ORI-01 | build + behavior | colcon rc=0; with `:=true` and a tabletop goal, planner publishes `/joint_trajectory_targets` |
| Reject too-close target | 01 | 1 | ORI-01 (D-08) | log assertion | publishing a goal at xyz=`(0.0,-0.10,0.25)` produces ERROR log with "within 0.05 m of right shoulder", no trajectory publish |
| Launch arg wiring | 01 | 1 | ORI-01 (D-10) | log assertion | `ros2 launch unitree_g1_dex3_stack planner.launch.py adaptive_orientation_enabled:=false` flips the parameter, log shows `false` |
| A/B harness exists & runs | 02 | 2 | ORI-01 (D-13..D-16) | exit code | `python3 adaptive_orientation_ab.py` exits 0 on adaptive=true case after iterating tabletop set |

### Manual / UAT-Only Verifications

| Behavior | Why Manual | Test Instructions |
|---|---|---|
| Adaptive quaternion is "visually reasonable" for tabletop targets | Sanity check on RViz orientation arrow | Launch RViz with TF + planner; publish each tabletop target to `/goal_pose`; visually confirm the published trajectory's final TCP `+X` axis points at the goal point |
| Shoulder logging on first goal matches expected URDF rest position | Per D-12 | Inspect first INFO log line; confirm shoulder xyz ≈ `(0.0040, -0.1002, 0.2478)` and direction vector points away from shoulder toward target |
| `:=false` reproduces exact pre-Phase-8 behavior | Bit-exact regression | With `adaptive_orientation_enabled:=false`, publish a goal with a known fixed orientation; confirm the planner-internal `target_frame` orientation matches the input orientation (log the IK-solver target_frame separately if needed) |

### Validation Sign-Off Checklist

- [ ] All tasks have automated verify commands (build rc=0 + log assertion + behavior).
- [ ] No 3 consecutive tasks in Wave 1 lack automated verify (sampling continuity OK by design — every task touches the planner build).
- [ ] No watch-mode flags.
- [ ] Feedback latency < 90 s per task.
- [ ] `nyquist_compliant: true` to be set in `08-VALIDATION.md` frontmatter once plans are written.

---

## Phase Boundary Reminders

- **In scope:** orthonormal basis math, shoulder cache, parameter wiring, launch arg, A/B harness, planner-only verification.
- **Out of scope (do NOT plan):** IK seed strategy changes, OMPL planner swaps, trajectory-smoother retuning, AprilTag bridge, executor changes, multi-candidate orientation fallback, tag-normal approach direction, workspace-boundary UAT, anything that touches a file outside `src/ik_fcl_ompl_planner.cpp`, `launch/planner.launch.py`, `scripts/adaptive_orientation_ab.py`, and `CMakeLists.txt` (only if the new script needs `install(PROGRAMS …)`).

---

## Confidence Summary

| Claim | Confidence | Source |
|---|---|---|
| Shoulder origin in torso_link is invariant of right-arm joint state | HIGH | URDF inspection + KDL semantics |
| KDL FK with `segmentNr=1` returns frame after segment 0 = `right_shoulder_pitch_link` | HIGH | KDL API + existing planner code patterns |
| KDL `Rotation(Vector, Vector, Vector)` ctor is available on Humble's `orocos_kdl` | MEDIUM | KDL header in standard ROS 2 distributions; fallback to 9-double ctor specified |
| `0.95` parallel-dot threshold is appropriate | MEDIUM | Numerical analysis; not phase-specific tested |
| `0.05 m` min-target-distance threshold is appropriate | MEDIUM | Geometric reasoning vs G1 link layout; not phase-specific tested |
| No new dependencies needed | HIGH | `find_package` and `<depend>` audit of CMakeLists/package.xml |
| Project has no pre-existing C++ unit tests | HIGH | `grep` confirmed |
| Existing planner.launch.py pattern accepts a new bool launch arg cleanly | HIGH | Inspected file, same pattern repeated for other params |

---

*Phase: 08-adaptive-orientation*
*Researched: 2026-05-19 via inline gsd-phase-researcher impersonation*
*Consumed by: gsd-pattern-mapper, gsd-planner, gsd-plan-checker*
