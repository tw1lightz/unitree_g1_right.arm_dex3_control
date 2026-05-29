# Phase 8 — Pattern Map: 自适应末端位姿

**Mapped:** 2026-05-19
**Files analyzed:** 4
**Analogs found:** 4 / 4

---

## File Classification

| New / Modified File | Role | Data Flow | Closest Analog | Match Quality |
|---|---|---|---|---|
| `src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` (modified) | controller (ROS 2 node) | request-response (`/goal_pose` → `/joint_trajectory_targets`) | self — existing `goalPoseCallback` and `init()` | exact (in-place edit) |
| `src/unitree_g1_dex3_stack-main/launch/planner.launch.py` (modified) | config (launch description) | startup-config | self — existing `DeclareLaunchArgument` block | exact (in-place edit) |
| `src/unitree_g1_dex3_stack-main/scripts/adaptive_orientation_ab.py` (new) | utility (Python ROS 2 node) | event-driven publisher + subscriber | `scripts/keyboard_trigger_node.py` (`/goal_pose` publisher) + `scripts/tcp_torso_pose.py` (rclpy boilerplate) | role-match, two analogs |
| `src/unitree_g1_dex3_stack-main/CMakeLists.txt` (modified) | config (build) | static | self — existing `install(PROGRAMS …)` block | exact (in-place edit) |

---

## Pattern Assignments

### `src/ik_fcl_ompl_planner.cpp` (controller, request-response)

**Analog:** itself. Phase 8 inserts new logic into existing methods. All conventions below are extracted from the same file as authoritative project style.

**Member-variable convention** (existing private block, near `tf2_ros::Buffer tf_buffer_`):
- Trailing underscore on every private member.
- Default initializers in-class for primitives; complex types in-body of `init()`.

```cpp
double velocity_scale_ = 0.05;
std::string base_link_ = "pelvis";
tf2_ros::Buffer tf_buffer_;
tf2_ros::TransformListener tf_listener_;
```

→ Phase 8 adds two members in the same block, same style:
```cpp
KDL::Vector right_shoulder_pos_in_base_;
bool adaptive_orientation_enabled_ = true;
```

**Parameter declaration pattern** (existing `init()`, around line ~70):
```cpp
this->declare_parameter("velocity_scale", 0.05);
this->declare_parameter("base_link", "torso_link");
this->declare_parameter("right_tip", "right_tcp_link");
// ...
this->get_parameter("velocity_scale", velocity_scale_);
this->get_parameter("base_link", base_link_);
```

→ Phase 8 follows the same declare/get pattern for `adaptive_orientation_enabled`. Place declare next to `tcp_offset_x` declaration (around line ~120, the post-chain-rebuild block) and get on the very next line. Logging via `RCLCPP_INFO` consistent with the existing "Using base link / Planning timeout / …" startup INFO block.

**Logging convention** (existing throughout):
- Startup configuration: `RCLCPP_INFO(...)` once.
- Per-goal lifecycle: `RCLCPP_INFO(...)` (single line per goal, like the existing "Transformed goal_pose from …" line).
- Recoverable failures: `RCLCPP_WARN(...)`.
- Hard failures with early return: `RCLCPP_ERROR(...)`.
- Boot-stop failures: `RCLCPP_FATAL(...)` followed by `rclcpp::shutdown(); return;`.

→ Phase 8 D-12 log uses `RCLCPP_INFO`. D-08 reject uses `RCLCPP_ERROR` + `return` (no shutdown).

**Existing `goalPoseCallback` early-return pattern** (after IK failure, lines around 360):
```cpp
if (!ik_success) {
    RCLCPP_WARN(this->get_logger(), "IK failed with current state as seed. Trying neutral seed.");
    // ...
    } else {
        RCLCPP_ERROR(this->get_logger(), "IK failed with both current and neutral seed. Aborting.");
        return;
    }
}
```

→ Phase 8 D-08 reject mirrors this exactly: `RCLCPP_ERROR(...)` then `return;`.

**KDL-type conventions in this file**:
- Existing FK call: `fk_right_solver->JntToCart(joints, out, segmentIndex)`.
- `KDL::Frame target_frame(KDL::Rotation::Quaternion(...), KDL::Vector(...))`.
- `KDL::JntArray seed(planning_joints.size());` (default-constructed = zeros).

→ Phase 8 reuses `fk_right_solver` for the shoulder cache and reuses `KDL::Vector` / `KDL::Rotation` end-to-end.

**Existing per-callback PoseStamped mutation** (the file already mutates `pose_in_base` — it is a stack copy, not a `*pose` mutation, so no upstream-aliasing surprise):
```cpp
geometry_msgs::msg::PoseStamped pose_in_base;
if (pose->header.frame_id.empty() || pose->header.frame_id == base_link_) {
    pose_in_base = *pose;
} else {
    pose_in_base = tf_buffer_.transform(*pose, base_link_, tf2::durationFromSec(0.5));
}
```

→ Phase 8 mutates `pose_in_base.pose.orientation` in place at the splice point. No fresh PoseStamped is created. Add an inline comment explaining the intentional mutation (PIT-05 mitigation).

---

### `launch/planner.launch.py` (config, startup-config)

**Analog:** itself. The launch follows the project-wide pattern documented in `.planning/codebase/CONVENTIONS.md` ("Launch File Pattern: All launch files use `OpaqueFunction` to resolve `LaunchConfiguration` values").

**Existing arg-declaration pattern** (lines 39-46):
```python
args = [
    DeclareLaunchArgument('trajectory_time_step', default_value='0.05'),
    DeclareLaunchArgument('planning_timeout', default_value='1.0'),
    DeclareLaunchArgument('base_link', default_value='torso_link'),
    DeclareLaunchArgument('right_tip', default_value='right_tcp_link'),
    DeclareLaunchArgument('tcp_offset_x', default_value='0.175'),
    DeclareLaunchArgument('planner_type', default_value='RRTConnect'),
    DeclareLaunchArgument('collision_skip_pairs',
        default_value='right_hand_thumb_0_link:right_wrist_yaw_link'),
]
```

→ Phase 8 adds one entry following the same convention:
```python
DeclareLaunchArgument('adaptive_orientation_enabled', default_value='true'),
```

**Existing perform-and-coerce pattern** (lines 9-15) — note the explicit Python type cast required by OpaqueFunction context:
```python
trajectory_time_step = float(LaunchConfiguration('trajectory_time_step').perform(context))
planning_timeout = float(LaunchConfiguration('planning_timeout').perform(context))
base_link = str(LaunchConfiguration('base_link').perform(context))
```

→ Phase 8 adds a bool coercion (no `bool(...)` because Python `bool('false')` is `True`):
```python
adaptive_orientation_enabled = LaunchConfiguration('adaptive_orientation_enabled').perform(context).lower() == 'true'
```

**Existing parameters-dict pattern** (lines 17-26):
```python
parameters = {
    'trajectory_time_step': trajectory_time_step,
    'planning_timeout': planning_timeout,
    'base_link': base_link,
    'right_tip': right_tip,
    'tcp_offset_x': tcp_offset_x,
    'planner_type': planner_type,
}
if collision_skip_pairs:
    parameters['collision_skip_pairs'] = collision_skip_pairs
```

→ Phase 8 adds the new key inside the dict literal (no conditional needed):
```python
'adaptive_orientation_enabled': adaptive_orientation_enabled,
```

---

### `scripts/adaptive_orientation_ab.py` (utility, event-driven)

**Two analogs, each contributing distinct patterns.**

#### Analog 1 — `scripts/keyboard_trigger_node.py`: `/goal_pose` publishing

**Imports + class shape** (lines 1-12, 14-21):
```python
#!/usr/bin/env python3
"""ROS 2 node: ..."""

import os, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class KeyboardTriggerNode(Node):
    def __init__(self):
        super().__init__('keyboard_trigger_node')
        # ...
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
```

**`/goal_pose` publish pattern** (lines 56-72):
```python
goal = PoseStamped()
goal.header.frame_id = msg.header.frame_id
goal.header.stamp = self.get_clock().now().to_msg()
goal.pose.position.x = cx
goal.pose.position.y = y_target
goal.pose.position.z = cz
goal.pose.orientation.x = -0.68194788
goal.pose.orientation.y =  0.06844694
goal.pose.orientation.z = -0.07816853
goal.pose.orientation.w =  0.72398328
self.goal_pub.publish(goal)
self.get_logger().info(
    f'[KeyboardTrigger] Targeting nearest object at '
    f'({cx:.3f}, {y_target:.3f}, {cz:.3f}), publishing /goal_pose')
```

→ Phase 8 harness reuses this exact PoseStamped-build pattern, with:
- `goal.header.frame_id = 'torso_link'` (the test set is in torso frame per D-13).
- `goal.pose.orientation` left at any default (e.g., the same fixed quaternion from this analog) — when `adaptive=true` the planner overwrites it; when `adaptive=false` this serves as the fixed-orientation baseline. **Critical:** use the same fixed quaternion in both runs so the only variable is the planner's `adaptive_orientation_enabled` parameter.

**Main entrypoint pattern** (lines 95-108):
```python
def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

→ Phase 8 harness is non-interactive (driven by a sequence list, not keystrokes), so it does not `rclpy.spin()` indefinitely; it iterates the test set in the constructor or in a one-shot timer callback, then `node.destroy_node(); rclpy.shutdown()`. Pattern adapts to a one-shot driver.

#### Analog 2 — `scripts/tcp_torso_pose.py`: rclpy parameter + subscription boilerplate

**Parameter declaration pattern** (lines 18-26):
```python
self.declare_parameter('urdf_path', '')
self.declare_parameter('tcp_offset_x', 0.175)
self.declare_parameter('base_link', 'torso_link')
self.declare_parameter('tip_link', 'right_wrist_yaw_link')
self.declare_parameter('publish_rate', 10.0)

urdf_path = self.get_parameter('urdf_path').value
self.tcp_offset_x = self.get_parameter('tcp_offset_x').value
```

→ Phase 8 harness declares two parameters using the same pattern:
- `adaptive` (bool, default `true`) — for log labelling only; the planner reads its own `adaptive_orientation_enabled` from launch. The harness is a black-box client.
- `timeout_sec` (float, default `3.0`) — per-target wait time before declaring failure.

**Subscriber pattern** (line 96-97):
```python
self.sub_joint_states = self.create_subscription(
    JointState, '/joint_states', self.joint_state_callback, 10)
```

→ Phase 8 harness subscribes to `/joint_trajectory_targets`:
```python
self.traj_sub = self.create_subscription(
    JointTrajectory, '/joint_trajectory_targets', self.traj_callback, 10)
```

---

### `CMakeLists.txt` (config, build)

**Analog:** itself. Existing `install(PROGRAMS …)` block (lines 102-107):
```cmake
install(PROGRAMS
  scripts/tcp_torso_pose.py
  scripts/keyboard_trigger_node.py
  scripts/apriltag_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)
```

→ Phase 8 adds **one line** to this block:
```cmake
  scripts/adaptive_orientation_ab.py
```

No new `<depend>` entry in `package.xml` (rclpy + std_msgs/geometry_msgs/trajectory_msgs are already declared). No `find_package` change. No `add_executable` change.

---

## Shared Patterns

### Trailing-underscore private members (C++)
**Source:** `.planning/codebase/CONVENTIONS.md` "Member variables use trailing underscore" — verified throughout `ik_fcl_ompl_planner.cpp`.
**Apply to:** every new C++ member added in Phase 8 (`right_shoulder_pos_in_base_`, `adaptive_orientation_enabled_`).

### Per-goal single-line INFO logging
**Source:** `ik_fcl_ompl_planner.cpp` lines around the existing "Transformed goal_pose from …" message and "TRAC-IK result: …" message.
**Apply to:** D-12 adaptive-orientation log entry. One line, `%.3f`/`%.4f` precision matching existing entries (`%.3f` for positions/directions, `%.4f` for quaternion components).

### `RCLCPP_ERROR` + early `return` for unrecoverable per-goal errors
**Source:** existing IK-failure block in `goalPoseCallback`.
**Apply to:** D-08 reject path (target too close to shoulder).

### Launch arg coercion via `LaunchConfiguration(...).perform(context)`
**Source:** `launch/planner.launch.py` lines 9-15.
**Apply to:** the new `adaptive_orientation_enabled` arg. **Important:** use `.lower() == 'true'` for bool, not Python's built-in `bool()`.

### One-shot ROS 2 Python script with `try/except KeyboardInterrupt`
**Source:** `scripts/keyboard_trigger_node.py` main() and `scripts/tcp_torso_pose.py` main().
**Apply to:** the A/B harness `main()` block, with `rclpy.try_shutdown()` (not `rclpy.shutdown()` — matches keyboard_trigger_node.py).

---

## No Analog Found

None. All four files have direct project-internal analogs.

---

## Metadata

**Analog search scope:** `src/unitree_g1_dex3_stack-main/src/`, `src/unitree_g1_dex3_stack-main/launch/`, `src/unitree_g1_dex3_stack-main/scripts/`, `src/unitree_g1_dex3_stack-main/CMakeLists.txt`.
**Files scanned (read in full or excerpted):** `ik_fcl_ompl_planner.cpp`, `planner.launch.py`, `keyboard_trigger_node.py`, `tcp_torso_pose.py`, `CMakeLists.txt`, `package.xml`, `.planning/codebase/CONVENTIONS.md`.
**Pattern extraction date:** 2026-05-19.

---

*Phase: 08-adaptive-orientation*
*Mapped: 2026-05-19 via inline gsd-pattern-mapper impersonation*
*Consumed by: gsd-planner*
