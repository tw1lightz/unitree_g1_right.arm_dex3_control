# Phase 9: End-to-End Integration — Pattern Map

**Mapped:** 2026-05-19
**Files analyzed:** 7 (3 new, 1 delete, 3 modified)
**Analogs found:** 6 / 7

## File Classification

| New/Modified File | Role | Data Flow | Closest Analog | Match Quality |
|---|---|---|---|---|
| `src/unitree_g1_dex3_stack-main/scripts/apriltag_goal_bridge.py` | service (bridge) | event-driven (keyboard) | `scripts/keyboard_trigger_node.py` | exact |
| `src/unitree_g1_dex3_stack-main/launch/apriltag_reach.launch.py` | launch | startup orchestration | `launch/reach.launch.py` | exact |
| `src/unitree_g1_dex3_stack-main/scripts/apriltag_reach_uat.py` | test (UAT) | event-driven | `scripts/adaptive_orientation_ab.py` | exact |
| `src/unitree_g1_dex3_stack-main/scripts/keyboard_trigger_node.py` | — | — | DELETE — no pattern needed | — |
| `src/unitree_g1_dex3_stack-main/CMakeLists.txt` | config | — | Self (modify `install(PROGRAMS ...)` section) | exact |
| `src/unitree_g1_dex3_stack-main/package.xml` | config | — | Self (probably no change needed) | exact |
| `src/unitree_g1_dex3_stack-main/README.md` | doc | — | Self (extend with three-entry table) | exact |

## Pattern Assignments

### `scripts/apriltag_goal_bridge.py` (service/bridge, event-driven)

**Analog:** `scripts/keyboard_trigger_node.py`

**Imports pattern** (lines 1-16):
```python
#!/usr/bin/env python3
"""ROS 2 node: cache /apriltag/target_pose, trigger on G to /goal_pose."""

import os
import select
import termios
import tty
import collections
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory

import tf2_ros
import tf2_geometry_msgs  # noqa: F401
```

**Keyboard raw-terminal reading pattern** (`keyboard_trigger_node.py` lines 25-28, 37-39, 84-87):
```python
# Setup (in __init__)
self.fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
self.old_settings = termios.tcgetattr(self.fd)
tty.setcbreak(self.fd)

# Periodic check in timer callback
def timer_callback(self):
    if select.select([self.fd], [], [], 0.0)[0]:
        ch = os.read(self.fd, 1).decode('utf-8', errors='ignore')
        if ch in ('k', 'K'):
            self.trigger()

# Cleanup (in destroy_node override)
def destroy_node(self):
    termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
    os.close(self.fd)
    super().destroy_node()
```

**ROS2 node skeleton with parameter declarations** (`apriltag_detector_node.py` lines 46-65, 386-401):
```python
class AprilTagGoalBridge(Node):
    def __init__(self):
        super().__init__('apriltag_goal_bridge')
        self.declare_parameter('reach_max_distance', 0.55)
        self.declare_parameter('stale_threshold_s', 1.0)
        self.declare_parameter('smoothing_window', 5)
        self.declare_parameter('trigger_key', 'g')

        # Read parameters
        self.reach_max = self.get_parameter('reach_max_distance').value
        self.stale_threshold = self.get_parameter('stale_threshold_s').value

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagGoalBridge()
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

**TF2 Buffer/Listener pattern** (`apriltag_detector_node.py` lines 93-94):
```python
self.tf_buffer = tf2_ros.Buffer()
self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
```

**Subscription/Publisher pattern** (`keyboard_trigger_node.py` lines 22-23):
```python
self.create_subscription(PoseStamped, '/apriltag/target_pose', self._target_cb, 10)
self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
self.create_timer(0.1, self._tick)  # timer for keyboard polling
```

**Guard pattern (repeated press prevention)** (`keyboard_trigger_node.py` lines 43-46):
```python
def trigger(self):
    if self.waiting_for_result:
        self.get_logger().warn('[KeyboardTrigger] Already waiting for detection result')
        return
    self.waiting_for_result = True
    self.trigger_pub.publish(Empty())
```

**Goal publish pattern** (`keyboard_trigger_node.py` lines 69-82):
```python
goal = PoseStamped()
goal.header.frame_id = 'torso_link'
goal.header.stamp = self.get_clock().now().to_msg()
goal.pose.position.x = avg_pos[0]
goal.pose.position.y = avg_pos[1]
goal.pose.position.z = avg_pos[2]
goal.pose.orientation = self._last_orientation
self.goal_pub.publish(goal)
self.get_logger().info(
    f'[apriltag_goal_bridge] G pressed — target=({avg_pos[0]:.3f}, '
    f'{avg_pos[1]:.3f}, {avg_pos[2]:.3f}) @ torso_link, '
    f'|target-shoulder|={dist:.3f} m, publishing /goal_pose')
```

---

### `launch/apriltag_reach.launch.py` (launch, startup orchestration)

**Analogs:** `launch/reach.launch.py` (TimerAction skeleton) + `launch/apriltag.launch.py` (RealSense + detector + static TF section)

**Imports + skeleton pattern** (`reach.launch.py` lines 1-8):
```python
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share = get_package_share_directory('unitree_g1_dex3_stack')
    launch_dir = os.path.join(package_share, 'launch')
```

**TimerAction delayed launch pattern** (`reach.launch.py` lines 49-52, 58):
```python
delayed_actions = TimerAction(period=3.0, actions=[
    planner_launch,
    control_launch,
])

return LaunchDescription([
    planning_timeout_arg,
    robot_launch,
    d435_tf_node,
    delayed_actions,
])
```

**RealSense include pattern with launch arguments** (`apriltag.launch.py` lines 66-83):
```python
realsense_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(realsense_share, 'launch', 'rs_launch.py')
    ),
    launch_arguments={
        'serial_no': '_243722074823',
        'enable_color': 'true',
        'enable_depth': 'false',
        'enable_infra1': 'false',
        'enable_infra2': 'false',
        'enable_gyro': 'false',
        'enable_accel': 'false',
        'enable_sync': 'false',
        'align_depth.enable': 'false',
        'rgb_camera.color_profile': '640x480x15',
        'initial_reset': 'true',
    }.items(),
)
```

**Static TF node pattern** (`apriltag.launch.py` lines 86-91):
```python
d435_to_camera_link = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='d435_link_to_camera_link',
    arguments=['0', '0', '0', '0', '0', '0', 'd435_link', 'camera_link'],
)
```

**AprilTag detector node definition with `emulate_tty=True`** (`apriltag.launch.py` lines 96-106):
```python
apriltag_node = Node(
    package='unitree_g1_dex3_stack',
    executable='apriltag_detector_node.py',
    name='apriltag_detector',
    output='screen',
    emulate_tty=True,
    parameters=[
        os.path.join(package_share, 'config', 'apriltag.yaml'),
        {'imshow': LaunchConfiguration('imshow')},
    ],
)
```

**Planner include with parameter passthrough** (`reach.launch.py` lines 34-41):
```python
planner_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(launch_dir, 'planner.launch.py')
    ),
    launch_arguments={
        'planning_timeout': LaunchConfiguration('planning_timeout'),
        'adaptive_orientation_enabled': LaunchConfiguration('adaptive_orientation_enabled'),
    }.items()
)
```

**Launch argument declarations** (`reach.launch.py` lines 15-19):
```python
planning_timeout_arg = DeclareLaunchArgument(
    'planning_timeout',
    default_value='1.0',
    description='Planning timeout in seconds'
)
```

---

### `scripts/apriltag_reach_uat.py` (test/UAT, event-driven)

**Analog:** `scripts/adaptive_orientation_ab.py`

**Imports pattern** (`adaptive_orientation_ab.py` lines 36-41):
```python
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
```

**Target list pattern** (`adaptive_orientation_ab.py` lines 50-60):
```python
TARGETS = [
    # (label,            x,    y,     z) in torso_link, meters
    ('center',         0.40, -0.20,  0.00),
    ('right-side',     0.40, -0.40,  0.00),
    ('low',            0.40, -0.20, -0.10),
    ('diag',           0.45, -0.30,  0.05),
]
```

**Node setup with subscription and publisher** (`adaptive_orientation_ab.py` lines 71-99):
```python
class AdaptiveOrientationAB(Node):
    def __init__(self):
        super().__init__('adaptive_orientation_ab')
        self.declare_parameter('timeout_sec', 3.0)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.traj_sub = self.create_subscription(
            JointTrajectory, '/joint_trajectory_targets', self._traj_cb, 10)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10)

        # State machine
        self._phase = 'init'
        self._current_index = 0
        self._traj_received = False
        self.results = []
        self._finished = False
        self._exit_status = 0

        self.create_timer(0.1, self._tick)
```

**State machine tick pattern** (`adaptive_orientation_ab.py` lines 127-153):
```python
def _tick(self):
    if self._phase == 'publishing':
        label, x, y, z = TARGETS[self._current_index]
        self._publish_goal(label, x, y, z)
        self._phase = 'waiting'
        return

    if self._phase == 'waiting':
        elapsed = self._now_sec() - self._publish_time
        if self._traj_received_for_current:
            self.results.append((label, x, y, z, True))
            self._advance()
        elif elapsed > self.timeout_sec:
            self.results.append((label, x, y, z, False))
            self._advance()
        return
```

**Summary/result table pattern** (`adaptive_orientation_ab.py` lines 164-174):
```python
def _summarize(self):
    passed = sum(1 for r in self.results if r[4])
    for label, x, y, z, ok in self.results:
        status = 'PASS' if ok else 'FAIL'
        self.get_logger().info(
            f'{status:<4}  "{label:<13}"  ({x:+.3f}, {y:+.3f}, {z:+.3f})')
    self.get_logger().info(
        f'PASS_COUNT {passed}/{len(TARGETS)}')
    self._exit_status = 0 if passed == len(TARGETS) else 1
```

**Main with exit code** (`adaptive_orientation_ab.py` lines 177-194):
```python
def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveOrientationAB()
    exit_status = 1
    try:
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        exit_status = node._exit_status
    except KeyboardInterrupt:
        exit_status = 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(exit_status)
```

**KDL FK for TCP position** (`scripts/read_tcp_pose.py` lines 97-118, 148-156):
```python
import pinocchio as pin

def build_reduced_model(urdf_path):
    """Build Pinocchio model with only the right arm 7 DOF + TCP frame."""
    model = pin.buildModelFromUrdf(urdf_path)

    # Lock all non-right-arm joints
    RIGHT_ARM_URDF_JOINTS = [
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    ]
    lock_ids = []
    for i, name in enumerate(model.names):
        if name not in RIGHT_ARM_URDF_JOINTS and name != "universe":
            lock_ids.append(model.getJointId(name))
    reduced = pin.buildReducedModel(model, lock_ids, np.zeros(model.nq))

    # Add TCP frame: right_wrist_yaw_joint + X axis offset 0.175 m
    reduced.addFrame(
        pin.Frame(
            "right_tcp",
            reduced.getJointId("right_wrist_yaw_joint"),
            pin.SE3(np.eye(3), np.array([0.175, 0.0, 0.0]).T),
            pin.FrameType.OP_FRAME,
        )
    )
    return reduced

# FK: all frame poses relative to model root (pelvis)
pin.framesForwardKinematics(reduced, data, right_q)

# TCP relative to torso_link: torso_inv * tcp
torso_id = reduced.getFrameId("torso_link")
tcp_id = reduced.getFrameId("right_tcp")
torso_pose = data.oMf[torso_id]
tcp_in_pelvis = data.oMf[tcp_id]
tcp_pose = torso_pose.actInv(tcp_in_pelvis)
tcp_pos = tcp_pose.translation  # (x, y, z) in torso_link
```

---

### `CMakeLists.txt` (config — modify install section)

**Analog:** Self — modify lines 116-122

**Current install(PROGRAMS ...) pattern** (lines 116-122):
```cmake
install(PROGRAMS
  scripts/tcp_torso_pose.py
  scripts/keyboard_trigger_node.py    # ← REMOVE THIS LINE
  scripts/apriltag_detector_node.py
  scripts/adaptive_orientation_ab.py
  DESTINATION lib/${PROJECT_NAME}
)
```

**Modified form (add bridge + UAT, remove keyboard_trigger):**
```cmake
install(PROGRAMS
  scripts/tcp_torso_pose.py
  scripts/apriltag_detector_node.py
  scripts/adaptive_orientation_ab.py
  scripts/apriltag_goal_bridge.py      # NEW
  scripts/apriltag_reach_uat.py        # NEW
  DESTINATION lib/${PROJECT_NAME}
)
```

---

### `package.xml` (config)

**Analog:** Self — probably no change needed.

Phase 9 uses only Python stdlib + existing ROS 2 deps (rclpy, geometry_msgs, trajectory_msgs, tf2_ros), all already declared in `package.xml`. Confirmed at lines 1-57 of the current file. No new `<depend>` or `<exec_depend>` required.

---

### `README.md` (doc)

**Analog:** Self — extend with three-entry launch table + G trigger key + UAT command + pupil-apriltags pip reminder.

**Current structure to extend** (lines 1-72): Has top-level header, environment setup section, quick start section, parameter table, current YOLO-focused description.

**Three-entry launch table to add** (from RESEARCH.md line 198-204):
```markdown
## 三条启动入口

| Launch | 用途 |
|--------|------|
| `apriltag_reach.launch.py` | 端到端：AprilTag → bridge → planner → executor，按 G 触发 |
| `reach.launch.py` | 仅 planner + executor，`ros2 topic pub /goal_pose` 手动测试 |
| `apriltag.launch.py` | 仅检测：调试 tag 识别 / 摆位 / decision_margin |
```

## Shared Patterns

### Keyboard Raw-Terminal Reading
**Source:** `scripts/keyboard_trigger_node.py` lines 25-28, 37-41, 84-87
**Apply to:** `scripts/apriltag_goal_bridge.py`
```python
# Setup (__init__)
self.fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
self.old_settings = termios.tcgetattr(self.fd)
tty.setcbreak(self.fd)

# Poll (timer callback, 0.1s)
if select.select([self.fd], [], [], 0.0)[0]:
    ch = os.read(self.fd, 1).decode('utf-8', errors='ignore')
    if ch.lower() == 'g':
        self._on_trigger()

# Cleanup (destroy_node)
termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
os.close(self.fd)
super().destroy_node()
```

### ROS 2 Python Node Skeleton (Sub/Pub/Params/Timer)
**Source:** `scripts/apriltag_detector_node.py` lines 46-131
**Apply to:** `scripts/apriltag_goal_bridge.py`, `scripts/apriltag_reach_uat.py`
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class NodeName(Node):
    def __init__(self):
        super().__init__('node_name')
        self.declare_parameter('param_name', default_value)
        self.param = self.get_parameter('param_name').value
        self.create_subscription(PoseStamped, '/topic', self._cb, 10)
        self.pub = self.create_publisher(PoseStamped, '/topic', 10)
        self.create_timer(0.1, self._tick)

def main(args=None):
    rclpy.init(args=args)
    node = NodeName()
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

### TF2 Lookup with Retry
**Source:** `scripts/apriltag_detector_node.py` lines 93-94, 308-317
**Apply to:** `scripts/apriltag_goal_bridge.py` (shoulder origin lookup)
```python
self.tf_buffer = tf2_ros.Buffer()
self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

# In retry timer:
try:
    transform = self.tf_buffer.lookup_transform(
        'torso_link', 'right_shoulder_pitch_link',
        rclpy.time.Time(),
        timeout=rclpy.duration.Duration(seconds=0.5))
    self._shoulder_origin = (
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z,
    )
except Exception as ex:
    self.get_logger().warn(f'TF lookup failed: {ex}')
```

### Sliding Window Position Average
**Source:** Python stdlib `collections.deque`
**Apply to:** `scripts/apriltag_goal_bridge.py` (position cache)
```python
import collections

self.position_cache = collections.deque(maxlen=5)  # smoothing_window

def _target_cb(self, msg):
    self.position_cache.append((
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z,
    ))
    self._last_orientation = msg.pose.orientation

def _get_averaged_position(self):
    if not self.position_cache:
        return None
    xs = [p[0] for p in self.position_cache]
    ys = [p[1] for p in self.position_cache]
    zs = [p[2] for p in self.position_cache]
    return (sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs))
```

### TimerAction Delayed Launch Composition
**Source:** `launch/reach.launch.py` lines 49-52
**Apply to:** `launch/apriltag_reach.launch.py`
```python
delayed_actions = TimerAction(period=3.0, actions=[
    apriltag_node,
    bridge_node,
    planner_launch,
    control_launch,
])

return LaunchDescription([
    imshow_arg,
    adaptive_arg,
    planning_timeout_arg,
    robot_launch,
    realsense_launch,
    d435_to_camera_link,
    delayed_actions,
])
```

### UAT Harness Structure (State Machine + Results Table + Exit Code)
**Source:** `scripts/adaptive_orientation_ab.py` lines 71-194
**Apply to:** `scripts/apriltag_reach_uat.py`
Key elements:
- Timer-driven FSM at 0.1s interval with `_phase` state variable
- Slave `_tick()` dispatch per phase
- `spin_once` main loop instead of `spin` (for exit-on-completion)
- `_summarize()` prints per-target table + `PASS_COUNT N/4`
- `sys.exit(0)` on all pass, `sys.exit(1)` on any fail

### `emulate_tty=True` for Keyboard Nodes in Launch
**Source:** `launch/apriltag.launch.py` lines 100-101
**Apply to:** `launch/apriltag_reach.launch.py` (bridge node definition)
```python
bridge_node = Node(
    package='unitree_g1_dex3_stack',
    executable='apriltag_goal_bridge.py',
    name='apriltag_goal_bridge',
    output='screen',
    emulate_tty=True,    # REQUIRED for keyboard reading in ros2 launch
    parameters=[{...}],
)
```

### Planner Adaptive Orientation Parameter Passthrough
**Source:** `launch/planner.launch.py` line 55
**Apply to:** `launch/apriltag_reach.launch.py`
```python
adaptive_arg = DeclareLaunchArgument(
    'adaptive_orientation_enabled', default_value='true',
    description='Pass through to planner.launch.py')

# In delayed actions:
planner_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(launch_dir, 'planner.launch.py')),
    launch_arguments={
        'planning_timeout': LaunchConfiguration('planning_timeout'),
        'adaptive_orientation_enabled': LaunchConfiguration(
            'adaptive_orientation_enabled'),
    }.items())
```

### Trajectory Completion Signal (Bridge Guard Reset)
**Source:** CONTEXT D-03 + D-25 (agent's discretion)
**Apply to:** `scripts/apriltag_goal_bridge.py`
```python
self._waiting_for_completion = False

def _traj_cb(self, msg: JointTrajectory):
    # Reset guard: trajectory published means planner accepted goal
    self._waiting_for_completion = False

def _on_trigger(self):
    if self._waiting_for_completion:
        self.get_logger().warn(
            '[apriltag_goal_bridge] previous goal still in flight, ignoring G')
        return
    self._waiting_for_completion = True
    # ... publish /goal_pose ...
```

## Anti-Patterns (Do NOT Copy)

| Anti-Pattern | Source | Why Avoid |
|---|---|---|
| `SetEnvironmentVariable` for CycloneDDS | `apriltag.launch.py` had this | `robot.launch.py` already sets CycloneDDS env; duplicate causes conflicts. `apriltag_reach.launch.py` includes `robot.launch.py`, so do NOT add env vars. |
| Orientation averaging | (not in codebase, but common) | Planner's `computeAdaptiveOrientation` overwrites `/goal_pose.orientation`. Averaging quaternions in bridge is wasted computation. |
| Hardcoded quaternion | `keyboard_trigger_node.py` lines 75-78 | Bridge should copy the last detected orientation, not hardcode; planner overwrites it anyway. |
| Starting nodes as separate `Node()` instead of `IncludeLaunchDescription` | (general ROS pitfall) | `planner.launch.py` and `control.launch.py` must be included, not defined as new Node actions. |

## No Analog Found

All 7 files have strong analogs. The `scripts/apriltag_reach_uat.py` KDL FK computation uses patterns from `scripts/read_tcp_pose.py` (Pinocchio buildReducedModel, framesForwardKinematics, SE3 actInv), which is a partial-match analog for that specific substep.

## Metadata

**Analog search scope:** `src/unitree_g1_dex3_stack-main/scripts/`, `src/unitree_g1_dex3_stack-main/launch/`
**Files scanned:** 11 (5 scripts + 6 launch + CMakeLists + package.xml + README)
**Pattern extraction date:** 2026-05-19
