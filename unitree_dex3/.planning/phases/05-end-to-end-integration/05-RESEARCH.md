# Phase 5: End-to-End Integration — Research

**Researched:** 2026-05-14
**Phase:** 05-end-to-end-integration
**Requirements:** INTG-01, INTG-03

## 1. ROS 2 Launch Composition Pattern

### Findings
The project already uses `IncludeLaunchDescription` successfully in `visual_detect_yolo.launch.py`. This file includes both `robot.launch.py` and `perception.launch.py` with `launch_arguments` forwarding. The pattern is proven in this codebase.

`SetEnvironmentVariable` in `robot.launch.py` sets `RMW_IMPLEMENTATION` and `CYCLONEDDS_URI` at the LaunchDescription level. When included via `IncludeLaunchDescription`, these environment variables propagate to all child processes spawned by that included launch — they are set in the launch process environment before any nodes start.

### Code Pattern (from visual_detect_yolo.launch.py)
```python
robot_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(package_share, 'launch', 'robot.launch.py')
    ),
    launch_arguments={
        'urdf_name': LaunchConfiguration('urdf_name'),
    }.items()
)
```

### Risks/Pitfalls
- `SetEnvironmentVariable` in an included launch affects the entire launch process, not just that sub-launch. Since `reach.launch.py` includes `robot.launch.py` first, CycloneDDS will be set for all subsequent nodes. This is the desired behavior (D-04).
- `IncludeLaunchDescription` with `OpaqueFunction` inside works correctly — `perception.launch.py` uses this pattern and is already included by `visual_detect_yolo.launch.py`.

## 2. Keyboard Input in ROS 2 Python Node

### Findings
For reading keyboard input without extra dependencies, the standard approach is `sys.stdin` with `termios` to set raw mode and `select` for non-blocking reads. This is Linux-only but the robot runs Ubuntu.

Key considerations:
- `termios` + `select` approach: zero extra dependencies, works on Linux
- `readchar` library: cleaner API but adds a pip dependency
- **Decision per CONTEXT.md:** "选最简单、无额外依赖的方案" → use `termios`/`select`

### Code Pattern
```python
import sys, select, termios, tty

def get_key(timeout=0.1):
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return key
```

### Risks/Pitfalls
- **`emulate_tty=True` is REQUIRED** in the launch Node() action. Without it, the node's stdin is connected to `/dev/null` and `termios.tcgetattr()` will raise an error.
- Even with `emulate_tty=True`, keyboard input only works if the launch is run in a terminal with focus. This is acceptable for the use case (operator presses K in the terminal running the launch).
- The node should use a timer callback (e.g., 10 Hz) to poll `get_key()` rather than blocking `rclpy.spin()`. Pattern: `self.create_timer(0.1, self.timer_callback)` where `timer_callback` calls `get_key()`.

## 3. Launch File Parameter Forwarding (target_class → allowed_classes)

### Findings
`visual_detect_yolo.launch.py` already solves this exact problem. It forwards `target_class` to `perception.launch.py`'s `allowed_classes` using string concatenation:

```python
'allowed_classes': ["['", LaunchConfiguration('target_class'), "']"],
```

This produces the string `"['bottle']"` which `perception.launch.py` parses via `ast.literal_eval()`.

### Code Pattern
For `reach.launch.py`, replicate the same pattern:
```python
perception_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(package_share, 'launch', 'perception.launch.py')),
    launch_arguments={
        'model_path': LaunchConfiguration('model_path'),
        'imshow': LaunchConfiguration('imshow'),
        'allowed_classes': ["['", LaunchConfiguration('target_class'), "']"],
    }.items()
)
```

### Risks/Pitfalls
- The `allowed_classes` format is fragile — it relies on `ast.literal_eval` in `perception.launch.py`. The existing pattern works, so reuse it exactly.
- Default `target_class` in CONTEXT.md is `'bottle'` (D-02).

## 4. Detection3DArray Message Structure

### Findings
From `detection_to_goal_node.cpp` and `vision_msgs` message definitions:

```
vision_msgs/Detection3DArray:
  header: std_msgs/Header
  detections[]: Detection3D
    header: std_msgs/Header
    results[]: ObjectHypothesisWithPose
      hypothesis:
        class_id: string
        score: float
      pose: PoseWithCovariance
    bbox: BoundingBox3D
      center: geometry_msgs/Pose  (position.x, position.y, position.z + orientation)
      size: geometry_msgs/Vector3 (x, y, z)
```

For D-07 target point calculation:
- `center = detection.bbox.center.position` → (x, y, z)
- `size = detection.bbox.size` → (x, y, z)
- Frame: `camera_color_optical_frame` (z forward, y down)
- `y_bottom = center.y + size.y / 2`
- `y_target = y_bottom - size.y * 0.1`

### Code Pattern (Python)
```python
det = msg.detections[nearest_idx]
cx = det.bbox.center.position.x
cy = det.bbox.center.position.y
cz = det.bbox.center.position.z
sy = det.bbox.size.y

y_bottom = cy + sy / 2.0
y_target = y_bottom - sy * 0.1
# Final target: (cx, y_target, cz) in camera_color_optical_frame
```

### Risks/Pitfalls
- The `header.frame_id` on each detection comes from `project_to_3d_node` which uses `output_frame` parameter (default: `camera_color_optical_frame`). The keyboard_trigger_node should use `detection.header.frame_id` (or the array's header) for the published PoseStamped.
- The planner internally handles TF transform from `camera_color_optical_frame` to `torso_link` via `tf_buffer_.transform()` with 0.5s timeout.

## 5. CMakeLists.txt Install for New Python Script

### Findings
The existing `install(PROGRAMS ...)` block already installs Python scripts:
```cmake
install(PROGRAMS
  scripts/ultralytics_detector.py
  scripts/tcp_torso_pose.py
  DESTINATION lib/${PROJECT_NAME}
)
```

Adding `scripts/keyboard_trigger_node.py` to this list is sufficient. The script needs:
- `#!/usr/bin/env python3` shebang
- Execute permission (`chmod +x`)

No additional `ament_python` setup needed — this package uses `ament_cmake` with `install(PROGRAMS ...)` for Python scripts.

### Risks/Pitfalls
- Must add the script to `install(PROGRAMS ...)` in CMakeLists.txt or it won't be found at runtime.
- The shebang `#!/usr/bin/env python3` must point to system Python where `rclpy` is available.

## 6. emulate_tty Requirement

### Findings
When a ROS 2 node is launched via `ros2 launch`, its stdin/stdout are managed by the launch system:
- **Without `emulate_tty=True`:** stdout is line-buffered, stdin is `/dev/null`. `termios.tcgetattr(sys.stdin)` will fail with `Inappropriate ioctl for device`.
- **With `emulate_tty=True`:** A pseudo-terminal (pty) is allocated. stdout becomes unbuffered (real-time output), and stdin is connected to the pty. However, keyboard input from the user's terminal is NOT automatically forwarded to the pty.

**Critical finding:** `emulate_tty=True` alone does NOT forward keyboard input from the terminal running `ros2 launch` to the node's stdin. The pty is for output formatting only.

**Solution options:**
1. **Separate terminal:** Run `keyboard_trigger_node.py` in its own terminal via `xterm -e` or `gnome-terminal --`. This is complex and fragile.
2. **Launch prefix with xterm:** `prefix=['xterm', '-e']` — opens a new terminal window for the node.
3. **Direct stdin sharing:** Use `launch_ros` Node with `stdin=LaunchConfiguration(...)` — not supported.
4. **Best approach for this use case:** The keyboard node reads from `/dev/tty` directly instead of `sys.stdin`. `/dev/tty` always refers to the controlling terminal of the process, regardless of stdin redirection.

### Code Pattern (recommended: /dev/tty approach)
```python
import os, select, termios, tty

class KeyboardReader:
    def __init__(self):
        self.fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

    def get_key(self, timeout=0.1):
        rlist, _, _ = select.select([self.fd], [], [], timeout)
        if rlist:
            return os.read(self.fd, 1).decode('utf-8', errors='ignore')
        return ''

    def cleanup(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        os.close(self.fd)
```

With `/dev/tty`, the node reads directly from the controlling terminal. This works even when launched via `ros2 launch` because the launch process inherits the terminal. `emulate_tty=True` is still recommended for clean log output but is not strictly required for keyboard reading with this approach.

### Risks/Pitfalls
- `/dev/tty` fails if there is no controlling terminal (e.g., launched via SSH without `-t`, or from a systemd service). For the physical robot use case (operator at terminal), this is acceptable.
- Must restore terminal settings on node shutdown (use `try/finally` or `atexit`).
- If multiple nodes try to read `/dev/tty`, they'll compete. Only one keyboard reader node should exist.

## 7. TimerAction with IncludeLaunchDescription

### Findings
`TimerAction` wrapping `IncludeLaunchDescription` is already used in this project:
```python
# In perception.launch.py:
realsense_launch_delayed = TimerAction(period=5.0, actions=[realsense_launch])
```

This pattern works correctly in ROS 2 Humble. The `TimerAction` delays the execution of all actions in its list by the specified period.

For `reach.launch.py` (D-03): `TimerAction(period=3.0)` wrapping the perception, planner, and control includes ensures `robot_state_publisher` is ready before dependent nodes start.

### Code Pattern
```python
delayed_launches = TimerAction(
    period=3.0,
    actions=[
        perception_launch,
        planner_launch,
        control_launch,
        keyboard_trigger_node,
    ]
)
```

### Risks/Pitfalls
- 3.0s delay (D-03) should be sufficient for `robot_state_publisher` to publish `/robot_description` and TF. The existing `perception.launch.py` uses 5.0s for RealSense (hardware initialization). 3.0s for software-only nodes is conservative enough.
- All actions inside a single `TimerAction` start simultaneously after the delay. If ordering between perception/planner/control matters, use separate `TimerAction` instances with staggered periods. For this case, simultaneous start is fine — the planner waits for `/goal_pose` messages, and the executor waits for trajectory messages.

## 8. Integration Data Flow Verification

### End-to-End Pipeline (INTG-01)
```
[YOLO detector] → /yolo/bounding_boxes (BoundingBoxes)
    ↓
[project_to_3d_node] → /detections_3d (Detection3DArray, frame: camera_color_optical_frame)
    ↓
[keyboard_trigger_node] ← user presses K
    ↓ picks nearest, computes target point (D-07)
    ↓ publishes /goal_pose (PoseStamped, frame: camera_color_optical_frame)
    ↓
[ik_fcl_ompl_planner] ← subscribes /goal_pose
    ↓ TF transform camera_color_optical_frame → torso_link
    ↓ OMPL planning → trajectory
    ↓ publishes /planned_trajectory (JointTrajectory)
    ↓
[joint_trajectory_executor] ← subscribes /planned_trajectory
    ↓ sends LowCmd to robot (28-joint lock + right arm trajectory)
```

### Missing Link: TF from camera to robot
The `visual_detect_yolo.launch.py` includes a static TF publisher:
```python
d435_to_camera_link = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='d435_link_to_camera_link',
    arguments=['0', '0', '0', '0', '0', '0', 'd435_link', 'camera_link'],
)
```

This bridges `d435_link` (in URDF) to `camera_link` (RealSense driver frame). The full TF chain is:
`camera_color_optical_frame` → `camera_link` → `d435_link` → (URDF chain) → `torso_link`

**`reach.launch.py` MUST include this static TF publisher** or the planner's `tf_buffer_.transform()` will fail. This is not in any of the 4 sub-launches — it's only in `visual_detect_yolo.launch.py`.

### Risks/Pitfalls
- **CRITICAL:** The `d435_link` → `camera_link` static TF is NOT in `robot.launch.py` or `perception.launch.py`. It must be added to `reach.launch.py` directly (same as `visual_detect_yolo.launch.py` does).
- The planner subscribes to `/goal_pose` (confirmed in CONTEXT.md canonical refs). The keyboard_trigger_node publishes to `/goal_pose`. No topic remapping needed.
- The planner's `detection_topic` parameter defaults to `/detections` (not `/detections_3d`). But the planner actually subscribes to `/goal_pose` for the goal — the `detection_topic` param appears unused in the current right-arm-only planner (Phase 1 removed detection-based planning in favor of goal_pose input).

## Validation Architecture

### Pre-flight Checks (before running on robot)
1. `colcon build` succeeds with `keyboard_trigger_node.py` installed
2. `ros2 launch unitree_g1_dex3_stack reach.launch.py` starts without errors
3. All expected nodes appear in `ros2 node list`
4. TF tree is complete: `ros2 run tf2_tools view_frames` shows path from `camera_color_optical_frame` to `torso_link`
5. Topics connected: `ros2 topic info /goal_pose` shows keyboard_trigger_node as publisher and planner as subscriber

### Integration Test (on robot)
1. Launch full pipeline → all nodes running
2. Place object in camera view → `/detections_3d` publishes detections
3. Press K → `/goal_pose` published with correct coordinates
4. Planner receives goal → plans trajectory → publishes to `/planned_trajectory`
5. Executor receives trajectory → arm moves to target position
6. Arm does not collide with body during motion

### Documentation Validation
1. README contains: pip install command, single launch command, K-key instruction
2. ARCHITECTURE.md contains: node diagram, topic list, parameter table, decision rationale

## RESEARCH COMPLETE
