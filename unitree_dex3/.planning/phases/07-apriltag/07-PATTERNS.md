# Phase 7: AprilTag 检测节点 — Patterns

**Mapped:** 2026-05-18

## File Map

| File | Role | Action | Analog |
|------|------|--------|--------|
| `src/unitree_g1_dex3_stack-main/scripts/apriltag_detector_node.py` | rclpy Python detector node | CREATE — single Node subclass: param load → CameraInfo cache → image callback (detect → filter → PnP → offset → TF → publish → draw) | `scripts/tcp_torso_pose.py` (Python rclpy node skeleton, `declare_parameter`/`get_parameter`, KeyboardInterrupt main, `/robot_description` fallback pattern); `src/visual_detection_tester.cpp` (TF buffer/listener init, sensor_data QoS, OpenCV namedWindow + imshow + waitKey UX) |
| `src/unitree_g1_dex3_stack-main/config/apriltag.yaml` | ROS 2 parameter file | CREATE — 11 fields per CONTEXT D-08 (tag_family, tag_size, target_tag_id, offset_xyz, decision_margin_min, output_frame, rgb_topic, camera_info_topic, tag_pose_topic, target_pose_topic, tf_lookup_timeout_s) | No prior YAML param file in this package — schema follows ROS 2 standard `<node_name>: ros__parameters: ...` (referenced by `tcp_torso_pose.py` declare_parameter usage) |
| `src/unitree_g1_dex3_stack-main/launch/apriltag.launch.py` | Independent test launch | CREATE — robot.launch.py include + realsense include (640x480x15, align_depth=false) + d435_link_to_camera_link static TF + apriltag_detector_node | `launch/visual_detect_click.launch.py` (parent template — ~95 % structure identical, only RS profile + node target swapped) |
| `src/unitree_g1_dex3_stack-main/CMakeLists.txt` | Build config | MODIFY — add `scripts/apriltag_detector_node.py` to `install(PROGRAMS ...)`; add new `install(DIRECTORY config DESTINATION share/${PROJECT_NAME})` block | Self — existing `install(PROGRAMS scripts/tcp_torso_pose.py scripts/keyboard_trigger_node.py ...)` pattern (L123-128 in current file) and existing `install(DIRECTORY launch robots ...)` pattern (L130-133) |
| `src/unitree_g1_dex3_stack-main/package.xml` | Package manifest | MODIFY — add `<exec_depend>realsense2_camera</exec_depend>`, `<exec_depend>python3-opencv</exec_depend>` (cv_bridge, tf2_ros, tf2_geometry_msgs already present from prior phases) | Self — existing `<depend>` block style (`<depend>cv_bridge</depend>` pattern) |
| `src/unitree_g1_dex3_stack-main/README.md` | Documentation | MODIFY — append a one-line install hint: `pip install pupil-apriltags` for new deployment machines | Self — existing README style (concise English bullets, deployment-oriented) |

## Pattern Details

### apriltag_detector_node.py

**Role:** Single rclpy Node subclass implementing the full detect → filter → PnP-offset → TF transform → publish → optional OpenCV draw pipeline. Subscribes to RGB image + CameraInfo with `qos_profile_sensor_data` reliability; publishes two `geometry_msgs/PoseStamped` topics (raw + offset-applied) only on filtered hits; owns a `tf2_ros.Buffer` + `TransformListener` to transform poses from `camera_color_optical_frame` to `torso_link`; optionally maintains an OpenCV window with corner polygon (green/red), tag ID text, projected three-axis triad, and FPS+margin HUD.

**Action:** CREATE.

**Analog #1 — Python rclpy node skeleton (`scripts/tcp_torso_pose.py`):**

The node skeleton (shebang, class init, parameter declaration, main with try/except/finally) is a near-direct copy. Reuse the patterns shown in this excerpt (current `tcp_torso_pose.py` lines 1-50 and 175-189):

```python
#!/usr/bin/env python3
"""ROS 2 node ..."""

import os
import sys

import rclpy
from rclpy.node import Node
# ... msg/lib imports ...


class TcpTorsoPoseNode(Node):
    def __init__(self):
        super().__init__('tcp_torso_pose')

        # ---------- parameters ----------
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('tcp_offset_x', 0.175)
        self.declare_parameter('base_link', 'torso_link')
        self.declare_parameter('tip_link', 'right_wrist_yaw_link')
        self.declare_parameter('publish_rate', 10.0)

        urdf_path = self.get_parameter('urdf_path').value
        self.tcp_offset_x = self.get_parameter('tcp_offset_x').value
        # ...

# (end of file)
def main(args=None):
    rclpy.init(args=args)
    node = TcpTorsoPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Pattern to follow:** Class name `AprilTagDetectorNode`, node name `'apriltag_detector'` (matches YAML `apriltag_detector: ros__parameters:` block). Declare 12 parameters (11 from D-08 + `imshow`). Same `KeyboardInterrupt` collateral. Use `rclpy.try_shutdown()` per `keyboard_trigger_node.py` pattern (slightly safer than plain `shutdown()`).

**Analog #2 — TF buffer + listener + transform (`src/visual_detection_tester.cpp` L31-69):**

C++ pattern (current code):

```cpp
class VisualDetectionTester : public rclcpp::Node {
public:
  VisualDetectionTester()
  : Node("visual_detection_tester"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) {
    // ...
    tf_buffer_.setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface()));
    // ...
  }
};
```

**Pattern to follow (Python translation):**

```python
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  side-effect: registers PoseStamped transform plugin
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

# in __init__:
self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)

# in image callback:
try:
    pose_torso = self.tf_buffer.transform(
        pose_cam, self.output_frame,
        timeout=Duration(seconds=self.tf_lookup_timeout_s)
    )
except tf2_ros.TransformException as ex:
    self.get_logger().warn(f"TF transform failed: {ex}", throttle_duration_sec=2.0)
    return
```

The `import tf2_geometry_msgs` line is the critical detail (Python-only; C++ uses header includes). Keep `# noqa: F401` to silence "unused import" linters — the import has side effects.

**Analog #3 — OpenCV namedWindow + imshow + quit-on-q (`src/visual_detection_tester.cpp` L67-77):**

C++ pattern:

```cpp
cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
// ... in imageCallback:
cv::imshow(window_name_, display_image);
int key = cv::waitKey(1) & 0xff;
if (key == 'q') { /* ... */ }
```

**Pattern to follow:**

```python
# in __init__ (only if imshow=True):
self.window_name = f"AprilTag detector (id={self.target_tag_id})"
if self.imshow_enabled:
    cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(self.window_name, 640, 480)

# in image callback (after PnP + filter + draw):
if self.imshow_enabled:
    cv2.imshow(self.window_name, display_image)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        cv2.destroyWindow(self.window_name)
        self.imshow_enabled = False  # node continues but GUI is gone
```

**Analog #4 — sensor_data QoS subscription (`src/visual_detection_tester.cpp` L66-68):**

C++ pattern: `rgb_sub_.subscribe(this, rgb_topic_, rmw_qos_profile_sensor_data);`

**Pattern to follow (Python):**

```python
from rclpy.qos import qos_profile_sensor_data
self.create_subscription(Image, self.rgb_topic, self.image_cb, qos_profile_sensor_data)
self.create_subscription(CameraInfo, self.camera_info_topic, self.info_cb, qos_profile_sensor_data)
```

For the two PoseStamped publishers, use the default reliable QoS (just pass `10` for queue depth):

```python
self.tag_pose_pub = self.create_publisher(PoseStamped, self.tag_pose_topic, 10)
self.target_pose_pub = self.create_publisher(PoseStamped, self.target_pose_topic, 10)
```

This matches `keyboard_trigger_node.py` line `self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)` — reliable QoS, queue depth 10.

---

### apriltag.yaml

**Role:** ROS 2 parameter file loaded via `parameters=[<config_path>]` in launch, exposing 11 detection-behavior knobs per CONTEXT D-08.

**Action:** CREATE.

**Analog:** No existing YAML parameter file in this package. Schema follows the ROS 2 standard format used everywhere ros2 launch's `parameters=[<file>]` is accepted.

**Pattern to follow:**

```yaml
# AprilTag detector ROS 2 parameters
# Loaded by apriltag.launch.py via parameters=[<this_file>]

apriltag_detector:                  # MUST match Node name in apriltag_detector_node.py
  ros__parameters:
    # --- detection ---
    tag_family: "tag36h11"
    tag_size: 0.08                  # tag physical edge length in meters (D-09)
    target_tag_id: 0                # only this id is published (D-07)

    # --- offset (tag-local frame, applied after PnP) ---
    offset_xyz: [0.0, 0.0, 0.05]    # placeholder; tune per object during integration (D-01)

    # --- filtering ---
    decision_margin_min: 25.0       # adjust on-site for lighting / print quality (D-10)

    # --- topics & frames ---
    output_frame: "torso_link"      # both PoseStamped messages publish in this frame (D-02)
    rgb_topic: "/camera/color/image_raw"
    camera_info_topic: "/camera/color/camera_info"
    tag_pose_topic: "/apriltag/tag_pose"
    target_pose_topic: "/apriltag/target_pose"

    # --- TF ---
    tf_lookup_timeout_s: 0.5        # match planner TF lookup pattern (D-04)
```

The top-level node name `apriltag_detector` must match the `Node.__init__('apriltag_detector')` argument so `--params-file` resolution succeeds.

`imshow` is intentionally NOT in YAML (it's a launch arg per D-15; passed through `parameters=[..., {'imshow': LaunchConfiguration('imshow')}]`).

---

### apriltag.launch.py

**Role:** Independent test launch composing robot model + RealSense + static TF + detector node, runnable as `ros2 launch unitree_g1_dex3_stack apriltag.launch.py`. Supports `imshow:=false` for headless SSH runs.

**Action:** CREATE.

**Analog:** `launch/visual_detect_click.launch.py` — the closest existing template; structure is ~95 % identical, only differing in RealSense profile values and node target.

**Current code excerpt (`launch/visual_detect_click.launch.py` lines 1-78):**

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_share = get_package_share_directory('unitree_g1_dex3_stack')
    realsense_share = get_package_share_directory('realsense2_camera')

    urdf_name_arg = DeclareLaunchArgument('urdf_name',
        default_value='g1_29dof_lock_waist_with_hand_rev_1_0.urdf', ...)
    urdf_path_arg = DeclareLaunchArgument('urdf_path', default_value='', ...)

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'robot.launch.py')
        ),
        launch_arguments={
            'urdf_name': LaunchConfiguration('urdf_name'),
            'urdf_path': LaunchConfiguration('urdf_path'),
        }.items()
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_sync': 'true',
            'align_depth.enable': 'true',                # ← Phase 7: change to 'false'
            'rgb_camera.profile': '1280x720x15',         # ← Phase 7: change to '640x480x15'
            'depth_module.profile': '1280x720x15',       # ← Phase 7: change to '640x480x15'
        }.items()
    )

    d435_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='d435_link_to_camera_link',
        arguments=['0', '0', '0', '0', '0', '0', 'd435_link', 'camera_link'],
    )

    visual_detection_click_tester = Node(
        package='unitree_g1_dex3_stack',
        executable='visual_detection_tester',
        name='visual_detection_click_tester',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'rgb_topic': '/camera/color/image_raw',
            ...
        }]
    )

    return LaunchDescription([
        urdf_name_arg, urdf_path_arg,
        robot_launch, realsense_launch, d435_to_camera_link,
        visual_detection_click_tester,
    ])
```

**Pattern to follow (Phase 7):**

1. Keep imports + `package_share` / `realsense_share` resolution identical.
2. Change `urdf_name` default to `'g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf'` (match `robot.launch.py` and `reach.launch.py` defaults).
3. Add three new launch args:
   - `config_file` — default = `os.path.join(package_share, 'config', 'apriltag.yaml')`
   - `imshow` — default `'true'`
4. RealSense include — change `align_depth.enable` to `'false'`, both profiles to `'640x480x15'`.
5. d435_to_camera_link — copy verbatim (zero-translation static publisher).
6. Replace `visual_detection_click_tester` with:
   ```python
   apriltag_node = Node(
       package='unitree_g1_dex3_stack',
       executable='apriltag_detector_node.py',
       name='apriltag_detector',                # MUST match YAML key
       output='screen',
       emulate_tty=True,
       parameters=[
           LaunchConfiguration('config_file'),  # YAML loaded as dict
           {'imshow': LaunchConfiguration('imshow')},  # launch-time override
       ],
   )
   ```
7. Return list ordering: declarations first, then launches, then static TF, then node:
   ```python
   return LaunchDescription([
       urdf_name_arg, urdf_path_arg, config_file_arg, imshow_arg,
       robot_launch, realsense_launch, d435_to_camera_link, apriltag_node,
   ])
   ```

**Do NOT include:** rviz2, planner.launch.py, control.launch.py, keyboard_trigger_node, `TimerAction` wrapper. Apriltag launch is single-purpose detection only (D-19).

**CycloneDDS env vars:** `robot.launch.py` sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and `CYCLONEDDS_URI` once at the top. `apriltag.launch.py` includes `robot.launch.py` first, so the env vars propagate automatically — do NOT duplicate them in `apriltag.launch.py`.

---

### CMakeLists.txt

**Role:** Build system — declares targets, dependencies, install rules.

**Action:** MODIFY — additive only (no removals).

**Current code excerpt (`CMakeLists.txt` lines 123-133):**

```cmake
install(PROGRAMS
  scripts/tcp_torso_pose.py
  scripts/keyboard_trigger_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  robots
  DESTINATION share/${PROJECT_NAME}
)
```

**Pattern to follow:**

1. Add `scripts/apriltag_detector_node.py` to the existing `install(PROGRAMS ...)` block (alphabetical or end-of-list — match local convention; current order is roughly chronological, so end-of-list is fine):
   ```cmake
   install(PROGRAMS
     scripts/tcp_torso_pose.py
     scripts/keyboard_trigger_node.py
     scripts/apriltag_detector_node.py
     DESTINATION lib/${PROJECT_NAME}
   )
   ```
2. Add a new `install(DIRECTORY ...)` block immediately after the existing one (do NOT merge `config` into the `launch robots` block — keep concerns separate; matches Phase 6's incremental-change convention):
   ```cmake
   install(DIRECTORY
     config
     DESTINATION share/${PROJECT_NAME}
   )
   ```

**Do NOT touch:** `find_package` block, `add_executable` lines, `BUILD_IK_FCL_OMPL_PLANNER` block, `target_link_libraries`, `ament_target_dependencies`. Phase 7 is Python-only and doesn't affect any C++ target.

---

### package.xml

**Role:** ROS 2 package manifest declaring build/runtime dependencies for rosdep + ament.

**Action:** MODIFY — additive only.

**Current code excerpt (lines 28-43, the `<depend>` block):**

```xml
  <!-- OpenCV dependency -->
  <depend>OpenCV</depend>

  <!-- Perception message types -->
  <depend>vision_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>message_filters</depend>
  <depend>tf2</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
```

`cv_bridge`, `tf2_ros`, `tf2_geometry_msgs` are already present from Phase 5/6.

**Pattern to follow — add two `<exec_depend>` entries**:

```xml
  <!-- AprilTag detector runtime deps (Phase 7) -->
  <exec_depend>realsense2_camera</exec_depend>
  <exec_depend>python3-opencv</exec_depend>
```

`<exec_depend>` (vs `<depend>`) means runtime-only — ament won't try to link against these at build time, which is correct for a Python script consumer of these binary packages.

`pupil-apriltags` is intentionally NOT added — it's a pip package not in rosdep. Its install hint goes in README.

---

### README.md

**Role:** Top-level package readme; current content is concise English bullets describing the project layout.

**Action:** MODIFY — append a one-line install hint for new deployment machines (per CONTEXT D-13 + D-14 last bullet).

**Pattern to follow:** Add a section near the bottom:

```markdown
## Phase 7: AprilTag detector

The Phase 7 detection node depends on the `pupil-apriltags` Python package (not in rosdep). Install once per deployment machine:

```bash
pip install pupil-apriltags
```

Run independently:

```bash
ros2 launch unitree_g1_dex3_stack apriltag.launch.py
ros2 launch unitree_g1_dex3_stack apriltag.launch.py imshow:=false   # headless
```

The node publishes `/apriltag/tag_pose` and `/apriltag/target_pose` (both `geometry_msgs/PoseStamped`, frame `torso_link`).
```

If the existing README is in Chinese or has a different style, match the existing voice — keep this hint terse and command-oriented.

## PATTERN MAPPING COMPLETE
