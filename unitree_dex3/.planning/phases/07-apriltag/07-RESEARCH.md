# Phase 7: AprilTag 检测节点 — Research

**Researched:** 2026-05-18
**Phase:** 07-apriltag
**Requirements:** TAG-01, TAG-02, TAG-03, TAG-04

## Executive Summary

Phase 7 builds a self-contained Python rclpy detection node on top of three external pieces (pupil-apriltags PnP detector, cv_bridge ROS↔OpenCV image conversion, tf2_ros buffer with `tf2_geometry_msgs` registered for `PoseStamped`). All three are mature with stable APIs; the only mechanical risk is forgetting the `import tf2_geometry_msgs` side-effect that registers the `do_transform_pose` plugin into `tf_buffer.transform()`. The independent launch is a near-clone of `visual_detect_click.launch.py` with two RealSense parameter changes (`rgb_camera.profile=640x480x15`, `align_depth.enable=false`) and the node target swapped. CONTEXT.md has already locked all 20 design decisions (D-01 through D-20); research below confirms each external API surface against the locked decisions.

Validation architecture below maps TAG-01..TAG-04 to concrete observable assertions runnable with `ros2 topic echo`, `ros2 param get`, and `colcon build`.

## 1. pupil-apriltags Python API (D-01, D-04, D-09, D-10, D-11)

### Detector construction

```python
from pupil_apriltags import Detector

detector = Detector(
    families="tag36h11",   # D-08: tag_family
    nthreads=1,            # single-threaded sufficient at 640x480x15
    quad_decimate=1.0,     # full resolution; >1 downsamples for speed
    quad_sigma=0.0,
    refine_edges=1,
    decode_sharpening=0.25,
    debug=0,
)
```

The constructor's `families` argument accepts a space-separated string of family names — `"tag36h11"` is correct and standard for this project.

### Per-frame detection

```python
detections = detector.detect(
    gray_image,                       # numpy.ndarray, dtype=uint8, shape=(H, W)
    estimate_tag_pose=True,
    camera_params=(fx, fy, cx, cy),   # tuple of 4 floats from CameraInfo.K
    tag_size=tag_size_param,          # meters; D-09: 0.08
)
```

Returned `Detection` objects have these fields (verified against `pupil_apriltags` 1.0.4):

| Field | Type | Use in Phase 7 |
|---|---|---|
| `tag_family` | `bytes` | Logged at startup only (filter by family is implicit since detector was constructed with `tag36h11`) |
| `tag_id` | `int` | D-07 filter: `if d.tag_id != target_tag_id: continue` |
| `hamming` | `int` | D-11 filter: `if d.hamming != 0: continue` |
| `decision_margin` | `float` | D-10 filter: `if d.decision_margin < decision_margin_min: continue`; HUD display |
| `corners` | `np.ndarray (4,2) float64` | OpenCV polygon draw + projectPoints |
| `center` | `np.ndarray (2,) float64` | Optional HUD anchor |
| `pose_R` | `np.ndarray (3,3) float64` | Tag rotation in camera-optical frame |
| `pose_t` | `np.ndarray (3,1) float64` | Tag translation in camera-optical frame (meters) |
| `pose_err` | `float` | Reprojection error; not used for filtering this phase |

**Key constraint:** `pose_R` and `pose_t` are populated **only** when `estimate_tag_pose=True` AND both `camera_params` and `tag_size` are provided. Missing any of the three leaves these attributes as `None`.

### `pose_R` / `pose_t` semantics

`pose_R @ p_tag + pose_t` maps a point from tag-local frame to camera-optical frame. This is the standard PnP convention — verified against the `pupil_apriltags` source (`apriltag_python/python/wrapper.py`) which uses OpenCV `solvePnP` internally.

**Phase 7 use:** Build a 4×4 homogeneous transform `T_cam_tag` from `pose_R`, `pose_t`. Apply offset in tag frame by composing `T_cam_target = T_cam_tag @ T_tag_target` where `T_tag_target = [[I, offset_xyz.T], [0, 1]]` (D-01). Then convert each to a `geometry_msgs/Pose` (translation + quaternion from rotation matrix) and wrap in `PoseStamped` with `frame_id="camera_color_optical_frame"`.

### Rotation → quaternion

Use `scipy.spatial.transform.Rotation.from_matrix(R).as_quat()` returning `[x, y, z, w]` order — the same order ROS uses for `geometry_msgs/Quaternion`. `scipy` is already on the system Python (numpy/scipy come with `pupil-apriltags` indirect deps and Ubuntu defaults).

Alternative without scipy: implement the standard matrix→quaternion routine inline (Shepperd's method, ~15 lines). Keep scipy approach — it is well-tested, single import, and handles edge cases (negative trace, etc.) correctly.

### Performance expectation

`pupil-apriltags` underlying C library detects 36h11 tags at ~5–10 ms per 640×480 frame on Ryzen-class CPUs. At 15 fps the node will idle most of each 67 ms cycle, leaving plenty of headroom for OpenCV draw + TF transform + publish. **No performance risk.**

## 2. cv_bridge in Python (rclpy)

### Conversion direction

```python
from cv_bridge import CvBridge
self.bridge = CvBridge()

# Image -> numpy (BGR8 default)
cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
```

`imgmsg_to_cv2` returns `numpy.ndarray` directly. With `desired_encoding="bgr8"`, the wrapper handles RealSense's native `rgb8` → `bgr8` conversion automatically. With `desired_encoding="passthrough"`, it returns whatever the source encoding is (RGB8 from RealSense color stream).

**Recommendation:** Use `"bgr8"` to keep the OpenCV imshow window's color channels correct without manual swapping.

### Sensor data QoS

RealSense publishes `/camera/color/image_raw` with `BEST_EFFORT` reliability (sensor data QoS). The subscriber MUST also use sensor QoS or it will silently drop messages.

```python
from rclpy.qos import qos_profile_sensor_data
self.create_subscription(Image, rgb_topic, self.image_cb, qos_profile_sensor_data)
```

Same applies to `CameraInfo`. Confirmed by examining `visual_detection_tester.cpp` (L66-68 uses `rmw_qos_profile_sensor_data`).

### CameraInfo K-matrix layout

`sensor_msgs/CameraInfo.k` is a row-major 9-element float64 array:

```
K = [fx  0  cx
      0 fy  cy
      0  0   1]
```

`fx = K[0]`, `fy = K[4]`, `cx = K[2]`, `cy = K[5]`. CameraInfo arrives once per frame on the color topic; cache `(fx, fy, cx, cy)` after first message and skip subsequent updates.

## 3. tf2_geometry_msgs Python registration (D-04)

### The critical import

`tf2_ros.Buffer.transform(pose_stamped, target_frame)` dispatches by message type using a registry populated at import time. For `PoseStamped` to be transformable, the package `tf2_geometry_msgs` MUST be imported (the import itself registers the `do_transform_pose` plugin via the `TransformRegistration` mechanism).

```python
import tf2_ros                       # Buffer + TransformListener
import tf2_geometry_msgs             # registers PoseStamped/Pose/PointStamped (side-effect)
from geometry_msgs.msg import PoseStamped
```

Without `import tf2_geometry_msgs`, `tf_buffer.transform(pose_stamped, ...)` raises `tf2.TypeException: ('Type %s if not loaded or supported', PoseStamped)`. **This is the most common bug in Python tf2 code.** The lint-style fix: keep the import even if not directly referenced; add `# noqa: F401` if the linter complains.

### Buffer + Listener setup

```python
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)
```

The listener spawns its own subscription on `/tf` and `/tf_static`. `Buffer()` uses default cache time (10 s).

### Transform call

```python
try:
    pose_torso = self.tf_buffer.transform(
        pose_cam, "torso_link",
        timeout=Duration(seconds=tf_lookup_timeout_s)
    )
except tf2_ros.TransformException as ex:
    self.get_logger().warn(f"TF transform failed: {ex}", throttle_duration_sec=2.0)
    return
```

`tf2_ros.TransformException` is the umbrella exception (covers `LookupException`, `ConnectivityException`, `ExtrapolationException`). The `throttle_duration_sec` argument on `get_logger().warn()` is supported in rclpy ≥ Foxy and ≥ ROS 2 Humble (which this project uses).

### TF chain availability

The complete `camera_color_optical_frame` → `torso_link` chain comes from three publishers:

1. **realsense2_camera** publishes `camera_color_frame ← camera_color_optical_frame` and `camera_link ← camera_color_frame` as part of its driver startup (around 1–2 s after the node spawns).
2. **`d435_link_to_camera_link`** static transform publisher (in apriltag.launch.py) bridges `d435_link → camera_link`.
3. **robot_state_publisher** publishes `torso_link → ... → d435_link` from the URDF (Phase 6 confirmed `torso_link` and `d435_link` are present in `g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf` L425, L512-516).

If any segment is missing, the buffer will fail with `LookupException` → caught → frame dropped (D-04 + Agent's Discretion).

## 4. RealSense launch profile changes (D-15)

### rs_launch.py argument names

`realsense2_camera/launch/rs_launch.py` exposes parameters as launch arguments via `DeclareLaunchArgument`. Confirmed against the `realsense-ros` ROS 2 Humble branch:

| Argument | Type | Purpose | Phase 7 value |
|---|---|---|---|
| `rgb_camera.profile` | string | RGB stream WxHxFPS | `640x480x15` |
| `depth_module.profile` | string | Depth stream WxHxFPS | `640x480x15` (kept consistent) |
| `align_depth.enable` | bool | Enable depth-to-color alignment | `false` (PnP only needs RGB) |
| `enable_sync` | bool | Time-sync color + depth + IMU | `true` (kept; cheap, no harm) |
| `enable_color` | bool | Enable color stream | default `true`; not set explicitly |
| `enable_depth` | bool | Enable depth stream | default `true`; depth still streams but unused by this node |

`align_depth.enable=false` saves the alignment-pass GPU/CPU work and ~30 % USB bandwidth. RGB-only PnP doesn't need aligned depth.

### Comparison with `visual_detect_click.launch.py`

That launch passes `'rgb_camera.profile': '1280x720x15'` and `'align_depth.enable': 'true'` (it uses depth for click-to-3D). Phase 7 launch flips both. All other RealSense args remain default.

## 5. OpenCV drawing primitives (D-17, D-18, D-20)

### Required cv2 calls

| Operation | Function | Notes |
|---|---|---|
| Window | `cv2.namedWindow(name, cv2.WINDOW_NORMAL)` + `cv2.resizeWindow` | NORMAL allows manual resize; AUTOSIZE locks to first frame |
| Display | `cv2.imshow(name, bgr_image)` + `cv2.waitKey(1)` | `waitKey(1)` flushes events; required every frame |
| Polygon | `cv2.polylines(img, [corners.astype(int)], True, color, 2)` | corners = (4,2) ndarray |
| Text | `cv2.putText(img, text, (x,y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)` | LINE_AA for smooth edges |
| Project axes | `cv2.projectPoints(obj_pts, rvec, tvec, K, dist)` → returns `(N,1,2)` float | Use `cv2.Rodrigues(R)` to get rvec |
| Line | `cv2.line(img, p1, p2, color, 2)` | Use rounded int tuples |
| Close | `cv2.destroyWindow(name)` or `cv2.destroyAllWindows()` | Best-effort in node destructor |

### Quit-on-q pattern

```python
key = cv2.waitKey(1) & 0xFF
if key == ord('q'):
    self.window_open = False
    cv2.destroyWindow(self.window_name)
```

Setting `self.window_open = False` ensures subsequent frames skip imshow. The node continues to publish PoseStamped messages — only the GUI is gone. Ctrl+C still terminates the process via the standard rclpy KeyboardInterrupt path.

### Distortion vector for projectPoints

RealSense color stream typically reports `D = [0,0,0,0,0]` (rectified). Pass `np.zeros(5)` as the `distCoeffs` argument. If `CameraInfo.d` is non-empty and non-zero, use it directly: `np.array(camera_info.d, dtype=np.float64)`.

### FPS HUD via sliding window

```python
import collections, time

self.frame_times = collections.deque(maxlen=30)

def image_cb(self, msg):
    now = time.monotonic()
    self.frame_times.append(now)
    if len(self.frame_times) >= 2:
        fps = (len(self.frame_times) - 1) / (self.frame_times[-1] - self.frame_times[0])
    else:
        fps = 0.0
    # ... draw HUD with fps
```

Sliding window over the last 30 frames smooths instantaneous jitter while still reflecting USB-bandwidth drops within ~2 seconds.

## 6. CMakeLists.txt + package.xml mechanics (D-14)

### What needs to change

Both files require **purely additive** changes — no removals.

**CMakeLists.txt:**

1. Add to `install(PROGRAMS ...)`:
   ```cmake
   scripts/apriltag_detector_node.py
   ```
2. Add a new install block for `config/`:
   ```cmake
   install(DIRECTORY
     config
     DESTINATION share/${PROJECT_NAME}
   )
   ```
   The existing `install(DIRECTORY launch robots ...)` block stays as-is.

**package.xml:**

Add new `<exec_depend>` entries (these declare *runtime* dependencies for ROS launches and `ros2 run`, since the script is Python):

```xml
<exec_depend>realsense2_camera</exec_depend>
<exec_depend>python3-opencv</exec_depend>
<!-- tf2_ros, tf2_geometry_msgs, cv_bridge are already <depend> in the file (Phase 5/6) -->
```

`tf2_ros`, `tf2_geometry_msgs`, `cv_bridge` are already `<depend>` in the file from prior phases — confirmed via `grep "<depend>" src/unitree_g1_dex3_stack-main/package.xml`. No need to add.

`pupil-apriltags` is intentionally NOT added — it's a pip package not in rosdep. Document the install hint in README.

### What does NOT need to change

- `find_package` calls — Python node doesn't compile against C++ libraries
- `add_executable` / `target_link_libraries` — N/A for Python scripts
- `BUILD_IK_FCL_OMPL_PLANNER` block — unaffected

## 7. Independent launch (D-15) — concrete structure

`launch/apriltag.launch.py` skeleton based on `visual_detect_click.launch.py` (the closest analog):

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share = get_package_share_directory('unitree_g1_dex3_stack')
    realsense_share = get_package_share_directory('realsense2_camera')

    urdf_name_arg = DeclareLaunchArgument('urdf_name',
        default_value='g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf')
    urdf_path_arg = DeclareLaunchArgument('urdf_path', default_value='')

    config_default = os.path.join(package_share, 'config', 'apriltag.yaml')
    config_file_arg = DeclareLaunchArgument('config_file', default_value=config_default)
    imshow_arg = DeclareLaunchArgument('imshow', default_value='true')

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(package_share, 'launch', 'robot.launch.py')),
        launch_arguments={
            'urdf_name': LaunchConfiguration('urdf_name'),
            'urdf_path': LaunchConfiguration('urdf_path'),
        }.items()
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_share, 'launch', 'rs_launch.py')),
        launch_arguments={
            'enable_sync': 'true',
            'align_depth.enable': 'false',
            'rgb_camera.profile': '640x480x15',
            'depth_module.profile': '640x480x15',
        }.items()
    )

    d435_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='d435_link_to_camera_link',
        arguments=['0', '0', '0', '0', '0', '0', 'd435_link', 'camera_link'],
    )

    apriltag_node = Node(
        package='unitree_g1_dex3_stack',
        executable='apriltag_detector_node.py',
        name='apriltag_detector',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config_file'), {
            'imshow': LaunchConfiguration('imshow'),
        }],
    )

    return LaunchDescription([
        urdf_name_arg, urdf_path_arg, config_file_arg, imshow_arg,
        robot_launch, realsense_launch, d435_to_camera_link, apriltag_node,
    ])
```

Key choices:
- `imshow` is added to the `parameters=[...]` list as a node parameter even though CONTEXT D-15 calls it a "launch arg." The launch arg controls the value; the node receives it via `declare_parameter("imshow", True)` at startup. This is the standard ROS 2 idiom for "launch-time switch" — it stays in the YAML/parameter universe rather than e.g. `IfCondition` over the whole node (which would prevent the node from publishing topics in headless deployments). Both work; parameter approach is simpler and matches the `visual_detection_tester.cpp` convention of accepting display state as a parameter.
- `urdf_name` defaults to the **collision primitives** URDF (matches `robot.launch.py`'s default and `reach.launch.py` runtime usage). Phase 6 added `right_tcp_link` to both URDFs but the collision primitives URDF is the one actually loaded on the robot; using the same default keeps `apriltag.launch.py` consistent with `reach.launch.py`.

## 8. RGB image encoding sanity check

RealSense color stream publishes `Image.encoding == "rgb8"` by default. cv_bridge handles `rgb8 → bgr8` automatically when `desired_encoding="bgr8"` is requested — no manual `cv2.cvtColor(BGR2RGB)` or vice versa needed.

For grayscale conversion (input to `Detector.detect`):

```python
gray = cv2.cvtColor(cv_image_bgr, cv2.COLOR_BGR2GRAY)
```

`cv2.COLOR_RGB2GRAY` and `cv2.COLOR_BGR2GRAY` both produce identical grayscale output up to numerical noise (the conversion uses ITU-R BT.601 weights symmetrically); using BGR2GRAY is fine.

## 9. Risk Assessment

### Low Risk

- **pupil-apriltags API stability** — version 1.0.4 has been stable since 2020; the public API used here (`Detector(families=...).detect(...)`) has not changed.
- **OpenCV imshow + rclpy single-thread executor** — Both `cv2.imshow` and `cv2.waitKey(1)` are non-blocking from Python's perspective and run on the executor's thread. Confirmed by `visual_detection_tester.cpp` doing the same C++-side without trouble. Single-threaded executor is the rclpy default.
- **CameraInfo arrival timing** — `realsense2_camera` publishes CameraInfo on every color frame (i.e., 15 Hz), so first cache happens within ~67 ms of first frame. No race risk.
- **`align_depth.enable=false`** — Doesn't affect any Phase 7 code path. Phase 9 may need depth alignment back; that's a future concern.

### Medium Risk

- **`tf2_geometry_msgs` import omission** — Most likely failure mode if implementation drifts from research. Mitigation: explicit grep-based acceptance criterion in plan: `grep "import tf2_geometry_msgs" scripts/apriltag_detector_node.py` returns a match.
- **TF chain not yet established at first frame** — On startup, RealSense, robot_state_publisher, and the static publisher take 1–5 s to fully populate `/tf`. First few frames will hit `LookupException`. Mitigation: the plan's filter logic catches `tf2_ros.TransformException` and silently drops the frame with throttled warn (D-04). After ~5 s the transform succeeds.
- **`pupil-apriltags` not on system Python path** — User has confirmed `pip install pupil-apriltags`. If the install used `--user`, the path is `~/.local/lib/python3.X/site-packages` which Ubuntu adds to default user `sys.path`. If `pip` was system-wide via `sudo pip install`, the path is `/usr/local/lib/python3.X/dist-packages`, also default. Either way, `from pupil_apriltags import Detector` works without `PYTHONPATH` munging.

### Edge Cases

- **No tag detected for many frames** — The node runs detection every frame but only publishes on filtered hits. A long quiet period is normal (camera not pointed at tag) and produces no log spam.
- **Multiple tag36h11 in view, only one matches `target_tag_id`** — D-07 filters to single ID; non-matching tags are visualized in red (rejected) with their ID drawn so the user can see what's around.
- **Tag partially occluded** — `pupil-apriltags` may still detect with low `decision_margin` (<10), filtered out by D-10. Visualization shows red polygon → user adjusts pose.
- **`imshow=false` on headless SSH** — Window creation is skipped; node logs at startup that GUI is disabled. Topics still publish correctly.
- **`offset_xyz=[0,0,0]`** — Valid degenerate case; `target_pose == tag_pose`. Both topics still publish; downstream user sees identical messages.
- **Scientific notation in YAML** — ROS 2 parameter loader handles `1.0e-2` correctly; not a concern for our default values but documented for future tuning.

## 10. Validation Architecture

Each requirement maps to discrete observable assertions checkable from a running launch.

### TAG-01: AprilTag 检测节点发布 6-DOF 位姿

- **Build:** `colcon build --packages-select unitree_g1_dex3_stack` exits 0
- **Install presence:** `ros2 pkg executables unitree_g1_dex3_stack | grep apriltag_detector_node.py` returns a match
- **Source presence:** `grep "from pupil_apriltags import Detector" src/unitree_g1_dex3_stack-main/scripts/apriltag_detector_node.py` returns a match
- **Source presence:** `grep "estimate_tag_pose=True" src/unitree_g1_dex3_stack-main/scripts/apriltag_detector_node.py` returns a match
- **Topic publication (live):** `ros2 launch unitree_g1_dex3_stack apriltag.launch.py imshow:=false` + place tag in view + `ros2 topic echo /apriltag/tag_pose --once` returns a `geometry_msgs/PoseStamped` with `header.frame_id=='torso_link'` and non-zero `pose.position` + non-identity `pose.orientation`
- **Topic publication (live):** `ros2 topic echo /apriltag/target_pose --once` returns a `geometry_msgs/PoseStamped` with `header.frame_id=='torso_link'`
- **Topic discoverability:** `ros2 topic list | grep -E "/apriltag/(tag|target)_pose"` returns both topics

### TAG-02: 可配置 tag→物品 XYZ 偏移 via YAML

- **YAML presence:** `test -f src/unitree_g1_dex3_stack-main/config/apriltag.yaml` exits 0
- **YAML schema (all 11 fields from D-08 present):** `grep -E "(tag_family|tag_size|target_tag_id|offset_xyz|decision_margin_min|output_frame|rgb_topic|camera_info_topic|tag_pose_topic|target_pose_topic|tf_lookup_timeout_s):" src/unitree_g1_dex3_stack-main/config/apriltag.yaml | wc -l` returns 11
- **YAML installed:** `test -f install/unitree_g1_dex3_stack/share/unitree_g1_dex3_stack/config/apriltag.yaml` exits 0 (after build + install)
- **Param loading:** `ros2 param get /apriltag_detector offset_xyz` returns `[0.0, 0.0, 0.05]` (default placeholder)
- **Override demonstration:** Edit YAML to `offset_xyz: [0.10, 0.0, 0.0]`, rebuild config (`cp src/.../config/apriltag.yaml install/.../share/.../config/apriltag.yaml`), relaunch; `target_pose - tag_pose` differs by ~0.10 m along the tag's local X axis (visible in `ros2 topic echo` and OpenCV three-axis projection)
- **Source — offset is read from parameter, not hardcoded:** `grep -E "self.declare_parameter\(['\"](offset_xyz|tag_size|target_tag_id|decision_margin_min)['\"]" src/unitree_g1_dex3_stack-main/scripts/apriltag_detector_node.py | wc -l` returns 4

### TAG-03: 位姿通过 TF 从 camera_color_optical_frame 变换到 torso_link

- **Source — TF buffer + listener:** `grep -E "(from tf2_ros import|tf2_ros.Buffer\(\)|TransformListener)" src/unitree_g1_dex3_stack-main/scripts/apriltag_detector_node.py | wc -l` ≥ 3
- **Source — `tf2_geometry_msgs` registered:** `grep "import tf2_geometry_msgs" src/unitree_g1_dex3_stack-main/scripts/apriltag_detector_node.py` returns a match
- **Source — `tf_buffer.transform(... 'torso_link' ...)` invoked:** `grep -E "tf_buffer\.transform.*output_frame|tf_buffer\.transform.*torso_link" src/unitree_g1_dex3_stack-main/scripts/apriltag_detector_node.py` returns a match (parameter or literal)
- **Source — TransformException handled:** `grep "tf2_ros.TransformException\|TransformException" src/unitree_g1_dex3_stack-main/scripts/apriltag_detector_node.py` returns a match
- **Live frame check:** `ros2 topic echo /apriltag/tag_pose --once` shows `header.frame_id == "torso_link"` (NOT `camera_color_optical_frame`)
- **TF chain check:** `ros2 run tf2_ros tf2_echo torso_link camera_color_optical_frame` returns a non-error transform within 5 s of launch

### TAG-04: 低质量检测被过滤

- **Source — three-stage filter present:** Each of these greps returns a match:
  - `grep "decision_margin" src/unitree_g1_dex3_stack-main/scripts/apriltag_detector_node.py`
  - `grep -E "hamming\s*[!=]" src/unitree_g1_dex3_stack-main/scripts/apriltag_detector_node.py` (hamming != 0 filter)
  - `grep -E "tag_id\s*[!=]" src/unitree_g1_dex3_stack-main/scripts/apriltag_detector_node.py` (target_tag_id filter)
- **Source — decision_margin is parameter-driven, not hardcoded:** `grep -E "self\.decision_margin_min|self\.get_parameter\(['\"]decision_margin_min" src/unitree_g1_dex3_stack-main/scripts/apriltag_detector_node.py` returns a match (declared via parameter)
- **Live behavior — no publish without filtered detection:** With camera pointed away from any tag, `timeout 3 ros2 topic echo /apriltag/tag_pose --once` exits non-zero (no message arrives) — confirms event-driven publishing (D-03)
- **Live behavior — visual feedback:** With tag visible, OpenCV window shows polygon; if margin drops below threshold (e.g., far/blurry tag), polygon turns red — confirms filter integrated into viz (D-18 #1)

## RESEARCH COMPLETE
