# Integrations

## Hardware

### Unitree G1 Humanoid Robot
- Communication via `unitree_hg` custom ROS 2 messages
- Low-level motor commands published to `/arm_sdk` topic (`LowCmd`)
- Robot state received from `/lf/lowstate` topic (`LowState`)
- DEX3 hand commands to `/dex3/{left,right}/cmd` (`HandCmd`)
- Hand state feedback from `/lf/dex3/{left,right}/state` (`HandState`)
- Pressure sensor data in `PressSensorState` within `HandState`

### Intel RealSense D435 Camera
- Launched via `realsense2_camera` package (`rs_launch.py`)
- RGB stream: `/camera/color/image_raw` (1280x720 @ 15fps)
- Depth stream: `/camera/aligned_depth_to_color/image_raw` (1280x720 @ 15fps)
- Camera intrinsics: `/camera/color/camera_info`
- Static TF published between `d435_link` and `camera_link` in test launch files

## ROS 2 Topic Architecture

### Perception Pipeline
| Topic | Type | Direction | Node |
|-------|------|-----------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | Input | `ultralytics_detector` |
| `/yolo/bounding_boxes` | `bboxes_ex_msgs/BoundingBoxes` | Output | `ultralytics_detector` |
| `/yolo/image_raw` | `sensor_msgs/Image` | Output | `ultralytics_detector` |
| `/objects_3d` | `sensor_msgs/PointCloud2` | Output | `project_to_3d_node` |
| `/detections_3d` | `vision_msgs/Detection3DArray` | Output | `project_to_3d_node` |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Output | `detection_to_goal_node` |
| `/detection_selection` | `std_msgs/String` | Input | `detection_to_goal_node` |

### Control Pipeline
| Topic | Type | Direction | Node |
|-------|------|-----------|------|
| `/arm_sdk` | `unitree_hg/LowCmd` | Output | `joint_trajectory_executor` |
| `/lf/lowstate` | `unitree_hg/LowState` | Input | `joint_trajectory_executor` |
| `/joint_trajectory_targets` | `trajectory_msgs/JointTrajectory` | Input | `joint_trajectory_executor` |
| `/dex3/{side}/command` | `std_msgs/Bool` | Bridge | `joint_trajectory_executor` → `dex3_controller` |
| `/dex3/{side}/cmd` | `unitree_hg/HandCmd` | Output | `dex3_controller` |
| `/joint_states` | `sensor_msgs/JointState` | Output | `joint_state_publisher` |

### Planning
| Topic | Type | Direction | Node |
|-------|------|-----------|------|
| `/goal_pose` | `geometry_msgs/PoseStamped` | Input | `ik_fcl_ompl_planner` |
| `/joint_trajectory_targets` | `trajectory_msgs/JointTrajectory` | Output | `ik_fcl_ompl_planner` |
| `/joint_states` | `sensor_msgs/JointState` | Input | `ik_fcl_ompl_planner` |

## External Services

- **Robot State Publisher** (`/robot_state_publisher`) — All control nodes fetch `robot_description` URDF parameter from this service at startup
- No external APIs, databases, or cloud services

## Custom Messages

### `bboxes_ex_msgs`
- `BoundingBox.msg`: `probability`, `xmin`, `ymin`, `xmax`, `ymax`, `class_id`
- `BoundingBoxes.msg`: `header` + array of `BoundingBox`
