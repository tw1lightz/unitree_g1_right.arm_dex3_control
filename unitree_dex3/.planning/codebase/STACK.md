# Stack

## Languages

- **C++17** — Primary language for all ROS 2 nodes (joint state publisher, hand controller, trajectory executor, motion planner, perception nodes)
- **Python 3** — Used for the Ultralytics YOLO detector node (`scripts/ultralytics_detector.py`) and all launch files
- **CMake** — Build system via `ament_cmake`

## Runtime

- **ROS 2** (Foxy or later) — Middleware framework for all inter-node communication
- **Ubuntu Linux** — Target OS (runs on Unitree G1 onboard computer at `/home/unitree/`)
- **Conda** — Python environment manager (environment name: `grab`, used for perception pipeline)

## Frameworks & Libraries

### ROS 2 Core
- `rclcpp` — C++ ROS 2 client library
- `rclpy` — Python ROS 2 client library (used by detector node)
- `ament_cmake` — Build tool for C++ packages
- `rosidl_default_generators` — Message generation for custom `bboxes_ex_msgs`

### Robot Control
- `unitree_hg` — Unitree custom messages (`HandCmd`, `HandState`, `MotorCmd`, `MotorState`, `LowCmd`, `LowState`, `PressSensorState`)
- `urdf` / `kdl_parser` — URDF parsing and KDL kinematic tree construction

### Motion Planning
- **OMPL** — Open Motion Planning Library (RRTConnect planner)
- **FCL** — Flexible Collision Library (vendored in `src/fcl/`, built from source)
- **TRAC-IK** — Inverse kinematics solver (vendored in `src/trac_ik/`, built from source)
- `geometric_shapes` — Shape representation for collision geometry
- `resource_retriever` — URI-based mesh loading for collision models

### Perception
- **Ultralytics YOLO** — Object detection (Python, custom-trained model `best.pt`)
- **OpenCV** — Image processing, visualization
- **PCL** (Point Cloud Library) — 3D point cloud processing, outlier removal, centroid computation
- `cv_bridge` — ROS ↔ OpenCV image conversion
- `image_transport` / `message_filters` — Synchronized image subscription
- `vision_msgs` — Standard ROS 2 3D detection messages (`Detection3DArray`)
- **Intel RealSense** (`realsense2_camera`) — RGB-D camera driver

### TF2
- `tf2`, `tf2_ros`, `tf2_geometry_msgs` — Coordinate frame transforms

## Dependencies (Vendored)

- `src/fcl/` — FCL collision library (full source, built with colcon)
- `src/trac_ik/` — TRAC-IK solver suite (includes `trac_ik_lib`, `trac_ik_kinematics_plugin`, `trac_ik_python`, `trac_ik_examples`)

## Build System

- **colcon** — ROS 2 workspace build tool
- Build artifacts in `build/` and `install/` directories
- `BUILD_IK_FCL_OMPL_PLANNER` CMake option (OFF by default) gates the planner node build
- Packages built: `bboxes_ex_msgs`, `fcl`, `trac_ik`, `trac_ik_lib`, `trac_ik_kinematics_plugin`, `unitree_g1_dex3_stack`

## Configuration

- Robot model configured via URDF files in `src/unitree_g1_dex3_stack-main/robots/g1_description/`
- Default URDF: `g1_29dof_lock_waist_with_hand_rev_1_0.urdf`
- YOLO model weights: `best.pt` (20MB, at workspace root)
- All node parameters are ROS 2 parameters declared in code and configurable via launch files
