<!-- GSD:project-start source:PROJECT.md -->
## Project

**Unitree G1 Right-Arm Safe Reach**

A safe, stable right-arm reaching system for the Unitree G1 humanoid robot. The robot stands in place (official running mode maintains balance), detects objects via the existing YOLO + RealSense perception pipeline, transforms coordinates from camera frame to robot frame, and uses OMPL + FCL + TRAC-IK to plan and execute a collision-free trajectory for the right arm only to reach the target position. No dexterous hand control — the task is complete when the right arm end-effector arrives at the target.

**Core Value:** The right arm moves safely to the target position without colliding with the robot's own body or the environment, and without exceeding joint limits.

### Constraints

- **Hardware**: Must run on G1 onboard computer (limited compute)
- **Safety**: All four safety aspects required — self-collision, environment collision, joint limits, trajectory smoothness
- **Coexistence**: Right arm control must coexist with official running mode without interference
- **Real-time**: Planning and execution must be fast enough for practical use (planner timeout configurable, default 1.0s)
- **Right arm only**: Only joints from `right_shoulder_pitch` to `right_wrist_yaw` (7 DOF) are controlled
<!-- GSD:project-end -->

<!-- GSD:stack-start source:codebase/STACK.md -->
## Technology Stack

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
<!-- GSD:stack-end -->

<!-- GSD:conventions-start source:CONVENTIONS.md -->
## Conventions

## Code Style
### C++
- **Standard**: C++17
- **Compiler flags**: `-Wall -Wextra -Wpedantic`
- Classes use PascalCase (`Dex3Controller`, `IKFCLPlannerNode`)
- Member variables use trailing underscore (`hand_cmd_pub_`, `side`, `tactile_threshold_`)
- Methods use camelCase (`handCmdCallback`, `lowstateCallback`)
- Enums use `kPascalCase` values (`kLeftShoulderPitch`, `kThumb0`)
- `using namespace std::chrono_literals` is used in control nodes
### Python
- Standard Python 3 style
- ROS 2 node class inherits from `rclpy.node.Node`
- Parameters declared with `self.declare_parameter()`
## Patterns
### URDF-at-Runtime
### QoS Best-Effort
### Side-Parameterized Nodes
### Launch File Pattern
## Error Handling
- Fatal errors (missing URDF, service unavailable) call `RCLCPP_FATAL()` then `rclcpp::shutdown()`
- Service wait timeouts vary: some nodes wait indefinitely, others timeout after N retries
- Joint commands are clamped to URDF limits before sending to hardware
- Python detector wraps `rclpy.spin()` in try/except for clean KeyboardInterrupt shutdown
## Logging
- Uses ROS 2 logging: `RCLCPP_INFO`, `RCLCPP_WARN`, `RCLCPP_ERROR`, `RCLCPP_FATAL`
- Throttled warnings for service wait loops: `RCLCPP_WARN_THROTTLE`
- No custom logging framework
<!-- GSD:conventions-end -->

<!-- GSD:architecture-start source:ARCHITECTURE.md -->
## Architecture

## Pattern
## Layers
### 1. Perception Layer
- `ultralytics_detector` (Python) — Runs YOLO inference on RGB images, publishes 2D bounding boxes
- `project_to_3d_node` (C++) — Fuses 2D detections with depth images to produce 3D point clouds and `Detection3DArray` messages. Uses PCL for outlier removal and centroid computation. Supports TF2 frame transformation.
- `detection_to_goal_node` (C++) — Converts user-selected 3D detection into a `PoseStamped` goal for the planner
### 2. Planning Layer
- `ik_fcl_ompl_planner` (C++) — The most complex node (~900 lines). Uses:
### 3. Control Layer
- `joint_trajectory_executor` (C++) — Receives `JointTrajectory`, sends motor commands via `LowCmd` to `/arm_sdk`. Coordinates hand open/close via Bool commands to hand controllers. Clamps commands to URDF joint limits.
- `dex3_controller` (C++) — Controls one DEX3 hand (left or right). Implements closed-loop tactile feedback grasping. Reads pressure sensors to detect grasp success. Parameterized side and tactile threshold.
### 4. State Layer
- `joint_state_publisher` (C++) — Reads `LowState` and `HandState`, publishes unified `JointState` for RViz and other consumers. Dynamically maps joint names from URDF.
- `robot_state_publisher` (standard ROS 2) — Publishes TF tree from URDF
### 5. Diagnostics / Testing
- `right_hand_pressure_monitor` (C++) — Logs pressure sensor values for debugging
- `visual_detection_tester` (C++) — Interactive click-based detection testing
- `visual_detection_yolo_tester` (C++) — YOLO-based detection testing with keyboard input, TF2 transform display
## Data Flow
```
```
## Key Abstractions
- **URDF-driven configuration**: All nodes extract joint names, limits, and kinematic chains from the URDF at runtime. No hardcoded joint lists in control code (joint definitions in `g1_dex3_joint_defs.hpp` are for index mapping only).
- **Side-parameterized hand control**: `dex3_controller` is instantiated twice (left/right) with a `side` parameter.
- **Configurable perception**: Allowed object classes, model path, topics all parameterized in launch files.
## Entry Points
- `ros2 launch unitree_g1_dex3_stack robot.launch.py` — Robot model + joint states
- `ros2 launch unitree_g1_dex3_stack perception.launch.py` — Camera + YOLO + 3D projection
- `ros2 launch unitree_g1_dex3_stack planner.launch.py` — Motion planner
- `ros2 launch unitree_g1_dex3_stack control.launch.py` — Trajectory executor + hand controllers
- `run_perception.sh` — Convenience script to activate conda + source workspace + launch perception
<!-- GSD:architecture-end -->

<!-- GSD:skills-start source:skills/ -->
## Project Skills

No project skills found. Add skills to any of: `.claude/skills/`, `.agents/skills/`, `.cursor/skills/`, or `.github/skills/` with a `SKILL.md` index file.
<!-- GSD:skills-end -->

<!-- GSD:workflow-start source:GSD defaults -->
## GSD Workflow Enforcement

Before using Edit, Write, or other file-changing tools, start work through a GSD command so planning artifacts and execution context stay in sync.

Use these entry points:
- `/gsd-quick` for small fixes, doc updates, and ad-hoc tasks
- `/gsd-debug` for investigation and bug fixing
- `/gsd-execute-phase` for planned phase work

Do not make direct repo edits outside a GSD workflow unless the user explicitly asks to bypass it.
<!-- GSD:workflow-end -->



<!-- GSD:profile-start -->
## Developer Profile

> Profile not yet configured. Run `/gsd-profile-user` to generate your developer profile.
> This section is managed by `generate-claude-profile` -- do not edit manually.
<!-- GSD:profile-end -->
