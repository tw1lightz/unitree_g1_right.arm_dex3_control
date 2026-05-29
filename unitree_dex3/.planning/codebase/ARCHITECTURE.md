# Architecture

## Pattern

ROS 2 node-based architecture with modular, loosely-coupled nodes communicating via topics. The system follows a **perception → planning → control** pipeline pattern for tabletop object manipulation.

## Layers

### 1. Perception Layer
Detects objects in 2D, projects to 3D, and selects targets.

- `ultralytics_detector` (Python) — Runs YOLO inference on RGB images, publishes 2D bounding boxes
- `project_to_3d_node` (C++) — Fuses 2D detections with depth images to produce 3D point clouds and `Detection3DArray` messages. Uses PCL for outlier removal and centroid computation. Supports TF2 frame transformation.
- `detection_to_goal_node` (C++) — Converts user-selected 3D detection into a `PoseStamped` goal for the planner

### 2. Planning Layer
Plans collision-free arm trajectories to target poses.

- `ik_fcl_ompl_planner` (C++) — The most complex node (~900 lines). Uses:
  - TRAC-IK for inverse kinematics
  - FCL for self-collision checking (loads URDF collision meshes)
  - OMPL (RRTConnect) for motion planning in joint space
  - TF2 for frame transforms between camera and robot frames
  - Outputs `JointTrajectory` messages

### 3. Control Layer
Executes trajectories and controls the dexterous hands.

- `joint_trajectory_executor` (C++) — Receives `JointTrajectory`, sends motor commands via `LowCmd` to `/arm_sdk`. Coordinates hand open/close via Bool commands to hand controllers. Clamps commands to URDF joint limits.
- `dex3_controller` (C++) — Controls one DEX3 hand (left or right). Implements closed-loop tactile feedback grasping. Reads pressure sensors to detect grasp success. Parameterized side and tactile threshold.

### 4. State Layer
Publishes robot state for visualization and feedback.

- `joint_state_publisher` (C++) — Reads `LowState` and `HandState`, publishes unified `JointState` for RViz and other consumers. Dynamically maps joint names from URDF.
- `robot_state_publisher` (standard ROS 2) — Publishes TF tree from URDF

### 5. Diagnostics / Testing
- `right_hand_pressure_monitor` (C++) — Logs pressure sensor values for debugging
- `visual_detection_tester` (C++) — Interactive click-based detection testing
- `visual_detection_yolo_tester` (C++) — YOLO-based detection testing with keyboard input, TF2 transform display

## Data Flow

```
Camera (RealSense D435)
    ↓ RGB + Depth images
ultralytics_detector (YOLO)
    ↓ 2D bounding boxes
project_to_3d_node
    ↓ 3D detections (Detection3DArray)
detection_to_goal_node
    ↓ PoseStamped (user-selected target)
ik_fcl_ompl_planner
    ↓ JointTrajectory (collision-free path)
joint_trajectory_executor
    ↓ LowCmd (motor commands) + Bool (hand commands)
    ├── Unitree G1 arm motors
    └── dex3_controller (left/right)
            ↓ HandCmd (finger positions)
            DEX3 hands (tactile feedback loop)
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
