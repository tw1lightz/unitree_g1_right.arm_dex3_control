# Concerns

## Technical Debt

### Duplicated URDF Loading Pattern
The same URDF-from-service-parameter loading code is copy-pasted across 4 nodes (`joint_state_publisher.cpp`, `dex3_controller.cpp`, `joint_trajectory_executor.cpp`, `ik_fcl_ompl_planner.cpp`). Each has slightly different timeout/retry behavior. Could be extracted to a shared utility.

### Inconsistent Service Wait Behavior
- `joint_state_publisher` — 5 retry max, then shutdown
- `dex3_controller` — 25 retries (5s), then shutdown
- `joint_trajectory_executor` — Waits indefinitely
- `ik_fcl_ompl_planner` — Waits indefinitely

### Hardcoded Paths
- `perception.launch.py` defaults model path to `/home/unitree/Desktop/unitree_dex3/best.pt`
- `visual_detect_yolo.launch.py` also hardcodes the same path
- These work on the target robot but break portability

### Planner Build Flag
- `BUILD_IK_FCL_OMPL_PLANNER` is OFF by default in CMakeLists.txt, meaning the planner node is not built unless explicitly enabled. This may confuse users who expect it to be available.

### JointLimits Struct Redefined
`JointLimits` struct is defined identically in both `dex3_controller.cpp` and `joint_trajectory_executor.cpp`. Should be in a shared header.

## Security

- No authentication or access control on ROS 2 topics (standard for research robotics)
- No secrets or API keys in the codebase
- YOLO model is a local file, not downloaded at runtime

## Performance

- Camera runs at 1280x720 @ 15fps (perception limited by this rate)
- OMPL planning timeout is configurable (default 1.0s), may need tuning for complex scenes
- `project_to_3d_node` uses PCL statistical outlier removal which can be CPU-intensive
- YOLO inference runs in Python (potentially slower than C++ inference)

## Fragile Areas

- **Node startup ordering**: Control nodes block on `/robot_state_publisher` service. If `robot.launch.py` isn't launched first, nodes hang or crash. The `perception.launch.py` uses a 5-second `TimerAction` delay for the RealSense to mitigate startup race.
- **Tactile feedback grasping**: `dex3_controller` uses a threshold-based closed-loop grasp. Threshold tuning (`tactile_threshold` param, default 10.2) is critical and hardware-dependent.
- **TF frame assumptions**: Nodes assume specific frame names (`torso_link`, `camera_color_optical_frame`, `d435_link`). Changing the URDF or camera setup may break transforms.
- **Conda environment dependency**: `run_perception.sh` assumes the `grab` conda environment exists with Ultralytics installed. No requirements.txt or environment.yml provided.

## Missing Capabilities

- No automated tests
- No CI/CD pipeline
- No simulation support (e.g., Gazebo)
- No documentation beyond README.md
- No error recovery / fault tolerance at the system level
- No recording / playback infrastructure (rosbag integration)
