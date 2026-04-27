# Structure

## Directory Layout

```
unitree_dex3/                           # Workspace root (colcon workspace)
├── best.pt                             # Trained YOLO model weights (~20MB)
├── run_perception.sh                   # Shell script: conda activate + launch perception
├── .windsurfrules                      # IDE coding guidelines
├── build/                              # colcon build output
├── install/                            # colcon install output (runtime)
├── log/                                # colcon build logs
└── src/                                # Source packages
    ├── unitree_g1_dex3_stack-main/      # Main ROS 2 package
    │   ├── CMakeLists.txt              # Build config (conditionally builds planner)
    │   ├── package.xml                 # ROS 2 package manifest
    │   ├── README.md                   # Project documentation
    │   ├── include/
    │   │   └── g1_dex3_joint_defs.hpp  # Joint enums and name→index maps
    │   ├── src/
    │   │   ├── joint_state_publisher.cpp        # 150 lines
    │   │   ├── dex3_controller.cpp              # 540 lines
    │   │   ├── joint_trajectory_executor.cpp    # 231 lines
    │   │   ├── ik_fcl_ompl_planner.cpp          # 903 lines
    │   │   ├── project_to_3d_node.cpp           # 443 lines
    │   │   ├── detection_to_goal_node.cpp       # 56 lines
    │   │   ├── right_hand_pressure_monitor.cpp  # 82 lines
    │   │   ├── visual_detection_tester.cpp      # ~270 lines
    │   │   └── visual_detection_yolo_tester.cpp # 266 lines
    │   ├── scripts/
    │   │   └── ultralytics_detector.py          # 96 lines, YOLO ROS 2 node
    │   ├── launch/
    │   │   ├── robot.launch.py                  # Robot model + joint state pub
    │   │   ├── perception.launch.py             # RealSense + YOLO + 3D projection
    │   │   ├── planner.launch.py                # OMPL+FCL+IK motion planner
    │   │   ├── control.launch.py                # Trajectory executor + hand controllers
    │   │   ├── visual_detect_click.launch.py    # Click-based detection testing
    │   │   └── visual_detect_yolo.launch.py     # YOLO detection testing
    │   ├── robots/
    │   │   ├── g1_description/                  # G1 robot URDF + meshes (158 items)
    │   │   │   ├── g1_29dof_lock_waist_with_hand_rev_1_0.urdf  # Default
    │   │   │   ├── g1_29dof_with_hand_rev_1_0.urdf
    │   │   │   └── g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf
    │   │   └── dexterous_hand_description/      # DEX3 hand meshes (18 items)
    │   └── rviz_config/                         # RViz display configuration
    ├── bboxes_ex_msgs/                  # Custom bounding box message package
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── msg/
    │       ├── BoundingBox.msg          # Single detection bbox
    │       └── BoundingBoxes.msg        # Array of bboxes with header
    ├── fcl/                             # Vendored FCL collision library (565 items)
    └── trac_ik/                         # Vendored TRAC-IK solver (45 items)
        ├── trac_ik_lib/
        ├── trac_ik_kinematics_plugin/
        ├── trac_ik_python/
        └── trac_ik_examples/
```

## Key Locations

- **Node source code**: `src/unitree_g1_dex3_stack-main/src/`
- **Launch files**: `src/unitree_g1_dex3_stack-main/launch/`
- **Robot models**: `src/unitree_g1_dex3_stack-main/robots/`
- **Joint definitions header**: `src/unitree_g1_dex3_stack-main/include/g1_dex3_joint_defs.hpp`
- **YOLO model weights**: `best.pt` (workspace root)
- **Python detector**: `src/unitree_g1_dex3_stack-main/scripts/ultralytics_detector.py`

## Naming Conventions

- Source files: `snake_case.cpp`
- ROS 2 node names: `snake_case`
- Launch files: `snake_case.launch.py`
- Topics: `/snake_case` with namespace prefixes (`/dex3/`, `/yolo/`, `/lf/`)
- C++ enums: `kPascalCase` (e.g., `kLeftShoulderPitch`)
- Message packages: `snake_case_msgs`
