# Phase 6: YOLO 清理 + TCP Offset 集成 — Patterns

**Mapped:** 2026-05-15

## File Map

| File | Role | Action | Analog |
|------|------|--------|--------|
| `src/unitree_g1_dex3_stack-main/CMakeLists.txt` | Build config | MODIFY — remove YOLO targets, `bboxes_ex_msgs` dep, PCL/image_transport deps | Self (existing targets pattern) |
| `src/unitree_g1_dex3_stack-main/package.xml` | Package manifest | MODIFY — remove `bboxes_ex_msgs`, `image_transport`, `pcl_conversions`, `pcl_msgs` | Self (existing `<depend>` pattern) |
| `src/unitree_g1_dex3_stack-main/launch/planner.launch.py` | Launch config | MODIFY — `right_tip` default → `right_tcp_link`, add `tcp_offset_x` param, remove detection params | Self (existing `DeclareLaunchArgument` + `OpaqueFunction` pattern) |
| `src/unitree_g1_dex3_stack-main/launch/reach.launch.py` | Launch orchestrator | MODIFY — remove YOLO args/nodes, keep robot+d435_tf+planner+control | Self (existing `IncludeLaunchDescription` + `TimerAction` pattern) |
| `src/unitree_g1_dex3_stack-main/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0.urdf` | Robot model | MODIFY — add `right_tcp_link` fixed joint after `right_wrist_yaw_link` | `right_hand_palm_joint` (sibling fixed joint) |
| `src/unitree_g1_dex3_stack-main/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf` | Robot model (collision) | MODIFY — add `right_tcp_link` fixed joint after `right_wrist_yaw_link` | `right_hand_palm_joint` (sibling fixed joint) |
| `src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` | Planner node | MODIFY — `right_tip` default → `right_tcp_link`, add `tcp_offset_x` param + KDL chain override | Self (existing `declare_parameter`/`get_parameter` pattern) |

## Pattern Details

### CMakeLists.txt
**Role:** Build system — defines compile targets, dependencies, install rules
**Action:** Remove deleted node targets (`project_to_3d_node`, `detection_to_goal_node`, `visual_detection_yolo_tester`), remove `bboxes_ex_msgs`/`image_transport`/`pcl_conversions`/`PCL` from `find_package`, remove `${PCL_INCLUDE_DIRS}`, remove `target_link_libraries(project_to_3d_node ...)`, remove from `install(TARGETS ...)`, remove `scripts/ultralytics_detector.py` and `scripts/elevator_ocr_node.py` from `install(PROGRAMS ...)`

**Current code excerpt — find_package block (lines 34-36):**
```cmake
find_package(bboxes_ex_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
```

**Current code excerpt — add_executable (lines 63-66):**
```cmake
add_executable(project_to_3d_node src/project_to_3d_node.cpp)
add_executable(detection_to_goal_node src/detection_to_goal_node.cpp)
add_executable(visual_detection_yolo_tester src/visual_detection_yolo_tester.cpp)
```

**Current code excerpt — ament_target_dependencies (lines 79-85):**
```cmake
ament_target_dependencies(project_to_3d_node
  rclcpp vision_msgs bboxes_ex_msgs sensor_msgs geometry_msgs std_msgs cv_bridge image_transport message_filters pcl_conversions tf2 tf2_ros
)
ament_target_dependencies(detection_to_goal_node
  rclcpp vision_msgs bboxes_ex_msgs sensor_msgs geometry_msgs std_msgs
)
ament_target_dependencies(visual_detection_yolo_tester
  rclcpp vision_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs
)
```

**Current code excerpt — target_link_libraries (line 101):**
```cmake
target_link_libraries(project_to_3d_node
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
```

**Current code excerpt — install TARGETS (lines 107-115):**
```cmake
install(TARGETS
  joint_state_publisher
  dex3_controller
  joint_trajectory_executor
  project_to_3d_node
  detection_to_goal_node
  right_hand_pressure_monitor
  visual_detection_tester
  visual_detection_yolo_tester
  DESTINATION lib/${PROJECT_NAME}
)
```

**Current code excerpt — install PROGRAMS (lines 123-128):**
```cmake
install(PROGRAMS
  scripts/ultralytics_detector.py
  scripts/elevator_ocr_node.py
  scripts/tcp_torso_pose.py
  scripts/keyboard_trigger_node.py
  DESTINATION lib/${PROJECT_NAME}
)
```

**Current code excerpt — include_directories (line 52):**
```cmake
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
```

**Pattern to follow:** Remove lines referencing deleted targets/deps. Keep structure intact for remaining targets. Remove `${PCL_INCLUDE_DIRS}` from `include_directories`.

---

### package.xml
**Role:** ROS2 package manifest — declares build/runtime dependencies
**Action:** Remove `bboxes_ex_msgs`, `image_transport`, `pcl_conversions`, `pcl_msgs` depend entries

**Current code excerpt (lines 38-43):**
```xml
  <!-- Perception message types -->
  <depend>vision_msgs</depend>
  <depend>bboxes_ex_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>image_transport</depend>
  <depend>message_filters</depend>
  <depend>pcl_conversions</depend>
  <depend>pcl_msgs</depend>
```

**Pattern to follow:** Remove the 4 lines (`bboxes_ex_msgs`, `image_transport`, `pcl_conversions`, `pcl_msgs`). Keep `vision_msgs`, `cv_bridge`, `message_filters`.

---

### planner.launch.py
**Role:** Launch config for planner node — resolves parameters via `OpaqueFunction`
**Action:** Change `right_tip` default from `right_wrist_yaw_link` to `right_tcp_link`. Add `tcp_offset_x` `DeclareLaunchArgument` (default `0.175`). Pass `tcp_offset_x` to node parameters. Remove `detection_topic` and `selected_class_topic` args.

**Current code excerpt — launch_setup function (lines 7-14):**
```python
    trajectory_time_step = float(LaunchConfiguration('trajectory_time_step').perform(context))
    planning_timeout = float(LaunchConfiguration('planning_timeout').perform(context))
    base_link = str(LaunchConfiguration('base_link').perform(context))
    right_tip = str(LaunchConfiguration('right_tip').perform(context))
    detection_topic = str(LaunchConfiguration('detection_topic').perform(context))
    selected_class_topic = str(LaunchConfiguration('selected_class_topic').perform(context))
    planner_type = str(LaunchConfiguration('planner_type').perform(context))
```

**Current code excerpt — parameters dict (lines 16-23):**
```python
    parameters = {
        'trajectory_time_step': trajectory_time_step,
        'planning_timeout': planning_timeout,
        'base_link': base_link,
        'right_tip': right_tip,
        'detection_topic': detection_topic,
        'selected_class_topic': selected_class_topic,
        'planner_type': planner_type,
    }
```

**Current code excerpt — DeclareLaunchArguments (lines 42-49):**
```python
    args = [
        DeclareLaunchArgument('trajectory_time_step', default_value='0.05'),
        DeclareLaunchArgument('planning_timeout', default_value='1.0'),
        DeclareLaunchArgument('base_link', default_value='torso_link'),
        DeclareLaunchArgument('right_tip', default_value='right_wrist_yaw_link'),
        DeclareLaunchArgument('detection_topic', default_value='/detections'),
        DeclareLaunchArgument('selected_class_topic', default_value='/selected_detection_class'),
        DeclareLaunchArgument('planner_type', default_value='RRTConnect'),
        DeclareLaunchArgument('collision_skip_pairs', default_value='right_hand_thumb_0_link:right_wrist_yaw_link'),
    ]
```

**Pattern to follow:** Same `OpaqueFunction` + `LaunchConfiguration.perform()` pattern. Add `tcp_offset_x` as `float(LaunchConfiguration('tcp_offset_x').perform(context))`, add to parameters dict, add `DeclareLaunchArgument('tcp_offset_x', default_value='0.175')`. Change `right_tip` default to `right_tcp_link`. Remove `detection_topic` and `selected_class_topic` from all three locations.

---

### reach.launch.py
**Role:** Top-level launch orchestrator — composes robot + perception + planner + control
**Action:** Remove `model_path_arg`, `target_class_arg`, `imshow_arg`, `perception_launch`, `keyboard_node`. Keep `planning_timeout_arg`, `robot_launch`, `d435_tf_node`, `planner_launch`, `control_launch`, `TimerAction`.

**Current code excerpt — YOLO args to remove (lines 14-27):**
```python
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/home/unitree/Desktop/unitree_dex3/best.pt',
        description='Path to YOLO model file'
    )
    target_class_arg = DeclareLaunchArgument(
        'target_class',
        default_value='terminal',
        description='Object class to reach for'
    )
    imshow_arg = DeclareLaunchArgument(
        'imshow',
        default_value='false',
        description='Whether to open an OpenCV display window'
    )
```

**Current code excerpt — perception_launch to remove (lines 41-51):**
```python
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'perception.launch.py')
        ),
        launch_arguments={
            'model_path': LaunchConfiguration('model_path'),
            'imshow': LaunchConfiguration('imshow'),
            'allowed_classes': ["['", LaunchConfiguration('target_class'), "']"],
        }.items()
    )
```

**Current code excerpt — keyboard_node to remove (lines 65-71):**
```python
    keyboard_node = Node(
        package='unitree_g1_dex3_stack',
        executable='keyboard_trigger_node.py',
        name='keyboard_trigger_node',
        output='screen',
        emulate_tty=True,
    )
```

**Current code excerpt — TimerAction (lines 73-78):**
```python
    delayed_actions = TimerAction(period=3.0, actions=[
        perception_launch,
        planner_launch,
        control_launch,
        keyboard_node,
    ])
```

**Pattern to follow:** Keep `IncludeLaunchDescription` + `TimerAction` structure. Simplified `TimerAction` contains only `planner_launch` and `control_launch`. Return list: `[planning_timeout_arg, robot_launch, d435_tf_node, delayed_actions]`.

---

### g1_29dof_lock_waist_with_hand_rev_1_0.urdf
**Role:** Robot kinematic/visual model — defines links, joints, meshes
**Action:** Insert `right_tcp_joint` (fixed) + `right_tcp_link` between `right_wrist_yaw_link </link>` and `right_hand_palm_joint`

**Current code excerpt (lines 1255-1261):**
```xml
  </link>
  <joint name="right_hand_palm_joint" type="fixed">
    <origin xyz="0.0415 -0.003 0" rpy="0 0 0" />
    <parent link="right_wrist_yaw_link" />
    <child link="right_hand_palm_link" />
  </joint>
```

**Pattern to follow (analog: `right_hand_palm_joint` structure):**
```xml
  <!-- TCP (Tool Center Point) virtual link: 0.175m along wrist_yaw X axis -->
  <joint name="right_tcp_joint" type="fixed">
    <origin xyz="0.175 0 0" rpy="0 0 0" />
    <parent link="right_wrist_yaw_link" />
    <child link="right_tcp_link" />
  </joint>
  <link name="right_tcp_link" />
```
Insert immediately before `<joint name="right_hand_palm_joint" ...>`.

---

### g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf
**Role:** Robot collision model — simplified collision geometries for FCL
**Action:** Same as above — insert `right_tcp_joint` + `right_tcp_link` before `right_hand_palm_joint`

**Current code excerpt (lines 1062-1066):**
```xml
  <joint name="right_hand_palm_joint" type="fixed">
    <origin xyz="0.0415 -0.003 0" rpy="0 0 0" />
    <parent link="right_wrist_yaw_link" />
    <child link="right_hand_palm_link" />
  </joint>
```

**Pattern to follow:** Identical insertion as above. No collision/visual elements needed for `right_tcp_link`.

---

### ik_fcl_ompl_planner.cpp
**Role:** Core planner node — IK solving, collision checking, OMPL path planning
**Action:** (1) Change `right_tip` default from `right_wrist_yaw_link` to `right_tcp_link`. (2) Add `tcp_offset_x` parameter declaration + chain override logic after `getChain()` and before joint limits iteration.

**Current code excerpt — parameter declaration (line 65):**
```cpp
this->declare_parameter("right_tip", "right_wrist_yaw_link");
```

**Current code excerpt — getChain (line 127):**
```cpp
if (!kdl_tree.getChain(base_link_, right_tip_, kdl_chain_right)) {
    RCLCPP_FATAL(this->get_logger(), "Failed to extract KDL chain for right arm");
    rclcpp::shutdown();
    return;
}
```

**Current code excerpt — joint limits iteration (immediately after getChain, line ~133):**
```cpp
// Parse joint limits from URDF for OMPL bounds
for (const auto& joint_pair : urdf_model.joints_) { ... }

// Build joint limits array for right arm
KDL::JntArray right_lower(kdl_chain_right.getNrOfJoints()), right_upper(kdl_chain_right.getNrOfJoints());
size_t idx = 0;
for (unsigned int i = 0; i < kdl_chain_right.getNrOfSegments(); ++i) { ... }
```

**Pattern to follow — tcp_offset_x override (insert between getChain success and joint limits iteration):**
```cpp
// TCP offset runtime override
this->declare_parameter("tcp_offset_x", 0.175);
double tcp_offset_x;
this->get_parameter("tcp_offset_x", tcp_offset_x);

unsigned int n_seg = kdl_chain_right.getNrOfSegments();
if (n_seg > 0) {
    KDL::Segment last_seg = kdl_chain_right.getSegment(n_seg - 1);
    if (last_seg.getJoint().getType() == KDL::Joint::None) {
        KDL::Chain new_chain;
        for (unsigned int i = 0; i < n_seg - 1; ++i) {
            new_chain.addSegment(kdl_chain_right.getSegment(i));
        }
        KDL::Frame tcp_frame(KDL::Vector(tcp_offset_x, 0.0, 0.0));
        new_chain.addSegment(KDL::Segment(last_seg.getName(),
                                           KDL::Joint(KDL::Joint::None),
                                           tcp_frame));
        kdl_chain_right = new_chain;
        RCLCPP_INFO(this->get_logger(), "TCP offset overridden to %.4f m", tcp_offset_x);
    }
}
```

**Key constraint:** This block MUST execute before:
1. Joint limits array construction (iterates segments)
2. Adjacent-link skip pairs derivation (iterates segments)
3. TRAC-IK construction (receives chain)
4. FK solver construction (receives chain)

## PATTERN MAPPING COMPLETE
