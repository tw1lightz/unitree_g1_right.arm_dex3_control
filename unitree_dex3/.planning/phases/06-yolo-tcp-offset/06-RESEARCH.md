# Phase 6: YOLO 清理 + TCP Offset 集成 — Research

**Researched:** 2026-05-15
**Phase:** 06-yolo-tcp-offset
**Requirements:** CLEAN-01, TCP-01, TCP-02

## Executive Summary

The YOLO cleanup is straightforward — 10+ files to delete, 2 CMakeLists targets + 1 package.xml dep to remove, and the `bboxes_ex_msgs` package to delete entirely. The TCP offset integration is well-supported: adding a fixed `right_tcp_link` to URDF extends the KDL chain automatically (fixed joints add a frame offset without adding DOF), and TRAC-IK receives the chain directly so it inherits the extension. Runtime override via `tcp_offset_x` parameter requires modifying the last segment's frame in the KDL chain after `getChain()` but before TRAC-IK construction.

## 1. YOLO Cleanup Scope

### Files to Delete

| File | Type | Notes |
|------|------|-------|
| `src/unitree_g1_dex3_stack-main/scripts/ultralytics_detector.py` | Python node | YOLO detector |
| `src/unitree_g1_dex3_stack-main/src/project_to_3d_node.cpp` | C++ node | 2D→3D projection, depends on `bboxes_ex_msgs` |
| `src/unitree_g1_dex3_stack-main/src/detection_to_goal_node.cpp` | C++ node | Detection→goal_pose bridge, depends on `bboxes_ex_msgs` |
| `src/unitree_g1_dex3_stack-main/src/visual_detection_yolo_tester.cpp` | C++ node | YOLO test tool, depends on `vision_msgs/Detection3DArray` |
| `src/unitree_g1_dex3_stack-main/launch/perception.launch.py` | Launch | Full YOLO perception pipeline |
| `src/unitree_g1_dex3_stack-main/launch/visual_detect_yolo.launch.py` | Launch | YOLO test launch |
| `src/unitree_g1_dex3_stack-main/launch/elevator_perception.launch.py` | Launch | Elevator button YOLO pipeline (also uses `ultralytics_detector`, `project_to_3d_node`, `detection_to_goal_node`) |
| `src/unitree_g1_dex3_stack-main/docs/ARCHITECTURE(yolo,failed).md` | Doc | Obsolete architecture doc |
| `src/bboxes_ex_msgs/` | Entire package | Custom bbox messages, only used by deleted nodes |
| `best.pt` (project root) | Model file | YOLO weights |
| `run_perception.sh` (project root) | Shell script | YOLO launch helper |

### Files to KEEP (per D-02)

- `src/unitree_g1_dex3_stack-main/src/visual_detection_tester.cpp` — AprilTag/click-based detection tester. No `bboxes_ex_msgs` dependency. Uses `cv_bridge`, `message_filters`, `tf2`. Already has its own CMake target with correct deps.
- `src/unitree_g1_dex3_stack-main/launch/visual_detect_click.launch.py` — Click-based detection launch. References `visual_detection_tester` executable only.

### Hidden Dependencies Found

1. **`elevator_ocr_node.py`** (line 10): `from bboxes_ex_msgs.msg import BoundingBox, BoundingBoxes` — This script imports `bboxes_ex_msgs`. It's installed via CMakeLists but only used in `elevator_perception.launch.py` (which is being deleted). **Decision needed:** delete `elevator_ocr_node.py` too, or leave it broken (it won't be launched). Recommend deletion since its only consumer is being deleted.

2. **`keyboard_trigger_node.py`**: Referenced in `reach.launch.py` (being simplified) and installed in CMakeLists. Not harmful to keep installed but should be removed from `reach.launch.py`. Can optionally remove from install list.

3. **`vision_msgs` dependency**: Still needed by `ik_fcl_ompl_planner.cpp` (includes `vision_msgs/msg/detection3_d_array.hpp`). Do NOT remove from package.xml/CMakeLists.

4. **`OpenCV`, `cv_bridge`, `image_transport`, `message_filters`, `pcl_conversions`, `PCL`**: These are used by `visual_detection_tester.cpp` (kept) and `project_to_3d_node.cpp` (deleted). After deletion, `visual_detection_tester` still needs `cv_bridge`, `message_filters`, `OpenCV`. The `image_transport`, `pcl_conversions`, `PCL` deps are only used by `project_to_3d_node` — can be removed from CMakeLists `find_package` and package.xml. However, keeping them is harmless and simpler.

## 2. URDF Virtual Link for TCP

### Current Chain End Structure

In both URDF files, after `right_wrist_yaw_link`:
```xml
<joint name="right_hand_palm_joint" type="fixed">
  <origin xyz="0.0415 -0.003 0" rpy="0 0 0" />
  <parent link="right_wrist_yaw_link" />
  <child link="right_hand_palm_link" />
</joint>
```

The `right_tcp_link` should be a **direct child of `right_wrist_yaw_link`** (sibling to `right_hand_palm_link`), NOT a child of `right_hand_palm_link`. This is because:
- The TCP offset (0.175m) is measured from `right_wrist_yaw_link` origin along its X axis
- KDL `getChain(base, "right_tcp_link")` will traverse `torso_link → ... → right_wrist_yaw_link → right_tcp_link`

### URDF Syntax to Add

Insert immediately after the `right_wrist_yaw_link` `</link>` closing tag (before `right_hand_palm_joint`):

```xml
<!-- TCP (Tool Center Point) virtual link: 0.175m along wrist_yaw X axis -->
<joint name="right_tcp_joint" type="fixed">
  <origin xyz="0.175 0 0" rpy="0 0 0" />
  <parent link="right_wrist_yaw_link" />
  <child link="right_tcp_link" />
</joint>
<link name="right_tcp_link" />
```

No `<visual>`, `<collision>`, or `<inertial>` elements needed — it's a virtual reference frame.

### KDL/TRAC-IK Compatibility

- **KDL fixed joints**: When `kdl_tree.getChain("torso_link", "right_tcp_link")` is called, KDL traverses the tree and includes the fixed joint as a `KDL::Segment` with `Joint::None` type. This adds a frame offset (the 0.175m translation) without adding a DOF. `getNrOfJoints()` remains unchanged (7 joints for right arm).
- **TRAC-IK**: The constructor `TRAC_IK::TRAC_IK(node, chain, lower, upper, ...)` takes the chain directly. Since the chain has the same number of joints (fixed joints don't count), the `lower`/`upper` arrays remain the same size. TRAC-IK will solve IK to place the chain tip (now `right_tcp_link`) at the goal pose.
- **FK solver**: `ChainFkSolverPos_recursive` will compute FK to the chain tip, which is now `right_tcp_link` — automatically including the 0.175m offset.
- **Collision detection**: `right_tcp_link` has no collision geometry, so `buildCollisionObjects()` will skip it (the code checks `if (!link->collision || !link->collision->geometry) continue;`).

### Both URDF Files to Modify

1. `g1_29dof_lock_waist_with_hand_rev_1_0.urdf` — line ~1260 (after `right_wrist_yaw_link` `</link>`)
2. `g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf` — line ~1064 (after `right_wrist_yaw_link` `</link>`)

## 3. KDL Chain Runtime Override

### Why Runtime Override?

The URDF defines the default TCP offset (0.175m). The `tcp_offset_x` ROS parameter allows overriding this at runtime without editing URDF — useful for different tools or calibration adjustments.

### Implementation Strategy

After `kdl_tree.getChain(base_link_, right_tip_, kdl_chain_right)` succeeds (line ~127), and before TRAC-IK construction (line ~180):

```cpp
// Declare and read tcp_offset_x parameter
this->declare_parameter("tcp_offset_x", 0.175);
double tcp_offset_x;
this->get_parameter("tcp_offset_x", tcp_offset_x);

// Override the last segment's frame in the chain
// The last segment is right_tcp_link with Joint::None
unsigned int n_seg = kdl_chain_right.getNrOfSegments();
if (n_seg > 0) {
    KDL::Segment last_seg = kdl_chain_right.getSegment(n_seg - 1);
    if (last_seg.getJoint().getType() == KDL::Joint::None) {
        // Rebuild chain with modified last segment
        KDL::Chain new_chain;
        for (unsigned int i = 0; i < n_seg - 1; ++i) {
            new_chain.addSegment(kdl_chain_right.getSegment(i));
        }
        // Replace last segment's frame with parameter value
        KDL::Frame tcp_frame(KDL::Vector(tcp_offset_x, 0.0, 0.0));
        new_chain.addSegment(KDL::Segment(last_seg.getName(),
                                           KDL::Joint(KDL::Joint::None),
                                           tcp_frame));
        kdl_chain_right = new_chain;
        RCLCPP_INFO(this->get_logger(), "TCP offset overridden to %.4f m", tcp_offset_x);
    }
}
```

### KDL API Details

- `KDL::Chain::getSegment(i)` — returns `const Segment&`
- `KDL::Chain::addSegment(Segment)` — appends segment to chain
- `KDL::Chain::getNrOfSegments()` — total segments (including fixed)
- `KDL::Chain::getNrOfJoints()` — only movable joints (excludes `Joint::None`)
- `KDL::Segment(name, joint, frame)` — constructor with tip frame relative to parent
- `KDL::Joint(KDL::Joint::None)` — fixed joint (no DOF)
- `KDL::Frame(KDL::Vector(x, y, z))` — translation-only frame

### Key Constraint

The chain rebuild MUST happen before:
1. Joint limits array construction (iterates segments)
2. Adjacent-link skip pairs derivation (iterates segments)
3. TRAC-IK construction (receives chain)
4. FK solver construction (receives chain)

So the override should be placed immediately after `getChain()` succeeds.

## 4. Launch File Simplification

### Current `reach.launch.py` Structure

```
DeclareLaunchArgument: model_path (YOLO)        ← REMOVE
DeclareLaunchArgument: target_class (YOLO)      ← REMOVE
DeclareLaunchArgument: imshow (YOLO)            ← REMOVE
DeclareLaunchArgument: planning_timeout         ← KEEP

IncludeLaunchDescription: robot.launch.py       ← KEEP
Node: d435_tf_node (static TF publisher)        ← KEEP (Phase 7 needs it)

TimerAction(3s):
  IncludeLaunchDescription: perception.launch.py  ← REMOVE
  IncludeLaunchDescription: planner.launch.py     ← KEEP
  IncludeLaunchDescription: control.launch.py     ← KEEP
  Node: keyboard_trigger_node                     ← REMOVE
```

### Simplified `reach.launch.py`

```python
# Only: robot + d435_tf + planner + control
# planning_timeout arg kept
# TimerAction still useful to let robot_state_publisher start first
```

### `planner.launch.py` Changes

Current default: `right_tip` = `right_wrist_yaw_link`
New default: `right_tip` = `right_tcp_link`

Add new `DeclareLaunchArgument`:
- `tcp_offset_x` (default `0.175`)

Pass to node parameters.

Also: `detection_topic` and `selected_class_topic` parameters are YOLO-era leftovers in planner.launch.py. They're declared but the planner code subscribes to `/goal_pose` (not detections). These can be removed from planner.launch.py for clarity, but they're harmless if left.

## 5. Build System Cleanup

### CMakeLists.txt Changes

**Remove `find_package`:**
- `find_package(bboxes_ex_msgs REQUIRED)` — only used by deleted nodes

**Optionally remove (only used by `project_to_3d_node`):**
- `find_package(image_transport REQUIRED)` — not used by any remaining target
- `find_package(pcl_conversions REQUIRED)` — not used by any remaining target
- `find_package(PCL REQUIRED)` — not used by any remaining target

**Keep:**
- `find_package(OpenCV REQUIRED)` — used by `visual_detection_tester`
- `find_package(cv_bridge REQUIRED)` — used by `visual_detection_tester`
- `find_package(message_filters REQUIRED)` — used by `visual_detection_tester`
- `find_package(vision_msgs REQUIRED)` — used by `ik_fcl_ompl_planner`

**Remove `add_executable`:**
- `add_executable(project_to_3d_node ...)`
- `add_executable(detection_to_goal_node ...)`
- `add_executable(visual_detection_yolo_tester ...)`

**Remove `ament_target_dependencies`:**
- `ament_target_dependencies(project_to_3d_node ...)`
- `ament_target_dependencies(detection_to_goal_node ...)`
- `ament_target_dependencies(visual_detection_yolo_tester ...)`

**Remove `target_link_libraries`:**
- `target_link_libraries(project_to_3d_node ...)` (OpenCV + PCL)

**Remove from `install(TARGETS ...)`:**
- `project_to_3d_node`
- `detection_to_goal_node`
- `visual_detection_yolo_tester`

**Remove from `install(PROGRAMS ...)`:**
- `scripts/ultralytics_detector.py`
- Optionally: `scripts/elevator_ocr_node.py` (broken without `bboxes_ex_msgs`)

**Keep in `install(PROGRAMS ...)`:**
- `scripts/tcp_torso_pose.py`
- `scripts/keyboard_trigger_node.py` (harmless, may be useful for manual testing)

### package.xml Changes

**Remove:**
- `<depend>bboxes_ex_msgs</depend>`

**Optionally remove:**
- `<depend>image_transport</depend>` — only used by deleted `project_to_3d_node`
- `<depend>pcl_conversions</depend>` — only used by deleted `project_to_3d_node`
- `<depend>pcl_msgs</depend>` — only used by deleted `project_to_3d_node`

**Keep:**
- `<depend>OpenCV</depend>` — `visual_detection_tester`
- `<depend>cv_bridge</depend>` — `visual_detection_tester`
- `<depend>message_filters</depend>` — `visual_detection_tester`
- `<depend>vision_msgs</depend>` — `ik_fcl_ompl_planner`

### include_directories Cleanup

- `${PCL_INCLUDE_DIRS}` — can be removed if PCL `find_package` is removed
- `${OpenCV_INCLUDE_DIRS}` — keep (for `visual_detection_tester`)

## 6. Risk Assessment

### Low Risk
- **YOLO file deletion**: No remaining code depends on these files. Clean cut.
- **`bboxes_ex_msgs` package deletion**: Only referenced by deleted nodes and `elevator_ocr_node.py`.
- **Launch simplification**: Removing nodes from launch doesn't affect other nodes.

### Medium Risk
- **URDF modification**: Adding `right_tcp_link` as a sibling of `right_hand_palm_link` (both children of `right_wrist_yaw_link`) creates a branch in the KDL tree. `getChain("torso_link", "right_tcp_link")` will correctly traverse to the new link. However, if any code hardcodes chain traversal expecting `right_hand_palm_link` to be the only child, it could break. **Mitigation:** The planner only uses `getChain()` with the `right_tip` parameter — no hardcoded assumptions.
- **KDL chain rebuild for tcp_offset_x override**: Rebuilding the chain segment-by-segment is safe but must happen at the right point in initialization. If placed after joint limits are already computed, the limits array won't match. **Mitigation:** Place immediately after `getChain()`, before any iteration over segments.

### Edge Cases
- **`tcp_offset_x = 0`**: Would place TCP at `right_wrist_yaw_link` origin. Valid but unusual. The URDF still has 0.175m, so only the runtime override would produce 0.
- **`right_tip` parameter set to something other than `right_tcp_link`**: If user overrides `right_tip` to `right_wrist_yaw_link` (old behavior), the chain won't have a fixed segment at the end, so the `tcp_offset_x` override logic should check if the last segment is actually a fixed joint before attempting modification. If not, skip the override gracefully.
- **`visual_detection_tester` compilation**: After removing PCL/image_transport from `find_package`, verify it still compiles. Its deps are: `rclcpp sensor_msgs geometry_msgs cv_bridge message_filters tf2 tf2_ros tf2_geometry_msgs` + OpenCV link. All retained.

### Testing Approach
1. `colcon build` — verify no compilation errors
2. Launch `reach.launch.py` — verify robot + planner + control start without errors
3. `ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped ...` — verify planner still plans
4. Compare FK output of `tcp_torso_pose.py` (which manually applies offset) with planner's FK (which now has it in chain) — should match
5. Override `tcp_offset_x:=0.0` and verify planner solves to wrist_yaw_link position

## 7. Validation Architecture

### CLEAN-01: YOLO Code Removed
- **Check:** `find . -name "*.py" -o -name "*.cpp" | xargs grep -l "ultralytics\|bboxes_ex_msgs\|yolo"` returns nothing in active code
- **Check:** `colcon build --packages-select unitree_g1_dex3_stack` succeeds without `bboxes_ex_msgs`
- **Check:** `src/bboxes_ex_msgs/` directory does not exist
- **Check:** `best.pt` and `run_perception.sh` do not exist

### TCP-01: TCP Offset in IK Chain
- **Check:** URDF contains `right_tcp_link` with fixed joint at x=0.175
- **Check:** Planner logs show `right_tip: right_tcp_link` on startup
- **Check:** `ros2 topic pub /goal_pose` with a known reachable pose → planner succeeds
- **Check:** FK comparison: `tcp_torso_pose.py` output matches planner's internal FK to chain tip (both should report same TCP position for same joint angles)

### TCP-02: TCP Offset Configurable
- **Check:** `ros2 param get /ik_fcl_ompl_planner tcp_offset_x` returns 0.175 (default)
- **Check:** Launch with `tcp_offset_x:=0.20` → planner logs show "TCP offset overridden to 0.2000 m"
- **Check:** No hardcoded 0.175 in C++ source (only in URDF default and parameter default)

## RESEARCH COMPLETE
