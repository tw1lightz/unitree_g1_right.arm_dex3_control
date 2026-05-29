# Architecture Research: v1.1

## Summary

This document describes how AprilTag 36h11 detection, TCP offset IK correction, and adaptive end-effector orientation integrate with the existing C++ OMPL planner and ROS 2 node architecture.

---

## New Components

### 1. `apriltag_ros` node (external package, to install)

- **Package**: `ros-humble-apriltag-ros` (available via apt, not yet installed)
- **Role**: Detects AprilTag 36h11 markers from the RealSense RGB stream
- **Subscribes**: `/camera/camera/color/image_raw` (sensor_msgs/Image), `/camera/camera/color/camera_info` (sensor_msgs/CameraInfo)
- **Publishes**: `/apriltag/detections` (apriltag_msgs/AprilTagDetectionArray) — includes tag ID, 6-DOF pose in camera optical frame
- **Config**: YAML file specifying tag family (36h11), tag sizes, and which tag IDs to detect
- **Frame**: Outputs poses in `camera_color_optical_frame`

### 2. `apriltag_to_goal_node` (NEW Python node)

- **Role**: Replaces `keyboard_trigger_node` + `detection_to_goal_node` + `ultralytics_detector` + `project_to_3d_node` for the AprilTag pipeline
- **Subscribes**: `/apriltag/detections` (AprilTagDetectionArray)
- **Publishes**: `/goal_pose` (geometry_msgs/PoseStamped)
- **Logic**:
  1. Receives AprilTag 6-DOF pose (position + orientation of tag center in camera frame)
  2. Applies configurable offset (dx, dy, dz) relative to the tag frame to compute the actual target point (e.g., "10cm in front of the tag")
  3. Computes adaptive orientation for the TCP (see below)
  4. Applies TCP offset correction: shifts the goal 0.175m back along the approach axis so that IK solves for `right_wrist_yaw_link` but the TCP arrives at the target
  5. Publishes the corrected goal as PoseStamped in `camera_color_optical_frame` (planner handles TF to torso_link)
- **Parameters**:
  - `target_tag_id` (int): Which tag ID to target
  - `tag_offset_x/y/z` (double): Offset from tag center to desired reach point, in tag-local frame
  - `tcp_offset` (double, default 0.175): TCP offset along wrist X axis
  - `orientation_mode` (string): `"adaptive"` | `"fixed"` | `"from_tag"`
- **Trigger**: Can be keyboard-triggered (subscribe to `/trigger` Empty) or auto-trigger on first detection

### 3. `apriltag_reach.launch.py` (NEW launch file)

- **Role**: Replaces `reach.launch.py` for the AprilTag pipeline
- **Includes**: `robot.launch.py`, `planner.launch.py`, `control.launch.py`
- **Starts**: `apriltag_ros` node, `apriltag_to_goal_node`, keyboard trigger (optional)
- **Does NOT include**: `perception.launch.py` (YOLO pipeline removed)

---

## Modified Components

### 1. `ik_fcl_ompl_planner.cpp` — NO MODIFICATION NEEDED

The planner already:
- Subscribes to `/goal_pose` (PoseStamped) — frame-agnostic, uses TF2 to transform to `torso_link`
- Solves IK for the received pose as the target for `right_wrist_yaw_link`
- Plans with OMPL and publishes JointTrajectory

**Key insight**: The TCP offset and adaptive orientation are handled UPSTREAM in `apriltag_to_goal_node`. The planner receives a corrected goal that already accounts for TCP offset. This means:
- The IK target frame remains `right_wrist_yaw_link`
- The goal pose published to `/goal_pose` is the desired pose of `right_wrist_yaw_link` (not the TCP tip)
- No C++ code changes required in the planner

### 2. `joint_trajectory_executor.cpp` — NO MODIFICATION NEEDED

Already works correctly — receives JointTrajectory and executes.

### 3. `CMakeLists.txt` — MINOR MODIFICATION

- Add install rule for `apriltag_to_goal_node.py` script
- Add install rule for new launch file and config directory

### 4. `package.xml` — MINOR MODIFICATION

- Add `<exec_depend>apriltag_ros</exec_depend>`
- Add `<exec_depend>apriltag_msgs</exec_depend>`

---

## Data Flow

### Current (v1.0 — YOLO, being removed)

```
RealSense D435i
    ↓ RGB + Depth
ultralytics_detector (YOLO)
    ↓ 2D bboxes
project_to_3d_node
    ↓ Detection3DArray (camera frame)
keyboard_trigger_node
    ↓ /goal_pose (PoseStamped, fixed orientation, NO TCP correction)
ik_fcl_ompl_planner
    ↓ /joint_trajectory_targets (JointTrajectory)
joint_trajectory_executor
    ↓ LowCmd → robot
```

### New (v1.1 — AprilTag)

```
RealSense D435i
    ↓ RGB + CameraInfo
apriltag_ros (apriltag_node)
    ↓ /apriltag/detections (AprilTagDetectionArray, 6-DOF pose in camera frame)
apriltag_to_goal_node
    │  1. Select target tag by ID
    │  2. Apply tag-relative offset → target point
    │  3. Compute adaptive orientation
    │  4. Apply TCP offset correction (shift goal back 0.175m)
    ↓ /goal_pose (PoseStamped, camera_color_optical_frame)
ik_fcl_ompl_planner (UNCHANGED)
    │  TF transform camera → torso_link
    │  TRAC-IK → OMPL → JointTrajectory
    ↓ /joint_trajectory_targets (JointTrajectory)
joint_trajectory_executor (UNCHANGED)
    ↓ LowCmd → robot
```

### Key Differences

| Aspect | v1.0 (YOLO) | v1.1 (AprilTag) |
|--------|-------------|-----------------|
| Detection | YOLO 2D → depth → 3D centroid | AprilTag direct 6-DOF pose |
| Depth required | Yes (RGB-D fusion) | No (monocular pose estimation) |
| Orientation source | Hardcoded quaternion | Adaptive from geometry |
| TCP offset | Not corrected (IK target = TCP target) | Corrected (IK target shifted back) |
| Nodes in pipeline | 4 (detector + project + trigger + planner) | 3 (apriltag_ros + goal_node + planner) |
| Trigger | Keyboard (K key) | Keyboard or auto |

---

## Integration Points

### 1. `/goal_pose` topic (geometry_msgs/PoseStamped)

- **Contract**: Same as v1.0 — the planner subscribes to this topic
- **Change**: The publisher changes from `keyboard_trigger_node` to `apriltag_to_goal_node`
- **Frame**: Can be any frame with a valid TF path to `torso_link` (planner handles transform)

### 2. TF tree

- **Existing**: `torso_link` ← ... ← `head_link` ← `d435_link` ← `camera_link` ← `camera_color_optical_frame`
- **New requirement**: `apriltag_ros` publishes tag poses in `camera_color_optical_frame` — this is already connected to `torso_link` via the existing TF chain
- **No new TF publishers needed**

### 3. RealSense camera topics

- **Existing**: `/camera/camera/color/image_raw`, `/camera/camera/color/camera_info`
- **Used by**: `apriltag_ros` (same topics as YOLO used)
- **Depth topics**: No longer needed for detection (AprilTag is monocular)

### 4. TCP Offset Correction Logic

The TCP offset correction in `apriltag_to_goal_node` works as follows:

```
Given:
  - target_point: where we want the TCP tip to arrive (in camera frame)
  - target_orientation: desired orientation of the TCP (quaternion)
  - tcp_offset = 0.175m along local X of right_wrist_yaw_link

Correction:
  - The wrist_yaw_link is 0.175m BEHIND the TCP tip along its local X axis
  - So: goal_for_planner.position = target_point - 0.175 * approach_direction
  - Where approach_direction = rotation_matrix(target_orientation) * [1, 0, 0]
```

This keeps the planner's IK chain endpoint (`right_wrist_yaw_link`) correct while ensuring the physical TCP reaches the target.

### 5. Adaptive Orientation Strategy

The adaptive orientation module in `apriltag_to_goal_node` selects orientation based on:

1. **From tag** (`orientation_mode="from_tag"`): Use the tag's detected orientation directly — approach perpendicular to the tag surface
2. **Adaptive** (`orientation_mode="adaptive"`): Compute orientation based on the vector from shoulder to target:
   - Calculate approach vector from right shoulder position to target
   - Align TCP X-axis (approach direction) with this vector
   - Constrain roll to keep the hand roughly upright
   - If IK fails (detected by timeout on `/goal_pose` without trajectory response), try alternative orientations
3. **Fixed** (`orientation_mode="fixed"`): Use the hardcoded quaternion from v1.0 (fallback)

The "from_tag" mode is the recommended default for AprilTag targets since the tag orientation directly encodes the surface normal of the object.

---

## Suggested Build Order

Build order considers dependencies — each step can be tested independently before proceeding.

### Step 1: Install `apriltag_ros` and verify detection

**Dependencies**: None (uses existing RealSense driver)
**Tasks**:
- `sudo apt install ros-humble-apriltag-ros ros-humble-apriltag-msgs`
- Create AprilTag config YAML (tag family: 36h11, tag sizes)
- Test: launch RealSense + apriltag_ros, verify `/apriltag/detections` publishes correct poses
- Print a 36h11 tag and verify detection at expected distances

**Verification**: `ros2 topic echo /apriltag/detections` shows correct tag ID and pose

### Step 2: Implement `apriltag_to_goal_node` with TCP offset correction

**Dependencies**: Step 1 (needs apriltag_msgs type definitions)
**Tasks**:
- Write Python node subscribing to AprilTagDetectionArray
- Implement tag-relative offset computation
- Implement TCP offset correction (shift goal back 0.175m along approach axis)
- Start with `orientation_mode="from_tag"` (simplest — use tag orientation directly)
- Publish corrected PoseStamped to `/goal_pose`

**Verification**: 
- Unit test: given a known tag pose, verify the published goal_pose has correct TCP-corrected position
- Integration test: launch with planner, verify IK succeeds and trajectory is published

### Step 3: Implement adaptive orientation

**Dependencies**: Step 2 (extend the existing node)
**Tasks**:
- Add `orientation_mode="adaptive"` logic
- Compute approach vector from shoulder to target
- Generate valid orientation quaternion
- Add fallback: if first orientation fails (no trajectory within timeout), try alternatives

**Verification**: Test with tags at various positions (above, below, to the side) — verify IK success rate improves vs fixed orientation

### Step 4: Create `apriltag_reach.launch.py`

**Dependencies**: Steps 1-3
**Tasks**:
- Write launch file combining: robot.launch.py + RealSense + apriltag_ros + apriltag_to_goal_node + planner.launch.py + control.launch.py
- Add keyboard trigger option (subscribe to Empty, only publish goal on trigger)
- Parameterize: target_tag_id, tag_offset, tcp_offset, orientation_mode

**Verification**: Single `ros2 launch` command starts full pipeline, tag detection → arm reaches target

### Step 5: Remove YOLO code (cleanup)

**Dependencies**: Step 4 verified working
**Tasks**:
- Remove `ultralytics_detector.py` from install targets
- Remove `perception.launch.py` (or keep as legacy, mark deprecated)
- Remove `detection_to_goal_node.cpp` from build (or keep for other use cases)
- Update `reach.launch.py` to point to new pipeline (or deprecate)
- Remove YOLO model dependency from default launch args

**Verification**: `colcon build` succeeds, no broken imports or missing nodes

---

## Risk Assessment

| Risk | Mitigation |
|------|-----------|
| AprilTag detection range limited (~2-3m for small tags) | Use larger tags (15-20cm) or multiple tags |
| Tag partially occluded → no detection | Use multiple tags on target; require only 1 visible |
| TCP offset correction assumes rigid tool | Correct for this robot — TCP is fixed relative to wrist |
| Adaptive orientation may still fail IK | Fallback to multiple candidate orientations; log failures |
| `apriltag_ros` CPU load | Lightweight vs YOLO; runs on single RGB stream at 15fps |

---

## Summary of Changes

| Category | Files Changed | Effort |
|----------|--------------|--------|
| New Python node | `scripts/apriltag_to_goal_node.py` | Medium |
| New launch file | `launch/apriltag_reach.launch.py` | Low |
| New config | `config/apriltag_tags.yaml` | Low |
| Package metadata | `CMakeLists.txt`, `package.xml` | Trivial |
| System dependency | `apt install ros-humble-apriltag-ros` | Trivial |
| C++ planner | **No changes** | None |
| C++ executor | **No changes** | None |

**Total new code**: ~150-200 lines Python (apriltag_to_goal_node) + ~50 lines launch + config
**Modified code**: ~10 lines (CMakeLists + package.xml)
**Deleted code**: YOLO-related nodes can be removed after verification (optional cleanup)
