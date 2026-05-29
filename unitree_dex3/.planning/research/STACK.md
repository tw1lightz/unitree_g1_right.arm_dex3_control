# Stack Research: v1.1 Additions

## New Dependencies

| Library | Version | Purpose | Integration Point |
|---------|---------|---------|-------------------|
| `pupil-apriltags` | `==1.0.4.post11` | AprilTag 36h11 detection + pose estimation. Bundles apriltag3 C library, no external deps. | Subscribes to D435i color image + camera_info; publishes tag pose as PoseStamped |
| `scipy` | `>=1.10.0` | `Rotation` for rotation math, `Slerp` for orientation interpolation | Used in detection node (pose_R → quat) and adaptive orientation computation |
| `numpy` | `>=1.24.0` | Array operations for tag pose processing | Already present as OpenCV/PyKDL dep, pin minimum |

## Configuration Files Needed

1. **AprilTag parameters YAML** (`config/apriltag_params.yaml`):
   ```yaml
   apriltag_detector:
     ros__parameters:
       tag_family: "tag36h11"
       tag_size: 0.05          # meters - physical tag size
       target_offset_x: 0.0    # configurable offset from tag to object
       target_offset_y: 0.0
       target_offset_z: 0.0
   ```

2. **TCP offset parameter** in planner:
   ```yaml
   tcp_offset_x: 0.175  # meters, +X from right_wrist_yaw_link
   ```

## What NOT to Add

| Library/Package | Reason |
|-----------------|--------|
| `apriltag_ros` / `apriltag_ros2` | Heavyweight ROS wrapper; pupil-apriltags called directly is simpler |
| `pyapriltags` | Less maintained fork of dt-apriltags |
| `dt-apriltags` | Predecessor to pupil-apriltags, no longer updated |
| `transforms3d` | scipy.spatial.transform.Rotation covers all needs |
| `pytracik` (pip) | Project already builds trac_ik from source as ROS 2 C++ package |
| Additional URDF TCP link | Not needed — offset applied as inverse transform on IK goal |

## Integration Notes

- Detection node: standalone Python ROS 2 node using cv_bridge + pupil-apriltags
- TCP offset: apply inverse offset in C++ planner (~3 lines change), no new library
- Adaptive orientation: scipy Rotation in Python goal composer node
- Install: `pip install "pupil-apriltags==1.0.4.post11" "scipy>=1.10.0"`
