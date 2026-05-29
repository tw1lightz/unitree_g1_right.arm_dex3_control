# Features Research: v1.1

## AprilTag Detection

### Table Stakes
- Detect tag36h11 from D435i color stream
- Estimate 6-DOF tag pose (position + orientation) in camera frame
- Apply configurable XYZ offset from tag center to actual object position
- Transform tag pose from camera frame to torso_link via TF
- Publish goal pose for planner consumption

### Differentiators
- Multi-tag support (detect multiple tags, select by ID)
- Detection confidence filtering (reject ambiguous detections)
- Pose smoothing (temporal filter to reduce jitter)

### Edge Cases
- Tag partially occluded → detection fails or pose inaccurate
- Tag at extreme angle (>60°) → pose estimation degrades
- IR interference from D435i depth projector → disable depth projector during detection or use color-only
- Tag too far (>1.5m) or too close (<0.2m) → detection unreliable
- Multiple tags in view → need ID-based selection

## TCP Offset Integration

### Table Stakes
- Apply inverse TCP offset to goal pose before IK solving
- Offset direction: +X in wrist_yaw_link local frame (0.175m)
- Result: IK solves for wrist position such that TCP reaches target

### Differentiators
- Configurable offset via ROS parameter (not hardcoded)
- Offset validation (reject if offset makes target unreachable)

### Edge Cases
- Offset applied in wrong frame (world vs local) → completely wrong position
- Offset direction confusion (should be subtracted from target, not added)
- Near workspace boundary: target reachable but wrist position (target - offset) is not
- Collision check must consider TCP extension (arm + offset volume)

## Adaptive Orientation

### Table Stakes
- Compute feasible end-effector orientation based on target position relative to shoulder
- Orientation that avoids IK singularities (overhead, fully extended)
- At minimum: approach direction pointing from shoulder toward target

### Differentiators
- Multiple candidate orientations tried if first fails IK
- Orientation derived from tag normal (approach perpendicular to surface)
- Smooth orientation transition between consecutive targets

### Edge Cases
- Target directly above shoulder → singularity, any fixed orientation fails
- Target at workspace boundary → very few feasible orientations
- Orientation constraint too tight → OMPL planning time explodes
- Orientation too loose → arm arrives in awkward/unsafe pose

## Complexity Assessment

| Feature | Complexity | Risk |
|---------|-----------|------|
| AprilTag detection | Low | Low — well-understood library |
| Configurable offset | Low | Low — pure math |
| TCP offset in planner | Medium | Medium — frame convention errors |
| Adaptive orientation | Medium-High | High — IK success depends on strategy |
| YOLO removal | Low | Low — deletion only |
