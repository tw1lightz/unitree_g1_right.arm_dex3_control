---
status: resolved
trigger: "为什么这个矩阵和实际测量是差不多的，但是用这个项目里的yolo detect launch检测一个物品并计算到pelvis系下的坐标，差别就很大？"
created: 2026-04-17T11:45:28+08:00
updated: 2026-04-17T13:47:00+08:00
---

## Current Focus
hypothesis: project_to_3d_node may be publishing camera-frame centroids with a rewritten frame_id, and even on the default path the centroid over the whole 2D bbox may be badly biased by background depth or stale detections.
test: Inspect perception launch defaults and the full detection-to-3D-to-pelvis conversion path, including whether tf is actually applied and whether 2D detections are synchronized with depth.
expecting: If tf is missing, output_frame:=pelvis will produce grossly wrong coordinates; if tf is correct on the default path, bbox-depth aggregation or timing mismatch should explain the residual large error.
next_action: verify whether Detection3D positions are transformed or only relabeled, then quantify the remaining projection bias sources in the default visual detection launch flow
reasoning_checkpoint: null
tdd_checkpoint: null

## Symptoms
expected: The object position reported in pelvis coordinates should be close to manual measurement when using the project's YOLO detection pipeline, since the camera-to-pelvis extrinsic from URDF is already close to the measured transform.
actual: The standalone camera-to-pelvis matrix is close to reality, but the object position produced by the YOLO detect launch and interpreted in pelvis coordinates differs a lot from manual measurement.
errors: none reported
reproduction: Launch the project's YOLO perception flow, detect an object, inspect the computed 3D position in pelvis coordinates, and compare it against manual measurement.
started: currently under investigation; unknown whether it ever matched well

## Eliminated

## Evidence
- timestamp: 2026-04-17T11:45:28+08:00
  checked: src/unitree_g1_dex3_stack-main/src/project_to_3d_node.cpp
  found: 3D points and centroids are computed directly from camera intrinsics in the RGB/depth image plane, while detection_array.header.frame_id, detection.header.frame_id, and cloud_msg.header.frame_id are overwritten to output_frame_ with no tf lookup or transform application.
  implication: If output_frame is set to pelvis, the node will label camera-frame coordinates as pelvis-frame coordinates, causing large pose errors.
- timestamp: 2026-04-17T11:45:28+08:00
  checked: src/unitree_g1_dex3_stack-main/launch/visual_detection_test.launch.py and src/unitree_g1_dex3_stack-main/src/visual_detection_tester.cpp
  found: The visual detection test launch forces output_frame to camera_color_optical_frame, and the tester later transforms the selected Detection3D pose to pelvis via tf2.
  implication: The default test path avoids the frame relabel bug, so any large remaining error there must come from projection quality, timing mismatch, or bbox/depth aggregation rather than the URDF extrinsic alone.
- timestamp: 2026-04-17T11:45:28+08:00
  checked: src/unitree_g1_dex3_stack-main/src/project_to_3d_node.cpp
  found: The node uses the latest 2D detections from a separate subscription without synchronizing them to the rgb/depth callback, and it computes the 3D target as the centroid of all valid depth pixels inside the full 2D bbox.
  implication: Background/table pixels or stale detections can shift the centroid substantially even when the camera-to-pelvis extrinsic is correct.
- timestamp: 2026-04-17T13:47:00+08:00
  checked: src/unitree_g1_dex3_stack-main/src/visual_detection_tester.cpp and src/YOLOX-ROS/yolox_ros_cpp/yolox_ros_cpp/src/yolox_ros_cpp.cpp
  found: YOLOX publishes bounding boxes with the source image header stamp, but visual_detection_tester overwrote the detection stamp with Time(0) before transforming to pelvis, forcing tf2 to use the latest transform instead of the capture-time transform.
  implication: Even when project_to_3d_node publishes camera-frame coordinates correctly, any robot or camera motion between image capture and pressing s can create large pelvis-frame drift.

## Resolution
root_cause: project_to_3d_node had two frame/timing defects in the YOLO path: it could relabel camera-frame detections and point clouds as output_frame without applying tf, and it reused the latest bbox message asynchronously instead of pairing detections with the matching rgb/depth frame; on the default visual_detection_test path, visual_detection_tester then threw away the detection timestamp and transformed with the latest tf, adding more pelvis drift when the robot/camera moved.
fix: project_to_3d_node now synchronizes rgb, depth, camera_info, and YOLO bbox messages in one ApproximateTime callback and applies a real tf transform before publishing into a non-camera output_frame; visual_detection_tester now preserves the detection timestamp instead of forcing Time(0).
verification: `colcon build --packages-select unitree_g1_dex3_stack` succeeded in `/home/unitree/Desktop/unitree_dex3`; live sensor/robot validation is still needed to quantify any remaining bbox-depth centroid bias.
files_changed: ["src/unitree_g1_dex3_stack-main/src/project_to_3d_node.cpp", "src/unitree_g1_dex3_stack-main/src/visual_detection_tester.cpp", "src/unitree_g1_dex3_stack-main/CMakeLists.txt"]
