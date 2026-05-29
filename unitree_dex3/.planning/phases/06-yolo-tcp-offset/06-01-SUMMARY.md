---
phase: 06-yolo-tcp-offset
plan: 06-01
type: summary
status: completed
completed_at: "2026-05-15T18:46:33+08:00"
requirements:
  - CLEAN-01
artifacts:
  - src/unitree_g1_dex3_stack-main/CMakeLists.txt
  - src/unitree_g1_dex3_stack-main/package.xml
  - src/unitree_g1_dex3_stack-main/launch/reach.launch.py
verification:
  - static_check
  - colcon_build
---

# Summary: Phase 06 Plan 01 — YOLO Cleanup

## Result

Plan 06-01 is delivered. All YOLO detection code, the `bboxes_ex_msgs` ROS message package, the perception launch files, the YOLO weights file, and the helper shell wrapper are gone from the source tree. `CMakeLists.txt` and `package.xml` no longer reference any deleted nodes or packages. `reach.launch.py` is reduced to robot + d435_tf + planner + control. `colcon build --packages-select unitree_g1_dex3_stack` succeeds.

## Implementation

This plan was executed and squashed into a single phase-wide commit:

- **Phase commit:** `d67ee62` — `feat(phase-6): YOLO cleanup + TCP offset integration`

### Files deleted

- `src/unitree_g1_dex3_stack-main/scripts/ultralytics_detector.py`
- `src/unitree_g1_dex3_stack-main/src/project_to_3d_node.cpp`
- `src/unitree_g1_dex3_stack-main/src/detection_to_goal_node.cpp`
- `src/unitree_g1_dex3_stack-main/src/visual_detection_yolo_tester.cpp`
- `src/unitree_g1_dex3_stack-main/launch/perception.launch.py`
- `src/unitree_g1_dex3_stack-main/launch/visual_detect_yolo.launch.py`
- `src/bboxes_ex_msgs/` (entire package: `package.xml`, `CMakeLists.txt`, `msg/BoundingBox.msg`, `msg/BoundingBoxes.msg`)
- `best.pt` (project root)
- `run_perception.sh` (project root)
- `.windsurfrules` (project root, unrelated YOLO-era artifact)

### Files modified

- `src/unitree_g1_dex3_stack-main/CMakeLists.txt`
  - Removed `find_package(bboxes_ex_msgs)`, `find_package(image_transport)`, `find_package(pcl_conversions)`, `find_package(PCL)`.
  - Removed `${PCL_INCLUDE_DIRS}` from `include_directories(...)`.
  - Removed `add_executable` and `ament_target_dependencies` for `project_to_3d_node`, `detection_to_goal_node`, `visual_detection_yolo_tester`.
  - Removed `target_link_libraries(project_to_3d_node ...)` (PCL/OpenCV).
  - Removed all three deleted targets from `install(TARGETS ...)`.
  - Removed `scripts/ultralytics_detector.py` from `install(PROGRAMS ...)`.
  - Build option `BUILD_IK_FCL_OMPL_PLANNER` was added during the same commit and gates `find_package(trac_ik_lib | ompl | fcl | geometric_shapes | resource_retriever)`, `add_executable(ik_fcl_ompl_planner)`, `ament_target_dependencies(ik_fcl_ompl_planner)`, `target_link_libraries(ik_fcl_ompl_planner)`, and `install(TARGETS ik_fcl_ompl_planner)`. This is a deviation from the plan (see Deviations).
- `src/unitree_g1_dex3_stack-main/package.xml`
  - Removed `<depend>bboxes_ex_msgs</depend>`, `<depend>image_transport</depend>`, `<depend>pcl_conversions</depend>`, `<depend>pcl_msgs</depend>`.
- `src/unitree_g1_dex3_stack-main/launch/reach.launch.py`
  - Rewrote to launch only: `planning_timeout_arg`, `robot.launch.py`, `d435_tf_node` static publisher, `TimerAction(period=3.0, [planner.launch.py, control.launch.py])`.
  - Removed `model_path_arg`, `target_class_arg`, `imshow_arg`, `perception.launch.py` include, `keyboard_trigger_node`.

## Verification

All Plan 06-01 acceptance criteria pass against the post-`d67ee62` working tree:

- `ls` of every deleted file/directory: returns "No such file or directory" (11 / 11).
- `grep -c "bboxes_ex_msgs|project_to_3d_node|detection_to_goal_node|visual_detection_yolo_tester|PCL|image_transport|pcl_conversions|ultralytics_detector"` against `CMakeLists.txt`: 0 / 0 (8 patterns).
- `grep -c "bboxes_ex_msgs|image_transport|pcl_conversions|pcl_msgs"` against `package.xml`: 0 / 0 (4 patterns).
- `grep -c "perception|model_path|target_class|imshow|keyboard_trigger"` against `reach.launch.py`: 0 / 0 (5 patterns).
- `grep` for `visual_detection_tester`, `find_package(OpenCV`, `vision_msgs`, `cv_bridge`, `static_transform_publisher`, `TimerAction` returns matches (kept dependencies / structures present).
- `colcon build --packages-select unitree_g1_dex3_stack --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON` exits 0; binaries `ik_fcl_ompl_planner` and `visual_detection_tester` produced.

## Requirements Satisfied

- **CLEAN-01:** YOLO 检测相关代码已从 launch、依赖、源码树彻底移除。

## Deviations from Plan

### [Rule 1 — bug-fix during execution] BUILD_IK_FCL_OMPL_PLANNER opt-in gate

- **Issue:** Plain `colcon build --packages-select unitree_g1_dex3_stack` was failing on machines without `trac_ik_lib`, `ompl`, `fcl`, `geometric_shapes`, or `resource_retriever` installed (these are heavyweight planner dependencies that aren't always available on a fresh G1 development machine).
- **Fix:** Added `option(BUILD_IK_FCL_OMPL_PLANNER ... OFF)` and gated the planner's `find_package`, `add_executable`, `ament_target_dependencies`, `target_link_libraries`, and `install(TARGETS)` blocks behind that flag.
- **Impact on Plan 06-01 acceptance:** None — the task's `colcon build` smoke test still succeeds (default OFF skips planner). The intent of the criterion is preserved.
- **Impact on Plan 06-03 acceptance:** Plan 06-03 Task 3's "the ik_fcl_ompl_planner target must compile cleanly" is now reached via `colcon build ... -DBUILD_IK_FCL_OMPL_PLANNER=ON`, which has been verified to exit 0.

### Out-of-plan additions

- `BUILD_NEW_MACHINE.md` (new docs file at project root explaining how to set up a new G1 development machine post-cleanup) and `CLAUDE.md` updates were made in the same commit. These are documentation, not in any 06-* plan, but were bundled into `d67ee62`.
- `test_button_ocr.py` and `yolo_last_detection.jpg` are present at the project root. These are not in Plan 06-01's deletion list (they were not enumerated in the plan), and are harmless dangling artifacts. Deferred to a future cleanup if desired.

## Notes

- The original execution path bundled all three Phase 6 plans (06-01, 06-02, 06-03) into the single commit `d67ee62`. This SUMMARY.md was authored retrospectively at phase close-out time after the `safe_resume_gate` detected that production commits existed but per-plan SUMMARY.md files had never been written.
- `keyboard_trigger_node.py` and `tcp_torso_pose.py` remain installed via `install(PROGRAMS ...)` per the plan's "keep harmless" guidance.
