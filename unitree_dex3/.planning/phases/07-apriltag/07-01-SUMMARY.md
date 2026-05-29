---
phase: 7
plan: 01
status: complete
completed: 2026-05-18
requirements:
  - TAG-02
commits:
  - 97d8040
  - 98f6b70
  - fe2c140
key-files:
  created:
    - src/unitree_g1_dex3_stack-main/config/apriltag.yaml
  modified:
    - src/unitree_g1_dex3_stack-main/CMakeLists.txt
    - src/unitree_g1_dex3_stack-main/package.xml
---

# Plan 07-01 Summary: 构建配置 + YAML 参数文件

## What was built

Build scaffolding for the Phase 7 AprilTag detector. Three additive
changes only — no existing build target, find_package, install entry,
or dependency was removed or altered.

### 1. `config/apriltag.yaml` (new file, 28 lines)

Top-level YAML key `apriltag_detector:` with a single nested
`ros__parameters:` block, declaring all 11 fields from CONTEXT D-08:

| Field | Default | Purpose |
|------|---------|---------|
| `tag_family` | `"tag36h11"` | pupil-apriltags Detector family |
| `tag_size` | `0.08` | physical edge length (meters) |
| `target_tag_id` | `0` | only this id is published (D-07) |
| `offset_xyz` | `[0.0, 0.0, 0.05]` | tag-local frame offset (D-01) |
| `decision_margin_min` | `25.0` | quality gate (D-10) |
| `output_frame` | `"torso_link"` | TF target frame for both poses (D-09) |
| `rgb_topic` | `"/camera/color/image_raw"` | RealSense color stream |
| `camera_info_topic` | `"/camera/color/camera_info"` | intrinsics for PnP |
| `tag_pose_topic` | `"/apriltag/tag_pose"` | raw tag center publish |
| `target_pose_topic` | `"/apriltag/target_pose"` | offset-applied publish |
| `tf_lookup_timeout_s` | `0.5` | tf_buffer.transform timeout |

Inline comment headers group fields (detection / target / offset /
filtering / topics / TF) for readability. **No `imshow` field** —
that is a launch arg, not a YAML param (D-15). YAML is valid (passed
`yaml.safe_load`).

### 2. `CMakeLists.txt` (additive edits)

- `install(PROGRAMS scripts/apriltag_detector_node.py ...)` — added
  to the existing block after `keyboard_trigger_node.py`. The script
  itself is created in Plan 07-02; install will be exercised in Plan
  07-02 Task 2's `colcon build` verification.
- New separate `install(DIRECTORY config DESTINATION
  share/${PROJECT_NAME})` block, added **after** the existing
  `launch/robots` block (kept distinct, matching the Phase 6
  incremental-change convention from `07-PATTERNS.md`).

All existing C++ targets (`joint_state_publisher`, `dex3_controller`,
`joint_trajectory_executor`, `right_hand_pressure_monitor`,
`visual_detection_tester`), the `BUILD_IK_FCL_OMPL_PLANNER`
conditional block, and every `find_package` line are preserved
verbatim.

### 3. `package.xml` (additive edits)

Two new `<exec_depend>` entries inserted after `tf2_geometry_msgs`,
under a comment banner `<!-- AprilTag detector runtime deps (Phase 7) -->`:

- `<exec_depend>realsense2_camera</exec_depend>` — pulled in for
  the `rs_launch.py` include in Plan 07-03's launch file.
- `<exec_depend>python3-opencv</exec_depend>` — Python OpenCV
  bindings used by the detector node for `cv2.imshow`,
  `cv2.projectPoints`, etc.

`pupil-apriltags` is intentionally **not** declared here — it is
pip-only and not in rosdep. Its install hint goes into README.md in
Plan 07-03 Task 2.

## Deviations from plan

None. All `<must_haves>` truths and all 9/14/16 acceptance-criteria
checks pass on the first attempt.

## Self-Check: PASSED

- 16/16 YAML acceptance-criteria pass (file existence, header, all 11
  fields with exact defaults, no imshow, valid YAML).
- 8/8 CMakeLists.txt acceptance-criteria pass (3 install(PROGRAMS)
  scripts, 2 install(DIRECTORY) blocks with config in its own block,
  ament_cmake/visual_detection_tester/BUILD_IK_FCL_OMPL_PLANNER all
  preserved).
- 10/10 package.xml acceptance-criteria pass (2 new exec_depends,
  no pupil-apriltags, 5 existing depends preserved, valid XML by
  both Python `ET.parse` and `xmllint`).

## What this enables

- Plan 07-02 can now `Write` `scripts/apriltag_detector_node.py` and
  expect `colcon build --packages-select unitree_g1_dex3_stack` to
  install it via `install(PROGRAMS)`.
- Plan 07-02's node can call `self.declare_parameter(...)` for the
  exact 11 keys this YAML declares, and Plan 07-03's launch can pass
  `parameters=[<this yaml>]` to load them all in one shot.
- Plan 07-03's launch can `IncludeLaunchDescription(rs_launch.py)`
  because `realsense2_camera` is now declared as a runtime dep.

## Commits

- `97d8040 feat(7-01): add config/apriltag.yaml with 11 D-08 detector params`
- `98f6b70 build(7-01): install apriltag_detector_node.py + config dir`
- `fe2c140 build(7-01): declare realsense2_camera + python3-opencv exec_depends`
