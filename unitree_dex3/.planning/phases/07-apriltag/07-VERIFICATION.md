---
phase: 7
status: passed
verified: 2026-05-18
verifier: kiro_default (inline orchestrator)
plans_verified:
  - 07-01
  - 07-02
  - 07-03
requirements:
  - TAG-01
  - TAG-02
  - TAG-03
  - TAG-04
must_haves_total: 67
must_haves_passed: 67
must_haves_failed: 0
human_verification_count: 4
---

# Phase 7 Verification: AprilTag 检测节点

## Verdict

**PASSED.** All 67 plan-level `<must_haves>` truths verified against
the actual repo state. All Phase 7 requirements (TAG-01 / TAG-02 /
TAG-03 / TAG-04) are traceable to source code and pass static checks.
Build verification and `ros2 launch --print` both succeed. Four items
require live hardware testing — listed under **Human verification**
below.

## Phase goal — does the codebase achieve it?

> *Implement AprilTag 36h11 detection, publish tag pose and transform
> to torso_link frame.*

Yes. The phase delivers:

- A self-contained Python rclpy node
  (`scripts/apriltag_detector_node.py`, 374 lines) that subscribes to
  the RealSense color stream + `CameraInfo`, runs pupil-apriltags
  with `estimate_tag_pose=True` for 6-DOF estimation, filters by
  `target_tag_id` + `hamming==0` + `decision_margin >= min`, applies
  a tag-local XYZ offset, transforms both the raw and offset poses
  to `torso_link` via tf2, and publishes them as
  `geometry_msgs/PoseStamped` on `/apriltag/tag_pose` and
  `/apriltag/target_pose`.
- A YAML parameter file (`config/apriltag.yaml`) declaring all 11
  tunable knobs from CONTEXT D-08, ready to be loaded by the launch.
- A standalone test launch (`launch/apriltag.launch.py`) that brings
  up the robot, RealSense (640x480x15, align_depth=false), the
  d435→camera_link static TF, and the detector in one
  `ros2 launch` command, with `imshow:=false` for headless mode.
- A README section documenting the pip install requirement and the
  launch examples.

## Plan-by-plan must_haves

### Plan 07-01 — build config + YAML (15/15 must_haves PASS)

| Truth | Result |
|-------|--------|
| `config/apriltag.yaml` exists | OK |
| top-level `apriltag_detector:` matches Plan 02 Node name | OK |
| nested `ros__parameters:` block | OK |
| all 11 D-08 fields declared with exact defaults | 11/11 |
| no `imshow` field in YAML (D-15 — launch arg only) | OK (count 0) |
| CMakeLists `install(PROGRAMS scripts/apriltag_detector_node.py …)` | OK |
| separate `install(DIRECTORY config …)` block (not merged with launch/robots) | OK (2 install(DIRECTORY) blocks total) |
| package.xml `<exec_depend>realsense2_camera</exec_depend>` | OK |
| package.xml `<exec_depend>python3-opencv</exec_depend>` | OK |
| no `pupil-apriltags` in package.xml (pip-only) | OK (count 0) |
| existing CMakeLists install entries preserved | OK (3 install(PROGRAMS) scripts visible) |
| existing package.xml depends preserved (cv_bridge, tf2_ros, tf2_geometry_msgs, OpenCV, vision_msgs) | 5/5 |

### Plan 07-02 — detector node (31/31 must_haves PASS)

| Truth | Result |
|-------|--------|
| script exists, executable, valid Python 3 | OK |
| `Node('apriltag_detector')` matches YAML key | OK |
| 12 declare_parameter calls (11 YAML + imshow launch arg) | 12/12 |
| `pupil_apriltags.Detector(families=self.tag_family)` | OK |
| `detect(estimate_tag_pose=True, camera_params=…, tag_size=…)` | OK |
| filter chain D-07: `d.tag_id == self.target_tag_id` | OK |
| filter chain D-11: `d.hamming == 0` | OK |
| filter chain D-10: `d.decision_margin >= self.decision_margin_min` | OK |
| tag-local offset D-01: `T_cam_target = T_cam_tag @ T_tag_target` | OK |
| `T_tag_target[:3,3] = self.offset_xyz` (no hardcoded literal) | OK |
| both poses through `tf_buffer.transform(…, output_frame, timeout=Duration(...))` | OK (3 calls — 2 in image_cb + Duration construction) |
| poses constructed with `header.frame_id = "camera_color_optical_frame"` | OK |
| `tag_pose_pub.publish` + `target_pose_pub.publish` only on accepted detections | OK (event-driven D-03) |
| `import tf2_geometry_msgs` (side-effect plugin registration) | OK |
| `tf2_ros.TransformException` caught + throttled warn (2 s) | OK |
| OpenCV viz: polylines (green/red), id text, 3-axis projectPoints, HUD | OK |
| `cv2.imshow` + `cv2.waitKey(1)` gated by `if display is not None:` (line 296) | OK (verified by structural read of lines 296–) |
| `display = bgr.copy() if self.imshow_enabled else None` (line ~188) → headless path skips namedWindow + imshow + waitKey | OK |
| FPS sliding window via `collections.deque(maxlen=30)` (D-20) | OK |
| `q` key sets `imshow_enabled = False` and calls `destroyWindow` | OK |
| sensor_data QoS on Image + CameraInfo subs (D-13) | OK (3 references — import + 2 subs) |
| no Phase-6 leakage (`0.175`, `right_wrist_yaw_link`, `right_tcp_link`) | 0 |
| no hardcoded tag_size / target_tag_id / decision_margin_min outside declare_parameter | OK |
| `colcon build --packages-select unitree_g1_dex3_stack` succeeds | rc=0, 1 package finished |
| `install/.../lib/.../apriltag_detector_node.py` present + executable | OK |
| `ros2 pkg executables unitree_g1_dex3_stack` lists `apriltag_detector_node.py` | OK |

### Plan 07-03 — launch + README (21/21 must_haves PASS)

| Truth | Result |
|-------|--------|
| `launch/apriltag.launch.py` exists, valid Python | OK |
| 4 `DeclareLaunchArgument` (urdf_name, urdf_path, config_file, imshow) | 5 hits — 4 declarations + 1 import |
| imports robot.launch.py + rs_launch.py via IncludeLaunchDescription | OK |
| `rgb_camera.profile=640x480x15` | OK |
| `depth_module.profile=640x480x15` | OK |
| `align_depth.enable=false` | OK |
| no 1280x720 (Phase 7 changed the profile from visual_detect_click) | 0 hits |
| `executable='static_transform_publisher'` with d435_link → camera_link | OK |
| `package='unitree_g1_dex3_stack' executable='apriltag_detector_node.py' name='apriltag_detector'` | OK |
| `parameters=[LaunchConfiguration('config_file'), {'imshow': LaunchConfiguration('imshow')}]` | OK |
| no rviz / planner / control / keyboard / model_path / target_class / goal_pose tokens | 0 (post-docstring rephrase) |
| no SetEnvironmentVariable / CYCLONEDDS_URI redeclaration (inherited from robot.launch.py) | 0 |
| no TimerAction wrapper | 0 |
| README has `## Phase 7: AprilTag …` heading | OK |
| README has `pip install pupil-apriltags` hint | OK |
| README has 2 `ros2 launch unitree_g1_dex3_stack apriltag.launch.py` examples (default + imshow:=false) | 2 |
| `colcon build --packages-select unitree_g1_dex3_stack` succeeds | rc=0 |
| `install/.../share/unitree_g1_dex3_stack/launch/apriltag.launch.py` exists | OK |
| `ros2 launch unitree_g1_dex3_stack apriltag.launch.py --print` rc=0; lists apriltag_detector + static_transform_publisher | OK |

## Requirement traceability (TAG-01..04)

| Req | Source | Verification |
|-----|--------|--------------|
| **TAG-01** detection node + 6-DOF publish | `scripts/apriltag_detector_node.py` lines ~225–246 build PoseStamped from `d.pose_t` + `R.from_matrix(d.pose_R).as_quat()`; lines ~290–291 publish | Static OK; live verification needed |
| **TAG-02** YAML configurable offset | `config/apriltag.yaml` `offset_xyz: [0.0, 0.0, 0.05]`; node reads via `declare_parameter('offset_xyz', …)` and applies `T_tag_target[:3,3] = self.offset_xyz` (no hardcode) | Static OK; live verification needed (change YAML → see new offset) |
| **TAG-03** TF camera → torso_link | `tf_buffer.transform(pose, self.output_frame, timeout=Duration(seconds=self.tf_lookup_timeout_s))` for both poses; `output_frame` defaults to `'torso_link'` (D-09); `import tf2_geometry_msgs` registers the PoseStamped plugin | Static OK; live tf2_echo needed |
| **TAG-04** filter rejection | 3-conjunct filter `d.tag_id == target AND d.hamming == 0 AND d.decision_margin >= min`; rejected detections skip publish (event-driven D-03); rejected tags drawn red in viz | Static OK; live behavior verification needed (low-margin tag → red polygon, no publish) |

## Cross-cutting CONTEXT decision conformance

D-01 / D-02 / D-03 / D-04 / D-07 / D-08 / D-09 / D-10 / D-11 / D-13 /
D-14 / D-15 / D-16 / D-17 / D-18 / D-20 — all reflected in source.
See Plan 07-02 SUMMARY for the per-decision mapping table.

D-05 / D-06 / D-12 / D-19 are handled at the launch / package level
or are explicit deferrals (rviz / planner integration is Phase 9).

## Human verification (live, hardware-dependent)

The following four checks need a physical robot + RealSense + a
printed `tag36h11` (id=0, 8 cm edge). They were **not** run during
phase execution because no hardware was available. They should be
run as a manual UAT pass before declaring v1.1 milestone shipping.

1. **TAG-01 live**: `ros2 launch unitree_g1_dex3_stack apriltag.launch.py`,
   place tag in front of camera, then
   `ros2 topic echo /apriltag/tag_pose --once`.
   Expected: `header.frame_id == 'torso_link'`, non-zero position,
   non-identity orientation.
2. **TAG-02 live**: change `offset_xyz` in
   `config/apriltag.yaml` to `[0.10, 0.0, 0.0]`, rebuild, relaunch,
   echo `/apriltag/target_pose --once`. Expected: `target_pose -
   tag_pose ≈ 0.10 m` along the tag's local X axis.
3. **TAG-03 live**: while launch is running,
   `ros2 run tf2_ros tf2_echo torso_link camera_color_optical_frame`.
   Expected: stable transform within ~5 s, no errors.
4. **TAG-04 live**: point camera away (or use a low-margin / blurry
   tag). Expected: `timeout 3 ros2 topic echo /apriltag/tag_pose
   --once` times out (no publish on rejected detections); OpenCV
   window shows red polygon for any visible-but-rejected tag.

## Known out-of-scope issues (informational, not phase blockers)

1. **Broken cmake shim at `/home/unitree/.local/bin/cmake`** —
   imports a non-existent `cmake` Python module. Workaround:
   `export PATH="/usr/bin:$PATH"` before `colcon build`. This is a
   pre-existing dev-env defect (created 2026-05-15), unrelated to
   Phase 7. Suggest filing a separate todo: either
   `pip install --upgrade cmake` to install the matching Python
   package, or `rm /home/unitree/.local/bin/cmake` to drop the shim
   entirely.
2. **Stale executables in install tree** — `project_to_3d_node`,
   `detection_to_goal_node`, `visual_detection_yolo_tester`,
   `ultralytics_detector.py`, `elevator_ocr_node.py` still appear
   in `ros2 pkg executables unitree_g1_dex3_stack`. CMakeLists.txt
   no longer references them (Phase 6 cleanup). A clean rebuild
   (`rm -rf build install && colcon build`) would clear them. Not
   in Phase 7 scope.
3. **Tracked build/install/log artifacts** — `build/`, `install/`,
   `log/` are gitignored at project root but were tracked from
   before the gitignore was added. Their `M` status in `git status`
   is build noise, not project change. Pre-existing; not in scope.

None of the above affect Phase 7 functional correctness.

## Commits delivering Phase 7 (9 total, atop `9858e29`)

| Commit | Subject |
|--------|---------|
| `97d8040` | feat(7-01): add config/apriltag.yaml with 11 D-08 detector params |
| `98f6b70` | build(7-01): install apriltag_detector_node.py + config dir |
| `fe2c140` | build(7-01): declare realsense2_camera + python3-opencv exec_depends |
| `ff36302` | docs(7-01): plan summary — build config + apriltag.yaml complete |
| `30ae6aa` | feat(7-02): apriltag_detector_node.py — full detect→filter→TF→publish pipeline |
| `0af4b67` | docs(7-02): plan summary — detector node + build verification complete |
| `d8d6f13` | feat(7-03): apriltag.launch.py — standalone detector test launch |
| `88b61bd` | docs(7-03): README — Phase 7 AprilTag section |
| `58b7be5` | docs(7-03): plan summary — launch + README complete; Phase 7 deliverable closed |
