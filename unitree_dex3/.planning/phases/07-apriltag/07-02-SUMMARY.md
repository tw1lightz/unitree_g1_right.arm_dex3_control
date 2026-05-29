---
phase: 7
plan: 02
status: complete
completed: 2026-05-18
requirements:
  - TAG-01
  - TAG-03
  - TAG-04
commits:
  - 30ae6aa
key-files:
  created:
    - src/unitree_g1_dex3_stack-main/scripts/apriltag_detector_node.py
  modified: []
---

# Plan 07-02 Summary: AprilTag 检测节点实现

## What was built

A single-file rclpy node (`scripts/apriltag_detector_node.py`, 374
lines, executable Python 3) implementing the full Phase 7 detection
pipeline. Build verification passes against the install scaffolding
laid down by Plan 07-01.

## Pipeline — `image_cb` step-by-step

1. **FPS bookkeeping (D-20)** — `collections.deque(maxlen=30)` of
   `time.monotonic()` samples; HUD shows `(N-1)/(t[-1]-t[0])`.
2. **Camera-info gate** — first frame after `info_cb` arrives. Until
   then, throttled (5 s) warn `Waiting for CameraInfo on …` and skip.
3. **cv_bridge → BGR8** — `desired_encoding='bgr8'`. `CvBridgeError`
   caught, throttled warn, frame dropped.
4. **`cv2.cvtColor` BGR→GRAY** — input to pupil-apriltags.
5. **`detector.detect`** — `families=self.tag_family`,
   `estimate_tag_pose=True`, `camera_params=(fx,fy,cx,cy)`,
   `tag_size=self.tag_size`. Detector built once in `__init__`.
6. **Filter chain (D-07 + D-11 + D-10)**:
   `accepted = (d.tag_id == self.target_tag_id) and (d.hamming == 0)
   and (d.decision_margin >= self.decision_margin_min)`.
7. **Visualization (only if `imshow_enabled`)**:
   - `cv2.polylines` with green BGR for accepted / red for rejected.
   - `cv2.putText` tag id label above corner 0.
   - For accepted only: `cv2.projectPoints` of 3 cm axes — x red,
     y green, z blue — drawn as 3 lines via `cv2.line`.
8. **Skip publish for rejected** — D-03 event-driven semantics.
9. **Raw tag PoseStamped in `camera_color_optical_frame`** —
   `R.from_matrix(d.pose_R).as_quat()` for orientation,
   `d.pose_t.reshape(3)` for position.
10. **Tag-local offset (D-01)** —
    `T_cam_target = T_cam_tag · Translate(self.offset_xyz)` using
    plain numpy 4×4 matrices; quaternion extracted from
    `T_cam_target[:3,:3]`. `offset_xyz` comes straight from
    parameter, never hardcoded.
11. **TF transform to `output_frame` (D-09 + D-04)** — both raw and
    target poses through `self.tf_buffer.transform(pose,
    self.output_frame, timeout=Duration(seconds=tf_lookup_timeout_s))`.
    `tf2_ros.TransformException` caught; throttled (2 s) warn; per-
    detection skip without crashing the node.
12. **Publish** — `tag_pose_pub.publish(pose_torso)` and
    `target_pose_pub.publish(target_torso)`. Both publishers use
    default reliable QoS, depth 10.
13. **HUD + window blit** — text rendered with a black background
    rectangle for legibility; bottom-right anchored. `cv2.imshow` +
    `cv2.waitKey(1)`. Pressing `q` runs `cv2.destroyWindow` and sets
    `imshow_enabled = False` — node continues publishing headlessly.

## Locked CONTEXT decisions reflected in source

| Decision | Where in source |
|----------|----------------|
| D-01 offset semantics (tag-local) | `T_cam_tag @ T_tag_target` |
| D-02 dual topics | `tag_pose_pub` + `target_pose_pub` |
| D-03 event-driven | `if not accepted: continue` before publish |
| D-04 throttled warn on TF failure | `_warn_throttled` 2 s window |
| D-07 target_tag_id filter | filter expression |
| D-08 11 YAML fields | 11 `declare_parameter` calls |
| D-09 frame_id rewrite via tf2 | `tf_buffer.transform → output_frame` |
| D-10 decision_margin_min filter | filter expression |
| D-11 hamming==0 filter | filter expression |
| D-13 sensor_data QoS for sensor topics | both subs use `qos_profile_sensor_data` |
| D-14 tf2_geometry_msgs side-effect import | `import tf2_geometry_msgs  # noqa` |
| D-15 imshow as launch arg | declared as parameter, defaulted true |
| D-16 single-threaded executor | plain `rclpy.spin(node)` in main |
| D-17 cv2.WINDOW_NORMAL + resize | `namedWindow + resizeWindow(640,480)` |
| D-18 viz: corners + axes + HUD + quit-on-q | implemented exactly as spec |
| D-20 FPS sliding window 30 | `deque(maxlen=30)` |

## Build verification (Task 2)

`colcon build --packages-select unitree_g1_dex3_stack` exits 0,
`1 package finished`, no failures.

| Check | Result |
|-------|--------|
| build rc | 0 |
| `install/…/lib/unitree_g1_dex3_stack/apriltag_detector_node.py` exists & exec | OK |
| `install/…/share/unitree_g1_dex3_stack/config/apriltag.yaml` exists | OK |
| `ros2 pkg executables unitree_g1_dex3_stack` lists `apriltag_detector_node.py` | 1 hit |

### Environment workaround

The user's `/home/unitree/.local/bin/cmake` is a broken Python shim
that imports a non-existent `cmake` module
(`ModuleNotFoundError: No module named 'cmake'`). It shadows
`/usr/bin/cmake` because `~/.local/bin` precedes `/usr/bin` in PATH.
Workaround applied for this build: `export PATH="/usr/bin:$PATH"`
before invoking `colcon build`.

This is a pre-existing environment issue (the shim was created on
2026-05-15) and is **out of scope** for Phase 7. It should be flagged
as a separate dev-env todo (e.g. `pip install --upgrade cmake` or
remove the shim).

### Stale install artifacts (informational)

`ros2 pkg executables` still shows `project_to_3d_node`,
`detection_to_goal_node`, `visual_detection_yolo_tester`,
`ultralytics_detector.py`, `elevator_ocr_node.py` — these are leftover
from earlier builds before Phase 6's YOLO cleanup. CMakeLists.txt no
longer references them, so a clean `rm -rf build install && colcon
build` would remove them. Not in Phase 7 scope; flagged for awareness.

## Deviations from plan

None. All `<must_haves>`, all 29 source acceptance checks, and all 5
build acceptance checks pass.

## Self-Check: PASSED

- `python3 -c "import ast; ast.parse(...)"` — valid syntax.
- All 11 YAML keys + `imshow` declared (12 `declare_parameter` calls).
- Filter chain present (3 conjuncts).
- Both `tf_buffer.transform` calls present; `TransformException`
  caught.
- Both publishers wired; `import tf2_geometry_msgs` side-effect import
  present.
- OpenCV draws all present (polylines, projectPoints, putText, imshow,
  waitKey, ord('q')).
- `deque(maxlen=30)` present.
- No Phase-6 leakage (`0.175`, `right_wrist_yaw_link`, `right_tcp_link`
  count = 0).
- No hardcoded `tag_size`, `target_tag_id`, etc. outside
  `declare_parameter`.

## What this enables

- Plan 07-03 can wire the YAML + the launch arg `imshow` into a Node
  action via `parameters=[<yaml>, {'imshow': LaunchConfiguration(...)}]`
  and the node will respond to both.
- Phase 8 can subscribe to `/apriltag/target_pose` and use the pose to
  compute adaptive end-effector orientation (ORI-01).
- Phase 9 can compose the full pipeline launch.

## Commits

- `30ae6aa feat(7-02): apriltag_detector_node.py — full detect→filter→TF→publish pipeline`
