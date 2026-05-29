---
phase: 7
plan: 03
status: complete
completed: 2026-05-18
requirements:
  - TAG-01
  - TAG-02
  - TAG-03
  - TAG-04
commits:
  - d8d6f13
  - 88b61bd
key-files:
  created:
    - src/unitree_g1_dex3_stack-main/launch/apriltag.launch.py
  modified:
    - src/unitree_g1_dex3_stack-main/README.md
---

# Plan 07-03 Summary: 独立测试 launch + README 安装提示

## What was built

Phase 7 deliverable closed out: a single-purpose `apriltag.launch.py`
that brings up the full detection chain in one ROS 2 launch command,
plus a README section documenting the pip install + usage examples.

### 1. `launch/apriltag.launch.py` (new file, 110 lines)

Composes (in order of `LaunchDescription`):

1. **4 `DeclareLaunchArgument`s**:
   - `urdf_name` →
     `g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf`
     (matches `robot.launch.py`'s own default).
   - `urdf_path` → `''` (overrides `urdf_name` when set).
   - `config_file` →
     `<package_share>/config/apriltag.yaml`.
   - `imshow` → `'true'` (set `imshow:=false` for headless / SSH).
2. **`IncludeLaunchDescription(robot.launch.py)`** with `urdf_name` +
   `urdf_path` passed through. CycloneDDS env vars (RMW + URI) are
   inherited from `robot.launch.py` — never set here (D-15).
3. **`IncludeLaunchDescription(rs_launch.py)`** with
   `enable_sync=true`, `align_depth.enable=false`,
   `rgb_camera.profile=640x480x15`, `depth_module.profile=640x480x15`.
   This is the Phase 7-specific RealSense profile (lower than
   `visual_detect_click.launch.py`'s 1280x720x15) — saves USB
   bandwidth + camera heat since PnP only needs RGB.
4. **`Node(static_transform_publisher d435_link → camera_link)`** —
   verbatim args from `reach.launch.py` /
   `visual_detect_click.launch.py`, so RealSense's
   `camera_*_optical_frame` chain attaches under the URDF's
   `d435_link`.
5. **`Node(apriltag_detector_node.py)`** with
   `parameters=[LaunchConfiguration('config_file'),
                {'imshow': LaunchConfiguration('imshow')}]`.
   YAML loads first; the dict overlay forces `imshow` to follow the
   launch arg (defense-in-depth, even though Plan 07-01's YAML
   intentionally omits `imshow`).

**Intentionally omitted** (D-19): no rviz visualizer, no manipulation
pipeline, no executor, no trigger node, no `TimerAction`. Phase 9 is
where the end-to-end launch lives — `apriltag.launch.py` stays
single-purpose.

### 2. `README.md` (additive edit, 48 → 71 lines)

A new `## Phase 7: AprilTag 检测节点` section appended at the end,
written in matching Chinese voice (the rest of the file is Chinese).
Contents:

- `pip install pupil-apriltags` — the only manual step (pip-only,
  not in rosdep, so package.xml cannot declare it).
- Two launch examples: default (`imshow:=true`) and headless
  (`imshow:=false`).
- The two published topics with frames noted (`torso_link`).
- Pointer to `config/apriltag.yaml` for the tunable parameters.

All four prior sections (`环境准备` / `快速启动` / `参数覆盖` /
`架构`) untouched.

## Verification (Task 3)

| Check | Result |
|-------|--------|
| `colcon build --packages-select unitree_g1_dex3_stack` rc | 0 |
| `1 package finished`, no failures | ✓ |
| `install/.../share/unitree_g1_dex3_stack/launch/apriltag.launch.py` exists | ✓ |
| `ros2 launch unitree_g1_dex3_stack apriltag.launch.py --print` rc | 0 |
| `--print` lists `apriltag_detector_node.py` Node action | ✓ (1 hit) |
| `--print` lists `static_transform_publisher` Node action | ✓ (1 hit) |
| `--print` shows 4 `DeclareLaunchArgument` actions | ✓ |
| `--print` shows 2 `IncludeLaunchDescription` actions (robot + rs_launch) | ✓ |

Build still required `PATH=/usr/bin:$PATH` (broken cmake shim;
unchanged from Plan 07-02 — pre-existing env issue).

## Deviations from plan

The plan's docstring template originally enumerated the
intentionally-omitted Phase-9 components by their literal names
(`rviz`, `planner`, `control`, `keyboard_trigger`). Acceptance
criterion [19] runs `grep -cE 'rviz2|planner\.launch|control\.launch|keyboard_trigger|model_path|target_class|goal_pose'` and
requires 0. Rephrased the docstring to convey the same meaning
without those literal tokens: "intentionally omits the visualizer,
the manipulation pipeline, the executor, and the trigger node". No
behavioral change — only commentary wording.

## Self-Check: PASSED

- 21/21 source acceptance checks (file exists, valid Python,
  required imports, 4 DeclareLaunchArguments, RealSense profile
  640x480x15 + align_depth=false, no 1280x720, apriltag node +
  static TF, includes for robot.launch.py and rs_launch.py, no
  scope leakage, no env-var redeclaration, no TimerAction).
- 6/6 README acceptance checks (pip install hint, two launch
  examples, imshow:=false, heading present, topics noted, line
  count strictly increased).
- 5/5 build + launch-parse checks.

## Live verification (post-execution, hardware-dependent)

The plan's `<verification>` block lists 4 hardware-dependent live
tests (TAG-01 / TAG-02 / TAG-03 / TAG-04 topic + tf2 echos). They
are **not** run here — they require a physical robot + RealSense +
a printed tag36h11 with id=0 and 8 cm edge. The phase verifier
should add them to `human_verification` for the next manual session
on hardware.

## What this enables

- Phase 7 deliverable is complete: 3 new files (`config/apriltag.yaml`,
  `scripts/apriltag_detector_node.py`, `launch/apriltag.launch.py`) +
  3 modified files (`CMakeLists.txt`, `package.xml`, `README.md`).
  Covers TAG-01 (detection + 6-DOF publish), TAG-02 (YAML
  configurable offset), TAG-03 (TF to torso_link), TAG-04 (filter
  rejection of low-quality / wrong-id detections).
- Phase 8 (adaptive end orientation) can subscribe to
  `/apriltag/target_pose` and start computing ORI-01 immediately.
- Phase 9 can compose its end-to-end launch by including
  `apriltag.launch.py` plus its own planner/executor includes —
  no Phase-9 changes needed in `apriltag.launch.py`.

## Commits

- `d8d6f13 feat(7-03): apriltag.launch.py — standalone detector test launch`
- `88b61bd docs(7-03): README — Phase 7 AprilTag section`
