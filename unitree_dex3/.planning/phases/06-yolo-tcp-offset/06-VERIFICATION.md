---
phase: 06-yolo-tcp-offset
type: verification
status: passed
verified_at: "2026-05-18T09:53:00+08:00"
verifier: kiro_default (inline — runtime has no gsd-verifier subagent type)
phase_goal: "移除不可行的 YOLO 代码，将 TCP offset 集成到 planner IK 链末端。"
phase_requirements:
  - CLEAN-01
  - TCP-01
  - TCP-02
plans_verified:
  - 06-01
  - 06-02
  - 06-03
---

# Phase 06 Verification — YOLO 清理 + TCP Offset 集成

## Status

**PASSED.** All `<must_haves>` truths from the three plans were checked against the live workspace and pass. The phase goal is achieved.

This verification was run inline by the orchestrator agent because the Kiro CLI runtime does not register a `gsd-verifier` subagent type; the closest equivalent (`kiro_default`) was driven manually using the verifier's standard checklist (must_haves grep, build smoke test, requirement traceability, regression check vs prior phases).

## Phase goal check

> 移除不可行的 YOLO 代码，将 TCP offset 集成到 planner IK 链末端。

Both halves are achieved:

- **YOLO removed:** 11 / 11 enumerated YOLO files/dirs/binaries are gone from the source tree; CMakeLists.txt and package.xml carry zero YOLO references; reach.launch.py is reduced to robot + d435_tf + planner + control.
- **TCP offset in IK chain:** `right_tcp_link` exists in both URDF files (default and collision-primitives variants) at 0.175 m along the wrist_yaw X axis; the planner declares `right_tip` defaulting to `right_tcp_link` and a `tcp_offset_x` ROS parameter (default 0.175) that overrides the chain's last fixed segment frame at runtime.

## Must-have truths — per plan

### Plan 06-01 (CLEAN-01)

| # | Truth | Result |
|---|-------|--------|
| 1 | `src/bboxes_ex_msgs/` directory does not exist | PASS |
| 2 | `ultralytics_detector.py`, `project_to_3d_node.cpp`, `detection_to_goal_node.cpp`, `visual_detection_yolo_tester.cpp` do not exist | PASS (4 / 4) |
| 3 | `perception.launch.py`, `visual_detect_yolo.launch.py` do not exist | PASS (2 / 2 — `elevator_perception.launch.py` was never present in the working tree at planning time, see Notes) |
| 4 | `best.pt` and `run_perception.sh` do not exist in project root | PASS (2 / 2) |
| 5 | `CMakeLists.txt` does not reference `bboxes_ex_msgs`, `project_to_3d_node`, `detection_to_goal_node`, or `visual_detection_yolo_tester` | PASS (4 / 4 grep counts = 0) |
| 6 | `package.xml` does not contain `bboxes_ex_msgs`, `image_transport`, `pcl_conversions`, or `pcl_msgs` | PASS (4 / 4 grep counts = 0) |
| 7 | `reach.launch.py` does not reference `perception.launch.py`, `model_path`, `target_class`, `imshow`, or `keyboard_trigger_node` | PASS (5 / 5 grep counts = 0) |
| 8 | `colcon build --packages-select unitree_g1_dex3_stack` succeeds | PASS (with `BUILD_IK_FCL_OMPL_PLANNER=ON` — see Deviations) |

### Plan 06-02 (TCP-01 partial)

For each URDF (default + collision_primitives):

| # | Truth | Result |
|---|-------|--------|
| 1 | Has joint `right_tcp_joint` of `type="fixed"` with parent `right_wrist_yaw_link` and child `right_tcp_link` | PASS (both URDFs) |
| 2 | `right_tcp_joint` origin is `xyz="0.175 0 0" rpy="0 0 0"` | PASS (both URDFs) |
| 3 | `right_tcp_link` is defined as empty `<link name="right_tcp_link" />` (no visual / collision / inertial) | PASS (both URDFs) |
| 4 | `right_tcp_joint` appears before `right_hand_palm_joint` in file order | PASS (default URDF: 1259 < 1265; collision URDF: 1063 < 1069) |
| 5 | XML parses without errors | PASS (both URDFs via `python3 xml.etree.ElementTree`) |

### Plan 06-03 (TCP-01 + TCP-02)

| # | Truth | Result |
|---|-------|--------|
| 1 | `ik_fcl_ompl_planner.cpp` declares parameter `right_tip` with default `right_tcp_link` | PASS |
| 2 | `ik_fcl_ompl_planner.cpp` declares parameter `tcp_offset_x` with default `0.175` | PASS |
| 3 | After `getChain()` succeeds, the code rebuilds `kdl_chain_right` with the `tcp_offset_x` value if the last segment is a fixed joint | PASS (4 references to `KDL::Joint::None`, 1 `KDL::Vector(tcp_offset_x, ...)`, 1 `"TCP offset overridden"` log line) |
| 4 | `planner.launch.py` `DeclareLaunchArgument` for `right_tip` has default `right_tcp_link` | PASS |
| 5 | `planner.launch.py` has `DeclareLaunchArgument` for `tcp_offset_x` with default `0.175` | PASS (3 `tcp_offset_x` mentions: arg + perform + parameters dict) |
| 6 | `planner.launch.py` does not contain `detection_topic` or `selected_class_topic` | PASS (both grep counts = 0) |
| 7 | `colcon build --packages-select unitree_g1_dex3_stack` succeeds | PASS (with `BUILD_IK_FCL_OMPL_PLANNER=ON`) |
| 8 | No hardcoded 0.175 in C++ source (only as parameter default) | PASS — only one occurrence of `0.175` in `ik_fcl_ompl_planner.cpp`, on the `declare_parameter("tcp_offset_x", 0.175)` line |

## Build smoke test

```
$ colcon build --packages-select unitree_g1_dex3_stack \
    --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON
Finished <<< unitree_g1_dex3_stack [0.42s]
Summary: 1 package finished [1.08s]
```

Exit code 0. Binaries produced:

- `build/unitree_g1_dex3_stack/ik_fcl_ompl_planner`
- `build/unitree_g1_dex3_stack/visual_detection_tester`
- (plus the previously existing `dex3_controller`, `joint_state_publisher`, `joint_trajectory_executor`, `right_hand_pressure_monitor` — all still link cleanly after the dependency cleanup)

## Requirement traceability

| Requirement | Plan(s) | Status |
|-------------|---------|--------|
| CLEAN-01 — Remove YOLO code | 06-01 | ✓ Complete |
| TCP-01 — TCP offset in IK chain | 06-02 + 06-03 | ✓ Complete |
| TCP-02 — TCP offset configurable via ROS parameter | 06-03 | ✓ Complete |

All three Phase 6 requirement IDs from `REQUIREMENTS.md` are accounted for. No leftover plan references unmapped IDs.

## Cross-phase regression check

Prior phases (01–05) deal with planner core, executor, smoothing, and integration. Phase 6 deletions touched only YOLO-era nodes (`project_to_3d_node`, `detection_to_goal_node`, `visual_detection_yolo_tester`) and the `bboxes_ex_msgs` package, none of which were exercised by phases 01–05's verification artifacts. The retained `visual_detection_tester` (Phase 4-era click-based debugging tool) still compiles and links — confirmed by the post-cleanup `colcon build` smoke test.

The planner core change (`right_tip` default, `tcp_offset_x` parameter, KDL chain rebuild) is additive: existing behavior is preserved when `right_tip:=right_wrist_yaw_link` is supplied via launch override (the chain rebuild block guards on `last_seg.getJoint().getType() == KDL::Joint::None` and skips if the user reverted to a chain that ends on a movable joint). Phase 4's executor and Phase 3's smoother subscribe to topics that the planner publishes (joint trajectories) — those interfaces are untouched.

No regressions detected.

## CONTEXT.md decisions honored

| Decision | Plan | Status |
|----------|------|--------|
| D-01 — delete enumerated YOLO files | 06-01 | ✓ |
| D-02 — keep `visual_detection_tester.cpp` and `visual_detect_click.launch.py` | 06-01 | ✓ (target still in CMakeLists.txt + launch file present) |
| D-03 — clean CMakeLists.txt + package.xml of YOLO refs | 06-01 | ✓ |
| D-04 — URDF virtual fixed link `right_tcp_link` at x=0.175 | 06-02 | ✓ |
| D-05 — apply to both default + collision_primitives URDFs | 06-02 | ✓ |
| D-06 — planner `right_tip` default → `right_tcp_link` (code + launch) | 06-03 | ✓ |
| D-07 — `tcp_offset_x` ROS parameter (default 0.175) overrides chain end at runtime | 06-03 | ✓ |
| D-08 — simplify `reach.launch.py` to robot + planner + control + d435_tf | 06-01 | ✓ |
| D-09 — Phase 9 will add `apriltag_reach.launch.py` | n/a (deferred) | acknowledged |
| D-10 — `/goal_pose` interface unchanged | 06-03 | ✓ |

## Deviations summary

One deviation, recorded in `06-01-SUMMARY.md` and re-referenced in `06-03-SUMMARY.md`:

- `option(BUILD_IK_FCL_OMPL_PLANNER ... OFF)` was added to `CMakeLists.txt` to avoid hard-failing `colcon build` on machines without `trac_ik_lib`/`ompl`/`fcl`/`geometric_shapes`/`resource_retriever`. The planner target still compiles cleanly when the option is enabled (verified above). The intent of both 06-01 Task 5 and 06-03 Task 3 acceptance criteria — "the package builds, the planner compiles" — is preserved.

## Notes

- This phase was originally executed and committed in a single squash commit `d67ee62 feat(phase-6): YOLO cleanup + TCP offset integration` on 2026-05-15. The per-plan SUMMARY.md files, STATE.md/ROADMAP.md updates, and this VERIFICATION.md were produced retrospectively at phase close-out time on 2026-05-18 after the `safe_resume_gate` detected a partial close-out state.
- `elevator_perception.launch.py` and `elevator_ocr_node.py` were referenced in the original plan as "potentially needs deletion" but were not present in the working tree at planning time and never made it into `git ls-files` — the deletion is a no-op for them.
- `test_button_ocr.py` and `yolo_last_detection.jpg` at the project root are dangling YOLO-era debris not enumerated in any plan. They are harmless (not built, not installed, not imported anywhere) and are deferred to a future cleanup if the user wants the project root pristine.

## Human verification deferred to Phase 9

The end-to-end on-robot validation steps from the plans (FK comparison vs. `tcp_torso_pose.py`, `tcp_offset_x:=0.20` runtime override smoke test, `ros2 topic pub /goal_pose` to a known reachable pose) require a running robot and are explicitly deferred to Phase 9 (端到端集成) per CONTEXT.md decision D-10. Phase 6 verification stops at static + build correctness, which is what the plans specify.
