---
phase: 09-apriltag-reach
verified: 2026-05-19T16:55:00Z
status: human_needed
score: 21/21 plan-level must-haves verified; 2/4 ROADMAP SCs verified (2 hardware-dependent)
overrides_applied: 0
gaps: []
human_verification:
  - test: "End-to-end launch verification"
    expected: "Run `ros2 launch unitree_g1_dex3_stack apriltag_reach.launch.py` and verify all 7 pipeline components start (robot, RealSense, static TF, detector, bridge, planner, control). All nodes appear in `ros2 node list`."
    why_human: "Requires physical robot hardware (RealSense camera, robot connection), ROS 2 environment, and hardware setup."
  - test: "Bridge G key trigger publishes /goal_pose"
    expected: "With the pipeline running and an AprilTag visible, press G. Bridge logs show 'G pressed' message with target position and Euclidean distance. /goal_pose topic has a published PoseStamped message."
    why_human: "Requires physical robot, AprilTag, and terminal interaction for G key press."
  - test: "Stale/empty cache guard behavior"
    expected: "Cover or remove the AprilTag, wait >1 second, press G. Bridge logs show 'no fresh AprilTag pose' or 'no AprilTag detected yet' warning. Without any tag visible at startup, press G and verify 'no AprilTag detected yet' warning."
    why_human: "Requires physical AprilTag manipulation and visual confirmation of terminal output."
  - test: "End-to-end UAT — TCP error < 3cm for 4/4 targets"
    expected: "Run `ros2 run unitree_g1_dex3_stack apriltag_reach_uat.py` alongside the full pipeline. Move the tag through all 4 tabletop points (center, right-side, low, diag). All 4 points PASS with error <= 3cm. Final output: PASS_COUNT 4/4, exit code 0."
    why_human: "Requires physical robot, RealSense, AprilTag tag, and manual tag placement across 4 positions."
  - test: "Physical robot safe motion demonstration"
    expected: "Run the full apriltag_reach.launch.py pipeline and verify the robot arm executes reach motions toward the detected AprilTag position. Robot motion is smooth, within workspace bounds, and safe."
    why_human: "Physical robot safety-critical operation requiring visual supervision and physical environment."
---

# Phase 9: AprilTag Reach — Verification Report

**Phase Goal:** 将 AprilTag 检测 + TCP 修正 + 自适应位姿整合为完整流水线。
**Verified:** 2026-05-19T16:55:00Z
**Status:** human_needed (all automated checks passed; hardware-dependent items require human verification)
**Re-verification:** No — initial verification

## Goal Achievement

### Observable Truths — PLAN Must-Haves (21/21 VERIFIED)

| # | Truth | Status | Evidence |
|---|-------|--------|----------|
| 1 | Bridge subscribes to /apriltag/target_pose and maintains a sliding-window cache of recent positions for averaging | VERIFIED | `apriltag_goal_bridge.py` L48-49: `collections.deque(maxlen=self.smoothing_window)`, L67-68: subscription to `/apriltag/target_pose`, L109-118: `_target_cb` appends position to cache |
| 2 | Press G triggers single-shot /goal_pose with sliding-averaged position when all guards pass | VERIFIED | `apriltag_goal_bridge.py` L183-195: `_tick` polls keyboard, calls `_on_trigger` on G match; L252-267: publishes averaged PoseStamped to `/goal_pose` |
| 3 | Bridge rejects trigger with distinct WARN on: empty cache (D-10), stale data >1.0s (D-09), in-flight goal (D-03), or distance >= reach_max_distance (D-11/D-14) | VERIFIED | `apriltag_goal_bridge.py` L206: empty cache WARN; L228: stale WARN; L218: in-flight WARN; L248: reachability WARN. Also L212: shoulder origin WARN |
| 4 | Bridge caches the shoulder-to-torso offset at startup via TF lookup and uses it for reachability pre-checks (D-13) | VERIFIED | `apriltag_goal_bridge.py` L149-178: `_retry_shoulder_lookup` does `lookup_transform('torso_link', 'right_shoulder_pitch_link')`, caches result; L242-249: uses for distance check |
| 5 | Bridge clears the in-flight guard after each trajectory completes, accepting the next G press (D-03) | VERIFIED | `apriltag_goal_bridge.py` L123-144: `_traj_cb` sets one-shot timer via `_on_completion` that sets `_waiting_for_completion = False` |
| 6 | Launch file shows all 7 pipeline components | VERIFIED | `apriltag_reach.launch.py`: robot.launch.py (L38-42), rs_launch.py (L45-62), static TF (L65-70), detector (L75-85), bridge (L88-100), planner (L103-111), control (L114-118) |
| 7 | Detector, bridge, planner, and control are wrapped in TimerAction(period=3.0) | VERIFIED | `apriltag_reach.launch.py` L121-126: `TimerAction(period=3.0, actions=[...])` wrapping all 4 delayed components |
| 8 | Launch arg imshow (default 'true') passes through to detector | VERIFIED | `apriltag_reach.launch.py` L17-21: `DeclareLaunchArgument('imshow', default_value='true')`; L83: `{'imshow': LaunchConfiguration('imshow')}` |
| 9 | Launch arg adaptive_orientation_enabled (default 'true') passes through to planner | VERIFIED | `apriltag_reach.launch.py` L23-27: `DeclareLaunchArgument('adaptive_orientation_enabled', default_value='true')`; L109: passed to planner.launch.py |
| 10 | Launch arg planning_timeout (default '1.0') passes through to planner | VERIFIED | `apriltag_reach.launch.py` L29-33: `DeclareLaunchArgument('planning_timeout', default_value='1.0')`; L108: passed to planner.launch.py |
| 11 | UAT harness runs 4 tabletop points (center, right-side, low, diag) with operator-guided cycle | VERIFIED | `apriltag_reach_uat.py` L30-36: `TARGETS` with 4 entries; L158-250: FSM phases init -> waiting_tag -> waiting_traj -> measuring -> next_point -> done |
| 12 | Harness compares expected vs actual TCP position with <= 3cm threshold (D-23) | VERIFIED | `apriltag_reach_uat.py` L83-84: `error_threshold=0.03`; L147-154: stores expected from /apriltag/target_pose; L206-226: computes error vs FK actual |
| 13 | Output per-target: expected=(x,y,z), actual=(x,y,z), error_m=X.XXX, PASS|FAIL | VERIFIED | `apriltag_reach_uat.py` L220-226: logs expected/actual/error/PASS-FAIL per point |
| 14 | Final output: PASS_COUNT N/4, exit 0 if 4/4 (D-24), exit 1 otherwise | VERIFIED | `apriltag_reach_uat.py` L303-304: PASS_COUNT output; L306: `exit_status = 0 if passed == len(TARGETS) else 1`; L324: `sys.exit(exit_status)` |
| 15 | KDL FK computation uses Pinocchio buildReducedModel, chain torso_link -> right_tcp_link, 7 right arm joints from /joint_states | VERIFIED | `apriltag_reach_uat.py` L53-69: `_build_reduced_model` function; L254-276: `_compute_tcp_position` with framesForwardKinematics, torso_id.actInv(tcp_id); L40-48: 7-joint RIGHT_ARM_URDF_JOINTS |
| 16 | CMakeLists.txt install includes bridge and UAT, excludes keyboard_trigger_node | VERIFIED | `CMakeLists.txt` L116-123: install includes `apriltag_goal_bridge.py` and `apriltag_reach_uat.py`; keyboard_trigger_node.py not present |
| 17 | keyboard_trigger_node.py deleted from filesystem | VERIFIED | Glob search: no results for `keyboard_trigger_node.py` |
| 18 | package.xml has all required dependencies declared | VERIFIED | `package.xml` contains: sensor_msgs (L16), geometry_msgs (L17), trajectory_msgs (L19), tf2_ros (L43), tf2_geometry_msgs (L44), rclpy (L47) |
| 19 | README.md documents three launch entries with purpose table | VERIFIED | `README.md` L67-74: three-entry table (apriltag_reach.launch.py / reach.launch.py / apriltag.launch.py) with Chinese descriptions |
| 20 | README.md documents UAT command and G key trigger | VERIFIED | `README.md` L92-94: `ros2 run ... apriltag_reach_uat.py`; L24/L81: G key trigger documentation |
| 21 | README.md reminds user to pip install pupil-apriltags | VERIFIED | `README.md` L48-49 and L106: `pip install pupil-apriltags` |

**Score:** 21/21 plan-level must-haves truths verified

### ROADMAP Success Criteria

| # | Criterion | Status | Evidence |
|---|-----------|--------|----------|
| 1 | `apriltag_reach.launch.py` 单命令启动完整流水线 | VERIFIED | Launch file composes all 7 pipeline components (robot, RealSense, static TF, detector, bridge, planner, control) with TimerAction delay. File exists, compiles, and is structurally complete. |
| 2 | 全流程验证：AprilTag 检测 -> 偏移计算 -> TF 变换 -> 自适应 orientation -> planner -> executor | VERIFIED | Full pipeline traceable: detector (in launch) -> bridge (subscribes /apriltag/target_pose, caches, G-trigger) -> planner (receives /goal_pose with adaptive_orientation_enabled=true passthrough) -> control (via included control.launch.py). All wiring verified in code. |
| 3 | TCP 实际到达位置与目标位置误差在可接受范围内 | NEEDS HUMAN | UAT infrastructure exists (FK computation, 3cm threshold, PASS/FAIL output) but actual execution requires physical robot hardware and AprilTag. Cannot verify programmatically. |
| 4 | 在物理机器人上演示成功 | NEEDS HUMAN | Full pipeline builds and wires all components correctly, but physical robot demonstration with RealSense camera, tag placement, and motion execution requires hardware environment. |

### Required Artifacts

| Artifact | Expected | Status | Details |
| -------- | --------- | ------ | ------- |
| `scripts/apriltag_goal_bridge.py` | Bridge node with 5 guards (D-01..D-14) | VERIFIED | 303 lines, class AprilTagGoalBridge, 8 methods + main. All guards, TF lookup, keyboard reading, trajectory completion timer implemented. |
| `launch/apriltag_reach.launch.py` | End-to-end launch with 7 components | VERIFIED | 137 lines, generate_launch_description. 7 components, TimerAction 3.0s, 3 launch args, emulate_tty=True on interactive nodes. |
| `scripts/apriltag_reach_uat.py` | UAT harness with FK TCP measurement | VERIFIED | 329 lines, class AprilTagReachUAT, 4 TARGETS, Pinocchio FK, PASS/FAIL summary, exit 0 on 4/4. |
| `CMakeLists.txt` | Updated install entries | VERIFIED | install(PROGRAMS) includes bridge + UAT, excludes keyboard_trigger_node. |
| `README.md` | Updated documentation | VERIFIED | Three-entry launch table, G key, UAT command, pupil-apriltags reminder. |

### Key Link Verification

| From | To | Via | Status | Details |
| ---- | --- | --- | ------ | ------- |
| bridge /apriltag/target_pose sub | apriltag_detector_node /apriltag/target_pose pub | ROS 2 topic | WIRED | Topic name matches; bridge callback stores positions in deque |
| bridge /goal_pose pub | planner.launch.py /goal_pose sub | ROS 2 topic | WIRED | Bridge publishes PoseStamped to `/goal_pose`; planner processes it as defined in Phase 6/8 |
| bridge /joint_trajectory_targets sub | planner /joint_trajectory_targets pub | ROS 2 topic | WIRED | Bridge subscribes and sets completion timer based on trajectory duration |
| apriltag_reach.launch.py | robot.launch.py | IncludeLaunchDescription | WIRED | L38-42: IncludeLaunchDescription of robot.launch.py |
| apriltag_reach.launch.py | rs_launch.py | IncludeLaunchDescription | WIRED | L45-62: IncludeLaunchDescription with rs_launch.py arguments |
| apriltag_reach.launch.py | planner.launch.py | IncludeLaunchDescription | WIRED | L103-111: IncludeLaunchDescription with LaunchConfiguration passthrough |
| apriltag_reach.launch.py | control.launch.py | IncludeLaunchDescription | WIRED | L114-118: IncludeLaunchDescription of control.launch.py |
| apriltag_reach.launch.py bridge_node | scripts/apriltag_goal_bridge.py | executable= | WIRED | L88-100: Node definition with `executable='apriltag_goal_bridge.py'` |
| UAT /joint_states sub | robot_state_publisher | ROS 2 topic | WIRED | L116: subscription to `/joint_states` (QoS 10) |
| UAT /joint_trajectory_targets sub | planner | ROS 2 topic | WIRED | L117: subscription to `/joint_trajectory_targets` (QoS 10) |
| UAT KDL FK chain | planner URDF | URDF file path | WIRED | L132-136: same URDF path as planner.launch.py default |
| CMakeLists.txt install | bridge and UAT scripts | install(PROGRAMS) | WIRED | Both scripts in install block |
| README.md launch table | three launch files | documentation | WIRED | All three entries documented with Chinese descriptions |

### Data-Flow Trace (Level 4)

| Artifact | Data Variable | Source | Produces Real Data | Status |
| -------- | ------------- | ------ | ------------------ | ------ |
| `apriltag_goal_bridge.py` | `position_cache` | /apriltag/target_pose topic | FLOWING | Bridge subscribes and caches real-time poses from detector; no static/empty fallback |
| `apriltag_goal_bridge.py` | `_shoulder_origin` | TF2 lookup_transform | FLOWING | Live TF tree from robot_state_publisher; no hardcoded shoulder position |
| `apriltag_reach_uat.py` | `_expected_pos` | /apriltag/target_pose topic | FLOWING | UAT receives real-time poses from detector topic |
| `apriltag_reach_uat.py` | `_joint_state` | /joint_states topic | FLOWING | UAT receives live joint states from robot_state_publisher for FK computation |
| `apriltag_reach.launch.py` | N/A (launch config) | N/A | N/A | Configuration file, not data-rendering |

### Behavioral Spot-Checks

| Behavior | Command | Result | Status |
| -------- | ------- | ------ | ------ |
| Python syntax (bridge) | `python3 -m py_compile scripts/apriltag_goal_bridge.py` | Compile OK (per Plan 01 SUMMARY) | PASS |
| Python syntax (launch) | `python3 -m py_compile launch/apriltag_reach.launch.py` | Compile OK (per Plan 02 SUMMARY) | PASS |
| Python syntax (UAT) | `python3 -m py_compile scripts/apriltag_reach_uat.py` | Compile OK (per Plan 03 SUMMARY) | PASS |
| AST structural (bridge) | python3 AST assertion for class + methods | All required classes/methods found | PASS |
| AST structural (launch) | python3 AST assertion for generate_launch_description | generate_launch_description + all 7 components | PASS |
| AST structural (UAT) | python3 AST assertion for class + methods | All required classes/methods found | PASS |
| Launch ---print check | `ros2 launch unitree_g1_dex3_stack apriltag_reach.launch.py --print` | Requires ROS 2 env (hardware) | SKIP |

### Requirements Coverage

| Requirement | Source Plan | Description | Status | Evidence |
| ----------- | ---------- | ----------- | ------ | -------- |
| INTG-01 | 09-01, 09-02, 09-04 | 新 launch 文件 apriltag_reach.launch.py 替代原 YOLO pipeline | SATISFIED | Launch file exists at `launch/apriltag_reach.launch.py` with all 7 components; bridge node provides detection-to-planning bridge; keyboard_trigger_node.py deleted; CMakeLists updated |
| INTG-02 | 09-03, 09-04 | 端到端验证：AprilTag 检测 -> 偏移计算 -> TF 变换 -> planner -> executor 全流程 | SATISFIED | UAT harness exists at `scripts/apriltag_reach_uat.py` with Pinocchio FK, 4-point test, 3cm threshold, PASS/FAIL reporting (infrastructure verified; execution requires hardware) |

### Anti-Patterns Found

No anti-patterns found across all 3 code files (303 + 137 + 329 = 769 lines reviewed):

| File | Line | Pattern | Severity | Impact |
| ---- | ---- | ------- | -------- | ------ |
| — | — | TBD/FIXME/XXX/HACK markers | — | None (0 markers found) |
| — | — | Placeholder/stub patterns | — | None (0 instances found) |
| — | — | numpy import in bridge | — | None (bridge uses pure Python sum/len per D-07) |
| — | — | pandas/tabulate in UAT | — | None (UAT uses pure Python print per D-25) |
| — | — | CycloneDDS double-set | — | None (no SetEnvironmentVariable in launch file per Pitfall 1) |
| — | — | Orientation averaging | — | None (bridge copies raw last_orientation per D-08) |

### Human Verification Required

#### 1. End-to-end launch verification

**Test:** Run `ros2 launch unitree_g1_dex3_stack apriltag_reach.launch.py` and verify all 7 pipeline components start.
**Expected:** All nodes appear in `ros2 node list`. Robot state publisher, RealSense camera, AprilTag detector, bridge, planner, and control are running.
**Why human:** Requires physical robot hardware (RealSense camera, robot connection), ROS 2 environment, and hardware setup.

#### 2. Bridge G key trigger publishes /goal_pose

**Test:** With the pipeline running and an AprilTag visible, press G and verify the bridge publishes /goal_pose.
**Expected:** Bridge logs show "G pressed" message with target position and Euclidean distance. `/goal_pose` topic has a published PoseStamped message.
**Why human:** Requires physical robot, AprilTag, and terminal interaction for G key press.

#### 3. Stale/empty cache guard behavior

**Test:** Cover or remove the AprilTag, wait >1 second, press G. Then test without any tag visible at startup.
**Expected:** Bridge logs show "no fresh AprilTag pose" or "no AprilTag detected yet" warning accordingly.
**Why human:** Requires physical AprilTag manipulation and visual confirmation of terminal output.

#### 4. End-to-end UAT — TCP error < 3cm for 4/4 targets

**Test:** Run `ros2 run unitree_g1_dex3_stack apriltag_reach_uat.py` alongside the full pipeline. Move the tag through all 4 tabletop points (center, right-side, low, diag).
**Expected:** All 4 points PASS with error <= 3cm. Final output: PASS_COUNT 4/4, exit code 0.
**Why human:** Requires physical robot, RealSense, AprilTag tag, and manual tag placement across 4 positions.

#### 5. Physical robot safe motion demonstration

**Test:** Run the full apriltag_reach.launch.py pipeline and verify the robot arm executes reach motions toward the detected AprilTag position.
**Expected:** Robot arm moves smoothly toward the detected tag position within workspace bounds. Motion is safe and controlled.
**Why human:** Physical robot safety-critical operation requiring visual supervision and physical environment.

## Gaps Summary

No gaps found at the code level. All 21 plan-level must-haves truths are VERIFIED by direct evidence in the codebase. All key links are WIRED. No anti-patterns or debt markers present.

**Hardware-dependent criteria are deferred to human testing:**
- ROADMAP SC3 (TCP error within 3cm) — UAT infrastructure is complete and verified; actual execution requires physical robot
- ROADMAP SC4 (physical robot demonstration) — full pipeline is built and wired; physical demo requires hardware environment

---

_Verified: 2026-05-19T16:55:00Z_
_Verifier: Claude (gsd-verifier)_
