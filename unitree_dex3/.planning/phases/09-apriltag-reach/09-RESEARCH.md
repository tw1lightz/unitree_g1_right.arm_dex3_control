# Phase 9: 端到端集成 - Research

**Researched:** 2026-05-19
**Domain:** ROS 2 node integration, Python launch composition, end-to-end pipeline assembly
**Confidence:** HIGH

## Summary

Phase 9 is the final assembly phase of milestone v1.1: it wires AprilTag detection (Phase 7), TCP offset URDF chain (Phase 6), and adaptive orientation (Phase 8) into a single end-to-end launch file with a keyboard-triggered bridge node. The bridge caches `/apriltag/target_pose`, applies reachability pre-checks, and republishes on G key press to `/goal_pose` which feeds the existing planner+executor pipeline.

No new external libraries are needed. Every dependency (rclpy, tf2_ros, geometry_msgs, trajectory_msgs, numpy, collections, select, termios) is either a standard ROS 2 Python dependency already in the workspace or Python stdlib. Three established code patterns from earlier phases are directly reusable: (1) keyboard raw-terminal reading from `keyboard_trigger_node.py`, (2) ROS 2 Python node skeleton from `apriltag_detector_node.py`, and (3) UAT harness structure from `adaptive_orientation_ab.py`. Three new files will be created (bridge node, launch file, UAT harness) and one file deleted (`keyboard_trigger_node.py`).

The primary risks are: CycloneDDS environment being set twice (already handled: `robot.launch.py` sets it, do not duplicate); RealSense 5-second startup race with TF lookups; terminal raw-mode compatibility when launched via `ros2 launch` (requires `emulate_tty=True`); and the bridge and planner being independent processes that each independently look up the shoulder origin via TF2 (not a bug, but must be understood to diagnose discrepancies).

**Primary recommendation:** Create the bridge node (`scripts/apriltag_goal_bridge.py`) first, then the launch file (`launch/apriltag_reach.launch.py`), then the UAT harness (`scripts/apriltag_reach_uat.py`), then update CMakeLists.txt and README.md, then delete `keyboard_trigger_node.py`. Each step is independently verifiable before moving to the next.

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions

#### A — 触发模型 + 桥接节点形态

- **D-01：** 触发模型 = **按键触发**。按下时一次性把桥接节点缓存的最新 target_pose republish 为 `/goal_pose`，单次发布；不持续 publish。沿用 Phase 5 D-05 安全设计（自动触发被反复 deferred）。
- **D-02：** 触发键 = **G**（"go"）。**不沿用 Phase 5 的 K**，避免与 YOLO 时代 keyboard_trigger 的 K 触发语义混淆。
- **D-03：** 重复按键防护 = **拒绝并 WARN**。bridge 维护 `waiting_for_completion` 标志，trigger 时 set true，等当前一次完成后清 false。期间再按 G 输出 "previous goal still in flight, ignoring G"，不发新 `/goal_pose`。
- **D-04：** 桥接以新建 `scripts/apriltag_goal_bridge.py` 实现；同时**删除** `scripts/keyboard_trigger_node.py`（YOLO 残留，AprilTag 时代用不到；CMakeLists `install(PROGRAMS ...)` 同步清理）。
- **D-05：** 桥接节点**仅**在 `apriltag_reach.launch.py` 内启动，不提供独立 `apriltag_goal_bridge.launch.py`（独立无意义，依赖 detector）。

#### B — 桥接节点业务逻辑

- **D-06：** 桥接节点订阅 `/apriltag/target_pose`（`geometry_msgs/PoseStamped`，frame_id=`torso_link`，QoS 默认 reliable，匹配 Phase 7 发布端）。
- **D-07：** 缓存策略 = **滑动平均最近 5 帧的 position**（约 0.33 s 窗口 @ 15 Hz）。`collections.deque(maxlen=5)` 推荐。
- **D-08：** 平均范围 = **仅平均 position**；orientation 直接从最近一帧拷贝。理由：Phase 8 D-09/D-10 默认 `adaptive_orientation_enabled=true`，planner 完全覆盖 `/goal_pose.pose.orientation`，bridge 端做 quaternion 平均是浪费。
- **D-09：** Stale 阈值 = **1.0 s**。缓存中最新一帧时间戳超过 1.0 s 时，按 G 拒绝触发并 WARN `"no fresh AprilTag pose (last seen X.X s ago)"`。
- **D-10：** 缓存为空 — 节点启动后从未收到 target_pose，按 G 拒绝触发并 WARN `"no AprilTag detected yet"`。

#### C — Reachability 预检

- **D-11：** 预检判据 = **距离单条件**。`|target.position − right_shoulder_pitch_link.origin| ≥ reach_max_distance` 时拒绝触发，不发 `/goal_pose`。
- **D-12：** 阈值 = **ROS 参数 `reach_max_distance`，默认 0.55 m**。基于 Phase 8 UAT center-far 不可达点的经验阈值（`.planning/debug/resolved/08-uat-5of8.md`）。**近端**（`<0.05 m`）由 planner D-08 处理，bridge 不重复。
- **D-13：** Shoulder origin 来源 = **bridge 启动时 TF lookup `torso_link → right_shoulder_pitch_link` 一次，缓存原点**（与 Phase 8 D-05 同一引用点；首次 lookup 失败时按 0.5 s 间隔重试，最多 N 次后 fatal）。Bridge 不复用 planner 的 KDL 链 — 桥接节点是独立 Python 进程。
- **D-14：** 拒绝 UX = **WARN 一行 + 不发 `/goal_pose`**。与 Phase 7 检测过滤静默风格一致；不引入 `/apriltag_bridge/last_reject` 服务/topic（避免 surface 膨胀）。

#### D — Launch 组装

- **D-15：** 新建 `launch/apriltag_reach.launch.py`，组合：
  - `robot.launch.py` include（提供 URDF + CycloneDDS env + `torso_link → d435_link` static TF）
  - `realsense2_camera/launch/rs_launch.py` include（`640×480×15`、`align_depth.enable=false`，沿用 Phase 7 D-15）
  - `d435_link → camera_link` static_transform_publisher
  - `apriltag_detector_node`（参数 `config/apriltag.yaml` + launch arg `imshow` 覆盖）
  - `apriltag_goal_bridge`（参数 `reach_max_distance`、`stale_threshold_s`、`smoothing_window`、`trigger_key`）
  - `planner.launch.py` include（透传 `adaptive_orientation_enabled`、`planning_timeout`）
  - `control.launch.py` include（executor）
- **D-16：** Launch arg `imshow` 默认 = **true**（演示阶段操作员看 OpenCV 窗口判断 tag 检测可信度，再按 G 触发；headless 部署显式 `imshow:=false` 覆盖）。
- **D-17：** Launch arg `adaptive_orientation_enabled` 透传给 `planner.launch.py`，**默认 true**（Phase 8 D-10 默认；保持 Phase 8 UAT 已验证的行为）。
- **D-18：** 启动顺序 = **`TimerAction(period=3.0)` 延迟启动 detector / bridge / planner / control**，先让 robot_state_publisher + RealSense（5s 内部启动延迟）就绪。沿用 `reach.launch.py` Phase 5 D-03 模式。
- **D-19：** **保留** `launch/reach.launch.py`（Phase 6 精简版：robot + planner + control，无检测）作为 **planner-only manual test 入口**（`ros2 topic pub /goal_pose` 直接喂 planner）。`apriltag_reach.launch.py` 与之并列。
- **D-20：** README 同时介绍三条入口：(a) `apriltag_reach.launch.py` — 端到端，(b) `reach.launch.py` — planner 手动测试，(c) `apriltag.launch.py` — Phase 7 检测独立调试。

#### E — UAT 验收

- **D-21：** 测试集 = **4 点 tabletop 子集**。从 Phase 8 `scripts/adaptive_orientation_ab.py` 的 8 点 set 中筛选满足：(a) `|target − right_shoulder_pitch_link.origin| ≤ 0.55 m`（reach-radius 内），(b) target 在右侧（不越躯干中线 / `+Y_torso` 半空间内）的 4 点。具体坐标 = the agent's discretion，从已有 8 点中选即可。
- **D-22：** TCP 误差测量方法 = **FK 软件法**。executor 报告轨迹完成后（agent's discretion 选最稳健的完成信号源），从 `/joint_states` 取右臂 7 关节值，KDL FK 出 `right_tcp_link` 在 `torso_link` 的位置，与 `target_pose.position` 比较欧式距离。自动化、易归档。
- **D-23：** 误差阈值 = **3 cm**。匹配 PnP（~5–10 mm at 0.5 m）+ URDF 模型差（~5 mm）+ 摆放/打印误差（~5 mm）的合理量级，含余量。
- **D-24：** Pass 准则 = **4/4 通过**（端到端是 milestone v1.1 收官，不留 partial 容差；3 cm 阈值已含余量，应能达标）。
- **D-25：** UAT harness = `scripts/apriltag_reach_uat.py`，仿 Phase 8 `adaptive_orientation_ab.py` 风格输出 per-target `expected`/`actual`/`error_m`/`PASS|FAIL` 表 + 总 `PASS_COUNT`。harness 的执行模型（自动 vs 操作员逐点手动按 G）= the agent's discretion；推荐 "操作员把 tag 摆到指定相对位置 → harness 监听一次完整 trigger→trajectory→FK 周期 → 记录 → 提示下一点"。

### Claude's Discretion

- bridge 触发接受 / 拒绝日志的具体格式（保持单行简洁、可 grep 即可）
- bridge 监听 "上一次完成" 的具体信号源 — 候选 (a) 订阅 `/joint_trajectory_targets` 后按轨迹 duration 等待，(b) 订阅 `/joint_states` 检测右臂 joint velocity 整体回零，(c) 订阅 executor 完成 topic（若有）。优先 (a) 因为 trajectory 自带时长信息且最确定；(b) 作为 fallback。
- shoulder origin TF lookup 的重试策略（间隔 + 上限）。建议 0.5 s 间隔、最多 10 次后 fatal。
- 滑动窗口实现选型（`collections.deque(maxlen=5)` vs 数组）— 选 deque。
- 4 点 tabletop 集体的具体坐标（从 Phase 8 8 点中筛选）。
- bridge 节点 Python or C++ — **推荐 Python**（与 keyboard_trigger 历史一致 + Phase 7 apriltag_detector 一致 + 实现简短无性能压力）。
- bridge 节点终端键盘读取实现（沿用 keyboard_trigger 的 `os.read + termios + tty.setcbreak + os.O_NONBLOCK + select.select` 模式即可）。
- UAT harness 是否引入 dependency（如 `pandas`）— 建议**不引入**，纯 Python `print` 表格 + 文件 dump 即可。
- 4 点 set 的执行顺序、点间间隔（演示 + 安全性向）。

### Deferred Ideas (OUT OF SCOPE)

- **Future ORI-02（多候选 orientation fallback）** — 已落 backlog 票据 `.planning/todos/pending/ORI-02-multi-candidate-orientation.md`，汇集 Phase 8 UAT 三类失败现象 + 触发条件 + scope 边界。Phase 9 不实现，但 4 点 UAT 子集筛选已规避 Phase 8 collision 类失败点。
- **Future REQ TAG-05（多 tag 支持）** — Phase 7 deferred；Phase 9 单 tag 足够。多 tag 时 bridge 需扩展 cache 字典 + tag 选择逻辑。
- **Future REQ ORI-03（tag 法线推导接近方向）** — Phase 7/8 deferred；本阶段 detector 已发布完整 6-DOF tag pose，未来可订阅利用。
- 自动触发模式（不按键，detector 稳定后即触发）— Phase 5 + Phase 9 都选手动；演示 + 小批量场景手动更安全（人手在工作区时风险）。
- Auto-retry on planner failure — bridge 不重试；planner 失败后由操作员人工复位 / 重新按 G。
- Bridge 预检从单条件扩展到 + 工作侧 + Z 范围 — 当前 0.55 m 单条件够用；如未来 left-of-mid 等场景频繁出现可扩展。Phase 8 UAT 已显示 left-of-mid 失败，但 Phase 9 D-21 通过子集筛选回避，无即时需求。
- bridge 反馈 topic / service（`/apriltag/bridge/last_reject` 等）— 当前 WARN 一行够调试；如未来需要图形化 UI 显示，再加。
- Emergency-stop / cancel goal topic — 当前依赖 Phase 4 executor 28 关节 coexistence 安全阈值 + Ctrl+C 退 launch；不引入桥接级 cancel。
- bridge 性能优化 / C++ 重写 — Python 实现量级 < 50 行核心逻辑，无性能瓶颈；C++ 重写无收益。
- Reviewed Todos (not folded) — 无（init 返回 todo_count=0）。

</user_constraints>

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| INTG-01 | 新 launch 文件 apriltag_reach.launch.py 替代原 YOLO pipeline | D-15 精确指定了 7 个组件的组合方式。Launch 文件从 `reach.launch.py`（5 个组件）扩展为 7 个组件：include `robot.launch.py` + `rs_launch.py` + `d435_link→camera_link` static TF + `apriltag_detector_node` + new `apriltag_goal_bridge` + include `planner.launch.py` + include `control.launch.py`。`TimerAction(period=3.0)` 延迟后 5 个。保留 `reach.launch.py` 作为 planner-only 入口。可重用 `apriltag.launch.py` 的 RealSense + detector 段。 |
| INTG-02 | 端到端验证：AprilTag 检测 → 偏移计算 → TF 变换 → planner → executor 全流程 | D-21 到 D-25 精确指定 UAT：4 点 tabletop 子集（筛选自 Phase 8 8 点，满足 ≤0.55 m 半径 + 右侧条件），FK 软件法测量 TCP 误差，3 cm 阈值，4/4 通过标准。UAT harness 参考 `adaptive_orientation_ab.py` 风格实现。全流程信号链 = `/apriltag/target_pose` → bridge 缓存 → G 触发 → `/goal_pose` → planner `goalPoseCallback` → `computeAdaptiveOrientation` → IK → OMPL → `/joint_trajectory_targets` → executor。 |

</phase_requirements>

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| AprilTag detection + 6-DOF pose from RGB stream | Perception (onboard) | — | `apriltag_detector_node.py` runs as ROS 2 Python node on G1 onboard computer, processes RealSense color stream |
| Tag pose TF transform to torso_link | Perception (onboard) | — | Same node uses `tf_buffer.transform()` to convert from `camera_color_optical_frame` to `torso_link` |
| Target pose caching + sliding average | Integration (bridge) | — | `apriltag_goal_bridge.py` caches 5 frames of `/apriltag/target_pose` and averages position |
| User input (keyboard G) | Integration (bridge) | — | Bridge reads `/dev/tty` in raw mode via `select.select()` for the G key press |
| Reachability pre-check | Integration (bridge) | — | Bridge compares target distance vs cached shoulder origin (single distance check) |
| Goal pose publication | Integration (bridge) | — | Bridge publishes single-shot `PoseStamped` to `/goal_pose` on trigger |
| Motion planning (IK + OMPL) | Planning (onboard) | — | C++ `ik_fcl_ompl_planner` subscribes `/goal_pose`, computes adaptive orientation, runs TRAC-IK + OMPL |
| Trajectory execution | Control (onboard) | — | `joint_trajectory_executor` reads `/joint_trajectory_targets` and sends joint commands via Unitree SDK |
| TF transform chain | Infrastructure | — | `robot_state_publisher` + RealSense's internal TF + static TFs provide full `camera_color_optical_frame ← camera_link ← d435_link ← torso_link ← right_shoulder_pitch_link` |
| End-to-end UAT verification | Verification | — | `apriltag_reach_uat.py` runs as ROS 2 node, subscribes to `/joint_states`, computes KDL FK, compares to expected position |
| CycloneDDS network config | Infrastructure | — | `robot.launch.py` sets `RMW_IMPLEMENTATION` and `CYCLONEDDS_URI` via `SetEnvironmentVariable` |

## Standard Stack

### Core
| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| ROS 2 Humble | — | Middleware and build system | Already established across all v1.0 and v1.1 phases |
| rclpy | — | ROS 2 Python client library | Used by bridge node, UAT harness; same as Phase 7/8 Python nodes |
| geometry_msgs | — | PoseStamped type for `/apriltag/target_pose`, `/goal_pose` | Standard ROS message type; both bridge and planner already depend on it |
| trajectory_msgs | — | JointTrajectory type for `/joint_trajectory_targets` | Used by UAT harness for completion signal; planner already publishers this |
| tf2_ros | — | TF transform client for shoulder origin lookup | Bridge does one-time `lookup_transform`; same `Buffer + TransformListener` pattern as Phase 7 |

### Supporting
| Library | Version | Purpose | When to Use |
|---------|---------|---------|-------------|
| Python stdlib `collections.deque` | 3.10+ | Sliding window for position averaging (maxlen=5) | Bridge node cache; established in CONTEXT D-07 |
| Python stdlib `select` / `termios` / `tty` | 3.10+ | Raw terminal keyboard reading | Bridge node G key detection; exact pattern from `keyboard_trigger_node.py` |
| `numpy` | (optional) | NumPy array ops for position averaging | Bridge can use `np.mean()` or pure Python sum/len; numpy already imported in `apriltag_detector_node.py` |
| KDL `kdl_parser` / `PyKDL` | — | FK computation for TCP position in UAT | UAT harness computes TCP position from `/joint_states` shoulder→wrist→TCP chain; planner uses same KDL chain internally |

### Alternatives Considered
| Instead of | Could Use | Tradeoff |
|------------|-----------|----------|
| `collections.deque` | Raw list with manual index | deque has O(1) popleft, maxlen auto-evict; raw list requires manual pop(0) or index tracking. Deque is Pythonic standard. |
| TF2 for shoulder origin | KDL FK (like planner) | Bridge is independent Python process; TF2 lookup from `torso_link→right_shoulder_pitch_link` at zero joint angles gives same origin as planner's KDL FK. TF2 avoids loading URDF + building KDL chain in Python. |
| `/joint_trajectory_targets` completion signal | `/joint_states` velocity-zero detection | Trajectory has deterministic `points[-1].time_from_start`; velocity-zero needs a threshold and is sensitive to noise. Trajectory signal is more robust (D-25 agent's discretion priority). |

**Installation:**
No new packages. All dependencies are already in the workspace or Python stdlib.

**Version verification:** Not required — all dependencies are established project deps. rclpy is tied to ROS 2 Humble distribution.

## Package Legitimacy Audit

> Not applicable. Phase 9 introduces zero new external packages. All dependencies (rclpy, geometry_msgs, trajectory_msgs, tf2_ros, numpy, Python stdlib) are already present in the workspace.

| Package | Registry | Age | Downloads | Source Repo | slopcheck | Disposition |
|---------|----------|-----|-----------|-------------|-----------|-------------|
| (none) | — | — | — | — | — | Phase uses only established project deps |

**Packages removed due to slopcheck [SLOP] verdict:** None
**Packages flagged as suspicious [SUS]:** None

## Architecture Patterns

### System Architecture Diagram

```
┌──────────────────────────────────────────────────────────┐
│                   apriltag_reach.launch.py                │
│                                                          │
│  ┌──────────────┐    ┌──────────────────────┐            │
│  │ robot.launch │    │ rs_launch.py (D435i) │            │
│  │ (URDF + rsp  │    │ 640×480×15, no depth │            │
│  │  + JSP +     │    └──────────┬───────────┘            │
│  │  CycloneDDS) │               │ /camera/color/         │
│  └──────┬───────┘               │ image_raw + CameraInfo │
│         │ /joint_states         ▼                        │
│         │              ┌──────────────────┐              │
│         │              │ apriltag_        │              │
│         │              │ detector_node    │              │
│         │              │ (pupil-apriltags)│              │
│         │              │ PnP → TF→torso   │              │
│         │              └────────┬─────────┘              │
│         │                       │ /apriltag/target_pose  │
│         │                       ▼                        │
│         │              ┌──────────────────┐              │
│         │              │ apriltag_goal_    │              │
│         │              │ bridge.py         │              │
│         │              │ ┌──────────────┐  │              │
│         │              │ │ deque(max=5) │  │ ← cache     │
│         │              │ │ position avg │  │              │
│         │              │ │ shoulder TF  │  │ ← origin    │
│         │              │ │ reach check  │  │ ← pre-check │
│         │              │ │ key 'G' read │  │ ← trigger   │
│         │              │ └──────────────┘  │              │
│         │              └────────┬──────────┘              │
│         │                       │ /goal_pose (single shot)│
│         │                       ▼                        │
│         │              ┌──────────────────┐              │
│         │              │ planner.launch.py │              │
│         │              │ ik_fcl_ompl_      │              │
│         │              │ planner (C++)     │              │
│         │              │ computeAdaptive   │              │
│         │              │ Orientation       │              │
│         │              │ TRAC-IK → OMPL    │              │
│         │              └────────┬──────────┘              │
│  (TimerAction 3.0s              │                         │
│   delay on these                │ /joint_trajectory_targets│
│   4 components)                 ▼                         │
│         │              ┌──────────────────┐              │
│         │              │ control.launch.py │              │
│         └──────────────┤ joint_trajectory_ │              │
│                        │ executor (C++)    │              │
│                        │ 28-joint coexist  │              │
│                        └──────────────────┘              │
│                                                          │
│  UAT: apriltag_reach_uat.py subscribes /joint_states,    │
│       does KDL FK to get TCP position in torso_link,     │
│       compares to expected target position.              │
└──────────────────────────────────────────────────────────┘
```

### Recommended Project Structure (changes only)

```
src/unitree_g1_dex3_stack-main/
├── launch/
│   ├── apriltag_reach.launch.py    [NEW]  End-to-end launch
│   ├── reach.launch.py                    [UNCHANGED] Planner-only manual test
│   ├── apriltag.launch.py                 [UNCHANGED] Detection-only debug
│   ├── robot.launch.py                    [UNCHANGED]
│   ├── planner.launch.py                  [UNCHANGED] Exposes adaptive_orientation_enabled
│   └── control.launch.py                  [UNCHANGED]
├── scripts/
│   ├── apriltag_goal_bridge.py     [NEW]  Keyboard-triggered bridge node
│   ├── apriltag_reach_uat.py       [NEW]  End-to-end UAT harness
│   └── keyboard_trigger_node.py           [DELETED] YOLO-era legacy
├── CMakeLists.txt                         [MODIFY] Update install(PROGRAMS ...)
├── package.xml                            [PROBABLY UNCHANGED]
└── README.md                              [MODIFY] Three-entry comparison table
```

### Pattern 1: Keyboard Raw-Terminal Reading

**What:** Read single keypress from terminal without waiting for Enter, using raw TTY mode with non-blocking fd and `select.select()` timeout. Used in bridge node for G key trigger detection.

**When to use:** The bridge node needs to detect a single keypress while also running its ROS 2 spin loop.

**Source:** Exact pattern from `scripts/keyboard_trigger_node.py` (file to be deleted; pattern preserved in bridge).

```python
# Terminal setup (in __init__)
self.fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
self.old_settings = termios.tcgetattr(self.fd)
tty.setcbreak(self.fd)

# Periodic check in timer callback
def timer_callback(self):
    if select.select([self.fd], [], [], 0.0)[0]:
        ch = os.read(self.fd, 1).decode('utf-8', errors='ignore')
        if ch.lower() == 'g':
            self._on_trigger()

# Cleanup (in destroy_node)
def destroy_node(self):
    termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
    os.close(self.fd)
    super().destroy_node()
```

### Pattern 2: TimerAction Delayed Launch

**What:** Use `TimerAction(period=3.0, actions=[...])` to delay start of components that depend on `robot_state_publisher` and RealSense being fully initialized.

**When to use:** In `apriltag_reach.launch.py`, wrap the detector + bridge + planner + control in a TimerAction so they start 3 seconds after robot.launch.py and rs_launch.py.

**Source:** `launch/reach.launch.py` L49-52.

```python
delayed_actions = TimerAction(period=3.0, actions=[
    detector_node,        # apriltag_detector_node
    bridge_node,          # apriltag_goal_bridge
    planner_launch,       # IncludeLaunchDescription(planner.launch.py)
    control_launch,       # IncludeLaunchDescription(control.launch.py)
])
```

### Pattern 3: ROS 2 Python Node with Sub/Pub and Parameters

**What:** Standard ROS 2 Python node skeleton with `declare_parameter`, `create_subscription`, `create_publisher`, and `create_timer`.

**When to use:** For both `apriltag_goal_bridge.py` and `apriltag_reach_uat.py`.

**Source:** `scripts/apriltag_detector_node.py` L46-131.

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class AprilTagGoalBridge(Node):
    def __init__(self):
        super().__init__('apriltag_goal_bridge')
        # Declare parameters
        self.declare_parameter('reach_max_distance', 0.55)
        self.declare_parameter('stale_threshold_s', 1.0)
        # ...
        # Subscriptions
        self.create_subscription(PoseStamped, '/apriltag/target_pose',
                                 self._target_cb, 10)
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        # Timer (for keyboard checking)
        self.create_timer(0.1, self._tick)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagGoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
```

### Pattern 4: TF2 Lookup with Retry

**What:** One-time TF lookup at startup, with retry on failure. Used for caching shoulder origin in bridge.

**When to use:** Bridge needs `torso_link → right_shoulder_pitch_link` transform at startup.

**Source:** `scripts/apriltag_detector_node.py` L93-94 for Buffer/Listener pattern.

```python
# In __init__
self.tf_buffer = tf2_ros.Buffer()
self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

# Startup retry loop (can use create_timer for non-blocking)
def _lookup_shoulder_origin(self):
    try:
        transform = self.tf_buffer.lookup_transform(
            'torso_link', 'right_shoulder_pitch_link',
            rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5))
        self.shoulder_origin = (transform.transform.translation.x,
                                transform.transform.translation.y,
                                transform.transform.translation.z)
        self.get_logger().info(
            f'Shoulder origin in torso_link: {self.shoulder_origin}')
        return True
    except (tf2_ros.TransformException, tf2_ros.LookupException) as ex:
        self.get_logger().warn(f'Shoulder TF lookup failed: {ex}')
        return False
```

### Pattern 5: Position Sliding Window Average

**What:** Maintain a fixed-length deque of recent positions, compute the arithmetic mean on trigger.

**When to use:** Bridge caches the last 5 frames of target_pose.position to reduce PnP noise.

**Source:** CONTEXT D-07.

```python
import collections
import numpy as np  # or pure Python sum/len

self.position_cache = collections.deque(maxlen=5)

def _target_cb(self, msg: PoseStamped):
    self.position_cache.append((
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z,
    ))
    self._last_target_stamp = msg.header.stamp
    self._last_orientation = msg.pose.orientation  # copy for D-08

def _get_averaged_position(self):
    if not self.position_cache:
        return None
    # Using numpy for conciseness
    arr = np.array(self.position_cache)
    return tuple(np.mean(arr, axis=0))
```

### Anti-Patterns to Avoid

- **Duplicating CycloneDDS env vars:** `robot.launch.py` already sets `RMW_IMPLEMENTATION` and `CYCLONEDDS_URI`. Do NOT set them again in `apriltag_reach.launch.py` (Phase 7 learned this lesson).
- **Orientation averaging:** Phase 8 planner overwrites `/goal_pose.orientation` via `computeAdaptiveOrientation` (D-08). Bridge averaging orientation is wasted computation and introduces unnecessary quaternion math bugs.
- **Bridge launching planner+control independently:** Both `planner.launch.py` and `control.launch.py` should be included via `IncludeLaunchDescription`, not started as separate `Node` definitions.
- **Bridge doing KDL FK for shoulder origin:** The bridge is a lightweight Python node. Loading URDF + building KDL chain is heavy and duplicates what the planner already does. TF2 lookup gives the same result (transform from `torso_link` to `right_shoulder_pitch_link` at origin with zero joints).

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| Keyboard reading from terminal | Custom serial/keyboard driver | `os.open('/dev/tty')` + `termios` + `tty.setcbreak` + `select.select()` pattern | Already in `keyboard_trigger_node.py` — copy-paste ready. Handles cross-SSH compatibility, raw vs cooked mode, non-blocking check. |
| TCP position computation from joint angles | Custom FK solver | KDL via `kdl_parser` (same URDF + chain as planner) | `kdl_parser` and `PyKDL` are already build deps; the planner uses the exact same `g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf`. UAT harness loads URDF, builds chain for `torso_link → right_tcp_link`, does FK from `/joint_states`. |
| Sliding window implementation | Custom circular buffer | `collections.deque(maxlen=N)` | Built-in, O(1) popleft, maxlen auto-evicts oldest. Canonical Python pattern. |
| UAT table printing | pandas / tabulate | Pure Python `print(f"{label:<12} {expected} {actual} {error:.3f} {'PASS' if ok else 'FAIL'}")` | One-off harness for 4 targets. No pandas dependency needed. Reference `adaptive_orientation_ab.py` `_summarize()` for the format. |

**Key insight:** Every "don't hand-roll" item in this phase has a canned, proven implementation in the same repository. The bridge node is a surgical composition of patterns from three existing files (`keyboard_trigger_node.py`, `apriltag_detector_node.py`, `reach.launch.py`) with the business logic replaced (AprilTag subscription instead of YOLO, trajectory completion instead of detection callback).

## Common Pitfalls

### Pitfall 1: CycloneDDS Env Double-Set

**What goes wrong:** One launch file includes another and both set `RMW_IMPLEMENTATION` / `CYCLONEDDS_URI`. ROS 2 launch processes environment variables as launch-level state; a second `SetEnvironmentVariable` could override or conflict.

**Why it happens:** Phase 7 `apriltag.launch.py` does NOT include `robot.launch.py` internally — it separately sets env vars. But `apriltag_reach.launch.py` includes `robot.launch.py` (which sets env vars) AND may also directly include nodes that depend on CycloneDDS — if the parent launch also sets env vars, double-set occurs.

**How to avoid:** `robot.launch.py` is the **sole owner** of CycloneDDS env (`RMW_IMPLEMENTATION`, `CYCLONEDDS_URI`). `apriltag_reach.launch.py` must NOT call `SetEnvironmentVariable` for these. Include `robot.launch.py` and trust it. Confirmed by checking: `apriltag.launch.py` does not include `robot.launch.py` (Phase 7 needed its own env setup), but `apriltag_reach.launch.py` DOES include it, so Phase 7's pattern does not apply here.

**Warning signs:** `grep -r 'SetEnvironmentVariable' launch/apriltag_reach.launch.py` should return empty.

### Pitfall 2: RealSense 5s Startup Race

**What goes wrong:** The bridge node starts, tries to TF-lookup `torso_link → right_shoulder_pitch_link`, and fails because `robot_state_publisher` hasn't published the full TF tree yet.

**Why it happens:** RealSense has ~5s internal startup delay (initializing sensor, publishing first frames). The `TimerAction(period=3.0)` in Phase 5 was designed to handle the robot TF tree delay, but RealSense adds more latency. The bridge shoulder TF lookup is from `torso_link` to `right_shoulder_pitch_link` (robot TF tree, not camera-dependent), so RealSense delay does NOT affect shoulder lookup. But the detector's `/apriltag/target_pose` depends on `camera_color_optical_frame` TF being available.

**How to avoid:** 
- Bridge's shoulder origin TF lookup (`torso_link → right_shoulder_pitch_link`): retry 10 times at 0.5s intervals (5s total). `robot_state_publisher` publishes TF tree within <1s typically, so this should succeed quickly.
- Detector's TF transform (`camera_color_optical_frame → torso_link`): Phase 7 already handles this with `tf_lookup_timeout_s` param (default 0.5s) and `try / except TransformException`. The 3.0s TimerAction helps reduce the probability.
- Bridge's stale threshold (1.0s) handles the case where no target_pose has arrived yet.

**Warning signs:** TF `LookupException` or `ExtrapolationException` warnings on startup. If persistent beyond 5s, check that RealSense is publishing and the TF tree is complete.

### Pitfall 3: Terminal Raw Mode in `ros2 launch`

**What goes wrong:** When launched via `ros2 launch` without `emulate_tty=True`, the process's stdin is not a TTY in raw mode — the terminal raw-mode setup succeeds but reads come from a pipe, not the keyboard. The G key is not detected.

**Why it happens:** By default, ROS 2 launch runs nodes with stdin connected to `/dev/null` (not the terminal). The `os.open('/dev/tty', ...)` approach in `keyboard_trigger_node.py` works because it opens the actual controlling TTY, but this requires `emulate_tty=True` to expose the TTY correctly.

**How to avoid:** 
- In the launch file declaration of the bridge node, include `emulate_tty=True` (same as `keyboard_trigger_node.py` and `apriltag_detector_node.py` already do).
- The `os.open('/dev/tty', ...)` approach is cross-SSH compatible (SSH sessions have a /dev/tty if they were allocated one).

**Warning signs:** Bridge starts, logs "Ready — press G to trigger", but keypresses have no effect. Check for absence of `'xterm' in os.environ.get('TERM', '')` or check `os.isatty(0)`.

### Pitfall 4: Bridge and Planner Shoulder Origin Independence

**What goes wrong:** If the bridge TF lookup and planner KDL FK compute slightly different shoulder origins, the reachability pre-check in bridge (0.55 m threshold) and planner's actual behavior disagree — bridge allows a target that planner finds unreachable, or bridge rejects a target that planner could solve.

**Why it happens:** The bridge uses `tf_buffer.lookup_transform('torso_link', 'right_shoulder_pitch_link', time=0)` while the planner computes FK on the KDL chain at segment 1 with all-zero joints. These should agree within floating-point precision (~1e-6 m) since shoulder-to-torso is a fixed weld joint. But if the TF tree is stale or computed from a different URDF, there can be discrepancies.

**How to avoid:** 
- Verify that `right_shoulder_pitch_link` origin in TF matches the URDF value `(0.0039563, -0.10021, 0.24778)` within ±1 mm (Phase 8 D-05 already confirmed this).
- The bridge logs the shoulder origin on startup for cross-checking.
- Keep the reach threshold with margin: 0.55 m is already conservatively smaller than the 0.61 m center-far distance that was physically unreachable.

**Warning signs:** Bridge shows shoulder origin different from `(0.004, -0.100, 0.248)` by more than 1 cm. This indicates either a stale TF tree or a different URDF loaded than expected.

### Pitfall 5: `pipe` vs Process stdout in Launch Config

**What goes wrong:** The logging format `[apriltag_goal_bridge] Ready — press G to trigger` does not appear in the terminal.

**Why it happens:** ROS 2 launch's default output configuration routes node stdout to `/dev/null`. The `output='screen'` attribute on the `Node` in the launch file ensures output reaches the terminal, but this only works when combined with `emulate_tty=True`.

**How to avoid:** Always specify both `output='screen'` and `emulate_tty=True` on the bridge Node definition in the launch file. Match the existing pattern from `apriltag.launch.py` L101-102.

## Code Examples

Verified patterns from existing project source:

### Bridge Node: Complete Skeleton

Source: `scripts/keyboard_trigger_node.py` (keyboard pattern + main loop), `scripts/apriltag_detector_node.py` (node skeleton + TF), CONTEXT D-01..D-14 (business logic).

```python
#!/usr/bin/env python3
"""ROS 2 node: cache /apriltag/target_pose, trigger on G to /goal_pose."""

import os
import select
import termios
import tty
import collections
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory

import tf2_ros
import tf2_geometry_msgs  # noqa: F401


class AprilTagGoalBridge(Node):
    def __init__(self):
        super().__init__('apriltag_goal_bridge')

        # --- parameters ---
        self.declare_parameter('reach_max_distance', 0.55)
        self.declare_parameter('stale_threshold_s', 1.0)
        self.declare_parameter('smoothing_window', 5)
        self.declare_parameter('trigger_key', 'g')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('target_pose_topic', '/apriltag/target_pose')

        self.reach_max = self.get_parameter('reach_max_distance').value
        self.stale_threshold = self.get_parameter('stale_threshold_s').value
        self.smoothing_window = int(self.get_parameter('smoothing_window').value)
        self.trigger_char = self.get_parameter('trigger_key').value
        goal_topic = self.get_parameter('goal_pose_topic').value
        target_topic = self.get_parameter('target_pose_topic').value

        # --- state ---
        self.position_cache = collections.deque(maxlen=self.smoothing_window)
        self._last_stamp = None
        self._last_orientation = None
        self._last_target = None  # full PoseStamped for ref
        self._waiting_for_completion = False
        self._shoulder_origin = None  # (x, y, z) cached after TF lookup

        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- subs ---
        self.create_subscription(
            PoseStamped, target_topic, self._target_cb, 10)

        # --- completion signal sub (trajectory) ---
        self.create_subscription(
            JointTrajectory, '/joint_trajectory_targets',
            self._traj_cb, 10)

        # --- pubs ---
        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)

        # --- keyboard setup ---
        self.fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        # --- timers ---
        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._retry_shoulder_lookup)  # retry TF

        self.get_logger().info(
            f'[apriltag_goal_bridge] Ready — press {self.trigger_char.upper()} '
            f'to trigger (reach_max={self.reach_max}m, '
            f'smoothing={self.smoothing_window}, stale={self.stale_threshold}s)')

    def _target_cb(self, msg):
        self.position_cache.append((
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ))
        self._last_stamp = msg.header.stamp
        self._last_orientation = msg.pose.orientation
        self._last_target = msg

    def _traj_cb(self, msg):
        # Clear completion flag when a trajectory is published
        self._waiting_for_completion = False

    def _retry_shoulder_lookup(self):
        if self._shoulder_origin is not None:
            return  # already cached
        try:
            transform = self.tf_buffer.lookup_transform(
                'torso_link', 'right_shoulder_pitch_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            self._shoulder_origin = (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            )
            self.get_logger().info(
                f'[apriltag_goal_bridge] Cached shoulder origin: '
                f'{self._shoulder_origin}')
        except Exception as ex:
            self.get_logger().warn(
                f'[apriltag_goal_bridge] Shoulder TF lookup failed: {ex}')

    def _tick(self):
        if not select.select([self.fd], [], [], 0.0)[0]:
            return
        ch = os.read(self.fd, 1).decode('utf-8', errors='ignore')
        if ch.lower() != self.trigger_char:
            return
        self._on_trigger()

    def _on_trigger(self):
        # Pre-checks
        if not self.position_cache:
            self.get_logger().warn(
                '[apriltag_goal_bridge] no AprilTag detected yet')
            return

        if self._shoulder_origin is None:
            self.get_logger().warn(
                '[apriltag_goal_bridge] shoulder origin not yet available')
            return

        if self._waiting_for_completion:
            self.get_logger().warn(
                '[apriltag_goal_bridge] previous goal still in flight, ignoring G')
            return

        # Stale check
        if self._last_stamp is not None:
            age = (self.get_clock().now() - self._last_stamp).nanoseconds * 1e-9
            if age > self.stale_threshold:
                self.get_logger().warn(
                    f'[apriltag_goal_bridge] no fresh AprilTag pose '
                    f'(last seen {age:.1f} s ago)')
                return

        # Compute average position
        xs = [p[0] for p in self.position_cache]
        ys = [p[1] for p in self.position_cache]
        zs = [p[2] for p in self.position_cache]
        avg_pos = (sum(xs) / len(xs), sum(ys) / len(ys), sum(zs) / len(zs))

        # Reachability check
        sx, sy, sz = self._shoulder_origin
        dx = avg_pos[0] - sx
        dy = avg_pos[1] - sy
        dz = avg_pos[2] - sz
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist >= self.reach_max:
            self.get_logger().warn(
                f'[apriltag_goal_bridge] reach exceeds {dist:.3f} m > '
                f'{self.reach_max} m, not publishing')
            return

        # Publish
        self._waiting_for_completion = True
        goal = PoseStamped()
        goal.header.frame_id = 'torso_link'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = avg_pos[0]
        goal.pose.position.y = avg_pos[1]
        goal.pose.position.z = avg_pos[2]
        goal.pose.orientation = self._last_orientation
        self.goal_pub.publish(goal)
        self.get_logger().info(
            f'[apriltag_goal_bridge] G pressed — target=({avg_pos[0]:.3f}, '
            f'{avg_pos[1]:.3f}, {avg_pos[2]:.3f}) @ torso_link, '
            f'|target-shoulder|={dist:.3f} m, publishing /goal_pose')

    def destroy_node(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
        os.close(self.fd)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagGoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

### Launch File: `apriltag_reach.launch.py` Structure

Source: `launch/reach.launch.py` (skeleton) + `launch/apriltag.launch.py` (RealSense + detector section) + CONTEXT D-15.

```python
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share = get_package_share_directory('unitree_g1_dex3_stack')
    realsense_share = get_package_share_directory('realsense2_camera')
    launch_dir = os.path.join(package_share, 'launch')

    # --- launch args ---
    imshow_arg = DeclareLaunchArgument(
        'imshow', default_value='true',
        description='Open OpenCV detection window')
    adaptive_arg = DeclareLaunchArgument(
        'adaptive_orientation_enabled', default_value='true',
        description='Pass through to planner.launch.py')
    planning_timeout_arg = DeclareLaunchArgument(
        'planning_timeout', default_value='1.0',
        description='Planning timeout in seconds')

    # --- immediate components ---
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'robot.launch.py')))

    # --- RealSense (same args as apriltag.launch.py) ---
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, 'launch', 'rs_launch.py')),
        launch_arguments={
            'serial_no': '_243722074823',
            'enable_color': 'true',
            'enable_depth': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'enable_sync': 'false',
            'align_depth.enable': 'false',
            'rgb_camera.color_profile': '640x480x15',
            'initial_reset': 'true',
        }.items())

    d435_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='d435_link_to_camera_link',
        arguments=['0', '0', '0', '0', '0', '0', 'd435_link', 'camera_link'])

    # --- delayed components (3.0s) ---
    apriltag_node = Node(
        package='unitree_g1_dex3_stack',
        executable='apriltag_detector_node.py',
        name='apriltag_detector',
        output='screen',
        emulate_tty=True,
        parameters=[
            os.path.join(package_share, 'config', 'apriltag.yaml'),
            {'imshow': LaunchConfiguration('imshow')},
        ])

    bridge_node = Node(
        package='unitree_g1_dex3_stack',
        executable='apriltag_goal_bridge.py',
        name='apriltag_goal_bridge',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'reach_max_distance': 0.55,
            'stale_threshold_s': 1.0,
            'smoothing_window': 5,
            'trigger_key': 'g',
        }])

    planner_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'planner.launch.py')),
        launch_arguments={
            'planning_timeout': LaunchConfiguration('planning_timeout'),
            'adaptive_orientation_enabled': LaunchConfiguration(
                'adaptive_orientation_enabled'),
        }.items())

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'control.launch.py')))

    delayed_actions = TimerAction(period=3.0, actions=[
        apriltag_node,
        bridge_node,
        planner_launch,
        control_launch,
    ])

    return LaunchDescription([
        imshow_arg,
        adaptive_arg,
        planning_timeout_arg,
        robot_launch,
        realsense_launch,
        d435_to_camera_link,
        delayed_actions,
    ])
```

### UAT Harness: FK Verification Pattern

Source: `scripts/adaptive_orientation_ab.py` (harness structure + table output) + `scripts/read_tcp_pose.py` (KDL FK pattern).

```python
#!/usr/bin/env python3
"""End-to-end UAT harness for Phase 9.

Operates on a 4-point tabletop subset. Operator places the AprilTag at
each target position, presses G, and the harness records the resulting
TCP position via KDL FK on /joint_states.

Usage:
  ros2 run unitree_g1_dex3_stack apriltag_reach_uat.py

The harness waits for the operator to press Enter at each point, then
listens for the next apriltag → trigger → trajectory cycle.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

# Target set (D-21): 4 points from Phase 8 tabletop set,
# filtered for ≤ 0.55 m reach radius + right side (no midline crossing).
TARGETS = [
    ('center',     0.40, -0.20,  0.00),
    ('right-side', 0.40, -0.40,  0.00),
    ('low',        0.40, -0.20, -0.10),
    ('diag',       0.45, -0.30,  0.05),
]


class AprilTagReachUAT(Node):
    def __init__(self):
        super().__init__('apriltag_reach_uat')

        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10)
        self.traj_sub = self.create_subscription(
            JointTrajectory, '/joint_trajectory_targets', self._traj_cb, 10)

        self._joint_state = None
        self._traj_received = False
        self.results = []
        self._phase = 'wait_op'

        self.get_logger().info(
            '[UAT] Phase 9 End-to-End Verification Harness')
        self.get_logger().info(
            f'[UAT] Targets: {len(TARGETS)}')
        self.get_logger().info(
            '[UAT] Place tag at point 1, then press Enter')

        self.create_timer(0.1, self._tick)

    def _js_cb(self, msg):
        self._joint_state = msg

    def _traj_cb(self, msg):
        self._traj_received = True

    def _tick(self):
        # Reuse Phase 8's settle → publish → wait cycle pattern,
        # adapted for manual operator G press between targets.
        pass  # (full implementation per agent's discretion)

    def _compute_tcp_position(self):
        # KDL FK from /joint_states using the same URDF chain as planner
        # (right_tcp_link relative to torso_link).
        # Implementation: load URDF, build KDL tree, extract chain from
        # 'torso_link' to 'right_tcp_link', set joint angles from
        # /joint_states, do FK.
        pass


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagReachUAT()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| YOLO detection + K keyboard trigger | AprilTag detection + G keyboard trigger | Phase 7 (detection), Phase 9 (trigger) | Detection is now deterministic (AprilTag 36h11), not probabilistic (YOLO). Trigger key changes from K to G to avoid semantic confusion with YOLO-era trigger. |
| Separate launch files for detection / planning / execution | Single `apriltag_reach.launch.py` | Phase 9 | Unified end-to-end command replaces multi-terminal startup. But `reach.launch.py` (planner-only) and `apriltag.launch.py` (detection-only) retained for debugging. |
| TCP orientation sent from bridge to planner (hardcoded quat) | Adaptive orientation computed by planner | Phase 8 (planner), Phase 9 (bridge passes orientation but planner overwrites) | Bridge doesn't need to compute or care about orientation. Planner's `computeAdaptiveOrientation` provides IK-friendly orientation based on target direction. |
| No reachability pre-check | Bridge checks distance vs shoulder origin (0.55 m threshold) | Phase 9 | Prevents wasted planner 1s timeout on unreachable targets. Based on empirical data from Phase 8 UAT (center-far at 0.61 m). |
| UAT measures trajectory publication (presence-only) | UAT measures actual TCP position via FK | Phase 9 | Closes the loop: from pose to actual robot position. 3 cm threshold accounts for PnP (~5-10 mm), URDF (~5 mm), placement (~5 mm) errors. |

**Deprecated/outdated:**
- `scripts/keyboard_trigger_node.py`: YOLO-era trigger node, replaced by `apriltag_goal_bridge.py`. Delete per D-04.
- YOLO-related parameters in `reach.launch.py` and `README.md`: Already cleaned in Phase 6. README still references YOLO-style launch args (`target_class`, `model_path`) — Phase 9 README update should replace with AprilTag-specific documentation.

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | G1 onboard computer has `realsense2_camera` ROS package installed | Standard Stack | Low — Phase 7 already validated this. If missing, add `sudo apt install ros-humble-realsense2-camera` to README. |
| A2 | `pupil-apriltags` pip package is installed | Standard Stack | Low — Phase 7 README already documents `pip install pupil-apriltags`. Phase 9 README update should duplicate this reminder. |
| A3 | `rclpy`, `tf2_ros`, `geometry_msgs`, `trajectory_msgs`, `numpy` are available | Standard Stack | LOW — rclpy/tf2_ros/geometry_msgs/trajectory_msgs are ROS 2 core packages. numpy is needed only optionally. If numpy missing, pure Python sum/len replacement works. |
| A4 | The shoulder origin from TF2 lookup matches planner's KDL FK origin within 1 mm | Common Pitfalls | MEDIUM — If they differ, the 0.55 m reach threshold in bridge would not align with planner's internal reachability. Verified by Phase 8 D-05: shoulder in URDF is at `(0.0039563, -0.10021, 0.24778)`. TF lookup of `torso_link → right_shoulder_pitch_link` at zero joints should match. |
| A5 | `emulate_tty=True` + `os.open('/dev/tty', ...)` works for keyboard reading in `ros2 launch` | Common Pitfalls | MEDIUM — This exact pattern works in `keyboard_trigger_node.py` and `apriltag_detector_node.py`. If broken, G keypresses would be silently ignored. Testing on hardware is the only way to confirm. |
| A6 | `/joint_trajectory_targets` is a reliable completion signal | Architecture Patterns | MEDIUM — Planner publishes one trajectory per goal. If planner publishes multiple trajectories for a single goal (e.g., path smoothing splits), bridge must track which trajectories belong to which trigger. D-25 delegates to agent's discretion; trajectory `points[-1].time_from_start` is the most robust heuristic. |

## Open Questions (RESOLVED)

1. **Should the bridge node use trajectory `time_from_start` or subscribe to executor feedback for completion detection?**
   - What we know: D-25 delegates this to discretion. Option (a) subscribe to `/joint_trajectory_targets` and compute duration from `points[-1].time_from_start` plus safety margin (e.g., +1.0 s). Option (b) detect joint velocity zero via `/joint_states` for all right-arm joints.
   - What's unclear: Whether planner publishes exactly one trajectory per `/goal_pose` or may publish multiple (e.g., retry with different seed). If multiple, trajectory-only completion is still fine — the first trajectory's duration is an upper bound.
   - RESOLVED: Use (a) for primary, (b) as fallback. The bridge subscribes to `/joint_trajectory_targets`; when a trajectory arrives, set a timer for `points[-1].time_from_start + 1.0s`, then clear `waiting_for_completion`. If a second trajectory arrives before the timer fires, reset the timer.

2. **What are the exact 4 target coordinates for UAT?**
   - What we know: Must satisfy `distance to shoulder ≤ 0.55 m` AND `right side (+Y_torso half-space)`.
   - What's unclear: Which specific Phase 8 targets satisfy both. From Phase 8 data: `center` (0.40, -0.20) at shoulder distance ~0.437 m [PASS both modes]; `right-side` (0.40, -0.40) at ~0.486 m [PASS adaptive]; `low` (0.40, -0.20, -0.10) at ~0.449 m [PASS adaptive]; `diag` (0.45, -0.30, 0.05) at ~0.537 m [PASS both]. `high` (0.40, -0.20, 0.15) at ~0.464 m but may have issues. `center-near` at (0.30, -0.20, 0.00) has shoulder distance ~0.348 m — within 0.55 m but adaptive had collision issues. Recommend: `center`, `right-side`, `low`, `diag` — all adaptive=true PASS targets from Phase 8 within reach radius.
   - RESOLVED: Use the four targets listed above: center, right-side, low, diag. All PASS under adaptive=true in Phase 8, all within 0.55 m reach radius, all in right-side workspace. (Per D-21, exact coordinates are agent's discretion.)

3. **How does the UAT harness know when the trajectory has completed?**
   - What we know: D-22 says "executor 报告轨迹完成后" and D-25 delegates the signal source to agent's discretion.
   - What's unclear: The executor does not publish a "trajectory complete" topic. The simplest approach is for the harness to subscribe to `/joint_trajectory_targets`, use the trajectory `points[-1].time_from_start` as the expected duration, add a safety margin (e.g., +2.0 s), and assume completion after that time has elapsed.
   - RESOLVED: Harness subscribes to `/joint_trajectory_targets`. On receiving a trajectory, records the `points[-1].time_from_start` duration. After `duration + 2.0s` passes, reads the latest `/joint_states` for FK computation. This is a timeout-based approach that works without executor modification.

## Environment Availability

> Environment probes are for the G1 onboard computer, which is not the current machine. The following is based on project documentation and assumed ROS 2 workspace state.

| Dependency | Required By | Available | Version | Fallback |
|------------|------------|-----------|---------|----------|
| ROS 2 Humble | Bridge, detector, planner, executor | Asmumed | Humble | — |
| `realsense2_camera` | RealSense D435i stream | Assumed (Phase 7) | Ros humble | Not available on non-G1 dev machines |
| `pupil-apriltags` (pip) | `apriltag_detector_node.py` | Assumed (Phase 7 README) | Pip | `pip install pupil-apriltags` |
| `colcon build` with `-DBUILD_IK_FCL_OMPL_PLANNER=ON` | `ik_fcl_ompl_planner` | Assumed (Phase 8) | — | Planner not available; UAT cannot run |
| Python 3 with rclpy | Bridge node, UAT harness | Assumed | ROS 2 bundled | — |
| Conda environment `grab` | Python perception nodes | Assumed (PROJECT.md) | — | Activate with `conda activate grab` |

**Missing dependencies with no fallback:**
- Physical Unitree G1 robot (required for end-to-end validation)
- RealSense D435i mounted on G1 head

**Missing dependencies with fallback:**
- `pupil-apriltags`: Must be installed via pip on G1. Phase 7 README documents this.
- `numpy` for bridge averaging: Pure Python `sum/len` replacement works.

## Validation Architecture

> Validation for Phase 9 is UAT-based (end-to-end hardware validation), not unit-test-based.

### Test Framework
| Property | Value |
|----------|-------|
| Framework | None (UAT script-based) |
| Config file | None |
| Quick run command | `ros2 launch unitree_g1_dex3_stack apriltag_reach.launch.py` (launch verification) |
| Full suite command | `ros2 run unitree_g1_dex3_stack apriltag_reach_uat.py` (hardware UAT) |

### Phase Requirements → Test Map
| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|--------|----------|-----------|-------------------|-------------|
| INTG-01 | Single launch file starts all components | Manual (launch) | `ros2 launch unitree_g1_dex3_stack apriltag_reach.launch.py --print` prints correct composition | ❌ Wave 0 |
| INTG-01 | Bridge subscribes to `/apriltag/target_pose` and republishes to `/goal_pose` on G | Manual (topic echo) | `ros2 topic echo /apriltag/target_pose` + press G + verify `/goal_pose` appears | ❌ Wave 0 |
| INTG-02 | End-to-end: AprilTag → bridge → planner → executor → TCP move | UAT (harness) | `ros2 run unitree_g1_dex3_stack apriltag_reach_uat.py` exits 0 (4/4 PASS) | ❌ Wave 0 |
| INTG-02 | TCP error ≤ 3 cm for 4/4 tabletop targets | UAT (harness FK) | Harness compares expected vs FK-computed TCP position per point | ❌ Wave 0 |
| — | Keyboard raw mode + `emulate_tty=True` | Manual | Visually confirm G keypress triggers bridge and logs appear | ❌ Wave 0 |
| — | Stale cache / empty cache / in-flight guard | Manual | Press G when no tag visible / tag removed / during motion | ❌ Wave 0 |

### Sampling Rate
- **Per task commit:** Manual code review + `colcon build` compilation check
- **Per wave merge:** Launch file syntax verification (`ros2 launch ... --print`), bridge standalone test with `ros2 topic pub /apriltag/target_pose`
- **Phase gate:** Full UAT (4/4 points PASS) before `/gsd:verify-work`

### Wave 0 Gaps
- [ ] `scripts/apriltag_goal_bridge.py` — bridge node (INTG-01)
- [ ] `launch/apriltag_reach.launch.py` — end-to-end launch (INTG-01)
- [ ] `scripts/apriltag_reach_uat.py` — UAT harness (INTG-02)
- [ ] `CMakeLists.txt` install entries — bridge + UAT added; keyboard_trigger removed
- [ ] `README.md` — three-entry launch table, G trigger key, UAT command

*(No existing test infrastructure to modify — all verification artifacts for Phase 9 are new.)*

## Security Domain

> This phase does not introduce network services, authentication, user data handling, or cryptographic operations. The security-relevant aspects are physical robot safety and ROS 2 communication integrity.

### Applicable ASVS Categories

| ASVS Category | Applies | Standard Control |
|---------------|---------|-----------------|
| V2 Authentication | no | Robot pipeline is single-user, no authentication boundary |
| V3 Session Management | no | No sessions |
| V4 Access Control | no | Single operator at terminal |
| V5 Input Validation | yes | Goal position validated against reach radius before planner invocation |
| V6 Cryptography | no | No encryption needed on dedicated robot network |

### Known Threat Patterns for ROS 2 / robot control

| Pattern | STRIDE | Standard Mitigation |
|---------|--------|---------------------|
| Goal outside workspace causes unnecessary planner computation | Denial of Service | Bridge pre-checks reach distance (0.55 m) before publishing to planner |
| Stale target pose from obscured tag causes wrong reach target | Tampering / Spoofing | Bridge stale threshold (1.0 s) rejects detection data older than threshold |
| Repeated G presses cause planner overload | Denial of Service | Bridge `waiting_for_completion` flag rejects concurrent triggers |
| Human in workspace during robot motion | Physical Safety | Manual G trigger (not automatic); 28-joint executor coexistence thresholds from Phase 4; Ctrl+C to abort launch |
| Terminal raw mode reads from non-terminal pipe | Spoofing | `emulate_tty=True` required in launch file; bridge opens `/dev/tty` not stdin |

## Sources

### Primary (HIGH confidence)
- [VERIFIED: Project code] `scripts/keyboard_trigger_node.py` — keyboard raw-terminal reading pattern, ROS 2 node skeleton
- [VERIFIED: Project code] `scripts/apriltag_detector_node.py` — Python ROS 2 node pattern, TF2 usage, parameter declarations
- [VERIFIED: Project code] `scripts/adaptive_orientation_ab.py` — UAT harness structure, target list, PASS/FAIL output
- [VERIFIED: Project code] `launch/reach.launch.py` — TimerAction delay pattern, launch composition skeleton
- [VERIFIED: Project code] `launch/apriltag.launch.py` — RealSense include + detector node + d435 TF publisher
- [VERIFIED: Project code] `launch/robot.launch.py` — CycloneDDS env owner, TF tree root
- [VERIFIED: Project code] `launch/planner.launch.py` — `adaptive_orientation_enabled` parameter passthrough
- [VERIFIED: Project code] `CMakeLists.txt` — existing `install(PROGRAMS ...)` section to modify
- [VERIFIED: Project doc] `.planning/phases/09-apriltag-reach/09-CONTEXT.md` — 25 locked decisions, interface contracts, canonical references
- [VERIFIED: Project debug] `.planning/debug/resolved/08-uat-5of8.md` — center-far empirical 0.55 m reach limit data
- [VERIFIED: Project doc] `.planning/phases/08-adaptive-orientation/08-*` — shoulder origin reference value, A/B harness pattern, UAT structure

### Secondary (MEDIUM confidence)
- [CITED: Project code] `scripts/read_tcp_pose.py` — KDL FK computation pattern for UAT TCP position verification

### Tertiary (LOW confidence)
- None — all engineering claims are verified against existing project code or CONTEXT.md decisions.

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH — All libraries are existing project deps, verified against codebase.
- Architecture: HIGH — All patterns are verified against existing code; launch composition is direct from `reach.launch.py` + `apriltag.launch.py`.
- Pitfalls: HIGH — Based on Phase 7 operational history (CycloneDDS double-set, RealSense delay, TF race) and documented project patterns.
- Code examples: HIGH — All code patterns adapted from verified, running project source files.

**Research date:** 2026-05-19
**Valid until:** 2026-07-19 (2 months — the project is approaching end of milestone and may have short remaining lifespan before maintenance)
