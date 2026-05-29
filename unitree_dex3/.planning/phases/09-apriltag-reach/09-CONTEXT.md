# Phase 9: 端到端集成 - Context

**Gathered:** 2026-05-19
**Status:** Ready for planning

<domain>
## Phase Boundary

将 Phase 6 的 TCP offset URDF 链（`right_tcp_link`）+ Phase 7 的 AprilTag 检测节点（`/apriltag/target_pose`）+ Phase 8 的 planner 自适应 orientation 整合为完整端到端流水线。

**交付物：**
1. `launch/apriltag_reach.launch.py` — 单命令启动 robot + RealSense + apriltag_detector + apriltag_goal_bridge + planner + control（INTG-01）
2. `scripts/apriltag_goal_bridge.py` — 新桥接节点：缓存 `/apriltag/target_pose`、按 G 键触发 republish 到 `/goal_pose`、reach-radius 预检
3. `scripts/apriltag_reach_uat.py` — 端到端 UAT harness（FK 误差测量；4 点 tabletop set；4/4 PASS 目标）
4. 删除 `scripts/keyboard_trigger_node.py`（YOLO 时代遗留触发节点；CMakeLists install 与 README 同步清理）
5. `README.md` 更新 — 端到端启动命令、触发键 G、UAT 跑法、与 `reach.launch.py` / `apriltag.launch.py` 三条入口的用途区分
6. 满足 INTG-01、INTG-02 的端到端验证（4/4 通过，TCP 实际位置与 target 误差 ≤ 3 cm）

**不在本阶段：** 改 apriltag_detector 业务逻辑、改 planner 内部算法、新增 ORI-02 多候选 orientation、grasp / DEX3 手部控制、多 tag 支持。

</domain>

<decisions>
## Implementation Decisions

### A — 触发模型 + 桥接节点形态

- **D-01：** 触发模型 = **按键触发**。按下时一次性把桥接节点缓存的最新 target_pose republish 为 `/goal_pose`，单次发布；不持续 publish。沿用 Phase 5 D-05 安全设计（自动触发被反复 deferred）。
- **D-02：** 触发键 = **G**（"go"）。**不沿用 Phase 5 的 K**，避免与 YOLO 时代 keyboard_trigger 的 K 触发语义混淆。
- **D-03：** 重复按键防护 = **拒绝并 WARN**。bridge 维护 `waiting_for_completion` 标志，trigger 时 set true，等当前一次完成后清 false。期间再按 G 输出 "previous goal still in flight, ignoring G"，不发新 `/goal_pose`。
- **D-04：** 桥接以新建 `scripts/apriltag_goal_bridge.py` 实现；同时**删除** `scripts/keyboard_trigger_node.py`（YOLO 残留，AprilTag 时代用不到；CMakeLists `install(PROGRAMS ...)` 同步清理）。
- **D-05：** 桥接节点**仅**在 `apriltag_reach.launch.py` 内启动，不提供独立 `apriltag_goal_bridge.launch.py`（独立无意义，依赖 detector）。

### B — 桥接节点业务逻辑

- **D-06：** 桥接节点订阅 `/apriltag/target_pose`（`geometry_msgs/PoseStamped`，frame_id=`torso_link`，QoS 默认 reliable，匹配 Phase 7 发布端）。
- **D-07：** 缓存策略 = **滑动平均最近 5 帧的 position**（约 0.33 s 窗口 @ 15 Hz）。`collections.deque(maxlen=5)` 推荐。
- **D-08：** 平均范围 = **仅平均 position**；orientation 直接从最近一帧拷贝。理由：Phase 8 D-09/D-10 默认 `adaptive_orientation_enabled=true`，planner 完全覆盖 `/goal_pose.pose.orientation`，bridge 端做 quaternion 平均是浪费。
- **D-09：** Stale 阈值 = **1.0 s**。缓存中最新一帧时间戳超过 1.0 s 时，按 G 拒绝触发并 WARN `"no fresh AprilTag pose (last seen X.X s ago)"`。
- **D-10：** 缓存为空 — 节点启动后从未收到 target_pose，按 G 拒绝触发并 WARN `"no AprilTag detected yet"`。

### C — Reachability 预检

- **D-11：** 预检判据 = **距离单条件**。`|target.position − right_shoulder_pitch_link.origin| ≥ reach_max_distance` 时拒绝触发，不发 `/goal_pose`。
- **D-12：** 阈值 = **ROS 参数 `reach_max_distance`，默认 0.55 m**。基于 Phase 8 UAT center-far 不可达点的经验阈值（`.planning/debug/resolved/08-uat-5of8.md`）。**近端**（`<0.05 m`）由 planner D-08 处理，bridge 不重复。
- **D-13：** Shoulder origin 来源 = **bridge 启动时 TF lookup `torso_link → right_shoulder_pitch_link` 一次，缓存原点**（与 Phase 8 D-05 同一引用点；首次 lookup 失败时按 0.5 s 间隔重试，最多 N 次后 fatal）。Bridge 不复用 planner 的 KDL 链 — 桥接节点是独立 Python 进程。
- **D-14：** 拒绝 UX = **WARN 一行 + 不发 `/goal_pose`**。与 Phase 7 检测过滤静默风格一致；不引入 `/apriltag_bridge/last_reject` 服务/topic（避免 surface 膨胀）。

### D — Launch 组装

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

### E — UAT 验收

- **D-21：** 测试集 = **4 点 tabletop 子集**。从 Phase 8 `scripts/adaptive_orientation_ab.py` 的 8 点 set 中筛选满足：(a) `|target − right_shoulder_pitch_link.origin| ≤ 0.55 m`（reach-radius 内），(b) target 在右侧（不越躯干中线 / `+Y_torso` 半空间内）的 4 点。具体坐标 = the agent's discretion，从已有 8 点中选即可。
- **D-22：** TCP 误差测量方法 = **FK 软件法**。executor 报告轨迹完成后（agent's discretion 选最稳健的完成信号源），从 `/joint_states` 取右臂 7 关节值，KDL FK 出 `right_tcp_link` 在 `torso_link` 的位置，与 `target_pose.position` 比较欧式距离。自动化、易归档。
- **D-23：** 误差阈值 = **3 cm**。匹配 PnP（~5–10 mm at 0.5 m）+ URDF 模型差（~5 mm）+ 摆放/打印误差（~5 mm）的合理量级，含余量。
- **D-24：** Pass 准则 = **4/4 通过**（端到端是 milestone v1.1 收官，不留 partial 容差；3 cm 阈值已含余量，应能达标）。
- **D-25：** UAT harness = `scripts/apriltag_reach_uat.py`，仿 Phase 8 `adaptive_orientation_ab.py` 风格输出 per-target `expected`/`actual`/`error_m`/`PASS|FAIL` 表 + 总 `PASS_COUNT`。harness 的执行模型（自动 vs 操作员逐点手动按 G）= the agent's discretion；推荐 "操作员把 tag 摆到指定相对位置 → harness 监听一次完整 trigger→trajectory→FK 周期 → 记录 → 提示下一点"。

### Folded Todos

无。`gsd-sdk query todo.match-phase 9` 返回 `todo_count=0`。讨论中浮现的 ORI-02 票据组织工作不属于 Phase 9 scope，已就地处理：

- 创建 `.planning/todos/pending/ORI-02-multi-candidate-orientation.md` — 汇集 Phase 8 UAT 三类失败现象 + 触发条件 + scope 边界
- `.planning/REQUIREMENTS.md` ORI-02 单行加 → ticket 引用，避免两处描述漂移

### the Agent's Discretion

- bridge 触发接受 / 拒绝日志的具体格式（保持单行简洁、可 grep 即可）
- bridge 监听 "上一次完成" 的具体信号源 — 候选 (a) 订阅 `/joint_trajectory_targets` 后按轨迹 duration 等待，(b) 订阅 `/joint_states` 检测右臂 joint velocity 整体回零，(c) 订阅 executor 完成 topic（若有）。优先 (a) 因为 trajectory 自带时长信息且最确定；(b) 作为 fallback。
- shoulder origin TF lookup 的重试策略（间隔 + 上限）。建议 0.5 s 间隔、最多 10 次后 fatal。
- 滑动窗口实现选型（`collections.deque(maxlen=5)` vs 数组）— 选 deque。
- 4 点 tabletop 集体的具体坐标（从 Phase 8 8 点中筛选）。
- bridge 节点 Python or C++ — **推荐 Python**（与 keyboard_trigger 历史一致 + Phase 7 apriltag_detector 一致 + 实现简短无性能压力）。
- bridge 节点终端键盘读取实现（沿用 keyboard_trigger 的 `os.read + termios + tty.setcbreak + os.O_NONBLOCK + select.select` 模式即可）。
- UAT harness 是否引入 dependency（如 `pandas`）— 建议**不引入**，纯 Python `print` 表格 + 文件 dump 即可。
- 4 点 set 的执行顺序、点间间隔（演示 + 安全性向）。

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### 项目级 / 需求

- `.planning/PROJECT.md` — Core value、milestone v1.1 目标、现有 v1.0 pipeline 状态。
- `.planning/REQUIREMENTS.md` §"v1.1 Requirements" — `INTG-01` (`apriltag_reach.launch.py` 替代 YOLO pipeline), `INTG-02` (端到端验证全流程)。
- `.planning/ROADMAP.md` Phase 9 — 4 条 success criteria。
- `.planning/STATE.md` — 当前进度（75%，Phase 9 ready to plan）。

### 前序 phase 决策（继承）

- `.planning/phases/06-yolo-tcp-offset/06-CONTEXT.md` D-04..D-09 — TCP offset 通过 URDF `right_tcp_link` 集成；`reach.launch.py` 精简形态保留作为 planner-only 入口（本阶段 D-19 沿用此安排）。
- `.planning/phases/07-apriltag/07-CONTEXT.md` D-02/D-04/D-15 — `/apriltag/target_pose` 双 topic、frame_id=`torso_link`、`640×480×15` RealSense profile + `align_depth.enable=false`、`d435_link → camera_link` static TF。本阶段 D-15 launch 组合直接复用 Phase 7 配置。
- `.planning/phases/08-adaptive-orientation/08-CONTEXT.md` D-05/D-09/D-10 — shoulder origin = `right_shoulder_pitch_link.origin`、planner 默认覆盖 `/goal_pose.orientation`、`adaptive_orientation_enabled=true` 默认。本阶段 D-08（仅平均位置）+ D-13（bridge 用同一 shoulder 引用）+ D-17（透传 default true）依赖此约束。
- `.planning/phases/05-end-to-end-integration/05-CONTEXT.md` D-03/D-05 — `TimerAction(period=3.0)` 启动顺序模式 + 按键触发节点形态参考。本阶段 D-18（启动顺序）+ D-04（bridge 替换老 keyboard_trigger）。
- `.planning/phases/04-right-arm-only-executor/04-CONTEXT.md` D-06 — 28 关节 kp=60 coexistence；本阶段 control.launch.py include 不改。

### Phase 8 UAT 历史 + 后续票据

- `.planning/debug/resolved/08-uat-5of8.md` — Phase 8 5/8 UAT 失败现场分析（center-far / left-of-mid / center-near 三类）。Phase 9 D-21（4 点子集）筛选依据。
- `.planning/todos/pending/ORI-02-multi-candidate-orientation.md` — Future ORI-02 backlog 票据；本阶段 deferred 第一项。

### 节点 / 接口契约

- `src/unitree_g1_dex3_stack-main/scripts/apriltag_detector_node.py` — `/apriltag/target_pose` 发布器；本阶段不改。
- `src/unitree_g1_dex3_stack-main/config/apriltag.yaml` — detector 参数；本阶段不改。
- `src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` — `/goal_pose` 订阅器；`adaptive_orientation_enabled`、`computeAdaptiveOrientation`、shoulder origin 缓存（Phase 8 D-05/D-06）。本阶段不改源码，但 launch 透传参数。
- `src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp` — `/joint_trajectory_targets` 订阅 + 执行；28 joint coexistence。本阶段不改。
- `src/unitree_g1_dex3_stack-main/launch/planner.launch.py` — 已暴露 `adaptive_orientation_enabled`、`planning_timeout`、`tcp_offset_x`、`right_tip` 参数。本阶段透传。
- `src/unitree_g1_dex3_stack-main/launch/control.launch.py` — `joint_trajectory_executor` 启动模板。本阶段 include 不改。
- `src/unitree_g1_dex3_stack-main/launch/robot.launch.py` — robot_state_publisher + CycloneDDS env + `torso_link → d435_link` static TF。本阶段 include 不改。
- `src/unitree_g1_dex3_stack-main/launch/apriltag.launch.py` — Phase 7 独立 launch；本阶段**不**整体 include（避免重复 robot include），但**复用其 RealSense + d435 static TF + apriltag_detector_node 段**到新 `apriltag_reach.launch.py`。
- `src/unitree_g1_dex3_stack-main/launch/reach.launch.py` — Phase 6 精简版；本阶段 D-19 保留。
- `src/unitree_g1_dex3_stack-main/scripts/adaptive_orientation_ab.py` — Phase 8 UAT harness 风格；本阶段 `apriltag_reach_uat.py` 直接参考其 target list、PoseStamped publish、PASS_COUNT 输出。

### 本阶段交付（待创建 / 修改 / 删除）

- **新建：** `src/unitree_g1_dex3_stack-main/scripts/apriltag_goal_bridge.py`
- **新建：** `src/unitree_g1_dex3_stack-main/launch/apriltag_reach.launch.py`
- **新建：** `src/unitree_g1_dex3_stack-main/scripts/apriltag_reach_uat.py`
- **删除：** `src/unitree_g1_dex3_stack-main/scripts/keyboard_trigger_node.py`
- **修改：** `src/unitree_g1_dex3_stack-main/CMakeLists.txt` — `install(PROGRAMS ...)` 列表增 bridge / uat、删 keyboard_trigger
- **修改：** `src/unitree_g1_dex3_stack-main/package.xml` — 视新 Python 依赖追加（rclpy/numpy/tf2_ros 已有；不太可能新增）
- **修改：** `src/unitree_g1_dex3_stack-main/README.md` — 三条入口用法 + 触发键 G + UAT 命令

### 接口契约（不变）

- `/apriltag/target_pose` (`geometry_msgs/PoseStamped`, frame_id=`torso_link`) — bridge 订阅入口
- `/apriltag/tag_pose` — 本阶段不订阅（仅 detector 内部产物 + 调试用）
- `/goal_pose` (`geometry_msgs/PoseStamped`) — bridge 发布出口；planner 订阅入口；orientation 字段会被 planner 覆盖（D-08 + Phase 8 D-09）
- `/joint_states` (`sensor_msgs/JointState`) — UAT FK 输入
- `/joint_trajectory_targets` (`trajectory_msgs/JointTrajectory`) — planner 输出 + executor 输入；bridge 监听用作 "上一次完成" 信号源（agent's discretion）
- TF 链：`camera_color_optical_frame ← camera_link ← d435_link ← torso_link ← right_shoulder_pitch_link`（由 robot.launch.py + RealSense + static publishers 联合提供）

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets

- **`launch/apriltag.launch.py`** — RealSense 段（serial_no、`rgb_camera.color_profile=640x480x15`、`align_depth.enable=false`）+ `d435_link_to_camera_link` static TF + `apriltag_detector_node`（parameters 列表 + `imshow` overlay）。直接拷贝段落到 `apriltag_reach.launch.py`，省去重写参数。
- **`launch/reach.launch.py`** — `IncludeLaunchDescription(robot.launch.py) + d435_tf_node + TimerAction(period=3.0, actions=[planner_launch, control_launch])` 骨架。`apriltag_reach.launch.py` 在此基础上把 detector + bridge 加进 TimerAction 内。
- **`scripts/keyboard_trigger_node.py`** — Python 终端 raw-mode 键盘读取（`os.read + termios + tty.setcbreak + os.O_NONBLOCK + select.select` 模式 + KeyboardInterrupt 收尾）。**实现可直接搬到** `apriltag_goal_bridge.py`，把订阅/业务逻辑替换为 `/apriltag/target_pose` 缓存 + reach 检查。删除原文件不丢实现样板。
- **`scripts/apriltag_detector_node.py`** — Phase 7 Python 节点骨架（`super().__init__("apriltag_detector")`、`declare_parameter`、TF buffer/listener 初始化、QoS sensor_data）。`apriltag_goal_bridge.py` 同模式。
- **`scripts/adaptive_orientation_ab.py`** — Phase 8 UAT harness（target list 数组、`/goal_pose` PoseStamped publish、监听 `/joint_trajectory_targets`、PASS_COUNT 输出 + exit code）。`apriltag_reach_uat.py` 母版。

### Established Patterns

- **ROS 2 Python 节点：** `#!/usr/bin/env python3` + `class Node(rclpy.node.Node)` + `def main(args=None): rclpy.init(...); rclpy.spin(node); finally: node.destroy_node(); rclpy.shutdown()`（Phase 7 D-13 风格）
- **TF lookup：** `Buffer(self.get_clock()) + TransformListener(self.tf_buffer, self)` + `tf_buffer.lookup_transform(target_frame, source_frame, time, timeout=Duration(seconds=0.5))`；`try / catch (TransformException)`（Phase 7 D-04 + 与 `ik_fcl_ompl_planner.cpp` L378 一致）
- **ROS 参数化阈值：** `declare_parameter('reach_max_distance', 0.55) + get_parameter(...).value`（Phase 7 D-08 / Phase 8 D-10 风格）
- **滑动窗口缓存：** `collections.deque(maxlen=N)` + 列表平均（numpy 可选）
- **TimerAction 启动延迟：** `TimerAction(period=3.0, actions=[...])` 包裹后续节点（Phase 5 D-03 / `reach.launch.py`）
- **CycloneDDS env：** 由 `robot.launch.py` 通过 `SetEnvironmentVariable` 设置；apriltag_reach.launch.py include 后自动继承，**不要重复设置**（Phase 7 已踩过）
- **QoS：** `/apriltag/target_pose` 用默认 reliable（与 planner 订阅 `/goal_pose` reliable 匹配）；`/joint_states` 通常 sensor_data；UAT harness 订阅 `/joint_trajectory_targets` 用默认 reliable

### Integration Points

- **bridge 订阅 `/apriltag/target_pose` → 缓存 → 按 G 发 `/goal_pose`：** 不修改 detector 或 planner，纯组合层。
- **shoulder origin 缓存：** TF lookup 一次后留用；与 Phase 8 planner 内部 KDL FK 缓存的同一原点（不同进程独立 lookup，节省进程间通信）。
- **bridge 完成信号源（agent's discretion）：** `/joint_trajectory_targets` 自带 `points[-1].time_from_start`，bridge 订阅后按时长等待最稳健；`/joint_states` velocity-zero 检测作为 fallback。
- **UAT FK 计算：** harness 加载与 planner 同 URDF（`g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf`），KDL 链同 `base_link → right_tcp_link`（与 Phase 6 D-04 + planner.launch.py `right_tip='right_tcp_link'` 默认一致）。
- **`reach.launch.py` 与 `apriltag_reach.launch.py` 不冲突：** 两者都 include `robot.launch.py`，但用户一次只启一个；保留双入口。

### Build/Runtime Concerns

- **pupil-apriltags pip 依赖**（Phase 7 README 已提示）— Phase 9 README 也要列。
- **CycloneDDS env**：robot.launch.py 已设置，apriltag_reach.launch.py 不重复。
- **RealSense 启动延迟 ~5s**（Phase 5 已记录）— `TimerAction(period=3.0)` 缓解 detector / bridge 启动时的 TF lookup 失败概率；首帧 lookup 失败 warn 后丢弃即可（Phase 7 D-04 同思路）。
- **bridge 终端 raw mode**：`emulate_tty=True` + `os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)` 保证 launch 内可读键盘（与 keyboard_trigger 现有模式一致）。SSH 远程 launch 时按 G 仍可工作（终端 raw 模式跨 SSH 兼容）。
- **删除 keyboard_trigger_node.py**：CMakeLists `install(PROGRAMS scripts/keyboard_trigger_node.py ...)` 同步移除；`reach.launch.py` 已不引用（Phase 6 D-08 精简时已移除），无 launch 端清理债。

</code_context>

<specifics>
## Specific Ideas

- **bridge 触发后单次 republish**：模拟 Phase 5 keyboard_trigger 的一次性发 PoseStamped 语义，不持续 publish；planner 单次接收即触发完整规划。
- **`waiting_for_completion` 状态机**：trigger 时 set true，订阅 `/joint_trajectory_targets` 取首个 trajectory 的 `points[-1].time_from_start` + 安全余量（如 +1.0 s）作为 timeout，到期后清 false；如 trajectory 期间收到新 trajectory（planner 重复发），刷新 timeout。
- **README 的三条入口对比表**：
  ```
  | Launch                       | 用途                                  |
  |------------------------------|--------------------------------------|
  | apriltag_reach.launch.py     | 端到端：tag → planner → executor，按 G 触发 |
  | reach.launch.py              | 仅 planner + executor，ros2 topic pub /goal_pose 手动测试 |
  | apriltag.launch.py           | 仅检测：调试 tag 识别 / 摆位 / decision_margin |
  ```
- **UAT harness 跑法（推荐）**：
  1. 操作员把 tag + target 摆到点 1
  2. harness 监听 `/apriltag/target_pose` 第一个 fresh 消息 → 记录 `expected = target_pose.position`
  3. 操作员按 G
  4. harness 监听 `/joint_trajectory_targets` 完成 → FK out `actual = right_tcp_link` 在 `torso_link`
  5. error = ||expected − actual||₂；输出一行 `Point 1: expected=(...), actual=(...), error=X.XX cm, [PASS|FAIL]`
  6. 提示操作员摆下一点
  7. 4 点全跑完输出 `PASS_COUNT: N/4`，`exit 0` if 4/4
- **bridge 参数清单（建议）**：`reach_max_distance` (0.55), `stale_threshold_s` (1.0), `smoothing_window` (5), `trigger_key` ("g"), `goal_pose_topic` ("/goal_pose"), `target_pose_topic` ("/apriltag/target_pose")。
- **bridge 启动 INFO 一行**：`[apriltag_goal_bridge] Ready — press G to trigger (reach_max=0.55m, smoothing=5, stale=1.0s)`，方便操作员确认参数生效。
- **bridge 触发成功 INFO**：`[apriltag_goal_bridge] G pressed — target=(x.xxx, y.xxx, z.xxx) @ torso_link, |target-shoulder|=X.XX m, publishing /goal_pose`。
- **bridge 触发拒绝 WARN**（4 类）：`reach exceeds X.XX m > 0.55m`、`no fresh AprilTag pose (last seen X.X s ago)`、`no AprilTag detected yet`、`previous goal still in flight, ignoring G`。

</specifics>

<deferred>
## Deferred Ideas

- **Future ORI-02（多候选 orientation fallback）** — 已落 backlog 票据 `.planning/todos/pending/ORI-02-multi-candidate-orientation.md`，汇集 Phase 8 UAT 三类失败现象 + 触发条件 + scope 边界。Phase 9 不实现，但 4 点 UAT 子集筛选已规避 Phase 8 collision 类失败点。
- **Future REQ TAG-05（多 tag 支持）** — Phase 7 deferred；Phase 9 单 tag 足够。多 tag 时 bridge 需扩展 cache 字典 + tag 选择逻辑。
- **Future REQ ORI-03（tag 法线推导接近方向）** — Phase 7/8 deferred；本阶段 detector 已发布完整 6-DOF tag pose，未来可订阅利用。
- **自动触发模式**（不按键，detector 稳定后即触发）— Phase 5 + Phase 9 都选手动；演示 + 小批量场景手动更安全（人手在工作区时风险）。
- **Auto-retry on planner failure** — bridge 不重试；planner 失败后由操作员人工复位 / 重新按 G。
- **Bridge 预检从单条件扩展到 + 工作侧 + Z 范围** — 当前 0.55 m 单条件够用；如未来 left-of-mid 等场景频繁出现可扩展。Phase 8 UAT 已显示 left-of-mid 失败，但 Phase 9 D-21 通过子集筛选回避，无即时需求。
- **bridge 反馈 topic / service**（`/apriltag_bridge/last_reject` 等）— 当前 WARN 一行够调试；如未来需要图形化 UI 显示，再加。
- **Emergency-stop / cancel goal topic** — 当前依赖 Phase 4 executor 28 关节 coexistence 安全阈值 + Ctrl+C 退 launch；不引入桥接级 cancel。
- **bridge 性能优化 / C++ 重写** — Python 实现量级 < 50 行核心逻辑，无性能瓶颈；C++ 重写无收益。
- **Reviewed Todos (not folded)** — 无（init 返回 todo_count=0）。

</deferred>

---

*Phase: 09-apriltag-reach*
*Context gathered: 2026-05-19*
