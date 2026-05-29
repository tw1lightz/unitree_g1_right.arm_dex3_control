# Phase 5: End-to-End Integration - Context

**Gathered:** 2026-05-14
**Status:** Ready for planning

<domain>
## Phase Boundary

新建 `reach.launch.py` 将完整流水线（robot + perception + planner + control）整合为单条启动命令；新建 `keyboard_trigger_node.py` 实现按 K 键触发抓取；将 `ultralytics`/`torch` 依赖从 conda 迁移到系统 Python；重写 README 并新增完整架构文档。

交付物：
1. `launch/reach.launch.py` — 一条命令启动全部节点
2. `scripts/keyboard_trigger_node.py` — 键盘触发节点
3. `src/unitree_g1_dex3_stack-main/README.md` — 最小实用版（重写）
4. `src/unitree_g1_dex3_stack-main/docs/ARCHITECTURE.md` — 完整文档（含决策背景）

</domain>

<decisions>
## Implementation Decisions

### A — 集成启动文件结构
- **D-01:** 新建 `launch/reach.launch.py`，通过 `IncludeLaunchDescription` 组合 4 个子 launch：`robot.launch.py`、`perception.launch.py`、`planner.launch.py`、`control.launch.py`。
- **D-02:** 顶层只暴露最小参数集：`model_path`（默认 `/home/unitree/Desktop/unitree_dex3/best.pt`）、`target_class`（默认 `bottle`，传给 `perception.launch.py` 的 `allowed_classes`）、`imshow`（默认 `true`）、`planning_timeout`（默认 `1.0`）。其余参数使用子 launch 默认值。
- **D-03:** 启动顺序：`robot.launch.py` 立即启动；`TimerAction(period=3.0)` 后再启动 `perception.launch.py`、`planner.launch.py`、`control.launch.py`。确保 `robot_state_publisher` 就绪后规划器才启动。
- **D-04:** CycloneDDS 环境变量（`RMW_IMPLEMENTATION`、`CYCLONEDDS_URI`）通过 include `robot.launch.py` 自动继承，`reach.launch.py` 不重复设置。

### B — 目标触发机制
- **D-05:** 新建 `scripts/keyboard_trigger_node.py`（Python ROS 2 节点）。终端监听键盘输入，按 K 时触发一次抓取流程。
- **D-06:** 节点订阅 `/detections_3d`（`vision_msgs/Detection3DArray`），按 K 时从最新检测结果中取**距离最近**的目标（按 3D 位置到原点欧氏距离排序）。不过滤类别（模型只训了一种物品）。
- **D-07:** 目标点计算（坐标系：`camera_color_optical_frame`，z 轴朝前，y 轴朝下）：
  - 取 bbox 底部中点：`y_bottom = center.position.y + size.y / 2`
  - 向上偏移 bbox 高度的 10%：`y_target = y_bottom - size.y * 0.1`
  - x、z 保持 bbox 中心不变
  - 最终发布 `geometry_msgs/PoseStamped`（frame_id 与 detection header 一致）到 `/goal_pose`
- **D-08:** `keyboard_trigger_node.py` 直接发布 `/goal_pose`，绕过 `detection_to_goal_node` 的 `/detection_selection` 中间层。`detection_to_goal_node` 仍保留在 `perception.launch.py` 中（供手动 CLI 触发使用），不删除。
- **D-09:** `keyboard_trigger_node.py` 加入 `reach.launch.py`，随整体流水线一起启动。

### C — 依赖处理
- **D-10:** 将 `ultralytics` 和 `torch` 安装到系统 Python（`pip install ultralytics torch`），不再依赖 `grab` conda 环境。`reach.launch.py` 无需特殊处理，直接启动。
- **D-11:** README 写明前置安装命令：`pip install ultralytics torch`。

### D — 文档
- **D-12:** 原地重写 `src/unitree_g1_dex3_stack-main/README.md` 为**最小实用版**：环境准备（pip install）、一条启动命令、按 K 触发说明、常用参数覆盖示例。
- **D-13:** 新建 `src/unitree_g1_dex3_stack-main/docs/ARCHITECTURE.md` 为**完整文档**：系统架构、话题列表、节点说明、参数表、各阶段关键决策背景（为什么用 OMPL+FCL、为什么 28 关节锁定、为什么 velocity_scale=0.2 等）、故障排查。

### Agent's Discretion
- `keyboard_trigger_node.py` 的键盘读取实现方式（`sys.stdin` raw mode 或 `readchar` 库）— 选最简单、无额外依赖的方案。
- `reach.launch.py` 中 `keyboard_trigger_node` 是否需要 `emulate_tty=True` — 根据键盘读取方式决定。

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### 现有启动文件（直接被 reach.launch.py include）
- `src/unitree_g1_dex3_stack-main/launch/robot.launch.py` — CycloneDDS 设置、robot_state_publisher、joint_state_publisher；含 `urdf_name`/`urdf_path` 参数
- `src/unitree_g1_dex3_stack-main/launch/perception.launch.py` — RealSense + YOLO + project_to_3d + detection_to_goal；含 `model_path`、`allowed_classes`、`imshow`、`detection3d_topic` 参数；RealSense 有内置 5s 延迟
- `src/unitree_g1_dex3_stack-main/launch/planner.launch.py` — ik_fcl_ompl_planner；含 `planning_timeout`、`base_link`、`right_tip` 等参数
- `src/unitree_g1_dex3_stack-main/launch/control.launch.py` — joint_trajectory_executor

### 感知节点（理解话题流）
- `src/unitree_g1_dex3_stack-main/src/detection_to_goal_node.cpp` — 订阅 `/detections_3d` + `/detection_selection`，发布 `/goal_pose`；keyboard_trigger_node 绕过此节点直接发布 `/goal_pose`
- `src/unitree_g1_dex3_stack-main/scripts/ultralytics_detector.py` — 需要 `ultralytics` 包；`#!/usr/bin/env python3` shebang

### 话题接口
- `/detections_3d` → `vision_msgs/Detection3DArray`：`detection.bbox.center`（Pose）+ `detection.bbox.size`（Vector3）；坐标系 `camera_color_optical_frame`
- `/goal_pose` → `geometry_msgs/PoseStamped`：规划器输入；planner 内部处理 TF 变换到 `torso_link`

### 前序阶段约束
- `.planning/phases/04-right-arm-only-executor/04-CONTEXT.md` — D-06 Option A：28 关节锁定 kp=60，不干扰 running mode
- `.planning/phases/01-right-arm-only-planner/01-CONTEXT.md` — 规划器订阅 `/goal_pose`，内部 TF 变换到 `torso_link`（tf_buffer_.transform，timeout 0.5s）

### 参考启动文件（结构参考）
- `src/unitree_g1_dex3_stack-main/launch/visual_detect_yolo.launch.py` — 现有组合 launch 示例，展示如何 include robot + perception + 测试节点

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- `perception.launch.py` 的 `OpaqueFunction` + `LaunchConfiguration.perform()` 模式 — `reach.launch.py` 若需要动态参数处理可复用此模式
- `visual_detect_yolo.launch.py` — 现有组合 launch 结构参考，展示 `IncludeLaunchDescription` + `launch_arguments` 传参方式
- `detection_to_goal_node.cpp` L12-14：`detections_` 缓存最新 Detection3DArray — `keyboard_trigger_node.py` 需实现同样的缓存订阅模式

### Established Patterns
- ROS 2 Python 节点：`#!/usr/bin/env python3` + `rclpy.spin()` — `keyboard_trigger_node.py` 遵循此模式
- `TimerAction(period=5.0)` — `perception.launch.py` 已用此模式延迟 RealSense 启动，`reach.launch.py` 用同样方式延迟 3s
- `RCLCPP_INFO/WARN/ERROR` 日志风格 — 文档中引用节点行为时保持一致

### Integration Points
- `keyboard_trigger_node.py` 订阅 `/detections_3d`，发布 `/goal_pose` — 与规划器的现有订阅直接对接
- `reach.launch.py` 的 `target_class` 参数传给 `perception.launch.py` 的 `allowed_classes`（格式：`"['bottle']"`）
- `keyboard_trigger_node` 需要 `emulate_tty=True`（或等效方式）才能在 launch 内读取键盘输入

</code_context>

<specifics>
## Specific Ideas

- 目标点偏移逻辑（D-07）：取 bbox 底部中点再上移 10% 高度，避免末端执行器插入物体内部。坐标系 `camera_color_optical_frame`（y 轴朝下），"向上"= -y 方向。
- `keyboard_trigger_node.py` 按 K 后应打印确认信息，例如 `[KeyboardTrigger] K pressed — targeting nearest object at (x, y, z), publishing /goal_pose`。
- 完整文档（ARCHITECTURE.md）需包含各阶段决策背景：Phase 1（OMPL+FCL 选型）、Phase 2（路径简化）、Phase 3（velocity_scale=0.2）、Phase 4（28 关节锁定 D-06 Option A）。

</specifics>

<deferred>
## Deferred Ideas

- **类别过滤参数** — 当前模型只训了一种物品，无需过滤。若未来模型支持多类别，可给 `keyboard_trigger_node.py` 加 `target_class` 参数。
- **自动触发模式** — 检测到目标后自动发布 `/goal_pose`，无需按键。当前选择手动触发更安全。
- **DEX3 手部控制** — 抓取动作，v1.0 范围外。

</deferred>

---

*Phase: 05-end-to-end-integration*
*Context gathered: 2026-05-14*
