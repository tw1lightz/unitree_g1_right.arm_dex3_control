# Phase 7: AprilTag 检测节点 - Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-05-18
**Phase:** 07-apriltag
**Areas discussed:** A 输出语义, B YAML 配置 + Tag ID, C 检测过滤, D 实现语言/包/launch, E 实时可视化

---

## A · 位姿输出语义

### A.1 偏移参考坐标系

| Option | Description | Selected |
|--------|-------------|----------|
| A1 | tag 局部系（PnP 出 tag 位姿后左乘常量 offset；偏移随 tag 旋转） | ✓ |
| A2 | camera_color_optical_frame（按相机视角；物品位置随相机转动而漂移） | |
| A3 | torso_link（按机器人朝向） | |
| A4 | 可配置 `offset_frame` YAML 字段 | |

**User's choice:** A1 — tag 局部系
**Notes:** 用户接受推荐组合 "A1 + A.2-y + A.3-y"；语义对应"物品在 tag 自身坐标系中的固定位置"。

### A.2 Topic 拆分

| Option | Description | Selected |
|--------|-------------|----------|
| A.2-x | 单 topic：直接发 `/goal_pose`（对接 planner） | |
| A.2-y | 双 topic：`/apriltag/tag_pose`（raw）+ `/apriltag/target_pose`（含 offset），均在 `torso_link`；不发 `/goal_pose`，Phase 9 桥接 | ✓ |

**User's choice:** A.2-y
**Notes:** 解耦干净、独立测试不需要 planner、保留 raw tag pose 给 Phase 8 自适应位姿用。

### A.3 发布时机

| Option | Description | Selected |
|--------|-------------|----------|
| A.3-x | 每帧发布（流式 ~15 Hz） | |
| A.3-y | 仅检测到合格 tag 时发布（事件驱动） | ✓ |
| A.3-z | latest + heartbeat | |

**User's choice:** A.3-y
**Notes:** planner 是事件型 `/goal_pose` 触发，不需要持续刷流量。

---

## B · YAML 配置 + Tag ID 选择

### B.1 Tag 物理边长

| Option | Description | Selected |
|--------|-------------|----------|
| 0.05 m | 5 cm | |
| 0.08 m | 8 cm | ✓ |
| 0.10 m | 10 cm | |
| 其他 | 自定义 | |

**User's choice:** 0.08 m，精度可信
**Notes:** PnP 对此值敏感（影响深度方向米级标尺）。

### B.2 Tag ID 选择策略

| Option | Description | Selected |
|--------|-------------|----------|
| B.2-x | 单一 `target_tag_id`（默认 0） | ✓ |
| B.2-y | 任何 36h11 都接受，取最高置信度 | |
| B.2-z | YAML 多 tag id→offset 表 | |

**User's choice:** B.2-x，target_tag_id = 0
**Notes:** 多 tag 是 Future REQ TAG-05；本阶段保持最简，扩展只动 YAML schema。

### B.3-a 参数加载方式

| Option | Description | Selected |
|--------|-------------|----------|
| Yes | ROS `declare_parameter` + `--params-file` YAML | ✓ |
| No | 节点内自实现 YAML 加载 | |

**User's choice:** Yes
**Notes:** 与 `tcp_torso_pose.py`、`ik_fcl_ompl_planner.cpp` 一致。

### B.3-b 默认 offset_xyz

| Option | Description | Selected |
|--------|-------------|----------|
| 用户给具体值 | — | |
| 占位 `[0.0, 0.0, 0.05]` | 集成时 YAML 改 | ✓ |

**User's choice:** 占位
**Notes:** 真实物品偏移在端到端调试时再改 YAML。

---

## C · 检测过滤策略（TAG-04）

### C.1 decision_margin 阈值

| Option | Description | Selected |
|--------|-------------|----------|
| C.1-x | 硬编码 25.0 | |
| C.1-y | ROS 参数 `decision_margin_min`，默认 25.0 | ✓ |

**User's choice:** C.1-y
**Notes:** 现场必须能 YAML 调；接受推荐组合 "C.1-y + C.2-x + C.3-x"。

### C.2 hamming 距离过滤

| Option | Description | Selected |
|--------|-------------|----------|
| C.2-x | 硬编码 hamming == 0 | ✓ |
| C.2-y | 默认接受 ≤ 1（库默认） | |
| C.2-z | ROS 参数 `hamming_max` | |

**User's choice:** C.2-x
**Notes:** 安全保守值，避免参数过多。

### C.3 跨帧一致性 / 抖动抑制

| Option | Description | Selected |
|--------|-------------|----------|
| C.3-x | 不做跨帧确认 | ✓ |
| C.3-y | 连续 N 帧稳定才发 | |
| C.3-z | N 帧滑动平均 | |

**User's choice:** C.3-x
**Notes:** Phase 9 触发模型是事件型；平滑放下游桥接节点更合适。

---

## D · 实现语言 + 包归属 + Launch

### D.1 实现语言

| Option | Description | Selected |
|--------|-------------|----------|
| D1 | Python (rclpy) + pupil-apriltags | ✓ |
| D2 | C++ + apriltag C 库 | |
| D3 | apriltag_ros2 上游包 | |

**User's choice:** D1
**Notes:** REQ 已点名 pupil-apriltags；用户已 pip install；与现有 Python 节点惯例一致。

### D.2 包归属

| Option | Description | Selected |
|--------|-------------|----------|
| D.2-x | 留在现有 `unitree_g1_dex3_stack` 包 | ✓ |
| D.2-y | 新建独立包 | |

**User's choice:** D.2-x
**Notes:** 与项目惯例一致，CMakeLists/package.xml 增量改动最小。

### D.3 独立测试 launch

| Option | Description | Selected |
|--------|-------------|----------|
| D.3-x | 新建 `launch/apriltag.launch.py` | ✓ |
| D.3-y | 复用并参数化 `visual_detect_click.launch.py` | |

**User's choice:** D.3-x
**Notes:** RealSense 改为 `640×480x15fps`、`align_depth.enable=false`；不内嵌 rviz2/planner/control。

---

## E · 实时可视化

### E.1 可视化方案

| Option | Description | Selected |
|--------|-------------|----------|
| RViz2 + TF 三轴 + Pose Axes display + RViz preset | 完整 ROS 生态可视化 | |
| OpenCV imshow 直接弹窗 | `cv2.namedWindow + cv2.imshow` | ✓ |
| 二者都做 | — | |

**User's choice:** OpenCV imshow（用户原话："不需要用rviz2，运行launch后打开相机窗口在里面画就行了"）
**Notes:** 用户判断检测可信度的诉求由窗口绘制满足；TF 广播 / debug image topic / rviz2 都不做。

### E.2 窗口绘制内容

| 内容 | 说明 | Selected |
|------|------|----------|
| 4 角点多边形 + 颜色（绿=accept / 红=reject） | 直观看到过滤是否生效 | ✓ |
| Tag ID 文字 | — | ✓ |
| PnP 三轴投影到图像（红 x / 绿 y / 蓝 z，~3cm 长） | 用户原话"tag 中心点的坐标系" | ✓ |
| HUD：margin + fps + id | 现场判断 USB 带宽 / 阈值 | ✓ |

**User's choice:** 全部
**Notes:** 按 `q` 关窗（节点继续），Ctrl+C 整体退出，与 `visual_detection_tester` UX 一致。

### E.3 imshow 开关

| Option | Description | Selected |
|--------|-------------|----------|
| Launch arg `imshow`（default true） | 部署/环境维度参数 | ✓ |
| ROS 参数 | 节点行为参数 | |
| 永远开启硬编码 | — | |

**User's choice:** Launch arg
**Notes:** 远程 SSH 时 `imshow:=false` 关掉 GUI，节点纯发 topic。

### E.4 TF 广播？

| Option | Description | Selected |
|--------|-------------|----------|
| 广播 `torso_link → apriltag_<id>` | 供下游 lookup_transform | |
| 不广播（本阶段） | Phase 9 按需再加 | ✓ |

**User's choice:** 不广播
**Notes:** PoseStamped topic 已经满足；不引入未使用的 TF frame。

---

## Agent's Discretion

下列细节用户授权 agent 决策，CONTEXT.md 已记录默认处理：

- TF lookup 失败时丢弃当前帧 + warn 节流
- 节点启动早于 robot_state_publisher 时主回调按需 wait（不在 `__init__` 阻塞）
- OpenCV 绘制细节（颜色饱和度、文字字号、HUD 位置、PnP 三轴长度）选简洁默认
- CMakeLists 中 `install(PROGRAMS …)` 的格式与现有保持一致
- YAML 注释只给关键字段写说明
- `apriltag_detector_node.py` 内部组织：单 Node 子类即可（与 `tcp_torso_pose.py` 风格一致）
- `imshow` 走 launch arg 而非 ROS 参数（YAML = 节点行为参数；launch arg = 部署/环境参数）

## Deferred Ideas

讨论中浮现的、不属于本阶段的想法（已记入 CONTEXT.md `<deferred>`）：

- Multi-tag id→offset 表（Future REQ TAG-05）
- 跨帧平滑（Phase 9 桥接节点）
- TF 广播 `apriltag_<id>` / `apriltag_<id>_target`（Phase 9 按需）
- `/apriltag/debug_image` topic（远程查看场景）
- RViz2 预设（用户拒绝；Phase 9 需要时再做）
- Phase 9 桥接 `/apriltag/target_pose → /goal_pose`（launch remap / 改造 keyboard_trigger / 新桥接节点）
- 节点性能优化（quad_decimate、nthreads）
- 基于 tag 法线推导接近方向（Future REQ ORI-03，Phase 8）
- 多候选姿态 fallback（Future REQ ORI-02，Phase 8）
