# Phase 7: AprilTag 检测节点 - Context

**Gathered:** 2026-05-18
**Status:** Ready for planning

<domain>
## Phase Boundary

实现一个**独立的 ROS 2 检测节点**，从 D435i 头部 RealSense 的 RGB 流（`/camera/color/image_raw`，640×480 @ 15fps）使用 `pupil-apriltags` 检测 AprilTag 36h11，根据 YAML 配置应用一个 tag 局部坐标系下的 XYZ 偏移得到"目标物品位姿"，再通过 TF 把两个位姿都变换到 `torso_link` 帧后发布。同时启动一个 OpenCV imshow 窗口实时绘制 tag 角点 + ID + 三轴 + 过滤指标，便于现场判断检测可信度。节点能用单条 `ros2 launch` 命令独立启动，不依赖 planner。

**交付物：**
1. `scripts/apriltag_detector_node.py` — 主节点（rclpy + pupil-apriltags + cv_bridge + tf2_ros + OpenCV 绘制）
2. `config/apriltag.yaml` — ROS 参数文件（tag_size、target_tag_id、offset_xyz、过滤阈值、topic 名等）
3. `launch/apriltag.launch.py` — 独立测试 launch（robot + realsense 640×480x15 + d435 static TF + 节点）
4. `CMakeLists.txt` 与 `package.xml` 增量更新（install + exec_depend）

**Phase 9 才整合到完整 `apriltag_reach.launch.py`** — 本阶段不接 planner、不发 `/goal_pose`、不做端到端测试。

</domain>

<decisions>
## Implementation Decisions

### A — 输出语义（位姿 + 偏移 + topic + 时机）

- **D-01：偏移坐标系 = tag 局部系。** PnP 求出 tag 6-DOF 位姿后，左乘一个常量平移矩阵 `T_tag_to_target = Translate(offset_xyz)` 得到目标位姿（语义：tag 局部系下的 XYZ 偏移，tag 旋转时偏移随之旋转）。这样 YAML 中的 `offset_xyz` 直观对应"物品在 tag 自身坐标系中的相对位置"。
- **D-02：双 topic 输出，全部 frame_id = `torso_link`。**
  - `/apriltag/tag_pose`（`geometry_msgs/PoseStamped`）— 偏移**前**的 raw tag 中心位姿
  - `/apriltag/target_pose`（`geometry_msgs/PoseStamped`）— 偏移**后**的物品目标位姿
  - **不直接发 `/goal_pose`** — Phase 9 桥接时再决定（remap、改造 `keyboard_trigger_node`、或新桥接节点）。
- **D-03：发布时机 = 事件驱动。** 每收到一帧 RGB 图，跑一次检测；只有当至少一个检测通过 `target_tag_id == 0` + `hamming == 0` + `decision_margin >= decision_margin_min` 三重过滤时，才发布两个 PoseStamped。无合格检测时静默（不发空消息）。
- **D-04：TF 变换链。** PnP 出来的 tag 位姿在 `camera_color_optical_frame` 下；用 `tf_buffer.transform(pose_stamped, "torso_link", timeout=Duration(seconds=0.5))` 变换到 `torso_link`，与 planner 中的 TF 模式（`ik_fcl_ompl_planner.cpp` L378）保持一致。TF 链 `camera_color_optical_frame ← camera_link ← d435_link ← torso_link` 由 launch 启动的 `realsense2_camera` + `d435_link_to_camera_link` static publisher + URDF (`robot.launch.py`) 三段共同提供。

### B — YAML 配置 + Tag ID 选择

- **D-05：使用 ROS 标准参数模式 + `--params-file`。** 节点通过 `declare_parameter()` + `get_parameter()` 读取，与 `tcp_torso_pose.py`、`ik_fcl_ompl_planner.cpp` 一致；launch 用 `parameters=[<config_path>]` 加载。
- **D-06：YAML 路径 = `src/unitree_g1_dex3_stack-main/config/apriltag.yaml`。** 通过 `install(DIRECTORY config DESTINATION share/${PROJECT_NAME})` 安装到 `share/`。
- **D-07：单一 `target_tag_id` 字段（默认 0）。** 检测到的所有 tag36h11 中只关心匹配该 ID 的，其余忽略；多 tag 支持是 Future REQ TAG-05，仅 YAML schema 升级即可。
- **D-08：参数 + 默认值清单（YAML 字段）：**

  ```yaml
  apriltag_detector:
    ros__parameters:
      tag_family: "tag36h11"
      tag_size: 0.08              # 米；用户精确值
      target_tag_id: 0
      offset_xyz: [0.0, 0.0, 0.05] # tag 局部系；占位，集成时按物品实际位置改
      decision_margin_min: 25.0
      output_frame: "torso_link"
      rgb_topic: "/camera/color/image_raw"
      camera_info_topic: "/camera/color/camera_info"
      tag_pose_topic: "/apriltag/tag_pose"
      target_pose_topic: "/apriltag/target_pose"
      tf_lookup_timeout_s: 0.5
  ```

  `imshow` 不放 YAML，是 launch arg（D-15），保持"YAML = 节点行为参数；launch arg = 部署/环境参数"的清晰边界。
- **D-09：tag 物理边长 = 0.08 m（精度可信）。** 用户已确认打印的 36h11 边长精确 8 cm。该值进入 YAML，**不要硬编码到代码中**。

### C — 检测过滤（TAG-04）

- **D-10：`decision_margin_min` 做 ROS 参数，默认 25.0（在 YAML 中可现场调）。** 现场光线/打印质量变化大时直接 YAML 改值，无需重编译。
- **D-11：`hamming == 0` 硬编码。** 拒绝任何被纠错过的检测。tag36h11 编码已足够鲁棒；本阶段单 tag 场景下，hamming > 0 应该被怀疑为误检。这个保守安全值不做参数化，避免 YAML 旋钮过多。
- **D-12：不做跨帧平滑/N 帧确认。** 每帧独立发布。理由：(1) Phase 9 触发模型是事件型（按 K → 取一次 target_pose → planner），节点端时间平滑没用；(2) 真要平滑应放在 Phase 9 桥接节点（取最近 N 帧的 `/apriltag/target_pose` 平均）；(3) 当前节点保持最简，便于独立测试。

### D — 实现语言 + 包归属 + 独立 launch

- **D-13：Python 节点（rclpy）。** 与 `scripts/tcp_torso_pose.py`、`scripts/keyboard_trigger_node.py` 同模式。`pupil-apriltags` 库内核是 C 实现，640×480@15fps 完全无性能压力。**用户已通过 `pip install pupil-apriltags` 手动安装到系统 Python**，因此**不**经 conda `grab` 环境（与 Phase 5 把 `ultralytics`/`torch` 装到系统 Python 的策略一致）。
- **D-14：留在现有 `unitree_g1_dex3_stack` 包。** 不新建独立包。CMakeLists.txt 增量改：
  - `install(PROGRAMS scripts/apriltag_detector_node.py …)` 加该文件
  - 新增 `install(DIRECTORY config DESTINATION share/${PROJECT_NAME})`
  - `package.xml` 加 `<exec_depend>tf2_ros</exec_depend>`、`<exec_depend>tf2_geometry_msgs</exec_depend>`、`<exec_depend>cv_bridge</exec_depend>`、`<exec_depend>realsense2_camera</exec_depend>`、`<exec_depend>python3-opencv</exec_depend>`；`pupil-apriltags` 通过 README 提示用户 `pip install`，不写进 package.xml（pip 包不在 rosdep 数据库）。
- **D-15：独立 launch 文件 `launch/apriltag.launch.py`。** 结构对齐 `visual_detect_click.launch.py`：

  ```
  robot.launch.py            # 提供 URDF TF（含 torso_link → d435_link）
    + realsense2_camera/launch/rs_launch.py
        rgb_camera.profile=640x480x15
        align_depth.enable=false
        enable_sync=true
    + d435_link_to_camera_link static_transform_publisher（args ['0','0','0','0','0','0','d435_link','camera_link']）
    + apriltag_detector_node（parameters=[apriltag.yaml]，emulate_tty=True）
  ```

  Launch arguments：
  - `imshow` (default `true`) — 控制节点是否开 OpenCV 窗口（D-17）
  - `config_file` (default `<pkg>/config/apriltag.yaml`) — 允许覆盖 YAML
  - `urdf_name`、`urdf_path` — 透传给 `robot.launch.py`

  **不内嵌 rviz2、不内嵌 planner、不内嵌 control launch。** Phase 9 端到端 launch 不复用本 launch，但可参考其 realsense + static TF 段。

### E — 实时可视化（OpenCV imshow 窗口）

- **D-16：使用 OpenCV 窗口而非 RViz2。** 用户明确不要 RViz2 / TF 三轴 / debug image topic；要"启动 launch 后弹出相机窗口直接画"。
- **D-17：节点内开窗口 `cv2.namedWindow` + `cv2.imshow` + `cv2.waitKey(1)`，按 `q` 关窗（节点继续跑），Ctrl+C 整体退出。** 与 `src/visual_detection_tester.cpp` (L52-53) 的 UX 一致。窗口标题：`AprilTag detector (id=<target_tag_id>)`。窗口在 `imshow=true` 时启用、`imshow=false` 时跳过（无 GUI / SSH 场景）。
- **D-18：每帧叠加绘制内容：**
  1. tag 4 个角点连成多边形（**绿色 = 通过 hamming + margin 过滤；红色 = 被过滤拒绝**）— 能直接看到过滤是否生效
  2. tag id 文字（左上角 corner 附近）
  3. PnP 求出的三轴投影回图像（用 `cv2.projectPoints` 把 tag 局部系下三个端点 `[(0.03,0,0), (0,0.03,0), (0,0,0.03)]` 投影到像素，红=x、绿=y、蓝=z）
  4. 右下角 HUD：`margin=XX.X  fps=XX  id=N`
- **D-19：不广播 TF、不发 `/apriltag/debug_image`、不内嵌 rviz2。** 这些功能 Phase 9（或更后）按需再加；本阶段保持节点单职责。
- **D-20：FPS HUD 用滑动窗口测量节点回调频率（最近 30 帧平均），便于现场判断 USB 带宽 / 处理瓶颈。**

### Agent's Discretion

- **TF lookup 失败处理**：tag 检测到但 `tf_buffer.transform()` 抛 `TransformException` → **丢弃当前帧**（不发布两个 PoseStamped），用 `get_logger().warn(...)` + 简单节流（自维护一个上次 warn 时间）记录原因。与 `ik_fcl_ompl_planner.cpp` L382 一致的处理思路。
- **节点启动早于 robot_state_publisher**：用 `tf_buffer.can_transform("torso_link", "camera_color_optical_frame", time, timeout)` 在主回调中按需 wait；不在 `__init__` 中阻塞。如果首帧 lookup 失败，按上一条丢弃即可。
- **OpenCV 绘制实现细节**（颜色饱和度、文字字号、HUD 位置、PnP 三轴长度）— 选简洁清晰的默认值即可。
- **CMakeLists 中 `install(PROGRAMS …)`**：与现有 `tcp_torso_pose.py` / `keyboard_trigger_node.py` 同一行还是分开 — 选格式一致即可。
- **YAML 中是否给所有参数都写注释**：给关键的（tag_size、offset_xyz、target_tag_id、decision_margin_min）写一行说明；其余可自解释。
- **`apriltag_detector_node.py` 内部代码组织**（一个 `Node` 子类还是拆模块）— 与 `tcp_torso_pose.py` 风格一致，单类即可。

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### 新增文件（Phase 7 创建）
- `src/unitree_g1_dex3_stack-main/scripts/apriltag_detector_node.py` — 主节点
- `src/unitree_g1_dex3_stack-main/config/apriltag.yaml` — ROS 参数文件（D-08 字段清单）
- `src/unitree_g1_dex3_stack-main/launch/apriltag.launch.py` — 独立测试 launch

### 修改文件（Phase 7 增量）
- `src/unitree_g1_dex3_stack-main/CMakeLists.txt` — 添加 `install(PROGRAMS scripts/apriltag_detector_node.py)`、新增 `install(DIRECTORY config DESTINATION share/${PROJECT_NAME})`
- `src/unitree_g1_dex3_stack-main/package.xml` — 添加 `tf2_ros`、`tf2_geometry_msgs`、`cv_bridge`、`realsense2_camera`、`python3-opencv` 的 `<exec_depend>`

### Pattern 参考（节点实现 / launch 组合）
- `src/unitree_g1_dex3_stack-main/scripts/tcp_torso_pose.py` — Python ROS 2 节点骨架；`declare_parameter` + `get_parameter` 模式；`/robot_description` fallback；KeyboardInterrupt 收尾。**apriltag 节点不需要 KDL，但参数模式直接复用。**
- `src/unitree_g1_dex3_stack-main/scripts/keyboard_trigger_node.py` — Python ROS 2 节点惯例（最小骨架）；`#!/usr/bin/env python3` shebang
- `src/unitree_g1_dex3_stack-main/src/visual_detection_tester.cpp` — `cv::namedWindow` + 在 `imageCallback` 中 `cv::imshow` + `tf2_ros::Buffer::transform` 模式
- `src/unitree_g1_dex3_stack-main/launch/visual_detect_click.launch.py` — Launch 组合模板：robot include + realsense include + d435 static TF + Node。**几乎是 apriltag.launch.py 的母版**，把 `visual_detection_tester` 替换为 `apriltag_detector_node`，rs profile 从 1280x720 改为 640x480x15，关闭 align_depth
- `src/unitree_g1_dex3_stack-main/launch/reach.launch.py` — Phase 6 精简后形态；`d435_tf_node` 已经是 `[0,0,0,0,0,0,d435_link,camera_link]` 的 static publisher

### TF + 接口
- `src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` L51-52, L277, L378-382 — `tf_buffer_(this->get_clock())` + `tf_listener_(tf_buffer_)` + `tf_buffer_.transform(pose, base_link, durationFromSec(0.5))` + `try / catch (tf2::TransformException&)`。**Python 等价**：`from tf2_ros import Buffer, TransformListener; from tf2_geometry_msgs import do_transform_pose;` 然后 `tf_buffer.transform(pose_stamped, target_frame, timeout)`（依赖 `tf2_geometry_msgs` Python 注册）
- `src/unitree_g1_dex3_stack-main/launch/robot.launch.py` — URDF 加载到 `/robot_description`；提供 `torso_link` → `d435_link` static TF；apriltag.launch.py 必须 include 此 launch
- `src/unitree_g1_dex3_stack-main/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf` L425, L512-516 — `torso_link` 与 `d435_link` 定义、二者 fixed joint

### 接口契约（不变）
- `realsense2_camera` 包发布 `/camera/color/image_raw`（`sensor_msgs/Image`）+ `/camera/color/camera_info`（`sensor_msgs/CameraInfo`），frame_id = `camera_color_optical_frame`
- `geometry_msgs/PoseStamped` — 输出消息类型；frame_id 由节点设置为 `torso_link`
- `pupil_apriltags.Detector(families="tag36h11")` 返回 `Detection` 对象，含 `tag_id`、`hamming`、`decision_margin`、`corners` (4×2 ndarray)、`pose_R` (3×3) / `pose_t` (3×1)（仅当 `estimate_tag_pose=True` 且传入 `tag_size` + `camera_params`）

### 前序阶段约束
- `.planning/phases/06-yolo-tcp-offset/06-CONTEXT.md` — Phase 6 已彻底移除 YOLO 链路；保留 `visual_detection_tester.cpp` 与 `visual_detect_click.launch.py` 作为 AprilTag 调试参考；TCP offset 已通过 `right_tcp_link` 集成到 planner IK 链 — **本阶段不涉及 TCP offset**
- `.planning/phases/05-end-to-end-integration/05-CONTEXT.md` D-07/D-08 — `keyboard_trigger_node.py` 当前订阅 `/detections_3d` 发 `/goal_pose` 的旧链路 — **本阶段不修改该节点**；Phase 9 才会改造它
- `.planning/phases/04-right-arm-only-executor/04-CONTEXT.md` — executor 接受 `right_*_joint` 7 关节 trajectory；不受本阶段影响
- `.planning/REQUIREMENTS.md` TAG-01..TAG-04 — 本阶段必须满足的需求条目
- `.planning/ROADMAP.md` Phase 7 success criteria 1-6 — 验收清单

### 配置 & 依赖说明
- `pupil-apriltags`（pip 包）— 用户已 `pip install pupil-apriltags`；README 在节点章节里需提示新部署机器执行同样命令；package.xml 不写（rosdep 不知道 pip 包）
- `python3-opencv` — package.xml 写 `<exec_depend>python3-opencv</exec_depend>`，rosdep 可装；cv_bridge 已是项目依赖
- RealSense `rgb_camera.profile=640x480x15` — 与 `visual_detect_click.launch.py` 的 `1280x720x15` 不同，是用户在 Phase 7 决议变更
- `align_depth.enable=false` — apriltag PnP 不需要 depth；省 USB 带宽、降相机发热

### 库 API 速记（pupil-apriltags）
```python
from pupil_apriltags import Detector
detector = Detector(
    families="tag36h11",
    nthreads=1,            # 单线程足够，避免 GIL 抢占
    quad_decimate=1.0,     # 1.0 = 全分辨率；提高数字会下采样加速但损失精度
    refine_edges=1,
)
# camera_params = (fx, fy, cx, cy) — 从 /camera/color/camera_info 读
# K 矩阵：[0,0]=fx, [1,1]=fy, [0,2]=cx, [1,2]=cy
detections = detector.detect(
    gray_image,                     # 单通道 uint8
    estimate_tag_pose=True,
    camera_params=(fx, fy, cx, cy),
    tag_size=0.08,                  # 米
)
for d in detections:
    if d.tag_id != target_tag_id: continue
    if d.hamming != 0: continue                          # D-11
    if d.decision_margin < decision_margin_min: continue # D-10
    # d.pose_R (3×3) 和 d.pose_t (3×1) 是 tag 在相机系下的旋转和平移
```

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- **`scripts/tcp_torso_pose.py`** — 复用 ROS 2 Python 节点骨架：`super().__init__("apriltag_detector")`、`declare_parameter(...)`、`get_parameter(...).value`、Logger 风格、KeyboardInterrupt 收尾。**KDL 部分用不到，跳过。**
- **`scripts/keyboard_trigger_node.py`** — 最小 Python 节点骨架；`emulate_tty=True` 使日志即时显示
- **`src/visual_detection_tester.cpp`** L31-69 — `tf_buffer_(this->get_clock())` + `tf_listener_(tf_buffer_)` + `tf_buffer_.setCreateTimerInterface(...)` 模式；C++ 写法转译为 Python：`Buffer(self.get_clock())`、`TransformListener(self.tf_buffer, self)`
- **`src/visual_detection_tester.cpp`** L67-77 — `cv::namedWindow` + `cv::imshow` + `cv::waitKey` 与节点回调共存模式；Python 节点直接 `cv2.namedWindow + cv2.imshow + cv2.waitKey(1)` 即可
- **`launch/visual_detect_click.launch.py`** — 整套 launch 组合（robot + realsense + d435 static TF + node）；`apriltag.launch.py` 几乎照搬，只换 rs profile 和 node executable

### Established Patterns
- **ROS 2 Python 节点：** `#!/usr/bin/env python3` + `class Node(rclpy.node.Node)` + `def main(args=None): rclpy.init(...); rclpy.spin(node); finally: node.destroy_node(); rclpy.shutdown()` + `if __name__ == '__main__': main()`
- **参数：** `self.declare_parameter('name', default)` + `self.get_parameter('name').value`；类型由默认值推断
- **Launch 文件：** `OpaqueFunction` 模式（如需运行时解析 `LaunchConfiguration`）；普通节点用 `Node(parameters=[<yaml_path>])` 直接加载
- **CycloneDDS 环境：** 由 `robot.launch.py` 通过 `SetEnvironmentVariable` 设置；apriltag.launch.py include 后自动继承，**不要重复设置**
- **QoS：** RGB image 用 `rclpy.qos.qos_profile_sensor_data`（best-effort），与 `visual_detection_tester.cpp` L70-72 一致；PoseStamped 用默认 reliable QoS（与 planner 订阅 `/goal_pose` 的 reliable QoS 匹配）

### Integration Points
- **TF 树：** `realsense2_camera` 默认会发相机内部链路 TF；URDF 提供 `torso_link → d435_link`；`d435_link_to_camera_link` static publisher 接上 `d435_link → camera_link`。最终能从 `camera_color_optical_frame` 一路 lookup 到 `torso_link`。
- **`/camera/color/camera_info`：** PnP 需要 `fx, fy, cx, cy`；从 CameraInfo 的 `K` 矩阵取。CameraInfo 仅在节点启动后第一帧到达时缓存即可（之后不变）。
- **没有新的 colcon build 目标**（Python 节点不需要 `add_executable`）— 仅修改 install 列表，build 改动最小化
- **既有 `reach.launch.py` 不改** — Phase 6 已经把 `d435_tf_node` 留在那里供本阶段使用；本阶段独立 launch `apriltag.launch.py` 不影响 `reach.launch.py`

### Build/Runtime Concerns
- **`pupil-apriltags` 系统 Python 安装路径**：`pip install pupil-apriltags` 默认装在 `~/.local/lib/python3.X/site-packages/`；ROS 2 spawn 节点时 `PYTHONPATH` 须包含此路径。Ubuntu 默认 user-site 已在 `sys.path`，无需额外设置
- **OpenCV 窗口 + 多线程**：rclpy 默认单线程 executor；`cv2.imshow` 与 `cv2.waitKey(1)` 在 image 回调内调用即可，与 `visual_detection_tester.cpp` 的 imageCallback 同步模式一致
- **realsense2_camera 启动延迟约 5s**（Phase 5 已记录）；如果检测节点启动后等不到第一帧，先打印 warn 即可，不要 fatal

</code_context>

<specifics>
## Specific Ideas

- **D435i 分辨率全局调整为 640×480 @ 15fps**（rgb 与 depth profile 一致）。这是 Phase 7 决议的环境变更，与 `visual_detect_click.launch.py` 的 1280×720 不同。Phase 9 端到端 launch 应沿用 640×480。
- **`pupil-apriltags` 已手动 `pip install` 到系统 Python**，节点直接 `from pupil_apriltags import Detector` 即可。README/CLAUDE.md（如有）需要在 Phase 7 实现时**追加一行安装提示**给新部署机器。
- **过滤反馈直接画在 OpenCV 窗口上**（绿框/红框区分 accept/reject）— 用户的核心调试需求是"判断检测结果可信度"，可视化反馈比日志更直接。
- **HUD 显示 fps + decision_margin** — 现场判断 USB 带宽是否够、margin 阈值是否合适。
- **`cv2.projectPoints` 画三轴**：定义 tag 局部系下三个端点 `[(0.03,0,0), (0,0.03,0), (0,0,0.03)]` 和原点 `(0,0,0)`，`projectPoints` 投影到像素后用 `cv2.line` 连线。这是经典的 ARUCO/AprilTag 可视化模式。
- **`imshow` 走 launch arg**（不放 YAML） — 它影响节点是否创建窗口，是部署/环境维度参数；YAML 留给检测行为参数。

</specifics>

<deferred>
## Deferred Ideas

以下想法在讨论中浮现但**不属于 Phase 7 范围**，记录在此供 ROADMAP backlog / 未来阶段参考：

- **Multi-tag id→offset 表（Future REQ TAG-05）** — 一个 tag 对应一个物品，多 tag 时可以扩展 YAML schema：`tags: {0: {offset_xyz: [...]}, 1: {offset_xyz: [...]}}`。当前阶段单 tag 完全够用，schema 升级时 deserialization 改一改即可。
- **跨帧平滑（N 帧稳定确认 / 滑动平均）** — 真要做应该放在 Phase 9 的桥接节点里：取 `/apriltag/target_pose` 最近 N 帧平均后写入 `/goal_pose`。本阶段不做。
- **TF 广播 `torso_link → apriltag_<id>` / `apriltag_<id>_target`** — 用户明确不需要 RViz / TF 三轴可视化。Phase 9 如果有下游节点想 `lookup_transform` 查询 tag 位姿，再加 `tf2_ros::TransformBroadcaster`，约 15 行代码。
- **`/apriltag/debug_image` topic（带角点叠加的 RGB 图）** — 替代方案是 OpenCV imshow 直接显示。如果未来需要远程查看（无 GUI），加这个 topic + RViz Image display 即可。
- **RViz2 自带预设 (`config/apriltag_debug.rviz`)** — 用户拒绝。Phase 9 端到端展示如果想要 RViz 演示，再创建。
- **Phase 9 桥接 `/apriltag/target_pose → /goal_pose`** — 三种路径：(a) launch remap，(b) 改造 `keyboard_trigger_node.py` 改订阅 `/apriltag/target_pose` 而非 `/detections_3d`（PoseStamped 直发，无需 bbox 计算逻辑），(c) 新写一个桥接节点。Phase 9 决定。
- **节点性能优化（quad_decimate>1.0、多线程）** — 640×480@15fps 当前足够；如果未来分辨率提升或现场 latency 过高，再调 `pupil-apriltags` 的 `quad_decimate` 与 `nthreads` 参数。
- **基于 tag 法线推导接近方向（Future REQ ORI-03）** — Phase 8 自适应位姿可能用到 tag 的 z 轴（法向）作为接近方向输入；需要 Phase 7 已发布的 `/apriltag/tag_pose` 提供完整的 6-DOF。本阶段输出已具备这个信息，Phase 8 直接订阅即可。
- **多候选姿态 fallback（Future REQ ORI-02）** — Phase 8 范围。

</deferred>

---

*Phase: 07-apriltag*
*Context gathered: 2026-05-18*
