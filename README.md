# unitree_g1_dex3_stack

Unitree G1 右臂 + Dex-3 灵巧手 ROS 2 全栈：AprilTag V4L2 检测 → OMPL 规划 → 右臂执行 → 灵巧手按压。

**运行环境**：所有 ROS 2 节点均在 Docker 容器内运行（`unitree-dex3:humble`），宿主机通过 `run.sh` 启动。

---

## 1. Docker 环境

```bash
# 启动容器（交互式 shell，自动 source ROS 和 install_container）
cd /home/unitree/Desktop/unitree_container
./run.sh

# 容器内 shell 已自动 source：
#   /opt/ros/humble/setup.bash
#   /opt/unitree_ros2/cyclonedds_ws/install/setup.bash
#   /workspaces/unitree_dex3/install_container/setup.bash

# 直接在容器内执行单条命令（不进入 shell）
./run.sh <command>
```

> **注意**：首次 `./run.sh` 会创建后台容器（`unitree-dex3-dev`），后续调用复用已有容器。

### 1.1 新机器从零部署

新机器建议按 **Docker 部署**，不要走旧的原生 ROS 编译流程。

#### 需要拷贝的目录

默认放在新机器的 `/home/unitree/Desktop/` 下：

| 目录 | 是否必须 | 说明 |
|---|---|---|
| `unitree_container/` | 必须 | Dockerfile、镜像构建脚本、容器启动脚本、DDS/环境变量配置 |
| `unitree_dex3/` | 必须 | ROS 2 工作区；至少需要 `src/fcl`、`src/trac_ik`、`src/unitree_g1_dex3_stack-main` |
| `unitree_dex3_cpp/` | Button Press 必须 | Dex-3 灵巧手 setpoint 脚本和 `unitree_cpp` Python 绑定 |
| `xr_teleoperate/` | 可选 | 仅 `right-arm-mode` 需要其 URDF/资源 |
| `unitree_sdk2_python/` | 可选 | 镜像内默认已安装；本地挂载只是备用 |

不需要拷贝 `build/`、`install/`、`log/`、`build_container/`、`install_container/`、`log_container/`，这些都是编译产物，到新机器后重新生成。

#### Step 1：构建 Docker 镜像

```bash
cd /home/unitree/Desktop/unitree_container
bash build.sh
```

如果需要代理：

```bash
USE_PROXY=1 PROXY_URL=http://192.168.100.20:7890 bash build.sh
```

#### Step 2：首次全量编译 ROS 2 工作区

首次编译必须让 `colcon` 从 `src/` 里按依赖顺序构建 `fcl`、`trac_ik_lib` 和主包：

```bash
cd /home/unitree/Desktop/unitree_container
./run.sh bash -lc 'colcon --log-base /workspaces/unitree_dex3/log_container build \
  --base-paths /workspaces/unitree_dex3/src \
  --build-base /workspaces/unitree_dex3/build_container \
  --install-base /workspaces/unitree_dex3/install_container \
  --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON -DPython3_EXECUTABLE=/usr/bin/python3'
```

> **坑**：首次编译不要加 `--packages-select unitree_g1_dex3_stack`。主包依赖源码工作区里的 `fcl` 和 `trac_ik_lib`，必须先把依赖编出来。后续只改主包时才用 `--packages-select unitree_g1_dex3_stack` 增量编译。

#### Step 3：编译 Dex-3 Python 绑定

Button Press 的灵巧手控制脚本依赖 `unitree_dex3_cpp`：

```bash
cd /home/unitree/Desktop/unitree_container
./run.sh bash -lc 'cd /workspaces/unitree_dex3_cpp && \
  python3 -m pip install -e . --no-build-isolation --no-deps && \
  python3 -c "import unitree_cpp; print(\"unitree_cpp OK\")"'
```

#### Step 4：基础验证

```bash
cd /home/unitree/Desktop/unitree_container

# 主包是否可被 ROS 找到
./run.sh bash -lc 'ros2 pkg prefix unitree_g1_dex3_stack'

# 安全验证：只启动相机识别，不启动 planner / executor / button press 节点
./run.sh ros2 launch unitree_g1_dex3_stack apriltag_button_press.launch.py camera_only:=true
```

#### 新机器需要检查的硬件参数

| 参数 | 位置 | 说明 |
|---|---|---|
| `UNITREE_NET_IF` | `unitree_container/run.sh` 或启动前环境变量 | G1 通信网卡，当前默认 `enP8p1s0` |
| `expected_serial` | `config/apriltag_button_press.yaml` / `config/v4l2_apriltag_trigger.yaml` | D435i 序列号，换相机后需要改 |
| `camera_matrix` | 同上 | D435i RGB 相机内参，严格场景应使用新相机标定值 |

---

## 2. 编译

所有编译必须在容器内执行，使用 `build_container` / `install_container` / `log_container` 三套目录。

### 首次全量编译（从宿主机）

```bash
cd /home/unitree/Desktop/unitree_container
./run.sh bash -lc 'colcon --log-base /workspaces/unitree_dex3/log_container build \
  --base-paths /workspaces/unitree_dex3/src \
  --build-base /workspaces/unitree_dex3/build_container \
  --install-base /workspaces/unitree_dex3/install_container \
  --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON -DPython3_EXECUTABLE=/usr/bin/python3'
```

### 后续只重编主包

```bash
cd /home/unitree/Desktop/unitree_container
./run.sh bash -lc 'colcon --log-base /workspaces/unitree_dex3/log_container build \
  --base-paths /workspaces/unitree_dex3/src \
  --build-base /workspaces/unitree_dex3/build_container \
  --install-base /workspaces/unitree_dex3/install_container \
  --packages-select unitree_g1_dex3_stack \
  --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON -DPython3_EXECUTABLE=/usr/bin/python3'
```

编译后容器 shell 会自动 source `install_container/setup.bash`，无需手动 source。

> **坑**：修改源码后必须重新编译，`install_container/` 是编译产物的拷贝，直接改源码不会生效。

---

## 3. Launch 入口总览

| Launch 文件 | 用途 |
|---|---|
| `apriltag_button_press.launch.py` | **按钮按压全流程**：AprilTag 检测 → planner → executor → Dex-3 灵巧手伸指/按压/收回 |
| `apriltag_reach.launch.py` | **端到端到达**：AprilTag 检测 → G 键触发 → planner → executor（不含灵巧手） |
| `reach.launch.py` | **Planner 手动测试**：仅 robot + planner + executor，手动 pub `/goal_pose` |
| `apriltag.launch.py` | **相机独立调试**：仅 robot + V4L2 AprilTag 检测，不启动 planner/executor |

---

## 4. Button Press（按钮按压）

### 4.1 快速启动

```bash
cd /home/unitree/Desktop/unitree_container

# Dry-run（不控制灵巧手，安全测试规划和手臂运动）
./run.sh ros2 launch unitree_g1_dex3_stack apriltag_button_press.launch.py dry_run:=true

# 真机执行（含灵巧手伸指和按压）
./run.sh ros2 launch unitree_g1_dex3_stack apriltag_button_press.launch.py dry_run:=false

# 只启动相机识别（不启动 planner / executor / button press 节点）
./run.sh ros2 launch unitree_g1_dex3_stack apriltag_button_press.launch.py camera_only:=true
```

启动后在终端**按 G 键**触发一次按压序列。

### 4.2 按压执行流程

按 G 后，`apriltag_button_press_node` 依次执行：

1. **拍照 & 检测**：发布 `/apriltag/capture_trigger` → V4L2 trigger 拍照 → AprilTag 检测 → 发布 `/apriltag/target_pose`
2. **pre-contact**：手臂先运动到目标前方 `pre_contact_offset_x`（默认 5cm）处
3. **伸出中指**：调用 Dex-3 setpoint 脚本设置 `pre_extend_pose`
4. **press-target**：手臂前进到 tag 目标位置（按压）
5. **retreat**：手臂退回 pre-contact 位置
6. **合上手指**：调用 Dex-3 setpoint 脚本设置 `close_pose`
7. **return-to-standing**：发布 `/executor/return_to_standing`，手臂回到站立姿态

### 4.3 Launch 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `dry_run` | `false` | 跳过 Dex-3 灵巧手 subprocess 调用 |
| `camera_only` | `false` | 只启动相机和检测，不启动规划/控制 |
| `planning_timeout` | `1.0` | OMPL 规划超时（秒） |
| `v4l2_video_device` | `auto` | V4L2 设备路径，`auto` 自动扫描 D435i |
| `debug_image_dir` | `/workspaces/unitree_dex3/detect_img` | AprilTag 调试图像保存目录 |

### 4.4 关键配置文件

**`config/apriltag_button_press.yaml`** — 三个节点的参数集中配置：

#### V4L2 相机 & AprilTag 检测

| 参数 | 默认值 | 说明 |
|---|---|---|
| `expected_serial` | `253243060636` | D435i 序列号，用于 auto 模式设备匹配 |
| `video_device` | `auto` | V4L2 设备路径，`auto` 自动扫描 |
| `image_width` / `image_height` | `640` / `480` | 采集分辨率 |
| `warmup_frames` | `12` | 按 G 后先丢弃的暖机帧数 |
| `warmup_min_s` | `2.0` | 暖机最低时间（秒） |
| `sample_count` | `4` | 每次触发采样帧数 |
| `tag_size` | `0.08` | AprilTag 物理尺寸（米） |
| `target_tag_id` | `0` | 目标 tag ID |
| `offset_xyz` | `[0.0, 0.0, -0.1]` | tag 局部坐标系下的目标偏移 |
| `reach_max_distance` | `0.55` | 肩到目标最大距离（米） |
| `camera_matrix` | `[602.02, 0, 330.96, ...]` | 相机内参（640×480，出厂标定值） |

#### Button Press 节点

| 参数 | 默认值 | 说明 |
|---|---|---|
| `trigger_key` | `g` | 触发键 |
| `base_rpy` | `[0.0117, -0.0823, 0.0013]` | 默认末端朝向（RPY 弧度） |
| `alt_rpy_y_threshold` | `-0.0954` | 目标 y > 此值时切换备选朝向 |
| `alt_rpy` | `[-0.0340, -0.1024, 0.5480]` | 备选末端朝向 |
| `pre_contact_offset_x` | `0.05` | pre-contact 在 x 方向回退距离（米） |
| `pre_extend_pose` | `[0,-1.05,-1.7,1.7,1.8,0,0]` | 灵巧手伸中指关节角 |
| `close_pose` | `[0,-1.05,-1.7,1.7,1.8,1.7,1.8]` | 灵巧手合拢关节角 |
| `dex3_setpoint_script` | `/workspaces/unitree_dex3_cpp/example/control_dex3_right_setpoint.py` | 灵巧手控制脚本路径 |
| `dex3_net_if` | `enP8p1s0` | 灵巧手通信网卡 |
| `capture_wait_timeout_s` | `6.0` | 等待 AprilTag 检测结果超时 |
| `traj_wait_timeout_s` | `5.0` | 等待轨迹规划结果超时 |
| `traj_completion_buffer_s` | `0.8` | 轨迹执行完后额外等待时间 |
| `ramp_wait_s` | `3.5` | return-to-standing 后等待 ramp 完成 |

#### Planner

| 参数 | 默认值 | 说明 |
|---|---|---|
| `tcp_offset_x` | `0.12` | TCP 相对末端 link 的 x 偏移 |
| `adaptive_orientation_enabled` | `false` | button press 模式下关闭自适应朝向 |
| `velocity_scale` | `0.01` | 轨迹速度缩放 |
| `fallback_enabled` | `true` | IK/OMPL 失败时自动回退搜索 |
| `ik_timeout` | `0.1` | 单次 TRAC-IK 超时（秒） |

---

## 5. AprilTag Reach（到达）

```bash
cd /home/unitree/Desktop/unitree_container

# 端到端（检测 + 规划 + 执行）
./run.sh ros2 launch unitree_g1_dex3_stack apriltag_reach.launch.py

# 只检测不执行
./run.sh ros2 launch unitree_g1_dex3_stack apriltag_reach.launch.py detect_only:=true

# 只启动相机
./run.sh ros2 launch unitree_g1_dex3_stack apriltag_reach.launch.py camera_only:=true
```

启动后按 **G 键**触发检测和手臂运动。配置文件为 `config/v4l2_apriltag_trigger.yaml` + `config/apriltag.yaml`。

---

## 6. 其他常用操作

### 手动发布目标位姿

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "torso_link"}, pose: {position: {x: 0.4, y: -0.2, z: 0.18}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}' --once
```

### 读取右臂 TCP 位姿

```bash
# 先确保 robot.launch 在运行
ros2 run unitree_g1_dex3_stack tcp_torso_pose.py
```

### 手动触发相机拍照

```bash
ros2 topic pub /apriltag/capture_trigger std_msgs/msg/Empty '{}' --once
```

### 手动发送 return-to-standing

```bash
ros2 topic pub /executor/return_to_standing std_msgs/msg/Empty '{}' --once
```

### 右臂拖拽模式（卸力 / 锁定）

通过卸力和锁定实现右臂的拖拽和固定：

```bash
# 宿主机启动
cd /home/unitree/Desktop/unitree_container
./run.sh right-arm-mode
```

启动后进入交互模式，支持以下命令：

| 命令 | 功能 |
|------|------|
| `free` | 卸力，可自由拖动右臂 |
| `lock` | 锁定，右臂保持当前姿态不动 |
| `status` | 查看当前 7 个关节角度 |

> **注意**：`right-arm-mode` 依赖 `xr_teleoperate`（已包含在部署包中），Docker 内挂载路径为 `/workspaces/xr_teleoperate/teleop`。

### 灵巧手控制（容器内）

```bash
# 伸出中指
python3 /workspaces/unitree_dex3_cpp/example/control_dex3_right_setpoint.py enP8p1s0 0 -1.05 -1.7 1.7 1.8 0 0

# 合上
python3 /workspaces/unitree_dex3_cpp/example/control_dex3_right_setpoint.py enP8p1s0 0 -1.05 -1.7 1.7 1.8 1.7 1.8
```

## 7. 问题排查

### 相机打不开 / V4L2 设备占用

```bash
# 检查 /dev/video4 是否被占用
fuser -v /dev/video4

# 常见占用者：teleimager.image_server
# 杀掉占用进程后重试
kill <PID>
```

> V4L2 auto 模式扫描 `/dev/v4l/by-path/` 和 `/dev/v4l/by-id/`，匹配 D435i 序列号 `253243060636` 和 USB interface `03`。
> 当前机器上通常是 `/dev/video4`。

### 图像偏暗

冷启动时前几帧可能偏暗。已通过 `warmup_frames: 12` 和 `warmup_min_s: 2.0` 缓解。
如果仍然偏暗，可尝试增大 `warmup_frames` 或启用 `continuous_capture: true`（但会增加 CPU 占用）。

### TRAC-IK 无解（"No solution found" 循环）

planner 日志中出现反复的 `TRAC-IK random-seed result: No solution found`。
当前已调优：`ik_timeout: 0.1`，`ik_random_seed_tries: 3`，失败后只输出一条 WARN 汇总。
如果目标确实不可达，会触发 fallback 回退搜索（沿肩→目标方向收缩，步长 `fallback_step: 0.005`）。

### Dex-3 setpoint 脚本找不到

```
Dex-3 setpoint script not found: /home/unitree/Desktop/unitree_dex3_cpp/example/control_dex3_right_setpoint.py
```

路径必须是**容器内路径** `/workspaces/unitree_dex3_cpp/example/control_dex3_right_setpoint.py`，不是宿主机路径 `/home/unitree/Desktop/...`。已在配置中修正。

### rclpy import 失败

宿主机有 Miniforge/Conda Python 3.13，会覆盖 `/usr/bin/env python3`。
所有 ROS 2 Python 节点 shebang 已改为 `#!/usr/bin/python3`（硬编码系统 Python 3.10）。

### launch 退出时节点超时被 SIGKILL

`apriltag_button_press_node` 已处理 SIGINT/SIGTERM：信号回调中调用 `rclpy.try_shutdown()`，避免被 launch 系统 10s 超时后强杀。

### DDS 断言崩溃

```
dds_writecdr_impl_common: Assertion ... failed
```

Python cyclonedds 意外加载了 ROS Humble 的 `/opt/ros/humble/.../libddsc.so`。
容器内通过 `LD_LIBRARY_PATH=/usr/local/lib:...` 确保优先加载 `/usr/local/lib/libddsc.so.0`。

---

## 8. 已知的坑

1. **源码改完必须重编译** — `install_container/` 是编译拷贝，直接改 `src/` 下的源码不会生效
2. **路径是容器路径** — 配置文件中所有路径（`dex3_setpoint_script`、`debug_image_dir` 等）都是 `/workspaces/...`，不是 `/home/unitree/Desktop/...`
3. **V4L2 设备独占** — 同一时间只能一个进程打开 `/dev/video4`。启动 AprilTag 前必须确认 teleimager 等没有占用
4. **不要用 conda python 跑 ROS 节点** — conda 环境下的 python3 无法 import rclpy
5. **button press 中 `detect_only: true`** — V4L2 trigger 在 button press launch 中固定为 detect_only 模式，它只检测和发布 pose，不直接发布 `/goal_pose`。按压流程由 `apriltag_button_press_node` 编排
6. **`trigger_key: ""` vs `trigger_key: "g"`** — button press 的 V4L2 trigger 不监听键盘（`trigger_key` 为空），而是由 button press 节点通过 `/apriltag/capture_trigger` topic 触发拍照
7. **`auto_return_to_standing: false`** — button press launch 关闭了 executor 的自动回站立，由 button press 节点在序列完成后主动发送 `/executor/return_to_standing`
8. **base_rpy / alt_rpy** — 按压朝向不用 `adaptive_orientation`，而是根据 tag y 坐标选择 `base_rpy` 或 `alt_rpy`。调姿态时改这两个值
9. **warmup 不能太短** — D435i 冷启动需要 2 秒以上才能稳定曝光，`warmup_min_s` 不要低于 2.0
10. **TF 命名空间隔离** — 所有节点使用 `/unitree_g1_dex3/tf` 和 `/unitree_g1_dex3/tf_static`，不是默认的 `/tf`。其他工具监听时注意 remap

---

## 9. 文件结构

```
src/unitree_g1_dex3_stack-main/
├── config/
│   ├── apriltag.yaml                  # apriltag_reach 配置（planner 参数）
│   ├── apriltag_button_press.yaml     # button press 全部参数
│   └── v4l2_apriltag_trigger.yaml     # apriltag_reach 的 V4L2 相机参数
├── launch/
│   ├── apriltag_button_press.launch.py
│   ├── apriltag_reach.launch.py
│   ├── apriltag.launch.py
│   ├── reach.launch.py
│   ├── planner.launch.py
│   ├── control.launch.py
│   └── robot.launch.py
├── scripts/
│   ├── v4l2_apriltag_trigger.py       # V4L2 AprilTag 检测节点
│   ├── apriltag_button_press_node.py  # 按压序列编排节点
│   ├── tcp_torso_pose.py              # TCP 位姿读取工具
│   └── apriltag_reach_uat.py          # 到达 UAT 测试
└── src/
    ├── ik_fcl_ompl_planner.cpp        # C++ OMPL 规划器
    └── joint_trajectory_executor.cpp  # C++ 轨迹执行器
```

---

## 10. ROS 2 Topic 速查

| Topic | 类型 | 说明 |
|---|---|---|
| `/apriltag/capture_trigger` | `std_msgs/Empty` | 触发 V4L2 拍照 |
| `/apriltag/tag_pose` | `geometry_msgs/PoseStamped` | tag 原始位姿（torso_link 系） |
| `/apriltag/target_pose` | `geometry_msgs/PoseStamped` | tag + offset 目标位姿 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 规划目标（planner 输入） |
| `/joint_trajectory_targets` | `trajectory_msgs/JointTrajectory` | 规划输出轨迹 |
| `/executor/return_to_standing` | `std_msgs/Empty` | 触发手臂回站立 |


---

# 技术架构（首页版）

> 原始文档路径：`unitree_dex3/src/unitree_g1_dex3_stack-main/docs/ARCHITECTURE.md`


## 1. 系统总览

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Docker 容器 (unitree-dex3:humble)                │
│                                                                         │
│  ┌──────────┐   ┌──────────────────┐   ┌───────────┐   ┌────────────┐  │
│  │ robot    │   │ v4l2_apriltag    │   │ ik_fcl    │   │ joint      │  │
│  │ _state   │   │ _trigger (Py)    │   │ _ompl     │   │ _trajectory│  │
│  │ _publisher│   │                  │   │ _planner  │   │ _executor  │  │
│  │ (ROS2)   │   │ AprilTag 检测     │   │ (C++)     │   │ (C++)      │  │
│  └────┬─────┘   └────────┬─────────┘   └─────┬─────┘   └──────┬─────┘  │
│       │                  │                    │                │         │
│       │   /joint_states  │ /apriltag/         │ /joint_        │ /arm   │
│       │                  │  target_pose       │  trajectory    │  _sdk  │
│       │                  │                    │  _targets      │        │
│  ┌────┴─────┐            │              ┌─────┴─────┐          │        │
│  │ joint    │            │              │           │          │        │
│  │ _state   │            │              │ /goal_pose│          │        │
│  │ _publisher│            │              │           │          │        │
│  │ (C++)    │            │              └───────────┘          │        │
│  └──────────┘            │                                     │        │
│                          │   ┌──────────────────────┐          │        │
│                          │   │ apriltag_button      │          │        │
│                          └──▶│ _press_node (Py)     ├──────────┘        │
│                              │ 序列编排器            │                   │
│                              └──────────────────────┘                   │
│                                                                         │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │ Unitree SDK (DDS)                                                │   │
│  │   /lf/lowstate ← G1 机体      /arm_sdk → G1 右臂 7-DOF          │   │
│  │   Dex-3 setpoint → 灵巧手 7-DOF (subprocess, unitree_sdk2py)    │   │
│  └──────────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────────┘
        │                                                   │
    ┌───┴───┐                                         ┌─────┴─────┐
    │D435i  │ V4L2 /dev/video4                        │ G1 机体    │
    │RGB    │ 640×480 YUYV                            │ DDS Topic │
    └───────┘                                         └───────────┘
```

---

## 2. 节点职责

### 2.1 基础层（robot.launch.py）

| 节点 | 语言 | 职责 |
|---|---|---|
| `robot_state_publisher` | C++ (ROS2 内置) | 加载 URDF，发布 `/unitree_g1_dex3/tf_static`，提供 `robot_description` 参数 |
| `joint_state_publisher` | C++ | 订阅 `/lf/lowstate` (Unitree SDK DDS)，解析 29-DOF 关节角度 + Dex-3 手指角度，发布 `/joint_states` |

URDF 文件：`robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf`

### 2.2 感知层（V4L2 AprilTag Trigger）

| 节点 | 语言 | 职责 |
|---|---|---|
| `v4l2_apriltag_trigger` | Python | V4L2 采集 D435i RGB → pupil-apriltags 检测 → TF 变换到 torso_link → 发布 tag/target pose |

**工作模式**：
- **on-demand**（默认）：收到触发后才打开 V4L2 设备，暖机 → 采样 → 检测 → 关闭设备
- **continuous**：后台线程持续读帧，触发时从缓存取最新帧

**触发方式**：
- 键盘 G 键（`trigger_key: "g"`）— 用于 `apriltag_reach`
- Topic `/apriltag/capture_trigger`（`trigger_key: ""`）— 用于 `button_press`，由编排节点发布

**输出**：
- `/apriltag/tag_pose` — tag 原始位姿（torso_link 系）
- `/apriltag/target_pose` — tag + `offset_xyz` 偏移后的目标位姿

### 2.3 规划层（ik_fcl_ompl_planner）

| 节点 | 语言 | 职责 |
|---|---|---|
| `ik_fcl_ompl_planner` | C++ | 接收 `/goal_pose` → TRAC-IK 求解 → OMPL RRTConnect 规划 → 发布关节轨迹 |

**核心流程**：

```
/goal_pose 到达
    │
    ▼
[自适应朝向] ← 可选：根据 shoulder→target 方向计算末端朝向
    │
    ▼
[TRAC-IK] ← 目标笛卡尔位姿 → 7-DOF 关节角
    │
    ├── 成功 → [OMPL RRTConnect] 当前关节角 → 目标关节角
    │               │
    │               ├── 成功 → 轨迹简化 → 时间参数化(velocity_scale) → 发布 /joint_trajectory_targets
    │               └── 失败 → fallback
    │
    └── 失败 → fallback: 沿 shoulder→target 方向逐步回退，每步重试 IK+OMPL
```

**Fallback 策略**：
- 从原目标向 `right_shoulder_pitch_link` 方向回退
- 步长 `fallback_step`（0.005m），最大回退 `fallback_max_retraction`（0.05m）

**碰撞检测**（可选）：
- FCL 库，基于 URDF 碰撞几何
- 默认关闭（`collision_detection_enabled: false`），因为右臂操作空间碰撞风险低

### 2.4 执行层（joint_trajectory_executor）

| 节点 | 语言 | 职责 |
|---|---|---|
| `joint_trajectory_executor` | C++ | 接收关节轨迹 → 250Hz 线性插值 → KDL 重力补偿 → 发布 `/arm_sdk` (LowCmd) |

**核心特性**：
- 订阅 `/joint_trajectory_targets`，沿 waypoint 做 250Hz (4ms) 线性插值
- 每个控制周期叠加 KDL RNEA 计算的重力力矩补偿
- 轨迹执行完毕后进入 **hold** 模式（维持最后关节角 + 重力补偿）
- 收到 `/executor/return_to_standing` 后，**ramp** 回站立姿态（kp/kd 渐变到 0）
- `auto_return_to_standing`：轨迹执行完自动回站立（reach 模式开启，button_press 关闭）

**控制参数**：
- 右臂 7-DOF：shoulder×3 + elbow×1 + wrist×3
- kp/kd 分关节配置，wrist kd = 5.0
- 输出到 Unitree `/arm_sdk` DDS topic（`unitree_hg/LowCmd`）

### 2.5 编排层（apriltag_button_press_node）

| 节点 | 语言 | 职责 |
|---|---|---|
| `apriltag_button_press_node` | Python | 键盘触发 → 拍照 → 规划 → 手臂运动 → 灵巧手控制 → 回位 |

**不同于 reach 的关键设计**：
- V4L2 trigger 以 `detect_only=true` 运行，不自主发布 `/goal_pose`
- 由 button_press_node 通过 topic 触发拍照、收集 target_pose、构造 goal_pose
- 末端朝向不用 adaptive_orientation，而是 RPY 查表（`base_rpy` / `alt_rpy`）
- 灵巧手通过 subprocess 调用 `control_dex3_right_setpoint.py`（独立于 ROS）

---

## 3. 数据流

### 3.1 AprilTag Reach 数据流

```
[键盘 G] → v4l2_apriltag_trigger
               │
               ├── /apriltag/target_pose ──▶ (内部) ──▶ /goal_pose
               │
               ▼
         ik_fcl_ompl_planner
               │
               ├── /joint_trajectory_targets
               │
               ▼
         joint_trajectory_executor
               │
               ├── /arm_sdk (LowCmd)
               │
               ▼
           G1 右臂运动
               │
               ▼
         [auto_return_to_standing] → ramp 回站立
```

### 3.2 Button Press 数据流

```
[键盘 G] → apriltag_button_press_node
               │
               ├── /apriltag/capture_trigger ──▶ v4l2_apriltag_trigger
               │                                      │
               │◀── /apriltag/target_pose ◀────────────┘
               │
               ├── (根据计算出april tag的偏移点在torso link下的y轴坐标选择两种姿态其一，避免右臂撞到机身)/goal_pose (pre-contact) ──▶ planner ──▶ executor ──▶ 手臂到 pre
               │
               ├── [subprocess] Dex-3 伸中指
               │
               ├── /goal_pose (press) ──▶ planner ──▶ executor ──▶ 手臂前进按压
               │
               ├── /goal_pose (retreat) ──▶ planner ──▶ executor ──▶ 手臂退回
               │
               ├── [subprocess] Dex-3 合拢
               │
               └── /executor/return_to_standing ──▶ executor ──▶ ramp 回站立
```

---

## 4. 坐标系

```
                    torso_link (base_link)
                         │
              ┌──────────┴──────────┐
              │                     │
     right_shoulder_pitch_link    d435_link ──▶ camera_link
              │                                     │
     right_shoulder_roll_link              camera_color_frame
              │                                     │
     right_shoulder_yaw_link           camera_color_optical_frame
              │                         (AprilTag 检测坐标系)
         right_elbow_link
              │
     right_wrist_roll_link
              │
     right_wrist_pitch_link
              │
     right_wrist_yaw_link
              │
       right_tcp_link  ← IK 求解末端 (含 tcp_offset_x = 0.12m)
```

**TF 命名空间**：`/unitree_g1_dex3/tf` 和 `/unitree_g1_dex3/tf_static`（与系统默认 `/tf` 隔离）

**相机 TF 链**：`d435_link` → `camera_link` → `camera_color_frame` → `camera_color_optical_frame`（静态 TF，launch 中发布）

AprilTag 检测在 `camera_color_optical_frame` 系下得到 tag 位姿，通过 TF 变换到 `torso_link` 系后发布。

---

## 5. 硬件接口

### 5.1 DDS Topic（Unitree SDK）

| Topic | 方向 | 说明 |
|---|---|---|
| `/lf/lowstate` | G1 → 容器 | 全身 35-DOF 关节状态 + IMU + 手指状态 |
| `/arm_sdk` | 容器 → G1 | 右臂 7-DOF 关节指令（位置+kp+kd+力矩） |

- 使用 CycloneDDS（`rmw_cyclonedds_cpp`）
- 网卡：`enP8p1s0`
- 容器通过 `--network host --privileged` 直连 DDS 网络

### 5.2 V4L2 相机

| 属性 | 值 |
|---|---|
| 设备 | Intel RealSense D435i（仅使用 RGB） |
| V4L2 节点 | `/dev/video4`（auto 扫描匹配序列号 `253243060636`） |
| 分辨率 | 640×480 |
| 格式 | YUYV |
| USB Interface | `03` |

不使用 RealSense SDK / `realsense2_camera`，直接 V4L2 采集 RGB。

### 5.3 Dex-3 灵巧手

| 属性 | 值 |
|---|---|
| 控制方式 | Unitree SDK2 Python，subprocess 调用 |
| 脚本 | `/workspaces/unitree_dex3_cpp/example/control_dex3_right_setpoint.py` |
| 网卡 | `enP8p1s0` |
| 自由度 | 7-DOF (拇指×2 + 食指×2 + 中指×2 + 全合/全开×1) |

---

## 6. 构建系统

### 6.1 CMake 选项

| 选项 | 默认 | 说明 |
|---|---|---|
| `BUILD_IK_FCL_OMPL_PLANNER` | `OFF` | 编译 C++ 规划器（依赖 TRAC-IK, OMPL, FCL） |
| `Python3_EXECUTABLE` | 系统默认 | 容器内必须指定 `/usr/bin/python3` |

### 6.2 C++ 可执行文件

| 二进制 | 源文件 | 说明 |
|---|---|---|
| `joint_state_publisher` | `joint_state_publisher.cpp` | 关节状态发布 |
| `joint_trajectory_executor` | `joint_trajectory_executor.cpp` | 轨迹执行 + 重力补偿 |
| `ik_fcl_ompl_planner` | `ik_fcl_ompl_planner.cpp` | IK + 碰撞检测 + 运动规划 |
| `dex3_controller` | `dex3_controller.cpp` | Dex-3 ROS 控制器（未在当前 launch 中使用） |
| `right_hand_pressure_monitor` | `right_hand_pressure_monitor.cpp` | 手指压力监控 |
| `visual_detection_tester` | `visual_detection_tester.cpp` | 视觉检测调试工具 |

### 6.3 Python 脚本

| 脚本 | 说明 |
|---|---|
| `v4l2_apriltag_trigger.py` | V4L2 + AprilTag 检测主节点 |
| `apriltag_button_press_node.py` | 按压序列编排 |
| `tcp_torso_pose.py` | TCP 位姿读取工具 |
| `apriltag_reach_uat.py` | 到达验收测试 |
| `adaptive_orientation_ab.py` | 自适应朝向 A/B 测试 |
| `apriltag_goal_bridge.py` | AprilTag→goal_pose 桥接（reach 模式内置到 v4l2_trigger） |
| `gravity_torque_publisher.py` | 重力力矩发布（调试用） |

---

## 7. 配置文件对照

| 配置文件 | 使用场景 | 包含的节点参数 |
|---|---|---|
| `apriltag_button_press.yaml` | button_press launch | v4l2_trigger + button_press_node + planner |
| `v4l2_apriltag_trigger.yaml` | reach / apriltag launch | v4l2_trigger（检测参数不同，如 offset_xyz、trigger_key） |
| `apriltag.yaml` | reach launch | planner + tcp_torso_pose |

**button_press 和 reach 的配置差异**：

| 差异项 | button_press | reach |
|---|---|---|
| `offset_xyz` | `[0.0, 0.0, -0.1]` | `[0.12, -0.01, -0.13]` |
| `trigger_key` | `""` (topic 触发) | `"g"` (键盘触发) |
| `detect_only` | `true` (固定) | `false` (默认) |
| `adaptive_orientation` | `false` | `true` |
| `velocity_scale` | `0.01` | `0.02` |
| `auto_return_to_standing` | `false` | `true` |

---

## 8. Launch 组合关系

```
apriltag_button_press.launch.py
├── robot.launch.py
│   ├── robot_state_publisher (URDF)
│   └── joint_state_publisher (DDS → /joint_states)
├── static TF: d435_link → camera_link → camera_color_frame → camera_color_optical_frame
├── [TimerAction 3s]
│   ├── v4l2_apriltag_trigger (detect_only=true, trigger_key="")
│   ├── planner.launch.py
│   │   └── ik_fcl_ompl_planner
│   ├── control.launch.py
│   │   └── joint_trajectory_executor (auto_return_to_standing=false)
│   └── apriltag_button_press_node

apriltag_reach.launch.py
├── robot.launch.py
├── static TF (同上)
├── [TimerAction 3s]
│   ├── v4l2_apriltag_trigger (detect_only=false, trigger_key="g")
│   ├── planner.launch.py
│   └── control.launch.py

reach.launch.py (手动测试)
├── robot.launch.py
├── planner.launch.py
└── control.launch.py

apriltag.launch.py (相机调试)
├── robot.launch.py
├── static TF
└── v4l2_apriltag_trigger
```

---

## 9. 运行时依赖

| 层级 | 依赖 |
|---|---|
| **系统** | ROS 2 Humble, CycloneDDS, Python 3.10 |
| **C++ 库** | KDL, TRAC-IK, OMPL, FCL, geometric_shapes |
| **Python 库** | pupil-apriltags, scipy, numpy, opencv-python, tf2_ros |
| **Unitree** | unitree_hg (ROS msg), unitree_sdk2_python (Dex-3 控制) |
| **硬件** | G1 机体 (DDS), D435i (V4L2), Dex-3 灵巧手 (DDS) |
