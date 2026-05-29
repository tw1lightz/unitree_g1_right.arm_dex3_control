# 新机器编译指南

本文档记录在一台新的 Ubuntu 22.04 / ROS 2 Humble 机器上编译 `unitree_dex3` 工作区的流程。流程基于 G1 NX 实机验证。

## 1. 适用环境

- Ubuntu 22.04
- ROS 2 Humble
- `colcon` 工作区路径示例：`~/Desktop/unitree_dex3`
- Unitree ROS2 消息工作区路径示例：`~/unitree_ros2`

如果机器需要代理，先在终端启用代理，再执行下载或 `apt` 操作。

## 2. 安装系统依赖

先安装 ROS 2 Humble 和基础构建工具，然后安装本项目编译需要的依赖：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rosidl-generator-dds-idl \
  ros-humble-vision-msgs \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-message-filters \
  ros-humble-pcl-conversions \
  ros-humble-tf2 \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-robot-state-publisher \
  ros-humble-kdl-parser \
  ros-humble-urdfdom-py \
  ros-humble-urdf \
  ros-humble-ompl \
  ros-humble-geometric-shapes \
  ros-humble-resource-retriever \
  libccd-dev \
  libnlopt-dev \
  libeigen3-dev \
  liboctomap-dev \
  libopencv-dev \
  libpcl-dev \
  python3-pykdl \
  python3-numpy \
  python3-pip
```

### 2.1 Python 依赖（标定与重力补偿脚本）

标定脚本（`scripts/calibrate_kdl_tau.py`、`scripts/gravity_calibration_manual.py`）需要 `PyKDL` 和 `numpy`，已在上面通过 apt 安装。

如果需要使用 Pinocchio 重力补偿节点（`gravity_torque_publisher.py`，可选），安装 Pinocchio：

```bash
pip3 install pin
```

### 2.2 AprilTag 检测的 Python 依赖

`apriltag_detector_node.py` 和 `apriltag_goal_bridge.py` 依赖以下 pip 包：

```bash
pip install pupil-apriltags
```

**注意**: pip 安装的 `numpy` 版本可能和 ROS 2 Humble 的 `cv_bridge`（基于 numpy 1.x 编译）冲突。如果运行时出现：

```text
AttributeError: _ARRAY_API not found
```

说明 pip 的 numpy 2.x 覆盖了系统 numpy 1.x。降级即可：

```bash
pip install "numpy<2"
```

同理，如果 `pip install opencv-python` 版本过新，也可能与 cv_bridge 冲突。本项目的图像处理通过 apt 的 `python3-opencv` 完成，不需要 pip 的 `opencv-python`；如有冲突可卸载：

```bash
pip uninstall opencv-python
```

`libccd-dev` 是必须项；否则源码内的 `fcl` 会在 CMake 阶段报错：

```text
CCD is required by FCL
```

## 3. 准备 Unitree ROS2 消息包

本项目依赖 `unitree_hg` 消息包。它来自 Unitree 官方 `unitree_ros2` 仓库。

推荐放在用户目录：

```bash
cd ~
git clone https://github.com/unitreerobotics/unitree_ros2.git
```

构建 Unitree ROS2 工作区：

```bash
cd ~/unitree_ros2/cyclonedds_ws
source /opt/ros/humble/setup.bash
colcon build
```

验证 `unitree_hg` 是否安装成功：

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
ros2 pkg prefix unitree_hg
```

期望能输出类似：

```text
/home/unitree/unitree_ros2/cyclonedds_ws/install/unitree_hg
```

如果构建本项目时报：

```text
Could not find a package configuration file provided by "unitree_hg"
```

说明当前终端没有 source Unitree ROS2 工作区：

```bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
```

## 4. 准备 RealSense ROS2 工作区

`apriltag_reach.launch.py` 启动 RealSense D435 相机，依赖 `realsense2_camera` 消息包。如果机器上还未编译，先安装：

```bash
cd ~
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros
source /opt/ros/humble/setup.bash
colcon build
```

建议将 realsense 工作区的 source 写入 `~/.bashrc`：

```bash
echo "source ~/realsense-ros2/install/setup.bash" >> ~/.bashrc
```

否则每个新终端启动 launch 前都需要手动 source。

## 5. 准备本项目工作区

确保目录结构类似：

```text
~/Desktop/unitree_dex3/
  src/
    bboxes_ex_msgs/
    fcl/
    trac_ik/
    unitree_g1_dex3_stack-main/
```

进入工作区：

```bash
cd ~/Desktop/unitree_dex3
```

每次新开终端，先 source：

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/realsense-ros2/install/setup.bash
```

如果本工作区已经编译过，也可以追加：

```bash
source install/setup.bash
```

## 6. 编译

### 6.1 首次全量编译

```bash
cd ~/Desktop/unitree_dex3
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash

colcon build --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON
```

成功时应看到类似：

```text
Summary: 5 packages finished
```

只要没有 `failed`，就算编译成功。

### 6.2 只重编主包

修改 `unitree_g1_dex3_stack-main` 后，可只编译主包：

```bash
cd ~/Desktop/unitree_dex3
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source install/setup.bash 2>/dev/null || true

colcon build --packages-select unitree_g1_dex3_stack --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON
source install/setup.bash
```

## 7. 运行验证

### 7.1 启动 robot model

先启动 `robot_state_publisher`，让 planner 能读取正确的 `robot_description`：

```bash
cd ~/Desktop/unitree_dex3
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source install/setup.bash

ros2 launch unitree_g1_dex3_stack robot.launch.py
```

保持这个终端运行。

### 7.2 启动 planner

另开一个终端：

```bash
cd ~/Desktop/unitree_dex3
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source install/setup.bash

ros2 launch unitree_g1_dex3_stack planner.launch.py
```

启动成功后，应看到 planner 初始化日志，包括：

```text
Simplification: method=..., timeout=..., max_steps=..., max_empty_steps=...
Right arm: base_link = torso_link, tip_link = right_wrist_yaw_link
```

## 8. 常见问题

### 8.1 `unitree_hg` 找不到

错误：

```text
Could not find a package configuration file provided by "unitree_hg"
```

处理：

```bash
source /opt/ros/humble/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
```

然后重新构建。

### 8.2 `CCD is required by FCL`

错误：

```text
CCD is required by FCL
```

处理：

```bash
sudo apt update
sudo apt install -y libccd-dev
```

然后重新构建。

### 8.3 `resource_retriever/retriever.h` 找不到

ROS 2 Humble 使用：

```cpp
#include <resource_retriever/retriever.hpp>
```

不要使用旧头文件：

```cpp
#include <resource_retriever/retriever.h>
```

### 8.4 `ObjectHypothesisWithPose` 没有 `id` 或 `score`

ROS 2 Humble 的 `vision_msgs` 结构应使用：

```cpp
detection.results.front().hypothesis.class_id
detection.results.front().hypothesis.score
```

不要使用旧字段：

```cpp
detection.results.front().id
detection.results.front().score
```

### 8.5 `PathSimplifier` 没有 `shortcutPath`

ROS Humble 当前 OMPL 版本没有 `shortcutPath()`。本项目使用：

```cpp
ps.partialShortcutPath(path, max_steps, max_empty_steps);
```

### 8.6 `planner.launch.py` 报空 tuple 参数

错误：

```text
Expected 'value' to be one of [...], but got '()' of type '<class 'tuple'>'
```

原因通常是 launch 文件把空列表作为 ROS 参数传入。`collision_skip_pairs` 为空时不要传入该参数，让 C++ 节点使用默认空 vector。

### 8.7 `Failed to extract KDL chain for right arm`

错误：

```text
Failed to extract KDL chain for right arm
```

常见原因：没有先启动本项目的 `robot.launch.py`，或者当前系统中已有另一个 `/robot_state_publisher` 提供了错误的 `robot_description`。

正确顺序：

1. 先运行 `ros2 launch unitree_g1_dex3_stack robot.launch.py`
2. 再运行 `ros2 launch unitree_g1_dex3_stack planner.launch.py`

可检查当前 URDF 是否包含目标链路：

```bash
ros2 param get /robot_state_publisher robot_description | grep -E "torso_link|right_wrist_yaw_link|right_hand_palm_link"
```

### 8.8 `tcp_torso_pose.py` 报 `No module named 'kdl_parser_py'`

错误：

```text
ModuleNotFoundError: No module named 'kdl_parser_py'
```

原因：ROS 2 Humble 的 apt 仓库**不提供** `kdl_parser_py` 这个 Python 包，只有 C++ 版的 `ros-humble-kdl-parser`。上游 `ros/kdl_parser` 仓库 `humble` 分支虽然带 `kdl_parser_py` 子目录，但里面用的是 Python 2 写法（`kdl.Joint.None`），在 Python 3 下连导入都过不了，从源码构建也无法直接解决。

处理方式：本项目的 `tcp_torso_pose.py` 已不再依赖 `kdl_parser_py`，改为在脚本内内联一份 Py3 兼容的 `treeFromUrdfModel`（基于 `python3-pykdl` 提供的 `PyKDL.Joint.Fixed`）。脚本现在只依赖 `urdf_parser_py`：

```bash
sudo apt install -y ros-humble-urdfdom-py
```

如果机器上 `urdf_parser_py` 也找不到，会先报：

```text
ModuleNotFoundError: No module named 'urdf_parser_py'
```

按上面的命令装 `ros-humble-urdfdom-py` 即可。

### 8.9 `realsense2_camera` 找不到

错误：

```text
PackageNotFoundError: "package 'realsense2_camera' not found, searching: [...]
```

原因：`apriltag_reach.launch.py` 引用了 `realsense2_camera`，但当前终端没有 source realsense-ros2 工作区。

处理：

```bash
source ~/realsense-ros2/install/setup.bash
```

建议将该行写入 `~/.bashrc` 以持久化，见第 4 节。

### 8.10 `_ARRAY_API not found` / `No module named 'pupil_apriltags'`

错误：

```text
AttributeError: _ARRAY_API not found
ModuleNotFoundError: No module named 'pupil_apriltags'
```

原因：(1) pip 安装的 numpy 2.x 覆盖了系统 numpy 1.x，但 ROS 2 Humble 的 `cv_bridge` 是用 numpy 1.x 编译的；(2) `pupil_apriltags` 未安装。

处理：

```bash
pip install "numpy<2" pupil-apriltags
```

### 8.11 `right_shoulder_pitch_link` TF 不存在

运行 `apriltag_reach.launch.py` 后 `apriltag_goal_bridge` 反复报：

```text
"right_shoulder_pitch_link" passed to lookupTransform argument source_frame does not exist.
```

最后以 `FATAL` 退出。

原因：`joint_state_publisher` 无法从机器人订阅 `/lf/lowstate`（电机状态），导致 `/joint_states` 为空，`robot_state_publisher` 无法发布完整 TF 树。通常是因为机器人本体未上电、电机控制器未启动。

排查：

```bash
# 看 joint_states 是否有数据
ros2 topic echo /joint_states --once

# 看 lowstate 是否有数据
ros2 topic echo /lf/lowstate --once
```

如果 `/lf/lowstate` 无数据，需要先在机器人上启动 Unitree HG SDK，确保电机上电。

## 9. 编译成功判定

目标包成功：

```text
Finished <<< unitree_g1_dex3_stack
Summary: 1 package finished
```

全工作区成功：

```text
Summary: 5 packages finished
```

如果只有 `had stderr output`，但没有 `failed`，通常只是 warning，不是构建失败。
