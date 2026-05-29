# Phase 5: End-to-End Integration - Discussion Log

> **Audit trail only.** Do not use as input to planning, research, or execution agents.
> Decisions are captured in CONTEXT.md — this log preserves the alternatives considered.

**Date:** 2026-05-14
**Phase:** 05-end-to-end-integration
**Areas discussed:** 集成启动文件结构, 目标触发机制, Conda 环境处理, README 更新范围

---

## A — 集成启动文件结构

| Option | Description | Selected |
|--------|-------------|----------|
| `launch/reach.launch.py` | 和其他 launch 文件放在一起，`ros2 launch` 可直接调用 | ✓ |
| 工作区根目录 `reach.launch.py` | 更显眼，但不在包内无法用 `ros2 launch` 调用 | |

**顶层参数：**

| Option | Description | Selected |
|--------|-------------|----------|
| 最小集 | `model_path`、`target_class`、`imshow`、`planning_timeout` | ✓ |
| 全透传 | 所有子 launch 参数都暴露，灵活但冗长 | |

**启动延迟：**

| Option | Description | Selected |
|--------|-------------|----------|
| 加延迟 | robot.launch.py 先起，TimerAction 3s 后再起其余三个 | ✓ |
| 不加延迟 | 依赖各节点自己的重试逻辑 | |

**CycloneDDS：**

| Option | Description | Selected |
|--------|-------------|----------|
| 直接 include robot.launch.py | CycloneDDS 设置自动继承 | ✓ |
| reach.launch.py 自己设置 | 更显式但重复代码 | |

---

## B — 目标触发机制

| Option | Description | Selected |
|--------|-------------|----------|
| CLI 手动发布 | `ros2 topic pub --once /detection_selection ...` | |
| 键盘节点 | 新建节点，按键触发 | ✓ |
| 自动触发 | 检测到目标后自动发布，可能误触发 | |

**用户补充：** 终端监听，按 K 时调用 YOLO 识别，进行坐标转换，执行动作。

**实现方式：**

| Option | Description | Selected |
|--------|-------------|----------|
| 新建 `keyboard_trigger_node.py` | 独立 Python 节点，~50 行 | ✓ |
| 修改 `detection_to_goal_node.cpp` | 侵入性更大 | |

**多目标选择：**

| Option | Description | Selected |
|--------|-------------|----------|
| 取置信度最高的 | `results[0].hypothesis.score` 最大 | |
| 取距离最近的 | 按 3D 位置到原点欧氏距离排序 | ✓ |
| 打印列表让用户再选 | 需要额外交互 | |

**类别过滤：**

| Option | Description | Selected |
|--------|-------------|----------|
| 按 target_class 过滤 | 只选特定类别 | |
| 不过滤 | 模型只训了一种物品，无需过滤 | ✓ |

**目标点计算（用户补充）：**
取 bbox 底部边中点，向上偏移 bbox 高度的 10%。坐标系 `camera_color_optical_frame`（y 轴朝下），"向上"= -y 方向：
- `y_target = center.position.y + size.y / 2 - size.y * 0.1`
- x、z 保持 bbox 中心不变

---

## C — Conda 环境处理

| Option | Description | Selected |
|--------|-------------|----------|
| README 说明前置条件 | 用户手动 `conda activate grab` | |
| wrapper 脚本 | `reach.sh` 先激活 conda 再 launch，可移植性差 | |
| pip 装到系统 Python | `pip install ultralytics torch`，不再依赖 conda | ✓ |

**用户提问：** 能否不用 conda，通过 ros2 编译方式处理依赖？
**结论：** colcon 无法管理 pip 包，选择直接 pip install 到系统 Python。

---

## D — README 更新范围

| Option | Description | Selected |
|--------|-------------|----------|
| 原地重写现有 README.md | 保持文件位置，内容替换 | ✓ |
| 工作区根目录新建 README.md | 更显眼但包内仍过时 | |
| 两个都写 | 根目录简短 + 包内详细 | |

**文档内容：**

| Option | Description | Selected |
|--------|-------------|----------|
| 最小实用版 | 环境准备、启动命令、按 K 触发、常用参数 | ✓（README.md）|
| 完整文档版 | 架构、话题、节点、参数、决策背景 | ✓（docs/ARCHITECTURE.md）|

**文档位置：**

| Option | Description | Selected |
|--------|-------------|----------|
| README.md + docs/ARCHITECTURE.md | README 简洁，完整文档在 docs/ | ✓ |
| README.md + README_FULL.md | 都在包根目录 | |

**完整文档是否含决策背景：** 是，包含各阶段"为什么这样做"。

---

## Agent's Discretion

- `keyboard_trigger_node.py` 键盘读取实现方式（`sys.stdin` raw mode 或 `readchar`）
- `reach.launch.py` 中 `keyboard_trigger_node` 是否需要 `emulate_tty=True`

## Deferred Ideas

- 类别过滤参数（未来多类别模型时添加）
- 自动触发模式（当前手动更安全）
- DEX3 手部控制（v1.0 范围外）
