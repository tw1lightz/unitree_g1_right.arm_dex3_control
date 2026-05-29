# Phase 6: YOLO 清理 + TCP Offset 集成 - Discussion Log

**Date:** 2026-05-15
**Duration:** ~12 minutes
**Areas discussed:** 4/4

## Area 1: YOLO 清理范围

### Q1: 清理策略
- **Options:** 彻底清理 / 只删 YOLO 直接相关 / Agent 决定
- **Selected:** 彻底清理
- **Note:** 删除所有 YOLO + project_to_3d + detection_to_goal + bboxes_ex_msgs + perception.launch.py + best.pt + run_perception.sh

### Q2: visual_detection_tester 处理
- **Options:** 一起删 / 保留 / Agent 决定
- **Selected:** 保留（对 AprilTag 调试有参考价值）

### Q3: 保留文件的编译依赖问题
- **Options:** 从 CMakeLists 移除编译 / 重构依赖 / Agent 决定
- **Selected:** Agent 决定

## Area 2: TCP offset 集成方式

### Q1: 集成方案
- **Options:** URDF 虚拟 fixed link / Planner 代码反向补偿 / KDL chain 动态添加 segment
- **Selected:** URDF 虚拟 fixed link（`right_tcp_link`）

### Q2: TCP offset 可配置实现
- **Options:** ROS 参数覆盖 URDF 值 / 只在 URDF 配置 / xacro 参数化
- **Selected:** ROS 参数 `tcp_offset_x` 覆盖 URDF 默认值

### Q3: 修改哪些 URDF 文件
- **Options:** 只改碰撞检测版 / 默认+碰撞检测版 / 三个全改
- **Selected:** 默认 + 碰撞检测版（RViz 可视化 + planner 都正确）

## Area 3: reach.launch.py 处理

### Q1: 处理方式
- **Options:** 删除 / 精简为 robot+planner+control / 精简但保留 keyboard_trigger_node
- **Selected:** 精简为 robot + planner + control

## Area 4: Planner 接口变化

### Q1: Phase 7 前的验证方式
- **Options:** ros2 topic pub 手动验证 / 保留 keyboard_trigger_node 独立运行 / 写测试脚本
- **Selected:** 纯 `ros2 topic pub` 手动验证

## Deferred Ideas

- keyboard_trigger_node 改造（Phase 9）
- 多 TCP offset 配置（未来需要时）
- URDF xacro 化（工作量大，当前不需要）

## Agent's Discretion Items

- `visual_detection_tester.cpp` 编译依赖处理方式
- `tcp_offset_x` 参数覆盖 KDL chain 末端偏移的具体实现
- `keyboard_trigger_node.py` 是否从 install 清理

---
*Discussion log: 2026-05-15*
