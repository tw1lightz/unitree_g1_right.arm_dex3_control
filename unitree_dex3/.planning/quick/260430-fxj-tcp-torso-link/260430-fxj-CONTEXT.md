# Quick Task 260430-fxj: 写一个计算右臂设定tcp在torso link下的位姿的脚本 - Context

**Gathered:** 2026-04-30
**Status:** Ready for planning

<domain>
## Task Boundary

编写一个脚本，计算右臂设定 TCP 在 torso_link 坐标系下的位姿。
</domain>

<decisions>
## Implementation Decisions

### TCP定义
- 父关节: right_wrist_yaw_joint
- 从该关节坐标系出发，沿 X 轴正方向偏移 0.145m
- 无旋转偏移（保持 wrist_yaw_joint 的朝向）

### 静态vs动态
- 动态 FK：订阅 /joint_states 获取当前关节角度，实时计算 TCP 在 torso_link 下的位姿

### 脚本形式
- ROS2 Python 节点，使用 PyKDL + URDF 做正向运动学

### 输出格式
- 简洁：xyz (米) + rpy (弧度)，欧拉角采用固定轴 (fixed-axis) XYZ 顺序

### Claude's Discretion
- 节点命名、topic 命名、参数声明方式遵循现有项目惯例
- TCP offset 通过 ROS2 parameter 声明，可运行时配置
- xyz 输出保留 4 位小数，rpy 保留 4 位小数
</decisions>

<specifics>
## Specific Ideas

- TCP 是从 right_wrist_yaw_joint 坐标系沿 +X 偏移 0.145m，方向与 wrist_yaw_joint 一致
- 该脚本用于实际点位示教/验证，所以需要实时关节角
</specifics>

<canonical_refs>
## Canonical References

- URDF: `src/unitree_g1_dex3_stack-main/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0.urdf`
- 现有 FK 参考: `src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` 中的 KDL ChainFkSolverPos_recursive 用法
- 现有 joint state 发布: `src/unitree_g1_dex3_stack-main/src/joint_state_publisher.cpp`
</canonical_refs>
