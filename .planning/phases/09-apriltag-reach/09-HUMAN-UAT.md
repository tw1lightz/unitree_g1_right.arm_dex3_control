---
status: partial
phase: 09-apriltag-reach
source: [09-VERIFICATION.md]
started: 2026-05-19
updated: 2026-05-19
---

## Current Test

[awaiting human testing]

## Tests

### 1. End-to-end launch — 单命令启动完整流水线
expected: `ros2 launch unitree_g1_dex3_stack apriltag_reach.launch.py` 正常启动，7 个组件 (robot_state_publisher, RealSense, static TF, apriltag_detector, apriltag_goal_bridge, planner, executor) 均在 ros2 node list 可见
result: [pending]

### 2. Bridge G trigger — 按 G 触发目标发布
expected: tag 可见时按 G → `/goal_pose` topic 出现 PoseStamped 消息（`ros2 topic echo /goal_pose` 可观察到），bridge INFO 日志输出 target 坐标与 reach distance
result: [pending]

### 3. Guard behavior — 异常场景保护
expected:
- 遮挡/移除 tag → 按 G 收到 WARN "no fresh AprilTag pose"
- tag 未检测过 → 按 G 收到 WARN "no AprilTag detected yet"
- 运动过程中连续按 G → 第二次收到 WARN "previous goal still in flight"
result: [pending]

### 4. End-to-end UAT — 4 点 tabletop 验收
expected: `ros2 run unitree_g1_dex3_stack apriltag_reach_uat.py` 输出 4/4 PASS, 每点误差 ≤ 3 cm, exit code 0
result: [pending]

### 5. Physical robot demo — 物理机器人安全演示
expected: 机器人右臂安全到达 tag 指示目标位置，无碰撞、无过限位，操作员全程可控
result: [pending]

## Summary

total: 5
passed: 0
issues: 0
pending: 5
skipped: 0
blocked: 0

## Gaps
