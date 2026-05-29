# Research Summary: v1.1 AprilTag 定位 + TCP 修正

## Stack Additions

- `pupil-apriltags==1.0.4.post11` — 轻量 AprilTag 检测，无外部依赖
- `scipy>=1.10.0` — Rotation 数学 + Slerp 插值
- 不需要 apriltag_ros、pytracik 等重量级包

## Feature Table Stakes

- AprilTag 36h11 检测 + 6-DOF 位姿估计 + 可配置偏移
- TCP offset 反向应用到 IK 目标（~3 行 C++ 修改）
- 自适应 orientation：基于目标位置计算可行姿态

## Architecture Key Points

- **C++ planner 几乎不改** — TCP offset 可在上游 Python 节点处理，或在 planner 中加 3 行反向偏移
- 新增 Python 节点：apriltag_detector + goal_composer（偏移 + 自适应姿态）
- 数据流：D435i → apriltag_detector → goal_composer (offset + orientation) → /goal_pose → planner (不变) → executor (不变)
- 新 launch 文件替代原 YOLO 相关启动

## Watch Out For

1. **TCP offset 帧错误**（最高风险）— offset 必须在 wrist_yaw_link 局部帧中反向应用
2. **RealSense IR 干扰** — D435i 深度投射器可能干扰 AprilTag 检测，考虑关闭或用 color-only
3. **现有 planner 可能有冗余 IK 调用 bug**（lines ~450-470）— 需先修复再加 TCP offset
4. **自适应 orientation 在肩部正上方会遇到奇异点** — 需要 fallback 策略
5. **Tag 尺寸 vs 检测距离** — tag 太小远距离检测不到，需匹配实际使用距离

## Suggested Build Order

1. 移除 YOLO 代码（清理）
2. AprilTag 检测节点（独立可测试）
3. TCP offset 集成到 planner
4. 自适应 orientation 逻辑
5. 端到端集成 + 新 launch 文件
