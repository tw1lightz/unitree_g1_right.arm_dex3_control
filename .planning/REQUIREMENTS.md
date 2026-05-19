# Requirements: Unitree G1 Right-Arm Safe Reach

**Defined:** 2026-05-15
**Core Value:** The right arm TCP moves safely to the target position without colliding with the robot's own body or the environment, and without exceeding joint limits.

## v1.1 Requirements

Requirements for milestone v1.1. Each maps to roadmap phases.

### 清理（Cleanup）

- [x] **CLEAN-01**: 移除 YOLO 检测相关代码（ultralytics_detector.py、project_to_3d_node 等），从 launch 和依赖中彻底删除

### AprilTag 检测（Detection）

- [x] **TAG-01**: AprilTag 36h11 检测节点，使用 pupil-apriltags 从 D435i color stream 检测并发布 tag 6-DOF 位姿
- [x] **TAG-02**: 可配置 tag→物品 XYZ 偏移，通过 YAML 配置文件设置
- [x] **TAG-03**: 将 tag 位姿从相机帧通过 TF 变换到 torso_link 帧
- [x] **TAG-04**: 检测置信度过滤，拒绝模糊/低质量检测结果

### TCP 修正（TCP Correction）

- [x] **TCP-01**: 将 0.175m TCP offset 加入运动学链末端，IK/OMPL 直接以 TCP 为目标点求解
- [x] **TCP-02**: TCP offset 可配置（ROS 参数），不硬编码

### 自适应位姿（Adaptive Orientation）

- [x] **ORI-01**: 根据目标位置相对肩部的方向，自动计算可行的末端 orientation

### 集成（Integration）

- [ ] **INTG-01**: 新 launch 文件 apriltag_reach.launch.py 替代原 YOLO pipeline
- [ ] **INTG-02**: 端到端验证：AprilTag 检测 → 偏移计算 → TF 变换 → planner → executor 全流程

## Future Requirements

后续按需添加，不在当前 roadmap 中。

### AprilTag 增强

- **TAG-05**: 多 tag 支持（按 ID 选择目标 tag）

### 位姿增强

- **ORI-02**: IK 失败时多候选姿态 fallback
- **ORI-03**: 基于 tag 法线推导接近方向（垂直于表面）

## Out of Scope

| Feature | Reason |
|---------|--------|
| YOLO 检测 | 已验证不可行，彻底移除 |
| Left arm control | 只需右臂 |
| DEX3 hand control | 只做到达，不抓取 |
| Walking/locomotion | 站立模式，running mode 保持平衡 |
| Simulation (Gazebo) | 物理机器人测试 |
| Dynamic obstacles | 静态环境足够 |
| MoveIt 2 | 直接 OMPL+FCL 更简单 |

## Traceability

| Requirement | Phase | Status |
|-------------|-------|--------|
| CLEAN-01 | Phase 6 | Complete |
| TCP-01 | Phase 6 | Complete |
| TCP-02 | Phase 6 | Complete |
| TAG-01 | Phase 7 | Complete |
| TAG-02 | Phase 7 | Complete |
| TAG-03 | Phase 7 | Complete |
| TAG-04 | Phase 7 | Complete |
| ORI-01 | Phase 8 | Complete |
| INTG-01 | Phase 9 | Pending |
| INTG-02 | Phase 9 | Pending |

**Coverage:**
- v1.1 requirements: 10 total
- Mapped to phases: 10
- Unmapped: 0 ✓

---
*Requirements defined: 2026-05-15*
*Last updated: 2026-05-18 — Phase 7 complete (TAG-01, TAG-02, TAG-03, TAG-04 marked complete)*
