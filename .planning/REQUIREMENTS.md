# Requirements: Unitree G1 Right-Arm Safe Reach

**Defined:** 2025-04-27
**Core Value:** The right arm moves safely to the target position without colliding with the robot's own body or the environment, and without exceeding joint limits.

## v1 Requirements

Requirements for initial release. Each maps to roadmap phases.

### Motion Planning

- [ ] **PLAN-01**: Planner only plans for right arm chain (torso_link → right_wrist_yaw_link), left arm logic removed or disabled
- [ ] **PLAN-02**: Self-collision checking verifies right arm links against ALL other body links (torso, legs, left arm), not just arm-internal pairs
- [ ] **PLAN-03**: OMPL path simplification applied after planning to remove unnecessary waypoints
- [ ] **PLAN-04**: Joint limits from URDF enforced as OMPL state space bounds for all right arm joints

### Trajectory Execution

- [ ] **EXEC-01**: Trajectory time parameterization uses velocity-based timing instead of fixed time steps, respecting URDF velocity limits
- [ ] **EXEC-02**: Trajectory validated before execution: all joint positions within URDF limits, no excessive velocity between consecutive waypoints

### Integration

- [ ] **INTG-01**: End-to-end pipeline works: YOLO detection → 3D projection → TF transform (camera → torso_link) → planner goal → trajectory execution on right arm
- [ ] **INTG-02**: Right arm joint commands sent via LowCmd without interfering with official running mode control of other joints
- [ ] **INTG-03**: Launch files provided to start the complete pipeline (perception + planning + execution)

## v2 Requirements

Deferred to future release. Tracked but not in current roadmap.

### Environment Collision

- **ENV-01**: Static table added as FCL box primitive collision object with configurable dimensions
- **ENV-02**: Dynamic obstacle tracking from point cloud data

### Safety Hardening

- **SAFE-01**: Joint state freshness check before planning (reject if >500ms stale)
- **SAFE-02**: Graceful error handling for service unavailable, TF failure, IK failure
- **SAFE-03**: Emergency stop topic to halt arm motion immediately
- **SAFE-04**: Collision padding/margin configurable for FCL checks

### Execution Enhancements

- **EXEC-03**: Runtime velocity scaling parameter to slow down execution
- **EXEC-04**: Multiple OMPL planner algorithms selectable (RRT*, PRM, etc.)

## Out of Scope

| Feature | Reason |
|---------|--------|
| Left arm control | Only right arm needed for this milestone |
| DEX3 hand control | Task is reach only, no grasping |
| Walking/locomotion | Robot stands, official running mode handles balance |
| YOLO model training | Using existing best.pt model |
| Camera calibration | Already done |
| Simulation (Gazebo) | Testing on physical robot only |
| MoveIt 2 integration | Direct OMPL+FCL is simpler and already working |
| Dynamic obstacles | Static environment sufficient for v1 |

## Traceability

| Requirement | Phase | Status |
|-------------|-------|--------|
| PLAN-01 | Phase 1 | Pending |
| PLAN-02 | Phase 1 | Pending |
| PLAN-04 | Phase 1 | Pending |
| PLAN-03 | Phase 2 | Pending |
| EXEC-01 | Phase 3 | Pending |
| EXEC-02 | Phase 3 | Pending |
| INTG-02 | Phase 4 | Pending |
| INTG-01 | Phase 5 | Pending |
| INTG-03 | Phase 5 | Pending |

**Coverage:**
- v1 requirements: 9 total
- Mapped to phases: 9
- Unmapped: 0 ✓

---
*Requirements defined: 2025-04-27*
*Last updated: 2025-04-27 after initial definition*
