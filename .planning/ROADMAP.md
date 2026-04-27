# Roadmap: Unitree G1 Right-Arm Safe Reach

**Created:** 2025-04-27
**Milestone:** v1.0 — Safe right-arm reaching
**Phases:** 5
**Requirements:** 9

## Phase 1: Right-Arm-Only Planner

**Goal:** Modify `ik_fcl_ompl_planner` to only plan for the right arm, with correct self-collision checking.

**Requirements:** PLAN-01, PLAN-02, PLAN-04

**UI hint**: no

**Success Criteria:**
1. Planner always uses right arm KDL chain regardless of goal position
2. Left arm chain/IK/FK code removed or disabled
3. `isInCollision()` checks right arm links against ALL other body links (torso, legs, left arm)
4. OMPL bounds correctly set from URDF joint limits for all 7 right arm joints
5. Planner compiles and launches successfully

**Depends on:** —

---

## Phase 2: Path Simplification & Quality

**Goal:** Add OMPL path simplification and improve trajectory output quality.

**Requirements:** PLAN-03

**UI hint**: no

**Success Criteria:**
1. OMPL `PathSimplifier` or `simplify()` called after solving, before trajectory conversion
2. `setStateValidityCheckingResolution()` enabled (0.01-0.05) for collision checking between waypoints
3. Simplified paths have fewer waypoints than raw paths (verified by logging)
4. All simplified paths still collision-free (verified by state validity check)

**Depends on:** Phase 1

---

## Phase 3: Trajectory Smoothing & Validation

**Goal:** Replace fixed time steps with velocity-based time parameterization and add pre-execution validation.

**Requirements:** EXEC-01, EXEC-02

**UI hint**: no

**Success Criteria:**
1. Trajectory time stamps computed from URDF joint velocity limits (not fixed 50ms)
2. Maximum joint velocity between consecutive waypoints does not exceed URDF limit
3. All trajectory joint positions verified within URDF limits before execution
4. Trajectories with limit violations are rejected with clear error log
5. Resulting motion is observably smoother than fixed-time-step version

**Depends on:** Phase 2

---

## Phase 4: Right-Arm-Only Executor

**Goal:** Modify `joint_trajectory_executor` to only send right arm joint commands, coexisting with running mode.

**Requirements:** INTG-02

**UI hint**: no

**Success Criteria:**
1. `LowCmd` only populates motor commands for right arm joint indices (kRightShoulderPitch through kRightWristYaw)
2. All non-right-arm joint command entries left at zero/untouched
3. Hand open/close commands removed (no DEX3 control)
4. Running mode maintains balance while right arm moves
5. No observable interference between arm motion and body stability

**Depends on:** Phase 3

---

## Phase 5: End-to-End Integration

**Goal:** Wire the complete pipeline together and provide launch files for the full system.

**Requirements:** INTG-01, INTG-03

**UI hint**: no

**Success Criteria:**
1. Single launch command starts perception + planning + execution pipeline
2. YOLO detects object → 3D position computed → TF transform to torso_link → planner receives goal → right arm moves to target
3. Full pipeline demonstrated on physical robot
4. README or documentation updated with usage instructions

**Depends on:** Phase 4

---

## Coverage

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

**v1 requirements:** 9 total
**Mapped to phases:** 9
**Unmapped:** 0 ✓

---
*Roadmap created: 2025-04-27*
*Last updated: 2025-04-27 after initial creation*
