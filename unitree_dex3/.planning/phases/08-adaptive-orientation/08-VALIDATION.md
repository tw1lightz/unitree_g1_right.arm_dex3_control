---
phase: 08
slug: adaptive-orientation
status: draft
nyquist_compliant: false
wave_0_complete: true
created: 2026-05-19
---

# Phase 08 — Validation Strategy

> Per-phase validation contract for feedback sampling during execution.
> Derived from `08-RESEARCH.md` § Validation Architecture.

---

## Test Infrastructure

| Property | Value |
|----------|-------|
| **Framework** | Custom Python integration harness (`scripts/adaptive_orientation_ab.py`). No `gtest` introduced — the workspace has no pre-existing `ament_add_gtest` infrastructure (verified by grep) and the math under test is small enough that integration coverage is sufficient. |
| **Config file** | none — harness is a self-contained ROS 2 node |
| **Quick run command** | `colcon build --packages-select unitree_g1_dex3_stack --cmake-args -DBUILD_IK_FCL_OMPL_PLANNER=ON` |
| **Full suite command** | `ros2 launch unitree_g1_dex3_stack planner.launch.py adaptive_orientation_enabled:=true &` then `ros2 run unitree_g1_dex3_stack adaptive_orientation_ab.py --ros-args -p adaptive:=true` (and once more with `:=false` for the baseline) |
| **Estimated runtime** | Build ≈ 60 s. Each A/B harness pass ≈ 30 s. Two passes ≈ 60 s. |

---

## Sampling Rate

- **After every task commit:** Run quick run command. Build must rc=0.
- **After Plan 01 (planner code) completes:** Smoke check — start planner, publish one tabletop goal manually, confirm `Adaptive orientation: ...` INFO log line and `/joint_trajectory_targets` publish.
- **After Plan 02 (harness) completes:** Run full A/B suite. Adaptive=true must achieve 100% per D-15.
- **Before `/gsd-verify-work`:** Both A/B runs recorded in `08-VERIFICATION.md`.
- **Max feedback latency:** 90 s (build) + 60 s (A/B suite) = under 3 minutes total.

---

## Per-Task Verification Map

> Filled in by the planner once `08-NN-PLAN.md` files exist. Each task in each plan must map to a row.

| Task ID | Plan | Wave | Requirement | Threat Ref | Secure Behavior | Test Type | Automated Command | File Exists | Status |
|---------|------|------|-------------|------------|-----------------|-----------|-------------------|-------------|--------|
| 08-01-01 | 01 | 1 | ORI-01 (D-05, D-06) | — | N/A | log assertion | colcon build rc=0; planner stdout contains `Right shoulder reference point in 'torso_link': [0.0040, -0.1002, 0.2478]` (±1e-3 m) | ✅ existing planner | ⬜ pending |
| 08-01-02 | 01 | 1 | ORI-01 (D-10) | — | N/A | log assertion | planner stdout contains `adaptive_orientation_enabled = true` on startup | ✅ existing planner | ⬜ pending |
| 08-01-03 | 01 | 1 | ORI-01 (D-01..D-04) | — | N/A | build | colcon rc=0; `nm -C` shows `IKFCLPlannerNode::computeAdaptiveOrientation` symbol | ✅ existing planner | ⬜ pending |
| 08-01-04 | 01 | 1 | ORI-01 (D-09, D-12) | — | N/A | log + behavior | publish goal `(0.40,-0.20,0.00)` to `/goal_pose` with arbitrary orientation; planner emits `Adaptive orientation: target=[0.400,-0.200,0.000] shoulder=[0.004,-0.100,0.248] dir=[…] q=[…]` and publishes `/joint_trajectory_targets` | ✅ existing planner | ⬜ pending |
| 08-01-05 | 01 | 1 | ORI-01 (D-08) | — | reject unsafe input | log + behavior | publish goal `(0.0,-0.10,0.25)` (≤ 0.05 m of shoulder); planner emits `RCLCPP_ERROR ... within 0.05 m of right shoulder ...`; no `/joint_trajectory_targets` publish for that goal | ✅ existing planner | ⬜ pending |
| 08-01-06 | 01 | 1 | ORI-01 (D-11) | — | N/A | log + bit-exact regression | launch with `adaptive_orientation_enabled:=false`; planner stdout shows `false`; publishing a goal with fixed orientation routes that orientation **unchanged** into TRAC-IK target_frame (verify via DEBUG log of `pose_in_base.orientation` immediately before `target_frame` construction) | ✅ existing planner | ⬜ pending |
| 08-01-07 | 01 | 1 | ORI-01 (D-10) | — | N/A | log assertion | `ros2 launch unitree_g1_dex3_stack planner.launch.py adaptive_orientation_enabled:=false` flips the param at startup; INFO log on planner shows `adaptive_orientation_enabled = false` | ✅ existing planner | ⬜ pending |
| 08-02-01 | 02 | 2 | ORI-01 (D-13, D-14) | — | N/A | exit code | `python3 -m py_compile scripts/adaptive_orientation_ab.py` rc=0 | ✅ to-be-created | ⬜ pending |
| 08-02-02 | 02 | 2 | ORI-01 (D-15) | — | N/A | exit code | with planner running and `:=true`, harness reports 100% pass for the 8-target tabletop set; harness exits 0 | ✅ to-be-created | ⬜ pending |
| 08-02-03 | 02 | 2 | ORI-01 (D-16) | — | N/A | log assertion | harness baseline run with `:=false` reports per-target pass/fail counts (no executor invoked, no robot motion) | ✅ to-be-created | ⬜ pending |

*Status: ⬜ pending · ✅ green · ❌ red · ⚠️ flaky*

---

## Wave 0 Requirements

None. Build infrastructure (`BUILD_IK_FCL_OMPL_PLANNER=ON`), KDL/TRAC-IK/Eigen, rclpy, and ROS 2 message types are all already in place. No fixture scaffolding required.

*Wave 0 status: complete by default — `wave_0_complete: true` in frontmatter.*

---

## Manual-Only Verifications

| Behavior | Requirement | Why Manual | Test Instructions |
|----------|-------------|------------|-------------------|
| Adaptive quaternion is "visually reasonable" for tabletop targets | D-12 | RViz orientation arrow inspection | Launch RViz with TF + planner. Publish each tabletop target. Visually confirm the published trajectory's final TCP `+X` axis points roughly at the goal point. |
| First-goal shoulder logging matches URDF rest position | D-12 | Sanity check on first INFO log line | After planner startup INFO, publish first goal. Confirm `shoulder=[0.0040, -0.1002, 0.2478]` ±1e-3 m. |
| `:=false` reproduces exact pre-Phase-8 behavior | D-11 | Bit-exact regression | With `adaptive_orientation_enabled:=false`, publish a goal with a known fixed orientation (e.g., `keyboard_trigger_node.py`'s hard-coded quaternion). Confirm `pose_in_base.orientation` is bit-exactly equal to the input orientation before `target_frame` construction. |

---

## Validation Sign-Off

- [ ] All tasks have `<acceptance_criteria>` mapped to an automated verify command above
- [ ] Sampling continuity: no 3 consecutive tasks without automated verify (Wave 1 tasks all hit colcon build; Wave 2 tasks hit harness exit codes)
- [ ] Wave 0 covers all MISSING references — N/A (no MISSING in this phase)
- [ ] No watch-mode flags (no `--watch`, no incremental rebuild reliance)
- [ ] Feedback latency < 90 s (build) + 60 s (harness) per task
- [ ] `nyquist_compliant: true` — flip after planner emits final PLAN.md set and plan-checker confirms each task has automated verify

**Approval:** pending — to be flipped to `approved 2026-05-19` once `08-NN-PLAN.md` files pass the plan-checker.
