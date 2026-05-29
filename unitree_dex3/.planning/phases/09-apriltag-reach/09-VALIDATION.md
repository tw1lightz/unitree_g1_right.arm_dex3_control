---
phase: 9
slug: apriltag-reach
status: draft
nyquist_compliant: true
wave_0_complete: true
created: 2026-05-19
---

# Phase 9 — Validation Strategy

> Per-phase validation contract for feedback sampling during execution.

---

## Test Infrastructure

| Property | Value |
|----------|-------|
| **Framework** | None (UAT script-based) |
| **Config file** | none — Wave 0 installs |
| **Quick run command** | `ros2 launch unitree_g1_dex3_stack apriltag_reach.launch.py --print` (launch composition check) |
| **Full suite command** | `ros2 run unitree_g1_dex3_stack apriltag_reach_uat.py` (hardware UAT) |
| **Estimated runtime** | ~60 seconds (launch check) / ~180 seconds (full UAT) |

---

## Sampling Rate

- **After every task commit:** `colcon build` compilation check + manual code review
- **After every plan wave:** Launch file syntax verification (`ros2 launch ... --print`), bridge standalone test with `ros2 topic pub /apriltag/target_pose`
- **Before `/gsd:verify-work`:** Full UAT (4/4 points PASS, exit 0)
- **Max feedback latency:** 180 seconds

---

## Per-Task Verification Map

| Task ID | Plan | Wave | Requirement | Threat Ref | Secure Behavior | Test Type | Automated Command | File Exists | Status |
|---------|------|------|-------------|------------|-----------------|-----------|-------------------|-------------|--------|
| 09-01-01 | 01 | 1 | INTG-01 | T-09-01 | Bridge pre-checks reach distance before publishing | Manual (topic) | `ros2 topic echo /goal_pose` after G press | ❌ W0 | ⬜ pending |
| 09-01-02 | 01 | 1 | INTG-01 | T-09-02 | Stale threshold rejects old detections | Manual (timeout) | Remove tag, wait >1s, press G → verify WARN | ❌ W0 | ⬜ pending |
| 09-01-03 | 01 | 1 | INTG-01 | T-09-03 | `waiting_for_completion` rejects concurrent G | Manual (double-press) | Press G twice rapidly → verify rejection WARN | ❌ W0 | ⬜ pending |
| 09-01-04 | 01 | 1 | INTG-02 | T-09-04 | Physical safety: manual trigger, not automatic | UAT (harness) | `ros2 run unitree_g1_dex3_stack apriltag_reach_uat.py` exits 0 | ❌ W0 | ⬜ pending |
| 09-03-01 | 03 | 1 | INTG-02 | T-09-05 | TCP error ≤ 3 cm for all 4 targets | UAT (harness FK) | Harness FK comparison vs expected per point | ❌ W0 | ⬜ pending |

*Status: ⬜ pending · ✅ green · ❌ red · ⚠️ flaky*

---

## Wave 0 Requirements

- [ ] `scripts/apriltag_goal_bridge.py` — bridge node (INTG-01)
- [ ] `launch/apriltag_reach.launch.py` — end-to-end launch (INTG-01)
- [ ] `scripts/apriltag_reach_uat.py` — UAT harness (INTG-02)
- [ ] `CMakeLists.txt` — install entries updated: bridge + UAT added; keyboard_trigger removed
- [ ] `README.md` — three-entry launch table, G trigger key, UAT command

---

## Manual-Only Verifications

| Behavior | Requirement | Why Manual | Test Instructions |
|----------|-------------|------------|-------------------|
| Single launch starts all components | INTG-01 | Hardware dependencies (RealSense, robot) | `ros2 launch unitree_g1_dex3_stack apriltag_reach.launch.py` → verify all nodes running |
| Bridge G trigger publishes `/goal_pose` | INTG-01 | Requires physical robot + tag setup | Press G with visible tag → verify `/goal_pose` topic |
| End-to-end TCP reach 4/4 targets | INTG-02 | Physical robot UAT | Run `apriltag_reach_uat.py`, move tag to 4 tabletop positions |
| Keyboard raw mode in launch | INTG-01 | Terminal environment dependent | Verify G keypress works in launch context |
| Stale/empty cache guard | INTG-01 | Requires timed tag manipulation | Cover/remove tag → press G → verify rejection WARN |

---

## Validation Sign-Off

- [ ] All tasks have `<acceptance_criteria>` or Wave 0 dependencies
- [ ] Sampling continuity: build check after every task commit
- [ ] Wave 0 covers all MISSING references
- [ ] No watch-mode flags
- [ ] Feedback latency < 240s (full UAT + robot setup)
- [ ] `nyquist_compliant: true` set in frontmatter

**Approval:** pending
