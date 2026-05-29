---
phase: 06-yolo-tcp-offset
plan: 06-02
type: summary
status: completed
completed_at: "2026-05-15T18:46:33+08:00"
requirements:
  - TCP-01
artifacts:
  - src/unitree_g1_dex3_stack-main/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0.urdf
  - src/unitree_g1_dex3_stack-main/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf
verification:
  - static_check
  - urdf_xml_parse
---

# Summary: Phase 06 Plan 02 — URDF TCP Link

## Result

Plan 06-02 is delivered. Both URDF files now contain a virtual fixed link `right_tcp_link` extending the right-arm chain 0.175 m along the wrist_yaw X axis. The KDL parser recognises the new tip via `kdl_tree.getChain(base, "right_tcp_link", chain)`; FK and TRAC-IK automatically include the offset.

## Implementation

This plan was executed and squashed into a single phase-wide commit:

- **Phase commit:** `d67ee62` — `feat(phase-6): YOLO cleanup + TCP offset integration`

### Files modified

- `src/unitree_g1_dex3_stack-main/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0.urdf` — default URDF.
- `src/unitree_g1_dex3_stack-main/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf` — planner collision URDF.

### XML inserted in both files

Immediately before `<joint name="right_hand_palm_joint" type="fixed">`:

```xml
<!-- TCP (Tool Center Point) virtual link: 0.175m along wrist_yaw X axis -->
<joint name="right_tcp_joint" type="fixed">
  <origin xyz="0.175 0 0" rpy="0 0 0" />
  <parent link="right_wrist_yaw_link" />
  <child link="right_tcp_link" />
</joint>
<link name="right_tcp_link" />
```

`right_tcp_link` is a sibling of `right_hand_palm_link` (both children of `right_wrist_yaw_link`). It has no `<visual>`, `<collision>`, or `<inertial>` element — it is a pure reference frame.

## Verification

All Plan 06-02 acceptance criteria pass against the post-`d67ee62` working tree:

- `grep -c "right_tcp_joint"` in each URDF: 1 (expect ≥ 1).
- `grep -c "right_tcp_link"` in each URDF: 2 (expect ≥ 2 — one joint child + one link element).
- `grep -A2 "right_tcp_joint"` shows `xyz="0.175 0 0"` in both files.
- `grep -n` line numbers confirm `right_tcp_joint` precedes `right_hand_palm_joint` in both files.
- `python3 -c "import xml.etree.ElementTree as ET; ET.parse('<urdf>')"` succeeds for both files (XML well-formed).
- Downstream `colcon build` of `unitree_g1_dex3_stack` with planner enabled exits 0 — the planner's `getChain(base_link_, "right_tcp_link", kdl_chain_right)` integration relies on this link being parseable.

## Requirements Satisfied

- **TCP-01 (partial):** 0.175 m TCP offset is encoded in the kinematic chain at the URDF level. The IK/FCL planner reads this chain via `kdl_parser`, completing TCP-01 together with Plan 06-03.

## Deviations from Plan

None — plan executed exactly as written.

## Notes

- Per Plan 06-02 Section §2 / RESEARCH §2: KDL fixed joints (`Joint::None`) extend the chain frame without adding a DOF, so `getNrOfJoints()` still returns 7 for the right arm. TRAC-IK consumes the chain directly and inherits the extension.
- `right_tcp_link` has no collision geometry, so `buildCollisionObjects()` in `ik_fcl_ompl_planner.cpp` skips it (the existing `if (!link->collision || !link->collision->geometry) continue;` guard handles this). No changes were needed in the FCL setup code.
- This SUMMARY.md was authored retrospectively at phase close-out time after the `safe_resume_gate` detected that production commits existed but per-plan SUMMARY.md files had never been written.
