# Phase 2: Path Simplification & Quality - Context

**Gathered:** 2026-05-11
**Status:** Ready for planning

<domain>
## Phase Boundary

Add OMPL path simplification and state validity resolution checking to `ik_fcl_ompl_planner.cpp`. Specifically: call `simplifySolution()` (or `PathSimplifier`) after `solve()` to remove unnecessary joint-space detours (e.g., arm going backward before reaching target), uncomment `setStateValidityCheckingResolution(0.01)` to enable inter-waypoint collision checking, and log before/after waypoint counts to verify reduction. The `path.interpolate()` call is kept after simplification.

</domain>

<decisions>
## Implementation Decisions

### Simplification API
- **D-01:** Use a `simplify_method` string ROS 2 parameter (default `"simple"`) to select the simplification approach at runtime:
  - `"simple"` → `ss->simplifySolution(simplify_timeout)` — one call, reuses the existing state checker lambda automatically
  - `"manual"` → manually construct `og::PathSimplifier` with explicit `simplify_max_steps` (default 100) and `simplify_max_empty_steps` (default 50)
- **D-02:** Add a `simplify_timeout` parameter (float, default 0.5s) — independent of `planning_timeout`. Both `simplifySolution()` and the manual `PathSimplifier` path respect this limit.

### Execution Flow
- **D-03:** Keep `path.interpolate()` after simplification. New flow:
  ```
  solve() → simplifySolution()/PathSimplifier → [log waypoint count] → path.interpolate() → convert to JointTrajectory → publish
  ```
  Waypoint count is logged **before** `interpolate()` so the "Simplified: N → M" comparison reflects true simplification, not interpolation densification.

### State Validity Resolution
- **D-04:** Uncomment the existing commented line `ss->getSpaceInformation()->setStateValidityCheckingResolution(0.01)` — use the literal `0.01`. No new parameter needed.

### Logging
- **D-05:** Always log `"Simplified: N → M waypoints (-X%)"` as `RCLCPP_INFO`. No special handling when N==M (0% reduction is normal for already-optimal short paths).

### Path Variability
- **D-06:** Accept that RRTConnect produces different paths each run — this is inherent to sampling-based planning. Simplification (shortcutting) is expected to eliminate backward-then-forward joint-space detours. No fixed random seed added.

</decisions>

<canonical_refs>
## Canonical References

**Downstream agents MUST read these before planning or implementing.**

### Source Code
- `src/unitree_g1_dex3_stack-main/src/ik_fcl_ompl_planner.cpp` — File being modified. Key lines: 543 (`SimpleSetup` construction), 545 (state validity checker lambda), 578 (commented-out resolution line), 580–614 (solve/interpolate/publish block)

### Robot Model
- `src/unitree_g1_dex3_stack-main/robots/g1_description/g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf` — Selected URDF (from Phase 1 D-04)

### Supporting Files
- `src/unitree_g1_dex3_stack-main/include/g1_dex3_joint_defs.hpp` — Joint enums and name→index maps

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- `ss` (`og::SimpleSetup`, line 543) — already has state checker set; `ss->simplifySolution()` reuses it automatically with no extra wiring
- `setStateValidityCheckingResolution(0.01)` — already written at line 578, just commented out; uncomment to enable
- `path.interpolate()` at line 588 — kept in place, moved to after simplification call

### Established Patterns
- RCLCPP logging macros (INFO, WARN, ERROR) — D-05: use `RCLCPP_INFO` for simplification result log
- ROS 2 `declare_parameter` / `get_parameter` — all new params (`simplify_method`, `simplify_timeout`, `simplify_max_steps`, `simplify_max_empty_steps`) follow this pattern

### Integration Points
- Simplification inserted between `ss->solve()` (line 586) and `path.interpolate()` (line 588) — no changes to upstream (goal/start setup) or downstream (trajectory conversion/publish)
- `planning_timeout_` already used for `ss->solve()` — `simplify_timeout` is a separate parameter, not derived from it

</code_context>

<specifics>
## Specific Ideas

- User observed "arm goes backward before reaching target" during Phase 1 testing — this is the primary motivation for shortcutting. `simplifySolution()` addresses it via random shortcut sampling in joint space.

</specifics>

<deferred>
## Deferred Ideas

- **Fixed random seed (`planner_seed` param)** — would make paths reproducible for debugging. Decided not needed: simplification quality is sufficient, path variability is acceptable.
- **`simplify_method = "none"` option** — would allow disabling simplification entirely for regression comparison. Not requested; can be added in a quick task if needed.

</deferred>

---

*Phase: 02-path-simplification-quality*
*Context gathered: 2026-05-11*
