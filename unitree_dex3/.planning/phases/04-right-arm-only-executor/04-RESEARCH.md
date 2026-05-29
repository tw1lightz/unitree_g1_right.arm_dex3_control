# Phase 4: Right-Arm-Only Executor — Research

**Researched:** 2026-05-13
**Question answered:** "What do I need to know to PLAN this phase well?"
**Sources:**
- `src/unitree_g1_dex3_stack-main/src/joint_trajectory_executor.cpp` (current 368-line file)
- `src/unitree_g1_dex3_stack-main/include/g1_dex3_joint_defs.hpp` (`JointIndex` enum + name map)
- `.planning/phases/01-right-arm-only-planner/01-09-SUMMARY.md` / `01-11-SUMMARY.md` / `01-12-SUMMARY.md` (master-switch + timing + stale-state lessons)
- `.planning/phases/04-right-arm-only-executor/04-CONTEXT.md` (locked decisions D-01 … D-06)
- `/home/unitree/Desktop/xr_teleoperate/free_arm_demo.py` (Unitree-supplied reference for arm_sdk authority handover)

---

## 1. Current Executor Anatomy (mapped to CONTEXT.md line refs)

| Region | Lines | Purpose | Phase 4 disposition |
|--------|-------|---------|---------------------|
| Hand publishers (`left_hand_pub_`, `right_hand_pub_`) declared | 134-135 | DEX3 hand control | **Kept inert** per D-01 |
| Hand publishers `create_publisher<Bool>` in ctor | 58-59 | DEX3 hand control | **Kept inert** per D-01 |
| Empty-`joint_names` ERROR + return | 154-157 | Existing guard | **Kept** per D-04 |
| `is_left_hand` substring detection (selects `hand_cmd_pub`) | 158-168 | Side detection | **Kept inert** per D-01 |
| `standing_pose` snapshot from `latest_joint_positions_` | 174-177 | Plan 01-09 ramp target | **Kept; full 29-slot snapshot retained** |
| Hand-open `publish(hand_cmd false)` | 181-182 | DEX3 open command | **DELETE** (the line `hand_cmd_pub->publish(hand_cmd);` only) |
| `sleep_for(1s)` after hand-open | 184 | Settling delay | **Kept inert** per D-01 (now an unconditional 1 s settle) |
| Waypoint loop (writes 28 joints + `kNotUsedJoint`) | 193-232 | Trajectory tracking | **Narrow writes to 7 right-arm joints + `kNotUsedJoint`** |
| `joint_limits_.at(target_joint_name)` clamp inside loop | 219-220 | Defense-in-depth | **Kept**, runs only on right-arm names |
| `trajectory_endpoint` extraction from `msg->points.back()` | 241-260 | Plan 01-12 baseline | **Kept; full 29-slot vector retained** so non-right-arm slots can still be sourced from `standing_pose` |
| Hand-close `publish(hand_cmd true)` | 269-270 | DEX3 close command | **DELETE** (the line `hand_cmd_pub->publish(hand_cmd);` only — the assignment `hand_cmd.data = true;` is harmless dead code; can stay or be removed, planner picks) |
| Hold loop (1 s @ 250 Hz, master=0.5) | 272-294 | Plan 01-11 hold | **Narrow writes to 7 right-arm joints + master**, preserve timing |
| Exit ramp (3 s @ 250 Hz, master 0.5→0, q from end-point→standing) | 307-340 | Plan 01-09/01-11/01-12 | **Narrow writes to 7 right-arm joints + master**, preserve timing/master curve/q interpolation |
| Destructor `kNotUsedJoint.q = 0.0` final publish | 127-129 | Plan 01-04/01-06 release | **Kept** — already publishes only `kNotUsedJoint`, all other slots default-constructed; this is exactly the pattern Phase 4 is now adopting in the active loops |

**Key invariant**: the destructor at line 127-129 is already a precedent for "publish a `LowCmd` with only `kNotUsedJoint.q` set, all 28 motor slots default-constructed". It has been running for 1+ year of testing without observable harm to body control. This is empirical evidence for the "leave default" path of D-06 (see §3 below).

---

## 2. The 7 Right-Arm Joints (canonical set)

From `g1_dex3_joint_defs.hpp` lines 43-50 and 101-107:

| Enum (idx) | Joint name (string, used in trajectory `joint_names`) |
|------------|-------------------------------------------------------|
| `kRightShoulderPitch` (22) | `right_shoulder_pitch_joint` |
| `kRightShoulderRoll` (23) | `right_shoulder_roll_joint` |
| `kRightShoulderYaw` (24) | `right_shoulder_yaw_joint` |
| `kRightElbow` (25) | `right_elbow_joint` |
| `kRightWristRoll` (26) | `right_wrist_roll_joint` |
| `kRightWristPitch` (27) | `right_wrist_pitch_joint` |
| `kRightWristYaw` (28) | `right_wrist_yaw_joint` |

**Master switch slot:** `kNotUsedJoint` (idx 29). Not in `joint_name_to_index`.

The 21 non-right-arm body joints (idx 0-21) are: 6 left leg + 6 right leg + 3 waist + 7 left arm.

---

## 3. arm_sdk Master-Switch Semantics — D-06 Investigation

### 3.1 What we already know from prior phases

From `01-09-SUMMARY.md` (the diagnostic that closed the compound-bug ladder):

> "arm_sdk's master switch is **binary (or near-binary)**, so [Plan 01-08's] kp fade had no influence on body authority during the ramp; body snapped at the threshold crossing"

So `kNotUsedJoint.q` is **not** a continuous blend coefficient between motion-sdk and body-controller commands. It is closer to a discrete handover: above some threshold, motion-sdk wins; below, body-controller wins. The exact threshold is not documented in our project notes — Plan 01-09's reframe ("drive arm to standing yourself before handover") deliberately makes the answer not matter.

### 3.2 Unitree reference: `xr_teleoperate/free_arm_demo.py` (canonical pattern)

`free_arm_demo.py` is Unitree-supplied teleop reference code that shares the same `/arm_sdk` topic with the body controller. Its strategy when running coexistent with body-controller-driven standing:

```python
# free_arm_demo.py L159-185 — populates motor_cmd[] for ALL 29 joints every frame
for jid in G1_29_JointIndex:
    msg.motor_cmd[jid].mode = 1
    if is_right_arm_motor(jid.value):
        msg.motor_cmd[jid].kp = 0.0     # right arm: no position tracking → free to drag
        msg.motor_cmd[jid].kd = 0.5
    else:
        msg.motor_cmd[jid].kp = 80…300  # body + left arm: HIGH gains → locked at current q
        msg.motor_cmd[jid].kd = 3.0
    msg.motor_cmd[jid].q  = current_measured_q[jid]

msg.motor_cmd[kNotUsedJoint0].q = 1.0  # full motion-sdk authority
```

**Implication:** The Unitree reference does **not** trust "leave default" for non-right-arm slots. It explicitly **locks** them with high kp/kd at their current measured q. The author's mental model is "once I set master=1, motion-sdk owns *every* slot in `motor_cmd[]`; body controller no longer drives them".

This is the **opposite** assumption from CONTEXT.md's domain statement and from the existing `joint_trajectory_executor.cpp`.

### 3.3 Reconciling the two pieces of evidence

| Evidence | Implies |
|----------|---------|
| `joint_trajectory_executor.cpp` waypoint loop has been writing `q=latest_joint_positions_[idx], kp=60, kd=1.5, mode=1` for **all 28** body joints since Plan 01-10, master=0.5 → trajectory tracking works | Either (a) `master=0.5` is below the binary threshold so body-controller still drives non-right-arm joints, or (b) `master=0.5` activates motion-sdk but our `kp=60` lock-at-current is close enough to the body-controller's standing setpoint that the arm doesn't visibly drift |
| Plan 01-09 reframe attribute "near-binary master switch" | Master is closer to a hard switch than a blend |
| free_arm_demo.py uses `master=1.0` and explicitly locks body with `kp=300` | Above the threshold, motion-sdk *is* in charge of every populated slot; body controller is suppressed |
| Destructor publishes `kNotUsedJoint.q = 0.0` with all motor_cmd slots default-zero, no harm observed | When `master=0.0`, the populated/zero state of `motor_cmd[]` is irrelevant — body controller is in charge |

The simplest model consistent with all four observations:

> **`kNotUsedJoint.q` selects the source of motor commands.** When `q ≥ threshold` (somewhere between 0 and 1, likely around 0.5 based on Plan 01-09's diagnosis), motion-sdk's published `motor_cmd[]` array is forwarded to the motor controller. When `q < threshold`, body controller's commands are forwarded. There is **no continuous blend** of the two sides' commands.

Under this model, **what `motor_cmd[]` contains for non-right-arm joints when master ≥ threshold matters**:

- **Default-constructed** (`q=0, mode=0, kp=0, kd=0, dq=0, tau=0`): if `mode=0` is interpreted as "disabled / no torque", motors go limp → **catastrophic** (left arm sags, robot may lose balance even in standing mode). If `mode=0` means "track q=0 with kp=0", torque output is 0 (because servo error × kp = 0), so motors are limp again — same outcome. **HIGH RISK**.
- **`mode=0, q=latest_joint_positions_[idx], kp=0, kd=0`** (explicit "no opinion" pattern): same servo math gives 0 torque. Same risk as default-constructed, but at least makes the intent legible. **HIGH RISK**.
- **`mode=1, q=latest_joint_positions_[idx], kp=60, kd=1.5`** (the *current* code, kept-as-is): motion-sdk locks each non-right-arm joint at its standing-pose snapshot. **SAFE under coexistence with body controller's standing mode** (because both sources agree on the setpoint; small disagreement just produces a small torque). **UNSAFE under coexistence with running mode** (running mode wants legs/waist to move actively for balance, but motion-sdk is locking them at the standing snapshot — fights the body controller).

### 3.4 D-06 finding — three options for the planner to choose from

| Option | Non-right-arm `motor_cmd[idx]` payload | Coexists with standing mode | Coexists with running mode | Matches CONTEXT.md domain text | Source / precedent |
|--------|----------------------------------------|------------------------------|-----------------------------|---------------------------------|--------------------|
| **A. Keep current "lock 28 with kp=60"** | `mode=1, q=latest, kp=60, kd=1.5, dq=0, tau=0` | ✅ (proven by Phase 1 testing) | ❌ (fights running mode) | ❌ (CONTEXT says "left at zero/untouched") | Existing `joint_trajectory_executor.cpp` Plans 01-10..01-12 |
| **B. Default-zero (literal CONTEXT)** | nothing written; default-constructed by ROS2 | ⚠️ unknown — never tested under master=0.5 | ⚠️ unknown — likely catastrophic if firmware passes through `mode=0,kp=0` to motor | ✅ | CONTEXT.md `<domain>` paragraph; destructor at L127-129 (but destructor only fires after master=0) |
| **C. Explicit "no-op" — `mode=0, q=latest, kp=0, kd=0`** | each non-right-arm slot explicitly written | ⚠️ unknown — depends on firmware mode-0 semantics | ⚠️ unknown | ⚠️ partially (writes "zero-effort" not literal-zero) | None in our codebase |

> **None of these three options has been bench-verified against the running mode**, because Phase 1-3 testing was always against a robot held in standing mode by the body controller, not under running mode.

### 3.5 Recommended path for the planner

**Keep Option A** (the current "lock 28 joints" pattern), with a documented caveat:

1. The phase-4 ROADMAP success criterion 4 ("Running mode maintains balance while right arm moves") is **not verifiable from a code change alone**. It will require an on-robot bench test under running mode.
2. CONTEXT.md's `<domain>` text saying "non-right-arm slots left at default" is **inconsistent with the safest known implementation** of arm_sdk coexistence (free_arm_demo.py). The planner agent must surface this contradiction back to the user as a blocking question before changing the publish-loop strategy.
3. If the user **insists** on literal "leave default" (Option B) per CONTEXT.md `<domain>`, plan it as a separate sub-plan with explicit on-robot bench test before / after, AND compare against running mode coexistence. Do **not** roll it in with the hand-publish removal in the same plan.

**Suggested user-facing question** (the planner should propose this; orchestrator surfaces it):

> CONTEXT.md `<domain>` says non-right-arm `motor_cmd[]` slots should be "left at default (zero/untouched)". The Unitree reference `free_arm_demo.py` does the **opposite** — it explicitly locks all non-right-arm joints with high kp/kd at their measured q. Existing `joint_trajectory_executor.cpp` (Plan 01-10) follows the Unitree-reference pattern with kp=60. Phase 4 success criterion 4 (running mode maintains balance) has not been bench-verified for any of the three strategies. Which policy do you want for Phase 4?
>
> A) Keep current "lock all 28 joints with kp=60 at latest_q" (lowest deviation from working code).
> B) Truly leave non-right-arm slots default-constructed (matches CONTEXT.md text; UNTESTED under both standing and running modes).
> C) Explicit "no-opinion" pattern (`mode=0, kp=0, kd=0, q=latest`; UNTESTED).

The planner's plan should be parameterized so a single `enum NonRightArmPolicy { kLockAtLatest, kDefault, kNoOpinion };` (file-private) selects the strategy, defaulting to **A** for safety. This keeps the diff minimal and the policy reversible.

---

## 4. Trajectory Validation Strategy (D-03, D-04, D-05)

### 4.1 Two-stage validation (CONTEXT.md D-03)

**Stage 1 — Foreign-joint WARN + strip:**
```
right-arm name set := {
  "right_shoulder_pitch_joint",
  "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint",
  "right_elbow_joint",
  "right_wrist_roll_joint",
  "right_wrist_pitch_joint",
  "right_wrist_yaw_joint"
}
foreign := {n in msg->joint_names where n not in right-arm name set}
if foreign not empty:
  RCLCPP_WARN(...) listing foreign names
  filtered_indices := [i for i, n in enumerate(msg->joint_names) if n in right-arm name set]
  filtered_names   := [msg->joint_names[i] for i in filtered_indices]
else:
  filtered_indices := [0..len(joint_names))
  filtered_names   := msg->joint_names
```

**Stage 2 — Completeness ERROR:**
```
present := set(filtered_names)
missing := right-arm name set - present
if missing not empty:
  RCLCPP_ERROR(...) listing missing names
  return  # do not publish any LowCmd this trajectory
```

### 4.2 Read-only message contract (D-05)

`filtered_indices` and `filtered_names` are **local working copies**. The original `msg` (a `JointTrajectory::SharedPtr`) is read-only as in current code. Per-point payload (`positions`, `velocities`, `accelerations`) is accessed by index lookup `point.positions[filtered_indices[j]]` rather than by mutating `msg`.

This handles the parallel-array semantics correctly: stripping foreign joint at column `k` means index `k` is simply not in `filtered_indices`; the original array is never modified.

### 4.3 Order of operations — must run BEFORE `master=0.5`

If validation runs *after* the executor has published `master=0.5` (or written the `is_left_hand` detection block), an ERROR-and-return leaves the master switch in an unstable state — body controller may snap. Therefore validation **must** run as the very first work in `trajectoryCallback`, immediately after the existing empty-`joint_names` guard at L154-157, and **before** the `is_left_hand` detection at L158-168 (which is being kept inert per D-01 anyway).

The completeness ERROR at stage 2 may be reached without ever publishing a master change — the function simply returns with the previous master value (whatever it was before the trajectory arrived) still in effect. This is safe because:
- If the previous trajectory completed normally, its ramp ended at `master=0.0` (body controller in charge). Returning preserves that — no impulse.
- If the previous trajectory was interrupted, its break-out path also runs the ramp, ending at `master=0.0`. Same outcome.
- At fresh node start, no master has been published yet — body controller has been in charge all along. Returning preserves that.

### 4.4 Existing empty-`joint_names` guard interaction (D-04)

The L154-157 guard fires before the new validation. After D-03 stage 2 reduces "incomplete right-arm" to ERROR+return, the empty-`joint_names` case is naturally subsumed (an empty list will fail the "all 7 right-arm joints present" check). Per D-04, the L154-157 guard **stays** as a fast-path early return with its own ERROR message — the new D-03 guards are a stricter superset.

---

## 5. Implementation Strategy for Narrowing the 3 Publish Loops (Claude's Discretion)

### 5.1 Three options for the right-arm index set (CONTEXT.md `Implementation strategy for narrowing the 3 publish loops`)

| Option | Where it lives | Pros | Cons |
|--------|----------------|------|------|
| **a. `joint_name_to_index` lookups by 7 right-arm names** | Implicit (use existing map) | Zero new code in header; reuses existing string→index lookup | 7 string lookups per frame × 250 Hz = 1750 string-map lookups/s. Minor perf hit; readability is fine. Couples publish loops to string-name knowledge. |
| **b. Hardcoded `JointIndex` enum values inline** | Inline `for (auto idx : {kRightShoulderPitch, kRightShoulderRoll, …})` | No new symbols; smallest diff. | Repeated 3× across waypoint/hold/ramp loops; risk of drift between copies. |
| **c. File-private `static constexpr std::array<JointIndex, 7> kRightArmJointIndices`** | `joint_trajectory_executor.cpp` (anonymous namespace or `static`) | Single source of truth; cheap; type-safe; iterable with range-for. | One new symbol in the file. |
| **d. Header constant `constexpr std::array<JointIndex, 7> kRightArmJointIndices` in `g1_dex3_joint_defs.hpp`** | Public header | Reusable from planner / future dex3 code. | Couples header to "right-arm-only" semantics that the planner already encodes elsewhere; widens header API. Header ownership unclear (do other consumers want this?). |

**Recommendation: Option c** — `static constexpr std::array<JointIndex, 7> kRightArmJointIndices` at file scope (anonymous namespace) inside `joint_trajectory_executor.cpp`. Plus a parallel `static const std::array<std::string, 7> kRightArmJointNames` (note: `std::string` cannot be `constexpr` until C++20 with `consteval`; use `static const std::array<std::string_view, 7>` for `constexpr`-compatibility, or `static const std::array<std::string, 7>` with runtime construction — either works for our use). The names array is the input to validation (§4.1); the indices array is the input to publish-loop narrowing (this section).

These two arrays must stay in sync — the planner should write them adjacent to each other in the file, with a comment to that effect.

### 5.2 Pattern for the narrowed publish loop (illustrative, applies to waypoint / hold / ramp)

Conceptual change inside each of the three loops, **for non-right-arm joints**:

```text
for each idx in joint_name_to_index:
  if NonRightArmPolicy = LockAtLatest (Option A from §3.5):
    cmd_msg.motor_cmd[idx] = current Plan 01-10/01-11/01-12 payload  (UNCHANGED)
  else if NonRightArmPolicy = Default (Option B):
    skip — leave default-constructed
  else if NonRightArmPolicy = NoOpinion (Option C):
    cmd_msg.motor_cmd[idx] = {mode=0, q=latest_joint_positions_[idx], kp=0, kd=0, dq=0, tau=0}
```

Then a separate loop **for right-arm joints only**:
```text
for each idx in kRightArmJointIndices:
  cmd_msg.motor_cmd[idx] = right-arm payload (mode=1, q=trajectory_or_endpoint_or_interp, kp=60, kd=1.5, dq=0, tau=0)
cmd_msg.motor_cmd[kNotUsedJoint].q = current master value (0.5 or interp)
```

The right-arm payload differs by loop:
- **Waypoint loop:** `q = clamp(point.positions[trajectory_column_for_this_idx], joint_limits)` overwrites a per-frame baseline. The baseline can be `q=latest_joint_positions_[idx]` (legacy) or the right-arm component of `standing_pose` (more deterministic). Recommend **`q=standing_pose[idx]`** as baseline to remove the dependence on `latest_joint_positions_` freshness during the long callback (Plan 01-12 lesson reapplied).
- **Hold loop:** `q = trajectory_endpoint[idx]` (unchanged).
- **Ramp loop:** `q = (1-t) * ramp_start_positions[idx] + t * standing_pose[idx]` (unchanged).

**Important:** keep `standing_pose`, `ramp_start_positions`, `trajectory_endpoint` as **29-slot vectors** (as today). Even though the publish loop only reads the 7 right-arm indices from them, keeping the data structures full-width avoids subtle bugs where some other loop later expects 29 slots.

---

## 6. Hand Publish Removal — Exact Diff (D-01)

Per D-01 "minimal removal":
- **Delete only line 182**: `hand_cmd_pub->publish(hand_cmd); // Open hand command`
- **Delete only line 270**: `hand_cmd_pub->publish(hand_cmd);`

Keep everything else (publishers, members, ctor `create_publisher`, `is_left_hand` detection, `sleep_for(1s)`, the `hand_cmd.data = false/true` assignments).

After removal, `hand_cmd_pub` becomes a local variable that is assigned but never used. **Do NOT add `[[maybe_unused]]`** per D-02. If GCC/clang warns, accept the warning.

The `hand_cmd.data = false;` and `hand_cmd.data = true;` assignment statements are technically dead too. Planner's choice whether to keep or delete them — keeping makes future re-enable a one-line change (just re-add the publish call); deleting cleans the diff. Recommend **keep** for future-resilience.

---

## 7. Test / Verification Strategy

### 7.1 What can be verified offline (build / unit / static)

| Check | Command | Why |
|-------|---------|-----|
| Build | `colcon build --packages-select unitree_g1_dex3_stack` | Catches syntax / missing-include / `joint_name_to_index` typo errors |
| `grep -c 'hand_cmd_pub->publish'` in modified file | Should be `0` | D-01 minimal-removal verification |
| `grep -c 'kRightArmJointIndices'` in modified file | Should be ≥ 4 (definition + 3 publish loops) | Index-array adoption sanity |
| `grep -c 'motor_cmd\[idx\].mode = 1'` | Per-loop count depends on chosen NonRightArmPolicy | A: 3 (full-fill, like today); B: 0; C: 0 (mode=0 instead) |
| `wc -l joint_trajectory_executor.cpp` | Within ±50 of 368 | Surgery, not rewrite |
| Foreign-joint stripping unit-test (planner agent's choice if it's worth a `gtest`-style harness) | n/a in current build | The trajectoryCallback is `private`; testing in isolation requires friending or refactoring. Probably **not worth** for this phase given the on-robot bench test will exercise all paths. |

### 7.2 What requires on-robot verification

ROADMAP success criteria 1-3 are mostly verifiable by reading the diff or by `grep`. Criteria 4-5 ("running mode maintains balance", "no observable interference between arm motion and body stability") **cannot be verified offline**. They require:

1. **Standing-mode baseline (regression):** confirm Phase 1-3 trajectories still execute smoothly under standing mode — the user's existing test pattern from `/home/unitree/Desktop/test_log`.
2. **Running-mode test (new):** robot in running mode (locomotion controller active). Send a single right-arm trajectory. Observe:
   - Body remains stable (no balance perturbation correlated with arm motion).
   - Arm reaches goal and returns to standing-arm pose smoothly.
   - 1 s post-trajectory hold and 3 s ramp behave as in standing mode.
3. **Foreign-joint warn-and-strip test:** publish a synthetic 14-joint trajectory (both arms, e.g. Phase 1's pre-fix planner output). Expect: `RCLCPP_WARN` listing the 7 left-arm names; right-arm portion executes normally.
4. **Incomplete-trajectory error test:** publish a 5-joint right-arm trajectory (omit roll + yaw of wrist). Expect: `RCLCPP_ERROR` listing the 2 missing names; no `LowCmd` published; body unaffected.

Bench tests 3 and 4 are nice-to-have. Tests 1 and 2 are essential.

---

## 8. Risks / Landmines

1. **`joint_name_to_index.at()` throw on unknown joint** (line 217): with D-03 stage-1 stripping, this `.at()` becomes unreachable for foreign joints. But: if a future change drops the validation, an unknown name in the trajectory would throw `std::out_of_range` mid-publish-loop, which crashes the executor — which destructs while `master=0.5` if the throw is between `master=0.5` and the ramp-end `master=0.0`. The destructor publishes `master=0.0` (line 128-129) which is the safe path, but the timing is poor (instantaneous master drop). **Mitigation:** the validation in §4 must run before the master is changed, and the `.at()` call must be inside `try`/`catch` if defense-in-depth is desired. Recommend: leave `.at()` as-is, rely on D-03 validation as the contract.

2. **`is_left_hand` substring match on stripped right-arm names**: with D-03 stripping, the working trajectory contains only `right_*_joint` names. The L158-168 detection always picks `right_hand_pub_`. Consistent with existing behavior post-Phase 1.

3. **`latest_joint_positions_` is stale during long callback** (Plan 01-12 lesson): unchanged in Phase 4. We continue using `standing_pose` and `trajectory_endpoint` snapshots. New consideration: if NonRightArmPolicy = NoOpinion or LockAtLatest, the per-frame value `latest_joint_positions_[idx]` for non-right-arm joints is also stale — but this matches what the current code has been doing for 1+ year. No new risk.

4. **`standing_pose` empty if `latest_joint_positions_` was empty at callback entry** (line 175-177): existing fallback handling is "if `latest_joint_positions_` is empty, leave `standing_pose` empty" → in the ramp loop, `standing_pose.size() > idx` becomes false → fallback to `latest_joint_positions_[idx]` → also empty → fallback to `q=0`. Phase 4 reuses this chain unchanged. The same fallback path applies to the new "non-right-arm publish strategy" if NonRightArmPolicy uses `latest_joint_positions_`.

5. **Master-switch under default-zero non-right-arm slots (D-06)**: covered in §3 above. Surface as a blocking question; default to Option A.

6. **Compiler warning on unused `hand_cmd_pub`**: per D-02 do not suppress. If it bothers CI, future cleanup phase removes the inert hand block entirely.

7. **`joint_name_to_index` ordering vs `std::map` iteration order**: the existing waypoint loop iterates `for (const auto& pair : joint_name_to_index)`. `std::map<std::string, JointIndex>` iterates alphabetically by string key, **not** by index value. This is fine for the existing fill-then-overwrite pattern, but if Phase 4 splits into "fill non-right-arm" + "fill right-arm by `kRightArmJointIndices`", make sure ordering does not matter (which it doesn't — each `motor_cmd[idx]` slot is independent).

---

## 9. Recommended Phase 4 Plan Decomposition (suggestions for the planner; final shape is the planner's call)

**Wave 1 (sequential prerequisite):**
- **Plan 04-01** — Foundations + hand-publish removal.
  - Add `kRightArmJointIndices` (7 `JointIndex` values, file-scope `static constexpr`).
  - Add `kRightArmJointNames` (7 `std::string_view` or `std::string`, file-scope `static const`, kept in sync with the indices array via adjacent definition + comment).
  - Delete the two `hand_cmd_pub->publish(hand_cmd)` calls (L182 and L270).
  - No publish-loop narrowing yet → behavior identical to today modulo no hand publish.
  - Build verification only.

**Wave 2 (can be parallel; both depend on 04-01):**
- **Plan 04-02** — Trajectory validation (D-03, D-04, D-05).
  - Insert WARN-and-strip + completeness-ERROR block immediately after empty-`joint_names` guard.
  - Introduce `filtered_indices` / `filtered_names` local working copies.
  - Replace `for (j = 0; j < point.positions.size(); ++j)` in the waypoint loop with a loop driven by `filtered_indices`.
  - Build verification + on-robot bench tests #3 and #4 from §7.2.
- **Plan 04-03** — Narrow the three publish loops (D-06 strategy).
  - Surface the §3.5 question to the user before coding.
  - Plumb the chosen `NonRightArmPolicy` through waypoint / hold / ramp.
  - Build verification + on-robot bench tests #1 and #2 from §7.2 (standing baseline + running mode).

Alternative: roll Plans 04-02 and 04-03 into a single 04-02 if the planner judges the validation work too small to justify a separate plan. Both options are reasonable.

The planner agent should also include the §3.5 user-facing question explicitly in 04-03's plan body (not silently defaulting to Option A) so the user has a decision-point.

---

## RESEARCH COMPLETE

Key blockers / surface-to-user items:
- **D-06 contradiction**: CONTEXT.md `<domain>` says "leave default" for non-right-arm slots; existing executor (Plan 01-10) and Unitree reference (free_arm_demo.py) both *populate* those slots. Three documented options (A/B/C in §3.5); Option A (status quo) is the safest default but does not match CONTEXT.md text. Planner should ask user which policy to enforce.
- **Running-mode bench test** is not a code deliverable — planner should plan it as an explicit `autonomous: false` UAT step, not as an executor task.

Everything else (D-01 hand removal, D-03 validation, D-04 empty-guard interaction, D-05 read-only msg, indices-array implementation strategy) has a clear path forward documented above.
