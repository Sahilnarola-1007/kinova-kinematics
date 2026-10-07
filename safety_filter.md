# safety_filter.md — SafetyFilter (kinova_kinematics)

Three-site safety filter for the Idea 5 adapter. **Layer: below the contract**, 1 kHz path,
ACTION side — between the frozen policy's action output and the commanded joint positions.
Part of `kinova_kinematics`; summarised in `design.md` §5.6.

**Decision (D-07) `[DESIGN]`:** engineered clipping with logged interventions. A formal barrier
certificate (control barrier function) is future work — a swap behind the same interface, not a
retrain. See `design_decisions.md`.

Provenance tags: `[MEASURED]` lab PC / hardware, run identified · `[SPEC]` vendor document ·
`[DESIGN]` project decision · `[UNVERIFIED]` not checked, with what would check it.

---

## In one paragraph

The policy emits an action. Nothing guarantees that action is safe for *this* arm — the
policy is frozen and knows nothing about this arm's joint limits or the table edge. The SafetyFilter is the component that makes the action executable on real
hardware, and it does so at three points in the pipeline where a different kind of
quantity is available to check: the raw force command, the assembled Cartesian twist, and
the resulting joint rates. Its worst measured p99.9 is **≤ 40 ns per cycle** `[MEASURED, §Verification]`, 0.004 % of the
1 kHz budget, so its cost is never an argument against making it stricter.

Two things it deliberately does **not** do: it never touches `vz` as a magnitude bound
(the admittance reflex owns the normal axis), and it never feeds its own output back to
the policy as an observation (that would leak robot-specific information above the
contract).

---

## Data flow

```
  ┌─────────────────────────────────────┐
  │  Frozen policy  (~30 Hz)            │
  │  action = [F*n, vx, vy, wx, wy, wz] │
  └──────────────┬──────────────────────┘
                 │
       ┌─────────▼──────────┐
       │  Site 1 — action   │  Clamp F*n only (scaling).
       │  clipForce()       │  Pre-reflex. Sign preserved.
       └─────────┬──────────┘
                 │ F*n_clipped + [vx, vy, wx, wy, wz]
                 │
  ┌──────────────▼──────────────────────────────────┐
  │  Admittance reflex          F/T sensor (500 Hz) │
  │  own clamp + anti-windup ◄──── uses -fz         │
  │  produces vz from F*n + measured wrench         │
  └──────────────┬──────────────────────────────────┘
                 │ vz
                 │
       ┌─────────▼──────────────────────────────────┐
       │  Assemble twist, task frame                │
       │  [vx, vy, vz_reflex, wx, wy, wz]           │
       └─────────┬──────────────────────────────────┘
                 │
       ┌─────────▼──────────────────────────────────┐
       │  Site 2 — Cartesian       clipCartesian()  │
       │                                            │
       │  Group A:  α = min(1, v_tan_max / |v_tan|) │
       │            scales [vx, vy] only            │
       │                                            │
       │  Group B:  β = min(1, ω_max / |ω|)         │
       │            scales [wx, wy, wz] only        │
       │                                            │
       │  vz EXCLUDED from both norms               │
       │                                            │
       │  Workspace box: REFUSAL on all axes        │
       │  (zero outward component, allow inward)    │
       └─────────┬──────────────────────────────────┘
                 │ twist_task_clipped
                 │
       ┌─────────▼──────────────────────────────────┐
       │  Rotate task → base frame                  │
       │  then jointVelocityDLS()                   │
       └─────────┬──────────────────────────────────┘
                 │ q̇ (7-vector, rad/s)
                 │
       ┌─────────▼──────────────────────────────────┐
       │  Site 3 — joint           clipJoint()      │
       │                                            │
       │  Velocity: UNIFORM scaling of full q̇       │
       │    α = min_i( cap_i / |q̇_i| )              │
       │    Per-joint clipping is forbidden —       │
       │    it distorts the Cartesian direction.    │
       │                                            │
       │  Position: REFUSAL via single-step         │
       │    lookahead on q_send + q̇·dt.             │
       │    Zero the offending joint component.     │
       └─────────┬──────────────────────────────────┘
                 │ q̇_safe
                 │
       ┌─────────▼──────────────────────────────────┐
       │  Commanded-anchor integrator               │
       │  q_send[k] = q_send[k-1] + q̇_safe · dt     │
       │  then Refresh() to the arm                 │
       └────────────────────────────────────────────┘
```

### Why three sites and not one

Each site is placed at the only point in the pipeline where the quantity it bounds
actually exists. Force is only meaningful before the reflex consumes it. The Cartesian
twist does not exist until the reflex has supplied `vz`. Joint rates do not exist until
DLS IK has run. A single filter at the end could not enforce a force bound, because by
then the force command has already been turned into motion; a single filter at the front
could not enforce a joint limit, because the mapping from action to joint space is not
known until the Jacobian is evaluated.

---

## Two bound classes

The filter enforces two mathematically distinct types of bound.
The result types distinguish them so the logger knows what happened.

| Class | When it fires | What it does | Direction preserved? |
|-------|---------------|--------------|----------------------|
| **Scaling** | Magnitude too large | Multiply by α ∈ (0, 1) | Yes |
| **Refusal** | Motion toward a forbidden region | Zero the offending component | No — by necessity |

Scaling bounds: `f_n_max`, `v_tan_max`, `ω_max`, joint velocity caps.
Refusal bounds: workspace box, joint position limits.

The distinction matters for interpretation, not just bookkeeping. A scaling event means
*"you asked for too much of the right thing"* — the commanded direction survives, the
task usually still completes, more slowly. A refusal means *"you asked to go somewhere
forbidden"* — the direction is altered, and the resulting motion is not what the policy
requested. Refusals are therefore the more serious finding in a log, and the O3
requirement is written against both.

---

## Why vz is excluded from Site 2

The normal-axis velocity `vz` is produced by the admittance reflex from `F*n` and the
measured wrench. The reflex has its own velocity clamp and anti-windup.

If Site 2 included `vz` in the `|v|` norm, a large reflex output would push the norm over
`v_tan_max` and trigger scaling on `[vx, vy]` — throttling the tangential search to fix a
normal-axis value the filter doesn't own. Worse, the reflex's PI would chase an
unreachable force target (the filter silently reduces its commanded velocity), the
integrator would ramp to the clamp (~4 s under sustained error `[MEASURED — 13 Hz wipe
predecessor, admittance-controller; run not identified here]`), and the
effective controller would no longer be the tuned one. Force-band numbers become
unrepeatable.

The general form of the rule: **two limiters in series on the same axis make neither one
analysable.** You can no longer attribute an observed force error to the reflex tuning or
to the filter, because both are acting and neither logs the other's contribution.

The fix is structural: `vz` does not belong in a motion-control magnitude bound because
the normal axis is force-controlled — the hybrid force/motion principle
`[UNVERIFIED — Sahil must check the primary sources before citing any author]`. The workspace box is the only Site 2
bound that touches `vz`, and it does so as a refusal (safety-of-last-resort), not a
scaling.

---

## Bounds table

All values are parameters from `SafetyBounds`. They are header defaults `[DESIGN, provisional —
D-09 open]` until the manifest mechanism lands (D-04); nothing is sourced from a manifest yet.

### Site 1 — Force

| Bound | Default | Units |
|-------|---------|-------|
| `f_n_max` | 15.0 | N |

### Site 2 — Cartesian

| Bound | Default | Units |
|-------|---------|-------|
| `v_tan_max` | 0.15 | m/s |
| `omega_max` | 0.50 | rad/s |
| Workspace box | 0.10–0.80 x, −0.15–0.80 y, 0.02–0.90 z | m, base frame |

The box was widened on 7 Oct 2026 `[DESIGN, provisional]`. The earlier box (±0.5 x, ±0.5 y,
0.05–0.70 z) excluded the arm's Home pose: the tool tip sits at x = 0.570 m there `[COMPUTED —
computeFK on the logged start pose, run 1007_1556]`, 7 cm outside the old x face. What that did
on hardware is recorded under *Hardware findings* below.

Reference `[SPEC — Kinova Gen3 User Guide, spherical wrist; Sahil to record the table number]`:
Cartesian hard limits 0.5 m/s linear, 0.8727 rad/s (50 °/s) angular, 40 N force, 15 N·m torque.
Our defaults are within these.

### Site 3 — Joint

| Bound | Joints 1–4 | Joints 5–7 | Units | Source |
|-------|-----------|-----------|-------|--------|
| Velocity cap | 2.094 (120 °/s) | 3.491 (200 °/s) | rad/s | `[SPEC]` Kortex actuator spec, Aug 2026; not confirmed by hardware test |
| Position min | −∞, −2.2515, −∞, −2.5800, −∞, −2.0996, −∞ | | rad | `[SPEC]` Kinova Gen3 User Guide Table 39 |
| Position max | +∞, +2.2515, +∞, +2.5800, +∞, +2.0996, +∞ | | rad | `[SPEC]` Kinova Gen3 User Guide Table 39 |

Joints 1, 3, 5, 7 are continuous rotation (no position limit). In code the sentinel is
±1e9, not infinity, so any limit test must compare against the sentinel threshold rather
than calling `isinf`.

**Angle convention — a precondition on `clipJoint`.** The position limits are symmetric about
zero, so `q_send` must be in the signed convention, (−π, +π]. Kortex reports and accepts joint
positions wrapped to [0°, 360°) `[SPEC — Kinova support, 4 Oct 2026]`, so the caller converts.
The 1 kHz loop (`kinova_lowlevel`, `step1_loop`) does it at one seam: wrapped → signed on every
measured joint on the way in, signed → wrapped on every commanded joint on the way out. The
filter cannot detect a violation of this precondition; see *Hardware findings*.

Note `[SPEC]`: the User Guide also lists high-level control library velocity limits
(Table 40: 79.64 °/s / 69.91 °/s general; Table 41: 50 °/s admittance). These are read as
**not applied in low-level servoing mode** `[UNVERIFIED by test — an over-cap command in
LOW_LEVEL_SERVOING would settle it]`, which is the mode this project uses. The actuator caps
(120 / 200 °/s) are the bounds for our path.

**Site 3 velocity scaling is load-bearing, not defensive dead code.** In tested
mid-workspace poses, DLS output reaches ‖q̇‖ = 3.39 rad/s `[COMPUTED — pose not recorded;
record it]` against the 2.094 rad/s cap. It fires in normal operation and must be included in
any latency number.

---

## Intervention counter

Every clip site reports a reason code from `FilterReason`. The `InterventionCounter`
accumulates per-bound counts across a run.

This is not optional plumbing — the O3 objective ("zero bound violations across every
reported run") is unmeasurable without per-bound reason codes and a count. A nonzero
count is a **finding** (bounds and training distribution disagree), not a bug to suppress.

| Counter field | Fires when |
|---------------|------------|
| `force_scaled` | Site 1 clips F*n |
| `v_tan_scaled` | Site 2 clips tangential velocity |
| `omega_scaled` | Site 2 clips angular velocity |
| `workspace_refused` | Site 2 refuses workspace boundary |
| `joint_vel_scaled` | Site 3 uniformly scales q̇ |
| `joint_pos_refused` | Site 3 refuses a joint position |
| `non_finite_rejected` | Any site receives NaN or ±inf |

`non_finite_rejected` increments **once per site**, so a run in which every cycle is
poisoned reports 3 × cycles, not 1 × cycles. This is deliberate: it tells you *how many
guards fired*, not how many cycles were bad, and a drop to 1× would mean a site had
stopped guarding its input.

`JointFilterResult::pos_refused_mask` is a per-call bitmask, bit *j* set = joint *j*
refused. It is not accumulated in the counter. Because joints 1, 3, 5, 7 are continuous
rotation, **the mask can never legally exceed 42** (`0b0101010`, bits 1/3/5 = joints
2/4/6). A larger value means a sentinel was treated as a real limit.

In the 1 kHz loop the mask of the most recent cycle is exposed by
`ControlCycle::lastPosRefusedMask()` and logged per cycle by `step1_loop` as the
`pos_refused_mask` column (added 7 Oct 2026). The cumulative counter is reachable through
`ControlCycle::interventions()`. Site 2 refusals are counted there but are not yet logged per
cycle.

### Why `inf` was the dangerous case

`NaN > bound` is false, so NaN slips through a naive comparison silently. `inf > bound` is
**true**, so an infinite input registers as a genuine bound violation and inflates the O3
count with phantom interventions. Both are now rejected explicitly at every site, and the
benchmark exercises both flavours.

---

## Design rules

1. **Bounds are parameters, never literals in the algorithm.** `SafetyBounds` struct; header
   defaults today, manifest-sourced after D-04. Values are `[DESIGN, provisional]` until the
   trained action distribution sets them (D-09).

2. **Clipped actions are not fed back to the policy.** The clip effect reaches the next
   observation via measured force and pose — that is correct and unavoidable. The clip
   quantity itself never appears as an observation element (it is a per-robot bound, and
   admitting it would put robot-specific information above the contract).

3. **No allocation on the hot path.** All Eigen types are fixed-size. All results are
   returned by value on the stack.

4. **Thread safety: not required.** Called from the single 1 kHz thread.

5. **Non-overlapping responsibility.** Each axis is owned by exactly one limiter.
   The reflex owns vz. Site 2 owns vx, vy, ω. Site 3 owns joint velocity and position.
   The workspace box is the only cross-cutting bound and it acts as refusal
   (safety-of-last-resort), not a competing controller.

6. **Uniform scaling, never per-joint clipping.** Clipping joints independently changes
   the ratio between them, which changes the Cartesian direction the joint rates were
   solving for — the arm moves somewhere the policy did not ask to go. Uniform scaling by
   the single worst ratio preserves direction and only slows the motion.

---

## Verification

### Unit tests — `test_safety_filter.cpp`, 28 GTest cases

Coverage: each bound in isolation, sign preservation on Site 1, `vz` exclusion from both
Site 2 norms, inward-motion allowance at the workspace faces, uniform-scaling ratio
preservation on Site 3, continuous-joint sentinel handling, and non-finite rejection at
all three sites.

Re-run after the workspace box change: `colcon test` on `kinova_kinematics` reports 47 tests,
0 failures `[MEASURED — lab PC, hardware build tree, 7 Oct 2026]` (45 gtest cases plus the two
binary-level CTest entries).

### Latency benchmark — `benchmark_safety_filter.cpp`

Times a full cycle (`clipForce` → `clipCartesian` → `clipJoint`) under four scenarios,
because the filter has data-dependent branches and therefore no single worst case.

| Scenario | What it exercises |
|----------|-------------------|
| `NOMINAL` | The deployment path. Every input inside every bound — the filter should be silent (D-09). |
| `ALL_TRIP` | Maximum work. Every bound fires on every cycle. |
| `ALTERNATING` | Nominal and all-trip inputs interleaved, to defeat the branch predictor. |
| `NON_FINITE` | NaN on even entries, +inf on odd. Times the reject path. |

**Protocol.** `RelWithDebInfo` (`-O2 -g -DNDEBUG`, `[MEASURED]` verified in `compile_commands.json` 4 Sep 2026),
`chrt -f 80 taskset -c 2`, N = 1e6 per scenario, inputs cycled from a 64-entry L1-resident
table so the compiler cannot hoist the call. Identical to the `benchmark_dls` protocol —
which is what makes the two numbers additive. `RelWithDebInfo` rather than `Release`
deliberately: `-g` costs zero cycles and is what makes `perf` output readable when
profiling the composed 1 kHz loop.

**Results `[MEASURED]` — lab PC, core 2 pinned, 4 Sep 2026. Two runs of identical code, shown
as `run1 / run2` where they differ. All values in ns:**

| Scenario | mean | batched | p50 | p99 | p99.9 | max |
|----------|-----:|--------:|----:|----:|------:|----:|
| NOMINAL | 21.0 / 20.7 | 10.0 / 10.1 | 21 | 27 / 26 | 35 / 33 | 5391 / 1623 |
| ALL_TRIP | 30.2 / 30.5 | 15.0 / 15.2 | 30 | 36 / 35 | 39 / 38 | 1311 / 964 |
| ALTERNATING | 25.6 | 12.6 / 12.5 | 28 / 27 | 35 / 34 | 40 / 36 | 1142 / 6851 |
| NON_FINITE | 12.6 | 5.1 / 5.0 | 13 | 14 | 21 / 26 | 785 / 7338 |

Clock floor: **not cited** — readings conflict across sources (9 ns vs 13 ns); resolve at Step 9
before any timing number is published. The bracket below holds under either value.

**Reportable number: worst p99.9 across scenarios and runs ≤ 40 ns = 0.004 % of the 1 kHz
budget.**

Reading the columns:

- **batched** — 64 filter cycles between one pair of clock reads, so clock overhead is
  divided away. Optimistic: those 64 cycles run back to back, which the real loop never
  does.
- **mean** — per-iteration, so no back-to-back effect, but each sample pays one
  clock read (single-digit ns) the real loop never pays. Pessimistic.
- **True single-cycle cost is bracketed between them: 5–30 ns `[MEASURED, two bounds]`.**
  Report the bracket, never either column alone and never a point value.
- **p50/p99/p99.9/max** come from the per-iteration run, because batching averages the
  tail away. They carry the clock read too; treat them as upper bounds.
- Ordering is physically sensible: NON_FINITE cheapest (early reject, no arithmetic),
  NOMINAL next, ALL_TRIP most expensive (every scaling branch executes).

**What this does not show.** The four scenarios are separated by 1–5 ns at a single-digit-ns
clock resolution, so they are **indistinguishable in the tail** — and the worst-case label
actually moved between the two runs (ALTERNATING at 40 ns, then ALL_TRIP at 38 ns) on
identical code. The branch-predictor hypothesis behind ALTERNATING is neither supported
nor refuted. Claim a bound, never a ranking. The `max` column is OS events, not filter
work: the lone outlier landed on NOMINAL in run 1 and on NON_FINITE in run 2.

**Composed cost, DLS + filter — `[ESTIMATE, not measured]`:** mean-sum 1.90 µs + ~0.02 µs
≈ 1.9 µs; tail-sum 3.04 µs + 0.04 µs ≈ 3.1 µs. Go/no-go only. The p99.9 of a sum is not the
sum of the p99.9s — that treats the tails as perfectly correlated. The reportable number comes
from instrumenting the composed 1 kHz loop end to end (Step 8).

### Counter self-checks

The benchmark asserts the `InterventionCounter` deltas each scenario claims to produce and
fails loudly (exit 1) if they disagree. A benchmark that believes it is exercising the
worst case but silently isn't is worse than no benchmark. Counters are reset immediately
before the timed loop, so deltas belong to exactly N calls and exact counts are checkable.

| Scenario | Asserted | Measured 4 Sep 2026 |
|----------|----------|---------------------|
| NOMINAL | `total() == 0` | all counters **0** ✓ |
| ALL_TRIP | every bound > 0; `non_finite_rejected == 0`; `pos_refused_mask == 42` | 1,000,000 on all six bounds; mask **42** ✓ |
| ALTERNATING | same as ALL_TRIP | 500,000 on all six bounds; mask **42** ✓ |
| NON_FINITE | `non_finite_rejected == 3 × N` | **3,000,000** ✓ |

All four scenarios pass, exit 0. What each result establishes:

- **NOMINAL silent.** With every input inside every bound, no site fires across 10⁶
  cycles. This is the D-09 premise — bounds sit outside the action distribution, so the
  filter is structurally silent in normal operation.
- **3 × N, not N.** The non-finite guard is live at all three sites independently. A
  reading of N would mean one or two sites had stopped guarding.
- **Mask exactly 42.** Only joints 2, 4, 6 refused. The ±1e9 continuous-rotation
  sentinels on joints 1, 3, 5, 7 were correctly never treated as real limits.
- **ALTERNATING is exactly half of ALL_TRIP** on every counter (500,000 vs 1,000,000).
  The input table alternates nominal and all-trip entries, so half the cycles should trip.
  It does — which independently confirms the table is cycling and not stuck on one entry.

---

## Hardware findings — 6–7 October 2026

First runs of the filter inside the 1 kHz loop on the arm, free space, scripted motion
(`ScriptedSource`: vx 0.02 m/s at 0.2 Hz, vy 0.02 m/s at 0.3 Hz, wx 0.05 rad/s at 0.15 Hz,
wz 0.10 rad/s at 0.25 Hz; task frame = base frame; start at the Home pose). Run folders are
under `results/step3_hw/scripted_action/`.

### A directional refusal on a state that is already outside its bound is a one-way valve

Both refusals pass inward motion on purpose, so that an arm that has left the bound can come
back. The consequence: if the state *starts* outside the bound, every outward command is
zeroed, every inward command passes, and a commanded oscillation turns into one-signed travel.
Nothing faults and nothing looks wrong cycle by cycle. This happened at two sites in the same
run, `1006_1503_repeat_poseA_25s_expect_fault` (23,654 cycles before the arm faulted):

| Site | State outside its bound | What the arm did `[MEASURED — computeFK on logged q_meas]` |
|------|-------------------------|-----------------------------------------------------------|
| Site 2, workspace x | Tip at x = 0.570 m against `ws_x_max` = 0.50 m | Tip travelled −13.1 cm in x. Commanded: 0 → +3.2 cm → 0, never negative. |
| Site 3, joint 4 | `q_send` seeded wrapped: 229.998° (4.01 rad) read against a +2.58 rad limit | `q_send` for joint 4 increased on 0 of 23,653 cycles and travelled −22.43°, ending 4.6° past its −147.82° limit, where the arm faulted. The tip fell 7.6 cm in z, an axis that was never commanded. |

The y axis tracked correctly throughout: the tip was inside the y faces, and joint 4 contributes
nothing to y at this pose.

Attribution `[COMPUTED — independent Python reimplementation of FK, Jacobian, DLS, the three
filter sites, the scripted source and a first-order servo, τ = 15.7 ms, λ = 0.05]`: with both
mechanisms present the model reproduces the logged run to within 0.07 cm at the tip and 0.015°
on joint 4 over all 23,654 cycles. In the model the workspace refusal fires on 7,546 cycles
(cycles 77 to 12,550) and the joint 4 refusal on 8,832. Those two counts are model output — the
6 Oct log carried neither a per-cycle mask nor the counter.

The second case is an angle-convention error, not a filter defect: the directional test is
correct and is unchanged. It is recorded here because the filter is where it showed.

### After the two corrections

Signed convention at the loop seam, workspace box widened. Same pose, same scripted source:

| Run | Cycles | `pos_refused_mask` ≠ 0 | Tip x / y / z range [cm from start] | Result |
|-----|-------:|-----------------------:|-------------------------------------|--------|
| `1007_1556_signed_ws080_poseA_25s` | 25,000 | 0 | −0.10…+3.08 / −0.01…+2.45 / −0.17…0.00 | completed, no fault |
| `1007_1616_signed_ws080_poseA_60s` | 60,000 | 0 | −0.23…+3.08 / −0.01…+2.46 / −0.34…0.00 | completed, no fault |

Mask and completion `[MEASURED — step1_loop log]`; tip ranges `[MEASURED — computeFK on logged
q_meas]`. Joint 4 moved in both directions (12,402 cycles positive, 12,514 negative in the 25 s
run). The tip stayed between x = 0.567 and 0.600 m, y = 0.001 and 0.026 m, z = 0.430 and
0.434 m over the 60 s run — inside the box on every axis, so the workspace refusal had nothing
to fire on. The Site 2 counter itself was not printed in these runs.

The y peak of 2.45 cm against 2.12 cm commanded is not a filter effect; it is the DLS damping
at λ = 0.05 (`design.md` §8.6).

---

## Limitations and future work

- **Workspace box uses base-frame position but zeros task-frame velocity.** Correct
  only when the two frames are roughly axis-aligned `[DESIGN]` (insertion on a horizontal
  surface). Verify if the task frame rotates significantly from base.

- **Formal barrier certificate (future work).** The current implementation is engineered
  clipping (D-07). A barrier-function filter would be a swap behind the same `SafetyFilter`
  interface — same three call sites, same result types, different math inside. A refactor,
  not a retrain.

- **Bound values:** genuinely blocked on the trained action distribution. The filter
  is built now with the struct; values land after training (D-09).

- **D-27 — joint limits duplicated.** The limits are written twice, in `KinovaKinematics.cpp`
  and in `SafetyFilter.hpp`: two sources of truth for one physical fact. As of 7 Oct 2026 the
  two copies agree (±2.2515, ±2.5800, ±2.0996 rad on joints 2/4/6; ±1e9 sentinels on the
  continuous joints), but nothing enforces that. A single manifest is required.

- **A refusal on a state already outside its bound is silent and one-directional.** See
  *Hardware findings*. Two consequences for the caller: the start state has to be inside every
  refusal bound for commanded motion to be reproduced, and a refusal that is not logged per
  cycle is indistinguishable from drift. Site 3's mask is logged per cycle; Site 2's refusal is
  only counted.

- **Site 3 trusts the angle convention of `q_send`.** Limits are symmetric about zero and the
  filter has no way to tell a wrapped angle from a signed one. The conversion is the caller's
  job (see *Bounds table*, Site 3).

- **Position refusal bounds the commanded setpoint, not the measured joint.** `clipJoint`
  checks `q_send + q̇·dt` against the limit, `dt = pos_lookahead_dt` = 1 ms `[DESIGN]`. Under
  a first-order servo the joint is pulled toward `q_send` and settles there, so lag alone
  cannot produce a violation — it only delays arrival. τ = 15.7 ms `[MEASURED, joint 7 only;
  UNVERIFIED for joints 2, 4, 6]` is the lag, and a τ-sized horizon would apply only to a
  check made against `q_meas`; applying it to a `q_send` check would refuse ~16× early for
  no safety gain. **Open — overshoot.** If the servo overshoots its setpoint, setpoint
  bounding is not sufficient. Not measured. Step 3 measures peak `q_meas − q_send` after a
  step on joints 2, 4 and 6 at several speeds; that peak becomes a margin ε, refusing
  against `limit − ε`, rather than a change of horizon. If ε = 0 the current check stands
  unchanged and the `stop_horizon_s` rename is dropped.

- **Low-level limit enforcement — answered.** The firmware does not apply joint position
  limits in low-level servoing: "Joint limits is a high level control feature that won't be
  applied in low level" `[SPEC — Kinova support, 4 Oct 2026]`. Consistent with the 6 Oct run,
  where joint 4 was commanded 4.6° past its limit before the arm faulted `[MEASURED]`. Site 3
  is the only enforcement of the position limits on this path.

- **Sustained-intervention flag.** Sustained admittance scaling drives integrator
  divergence. Whether the filter should raise a status flag on sustained intervention (as
  opposed to counting it) is undecided.

---

## Files

| File | Purpose |
|------|---------|
| `SafetyFilter.hpp` | Interface — types, bounds struct, class declaration |
| `SafetyFilter.cpp` | Implementation — three clip site methods |
| `tests/test_safety_filter.cpp` | 28 GTest cases |
| `tests/benchmark_safety_filter.cpp` | Four-scenario latency benchmark with counter self-checks |
| `safety_filter.md` | This document — architecture, rationale, bounds, results |

---

*Last updated: 7 October 2026 — workspace box widened; first hardware findings recorded;
angle-convention precondition on Site 3 stated; firmware limit question closed; D-27 wording
corrected to the current code. Previous: 11 September 2026 — documentation audit (provenance
tags, clock floor withdrawn, test count corrected to 28). Benchmark closed 4 Sep 2026, Phase 0
Step 6.4.*
