# kinova_kinematics

> From-scratch forward kinematics, Jacobian, damped-least-squares differential IK and the
> three-site safety filter for the Kinova Gen3 7-DOF — the kinematics-and-clipping stage of the
> adapter, **below the contract**, on the 1 kHz path.

**Part of:** Idea 5 — a frozen force-conditioned connector-insertion policy ported across cobots
through a locked interface contract. Real validation on one arm (Gen3 7-DOF); cross-body
evidence primarily in simulation (MuJoCo). See `idea5_full_proposal.pdf` in the umbrella project.

**Layer:** Below the contract (adapter). Knows the robot. This is the part a port rewrites.

---

## What problem this solves

The policy outputs a six-element action in the task frame; the arm accepts joint positions.
Something has to turn a Cartesian twist into joint rates a thousand times a second, without a
vendor library, and refuse anything that would hurt the arm. This package is that something:
FK for the observation side, a damped differential IK for the action side, and a safety filter
that clips force, Cartesian velocity and joint rates at the three points where each quantity
first exists. All maths is written on Eigen from first principles — no Kortex, no MoveIt — so
it builds and is fully tested on a machine with no robot attached.

---

## Architecture

```
 ~10 Hz   ABOVE THE CONTRACT   perception → task-frame builder → TargetSource (phase + goal)
                               │ task frame · goal · phase
 ~30 Hz   AT THE CONTRACT      frozen ONNX policy   obs 38 (v1, pending D-01) ↑   action 6 ↓
                               │ [F*n, vx, vy, wx, wy, wz]
  1 kHz   BELOW THE CONTRACT ┌──────────────────────────────────────────────────────────┐
          (adapter)          │  SafetyFilter Site 1 (F*n)          ◄── THIS PACKAGE     │
                             │  admittance reflex → vz             (admittance-controller)│
                             │  SafetyFilter Site 2 (twist)        ◄── THIS PACKAGE     │
                             │  jointVelocityDLS → q̇               ◄── THIS PACKAGE     │
                             │  SafetyFilter Site 3 (q̇)            ◄── THIS PACKAGE     │
                             │  q_send += q̇·dt → Refresh()         (kinova-wrapper)     │
                             │  computeFK → pose, 6D rotation      ◄── THIS PACKAGE     │
                             │  6-axis F/T sensor                  (mae-sensor-driver)  │
                             └──────────────────────────────────────────────────────────┘
```

Two independent paths run through this package and share no code:

```
  OBSERVATION SIDE (measured, flows up)        ACTION SIDE (commanded, flows down)
  ───────────────────────────────────          ─────────────────────────────────────
  q_meas [rad]                                 twist_task [m/s, rad/s]  (vz from reflex)
     │                                            │  Site 2 clip; caller rotates task→base
     ▼                                            ▼
  computeFK ─► T (base) [m]                    jointVelocityDLS(q_meas, twist_base, λ)
     │  getRotation: first two columns             │  J at MEASURED q  →  (JJᵀ+λ²I) LDLᵀ  →  q̇
     │  = 6D orientation obs — NO Gram-Schmidt      ▼
     ▼                                         Site 3 clip (uniform scale, pos refusal)
  observation builder (kinova-wrapper)            ▼  q̇_safe [rad/s]
                                               q_send = q_send_prev + q̇·dt  (kinova-wrapper)
```

*Figure: this package's position in the Idea 5 layer model and the obs/action split inside it.
Structural — nothing measured. Sep 2026.*

---

## Interface

All functions are C++ methods; no ROS topics are published by this package. Rates are the
rates at which the 1 kHz loop is designed to call them `[DESIGN]`.

| Name | Side | Direction | Type | Frame | Units | Rate |
|---|---|---|---|---|---|---|
| `computeFK(q)` | OBS | in q → out T | `array<double,7>` → `Matrix4d` | q joint space; T **base** | rad → m | 1 kHz |
| `getPosition(T)` | OBS | in T → out p | `Matrix4d` → `Vector3d` | base | m | 1 kHz |
| `getRotation(T)` | OBS | in T → out R | `Matrix4d` → `Matrix3d` | base | — (cols 0–1 = 6D obs) | 1 kHz |
| `computeJacobian(q)` | ACTION | in q → out J | `array<double,7>` → `Matrix<6,7>` | base | rows 0–2 m/rad, 3–5 dimensionless | 1 kHz (inside DLS) |
| `jointVelocityDLS(q_meas, twist, λ)` | ACTION | in → out q̇ | → `DlsResult` | twist **base**; q̇ joint | m/s, rad/s → rad/s | 1 kHz |
| `solveDLS(J, twist, λ)` | ACTION | test seam | → `DlsResult` | base | as above | test only |
| `manipulability(J)` | monitor | in J → out w | `Matrix<6,7>` → `double` | — | mixed (§5.4 design.md) | optional |
| `solveIK(pose, guess, …)` | offline | pose → q | `Matrix4d` → `IKResult` | base | m, rad | **not on the loop** |
| `SafetyFilter::clipForce(F*n)` | ACTION, Site 1 | in → out | `double` → `ForceFilterResult` | tool z | N | 1 kHz |
| `SafetyFilter::clipCartesian(twist, ee_pos)` | ACTION, Site 2 | in → out | `Matrix<6,1>`, `Vector3d` → `CartesianFilterResult` | twist **task**; ee_pos **base** | m/s, rad/s; m | 1 kHz |
| `SafetyFilter::clipJoint(q̇, q_send)` | ACTION, Site 3 | in → out | `Matrix<7,1>` ×2 → `JointFilterResult` | joint space | rad/s; rad | 1 kHz |
| `SafetyFilter::interventions()` | telemetry | out | `InterventionCounter&` | — | counts | on demand |

`DlsResult::ok == false` ⇒ `qdot` is zero and must not be commanded; the loop holds `q_send`.
`DlsResult::manipulability` is `NaN` by design (D-20) — the monitor is `manipulability(J)`.

**Excluded from the observation on purpose:** joint angles, joint velocities, joint torques,
motor currents, manipulability. All are robot-specific and stay below the contract. Only the
FK-derived pose and 6D rotation cross upward.

---

## Build and run

This package depends only on `ament_cmake` and `Eigen3`. It has **no Kortex dependency**, so
the mock/real distinction that governs `kinova-wrapper` does not apply here: the flag
`-DUSE_KORTEX_MOCK` is not read by this package's `CMakeLists.txt`. Both rows build the same
binaries.

| Mode | Command | Needs the robot? |
|---|---|---|
| Mock (default) | `colcon build --packages-select kinova_kinematics --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo` | No |
| Real hardware | same command — no Kortex code in this package | No |

```bash
# unit tests (GoogleTest; 17 DLS/manipulability cases + 28 safety-filter cases)
colcon test --packages-select kinova_kinematics
colcon test-result --verbose

# FK smoke check (own main(), prints FK / Jacobian / IK round-trip)
./build/kinova_kinematics/test_fk

# latency benchmarks — SCHED_FIFO 80, pinned to core 2 (P-core). Build type is part of
# the protocol: RelWithDebInfo (-O2 -g -DNDEBUG), verified in compile_commands.json 4 Sep 2026
chrt -f 80 taskset -c 2 ./build/kinova_kinematics/benchmark_dls dls_latency.csv
chrt -f 80 taskset -c 2 ./build/kinova_kinematics/benchmark_safety_filter filter_latency
```

Numbers measured at `-O0` are meaningless: Eigen's expression templates only collapse under
inlining.

---

## Results

Tags: `[MEASURED]` on the lab PC (Ubuntu 24.04, core 2 pinned), run identified ·
`[COMPUTED]` from this library's own code, no hardware · `[SPEC]` vendor document ·
`[DESIGN]` project decision · `[UNVERIFIED]` not yet checked.

**Differential IK — `jointVelocityDLS`** `[MEASURED]` `benchmark_dls`, 4 Sep 2026,
N = 10⁶, RelWithDebInfo, `chrt -f 80 taskset -c 2`:

| mean | p50 | p99 | p99.9 | max (1 in 10⁶) | budget |
|---|---|---|---|---|---|
| 1.90 µs | 1.86 µs | 2.35 µs | **3.04 µs** | 94.4 µs | 1000 µs |

~0.2 % of the cycle typically, ~0.3 % at the 1-in-1000 tail. The tight p99.9-vs-mean gap is
consistent with no heap allocation on the hot path `[INFERRED]` — fixed-size Eigen throughout.

**Safety filter — one full cycle, three sites** `[MEASURED]` `benchmark_safety_filter`,
4 Sep 2026, two runs of identical code, four scenarios, same protocol as above:

| Quantity | Value |
|---|---|
| Worst p99.9 across four scenarios and two runs | **≤ 40 ns** (0.004 % of cycle) |
| True single-cycle cost | **bracketed 5–30 ns** (batched mean is the lower bound, per-iteration mean the upper) |
| Counter invariants | NOMINAL silent; `non_finite_rejected = 3 × N`; `pos_refused_mask = 42` — all asserted, all passed |

No scenario is reported as "the worst case": the label moved between the two runs on identical
code, and the scenarios sit 1–5 ns apart at a single-digit-ns clock resolution.

**Composed IK + filter** `[ESTIMATE — not measured]`: mean-sum ≈ 1.9 µs; tail-sum (p99.9 + p99.9)
≈ 3.1 µs. Neither is a result. The reportable composed figure comes from instrumenting the
assembled 1 kHz loop end to end (kinova-wrapper, Step 8).

**Clock floor:** readings conflict across sources (9 ns vs 13 ns). Not cited until resolved
(Step 9).

**FK** `[COMPUTED]`: flange z = 1.1873 m at q = 0 before the tool offset; ±0.3 rad symmetry
checks pass. **Hardware FK-vs-Kortex comparison not yet recorded** — it will carry a ~7 mm
z difference by construction (Kortex tool offset 0.12 m vs physically measured 0.113 m, D-14).

**Latency distribution figure:** CSVs exist (`dls_latency.csv`, `filter_latency_<scenario>.csv`);
the histogram is committed at Step 9.

---

## Limitations and scope

- **Measured vs estimated.** IK latency and filter latency are measured; the composed cost is
  a sum-of-tails estimate; the Jacobian/solve split is not measured; the clock floor is
  conflicted. The σ spectrum and link-scale table in `design.md` §5.3 are `[COMPUTED]` by an
  independent Python reimplementation, not yet reproduced from `computeJacobian`.
- **Hardware vs off-robot.** Everything in this package has run only off-robot. FK is verified
  analytically at q = 0; the hardware FK-vs-Kortex comparison is pending (Step 3). Joint
  velocity and position limits are `[SPEC]` from the Kortex actuator spec and User Guide, not
  confirmed by hardware test. Whether the arm's firmware enforces position limits itself in
  low-level servoing is `[UNVERIFIED]` — the filter is assumed to be the only enforcement.
- **Decided vs open.** Engineered clipping is the decision (D-07; a formal barrier certificate
  is future work). Open: D-01 observation width (38 v1, 39 if a recovery phase is adopted);
  D-04 manifest mechanism — `SafetyBounds` defaults are header literals until it lands; D-09
  final bound values (blocked on the trained action distribution); D-27 joint-limit values and
  sentinels duplicated between `KinovaKinematics.cpp` and `SafetyFilter.hpp` and drifted at the
  4th decimal; production λ (provisional 0.05, set by the λ sweep); manipulability metric (D-20).
- **Stated precondition.** Site 1 clips a desired normal force that only means something if the
  target arm carries a mounted, calibrated external 6-axis wrist F/T sensor. An arm without one
  is out of scope for this stack.
- **Claim boundary.** This package supports *architectural portability*: a per-robot adapter
  (DH table, λ, bounds) below a fixed contract. It is not hardware-agnostic — λ must be re-tuned
  per arm (`design.md` §5.3), and bounds are per-robot.
- **Numerical, not analytic, Jacobian** — forward finite difference, eps = 1e-6 `[DESIGN]`.
  Measured fast enough; an analytic Jacobian is the fallback only if a timing problem appears.
- **Single scalar λ is dimensionally inconsistent** across the m/rad and dimensionless Jacobian
  rows. Documented, not fixed in v1 (`design.md` §5.3).
- **Site 2 frame mismatch.** `clipCartesian` bounds a task-frame twist but tests the workspace
  box on a base-frame position. Correct only while the two frames are near axis-aligned
  (horizontal-surface insertion) `[DESIGN]` — re-verify if the task frame rotates.
- **`pos_lookahead_dt` = 1 ms is a single-step lookahead, not a stopping horizon.** Under the
  measured first-order servo lag the stopping distance is `v·τ`, τ = 15.7 ms `[MEASURED, joint 7
  only; UNVERIFIED for joints 2/4/6]`. Rename to `stop_horizon_s` and set to τ is an open item.
- **No joint velocity limits inside the IK** — deliberate; `jointVelocityDLS` is unclamped so
  that the one auditable limiter is Site 3.
- **`solveIK` is offline only.** Dynamic-size Eigen, explicit inverse, `std::cerr` — none of
  that is acceptable on the 1 kHz path and none of it runs there.

---

## Design notes

- [`design.md`](design.md) — derivations (DLS, BOUND-1, λ selection), numerical method,
  failure modes, testing, measured results, open decisions.
- [`safety_filter.md`](safety_filter.md) — the three sites, two bound classes, counter
  invariants, benchmark protocol.

---

## Repository layout

```
kinova_kinematics/
├── include/kinova_kinematics/
│   ├── KinovaKinematics.hpp        # FK, Jacobian, DLS, solveIK, manipulability
│   └── SafetyFilter.hpp            # three clip sites, SafetyBounds, InterventionCounter
├── src/
│   ├── KinovaKinematics.cpp
│   └── SafetyFilter.cpp
├── tests/
│   ├── test_fk.cpp                 # FK/Jacobian/IK smoke, own main()
│   ├── test_dls.cpp                # 17 gtest cases: DLS core, guards, manipulability
│   ├── test_safety_filter.cpp      # 28 gtest cases: three sites + counter
│   ├── benchmark_dls.cpp           # IK latency → CSV
│   └── benchmark_safety_filter.cpp # filter latency, 4 scenarios, counter self-checks → CSV
├── CMakeLists.txt
├── package.xml
├── design.md
├── safety_filter.md
└── README.md
```

---

## Status

**Done.** FK, numerical Jacobian, `solveDLS` / `jointVelocityDLS`, `manipulability()`,
`solveIK` (offline), three-site `SafetyFilter` with `InterventionCounter`. 
green off-robot (45 gtest cases in two binaries; `colcon test-result` reports 47,
which includes the two binary-level CTest entries). 
IK and filter latency benchmarked (Steps 6.2, 6.4 closed 4 Sep 2026).

**In progress.** Integration into the 1 kHz loop (kinova-wrapper, Block A Step 1); λ sweep.

**Open.** D-01, D-04, D-09, D-20, D-27; hardware FK-vs-Kortex check (Step 3); τ on joints
2/4/6 (Step 3); clock-floor conflict (Step 9); servo overshoot on joints 2/4/6 (Step 3); adaptive λ (decide
before a second robot enters MuJoCo).

---

*Advanced Biomechatronics and Locomotion Lab, Carleton University.*
