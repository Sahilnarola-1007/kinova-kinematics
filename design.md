# design.md — kinova_kinematics

Engineering record for the kinematics-and-clipping stage of the Idea 5 adapter. The README is
for deciding whether to care; this file is for the reader who has decided.

Provenance tags used throughout: `[MEASURED]` on this hardware / lab PC, run identified ·
`[COMPUTED]` from this library's own code or an independent reimplementation, no hardware ·
`[DERIVED]` reproducible by hand from a stated formula · `[SPEC]` vendor document ·
`[DESIGN]` a project decision · `[INFERRED]` a model that fits observed behaviour ·
`[UNVERIFIED]` believed, not checked — with what would check it.

---

## 1 · Scope and layer

**Layer.** Below the contract (adapter), 1 kHz path. Knows the robot; rewritten per port.

**In scope.** Forward kinematics (observation side), numerical Jacobian and damped-least-squares
differential IK (action side), a Yoshikawa singularity monitor, an offline iterative pose solver,
and the three-site safety filter with intervention counting.

**Out of scope, on purpose.** Task→base rotation, integration into `q_send`, the admittance
reflex, the 1 kHz cyclic loop and its watchdog. These live in `kinova_lowlevel`: `ControlCycle`
is the per-cycle chain, `step1_loop` the transport, pacing and logging.
Perception is out of scope for the whole portability argument: the socket is a fixed fixture and
lateral error is injected in software at reset.

**Convention.** Classical Denavit–Hartenberg, 8 frames (rows 0–7), radians throughout
`[MEASURED — convention confirmed by FK checks, §8.1]`. The public API takes and returns radians;
the deg↔rad seam is owned by the 1 kHz loop (D-25), and so is the wrapped↔signed seam (§4).

**Idea 5 goal, current wording.** One frozen force-conditioned policy performs a connector
insertion across kinematically different cobots driven through position/velocity interfaces.
Real validation on a Gen3 7-DOF; cross-body evidence primarily in MuJoCo, with a Gen3 6-DOF in
the lab as a stretch objective; zero target-robot data, zero gradient steps on the policy. The
claim is **architectural portability**, not hardware-agnosticism. Every target arm requires a
mounted, calibrated 6-axis wrist F/T sensor — a stated precondition, not an accessory.

---

## 2 · Requirements

| ID | Requirement | Definition of done | Status |
|---|---|---|---|
| R1 | FK of the tool tip in the base frame from 7 joint angles | Analytic checks at q = 0 and ±0.3 rad symmetry pass; hardware FK-vs-Kortex comparison recorded with the D-14 offset accounted for | Off-robot checks pass `[COMPUTED]`; hardware pending (Step 3) |
| R2 | Single-shot differential IK inside the 1 ms cycle | p99.9 ≤ 10 % of budget under SCHED_FIFO + pinned core, N ≥ 10⁶ | **Met**: p99.9 = 3.04 µs `[MEASURED]` (§8.2) |
| R3 | Bounded joint-velocity amplification near singularity | Derived bound BOUND-1 (§5.3) holds in tests at every λ/σ pair and on real Jacobians | **Met** (`test_dls.cpp`) |
| R4 | Fail-safe on non-finite input | Any NaN/inf in q, J, twist, λ ⇒ `ok = false`, `qdot` exactly zero, at every guard | **Met** (`test_dls.cpp`, `test_safety_filter.cpp`) |
| R5 | Three-site clipping with per-bound intervention counts (O3) | Each bound fires in isolation and is counted once; NOMINAL inputs never fire; non-finite is counted per site | **Met** (28 tests; benchmark self-checks). First hardware runs in the loop: §8.6 |
| R6 | Builds and tests with no robot and no vendor SDK | `colcon build` + `colcon test` on a machine with only Eigen3 | **Met** |
| R7 | Filter cost never a reason to weaken a bound | Worst p99.9 ≤ 1 % of budget | **Met**: ≤ 40 ns `[MEASURED]` (§8.3) |

---

## 3 · Data flow

```
  OBSERVATION SIDE (measured, flows up)        ACTION SIDE (commanded, flows down)
  ─────────────────────────────────────        ──────────────────────────────────────────────
  BaseCyclic feedback, 1 kHz                   policy action [F*n, vx, vy, wx, wy, wz]  (~30 Hz)
     │ q_meas [deg, wrapped] → [rad, signed]       │ F*n tool-z [N]; motion in TASK frame
     │ (loop, D-25)                                │
     ▼                                              ▼
  computeFK(q_meas) ─► T_base [m]              SafetyFilter Site 1  clipForce(F*n)         THIS PKG
     │                                              ▼
     ├─ getPosition → p_base [m]               admittance reflex (PI on −fz) → vz         (kinova_lowlevel)
     └─ getRotation → R, cols 0–1 = 6D obs          ▼
        NO Gram-Schmidt: nothing is ever       assemble twist_task [vx vy vz wx wy wz]
        reconstructed from the 6D vector            ▼
     │                                         SafetyFilter Site 2  clipCartesian(twist_task,
     ▼                                              │                            p_base)    THIS PKG
  task-frame pose / velocity built by              ▼  vz excluded from the scaling norms
  the loop from T_base and the latched          caller rotates task → base (rotation only)
  task frame  (kinova-wrapper)                      ▼
     │                                         jointVelocityDLS(q_meas, twist_base, λ)      THIS PKG
     ▼                                              │  J at MEASURED q; (JJᵀ+λ²I) LDLᵀ; q̇ = Jᵀy
  ObservationPublisher, 30 Hz                       ▼  q̇ [rad/s]
  obs = 38 (v1, pending D-01 —                 SafetyFilter Site 3  clipJoint(q̇, q_send)   THIS PKG
        39 if recovery phase adopted)               │  uniform scale; single-step position refusal
                                                    ▼  q̇_safe
                                               q_send = q_send_prev + q̇_safe·dt  (commanded anchor, P19)
                                                    ▼
                                               wrap to [0°, 360°) → Refresh() → arm   (kinova_lowlevel)

  Dotted dependency: q_meas (OBS) is also the point at which J is evaluated (ACTION). It is the
  only quantity that crosses sides, and it crosses as an input, never as a policy observation.
```

`solveIK` is not on either path. It is an iterative *pose* solver for offline planning and
test fixtures; its error-proportional damping (`λ = 0.5·‖e‖ + 1e-4`) is meaningful only when
there is a pose error, so it must never be carried into the differential solve.

---

## 4 · Interfaces

Authoritative frames and units. A frame or unit error is the most expensive class of bug in
this project.

| Function | Side | Input | Output | Frame | Units |
|---|---|---|---|---|---|
| `computeFK(q)` | OBS | `array<double,7>` | `Matrix4d` T | T in **base**, includes tool offset along the final frame's local z | rad → m |
| `getPosition(T)` / `getRotation(T)` | OBS | `Matrix4d` | `Vector3d` / `Matrix3d` | base | m / — |
| `computeJacobian(q)` | ACTION | `array<double,7>` | `Matrix<6,7>` | base | rows 0–2 m/rad; rows 3–5 dimensionless |
| `jointVelocityDLS(q_meas, twist, λ)` | ACTION | q [rad]; twist **base**, rows 0–2 [m/s], 3–5 [rad/s]; λ > 0 | `DlsResult` | q̇ joint space, order 1..7 | rad/s |
| `solveDLS(J, twist, λ)` | ACTION | J, twist, λ | `DlsResult` | as above | as above — public test seam (D-26) |
| `manipulability(J)` | monitor | `Matrix<6,7>` | `double` w ≥ 0, NaN if J non-finite | — | mixed (§5.3.4) |
| `solveIK(pose, guess, iters, tol_p, tol_o)` | offline | `Matrix4d` base, guess [rad] | `IKResult` | base | m, rad |
| `SafetyFilter::clipForce(F*n)` | ACTION, Site 1 | `double` | `ForceFilterResult` | tool z | N |
| `SafetyFilter::clipCartesian(twist_task, p_base)` | ACTION, Site 2 | `Matrix<6,1>`, `Vector3d` | `CartesianFilterResult` | twist **task**, position **base** | m/s, rad/s; m |
| `SafetyFilter::clipJoint(q̇, q_send)` | ACTION, Site 3 | `Matrix<7,1>` ×2 | `JointFilterResult` | joint space | rad/s; rad |

```cpp
struct DlsResult {
    Eigen::Matrix<double,7,1> qdot;            // [rad/s], joints 1..7; zero on failure
    Eigen::Matrix<double,6,1> twist_achieved;  // J·qdot, BASE [m/s; rad/s]; differs from the
                                               // command by the damping residual
    double lambda_used;                        // NaN until a solve ran (mixed units, §5.3.3)
    double manipulability;                     // NaN by design (D-20) — never computed here
    bool   ok;                                 // false ⇒ qdot is zero; do not command
};
```

**Three boundaries the caller owns, all known bug sources:**

1. **Task → base.** The policy and the reflex work in the task frame (latched at CONTACT entry).
   `jointVelocityDLS` takes a base-frame twist. Apply a **rotation only**, never the 4×4
   homogeneous transform: a twist is a vector pair, not a point, and applying the translation
   leaks a phantom offset.
2. **rad ↔ deg.** `q_send` is integrated in radians inside `ControlCycle`
   (`q_send += q̇·dt`, decided 30 Sep 2026) and converted to degrees once per cycle, where the
   command is staged; Kortex positions are degrees `[SPEC]`. Omitting the conversion produces
   motion 57× too slow that looks exactly like a damping problem.
3. **wrapped ↔ signed.** Kortex reports and accepts positions wrapped to [0°, 360°)
   `[SPEC — Kinova support, 4 Oct 2026]`. Site 3's position limits are symmetric about zero, so
   everything between the two ends of the loop — the measured angles, the `q_send` integrator,
   the filter — is in the signed convention, (−π, +π]. The loop converts every measured joint
   to signed on the way in and wraps every commanded joint on the way out. FK and the Jacobian
   are built from sines and cosines and are indifferent to the choice; `clipJoint` is not.
   Getting this wrong is silent (§6, §8.6).

**Not this library's job — and where it lives**

| Excluded | Why | Owner |
|---|---|---|
| Velocity clamping inside DLS | Would distort the twist direction and hide it from the one auditable safety layer | SafetyFilter Site 3 |
| Null-space projection | A secondary objective changes q̇ for reasons unrelated to the command and makes the λ sweep uninterpretable | Deferred, post-Phase-0 |
| Task→base rotation | Would give this library a dependency on task state | 1 kHz loop |
| Integration to position | `q_send` is the loop's integrator state; the library must be stateless w.r.t. the loop | 1 kHz loop |
| Joint-limit checking inside DLS | Same as clamping: one auditable layer | SafetyFilter Site 3 |

---

## 5 · Algorithms

Each subsection answers: inputs/outputs with frames and units; why this and not the obvious
alternative; how it fails; what number proves it and where that number was measured.

### 5.1 Forward kinematics

**In/out.** q [rad] → T ∈ SE(3), base frame, metres. Classical DH,
`T_i = Rot_z(θ)·Trans_z(d)·Trans_x(a)·Rot_x(α)`, row 0 is the fixed base frame, rows 1–7 carry
joint angles plus offsets, then a pure translation `tool_offset_z` along the final frame's local z.

| Frame | α | a | d [m] | θ offset |
|---|---|---|---|---|
| 0 (base) | π | 0 | 0 | 0 |
| 1 | π/2 | 0 | −0.2848 | 0 |
| 2 | π/2 | 0 | −0.0118 | π |
| 3 | π/2 | 0 | −0.4208 | π |
| 4 | π/2 | 0 | −0.0128 | π |
| 5 | π/2 | 0 | −0.3143 | π |
| 6 | π/2 | 0 | 0 | π |
| 7 | π | 0 | −0.1674 | π |
| tool | — | — | +0.113 along local z | — |

Link lengths `[SPEC — Kinova Gen3 7-DOF kinematic parameters; Sahil to record the exact User
Guide table/figure reference]`. Convention `[MEASURED]`: classical DH reproduces flange
z = 1.1873 m at q = 0 and passes symmetry checks (§8.1). Tool offset `[MEASURED]` 0.113 m
flange-to-tip for the current test handle (D-14); a constructor parameter, not a literal.

**Why classical, not modified DH.** Empirical: the classical chain matched the Kortex-reported
pose in the earlier high-level phase and the modified chain did not `[MEASURED, D-series entry —
run not identified here; see design_decisions.md]`.

**Fails when** a joint angle is non-finite (propagates to every element of T — caught upstream
in `jointVelocityDLS`, not in `computeFK`), or when the DH table is transcribed in degrees.

**Number.** z = 1.1873 m at q = 0 `[COMPUTED]`. Hardware comparison pending; it will carry ~7 mm
by construction (Kortex configured 0.12 m vs measured 0.113 m).

### 5.2 Jacobian — numerical

**In/out.** q [rad] → J ∈ ℝ^{6×7}, base frame; rows 0–2 [m/rad], rows 3–5 dimensionless.
Forward finite difference, `eps = 1e-6` rad `[DESIGN]`: 8 DH-chain evaluations per call, linear
part `(p₁ − p₀)/eps`, angular part the vee of `R₁R₀ᵀ` divided by `2·eps`.

**Why numerical.** It is fast enough — the whole `jointVelocityDLS` call is 1.90 µs mean
`[MEASURED]` — and it cannot disagree with the FK it is derived from. An analytic Jacobian is the
fallback only if a timing problem appears; none has.

**Fails when** `eps` is too small (cancellation) or too large (truncation) — 1e-6 is a
conventional middle `[DESIGN, not swept]`; and when q is non-finite.

**Number.** Analytic `Jv` vs central finite difference agree to 1.4e-10 at a random non-singular
pose `[COMPUTED — independent Python reimplementation, Sep 2026; not yet reproduced from
`computeJacobian` in C++]`.

### 5.3 Damped least-squares differential IK

#### 5.3.1 The problem

**In/out.** twist v (base, [m/s; rad/s]) and measured q_meas [rad] → q̇ [rad/s] with
`J(q_meas)·q̇ ≈ v`. J is 6×7: six equations, seven unknowns — **underdetermined**. There is an
infinite family of exact solutions; the question is which one, not how to approximate an
unreachable one. The right object is the minimum-norm solution `q̇ = Jᵀ(JJᵀ)⁻¹v`. The
least-squares form `(JᵀJ)⁻¹Jᵀ` is for the overdetermined case and is wrong here — `JᵀJ` is 7×7
and singular by construction.

#### 5.3.2 Why damping, and the derived bound

Each singular direction of the pseudoinverse is scaled by `1/σ`; near a singularity `σ → 0` and
a small commanded twist demands unbounded joint velocity — on hardware, a violent uncommanded
motion. Damping replaces the gain:

```
1/σ  →  g(σ) = σ / (σ² + λ²)          q̇ = Jᵀ (J Jᵀ + λ² I₆)⁻¹ v
```

`g → 0` as `σ → 0`: the solve stops trying to move in directions the arm cannot move in.

**BOUND-1** `[DERIVED]`. `g'(σ) = (λ² − σ²)/(σ² + λ²)²` is zero at `σ = λ`, giving
`g_max = 1/(2λ)`. Hence for any commanded twist

```
‖q̇‖ ≤ ‖v‖ / (2λ)                                                (BOUND-1)
```

Exact and reproducible by hand. Attained only when the twist aligns with a singular direction
whose σ equals λ. This is the number that makes Site 3's clamp threshold defensible rather than
arbitrary. Tests: `AmplificationCeilingIsAttainedWhenSigmaEqualsLambda` (tight),
`NeverExceedsTheAmplificationCeiling` (upper bound over λ × σ grid),
`RespectsTheAmplificationCeilingAcrossConfigurations` (real Jacobians).

#### 5.3.3 Choosing λ — the tension, and the mixed-units defect

A single constant λ cannot satisfy both requirements at once `[DERIVED]`:

| λ | Near singularity | Normal operation |
|---|---|---|
| Large (0.29) | Tight bound, safe | Heavy loss: 49 % at σ = 0.297, 78 % at σ = 0.152 (§5.3.4) |
| Small (0.05) | BOUND-1 gives ‖q̇‖ ≤ 10·‖v‖ — too loose to rely on alone | Accurate tracking |

The literature answer is variable damping keyed to σ_min, which needs an SVD every cycle —
not measured, and a different, more expensive operation than the LDLᵀ solve.

**Decision `[DESIGN]`.** λ is held small and constant for tracking; the velocity bound is
enforced downstream at Site 3. **Provisional λ = 0.05** — a starting point, replaced by the λ
sweep (§7 item 7). λ is a per-call parameter, validated inside `solveDLS`
(`!(λ > 0) || !isfinite(λ)` rejects 0, negative, NaN, ±∞). λ must be strictly positive: at
λ = 0, `JJᵀ` may be exactly singular and the factorisation is no longer guaranteed.

**Mixed units — a real defect, not fixed in v1.** `λ²I₆` adds the same scalar to all six
diagonal entries of `JJᵀ`, but the blocks of `JJᵀ` carry units m²/rad², m/rad and 1. The
effective damping on translation and on rotation differs by roughly the square of a
characteristic length; a λ tuned for translation is not the λ for rotation. Recorded so it is
never mistaken for a tuning problem. *Proposal, not in locked scope, deferred to 6.5:*
pre-multiply twist and J by `W = diag(1/L,1/L,1/L,1,1,1)`, L a characteristic arm length
(order 0.5 m). It changes the meaning of λ; decide after the sweep, never during it.
Dimensional inhomogeneity of the Jacobian and the "characteristic length" remedy are known in
the literature `[UNVERIFIED — Sahil must check primary sources before citing]`.

#### 5.3.4 λ is robot-specific — measured spectrum and link-scale sensitivity

λ is a threshold on σ, and the σ spectrum depends on the arm's geometry. σ has no consistent
unit (m/rad for translation-dominated directions, dimensionless for rotation-dominated), so
neither does λ — the §5.3.3 defect in the form that matters for tuning.

**Spectrum at `kHomeRad`** (the `test_dls.cpp` fixture; tool offset 0.113 m; reach 0.716 m)
`[COMPUTED — independent Python reimplementation of the DH table, Sep 2026; cross-checked by FK
z = 1.1873 at q = 0 and Jv agreement 1.4e-10; NOT yet reproduced from `computeJacobian`]`.
"% linear" = ‖u_v‖², the translational share of that direction; "lost" = λ²/(σ²+λ²).

| σ | % linear | lost at λ = 0.05 | lost at λ = 0.29 |
|---|---|---|---|
| 1.893 | 16 | 0.1 % | 2.3 % |
| 1.798 | 19 | 0.1 % | 2.5 % |
| 1.175 | 1 | 0.2 % | 5.7 % |
| 0.297 | 99 | 2.8 % | 49 % |
| 0.180 | 84 | 7.1 % | 72 % |
| 0.152 | 80 | 9.7 % | 78 % |

Damping engages on translation, not rotation, at this pose. `kHomeRad` is not the worst pose
seen — σ ≈ 0.105 has been observed at an ordinary pose `[COMPUTED, pose not recorded — record
it before citing]`.

**Link-scale sensitivity** — same solver, same λ, same joint configuration; only the DH `d`
values and the tool offset scaled by k `[COMPUTED, same provenance]`:

| k (k = 1 is the Gen3) | reach [m] | σ_min | σ_min(k)/σ_min(1) | k | linear share | rot share | lost at λ = 0.05 |
|---|---|---|---|---|---|---|---|
| 1.0 | 0.716 | 0.1519 | 1.0000 | 1.0000 | 80.1 % | 19.9 % | 9.8 % |
| 0.5 | 0.358 | 0.0823 | 0.5418 | 0.5000 | 94.2 % | 5.8 % | 27.0 % |
| 0.1 | 0.072 | 0.0169 | 0.1113 | 0.1000 | 99.8 % | 0.2 % | 89.7 % |

1. σ_min shrinks with the arm but not proportionally: scaling scales `Jv` by k and leaves `Jω`
   alone. "σ scales with link length" is false as a flat statement.
2. The singular direction re-orients toward pure translation. No closed-form correction; the SVD
   must be recomputed, not rescaled.
3. A fixed λ becomes destructive on a smaller arm: 9.8 % loss on the Gen3 becomes 89.7 % at
   tenth scale, at the same, non-singular configuration.

λ lives in the adapter, below the contract. A per-robot λ does not weaken the
architectural-portability claim — it is exactly the kind of robot-specific quantity the
contract exists to keep below the boundary.

**Procedure for a new robot** `[DERIVED from BOUND-1]`:
1. Measure the spectrum over poses the arm will visit; record the smallest
   translation-dominated σ.
2. Upper bound from accuracy: for tolerable loss L, `λ ≤ σ·sqrt(L/(1−L))`. Gen3: σ = 0.152,
   L = 10 % → λ ≤ 0.0507 — reproduces the shipped 0.05.
3. Lower bound from the velocity cap, *only if damping is the sole limiter*:
   `λ ≥ ‖v‖_max / (2·q̇_cap)`. Gen3: 0.559/(2·2.094) = 0.133.
4. Verify by sweep on the new arm.

Steps 2 and 3 are incompatible on the Gen3 (0.05 < 0.133) — deliberately. λ = 0.05 permits
‖q̇‖ ≤ 5.59 rad/s against a 2.094 rad/s cap, which is why Site 3 fires in normal operation
(‖q̇‖ = 3.39 rad/s observed at mid-workspace poses `[COMPUTED — pose not recorded]`) rather than
acting as last resort. **λ buys accuracy; Site 3 enforces the cap.**

*Deferred alternative:* adaptive λ keyed to an online σ_min estimate `[UNVERIFIED literature —
Sahil must check before citing any author]`. Cost of a per-cycle σ_min estimate at 1 kHz is not
measured. Forcing trigger: before a second robot enters MuJoCo.

#### 5.3.5 Numerical method — LDLᵀ, no inverse

```
A = J Jᵀ + λ² I₆        6×6, symmetric, positive definite for λ > 0   (xᵀAx = ‖Jᵀx‖² + λ²‖x‖² > 0)
solve A y = v           LDLᵀ, y ∈ ℝ⁶
q̇ = Jᵀ y                7×6 · 6×1
```

**Why 6×6, not 7×7.** `Jᵀ(JJᵀ+λ²I₆)⁻¹ = (JᵀJ+λ²I₇)⁻¹Jᵀ` algebraically; the left form factorises
a 6×6, the right a 7×7. The 6×6 is cheaper and natural for the underdetermined case.

**Why LDLᵀ, not an explicit inverse or Cholesky.** No inverse is ever formed — `A.inverse()`
computes more than is needed and discards conditioning information. Plain Cholesky (LLT) is
also valid since A is SPD and marginally faster; LDLᵀ is chosen for backward stability, for
graceful degradation if A becomes near-semidefinite, and because the pivot diagonal is a free
conditioning indicator for later `[DESIGN]`. Fewer flops than forming the damped
pseudoinverse — order 3–4× `[UNVERIFIED — count it or measure it before citing a factor]`.

**Fails when** λ is not > 0 and finite (guarded), J or v non-finite (guarded — whether Eigen's
`info()` flags a NaN-filled matrix is `[UNVERIFIED]`, hence explicit `allFinite()` on both ends),
LDLᵀ reports failure, or the solution y is non-finite. All four ⇒ `ok = false`, `qdot` zero.

**Number.** 1.90 µs mean, 3.04 µs p99.9 for the full `jointVelocityDLS` call `[MEASURED]` §8.2.
Fixed-size `Matrix<double,6,6>`, stack-allocated, `noalias()`; no heap on the path
`[INFERRED from the tight p99.9-vs-mean gap]`.

### 5.4 Manipulability monitor

`w = sqrt(det(JJᵀ))` (Yoshikawa), one 6×6 determinant, floored at 0 before the sqrt (rounding
can make the computed det a tiny negative near singularity). Returns **NaN** when J is
non-finite — "could not compute" — never 0.0, which is a real value meaning "exactly singular".
`DlsResult::manipulability` stays NaN by design (D-20): the loop calls `manipulability(J)` and
decides. Carries the §5.3.3 mixed-units wart; one small σ can be masked by large others.
Choice of the production metric is deferred to 6.5 on measured timing (§9).

### 5.5 Offline pose solver — `solveIK`

DLS iteration with error-proportional damping `λ = 0.5·‖e‖ + 1e-4`, orientation error by vee of
`R_target·R_currentᵀ`, plus a null-space joint-centring term `α·(I − J⁺J)·z₀`, α = 0.5.
Dynamic-size Eigen, explicit inverse, `std::cerr` on non-convergence — acceptable **only**
because it never runs on the 1 kHz path. `max_iterations ≤ 0` returns failure with −1.0 error
norms ("did not run"). Round-trip smoke: 100 random configurations, expect ≥ 90 to close within
1 mm `[COMPUTED, test_fk.cpp; count not recorded here]`.

### 5.6 Safety filter — three sites

Summary; full rationale, bounds table, counter invariants and protocol in `safety_filter.md`.

```
 Site 1  clipForce(F*n)          pre-reflex   scale |F*n| to f_n_max, sign preserved
 Site 2  clipCartesian(twist)    post-reflex  α on [vx vy] · β on [wx wy wz] · workspace refusal
                                              vz EXCLUDED from both norms (reflex owns it)
 Site 3  clipJoint(q̇, q_send)    post-DLS     UNIFORM scale of the 7-vector · single-step
                                              position refusal, directional (inward passes)
```

Why three sites and not one: force exists only before the reflex consumes it; the Cartesian twist
only after the reflex supplies vz; joint rates only after DLS. Why uniform scaling: per-joint
clipping changes the joint-space direction, hence the Cartesian direction the DLS just solved
for. Why vz is excluded: two limiters in series on the force-controlled axis make neither one
analysable. Decision D-07: engineered clipping with logged interventions; a formal barrier
certificate is future work — a swap behind the same interface, not a retrain.

Two preconditions the filter cannot check for itself, both met by the loop and both learned on
hardware (§8.6): `q_send` reaches Site 3 in the signed convention, and the start state lies
inside every refusal bound. The workspace box was widened on 7 Oct 2026 to contain the Home
pose; values in `safety_filter.md`.

---

## 6 · Failure modes

| Mode | Where detected | Behaviour |
|---|---|---|
| `q_meas` contains NaN/inf (dropped or partial BaseCyclic frame) | per-joint `isfinite` in `jointVelocityDLS` | `ok = false`, `qdot` zero, before it reaches the DH chain |
| `twist` or `J` non-finite | `allFinite()` in `solveDLS` | `ok = false`, `qdot` zero |
| λ ≤ 0, NaN, ±∞ (config typo) | guard in `solveDLS` | `ok = false`, `qdot` zero |
| LDLᵀ reports failure | Eigen `info() != Success` | `ok = false`, `qdot` zero |
| Solution y non-finite | `y.allFinite()` | `ok = false`, `qdot` zero |
| Near-singular configuration | `manipulability(J)` by the caller | **Not a failure.** Damping bounds it (BOUND-1); `ok` stays true; Site 3 caps; the caller decides |
| q outside joint limits | not checked in the IK | Site 3 position refusal, single-step lookahead |
| Non-finite input at any filter site | `isfinite` / `allFinite()` at each site | zero output, `NON_FINITE_REJECTED`, counted once per site; `pos_refused_mask = 0` |
| inf in a filter input | same guard | Caught explicitly — `inf > bound` is *true*, so an unguarded site would log a phantom intervention into the O3 count |
| `q_send` reaches Site 3 wrapped to [0, 2π) instead of signed | **not detectable in the filter** | Silent. A joint above π reads as far beyond its positive limit, so the directional refusal zeroes every positive rate and passes every negative one: the joint can only move one way. Observed on hardware 6 Oct 2026 (§8.6). Prevented at the loop's wrapped↔signed seam (§4) |
| Tool tip outside the workspace box | counted (`workspace_refused`); not an error | Outward motion is zeroed, inward passes, so a commanded oscillation becomes one-signed travel that reads as drift. Observed on hardware 6 Oct 2026 (§8.6) |

**`ok == false` means hold, not skip.** The loop's correct response is to keep `q_send` at its
previous value and still call `Refresh()` — skipping the call trips the firmware watchdog.

---

## 7 · Testing

All tests run off-robot; no mock is needed because there is no vendor dependency. `solveDLS`
being public (D-26) is what makes the linear algebra testable against an injected J.

**`test_dls.cpp` — 17 cases.** Fixtures are `[diag(s₀..s₅) | 0]` Jacobians, so singular values
are known exactly and one can be placed at σ = λ; a **duplicate-column** variant gives a
non-trivial null space so the minimum-norm claim is falsifiable rather than tautological.

1. Fixture self-check. 2. Pseudoinverse limit as λ → 0. 3. Minimum norm (even split across
duplicated joints). 4. Zero twist ⇒ exactly zero. 5. Linearity. 6. BOUND-1 attained at σ = λ.
7. BOUND-1 never exceeded over a λ × σ grid. 8. λ monotonicity (slower, larger residual).
9. Diagnostics describe the solve that ran. 10–12. Guards: λ (0, negative, NaN, **inf** — inf
passes `λ > 0` and is caught only by `isfinite`), non-finite J, non-finite twist.
13. Non-finite q at every joint. 14. BOUND-1 on real Jacobians at four configurations, σ_min
printed from a test-only SVD. 15–17. `manipulability`: known value 720, rank-deficient ⇒ 0.0,
non-finite ⇒ NaN.

**`test_safety_filter.cpp` — 28 cases.** Each bound in isolation; sign preservation at Site 1;
vz exclusion from both Site 2 norms; α/β independence; inward motion allowed at the workspace
faces and past joint limits (recovery); uniform-scaling ratio preservation; continuous-joint
sentinels never refuse; zero-input paths (HOLD runs them 1000×/s); non-finite rejection at all
three sites with telemetry pinned (no phantom bound counts); counter accumulation, `total()`,
`reset()`, and 1000 nominal cycles with zero interventions. Tolerance `kTol = 1e-9` — seven
orders above double round-off on O(1) SI quantities, eight below the smallest meaningful bound
difference `[DERIVED]`.

**Benchmarks** (`benchmark_dls.cpp`, `benchmark_safety_filter.cpp`) are not tests; the filter
benchmark asserts its counter invariants and exits 1 on disagreement.

**Fixture warning — q = 0 is a singularity** `[COMPUTED]`: rank 3, three singular values exactly
zero from this DH table. Valid as an FK fixture (z = 1.1873 m validates the table), invalid for
anything conditioning-related and invalid as the loop's initialisation pose.

**Pending:** 7. λ sweep — λ vs residual (nominal pose) and peak ‖q̇‖ (near-singular pose);
selects production λ. First hardware evidence on the tracking side is in §8.6: 15 % overshoot
on one axis at λ = 0.05 at the Home pose. Hardware FK-vs-Kortex (Step 3).

**Determinism.** No randomness in gtest cases; `test_fk.cpp` uses a fixed seed (42).

---

## 8 · Measured results

### 8.1 FK checks `[COMPUTED]`

Flange z = 1.1873 m at q = 0 (before tool offset); ±0.3 rad on joint 1 gives equal z and
mirrored x. Validates the DH table, not the Jacobian (q = 0 is singular).

### 8.2 Differential IK latency `[MEASURED]`

`benchmark_dls`, 4 Sep 2026, N = 10⁶, RelWithDebInfo (`-O2 -g -DNDEBUG -std=gnu++17`, verified
in `compile_commands.json` 4 Sep 2026), SCHED_FIFO `chrt -f 80`, `taskset -c 2` (P-core).

| mean | p50 | p99 | p99.9 | max (1 in 10⁶) | budget |
|---|---|---|---|---|---|
| 1.90 µs | 1.86 µs | 2.35 µs | **3.04 µs** | 94.4 µs | 1000 µs |

0.2 % of the cycle typically, 0.3 % at the 1-in-1000 tail. Jacobian/solve split **not measured**
— the expectation that the numerical Jacobian dominates is `[UNVERIFIED]`; split at 6.5 only if
a budget problem appears.

### 8.3 Safety filter latency `[MEASURED]`

`benchmark_safety_filter`, 4 Sep 2026, N = 10⁶ per scenario, same protocol as §8.2 (which is
what makes the two numbers comparable). Two runs of identical code, `run1 / run2` where they
differ, all ns:

| Scenario | mean | batched | p50 | p99 | p99.9 | max |
|---|---:|---:|---:|---:|---:|---:|
| NOMINAL | 21.0 / 20.7 | 10.0 / 10.1 | 21 | 27 / 26 | 35 / 33 | 5391 / 1623 |
| ALL_TRIP | 30.2 / 30.5 | 15.0 / 15.2 | 30 | 36 / 35 | 39 / 38 | 1311 / 964 |
| ALTERNATING | 25.6 | 12.6 / 12.5 | 28 / 27 | 35 / 34 | 40 / 36 | 1142 / 6851 |
| NON_FINITE | 12.6 | 5.1 / 5.0 | 13 | 14 | 21 / 26 | 785 / 7338 |

**Reportable:** worst p99.9 across scenarios and runs **≤ 40 ns** = 0.004 % of the cycle.
**True single-cycle cost is bracketed 5–30 ns**: `batched` (64 cycles per clock-read pair)
excludes clock overhead but runs back to back as the real loop never does — lower bound;
`mean` (per-iteration) pays one clock read the real loop never pays — upper bound. Report the
bracket, never either column alone.

**No worst-case scenario is named.** The label moved between the runs (ALTERNATING 40 ns, then
ALL_TRIP 38 ns) on identical code; the scenarios are 1–5 ns apart at single-digit-ns clock
resolution. The branch-predictor hypothesis behind ALTERNATING is neither supported nor refuted.
The `max` column is OS events, not filter work: the outlier landed on NOMINAL in one run and
NON_FINITE in the other.

**Counter invariants, same runs:** NOMINAL all counters 0; ALL_TRIP 1,000,000 on every bound,
mask 42; ALTERNATING 500,000 on every bound (confirms the table cycles), mask 42; NON_FINITE
`non_finite_rejected` = 3,000,000 = 3 × N.

### 8.4 Composed cost — estimate, not a result

Mean-sum 1.90 µs + ~0.02 µs ≈ 1.9 µs; tail-sum 3.04 µs + 0.04 µs ≈ 3.1 µs. The p99.9 of a sum
is not the sum of the p99.9s (that assumes perfectly correlated tails — conservative, not
measured). The reportable composed number comes from instrumenting the assembled loop end to
end (Step 8).

First end-to-end measurement `[MEASURED — hardware, `kinova_lowlevel` `step1_loop`, 7 Oct 2026,
runs 1007_1556 (25,000 cycles) and 1007_1616 (60,000 cycles), RelWithDebInfo, `chrt -f 80
taskset -c 2`]`:

| Quantity | p50 | p99.9 |
|---|---|---|
| `Refresh()` round trip | 324 / 286 µs | 789 / 812 µs |
| Adapter compute: cycle start to end of work, minus the round trip, differenced per cycle | 9.5 / 9.1 µs | 26 / 39 µs |

The compute figure is the whole per-cycle chain as it runs — FK, Jacobian, manipulability, DLS,
all three filter sites, integration, the gap check and the log record — so it is larger than
the 1.9 µs sum above, and it is about 1 % of the cycle. Missed deadlines: 0 of 25,000 and 3 of
60,000. These are microsecond-scale loop numbers; the clock-floor conflict of §8.5 concerns the
nanosecond-scale benchmarks.

### 8.5 Clock floor — conflicted, not cited

Readings of 9 ns and 13 ns exist across sources for the same measurement. Resolve before
publishing any timing number (Step 9). No figure in this document depends on it except the
5–30 ns bracket's interpretation, which holds under either value.

### 8.6 Hardware runs in the 1 kHz loop — 6–7 Oct 2026

Free space, no contact, reflex returning zero. `ScriptedSource`: vx 0.02 m/s at 0.2 Hz,
vy 0.02 m/s at 0.3 Hz, wx 0.05 rad/s at 0.15 Hz, wz 0.10 rad/s at 0.25 Hz, held over a
77-cycle policy period; task frame = base frame; λ = 0.05; start at the Home pose (joints
nominally 0, 15, 180, 230, 0, 55, 90° as Kortex reports them; w = 0.0326). The commanded tip path is
one-signed: x runs 0 → +3.18 cm → 0 and y runs 0 → +2.12 cm → 0, z is never commanded. Tip
positions below are `computeFK` on the logged measured joints. Run folders are under
`results/step3_hw/scripted_action/`.

**The 6 Oct run — two silent one-way refusals.** `1006_1503_repeat_poseA_25s_expect_fault`,
23,654 cycles before the arm faulted `[MEASURED]`:

| Axis | Commanded | Measured |
|---|---|---|
| x | 0 → +3.18 cm → 0 | oscillation on a falling baseline, −13.1 cm net |
| y | 0 → +2.12 cm → 0 | correct, returns each period |
| z | none | −7.6 cm |
| joint 4 `q_send` | — | never increased (0 of 23,653 cycles); −22.43° net; ended 4.6° past its limit |

Cause: the tip started 7 cm outside the workspace box in x, and joint 4's `q_send` was in the
wrapped convention, so both the Site 2 and the Site 3 refusal passed motion in one direction
only (§6; `safety_filter.md`, *Hardware findings*). It was **not** null-space drift, which had
been the working hypothesis: with both refusals removed the configuration barely moves (below).

**The 7 Oct runs — after the signed-convention fix and the wider box** `[MEASURED]`:

| Run | Cycles | Refusals (Site 3 mask) | Tip x range | Tip y range | Tip z range | Max command gap |
|---|---:|---:|---|---|---|---|
| `1007_1556_signed_ws080_poseA_25s` | 25,000 | 0 | −0.10…+3.08 cm | −0.01…+2.45 cm | −0.17…0.00 cm | 0.26° |
| `1007_1616_signed_ws080_poseA_60s` | 60,000 | 0 | −0.23…+3.08 cm | −0.01…+2.46 cm | −0.34…0.00 cm | 0.27° |

No HOLD cycles, no abort, no fault in either run.

What the two runs establish:

1. **Tracking error at λ = 0.05 is visible, and it is cross-axis.** The y peak is 2.45 cm
   against 2.12 cm commanded (+15 %); the x peak is 3.08 cm against 3.18 cm (−3 %) `[MEASURED]`.
   The same model at λ = 0.01 gives 2.13 cm and 3.17 cm `[COMPUTED — Python reimplementation]`,
   which attributes the error to the damping rather than to the servo or the action hold. This
   is the §5.3.3 accuracy cost, seen on hardware for the first time, and it is an input to the
   λ sweep.
2. **Open-loop drift is about 1 mm per 20 s.** Every scripted frequency completes whole periods
   in 20 s, so the tip should be back at its start at cycles 20,000, 40,000 and 60,000. In the
   60 s run it was at (−0.08, 0.00, −0.10), (−0.15, 0.00, −0.19) and (−0.24, −0.01, −0.29) cm
   `[MEASURED]`: linear, about 0.08 cm in x and 0.10 cm in z per 20 s. Orientation error
   against the start grows the same way, 0.21° at 20 s and 0.62° at 60 s. The chain has no
   task-space position feedback, so this accumulates.
3. **The redundancy does not wander.** After 60 s every joint is within 0.50° of where it
   started `[MEASURED]`. The minimum-norm solution with no null-space term (§4) did not
   reconfigure the arm over this motion.
4. **The kinematic chain behaves as an independent model predicts.** A Python reimplementation
   of FK, the finite-difference Jacobian, the DLS solve, the filter and a first-order servo
   (τ = 15.7 ms) reproduces all three logged runs: tip within 0.08 cm and joint 4 within 0.07°
   over every cycle `[COMPUTED vs MEASURED]`. For the two 7 Oct runs the ranges in the table
   above were predicted before the arm moved.
5. **Continuous joints crossed the 0°/360° command seam without incident.** Joints 1 and 5 sit
   at 0° at this pose; their wrapped command crossed the seam 25 and 29 times in the 60 s run
   and the command-to-measured gap peaked at 0.27° `[MEASURED]`. At these speeds the arm
   takes the short way round.

### 8.7 Verification ledger

| Claim | Basis |
|---|---|
| Classical DH is correct for the Gen3 | `[COMPUTED]` z = 1.1873 m at q = 0; symmetry. Hardware comparison pending (Step 3, D-14) |
| BOUND-1, λ selection formulae | `[DERIVED]` §5.3.2, §5.3.4 |
| IK mean 1.90 µs / p99.9 3.04 µs | `[MEASURED]` §8.2 |
| Filter worst p99.9 ≤ 40 ns; cost 5–30 ns | `[MEASURED]` bound; bracket from two bounds §8.3 |
| Non-finite guard live at all three sites; mask 42; NOMINAL silent | `[MEASURED]` counter self-checks §8.3 |
| No heap on the 1 kHz path | `[INFERRED]` from tail tightness; fixed-size Eigen throughout |
| Chain runs on the arm inside the 1 kHz loop; 60,000 cycles of scripted motion, zero refusals, no fault | `[MEASURED]` §8.6 |
| Adapter compute p50 ≈ 9 µs, p99.9 ≤ 39 µs on hardware | `[MEASURED]` §8.4 |
| FK, Jacobian and DLS agree with an independent model on hardware, tip within 0.08 cm | `[COMPUTED vs MEASURED]` §8.6 |
| Firmware does not enforce joint position limits in low-level servoing | `[SPEC]` Kinova support, 4 Oct 2026; consistent with the 6 Oct run §8.6 |
| Kortex positions are wrapped to [0°, 360°); commands must be wrapped before sending | `[SPEC]` Kinova support, 4 Oct 2026 |
| Joint velocity limits 120 / 200 °/s | `[SPEC]` Kortex actuator specification, Aug 2026; not confirmed by hardware test |
| Joint position limits ±2.2515 / 2.5800 / 2.0996 rad | `[SPEC]` Kinova Gen3 User Guide Table 39 |
| High-level library limits (Tables 40–41) not applied in low-level servoing | `[SPEC]` User Guide reading; `[UNVERIFIED]` by test |
| σ spectrum, link-scale table | `[COMPUTED]` independent reimplementation; not yet reproduced in C++ |
| q = 0 is rank 3 | `[COMPUTED]` |
| Numerical Jacobian dominates per-call cost | `[UNVERIFIED]` — split not measured |
| Literature references (variable damping, characteristic length, hybrid force/motion) | `[UNVERIFIED — Sahil must check]`; no author or venue is cited here on purpose |

---

## 9 · Open decisions

| ID / item | Gates | What settles it |
|---|---|---|
| D-01 obs width 38 vs 39 | MuJoCo environment build | Decision with Prof. Ahmadi on the recovery phase |
| D-04 manifest mechanism | Sourcing `SafetyBounds` and joint limits from one place | Block A Step 6 |
| D-07 CBF vs engineered clipping | Decided: clipping. Barrier certificate is future work | Prof. Ahmadi, only if reopened |
| D-09 final bound values | O3 being meaningful | Trained action distribution |
| D-14 tool offset | Hardware FK check | Step 3 records FK-vs-Kortex with the 7 mm accounted for |
| D-20 manipulability metric | Singularity monitor in the loop | Wall-clock cost of each candidate against the remaining budget (§5.4) — measure before choosing |
| D-27 joint-limit duplication | Manifest schema; `pos_refused_mask` invariant | Single source of truth. As of 7 Oct 2026 both copies carry the same values and ±1e9 sentinels, but they are still two copies. The sentinel choice still matters for the manifest: ±2π and ±1e9 are **not equivalent** — 2π refuses at 6.28 rad, 1e9 never refuses — and a manifest adopting ±2π silently changes the mask from 42 to 127 |
| Production λ | Hardware integration | λ sweep (§7 item 7). Hardware evidence at λ = 0.05: +15 % on one axis at the Home pose (§8.6) |
| Adaptive λ | Second robot in MuJoCo | Measure a per-cycle σ_min estimate's cost first |
| Position refusal: is bounding `q_send` enough? | Whether Site 3 needs a margin | Step 3 overshoot measurement on joints 2/4/6: peak `q_meas − q_send` after a step, several speeds. If zero, `v·dt` on `q_send` is correct and the `stop_horizon_s` rename is dropped; otherwise add margin ε to `SafetyBounds` (with D-04). τ = 15.7 ms is the lag, not the fix — a τ horizon applies only to a check made against `q_meas` |
| Clock-floor conflict | Any published timing number | Re-measure under one protocol (Step 9) |
| Sustained-intervention flag | Reflex integrator divergence under sustained scaling | Undecided whether the filter raises a flag or only counts |
| Per-cycle logging of Site 2 refusals | Telling a workspace refusal from drift once the reflex drives vz | Undecided. Site 3's mask is logged per cycle; Site 2 is only counted |
| Workspace box values | O3 being meaningful; the start pose lying inside the box | Provisional box set 7 Oct 2026 to contain the Home pose; final values with D-09 |

**Closed since the last revision**

| Item | Outcome |
|---|---|
| `IKResult::joint_states` fill on failure | Keep the last iterate (decided 11 Sep 2026): `solveIK` is offline and never commanded, and the last iterate shows where the solver stalled |
| Firmware position-limit enforcement in low-level servoing | Not enforced `[SPEC — Kinova support, 4 Oct 2026]`. Site 3 is the sole guard |
| Does the minimum-norm solution drift in the null space over the scripted motion? | No: joints within 0.50° of the start after 60 s (§8.6). A null-space term stays deferred |

---

## 10 · Limitations

- Hardware exposure so far is free-space scripted motion from one pose (§8.6). Still pending:
  the FK-vs-Kortex comparison, τ on the limited joints, servo overshoot, and anything in
  contact.
- The Jacobian is numerical; `eps` is unswept.
- One scalar λ across mixed-unit rows; λ is per-robot and must be re-tuned on every port.
- The composed loop cost is measured on hardware at microsecond resolution (§8.4); the
  sum-of-benchmarks figure remains an estimate and the nanosecond clock floor is conflicted.
- Site 2 mixes a task-frame twist with a base-frame position; valid only near axis alignment.
- Site 3 bounds the commanded setpoint, not the measured joint. Lag alone cannot cause a
  violation; servo overshoot could, and is not measured (Step 3).
- `SafetyBounds` defaults are header literals awaiting D-04/D-09; joint limits are duplicated
  (D-27).
- Refusals are directional, so a state that starts outside a refusal bound moves one way only,
  silently. The filter relies on the caller for a start state inside the bounds and for the
  signed angle convention at Site 3.
- No task-space position feedback: the tip drifts about 1 mm per 20 s under scripted motion
  (§8.6).
- λ = 0.05 costs up to 15 % tracking error on one axis at the Home pose (§8.6).
- The σ-spectrum and link-scale tables come from a Python reimplementation, not from this C++.
- No literature is cited with authors here because none has been read in primary form.
- Precondition inherited from the stack: an external, calibrated 6-axis wrist F/T sensor.

---

## 11 · Changelog

| Date | Change |
|---|---|
| 22 Aug 2026 | FK, Jacobian, `solveIK` implemented; classical DH confirmed; D-14 tool-offset question opened |
| Aug 2026 | `solveDLS` / `jointVelocityDLS` (Step 6.1); joint velocity limits from Kortex spec into `SafetyBounds` |
| 4 Sep 2026 | Step 6.2 IK benchmark and Step 6.4 filter benchmark closed; counter invariants verified over two runs; build flags verified |
| Sep 2026 | Tool offset made a constructor parameter (D-14 bookkeeping); `manipulability()` monitor added; `max_iterations ≤ 0` guard added to `solveIK` |
| 11 Sep 2026 | Documentation audited against `REPO_DOC_AUDIT_SPEC.md` v1.0: restructured to the required skeleton, provenance tags applied, clock floor withdrawn, composed cost relabelled as estimate |
| 4 Oct 2026 | Kinova support answers recorded: joint limits not applied in low-level servoing; positions reported wrapped to [0°, 360°); commands must be wrapped |
| 6–7 Oct 2026 | First scripted-motion hardware runs in the 1 kHz loop (`kinova_lowlevel`). Two silent one-way refusals found and removed: wrapped `q_send` at Site 3, tip outside the workspace box at Site 2. Workspace box widened. 25 s and 60 s runs clean. §4 boundary list, §6, §8.4, §8.6, §9 and §10 updated; null-space-drift hypothesis withdrawn |

---

*Advanced Biomechatronics and Locomotion Lab, Carleton University.*
