/**
 * @file test_safety_filter.cpp
 * @brief Unit tests for SafetyFilter — three clip sites + intervention counter.
 *        Mock-safe: pure computation, no Kortex, no hardware. Runs in CI.
 *
 * Assertion style: where the exact formula is an implementation choice, the test
 * pins the RELATIONSHIP (e.g. value == scale * input) rather than the formula, so
 * a legal refactor does not break the suite. Where a structural decision is being
 * guarded (vz exclusion, uniform scaling, directional refusal) the test is exact.
 *
 * @author Sahil Narola
 * @date   August 2026
 */

#include <gtest/gtest.h>
#include <kinova_kinematics/SafetyFilter.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace
{

/// Absolute tolerance for all scale / velocity / force comparisons.
///
/// Justification: every quantity under test is O(1) in SI units. The longest
/// numeric path is norm -> divide -> multiply, i.e. a sqrt and two rounded ops,
/// so the true error is a few ULP of double (~1e-16 relative, ~1e-16 absolute at
/// this magnitude). 1e-9 leaves seven orders of margin — enough to survive
/// compiler reassociation under -O2 and FMA contraction, still eight orders
/// tighter than the smallest physically meaningful difference in these bounds
/// (1e-4 rad/s). A failure at 1e-9 is a logic error, never a rounding artefact.
constexpr double kTol = 1e-9;

using Twist6 = Eigen::Matrix<double, 6, 1>;
using Joint7 = Eigen::Matrix<double, 7, 1>;

Twist6 makeTwist(double vx, double vy, double vz,
                 double wx, double wy, double wz)
{
    Twist6 t;
    t << vx, vy, vz, wx, wy, wz;
    return t;
}

Joint7 makeJoint(double j0, double j1, double j2, double j3,
                 double j4, double j5, double j6)
{
    Joint7 q;
    q << j0, j1, j2, j3, j4, j5, j6;
    return q;
}
}

/// Reason arrays are unordered — never index them, search them.
template <std::size_t N>
bool hasReason(const std::array<FilterReason, N>& reasons, FilterReason want)
{
    for (const FilterReason r : reasons)
    {
        if (r == want) { return true; }
    }
    return false;
}

class SafetyFilterTest : public ::testing::Test
{
protected:
    /// Header defaults. Mutate in a test BEFORE calling make().
    SafetyBounds bounds_;

    SafetyFilter make() const { return SafetyFilter(bounds_); }

    /// A point comfortably inside the default workspace box, base frame [m].
    static Eigen::Vector3d insideBox() { return Eigen::Vector3d(0.0, 0.0, 0.35); }

    /// NOTE (frame): clipCartesian takes a TASK-frame twist and a BASE-frame
    /// position. Every workspace test below assumes the latched task->base
    /// rotation is identity, which is what makes component-wise assertions on
    /// the twist meaningful. This mirrors the known limitation recorded in
    /// safety_filter.md; it is not a claim that the two frames are the same.
};

// ─── Site 1: clipForce ───────────────────────────────────────────────────────

TEST_F(SafetyFilterTest, ForceWithinBoundPassesUnchanged)
{
    SafetyFilter f = make();
    const double f_in = 0.5 * bounds_.f_n_max;

    const ForceFilterResult r = f.clipForce(f_in);

    EXPECT_NEAR(r.value, f_in, kTol);
    EXPECT_NEAR(r.scale, 1.0, kTol);
    EXPECT_EQ(r.reason, FilterReason::PASS);
    EXPECT_EQ(f.interventions().force_scaled, 0u);
}

TEST_F(SafetyFilterTest, ForceAboveBoundClampsToBound)
{
    SafetyFilter f = make();
    const double f_in = 3.0 * bounds_.f_n_max;

    const ForceFilterResult r = f.clipForce(f_in);

    EXPECT_NEAR(r.value, bounds_.f_n_max, kTol);
    EXPECT_LT(r.scale, 1.0);
    // Relationship, not formula: whatever scale means, it must reproduce value.
    EXPECT_NEAR(r.value, r.scale * f_in, kTol);
    EXPECT_EQ(r.reason, FilterReason::FORCE_SCALED);
    EXPECT_EQ(f.interventions().force_scaled, 1u);
}

TEST_F(SafetyFilterTest, ForceClampPreservesSign)
{
    SafetyFilter f = make();
    const double f_in = -3.0 * bounds_.f_n_max;

    const ForceFilterResult r = f.clipForce(f_in);

    EXPECT_NEAR(r.value, -bounds_.f_n_max, kTol);
    EXPECT_LT(r.value, 0.0) << "sign flipped: a pull-off command became a press";
    EXPECT_EQ(r.reason, FilterReason::FORCE_SCALED);
}

TEST_F(SafetyFilterTest, ForceExactlyAtBoundIsNotAnIntervention)
{
    // Clamping a value to itself changes nothing, so it is not an intervention.
    // If this fails the .cpp uses >= instead of >. Pick one, fix the other side,
    // and record the choice — O3 counts depend on it.
    SafetyFilter f = make();

    const ForceFilterResult r = f.clipForce(bounds_.f_n_max);

    EXPECT_NEAR(r.value, bounds_.f_n_max, kTol);
    EXPECT_NEAR(r.scale, 1.0, kTol);
    EXPECT_EQ(r.reason, FilterReason::PASS);
    EXPECT_EQ(f.interventions().force_scaled, 0u);
}

TEST_F(SafetyFilterTest, ForceZeroInputDoesNotDivideByZero)
{
    // Fires on every HOLD cycle and every cycle before the policy produces
    // output — this is a hot path, not an edge case. A zero request takes the
    // mag <= f_n_max branch (no divide), so it passes through as a clean zero.
    // No isfinite guard here: the EXPECT_NEAR checks below already fail on any
    // NaN/inf (every comparison against NaN is false), so a separate finiteness
    // assertion would only restate them. Non-finite INPUT is covered by
    // ForceNonFiniteInputIsRejected, where finiteness is the actual contract.
    SafetyFilter f = make();

    const ForceFilterResult r = f.clipForce(0.0);

    EXPECT_NEAR(r.value, 0.0, kTol);
    EXPECT_NEAR(r.scale, 1.0, kTol);
    EXPECT_EQ(r.reason, FilterReason::PASS);
    EXPECT_EQ(f.interventions().total(), 0u);
}

TEST_F(SafetyFilterTest, ForceNonFiniteInputIsRejected)
{
    // A NaN reaching Kortex is undefined arm behaviour. The declared failure
    // mode for the 1 kHz path is "arm holds", so a non-finite request must
    // become a finite, zero command.
    SafetyFilter f = make();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    const ForceFilterResult r_nan = f.clipForce(nan);
    EXPECT_TRUE(std::isfinite(r_nan.value)) << "NaN propagated through Site 1";
    EXPECT_NEAR(r_nan.value, 0.0, kTol);
    EXPECT_EQ(r_nan.reason, FilterReason::NON_FINITE_REJECTED);

    const ForceFilterResult r_inf = f.clipForce(inf);
    EXPECT_TRUE(std::isfinite(r_inf.value)) << "inf propagated through Site 1";
    EXPECT_LE(std::abs(r_inf.value), bounds_.f_n_max + kTol);
    EXPECT_EQ(r_inf.reason, FilterReason::NON_FINITE_REJECTED);

    // TELEMETRY. The original defect had two halves: mag <= f_n_max is false for
    // NaN, so the input fell into the clipping branch, which BOTH returned NaN
    // AND incremented force_scaled. Fixing only the output would leave O3
    // counting phantom force clips. Pin both halves.
    EXPECT_EQ(f.interventions().non_finite_rejected, 2u);
    EXPECT_EQ(f.interventions().force_scaled, 0u)
        << "a non-finite input was counted as a force clip; O3 telemetry is corrupt";
    EXPECT_EQ(f.interventions().total(), 2u);
}

// ─── Site 2: clipCartesian ───────────────────────────────────────────────────

TEST_F(SafetyFilterTest, TangentialWithinBoundPassesUnchanged)
{
    SafetyFilter f = make();
    const Twist6 t = makeTwist(0.05, 0.05, 0.0, 0.1, 0.1, 0.1);

    const CartesianFilterResult r = f.clipCartesian(t, insideBox());

    EXPECT_NEAR(r.alpha, 1.0, kTol);
    EXPECT_NEAR(r.beta, 1.0, kTol);
    EXPECT_NEAR((r.twist - t).norm(), 0.0, kTol);
    EXPECT_EQ(r.num_interventions, 0);
    EXPECT_EQ(f.interventions().total(), 0u);
}

TEST_F(SafetyFilterTest, TangentialScalingPreservesDirection)
{
    // 3-4-5 triangle: |[0.3, 0.4]| = 0.5, so alpha = 0.15 / 0.5 = 0.3.
    SafetyFilter f = make();
    const Twist6 t = makeTwist(0.3, 0.4, 0.0, 0.0, 0.0, 0.0);

    const CartesianFilterResult r = f.clipCartesian(t, insideBox());

    EXPECT_NEAR(r.alpha, 0.3, kTol);
    EXPECT_NEAR(r.twist(0), 0.09, kTol);
    EXPECT_NEAR(r.twist(1), 0.12, kTol);
    // The property that matters: heading unchanged, speed reduced to the bound.
    EXPECT_NEAR(r.twist(1) / r.twist(0), t(1) / t(0), kTol)
        << "slowing down also steered the tool";
    EXPECT_NEAR(std::hypot(r.twist(0), r.twist(1)), bounds_.v_tan_max, kTol);
    EXPECT_TRUE(hasReason(r.reasons, FilterReason::V_TAN_SCALED));
    EXPECT_EQ(f.interventions().v_tan_scaled, 1u);
}

TEST_F(SafetyFilterTest, NormalVelocityExcludedFromTangentialNorm)
{
    // STRUCTURAL — guards the vz-exclusion decision. The admittance reflex owns
    // the normal axis; if vz ever enters the tangential norm there are two
    // limiters in series on the force-controlled axis and neither is analysable.
    SafetyFilter f = make();

    // Case A: huge vz, tiny tangential.
    const Twist6 a = makeTwist(0.01, 0.0, 5.0, 0.0, 0.0, 0.0);
    const CartesianFilterResult ra = f.clipCartesian(a, insideBox());
    EXPECT_NEAR(ra.alpha, 1.0, kTol) << "vz leaked into the tangential norm";
    EXPECT_NEAR(ra.twist(2), 5.0, kTol) << "the reflex was throttled by Site 2";
    EXPECT_EQ(ra.num_interventions, 0);

    // Case B: tangential exactly at the bound, huge vz. Still no scaling.
    const Twist6 b = makeTwist(bounds_.v_tan_max, 0.0, 5.0, 0.0, 0.0, 0.0);
    const CartesianFilterResult rb = f.clipCartesian(b, insideBox());
    EXPECT_NEAR(rb.alpha, 1.0, kTol);
    EXPECT_NEAR(rb.twist(2), 5.0, kTol);
}

TEST_F(SafetyFilterTest, AngularScalingIndependentOfLinear)
{
    // COUPLING TEST. alpha and beta must come from two separate norms.
    // This does NOT claim the linear:angular ratio is preserved — it is not,
    // and cannot be, because vz is exempt from scaling by design. It claims
    // only that one block going over-bound does not throttle the other.
    SafetyFilter f = make();

    // Angular over, linear under => beta < 1, alpha == 1.
    const Twist6 a = makeTwist(0.01, 0.0, 0.0, 1.0, 0.0, 0.0);
    const CartesianFilterResult ra = f.clipCartesian(a, insideBox());
    EXPECT_NEAR(ra.alpha, 1.0, kTol) << "angular clip bled into the linear block";
    EXPECT_NEAR(ra.twist(0), 0.01, kTol);
    EXPECT_NEAR(ra.beta, bounds_.omega_max / 1.0, kTol);
    EXPECT_NEAR(ra.twist.tail<3>().norm(), bounds_.omega_max, kTol); // last 3 elements of twist
    EXPECT_TRUE(hasReason(ra.reasons, FilterReason::OMEGA_SCALED));
    EXPECT_FALSE(hasReason(ra.reasons, FilterReason::V_TAN_SCALED));

    // Mirror: linear over, angular under => alpha < 1, beta == 1.
    SafetyFilter g = make();
    const Twist6 b = makeTwist(0.3, 0.4, 0.0, 0.1, 0.0, 0.0);
    const CartesianFilterResult rb = g.clipCartesian(b, insideBox());
    EXPECT_LT(rb.alpha, 1.0);
    EXPECT_NEAR(rb.beta, 1.0, kTol) << "linear clip bled into the angular block";
    EXPECT_NEAR(rb.twist(3), 0.1, kTol);
    EXPECT_TRUE(hasReason(rb.reasons, FilterReason::V_TAN_SCALED));
    EXPECT_FALSE(hasReason(rb.reasons, FilterReason::OMEGA_SCALED));
}

TEST_F(SafetyFilterTest, ZeroTwistDoesNotDivideByZero)
{
    // Runs on every HOLD cycle: 1000 times per second of hold.
    SafetyFilter f = make();

    const CartesianFilterResult r = f.clipCartesian(Twist6::Zero(), insideBox());

    EXPECT_TRUE(r.twist.allFinite());
    EXPECT_NEAR(r.twist.norm(), 0.0, kTol);
    EXPECT_NEAR(r.alpha, 1.0, kTol);
    EXPECT_NEAR(r.beta, 1.0, kTol);
    EXPECT_EQ(r.num_interventions, 0);
    EXPECT_EQ(f.interventions().total(), 0u);
}

TEST_F(SafetyFilterTest, NonFiniteCartesianInputIsRejected)
{
    // Parity with Site 1: a NaN/inf twist, or a non-finite EE position gating
    // the workspace refusal, must hold the arm rather than reach Kortex. The
    // norm math cannot catch it — sqrt(NaN) and every over-bound comparison are
    // false/NaN, so an unguarded Site 2 sprays the NaN across the whole twist.
    SafetyFilter f = make();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    // NaN in a single twist component.
    const Twist6 t_nan = makeTwist(0.05, nan, 0.0, 0.0, 0.0, 0.0);
    const CartesianFilterResult r_nan = f.clipCartesian(t_nan, insideBox());
    EXPECT_TRUE(r_nan.twist.allFinite()) << "NaN propagated through Site 2";
    EXPECT_NEAR(r_nan.twist.norm(), 0.0, kTol) << "non-finite input must hold";
    EXPECT_TRUE(hasReason(r_nan.reasons, FilterReason::NON_FINITE_REJECTED));
    EXPECT_EQ(r_nan.num_interventions, 1) << "reject is one intervention, not zero";

    // inf in the angular block.
    const Twist6 t_inf = makeTwist(0.0, 0.0, 0.0, inf, 0.0, 0.0);
    const CartesianFilterResult r_inf = f.clipCartesian(t_inf, insideBox());
    EXPECT_TRUE(r_inf.twist.allFinite()) << "inf propagated through Site 2";
    EXPECT_NEAR(r_inf.twist.norm(), 0.0, kTol);
    EXPECT_TRUE(hasReason(r_inf.reasons, FilterReason::NON_FINITE_REJECTED));

    // Non-finite EE position (bad FK) with a finite twist is equally fatal.
    const Eigen::Vector3d bad_pos(0.0, nan, 0.35);
    const CartesianFilterResult r_pos =
        f.clipCartesian(makeTwist(0.05, 0.0, 0.0, 0.0, 0.0, 0.0), bad_pos);
    EXPECT_TRUE(r_pos.twist.allFinite()) << "NaN position propagated through Site 2";
    EXPECT_NEAR(r_pos.twist.norm(), 0.0, kTol);
    EXPECT_TRUE(hasReason(r_pos.reasons, FilterReason::NON_FINITE_REJECTED));

    // TELEMETRY. Three rejects, and NONE of them may be attributed to a bound.
    // omega_scaled is the sharp one: |[inf,0,0]| > omega_max is TRUE, so an
    // unguarded Site 2 would have logged the inf twist as a legitimate angular
    // clip — a real intervention that never happened, in the O3 count.
    EXPECT_EQ(f.interventions().non_finite_rejected, 3u);
    EXPECT_EQ(f.interventions().v_tan_scaled, 0u);
    EXPECT_EQ(f.interventions().omega_scaled, 0u)
        << "an inf twist was counted as an angular clip; O3 telemetry is corrupt";
    EXPECT_EQ(f.interventions().workspace_refused, 0u);
    EXPECT_EQ(f.interventions().total(), 3u);
}

TEST_F(SafetyFilterTest, WorkspaceRefusalZerosOnlyOutwardComponent)
{
    SafetyFilter f = make();
    const Eigen::Vector3d outside(bounds_.ws_x_max + 0.01, 0.0, 0.35);
    const Twist6 t = makeTwist(0.05, 0.05, 0.05, 0.0, 0.0, 0.0);

    const CartesianFilterResult r = f.clipCartesian(t, outside);

    EXPECT_NEAR(r.twist(0), 0.0, kTol) << "outward x not refused";
    EXPECT_NEAR(r.twist(1), 0.05, kTol) << "in-bounds y was modified; refusal must be per-axis";
    EXPECT_NEAR(r.twist(2), 0.05, kTol) << "in-bounds z was modified; refusal must be per-axis";    EXPECT_TRUE(hasReason(r.reasons, FilterReason::WORKSPACE_REFUSED));
    EXPECT_EQ(f.interventions().workspace_refused, 1u);
}

TEST_F(SafetyFilterTest, WorkspaceAllowsInwardMotionWhenOutsideBox)
{
    // RECOVERY. Without this the arm is trapped the first time it drifts out —
    // every command gets zeroed including the one that would rescue it.
    SafetyFilter f = make();

    // Past +x, driving back toward -x.
    const Eigen::Vector3d past_x(bounds_.ws_x_max + 0.01, 0.0, 0.35);
    const CartesianFilterResult rx =
        f.clipCartesian(makeTwist(-0.05, 0.0, 0.0, 0.0, 0.0, 0.0), past_x);
    EXPECT_NEAR(rx.twist(0), -0.05, kTol) << "inward motion refused; arm is stuck";
    EXPECT_FALSE(hasReason(rx.reasons, FilterReason::WORKSPACE_REFUSED));

    // Below z_min, driving back up. This is the table-collision case.
    const Eigen::Vector3d below_z(0.0, 0.0, bounds_.ws_z_min - 0.01);
    const CartesianFilterResult rz =
        f.clipCartesian(makeTwist(0.0, 0.0, 0.05, 0.0, 0.0, 0.0), below_z);
    EXPECT_NEAR(rz.twist(2), 0.05, kTol) << "cannot lift off the floor";
    EXPECT_FALSE(hasReason(rz.reasons, FilterReason::WORKSPACE_REFUSED));

    EXPECT_EQ(f.interventions().workspace_refused, 0u);
}

TEST_F(SafetyFilterTest, WorkspaceRefusalIsReportedSeparatelyFromScaling)
{
    // Two different problems in one cycle must be logged as two.
    SafetyFilter f = make();
    const Eigen::Vector3d outside(bounds_.ws_x_max + 0.01, 0.0, 0.35);
    const Twist6 t = makeTwist(0.3, 0.4, 0.0, 0.0, 0.0, 0.0);  // |v_tan| = 0.5

    const CartesianFilterResult r = f.clipCartesian(t, outside);

    EXPECT_TRUE(hasReason(r.reasons, FilterReason::V_TAN_SCALED));
    EXPECT_TRUE(hasReason(r.reasons, FilterReason::WORKSPACE_REFUSED));
    EXPECT_EQ(r.num_interventions, 2);
    EXPECT_EQ(f.interventions().v_tan_scaled, 1u);
    EXPECT_EQ(f.interventions().workspace_refused, 1u);

    // Order-independent: whichever runs first(either vel clipping or workspace refusal),
    // both designs must follow the rules.
    EXPECT_NEAR(r.twist(0), 0.0, kTol);
    EXPECT_LE(std::hypot(r.twist(0), r.twist(1)), bounds_.v_tan_max + kTol);
}

// ─── Site 3: clipJoint ───────────────────────────────────────────────────────

TEST_F(SafetyFilterTest, JointVelocityWithinCapsPassesUnchanged)
{
    SafetyFilter f = make();
    Joint7 qd;
    for (int i = 0; i < 7; ++i) { qd(i) = 0.5 * bounds_.joint_vel_max[static_cast<std::size_t>(i)]; }

    const JointFilterResult r = f.clipJoint(qd, Joint7::Zero());

    EXPECT_NEAR(r.vel_scale, 1.0, kTol);
    EXPECT_NEAR((r.qdot - qd).norm(), 0.0, kTol);
    EXPECT_EQ(r.pos_refused_mask, 0u);
    EXPECT_EQ(r.num_interventions, 0);
    EXPECT_EQ(f.interventions().total(), 0u);
}

TEST_F(SafetyFilterTest, UniformScalingUsesTightestJointRatio)
{
    // Joint 0 over by 2x (ratio 0.5), joint 4 over by 4x (ratio 0.25).
    // Both are continuous-rotation joints, so no position refusal can interfere.
    SafetyFilter f = make();
    Joint7 qd = Joint7::Zero();
    qd(0) = 2.0 * bounds_.joint_vel_max[0];
    qd(4) = 4.0 * bounds_.joint_vel_max[4];

    const JointFilterResult r = f.clipJoint(qd, Joint7::Zero());

    EXPECT_NEAR(r.vel_scale, 0.25, kTol) << "scale not set by the worst joint";
    EXPECT_NEAR(r.qdot(4), bounds_.joint_vel_max[4], kTol);
    EXPECT_NEAR(r.qdot(0), 0.25 * qd(0), kTol);
    EXPECT_LT(std::abs(r.qdot(0)), bounds_.joint_vel_max[0])
        << "the non-binding joint should end up well under its own cap";
    EXPECT_TRUE(hasReason(r.reasons, FilterReason::JOINT_VEL_SCALED));
    EXPECT_EQ(f.interventions().joint_vel_scaled, 1u);
}

TEST_F(SafetyFilterTest, UniformScalingPreservesJointSpaceDirection)
{
    // Per-joint clipping would satisfy the caps and silently change where the
    // end-effector goes. Uniform scaling changes only how fast it gets there.
    SafetyFilter f = make();
    const Joint7 qd = makeJoint(1.0, -2.0, 3.0, -4.0, 5.0, -6.0, 7.0);

    const JointFilterResult r = f.clipJoint(qd, Joint7::Zero());

    EXPECT_LT(r.vel_scale, 1.0);
    for (int i = 0; i < 7; ++i)
    {
        EXPECT_NEAR(r.qdot(i), r.vel_scale * qd(i), kTol)
            << "joint " << i << " was not scaled by the common factor";
        EXPECT_LE(std::abs(r.qdot(i)),
                  bounds_.joint_vel_max[static_cast<std::size_t>(i)] + kTol);
    }
    EXPECT_NEAR((r.qdot.normalized() - qd.normalized()).norm(), 0.0, kTol)
        << "joint-space direction changed; Cartesian direction is now wrong";
}

TEST_F(SafetyFilterTest, SmallActuatorCapIsHigherThanLargeActuatorCap)
{
    // Guards the actuator table against a copy-paste regression.
    // [SPEC] Kortex actuator spec: joints 1-4 = 120 deg/s, joints 5-7 = 200 deg/s.
    for (std::size_t i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(bounds_.joint_vel_max[i], 2.0944, 1e-4) << "large actuator " << i;
    }
    for (std::size_t i = 4; i < 7; ++i)
    {
        EXPECT_NEAR(bounds_.joint_vel_max[i], 3.4907, 1e-4) << "small actuator " << i;
    }
    EXPECT_GT(bounds_.joint_vel_max[4], bounds_.joint_vel_max[3]);

    // Position limits: only joints 2, 4, 6 (0-indexed 1, 3, 5) are bounded.
    // NOTE: these literals duplicate KinovaKinematics.cpp and already disagree
    // at the 4th digit. When the manifest lands, this test reads the manifest
    // and this comment goes away.
    for (const std::size_t i : {std::size_t{0}, std::size_t{2}, std::size_t{4}, std::size_t{6}})
    {
        EXPECT_GT(bounds_.joint_pos_max[i], 100.0) << "joint " << i << " must be continuous";
        EXPECT_LT(bounds_.joint_pos_min[i], -100.0);
    }
    EXPECT_NEAR(bounds_.joint_pos_max[1], 2.2515, 1e-4);
    EXPECT_NEAR(bounds_.joint_pos_max[3], 2.5800, 1e-4);
    EXPECT_NEAR(bounds_.joint_pos_max[5], 2.0996, 1e-4);
}

TEST_F(SafetyFilterTest, PositionRefusalZerosOnlyOffendingJoint)
{
    // Bit convention: bit j (0-indexed) = joint index j = physical joint j+1.
    // If the .cpp is 1-indexed, fix the .cpp — the arrays it indexes are 0-based.
    SafetyFilter f = make();

    Joint7 q_send = Joint7::Zero();
    q_send(1) = bounds_.joint_pos_max[1] - 0.0005;   // 0.5 millirad inside the limit

    Joint7 qd = Joint7::Constant(0.1);               // all well under their caps
    qd(1) = 2.0;                                     // under cap 2.0944, so no vel scaling
    // lookahead: q_send(1) + 2.0 * 0.001 = limit + 0.0015  ->  crosses

    const JointFilterResult r = f.clipJoint(qd, q_send);

    EXPECT_NEAR(r.vel_scale, 1.0, kTol) << "velocity scaling should not fire here";
    EXPECT_NEAR(r.qdot(1), 0.0, kTol) << "offending joint not stopped";
    for (int i = 0; i < 7; ++i)
    {
        if (i == 1) { continue; }
        EXPECT_NEAR(r.qdot(i), 0.1, kTol) << "joint " << i << " zeroed as collateral";
    }
    EXPECT_NE(r.pos_refused_mask & (1u << 1), 0u) << "mask bit 1 not set";
    EXPECT_EQ(r.pos_refused_mask, 1u << 1) << "extra joints reported as refused";
    EXPECT_TRUE(hasReason(r.reasons, FilterReason::JOINT_POS_REFUSED));
    EXPECT_EQ(f.interventions().joint_pos_refused, 1u);
}

TEST_F(SafetyFilterTest, PositionRefusalAllowsMotionAwayFromLimit)
{
    // RECOVERY. The arm can end up outside a limit by overshoot, by a bad seed
    // at startup, or by manual jogging before the run. If the check is not
    // directional, every command from that pose is zeroed and the joint is
    // stuck until someone power-cycles the arm.
    SafetyFilter f = make();

    // Past the upper limit, driving negative (back inside).
    Joint7 q_hi = Joint7::Zero();
    q_hi(1) = bounds_.joint_pos_max[1] + 0.05;
    Joint7 qd_in = Joint7::Zero();
    qd_in(1) = -1.0;

    const JointFilterResult r_hi = f.clipJoint(qd_in, q_hi);
    EXPECT_NEAR(r_hi.qdot(1), -1.0, kTol) << "recovery motion refused; joint is stuck";
    EXPECT_EQ(r_hi.pos_refused_mask, 0u);

    // Mirror on the lower limit, driving positive.
    Joint7 q_lo = Joint7::Zero();
    q_lo(5) = bounds_.joint_pos_min[5] - 0.05;
    Joint7 qd_out = Joint7::Zero();
    qd_out(5) = 1.0;

    const JointFilterResult r_lo = f.clipJoint(qd_out, q_lo);
    EXPECT_NEAR(r_lo.qdot(5), 1.0, kTol);
    EXPECT_EQ(r_lo.pos_refused_mask, 0u);

    EXPECT_EQ(f.interventions().joint_pos_refused, 0u);
}

TEST_F(SafetyFilterTest, ContinuousJointsNeverPositionRefuse)
{
    // Joints 1, 3, 5, 7 (0-indexed 0, 2, 4, 6) rotate without limit. A commanded
    // anchor that has integrated past 2*pi must not trip a phantom limit.
    SafetyFilter f = make();

    Joint7 q_send = Joint7::Zero();
    Joint7 qd     = Joint7::Zero();
    for (const int i : {0, 2, 4, 6})
    {
        q_send(i) = 100.0;   // ~16 full turns
        qd(i)     = 1.0;     // still driving further out
    }

    const JointFilterResult r = f.clipJoint(qd, q_send);

    EXPECT_EQ(r.pos_refused_mask, 0u) << "a continuous joint was position-refused";
    for (const int i : {0, 2, 4, 6})
    {
        EXPECT_NEAR(r.qdot(i), 1.0, kTol);
    }
    EXPECT_EQ(f.interventions().joint_pos_refused, 0u);
}

TEST_F(SafetyFilterTest, ZeroJointVelocityDoesNotDivideByZero)
{
    // The HOLD state commands exactly this, 1000 times a second.
    SafetyFilter f = make();

    const JointFilterResult r = f.clipJoint(Joint7::Zero(), Joint7::Zero());

    EXPECT_NEAR(r.qdot.norm(), 0.0, kTol);
    EXPECT_TRUE(std::isfinite(r.vel_scale));
    EXPECT_NEAR(r.vel_scale, 1.0, kTol);
    EXPECT_EQ(r.pos_refused_mask, 0u);
    EXPECT_EQ(r.num_interventions, 0);

    // Parked exactly on a limit with zero velocity is not a violation —
    // zero motion cannot cross anything.
    Joint7 q_at_limit = Joint7::Zero();
    q_at_limit(1) = bounds_.joint_pos_max[1];
    const JointFilterResult r2 = f.clipJoint(Joint7::Zero(), q_at_limit);
    EXPECT_EQ(r2.pos_refused_mask, 0u) << "refused a zero-velocity command";

    EXPECT_EQ(f.interventions().total(), 0u);
}

TEST_F(SafetyFilterTest, NonFiniteJointInputIsRejected)
{
    // A NaN/inf joint command reaching Kortex is undefined arm behaviour; the
    // declared failure mode is "hold", so a non-finite request must become a
    // finite, zero command. The bare velocity/position logic cannot catch this:
    // every comparison against NaN is false, so an unguarded filter passes it
    // straight through untouched. This test fails against a filter with no
    // isfinite guard.
    SafetyFilter f = make();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    // NaN in a single commanded velocity.
    Joint7 qd_nan = Joint7::Zero();
    qd_nan(3) = nan;
    const JointFilterResult r_nan = f.clipJoint(qd_nan, Joint7::Zero());
    EXPECT_TRUE(r_nan.qdot.allFinite()) << "NaN velocity propagated through Site 3";
    EXPECT_NEAR(r_nan.qdot.norm(), 0.0, kTol) << "non-finite input must hold";
    EXPECT_TRUE(hasReason(r_nan.reasons, FilterReason::NON_FINITE_REJECTED));
    EXPECT_EQ(r_nan.num_interventions, 1);
    // STRUCTURAL. mask 0, not all-ones: no joint hit a limit — the whole input
    // was garbage. A reader of the log must be able to tell "joint 4 reached its
    // limit" apart from "IK returned NaN", and the mask is where that shows.
    EXPECT_EQ(r_nan.pos_refused_mask, 0u)
        << "a non-finite reject was reported as a joint-limit refusal";

    // inf in a single commanded velocity.
    Joint7 qd_inf = Joint7::Zero();
    qd_inf(5) = inf;
    const JointFilterResult r_inf = f.clipJoint(qd_inf, Joint7::Zero());
    EXPECT_TRUE(r_inf.qdot.allFinite()) << "inf velocity propagated through Site 3";
    EXPECT_NEAR(r_inf.qdot.norm(), 0.0, kTol);
    EXPECT_TRUE(hasReason(r_inf.reasons, FilterReason::NON_FINITE_REJECTED));
    EXPECT_EQ(r_inf.pos_refused_mask, 0u);

    // A non-finite anchor (bad feedback or IK seed) with a finite command is
    // equally fatal: the lookahead q_send + q̇*dt becomes NaN and every limit
    // comparison silently passes.
    Joint7 q_bad = Joint7::Zero();
    q_bad(2) = nan;
    const JointFilterResult r_q = f.clipJoint(Joint7::Constant(0.1), q_bad);
    EXPECT_TRUE(r_q.qdot.allFinite()) << "NaN anchor propagated through Site 3";
    EXPECT_NEAR(r_q.qdot.norm(), 0.0, kTol);
    EXPECT_TRUE(hasReason(r_q.reasons, FilterReason::NON_FINITE_REJECTED));
    EXPECT_EQ(r_q.pos_refused_mask, 0u);

    // TELEMETRY. joint_vel_scaled is the sharp one here: |inf| > joint_vel_max[5]
    // is TRUE, so an unguarded Site 3 would have computed alpha = cap/inf = 0,
    // taken the alpha < 1 branch, and logged a legitimate velocity clip.
    EXPECT_EQ(f.interventions().non_finite_rejected, 3u);
    EXPECT_EQ(f.interventions().joint_vel_scaled, 0u)
        << "an inf velocity was counted as a joint-velocity clip; O3 is corrupt";
    EXPECT_EQ(f.interventions().joint_pos_refused, 0u);
    EXPECT_EQ(f.interventions().total(), 3u);
}

// ─── Intervention counter ────────────────────────────────────────────────────

TEST_F(SafetyFilterTest, CounterAccumulatesAcrossCalls)
{
    SafetyFilter f = make();
    constexpr int kCalls = 5;

    for (int i = 0; i < kCalls; ++i) { f.clipForce(100.0); }

    const InterventionCounter& c = f.interventions();
    EXPECT_EQ(c.force_scaled, static_cast<uint64_t>(kCalls));
    EXPECT_EQ(c.v_tan_scaled, 0u);
    EXPECT_EQ(c.omega_scaled, 0u);
    EXPECT_EQ(c.workspace_refused, 0u);
    EXPECT_EQ(c.joint_vel_scaled, 0u);
    EXPECT_EQ(c.joint_pos_refused, 0u);
    EXPECT_EQ(c.non_finite_rejected, 0u);
    EXPECT_EQ(c.total(), static_cast<uint64_t>(kCalls));
}

TEST_F(SafetyFilterTest, CounterTotalEqualsSumOfFields)
{
    // Trigger each bound exactly once, in isolation.
    SafetyFilter f = make();

    f.clipForce(100.0);                                                        // force
    f.clipCartesian(makeTwist(0.3, 0.4, 0.0, 0.0, 0.0, 0.0), insideBox());     // v_tan
    f.clipCartesian(makeTwist(0.0, 0.0, 0.0, 1.0, 0.0, 0.0), insideBox());     // omega
    f.clipCartesian(makeTwist(0.05, 0.0, 0.0, 0.0, 0.0, 0.0),
                    Eigen::Vector3d(bounds_.ws_x_max + 0.01, 0.0, 0.35));      // workspace

    Joint7 qd_fast = Joint7::Zero();
    qd_fast(0) = 10.0 * bounds_.joint_vel_max[0];
    f.clipJoint(qd_fast, Joint7::Zero());                                      // joint vel

    Joint7 q_near = Joint7::Zero();
    q_near(1) = bounds_.joint_pos_max[1] - 0.0005;
    Joint7 qd_out = Joint7::Zero();
    qd_out(1) = 2.0;
    f.clipJoint(qd_out, q_near);                                               // joint pos

    f.clipForce(std::numeric_limits<double>::quiet_NaN());                     // non-finite

    const InterventionCounter& c = f.interventions();
    EXPECT_EQ(c.force_scaled, 1u);
    EXPECT_EQ(c.v_tan_scaled, 1u);
    EXPECT_EQ(c.omega_scaled, 1u);
    EXPECT_EQ(c.workspace_refused, 1u);
    EXPECT_EQ(c.joint_vel_scaled, 1u);
    EXPECT_EQ(c.joint_pos_refused, 1u);
    EXPECT_EQ(c.non_finite_rejected, 1u);
    // If total() does not sum every field this is where it shows. Each field is
    // pinned above, so a mismatch here is total(), not the counting.
    EXPECT_EQ(c.total(), 7u);
}

TEST_F(SafetyFilterTest, CounterResetClearsEveryField)
{
    SafetyFilter f = make();

    f.clipForce(100.0);
    f.clipCartesian(makeTwist(0.3, 0.4, 0.0, 1.0, 0.0, 0.0), insideBox());
    f.clipForce(std::numeric_limits<double>::quiet_NaN());
    // Every field must be non-zero BEFORE the reset, or the corresponding
    // assertion after it passes vacuously and proves nothing about reset().
    ASSERT_GT(f.interventions().non_finite_rejected, 0u)
        << "non_finite_rejected never set — its reset assertion would be vacuous";
    ASSERT_GT(f.interventions().total(), 0u) << "nothing to reset — test is vacuous";

    f.interventions().reset();

    const InterventionCounter& c = f.interventions();
    EXPECT_EQ(c.force_scaled, 0u);
    EXPECT_EQ(c.v_tan_scaled, 0u);
    EXPECT_EQ(c.omega_scaled, 0u);
    EXPECT_EQ(c.workspace_refused, 0u);
    EXPECT_EQ(c.joint_vel_scaled, 0u);
    EXPECT_EQ(c.joint_pos_refused, 0u);
    EXPECT_EQ(c.non_finite_rejected, 0u);
    EXPECT_EQ(c.total(), 0u);
}

TEST_F(SafetyFilterTest, PassingCallsDoNotIncrementAnyCounter)
{
    // O3 rests entirely on this: a filter that never fires must report zero.
    // One second of nominal operation at 1 kHz.
    SafetyFilter f = make();

    Joint7 qd_ok;
    for (int j = 0; j < 7; ++j) { qd_ok(j) = 0.5 * bounds_.joint_vel_max[static_cast<std::size_t>(j)]; }
    const Twist6 t_ok = makeTwist(0.05, 0.05, 2.0, 0.1, 0.1, 0.1);  // large vz is legal

    for (int i = 0; i < 1000; ++i)
    {
        f.clipForce(0.5 * bounds_.f_n_max);
        f.clipCartesian(t_ok, insideBox());
        f.clipJoint(qd_ok, Joint7::Zero());
    }

    EXPECT_EQ(f.interventions().total(), 0u)
        << "the filter fired during nominal operation — O3 is not satisfiable";
}