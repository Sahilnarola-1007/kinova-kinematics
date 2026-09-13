/**
 * @file test_safety_filter.cpp
 * @brief Unit tests for SafetyFilter — three clip sites + InterventionCounter.
 *        ACTION side, below the contract. Pure computation, no hardware, CI-safe.
 *
 * Where a formula is an implementation choice the test pins the relationship
 * (value == scale * input); where a structural decision is guarded (vz exclusion,
 * uniform scaling, directional refusal) the test is exact. Rationale: safety_filter.md.
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

/// Absolute tolerance [DERIVED]: quantities are O(1) SI, longest path is
/// sqrt + two rounded ops (~1e-16), so 1e-9 is seven orders above round-off and
/// eight below the smallest meaningful bound difference. A failure is logic, not rounding.
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

    /// Inside the default workspace box, BASE frame [m].
    static Eigen::Vector3d insideBox() { return Eigen::Vector3d(0.0, 0.0, 0.35); }

    /// Frame note: clipCartesian takes a TASK-frame twist and a BASE-frame position.
    /// Workspace tests assume task→base = identity so component-wise assertions
    /// are meaningful (the known limitation in safety_filter.md, not a claim).
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
    EXPECT_NEAR(r.value, r.scale * f_in, kTol);   // relationship, not formula
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
    // Clamping a value to itself is not an intervention; O3 counts depend on > vs >=.
    SafetyFilter f = make();

    const ForceFilterResult r = f.clipForce(bounds_.f_n_max);

    EXPECT_NEAR(r.value, bounds_.f_n_max, kTol);
    EXPECT_NEAR(r.scale, 1.0, kTol);
    EXPECT_EQ(r.reason, FilterReason::PASS);
    EXPECT_EQ(f.interventions().force_scaled, 0u);
}

TEST_F(SafetyFilterTest, ForceZeroInputDoesNotDivideByZero)
{
    // HOLD commands this 1000×/s: a hot path, not an edge case. EXPECT_NEAR
    // already fails on NaN, so no separate isfinite assertion.
    SafetyFilter f = make();

    const ForceFilterResult r = f.clipForce(0.0);

    EXPECT_NEAR(r.value, 0.0, kTol);
    EXPECT_NEAR(r.scale, 1.0, kTol);
    EXPECT_EQ(r.reason, FilterReason::PASS);
    EXPECT_EQ(f.interventions().total(), 0u);
}

TEST_F(SafetyFilterTest, ForceNonFiniteInputIsRejected)
{
    // Declared failure mode is "arm holds": non-finite in → finite zero out.
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

    // Telemetry: an unguarded site counted NaN as a force clip. Pin both halves.
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
    // |[0.3, 0.4]| = 0.5 ⇒ alpha = 0.15/0.5 = 0.3
    SafetyFilter f = make();
    const Twist6 t = makeTwist(0.3, 0.4, 0.0, 0.0, 0.0, 0.0);

    const CartesianFilterResult r = f.clipCartesian(t, insideBox());

    EXPECT_NEAR(r.alpha, 0.3, kTol);
    EXPECT_NEAR(r.twist(0), 0.09, kTol);
    EXPECT_NEAR(r.twist(1), 0.12, kTol);
    EXPECT_NEAR(r.twist(1) / r.twist(0), t(1) / t(0), kTol)   // heading unchanged
        << "slowing down also steered the tool";
    EXPECT_NEAR(std::hypot(r.twist(0), r.twist(1)), bounds_.v_tan_max, kTol);
    EXPECT_TRUE(hasReason(r.reasons, FilterReason::V_TAN_SCALED));
    EXPECT_EQ(f.interventions().v_tan_scaled, 1u);
}

TEST_F(SafetyFilterTest, NormalVelocityExcludedFromTangentialNorm)
{
    // STRUCTURAL: the reflex owns vz; vz in the tangential norm = two limiters
    // in series on the force-controlled axis.
    SafetyFilter f = make();

    // Case A: huge vz, tiny tangential.
    const Twist6 a = makeTwist(0.01, 0.0, 5.0, 0.0, 0.0, 0.0);
    const CartesianFilterResult ra = f.clipCartesian(a, insideBox());
    EXPECT_NEAR(ra.alpha, 1.0, kTol) << "vz leaked into the tangential norm";
    EXPECT_NEAR(ra.twist(2), 5.0, kTol) << "the reflex was throttled by Site 2";
    EXPECT_EQ(ra.num_interventions, 0);

    // Case B: tangential at the bound, huge vz. Still no scaling.
    const Twist6 b = makeTwist(bounds_.v_tan_max, 0.0, 5.0, 0.0, 0.0, 0.0);
    const CartesianFilterResult rb = f.clipCartesian(b, insideBox());
    EXPECT_NEAR(rb.alpha, 1.0, kTol);
    EXPECT_NEAR(rb.twist(2), 5.0, kTol);
}

TEST_F(SafetyFilterTest, AngularScalingIndependentOfLinear)
{
    // alpha and beta come from separate norms. This does NOT claim the
    // linear:angular ratio is preserved (it cannot be — vz is exempt).
    SafetyFilter f = make();

    // Angular over, linear under => beta < 1, alpha == 1.
    const Twist6 a = makeTwist(0.01, 0.0, 0.0, 1.0, 0.0, 0.0);
    const CartesianFilterResult ra = f.clipCartesian(a, insideBox());
    EXPECT_NEAR(ra.alpha, 1.0, kTol) << "angular clip bled into the linear block";
    EXPECT_NEAR(ra.twist(0), 0.01, kTol);
    EXPECT_NEAR(ra.beta, bounds_.omega_max / 1.0, kTol);
    EXPECT_NEAR(ra.twist.tail<3>().norm(), bounds_.omega_max, kTol);
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
    // HOLD path, 1000×/s.
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
    // Parity with Site 1. The norm math cannot catch NaN (comparisons are false),
    // so an unguarded Site 2 sprays it across the whole twist.
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

    // Non-finite EE position (bad FK) with a finite twist.
    const Eigen::Vector3d bad_pos(0.0, nan, 0.35);
    const CartesianFilterResult r_pos =
        f.clipCartesian(makeTwist(0.05, 0.0, 0.0, 0.0, 0.0, 0.0), bad_pos);
    EXPECT_TRUE(r_pos.twist.allFinite()) << "NaN position propagated through Site 2";
    EXPECT_NEAR(r_pos.twist.norm(), 0.0, kTol);
    EXPECT_TRUE(hasReason(r_pos.reasons, FilterReason::NON_FINITE_REJECTED));

    // Telemetry: |[inf,0,0]| > omega_max is TRUE, so an unguarded site logs a
    // phantom angular clip into the O3 count.
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
    // RECOVERY: otherwise an arm that drifts out is trapped.
    SafetyFilter f = make();

    // Past +x, driving back toward -x.
    const Eigen::Vector3d past_x(bounds_.ws_x_max + 0.01, 0.0, 0.35);
    const CartesianFilterResult rx =
        f.clipCartesian(makeTwist(-0.05, 0.0, 0.0, 0.0, 0.0, 0.0), past_x);
    EXPECT_NEAR(rx.twist(0), -0.05, kTol) << "inward motion refused; arm is stuck";
    EXPECT_FALSE(hasReason(rx.reasons, FilterReason::WORKSPACE_REFUSED));

    // Below z_min, driving back up (table-collision case).
    const Eigen::Vector3d below_z(0.0, 0.0, bounds_.ws_z_min - 0.01);
    const CartesianFilterResult rz =
        f.clipCartesian(makeTwist(0.0, 0.0, 0.05, 0.0, 0.0, 0.0), below_z);
    EXPECT_NEAR(rz.twist(2), 0.05, kTol) << "cannot lift off the floor";
    EXPECT_FALSE(hasReason(rz.reasons, FilterReason::WORKSPACE_REFUSED));

    EXPECT_EQ(f.interventions().workspace_refused, 0u);
}

TEST_F(SafetyFilterTest, WorkspaceRefusalIsReportedSeparatelyFromScaling)
{
    // Two problems in one cycle are logged as two.
    SafetyFilter f = make();
    const Eigen::Vector3d outside(bounds_.ws_x_max + 0.01, 0.0, 0.35);
    const Twist6 t = makeTwist(0.3, 0.4, 0.0, 0.0, 0.0, 0.0);  // |v_tan| = 0.5

    const CartesianFilterResult r = f.clipCartesian(t, outside);

    EXPECT_TRUE(hasReason(r.reasons, FilterReason::V_TAN_SCALED));
    EXPECT_TRUE(hasReason(r.reasons, FilterReason::WORKSPACE_REFUSED));
    EXPECT_EQ(r.num_interventions, 2);
    EXPECT_EQ(f.interventions().v_tan_scaled, 1u);
    EXPECT_EQ(f.interventions().workspace_refused, 1u);

    // Order-independent: both rules hold whichever runs first.
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
    // Joint 0 over 2x (ratio 0.5), joint 4 over 4x (ratio 0.25); both continuous,
    // so no position refusal interferes.
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
    // Per-joint clipping would satisfy the caps and change where the EE goes.
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
    // Guards the [SPEC] table (120 deg/s joints 1-4, 200 deg/s joints 5-7) against
    // a copy-paste regression.
    for (std::size_t i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(bounds_.joint_vel_max[i], 2.0944, 1e-4) << "large actuator " << i;
    }
    for (std::size_t i = 4; i < 7; ++i)
    {
        EXPECT_NEAR(bounds_.joint_vel_max[i], 3.4907, 1e-4) << "small actuator " << i;
    }
    EXPECT_GT(bounds_.joint_vel_max[4], bounds_.joint_vel_max[3]);

    // Only joints 2, 4, 6 (indices 1, 3, 5) are bounded [SPEC Table 39]. These
    // literals duplicate KinovaKinematics.cpp and disagree at the 4th digit —
    // D-27; this test reads the manifest once D-04 lands.
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
    // Bit j (0-indexed) = joint index j = physical joint j+1.
    SafetyFilter f = make();

    Joint7 q_send = Joint7::Zero();
    q_send(1) = bounds_.joint_pos_max[1] - 0.0005;   // 0.5 mrad inside the limit

    Joint7 qd = Joint7::Constant(0.1);
    qd(1) = 2.0;                                     // under the 2.0944 cap: no vel scaling
    // lookahead q_send(1) + 2.0·0.001 = limit + 0.0015 → crosses

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
    // RECOVERY: overshoot, bad seed or manual jogging can leave a joint outside
    // its limit; a non-directional check would strand it.
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
    // Joints 1, 3, 5, 7 (indices 0, 2, 4, 6) are continuous; an anchor past 2π
    // must not trip a phantom limit (D-27: ±2π elsewhere would).
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
    // HOLD path, 1000×/s.
    SafetyFilter f = make();

    const JointFilterResult r = f.clipJoint(Joint7::Zero(), Joint7::Zero());

    EXPECT_NEAR(r.qdot.norm(), 0.0, kTol);
    EXPECT_TRUE(std::isfinite(r.vel_scale));
    EXPECT_NEAR(r.vel_scale, 1.0, kTol);
    EXPECT_EQ(r.pos_refused_mask, 0u);
    EXPECT_EQ(r.num_interventions, 0);

    // Parked on a limit with zero velocity is not a violation.
    Joint7 q_at_limit = Joint7::Zero();
    q_at_limit(1) = bounds_.joint_pos_max[1];
    const JointFilterResult r2 = f.clipJoint(Joint7::Zero(), q_at_limit);
    EXPECT_EQ(r2.pos_refused_mask, 0u) << "refused a zero-velocity command";

    EXPECT_EQ(f.interventions().total(), 0u);
}

TEST_F(SafetyFilterTest, NonFiniteJointInputIsRejected)
{
    // Declared failure mode is "hold". Comparisons against NaN are false, so an
    // unguarded filter passes it through untouched.
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
    // STRUCTURAL: mask 0, not all-ones — logs must tell "joint hit a limit" from
    // "IK returned NaN".
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

    // Non-finite anchor: the lookahead becomes NaN and every limit check passes.
    Joint7 q_bad = Joint7::Zero();
    q_bad(2) = nan;
    const JointFilterResult r_q = f.clipJoint(Joint7::Constant(0.1), q_bad);
    EXPECT_TRUE(r_q.qdot.allFinite()) << "NaN anchor propagated through Site 3";
    EXPECT_NEAR(r_q.qdot.norm(), 0.0, kTol);
    EXPECT_TRUE(hasReason(r_q.reasons, FilterReason::NON_FINITE_REJECTED));
    EXPECT_EQ(r_q.pos_refused_mask, 0u);

    // Telemetry: |inf| > cap is TRUE ⇒ alpha = 0 ⇒ an unguarded site logs a
    // phantom velocity clip.
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
    // Each bound exactly once, in isolation.
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
    // Fields are pinned above, so a mismatch here is total(), not the counting.
    EXPECT_EQ(c.total(), 7u);
}

TEST_F(SafetyFilterTest, CounterResetClearsEveryField)
{
    SafetyFilter f = make();

    f.clipForce(100.0);
    f.clipCartesian(makeTwist(0.3, 0.4, 0.0, 1.0, 0.0, 0.0), insideBox());
    f.clipForce(std::numeric_limits<double>::quiet_NaN());
    // Non-zero before reset, or the post-reset assertions are vacuous.
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
    // O3 rests on this: one second of nominal 1 kHz operation, zero interventions.
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