/**
 * @file test_dls.cpp
 * @brief Unit tests for solveDLS (J injected) and jointVelocityDLS (J from the DH chain).
 *        ACTION side, below the contract. Off-robot, no randomness.
 *
 * Properties under test (design.md §5.3): minimum-norm solution of the underdetermined
 * 6x7 system; BOUND-1 ‖q̇‖ ≤ ‖v‖/(2λ) [DERIVED], tight at σ = λ; fail-safe contract —
 * rejected input ⇒ ok == false, qdot and twist_achieved exactly zero, lambda_used and
 * manipulability NaN (0.0 would read as "exactly singular").
 */

#include <gtest/gtest.h>
#include <kinova_kinematics/KinovaKinematics.hpp>

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <cstddef>

namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

/// [ diag(s0..s5) | 0 ]: singular values are exactly s0..s5, so one can be placed
/// at σ = λ where BOUND-1 is attained. All s >= 0.
Eigen::Matrix<double, 6, 7> makeDiagonalJ(double s0, double s1, double s2,
                                          double s3, double s4, double s5)
{
    Eigen::Matrix<double, 6, 7> J = Eigen::Matrix<double, 6, 7>::Zero();
    J(0, 0) = s0;  J(1, 1) = s1;  J(2, 2) = s2;
    J(3, 3) = s3;  J(4, 4) = s4;  J(5, 5) = s5;
    return J;
}

Eigen::Matrix<double, 6, 7> makeUniformJ(double s)
{
    return makeDiagonalJ(s, s, s, s, s, s);
}

/// Column 7 duplicates column 1. With a zero last column, minimum norm is a
/// tautology (Jᵀy has a zero 7th component for every y); a duplicated column
/// gives null space (1,0,0,0,0,0,-1) and makes the claim falsifiable.
Eigen::Matrix<double, 6, 7> makeDuplicateColumnJ(double s0, double s1, double s2,
                                                 double s3, double s4, double s5)
{
    Eigen::Matrix<double, 6, 7> J = makeDiagonalJ(s0, s1, s2, s3, s4, s5);
    J.col(6) = J.col(0);
    return J;
}

Eigen::Matrix<double, 6, 1> makeTwist(double a, double b, double c,
                                      double d, double e, double f)
{
    Eigen::Matrix<double, 6, 1> v;
    v << a, b, c, d, e, f;
    return v;
}

/// Kortex "Home" preset in rad [UNVERIFIED — confirm against the Web App]. Not q = 0,
/// which is rank 3 (design.md §7).
constexpr std::array<double, 7> kHomeRad =
    {0.0, 0.262, 3.14, -2.269, 0.0, 0.960, 1.571};

/// Full fail-safe contract (file header).
void expectSafeFailure(const DlsResult& r)
{
    EXPECT_FALSE(r.ok);
    EXPECT_TRUE(r.qdot.isZero(0.0))
        << "rejected input must command exactly zero joint velocity";
    EXPECT_TRUE(r.twist_achieved.isZero(0.0))
        << "rejected input must report zero achieved twist";
    EXPECT_TRUE(std::isnan(r.lambda_used))
        << "no damping was applied, so none may be reported";
    EXPECT_TRUE(std::isnan(r.manipulability))
        << "manipulability must never default to 0.0; see file header";
}

}  // namespace


// ===========================================================================
// Test fixtures
// ===========================================================================

/// A wrong helper makes every test below wrong in the same way and still green.
TEST(DlsFixtures, HelperJacobiansHaveTheStructureTheyClaim)
{
    const auto Jd = makeDiagonalJ(0.5, 1.0, 1.5, 2.0, 2.5, 3.0);
    const Eigen::Matrix<double, 6, 6> JJt = Jd * Jd.transpose();

    EXPECT_TRUE(JJt.isDiagonal());
    EXPECT_NEAR(JJt(0, 0), 0.25, 1e-15);          // 0.5^2
    EXPECT_NEAR(JJt(5, 5), 9.00, 1e-15);          // 3.0^2
    EXPECT_TRUE(Jd.col(6).isZero(0.0));

    // Duplicated column doubles JJᵀ(0,0) only.
    const auto Jr = makeDuplicateColumnJ(1.0, 0.8, 1.2, 0.9, 1.1, 1.0);
    const Eigen::Matrix<double, 6, 6> RRt = Jr * Jr.transpose();

    EXPECT_TRUE(RRt.isDiagonal());
    EXPECT_NEAR(RRt(0, 0), 2.0, 1e-15);           // 1.0^2 + 1.0^2
    EXPECT_NEAR(RRt(1, 1), 0.64, 1e-15);          // 0.8^2
    EXPECT_TRUE(Jr.col(6).isApprox(Jr.col(0)));
}


// ===========================================================================
// Solver correctness, with the Jacobian injected
// ===========================================================================

/// λ → 0 must recover the undamped pseudoinverse (λ → ∞ recovers standing still).
TEST(DlsSolverCore, ApproachesThePseudoinverseAsDampingVanishes)
{
    KinovaKinematics kin;
    const auto J = makeUniformJ(1.0);
    const auto v = makeTwist(0.1, -0.2, 0.3, 0.4, -0.5, 0.6);

    const auto r = kin.solveDLS(J, v, 1e-6);
    ASSERT_TRUE(r.ok);

    // σ = 1 everywhere ⇒ qdot = twist on joints 0-5; damping error is λ² = 1e-12.
    for (int i = 0; i < 6; ++i) {
        EXPECT_NEAR(r.qdot(i), v(i), 1e-9) << "joint " << i;
    }
    EXPECT_NEAR(r.qdot(6), 0.0, 1e-15);
}

/// Minimum norm: joints 1 and 7 are interchangeable, any split a + b = vx/s0 is
/// exact, and the slowest is the even split (single-joint would be √2 faster).
TEST(DlsSolverCore, SplitsRedundantMotionEvenlyBetweenInterchangeableJoints)
{
    KinovaKinematics kin;
    const double s0 = 1.0;
    const auto J = makeDuplicateColumnJ(s0, 0.8, 1.2, 0.9, 1.1, 1.0);
    const auto v = makeTwist(0.4, 0.0, 0.0, 0.0, 0.0, 0.0);

    const auto r = kin.solveDLS(J, v, 1e-6);
    ASSERT_TRUE(r.ok);

    const double even_split = v(0) / (2.0 * s0);       // 0.2 rad/s each
    EXPECT_NEAR(r.qdot(0), even_split, 1e-9);
    EXPECT_NEAR(r.qdot(6), even_split, 1e-9);

    EXPECT_LT(r.qdot.norm(), v(0) / s0);   // slower than the single-joint solution

    EXPECT_LT((r.twist_achieved - v).norm(), 1e-9);
}

/// Exactly zero, not nearly: a zero RHS back-substitutes to exact zeros, so any
/// drift means something is being added.
TEST(DlsSolverCore, ZeroTwistProducesExactlyZeroJointVelocity)
{
    KinovaKinematics kin;
    const auto r = kin.solveDLS(makeUniformJ(1.0),
                                Eigen::Matrix<double, 6, 1>::Zero(), 0.05);
    ASSERT_TRUE(r.ok);
    EXPECT_TRUE(r.qdot.isZero(0.0));
}

/// Linearity proves no clamping hides inside the solver — limiting belongs to
/// SafetyFilter Site 3 only.
TEST(DlsSolverCore, ScalesLinearlyWithTheCommandedTwist)
{
    KinovaKinematics kin;
    const auto J = makeDiagonalJ(0.9, 1.0, 1.1, 0.8, 1.2, 1.0);
    const auto v = makeTwist(0.05, -0.05, 0.05, 0.1, -0.1, 0.1);

    const auto r1 = kin.solveDLS(J, v,       0.05);
    const auto r2 = kin.solveDLS(J, 2.0 * v, 0.05);
    ASSERT_TRUE(r1.ok);
    ASSERT_TRUE(r2.ok);

    EXPECT_LT((r2.qdot - 2.0 * r1.qdot).norm(), 1e-12);
}

/// BOUND-1 is tight: attained at σ = λ. The Site 3 threshold argument rests on it.
TEST(DlsSolverCore, AmplificationCeilingIsAttainedWhenSigmaEqualsLambda)
{
    KinovaKinematics kin;
    const double lambda = 0.05;

    // Direction 0 at the worst case; the healthy five are commanded zero.
    const auto J = makeDiagonalJ(lambda, 1.0, 1.0, 1.0, 1.0, 1.0);
    const auto v = makeTwist(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    const auto r = kin.solveDLS(J, v, lambda);
    ASSERT_TRUE(r.ok);

    // λ/(λ²+λ²) = 1/(2λ) = 10.0
    EXPECT_NEAR(r.qdot(0), 1.0 / (2.0 * lambda), 1e-9);
    EXPECT_NEAR(r.qdot.norm(), v.norm() / (2.0 * lambda), 1e-9);
}

/// BOUND-1 is an upper bound at every (λ, σ) pair, including σ = λ.
TEST(DlsSolverCore, NeverExceedsTheAmplificationCeiling)
{
    KinovaKinematics kin;
    const auto v = makeTwist(0.05, -0.05, 0.05, 0.1, -0.1, 0.1);

    for (double lambda : {0.001, 0.01, 0.05, 0.2, 1.0}) {
        // 1e-9 is numerically singular; σ = λ is the worst case.
        for (double sigma : {1e-9, 1e-4, 0.01, lambda, 0.5, 2.0}) {
            SCOPED_TRACE(testing::Message()
                         << "lambda=" << lambda << " sigma=" << sigma);
            const auto r = kin.solveDLS(makeUniformJ(sigma), v, lambda);
            ASSERT_TRUE(r.ok);
            EXPECT_LE(r.qdot.norm(), v.norm() / (2.0 * lambda) + 1e-9);
        }
    }
}

/// More λ ⇒ slower motion and larger residual, monotonically. A sign error in
/// the damping term flips one of these and nothing else here would notice.
TEST(DlsSolverCore, MoreDampingMeansSlowerMotionAndLargerResidual)
{
    KinovaKinematics kin;
    const auto J = makeDiagonalJ(0.9, 1.0, 1.1, 0.8, 1.2, 1.0);
    const auto v = makeTwist(0.1, 0.1, 0.1, 0.1, 0.1, 0.1);

    double prev_speed    = kInf;
    double prev_residual = -1.0;

    for (double lambda : {0.001, 0.01, 0.05, 0.1, 0.5, 1.0}) {
        SCOPED_TRACE(testing::Message() << "lambda=" << lambda);
        const auto r = kin.solveDLS(J, v, lambda);
        ASSERT_TRUE(r.ok);

        const double residual = (r.twist_achieved - v).norm();
        EXPECT_LT(r.qdot.norm(), prev_speed);
        EXPECT_GT(residual, prev_residual);

        prev_speed    = r.qdot.norm();
        prev_residual = residual;
    }
}

/// twist_achieved feeds the loop's tracking log and the λ sweep; stale or zero
/// would make both silently meaningless.
TEST(DlsSolverCore, DiagnosticFieldsDescribeTheSolveThatRan)
{
    KinovaKinematics kin;
    const auto J = makeDiagonalJ(0.9, 1.0, 1.1, 0.8, 1.2, 1.0);
    const auto v = makeTwist(0.05, 0.02, -0.03, 0.1, 0.0, -0.1);
    const double lambda = 0.037;

    const auto r = kin.solveDLS(J, v, lambda);
    ASSERT_TRUE(r.ok);

    EXPECT_LT((r.twist_achieved - J * r.qdot).norm(), 1e-12);
    EXPECT_GT(r.twist_achieved.norm(), 0.0) << "must not be left at zero";
    EXPECT_DOUBLE_EQ(r.lambda_used, lambda);
}


// ===========================================================================
// Input guards. Every one of these must fail safe, not merely fail.
// ===========================================================================

/// Three distinct mechanisms: 0/negative fail λ > 0; NaN fails it because every
/// NaN comparison is false; inf PASSES λ > 0 and is caught only by isfinite —
/// without that, inf λ returns ok == true with zero motion, a hold reported as
/// a successful solve. Whether Eigen's LDLᵀ flags an exactly singular matrix is
/// [UNVERIFIED]; this guard is the protection at λ = 0 regardless.
TEST(DlsSolverGuards, RejectsDampingThatIsNotPositiveAndFinite)
{
    KinovaKinematics kin;
    const auto J = makeUniformJ(1.0);
    const auto v = makeTwist(0.1, 0, 0, 0, 0, 0);

    for (double lambda : {0.0, -0.05, -1e-12, kNaN, kInf}) {
        SCOPED_TRACE(testing::Message() << "lambda=" << lambda);
        expectSafeFailure(kin.solveDLS(J, v, lambda));
    }
}

/// Non-finite J — in production, a corrupt joint angle through the DH chain.
TEST(DlsSolverGuards, RejectsNonFiniteJacobianEntries)
{
    KinovaKinematics kin;
    const auto v = makeTwist(0.1, 0, 0, 0, 0, 0);

    for (double bad : {kNaN, kInf}) {
        SCOPED_TRACE(testing::Message() << "bad entry = " << bad);
        auto J = makeUniformJ(1.0);
        J(3, 4) = bad;
        expectSafeFailure(kin.solveDLS(J, v, 0.05));
    }
}

/// Non-finite twist (reflex or policy produced garbage). inf and NaN are
/// distinct values; both must be caught.
TEST(DlsSolverGuards, RejectsNonFiniteCommandedTwist)
{
    KinovaKinematics kin;

    for (double bad : {kNaN, kInf}) {
        SCOPED_TRACE(testing::Message() << "bad entry = " << bad);
        auto v = makeTwist(0.1, 0, 0, 0, 0, 0);
        v(2) = bad;
        expectSafeFailure(kin.solveDLS(makeUniformJ(1.0), v, 0.05));
    }
}


/// One bad angle contaminates all 42 Jacobian entries; catch it before the chain.
TEST(DlsJointVelocity, RejectsNonFiniteJointAngles)
{
    KinovaKinematics kin;
    const auto v = makeTwist(0.05, 0, 0, 0, 0, 0);

    for (int i = 0; i < KinovaKinematics::NUM_JOINTS; ++i) {
        for (double bad : {kNaN, kInf}) {
            SCOPED_TRACE(testing::Message()
                         << "joint " << i << " set to " << bad);
            std::array<double, 7> q = kHomeRad;
            q[i] = bad;
            expectSafeFailure(kin.jointVelocityDLS(q, v, 0.05));
        }
    }
}

/// BOUND-1 on real Jacobians. The SVD is test-only (too slow for 1 kHz) and
/// prints σ_min so near-singularity is reported, not assumed. Config 1 is q = 0
/// (rank 3, design.md §7).
TEST(DlsJointVelocity, RespectsTheAmplificationCeilingAcrossConfigurations)
{
    KinovaKinematics kin;
    const double lambda = 0.05;
    const auto v = makeTwist(0.05, 0.0, 0.0, 0.0, 0.0, 0.0);

    const std::array<std::array<double, 7>, 4> configs = {{
        kHomeRad,
        {{0.0,  0.0,  0.0,  0.0,  0.0, 0.0,  0.0}},
        {{0.5,  0.5,  0.5, -1.0,  0.5, 0.5,  0.5}},
        {{1.0, -0.4,  2.0, -1.5,  0.3, 0.7, -0.2}},
    }};

    for (std::size_t k = 0; k < configs.size(); ++k) {
        SCOPED_TRACE(testing::Message() << "configuration " << k);
        const auto r = kin.jointVelocityDLS(configs[k], v, lambda);
        ASSERT_TRUE(r.ok);
        EXPECT_TRUE(r.qdot.allFinite());
        EXPECT_LE(r.qdot.norm(), v.norm() / (2.0 * lambda) + 1e-9);

        const Eigen::JacobiSVD<Eigen::Matrix<double, 6, 7>>
            svd(kin.computeJacobian(configs[k]));
        std::cout << "  configuration " << k
                  << "  smallest singular value = " << svd.singularValues().minCoeff()
                  << "  |qdot| = " << r.qdot.norm() << " rad/s\n";
    }
}

// --- manipulability() monitor: known value, singular, non-finite.
// manipulability() takes J directly; the instance never touches the DH table.

TEST(Manipulability, DiagonalJacobianKnownValue)
{
    KinovaKinematics kin;

    // JJᵀ = diag(1,4,9,16,25,36), det = 518400, w = 720 = 1·2·3·4·5·6
    Eigen::Matrix<double,6,7> J = Eigen::Matrix<double,6,7>::Zero();
    for (int i = 0; i < 6; ++i)
        J(i, i) = static_cast<double>(i + 1);

    const double w = kin.manipulability(J);

    // Exact integers through det; 1e-9 is ~1e-12 relative, far above double eps.
    EXPECT_NEAR(w, 720.0, 1e-9);
}

TEST(Manipulability, RankDeficientJacobianIsZero)
{
    KinovaKinematics kin;

    // Row zeroed → rank 5 → det = 0. w == 0.0 is a REAL value ("exactly singular").
    Eigen::Matrix<double,6,7> J = Eigen::Matrix<double,6,7>::Zero();
    for (int i = 0; i < 6; ++i)
        J(i, i) = static_cast<double>(i + 1);
    J.row(2).setZero();

    const double w = kin.manipulability(J);

    // Floor-at-0 pins any rounding noise.
    EXPECT_NEAR(w, 0.0, 1e-12);
}

TEST(Manipulability, NonFiniteJacobianReturnsNaN)
{
    KinovaKinematics kin;

    // Dropped frame → NaN in J → must return NaN ("could not compute"), never 0.0.
    // isnan, not EXPECT_NEAR: NaN != NaN.
    Eigen::Matrix<double,6,7> J = Eigen::Matrix<double,6,7>::Zero();
    for (int i = 0; i < 6; ++i)
        J(i, i) = static_cast<double>(i + 1);
    J(0, 0) = std::numeric_limits<double>::quiet_NaN();

    const double w = kin.manipulability(J);

    EXPECT_TRUE(std::isnan(w));
}
