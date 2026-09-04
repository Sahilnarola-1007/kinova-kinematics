/**
 * @file test_dls.cpp
 * @brief Unit tests for damped least-squares differential inverse kinematics.
 *
 * WHAT IS UNDER TEST
 *
 *   solveDLS(J, twist, lambda)          pure linear algebra; Jacobian injected
 *   jointVelocityDLS(q_rad, twist, lam) builds J from the DH chain, then delegates
 *
 * Both solve the same problem: given a desired end-effector twist, find joint
 * velocities that produce it. The arm has 7 joints and the twist has 6
 * components, so the system is underdetermined and infinitely many exact
 * solutions exist. The solver returns
 *
 *     qdot = J^T (J J^T + lambda^2 I)^-1 twist
 *
 * which is the least-squares solution with a penalty on joint speed. Two
 * properties follow, and most of the tests below check one of them:
 *
 *   MINIMUM NORM. Of all joint velocities achieving the twist, this returns
 *   the slowest. That is the point of the redundant seventh joint.
 *
 *   BOUNDED AMPLIFICATION. Near a singularity an undamped solver demands
 *   unbounded joint speed. Damping caps it. Per singular value sigma, the
 *   gain from twist to joint velocity is
 *
 *       g(sigma) = sigma / (sigma^2 + lambda^2)
 *
 *   which peaks at sigma = lambda with value 1/(2*lambda). So no commanded
 *   twist can ever produce joint velocity larger than ||twist|| / (2*lambda),
 *   whatever the arm configuration. This ceiling is what the downstream safety
 *   filter's clamp threshold is derived from, so the tests check both that it
 *   is reachable and that it is never exceeded.
 *
 * FAIL-SAFE CONTRACT
 *
 * On any rejected input the solver must return ok == false with qdot and
 * twist_achieved exactly zero, so a caller that ignores `ok` commands a
 * stationary arm rather than garbage. The diagnostic fields lambda_used and
 * manipulability must remain NaN.
 *
 * NaN is deliberate for manipulability. The manipulability metric is not
 * computed yet. Zero is not an acceptable placeholder, because zero is a
 * meaningful value in this metric: it means the arm is exactly singular, the
 * single most dangerous configuration. A default that reads as the worst real
 * state is a defect waiting to be believed.
 *
 * These tests use no hardware and no randomness. Every expected value is
 * reproducible by hand from the formula above.
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

/**
 * A 6x7 Jacobian of the form [ diag(s0..s5) | 0 ].
 *
 * J J^T is then diag(s0^2 .. s5^2), so the singular values are exactly s0..s5
 * with no SVD and no numerical error. That is what makes it possible to place
 * a singular value exactly at sigma = lambda, the one point where the
 * amplification ceiling is attained. No real arm configuration will land there
 * by chance.
 *
 * Every s must be non-negative; a singular value is non-negative by definition.
 */
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

/**
 * As above, but column 7 duplicates column 1: joints 1 and 7 have identical
 * effect on the end-effector.
 *
 * This exists because the minimum-norm property is untestable on a Jacobian
 * whose last column is zero. There, component 7 of J^T y is zero for every y,
 * so any solver of this form returns zero on joint 7 whether or not it
 * minimises anything. The check would pass on a solver that was wrong
 * everywhere else.
 *
 * With a duplicated column the null space is the diagonal direction
 * (1, 0, 0, 0, 0, 0, -1) rather than a coordinate axis, and minimum norm
 * becomes a claim the solver can get wrong.
 */
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

/// The one configuration taken from the robot config rather than constructed.
constexpr std::array<double, 7> kHomeRad =
    {0.0, 0.262, 3.14, -2.269, 0.0, 0.960, 1.571};

/// Asserts the full fail-safe contract described in the file header.
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

/// If a helper is wrong, every test built on it is wrong in the same way and
/// all of them still pass. Check the helpers before trusting anything below.
TEST(DlsFixtures, HelperJacobiansHaveTheStructureTheyClaim)
{
    const auto Jd = makeDiagonalJ(0.5, 1.0, 1.5, 2.0, 2.5, 3.0);
    const Eigen::Matrix<double, 6, 6> JJt = Jd * Jd.transpose();

    EXPECT_TRUE(JJt.isDiagonal());
    EXPECT_NEAR(JJt(0, 0), 0.25, 1e-15);          // 0.5^2
    EXPECT_NEAR(JJt(5, 5), 9.00, 1e-15);          // 3.0^2
    EXPECT_TRUE(Jd.col(6).isZero(0.0));

    // Duplicating column 1 into column 7 doubles the first diagonal entry of
    // J J^T and leaves the rest untouched.
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

/// As damping goes to zero the solver must converge to the undamped
/// pseudoinverse. (The opposite limit is not the pseudoinverse: at large
/// lambda the gain goes to zero and the solver converges to standing still.)
TEST(DlsSolverCore, ApproachesThePseudoinverseAsDampingVanishes)
{
    KinovaKinematics kin;
    const auto J = makeUniformJ(1.0);
    const auto v = makeTwist(0.1, -0.2, 0.3, 0.4, -0.5, 0.6);

    const auto r = kin.solveDLS(J, v, 1e-6);
    ASSERT_TRUE(r.ok);

    // All singular values are 1, so the exact answer is qdot = twist on the
    // first six joints. Damping introduces a relative error of lambda^2 = 1e-12,
    // which is why the tolerance is 1e-9 rather than zero.
    for (int i = 0; i < 6; ++i) {
        EXPECT_NEAR(r.qdot(i), v(i), 1e-9) << "joint " << i;
    }
    EXPECT_NEAR(r.qdot(6), 0.0, 1e-15);
}

/// The defining property: of the infinitely many joint velocities achieving
/// the commanded twist, the slowest one is returned.
///
/// Joints 1 and 7 are interchangeable here, so any split (a, b) with
/// a + b = twist_x / s0 is exact. Minimum norm is the even split. A solver
/// that drove joint 1 alone would hit the same twist at sqrt(2) times the
/// joint speed, and would fail this test.
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

    // Strictly slower than the single-joint solution it could have returned.
    EXPECT_LT(r.qdot.norm(), v(0) / s0);

    // ...while still achieving the commanded twist.
    EXPECT_LT((r.twist_achieved - v).norm(), 1e-9);
}

/// Zero command, zero motion — and exactly zero, not nearly. Back-substitution
/// on a zero right-hand side produces exact zeros in floating point, so any
/// drift here means something is being added that should not be.
TEST(DlsSolverCore, ZeroTwistProducesExactlyZeroJointVelocity)
{
    KinovaKinematics kin;
    const auto r = kin.solveDLS(makeUniformJ(1.0),
                                Eigen::Matrix<double, 6, 1>::Zero(), 0.05);
    ASSERT_TRUE(r.ok);
    EXPECT_TRUE(r.qdot.isZero(0.0));
}

/// The map from twist to joint velocity is linear, so doubling the command
/// must double the response. This is how we prove no clamping or saturation
/// is hiding inside the solver: limiting belongs to the safety filter
/// downstream, and a limit applied twice is a limit nobody can reason about.
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

/// The amplification ceiling of 1/(2*lambda) is TIGHT, not a loose
/// over-estimate. It is attained when a singular value sits exactly at lambda.
/// This matters because the safety filter's clamp threshold is derived from
/// this number: if the true peak were higher, the clamp would be set too low
/// and would fire during normal operation.
TEST(DlsSolverCore, AmplificationCeilingIsAttainedWhenSigmaEqualsLambda)
{
    KinovaKinematics kin;
    const double lambda = 0.05;

    // Direction 0 sits exactly at the worst case; the other five are healthy
    // and commanded zero, so they contribute nothing to the norm.
    const auto J = makeDiagonalJ(lambda, 1.0, 1.0, 1.0, 1.0, 1.0);
    const auto v = makeTwist(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    const auto r = kin.solveDLS(J, v, lambda);
    ASSERT_TRUE(r.ok);

    // By hand: lambda / (lambda^2 + lambda^2) = 1/(2*lambda) = 10.0
    EXPECT_NEAR(r.qdot(0), 1.0 / (2.0 * lambda), 1e-9);
    EXPECT_NEAR(r.qdot.norm(), v.norm() / (2.0 * lambda), 1e-9);
}

/// The ceiling is never exceeded, at any conditioning, at any damping value.
/// The test above shows it is reachable; this shows it is an upper bound.
TEST(DlsSolverCore, NeverExceedsTheAmplificationCeiling)
{
    KinovaKinematics kin;
    const auto v = makeTwist(0.05, -0.05, 0.05, 0.1, -0.1, 0.1);

    for (double lambda : {0.001, 0.01, 0.05, 0.2, 1.0}) {
        // 1e-9 is numerically singular; lambda itself is the worst case.
        for (double sigma : {1e-9, 1e-4, 0.01, lambda, 0.5, 2.0}) {
            SCOPED_TRACE(testing::Message()
                         << "lambda=" << lambda << " sigma=" << sigma);
            const auto r = kin.solveDLS(makeUniformJ(sigma), v, lambda);
            ASSERT_TRUE(r.ok);
            EXPECT_LE(r.qdot.norm(), v.norm() / (2.0 * lambda) + 1e-9);
        }
    }
}

/// Damping trades tracking accuracy for joint speed, and must do so
/// monotonically in the right direction: more damping, slower motion, larger
/// residual. A sign error in the damping term reverses one of these two and
/// nothing else in this suite would notice.
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

/// The diagnostic fields must describe the solve that actually happened.
///
/// twist_achieved is what the 1 kHz loop logs as tracking error and what the
/// damping sweep is read from. If it were stale, computed from a different
/// Jacobian, or left at zero, both would be silently meaningless.
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

/// Damping must be a positive finite number. Zero or negative removes the
/// damping entirely, and at a true singularity the matrix is genuinely
/// rank-deficient: Eigen's LDL^T factorisation reports success on an exactly
/// singular matrix and returns a finite, meaningless answer, so this guard is
/// the only protection there.
///
/// Three rejection mechanisms are covered, and they are not the same:
///   - zero and negative fail the positivity test directly;
///   - NaN fails it because every comparison involving NaN is false, so NaN
///     falls through the negation and is rejected;
///   - infinity PASSES the positivity test and is not covered by the
///     finiteness check applied to the Jacobian and twist. If this case fails,
///     the guard has a hole and an infinite damping value would return
///     ok == true with zero joint velocity: a stationary arm reported as a
///     successful solve. A malformed configuration value is exactly the input
///     this guard exists to catch.
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

/// A non-finite entry anywhere in the Jacobian. In production this arrives as
/// a corrupt joint angle propagating through the DH chain.
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

/// A non-finite commanded twist, i.e. the admittance reflex or the policy
/// produced garbage. The solver must absorb it rather than forward it to the
/// arm. Infinity is a distinct value from NaN and both must be caught.
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


/// A corrupt joint angle must be caught before it reaches the DH chain.
/// One bad angle contaminates all 42 entries of the Jacobian, at which point
/// the failure is no longer traceable to its source.
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

/// The amplification ceiling must hold on real Jacobians, not only on the
/// constructed ones above.
///
/// The SVD here is far too slow for the 1 kHz control loop but is free in a
/// test, and it reports which of these configurations is actually
/// near-singular rather than leaving it assumed.
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

// --- manipulability() monitor: known value, singular, non-finite ---
// Instance is just a vehicle — manipulability() takes J directly and never
// touches the DH table, so a default-constructed object is fine.

TEST(Manipulability, DiagonalJacobianKnownValue)
{
    KinovaKinematics kin;

    // J = [diag(1,2,3,4,5,6) | 0]  ->  J·Jᵀ = diag(1,4,9,16,25,36)
    // det = 518400,  w = sqrt(518400) = 720  (= 1·2·3·4·5·6), hand-computable.
    Eigen::Matrix<double,6,7> J = Eigen::Matrix<double,6,7>::Zero();
    for (int i = 0; i < 6; ++i)
        J(i, i) = static_cast<double>(i + 1);

    const double w = kin.manipulability(J);

    // Exact-representable integers through det; sqrt of a perfect square.
    // 1e-9 is ~1e-12 relative at w=720 — orders above double epsilon.
    EXPECT_NEAR(w, 720.0, 1e-9);
}

TEST(Manipulability, RankDeficientJacobianIsZero)
{
    KinovaKinematics kin;

    // Same J, row zeroed -> rank 5 -> one singular value 0 -> det = 0.
    // w == 0.0 is a REAL value ("exactly singular"), distinct from the NaN
    // "could-not-compute" case below.
    Eigen::Matrix<double,6,7> J = Eigen::Matrix<double,6,7>::Zero();
    for (int i = 0; i < 6; ++i)
        J(i, i) = static_cast<double>(i + 1);
    J.row(2).setZero();

    const double w = kin.manipulability(J);

    // A zero column drives det to exactly 0; floor-at-0 guard pins any noise.
    EXPECT_NEAR(w, 0.0, 1e-12);
}

TEST(Manipulability, NonFiniteJacobianReturnsNaN)
{
    KinovaKinematics kin;

    // Dropped BaseCyclic frame -> NaN in q -> NaN in J. Must return the
    // "could-not-compute" signal (NaN), NEVER 0.0 ("singular"). isnan, not NEAR:
    // NaN != NaN, so EXPECT_NEAR would always fail.
    Eigen::Matrix<double,6,7> J = Eigen::Matrix<double,6,7>::Zero();
    for (int i = 0; i < 6; ++i)
        J(i, i) = static_cast<double>(i + 1);
    J(0, 0) = std::numeric_limits<double>::quiet_NaN();

    const double w = kin.manipulability(J);

    EXPECT_TRUE(std::isnan(w));
}
