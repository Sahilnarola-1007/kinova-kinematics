#include<kinova_kinematics/KinovaKinematics.hpp>
#include<iostream>
#include<vector>
#include<eigen3/Eigen/Dense>
#include<cmath>
#include<array>
#include<limits>

using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::Vector3d;


// ── KinovaKinematics ─────────────────────────────────────────────────────────
// Layer:   Below the contract (adapter), 1 kHz path
// Side:    OBS computeFK; ACTION computeJacobian / solveDLS / jointVelocityDLS
// In/Out:  see KinovaKinematics.hpp header block
// Fails:   non-finite input → ok=false, q̇ = 0; near-singular → damped, never refused here
// Ref:     design.md §5, §6
// ─────────────────────────────────────────────────────────────────────────────

namespace {
    // Local to this translation unit: M_PI is POSIX, not ISO C++, and -Wpedantic is on.
    constexpr double PI       = 3.14159265358979323846;
    constexpr double kDegToRad = PI / 180.0;
    constexpr double kRadToDeg = 180.0 / PI;
    }  // namespace

// DH link lengths [SPEC] Kinova Gen3 kinematic parameters (record the User Guide ref).
// joint_min_/max_: [SPEC] User Guide Table 39 for joints 2/4/6; ±2π sentinels for the
// continuous joints — D-27: differs from SafetyFilter.hpp in value and sentinel.
KinovaKinematics::KinovaKinematics(double tool_offset_z):
    dh_params_{
            {{PI,   0.0,  0.0,    0.0},
            {PI/2, 0.0, -0.2848, 0.0},
            {PI/2, 0.0, -0.0118, PI},
            {PI/2, 0.0, -0.4208, PI},
            {PI/2, 0.0, -0.0128, PI},
            {PI/2, 0.0, -0.3143, PI},
            {PI/2, 0.0,  0.0,    PI},
            {PI,   0.0, -0.1674, PI}
            }},
            joint_min_{-1e9, -2.2515, -1e9, -2.5800, -1e9, -2.0996, -1e9},
            joint_max_{1e9,  2.2515,  1e9,  2.5800,  1e9,  2.0996,  1e9},
            tool_offset_z_{tool_offset_z}
            {}

Matrix4d KinovaKinematics::dhTransform(double alpha, double a,
                                        double d, double theta)const{
    double ct=std::cos(theta);
    double st=std::sin(theta);
    double ca=std::cos(alpha);
    double sa=std::sin(alpha);

    Matrix4d T;

    // Classical DH: Rot_z(θ) · Trans_z(d) · Trans_x(a) · Rot_x(α)
    T << ct, -st*ca,  st*sa, a*ct,
         st,  ct*ca, -ct*sa, a*st,
         0,   sa,     ca,    d,
         0,   0,      0,     1;

    return T;
}

Matrix4d KinovaKinematics::computeFK(const std::array<double, 7> &joint_angles) const{

    Matrix4d T, Temp;
    double theta;
    Eigen::Matrix4d T_tool = Eigen::Matrix4d::Identity(); // tool offset, final frame's local z
      

    // Row 0: fixed base frame, no joint angle
    T = dhTransform(dh_params_[0].alpha, dh_params_[0].a,
                    dh_params_[0].d, dh_params_[0].theta_offset);

    for(int i=1; i<NUM_FRAMES; i++)
    {
        // Rows 1-7: joint angle + DH offset [rad]
        theta = joint_angles[i-1] + dh_params_[i].theta_offset;

        Temp = dhTransform(dh_params_[i].alpha, dh_params_[i].a,
                           dh_params_[i].d, theta);
        T = T * Temp;
    }
    
    // Flange → tip along local z (D-14). FK is reported at the tip.
    T_tool(2, 3) = tool_offset_z_;
    T = T * T_tool;
    return T;
    
}

Vector3d KinovaKinematics::getPosition(const Matrix4d &transform) const{
    return {transform(0,3),
            transform(1,3),
            transform(2,3)};
}

Matrix3d KinovaKinematics::getRotation(const Matrix4d &transform) const{
    return transform.block<3,3>(0,0);
}

Eigen::Matrix<double,6,7> KinovaKinematics::computeJacobian(const std::array<double,7> &joint_angles) const{

    Eigen::Matrix<double,6,7> J = Eigen::Matrix<double,6,7>::Zero();

    // Forward-difference step [rad]. [DESIGN] 1e-6: conventional middle between
    // truncation (too large) and cancellation (too small); not swept.
    constexpr double eps = 1e-6;

    std::array<double,7> perturbed_joint_angles;
    Matrix4d T0, T1;
    Matrix3d T0_angular, T1_angular, R_delta;
    Vector3d w, p0, p1;

    // Baseline pose; each column perturbs one joint against it.
    T0 = computeFK(joint_angles);
    T0_angular = T0.block<3,3>(0,0);
    p0 = T0.block<3,1>(0,3);

    for(int i=0; i<NUM_JOINTS; i++){

        perturbed_joint_angles = joint_angles;
        perturbed_joint_angles[i] = joint_angles[i] + eps;

        T1 = computeFK(perturbed_joint_angles);
        T1_angular = T1.block<3,3>(0,0);
        p1 = T1.block<3,1>(0,3);

        // Small rotation from T0 to T1, expressed in BASE: R1·R0ᵀ ≈ I + eps·[w]×
        R_delta = T1_angular * T0_angular.transpose();

        // Linear rows [m/rad], BASE frame
        J.block<3,1>(0,i) = (p1 - p0) / eps;

        // Angular rows [dimensionless]: vee of the skew part, divided by 2·eps
        w << (R_delta(2,1) - R_delta(1,2)) / (2*eps),
             (R_delta(0,2) - R_delta(2,0)) / (2*eps),
             (R_delta(1,0) - R_delta(0,1)) / (2*eps);

        J.block<3,1>(3,i) = w;
    }

    return J;
}

DlsResult KinovaKinematics::solveDLS(const Eigen::Matrix<double,6,7>& J,
                                const Eigen::Matrix<double,6,1>& twist_des,
                                double lambda) const
{

    DlsResult out;   // default: q̇ = 0, ok = false — every early return holds the arm

    // λ > 0 makes A = JJᵀ + λ²I symmetric positive definite at EVERY configuration
    // (xᵀAx = ‖Jᵀx‖² + λ²‖x‖² > 0): singularity only ill-conditions A. λ = 0 leaves
    // A semi-definite and rank-deficient at a singularity — the one case LDLᵀ cannot
    // survive. This guard catches a config typo, not geometry. inf passes λ > 0 and
    // is caught only by isfinite.
    if (!(lambda > 0.0) || !std::isfinite(lambda)) 
        return out;

    // A dropped/partial BaseCyclic frame puts NaN in q and it reaches the wire silently.
    // Whether Eigen's ldlt.info() flags a NaN-filled matrix is [UNVERIFIED], so
    // finiteness is checked explicitly on both ends. (test_dls.cpp guards.)
    if (!J.allFinite() || !twist_des.allFinite()) 
        return out;

    // 6x6 fixed size → stack, no heap on the 1 kHz path; noalias() suppresses the
    // product temporary. design.md §5.3.5.
    Eigen::Matrix<double,6,6> A;
    A.noalias() = J * J.transpose();
    A.diagonal().array() += lambda * lambda;

    // LDLᵀ, never an explicit inverse: backward-stable, pivot diagonal is a free
    // conditioning indicator, and the 7x6 damped pseudoinverse is never formed.
    const Eigen::LDLT<Eigen::Matrix<double,6,6>> ldlt(A);
    if (ldlt.info() != Eigen::Success) 
        return out;

    const Eigen::Matrix<double,6,1> y = ldlt.solve(twist_des);
    if (!y.allFinite()) 
        return out;

    out.lambda_used = lambda;
    out.qdot.noalias() = J.transpose() * y;   // [rad/s], 7x1

    out.twist_achieved.noalias() = J * out.qdot;   // BASE [m/s; rad/s]; residual = damping
    
    out.ok = true;
    return out;
}

DlsResult KinovaKinematics::jointVelocityDLS(
        const std::array<double,7>& q_meas_rad,
        const Eigen::Matrix<double,6,1>& twist_des,
        double lambda) const
{
    DlsResult result;   // default: q̇ = 0, ok = false

    // Reject a non-finite joint angle before it contaminates all 42 Jacobian entries.
    for (int i = 0; i < NUM_JOINTS; ++i) {
        if (!std::isfinite(q_meas_rad[i])) 
            return result;
    }

    // J at the MEASURED q (the servo lags the command, so q_send describes a pose
    // the arm is not in). Mixed row units are why one scalar λ is dimensionally
    // inconsistent — known, accepted for v1, design.md §5.3.3.
    const Eigen::Matrix<double,6,7> J = computeJacobian(q_meas_rad);

    return solveDLS(J, twist_des, lambda);

}

double KinovaKinematics::manipulability(const Eigen::Matrix<double,6,7>& J) const
        {
            // NaN = "could not compute" (non-finite J from a bad frame). Never 0.0,
            // which is a real value meaning "exactly singular".
            if (!J.allFinite())
                return std::numeric_limits<double>::quiet_NaN();

            Eigen::Matrix<double,6,6> A;   // 6x6 symmetric PSD, stack, no heap
            A.noalias() = J * J.transpose();

            // det ≥ 0 mathematically; rounding near singularity can give a tiny
            // negative. Floor at 0 (not abs: abs would turn -1e-18 into a false
            // "slightly non-singular").
            const double det = A.determinant();
            return std::sqrt(det < 0.0 ? 0.0 : det);
        }

IKResult KinovaKinematics::solveIK(
            const Eigen::Matrix4d& target_pose,
            const std::array<double,7> &initial_guess,
            int max_iterations,
            double position_tol,
            double orientation_tol) const{

    // Non-positive iteration count: loop would never run and dx/dw would be
    // uninitialised. -1.0 norms signal "did not run".
    if (max_iterations <= 0) {
        return {false, initial_guess, -1.0, -1.0};
    }

    double alpha=0.5;      // null-space gain [DESIGN]
    std::array<double,7> joint_angles = initial_guess;

    // Dynamic-size Eigen and an explicit inverse below are acceptable ONLY because
    // this solver is offline (design.md §5.5). Never call it from the 1 kHz loop.
    Matrix4d current_pose;
    Matrix3d R_current, R_target, R_delta;
    Vector3d dx,dw ;  // position error [m], orientation error [rad], BASE frame
    Vector3d P_current, P_target;
    Eigen::VectorXd dq(7),z0(7);
    Eigen::VectorXd de = Eigen::VectorXd::Zero(6);;                                      // stacked [dx; dw]
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd N = Eigen::MatrixXd::Zero(7, 7);
    Eigen::MatrixXd I_7 = Eigen::MatrixXd::Identity(7, 7);
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 7);
    Eigen::MatrixXd J_pinv = Eigen::MatrixXd::Zero(7,6);
    Eigen::MatrixXd I_6 = Eigen::MatrixXd::Identity(6, 6);

    int i = 0;
    while(i < max_iterations){

        current_pose = computeFK(joint_angles);

        P_target  = target_pose.block<3,1>(0,3);
        P_current = current_pose.block<3,1>(0,3);
        dx = P_target - P_current;

        // Orientation error: vee of the skew part of R_target·R_currentᵀ (small-angle)
        R_target  = target_pose.block<3,3>(0,0);
        R_current = current_pose.block<3,3>(0,0);
        R_delta   = R_target * R_current.transpose();

        dw << (R_delta(2,1) - R_delta(1,2)) / 2,
              (R_delta(0,2) - R_delta(2,0)) / 2,
              (R_delta(1,0) - R_delta(0,1)) / 2;

        if(dx.norm() < position_tol && dw.norm() < orientation_tol){
            return {true, joint_angles, dx.norm(), dw.norm()};
        }

        J = computeJacobian(joint_angles);   // must be re-evaluated every iterate

        de.block<3,1>(0,0) = dx;
        de.block<3,1>(3,0) = dw;

        // Error-proportional damping [DESIGN]: only meaningful when there IS a pose
        // error — never carry this heuristic into the differential solve.
        double lambda = 0.5 * de.norm() + 1e-4;

        // dq = Jᵀ (JJᵀ + λ²I)⁻¹ de  — explicit inverse tolerated offline only
        A  = J * J.transpose();
        A  = A + (lambda * lambda) * I_6;
        J_pinv = J.transpose()*A.inverse();
        dq = J_pinv* de;

        // Null-space joint-centring: (I − J⁺J) z0 does not move the end-effector
        N = I_7 - J_pinv * J;
        z0=jointLimitGradient(joint_angles);
        dq = dq + alpha * N * z0;
        
        for(int j=0; j<NUM_JOINTS; j++){
            joint_angles[j] = joint_angles[j] + dq(j);
        }
        i++;
    }

    std::cerr << "[solveIK] Warning: did not converge after " << max_iterations
              << " iterations. pos_err=" << dx.norm()
              << "m  ori_err=" << dw.norm() << "rad\n";

    return {false, joint_angles, dx.norm(), dw.norm()};
}

Eigen::VectorXd KinovaKinematics::jointLimitGradient(const std::array<double,7> & joint_angles) const{

    // Gradient toward the centre of each joint range, scaled by 1/range² so
    // narrow joints are pushed harder. Uses joint_min_/max_ (D-27 duplicate).
    Eigen::VectorXd z0(7);
    double q_mid{};

    for(int i=0;i<NUM_JOINTS;i++){
        q_mid=(joint_max_[i]+joint_min_[i])/2;
        z0(i)=(q_mid-joint_angles[i])/((joint_max_[i]-joint_min_[i])*(joint_max_[i]-joint_min_[i]));
    }

    return z0;
} 
