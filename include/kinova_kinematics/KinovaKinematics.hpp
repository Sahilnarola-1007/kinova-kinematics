/**
 * @file KinovaKinematics.hpp
 * @brief FK, numerical Jacobian, DLS differential IK, manipulability monitor and an
 *        offline pose solver for the Kinova Gen3 7-DOF. Classical DH, 8 frames, radians.
 *
 * ── KinovaKinematics ─────────────────────────────────────────────────────────
 * Layer:   Below the contract (adapter), 1 kHz path
 * Side:    OBS    computeFK / getPosition / getRotation  (q_meas → pose, 6D rotation)
 *          ACTION computeJacobian / jointVelocityDLS / solveDLS (twist → q̇)
 *          monitor manipulability()   offline solveIK (never on the loop)
 * In:      q_meas [rad] (OBS+ACTION); twist_des, BASE frame [m/s, rad/s] (ACTION)
 * Out:     T in BASE [m] (OBS); q̇ [rad/s], 7 elements (ACTION)
 * Frames:  task→base rotation is the caller's job; this file never sees the task frame
 * Fails:   non-finite q / J / twist / λ → ok=false, q̇ = 0; near-singular J → damped (BOUND-1)
 * Ref:     design.md §3–§6, D-14, D-20, D-25, D-26, D-27
 * ─────────────────────────────────────────────────────────────────────────────
 */

#pragma once

#include<eigen3/Eigen/Dense>
#include<array>
#include<limits>

/**
 * @brief Result of one differential inverse-kinematics solve. ACTION side.
 *
 * On failure every field stays at its default: qdot all zeros, ok false. A caller
 * that ignores @ref ok therefore commands zero motion — "arm holds", not "arm jumps".
 */
struct DlsResult
{
    Eigen::Matrix<double,7,1> qdot           = Eigen::Matrix<double,7,1>::Zero();  ///< [rad/s], joints 1..7
    Eigen::Matrix<double,6,1> twist_achieved = Eigen::Matrix<double,6,1>::Zero();  ///< J*qdot, BASE frame [m/s; rad/s]
    double lambda_used    = std::numeric_limits<double>::quiet_NaN();
    double manipulability = std::numeric_limits<double>::quiet_NaN();  ///< NaN by design (D-20): never computed here; 0.0 would mean "singular"
    bool   ok             = false;                                     ///< false => qdot is zero, do not command
};

struct DHParam
{
    double alpha{};         // twist angle (radians)
    double a{};             // link length (meters)
    double d{};             // link offset (meters)
    double theta_offset{};  // added to joint angle (radians)
};

struct IKResult{
    bool                 success           = false;
    std::array<double,7> joint_states      = {}; 
    double               position_error    = std::numeric_limits<double>::quiet_NaN();
    double               orientation_error = std::numeric_limits<double>::quiet_NaN();
};

class KinovaKinematics{

    public:

        // member variables
        static constexpr int NUM_JOINTS=7;
        static constexpr int NUM_FRAMES=8;  // rows 0-7 DH table

        /**
         * @brief Constructor — populates Classical DH table for Kinova Gen3.
         * @param tool_offset_z  Distance [m] along the final frame's local z from
         *                       flange to the tip FK reports. Tool-dependent (D-14):
         *                       a parameter, never a literal. Default 0.113 m
         *                       [MEASURED] flange-to-tip of the current test handle;
         *                       Kortex is configured at 0.12 m, so FK-vs-Kortex carries
         *                       ~7 mm by construction.
         */
        explicit KinovaKinematics(double tool_offset_z = 0.113);

        /**
         * @brief Forward kinematics, classical DH chain. OBS side.
         * @param joint_angles 7 joint angles [rad].
         * @return 4x4 pose of the tool tip in the BASE frame [m].
         */
        Eigen::Matrix4d computeFK(const std::array<double, 7> &joint_angles) const;

        /**
         * @brief Map a desired Cartesian twist to joint velocities by damped
         *        least squares. ACTION side, 1 kHz entry point. Pure map: no state,
         *        no clamping, no null space.
         *
         * Solves  (J·Jᵀ + λ²·I₆)·y = twist,  then  q̇ = Jᵀ·y.
         * Equivalent to q̇ = Jᵀ(J·Jᵀ + λ²I)⁻¹·twist without forming any inverse.
         *
         * The Jacobian is evaluated at the MEASURED configuration, never at the
         * commanded one: the joint servo lags the command, so a Jacobian at
         * q_send describes a pose the arm is not in.
         *
         * Output is NOT magnitude-limited. Scaling is the SafetyFilter's job,
         * and it must scale the 7-vector UNIFORMLY — per-joint clipping changes
         * the Cartesian direction of the motion.
         *
         * @param q_meas_rad Measured joint angles [rad], from BaseCyclic feedback.
         * @param twist_des  Desired end-effector twist, base frame:
         *                   rows 0-2 linear [m/s], rows 3-5 angular [rad/s].
         * @param lambda     Damping, must be > 0 and finite. Peak amplification is
         *                   1/(2λ) at σ = λ [DERIVED, design.md §5.3.2]. λ is a
         *                   threshold on σ and the σ spectrum is arm-specific, so λ
         *                   is re-tuned per robot (design.md §5.3.4). It lives in the
         *                   adapter, below the contract. Provisional 0.05 [DESIGN];
         *                   set by the λ sweep.
         * @return DlsResult; check ok before using qdot.
         */
        DlsResult jointVelocityDLS(const std::array<double,7>& q_meas_rad,
                                   const Eigen::Matrix<double,6,1>& twist_des,
                                   double lambda) const;

        /**
         * @brief Extract position from a 4x4 transform. OBS side.
         * @return [x, y, z] in the transform's frame (BASE for computeFK output) [m].
         */
        Eigen::Vector3d getPosition(const Eigen::Matrix4d &transform) const;

        /**
         * @brief Extract the 3x3 rotation from a 4x4 transform. OBS side.
         * @return R. Columns 0 and 1 are the 6D orientation observation as-is — no
         *         Gram-Schmidt anywhere in the loop; nothing is reconstructed from it.
         */
        Eigen::Matrix3d getRotation(const Eigen::Matrix4d& transform) const;

        /**
         * @brief Geometric Jacobian by forward finite difference, eps = 1e-6 rad
         *        [DESIGN, unswept]. ACTION side; evaluate at the MEASURED q.
         *        Rows 0-2 [m/rad], rows 3-5 dimensionless. BASE frame.
         * @param joint_angles Joint configuration [rad].
         * @return 6x7 Jacobian.
         */
        Eigen::Matrix<double,6,7> computeJacobian(const std::array<double,7> &joint_angles) const;

        /**
         * @brief Iterative pose IK: damped least squares with error-proportional
         *        damping plus a null-space joint-centring term. OFFLINE ONLY — dynamic
         *        Eigen, explicit inverse, std::cerr; never on the 1 kHz path.
         * @param target_pose    Desired tool pose, BASE frame [m].
         * @param initial_guess  Starting configuration [rad].
         * @param max_iterations Default 300 [DESIGN]. <= 0 returns failure, errors -1.0.
         * @param position_tol   [m], default 1e-4 [DESIGN].
         * @param orientation_tol [rad], default 1e-3 [DESIGN].
         * @return IKResult. On failure joint_states holds the LAST ITERATE, not a solution —
         *         useful for seeing where the solver stalled. Any caller must check success
         *         before using it; never feed this output to the 1 kHz path.
         */
        IKResult solveIK(
            const Eigen::Matrix4d& target_pose,
            const std::array<double,7> &initial_guess,
            int max_iterations=300,
            double position_tol=1e-4,
            double orientation_tol=1e-3
        )const;

        /**
         * @brief Linear-algebra core of jointVelocityDLS with J injected. Public so
         *        the solve is unit-testable against hand-built Jacobians (D-26).
         * @param J         6x7 Jacobian, BASE [m/rad; dimensionless].
         * @param twist_des Desired twist, BASE [m/s; rad/s].
         * @param lambda    Damping, > 0 and finite.
         * @return DlsResult.
         */
        DlsResult solveDLS(const Eigen::Matrix<double,6,7>& J,
                   const Eigen::Matrix<double,6,1>& twist_des,
                   double lambda) const;

        double getToolOffsetZ() const { return tool_offset_z_; }

        /**
         * @brief Yoshikawa manipulability w = sqrt(det(J·Jᵀ)) — a singularity MONITOR
         *        for the loop. Never an observation element (robot-specific; stays
         *        below the contract). DlsResult::manipulability stays NaN by D-20.
         *        Carries the mixed-units wart of design.md §5.3.3.
         * @param J 6x7 Jacobian at the measured q.
         * @return w >= 0; 0.0 means "exactly singular" (a real value). NaN when J is
         *         non-finite ("could not compute") — distinct from 0.0 on purpose.
         */
        double manipulability(const Eigen::Matrix<double,6,7>& J) const;


    private:

        /**
         * @brief One classical DH link: Rot_z(theta) · Trans_z(d) · Trans_x(a) · Rot_x(alpha).
         * @return Transform from frame (i-1) to frame (i). alpha, theta [rad]; a, d [m].
         */
        Eigen::Matrix4d dhTransform(double alpha, double a, double d, double theta) const;
        
        // Null-space secondary objective for solveIK: gradient pushing each joint
        // toward the centre of its range. Offline only.
        Eigen::VectorXd jointLimitGradient(const std::array<double,7> & joint_angles) const;
        
        // Classical DH table: 8 rows (row 0 = base frame, rows 1-7 = joints 1-7).
        // Link lengths [SPEC] Kinova Gen3 kinematic parameters; convention [MEASURED].
        std::array<DHParam,8> dh_params_;

        // Joint range used ONLY by jointLimitGradient (solveIK). Duplicates
        // SafetyFilter.hpp with different values and different continuous-joint
        // sentinels (±2π here, ±1e9 there) — D-27, single manifest required.
        // Not used for enforcement: Site 3 of the SafetyFilter is the only enforcer.
        std::array<double,7> joint_min_;
        std::array<double,7> joint_max_;
        double tool_offset_z_;   // D-14: tool-dependent, set at construction
        
};
