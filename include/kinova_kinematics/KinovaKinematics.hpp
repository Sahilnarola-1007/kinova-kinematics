/**
 * @file KinovaKinematics.hpp
 * @brief FK, numerical Jacobian, and DLS IK solver for Kinova Gen3 7-DOF arm.
 *        Uses Classical DH convention (8 frames, rows 0-7).
 */

#pragma once

#include<eigen3/Eigen/Dense>
#include<array>
#include<limits>

/**
 * @brief Result of one differential inverse-kinematics solve.
 *
 * On failure every field is left at its default: q_dot_deg_s is all zeros,
 * ok is false. A caller that ignores @ref ok therefore commands zero motion
 * rather than garbage — the failure mode is "arm holds", not "arm jumps".
 */
struct DlsResult
{
    Eigen::Matrix<double,7,1> qdot           = Eigen::Matrix<double,7,1>::Zero();  ///< [rad/s], joints 1..7
    Eigen::Matrix<double,6,1> twist_achieved = Eigen::Matrix<double,6,1>::Zero();  ///< J*qdot, BASE frame [m/s; rad/s]
    double lambda_used    = std::numeric_limits<double>::quiet_NaN();
    double manipulability = std::numeric_limits<double>::quiet_NaN();  ///< P22 — NOT computed yet, metric deferred
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
         * @param tool_offset_z  Distance [m] along the tool's local z-axis from
         *                       the final DH frame to the tip FK is computed
         *                       w.r.t. Tool-dependent (D-14): a parameter, never
         *                       a literal. Default 0.113 m = current test handle.
         */
        explicit KinovaKinematics(double tool_offset_z = 0.113);

        /**
         * @brief Compute forward kinematics using Classical DH chain.
         * @param joint_angles 7 joint angles in radians.
         * @return 4x4 homogeneous transform of end-effector in base frame.
         */
        Eigen::Matrix4d computeFK(const std::array<double, 7> &joint_angles) const;

        /**
         * @brief Map a desired Cartesian twist to joint velocities by damped
         *        least squares. Pure map: no state, no clamping, no null space.
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
         * @param lambda     Damping factor, must be > 0. Peak joint amplification
         *                   is 1/(2λ), reached when a singular value σ equals λ.
         *                   λ is therefore a threshold on σ, and the σ spectrum
         *                   depends on the arm's geometry — λ must be re-tuned
         *                   per robot (design.md §5.4.1). It lives in the adapter,
         *                   below the interface contract, so a per-robot λ does
         *                   not weaken the portability claim.
         * @return DlsResult; check ok before using q_dot_rad.
         */
        DlsResult jointVelocityDLS(const std::array<double,7>& q_meas_rad,
                                   const Eigen::Matrix<double,6,1>& twist_des,
                                   double lambda) const;

        /**
         * @brief Extract [x, y, z] position from a 4x4 homogeneous transform.
         * @param transform 4x4 homogeneous transform matrix.
         * @return 3x1 position vector (meters).
         */
        Eigen::Vector3d getPosition(const Eigen::Matrix4d &transform) const;

        /**
         * @brief Extract 3x3 rotation matrix from a 4x4 homogeneous transform.
         * @param transform 4x4 homogeneous transform matrix.
         * @return 3x3 rotation matrix.
         */
        Eigen::Matrix3d getRotation(const Eigen::Matrix4d& transform) const;

        /**
         * @brief Compute numerical Jacobian via finite differences.
         *        Top 3 rows: linear velocity (m/rad).
         *        Bottom 3 rows: angular velocity (rad/rad).
         * @param joint_angles Current joint configuration (radians).
         * @return 6x7 Jacobian matrix.
         */
        Eigen::Matrix<double,6,7> computeJacobian(const std::array<double,7> &joint_angles) const;

        /**
         * @brief Solve IK using damped least squares (Levenberg-Marquardt).
         * @param target_pose Desired EE pose as 4x4 homogeneous transform.
         * @param initial_guess Starting joint configuration (radians).
         * @param max_iterations Maximum solver iterations (default: 300).
         * @param position_tol Convergence threshold for position error in meters (default: 1e-4 = 0.1mm).
         * @param orientation_tol Convergence threshold for orientation error in radians (default: 1e-3 ≈ 0.057°).
         * @return IKResult with success flag, joint angles, and final errors.
         */
        IKResult solveIK(
            const Eigen::Matrix4d& target_pose,
            const std::array<double,7> &initial_guess,
            int max_iterations=300,
            double position_tol=1e-4,
            double orientation_tol=1e-3
        )const;

        /**
         * @brief Linear-algebra core of jointVelocityDLS, with the Jacobian
         *        supplied directly. Exists so the solve can be unit-tested
         *        against hand-computed matrices without a DH table.
         * @param J         6x7 Jacobian [m/rad; rad/rad].
         * @param twist_des Desired twist [m/s; rad/s].
         * @param lambda    Damping, > 0.
         * @return DLSResult struct
         */
        DlsResult solveDLS(const Eigen::Matrix<double,6,7>& J,
                   const Eigen::Matrix<double,6,1>& twist_des,
                   double lambda) const;

        double getToolOffsetZ() const { return tool_offset_z_; }

        /**
         * @brief Yoshikawa manipulability index w = sqrt(det(J·Jᵀ)) — a singularity
         *        MONITOR. Not an observation element, not a policy input. Separate
         *        from DlsResult::manipulability, which stays NaN by D-20 (P22).
         * @param J 6x7 Jacobian at the measured config [m/rad; rad/rad].
         * @return w >= 0. w == 0.0 means "exactly singular" (a real value).
         *         Returns NaN when J is non-finite ("could not compute" — distinct
         *         from 0.0 so the loop never confuses the two).
         */
        double manipulability(const Eigen::Matrix<double,6,7>& J) const;


    private:

        /**
         * @brief Build 4x4 Classical DH transform for one joint.
         *        Order: Rot_z(theta) · Trans_z(d) · Trans_x(a) · Rot_x(alpha)
         * @param alpha Twist angle (radians).
         * @param a Link length (meters).
         * @param d Link offset (meters).
         * @param theta Joint angle + offset (radians).
         * @return 4x4 homogeneous transform from frame (i-1) to frame (i).
         */
        Eigen::Matrix4d dhTransform(double alpha, double a, double d, double theta) const;
        
        // Z0=pushes each angle towards its center
        Eigen::VectorXd jointLimitGradient(const std::array<double,7> & joint_angles) const;
        
        // Classical DH table: 8 rows (row 0 = base frame, rows 1-7 = joints 1-7)
        std::array<DHParam,8> dh_params_;

        std::array<double,7> joint_min_;
        std::array<double,7> joint_max_;
        double tool_offset_z_;   // D-14: tool-dependent, set at construction
        
};
