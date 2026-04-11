/**
 * @file KinovaKinematics.hpp
 * @brief FK, numerical Jacobian, and DLS IK solver for Kinova Gen3 7-DOF arm.
 *        Uses Classical DH convention (8 frames, rows 0-7).
 */

#pragma once

#include<eigen3/Eigen/Dense>
#include<array>
#include<vector>

struct DHParam
{
    double alpha{};         // twist angle (radians)
    double a{};             // link length (meters)
    double d{};             // link offset (meters)
    double theta_offset{};  // added to joint angle (radians)
};

struct IKResult{
    bool success;
    std::array<double,7> joint_states;
    double position_error;      // meters
    double orientation_error;   // radians
};

class KinovaKinematics{

    public:

        // member variables
        static constexpr int NUM_JOINTS=7;
        static constexpr int NUM_FRAMES=8;  // rows 0-7 DH table

        /**
         * @brief Constructor — populates Classical DH parameter table for Kinova Gen3.
         */
        KinovaKinematics();

        /**
         * @brief Compute forward kinematics using Classical DH chain.
         * @param joint_angles 7 joint angles in radians.
         * @return 4x4 homogeneous transform of end-effector in base frame.
         */
        Eigen::Matrix4d computeFK(const std::array<double, 7> &joint_angles);

        /**
         * @brief Extract [x, y, z] position from a 4x4 homogeneous transform.
         * @param transform 4x4 homogeneous transform matrix.
         * @return 3x1 position vector (meters).
         */
        Eigen::Vector3d getPosition(const Eigen::Matrix4d &transform);

        /**
         * @brief Extract 3x3 rotation matrix from a 4x4 homogeneous transform.
         * @param transform 4x4 homogeneous transform matrix.
         * @return 3x3 rotation matrix.
         */
        Eigen::Matrix3d getRotation(const Eigen::Matrix4d& transform);

        /**
         * @brief Compute numerical Jacobian via finite differences.
         *        Top 3 rows: linear velocity (m/rad).
         *        Bottom 3 rows: angular velocity (rad/rad).
         * @param joint_angles Current joint configuration (radians).
         * @return 6x7 Jacobian matrix.
         */
        Eigen::MatrixXd computeJacobian(const std::array<double,7> &joint_angles);

        /**
         * @brief Solve IK using damped least squares (Levenberg-Marquardt).
         * @param target_pose Desired EE pose as 4x4 homogeneous transform.
         * @param initial_guess Starting joint configuration (radians).
         * @param max_iterations Maximum solver iterations (default: 100).
         * @param position_tol Convergence threshold for position error in meters (default: 1e-4 = 0.1mm).
         * @param orientation_tol Convergence threshold for orientation error in radians (default: 1e-3 ≈ 0.057°).
         * @return IKResult with success flag, joint angles, and final errors.
         */
        IKResult solveIK(
            const Eigen::Matrix4d& target_pose,
            const std::array<double,7> &initial_guess,
            int max_iterations=100,
            double position_tol=1e-4,
            double orientation_tol=1e-3
        );

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
        Eigen::Matrix4d dhTransform(double alpha, double a, double d, double theta);

        // Classical DH table: 8 rows (row 0 = base frame, rows 1-7 = joints 1-7)
        std::array<DHParam,8> dh_params_;
};
