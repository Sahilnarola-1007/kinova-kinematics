#include<kinova_kinematics/KinovaKinematics.hpp>
#include<iostream>
#include<vector>
#include<eigen3/Eigen/Dense>
#include<cmath>
#include<array>

using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::Vector3d;

constexpr double PI = 3.141592653589793;  // file-scope constant, not exposed in header

KinovaKinematics::KinovaKinematics():
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
            joint_min_{-2*PI,-2.2497,-2*PI,-2.5795,-2*PI,-2.0996311,-2*PI},
            joint_max_{2*PI,2.2497,2*PI,2.5795,2*PI,2.0996311,2*PI}
            {}

Matrix4d KinovaKinematics::dhTransform(double alpha, double a,
                                        double d, double theta){
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

Matrix4d KinovaKinematics::computeFK(const std::array<double, 7> &joint_angles){

    Matrix4d T, Temp;
    double theta;

    // Row 0: base frame transform (no joint angle)
    T = dhTransform(dh_params_[0].alpha, dh_params_[0].a,
                    dh_params_[0].d, dh_params_[0].theta_offset);

    for(int i=1; i<NUM_FRAMES; i++)
    {
        // θ(i) for row 1-7: joint angle + DH offset
        theta = joint_angles[i-1] + dh_params_[i].theta_offset;

        // DH transformation matrix for frame i
        Temp = dhTransform(dh_params_[i].alpha, dh_params_[i].a,
                           dh_params_[i].d, theta);
        T = T * Temp;
    }

    Eigen::Matrix4d T_tool = Eigen::Matrix4d::Identity();
    T_tool(2, 3) = 0.12;  // tool offset along local z-axis
    T = T * T_tool;
    return T;
    
}

Vector3d KinovaKinematics::getPosition(const Matrix4d &transform){
    return {transform(0,3),
            transform(1,3),
            transform(2,3)};
}

Matrix3d KinovaKinematics::getRotation(const Matrix4d &transform){
    return transform.block<3,3>(0,0);
}

Eigen::MatrixXd KinovaKinematics::computeJacobian(const std::array<double,7> &joint_angles){

    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 7);  // 6x7 Jacobian, initialized to zero

    // Finite difference step — small enough for accuracy, large enough for float precision
    double eps = 1e-6;

    std::array<double,7> perturbed_joint_angles;
    Matrix4d T0, T1;
    Matrix3d T0_angular, T1_angular, R_delta;
    Vector3d w, p0, p1;

    // Compute FK at current config — used as baseline for all columns
    T0 = computeFK(joint_angles);
    T0_angular = T0.block<3,3>(0,0);
    p0 = T0.block<3,1>(0,3);

    for(int i=0; i<NUM_JOINTS; i++){

        perturbed_joint_angles = joint_angles;

        // Perturb only joint i by eps — all other joints unchanged
        perturbed_joint_angles[i] = joint_angles[i] + eps;

        // FK after perturbation
        T1 = computeFK(perturbed_joint_angles);
        T1_angular = T1.block<3,3>(0,0);
        p1 = T1.block<3,1>(0,3);

        // R_delta: rotation from T0 to T1 (R2 * R1t)
        R_delta = T1_angular * T0_angular.transpose();

        // Linear part: rate of EE position change (m/rad)
        J.block<3,1>(0,i) = (p1 - p0) / eps;

        // Angular part: axis-angle rate via skew-symmetric extraction (rad/rad)
        w << (R_delta(2,1) - R_delta(1,2)) / (2*eps),
             (R_delta(0,2) - R_delta(2,0)) / (2*eps),
             (R_delta(1,0) - R_delta(0,1)) / (2*eps);

        J.block<3,1>(3,i) = w;
    }

    return J;
}

IKResult KinovaKinematics::solveIK(
            const Eigen::Matrix4d& target_pose,
            const std::array<double,7> &initial_guess,
            int max_iterations,
            double position_tol,
            double orientation_tol){

    //double lambda = 0.5;  // damping factor — prevents dq explosion near singularities
    double alpha=0.5;      //gain
    std::array<double,7> joint_angles = initial_guess;

    Matrix4d current_pose;
    Matrix3d R_current, R_target, R_delta;  // rotation matrices for orientation error
    Vector3d dx, P_current, P_target, dw;
    Eigen::VectorXd dq(7),z0(7);
    Eigen::VectorXd de(6);                                      // stacked error: [dp; dw] (6x1)
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd N = Eigen::MatrixXd::Zero(7, 7);
    Eigen::MatrixXd I_7 = Eigen::MatrixXd::Identity(7, 7);       // null space projector identity
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, 7);
    Eigen::MatrixXd J_pinv = Eigen::MatrixXd::Zero(7,6);
    Eigen::MatrixXd I_6 = Eigen::MatrixXd::Identity(6, 6);       // damping identity matrix

    int i = 0;
    while(i < max_iterations){

        // Step 1: compute FK at current joint config
        current_pose = computeFK(joint_angles);

        // Step 2: compute position error dp (3x1, meters)
        P_target  = target_pose.block<3,1>(0,3);
        P_current = current_pose.block<3,1>(0,3);
        dx = P_target - P_current;

        // Step 3: compute orientation error dw via skew-symmetric extraction (3x1, radians)
        R_target  = target_pose.block<3,3>(0,0);
        R_current = current_pose.block<3,3>(0,0);
        R_delta   = R_target * R_current.transpose();  // R2 * R1t

        dw << (R_delta(2,1) - R_delta(1,2)) / 2,
              (R_delta(0,2) - R_delta(2,0)) / 2,
              (R_delta(1,0) - R_delta(0,1)) / 2;

        // Step 4: check convergence
        if(dx.norm() < position_tol && dw.norm() < orientation_tol){
            return {true, joint_angles, dx.norm(), dw.norm()};
        }

        // Step 5: recompute Jacobian at current q (must update every iteration)
        J = computeJacobian(joint_angles);

        // Step 6: stack full 6x1 error vector
        de.block<3,1>(0,0) = dx;  // position error (rows 0-2)
        de.block<3,1>(3,0) = dw;  // orientation error (rows 3-5)

        double lambda = 0.5 * de.norm() + 1e-4;

        // Step 7: damped least squares — dq = Jt * (J*Jt + lambda^2 * I)^-1 * de
        A  = J * J.transpose();
        A  = A + (lambda * lambda) * I_6;
        J_pinv = J.transpose()*A.inverse(); //damped pseudoinverse 
        dq = J_pinv* de; //7x1

        //Step 8: Null-space projector
        N = I_7 - J_pinv * J;

        //Step 9:Secondary gradient
        z0=jointLimitGradient(joint_angles);  //7x1
        
        // Step 10: add null-space term
        dq = dq + alpha * N * z0;
        
        // Step 11: update joint angles with Null space
        for(int j=0; j<NUM_JOINTS; j++){
            joint_angles[j] = joint_angles[j] + dq(j);
        }
        i++;
    }

    // Max iterations reached without convergence
    std::cerr << "[solveIK] Warning: did not converge after " << max_iterations
              << " iterations. pos_err=" << dx.norm()
              << "m  ori_err=" << dw.norm() << "rad\n";

    return {false, joint_angles, dx.norm(), dw.norm()};
}

Eigen::VectorXd KinovaKinematics::jointLimitGradient(const std::array<double,7> & joint_angles){

    //z0= 7x1 vector
    Eigen::VectorXd z0(7);
    double q_mid{};

    for(int i=0;i<NUM_JOINTS;i++){
        q_mid=(joint_max_[i]+joint_min_[i])/2;
        z0(i)=(q_mid-joint_angles[i])/((joint_max_[i]-joint_min_[i])*(joint_max_[i]-joint_min_[i]));
    }

    return z0;
} 
