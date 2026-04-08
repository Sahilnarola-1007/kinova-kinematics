#include<kinova_kinematics/KinovaKinematics.hpp>
#include<iostream>
#include<vector>
#include<eigen3/Eigen/Dense>
#include<cmath>
#include<array>

using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::Vector3d;

constexpr double PI = 3.141592653589793;  //only this file specific

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
            }}
            {}

Matrix4d KinovaKinematics::dhTransform(double alpha,double a,
                                        double d,double theta){
            double ct=std::cos(theta);
            double st=std::sin(theta);
            double ca=std::cos(alpha);
            double sa=std::sin(alpha);

            Matrix4d T;
            
            //classical DH:Rot_z(θ) · Trans_z(d) · Trans_x(a) · Rot_x(α)
            T << ct, -st*ca,  st*sa, a*ct,
                st,  ct*ca, -ct*sa, a*st,
                0,   sa,     ca,    d,
                0,   0,      0,     1;


            return T;
}

Matrix4d KinovaKinematics::computeFK(const std::array<double, 7> & joint_angles){
    
    Matrix4d T,Temp;
    double theta;

    T= dhTransform(dh_params_[0].alpha, dh_params_[0].a,
                  dh_params_[0].d, dh_params_[0].theta_offset);  // Just 0: No joint angle       
    
    for(int i=1;i<NUM_FRAMES;i++)
    {
        //θ(i) for raw 1-7
        theta=joint_angles[i-1] + dh_params_[i].theta_offset;
        
        //DH tranformation matrix
        Temp=dhTransform(dh_params_[i].alpha, dh_params_[i].a,
                  dh_params_[i].d, theta);
        T=T*Temp;         
    }

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


