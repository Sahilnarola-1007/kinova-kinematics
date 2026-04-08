#pragma once

#include<iostream>
#include<eigen3/Eigen/Dense>
#include<array>

struct DHParam
{
    double alpha{};         //twist angle(radians)
    double a{};             //link length
    double d{};             //link offset(meters)
    double theta_offset{};  //added to joint angle(radians)

};

class KinovaKinematics{

    public:
        // member variables
        static constexpr int NUM_JOINTS=7;
        static constexpr int NUM_FRAMES=8;  // rows 0-7 DH table

        //member functions
        KinovaKinematics();  //populates DH params
        
        // Core: 7 joint angles in → 4x4 matrix out
        Eigen::Matrix4d computeFK(const std::array<double, 7> &joint_angles);

        //Helpers
        Eigen::Vector3d getPosition(const Eigen::Matrix4d &transform);  // Extract [x,y,z] from transform
        Eigen::Matrix3d getRotation(const Eigen::Matrix4d& transform);  // Extract 3x3 rotation from transform
    private:
        
        // Classical DH: builds 4x4 transform from params(Transformation from frame (i-1) → frame (i))
        Eigen::Matrix4d dhTransform(double alpha, double a, double d, double theta);  
        
        // 8 rows: 0 to 7
        std::array<DHParam,8> dh_params_;       
};
