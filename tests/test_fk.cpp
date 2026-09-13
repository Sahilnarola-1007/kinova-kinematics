// test_fk.cpp — FK / Jacobian / IK round-trip smoke check, own main() (not gtest).
// OBS side (computeFK) plus offline solveIK. Prints values for eyeballing against
// design.md §8.1: flange z = 1.1873 m at q = 0 [COMPUTED]. q = 0 is singular (rank 3),
// so the Jacobian printed there validates nothing about conditioning.
#include <kinova_kinematics/KinovaKinematics.hpp>
#include <iostream>
#include <array>
#include<random>

int main() {
    KinovaKinematics fk;

    // Test 1: Zero config
    std::array<double, 7> zero = {0, 0, 0, 0, 0, 0, 0};
    auto T = fk.computeFK(zero);
    auto pos = fk.getPosition(T);

    std::cout << "=== Zero Config ===" << std::endl;
    std::cout << "Full transform:\n" << T << std::endl;
    std::cout << "Position: " << pos.transpose() << std::endl;

    // Test 2: Joint 1 rotation by π/2
    constexpr double kPi = 3.14159265358979323846;   // M_PI is POSIX, not ISO C++
    std::array<double, 7> j1_90 = {kPi/2, 0, 0, 0, 0, 0, 0};
    auto T2 = fk.computeFK(j1_90);
    auto pos2 = fk.getPosition(T2);
    std::cout << "\n=== Joint 1 at π/2 ===" << std::endl;
    std::cout << "Position: " << pos2.transpose() << std::endl;
    // Expect z unchanged (1.1873 m); x/y rotated 90° about base z.

    // Test 3: Symmetry check
    std::array<double, 7> pos_j1 = { 0.3, 0, 0, 0, 0, 0, 0};
    std::array<double, 7> neg_j1 = {-0.3, 0, 0, 0, 0, 0, 0};
    auto p1 = fk.getPosition(fk.computeFK(pos_j1));
    auto p2 = fk.getPosition(fk.computeFK(neg_j1));
    std::cout << "\n=== Symmetry Check ===" << std::endl;
    std::cout << "+0.3: " << p1.transpose() << std::endl;
    std::cout << "-0.3: " << p2.transpose() << std::endl;
    // Expect equal z, opposite-sign x.

    // Test 4: Jacobian at home position
        std::cout << "\n=== Jacobian at Zero Config ===" << std::endl;
        auto J = fk.computeJacobian(zero);
        std::cout << J << std::endl;
    
    // IK round-trip: FK(q_rand) → solveIK from zero → FK again, 1 mm criterion. Fixed seed.
    std::cout << "\n=== IK Round-Trip Test ===" << std::endl;
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> dist(-1.5, 1.5);

    int success = 0;
    for(int t = 0; t < 100; t++){
        // Random joint config
        std::array<double,7> q_rand;
        for(auto& v : q_rand) v = dist(rng);

        // FK → target pose
        Eigen::Matrix4d target = fk.computeFK(q_rand);

        // IK from zero guess
        std::array<double,7> zero_guess = {0,0,0,0,0,0,0};
        IKResult result = fk.solveIK(target, zero_guess);

        if(result.success){
            // FK again to verify
            Eigen::Matrix4d T_check = fk.computeFK(result.joint_states);
            double pos_err = (T_check.block<3,1>(0,3) - target.block<3,1>(0,3)).norm();
            if(pos_err < 1e-3) success++;
        }
    }

    std::cout << "Passed: " << success << "/100" << std::endl;
    std::cout << "Expect: 90+/100" << std::endl;   // [DESIGN] acceptance threshold, not a measured count
    return 0;
}