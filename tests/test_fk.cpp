#include <kinova_kinematics/KinovaKinematics.hpp>
#include <iostream>
#include <array>

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
    std::array<double, 7> j1_90 = {M_PI/2, 0, 0, 0, 0, 0, 0};
    auto T2 = fk.computeFK(j1_90);
    auto pos2 = fk.getPosition(T2);
    std::cout << "\n=== Joint 1 at π/2 ===" << std::endl;
    std::cout << "Position: " << pos2.transpose() << std::endl;
    // Expect: z same as zero config (1.1873)
    // x and y swapped (arm rotated 90° about base)

    // Test 3: Symmetry check
    std::array<double, 7> pos_j1 = { 0.3, 0, 0, 0, 0, 0, 0};
    std::array<double, 7> neg_j1 = {-0.3, 0, 0, 0, 0, 0, 0};
    auto p1 = fk.getPosition(fk.computeFK(pos_j1));
    auto p2 = fk.getPosition(fk.computeFK(neg_j1));
    std::cout << "\n=== Symmetry Check ===" << std::endl;
    std::cout << "+0.3: " << p1.transpose() << std::endl;
    std::cout << "-0.3: " << p2.transpose() << std::endl;
    // Expect: z equal, x opposite sign

    return 0;
}