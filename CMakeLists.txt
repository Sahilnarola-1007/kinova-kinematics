cmake_minimum_required(VERSION 3.16)
project(kinova_kinematics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror")
set(KINEMATICS_SRC src/KinovaKinematics.cpp)

find_package(Eigen3 REQUIRED)

# Library 
add_library(kinova_kinematics src/KinovaKinematics.cpp)
target_include_directories(kinova_kinematics PUBLIC include)
target_link_libraries(kinova_kinematics Eigen3::Eigen)

# Test executable
add_executable(test_fk tests/test_fk.cpp)
target_link_libraries(test_fk kinova_kinematics)