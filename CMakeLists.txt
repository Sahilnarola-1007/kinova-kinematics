cmake_minimum_required(VERSION 3.16)
project(kinova_kinematics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# Library
add_library(kinova_kinematics src/KinovaKinematics.cpp)
target_include_directories(kinova_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(kinova_kinematics PUBLIC Eigen3::Eigen)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Install library + export target
install(
  TARGETS kinova_kinematics
  EXPORT kinova_kinematicsTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(kinova_kinematicsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_export_include_directories(include)

# Test executable
add_executable(test_fk tests/test_fk.cpp)
target_link_libraries(test_fk kinova_kinematics)

ament_package()