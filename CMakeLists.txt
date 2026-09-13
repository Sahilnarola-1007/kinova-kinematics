cmake_minimum_required(VERSION 3.16)
project(kinova_kinematics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# ── Library ──────────────────────────────────────────────────────────────────
add_library(kinova_kinematics
  src/KinovaKinematics.cpp
  src/SafetyFilter.cpp)
target_include_directories(kinova_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(kinova_kinematics PUBLIC Eigen3::Eigen)
# PUBLIC so the C++17 requirement propagates to every target that links the
# library below (both tools and both gtests) — no need to repeat it per target.
target_compile_features(kinova_kinematics PUBLIC cxx_std_17)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Install library + export target
install(
  TARGETS kinova_kinematics
  EXPORT kinova_kinematicsTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  INCLUDES DESTINATION include)

ament_export_targets(kinova_kinematicsTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_export_include_directories(include)

# ── Manual-run tools (own main(); run by hand, NOT by `colcon test`) ──────────
#   test_fk                 : FK / IK smoke check
#   benchmark_dls           : Step 6.2 IK latency — run pinned + SCHED_FIFO
#   benchmark_safety_filter : Step 6.4 filter latency, same protocol
# No Kortex dependency anywhere in this package: USE_KORTEX_MOCK is not read here.

add_executable(test_fk tests/test_fk.cpp)
target_link_libraries(test_fk kinova_kinematics)

add_executable(benchmark_dls tests/benchmark_dls.cpp)
target_link_libraries(benchmark_dls kinova_kinematics)

add_executable(benchmark_safety_filter tests/benchmark_safety_filter.cpp)
target_link_libraries(benchmark_safety_filter kinova_kinematics)

# ── Unit tests (gtest; discovered and run by `colcon test`) ───────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_dls tests/test_dls.cpp)
  target_link_libraries(test_dls kinova_kinematics)

  ament_add_gtest(test_safety_filter tests/test_safety_filter.cpp)
  target_link_libraries(test_safety_filter kinova_kinematics)
endif()

ament_package()