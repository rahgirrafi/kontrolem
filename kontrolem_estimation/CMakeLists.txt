cmake_minimum_required(VERSION 3.16)
project(kontrolem_estimation LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(kontrolem_control REQUIRED)
find_package(kontrolem_model REQUIRED)

# ROS-free estimator core. Pinocchio stays hidden inside kontrolem_model (pImpl), so
# this library and its users only ever see Eigen.
add_library(kontrolem_estimation SHARED
  src/base_estimator.cpp
  src/invariant_estimator.cpp)
target_include_directories(kontrolem_estimation PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(kontrolem_estimation PUBLIC
  Eigen3::Eigen
  kontrolem_control::kontrolem_control
  kontrolem_model::kontrolem_model)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS kontrolem_estimation
  EXPORT export_kontrolem_estimation
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(DIRECTORY robots/ DESTINATION share/${PROJECT_NAME}/robots)

ament_export_targets(export_kontrolem_estimation HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 kontrolem_control kontrolem_model)

if(BUILD_TESTING)
  # Offline correctness gate (no ROS/Gazebo): the contact-aided filter must (1) hold a
  # static standing state under noisy IMU without drifting, (2) recover a known base
  # velocity from stance leg odometry, and (3) integrate a known acceleration in
  # predict(). Runs a floating quadruped through kontrolem_model (Pinocchio C++).
  add_executable(test_base_estimator test/test_base_estimator.cpp)
  target_link_libraries(test_base_estimator kontrolem_estimation)
  target_compile_definitions(test_base_estimator PRIVATE
    FLOATING_QUAD_URDF="${CMAKE_CURRENT_SOURCE_DIR}/robots/floating_quadruped.urdf")
  add_test(NAME base_estimator COMMAND test_base_estimator)

  # Offline gate for the Right-Invariant EKF (M11): the M8 parity gates (static hold,
  # velocity recovery, predict-integration, tilt recovery) PLUS the milestone proof — a
  # simulated crawl walk with IMU noise + leg-odometry drift + injected contact-sensing
  # flicker, where the InEKF must track the true base tighter than the complementary filter.
  add_executable(test_invariant_estimator test/test_invariant_estimator.cpp)
  target_link_libraries(test_invariant_estimator kontrolem_estimation)
  target_compile_definitions(test_invariant_estimator PRIVATE
    FLOATING_QUAD_URDF="${CMAKE_CURRENT_SOURCE_DIR}/robots/floating_quadruped.urdf")
  add_test(NAME invariant_estimator COMMAND test_invariant_estimator)
endif()

ament_package()
