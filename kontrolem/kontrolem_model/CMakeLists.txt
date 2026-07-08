cmake_minimum_required(VERSION 3.16)
project(kontrolem_model LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(pinocchio REQUIRED)

# Shared so Pinocchio (linked PRIVATE, hidden behind pImpl) is fully
# encapsulated: downstream links libkontrolem_model.so and never sees a
# Pinocchio symbol at compile OR link time.
add_library(kontrolem_model SHARED src/robot_model.cpp)
target_include_directories(kontrolem_model PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(kontrolem_model
  PUBLIC Eigen3::Eigen
  PRIVATE pinocchio::pinocchio_parsers)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS kontrolem_model
  EXPORT export_kontrolem_model
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(DIRECTORY robots/ DESTINATION share/${PROJECT_NAME}/robots)

# Public interface exposes only Eigen; Pinocchio is deliberately NOT exported.
ament_export_targets(export_kontrolem_model HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)

if(BUILD_TESTING)
  # Dependency-free correctness check (analytic linearization vs finite diff),
  # registered as a ctest so `colcon test` runs it.
  add_executable(test_linearize_fd test/test_linearize_fd.cpp)
  target_link_libraries(test_linearize_fd kontrolem_model)
  target_compile_definitions(test_linearize_fd PRIVATE
    CART_POLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/robots/cart_pole.urdf")
  add_test(NAME linearize_fd COMMAND test_linearize_fd)

  # Floating-base model service (M3.1): SE(3) dims, CoM, manifold integrate.
  add_executable(test_floating_base test/test_floating_base.cpp)
  target_link_libraries(test_floating_base kontrolem_model)
  target_compile_definitions(test_floating_base PRIVATE
    FLOATING_URDF="${CMAKE_CURRENT_SOURCE_DIR}/robots/floating_biped.urdf")
  add_test(NAME floating_base COMMAND test_floating_base)

  # Contact Jacobian (M3.2): analytic vs finite-difference of foot world position.
  add_executable(test_contact_jacobian test/test_contact_jacobian.cpp)
  target_link_libraries(test_contact_jacobian kontrolem_model)
  target_compile_definitions(test_contact_jacobian PRIVATE
    FLOATING_URDF="${CMAKE_CURRENT_SOURCE_DIR}/robots/floating_biped.urdf")
  add_test(NAME contact_jacobian COMMAND test_contact_jacobian)

  # rollout + linearize_along (M2.5 LTV/prediction model queries).
  add_executable(test_rollout test/test_rollout.cpp)
  target_link_libraries(test_rollout kontrolem_model)
  target_compile_definitions(test_rollout PRIVATE
    CART_POLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/robots/cart_pole.urdf")
  add_test(NAME rollout COMMAND test_rollout)
endif()

ament_package()
