cmake_minimum_required(VERSION 3.16)
project(kontrolem_controllers LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(kontrolem_control REQUIRED)
find_package(kontrolem_model REQUIRED)
find_package(osqp REQUIRED)

add_library(kontrolem_controllers SHARED
  src/lqr_controller.cpp
  src/lqg_controller.cpp
  src/mpc_controller.cpp
  src/qp_solver.cpp
  src/qp_task_space_controller.cpp)
target_include_directories(kontrolem_controllers PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(kontrolem_controllers PUBLIC
  Eigen3::Eigen
  kontrolem_control::kontrolem_control
  kontrolem_model::kontrolem_model
  PRIVATE osqp::osqp)   # OSQP hidden behind the QpSolver pImpl -> PRIVATE

install(DIRECTORY include/ DESTINATION include)
install(TARGETS kontrolem_controllers
  EXPORT export_kontrolem_controllers
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)

ament_export_targets(export_kontrolem_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 kontrolem_control kontrolem_model)

if(BUILD_TESTING)
  add_executable(test_lqr_cartpole test/test_lqr_cartpole.cpp)
  target_link_libraries(test_lqr_cartpole kontrolem_controllers)
  target_compile_definitions(test_lqr_cartpole PRIVATE
    CART_POLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/cart_pole.urdf")
  add_test(NAME lqr_cartpole COMMAND test_lqr_cartpole)

  add_executable(test_lqr_tracking test/test_lqr_tracking.cpp)
  target_link_libraries(test_lqr_tracking kontrolem_controllers)
  target_compile_definitions(test_lqr_tracking PRIVATE
    CART_POLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/cart_pole.urdf")
  add_test(NAME lqr_tracking COMMAND test_lqr_tracking)

  add_executable(test_mpc_cartpole test/test_mpc_cartpole.cpp)
  target_link_libraries(test_mpc_cartpole kontrolem_controllers)
  target_compile_definitions(test_mpc_cartpole PRIVATE
    CART_POLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/cart_pole.urdf")
  add_test(NAME mpc_cartpole COMMAND test_mpc_cartpole)

  add_executable(test_mpc_tracking test/test_mpc_tracking.cpp)
  target_link_libraries(test_mpc_tracking kontrolem_controllers)
  target_compile_definitions(test_mpc_tracking PRIVATE
    CART_POLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/cart_pole.urdf")
  add_test(NAME mpc_tracking COMMAND test_mpc_tracking)

  add_executable(test_lqg_cartpole test/test_lqg_cartpole.cpp)
  target_link_libraries(test_lqg_cartpole kontrolem_controllers)
  target_compile_definitions(test_lqg_cartpole PRIVATE
    CART_POLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/cart_pole.urdf")
  add_test(NAME lqg_cartpole COMMAND test_lqg_cartpole)

  add_executable(test_qp_cartpole test/test_qp_cartpole.cpp)
  target_link_libraries(test_qp_cartpole kontrolem_controllers)
  target_compile_definitions(test_qp_cartpole PRIVATE
    CART_POLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/cart_pole.urdf")
  add_test(NAME qp_cartpole COMMAND test_qp_cartpole)

  add_executable(test_allocation test/test_allocation.cpp)
  target_link_libraries(test_allocation kontrolem_controllers)
  target_compile_definitions(test_allocation PRIVATE
    CART_POLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/cart_pole.urdf")
  add_test(NAME allocation_audit COMMAND test_allocation)

  add_executable(test_malloc_audit test/test_malloc_audit.cpp)
  target_link_libraries(test_malloc_audit kontrolem_controllers)
  target_compile_definitions(test_malloc_audit PRIVATE
    CART_POLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/cart_pole.urdf")
  add_test(NAME malloc_audit COMMAND test_malloc_audit)
endif()

ament_package()
