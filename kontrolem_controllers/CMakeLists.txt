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
  src/lpv_controller.cpp
  src/lqg_controller.cpp
  src/mpc_controller.cpp
  src/qp_solver.cpp
  src/qp_task_space_controller.cpp
  src/wbc_controller.cpp)
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

  add_executable(test_lqr_double_pole test/test_lqr_double_pole.cpp)
  target_link_libraries(test_lqr_double_pole kontrolem_controllers)
  target_compile_definitions(test_lqr_double_pole PRIVATE
    CDP_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/cart_double_pole.urdf")
  add_test(NAME lqr_double_pole COMMAND test_lqr_double_pole)

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

  # WBC standing + push recovery on the floating quadruped (M4), closed-loop
  # against the model's contact-constrained forward dynamics (the "ground").
  add_executable(test_wbc_standing test/test_wbc_standing.cpp)
  target_link_libraries(test_wbc_standing kontrolem_controllers)
  target_compile_definitions(test_wbc_standing PRIVATE
    FLOATING_QUAD_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/floating_quadruped.urdf")
  add_test(NAME wbc_standing COMMAND test_wbc_standing)

  # M9: WBC tracks a commanded base-pose trajectory (squat/sway/tilt/yaw) with feet planted.
  add_executable(test_wbc_posture test/test_wbc_posture.cpp)
  target_link_libraries(test_wbc_posture kontrolem_controllers)
  target_compile_definitions(test_wbc_posture PRIVATE
    FLOATING_QUAD_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/floating_quadruped.urdf")
  add_test(NAME wbc_posture COMMAND test_wbc_posture)

  # Heterogeneous bumpless-transfer spike (Part D D1/D4): switch LQG <-> MPC at
  # runtime and measure the command discontinuity; naive (stale observer) vs
  # bumpless (observer seeded from state).
  add_executable(test_switching_spike test/test_switching_spike.cpp)
  target_link_libraries(test_switching_spike kontrolem_controllers)
  target_compile_definitions(test_switching_spike PRIVATE
    CART_POLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/cart_pole.urdf")
  add_test(NAME switching_spike COMMAND test_switching_spike)

  # Multi-controller Supervisor: manual LQG<->MPC handoff with bumpless activation
  # + command blend; the runtime meta-controller, proven off-ROS.
  add_executable(test_supervisor test/test_supervisor.cpp)
  target_link_libraries(test_supervisor kontrolem_controllers)
  target_compile_definitions(test_supervisor PRIVATE
    CART_POLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/cart_pole.urdf")
  add_test(NAME supervisor COMMAND test_supervisor)

  # Supervisor automatic fail-forward: LQR (modest trust region) -> MPC on a
  # disturbance that leaves LQR's region, with no manual command.
  add_executable(test_supervisor_auto test/test_supervisor_auto.cpp)
  target_link_libraries(test_supervisor_auto kontrolem_controllers)
  target_compile_definitions(test_supervisor_auto PRIVATE
    CART_POLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/cart_pole.urdf")
  add_test(NAME supervisor_auto COMMAND test_supervisor_auto)

  add_executable(test_supervisor_recover test/test_supervisor_recover.cpp)
  target_link_libraries(test_supervisor_recover kontrolem_controllers)
  target_compile_definitions(test_supervisor_recover PRIVATE
    CART_POLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/cart_pole.urdf")
  add_test(NAME supervisor_recover COMMAND test_supervisor_recover)

  add_executable(test_lpv_arm test/test_lpv_arm.cpp)
  target_link_libraries(test_lpv_arm kontrolem_controllers)
  target_compile_definitions(test_lpv_arm PRIVATE
    ARM2_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/arm2.urdf")
  add_test(NAME lpv_arm COMMAND test_lpv_arm)

  add_executable(test_allocation test/test_allocation.cpp)
  target_link_libraries(test_allocation kontrolem_controllers)
  target_compile_definitions(test_allocation PRIVATE
    CART_POLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/cart_pole.urdf")
  add_test(NAME allocation_audit COMMAND test_allocation)

  add_executable(test_malloc_audit test/test_malloc_audit.cpp)
  target_link_libraries(test_malloc_audit kontrolem_controllers)
  target_compile_definitions(test_malloc_audit PRIVATE
    CART_POLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/cart_pole.urdf"
    FLOATING_QUAD_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/floating_quadruped.urdf")
  add_test(NAME malloc_audit COMMAND test_malloc_audit)
endif()

ament_package()
