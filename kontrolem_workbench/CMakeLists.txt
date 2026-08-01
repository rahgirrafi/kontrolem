cmake_minimum_required(VERSION 3.16)
project(kontrolem_workbench LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(kontrolem_control REQUIRED)
find_package(kontrolem_controllers REQUIRED)
find_package(kontrolem_model REQUIRED)

# --- libworkbench: the ROS-free engine (formatting, codec, lint, scaffold,
# simulator, metrics, analyzers). No pluginlib here — everything is testable
# offline against directly-linked factories.
add_library(workbench
  src/spec_format.cpp
  src/codec.cpp
  src/lint.cpp
  src/scaffold.cpp
  src/simulator.cpp
  src/metrics.cpp
  src/analyzers.cpp)
target_include_directories(workbench PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(workbench PUBLIC
  kontrolem_control::kontrolem_control
  kontrolem_controllers::kontrolem_controllers
  kontrolem_model::kontrolem_model
  yaml-cpp)

# --- kontrolem_setup: the CLI. The ONLY target that touches pluginlib (it
# discovers laws through the same registry the runtime uses).
add_executable(kontrolem_setup src/main.cpp)
target_link_libraries(kontrolem_setup workbench)
ament_target_dependencies(kontrolem_setup pluginlib)

install(TARGETS kontrolem_setup
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/plot_run.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_spec_format test/test_spec_format.cpp)
  target_link_libraries(test_spec_format workbench)
  add_test(NAME spec_format COMMAND test_spec_format)

  add_executable(test_lint test/test_lint.cpp)
  target_link_libraries(test_lint workbench)
  target_compile_definitions(test_lint PRIVATE
    ARM2_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/arm2.urdf"
    SCRATCH_DIR="${CMAKE_CURRENT_BINARY_DIR}")
  add_test(NAME lint COMMAND test_lint)

  add_executable(test_simulator test/test_simulator.cpp)
  target_link_libraries(test_simulator workbench)
  target_compile_definitions(test_simulator PRIVATE
    CART_POLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/cart_pole.urdf"
    ARM2_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/arm2.urdf"
    SCRATCH_DIR="${CMAKE_CURRENT_BINARY_DIR}")
  add_test(NAME simulator COMMAND test_simulator)

  add_executable(test_analyzers test/test_analyzers.cpp)
  target_link_libraries(test_analyzers workbench)
  target_compile_definitions(test_analyzers PRIVATE
    CART_POLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/cart_pole.urdf"
    ARM2_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/arm2.urdf")
  add_test(NAME analyzers COMMAND test_analyzers)

  # Integration: needs a sourced workspace (ament index) at run time.
  add_executable(test_discovery test/test_discovery.cpp)
  target_link_libraries(test_discovery kontrolem_control::kontrolem_control)
  ament_target_dependencies(test_discovery pluginlib)
  add_test(NAME discovery COMMAND test_discovery)
endif()

ament_package()
