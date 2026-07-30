cmake_minimum_required(VERSION 3.16)
project(kontrolem_controller_example LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(kontrolem_control REQUIRED)
find_package(kontrolem_model REQUIRED)

# A single third-party package: the controller, its factory, and the pluginlib
# export all live together (a real out-of-tree law does NOT split them — that
# split only exists in the framework to keep the core ROS-free).
add_library(kontrolem_controller_example SHARED src/example_controller.cpp)
target_include_directories(kontrolem_controller_example PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(kontrolem_controller_example PUBLIC
  Eigen3::Eigen
  kontrolem_control::kontrolem_control
  kontrolem_model::kontrolem_model)
ament_target_dependencies(kontrolem_controller_example PUBLIC pluginlib)

# Register under the kontrolem_control base package so the runtime's
# ClassLoader<kontrolem_control::ControllerFactory> discovers it.
pluginlib_export_plugin_description_file(kontrolem_control plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS kontrolem_controller_example
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  # The acceptance proof: discover + build "example" through the SAME
  # pluginlib::ClassLoader the runtime uses, with zero framework edits. It does
  # NOT link the example library — pluginlib dlopens it by name at runtime, which
  # is exactly the drop-in path (needs the workspace sourced so the ament index +
  # plugin .so are found).
  add_executable(test_example_discoverable test/test_example_discoverable.cpp)
  target_link_libraries(test_example_discoverable
    kontrolem_control::kontrolem_control kontrolem_model::kontrolem_model)
  ament_target_dependencies(test_example_discoverable pluginlib)
  target_compile_definitions(test_example_discoverable PRIVATE
    ARM2_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/arm2.urdf")
  add_test(NAME example_discoverable COMMAND test_example_discoverable)
endif()

ament_package()
