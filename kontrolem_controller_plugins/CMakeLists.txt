cmake_minimum_required(VERSION 3.16)
project(kontrolem_controller_plugins LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(kontrolem_control REQUIRED)
find_package(kontrolem_controllers REQUIRED)
find_package(kontrolem_model REQUIRED)

# The pluginlib export shim: one shared library that registers every shipped
# control-law factory under kontrolem_control::ControllerFactory. The factory
# logic itself lives in (ROS-free) kontrolem_controllers; this only exports it.
add_library(kontrolem_controller_plugins SHARED src/plugins.cpp)
target_link_libraries(kontrolem_controller_plugins PUBLIC
  kontrolem_control::kontrolem_control
  kontrolem_controllers::kontrolem_controllers
  kontrolem_model::kontrolem_model)
ament_target_dependencies(kontrolem_controller_plugins PUBLIC pluginlib)

# Register the plugins under the kontrolem_control base package so a
# pluginlib::ClassLoader<kontrolem_control::ControllerFactory> discovers them.
pluginlib_export_plugin_description_file(kontrolem_control plugins.xml)

install(TARGETS kontrolem_controller_plugins
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
