cmake_minimum_required(VERSION 3.16)
project(kontrolem_gz LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ignition-gazebo6 REQUIRED)

add_library(kontrolem_gz SHARED
  src/gz_base_state_system.cpp)
target_include_directories(kontrolem_gz PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(kontrolem_gz PUBLIC
  gz_ros2_control hardware_interface pluginlib rclcpp)
# Gazebo ECM (Link/Model/EntityComponentManager) — read base_link ground truth.
target_link_libraries(kontrolem_gz PUBLIC ignition-gazebo6::core)

# Register as a gz_ros2_control hardware plugin (GazeboSimSystemInterface base).
pluginlib_export_plugin_description_file(gz_ros2_control kontrolem_gz_plugin.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS kontrolem_gz
  EXPORT export_kontrolem_gz
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_kontrolem_gz HAS_LIBRARY_TARGET)
ament_export_dependencies(gz_ros2_control hardware_interface pluginlib rclcpp)

ament_package()
