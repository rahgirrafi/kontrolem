cmake_minimum_required(VERSION 3.16)
project(kontrolem_state_bridge LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(kontrolem_state_bridge SHARED
  src/odometry_base_bridge.cpp)
target_include_directories(kontrolem_state_bridge PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
ament_target_dependencies(kontrolem_state_bridge PUBLIC
  hardware_interface pluginlib rclcpp nav_msgs realtime_tools)

# Register the topic->interface bridge as a ros2_control hardware plugin.
pluginlib_export_plugin_description_file(hardware_interface kontrolem_state_bridge_plugin.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS kontrolem_state_bridge
  EXPORT export_kontrolem_state_bridge
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_kontrolem_state_bridge HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp nav_msgs realtime_tools)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Offline unit test of the pure Odometry->base-state mapping (no ROS runtime).
  ament_add_gtest(test_odom_to_base test/test_odom_to_base.cpp)
  target_include_directories(test_odom_to_base PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include")
endif()

ament_package()
