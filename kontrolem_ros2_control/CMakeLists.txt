cmake_minimum_required(VERSION 3.16)
project(kontrolem_ros2_control LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kontrolem_msgs REQUIRED)
find_package(kontrolem_control REQUIRED)
find_package(kontrolem_model REQUIRED)
find_package(kontrolem_controllers REQUIRED)

add_library(kontrolem_ros2_control SHARED
  src/kontrolem_controller.cpp
  src/base_state_sensor.cpp
  src/contact_sensor.cpp
  src/floating_state_probe.cpp)
target_include_directories(kontrolem_ros2_control PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(kontrolem_ros2_control PUBLIC
  kontrolem_control::kontrolem_control
  kontrolem_model::kontrolem_model
  kontrolem_controllers::kontrolem_controllers)
ament_target_dependencies(kontrolem_ros2_control PUBLIC
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle
  realtime_tools std_msgs kontrolem_msgs)

# Register as a ros2_control controller plugin.
pluginlib_export_plugin_description_file(controller_interface kontrolem_ros2_control_plugin.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS kontrolem_ros2_control
  EXPORT export_kontrolem_ros2_control
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_kontrolem_ros2_control HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle
  realtime_tools kontrolem_msgs
  kontrolem_control kontrolem_model kontrolem_controllers)

ament_package()
