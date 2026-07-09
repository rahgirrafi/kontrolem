cmake_minimum_required(VERSION 3.16)
project(kontrolem_description LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(kontrolem_model REQUIRED)

add_library(kontrolem_description SHARED
  src/cart_pole_sim_system.cpp
  src/floating_base_sim_system.cpp
  src/floating_contact_sim_system.cpp)
target_include_directories(kontrolem_description PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(kontrolem_description PUBLIC
  Eigen3::Eigen
  kontrolem_model::kontrolem_model)
ament_target_dependencies(kontrolem_description PUBLIC
  hardware_interface pluginlib rclcpp)

# Register the simulation SystemInterface as a ros2_control hardware plugin.
pluginlib_export_plugin_description_file(hardware_interface kontrolem_description_plugin.xml)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(TARGETS kontrolem_description
  EXPORT export_kontrolem_description
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_kontrolem_description HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp kontrolem_model)

ament_package()
