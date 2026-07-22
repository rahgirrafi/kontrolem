cmake_minimum_required(VERSION 3.16)
project(kontrolem_locomotion LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(kontrolem_control REQUIRED)

# Header-only concrete gait patterns: the implementation side of the GaitSource
# seam that kontrolem_control defines (mirrors control-contract / controllers-impl).
add_library(kontrolem_locomotion INTERFACE)
target_include_directories(kontrolem_locomotion INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(kontrolem_locomotion INTERFACE
  Eigen3::Eigen
  kontrolem_control::kontrolem_control)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS kontrolem_locomotion EXPORT export_kontrolem_locomotion)

ament_export_targets(export_kontrolem_locomotion)
ament_export_dependencies(Eigen3 kontrolem_control)

ament_package()
