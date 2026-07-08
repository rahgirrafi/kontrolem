cmake_minimum_required(VERSION 3.16)
project(kontrolem_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch test
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS)

ament_package()
