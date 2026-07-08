cmake_minimum_required(VERSION 3.16)
project(kontrolem_control LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(kontrolem_model REQUIRED)

# Header-only contract: no compiled sources, just the interface.
add_library(kontrolem_control INTERFACE)
target_include_directories(kontrolem_control INTERFACE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>")
target_link_libraries(kontrolem_control INTERFACE
  Eigen3::Eigen
  kontrolem_model::kontrolem_model)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS kontrolem_control EXPORT export_kontrolem_control)

ament_export_targets(export_kontrolem_control)
ament_export_dependencies(Eigen3 kontrolem_model)

if(BUILD_TESTING)
  # Proves the contract is implementable and compiles (trivial NullController).
  add_executable(test_contract_compiles test/test_contract_compiles.cpp)
  target_link_libraries(test_contract_compiles kontrolem_control)
  # Reuse the cart-pole shipped by kontrolem_model (same repo, source tree).
  target_compile_definitions(test_contract_compiles PRIVATE
    CART_POLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../kontrolem_model/robots/cart_pole.urdf")
  add_test(NAME contract_compiles COMMAND test_contract_compiles)
endif()

ament_package()
