cmake_minimum_required(VERSION 3.10)
project(vacuum_safety)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vacuum_interfaces REQUIRED)

# The decisions, with no ROS in them, so every rule can be unit tested with made-up inputs.
add_library(gate_logic src/gate_logic.cpp)
target_include_directories(gate_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# The node: collects inputs from ROS, asks the logic, publishes the wheel command.
add_executable(safety_gate src/safety_gate_node.cpp)
target_link_libraries(safety_gate gate_logic)
ament_target_dependencies(safety_gate rclcpp geometry_msgs sensor_msgs std_msgs vacuum_interfaces)

install(TARGETS safety_gate DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gate_logic test/test_gate_logic.cpp)
  target_link_libraries(test_gate_logic gate_logic)
endif()

ament_package()
