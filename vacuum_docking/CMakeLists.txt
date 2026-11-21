cmake_minimum_required(VERSION 3.10)
project(vacuum_docking)

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
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(dock_pose_publisher src/dock_pose_publisher.cpp)
ament_target_dependencies(dock_pose_publisher rclcpp geometry_msgs tf2 tf2_msgs)

install(TARGETS dock_pose_publisher DESTINATION lib/${PROJECT_NAME})

ament_package()
