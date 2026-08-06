cmake_minimum_required(VERSION 3.16)
project(move_group_interface_probe)

find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(move_group_interface_probe src/move_group_interface_probe.cpp)
ament_target_dependencies(move_group_interface_probe moveit_core moveit_ros_planning_interface rclcpp)

install(TARGETS move_group_interface_probe DESTINATION lib/${PROJECT_NAME})

ament_package()
