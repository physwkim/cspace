cmake_minimum_required(VERSION 3.22)
project(moveit_oracle CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(moveit_core REQUIRED)
find_package(urdfdom REQUIRED)
find_package(srdfdom REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(random_numbers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(octomap REQUIRED)
# For the ik op's vendored KDL::ChainIkSolverVelMimicSVD (see
# src/third_party/kdl_kinematics_plugin/) -- the same two packages
# moveit_kinematics/CMakeLists.txt itself finds, without the rest of that
# package's rclcpp/pluginlib/moveit_ros_planning dependency tree.
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
# For the acceleration_filter op's rclcpp::Node and pluginlib::ClassLoader --
# see oracle.cpp's file header and that op's own comment for why this one op
# is the exception to the "no ROS node" rule the comment above describes.
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

add_executable(moveit_oracle
  src/oracle.cpp
  src/third_party/kdl_kinematics_plugin/chainiksolver_vel_mimic_svd.cpp
)

# Namespaced targets, not bare library names: moveit_core exports
# moveit_core::moveit_robot_model, and a bare `moveit_robot_model` is treated by
# CMake as a raw -l flag, which links but carries no include directories -- the
# build then fails on a missing moveit/robot_model/robot_model.hpp.
target_link_libraries(moveit_oracle
  PRIVATE
    moveit_core::moveit_robot_model
    moveit_core::moveit_robot_state
    moveit_core::moveit_robot_trajectory
    moveit_core::moveit_trajectory_processing
    moveit_core::moveit_collision_detection
    moveit_core::moveit_collision_detection_fcl
    moveit_core::moveit_distance_field
    moveit_core::moveit_collision_distance_field
    moveit_core::moveit_dynamics_solver
    moveit_core::moveit_kinematic_constraints
    moveit_core::moveit_planning_scene
    moveit_core::moveit_smoothing_base
    urdfdom::urdf_parser
    srdfdom::srdfdom
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    geometric_shapes::geometric_shapes
    ${random_numbers_LIBRARIES}
    ${moveit_msgs_TARGETS}
    ${OCTOMAP_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    kdl_parser::kdl_parser
    rclcpp::rclcpp
    pluginlib::pluginlib
)
# octomap ships old-style CMake variables (OCTOMAP_INCLUDE_DIRS/LIBRARIES),
# not an imported target, per its own octomap-config.cmake. orocos_kdl is the
# same shape.
target_include_directories(moveit_oracle PRIVATE
  ${random_numbers_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

install(TARGETS moveit_oracle RUNTIME DESTINATION bin)
