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
# For the plan op's OMPL RRTConnect baseline (Phase 7's completion condition).
# ompl ships as a prebuilt ROS package in the base image, so this adds nothing
# to MOVEIT2_PACKAGES -- see oracle.cpp's `plan()` for why it links plain OMPL
# rather than moveit_planners_ompl.
find_package(ompl REQUIRED)
# For the pilz_trajectory op's three generators. Unlike ompl this is a moveit2
# package, so it is built from source and rides MOVEIT2_PACKAGES -- see
# src-digest.sh's own comment on that line.
find_package(pilz_industrial_motion_planner REQUIRED)
# For the chomp_quad_cost_inverse op's real ChompCost and the chomp_plan op's
# real ChompPlanner::solve -- see oracle.cpp's include comment for why the
# upstream class and not a transcription.
find_package(chomp_motion_planner REQUIRED)
# For the stomp_plan op, whose three dependencies are all already in the image
# but none of which arrive as a built `moveit_planners_stomp`: `stomp` is the
# prebuilt ros-industrial core solver, `rsl` is the RNG both C++ planners draw
# from, and generate_parameter_library turns upstream's own res/stomp_moveit.yaml
# into the Params struct StompPlanningContext takes. See STOMP_MOVEIT_SRC below.
find_package(stomp REQUIRED)
find_package(rsl REQUIRED)
find_package(generate_parameter_library REQUIRED)

# `moveit_planners_stomp` is not one of the packages this image builds
# (MOVEIT2_PACKAGES in src-digest.sh), because adding it would invalidate the
# Dockerfile's expensive first stage and rebuild moveit_core from scratch for
# one small package. It does not have to be built to be *used*: the Dockerfile
# copies the whole moveit2 tree to /ws/src/moveit2 at the pinned SHA and leaves
# it there, so the planning context compiles straight from upstream's sources.
#
# The path is absolute and image-only on purpose. This project is only ever
# configured inside the oracle image (the Dockerfile's second stage is its sole
# caller), and a relative path would be a claim about a host layout that has no
# bearing on where these sources actually live. Asserted rather than assumed: a
# missing directory here means the base image changed shape, and failing at
# configure time with that sentence is much cheaper than a link error later.
set(STOMP_MOVEIT_SRC /ws/src/moveit2/moveit_planners/stomp)
if(NOT EXISTS ${STOMP_MOVEIT_SRC}/src/stomp_moveit_planning_context.cpp)
  message(FATAL_ERROR
    "upstream STOMP sources not found at ${STOMP_MOVEIT_SRC} -- the stomp_plan op "
    "compiles them from the moveit2 tree the image ships at /ws/src")
endif()

find_package(tf2_eigen REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# The macro pins its yaml argument under the calling project's source dir --
# its own body does `set(YAML_FILE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/${YAML_FILE})`
# unconditionally -- so an absolute path silently becomes
# /ws/oracle-src//ws/src/... and the build stops on a missing rule. The path
# stays absolute everywhere else in this file; only this one call gets the
# relative spelling the macro requires, derived rather than hand-written so the
# two cannot drift.
file(RELATIVE_PATH STOMP_MOVEIT_YAML_REL
     ${CMAKE_CURRENT_SOURCE_DIR} ${STOMP_MOVEIT_SRC}/res/stomp_moveit.yaml)
generate_parameter_library(stomp_moveit_parameters ${STOMP_MOVEIT_YAML_REL})

# generate_parameter_library writes its header to
# ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}/<lib>.hpp -- keyed on the
# *consuming* project's name, which here is moveit_oracle. Upstream's own
# sources include it as <moveit_planners_stomp/stomp_moveit_parameters.hpp>
# (both stomp_moveit_planning_context.{hpp,cpp} do), the name it has when the
# package builds itself. A directory symlink gives the generated header that
# second name without editing upstream's include lines, which is the thing that
# must not happen: the whole point of compiling these sources is that they are
# upstream's, unmodified.
file(CREATE_LINK ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME}
                 ${CMAKE_CURRENT_BINARY_DIR}/include/moveit_planners_stomp
     SYMBOLIC)

add_executable(moveit_oracle
  src/oracle.cpp
  src/pilz_polyline_factory.cpp
  src/third_party/kdl_kinematics_plugin/chainiksolver_vel_mimic_svd.cpp
  ${STOMP_MOVEIT_SRC}/src/stomp_moveit_planning_context.cpp
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
    moveit_core::moveit_kinematics_metrics
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
    ${OMPL_LIBRARIES}
    # `planning_context_loader_{ptp,lin,circ,polyline}` carry the four
    # generator constructors (each ships its own `trajectory_generator_*.cpp`;
    # POLYLINE's target also ships `path_polyline_generator.cpp`, which its
    # `plan` calls);
    # `trajectory_generation_common` carries the inherited
    # `TrajectoryGenerator::generate`; `joint_limits_common` carries
    # `LimitsContainer`/`JointLimitsContainer`;
    # `cartesian_limits_parameters` carries `cartesian_limits::Params`,
    # generated at build time by generate_parameter_library and therefore
    # not findable as a plain header. Verified against the export set in
    # `pilz_industrial_motion_plannerTargetsExport.cmake`.
    pilz_industrial_motion_planner::planning_context_loader_ptp
    pilz_industrial_motion_planner::planning_context_loader_lin
    pilz_industrial_motion_planner::planning_context_loader_circ
    pilz_industrial_motion_planner::planning_context_loader_polyline
    pilz_industrial_motion_planner::trajectory_generation_common
    pilz_industrial_motion_planner::joint_limits_common
    pilz_industrial_motion_planner::cartesian_limits_parameters
    chomp_motion_planner::chomp_motion_planner
    # The stomp_plan op. `moveit_constraint_samplers` carries the
    # `ConstraintSamplerManager` StompPlanningContext::solve uses to turn the
    # request's goal constraints into a goal state; the other four are what
    # moveit_planners_stomp's own CMakeLists links (as moveit_core::moveit_core,
    # resolved here to the specific libraries the planning context actually
    # pulls in) plus the two message packages and the core solver.
    moveit_core::moveit_constraint_samplers
    moveit_core::moveit_planning_interface
    stomp::stomp
    stomp_moveit_parameters
    rsl::rsl
    tf2_eigen::tf2_eigen
    ${visualization_msgs_TARGETS}
    ${std_msgs_TARGETS}
)
# octomap ships old-style CMake variables (OCTOMAP_INCLUDE_DIRS/LIBRARIES),
# not an imported target, per its own octomap-config.cmake. orocos_kdl and
# ompl are the same shape.
target_include_directories(moveit_oracle PRIVATE
  ${random_numbers_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  # Upstream's STOMP headers, read straight out of the moveit2 tree the image
  # ships -- this package is compiled from source rather than linked, see
  # STOMP_MOVEIT_SRC above.
  ${STOMP_MOVEIT_SRC}/include
  # Where generate_parameter_library wrote stomp_moveit_parameters.hpp, plus
  # the moveit_planners_stomp symlink that gives it the name upstream's
  # sources include it by.
  ${CMAKE_CURRENT_BINARY_DIR}/include
)

install(TARGETS moveit_oracle RUNTIME DESTINATION bin)
