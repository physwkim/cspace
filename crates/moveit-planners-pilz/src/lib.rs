// Copyright (c) 2018, Pilz GmbH & Co. KG
// Copyright 2020, PAL Robotics S.L.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause AND Apache-2.0
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/
//     cartesian_trajectory.hpp
//     cartesian_trajectory_point.hpp
//     joint_limits_container.hpp
//     joint_limits_extension.hpp
//     limits_container.hpp
//     path_circle_generator.hpp
//     trajectory_functions.hpp
//     trajectory_generator.hpp
//     trajectory_generator_circ.hpp
//     trajectory_generator_lin.hpp
//     trajectory_generator_ptp.hpp
//     velocity_profile_atrap.hpp
//   moveit_planners/pilz_industrial_motion_planner/include/joint_limits_copy/
//     joint_limits.hpp  (Apache-2.0, PAL Robotics; vendored upstream)
//   moveit_planners/pilz_industrial_motion_planner/src/
//     cartesian_limits_parameters.yaml
//     joint_limits_container.cpp
//     limits_container.cpp
//     path_circle_generator.cpp
//     trajectory_functions.cpp
//     trajectory_generator.cpp
//     trajectory_generator_circ.cpp
//     trajectory_generator_lin.cpp
//     trajectory_generator_ptp.cpp
//     velocity_profile_atrap.cpp
//
// This crate-level citation is the union of what every module below cites in
// its own header; the two vendored orocos_kdl stems (path_line,
// velocity_profile_trap) are independently derived, not ported, and cite
// their upstream call sites with `Used by` in their own file headers instead
// — see those modules' own doc comments for why.

//! The Pilz industrial motion planner: analytical, deterministic LIN/PTP/CIRC
//! trajectory generation, ported from `pilz_industrial_motion_planner`.
//!
//! Unlike the sampling-based planners in `moveit-planners-sbp`, Pilz's
//! trajectories are closed-form, so they can be compared to the upstream
//! oracle bit-for-bit within a tight numeric tolerance rather than only
//! statistically. That determinism is also why this crate exists separately
//! from `moveit-planners-sbp`: the two families have nothing in common at the
//! API level once you get past "both produce a `RobotTrajectory`".
//!
//! # Scope: analytical core only
//!
//! This crate ports only the analytical core (three source directories'
//! worth of pure computation, no ROS node in sight):
//!
//! - [`velocity_profile`] — `velocity_profile_atrap.{hpp,cpp}`: the
//!   trapezoidal/triangular velocity profile shared by every Pilz trajectory
//!   type.
//! - [`path_circle`] — `path_circle_generator.{hpp,cpp}`: three-point and
//!   center-plus-two-point circle solving for `CIRC` motions, plus
//!   [`path_circle::PathCircle`], the interpolated arc that consumes that
//!   geometry (independently derived from LGPL-2.1-or-later `orocos_kdl`, not
//!   line-ported — see that type's own module doc).
//! - [`limits`] — `joint_limits_container.{hpp,cpp}`,
//!   `joint_limits_extension.hpp`, `limits_container.{hpp,cpp}`: per-joint
//!   and Cartesian limit storage and fusion.
//! - [`cartesian_trajectory`] — `cartesian_trajectory.hpp`,
//!   `cartesian_trajectory_point.hpp`: a Cartesian-space trajectory type.
//! - [`trajectory_functions`] — `trajectory_functions.{hpp,cpp}`: IK/FK
//!   round-trips (via `moveit-kinematics`), joint-limit-aware sampling, and
//!   the blending-sphere search shared by every generator.
//! - [`trajectory_generator`] — `trajectory_generator.{hpp,cpp}`'s
//!   `validateRequest` family only; see that module's doc for exactly what
//!   is ported versus deferred.
//!
//! - [`trajectory_generator_ptp`] — `trajectory_generator_ptp.{hpp,cpp}`: the
//!   concrete point-to-point generator.
//! - [`velocity_profile_trap`] — `velocityprofile_trap.{hpp,cpp}` (vendored
//!   orocos_kdl, not Pilz's own tree): the symmetric trapezoidal profile
//!   `LIN`/`CIRC` use to time-parametrize Cartesian arc length.
//! - [`path_line`] — `path_line.{hpp,cpp}` and
//!   `rotational_interpolation_sa.{hpp,cpp}` (vendored orocos_kdl): the
//!   straight-line Cartesian path `LIN` samples.
//! - [`trajectory_generator_lin`] — `trajectory_generator_lin.{hpp,cpp}`: the
//!   concrete straight-line generator.
//! - [`trajectory_generator_circ`] — `trajectory_generator_circ.{hpp,cpp}`:
//!   the concrete circular-arc generator, composed with [`path_circle`]'s
//!   [`path_circle::PathCircle`] — independently derived, not a line-by-line
//!   port; see that type's own module doc for why.
//!
//! Not yet in scope, planned for later rounds:
//! `trajectory_blender_transition_window`.
//!
//! # Deliberately not ported: the ROS layer (D1/D2)
//!
//! The following upstream files are the ROS-facing shell around the
//! analytical core above and are excluded by PORTING-PLAN.md's D1 (no ROS
//! dependency) and D2 (no MoveGroup/action-server layer):
//!
//! - `move_group_sequence_action.{hpp,cpp}`,
//!   `move_group_sequence_service.{hpp,cpp}` — `actionlib`/`rclcpp` action
//!   and service servers wrapping the planner for `move_group`; nothing here
//!   computes a trajectory, they only marshal ROS requests into calls on the
//!   types below.
//! - `planning_context_loader*.{hpp,cpp}` — a `pluginlib`-loaded factory that
//!   builds a `planning_interface::PlanningContext` per motion command type;
//!   its entire job is ROS plugin registration, not planning math.
//! - `pilz_industrial_motion_planner.cpp` — the `planning_interface::PlannerManager`
//!   plugin itself, i.e. the `move_group` entry point.
//! - `command_list_manager.{hpp,cpp}` — sequences multiple motion commands
//!   (`MotionSequenceRequest`) and manages blending between them at the
//!   `moveit_msgs` request level; this is orchestration over the trajectory
//!   generators, not trajectory generation.
//! - `plan_components_builder.{hpp,cpp}` — assembles per-command
//!   `RobotTrajectory` segments (produced by the generators this crate does
//!   port) into one blended `RobotTrajectory` for a command list; depends on
//!   `command_list_manager`'s request types.
//!
//! None of these five compute a LIN/PTP/CIRC trajectory; they route
//! `moveit_msgs` requests to the analytical types this crate ports. A future
//! `moveit-ros` (or equivalent) crate is the right home for a Rust
//! equivalent, if one is ever built.
//!
//! # ROS dependencies found, and how each was resolved
//!
//! Every one of the nine upstream analytical-core source files was checked
//! individually for `rclcpp`/`moveit_msgs`/`tf2_ros` usage. Across this
//! round's three files (`velocity_profile_atrap`, `path_circle_generator`,
//! `joint_limits_container` + `limits_container`):
//!
//! - `velocity_profile_atrap.{hpp,cpp}` and `path_circle_generator.{hpp,cpp}`
//!   have no ROS dependency at all (KDL types only).
//! - `joint_limits_container.cpp` uses `rclcpp::Logger`/`RCLCPP_ERROR_STREAM`
//!   exclusively for logging inside `addLimit`'s two rejection branches.
//! - `limits_container.cpp` uses `rclcpp::Logger`/`RCLCPP_DEBUG` exclusively
//!   for logging inside `printCartesianLimits()`.
//!
//! No computation depends on ROS in any of the three files this round ports;
//! every logging call site above is replaced with a native `Result`/`bool`
//! return instead of a log-and-continue.

pub mod cartesian_trajectory;
pub mod limits;
pub mod path_circle;
pub mod path_line;
pub mod trajectory_functions;
pub mod trajectory_generator;
pub mod trajectory_generator_circ;
pub mod trajectory_generator_lin;
pub mod trajectory_generator_ptp;
pub mod velocity_profile;
pub mod velocity_profile_trap;
