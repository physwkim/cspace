// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2007, Ruben Smits
// Copyright (c) 2013, Sachin Chitta, Willow Garage
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/kinematics_base/include/moveit/kinematics_base/kinematics_base.hpp
//   moveit_core/kinematics_base/src/kinematics_base.cpp
//   moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp
//   moveit_kinematics/kdl_kinematics_plugin/include/moveit/kdl_kinematics_plugin/kdl_kinematics_plugin.hpp
//   moveit_kinematics/kdl_kinematics_plugin/src/chainiksolver_vel_mimic_svd.cpp
//   moveit_kinematics/kdl_kinematics_plugin/include/moveit/kdl_kinematics_plugin/chainiksolver_vel_mimic_svd.hpp
//   moveit_kinematics/kdl_kinematics_plugin/include/moveit/kdl_kinematics_plugin/joint_mimic.hpp

//! Numeric inverse kinematics for moveit-rs.
//!
//! Upstream splits this across `kinematics_base` (the plugin interface,
//! loaded at runtime through pluginlib by class name) and
//! `kdl_kinematics_plugin` (the one solver every MoveIt robot actually gets
//! by default: a damped/truncated-least-squares Newton iteration over a KDL
//! chain's Jacobian). Per `PORTING-PLAN.md` decision D4, the runtime
//! plugin-by-string-name lookup is not ported — [`KinematicsSolver`]
//! implementations register themselves at compile time in
//! [`KINEMATICS_SOLVERS`] through [`linkme`], and a caller picks one by
//! constructing it directly ([`NewtonRaphsonSolver::new`],
//! [`LevenbergMarquardtSolver::new`]) or by scanning the registry for a
//! [`SolverRegistration::name`].
//!
//! # What upstream reaches through KDL, and what this port reaches through
//! `moveit-state`
//!
//! `kdl_kinematics_plugin` builds a `KDL::Chain` from the URDF and gets its
//! Jacobian and forward kinematics from KDL's own solvers
//! (`ChainJntToJacSolver`, `ChainFkSolverPos_recursive`). This port has no
//! KDL dependency (D1/D2): both come from [`moveit_state::Posed`] —
//! [`moveit_state::Posed::jacobian`] and
//! [`moveit_state::Posed::global_link_transform`] — which already
//! encapsulate the chain-validity checks
//! (`moveit_model::JointModelGroup::is_chain`) this crate would otherwise
//! have to re-derive.
//!
//! # Do not port the ROS surface
//!
//! No `rclcpp::Node`, no `geometry_msgs::Pose`, no
//! `moveit_msgs::MoveItErrorCodes`, no pluginlib registration macro. Poses
//! are [`moveit_geometry::Isometry3`]; errors that mean "this solver cannot
//! be built for this group" go through [`moveit_error::Error`]; failure to
//! converge is not an error (upstream itself models it as a `bool`, not an
//! exception) — see [`KinematicsSolver::solve`].
//! `initialize(node, robot_model, group_name, base_frame, tip_frames,
//! search_discretization)` becomes a plain constructor taking the model, the
//! group name and a [`SolverParams`].
//!
//! # Position-only IK and joint limits are modes, not solvers
//!
//! Matching upstream (`KinematicsQueryOptions`/`params_.position_only_ik`
//! select behaviour inside the one `CartToJnt`, they do not select a
//! different class), [`SolverParams::position_only`] and joint-limit
//! clipping (`chain::ChainInfo`'s bounds, applied every iteration by
//! `cart_to_jnt`'s `clip_to_joint_limits`) are parameters and always-on
//! behaviour of both solvers below, not separate types.

mod cart_to_jnt;
mod chain;
mod lma;
mod newton_raphson;
mod params;
mod registry;
mod velocity;

pub use lma::LevenbergMarquardtSolver;
pub use newton_raphson::NewtonRaphsonSolver;
pub use params::SolverParams;
pub use registry::{
    KINEMATICS_SOLVERS, KinematicsSolver, SolutionCallback, SolveOptions, SolverRegistration,
};
