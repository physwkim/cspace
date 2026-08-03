// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/include/moveit/robot_model/joint_model.hpp
//   moveit_core/robot_model/include/moveit/robot_model/revolute_joint_model.hpp
//   moveit_core/robot_model/include/moveit/robot_model/prismatic_joint_model.hpp
//   moveit_core/robot_model/include/moveit/robot_model/planar_joint_model.hpp
//   moveit_core/robot_model/include/moveit/robot_model/floating_joint_model.hpp
//   moveit_core/robot_model/include/moveit/robot_model/fixed_joint_model.hpp

//! The robot model layer of the moveit-rs port: joints, links, the full
//! kinematic tree, and SRDF planning groups.
//!
//! # Scope
//!
//! [`joint`] ports [`joint::JointModel`] and its five concrete kinds
//! (Revolute, Prismatic, Planar, Floating, Fixed), [`joint::VariableBounds`]
//! and mimic relations, in isolation from any tree structure. [`LinkModel`],
//! [`RobotModel`] and [`JointModelGroup`] build on top of it: `RobotModel`
//! assembles a URDF and its matching SRDF into the full kinematic tree,
//! resolving mimic relationships and SRDF `<group>` elements against it. See
//! `PORTING-PLAN.md` for what later phases (collision geometry, kinematics
//! solvers, `RobotState`) still own.

mod aabb;
mod diagnostic;
pub mod joint;
mod joint_model_group;
mod link_model;
mod robot_model;

pub use diagnostic::Diagnostic;
pub use joint_model_group::JointModelGroup;
pub use link_model::{LinkModel, LinkShape};
pub use robot_model::RobotModel;
