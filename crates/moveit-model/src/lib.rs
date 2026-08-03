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

//! The joint layer of the moveit-rs robot model port.
//!
//! # Scope
//!
//! This crate is deliberately narrow: it ports [`joint::JointModel`] and its
//! five concrete kinds (Revolute, Prismatic, Planar, Floating, Fixed),
//! [`joint::VariableBounds`] and mimic relations. `LinkModel`, `RobotModel`
//! and `JointModelGroup` are owned by other crates in this port — see
//! `PORTING-PLAN.md`.

pub mod joint;
