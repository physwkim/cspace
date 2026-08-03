// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_state/include/moveit/robot_state/robot_state.hpp
//   moveit_core/robot_state/src/robot_state.cpp
//   moveit_core/dynamics_solver/include/moveit/dynamics_solver/dynamics_solver.hpp
//   moveit_core/dynamics_solver/src/dynamics_solver.cpp

//! `RobotState`, forward kinematics and inverse dynamics for moveit-rs.
//!
//! # Scope
//!
//! This crate ports variable storage (position, velocity, acceleration,
//! effort), the position setters (all `setVariablePositions`/
//! `setJointPositions` overloads), mimic-joint propagation on every
//! position write path, bounds (`enforceBounds`/`satisfiesBounds`/
//! `harmonizePositions`, including velocity bounds), default/random
//! positions, forward kinematics (`updateLinkTransforms`,
//! `getGlobalLinkTransform`, `getJointTransform`, `getFrameTransform`,
//! `knowsFrameTransform`), and inverse dynamics ([`DynamicsSolver`], see
//! the `dynamics` module's doc comment for its own scope and deviations).
//!
//! Deferred, out of scope for this task: `setFromIK`/`setFromDiffIK`,
//! attached bodies, `interpolate`, distance metrics, `computeAABB`, and
//! anything touching `moveit_msgs`. See the `state` module's doc comments
//! for the per-method deviations, and this crate's test report for what
//! remains `UNFIXED`.

mod dynamics;
mod state;

pub use dynamics::{DynamicsSolver, MaxPayload};
pub use state::{JointIndex, Posed, RobotState};
