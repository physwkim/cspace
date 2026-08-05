// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2011-2013, Willow Garage, Inc.
// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_state/include/moveit/robot_state/robot_state.hpp
//   moveit_core/robot_state/src/robot_state.cpp
//   moveit_core/dynamics_solver/include/moveit/dynamics_solver/dynamics_solver.hpp
//   moveit_core/dynamics_solver/src/dynamics_solver.cpp
//   moveit_core/robot_state/include/moveit/robot_state/conversions.hpp
//   moveit_core/robot_state/src/conversions.cpp

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
//! It also ports the CSV half of `robot_state/conversions` —
//! [`robot_state_to_csv`], [`robot_state_to_csv_by_groups`] and
//! [`csv_to_robot_state`], upstream's `robotStateToStream`/
//! `streamToRobotState`. The `moveit_msgs` half of that same header is not
//! here and will not be: D1/D6 keep message conversion in `ros/moveit-ros`.
//! The `conversions` module's doc comment names which Rust symbol carries
//! each of those functions.
//!
//! Elsewhere, not here: `setFromIK` and `setFromIKSubgroups` need a
//! kinematics solver, and `moveit-state -> moveit-kinematics` is a cycle, so
//! they live in `moveit_kinematics`'s `set_from_ik` module as free functions
//! over `&mut RobotState`; attached bodies live on `moveit_scene`, for the
//! reason that crate's `attached_body` module doc gives.
//!
//! Deferred, out of scope for this task: `setFromDiffIK`, `interpolate`,
//! distance metrics, `computeAABB`, and anything touching `moveit_msgs`. See
//! the `state` module's doc comments for the per-method deviations, and this
//! crate's test report for what remains `UNFIXED`.

mod conversions;
mod dynamics;
mod state;

pub use conversions::{csv_to_robot_state, robot_state_to_csv, robot_state_to_csv_by_groups};
pub use dynamics::{DynamicsSolver, MaxPayload};
pub use state::{JointIndex, Posed, RobotState};
