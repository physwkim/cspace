// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_state/include/moveit/robot_state/robot_state.hpp
//   moveit_core/robot_state/src/robot_state.cpp

//! `RobotState` and forward kinematics for moveit-rs.
//!
//! # Scope
//!
//! This crate ports variable storage, the position setters (all
//! `setVariablePositions`/`setJointPositions` overloads), mimic-joint
//! propagation on every write path, bounds (`enforceBounds`/
//! `satisfiesBounds`/`harmonizePositions`), default/random positions, and
//! forward kinematics (`updateLinkTransforms`, `getGlobalLinkTransform`,
//! `getJointTransform`, `getFrameTransform`, `knowsFrameTransform`).
//!
//! Deferred, out of scope for this task: the Jacobian, `setFromIK`/
//! `setFromDiffIK`, attached bodies, `interpolate`, distance metrics,
//! `computeAABB`, anything touching `moveit_msgs`, and velocity/
//! acceleration/effort tracking. See the `state` module's doc comments for the
//! per-method deviations, and this crate's test report for what remains
//! `UNFIXED`.

mod state;

pub use state::{JointIndex, Posed, RobotState};
