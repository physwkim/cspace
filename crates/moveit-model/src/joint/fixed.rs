// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/include/moveit/robot_model/fixed_joint_model.hpp
//   moveit_core/robot_model/src/fixed_joint_model.cpp

use moveit_geometry::Isometry3;

/// A fixed joint: zero degrees of freedom, no variables.
///
/// Upstream `moveit::core::FixedJointModel`. Carries no state — every
/// upstream method on this type is a no-op or a constant, reproduced as
/// free functions on [`crate::joint::JointModel`]'s `Fixed` dispatch arm
/// rather than as methods on an empty struct here.
pub(super) fn compute_transform() -> Isometry3 {
    Isometry3::identity()
}
