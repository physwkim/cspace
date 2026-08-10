// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/include/moveit/robot_model/joint_model.hpp (struct VariableBounds)
//   moveit_msgs/msg/JointLimits.msg

/// Position, velocity, acceleration and jerk bounds for one joint variable.
///
/// Upstream `moveit::core::VariableBounds`.
///
/// The `*_bounded` flag and the corresponding `min`/`max` pair are
/// independent: a variable can report `position_bounded == true` while
/// `min_position`/`max_position` are `-inf`/`inf` (a floating joint's
/// translation does exactly this — see [`crate::model::joint::FloatingJoint`]).
/// `bounded == false` does not imply the range is infinite, and an infinite
/// range does not imply `bounded == false`. Both fields are kept explicit
/// rather than deriving one from the other, matching upstream's own
/// representation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VariableBounds {
    /// `min_position_`
    pub min_position: f64,
    /// `max_position_`
    pub max_position: f64,
    /// `position_bounded_`
    pub position_bounded: bool,

    /// `min_velocity_`
    pub min_velocity: f64,
    /// `max_velocity_`
    pub max_velocity: f64,
    /// `velocity_bounded_`
    pub velocity_bounded: bool,

    /// `min_acceleration_`
    pub min_acceleration: f64,
    /// `max_acceleration_`
    pub max_acceleration: f64,
    /// `acceleration_bounded_`
    pub acceleration_bounded: bool,

    /// `min_jerk_`
    pub min_jerk: f64,
    /// `max_jerk_`
    pub max_jerk: f64,
    /// `jerk_bounded_`
    pub jerk_bounded: bool,
}

impl Default for VariableBounds {
    /// Matches upstream's default constructor: every bound `0.0`, every
    /// `*_bounded` flag `false`.
    fn default() -> Self {
        Self {
            min_position: 0.0,
            max_position: 0.0,
            position_bounded: false,
            min_velocity: 0.0,
            max_velocity: 0.0,
            velocity_bounded: false,
            min_acceleration: 0.0,
            max_acceleration: 0.0,
            acceleration_bounded: false,
            min_jerk: 0.0,
            max_jerk: 0.0,
            jerk_bounded: false,
        }
    }
}

/// The subset of `moveit_msgs::msg::JointLimits` that
/// [`crate::model::joint::JointModel::set_variable_bounds_from_limits`] and
/// [`crate::model::joint::JointModel::variable_bounds_msg`] round-trip.
///
/// # Deviation from upstream
///
/// This is not the ROS message type — out of scope per `PORTING-PLAN.md`
/// D1/D2, which keep core crates ROS-free. It is a plain Rust struct with the
/// same field names and semantics as `JointLimits.msg`, used only to carry
/// bound overrides through [`VariableBounds`] without inventing a different
/// shape.
#[derive(Debug, Clone, PartialEq)]
pub struct JointLimits {
    /// `joint_name`. Matched against a [`crate::model::joint::JointModel`]'s full
    /// variable names, not the joint's own name — mirrors upstream
    /// `setVariableBounds(const std::vector<JointLimits>&)`, which compares
    /// `joint_limit.joint_name == variable_names_[j]`.
    pub joint_name: String,
    /// `has_position_limits`
    pub has_position_limits: bool,
    /// `min_position`
    pub min_position: f64,
    /// `max_position`
    pub max_position: f64,
    /// `has_velocity_limits`
    pub has_velocity_limits: bool,
    /// `max_velocity`. Symmetric: upstream stores one magnitude and treats
    /// the range as `[-max_velocity, max_velocity]`.
    pub max_velocity: f64,
    /// `has_acceleration_limits`
    pub has_acceleration_limits: bool,
    /// `max_acceleration`, symmetric as with `max_velocity`.
    pub max_acceleration: f64,
    /// `has_jerk_limits`
    pub has_jerk_limits: bool,
    /// `max_jerk`, symmetric as with `max_velocity`.
    pub max_jerk: f64,
}
