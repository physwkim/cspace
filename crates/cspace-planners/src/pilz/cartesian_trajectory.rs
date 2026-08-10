// Copyright (c) 2018, Pilz GmbH & Co. KG
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/cartesian_trajectory.hpp
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/cartesian_trajectory_point.hpp

//! A Cartesian-space trajectory: a named group and link plus a time-ordered
//! sequence of poses/velocities/accelerations. Consumed by
//! [`crate::pilz::trajectory_functions::generate_joint_trajectory_from_cartesian`].
//!
//! # Deviations from upstream
//!
//! - `CartesianTrajectoryPoint::pose` is `geometry_msgs::msg::Pose` upstream;
//!   here it is [`cspace_core::geometry::Isometry3`], this port's pose type
//!   everywhere else. `velocity`/`acceleration` are `geometry_msgs::msg::Twist`
//!   upstream; [`Twist`] below replaces it with the same two
//!   [`cspace_core::geometry::Vector3`] fields, since no ROS message types are
//!   ported (`PORTING-PLAN.md` D1/D2).
//! - `time_from_start` is `rclcpp::Duration` upstream; here it is a plain
//!   `f64` in seconds, matching every other duration in this crate (see
//!   e.g. `crate::pilz::limits::JointLimit`).

use cspace_core::geometry::{Isometry3, Vector3};

/// A linear + angular velocity (or acceleration) pair.
///
/// Replaces upstream `geometry_msgs::msg::Twist` — see this module's
/// `# Deviations`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Twist {
    /// Linear component, in metres/second (or metres/second^2).
    pub linear: Vector3,
    /// Angular component, in radians/second (or radians/second^2).
    pub angular: Vector3,
}

/// One sample of a [`CartesianTrajectory`].
///
/// Upstream `pilz_industrial_motion_planner::CartesianTrajectoryPoint`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartesianTrajectoryPoint {
    /// The link's pose at this sample.
    pub pose: Isometry3,
    /// The link's velocity at this sample.
    pub velocity: Twist,
    /// The link's acceleration at this sample.
    pub acceleration: Twist,
    /// Time since the trajectory's start, in seconds.
    pub time_from_start: f64,
}

/// A Cartesian-space trajectory for one link of one planning group.
///
/// Upstream `pilz_industrial_motion_planner::CartesianTrajectory`.
#[derive(Debug, Clone, Default)]
pub struct CartesianTrajectory {
    /// The planning group this trajectory was generated for.
    pub group_name: String,
    /// The link whose motion `points` describes.
    pub link_name: String,
    /// Time-ordered samples.
    pub points: Vec<CartesianTrajectoryPoint>,
}
