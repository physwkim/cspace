// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/trajectory_processing/include/moveit/trajectory_processing/trajectory_tools.hpp
//   moveit_core/trajectory_processing/src/trajectory_tools.cpp

//! Upstream `trajectory_processing::trajectory_tools`'s five free functions,
//! two of which are ported here and three of which are not.
//!
//! [`apply_totg_time_parameterization`] and [`apply_ruckig_smoothing`] are
//! the convenience entry points every upstream caller actually uses instead
//! of constructing a `TimeOptimalTrajectoryGeneration`/`RuckigSmoothing`
//! instance directly, so they are ported even though they add no behavior
//! beyond a call each into [`crate::time_optimal_trajectory_generation::compute_time_stamps`]/
//! [`crate::ruckig_smoothing::apply_smoothing`] — see each function's doc
//! comment for the exact equivalence, confirmed line-for-line against
//! `trajectory_tools.cpp:63-76`.
//!
//! [`apply_totg_time_parameterization`] wraps the scaling-only
//! `compute_time_stamps` overload specifically (matching upstream, which
//! only ever constructs a scaling-only `TimeOptimalTrajectoryGeneration`
//! call from here), so its test below builds on the same
//! `RobotModel::joint_model_mut`-based acceleration-bounds setup as
//! [`crate::time_optimal_trajectory_generation`]'s now-closed "Known gap"
//! section describes.
//!
//! # Out of scope
//!
//! Three of upstream's five functions take or return a ROS message type
//! (`moveit_msgs`/`trajectory_msgs`), out of scope per `PORTING-PLAN.md`
//! §0's D1 interpretation (see
//! [`crate::time_optimal_trajectory_generation`]'s "Out of scope" section
//! for the same citation):
//!
//! - `isTrajectoryEmpty(const moveit_msgs::msg::RobotTrajectory&)` (cpp:54-57).
//! - `trajectoryWaypointCount(const moveit_msgs::msg::RobotTrajectory&)` (cpp:59-62).
//! - `createTrajectoryMessage(...) -> trajectory_msgs::msg::JointTrajectory`
//!   (cpp:78-109). Note this one does not take a ROS type as input — only
//!   its return type is one; the exclusion is symmetric with the other two
//!   because D1's rule is about the type appearing in the core crate's
//!   signature at all, not about which side of the call it is on.

use crate::robot_trajectory::RobotTrajectory;
use crate::ruckig_smoothing::{self, SmoothingOptions};
use crate::time_optimal_trajectory_generation::{self, TotgOptions};
use moveit_error::Result;

/// `applyTOTGTimeParameterization` (cpp:63-69).
///
/// Equivalent to constructing upstream's `TimeOptimalTrajectoryGeneration
/// totg(path_tolerance, resample_dt, min_angle_change)` and calling
/// `totg.computeTimeStamps(trajectory, velocity_scaling_factor,
/// acceleration_scaling_factor)`: builds a [`TotgOptions`] from all five
/// arguments and calls
/// [`compute_time_stamps`](time_optimal_trajectory_generation::compute_time_stamps).
/// Rust has no default parameters, so upstream's three defaulted trailing
/// arguments (`path_tolerance = 0.1`, `resample_dt = 0.1`,
/// `min_angle_change = 0.001`, matching [`TotgOptions::default`]) are still
/// required positionally here — pass `TotgOptions::default()`'s field
/// values, or call
/// [`compute_time_stamps`](time_optimal_trajectory_generation::compute_time_stamps)
/// directly with a [`TotgOptions`] built by struct-update syntax, if that
/// reads better at a given call site.
///
/// # Errors
///
/// Same as
/// [`compute_time_stamps`](time_optimal_trajectory_generation::compute_time_stamps).
pub fn apply_totg_time_parameterization(
    trajectory: &mut RobotTrajectory<'_>,
    velocity_scaling_factor: f64,
    acceleration_scaling_factor: f64,
    path_tolerance: f64,
    resample_dt: f64,
    min_angle_change: f64,
) -> Result<()> {
    time_optimal_trajectory_generation::compute_time_stamps(
        trajectory,
        &TotgOptions {
            path_tolerance,
            resample_dt,
            min_angle_change,
            max_velocity_scaling_factor: velocity_scaling_factor,
            max_acceleration_scaling_factor: acceleration_scaling_factor,
        },
    )
}

/// `applyRuckigSmoothing` (cpp:70-76).
///
/// Equivalent to constructing upstream's `RuckigSmoothing time_param` and
/// calling `time_param.applySmoothing(trajectory, velocity_scaling_factor,
/// acceleration_scaling_factor, mitigate_overshoot, overshoot_threshold)`:
/// builds a [`SmoothingOptions`] from all four trailing arguments and calls
/// [`apply_smoothing`](ruckig_smoothing::apply_smoothing). Rust has no
/// default parameters, so upstream's two defaulted trailing arguments
/// (`mitigate_overshoot = false`, `overshoot_threshold = 0.01`, matching
/// [`SmoothingOptions::default`]) are still required positionally here.
///
/// # Errors
///
/// Same as [`apply_smoothing`](ruckig_smoothing::apply_smoothing).
pub fn apply_ruckig_smoothing(
    trajectory: &mut RobotTrajectory<'_>,
    velocity_scaling_factor: f64,
    acceleration_scaling_factor: f64,
    mitigate_overshoot: bool,
    overshoot_threshold: f64,
) -> Result<()> {
    ruckig_smoothing::apply_smoothing(
        trajectory,
        &SmoothingOptions {
            max_velocity_scaling_factor: velocity_scaling_factor,
            max_acceleration_scaling_factor: acceleration_scaling_factor,
            mitigate_overshoot,
            overshoot_threshold,
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_srdf::SrdfModel;
    use moveit_state::RobotState;
    use std::fs;

    fn panda() -> RobotModel {
        let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.urdf");
        let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.srdf");
        let urdf_xml = fs::read_to_string(urdf_path).unwrap_or_else(|e| panic!("{urdf_path}: {e}"));
        let urdf = urdf_rs::read_file(urdf_path).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("fixture model must build")
    }

    fn two_waypoint_trajectory(model: &RobotModel) -> RobotTrajectory<'_> {
        let mut trajectory = RobotTrajectory::for_group_name(model, "panda_arm")
            .expect("panda_arm group must exist");
        let mut start = RobotState::new(model);
        start.set_to_default_values();
        trajectory
            .add_suffix_way_point(start, 0.0)
            .expect("add start waypoint");
        let mut goal = RobotState::new(model);
        goal.set_to_default_values();
        for name in [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
        ] {
            let current = goal.variable_position(name).expect("variable exists");
            goal.set_variable_position(name, current + 0.5)
                .expect("set_variable_position");
        }
        trajectory
            .add_suffix_way_point(goal, 0.1)
            .expect("add goal waypoint");
        trajectory
    }

    /// `panda.urdf` has no `<limit>` acceleration field (URDF's schema has
    /// none), so every `panda_arm` joint needs `acceleration_bounded` set
    /// programmatically before the scaling-only overload can succeed —
    /// same read ([`moveit_model::joint::JointModel::variable_bounds_msg`])
    /// / mutate / write
    /// ([`moveit_model::joint::JointModel::set_variable_bounds_from_limits`])
    /// pattern `totg_robot_trajectory_scaling_only_parity.rs` uses against
    /// the oracle.
    fn set_uniform_acceleration_bound(model: &mut RobotModel, max_acceleration: f64) {
        for name in [
            "panda_joint1",
            "panda_joint2",
            "panda_joint3",
            "panda_joint4",
            "panda_joint5",
            "panda_joint6",
            "panda_joint7",
        ] {
            let joint = model.joint_model_mut(name).expect("panda_arm joint exists");
            let mut limits = joint.variable_bounds_msg();
            for limit in &mut limits {
                limit.has_acceleration_limits = true;
                limit.max_acceleration = max_acceleration;
            }
            joint.set_variable_bounds_from_limits(&limits);
        }
    }

    /// `apply_totg_time_parameterization` wraps
    /// [`time_optimal_trajectory_generation::compute_time_stamps`] (the
    /// scaling-only overload); this asserts the wrapper forwards its five
    /// arguments into an equivalent [`TotgOptions`] faithfully by requiring
    /// both calls to reach the same successful numeric result, not just
    /// "both fail the same way" (the only thing checkable before
    /// `RobotModel::joint_model_mut` existed — see
    /// [`time_optimal_trajectory_generation`]'s former "Known gap" doc
    /// section, now closed).
    #[test]
    fn apply_totg_time_parameterization_with_upstream_defaults_forwards_to_compute_time_stamps() {
        let mut model = panda();
        set_uniform_acceleration_bound(&mut model, 3.0);
        let mut via_tool = two_waypoint_trajectory(&model);
        let mut via_core = two_waypoint_trajectory(&model);

        apply_totg_time_parameterization(
            &mut via_tool,
            1.0,
            1.0,
            crate::DEFAULT_PATH_TOLERANCE,
            0.1,
            0.001,
        )
        .expect("apply_totg_time_parameterization must succeed");
        time_optimal_trajectory_generation::compute_time_stamps(
            &mut via_core,
            &TotgOptions::default(),
        )
        .expect("compute_time_stamps must succeed");

        assert_eq!(via_tool.way_point_count(), via_core.way_point_count());
        for idx in 0..via_tool.way_point_count() {
            assert_relative_eq!(
                via_tool.way_point_duration_from_previous(idx),
                via_core.way_point_duration_from_previous(idx),
                epsilon = 1e-12
            );
        }
    }

    #[test]
    fn apply_ruckig_smoothing_with_upstream_defaults_matches_apply_smoothing() {
        let model = panda();
        let mut via_tool = two_waypoint_trajectory(&model);
        let mut via_core = two_waypoint_trajectory(&model);

        apply_ruckig_smoothing(&mut via_tool, 1.0, 1.0, false, 0.01)
            .expect("apply_ruckig_smoothing must succeed");
        ruckig_smoothing::apply_smoothing(&mut via_core, &SmoothingOptions::default())
            .expect("apply_smoothing must succeed");

        assert_eq!(via_tool.way_point_count(), via_core.way_point_count());
        for idx in 0..via_tool.way_point_count() {
            assert_relative_eq!(
                via_tool.way_point_duration_from_previous(idx),
                via_core.way_point_duration_from_previous(idx),
                epsilon = 1e-12
            );
        }
    }
}
