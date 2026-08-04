// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No single upstream file: each variant below replaces one
// `moveit::core::MoveItErrorCode` value a ported adapter's `adapt` used to
// return (`moveit_msgs::msg::MoveItErrorCodes::START_STATE_INVALID`,
// `START_STATE_IN_COLLISION`, `INVALID_MOTION_PLAN`, `FAILURE`). D1 excludes
// the `moveit_msgs` type itself; these two enums carry the same
// distinctions as plain Rust errors instead of an integer code plus a
// `source`/`message` pair a caller has to interpret separately.

//! Error types for [`crate::PlanningRequestAdapter`]/
//! [`crate::PlanningResponseAdapter`].

use thiserror::Error;

/// Why a [`crate::PlanningRequestAdapter`] rejected a
/// [`crate::PlanningRequest`]. Replaces the request-adapter-relevant subset
/// of `moveit_msgs::msg::MoveItErrorCodes` (`START_STATE_INVALID`,
/// `START_STATE_IN_COLLISION`); every request adapter in this crate that can
/// fail returns one of these two.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum RequestAdapterError {
    /// `check_start_state_bounds::CheckStartStateBounds`: the start state is
    /// outside the model's joint bounds, or needed a continuous-joint wrap /
    /// quaternion re-normalization that `fix_start_state` was not asked to
    /// apply.
    #[error("{adapter}: start state out of bounds")]
    StartStateInvalid {
        /// The rejecting adapter's [`crate::PlanningRequestAdapter::description`].
        adapter: &'static str,
    },
    /// `check_start_state_collision::CheckStartStateCollision`: the start
    /// state collides.
    #[error("{adapter}: start state in collision: {detail}")]
    StartStateInCollision {
        /// The rejecting adapter's [`crate::PlanningRequestAdapter::description`].
        adapter: &'static str,
        /// `contact_information`: `"<n> contact(s) detected : <pair>, ..."`.
        detail: String,
    },
}

/// Why a [`crate::PlanningResponseAdapter`] rejected a
/// [`crate::PlanningResponse`]. Replaces the response-adapter-relevant
/// subset of `moveit_msgs::msg::MoveItErrorCodes` (`INVALID_MOTION_PLAN`,
/// `FAILURE`) — `SUCCESS` is `Ok(())`.
#[derive(Debug, Error)]
pub enum ResponseAdapterError {
    /// `validate_path::ValidateSolution`: [`moveit_scene::PlanningScene::is_path_valid`]
    /// found at least one invalid waypoint.
    #[error("{adapter}: computed path is not valid; invalid waypoints: {invalid_waypoints:?}")]
    InvalidMotionPlan {
        /// The rejecting adapter's [`crate::PlanningResponseAdapter::description`].
        adapter: &'static str,
        /// [`moveit_scene::PathValidity::invalid_waypoints`], unchanged.
        invalid_waypoints: Vec<usize>,
    },
    /// `add_ruckig_traj_smoothing::AddRuckigTrajectorySmoothing` /
    /// `add_time_optimal_parameterization::AddTimeOptimalParameterization`:
    /// the underlying `moveit_trajectory` call returned [`moveit_error::Error`].
    #[error("{adapter}: failed to compute a trajectory: {source}")]
    Failed {
        /// The rejecting adapter's [`crate::PlanningResponseAdapter::description`].
        adapter: &'static str,
        /// The underlying `moveit_trajectory` failure.
        #[source]
        source: moveit_error::Error,
    },
}
