// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No single upstream file: [`PlanningRequest`] replaces the fields of
// `moveit_msgs::msg::MotionPlanRequest` this crate's six request adapters
// actually read (`group_name`, `goal_constraints`, `path_constraints`,
// `workspace_parameters`, `max_velocity_scaling_factor`,
// `max_acceleration_scaling_factor`); [`WorkspaceBounds`] replaces
// `moveit_msgs::msg::WorkspaceParameters` minus its `header` (D1: no ROS
// `std_msgs::msg::Header`).

//! The canonical planning-request type. See the crate doc comment's
//! "Deviation from `moveit-planners-sbp::registry`" section for why this
//! shape, not a transcription of `moveit_msgs::msg::MotionPlanRequest`
//! (which this crate cannot depend on, D1) nor `moveit-planners-sbp`'s
//! existing `PlanningRequest` (a concrete-state goal with RRT-Connect's own
//! tuning fields), is the request type the adapters in this crate operate
//! on.

use moveit_constraints::KinematicConstraintSet;
use moveit_geometry::Vector3;

/// Replaces `moveit_msgs::msg::WorkspaceParameters` (minus `header`, D1): the
/// axis-aligned box a sampling-based planner should search within.
///
/// `Default` is the all-zero box, matching an unset ROS message field —
/// [`crate::request_adapters::ValidateWorkspaceBounds`] treats this exact
/// value as "not specified" and replaces it with a centered cube.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorkspaceBounds {
    /// `min_corner`.
    pub min_corner: Vector3,
    /// `max_corner`.
    pub max_corner: Vector3,
}

impl Default for WorkspaceBounds {
    fn default() -> Self {
        Self {
            min_corner: Vector3::zeros(),
            max_corner: Vector3::zeros(),
        }
    }
}

/// A motion planning query, in the shape this crate's request adapters (and,
/// through them, [`crate::run_request_adapters`]) operate on.
///
/// See the crate doc comment for why `goal_constraints` is
/// `Vec<KinematicConstraintSet>` rather than a concrete state, and why
/// planner-specific tuning (RRT-Connect's step size, STOMP's iteration
/// count, ...) is deliberately not a field here.
#[derive(Debug, Clone)]
pub struct PlanningRequest {
    /// The [`moveit_model::JointModelGroup`] to plan for.
    pub group_name: String,
    /// Candidate goal constraint sets — a state satisfying *any one* set is
    /// an acceptable goal. Matches
    /// `MotionPlanRequest::goal_constraints: Vec<Constraints>`'s
    /// any-of-these-sets contract exactly (`planning_scene.cpp`'s own
    /// `isPathValid` reads it the same way — see
    /// [`moveit_scene::PlanningScene::is_path_valid`]'s `goal_constraints`
    /// parameter).
    pub goal_constraints: Vec<KinematicConstraintSet>,
    /// Constraints every waypoint (not just the goal) must satisfy. `None`
    /// means unconstrained.
    pub path_constraints: Option<KinematicConstraintSet>,
    /// The box a sampling-based planner should search within.
    /// [`crate::request_adapters::ValidateWorkspaceBounds`] fills this in
    /// from a default when left at [`WorkspaceBounds::default`].
    pub workspace_bounds: WorkspaceBounds,
    /// A factor in `(0, 1]` scaling every joint's velocity limit. Read by
    /// [`crate::response_adapters::AddRuckigTrajectorySmoothing`]/
    /// [`crate::response_adapters::AddTimeOptimalParameterization`], exactly
    /// as upstream's identically-named field is.
    pub max_velocity_scaling_factor: f64,
    /// A factor in `(0, 1]` scaling every joint's acceleration limit. Same
    /// readers as [`PlanningRequest::max_velocity_scaling_factor`].
    pub max_acceleration_scaling_factor: f64,
}
