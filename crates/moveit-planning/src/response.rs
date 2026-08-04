// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No single upstream file: [`PlanningResponse`] replaces the field
// `moveit_msgs::msg::MotionPlanResponse::trajectory` — the only field any of
// this crate's three response adapters reads or writes (`error_code` is this
// crate's `Result<(), ResponseAdapterError>` return instead, see
// `crate::error`'s module doc).

//! The canonical planning-response type.

use moveit_trajectory::RobotTrajectory;

/// A successful plan, in the shape this crate's response adapters operate
/// on.
///
/// # Deviation from `moveit-planners-sbp::registry::PlanningResponse`
///
/// sbp's own `PlanningResponse` is `trajectory: Vec<RobotState<'m>>` — bare
/// waypoints, no per-waypoint timing. [`crate::response_adapters::AddRuckigTrajectorySmoothing`]/
/// [`crate::response_adapters::AddTimeOptimalParameterization`] both exist
/// to *compute* that timing (`RuckigSmoothing`/`TimeOptimalTrajectoryGeneration`,
/// both already ported in `moveit_trajectory`), so a response type without
/// anywhere to put a duration cannot carry what these two adapters produce.
/// [`moveit_trajectory::RobotTrajectory`] already stores one
/// `duration_from_previous` per waypoint (`way_point_duration_from_previous`)
/// for exactly this reason, so [`PlanningResponse::trajectory`] uses it
/// directly instead of re-deriving a parallel `Vec<f64>` of durations here.
///
/// # No `Option`
///
/// Upstream's `res.trajectory` is a nullable `RobotTrajectoryPtr`, checked
/// with `if (!res.trajectory)` at the top of every response adapter here
/// (`validate_path.cpp:86`, `add_ruckig_traj_smoothing.cpp:71`,
/// `add_time_optimal_parameterization.cpp:73`) before ever touching it. A
/// [`PlanningResponse`] in this crate is only ever constructed once a
/// `moveit-planners-sbp`-style `PlanningContext::solve` has already
/// succeeded (see `moveit-planners-sbp::registry::PlanningContext::solve`'s
/// `Result<PlanningResponse, PlanError>` return, which this type's own
/// eventual relocation there will keep), so `trajectory` is never absent by
/// construction and that null check has nothing to guard here — see each
/// response adapter's module doc for the corresponding "Not ported" note.
#[derive(Debug, Clone)]
pub struct PlanningResponse<'m> {
    /// The solved trajectory, waypoints and per-waypoint timing alike.
    pub trajectory: RobotTrajectory<'m>,
    /// Which planner produced this response. A planner that does not fill
    /// this in gets it backfilled from the request's
    /// [`crate::PlanningRequest::planner_id`] by
    /// [`crate::pipeline::generate_plan`] — see that module's doc, "Semantic
    /// 4: `planner_id` fallback". `""` (the same value
    /// [`Default::default`] gives a plain `String`) means "not yet set",
    /// matching an unset `moveit_msgs::msg::MotionPlanResponse::planner_id`.
    pub planner_id: String,
}
