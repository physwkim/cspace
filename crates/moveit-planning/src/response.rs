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
/// # D8 delta audit (round 21): every member of upstream `MotionPlanResponse`
///
/// `moveit_core/planning_interface/include/moveit/planning_interface/planning_response.hpp:48-70`
/// (`planning_interface::MotionPlanResponse`) has 6 members, one line each:
///
/// - `trajectory` (`RobotTrajectoryPtr`) — ported as
///   [`PlanningResponse::trajectory`] (see "No `Option`" below for why this
///   port drops the null case).
/// - `planning_time` (`double`) — unported, in scope, parity-incomparable:
///   adding the field is legitimate, but PORTING-PLAN.md §138.3 removed
///   wall-clock timing from every oracle response (`oracle.cpp`'s `plan`/
///   `pilzTrajectory`, commit `c0838b4`) precisely because a C++ stopwatch
///   and a Rust stopwatch can never be differentially compared — so this
///   port cannot gain a comparable value for this field even if it added
///   one, and adding an uncomparable one has no test that could use it.
/// - `error_code` (`moveit::core::MoveItErrorCode`) — distinct: replaced by
///   `Result<PlanningResponse, PipelineError>`'s own `Err`, matching
///   `crate::error`'s existing convention of using `Result` instead of a
///   status-code field.
/// - `start_state` (`moveit_msgs::msg::RobotState`) — unported, in scope, a
///   real gap, not a design choice: `moveit-planners-sbp::registry`'s own
///   test comment (`end_to_end_solve_on_panda_arm_reaches_the_requested_goal`,
///   `registry.rs:428-431`) already documents that
///   `PlanningSceneValidityChecker::is_valid` "leaves the scene's current
///   state at whatever it last checked" as a side effect — so unlike
///   `MotionPlanRequest::start_state` (where the scene's current state
///   reliably *is* the start, see `request.rs`'s own audit above),
///   `scene.current_state()` read back **after** `generate_plan` returns is
///   not reliably the state the plan actually started from. Upstream
///   carries this field precisely to survive that same class of
///   post-hoc-unreliability; this port has no equivalent, so a caller
///   currently has no way to recover the true start state after the fact
///   except capturing it themselves before calling
///   [`crate::pipeline::generate_plan`] — exactly the workaround that same
///   `registry.rs` test already has to use.
/// - `planner_id` (`std::string`) — ported as [`PlanningResponse::planner_id`].
/// - `operator bool` — distinct: replaced by `generate_plan`'s own
///   `Result`-typed return (`Ok`/`Err` is a strict superset of a bare
///   success/failure bool).
///
/// Total: 2 ported, 2 distinct, 2 unported-in-scope = 6, matching the
/// member count exactly.
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
