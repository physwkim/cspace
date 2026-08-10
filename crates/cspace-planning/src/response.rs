// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No single upstream file: [`PlanningResponse`] replaces the field
// `moveit_msgs::msg::MotionPlanResponse::trajectory` — the only field any of
// this crate's three response adapters reads or writes (`error_code` is this
// crate's `Result<(), ResponseAdapterError>` return instead, see
// `crate::error`'s module doc).

//! The canonical planning-response type.

use cspace_core::state::RobotState;
use cspace_core::trajectory::RobotTrajectory;

/// A successful plan, in the shape this crate's response adapters operate
/// on.
///
/// # Deviation from `cspace_planners::sbp::registry::PlanningResponse`
///
/// sbp's own `PlanningResponse` is `trajectory: Vec<RobotState<'m>>` — bare
/// waypoints, no per-waypoint timing. [`crate::response_adapters::AddRuckigTrajectorySmoothing`]/
/// [`crate::response_adapters::AddTimeOptimalParameterization`] both exist
/// to *compute* that timing (`RuckigSmoothing`/`TimeOptimalTrajectoryGeneration`,
/// both already ported in `cspace_core::trajectory`), so a response type without
/// anywhere to put a duration cannot carry what these two adapters produce.
/// [`cspace_core::trajectory::RobotTrajectory`] already stores one
/// `duration_from_previous` per waypoint (`way_point_duration_from_previous`)
/// for exactly this reason, so [`PlanningResponse::trajectory`] uses it
/// directly instead of re-deriving a parallel `Vec<f64>` of durations here.
///
/// # D8 delta audit (round 21): every member of upstream `MotionPlanResponse`
///
/// `moveit_core/planning_interface/include/moveit/planning_interface/planning_response.hpp:48-73`
/// (`planning_interface::MotionPlanResponse`) has 6 members, one line each:
///
/// - `trajectory` (`RobotTrajectoryPtr`) — ported as
///   [`PlanningResponse::trajectory`] (see "No `Option`" below for why this
///   port drops the null case).
/// - `planning_time` (`double`) — unported, in scope: every upstream fill
///   site sits inside a `PlanningContext`-equivalent's own `solve()`, never
///   `PlanningPipeline::generatePlan` itself — checked both ways, pinned
///   `e017c91e`: `rg -n '\.planning_time\s*=' moveit_core moveit_ros
///   moveit_planners moveit_py` finds
///   `planning_interface::MotionPlanResponse::planning_time` set in
///   `ompl_interface/src/model_based_planning_context.cpp:799`,
///   `chomp/chomp_interface/src/chomp_planning_context.cpp:62`,
///   `stomp/src/stomp_moveit_planning_context.cpp:277`, and
///   `trajectory_generator.cpp:267,277` (`pilz_industrial_motion_planner`'s
///   own `PlanningContext::solve()`). The same `rg` also matches
///   `move_group_sequence_service.cpp:128`/`move_group_sequence_action.cpp:264`,
///   but those set `.planning_time` on `moveit_msgs::msg::MotionSequenceResponse`
///   (a ROS-service response's own field, same name, unrelated type) —
///   `pilz_industrial_motion_planner`'s sequence-capability wrapper, not
///   this member. This is the same structural class as
///   [`crate::request::PlanningRequest`]'s own doc comment's
///   `allowed_planning_time` bullet: a value a concrete planner owns, not
///   this crate's pipeline. PORTING-PLAN.md §153.1 made this exclusion
///   expire "the moment any crate implements a concrete planner against
///   these types", and D8 fired that trigger: `cspace_planners::sbp`'s
///   `RrtConnectManager` now implements [`crate::planner::PlannerManager`]
///   and its context implements [`crate::planner::PlanningContext`], so
///   there is a fill site — `RrtConnectContext::solve`, the exact analogue
///   of the four upstream sites above. The field is still absent, and that
///   is now a *gap*, not an exclusion: it is unfilled, not unowned. What
///   has not changed is that nothing could check it — PORTING-PLAN.md
///   §138.3 removed wall-clock timing from every oracle response
///   (`oracle.cpp`'s `plan`/`pilzTrajectory`, commit `c0838b4`), so a
///   stopwatch added here would be compared against nothing. Wherever it
///   lands, it lands in a context's `solve`, not in
///   [`crate::pipeline::generate_plan`], which (like
///   `PlanningPipeline::generatePlan`) never touches it.
/// - `error_code` (`moveit::core::MoveItErrorCode`) — distinct: replaced by
///   `Result<PlanningResponse, PipelineError>`'s own `Err`, matching
///   `crate::error`'s existing convention of using `Result` instead of a
///   status-code field.
/// - `start_state` (`moveit_msgs::msg::RobotState`) — ported as
///   [`PlanningResponse::start_state`] (round 22). Round 21 called the gap a
///   side effect "damaging" the scene's current state; that read the
///   symptom right but stopped one step short — see
///   `crate::pipeline::generate_plan`'s "Semantic 6" doc for the correction.
///   `cspace_planners::sbp::planning_scene_validity::PlanningSceneValidityChecker`
///   (`planning_scene_validity.rs:128-137`, read-only from here) documents
///   this as a deliberate **contract**, not a defect: "a caller that needs
///   the pre-planning state preserved clones it once, itself, before
///   handing the scene to this type." [`crate::pipeline::generate_plan`] is
///   exactly that caller, and now fulfills it — one clone per query, not
///   per validity check.
///
///   Upstream's own fill site does not exist to match: exhaustively
///   searched (`rg -n '\.start_state\s*=' moveit_core moveit_ros
///   moveit_planners moveit_py`, pinned commit `e017c91e`) for every write
///   to a `planning_interface::MotionPlanResponse`-typed `start_state` —
///   zero hits. Every planner (`ompl_interface`, `chomp_motion_planner`,
///   `stomp` `moveit_planners/stomp/src/stomp_moveit_planning_context.cpp`,
///   `pilz_industrial_motion_planner`) and `PlanningPipeline::generatePlan`
///   itself (`planning_pipeline.cpp`) leave it at the default member
///   value a plain `moveit_msgs::msg::RobotState start_state;` gets — the
///   only site touching it at all is the Python binding's read-only
///   property getter (`moveit_py/src/moveit/moveit_core/planning_interface/planning_response.cpp:52,92`).
///   So there is no real pre-adapter/post-adapter precedent to reproduce;
///   this port picks the value consistent with the field's own doc comment
///   at `planning_response.hpp:64` ("The full starting state used for
///   planning"): captured once, after the request-adapter chain runs (an
///   adapter can mutate `scene.current_state()`, e.g. a bounds-clamping
///   one) and before the first planner call — the state the planner(s)
///   this query actually ran against.
/// - `planner_id` (`std::string`) — ported as [`PlanningResponse::planner_id`].
/// - `operator bool` — distinct: replaced by `generate_plan`'s own
///   `Result`-typed return (`Ok`/`Err` is a strict superset of a bare
///   success/failure bool).
///
/// Total: 3 ported, 2 distinct, 1 unported-in-scope = 6, matching the
/// member count exactly.
///
/// # No `Option`
///
/// Upstream's `res.trajectory` is a nullable `RobotTrajectoryPtr`, checked
/// with `if (!res.trajectory)` at the top of every response adapter here
/// (`validate_path.cpp:86`, `add_ruckig_traj_smoothing.cpp:71`,
/// `add_time_optimal_parameterization.cpp:73`) before ever touching it. A
/// [`PlanningResponse`] in this crate is only ever constructed once a
/// `cspace_planners::sbp`-style `PlanningContext::solve` has already
/// succeeded (see `cspace_planners::sbp::registry::PlanningContext::solve`'s
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
    /// The full state planning actually started from. Filled once by
    /// [`crate::pipeline::generate_plan`] (see that module's doc, "Semantic
    /// 6: `start_state` is captured once, before the planner ever runs") —
    /// never by an individual [`crate::planner::PlannerManager`] impl, since only
    /// [`generate_plan`](crate::pipeline::generate_plan) sits at the point
    /// where the request-adapter chain has already run but no planner has
    /// yet touched `scene`'s current state.
    pub start_state: RobotState<'m>,
}
