// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_planner.hpp
//   (class ChompPlanner)
//   moveit_planners/chomp/chomp_motion_planner/src/chomp_planner.cpp
//   (ChompPlanner::solve)

//! [`solve`], the model-independent numeric core of `chomp::ChompPlanner::solve`.
//!
//! # Round 20 (`PORTING-PLAN.md` §154 review): why this ported cleanly
//!
//! Round 19 deferred this file as "ROS-message-shaped, out of scope" without
//! measuring it. A field-by-field read of `solve`'s two upstream parameter
//! types settles it:
//!
//! - `planning_interface::MotionPlanRequest` (a bare `typedef` of
//!   `moveit_msgs::msg::MotionPlanRequest`, 16 fields) -- `solve` reads
//!   exactly 3: `start_state`, `group_name`, `goal_constraints`
//!   (`rg -n 'req\.' chomp_planner.cpp`, pinned SHA). The other 13
//!   (`workspace_parameters`, `path_constraints`, `trajectory_constraints`,
//!   `reference_trajectories`, `pipeline_id`, `planner_id`,
//!   `num_planning_attempts`, `allowed_planning_time`,
//!   `max_{velocity,acceleration}_scaling_factor`,
//!   `cartesian_speed_limited_link`, `max_cartesian_speed`,
//!   `smoothness_level`) are untouched.
//! - `planning_interface::MotionPlanDetailedResponse` (`planning_response.hpp:75-84`
//!   -- a hand-written struct, **not** the ROS-generated
//!   `moveit_msgs::msg::MotionPlanDetailedResponse`, which has a different,
//!   unrelated field set) has 5 fields, all 5 written: `planner_id`,
//!   `error_code`, `trajectory`, `description`, `processing_time`.
//!
//! Of `solve`'s ~240 body lines (`chomp_planner.cpp:66-306`), 29 lines
//! (`rg -c 'RCLCPP_|chrono' chomp_planner.cpp`) are pure logging
//! (`RCLCPP_*`, dropped workspace-wide -- no logging framework is ported)
//! or wall-clock bookkeeping (`std::chrono`, dropped per
//! `PORTING-PLAN.md` §138.3: a C++ stopwatch and a Rust one are never
//! differentially comparable, so `processing_time` has no test that could
//! use it even if ported). Every remaining line -- state-bounds checking,
//! goal-state construction, the continuous-joint wraparound fix, the four
//! trajectory-initialization methods, the optimizer construction/optimize
//! recovery loop, final trajectory extraction, the collision-free check,
//! and the goal-tolerance check -- already has a direct counterpart
//! somewhere in this workspace (`cspace_core::state`, `cspace_core::model`,
//! `cspace_planning::constraints`, `cspace_core::error`, and this crate's own already-ported
//! [`crate::chomp::trajectory::ChompTrajectory`]/[`crate::chomp::optimizer::ChompOptimizer`]/
//! [`crate::chomp::parameters::ChompParameters`]). Zero lines are blocked by a
//! genuinely missing type. This settles round 20 item 2's "count, don't
//! guess" instruction: the unblocked majority dominates, so this round
//! ports it rather than re-deferring on an unmeasured judgment.
//!
//! # Deviation: no `cspace_planning::scene`, no `cspace-planning` dependency
//!
//! Upstream's `solve` takes a `planning_scene::PlanningSceneConstPtr` and
//! uses exactly three of its members: `getCurrentState()` (the starting
//! point before the request's `start_state` message is overlaid onto it),
//! `getTransforms()` (only to feed that message-overlay conversion), and
//! `getRobotModel()`. This crate's own [`crate::chomp::optimizer::ChompOptimizer::new`]
//! already replaced the same `PlanningSceneConstPtr` parameter with a plain
//! `start_state: &RobotState` (round 19) -- there is no `moveit_msgs::msg::RobotState`
//! in this workspace (D1) for the message-overlay step to exist at all, so
//! a caller here passes the already-resolved `start_state` directly, and
//! `RobotModel` comes from [`cspace_core::state::RobotState::model`] instead of a
//! scene. This keeps this crate's dependency graph exactly as narrow as
//! [`crate::chomp::optimizer`]'s own "no `cspace_planning::scene`" decision (see that
//! module's "`isCurrentTrajectoryMeshToMeshCollisionFree` becomes an
//! injected closure" doc) -- adding `cspace_planning::scene` here just to immediately
//! discard two of its three uses would reopen exactly the dependency
//! question that decision already closed.
//!
//! `cspace_planning::PlanningRequest`/`PlanningResponse` (the workspace's
//! other candidate canonical shape, per round 20's brief) are deliberately
//! **not** used as this function's parameter/return types. Measured, not
//! assumed: `grep -rl "cspace-planning" crates/*/Cargo.toml` matches only
//! `cspace-planning`'s own manifest -- it has zero consumers anywhere in
//! this workspace today. The brief's stated precedent ("`sbp` swapped
//! upstream's ROS response type for `cspace_planning`'s canonical type and
//! ported it") does not hold up under the same check:
//! `cspace_planners::sbp`'s `registry::PlanningRequest`/`PlanningResponse`
//! (`registry.rs`) are its own crate-local types, and
//! `cspace-planners/Cargo.toml` has no `cspace-planning` dependency at
//! all -- confirmed independently, not merely quoted from the brief. The
//! real, reusable precedent sbp/stomp/pilz all establish is narrower than
//! the brief implied: define a bespoke request/response shape local to the
//! planner crate, backed by already-ported lower-layer types
//! (`cspace_core::state`, `cspace_planning::constraints`), rather than depending on
//! `cspace_planning`'s adapter-pipeline-shaped types or on `cspace_planning::scene`.
//! [`ChompRequest`], [`GoalJointConstraint`] and [`ChompSolution`] below
//! follow that same pattern.
//!
//! # Deviation: `planning_time_limit + 5` is validated before narrowing (`§172`/`§153.1`)
//!
//! Upstream's recovery loop (`chomp_planner.cpp:200-201`) computes
//! `params_nonconst.planning_time_limit_ + 5` (a `double`) and passes it,
//! with no `static_cast`, to `setRecoveryParams`'s `int planning_time_limit`
//! parameter -- an implicit narrowing conversion that is undefined behaviour
//! in C++ whenever the sum falls outside `int`'s range (or is
//! non-finite -- `planning_time_limit_` is a free-standing `pub` `f64` on
//! [`crate::chomp::parameters::ChompParameters`], reachable from
//! `cspace-planning`'s response-adapter code the same way
//! `TotgOptions::resample_dt` is; see `cspace_core::trajectory`'s
//! `time_optimal_trajectory_generation` module for that precedent). [`solve`]
//! rejects `planning_time_limit + 5.0` outside `[i32::MIN, i32::MAX]` (as an
//! `f64` comparison, before any cast) with a typed [`Error`](cspace_core::error::Error) rather than
//! reproducing C++'s UB via Rust's saturating `as`, which has no "right
//! answer" to match here. This deviation is scoped to exactly that rejected
//! range and expires if upstream adds its own validation to
//! `setRecoveryParams`/`ChompParameters::planning_time_limit_`.
use crate::chomp::optimizer::{
    ChompCollisionContext, ChompLoopTrace, ChompObjectiveProgress, ChompOptimizer,
};
use crate::chomp::parameters::ChompParameters;
use crate::chomp::trajectory::ChompTrajectory;
use crate::chomp::utils::shortest_angular_distance;
use cspace_collision::AllowedCollisionMatrix;
use cspace_core::error::{Error, MoveItErrorCode, Result};
use cspace_core::model::joint::JointType;
use cspace_core::state::RobotState;
use cspace_core::trajectory::RobotTrajectory;
use cspace_planning::constraints::JointConstraint;
use nalgebra::DMatrix;
use rand::Rng;

/// One entry of `req.goal_constraints[0].joint_constraints`: a raw
/// `moveit_msgs::msg::JointConstraint` (name, desired position, asymmetric
/// tolerance, weight), unresolved against any [`cspace_core::model::RobotModel`].
///
/// # Deviation: not `cspace_planning::constraints::JointConstraint`
///
/// [`cspace_planning::constraints::JointConstraint::new`] is the resolved,
/// bounds-clamped, continuous-angle-normalized form -- and [`solve`] needs
/// *both* forms upstream uses, for two different purposes, from the same
/// input data: the unresolved raw position to set `goal_state`'s variable
/// directly (`chomp_planner.cpp:108`, so that an out-of-bounds goal is
/// still caught by the immediately-following `goal_state.satisfiesBounds()`
/// check), and a freshly resolved `cspace_planning::constraints::JointConstraint`
/// built from the *same* raw fields for the final tolerance-satisfaction
/// check (`chomp_planner.cpp:292-301`, mirrored in [`solve`] below). Storing
/// only the resolved form (e.g. by taking
/// `&[cspace_planning::constraints::KinematicConstraintSet]` for the whole goal)
/// would lose the raw value the first use needs: resolution silently
/// clamps an out-of-bounds position into bounds
/// (`cspace-planning/src/constraints/joint.rs:167-178`), which would make upstream's
/// `INVALID_ROBOT_STATE` goal-bounds error permanently unreachable through
/// this path. This type exists specifically to keep both uses honest.
#[derive(Debug, Clone, PartialEq)]
pub struct GoalJointConstraint {
    /// `joint_name` (or `"joint/local_variable"` for one variable of a
    /// multi-DOF joint, matching `cspace_planning::constraints::JointConstraint::new`'s
    /// own convention).
    pub joint_name: String,
    /// The desired position, unresolved -- may be outside the joint's
    /// bounds; see this type's doc comment for why that must be preserved.
    pub position: f64,
    /// `tolerance_above`.
    pub tolerance_above: f64,
    /// `tolerance_below`.
    pub tolerance_below: f64,
    /// `weight`.
    pub weight: f64,
}

/// One goal candidate: upstream's `req.goal_constraints[i]`
/// (`moveit_msgs::msg::Constraints`), narrowed to the one sub-field
/// `ChompPlanner::solve` supports.
///
/// # Deviation: joint-only by construction
///
/// Upstream's `Constraints` also carries `position_constraints`/
/// `orientation_constraints`/`visibility_constraints`; `solve` rejects any
/// goal candidate with a non-empty `position_constraints` or
/// `orientation_constraints` (`chomp_planner.cpp:97-103`,
/// [`Error::Code`]`(`[`MoveItErrorCode::InvalidGoalConstraints`]`)`).
/// Rather than accept the full `cspace_planning::constraints::KinematicConstraintSet`
/// shape and run that check at runtime, [`ChompGoal`] only has room for
/// joint constraints in the first place -- the illegal states this crate's
/// other planners must check for at runtime (see `cspace_planners::stomp`'s
/// `Constraint::Joint`-filtering idiom) are unrepresentable here. A caller
/// juggling a richer, `KinematicConstraintSet`-shaped goal (e.g. a future
/// dispatcher choosing between planners) is responsible for deciding
/// up-front whether CHOMP can even attempt it and for extracting the joint
/// half itself; that dispatcher does not exist yet in this workspace. The
/// upstream error path this replaces is the emptiness check below (`solve`
/// still rejects zero joint constraints, matching upstream's
/// `joint_constraints.empty()` half of the same check).
#[derive(Debug, Clone, PartialEq, Default)]
pub struct ChompGoal {
    /// `goal_constraints[i].joint_constraints`. Must be non-empty for
    /// [`solve`] to accept this goal, matching upstream.
    pub joint_constraints: Vec<GoalJointConstraint>,
}

/// A solved CHOMP trajectory, upstream's `MotionPlanDetailedResponse` narrowed
/// to what `ChompPlanner::solve` actually ever fills in (it always resizes
/// every response vector to exactly 1 -- see this module's doc for the full
/// field audit).
///
/// `error_code` and `processing_time` have no field here: `error_code` is
/// this function's own `Result::Err`, matching
/// `cspace_planning::PlanningResponse`'s "distinct" precedent for the same
/// upstream field (see `cspace-planning/src/response.rs`'s D8 delta audit);
/// `processing_time` is dropped per `PORTING-PLAN.md` §138.3 (see this
/// module's doc).
#[derive(Debug, Clone)]
pub struct ChompSolution<'m> {
    /// The optimized trajectory. Upstream's `res.trajectory` is
    /// `Vec<RobotTrajectoryPtr>`, always resized to exactly 1 by this
    /// function; kept as a bare [`RobotTrajectory`] here rather than a
    /// single-element `Vec`, matching `cspace_planning::PlanningResponse::trajectory`'s
    /// own precedent for the same upstream pattern.
    pub trajectory: RobotTrajectory<'m>,
    /// Always `"chomp"`, matching upstream's `res.planner_id = "chomp"`
    /// (`chomp_planner.cpp:68`, unconditional, set before any failure
    /// branch can return).
    pub planner_id: String,
    /// Always `"plan"`, matching upstream's `res.description[0] = "plan"`
    /// (`chomp_planner.cpp:273`) -- the only value this field can ever hold,
    /// kept as a real field rather than an implicit constant since it is
    /// genuine upstream response content a caller may display.
    pub description: String,
    /// CHOMP's own objective at the seed trajectory and at `trajectory`, or
    /// `None` when `max_iterations` is 0 and no iteration ever evaluated it.
    ///
    /// # Deviation: upstream discards this at exactly this boundary
    ///
    /// This field has **no upstream counterpart**, and unlike the four fields
    /// above it is not a narrowing of one. Upstream's `solve` fills
    /// `planning_interface::MotionPlanDetailedResponse`, which carries no
    /// objective value, and `chomp_planner.cpp` does not contain the word
    /// `cost` anywhere -- the optimizer computes the number, keeps it on the
    /// private `best_group_trajectory_cost_` (`chomp_optimizer.hpp:150`), and
    /// `ChompPlanner::solve` returns `void` having never read it. The three
    /// accessors that could read it are private too
    /// (`chomp_optimizer.hpp:208-210`, below `private:` at `:83`).
    ///
    /// It is added because the value's absence was itself measured:
    /// `PORTING-PLAN.md` §264.12 records that with no cost at this boundary,
    /// CHOMP's quality item cannot make the *paired improvement* claim its
    /// Phase 7 analogue makes, and degrades to a ratio band against a
    /// straight-line lower bound. What is mirrored rather than invented is the
    /// shape: upstream's single emission of these numbers
    /// (`chomp_optimizer.cpp:310`) logs the two terms separately, so they are
    /// reported separately here. See [`crate::chomp::optimizer::ChompObjective`] for
    /// that reading in full.
    pub objective: Option<ChompObjectiveProgress>,
    /// What the optimizer's loop did to produce `trajectory`, or `None` when
    /// `optimize` was never called.
    ///
    /// # Deviation: the same boundary, for the same reason
    ///
    /// Also no upstream counterpart. `improvement == 0` on this response has
    /// several possible causes and they are not distinguishable from the
    /// objective alone -- see [`crate::chomp::optimizer::ChompLoopTrace`], which
    /// names each and the field that separates it. `PORTING-PLAN.md` §296
    /// is the measurement that needed them separated.
    pub loop_trace: Option<ChompLoopTrace>,
}

/// [`solve`]'s inputs, bundled to keep the function's own argument count
/// reasonable -- upstream's four value parameters
/// (`planning_scene`/`req`/`params`/`res`) expand to more independent pieces
/// once `planning_scene` and `res`'s dual read/write role are unpacked (see
/// this module's doc and [`ChompSolution`]'s doc), so grouping them here
/// follows the same request-struct convention `cspace-planning`,
/// `cspace_planners::sbp::registry`, and `cspace_planners::pilz` already use
/// for their own `solve`-equivalents, rather than a lint-suppressed
/// long parameter list.
#[derive(Debug, Clone, Copy)]
pub struct ChompRequest<'a, 'm> {
    /// Upstream's `planning_scene->getCurrentState()`, already overlaid
    /// with `req.start_state` -- see this module's "no `cspace_planning::scene`"
    /// deviation.
    pub start_state: &'a RobotState<'m>,
    /// `req.group_name`.
    pub group_name: &'a str,
    /// `req.goal_constraints`. Must have exactly one entry, matching
    /// upstream's own check.
    pub goal_constraints: &'a [ChompGoal],
    /// The CHOMP tuning parameters (a separate constructor argument
    /// upstream too, not part of `req`).
    pub params: &'a ChompParameters,
    /// Upstream's `res.trajectory[0]`, read (not written) only when
    /// `params.trajectory_initialization_method == "fillTrajectory"`
    /// (`chomp_planner.cpp:151-164`) -- upstream overloads the *output*
    /// parameter as an input for this one method; this port makes that
    /// input an explicit field instead of overloading [`ChompSolution`].
    pub seed_trajectory: Option<&'a RobotTrajectory<'m>>,
}

/// Builds the two-point (start, goal) seed [`ChompTrajectory`] `solve` fills
/// in and optimizes -- upstream `chomp_planner.cpp:76-136`: the start/goal
/// bounds checks, `robotStateToArray` at both ends, and the continuous-joint
/// shortest-angular-distance fix. Factored out of [`solve`] itself (rather
/// than upstream's single inline block) so this exact, pinned output is
/// unit testable directly, without depending on how many optimizer
/// iterations run: every point strictly between index 0 and the goal index
/// is still `0.0` here, not yet filled in by a
/// `trajectory_initialization_method`.
///
/// The returned trajectory's row 0 and last row (both written here) stay
/// exactly these values all the way through [`solve`]'s optimizer loop, not
/// just approximately: [`ChompTrajectory::from_num_points`] hardcodes the
/// un-padded trajectory's own `start_index = 1` / `end_index = num_points -
/// 2` (excluding *both* endpoints from the free range), and
/// [`ChompTrajectory::update_from_group_trajectory`] -- the only site that
/// ever writes optimizer output back onto this outer trajectory -- only
/// overwrites rows in `[start_index, end_index]`. (The *internal*, padded
/// `group_trajectory` `ChompOptimizer` builds from this one via
/// `from_source_trajectory` has a different free range that does include
/// its own copies of the start/goal rows, per
/// [`crate::chomp::trajectory::ChompTrajectory::num_free_points`]'s doc -- but that
/// range is never written back past this outer trajectory's own
/// `start_index`/`end_index` bounds.)
fn build_seed_trajectory<'m>(
    start_state: &RobotState<'m>,
    group_name: &str,
    goal_constraints: &[ChompGoal],
) -> Result<ChompTrajectory> {
    let robot_model = start_state.model();

    if !start_state.satisfies_bounds(0.0) {
        return Err(Error::Code(MoveItErrorCode::InvalidRobotState));
    }

    let mut trajectory = ChompTrajectory::from_duration(robot_model, 3.0, 0.03, group_name)?;
    let group = robot_model.joint_model_group(group_name)?;
    trajectory.assign_chomp_trajectory_point_from_robot_state(start_state, 0, group)?;

    if goal_constraints.len() != 1 {
        return Err(Error::Code(MoveItErrorCode::InvalidGoalConstraints));
    }
    if goal_constraints[0].joint_constraints.is_empty() {
        return Err(Error::Code(MoveItErrorCode::InvalidGoalConstraints));
    }

    let goal_index = trajectory.num_points() - 1;
    let mut goal_state = start_state.clone();
    for jc in &goal_constraints[0].joint_constraints {
        goal_state.set_variable_position(&jc.joint_name, jc.position)?;
    }
    if !goal_state.satisfies_bounds(0.0) {
        return Err(Error::Code(MoveItErrorCode::InvalidRobotState));
    }
    trajectory.assign_chomp_trajectory_point_from_robot_state(&goal_state, goal_index, group)?;

    // Fix the goal to move the shortest angular distance for wrap-around
    // joints. Ported from `chomp_planner.cpp:119-136`.
    for (i, &model_index) in group.active_joint_indices().iter().enumerate() {
        let joint_model = robot_model.joint_model_at(model_index);
        if joint_model.joint_type() == JointType::Revolute
            && let Some(revolute) = joint_model.as_revolute()
            && revolute.is_continuous()
        {
            let start = trajectory.trajectory_point(0)[i];
            let mut goal_row = trajectory.trajectory_point(goal_index);
            let end = goal_row[i];
            goal_row[i] = start + shortest_angular_distance(start, end);
            trajectory.set_trajectory_point(goal_index, &goal_row);
        }
    }

    Ok(trajectory)
}

/// Validates `planning_time_limit + 5` fits in [`i32`] before the recovery
/// loop narrows it into `ChompParameters::set_recovery_params`'s `i32`
/// parameter. See [`solve`]'s module doc, "Deviation: `planning_time_limit +
/// 5` is validated before narrowing (`§172`/`§153.1`)", for why this checks
/// the sum in `f64` space rather than transcribing upstream's uncast,
/// UB-on-overflow narrowing directly.
fn validate_recovery_time_limit(planning_time_limit: f64) -> Result<i32> {
    let recovery_time_limit = planning_time_limit + 5.0;
    if recovery_time_limit.is_finite()
        && recovery_time_limit >= f64::from(i32::MIN)
        && recovery_time_limit <= f64::from(i32::MAX)
    {
        Ok(recovery_time_limit as i32)
    } else {
        Err(Error::other(format!(
            "planning_time_limit {planning_time_limit} + 5 does not fit in the recovery \
             parameters' i32 field"
        )))
    }
}

/// The shared implementation behind [`solve`] and [`solve_with_trace`] --
/// see [`solve`]'s doc comment for the port itself and its `# Errors`
/// section, both unchanged here. The only addition is `trace_out`: written
/// every time an attempt's [`ChompOptimizer::optimize`] call returns `Ok`,
/// so it holds the most recently *completed* attempt's [`ChompLoopTrace`]
/// no matter which of this function's `Err` returns fires afterwards. See
/// [`solve_with_trace`]'s doc for why a bare port of `ChompPlanner::solve`
/// cannot expose this itself.
fn solve_inner<'m>(
    request: &ChompRequest<'_, 'm>,
    collision: &mut ChompCollisionContext<'_, 'm>,
    acm: Option<&AllowedCollisionMatrix>,
    mesh_to_mesh_collision_free: &mut dyn FnMut(&RobotState<'m>, &DMatrix<f64>) -> bool,
    rng: &mut impl Rng,
    trace_out: &mut Option<ChompLoopTrace>,
) -> Result<ChompSolution<'m>> {
    let start_state = request.start_state;
    let group_name = request.group_name;
    let robot_model = start_state.model();

    let mut trajectory = build_seed_trajectory(start_state, group_name, request.goal_constraints)?;
    let group = robot_model.joint_model_group(group_name)?;

    match request.params.trajectory_initialization_method.as_str() {
        "quintic-spline" => trajectory.fill_in_min_jerk(),
        "linear" => trajectory.fill_in_linear_interpolation(),
        "cubic" => trajectory.fill_in_cubic_interpolation(),
        "fillTrajectory" => {
            let seed = request
                .seed_trajectory
                .ok_or(Error::Code(MoveItErrorCode::Failure))?;
            if !trajectory.fill_in_from_trajectory(seed)? {
                return Err(Error::Code(MoveItErrorCode::Failure));
            }
        }
        _ => return Err(Error::Code(MoveItErrorCode::Failure)),
    }

    // Recovery loop: replan with progressively looser parameters if
    // `enable_failure_recovery` is set and the optimizer fails. Ported from
    // `chomp_planner.cpp:177-243`.
    let mut params_nonconst = request.params.clone();
    let mut replan_count = 0i32;
    let mut replan_flag = false;
    let mut optimizer;
    loop {
        if replan_flag {
            // See `validate_recovery_time_limit`'s doc and this module's
            // "Deviations from upstream" note (`§172`/`§153.1`) for why this
            // validates before narrowing instead of transcribing upstream's
            // uncast `planning_time_limit_ + 5` directly.
            let recovery_time_limit =
                validate_recovery_time_limit(params_nonconst.planning_time_limit)?;
            params_nonconst.set_recovery_params(
                params_nonconst.learning_rate + 0.02,
                params_nonconst.ridge_factor + 0.002,
                recovery_time_limit,
                params_nonconst.max_iterations + 50,
            );
        }

        // Upstream separately checks `optimizer->isInitialized()` after
        // construction; this port's constructor returns `Err` for every
        // failure that check could observe (see `ChompOptimizer::new`'s own
        // doc, "every other upstream failure ... is a typed `Err`
        // instead"), so a successful `?` here already implies
        // `is_initialized() == true` and a second check would be dead code.
        optimizer = ChompOptimizer::new(
            &trajectory,
            group_name,
            &params_nonconst,
            start_state,
            collision,
            acm,
        )
        .map_err(|_| Error::Code(MoveItErrorCode::PlanningFailed))?;

        let optimization_result =
            optimizer.optimize(&mut trajectory, collision, mesh_to_mesh_collision_free, rng)?;
        // `optimize` unconditionally sets its trace before returning `Ok`
        // (`ChompLoopTrace`'s doc), so capture it here rather than only in
        // the `Ok(ChompSolution { .. })` built below -- this is what makes
        // the trace reachable from the `Err` returns past this point too
        // (`is_collision_free() == false`, goal-tolerance failure). A later
        // replan iteration overwrites it with its own attempt's trace; if
        // that iteration's `ChompOptimizer::new` fails before reaching this
        // line, this keeps the last attempt that actually ran one instead
        // of losing it.
        *trace_out = optimizer.loop_trace();

        if params_nonconst.enable_failure_recovery {
            if !optimization_result && replan_count < params_nonconst.max_recovery_attempts {
                replan_count += 1;
                replan_flag = true;
            } else {
                break;
            }
        } else {
            break;
        }
    }

    // Fill in the entire output trajectory. Ported from
    // `chomp_planner.cpp:255-268`.
    let mut result = RobotTrajectory::for_group_name(robot_model, group_name)?;
    for i in 0..trajectory.num_points() {
        let row = trajectory.trajectory_point(i);
        let mut state = start_state.clone();
        for (joint_index, &model_index) in group.active_joint_indices().iter().enumerate() {
            let joint = robot_model.joint_model_at(model_index);
            state.set_variable_position(joint.name(), row[joint_index])?;
        }
        result.add_suffix_way_point(state, 0.0)?;
    }

    if !optimizer.is_collision_free() {
        return Err(Error::Code(MoveItErrorCode::InvalidMotionPlan));
    }

    // Check that the final state is within goal tolerances. Ported from
    // `chomp_planner.cpp:292-302`, resolving a fresh
    // `cspace_planning::constraints::JointConstraint` per goal joint constraint from
    // the same raw fields [`GoalJointConstraint`] carries -- see that
    // type's doc comment for why the raw form, not a pre-resolved
    // `KinematicConstraintSet`, is what this function accepts. Upstream's
    // `!jc.configure(constraint) || !jc.decide(last_state).satisfied`
    // converges *both* halves to the same `GOAL_CONSTRAINTS_VIOLATED` code
    // (`chomp_planner.cpp:294-299`) -- a plain `?` on `JointConstraint::new`
    // would instead let the construct half escape as whatever typed
    // `cspace_core::error::Error` variant construction failed with (e.g.
    // `Error::Construct` for a negative tolerance), so it is mapped
    // explicitly here instead.
    let last_state = result.last_way_point_mut()?;
    let posed = last_state.update();
    for gjc in &request.goal_constraints[0].joint_constraints {
        let jc = JointConstraint::new(
            robot_model,
            &gjc.joint_name,
            gjc.position,
            gjc.tolerance_above,
            gjc.tolerance_below,
            gjc.weight,
        )
        .map_err(|_| Error::Code(MoveItErrorCode::GoalConstraintsViolated))?;
        if !jc.decide(&posed).satisfied {
            return Err(Error::Code(MoveItErrorCode::GoalConstraintsViolated));
        }
    }

    Ok(ChompSolution {
        trajectory: result,
        planner_id: "chomp".to_string(),
        description: "plan".to_string(),
        // The optimizer that produced `result`. Under
        // `enable_failure_recovery` the loop above builds a fresh
        // `ChompOptimizer` per attempt, so this reads the last one -- the
        // only one whose `best_group_trajectory_` is what `trajectory`
        // was filled from.
        objective: optimizer.objective(),
        loop_trace: optimizer.loop_trace(),
    })
}

/// Ported from `ChompPlanner::solve` (`chomp_planner.cpp:63-306`). See this
/// module's doc comment for the field-coverage measurement behind porting
/// it (`eb4fa4e`), and [`ChompRequest`]'s doc for why `planning_scene`/
/// `cspace_planning` are not parameters here.
///
/// # Errors
///
/// [`MoveItErrorCode::InvalidRobotState`] if `request.start_state` or the
/// constructed goal state violates joint limits.
/// [`MoveItErrorCode::InvalidGoalConstraints`] if `request.goal_constraints`
/// does not have exactly one entry, or that entry has no joint constraints
/// (see [`ChompGoal`]'s doc comment for the position/orientation-rejection
/// half of upstream's check, which this port makes structural instead).
/// [`MoveItErrorCode::PlanningFailed`] if the optimizer fails to
/// initialize -- upstream sets `PLANNING_FAILED` explicitly on this path
/// (`chomp_planner.cpp:211`).
/// [`MoveItErrorCode::Failure`] if `trajectory_initialization_method` is
/// not one of [`crate::chomp::parameters::VALID_INITIALIZATION_METHODS`], the
/// `"fillTrajectory"` method's required seed trajectory is missing, or
/// `fill_in_from_trajectory` reports fewer than two points -- upstream
/// leaves `res.error_code` **unset** on all three of these paths
/// (`chomp_planner.cpp:151-169`; no `res.error_code.val = ...` assignment
/// on any of the three branches, unlike every other failure branch in
/// this function), an upstream gap this port does not reproduce as "leave
/// the error unspecified": [`MoveItErrorCode::Failure`] is used instead,
/// matching the same "no code was actually set upstream" fallback
/// `cspace_planners::pilz::trajectory_generator`'s `failure` helper already
/// uses for the same situation.
/// [`MoveItErrorCode::InvalidMotionPlan`] if the optimizer does not report
/// the final trajectory collision-free.
/// [`MoveItErrorCode::GoalConstraintsViolated`] if the final trajectory
/// state does not satisfy every goal joint constraint's tolerance.
pub fn solve<'m>(
    request: &ChompRequest<'_, 'm>,
    collision: &mut ChompCollisionContext<'_, 'm>,
    acm: Option<&AllowedCollisionMatrix>,
    mesh_to_mesh_collision_free: &mut dyn FnMut(&RobotState<'m>, &DMatrix<f64>) -> bool,
    rng: &mut impl Rng,
) -> Result<ChompSolution<'m>> {
    let mut trace = None;
    solve_inner(
        request,
        collision,
        acm,
        mesh_to_mesh_collision_free,
        rng,
        &mut trace,
    )
}

/// [`solve`], plus the [`ChompLoopTrace`] of the attempt that produced this
/// outcome -- on success **and** on `Err`.
///
/// # Deviation: no upstream counterpart, closing a port-only gap
///
/// [`ChompSolution::loop_trace`] already carries the trace on success.
/// [`ChompOptimizer::optimize`] records why its loop stopped
/// (`chomp_optimizer.cpp:290-518`) into a [`ChompLoopTrace`] unconditionally
/// -- converged or not -- but [`solve`] only ever attached that trace to the
/// `Ok` value it builds. Every `Err` it can return *after* an optimizer has
/// completed a loop (`is_collision_free() == false`, or a goal-tolerance
/// failure on an otherwise-successful optimization) drops `optimizer`, and
/// the trace along with it -- so the diagnostic was reachable on exactly
/// the runs that did not need it, and unreachable on every run that did.
///
/// [`solve`]'s own signature and semantics stay exactly as they are for
/// existing callers: this crate has no upstream contract to keep for a
/// port-only diagnostic, so the trace rides beside the `Result` here
/// instead of inside [`Error`] -- `Error` is shared workspace-wide with no
/// CHOMP-specific variant to carry it, and giving it one would mean every
/// other crate's `match` on `Error` gaining a case it can never produce.
///
/// `None` when no attempt ever completed a loop: every failure before the
/// recovery loop's first `optimize()` call returns (bounds or
/// goal-constraint validation, trajectory initialization, optimizer
/// construction) has no loop to report on. Under
/// [`ChompParameters::enable_failure_recovery`], this is the most recently
/// *completed* attempt's trace, which is the final attempt unless a later
/// replan's own [`ChompOptimizer::new`] failed to construct.
pub fn solve_with_trace<'m>(
    request: &ChompRequest<'_, 'm>,
    collision: &mut ChompCollisionContext<'_, 'm>,
    acm: Option<&AllowedCollisionMatrix>,
    mesh_to_mesh_collision_free: &mut dyn FnMut(&RobotState<'m>, &DMatrix<f64>) -> bool,
    rng: &mut impl Rng,
) -> (Result<ChompSolution<'m>>, Option<ChompLoopTrace>) {
    let mut trace = None;
    let result = solve_inner(
        request,
        collision,
        acm,
        mesh_to_mesh_collision_free,
        rng,
        &mut trace,
    );
    (result, trace)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chomp::optimizer::ChompExit;
    use approx::assert_relative_eq;
    use cspace_collision::distance_field::{
        DistanceField, DistanceFieldCollisionCache, DistanceFieldConfig, GridGeometry,
        PropagationDistanceField, add_link_body_decompositions,
    };
    use cspace_core::geometry::Vector3;
    use cspace_core::model::MeshSearchPaths;
    use cspace_core::model::RobotModel;
    use cspace_core::srdf::SrdfModel;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    const GROUP: &str = "chain";

    /// A two-joint chain: `j1` a bounded revolute joint (`[-1, 1]`), `j2` a
    /// continuous revolute joint -- one fixture covers both the
    /// bounds-violation tests (via `j1`) and the wrap-around fix test (via
    /// `j2`), following `optimizer.rs`'s own `chomp_collision_model` shape
    /// (box collision geometry, links spaced by a `0.3 0 0` origin so the
    /// three collision boxes never touch at the identity pose).
    fn two_joint_chain_model() -> RobotModel {
        let urdf_xml = r#"<?xml version="1.0"?>
<robot name="two_joint_chomp_planner">
  <link name="base">
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <link name="mid">
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <link name="tip">
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base"/>
    <child link="mid"/>
    <origin xyz="0.3 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="j2" type="continuous">
    <parent link="mid"/>
    <child link="tip"/>
    <origin xyz="0.3 0 0"/>
    <axis xyz="0 0 1"/>
    <limit effort="1" velocity="1"/>
  </joint>
</robot>
"#;
        let srdf_xml = r#"<?xml version="1.0"?>
<robot name="two_joint_chomp_planner">
  <group name="chain">
    <chain base_link="base" tip_link="tip"/>
  </group>
</robot>
"#;
        let urdf: urdf_rs::Robot = urdf_rs::read_from_string(urdf_xml).unwrap();
        let srdf = SrdfModel::parse_str(srdf_xml).expect("srdf must parse");
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("two_joint_chomp_planner model must build");
        // PORTING-PLAN.md §196: an SRDF chain group over a fixed joint
        // resolves to `updated_link_names() == []` with no error and no
        // warning, so every test built on this fixture would pass
        // vacuously -- every check below that reads this group (the many
        // `group_name: GROUP` collision requests,
        // `RobotTrajectory::for_group_name`) resolves through that set, not
        // the raw `link_names()`/`joint_names()` topology (see
        // `ParryCollisionEnv::active_group_links`). Both `j1` and `j2`
        // above are active joints, not `fixed`, but assert the group
        // actually has updated links rather than trusting that stays true.
        cspace_core::test_support::assert_group_has_updated_links(&model, GROUP);
        model
    }

    fn collision_field_config() -> DistanceFieldConfig {
        let size = Vector3::new(3.0, 3.0, 3.0);
        let origin_center = Vector3::new(0.0, 0.0, 0.0);
        DistanceFieldConfig {
            geometry: GridGeometry::new(size, origin_center - 0.5 * size, 0.02).unwrap(),
            max_propagation_distance: 0.3,
            use_signed_distance_field: false,
        }
    }

    fn collision_cache(model: &RobotModel) -> DistanceFieldCollisionCache<'_> {
        let padding = cspace_collision::LinkPaddingScale::new();
        let decompositions = add_link_body_decompositions(model, 0.02, &padding, None).unwrap();
        DistanceFieldCollisionCache::new(decompositions, collision_field_config(), 0.0)
    }

    fn empty_env_field() -> PropagationDistanceField {
        let config = collision_field_config();
        PropagationDistanceField::new(
            config.geometry,
            config.max_propagation_distance,
            config.use_signed_distance_field,
        )
        .unwrap()
    }

    fn joint_goal(joint_name: &str, position: f64) -> GoalJointConstraint {
        GoalJointConstraint {
            joint_name: joint_name.to_string(),
            position,
            tolerance_above: 0.01,
            tolerance_below: 0.01,
            weight: 1.0,
        }
    }

    fn assert_code(result: &Result<ChompSolution<'_>>, expected: MoveItErrorCode) {
        match result {
            Err(Error::Code(code)) => assert_eq!(*code, expected, "got {result:?}"),
            other => panic!("expected Err(Error::Code({expected:?})), got {other:?}"),
        }
    }

    #[test]
    fn solve_rejects_start_state_violating_bounds() {
        let model = two_joint_chain_model();
        let mut start_state = RobotState::new(&model);
        start_state.set_variable_position("j1", 5.0).unwrap();
        let params = ChompParameters::default();
        let mut cache = collision_cache(&model);
        let field = empty_env_field();
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let mut rng = ChaCha8Rng::seed_from_u64(1);
        let goal = ChompGoal {
            joint_constraints: vec![joint_goal("j1", 0.0), joint_goal("j2", 0.0)],
        };
        let request = ChompRequest {
            start_state: &start_state,
            group_name: GROUP,
            goal_constraints: std::slice::from_ref(&goal),
            params: &params,
            seed_trajectory: None,
        };

        let result = solve(&request, &mut collision, None, &mut |_, _| false, &mut rng);

        assert_code(&result, MoveItErrorCode::InvalidRobotState);
    }

    /// The other side of
    /// `solve_with_trace_reports_iteration_bound_on_a_failed_plan`'s
    /// boundary: an `Err` that fires *before* `ChompOptimizer::optimize`
    /// is ever called has no loop to report on, and [`solve_with_trace`]
    /// must say so with `None` rather than carrying over some earlier
    /// call's trace (this is the first request `collision`/`rng` see in
    /// the test, so there is no earlier call to leak).
    #[test]
    fn solve_with_trace_reports_no_trace_when_the_optimizer_never_runs() {
        let model = two_joint_chain_model();
        let mut start_state = RobotState::new(&model);
        start_state.set_variable_position("j1", 5.0).unwrap();
        let params = ChompParameters::default();
        let mut cache = collision_cache(&model);
        let field = empty_env_field();
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let mut rng = ChaCha8Rng::seed_from_u64(1);
        let goal = ChompGoal {
            joint_constraints: vec![joint_goal("j1", 0.0), joint_goal("j2", 0.0)],
        };
        let request = ChompRequest {
            start_state: &start_state,
            group_name: GROUP,
            goal_constraints: std::slice::from_ref(&goal),
            params: &params,
            seed_trajectory: None,
        };

        let (result, trace) =
            solve_with_trace(&request, &mut collision, None, &mut |_, _| false, &mut rng);

        assert_code(&result, MoveItErrorCode::InvalidRobotState);
        assert_eq!(trace, None);
    }

    #[test]
    fn solve_rejects_zero_goal_constraints() {
        let model = two_joint_chain_model();
        let start_state = RobotState::new(&model);
        let params = ChompParameters::default();
        let mut cache = collision_cache(&model);
        let field = empty_env_field();
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let mut rng = ChaCha8Rng::seed_from_u64(1);
        let request = ChompRequest {
            start_state: &start_state,
            group_name: GROUP,
            goal_constraints: &[],
            params: &params,
            seed_trajectory: None,
        };

        let result = solve(&request, &mut collision, None, &mut |_, _| false, &mut rng);

        assert_code(&result, MoveItErrorCode::InvalidGoalConstraints);
    }

    #[test]
    fn solve_rejects_multiple_goal_constraints() {
        let model = two_joint_chain_model();
        let start_state = RobotState::new(&model);
        let params = ChompParameters::default();
        let mut cache = collision_cache(&model);
        let field = empty_env_field();
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let mut rng = ChaCha8Rng::seed_from_u64(1);
        let goal = ChompGoal {
            joint_constraints: vec![joint_goal("j1", 0.0)],
        };
        let request = ChompRequest {
            start_state: &start_state,
            group_name: GROUP,
            goal_constraints: &[goal.clone(), goal],
            params: &params,
            seed_trajectory: None,
        };

        let result = solve(&request, &mut collision, None, &mut |_, _| false, &mut rng);

        assert_code(&result, MoveItErrorCode::InvalidGoalConstraints);
    }

    #[test]
    fn solve_rejects_goal_with_no_joint_constraints() {
        let model = two_joint_chain_model();
        let start_state = RobotState::new(&model);
        let params = ChompParameters::default();
        let mut cache = collision_cache(&model);
        let field = empty_env_field();
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let mut rng = ChaCha8Rng::seed_from_u64(1);
        let goal = ChompGoal::default();
        let request = ChompRequest {
            start_state: &start_state,
            group_name: GROUP,
            goal_constraints: std::slice::from_ref(&goal),
            params: &params,
            seed_trajectory: None,
        };

        let result = solve(&request, &mut collision, None, &mut |_, _| false, &mut rng);

        assert_code(&result, MoveItErrorCode::InvalidGoalConstraints);
    }

    #[test]
    fn solve_rejects_goal_state_violating_bounds() {
        let model = two_joint_chain_model();
        let start_state = RobotState::new(&model);
        let params = ChompParameters::default();
        let mut cache = collision_cache(&model);
        let field = empty_env_field();
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let mut rng = ChaCha8Rng::seed_from_u64(1);
        let goal = ChompGoal {
            joint_constraints: vec![joint_goal("j1", 5.0), joint_goal("j2", 0.0)],
        };
        let request = ChompRequest {
            start_state: &start_state,
            group_name: GROUP,
            goal_constraints: std::slice::from_ref(&goal),
            params: &params,
            seed_trajectory: None,
        };

        let result = solve(&request, &mut collision, None, &mut |_, _| false, &mut rng);

        assert_code(&result, MoveItErrorCode::InvalidRobotState);
    }

    #[test]
    fn solve_rejects_invalid_trajectory_initialization_method() {
        let model = two_joint_chain_model();
        let start_state = RobotState::new(&model);
        let params = ChompParameters {
            trajectory_initialization_method: "not-a-real-method".to_string(),
            ..ChompParameters::default()
        };
        let mut cache = collision_cache(&model);
        let field = empty_env_field();
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let mut rng = ChaCha8Rng::seed_from_u64(1);
        let goal = ChompGoal {
            joint_constraints: vec![joint_goal("j1", 0.0), joint_goal("j2", 0.0)],
        };
        let request = ChompRequest {
            start_state: &start_state,
            group_name: GROUP,
            goal_constraints: std::slice::from_ref(&goal),
            params: &params,
            seed_trajectory: None,
        };

        let result = solve(&request, &mut collision, None, &mut |_, _| false, &mut rng);

        assert_code(&result, MoveItErrorCode::Failure);
    }

    #[test]
    fn solve_rejects_fill_trajectory_without_seed() {
        let model = two_joint_chain_model();
        let start_state = RobotState::new(&model);
        let params = ChompParameters {
            trajectory_initialization_method: "fillTrajectory".to_string(),
            ..ChompParameters::default()
        };
        let mut cache = collision_cache(&model);
        let field = empty_env_field();
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let mut rng = ChaCha8Rng::seed_from_u64(1);
        let goal = ChompGoal {
            joint_constraints: vec![joint_goal("j1", 0.0), joint_goal("j2", 0.0)],
        };
        let request = ChompRequest {
            start_state: &start_state,
            group_name: GROUP,
            goal_constraints: std::slice::from_ref(&goal),
            params: &params,
            seed_trajectory: None,
        };

        let result = solve(&request, &mut collision, None, &mut |_, _| false, &mut rng);

        assert_code(&result, MoveItErrorCode::Failure);
    }

    #[test]
    fn solve_rejects_fill_trajectory_with_too_short_seed() {
        let model = two_joint_chain_model();
        let start_state = RobotState::new(&model);
        let params = ChompParameters {
            trajectory_initialization_method: "fillTrajectory".to_string(),
            ..ChompParameters::default()
        };
        let mut cache = collision_cache(&model);
        let field = empty_env_field();
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let mut rng = ChaCha8Rng::seed_from_u64(1);
        let goal = ChompGoal {
            joint_constraints: vec![joint_goal("j1", 0.0), joint_goal("j2", 0.0)],
        };
        let mut seed = RobotTrajectory::for_group_name(&model, GROUP).unwrap();
        seed.add_suffix_way_point(RobotState::new(&model), 0.0)
            .unwrap();
        let request = ChompRequest {
            start_state: &start_state,
            group_name: GROUP,
            goal_constraints: std::slice::from_ref(&goal),
            params: &params,
            seed_trajectory: Some(&seed),
        };

        let result = solve(&request, &mut collision, None, &mut |_, _| false, &mut rng);

        assert_code(&result, MoveItErrorCode::Failure);
    }

    #[test]
    fn solve_succeeds_with_no_obstacles_and_produces_a_101_point_trajectory() {
        let model = two_joint_chain_model();
        let start_state = RobotState::new(&model);
        let params = ChompParameters {
            max_iterations: 5,
            ..ChompParameters::default()
        };
        let mut cache = collision_cache(&model);
        let field = empty_env_field();
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let mut rng = ChaCha8Rng::seed_from_u64(7);
        let goal = ChompGoal {
            joint_constraints: vec![joint_goal("j1", 0.5), joint_goal("j2", 1.0)],
        };
        let request = ChompRequest {
            start_state: &start_state,
            group_name: GROUP,
            goal_constraints: std::slice::from_ref(&goal),
            params: &params,
            seed_trajectory: None,
        };

        let solution = solve(&request, &mut collision, None, &mut |_, _| false, &mut rng)
            .expect("no obstacles, in-bounds goal: solve must succeed");

        assert_eq!(solution.planner_id, "chomp");
        assert_eq!(solution.description, "plan");
        // `ChompPlanner::solve` hardcodes a 3.0s / 0.03s trajectory
        // (`chomp_planner.cpp:87`), giving 101 points regardless of the
        // request -- not configurable, matching upstream exactly.
        assert_eq!(solution.trajectory.way_point_count(), 101);
        assert_eq!(solution.trajectory.group_name(), GROUP);

        // `ChompTrajectory::from_num_points` hardcodes the un-padded
        // trajectory's own `start_index = 1` / `end_index = num_points - 2`
        // (`trajectory.rs:127-128`), and `update_from_group_trajectory`
        // (`trajectory.rs:454-462`, the only site that ever writes optimizer
        // output back onto this outer trajectory) only overwrites rows in
        // `[start_index, end_index]` -- so row 0 and the last row are never
        // written by any call to `optimize()`, at any iteration count. The
        // start/goal rows this test asserts below are therefore expected to
        // be *exact*, not merely converged-close.
        let first = solution.trajectory.first_way_point().unwrap();
        assert_relative_eq!(first.variable_position("j1").unwrap(), 0.0, epsilon = 1e-12);
        assert_relative_eq!(first.variable_position("j2").unwrap(), 0.0, epsilon = 1e-12);

        let last = solution.trajectory.last_way_point().unwrap();
        assert_relative_eq!(last.variable_position("j1").unwrap(), 0.5, epsilon = 1e-12);
        assert_relative_eq!(last.variable_position("j2").unwrap(), 1.0, epsilon = 1e-12);
    }

    /// Upstream's `!jc.configure(constraint)` half of the goal-tolerance
    /// `||` (`chomp_planner.cpp:292-302`): a malformed goal joint
    /// constraint (here, a negative tolerance) must fail with the *same*
    /// `GoalConstraintsViolated` code as a satisfied-tolerance check that
    /// simply fails, not with `JointConstraint::new`'s own construction
    /// error -- upstream converges both halves of that `||` to one error
    /// code, and never reaches `decide()` for the failing half either way.
    #[test]
    fn solve_reports_goal_constraints_violated_for_a_malformed_goal_tolerance() {
        let model = two_joint_chain_model();
        let start_state = RobotState::new(&model);
        let params = ChompParameters {
            max_iterations: 5,
            ..ChompParameters::default()
        };
        let mut cache = collision_cache(&model);
        let field = empty_env_field();
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let mut rng = ChaCha8Rng::seed_from_u64(7);
        let mut malformed_j1_goal = joint_goal("j1", 0.5);
        // `JointConstraint::new` rejects a negative tolerance outright --
        // this plan otherwise succeeds (same fixture as
        // `solve_succeeds_with_no_obstacles_and_produces_a_101_point_trajectory`),
        // so the only way to reach `Err` here is through the goal-tolerance
        // loop's construction step, not a genuinely unsatisfied tolerance.
        malformed_j1_goal.tolerance_above = -0.01;
        let goal = ChompGoal {
            joint_constraints: vec![malformed_j1_goal, joint_goal("j2", 1.0)],
        };
        let request = ChompRequest {
            start_state: &start_state,
            group_name: GROUP,
            goal_constraints: std::slice::from_ref(&goal),
            params: &params,
            seed_trajectory: None,
        };

        let result = solve(&request, &mut collision, None, &mut |_, _| false, &mut rng);

        assert_code(&result, MoveItErrorCode::GoalConstraintsViolated);
    }

    /// Item 3 (this round): upstream's `chomp_moveit_test_rrbot.cpp`
    /// `collisionAtEndOfPath` (target `[M_PI/2.0, 0]`, blocked by a fixed
    /// obstacle link the URDF places in the arm's path) drives
    /// `MotionPlanResponse::error_code == INVALID_MOTION_PLAN` -- the one
    /// upstream integration-test outcome no `solve_*` test here had
    /// exercised (the other four rrbot/panda cases either match an
    /// existing `solve_rejects_*`/`solve_succeeds_*` test already, or are
    /// `move_group`'s own pre-planning state validation, not
    /// `chomp_motion_planner`'s -- see `doc/claim-audit/cspace-planners-chomp.md`).
    ///
    /// `M_PI/2.0` itself is rrbot-URDF-specific (link lengths, obstacle
    /// placement) and not portable, matching the precedent already set for
    /// `ChompGoal`'s "joint-only by construction" deviation: what upstream's
    /// test actually pins is the *mechanism* -- `solve` returns
    /// `Err(InvalidMotionPlan)` when [`ChompOptimizer::is_collision_free`]
    /// is `false` after `optimize()` returns (`planner.rs`'s own
    /// `if !optimizer.is_collision_free()` check, ported from
    /// `chomp_planner.cpp:270-273`) -- not the specific joint values that
    /// happen to trigger it on rrbot's geometry.
    ///
    /// # Why the goal cannot simply sit on the obstacle
    ///
    /// The first attempt at this test used `start == goal ==` the model's
    /// identity pose with an obstacle on the (stationary) tip -- and
    /// failed, `solve` returning `Ok`. [`ChompOptimizer::get_collision_cost`]
    /// (`chomp_optimizer.cpp:942-963`) weights every point's collision
    /// potential by `collision_point_vel_mag`, that point's *velocity*
    /// along the trajectory -- CHOMP's obstacle cost is a swept cost (work
    /// done moving through a field), not a static-occupancy cost. A
    /// perfectly stationary trajectory has zero velocity everywhere, so
    /// `c_cost` is exactly `0.0` regardless of penetration depth, which is
    /// always `< collision_threshold`; `optimize()`'s collision-threshold
    /// branch (this file's `d10b014` deviation note on `optimize()`) then
    /// forces `is_collision_free = true` on that same pass, unconditionally
    /// overwriting whatever [`ChompOptimizer::perform_forward_kinematics`]'s
    /// own per-point check had just found. So the goal must actually be
    /// *approached through* the obstacle, not merely coincide with it.
    ///
    /// This test instead starts at `j1 == -0.8` and targets `j1 == 0.8`
    /// (`j2` fixed at `0.0` throughout), sweeping `tip` directly through
    /// `(0.6, 0, 0)` -- `tip`'s own position at `j1 == 0.0` (two `0.3 0 0`
    /// joint origins from `base`, see `two_joint_chain_model`'s doc
    /// comment) -- where the obstacle point sits. With `max_iterations: 1`
    /// and the default `trajectory_initialization_method`
    /// (`"quintic-spline"`), the single `perform_forward_kinematics` pass
    /// sees nonzero velocity at the obstacle crossing, so `c_cost` clears
    /// `collision_threshold` and the mask above does not fire;
    /// `is_collision_free` survives the pass as `false`.
    #[test]
    fn solve_returns_invalid_motion_plan_when_the_path_cannot_escape_collision() {
        let model = two_joint_chain_model();
        let mut start_state = RobotState::new(&model);
        start_state.set_variable_position("j1", -0.8).unwrap();
        let params = ChompParameters {
            max_iterations: 1,
            ..ChompParameters::default()
        };
        let mut cache = collision_cache(&model);
        let mut field = empty_env_field();
        // `tip`'s position at `j1 == 0.0`, directly on the swept path from
        // `j1 == -0.8` to `j1 == 0.8` -- see this test's doc comment.
        field.add_points_to_field(&[Vector3::new(0.6, 0.0, 0.0)]);
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let mut rng = ChaCha8Rng::seed_from_u64(3);
        let goal = ChompGoal {
            joint_constraints: vec![joint_goal("j1", 0.8), joint_goal("j2", 0.0)],
        };
        let request = ChompRequest {
            start_state: &start_state,
            group_name: GROUP,
            goal_constraints: std::slice::from_ref(&goal),
            params: &params,
            seed_trajectory: None,
        };

        let result = solve(&request, &mut collision, None, &mut |_, _| false, &mut rng);

        assert_code(&result, MoveItErrorCode::InvalidMotionPlan);
    }

    /// [`solve_with_trace`]'s whole reason to exist: the [`ChompLoopTrace`]
    /// must survive past an `Err(InvalidMotionPlan)`, not just past a
    /// success. Same fixture as the test just above
    /// (`solve_returns_invalid_motion_plan_when_the_path_cannot_escape_collision`,
    /// `max_iterations: 1`, no mesh/threshold break fires -- see that
    /// test's doc comment for why) so the loop's only way out is the `for`
    /// condition going false, i.e. [`ChompExit::IterationBound`].
    #[test]
    fn solve_with_trace_reports_iteration_bound_on_a_failed_plan() {
        let model = two_joint_chain_model();
        let mut start_state = RobotState::new(&model);
        start_state.set_variable_position("j1", -0.8).unwrap();
        let params = ChompParameters {
            max_iterations: 1,
            ..ChompParameters::default()
        };
        let mut cache = collision_cache(&model);
        let mut field = empty_env_field();
        field.add_points_to_field(&[Vector3::new(0.6, 0.0, 0.0)]);
        let mut collision = ChompCollisionContext {
            cache: &mut cache,
            env_distance_field: &field,
        };
        let mut rng = ChaCha8Rng::seed_from_u64(3);
        let goal = ChompGoal {
            joint_constraints: vec![joint_goal("j1", 0.8), joint_goal("j2", 0.0)],
        };
        let request = ChompRequest {
            start_state: &start_state,
            group_name: GROUP,
            goal_constraints: std::slice::from_ref(&goal),
            params: &params,
            seed_trajectory: None,
        };

        let (result, trace) =
            solve_with_trace(&request, &mut collision, None, &mut |_, _| false, &mut rng);

        assert_code(&result, MoveItErrorCode::InvalidMotionPlan);
        let trace = trace.expect("an attempt whose optimizer ran a loop must report a trace");
        assert_eq!(trace.exit, ChompExit::IterationBound);
        assert_eq!(
            trace.evaluations, 1,
            "max_iterations: 1 means the loop's only pass evaluates once"
        );
    }

    #[test]
    fn build_seed_trajectory_applies_shortest_angular_distance_to_a_continuous_joint_goal() {
        let model = two_joint_chain_model();
        let mut start_state = RobotState::new(&model);
        start_state.set_variable_position("j2", 3.0).unwrap();
        // Naively, the goal would land at exactly -3.0 rad. The shortest
        // path from 3.0 rad wraps the other way around the circle:
        // transcribed from `shortestAngularDistance` (`chomp_utils.cpp`),
        // the adjusted goal is `3.0 + shortest_angular_distance(3.0, -3.0)`
        // `== 3.0 + (2*PI - 6.0) == 3.283185307179586...`, not `-3.0`.
        let goal = ChompGoal {
            joint_constraints: vec![joint_goal("j1", 0.0), joint_goal("j2", -3.0)],
        };

        let trajectory =
            build_seed_trajectory(&start_state, GROUP, std::slice::from_ref(&goal)).unwrap();

        let goal_index = trajectory.num_points() - 1;
        let goal_row = trajectory.trajectory_point(goal_index);
        let j2_column = model
            .joint_model_group(GROUP)
            .unwrap()
            .active_joint_indices()
            .iter()
            .position(|&idx| model.joint_model_at(idx).name() == "j2")
            .expect("j2 is an active joint of the group");
        assert_relative_eq!(
            goal_row[j2_column],
            3.0 + shortest_angular_distance(3.0, -3.0),
            epsilon = 1e-12
        );
        assert_relative_eq!(
            3.0 + shortest_angular_distance(3.0, -3.0),
            3.283_185_307_179_586,
            epsilon = 1e-12
        );
        // `j1` is not continuous, so its goal row is the raw requested
        // value, untouched by the wrap-around fix.
        let j1_column = model
            .joint_model_group(GROUP)
            .unwrap()
            .active_joint_indices()
            .iter()
            .position(|&idx| model.joint_model_at(idx).name() == "j1")
            .expect("j1 is an active joint of the group");
        assert_relative_eq!(goal_row[j1_column], 0.0, epsilon = 1e-12);
    }

    #[test]
    fn validate_recovery_time_limit_accepts_a_typical_value() {
        assert_eq!(validate_recovery_time_limit(6.0).unwrap(), 11);
    }

    #[test]
    fn validate_recovery_time_limit_rejects_a_value_whose_plus_five_overflows_i32() {
        // `f64::from(i32::MAX) - 4.0` puts `planning_time_limit + 5` one ULP
        // past `i32::MAX`: upstream's uncast `static_cast<int>` at this
        // boundary is undefined behaviour, and a naive
        // `planning_time_limit as i32 + 5` transcription would saturate the
        // cast to `i32::MAX` and then overflow the following `+ 5` in `i32`
        // space. This must be a typed error, not a panic or a wrapped value.
        let planning_time_limit = f64::from(i32::MAX) - 4.0;
        assert!(validate_recovery_time_limit(planning_time_limit).is_err());
    }

    #[test]
    fn validate_recovery_time_limit_accepts_the_exact_i32_max_boundary() {
        let planning_time_limit = f64::from(i32::MAX) - 5.0;
        assert_eq!(
            validate_recovery_time_limit(planning_time_limit).unwrap(),
            i32::MAX
        );
    }

    #[test]
    fn validate_recovery_time_limit_accepts_the_exact_i32_min_boundary() {
        let planning_time_limit = f64::from(i32::MIN) - 5.0;
        assert_eq!(
            validate_recovery_time_limit(planning_time_limit).unwrap(),
            i32::MIN
        );
    }

    #[test]
    fn validate_recovery_time_limit_rejects_a_value_whose_plus_five_underflows_i32() {
        let planning_time_limit = f64::from(i32::MIN) - 6.0;
        assert!(validate_recovery_time_limit(planning_time_limit).is_err());
    }

    #[test]
    fn validate_recovery_time_limit_rejects_nan() {
        assert!(validate_recovery_time_limit(f64::NAN).is_err());
    }

    #[test]
    fn validate_recovery_time_limit_rejects_infinity() {
        assert!(validate_recovery_time_limit(f64::INFINITY).is_err());
        assert!(validate_recovery_time_limit(f64::NEG_INFINITY).is_err());
    }
}
