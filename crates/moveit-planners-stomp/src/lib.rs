// Copyright (c) 2020, PickNik Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/stomp/

//! STOMP planner support types, ported from `moveit_planners/stomp/`'s
//! ROS-independent headers: matrix<->trajectory conversion
//! ([`conversion_functions`]), trajectory filters ([`filter_functions`]),
//! a composable [`moveit_stomp_core::Task`] ([`composable_task`]), cost
//! functions ([`cost_functions`]), noise generation ([`noise_generators`]),
//! and the planner entry point itself ([`planner`]) that wires all of the
//! above into a call to [`moveit_stomp_core::Stomp`].
//!
//! # `stomp`, the optimizer core: ported, but into `moveit-stomp-core`, not here
//!
//! STOMP's actual optimization loop lives in the separate upstream
//! repository `ros-industrial/stomp`, not in `moveit2` -- see
//! `moveit-stomp-core`'s own module doc for how that source was obtained
//! and verified, and for the one-crate-one-upstream reasoning behind
//! keeping it in its own crate rather than here. This crate depends on it:
//! `filter_functions::simple_smoothing_matrix` calls
//! `moveit_stomp_core::generate_smoothing_matrix`,
//! `noise_generators::normal_distribution_generator` calls
//! `moveit_stomp_core::generate_finite_difference_matrix`/
//! `full_piv_lu_try_inverse_or_empty`, and [`planner::plan`] constructs and
//! drives a `moveit_stomp_core::Stomp` directly.
//!
//! # Round 23: `cost_functions.hpp`/`noise_generators.hpp`/`stomp_moveit_task.hpp`, the generic halves
//!
//! An earlier round deferred all three; this round ports the parts of each
//! that do not need a `PlanningScene`:
//!
//! - [`cost_functions::cost_function_from_state_validator`]/[`cost_functions::sum`]
//!   (`costs::getCostFunctionFromStateValidator`/`costs::sum`) -- generic
//!   over a caller-supplied [`cost_functions::StateValidatorFn`].
//! - [`noise_generators::normal_distribution_generator`]
//!   (`noise::getNormalDistributionGenerator`) -- no `PlanningScene`
//!   dependency at all, fully ported.
//! - [`composable_task::ComposableTask`] (`stomp_moveit::ComposableTask`
//!   from `stomp_moveit_task.hpp`) -- the `Task` implementation itself.
//!   `NoiseGeneratorFn`/`CostFn`/`PostIterationFn`/`DoneFn` join `FilterFn`
//!   (already carried in from an earlier round, see `filter_functions`'
//!   module doc) as the pieces of that header this crate needs.
//!
//! # Round 24: `costs::getCollisionCostFunction`/`costs::getConstraintsCostFunction`, the `PlanningScene`-backed half
//!
//! Round 23 left these two factories unported, reasoning that
//! `moveit-scene`/`moveit-collision` were out of this crate's dependency
//! reach. That was checked and found false: neither was actually blocked by
//! any rule, just not yet listed in `Cargo.toml`, and the sibling planner
//! crate `moveit-planners-sbp` already depends on both without a cycle
//! (`cargo tree -p moveit-scene -e normal`/`cargo tree -p moveit-collision
//! -e normal` neither lists `stomp`). Both are now `[workspace.dependencies]`
//! entries here too, and [`cost_functions::get_collision_cost_function`]/
//! [`cost_functions::get_constraints_cost_function`] are ported -- see that
//! module's own doc for the `RefCell<&mut PlanningScene>` bridge pattern
//! (reused from `moveit-planners-sbp::planning_scene_validity`'s existing
//! precedent) and the two documented deviations from upstream.
//!
//! # Round 24: cancellation lifted to the caller, and `PlanRequest`
//!
//! [`planner::plan`] now takes a [`moveit_stomp_core::CancelHandle`]
//! parameter instead of building and discarding one internally -- see
//! [`planner`]'s own "Round 24: cancellation, lifted to the caller" for the
//! full reasoning and why `PlanningContext` is deliberately not introduced
//! here. Adding that parameter pushed [`planner::plan`] to eight arguments;
//! [`planner::PlanRequest`] bundles the four that form one motion query
//! (`start_state`/`goal_state`/`group`/`input_trajectory`) back down to a
//! single parameter, a plain data grouping with no behavior of its own, not
//! a step toward that same trait.
//!
//! # Round 24: the `into_uniformly_timed` invariant now carries its own test
//!
//! Round 23 asserted that [`conversion_functions::UnparameterizedTrajectory`]
//! rules out two sides silently time-parameterizing the same trajectory,
//! because the only path to a real `RobotTrajectory` is
//! `into_uniformly_timed`. That claim rested on prose alone. It now carries
//! a `compile_fail` doctest on `UnparameterizedTrajectory` itself,
//! demonstrating that the wrapped `RobotTrajectory` cannot be reached any
//! other way -- not by convention, but because the field is private and no
//! other public method exposes it.
//!
//! # Not ported: the ROS/task-engine layer (D1/D2 exclusion)
//!
//! `stomp_moveit_planning_context.{hpp,cpp}`'s ROS-specific pieces (goal
//! constraint sampling, seed-trajectory extraction from a `MotionPlanRequest`,
//! the `allowed_planning_time` timeout watcher thread, pluginlib
//! registration) and `trajectory_visualization.hpp` are not ported -- see
//! [`planner`]'s own module doc for exactly which lines of
//! `stomp_moveit_planning_context.cpp` this crate's [`planner::plan`] ports
//! and which it leaves out. Per PORTING-PLAN.md's D1 ("final form: a
//! ROS-independent Rust motion-planning library") and D2 ("ROS 2 bindings
//! isolated to an optional `moveit-ros` crate"), this crate carries only
//! the ROS-independent computational core; the
//! `planning_interface::PlanningContext` plugin glue and the `rclcpp`-visible
//! visualization types belong to a ROS integration layer this workspace
//! does not port into `moveit-planners-stomp` itself.
//!
//! # Round 23: reconciled with `moveit-planning`'s `AddTimeOptimalParameterization`
//!
//! `moveit-planning::response_adapters::AddTimeOptimalParameterization`'s
//! own module doc claims to "close" a `fill_robot_trajectory` placeholder-`dt`
//! gap it describes as "every waypoint after the first gets a placeholder
//! `dt = 0.1`". That description is the *round-20* shape of
//! `fill_robot_trajectory` -- round 21's "Deviation:
//! unparameterized-by-construction" (see `conversion_functions`' own module
//! doc) replaced it: `fill_robot_trajectory`/`matrix_to_robot_trajectory`
//! return [`conversion_functions::UnparameterizedTrajectory`], which sets
//! every waypoint's duration to an inert `0.0` and exposes no duration
//! accessor at all. There is no `0.1` placeholder anywhere in this crate's
//! output for that adapter's test to have been reproducing since round 21;
//! its doc is stale, though its actual behavior (re-time whatever
//! `RobotTrajectory` it is given) is unaffected by that staleness. This
//! crate does not depend on `moveit-planning` (nor vice versa -- confirmed
//! via that crate's own `response.rs` doc, "No `Option`", and its `lib.rs`),
//! so this is a documentation-only mismatch, not a compile-time or runtime
//! conflict; not fixed here since `moveit-planning` belongs to a different
//! round's worker.
//!
//! **Resolution**: [`planner::plan`] returns an
//! [`conversion_functions::UnparameterizedTrajectory`], never a
//! [`moveit_trajectory::RobotTrajectory`] directly. The only way to obtain
//! a real, timed `RobotTrajectory` -- the type
//! `moveit_planning::response::PlanningResponse::trajectory` actually
//! requires, confirmed by reading that field's type directly rather than
//! assuming it -- is [`conversion_functions::UnparameterizedTrajectory::into_uniformly_timed`],
//! which forces the caller to name an explicit `dt`. A STOMP-backed
//! `PlanningContext::solve` (that trait is not introduced in this crate --
//! see [`planner`]'s own "Round 24: cancellation, lifted to the caller" for
//! why) would call `into_uniformly_timed(config.delta_t)`:
//! `delta_t` is STOMP's own optimization timestep, a physically meaningful
//! choice, not an arbitrary placeholder. `AddTimeOptimalParameterization`
//! running afterward, as a separate response-adapter-pipeline stage over
//! that now-real `RobotTrajectory`, is not "two sides silently fighting
//! over the same field" -- it is the standard MoveIt response-adapter
//! architecture (a planner produces an initial valid timing; a later
//! pipeline stage explicitly re-times it with real dynamics), and the two
//! writes are sequential, not concurrent: `plan`'s caller is the sole
//! writer of the *first*, uniformly-timed `RobotTrajectory`;
//! `AddTimeOptimalParameterization::adapt` is the sole writer of the
//! *second*, time-optimal overwrite. `UnparameterizedTrajectory`'s own type
//! (no duration accessor until `into_uniformly_timed` is called) is what
//! rules out a third possibility upstream's own C++ does not rule out at
//! all: a caller reading `fill_robot_trajectory`'s placeholder duration as
//! if it were real timing before any adapter has run.
//!
//! # `MultivariateGaussian`'s new home
//!
//! `multivariate_gaussian.hpp`'s class does not live in this crate.
//! `moveit_planners/chomp/chomp_motion_planner/`'s own
//! `MultivariateGaussian` (a future round's `moveit-planners-chomp`) is the
//! same algorithm in a separately maintained file, diffed directly against
//! this one this round -- see `moveit_sampling::multivariate_gaussian`'s
//! module doc for the full comparison. Rather than have one planner crate
//! depend on the other, both depend on the `moveit-sampling` crate instead.
//!
//! # `assert_relative_eq!` reckoning (§79 convention, applied from the start)
//!
//! ```text
//! perl tools/ci/count-relative-eq.pl crates/moveit-planners-stomp/src/*.rs
//! both=0 epsilon_only=0 max_relative_only=0 neither=0
//! ```
//!
//! Run for real against the tree as committed this round. Zero calls: this
//! crate's tests compare exact-integer-representable f64 values (bound
//! clamps, waypoint counts, matrix round-trips through the same arithmetic
//! on both sides) with `assert_eq!`/`assert_ne!`, never a tolerance-bearing
//! float comparison. Nothing to classify, nothing to bisect, no tolerance
//! floor to re-measure.

use moveit_error::{Error, Result};

pub mod composable_task;
pub mod conversion_functions;
pub mod cost_functions;
pub mod filter_functions;
pub mod noise_generators;
pub mod planner;

/// The precondition `conversion_functions` and `filter_functions` both
/// require: `name`'s joint must have exactly one variable. See
/// `conversion_functions`' module doc, "Single-variable-joint
/// precondition".
pub(crate) fn require_single_variable(name: &str, variable_count: usize) -> Result<()> {
    if variable_count != 1 {
        return Err(Error::Other(format!(
            "joint '{name}' has {variable_count} variables, but STOMP's matrix \
             representation requires every active joint in the group to have \
             exactly one variable"
        )));
    }
    Ok(())
}
