// Copyright (c) 2023, PickNik Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/stomp/src/stomp_moveit_planning_context.cpp

//! `solveWithStomp` and the STOMP-specific core of
//! `StompPlanningContext::solve` -- the actual wiring point: this module is
//! where `moveit-planners-stomp` calls `moveit_stomp_core::Stomp`.
//!
//! # What's ported, and what stays in the ROS/task-engine layer (D1/D2)
//!
//! `stomp_moveit_planning_context.cpp` mixes STOMP-specific computation
//! with `planning_interface`/`rclcpp` glue. This module ports the former;
//! `lib.rs`'s "Not ported" section already excludes the latter as a whole
//! crate-level decision, narrowed here to the specific pieces this file
//! actually skips:
//!
//! - `extractSeedTrajectory` (cpp:94-144): builds a seed
//!   [`RobotTrajectory`] from a `MotionPlanRequest`'s `trajectory_constraints`
//!   (ROS message walking, no STOMP-specific computation). [`plan`]'s
//!   caller passes `input_trajectory: Option<&RobotTrajectory>` directly
//!   instead.
//! - The goal-state constraint sampler (cpp:224-234,
//!   `constraint_samplers::ConstraintSamplerManager`) and the
//!   `allowed_planning_time` timeout watcher thread (cpp:247-257): both
//!   ROS-request-specific. A caller that wants mid-solve cancellation still
//!   has [`Stomp::cancel_handle`], obtained before calling [`plan`] (which
//!   owns construction of the `Stomp` this round, so a handle cannot yet be
//!   obtained *during* a `plan` call -- see this module's own "UNFIXED"
//!   note below).
//! - `visualization::getIterationPathPublisher`/`getSuccessTrajectoryPublisher`
//!   (cpp:179-182): `rclcpp::Publisher`-backed, ROS-only.
//!
//! # UNFIXED: no timeout/cancellation wiring inside `plan`
//!
//! Upstream's `solve` starts an async watcher thread that calls
//! `stomp->cancel()` after `req.allowed_planning_time` elapses
//! (cpp:247-257) -- directly backed by [`Stomp::cancel_handle`] in this
//! port (built round 22 for exactly this). [`plan`] does not wire this: it
//! constructs and owns the `Stomp` for the duration of one synchronous
//! call, so there is no point before `solve_with_stomp` runs at which a
//! caller could obtain a handle to race against it. Deferred to whatever
//! future round gives this crate a persistent `PlanningContext`-shaped
//! object (a `moveit-planners-sbp`-style registry entry) that can expose
//! `cancel_handle()` to a caller before `solve` is invoked, the same shape
//! upstream's own `StompPlanningContext` has via its `stomp_` member.

use rand::Rng;

use moveit_error::Result;
use moveit_model::JointModelGroup;
use moveit_state::RobotState;
use moveit_stomp_core::{Stomp, StompConfiguration, TrajectoryInitialization};
use moveit_trajectory::RobotTrajectory;

use crate::composable_task::{ComposableTask, CostFn};
use crate::conversion_functions::{
    UnparameterizedTrajectory, matrix_to_robot_trajectory, positions, robot_trajectory_to_matrix,
};
use crate::filter_functions;
use crate::noise_generators::normal_distribution_generator;

/// Upstream's own hardcoded per-joint noise standard deviation
/// (`stomp_moveit_planning_context.cpp`: `const std::vector<double>
/// stddev(group->getActiveJointModels().size(), 0.1);`, annotated
/// `TODO(henningkayser): parameterize stddev`). Not this port's invention
/// -- the literal upstream never made configurable.
pub const DEFAULT_NOISE_STDDEV: f64 = 0.1;

/// `solveWithStomp` (cpp:67-91): runs `stomp` seeded from
/// `input_trajectory`'s waypoints if one is given and non-empty
/// (`!input_trajectory || input_trajectory->empty()`, cpp:75), otherwise
/// from `start_state`/`goal_state`'s endpoints -- the exact branch upstream
/// takes.
///
/// `Ok(None)` where upstream returns `false` (`success`, cpp:74): STOMP
/// simply did not find a valid solution within its configured iteration
/// budget, an expected outcome rather than a port-level error -- the same
/// "not found, not broken" shape as [`moveit_sampling::MultivariateGaussian::new`]'s
/// `None`. `Err` is reserved for a genuine precondition violation (see
/// [`positions`]/[`crate::conversion_functions::fill_robot_trajectory`]'s
/// "Single-variable-joint precondition").
pub fn solve_with_stomp<'m>(
    stomp: &mut Stomp<'_>,
    start_state: &RobotState<'m>,
    goal_state: &RobotState<'m>,
    group: &'m JointModelGroup,
    input_trajectory: Option<&RobotTrajectory<'m>>,
) -> Result<Option<UnparameterizedTrajectory<'m>>> {
    let seed = input_trajectory.filter(|trajectory| trajectory.way_point_count() > 0);
    let (success, waypoints) = match seed {
        Some(trajectory) => {
            let input = robot_trajectory_to_matrix(trajectory, group)?;
            stomp.solve(&input)
        }
        None => {
            let start = positions(start_state, group)?;
            let goal = positions(goal_state, group)?;
            stomp.solve_from_endpoints(start.as_slice(), goal.as_slice())
        }
    };

    if !success {
        return Ok(None);
    }
    Ok(Some(matrix_to_robot_trajectory(
        &waypoints,
        start_state,
        group,
    )?))
}

/// `getStompConfig` + `createStompTask` + `Stomp::new` +
/// [`solve_with_stomp`] -- `StompPlanningContext::solve`'s STOMP-specific
/// core (cpp:236-245, cpp:260). See this module's own doc for what's
/// deliberately left out (D1/D2, the ROS/task-engine layer).
///
/// # Construction order (cited: `stomp_moveit_planning_context.cpp:236-245`)
///
/// 1. `getStompConfig` (cpp:191-207): `config.num_dimensions` from the
///    group's active joint count, `config.initialization_method` hardcoded
///    to `LinearInterpolation` (upstream's own hardcoding, annotated
///    `TODO(henningkayser): set from request or params`) -- every other
///    field of `config` is the caller's, unchanged. [`plan`] performs
///    exactly these two overrides on the `config` it is given.
/// 2. If `input_trajectory` is given and non-empty, override
///    `config.num_timesteps` from its waypoint count (cpp:240-243).
/// 3. `createStompTask` (cpp:147-188): `noise_generator_fn` via
///    [`normal_distribution_generator`] with `stddev = vec![DEFAULT_NOISE_STDDEV;
///    num_dimensions]` (cpp:175-176), `filter_fn` via
///    `filter_functions::chain([simple_smoothing_matrix, enforce_position_bounds])`
///    (cpp:177-178), `cost_fn` -- see "Cost function is caller-supplied"
///    below. `iteration_callback_fn`/`done_callback_fn` (cpp:179-182,
///    ROS-only visualization) become no-op closures.
/// 4. `Stomp::new(config, task)` (cpp:245).
/// 5. [`solve_with_stomp`] (cpp:260).
///
/// # Cost function is caller-supplied, not built here
///
/// Upstream's `createStompTask` builds `cost_fn` from
/// `costs::getCollisionCostFunction`/`costs::getConstraintsCostFunction`
/// (cpp:162-172), both `PlanningScene`-backed factories out of this crate's
/// dependency reach (see `cost_functions`' own module doc). [`plan`] takes
/// a ready-made [`CostFn`] instead of building one: a caller with access to
/// `moveit-scene`/`moveit-collision` builds one via
/// `cost_functions::cost_function_from_state_validator` over their own
/// [`crate::cost_functions::StateValidatorFn`], composing the same way
/// upstream's `createStompTask` does, just with the `PlanningScene` wiring
/// left to the caller instead of hardcoded into this function.
pub fn plan<'m>(
    mut config: StompConfiguration,
    cost_fn: CostFn<'m>,
    start_state: &RobotState<'m>,
    goal_state: &RobotState<'m>,
    group: &'m JointModelGroup,
    input_trajectory: Option<&RobotTrajectory<'m>>,
    rng: impl Rng + 'm,
) -> Result<Option<UnparameterizedTrajectory<'m>>> {
    config.num_dimensions = group.active_joint_names().len();
    config.initialization_method = TrajectoryInitialization::LinearInterpolation;
    if let Some(trajectory) = input_trajectory {
        if trajectory.way_point_count() > 0 {
            config.num_timesteps = trajectory.way_point_count();
        }
    }

    let stddev = vec![DEFAULT_NOISE_STDDEV; config.num_dimensions];
    let noise_generator_fn = normal_distribution_generator(config.num_timesteps, stddev, rng)?;
    let filter_fn = filter_functions::chain(vec![
        filter_functions::simple_smoothing_matrix(config.num_timesteps)?,
        filter_functions::enforce_position_bounds(start_state.model(), group)?,
    ]);
    let task = ComposableTask::new(
        noise_generator_fn,
        cost_fn,
        filter_fn,
        Box::new(|_iteration, _cost, _parameters| {}),
        Box::new(|_success, _total_iterations, _final_cost, _parameters| {}),
    );

    let mut stomp = Stomp::new(config, Box::new(task));
    solve_with_stomp(&mut stomp, start_state, goal_state, group, input_trajectory)
}

#[cfg(test)]
mod tests {
    use super::*;
    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_srdf::SrdfModel;
    use moveit_stomp_core::TrajectoryInitialization as Init;
    use nalgebra::{DMatrix, DVector};
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;
    use std::fs;

    fn fixture_path(file_name: &str) -> String {
        format!(
            concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
            file_name
        )
    }

    fn panda_model() -> RobotModel {
        let urdf_path = fixture_path("panda.urdf");
        let srdf_path = fixture_path("panda.srdf");
        let urdf_xml =
            fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
        let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("fixture model must build")
    }

    fn base_config(num_timesteps: usize, num_dimensions: usize) -> StompConfiguration {
        StompConfiguration {
            num_iterations: 30,
            num_iterations_after_valid: 0,
            num_timesteps,
            num_dimensions,
            delta_t: 0.1,
            initialization_method: Init::LinearInterpolation,
            exponentiated_cost_sensitivity: 0.5,
            num_rollouts: 15,
            max_rollouts: 15,
            control_cost_weight: 0.0,
        }
    }

    fn no_cost_fn() -> CostFn<'static> {
        Box::new(|values: &DMatrix<f64>| Some((DVector::zeros(values.ncols()), true)))
    }

    #[test]
    fn solve_with_stomp_finds_no_seed_uses_endpoints_and_matches_group_dimensions() {
        let model = panda_model();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut start = RobotState::new(&model);
        start.set_to_default_values();
        let mut goal = start.clone();
        goal.set_joint_positions("panda_joint1", &[0.3]).unwrap();

        let num_timesteps = 10;
        let config = base_config(num_timesteps, group.active_joint_names().len());
        let task = ComposableTask::new(
            normal_distribution_generator(
                num_timesteps,
                vec![DEFAULT_NOISE_STDDEV; group.active_joint_names().len()],
                ChaCha8Rng::seed_from_u64(1),
            )
            .unwrap(),
            no_cost_fn(),
            filter_functions::no_filter(),
            Box::new(|_, _, _| {}),
            Box::new(|_, _, _, _| {}),
        );
        let mut stomp = Stomp::new(config, Box::new(task));

        let result = solve_with_stomp(&mut stomp, &start, &goal, group, None)
            .expect("panda_arm move must not hit the single-variable-joint precondition")
            .expect("a zero-cost task must always report success");
        assert_eq!(result.way_point_count(), num_timesteps);
    }

    #[test]
    fn plan_end_to_end_produces_a_trajectory_from_endpoints() {
        let model = panda_model();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut start = RobotState::new(&model);
        start.set_to_default_values();
        let mut goal = start.clone();
        goal.set_joint_positions("panda_joint1", &[0.3]).unwrap();

        let num_timesteps = 10;
        let config = base_config(num_timesteps, group.active_joint_names().len());
        let result = plan(
            config,
            no_cost_fn(),
            &start,
            &goal,
            group,
            None,
            ChaCha8Rng::seed_from_u64(2),
        )
        .unwrap()
        .expect("a zero-cost task must always report success");

        assert_eq!(result.way_point_count(), num_timesteps);
        let trajectory = result.into_uniformly_timed(config.delta_t).unwrap();
        assert_eq!(trajectory.way_point_duration_from_previous(0), 0.0);
        assert_eq!(
            trajectory.way_point_duration_from_previous(1),
            config.delta_t
        );
    }

    #[test]
    fn plan_overrides_num_timesteps_from_a_nonempty_seed_trajectory() {
        let model = panda_model();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut start = RobotState::new(&model);
        start.set_to_default_values();
        let mut goal = start.clone();
        goal.set_joint_positions("panda_joint1", &[0.3]).unwrap();

        let seed_timesteps = 7;
        let seed_values = DMatrix::zeros(group.active_joint_names().len(), seed_timesteps);
        let seed = matrix_to_robot_trajectory(&seed_values, &start, group)
            .unwrap()
            .into_uniformly_timed(0.1)
            .unwrap();

        // config's own num_timesteps (5) must be overridden by the seed's
        // waypoint count (7), matching cpp:240-243.
        let config = base_config(5, group.active_joint_names().len());
        let result = plan(
            config,
            no_cost_fn(),
            &start,
            &goal,
            group,
            Some(&seed),
            ChaCha8Rng::seed_from_u64(3),
        )
        .unwrap()
        .expect("a zero-cost task must always report success");

        assert_eq!(result.way_point_count(), seed_timesteps);
    }

    /// The proof item 3 of round 23 asked for: not "the optimizer was
    /// called", but "in a scene with an obstacle, the resulting trajectory
    /// has lower cost than the initial trajectory". The obstacle is a
    /// forbidden band in `panda_joint1`'s own value -- the group's active
    /// joint-position matrix is exactly [`crate::cost_functions::StateValidatorFn`]'s
    /// input space, so this needs no Cartesian collision geometry to make
    /// the point.
    #[test]
    fn plan_finds_a_lower_cost_trajectory_than_the_initial_straight_line_through_an_obstacle() {
        let model = panda_model();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut start = RobotState::new(&model);
        start.set_to_default_values();
        let mut goal = start.clone();
        goal.set_joint_positions("panda_joint1", &[0.6]).unwrap();

        let num_timesteps = 15;
        // Straight-line joint1 path from 0.0 to 0.6 crosses 0.3 around the
        // middle of the trajectory -- this band sits squarely across it.
        let obstacle_center = 0.3;
        let obstacle_radius = 0.15;
        let obstacle_penalty = 10.0;
        let make_validator = move || -> crate::cost_functions::StateValidatorFn<'static> {
            Box::new(move |state: &DVector<f64>| {
                if (state[0] - obstacle_center).abs() < obstacle_radius {
                    obstacle_penalty
                } else {
                    0.0
                }
            })
        };

        let mut config = base_config(num_timesteps, group.active_joint_names().len());
        config.num_iterations = 100;
        config.num_iterations_after_valid = 5;
        config.num_rollouts = 30;
        config.max_rollouts = 30;
        config.control_cost_weight = 0.0;

        // Constructed the same way `plan` builds it internally
        // (`normal_distribution_generator` + `chain([simple_smoothing_matrix,
        // enforce_position_bounds])` + `ComposableTask` + `Stomp::new`) --
        // built directly here, rather than through [`plan`], because [`plan`]
        // (matching upstream's own `solveWithStomp`, cpp:84-90) discards the
        // optimized matrix entirely whenever the *final* trajectory is not
        // fully valid, and this test's claim is the weaker, task-mandated
        // one -- lower cost, not full validity -- so it needs the matrix
        // regardless of `stomp`'s own success flag.
        let n = group.active_joint_names().len();
        let noise_generator_fn = normal_distribution_generator(
            num_timesteps,
            vec![DEFAULT_NOISE_STDDEV; n],
            ChaCha8Rng::seed_from_u64(123),
        )
        .unwrap();
        let filter_fn = filter_functions::chain(vec![
            filter_functions::simple_smoothing_matrix(num_timesteps).unwrap(),
            filter_functions::enforce_position_bounds(&model, group).unwrap(),
        ]);
        let plan_cost_fn =
            crate::cost_functions::cost_function_from_state_validator(make_validator(), 0.0);
        let task = ComposableTask::new(
            noise_generator_fn,
            plan_cost_fn,
            filter_fn,
            Box::new(|_, _, _| {}),
            Box::new(|_, _, _, _| {}),
        );
        let mut stomp = Stomp::new(config, Box::new(task));

        let start_positions = positions(&start, group).unwrap();
        let goal_positions = positions(&goal, group).unwrap();
        let (_success, optimized_matrix) =
            stomp.solve_from_endpoints(start_positions.as_slice(), goal_positions.as_slice());

        // The initial trajectory `stomp` itself starts from -- linear
        // interpolation between start and goal, matching
        // `config.initialization_method` (`LinearInterpolation`, set above).
        let mut initial_matrix = DMatrix::zeros(n, num_timesteps);
        for i in 0..n {
            let dtheta = (goal_positions[i] - start_positions[i]) / (num_timesteps as f64 - 1.0);
            for t in 0..num_timesteps {
                initial_matrix[(i, t)] = start_positions[i] + t as f64 * dtheta;
            }
        }

        let mut eval_cost_fn =
            crate::cost_functions::cost_function_from_state_validator(make_validator(), 0.0);
        let (initial_costs, initial_valid) = eval_cost_fn(&initial_matrix).unwrap();
        let (optimized_costs, _optimized_valid) = eval_cost_fn(&optimized_matrix).unwrap();

        assert!(
            !initial_valid,
            "test setup must make the initial straight-line trajectory cross the obstacle"
        );
        let initial_cost: f64 = initial_costs.sum();
        let optimized_cost: f64 = optimized_costs.sum();
        assert!(
            optimized_cost < initial_cost,
            "optimized cost {optimized_cost} must be lower than the initial trajectory's cost \
             {initial_cost}"
        );
    }
}
