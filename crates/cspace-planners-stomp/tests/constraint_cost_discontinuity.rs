// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Measures the effect, on optimized-trajectory quality, of gating
//! [`get_constraints_cost_function`] on `satisfied` (unmerged commit
//! `7f561e20`, `fix(stomp): gate constraint cost on satisfied, not raw
//! distance`) instead of leaving it a continuous `distance * cost_scale`
//! (this branch's current, unfixed form).
//!
//! This test does not edit
//! `crates/cspace-planners-stomp/src/cost_functions.rs`. It uses
//! [`get_constraints_cost_function`] for the *gated* side -- `7f561e20`
//! landed the `if satisfied { 0.0 } else { distance * cost_scale }` gate
//! there -- and replicates upstream's continuous body
//! (`distance * cost_scale` unconditionally) as a standalone closure
//! ([`continuous_constraints_cost_function`]) built only from that module's
//! already-`pub` pieces ([`StateValidatorFn`],
//! [`cost_function_from_state_validator`], [`CONSTRAINT_CHECK_DISTANCE`])
//! plus [`cspace_constraints::KinematicConstraintSet::decide`] and
//! [`cspace_planners_stomp::conversion_functions::set_positions`] -- for the
//! *continuous* side. When this file was written those two sides were the
//! other way round; the merge that brought `7f561e20` into the same tree
//! swapped which one production provides, not what is being compared.
//!
//! # What is actually measured, and the mechanism behind it
//!
//! `7f561e20`'s own doc comment argues informally that the gate's
//! post-boundary slope still pulls a violating state back toward the
//! target -- the open question it leaves is what happens *inside* the
//! tolerance band. This test found something stronger than "reduced
//! precision inside the band": once every noisy rollout STOMP samples on a
//! given iteration lands inside tolerance, the gated cost function returns
//! the identical `0.0` for every single one of them, and the optimizer's
//! update stops moving *at all*, permanently, for as many further
//! iterations as it stays there -- not merely "no longer pulled toward the
//! exact center", but no net displacement to machine precision.
//!
//! Root cause, traced into `cspace_stomp_core::Stomp::compute_probabilities`
//! (`crates/cspace-stomp-core/src/stomp.rs`, out of this round's edit scope
//! but read for this trace): each rollout's softmax weight is `p =
//! importance_weight * exp(-h * (cost - min_cost) / denom)`, and `denom =
//! (max_cost - min_cost).max(MIN_COST_DIFFERENCE)` -- an existing,
//! already-correct division-by-zero guard for the all-tied-costs case (this
//! is itself the same defect family this round's chomp/stomp sweep is
//! auditing, already closed here). When every rollout's cost is tied at
//! `0.0` (gated, all inside tolerance), `max_cost - min_cost == 0.0`,
//! `denom` clamps to `MIN_COST_DIFFERENCE`, and every rollout's exponent is
//! `0.0` -- so every rollout gets *equal* probability, not zero, and
//! `update_parameters` folds in the probability-weighted mean of that
//! iteration's (zero-mean-by-construction, see
//! [`cspace_planners_stomp::noise_generators::normal_distribution_generator`])
//! noise samples, which is small enough after 1 iteration to already read
//! as bit-identical to the unperturbed mean in this test's own instrumented
//! run (checked directly, see this test's own history for the per-iteration
//! trace this doc summarizes). The continuous cost function never ties like
//! this -- `distance * cost_scale` is (almost surely, for real-valued noise)
//! a distinct value per rollout, so `compute_probabilities` always has a
//! real, non-degenerate gradient to work with, and the trajectory keeps
//! moving every iteration.
//!
//! The metric below is the mean absolute distance of the optimized
//! trajectory's constrained joint from the exact nominal target (`0.0`),
//! after a fixed iteration count, same RNG seed and configuration between
//! the two cost functions, starting exactly at the tolerance edge (`0.02`,
//! see [`TOLERANCE`]) with a noise scale ([`NOISE_STDDEV`]) small enough
//! that this "all rollouts tie" regime is reached quickly -- which is also
//! the realistic terminal regime for any STOMP run that has already
//! converged close to satisfying a constraint, not a contrived corner case.
//!
//! `num_iterations_after_valid` is set equal to `num_iterations` for both
//! runs to disable the early-valid-stop break (`Stomp::solve`'s own
//! `valid_iterations > num_iterations_after_valid` check) so both runs
//! execute the identical fixed number of iterations regardless of either
//! cost function's own validity signal -- `update_parameters` itself
//! applies every iteration's weighted update unconditionally whether or not
//! that iteration reports "valid" (`Stomp::run_single_iteration`/`solve`),
//! so this isolates the reweighting-quality question from the already-known
//! solve/reject-rate difference `7f561e20`'s own commit message measured
//! (0/125 -> 119/125).

use std::cell::RefCell;
use std::fs;

use cspace_constraints::{Constraint, JointConstraint, KinematicConstraintSet};
use cspace_core::model::{JointModelGroup, MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_planners_stomp::composable_task::{ComposableTask, CostFn};
use cspace_planners_stomp::conversion_functions::set_positions;
use cspace_planners_stomp::cost_functions::{
    CONSTRAINT_CHECK_DISTANCE, StateValidatorFn, cost_function_from_state_validator,
    get_constraints_cost_function,
};
use cspace_planners_stomp::filter_functions::no_filter;
use cspace_planners_stomp::noise_generators::normal_distribution_generator;
use cspace_scene::PlanningScene;
use cspace_stomp_core::{Stomp, StompConfiguration, TrajectoryInitialization};
use nalgebra::DMatrix;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
        file_name
    )
}

fn load_panda() -> (RobotModel, SrdfModel) {
    let urdf_path = fixture_path("panda.urdf");
    let srdf_path = fixture_path("panda.srdf");
    let urdf_xml =
        fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    let model = RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build");
    (model, srdf)
}

/// Upstream's own body (`cost_functions.hpp:246`,
/// `constraints.decide(state).distance * cost_scale` unconditionally),
/// built only from `cost_functions`'s public pieces instead of editing that
/// file. Mirrors [`get_constraints_cost_function`]'s own shape one-for-one,
/// including reusing [`CONSTRAINT_CHECK_DISTANCE`] as the interpolation step
/// size, so the only difference between this and the real
/// [`get_constraints_cost_function`] below is the `result.satisfied` gate
/// `7f561e20` added there.
///
/// This replica is the *continuous* arm, not the gated one: when this file
/// was written the production function was still upstream's continuous form
/// and the replica supplied the gate. `7f561e20` (residual-ci-wired) landed
/// the gate in production, so the two arms swapped sides at the merge --
/// the comparison this file measures is unchanged, but the side that has to
/// be replicated locally is now the one production no longer offers.
fn continuous_constraints_cost_function<'a, 'm>(
    scene: &'a RefCell<&'a mut PlanningScene<'m>>,
    group: &'a JointModelGroup,
    constraints: &'a KinematicConstraintSet,
    cost_scale: f64,
) -> CostFn<'a> {
    let validator: StateValidatorFn<'a> = Box::new(move |positions| {
        let mut scene = scene.borrow_mut();
        set_positions(positions, group, scene.current_state_mut()).expect(
            "panda_arm's joints are all single-variable -- checked by every sibling test in \
             this crate that already drives this group through this same call",
        );
        let posed = scene.current_state_mut().update();
        constraints.decide(&posed).distance * cost_scale
    });
    cost_function_from_state_validator(validator, CONSTRAINT_CHECK_DISTANCE)
}

const NUM_TIMESTEPS: usize = 6;
const NUM_ITERATIONS: usize = 25;
const NUM_ROLLOUTS: usize = 30;
const TOLERANCE: f64 = 0.02;
const NOISE_STDDEV: f64 = 0.03;
const COST_SCALE: f64 = 1.0;

fn config(num_dimensions: usize) -> StompConfiguration {
    StompConfiguration {
        num_iterations: NUM_ITERATIONS,
        // Equal to num_iterations: disables the early-valid-stop break (see
        // this file's module doc) so both variants run the identical fixed
        // iteration count regardless of either cost function's own
        // validity signal.
        num_iterations_after_valid: NUM_ITERATIONS,
        num_timesteps: NUM_TIMESTEPS,
        num_dimensions,
        delta_t: 0.1,
        initialization_method: TrajectoryInitialization::LinearInterpolation,
        exponentiated_cost_sensitivity: 0.5,
        num_rollouts: NUM_ROLLOUTS,
        max_rollouts: NUM_ROLLOUTS,
        control_cost_weight: 0.0,
    }
}

/// Runs one STOMP solve with `cost_fn` and returns the mean absolute value
/// of `panda_joint1`'s (row 0 -- the only joint the constraint scores)
/// optimized trajectory: the distance from the exact nominal target (0.0)
/// this run's cost function actually steered the trajectory to.
fn run(seed: u64, cost_fn: CostFn<'_>, num_dimensions: usize) -> f64 {
    let noise_generator = normal_distribution_generator(
        NUM_TIMESTEPS,
        vec![NOISE_STDDEV; num_dimensions],
        ChaCha8Rng::seed_from_u64(seed),
    )
    .unwrap();
    let task = ComposableTask::new(
        noise_generator,
        cost_fn,
        no_filter(),
        Box::new(|_, _, _| {}),
        Box::new(|_, _, _, _| {}),
    );
    let mut stomp = Stomp::new(config(num_dimensions), Box::new(task));

    // Start and end both exactly at the tolerance boundary on panda_joint1
    // (the other 6 joints are unconstrained and irrelevant to the metric):
    // every waypoint of the seed trajectory sits precisely at the edge the
    // discontinuity is about.
    let mut endpoint = vec![0.0; num_dimensions];
    endpoint[0] = TOLERANCE;
    let (_valid, optimized) = stomp.solve_from_endpoints(&endpoint, &endpoint);

    optimized.row(0).iter().map(|v| v.abs()).sum::<f64>() / NUM_TIMESTEPS as f64
}

#[test]
fn gated_constraint_cost_settles_farther_from_the_exact_target_than_continuous() {
    let (model, srdf) = load_panda();
    let group = model.joint_model_group("panda_arm").unwrap();
    let num_dimensions = group.active_joint_names().len();
    let constraint =
        JointConstraint::new(&model, "panda_joint1", 0.0, TOLERANCE, TOLERANCE, 1.0).unwrap();
    let mut set = KinematicConstraintSet::new();
    set.push(Constraint::Joint(constraint));

    // Multiple independent seeds, averaged: a single seed's 30-rollout,
    // 25-iteration run is not proof against sampling noise on its own.
    const SEEDS: [u64; 8] = [1, 2, 3, 4, 5, 6, 7, 8];

    let mut continuous_total = 0.0;
    let mut gated_total = 0.0;
    let mut continuous_wins = 0;
    for &seed in &SEEDS {
        let mut scene = PlanningScene::new(&model, &srdf);
        let cell = RefCell::new(&mut scene);
        let continuous_cost_fn =
            continuous_constraints_cost_function(&cell, group, &set, COST_SCALE);
        let continuous_metric = run(seed, continuous_cost_fn, num_dimensions);

        let mut scene = PlanningScene::new(&model, &srdf);
        let cell = RefCell::new(&mut scene);
        let gated_cost_fn = get_constraints_cost_function(&cell, group, &set, COST_SCALE).unwrap();
        let gated_metric = run(seed, gated_cost_fn, num_dimensions);

        println!(
            "seed {seed}: continuous mean|dist|={continuous_metric:.6}  \
             gated mean|dist|={gated_metric:.6}"
        );
        if continuous_metric <= gated_metric {
            continuous_wins += 1;
        }
        continuous_total += continuous_metric;
        gated_total += gated_metric;
    }

    let continuous_mean = continuous_total / SEEDS.len() as f64;
    let gated_mean = gated_total / SEEDS.len() as f64;
    println!(
        "aggregate over {} seeds: continuous mean|dist|={continuous_mean:.6}  \
         gated mean|dist|={gated_mean:.6}  continuous closer-or-tied in {continuous_wins}/{} seeds",
        SEEDS.len(),
        SEEDS.len()
    );

    // gated_mean is expected to sit at essentially exactly TOLERANCE (the
    // starting value): once every rollout ties at cost 0.0, the update is a
    // near-zero-mean average of that iteration's noise (see this file's
    // module doc, "Root cause") and the trajectory stops moving from its
    // starting position to within floating-point noise. continuous_mean is
    // expected to be measurably smaller, since its cost function never
    // ties and keeps pulling toward the exact target every iteration.
    assert!(
        continuous_mean < gated_mean,
        "expected the continuous cost function to settle closer to the exact target than the \
         gated one (continuous_mean={continuous_mean:.6}, gated_mean={gated_mean:.6}) -- if this \
         fails, the discontinuity's effect on cost quality is not what this test's own doc \
         comment predicts and that prediction needs revising, not this assertion"
    );
    assert!(
        continuous_wins >= SEEDS.len() * 3 / 4,
        "expected continuous to be closer-or-tied in at least 3/4 of seeds, was \
         {continuous_wins}/{}",
        SEEDS.len()
    );
}

/// Isolates the gated [`get_constraints_cost_function`] itself (no `Stomp`
/// in the loop) against a hand-picked waypoint outside tolerance,
/// confirming it genuinely discriminates satisfied/violated rather than the
/// main test's "stuck exactly at the start value" result coming from a cost
/// function that always returns `0.0`.
#[test]
fn gated_constraints_cost_function_reports_nonzero_cost_past_the_tolerance_edge() {
    let (model, srdf) = load_panda();
    let group = model.joint_model_group("panda_arm").unwrap();
    let num_dimensions = group.active_joint_names().len();
    let constraint =
        JointConstraint::new(&model, "panda_joint1", 0.0, TOLERANCE, TOLERANCE, 1.0).unwrap();
    let mut set = KinematicConstraintSet::new();
    set.push(Constraint::Joint(constraint));

    let mut scene = PlanningScene::new(&model, &srdf);
    let cell = RefCell::new(&mut scene);
    let mut cost_fn = get_constraints_cost_function(&cell, group, &set, COST_SCALE).unwrap();

    // 3 waypoints: inside tolerance (0.01), outside tolerance (0.05), inside (0.0).
    let mut values = DMatrix::zeros(num_dimensions, 3);
    values[(0, 0)] = 0.01;
    values[(0, 1)] = 0.05;
    values[(0, 2)] = 0.0;
    let (costs, validity) = cost_fn(&values).unwrap();
    assert!(!validity, "waypoint 1 (0.05) is outside tolerance 0.02");
    assert!(
        costs[1] > 0.0,
        "outside-tolerance waypoint must have nonzero cost, got {costs:?}"
    );
}
