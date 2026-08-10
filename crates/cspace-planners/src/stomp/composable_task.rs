// Copyright (c) 2023, PickNik Inc.
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/stomp/include/stomp_moveit/stomp_moveit_task.hpp

//! `stomp_moveit::ComposableTask`: a [`cspace_stomp_core::Task`] built from
//! plain closures instead of a hand-written implementation per use case.
//!
//! # `FilterFn` stays in `filter_functions`
//!
//! Upstream declares `NoiseGeneratorFn`, `CostFn`, `FilterFn`,
//! `PostIterationFn`, and `DoneFn` together in this one header.
//! [`filter_functions::FilterFn`](crate::stomp::filter_functions::FilterFn) was
//! already carried into this crate in an earlier round (every filter
//! function needs the signature); the other four are new this round and
//! live here instead of being duplicated.
//!
//! # `computeCosts`/`computeNoisyCosts` both forward to the same `cost_fn_`
//!
//! Upstream's `ComposableTask` stores one `CostFn cost_fn_` and both
//! `computeCosts` and `computeNoisyCosts` call it -- there is no separate
//! "noisy" cost function at this layer, just one cost function evaluated
//! against whichever matrix (noisy rollout or current best) is passed in.
//! [`ComposableTask::compute_costs`] and [`ComposableTask::compute_noisy_costs`]
//! below both call the same stored `cost_fn`, reproducing that exactly.
//!
//! # `filterNoisyParameters` is not overridden
//!
//! Upstream's `ComposableTask` class only overrides `filterParameterUpdates`
//! (wired from `filter_fn_`); it does not override `filterNoisyParameters`
//! at all, so that hook falls through to `stomp::Task`'s own default (a
//! no-op that reports "unfiltered"). This port leaves
//! [`cspace_stomp_core::Task::filter_noisy_parameters`] at its trait
//! default for the same reason -- there is no `filter_noisy_parameters`
//! method on [`ComposableTask`] to override it with.

use cspace_stomp_core::Task;
use nalgebra::{DMatrix, DVector};

use crate::stomp::filter_functions::FilterFn;

/// `NoiseGeneratorFn`. Constructs a new noisy-parameters/noise pair each
/// call -- this crate's "return values, not out-parameters" convention (see
/// `cspace_stomp_core::task`'s module doc, the same convention this port
/// follows), so `None` is upstream's `bool` return of `false` and the
/// `Eigen::MatrixXd&` out-parameters become the `Some` payload.
/// `FnMut`, not `Fn`: a real noise generator holds RNG state that advances
/// on every call (see [`crate::stomp::noise_generators::normal_distribution_generator`]).
pub type NoiseGeneratorFn<'a> =
    Box<dyn FnMut(&DMatrix<f64>) -> Option<(DMatrix<f64>, DMatrix<f64>)> + 'a>;

/// `CostFn`. Same "return values, not out-parameters" shape as
/// [`NoiseGeneratorFn`]: `None` is upstream's `false`, `Some((costs,
/// validity))` is the `Eigen::VectorXd&`/`bool&` out-parameter pair.
pub type CostFn<'a> = Box<dyn FnMut(&DMatrix<f64>) -> Option<(DVector<f64>, bool)> + 'a>;

/// `PostIterationFn`. Upstream's `void` return has no failure signal to
/// translate -- a plain closure, not `Option`-wrapped.
pub type PostIterationFn<'a> = Box<dyn FnMut(i32, f64, &DMatrix<f64>) + 'a>;

/// `DoneFn`. Same shape as [`PostIterationFn`]: upstream's `void` return.
pub type DoneFn<'a> = Box<dyn FnMut(bool, i32, f64, &DMatrix<f64>) + 'a>;

/// `stomp_moveit::ComposableTask`: forwards every [`Task`] method to a
/// stored closure. See this module's doc for the two upstream shapes this
/// preserves exactly: `computeCosts`/`computeNoisyCosts` sharing one
/// `cost_fn_`, and `filterNoisyParameters` not being overridden at all.
pub struct ComposableTask<'a> {
    noise_generator_fn: NoiseGeneratorFn<'a>,
    cost_fn: CostFn<'a>,
    filter_fn: FilterFn<'a>,
    post_iteration_fn: PostIterationFn<'a>,
    done_fn: DoneFn<'a>,
}

impl<'a> ComposableTask<'a> {
    /// `ComposableTask(noise_generator_fn, cost_fn, filter_fn,
    /// post_iteration_fn, done_fn)`.
    pub fn new(
        noise_generator_fn: NoiseGeneratorFn<'a>,
        cost_fn: CostFn<'a>,
        filter_fn: FilterFn<'a>,
        post_iteration_fn: PostIterationFn<'a>,
        done_fn: DoneFn<'a>,
    ) -> Self {
        Self {
            noise_generator_fn,
            cost_fn,
            filter_fn,
            post_iteration_fn,
            done_fn,
        }
    }
}

impl<'a> Task for ComposableTask<'a> {
    fn generate_noisy_parameters(
        &mut self,
        parameters: &DMatrix<f64>,
        _start_timestep: usize,
        _num_timesteps: usize,
        _iteration_number: i32,
        _rollout_number: i32,
    ) -> Option<(DMatrix<f64>, DMatrix<f64>)> {
        (self.noise_generator_fn)(parameters)
    }

    fn compute_noisy_costs(
        &mut self,
        parameters: &DMatrix<f64>,
        _start_timestep: usize,
        _num_timesteps: usize,
        _iteration_number: i32,
        _rollout_number: i32,
    ) -> Option<(DVector<f64>, bool)> {
        (self.cost_fn)(parameters)
    }

    fn compute_costs(
        &mut self,
        parameters: &DMatrix<f64>,
        _start_timestep: usize,
        _num_timesteps: usize,
        _iteration_number: i32,
    ) -> Option<(DVector<f64>, bool)> {
        (self.cost_fn)(parameters)
    }

    fn filter_parameter_updates(
        &mut self,
        _start_timestep: usize,
        _num_timesteps: usize,
        _iteration_number: i32,
        parameters: &DMatrix<f64>,
        updates: &mut DMatrix<f64>,
    ) -> bool {
        (self.filter_fn)(parameters, updates)
    }

    fn post_iteration(
        &mut self,
        _start_timestep: usize,
        _num_timesteps: usize,
        iteration_number: i32,
        cost: f64,
        parameters: &DMatrix<f64>,
    ) {
        (self.post_iteration_fn)(iteration_number, cost, parameters);
    }

    fn done(
        &mut self,
        success: bool,
        total_iterations: i32,
        final_cost: f64,
        parameters: &DMatrix<f64>,
    ) {
        (self.done_fn)(success, total_iterations, final_cost, parameters);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compute_costs_and_compute_noisy_costs_call_the_same_cost_fn() {
        // The exact upstream shape this module doc calls out: one cost_fn_
        // backs both hooks. A call counter proves both `Task` methods reach
        // the same closure, not two independent ones.
        let mut call_count = 0;
        let mut task = ComposableTask::new(
            Box::new(|_| Some((DMatrix::zeros(1, 1), DMatrix::zeros(1, 1)))),
            Box::new(move |values: &DMatrix<f64>| {
                call_count += 1;
                Some((
                    DVector::from_element(values.ncols(), call_count as f64),
                    true,
                ))
            }),
            Box::new(|_values, _filtered| true),
            Box::new(|_, _, _| {}),
            Box::new(|_, _, _, _| {}),
        );

        let parameters = DMatrix::zeros(2, 3);
        let (costs1, valid1) = task.compute_costs(&parameters, 0, 3, 1).unwrap();
        let (costs2, valid2) = task.compute_noisy_costs(&parameters, 0, 3, 1, 0).unwrap();

        assert!(valid1);
        assert!(valid2);
        assert_eq!(costs1, DVector::from_element(3, 1.0));
        assert_eq!(costs2, DVector::from_element(3, 2.0));
    }

    #[test]
    fn filter_noisy_parameters_stays_at_the_task_trait_default() {
        // ComposableTask does not override this hook -- confirms the trait
        // default (true, false) is what a caller actually observes.
        let mut task = ComposableTask::new(
            Box::new(|_| None),
            Box::new(|_| None),
            Box::new(|_values, _filtered| true),
            Box::new(|_, _, _| {}),
            Box::new(|_, _, _, _| {}),
        );
        let mut parameters = DMatrix::zeros(1, 1);
        assert_eq!(
            task.filter_noisy_parameters(0, 1, 1, 0, &mut parameters),
            (true, false)
        );
    }

    #[test]
    fn filter_parameter_updates_forwards_to_the_filter_fn() {
        let mut task = ComposableTask::new(
            Box::new(|_| None),
            Box::new(|_| None),
            Box::new(|_values, filtered| {
                *filtered = DMatrix::from_element(1, 1, 42.0);
                true
            }),
            Box::new(|_, _, _| {}),
            Box::new(|_, _, _, _| {}),
        );
        let parameters = DMatrix::zeros(1, 1);
        let mut updates = DMatrix::zeros(1, 1);
        assert!(task.filter_parameter_updates(0, 1, 1, &parameters, &mut updates));
        assert_eq!(updates, DMatrix::from_element(1, 1, 42.0));
    }

    #[test]
    fn post_iteration_and_done_forward_their_arguments() {
        let mut post_iteration_seen = None;
        let mut done_seen = None;
        let mut task = ComposableTask::new(
            Box::new(|_| None),
            Box::new(|_| None),
            Box::new(|_values, _filtered| true),
            Box::new(|iteration, cost, _params| post_iteration_seen = Some((iteration, cost))),
            Box::new(|success, total, cost, _params| done_seen = Some((success, total, cost))),
        );
        let parameters = DMatrix::zeros(1, 1);
        task.post_iteration(0, 1, 5, 2.5, &parameters);
        task.done(true, 10, 0.1, &parameters);
        drop(task);

        assert_eq!(post_iteration_seen, Some((5, 2.5)));
        assert_eq!(done_seen, Some((true, 10, 0.1)));
    }
}
