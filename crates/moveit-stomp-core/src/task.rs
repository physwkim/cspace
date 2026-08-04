// Copyright (c) 2016, Southwest Research Institute
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: Apache-2.0
//
// Ported from ros-industrial/stomp @ b1a87c80f7338caae25a5c689b876da15492aa75:
//   include/stomp/task.h

//! `stomp::Task`: the pluggable "what am I optimizing" interface
//! [`crate::stomp::Stomp`] drives. Implemented by a caller that knows how to
//! generate noise, score a trajectory, and (optionally) filter it -- this
//! crate never implements it itself.
//!
//! # Out-parameters, split by shape
//!
//! Three methods ([`Task::generate_noisy_parameters`],
//! [`Task::compute_noisy_costs`], [`Task::compute_costs`]) construct a
//! brand-new value from scratch each call; these follow this crate's
//! established "return values, not out-parameters" convention (see
//! `lib.rs`), returning `Option<(payload...)>` where `None` is upstream's
//! `bool` return of `false`.
//!
//! The other two ([`Task::filter_noisy_parameters`],
//! [`Task::filter_parameter_updates`]) transform a value already held by
//! the caller *in place* -- the same shape as
//! `moveit_planners_stomp::filter_functions::FilterFn`
//! (`Fn(&DMatrix<f64>, &mut DMatrix<f64>) -> bool`) elsewhere in this
//! workspace, not a fresh construction. These keep their `&mut DMatrix<f64>`
//! parameter rather than being converted to return a new owned matrix.

use nalgebra::{DMatrix, DVector};

/// `stomp::Task`. Every method takes `&mut self`: upstream's methods are
/// non-`const` virtuals, and real implementations hold mutable state (an
/// RNG, a collision-checking cache).
pub trait Task {
    /// `generateNoisyParameters`. `None` on failure (upstream's `false`
    /// return); on success, `(parameters_noise, noise)`.
    fn generate_noisy_parameters(
        &mut self,
        parameters: &DMatrix<f64>,
        start_timestep: usize,
        num_timesteps: usize,
        iteration_number: i32,
        rollout_number: i32,
    ) -> Option<(DMatrix<f64>, DMatrix<f64>)>;

    /// `computeNoisyCosts`. `None` on failure; on success, `(costs,
    /// validity)`.
    fn compute_noisy_costs(
        &mut self,
        parameters: &DMatrix<f64>,
        start_timestep: usize,
        num_timesteps: usize,
        iteration_number: i32,
        rollout_number: i32,
    ) -> Option<(DVector<f64>, bool)>;

    /// `computeCosts`. `None` on failure; on success, `(costs, validity)`.
    fn compute_costs(
        &mut self,
        parameters: &DMatrix<f64>,
        start_timestep: usize,
        num_timesteps: usize,
        iteration_number: i32,
    ) -> Option<(DVector<f64>, bool)>;

    /// `filterNoisyParameters`. Filters `parameters` in place; returns
    /// `(success, filtered)`, upstream's `bool` return and `bool& filtered`
    /// out-param respectively. Default: upstream's own default body (does
    /// nothing, reports unfiltered).
    fn filter_noisy_parameters(
        &mut self,
        _start_timestep: usize,
        _num_timesteps: usize,
        _iteration_number: i32,
        _rollout_number: i32,
        _parameters: &mut DMatrix<f64>,
    ) -> (bool, bool) {
        (true, false)
    }

    /// `filterParameterUpdates`. Filters `updates` in place; returns
    /// whether it succeeded. Default: upstream's own default body (no-op
    /// success).
    fn filter_parameter_updates(
        &mut self,
        _start_timestep: usize,
        _num_timesteps: usize,
        _iteration_number: i32,
        _parameters: &DMatrix<f64>,
        _updates: &mut DMatrix<f64>,
    ) -> bool {
        true
    }

    /// `postIteration`. Called at the end of each iteration. Default:
    /// upstream's own default body (no-op).
    fn post_iteration(
        &mut self,
        _start_timestep: usize,
        _num_timesteps: usize,
        _iteration_number: i32,
        _cost: f64,
        _parameters: &DMatrix<f64>,
    ) {
    }

    /// `done`. Called once at the end of the optimization process. Default:
    /// upstream's own default body (no-op).
    fn done(
        &mut self,
        _success: bool,
        _total_iterations: i32,
        _final_cost: f64,
        _parameters: &DMatrix<f64>,
    ) {
    }
}
