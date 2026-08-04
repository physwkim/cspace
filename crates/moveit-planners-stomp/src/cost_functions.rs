// Copyright (c) 2023, PickNik Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/stomp/include/stomp_moveit/cost_functions.hpp

//! `stomp_moveit::costs`: the generic (no `PlanningScene`) half of
//! `cost_functions.hpp`.
//!
//! # Ported: `getCostFunctionFromStateValidator`, `costs::sum`
//!
//! Both operate on an arbitrary [`StateValidatorFn`] the caller supplies --
//! no `moveit_core::planning_scene::PlanningScene` dependency, so both are
//! in reach of this crate.
//!
//! # Not ported: `getCollisionCostFunction`, `getConstraintsCostFunction`
//!
//! Both are *factories* that build a [`StateValidatorFn`] from a
//! `PlanningScene` (collision checking, `kinematic_constraints`
//! satisfaction respectively). Neither `moveit-scene` nor
//! `moveit-collision`'s `ParryCollisionEnv` is a dependency of this crate
//! this round -- deferred with the same reasoning `lib.rs` already
//! recorded for `cost_functions.hpp` as a whole, now narrowed to just these
//! two functions. A caller of [`crate::planner::plan`] supplies its own
//! [`StateValidatorFn`] (e.g. backed by `moveit-collision` directly) in the
//! meantime.
//!
//! # `long` truncation in the Gaussian-smoothing kernel bounds
//!
//! `const long kernel_start = mu - static_cast<long>(sigma) * 4;` truncates
//! `sigma` to a `long` *before* multiplying by 4, then truncates the
//! `double` result of `mu - (double)that` to `long` again on assignment.
//! `sigma * 4.0` (no truncation) is a different number whenever `sigma`
//! is not already an integer (`sigma = max(1.0, 0.5 * window_size)` is
//! fractional for any even `window_size`). This port reproduces both
//! truncations with explicit `as i64` casts rather than computing in `f64`
//! throughout -- see [`cost_function_from_state_validator`]'s body.

use nalgebra::{DMatrix, DVector};

use crate::composable_task::CostFn;

/// `StateValidatorFn`: upstream's `std::function<double(const
/// Eigen::VectorXd&)>` -- a single state's positions in, a penalty out (`0.0`
/// valid, `> 0.0` invalid, magnitude is the cost).
pub type StateValidatorFn<'a> = Box<dyn Fn(&DVector<f64>) -> f64 + 'a>;

/// `getCostFunctionFromStateValidator(state_validator_fn,
/// interpolation_step_size)`.
///
/// Per timestep: scores the waypoint itself via `state_validator_fn`; if
/// valid and not the last timestep and `interpolation_step_size > 0`, walks
/// interpolated samples toward the next waypoint at an L2-norm step size
/// (capped at a fraction of `0.5`) looking for the first invalid one, and if
/// found, splits that sample's penalty between the two bracketing
/// timesteps weighted by interpolation fraction. Contiguous invalid
/// timesteps form a "window"; each window's total cost is then spread as a
/// Gaussian centered at the window's midpoint (`sigma = max(1.0, 0.5 *
/// window_size)`, `+/- 4*sigma` truncated per this module's doc), rescaled
/// so the kernel's own total still equals the window's original total cost.
///
/// # Preserved: later windows can overwrite an earlier window's smoothed
/// costs
///
/// The kernel-write loop (`costs(j) = <gaussian density>`) overwrites
/// whatever `costs(j)` already holds, including a previous window's
/// already-smoothed contribution if two windows' `+/- 4*sigma` kernels
/// overlap. Upstream does this unconditionally; this port does too, rather
/// than silently accumulating instead of overwriting.
///
/// Always returns `Some` -- upstream's lambda always returns `true`; only
/// `validity` (not this function's own success) ever reports "invalid".
pub fn cost_function_from_state_validator<'a>(
    state_validator_fn: StateValidatorFn<'a>,
    interpolation_step_size: f64,
) -> CostFn<'a> {
    Box::new(move |values: &DMatrix<f64>| {
        let num_timesteps = values.ncols();
        let mut costs = DVector::zeros(num_timesteps);
        let mut validity = true;

        let mut invalid_windows: Vec<(usize, usize)> = Vec::new();
        let mut in_invalid_window = false;

        for timestep in 0..num_timesteps {
            let current = values.column(timestep).clone_owned();
            costs[timestep] += state_validator_fn(&current);
            let mut found_invalid_state = costs[timestep] > 0.0;

            let continue_interpolation = !found_invalid_state
                && timestep + 1 < num_timesteps
                && interpolation_step_size > 0.0;
            if continue_interpolation {
                let next = values.column(timestep + 1).clone_owned();
                let interpolation_step =
                    (0.5_f64).min(interpolation_step_size / (&next - &current).norm());
                let mut interpolation_fraction = interpolation_step;
                while interpolation_fraction < 1.0 {
                    let sample_vec =
                        (1.0 - interpolation_fraction) * &current + interpolation_fraction * &next;
                    let penalty = state_validator_fn(&sample_vec);
                    found_invalid_state = penalty > 0.0;
                    if found_invalid_state {
                        costs[timestep] += (1.0 - interpolation_fraction) * penalty;
                        costs[timestep + 1] += interpolation_fraction * penalty;
                        break;
                    }
                    interpolation_fraction += interpolation_step;
                }
            }

            if found_invalid_state {
                validity = false;
                if !in_invalid_window {
                    invalid_windows.push((timestep, timestep));
                    in_invalid_window = true;
                }
                invalid_windows.last_mut().unwrap().1 = timestep;
            } else {
                in_invalid_window = false;
            }
        }

        for &(start, end) in &invalid_windows {
            let window_cost: f64 = (start..=end).map(|i| costs[i]).sum();
            let window_size = (end - start) as f64 + 1.0;
            let sigma = (1.0_f64).max(0.5 * window_size);
            let mu = 0.5 * (start as f64 + end as f64);

            // See this module's doc, "`long` truncation in the Gaussian-
            // smoothing kernel bounds": truncate `sigma` to an integer
            // before multiplying by 4, then truncate the offset `mu` result
            // to an integer again.
            let sigma_offset = (sigma as i64) * 4;
            let kernel_start = (mu - sigma_offset as f64) as i64;
            let kernel_end = (mu + sigma_offset as f64) as i64;
            let bounded_kernel_start = kernel_start.max(0) as usize;
            let bounded_kernel_end = kernel_end.min(num_timesteps as i64 - 1).max(0) as usize;

            for j in bounded_kernel_start..=bounded_kernel_end {
                costs[j] = (-((j as f64 - mu).powi(2)) / (2.0 * sigma.powi(2))).exp()
                    / (sigma * (2.0 * std::f64::consts::PI).sqrt());
            }

            let cost_sum: f64 = (bounded_kernel_start..=bounded_kernel_end)
                .map(|i| costs[i])
                .sum();
            let scale = window_cost / cost_sum;
            for j in bounded_kernel_start..=bounded_kernel_end {
                costs[j] *= scale;
            }
        }

        Some((costs, validity))
    })
}

/// `costs::sum(cost_functions)`: evaluates every function in
/// `cost_functions` against the same `values` and adds their costs,
/// AND-ing their validity flags.
///
/// # Deviation: a failing constituent fails the sum
///
/// Upstream ignores each `cost_fn`'s own `bool` return entirely (`bool
/// valid = true; cost_fn(values, costs, valid); ...`) -- even a `false`
/// return still has its (unspecified) `costs`/`valid` out-parameters added
/// in. This port's [`CostFn`] returns `Option`, which carries no such
/// partial payload for a `None` case, so there is nothing to add in; `sum`
/// instead returns `None` itself if any constituent does. Unreachable in
/// practice through this crate's own [`cost_function_from_state_validator`]
/// (always returns `Some`, matching its own upstream lambda always
/// returning `true`) -- this only changes behavior for a hypothetical
/// caller-supplied [`CostFn`] that returns `None`, which upstream itself
/// has no real call site of either.
pub fn sum<'a>(mut cost_functions: Vec<CostFn<'a>>) -> CostFn<'a> {
    Box::new(move |values: &DMatrix<f64>| {
        let mut overall_costs = DVector::zeros(values.ncols());
        let mut overall_validity = true;
        for cost_fn in cost_functions.iter_mut() {
            let (costs, valid) = cost_fn(values)?;
            overall_costs += &costs;
            overall_validity = overall_validity && valid;
        }
        Some((overall_costs, overall_validity))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sphere_validator(center: f64, radius: f64, penalty: f64) -> StateValidatorFn<'static> {
        Box::new(move |state: &DVector<f64>| {
            if (state[0] - center).abs() < radius {
                penalty
            } else {
                0.0
            }
        })
    }

    #[test]
    fn a_fully_valid_trajectory_has_zero_cost_and_is_valid() {
        let mut cost_fn =
            cost_function_from_state_validator(sphere_validator(100.0, 0.1, 1.0), 0.0);
        let values = DMatrix::from_row_slice(1, 5, &[0.0, 1.0, 2.0, 3.0, 4.0]);
        let (costs, validity) = cost_fn(&values).unwrap();
        assert!(validity);
        assert_eq!(costs, DVector::zeros(5));
    }

    #[test]
    fn an_invalid_waypoint_is_penalized_and_marks_the_trajectory_invalid() {
        let mut cost_fn = cost_function_from_state_validator(sphere_validator(2.0, 0.5, 3.0), 0.0);
        let values = DMatrix::from_row_slice(1, 5, &[0.0, 1.0, 2.0, 3.0, 4.0]);
        let (costs, validity) = cost_fn(&values).unwrap();
        assert!(!validity);
        // Cost is spread by the Gaussian smoothing step, not left as a bare
        // spike at timestep 2 -- but the total across the window must equal
        // the original penalty, and every entry outside the +/-4*sigma
        // kernel (all of it, here: window_size=1 => sigma=1.0 => kernel
        // radius 4) stays at its pre-smoothing value.
        assert!((costs.sum() - 3.0).abs() < 1e-9);
        assert!(costs[2] > 0.0);
    }

    #[test]
    fn interpolation_catches_an_invalid_state_between_two_valid_waypoints() {
        // Waypoints at x=0 and x=4 both valid; the obstacle sits at x=2,
        // reachable only by interpolating between them.
        let mut cost_fn = cost_function_from_state_validator(sphere_validator(2.0, 0.4, 5.0), 0.5);
        let values = DMatrix::from_row_slice(1, 2, &[0.0, 4.0]);
        let (costs, validity) = cost_fn(&values).unwrap();
        assert!(!validity);
        assert!(costs.sum() > 0.0);
    }

    #[test]
    fn zero_interpolation_step_size_skips_interpolation() {
        // Same obstacle-between-waypoints setup as above, but
        // interpolation_step_size = 0.0 must disable the interpolation walk
        // entirely -- both waypoints score 0.0 at the obstacle's midpoint.
        let mut cost_fn = cost_function_from_state_validator(sphere_validator(2.0, 0.4, 5.0), 0.0);
        let values = DMatrix::from_row_slice(1, 2, &[0.0, 4.0]);
        let (costs, validity) = cost_fn(&values).unwrap();
        assert!(validity);
        assert_eq!(costs, DVector::zeros(2));
    }

    #[test]
    fn zero_timesteps_does_not_panic() {
        let mut cost_fn = cost_function_from_state_validator(sphere_validator(0.0, 1.0, 1.0), 0.1);
        let values = DMatrix::<f64>::zeros(1, 0);
        let (costs, validity) = cost_fn(&values).unwrap();
        assert!(validity);
        assert_eq!(costs.len(), 0);
    }

    #[test]
    fn sum_adds_costs_and_ands_validity() {
        let a: CostFn<'static> = Box::new(|values: &DMatrix<f64>| {
            Some((DVector::from_element(values.ncols(), 1.0), true))
        });
        let b: CostFn<'static> = Box::new(|values: &DMatrix<f64>| {
            Some((DVector::from_element(values.ncols(), 2.0), false))
        });
        let mut summed = sum(vec![a, b]);
        let values = DMatrix::zeros(1, 3);
        let (costs, validity) = summed(&values).unwrap();
        assert_eq!(costs, DVector::from_element(3, 3.0));
        assert!(!validity);
    }

    #[test]
    fn sum_of_zero_cost_functions_is_zero_and_valid() {
        let mut summed: CostFn<'static> = sum(Vec::new());
        let values = DMatrix::zeros(1, 4);
        let (costs, validity) = summed(&values).unwrap();
        assert!(validity);
        assert_eq!(costs, DVector::zeros(4));
    }

    #[test]
    fn sum_propagates_a_failing_constituent_as_none() {
        let failing: CostFn<'static> = Box::new(|_values: &DMatrix<f64>| None);
        let mut summed = sum(vec![failing]);
        let values = DMatrix::zeros(1, 2);
        assert!(summed(&values).is_none());
    }
}
