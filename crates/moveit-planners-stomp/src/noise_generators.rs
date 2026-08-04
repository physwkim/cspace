// Copyright (c) 2023, PickNik Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/stomp/include/stomp_moveit/noise_generators.hpp

//! `stomp_moveit::noise::getNormalDistributionGenerator`.
//!
//! # `acceleration` is `generate_finite_difference_matrix`, not a re-derivation
//!
//! Upstream builds `acceleration` by filling five diagonals of an
//! `Eigen::MatrixXd::Zero(num_timesteps, num_timesteps)` directly at
//! `ACC_MATRIX_DIAGONAL_INDICES = {-2,-1,0,1,2}` with
//! `ACC_MATRIX_DIAGONAL_VALUES = {-1/12, 16/12, -30/12, 16/12, -1/12}`.
//! `Eigen::MatrixXd::diagonal(k)` has length `n - |k|`, so this is exactly a
//! banded matrix where each row only receives the stencil coefficients that
//! stay in range -- e.g. row 0 gets no contribution from `k = -2` or `k =
//! -1` (those diagonals start at row `|k|`), the same boundary truncation
//! [`moveit_stomp_core::generate_finite_difference_matrix`] performs by
//! dropping out-of-range stencil offsets per row. The five values/offsets
//! are also exactly `moveit_stomp_core::FINITE_CENTRAL_DIFF_COEFFS`'s
//! `Acceleration` row's nonzero entries. Confirmed by direct construction
//! (not assumed): both produce the identical `num_timesteps x num_timesteps`
//! matrix for `dt = 1.0` (upstream's `acceleration` has no `dt` scaling at
//! all, i.e. implicitly `dt = 1`, matching
//! `generate_finite_difference_matrix(num_timesteps, Acceleration, 1.0)`'s
//! own `1/dt^order = 1`). This port calls that function directly instead of
//! re-deriving the same banded construction a second time.
//!
//! # Two different `maxCoeff` reductions -- not unified
//!
//! `stomp::Stomp::reset_variables`'s scale factor is
//! `std::abs(inv_control_cost_matrix_R_.maxCoeff())` -- absolute value of
//! the single largest *signed* entry. This function's scale factor is
//! `covariance.array().abs().matrix().maxCoeff()` -- the max over every
//! entry's *absolute value* (`cwiseAbs().maxCoeff()`'s equivalent). These
//! are different reductions in two different upstream files; this port
//! keeps them different rather than sharing one helper that would blur the
//! distinction.
//!
//! # One `MultivariateGaussian`, not `num_dimensions` identical instances
//!
//! Upstream constructs `rand_generators`, a `MultivariateGaussianPtr` per
//! dimension -- but every one of them is built from the exact same
//! `(Eigen::VectorXd::Zero(num_timesteps), covariance)` arguments. In C++
//! this matters only because each carries its own RNG-adjacent state
//! indirectly through Eigen's global RNG; in this port,
//! [`moveit_sampling::MultivariateGaussian`] is itself stateless --
//! `sample_with_covariance` takes the `Rng` externally (see that crate's
//! own module doc) -- so `num_dimensions` separately constructed but
//! identical objects would be functionally interchangeable with one shared
//! object. This port constructs one and reuses it per dimension inside the
//! returned closure.
//!
//! # Preserved: the first and last raw-noise elements are always zero
//!
//! `raw_noise->head(1).setZero(); raw_noise->tail(1).setZero();` before
//! scaling by `stddev[i]` -- every generated noise vector pins waypoint 0
//! and the last waypoint at exactly zero noise, keeping trajectory
//! endpoints fixed against perturbation. Reproduced exactly below.

use moveit_error::{Error, Result};
use moveit_sampling::MultivariateGaussian;
use moveit_stomp_core::{
    DerivativeOrder, full_piv_lu_try_inverse_or_empty, generate_finite_difference_matrix,
};
use nalgebra::{DMatrix, DVector};
use rand::Rng;

use crate::composable_task::NoiseGeneratorFn;

/// `getNormalDistributionGenerator(num_timesteps, stddev)`. `stddev` must
/// have one entry per dimension (row) the returned generator will be called
/// with -- upstream's `stddev.at(i)` for `i` in `0..values.rows()`.
///
/// # Errors
///
/// [`Error::Other`] if `acceleration^T * acceleration` is not invertible
/// (upstream's unchecked `fullPivLu().inverse()`), or if the resulting
/// normalized covariance is not positive-definite (see
/// [`MultivariateGaussian::new`]'s own "Deviation: construction can fail").
/// Neither is reachable for any `num_timesteps >= 1` -- see
/// `filter_functions::simple_smoothing_matrix`'s own note on the same
/// premise for `A^T * A` shaped matrices -- but both are surfaced instead
/// of assumed, matching this port's established `Result` convention for
/// STOMP's control-cost-shaped matrix inversions.
pub fn normal_distribution_generator<'a>(
    num_timesteps: usize,
    stddev: Vec<f64>,
    mut rng: impl Rng + 'a,
) -> Result<NoiseGeneratorFn<'a>> {
    let acceleration =
        generate_finite_difference_matrix(num_timesteps, DerivativeOrder::Acceleration, 1.0);
    let raw_covariance = acceleration.transpose() * &acceleration;
    let mut covariance = full_piv_lu_try_inverse_or_empty(raw_covariance).ok_or_else(|| {
        Error::Other(format!(
            "normal_distribution_generator({num_timesteps}, ..): acceleration^T * acceleration \
             is not invertible"
        ))
    })?;
    let max_abs = covariance.iter().fold(0.0_f64, |acc, &x| acc.max(x.abs()));
    if max_abs > 0.0 {
        covariance /= max_abs;
    }
    let generator = MultivariateGaussian::new(DVector::zeros(num_timesteps), covariance)
        .ok_or_else(|| {
            Error::Other(
                "normal_distribution_generator: the normalized covariance is not positive-definite"
                    .to_string(),
            )
        })?;

    let mut raw_noise = DVector::zeros(num_timesteps);
    Ok(Box::new(move |values: &DMatrix<f64>| {
        let mut noise = DMatrix::zeros(values.nrows(), values.ncols());
        let mut noisy_values = DMatrix::zeros(values.nrows(), values.ncols());
        for i in 0..values.nrows() {
            generator.sample_with_covariance(&mut raw_noise, &mut rng);
            if num_timesteps > 0 {
                raw_noise[0] = 0.0;
                raw_noise[num_timesteps - 1] = 0.0;
            }
            let scale = stddev[i];
            for t in 0..num_timesteps {
                noise[(i, t)] = scale * raw_noise[t];
                noisy_values[(i, t)] = values[(i, t)] + noise[(i, t)];
            }
        }
        Some((noisy_values, noise))
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    #[test]
    fn generated_noise_pins_the_first_and_last_timestep_to_zero() {
        let mut generate =
            normal_distribution_generator(6, vec![1.0, 1.0], ChaCha8Rng::seed_from_u64(11))
                .unwrap();
        let values = DMatrix::zeros(2, 6);
        let (noisy_values, noise) = generate(&values).unwrap();

        for row in 0..2 {
            assert_eq!(noise[(row, 0)], 0.0);
            assert_eq!(noise[(row, 5)], 0.0);
            assert_eq!(noisy_values[(row, 0)], values[(row, 0)]);
            assert_eq!(noisy_values[(row, 5)], values[(row, 5)]);
        }
    }

    #[test]
    fn noisy_values_equal_values_plus_noise() {
        let mut generate =
            normal_distribution_generator(5, vec![0.5], ChaCha8Rng::seed_from_u64(3)).unwrap();
        let values = DMatrix::from_row_slice(1, 5, &[0.0, 1.0, 2.0, 3.0, 4.0]);
        let (noisy_values, noise) = generate(&values).unwrap();
        assert_eq!(noisy_values, &values + &noise);
    }

    #[test]
    fn stddev_scales_the_noise_magnitude() {
        // Same seed, same base draw -- only stddev differs, so the ratio of
        // interior noise entries between the two calls must equal the ratio
        // of stddevs.
        let mut small =
            normal_distribution_generator(5, vec![0.1], ChaCha8Rng::seed_from_u64(42)).unwrap();
        let mut large =
            normal_distribution_generator(5, vec![10.0], ChaCha8Rng::seed_from_u64(42)).unwrap();
        let values = DMatrix::zeros(1, 5);
        let (_, noise_small) = small(&values).unwrap();
        let (_, noise_large) = large(&values).unwrap();

        for t in 1..4 {
            assert!((noise_large[(0, t)] - 100.0 * noise_small[(0, t)]).abs() < 1e-9);
        }
    }

    #[test]
    fn repeated_calls_draw_fresh_noise_via_advancing_rng_state() {
        let mut generate =
            normal_distribution_generator(5, vec![1.0], ChaCha8Rng::seed_from_u64(7)).unwrap();
        let values = DMatrix::zeros(1, 5);
        let (_, first) = generate(&values).unwrap();
        let (_, second) = generate(&values).unwrap();
        assert_ne!(first, second);
    }

    #[test]
    fn zero_timesteps_does_not_panic() {
        let mut generate =
            normal_distribution_generator(0, vec![1.0], ChaCha8Rng::seed_from_u64(1)).unwrap();
        let values = DMatrix::<f64>::zeros(1, 0);
        let (noisy_values, noise) = generate(&values).unwrap();
        assert_eq!(noisy_values.shape(), (1, 0));
        assert_eq!(noise.shape(), (1, 0));
    }
}
