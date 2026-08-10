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
//! [`cspace_stomp_core::generate_finite_difference_matrix`] performs by
//! dropping out-of-range stencil offsets per row. The five values/offsets
//! are also exactly `cspace_stomp_core::FINITE_CENTRAL_DIFF_COEFFS`'s
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
//! [`cspace_core::sampling::MultivariateGaussian`] is itself stateless --
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

use cspace_core::error::{Error, Result};
use cspace_core::sampling::MultivariateGaussian;
use cspace_stomp_core::{
    DerivativeOrder, full_piv_lu_try_inverse_or_empty, generate_finite_difference_matrix,
};
use nalgebra::{DMatrix, DVector};
use rand::Rng;

use crate::stomp::composable_task::NoiseGeneratorFn;

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
    use cspace_stomp_core::DerivativeOrder;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    /// The actual conditioning behind
    /// `num_timesteps_never_produces_a_covariance_multivariate_gaussian_new_rejects`'s
    /// rejection-reachability argument (see that test's own doc). "A Gram
    /// matrix's inverse is PD whenever the Gram matrix is invertible" is
    /// exact-arithmetic reasoning; `Cholesky::new` runs in `f64`, where a
    /// pivot can round to non-positive well before a matrix is
    /// mathematically singular. This measures how far `acceleration^T *
    /// acceleration`'s actual conditioning is from that failure mode at
    /// this test's own two edges, `n = 60` (the largest `num_timesteps`
    /// any real fixture in this workspace uses) and `n = 200` (the
    /// largest sampled point).
    ///
    /// Measured via `nalgebra`'s `symmetric_eigenvalues` (the matrix is
    /// symmetric by construction, `A^T * A`): at `n = 60`, `min_eig ~=
    /// 7.11e-6`, `max_eig ~= 28.4`, condition number `~= 4.00e6`. At `n =
    /// 200`, `min_eig ~= 5.99e-8`, `max_eig ~= 28.4` (essentially
    /// unchanged -- the largest eigenvalue is set by the stencil's own
    /// fixed coefficients, not `n`), condition number `~= 4.75e8`.
    /// Growth from `n = 60` to `n = 200` (a 3.33x increase in `n`) is a
    /// ~119x increase in condition number -- `log(119)/log(3.33) ~= 4.0`,
    /// consistent with the known `O(n^4)` conditioning growth of a
    /// second-derivative finite-difference Gram matrix (`k = 2` =>
    /// `O(n^{2k})`).
    ///
    /// **Distance from the failure zone.** A backward-stable Cholesky's
    /// pivots are corrupted by rounding error roughly of order `n times
    /// f64::EPSILON times max_eig`; a pivot risks rounding to
    /// non-positive once `min_eig` drops below roughly that size, i.e.
    /// once the condition number approaches roughly `1 / (n *
    /// f64::EPSILON)`. At `n = 200` that threshold works out to roughly
    /// `2.25e13` -- the measured `4.75e8` sits about four to five orders
    /// of magnitude below it, not close. Extrapolating the measured
    /// `O(n^4)` growth law to find where conditioning would reach that
    /// threshold (`n^4 * (4.75e8 / 200^4)` equal to roughly `2.25e13`)
    /// gives `n` around `2950` -- about 15x past the largest point
    /// checked and about 49x past the largest `num_timesteps` any real
    /// fixture in this workspace uses. Not close -- four to five orders
    /// of magnitude of headroom at the densest point checked, growing
    /// wider (in absolute pivot-risk terms) as `n` shrinks back toward
    /// this workspace's real usage.
    ///
    /// **What this test does and does not guard, checked by mutation, not
    /// asserted.** This test pins the two measured numbers as a
    /// regression guard on `generate_finite_difference_matrix`'s
    /// `DerivativeOrder::Acceleration` coefficients specifically --
    /// mutating one coefficient (`FINITE_CENTRAL_DIFF_COEFFS`'s
    /// acceleration row, `-30.0/12.0` to `-30.5/12.0`) reddens only this
    /// test (plus one unrelated, already-fragile `cspace-stomp-core`
    /// convergence probe as collateral); no test that checks this
    /// function's actual noise/covariance output catches it. It does
    /// **not** guard this function's own use of that matrix: this test
    /// recomputes `acceleration^T * acceleration` directly from
    /// `generate_finite_difference_matrix`, the same two lines this
    /// function's body computes, rather than calling this function --
    /// mutating this function's own call (`DerivativeOrder::Acceleration`
    /// to `DerivativeOrder::Velocity`, a wrong-derivative-order bug) does
    /// **not** redden this test at all; six *other* tests
    /// (`num_timesteps_never_produces_a_covariance_multivariate_gaussian_new_rejects`
    /// and five output-checking tests in this module and `planner.rs`)
    /// catch that class of bug instead. A prior version of this doc
    /// claimed protection against "this function's normalization" too --
    /// that was untested and wrong, per this same bite-check.
    ///
    /// # Not the same shape as a smoke test (round: margin audit follow-up)
    ///
    /// This test body has two separate assertions per `n`, and only one of
    /// them is loose. `cond < failure_threshold / 1e4` is the wide-margin
    /// claim (four to five orders of magnitude, see above) -- deliberately
    /// loose, matching this test's own name, and not meant to be tight: it
    /// answers "are we anywhere near Cholesky failure," not "is the
    /// coefficient table exactly right." `(cond - expected_cond).abs() <
    /// expected_cond * tolerance` (`tolerance = 0.05`) is the tight
    /// companion: it pins the measured condition number to within 5% of a
    /// specific expected value, and it is this assertion, not the wide one,
    /// that the bite-check above actually exercises -- re-run this round
    /// (`FINITE_CENTRAL_DIFF_COEFFS`'s acceleration row, `-30.0/12.0` to
    /// `-30.5/12.0`, reverted, `git diff` confirmed clean before
    /// committing): this test still reddens on `expected_cond`, unchanged
    /// from the prior round's result. So unlike
    /// `cspace-stomp-core::stomp::tests`' six `solve_*_converges` tests
    /// (see `BIAS_THRESHOLD`'s own doc, reclassified this round as smoke
    /// tests with no hidden tight reading), a wide margin on one assertion
    /// here does not mean the test lacks power -- it means the power lives
    /// in the *other* assertion, which this bite-check already confirmed
    /// catches a real regression.
    #[test]
    fn acceleration_gram_matrix_conditioning_has_wide_margin_from_cholesky_failure() {
        // (n, expected order-of-magnitude condition number, generous
        // relative tolerance -- these pin the measurement, not a precise
        // physical constant, so the tolerance only needs to catch a
        // regression of orders of magnitude, not float noise).
        let cases = [(60usize, 4.00e6, 0.05), (200usize, 4.75e8, 0.05)];
        for (n, expected_cond, tolerance) in cases {
            let acceleration =
                generate_finite_difference_matrix(n, DerivativeOrder::Acceleration, 1.0);
            let raw_covariance = acceleration.transpose() * &acceleration;
            let eigenvalues = raw_covariance.symmetric_eigenvalues();
            let min_eig = eigenvalues.iter().cloned().fold(f64::INFINITY, f64::min);
            let max_eig = eigenvalues
                .iter()
                .cloned()
                .fold(f64::NEG_INFINITY, f64::max);
            let cond = max_eig / min_eig;

            // The failure-risk threshold a backward-stable Cholesky
            // approaches, per this test's own doc.
            let failure_threshold = 1.0 / (n as f64 * f64::EPSILON);
            assert!(
                cond < failure_threshold / 1e4,
                "n={n}: condition number {cond:e} is within 1e4x of the estimated Cholesky \
                 failure zone {failure_threshold:e} -- the sampled-point coverage above n=60 \
                 in num_timesteps_never_produces_a_covariance_multivariate_gaussian_new_rejects \
                 needs to become contiguous, not just this assertion"
            );
            assert!(
                (cond - expected_cond).abs() < expected_cond * tolerance,
                "n={n}: measured condition number {cond:e}, expected ~{expected_cond:e} -- if \
                 this genuinely moved, re-derive this test's doc comment's margin numbers, \
                 don't just widen the tolerance"
            );
        }
    }

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

    /// Boundary evidence for this function's own doc, "Neither is reachable
    /// for any `num_timesteps >= 1`", specifically the second `ok_or_else`
    /// (`MultivariateGaussian::new` rejecting the normalized covariance).
    /// Round-33 review (`doc/claim-audit/cspace-sampling.md`'s §194
    /// re-check) asked whether `MultivariateGaussian::new`'s fallibility
    /// -- which upstream's own constructor does not have -- rejects any
    /// input a real caller can reach here.
    ///
    /// It cannot via `stddev`: `stddev` never reaches `covariance` in this
    /// function's body at all (it only scales already-sampled noise,
    /// below) -- it is structurally disconnected from the argument
    /// `MultivariateGaussian::new` checks, not merely untested against it.
    /// The only caller-reachable input that *does* feed `covariance` is
    /// `num_timesteps`, and by construction `covariance` is the
    /// max-abs-normalized inverse of `acceleration^T * acceleration` for
    /// an already-confirmed-invertible `acceleration` (the first
    /// `ok_or_else` above) -- a Gram matrix's inverse is positive-definite
    /// whenever the Gram matrix itself is invertible, and positive scalar
    /// normalization preserves that, so `MultivariateGaussian::new`'s
    /// rejection is mathematically unreachable here, not merely empirically
    /// rare, for any `num_timesteps` past that same first check. This test
    /// backs that argument the same way `filter_functions::simple_smoothing_matrix`'s
    /// own sibling premise (`filter_functions.rs`'s test module, "no
    /// realistic `(num_timesteps, dt)` input... makes it singular") is
    /// backed there: not proof for every `usize`, but wide enough to catch
    /// a floating-point-conditioning failure if the mathematical argument
    /// were wrong in practice, not just in theory.
    ///
    /// What is actually covered, exactly: `1..=200` **contiguously**, no
    /// gap. This used to be `1..=60` contiguous plus four sampled points
    /// (`80`, `100`, `150`, `200`) -- non-contiguous because the
    /// `O(n^3)` `full_piv_lu`/Cholesky cost of a full contiguous sweep to
    /// 200 was measured (this crate's earlier rounds) to time out past
    /// 100s under the workspace's then-`opt-level = 0` dev profile.
    /// `e733f19` raised the workspace dev profile to `opt-level = 1`;
    /// under that profile this test's full `1..=200` contiguous body
    /// measures `0.7s` (`cargo nextest run -p cspace-planners --
    /// num_timesteps_never_produces_a_covariance_multivariate_gaussian_new_rejects`),
    /// the single slowest test in this crate but not by a margin that
    /// matters -- the reason for sampling instead of a full sweep no
    /// longer holds, so the gap is closed rather than left justified by a
    /// stale cost. (`60` itself was never tied to real usage in this
    /// workspace -- the largest `num_timesteps` any call to *this
    /// function* makes anywhere in this workspace's own tests is
    /// `planner.rs`'s `15`; `solve_with_60_timesteps_converges`'s `60`
    /// exercises `cspace-stomp-core`'s own `DummyTask::new`, a different,
    /// diagonal-and-trivially-PD covariance, not the
    /// acceleration-Gram-matrix-inverse shape this function builds.)
    ///
    /// **Conclusion for D14/§199's shape:** no caller -- real or synthetic
    /// -- can reach a `covariance` `MultivariateGaussian::new` rejects
    /// through this function, so this is not the same defect family as
    /// D14: there is no upstream-accepted wire value this port's stricter
    /// constructor silently drops on the floor.
    #[test]
    fn num_timesteps_never_produces_a_covariance_multivariate_gaussian_new_rejects() {
        for n in 1..=200usize {
            match normal_distribution_generator(n, vec![1.0], ChaCha8Rng::seed_from_u64(1)) {
                Ok(_) => {}
                Err(e) => panic!("num_timesteps={n} failed: {e:?}"),
            }
        }
    }

    /// Port of `test_noise_generator.cpp`'s `testStartEndUnchanged` --
    /// upstream's exact `TIMESTEPS=100`, `VARIABLES=6`, `STDDEV=0.2`
    /// literals, not this module's usual small synthetic fixtures. Also
    /// closes a gap this round's value-level cross-reference found:
    /// `stddev_scales_the_noise_magnitude` above never independently checks
    /// that generated noise is actually nonzero, so it would pass trivially
    /// under a hypothetical all-zero-noise bug -- upstream's
    /// `EXPECT_NE(noise, NOISE)` (`NOISE` is the all-zero matrix passed in
    /// as the out-parameter) is reproduced explicitly here.
    ///
    /// # Margin/reachability audit (round: margin audit): no bound, no shadowing
    ///
    /// All six assertions here are exact equality/inequality checks
    /// (`assert_eq!`/`assert_ne!`), not inequality-against-a-threshold
    /// checks -- there is no "measured value vs. bound" margin to compute,
    /// unlike `cost_functions.rs`'s `0.681`/`PENALTY` bounds. Checked
    /// separately for the shadowed-assertion defect that motivated splitting
    /// `upstream_test_get_cost_function_invalid_states`
    /// (`cost_functions.rs`): the three leading `assert_ne!`s only fail
    /// under an all-zero-noise bug, and under that specific bug the trailing
    /// loop's `assert_eq!`s would pass vacuously anyway (`values ==
    /// noisy_values` trivially holds everywhere, not just at the pinned
    /// start/end indices) -- so an earlier `assert_ne!` failing never hides
    /// information the loop would otherwise have reported. No reordering or
    /// split needed here.
    #[test]
    fn upstream_test_start_end_unchanged() {
        const TIMESTEPS: usize = 100;
        const VARIABLES: usize = 6;
        let stddev = vec![0.2; VARIABLES];
        let mut generate =
            normal_distribution_generator(TIMESTEPS, stddev, ChaCha8Rng::seed_from_u64(1)).unwrap();
        let values = DMatrix::from_element(VARIABLES, TIMESTEPS, 1.0);
        let (noisy_values, noise) = generate(&values).unwrap();

        assert_ne!(noise, DMatrix::zeros(VARIABLES, TIMESTEPS));
        assert_ne!(noisy_values, DMatrix::zeros(VARIABLES, TIMESTEPS));
        assert_ne!(values, noisy_values);
        for row in 0..VARIABLES {
            assert_eq!(values[(row, 0)], noisy_values[(row, 0)]);
            assert_eq!(
                values[(row, TIMESTEPS - 1)],
                noisy_values[(row, TIMESTEPS - 1)]
            );
        }
    }
}
