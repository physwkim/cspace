// Copyright (c) 2009, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/stomp/include/stomp_moveit/math/multivariate_gaussian.hpp
//   moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/multivariate_gaussian.hpp

//! Multivariate Gaussian sampling via a Cholesky-decomposed covariance,
//! [`MultivariateGaussian`].
//!
//! # One class, two upstream files -- diffed directly, round 20
//!
//! STOMP's `stomp_moveit::math::MultivariateGaussian` and CHOMP's
//! `chomp::MultivariateGaussian` are the same algorithm (`mean_`/
//! `covariance_`/`covariance_cholesky_` via `covariance.llt().matrixL()`,
//! the same standard-normal sample loop) in two separately maintained
//! files, not one shared header included from both trees. The only real
//! differences are the namespace and STOMP's `sample()` taking an extra
//! `bool use_covariance = true` parameter that CHOMP's `sample()` does not
//! have (CHOMP always applies the covariance). A planner depending on a
//! sibling planner is not this workspace's dependency direction, so rather
//! than have `moveit-planners-chomp` depend on `moveit-planners-stomp` (or
//! vice versa) for one shared class, this port gives the class its own
//! crate that both depend on.
//!
//! # Deviation: two named methods, not a `bool` parameter
//!
//! Upstream's `use_covariance` parameter makes `sample()` mean two
//! different things depending on a runtime flag: "shift a raw standard-
//! normal draw by `mean`" or "scale that draw by the Cholesky factor and
//! *then* shift it" are different operations, not the same operation with a
//! tunable knob. This port splits them into
//! [`MultivariateGaussian::sample_with_covariance`] and
//! [`MultivariateGaussian::sample_without_covariance`] instead of carrying
//! the flag through as a parameter:
//!
//! - [`MultivariateGaussian::sample_with_covariance`] is STOMP's
//!   `sample(output, /* use_covariance = */ true)` -- STOMP's own default,
//!   and its only call site in this tree, `noise_generators.hpp`'s
//!   `rand_generators[i]->sample(*raw_noise)` (not ported this round -- see
//!   `moveit-planners-stomp`'s `lib.rs`), never passes `false`. It is also
//!   CHOMP's *only* `sample(output)` -- CHOMP has no `use_covariance`
//!   parameter at all, and always applies the covariance.
//! - [`MultivariateGaussian::sample_without_covariance`] is STOMP's
//!   `sample(output, false)` branch. Part of STOMP's public class
//!   interface, ported for completeness, but this port found no call site
//!   anywhere in the `moveit2` tree that passes `false` -- CHOMP's own
//!   `MultivariateGaussian` does not have this branch at all.
//!
//! # Deviation: construction can fail
//!
//! Upstream computes `covariance.llt().matrixL()` unconditionally in the
//! constructor and never checks Eigen's `LLT::info()`. For a covariance
//! that is not positive-definite, `matrixL()` still returns a matrix --
//! built from the square root of a negative pivot, `NaN` -- and every
//! subsequent `sample` call silently produces `NaN` waypoints with no
//! signal at the call site that anything went wrong. This port makes that
//! state unconstructable: [`MultivariateGaussian::new`] returns `None` for
//! a `mean`/`covariance` shape mismatch or when `covariance`'s Cholesky
//! decomposition fails.

use nalgebra::{Cholesky, DMatrix, DVector};
use rand::{Rng, RngExt};
use rand_distr::StandardNormal;

/// `stomp_moveit::math::MultivariateGaussian` / `chomp::MultivariateGaussian`.
#[derive(Debug, Clone)]
pub struct MultivariateGaussian {
    mean: DVector<f64>,
    covariance_cholesky: DMatrix<f64>,
}

impl MultivariateGaussian {
    /// `MultivariateGaussian(mean, covariance)`. `None` if `covariance` is
    /// not square with `mean.len()` rows and columns, or is not
    /// positive-definite -- see the module doc's "Deviation: construction
    /// can fail".
    pub fn new(mean: DVector<f64>, covariance: DMatrix<f64>) -> Option<Self> {
        let size = mean.len();
        if covariance.nrows() != size || covariance.ncols() != size {
            return None;
        }
        let cholesky = Cholesky::new(covariance)?;
        Some(Self {
            mean,
            covariance_cholesky: cholesky.l(),
        })
    }

    /// The distribution's dimension.
    pub fn size(&self) -> usize {
        self.mean.len()
    }

    /// STOMP's `sample(output, /* use_covariance = */ true)` and CHOMP's
    /// only `sample(output)`: draws [`Self::size`] iid standard-normal
    /// values, scales by the Cholesky factor, then shifts by `mean`.
    /// `output` is resized to [`Self::size`] first if it does not already
    /// match. See the module doc's "Deviation: two named methods, not a
    /// `bool` parameter".
    pub fn sample_with_covariance(&self, output: &mut DVector<f64>, rng: &mut impl Rng) {
        self.draw_standard_normal(output, rng);
        *output = &self.mean + &self.covariance_cholesky * &*output;
    }

    /// STOMP's `sample(output, /* use_covariance = */ false)`: draws
    /// [`Self::size`] iid standard-normal values and shifts by `mean`,
    /// without applying the covariance's Cholesky factor. `output` is
    /// resized to [`Self::size`] first if it does not already match. See
    /// the module doc's "Deviation: two named methods, not a `bool`
    /// parameter".
    pub fn sample_without_covariance(&self, output: &mut DVector<f64>, rng: &mut impl Rng) {
        self.draw_standard_normal(output, rng);
        *output = &self.mean + &*output;
    }

    fn draw_standard_normal(&self, output: &mut DVector<f64>, rng: &mut impl Rng) {
        let size = self.size();
        if output.len() != size {
            *output = DVector::zeros(size);
        }
        for i in 0..size {
            output[i] = rng.sample(StandardNormal);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

    // Assertion-discrimination sweep (round 2): `new` has exactly two
    // `None`-producing sites -- the shape guard at line 82 and
    // `Cholesky::new(..)?` at line 84 -- and a bare `.is_none()` cannot
    // say which one fired in general (`Option::None` carries no payload
    // to swap, unlike `Error::other("msg")`). Verdict here is
    // `single-branch`, but per-test rather than per-function: each
    // test's specific input can reach only one of the two sites, proven
    // empirically (not by eyeball) by no-op'ing the shape guard
    // (`if covariance.nrows() != size || covariance.ncols() != size`
    // -> `if false`) and re-running this cluster:
    //   - `mismatched_covariance_shape_is_none` (3x3 identity, mean len
    //     2): with the guard gone, `Cholesky::new` on the now-unchecked
    //     3x3 identity succeeds (it is square and positive-definite),
    //     so the assertion flips to `Some` and fails -- the guard was
    //     this test's only route to `None`.
    //   - `non_square_covariance_is_none` (2x3, mean len 2): with the
    //     guard gone, `Cholesky::new` on a non-square matrix panics
    //     (nalgebra: "The input matrix must be square") rather than
    //     returning `None` -- the guard was this test's only route to
    //     `None` too; the Cholesky site cannot substitute for it even
    //     by accident.
    //   - `indefinite_covariance_is_none` / the zero-covariance test
    //     below: shapes already match (`nrows() == ncols() == size`),
    //     so the guard's condition is false regardless of whether the
    //     guard exists; disabling it changes nothing, and both tests
    //     passed unaffected in the same run. Their `None` can only come
    //     from `Cholesky::new` failing.
    // Reachability-bite output: 2 failed (the two shape tests, one via
    // assertion, one via panic), 2 passed unaffected (the two Cholesky
    // tests) -- exactly the split the input construction predicts.
    // Mutation reverted (`git diff` empty) before this comment was
    // written; no source change was needed.
    #[test]
    fn mismatched_covariance_shape_is_none() {
        let mean = DVector::from_vec(vec![0.0, 0.0]);
        let covariance = DMatrix::identity(3, 3);
        assert!(MultivariateGaussian::new(mean, covariance).is_none());
    }

    #[test]
    fn non_square_covariance_is_none() {
        let mean = DVector::from_vec(vec![0.0, 0.0]);
        let covariance = DMatrix::from_row_slice(2, 3, &[1.0, 0.0, 0.0, 0.0, 1.0, 0.0]);
        assert!(MultivariateGaussian::new(mean, covariance).is_none());
    }

    #[test]
    fn indefinite_covariance_is_none() {
        // Eigenvalues 3 and -1: not positive-definite.
        let mean = DVector::from_vec(vec![0.0, 0.0]);
        let covariance = DMatrix::from_row_slice(2, 2, &[1.0, 2.0, 2.0, 1.0]);
        assert!(MultivariateGaussian::new(mean, covariance).is_none());
    }

    #[test]
    fn positive_definite_covariance_constructs() {
        let mean = DVector::from_vec(vec![1.0, -2.0]);
        let covariance = DMatrix::from_row_slice(2, 2, &[4.0, 0.0, 0.0, 9.0]);
        assert!(MultivariateGaussian::new(mean, covariance).is_some());
    }

    #[test]
    fn zero_covariance_is_positive_semidefinite_not_definite_and_is_none() {
        // The boundary this port's precondition actually excludes: strictly
        // PSD (not PD) covariance. Upstream's LLT would compute a
        // covariance_cholesky_ of all zeros here and silently return `mean`
        // unperturbed from every `sample` call; this port refuses to
        // construct instead.
        let mean = DVector::from_vec(vec![0.0]);
        let covariance = DMatrix::from_row_slice(1, 1, &[0.0]);
        assert!(MultivariateGaussian::new(mean, covariance).is_none());
    }

    #[test]
    fn identity_covariance_makes_with_covariance_match_without_covariance() {
        // L = I for an identity covariance, so mean + L*x == mean + x
        // bit-for-bit: off-diagonal L entries are computed as exactly
        // 0.0 - 0.0 (no rounding), and the diagonal is sqrt(1.0) == 1.0
        // exactly.
        let mean = DVector::from_vec(vec![0.3, -0.7, 1.1]);
        let covariance = DMatrix::identity(3, 3);
        let g = MultivariateGaussian::new(mean, covariance).unwrap();

        let mut rng1 = ChaCha8Rng::seed_from_u64(7);
        let mut rng2 = ChaCha8Rng::seed_from_u64(7);
        let mut with_covariance = DVector::zeros(3);
        let mut without_covariance = DVector::zeros(3);
        g.sample_with_covariance(&mut with_covariance, &mut rng1);
        g.sample_without_covariance(&mut without_covariance, &mut rng2);

        assert_eq!(with_covariance, without_covariance);
    }

    #[test]
    fn sample_resizes_an_undersized_output_vector() {
        let mean = DVector::from_vec(vec![0.0, 0.0]);
        let covariance = DMatrix::identity(2, 2);
        let g = MultivariateGaussian::new(mean, covariance).unwrap();
        let mut rng = ChaCha8Rng::seed_from_u64(1);
        let mut output = DVector::zeros(0);
        g.sample_with_covariance(&mut output, &mut rng);
        assert_eq!(output.len(), 2);
    }

    #[test]
    fn empirical_mean_and_variance_converge_over_many_samples() {
        // Property test, not a bit-exact one -- `MultivariateGaussian` wraps
        // a genuinely random process, so this checks the distribution's own
        // guarantees (mean, per-component variance) rather than a specific
        // sampled value. 20,000 draws, diagonal covariance diag(4, 9) (sigma
        // 2 and 3), tolerance sized from the standard error of a sample mean
        // (sigma/sqrt(n)) over that many draws, not guessed:
        // sigma/sqrt(20000) ~= 0.014-0.021, so 0.15 (~7-10 sigma of headroom)
        // catches a real bug without being a coin flip on a normal run.
        let mean = DVector::from_vec(vec![5.0, -3.0]);
        let covariance = DMatrix::from_row_slice(2, 2, &[4.0, 0.0, 0.0, 9.0]);
        let g = MultivariateGaussian::new(mean.clone(), covariance).unwrap();
        let mut rng = ChaCha8Rng::seed_from_u64(42);

        let n = 20_000;
        let mut sum = DVector::zeros(2);
        let mut sum_sq = DVector::zeros(2);
        let mut output = DVector::zeros(2);
        for _ in 0..n {
            g.sample_with_covariance(&mut output, &mut rng);
            sum += &output;
            sum_sq += output.component_mul(&output);
        }
        let empirical_mean = &sum / n as f64;
        let empirical_var = &sum_sq / n as f64 - empirical_mean.component_mul(&empirical_mean);

        assert_relative_eq!(empirical_mean[0], mean[0], epsilon = 0.15);
        assert_relative_eq!(empirical_mean[1], mean[1], epsilon = 0.15);
        assert_relative_eq!(empirical_var[0], 4.0, epsilon = 0.5);
        assert_relative_eq!(empirical_var[1], 9.0, epsilon = 0.9);
    }

    #[test]
    fn without_covariance_ignores_correlation_with_covariance_does_not() {
        // A covariance with strong off-diagonal correlation: with_covariance
        // must reproduce that correlation empirically; without_covariance,
        // which never touches covariance_cholesky, must not.
        let mean = DVector::from_vec(vec![0.0, 0.0]);
        let covariance = DMatrix::from_row_slice(2, 2, &[1.0, 0.9, 0.9, 1.0]);
        let g = MultivariateGaussian::new(mean, covariance).unwrap();
        let mut rng = ChaCha8Rng::seed_from_u64(99);

        let n = 20_000;
        let mut sum_xy_with = 0.0;
        let mut sum_xy_without = 0.0;
        let mut output = DVector::zeros(2);
        for _ in 0..n {
            g.sample_with_covariance(&mut output, &mut rng);
            sum_xy_with += output[0] * output[1];
            g.sample_without_covariance(&mut output, &mut rng);
            sum_xy_without += output[0] * output[1];
        }
        let empirical_cov_with = sum_xy_with / n as f64;
        let empirical_cov_without = sum_xy_without / n as f64;

        // Target correlation 0.9, standard error of this estimator is large
        // (correlated-product estimator, not a plain sample mean) -- 0.15
        // margin around 0.9 still cleanly separates it from 0.0.
        assert_relative_eq!(empirical_cov_with, 0.9, epsilon = 0.15);
        assert_relative_eq!(empirical_cov_without, 0.0, epsilon = 0.15);
    }
}
