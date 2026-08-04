// Copyright (c) 2009, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/multivariate_gaussian.hpp

//! Multivariate Gaussian sampling via a Cholesky-decomposed covariance,
//! [`MultivariateGaussian`] — CHOMP's own `chomp::MultivariateGaussian`.
//!
//! # Why this crate carries its own copy, not a shared one
//!
//! STOMP's `stomp_moveit::math::MultivariateGaussian` and CHOMP's
//! `chomp::MultivariateGaussian` are the same algorithm (`mean_`/
//! `covariance_`/`covariance_cholesky_` via `covariance.llt().matrixL()`,
//! the same standard-normal sample loop) in two separately maintained
//! files, not one shared header included from both trees — and, decisively,
//! the two upstream trees carry different licenses: `ros-industrial/stomp`
//! (STOMP's upstream) is Apache-2.0, `moveit2` (CHOMP's upstream) is
//! BSD-3-Clause. A single shared struct ported from both headers under one
//! `SPDX-License-Identifier` necessarily mislabels one side — exactly what
//! `tools/ci/check-license-matches-upstream.sh` exists to catch (a crate's
//! sources must carry one SPDX identifier that matches its declared
//! license, traced to one upstream). Round 18 decided each planner crate
//! ports its own copy instead of a `moveit-planners-chomp` /
//! `moveit-planners-stomp` dependency on a shared `moveit-sampling` crate:
//! this file is CHOMP's, transcribed only from
//! `chomp_motion_planner/include/chomp_motion_planner/multivariate_gaussian.hpp`,
//! `SPDX-License-Identifier: BSD-3-Clause` above matching that upstream
//! exactly. STOMP's own copy (with its `use_covariance` branch — see below)
//! is `p3-shapes`'s to carry in `moveit-stomp-core`, not this crate's.
//!
//! # No `use_covariance` parameter
//!
//! Upstream's `chomp::MultivariateGaussian::sample()` takes no `bool` — it
//! always scales by the Cholesky factor before shifting by the mean
//! (`output = mean_ + covariance_cholesky_ * output`). STOMP's sibling class
//! adds a `use_covariance` flag that skips that scaling when `false`; CHOMP
//! never had that second mode, so [`MultivariateGaussian::sample`] here has
//! only the one behavior — nothing to split into two named methods for.
//!
//! # Deviation: construction can fail
//!
//! Upstream computes `covariance_.llt().matrixL()` unconditionally in the
//! constructor and never checks Eigen's `LLT::info()`. For a covariance that
//! is not positive-definite, `matrixL()` still returns a matrix — built from
//! the square root of a negative pivot, `NaN` — and every subsequent
//! `sample()` call silently produces `NaN` output with no signal at the call
//! site that anything went wrong. This port makes that state unconstructable:
//! [`MultivariateGaussian::new`] returns `None` for a `mean`/`covariance`
//! shape mismatch or when `covariance`'s Cholesky decomposition fails.
//!
//! # No live consumer yet
//!
//! `chomp_optimizer.cpp`'s one construction site,
//! `MultivariateGaussian(Eigen::VectorXd::Zero(num_vars_free_),
//! joint_costs_[i].getQuadraticCostInverse())` in `initialize()`, feeds
//! exclusively the Hamiltonian-Monte-Carlo perturbation path
//! (`perturbTrajectory`/`getRandomMomentum`/`updateMomentum`/
//! `updatePositionFromMomentum`), and every call site of that path in
//! `optimize()` is commented out upstream — see [`crate::optimizer`]'s
//! module doc for the full account, including that three of those four
//! methods have no implementation anywhere in `chomp_optimizer.cpp` at all.
//! This module is ported ahead of that need (round 18's placement decision),
//! not because `initialize()` or the HMC path is ported yet — they are not.

use nalgebra::{Cholesky, DMatrix, DVector};
use rand::{Rng, RngExt};
use rand_distr::StandardNormal;

/// `chomp::MultivariateGaussian`.
#[derive(Debug, Clone)]
pub struct MultivariateGaussian {
    mean: DVector<f64>,
    covariance_cholesky: DMatrix<f64>,
}

impl MultivariateGaussian {
    /// `MultivariateGaussian(mean, covariance)`. `None` if `covariance` is
    /// not square with `mean.len()` rows and columns, or is not
    /// positive-definite — see the module doc's "Deviation: construction can
    /// fail".
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

    /// The distribution's dimension (`size_`).
    pub fn size(&self) -> usize {
        self.mean.len()
    }

    /// `sample(output)`: draws [`Self::size`] iid standard-normal values,
    /// scales by the Cholesky factor, then shifts by `mean`. `output` is
    /// resized to [`Self::size`] first if it does not already match.
    pub fn sample(&self, output: &mut DVector<f64>, rng: &mut impl Rng) {
        let size = self.size();
        if output.len() != size {
            *output = DVector::zeros(size);
        }
        for i in 0..size {
            output[i] = rng.sample(StandardNormal);
        }
        *output = &self.mean + &self.covariance_cholesky * &*output;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rand::SeedableRng;
    use rand_chacha::ChaCha8Rng;

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
    fn sample_resizes_an_undersized_output_vector() {
        let mean = DVector::from_vec(vec![0.0, 0.0]);
        let covariance = DMatrix::identity(2, 2);
        let g = MultivariateGaussian::new(mean, covariance).unwrap();
        let mut rng = ChaCha8Rng::seed_from_u64(1);
        let mut output = DVector::zeros(0);
        g.sample(&mut output, &mut rng);
        assert_eq!(output.len(), 2);
    }

    #[test]
    fn identity_covariance_sample_matches_a_raw_standard_normal_shift() {
        // L = I for an identity covariance, so mean + L*x == mean + x
        // bit-for-bit: off-diagonal L entries are computed as exactly
        // 0.0 - 0.0 (no rounding), and the diagonal is sqrt(1.0) == 1.0
        // exactly.
        let mean = DVector::from_vec(vec![0.3, -0.7, 1.1]);
        let covariance = DMatrix::identity(3, 3);
        let g = MultivariateGaussian::new(mean.clone(), covariance).unwrap();

        let mut rng = ChaCha8Rng::seed_from_u64(7);
        let mut raw = DVector::zeros(3);
        for i in 0..3 {
            raw[i] = rng.sample(StandardNormal);
        }
        let expected = &mean + &raw;

        let mut rng = ChaCha8Rng::seed_from_u64(7);
        let mut output = DVector::zeros(3);
        g.sample(&mut output, &mut rng);

        assert_eq!(output, expected);
    }

    #[test]
    fn empirical_mean_and_variance_converge_over_many_samples() {
        // Property test, not a bit-exact one -- MultivariateGaussian wraps a
        // genuinely random process, so this checks the distribution's own
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
            g.sample(&mut output, &mut rng);
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
    fn correlated_covariance_sample_reproduces_the_correlation() {
        let mean = DVector::from_vec(vec![0.0, 0.0]);
        let covariance = DMatrix::from_row_slice(2, 2, &[1.0, 0.9, 0.9, 1.0]);
        let g = MultivariateGaussian::new(mean, covariance).unwrap();
        let mut rng = ChaCha8Rng::seed_from_u64(99);

        let n = 20_000;
        let mut sum_xy = 0.0;
        let mut output = DVector::zeros(2);
        for _ in 0..n {
            g.sample(&mut output, &mut rng);
            sum_xy += output[0] * output[1];
        }
        let empirical_cov = sum_xy / n as f64;

        // Target correlation 0.9; 0.15 margin cleanly separates it from 0.0
        // (the value an uncorrelated draw would converge to).
        assert_relative_eq!(empirical_cov, 0.9, epsilon = 0.15);
    }
}
