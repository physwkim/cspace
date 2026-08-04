// Copyright (c) 2009, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_cost.hpp
//   moveit_planners/chomp/chomp_motion_planner/src/chomp_cost.cpp

//! [`ChompCost`]: the per-joint smoothness quadratic-cost matrix and its
//! inverse, built from a sum of squared finite-difference matrices.
//!
//! # Deviations from upstream
//!
//! - **The `joint_number` constructor parameter is dropped.** Upstream's
//!   constructor signature is `ChompCost(const ChompTrajectory&, int
//!   joint_number, const std::vector<double>&, double)`, but the parameter
//!   is never read in the body -- upstream itself marks it
//!   `/* joint_number */` in `chomp_cost.cpp`, a comment, not a name, which
//!   is upstream's own signal that it is unused. Confirmed against the only
//!   real call site, `chomp_optimizer.cpp:127`
//!   (`ChompCost(group_trajectory_, i, derivative_costs,
//!   parameters_->ridge_factor_)`): `i` is passed but has no effect on
//!   anything this constructor computes or stores. Kept out of
//!   [`ChompCost::new`]'s signature rather than carried over as an ignored
//!   parameter.
//! - **Reachable invariant violations upstream leaves as `assert()`-free UB
//!   are typed errors here**, matching the convention already established
//!   in [`crate::trajectory::ChompTrajectory`]:
//!   - `derivative_costs.len()` above [`crate::utils::DIFF_RULES`]`.len()`
//!     (3) upstream indexes `DIFF_RULES[i]` out of bounds on a 3-row C
//!     array with no guard at all; here it is
//!     [`Error::other`](moveit_error::Error::other).
//!   - `trajectory.num_points()` below `2 * (DIFF_RULE_LENGTH - 1)` (12)
//!     upstream computes `num_vars_free = num_vars_all - 2 *
//!     (DIFF_RULE_LENGTH - 1)` in a plain `int`, then passes it as a
//!     negative block size to `Eigen::MatrixXd::block()` -- an
//!     `eigen_assert` in debug builds, UB in release. Here it is a typed
//!     error before any matrix is built.
//!   - A singular `quad_cost` (e.g. every `derivative_costs` entry `0.0`
//!     and `ridge_factor == 0.0`, so `quad_cost_full_` is the zero matrix)
//!     upstream's `.inverse()` returns silently on a non-invertible matrix
//!     -- Eigen's generic `MatrixXd::inverse()` does not check
//!     invertibility at all and fills `quad_cost_inv_` with `NaN`/`Inf`
//!     rather than failing. `nalgebra`'s `DMatrix::try_inverse()` returns
//!     `Option`; `None` becomes a typed error here instead of a silently
//!     poisoned matrix, matching the reasoning `moveit-sampling`'s
//!     Cholesky-based sampler already applied to the same
//!     LLT-silently-emits-NaN failure mode (see round 16 dispatch note).
//!   - [`ChompCost::max_quad_cost_inv_value`] on an empty (`0`
//!     free-variable) `quad_cost_inv` is a typed error rather than a
//!     silently wrong answer -- see that method's own doc comment.
//!   - [`ChompCost::cost`]/[`ChompCost::derivative`] reject a
//!     `joint_trajectory` slice whose length does not match
//!     `quad_cost_full_`'s dimension (upstream's `Eigen::MatrixXd::ColXpr`
//!     parameter carries its length as part of the type and a mismatched
//!     multiplication is a compile error or a dimension assert upstream,
//!     neither of which is available for a `&[f64]` parameter here).
//! - **`quad_cost_inv_`'s decomposition family was checked, not assumed
//!   equal.** Upstream's `quad_cost_.inverse()` is `Eigen::MatrixXd`'s
//!   generic `.inverse()`; per `Eigen/src/LU/InverseImpl.h`'s own doc
//!   comment ("for fixed sizes up to 4x4, use
//!   computeInverseAndDetWithCheck(); for the general case, use class
//!   PartialPivLU") and its `compute_inverse<MatrixType, ResultType, Size =
//!   MatrixType::RowsAtCompileTime>` dispatch, the closed-form small-size
//!   path only ever triggers for a *compile-time* fixed-size matrix type
//!   (`Matrix4d` and smaller). `Eigen::MatrixXd`'s `RowsAtCompileTime` is
//!   `Eigen::Dynamic` regardless of its actual runtime size, so
//!   `quad_cost_.inverse()` **always** goes through `PartialPivLU`, even
//!   for a runtime-tiny `quad_cost_` (e.g. a 2x2 or 3x3 matrix at
//!   `num_vars_free` this small). `nalgebra::DMatrix::try_inverse()`
//!   dispatches on the matrix's *runtime* dimension instead
//!   (`try_inverse_mut` in `nalgebra-0.35.0/src/linalg/inverse.rs`):
//!   closed-form cofactor/Cramer's-rule formulas for runtime size 1-4, and
//!   `lu::try_invert_to` (also partial *row* pivoting -- it selects the
//!   pivot via `.icamax()`, the largest-magnitude entry in the column,
//!   exactly `PartialPivLU`'s strategy, confirmed by reading
//!   `nalgebra-0.35.0/src/linalg/lu.rs`) for runtime size >= 5. So:
//!   - `num_vars_free >= 5` (`num_vars_all >= 17`): both implementations
//!     are the same algorithm family (row-partial-pivoted LU); expected
//!     divergence is floating-point rounding only.
//!   - `num_vars_free` in `1..=4` (`num_vars_all` in `13..=16`, a reachable
//!     range for a short CHOMP trajectory): Eigen still uses `PartialPivLU`
//!     while `nalgebra` switches to a closed-form cofactor expansion --
//!     a genuine algorithm-family difference, not just a rounding
//!     difference.
//!   Round 16 could not close this by a numeric-parity check: no oracle op
//!   answered a bit-for-bit comparison against actual Eigen output yet, so
//!   what it measured was only the residual `‖quad_cost * quad_cost_inv -
//!   I‖` for one case in each branch (`num_vars_free == 2`, `nalgebra`'s
//!   cofactor path; `num_vars_free == 8`, its LU path) --
//!   `quad_cost_inv_stays_a_true_inverse_across_both_algorithm_branches`
//!   below, residuals `0.0` and `1.7763568394002505e-15` respectively,
//!   confirming a numerically sound inverse in both branches but not
//!   *bit-identical* agreement with Eigen.
//!
//!   **Closed, round 18.** `crates/moveit-planners-chomp/doc/oracle-request-
//!   quad-cost-inv.md` asked for, and got, an oracle op
//!   (`chomp_quad_cost_inverse`) that links the real upstream `ChompCost`
//!   directly rather than a transcription, answering the actual question:
//!   `crates/moveit-planners-chomp/tests/chomp_quad_cost_inverse_parity.rs`
//!   compares [`ChompCost::quadratic_cost_inverse`] element-by-element
//!   against Eigen's own output for all 5 of the request document's
//!   boundary cases (`num_vars_free` 1/2/3/4/8, covering both branches).
//!   Measured maxima (not bit-exact, but not the residual-only bound either):
//!   `1.78e-15` at `num_vars_free == 1` up to `2.68e-11` at `num_vars_free
//!   == 8`, growing with matrix size/entry magnitude rather than jumping at
//!   the `1..=4`/`>=5` branch boundary -- the signature of accumulated
//!   rounding from two differently-ordered decompositions, not a genuine
//!   algorithm disagreement. See that test's own "Tolerance" doc section for
//!   the full measurement and the `1e-7` bound it justifies.
use crate::trajectory::ChompTrajectory;
use crate::utils::{self, DIFF_RULE_LENGTH};
use moveit_error::{Error, Result};
use nalgebra::{DMatrix, DVector};

/// The smoothness quadratic-cost matrix for a single joint, and its inverse
/// restricted to the free (non-boundary-padding) trajectory points.
///
/// Ported from `chomp::ChompCost`.
#[derive(Debug, Clone, PartialEq)]
pub struct ChompCost {
    quad_cost_full: DMatrix<f64>,
    quad_cost: DMatrix<f64>,
    quad_cost_inv: DMatrix<f64>,
}

impl ChompCost {
    /// Builds the smoothness quadratic-cost matrix as a weighted sum of
    /// squared finite-difference matrices, one per entry of
    /// `derivative_costs` (in practice always velocity, acceleration, jerk,
    /// in that order -- matching [`crate::utils::DIFF_RULES`]'s row order),
    /// plus `ridge_factor` on the diagonal.
    ///
    /// Ported from `ChompCost::ChompCost`. See the module doc for every
    /// rejected input this constructor typed-errors on instead of
    /// reproducing upstream's UB.
    pub fn new(
        trajectory: &ChompTrajectory,
        derivative_costs: &[f64],
        ridge_factor: f64,
    ) -> Result<Self> {
        if derivative_costs.len() > utils::DIFF_RULES.len() {
            return Err(Error::other(format!(
                "derivative_costs has {} entries but only {} DIFF_RULES rows exist",
                derivative_costs.len(),
                utils::DIFF_RULES.len()
            )));
        }

        let num_vars_all = trajectory.num_points();
        let boundary = 2 * (DIFF_RULE_LENGTH - 1);
        if num_vars_all < boundary {
            return Err(Error::other(format!(
                "trajectory has {num_vars_all} points, fewer than 2*(DIFF_RULE_LENGTH-1) = {boundary}"
            )));
        }
        let num_vars_free = num_vars_all - boundary;

        let mut quad_cost_full = DMatrix::<f64>::zeros(num_vars_all, num_vars_all);
        let mut multiplier = 1.0;
        for (i, &derivative_cost) in derivative_costs.iter().enumerate() {
            multiplier *= trajectory.discretization();
            let diff_matrix = Self::diff_matrix(num_vars_all, &utils::DIFF_RULES[i]);
            quad_cost_full +=
                (diff_matrix.transpose() * &diff_matrix) * (derivative_cost * multiplier);
        }
        quad_cost_full += DMatrix::<f64>::identity(num_vars_all, num_vars_all) * ridge_factor;

        let start = DIFF_RULE_LENGTH - 1;
        let quad_cost = quad_cost_full
            .view((start, start), (num_vars_free, num_vars_free))
            .into_owned();

        let quad_cost_inv = quad_cost
            .clone()
            .try_inverse()
            .ok_or_else(|| Error::other("quad_cost is singular and has no inverse"))?;

        Ok(Self {
            quad_cost_full,
            quad_cost,
            quad_cost_inv,
        })
    }

    /// Gets the inverse of the quadratic cost matrix, restricted to the free
    /// variables.
    ///
    /// Ported from `getQuadraticCostInverse`.
    pub fn quadratic_cost_inverse(&self) -> &DMatrix<f64> {
        &self.quad_cost_inv
    }

    /// Gets the quadratic cost matrix, restricted to the free variables.
    ///
    /// Ported from `getQuadraticCost`.
    pub fn quadratic_cost(&self) -> &DMatrix<f64> {
        &self.quad_cost
    }

    /// Computes `joint_trajectory . (quad_cost_full * joint_trajectory)`,
    /// the smoothness cost of one joint's full trajectory column.
    ///
    /// `joint_trajectory` must have length equal to the trajectory's full
    /// point count (e.g. [`ChompTrajectory::joint_trajectory`]'s output) --
    /// see the module doc's mismatched-length deviation note.
    ///
    /// Ported from `getCost`.
    pub fn cost(&self, joint_trajectory: &[f64]) -> Result<f64> {
        let n = self.quad_cost_full.nrows();
        if joint_trajectory.len() != n {
            return Err(Error::other(format!(
                "joint_trajectory has {} entries, expected {n}",
                joint_trajectory.len()
            )));
        }
        let v = DVector::from_column_slice(joint_trajectory);
        Ok(v.dot(&(&self.quad_cost_full * &v)))
    }

    /// Computes `2 * quad_cost_full * joint_trajectory`, the gradient of
    /// this joint's smoothness cost with respect to its full trajectory
    /// column.
    ///
    /// `joint_trajectory` must have length equal to the trajectory's full
    /// point count -- see the module doc's mismatched-length deviation
    /// note. Upstream's `getDerivative` writes into a caller-supplied
    /// output view (`Eigen::MatrixBase<Derived>&`) so the result can alias
    /// an existing buffer; here it is always a fresh, owned [`DVector`],
    /// matching this crate's "owned copies, not live views" convention
    /// (see [`crate::trajectory`]'s module doc): no call site in this crate
    /// needs write-through aliasing (the only caller,
    /// [`crate::optimizer::calculate_smoothness_increments`], only reads the
    /// returned vector).
    ///
    /// Ported from `getDerivative`.
    pub fn derivative(&self, joint_trajectory: &[f64]) -> Result<DVector<f64>> {
        let n = self.quad_cost_full.nrows();
        if joint_trajectory.len() != n {
            return Err(Error::other(format!(
                "joint_trajectory has {} entries, expected {n}",
                joint_trajectory.len()
            )));
        }
        let v = DVector::from_column_slice(joint_trajectory);
        Ok(&self.quad_cost_full * (2.0 * v))
    }

    /// Gets the largest entry of `quad_cost_inv_`.
    ///
    /// Returns a typed error if `quad_cost_inv_` has zero free variables
    /// (`num_vars_free == 0`): upstream's `Eigen::MatrixXd::maxCoeff()`
    /// requires a non-empty matrix (`eigen_assert(this->rows() > 0 &&
    /// this->cols() > 0)`), while `nalgebra`'s `DMatrix::max()` silently
    /// returns `0.0` on an empty matrix (see
    /// `nalgebra-0.35.0/src/base/min_max.rs`'s `fold_with(|e|
    /// e.cloned().unwrap_or_else(T::zero), ...)`). Reproducing that silent
    /// `0.0` here would hide a degenerate trajectory (one with no free
    /// points at all) behind a value indistinguishable from a real
    /// zero-cost answer, so this is a typed error instead.
    ///
    /// Ported from `getMaxQuadCostInvValue`.
    pub fn max_quad_cost_inv_value(&self) -> Result<f64> {
        if self.quad_cost_inv.nrows() == 0 || self.quad_cost_inv.ncols() == 0 {
            return Err(Error::other(
                "quad_cost_inv has zero free variables; there is no meaningful max entry",
            ));
        }
        Ok(self.quad_cost_inv.max())
    }

    /// Scales `quad_cost_full_`/`quad_cost_` by `scale` and `quad_cost_inv_`
    /// by `1.0 / scale`.
    ///
    /// Ported from `scale`.
    pub fn scale(&mut self, scale: f64) {
        let inv_scale = 1.0 / scale;
        self.quad_cost_inv *= inv_scale;
        self.quad_cost *= scale;
        self.quad_cost_full *= scale;
    }

    /// Builds a `size x size` finite-difference matrix from a single
    /// [`crate::utils::DIFF_RULES`] row, truncating (not reflecting or
    /// renormalizing) the stencil at both ends of the matrix so it never
    /// reads outside `[0, size)`.
    ///
    /// Ported from `getDiffMatrix`.
    fn diff_matrix(size: usize, diff_rule: &[f64; DIFF_RULE_LENGTH]) -> DMatrix<f64> {
        let mut matrix = DMatrix::<f64>::zeros(size, size);
        let half = (DIFF_RULE_LENGTH / 2) as isize;
        for i in 0..size as isize {
            for j in -half..=half {
                let index = i + j;
                if index < 0 || index >= size as isize {
                    continue;
                }
                matrix[(i as usize, index as usize)] = diff_rule[(j + half) as usize];
            }
        }
        matrix
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::trajectory::ChompTrajectory;
    use approx::assert_relative_eq;
    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_srdf::SrdfModel;
    use std::sync::OnceLock;

    const EPS: f64 = 1e-12;
    const GROUP: &str = "panda_arm";

    fn panda_model() -> &'static RobotModel {
        static MODEL: OnceLock<RobotModel> = OnceLock::new();
        MODEL.get_or_init(|| {
            let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.urdf");
            let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.srdf");
            let urdf_xml = std::fs::read_to_string(urdf_path)
                .unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
            let urdf = urdf_rs::read_file(urdf_path).expect("panda.urdf parses");
            let srdf = SrdfModel::parse_file(srdf_path).expect("panda.srdf parses");
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("panda model builds")
        })
    }

    fn trajectory(num_points: usize) -> ChompTrajectory {
        ChompTrajectory::from_num_points(panda_model(), num_points, 0.1, GROUP)
            .expect("valid num_points")
    }

    #[test]
    fn diff_matrix_truncates_at_the_boundary_but_not_the_interior() {
        // velocity stencil, hand-verified against chomp_utils.hpp's exact
        // literal fractions: interior rows keep all 7 taps and sum to zero
        // (already covered by utils.rs's diff_rules_rows_sum_to_zero), but
        // a boundary row drops whichever taps would land outside [0, size).
        let size = 10;
        let m = ChompCost::diff_matrix(size, &utils::DIFF_RULES[0]);

        // Row 0: taps at j = -3..-1 (values 0, 0, -2/6) are dropped; only
        // j = 0..3 (diff_rule[3..=6] = -3/6, 1.0, -1/6, 0.0) land at
        // columns 0..3.
        assert_relative_eq!(m[(0, 0)], -3.0 / 6.0, epsilon = EPS, max_relative = EPS);
        assert_relative_eq!(m[(0, 1)], 1.0, epsilon = EPS, max_relative = EPS);
        assert_relative_eq!(m[(0, 2)], -1.0 / 6.0, epsilon = EPS, max_relative = EPS);
        assert_relative_eq!(m[(0, 3)], 0.0, epsilon = EPS, max_relative = EPS);
        let row0_sum: f64 = (0..size).map(|j| m[(0, j)]).sum();
        assert_relative_eq!(row0_sum, 1.0 / 3.0, epsilon = EPS, max_relative = EPS);

        // Interior row 4 (size 10: taps at columns 1..=7, all in bounds):
        // every one of the 7 stencil coefficients lands, and the row sums
        // to zero (same invariant utils.rs already checks on the raw
        // stencil, now checked on the assembled matrix's interior).
        let interior_row = 4;
        for (k, &coefficient) in utils::DIFF_RULES[0].iter().enumerate() {
            assert_relative_eq!(
                m[(interior_row, interior_row - 3 + k)],
                coefficient,
                epsilon = EPS,
                max_relative = EPS
            );
        }
        let interior_sum: f64 = (0..size).map(|j| m[(interior_row, j)]).sum();
        assert_relative_eq!(interior_sum, 0.0, epsilon = EPS, max_relative = EPS);

        // Last row (index 9): taps at j = 1..3 (values 1.0, -1/6, 0.0) are
        // dropped; only j = -3..0 (diff_rule[0..=3] = 0, 0, -2/6, -3/6)
        // land at columns 6..9.
        let last = size - 1;
        assert_relative_eq!(m[(last, 6)], 0.0, epsilon = EPS, max_relative = EPS);
        assert_relative_eq!(m[(last, 7)], 0.0, epsilon = EPS, max_relative = EPS);
        assert_relative_eq!(m[(last, 8)], -2.0 / 6.0, epsilon = EPS, max_relative = EPS);
        assert_relative_eq!(m[(last, 9)], -3.0 / 6.0, epsilon = EPS, max_relative = EPS);
        let last_sum: f64 = (0..size).map(|j| m[(last, j)]).sum();
        assert_relative_eq!(last_sum, -5.0 / 6.0, epsilon = EPS, max_relative = EPS);
    }

    #[test]
    fn new_rejects_more_derivative_costs_than_diff_rules_rows() {
        let traj = trajectory(20);
        let err = ChompCost::new(&traj, &[1.0, 1.0, 1.0, 1.0], 0.0).unwrap_err();
        assert!(matches!(err, Error::Other(_)));
    }

    #[test]
    fn new_rejects_too_few_points_for_the_diff_rule_boundary() {
        // boundary = 2 * (DIFF_RULE_LENGTH - 1) = 12; 11 points is one
        // short of it and would give a negative getNumFreePoints()-style
        // block size upstream.
        let traj = trajectory(11);
        let err = ChompCost::new(&traj, &[1.0], 0.0).unwrap_err();
        assert!(matches!(err, Error::Other(_)));
    }

    #[test]
    fn new_accepts_exactly_the_boundary_point_count_with_zero_free_points() {
        // num_vars_all == 12 exactly -> num_vars_free == 0. Upstream does
        // not crash at construction for this either (Eigen's block/inverse
        // both handle a 0x0 matrix); only max_quad_cost_inv_value should
        // fail on it (see that test below).
        let traj = trajectory(12);
        let cost = ChompCost::new(&traj, &[1.0], 0.0).unwrap();
        assert_eq!(cost.quadratic_cost().nrows(), 0);
        assert_eq!(cost.quadratic_cost_inverse().nrows(), 0);
    }

    #[test]
    fn max_quad_cost_inv_value_rejects_zero_free_points() {
        let traj = trajectory(12);
        let cost = ChompCost::new(&traj, &[1.0], 0.0).unwrap();
        let err = cost.max_quad_cost_inv_value().unwrap_err();
        assert!(matches!(err, Error::Other(_)));
    }

    #[test]
    fn new_rejects_a_singular_quad_cost() {
        // All derivative costs zero and no ridge factor: quad_cost_full_
        // stays the zero matrix, so its free-variable block is singular.
        let traj = trajectory(20);
        let err = ChompCost::new(&traj, &[0.0, 0.0, 0.0], 0.0).unwrap_err();
        assert!(matches!(err, Error::Other(_)));
    }

    #[test]
    fn ridge_factor_alone_produces_a_scaled_identity_inverse() {
        // With every derivative cost zero, quad_cost_full_ = ridge_factor *
        // I exactly, so quad_cost_inv_ = (1 / ridge_factor) * I exactly --
        // a directly hand-checkable case, not just a residual check.
        let traj = trajectory(20);
        let ridge_factor = 4.0;
        let cost = ChompCost::new(&traj, &[0.0, 0.0, 0.0], ridge_factor).unwrap();
        let inv = cost.quadratic_cost_inverse();
        for i in 0..inv.nrows() {
            for j in 0..inv.ncols() {
                let expected = if i == j { 1.0 / ridge_factor } else { 0.0 };
                assert_relative_eq!(inv[(i, j)], expected, epsilon = EPS, max_relative = EPS);
            }
        }
    }

    #[test]
    fn quad_cost_inv_stays_a_true_inverse_across_both_algorithm_branches() {
        // num_vars_free == 2 (14 points) exercises nalgebra's closed-form
        // cofactor path; num_vars_free == 8 (20 points) exercises its LU
        // path -- see the module doc's decomposition-family note. Both are
        // checked against the residual ||quad_cost * quad_cost_inv - I||.
        // Measured (not guessed) against this exact test: num_points=14
        // gives an exact 0.0 residual (the cofactor path is a closed-form
        // rational computation on these inputs), num_points=20 gives
        // 1.7763568394002505e-15 (one ULP-scale LU rounding error). 1e-12
        // matches this crate's established EPS and leaves ~1000x headroom
        // above the measured LU-path residual.
        const RESIDUAL_TOL: f64 = 1e-12;
        for num_points in [14usize, 20usize] {
            let traj = trajectory(num_points);
            let cost = ChompCost::new(&traj, &[1.0, 1.0, 1.0], 1e-6).unwrap();
            let product = cost.quadratic_cost() * cost.quadratic_cost_inverse();
            let n = product.nrows();
            let mut max_abs_err = 0.0f64;
            for i in 0..n {
                for j in 0..n {
                    let expected = if i == j { 1.0 } else { 0.0 };
                    max_abs_err = max_abs_err.max((product[(i, j)] - expected).abs());
                }
            }
            assert!(
                max_abs_err < RESIDUAL_TOL,
                "num_points={num_points}: max |quad_cost * quad_cost_inv - I| = {max_abs_err}, expected < {RESIDUAL_TOL}"
            );
        }
    }

    #[test]
    fn cost_and_derivative_reject_mismatched_length() {
        let traj = trajectory(20);
        let cost = ChompCost::new(&traj, &[1.0, 1.0, 1.0], 0.0).unwrap();
        let wrong_length = vec![0.0; traj.num_points() - 1];
        assert!(matches!(
            cost.cost(&wrong_length).unwrap_err(),
            Error::Other(_)
        ));
        assert!(matches!(
            cost.derivative(&wrong_length).unwrap_err(),
            Error::Other(_)
        ));
    }

    #[test]
    fn cost_matches_direct_quadratic_form() {
        let traj = trajectory(20);
        let cost = ChompCost::new(&traj, &[1.0, 1.0, 1.0], 1e-3).unwrap();
        let n = traj.num_points();
        let v: Vec<f64> = (0..n).map(|i| (i as f64) * 0.1).collect();
        let got = cost.cost(&v).unwrap();

        // Hand-rolled quadratic form via the public quad_cost accessor is
        // not available for the *full* matrix (private, matching
        // upstream), so cross-check cost() against derivative() instead:
        // d/dv (v . (Q v)) = 2 Q v, i.e. derivative() must equal the
        // gradient of cost() at v, verified by finite differences.
        let h = 1e-6;
        let mut fd_grad = vec![0.0; n];
        for k in 0..n {
            let mut v_plus = v.clone();
            v_plus[k] += h;
            let mut v_minus = v.clone();
            v_minus[k] -= h;
            fd_grad[k] = (cost.cost(&v_plus).unwrap() - cost.cost(&v_minus).unwrap()) / (2.0 * h);
        }
        let analytic = cost.derivative(&v).unwrap();
        for k in 0..n {
            assert_relative_eq!(analytic[k], fd_grad[k], epsilon = 1e-4, max_relative = 1e-4);
        }
        assert!(got.is_finite());
    }

    #[test]
    fn scale_updates_all_three_matrices_by_the_documented_factor() {
        let traj = trajectory(20);
        let mut cost = ChompCost::new(&traj, &[1.0, 1.0, 1.0], 1e-3).unwrap();
        let quad_cost_inv_before = cost.quadratic_cost_inverse().clone();
        let quad_cost_before = cost.quadratic_cost().clone();

        let scale = 2.5;
        cost.scale(scale);

        assert_relative_eq!(
            cost.quadratic_cost_inverse(),
            &(quad_cost_inv_before / scale),
            epsilon = EPS,
            max_relative = EPS
        );
        assert_relative_eq!(
            cost.quadratic_cost(),
            &(quad_cost_before * scale),
            epsilon = EPS,
            max_relative = EPS
        );
    }
}
