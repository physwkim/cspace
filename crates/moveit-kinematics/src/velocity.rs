// Copyright (c) 2013, Sachin Chitta, Willow Garage
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_kinematics/kdl_kinematics_plugin/src/chainiksolver_vel_mimic_svd.cpp

use nalgebra::{DMatrix, DVector, SVD};

use crate::chain::ChainInfo;

/// `jacToJacReduced`: fold every full-space column into its reduced-space
/// (active-joint) column, scaled by [`ChainInfo::multiplier`] — `1.0` for an
/// active joint's own column, the mimic factor for a mimic's. A mimic's
/// [`ChainInfo::multiplier`]-scaled contribution accumulates into the same
/// column as its master's own (unscaled) contribution, exactly as upstream's
/// `result = vel1 + multiplier * vel2` accumulation does. (The mimic
/// *offset* has no part in this: differentiating `mimic_value = factor *
/// master_value + offset` against `master_value` leaves only `factor`.)
fn fold_jacobian(chain: &ChainInfo, jacobian_full: &DMatrix<f64>) -> DMatrix<f64> {
    let mut reduced = DMatrix::<f64>::zeros(jacobian_full.nrows(), chain.reduced_dimension());
    for i in 0..chain.dimension() {
        let column = chain.map_index[i];
        let factor = chain.multiplier[i];
        for row in 0..jacobian_full.nrows() {
            reduced[(row, column)] += factor * jacobian_full[(row, i)];
        }
    }
    reduced
}

/// The inverse of [`fold_jacobian`] for a velocity result: full-space entry
/// `i`'s own rate is its master's reduced-space rate scaled by
/// [`ChainInfo::multiplier`] — `qdot_out(i) = qdot_out_reduced[map_index[i]]
/// * multiplier[i]`, matching upstream's own expansion at the end of
/// `ChainIkSolverVelMimicSVD::CartToJnt`.
fn expand_to_full(chain: &ChainInfo, reduced: &DVector<f64>) -> DVector<f64> {
    DVector::from_fn(chain.dimension(), |i, _| {
        reduced[chain.map_index[i]] * chain.multiplier[i]
    })
}

/// `ChainIkSolverVelMimicSVD::CartToJnt` (the velocity step): given the
/// current full-space geometric Jacobian and a desired Cartesian twist, mimic
/// fold, weight, SVD-solve, unweight, and expand back to full space. `pinv`
/// turns one singular value (plus the largest singular value, for a relative
/// threshold) into its pseudo-inverse scalar — truncated for
/// [`crate::NewtonRaphsonSolver`], Tikhonov-damped for
/// [`crate::LevenbergMarquardtSolver`]; see those types' doc comments.
///
/// `twist` and `cartesian_weights` are both 6-vectors, rows 0-2 linear, rows
/// 3-5 angular (see [`crate::chain::ChainInfo::full_jacobian`]'s doc
/// comment for the row convention). `joint_weights` is reduced-space
/// (`chain.reduced_dimension()` entries, [`crate::chain::ChainInfo`]-order).
///
/// # Deviation from upstream: hand-rolled around `nalgebra::SVD`, not
/// `SVD::solve`
///
/// `nalgebra::linalg::SVD::solve` truncates by an *absolute* singular-value
/// threshold; upstream's `JacobiSVD::setThreshold(0.001)` (the default
/// [`crate::SolverParams::svd_threshold`] this port copies) is *relative* to
/// the largest singular value. Nothing in `nalgebra::SVD`'s public API
/// exposes a relative-threshold `solve`, and the two solvers below need
/// different `f(singular_value)` shapes anyway (truncation vs. damping), so
/// both go through this one shared reconstruction — `x = V * diag(f(s_i)) *
/// U^T * b` — instead.
pub(crate) fn solve_velocity(
    chain: &ChainInfo,
    jacobian_full: &DMatrix<f64>,
    twist: &DVector<f64>,
    cartesian_weights: &DVector<f64>,
    joint_weights: &[f64],
    pinv: impl Fn(f64, f64) -> f64,
) -> DVector<f64> {
    let jac_reduced = fold_jacobian(chain, jacobian_full);
    let rows = jac_reduced.nrows();
    let cols = jac_reduced.ncols();

    let mut jac_weighted = jac_reduced;
    for row in 0..rows {
        for col in 0..cols {
            jac_weighted[(row, col)] *= cartesian_weights[row] * joint_weights[col];
        }
    }
    let twist_weighted = DVector::from_fn(rows, |row, _| twist[row] * cartesian_weights[row]);

    let svd = SVD::new(jac_weighted, true, true);
    let u = svd
        .u
        .expect("SVD::new(_, compute_u: true, _) always fills u");
    let v_t = svd
        .v_t
        .expect("SVD::new(_, _, compute_v: true) always fills v_t");
    let singular_values = &svd.singular_values;
    let smax = singular_values.max();

    let k = singular_values.len();
    let u_t_b = u.transpose() * &twist_weighted;
    let scaled = DVector::from_fn(k, |i, _| pinv(singular_values[i], smax) * u_t_b[i]);
    let y = v_t.transpose() * scaled;

    let qdot_reduced = DVector::from_fn(cols, |col, _| y[col] * joint_weights[col]);
    expand_to_full(chain, &qdot_reduced)
}

#[cfg(test)]
mod tests {
    use std::fs;

    use moveit_model::{MeshSearchPaths, RobotModel};
    use moveit_srdf::SrdfModel;
    use moveit_state::RobotState;

    use super::*;

    fn fixture_path(file_name: &str) -> String {
        format!(
            concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
            file_name
        )
    }

    fn build_model(urdf_file: &str, srdf_file: &str) -> RobotModel {
        let urdf_path = fixture_path(urdf_file);
        let srdf_path = fixture_path(srdf_file);
        let urdf_xml =
            fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
        let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("fixture model must build")
    }

    /// `fold_jacobian`'s defining invariant, on a real (not synthetic) mimic
    /// chain: `pr2`'s `l_gripper_finger_chain` group is
    /// `l_gripper_l_finger_joint` (active) followed by
    /// `l_gripper_l_finger_tip_joint` (mimic, `multiplier = 1.0`). The
    /// mimic's own full-space column must be non-zero (see
    /// [`crate::chain::ChainInfo::full_jacobian`]'s doc comment for why
    /// this crate cannot reuse `Posed::jacobian`, which would leave it
    /// zero), and `fold_jacobian` must accumulate it, scaled by
    /// `multiplier`, into the *master's* reduced column rather than
    /// dropping it or folding it into a column of its own.
    #[test]
    fn pr2_gripper_mimic_column_folds_into_its_masters_column_not_its_own() {
        let model = build_model("pr2.urdf", "pr2.srdf");
        let chain =
            ChainInfo::build(&model, "l_gripper_finger_chain").expect("real pr2 mimic chain");
        assert_eq!(chain.dimension(), 2, "one active + one mimic joint");
        assert_eq!(chain.reduced_dimension(), 1, "only the master is active");
        assert_eq!(
            chain.multiplier[1], 1.0,
            "l_gripper_l_finger_tip_joint's real multiplier"
        );
        assert_eq!(
            chain.map_index[1], chain.map_index[0],
            "the mimic folds into its master's own column, not a column of its own"
        );

        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        state
            .set_variable_position("l_gripper_l_finger_joint", 0.3)
            .unwrap();
        let posed = state.update();

        let full = chain.full_jacobian(&posed);
        assert!(
            full.column(1).norm() > 1e-9,
            "the mimic's own column must be real, not the all-zero column \
             Posed::jacobian would leave for a joint outside active_joint_indices()"
        );

        let reduced = fold_jacobian(&chain, &full);
        let expected = full.column(0) + full.column(1);
        assert!(
            (reduced.column(0) - expected).norm() < 1e-12,
            "reduced column must be the sum of both full-space columns, not just the master's own"
        );
    }
}
