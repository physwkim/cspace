// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Used by moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf's
//   moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp
// (`KDLKinematicsPlugin::CartToJnt`'s `ik_solver.CartToJnt(...)` call, line
// 467 — the Newton iteration this crate ports as
// [`crate::kinematics::cart_to_jnt::cart_to_jnt`]). See the module doc's "Why this file
// stays BSD-3-Clause" section for `chainiksolver_vel_mimic_svd.{hpp,cpp}`,
// the LGPL-2.1-or-later source this file's velocity solve plays the role of
// instead of porting.

//! The velocity-IK step: mimic-fold a Jacobian, weight it, solve the
//! weighted least-squares problem by SVD, unweight, and expand the mimic
//! fold back out. Plays the role of upstream's
//! `ChainIkSolverVelMimicSVD::CartToJnt` (the weighted overload) —
//! `moveit_kinematics/kdl_kinematics_plugin/src/chainiksolver_vel_mimic_svd.cpp`,
//! moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf — but that file is
//! LGPL-2.1-or-later (`third_party/orocos_kinematics_dynamics/`
//! `chainiksolver_vel_mimic_svd.hpp`'s own header: `Copyright (C) 2007 Ruben
//! Smits`, `URL: http://www.orocos.org/kdl`, LGPL-2.1-or-later; modified for
//! mimic joints under `Copyright (C) 2013 Sachin Chitta, Willow Garage`,
//! inside that same LGPL file, so under the same license), heavier copyleft
//! than this workspace's BSD-3-Clause.
//!
//! # Why this file stays BSD-3-Clause
//!
//! Nothing below is transcribed from `chainiksolver_vel_mimic_svd.cpp`; each
//! piece is derived independently from its own mathematical definition:
//!
//! - [`reduction_matrix`]/[`fold_jacobian`]/[`expand_to_full`]: the mimic
//!   constraint `full_value = multiplier * reduced_value + offset` is
//!   ordinary affine-function differentiation (`offset` vanishes under
//!   `d/dt`), and representing "gather each reduced-space value into its
//!   full-space slot, scaled" as a matrix and folding/expanding by
//!   multiplying by it (on the right for a Jacobian's columns, on the left
//!   for a vector) is the standard linear-algebra way to apply the same
//!   linear map to two different objects.
//! - `solve_velocity`'s per-element weighting is the standard weighted
//!   least-squares reduction (multiply the rows/columns being weighted by
//!   the weights before solving the now-unweighted problem) — see that
//!   function's own `# Deviation` section for its `pinv`/SVD-reconstruction
//!   shape, which is independently justified there against
//!   `nalgebra::SVD`'s public API rather than against upstream's `Eigen`
//!   usage.
//!
//! What is reused from the LGPL source is *interface facts*: which method
//! this module's `solve_velocity` corresponds to (`CartToJnt`, named above,
//! for readers cross-referencing the two codebases — a pointer, not
//! expression), the 6-row linear/angular twist convention (an interface
//! fact of the `Twist` type every caller must already agree on to pass
//! valid data), and [`crate::kinematics::chain::ChainInfo`]'s own `map_index`/
//! `multiplier` field names, which are this crate's own vocabulary
//! (`chain.rs`, ported from moveit2's own BSD `kdl_kinematics_plugin.cpp`)
//! and not LGPL-file-derived at all.

use nalgebra::{DMatrix, DVector, SVD};

use crate::kinematics::chain::ChainInfo;

/// The linear map from reduced (active-joint) space to full (every joint,
/// mimic included) space implied by [`ChainInfo`]'s mimic table: column
/// `map_index[i]` of row `i` is `multiplier[i]`, every other entry `0.0`.
/// This is the differential of the mimic constraint itself —
/// `full_value[i] = multiplier[i] * reduced_value[map_index[i]] + offset[i]`
/// (an active joint is its own trivial mimic, `map_index[i] == i`,
/// `multiplier[i] == 1.0`) has `offset[i]` vanish under `d/dt`, leaving
/// exactly this matrix as `d(full)/d(reduced)`. [`fold_jacobian`] and
/// [`expand_to_full`] are then both just multiplication by it, on opposite
/// sides.
fn reduction_matrix(chain: &ChainInfo) -> DMatrix<f64> {
    let mut m = DMatrix::<f64>::zeros(chain.dimension(), chain.reduced_dimension());
    for i in 0..chain.dimension() {
        m[(i, chain.map_index[i])] = chain.multiplier[i];
    }
    m
}

/// Folds a full-space Jacobian into reduced space. Given `qdot_full =
/// M * qdot_reduced` ([`reduction_matrix`]'s defining identity) and
/// `twist = jacobian_full * qdot_full`, substituting gives `twist =
/// (jacobian_full * M) * qdot_reduced` — so `jacobian_full * M` is exactly
/// the Jacobian that maps reduced-space joint rates to the same twist.
fn fold_jacobian(chain: &ChainInfo, jacobian_full: &DMatrix<f64>) -> DMatrix<f64> {
    jacobian_full * reduction_matrix(chain)
}

/// The inverse of [`fold_jacobian`] for a velocity result: `qdot_full = M *
/// qdot_reduced`, [`reduction_matrix`]'s own defining identity, applied
/// directly.
fn expand_to_full(chain: &ChainInfo, reduced: &DVector<f64>) -> DVector<f64> {
    reduction_matrix(chain) * reduced
}

/// The velocity-IK step (see the module doc for which upstream method this
/// plays the role of): given the current full-space geometric Jacobian and
/// a desired Cartesian twist, mimic-fold, weight, SVD-solve the weighted
/// least-squares problem, unweight, and expand back to full space. `pinv`
/// turns one singular value (plus the largest singular value, for a relative
/// threshold) into its pseudo-inverse scalar — truncated for
/// [`crate::kinematics::NewtonRaphsonSolver`], Tikhonov-damped for
/// [`crate::kinematics::LevenbergMarquardtSolver`]; see those types' doc comments.
///
/// `twist` and `cartesian_weights` are both 6-vectors, rows 0-2 linear, rows
/// 3-5 angular (see [`crate::kinematics::chain::ChainInfo::full_jacobian`]'s doc
/// comment for the row convention). `joint_weights` is reduced-space
/// (`chain.reduced_dimension()` entries, [`crate::kinematics::chain::ChainInfo`]-order).
///
/// # Deviation from upstream: hand-rolled around `nalgebra::SVD`, not
/// `SVD::solve`
///
/// `nalgebra::linalg::SVD::solve` truncates by an *absolute* singular-value
/// threshold; upstream's `JacobiSVD::setThreshold(0.001)` (the default
/// [`crate::kinematics::SolverParams::svd_threshold`] this port copies) is *relative* to
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

    use crate::model::{MeshSearchPaths, RobotModel};
    use crate::srdf::SrdfModel;
    use crate::state::RobotState;

    use super::*;

    fn fixture_path(file_name: &str) -> String {
        format!(
            concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/kinematics/{}"),
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
    /// [`crate::kinematics::chain::ChainInfo::full_jacobian`]'s doc comment for why
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
