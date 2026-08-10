// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Regression coverage for `SolverParams::validate` (`params.rs`): the
//! three numeric fields it gates (`epsilon`, `svd_threshold`, `lma_lambda`)
//! each reach a division whose denominator can be an exact `0.0`, or a
//! convergence-guard comparison a NaN value slips through (every NaN
//! comparison is false), once the field itself is non-positive or NaN.
//!
//! Confirmed before this fix landed, on [`coincident_axes_model`]: with
//! `svd_threshold: -1.0` and no validation, `NewtonRaphsonSolver::solve`
//! returned `Some([NaN, NaN])` for a target this exact chain reaches
//! trivially with the default, valid `svd_threshold` -- a different,
//! corrupted answer handed back through the public API, not merely a
//! non-finite float observed mid-computation. `negative_svd_threshold` and
//! its siblings below assert the fix (construction now rejects the field
//! outright) rather than replaying that corrupted solve, because a
//! validated `SolverParams` can no longer reach `NewtonRaphsonSolver::solve`
//! at all -- there is no longer a `Some([NaN, NaN])` to observe through
//! this crate's public API.

use cspace_kinematics::{
    KinematicsSolver, LevenbergMarquardtSolver, NewtonRaphsonSolver, SolverParams,
};
use cspace_model::{MeshSearchPaths, RobotModel};
use cspace_srdf::SrdfModel;

/// Two revolute joints at the exact same origin, on the exact same axis:
/// `j1` and `j2` have an identical effect on `tip` (both rotate the same
/// downstream rigid body about the same axis through the same point), so
/// the position Jacobian's two columns are linearly dependent by
/// construction, for every `(q1, q2)` and every reachable target -- not a
/// contrived floating-point coincidence. The chain's true rank is 1, not 2,
/// so a real SVD of its Jacobian always carries one singular value at
/// (numerically) exactly zero. `NewtonRaphsonSolver`/`LevenbergMarquardtSolver`'s
/// pseudo-inverse is the only thing standing between that zero singular
/// value and a division by it.
const COINCIDENT_AXES_URDF: &str = r#"<?xml version="1.0"?>
<robot name="coincident_axes">
  <link name="root"/>
  <link name="mid"/>
  <link name="pivot"/>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="root"/>
    <child link="mid"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="mid"/>
    <child link="pivot"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <joint name="j3" type="fixed">
    <parent link="pivot"/>
    <child link="tip"/>
    <origin xyz="1 0 0"/>
  </joint>
</robot>
"#;

const COINCIDENT_AXES_SRDF: &str = r#"<?xml version="1.0"?>
<robot name="coincident_axes">
  <group name="chain">
    <chain base_link="root" tip_link="tip"/>
  </group>
</robot>
"#;

fn coincident_axes_model() -> RobotModel {
    let urdf = urdf_rs::read_from_string(COINCIDENT_AXES_URDF).expect("inline URDF must parse");
    let srdf = SrdfModel::parse_str(COINCIDENT_AXES_SRDF).expect("inline SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, COINCIDENT_AXES_URDF, &srdf, &MeshSearchPaths::none())
        .expect("inline model must build")
}

fn target_at(angle: f64) -> cspace_geometry::Isometry3 {
    cspace_geometry::Isometry3::from_parts(
        nalgebra::Translation3::new(angle.cos(), angle.sin(), 0.0),
        nalgebra::UnitQuaternion::from_axis_angle(&nalgebra::Vector3::z_axis(), angle),
    )
}

#[test]
fn negative_svd_threshold_is_rejected_at_construction() {
    let model = coincident_axes_model();
    let params = SolverParams {
        svd_threshold: -1.0,
        ..SolverParams::default()
    };
    let result = NewtonRaphsonSolver::new(&model, "chain", &params);
    assert!(
        result.is_err(),
        "a negative svd_threshold must be rejected at construction, not left to divide by the \
         chain's own exact-zero singular value"
    );
}

#[test]
fn nan_svd_threshold_is_rejected_at_construction() {
    let model = coincident_axes_model();
    let params = SolverParams {
        svd_threshold: f64::NAN,
        ..SolverParams::default()
    };
    assert!(NewtonRaphsonSolver::new(&model, "chain", &params).is_err());
}

#[test]
fn zero_epsilon_is_rejected_at_construction() {
    let model = coincident_axes_model();
    let params = SolverParams {
        epsilon: 0.0,
        ..SolverParams::default()
    };
    assert!(
        NewtonRaphsonSolver::new(&model, "chain", &params).is_err(),
        "epsilon <= 0.0 makes every cart_to_jnt convergence guard degenerate"
    );
}

#[test]
fn nan_epsilon_is_rejected_at_construction() {
    let model = coincident_axes_model();
    let params = SolverParams {
        epsilon: f64::NAN,
        ..SolverParams::default()
    };
    assert!(NewtonRaphsonSolver::new(&model, "chain", &params).is_err());
}

#[test]
fn zero_lma_lambda_is_rejected_at_construction() {
    let model = coincident_axes_model();
    let params = SolverParams {
        lma_lambda: 0.0,
        ..SolverParams::default()
    };
    assert!(
        LevenbergMarquardtSolver::new(&model, "chain", &params).is_err(),
        "lma_lambda <= 0.0 lets the chain's own exact-zero singular value divide 0.0 / 0.0"
    );
}

#[test]
fn negative_lma_lambda_is_rejected_at_construction() {
    let model = coincident_axes_model();
    let params = SolverParams {
        lma_lambda: -0.01,
        ..SolverParams::default()
    };
    assert!(LevenbergMarquardtSolver::new(&model, "chain", &params).is_err());
}

/// The fix must not reject the geometry itself -- only invalid parameters.
/// A rank-deficient chain is ordinary, reachable robot geometry (see this
/// file's module doc comment), and with the default, valid `SolverParams`,
/// both solvers must still converge to a real answer on it.
#[test]
fn both_solvers_still_converge_on_a_rank_deficient_chain_with_valid_params() {
    let model = coincident_axes_model();
    let angle = 0.6_f64;
    let target = target_at(angle);
    let seed = [0.0_f64, 0.0_f64];
    let params = SolverParams::default();

    let mut newton =
        NewtonRaphsonSolver::new(&model, "chain", &params).expect("valid params must construct");
    let newton_solution = newton
        .solve(&seed, &target)
        .expect("a reachable target must converge with a valid svd_threshold");
    assert!(newton_solution.iter().all(|q| q.is_finite()));
    assert!((newton_solution[0] + newton_solution[1] - angle).abs() < 1e-6);

    let mut lma = LevenbergMarquardtSolver::new(&model, "chain", &params)
        .expect("valid params must construct");
    let lma_solution = lma
        .solve(&seed, &target)
        .expect("a reachable target must converge with a valid lma_lambda");
    assert!(lma_solution.iter().all(|q| q.is_finite()));
    assert!((lma_solution[0] + lma_solution[1] - angle).abs() < 1e-6);
}
