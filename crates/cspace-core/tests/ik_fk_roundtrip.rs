// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Black-box `KinematicsSolver` invariant-boundary tests.
//!
//! Phase 4's own completion condition (see `PORTING-PLAN.md`) is
//! deliberately not solution equality against the C++ oracle -- IK
//! solutions are seed-dependent, so comparing them is meaningless. What is
//! self-contained and belongs here is the half that does not need the
//! oracle: for a target pose built by running forward kinematics on a
//! known-reachable joint configuration, solving IK back and re-running FK
//! on the solution lands within `1e-6` of that target *on each of the
//! targets below*.
//!
//! That `1e-6` is a measured property of these five targets, not a bound
//! the solver guarantees. What `cart_to_jnt` guarantees is
//! `SolverParams::epsilon` (`1e-5`): it returns the configuration whose own
//! `max(position_error, orientation_error)` it just measured at or under
//! that, so a converged solve is free to be anywhere in `(0, epsilon]` and
//! frequently is -- on 5,000 random `panda_arm` targets, 1,513 successful
//! solutions land in `(1e-6, 1e-5]` (PORTING-PLAN.md §221.2, which is why
//! §5's Phase 4 condition now names `epsilon` rather than `1e-6`). These
//! five clear `1e-6` with between 3x and 183x of room (measured: `1.2e-8`,
//! `1.86e-7`, `3.29e-7`, `5.46e-9`, `7.22e-9` in translation), so the
//! constant discriminates here without asserting a guarantee that does not
//! exist. Each test below
//! targets one boundary this crate's own doc comments call out as
//! consequential rather than a narrative "solve some pose" scenario:
//! bounded vs. continuous/unbounded joints, an independent chain vs. one
//! with a real mimic joint, full-pose vs. position-only convergence, and
//! construction on a non-chain group.

use std::fs;

use cspace_core::geometry::Isometry3;
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::{Posed, RobotState};

use cspace_core::kinematics::{
    KinematicsSolver, LevenbergMarquardtSolver, NewtonRaphsonSolver, SolverParams,
};

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

/// The target pose every `KinematicsSolver::solve` in this file is given:
/// the tip link's pose expressed in the chain's own base-link frame, i.e.
/// `root_pose_world * tip_pose_world` -- the same relation
/// `chain::ChainInfo::root_pose_world` computes internally, rebuilt here
/// from only public `RobotModel`/`Posed` API so this file never needs that
/// private type.
fn chain_relative_pose(model: &RobotModel, group_name: &str, posed: &Posed) -> Isometry3 {
    let group = model.joint_model_group(group_name).unwrap();
    let tip_name = group
        .link_names()
        .last()
        .expect("a chain group has at least one link");
    let tip_pose_world = posed.global_link_transform(tip_name).unwrap();

    let root_joint = group.joint_indices()[0];
    let root_link = model
        .link_models()
        .iter()
        .find(|l| l.parent_joint_index() == root_joint)
        .and_then(|l| l.parent_link_index());

    match root_link {
        Some(root_link) => {
            let root_pose_world = posed.global_link_transform_at(root_link);
            root_pose_world.inverse() * tip_pose_world
        }
        None => tip_pose_world,
    }
}

fn fk(model: &RobotModel, group_name: &str, joint_names: &[String], values: &[f64]) -> Isometry3 {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    for (name, &value) in joint_names.iter().zip(values) {
        state.set_variable_position(name, value).unwrap();
    }
    let posed = state.update();
    chain_relative_pose(model, group_name, &posed)
}

fn assert_within_1e_6(actual: &Isometry3, target: &Isometry3, context: &str) {
    let translation_error = (actual.translation.vector - target.translation.vector).norm();
    let rotation_error = (target.rotation.inverse() * actual.rotation).angle();
    assert!(
        translation_error < 1e-6,
        "{context}: translation error {translation_error:e} >= 1e-6"
    );
    assert!(
        rotation_error < 1e-6,
        "{context}: rotation error {rotation_error:e} rad >= 1e-6"
    );
}

fn assert_within_bounds(model: &RobotModel, joint_names: &[String], values: &[f64], context: &str) {
    for (name, &value) in joint_names.iter().zip(values) {
        let bounds = &model.joint_model(name).unwrap().variable_bounds()[0];
        assert!(
            value >= bounds.min_position - 1e-9 && value <= bounds.max_position + 1e-9,
            "{context}: {name} = {value} outside [{}, {}]",
            bounds.min_position,
            bounds.max_position
        );
    }
}

/// `panda_arm`: seven bounded revolute joints, no mimic. The baseline case
/// both solvers must handle -- run through both, since neither is a
/// strictly weaker version of the other (truncated-SVD vs. Tikhonov-damped
/// pseudo-inverse).
#[test]
fn panda_arm_bounded_chain_round_trips_through_both_solvers() {
    let model = build_model("panda.urdf", "panda.srdf");
    let params = SolverParams::default();

    let true_values = [0.3, -0.4, 0.2, -1.9, 0.1, 1.2, 0.5];

    let mut nr = NewtonRaphsonSolver::new(&model, "panda_arm", &params)
        .expect("panda_arm is a valid chain group");
    let joint_names = nr.joint_names().to_vec();
    assert_eq!(joint_names.len(), 7);

    let target = fk(&model, "panda_arm", &joint_names, &true_values);
    let seed = vec![0.0; joint_names.len()];

    let solution = nr
        .solve(&seed, &target)
        .expect("newton-raphson must converge from a bounded, reachable, mimic-free target");
    assert_within_bounds(&model, &joint_names, &solution, "newton-raphson solution");
    let solved_pose = fk(&model, "panda_arm", &joint_names, &solution);
    assert_within_1e_6(
        &solved_pose,
        &target,
        "newton-raphson FK(solution) vs target",
    );

    let mut lma = LevenbergMarquardtSolver::new(&model, "panda_arm", &params)
        .expect("panda_arm is a valid chain group");
    let solution = lma
        .solve(&seed, &target)
        .expect("lma must converge from a bounded, reachable, mimic-free target");
    assert_within_bounds(&model, &joint_names, &solution, "lma solution");
    let solved_pose = fk(&model, "panda_arm", &joint_names, &solution);
    assert_within_1e_6(&solved_pose, &target, "lma FK(solution) vs target");
}

/// `pr2`'s `right_arm` chain includes two `continuous` (URDF-unbounded)
/// revolute joints -- `r_forearm_roll_joint`, `r_wrist_roll_joint` -- whose
/// `VariableBounds` are still the finite `[-pi, pi]` convention
/// (`chain::ChainInfo::build`'s doc comment: taken unconditionally, not
/// exempted). This is the boundary a bounded-only fixture like `panda_arm`
/// cannot exercise.
#[test]
fn pr2_right_arm_continuous_joints_round_trip() {
    let model = build_model("pr2.urdf", "pr2.srdf");
    let params = SolverParams::default();

    let mut nr = NewtonRaphsonSolver::new(&model, "right_arm", &params)
        .expect("right_arm is a valid chain group");
    let joint_names = nr.joint_names().to_vec();
    assert_eq!(joint_names.len(), 7);

    let true_values = [-0.5, 0.3, -1.0, -0.6, 2.0, -0.4, 1.5];
    let target = fk(&model, "right_arm", &joint_names, &true_values);
    let seed = vec![0.0; joint_names.len()];

    let solution = nr
        .solve(&seed, &target)
        .expect("newton-raphson must converge on a chain with continuous joints");
    assert_within_bounds(&model, &joint_names, &solution, "right_arm solution");
    let solved_pose = fk(&model, "right_arm", &joint_names, &solution);
    assert_within_1e_6(&solved_pose, &target, "right_arm FK(solution) vs target");
}

/// `pr2`'s `l_gripper_finger_chain` (this crate's own fixture addition --
/// see `tests/fixtures/pr2.srdf`'s comment) has one active joint,
/// `l_gripper_l_finger_joint`, and one real mimic,
/// `l_gripper_l_finger_tip_joint` (`multiplier = 1.0`, `offset = 0.0`,
/// straight off the PR2 URDF). If the mimic fold silently dropped the
/// mimic joint's own contribution to the tip's motion (the exact defect
/// `chain::ChainInfo::full_jacobian`'s doc comment documents and
/// `velocity::tests::pr2_gripper_mimic_column_folds_into_its_masters_column_not_its_own`
/// catches directly), this end-to-end solve would still be free to
/// converge -- Newton's step-halving backoff does not require an exactly
/// correct Jacobian to eventually find a root of a smooth 1-DOF-active
/// problem. What it cannot do is land the *tip*, whose position is now
/// jointly driven by both the active and the mimic joint, within `1e-6` of
/// a target that was only reachable by moving both together.
#[test]
fn pr2_gripper_mimic_chain_round_trips() {
    let model = build_model("pr2.urdf", "pr2.srdf");
    let params = SolverParams::default();

    let mut nr = NewtonRaphsonSolver::new(&model, "l_gripper_finger_chain", &params)
        .expect("l_gripper_finger_chain is a valid chain group");
    let joint_names = nr.joint_names().to_vec();
    assert_eq!(
        joint_names,
        vec!["l_gripper_l_finger_joint".to_owned()],
        "the mimic joint must not appear in the reduced-space solver API"
    );

    let true_values = [0.4];
    let target = fk(&model, "l_gripper_finger_chain", &joint_names, &true_values);
    let seed = vec![0.05];

    let solution = nr
        .solve(&seed, &target)
        .expect("newton-raphson must converge on the real pr2 mimic chain");
    assert_within_bounds(&model, &joint_names, &solution, "mimic chain solution");
    let solved_pose = fk(&model, "l_gripper_finger_chain", &joint_names, &solution);
    assert_within_1e_6(&solved_pose, &target, "mimic chain FK(solution) vs target");
}

/// `SolverParams::position_only`: a target's position component alone must
/// be met to `1e-6`, without requiring the orientation error term to reach
/// `epsilon` as well -- `cart_to_jnt`'s convergence check folds in
/// `orientation_weight()`, which `position_only` forces to `0.0`.
#[test]
fn position_only_mode_converges_on_a_reachable_target_position() {
    let model = build_model("panda.urdf", "panda.srdf");
    let params = SolverParams {
        position_only: true,
        ..Default::default()
    };

    let mut nr = NewtonRaphsonSolver::new(&model, "panda_arm", &params)
        .expect("panda_arm is a valid chain group");
    let joint_names = nr.joint_names().to_vec();

    let true_values = [0.2, 0.1, -0.3, -1.5, 0.4, 1.0, -0.2];
    let target = fk(&model, "panda_arm", &joint_names, &true_values);
    let seed = vec![0.0; joint_names.len()];

    let solution = nr
        .solve(&seed, &target)
        .expect("position-only ik must converge on a reachable position");
    assert_within_bounds(&model, &joint_names, &solution, "position-only solution");

    let solved_pose = fk(&model, "panda_arm", &joint_names, &solution);
    let translation_error = (solved_pose.translation.vector - target.translation.vector).norm();
    assert!(
        translation_error < 1e-6,
        "position-only translation error {translation_error:e} >= 1e-6"
    );
}

/// `panda`'s `hand` group fails `is_chain()` (see
/// `cspace-state`'s own `panda_hand_group_has_one_root_but_fails_the_adjacency_check`)
/// -- `ChainInfo::build`, and therefore every solver constructor, must
/// surface that as a construction error rather than building a solver that
/// can never converge.
#[test]
fn constructing_a_solver_on_a_non_chain_group_is_an_error() {
    let model = build_model("panda.urdf", "panda.srdf");
    let params = SolverParams::default();
    let err = match NewtonRaphsonSolver::new(&model, "hand", &params) {
        Ok(_) => panic!("hand is not a chain group; construction must fail"),
        Err(e) => e,
    };
    assert!(err.to_string().contains("not a chain"), "got: {err}");
}
