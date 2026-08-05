// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Invariant-boundary tests for [`moveit_kinematics::set_from_ik`],
//! [`moveit_kinematics::resolve_ik_queries`] and
//! [`moveit_kinematics::set_from_ik_subgroups`].
//!
//! The boundaries, each of which the port can land on either side of:
//!
//! - a target frame that *is* the solver's tip, against one that is only
//!   welded to it (`panda_hand`, one fixed 45° joint past `panda_link8`),
//!   against one on the far side of a moving joint (`panda_link6`), against
//!   one the model has never heard of;
//! - a frame the model resolves, against a frame only the caller's
//!   [`moveit_kinematics::AttachedFrames`] can;
//! - a solver tip a target claimed, against one no target claimed and that
//!   the fill therefore has to supply;
//! - a solver whose base frame is a link, against one whose base frame is
//!   the model frame — which, on this fixture, is `"world"`, a name the
//!   model has no link for, so the two branches differ by `Ok` against
//!   `Err`;
//! - the validity hook accepting against rejecting, and — the invariant that
//!   separates this port from upstream — the state after each;
//! - a group variable the solver writes, against a mimic variable it never
//!   does (`l_gripper_l_finger_tip_joint`), which is what makes "read the
//!   group back out of the state" different from "permute the solution".
//!
//! # Tolerances
//!
//! Every constant below was measured on these fixtures and printed before it
//! was written down; none is copied from another test or from a neighbouring
//! file. The margin over the measurement is stated at each one.
//!
//! Every solver here is built with `max_restarts: 0`, so each solve is one
//! deterministic Newton-Raphson attempt from the stated seed and no measured
//! residual can move because a random restart did or did not fire. The two
//! tests that need the retry loop itself say so.

use std::fs;

use moveit_error::Error;
use moveit_geometry::Isometry3;
use moveit_model::{JointModelGroup, MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;

use moveit_kinematics::{
    AttachedFrame, AttachedFrames, IkContext, IkTarget, KinematicsSolver, NewtonRaphsonSolver,
    NoAttachedFrames, SolveOptions, SolverParams, resolve_ik_queries, set_from_ik,
    set_from_ik_subgroups,
};

/// Distance between the query built by carrying a welded frame's pose across
/// to the solver's tip and the query built by naming that tip directly.
/// Measured `2.290814649885403e-16` m; this is ~44x that. The two are the
/// same rigid pose expressed two ways, so anything above rounding means the
/// carry used the wrong transform.
const CARRY_TOL_M: f64 = 1e-14;

/// Angle for the same comparison. Measured `1.1254018550399534e-16` rad;
/// ~89x that. The discrimination this leaves is wide: skipping the carry
/// entirely lands `7.853981633969999e-1` rad away — `panda_hand_joint`'s
/// fixed 45° — which is 13 orders of magnitude outside this bound.
const CARRY_TOL_RAD: f64 = 1e-14;

/// Distance from `panda_link8` to the pose [`set_from_ik`] was asked for.
/// Measured `5.446748248979708e-10` m naming the tip and
/// `5.446749311269383e-10` m naming `panda_hand`; this is ~18x the larger.
/// It stays well inside `SolverParams::epsilon` (`1e-5`), the loosest
/// residual the solver's own convergence contract permits.
const SOLVE_TOL_M: f64 = 1e-8;

/// Angle for the same comparison. Measured `5.177142069000985e-16` rad;
/// ~19x that.
const SOLVE_TOL_RAD: f64 = 1e-14;

/// Distance from each PR2 wrist to its own target after a two-subgroup
/// sweep. Measured `1.3948176833900462e-7` m on both arms — the two are
/// bit-identical, the arms being mirror images started from mirrored seeds
/// — and this is ~14x that. Two orders of magnitude looser than the panda
/// figure above and still two inside `SolverParams::epsilon`: the PR2 arm
/// is longer and its shoulder is offset from the base, so one
/// Newton-Raphson attempt lands further out.
const SUBGROUP_TOL_M: f64 = 2e-6;

/// Angle for the same comparison. Measured `8.025503781904034e-9` rad on
/// both arms; ~12x that.
const SUBGROUP_TOL_RAD: f64 = 1e-7;

/// The `panda_arm` configuration every panda test starts from. Chosen only
/// for being clear of the joint limits and singularities, so the single
/// permitted attempt converges.
const PANDA_START: [f64; 7] = [0.0, -0.4, 0.0, -1.9, 0.0, 1.6, 0.75];

/// The `left_arm` and `right_arm` configurations the PR2 tests start from,
/// mirrored across the robot for the same reason.
const PR2_LEFT_START: [f64; 7] = [0.5, 0.3, 0.0, -1.0, 0.0, -0.5, 0.0];
/// See [`PR2_LEFT_START`].
const PR2_RIGHT_START: [f64; 7] = [-0.5, 0.3, 0.0, -1.0, 0.0, -0.5, 0.0];

/// How far along +x each reachable target sits from the pose the start
/// configuration already holds. Small enough that one Newton-Raphson
/// attempt reaches it, large enough that the solve is not a no-op.
const REACHABLE_STEP: f64 = 0.05;

/// How far along +x the unreachable targets sit. Past `panda_arm`'s ~0.85 m
/// reach by more than 4 m, so no seed and no branch can reach it.
const UNREACHABLE_STEP: f64 = 5.0;

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

fn panda_model() -> RobotModel {
    build_model("panda.urdf", "panda.srdf")
}

fn pr2_model() -> RobotModel {
    build_model("pr2.urdf", "pr2.srdf")
}

/// One deterministic attempt per solve. See this file's `# Tolerances`.
fn one_attempt() -> SolverParams {
    SolverParams {
        max_restarts: 0,
        ..SolverParams::default()
    }
}

fn solver_for(model: &RobotModel, group: &str, params: &SolverParams) -> NewtonRaphsonSolver {
    NewtonRaphsonSolver::new(model, group, params)
        .unwrap_or_else(|e| panic!("{group} must be a solvable chain: {e}"))
}

/// A state with `group`'s active joints set to `values` and everything else
/// at its default.
fn state_at<'m>(model: &'m RobotModel, joint_names: &[String], values: &[f64]) -> RobotState<'m> {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    for (name, value) in joint_names.iter().zip(values) {
        state
            .set_variable_position(name, *value)
            .expect("fixture joint names are this model's variables");
    }
    state
}

fn panda_state(model: &RobotModel) -> RobotState<'_> {
    let names = solver_for(model, "panda_arm", &one_attempt())
        .joint_names()
        .to_vec();
    state_at(model, &names, &PANDA_START)
}

fn world_pose(state: &mut RobotState<'_>, link: &str) -> Isometry3 {
    state
        .update()
        .global_link_transform(link)
        .unwrap_or_else(|e| panic!("{link} must be a link: {e}"))
}

/// `pose` shifted `dx` metres along the model frame's +x.
fn shifted(pose: &Isometry3, dx: f64) -> Isometry3 {
    let mut moved = *pose;
    moved.translation.vector.x += dx;
    moved
}

fn translation_error(a: &Isometry3, b: &Isometry3) -> f64 {
    (a.translation.vector - b.translation.vector).norm()
}

fn rotation_error(a: &Isometry3, b: &Isometry3) -> f64 {
    a.rotation.angle_to(&b.rotation)
}

/// A [`NewtonRaphsonSolver`] with some of its *labels* replaced.
///
/// Every solve still goes to the real solver underneath; only what the
/// solver calls its group, its base frame and its tips changes. That is the
/// only way to put the port on the far side of three boundaries the panda
/// fixture's own solver always lands on the near side of: a solver with more
/// than one tip, a solver based on the model frame, and a solver whose joints
/// are not variables of the group it names.
struct Relabelled {
    inner: NewtonRaphsonSolver,
    group_name: String,
    base_frame: String,
    tip_frames: Vec<String>,
}

impl Relabelled {
    fn new(inner: NewtonRaphsonSolver) -> Self {
        Self {
            group_name: inner.group_name().to_owned(),
            base_frame: inner.base_frame().to_owned(),
            tip_frames: inner.tip_frames(),
            inner,
        }
    }

    fn with_group(mut self, group_name: &str) -> Self {
        self.group_name = group_name.to_owned();
        self
    }

    fn with_base(mut self, base_frame: &str) -> Self {
        self.base_frame = base_frame.to_owned();
        self
    }

    fn with_tips(mut self, tips: &[&str]) -> Self {
        self.tip_frames = tips.iter().map(|t| (*t).to_owned()).collect();
        self
    }
}

impl KinematicsSolver for Relabelled {
    fn group_name(&self) -> &str {
        &self.group_name
    }

    fn joint_names(&self) -> &[String] {
        self.inner.joint_names()
    }

    fn base_frame(&self) -> &str {
        &self.base_frame
    }

    fn tip_frame(&self) -> &str {
        &self.tip_frames[0]
    }

    fn tip_frames(&self) -> Vec<String> {
        self.tip_frames.clone()
    }

    fn solve_with_options(
        &mut self,
        seed: &[f64],
        target: &Isometry3,
        options: &mut SolveOptions,
    ) -> Option<Vec<f64>> {
        self.inner.solve_with_options(seed, target, options)
    }
}

/// One attached body, `"grasped_box"`, welded to `panda_hand`.
///
/// [`AttachedFrames`] draws no line between a body's own frame and one of its
/// subframes — upstream's `getLinkModelIncludingAttachedBodies` answers both
/// with the attached link, and both are rigid — so this one implementation
/// stands for both cases.
struct GraspedBox {
    hand_pose_box: Isometry3,
}

impl AttachedFrames for GraspedBox {
    fn attached_frame(&self, frame: &str) -> Option<AttachedFrame<'_>> {
        (frame == "grasped_box").then_some(AttachedFrame {
            link_name: "panda_hand",
            link_pose_frame: self.hand_pose_box,
        })
    }
}

// ---- resolve_ik_queries: matching a target to a tip ----------------------

#[test]
fn a_target_naming_the_tip_itself_is_only_moved_into_the_solver_base_frame() {
    let model = panda_model();
    let solver = solver_for(&model, "panda_arm", &one_attempt());
    let mut state = panda_state(&model);

    let target = shifted(&world_pose(&mut state, "panda_link8"), REACHABLE_STEP);
    let expected = world_pose(&mut state, "panda_link0").inverse() * target;

    let queries = resolve_ik_queries(
        &mut state,
        &solver,
        &[IkTarget {
            pose: target,
            frame: "panda_link8",
        }],
        &NoAttachedFrames,
    )
    .expect("panda_link8 is the solver's own tip");

    assert_eq!(queries.len(), 1, "one tip, one query");
    assert!(
        translation_error(&queries[0], &expected) <= CARRY_TOL_M
            && rotation_error(&queries[0], &expected) <= CARRY_TOL_RAD,
        "exact-name match must pass the pose through unchanged apart from the \
         base-frame multiply: got {:?}, expected {expected:?}",
        queries[0]
    );
}

#[test]
fn a_target_naming_a_welded_frame_is_carried_across_to_the_tip() {
    let model = panda_model();
    let solver = solver_for(&model, "panda_arm", &one_attempt());
    let mut state = panda_state(&model);

    // The same rigid motion, stated once about panda_hand and once about the
    // tip it is welded to. The two queries must agree.
    let hand_target = shifted(&world_pose(&mut state, "panda_hand"), REACHABLE_STEP);
    let tip_target = shifted(&world_pose(&mut state, "panda_link8"), REACHABLE_STEP);
    let base_pose_world = world_pose(&mut state, "panda_link0");

    let carried = resolve_ik_queries(
        &mut state,
        &solver,
        &[IkTarget {
            pose: hand_target,
            frame: "panda_hand",
        }],
        &NoAttachedFrames,
    )
    .expect("panda_hand is welded to panda_link8");

    let expected = base_pose_world.inverse() * tip_target;
    assert!(
        translation_error(&carried[0], &expected) <= CARRY_TOL_M
            && rotation_error(&carried[0], &expected) <= CARRY_TOL_RAD,
        "carrying panda_hand's goal to panda_link8 must land on the tip's own \
         goal: got {:?}, expected {expected:?}",
        carried[0]
    );

    // ... and the carry is not a no-op: handing the hand's pose straight to
    // the solver would be 45 degrees out, panda_hand_joint's fixed rotation.
    let uncarried = base_pose_world.inverse() * hand_target;
    assert!(
        rotation_error(&carried[0], &uncarried) > 0.7,
        "passing panda_hand's pose through unchanged must be visibly wrong, \
         else this test cannot tell the carry happened; got {} rad",
        rotation_error(&carried[0], &uncarried)
    );
}

#[test]
fn a_target_naming_a_frame_across_a_moving_joint_is_not_a_tip_match() {
    let model = panda_model();
    let solver = solver_for(&model, "panda_arm", &one_attempt());
    let mut state = panda_state(&model);

    // panda_link6's parent joint is revolute, so its rigid parent is itself,
    // while panda_link8's is panda_link7. Different parents, no match.
    let target = shifted(&world_pose(&mut state, "panda_link6"), REACHABLE_STEP);
    let error = resolve_ik_queries(
        &mut state,
        &solver,
        &[IkTarget {
            pose: target,
            frame: "panda_link6",
        }],
        &NoAttachedFrames,
    )
    .expect_err("panda_link6 is not rigidly connected to panda_link8");

    assert!(
        matches!(error, Error::Other(ref m) if m.contains("panda_link6")),
        "a frame that reaches no tip is a caller error naming that frame, got {error:?}"
    );
}

#[test]
fn a_target_naming_nothing_in_the_model_is_unknown_name_not_no_match() {
    let model = panda_model();
    let solver = solver_for(&model, "panda_arm", &one_attempt());
    let mut state = panda_state(&model);

    let error = resolve_ik_queries(
        &mut state,
        &solver,
        &[IkTarget {
            pose: Isometry3::identity(),
            frame: "no_such_frame",
        }],
        &NoAttachedFrames,
    )
    .expect_err("no_such_frame is neither a link nor attached");

    assert!(
        matches!(
            error,
            Error::UnknownName { kind: "IK frame", ref name } if name == "no_such_frame"
        ),
        "an unresolvable frame must be distinguishable from a resolvable one \
         that reaches no tip, got {error:?}"
    );
}

#[test]
fn an_attached_frame_reaches_the_tip_through_the_link_it_hangs_from() {
    let model = panda_model();
    let solver = solver_for(&model, "panda_arm", &one_attempt());
    let mut state = panda_state(&model);

    // A box held 8 cm in front of the hand and rolled 30 degrees, so neither
    // half of the attachment transform can be dropped without moving the
    // answer.
    let hand_pose_box = Isometry3::from_parts(
        nalgebra::Translation3::new(0.0, 0.0, 0.08),
        nalgebra::UnitQuaternion::from_euler_angles(std::f64::consts::FRAC_PI_6, 0.0, 0.0),
    );
    let attached = GraspedBox { hand_pose_box };

    let world_pose_box = world_pose(&mut state, "panda_hand") * hand_pose_box;
    let box_target = shifted(&world_pose_box, REACHABLE_STEP);
    let tip_target = shifted(&world_pose(&mut state, "panda_link8"), REACHABLE_STEP);
    let expected = world_pose(&mut state, "panda_link0").inverse() * tip_target;

    let queries = resolve_ik_queries(
        &mut state,
        &solver,
        &[IkTarget {
            pose: box_target,
            frame: "grasped_box",
        }],
        &attached,
    )
    .expect("grasped_box hangs off panda_hand, which is welded to the tip");

    assert!(
        translation_error(&queries[0], &expected) <= CARRY_TOL_M
            && rotation_error(&queries[0], &expected) <= CARRY_TOL_RAD,
        "an attached frame's goal must carry to the tip the same way a link's \
         does: got {:?}, expected {expected:?}",
        queries[0]
    );
}

#[test]
fn a_tip_a_target_already_claimed_cannot_be_matched_twice() {
    let model = panda_model();
    let solver = solver_for(&model, "panda_arm", &one_attempt());
    let mut state = panda_state(&model);

    let target = shifted(&world_pose(&mut state, "panda_link8"), REACHABLE_STEP);
    let error = resolve_ik_queries(
        &mut state,
        &solver,
        &[
            IkTarget {
                pose: target,
                frame: "panda_link8",
            },
            IkTarget {
                pose: target,
                frame: "panda_hand",
            },
        ],
        &NoAttachedFrames,
    )
    .expect_err("both targets want the one tip this solver has");

    assert!(
        matches!(error, Error::Other(ref m) if m.contains("IK target 1")),
        "the *second* target is the one with nowhere to go, got {error:?}"
    );
}

// ---- resolve_ik_queries: filling the tips no target named ----------------

#[test]
fn a_tip_no_target_named_is_filled_with_the_pose_it_currently_holds() {
    let model = panda_model();
    let solver = Relabelled::new(solver_for(&model, "panda_arm", &one_attempt()))
        .with_tips(&["panda_link8", "panda_hand"]);
    let mut state = panda_state(&model);

    let tip_target = shifted(&world_pose(&mut state, "panda_link8"), REACHABLE_STEP);
    let base_pose_world = world_pose(&mut state, "panda_link0");
    let hand_now = world_pose(&mut state, "panda_hand");

    let queries = resolve_ik_queries(
        &mut state,
        &solver,
        &[IkTarget {
            pose: tip_target,
            frame: "panda_link8",
        }],
        &NoAttachedFrames,
    )
    .expect("the first tip is named, the second is filled");

    assert_eq!(queries.len(), 2, "one query per tip, named or not");
    let expected_named = base_pose_world.inverse() * tip_target;
    let expected_filled = base_pose_world.inverse() * hand_now;
    assert!(
        translation_error(&queries[0], &expected_named) <= CARRY_TOL_M
            && rotation_error(&queries[0], &expected_named) <= CARRY_TOL_RAD,
        "the named tip keeps its own goal: got {:?}",
        queries[0]
    );
    assert!(
        translation_error(&queries[1], &expected_filled) <= CARRY_TOL_M
            && rotation_error(&queries[1], &expected_filled) <= CARRY_TOL_RAD,
        "the unnamed tip is asked to stay where it is: got {:?}, expected \
         {expected_filled:?}",
        queries[1]
    );
    // The fill is not the goal repeated: the two differ by the step this
    // test moved the named tip along, plus the welded 45 degrees.
    assert!(
        translation_error(&queries[0], &queries[1]) > REACHABLE_STEP / 2.0,
        "the filled query must be distinguishable from the named one"
    );
}

#[test]
fn naming_no_target_at_all_fills_every_tip_and_asks_the_solver_to_stay() {
    let model = panda_model();
    let solver = Relabelled::new(solver_for(&model, "panda_arm", &one_attempt()))
        .with_tips(&["panda_link8", "panda_hand"]);
    let mut state = panda_state(&model);

    let base_pose_world = world_pose(&mut state, "panda_link0");
    let expected: Vec<Isometry3> = ["panda_link8", "panda_hand"]
        .iter()
        .map(|link| base_pose_world.inverse() * world_pose(&mut state, link))
        .collect();

    let queries = resolve_ik_queries(&mut state, &solver, &[], &NoAttachedFrames)
        .expect("an empty target list is the fill loop's whole job");

    assert_eq!(queries.len(), 2);
    for (got, want) in queries.iter().zip(&expected) {
        assert!(
            translation_error(got, want) <= CARRY_TOL_M
                && rotation_error(got, want) <= CARRY_TOL_RAD,
            "got {got:?}, expected {want:?}"
        );
    }
}

// ---- to_solver_frame: the model-frame short circuit ----------------------

#[test]
fn a_solver_based_on_the_model_frame_needs_no_link_of_that_name() {
    let model = panda_model();
    assert_eq!(model.model_frame(), "world");
    assert!(
        !model.has_link_model("world"),
        "this test only discriminates while the model frame is not a link"
    );

    let solver = Relabelled::new(solver_for(&model, "panda_arm", &one_attempt()))
        .with_base(model.model_frame());
    let mut state = panda_state(&model);
    let target = shifted(&world_pose(&mut state, "panda_link8"), REACHABLE_STEP);

    let queries = resolve_ik_queries(
        &mut state,
        &solver,
        &[IkTarget {
            pose: target,
            frame: "panda_link8",
        }],
        &NoAttachedFrames,
    )
    .expect("the model frame needs no link lookup");

    assert!(
        translation_error(&queries[0], &target) <= CARRY_TOL_M
            && rotation_error(&queries[0], &target) <= CARRY_TOL_RAD,
        "a pose already in the model frame is a pose already in this solver's \
         base frame: got {:?}, expected {target:?}",
        queries[0]
    );
}

#[test]
fn a_solver_based_on_a_frame_the_model_does_not_have_is_unknown_name() {
    let model = panda_model();
    let solver =
        Relabelled::new(solver_for(&model, "panda_arm", &one_attempt())).with_base("no_such_base");
    let mut state = panda_state(&model);
    let target = shifted(&world_pose(&mut state, "panda_link8"), REACHABLE_STEP);

    let error = resolve_ik_queries(
        &mut state,
        &solver,
        &[IkTarget {
            pose: target,
            frame: "panda_link8",
        }],
        &NoAttachedFrames,
    )
    .expect_err("no_such_base is neither the model frame nor a link");

    assert!(
        matches!(error, Error::UnknownName { kind: "link", ref name } if name == "no_such_base"),
        "got {error:?}"
    );
}

// ---- set_from_ik: what reaches the state ---------------------------------

#[test]
fn a_reachable_target_leaves_its_own_solution_in_the_state() {
    let model = panda_model();
    let mut solver = solver_for(&model, "panda_arm", &one_attempt());
    let mut state = panda_state(&model);
    let entry = state.positions().to_vec();

    let target = shifted(&world_pose(&mut state, "panda_link8"), REACHABLE_STEP);
    let solved = set_from_ik(
        &mut state,
        &mut solver,
        &[IkTarget {
            pose: target,
            frame: "panda_link8",
        }],
        &mut IkContext::default(),
    )
    .expect("the request is well formed");

    assert!(solved, "a 5 cm step is inside panda_arm's workspace");
    let reached = world_pose(&mut state, "panda_link8");
    assert!(
        translation_error(&reached, &target) <= SOLVE_TOL_M
            && rotation_error(&reached, &target) <= SOLVE_TOL_RAD,
        "the state must hold the solution, not the seed: reached {reached:?}, \
         asked for {target:?}"
    );
    assert_ne!(
        state.positions(),
        entry.as_slice(),
        "the solve must have moved something"
    );
}

#[test]
fn a_welded_frames_target_is_solved_for_that_frame_not_for_the_tip() {
    let model = panda_model();
    let mut solver = solver_for(&model, "panda_arm", &one_attempt());
    let mut state = panda_state(&model);

    let target = shifted(&world_pose(&mut state, "panda_hand"), REACHABLE_STEP);
    let solved = set_from_ik(
        &mut state,
        &mut solver,
        &[IkTarget {
            pose: target,
            frame: "panda_hand",
        }],
        &mut IkContext::default(),
    )
    .expect("panda_hand is welded to the solver's tip");

    assert!(solved);
    let reached = world_pose(&mut state, "panda_hand");
    assert!(
        translation_error(&reached, &target) <= SOLVE_TOL_M
            && rotation_error(&reached, &target) <= SOLVE_TOL_RAD,
        "panda_hand, not panda_link8, is what was asked for: reached \
         {reached:?}, asked for {target:?}"
    );
}

#[test]
fn an_unreachable_target_leaves_the_state_exactly_as_it_found_it() {
    let model = panda_model();
    let mut solver = solver_for(&model, "panda_arm", &one_attempt());
    let mut state = panda_state(&model);
    let entry = state.positions().to_vec();

    let target = shifted(&world_pose(&mut state, "panda_link8"), UNREACHABLE_STEP);
    let solved = set_from_ik(
        &mut state,
        &mut solver,
        &[IkTarget {
            pose: target,
            frame: "panda_link8",
        }],
        &mut IkContext::default(),
    )
    .expect("an unreachable pose is a legitimate request, not an error");

    assert!(!solved, "5 m is far outside panda_arm's reach");
    assert_eq!(
        state.positions(),
        entry.as_slice(),
        "a failed solve must not leave a partial answer behind"
    );
}

#[test]
fn a_solver_reporting_two_tips_is_refused_rather_than_solved_for_the_first() {
    let model = panda_model();
    let mut solver = Relabelled::new(solver_for(&model, "panda_arm", &one_attempt()))
        .with_tips(&["panda_link8", "panda_hand"]);
    let mut state = panda_state(&model);
    let entry = state.positions().to_vec();

    let target = shifted(&world_pose(&mut state, "panda_link8"), REACHABLE_STEP);
    let error = set_from_ik(
        &mut state,
        &mut solver,
        &[IkTarget {
            pose: target,
            frame: "panda_link8",
        }],
        &mut IkContext::default(),
    )
    .expect_err("solve_with_options takes one pose");

    assert!(
        matches!(error, Error::Other(ref m) if m.contains("set_from_ik_subgroups")),
        "the error must point at the entry point that does take this, got {error:?}"
    );
    assert_eq!(state.positions(), entry.as_slice());
}

#[test]
fn a_solver_whose_joints_are_not_the_groups_variables_is_unknown_name() {
    let model = panda_model();
    // panda_arm's joints, but claiming to be the hand's solver.
    let mut solver =
        Relabelled::new(solver_for(&model, "panda_arm", &one_attempt())).with_group("hand");
    let mut state = panda_state(&model);

    let target = shifted(&world_pose(&mut state, "panda_link8"), REACHABLE_STEP);
    let error = set_from_ik(
        &mut state,
        &mut solver,
        &[IkTarget {
            pose: target,
            frame: "panda_link8",
        }],
        &mut IkContext::default(),
    )
    .expect_err("panda_joint1 is not a variable of the hand group");

    assert!(
        matches!(
            error,
            Error::UnknownName { kind: "group variable", ref name } if name == "panda_joint1"
        ),
        "got {error:?}"
    );
}

// ---- set_from_ik: the group-state validity hook --------------------------

#[test]
fn the_hook_sees_a_state_that_already_holds_the_candidate() {
    let model = panda_model();
    let mut solver = solver_for(&model, "panda_arm", &one_attempt());
    let mut state = panda_state(&model);
    let target = shifted(&world_pose(&mut state, "panda_link8"), REACHABLE_STEP);

    let mut tip_errors: Vec<f64> = Vec::new();
    let mut hook = |state: &mut RobotState<'_>, _group: &JointModelGroup, _values: &[f64]| {
        let reached = state
            .update()
            .global_link_transform("panda_link8")
            .expect("panda_link8 is a link");
        tip_errors.push(translation_error(&reached, &target));
        true
    };
    let solved = set_from_ik(
        &mut state,
        &mut solver,
        &[IkTarget {
            pose: target,
            frame: "panda_link8",
        }],
        &mut IkContext {
            attached: &NoAttachedFrames,
            consistency_limits: None,
            validity: Some(&mut hook),
        },
    )
    .expect("the request is well formed");

    assert!(solved);
    assert_eq!(tip_errors.len(), 1, "one converged attempt, one hook call");
    assert!(
        tip_errors[0] <= SOLVE_TOL_M,
        "forward kinematics inside the hook must already show the candidate; \
         the hook saw the tip {} m from the target",
        tip_errors[0]
    );
}

#[test]
fn the_hook_receives_every_group_variable_including_the_mimic_no_solver_writes() {
    let model = pr2_model();
    let group = model
        .joint_model_group("l_gripper_finger_chain")
        .expect("fixture group");
    let mut solver = solver_for(&model, "l_gripper_finger_chain", &one_attempt());
    assert_eq!(
        (solver.joint_names().len(), group.variable_names().len()),
        (1, 2),
        "this boundary only exists while the group has a variable the solver \
         does not"
    );

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    state
        .set_variable_position("l_gripper_l_finger_joint", 0.1)
        .expect("fixture joint");
    let target = world_pose(&mut state, "l_gripper_l_finger_tip_link");
    // Start somewhere else, so the mimic's entry value is not its answer.
    state
        .set_variable_position("l_gripper_l_finger_joint", 0.3)
        .expect("fixture joint");
    let entry_mimic = state
        .variable_position("l_gripper_l_finger_tip_joint")
        .expect("fixture joint");

    let mut seen: Vec<Vec<f64>> = Vec::new();
    let mut hook = |_state: &mut RobotState<'_>, _group: &JointModelGroup, values: &[f64]| {
        seen.push(values.to_vec());
        true
    };
    let solved = set_from_ik(
        &mut state,
        &mut solver,
        &[IkTarget {
            pose: target,
            frame: "l_gripper_l_finger_tip_link",
        }],
        &mut IkContext {
            attached: &NoAttachedFrames,
            consistency_limits: None,
            validity: Some(&mut hook),
        },
    )
    .expect("the request is well formed");

    assert!(solved);
    assert_eq!(seen.len(), 1);
    assert_eq!(
        seen[0].len(),
        2,
        "the hook's slice is group-variable-sized, not solver-joint-sized"
    );
    let mimic = seen[0][1];
    assert!(
        (mimic - entry_mimic).abs() > 0.1,
        "the mimic entry must be the value the write propagated ({}), not the \
         one the state happened to hold on entry ({entry_mimic})",
        seen[0][0]
    );
    assert!(
        (mimic - seen[0][0]).abs() <= CARRY_TOL_M,
        "l_gripper_l_finger_tip_joint mimics l_gripper_l_finger_joint one for \
         one; got {mimic} against {}",
        seen[0][0]
    );
}

#[test]
fn a_rejecting_hook_reports_no_solution_and_rewinds_what_it_wrote() {
    let model = panda_model();
    let mut solver = solver_for(&model, "panda_arm", &one_attempt());
    let mut state = panda_state(&model);
    let entry = state.positions().to_vec();
    let target = shifted(&world_pose(&mut state, "panda_link8"), REACHABLE_STEP);

    let mut calls = 0usize;
    let mut hook = |_state: &mut RobotState<'_>, _group: &JointModelGroup, _values: &[f64]| {
        calls += 1;
        false
    };
    let solved = set_from_ik(
        &mut state,
        &mut solver,
        &[IkTarget {
            pose: target,
            frame: "panda_link8",
        }],
        &mut IkContext {
            attached: &NoAttachedFrames,
            consistency_limits: None,
            validity: Some(&mut hook),
        },
    )
    .expect("the request is well formed");

    assert!(!solved, "a rejected candidate is not a solution");
    assert_eq!(
        calls, 1,
        "the one converged attempt was offered and refused"
    );
    assert_eq!(
        state.positions(),
        entry.as_slice(),
        "the hook is handed the candidate in the state, so a rejection has to \
         put the state back"
    );
}

#[test]
fn an_accepting_hooks_writes_outside_the_group_do_not_survive() {
    let model = panda_model();
    let mut solver = solver_for(&model, "panda_arm", &one_attempt());
    let mut state = panda_state(&model);
    let entry_finger = state
        .variable_position("panda_finger_joint1")
        .expect("fixture joint");
    let target = shifted(&world_pose(&mut state, "panda_link8"), REACHABLE_STEP);

    let mut hook = |state: &mut RobotState<'_>, _group: &JointModelGroup, _values: &[f64]| {
        state
            .set_variable_position("panda_finger_joint1", entry_finger + 0.02)
            .expect("fixture joint");
        true
    };
    let solved = set_from_ik(
        &mut state,
        &mut solver,
        &[IkTarget {
            pose: target,
            frame: "panda_link8",
        }],
        &mut IkContext {
            attached: &NoAttachedFrames,
            consistency_limits: None,
            validity: Some(&mut hook),
        },
    )
    .expect("the request is well formed");

    assert!(solved);
    assert_eq!(
        state
            .variable_position("panda_finger_joint1")
            .expect("fixture joint"),
        entry_finger,
        "on success the state holds the solution and nothing else the hook did"
    );
    let reached = world_pose(&mut state, "panda_link8");
    assert!(translation_error(&reached, &target) <= SOLVE_TOL_M);
}

// ---- set_from_ik_subgroups ----------------------------------------------

fn pr2_arms_state(model: &RobotModel) -> RobotState<'_> {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    let left = solver_for(model, "left_arm", &one_attempt())
        .joint_names()
        .to_vec();
    let right = solver_for(model, "right_arm", &one_attempt())
        .joint_names()
        .to_vec();
    for (name, value) in left.iter().zip(&PR2_LEFT_START) {
        state.set_variable_position(name, *value).expect("fixture");
    }
    for (name, value) in right.iter().zip(&PR2_RIGHT_START) {
        state.set_variable_position(name, *value).expect("fixture");
    }
    state
}

fn pr2_arm_solvers(model: &RobotModel) -> Vec<Box<dyn KinematicsSolver>> {
    vec![
        Box::new(solver_for(model, "left_arm", &one_attempt())),
        Box::new(solver_for(model, "right_arm", &one_attempt())),
    ]
}

#[test]
fn one_sweep_puts_both_subgroups_on_their_own_targets() {
    let model = pr2_model();
    let mut state = pr2_arms_state(&model);
    let mut solvers = pr2_arm_solvers(&model);

    let left = shifted(&world_pose(&mut state, "l_wrist_roll_link"), REACHABLE_STEP);
    let right = shifted(&world_pose(&mut state, "r_wrist_roll_link"), REACHABLE_STEP);
    let solved = set_from_ik_subgroups(
        &mut state,
        "arms",
        &mut solvers,
        &[
            IkTarget {
                pose: left,
                frame: "l_wrist_roll_link",
            },
            IkTarget {
                pose: right,
                frame: "r_wrist_roll_link",
            },
        ],
        &mut IkContext::default(),
        1,
    )
    .expect("the request is well formed");

    assert!(solved, "both arms can take a 5 cm step");
    for (link, want) in [("l_wrist_roll_link", left), ("r_wrist_roll_link", right)] {
        let reached = world_pose(&mut state, link);
        assert!(
            translation_error(&reached, &want) <= SUBGROUP_TOL_M
                && rotation_error(&reached, &want) <= SUBGROUP_TOL_RAD,
            "{link} reached {reached:?}, asked for {want:?}"
        );
    }
}

#[test]
fn a_subgroup_that_cannot_reach_rewinds_the_one_that_could() {
    let model = pr2_model();
    let mut state = pr2_arms_state(&model);
    let mut solvers = pr2_arm_solvers(&model);
    let entry = state.positions().to_vec();

    let left = shifted(&world_pose(&mut state, "l_wrist_roll_link"), REACHABLE_STEP);
    let right = shifted(
        &world_pose(&mut state, "r_wrist_roll_link"),
        UNREACHABLE_STEP,
    );
    let solved = set_from_ik_subgroups(
        &mut state,
        "arms",
        &mut solvers,
        &[
            IkTarget {
                pose: left,
                frame: "l_wrist_roll_link",
            },
            IkTarget {
                pose: right,
                frame: "r_wrist_roll_link",
            },
        ],
        &mut IkContext::default(),
        1,
    )
    .expect("the request is well formed");

    assert!(!solved);
    assert_eq!(
        state.positions(),
        entry.as_slice(),
        "the left arm solved first; a failure on the right must undo it"
    );
}

/// The other side of [`a_rejecting_group_hook_rewinds_every_sweep_it_refuses`]:
/// on *acceptance* the answer is the sweep's, not whatever the hook left in
/// the state on its way to saying yes.
#[test]
fn an_accepting_group_hooks_writes_do_not_survive_the_sweep() {
    let model = pr2_model();
    let mut state = pr2_arms_state(&model);
    let mut solvers = pr2_arm_solvers(&model);
    let entry_torso = state
        .variable_position("torso_lift_joint")
        .expect("fixture joint");

    let left = shifted(&world_pose(&mut state, "l_wrist_roll_link"), REACHABLE_STEP);
    let right = shifted(&world_pose(&mut state, "r_wrist_roll_link"), REACHABLE_STEP);

    let mut hook = |state: &mut RobotState<'_>, _group: &JointModelGroup, _values: &[f64]| {
        state
            .set_variable_position("torso_lift_joint", entry_torso + 0.05)
            .expect("fixture joint");
        true
    };
    let solved = set_from_ik_subgroups(
        &mut state,
        "arms",
        &mut solvers,
        &[
            IkTarget {
                pose: left,
                frame: "l_wrist_roll_link",
            },
            IkTarget {
                pose: right,
                frame: "r_wrist_roll_link",
            },
        ],
        &mut IkContext {
            attached: &NoAttachedFrames,
            consistency_limits: None,
            validity: Some(&mut hook),
        },
        1,
    )
    .expect("the request is well formed");

    assert!(solved);
    assert_eq!(
        state
            .variable_position("torso_lift_joint")
            .expect("fixture joint"),
        entry_torso,
        "torso_lift_joint is outside `arms`; the accepted sweep never set it, \
         so the hook's write must not be what the caller is left holding"
    );
    for (link, want) in [("l_wrist_roll_link", left), ("r_wrist_roll_link", right)] {
        let reached = world_pose(&mut state, link);
        assert!(
            translation_error(&reached, &want) <= SUBGROUP_TOL_M
                && rotation_error(&reached, &want) <= SUBGROUP_TOL_RAD,
            "{link} reached {reached:?}, asked for {want:?}"
        );
    }
}

#[test]
fn a_rejecting_group_hook_rewinds_every_sweep_it_refuses() {
    let model = pr2_model();
    let mut state = pr2_arms_state(&model);
    let mut solvers = pr2_arm_solvers(&model);
    let entry = state.positions().to_vec();

    let left = shifted(&world_pose(&mut state, "l_wrist_roll_link"), REACHABLE_STEP);
    let right = shifted(&world_pose(&mut state, "r_wrist_roll_link"), REACHABLE_STEP);

    let mut sweeps = 0usize;
    let mut hook = |_state: &mut RobotState<'_>, group: &JointModelGroup, values: &[f64]| {
        sweeps += 1;
        assert_eq!(
            values.len(),
            group.variable_names().len(),
            "the group hook judges the assembled group, not one subgroup"
        );
        false
    };
    let solved = set_from_ik_subgroups(
        &mut state,
        "arms",
        &mut solvers,
        &[
            IkTarget {
                pose: left,
                frame: "l_wrist_roll_link",
            },
            IkTarget {
                pose: right,
                frame: "r_wrist_roll_link",
            },
        ],
        &mut IkContext {
            attached: &NoAttachedFrames,
            consistency_limits: None,
            validity: Some(&mut hook),
        },
        2,
    )
    .expect("the request is well formed");

    assert!(!solved);
    assert_eq!(sweeps, 2, "both permitted sweeps solved and were refused");
    assert_eq!(
        state.positions(),
        entry.as_slice(),
        "a refused sweep must not be left in the state"
    );
}

#[test]
fn zero_attempts_solves_nothing_and_touches_nothing() {
    let model = pr2_model();
    let mut state = pr2_arms_state(&model);
    let mut solvers = pr2_arm_solvers(&model);
    let entry = state.positions().to_vec();

    let left = shifted(&world_pose(&mut state, "l_wrist_roll_link"), REACHABLE_STEP);
    let right = shifted(&world_pose(&mut state, "r_wrist_roll_link"), REACHABLE_STEP);
    let solved = set_from_ik_subgroups(
        &mut state,
        "arms",
        &mut solvers,
        &[
            IkTarget {
                pose: left,
                frame: "l_wrist_roll_link",
            },
            IkTarget {
                pose: right,
                frame: "r_wrist_roll_link",
            },
        ],
        &mut IkContext::default(),
        0,
    )
    .expect("the request is well formed");

    assert!(!solved, "no sweep ran, so nothing was solved");
    assert_eq!(state.positions(), entry.as_slice());
}

#[test]
fn no_subgroup_solvers_is_an_error_not_a_vacuous_success() {
    let model = pr2_model();
    let mut state = pr2_arms_state(&model);

    let error = set_from_ik_subgroups(
        &mut state,
        "arms",
        &mut [],
        &[],
        &mut IkContext::default(),
        1,
    )
    .expect_err("a sweep over no solvers would otherwise succeed at nothing");

    assert!(
        matches!(error, Error::Other(ref m) if m.contains("at least one")),
        "got {error:?}"
    );
}

#[test]
fn one_target_per_subgroup_solver_or_it_is_an_error() {
    let model = pr2_model();
    let mut state = pr2_arms_state(&model);
    let mut solvers = pr2_arm_solvers(&model);
    let left = shifted(&world_pose(&mut state, "l_wrist_roll_link"), REACHABLE_STEP);

    let error = set_from_ik_subgroups(
        &mut state,
        "arms",
        &mut solvers,
        &[IkTarget {
            pose: left,
            frame: "l_wrist_roll_link",
        }],
        &mut IkContext::default(),
        1,
    )
    .expect_err("two solvers, one target");

    assert!(
        matches!(error, Error::Other(ref m) if m.contains("2 subgroup solvers for 1 targets")),
        "got {error:?}"
    );
}

#[test]
fn a_solver_for_a_group_that_is_not_a_subgroup_is_refused() {
    let model = pr2_model();
    let mut state = pr2_arms_state(&model);
    // `base` is a group of this model, but not one of `arms`' subgroups.
    let mut solvers: Vec<Box<dyn KinematicsSolver>> = vec![Box::new(
        Relabelled::new(solver_for(&model, "left_arm", &one_attempt())).with_group("base"),
    )];
    let left = shifted(&world_pose(&mut state, "l_wrist_roll_link"), REACHABLE_STEP);

    let error = set_from_ik_subgroups(
        &mut state,
        "arms",
        &mut solvers,
        &[IkTarget {
            pose: left,
            frame: "l_wrist_roll_link",
        }],
        &mut IkContext::default(),
        1,
    )
    .expect_err("base is not a subgroup of arms");

    assert!(
        matches!(error, Error::Other(ref m) if m.contains("not a subgroup")),
        "got {error:?}"
    );
}
