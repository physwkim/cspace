// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Invariant-boundary tests for `impl cspace_core::kinematics::AttachedFrames for
//! cspace_scene::PlanningScene`, the seam that lets `setFromIK` name a frame
//! this crate owns.
//!
//! Upstream needs no seam: `RobotState::setFromIK` resolves its target frame
//! through `getRigidlyConnectedParentLinkModel` (`robot_state.cpp:1924`,
//! `:1931`) and `getFrameTransform` (`:1930`, `:1937`), which reach
//! `attached_body_map_` through `getLinkModelIncludingAttachedBodies`
//! (`:910-937`) and `getFrameInfo` (`:1338-1384`) — both members of the
//! state itself. In this workspace attached bodies live on
//! [`PlanningScene`], so this impl *is* that tier for `set_from_ik`, and
//! nothing else supplies it.
//!
//! # Boundaries
//!
//! The impl answers with three things — resolve-or-miss, `link_pose_frame`,
//! `link_name` — and each test below pins exactly one of them, so that each
//! has a mutation of the impl that fails it alone:
//!
//! | test | mutation that fails it, and only it |
//! | --- | --- |
//! | [`a_bare_attached_body_id_resolves_with_no_local_offset`] | return [`None`] for a name without `/` |
//! | [`an_attached_subframe_carries_the_pose_the_scene_stores`] | drop the rotation from `link_pose_frame` |
//! | [`a_name_no_attached_body_owns_is_not_an_attached_frame`] | answer a miss with the attach link and identity |
//! | [`the_attach_link_is_the_one_the_rigid_parent_match_reads`] | return `panda_link0` as `link_name` |
//! | [`the_same_target_without_the_seam_is_an_unknown_ik_frame`] | make `NoAttachedFrames` resolve |
//!
//! The last two are a pair: the fourth shows the seam makes an attached
//! subframe a usable IK target, and the fifth shows the identical call
//! without the seam does not resolve — without it, the fourth could be
//! passing on something `set_from_ik` already did.
//!
//! # Tolerances
//!
//! Every constant below was measured on this fixture and printed before it
//! was written down. The solver is built with `max_restarts: 0`, so each
//! solve is one deterministic Newton-Raphson attempt from [`PANDA_START`].

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::FRAC_PI_6;
use std::fs;
use std::sync::Arc;

use cspace_core::error::Error;
use cspace_core::geometry::{Cuboid, Isometry3, Shape};
use cspace_core::kinematics::{
    AttachedFrame, AttachedFrames, IkContext, IkTarget, KinematicsSolver, NewtonRaphsonSolver,
    SolverParams, set_from_ik,
};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_scene::PlanningScene;
use nalgebra::{Translation3, UnitQuaternion, Vector3};

/// Error allowed when a pose this impl reports is compared against the pose
/// the scene stores, or against the pose [`PlanningScene::frame_transform`]
/// reports for the same string. Both sides run the same multiply on the same
/// operands, so every such comparison here measures `0` m and `0` rad
/// exactly. Held just off exact zero so a future reassociation of the
/// product cannot turn a real agreement into a failure, and ten orders of
/// magnitude below [`SUBFRAME_OFFSET_M`]'s smallest component (`0.01` m),
/// eleven below [`SUBFRAME_YAW_RAD`] (`0.524` rad), so no dropped or
/// transposed part of the stored pose can hide under it.
const STORED_POSE_TOL: f64 = 1e-12;

/// Distance from the attached subframe to the pose `set_from_ik` was asked
/// for. Measured `5.446756e-10` m; this is 18x that, and still three decades
/// under `SolverParams::epsilon` (`1e-5`), the loosest residual the solver's
/// own convergence contract permits — so it fails a solve that converged on
/// the wrong frame rather than one that merely converged loosely.
const SOLVE_TOL_M: f64 = 1e-8;

/// The `panda_arm` configuration every solve here starts from. Clear of the
/// joint limits and of the outstretched singularity, so the single permitted
/// attempt converges.
const PANDA_START: [f64; 7] = [0.0, -0.4, 0.0, -1.9, 0.0, 1.6, 0.75];

/// How far along +x the reachable target sits from the pose the start
/// configuration already holds.
const REACHABLE_STEP_M: f64 = 0.05;

/// The grasped box's subframe translation in `panda_hand`'s frame.
/// Non-identity in all three axes, with three distinct magnitudes, so that a
/// dropped offset and a transposed one both show up.
const SUBFRAME_OFFSET_M: [f64; 3] = [0.01, 0.02, 0.13];

/// The grasped box's subframe rotation about +z, in `panda_hand`'s frame.
/// Present because it is the one part of the stored pose that moves no
/// frame origin: it separates "the stored pose is carried" from "the solve
/// reached the right place", which a translation-only offset cannot.
const SUBFRAME_YAW_RAD: f64 = FRAC_PI_6;

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        file_name
    )
}

fn panda_srdf() -> SrdfModel {
    let srdf_path = fixture_path("panda.srdf");
    SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse")
}

fn panda_model(srdf: &SrdfModel) -> RobotModel {
    let urdf_path = fixture_path("panda.urdf");
    let urdf_xml =
        fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

fn one_attempt() -> SolverParams {
    SolverParams {
        max_restarts: 0,
        ..SolverParams::default()
    }
}

fn panda_arm_solver(model: &RobotModel) -> NewtonRaphsonSolver {
    NewtonRaphsonSolver::new(model, "panda_arm", &one_attempt())
        .expect("panda_arm must be a solvable chain")
}

/// The pose stored for `grasped_box/grip` in `panda_hand`'s frame.
fn subframe_pose() -> Isometry3 {
    Isometry3::from_parts(
        Translation3::new(
            SUBFRAME_OFFSET_M[0],
            SUBFRAME_OFFSET_M[1],
            SUBFRAME_OFFSET_M[2],
        ),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), SUBFRAME_YAW_RAD),
    )
}

/// A panda scene at [`PANDA_START`] carrying one box welded to `panda_hand`,
/// with one subframe at [`subframe_pose`].
///
/// `panda_hand` reaches `panda_arm`'s tip `panda_link8` through the fixed
/// `panda_hand_joint`, so every frame of this box shares
/// `getRigidlyConnectedParentLinkModel` with that tip — which is the branch
/// of `set_from_ik`'s tip matching these tests reach.
fn scene_holding_a_box<'m>(model: &'m RobotModel, srdf: &SrdfModel) -> PlanningScene<'m> {
    let mut scene = PlanningScene::new(model, srdf);
    let arm_variables = panda_arm_solver(model).joint_names().to_vec();
    let state = scene.current_state_mut();
    state.set_to_default_values();
    for (name, value) in arm_variables.iter().zip(PANDA_START) {
        state
            .set_variable_position(name, value)
            .expect("a solver joint name is a variable of this model");
    }

    scene
        .attach_new(
            "grasped_box",
            "panda_hand",
            vec![Arc::new(Shape::Cuboid(
                Cuboid::new(0.04, 0.04, 0.04).expect("extents are non-negative"),
            ))],
            vec![Isometry3::identity()],
            BTreeSet::new(),
            BTreeMap::from([("grip".to_owned(), subframe_pose())]),
        )
        .expect("panda_hand is a link of this model");
    scene
}

fn translation_error(a: &Isometry3, b: &Isometry3) -> f64 {
    (a.translation.vector - b.translation.vector).norm()
}

fn rotation_error(a: &Isometry3, b: &Isometry3) -> f64 {
    a.rotation.angle_to(&b.rotation)
}

fn seam_answer<'s>(scene: &'s PlanningScene<'_>, frame: &str) -> AttachedFrame<'s> {
    AttachedFrames::attached_frame(scene, frame)
        .unwrap_or_else(|| panic!("{frame} must resolve as an attached frame"))
}

#[test]
fn a_bare_attached_body_id_resolves_with_no_local_offset() {
    let srdf = panda_srdf();
    let model = panda_model(&srdf);
    let scene = scene_holding_a_box(&model, &srdf);

    // Upstream reads `AttachedBody::getPose()` here; this port stores no
    // body-local pose at all (see `AttachedBody`'s module doc), so the whole
    // content of the id tier is that it resolves, at identity.
    let answer = seam_answer(&scene, "grasped_box");
    assert!(
        translation_error(&answer.link_pose_frame, &Isometry3::identity()) <= STORED_POSE_TOL
            && rotation_error(&answer.link_pose_frame, &Isometry3::identity()) <= STORED_POSE_TOL,
        "a body id names the attach link's own frame, not an offset from it: \
         got {:?}",
        answer.link_pose_frame
    );
}

#[test]
fn an_attached_subframe_carries_the_pose_the_scene_stores() {
    let srdf = panda_srdf();
    let model = panda_model(&srdf);
    let scene = scene_holding_a_box(&model, &srdf);

    let answer = seam_answer(&scene, "grasped_box/grip");
    let stored = subframe_pose();
    assert!(
        translation_error(&answer.link_pose_frame, &stored) <= STORED_POSE_TOL,
        "the subframe's stored translation must reach the seam intact: got \
         {:?}, stored {stored:?}",
        answer.link_pose_frame
    );
    assert!(
        rotation_error(&answer.link_pose_frame, &stored) <= STORED_POSE_TOL,
        "the subframe's stored rotation must reach the seam intact: got \
         {:?}, stored {stored:?}",
        answer.link_pose_frame
    );
}

#[test]
fn a_name_no_attached_body_owns_is_not_an_attached_frame() {
    let srdf = panda_srdf();
    let model = panda_model(&srdf);
    let scene = scene_holding_a_box(&model, &srdf);

    assert!(
        AttachedFrames::attached_frame(&scene, "no_such_body").is_none(),
        "a name outside the attached set must miss, so set_from_ik reports \
         UnknownName instead of resolving it somewhere"
    );
    assert!(
        AttachedFrames::attached_frame(&scene, "grasped_box/no_such_subframe").is_none(),
        "a real body with an unreal subframe must miss too, so the id prefix \
         alone cannot stand in for a subframe"
    );
}

#[test]
fn the_attach_link_is_the_one_the_rigid_parent_match_reads() {
    let srdf = panda_srdf();
    let model = panda_model(&srdf);
    let mut scene = scene_holding_a_box(&model, &srdf);
    let mut solver = panda_arm_solver(&model);

    // `link_name` is not interchangeable with any other link that would
    // recompose to the same global pose: `set_from_ik` feeds it to
    // `getRigidlyConnectedParentLinkModel`, which reads the link and not the
    // pose, and matches the target against the solver's tip on that alone.
    assert_eq!(
        seam_answer(&scene, "grasped_box/grip").link_name,
        "panda_hand",
        "the seam must name the link the body is attached to"
    );

    let mut target = scene
        .frame_transform("grasped_box/grip")
        .expect("the scene resolves the subframe");
    target.translation.vector.x += REACHABLE_STEP_M;

    let seam = SceneFrames::snapshot(&scene);
    let solved = set_from_ik(
        scene.current_state_mut(),
        &mut solver,
        &[IkTarget {
            pose: target,
            frame: "grasped_box/grip",
        }],
        &mut IkContext {
            attached: &seam,
            consistency_limits: None,
            validity: None,
        },
    )
    .expect("grasped_box/grip is rigidly connected to panda_arm's tip");
    assert!(solved, "the shifted subframe target must be reachable");

    let reached = scene
        .frame_transform("grasped_box/grip")
        .expect("the scene resolves the subframe");
    assert!(
        translation_error(&reached, &target) <= SOLVE_TOL_M,
        "the solve must put the subframe on the target, not the tip: reached \
         {reached:?}, asked for {target:?}"
    );
}

#[test]
fn the_same_target_without_the_seam_is_an_unknown_ik_frame() {
    let srdf = panda_srdf();
    let model = panda_model(&srdf);
    let mut scene = scene_holding_a_box(&model, &srdf);
    let mut solver = panda_arm_solver(&model);

    let mut target = scene
        .frame_transform("grasped_box/grip")
        .expect("the scene resolves the subframe");
    target.translation.vector.x += REACHABLE_STEP_M;

    let error = set_from_ik(
        scene.current_state_mut(),
        &mut solver,
        &[IkTarget {
            pose: target,
            frame: "grasped_box/grip",
        }],
        &mut IkContext::default(),
    )
    .expect_err("IkContext::default() carries NoAttachedFrames");

    assert!(
        matches!(
            error,
            Error::UnknownName { kind: "IK frame", ref name } if name == "grasped_box/grip"
        ),
        "got {error:?}"
    );
}

/// This impl's answers for every frame the scene currently owns, taken once
/// so the IK call can hold the state's exclusive borrow.
///
/// Deliberately not a second resolver: every entry comes from
/// [`AttachedFrames::attached_frame`] on the scene itself, so a test that
/// goes through this still exercises the impl under test rather than a
/// hand-rolled stand-in. The only thing it adds is the lifetime split. A
/// name the seam misses is left out rather than panicked on, so that a miss
/// surfaces as `set_from_ik`'s own [`Error::UnknownName`] at the frame that
/// needed it, not as a failure of whatever else the scene happens to carry.
struct SceneFrames(Vec<(String, String, Isometry3)>);

impl SceneFrames {
    fn snapshot(scene: &PlanningScene<'_>) -> Self {
        let mut names: Vec<String> = Vec::new();
        for body in scene.attached_bodies() {
            names.push(body.id().to_owned());
            names.extend(
                body.subframe_names()
                    .map(|sub| format!("{}/{sub}", body.id())),
            );
        }
        Self(
            names
                .into_iter()
                .filter_map(|name| {
                    let answer = AttachedFrames::attached_frame(scene, &name)?;
                    Some((name, answer.link_name.to_owned(), answer.link_pose_frame))
                })
                .collect(),
        )
    }
}

impl AttachedFrames for SceneFrames {
    fn attached_frame(&self, frame: &str) -> Option<AttachedFrame<'_>> {
        self.0
            .iter()
            .find_map(|(name, link_name, link_pose_frame)| {
                (name == frame).then_some(AttachedFrame {
                    link_name,
                    link_pose_frame: *link_pose_frame,
                })
            })
    }
}
