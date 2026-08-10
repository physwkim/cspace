// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `constraints` op for
//! `crate::utils`'s construction functions, plus pure-Rust boundary tests for
//! the update/merge functions the oracle has no direct endpoint for.
//!
//! Ground truth for every `oracle_*` test is the oracle's own response,
//! captured verbatim into `tests/fixtures/panda_constraints.json` by sending
//! the equivalent `constraints` request (same `joint_values`, same
//! constraint parameters this file builds through `cspace_planning::constraints::utils`)
//! at `moveit-rs/oracle:5188956fc433d046`. This doubles as the first oracle
//! parity check for [`cspace_planning::constraints::Constraint::decide`] itself
//! (`decide.rs`'s own module doc notes that check did not exist before the
//! oracle gained a `constraints` op), since every construction function here
//! is exercised only through `decide()`'s output, never by asserting on the
//! constructed value directly.
//!
//! `panda_constraints.json` itself carries only each case's `results`
//! (`satisfied`/`distance`), not the wire request that produced them -- the
//! `joint_values`/tolerances/poses live only in this file's own literals
//! (`s0`/`s1`/`s2`/`SB`, the `construct_goal_*` call arguments above). Unlike
//! `panda_is_state_valid.json`/`pr2_attached_collision.json`
//! (`cspace-scene`'s hand-built "cases" fixtures, whose summary fields are
//! *not* enough to reconstruct a wire request), every one of those inputs is
//! a plain `constraints`-op field, so `tests/fixtures/panda_constraints_request.json`/
//! `panda_constraints_response.json` reconstructs the 12 cases' wire requests
//! by hand from these literals and were confirmed byte-for-byte against
//! `panda_constraints.json`'s already-committed `results` before being
//! committed (see `tests/fixtures/oracle-models.json`'s `panda_constraints`
//! entry) -- this closes the `tools/ci/verify-fixture-replay.sh` gap without
//! changing what this file itself reads from, since the 12 cases exercise
//! several distinct `cspace_planning::constraints::utils` construction functions, not
//! one generic request shape a test could deserialize and dispatch through.

use std::fs;

use serde::Deserialize;

use cspace_core::error::Error;
use cspace_core::geometry::{Isometry3, Shape, Sphere, Transforms, UnitQuaternion, Vector3};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;

use cspace_planning::constraints::utils::{
    construct_goal_joint_constraints, construct_goal_orientation_constraints,
    construct_goal_pose_constraints, construct_goal_pose_constraints_box,
    construct_goal_position_constraints, count_individual_constraints, merge_constraints,
    resolve_orientation_constraint_frame, resolve_position_constraint_frame,
    update_joint_constraints, update_orientation_constraint, update_pose_constraint,
    update_position_constraint,
};
use cspace_planning::constraints::{
    Constraint, JointConstraint, KinematicConstraintSet, OrientationConstraint,
    OrientationTolerance, PositionConstraint,
};

const TOLERANCE: f64 = 1e-6;

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/constraints/{}"),
        file_name
    )
}

fn panda_model() -> RobotModel {
    let urdf_path = fixture_path("panda.urdf");
    let srdf_path = fixture_path("panda.srdf");
    let urdf_xml = fs::read_to_string(&urdf_path).expect("read panda.urdf");
    let urdf = urdf_rs::read_file(&urdf_path).expect("parse panda.urdf");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("parse panda.srdf");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("build panda model")
}

fn tf(model: &RobotModel) -> Transforms {
    Transforms::new(model.model_frame()).expect("model_frame is always a valid target frame")
}

/// Builds a state with every named variable set explicitly (no reliance on
/// `set_to_default_values`'s specific numbers, so every joint value in this
/// file is a literal both sides were sent).
fn state_with<'m>(model: &'m RobotModel, values: &[(&str, f64)]) -> RobotState<'m> {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    for (name, value) in values {
        state
            .set_variable_position(name, *value)
            .unwrap_or_else(|e| panic!("setting {name}: {e}"));
    }
    state
}

const PANDA_JOINTS: [&str; 7] = [
    "panda_joint1",
    "panda_joint2",
    "panda_joint3",
    "panda_joint4",
    "panda_joint5",
    "panda_joint6",
    "panda_joint7",
];

fn s0() -> [(&'static str, f64); 7] {
    PANDA_JOINTS.map(|n| (n, 0.0))
}

fn s1() -> [(&'static str, f64); 7] {
    PANDA_JOINTS.map(|n| (n, 0.05))
}

fn s2() -> [(&'static str, f64); 7] {
    PANDA_JOINTS.map(|n| (n, 0.5))
}

const SB: [(&str, f64); 2] = [("panda_joint2", -0.3), ("panda_joint4", -1.0)];

#[derive(Deserialize)]
struct ExpectedResult {
    satisfied: bool,
    distance: f64,
}

#[derive(Deserialize)]
struct ExpectedCase {
    id: u32,
    results: Vec<ExpectedResult>,
}

#[derive(Deserialize)]
struct ConstraintsFixture {
    cases: Vec<ExpectedCase>,
}

fn expected(id: u32) -> Vec<ExpectedResult> {
    let raw = fs::read_to_string(fixture_path("panda_constraints.json"))
        .expect("read panda_constraints.json");
    let fixture: ConstraintsFixture =
        serde_json::from_str(&raw).expect("parse panda_constraints.json");
    fixture
        .cases
        .into_iter()
        .find(|c| c.id == id)
        .unwrap_or_else(|| panic!("no fixture case with id {id}"))
        .results
}

fn assert_matches_oracle(
    actual: &[cspace_planning::constraints::ConstraintEvaluationResult],
    id: u32,
) {
    let expected = expected(id);
    assert_eq!(
        actual.len(),
        expected.len(),
        "case {id}: constraint count mismatch"
    );
    for (i, (a, e)) in actual.iter().zip(expected.iter()).enumerate() {
        assert_eq!(
            a.satisfied, e.satisfied,
            "case {id} result {i}: satisfied mismatch"
        );
        assert!(
            (a.distance - e.distance).abs() < TOLERANCE,
            "case {id} result {i}: distance {} vs oracle {}",
            a.distance,
            e.distance
        );
    }
}

fn goal_point_pose() -> Isometry3 {
    let mut pose = Isometry3::identity();
    pose.translation.vector = Vector3::new(0.4, 0.0, 0.6);
    pose
}

mod oracle_construct_goal_joint_constraints {
    use super::*;

    #[test]
    fn satisfied_at_goal() {
        let model = panda_model();
        let mut state = state_with(&model, &s0());
        let posed = state.update();
        let goal = construct_goal_joint_constraints(&model, &posed, "panda_arm", 0.1, 0.1).unwrap();
        assert_matches_oracle(&goal.decide_each(&posed), 1);
    }

    #[test]
    fn satisfied_within_tolerance() {
        let model = panda_model();
        let mut state_goal = state_with(&model, &s0());
        let goal =
            construct_goal_joint_constraints(&model, &state_goal.update(), "panda_arm", 0.1, 0.1)
                .unwrap();

        let mut state_probe = state_with(&model, &s1());
        let posed_probe = state_probe.update();
        assert_matches_oracle(&goal.decide_each(&posed_probe), 2);
    }

    #[test]
    fn violated_outside_tolerance() {
        let model = panda_model();
        let mut state_goal = state_with(&model, &s0());
        let goal =
            construct_goal_joint_constraints(&model, &state_goal.update(), "panda_arm", 0.1, 0.1)
                .unwrap();

        let mut state_probe = state_with(&model, &s2());
        let posed_probe = state_probe.update();
        assert_matches_oracle(&goal.decide_each(&posed_probe), 3);
    }

    #[test]
    fn unknown_group_is_error() {
        let model = panda_model();
        let mut state = state_with(&model, &s0());
        let posed = state.update();
        let err = construct_goal_joint_constraints(&model, &posed, "no_such_group", 0.1, 0.1)
            .unwrap_err();
        assert!(matches!(
            err,
            Error::UnknownName {
                kind: "group",
                ref name
            } if name == "no_such_group"
        ));
    }
}

mod oracle_update_joint_constraints {
    use super::*;

    /// Re-targets a goal built at `s0()` onto `s1()`'s position and checks
    /// the result decides identically to a goal the oracle was asked to
    /// evaluate directly at `s1()`'s position (fixture case 4) -- this is
    /// exactly what "update" is supposed to guarantee.
    #[test]
    fn retargets_to_new_state() {
        let model = panda_model();
        let mut goal = construct_goal_joint_constraints(
            &model,
            &state_with(&model, &s0()).update(),
            "panda_arm",
            0.1,
            0.1,
        )
        .unwrap();

        let mut state_new = state_with(&model, &s1());
        let posed_new = state_new.update();
        let updated = update_joint_constraints(&mut goal, &model, &posed_new, "panda_arm").unwrap();
        assert!(updated);

        assert_matches_oracle(&goal.decide_each(&posed_new), 4);
    }

    /// A constraint for a joint that is not active in the target group stops
    /// the update and reports `false`, but does not undo constraints already
    /// updated earlier in the same call -- upstream's early-return loop
    /// (`kinematic_constraints/utils.cpp:172-192`), reproduced here.
    #[test]
    fn stops_at_first_inactive_joint_without_undoing_earlier_updates() {
        let model = panda_model();
        let mut set = KinematicConstraintSet::new();
        set.push(Constraint::Joint(
            JointConstraint::new(&model, "panda_joint1", 0.0, 0.1, 0.1, 1.0).unwrap(),
        ));
        set.push(Constraint::Joint(
            // Active in "hand", not in "panda_arm".
            JointConstraint::new(&model, "panda_finger_joint1", 0.0, 0.1, 0.1, 1.0).unwrap(),
        ));

        let mut state = state_with(&model, &[("panda_joint1", 0.3)]);
        let posed = state.update();
        let updated = update_joint_constraints(&mut set, &model, &posed, "panda_arm").unwrap();
        assert!(!updated);

        let Constraint::Joint(first) = &set.constraints()[0] else {
            panic!("expected a joint constraint");
        };
        assert!((first.desired_joint_position() - 0.3).abs() < TOLERANCE);

        let Constraint::Joint(second) = &set.constraints()[1] else {
            panic!("expected a joint constraint");
        };
        assert!((second.desired_joint_position() - 0.0).abs() < TOLERANCE);
    }
}

mod oracle_update_pose_constraint {
    use super::*;

    /// A goal built at the identity pose, then re-targeted onto
    /// `goal_point_pose()`, must decide identically to a goal
    /// [`construct_goal_pose_constraints`] built at `goal_point_pose()`
    /// directly (fixture case 5) -- exactly what "update" guarantees.
    #[test]
    fn retargets_both_position_and_orientation() {
        let model = panda_model();
        let transforms = tf(&model);
        let mut goal = construct_goal_pose_constraints(
            &model,
            &transforms,
            "panda_link8",
            "world",
            Isometry3::identity(),
            0.05,
            0.3,
        )
        .unwrap();

        let updated = update_pose_constraint(
            &mut goal,
            &model,
            &transforms,
            "panda_link8",
            "world",
            goal_point_pose(),
        )
        .unwrap();
        assert!(updated);

        let mut state = state_with(&model, &s0());
        let posed = state.update();
        assert_matches_oracle(&goal.decide_each(&posed), 5);
    }

    /// If no position constraint names the link, `update_pose_constraint`
    /// must report `false` *and* never attempt the orientation update --
    /// upstream's `&&` short-circuit (`kinematic_constraints/utils.cpp:271-272`), reproduced by
    /// this port's own `&&` expression. Checked by giving the (never
    /// updated) orientation constraint a target far from a pose whose
    /// position half cannot be found, and confirming `decide()` still sees
    /// the *original* target, not the one this call was asked to apply.
    #[test]
    fn position_not_found_skips_orientation_update() {
        let model = panda_model();
        let transforms = tf(&model);
        let mut goal = construct_goal_orientation_constraints(
            &model,
            &transforms,
            "panda_link8",
            "world",
            UnitQuaternion::identity(),
            0.3,
        )
        .unwrap();

        let mut new_pose = Isometry3::identity();
        new_pose.rotation = UnitQuaternion::from_axis_angle(
            &nalgebra::Vector3::z_axis(),
            std::f64::consts::FRAC_PI_2,
        );
        let updated = update_pose_constraint(
            &mut goal,
            &model,
            &transforms,
            "panda_link8",
            "world",
            new_pose,
        )
        .unwrap();
        assert!(!updated);

        // Same state and fixture as `oracle_construct_goal_orientation_
        // constraints::far_from_goal` (case 9): if the orientation target had
        // actually been overwritten to `new_pose`'s 90-degree rotation, this
        // would no longer match that fixture's distance.
        let mut state = state_with(&model, &s0());
        let posed = state.update();
        assert_matches_oracle(&goal.decide_each(&posed), 9);
    }
}

mod oracle_construct_goal_pose_constraints {
    use super::*;

    #[test]
    fn far_from_goal() {
        let model = panda_model();
        let transforms = tf(&model);
        let mut state = state_with(&model, &s0());
        let posed = state.update();

        let goal = construct_goal_pose_constraints(
            &model,
            &transforms,
            "panda_link8",
            "world",
            goal_point_pose(),
            0.05,
            0.3,
        )
        .unwrap();
        assert_matches_oracle(&goal.decide_each(&posed), 5);
    }

    #[test]
    fn far_from_goal_perturbed_state() {
        let model = panda_model();
        let transforms = tf(&model);
        let mut state = state_with(&model, &SB);
        let posed = state.update();

        let goal = construct_goal_pose_constraints(
            &model,
            &transforms,
            "panda_link8",
            "world",
            goal_point_pose(),
            0.05,
            0.3,
        )
        .unwrap();
        assert_matches_oracle(&goal.decide_each(&posed), 6);
    }
}

mod oracle_construct_goal_pose_constraints_box {
    use super::*;

    #[test]
    fn far_from_goal() {
        let model = panda_model();
        let transforms = tf(&model);
        let mut state = state_with(&model, &s0());
        let posed = state.update();

        let goal = construct_goal_pose_constraints_box(
            &model,
            &transforms,
            "panda_link8",
            "world",
            goal_point_pose(),
            [0.1, 0.1, 0.1],
            [0.2, 0.2, 0.4],
        )
        .unwrap();
        assert_matches_oracle(&goal.decide_each(&posed), 7);
    }

    #[test]
    fn far_from_goal_perturbed_state() {
        let model = panda_model();
        let transforms = tf(&model);
        let mut state = state_with(&model, &SB);
        let posed = state.update();

        let goal = construct_goal_pose_constraints_box(
            &model,
            &transforms,
            "panda_link8",
            "world",
            goal_point_pose(),
            [0.1, 0.1, 0.1],
            [0.2, 0.2, 0.4],
        )
        .unwrap();
        assert_matches_oracle(&goal.decide_each(&posed), 8);
    }
}

mod oracle_construct_goal_orientation_constraints {
    use super::*;

    /// Confirms the quaternion-only overload's parameterization default:
    /// [`construct_goal_orientation_constraints`]'s doc comment explains why
    /// it must be `XyzEuler`, not `RotationVector` like the pose overload --
    /// this is the case that would fail if that were wrong, since the two
    /// parameterizations decompose the same 180-degree rotation error
    /// differently (see `far_from_goal_perturbed_state` for the pose
    /// overload's `RotationVector` distance at the same state, fixture case
    /// 8, which numerically differs from this case's fixture 10).
    #[test]
    fn far_from_goal() {
        let model = panda_model();
        let transforms = tf(&model);
        let mut state = state_with(&model, &s0());
        let posed = state.update();

        let goal = construct_goal_orientation_constraints(
            &model,
            &transforms,
            "panda_link8",
            "world",
            UnitQuaternion::identity(),
            0.3,
        )
        .unwrap();
        assert_matches_oracle(&goal.decide_each(&posed), 9);
    }

    #[test]
    fn far_from_goal_perturbed_state() {
        let model = panda_model();
        let transforms = tf(&model);
        let mut state = state_with(&model, &SB);
        let posed = state.update();

        let goal = construct_goal_orientation_constraints(
            &model,
            &transforms,
            "panda_link8",
            "world",
            UnitQuaternion::identity(),
            0.3,
        )
        .unwrap();
        assert_matches_oracle(&goal.decide_each(&posed), 10);
    }
}

mod oracle_construct_goal_position_constraints {
    use super::*;

    fn reference_point() -> Vector3 {
        Vector3::new(0.01, 0.02, 0.03)
    }

    #[test]
    fn far_from_goal() {
        let model = panda_model();
        let transforms = tf(&model);
        let mut state = state_with(&model, &s0());
        let posed = state.update();

        let goal = construct_goal_position_constraints(
            &model,
            &transforms,
            "panda_link8",
            reference_point(),
            "world",
            Vector3::new(0.4, 0.0, 0.6),
            0.05,
        )
        .unwrap();
        assert_matches_oracle(&goal.decide_each(&posed), 11);
    }

    #[test]
    fn far_from_goal_perturbed_state() {
        let model = panda_model();
        let transforms = tf(&model);
        let mut state = state_with(&model, &SB);
        let posed = state.update();

        let goal = construct_goal_position_constraints(
            &model,
            &transforms,
            "panda_link8",
            reference_point(),
            "world",
            Vector3::new(0.4, 0.0, 0.6),
            0.05,
        )
        .unwrap();
        assert_matches_oracle(&goal.decide_each(&posed), 12);
    }
}

mod update_orientation_constraint_boundary {
    use super::*;

    #[test]
    fn not_found_returns_false() {
        let model = panda_model();
        let transforms = tf(&model);
        let mut set = KinematicConstraintSet::new();
        let updated = update_orientation_constraint(
            &mut set,
            &model,
            &transforms,
            "panda_link8",
            "world",
            UnitQuaternion::identity(),
        )
        .unwrap();
        assert!(!updated);
        assert!(set.is_empty());
    }

    /// `not_found_returns_false` above uses an empty `set`, so its loop over
    /// `constraints_mut()` never runs at all -- it cannot tell "link name
    /// compared and didn't match" from "no comparison happened." This test
    /// puts one non-matching orientation constraint in `set` so the loop
    /// body actually executes, and checks the constraint survives untouched.
    #[test]
    fn mismatched_link_name_leaves_constraint_untouched() {
        let model = panda_model();
        let transforms = tf(&model);
        let oc = OrientationConstraint::new(
            &model,
            &transforms,
            "panda_link7",
            "world",
            UnitQuaternion::identity(),
            OrientationTolerance::XyzEuler {
                x: 0.1,
                y: 0.1,
                z: 0.1,
            },
            1.0,
        )
        .unwrap();
        let mut set = KinematicConstraintSet::new();
        set.push(Constraint::Orientation(oc));

        let updated = update_orientation_constraint(
            &mut set,
            &model,
            &transforms,
            "panda_link8",
            "world",
            UnitQuaternion::identity(),
        )
        .unwrap();

        assert!(!updated);
        assert_eq!(set.len(), 1);
        let Constraint::Orientation(oc) = &set.constraints()[0] else {
            panic!("expected orientation constraint");
        };
        assert_eq!(oc.link_name(), "panda_link7");
    }
}

mod update_position_constraint_boundary {
    use super::*;

    #[test]
    fn not_found_returns_false() {
        let model = panda_model();
        let transforms = tf(&model);
        let mut set = KinematicConstraintSet::new();
        let updated = update_position_constraint(
            &mut set,
            &model,
            &transforms,
            "panda_link8",
            "world",
            Vector3::zeros(),
        )
        .unwrap();
        assert!(!updated);
        assert!(set.is_empty());
    }

    /// `not_found_returns_false` above uses an empty `set`, so its loop over
    /// `constraints_mut()` never runs at all -- it cannot tell "link name
    /// compared and didn't match" from "no comparison happened." This test
    /// puts one non-matching position constraint in `set` so the loop body
    /// actually executes, and checks the constraint survives untouched.
    #[test]
    fn mismatched_link_name_leaves_constraint_untouched() {
        let model = panda_model();
        let transforms = tf(&model);
        let pc = PositionConstraint::new(
            &model,
            &transforms,
            "panda_link7",
            "world",
            Vector3::zeros(),
            &[(
                Shape::Sphere(Sphere::new(0.05).unwrap()),
                Isometry3::identity(),
            )],
            1.0,
        )
        .unwrap();
        let mut set = KinematicConstraintSet::new();
        set.push(Constraint::Position(pc));

        let updated = update_position_constraint(
            &mut set,
            &model,
            &transforms,
            "panda_link8",
            "world",
            Vector3::zeros(),
        )
        .unwrap();

        assert!(!updated);
        assert_eq!(set.len(), 1);
        let Constraint::Position(pc) = &set.constraints()[0] else {
            panic!("expected position constraint");
        };
        assert_eq!(pc.link_name(), "panda_link7");
    }

    /// A constraint with more than one region has no single
    /// `primitive_poses[0]` to update unambiguously -- see
    /// [`PositionConstraint::with_updated_position`]'s doc comment for why
    /// this port reports the whole update as an error instead of silently
    /// updating just the first region.
    #[test]
    fn multi_region_constraint_is_error() {
        let model = panda_model();
        let transforms = tf(&model);
        let sphere_at = |x: f64| {
            let mut pose = Isometry3::identity();
            pose.translation.vector = Vector3::new(x, 0.0, 0.0);
            (Shape::Sphere(Sphere::new(0.05).unwrap()), pose)
        };
        let pc = PositionConstraint::new(
            &model,
            &transforms,
            "panda_link8",
            "world",
            Vector3::zeros(),
            &[sphere_at(0.0), sphere_at(1.0)],
            1.0,
        )
        .unwrap();
        let mut set = KinematicConstraintSet::new();
        set.push(Constraint::Position(pc));

        let err = update_position_constraint(
            &mut set,
            &model,
            &transforms,
            "panda_link8",
            "world",
            Vector3::zeros(),
        )
        .unwrap_err();
        assert!(matches!(err, Error::Other { .. }));
    }
}

mod merge_constraints_boundary {
    use super::*;

    fn joint_set(
        model: &RobotModel,
        position: f64,
        tolerance: f64,
        weight: f64,
    ) -> KinematicConstraintSet {
        let mut set = KinematicConstraintSet::new();
        set.push(Constraint::Joint(
            JointConstraint::new(
                model,
                "panda_joint1",
                position,
                tolerance,
                tolerance,
                weight,
            )
            .unwrap(),
        ));
        set
    }

    /// `low = max(-0.2, -0.1) = -0.1`, `high = min(0.2, 0.3) = 0.2`,
    /// `position = clamp((0*1 + 0.1*1) / 2, -0.1, 0.2) = 0.05`, matching
    /// `mergeConstraints`'s formula (`kinematic_constraints/utils.cpp:83-98`) by hand.
    #[test]
    fn overlapping_windows_merge_to_the_intersection() {
        let model = panda_model();
        let first = joint_set(&model, 0.0, 0.2, 1.0);
        let second = joint_set(&model, 0.1, 0.2, 1.0);
        let merged = merge_constraints(&first, &second);

        assert_eq!(merged.len(), 1);
        let Constraint::Joint(m) = &merged.constraints()[0] else {
            panic!("expected a joint constraint");
        };
        assert!((m.desired_joint_position() - 0.05).abs() < TOLERANCE);
        assert!((m.joint_tolerance_above() - 0.15).abs() < TOLERANCE);
        assert!((m.joint_tolerance_below() - 0.15).abs() < TOLERANCE);
        assert!((m.weight() - 1.0).abs() < TOLERANCE);
    }

    /// `low = max(-0.05, 0.95) = 0.95`, `high = min(0.05, 1.05) = 0.05`,
    /// `low > high`: upstream discards the pair with an error log; this port
    /// drops it from the merged set instead.
    #[test]
    fn non_overlapping_windows_are_dropped() {
        let model = panda_model();
        let first = joint_set(&model, 0.0, 0.05, 1.0);
        let second = joint_set(&model, 1.0, 0.05, 1.0);
        let merged = merge_constraints(&first, &second);
        assert!(merged.is_empty());
    }

    /// A `NaN` reaches `merged` through `tolerance_below`, not `position`:
    /// `JointConstraint::new`'s only numeric screen is `tolerance_above <
    /// 0.0 || tolerance_below < 0.0`, and every comparison against `NaN` is
    /// false, so a `NaN` tolerance is constructible here exactly as
    /// `JointConstraint::configure`'s identical `jc.tolerance_above < 0.0 ||
    /// jc.tolerance_below < 0.0` leaves it constructible upstream.
    ///
    /// Upstream then computes `low = std::max(a.position -
    /// a.tolerance_below, ...)` = `std::max(NaN, ...)`. `std::max(a, b)` is
    /// `a < b ? b : a`, so a `NaN` *first* operand comes back out as `NaN`;
    /// `low > high` is false for it, and the `NaN` reaches `m.position` via
    /// `std::max(low, std::min(...))`. The merged constraint is therefore
    /// unsatisfiable-by-`NaN`, which is what makes it detectable downstream.
    ///
    /// Rust's `f64::max` is IEEE `maxNum` and *discards* `NaN`, so the
    /// pre-fix port silently substituted the other constraint's window and
    /// returned a finite, plausible-looking merge — the `NaN` disappeared.
    #[test]
    fn a_nan_tolerance_keeps_upstreams_nan_merge_instead_of_a_finite_window() {
        let model = panda_model();
        let mut first = KinematicConstraintSet::new();
        first.push(Constraint::Joint(
            JointConstraint::new(&model, "panda_joint1", 0.0, 0.2, f64::NAN, 1.0).unwrap(),
        ));
        let second = joint_set(&model, 0.1, 0.2, 1.0);

        let merged = merge_constraints(&first, &second);
        assert_eq!(merged.len(), 1);
        let Constraint::Joint(m) = &merged.constraints()[0] else {
            panic!("expected a joint constraint");
        };
        assert!(
            m.desired_joint_position().is_nan(),
            "expected upstream's NaN position, got {}",
            m.desired_joint_position()
        );
        // `std::max(0.0, high - m.position)` with a `NaN` right operand
        // returns the `0.0`: upstream's tolerances collapse, they do not
        // become `NaN`.
        assert_eq!(m.joint_tolerance_above(), 0.0);
        assert_eq!(m.joint_tolerance_below(), 0.0);
    }

    /// The demonstrated opposite of the case above: with the same `NaN` on
    /// the *second* constraint instead of the first, upstream's `std::max(a,
    /// b)` sees the `NaN` as its right operand and returns `a`, so the merge
    /// is finite. Without this the test above would also pass on a port that
    /// simply propagated `NaN` from either side.
    #[test]
    fn a_nan_tolerance_on_the_second_side_still_merges_finitely() {
        let model = panda_model();
        let first = joint_set(&model, 0.0, 0.2, 1.0);
        let mut second = KinematicConstraintSet::new();
        second.push(Constraint::Joint(
            JointConstraint::new(&model, "panda_joint1", 0.1, 0.2, f64::NAN, 1.0).unwrap(),
        ));

        let merged = merge_constraints(&first, &second);
        assert_eq!(merged.len(), 1);
        let Constraint::Joint(m) = &merged.constraints()[0] else {
            panic!("expected a joint constraint");
        };
        // low = max(0.0 - 0.2, NaN) = -0.2, high = min(0.2, 0.3) = 0.2,
        // position = clamp(0.05, -0.2, 0.2) = 0.05.
        assert!((m.desired_joint_position() - 0.05).abs() < TOLERANCE);
        assert!((m.joint_tolerance_above() - 0.15).abs() < TOLERANCE);
        assert!((m.joint_tolerance_below() - 0.25).abs() < TOLERANCE);
    }

    /// `f64::clamp` asserts `min <= max` and so **panics** when either bound
    /// is `NaN`. With `NaN` on both sides' `tolerance_below` the pre-fix
    /// `low`/`high` were both `NaN`, `low > high` was false so the
    /// non-overlap early return did not fire, and `merge_constraints` — a
    /// public entry point reached straight from a request message — aborted
    /// the process. Upstream has no assertion here at all: it computes a
    /// `NaN` merge and carries on.
    #[test]
    fn a_nan_tolerance_on_both_sides_merges_instead_of_panicking() {
        let model = panda_model();
        let mut first = KinematicConstraintSet::new();
        first.push(Constraint::Joint(
            JointConstraint::new(&model, "panda_joint1", 0.0, f64::NAN, f64::NAN, 1.0).unwrap(),
        ));
        let mut second = KinematicConstraintSet::new();
        second.push(Constraint::Joint(
            JointConstraint::new(&model, "panda_joint1", 0.1, f64::NAN, f64::NAN, 1.0).unwrap(),
        ));

        let merged = merge_constraints(&first, &second);
        assert_eq!(merged.len(), 1);
        let Constraint::Joint(m) = &merged.constraints()[0] else {
            panic!("expected a joint constraint");
        };
        assert!(m.desired_joint_position().is_nan());
        assert_eq!(m.joint_tolerance_above(), 0.0);
        assert_eq!(m.joint_tolerance_below(), 0.0);
    }

    #[test]
    fn constraint_present_in_only_one_side_is_kept() {
        let model = panda_model();
        let mut first = KinematicConstraintSet::new();
        first.push(Constraint::Joint(
            JointConstraint::new(&model, "panda_joint1", 0.0, 0.1, 0.1, 1.0).unwrap(),
        ));
        let mut second = KinematicConstraintSet::new();
        second.push(Constraint::Joint(
            JointConstraint::new(&model, "panda_joint2", 0.0, 0.1, 0.1, 1.0).unwrap(),
        ));

        let merged = merge_constraints(&first, &second);
        assert_eq!(merged.len(), 2);

        let merged_again = merge_constraints(&second, &first);
        assert_eq!(merged_again.len(), 2);
    }
}

#[test]
fn count_individual_constraints_is_len() {
    let model = panda_model();
    let mut set = KinematicConstraintSet::new();
    assert_eq!(count_individual_constraints(&set), 0);
    set.push(Constraint::Joint(
        JointConstraint::new(&model, "panda_joint1", 0.0, 0.1, 0.1, 1.0).unwrap(),
    ));
    assert_eq!(count_individual_constraints(&set), 1);
}

/// `resolveConstraintFrames` ported as [`resolve_position_constraint_frame`]/
/// [`resolve_orientation_constraint_frame`] -- no oracle op exists for it
/// (its only upstream caller is a `moveit_ros` planning request adapter, not
/// anything the C++ oracle exposes), so every case here is a hand-computed
/// boundary test, per `PORTING-PLAN.md` §23's "test by invariant boundary"
/// convention for this crate.
mod resolve_constraint_frame_boundary {
    use super::*;

    /// The entire point of the function: `"gripper_tool"` is not a link in
    /// `panda_model()` at all -- it stands in for an attached body's
    /// subframe, resolved only by the closure (as `PlanningScene::
    /// attached_frame` would resolve a real one). A pure translation `T`
    /// (identity rotation) keeps the expected offset a one-line hand
    /// computation: `frame_to_link * point(offset) = offset + translation`
    /// for an identity-rotation isometry. If this degenerated back into a
    /// no-op (as round 3's design would have), `resolved` would equal
    /// `"gripper_tool"` and `offset` would be unchanged -- neither happens.
    #[test]
    fn attached_subframe_resolves_to_its_link_with_an_adjusted_position_offset() {
        let model = panda_model();
        let mut state = state_with(&model, &s0());
        let posed = state.update();

        let mut frame_to_link = Isometry3::identity();
        frame_to_link.translation.vector = Vector3::new(1.0, 2.0, 3.0);
        let resolve_attached = |name: &str| {
            (name == "gripper_tool").then(|| ("panda_link8".to_string(), frame_to_link))
        };

        let offset = Vector3::new(0.1, 0.0, 0.0);
        let (resolved, adjusted) = resolve_position_constraint_frame(
            &model,
            &posed,
            "gripper_tool",
            offset,
            resolve_attached,
        )
        .unwrap()
        .expect("gripper_tool is resolvable via the closure");

        assert_eq!(resolved, "panda_link8");
        assert!((adjusted - Vector3::new(1.1, 2.0, 3.0)).norm() < TOLERANCE);
    }

    /// The orientation half of the same case: a pure rotation (no
    /// translation, irrelevant to orientation) composes the identity target
    /// orientation with `link_name_to_robot_link = frame_to_link.rotation().inverse()`
    /// -- so the resolved orientation should be exactly that inverse
    /// rotation, not the original identity.
    #[test]
    fn attached_subframe_resolves_to_its_link_with_an_adjusted_orientation() {
        let model = panda_model();
        let mut state = state_with(&model, &s0());
        let posed = state.update();

        let rotation =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_2);
        let frame_to_link = Isometry3::from_parts(nalgebra::Translation3::identity(), rotation);
        let resolve_attached = |name: &str| {
            (name == "gripper_tool").then(|| ("panda_link8".to_string(), frame_to_link))
        };

        let (resolved, adjusted) = resolve_orientation_constraint_frame(
            &model,
            &posed,
            "gripper_tool",
            UnitQuaternion::identity(),
            OrientationTolerance::RotationVector {
                x: 0.1,
                y: 0.1,
                z: 0.1,
            },
            resolve_attached,
        )
        .unwrap()
        .expect("gripper_tool is resolvable via the closure");

        assert_eq!(resolved, "panda_link8");
        let expected = rotation.inverse();
        assert!(adjusted.angle_to(&expected) < TOLERANCE);
    }

    /// `link_name` already names a real link: both halves return it
    /// untouched, with the caller's offset/orientation unchanged -- upstream's
    /// own `c.link_name != robot_link->getName()` guard, exercised on the
    /// branch where it is false.
    #[test]
    fn a_plain_link_name_is_returned_unchanged() {
        let model = panda_model();
        let mut state = state_with(&model, &s0());
        let posed = state.update();
        let offset = Vector3::new(0.4, 0.5, 0.6);

        let (resolved, adjusted) =
            resolve_position_constraint_frame(&model, &posed, "panda_link8", offset, |_| None)
                .unwrap()
                .expect("a real link always resolves");
        assert_eq!(resolved, "panda_link8");
        assert_eq!(adjusted, offset);

        let orientation = UnitQuaternion::identity();
        let (resolved, adjusted) = resolve_orientation_constraint_frame(
            &model,
            &posed,
            "panda_link8",
            orientation,
            OrientationTolerance::XyzEuler {
                x: 0.1,
                y: 0.1,
                z: 0.1,
            },
            |_| None,
        )
        .unwrap()
        .expect("a real link always resolves");
        assert_eq!(resolved, "panda_link8");
        assert_eq!(adjusted, orientation);
    }

    /// The model frame maps to the root link, with `frame_to_link` derived
    /// from the *current* state rather than a closure -- confirmed against
    /// an independently-taken `global_link_transform` on the same state
    /// (the only ground truth available without an oracle op for this
    /// function; see this module's own doc comment).
    #[test]
    fn the_model_frame_resolves_to_the_root_link() {
        let model = panda_model();
        let mut state = state_with(&model, &s0());
        let posed = state.update();
        let root_transform = posed.global_link_transform(model.root_link_name()).unwrap();

        let offset = Vector3::new(0.2, 0.0, 0.0);
        let (resolved, adjusted) =
            resolve_position_constraint_frame(&model, &posed, model.model_frame(), offset, |_| {
                None
            })
            .unwrap()
            .expect("the model frame always resolves");

        assert_eq!(resolved, model.root_link_name());
        let expected = (root_transform.inverse() * nalgebra::Point3::from(offset)).coords;
        assert!((adjusted - expected).norm() < TOLERANCE);
    }

    /// Neither a link, the model frame, nor anything the closure recognises:
    /// `Ok(None)`, matching upstream's `frame_found = false`.
    #[test]
    fn an_unrecognised_frame_is_none() {
        let model = panda_model();
        let mut state = state_with(&model, &s0());
        let posed = state.update();

        assert!(
            resolve_position_constraint_frame(
                &model,
                &posed,
                "nonexistent_frame",
                Vector3::zeros(),
                |_| None,
            )
            .unwrap()
            .is_none()
        );
        assert!(
            resolve_orientation_constraint_frame(
                &model,
                &posed,
                "nonexistent_frame",
                UnitQuaternion::identity(),
                OrientationTolerance::RotationVector {
                    x: 0.1,
                    y: 0.1,
                    z: 0.1
                },
                |_| None,
            )
            .unwrap()
            .is_none()
        );
    }

    /// Upstream refuses to retarget an `XyzEuler` orientation constraint
    /// across an actual frame change (`kinematic_constraints/utils.cpp:661-664`): Euler-angle
    /// tolerances have no rotation-composition rule, only rotation vectors
    /// do. This is the one case that is a genuine frame change (attached
    /// subframe, not a same-link no-op), so it must error rather than
    /// silently retarget.
    #[test]
    fn xyz_euler_tolerance_across_a_real_frame_change_is_an_error() {
        let model = panda_model();
        let mut state = state_with(&model, &s0());
        let posed = state.update();
        let resolve_attached = |name: &str| {
            (name == "gripper_tool").then(|| ("panda_link8".to_string(), Isometry3::identity()))
        };

        let err = resolve_orientation_constraint_frame(
            &model,
            &posed,
            "gripper_tool",
            UnitQuaternion::identity(),
            OrientationTolerance::XyzEuler {
                x: 0.1,
                y: 0.1,
                z: 0.1,
            },
            resolve_attached,
        )
        .unwrap_err();
        assert!(matches!(err, Error::Other { .. }));
    }
}
