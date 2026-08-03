// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Unit tests for the four constraint types' `decide()`/construction logic,
//! one case per invariant boundary (tolerance edge, mobile-vs-fixed frame,
//! Euler-singularity branch, `Option`-vs-`None` criterion) rather than one
//! per narrative scenario. `panda.urdf`/`panda.srdf` (copied from
//! `moveit-state`'s fixtures) supply a real, already-oracle-verified model
//! and FK — see `crates/moveit-model/tests/fixtures/panda_model_info.json`
//! and `crates/moveit-state/tests/fixtures/panda_fk.json` for that
//! verification. Oracle parity for `decide()` itself (the 2,000-combination
//! check `PORTING-PLAN.md` §5 Phase 5 requires) lives in a separate
//! integration test once the oracle gains a `constraints` op.

use std::fs;

use moveit_error::Error;
use moveit_geometry::{Isometry3, Shape, Sphere, Transforms, UnitQuaternion, Vector3};
use moveit_model::RobotModel;
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;

use moveit_constraints::{
    Constraint, JointConstraint, KinematicConstraintSet, OrientationConstraint,
    OrientationTolerance, PositionConstraint, SensorSpec, SensorViewDirection, TargetSpec,
    VisibilityConstraint, VisibilityCriteria, VisibilityDecision,
};

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        file_name
    )
}

fn panda_model() -> RobotModel {
    let urdf_path = fixture_path("panda.urdf");
    let srdf_path = fixture_path("panda.srdf");
    let urdf_xml = fs::read_to_string(&urdf_path).expect("read panda.urdf");
    let urdf = urdf_rs::read_file(&urdf_path).expect("parse panda.urdf");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("parse panda.srdf");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf).expect("build panda model")
}

/// PR2 has continuous joints (e.g. `fl_caster_rotation_joint`); panda and
/// fanuc do not, so the continuous-joint branch of [`JointConstraint`] needs
/// this fixture specifically.
fn pr2_model() -> RobotModel {
    let urdf_path = fixture_path("pr2.urdf");
    let srdf_path = fixture_path("pr2.srdf");
    let urdf_xml = fs::read_to_string(&urdf_path).expect("read pr2.urdf");
    let urdf = urdf_rs::read_file(&urdf_path).expect("parse pr2.urdf");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("parse pr2.srdf");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf).expect("build pr2 model")
}

fn sphere_region(radius: f64, pose: Isometry3) -> (Shape, Isometry3) {
    (Shape::Sphere(Sphere::new(radius).unwrap()), pose)
}

mod joint {
    use super::*;

    #[test]
    fn satisfied_at_exact_position() {
        let model = panda_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        state.set_variable_position("panda_joint1", 0.5).unwrap();
        let posed = state.update();

        let c = JointConstraint::new(&model, "panda_joint1", 0.5, 0.1, 0.1, 1.0).unwrap();
        let r = c.decide(&posed);
        assert!(r.satisfied);
        assert!(r.distance.abs() < 1e-12);
    }

    #[test]
    fn satisfied_exactly_at_tolerance_edge() {
        let model = panda_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        state.set_variable_position("panda_joint1", 0.6).unwrap();
        let posed = state.update();

        // position=0.5, tolerance_above=0.1 -> boundary is exactly 0.6.
        let c = JointConstraint::new(&model, "panda_joint1", 0.5, 0.1, 0.1, 1.0).unwrap();
        assert!(c.decide(&posed).satisfied);
    }

    #[test]
    fn violated_just_past_tolerance_edge() {
        let model = panda_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        state.set_variable_position("panda_joint1", 0.7).unwrap();
        let posed = state.update();

        let c = JointConstraint::new(&model, "panda_joint1", 0.5, 0.1, 0.1, 1.0).unwrap();
        let r = c.decide(&posed);
        assert!(!r.satisfied);
        assert!(r.distance > 0.0);
    }

    #[test]
    fn continuous_joint_wraps_across_the_pi_boundary() {
        use std::f64::consts::PI;

        // fl_caster_rotation_joint is continuous (see `pr2_model`'s doc
        // comment): a target near +pi and a current value near -pi are only
        // `2*EPS` apart going the short way around, so the wraparound branch
        // must report this satisfied under a tight tolerance that the naive
        // `current - position` difference (about `2*pi`) would violate.
        let model = pr2_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        state
            .set_variable_position("fl_caster_rotation_joint", -PI + 1e-6)
            .unwrap();
        let posed = state.update();

        let c = JointConstraint::new(
            &model,
            "fl_caster_rotation_joint",
            PI - 1e-6,
            1e-3,
            1e-3,
            1.0,
        )
        .unwrap();
        let r = c.decide(&posed);
        assert!(r.satisfied, "distance = {}", r.distance);
    }

    #[test]
    fn continuous_joint_violation_does_not_falsely_wrap() {
        use std::f64::consts::PI;

        let model = pr2_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        state
            .set_variable_position("fl_caster_rotation_joint", 0.0)
            .unwrap();
        let posed = state.update();

        let c =
            JointConstraint::new(&model, "fl_caster_rotation_joint", PI, 1e-3, 1e-3, 1.0).unwrap();
        assert!(!c.decide(&posed).satisfied);
    }

    #[test]
    fn new_rejects_negative_tolerance() {
        let model = panda_model();
        assert!(JointConstraint::new(&model, "panda_joint1", 0.0, -0.1, 0.1, 1.0).is_err());
        assert!(JointConstraint::new(&model, "panda_joint1", 0.0, 0.1, -0.1, 1.0).is_err());
    }

    #[test]
    fn new_rejects_non_positive_weight() {
        let model = panda_model();
        assert!(JointConstraint::new(&model, "panda_joint1", 0.0, 0.1, 0.1, 0.0).is_err());
        assert!(JointConstraint::new(&model, "panda_joint1", 0.0, 0.1, 0.1, -1.0).is_err());
    }

    #[test]
    fn new_rejects_unknown_joint() {
        let model = panda_model();
        assert!(matches!(
            JointConstraint::new(&model, "no_such_joint", 0.0, 0.1, 0.1, 1.0),
            Err(Error::UnknownName { .. })
        ));
    }

    #[test]
    fn new_clamps_a_position_outside_the_joint_bounds() {
        // panda_joint1's effective max bound is its safety-controller soft
        // limit (2.8973), not the raw <limit> (2.9671) -- moveit-model
        // already prefers the soft limit when present (see
        // crates/moveit-model/src/joint/urdf.rs), matching upstream. Asking
        // for 10.0 clamps to that bound with tolerance_below squeezed to EPS
        // (upstream: "bounds.max_position_ < joint_position_ -
        // joint_tolerance_below_" clamps and zeroes tolerance_below, not
        // tolerance_above).
        let model = panda_model();
        let c = JointConstraint::new(&model, "panda_joint1", 10.0, 0.1, 0.1, 1.0).unwrap();
        assert!((c.desired_joint_position() - 2.8973).abs() < 1e-9);
        assert!(c.joint_tolerance_below() <= f64::EPSILON);
        assert_eq!(c.joint_tolerance_above(), 0.1);
    }
}

mod position {
    use super::*;

    fn tf(model: &RobotModel) -> Transforms {
        Transforms::new(model.model_frame()).unwrap()
    }

    #[test]
    fn satisfied_when_link_origin_is_inside_the_region() {
        let model = panda_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let link_pos = posed
            .global_link_transform("panda_link8")
            .unwrap()
            .translation
            .vector;

        let transforms = tf(&model);
        let region = sphere_region(
            0.05,
            Isometry3::translation(link_pos.x, link_pos.y, link_pos.z),
        );
        let c = PositionConstraint::new(
            &model,
            &transforms,
            "panda_link8",
            model.model_frame(),
            Vector3::zeros(),
            &[region],
            1.0,
        )
        .unwrap();
        assert!(c.decide(&posed).satisfied);
    }

    #[test]
    fn violated_when_link_origin_is_outside_every_region() {
        let model = panda_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let transforms = tf(&model);
        let region = sphere_region(0.01, Isometry3::translation(50.0, 50.0, 50.0));
        let c = PositionConstraint::new(
            &model,
            &transforms,
            "panda_link8",
            model.model_frame(),
            Vector3::zeros(),
            &[region],
            1.0,
        )
        .unwrap();
        let r = c.decide(&posed);
        assert!(!r.satisfied);
        assert!(r.distance > 0.0);
    }

    #[test]
    fn mobile_frame_is_resolved_fresh_from_state() {
        // Region centered exactly at panda_link0's origin, expressed relative
        // to panda_link0 itself (a mobile frame identical to the target
        // link's own frame at the identity offset) -- must stay satisfied
        // regardless of panda_link0's own pose in the world.
        let model = panda_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let transforms = tf(&model);
        let region = sphere_region(0.01, Isometry3::identity());
        let c = PositionConstraint::new(
            &model,
            &transforms,
            "panda_link0",
            "panda_link0",
            Vector3::zeros(),
            &[region],
            1.0,
        )
        .unwrap();
        assert!(c.mobile_reference_frame());
        assert!(c.decide(&posed).satisfied);
    }

    #[test]
    fn link_offset_shifts_the_evaluated_point() {
        let model = panda_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let link_pos = posed
            .global_link_transform("panda_link8")
            .unwrap()
            .translation
            .vector;

        let transforms = tf(&model);
        // Region centered where the link would be after a +0.1 x-offset.
        let region = sphere_region(
            0.01,
            Isometry3::translation(link_pos.x + 0.1, link_pos.y, link_pos.z),
        );
        let c = PositionConstraint::new(
            &model,
            &transforms,
            "panda_link8",
            model.model_frame(),
            Vector3::new(0.1, 0.0, 0.0),
            &[region],
            1.0,
        )
        .unwrap();
        assert!(c.has_link_offset());
        assert!(c.decide(&posed).satisfied);
    }

    #[test]
    fn new_rejects_empty_frame_id() {
        let model = panda_model();
        let transforms = tf(&model);
        let region = sphere_region(0.01, Isometry3::identity());
        assert!(
            PositionConstraint::new(
                &model,
                &transforms,
                "panda_link8",
                "",
                Vector3::zeros(),
                &[region],
                1.0,
            )
            .is_err()
        );
    }

    #[test]
    fn new_rejects_no_regions() {
        let model = panda_model();
        let transforms = tf(&model);
        assert!(
            PositionConstraint::new(
                &model,
                &transforms,
                "panda_link8",
                model.model_frame(),
                Vector3::zeros(),
                &[],
                1.0,
            )
            .is_err()
        );
    }

    #[test]
    fn new_rejects_unresolvable_mobile_frame() {
        let model = panda_model();
        let transforms = tf(&model);
        let region = sphere_region(0.01, Isometry3::identity());
        assert!(
            PositionConstraint::new(
                &model,
                &transforms,
                "panda_link8",
                "no_such_frame",
                Vector3::zeros(),
                &[region],
                1.0,
            )
            .is_err()
        );
    }
}

mod orientation {
    use super::*;

    fn tf(model: &RobotModel) -> Transforms {
        Transforms::new(model.model_frame()).unwrap()
    }

    #[test]
    fn satisfied_when_orientation_matches_exactly() {
        let model = panda_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let link_rot = posed.global_link_transform("panda_link8").unwrap().rotation;

        let transforms = tf(&model);
        let c = OrientationConstraint::new(
            &model,
            &transforms,
            "panda_link8",
            model.model_frame(),
            link_rot,
            OrientationTolerance::XyzEuler {
                x: 0.01,
                y: 0.01,
                z: 0.01,
            },
            1.0,
        )
        .unwrap();
        let r = c.decide(&posed);
        assert!(r.satisfied);
        assert!(r.distance.abs() < 1e-9);
    }

    #[test]
    fn violated_past_xyz_euler_tolerance() {
        let model = panda_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let link_rot = posed.global_link_transform("panda_link8").unwrap().rotation;
        let perturbed =
            link_rot * UnitQuaternion::from_axis_angle(&nalgebra::Vector3::x_axis(), 0.5);

        let transforms = tf(&model);
        let c = OrientationConstraint::new(
            &model,
            &transforms,
            "panda_link8",
            model.model_frame(),
            perturbed,
            OrientationTolerance::XyzEuler {
                x: 0.01,
                y: 0.01,
                z: 0.01,
            },
            1.0,
        )
        .unwrap();
        assert!(!c.decide(&posed).satisfied);
    }

    #[test]
    fn rotation_vector_tolerance_agrees_with_xyz_euler_at_identity_error() {
        let model = panda_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let link_rot = posed.global_link_transform("panda_link8").unwrap().rotation;

        let transforms = tf(&model);
        let euler = OrientationConstraint::new(
            &model,
            &transforms,
            "panda_link8",
            model.model_frame(),
            link_rot,
            OrientationTolerance::XyzEuler {
                x: 0.01,
                y: 0.01,
                z: 0.01,
            },
            1.0,
        )
        .unwrap();
        let rotvec = OrientationConstraint::new(
            &model,
            &transforms,
            "panda_link8",
            model.model_frame(),
            link_rot,
            OrientationTolerance::RotationVector {
                x: 0.01,
                y: 0.01,
                z: 0.01,
            },
            1.0,
        )
        .unwrap();
        assert!(euler.decide(&posed).satisfied);
        assert!(rotvec.decide(&posed).satisfied);
    }

    #[test]
    fn mobile_frame_is_resolved_fresh_from_state() {
        let model = panda_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let transforms = tf(&model);
        let c = OrientationConstraint::new(
            &model,
            &transforms,
            "panda_link0",
            "panda_link0",
            UnitQuaternion::identity(),
            OrientationTolerance::XyzEuler {
                x: 0.01,
                y: 0.01,
                z: 0.01,
            },
            1.0,
        )
        .unwrap();
        assert!(c.mobile_reference_frame());
        assert!(c.decide(&posed).satisfied);
    }

    #[test]
    fn new_rejects_unresolvable_frame() {
        let model = panda_model();
        let transforms = tf(&model);
        assert!(
            OrientationConstraint::new(
                &model,
                &transforms,
                "panda_link8",
                "no_such_frame",
                UnitQuaternion::identity(),
                OrientationTolerance::XyzEuler {
                    x: 0.01,
                    y: 0.01,
                    z: 0.01,
                },
                1.0,
            )
            .is_err()
        );
    }

    #[test]
    fn new_rejects_non_positive_weight() {
        let model = panda_model();
        let transforms = tf(&model);
        assert!(
            OrientationConstraint::new(
                &model,
                &transforms,
                "panda_link8",
                model.model_frame(),
                UnitQuaternion::identity(),
                OrientationTolerance::XyzEuler {
                    x: 0.01,
                    y: 0.01,
                    z: 0.01,
                },
                0.0,
            )
            .is_err()
        );
    }
}

mod visibility {
    use super::*;

    fn tf(model: &RobotModel) -> Transforms {
        Transforms::new(model.model_frame()).unwrap()
    }

    #[test]
    fn disabled_with_no_criteria_reports_satisfied() {
        let model = panda_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let transforms = tf(&model);
        let c = VisibilityConstraint::new(
            &model,
            &transforms,
            SensorSpec {
                frame_id: model.model_frame(),
                pose: Isometry3::translation(0.0, 0.0, 1.0),
                view_direction: SensorViewDirection::SensorZ,
            },
            TargetSpec {
                frame_id: model.model_frame(),
                pose: Isometry3::identity(),
            },
            8,
            VisibilityCriteria::default(),
            1.0,
        )
        .unwrap();
        assert!(!c.enabled());
        assert_eq!(
            c.decide_geometry(&posed),
            VisibilityDecision::Decided(moveit_constraints::ConstraintEvaluationResult::new(
                true, 0.0
            ))
        );
    }

    #[test]
    fn view_angle_violation_is_decided_without_a_collision_check() {
        let model = panda_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let transforms = tf(&model);
        // Sensor looking along +Z, sitting directly below a target whose own
        // +Z (surface normal) also points up -- the sensor is looking at the
        // target's *back*, so the view-angle check must fail immediately.
        let c = VisibilityConstraint::new(
            &model,
            &transforms,
            SensorSpec {
                frame_id: model.model_frame(),
                pose: Isometry3::translation(0.0, 0.0, -1.0),
                view_direction: SensorViewDirection::SensorZ,
            },
            TargetSpec {
                frame_id: model.model_frame(),
                pose: Isometry3::identity(),
            },
            8,
            VisibilityCriteria {
                max_view_angle: Some(0.1),
                ..Default::default()
            },
            1.0,
        )
        .unwrap();
        assert!(c.enabled());
        match c.decide_geometry(&posed) {
            VisibilityDecision::Decided(r) => assert!(!r.satisfied),
            VisibilityDecision::NeedsConeCollisionCheck => {
                panic!("view-angle violation must not require a collision check")
            }
        }
    }

    #[test]
    fn range_angle_violation_is_decided_without_a_collision_check() {
        let model = panda_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let transforms = tf(&model);
        // Sensor views along +X but the target sits off to the side along Y,
        // well outside a tight max_range_angle.
        let c = VisibilityConstraint::new(
            &model,
            &transforms,
            SensorSpec {
                frame_id: model.model_frame(),
                pose: Isometry3::identity(),
                view_direction: SensorViewDirection::SensorX,
            },
            TargetSpec {
                frame_id: model.model_frame(),
                pose: Isometry3::translation(0.0, 1.0, 0.0),
            },
            8,
            VisibilityCriteria {
                max_range_angle: Some(0.1),
                ..Default::default()
            },
            1.0,
        )
        .unwrap();
        match c.decide_geometry(&posed) {
            VisibilityDecision::Decided(r) => assert!(!r.satisfied),
            VisibilityDecision::NeedsConeCollisionCheck => {
                panic!("range-angle violation must not require a collision check")
            }
        }
    }

    #[test]
    fn target_radius_alone_needs_a_cone_collision_check() {
        let model = panda_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let transforms = tf(&model);
        let c = VisibilityConstraint::new(
            &model,
            &transforms,
            SensorSpec {
                frame_id: model.model_frame(),
                pose: Isometry3::translation(0.0, 0.0, 1.0),
                view_direction: SensorViewDirection::SensorZ,
            },
            TargetSpec {
                frame_id: model.model_frame(),
                pose: Isometry3::identity(),
            },
            8,
            VisibilityCriteria {
                target_radius: Some(0.02),
                ..Default::default()
            },
            1.0,
        )
        .unwrap();
        assert_eq!(
            c.decide_geometry(&posed),
            VisibilityDecision::NeedsConeCollisionCheck
        );
    }

    #[test]
    fn zero_valued_criteria_normalize_to_unconstrained() {
        // Same magic-zero shape the crate's doc comments call out: 0.0 must
        // behave exactly like None, not like an active zero-tolerance
        // criterion.
        let model = panda_model();
        let transforms = tf(&model);
        let c = VisibilityConstraint::new(
            &model,
            &transforms,
            SensorSpec {
                frame_id: model.model_frame(),
                pose: Isometry3::identity(),
                view_direction: SensorViewDirection::SensorZ,
            },
            TargetSpec {
                frame_id: model.model_frame(),
                pose: Isometry3::identity(),
            },
            8,
            VisibilityCriteria {
                target_radius: Some(0.0),
                max_view_angle: Some(0.0),
                max_range_angle: Some(0.0),
            },
            1.0,
        )
        .unwrap();
        assert!(!c.enabled());
    }

    #[test]
    fn cone_sides_below_three_is_clamped() {
        let model = panda_model();
        let transforms = tf(&model);
        let c = VisibilityConstraint::new(
            &model,
            &transforms,
            SensorSpec {
                frame_id: model.model_frame(),
                pose: Isometry3::identity(),
                view_direction: SensorViewDirection::SensorZ,
            },
            TargetSpec {
                frame_id: model.model_frame(),
                pose: Isometry3::identity(),
            },
            1,
            VisibilityCriteria::default(),
            1.0,
        )
        .unwrap();
        assert_eq!(c.cone_sides(), 3);
    }
}

mod set {
    use super::*;

    #[test]
    fn empty_set_is_vacuously_satisfied() {
        let set = KinematicConstraintSet::new();
        assert!(set.is_empty());
        let model = panda_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let r = set.decide(&posed).unwrap();
        assert!(r.satisfied);
        assert_eq!(r.distance, 0.0);
    }

    #[test]
    fn satisfied_iff_every_member_is() {
        let model = panda_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        state.set_variable_position("panda_joint1", 0.5).unwrap();
        let posed = state.update();

        let ok = JointConstraint::new(&model, "panda_joint1", 0.5, 0.01, 0.01, 1.0).unwrap();
        let bad = JointConstraint::new(&model, "panda_joint2", 0.5, 0.01, 0.01, 1.0).unwrap();

        let mut set = KinematicConstraintSet::new();
        set.push(Constraint::Joint(ok));
        assert!(set.decide(&posed).unwrap().satisfied);

        set.push(Constraint::Joint(bad));
        assert_eq!(set.len(), 2);
        assert!(!set.decide(&posed).unwrap().satisfied);
    }

    #[test]
    fn an_undecidable_visibility_member_is_reported_not_swallowed() {
        let model = panda_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let transforms = Transforms::new(model.model_frame()).unwrap();
        let vis = VisibilityConstraint::new(
            &model,
            &transforms,
            SensorSpec {
                frame_id: model.model_frame(),
                pose: Isometry3::translation(0.0, 0.0, 1.0),
                view_direction: SensorViewDirection::SensorZ,
            },
            TargetSpec {
                frame_id: model.model_frame(),
                pose: Isometry3::identity(),
            },
            8,
            VisibilityCriteria {
                target_radius: Some(0.02),
                ..Default::default()
            },
            1.0,
        )
        .unwrap();

        let mut set = KinematicConstraintSet::new();
        set.push(Constraint::Visibility(vis));
        let err = set.decide(&posed).unwrap_err();
        assert_eq!(err.index, 0);
    }
}
