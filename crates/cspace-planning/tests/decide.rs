// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Unit tests for the four constraint types' `decide()`/construction logic,
//! one case per invariant boundary (tolerance edge, mobile-vs-fixed frame,
//! Euler-singularity branch, `Option`-vs-`None` criterion) rather than one
//! per narrative scenario. `panda.urdf`/`panda.srdf` (copied from
//! `cspace_core::state`'s fixtures) supply a real, already-oracle-verified model
//! and FK — see `crates/cspace-core/tests/fixtures/constraints/model/panda_model_info.json`
//! and `crates/cspace-core/tests/fixtures/constraints/state/panda_fk.json` for that
//! verification. Oracle parity for `decide()` itself (the 2,000-combination
//! check `PORTING-PLAN.md` §5 Phase 5 requires) lives in a separate
//! integration test once the oracle gains a `constraints` op.

use std::fs;

use cspace_core::error::Error;
use cspace_core::geometry::{Isometry3, Mesh, Shape, Sphere, Transforms, UnitQuaternion, Vector3};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;

use cspace_planning::constraints::{
    Constraint, ConstraintEvaluationResult, JointConstraint, KinematicConstraintSet,
    OrientationConstraint, OrientationTolerance, PositionConstraint, SensorSpec,
    SensorViewDirection, TargetSpec, VisibilityConstraint, VisibilityCriteria,
};

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

/// PR2 has continuous joints (e.g. `fl_caster_rotation_joint`); panda and
/// fanuc do not, so the continuous-joint branch of [`JointConstraint`] needs
/// this fixture specifically.
fn pr2_model() -> RobotModel {
    let urdf_path = fixture_path("pr2.urdf");
    let srdf_path = fixture_path("pr2.srdf");
    let urdf_xml = fs::read_to_string(&urdf_path).expect("read pr2.urdf");
    let urdf = urdf_rs::read_file(&urdf_path).expect("parse pr2.urdf");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("parse pr2.srdf");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("build pr2 model")
}

fn sphere_region(radius: f64, pose: Isometry3) -> (Shape, Isometry3) {
    (Shape::Sphere(Sphere::new(radius).unwrap()), pose)
}

/// Asserts that a constructor failed *for the reason the test names*, not
/// merely that it failed.
///
/// Several rejection tests in this file sit next to a sibling that rejects
/// the same call from a different branch: `PositionConstraint::new`'s
/// `Body::from_shape(shape)?` error next to the `Ok(None)` error on the very
/// same line, an unresolvable `frame_id` next to an empty one. A bare
/// `.is_err()` cannot tell those apart, so routing one branch into the other
/// leaves every one of them green -- replacing that `?` with a fallback to
/// `None` (which lands the call in the sibling's error instead) keeps
/// `new_rejects_a_mesh_whose_body_construction_fails` passing, even though
/// the branch it exists to pin is then never taken. Matching on the rendered
/// error is what makes each test name the branch it actually exercises;
/// `constraint_sampler_manager.rs`'s `UnknownName` match is the same idea
/// where the variant alone is enough to discriminate.
#[track_caller]
fn assert_err_mentions<T: std::fmt::Debug>(result: Result<T, Error>, needle: &str) {
    let rendered = result
        .expect_err("expected this call to be rejected")
        .to_string();
    assert!(
        rendered.contains(needle),
        "expected the rejection to come from the branch that reports {needle:?}, got: {rendered}"
    );
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
    fn new_normalizes_non_positive_weight_to_one() {
        let model = panda_model();
        // PORTING-PLAN.md D14/§199 boundary cases: 0.0, EPS itself, and a
        // negative weight all normalize (upstream's own guard is `<=
        // epsilon`, not `< 0.0`); a value just above EPS passes through
        // unchanged.
        for weight in [0.0, f64::EPSILON, -1.0] {
            let c = JointConstraint::new(&model, "panda_joint1", 0.0, 0.1, 0.1, weight).unwrap();
            assert_eq!(c.weight(), 1.0, "weight {weight} should normalize to 1.0");
        }
        let above_eps = f64::EPSILON * 2.0;
        let c = JointConstraint::new(&model, "panda_joint1", 0.0, 0.1, 0.1, above_eps).unwrap();
        assert_eq!(
            c.weight(),
            above_eps,
            "weight just above EPS should pass through unchanged"
        );
    }

    #[test]
    fn new_rejects_unknown_joint() {
        let model = panda_model();
        assert!(matches!(
            JointConstraint::new(&model, "no_such_joint", 0.0, 0.1, 0.1, 1.0),
            Err(Error::UnknownName { kind: "joint", .. })
        ));
    }

    #[test]
    fn new_clamps_a_position_outside_the_joint_bounds() {
        // panda_joint1's effective max bound is its safety-controller soft
        // limit (2.8973), not the raw <limit> (2.9671) -- cspace_core::model
        // already prefers the soft limit when present (see
        // crates/cspace-core/src/model/joint/urdf.rs), matching upstream. Asking
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
        // `PositionConstraint::new`'s own `frame_id.trim().is_empty()`
        // branch, not the general unresolvable-frame one
        // `new_rejects_unresolvable_mobile_frame` covers.
        assert_err_mentions(
            PositionConstraint::new(
                &model,
                &transforms,
                "panda_link8",
                "",
                Vector3::zeros(),
                &[region],
                1.0,
            ),
            "no frame specified for position constraint",
        );
    }

    #[test]
    fn new_rejects_no_regions() {
        let model = panda_model();
        let transforms = tf(&model);
        assert_err_mentions(
            PositionConstraint::new(
                &model,
                &transforms,
                "panda_link8",
                model.model_frame(),
                Vector3::zeros(),
                &[],
                1.0,
            ),
            "needs at least one constraint region",
        );
    }

    /// A shape with no `bodies::Body` counterpart — `Body::from_shape`
    /// returns `Ok(None)` for [`Shape::Cone`], [`Shape::Plane`] and
    /// [`Shape::OcTree`] — makes [`PositionConstraint::new`] error rather
    /// than drop the region.
    ///
    /// The upstream branch this pins is **not** `:401-419`'s
    /// warn-and-`continue`. Those two skips are a pose array shorter than
    /// the shape array (`:400-403`, type-excluded here: `ConstraintRegion`
    /// pairs shape and pose in one `Vec`) and `constructShapeFromMsg`
    /// returning null on a malformed message (`:417-418`, also type-excluded
    /// here: a `Shape` value always exists). The branch that corresponds to
    /// this test is `:412-413`, which takes
    /// `createEmptyBodyFromShapeType(shape->type)` straight into
    /// `body->setDimensionsDirty(shape.get())` with no null check — so
    /// upstream does not skip a bodyless shape, it dereferences null on one.
    ///
    /// `Shape::Cone` is the variant that makes the comparison a real one:
    /// `constraint_region.primitives` is a `shape_msgs/SolidPrimitive[]`,
    /// `SolidPrimitive::CONE` is one of the four types
    /// `constructShapeFromMsg` builds (`shape_operations.cpp:101-106`), and
    /// nothing between it and `:412` filters it out, so a client can send a
    /// cone region and crash upstream. `Shape::Plane` and `Shape::OcTree`
    /// take the same port-side branch but cannot arrive on that upstream
    /// path at all (neither is a `SolidPrimitive` or a `Mesh`); they are
    /// asserted here because they are the same `from_shape` `None`, not
    /// because upstream has a matching case.
    #[test]
    fn new_rejects_a_shape_with_no_body_counterpart() {
        let model = panda_model();
        let transforms = tf(&model);
        let bodyless = [
            Shape::Cone(cspace_core::geometry::Cone::new(0.1, 0.2).unwrap()),
            Shape::Plane(cspace_core::geometry::Plane {
                a: 0.0,
                b: 0.0,
                c: 1.0,
                d: 0.0,
            }),
        ];
        for shape in bodyless {
            let region = (shape.clone(), Isometry3::identity());
            // The `Ok(None)` half of `Body::from_shape(shape)?` -- not the
            // construction-failure half `new_rejects_a_mesh_whose_body_construction_fails`
            // covers, which errors on the same line with a different message.
            assert_err_mentions(
                PositionConstraint::new(
                    &model,
                    &transforms,
                    "panda_link8",
                    model.model_frame(),
                    Vector3::zeros(),
                    &[region],
                    1.0,
                ),
                "has no bodies:: counterpart",
            );
        }
    }

    /// Distinct from `new_rejects_a_shape_with_no_body_counterpart` above:
    /// that test pins `Body::from_shape`'s `Ok(None)` case (a shape type
    /// with no `bodies::` counterpart at all). This test pins the other
    /// half of the same `Body::from_shape(shape)?` line -- a shape type
    /// that *does* have a counterpart, but whose construction genuinely
    /// fails. `Shape::Mesh` with zero vertices is the deterministic way to
    /// trigger that, but not via the qhull-failure branch this doc block
    /// previously named: `ConvexMesh::new` has two distinct
    /// `Error::Construct` sites (`bodies.rs`'s `build_mesh_data`), and a
    /// zero-vertex mesh takes the first one -- an explicit
    /// `mesh.vertices.is_empty()` guard (`bodies.rs:2508-2513`) that
    /// returns before the convex-hull computation (`try_convex_hull`) is
    /// ever attempted. Upstream has no equivalent pre-check:
    /// `ConvexMesh::useDimensions` calls straight into `qh_new_qhull`
    /// regardless of vertex count. The port's *second* `Error::Construct`
    /// site -- `try_convex_hull` itself failing -- is the actual analog of
    /// upstream's qhull-failure branch (`bodies.cpp:936-943`, which logs a
    /// warning and returns, leaving `mesh_data_` in its
    /// default-constructed empty, always-non-containing state -- a silent
    /// degradation, not a null deref); that site is not what this test
    /// exercises and has no test of its own yet. This port hard-errors on
    /// both of its sites instead of upstream's silent one (`bodies.rs`'s
    /// `ConvexMesh::new` doc comment, "this port surfaces the failure
    /// instead of building a body that can never contain anything"); the
    /// vertex-count guard this test pins has no upstream counterpart at
    /// all -- a deliberate divergence, not a defect, but one with no test
    /// at this crate's boundary before this: `cspace_core::geometry`'s own
    /// `convex_mesh_zero_vertex_is_an_error` pins `ConvexMesh::new` in
    /// isolation, not that the error actually propagates out through
    /// `PositionConstraint::new` rather than being swallowed somewhere in
    /// between.
    ///
    /// Reachability: a zero-vertex mesh specifically cannot arrive via a
    /// real `moveit_msgs::PositionConstraint` -- `constructShapeFromMsg`'s
    /// `Mesh` overload (`shape_operations.cpp:54-76`) already rejects an
    /// empty `vertices`/`triangles` array (`:56-60`) before a `Shape` exists,
    /// the same type-exclusion `new_rejects_a_shape_with_no_body_counterpart`
    /// already documents for `:417-418`. This test is exercising what the
    /// port's own `Shape::Mesh` variant allows to be constructed directly
    /// (nothing enforces non-empty vertices on the struct itself), not a
    /// wire-reachable case -- same relationship the existing test has to
    /// `Shape::Plane`/`Shape::OcTree`.
    #[test]
    fn new_rejects_a_mesh_whose_body_construction_fails() {
        let model = panda_model();
        let transforms = tf(&model);
        let region = (
            Shape::Mesh(Mesh {
                vertices: Vec::new(),
                triangles: Vec::new(),
                triangle_normals: None,
                vertex_normals: None,
            }),
            Isometry3::identity(),
        );
        assert_err_mentions(
            PositionConstraint::new(
                &model,
                &transforms,
                "panda_link8",
                model.model_frame(),
                Vector3::zeros(),
                &[region],
                1.0,
            ),
            "convex mesh body requires at least one vertex",
        );
    }

    /// `PositionConstraint::new`'s first fallible call, `model.link_model(link_name)?`,
    /// is a sibling of `new_rejects_unresolvable_mobile_frame`'s frame-id
    /// guard below -- both reach `Error::UnknownName`, one with `kind:
    /// "link"`, the other `kind: "frame"`. `planning_scene_validity.rs` (in
    /// `cspace_planners::sbp`) already had to be fixed to discriminate the
    /// two downstream; this crate had no test at all for the `"link"` side
    /// before this one.
    #[test]
    fn new_rejects_unknown_link() {
        let model = panda_model();
        let transforms = tf(&model);
        let region = sphere_region(0.01, Isometry3::identity());
        assert_err_mentions(
            PositionConstraint::new(
                &model,
                &transforms,
                "no_such_link",
                model.model_frame(),
                Vector3::zeros(),
                &[region],
                1.0,
            ),
            r#"no link named "no_such_link""#,
        );
    }

    #[test]
    fn new_rejects_unresolvable_mobile_frame() {
        let model = panda_model();
        let transforms = tf(&model);
        let region = sphere_region(0.01, Isometry3::identity());
        assert_err_mentions(
            PositionConstraint::new(
                &model,
                &transforms,
                "panda_link8",
                "no_such_frame",
                Vector3::zeros(),
                &[region],
                1.0,
            ),
            r#"no frame named "no_such_frame""#,
        );
    }

    #[test]
    fn new_normalizes_non_positive_weight_to_one() {
        let model = panda_model();
        let transforms = tf(&model);
        // PORTING-PLAN.md D14/§199 boundary cases: 0.0, EPS itself, and a
        // negative weight all normalize; a value just above EPS passes
        // through unchanged.
        for weight in [0.0, f64::EPSILON, -1.0] {
            let region = sphere_region(0.01, Isometry3::identity());
            let c = PositionConstraint::new(
                &model,
                &transforms,
                "panda_link8",
                model.model_frame(),
                Vector3::zeros(),
                &[region],
                weight,
            )
            .unwrap();
            assert_eq!(c.weight(), 1.0, "weight {weight} should normalize to 1.0");
        }
        let above_eps = f64::EPSILON * 2.0;
        let region = sphere_region(0.01, Isometry3::identity());
        let c = PositionConstraint::new(
            &model,
            &transforms,
            "panda_link8",
            model.model_frame(),
            Vector3::zeros(),
            &[region],
            above_eps,
        )
        .unwrap();
        assert_eq!(
            c.weight(),
            above_eps,
            "weight just above EPS should pass through unchanged"
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
        assert_err_mentions(
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
            ),
            r#"no frame named "no_such_frame""#,
        );
    }

    /// `kinematic_constraint.cpp:618-619` only warns on an empty
    /// `frame_id` and falls through to build a constraint anyway (its
    /// `decide()` then resolves the frame to identity every time via
    /// `getFrameTransform("")`) -- this port rejects it instead (this
    /// type's "Deviation from upstream" doc comment). An empty string is
    /// not merely "unresolvable" the way `new_rejects_unresolvable_frame`'s
    /// `"no_such_frame"` is: upstream itself branches on `.empty()`
    /// specifically, separately from its general fixed/mobile frame
    /// resolution, so this needs its own case to prove the port's reject
    /// covers that exact branch, not just the general one.
    #[test]
    fn new_rejects_empty_frame_id() {
        let model = panda_model();
        let transforms = tf(&model);
        // Unlike `PositionConstraint`, this type has no dedicated
        // empty-`frame_id` branch: the empty string falls through the same
        // general resolution `new_rejects_unresolvable_frame` exercises and
        // is reported by name. Asserting the name is what shows *which*
        // branch answered -- the doc above claims this case proves the port
        // covers the empty-string branch specifically, and it does so only
        // in the sense that the general branch swallows it.
        assert_err_mentions(
            OrientationConstraint::new(
                &model,
                &transforms,
                "panda_link8",
                "",
                UnitQuaternion::identity(),
                OrientationTolerance::XyzEuler {
                    x: 0.01,
                    y: 0.01,
                    z: 0.01,
                },
                1.0,
            ),
            r#"no frame named """#,
        );
    }

    #[test]
    fn new_normalizes_non_positive_weight_to_one() {
        let model = panda_model();
        let transforms = tf(&model);
        // PORTING-PLAN.md D14/§199 boundary cases: 0.0, EPS itself, and a
        // negative weight all normalize; a value just above EPS passes
        // through unchanged.
        for weight in [0.0, f64::EPSILON, -1.0] {
            let c = OrientationConstraint::new(
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
                weight,
            )
            .unwrap();
            assert_eq!(c.weight(), 1.0, "weight {weight} should normalize to 1.0");
        }
        let above_eps = f64::EPSILON * 2.0;
        let c = OrientationConstraint::new(
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
            above_eps,
        )
        .unwrap();
        assert_eq!(
            c.weight(),
            above_eps,
            "weight just above EPS should pass through unchanged"
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
        assert_eq!(c.decide(&posed), ConstraintEvaluationResult::new(true, 0.0));
    }

    /// Sensor and target at the same point, so the range direction is the
    /// zero vector.
    ///
    /// Upstream `VisibilityConstraint::decide` normalizes it with Eigen's
    /// `.normalized()`, which returns the input unchanged when
    /// `squaredNorm()` is zero rather than dividing — measured inside this
    /// repo's oracle image (Eigen 3.4.0, the version moveit2 builds
    /// against): `Vector3d::Zero().normalized()` is `[0, 0, 0]`, so `dp` is
    /// `0`, `acos(dp)` is `pi/2`, and `max_range_angle_ < pi/2` fires for any
    /// ordinary criterion.
    ///
    /// nalgebra's `.normalize()` has no such guard — it is
    /// `unscale(self.norm())`, so a zero vector gives `0.0 / 0.0`. The whole
    /// check then evaporated: `dp` was NaN, `dp < 0.0` was false, `acos(dp)`
    /// was NaN, and `max_range_angle < NaN` was false too, so a degenerate
    /// pose was reported *satisfied* where upstream reports it violated.
    #[test]
    fn a_zero_length_range_direction_is_violated_not_satisfied() {
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
                pose: Isometry3::identity(),
                view_direction: SensorViewDirection::SensorZ,
            },
            TargetSpec {
                frame_id: model.model_frame(),
                pose: Isometry3::identity(),
            },
            8,
            VisibilityCriteria {
                max_range_angle: Some(0.1),
                ..Default::default()
            },
            1.0,
        )
        .unwrap();
        assert!(c.enabled());
        assert!(!c.decide(&posed).satisfied);
    }

    /// The demonstrated opposite: the same constraint with the sensor moved
    /// off the target along its own view axis has a well-defined direction,
    /// `acos(dp)` is `0`, and `0.1 < 0` is false — so it stays satisfied.
    /// Without this, the test above would also pass on a port that simply
    /// reported every range check violated.
    #[test]
    fn a_range_direction_along_the_view_axis_is_still_satisfied() {
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
                pose: Isometry3::translation(0.0, 0.0, -1.0),
                view_direction: SensorViewDirection::SensorZ,
            },
            TargetSpec {
                frame_id: model.model_frame(),
                pose: Isometry3::identity(),
            },
            8,
            VisibilityCriteria {
                max_range_angle: Some(0.1),
                ..Default::default()
            },
            1.0,
        )
        .unwrap();
        assert!(c.enabled());
        assert!(c.decide(&posed).satisfied);
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
        assert!(!c.decide(&posed).satisfied);
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
        assert!(!c.decide(&posed).satisfied);
    }

    // `panda.urdf`'s `<collision>` geometry is 100% `<mesh>` (STL) -- and
    // `cspace_core::model`'s URDF loader skips mesh collision geometry entirely
    // (see `cspace-collision`'s `parry.rs` module doc, "world objects are
    // never padded or scaled" section and `scaled_padded_shape`'s doc), so a
    // panda `RobotModel` has *zero* parry-representable collision geometry:
    // every one of its links produces no `PosedBody` at all, and the cone
    // check can never report a hit against it regardless of geometry. Both
    // cone tests below use `pr2_model()` instead, whose `base_bellow_link`
    // has a real primitive (`<box size="0.05 0.37 0.3"/>`) collision shape.

    #[test]
    fn accessors_report_back_every_constructor_argument() {
        use approx::assert_relative_eq;

        let model = panda_model();
        let transforms = tf(&model);
        let sensor_pose = Isometry3::translation(0.1, 0.2, 0.3);
        let target_pose = Isometry3::translation(0.4, 0.5, 0.6);
        let c = VisibilityConstraint::new(
            &model,
            &transforms,
            SensorSpec {
                frame_id: model.model_frame(),
                pose: sensor_pose,
                view_direction: SensorViewDirection::SensorX,
            },
            TargetSpec {
                frame_id: model.model_frame(),
                pose: target_pose,
            },
            8,
            VisibilityCriteria {
                target_radius: Some(0.05),
                max_view_angle: Some(0.2),
                max_range_angle: Some(0.3),
            },
            2.5,
        )
        .unwrap();

        // `frame_id == model.model_frame()` on both ends, so the fixed-frame
        // transform each pose goes through at construction is the identity
        // -- these must come back unchanged, not merely "close".
        assert_relative_eq!(c.sensor(), sensor_pose, epsilon = 1e-12);
        assert_relative_eq!(c.target(), target_pose, epsilon = 1e-12);
        // Verbatim enum, not a position-based cast back from an integer --
        // see `SensorViewDirection`'s own doc comment on why the wire
        // encoding is reversed relative to this enum's declaration order.
        assert_eq!(c.sensor_view_direction(), SensorViewDirection::SensorX);
        assert_eq!(c.target_radius(), Some(0.05));
        assert_eq!(c.max_view_angle(), Some(0.2));
        assert_eq!(c.max_range_angle(), Some(0.3));
        assert_eq!(c.weight(), 2.5);
    }

    #[test]
    fn cone_far_from_the_robot_is_satisfied() {
        // Sensor and target both 10m away from base_bellow_link along +X --
        // far outside any pr2 link's reach, so the cone can't intersect one.
        let model = pr2_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let link_pos = posed
            .global_link_transform("base_bellow_link")
            .unwrap()
            .translation
            .vector;

        let transforms = tf(&model);
        let c = VisibilityConstraint::new(
            &model,
            &transforms,
            SensorSpec {
                frame_id: model.model_frame(),
                pose: Isometry3::translation(link_pos.x + 10.0, link_pos.y, link_pos.z + 1.0),
                view_direction: SensorViewDirection::SensorZ,
            },
            TargetSpec {
                frame_id: model.model_frame(),
                pose: Isometry3::translation(link_pos.x + 10.0, link_pos.y, link_pos.z),
            },
            8,
            VisibilityCriteria {
                target_radius: Some(0.1),
                ..Default::default()
            },
            1.0,
        )
        .unwrap();
        assert!(c.enabled());
        assert_eq!(c.decide(&posed), ConstraintEvaluationResult::new(true, 0.0));
    }

    #[test]
    fn cone_through_a_robot_link_is_violated() {
        // Sensor 1m above base_bellow_link's own origin, target *at* that
        // origin: the cone's base is a filled disc (its two "base triangle"
        // fans reach from the target-center vertex out to the rim), so
        // putting the target exactly on the link plants that filled cap
        // disc through its 0.05x0.37x0.3 box -- a cap disc built somewhere
        // else along the sensor-target axis would only place the cone's
        // hollow lateral shell near the box, which can pass clean through a
        // box small enough to fit inside without ever touching that shell.
        let model = pr2_model();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let link_pos = posed
            .global_link_transform("base_bellow_link")
            .unwrap()
            .translation
            .vector;

        let transforms = tf(&model);
        let c = VisibilityConstraint::new(
            &model,
            &transforms,
            SensorSpec {
                frame_id: model.model_frame(),
                pose: Isometry3::translation(link_pos.x, link_pos.y, link_pos.z + 1.0),
                view_direction: SensorViewDirection::SensorZ,
            },
            TargetSpec {
                frame_id: model.model_frame(),
                pose: Isometry3::translation(link_pos.x, link_pos.y, link_pos.z),
            },
            8,
            VisibilityCriteria {
                target_radius: Some(0.5),
                ..Default::default()
            },
            1.0,
        )
        .unwrap();
        assert!(c.enabled());
        let r = c.decide(&posed);
        assert!(
            !r.satisfied,
            "expected the cone's base cap to hit base_bellow_link"
        );
        assert!(r.distance > 0.0);
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
    fn negative_target_radius_activates_at_its_magnitude_but_negative_angles_stay_inactive() {
        // kinematic_constraint.cpp:818 fabs()'s target_radius_ before the
        // >eps gate (a negative wire value still activates the criterion,
        // at |value|); :879-880 assigns max_view_angle_/max_range_angle_
        // straight from the message with no fabs() before the same gate (a
        // negative wire value fails ">eps" and leaves the criterion
        // inactive). The two must not be normalized identically.
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
                target_radius: Some(-0.5),
                max_view_angle: Some(-0.1),
                max_range_angle: Some(-0.2),
            },
            1.0,
        )
        .unwrap();
        assert_eq!(
            c.target_radius(),
            Some(0.5),
            "negative target_radius should activate at its magnitude"
        );
        assert_eq!(
            c.max_view_angle(),
            None,
            "negative max_view_angle should stay inactive, not activate at its magnitude"
        );
        assert_eq!(
            c.max_range_angle(),
            None,
            "negative max_range_angle should stay inactive, not activate at its magnitude"
        );
        assert!(c.enabled(), "target_radius alone should still enable it");
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

    #[test]
    fn new_normalizes_non_positive_weight_to_one() {
        let model = panda_model();
        let transforms = tf(&model);
        // PORTING-PLAN.md D14/§199 boundary cases: 0.0, EPS itself, and a
        // negative weight all normalize; a value just above EPS passes
        // through unchanged.
        for weight in [0.0, f64::EPSILON, -1.0] {
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
                VisibilityCriteria::default(),
                weight,
            )
            .unwrap();
            assert_eq!(c.weight(), 1.0, "weight {weight} should normalize to 1.0");
        }
        let above_eps = f64::EPSILON * 2.0;
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
            VisibilityCriteria::default(),
            above_eps,
        )
        .unwrap();
        assert_eq!(
            c.weight(),
            above_eps,
            "weight just above EPS should pass through unchanged"
        );
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
        let r = set.decide(&posed);
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
        assert!(set.decide(&posed).satisfied);

        set.push(Constraint::Joint(bad));
        assert_eq!(set.len(), 2);
        assert!(!set.decide(&posed).satisfied);
    }
}
