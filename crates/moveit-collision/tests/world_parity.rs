// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `world` op.
//!
//! `World` has no `RobotModel` dependency, so unlike `acm_parity.rs` there is
//! no SRDF driving the scenario: the objects, shapes and subframes below are
//! authored directly in this file with `moveit_geometry` types and replayed
//! through [`moveit_collision::World`]'s own API. What is never
//! hand-transcribed is the *expected* output — `tests/fixtures/world_response.json`
//! is the oracle's own, unedited response (built from the identical scenario
//! against the real `collision_detection::World`, dumped by the oracle's
//! `world` op in `tools/moveit-oracle/src/oracle.cpp`) — so a composition
//! convention this port got wrong shows up as a fixture mismatch instead of
//! being baked into a hand-computed expected value that could carry the same
//! mistake.
//!
//! The scenario covers two things a unit test authored purely from reading
//! `world.cpp` cannot: whether `nalgebra::Isometry3` composition
//! (`object_pose * shape_pose`, `object_pose * subframe_pose`) agrees with
//! `Eigen::Isometry3d`'s under an actual rotation (object `shelf`, a 90°
//! rotation about Z), and the `knowsTransform`/`getTransform` ambiguity
//! (`a`/`a/b` — see `world.rs`'s module docs, deviation 8) as observed from
//! the real, unmodified upstream methods rather than re-derived from this
//! crate's own reading of them.

use std::collections::BTreeMap;
use std::f64::consts::FRAC_PI_2;
use std::fs;
use std::sync::Arc;

use approx::assert_relative_eq;
use serde::Deserialize;

use moveit_collision::World;
use moveit_geometry::{Isometry3, Shape, Sphere};
use nalgebra::{Matrix3, Translation3, UnitQuaternion, Vector3};

#[derive(Deserialize)]
struct ShapeDump {
    pose: [f64; 16],
    global_pose: [f64; 16],
}

#[derive(Deserialize)]
struct SubframeDump {
    pose: [f64; 16],
    global_pose: [f64; 16],
}

#[derive(Deserialize)]
struct ObjectDump {
    id: String,
    pose: [f64; 16],
    shapes: Vec<ShapeDump>,
    subframes: BTreeMap<String, SubframeDump>,
}

#[derive(Deserialize)]
struct QueryDump {
    name: String,
    knows_transform: bool,
    #[serde(default)]
    transform: Option<[f64; 16]>,
}

#[derive(Deserialize)]
struct WorldDump {
    objects: Vec<ObjectDump>,
    queries: Vec<QueryDump>,
}

#[derive(Deserialize)]
struct OracleResponse {
    result: WorldDump,
}

fn load_fixture() -> WorldDump {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/world_response.json"
    );
    let raw = fs::read_to_string(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let response: OracleResponse =
        serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"));
    response.result
}

/// Row-major 4x4, matching `toRowMajor4x4` in `oracle.cpp`.
fn isometry_from_row_major(m: &[f64; 16]) -> Isometry3 {
    let rotation = Matrix3::new(m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]);
    let translation = Translation3::new(m[3], m[7], m[11]);
    Isometry3::from_parts(translation, UnitQuaternion::from_matrix(&rotation))
}

fn assert_isometry_eq(actual: Isometry3, expected_row_major: &[f64; 16]) {
    let expected = isometry_from_row_major(expected_row_major);
    assert_relative_eq!(
        actual.to_homogeneous(),
        expected.to_homogeneous(),
        epsilon = 1e-9,
        max_relative = 1e-9
    );
}

fn sphere() -> Arc<Shape> {
    Arc::new(Shape::Sphere(Sphere::new(0.1).unwrap()))
}

/// The same scenario the oracle's `world_response.json` fixture was recorded
/// from — see this module's own doc comment for the exact request JSON that
/// produced it (`shelf` rotated 90° about Z with two shapes and a subframe;
/// `a`/`a/b` set up for the subframe-name-collision ambiguity).
fn build_world() -> World {
    let mut world = World::new();

    let shelf_pose = Isometry3::from_parts(
        Translation3::new(1.0, 2.0, 3.0),
        UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2),
    );
    world
        .add_to_object(
            "shelf",
            shelf_pose,
            &[sphere(), sphere()],
            &[
                Isometry3::translation(0.0, 0.0, 1.0),
                Isometry3::translation(1.0, 0.0, 0.0),
            ],
        )
        .expect("fresh object with shapes must notify");
    let mut shelf_subframes = BTreeMap::new();
    shelf_subframes.insert("tip".to_owned(), Isometry3::translation(0.0, 1.0, 0.0));
    assert!(world.set_subframes_of_object("shelf", shelf_subframes));

    world.set_object_pose("a", Isometry3::identity());

    world
        .add_to_object(
            "a/b",
            Isometry3::translation(1.0, 0.0, 0.0),
            &[sphere()],
            &[Isometry3::translation(0.0, 0.0, 1.0)],
        )
        .expect("fresh object with a shape must notify");
    let mut ab_subframes = BTreeMap::new();
    ab_subframes.insert("c".to_owned(), Isometry3::translation(0.0, 1.0, 0.0));
    assert!(world.set_subframes_of_object("a/b", ab_subframes));

    world
}

#[test]
fn world_matches_oracle() {
    let world = build_world();
    let fixture = load_fixture();

    for object_dump in &fixture.objects {
        let obj = world
            .get_object(&object_dump.id)
            .unwrap_or_else(|| panic!("missing object {}", object_dump.id));
        assert_isometry_eq(obj.pose(), &object_dump.pose);
        assert_eq!(
            obj.shapes().len(),
            object_dump.shapes.len(),
            "{} shape count",
            object_dump.id
        );
        for (shape, shape_dump) in obj.shapes().iter().zip(&object_dump.shapes) {
            assert_isometry_eq(shape.pose(), &shape_dump.pose);
            assert_isometry_eq(shape.global_pose(), &shape_dump.global_pose);
        }
        for (name, subframe_dump) in &object_dump.subframes {
            let pose = obj
                .subframe_pose(name)
                .unwrap_or_else(|| panic!("missing subframe {name} on {}", object_dump.id));
            let global_pose = obj
                .global_subframe_pose(name)
                .unwrap_or_else(|| panic!("missing global subframe {name} on {}", object_dump.id));
            assert_isometry_eq(pose, &subframe_dump.pose);
            assert_isometry_eq(global_pose, &subframe_dump.global_pose);
        }
    }

    for query in &fixture.queries {
        assert_eq!(
            world.knows_transform(&query.name),
            query.knows_transform,
            "knows_transform({})",
            query.name
        );
        match &query.transform {
            Some(expected) => {
                let actual = world
                    .try_get_transform(&query.name)
                    .unwrap_or_else(|| panic!("expected a transform for {}", query.name));
                assert_isometry_eq(actual, expected);
            }
            None => assert!(
                world.try_get_transform(&query.name).is_none(),
                "expected no transform for {}",
                query.name
            ),
        }
    }

    // The oracle's own run confirmed the documented ambiguity concretely:
    // "a/b/c" is unresolved by knows_transform but resolved by get_transform.
    let ambiguous = fixture
        .queries
        .iter()
        .find(|q| q.name == "a/b/c")
        .expect("fixture must carry the ambiguity query");
    assert!(!ambiguous.knows_transform);
    assert!(ambiguous.transform.is_some());
}
