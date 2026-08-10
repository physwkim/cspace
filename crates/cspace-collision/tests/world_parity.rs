// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `world` op.
//!
//! `World` has no `RobotModel` dependency, so unlike `acm_parity.rs` there is
//! no SRDF to build the scenario from. Instead both sides of this test are
//! driven from `tests/fixtures/world_request.json` — a `world` op request,
//! committed alongside the oracle's own unedited response in
//! `world_response.json` — so the Rust-built [`cspace_collision::World`] and
//! the oracle's real `collision_detection::World` are guaranteed to be built
//! from the identical scenario: there is exactly one copy of "which objects,
//! shapes, poses and subframes" in the tree, not a Rust transcription and a
//! JSON transcription that could quietly drift apart. Only the *expected*
//! output is oracle ground truth; the request itself is not something either
//! side gets to assert about, it is the shared input.
//!
//! The scenario covers two things a unit test authored purely from reading
//! `world.cpp` cannot: whether `nalgebra::Isometry3` composition
//! (`object_pose * shape_pose`, `object_pose * subframe_pose`) agrees with
//! `Eigen::Isometry3d`'s under an actual rotation (object `shelf`, a 90°
//! rotation about Z), and the `knowsTransform`/`getTransform` ambiguity
//! (`a`/`a/b` — see `world.rs`'s module docs, deviation 8) as observed from
//! the real, unmodified upstream methods rather than re-derived from this
//! crate's own reading of them.
//!
//! `tests/fixtures/oracle-models.json`'s `"world"` entry names
//! `octree_world_robot.{urdf,srdf}` for `tools/ci/verify-fixture-replay.sh`
//! to replay this fixture against — an arbitrary but already-present choice,
//! not a dependency: the oracle binary always requires a `--urdf`/`--srdf`
//! pair to start (`Oracle`'s constructor parses one unconditionally, before
//! any op is read), but the `world` op itself never touches the model this
//! module doc already establishes has no bearing on `World`.

use std::collections::BTreeMap;
use std::fs;
use std::sync::Arc;

use approx::assert_relative_eq;
use serde::Deserialize;

use cspace_collision::World;
use cspace_geometry::{Isometry3, Shape, Sphere};
use cspace_test_support::isometry_from_row_major;

#[derive(Deserialize)]
struct RequestObject {
    id: String,
    pose: [f64; 16],
    #[serde(default)]
    shape_poses: Vec<[f64; 16]>,
    #[serde(default)]
    subframes: BTreeMap<String, [f64; 16]>,
}

#[derive(Deserialize)]
struct RequestFixture {
    objects: Vec<RequestObject>,
    queries: Vec<String>,
}

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

fn read_fixture(name: &str) -> String {
    let path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        name
    );
    fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

fn load_request() -> RequestFixture {
    let raw = read_fixture("world_request.json");
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse world_request.json: {e}"))
}

fn load_response() -> WorldDump {
    let raw = read_fixture("world_response.json");
    let response: OracleResponse =
        serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse world_response.json: {e}"));
    response.result
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

/// A dummy 0.1m sphere, matching the oracle's own choice in `oracle.cpp`:
/// only pose composition is under test here, not shape geometry.
fn sphere() -> Arc<Shape> {
    Arc::new(Shape::Sphere(Sphere::new(0.1).unwrap()))
}

/// Replay `request` through [`World`]'s own API — the single source for
/// "which objects, shapes, poses and subframes" this test and the committed
/// oracle request share.
fn build_world(request: &RequestFixture) -> World {
    let mut world = World::new();
    for object in &request.objects {
        let pose = isometry_from_row_major(&object.pose);
        if object.shape_poses.is_empty() {
            world.set_object_pose(&object.id, pose);
        } else {
            let shapes: Vec<Arc<Shape>> = object.shape_poses.iter().map(|_| sphere()).collect();
            let shape_poses: Vec<Isometry3> = object
                .shape_poses
                .iter()
                .map(isometry_from_row_major)
                .collect();
            world
                .add_to_object(&object.id, pose, &shapes, &shape_poses)
                .unwrap_or_else(|| panic!("fresh object {} with shapes must notify", object.id));
        }
        if !object.subframes.is_empty() {
            let subframes: BTreeMap<String, Isometry3> = object
                .subframes
                .iter()
                .map(|(name, pose)| (name.clone(), isometry_from_row_major(pose)))
                .collect();
            assert!(world.set_subframes_of_object(&object.id, subframes));
        }
    }
    world
}

#[test]
fn world_matches_oracle() {
    let request = load_request();
    let world = build_world(&request);
    let fixture = load_response();

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

    assert_eq!(
        request.queries,
        fixture
            .queries
            .iter()
            .map(|q| q.name.clone())
            .collect::<Vec<_>>(),
        "request/response query lists must be the same list, in the same order"
    );
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
