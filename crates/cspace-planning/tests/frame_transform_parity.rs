// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `frame_transform` op, ground
//! truth for [`PlanningScene::frame_transform`]/[`PlanningScene::knows_frame_transform`].
//!
//! Both sides are driven from the committed `panda_frame_transform_request.json`
//! (the oracle request) and its unedited `panda_frame_transform_response.json`
//! (the oracle's real answer) — see `world_parity.rs` in `cspace-collision`
//! for why sharing one request file this way matters: it is the only way to
//! guarantee the Rust-built scene and the oracle's real
//! `planning_scene::PlanningScene` start from the identical scenario.
//!
//! The scenario covers every tier of the ladder at once: `panda_link0` (a
//! link name), `world` (the model frame -- panda's virtual joint is
//! `floating`, so `model_frame` is genuinely `"world"`, not the root link,
//! unlike this crate's own fixed-base unit-test fixture), `box`/`box/tip`
//! (an attached-body id and subframe), `table`/`a`/`a/b` (world objects),
//! `a/b/c` (the documented `knowsTransform`/`getTransform` ambiguity --
//! object `a/b` has a subframe `c`, and its id is itself a `/`-prefix of
//! sibling object `a`'s name), and `nothing` (resolves in no tier).
//!
//! The ambiguity case is why this test exists rather than a Rust-only unit
//! test: it establishes what `PlanningScene`'s ladder does with a name where
//! upstream's own `knowsFrameTransform` and `getFrameTransform` disagree,
//! observed from the real oracle rather than re-derived from reading
//! `world.cpp`/`planning_scene.cpp`.
//!
//! `nothing` is the one query this test does not compare `transform` on:
//! upstream `PlanningScene::getFrameTransform`'s documented contract is
//! "return identity when no transform is available, use
//! `knowsFrameTransform` to tell the two apart" (`planning_scene.hpp:204`).
//! [`PlanningScene::frame_transform`] instead returns
//! [`cspace_core::error::Error::UnknownName`] for a name resolving in no tier, the
//! idiomatic `Result` shape every other lookup in this port already uses
//! (see e.g. [`cspace_collision::World::get_transform`], upstream's own
//! *throwing* single-arg `getTransform` overload) -- a deliberate deviation
//! from this one method's silent-identity upstream contract, not an
//! oversight.

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::sync::Arc;

use approx::assert_relative_eq;
use serde::Deserialize;

use cspace_collision::World;
use cspace_core::geometry::{Cuboid, Isometry3, Shape};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_planning::scene::PlanningScene;
use nalgebra::{Matrix3, Translation3, UnitQuaternion};

#[derive(Deserialize)]
struct RequestShape {
    #[serde(rename = "type")]
    _kind: String,
}

#[derive(Deserialize)]
struct RequestAttachedBody {
    id: String,
    link_name: String,
    shapes: Vec<RequestShape>,
    shape_poses: Vec<[f64; 16]>,
    #[serde(default)]
    subframes: BTreeMap<String, [f64; 16]>,
}

#[derive(Deserialize)]
struct RequestObject {
    id: String,
    pose: [f64; 16],
    #[serde(default)]
    subframes: BTreeMap<String, [f64; 16]>,
}

#[derive(Deserialize)]
struct RequestFixture {
    attached_bodies: Vec<RequestAttachedBody>,
    objects: Vec<RequestObject>,
    queries: Vec<String>,
}

#[derive(Deserialize)]
struct QueryDump {
    name: String,
    knows_transform: bool,
    transform: [f64; 16],
}

#[derive(Deserialize)]
struct ResponseResult {
    queries: Vec<QueryDump>,
}

#[derive(Deserialize)]
struct OracleResponse {
    result: ResponseResult,
}

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/scene/{}"),
        file_name
    )
}

fn load_request() -> RequestFixture {
    let path = fixture_path("panda_frame_transform_request.json");
    let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
}

fn load_response() -> ResponseResult {
    let path = fixture_path("panda_frame_transform_response.json");
    let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let response: OracleResponse =
        serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"));
    response.result
}

fn build_model() -> RobotModel {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.urdf");
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.srdf");
    let urdf_xml = fs::read_to_string(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let urdf = urdf_rs::read_file(path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

fn srdf() -> SrdfModel {
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.srdf");
    SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse")
}

/// Row-major 4x4, matching `toRowMajor4x4`/`fromRowMajor4x4` in `oracle.cpp`.
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
        epsilon = 1e-6,
        max_relative = 1e-6
    );
}

/// A dummy 0.1m cube: only pose composition is under test here, not shape
/// geometry, matching `world_parity.rs`'s own dummy-sphere choice.
fn cube() -> Arc<Shape> {
    Arc::new(Shape::Cuboid(Cuboid::new(0.1, 0.1, 0.1).unwrap()))
}

/// Replay `request`'s `objects`/`attached_bodies` through
/// [`PlanningScene`]'s own public API -- the single source for "which
/// objects, shapes, poses and subframes" this test and the committed oracle
/// request share.
///
/// The oracle's own world objects are built with `World::addToObject(id,
/// pose, ...)` -- the object itself carries `pose`, shapes at `Identity`
/// relative to it. [`PlanningScene::add_shape`] instead always creates the
/// object at `Isometry3::identity()` and poses the *shape* (upstream
/// `World::addToObject` overload it wraps, `shape_pose` relative to the
/// object) -- so reproducing the same object pose here goes through
/// [`cspace_collision::World`]'s own lower-level `add_to_object` directly
/// (via [`PlanningScene::with_world`]), not [`PlanningScene::add_shape`].
fn build_scene<'m>(
    model: &'m RobotModel,
    srdf: &SrdfModel,
    request: &RequestFixture,
) -> PlanningScene<'m> {
    let mut world = World::new();
    for object in &request.objects {
        let pose = isometry_from_row_major(&object.pose);
        world
            .add_to_object(&object.id, pose, &[cube()], &[Isometry3::identity()])
            .unwrap_or_else(|| panic!("fresh object {} must notify", object.id));
        if !object.subframes.is_empty() {
            let subframes: BTreeMap<String, Isometry3> = object
                .subframes
                .iter()
                .map(|(name, pose)| (name.clone(), isometry_from_row_major(pose)))
                .collect();
            assert!(world.set_subframes_of_object(&object.id, subframes));
        }
    }

    let mut scene = PlanningScene::with_world(model, srdf, world);

    for attached in &request.attached_bodies {
        let shapes: Vec<Arc<Shape>> = attached.shapes.iter().map(|_| cube()).collect();
        let shape_poses: Vec<Isometry3> = attached
            .shape_poses
            .iter()
            .map(isometry_from_row_major)
            .collect();
        let subframes: BTreeMap<String, Isometry3> = attached
            .subframes
            .iter()
            .map(|(name, pose)| (name.clone(), isometry_from_row_major(pose)))
            .collect();
        scene
            .attach_new(
                &attached.id,
                &attached.link_name,
                shapes,
                shape_poses,
                BTreeSet::new(),
                subframes,
            )
            .unwrap_or_else(|e| panic!("attach {}: {e}", attached.id));
    }

    scene
}

#[test]
fn panda_frame_transform_matches_the_oracle() {
    let model = build_model();
    let srdf = srdf();
    let request = load_request();
    let mut scene = build_scene(&model, &srdf, &request);
    let response = load_response();

    assert_eq!(
        request.queries,
        response
            .queries
            .iter()
            .map(|q| q.name.clone())
            .collect::<Vec<_>>(),
        "request/response query lists must be the same list, in the same order"
    );

    for query in &response.queries {
        assert_eq!(
            scene.knows_frame_transform(&query.name),
            query.knows_transform,
            "knows_frame_transform({})",
            query.name
        );

        if query.name == "nothing" {
            // Upstream returns identity here; this port's `Result`-shaped
            // API errors instead -- see this file's module doc.
            assert!(scene.frame_transform(&query.name).is_err());
            continue;
        }
        let actual = scene
            .frame_transform(&query.name)
            .unwrap_or_else(|e| panic!("frame_transform({}): {e}", query.name));
        assert_isometry_eq(actual, &query.transform);
    }

    // The oracle's own run confirmed the documented ambiguity concretely,
    // through the *scene*'s ladder, not just `World`'s: "a/b/c" is
    // unresolved by `knows_frame_transform` but still resolved by
    // `frame_transform`.
    let ambiguous = response
        .queries
        .iter()
        .find(|q| q.name == "a/b/c")
        .expect("fixture must carry the ambiguity query");
    assert!(!ambiguous.knows_transform);
    assert!(scene.frame_transform("a/b/c").is_ok());
}
