// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test for wiring [`cspace_core::geometry::shapes::OcTree`] into a posed
//! collision-world object, against the real moveit2 oracle's new
//! `octree_in_world` op.
//!
//! This is the round-2 gap this port's task flagged: MoveIt represents
//! sensor-derived obstacles as an octomap *inside the collision world*, and
//! nothing prior to this exercised the connection between the two.
//! `crates/cspace-collision`'s `World` port (owned by another worker this
//! round) is not touched here -- instead, this test replicates the exact
//! arithmetic a collision backend must perform: compose the object's pose
//! with the shape's own sub-pose, invert that composed pose to map a
//! world-frame query point into the octree's local frame, then query
//! occupancy there. `collision_detection_fcl`'s `fcl::OcTreed` wrap and
//! `collision_env_distance_field`'s `PosedBodyPointDecomposition(octree)`
//! each perform this same pose-to-local-frame step through their own
//! backend-specific mechanism (see `shapes.rs`'s module docs, "Who consumes
//! Shape::OcTree" section, for the full consumer analysis) -- this test
//! proves this crate's `OcTree`/`Isometry3` API already carries everything
//! that step needs, without porting either backend.
//!
//! `tests/fixtures/octree_in_world_{request,response}.json` is the request
//! array fed to the oracle's `octree_in_world` op and its unedited response,
//! captured against `moveit-rs/oracle:round2-octree` (see this change's
//! commit body for the exact build and the image's relationship to
//! `tools/moveit-oracle/build.sh`). Four scenarios: an identity-posed object
//! (occupied point, a miss, and a point outside the tree entirely), a
//! pure-translation object pose, a rotation-plus-translation object pose,
//! and an identity object pose with a non-identity per-shape sub-pose --
//! each scenario also queries the *raw, untransformed* local coordinate in
//! world space to confirm the pose composition is actually load-bearing
//! (query id, `mapped: false`) rather than the test accidentally passing
//! because the tree is queried in its own frame regardless of pose.

use std::fs;
use std::sync::Arc;

use cspace_core::geometry::shapes::OcTree as ShapeOcTree;
use cspace_core::geometry::{Isometry3, Shape};
use cspace_core::octomap::OcTree;
use cspace_core::test_support::isometry_from_row_major;
use nalgebra::Point3;
use serde::Deserialize;
use serde_json::Value;

/// Same tolerance as `octomap_parity.rs`: log-odds are `f32` on both sides.
const LOG_ODDS_EPS: f64 = 1e-5;
const OCCUPANCY_EPS: f64 = 1e-6;
/// Pose composition is plain `f64` matrix arithmetic on both sides.
const POSE_EPS: f64 = 1e-12;

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ActionSpec {
    UpdatePoint { point: [f64; 3], occupied: bool },
}

#[derive(Deserialize)]
struct QuerySpec {
    point: [f64; 3],
}

#[derive(Deserialize)]
struct RequestFixture {
    id: u64,
    resolution: f64,
    actions: Vec<ActionSpec>,
    object_pose: [f64; 16],
    #[serde(default)]
    shape_pose: Option<[f64; 16]>,
    queries: Vec<QuerySpec>,
}

#[derive(Deserialize)]
struct OracleResult {
    global_pose: [f64; 16],
    queries: Vec<Value>,
}

#[derive(Deserialize)]
struct OracleResponse {
    id: u64,
    ok: bool,
    result: OracleResult,
}

fn read_fixture(name: &str) -> String {
    let path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/geometry/{}"),
        name
    );
    fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

fn load_requests() -> Vec<RequestFixture> {
    let raw = read_fixture("octree_in_world_request.json");
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse octree_in_world_request.json: {e}"))
}

fn load_responses() -> Vec<OracleResponse> {
    let raw = read_fixture("octree_in_world_response.json");
    serde_json::from_str(&raw)
        .unwrap_or_else(|e| panic!("parse octree_in_world_response.json: {e}"))
}

fn row_major4x4(t: &Isometry3) -> [f64; 16] {
    let r = t.rotation.to_rotation_matrix();
    let v = t.translation.vector;
    [
        r[(0, 0)],
        r[(0, 1)],
        r[(0, 2)],
        v.x,
        r[(1, 0)],
        r[(1, 1)],
        r[(1, 2)],
        v.y,
        r[(2, 0)],
        r[(2, 1)],
        r[(2, 2)],
        v.z,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

fn expect_bool(v: &Value, key: &str, ctx: &str) -> bool {
    v.get(key)
        .unwrap_or_else(|| panic!("{ctx}: missing {key}"))
        .as_bool()
        .unwrap_or_else(|| panic!("{ctx}: {key} is not a bool"))
}

fn expect_f64(v: &Value, key: &str, ctx: &str) -> f64 {
    v.get(key)
        .unwrap_or_else(|| panic!("{ctx}: missing {key}"))
        .as_f64()
        .unwrap_or_else(|| panic!("{ctx}: {key} is not a number"))
}

#[test]
fn octree_shape_in_a_posed_world_object_matches_the_oracle() {
    let requests = load_requests();
    let responses = load_responses();
    assert_eq!(
        requests.len(),
        responses.len(),
        "request/response fixture count mismatch"
    );

    for (request, response) in requests.iter().zip(&responses) {
        let ctx = format!("id {}", request.id);
        assert_eq!(
            request.id, response.id,
            "{ctx}: request/response id mismatch"
        );
        assert!(response.ok, "{ctx}: oracle reported ok=false");
        assert_eq!(
            request.queries.len(),
            response.result.queries.len(),
            "{ctx}: query/result count mismatch"
        );

        let mut tree = OcTree::new(request.resolution);
        for action in &request.actions {
            match action {
                ActionSpec::UpdatePoint { point, occupied } => {
                    tree.update_node(Point3::from(*point), *occupied, false);
                }
            }
        }

        // Mirrors what a collision backend needs from `Shape::OcTree`: the
        // shape's own payload is exactly the octree, nothing more.
        let shape = Shape::OcTree(ShapeOcTree::from_tree(Arc::new(tree)));
        let Shape::OcTree(shape) = &shape else {
            unreachable!()
        };
        let inner = shape.octree.as_ref().expect("octree payload is Some");

        let object_pose = isometry_from_row_major(&request.object_pose);
        let shape_pose = request
            .shape_pose
            .map_or_else(Isometry3::identity, |m| isometry_from_row_major(&m));
        let global_pose = object_pose * shape_pose;

        let actual_global_pose = row_major4x4(&global_pose);
        for (i, (actual, expected)) in actual_global_pose
            .iter()
            .zip(&response.result.global_pose)
            .enumerate()
        {
            assert!(
                (actual - expected).abs() < POSE_EPS,
                "{ctx}: global_pose[{i}] {actual} vs oracle {expected}"
            );
        }

        for (query, result) in request.queries.iter().zip(&response.result.queries) {
            let world_point = Point3::from(query.point);
            let local_point = global_pose.inverse_transform_point(&world_point);

            let mapped = expect_bool(result, "mapped", &ctx);
            let actual_log_odds = inner.log_odds_at(local_point);
            assert_eq!(
                mapped,
                actual_log_odds.is_some(),
                "{ctx}: mapped mismatch for query point {:?}",
                query.point
            );
            if mapped {
                let expected_log_odds = expect_f64(result, "log_odds", &ctx);
                let expected_occupancy = expect_f64(result, "occupancy", &ctx);
                let actual_log_odds = actual_log_odds.unwrap();
                assert!(
                    (f64::from(actual_log_odds) - expected_log_odds).abs() < LOG_ODDS_EPS,
                    "{ctx}: log_odds {actual_log_odds} vs oracle {expected_log_odds}"
                );
                let actual_occupancy = inner.occupancy_at(local_point).unwrap();
                assert!(
                    (actual_occupancy - expected_occupancy).abs() < OCCUPANCY_EPS,
                    "{ctx}: occupancy {actual_occupancy} vs oracle {expected_occupancy}"
                );
            }
        }
    }
}
