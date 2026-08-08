// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test for [`moveit_geometry::compound_from_octree`] (the leaf-`Cuboid`
//! `parry3d_f64::shape::Compound` approximation of an octree, PORTING-PLAN.md
//! §4.8's option 2) against the real moveit2 oracle's new
//! `octree_shape_query` op, which runs an actual `fcl::collide`/`fcl::distance`
//! query between real FCL's own `fcl::OcTreed` and a second shape.
//!
//! This is the verification the §4.8 decision itself requires: a `Compound`
//! approximation that agrees with `fcl::OcTreed` on real queries is the
//! evidence the "option 2 first" call rests on, not this crate's own
//! expectations about what the answer should be.
//!
//! `tests/fixtures/octree_shape_query_{request,response}.json` is the
//! request array fed to the oracle's `octree_shape_query` op and its
//! unedited response, captured against `moveit-rs/oracle:ec3982c6057ad64f`.
//! Four scenarios, each built from an 8x8x8 block of 0.1m cells so pruning
//! collapses it to one coarse leaf, per the task's four required boundaries:
//!
//! - id 1: a query sphere entirely inside one coarse *free* leaf (a pruned
//!   0.8m free block), with the only occupied cell far away -- checks that a
//!   coarse free leaf contributes no `Cuboid` at all, not a false collision.
//! - id 2: a query box straddling the boundary between one coarse occupied
//!   leaf (the pruned 0.8m block) and one adjacent *fine* occupied leaf (a
//!   single un-merged 0.1m cell) -- checks that differently-sized leaf
//!   `Cuboid`s abut correctly across a resolution boundary.
//! - id 3: a query box whose face lands exactly on the coarse leaf's own
//!   face (zero gap) -- the exact-contact boundary a per-scenario test would
//!   never reach.
//! - id 4: a query sphere against a subtree that *was* occupied, then
//!   genuinely cleared (three miss passes, not one -- a single contradicting
//!   observation nets log-odds `0.847 - 0.405 = 0.442`, still above the
//!   occupancy threshold; three nets `0.847 - 3*0.405 = -0.368`, below it) and
//!   re-pruned to free. Confirms the `Compound` reflects current pruned
//!   occupancy, not a stale occupied leaf from before the clearing.

use std::fs;

use moveit_geometry::compound_from_octree;
use moveit_octomap::OcTree;
use moveit_test_support::isometry_from_row_major;
use nalgebra::Point3;
use parry3d_f64::query;
use parry3d_f64::shape::{Ball, Cuboid as ParryCuboid, Shape as ParryShape};
use serde::Deserialize;

/// Generous enough to cover every fixture's actual separation (the largest,
/// id 1, is ~16.6m) while still returning `None` on a genuine algorithm
/// failure rather than masking one.
const CONTACT_PREDICTION: f64 = 1000.0;
const DISTANCE_EPS: f64 = 1e-6;

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ActionSpec {
    UpdatePoint {
        point: [f64; 3],
        occupied: bool,
        #[serde(default)]
        lazy_eval: bool,
    },
    Prune,
    UpdateInnerOccupancy,
}

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ShapeSpec {
    Sphere { radius: f64 },
    Box { size: [f64; 3] },
}

#[derive(Deserialize)]
struct RequestFixture {
    id: u64,
    resolution: f64,
    actions: Vec<ActionSpec>,
    octree_pose: [f64; 16],
    shape: ShapeSpec,
    shape_pose: [f64; 16],
}

#[derive(Deserialize)]
struct OracleResult {
    collision: bool,
    distance: f64,
}

#[derive(Deserialize)]
struct OracleResponse {
    id: u64,
    ok: bool,
    result: OracleResult,
}

fn read_fixture(name: &str) -> String {
    let path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        name
    );
    fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

fn load_requests() -> Vec<RequestFixture> {
    let raw = read_fixture("octree_shape_query_request.json");
    serde_json::from_str(&raw)
        .unwrap_or_else(|e| panic!("parse octree_shape_query_request.json: {e}"))
}

fn load_responses() -> Vec<OracleResponse> {
    let raw = read_fixture("octree_shape_query_response.json");
    serde_json::from_str(&raw)
        .unwrap_or_else(|e| panic!("parse octree_shape_query_response.json: {e}"))
}

#[test]
fn leaf_cuboid_compound_matches_the_oracles_real_fcl_octree_query() {
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

        let mut tree = OcTree::new(request.resolution);
        for action in &request.actions {
            match action {
                ActionSpec::UpdatePoint {
                    point,
                    occupied,
                    lazy_eval,
                } => {
                    tree.update_node(Point3::from(*point), *occupied, *lazy_eval);
                }
                ActionSpec::Prune => tree.prune(),
                ActionSpec::UpdateInnerOccupancy => tree.update_inner_occupancy(),
            }
        }

        let octree_pose = isometry_from_row_major(&request.octree_pose);
        let shape_pose = isometry_from_row_major(&request.shape_pose);

        let Some(compound) = compound_from_octree(&tree) else {
            assert!(
                !response.result.collision,
                "{ctx}: no occupied leaves at all, but oracle reports collision"
            );
            continue;
        };

        let query_shape: Box<dyn ParryShape> =
            match &request.shape {
                ShapeSpec::Sphere { radius } => Box::new(Ball::new(*radius)),
                ShapeSpec::Box { size } => Box::new(ParryCuboid::new(
                    parry3d_f64::math::Vector::new(size[0] / 2.0, size[1] / 2.0, size[2] / 2.0),
                )),
            };

        let contact = query::contact(
            &octree_pose.into(),
            &compound,
            &shape_pose.into(),
            query_shape.as_ref(),
            CONTACT_PREDICTION,
        )
        .unwrap_or_else(|e| panic!("{ctx}: query::contact unsupported: {e:?}"))
        .unwrap_or_else(|| {
            panic!("{ctx}: query::contact found nothing within {CONTACT_PREDICTION}m")
        });

        let actual_collision = contact.dist <= 0.0;
        assert_eq!(
            actual_collision, response.result.collision,
            "{ctx}: collision mismatch (parry dist {}, oracle distance {})",
            contact.dist, response.result.distance
        );
        assert!(
            (contact.dist - response.result.distance).abs() < DISTANCE_EPS,
            "{ctx}: distance mismatch: parry {} vs oracle {}",
            contact.dist,
            response.result.distance
        );
    }
}
