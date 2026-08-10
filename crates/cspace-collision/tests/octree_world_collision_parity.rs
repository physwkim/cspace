// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test for the *wired* [`cspace_geometry::Shape::OcTree`] path --
//! `cspace_collision::ParryCollisionEnv::check_robot_collision`/
//! `distance_robot` against a [`World`] whose only object is an octree --
//! versus the real moveit2 oracle's `octree_in_world` op, extended this
//! round to optionally run a real `collision_detection::CollisionEnvFCL`
//! check (`request["robot"]`) over the exact same `World` object it already
//! builds for the pose/occupancy fields `octree_in_world_parity.rs` checks.
//!
//! This is deliberately not `octree_shape_query_parity.rs`: that file
//! verifies [`cspace_geometry::compound_from_octree`] in isolation, bypassing
//! `CollisionEnvFCL`/`RobotState`/ACM by design (its own doc comment). This
//! file is the level above it -- a real robot, a real `World`, and this
//! crate's actual `ParryCollisionEnv::check_robot_collision`/`distance_robot`
//! now that `convert_shape` (`parry.rs`) routes `Shape::OcTree` through that
//! same `compound_from_octree` and an `OctreeCache`.
//!
//! `tests/fixtures/octree_world_robot.{urdf,srdf}` is a minimal one-link
//! fixture (not one of the vendored robots -- see its own doc comment for
//! why it lives under `tests/fixtures/`, not the repo-root `fixtures/`
//! directory `tools/ci/verify-fixture-provenance.sh` covers): a single
//! `1x1x1` box link `"p"` on a floating joint from a fixed `"base"`, the same
//! shape `parry.rs`'s own internal test robot (`box_link`/`floating_joint`)
//! already uses, at its default (identity) pose -- so `"p"` occupies
//! `[-0.5, 0.5]^3` in the world frame on both sides of this comparison.
//!
//! `tests/fixtures/octree_world_collision_{request,response}.json` is the
//! request array fed to the oracle's extended `octree_in_world` op and its
//! unedited response, captured against `moveit-rs/oracle:6192b2fbe3931089`.
//! Five cases, each a single 0.1m-resolution octree world object at the
//! identity pose:
//!
//! - id 1: no occupied leaves at all (an octree attached, but empty) --
//!   `robot_collision` must be `false` and `robot_distance` the oracle's own
//!   "no candidate geometry" sentinel (`f64::MAX`), the same value this
//!   backend's [`DistanceResultsData`] default already uses for "no pair
//!   evaluated at all" -- confirming a genuinely empty (not merely absent)
//!   octree contributes nothing to either side, not just this crate's own
//!   guess about what "nothing" should mean.
//! - id 2: one occupied leaf at the origin, deep inside `"p"` -- collision.
//! - id 3: one occupied leaf far away (`x=10`) -- free, with a real distance.
//! - id 4: one occupied leaf whose face lands exactly on `"p"`'s own `+x`
//!   face (zero gap) -- the exact-contact boundary a per-scenario test would
//!   never reach; real FCL's own convention (`robot_collision: true`,
//!   `robot_distance: -0.0`) confirms this backend's "prediction 0.0 counts
//!   touching as collision" convention (`parry.rs`'s module doc, deviation 5)
//!   is upstream's convention too, not a guess.
//! - id 5: two adjacent leaves of different occupancy (`x=1.65` occupied,
//!   `x=1.6` free neighbor closer to the robot) -- if the free neighbor were
//!   wrongly folded into the occupied leaf's box (extending it toward the
//!   robot), `robot_distance` would read `1.0`; both sides agree on `1.1`,
//!   confirming neither merges them.

use std::fs;
use std::sync::Arc;

use serde::Deserialize;
use serde_json::Value;

use cspace_collision::{CollisionEnv, World};
use cspace_collision::{CollisionRequest, DistanceRequest, LinkPaddingScale, ParryCollisionEnv};
use cspace_geometry::{OcTree as ShapeOcTree, Shape};
use cspace_model::{MeshSearchPaths, RobotModel};
use cspace_octomap::OcTree;
use cspace_srdf::SrdfModel;
use cspace_state::RobotState;
use nalgebra::Point3;

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ActionSpec {
    UpdatePoint { point: [f64; 3], occupied: bool },
}

#[derive(Deserialize)]
struct RequestFixture {
    id: u64,
    resolution: f64,
    actions: Vec<ActionSpec>,
}

#[derive(Deserialize)]
struct OracleResult {
    robot_collision: bool,
    robot_distance: f64,
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
    let raw = read_fixture("octree_world_collision_request.json");
    // The captured request also carries "op"/"object_pose"/"queries"/"robot"
    // fields this test does not need to replay (the scene is fixed: identity
    // object pose, an octree-only world, the default robot pose) -- `Value`
    // ignores whatever `RequestFixture` does not name.
    let raw_values: Vec<Value> =
        serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse as JSON: {e}"));
    raw_values
        .into_iter()
        .map(|v| serde_json::from_value(v).unwrap_or_else(|e| panic!("parse request fixture: {e}")))
        .collect()
}

fn load_responses() -> Vec<OracleResponse> {
    let raw = read_fixture("octree_world_collision_response.json");
    serde_json::from_str(&raw)
        .unwrap_or_else(|e| panic!("parse octree_world_collision_response.json: {e}"))
}

fn build_model() -> RobotModel {
    let urdf_path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/octree_world_robot.urdf"
    );
    let srdf_path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/octree_world_robot.srdf"
    );
    let urdf_xml =
        fs::read_to_string(urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

const TOLERANCE: f64 = 1e-4;

#[test]
fn octree_world_object_collision_matches_the_oracle() {
    let requests = load_requests();
    let responses = load_responses();
    assert_eq!(
        requests.len(),
        responses.len(),
        "request/response fixture count mismatch"
    );

    let model = build_model();

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
                ActionSpec::UpdatePoint { point, occupied } => {
                    tree.update_node(Point3::from(*point), *occupied, false);
                }
            }
        }

        let mut world = World::new();
        world.add_shape(
            "octree_object",
            Arc::new(Shape::OcTree(ShapeOcTree::from_tree(Arc::new(tree)))),
            cspace_geometry::Isometry3::identity(),
        );
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let collision_result =
            env.check_robot_collision(&CollisionRequest::default(), &posed, &[], None);
        assert_eq!(
            collision_result.collision, response.result.robot_collision,
            "{ctx}: robot_collision mismatch"
        );

        let distance_request = DistanceRequest {
            enable_signed_distance: true,
            ..DistanceRequest::default()
        };
        let distance_result = env.distance_robot(&distance_request, &posed, &[]);
        let actual = distance_result.minimum_distance.distance;
        let expected = response.result.robot_distance;
        if expected == f64::MAX {
            assert_eq!(
                actual,
                f64::MAX,
                "{ctx}: robot_distance mismatch (no geometry case)"
            );
        } else {
            assert!(
                (actual - expected).abs() < TOLERANCE,
                "{ctx}: robot_distance mismatch: rust {actual} vs oracle {expected}"
            );
        }
    }
}
