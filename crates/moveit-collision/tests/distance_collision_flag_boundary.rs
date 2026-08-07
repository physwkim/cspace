// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Regression tests for `DistanceResult::collision` (`crates/moveit-collision/
//! src/common.rs:419`) -- a field no existing test in this workspace ever
//! reads (`rg 'distance_result\.|\.minimum_distance|DistanceResult' --glob
//! 'crates/**/tests/*.rs'` finds only `.distance`/`.link_names` hits). This
//! file is a measurement device, not a fix: `parry.rs` is owned by a
//! concurrent panel unifying `accumulate_collision`'s and
//! `accumulate_distance`'s notion of "touching," and every test below is red
//! today and is designed to go green once that fix lands, without this file
//! changing.
//!
//! # The two populations this backend's off-tie rule confuses
//!
//! `accumulate_collision` (`parry.rs`) answers `collision` from
//! `parry3d_f64::query::contact(..., 0.0)`: at bit-exact `dist == 0.0` it
//! defers to `fcl_tangency_verdict`, but at *any* other `dist` -- positive or
//! negative -- it reports `true` unconditionally. `accumulate_distance`
//! answers its own `collision` field from the structurally separate
//! `data.distance <= 0.0` check. The two rules were measured (this session,
//! `tools/moveit-diff/src/bin/positive_gap_band_separation.rs`) to disagree
//! specifically whenever the measured `dist` lands strictly positive,
//! regardless of *why* it is positive -- which happens for two distinct
//! reasons, 7-9 orders of magnitude apart in scale:
//!
//! - **Population A** -- a truly-touching pair whose `dist` rounds to a tiny
//!   *positive* residual (`|dist| ≲ 2 * f64::EPSILON * scale`). Octree case 4
//!   below is this: the oracle says `true`, `accumulate_collision` agrees
//!   (any nonzero `Some` counts), `accumulate_distance` wrongly says `false`
//!   (`dist = +4.129349354679189e-17` fails its own `<= 0.0` gate).
//! - **Population B** -- a truly-separated pair whose gap is smaller than
//!   parry's GJK solver's own relative-convergence tolerance (`eps_rel =
//!   sqrt(10.0 * f64::EPSILON)`, `~1e-8` scale of the shapes here). prbt's
//!   cylinder-on-floor-box boundary below is this: the oracle says `false`,
//!   `accumulate_distance` agrees (`dist > 0.0`), `accumulate_collision`
//!   wrongly says `true` (any nonzero `Some` counts, sign ignored) -- already
//!   documented and pinned as *currently-passing* (i.e. bug-locked) behaviour
//!   by `exact_tangency_boundary.rs::the_collision_boundary_sits_in_a_positive_gap`.
//!
//! So the two channels disagree with the oracle on *opposite* populations,
//! and -- as `the_two_channels_disagree_at_octree_case_4` and
//! `the_two_channels_also_disagree_at_the_prbt_boundary` below show -- they
//! disagree with *each other* on both, in the same direction (`bool` says
//! `true`, `dist` says `false`) every time the measured `dist` is strictly
//! positive. Neither channel alone is a passing path: fixing one by
//! naively adopting the other's rule (e.g. giving `accumulate_collision` a
//! plain `dist <= 0.0` sign check) was tried already and reverted -- see
//! `exact_tangency_boundary.rs`'s module doc -- because it repairs population
//! B while breaking population A.
//!
//! # Provenance
//!
//! Octree case 4's ground truth is the already-committed
//! `tests/fixtures/octree_world_collision_{request,response}.json` (id 4),
//! captured against `moveit-rs/oracle:6192b2fbe3931089` (see
//! `octree_world_collision_parity.rs`'s module doc). The prbt boundary
//! ground truth is `tests/fixtures/tangency_boundary_bside_{request,
//! response}.json`, captured this session against
//! `moveit-rs/oracle:d8512bbee12499c3` by feeding
//! `exact_tangency_boundary.rs`'s own scene (prbt's default state, a
//! `4x4x0.1` floor box) straight to the oracle's `collision` op at `top_z =
//! -3e-8` -- the same offset, and the same oracle answer
//! (`robot_distance = +2.999999999808711e-8`), that file's module doc already
//! transcribes from an earlier run; this fixture is a fresh, independent
//! capture, not a copy of that transcription.

use std::fs;
use std::sync::Arc;

use serde::Deserialize;
use serde_json::Value;

use moveit_collision::{
    AllowedCollisionMatrix, CollisionEnv, CollisionRequest, DistanceRequest, LinkPaddingScale,
    ParryCollisionEnv, World,
};
use moveit_geometry::{Cuboid, Isometry3, Shape};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_octomap::OcTree;
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;
use nalgebra::Point3;

fn read_fixture(name: &str) -> String {
    let path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        name
    );
    fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

// -- Octree case 4 (population A: a truly-touching pair, dist rounds positive) --

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum OctreeAction {
    UpdatePoint { point: [f64; 3], occupied: bool },
}

#[derive(Deserialize)]
struct OctreeRequest {
    id: u64,
    resolution: f64,
    actions: Vec<OctreeAction>,
}

#[derive(Deserialize)]
struct OctreeOracleResult {
    robot_collision: bool,
}

#[derive(Deserialize)]
struct OctreeOracleResponse {
    id: u64,
    result: OctreeOracleResult,
}

/// Case id 4 specifically: an octree leaf whose face lands exactly on the
/// fixture robot's own `+x` face, zero true gap. Reuses
/// `octree_world_collision_parity.rs`'s exact request/response fixtures and
/// scene construction (same robot fixture, same octree build) rather than
/// re-deriving either.
fn octree_case_4() -> (ParryCollisionEnv, RobotModel) {
    let requests: Vec<OctreeRequest> =
        serde_json::from_str::<Vec<Value>>(&read_fixture("octree_world_collision_request.json"))
            .unwrap_or_else(|e| panic!("parse octree request fixture as JSON: {e}"))
            .into_iter()
            .map(|v| serde_json::from_value(v).unwrap_or_else(|e| panic!("parse case: {e}")))
            .collect();
    let case = requests
        .into_iter()
        .find(|r| r.id == 4)
        .expect("octree_world_collision_request.json must contain id 4");

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
    let model = RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build");

    let mut tree = OcTree::new(case.resolution);
    for action in &case.actions {
        let OctreeAction::UpdatePoint { point, occupied } = action;
        tree.update_node(Point3::from(*point), *occupied, false);
    }
    let mut world = World::new();
    world.add_shape(
        "octree_object",
        Arc::new(Shape::OcTree(moveit_geometry::OcTree::from_tree(Arc::new(
            tree,
        )))),
        moveit_geometry::Isometry3::identity(),
    );
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
    (env, model)
}

fn octree_case_4_oracle_robot_collision() -> bool {
    let responses: Vec<OctreeOracleResponse> =
        serde_json::from_str(&read_fixture("octree_world_collision_response.json"))
            .unwrap_or_else(|e| panic!("parse octree response fixture: {e}"));
    responses
        .into_iter()
        .find(|r| r.id == 4)
        .expect("octree_world_collision_response.json must contain id 4")
        .result
        .robot_collision
}

// -- prbt boundary (population B: a truly-separated pair, gap below eps_rel) --

const FLOOR_THICKNESS: f64 = 0.1;

fn build_prbt() -> RobotModel {
    let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/prbt.urdf");
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/prbt.srdf");
    let urdf_xml = fs::read_to_string(urdf_path).expect("fixture URDF must be readable");
    let urdf = urdf_rs::read_file(urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

fn build_acm() -> AllowedCollisionMatrix {
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/prbt.srdf");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    AllowedCollisionMatrix::from_srdf(&srdf)
}

/// `exact_tangency_boundary.rs::floor_env`, reproduced exactly: a `4x4x0.1`
/// floor box whose top face sits at `top_z`.
fn prbt_boundary_env(top_z: f64) -> ParryCollisionEnv {
    let mut world = World::new();
    world.add_shape(
        "floor",
        Arc::new(Shape::Cuboid(
            Cuboid::new(4.0, 4.0, FLOOR_THICKNESS).expect("positive cuboid dimensions"),
        )),
        Isometry3::translation(0.0, 0.0, top_z - FLOOR_THICKNESS / 2.0),
    );
    ParryCollisionEnv::new(world, LinkPaddingScale::default())
}

#[derive(Deserialize)]
struct PrbtOracleResult {
    robot_collision: bool,
}

#[derive(Deserialize)]
struct PrbtOracleResponse {
    id: u64,
    result: PrbtOracleResult,
}

/// `top_z` is an oracle *input*, so it is carried by the request fixture; a
/// response fixture holds only what the oracle emitted. Joining the two on
/// `id` is what `octree_world_collision`'s reader above already does.
#[derive(Deserialize)]
struct PrbtOracleRequest {
    id: u64,
    top_z: f64,
}

/// Ground truth for `top_z`, read from the freshly-captured fixture rather
/// than the value transcribed in `exact_tangency_boundary.rs`'s module doc --
/// the two happen to agree (both are the same live oracle), but this reads
/// its own independently-captured JSON, not that file's prose.
fn prbt_boundary_oracle_robot_collision(top_z: f64) -> bool {
    let requests: Vec<PrbtOracleRequest> =
        serde_json::from_str(&read_fixture("tangency_boundary_bside_request.json"))
            .unwrap_or_else(|e| panic!("parse tangency boundary request fixture: {e}"));
    let id = requests
        .into_iter()
        .find(|r| r.top_z == top_z)
        .unwrap_or_else(|| {
            panic!("tangency_boundary_bside_request.json must contain top_z {top_z}")
        })
        .id;
    let responses: Vec<PrbtOracleResponse> =
        serde_json::from_str(&read_fixture("tangency_boundary_bside_response.json"))
            .unwrap_or_else(|e| panic!("parse tangency boundary response fixture: {e}"));
    responses
        .into_iter()
        .find(|r| r.id == id)
        .unwrap_or_else(|| {
            panic!("tangency_boundary_bside_response.json must contain id {id} (top_z {top_z})")
        })
        .result
        .robot_collision
}

// -- The tests --

/// Population A. `distance_robot(...).collision` at octree case 4 must match
/// the oracle's `true` (case 4's leaf face is exactly on the robot's own
/// face, zero true gap -- upstream's own FCL narrowphase calls that
/// touching). Currently red: this backend's `dist = +4.129349354679189e-17`
/// (a positive rounding residual, not a real gap) fails
/// `accumulate_distance`'s `dist <= 0.0` gate, so `.collision` reads `false`.
#[test]
fn octree_case_4_dist_channel_matches_the_oracle() {
    let (env, model) = octree_case_4();
    let oracle_robot_collision = octree_case_4_oracle_robot_collision();
    assert!(
        oracle_robot_collision,
        "octree case 4's own oracle fixture must say true, or this test is not exercising the \
         configuration its module doc describes"
    );

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();

    let distance_result = env.distance_robot(
        &DistanceRequest {
            enable_signed_distance: true,
            ..DistanceRequest::default()
        },
        &posed,
        &[],
    );
    assert_eq!(
        distance_result.collision, oracle_robot_collision,
        "distance_robot(...).collision at octree case 4: rust {} vs oracle {oracle_robot_collision} \
         (measured dist {})",
        distance_result.collision, distance_result.minimum_distance.distance
    );
}

/// Population B. `check_robot_collision(...).collision` at prbt's boundary
/// (`top_z = -3e-8`, a true `+3e-8` clear gap) must match the oracle's
/// `false`. Currently red: `accumulate_collision` counts any nonzero `Some`
/// from `query::contact(..., 0.0)` as touching regardless of sign, so a
/// clear-air gap this small still reads `true`. This is the same defect
/// `exact_tangency_boundary.rs::the_collision_boundary_sits_in_a_positive_gap`
/// already measured -- that test pins the *current* (wrong) answer as
/// passing, on purpose, as a "has this regressed further" tripwire; this one
/// asserts the *correct* answer, on purpose, so it goes red until the
/// concurrent panel's fix lands and stays green after.
#[test]
fn prbt_boundary_bool_channel_matches_the_oracle() {
    const TOP_Z: f64 = -3e-8;
    let oracle_robot_collision = prbt_boundary_oracle_robot_collision(TOP_Z);
    assert!(
        !oracle_robot_collision,
        "the prbt boundary fixture at top_z {TOP_Z} must say false, or this test is not \
         exercising the clear-air gap its module doc describes"
    );

    let model = build_prbt();
    let acm = build_acm();
    let env = prbt_boundary_env(TOP_Z);
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();

    let collision_result =
        env.check_robot_collision(&CollisionRequest::default(), &posed, &[], Some(&acm));
    assert_eq!(
        collision_result.collision, oracle_robot_collision,
        "check_robot_collision(...).collision at prbt top_z {TOP_Z}: rust {} vs oracle \
         {oracle_robot_collision}",
        collision_result.collision
    );
}

/// The two populations pull the oracle's own answer in opposite directions
/// (A: `true`, B: `false`) while this backend's two channels each get
/// exactly one of them wrong, in the opposite pattern (A: `bool` right,
/// `dist` wrong; B: `bool` wrong, `dist` right). A test that only exercised
/// one side could pass by coincidence of which channel it happened to check;
/// this one pins that the two ground truths genuinely disagree, so
/// `octree_case_4_dist_channel_matches_the_oracle` and
/// `prbt_boundary_bool_channel_matches_the_oracle` are not both testing the
/// same fact twice.
#[test]
fn octree_and_prbt_boundary_oracle_answers_are_opposite() {
    let octree_oracle = octree_case_4_oracle_robot_collision();
    let prbt_oracle = prbt_boundary_oracle_robot_collision(-3e-8);
    assert_ne!(
        octree_oracle, prbt_oracle,
        "octree case 4 (population A) and the prbt boundary (population B) must have opposite \
         oracle verdicts, or the two fixtures are not sampling the two populations this file's \
         module doc describes"
    );
}

/// `check_robot_collision` and `distance_robot` must agree on the same
/// input, or one of them is not answering "is this pair touching" at all.
/// Currently red at octree case 4: `bool_collision = true` (correct, any
/// nonzero `Some` counts), `dist_collision = false` (wrong, fails `dist <=
/// 0.0` on a positive rounding residual) -- the two functions disagree with
/// each other, not just with the oracle. This is the test whose judgment
/// should flip once the concurrent panel unifies `accumulate_collision`'s and
/// `accumulate_distance`'s notion of "touching" behind one shared rule.
#[test]
fn the_two_channels_disagree_at_octree_case_4() {
    let (env, model) = octree_case_4();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();

    let bool_collision = env
        .check_robot_collision(&CollisionRequest::default(), &posed, &[], None)
        .collision;
    let distance_result = env.distance_robot(
        &DistanceRequest {
            enable_signed_distance: true,
            ..DistanceRequest::default()
        },
        &posed,
        &[],
    );
    assert_eq!(
        bool_collision, distance_result.collision,
        "check_robot_collision(...).collision ({bool_collision}) and \
         distance_robot(...).collision ({}) disagree on the identical octree-case-4 input (dist \
         {}) -- accumulate_collision and accumulate_distance are not using the same rule for \
         \"touching\"",
        distance_result.collision, distance_result.minimum_distance.distance
    );
}

/// The same cross-channel disagreement as above, at the *other* population
/// (prbt's `top_z = -3e-8` boundary) -- confirming the disagreement is a
/// single rule mismatch that fires whenever the measured `dist` lands
/// strictly positive, not something specific to octree case 4's geometry.
/// Currently red: `bool_collision = true` (wrong here), `dist_collision =
/// false` (right here) -- the same `(true, false)` pattern as the octree
/// case, even though which channel is *correct* has flipped.
#[test]
fn the_two_channels_also_disagree_at_the_prbt_boundary() {
    const TOP_Z: f64 = -3e-8;
    let model = build_prbt();
    let acm = build_acm();
    let env = prbt_boundary_env(TOP_Z);
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();

    let bool_collision = env
        .check_robot_collision(&CollisionRequest::default(), &posed, &[], Some(&acm))
        .collision;
    let distance_result = env.distance_robot(
        &DistanceRequest {
            enable_signed_distance: true,
            acm: Some(&acm),
            ..DistanceRequest::default()
        },
        &posed,
        &[],
    );
    assert_eq!(
        bool_collision, distance_result.collision,
        "check_robot_collision(...).collision ({bool_collision}) and \
         distance_robot(...).collision ({}) disagree on the identical prbt-boundary input (dist \
         {}) -- the same rule mismatch as octree case 4, on the opposite population",
        distance_result.collision, distance_result.minimum_distance.distance
    );
}
