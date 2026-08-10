// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity tests for [`HybridCollisionEnv::check_robot_collision_distance_field`]
//! and [`HybridCollisionEnv::check_collision_distance_field`] against the
//! oracle's `group_state_representation` op, exercising the one thing no
//! other file in this crate reaches: a populated environment
//! [`PropagationDistanceField`] built by [`HybridCollisionEnv::build_env_distance_field`]
//! itself (via `self.parry.world()`), not one constructed by hand and passed
//! in as an argument. See `doc/oracle-request-hybrid-collision-env-distance-field.md`'s
//! "How this port will use the response" section: this is what proves
//! `build_env_distance_field`'s own reachability, not just the lower-level
//! [`cspace_collision::distance_field::DistanceFieldCollisionCache`] primitives it
//! composes (already covered by `collision_env_distance_field_parity.rs`,
//! always against an explicit, hand-built `env_distance_field` argument).
//!
//! Two request sets, two problems:
//!
//! - **`group_state_representation_robot_only`** (`mode: "robot_only"`,
//!   `oracle.cpp:4178-4230`): drives `checkRobotCollision`, which -- unlike
//!   `checkCollision` -- calls only `getEnvironmentCollisions`
//!   (`collision_env_distance_field.cpp:1447-1500`, both overloads read in
//!   full: no `getSelfCollisions`/`getIntraGroupCollisions` call exists on
//!   either path). pr2's `right_arm` self-collides at every joint
//!   configuration under its own upstream-shipped SRDF fixture, so at the
//!   default `checkCollision` mode self/intra would report a collision on
//!   every case in this file regardless of whether the environment branch ran
//!   at all -- `mode: "robot_only"` is what isolates the environment branch's
//!   own contribution. Three ids: F1 (a sphere placed clear of every
//!   `right_arm` link -- `collision: false` expected), F2 (a sphere placed to
//!   overlap `right_arm`'s mesh geometry -- `collision: true` expected, with
//!   the specific 7-link set this case's own doc history got wrong on the
//!   first prediction pass, see below), F4 (both spheres together -- the
//!   union of F2's colliding links with F1's non-effect).
//! - **`group_state_representation_environment_branch`** (default mode,
//!   `checkCollision`: self + intra + environment): a paired control at the
//!   identical joint state and ACM, run once with F2's sphere in the world
//!   and once with an empty world. Self/intra contributions are identical
//!   between the two runs by construction (same robot state, same ACM), so
//!   they cancel in a per-link diff; every `gradient.types`/`collision`
//!   difference between the two runs is the environment branch's own output.
//!   `tools/ci/verify-fixture-replay.sh` only replays each fixture
//!   independently against a live oracle -- it has no notion of asserting
//!   two fixtures differ from each other -- so that difference is asserted
//!   directly in this file's own test, not left to the replay gate.
//!
//! **F1/F2/F4 prediction outcome** (`doc/f1-f2-f4-predictions.md`, committed
//! before this file, before the `mode` branch existed to test against): F1's
//! `collision: false` was confirmed exactly. F2/F4's core mechanism --
//! `collision: true`, sentinel `body_name_2`, `shape_kinds_1` matching each
//! link's real URDF mesh geometry -- was confirmed for every link that
//! appears. The *link count* was refuted: the prediction derived a
//! 0.30m-clearance threshold and checked it against two nearby links
//! (`r_wrist_roll_link`, `r_gripper_palm_link`), predicting exactly those
//! two; the oracle reports **seven** -- `r_forearm_link`,
//! `r_gripper_l_finger_link`, `r_gripper_motor_accelerometer_link`,
//! `r_gripper_palm_link`, `r_gripper_r_finger_link`, `r_wrist_flex_link`,
//! `r_wrist_roll_link` -- because several more links' mesh geometry also
//! comes within the sphere's radius at this joint configuration and the
//! prediction never checked them. This is a prediction-completeness error
//! (the derivation method was right, the manual survey of which links to
//! apply it to was not), not a port defect: this file's assertions below
//! compare every link this crate's port reports against the oracle's own
//! per-link dump, not against the original 2-link prediction.

use std::collections::HashMap;
use std::fs;
use std::sync::Arc;

use serde::Deserialize;

use cspace_collision::distance_field::{
    DistanceFieldConfig, GridGeometry, HybridCollisionEnv, add_link_body_decompositions,
};
use cspace_collision::{AllowedCollisionMatrix, CollisionRequest, LinkPaddingScale, World};
use cspace_core::geometry::{Shape, Sphere};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;
use cspace_core::test_support::isometry_from_row_major;
use nalgebra::Vector3;

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/fixtures/distance_field/{}"
        ),
        file_name
    )
}

fn read_fixture(file_name: &str) -> String {
    let path = fixture_path(file_name);
    fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

/// `pr2_description`'s 18 collision meshes, committed under
/// `fixtures/meshes/pr2_description/` -- see
/// `collision_env_distance_field_parity.rs`'s copy of this same helper for
/// the full mapping citation (this crate's per-test-file convention
/// duplicates helpers rather than sharing them via `cspace_core::test_support`,
/// since `env!("CARGO_MANIFEST_DIR")` must resolve per-crate at each file's
/// own compile site).
fn fixture_mesh_search_paths() -> MeshSearchPaths {
    let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
    MeshSearchPaths::new([(
        "moveit_resources_pr2_description",
        format!("{meshes_root}/pr2_description"),
    )])
}

fn build_pr2_model() -> RobotModel {
    let urdf_path = fixture_path("pr2.urdf");
    let srdf_path = fixture_path("pr2.srdf");
    let urdf_xml =
        fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_file(&urdf_path).expect("pr2.urdf must parse");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("pr2.srdf must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &fixture_mesh_search_paths())
        .expect("pr2 model must build")
}

fn build_pr2_srdf() -> SrdfModel {
    SrdfModel::parse_file(fixture_path("pr2.srdf")).expect("pr2.srdf must parse")
}

/// Same defaults `collision_env_distance_field_parity.rs`'s own
/// `oracle_default_distance_field_config` documents in full: the oracle's
/// `CollisionEnvDistanceField env(model_, world)` constructor leaves every
/// argument at its `collision_env_distance_field.hpp:49-55` default.
fn oracle_default_distance_field_config() -> DistanceFieldConfig {
    let size = Vector3::new(3.0, 3.0, 4.0);
    let origin_center = Vector3::new(0.0, 0.0, 0.0);
    let resolution = 0.02;
    DistanceFieldConfig {
        geometry: GridGeometry::new(size, origin_center - 0.5 * size, resolution)
            .expect("grid geometry"),
        max_propagation_distance: 0.25,
        use_signed_distance_field: false,
    }
}

#[derive(Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ObjectShapeSpec {
    Sphere { radius: f64 },
}

impl ObjectShapeSpec {
    fn to_shape(&self) -> Shape {
        match self {
            Self::Sphere { radius } => Shape::Sphere(Sphere::new(*radius).unwrap()),
        }
    }
}

#[derive(Deserialize)]
struct ObjectSpec {
    id: String,
    pose: [f64; 16],
    shape: ObjectShapeSpec,
}

/// Builds a [`World`] with one shape per `objects` entry, matching how
/// `oracle.cpp`'s `group_state_representation` handler populates its own
/// `World` from the same request field.
fn world_from_objects(objects: &[ObjectSpec]) -> World {
    let mut world = World::new();
    for object in objects {
        let shape = Arc::new(object.shape.to_shape());
        world
            .add_shape(&object.id, shape, isometry_from_row_major(&object.pose))
            .expect("add_shape (non-empty shapes, matching poses)");
    }
    world
}

// --- group_state_representation_robot_only: F1/F2/F4 ---

#[derive(Deserialize)]
struct RobotOnlyRequest {
    id: u64,
    group: String,
    #[serde(default)]
    joint_values: HashMap<String, f64>,
    use_acm: bool,
    #[serde(default)]
    objects: Vec<ObjectSpec>,
}

#[derive(Deserialize)]
struct RobotOnlyGradient {
    collision: bool,
    types: Vec<i32>,
}

#[derive(Deserialize)]
struct RobotOnlyLink {
    has_link_decomposition: bool,
    gradient: Option<RobotOnlyGradient>,
    link_name: String,
}

#[derive(Deserialize)]
struct RobotOnlyResult {
    collision: bool,
    links: Vec<RobotOnlyLink>,
}

#[derive(Deserialize)]
struct RobotOnlyResponseEntry {
    result: RobotOnlyResult,
}

/// F2/F4's confirmed 7-link colliding set -- see this module's doc comment
/// for the prediction-vs-oracle history. Cross-checked directly against
/// [`check_collision_distance_field_environment_branch_paired_control`]
/// below: that test's paired diff (self/intra held fixed, only the
/// environment branch's own contribution varies) names the exact same seven
/// links independently, through a different oracle op and a different
/// request (`use_acm: true`, no `mode` field, `checkCollision` rather than
/// `checkRobotCollision`).
const F2_COLLIDING_LINKS: [&str; 7] = [
    "r_forearm_link",
    "r_gripper_l_finger_link",
    "r_gripper_motor_accelerometer_link",
    "r_gripper_palm_link",
    "r_gripper_r_finger_link",
    "r_wrist_flex_link",
    "r_wrist_roll_link",
];

#[test]
fn check_robot_collision_distance_field_matches_the_oracle_robot_only_mode() {
    let model = build_pr2_model();
    let srdf = build_pr2_srdf();
    let acm = AllowedCollisionMatrix::from_srdf(&srdf);
    assert!(model.diagnostics().is_empty());

    let padding = LinkPaddingScale::new();
    let link_body_decompositions = add_link_body_decompositions(&model, 0.02, &padding, None)
        .expect("add_link_body_decompositions");

    let requests: Vec<RobotOnlyRequest> = serde_json::from_str(&read_fixture(
        "group_state_representation_robot_only_request.json",
    ))
    .expect("parse group_state_representation_robot_only_request.json");
    let responses: Vec<RobotOnlyResponseEntry> = serde_json::from_str(&read_fixture(
        "group_state_representation_robot_only_response.json",
    ))
    .expect("parse group_state_representation_robot_only_response.json");
    assert_eq!(requests.len(), responses.len());
    assert_eq!(
        requests.len(),
        3,
        "this fixture's three ids are F1 (clear), F2 (overlap), F4 (both) -- see this module's own doc comment"
    );

    let config = oracle_default_distance_field_config();

    for (request, response) in requests.iter().zip(&responses) {
        let expected = &response.result;

        let world = world_from_objects(&request.objects);
        let mut env = HybridCollisionEnv::new(
            world,
            padding.clone(),
            link_body_decompositions.clone(),
            config,
            0.0,
        )
        .unwrap();

        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        state
            .set_variable_positions_by_name(&request.joint_values)
            .unwrap_or_else(|e| panic!("set joint values (id {}): {e}", request.id));
        let posed = state.update();

        let acm_arg = request.use_acm.then_some(&acm);
        let req = CollisionRequest {
            group_name: Some(request.group.clone()),
            contacts: true,
            max_contacts: 100,
            ..CollisionRequest::default()
        };

        let (res, gsr) = env
            .check_robot_collision_distance_field(&req, &posed, acm_arg, &[])
            .unwrap_or_else(|e| {
                panic!(
                    "check_robot_collision_distance_field (id {}): {e}",
                    request.id
                )
            });

        assert_eq!(
            res.collision, expected.collision,
            "collision (id {})",
            request.id
        );
        assert_eq!(
            gsr.dfce.link_names.len(),
            expected.links.len(),
            "link count (id {})",
            request.id
        );

        let mut actual_colliding_links: Vec<&str> = Vec::new();
        for (i, expected_link) in expected.links.iter().enumerate() {
            assert_eq!(
                gsr.dfce.link_has_geometry[i], expected_link.has_link_decomposition,
                "has_link_decomposition[{i}] (id {})",
                request.id
            );
            if !gsr.dfce.link_has_geometry[i] {
                continue;
            }
            let expected_gradient = expected_link
                .gradient
                .as_ref()
                .expect("has_link_decomposition implies a gradient entry");
            assert_eq!(
                gsr.gradients[i].collision, expected_gradient.collision,
                "gradient.collision[{i}] (id {})",
                request.id
            );
            let actual_types: Vec<i32> = gsr.gradients[i].types.iter().map(|t| *t as i32).collect();
            assert_eq!(
                actual_types, expected_gradient.types,
                "gradient.types[{i}] (id {})",
                request.id
            );
            if gsr.gradients[i].collision {
                actual_colliding_links.push(&expected_link.link_name);
            }
        }

        // F1 (id 1): clear of every link. F2 (id 2) and F4 (id 3): both
        // carry the overlapping sphere, so both must show the same
        // 7-link colliding set -- F4's second, non-overlapping sphere
        // contributes nothing extra (see this module's own doc comment).
        // Compared as a set (sorted), not a sequence: link order here is
        // `RobotState`'s kinematic-chain traversal order, which carries no
        // meaning for "which links collided."
        actual_colliding_links.sort_unstable();
        match request.id {
            1 => assert!(
                actual_colliding_links.is_empty(),
                "id 1 (F1): expected no colliding links, got {actual_colliding_links:?}"
            ),
            2 | 3 => assert_eq!(
                actual_colliding_links, F2_COLLIDING_LINKS,
                "id {} colliding link set",
                request.id
            ),
            other => panic!("unexpected id {other}"),
        }
    }
}

// --- group_state_representation_environment_branch: F3 paired control ---

#[derive(Deserialize)]
struct EnvironmentBranchRequest {
    id: u64,
    group: String,
    #[serde(default)]
    joint_values: HashMap<String, f64>,
    use_acm: bool,
    #[serde(default)]
    objects: Vec<ObjectSpec>,
}

#[derive(Deserialize)]
struct EnvironmentBranchGradient {
    collision: bool,
    types: Vec<i32>,
}

#[derive(Deserialize)]
struct EnvironmentBranchLink {
    has_link_decomposition: bool,
    gradient: Option<EnvironmentBranchGradient>,
    link_name: String,
}

#[derive(Deserialize)]
struct EnvironmentBranchResult {
    links: Vec<EnvironmentBranchLink>,
}

#[derive(Deserialize)]
struct EnvironmentBranchResponseEntry {
    result: EnvironmentBranchResult,
}

/// F3's paired control (this module's own doc comment): id 1 carries F2's
/// overlapping sphere, id 2 is the identical joint state/ACM with an empty
/// world. Both run through [`HybridCollisionEnv::check_collision_distance_field`]
/// (default mode, `checkCollision`: self + intra + environment) rather than
/// [`check_robot_collision_distance_field_matches_the_oracle_robot_only_mode`]'s
/// `checkRobotCollision`, so self/intra collisions are present in both runs
/// -- the point is not that either run is collision-free, but that the two
/// runs' *difference* isolates the environment branch's own contribution.
#[test]
fn check_collision_distance_field_environment_branch_paired_control() {
    let model = build_pr2_model();
    let srdf = build_pr2_srdf();
    let acm = AllowedCollisionMatrix::from_srdf(&srdf);
    assert!(model.diagnostics().is_empty());

    let padding = LinkPaddingScale::new();
    let link_body_decompositions = add_link_body_decompositions(&model, 0.02, &padding, None)
        .expect("add_link_body_decompositions");

    let requests: Vec<EnvironmentBranchRequest> = serde_json::from_str(&read_fixture(
        "group_state_representation_environment_branch_request.json",
    ))
    .expect("parse group_state_representation_environment_branch_request.json");
    let responses: Vec<EnvironmentBranchResponseEntry> = serde_json::from_str(&read_fixture(
        "group_state_representation_environment_branch_response.json",
    ))
    .expect("parse group_state_representation_environment_branch_response.json");
    assert_eq!(requests.len(), responses.len());
    assert_eq!(
        requests.len(),
        2,
        "this fixture's two ids are the paired control: id 1 sphere, id 2 empty -- see this module's own doc comment"
    );

    let config = oracle_default_distance_field_config();

    // (link_name, collision, types) per run, in link order -- collected
    // while checking each run against its own oracle response, then
    // diffed against each other below.
    let mut runs: Vec<Vec<(String, bool, Vec<i32>)>> = Vec::new();

    for (request, response) in requests.iter().zip(&responses) {
        let expected = &response.result;

        let world = world_from_objects(&request.objects);
        let mut env = HybridCollisionEnv::new(
            world,
            padding.clone(),
            link_body_decompositions.clone(),
            config,
            0.0,
        )
        .unwrap();

        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        state
            .set_variable_positions_by_name(&request.joint_values)
            .unwrap_or_else(|e| panic!("set joint values (id {}): {e}", request.id));
        let posed = state.update();

        let acm_arg = request.use_acm.then_some(&acm);
        let req = CollisionRequest {
            group_name: Some(request.group.clone()),
            contacts: true,
            max_contacts: 100,
            ..CollisionRequest::default()
        };

        let (_res, gsr) = env
            .check_collision_distance_field(&req, &posed, acm_arg, &[])
            .unwrap_or_else(|e| panic!("check_collision_distance_field (id {}): {e}", request.id));

        assert_eq!(
            gsr.dfce.link_names.len(),
            expected.links.len(),
            "link count (id {})",
            request.id
        );

        let mut run: Vec<(String, bool, Vec<i32>)> = Vec::new();
        for (i, expected_link) in expected.links.iter().enumerate() {
            assert_eq!(
                gsr.dfce.link_has_geometry[i], expected_link.has_link_decomposition,
                "has_link_decomposition[{i}] (id {})",
                request.id
            );
            if !gsr.dfce.link_has_geometry[i] {
                continue;
            }
            let expected_gradient = expected_link
                .gradient
                .as_ref()
                .expect("has_link_decomposition implies a gradient entry");
            assert_eq!(
                gsr.gradients[i].collision, expected_gradient.collision,
                "gradient.collision[{i}] (id {})",
                request.id
            );
            let actual_types: Vec<i32> = gsr.gradients[i].types.iter().map(|t| *t as i32).collect();
            assert_eq!(
                actual_types, expected_gradient.types,
                "gradient.types[{i}] (id {})",
                request.id
            );
            run.push((
                expected_link.link_name.clone(),
                gsr.gradients[i].collision,
                actual_types,
            ));
        }
        runs.push(run);
    }

    let (sphere_run, empty_run) = (&runs[0], &runs[1]);
    assert_eq!(
        sphere_run.len(),
        empty_run.len(),
        "both runs must report the same link set (same group, same joint state)"
    );

    let mut differing_links: Vec<&str> = sphere_run
        .iter()
        .zip(empty_run)
        .filter(|(s, e)| s != e)
        .map(|(s, _)| s.0.as_str())
        .collect();
    // Compared as a set (sorted), not a sequence -- see the same choice in
    // `check_robot_collision_distance_field_matches_the_oracle_robot_only_mode`.
    differing_links.sort_unstable();

    assert_eq!(
        differing_links, F2_COLLIDING_LINKS,
        "the paired-control diff must name exactly the environment branch's own \
         contribution -- and, as this module's doc comment notes, exactly the \
         same 7-link set group_state_representation_robot_only's F2 case names \
         independently through checkRobotCollision instead of checkCollision"
    );

    // The second, free refuting result this round's brief calls out: if the
    // two runs were identical, the environment branch never fired and this
    // case would need re-posing. `differing_links` above already proves
    // that did not happen, but assert it as its own explicit boundary too.
    assert!(
        !differing_links.is_empty(),
        "sphere and empty runs must differ somewhere, or the environment branch never fired"
    );
}
