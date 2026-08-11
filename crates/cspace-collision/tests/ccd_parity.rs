// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `ccd` op.
//!
//! `ccd` is `CollisionEnvBullet::checkRobotCollision(req, res, state1, state2,
//! acm)` -- the two-state swept check. `ParryCollisionEnv::
//! check_robot_collision_continuous` answers it through `bullet_ccd`, this
//! workspace's port of `checkRobotCollisionHelperCCD` over ported bullet, so
//! unlike `collision_parity.rs` the two sides here are not two independent
//! algorithms agreeing to a tolerance -- they are the *same* algorithm, in two
//! languages, at the same `f32` precision.
//!
//! That is what the assertions below claim. Every scalar is compared with
//! [`assert_eq`] on the widened `f64`, not with a tolerance: bullet computes
//! in `float`, both sides perform the same operations in the same order, and
//! IEEE 754 makes each of those operations exactly reproducible. A tolerance
//! here would be a place for a real divergence to hide -- and, worse, one
//! whose width nobody measured. Where a value genuinely cannot be reproduced
//! the answer is to exclude it and say why, not to widen a bar around it.
//!
//! # What is excluded, and why
//!
//! `nearest_points` alone. `addCastSingleResult` never assigns them and
//! `collision_detection::Contact` gives them no initialiser
//! (`collision_common.hpp:105`), so upstream's swept contacts carry whatever
//! the stack held -- two runs of `capture-ccd-fixtures.py` over the same
//! states read values like `2.07e-312` and `-1.60e+268` there and disagree on
//! every one. There is no value to reproduce; the capture script drops the
//! field, `bullet_ccd` writes zeros, and nothing below compares it.
//!
//! Everything else is compared: the boolean, the contact count, every pair
//! key, and within each pair every contact's two body names, two body types,
//! `depth`, `normal`, `pos` and `percent_interpolation`, in order.
//!
//! # Why the states come in pairs
//!
//! The fixtures are `tools/moveit-oracle/capture-ccd-fixtures.py`'s output:
//! consecutive pairs from one `random_states` batch, plus a default-to-first
//! pair, each checked against the same floor box the discrete fixtures use
//! plus a pillar the arms sweep through. A swept check whose two states coincide degenerates to a
//! discrete one -- the cast hull collapses onto the shape's own pose -- so a
//! fixture built from single states would exercise none of `CastHullShape`'s
//! support function and none of `addCastSingleResult`.

use std::collections::BTreeMap;
use std::fs;
use std::sync::Arc;

use serde::Deserialize;

use cspace_collision::{
    AllowedCollisionMatrix, BodyType, CollisionEnv, CollisionRequest, Contact, LinkPaddingScale,
    ParryCollisionEnv, World,
};
use cspace_core::geometry::{Cuboid, Isometry3, Shape, Vector3};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;

/// One captured `ccd` response, flattened alongside the two states that
/// produced it.
#[derive(Deserialize)]
struct CcdCase {
    joint_values: BTreeMap<String, f64>,
    joint_values2: BTreeMap<String, f64>,
    max_contacts: usize,
    max_contacts_per_pair: usize,
    collision: bool,
    contact_count: usize,
    contacts: Vec<OracleContact>,
}

/// One `ccdContactToJson` object. `nearest_points` and `shape_kinds_*` are
/// deliberately not deserialized -- see the module doc for `nearest_points`;
/// the shape kinds are the oracle's own description of the geometry it was
/// given, not something the port computes.
#[derive(Deserialize)]
struct OracleContact {
    body_name_1: String,
    body_type_1: String,
    body_name_2: String,
    body_type_2: String,
    depth: f64,
    normal: [f64; 3],
    pos: [f64; 3],
    percent_interpolation: f64,
}

#[derive(Deserialize)]
struct CcdFixture {
    cases: Vec<CcdCase>,
}

fn load_fixture(file_name: &str) -> CcdFixture {
    let path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        file_name
    );
    let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
}

fn fixture_mesh_search_paths() -> MeshSearchPaths {
    let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
    MeshSearchPaths::new([
        (
            "moveit_resources_panda_description",
            format!("{meshes_root}/panda_description"),
        ),
        (
            "moveit_resources_fanuc_description",
            format!("{meshes_root}/fanuc_description"),
        ),
        (
            "moveit_resources_pr2_description",
            format!("{meshes_root}/pr2_description"),
        ),
    ])
}

fn build_model(urdf_file: &str, srdf_file: &str) -> RobotModel {
    let path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
        urdf_file
    );
    let srdf_path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
        srdf_file
    );
    let urdf_xml = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let urdf = urdf_rs::read_file(&path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &fixture_mesh_search_paths())
        .expect("fixture model must build")
}

/// The oracle's `ccd` op checks against `AllowedCollisionMatrix(*model_->
/// getSRDF())`, so every case was captured with the robot's own
/// `disable_collisions` entries in force. This is a robot-vs-world check and
/// the SRDF's entries are all link-vs-link, so on these fixtures the matrix
/// changes nothing -- but it is an input the oracle had, and leaving it out
/// would make any future world-object entry read as a port defect.
fn build_acm(srdf_file: &str) -> AllowedCollisionMatrix {
    let srdf_path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
        srdf_file
    );
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    AllowedCollisionMatrix::from_srdf(&srdf)
}

/// The two world objects `capture-ccd-fixtures.py` sends the oracle: the same
/// `4x4x0.1` floor box the discrete fixtures use, top face at `z = 0`, and the
/// `0.2x0.2x1.5` pillar standing on it at `(0.5, 0, 0.75)` that the arms
/// actually sweep through.
fn fixture_env() -> ParryCollisionEnv {
    const FLOOR_THICKNESS: f64 = 0.1;
    let mut world = World::new();
    world.add_shape(
        "floor",
        Arc::new(Shape::Cuboid(
            Cuboid::new(4.0, 4.0, FLOOR_THICKNESS)
                .expect("4x4x0.1 are valid, positive cuboid dimensions"),
        )),
        Isometry3::translation(0.0, 0.0, -FLOOR_THICKNESS / 2.0),
    );
    world.add_shape(
        "pillar",
        Arc::new(Shape::Cuboid(
            Cuboid::new(0.2, 0.2, 1.5).expect("0.2x0.2x1.5 are valid, positive cuboid dimensions"),
        )),
        Isometry3::translation(0.5, 0.0, 0.75),
    );
    ParryCollisionEnv::new(world, LinkPaddingScale::default())
}

fn build_state<'m>(model: &'m RobotModel, joint_values: &BTreeMap<String, f64>) -> RobotState<'m> {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    for (name, &value) in joint_values {
        state
            .set_variable_position(name, value)
            .unwrap_or_else(|e| panic!("setting {name}: {e}"));
    }
    state
}

/// `bodyTypeName`'s three strings, the way back.
fn body_type_name(body_type: BodyType) -> &'static str {
    match body_type {
        BodyType::RobotLink => "robot_link",
        BodyType::RobotAttached => "robot_attached",
        BodyType::WorldObject => "world_object",
    }
}

fn assert_contact_matches(case: usize, index: usize, oracle: &OracleContact, port: &Contact) {
    let at = format!("case {case}, contact {index}");
    assert_eq!(port.body_name_1, oracle.body_name_1, "{at}: body_name_1");
    assert_eq!(
        body_type_name(port.body_type_1),
        oracle.body_type_1,
        "{at}: body_type_1"
    );
    assert_eq!(port.body_name_2, oracle.body_name_2, "{at}: body_name_2");
    assert_eq!(
        body_type_name(port.body_type_2),
        oracle.body_type_2,
        "{at}: body_type_2"
    );
    assert_eq!(port.depth, oracle.depth, "{at}: depth");
    assert_eq!(
        port.normal,
        Vector3::new(oracle.normal[0], oracle.normal[1], oracle.normal[2]),
        "{at}: normal"
    );
    assert_eq!(
        port.pos,
        Vector3::new(oracle.pos[0], oracle.pos[1], oracle.pos[2]),
        "{at}: pos"
    );
    assert_eq!(
        port.percent_interpolation, oracle.percent_interpolation,
        "{at}: percent_interpolation"
    );
}

fn assert_ccd_matches_oracle(fixture_name: &str, urdf_file: &str, srdf_file: &str) {
    let fixture = load_fixture(fixture_name);
    assert!(
        !fixture.cases.is_empty(),
        "{fixture_name} holds no cases: a fixture that captured nothing passes every \
         comparison below"
    );
    let model = build_model(urdf_file, srdf_file);
    let acm = build_acm(srdf_file);
    let env = fixture_env();

    for (case_index, case) in fixture.cases.iter().enumerate() {
        // The budget the oracle was asked at, read off the case rather than
        // restated here: replaying a case at a different budget compares two
        // different questions, and nothing else in the response says which was
        // asked.
        let request = CollisionRequest {
            contacts: true,
            max_contacts: case.max_contacts,
            max_contacts_per_pair: case.max_contacts_per_pair,
            ..CollisionRequest::default()
        };
        let mut state1 = build_state(&model, &case.joint_values);
        let mut state2 = build_state(&model, &case.joint_values2);
        let posed1 = state1.update();
        let posed2 = state2.update();

        let result = env
            .check_robot_collision_continuous(&request, &posed1, &posed2, &[], Some(&acm))
            .unwrap_or_else(|e| panic!("{fixture_name} case {case_index}: {e}"));

        assert_eq!(
            result.collision, case.collision,
            "{fixture_name} case {case_index}: collision"
        );

        let contacts = result
            .contacts
            .expect("contacts were requested, so the result carries the map");
        assert_eq!(
            contacts.count(),
            case.contact_count,
            "{fixture_name} case {case_index}: contact_count"
        );

        // The oracle emits its contacts by walking `res.contacts` (a
        // `std::map` keyed by the sorted name pair) and then each pair's
        // vector in order; `ContactData::by_pair` is a `BTreeMap` over the
        // same keys, so flattening it the same way puts the two sequences in
        // the same order without either side sorting the other's output.
        let flattened: Vec<&Contact> = contacts.by_pair.values().flatten().collect();
        assert_eq!(
            flattened.len(),
            case.contacts.len(),
            "{fixture_name} case {case_index}: number of contacts emitted"
        );
        for (index, (oracle, port)) in case.contacts.iter().zip(flattened).enumerate() {
            assert_contact_matches(case_index, index, oracle, port);
        }
    }
}

#[test]
fn panda_ccd_matches_the_oracle() {
    assert_ccd_matches_oracle("panda_ccd.json", "panda.urdf", "panda.srdf");
}

#[test]
fn fanuc_ccd_matches_the_oracle() {
    assert_ccd_matches_oracle("fanuc_ccd.json", "fanuc.urdf", "fanuc.srdf");
}

#[test]
fn pr2_ccd_matches_the_oracle() {
    assert_ccd_matches_oracle("pr2_ccd.json", "pr2.urdf", "pr2.srdf");
}

/// The fixtures are only evidence about the swept path if the two states in
/// each case actually differ, and only evidence about *contacts* if some case
/// found one. Both are properties of the captured file, not of the port, so
/// they are checked once here rather than assumed by each test above.
#[test]
fn the_fixtures_sweep_and_at_least_one_case_collides() {
    for name in ["panda_ccd.json", "fanuc_ccd.json", "pr2_ccd.json"] {
        let fixture = load_fixture(name);
        for (index, case) in fixture.cases.iter().enumerate() {
            assert_ne!(
                case.joint_values, case.joint_values2,
                "{name} case {index}: both states are identical, so this case is a discrete \
                 check wearing a swept check's shape"
            );
        }
        assert!(
            fixture.cases.iter().any(|case| case.collision),
            "{name}: no case collides, so every comparison in this file is between two empty \
             contact maps"
        );
    }
}
