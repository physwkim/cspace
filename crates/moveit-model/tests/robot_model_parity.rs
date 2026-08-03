// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle, at the `RobotModel` level.
//!
//! `tests/urdf_parity.rs` already covers `joint_details` (per-joint bounds,
//! type, mimic) against the same fixtures; this test covers everything that
//! only exists once the full URDF+SRDF tree is assembled: `name`,
//! `model_frame`, `root_link`, link/joint ordering, and group composition
//! (including the SRDF chain/link/subgroup expansion described in role
//! instructions — panda's `hand` group names one joint and three links, and
//! the oracle reports three joints for it).
//!
//! `tests/fixtures/{panda,fanuc}.srdf` are byte-identical copies of
//! `crates/moveit-srdf/tests/fixtures/{panda,fanuc}.srdf`, which are in turn
//! byte-identical to `third_party/moveit_resources/*_moveit_config/config/*.srdf`
//! — verified against a live oracle re-query, not assumed.

use std::fs;

use serde::Deserialize;

use moveit_model::RobotModel;
use moveit_srdf::SrdfModel;

#[derive(Deserialize)]
struct OracleModelInfo {
    name: String,
    model_frame: String,
    root_link: String,
    links: Vec<String>,
    joints: Vec<String>,
    groups: std::collections::BTreeMap<String, Vec<String>>,
}

#[derive(Deserialize)]
struct OracleResponse {
    result: OracleModelInfo,
}

fn load_fixture(file_name: &str) -> OracleModelInfo {
    let path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        file_name
    );
    let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let response: OracleResponse =
        serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"));
    response.result
}

fn build_model(urdf_file: &str, srdf_file: &str) -> RobotModel {
    let urdf_path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        urdf_file
    );
    let srdf_path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        srdf_file
    );
    let urdf_xml =
        fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    assert!(
        srdf.diagnostics().is_empty(),
        "fixture SRDF must parse cleanly: {:?}",
        srdf.diagnostics()
    );
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf).expect("fixture model must build")
}

fn assert_matches_oracle(model: &RobotModel, expected: &OracleModelInfo) {
    assert_eq!(model.name(), expected.name, "name");
    assert_eq!(model.model_frame(), expected.model_frame, "model_frame");
    assert_eq!(model.root_link_name(), expected.root_link, "root_link");
    assert_eq!(model.link_names(), expected.links.as_slice(), "link order");
    assert_eq!(
        model.joint_names(),
        expected.joints.as_slice(),
        "joint order"
    );

    assert_eq!(
        model.joint_model_group_names().count(),
        expected.groups.len(),
        "group count"
    );
    for (name, expected_joints) in &expected.groups {
        let group = model
            .joint_model_group(name)
            .unwrap_or_else(|_| panic!("missing group '{name}'"));
        assert_eq!(
            group.joint_names(),
            expected_joints.as_slice(),
            "joint list of group '{name}'"
        );
    }
}

#[test]
fn panda_robot_model_matches_the_oracle() {
    let model = build_model("panda.urdf", "panda.srdf");
    let expected = load_fixture("panda_model_info.json");
    assert_matches_oracle(&model, &expected);

    assert!(model.diagnostics().is_empty(), "{:?}", model.diagnostics());

    // The measured example from role instructions: `hand` names one joint
    // (`panda_finger_joint1`) plus three links, and expands to three joints
    // because `panda_hand`'s and `panda_rightfinger`'s parent joints
    // (`panda_hand_joint`, `panda_finger_joint2`) are pulled in by the link
    // expansion, not named directly.
    let hand = model.joint_model_group("hand").unwrap();
    assert_eq!(
        hand.joint_names(),
        [
            "panda_hand_joint",
            "panda_finger_joint1",
            "panda_finger_joint2"
        ]
    );

    // `panda_arm_hand` names both `panda_arm` and `hand` as subgroups, so it
    // must report both — and only those two, since no other group's joint
    // set is a subset of `panda_arm_hand`'s besides itself.
    let arm_hand = model.joint_model_group("panda_arm_hand").unwrap();
    assert_eq!(arm_hand.subgroup_names(), ["hand", "panda_arm"]);
}

#[test]
fn fanuc_robot_model_matches_the_oracle() {
    let model = build_model("fanuc.urdf", "fanuc.srdf");
    let expected = load_fixture("fanuc_model_info.json");
    assert_matches_oracle(&model, &expected);

    assert!(model.diagnostics().is_empty(), "{:?}", model.diagnostics());

    // fanuc's `manipulator` chain runs `base_link` to `tool0`; the fixed
    // joint `base_link-base` sits on a sibling branch off `base_link` (to
    // link `base`), not on the path to `tool0`, so it must NOT appear here.
    let manipulator = model.joint_model_group("manipulator").unwrap();
    assert!(
        !manipulator
            .joint_names()
            .iter()
            .any(|j| j == "base_link-base")
    );
    assert!(!manipulator.joint_names().iter().any(|j| j == "FixedBase"));
}
