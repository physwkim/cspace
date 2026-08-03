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
//! The robot descriptions themselves — `fixtures/{panda,fanuc}.{urdf,srdf}` —
//! live at the repo-root `fixtures/`, the one home for every committed robot
//! description; only the oracle-response JSON these tests assert against
//! lives locally in `tests/fixtures/`. The SRDFs are byte-identical to
//! `third_party/moveit_resources/*_moveit_config/config/*.srdf` — verified
//! against a live oracle re-query, not assumed.

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
    #[serde(default)]
    joint_details: Vec<OracleJointDetail>,
}

/// Only the field this test needs from the oracle's per-joint `model_info`
/// shape: the type-count cross-check below. `urdf_parity.rs` asserts the
/// full per-joint shape (bounds, mimic, variable names) against the joint
/// layer built directly from a URDF; this file asserts everything that only
/// exists once the full `RobotModel` pipeline has run (limit-presence
/// detection, virtual-joint root construction, mimic-chain resolution), so a
/// type-count cross-check plus the hand-picked planar/continuous assertions
/// below are enough to catch a pipeline-level regression without duplicating
/// `urdf_parity.rs`'s per-joint coverage.
#[derive(Deserialize)]
struct OracleJointDetail {
    type_name: String,
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
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
        urdf_file
    );
    let srdf_path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
        srdf_file
    );
    let urdf_xml =
        fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf).expect("fixture model must build")
}

/// Like [`build_model`], but also asserts the SRDF parsed with no
/// diagnostics — appropriate for panda/fanuc/PR2, whose SRDFs are clean, but
/// not for dual-arm panda, whose two `UnknownGroup` diagnostics are expected
/// (see `dual_arm_panda_robot_model_matches_the_oracle`).
fn build_clean_model(urdf_file: &str, srdf_file: &str) -> RobotModel {
    let srdf_path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
        srdf_file
    );
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    assert!(
        srdf.diagnostics().is_empty(),
        "fixture SRDF must parse cleanly: {:?}",
        srdf.diagnostics()
    );
    build_model(urdf_file, srdf_file)
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
    let model = build_clean_model("panda.urdf", "panda.srdf");
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
    let model = build_clean_model("fanuc.urdf", "fanuc.srdf");
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

/// PR2 is the first fixture with a `<virtual_joint type="planar">` and with
/// continuous revolute joints; panda and fanuc have neither. Its
/// `world_joint` and its 19 continuous joints are the only oracle-backed
/// coverage `PlanarJoint` and the continuous-joint bounds path have.
#[test]
fn pr2_robot_model_matches_the_oracle() {
    let model = build_clean_model("pr2.urdf", "pr2.srdf");
    let expected = load_fixture("pr2_model_info.json");
    assert_matches_oracle(&model, &expected);

    assert!(model.diagnostics().is_empty(), "{:?}", model.diagnostics());

    use std::collections::HashMap;
    let type_counts: HashMap<&str, usize> =
        expected
            .joint_details
            .iter()
            .fold(HashMap::new(), |mut acc, j| {
                *acc.entry(j.type_name.as_str()).or_default() += 1;
                acc
            });
    assert_eq!(type_counts.get("Revolute").copied().unwrap_or(0), 40);
    assert_eq!(type_counts.get("Fixed").copied().unwrap_or(0), 49);
    assert_eq!(type_counts.get("Prismatic").copied().unwrap_or(0), 5);
    assert_eq!(type_counts.get("Planar").copied().unwrap_or(0), 1);

    let mut model_type_counts: HashMap<&str, usize> = HashMap::new();
    let mut mimic_count = 0;
    let mut continuous_count = 0;
    for joint in model.joint_models() {
        *model_type_counts.entry(joint.type_name()).or_default() += 1;
        if joint.mimic().is_some() {
            mimic_count += 1;
        }
        if joint.joint_type() == moveit_model::joint::JointType::Revolute
            && !joint.variable_bounds()[0].position_bounded
        {
            continuous_count += 1;
        }
    }
    assert_eq!(model_type_counts, type_counts);
    assert_eq!(mimic_count, 6, "mimic joint count");
    assert_eq!(continuous_count, 19, "continuous revolute joint count");

    let world_joint = model.joint_model("world_joint").unwrap();
    assert_eq!(world_joint.type_name(), "Planar");
    assert_eq!(
        world_joint.variable_names(),
        ["world_joint/x", "world_joint/y", "world_joint/theta"]
    );
    let bounds = world_joint.variable_bounds();
    assert_eq!(
        bounds
            .iter()
            .map(|b| b.position_bounded)
            .collect::<Vec<_>>(),
        [true, true, false]
    );
    assert!((bounds[2].min_position - (-std::f64::consts::PI)).abs() < 1e-9);
    assert!((bounds[2].max_position - std::f64::consts::PI).abs() < 1e-9);
}

/// Dual-arm panda's SRDF has no `<virtual_joint>` element at all — the
/// oracle itself logs "No root/virtual joint specified in SRDF. Assuming
/// fixed joint", so its root joint is upstream's `ASSUMED_FIXED_ROOT_JOINT`
/// fallback and `model_frame`/`root_link` both come out as the URDF's root
/// link name (`world`), not a name chosen from the SRDF.
#[test]
fn dual_arm_panda_robot_model_matches_the_oracle() {
    let model = build_model("dual_arm_panda.urdf", "dual_arm_panda.srdf");
    let expected = load_fixture("dual_arm_panda_model_info.json");
    assert_matches_oracle(&model, &expected);

    assert!(model.diagnostics().is_empty(), "{:?}", model.diagnostics());

    // The two `UnknownGroup` diagnostics are expected SRDF-level findings
    // (see `moveit_srdf`'s own
    // `dual_arm_panda_drops_end_effectors_with_undefined_groups` test), not a
    // parse failure — `left_hand`/`right_hand` end effectors name component
    // groups this SRDF never defines.
    let srdf_path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
        "dual_arm_panda.srdf"
    );
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    assert_eq!(srdf.diagnostics().len(), 2, "{:?}", srdf.diagnostics());
}
