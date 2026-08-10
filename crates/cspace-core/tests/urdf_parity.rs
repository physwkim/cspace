// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle.
//!
//! Ground truth is the oracle's own `model_info` response, captured verbatim
//! into `tests/fixtures/{panda,fanuc}_model_info.json` by querying
//! `tools/moveit-oracle` for `panda_description`/`panda_moveit_config` and
//! `fanuc_description`/`fanuc_moveit_config` (see `PORTING-PLAN.md` for the
//! pinned upstream SHA and the fixture checkout). Comparing against a
//! deserialized fixture, rather than hand-transcribed Rust literals, means a
//! transcription typo can't make this test assert the wrong thing and a
//! future oracle change shows up as a fixture diff instead of silent drift.
//! The oracle-response JSON lives next to this test in `tests/fixtures/`; the
//! URDFs themselves live at the repo-root `fixtures/`, the one home for
//! committed robot descriptions.
//!
//! `virtual_joint` (panda, Floating) and `FixedBase` (fanuc, Fixed) come from
//! each robot's SRDF `<virtual_joint>` element, not from the URDF; SRDF
//! parsing is out of scope for this crate (see role instructions), so they
//! are constructed by hand here rather than read from the URDF fixture. The
//! oracle's `model_info` response does include them, so the fixture still
//! covers their expected shape.

use std::collections::HashMap;
use std::fs;

use serde::Deserialize;

use cspace_core::model::joint::{JointModel, JointType, joint_model_from_urdf};

/// One joint's shape as reported by the oracle's `model_info` op.
#[derive(Deserialize)]
struct OracleJointDetail {
    name: String,
    type_name: String,
    variable_names: Vec<String>,
    /// `(min, max)` per variable; `(None, None)` where the oracle reported
    /// `[null, null]` (an infinite bound — still `position_bounded`, see
    /// [`cspace_core::model::joint::FloatingJoint`]'s doc comment).
    bounds: Vec<(Option<f64>, Option<f64>)>,
    position_bounded: Vec<bool>,
    mimic: Option<OracleMimic>,
}

#[derive(Deserialize)]
struct OracleMimic {
    joint: String,
    multiplier: f64,
    offset: f64,
}

#[derive(Deserialize)]
struct OracleResult {
    joint_details: Vec<OracleJointDetail>,
}

#[derive(Deserialize)]
struct OracleResponse {
    result: OracleResult,
}

fn load_fixture(file_name: &str) -> Vec<OracleJointDetail> {
    let path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/model/{}"),
        file_name
    );
    let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let response: OracleResponse =
        serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"));
    response.result.joint_details
}

fn assert_matches_oracle(model: &JointModel, expected: &OracleJointDetail) {
    assert_eq!(model.name(), expected.name);
    assert_eq!(
        model.type_name(),
        expected.type_name,
        "type_name of '{}'",
        expected.name
    );
    assert_eq!(
        model.variable_names(),
        expected.variable_names.as_slice(),
        "variable_names of '{}'",
        expected.name
    );
    assert_eq!(
        model.variable_bounds().len(),
        expected.bounds.len(),
        "variable count of '{}'",
        expected.name
    );

    for (i, bounds) in model.variable_bounds().iter().enumerate() {
        assert_eq!(
            bounds.position_bounded, expected.position_bounded[i],
            "position_bounded[{i}] of '{}'",
            expected.name
        );
        if let (Some(min), Some(max)) = expected.bounds[i] {
            assert!(
                (bounds.min_position - min).abs() < 1e-9,
                "min_position[{i}] of '{}': {} != {min}",
                expected.name,
                bounds.min_position
            );
            assert!(
                (bounds.max_position - max).abs() < 1e-9,
                "max_position[{i}] of '{}': {} != {max}",
                expected.name,
                bounds.max_position
            );
        }
    }

    match (model.mimic(), &expected.mimic) {
        (None, None) => {}
        (Some(mimic), Some(expected_mimic)) => {
            assert_eq!(
                mimic.joint_name, expected_mimic.joint,
                "mimic joint of '{}'",
                expected.name
            );
            assert_eq!(
                mimic.factor, expected_mimic.multiplier,
                "mimic factor of '{}'",
                expected.name
            );
            assert_eq!(
                mimic.offset, expected_mimic.offset,
                "mimic offset of '{}'",
                expected.name
            );
        }
        (actual, expected_mimic) => {
            panic!(
                "mimic mismatch for '{}': actual={actual:?}, expected present={}",
                expected.name,
                expected_mimic.is_some()
            )
        }
    }
}

fn joints_by_name(urdf_path: &str) -> HashMap<String, JointModel> {
    let robot = urdf_rs::read_file(urdf_path).expect("fixture URDF must parse");
    robot
        .joints
        .iter()
        .map(|joint| {
            // Every revolute/continuous/prismatic joint in the panda and fanuc
            // fixtures carries an explicit `<limit>` element.
            let model = joint_model_from_urdf(joint, true).expect("fixture joint must convert");
            (model.name().to_string(), model)
        })
        .collect()
}

fn joint_type_count(joints: &HashMap<String, JointModel>, joint_type: JointType) -> usize {
    joints
        .values()
        .filter(|j| j.joint_type() == joint_type)
        .count()
}

fn oracle_type_count(expected: &[OracleJointDetail], type_name: &str) -> usize {
    expected.iter().filter(|j| j.type_name == type_name).count()
}

#[test]
fn panda_joint_layer_matches_the_oracle() {
    let mut joints = joints_by_name(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../fixtures/panda.urdf"
    ));
    joints.insert(
        "virtual_joint".to_string(),
        JointModel::new_floating("virtual_joint"),
    );

    let expected = load_fixture("panda_model_info.json");

    assert_eq!(joints.len(), expected.len(), "total joint count");
    for e in &expected {
        let model = joints
            .get(&e.name)
            .unwrap_or_else(|| panic!("missing joint '{}'", e.name));
        assert_matches_oracle(model, e);
    }

    assert_eq!(
        joint_type_count(&joints, JointType::Revolute),
        oracle_type_count(&expected, "Revolute"),
        "revolute count"
    );
    assert_eq!(
        joint_type_count(&joints, JointType::Fixed),
        oracle_type_count(&expected, "Fixed"),
        "fixed count"
    );
    assert_eq!(
        joint_type_count(&joints, JointType::Prismatic),
        oracle_type_count(&expected, "Prismatic"),
        "prismatic count"
    );
    assert_eq!(
        joint_type_count(&joints, JointType::Floating),
        oracle_type_count(&expected, "Floating"),
        "floating count"
    );
}

#[test]
fn fanuc_joint_layer_matches_the_oracle() {
    let mut joints = joints_by_name(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../fixtures/fanuc.urdf"
    ));
    joints.insert("FixedBase".to_string(), JointModel::new_fixed("FixedBase"));

    let expected = load_fixture("fanuc_model_info.json");

    assert_eq!(joints.len(), expected.len(), "total joint count");
    for e in &expected {
        let model = joints
            .get(&e.name)
            .unwrap_or_else(|| panic!("missing joint '{}'", e.name));
        assert_matches_oracle(model, e);
    }

    assert_eq!(
        joint_type_count(&joints, JointType::Revolute),
        oracle_type_count(&expected, "Revolute"),
        "revolute count"
    );
    assert_eq!(
        joint_type_count(&joints, JointType::Fixed),
        oracle_type_count(&expected, "Fixed"),
        "fixed count"
    );
}
