// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Mimic-propagation parity against the moveit2 C++ oracle.
//!
//! `tests/fk_parity.rs`'s fixtures already carry oracle ground truth for
//! this: each random case's `joint_values` comes from the oracle's
//! `random_states` op, whose own doc comment
//! (`tools/moveit-oracle/src/oracle.cpp`) says it "derives mimic values" via
//! real `RobotModel::getVariableRandomPositions` — so a follower variable's
//! recorded value there is already the value real moveit2 propagated from
//! its master, not something this port ever had to touch to produce the
//! fixture. This file reprocesses that same committed data: it hands the
//! port only the *master* value from a case (via
//! [`RobotState::set_variable_position`], which propagates mimic
//! internally) and lets the port derive the follower, then diffs the
//! derived value against the oracle follower value already sitting in the
//! same fixture case. No new oracle call is needed — see `PORTING-PLAN.md`
//! §237 for why this was previously reported as unmeasured.
//!
//! `fanuc` has no mimic joints (see `fanuc.urdf`) and is not covered here.

use std::collections::HashMap;
use std::fs;

use serde::Deserialize;

use cspace_model::{MeshSearchPaths, RobotModel};
use cspace_srdf::SrdfModel;
use cspace_state::RobotState;

#[derive(Deserialize)]
struct FkCase {
    joint_values: HashMap<String, f64>,
}

#[derive(Deserialize)]
struct FkFixture {
    cases: Vec<FkCase>,
}

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        file_name
    )
}

fn load_fk_fixture(file_name: &str) -> FkFixture {
    let path = fixture_path(file_name);
    let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
}

fn build_model(urdf_file: &str, srdf_file: &str) -> RobotModel {
    let urdf_path = fixture_path(urdf_file);
    let srdf_path = fixture_path(srdf_file);
    let urdf_xml =
        fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

/// For each `(master, follower)` pair and each of the fixture's random
/// cases (index 0 is the all-default case, skipped: every mimic pair is
/// trivially consistent at the model's default, propagated or not), set
/// only `master` from that case's oracle-recorded value and assert the
/// port derives exactly the oracle-recorded `follower` value.
fn assert_mimic_matches_oracle(
    model: &RobotModel,
    fixture_name: &str,
    pairs: &[(&str, &str)],
) -> usize {
    let fixture = load_fk_fixture(fixture_name);
    let mut checked = 0;
    for (case_index, case) in fixture.cases.iter().enumerate().skip(1) {
        for &(master, follower) in pairs {
            let master_value = *case.joint_values.get(master).unwrap_or_else(|| {
                panic!("{fixture_name} case {case_index}: fixture missing master '{master}'")
            });
            let expected_follower = *case.joint_values.get(follower).unwrap_or_else(|| {
                panic!("{fixture_name} case {case_index}: fixture missing follower '{follower}'")
            });

            let mut state = RobotState::new(model);
            state.set_to_default_values();
            state.set_variable_position(master, master_value).unwrap();

            let actual_follower = state.variable_position(follower).unwrap();
            assert_eq!(
                actual_follower, expected_follower,
                "{fixture_name} case {case_index}: '{follower}' (mimics '{master}') diverged \
                 from the oracle's own derived value"
            );
            checked += 1;
        }
    }
    checked
}

#[test]
fn panda_mimic_propagation_matches_the_oracle() {
    let model = build_model("panda.urdf", "panda.srdf");
    let checked = assert_mimic_matches_oracle(
        &model,
        "panda_fk.json",
        &[("panda_finger_joint1", "panda_finger_joint2")],
    );
    assert_eq!(checked, 3, "expected 1 pair x 3 random cases");
}

#[test]
fn dual_arm_panda_mimic_propagation_matches_the_oracle() {
    let model = build_model("dual_arm_panda.urdf", "dual_arm_panda.srdf");
    let checked = assert_mimic_matches_oracle(
        &model,
        "dual_arm_panda_fk.json",
        &[
            ("left_panda_finger_joint1", "left_panda_finger_joint2"),
            ("right_panda_finger_joint1", "right_panda_finger_joint2"),
        ],
    );
    assert_eq!(checked, 6, "expected 2 pairs x 3 random cases");
}

#[test]
fn pr2_mimic_propagation_matches_the_oracle() {
    let model = build_model("pr2.urdf", "pr2.srdf");
    let checked = assert_mimic_matches_oracle(
        &model,
        "pr2_fk.json",
        &[
            ("l_gripper_l_finger_joint", "l_gripper_l_finger_tip_joint"),
            ("l_gripper_l_finger_joint", "l_gripper_r_finger_joint"),
            ("l_gripper_l_finger_joint", "l_gripper_r_finger_tip_joint"),
            ("r_gripper_l_finger_joint", "r_gripper_l_finger_tip_joint"),
            ("r_gripper_l_finger_joint", "r_gripper_r_finger_joint"),
            ("r_gripper_l_finger_joint", "r_gripper_r_finger_tip_joint"),
        ],
    );
    assert_eq!(checked, 18, "expected 6 pairs x 3 random cases");
}
