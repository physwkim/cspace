// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Forward-kinematics parity test against the moveit2 C++ oracle.
//!
//! Ground truth is the oracle's own `fk` response, captured verbatim into
//! `tests/fixtures/{panda,fanuc,dual_arm_panda,pr2}_fk.json`: one default
//! case (`joint_values: {}`, i.e. `setToDefaultValues()`) plus three cases
//! sampled by the oracle's own `random_states` op (which uses
//! `RobotModel::getVariableRandomPositions`, so every case's joint values
//! are already bounds-respecting and mimic-consistent — see
//! `PORTING-PLAN.md` §7.3). Comparing against deserialized fixtures rather
//! than hand-transcribed Rust literals means a transcription typo can't
//! make this test assert the wrong thing, and a future oracle change shows
//! up as a fixture diff instead of silent drift.
//!
//! PR2 is the only fixture with a planar joint (`world_joint`, 1 of 8
//! groups, 95 links/joints) and is the first parity coverage this port has
//! for [`cspace_core::model::joint::PlanarJoint`]'s `normalize_angle` path.

use std::collections::HashMap;
use std::fs;

use serde::Deserialize;

use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;

#[derive(Deserialize)]
struct FkCase {
    joint_values: HashMap<String, f64>,
    link_transforms: HashMap<String, [f64; 16]>,
}

#[derive(Deserialize)]
struct FkFixture {
    cases: Vec<FkCase>,
}

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/state/{}"),
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
    // Unlike panda/fanuc/pr2, dual_arm_panda's SRDF names `<end_effector>`
    // groups cspace-srdf reports as `UnknownGroup` diagnostics — a real
    // quirk of that fixture's SRDF, not a parse failure, and `<end_effector>`
    // is out of `cspace-model`/`cspace-state`'s scope either way (see
    // `RobotModel`'s doc comment), so this test does not assert on
    // diagnostics being empty the way `robot_model_parity.rs` does for the
    // other three fixtures.
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

/// Row-major 4x4, matching the oracle's `toRowMajor4x4`.
fn to_row_major_4x4(transform: &cspace_core::geometry::Isometry3) -> [f64; 16] {
    let m = transform.to_homogeneous();
    let mut out = [0.0; 16];
    for r in 0..4 {
        for c in 0..4 {
            out[r * 4 + c] = m[(r, c)];
        }
    }
    out
}

/// The oracle's `fk` and this port's own FK disagree by more than floating
/// point round-off (they run different code entirely: Eigen vs nalgebra).
/// `1e-9` matches the tolerance `tests/urdf_parity.rs` already uses for
/// bounds comparison in the sibling `cspace-model` crate.
const TOLERANCE: f64 = 1e-9;

fn assert_fk_matches_oracle(model: &RobotModel, fixture_name: &str) {
    let fixture = load_fk_fixture(fixture_name);
    for (case_index, case) in fixture.cases.iter().enumerate() {
        let mut state = RobotState::new(model);
        state.set_to_default_values();
        for (name, &value) in &case.joint_values {
            state
                .set_variable_position(name, value)
                .unwrap_or_else(|e| panic!("{fixture_name} case {case_index}: {e}"));
        }
        let posed = state.update();

        for (link_name, expected) in &case.link_transforms {
            let actual = posed.global_link_transform(link_name).unwrap_or_else(|e| {
                panic!("{fixture_name} case {case_index}, link '{link_name}': {e}")
            });
            let actual = to_row_major_4x4(&actual);
            for i in 0..16 {
                assert!(
                    (actual[i] - expected[i]).abs() < TOLERANCE,
                    "{fixture_name} case {case_index}, link '{link_name}', element {i}: \
                     {} != {} (oracle)",
                    actual[i],
                    expected[i]
                );
            }
        }
    }
}

#[test]
fn panda_fk_matches_the_oracle() {
    let model = build_model("panda.urdf", "panda.srdf");
    assert_fk_matches_oracle(&model, "panda_fk.json");
}

#[test]
fn fanuc_fk_matches_the_oracle() {
    let model = build_model("fanuc.urdf", "fanuc.srdf");
    assert_fk_matches_oracle(&model, "fanuc_fk.json");
}

#[test]
fn dual_arm_panda_fk_matches_the_oracle() {
    let model = build_model("dual_arm_panda.urdf", "dual_arm_panda.srdf");
    assert_fk_matches_oracle(&model, "dual_arm_panda_fk.json");
}

#[test]
fn pr2_fk_matches_the_oracle() {
    let model = build_model("pr2.urdf", "pr2.srdf");
    assert_fk_matches_oracle(&model, "pr2_fk.json");
}
