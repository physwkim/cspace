// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `DynamicsSolver` parity test against the moveit2 C++ oracle.
//!
//! Ground truth is the oracle's own `dynamics` response, captured verbatim
//! into `tests/fixtures/{panda,fanuc,dual_arm_panda,pr2}_dynamics.json`:
//! five oracle-drawn random-position cases (fixed, deterministic non-zero
//! velocity/acceleration) plus two cases derivable without any oracle at
//! all, reusing the first random case's position —
//! `gravity_compensation` (zero velocity/acceleration) and `zero_gravity`
//! (those, plus zero gravity too). See
//! `tools/moveit-oracle/capture-dynamics-fixtures.py`'s module doc comment
//! for what the captured numbers do and do not mean: panda/fanuc/
//! dual_arm_panda's fixture URDFs have no `<inertial>` on any link (so
//! `torques` is exactly zero in every case for those three), and pr2's
//! `max_payload.payload` is `0.0` in every case because of
//! `DynamicsSolver::getMaxPayload`'s own indexing bug — see
//! `crates/moveit-state/src/dynamics.rs`'s module doc comment for why this
//! port reproduces that bug rather than fixing it.
//!
//! `max_torques` is not read from `RobotModel` (upstream's own
//! `DynamicsSolver` bypasses it too — see `dynamics.rs`), so this test
//! builds it directly from the fixture URDF the same way upstream's
//! constructor does: `<limit effort="...">` per joint in the group's full
//! (fixed-inclusive) joint order, `0.0` for a joint with no `<limit>`.

use std::collections::HashMap;
use std::fs;

use serde::Deserialize;

use moveit_geometry::Vector3;
use moveit_model::RobotModel;
use moveit_srdf::SrdfModel;
use moveit_state::DynamicsSolver;

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        file_name
    )
}

fn build_model(urdf_file: &str, srdf_file: &str) -> (RobotModel, urdf_rs::Robot) {
    let urdf_path = fixture_path(urdf_file);
    let srdf_path = fixture_path(srdf_file);
    let urdf_xml =
        fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    let model =
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf).expect("fixture model must build");
    (model, urdf)
}

/// `DynamicsSolver`'s own constructor precondition: one entry per group
/// joint (fixed included), matching upstream's `max_torques_` build loop
/// over `urdf_model->getJoint(name)->limits->effort` (`0.0` default,
/// matching `urdf_rs::JointLimit::effort`'s own default for a joint with no
/// `<limit>`, e.g. every fixed joint).
fn max_torques_from_urdf(model: &RobotModel, urdf: &urdf_rs::Robot, group: &str) -> Vec<f64> {
    let group = model.joint_model_group(group).expect("group must exist");
    group
        .joint_names()
        .iter()
        .map(|name| {
            urdf.joints
                .iter()
                .find(|j| &j.name == name)
                .map(|j| j.limit.effort)
                .unwrap_or(0.0)
        })
        .collect()
}

#[derive(Deserialize)]
struct MaxPayloadJson {
    joint_saturated: usize,
    payload: Option<f64>,
}

#[derive(Deserialize)]
struct DynamicsCase {
    name: String,
    gravity: [f64; 3],
    joint_values: HashMap<String, f64>,
    joint_velocities: HashMap<String, f64>,
    joint_accelerations: HashMap<String, f64>,
    payload: f64,
    joint_names: Vec<String>,
    torques: Vec<f64>,
    max_torques: Vec<f64>,
    max_payload: MaxPayloadJson,
    payload_torques: Vec<f64>,
}

#[derive(Deserialize)]
struct DynamicsFixture {
    group: String,
    cases: Vec<DynamicsCase>,
}

fn load_fixture(file_name: &str) -> DynamicsFixture {
    let path = fixture_path(file_name);
    let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
}

fn ordered(names: &[String], values: &HashMap<String, f64>) -> Vec<f64> {
    names.iter().map(|n| values[n]).collect()
}

/// Matches `fk_parity.rs`'s own tolerance and rationale: the oracle and
/// this port run different code entirely (Eigen/KDL vs nalgebra), so any
/// agreement tighter than floating-point round-off would be coincidence,
/// not a stronger guarantee. `1e-6` was tried first on the assumption that
/// RNE's extra cross products and inertia contractions (two sweeps over
/// the same chain, vs. FK's one composition pass) would compound rounding
/// further than plain FK -- empirically they don't: every fixture passes
/// at `1e-9` too, so this stays at `1e-9` to keep the same bug-catching
/// power `fk_parity.rs` has, rather than leaving unnecessary slack.
const TOLERANCE: f64 = 1e-9;

fn assert_close(actual: f64, expected: f64, context: &str) {
    assert!(
        (actual - expected).abs() < TOLERANCE,
        "{context}: {actual} != {expected} (oracle)"
    );
}

fn assert_dynamics_matches_oracle(model: &RobotModel, urdf: &urdf_rs::Robot, fixture_name: &str) {
    let fixture = load_fixture(fixture_name);
    let max_torques = max_torques_from_urdf(model, urdf, &fixture.group);

    for case in &fixture.cases {
        let gravity = Vector3::new(case.gravity[0], case.gravity[1], case.gravity[2]);
        let solver = DynamicsSolver::new(model, &fixture.group, gravity, max_torques.clone())
            .unwrap_or_else(|e| panic!("{fixture_name} case {}: {e}", case.name));

        let angles = ordered(&case.joint_names, &case.joint_values);
        let velocities = ordered(&case.joint_names, &case.joint_velocities);
        let accelerations = ordered(&case.joint_names, &case.joint_accelerations);

        assert_eq!(
            max_torques, case.max_torques,
            "{fixture_name} case {}: max_torques",
            case.name
        );

        let torques = solver
            .torques(&angles, &velocities, &accelerations)
            .unwrap_or_else(|e| panic!("{fixture_name} case {}: torques: {e}", case.name));
        for (i, name) in case.joint_names.iter().enumerate() {
            assert_close(
                torques[i],
                case.torques[i],
                &format!("{fixture_name} case {} torques[{name}]", case.name),
            );
        }

        let payload_torques = solver
            .payload_torques(&angles, case.payload)
            .unwrap_or_else(|e| panic!("{fixture_name} case {}: payload_torques: {e}", case.name));
        for (i, name) in case.joint_names.iter().enumerate() {
            assert_close(
                payload_torques[i],
                case.payload_torques[i],
                &format!("{fixture_name} case {} payload_torques[{name}]", case.name),
            );
        }

        let max_payload = solver
            .max_payload(&angles)
            .unwrap_or_else(|e| panic!("{fixture_name} case {}: max_payload: {e}", case.name));
        assert_eq!(
            max_payload.joint_saturated, case.max_payload.joint_saturated,
            "{fixture_name} case {} max_payload.joint_saturated",
            case.name
        );
        match case.max_payload.payload {
            Some(expected) => assert_close(
                max_payload.payload,
                expected,
                &format!("{fixture_name} case {} max_payload.payload", case.name),
            ),
            None => assert!(
                !max_payload.payload.is_finite(),
                "{fixture_name} case {}: oracle's max_payload.payload is null (division by \
                 zero gravity norm), expected a non-finite payload, got {}",
                case.name,
                max_payload.payload
            ),
        }
    }
}

#[test]
fn panda_dynamics_matches_the_oracle() {
    let (model, urdf) = build_model("panda.urdf", "panda.srdf");
    assert_dynamics_matches_oracle(&model, &urdf, "panda_dynamics.json");
}

#[test]
fn fanuc_dynamics_matches_the_oracle() {
    let (model, urdf) = build_model("fanuc.urdf", "fanuc.srdf");
    assert_dynamics_matches_oracle(&model, &urdf, "fanuc_dynamics.json");
}

#[test]
fn dual_arm_panda_dynamics_matches_the_oracle() {
    let (model, urdf) = build_model("dual_arm_panda.urdf", "dual_arm_panda.srdf");
    assert_dynamics_matches_oracle(&model, &urdf, "dual_arm_panda_dynamics.json");
}

#[test]
fn pr2_dynamics_matches_the_oracle() {
    let (model, urdf) = build_model("pr2.urdf", "pr2.srdf");
    assert_dynamics_matches_oracle(&model, &urdf, "pr2_dynamics.json");
}

/// No-oracle-needed physical identity: zero velocity and acceleration
/// reduces RNE to pure gravity compensation, which is exactly what a
/// zero-payload tip wrench also produces -- `gravity_compensation`'s
/// `torques` and its own `payload_torques` (payload `0.0`) must agree,
/// computed independently through this port's `torques` and
/// `payload_torques` entry points (not by comparing against the fixture,
/// which already carries this identity by construction -- this checks the
/// port's own two code paths agree with each other).
fn assert_gravity_compensation_identity(
    model: &RobotModel,
    urdf: &urdf_rs::Robot,
    fixture_name: &str,
) {
    let fixture = load_fixture(fixture_name);
    let max_torques = max_torques_from_urdf(model, urdf, &fixture.group);
    let case = fixture
        .cases
        .iter()
        .find(|c| c.name == "gravity_compensation")
        .expect("fixture must have a gravity_compensation case");
    assert_eq!(
        case.payload, 0.0,
        "{fixture_name}: case payload must be 0.0"
    );

    let gravity = Vector3::new(case.gravity[0], case.gravity[1], case.gravity[2]);
    let solver = DynamicsSolver::new(model, &fixture.group, gravity, max_torques)
        .unwrap_or_else(|e| panic!("{fixture_name}: {e}"));
    let angles = ordered(&case.joint_names, &case.joint_values);
    let velocities = vec![0.0; angles.len()];
    let accelerations = vec![0.0; angles.len()];

    let torques = solver
        .torques(&angles, &velocities, &accelerations)
        .expect("torques must succeed");
    let payload_torques = solver
        .payload_torques(&angles, 0.0)
        .expect("payload_torques must succeed");
    for i in 0..torques.len() {
        assert_close(
            torques[i],
            payload_torques[i],
            &format!("{fixture_name}: torques[{i}] vs payload_torques[{i}] at zero payload"),
        );
    }
}

/// No-oracle-needed physical identity: zero velocity, acceleration and
/// gravity give exactly zero torque on every joint (no motion, no external
/// force, nothing left to produce torque).
fn assert_zero_gravity_gives_zero_torque(
    model: &RobotModel,
    urdf: &urdf_rs::Robot,
    fixture_name: &str,
) {
    let fixture = load_fixture(fixture_name);
    let max_torques = max_torques_from_urdf(model, urdf, &fixture.group);
    let case = fixture
        .cases
        .iter()
        .find(|c| c.name == "zero_gravity")
        .expect("fixture must have a zero_gravity case");
    assert_eq!(
        case.gravity,
        [0.0, 0.0, 0.0],
        "{fixture_name}: case gravity must be zero"
    );

    let solver = DynamicsSolver::new(model, &fixture.group, Vector3::zeros(), max_torques)
        .unwrap_or_else(|e| panic!("{fixture_name}: {e}"));
    let angles = ordered(&case.joint_names, &case.joint_values);
    let velocities = vec![0.0; angles.len()];
    let accelerations = vec![0.0; angles.len()];

    let torques = solver
        .torques(&angles, &velocities, &accelerations)
        .expect("torques must succeed");
    for (i, t) in torques.iter().enumerate() {
        assert_close(
            *t,
            0.0,
            &format!("{fixture_name}: zero_gravity torques[{i}]"),
        );
    }
}

#[test]
fn panda_gravity_compensation_equals_zero_payload_torques() {
    let (model, urdf) = build_model("panda.urdf", "panda.srdf");
    assert_gravity_compensation_identity(&model, &urdf, "panda_dynamics.json");
}

#[test]
fn fanuc_gravity_compensation_equals_zero_payload_torques() {
    let (model, urdf) = build_model("fanuc.urdf", "fanuc.srdf");
    assert_gravity_compensation_identity(&model, &urdf, "fanuc_dynamics.json");
}

#[test]
fn dual_arm_panda_gravity_compensation_equals_zero_payload_torques() {
    let (model, urdf) = build_model("dual_arm_panda.urdf", "dual_arm_panda.srdf");
    assert_gravity_compensation_identity(&model, &urdf, "dual_arm_panda_dynamics.json");
}

#[test]
fn pr2_gravity_compensation_equals_zero_payload_torques() {
    let (model, urdf) = build_model("pr2.urdf", "pr2.srdf");
    assert_gravity_compensation_identity(&model, &urdf, "pr2_dynamics.json");
}

#[test]
fn panda_zero_gravity_gives_zero_torque() {
    let (model, urdf) = build_model("panda.urdf", "panda.srdf");
    assert_zero_gravity_gives_zero_torque(&model, &urdf, "panda_dynamics.json");
}

#[test]
fn fanuc_zero_gravity_gives_zero_torque() {
    let (model, urdf) = build_model("fanuc.urdf", "fanuc.srdf");
    assert_zero_gravity_gives_zero_torque(&model, &urdf, "fanuc_dynamics.json");
}

#[test]
fn dual_arm_panda_zero_gravity_gives_zero_torque() {
    let (model, urdf) = build_model("dual_arm_panda.urdf", "dual_arm_panda.srdf");
    assert_zero_gravity_gives_zero_torque(&model, &urdf, "dual_arm_panda_dynamics.json");
}

#[test]
fn pr2_zero_gravity_gives_zero_torque() {
    let (model, urdf) = build_model("pr2.urdf", "pr2.srdf");
    assert_zero_gravity_gives_zero_torque(&model, &urdf, "pr2_dynamics.json");
}
