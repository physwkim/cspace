// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Boundary tests for `setToRandomPositionsNearBy`, `setToDefaultValues(group,
//! name)`, `getMissingKeys`, the map/named-vector forms of
//! `setVariableVelocities`/`setVariableAccelerations`/`setVariableEffort`,
//! and `setJointVelocities`
//! (`moveit_core/robot_state/src/robot_state.cpp:280-520`).
//!
//! `panda`'s `panda_arm` (7 active revolute joints, no mimic) and `hand` (1
//! active + 1 mimic) groups cover the revolute/prismatic/mimic cases; the
//! per-joint-kind sampling boundaries (continuous wrap, infinite-bounds
//! zero, floating small-vs-large `da`) are unit-tested directly in
//! `moveit-state/src/state.rs` instead, since no fixture here puts a
//! floating or a continuous-revolute joint inside an SRDF group (see that
//! module's own test-module doc comment). `pr2`'s `base` group (exactly
//! `world_joint`, PR2's planar virtual joint) covers the planar case at the
//! public-API level.

use std::collections::HashMap;
use std::fs;

use approx::assert_relative_eq;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        file_name
    )
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

fn panda() -> RobotModel {
    build_model("panda.urdf", "panda.srdf")
}

fn pr2() -> RobotModel {
    build_model("pr2.urdf", "pr2.srdf")
}

fn full_variable_map(model: &RobotModel, value: f64) -> HashMap<String, f64> {
    model
        .variable_names()
        .iter()
        .map(|name| (name.clone(), value))
        .collect()
}

// ---- setToDefaultValues(group, name) -----------------------------------

#[test]
fn set_to_default_values_group_applies_a_known_state() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();

    let applied = state
        .set_to_default_values_group("panda_arm", "ready")
        .unwrap();

    assert!(applied);
    assert_relative_eq!(state.variable_position("panda_joint2").unwrap(), -0.785);
    assert_relative_eq!(state.variable_position("panda_joint7").unwrap(), 0.785);
}

#[test]
fn set_to_default_values_group_returns_false_and_leaves_state_unchanged_for_unknown_name() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    state.set_variable_position("panda_joint1", 1.23).unwrap();

    let applied = state
        .set_to_default_values_group("panda_arm", "no_such_state")
        .unwrap();

    assert!(!applied);
    assert_relative_eq!(state.variable_position("panda_joint1").unwrap(), 1.23);
}

#[test]
fn set_to_default_values_group_errors_on_unknown_group() {
    let model = panda();
    let mut state = RobotState::new(&model);
    assert!(
        state
            .set_to_default_values_group("no_such_group", "ready")
            .is_err()
    );
}

// ---- getMissingKeys -----------------------------------------------------

/// `panda_finger_joint2` (the `hand` group's mimic follower) must never be
/// reported missing, even when it is absent from the map; the active
/// `panda_finger_joint1` must be, when it is absent too.
#[test]
fn missing_keys_excludes_the_mimic_but_lists_the_active_gap() {
    let model = panda();
    let state = RobotState::new(&model);

    let mut values = full_variable_map(&model, 0.0);
    values.remove("panda_finger_joint1");
    values.remove("panda_finger_joint2");

    assert_eq!(
        state.missing_keys(&values),
        vec!["panda_finger_joint1".to_string()]
    );
}

#[test]
fn missing_keys_is_empty_when_every_non_mimic_variable_is_present() {
    let model = panda();
    let state = RobotState::new(&model);
    let values = full_variable_map(&model, 0.0);
    assert!(state.missing_keys(&values).is_empty());
}

// ---- setVariableVelocities/Accelerations/Effort: map and named forms ---

#[test]
fn set_variable_velocities_by_name_and_missing_reports_the_active_gap() {
    let model = panda();
    let mut state = RobotState::new(&model);
    let mut values = full_variable_map(&model, 1.5);
    values.remove("panda_joint3");

    let missing = state
        .set_variable_velocities_by_name_and_missing(&values)
        .unwrap();

    assert_eq!(missing, vec!["panda_joint3".to_string()]);
    assert!(state.has_velocities());
    assert_relative_eq!(state.variable_velocity("panda_joint1").unwrap(), 1.5);
}

#[test]
fn set_variable_velocities_named_errors_on_unknown_name() {
    let model = panda();
    let mut state = RobotState::new(&model);
    assert!(
        state
            .set_variable_velocities_named(&["no_such_variable"], &[1.0])
            .is_err()
    );
}

#[test]
fn set_variable_accelerations_by_name_and_missing_reports_the_active_gap() {
    let model = panda();
    let mut state = RobotState::new(&model);
    let mut values = full_variable_map(&model, 2.5);
    values.remove("panda_joint4");

    let missing = state
        .set_variable_accelerations_by_name_and_missing(&values)
        .unwrap();

    assert_eq!(missing, vec!["panda_joint4".to_string()]);
    assert!(state.has_accelerations());
    assert_relative_eq!(state.variable_acceleration("panda_joint1").unwrap(), 2.5);
}

#[test]
fn set_variable_accelerations_named_errors_on_unknown_name() {
    let model = panda();
    let mut state = RobotState::new(&model);
    assert!(
        state
            .set_variable_accelerations_named(&["no_such_variable"], &[1.0])
            .is_err()
    );
}

#[test]
fn set_variable_efforts_by_name_and_missing_reports_the_active_gap() {
    let model = panda();
    let mut state = RobotState::new(&model);
    let mut values = full_variable_map(&model, 3.5);
    values.remove("panda_joint5");

    let missing = state
        .set_variable_efforts_by_name_and_missing(&values)
        .unwrap();

    assert_eq!(missing, vec!["panda_joint5".to_string()]);
    assert!(state.has_effort());
    assert_relative_eq!(state.variable_effort("panda_joint1").unwrap(), 3.5);
}

#[test]
fn set_variable_efforts_named_errors_on_unknown_name() {
    let model = panda();
    let mut state = RobotState::new(&model);
    assert!(
        state
            .set_variable_efforts_named(&["no_such_variable"], &[1.0])
            .is_err()
    );
}

// ---- setJointVelocities: no mimic derivation, no dirty mark -------------

/// Unlike `set_joint_positions`, upstream's `setJointVelocities` calls
/// neither `updateMimicJoint` nor `markDirtyJointTransforms` — only the
/// leader's velocity must land; the mimic follower's stays untouched.
#[test]
fn set_joint_velocities_does_not_derive_the_mimic_slot() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    assert!(!state.has_velocities());

    state
        .set_joint_velocities("panda_finger_joint1", &[0.5])
        .unwrap();

    assert!(state.has_velocities());
    assert_relative_eq!(state.variable_velocity("panda_finger_joint1").unwrap(), 0.5);
    assert_relative_eq!(
        state.variable_velocity("panda_finger_joint2").unwrap(),
        0.0,
        epsilon = 1e-12
    );
}

#[test]
fn set_joint_velocities_errors_on_unknown_joint() {
    let model = panda();
    let mut state = RobotState::new(&model);
    assert!(state.set_joint_velocities("no_such_joint", &[1.0]).is_err());
}

#[test]
#[should_panic]
fn set_joint_velocities_panics_on_a_short_input() {
    let model = panda();
    let mut state = RobotState::new(&model);
    let _ = state.set_joint_velocities("panda_joint1", &[]);
}

/// `panda_hand_joint` is fixed (0 variables) — the early return must skip
/// even `has_velocity_ = true`, matching upstream's identical early return
/// in `setJointVelocities`.
#[test]
fn set_joint_velocities_is_a_no_op_for_a_fixed_joint() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_joint_velocities("panda_hand_joint", &[]).unwrap();
    assert!(!state.has_velocities());
}

// ---- setToRandomPositionsNearBy -----------------------------------------

#[test]
fn near_by_group_errors_on_unknown_group() {
    let model = panda();
    let mut seed = RobotState::new(&model);
    seed.set_to_default_values();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let mut rng = ChaCha8Rng::seed_from_u64(0);

    assert!(
        state
            .set_to_random_positions_near_by_group("no_such_group", &seed, 0.1, &mut rng)
            .is_err()
    );
}

/// Every sample must stay within `distance` of `seed`'s value (clamped to
/// bounds, so this checks the weaker "at most `distance` away" direction)
/// and within the group's own bounds, over many rounds with a seeded RNG.
#[test]
fn near_by_group_samples_stay_within_distance_and_bounds() {
    let model = panda();
    let mut seed = RobotState::new(&model);
    seed.set_to_default_values();
    let group = model.joint_model_group("panda_arm").unwrap();
    let seed_values: Vec<f64> = group
        .variable_names()
        .iter()
        .map(|name| seed.variable_position(name).unwrap())
        .collect();

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let mut rng = ChaCha8Rng::seed_from_u64(7);
    let distance = 0.05;

    for round in 0..20 {
        state
            .set_to_random_positions_near_by_group("panda_arm", &seed, distance, &mut rng)
            .unwrap();
        assert!(
            state.satisfies_bounds_group("panda_arm", 0.0).unwrap(),
            "round {round}: bounds violated"
        );
        for (name, &near) in group.variable_names().iter().zip(&seed_values) {
            let value = state.variable_position(name).unwrap();
            assert!(
                (value - near).abs() <= distance + 1e-9,
                "round {round}: {name} = {value} not within {distance} of {near}"
            );
        }
    }
}

/// `hand`'s mimic follower must still track its master after a near-by
/// sample, via `updateMimicJoints(group)`.
#[test]
fn near_by_group_propagates_to_the_mimic_follower() {
    let model = panda();
    let mut seed = RobotState::new(&model);
    seed.set_to_default_values();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let mut rng = ChaCha8Rng::seed_from_u64(3);

    for round in 0..10 {
        state
            .set_to_random_positions_near_by_group("hand", &seed, 0.01, &mut rng)
            .unwrap();
        let leader = state.variable_position("panda_finger_joint1").unwrap();
        let follower = state.variable_position("panda_finger_joint2").unwrap();
        assert!(
            (follower - leader).abs() < 1e-12,
            "round {round}: follower {follower} != leader {leader}"
        );
    }
}

/// The per-joint-`distances` overload: a `0.0` entry pins that joint
/// exactly to `seed`'s value.
#[test]
fn near_by_group_with_distances_pins_a_zero_distance_joint_exactly() {
    let model = panda();
    let mut seed = RobotState::new(&model);
    seed.set_to_default_values();
    let group = model.joint_model_group("panda_arm").unwrap();
    let n = group.variable_names().len();
    let mut distances = vec![0.2; n];
    distances[0] = 0.0;

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let mut rng = ChaCha8Rng::seed_from_u64(11);
    state
        .set_to_random_positions_near_by_group_with_distances(
            "panda_arm",
            &seed,
            &distances,
            &mut rng,
        )
        .unwrap();

    let name0 = &group.variable_names()[0];
    assert_eq!(
        state.variable_position(name0).unwrap(),
        seed.variable_position(name0).unwrap()
    );
}

#[test]
#[should_panic]
fn near_by_group_with_distances_panics_on_too_few_distances() {
    let model = panda();
    let mut seed = RobotState::new(&model);
    seed.set_to_default_values();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let mut rng = ChaCha8Rng::seed_from_u64(0);
    // "panda_arm" needs 7 distances.
    let _ = state.set_to_random_positions_near_by_group_with_distances(
        "panda_arm",
        &seed,
        &[0.1],
        &mut rng,
    );
}

/// `pr2`'s `base` group is exactly `world_joint`, PR2's planar virtual
/// joint: `x`/`y` must land at `0.0` (infinite bounds), matching
/// `PlanarJointModel::getVariableRandomPositionsNearBy`'s finiteness check.
#[test]
fn near_by_group_planar_translation_stays_zero_and_bounds_hold() {
    let model = pr2();
    let mut seed = RobotState::new(&model);
    seed.set_to_default_values();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let mut rng = ChaCha8Rng::seed_from_u64(5);

    state
        .set_to_random_positions_near_by_group("base", &seed, 1.0, &mut rng)
        .unwrap();

    assert_eq!(state.variable_position("world_joint/x").unwrap(), 0.0);
    assert_eq!(state.variable_position("world_joint/y").unwrap(), 0.0);
    assert!(state.satisfies_bounds_group("base", 0.0).unwrap());
}
