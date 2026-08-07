// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Boundary tests for the `setJointGroupPositions`/`setJointGroupActivePositions`/
//! `copyJointGroupPositions` family
//! (`moveit_core/robot_state/src/robot_state.cpp:571-638`).
//!
//! `panda`'s `hand` group is the fixture of choice throughout: it has
//! exactly one active joint (`panda_finger_joint1`) and one mimic joint
//! (`panda_finger_joint2`, factor `1.0`/offset `0.0` — see
//! `panda.urdf:216-223`), plus the fixed `panda_hand_joint` its member
//! link pulls in, so `group.joint_indices().len() == 3` while
//! `group.active_joint_indices().len() == 1` and
//! `group.variable_names().len() == 2` — the active-vs-all boundary these
//! functions exist to get right. `panda_arm` (7 active joints, no mimic)
//! covers the plainer multi-variable round trip.

use std::fs;

use approx::assert_relative_eq;

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

// ---- Positions: active-vs-all index split, mimic slot -----------------

/// Every group-scoped accessor in this family goes through
/// `RobotModel::joint_model_group`, which errors on an unknown name —
/// unlike upstream's own by-name convenience overloads, which silently
/// no-op (`if (jmg) ...`). This port's `_group` convention (already
/// established by `RobotState::enforce_bounds_group`) is to report the
/// error instead.
#[test]
fn positions_error_on_unknown_group_rather_than_no_op() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();

    assert!(
        state
            .set_joint_group_positions("no_such_group", &[])
            .is_err()
    );
    assert!(
        state
            .set_joint_group_active_positions("no_such_group", &[])
            .is_err()
    );
    assert!(state.joint_group_positions("no_such_group").is_err());
}

/// `hand`'s active/all split: 1 active joint, 3 total (`panda_hand_joint`
/// is fixed — its parent link `panda_hand` pulls it in per the SRDF
/// group's own `<link>` element, "the parent joint of that link is
/// automatically included" — contributing 0 variables either way; the
/// mimic follower is a member of the group but not of its active set).
#[test]
fn hand_group_active_count_excludes_the_mimic_joint() {
    let model = panda();
    let group = model.joint_model_group("hand").unwrap();
    assert_eq!(
        group.joint_indices().len(),
        3,
        "panda_hand_joint (fixed) + panda_finger_joint1 (active) + panda_finger_joint2 (mimic)"
    );
    assert_eq!(
        group.active_joint_indices().len(),
        1,
        "only panda_finger_joint1 is active"
    );
    assert_eq!(
        group.variable_names().len(),
        2,
        "panda_hand_joint contributes no variables"
    );
}

/// `set_joint_group_positions` (the *all*-variant, 2 entries for `hand`)
/// accepts a caller-supplied value for the mimic slot, but the trailing
/// mimic resync immediately overwrites it with `1.0 * leader + 0.0` —
/// matching upstream's own doc comment, "including values of mimic
/// joints" describes the *shape* of the input, not what ends up stored.
#[test]
fn set_joint_group_positions_overrides_the_supplied_mimic_value() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();

    // Two distinct, deliberately-wrong-for-the-mimic-slot values.
    state
        .set_joint_group_positions("hand", &[0.03, 0.02])
        .unwrap();

    let leader = state.variable_position("panda_finger_joint1").unwrap();
    let follower = state.variable_position("panda_finger_joint2").unwrap();
    assert!(
        leader == 0.03 || leader == 0.02,
        "leader must take one of the two supplied values, got {leader}"
    );
    assert_relative_eq!(follower, leader, epsilon = 1e-12);
}

/// `set_joint_group_active_positions` (1 entry for `hand`) writes only the
/// active joint directly; the mimic follower still tracks it, via the
/// trailing group-wide mimic resync.
#[test]
fn set_joint_group_active_positions_uses_the_active_count_not_all() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();

    state
        .set_joint_group_active_positions("hand", &[0.021])
        .unwrap();

    assert_relative_eq!(
        state.variable_position("panda_finger_joint1").unwrap(),
        0.021
    );
    assert_relative_eq!(
        state.variable_position("panda_finger_joint2").unwrap(),
        0.021
    );
}

/// `joint_group_positions` reads back in `group.variable_names()` order
/// (fixed joints like `panda_hand_joint` contribute no variable and so no
/// entry), the same order `set_joint_group_positions` expects on the way
/// in — round-tripping through both must reproduce the input exactly,
/// mimic slot included (its "supplied" value is what the mimic resync
/// settled on, not the original caller-supplied placeholder).
#[test]
fn joint_group_positions_round_trips_through_group_order() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();

    state
        .set_joint_group_active_positions("hand", &[0.017])
        .unwrap();

    let group = model.joint_model_group("hand").unwrap();
    let copied = state.joint_group_positions("hand").unwrap();
    let expected: Vec<f64> = group
        .variable_names()
        .iter()
        .map(|name| state.variable_position(name).unwrap())
        .collect();
    assert_eq!(copied, expected);
}

/// `panda_arm`: 7 active joints, no mimic — the plain multi-variable case
/// the `hand` fixture is too small to exercise on its own.
#[test]
fn joint_group_positions_round_trips_a_seven_joint_chain() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();

    let group = model.joint_model_group("panda_arm").unwrap();
    let n = group.variable_names().len();
    assert_eq!(n, 7);
    let values: Vec<f64> = (0..n).map(|i| 0.1 * (i as f64 + 1.0)).collect();

    state
        .set_joint_group_positions("panda_arm", &values)
        .unwrap();
    assert_eq!(state.joint_group_positions("panda_arm").unwrap(), values);

    state
        .set_joint_group_active_positions("panda_arm", &values)
        .unwrap();
    assert_eq!(state.joint_group_positions("panda_arm").unwrap(), values);
}

// ---- Size-mismatch boundary --------------------------------------------
//
// Upstream's own primitive here performs no length check at all (not even
// a debug-only assert); this port's closest faithful match is the slice
// index's own panic on a short input (see `RobotState::set_joint_group_positions`'s
// own doc comment). A slice *longer* than needed must not panic.

#[test]
#[should_panic]
fn set_joint_group_positions_panics_on_a_short_input() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    // "hand" needs 2 entries.
    let _ = state.set_joint_group_positions("hand", &[0.01]);
}

#[test]
#[should_panic]
fn set_joint_group_active_positions_panics_on_a_short_input() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    // "hand" needs 1 active entry.
    let _ = state.set_joint_group_active_positions("hand", &[]);
}

#[test]
fn set_joint_group_positions_accepts_a_longer_than_needed_input() {
    let model = panda();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    // "hand" needs 2 entries; the trailing 3rd must be silently ignored,
    // matching upstream (the memcpy only ever reads `getVariableCount()`
    // doubles regardless of how much more the buffer actually holds).
    state
        .set_joint_group_positions("hand", &[0.03, 0.02, 999.0])
        .unwrap();
    assert!(state.variable_position("panda_finger_joint1").unwrap() != 999.0);
}
