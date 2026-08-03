// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `Posed::jacobian` invariant-boundary tests.
//!
//! The live parity run against the C++ oracle (see `PORTING-PLAN.md`'s
//! Jacobian task) only ever exercises the success path on a genuine chain
//! group, so it never reaches the rejection branches or a non-revolute
//! joint type. This file targets exactly what that run cannot: chain vs.
//! non-chain groups (both a single root with a broken adjacency, and
//! multiple roots), an unsupported joint type inside an otherwise valid
//! chain, and — for the success path — an algebraic invariant that holds
//! regardless of which robot or joint type produced the Jacobian, so it
//! does not depend on transcribing expected numbers from anywhere.

use std::fs;

use moveit_geometry::Vector3;
use moveit_model::RobotModel;
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
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf).expect("fixture model must build")
}

fn build_model_from_str(urdf_xml: &str, srdf_xml: &str) -> RobotModel {
    let urdf = urdf_rs::read_from_string(urdf_xml).expect("inline URDF must parse");
    let srdf = SrdfModel::parse_str(srdf_xml).expect("inline SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, urdf_xml, &srdf).expect("inline model must build")
}

/// For any group `jacobian` accepts, the linear (translation) rows'
/// dependence on `reference_point` is fixed by the angular (rotation) rows:
/// moving the reference point by a world-frame offset `d` shifts every
/// column's linear block by `angular_col.cross(d)`, and never changes the
/// angular block at all — the defining property of a *geometric* Jacobian
/// (`v_p = v_o + omega x (p - o)`), independent of which joint types
/// produced the columns. Checking this needs no oracle-derived numbers.
fn assert_reference_point_shift_is_consistent(
    posed: &moveit_state::Posed,
    group: &str,
    tip_link: &str,
) {
    let ref1 = Vector3::new(0.0, 0.0, 0.0);
    let ref2 = Vector3::new(0.05, -0.02, 0.1);
    let j1 = posed.jacobian(group, &ref1).unwrap();
    let j2 = posed.jacobian(group, &ref2).unwrap();
    assert_eq!(j1.shape(), j2.shape());

    let tip_transform = posed.global_link_transform(tip_link).unwrap();
    let world_shift = tip_transform.rotation * (ref2 - ref1);

    for col in 0..j1.ncols() {
        let angular1 = j1.fixed_view::<3, 1>(3, col).clone_owned();
        let angular2 = j2.fixed_view::<3, 1>(3, col).clone_owned();
        assert!(
            (angular1 - angular2).norm() < 1e-12,
            "column {col}: angular block changed with the reference point"
        );

        let linear1 = j1.fixed_view::<3, 1>(0, col).clone_owned();
        let linear2 = j2.fixed_view::<3, 1>(0, col).clone_owned();
        let expected_shift = angular1.cross(&world_shift);
        let actual_shift = linear2 - linear1;
        assert!(
            (actual_shift - expected_shift).norm() < 1e-9,
            "column {col}: linear shift {actual_shift:?} != angular x reference shift {expected_shift:?}"
        );
    }
}

#[test]
fn panda_arm_is_a_revolute_chain_and_its_jacobian_obeys_the_reference_point_identity() {
    let model = build_model("panda.urdf", "panda.srdf");
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    state.set_variable_position("panda_joint2", -0.4).unwrap();
    state.set_variable_position("panda_joint4", -1.9).unwrap();
    state.set_variable_position("panda_joint6", 1.2).unwrap();
    let posed = state.update();

    let j = posed
        .jacobian("panda_arm", &Vector3::new(0.0, 0.0, 0.0))
        .unwrap();
    assert_eq!(j.shape(), (6, 7));

    assert_reference_point_shift_is_consistent(&posed, "panda_arm", "panda_link8");
}

#[test]
fn pr2_base_is_a_single_planar_joint_chain_and_its_jacobian_obeys_the_reference_point_identity() {
    let model = build_model("pr2.urdf", "pr2.srdf");
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    state.set_variable_position("world_joint/x", 1.0).unwrap();
    state
        .set_variable_position("world_joint/theta", 0.7)
        .unwrap();
    let posed = state.update();

    let j = posed
        .jacobian("base", &Vector3::new(0.0, 0.0, 0.0))
        .unwrap();
    assert_eq!(j.shape(), (6, 3));

    assert_reference_point_shift_is_consistent(&posed, "base", "base_footprint");
}

/// `panda`'s "hand" group has a single active joint (`panda_finger_joint1`;
/// `panda_finger_joint2` is a mimic, so it never becomes a root candidate),
/// so it clears the "exactly one root" gate — but that root's mimic
/// sibling is not its ancestor, so the depth-first joint list's adjacency
/// check (`jointPrecedes`) fails and the group is still not a chain.
#[test]
fn panda_hand_group_has_one_root_but_fails_the_adjacency_check() {
    let model = build_model("panda.urdf", "panda.srdf");
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();

    let err = posed
        .jacobian("hand", &Vector3::new(0.0, 0.0, 0.0))
        .unwrap_err();
    assert!(
        err.to_string().contains("not a chain"),
        "unexpected error: {err}"
    );
}

/// `pr2`'s "arms" group is `left_arm` and `right_arm` combined: two
/// independent active-joint subtrees, so it fails the "exactly one root"
/// gate directly.
#[test]
fn pr2_arms_group_has_two_roots() {
    let model = build_model("pr2.urdf", "pr2.srdf");
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();

    let err = posed
        .jacobian("arms", &Vector3::new(0.0, 0.0, 0.0))
        .unwrap_err();
    assert!(
        err.to_string().contains("not a chain"),
        "unexpected error: {err}"
    );
}

/// A single floating joint is trivially its own one-joint chain (the
/// adjacency check has no pair to fail), so `jacobian` reaches the
/// per-joint dispatch and must reject it there instead of silently
/// filling its columns as if it were revolute or prismatic.
#[test]
fn a_lone_floating_joint_is_a_trivial_chain_but_an_unsupported_joint_type() {
    let urdf = r#"<?xml version="1.0"?>
<robot name="floaty">
  <link name="root"/>
</robot>
"#;
    let srdf = r#"<?xml version="1.0"?>
<robot name="floaty">
  <virtual_joint name="virtual_joint" type="floating" parent_frame="world" child_link="root"/>
  <group name="whole">
    <joint name="virtual_joint"/>
  </group>
</robot>
"#;
    let model = build_model_from_str(urdf, srdf);
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();

    let err = posed
        .jacobian("whole", &Vector3::new(0.0, 0.0, 0.0))
        .unwrap_err();
    assert!(
        err.to_string().contains("unsupported type"),
        "unexpected error: {err}"
    );
}

/// An unknown group name is `Error::UnknownName`, not folded into the
/// "not a chain" message — [`moveit_model::RobotModel::joint_model_group`]
/// already distinguishes the two, and `jacobian` must not collapse them.
#[test]
fn an_unknown_group_name_is_unknown_name_not_not_a_chain() {
    let model = build_model("panda.urdf", "panda.srdf");
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();

    let err = posed
        .jacobian("no_such_group", &Vector3::new(0.0, 0.0, 0.0))
        .unwrap_err();
    assert!(matches!(err, moveit_error::Error::UnknownName { .. }));
}
