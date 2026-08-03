// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle.
//!
//! Ground truth captured by querying `tools/moveit-oracle` for
//! `panda_description`/`panda_moveit_config` and
//! `fanuc_description`/`fanuc_moveit_config` (see `PORTING-PLAN.md` for the
//! pinned upstream SHA and the fixture checkout). `virtual_joint`
//! (panda, Floating) and `FixedBase` (fanuc, Fixed) come from each robot's
//! SRDF `<virtual_joint>` element, not from the URDF; SRDF parsing is out of
//! scope for this crate (see role instructions), so they are constructed by
//! hand here rather than read from a fixture.
//!
//! fanuc.urdf's own `<limit>` tags spell pi as the literal `3.14`/`6.28`
//! (not `std::f64::consts::PI`); the expected bounds below transcribe those
//! literal strings so this test is exact against what the oracle actually
//! parsed, so `clippy::approx_constant` is silenced rather than "fixed" —
//! replacing them with the true constant would make the test parity-check
//! against a value the fixture does not contain.
#![allow(clippy::approx_constant)]

use std::collections::HashMap;

use moveit_model::joint::{JointModel, JointType, joint_model_from_urdf};

/// One joint's expected shape, transcribed from the oracle's `model_info`
/// response.
struct Expected {
    name: &'static str,
    type_name: &'static str,
    variable_names: &'static [&'static str],
    /// `(min, max)` per variable, or `None` where the oracle reported
    /// `[null, null]` (an infinite bound — still `position_bounded`, see
    /// [`moveit_model::joint::FloatingJoint`]'s doc comment).
    bounds: &'static [Option<(f64, f64)>],
    position_bounded: &'static [bool],
    mimic: Option<(&'static str, f64, f64)>,
}

fn assert_matches_oracle(model: &JointModel, expected: &Expected) {
    assert_eq!(model.name(), expected.name);
    assert_eq!(
        model.type_name(),
        expected.type_name,
        "type_name of '{}'",
        expected.name
    );
    assert_eq!(
        model.variable_names(),
        expected.variable_names,
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
        if let Some((min, max)) = expected.bounds[i] {
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

    match (model.mimic(), expected.mimic) {
        (None, None) => {}
        (Some(mimic), Some((joint_name, factor, offset))) => {
            assert_eq!(
                mimic.joint_name, joint_name,
                "mimic joint of '{}'",
                expected.name
            );
            assert_eq!(mimic.factor, factor, "mimic factor of '{}'", expected.name);
            assert_eq!(mimic.offset, offset, "mimic offset of '{}'", expected.name);
        }
        (actual, expected_mimic) => {
            panic!(
                "mimic mismatch for '{}': actual={actual:?}, expected={expected_mimic:?}",
                expected.name
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
            let model = joint_model_from_urdf(joint).expect("fixture joint must convert");
            (model.name().to_string(), model)
        })
        .collect()
}

#[test]
fn panda_joint_layer_matches_the_oracle() {
    let mut joints = joints_by_name(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/panda.urdf"
    ));
    joints.insert(
        "virtual_joint".to_string(),
        JointModel::new_floating("virtual_joint"),
    );

    let expected = [
        Expected {
            name: "virtual_joint",
            type_name: "Floating",
            variable_names: &[
                "virtual_joint/trans_x",
                "virtual_joint/trans_y",
                "virtual_joint/trans_z",
                "virtual_joint/rot_x",
                "virtual_joint/rot_y",
                "virtual_joint/rot_z",
                "virtual_joint/rot_w",
            ],
            bounds: &[
                None,
                None,
                None,
                Some((-1.0, 1.0)),
                Some((-1.0, 1.0)),
                Some((-1.0, 1.0)),
                Some((-1.0, 1.0)),
            ],
            position_bounded: &[true; 7],
            mimic: None,
        },
        Expected {
            name: "panda_joint1",
            type_name: "Revolute",
            variable_names: &["panda_joint1"],
            bounds: &[Some((-2.8973, 2.8973))],
            position_bounded: &[true],
            mimic: None,
        },
        Expected {
            name: "panda_joint2",
            type_name: "Revolute",
            variable_names: &["panda_joint2"],
            bounds: &[Some((-1.7628, 1.7628))],
            position_bounded: &[true],
            mimic: None,
        },
        Expected {
            name: "panda_joint3",
            type_name: "Revolute",
            variable_names: &["panda_joint3"],
            bounds: &[Some((-2.8973, 2.8973))],
            position_bounded: &[true],
            mimic: None,
        },
        Expected {
            name: "panda_joint4",
            type_name: "Revolute",
            variable_names: &["panda_joint4"],
            bounds: &[Some((-3.0718, 0.0175))],
            position_bounded: &[true],
            mimic: None,
        },
        Expected {
            name: "panda_joint5",
            type_name: "Revolute",
            variable_names: &["panda_joint5"],
            bounds: &[Some((-2.8973, 2.8973))],
            position_bounded: &[true],
            mimic: None,
        },
        Expected {
            name: "panda_joint6",
            type_name: "Revolute",
            variable_names: &["panda_joint6"],
            bounds: &[Some((-0.0175, 3.7525))],
            position_bounded: &[true],
            mimic: None,
        },
        Expected {
            name: "panda_joint7",
            type_name: "Revolute",
            variable_names: &["panda_joint7"],
            bounds: &[Some((-2.8973, 2.8973))],
            position_bounded: &[true],
            mimic: None,
        },
        Expected {
            name: "panda_joint8",
            type_name: "Fixed",
            variable_names: &[],
            bounds: &[],
            position_bounded: &[],
            mimic: None,
        },
        Expected {
            name: "panda_hand_joint",
            type_name: "Fixed",
            variable_names: &[],
            bounds: &[],
            position_bounded: &[],
            mimic: None,
        },
        Expected {
            name: "panda_finger_joint1",
            type_name: "Prismatic",
            variable_names: &["panda_finger_joint1"],
            bounds: &[Some((0.0, 0.04))],
            position_bounded: &[true],
            mimic: None,
        },
        Expected {
            name: "panda_finger_joint2",
            type_name: "Prismatic",
            variable_names: &["panda_finger_joint2"],
            bounds: &[Some((0.0, 0.04))],
            position_bounded: &[true],
            mimic: Some(("panda_finger_joint1", 1.0, 0.0)),
        },
    ];

    assert_eq!(joints.len(), expected.len(), "total joint count");
    for e in &expected {
        let model = joints
            .get(e.name)
            .unwrap_or_else(|| panic!("missing joint '{}'", e.name));
        assert_matches_oracle(model, e);
    }

    let revolute_count = joints
        .values()
        .filter(|j| j.joint_type() == JointType::Revolute)
        .count();
    let fixed_count = joints
        .values()
        .filter(|j| j.joint_type() == JointType::Fixed)
        .count();
    let prismatic_count = joints
        .values()
        .filter(|j| j.joint_type() == JointType::Prismatic)
        .count();
    let floating_count = joints
        .values()
        .filter(|j| j.joint_type() == JointType::Floating)
        .count();
    assert_eq!(revolute_count, 7);
    assert_eq!(fixed_count, 2);
    assert_eq!(prismatic_count, 2);
    assert_eq!(floating_count, 1);
}

#[test]
fn fanuc_joint_layer_matches_the_oracle() {
    let mut joints = joints_by_name(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/fanuc.urdf"
    ));
    joints.insert("FixedBase".to_string(), JointModel::new_fixed("FixedBase"));

    let expected = [
        Expected {
            name: "FixedBase",
            type_name: "Fixed",
            variable_names: &[],
            bounds: &[],
            position_bounded: &[],
            mimic: None,
        },
        Expected {
            name: "base_link-base",
            type_name: "Fixed",
            variable_names: &[],
            bounds: &[],
            position_bounded: &[],
            mimic: None,
        },
        Expected {
            name: "joint_1",
            type_name: "Revolute",
            variable_names: &["joint_1"],
            bounds: &[Some((-3.14, 3.14))],
            position_bounded: &[true],
            mimic: None,
        },
        Expected {
            name: "joint_2",
            type_name: "Revolute",
            variable_names: &["joint_2"],
            bounds: &[Some((-1.57, 2.79))],
            position_bounded: &[true],
            mimic: None,
        },
        Expected {
            name: "joint_3",
            type_name: "Revolute",
            variable_names: &["joint_3"],
            bounds: &[Some((-3.14, 4.61))],
            position_bounded: &[true],
            mimic: None,
        },
        Expected {
            name: "joint_4",
            type_name: "Revolute",
            variable_names: &["joint_4"],
            bounds: &[Some((-3.31, 3.31))],
            position_bounded: &[true],
            mimic: None,
        },
        Expected {
            name: "joint_5",
            type_name: "Revolute",
            variable_names: &["joint_5"],
            bounds: &[Some((-3.31, 3.31))],
            position_bounded: &[true],
            mimic: None,
        },
        Expected {
            name: "joint_6",
            type_name: "Revolute",
            variable_names: &["joint_6"],
            bounds: &[Some((-6.28, 6.28))],
            position_bounded: &[true],
            mimic: None,
        },
        Expected {
            name: "joint_6-tool0",
            type_name: "Fixed",
            variable_names: &[],
            bounds: &[],
            position_bounded: &[],
            mimic: None,
        },
    ];

    assert_eq!(joints.len(), expected.len(), "total joint count");
    for e in &expected {
        let model = joints
            .get(e.name)
            .unwrap_or_else(|| panic!("missing joint '{}'", e.name));
        assert_matches_oracle(model, e);
    }

    let revolute_count = joints
        .values()
        .filter(|j| j.joint_type() == JointType::Revolute)
        .count();
    let fixed_count = joints
        .values()
        .filter(|j| j.joint_type() == JointType::Fixed)
        .count();
    assert_eq!(revolute_count, 6);
    assert_eq!(fixed_count, 3);
}
