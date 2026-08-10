// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity with the C++ reference on the three `moveit_resources` SRDFs.
//!
//! Every expected value below was read out of `srdf::Model` by a probe linked
//! against the `libsrdfdom.so.2.0.8` in the `moveit-rs/oracle:latest` image —
//! the same library the differential oracle in `tools/moveit-oracle` uses — and
//! transcribed here. Nothing is asserted from what the SRDF text looks like.
//!
//! The repo-root `fixtures/*.srdf` are byte-for-byte copies of, respectively:
//!
//! - `third_party/moveit_resources/panda_moveit_config/config/panda.srdf`
//! - `third_party/moveit_resources/fanuc_moveit_config/config/fanuc.srdf`
//! - `third_party/moveit_resources/dual_arm_panda_moveit_config/config/panda.srdf`
//!
//! They are copied rather than read from `third_party/` because that directory
//! is an external checkout that `.gitignore` excludes, so it is absent from a
//! fresh clone and from CI. `fixtures/` at the repo root is the one home for
//! every committed robot description; per-crate `tests/fixtures/` directories
//! hold only oracle-response JSON, not URDF/SRDF text.

use std::collections::BTreeMap;

use cspace_srdf::{
    Chain, CollisionPair, Diagnostic, EndEffector, Group, SrdfModel, VirtualJoint, VirtualJointType,
};

const PANDA: &str = include_str!("../../../fixtures/panda.srdf");
const FANUC: &str = include_str!("../../../fixtures/fanuc.srdf");
const DUAL_ARM_PANDA: &str = include_str!("../../../fixtures/dual_arm_panda.srdf");

fn names(v: &[String]) -> Vec<&str> {
    v.iter().map(String::as_str).collect()
}

fn pair(link1: &str, link2: &str, reason: &str) -> CollisionPair {
    CollisionPair {
        link1: link1.to_owned(),
        link2: link2.to_owned(),
        reason: reason.to_owned(),
    }
}

fn joint_values(pairs: &[(&str, f64)]) -> BTreeMap<String, Vec<f64>> {
    pairs
        .iter()
        .map(|&(name, value)| (name.to_owned(), vec![value]))
        .collect()
}

// ---------------------------------------------------------------- panda ----

#[test]
fn panda_parses() {
    let model = SrdfModel::parse_str(PANDA).expect("panda.srdf parses");
    assert_eq!(model.name(), Some("panda"));
    assert_eq!(model.diagnostics(), &[]);
}

#[test]
fn panda_groups_match_reference() {
    let model = SrdfModel::parse_str(PANDA).unwrap();
    assert_eq!(
        model.groups(),
        &[
            Group {
                name: "panda_arm".to_owned(),
                chains: vec![Chain {
                    base_link: "panda_link0".to_owned(),
                    tip_link: "panda_link8".to_owned(),
                }],
                ..Group::default()
            },
            Group {
                name: "hand".to_owned(),
                links: vec![
                    "panda_hand".to_owned(),
                    "panda_leftfinger".to_owned(),
                    "panda_rightfinger".to_owned(),
                ],
                joints: vec!["panda_finger_joint1".to_owned()],
                ..Group::default()
            },
            Group {
                name: "panda_arm_hand".to_owned(),
                subgroups: vec!["panda_arm".to_owned(), "hand".to_owned()],
                ..Group::default()
            },
        ]
    );
}

#[test]
fn panda_group_states_match_reference() {
    let model = SrdfModel::parse_str(PANDA).unwrap();
    let states = model.group_states();
    assert_eq!(
        states.iter().map(|s| s.name.as_str()).collect::<Vec<_>>(),
        ["ready", "extended", "transport", "open", "close"]
    );
    assert_eq!(
        states.iter().map(|s| s.group.as_str()).collect::<Vec<_>>(),
        ["panda_arm", "panda_arm", "panda_arm", "hand", "hand"]
    );
    assert_eq!(
        states[0].joint_values,
        joint_values(&[
            ("panda_joint1", 0.0),
            ("panda_joint2", -0.785),
            ("panda_joint3", 0.0),
            ("panda_joint4", -2.356),
            ("panda_joint5", 0.0),
            ("panda_joint6", 1.571),
            ("panda_joint7", 0.785),
        ])
    );
    assert_eq!(
        states[2].joint_values,
        joint_values(&[
            ("panda_joint1", 0.0),
            ("panda_joint2", -0.5599),
            ("panda_joint3", 0.0),
            ("panda_joint4", -2.97),
            ("panda_joint5", 0.0),
            ("panda_joint6", 0.0),
            ("panda_joint7", 0.785),
        ])
    );
    assert_eq!(
        states[3].joint_values,
        joint_values(&[
            ("panda_finger_joint1", 0.035),
            ("panda_finger_joint2", 0.035)
        ])
    );
    assert_eq!(
        states[4].joint_values,
        joint_values(&[("panda_finger_joint1", 0.0), ("panda_finger_joint2", 0.0)])
    );
}

#[test]
fn panda_virtual_joint_matches_reference() {
    let model = SrdfModel::parse_str(PANDA).unwrap();
    assert_eq!(
        model.virtual_joints(),
        &[VirtualJoint {
            name: "virtual_joint".to_owned(),
            joint_type: VirtualJointType::Floating,
            parent_frame: "world".to_owned(),
            child_link: "panda_link0".to_owned(),
        }]
    );
}

#[test]
fn panda_end_effector_matches_reference() {
    let model = SrdfModel::parse_str(PANDA).unwrap();
    assert_eq!(
        model.end_effectors(),
        &[EndEffector {
            name: "hand".to_owned(),
            parent_link: "panda_link8".to_owned(),
            parent_group: Some("panda_arm".to_owned()),
            component_group: "hand".to_owned(),
        }]
    );
}

#[test]
fn panda_disabled_collision_pairs_match_reference() {
    let model = SrdfModel::parse_str(PANDA).unwrap();
    let pairs = model.disabled_collision_pairs();
    assert_eq!(pairs.len(), 34);
    assert_eq!(pairs[0], pair("panda_link0", "panda_link1", "Adjacent"));
    assert_eq!(pairs[18], pair("panda_link6", "panda_link7", "Adjacent"));
    // The three that follow the `hand` group in the document, proving the
    // three blocks the xacro emits are concatenated in document order.
    assert_eq!(
        pairs[19],
        pair("panda_hand", "panda_leftfinger", "Adjacent")
    );
    assert_eq!(
        pairs[21],
        pair("panda_leftfinger", "panda_rightfinger", "Default")
    );
    assert_eq!(pairs[33], pair("panda_link7", "panda_rightfinger", "Never"));
}

/// `panda.srdf` writes `<passive_joint name="panda_finger_joint2"/>` inside the
/// `hand` group. Upstream only walks direct children of `<robot>`, so it is not
/// a passive joint; the reference probe reports an empty list.
#[test]
fn panda_has_no_passive_joints() {
    let model = SrdfModel::parse_str(PANDA).unwrap();
    assert_eq!(model.passive_joints(), &[] as &[String]);
    // It is not a group joint either: `hand` names one joint, not two.
    let hand = model.groups().iter().find(|g| g.name == "hand").unwrap();
    assert_eq!(names(&hand.joints), ["panda_finger_joint1"]);
}

#[test]
fn panda_has_no_optional_sections() {
    let model = SrdfModel::parse_str(PANDA).unwrap();
    assert_eq!(model.link_sphere_approximations(), &[]);
    assert_eq!(model.no_default_collision_links(), &[] as &[String]);
    assert_eq!(model.enabled_collision_pairs(), &[]);
    assert!(model.joint_properties().is_empty());
}

// ---------------------------------------------------------------- fanuc ----

#[test]
fn fanuc_parses() {
    let model = SrdfModel::parse_str(FANUC).expect("fanuc.srdf parses");
    assert_eq!(model.name(), Some("fanuc"));
    assert_eq!(model.diagnostics(), &[]);
}

#[test]
fn fanuc_matches_reference() {
    let model = SrdfModel::parse_str(FANUC).unwrap();
    assert_eq!(
        model.groups(),
        &[Group {
            name: "manipulator".to_owned(),
            chains: vec![Chain {
                base_link: "base_link".to_owned(),
                tip_link: "tool0".to_owned(),
            }],
            ..Group::default()
        }]
    );
    assert_eq!(
        model.virtual_joints(),
        &[VirtualJoint {
            name: "FixedBase".to_owned(),
            joint_type: VirtualJointType::Fixed,
            parent_frame: "world".to_owned(),
            child_link: "base_link".to_owned(),
        }]
    );

    let states = model.group_states();
    assert_eq!(states.len(), 1);
    assert_eq!(states[0].name, "all-zeros");
    assert_eq!(states[0].group, "manipulator");
    assert_eq!(
        states[0].joint_values,
        joint_values(&[
            ("joint_1", 0.0),
            ("joint_2", 0.0),
            ("joint_3", 0.0),
            ("joint_4", 0.0),
            ("joint_5", 0.0),
            ("joint_6", 0.0),
        ])
    );

    assert_eq!(
        model.disabled_collision_pairs(),
        &[
            pair("base_link", "link_1", "Adjacent"),
            pair("base_link", "link_2", "Never"),
            pair("link_1", "link_2", "Adjacent"),
            pair("link_1", "link_3", "Never"),
            pair("link_2", "link_3", "Adjacent"),
            pair("link_3", "link_4", "Adjacent"),
            pair("link_3", "link_5", "Never"),
            pair("link_3", "link_6", "Never"),
            pair("link_4", "link_5", "Adjacent"),
            pair("link_5", "link_6", "Adjacent"),
        ]
    );
    assert_eq!(model.end_effectors(), &[]);
    assert_eq!(model.passive_joints(), &[] as &[String]);
}

// ------------------------------------------------------- dual-arm panda ----

#[test]
fn dual_arm_panda_parses() {
    let model = SrdfModel::parse_str(DUAL_ARM_PANDA).expect("dual-arm panda.srdf parses");
    assert_eq!(model.name(), Some("panda"));
}

#[test]
fn dual_arm_panda_groups_and_states() {
    let model = SrdfModel::parse_str(DUAL_ARM_PANDA).unwrap();
    assert_eq!(
        model.groups(),
        &[
            Group {
                name: "left_panda_arm".to_owned(),
                chains: vec![Chain {
                    base_link: "left_panda_link0".to_owned(),
                    tip_link: "left_panda_link8".to_owned(),
                }],
                ..Group::default()
            },
            Group {
                name: "right_panda_arm".to_owned(),
                chains: vec![Chain {
                    base_link: "right_panda_link0".to_owned(),
                    tip_link: "right_panda_link8".to_owned(),
                }],
                ..Group::default()
            },
        ]
    );

    let states = model.group_states();
    assert_eq!(states.len(), 2);
    assert_eq!(states[0].group, "left_panda_arm");
    assert_eq!(states[1].group, "right_panda_arm");
    assert!(states.iter().all(|s| s.name == "ready"));

    assert_eq!(model.virtual_joints(), &[]);
    assert_eq!(model.disabled_collision_pairs().len(), 68);
}

/// Both end effectors name `left_hand` / `right_hand`, which this SRDF never
/// defines as groups. Upstream drops an end effector whose component group is
/// unknown — that check needs no URDF, so it happens here too.
#[test]
fn dual_arm_panda_drops_end_effectors_with_undefined_groups() {
    let model = SrdfModel::parse_str(DUAL_ARM_PANDA).unwrap();
    assert_eq!(model.end_effectors(), &[]);
    assert_eq!(
        model.diagnostics(),
        &[
            Diagnostic::UnknownGroup {
                element: "end_effector",
                name: "left_hand".to_owned(),
                group: "left_hand".to_owned(),
            },
            Diagnostic::UnknownGroup {
                element: "end_effector",
                name: "right_hand".to_owned(),
                group: "right_hand".to_owned(),
            },
        ]
    );
}
