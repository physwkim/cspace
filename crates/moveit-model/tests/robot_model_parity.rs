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

use moveit_model::{Diagnostic, MeshSearchPaths, RobotModel};
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
    #[serde(default)]
    link_details: Vec<OracleLinkDetail>,
    #[serde(default)]
    group_end_effectors: std::collections::BTreeMap<String, OracleGroupEndEffector>,
    /// Ground truth for `JointModelGroup::default_state_names`/
    /// `variable_default_positions`: group name -> state name -> variable
    /// name -> value. A state's variable map may be a strict subset of the
    /// group's variables — see `assert_matches_oracle`'s `group_states` loop.
    #[serde(default)]
    group_states: std::collections::BTreeMap<
        String,
        std::collections::BTreeMap<String, std::collections::BTreeMap<String, f64>>,
    >,
    /// Ground truth for `JointModelGroup::is_chain`, keyed by group name.
    #[serde(default)]
    group_is_chain: std::collections::BTreeMap<String, bool>,
}

/// Ground truth for `RobotModel::get_common_root`, one entry per queried
/// pair, in request order — see `pr2_common_root_matches_the_oracle`.
#[derive(Deserialize)]
struct OracleCommonRootPair {
    a: String,
    b: String,
    common_root: String,
}

#[derive(Deserialize)]
struct OracleCommonRootResponse {
    result: Vec<OracleCommonRootPair>,
}

/// Ground truth for `JointModelGroup`'s end-effector fields
/// (`is_end_effector`/`end_effector_name`/`end_effector_parent`/
/// `attached_end_effector_names`), keyed by group name.
#[derive(Deserialize)]
struct OracleGroupEndEffector {
    end_effector_name: Option<String>,
    attached_end_effector_names: Vec<String>,
    end_effector_parent: Option<OracleEndEffectorParent>,
}

#[derive(Deserialize)]
struct OracleEndEffectorParent {
    group: Option<String>,
    link: String,
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

/// The oracle's per-link geometry, added to ground `LinkModel`'s collision
/// shapes, bounding-box offset and visual mesh metadata (`LinkModel`'s doc
/// comment, deviation 4) rather than leaving them untested by any fixture.
#[derive(Deserialize)]
struct OracleLinkDetail {
    name: String,
    shape_types: Vec<String>,
    centered_bounding_box_offset: [Option<f64>; 3],
    visual_mesh_filename: Option<String>,
    #[serde(default)]
    visual_mesh_origin: Option<[f64; 16]>,
    #[serde(default)]
    visual_mesh_scale: Option<[f64; 3]>,
}

#[derive(Deserialize)]
struct OracleResponse {
    result: OracleModelInfo,
}

fn to_row_major_4x4(transform: &moveit_geometry::Isometry3) -> [f64; 16] {
    let m = transform.to_homogeneous();
    let mut out = [0.0; 16];
    for r in 0..4 {
        for c in 0..4 {
            out[r * 4 + c] = m[(r, c)];
        }
    }
    out
}

/// Ground truth for `LinkModel`'s collision/visual geometry (`LinkModel`'s
/// doc comment, deviation 4): for every link, the supported-shape count and,
/// where the link has no `<mesh>`/`<capsule>` collision element at all, the
/// exact bounding-box offset (a link with any mesh collision gets a real
/// mesh-vertex-derived offset from the oracle that this port cannot
/// reproduce without a mesh-file loader, so that comparison is skipped for
/// those links rather than faked). Visual mesh filename/origin/scale are
/// plain URDF metadata, not loaded geometry, so those are asserted
/// unconditionally.
fn assert_link_geometry_matches_oracle(model: &RobotModel, expected: &[OracleLinkDetail]) {
    for detail in expected {
        let link = model
            .link_model(&detail.name)
            .unwrap_or_else(|_| panic!("missing link '{}'", detail.name));

        let has_unsupported_shape = detail
            .shape_types
            .iter()
            .any(|kind| kind == "mesh" || kind == "capsule");
        let supported_shape_count = detail
            .shape_types
            .iter()
            .filter(|kind| *kind != "mesh" && *kind != "capsule")
            .count();
        assert_eq!(
            link.shapes().len(),
            supported_shape_count,
            "supported shape count for link '{}'",
            detail.name
        );

        if !has_unsupported_shape {
            let offset = link.centered_bounding_box_offset();
            for (i, expected_component) in detail.centered_bounding_box_offset.iter().enumerate() {
                let expected_component = expected_component
                    .unwrap_or_else(|| panic!("non-finite bounding box offset component {i} for link '{}' with no mesh/capsule shape", detail.name));
                assert!(
                    (offset[i] - expected_component).abs() < 1e-9,
                    "bounding box offset component {i} for link '{}': {} vs oracle {}",
                    detail.name,
                    offset[i],
                    expected_component
                );
            }
        }

        assert_eq!(
            link.visual_mesh_filename(),
            detail.visual_mesh_filename.as_deref(),
            "visual mesh filename for link '{}'",
            detail.name
        );
        if let Some(expected_origin) = &detail.visual_mesh_origin {
            let origin = to_row_major_4x4(link.visual_mesh_origin());
            for (i, (actual, expected)) in origin.iter().zip(expected_origin.iter()).enumerate() {
                assert!(
                    (actual - expected).abs() < 1e-9,
                    "visual mesh origin component {i} for link '{}': {actual} vs oracle {expected}",
                    detail.name
                );
            }
        }
        if let Some(expected_scale) = &detail.visual_mesh_scale {
            let scale = link.visual_mesh_scale();
            for (i, expected_component) in expected_scale.iter().enumerate() {
                assert!(
                    (scale[i] - expected_component).abs() < 1e-9,
                    "visual mesh scale component {i} for link '{}': {} vs oracle {expected_component}",
                    detail.name,
                    scale[i]
                );
            }
        }
    }
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

fn build_model_with_urdf(urdf_file: &str, srdf_file: &str) -> (RobotModel, urdf_rs::Robot) {
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
    let model = RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build");
    (model, urdf)
}

/// The diagnostics [`RobotModel::from_urdf_and_srdf`] must report for a
/// fixture's link geometry: one [`Diagnostic::UnsupportedLinkGeometry`] per
/// `<collision>` element whose geometry is `<mesh>` or `<capsule>` (see
/// `LinkModel`'s doc comment, deviation 4), derived directly from the URDF
/// rather than hand-transcribed, in the same per-link order
/// [`RobotModel::link_names`] visits.
///
/// [`build_model_with_urdf`] passes [`MeshSearchPaths::none`] -- this test's
/// job is model *structure* (names, frames, groups, ordering), which is
/// orthogonal to whether a `<mesh>` element actually resolves to geometry;
/// mesh resolution itself is covered by `moveit-model`'s own
/// `mesh_search_paths` and `robot_model` unit tests and by
/// `moveit-collision`'s parity tests, against real search paths. So every
/// `<mesh>` element here is expected to report the fixed "no mesh search
/// paths were configured" detail, regardless of whether that resource would
/// actually resolve under `fixtures/meshes/`.
fn expected_unsupported_link_geometry_diagnostics(
    model: &RobotModel,
    urdf: &urdf_rs::Robot,
) -> Vec<Diagnostic> {
    let links_by_name: std::collections::HashMap<&str, &urdf_rs::Link> =
        urdf.links.iter().map(|l| (l.name.as_str(), l)).collect();
    model
        .link_names()
        .iter()
        .flat_map(|name| {
            let urdf_link = links_by_name[name.as_str()];
            urdf_link.collision.iter().filter_map(move |collision| {
                let (kind, detail) = match &collision.geometry {
                    urdf_rs::Geometry::Mesh { .. } => (
                        Some("mesh"),
                        Some("no mesh search paths were configured".to_string()),
                    ),
                    urdf_rs::Geometry::Capsule { .. } => (Some("capsule"), None),
                    _ => (None, None),
                };
                kind.map(|kind| Diagnostic::UnsupportedLinkGeometry {
                    link: name.clone(),
                    kind,
                    detail,
                })
            })
        })
        .collect()
}

/// Like [`build_model_with_urdf`], but also asserts the SRDF parsed with no
/// diagnostics — appropriate for panda/fanuc/PR2, whose SRDFs are clean, but
/// not for dual-arm panda, whose two `UnknownGroup` diagnostics are expected
/// (see `dual_arm_panda_robot_model_matches_the_oracle`).
fn build_clean_model_with_urdf(urdf_file: &str, srdf_file: &str) -> (RobotModel, urdf_rs::Robot) {
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
    build_model_with_urdf(urdf_file, srdf_file)
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

    for (name, expected_eef) in &expected.group_end_effectors {
        let group = model
            .joint_model_group(name)
            .unwrap_or_else(|_| panic!("missing group '{name}'"));

        assert_eq!(
            group.is_end_effector(),
            expected_eef.end_effector_name.is_some(),
            "is_end_effector for group '{name}'"
        );
        assert_eq!(
            (!group.end_effector_name().is_empty()).then(|| group.end_effector_name().to_string()),
            expected_eef.end_effector_name,
            "end_effector_name for group '{name}'"
        );
        assert_eq!(
            group.attached_end_effector_names(),
            expected_eef.attached_end_effector_names.as_slice(),
            "attached_end_effector_names for group '{name}'"
        );

        match (
            &expected_eef.end_effector_parent,
            group.end_effector_parent(),
        ) {
            (None, None) => {}
            (Some(expected_parent), Some(actual_parent)) => {
                assert_eq!(
                    actual_parent.group, expected_parent.group,
                    "end_effector_parent group for group '{name}'"
                );
                assert_eq!(
                    actual_parent.link, expected_parent.link,
                    "end_effector_parent link for group '{name}'"
                );
            }
            (expected_parent, actual_parent) => panic!(
                "end_effector_parent presence mismatch for group '{name}': oracle {expected_parent:?} vs model {actual_parent:?}"
            ),
        }
    }

    // The oracle serialises `getDefaultStateNames()` through a
    // key-sorted JSON object, so document order is not recoverable from it —
    // compare the name set, not the sequence, then check every state's
    // values exactly.
    for (group_name, expected_states) in &expected.group_states {
        let group = model
            .joint_model_group(group_name)
            .unwrap_or_else(|_| panic!("missing group '{group_name}'"));

        let mut actual_names: Vec<&String> = group.default_state_names().iter().collect();
        actual_names.sort();
        let mut expected_names: Vec<&String> = expected_states.keys().collect();
        expected_names.sort();
        assert_eq!(
            actual_names, expected_names,
            "default_state_names for group '{group_name}'"
        );

        for (state_name, expected_values) in expected_states {
            let actual_values = group
                .variable_default_positions(state_name)
                .unwrap_or_else(|| {
                    panic!("missing group_state '{state_name}' for group '{group_name}'")
                });
            assert_eq!(
                actual_values, expected_values,
                "group_state '{state_name}' for group '{group_name}'"
            );
        }
    }

    for (group_name, expected_is_chain) in &expected.group_is_chain {
        let group = model
            .joint_model_group(group_name)
            .unwrap_or_else(|_| panic!("missing group '{group_name}'"));
        assert_eq!(
            group.is_chain(),
            *expected_is_chain,
            "is_chain for group '{group_name}'"
        );
    }
}

/// Ground truth for `RobotModel::get_common_root` against a committed
/// oracle fixture of `{a, b} -> common_root` triples, covering the
/// invariant boundaries the porting task calls out: same joint, ancestor in
/// each direction, a pair spanning the root `world_joint`, and — the case
/// that breaks a textbook LCA — two joints that are themselves direct
/// siblings under the same link (`l_shoulder_pan_joint`/
/// `r_shoulder_pan_joint`, both parented at `torso_lift_link`, and
/// `fl_caster_rotation_joint`/`fr_caster_rotation_joint`, both parented at
/// `base_link`), where upstream's own `getCommonRoot` returns the model's
/// global root rather than the joint that actually branches them — see
/// `RobotModel::get_common_root`'s doc comment for why.
fn assert_common_root_matches_oracle(model: &RobotModel, fixture_file: &str) {
    let path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        fixture_file
    );
    let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let response: OracleCommonRootResponse =
        serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"));

    let joint_index = |name: &str| -> usize {
        model
            .joint_names()
            .iter()
            .position(|n| n == name)
            .unwrap_or_else(|| panic!("no joint named '{name}'"))
    };

    for pair in &response.result {
        let got = model.get_common_root(joint_index(&pair.a), joint_index(&pair.b));
        assert_eq!(
            model.joint_names()[got],
            pair.common_root,
            "get_common_root({}, {})",
            pair.a,
            pair.b
        );
    }
}

impl std::fmt::Debug for OracleEndEffectorParent {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("OracleEndEffectorParent")
            .field("group", &self.group)
            .field("link", &self.link)
            .finish()
    }
}

#[test]
fn panda_robot_model_matches_the_oracle() {
    let (model, urdf) = build_clean_model_with_urdf("panda.urdf", "panda.srdf");
    let expected = load_fixture("panda_model_info.json");
    assert_matches_oracle(&model, &expected);

    // Every panda link's `<collision>` is exactly one `<mesh>` — see
    // `LinkModel`'s doc comment, deviation 4.
    assert_eq!(
        model.diagnostics(),
        expected_unsupported_link_geometry_diagnostics(&model, &urdf).as_slice()
    );
    assert_link_geometry_matches_oracle(&model, &expected.link_details);

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
    let (model, urdf) = build_clean_model_with_urdf("fanuc.urdf", "fanuc.srdf");
    let expected = load_fixture("fanuc_model_info.json");
    assert_matches_oracle(&model, &expected);

    // Every fanuc link's `<collision>` is exactly one `<mesh>` — see
    // `LinkModel`'s doc comment, deviation 4.
    assert_eq!(
        model.diagnostics(),
        expected_unsupported_link_geometry_diagnostics(&model, &urdf).as_slice()
    );
    assert_link_geometry_matches_oracle(&model, &expected.link_details);

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
    let (model, urdf) = build_clean_model_with_urdf("pr2.urdf", "pr2.srdf");
    let expected = load_fixture("pr2_model_info.json");
    assert_matches_oracle(&model, &expected);

    // pr2 mixes `<mesh>` and `<box>` collision geometry — this is the
    // fixture that actually exercises `LinkModel::shapes` being non-empty
    // (see `LinkModel`'s doc comment, deviation 4).
    assert_eq!(
        model.diagnostics(),
        expected_unsupported_link_geometry_diagnostics(&model, &urdf).as_slice()
    );
    assert_link_geometry_matches_oracle(&model, &expected.link_details);

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

    // The real "missing joint" case for
    // `JointModelGroup::variable_default_positions`: the fixture SRDF's
    // `tuck_arms` group_state only values `l_shoulder_pan_joint`, and the
    // oracle itself logs "Group state 'tuck_arms' doesn't specify all group
    // joints in group 'arms'" for the other 13 active joints in `arms` — so
    // `buildGroupStates` must store exactly that one variable, not default
    // the rest to 0.0.
    let arms = model.joint_model_group("arms").unwrap();
    assert_eq!(arms.default_state_names(), ["tuck_arms"]);
    let tuck_arms = arms.variable_default_positions("tuck_arms").unwrap();
    assert_eq!(tuck_arms.len(), 1);
    assert_eq!(tuck_arms.get("l_shoulder_pan_joint"), Some(&0.2));

    // `left_arm`/`right_arm` are each a single unbranched chain;
    // `arms`/`arms_and_torso`/`whole_body` are not, since they union two
    // independently-rooted chains.
    assert!(model.joint_model_group("left_arm").unwrap().is_chain());
    assert!(model.joint_model_group("right_arm").unwrap().is_chain());
    assert!(!model.joint_model_group("arms").unwrap().is_chain());

    assert_common_root_matches_oracle(&model, "pr2_common_root.json");
}

/// Dual-arm panda's SRDF has no `<virtual_joint>` element at all — the
/// oracle itself logs "No root/virtual joint specified in SRDF. Assuming
/// fixed joint", so its root joint is upstream's `ASSUMED_FIXED_ROOT_JOINT`
/// fallback and `model_frame`/`root_link` both come out as the URDF's root
/// link name (`world`), not a name chosen from the SRDF.
#[test]
fn dual_arm_panda_robot_model_matches_the_oracle() {
    let (model, urdf) = build_model_with_urdf("dual_arm_panda.urdf", "dual_arm_panda.srdf");
    let expected = load_fixture("dual_arm_panda_model_info.json");
    assert_matches_oracle(&model, &expected);

    // Every dual-arm panda link's `<collision>` is exactly one `<mesh>` —
    // see `LinkModel`'s doc comment, deviation 4.
    assert_eq!(
        model.diagnostics(),
        expected_unsupported_link_geometry_diagnostics(&model, &urdf).as_slice()
    );
    assert_link_geometry_matches_oracle(&model, &expected.link_details);

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
