// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Shared test-fixture assertions for moveit-rs crates.
//!
//! Not a port of any upstream file -- upstream has no equivalent, since this
//! guards against a defect specific to *this* port's test fixtures, not
//! upstream's own (`RobotModel::updated_link_names()` is this port's own
//! accessor; nothing in moveit2 needs a matching test-side gate). See
//! [`assert_group_has_updated_links`]'s own doc for the defect it closes.
//!
//! Listed under `[dev-dependencies]` only, never `[dependencies]`: every
//! function here exists to be called from a `#[cfg(test)]` fixture builder.
//! It was first written independently in two places --
//! `moveit-distance-field`'s own `#[cfg(test)] mod test_support` and inline
//! `assert!`s in `moveit-planners-chomp`'s `optimizer.rs`/`planner.rs` --
//! before being lifted here so both route through one definition instead of
//! two copies drifting apart.
//!
//! A `#[cfg(test)]`-gated module in one crate cannot be imported by another
//! crate's tests at all, so some cross-crate boundary was unavoidable once a
//! second crate needed the same guard. The alternative considered was a
//! `pub` item behind a `test-support` feature on `moveit-model` itself,
//! avoiding a new crate; rejected because a feature that is off by default
//! is exactly the shape that escapes a crate-scoped `cargo clippy -p
//! moveit-model --all-targets` / `cargo doc -p moveit-model` gate (neither
//! passes `--features test-support` or `--all-features`), so a regression in
//! the gated code would pass every gate this repo actually runs per-crate.
//! A small always-compiled crate has no such blind spot: its own `-p
//! moveit-test-support` gate always sees it.

use moveit_model::RobotModel;

/// Fails loudly, at fixture-construction time, if `group_name`'s
/// `updated_link_names()` is empty -- the set every group-scoped
/// self/robot-collision or trajectory-group-name check across moveit-rs
/// actually walks (e.g. `moveit-distance-field`'s
/// `generate_distance_field_cache_entry`, `moveit-collision`'s
/// `ParryCollisionEnv::active_group_links`). A group can look fine by
/// `link_names()`/`joint_names()` (both plain topology, unfiltered by
/// whether a joint is active) and still resolve zero *updated* links
/// whenever none of its joints are active; any presence/absence assertion
/// built on that set then passes with nothing actually checked.
///
/// Call this right after building any test fixture whose model is meant to
/// exercise group-scoped collision checking or anything else keyed off a
/// group's *updated* link set, so a future edit that collapses the
/// fixture's group to zero active joints fails here, at the one shared
/// choke point, instead of downstream as a silently-passing assertion.
///
/// # Panics
///
/// Panics if `group_name` does not resolve on `model`, or if it resolves to
/// a group whose `updated_link_names()` is empty.
pub fn assert_group_has_updated_links(model: &RobotModel, group_name: &str) {
    let group = model
        .joint_model_group(group_name)
        .unwrap_or_else(|e| panic!("test fixture group {group_name:?} must resolve: {e}"));
    assert!(
        !group.updated_link_names().is_empty(),
        "test fixture group {group_name:?} has an empty updated_link_names() -- every \
         self/robot-collision check walks exactly this set, so any assertion built on it \
         (e.g. `assert!(!result.collision)`) would pass vacuously with nothing actually \
         checked. This usually means the group's connecting joint(s) are all \
         `type=\"fixed\"`/have no active DOF -- give the fixture at least one active \
         (e.g. revolute) joint."
    );
}

#[cfg(test)]
mod tests {
    use moveit_model::MeshSearchPaths;
    use moveit_srdf::SrdfModel;

    use super::*;

    fn build(urdf_xml: &str, srdf_xml: &str) -> RobotModel {
        let urdf: urdf_rs::Robot = urdf_rs::read_from_string(urdf_xml).expect("urdf must parse");
        let srdf = SrdfModel::parse_str(srdf_xml).expect("srdf must parse");
        RobotModel::from_urdf_and_srdf(&urdf, urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("model must build")
    }

    fn two_link_urdf(joint_type: &str) -> String {
        format!(
            r#"<robot name="test">
    <link name="base"/>
    <link name="mid"/>
    <link name="tip"/>
    <joint name="j1" type="{joint_type}">
        <parent link="base"/><child link="mid"/><axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
    </joint>
    <joint name="j2" type="{joint_type}">
        <parent link="mid"/><child link="tip"/><axis xyz="0 0 1"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
    </joint>
</robot>"#
        )
    }

    const SRDF: &str = r#"<robot name="test">
    <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
    <group name="chain">
        <chain base_link="base" tip_link="tip"/>
    </group>
</robot>"#;

    #[test]
    fn accepts_a_group_with_an_active_joint() {
        let model = build(&two_link_urdf("revolute"), SRDF);
        assert_group_has_updated_links(&model, "chain");
    }

    #[test]
    #[should_panic(expected = "has an empty updated_link_names()")]
    fn rejects_a_group_with_no_active_joints() {
        let model = build(&two_link_urdf("fixed"), SRDF);
        assert_group_has_updated_links(&model, "chain");
    }

    #[test]
    #[should_panic(expected = "must resolve")]
    fn rejects_an_unresolvable_group_name() {
        let model = build(&two_link_urdf("revolute"), SRDF);
        assert_group_has_updated_links(&model, "no_such_group");
    }
}
