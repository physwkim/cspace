// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Shared test-fixture helpers for moveit-rs crates -- assertions that guard
//! against a specific fixture defect, and small conversions oracle-parity
//! fixtures need on both sides of a comparison.
//!
//! Not a port of any upstream file. This used to add "upstream has no
//! equivalent test-side machinery of its own", which is false: upstream
//! ships `moveit_core/utils/include/moveit/utils/robot_model_test_utils.hpp`
//! and `src/robot_model_test_utils.cpp` (programmatic `RobotModel`
//! construction plus `moveit_resources` loaders) and
//! `include/moveit/utils/eigen_test_utils.hpp` (gtest predicates over Eigen
//! types). Both live outside a `test/` directory, so they are inside this
//! port's measured corpus and carry their own `doc/port-coverage.md` rows.
//!
//! What is true is that neither is what this crate holds, because this port
//! solves both problems elsewhere: robot models come from committed
//! `fixtures/*.{urdf,srdf}` through [`cspace_model::RobotModel::from_urdf_and_srdf`],
//! and Eigen-shaped equality is asserted with `approx::assert_relative_eq!`.
//! See [`assert_group_has_updated_links`]'s and [`isometry_from_row_major`]'s
//! own docs for what each closes.
//!
//! Listed under `[dev-dependencies]` only, never `[dependencies]`: every
//! function here exists to be called from a `#[cfg(test)]` fixture builder.
//! Each was first written independently in more than one crate --
//! [`assert_group_has_updated_links`] in `cspace-distance-field`'s own
//! `#[cfg(test)] mod test_support` and inline `assert!`s in
//! `cspace-planners-chomp`'s `optimizer.rs`/`planner.rs`;
//! [`isometry_from_row_major`] byte-for-byte identically in five
//! oracle-parity test files across `cspace-distance-field` and
//! `cspace-collision` -- before being lifted here so every call site routes
//! through one definition instead of copies drifting apart.
//!
//! A `#[cfg(test)]`-gated module in one crate cannot be imported by another
//! crate's tests at all, so some cross-crate boundary was unavoidable once a
//! second crate needed the same guard. The alternative considered was a
//! `pub` item behind a `test-support` feature on `cspace-model` itself,
//! avoiding a new crate; rejected because a feature that is off by default
//! is exactly the shape that escapes a crate-scoped `cargo clippy -p
//! cspace-model --all-targets` / `cargo doc -p cspace-model` gate (neither
//! passes `--features test-support` or `--all-features`), so a regression in
//! the gated code would pass every gate this repo actually runs per-crate.
//! A small always-compiled crate has no such blind spot: its own `-p
//! cspace-test-support` gate always sees it.

use cspace_model::RobotModel;
use nalgebra::{Isometry3, Matrix3, Translation3, UnitQuaternion};

/// Row-major 4x4 -> [`nalgebra::Isometry3`], matching the oracle's own
/// `toRowMajor4x4`/`fromRowMajor4x4` (`oracle.cpp`) -- the shape every
/// oracle-parity fixture's pose fields are dumped in. Independently
/// reimplemented byte-for-byte identically in five oracle-parity test files
/// across `cspace-distance-field` and `cspace-collision` before being lifted
/// here; see this crate's module doc for why a shared crate, not a shared
/// `#[cfg(test)]` module, is what closes that kind of duplication.
pub fn isometry_from_row_major(m: &[f64; 16]) -> Isometry3<f64> {
    let rotation = Matrix3::new(m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]);
    let translation = Translation3::new(m[3], m[7], m[11]);
    Isometry3::from_parts(translation, UnitQuaternion::from_matrix(&rotation))
}

/// Fails loudly, at fixture-construction time, if `group_name`'s
/// `updated_link_names()` is empty -- the set every group-scoped
/// self/robot-collision or trajectory-group-name check across moveit-rs
/// actually walks (e.g. `cspace-distance-field`'s
/// `generate_distance_field_cache_entry`, `cspace-collision`'s
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

/// A single point where a fixture-vs-oracle parity test deliberately does
/// *not* assert equality on one field, because the fixture was captured from
/// a real upstream run that still carries a bug this port has since fixed
/// (or upstream fixed independently -- either way, the port's output can no
/// longer agree with the captured value on that field).
///
/// A bare `if known_deviation { skip the equality assert }` -- the shape
/// every such deviation in this port's parity tests used before this type
/// existed, e.g. `crates/cspace-state/tests/dynamics_parity.rs`'s
/// `max_payload_is_known_deviation` -- absorbs every future regression on
/// that field too, silently: once the assertion is skipped, nothing
/// distinguishes "still correctly diverging from the buggy oracle value"
/// from "the fix regressed and the port's output now silently matches
/// something else entirely (including the oracle's original buggy value),"
/// or "the fixture no longer exercises the case that made the deviation
/// necessary." [`KnownOracleDeviation::finish`] closes that gap: it panics
/// unless at least one observed case actually differed from its oracle
/// value, so any of those three failure modes turns this gate red instead
/// of silently doing nothing.
///
/// Construct one per deviating field per fixture, call [`Self::observe`]
/// once per case in place of the normal equality assertion, then
/// [`Self::finish`] once after the last case. The field name, upstream
/// citation, and fix commit sha passed to [`Self::new`] all appear in
/// [`Self::finish`]'s panic message, so a failure names its own evidence
/// without a reader needing to find the call site first.
pub struct KnownOracleDeviation {
    field: &'static str,
    upstream_citation: &'static str,
    fix_commit: &'static str,
    diverged_on: Option<String>,
}

impl KnownOracleDeviation {
    /// `field` names the quantity being deviated (e.g. `"max_payload
    /// (joint_saturated, payload)"`); `upstream_citation` is the exact
    /// upstream `file:line` the deviation is measured against;
    /// `fix_commit` is the sha of the commit that decided to diverge. All
    /// three are echoed by [`Self::finish`]'s panic message.
    pub fn new(
        field: &'static str,
        upstream_citation: &'static str,
        fix_commit: &'static str,
    ) -> Self {
        Self {
            field,
            upstream_citation,
            fix_commit,
            diverged_on: None,
        }
    }

    /// Records one case's comparison in place of the normal equality
    /// assertion `field` would otherwise get. `oracle` is the fixture's own
    /// captured value; `actual` is the port's current output. Asserts
    /// nothing by itself -- call [`Self::finish`] once every case in the
    /// fixture has been observed.
    pub fn observe<T: PartialEq>(&mut self, case: &str, oracle: &T, actual: &T) {
        if self.diverged_on.is_none() && oracle != actual {
            self.diverged_on = Some(case.to_string());
        }
    }

    /// Call once, after every case has been passed to [`Self::observe`].
    ///
    /// # Panics
    ///
    /// Panics if no observed case actually diverged from its oracle value --
    /// the one condition every call site's skipped equality assertion is
    /// conditioned on.
    pub fn finish(self) {
        assert!(
            self.diverged_on.is_some(),
            "known deviation {:?} ({}, fixed at {}) never diverged from the oracle-captured \
             value on any observed case in this fixture -- either the fix regressed and the \
             port's output now silently matches the oracle again, or the fixture no longer \
             exercises the case this deviation exists for. Either way this call site must be \
             reconsidered, not kept as-is.",
            self.field,
            self.upstream_citation,
            self.fix_commit
        );
    }
}

#[cfg(test)]
mod tests {
    use cspace_model::MeshSearchPaths;
    use cspace_srdf::SrdfModel;

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

    #[test]
    fn known_oracle_deviation_finishes_once_a_case_diverges() {
        let mut deviation = KnownOracleDeviation::new("field", "upstream.cpp:1", "deadbeef");
        deviation.observe("case_a", &1, &1);
        deviation.observe("case_b", &1, &2);
        deviation.finish();
    }

    #[test]
    #[should_panic(expected = "never diverged from the oracle-captured value")]
    fn known_oracle_deviation_panics_when_every_case_matches_the_oracle() {
        let mut deviation = KnownOracleDeviation::new("field", "upstream.cpp:1", "deadbeef");
        deviation.observe("case_a", &1, &1);
        deviation.observe("case_b", &2, &2);
        deviation.finish();
    }

    #[test]
    #[should_panic(expected = "never diverged from the oracle-captured value")]
    fn known_oracle_deviation_panics_with_no_cases_observed() {
        KnownOracleDeviation::new("field", "upstream.cpp:1", "deadbeef").finish();
    }
}
