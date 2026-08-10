// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The parts of `RobotState::interpolate`'s three overloads that the oracle
//! comparison cannot reach.
//!
//! `tools/ci/verify-phase2-state-sweep.sh`'s `state_interpolation` clause
//! drives all three overloads against real moveit2 whole-state, so the
//! values they produce are compared there and are not re-asserted here. Two
//! things are outside what that comparison can carry, and they are what this
//! file covers:
//!
//! * **A non-finite `t`.** The oracle speaks JSON, and `serde_json` encodes
//!   both `NaN` and infinity as `null` — a request carrying one would either
//!   fail to serialize or arrive as a different number, so the sweep can
//!   only ever send finite `t`. Upstream's `checkInterpolationParamBounds`
//!   (`robot_model.hpp:63`) throws on exactly those two values, and only two
//!   of the three overloads call it.
//! * **Which joints were marked stale.** The oracle op reports variable
//!   positions; upstream's dirty bookkeeping is internal to `RobotState` and
//!   has no wire representation. An overload that wrote the right positions
//!   and marked nothing dirty agrees on every case in the sweep and then
//!   returns stale transforms from the next `update()`.

use std::fs;

use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/state/{}"),
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

/// Two states a measurable distance apart, and a third holding neither's
/// values — the destination, whose prior contents are what makes "this
/// overload did not touch that variable" observable.
fn endpoints(model: &RobotModel) -> (RobotState<'_>, RobotState<'_>, RobotState<'_>) {
    let mut from = RobotState::new(model);
    from.set_to_default_values();

    let mut to = RobotState::new(model);
    to.set_to_default_values();
    let shifted: Vec<f64> = to.positions().iter().map(|v| v + 0.25).collect();
    to.set_variable_positions(&shifted);

    let mut out = RobotState::new(model);
    out.set_to_default_values();
    let seeded: Vec<f64> = out.positions().iter().map(|v| v - 0.75).collect();
    out.set_variable_positions(&seeded);

    (from, to, out)
}

// ---- `t` bounds: the two operands of upstream's one `||` ------------------

/// Upstream's `checkInterpolationParamBounds` throws one exception carrying
/// one message for both operands of its `||`, and the port returns that
/// message verbatim in an `Error::Other`, whose `Display` is `{0}`.
///
/// Compared whole rather than by substring, and asserted at all rather than
/// left as a bare `expect_err`: `interpolate_group` can also fail with
/// `Error::UnknownName`, so "it returned some error" does not say the
/// bounds check is what refused the call.
fn assert_is_the_bounds_exception(error: impl std::fmt::Display) {
    assert_eq!(error.to_string(), "Interpolation parameter is NaN or inf.");
}

#[test]
fn nan_t_is_refused_by_the_whole_model_form() {
    let model = build_model("panda.urdf", "panda.srdf");
    let (from, to, mut out) = endpoints(&model);
    let before = out.positions().to_vec();

    let error = from
        .interpolate(&to, f64::NAN, &mut out)
        .expect_err("upstream throws Exception for a NaN interpolation parameter");
    assert_is_the_bounds_exception(error);
    assert_eq!(
        out.positions(),
        before.as_slice(),
        "a refused interpolation must not have written the destination"
    );
}

#[test]
fn infinite_t_is_refused_by_the_whole_model_form() {
    let model = build_model("panda.urdf", "panda.srdf");
    let (from, to, mut out) = endpoints(&model);

    assert_is_the_bounds_exception(
        from.interpolate(&to, f64::INFINITY, &mut out)
            .expect_err("upstream throws Exception for an infinite interpolation parameter"),
    );
    assert_is_the_bounds_exception(
        from.interpolate(&to, f64::NEG_INFINITY, &mut out)
            .expect_err("`std::isinf` is sign-agnostic, so −inf throws too"),
    );
}

#[test]
fn nan_t_is_refused_by_the_group_form() {
    let model = build_model("panda.urdf", "panda.srdf");
    let (from, to, mut out) = endpoints(&model);
    let before = out.positions().to_vec();

    assert_is_the_bounds_exception(
        from.interpolate_group(&to, f64::NAN, &mut out, "hand")
            .expect_err("the group overload opens with the same bounds check"),
    );
    assert_eq!(out.positions(), before.as_slice());
}

/// Upstream's single-joint overload (`robot_state.cpp:1159`) is the one that
/// does *not* call `checkInterpolationParamBounds` — it opens with the
/// zero-variable early return and goes straight to `joint->interpolate`. A
/// port that "helpfully" added the check to all three would reject a call
/// real moveit2 performs, so the asymmetry is asserted rather than assumed.
#[test]
fn nan_t_reaches_the_single_joint_form() {
    let model = build_model("panda.urdf", "panda.srdf");
    let (from, to, mut out) = endpoints(&model);

    from.interpolate_joint(&to, f64::NAN, &mut out, "panda_joint1")
        .expect("upstream performs this call and produces NaN, it does not throw");
    assert!(
        out.variable_position("panda_joint1").unwrap().is_nan(),
        "the NaN must have reached the joint's own interpolation"
    );
}

/// A fixed joint is accepted by name and writes nothing, even for a `t` the
/// other two overloads reject.
///
/// This does **not** discriminate the `variable_count() == 0` early return
/// itself: deleting that return leaves this test green, because
/// `interpolate_one` on a zero-width variable range copies nothing and
/// `update_mimic_joint` has the same guard one level down (measured — the
/// mutation was run and the whole `cspace_core::state` suite stayed green). The
/// return is kept because it is upstream's, and because it is what makes
/// the ordering claim in [`RobotState::interpolate_joint`]'s doc comment
/// true; what this test pins is the reachable half — that naming a fixed
/// joint is legal rather than an `UnknownName`, and leaves the destination
/// alone.
#[test]
fn a_zero_variable_joint_is_a_no_op() {
    let model = build_model("panda.urdf", "panda.srdf");
    let (from, to, mut out) = endpoints(&model);
    let before = out.positions().to_vec();

    from.interpolate_joint(&to, f64::NAN, &mut out, "panda_hand_joint")
        .expect("a fixed joint returns before the interpolation");
    assert_eq!(out.positions(), before.as_slice());
}

// ---- Dirty marking, which the wire cannot carry ---------------------------

/// `update()` on `state` must recompute `link` — i.e. the transform it
/// returns must be the one a freshly forced update produces. Called on a
/// destination that was *clean* before the interpolation, so a missing dirty
/// mark leaves `update()` with nothing to do and returns the seed's pose.
fn assert_transforms_are_not_stale(state: &mut RobotState<'_>, link: &str) {
    let lazy = state
        .update()
        .global_link_transform(link)
        .unwrap_or_else(|e| panic!("{link}: {e}"));
    let forced = state
        .update_forced()
        .global_link_transform(link)
        .unwrap_or_else(|e| panic!("{link}: {e}"));
    assert_eq!(
        lazy, forced,
        "{link}'s transform after update() is not the one its positions imply — \
         the interpolation wrote positions without marking the joint stale"
    );
}

#[test]
fn the_whole_model_form_marks_the_destination_stale() {
    let model = build_model("panda.urdf", "panda.srdf");
    let (from, to, mut out) = endpoints(&model);
    out.update();

    from.interpolate(&to, 0.5, &mut out).unwrap();
    assert_transforms_are_not_stale(&mut out, "panda_leftfinger");
}

#[test]
fn the_group_form_marks_the_destination_stale() {
    let model = build_model("panda.urdf", "panda.srdf");
    let (from, to, mut out) = endpoints(&model);
    out.update();

    from.interpolate_group(&to, 0.5, &mut out, "panda_arm")
        .unwrap();
    assert_transforms_are_not_stale(&mut out, "panda_link7");
}

/// The group's *mimic* joints, not just its active ones: upstream's
/// `updateMimicJoints(group)` marks each follower it writes, and
/// `panda_finger_joint2`'s link is reached through no active joint of
/// `hand`.
#[test]
fn the_group_form_marks_a_written_mimic_stale() {
    let model = build_model("panda.urdf", "panda.srdf");
    let (from, to, mut out) = endpoints(&model);
    out.update();

    from.interpolate_group(&to, 0.5, &mut out, "hand").unwrap();
    assert_transforms_are_not_stale(&mut out, "panda_rightfinger");
}

#[test]
fn the_single_joint_form_marks_the_destination_stale() {
    let model = build_model("panda.urdf", "panda.srdf");
    let (from, to, mut out) = endpoints(&model);
    out.update();

    from.interpolate_joint(&to, 0.5, &mut out, "panda_joint4")
        .unwrap();
    assert_transforms_are_not_stale(&mut out, "panda_link7");
}

/// The follower of the joint that was named, which the single-joint form
/// reaches through `updateMimicJoint` rather than through the loop.
#[test]
fn the_single_joint_form_marks_the_named_joints_follower_stale() {
    let model = build_model("panda.urdf", "panda.srdf");
    let (from, to, mut out) = endpoints(&model);
    out.update();

    from.interpolate_joint(&to, 0.5, &mut out, "panda_finger_joint1")
        .unwrap();
    assert_transforms_are_not_stale(&mut out, "panda_rightfinger");
}
