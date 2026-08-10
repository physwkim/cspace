// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Regression coverage for the NaN-comparison fix in
//! `has_relative_joint_space_jump` and `has_absolute_joint_space_jump`
//! (`cartesian_interpolator.rs`): both silently read a NaN increment or
//! threshold as "not a jump", because every NaN comparison is false.
//!
//! Before this fix, on [`two_joint_model`]: a path whose second joint value
//! is NaN (any single non-finite reading, from anywhere upstream of this
//! pure function) made `check_joint_space_jump` return `Percentage(1.0)`
//! with the *entire, untruncated* path kept -- the exact same answer as a
//! path with no jump at all, for a relative-mode threshold that a later,
//! perfectly ordinary large motion in the same path would otherwise have
//! tripped. The absolute mode's blast radius is narrower (only the one
//! waypoint pair carrying the NaN is affected) but the mechanism and the
//! silent "kept, no jump" answer are the same. All three tests below
//! assert the fix: `check_joint_space_jump` now truncates at the NaN
//! waypoint instead of silently keeping the whole path.

use cspace_core::kinematics::{JumpThreshold, check_joint_space_jump};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;

/// One revolute joint and one prismatic joint on the same two-link chain,
/// so both `has_absolute_joint_space_jump` match arms (`JointType::Revolute`
/// / `JointType::Prismatic`) are reachable from a single fixture.
const TWO_JOINT_URDF: &str = r#"<?xml version="1.0"?>
<robot name="two_joint">
  <link name="root"/>
  <link name="mid"/>
  <link name="tip"/>
  <joint name="revolute_joint" type="revolute">
    <parent link="root"/>
    <child link="mid"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
  </joint>
  <joint name="prismatic_joint" type="prismatic">
    <parent link="mid"/>
    <child link="tip"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"#;

const TWO_JOINT_SRDF: &str = r#"<?xml version="1.0"?>
<robot name="two_joint">
  <group name="chain">
    <chain base_link="root" tip_link="tip"/>
  </group>
</robot>
"#;

fn two_joint_model() -> RobotModel {
    let urdf = urdf_rs::read_from_string(TWO_JOINT_URDF).expect("inline URDF must parse");
    let srdf = SrdfModel::parse_str(TWO_JOINT_SRDF).expect("inline SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, TWO_JOINT_URDF, &srdf, &MeshSearchPaths::none())
        .expect("inline model must build")
}

fn state_at<'m>(model: &'m RobotModel, revolute: f64, prismatic: f64) -> RobotState<'m> {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    state
        .set_variable_position("revolute_joint", revolute)
        .expect("revolute_joint is a variable of two_joint");
    state
        .set_variable_position("prismatic_joint", prismatic)
        .expect("prismatic_joint is a variable of two_joint");
    state
}

/// `has_relative_joint_space_jump`: a NaN waypoint poisons the *average*
/// (`total` sums every increment, and any addend NaN makes a sum NaN), so
/// `threshold` itself goes NaN and every `increment > threshold` comparison
/// in the whole path reads false -- not just the increment touching the NaN
/// waypoint. Waypoint 3 is a large, perfectly ordinary motion that a
/// working relative threshold must catch; before the fix, waypoint 1's NaN
/// silently defeated the check for waypoint 3 too.
#[test]
fn relative_mode_does_not_let_a_nan_waypoint_hide_a_later_ordinary_jump() {
    let model = two_joint_model();
    let group = model.joint_model_group("chain").expect("chain exists");
    let mut path = vec![
        state_at(&model, 0.0, 0.0),
        state_at(&model, 0.01, 0.0),
        state_at(&model, f64::NAN, 0.0),
        state_at(&model, f64::NAN, 0.0), // increment to here from index 2 is 0.0, still NaN-derived
        state_at(&model, 3.0, 0.0),      // an ordinary, large motion
    ];

    let kept = check_joint_space_jump(&mut path, group, &JumpThreshold::relative(2.0));

    assert!(
        kept.value() < 1.0,
        "a NaN-poisoned average must not silently clear the path's own later jump; got kept \
         fraction {}",
        kept.value()
    );
}

/// `has_absolute_joint_space_jump`, revolute branch: `distance` (not the
/// threshold) is NaN here, from a single NaN joint value on one waypoint.
#[test]
fn absolute_mode_flags_a_nan_revolute_distance_as_a_jump() {
    let model = two_joint_model();
    let group = model.joint_model_group("chain").expect("chain exists");
    let mut path = vec![
        state_at(&model, 0.0, 0.0),
        state_at(&model, f64::NAN, 0.0),
        state_at(&model, 0.0, 0.0),
    ];

    let kept = check_joint_space_jump(&mut path, group, &JumpThreshold::absolute(0.5, 0.5));

    assert_eq!(
        kept.value(),
        1.0 / 3.0,
        "a NaN revolute distance is unverifiable and must be treated as a jump at waypoint 1, \
         truncating the path to its first waypoint alone"
    );
}

/// Same as the revolute case, but for `JointType::Prismatic` -- a distinct
/// match arm in `has_absolute_joint_space_jump`, not covered by the
/// revolute test above.
#[test]
fn absolute_mode_flags_a_nan_prismatic_distance_as_a_jump() {
    let model = two_joint_model();
    let group = model.joint_model_group("chain").expect("chain exists");
    let mut path = vec![
        state_at(&model, 0.0, 0.0),
        state_at(&model, 0.0, f64::NAN),
        state_at(&model, 0.0, 0.0),
    ];

    let kept = check_joint_space_jump(&mut path, group, &JumpThreshold::absolute(0.5, 0.5));

    assert_eq!(
        kept.value(),
        1.0 / 3.0,
        "a NaN prismatic distance is unverifiable and must be treated as a jump at waypoint 1"
    );
}

/// The fix must not turn every path into "a jump somewhere": ordinary,
/// entirely finite motion under both thresholds must still behave exactly
/// as before -- no jump when nothing exceeds the threshold, a jump exactly
/// where a large finite motion actually is.
#[test]
fn ordinary_finite_thresholds_are_unaffected_by_the_nan_fix() {
    let model = two_joint_model();
    let group = model.joint_model_group("chain").expect("chain exists");

    let mut steady = vec![
        state_at(&model, 0.0, 0.0),
        state_at(&model, 0.1, 0.0),
        state_at(&model, 0.2, 0.0),
        state_at(&model, 0.3, 0.0),
    ];
    let kept = check_joint_space_jump(&mut steady, group, &JumpThreshold::absolute(0.5, 0.5));
    assert_eq!(kept.value(), 1.0, "no motion here exceeds 0.5 rad");

    let mut jumpy = vec![
        state_at(&model, 0.0, 0.0),
        state_at(&model, 0.1, 0.0),
        state_at(&model, 2.0, 0.0), // 1.9 rad step, exceeds the 0.5 rad revolute threshold
    ];
    let kept = check_joint_space_jump(&mut jumpy, group, &JumpThreshold::absolute(0.5, 0.5));
    assert_eq!(
        kept.value(),
        2.0 / 3.0,
        "the 1.9 rad step from waypoint 1 to waypoint 2 must still be caught"
    );
}
