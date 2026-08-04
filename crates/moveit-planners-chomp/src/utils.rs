// Copyright (c) 2009, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/chomp/chomp_motion_planner/include/chomp_motion_planner/chomp_utils.hpp

//! Free functions and constants from `chomp_utils.hpp`.
//!
//! # Deviation from upstream: angle normalization is CHOMP's own copy
//!
//! `normalize_angle_positive`/`normalize_angle`/`shortest_angular_distance`
//! below are transcribed fresh from `chomp_utils.hpp`'s own bodies (marked
//! there `// copied from geometry/angles/angles.h`), not reused from any
//! other moveit-rs crate's angle-normalization helper. Upstream itself
//! keeps this as a separate local copy rather than sharing one
//! implementation across packages (the ROS `angles` package is a distinct
//! dependency `chomp_motion_planner` does not take), and other crates in
//! this port (`moveit-model::joint::planar`, `moveit-planners-sbp::so2`,
//! `moveit-constraints::joint`) each already keep their own separate
//! `normalize_angle`, matching that upstream non-sharing convention. The
//! two formulas are algebraically equivalent at the +-pi boundary (both map
//! `angle == +-PI` to `+PI`) but are textually different implementations,
//! so this is a fresh transcription, not a reuse.
use moveit_error::Result;
use moveit_state::RobotState;

/// The number of terms in a [`DIFF_RULES`] finite-difference stencil.
///
/// Ported from `chomp::DIFF_RULE_LENGTH`. Upstream declares this `const
/// int`; here it is `usize` since it also sizes the [`DIFF_RULES`] array,
/// which Rust requires to be `usize` — the value (7) is unchanged.
pub const DIFF_RULE_LENGTH: usize = 7;

/// Centered finite-difference coefficients for velocity (row 0),
/// acceleration (row 1) and jerk (row 2), each over a window of
/// [`DIFF_RULE_LENGTH`] samples centered on the point being differentiated.
///
/// Ported from `chomp::DIFF_RULES`. Transcribed as upstream's exact literal
/// fractions, not recomputed or reduced to a simpler form. One exception:
/// upstream's velocity row writes its center coefficient as `6 / 6.0`; that
/// is `1.0` bit-for-bit (dividing two equal nonzero finite `f64`s is exact,
/// no rounding), so it is written as `1.0` directly rather than `6.0 / 6.0`,
/// which `clippy::eq_op` rejects as suspicious.
pub const DIFF_RULES: [[f64; DIFF_RULE_LENGTH]; 3] = [
    // velocity
    [0.0, 0.0, -2.0 / 6.0, -3.0 / 6.0, 1.0, -1.0 / 6.0, 0.0],
    // acceleration
    [
        0.0,
        -1.0 / 12.0,
        16.0 / 12.0,
        -30.0 / 12.0,
        16.0 / 12.0,
        -1.0 / 12.0,
        0.0,
    ],
    // jerk
    [
        0.0,
        1.0 / 12.0,
        -17.0 / 12.0,
        46.0 / 12.0,
        -46.0 / 12.0,
        17.0 / 12.0,
        -1.0 / 12.0,
    ],
];

/// Writes `state`'s active-joint positions for `planning_group_name` into
/// `joint_array`, one scalar per active joint, in group-active-joint order.
///
/// Ported from `chomp::robotStateToArray`. Like upstream, this reads only
/// each joint's *first* variable (upstream: `getFirstVariableIndex()`), so
/// it silently ignores any variable beyond the first on a multi-DOF active
/// joint rather than rejecting the group — matching upstream, which has no
/// guard at this call site either (unlike
/// [`crate::trajectory::ChompTrajectory`]'s constructors, which do reject a
/// multi-DOF active joint; that check lives there because it is upstream's
/// own `assert()`-guarded invariant at that specific call site, not a
/// blanket rule this function also enforces).
///
/// Upstream dereferences `state.getJointModelGroup(planning_group_name)`
/// without a null check, a latent null-pointer dereference on an unknown
/// group name; here the lookup is a typed error instead
/// (`moveit_error::Error::UnknownName`, via `RobotModel::joint_model_group`).
pub fn robot_state_to_array(
    state: &RobotState,
    planning_group_name: &str,
    joint_array: &mut [f64],
) -> Result<()> {
    let group = state.model().joint_model_group(planning_group_name)?;
    for (joint_index, &model_index) in group.active_joint_indices().iter().enumerate() {
        let joint = state.model().joint_model_at(model_index);
        let values = state.joint_position(joint.name())?;
        joint_array[joint_index] = values[0];
    }
    Ok(())
}

/// Normalizes `angle` to `[0, 2*PI)`.
///
/// Ported from `chomp::normalizeAnglePositive`.
pub fn normalize_angle_positive(angle: f64) -> f64 {
    (angle % (2.0 * std::f64::consts::PI) + 2.0 * std::f64::consts::PI)
        % (2.0 * std::f64::consts::PI)
}

/// Normalizes `angle` to `(-PI, PI]`.
///
/// Ported from `chomp::normalizeAngle`.
pub fn normalize_angle(angle: f64) -> f64 {
    let a = normalize_angle_positive(angle);
    if a > std::f64::consts::PI {
        a - 2.0 * std::f64::consts::PI
    } else {
        a
    }
}

/// The signed shortest angular distance from `start` to `end`.
///
/// Ported from `chomp::shortestAngularDistance`.
pub fn shortest_angular_distance(start: f64, end: f64) -> f64 {
    let mut res =
        normalize_angle_positive(normalize_angle_positive(end) - normalize_angle_positive(start));
    if res > std::f64::consts::PI {
        res = -(2.0 * std::f64::consts::PI - res);
    }
    normalize_angle(res)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use moveit_error::Error;
    use moveit_model::MeshSearchPaths;
    use moveit_model::RobotModel;
    use moveit_srdf::SrdfModel;
    use std::f64::consts::PI;

    const EPS: f64 = 1e-12;

    /// A single-revolute-joint chain, just enough to exercise
    /// [`robot_state_to_array`]'s group lookup and joint-position read.
    fn one_joint_model() -> RobotModel {
        let urdf_xml = r#"<?xml version="1.0"?>
<robot name="one_joint_chomp_utils">
  <link name="base"/>
  <link name="tip"/>
  <joint name="j1" type="revolute">
    <parent link="base"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"#;
        let srdf_xml = r#"<?xml version="1.0"?>
<robot name="one_joint_chomp_utils">
  <group name="chain">
    <chain base_link="base" tip_link="tip"/>
  </group>
</robot>
"#;
        let urdf: urdf_rs::Robot = urdf_rs::read_from_string(urdf_xml).unwrap();
        let srdf = SrdfModel::parse_str(srdf_xml).expect("srdf must parse");
        RobotModel::from_urdf_and_srdf(&urdf, urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("one_joint_chomp_utils model must build")
    }

    #[test]
    fn robot_state_to_array_reads_the_active_joint_in_group_order() {
        let model = one_joint_model();
        let mut state = RobotState::new(&model);
        state.set_joint_positions("j1", &[0.4]).unwrap();
        let mut out = [0.0; 1];
        robot_state_to_array(&state, "chain", &mut out).unwrap();
        assert_relative_eq!(out[0], 0.4, epsilon = EPS, max_relative = EPS);
    }

    #[test]
    fn robot_state_to_array_rejects_an_unknown_group_name_as_a_typed_error() {
        let model = one_joint_model();
        let state = RobotState::new(&model);
        let mut out = [0.0; 1];
        let err = robot_state_to_array(&state, "no_such_group", &mut out).unwrap_err();
        assert!(matches!(err, Error::UnknownName { .. }));
    }

    #[test]
    fn diff_rules_rows_sum_to_zero() {
        // Each finite-difference stencil differentiates a polynomial and
        // must therefore annihilate a constant signal.
        for row in DIFF_RULES {
            let sum: f64 = row.iter().sum();
            assert_relative_eq!(sum, 0.0, epsilon = EPS, max_relative = EPS);
        }
    }

    #[test]
    fn diff_rule_length_matches_row_width() {
        for row in DIFF_RULES {
            assert_eq!(row.len(), DIFF_RULE_LENGTH);
        }
    }

    #[test]
    fn normalize_angle_positive_pi_boundary() {
        assert_relative_eq!(
            normalize_angle_positive(PI),
            PI,
            epsilon = EPS,
            max_relative = EPS
        );
        assert_relative_eq!(
            normalize_angle_positive(-PI),
            PI,
            epsilon = EPS,
            max_relative = EPS
        );
        assert_relative_eq!(
            normalize_angle_positive(0.0),
            0.0,
            epsilon = EPS,
            max_relative = EPS
        );
        assert_relative_eq!(
            normalize_angle_positive(2.0 * PI),
            0.0,
            epsilon = EPS,
            max_relative = EPS
        );
        assert_relative_eq!(
            normalize_angle_positive(-2.0 * PI),
            0.0,
            epsilon = EPS,
            max_relative = EPS
        );
    }

    #[test]
    fn normalize_angle_pi_boundary() {
        assert_relative_eq!(normalize_angle(PI), PI, epsilon = EPS, max_relative = EPS);
        assert_relative_eq!(normalize_angle(-PI), PI, epsilon = EPS, max_relative = EPS);
        assert_relative_eq!(
            normalize_angle(PI + 0.1),
            -PI + 0.1,
            epsilon = EPS,
            max_relative = EPS
        );
    }

    #[test]
    fn shortest_angular_distance_wraps_the_short_way() {
        assert_relative_eq!(
            shortest_angular_distance(-PI + 0.1, PI - 0.1),
            -0.2,
            epsilon = EPS,
            max_relative = EPS
        );
        assert_relative_eq!(
            shortest_angular_distance(0.0, PI / 2.0),
            PI / 2.0,
            epsilon = EPS,
            max_relative = EPS
        );
        assert_relative_eq!(
            shortest_angular_distance(0.0, 0.0),
            0.0,
            epsilon = EPS,
            max_relative = EPS
        );
    }
}
