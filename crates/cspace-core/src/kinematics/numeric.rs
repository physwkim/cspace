// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp
//   moveit_core/robot_model/src/revolute_joint_model.cpp
//   moveit_core/robot_model/src/prismatic_joint_model.cpp

//! `std::min`/`std::max`-compatible comparisons.
//!
//! Same contract as `crate::trajectory::numeric`'s [`cxx_min`]/[`cxx_max`] —
//! see that module's doc comment for the full NaN-asymmetry argument
//! (`std::min`/`std::max`'s NaN handling depends on operand position;
//! [`f64::min`]/[`f64::max`] discard NaN regardless of position). Duplicated
//! here rather than shared: promoting either copy to a common crate is a
//! cross-crate API change out of scope for this pass.
//!
//! Two live sites in this crate needed the fix (both in `cart_to_jnt.rs`):
//!
//! - `cart_to_jnt`'s `delta_twist_norm = position_error_norm.max(orientation_error_norm)`,
//!   matching `kdl_kinematics_plugin.cpp:444`'s
//!   `std::max(position_error, orientation_error)` — `position_error_norm` is
//!   a runtime norm, not a constant, so a NaN there used to be silently
//!   discarded by `f64::max` instead of propagating.
//! - `near_by_configuration`'s non-continuous-joint draw,
//!   `min.max(near - limit)..=max.min(near + limit)`, matching
//!   `revolute_joint_model.cpp:133-134` and `prismatic_joint_model.cpp:95-96`
//!   (identical in both): `std::max(bounds.min_position_, near - distance)`,
//!   `std::min(bounds.max_position_, near + distance)` — the bound is a
//!   model-sourced runtime value, not a constant.
//!
//! `cart_to_jnt`'s other two `std::min`-family calls,
//! `(0.2_f64).min(last_delta_twist_norm / delta_twist_norm)` and
//! `(0.1_f64).min(delta_twist_norm)`, stay plain `.min()`: the literal always
//! occupies the first/receiver operand, where a NaN second operand is
//! discarded identically by `std::min` and `f64::min` — see those call
//! sites' own comments for why no observable divergence is reachable there.

/// `std::min(a, b)`: `if b < a { b } else { a }`.
pub(crate) fn cxx_min(a: f64, b: f64) -> f64 {
    if b < a { b } else { a }
}

/// `std::max(a, b)`: `if a < b { b } else { a }`.
pub(crate) fn cxx_max(a: f64, b: f64) -> f64 {
    if a < b { b } else { a }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nan_as_first_argument_is_returned_by_min() {
        assert!(cxx_min(f64::NAN, 1.0).is_nan());
    }

    #[test]
    fn nan_as_second_argument_is_discarded_by_min() {
        assert_eq!(cxx_min(f64::MAX, f64::NAN), f64::MAX);
    }

    #[test]
    fn nan_as_first_argument_is_returned_by_max() {
        assert!(cxx_max(f64::NAN, 1.0).is_nan());
    }

    #[test]
    fn nan_as_second_argument_is_discarded_by_max() {
        assert_eq!(cxx_max(f64::MIN, f64::NAN), f64::MIN);
    }

    #[test]
    fn ordinary_values_match_normal_min_max() {
        assert_eq!(cxx_min(1.0, 2.0), 1.0);
        assert_eq!(cxx_min(2.0, 1.0), 1.0);
        assert_eq!(cxx_max(1.0, 2.0), 2.0);
        assert_eq!(cxx_max(2.0, 1.0), 2.0);
    }
}
