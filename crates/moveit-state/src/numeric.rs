// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/src/planar_joint_model.cpp

//! `std::min`-compatible comparison.
//!
//! # Deviation from upstream (a non-deviation, spelled out)
//!
//! `std::min(a, b)` is specified as `b < a ? b : a`, asymmetric in NaN: a
//! NaN **first** argument makes the comparison false and so is *returned*;
//! a NaN **second** argument also makes the comparison false and so is
//! *discarded* in favor of the first. [`f64::min`] instead follows IEEE 754
//! `minNum` — NaN is discarded wherever it sits, and propagates only when
//! both arguments are NaN.
//!
//! `PlanarJointModel::getVariableRandomPositionsNearBy`'s bare
//! `if (da > M_PI) da = M_PI;` keeps a NaN `da` as NaN (the comparison is
//! false, so the assignment never runs); `da.min(PI)` would silently
//! substitute `PI` instead. This is a plain conditional, not a library
//! call, so the divergence is inherent to the code as written, not a
//! choice between `std::min` and `std::fmin`.
//!
//! This is the fourth copy of this function in the workspace
//! (`moveit-trajectory`, `moveit-constraints`, `moveit-model` each have
//! one, alongside `cxx_max` in the first two). Duplicated rather than
//! shared for the same reason as those: no common dependency below
//! `moveit-error` to hang a shared crate off, and a new workspace member
//! is out of scope this round.

/// `std::min(a, b)`: `if b < a { b } else { a }`.
pub(crate) fn cxx_min(a: f64, b: f64) -> f64 {
    if b < a { b } else { a }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nan_as_first_argument_is_returned() {
        assert!(cxx_min(f64::NAN, 1.0).is_nan());
    }

    #[test]
    fn nan_as_second_argument_is_discarded() {
        assert_eq!(cxx_min(f64::MAX, f64::NAN), f64::MAX);
    }

    /// The `f64::min` spelling this replaces, pinned so the divergence
    /// this module exists for cannot be mistaken for an equivalence.
    #[test]
    fn the_f64_spelling_disagrees_on_a_nan_first_argument() {
        assert_eq!(f64::NAN.min(1.0), 1.0);
    }

    #[test]
    fn ordinary_values_match_normal_min() {
        assert_eq!(cxx_min(1.0, 2.0), 1.0);
        assert_eq!(cxx_min(2.0, 1.0), 1.0);
    }
}
