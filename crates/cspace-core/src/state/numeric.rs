// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/src/planar_joint_model.cpp

//! `std::min`/`std::max`-compatible comparisons.
//!
//! # Deviation from upstream (a non-deviation, spelled out)
//!
//! `std::min(a, b)` is specified as `b < a ? b : a` and `std::max(a, b)` as
//! `a < b ? b : a`. Both are asymmetric in NaN: a NaN **first** argument
//! makes the comparison false and so is *returned*; a NaN **second**
//! argument also makes the comparison false and so is *discarded* in favor
//! of the first. [`f64::min`]/[`f64::max`] instead follow IEEE 754
//! `minNum`/`maxNum` — NaN is discarded wherever it sits, and propagates
//! only when both arguments are NaN.
//!
//! The two disagree exactly when the first argument is NaN.
//! `getVariableRandomPositionsNearBy`'s `uniformReal(std::max(min, near -
//! distance), std::min(max, near + distance))` keeps a NaN `min`/`max`
//! bound as NaN; the `f64::max`/`f64::min` spelling would silently
//! substitute the (finite) `near ± distance` operand instead. Confirmed as
//! the `std::min`/`std::max` family and not `std::fmin`/`std::fmax` (which
//! *is* what `f64::min`/`f64::max` implement) by reading
//! `planar_joint_model.cpp`'s call sites directly.
//!
//! `PlanarJointModel::getVariableRandomPositionsNearBy`'s bare
//! `if (da > M_PI) da = M_PI;` (a different call site, in the same file)
//! keeps a NaN `da` as NaN too (the comparison is false, so the assignment
//! never runs); `da.min(PI)` would silently substitute `PI` instead. This
//! is a plain conditional, not a library call, so that divergence is
//! inherent to the code as written, not a choice between `std::min` and
//! `std::fmin`.
//!
//! This is the fourth copy of these two functions in the workspace
//! (`cspace-trajectory`, `cspace-constraints` each have both;
//! `cspace-model` only needed `cxx_min`). Duplicated rather than shared
//! for the same reason as those: no common dependency below
//! `cspace-error` to hang a shared crate off, and a new workspace member
//! is out of scope this round.

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

    /// The `f64` spellings these two replace, pinned so the divergence this
    /// module exists for cannot be mistaken for an equivalence.
    #[test]
    fn the_f64_spellings_disagree_on_a_nan_first_argument() {
        assert_eq!(f64::NAN.min(1.0), 1.0);
        assert_eq!(f64::NAN.max(1.0), 1.0);
    }

    #[test]
    fn ordinary_values_match_normal_min_max() {
        assert_eq!(cxx_min(1.0, 2.0), 1.0);
        assert_eq!(cxx_min(2.0, 1.0), 1.0);
        assert_eq!(cxx_max(1.0, 2.0), 2.0);
        assert_eq!(cxx_max(2.0, 1.0), 2.0);
    }
}
