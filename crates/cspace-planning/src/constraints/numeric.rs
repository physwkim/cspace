// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/kinematic_constraints/src/utils.cpp (mergeConstraints)
//   moveit_core/constraint_samplers/src/default_constraint_samplers.cpp
//   moveit_core/constraint_samplers/include/moveit/constraint_samplers/default_constraint_samplers.hpp
//   (JointInfo::potentiallyAdjustMinMaxBounds)

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
//! The two therefore disagree exactly when the first argument is NaN, which
//! is the argument every site in this crate puts a *computed* value in.
//! `mergeConstraints`' `std::max(a.position - a.tolerance_below, ...)` keeps
//! a NaN window as NaN; the `f64::max` spelling silently substituted the
//! other constraint's finite window and returned a plausible-looking merge
//! with the NaN gone.
//!
//! [`f64::clamp`] is a third spelling of the same comparison —
//! `mergeConstraints` writes it out as `std::max(low, std::min(x, high))` —
//! and it is worse than merely divergent: it asserts `min <= max` and so
//! **panics** when either bound is NaN, where upstream has no assertion at
//! all. Use [`cxx_max`]/[`cxx_min`] rather than `clamp` wherever the bounds
//! are computed rather than known-finite.
//!
//! This is the second copy of these two functions in the workspace
//! (`cspace_core::trajectory`'s `numeric` module is the first). They are
//! duplicated rather than shared because the two crates have no common
//! dependency below `cspace_core::error`, which is an error-type crate; a third
//! copy should become a shared crate instead.

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
