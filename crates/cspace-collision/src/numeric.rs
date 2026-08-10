// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection/src/collision_tools.cpp (intersectCostSources)
//   moveit_core/collision_detection/src/collision_octomap_filter.cpp (findSurface)

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
//! The two therefore disagree exactly when the first argument is NaN.
//! `intersectCostSources`'s `std::max(source_a.aabb_min[i],
//! source_b.aabb_min[i])` keeps a NaN `source_a` bound as NaN; `f64::max`
//! silently substitutes `source_b`'s finite bound instead.
//! `findSurface`'s `std::max(gs.dot(gs), epsilon)` keeps a NaN gradient
//! magnitude as NaN — deliberately, per the divergence documented on
//! [`crate::octomap_filter`]'s `sample_cloud` — where `f64::max` would
//! silently substitute `epsilon` and manufacture a finite step out of an
//! undefined gradient.
//!
//! This is the third copy of these two functions in the workspace
//! (`cspace_core::trajectory` and `cspace_planning::constraints` each have one). They are
//! duplicated rather than shared because none of the three crates has a
//! common dependency below `cspace_core::error`, an error-type crate; a fourth
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
