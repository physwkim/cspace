// Copyright (c) 2018, Pilz GmbH & Co. KG
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/pilz_industrial_motion_planner/src/joint_limits_container.cpp
//   (JointLimitsContainer::updateCommonLimit)
// and, as the algebraic reformulation this file's own doc explains,
// orocos_kdl/src/path_line.cpp's `Path_Line` constructor's `pathlength`
// selection.

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
//! `updateCommonLimit`'s `std::min(common_limit.max_velocity, max_velocity)`
//! keeps a NaN `common_limit.max_velocity` as NaN, so a poisoned joint limit
//! stays visibly poisoned through every later fusion; the `f64::min`
//! spelling would silently substitute the next joint's finite limit and
//! return a plausible-looking (and wrong) common limit instead.
//!
//! `std::min`/`std::max` are not the only overload upstream uses, and the
//! other one does *not* diverge from [`f64::min`]/[`f64::max`]:
//! `std::fmin`/`std::fmax` (IEEE `minNum`/`maxNum`) discard NaN wherever it
//! sits, exactly like the `f64` methods, so a call written as
//! `std::fmin`/`std::fmax` upstream is *correctly* ported as plain
//! [`f64::min`]/[`f64::max`] and must not be converted to
//! [`cxx_min`]/[`cxx_max`]. Which family applies is a property of the exact
//! call site, decided by reading it, not inferred from "this is a min/max".
//!
//! [`cxx_min`]/[`cxx_max`] reproduce the exact comparison upstream's
//! `std::min`/`std::max` (never `std::fmin`/`std::fmax`) use at every call
//! site this crate ports: [`crate::limits::JointLimitsContainer`]'s
//! `update_common_limit` (`updateCommonLimit`'s five `std::min`/`std::max`
//! calls fusing position/velocity/acceleration/deceleration limits), and
//! [`crate::path_line::PathLine::new`]'s `path_length` selection — not a
//! transcription of a literal `std::max` call (see that function's own doc
//! comment for why), but proven by direct derivation to be the exact
//! `std::max(dist, angle * eqradius)` upstream's `if (angle * eqradius >
//! dist)` branch computes, NaN behavior included: the branch takes its
//! `else` arm (`pathlength = dist`) whenever the comparison is false, which
//! is every case a naive `std::max` would also return `dist` for, NaN or
//! not.
//!
//! [`crate::path_polyline_generator::compute_blend_radius`] also has a
//! `std::min(dist1 / 2.0, dist2 / 2.0)` call (not `std::fmin`, so the same
//! family as the two sites above), but it is deliberately left as
//! [`f64::min`] rather than converted — see that function's test module for
//! the reachability argument and the regression test proving it.
//!
//! This is the third copy of these two functions in the workspace
//! (`moveit-trajectory`'s and `moveit-constraints`' `numeric` modules are
//! the first two). They are duplicated rather than shared because none of
//! the three crates has a common dependency below `moveit-error`, an
//! error-type crate; a fourth copy should become a shared crate instead.

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
