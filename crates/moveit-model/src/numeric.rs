// Copyright (c) 2013, Ioan A. Sucan
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2008, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/robot_model/src/joint_model.cpp

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
//! The two disagree exactly when the first argument is NaN.
//! `getVariableBoundsMsg`'s `std::min(fabs(min_velocity_), fabs(max_velocity_))`
//! (and the acceleration/jerk siblings) keeps a NaN bound as NaN; the
//! `f64::min` spelling would silently substitute the other, finite bound
//! and return a plausible-looking limit with the NaN gone.
//!
//! This is not `std::fmin`/IEEE `minNum` (which *is* what [`f64::min`]
//! implements) — `joint_model.cpp` calls plain `std::min`, verified by
//! reading the call site directly rather than assumed from the name.
//!
//! This is the third copy of this function in the workspace
//! (`moveit-trajectory` and `moveit-constraints` each have one, alongside
//! `cxx_max`). It is duplicated rather than shared because the crates have
//! no common dependency below `moveit-error`, which is an error-type
//! crate; a shared crate is out of scope this round (see the workspace's
//! in-flight branches) but the growing duplicate count should be revisited.

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
