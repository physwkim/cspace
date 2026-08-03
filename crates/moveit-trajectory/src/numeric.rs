// Copyright (c) 2011-2012, Georgia Tech Research Corporation
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp

//! `std::min`/`std::max`-compatible comparisons.
//!
//! # Deviation from upstream (a non-deviation, spelled out)
//!
//! Every `std::min`/`std::max` call this crate ports must resolve NaN the
//! way libstdc++ does, not the way [`f64::min`]/[`f64::max`] do — the two
//! disagree, and the disagreement is load-bearing for at least one boundary
//! case ported here (a zero-length path; see `Trajectory::min_max_path_acceleration`).
//!
//! `std::min(a, b)` is specified as `b < a ? b : a`; `std::max(a, b)` as
//! `a < b ? b : a`. Both are asymmetric in NaN: a NaN **first** argument
//! makes the comparison false and so is *returned*; a NaN **second**
//! argument also makes the comparison false and so is *discarded* in favor
//! of the first argument. Concretely, `std::min(DBL_MAX, NaN) == DBL_MAX`
//! but `std::min(NaN, DBL_MAX) == NaN`.
//!
//! [`f64::min`]/[`f64::max`] instead follow IEEE 754 `minNum`/`maxNum`:
//! NaN is discarded regardless of position, and only propagates when both
//! arguments are NaN. Using them here would silently change which
//! accumulator "wins" whenever a degenerate path segment (zero length, zero
//! tangent) feeds a NaN into a running `std::min` — for example
//! `getMinMaxPathAcceleration`'s `std::min(max_path_acceleration, ...)`
//! accumulator starts at `f64::MAX` and stays there once the RHS goes NaN
//! under upstream's rule, keeping the caller on a well-defined (if
//! permissive) code path instead of propagating NaN through the rest of
//! the trajectory.
//!
//! [`cxx_min`] and [`cxx_max`] reproduce the exact comparison upstream uses
//! at every call site this crate ports (`LinearPathSegment::getConfig`,
//! `CircularPathSegment`'s constructor, `Trajectory::getNext­AccelerationSwitchingPoint`,
//! `Trajectory::integrateBackward`'s intersection test, and every
//! `getMinMax*`/`get*MaxPathVelocity` helper).

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
