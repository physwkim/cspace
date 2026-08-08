// Copyright (c) 2011, Georgia Tech Research Corporation
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
//! [`cxx_min`] and [`cxx_max`] exist to reproduce the exact comparison
//! upstream's `std::min`/`std::max` calls use. A prior pass found seven call
//! sites still using plain [`f64::min`]/[`f64::max`] where upstream calls
//! `std::min`/`std::max`; all seven are now converted, but only two of the
//! four *conceptual* defects behind them were live — see each site's own
//! comment for the reachability argument, not just the conversion:
//!
//! - **Live**: `computeTimeStamps`'s `max_velocity[idx]`/`max_acceleration[idx]`
//!   assignments (both overloads, four call sites) — a corrupted (NaN)
//!   `max_velocity`/`max_acceleration` bound used to be silently replaced by
//!   `min_velocity`/`min_acceleration`'s magnitude instead of propagating;
//!   regression-tested in `time_optimal_trajectory_generation.rs`
//!   (`a_nan_max_velocity_bound_is_not_silently_replaced_by_min_velocity`,
//!   `a_nan_max_acceleration_bound_is_not_silently_replaced_by_min_acceleration`).
//! - **Fidelity/uniformity only, not independently reachable**:
//!   `Trajectory::integrateBackward`'s intersection test
//!   (`trajectory.rs`) and the resample loop's `t = std::min(duration, ...)`
//!   clamp (`time_optimal_trajectory_generation.rs`) — both converted to
//!   match upstream's exact comparison, but a NaN cannot observably
//!   discriminate `cxx_min`/`cxx_max` from `f64::min`/`f64::max` at either
//!   site (see each site's own comment for why), so neither has a
//!   fail-before/pass-after regression test.
//!
//! Sites that were already correct before this pass: `LinearPathSegment::getConfig`,
//! `CircularPathSegment`'s constructor, and the rest of
//! `Trajectory::getNextAccelerationSwitchingPoint`/`getMinMaxPathVelocity`.
//! Distinguish this from upstream's *other* min/max family before reaching
//! for these helpers: `std::fmin`/`std::fmax` are IEEE `minNum`/`maxNum` —
//! the same rule [`f64::min`]/[`f64::max`] already implement — so a call
//! this crate ports from `std::fmin`/`std::fmax` (as opposed to
//! `std::min`/`std::max`) is already correctly ported as a plain
//! `.min()`/`.max()`, and converting it to [`cxx_min`]/[`cxx_max`] would
//! itself be the divergence.
//!
//! [`cxx_clamp`] is the same idea applied to `std::clamp(v, lo, hi)`:
//! `v < lo ? lo : (hi < v ? hi : v)`, three plain `<` comparisons, none of
//! which trip on a NaN bound (a NaN comparison is just always false, so the
//! branch is skipped, not asserted). [`f64::clamp`] instead panics if
//! either bound is NaN. `ruckig_smoothing.rs` clamps six current/target
//! velocity and acceleration values against model-sourced
//! `max_velocity`/`max_acceleration` bounds (`ruckig_traj_smoothing.cpp`'s
//! `initializeRuckigState`/`getNextRuckigInput`, both `std::clamp`) — a
//! runtime bound, not a constant, so a NaN one used to panic instead of the
//! value passing through unclamped the way upstream's `std::clamp` does.

/// `std::min(a, b)`: `if b < a { b } else { a }`.
pub(crate) fn cxx_min(a: f64, b: f64) -> f64 {
    if b < a { b } else { a }
}

/// `std::max(a, b)`: `if a < b { b } else { a }`.
pub(crate) fn cxx_max(a: f64, b: f64) -> f64 {
    if a < b { b } else { a }
}

/// `std::clamp(v, lo, hi)`: `if v < lo { lo } else if hi < v { hi } else { v }`.
/// Unlike [`f64::clamp`], never panics on a NaN `lo`/`hi` — both comparisons
/// are simply false, so `v` passes through, matching upstream.
pub(crate) fn cxx_clamp(v: f64, lo: f64, hi: f64) -> f64 {
    if v < lo {
        lo
    } else if hi < v {
        hi
    } else {
        v
    }
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

    #[test]
    fn a_nan_bound_does_not_panic_and_the_value_passes_through() {
        assert_eq!(cxx_clamp(5.0, f64::NAN, 10.0), 5.0);
        assert_eq!(cxx_clamp(5.0, 0.0, f64::NAN), 5.0);
        assert_eq!(cxx_clamp(5.0, f64::NAN, f64::NAN), 5.0);
    }

    #[test]
    fn a_nan_value_returns_nan() {
        assert!(cxx_clamp(f64::NAN, 0.0, 10.0).is_nan());
    }

    #[test]
    fn ordinary_values_match_normal_clamp() {
        assert_eq!(cxx_clamp(-5.0, 0.0, 10.0), 0.0);
        assert_eq!(cxx_clamp(15.0, 0.0, 10.0), 10.0);
        assert_eq!(cxx_clamp(5.0, 0.0, 10.0), 5.0);
    }
}
