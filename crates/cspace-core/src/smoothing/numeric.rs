// Copyright (c) 2024, PickNik Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/online_signal_smoothing/src/acceleration_filter.cpp

//! `std::clamp`-compatible comparison.
//!
//! # Deviation from upstream (a non-deviation, spelled out)
//!
//! `std::clamp(v, lo, hi)` is specified as "if `v` compares less than `lo`,
//! returns `lo`; otherwise if `hi` compares less than `v`, returns `hi`;
//! otherwise returns `v`" — a literal three-way comparison, not a
//! `min`/`max` composition. `f64::clamp` instead `assert!(min <= max)`s and
//! **panics** whenever either bound is NaN (a NaN comparison against
//! anything is false either way, so `min <= max` can never hold), where
//! `std::clamp` has no such assertion at all and simply falls through to
//! `v` when a NaN `lo`/`hi` makes both its comparisons false.
//!
//! `jointLimitAccelerationScalingFactor`'s
//! `std::clamp(target_accel, variable_bound.min_acceleration_,
//! variable_bound.max_acceleration_)` therefore silently ignores a NaN
//! `min_acceleration_`/`max_acceleration_` bound (falls through to
//! `target_accel`, clamped by whichever bound is finite) rather than
//! crashing the process the way `f64::clamp` would. This is the same
//! panic-on-NaN-bound shape as `merge_constraints`'
//! [`f64::clamp`]-vs-`cxx_max`/`cxx_min` fix in `cspace-constraints`, but
//! is a genuinely different function to port: `std::max(lo,
//! std::min(v, hi))` (that fix's upstream idiom) and `std::clamp(v, lo,
//! hi)` (this one) disagree on a NaN `lo` — the composed form propagates
//! it (`std::max`'s first-argument rule), while `std::clamp` discards it.
//! [`cxx_clamp`] transcribes `std::clamp`'s three-way form directly rather
//! than composing `cxx_min`/`cxx_max`, precisely so it does not silently
//! inherit the other function's different NaN contract.
//!
//! A NaN `min_acceleration`/`max_acceleration` bound is reachable here
//! since [`crate::smoothing::acceleration_filter::joint_acceleration_bounds`] reads
//! it straight from [`crate::model::joint::VariableBounds`]'s public
//! fields with no validation.

/// `std::clamp(v, lo, hi)`: "if `v` compares less than `lo`, returns `lo`;
/// otherwise if `hi` compares less than `v`, returns `hi`; otherwise
/// returns `v`" — not a `cxx_min`/`cxx_max` composition, see the module
/// doc.
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
    fn nan_v_is_returned_unchanged() {
        assert!(cxx_clamp(f64::NAN, 0.0, 1.0).is_nan());
    }

    #[test]
    fn nan_lo_is_discarded_falling_through_to_the_hi_comparison() {
        assert_eq!(cxx_clamp(0.5, f64::NAN, 1.0), 0.5);
        assert_eq!(cxx_clamp(5.0, f64::NAN, 1.0), 1.0);
    }

    #[test]
    fn nan_hi_is_discarded_falling_through_to_the_lo_comparison() {
        assert_eq!(cxx_clamp(0.5, 0.0, f64::NAN), 0.5);
        assert_eq!(cxx_clamp(-5.0, 0.0, f64::NAN), 0.0);
    }

    /// The `f64::clamp` spelling this replaces, pinned so the divergence
    /// this module exists for cannot be mistaken for an equivalence:
    /// `f64::clamp` panics on a NaN bound; `cxx_clamp` does not.
    #[test]
    #[should_panic]
    fn the_f64_spelling_panics_on_a_nan_bound() {
        let _ = 0.5_f64.clamp(f64::NAN, 1.0);
    }

    #[test]
    fn ordinary_values_clamp_normally() {
        assert_eq!(cxx_clamp(0.5, 0.0, 1.0), 0.5);
        assert_eq!(cxx_clamp(-1.0, 0.0, 1.0), 0.0);
        assert_eq!(cxx_clamp(2.0, 0.0, 1.0), 1.0);
    }
}
