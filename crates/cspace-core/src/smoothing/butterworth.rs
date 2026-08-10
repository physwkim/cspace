// Copyright (c) 2021, PickNik Inc.
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/online_signal_smoothing/include/moveit/online_signal_smoothing/butterworth_filter.hpp
//   moveit_core/online_signal_smoothing/src/butterworth_filter.cpp
//
// `ButterworthFilterPlugin` (the `SmoothingBaseClass`/pluginlib wrapper
// around this filter) is not ported — see this crate's `lib.rs`.

use crate::error::{Error, Result};

/// `feedback_term_` magnitudes below this make the filter's feedback path
/// numerically indistinguishable from zero, and [`ButterworthFilter::new`]
/// rejects the coefficient that produced it. Matches upstream's
/// anonymous-namespace `EPSILON` in `butterworth_filter.cpp` — a value local
/// to that file, unrelated to (and 1000x tighter than) any other epsilon in
/// this workspace, so it is kept as its own constant rather than reused from
/// elsewhere. Public (unlike upstream's anonymous-namespace constant)
/// because it is part of [`ButterworthFilter::new`]'s documented contract: a
/// caller choosing `low_pass_filter_coeff` needs to know how close to `1.0`
/// is too close.
pub const EPSILON: f64 = 1e-9;

/// A first-order Butterworth low-pass filter (upstream `ButterworthFilter`).
///
/// Will not overshoot, by construction — see "Digital Implementation of
/// Butterworth First-Order Filter Type IIR" (Horvath, Cervenanska &
/// Kotianova, 2019) and Mienkina, "Filter-Based Algorithm for Metering
/// Applications" (NXP AN4265, 2016). Filters one scalar signal; upstream's
/// `ButterworthFilterPlugin` runs one instance per joint over a
/// `std::vector<ButterworthFilter>`, which is not ported here (see the crate
/// doc comment) — a caller filtering several signals constructs one
/// [`ButterworthFilter`] per signal itself.
///
/// Upstream's `FILTER_LENGTH` constant plus a `static_assert(FILTER_LENGTH
/// == 2)` guards that `previous_measurements_` has exactly the two slots the
/// arithmetic in [`filter`](Self::filter) depends on; here that guarantee is
/// structural (the field is `[f64; 2]`, not a runtime-sized buffer), so no
/// assertion is needed.
#[derive(Debug, Clone, PartialEq)]
pub struct ButterworthFilter {
    previous_measurements: [f64; 2],
    previous_filtered_measurement: f64,
    scale_term: f64,
    feedback_term: f64,
}

impl ButterworthFilter {
    /// `ButterworthFilter(double low_pass_filter_coeff)`.
    ///
    /// `low_pass_filter_coeff` is `2*pi / tan(omega_d * T)`, where `omega_d`
    /// is the cutoff frequency and `T` is the sampling period in seconds.
    /// Larger values smooth more but add more lag.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Construct`] in the same four cases, checked in the
    /// same order, that upstream's constructor throws `std::length_error`:
    /// a non-finite feedback term, a non-finite scale term, a coefficient
    /// `< 1.0` (which makes the filter unstable), or a feedback term within
    /// [`EPSILON`] of zero. Upstream throws from the constructor; this port
    /// returns `Result` instead, matching this workspace's `cspace_core::error`
    /// convention (see that crate's "Deviation from upstream" note).
    pub fn new(low_pass_filter_coeff: f64) -> Result<Self> {
        let scale_term = 1.0 / (1.0 + low_pass_filter_coeff);
        let feedback_term = 1.0 - low_pass_filter_coeff;

        if feedback_term.is_infinite() {
            return Err(Error::construct(
                "online_signal_smoothing::ButterworthFilter: infinite feedback_term_",
            ));
        }
        if scale_term.is_infinite() {
            return Err(Error::construct(
                "online_signal_smoothing::ButterworthFilter: infinite scale_term_",
            ));
        }
        if low_pass_filter_coeff < 1.0 {
            return Err(Error::construct(
                "online_signal_smoothing::ButterworthFilter: Filter coefficient < 1. makes the lowpass filter unstable",
            ));
        }
        if feedback_term.abs() < EPSILON {
            return Err(Error::construct(
                "online_signal_smoothing::ButterworthFilter: Filter coefficient value resulted in feedback term of 0",
            ));
        }

        Ok(Self {
            previous_measurements: [0.0, 0.0],
            previous_filtered_measurement: 0.0,
            scale_term,
            feedback_term,
        })
    }

    /// `filter(double new_measurement)`.
    pub fn filter(&mut self, new_measurement: f64) -> f64 {
        self.previous_measurements[1] = self.previous_measurements[0];
        self.previous_measurements[0] = new_measurement;

        self.previous_filtered_measurement = self.scale_term
            * (self.previous_measurements[1] + self.previous_measurements[0]
                - self.feedback_term * self.previous_filtered_measurement);

        self.previous_filtered_measurement
    }

    /// `reset(const double data)`.
    pub fn reset(&mut self, data: f64) {
        self.previous_measurements = [data, data];
        self.previous_filtered_measurement = data;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Upstream `TEST(SMOOTHING_PLUGINS, FilterConverge)`.
    #[test]
    fn upstream_test_filter_converge() {
        let mut lpf = ButterworthFilter::new(2.0).unwrap();
        assert_eq!(0.0, lpf.filter(0.0));
        let mut value = 0.0;
        for _ in 0..100 {
            value = lpf.filter(5.0);
        }
        assert_eq!(5.0, value);
        assert_ne!(5.0, lpf.filter(100.0));
    }

    /// Upstream `TEST(SMOOTHING_PLUGINS, FilterReset)`.
    #[test]
    fn upstream_test_filter_reset() {
        let mut lpf = ButterworthFilter::new(2.0).unwrap();
        assert_eq!(0.0, lpf.filter(0.0));
        lpf.reset(5.0);
        let value = lpf.filter(5.0);
        assert_eq!(5.0, value);
        assert_ne!(5.0, lpf.filter(100.0));
    }

    #[test]
    fn coefficient_below_one_is_rejected() {
        // Boundary: upstream's `coeff < 1` check. `matches!` alone cannot
        // tell this apart from `new`'s other three `Error::Construct`
        // sites (infinite feedback_term_/scale_term_, feedback_term_ ~ 0);
        // message-swap bite-checked against each of them.
        let err = ButterworthFilter::new(0.999_999).unwrap_err();
        assert!(err.to_string().contains("unstable"), "{err}");
    }

    #[test]
    fn coefficient_of_negative_one_makes_scale_term_infinite() {
        // scale_term_ = 1 / (1 + coeff); coeff == -1 divides by zero.
        // feedback_term_ = 1 - (-1) = 2 is finite, so this exercises the
        // second upstream check (isinf(scale_term_)), not the first.
        let err = ButterworthFilter::new(-1.0).unwrap_err();
        assert!(err.to_string().contains("scale_term_"), "{err}");
    }

    #[test]
    fn coefficient_of_exactly_one_makes_feedback_term_zero() {
        // feedback_term_ = 1 - coeff; coeff == 1 lands exactly on 0, which
        // the EPSILON = 1e-9 check rejects even though `coeff < 1` alone
        // does not (1.0 is not < 1.0) — the two checks guard different
        // boundaries of the same value.
        let err = ButterworthFilter::new(1.0).unwrap_err();
        assert!(
            err.to_string().contains("resulted in feedback term of 0"),
            "{err}"
        );
    }

    #[test]
    fn coefficient_just_above_one_is_still_within_the_feedback_term_epsilon_band() {
        // coeff = 1 + 1e-10: feedback_term_ = -1e-10, magnitude still under
        // EPSILON = 1e-9 even though coeff clears the `< 1` check.
        let err = ButterworthFilter::new(1.0 + 1e-10).unwrap_err();
        assert!(
            err.to_string().contains("resulted in feedback term of 0"),
            "{err}"
        );
    }

    #[test]
    fn coefficient_far_enough_above_one_is_accepted() {
        // coeff = 1 + 1e-8: feedback_term_ = -1e-8, magnitude outside the
        // EPSILON = 1e-9 band.
        assert!(ButterworthFilter::new(1.0 + 1e-8).is_ok());
    }

    #[test]
    fn coefficient_of_infinity_makes_feedback_term_infinite() {
        // feedback_term_ = 1 - inf = -inf: the very first upstream check.
        let err = ButterworthFilter::new(f64::INFINITY).unwrap_err();
        assert!(err.to_string().contains("feedback_term_"), "{err}");
    }

    /// Not an invariant this port enforces — a direct transcription check.
    /// Upstream's checks are `isinf(...)`/`coeff < 1`/`abs(...) < EPSILON`,
    /// none of which reject NaN (`isinf(NaN)` is false in C++, and every
    /// `<` comparison against NaN is false), so a NaN coefficient
    /// constructs successfully with NaN-valued internal state upstream. This
    /// port keeps that behaviour rather than adding a check upstream does
    /// not have.
    #[test]
    fn coefficient_of_nan_is_accepted_like_upstream() {
        let filter = ButterworthFilter::new(f64::NAN).unwrap();
        assert!(filter.scale_term.is_nan());
        assert!(filter.feedback_term.is_nan());
    }

    #[test]
    fn reset_then_filter_same_value_holds_steady() {
        let mut lpf = ButterworthFilter::new(4.0).unwrap();
        lpf.reset(-3.0);
        // Bit-exact for this input: measured `0e0` via a temporary
        // `eprintln!` diff sweep before converting from
        // `assert_relative_eq!` per PORTING-PLAN.md §78.1/§79. Not exact
        // by a general argument (`scale_term = 0.2` isn't exactly
        // representable) -- confirmed by measurement, not derivation.
        assert_eq!(lpf.filter(-3.0), -3.0);
    }
}
