// Copyright (c) 2011-2012, Georgia Tech Research Corporation
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp (class LinearPathSegment)
//
// Upstream defines `LinearPathSegment` only in the `.cpp` file (not the
// header): it is `Path::create`'s implementation detail, never named by a
// caller outside this crate's `path` module.

use nalgebra::DVector;

use crate::numeric::{cxx_max, cxx_min};

/// A straight segment between two configurations.
///
/// Upstream `LinearPathSegment`. The segment's `length_` (upstream's base
/// `PathSegment::length_`, set via `PathSegment((end - start).norm())`) is
/// computed once by [`Linear::new`] and handed back to the caller, which
/// stores it on the owning [`crate::path_segment::PathSegment`] rather than
/// here — see that type's doc comment for why.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Linear {
    start: DVector<f64>,
    end: DVector<f64>,
}

impl Linear {
    /// `LinearPathSegment(start, end)`. Returns the segment and its length.
    pub(crate) fn new(start: DVector<f64>, end: DVector<f64>) -> (Self, f64) {
        let length = (&end - &start).norm();
        (Self { start, end }, length)
    }

    /// `getConfig`.
    ///
    /// # Deviation from upstream (a non-deviation, see [`crate::numeric`])
    ///
    /// The clamp is `std::max(0.0, std::min(1.0, s))`, not [`f64::clamp`]:
    /// when `length` is `0.0` (a duplicate-waypoint or zero-length path,
    /// see the boundary tests on [`crate::Path::create`]), `s / length` is
    /// NaN and upstream's clamp deterministically resolves it to `1.0`
    /// (returning `end` unchanged) via the asymmetric NaN handling
    /// `cxx_min`/`cxx_max` reproduce. [`f64::clamp`] panics on a NaN
    /// bound and [`f64::min`]/[`f64::max`] would resolve differently.
    pub(crate) fn config(&self, s: f64, length: f64) -> DVector<f64> {
        let s = cxx_max(0.0, cxx_min(1.0, s / length));
        (1.0 - s) * &self.start + s * &self.end
    }

    /// `getTangent`. Constant along the segment; `s` is unused, matching
    /// upstream's unused `s` parameter.
    pub(crate) fn tangent(&self, length: f64) -> DVector<f64> {
        (&self.end - &self.start) / length
    }

    /// `getCurvature`. A straight segment has none.
    pub(crate) fn curvature(&self) -> DVector<f64> {
        DVector::zeros(self.start.len())
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn config_interpolates_linearly() {
        let (segment, length) = Linear::new(
            DVector::from_vec(vec![0.0, 0.0]),
            DVector::from_vec(vec![2.0, 4.0]),
        );
        let mid = segment.config(0.5 * length, length);
        assert_relative_eq!(mid[0], 1.0, epsilon = 1e-12);
        assert_relative_eq!(mid[1], 2.0, epsilon = 1e-12);
    }

    #[test]
    fn config_clamps_outside_the_segment() {
        let (segment, length) =
            Linear::new(DVector::from_vec(vec![0.0]), DVector::from_vec(vec![1.0]));
        assert_relative_eq!(segment.config(-1.0, length)[0], 0.0);
        assert_relative_eq!(segment.config(length + 1.0, length)[0], 1.0);
    }

    #[test]
    fn zero_length_segment_config_resolves_to_end_instead_of_nan() {
        let (segment, length) =
            Linear::new(DVector::from_vec(vec![3.0]), DVector::from_vec(vec![3.0]));
        assert_eq!(length, 0.0);
        assert_relative_eq!(segment.config(0.0, length)[0], 3.0);
    }

    #[test]
    fn tangent_is_the_unit_direction_scaled_by_inverse_length() {
        let (segment, length) = Linear::new(
            DVector::from_vec(vec![0.0, 0.0]),
            DVector::from_vec(vec![3.0, 4.0]),
        );
        let tangent = segment.tangent(length);
        assert_relative_eq!(tangent.norm(), 1.0, epsilon = 1e-12);
    }

    #[test]
    fn curvature_is_always_zero() {
        let (segment, _length) = Linear::new(
            DVector::from_vec(vec![0.0, 0.0]),
            DVector::from_vec(vec![1.0, 1.0]),
        );
        assert_eq!(segment.curvature(), DVector::from_vec(vec![0.0, 0.0]));
    }
}
