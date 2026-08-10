// Copyright (c) 2011-2012, Georgia Tech Research Corporation
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/trajectory_processing/include/moveit/trajectory_processing/time_optimal_trajectory_generation.hpp (class Path)
//   moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp (Path::create and friends)

use nalgebra::DVector;

use crate::error::{Error, Result};

use crate::trajectory::path_segment::PathSegment;

/// The intermediate waypoints of an input path need to be blended so the
/// whole path is differentiable. This is the maximum deviation tolerated at
/// those waypoints, in radians for revolute joints or metres for prismatic
/// ones.
///
/// Upstream `DEFAULT_PATH_TOLERANCE`.
pub const DEFAULT_PATH_TOLERANCE: f64 = 0.1;

/// A piecewise linear/circular path through configuration space, blended at
/// intermediate waypoints so it is differentiable everywhere. Build one
/// with [`Path::create`].
///
/// Upstream `trajectory_processing::Path`. `path_segments_` is a
/// `std::list<std::unique_ptr<PathSegment>>` there (needed because
/// `PathSegment` is a polymorphic base); here it is `Vec<PathSegment>`,
/// since `PathSegment` (crate-private — see the [`crate::trajectory::path_segment`]
/// module doc comment) is a closed sum type with no indirection to own.
/// Because of that, this port also drops
/// upstream's hand-written copy constructor (`Path::Path(const Path&)`,
/// which deep-clones every segment via virtual `clone()`): [`Clone`] on a
/// plain `Vec<PathSegment>` already does the equivalent deep copy.
#[derive(Debug, Clone, PartialEq)]
pub struct Path {
    length: f64,
    switching_points: Vec<(f64, bool)>,
    segments: Vec<PathSegment>,
}

impl Path {
    /// Build a path from `waypoints`, blending intermediate waypoints with
    /// circular arcs so the path is differentiable, deviating from the
    /// original waypoint by at most `max_deviation`.
    ///
    /// Upstream `Path::create`, which returns `std::optional<Path>`;
    /// failure is reported here as `Err`, carrying the message upstream
    /// only sent to `RCLCPP_ERROR`, rather than as a bare `None`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when: fewer than 2 waypoints are given;
    /// `max_deviation` is not finite and positive; or the path would
    /// require an (unsupported)
    /// 180° turn at some waypoint — the last case checked exactly as
    /// upstream checks it, on the *un-blended* waypoints, before any blend
    /// segment is built (see [`crate::trajectory::path_segment`]'s `Circular` for the
    /// separate, later-firing degeneracy check a blend's own construction
    /// can hit).
    pub fn create(waypoints: &[DVector<f64>], max_deviation: f64) -> Result<Self> {
        if waypoints.len() < 2 {
            return Err(Error::construct("a path needs at least 2 waypoints"));
        }
        if max_deviation.is_nan() || max_deviation <= 0.0 {
            return Err(Error::construct(
                "path max_deviation must be greater than 0.0",
            ));
        }
        // Upstream re-checks `max_deviation > 0.0` on every loop iteration
        // below (`Path::create`'s `if (max_deviation > 0.0 &&
        // waypoints_iterator3 != waypoints.end())`), which is redundant
        // with the guard above for any finite value but is upstream's only
        // defence against a NaN `max_deviation`: NaN slips the `<= 0.0`
        // check above the same way it slips `> 0.0` there, so upstream
        // falls back to linear-only segments for the rest of the loop
        // instead of building blends from NaN. Rejecting NaN explicitly
        // above makes that per-iteration re-check unnecessary here.

        // waypoints[i1], waypoints[i2], waypoints[i3] are three consecutive
        // waypoints of the input path: a LinearPathSegment starting at i1,
        // connected to a CircularPathSegment at i2, connected to another
        // LinearPathSegment towards i3. Applied iteratively, this blends
        // `max_deviation` at every intermediate waypoint.
        let mut segments: Vec<PathSegment> = Vec::new();
        let mut start_config = waypoints[0].clone();
        let mut i1 = 0usize;
        let mut i2 = 1usize;
        while i2 < waypoints.len() {
            let i3 = i2 + 1;
            if i3 < waypoints.len() {
                // Reject a path that requires a 180 deg. turn, which this
                // implementation does not support.
                let incoming = &waypoints[i2] - &waypoints[i1];
                let outgoing = &waypoints[i3] - &waypoints[i2];
                let incoming_norm = incoming.norm();
                let outgoing_norm = outgoing.norm();
                if incoming_norm > f64::EPSILON && outgoing_norm > f64::EPSILON {
                    let cos_angle = incoming.dot(&outgoing) / (incoming_norm * outgoing_norm);
                    const ANGLE_TOLERANCE: f64 = 1e-5;
                    if cos_angle <= -1.0 + ANGLE_TOLERANCE {
                        return Err(Error::construct(
                            "the path requires a 180 deg. turn, which is not supported by the current implementation",
                        ));
                    }
                }

                let mid_in = 0.5 * (&waypoints[i1] + &waypoints[i2]);
                let mid_out = 0.5 * (&waypoints[i2] + &waypoints[i3]);
                let blend = PathSegment::circular(&mid_in, &waypoints[i2], &mid_out, max_deviation);
                let end_config = blend.config(0.0);
                let next_start = blend.config(blend.length());
                if (&end_config - &start_config).norm() > 0.000_001 {
                    segments.push(PathSegment::linear(start_config, end_config));
                }
                segments.push(blend);
                start_config = next_start;
            } else {
                segments.push(PathSegment::linear(start_config, waypoints[i2].clone()));
                start_config = waypoints[i2].clone();
            }
            i1 = i2;
            i2 += 1;
        }

        // Assign each segment's absolute position, collect switching-point
        // candidates, and total the path length.
        let mut length = 0.0;
        let mut switching_points: Vec<(f64, bool)> = Vec::new();
        for segment in &mut segments {
            segment.set_position(length);
            for point in segment.switching_points() {
                switching_points.push((length + point, false));
            }
            length += segment.length();
            while switching_points
                .last()
                .is_some_and(|&(pos, _)| pos >= length)
            {
                switching_points.pop();
            }
            switching_points.push((length, true));
        }
        // The last entry pushed above always marks the end of the path
        // itself, not a real discontinuity; upstream drops it the same way.
        switching_points.pop();

        Ok(Self {
            length,
            switching_points,
            segments,
        })
    }

    /// `getLength`.
    pub fn length(&self) -> f64 {
        self.length
    }

    /// Find the segment containing arc length `s`, and `s` translated to
    /// that segment's local coordinate.
    ///
    /// Upstream `getPathSegment`, which takes `s` by mutable reference and
    /// returns the segment pointer; the two are combined into one return
    /// value here since neither escapes its caller.
    fn segment_at(&self, s: f64) -> (&PathSegment, f64) {
        let mut idx = 0;
        while idx + 1 < self.segments.len() && s >= self.segments[idx + 1].position() {
            idx += 1;
        }
        (&self.segments[idx], s - self.segments[idx].position())
    }

    /// `getConfig`.
    pub fn config(&self, s: f64) -> DVector<f64> {
        let (segment, local_s) = self.segment_at(s);
        segment.config(local_s)
    }

    /// `getTangent`.
    pub fn tangent(&self, s: f64) -> DVector<f64> {
        let (segment, local_s) = self.segment_at(s);
        segment.tangent(local_s)
    }

    /// `getCurvature`.
    pub fn curvature(&self, s: f64) -> DVector<f64> {
        let (segment, local_s) = self.segment_at(s);
        segment.curvature(local_s)
    }

    /// The next switching point at or after arc length `s`, and whether it
    /// is a discontinuity (a segment boundary) rather than a smooth
    /// curvature-driven candidate. Arc length equal to
    /// [`Path::length`] with `discontinuity = true` means "no more
    /// switching points before the end of the path".
    ///
    /// Upstream `getNextSwitchingPoint`, which returns the arc length and
    /// takes `discontinuity` as an out-parameter.
    pub(crate) fn next_switching_point(&self, s: f64) -> (f64, bool) {
        match self.switching_points.iter().find(|&&(pos, _)| pos > s) {
            Some(&(pos, discontinuity)) => (pos, discontinuity),
            None => (self.length, true),
        }
    }

    /// `getSwitchingPoints`.
    pub(crate) fn switching_points(&self) -> &[(f64, bool)] {
        &self.switching_points
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn v(values: &[f64]) -> DVector<f64> {
        DVector::from_vec(values.to_vec())
    }

    #[test]
    fn fewer_than_two_waypoints_is_rejected() {
        // `Path::create` has 3 `Error::construct` sites (waypoint count,
        // max_deviation, 180-degree turn); a bare `.is_err()` cannot say
        // which one fired (assertion-discrimination-round2.md sec. 3).
        assert!(
            Path::create(&[v(&[0.0])], DEFAULT_PATH_TOLERANCE)
                .unwrap_err()
                .to_string()
                .contains("at least 2 waypoints")
        );
        assert!(
            Path::create(&[], DEFAULT_PATH_TOLERANCE)
                .unwrap_err()
                .to_string()
                .contains("at least 2 waypoints")
        );
    }

    #[test]
    fn zero_max_deviation_is_rejected() {
        // See `fewer_than_two_waypoints_is_rejected` for why this checks
        // the message rather than just `.is_err()`.
        let waypoints = [v(&[0.0, 0.0]), v(&[1.0, 0.0])];
        assert!(
            Path::create(&waypoints, 0.0)
                .unwrap_err()
                .to_string()
                .contains("max_deviation must be greater than 0.0")
        );
        assert!(
            Path::create(&waypoints, -1.0)
                .unwrap_err()
                .to_string()
                .contains("max_deviation must be greater than 0.0")
        );
    }

    #[test]
    fn nan_max_deviation_is_rejected() {
        // `max_deviation <= 0.0` alone does not catch NaN (every
        // comparison against NaN is false), so before the guard above also
        // checked `is_nan()`, this returned `Ok(_)` -- a plain linear path
        // -- instead of the `Err` every other invalid `max_deviation`
        // produces. See `fewer_than_two_waypoints_is_rejected` for why this
        // checks the message rather than just `.is_err()`.
        let waypoints = [v(&[0.0, 0.0]), v(&[1.0, 0.0])];
        assert!(
            Path::create(&waypoints, f64::NAN)
                .unwrap_err()
                .to_string()
                .contains("max_deviation must be greater than 0.0")
        );
    }

    #[test]
    fn two_waypoints_build_a_single_linear_segment_no_blend() {
        let waypoints = [v(&[0.0, 0.0]), v(&[3.0, 4.0])];
        let path = Path::create(&waypoints, DEFAULT_PATH_TOLERANCE).unwrap();
        // Bit-exact: 3^2 + 4^2 = 25.0 is exactly representable and IEEE 754
        // sqrt is correctly rounded, so `path.length()` and the endpoint
        // configs (plain interpolation against the 0.0/1.0 arc-length
        // extremes) land exactly on the literals below -- measured via a
        // temporary `eprintln!` of `(actual - expected).abs()`, all `0e0`,
        // before converting from `assert_relative_eq!` per PORTING-PLAN.md
        // §78.1/§79.
        assert_eq!(path.length(), 5.0);
        assert_eq!((path.config(0.0) - &waypoints[0]).norm(), 0.0);
        assert_eq!((path.config(path.length()) - &waypoints[1]).norm(), 0.0);
    }

    #[test]
    fn three_collinear_waypoints_blend_to_a_straight_path() {
        let waypoints = [v(&[0.0, 0.0]), v(&[1.0, 0.0]), v(&[2.0, 0.0])];
        let path = Path::create(&waypoints, DEFAULT_PATH_TOLERANCE).unwrap();
        // Bit-exact for the same reason as
        // `two_waypoints_build_a_single_linear_segment_no_blend` above: the
        // blend at (1,0) collapses to `Circular::new`'s degenerate,
        // exact-intersection case (dot product of the two collinear
        // direction vectors is exactly 1.0, past the `> 0.999_999`
        // threshold), so `path.length()` is the exact sum `1.0 + 0.0 + 1.0`
        // and `mid[1]` reads straight off a literal `0.0` input with no
        // intervening arithmetic. Measured `0e0` before converting.
        assert_eq!(path.length(), 2.0);
        let mid = path.config(1.0);
        assert_eq!(mid[1], 0.0);
    }

    #[test]
    fn duplicate_consecutive_waypoints_do_not_panic() {
        let waypoints = [
            v(&[0.0, 0.0]),
            v(&[1.0, 0.0]),
            v(&[1.0, 0.0]),
            v(&[2.0, 1.0]),
        ];
        let path = Path::create(&waypoints, DEFAULT_PATH_TOLERANCE).unwrap();
        assert!(path.length() > 0.0);
        assert!(path.config(0.0).iter().all(|x| x.is_finite()));
        assert!(path.config(path.length()).iter().all(|x| x.is_finite()));
    }

    #[test]
    fn a_path_of_two_identical_waypoints_has_zero_length() {
        let p = v(&[5.0]);
        let waypoints = [p.clone(), p.clone()];
        let path = Path::create(&waypoints, DEFAULT_PATH_TOLERANCE).unwrap();
        assert_eq!(path.length(), 0.0);
        // Bit-exact: two identical waypoints produce a single degenerate
        // linear segment whose `config` returns the start point verbatim
        // with no arithmetic on it. Measured `0e0` before converting.
        assert_eq!((path.config(0.0) - &p).norm(), 0.0);
    }

    #[test]
    fn a_180_degree_turn_is_rejected() {
        let waypoints = [
            v(&[0.0, 0.0, 0.0]),
            v(&[1.0, 0.0, 0.0]),
            v(&[0.0, 0.0, 0.0]),
        ];
        // See `fewer_than_two_waypoints_is_rejected` for why this checks
        // the message rather than just `.is_err()`.
        assert!(
            Path::create(&waypoints, 0.01)
                .unwrap_err()
                .to_string()
                .contains("180 deg")
        );
    }

    #[test]
    fn switching_points_are_within_the_path_and_non_decreasing() {
        // Non-decreasing, not strictly increasing: a circular blend's own
        // switching point can land exactly at its segment's start (e.g. a
        // dimension whose tangent component is already extremal at s = 0),
        // which coincides with the segment-boundary marker the previous
        // loop iteration of `Path::create` already pushed at that same
        // cumulative length — upstream's own `while (...back().first >=
        // path.length_) pop_back()` cleanup does not remove that tie (it
        // only trims points beyond the *new* cumulative length), so this is
        // inherent upstream behaviour, not an artifact of this port.
        let waypoints = [
            v(&[0.0, 0.0]),
            v(&[1.0, 0.0]),
            v(&[1.0, 1.0]),
            v(&[0.0, 1.0]),
        ];
        let path = Path::create(&waypoints, 0.1).unwrap();
        let points = path.switching_points();
        for w in points.windows(2) {
            assert!(w[0].0 <= w[1].0);
        }
        for &(pos, _) in points {
            assert!(pos > 0.0 && pos < path.length());
        }
    }

    #[test]
    fn next_switching_point_reports_end_of_path_past_the_last_one() {
        let waypoints = [v(&[0.0, 0.0]), v(&[1.0, 0.0]), v(&[1.0, 1.0])];
        let path = Path::create(&waypoints, 0.1).unwrap();
        let (pos, discontinuity) = path.next_switching_point(path.length());
        assert_eq!(pos, path.length());
        assert!(discontinuity);
    }
}
