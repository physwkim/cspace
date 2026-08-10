// Copyright (c) 2011, Georgia Tech Research Corporation
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/trajectory_processing/src/time_optimal_trajectory_generation.cpp (class CircularPathSegment)
//
// Upstream defines `CircularPathSegment` only in the `.cpp` file (not the
// header), same as `LinearPathSegment` — see that module's doc comment.

use std::f64::consts::PI;

use nalgebra::DVector;

use crate::trajectory::numeric::cxx_min;

/// Eigen's `MatrixBase::normalized()` (read locally via
/// `/home/stevek/work/ITK/Modules/ThirdParty/Eigen3/src/itkeigen/Eigen/src/Core/Dot.h:92-102`
/// — Eigen is not vendored under this workspace's `third_party/`): divide
/// by the norm unless the squared norm is exactly `0.0`, in which case the
/// vector is returned unchanged. "Exactly `0.0`" includes floating-point
/// underflow of the squaring step, not just an algebraically zero input —
/// see [`Circular::new`]'s `x` computation for a concrete case where a
/// nonzero vector's squared norm still underflows to `0.0`. Unlike
/// `nalgebra`'s own `.normalize()`, this cannot produce NaN/Inf: a
/// zero (or underflowed-to-zero) squared norm takes the `else` branch
/// instead of dividing by it.
fn eigen_normalized(v: DVector<f64>) -> DVector<f64> {
    let squared_norm: f64 = v.iter().map(|c| c * c).sum();
    if squared_norm > 0.0 {
        v / squared_norm.sqrt()
    } else {
        v
    }
}

/// A circular blend at a waypoint, joining the straight segments before and
/// after it so the whole [`crate::trajectory::Path`] is differentiable there.
///
/// Upstream `CircularPathSegment`. Constructed from the midpoint of the
/// incoming segment, the waypoint itself (`intersection`), the midpoint of
/// the outgoing segment, and `max_deviation` (how far the blend is allowed
/// to bow away from `intersection`).
///
/// # Degenerate inputs — ported exactly, not guarded
///
/// [`Circular::new`] never fails; upstream's constructor has no failure
/// return either. Two conditions collapse it to a **zero-length** point
/// segment sitting at `intersection`:
///
/// - either adjacent midpoint coincides with `intersection` (consecutive
///   waypoints closer together than a `0.000001` threshold, including exact
///   duplicates) — a bare literal, not the crate's `EPS` (`1e-6`); the two
///   happen to share a value but upstream never ties them together, so this
///   port keeps them as separate literals too;
/// - the incoming and outgoing directions are within `~0.081°` of exactly
///   parallel (`dot > 0.999999`, e.g. three collinear waypoints in the same
///   direction) or exactly anti-parallel (`dot < -0.999999`) — this is
///   `CircularPathSegment`'s own degeneracy check, distinct from
///   [`crate::trajectory::Path::create`]'s separate, coarser 180°-turn rejection
///   (`cos_angle <= -1.0 + 1e-5`) that runs on the *unblended* waypoints
///   before a blend is ever constructed.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct Circular {
    radius: f64,
    center: DVector<f64>,
    x: DVector<f64>,
    y: DVector<f64>,
}

impl Circular {
    /// `CircularPathSegment(start, intersection, end, max_deviation)`.
    /// Returns the segment and its length.
    pub(crate) fn new(
        start: &DVector<f64>,
        intersection: &DVector<f64>,
        end: &DVector<f64>,
        max_deviation: f64,
    ) -> (Self, f64) {
        let degenerate = |center: DVector<f64>| {
            let dim = start.len();
            Self {
                radius: 1.0,
                center,
                x: DVector::zeros(dim),
                y: DVector::zeros(dim),
            }
        };

        if (intersection - start).norm() < 0.000_001 || (end - intersection).norm() < 0.000_001 {
            return (degenerate(intersection.clone()), 0.0);
        }

        // `eigen_normalized` for fidelity with upstream's `.normalized()`
        // (Eigen-based), not because either can independently diverge here:
        // the `< 0.000_001` guard just above bounds both receivers' norms
        // away from zero (>= 1e-6), identically in upstream and this port,
        // so the zero/underflow branch is unreachable at either call.
        let start_direction = eigen_normalized(intersection - start);
        let end_direction = eigen_normalized(end - intersection);
        let start_dot_end = start_direction.dot(&end_direction);

        // Catch division by 0 in the computations below: near-parallel
        // (0 deg.) or near-anti-parallel (180 deg.) directions. Written as
        // two separate comparisons (rather than a `RangeInclusive::contains`
        // check clippy would otherwise suggest) because a `contains` check
        // treats a NaN `start_dot_end` as "not in range" and would flip this
        // branch; two independent `>`/`<` comparisons instead leave NaN
        // resolving to "not degenerate" here, exactly as upstream's literal
        // `>`/`<` comparisons do.
        let too_parallel = start_dot_end > 0.999_999;
        let too_antiparallel = start_dot_end < -0.999_999;
        if too_parallel || too_antiparallel {
            return (degenerate(intersection.clone()), 0.0);
        }

        let angle = start_dot_end.acos();
        let start_distance = (start - intersection).norm();
        let end_distance = (end - intersection).norm();

        let distance = cxx_min(
            cxx_min(start_distance, end_distance),
            max_deviation * (0.5 * angle).sin() / (1.0 - (0.5 * angle).cos()),
        );

        let radius = distance / (0.5 * angle).tan();
        let length = angle * radius;

        // `eigen_normalized`, fidelity only: `too_parallel`/`too_antiparallel`
        // above bound `start_dot_end` to (-0.999_999, 0.999_999), which
        // bounds `end_direction - start_direction`'s norm away from zero by
        // a fixed algebraic floor (>= ~0.0014 for unit vectors at that
        // bound) independent of `max_deviation` — not independently
        // reachable.
        let center = intersection
            + eigen_normalized(&end_direction - &start_direction) * (radius / (0.5 * angle).cos());
        // `eigen_normalized`, LIVE: confirmed by probe (since deleted, see
        // `x_pre_underflows_to_a_finite_vector_not_nan_inf` below) that a
        // strictly-positive `max_deviation` far below any realistic bound —
        // still legal per `Path::create`'s only guard, `max_deviation > 0.0`
        // — drives `distance`/`radius` into subnormal range, and `x`'s
        // pre-normalize components land small enough that *squaring* them
        // for the norm underflows to exactly `0.0` even though the vector
        // itself is nonzero. `nalgebra`'s plain `.normalize()` then divides
        // by that exact-zero norm, producing `[NaN, -inf]`; Eigen's
        // `squaredNorm() > 0` guard sees the same underflow and returns the
        // (tiny but finite) vector unchanged instead.
        let x = eigen_normalized(intersection - distance * &start_direction - &center);
        let y = start_direction;

        (
            Self {
                radius,
                center,
                x,
                y,
            },
            length,
        )
    }

    /// `getConfig`.
    pub(crate) fn config(&self, s: f64) -> DVector<f64> {
        let angle = s / self.radius;
        &self.center + self.radius * (angle.cos() * &self.x + angle.sin() * &self.y)
    }

    /// `getTangent`.
    pub(crate) fn tangent(&self, s: f64) -> DVector<f64> {
        let angle = s / self.radius;
        -angle.sin() * &self.x + angle.cos() * &self.y
    }

    /// `getCurvature`.
    pub(crate) fn curvature(&self, s: f64) -> DVector<f64> {
        let angle = s / self.radius;
        (-1.0 / self.radius) * (angle.cos() * &self.x + angle.sin() * &self.y)
    }

    /// `getSwitchingPoints`. `length` is the segment's own length (stored
    /// by the owning [`crate::trajectory::path_segment::PathSegment`], not here — see
    /// that type's doc comment), used to drop candidates past the end of
    /// the (possibly zero-length, degenerate) segment.
    pub(crate) fn switching_points(&self, length: f64) -> Vec<f64> {
        let mut points: Vec<f64> = (0..self.x.len())
            .filter_map(|i| {
                let mut switching_angle = self.y[i].atan2(self.x[i]);
                if switching_angle < 0.0 {
                    switching_angle += PI;
                }
                let switching_point = switching_angle * self.radius;
                (switching_point < length).then_some(switching_point)
            })
            .collect();
        points.sort_by(|a, b| a.partial_cmp(b).expect("switching points are never NaN"));
        points
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn quarter_circle_blend_has_the_expected_length_and_config() {
        // A 90 deg. corner blended with enough room that the deviation cap
        // never binds: incoming (-1,0)->(0,0), outgoing (0,0)->(0,1).
        let start = DVector::from_vec(vec![-1.0, 0.0]);
        let intersection = DVector::from_vec(vec![0.0, 0.0]);
        let end = DVector::from_vec(vec![0.0, 1.0]);
        let (segment, length) = Circular::new(&start, &intersection, &end, 10.0);
        // radius = distance / tan(pi/4) = distance (angle between the
        // direction vectors is pi/2, since they're perpendicular).
        //
        // # Tolerance
        //
        // Measured per assertion via a temporary `eprintln!` diff sweep
        // before converting from `assert_relative_eq!`
        // (PORTING-PLAN.md §78.1/§79): `length` and `stop[1]` are
        // bit-exact (`0e0`), converted to `assert_eq!`. `begin[0]` has a
        // genuine 1-ULP floor (`2.220446049250313e-16`, exactly
        // `f64::EPSILON`) from `x`'s own `normalize()` not landing
        // `begin[0]` on `-segment.radius` bit-for-bit; kept as
        // `assert_relative_eq!` with `max_relative` pinned explicitly to
        // `f64::EPSILON` (making the previously-implicit default
        // explicit without loosening it) -- safe here since the compared
        // magnitude is ~1.0, far below where `epsilon = 1e-9` would ever
        // need the relative branch's help.
        assert_eq!(length, segment.radius * (PI / 2.0));
        let begin = segment.config(0.0);
        let stop = segment.config(length);
        assert_relative_eq!(
            begin[0],
            -segment.radius,
            epsilon = 1e-9,
            max_relative = f64::EPSILON
        );
        assert_eq!(stop[1], segment.radius);
    }

    #[test]
    fn max_deviation_caps_the_radius() {
        let start = DVector::from_vec(vec![-10.0, 0.0]);
        let intersection = DVector::from_vec(vec![0.0, 0.0]);
        let end = DVector::from_vec(vec![0.0, 10.0]);
        let (segment, _length) = Circular::new(&start, &intersection, &end, 0.1);
        assert!(segment.radius < 1.0);
    }

    #[test]
    fn collinear_same_direction_waypoints_collapse_to_a_point() {
        // start_direction == end_direction (dot ~ 1): CircularPathSegment's
        // own degeneracy check, not Path::create's 180-degree rejection.
        let start = DVector::from_vec(vec![0.0, 0.0]);
        let intersection = DVector::from_vec(vec![1.0, 0.0]);
        let end = DVector::from_vec(vec![2.0, 0.0]);
        let (segment, length) = Circular::new(&start, &intersection, &end, 0.1);
        assert_eq!(length, 0.0);
        assert_eq!(segment.config(0.0), intersection);
    }

    #[test]
    fn duplicate_midpoint_collapses_to_a_point() {
        let start = DVector::from_vec(vec![1.0, 0.0]);
        let intersection = DVector::from_vec(vec![1.0, 0.0]);
        let end = DVector::from_vec(vec![2.0, 0.0]);
        let (segment, length) = Circular::new(&start, &intersection, &end, 0.1);
        assert_eq!(length, 0.0);
        assert_eq!(segment.config(0.0), intersection);
    }

    #[test]
    fn switching_points_are_sorted_and_within_the_segment() {
        let start = DVector::from_vec(vec![-1.0, 0.0]);
        let intersection = DVector::from_vec(vec![0.0, 0.0]);
        let end = DVector::from_vec(vec![0.0, 1.0]);
        let (segment, length) = Circular::new(&start, &intersection, &end, 10.0);
        let points = segment.switching_points(length);
        assert!(points.iter().all(|&p| p < length));
        assert!(points.windows(2).all(|w| w[0] <= w[1]));
    }

    // A strictly-positive `max_deviation` (as `Path::create`'s only guard
    // requires, `max_deviation > 0.0`) far below any realistic bound still
    // drives `distance`/`radius` into subnormal range. At this exact
    // fixture, `x`'s pre-normalize vector comes out to `(0.0, -1e-323)` --
    // nonzero, but small enough that *squaring* it for the norm underflows
    // to exactly `0.0`. Before `eigen_normalized`, `x` came out `[NaN,
    // -inf]` (confirmed by a since-deleted probe: nalgebra's `.normalize()`
    // divides by that exact-zero norm unconditionally); traced by hand in
    // Python against the same literals, the pre-normalize `x` is finite in
    // both components, so Eigen's `squaredNorm() > 0` guard -- which sees
    // the identical underflow -- takes its `else` branch and returns that
    // finite (if tiny) vector unchanged. This regression test is the fixed
    // behavior, not the bug: nothing here is NaN or infinite.
    #[test]
    fn a_pathologically_tiny_positive_max_deviation_does_not_produce_nan_or_inf_in_x() {
        let start = DVector::from_vec(vec![-1.0, 0.0]);
        let intersection = DVector::from_vec(vec![0.0, 0.0]);
        let end = DVector::from_vec(vec![0.0, 1.0]);
        let max_deviation = 5e-324; // smallest positive subnormal f64, still > 0.0
        assert!(max_deviation > 0.0);

        let (segment, _length) = Circular::new(&start, &intersection, &end, max_deviation);
        assert!(
            segment.x.iter().all(|v| v.is_finite()),
            "x should stay finite even when its pre-normalize squared norm underflows to 0.0 \
             (Eigen's squaredNorm() > 0 guard returns the vector unchanged instead of dividing \
             by the underflowed zero); got {:?}",
            segment.x.as_slice()
        );
    }
}
