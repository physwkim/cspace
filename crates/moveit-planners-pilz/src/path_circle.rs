// Copyright (c) 2018, Pilz GmbH & Co. KG
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/path_circle_generator.hpp
//   moveit_planners/pilz_industrial_motion_planner/src/path_circle_generator.cpp

//! Circle solving for `CIRC` motions ([`circle_from_center`],
//! [`circle_from_interim`]), ported from upstream's `PathCircleGenerator`.
//!
//! # Scope: analytical geometry only, `KDL::Path_Circle` deferred
//!
//! Upstream's `circleFromCenter`/`circleFromInterim` both end by constructing
//! a `KDL::Path_Circle` — the interpolated position+orientation path object
//! used by `trajectory_generator_circ`, which is not yet in this crate's
//! scope. This module ports only the analytical geometry that upstream
//! computes *before* that construction: the circle's center, radius and
//! sweep angle, plus the auxiliary point upstream passes to `Path_Circle` to
//! disambiguate its plane and direction. [`CircleGeometry`] carries exactly
//! that. When `trajectory_generator_circ` is ported, it is expected to
//! consume a [`CircleGeometry`] plus the start/goal orientations to build the
//! full interpolated path — see `KDL::Path_Circle`'s constructor
//! (`orocos_kdl/src/path_circle.cpp`, vendored under
//! `third_party/orocos_kinematics_dynamics/`) for how `radius`/center/aux-point
//! feed into the plane-normal and rotational-interpolation setup that stays
//! out of scope here.
//!
//! Consequences of that deferral for this round's API:
//!
//! - Inputs are plain positions ([`Vector3`]), not full `KDL::Frame`
//!   poses — upstream's own geometry computation never reads the
//!   orientation components either; they are only consumed inside
//!   `Path_Circle` for orientation SLERP.
//! - `eqradius` is not a parameter here. Upstream threads it straight through
//!   to `Path_Circle`, where it is the knob that balances linear vs.
//!   rotational interpolation speed; nothing in the geometry solve itself
//!   reads it.
//! - [`CircleGeometry::radius`] is `|start - center|` (what `Path_Circle`
//!   itself recomputes from its `F_base_start`/`F_base_center` at
//!   construction time), not `eqradius`.
//!
//! # Deviations from upstream
//!
//! - **Errors are [`moveit_error::Error::Construct`], not KDL exceptions.**
//!   Upstream throws `ErrorMotionPlanningCenterPointDifferentRadius` (a
//!   `KDL::Error_MotionPlanning` subclass, numeric code `3006`) from
//!   [`circle_from_center`] and `KDL::Error_MotionPlanning_Circle_No_Plane`
//!   from [`circle_from_interim`]; this port has no KDL exception hierarchy,
//!   so both map to `Error::Construct` with a message naming the upstream
//!   exception, matching this crate's house error convention.
//! - **The `KDL::epsilon` swap around `Path_Circle` construction is not
//!   ported.** Upstream's `circleFromCenter` temporarily overwrites the
//!   global `KDL::epsilon` with `MAX_COLINEAR_NORM` so `Path_Circle`'s own
//!   internal degeneracy check uses the same tolerance as this class; since
//!   `Path_Circle` itself is deferred, there is nothing to swap the epsilon
//!   for yet. `MAX_COLINEAR_NORM` is reused directly as
//!   [`MAX_COLINEAR_NORM`] wherever this module needs the same check.

use moveit_error::{Error, Result};
use moveit_geometry::Vector3;

/// Upstream `PathCircleGenerator::MAX_RADIUS_DIFF`: the largest tolerated
/// difference between the start-to-center and goal-to-center distances in
/// [`circle_from_center`].
pub const MAX_RADIUS_DIFF: f64 = 1e-2;

/// Upstream `PathCircleGenerator::MAX_COLINEAR_NORM`: the smallest triangle-
/// normal norm treated as "not colinear" in [`circle_from_interim`].
pub const MAX_COLINEAR_NORM: f64 = 1e-5;

/// The analytical solution of a circular arc from `start` to `goal`: center,
/// radius, sweep angle and the auxiliary point used to disambiguate the
/// circle's plane and direction of travel.
///
/// See the [module docs](self) for why `Path_Circle`'s full interpolated path
/// is not part of this type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CircleGeometry {
    /// The circle's center.
    pub center: Vector3,
    /// `|start - center|`.
    pub radius: f64,
    /// The sweep angle from `start` to `goal`, in radians, already corrected
    /// for the intended arc direction (see [`circle_from_interim`]).
    pub alpha: f64,
    /// The point used to pin down the circle's plane and travel direction,
    /// consumed by `Path_Circle` as its `V_base_p` parameter.
    pub aux_point: Vector3,
}

/// Solve a circular arc from `start` to `goal` given an explicit `center`.
///
/// Upstream: `PathCircleGenerator::circleFromCenter`. A half circle cannot be
/// expressed this way, because `start`/`goal`/`center` are colinear and the
/// sweep direction is ambiguous — upstream's own doc comment notes this;
/// callers needing a half circle must use [`circle_from_interim`].
///
/// # Errors
///
/// [`Error::Construct`] if `start` and `goal` are not equidistant from
/// `center` within [`MAX_RADIUS_DIFF`] (upstream:
/// `ErrorMotionPlanningCenterPointDifferentRadius`).
pub fn circle_from_center(
    start: Vector3,
    goal: Vector3,
    center: Vector3,
) -> Result<CircleGeometry> {
    let a = (start - center).norm();
    let b = (goal - center).norm();
    let c = (start - goal).norm();

    if (a - b).abs() > MAX_RADIUS_DIFF {
        return Err(Error::construct(
            "distances between start-center and goal-center are different; a circle cannot be created",
        ));
    }

    let alpha = cosines(a, b, c);

    Ok(CircleGeometry {
        center,
        radius: a,
        alpha,
        aux_point: goal,
    })
}

/// Solve a circular arc from `start` to `goal` passing through `interim`.
///
/// Upstream: `PathCircleGenerator::circleFromInterim`. Computes the circle's
/// center as the circumcenter of the (`start`, `interim`, `goal`) triangle,
/// then corrects the sweep angle and auxiliary point when `interim` lies on
/// the major arc (i.e. the angle at `interim` is acute, so the naive law-of-
/// cosines angle would describe the minor arc instead of the one actually
/// passing through `interim`).
///
/// # Errors
///
/// [`Error::Construct`] if `start`, `interim` and `goal` are colinear —
/// the triangle normal's norm falls below [`MAX_COLINEAR_NORM`] (upstream:
/// `KDL::Error_MotionPlanning_Circle_No_Plane`).
pub fn circle_from_interim(
    start: Vector3,
    goal: Vector3,
    interim: Vector3,
) -> Result<CircleGeometry> {
    let t = interim - start;
    let u = goal - start;
    let v = goal - interim;
    let w = t.cross(&u);

    if w.norm() < MAX_COLINEAR_NORM {
        return Err(Error::construct(
            "start, interim and goal points are colinear; a circle cannot be created",
        ));
    }

    let center = start
        + (u * (t.dot(&t) * u.dot(&v)) - t * (u.dot(&u) * t.dot(&v))) * 0.5 / w.norm_squared();

    let t_center = center - start;
    let v_center = goal - center;
    let a = t_center.norm();
    let b = v_center.norm();
    let c = u.norm();
    let mut alpha = cosines(a, b, c);

    let mut aux_point = interim;

    // If the angle at the interim point is acute, the intended arc is the
    // major arc (the naive law-of-cosines angle above describes the minor
    // arc between start and goal, which does not pass through interim).
    let interim_angle = cosines(t.norm(), v.norm(), u.norm());
    if interim_angle < std::f64::consts::FRAC_PI_2 {
        alpha = 2.0 * std::f64::consts::PI - alpha;

        // Only reflect interim through the center when goal is not itself
        // colinear with start and center -- otherwise the reflection would
        // be meaningless as a plane-disambiguating auxiliary point.
        if t_center.cross(&v_center).norm() > MAX_COLINEAR_NORM {
            aux_point = 2.0 * center - goal;
        }
    }

    Ok(CircleGeometry {
        center,
        radius: a,
        alpha,
        aux_point,
    })
}

/// Law of cosines: the angle gamma opposite side `c` in a triangle with
/// sides `a`, `b`, `c` (`c² = a² + b² - 2ab·cos(gamma)`), in radians.
///
/// Upstream: `PathCircleGenerator::cosines`. The argument to `acos` is
/// clamped to `[-1, 1]` to absorb floating-point overshoot at the `0`/`π`
/// boundaries.
fn cosines(a: f64, b: f64, c: f64) -> f64 {
    (((a * a + b * b - c * c) / (2.0 * a * b)).clamp(-1.0, 1.0)).acos()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::{FRAC_PI_2, PI};

    /// A quarter circle solved from its explicit center.
    #[test]
    fn circle_from_center_quarter_circle() {
        let geom = circle_from_center(
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
        )
        .unwrap();

        assert_relative_eq!(geom.center, Vector3::new(0.0, 0.0, 0.0));
        assert_relative_eq!(geom.radius, 1.0);
        assert_relative_eq!(geom.alpha, FRAC_PI_2);
        assert_relative_eq!(geom.aux_point, Vector3::new(0.0, 1.0, 0.0));
    }

    /// Boundary: start and goal at different distances from the given center
    /// is rejected.
    #[test]
    fn circle_from_center_radius_mismatch_is_rejected() {
        let result = circle_from_center(
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 2.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
        );
        assert!(result.is_err());
    }

    /// Boundary: a radius difference just under `MAX_RADIUS_DIFF` is
    /// accepted (kept off the exact tolerance value itself, since `norm()`'s
    /// `sqrt` can round either side of a literal boundary).
    #[test]
    fn circle_from_center_radius_diff_within_tolerance_is_accepted() {
        let result = circle_from_center(
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0 + 0.9 * MAX_RADIUS_DIFF, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
        );
        assert!(result.is_ok());
    }

    /// Circumcenter recovered exactly for three points already known to lie
    /// on the unit circle, with the interim point on the *minor* arc (angle
    /// at interim exactly `π/2`, the acute/non-acute boundary — the
    /// direction correction must NOT trigger here since upstream's check is
    /// strict `<`).
    #[test]
    fn circle_from_interim_half_circle_boundary_angle_not_corrected() {
        let start = Vector3::new(1.0, 0.0, 0.0);
        let interim = Vector3::new(0.0, 1.0, 0.0);
        let goal = Vector3::new(-1.0, 0.0, 0.0);

        let geom = circle_from_interim(start, goal, interim).unwrap();

        assert_relative_eq!(geom.center, Vector3::new(0.0, 0.0, 0.0), epsilon = 1e-12);
        assert_relative_eq!(geom.radius, 1.0, epsilon = 1e-12);
        assert_relative_eq!(geom.alpha, PI, epsilon = 1e-12);
        assert_relative_eq!(geom.aux_point, interim, epsilon = 1e-12);
    }

    /// Interim point on the major arc: the angle at interim is acute, so the
    /// sweep angle and auxiliary point must both be direction-corrected.
    #[test]
    fn circle_from_interim_major_arc_is_direction_corrected() {
        let start = Vector3::new(1.0, 0.0, 0.0);
        let interim = Vector3::new(-1.0, 0.0, 0.0);
        let goal = Vector3::new(0.0, 1.0, 0.0);

        let geom = circle_from_interim(start, goal, interim).unwrap();

        assert_relative_eq!(geom.center, Vector3::new(0.0, 0.0, 0.0), epsilon = 1e-12);
        assert_relative_eq!(geom.radius, 1.0, epsilon = 1e-12);
        assert_relative_eq!(geom.alpha, 1.5 * PI, epsilon = 1e-12);
        assert_relative_eq!(
            geom.aux_point,
            Vector3::new(0.0, -1.0, 0.0),
            epsilon = 1e-12
        );
    }

    /// Boundary: colinear start/interim/goal is rejected.
    #[test]
    fn circle_from_interim_colinear_points_is_rejected() {
        let result = circle_from_interim(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(2.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
        );
        assert!(result.is_err());
    }
}
