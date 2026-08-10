// Copyright (c) 2018, Pilz GmbH & Co. KG
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/path_circle_generator.hpp
//   moveit_planners/pilz_industrial_motion_planner/src/path_circle_generator.cpp

//! Circle solving for `CIRC` motions ([`circle_from_center`],
//! [`circle_from_interim`]), ported from upstream's `PathCircleGenerator`,
//! plus the interpolated position+orientation path itself ([`PathCircle`]).
//!
//! # Scope
//!
//! [`circle_from_center`]/[`circle_from_interim`] port the analytical
//! geometry upstream's `circleFromCenter`/`circleFromInterim` compute before
//! constructing a `KDL::Path_Circle`: the circle's center, radius and sweep
//! angle, plus the auxiliary point used to disambiguate the circle's plane
//! and direction. [`PathCircle`] then plays the role of that
//! `KDL::Path_Circle` — see its own doc for how it is derived and why it is
//! *not* a line-by-line port.
//!
//! Only [`PathCircle::path_length`]/[`PathCircle::pos`] are provided, not
//! `Vel`/`Acc`/`Write`/`Clone`/`LengthToS`: `generate_joint_trajectory`
//! (this crate's `trajectory_functions` module) only ever calls
//! [`crate::trajectory_functions::CartesianPath::duration`]/`pos` on a
//! Cartesian path, the same scope limit already documented on
//! [`crate::path_line::PathLine`].
//!
//! # Deviations from upstream
//!
//! - **Errors are [`cspace_core::error::Error::Construct`], not KDL exceptions.**
//!   Upstream throws `ErrorMotionPlanningCenterPointDifferentRadius` (a
//!   `KDL::Error_MotionPlanning` subclass, numeric code `3006`) from
//!   [`circle_from_center`], `KDL::Error_MotionPlanning_Circle_No_Plane` from
//!   [`circle_from_interim`], and `Path_Circle`'s own constructor throws
//!   `Error_MotionPlanning_Circle_ToSmall`/`Error_MotionPlanning_Circle_No_Plane`
//!   again for degeneracies it detects itself ([`PathCircle::new`]); this
//!   port has no KDL exception hierarchy, so all map to `Error::Construct`
//!   with a message naming the upstream exception, matching this crate's
//!   house error convention.
//! - **The degeneracy tolerance is an explicit parameter, not a swapped
//!   global.** Upstream's `Path_Circle` constructor checks its own
//!   `radius`/plane-normal degeneracy against the *global* `KDL::epsilon`,
//!   and `circleFromCenter` temporarily overwrites that global with
//!   `MAX_COLINEAR_NORM` for the duration of the call (`circleFromInterim`
//!   does not — it leaves the global at its ordinary default). This module
//!   has no global to swap, so [`PathCircle::new`] takes that tolerance as
//!   an explicit `eps` argument instead: callers built on
//!   [`circle_from_center`] must pass [`MAX_COLINEAR_NORM`], callers built on
//!   [`circle_from_interim`] must pass
//!   [`crate::velocity_profile::KDL_EPSILON`] — reproducing the same two
//!   tolerances upstream's swap produces, without a mutable global.
//! - **A "both zero" guard upstream lacks, closing its own asymmetry.**
//!   `KDL::Path_Line`'s constructor has an explicit branch for
//!   `angle == 0 && dist == 0` (see [`crate::path_line::PathLine::new`]'s own
//!   deviation note); `KDL::Path_Circle`'s constructor has no equivalent
//!   branch — its `else` arm unconditionally computes `scalerot = oalpha /
//!   pathlength`, dividing by `dist` even when both `oalpha` and `dist` are
//!   zero, so upstream's `scalerot` is `NaN` there. [`PathCircle::new`] adds
//!   the arm `Path_Line` already has, with that arm's own placeholder values,
//!   rather than reproducing a `NaN` into the constructed path. That arm is
//!   reached through the ordinary CIRC path, not merely by an out-of-crate
//!   caller poking [`CircleGeometry`]'s `pub` fields:
//!   [`circle_from_interim`] does refuse a coincident start/goal — `goal -
//!   start` is then zero, so the triangle normal cannot clear
//!   [`MAX_COLINEAR_NORM`] — but [`circle_from_center`] has no colinearity
//!   guard at all. Its only geometry check is [`MAX_RADIUS_DIFF`] against
//!   `|a - b|`, which a coincident start/goal at a real radius passes,
//!   returning `alpha == 0.0`; `trajectory_generator_circ::build_path`'s
//!   `Center` arm hands that straight to [`PathCircle::new`]. Pinned by
//!   `coincident_sweep_and_rotation_does_not_divide_zero_by_zero` together
//!   with `a_zero_side_is_rejected_even_when_the_radius_check_passes`, whose
//!   control case asserts that `alpha == 0.0` return.
//!
//! # Why this file stays BSD-3-Clause
//!
//! `KDL::Path_Circle` and `RotationalInterpolation_SingleAxis` are
//! LGPL-2.1-or-later (`third_party/orocos_kinematics_dynamics/`), heavier
//! copyleft than this workspace's BSD-3-Clause. [`PathCircle`] is therefore
//! not transcribed from `orocos_kdl/src/path_circle.cpp`: its geometry is
//! derived independently from elementary vector algebra (an orthonormal
//! frame built from the start-to-center radius vector and the plane normal
//! through the auxiliary point; position sampled by rotating that radius
//! vector through the swept angle). What is reused from the LGPL source is
//! *interface facts*, not expression — named here by convention rather than
//! by file:line: the constructor's argument roles (a center point, an
//! auxiliary point pinning the plane, and a start/goal rotation pair), the
//! `eqradius` convention balancing translational against rotational arc
//! length into one path parameter (already independently derived for
//! [`crate::path_line::PathLine`], reused verbatim since `Path_Line` and
//! `Path_Circle` share the identical balancing rule), and the
//! `RotationalInterpolation_SingleAxis` axis-angle convention (already
//! ported as `path_line::get_rot_angle` for `PathLine`, reused as-is here).
//! Equivalence with upstream is proven the same way every other
//! generator in this crate proves it: oracle parity on captured fixtures
//! (`tests/pilz_trajectory_circ_parity.rs`), not line correspondence.

use cspace_core::error::{Error, Result};
use cspace_core::geometry::{Isometry3, UnitQuaternion, Vector3};
use nalgebra::Unit;

use crate::path_line::{get_rot_angle, kdl_normalize};

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
/// `ErrorMotionPlanningCenterPointDifferentRadius`), or if either distance
/// is zero — the [`MAX_RADIUS_DIFF`] check compares `|a - b|` and so admits
/// `a == b == 0.0`, which upstream's `cosines` divides by, returning a `NaN`
/// sweep angle. That second error has no upstream counterpart.
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

    let alpha = cosines(a, b, c)?;

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
    let mut alpha = cosines(a, b, c)?;

    let mut aux_point = interim;

    // If the angle at the interim point is acute, the intended arc is the
    // major arc (the naive law-of-cosines angle above describes the minor
    // arc between start and goal, which does not pass through interim).
    let interim_angle = cosines(t.norm(), v.norm(), u.norm())?;
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
///
/// # Deviation from upstream: the zero-denominator guard
///
/// Upstream divides by `2ab` unguarded
/// (`path_circle_generator.cpp:127-129`). A degenerate triangle makes that
/// `0.0/0.0`, and `NaN` survives both clamp forms — upstream's
/// `std::max(std::min(x, 1.0), -1.0)` returns its first argument when a
/// comparison against `NaN` is false, exactly as Rust's `clamp` propagates
/// `NaN` — so `acos` yields `NaN` and a `NaN` `alpha` flows into
/// [`CircleGeometry`] with nothing downstream rejecting it.
///
/// The same author guarded the same hazard in the sibling generator:
/// `path_polyline_generator.cpp:99-100` tests `norm_product` against
/// `MIN_SEGMENT_LENGTH²` and returns early, with the comment "avoid
/// division by zero". That guard was never carried across to `cosines`.
/// This is the same asymmetry KDL left between `Path_Line` and
/// `Path_Circle`, which [`PathCircle::new`] closes for its own division.
///
/// Guarding here rather than at the callers is deliberate: it is the one
/// place the division lives, so no caller can produce `NaN` regardless of
/// which guards happen to sit above it today. Of the three call sites only
/// [`circle_from_center`] is reachable with a zero side — its
/// `MAX_RADIUS_DIFF` check compares `|a - b|` and so admits `a == b == 0.0`
/// (start, goal and centre coincident) — while both [`circle_from_interim`]
/// sites sit below the `w.norm() < MAX_COLINEAR_NORM` colinearity error.
/// That is a reachability argument, and this guard is a structural one.
fn cosines(a: f64, b: f64, c: f64) -> Result<f64> {
    let denominator = 2.0 * a * b;
    if denominator <= 0.0 {
        return Err(Error::construct(
            "a circle side length is zero; the angle between the points is undefined",
        ));
    }
    Ok((((a * a + b * b - c * c) / denominator).clamp(-1.0, 1.0)).acos())
}

/// The interpolated position+orientation path of a circular arc, playing the
/// role of upstream's `KDL::Path_Circle` — independently derived, not
/// transcribed; see the [module docs](self) for why and what convention
/// facts are reused.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PathCircle {
    orient_start: UnitQuaternion,
    rot_axis: Unit<Vector3>,
    center: Vector3,
    x_axis: Vector3,
    y_axis: Vector3,
    radius: f64,
    path_length: f64,
    scale_lin: f64,
    scale_rot: f64,
}

impl PathCircle {
    /// Builds the interpolated arc from `start` to `goal` along `geometry`
    /// (as solved by [`circle_from_center`]/[`circle_from_interim`]).
    ///
    /// `eqradius` is the equivalent radius balancing rotational against
    /// translational path length, the same convention
    /// [`crate::path_line::PathLine::new`] uses. `eps` is the degeneracy
    /// tolerance for the radius and plane-normal checks below — see the
    /// [module docs](self)'s deviation note for which constant callers must
    /// pass depending on whether `geometry` came from [`circle_from_center`]
    /// ([`MAX_COLINEAR_NORM`]) or [`circle_from_interim`]
    /// ([`crate::velocity_profile::KDL_EPSILON`]).
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] if `start` coincides with `geometry.center`
    /// (upstream: `Error_MotionPlanning_Circle_ToSmall`), or if
    /// `geometry.aux_point` is colinear with `start` and `geometry.center`,
    /// leaving the circle's plane undetermined (upstream:
    /// `Error_MotionPlanning_Circle_No_Plane`).
    pub fn new(
        start: &Isometry3,
        goal: &Isometry3,
        geometry: &CircleGeometry,
        eqradius: f64,
        eps: f64,
    ) -> Result<Self> {
        let center = geometry.center;

        let (x_axis, radius) = kdl_normalize(start.translation.vector - center, eps);
        if radius < eps {
            return Err(Error::construct(
                "circle radius too small to determine a plane; a circle cannot be created",
            ));
        }

        // Gated on its own norm immediately, like `radius` above: relying
        // instead on `x_axis.cross(&tmpv)` to collapse to the zero vector
        // and trip the colinear guard below is not a checked invariant, it
        // is a coincidence of kdl_normalize's own Vector3::zeros()
        // substitution (cross(_, 0) == 0 exactly). Swap that substitution
        // for KDL's own Vector(1,0,0) fallback (`frames.cpp:147-156`) and
        // the outcome becomes value-dependent on x_axis's direction instead
        // of a guaranteed rejection -- measured: x_axis == (0,1,0) then
        // *succeeds*, building a geometrically arbitrary circle. See
        // `kdl_normalize`'s own doc comment for the substitution choice.
        let (tmpv, aux_norm) = kdl_normalize(geometry.aux_point - center, eps);
        if aux_norm < eps {
            return Err(Error::construct(
                "auxiliary point coincides with the circle center; a circle cannot be created",
            ));
        }
        let (z_axis, z_norm) = kdl_normalize(x_axis.cross(&tmpv), eps);
        if z_norm < eps {
            return Err(Error::construct(
                "start, center and auxiliary point are colinear; a circle cannot be created",
            ));
        }
        let y_axis = z_axis.cross(&x_axis);

        let r_start_end = start.rotation.inverse() * goal.rotation;
        let (oalpha, rot_axis) = get_rot_angle(&r_start_end, eps);

        let dist = geometry.alpha * radius;
        let (path_length, scale_lin, scale_rot) = if oalpha * eqradius > dist {
            (
                oalpha * eqradius,
                dist / (oalpha * eqradius),
                1.0 / eqradius,
            )
        } else if dist > 0.0 {
            // Translation is the limitation.
            (dist, 1.0, oalpha / dist)
        } else {
            // Both extents are zero. Upstream `Path_Circle` has no such arm
            // and divides `0.0/0.0` here; KDL added exactly this arm to
            // `Path_Line` (`path_line.cpp:78-82`, its own comment reads
            // "both were zero") and never carried it across. Placeholder
            // values match that arm, and this port's own `PathLine::new`.
            (0.0, 1.0, 1.0)
        };

        Ok(Self {
            orient_start: start.rotation,
            rot_axis,
            center,
            x_axis,
            y_axis,
            radius,
            path_length,
            scale_lin,
            scale_rot,
        })
    }

    /// Upstream `PathLength`.
    pub fn path_length(&self) -> f64 {
        self.path_length
    }

    /// Upstream `Pos`.
    pub fn pos(&self, s: f64) -> Isometry3 {
        let p = s * self.scale_lin / self.radius;
        let translation = self.center
            + self.x_axis * (self.radius * p.cos())
            + self.y_axis * (self.radius * p.sin());
        let theta = s * self.scale_rot;
        let rotation = self.orient_start * UnitQuaternion::from_axis_angle(&self.rot_axis, theta);
        Isometry3::from_parts(translation.into(), rotation)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use cspace_core::test_support::KnownOracleDeviation;
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

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
    ///
    /// ASSERTION-DISCRIMINATION AUDIT (round 2): `single-branch` --
    /// `circle_from_center` has exactly one `Error::` site (`rg -c
    /// 'Error::'` scoped to the function body: 1), so a bare `.is_err()`
    /// has exactly one cause.
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
    ///
    /// ASSERTION-DISCRIMINATION AUDIT (round 2): `single-branch` --
    /// `circle_from_interim` has exactly one `Error::` site (`rg -c
    /// 'Error::'` scoped to the function body: 1), so a bare `.is_err()`
    /// has exactly one cause.
    #[test]
    fn circle_from_interim_colinear_points_is_rejected() {
        let result = circle_from_interim(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(2.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
        );
        assert!(result.is_err());
    }

    // -- PathCircle --

    fn identity_pose(p: Vector3) -> Isometry3 {
        Isometry3::from_parts(p.into(), UnitQuaternion::identity())
    }

    /// `pos(0)`/`pos(path_length)` reproduce `start`/`goal` exactly, for a
    /// quarter circle with a non-trivial orientation change.
    #[test]
    fn pos_at_zero_and_path_length_reproduces_start_and_goal() {
        let start = Isometry3::from_parts(
            Vector3::new(1.0, 0.0, 0.0).into(),
            UnitQuaternion::identity(),
        );
        let goal = Isometry3::from_parts(
            Vector3::new(0.0, 1.0, 0.0).into(),
            UnitQuaternion::from_euler_angles(0.0, 0.0, FRAC_PI_2),
        );
        let geom = circle_from_center(
            start.translation.vector,
            goal.translation.vector,
            Vector3::new(0.0, 0.0, 0.0),
        )
        .unwrap();
        let path = PathCircle::new(&start, &goal, &geom, 1.0, MAX_COLINEAR_NORM).unwrap();

        let at_start = path.pos(0.0);
        assert_relative_eq!(
            at_start.translation.vector,
            start.translation.vector,
            epsilon = 1e-9
        );
        assert_relative_eq!(
            at_start.rotation.quaternion().coords,
            start.rotation.quaternion().coords,
            epsilon = 1e-9
        );

        let at_end = path.pos(path.path_length());
        assert_relative_eq!(
            at_end.translation.vector,
            goal.translation.vector,
            epsilon = 1e-9
        );
        let same_rotation =
            (at_end.rotation.quaternion().coords - goal.rotation.quaternion().coords).norm() < 1e-9
                || (at_end.rotation.quaternion().coords + goal.rotation.quaternion().coords).norm()
                    < 1e-9;
        assert!(
            same_rotation,
            "{:?} != +/-{:?}",
            at_end.rotation, goal.rotation
        );
    }

    /// A full quarter circle at the unit radius sweeps a path length of
    /// `radius * alpha` when translation, not rotation, is the limiting
    /// motion (identical start/goal orientation, so `oalpha == 0`).
    #[test]
    fn pure_translation_path_length_is_radius_times_alpha() {
        let start = identity_pose(Vector3::new(1.0, 0.0, 0.0));
        let goal = identity_pose(Vector3::new(0.0, 1.0, 0.0));
        let geom = circle_from_center(
            start.translation.vector,
            goal.translation.vector,
            Vector3::new(0.0, 0.0, 0.0),
        )
        .unwrap();
        let path = PathCircle::new(&start, &goal, &geom, 1.0, MAX_COLINEAR_NORM).unwrap();
        assert_relative_eq!(path.path_length(), FRAC_PI_2, epsilon = 1e-9);

        let midpoint = path.pos(path.path_length() / 2.0);
        assert_relative_eq!(
            midpoint.translation.vector,
            Vector3::new(FRAC_PI_4.cos(), FRAC_PI_4.sin(), 0.0),
            epsilon = 1e-9
        );
    }

    /// Boundary: `start == center` leaves no radius to build a plane from
    /// (`Error_MotionPlanning_Circle_ToSmall`). Constructed directly against
    /// a hand-built [`CircleGeometry`], bypassing
    /// `circle_from_center`/`circle_from_interim`, since neither solver can
    /// itself produce a zero-radius geometry from non-degenerate inputs.
    ///
    /// `PathCircle::new` has three `Error::construct` sites (`rg -c
    /// 'Error::' path_circle.rs` restricted to the function body: 3), so a
    /// bare `.is_err()` cannot say which fired -- worse, here it would not
    /// even prove *this* guard fired at all: with `x_axis` zeroed by the
    /// zero-radius guard, the colinear-plane guard two guards below it
    /// would also see a zero cross product and independently error too
    /// (the auxiliary-point guard between them does not fire here: this
    /// fixture's `aux_point` is far from `center`), so no-opping the radius
    /// guard alone does not flip `.is_err()` to `false` (checked directly:
    /// it stays `true`, driven by the colinear guard). Only the message
    /// discriminates -- checked below.
    #[test]
    fn zero_radius_is_rejected() {
        let start = identity_pose(Vector3::new(0.0, 0.0, 0.0));
        let goal = identity_pose(Vector3::new(1.0, 0.0, 0.0));
        let geom = CircleGeometry {
            center: Vector3::new(0.0, 0.0, 0.0),
            radius: 0.0,
            alpha: FRAC_PI_2,
            aux_point: Vector3::new(0.0, 1.0, 0.0),
        };
        let result = PathCircle::new(&start, &goal, &geom, 1.0, MAX_COLINEAR_NORM);
        let err = result.unwrap_err();
        assert!(
            err.to_string().contains("too small"),
            "expected the zero-radius message, got {err}"
        );
    }

    /// Boundary: `aux_point == center` leaves the auxiliary-point
    /// normalization with nothing to normalize. `CircleGeometry`'s fields
    /// are all `pub` and so is `PathCircle::new`, so this is reachable from
    /// outside this crate directly (neither `circle_from_center` nor
    /// `circle_from_interim`, this file's own in-crate callers, can
    /// themselves produce a coincident `aux_point`/`center`).
    ///
    /// Before this guard existed, this input was still rejected -- but only
    /// because `kdl_normalize`'s zero-vector substitution for `tmpv` made
    /// `x_axis.cross(&tmpv)` collapse to the zero vector, which the
    /// colinear-plane guard below then caught by coincidence, not by a
    /// check on this input's own degeneracy. Measured directly (temporarily
    /// swapping that substitution for KDL's own `Vector(1,0,0)` fallback,
    /// `frames.cpp:147-156`): with `x_axis == (0.0, 1.0, 0.0)`, the same
    /// input instead *succeeded*, building a geometrically arbitrary circle
    /// (`radius: 1.0`, `path_length: 1.5707963267948966`) rather than being
    /// rejected. This guard makes the rejection hold by construction
    /// instead of by that coincidence -- checked on the message, not a bare
    /// `.is_err()`, so a future change that removes this guard cannot pass
    /// silently by falling through to the colinear guard the way the old
    /// code did.
    #[test]
    fn auxiliary_point_coincident_with_center_is_rejected_by_its_own_guard() {
        let start = identity_pose(Vector3::new(0.0, 1.0, 0.0));
        let goal = identity_pose(Vector3::new(0.0, 0.0, 1.0));
        let geom = CircleGeometry {
            center: Vector3::new(0.0, 0.0, 0.0),
            radius: 1.0,
            alpha: FRAC_PI_2,
            aux_point: Vector3::new(0.0, 0.0, 0.0), // == center
        };
        let result = PathCircle::new(&start, &goal, &geom, 1.0, MAX_COLINEAR_NORM);
        let err = result.unwrap_err();
        assert!(
            err.to_string().contains("coincides"),
            "expected the coincident-auxiliary-point message, got {err}"
        );
    }

    /// Boundary: a half circle solved via [`circle_from_center`] succeeds at
    /// the geometry layer (`start`/`goal` are equidistant from `center`), but
    /// its auxiliary point (upstream convention: `goal` itself) is exactly
    /// colinear with the start-to-center radius vector, leaving the circle's
    /// plane undetermined (`Error_MotionPlanning_Circle_No_Plane`) -- the
    /// documented reason [`circle_from_center`] cannot express a half circle.
    ///
    /// `PathCircle::new`'s other two `Error::construct` sites are the
    /// zero-radius guard (see [`zero_radius_is_rejected`]'s own doc comment)
    /// and the auxiliary-point-coincides-with-center guard; `radius` here is
    /// `1.0` and `aux_point` (`goal`, via [`circle_from_center`]) is a unit
    /// distance from `center`, so neither guard can fire and there is no
    /// sibling-guard ambiguity in the other direction the way there is
    /// there. Checked on the message anyway, for the same reason every site
    /// in this family is: a future change to guard order or a shared early
    /// return should not let this test start passing for the wrong guard
    /// silently.
    #[test]
    fn half_circle_from_center_has_no_determinable_plane() {
        let start = identity_pose(Vector3::new(1.0, 0.0, 0.0));
        let goal = identity_pose(Vector3::new(-1.0, 0.0, 0.0));
        let geom = circle_from_center(
            start.translation.vector,
            goal.translation.vector,
            Vector3::new(0.0, 0.0, 0.0),
        )
        .unwrap();
        let result = PathCircle::new(&start, &goal, &geom, 1.0, MAX_COLINEAR_NORM);
        let err = result.unwrap_err();
        assert!(
            err.to_string().contains("colinear"),
            "expected the colinear-plane message, got {err}"
        );
    }

    /// KDL guards `Path_Line`'s `scalerot = alpha/pathlength` with an
    /// explicit three-way form whose third arm carries its own comment
    /// `// both were zero` (`orocos_kdl/src/path_line.cpp:67-83`);
    /// `Path_Circle`'s two-arm form (`path_circle.cpp:86-95`) never got the
    /// same treatment, so its `scalerot = oalpha/pathlength` evaluates
    /// `0.0/0.0` when the sweep angle and the rotation angle are both zero.
    ///
    /// `CircleGeometry`'s fields are all `pub` and so is [`PathCircle::new`],
    /// so this is reachable from outside this crate whatever the two in-crate
    /// callers ([`circle_from_center`]/[`circle_from_interim`], both of which
    /// reject a coincident start/goal at their own colinearity guard first)
    /// can produce. `alpha == 0.0` with a plane that is still determined
    /// needs only an auxiliary point off the start-to-center radius, which
    /// this fixture supplies.
    #[test]
    fn coincident_sweep_and_rotation_does_not_divide_zero_by_zero() {
        let start = identity_pose(Vector3::new(1.0, 0.0, 0.0));
        let goal = identity_pose(Vector3::new(1.0, 0.0, 0.0));
        let geom = CircleGeometry {
            center: Vector3::new(0.0, 0.0, 0.0),
            radius: 1.0,
            alpha: 0.0,
            aux_point: Vector3::new(0.0, 1.0, 0.0),
        };
        let path = PathCircle::new(&start, &goal, &geom, 1.0, MAX_COLINEAR_NORM)
            .expect("radius 1.0 and a non-colinear auxiliary point clear both guards");
        assert_eq!(
            path.path_length, 0.0,
            "a zero sweep over a unit radius is a zero-length path"
        );
        assert_eq!(
            path.scale_rot, 1.0,
            "scale_rot must take Path_Line's both-zero placeholder, not 0.0/0.0"
        );
        assert_eq!(
            path.scale_lin, 1.0,
            "scale_lin must take Path_Line's both-zero placeholder"
        );
    }

    /// [`KnownOracleDeviation`] proof that the "both zero" arm above actually
    /// diverges from upstream's own unguarded division, not just from the
    /// placeholder values this port chose for it.
    ///
    /// `oracle`/`actual` are compared as `is_nan()` booleans, not the raw
    /// `f64`s: under IEEE 754 `NaN != NaN` unconditionally, so a raw
    /// comparison would read "diverged" even if a regression reintroduced
    /// the division and both sides went back to `NaN` -- exactly the
    /// regression [`KnownOracleDeviation`] exists to catch, and the one
    /// direction a raw-value comparison cannot catch here.
    #[test]
    fn scale_rot_diverges_from_upstreams_unguarded_division() {
        let mut deviation = KnownOracleDeviation::new(
            "PathCircle::new's \"both zero\" scale_rot",
            "orocos_kdl/src/path_circle.cpp:86-95 (unguarded `scalerot = oalpha/pathlength`), \
             orocos_kdl/src/path_line.cpp:67-83 (the guard KDL wrote for Path_Line but never \
             carried across to Path_Circle)",
            "5b1f5021",
        );

        let start = identity_pose(Vector3::new(1.0, 0.0, 0.0));
        let goal = identity_pose(Vector3::new(1.0, 0.0, 0.0));
        let geom = CircleGeometry {
            center: Vector3::new(0.0, 0.0, 0.0),
            radius: 1.0,
            alpha: 0.0,
            aux_point: Vector3::new(0.0, 1.0, 0.0),
        };
        let path = PathCircle::new(&start, &goal, &geom, 1.0, MAX_COLINEAR_NORM)
            .expect("radius 1.0 and a non-colinear auxiliary point clear both guards");

        // Upstream's own unguarded expression (path_circle.cpp:94's
        // `scalerot = oalpha/pathlength`), evaluated with the zero
        // `oalpha`/`dist` (upstream's `pathlength`) this geometry produces:
        // a coincident start/goal rotation gives `oalpha == 0.0`, and
        // `alpha == 0.0` gives `dist == geometry.alpha * radius == 0.0`.
        // This is upstream's formula re-evaluated on those inputs, not a
        // value read from `PathCircle::new`, which never computes this
        // division at all.
        let oalpha = 0.0_f64;
        let dist = 0.0_f64;
        let upstream_scale_rot = oalpha / dist;

        deviation.observe(
            "alpha=0.0, coincident start/goal rotation",
            &upstream_scale_rot.is_nan(),
            &path.scale_rot.is_nan(),
        );
        deviation.finish();
    }

    #[test]
    fn coincident_start_goal_and_center_is_rejected_not_returned_as_nan() {
        // `MAX_RADIUS_DIFF` compares `|a - b|`, so `a == b == 0.0` clears it:
        // the radius check cannot see this case. Upstream's `cosines` then
        // divides `0.0 / 0.0` and hands back a `NaN` `alpha`
        // (`path_circle_generator.cpp:127-129`), which nothing downstream
        // rejects.
        let coincident = Vector3::new(0.4, -0.2, 0.9);
        let err = circle_from_center(coincident, coincident, coincident)
            .expect_err("a circle through three coincident points is not constructible");
        assert!(
            matches!(err, Error::Construct(_)),
            "must be the crate's construction error, not a NaN geometry: {err:?}"
        );
    }

    #[test]
    fn a_zero_side_is_rejected_even_when_the_radius_check_passes() {
        // Start and goal both AT the centre is the reachable shape above; this
        // is the narrower statement the guard actually makes, exercised
        // through the same public entry point: any zero side length is
        // refused rather than divided by.
        let center = Vector3::new(0.0, 0.0, 0.0);
        let on_circle = Vector3::new(1.0, 0.0, 0.0);
        assert!(
            circle_from_center(center, center, center).is_err(),
            "a == b == 0.0 must be refused"
        );
        // The control: a genuine zero-sweep circle (start == goal, both a real
        // radius from the centre) still succeeds, so the guard has not been
        // widened into rejecting ordinary degenerate-but-valid input.
        let ok = circle_from_center(on_circle, on_circle, center)
            .expect("start == goal at a real radius is a zero sweep, not a zero side");
        assert_eq!(ok.alpha, 0.0, "a coincident start/goal sweeps zero radians");
        assert_eq!(ok.radius, 1.0);
    }
}
