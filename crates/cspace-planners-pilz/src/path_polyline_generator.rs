// Copyright (c) 2018, Pilz GmbH & Co. KG
// Copyright (c) 2025, Aiman Haidar
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/pilz_industrial_motion_planner/include/pilz_industrial_motion_planner/path_polyline_generator.hpp
//   moveit_planners/pilz_industrial_motion_planner/src/path_polyline_generator.cpp

//! Polyline path building for `POLYLINE` motions
//! ([`polyline_from_waypoints`]), ported from upstream's
//! `PathPolylineGenerator`.
//!
//! Upstream turns a start pose plus a waypoint list into a
//! [`crate::path_rounded_composite::PathRoundedComposite`] in three steps:
//! drop waypoints too close to their predecessor ([`filter_waypoints`]),
//! choose the single corner radius that fits every corner
//! ([`compute_blend_radius`]), then feed the surviving poses through the
//! composite. `Path_RoundedComposite` cannot vary its radius per corner, so
//! the radius is the tightest corner's, scaled by the request's smoothness.
//!
//! # Deviations from upstream
//!
//! - **Errors are [`Error::Construct`], not KDL exceptions.** Upstream's
//!   colinearity check throws
//!   `ErrorMotionPlanningColinearConsicutiveWaypoints` (numeric code `3104`);
//!   the message here names that code, the same convention
//!   [`crate::path_rounded_composite`] uses for
//!   `Error_MotionPlanning_Not_Feasible`.
//! - **`smoothness` is clamped, not validated.** Upstream clamps into
//!   `[MIN_SMOOTHNESS, MAX_SMOOTHNESS]` with no diagnostic, so a request
//!   asking for `2.0` silently gets `0.99`. Reproduced rather than turned
//!   into a rejection.
//! - **[`filter_waypoints`] compares against the last kept pose, not a stale
//!   input index.** Upstream's `last_added_point_indx` counts kept waypoints
//!   but indexes into the input list, so it drifts after any drop and the
//!   filter can keep a waypoint it was written to remove. Fixed here rather
//!   than reproduced — see [`filter_waypoints`]'s own doc.

use cspace_core::error::{Error, Result};
use cspace_core::geometry::Isometry3;

use crate::path_rounded_composite::PathRoundedComposite;

/// Waypoints closer than this to their predecessor are dropped.
///
/// Upstream `PathPolylineGenerator::MIN_SEGMENT_LENGTH` (`0.2e-3`). The
/// filter exists so `Path_RoundedComposite::Add` is never handed the
/// near-zero segment its own codes 2/3 would reject.
pub const MIN_SEGMENT_LENGTH: f64 = 0.2e-3;

/// Lower clamp on the smoothness factor. Upstream `MIN_SMOOTHNESS`.
pub const MIN_SMOOTHNESS: f64 = 0.01;

/// Upper clamp on the smoothness factor. Upstream `MAX_SMOOTHNESS`.
///
/// Strictly below `1.0`: at exactly `1.0` the rounding arc of the tightest
/// corner would consume its whole shorter leg, which is
/// `Path_RoundedComposite::Add`'s codes 5/6.
pub const MAX_SMOOTHNESS: f64 = 0.99;

/// Three consecutive waypoints whose turn cross-product falls below this are
/// treated as colinear. Upstream `MIN_COLINEAR_NORM`.
pub const MIN_COLINEAR_NORM: f64 = 1e-9;

/// Builds the rounded polyline through `start_pose` then `waypoints`.
///
/// `smoothness` scales the largest radius every corner can take (clamped into
/// `[MIN_SMOOTHNESS, MAX_SMOOTHNESS]`); `eqradius` is the
/// translation/rotation balance passed on to each segment, the same
/// `eqradius` [`crate::path_line::PathLine::new`] takes.
///
/// Upstream `polylineFromWaypoints`.
///
/// # Errors
///
/// [`Error::Construct`] if any three consecutive surviving waypoints are
/// colinear (upstream `ErrorMotionPlanningColinearConsicutiveWaypoints`), or
/// for any of [`PathRoundedComposite`]'s own construction failures.
pub fn polyline_from_waypoints(
    start_pose: &Isometry3,
    waypoints: &[Isometry3],
    smoothness: f64,
    eqradius: f64,
) -> Result<PathRoundedComposite> {
    let filtered = filter_waypoints(start_pose, waypoints);
    let blend_radius = compute_blend_radius(&filtered, smoothness)?;

    let mut path = PathRoundedComposite::new(blend_radius, eqradius)?;
    for waypoint in &filtered {
        path.add(*waypoint)?;
    }
    path.finish();
    Ok(path)
}

/// Drops every waypoint within [`MIN_SEGMENT_LENGTH`] of the last kept one.
///
/// Upstream `filterWaypoints`. The returned vector always starts with
/// `start_pose`, which is never dropped.
///
/// # Deviation: compares against the last *kept* pose, not a stale input index
///
/// Upstream tracks `last_added_point_indx` (`path_polyline_generator.cpp:66`),
/// incremented once per *kept* waypoint (`:82`), and reads
/// `waypoints[last_added_point_indx]` (`:71`) — an index into the *input*
/// list. While nothing has been dropped the two coincide; each drop shifts
/// them apart by one, so from the first kept waypoint *after* a drop onward
/// upstream compares against an earlier input waypoint — including one this
/// same filter already dropped — and keeps waypoints it was written to
/// remove. The filter's purpose is to guarantee every surviving segment
/// exceeds `MIN_SEGMENT_LENGTH` before `Path_RoundedComposite::Add` sees it;
/// upstream's version stops doing that past the first drop, so `Add`'s own
/// `Not_Feasible` codes 2/3 can still fire on input the filter was supposed
/// to have cleaned — a rejected plan, not merely a different waypoint list.
///
/// This port reads `filtered.last()` instead: the actual last kept pose, no
/// separate counter to drift from it. A valid plan being rejected is worse
/// than the parity surface this costs — see
/// `pilz_trajectory_polyline_parity.rs`'s
/// `polyline_panda_arm_diverges_from_the_oracles_stale_filter_index_rejection`,
/// which now measures the divergence directly: the captured oracle fixture
/// still rejects with `INVALID_MOTION_PLAN` (`-2`), this port now returns
/// `SUCCESS`. Previously reproduced verbatim and recorded in the now-deleted
/// `doc/upstream-bugs.md` as `polyline-filter-waypoints-stale-index`.
pub fn filter_waypoints(start_pose: &Isometry3, waypoints: &[Isometry3]) -> Vec<Isometry3> {
    let mut filtered = vec![*start_pose];

    for waypoint in waypoints {
        let last_point = filtered
            .last()
            .expect("filtered always holds at least start_pose")
            .translation
            .vector;
        if (last_point - waypoint.translation.vector).norm() > MIN_SEGMENT_LENGTH {
            filtered.push(*waypoint);
        }
    }
    filtered
}

/// The largest corner radius that fits every corner, scaled by `smoothness`.
///
/// Upstream `computeBlendRadius`. Returns [`f64::INFINITY`] when there is no
/// interior corner to constrain the radius (fewer than three waypoints, or
/// every corner skipped for a too-short leg) — upstream's own initial value,
/// left as-is here because [`PathRoundedComposite::add`] never consults the
/// radius in those cases.
///
/// # Errors
///
/// [`Error::Construct`] if any three consecutive waypoints are colinear.
pub fn compute_blend_radius(waypoints: &[Isometry3], smoothness: f64) -> Result<f64> {
    let mut max_allowed_radius = f64::INFINITY;

    for i in 1..waypoints.len().saturating_sub(1) {
        let p1 = waypoints[i - 1].translation.vector;
        let p2 = waypoints[i].translation.vector;
        let p3 = waypoints[i + 1].translation.vector;

        let dist1 = (p2 - p1).norm();
        let dist2 = (p3 - p2).norm();
        // Upstream checks colinearity *before* the short-leg skip, so a
        // corner too short to constrain the radius is still rejected when it
        // is colinear. Order preserved.
        check_consecutive_colinear_waypoints(waypoints[i - 1], waypoints[i], waypoints[i + 1])?;
        if dist1 < MIN_SEGMENT_LENGTH || dist2 < MIN_SEGMENT_LENGTH {
            continue;
        }

        // `v1`/`v2` both point *away* from the corner, so `theta` is the
        // interior angle: `PI - alpha`, where `alpha` is what
        // `PathRoundedComposite::add` measures from vectors pointing along
        // travel. That makes `tan(theta / 2)` exactly `add`'s own
        // `tan((PI - alpha) / 2)`, so the radius chosen here lands `add`'s
        // `d` at the shorter leg's midpoint -- the largest radius that still
        // clears `add`'s codes 5/6 at this corner.
        let theta = segment_angle(p1, p2, p3);
        let local_max_radius = (theta / 2.0).tan().abs() * (dist1 / 2.0).min(dist2 / 2.0);

        // One radius has to serve every corner: `Path_RoundedComposite` takes
        // it once at construction.
        if local_max_radius < max_allowed_radius {
            max_allowed_radius = local_max_radius;
        }
    }

    Ok(max_allowed_radius * smoothness.clamp(MIN_SMOOTHNESS, MAX_SMOOTHNESS))
}

/// The interior angle at `p2`, between the segments to `p1` and to `p3`.
///
/// Upstream's `segment_angle` lambda. Returns `0.0` for a degenerate corner
/// rather than dividing by zero.
fn segment_angle(
    p1: cspace_core::geometry::Vector3,
    p2: cspace_core::geometry::Vector3,
    p3: cspace_core::geometry::Vector3,
) -> f64 {
    let v1 = p2 - p1;
    let v2 = p2 - p3;

    let norm_product = v1.norm() * v2.norm();
    if norm_product < MIN_SEGMENT_LENGTH * MIN_SEGMENT_LENGTH {
        return 0.0;
    }
    (v1.dot(&v2) / norm_product).clamp(-1.0, 1.0).acos()
}

/// Rejects three consecutive colinear waypoints.
///
/// Upstream `checkConsecutiveColinearWaypoints`. A colinear corner has no
/// determined rounding plane, so `Path_Circle` could not be constructed for
/// it at all.
///
/// # Errors
///
/// [`Error::Construct`] when `|v1 x v2|` is below [`MIN_COLINEAR_NORM`].
pub fn check_consecutive_colinear_waypoints(
    p1: Isometry3,
    p2: Isometry3,
    p3: Isometry3,
) -> Result<()> {
    let v1 = p2.translation.vector - p1.translation.vector;
    let v2 = p3.translation.vector - p2.translation.vector;

    if v1.cross(&v2).norm() < MIN_COLINEAR_NORM {
        return Err(Error::construct(
            "three colinear consecutive waypoints; a polyline path cannot be created \
             (upstream ErrorMotionPlanningColinearConsicutiveWaypoints, code 3104)",
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use cspace_core::geometry::{UnitQuaternion, Vector3};

    use super::*;

    fn pose(x: f64, y: f64, z: f64) -> Isometry3 {
        Isometry3::from_parts(Vector3::new(x, y, z).into(), UnitQuaternion::identity())
    }

    // -- filter_waypoints --

    #[test]
    fn filter_waypoints_keeps_the_start_pose_even_with_no_waypoints() {
        let filtered = filter_waypoints(&pose(1.0, 2.0, 3.0), &[]);
        assert_eq!(filtered.len(), 1);
        assert_relative_eq!(
            filtered[0].translation.vector,
            Vector3::new(1.0, 2.0, 3.0),
            epsilon = 1e-12
        );
    }

    #[test]
    fn filter_waypoints_drops_one_within_the_minimum_segment_length() {
        // Second waypoint is 1e-5 from the first -- below `0.2e-3`.
        let filtered = filter_waypoints(
            &pose(0.0, 0.0, 0.0),
            &[pose(1.0, 0.0, 0.0), pose(1.0 + 1e-5, 0.0, 0.0)],
        );
        assert_eq!(filtered.len(), 2);
    }

    #[test]
    fn filter_waypoints_keeps_one_at_exactly_above_the_minimum_segment_length() {
        // The boundary's other side: `MIN_SEGMENT_LENGTH` is a strict `>`,
        // so a separation just above it survives and one exactly at it does
        // not.
        let just_above = filter_waypoints(
            &pose(0.0, 0.0, 0.0),
            &[pose(MIN_SEGMENT_LENGTH * 1.001, 0.0, 0.0)],
        );
        assert_eq!(just_above.len(), 2);
        let exactly_at =
            filter_waypoints(&pose(0.0, 0.0, 0.0), &[pose(MIN_SEGMENT_LENGTH, 0.0, 0.0)]);
        assert_eq!(exactly_at.len(), 1);
    }

    #[test]
    fn filter_waypoints_compares_against_the_last_kept_pose_not_a_stale_input_index() {
        // Deviation pin for `polyline-filter-waypoints-stale-index` (see
        // `filter_waypoints`'s own doc). Input 1 is dropped; upstream's
        // `last_added_point_indx` would then be one behind and compare input
        // 3 against input 1 -- the waypoint the filter itself just dropped --
        // a full 1.0 away, so upstream keeps it (4 kept). This port compares
        // against `filtered.last()`, the pose actually kept (input 2), which
        // is 1e-5 away and is dropped (3 kept). Asserting 3 here is what
        // makes a regression back to upstream's stale-index rule fail this
        // test.
        let filtered = filter_waypoints(
            &pose(0.0, 0.0, 0.0),
            &[
                pose(1.0, 0.0, 0.0),
                pose(1.0 + 1e-5, 0.0, 0.0),
                pose(2.0, 0.0, 0.0),
                pose(2.0 + 1e-5, 0.0, 0.0),
            ],
        );
        assert_eq!(filtered.len(), 3);
    }

    // -- compute_blend_radius --

    #[test]
    fn compute_blend_radius_is_infinite_without_an_interior_corner() {
        let radius =
            compute_blend_radius(&[pose(0.0, 0.0, 0.0), pose(1.0, 0.0, 0.0)], 0.5).unwrap();
        assert!(radius.is_infinite(), "{radius}");
    }

    #[test]
    fn compute_blend_radius_takes_the_tightest_corner() {
        // Two right-angle corners with different leg lengths: the first has
        // half-legs 0.5/0.5, the second 0.5/0.1. `tan(45deg) == 1`, so the
        // radii are 0.5 and 0.1 and the tighter must win.
        let radius = compute_blend_radius(
            &[
                pose(0.0, 0.0, 0.0),
                pose(1.0, 0.0, 0.0),
                pose(1.0, 1.0, 0.0),
                pose(1.2, 1.0, 0.0),
            ],
            1.0,
        )
        .unwrap();
        // `smoothness` clamps to MAX_SMOOTHNESS, not to 1.0.
        assert_relative_eq!(radius, 0.1 * MAX_SMOOTHNESS, epsilon = 1e-12);
    }

    #[test]
    fn compute_blend_radius_clamps_smoothness_at_both_ends() {
        let corner = [
            pose(0.0, 0.0, 0.0),
            pose(1.0, 0.0, 0.0),
            pose(1.0, 1.0, 0.0),
        ];
        let low = compute_blend_radius(&corner, -5.0).unwrap();
        assert_relative_eq!(low, 0.5 * MIN_SMOOTHNESS, epsilon = 1e-12);
        let high = compute_blend_radius(&corner, 5.0).unwrap();
        assert_relative_eq!(high, 0.5 * MAX_SMOOTHNESS, epsilon = 1e-12);
    }

    #[test]
    fn compute_blend_radius_rejects_colinear_waypoints() {
        let err = compute_blend_radius(
            &[
                pose(0.0, 0.0, 0.0),
                pose(1.0, 0.0, 0.0),
                pose(2.0, 0.0, 0.0),
            ],
            0.5,
        )
        .unwrap_err();
        assert!(err.to_string().contains("3104"), "{err}");
    }

    #[test]
    fn compute_blend_radius_rejects_a_colinear_corner_whose_legs_are_too_short() {
        // The colinearity check runs before the short-leg `continue`, so a
        // corner that would otherwise be skipped is still rejected. Swapping
        // those two statements is what this case catches.
        let tiny = MIN_SEGMENT_LENGTH / 10.0;
        let err = compute_blend_radius(
            &[
                pose(0.0, 0.0, 0.0),
                pose(tiny, 0.0, 0.0),
                pose(2.0 * tiny, 0.0, 0.0),
            ],
            0.5,
        )
        .unwrap_err();
        assert!(err.to_string().contains("3104"), "{err}");
    }

    /// A NaN waypoint component reaching `dist1`/`dist2` reaches `theta` too
    /// (`segment_angle`'s `v1`/`v2` are the exact same difference vectors),
    /// so `local_max_radius` is NaN and the aggregation `if local_max_radius
    /// < max_allowed_radius` discards it exactly as it would discard any
    /// other NaN — leaving the return at the untouched initial `INFINITY`,
    /// not at some value shaped by `(dist1 / 2.0).min(dist2 / 2.0)`. This is
    /// why that call is left as `f64::min` rather than [`crate::numeric`]'s
    /// `cxx_min` despite porting the same `std::min` upstream: the spelling
    /// is unreachable from this function's observable output. See
    /// `crate::numeric`'s module doc.
    #[test]
    fn compute_blend_radius_masks_a_nan_corner_at_the_aggregation_comparison() {
        let radius = compute_blend_radius(
            &[
                pose(0.0, 0.0, 0.0),
                pose(1.0, 1.0, 0.0),
                pose(f64::NAN, 2.0, 0.0),
            ],
            0.5,
        )
        .unwrap();
        assert!(
            radius.is_infinite() && radius.is_sign_positive(),
            "{radius}"
        );
    }

    // -- polyline_from_waypoints --

    #[test]
    fn polyline_from_waypoints_rounds_every_corner_it_is_given() {
        let path = polyline_from_waypoints(
            &pose(0.0, 0.0, 0.0),
            &[
                pose(1.0, 0.0, 0.0),
                pose(1.0, 1.0, 0.0),
                pose(0.0, 1.0, 0.0),
            ],
            0.5,
            1.0,
        )
        .unwrap();
        // Two interior corners -> line, arc, line, arc, closing line.
        assert_eq!(path.segment_count(), 5);
    }

    #[test]
    fn polyline_from_waypoints_is_shorter_than_the_raw_polyline() {
        // Rounding a corner replaces two half-legs with a shorter arc, so the
        // path length must come in strictly under the unrounded 3.0 -- and
        // must still exceed the straight-line 1.0 + 1.0 + 1.0 minus the two
        // full corner cuts.
        let path = polyline_from_waypoints(
            &pose(0.0, 0.0, 0.0),
            &[
                pose(1.0, 0.0, 0.0),
                pose(1.0, 1.0, 0.0),
                pose(0.0, 1.0, 0.0),
            ],
            0.5,
            1.0,
        )
        .unwrap();
        assert!(path.path_length() < 3.0, "{}", path.path_length());
        assert!(path.path_length() > 2.5, "{}", path.path_length());
    }

    #[test]
    fn polyline_from_waypoints_rejects_a_colinear_run() {
        let err = polyline_from_waypoints(
            &pose(0.0, 0.0, 0.0),
            &[pose(1.0, 0.0, 0.0), pose(2.0, 0.0, 0.0)],
            0.5,
            1.0,
        )
        .unwrap_err();
        assert!(err.to_string().contains("3104"), "{err}");
    }
}
