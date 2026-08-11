// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Used by moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf's
//   moveit_planners/pilz_industrial_motion_planner/src/path_polyline_generator.cpp
//   moveit_planners/pilz_industrial_motion_planner/src/trajectory_generator_polyline.cpp
// (`polylineFromWaypoints`, `TrajectoryGeneratorPOLYLINE::plan`).

//! Corner-rounded polyline paths ([`PathRoundedComposite`]), playing the role
//! of `KDL::Path_RoundedComposite` — the primitive `POLYLINE` motions are
//! built on. See below for why this is *not* a line-by-line port of it.
//!
//! A rounded composite is fed a sequence of via poses; each interior corner
//! is replaced by two shortened straight segments and a tangent circular arc
//! of a fixed `radius`, so the resulting path is C¹ where the raw polyline
//! would have had a cusp. The segments themselves are this crate's existing
//! [`PathLine`]/[`PathCircle`], so the arc-length parametrization, the
//! `eqradius` translation/rotation balance and the rotational interpolation
//! all come from primitives already verified against the oracle by the `LIN`
//! and `CIRC` parity fixtures.
//!
//! # Scope
//!
//! Only `PathLength`/`Pos` are ported, the same scope limit
//! [`crate::pilz::path_line::PathLine`] and [`crate::pilz::path_circle::PathCircle`]
//! already document: [`crate::pilz::trajectory_functions::generate_joint_trajectory`]
//! samples a Cartesian path through
//! [`crate::pilz::trajectory_functions::CartesianPath`], which has exactly those
//! two operations. `Vel`/`Acc`/`Write`/`Clone`/`GetSegment`/
//! `GetCurrentSegmentLocation` have no caller here.
//!
//! `Path_Composite::LengthToS` is likewise absent, and that absence is
//! upstream's own: it unconditionally throws
//! `Error_MotionPlanning_Not_Applicable`, so a composite has no
//! length→arc-length map at all. `Path_RoundedComposite::Add` needs
//! `LengthToS` only on the two *line* segments it builds internally, which is
//! why [`PathLine::length_to_s`] exists and this type has no such method.
//!
//! # Deviations from upstream
//!
//! - **Segments are an enum, not `Path*`.** Upstream's `Path_Composite` holds
//!   `std::vector<std::pair<Path*, bool>>` — any `Path` subclass, with a
//!   per-entry ownership flag. A rounded composite only ever adds
//!   `Path_Line` and `Path_Circle`, so this module's private `Segment` enum
//!   names exactly those two; the
//!   ownership flag has no meaning without raw pointers. This makes
//!   "composite of composites" — which would re-open `LengthToS`'s
//!   `Not_Applicable` throw as a runtime failure — unrepresentable rather
//!   than merely unused.
//! - **No `Lookup` cache.** Upstream caches the last-resolved segment index
//!   and its `[start, end]` arc-length window in `mutable` members, so a
//!   monotone sampling sweep skips the linear scan. [`PathRoundedComposite::pos`]
//!   takes `&self` and does the scan every call: the cache is a pure
//!   performance device (it cannot change which segment a given `s` resolves
//!   to), and reproducing it would mean interior mutability for no
//!   behavioural gain.
//! - **Errors are [`Error::Construct`], not KDL exceptions.** Upstream throws
//!   `Error_MotionPlanning_Not_Feasible` carrying a numeric code.
//!   [`PathRoundedComposite`] instead returns a distinct message per rejected
//!   precondition, each naming the geometric condition that failed rather
//!   than a number: the six conditions are the ones the construction below
//!   needs, so they are stated in its own terms.
//!
//! # Why this file stays BSD-3-Clause
//!
//! `KDL::Path_RoundedComposite` (and the `Path_Composite` it derives from) is
//! LGPL-2.1-or-later (`third_party/orocos_kinematics_dynamics/`), heavier
//! copyleft than this workspace's BSD-3-Clause — the same situation
//! [`crate::pilz::path_line::PathLine`], [`crate::pilz::path_circle::PathCircle`] and
//! [`crate::pilz::velocity_profile_trap`] already resolved, and resolved the same
//! way. [`PathRoundedComposite`] is therefore not transcribed from
//! `orocos_kdl/src/path_roundedcomposite.cpp`: corner rounding is the
//! elementary tangent-circle construction, derived here from the interior
//! angle directly. At a corner with interior angle `theta` between two legs,
//! the circle of radius `r` tangent to both touches each leg a distance
//! `t = r / tan(theta / 2)` back from the vertex, and its center is that
//! tangency point displaced by `r` along the leg's inward in-plane normal —
//! which is `bc` with its `ab` component projected out. Everything else here
//! (accumulating segment ends, resolving a path parameter to a segment) is
//! bookkeeping with no upstream expression to share.
//!
//! What is reused from the LGPL source is *interface facts*, not expression —
//! named here by convention rather than by file:line: that a rounded
//! composite is fed via poses one at a time and closed by a separate call
//! (so the corner at pose `i` is only known once pose `i+1` arrives); the
//! `eqradius` convention balancing translational against rotational arc
//! length into one path parameter (already independently derived for
//! [`PathLine`], reused verbatim here since every segment is a
//! [`PathLine`]/[`PathCircle`]); the choice to reject rather than clamp a
//! radius that overruns a leg; and the `1e-7` degeneracy threshold named
//! below, which is a tolerance value rather than an expression and is matched
//! so this type accepts exactly the corner set upstream accepts.
//!
//! Equivalence with upstream is exercised end-to-end by
//! `tests/pilz_trajectory_polyline_parity.rs`, whose `panda_polyline`
//! fixture runs this type's output through the oracle's `POLYLINE`
//! generator; what that comparison cannot see — that the sampled joint
//! values lie on a *rounded* path at all rather than on some other path both
//! implementations agree about — is covered by
//! `tests/pilz_trajectory_polyline.rs`'s property tests. Neither compares
//! this type's own `pos`/`path_length` against upstream's directly, since
//! the oracle exposes no `Path_RoundedComposite` op.

use cspace_core::error::{Error, Result};
use cspace_core::geometry::Isometry3;

use crate::pilz::path_circle::{CircleGeometry, PathCircle};
use crate::pilz::path_line::PathLine;
use crate::pilz::velocity_profile::KDL_EPSILON;

/// One piece of a [`PathRoundedComposite`].
///
/// A rounded composite only ever builds these two kinds — see this module's
/// `# Deviations`.
#[derive(Debug, Clone, Copy, PartialEq)]
enum Segment {
    Line(PathLine),
    Circle(PathCircle),
}

impl Segment {
    fn path_length(&self) -> f64 {
        match self {
            Segment::Line(p) => p.path_length(),
            Segment::Circle(p) => p.path_length(),
        }
    }

    fn pos(&self, s: f64) -> Isometry3 {
        match self {
            Segment::Line(p) => p.pos(s),
            Segment::Circle(p) => p.pos(s),
        }
    }
}

/// Degeneracy threshold for a corner: leg lengths below it are treated as
/// zero, and interior angles within it of `0` or `PI` as doubling back or
/// running straight through.
///
/// # Not coarser than `KDL_EPSILON` — measured, corrected from an earlier draft
///
/// An earlier version of this comment claimed this constant was "three
/// orders of magnitude coarser" than [`KDL_EPSILON`] (`1e-6`), i.e. around
/// `1e-3`, on the theory that a corner within `ADD_EPSILON` of straight is
/// already too degenerate for [`PathRoundedComposite::add_corner`]'s inward
/// normal to matter. That claim was never true of the actual value below:
/// `1e-7` is one order of magnitude *finer* than `KDL_EPSILON`, not three
/// orders coarser. Had the claim been true, `add_corner`'s
/// `inward.norm() < KDL_EPSILON` branch would be unreachable dead code; being
/// false, that branch is live — confirmed by construction, not just by this
/// arithmetic: a corner with `theta = PI - 5e-7` passes this threshold
/// (`PI - theta = 5e-7 > ADD_EPSILON`) while `inward.norm() = theta.sin() ≈
/// 5e-7 < KDL_EPSILON`. See `add_corner`'s own comment for what that costs
/// pre-fix and how it's handled now.
///
/// This value is not raised to close that branch, because it is doing a
/// second job the branch's threshold has nothing to do with: it matches
/// upstream's own `eps = 1E-7` (`path_roundedcomposite.cpp:71`) for whether a
/// corner is accepted at all, so this type accepts exactly the corner set
/// upstream accepts (see this module's `# Why this file stays
/// BSD-3-Clause`). Raising it to close the normalize branch would silently
/// narrow that corner set below upstream's.
const ADD_EPSILON: f64 = 1e-7;

/// A polyline through a sequence of via poses whose interior corners are
/// rounded to a fixed radius.
///
/// Build with [`PathRoundedComposite::new`], feed via poses with
/// [`PathRoundedComposite::add`], then close with
/// [`PathRoundedComposite::finish`]. Upstream `KDL::Path_RoundedComposite`.
#[derive(Debug, Clone)]
pub struct PathRoundedComposite {
    segments: Vec<Segment>,
    /// Cumulative arc length at the end of each segment, so [`Self::pos`] can
    /// resolve a path parameter to a segment by scan.
    ends: Vec<f64>,
    path_length: f64,
    radius: f64,
    eqradius: f64,
    /// The two poses still awaiting a corner: no segment can be emitted for a
    /// vertex until the pose *after* it arrives, so `add` keeps a two-pose
    /// window and `fed` counts how many poses have entered it.
    base_start: Isometry3,
    base_via: Isometry3,
    fed: usize,
}

impl PathRoundedComposite {
    /// Starts an empty rounded composite with corner radius `radius` and
    /// translation/rotation balance `eqradius` (the same `eqradius`
    /// [`PathLine::new`] takes).
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] if `eqradius <= 0.0`: it divides the rotational
    /// arc length, so a non-positive value has no path parameter to define.
    pub fn new(radius: f64, eqradius: f64) -> Result<Self> {
        if eqradius <= 0.0 {
            return Err(Error::construct("eqradius must be positive"));
        }
        Ok(Self {
            segments: Vec::new(),
            ends: Vec::new(),
            path_length: 0.0,
            radius,
            eqradius,
            base_start: Isometry3::identity(),
            base_via: Isometry3::identity(),
            fed: 0,
        })
    }

    fn push(&mut self, segment: Segment) {
        self.path_length += segment.path_length();
        self.ends.push(self.path_length);
        self.segments.push(segment);
    }

    /// Feeds the next via pose.
    ///
    /// The first two calls only record poses; from the third on, each call
    /// emits the straight segment plus rounding arc for the corner the
    /// previous two poses and this one form.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] once a corner is being rounded and one of the
    /// tangent construction's preconditions fails, each with its own message:
    /// the waypoint arriving at the vertex coincides with it, the one leaving
    /// it does, the path doubles back at it, or the rounding arc would start
    /// before the arriving waypoint or end past the leaving one.
    pub fn add(&mut self, point: Isometry3) -> Result<()> {
        if self.fed == 0 {
            self.base_start = point;
        } else if self.fed == 1 {
            self.base_via = point;
        } else {
            self.add_corner(point)?;
        }
        self.fed += 1;
        Ok(())
    }

    /// Rounds the corner formed by the two pending poses and `point`, emitting
    /// the shortened incoming line and its tangent arc, then sliding the
    /// two-pose window forward.
    ///
    /// The construction is the tangent-circle one this module's
    /// `# Why this file stays BSD-3-Clause` section states: with `theta` the
    /// interior angle at the vertex, the tangency points sit
    /// `t = radius / tan(theta / 2)` back along each leg, and the center is
    /// the incoming tangency point displaced by `radius` along the inward
    /// in-plane normal.
    fn add_corner(&mut self, point: Isometry3) -> Result<()> {
        let incoming = self.base_via.translation.vector - self.base_start.translation.vector;
        let outgoing = point.translation.vector - self.base_via.translation.vector;
        let incoming_len = incoming.norm();
        let outgoing_len = outgoing.norm();
        if incoming_len < ADD_EPSILON {
            return Err(Error::construct(
                "no corner: the waypoint arriving at this vertex coincides with it",
            ));
        }
        if outgoing_len < ADD_EPSILON {
            return Err(Error::construct(
                "no corner: the waypoint leaving this vertex coincides with it",
            ));
        }

        // The legs as seen *from* the vertex: the interior angle is the angle
        // between them, so a straight-through vertex is `PI` and a doubling
        // back is `0`. `acos`'s argument is clamped because the quotient can
        // leave `[-1, 1]` by rounding alone at near-degenerate angles.
        let back = -incoming / incoming_len;
        let forth = outgoing / outgoing_len;
        let theta = back.dot(&forth).clamp(-1.0, 1.0).acos();
        if theta < ADD_EPSILON {
            return Err(Error::construct(
                "no corner: the path doubles back on itself at this vertex",
            ));
        }
        if (std::f64::consts::PI - theta) < ADD_EPSILON {
            // A straight-through vertex has no corner to round, so the whole
            // incoming leg is emitted and the window slides by one.
            self.push(Segment::Line(PathLine::new(
                &self.base_start,
                &self.base_via,
                self.eqradius,
            )));
            self.base_start = self.base_via;
            self.base_via = point;
            return Ok(());
        }

        // Tangent length: the distance from the vertex to each tangency point
        // of the inscribed circle of radius `self.radius`. `tan` is non-zero
        // because `theta` is now strictly inside `(0, PI)`.
        let tangent_len = self.radius / (theta / 2.0).tan();
        if tangent_len >= incoming_len {
            return Err(Error::construct(
                "rounding radius too large: its arc would start before the \
                 waypoint arriving at this vertex",
            ));
        }
        if tangent_len >= outgoing_len {
            return Err(Error::construct(
                "rounding radius too large: its arc would end past the \
                 waypoint leaving this vertex",
            ));
        }

        let incoming_line = PathLine::new(&self.base_start, &self.base_via, self.eqradius);
        let outgoing_line = PathLine::new(&self.base_via, &point, self.eqradius);
        let arc_start = incoming_line.pos(incoming_line.length_to_s(incoming_len - tangent_len));
        let arc_end = outgoing_line.pos(outgoing_line.length_to_s(tangent_len));

        // The inward in-plane normal to the incoming leg: `forth` with its
        // component along the leg projected out. It is perpendicular to the
        // leg, lies in the corner's plane, and points into the turn, so
        // stepping `radius` along it from the tangency point lands on the
        // center. Its norm is `sin(theta)` — mathematically non-zero for the
        // same reason `tan` above is (`theta` strictly inside `(0, PI)`), but
        // "non-zero" is not "not tiny": `ADD_EPSILON`'s doc comment works out
        // a concrete `theta` where `sin(theta) < KDL_EPSILON` despite passing
        // every guard above.
        //
        // # Deviation: reject rather than substitute KDL's `(1,0,0)`
        //
        // Upstream's analogous vector (`V_base_t` in
        // `path_roundedcomposite.cpp`) faces the identical gap and closes it
        // with `Vector::Normalize()`'s documented fallback: substitute the
        // base-frame unit X axis (`orocos_kdl/src/frames.cpp:147-156`). This file is not a
        // transcription of that function (see this module's `# Why this file
        // stays BSD-3-Clause` — corner rounding here is derived from the
        // interior angle directly, reusing only named "interface facts", and
        // this substitution value is not one of them), so matching that
        // fallback exactly is a choice, not a fidelity obligation, and it was
        // measured to be the wrong one: silently reusing `(1,0,0)` would
        // still build a `Circle` segment, just one whose center sits `radius`
        // away in an arbitrary world-frame direction unrelated to this
        // corner's actual geometry — a plausible-looking wrong answer, not a
        // failure a caller could detect.
        //
        // Before this fix, the branch left `inward` un-normalized instead
        // (neither upstream's substitution nor a real unit vector), which
        // measured out *worse* than either principled choice: for
        // `self.radius` small enough that `self.radius * inward.norm() <
        // KDL_EPSILON`, `PathCircle::new`'s own `radius < eps` guard happens
        // to catch it, but with a message about "circle radius" that
        // misnames the actual cause. For `self.radius` large enough that
        // product clears `KDL_EPSILON` — measured at `self.radius = 1e5`,
        // `theta = PI - 5e-7` — `PathCircle::new` raised no error at all and
        // silently returned a circle of radius `self.radius * inward.norm()
        // ≈ 0.05`, six orders of magnitude off the requested `1e5` and never
        // surfaced to the caller. An explicit rejection here, before `inward`
        // ever reaches `center`, replaces both outcomes with one named error
        // regardless of `self.radius`.
        //
        // # This crate now has three substitutions for one KDL contract
        //
        // Not resolved by this fix, named so it's on the record: this file
        // rejects; [`crate::pilz::path_line::kdl_normalize`] substitutes
        // `Vector3::zeros()` (its own doc argues, per call site, that no
        // current caller can observe the difference from KDL's `(1,0,0)`);
        // upstream substitutes `(1,0,0)`. Three spellings of one contract in
        // one crate.
        let along = -back;
        let inward = forth - along * along.dot(&forth);
        if inward.norm() < KDL_EPSILON {
            return Err(Error::construct(
                "rounding direction is underdetermined: this corner is too \
                 close to straight to determine which side to round toward",
            ));
        }
        let inward = inward.normalize();
        let center = arc_start.translation.vector + inward * self.radius;

        self.push(Segment::Line(PathLine::new(
            &self.base_start,
            &arc_start,
            self.eqradius,
        )));
        self.push(Segment::Circle(PathCircle::new(
            &arc_start,
            &arc_end,
            &CircleGeometry {
                center,
                radius: self.radius,
                // The arc sweeps the vertex's exterior angle, not its interior
                // one: turning by `PI - theta` is what carries the tangent
                // direction from the incoming leg onto the outgoing one.
                alpha: std::f64::consts::PI - theta,
                aux_point: arc_end.translation.vector,
            },
            self.eqradius,
            KDL_EPSILON,
        )?));

        self.base_start = arc_end;
        self.base_via = point;
        Ok(())
    }

    /// Emits the final straight segment, closing the composite.
    ///
    /// A composite fed exactly one pose emits a zero-length segment from that
    /// pose to itself: the second pose of the window is then still its
    /// default. That is upstream's behaviour, kept rather than tightened to a
    /// two-pose guard, so a caller that filters its way down to one waypoint
    /// gets the same degenerate path from both.
    pub fn finish(&mut self) {
        if self.fed >= 1 {
            self.push(Segment::Line(PathLine::new(
                &self.base_start,
                &self.base_via,
                self.eqradius,
            )));
        }
    }

    /// Total arc length of every emitted segment.
    pub fn path_length(&self) -> f64 {
        self.path_length
    }

    /// The pose at path parameter `s`.
    ///
    /// `s` outside `[0, path_length]` resolves to the first or last segment
    /// and is evaluated there, extrapolating rather than clamping — which is
    /// what upstream does too, its range `assert` being a no-op in a release
    /// build.
    pub fn pos(&self, s: f64) -> Isometry3 {
        let mut previous = 0.0;
        for (i, end) in self.ends.iter().enumerate() {
            if s <= *end || i == self.ends.len() - 1 {
                return self.segments[i].pos(s - previous);
            }
            previous = *end;
        }
        // Only reachable on an empty composite -- one that was never fed a
        // pose, so it has no segment to evaluate. Upstream indexes its empty
        // segment vector here instead.
        Isometry3::identity()
    }

    /// How many segments the composite holds.
    pub fn segment_count(&self) -> usize {
        self.segments.len()
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use cspace_core::geometry::{UnitQuaternion, Vector3};

    use super::*;

    fn pose(x: f64, y: f64, z: f64) -> Isometry3 {
        Isometry3::from_parts(Vector3::new(x, y, z).into(), UnitQuaternion::identity())
    }

    /// Feeds `points` through the full `new`/`add`/`finish` cycle.
    fn build(radius: f64, points: &[Isometry3]) -> Result<PathRoundedComposite> {
        let mut path = PathRoundedComposite::new(radius, 1.0)?;
        for p in points {
            path.add(*p)?;
        }
        path.finish();
        Ok(path)
    }

    // -- new: the one constructor rejection --

    #[test]
    fn new_rejects_a_non_positive_eqradius() {
        let err = PathRoundedComposite::new(0.1, 0.0).unwrap_err();
        assert!(
            err.to_string().contains("eqradius must be positive"),
            "{err}"
        );
    }

    // -- add: one case per rejected precondition. Each needle below is a
    // phrase only its own guard emits, so a test cannot pass on a sibling
    // guard firing instead -- proven by neutralizing each guard in turn and
    // confirming exactly its own case goes red. --

    #[test]
    fn add_rejects_a_zero_length_incoming_segment() {
        let err = build(
            0.1,
            &[
                pose(0.0, 0.0, 0.0),
                pose(0.0, 0.0, 0.0),
                pose(1.0, 0.0, 0.0),
            ],
        )
        .unwrap_err();
        assert!(
            err.to_string()
                .contains("arriving at this vertex coincides"),
            "{err}"
        );
    }

    #[test]
    fn add_rejects_a_zero_length_outgoing_segment() {
        let err = build(
            0.1,
            &[
                pose(0.0, 0.0, 0.0),
                pose(1.0, 0.0, 0.0),
                pose(1.0, 0.0, 0.0),
            ],
        )
        .unwrap_err();
        assert!(
            err.to_string().contains("leaving this vertex coincides"),
            "{err}"
        );
    }

    #[test]
    fn add_rejects_a_reversing_corner() {
        let err = build(
            0.1,
            &[
                pose(0.0, 0.0, 0.0),
                pose(1.0, 0.0, 0.0),
                pose(0.0, 0.0, 0.0),
            ],
        )
        .unwrap_err();
        assert!(err.to_string().contains("doubles back on itself"), "{err}");
    }

    #[test]
    fn add_rejects_a_radius_too_large_for_the_incoming_segment() {
        // Right-angle corner, so `d == radius`: 0.5 does not fit in the
        // 0.4-long incoming segment but does fit in the 10-long outgoing one.
        let err = build(
            0.5,
            &[
                pose(0.0, 0.0, 0.0),
                pose(0.4, 0.0, 0.0),
                pose(0.4, 10.0, 0.0),
            ],
        )
        .unwrap_err();
        assert!(err.to_string().contains("would start before"), "{err}");
    }

    #[test]
    fn add_rejects_a_radius_too_large_for_the_outgoing_segment() {
        let err = build(
            0.5,
            &[
                pose(0.0, 0.0, 0.0),
                pose(10.0, 0.0, 0.0),
                pose(10.0, 0.4, 0.0),
            ],
        )
        .unwrap_err();
        assert!(err.to_string().contains("would end past"), "{err}");
    }

    /// `theta = PI - 5e-7` clears `ADD_EPSILON` (`1e-7`) so it is not
    /// rejected as straight-through, but `inward.norm() = theta.sin() ≈
    /// 5e-7` is still under `KDL_EPSILON` (`1e-6`) — the live window
    /// `ADD_EPSILON`'s doc comment works out. `radius = 1e5` is chosen to
    /// demonstrate the worse of the two pre-fix outcomes: at this radius,
    /// `self.radius * inward.norm() ≈ 0.05` clears `PathCircle::new`'s own
    /// `eps` guard, so pre-fix this returned `Ok` with a circle whose actual
    /// radius was `~0.05`, six orders of magnitude off the requested `1e5`,
    /// rather than any error at all. See `add_corner`'s comment.
    #[test]
    fn add_rejects_a_near_straight_corner_whose_inward_normal_underflows() {
        let theta = std::f64::consts::PI - 5e-7;
        let err = build(
            1e5,
            &[
                pose(1.0, 0.0, 0.0),
                pose(0.0, 0.0, 0.0),
                pose(theta.cos(), theta.sin(), 0.0),
            ],
        )
        .unwrap_err();
        assert!(
            err.to_string()
                .contains("rounding direction is underdetermined"),
            "{err}"
        );
    }

    /// Demonstrated opposite: an ordinary near-straight corner just outside
    /// the underflow window (`theta` a full radian short of `PI`, nowhere
    /// near either epsilon) still rounds normally at the same radius.
    #[test]
    fn add_still_rounds_an_ordinary_near_straight_corner() {
        let theta = std::f64::consts::PI - 1.0;
        let path = build(
            0.1,
            &[
                pose(1.0, 0.0, 0.0),
                pose(0.0, 0.0, 0.0),
                pose(theta.cos(), theta.sin(), 0.0),
            ],
        )
        .unwrap();
        assert_eq!(path.segment_count(), 3);
    }

    // -- the two shapes `add` can take when it accepts --

    #[test]
    fn parallel_segments_are_not_rounded() {
        // Three colinear points: `alpha == 0`, so the corner branch emits one
        // whole line and `finish` emits the rest -- two segments, no arc.
        let path = build(
            0.1,
            &[
                pose(0.0, 0.0, 0.0),
                pose(1.0, 0.0, 0.0),
                pose(2.0, 0.0, 0.0),
            ],
        )
        .unwrap();
        assert_eq!(path.segment_count(), 2);
        assert_relative_eq!(path.path_length(), 2.0, epsilon = 1e-12);
    }

    #[test]
    fn a_rounded_corner_emits_a_line_an_arc_and_the_closing_line() {
        let path = build(
            0.1,
            &[
                pose(0.0, 0.0, 0.0),
                pose(1.0, 0.0, 0.0),
                pose(1.0, 1.0, 0.0),
            ],
        )
        .unwrap();
        assert_eq!(path.segment_count(), 3);
        // A right-angle corner rounded at r=0.1 cuts 0.1 off each leg and
        // replaces it with a quarter arc of length `PI * r / 2`.
        let expected = 0.9 + std::f64::consts::FRAC_PI_2 * 0.1 + 0.9;
        assert_relative_eq!(path.path_length(), expected, epsilon = 1e-9);
    }

    // -- pos: the composite's own job is routing `s` to a segment --

    #[test]
    fn pos_at_the_ends_reproduces_the_first_and_last_via_pose() {
        let path = build(
            0.1,
            &[
                pose(0.0, 0.0, 0.0),
                pose(1.0, 0.0, 0.0),
                pose(1.0, 1.0, 0.0),
            ],
        )
        .unwrap();

        let start = path.pos(0.0);
        assert_relative_eq!(start.translation.vector, Vector3::zeros(), epsilon = 1e-12);
        let end = path.pos(path.path_length());
        assert_relative_eq!(
            end.translation.vector,
            Vector3::new(1.0, 1.0, 0.0),
            epsilon = 1e-9
        );
    }

    #[test]
    fn pos_is_continuous_across_the_segment_boundaries() {
        let path = build(
            0.1,
            &[
                pose(0.0, 0.0, 0.0),
                pose(1.0, 0.0, 0.0),
                pose(1.0, 1.0, 0.0),
            ],
        )
        .unwrap();

        // Sampling either side of each internal boundary is what distinguishes
        // "the corner was rounded" from "the segments were concatenated with a
        // jump": a mis-placed arc center moves one side and not the other.
        for boundary in [0.9, 0.9 + std::f64::consts::FRAC_PI_2 * 0.1] {
            let before = path.pos(boundary - 1e-7).translation.vector;
            let after = path.pos(boundary + 1e-7).translation.vector;
            assert_relative_eq!(before, after, epsilon = 1e-6);
        }
    }

    #[test]
    fn the_arc_stays_at_the_rounding_radius_from_its_center() {
        let path = build(
            0.1,
            &[
                pose(0.0, 0.0, 0.0),
                pose(1.0, 0.0, 0.0),
                pose(1.0, 1.0, 0.0),
            ],
        )
        .unwrap();

        // Tangency is the property the rounding exists to provide, and it is
        // not implied by the endpoints matching: the center of a right-angle
        // corner rounded at r=0.1 is at (0.9, 0.1).
        let center = Vector3::new(0.9, 0.1, 0.0);
        let arc_end = 0.9 + std::f64::consts::FRAC_PI_2 * 0.1;
        for i in 0..=10 {
            let s = 0.9 + (arc_end - 0.9) * f64::from(i) / 10.0;
            let p = path.pos(s).translation.vector;
            assert_relative_eq!((p - center).norm(), 0.1, epsilon = 1e-9);
        }
    }
}
