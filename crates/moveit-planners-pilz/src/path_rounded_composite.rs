// Copyright (c) 2007, Ruben Smits
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from orocos_kinematics_dynamics 1.5.1 (third_party/orocos_kinematics_dynamics):
//   orocos_kdl/src/path_composite.hpp
//   orocos_kdl/src/path_composite.cpp
//   orocos_kdl/src/path_roundedcomposite.hpp
//   orocos_kdl/src/path_roundedcomposite.cpp

//! Corner-rounded polyline paths ([`PathRoundedComposite`]), the KDL
//! primitive `POLYLINE` motions are built on.
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
//! [`crate::path_line::PathLine`] and [`crate::path_circle::PathCircle`]
//! already document: [`crate::trajectory_functions::generate_joint_trajectory`]
//! samples a Cartesian path through
//! [`crate::trajectory_functions::CartesianPath`], which has exactly those
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
//!   `Error_MotionPlanning_Not_Feasible` with the numeric codes `1`..`6`
//!   (reported as `3001`..`3006` by `GetType`, which adds `3000`). Each
//!   message below names the code it replaces, because
//!   [`crate::trajectory_generator_polyline`] maps those codes to distinct
//!   user-facing messages and needs to keep telling them apart.

use moveit_error::{Error, Result};
use moveit_geometry::Isometry3;

use crate::path_circle::{CircleGeometry, PathCircle};
use crate::path_line::PathLine;
use crate::velocity_profile::KDL_EPSILON;

/// One piece of a [`PathRoundedComposite`].
///
/// Upstream stores `Path*`; a rounded composite only ever builds these two
/// kinds — see this module's `# Deviations`.
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

/// The `eps` upstream's `Path_RoundedComposite::Add` compares its two segment
/// lengths and its corner angle against. Hard-coded as a local `double eps =
/// 1E-7;` there, *not* `KDL::epsilon` — the two differ by three orders of
/// magnitude, so this is kept as its own named constant rather than folded
/// into [`KDL_EPSILON`].
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
    /// Cumulative arc length at the end of each segment — upstream's `dv`.
    ends: Vec<f64>,
    path_length: f64,
    radius: f64,
    eqradius: f64,
    /// Upstream's `F_base_start`/`F_base_via`/`nrofpoints`: the two poses
    /// still awaiting a corner, and how many have been fed in total.
    base_start: Isometry3,
    base_via: Isometry3,
    nrofpoints: usize,
}

impl PathRoundedComposite {
    /// Starts an empty rounded composite with corner radius `radius` and
    /// translation/rotation balance `eqradius` (the same `eqradius`
    /// [`PathLine::new`] takes).
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] if `eqradius <= 0.0` (upstream
    /// `Error_MotionPlanning_Not_Feasible(1)`).
    pub fn new(radius: f64, eqradius: f64) -> Result<Self> {
        if eqradius <= 0.0 {
            return Err(Error::construct(
                "eqradius must be positive (upstream Error_MotionPlanning_Not_Feasible code 1)",
            ));
        }
        Ok(Self {
            segments: Vec::new(),
            ends: Vec::new(),
            path_length: 0.0,
            radius,
            eqradius,
            base_start: Isometry3::identity(),
            base_via: Isometry3::identity(),
            nrofpoints: 0,
        })
    }

    fn push(&mut self, segment: Segment) {
        self.path_length += segment.path_length();
        self.ends.push(self.path_length);
        self.segments.push(segment);
    }

    /// Feeds the next via pose. Upstream `Add`.
    ///
    /// The first two calls only record poses; from the third on, each call
    /// emits the straight segment plus rounding arc for the corner the
    /// previous two poses and this one form.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] for each of upstream's
    /// `Error_MotionPlanning_Not_Feasible` codes `2`..`6`: a zero-length
    /// incoming or outgoing segment (`2`, `3`), a reversing corner whose
    /// interior angle is zero (`4`), or a rounding radius that does not fit
    /// in the incoming or outgoing segment (`5`, `6`).
    pub fn add(&mut self, point: Isometry3) -> Result<()> {
        if self.nrofpoints == 0 {
            self.base_start = point;
        } else if self.nrofpoints == 1 {
            self.base_via = point;
        } else {
            self.add_corner(point)?;
        }
        self.nrofpoints += 1;
        Ok(())
    }

    /// The `nrofpoints >= 2` branch of upstream's `Add`, split out only to
    /// keep [`PathRoundedComposite::add`]'s three-way dispatch readable.
    fn add_corner(&mut self, point: Isometry3) -> Result<()> {
        let ab = self.base_via.translation.vector - self.base_start.translation.vector;
        let bc = point.translation.vector - self.base_via.translation.vector;
        let abdist = ab.norm();
        let bcdist = bc.norm();
        if abdist < ADD_EPSILON {
            return Err(Error::construct(
                "zero distance between two consecutive waypoints \
                 (upstream Error_MotionPlanning_Not_Feasible code 2)",
            ));
        }
        if bcdist < ADD_EPSILON {
            return Err(Error::construct(
                "zero distance between two consecutive waypoints \
                 (upstream Error_MotionPlanning_Not_Feasible code 3)",
            ));
        }
        // `ab`/`bc` point *along* travel, so a straight-through corner has
        // `alpha == 0` and a full reversal has `alpha == PI`. Clamped before
        // `acos` exactly as upstream does: the quotient can leave `[-1, 1]`
        // by a rounding error alone.
        let alpha = (ab.dot(&bc) / abdist / bcdist).clamp(-1.0, 1.0).acos();
        if (std::f64::consts::PI - alpha) < ADD_EPSILON {
            return Err(Error::construct(
                "the path reverses direction at a waypoint \
                 (upstream Error_MotionPlanning_Not_Feasible code 4)",
            ));
        }
        if alpha < ADD_EPSILON {
            // Parallel segments: no corner to round, so the incoming segment
            // is emitted whole and the window shifts by one.
            self.push(Segment::Line(PathLine::new(
                &self.base_start,
                &self.base_via,
                self.eqradius,
            )));
            self.base_start = self.base_via;
            self.base_via = point;
            return Ok(());
        }

        // How far back from the corner the arc must start for a circle of
        // `radius` to be tangent to both segments. `tan` cannot return zero
        // here: `alpha` is in `(ADD_EPSILON, PI - ADD_EPSILON)`.
        let d = self.radius / ((std::f64::consts::PI - alpha) / 2.0).tan();
        if d >= abdist {
            return Err(Error::construct(
                "the rounding radius does not fit in the incoming segment \
                 (upstream Error_MotionPlanning_Not_Feasible code 5)",
            ));
        }
        if d >= bcdist {
            return Err(Error::construct(
                "the rounding radius does not fit in the outgoing segment \
                 (upstream Error_MotionPlanning_Not_Feasible code 6)",
            ));
        }

        let line1 = PathLine::new(&self.base_start, &self.base_via, self.eqradius);
        let line2 = PathLine::new(&self.base_via, &point, self.eqradius);
        let circle_start = line1.pos(line1.length_to_s(abdist - d));
        let circle_end = line2.pos(line2.length_to_s(d));

        // The in-plane normal pointing from the corner towards the arc's
        // center: `ab x (ab x bc)` is perpendicular to `ab` and lies in the
        // corner's plane, and its sign puts it on the inside of the corner.
        let v_base_t = ab.cross(&ab.cross(&bc));
        let v_base_t = if v_base_t.norm() < KDL_EPSILON {
            v_base_t
        } else {
            v_base_t.normalize()
        };
        let center = circle_start.translation.vector - v_base_t * self.radius;

        self.push(Segment::Line(PathLine::new(
            &self.base_start,
            &circle_start,
            self.eqradius,
        )));
        self.push(Segment::Circle(PathCircle::new(
            &circle_start,
            &circle_end,
            &CircleGeometry {
                center,
                radius: self.radius,
                alpha,
                aux_point: circle_end.translation.vector,
            },
            self.eqradius,
            KDL_EPSILON,
        )?));

        self.base_start = circle_end;
        self.base_via = point;
        Ok(())
    }

    /// Emits the final straight segment. Upstream `Finish`.
    ///
    /// A composite fed exactly one pose emits a zero-length segment from that
    /// pose to itself, because upstream's guard is `nrofpoints >= 1` and
    /// `F_base_via` is then still its default — reproduced rather than
    /// tightened to `>= 2`.
    pub fn finish(&mut self) {
        if self.nrofpoints >= 1 {
            self.push(Segment::Line(PathLine::new(
                &self.base_start,
                &self.base_via,
                self.eqradius,
            )));
        }
    }

    /// Upstream `PathLength`.
    pub fn path_length(&self) -> f64 {
        self.path_length
    }

    /// Upstream `Pos`.
    ///
    /// `s` outside `[0, path_length]` resolves to the first or last segment
    /// and is evaluated there — upstream `Lookup` `assert`s the range (a
    /// no-op in a release build) and then falls through to the last segment
    /// for any `s` past the end, which is what this reproduces.
    pub fn pos(&self, s: f64) -> Isometry3 {
        let mut previous = 0.0;
        for (i, end) in self.ends.iter().enumerate() {
            if s <= *end || i == self.ends.len() - 1 {
                return self.segments[i].pos(s - previous);
            }
            previous = *end;
        }
        // Only reachable on an empty composite, where upstream's `Lookup`
        // returns `0` and then indexes `gv[0]` out of bounds.
        Isometry3::identity()
    }

    /// How many segments the composite holds. Upstream `GetNrOfSegments`.
    pub fn segment_count(&self) -> usize {
        self.segments.len()
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;
    use moveit_geometry::{UnitQuaternion, Vector3};

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
        assert!(err.to_string().contains("code 1"), "{err}");
    }

    // -- add: one case per `Not_Feasible` code, so a test naming a code
    // cannot pass on a different code's guard --

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
        assert!(err.to_string().contains("code 2"), "{err}");
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
        assert!(err.to_string().contains("code 3"), "{err}");
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
        assert!(err.to_string().contains("code 4"), "{err}");
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
        assert!(err.to_string().contains("code 5"), "{err}");
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
        assert!(err.to_string().contains("code 6"), "{err}");
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
