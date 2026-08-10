// Copyright (c) 2017, Southwest Research Institute
// Copyright (c) 2013, John Schulman
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-2-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection_bullet/include/moveit/collision_detection_bullet/bullet_integration/bullet_utils.hpp

//! `addCastSingleResult` (`bullet_utils.hpp:419-517`) -- turning one Bullet
//! contact against the swept hull into a MoveIt contact, and in particular
//! into a `percent_interpolation`.
//!
//! # What a continuous check actually adds
//!
//! The boolean and the depth a swept check reports are the same kind of answer
//! a discrete check gives, just about a bigger shape. `percent_interpolation`
//! is the new information: *where between the two states* the swept hull first
//! touches, as a fraction. A port that got the boolean right and this number
//! wrong would report a collision at the wrong point of the motion, and every
//! caller that trims a trajectory at the first collision would trim it in the
//! wrong place.
//!
//! # How the fraction is recovered from a shape that was never swept
//!
//! [`crate::cast_hull_shape::CastHullShape`] answers support queries as the
//! max of two placements, so by the time GJK has a contact, which placement won
//! is gone. It is recovered by asking the *inner* shape for its support point
//! under each placement's own frame and comparing:
//!
//! - if the first placement's support along the contact normal beats the
//!   second's by more than [`BULLET_SUPPORT_FUNC_TOLERANCE`], the contact
//!   belongs to the start pose and the fraction is 0;
//! - the reverse gives 1;
//! - otherwise both placements are equally deep and the fraction is read off
//!   the *distances* from the contact point to the two support points.
//!
//! Note that the first two arms compare support values and the third compares
//! lengths. They are different quantities in different units, so there is no
//! continuity across the boundary: a contact that moves from just outside the
//! tolerance to just inside it jumps from an endpoint to whatever ratio the
//! lengths give. Upstream marks the section `TODO: this section is potentially
//! problematic. think hard about the math` (`:491`); this port reproduces it
//! rather than improving it, because the oracle it is checked against is
//! upstream.

use cspace_bullet::linear_math::{Scalar, Transform, Vec3};
use cspace_bullet::manifold::ManifoldPoint;

use crate::cast_hull_shape::{
    BULLET_LENGTH_TOLERANCE, BULLET_SUPPORT_FUNC_TOLERANCE, CastHullShape, get_average_support,
};

/// What `addCastSingleResult` leaves in the stored `collision_detection::Contact`.
///
/// Named as a return value rather than written into a caller's contact because
/// the fields upstream writes are split across two objects -- some into the
/// local `contact` before `processResult` copies it out, some into the stored
/// `col` afterwards -- and one of those writes lands on the wrong one. See
/// [`cast_single_result`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CastContact {
    /// `contact.normal`, after the `col->normal *= -1` that a cast-first pair
    /// applies.
    pub normal: Vec3,
    /// `contact.pos`.
    pub pos: Vec3,
    /// `contact.depth`.
    pub depth: Scalar,
    /// `col->percent_interpolation`: 0 at the start pose, 1 at the end pose.
    pub percent_interpolation: Scalar,
    /// Whether the two bodies' names and types are exchanged, which upstream
    /// does with two `std::swap`s so that the non-cast object is reported
    /// first.
    ///
    /// Returned rather than applied because the names and types are the
    /// caller's, not this crate's.
    pub swap_bodies: bool,
}

/// `addCastSingleResult` (`bullet_utils.hpp:419-517`), less the `processResult`
/// accumulation that decides whether a contact is stored at all.
///
/// `cast_shape_is_first` is `cd0->m_collisionFilterGroup == KinematicFilter`
/// (`:454`). Upstream asserts the two are never *both* kinematic (`:451-452`),
/// which `isOnlyKinematic` guarantees by rejecting such a pair before the
/// narrow phase runs; so this flag names which side is swept, and exactly one
/// side is.
///
/// `cast_world_transform` is `first_col_obj_wrap->getWorldTransform()` (`:472`)
/// -- the *cast* object's world transform, which is `tf1`, the first of the two
/// poses. The second pose is not passed in because it is not stored anywhere:
/// it exists only as `cast_shape.shape_transform`, the delta.
///
/// # Two writes upstream makes that do not reach the result
///
/// Reproduced as absences, not carried:
///
/// - `contact.pos = convertBtToEigen(cp.m_positionWorldOnB)` (`:463`) assigns
///   to the *local* `contact`, which `processResult` copied into the result
///   several lines earlier (`:444`). The stored `pos` therefore stays
///   `m_positionWorldOnA` even for a cast-first pair, and [`CastContact::pos`]
///   is that value on both branches.
/// - `std::swap(col->nearest_points[0], col->nearest_points[1])` (`:462`) swaps
///   two `Eigen::Vector3d`s that nothing on this path ever assigns.
///   `collision_detection::Contact::nearest_points` has no default initialiser
///   (`collision_common.hpp:105`), so on the continuous path its value is
///   whatever the stack held. This port carries no `nearest_points` for that
///   reason: there is no value to reproduce.
///
/// `localsup0` and `localsup1` (`:480`, `:484`) are likewise written by
/// `getAverageSupport` and never read -- `sup0`/`sup1` are recomputed from the
/// world-space points -- so only the point half of each call's result is bound
/// here.
#[must_use]
pub fn cast_single_result(
    point: &ManifoldPoint,
    cast_shape_is_first: bool,
    cast_shape: &CastHullShape,
    cast_world_transform: Transform,
) -> CastContact {
    let normal = -1.0 * point.normal_world_on_b;

    let normal_world_from_cast =
        if cast_shape_is_first { -1.0 } else { 1.0 } * point.normal_world_on_b;

    let tf_world0 = cast_world_transform;
    let tf_world1 = cast_world_transform * cast_shape.shape_transform;

    // `normal_world_from_cast * tf_worldN.getBasis()` -- `btVector3 *
    // btMatrix3x3` is the transpose product (`btMatrix3x3.h:1275-1280`), i.e.
    // the normal taken *into* each pose's local frame.
    let normal_local0 = tf_world0.basis.transposed_mul_vec(normal_world_from_cast);
    let normal_local1 = tf_world1.basis.transposed_mul_vec(normal_world_from_cast);

    let (_localsup0, pt_local0) = get_average_support(cast_shape.shape.as_ref(), normal_local0);
    let pt_world0 = tf_world0.transform_point(pt_local0);
    let (_localsup1, pt_local1) = get_average_support(cast_shape.shape.as_ref(), normal_local1);
    let pt_world1 = tf_world1.transform_point(pt_local1);

    let sup0 = normal_world_from_cast.dot(pt_world0);
    let sup1 = normal_world_from_cast.dot(pt_world1);

    let percent_interpolation = if sup0 - sup1 > BULLET_SUPPORT_FUNC_TOLERANCE {
        0.0
    } else if sup1 - sup0 > BULLET_SUPPORT_FUNC_TOLERANCE {
        1.0
    } else {
        let pt_on_cast = if cast_shape_is_first {
            point.position_world_on_a
        } else {
            point.position_world_on_b
        };
        let l0c = (pt_on_cast - pt_world0).length();
        let l1c = (pt_on_cast - pt_world1).length();

        if l0c + l1c < BULLET_LENGTH_TOLERANCE {
            0.5
        } else {
            l0c / (l0c + l1c)
        }
    };

    CastContact {
        normal: if cast_shape_is_first {
            normal * -1.0
        } else {
            normal
        },
        pos: point.position_world_on_a,
        depth: point.distance1,
        percent_interpolation,
        swap_bodies: cast_shape_is_first,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arc_probe::arc_probe_shapes;
    use crate::cast_hull_shape::ArcConvexShape;
    use cspace_bullet::probe_fixture::{IDENTITY, at, diff, rot60_at, row};
    use std::sync::Arc;

    /// The `pctinterp_*` rows of `tools/bullet-epa-reference/build.sh`'s
    /// stdout, produced by `moveit_cast.hpp`'s `castPercentInterpolation` --
    /// the tail of `addCastSingleResult` transcribed with the `ContactTestData`
    /// bookkeeping removed. That file's header lists exactly what was removed.
    const BULLET_REFERENCE: &str = "\
pctinterp_end|1
pctinterp_start|0
pctinterp_boundary_out|1
pctinterp_boundary_in|0.399995238
pctinterp_ratio_quarter|0.25
pctinterp_zero_delta|0.5
pctinterp_rot_delta|1
pctinterp_world_rot_delta|0
pctinterp_sphere_ratio|0.382782221
pctinterp_hull_ratio|0.292016059
pctinterp_end_cast_first|1
pctinterp_ratio_quarter_cast_first|0.25
";

    fn manifold_point(
        normal_world_on_b: Vec3,
        pos_a: Vec3,
        pos_b: Vec3,
        distance: Scalar,
    ) -> ManifoldPoint {
        let mut point = ManifoldPoint::new(Vec3::zero(), Vec3::zero(), normal_world_on_b, distance);
        point.position_world_on_a = pos_a;
        point.position_world_on_b = pos_b;
        point
    }

    #[test]
    fn bullet_reference_percent_interpolation() {
        let (unit_box, _, _, sphere, _, _, hull) = arc_probe_shapes();
        let nx = Vec3::new(1.0, 0.0, 0.0);
        let p_mid = Vec3::new(0.5, 0.25, 0.0);
        let p_face = Vec3::new(0.5, 0.0, 0.0);
        let across = at(0.0, 1.0, 0.0);
        let world_rot = rot60_at(0.1, 0.2, -0.1);

        /// One `pctinterp` call site of `probe.cpp`, field for field.
        struct Case<'a> {
            name: &'a str,
            shape: ArcConvexShape,
            t01: Transform,
            world: Transform,
            normal: Vec3,
            pos_a: Vec3,
            pos_b: Vec3,
            cast_first: bool,
        }

        let case = |name, shape, t01, world, normal, pos_a, pos_b, cast_first| Case {
            name,
            shape,
            t01,
            world,
            normal,
            pos_a,
            pos_b,
            cast_first,
        };
        let cases = [
            case(
                "end",
                unit_box.clone(),
                at(1.0, 0.0, 0.0),
                IDENTITY,
                nx,
                p_face,
                p_face,
                false,
            ),
            case(
                "start",
                unit_box.clone(),
                at(1.0, 0.0, 0.0),
                IDENTITY,
                -nx,
                p_face,
                p_face,
                false,
            ),
            case(
                "boundary_out",
                unit_box.clone(),
                at(0.02, 0.0, 0.0),
                IDENTITY,
                nx,
                p_face,
                p_face,
                false,
            ),
            case(
                "boundary_in",
                unit_box.clone(),
                at(0.005, 0.0, 0.0),
                IDENTITY,
                nx,
                Vec3::new(0.502, 0.0, 0.0),
                Vec3::new(0.502, 0.0, 0.0),
                false,
            ),
            case(
                "ratio_quarter",
                unit_box.clone(),
                across,
                IDENTITY,
                nx,
                p_mid,
                p_mid,
                false,
            ),
            case(
                "zero_delta",
                unit_box.clone(),
                IDENTITY,
                IDENTITY,
                nx,
                p_face,
                p_face,
                false,
            ),
            case(
                "rot_delta",
                unit_box.clone(),
                rot60_at(0.0, 1.0, 0.0),
                IDENTITY,
                nx,
                p_mid,
                p_mid,
                false,
            ),
            case(
                "world_rot_delta",
                unit_box.clone(),
                across,
                world_rot,
                nx,
                p_mid,
                p_mid,
                false,
            ),
            case(
                "sphere_ratio",
                sphere.clone(),
                across,
                IDENTITY,
                nx,
                p_mid,
                p_mid,
                false,
            ),
            case(
                "hull_ratio",
                hull.clone(),
                across,
                IDENTITY,
                nx,
                p_mid,
                p_mid,
                false,
            ),
            case(
                "end_cast_first",
                unit_box.clone(),
                at(1.0, 0.0, 0.0),
                IDENTITY,
                -nx,
                p_face,
                p_face,
                true,
            ),
            case(
                "ratio_quarter_cast_first",
                unit_box.clone(),
                across,
                IDENTITY,
                -nx,
                p_mid,
                Vec3::new(0.5, 0.8, 0.0),
                true,
            ),
        ];

        let mut bad = Vec::new();
        let mut covered = Vec::new();
        for c in cases {
            let cast = CastHullShape::new(Arc::clone(&c.shape), c.t01);
            let point = manifold_point(c.normal, c.pos_a, c.pos_b, -0.01);
            let got = cast_single_result(&point, c.cast_first, &cast, c.world);

            let name = c.name;
            let full = format!("pctinterp_{name}");
            covered.push(full.clone());
            let f = row(BULLET_REFERENCE, &full, 2);
            diff(
                &mut bad,
                name,
                "percent_interpolation",
                got.percent_interpolation,
                f[1].parse().unwrap(),
            );
        }
        assert!(bad.is_empty(), "{}", bad.join("\n"));

        let mut want: Vec<String> = BULLET_REFERENCE
            .lines()
            .filter_map(|l| l.split('|').next())
            .map(str::to_string)
            .collect();
        want.sort();
        covered.sort();
        assert_eq!(
            covered, want,
            "the cases and BULLET_REFERENCE disagree on which rows exist"
        );
    }

    /// A cast-first pair negates the reported normal, and only the reported
    /// normal: the position stays `m_positionWorldOnA` because upstream's write
    /// of `m_positionWorldOnB` lands on a copy the result no longer refers to.
    ///
    /// The fixture rows above cannot see either: `castPercentInterpolation` is
    /// the *tail* of `addCastSingleResult`, and both of these are decided in
    /// the part that writes into `ContactTestData`.
    #[test]
    fn a_cast_first_pair_negates_the_normal_and_keeps_position_world_on_a() {
        let (unit_box, ..) = arc_probe_shapes();
        let cast = CastHullShape::new(unit_box, at(1.0, 0.0, 0.0));
        let normal = Vec3::new(0.0, 0.0, 1.0);
        let pos_a = Vec3::new(1.0, 2.0, 3.0);
        let pos_b = Vec3::new(4.0, 5.0, 6.0);
        let point = manifold_point(normal, pos_a, pos_b, -0.25);

        let cast_second = cast_single_result(&point, false, &cast, IDENTITY);
        let cast_first = cast_single_result(&point, true, &cast, IDENTITY);

        assert_eq!(cast_second.normal, -normal);
        assert_eq!(cast_first.normal, normal);
        assert_eq!(cast_second.pos, pos_a);
        assert_eq!(
            cast_first.pos, pos_a,
            "the m_positionWorldOnB write at bullet_utils.hpp:463 never reaches the result"
        );
        assert!(cast_first.swap_bodies);
        assert!(!cast_second.swap_bodies);
        assert_eq!(cast_second.depth, -0.25);
    }
}
