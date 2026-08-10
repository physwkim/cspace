// Copyright (c) 2017, Southwest Research Institute
// Copyright (c) 2013, John Schulman
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-2-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection_bullet/include/moveit/collision_detection_bullet/bullet_integration/bullet_utils.hpp

//! `CastHullShape` and `getAverageSupport` -- the swept shape the continuous
//! check runs its narrow phase against, and the support-averaging the contact
//! conversion reads back off it.
//!
//! # The sweep is never built
//!
//! A swept convex hull between two poses is not computed here and never exists
//! as geometry. [`CastHullShape`] holds the original shape and the delta
//! transform between the two poses, and answers a support query by taking the
//! *larger of the two poses' support points along the query direction*. That
//! is exactly the support function of the convex hull of the two placements,
//! which is all GJK and EPA ever ask a shape for -- so the narrow phase runs
//! against the swept volume without anyone having enumerated it.
//!
//! The consequence for the port: [`CastHullShape`] is a
//! [`ConvexShape`] like any other, and every algorithm in `cspace_bullet`
//! drives it unchanged.
//!
//! # Why the averaging exists
//!
//! GJK returns *a* witness point on each shape. On a face-on contact between
//! two boxes that witness is some corner of the contact face -- whichever the
//! simplex happened to end on -- and which corner it is says nothing about the
//! geometry. [`get_average_support`] instead returns the centroid of every
//! vertex within [`BULLET_EPSILON`] of the maximum support, so a face-on
//! direction gives the face centre and an edge-on direction the edge midpoint.
//!
//! That is what makes `percent_interpolation` computable: the contact
//! conversion asks for the support point at both ends of the sweep and reads
//! the fraction from their two support values, and a query returning an
//! arbitrary corner would make that fraction depend on the simplex's history
//! rather than on the motion.

use cspace_bullet::broadphase_proxy::BroadphaseNativeType;
use cspace_bullet::linear_math::{Scalar, Transform, Vec3};
use cspace_bullet::shapes::ConvexShape;

/// `BULLET_EPSILON` (`bullet_utils.hpp:54`) -- "numerical precision limit".
///
/// Not a precision limit in any float sense: at `1e-3` it is six orders above
/// `f32::EPSILON`, and what it actually sets is how far off maximum a vertex
/// may be and still be averaged in by [`get_average_support`]. On a metre-scale
/// robot that is a millimetre of support, which is why a face that is a
/// thousandth off square still reports its centre.
pub const BULLET_EPSILON: Scalar = 1e-3;

/// `BULLET_SUPPORT_FUNC_TOLERANCE` (`bullet_utils.hpp:52`) -- `0.01f METERS`.
///
/// The gap in support value at which the contact conversion stops interpolating
/// and pins `percent_interpolation` to one end of the sweep.
pub const BULLET_SUPPORT_FUNC_TOLERANCE: Scalar = 0.01;

/// `BULLET_LENGTH_TOLERANCE` (`bullet_utils.hpp:53`) -- `0.001f METERS`.
///
/// When the contact point sits within this of *both* ends of the sweep, the
/// two distances cannot order them and `percent_interpolation` is 0.5.
pub const BULLET_LENGTH_TOLERANCE: Scalar = 0.001;

/// `BULLET_MARGIN` (`bullet_utils.hpp:51`) -- `0.0f`.
///
/// What `createShapePrimitive` calls `setMargin` with on every shape it builds,
/// and what makes Bullet's margin machinery a no-op on this path for every
/// shape except the sphere, whose `getMargin` returns its radius and ignores
/// the field entirely.
pub const BULLET_MARGIN: Scalar = 0.0;

/// `BULLET_DEFAULT_CONTACT_DISTANCE` (`bullet_utils.hpp:55`) -- `0.00f`, "all
/// pairs closer than this distance get reported".
///
/// At zero it decides more than a distance: `btCollisionDispatcher` looks a
/// child pair up in the contact-point table rather than the closest-points one
/// when `m_closestPointDistanceThreshold` is zero, which is a different
/// algorithm for a box-box pair, and every `extendAabb` in the compound
/// traversal grows by nothing.
pub const BULLET_DEFAULT_CONTACT_DISTANCE: Scalar = 0.0;

/// `CastHullShape` (`bullet_utils.hpp:240-334`) -- the convex hull of one shape
/// at two poses, represented by the shape and the delta between them.
///
/// Upstream owns a `btConvexShape*` and is handed to Bullet through a
/// `btCollisionObject`; here the shape is borrowed, because the cast shape's
/// lifetime is one continuous check and the shape it sweeps outlives it.
#[derive(Clone, Copy)]
pub struct CastHullShape<'a> {
    /// `m_shape` -- the shape being swept, in its own local frame.
    pub shape: &'a dyn ConvexShape,
    /// `shape_transform` -- the transform from the first pose to the second.
    ///
    /// Not a world transform: `setCastCollisionObjectsTransform` computes it as
    /// `tf1.inverseTimes(tf2)` and leaves the object's own world transform at
    /// `tf1`, so this is the delta the second placement sits at *relative to
    /// the first*.
    pub shape_transform: Transform,
}

impl<'a> CastHullShape<'a> {
    /// `CastHullShape(shape, t01)` (`bullet_utils.hpp:249-252`).
    #[must_use]
    pub fn new(shape: &'a dyn ConvexShape, t01: Transform) -> Self {
        Self {
            shape,
            shape_transform: t01,
        }
    }

    /// `updateCastTransform` (`bullet_utils.hpp:254-257`).
    ///
    /// The cast object is built once with an identity delta and re-pointed at
    /// each new pose pair through here; that is why the delta is mutable state
    /// on the shape rather than an argument to the query.
    pub fn update_cast_transform(&mut self, cast_transform: Transform) {
        self.shape_transform = cast_transform;
    }
}

impl ConvexShape for CastHullShape<'_> {
    /// `m_shapeType = CUSTOM_CONVEX_SHAPE_TYPE` (`bullet_utils.hpp:251`).
    ///
    /// This is the value that keeps the whole cast layer working without
    /// Bullet knowing anything about it: every `switch (m_shapeType)` in
    /// `btConvexShape`'s non-virtual fast paths falls through to `default:` and
    /// dispatches virtually, so the overrides below are what run.
    fn shape_type(&self) -> BroadphaseNativeType {
        BroadphaseNativeType::CUSTOM_CONVEX_SHAPE
    }

    /// `localGetSupportingVertex` (`bullet_utils.hpp:260-265`).
    ///
    /// The comparison is `>`, so the *second* support point wins a tie -- and a
    /// tie is not exotic: an identity delta makes the two points equal for
    /// every direction, which is the state a cast object is constructed in and
    /// the state a stationary link stays in.
    fn local_get_supporting_vertex(&self, vec: Vec3) -> Vec3 {
        let support_vector_0 = self.shape.local_get_supporting_vertex(vec);
        let support_vector_1 = self.shape_transform.transform_point(
            self.shape
                .local_get_supporting_vertex(self.shape_transform.basis.transposed_mul_vec(vec)),
        );
        if vec.dot(support_vector_0) > vec.dot(support_vector_1) {
            support_vector_0
        } else {
            support_vector_1
        }
    }

    /// `localGetSupportingVertexWithoutMargin` (`bullet_utils.hpp:330-333`) --
    /// the with-margin query, verbatim.
    ///
    /// Not a shortcut: [`CastHullShape::margin`] is zero, so the two differ by
    /// nothing anyway, and routing rather than duplicating is what upstream
    /// does. The inner shape's own margin is still applied, inside
    /// `local_get_supporting_vertex` above.
    fn local_get_supporting_vertex_without_margin(&self, vec: Vec3) -> Vec3 {
        self.local_get_supporting_vertex(vec)
    }

    /// `getMargin` (`bullet_utils.hpp:305-308`) -- always zero, whatever the
    /// swept shape's own margin is.
    fn margin(&self) -> Scalar {
        0.0
    }

    /// `setMargin` (`bullet_utils.hpp:301-303`) -- a no-op, which is why
    /// `makeCastCollisionObject`'s `subshape->setMargin(BULLET_MARGIN)` on a
    /// compound child changes nothing.
    fn set_margin(&mut self, _margin: Scalar) {}

    /// `getAabb` (`bullet_utils.hpp:277-284`) -- the union of the shape's AABB
    /// at both poses.
    ///
    /// The union of two boxes, not the box of the swept hull: for a rotating
    /// body those differ, and the looser one is what the broadphase gets.
    fn get_aabb(&self, t: &Transform) -> (Vec3, Vec3) {
        let (mut aabb_min, mut aabb_max) = self.shape.get_aabb(t);
        let (min1, max1) = self.shape.get_aabb(&(*t * self.shape_transform));

        // `btVector3::setMin`/`setMax`, which are `btSetMin`/`btSetMax` per
        // component (`btMinMax.h:26-38`): `if (b < a) a = b`, not `a.min(b)`.
        for i in 0..3 {
            if min1[i] < aabb_min[i] {
                aabb_min[i] = min1[i];
            }
            if max1[i] > aabb_max[i] {
                aabb_max[i] = max1[i];
            }
        }
        (aabb_min, aabb_max)
    }

    /// `getAabbSlow` (`bullet_utils.hpp:286-289`), which upstream implements as
    /// `throw std::runtime_error("shouldn't happen")`.
    ///
    /// The default this replaces would answer with six support queries -- a
    /// plausible AABB, computed by a function upstream declares unreachable.
    /// Inheriting it would mean a port that silently succeeds where upstream
    /// aborts, and the two answers need not agree.
    fn get_aabb_slow(&self, _trans: &Transform) -> (Vec3, Vec3) {
        panic!("shouldn't happen: CastHullShape::getAabbSlow (bullet_utils.hpp:286-289)")
    }
}

/// `getAverageSupport(shape, localNormal, outsupport, outpt)`
/// (`bullet_utils.hpp:344-383`) -- the maximum support value along
/// `local_normal`, and the centroid of the vertices achieving it.
///
/// The two branches are chosen by `dynamic_cast<const btPolyhedralConvexShape*>`,
/// which this port carries as
/// [`ConvexShape::polyhedral_vertices`]: a box or a hull gets the average, and
/// a sphere, cylinder or cone gets a single support query with no averaging at
/// all. That asymmetry is upstream's and is load-bearing -- a curved shape has
/// no vertex list to average, so its witness stays wherever the support
/// function puts it.
///
/// # The empty polyhedron
///
/// `pt_sum / pt_count` with a zero count is a division by zero, and this port
/// reproduces it rather than guarding: `pt_count` is a `float` upstream and the
/// result is `NaN`, propagating into the contact's position. Substituting a
/// zero or an early return here would make the port disagree with the oracle on
/// exactly the input where upstream is at its most fragile, and hide it. A hull
/// with no vertices cannot be built by `createShapePrimitive`, so nothing on
/// this path produces one.
#[must_use]
pub fn get_average_support(shape: &dyn ConvexShape, local_normal: Vec3) -> (Scalar, Vec3) {
    let mut pt_sum = Vec3::zero();
    let mut pt_count: Scalar = 0.0;
    let mut max_support: Scalar = -1000.0;

    let Some(vertices) = shape.polyhedral_vertices() else {
        let outpt = shape.local_get_supporting_vertex_without_margin(local_normal);
        return (local_normal.dot(outpt), outpt);
    };

    for pt in vertices.iter().copied() {
        let sup = pt.dot(local_normal);
        if sup > max_support + BULLET_EPSILON {
            pt_count = 1.0;
            pt_sum = pt;
            max_support = sup;
        } else if sup < max_support - BULLET_EPSILON {
            // Upstream's empty branch (`:368`). Written out rather than folded
            // into the `else` because the two are not the same condition: this
            // one rejects, and the fall-through below accumulates.
        } else {
            pt_count += 1.0;
            pt_sum += pt;
        }
    }
    (max_support, pt_sum / pt_count)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cspace_bullet::probe_fixture::{
        IDENTITY, at, diff, diff_vec3, probe_shapes, rot60_at, row,
    };

    /// The `cast_*` and `avgsup_*` rows of
    /// `tools/bullet-epa-reference/build.sh`'s stdout.
    ///
    /// Those rows come from `moveit_cast.hpp`, which is `CastHullShape` and
    /// `getAverageSupport` transcribed out of `bullet_utils.hpp` and compiled
    /// against the pinned bullet3 -- so they are upstream's own arithmetic, not
    /// a reading of it. See that file's header for why it is a copy.
    const BULLET_REFERENCE: &str = "\
cast_zero_delta|0.5|0.5|0.5|0.5|0.5|0.5|-0.5|-0.5|-0.5|0.5|0.5|0.5
cast_pos_delta|1.5|0.5|0.5|1.5|0.5|0.5|-0.5|-0.5|-0.5|1.5|0.5|0.5
cast_neg_delta|-0.5|0.5|0.5|-0.5|0.5|0.5|-0.5|-0.5|-0.5|1.5|0.5|0.5
cast_diag_delta|0.800000012|0.099999994|0.699999988|0.800000012|0.099999994|0.699999988|-0.5|-0.899999976|-0.5|0.800000012|0.5|0.699999988
cast_rot_delta|0.800000012|0.099999994|0.699999988|0.800000012|0.099999994|0.699999988|-0.533333361|-1.23333335|-0.633333385|1.13333344|0.5|1.03333342
cast_rot_delta_world|0.800000012|0.099999994|0.699999988|0.800000012|0.099999994|0.699999988|-0.733333349|-0.633333385|-0.933333397|1.06666672|1.03333342|0.733333349
cast_flat_rot_delta|0.966666698|-0.683333337|-0.233333334|0.966666698|-0.683333337|-0.233333334|-0.566666663|-0.616666675|-1.0333333|0.816666722|1.01666665|0.666666687
cast_margin_delta|1.5|0.5|0.5|1.5|0.5|0.5|-0.5|-0.5|-0.5|1.5|0.5|0.5
cast_sphere_delta|0.988675117|0.388675123|0.288675129|0.988675117|0.388675123|0.288675129|-0.5|-0.5|-0.5|1.20000005|0.600000024|0.5
cast_cyl_delta|0.704044044|-0.283823907|0.60404402|0.704044044|-0.283823907|0.60404402|-0.333333313|-0.966666698|-0.5|0.933333337|0.300000012|0.833333313
cast_cone_delta|0.566666722|-0.533333361|0.466666698|0.566666722|-0.533333361|0.466666698|-0.25|-0.533333361|-0.400000006|0.566666722|0.25|0.466666698
cast_hull_delta|0.5|-0.0999999642|0.300000012|0.5|-0.0999999642|0.300000012|-0.340000004|-0.833333373|-0.166666672|0.700000048|0.24000001|0.566666663
avgsup_box_face|0.5|0.5|0|0
avgsup_box_edge|0.707106769|0.5|0.5|0
avgsup_box_corner|0.866025388|0.5|0.5|0.5
avgsup_box_in_band|0.500249922|0.5|0|0
avgsup_box_out_of_band|0.500998974|0.5|0.5|0
avgsup_flat_box_face|0.699999988|0|0.699999988|0
avgsup_margin_box_face|0.5|0.5|0|0
avgsup_sphere|0|0|0|0
avgsup_cyl|0.533624113|0.212132037|0.212132037|0.5
avgsup_cone|0.230940104|0|0|0.400000006
avgsup_hull_face|0.100000001|0|0|0.100000001
avgsup_hull_corner|0.346410155|0.300000012|0.200000003|0.100000001
";

    #[test]
    fn bullet_reference_cast_hull_shape() {
        let (unit_box, flat_box, margin_box, sphere, _, cyl, cone, hull) = probe_shapes();
        let px = Vec3::new(1.0, 0.0, 0.0);
        let pxyz = Vec3::new(1.0, 1.0, 1.0);
        let rot = rot60_at(0.3, -0.4, 0.2);
        let world = rot60_at(0.1, 0.2, -0.1);

        let cases: [(&str, &dyn ConvexShape, Transform, Vec3, Transform); 12] = [
            ("zero_delta", &unit_box, IDENTITY, px, IDENTITY),
            ("pos_delta", &unit_box, at(1.0, 0.0, 0.0), px, IDENTITY),
            ("neg_delta", &unit_box, at(1.0, 0.0, 0.0), -px, IDENTITY),
            ("diag_delta", &unit_box, at(0.3, -0.4, 0.2), pxyz, IDENTITY),
            ("rot_delta", &unit_box, rot, pxyz, IDENTITY),
            ("rot_delta_world", &unit_box, rot, pxyz, world),
            ("flat_rot_delta", &flat_box, rot, px, world),
            (
                "margin_delta",
                &margin_box,
                at(1.0, 0.0, 0.0),
                pxyz,
                IDENTITY,
            ),
            ("sphere_delta", &sphere, at(0.7, 0.1, 0.0), pxyz, IDENTITY),
            ("cyl_delta", &cyl, rot, pxyz, IDENTITY),
            ("cone_delta", &cone, rot, pxyz, IDENTITY),
            ("hull_delta", &hull, rot, pxyz, IDENTITY),
        ];

        let mut bad = Vec::new();
        let mut covered = Vec::new();
        for (name, shape, t01, dir, world) in cases {
            let cast = CastHullShape::new(shape, t01);
            let (mn, mx) = cast.get_aabb(&world);

            let full = format!("cast_{name}");
            covered.push(full.clone());
            let f = row(BULLET_REFERENCE, &full, 13);
            let n = |i: usize| -> Scalar {
                f[i].parse()
                    .unwrap_or_else(|e| panic!("{full}: field {i} ({:?}): {e}", f[i]))
            };

            diff_vec3(
                &mut bad,
                name,
                "support",
                cast.local_get_supporting_vertex(dir),
                Vec3::new(n(1), n(2), n(3)),
            );
            diff_vec3(
                &mut bad,
                name,
                "support_no_margin",
                cast.local_get_supporting_vertex_without_margin(dir),
                Vec3::new(n(4), n(5), n(6)),
            );
            diff_vec3(&mut bad, name, "aabb_min", mn, Vec3::new(n(7), n(8), n(9)));
            diff_vec3(
                &mut bad,
                name,
                "aabb_max",
                mx,
                Vec3::new(n(10), n(11), n(12)),
            );
        }

        let normalized = |v: Vec3| v.normalize();
        let average_cases: [(&str, &dyn ConvexShape, Vec3); 12] = [
            ("box_face", &unit_box, px),
            ("box_edge", &unit_box, normalized(Vec3::new(1.0, 1.0, 0.0))),
            ("box_corner", &unit_box, normalized(pxyz)),
            (
                "box_in_band",
                &unit_box,
                normalized(Vec3::new(1.0, 0.0005, 0.0)),
            ),
            (
                "box_out_of_band",
                &unit_box,
                normalized(Vec3::new(1.0, 0.002, 0.0)),
            ),
            ("flat_box_face", &flat_box, Vec3::new(0.0, 1.0, 0.0)),
            ("margin_box_face", &margin_box, px),
            ("sphere", &sphere, normalized(pxyz)),
            ("cyl", &cyl, normalized(pxyz)),
            ("cone", &cone, normalized(pxyz)),
            ("hull_face", &hull, Vec3::new(0.0, 0.0, 1.0)),
            ("hull_corner", &hull, normalized(pxyz)),
        ];

        for (name, shape, local_normal) in average_cases {
            let (support, pt) = get_average_support(shape, local_normal);

            let full = format!("avgsup_{name}");
            covered.push(full.clone());
            let f = row(BULLET_REFERENCE, &full, 5);
            let n = |i: usize| -> Scalar {
                f[i].parse()
                    .unwrap_or_else(|e| panic!("{full}: field {i} ({:?}): {e}", f[i]))
            };

            diff(&mut bad, name, "support", support, n(1));
            diff_vec3(&mut bad, name, "pt", pt, Vec3::new(n(2), n(3), n(4)));
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

    /// The tie goes to the *transformed* support point, because upstream
    /// compares with `>`.
    ///
    /// `cast_zero_delta` above pins the value, but only for a delta whose
    /// transform is the identity -- under which the two branches return the
    /// same vector and the fixture cannot tell which one was taken. Here the
    /// delta is a pure rotation about the query direction, which the box is
    /// symmetric under: the two support points tie in *value* while being
    /// different vectors, so which branch ran is visible.
    #[test]
    fn a_tie_in_support_value_returns_the_transformed_point() {
        let (unit_box, ..) = probe_shapes();
        // 90 degrees about z: (0.5, 0.5, 0.5) maps to (-0.5, 0.5, 0.5), and
        // both have support 0.5 along +z.
        let spin = Transform::new(
            cspace_bullet::linear_math::Matrix3::from_rows(
                Vec3::new(0.0, -1.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ),
            Vec3::zero(),
        );
        let cast = CastHullShape::new(&unit_box, spin);
        let up = Vec3::new(0.0, 0.0, 1.0);

        let untransformed = unit_box.local_get_supporting_vertex(up);
        let got = cast.local_get_supporting_vertex(up);
        assert_eq!(up.dot(untransformed), up.dot(got), "the two must tie");
        assert_ne!(
            got, untransformed,
            "a tie must return the transformed branch, not the first"
        );
    }

    /// `getMargin` is zero even when the shape being swept has one.
    ///
    /// The `cast_margin_delta` row cannot see this on its own: the query it
    /// runs goes through `localGetSupportingVertex`, which applies the *inner*
    /// shape's margin, so the row is identical to the zero-margin box's.
    #[test]
    fn the_cast_shape_reports_no_margin_of_its_own() {
        let (_, _, margin_box, ..) = probe_shapes();
        assert_ne!(margin_box.margin(), 0.0);

        let mut cast = CastHullShape::new(&margin_box, IDENTITY);
        assert_eq!(cast.margin(), 0.0);
        cast.set_margin(0.25);
        assert_eq!(cast.margin(), 0.0, "setMargin is a no-op upstream");
    }

    /// `getAabbSlow` aborts rather than answering.
    #[test]
    #[should_panic(expected = "shouldn't happen")]
    fn get_aabb_slow_aborts_as_upstream_throws() {
        let (unit_box, ..) = probe_shapes();
        let cast = CastHullShape::new(&unit_box, IDENTITY);
        let _ = cast.get_aabb_slow(&IDENTITY);
    }
}
