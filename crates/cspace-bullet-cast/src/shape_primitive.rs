// Copyright (c) 2017, Southwest Research Institute
// Copyright (c) 2021, Southwest Research Institute
// Copyright (c) 2013, John Schulman
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-2-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection_bullet/src/bullet_integration/bullet_utils.cpp
//   moveit_core/collision_detection_bullet/src/bullet_integration/contact_checker_common.cpp

//! `createShapePrimitive` -- one `shapes::Shape` into the Bullet shape the
//! continuous check runs against.
//!
//! # The two collision-object types, and why only two
//!
//! `CollisionObjectType` (`basic_types.hpp`) has four values; the continuous
//! path constructs collision objects at exactly two sites and reaches two of
//! them. `CollisionEnvBullet::addToManager` picks `CONVEX_HULL` for a mesh and
//! `USE_SHAPE_TYPE` for everything else (`collision_env_bullet.cpp:253-267`);
//! `addAttachedObjects` picks `USE_SHAPE_TYPE` for every shape of an attached
//! body, mesh included (`collision_env_bullet.cpp:346`). `SDF` and
//! `MULTI_SPHERE` have no caller.
//!
//! [`CollisionObjectType`] therefore carries the two and not the four: a
//! variant with no producer is a branch no test can reach and no oracle run can
//! disagree about, and it would have to be transcribed out of an Apache-2.0
//! header this BSD-2-Clause crate cannot take code from.
//!
//! # Margins
//!
//! Every shape built here is `setMargin(BULLET_MARGIN)`-ed by its caller, and
//! `BULLET_MARGIN` is zero. Only the sphere keeps a margin regardless --
//! `btSphereShape::getMargin` returns the radius and ignores the field -- which
//! is why a sphere is the one primitive whose support function still moves after
//! the margin is cleared.

use std::sync::Arc;

use cspace_bullet::compound::{CompoundShape, Shape as BulletShape};
use cspace_bullet::convex_hull_computer::ConvexHullComputer;
use cspace_bullet::linear_math::{Scalar, Transform, Vec3};
use cspace_bullet::shapes::{
    BoxShape, ConeShapeZ, ConvexHullShape, ConvexShape, CylinderShapeZ, SphereShape,
    TriangleShapeEx,
};
use cspace_core::geometry::shapes::{Cone, Cuboid, Cylinder, Mesh, OcTree, Shape, Sphere};

use crate::cast_hull_shape::{BULLET_COMPOUND_USE_DYNAMIC_AABB, BULLET_MARGIN};

/// `CollisionObjectType` (`basic_types.hpp`), reduced to the two values the
/// continuous path produces -- see the module docs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CollisionObjectType {
    /// `USE_SHAPE_TYPE` -- build the Bullet shape that matches the geometry:
    /// a box for a box, and for a mesh a compound of one triangle per face.
    UseShapeType,
    /// `CONVEX_HULL` -- approximate the geometry by its convex hull. Only a
    /// mesh is ever asked for this.
    ConvexHull,
}

/// Why `createShapePrimitive` returned `nullptr`, or hit a branch this port does
/// not carry.
///
/// Upstream logs and returns null for the first three; a null shape then reaches
/// `shape->setMargin(...)` in the single-shape constructor
/// (`bullet_utils.cpp:576-577`) and dereferences it. Returning an error instead
/// is the one deliberate deviation here, and it is a deviation from a crash.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ShapeError {
    /// `"The mesh is empty!"` (`bullet_utils.cpp:196`) -- no vertices, or no
    /// triangles.
    EmptyMesh,
    /// `"This geometric shape type (%d) is not supported using BULLET yet"`
    /// (`bullet_utils.cpp:431`) -- a plane, which `shapes::Shape` has and
    /// Bullet's backend does not accept.
    UnsupportedGeometry {
        /// The `shapes::ShapeType` name upstream prints as an integer.
        shape_type: &'static str,
    },
    /// An octree world object whose `shapes::OcTree::octree` is null --
    /// upstream's default-constructed state, which `createShapePrimitive`
    /// dereferences without checking (`bullet_utils.cpp:206`).
    EmptyOcTree,
    /// `"Failed to create convex hull"` (`contact_checker_common.cpp:141`) --
    /// `btConvexHullComputer::compute` answered negative, which it does when
    /// `shrink` was large enough to leave the hull empty.
    ///
    /// Unreachable from `createConvexHull`'s callers here, which pass
    /// `createConvexHull`'s own default `shrink` of `-1`; the shrink pass is
    /// then skipped entirely and the returned shift is `0`. Carried because
    /// upstream carries the branch, and because the argument is a parameter
    /// rather than a constant one function down.
    ConvexHullFailed,
}

/// `createShapePrimitive(const shapes::Box*, ...)` (`bullet_utils.cpp:84-94`).
fn box_primitive(geom: &Cuboid) -> BoxShape {
    let half = |i: usize| (geom.size[i] / 2.0) as Scalar;
    BoxShape::new(Vec3::new(half(0), half(1), half(2)))
}

/// `createShapePrimitive(const shapes::Sphere*, ...)` (`bullet_utils.cpp:96-101`).
fn sphere_primitive(geom: &Sphere) -> SphereShape {
    SphereShape::new(geom.radius as Scalar)
}

/// `createShapePrimitive(const shapes::Cylinder*, ...)` (`bullet_utils.cpp:103-110`).
///
/// The half extents are `(r, r, l/2)`: a `btCylinderShapeZ` takes a box's half
/// extents, not a radius and a length.
fn cylinder_primitive(geom: &Cylinder) -> CylinderShapeZ {
    let r = geom.radius as Scalar;
    let l = (geom.length / 2.0) as Scalar;
    CylinderShapeZ::new(Vec3::new(r, r, l))
}

/// `createShapePrimitive(const shapes::Cone*, ...)` (`bullet_utils.cpp:112-119`).
///
/// The length is passed whole, unlike the cylinder's: `btConeShapeZ` takes a
/// height, not a half height.
fn cone_primitive(geom: &Cone) -> ConeShapeZ {
    ConeShapeZ::new(geom.radius as Scalar, geom.length as Scalar)
}

/// `createShapePrimitive(const shapes::OcTree*, USE_SHAPE_TYPE, cow)`
/// (`bullet_utils.cpp:200-245`) -- a compound of one zero-margin box per
/// occupied leaf, at the leaf's centre.
///
/// # The occupancy test is upstream's, through a monotone identity
///
/// Upstream compares probabilities: `it->getOccupancy() >=
/// geom->octree->getOccupancyThres()`. This port compares log-odds, which is
/// what [`cspace_core::octomap::Leaf::is_occupied`] does. The two agree
/// exactly, not approximately: `probability` is `1 - 1/(1 + exp(l))`, a
/// composition of IEEE-monotone steps, so `p(a) >= p(b)` and `a >= b` are the
/// same predicate -- distinct log-odds may collapse onto one probability, but
/// never invert.
fn octree_primitive(geom: &OcTree) -> Result<CompoundShape, ShapeError> {
    let tree = geom.octree.as_ref().ok_or(ShapeError::EmptyOcTree)?;

    let mut subshape = CompoundShape::new(BULLET_COMPOUND_USE_DYNAMIC_AABB);
    for leaf in tree.leaves() {
        if !leaf.is_occupied() {
            continue;
        }
        let coord = leaf.coordinate();
        let geom_trans = Transform::new(
            cspace_bullet::linear_math::Matrix3::identity(),
            Vec3::new(coord.x as Scalar, coord.y as Scalar, coord.z as Scalar),
        );
        let l = (leaf.size() / 2.0) as Scalar;
        let mut childshape = BoxShape::new(Vec3::new(l, l, l));
        childshape.set_margin(BULLET_MARGIN);

        subshape.add_child_shape(geom_trans, BulletShape::Convex(Arc::new(childshape)));
    }
    Ok(subshape)
}

/// `createShapePrimitive(const shapes::ShapeConstPtr&, type, cow)`
/// (`bullet_utils.cpp:400-435`) -- the dispatch on `geom->type`.
///
/// Upstream's `cow` argument is only there so the mesh and octree branches can
/// hand their sub-shapes to the wrapper's `manage`; here the returned
/// [`BulletShape`] owns them.
///
/// # Errors
///
/// See [`ShapeError`]. Upstream returns `nullptr` for the first three and the
/// caller dereferences it.
pub fn create_shape_primitive(
    geom: &Shape,
    collision_object_type: CollisionObjectType,
) -> Result<BulletShape, ShapeError> {
    match geom {
        Shape::Cuboid(cuboid) => Ok(BulletShape::Convex(Arc::new(box_primitive(cuboid)))),
        Shape::Sphere(sphere) => Ok(BulletShape::Convex(Arc::new(sphere_primitive(sphere)))),
        Shape::Cylinder(cylinder) => {
            Ok(BulletShape::Convex(Arc::new(cylinder_primitive(cylinder))))
        }
        Shape::Cone(cone) => Ok(BulletShape::Convex(Arc::new(cone_primitive(cone)))),
        Shape::Mesh(mesh) => mesh_primitive(mesh, collision_object_type),
        Shape::OcTree(octree) => Ok(BulletShape::Compound(octree_primitive(octree)?)),
        Shape::Plane(_) => Err(ShapeError::UnsupportedGeometry {
            shape_type: "plane",
        }),
    }
}

/// `createShapePrimitive(const shapes::Mesh*, type, cow)`
/// (`bullet_utils.cpp:121-197`).
///
/// # Errors
///
/// [`ShapeError::EmptyMesh`] for a mesh with no vertices or no triangles, and
/// [`ShapeError::ConvexHullFailed`] when the hull computer refuses the point
/// set. The triangle-soup branch returns no error of its own; a triangle whose
/// index runs past the vertex array panics here, where upstream reads out of
/// bounds and returns a shape built from whatever was there.
fn mesh_primitive(
    geom: &Mesh,
    collision_object_type: CollisionObjectType,
) -> Result<BulletShape, ShapeError> {
    // Upstream's guard is `vertex_count > 0 && triangle_count > 0`, and its
    // `else` is the `"The mesh is empty!"` return -- so a mesh with vertices
    // but no triangles is "empty" too, even though the hull branch would never
    // have read a triangle.
    if geom.vertices.is_empty() || geom.triangles.is_empty() {
        return Err(ShapeError::EmptyMesh);
    }
    match collision_object_type {
        CollisionObjectType::ConvexHull => {
            let input: Vec<Vec3> = geom
                .vertices
                .iter()
                .map(|v| Vec3::new(v[0] as Scalar, v[1] as Scalar, v[2] as Scalar))
                .collect();
            let vertices = create_convex_hull(
                &input,
                CREATE_CONVEX_HULL_SHRINK,
                CREATE_CONVEX_HULL_SHRINK_CLAMP,
            )
            .ok_or(ShapeError::ConvexHullFailed)?;

            let mut subshape = ConvexHullShape::new();
            for vertex in vertices {
                subshape.add_point(vertex);
            }
            Ok(BulletShape::Convex(Arc::new(subshape)))
        }
        CollisionObjectType::UseShapeType => {
            // One `btTriangleShapeEx` per face at an identity child transform,
            // the vertices read through the triangle's own indices. Upstream's
            // `if (subshape != nullptr)` guard around `addChildShape` is not
            // reproduced: `new` cannot return null there, and the guard is the
            // shape of the null checks the other branches genuinely need.
            let mut compound = CompoundShape::new(BULLET_COMPOUND_USE_DYNAMIC_AABB);
            for triangle in &geom.triangles {
                let corner = |i: usize| {
                    let v = geom.vertices[triangle[i] as usize];
                    Vec3::new(v[0] as Scalar, v[1] as Scalar, v[2] as Scalar)
                };
                let mut subshape = TriangleShapeEx::new(corner(0), corner(1), corner(2));
                subshape.set_margin(BULLET_MARGIN);
                compound.add_child_shape(
                    Transform::identity(),
                    BulletShape::Convex(Arc::new(subshape)),
                );
            }
            Ok(BulletShape::Compound(compound))
        }
    }
}

/// `createConvexHull`'s default `shrink` (`contact_checker_common.hpp:74`).
///
/// Negative, so `btConvexHullComputer::compute` skips the shrink pass; the one
/// caller in MoveIt never overrides it (`bullet_utils.cpp:144`).
const CREATE_CONVEX_HULL_SHRINK: Scalar = -1.0;

/// `createConvexHull`'s default `shrinkClamp` (`contact_checker_common.hpp:74`),
/// read only when [`CREATE_CONVEX_HULL_SHRINK`] is positive.
const CREATE_CONVEX_HULL_SHRINK_CLAMP: Scalar = -1.0;

/// `createConvexHull` (`contact_checker_common.cpp:123-183`), reduced to the
/// half its caller reads.
///
/// `None` is upstream's `return -1`, which `createShapePrimitive` turns
/// straight into a null shape (`bullet_utils.cpp:144-145`).
///
/// # Only the vertices
///
/// Upstream's second out-parameter, `faces`, is a flattened list of
/// per-face vertex counts and indices, walked out of the computer's edge
/// records by `getNextEdgeOfFace`. Nothing on the continuous path reads it:
/// `createShapePrimitive` declares the vector, passes it and then builds its
/// `btConvexHullShape` from `vertices` alone (`bullet_utils.cpp:136-152`), and
/// it is the only caller MoveIt has. `cspace_bullet::convex_hull_computer`
/// carries `faces` and `next_edge_of_face`, so the walk is one loop away from a
/// caller that wants it; producing it here would be output no code reads and no
/// oracle run could disagree about.
///
/// The double round trip upstream makes -- `btVector3` to `Eigen::Vector3d` and
/// back to `btVector3` at `addPoint` -- is not reproduced because it is exact:
/// every `f32` is an `f64`, and the values that come back are the same bits.
fn create_convex_hull(input: &[Vec3], shrink: Scalar, shrink_clamp: Scalar) -> Option<Vec<Vec3>> {
    let mut conv = ConvexHullComputer::new();
    if conv.compute(input, shrink, shrink_clamp) < 0.0 {
        return None;
    }
    Some(conv.vertices)
}

#[cfg(test)]
mod tests {
    use super::*;
    use cspace_bullet::probe_fixture::{IDENTITY, diff_vec3, row};
    use cspace_core::geometry::Vector3;
    use cspace_core::geometry::shapes::Plane;

    /// The `shape->setMargin(BULLET_MARGIN)` every caller of
    /// `createShapePrimitive` applies (`bullet_utils.cpp:577`,
    /// `bullet_utils.cpp:599`). Without it a box keeps `setSafeMargin`'s
    /// margin and every read is inset by it.
    fn zero_margin(mut shape: BulletShape) -> BulletShape {
        shape.set_margin(BULLET_MARGIN);
        shape
    }

    fn convex(shape: &BulletShape) -> &dyn ConvexShape {
        match shape {
            BulletShape::Convex(convex) => convex.as_ref(),
            BulletShape::Compound(_) => panic!("expected a convex shape, got a compound"),
        }
    }

    /// A box's `size` is full extents and `btBoxShape` takes half extents, so
    /// the conversion halves. Read back through the AABB rather than through a
    /// getter, because the AABB is what the broadphase sees.
    #[test]
    fn a_box_halves_its_extents() {
        let geom = Shape::Cuboid(Cuboid {
            size: [2.0, 4.0, 6.0],
        });
        let shape =
            zero_margin(create_shape_primitive(&geom, CollisionObjectType::UseShapeType).unwrap());

        let (mn, mx) = convex(&shape).get_aabb(&IDENTITY);
        assert_eq!(mn, Vec3::new(-1.0, -2.0, -3.0));
        assert_eq!(mx, Vec3::new(1.0, 2.0, 3.0));
    }

    /// The one asymmetry worth a test of its own: a cylinder's length is
    /// halved into `btCylinderShapeZ`'s half extents, and a cone's is not --
    /// `btConeShapeZ` takes a height and puts its apex at half of it. The two
    /// therefore reach the *same* z for the same `length`, which is exactly the
    /// invariant: a port that halved the cone as well would put its apex at
    /// 0.5 for a cone 2 long.
    #[test]
    fn a_cylinder_halves_its_length_and_a_cone_does_not() {
        let up = Vec3::new(0.0, 0.0, 1.0);
        let down = Vec3::new(0.0, 0.0, -1.0);

        let cylinder = zero_margin(
            create_shape_primitive(
                &Shape::Cylinder(Cylinder {
                    radius: 0.3,
                    length: 2.0,
                }),
                CollisionObjectType::UseShapeType,
            )
            .unwrap(),
        );
        let cone = zero_margin(
            create_shape_primitive(
                &Shape::Cone(Cone {
                    radius: 0.3,
                    length: 2.0,
                }),
                CollisionObjectType::UseShapeType,
            )
            .unwrap(),
        );

        assert_eq!(
            convex(&cylinder)
                .local_get_supporting_vertex_without_margin(up)
                .z,
            1.0,
            "a cylinder of length 2 caps at z = 1"
        );
        assert_eq!(
            convex(&cone)
                .local_get_supporting_vertex_without_margin(up)
                .z,
            1.0,
            "a cone of length 2 apexes at z = 1; a halved length would give 0.5"
        );
        assert_eq!(
            convex(&cone)
                .local_get_supporting_vertex_without_margin(down)
                .z,
            -1.0,
            "and its base is the other half, so the cone spans its full length"
        );
    }

    /// A sphere's radius passes through untouched, and stays the shape's margin
    /// even after the caller clears margins.
    #[test]
    fn a_sphere_keeps_its_radius_as_its_margin() {
        let shape = zero_margin(
            create_shape_primitive(
                &Shape::Sphere(Sphere { radius: 0.25 }),
                CollisionObjectType::UseShapeType,
            )
            .unwrap(),
        );
        assert_eq!(convex(&shape).margin(), 0.25);
    }

    /// A plane is a `shapes::Shape` Bullet's backend refuses.
    #[test]
    fn a_plane_is_refused_as_upstream_logs_and_returns_null() {
        let geom = Shape::Plane(Plane::default());
        assert_eq!(
            create_shape_primitive(&geom, CollisionObjectType::UseShapeType).err(),
            Some(ShapeError::UnsupportedGeometry {
                shape_type: "plane"
            })
        );
    }

    /// An octree with no tree is upstream's default-constructed state, which
    /// `createShapePrimitive` dereferences.
    #[test]
    fn an_octree_without_a_tree_is_an_error_rather_than_a_dereference() {
        let geom = Shape::OcTree(OcTree::new());
        assert_eq!(
            create_shape_primitive(&geom, CollisionObjectType::UseShapeType).err(),
            Some(ShapeError::EmptyOcTree)
        );
    }

    /// The triangle-soup branch: one child per *face*, not per vertex, at an
    /// identity child transform, with the face's own vertices.
    ///
    /// The two faces share vertices 0 and 2 and list them in different
    /// positions, which is what separates "read the triangle's indices" from
    /// "take three vertices in file order": the second child's corners are
    /// `v0, v2, v3`, and a port that walked the vertex array would put `v3`
    /// first.
    #[test]
    fn a_mesh_under_use_shape_type_is_a_compound_of_one_triangle_per_face() {
        let mesh = Mesh::new(
            vec![
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(1.0, 0.0, 0.0),
                Vector3::new(0.0, 1.0, 0.0),
                Vector3::new(0.0, 0.0, 1.0),
            ],
            vec![[0, 1, 2], [0, 2, 3]],
        )
        .unwrap();

        let shape = create_shape_primitive(&Shape::Mesh(mesh), CollisionObjectType::UseShapeType)
            .expect("a mesh with two faces builds");
        let BulletShape::Compound(compound) = shape else {
            panic!("the triangle-soup branch returns a compound");
        };
        assert_eq!(compound.num_child_shapes(), 2);

        let triangle = |i: usize| {
            let BulletShape::Convex(shape) = compound.child_shape(i) else {
                panic!("every child of the triangle soup is convex");
            };
            *shape
                .as_any()
                .downcast_ref::<TriangleShapeEx>()
                .expect("every child of the triangle soup is a triangle")
        };
        let corners = |i: usize| *triangle(i).vertices();
        assert_eq!(
            corners(0),
            [
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
            ]
        );
        assert_eq!(
            corners(1),
            [
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ]
        );
        for i in 0..2 {
            assert_eq!(*compound.child_transform(i), Transform::identity());
            assert_eq!(triangle(i).margin(), BULLET_MARGIN);
        }
    }

    /// `probe.cpp`'s `meshhull` rows: `createConvexHull` over a point set, the
    /// vertices fed to a `btConvexHullShape` in emission order, then the
    /// caller's `setMargin(BULLET_MARGIN)`.
    ///
    /// `chc_*` in `cspace_bullet::convex_hull_computer` already pins the
    /// computer itself; what these rows add is the part between it and the
    /// shape. The support directions include the three axes of `cube8`, where
    /// four corners are equally extreme and `maxDot`'s tie-break toward the
    /// lowest `addPoint` index is the only thing that decides the answer -- so
    /// a port that fed the vertices in any other order fails here and nowhere
    /// else. The AABB row separates a second ordering question: `addPoint`
    /// recalculates the cached AABB with the margin standing at that moment
    /// (the constructed 0.04), and the `setMargin(0)` afterwards does not
    /// recalculate, so a port that cleared the margin first agrees on every
    /// support point and disagrees on the box.
    const BULLET_REFERENCE: &str = "\
meshhull_cube8|8|8|-1.03999996|-1.03999996|-1.03999996|1.03999996|1.03999996|1.03999996
meshhullin_cube8_0|-1|-1|-1
meshhullin_cube8_1|-1|-1|1
meshhullin_cube8_2|-1|1|-1
meshhullin_cube8_3|-1|1|1
meshhullin_cube8_4|1|-1|-1
meshhullin_cube8_5|1|-1|1
meshhullin_cube8_6|1|1|-1
meshhullin_cube8_7|1|1|1
meshhulls_cube8_0|1|0|0|1|1|1
meshhulls_cube8_1|0|1|0|1|1|1
meshhulls_cube8_2|0|0|1|1|1|1
meshhulls_cube8_3|-1|0|0|-1|1|1
meshhulls_cube8_4|1|1|1|1|1|1
meshhulls_cube8_5|-1|1|-1|-1|1|-1
meshhulls_cube8_6|0.300000012|-0.699999988|0.25|1|-1|1
meshhull_shell|26|26|-1.01487923|-0.931586623|-0.976380467|0.991296709|1.0043062|1.01390517
meshhullin_shell_0|-0.877380371|-0.365575135|-0.310738862
meshhullin_shell_1|-0.468856454|0.771659553|-0.429785073
meshhullin_shell_2|0.714781821|-0.539457977|0.445052832
meshhullin_shell_3|-0.764458418|0.601541042|0.231843948
meshhullin_shell_4|0.951296747|0.293610096|-0.0939552337
meshhullin_shell_5|0.144645944|0.964306295|-0.221790448
meshhullin_shell_6|-0.518605113|-0.845397413|0.127875239
meshhullin_shell_7|0.395685554|0.402507722|-0.825481892
meshhullin_shell_8|0.0561828315|0.346460789|-0.936380506
meshhullin_shell_9|-0.675678492|0.722277045|-0.147561967
meshhullin_shell_10|-0.257323265|-0.674364388|0.692110837
meshhullin_shell_11|-0.479326606|-0.874961257|-0.0684752315
meshhullin_shell_12|-0.43338874|0.198120564|-0.879159987
meshhullin_shell_13|0.210057974|0.0859328061|0.973905146
meshhullin_shell_14|-0.974879324|0.204724655|-0.0877391398
meshhullin_shell_15|0.724001527|0.403052419|0.559795022
meshhullin_shell_16|-0.941792369|0.146885037|-0.302410394
meshhullin_shell_17|-0.435426056|-0.891586721|-0.124407448
meshhullin_shell_18|0.376857221|-0.609386146|-0.697586775
meshhullin_shell_19|-0.664462328|-0.40490672|0.628124535
meshhullin_shell_20|-0.660315454|0.693058372|0.289229095
meshhullin_shell_21|-0.268759996|-0.379425883|0.885327041
meshhullin_shell_22|-0.701542735|-0.443710804|-0.557636559
meshhullin_shell_23|-0.643340886|0.216777921|-0.734247804
meshhullin_shell_24|-0.513777375|-0.347146869|-0.784551978
meshhullin_shell_25|-0.436367154|-0.65085268|-0.621268511
meshhulls_shell_0|1|0|0|0.951296747|0.293597877|-0.0938054174
meshhulls_shell_1|0|1|0|0.144512549|0.964306235|-0.221706301
meshhulls_shell_2|0|0|1|0.209937677|0.0857727528|0.973905206
meshhulls_shell_3|-1|0|0|-0.974879324|0.204581872|-0.0876347572
meshhulls_shell_4|1|1|1|0.723911405|0.40296042|0.559723258
meshhulls_shell_5|-1|1|-1|-0.468824446|0.771559358|-0.429638714
meshhulls_shell_6|0.300000012|-0.699999988|0.25|0.714672744|-0.539337635|0.444911599";

    #[test]
    fn bullet_reference_mesh_hull() {
        let mut bad: Vec<String> = Vec::new();
        let mut covered: Vec<String> = Vec::new();

        for line in BULLET_REFERENCE.lines() {
            let header = line.split('|').next().unwrap();
            let Some(name) = header.strip_prefix("meshhull_") else {
                continue;
            };
            covered.push(header.to_string());

            let f = row(BULLET_REFERENCE, header, 9);
            let count: usize = f[1].parse().unwrap();
            let want_vertices: usize = f[2].parse().unwrap();
            let want_min = Vec3::new(
                f[3].parse().unwrap(),
                f[4].parse().unwrap(),
                f[5].parse().unwrap(),
            );
            let want_max = Vec3::new(
                f[6].parse().unwrap(),
                f[7].parse().unwrap(),
                f[8].parse().unwrap(),
            );

            // The input comes back out of the fixture rather than being
            // spelled a second time here: a point set transcribed into both
            // languages is a premise that can drift, and it would drift as a
            // coordinate mismatch blamed on the hull.
            let mut vertices: Vec<Vector3> = Vec::with_capacity(count);
            for i in 0..count {
                let input_row = format!("meshhullin_{name}_{i}");
                covered.push(input_row.clone());
                let g = row(BULLET_REFERENCE, &input_row, 4);
                vertices.push(Vector3::new(
                    g[1].parse::<Scalar>().unwrap().into(),
                    g[2].parse::<Scalar>().unwrap().into(),
                    g[3].parse::<Scalar>().unwrap().into(),
                ));
            }
            // The hull branch never reads a triangle, but the emptiness guard
            // above it does.
            let mesh = Mesh::new(vertices, vec![[0, 1, 2]]).unwrap();

            let shape = zero_margin(
                create_shape_primitive(&Shape::Mesh(mesh), CollisionObjectType::ConvexHull)
                    .expect("a hull over a non-degenerate point set"),
            );
            let (got_min, got_max) = shape.get_aabb(&IDENTITY);
            diff_vec3(&mut bad, name, "aabb_min", got_min, want_min);
            diff_vec3(&mut bad, name, "aabb_max", got_max, want_max);

            let BulletShape::Convex(hull) = &shape else {
                panic!("{name}: the hull branch builds a convex shape, not a compound");
            };
            let hull = hull
                .as_ref()
                .as_any()
                .downcast_ref::<ConvexHullShape>()
                .unwrap_or_else(|| panic!("{name}: the hull branch builds a ConvexHullShape"));
            if hull.unscaled_points().len() != want_vertices {
                bad.push(format!(
                    "{name}: port {} hull vertices, bullet {want_vertices}",
                    hull.unscaled_points().len()
                ));
            }

            for i in 0.. {
                let support_row = format!("meshhulls_{name}_{i}");
                if row_missing(BULLET_REFERENCE, &support_row) {
                    break;
                }
                covered.push(support_row.clone());
                let g = row(BULLET_REFERENCE, &support_row, 7);
                let dir = Vec3::new(
                    g[1].parse().unwrap(),
                    g[2].parse().unwrap(),
                    g[3].parse().unwrap(),
                );
                let want = Vec3::new(
                    g[4].parse().unwrap(),
                    g[5].parse().unwrap(),
                    g[6].parse().unwrap(),
                );
                diff_vec3(
                    &mut bad,
                    name,
                    &format!("support[{i}]"),
                    hull.local_get_supporting_vertex_without_margin(dir),
                    want,
                );
            }
        }

        let mut want: Vec<String> = BULLET_REFERENCE
            .lines()
            .map(|l| l.split('|').next().unwrap().to_string())
            .collect();
        want.sort();
        covered.sort();
        assert_eq!(
            covered, want,
            "the cases and BULLET_REFERENCE disagree on which rows exist"
        );
        assert!(bad.is_empty(), "{}", bad.join("\n"));
    }

    /// Whether `name` has no row, so a per-case row list can be walked to its
    /// end without the count being written down twice.
    fn row_missing(reference: &str, name: &str) -> bool {
        !reference.lines().any(|l| l.split('|').next() == Some(name))
    }

    /// A mesh with vertices but no triangles is "empty" too -- upstream's guard
    /// is a conjunction, so the hull branch never gets to ignore the triangles
    /// it does not read.
    #[test]
    fn a_mesh_with_no_triangles_is_empty_even_for_the_hull_branch() {
        let mesh = Mesh::new(vec![Vector3::new(0.0, 0.0, 0.0)], vec![]).unwrap();
        assert_eq!(
            create_shape_primitive(&Shape::Mesh(mesh), CollisionObjectType::ConvexHull).err(),
            Some(ShapeError::EmptyMesh)
        );
    }
}
