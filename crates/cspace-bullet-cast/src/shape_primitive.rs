// Copyright (c) 2017, Southwest Research Institute
// Copyright (c) 2013, John Schulman
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-2-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection_bullet/src/bullet_integration/bullet_utils.cpp

//! `createShapePrimitive` -- one `shapes::Shape` into the Bullet shape the
//! continuous check runs against.
//!
//! # The two collision-object types, and why only two
//!
//! `CollisionObjectType` (`basic_types.hpp`) has four values; the continuous
//! path constructs collision objects at exactly two sites and reaches two of
//! them. `CollisionEnvBullet::addToManager` picks `CONVEX_HULL` for a mesh and
//! `USE_SHAPE_TYPE` for everything else (`collision_env_bullet.cpp:253-263`);
//! `addAttachedObjects` picks `USE_SHAPE_TYPE` for every shape of an attached
//! body, mesh included (`:346`). `SDF` and `MULTI_SPHERE` have no caller.
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
use cspace_bullet::linear_math::{Scalar, Transform, Vec3};
use cspace_bullet::shapes::{BoxShape, ConeShapeZ, ConvexShape, CylinderShapeZ, SphereShape};
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
    /// `"The mesh is empty!"` (`bullet_utils.cpp:195`) -- no vertices, or no
    /// triangles.
    EmptyMesh,
    /// `"This geometric shape type (%d) is not supported using BULLET yet"`
    /// (`bullet_utils.cpp:430`) -- a plane, which `shapes::Shape` has and
    /// Bullet's backend does not accept.
    UnsupportedGeometry {
        /// The `shapes::ShapeType` name upstream prints as an integer.
        shape_type: &'static str,
    },
    /// An octree world object whose `shapes::OcTree::octree` is null --
    /// upstream's default-constructed state, which `createShapePrimitive`
    /// dereferences without checking (`bullet_utils.cpp:206`).
    EmptyOcTree,
    /// A mesh under [`CollisionObjectType::UseShapeType`], which builds a
    /// compound of `btTriangleShapeEx` (`bullet_utils.cpp:175`).
    ///
    /// Not ported: `cspace_bullet`'s scope is the convex narrow phase, and
    /// `btTriangleShapeEx` reaches `btConvexConvexAlgorithm` only through the
    /// GIMPACT concave machinery. The one producer is an attached body whose
    /// shape is a mesh (`collision_env_bullet.cpp:346`), so a robot carrying a
    /// mesh-shaped attached body cannot be continuous-checked by this port.
    AttachedMeshUnported,
    /// A mesh under [`CollisionObjectType::ConvexHull`], which needs
    /// `createConvexHull` and so `btConvexHullComputer`
    /// (`contact_checker_common.cpp:121-176`).
    ///
    /// Unlike [`Self::AttachedMeshUnported`] this is sequencing, not scope: the
    /// computer is being ported onto `cspace_bullet` and this arm becomes the
    /// call to it. Every mesh world object and every mesh robot link takes this
    /// branch (`collision_env_bullet.cpp:253-263`), so until then the continuous
    /// check covers only primitive-shaped geometry.
    ConvexHullComputerNotYetPorted,
}

impl ShapeError {
    /// Whether upstream reaches this by returning `nullptr`, rather than by
    /// building the shape.
    ///
    /// The distinction has one consumer and it is not cosmetic: the
    /// multi-shape constructor drops a null child from the compound and keeps
    /// going (`bullet_utils.cpp:596`). Dropping a shape *this port* has not
    /// reached yet -- [`Self::AttachedMeshUnported`],
    /// [`Self::ConvexHullComputerNotYetPorted`] -- would silently build an
    /// object smaller than its geometry, and the check would then report no
    /// collision for a reason nothing in the result names.
    #[must_use]
    pub fn is_upstream_null(&self) -> bool {
        match self {
            Self::EmptyMesh | Self::UnsupportedGeometry { .. } | Self::EmptyOcTree => true,
            Self::AttachedMeshUnported | Self::ConvexHullComputerNotYetPorted => false,
        }
    }
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
/// one of [`ShapeError::ConvexHullComputerNotYetPorted`] /
/// [`ShapeError::AttachedMeshUnported`] for the two branches -- see those
/// variants for which is sequencing and which is scope.
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
        CollisionObjectType::ConvexHull => Err(ShapeError::ConvexHullComputerNotYetPorted),
        CollisionObjectType::UseShapeType => Err(ShapeError::AttachedMeshUnported),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use cspace_bullet::probe_fixture::IDENTITY;
    use cspace_core::geometry::Vector3;
    use cspace_core::geometry::shapes::Plane;

    /// The `shape->setMargin(BULLET_MARGIN)` every caller of
    /// `createShapePrimitive` applies (`bullet_utils.cpp:577`, `:599`). Without
    /// it a box keeps `setSafeMargin`'s margin and every read is inset by it.
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

    /// The two mesh branches are distinguishable: one is scope, one is
    /// sequencing, and a caller deciding whether to fall back needs to know
    /// which it hit.
    #[test]
    fn the_two_mesh_branches_report_different_errors() {
        let mesh = Mesh::new(
            vec![
                Vector3::new(0.0, 0.0, 0.0),
                Vector3::new(1.0, 0.0, 0.0),
                Vector3::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2]],
        )
        .unwrap();
        let geom = Shape::Mesh(mesh);

        assert_eq!(
            create_shape_primitive(&geom, CollisionObjectType::ConvexHull).err(),
            Some(ShapeError::ConvexHullComputerNotYetPorted)
        );
        assert_eq!(
            create_shape_primitive(&geom, CollisionObjectType::UseShapeType).err(),
            Some(ShapeError::AttachedMeshUnported)
        );
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
