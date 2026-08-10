// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org
// Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: Zlib
//
// Ported from bullet3 @ 7dee3436e747958e7088dfdcea0e4ae031ce619e (tag 3.24):
//   bullet3/src/BulletCollision/BroadphaseCollision/btBroadphaseProxy.h
//   bullet3/src/BulletCollision/CollisionShapes/btConvexShape.h
//   bullet3/src/BulletCollision/CollisionShapes/btConvexShape.cpp
//   bullet3/src/BulletCollision/CollisionShapes/btConvexInternalShape.h
//   bullet3/src/BulletCollision/CollisionShapes/btConvexInternalShape.cpp
//   bullet3/src/BulletCollision/CollisionShapes/btPolyhedralConvexShape.h
//   bullet3/src/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp
//   bullet3/src/BulletCollision/CollisionShapes/btBoxShape.h
//   bullet3/src/BulletCollision/CollisionShapes/btSphereShape.h
//   bullet3/src/BulletCollision/CollisionShapes/btSphereShape.cpp
//   bullet3/src/BulletCollision/CollisionShapes/btCylinderShape.h
//   bullet3/src/BulletCollision/CollisionShapes/btCylinderShape.cpp
//   bullet3/src/BulletCollision/CollisionShapes/btConeShape.h
//   bullet3/src/BulletCollision/CollisionShapes/btConeShape.cpp
//   bullet3/src/BulletCollision/CollisionShapes/btConvexHullShape.h
//   bullet3/src/BulletCollision/CollisionShapes/btConvexHullShape.cpp

//! The convex shapes MoveIt's Bullet backend builds, and their support
//! functions.
//!
//! `createShapePrimitive` (`bullet_utils.cpp:84-210`) maps every MoveIt
//! geometry onto one of five convex shapes -- `btBoxShape`, `btSphereShape`,
//! `btCylinderShapeZ`, `btConeShapeZ`, `btConvexHullShape` -- or onto a
//! compound of them. Those five are here; nothing else in Bullet's shape
//! library is.
//!
//! # Margins are not what the constructor was given
//!
//! Bullet stores a convex shape *shrunk by its margin* and re-inflates it in
//! the support function, so a shape's dimensions depend on the margin history,
//! not only on the constructor argument. Three separate conventions collide,
//! and MoveIt drives all three at once by calling `setMargin(BULLET_MARGIN)`
//! -- which is `0.0f` (`bullet_utils.hpp:51`) -- on every shape it builds
//! (`bullet_utils.cpp:577`, `:587`, `:599`):
//!
//! - [`BoxShape`] and [`CylinderShapeZ`] subtract the margin from their stored
//!   dimensions at construction and *re-add it* on every `setMargin`, so
//!   MoveIt's `setMargin(0)` leaves them holding their true half extents.
//! - [`SphereShape`]'s margin **is** its radius, and its `setMargin` override
//!   drops the value on the floor: `getMargin()` returns `getRadius()`
//!   regardless (`btSphereShape.h:60-64`). MoveIt's `setMargin(0)` is a no-op
//!   on a sphere, and its support function is entirely margin.
//! - [`ConeShapeZ`] and [`ConvexHullShape`] store their geometry unshrunk and
//!   inherit the plain `btConvexInternalShape::setMargin`, so `setMargin(0)`
//!   simply deletes their default margin.
//!
//! The consequence worth stating out loud, because it survives into the
//! broadphase: a hull's cached local AABB is computed by `recalcLocalAabb()`
//! at `addPoint` time (`btConvexHullShape.cpp:50-55`) using the margin *as it
//! then stands*, and `btConvexInternalShape::setMargin` does not invalidate it
//! (`btConvexInternalShape.h:102-105`). So a MoveIt convex hull carries an
//! AABB inflated by `CONVEX_DISTANCE_MARGIN` per side forever, even though its
//! margin has since been set to zero. See
//! [`ConvexHullShape::add_point`].
//!
//! # Local scaling
//!
//! Absent. `m_localScaling` is `(1, 1, 1)` for every shape on this path:
//! MoveIt's Bullet integration never calls `setLocalScaling` on a Bullet shape
//! (the only two matches in `moveit_core/collision_detection_bullet/` are
//! `CastHullShape`'s own no-op overrides, `bullet_utils.hpp:291-297`).
//! Carrying a field that is provably constant would be configurability nobody
//! exercises; where upstream multiplies by it, the port drops the multiply and
//! says so.
//!
//! # Not ported
//!
//! `btCapsuleShape`, `btTriangleShape`, `btConvexPointCloudShape` and the
//! `btConvexInternalAabbCachingShape` family: `createShapePrimitive` builds
//! none of them for the collision-object types `CollisionEnvBullet` requests.
//! `btTriangleShapeEx` is the near miss -- `createShapePrimitive` does build a
//! compound of them, but only for `CollisionObjectType::USE_SHAPE_TYPE` on a
//! mesh, and `CollisionEnvBullet` asks for `CONVEX_HULL` on every mesh
//! (`collision_env_bullet.cpp`'s `getCollisionObjectType`), so that arm is
//! unreachable from the continuous check.

use crate::linear_math::{
    SIMD_EPSILON, Scalar, Transform, Vec3, bt_fsel, transform_aabb, transform_aabb_half_extents,
};

/// `CONVEX_DISTANCE_MARGIN` -- the margin every `btConvexInternalShape` starts
/// life with (`btCollisionMargin.h:22`).
pub const CONVEX_DISTANCE_MARGIN: Scalar = 0.04;

/// The `btCollisionShape`/`btConvexShape` interface this port needs: the
/// support function and margin GJK asks for, the AABB the broadphase asks for,
/// and the `setMargin` that decides what the other three answer.
///
/// # The virtual and non-virtual support paths
///
/// Bullet has two: the virtual `localGetSupportingVertexWithoutMargin`, and
/// `btConvexShape::localGetSupportVertexWithoutMarginNonVirtual`, a `switch`
/// on the shape type that GJK calls to skip the vtable
/// (`btConvexShape.cpp:131-305`). This trait has one method, because the two
/// were checked case by case and compute the same expression for every shape
/// MoveIt builds:
///
/// - sphere: the switch returns `(0,0,0)`; so does the virtual
///   (`btSphereShape.cpp:21-24`).
/// - box: the switch reads `getImplicitShapeDimensions()`; the virtual reads
///   `getHalfExtentsWithoutMargin()`, which returns that same member
///   (`btBoxShape.h:42-45`).
/// - cylinder: the switch inlines `upAxis == 2`'s `XX,YY,ZZ = 0,2,1`; the
///   virtual calls `CylinderLocalSupportZ`, which uses the same three
///   (`btCylinderShape.cpp:183-224`).
/// - cone: not in the switch at all -- it falls through `default:` to the
///   virtual call.
/// - convex hull: the switch calls `convexHullSupport`, whose body is the
///   virtual's body (`btConvexShape.cpp:119-127`).
///
/// The same holds for the margin: `getMarginNonVirtual`'s sphere arm returns
/// `getRadius()` exactly as the virtual `getMargin()` override does, and every
/// other arm returns `m_collisionMargin` (`btConvexShape.cpp:320-372`).
///
/// The equivalence also covers shapes this crate does not define: a custom
/// shape type (MoveIt's `CastHullShape` is one) reaches `default:` and is
/// dispatched virtually anyway.
pub trait ConvexShape {
    /// `btConvexShape::localGetSupportingVertexWithoutMargin`.
    fn local_get_supporting_vertex_without_margin(&self, vec: Vec3) -> Vec3;

    /// `btCollisionShape::getMargin`.
    fn margin(&self) -> Scalar;

    /// `btCollisionShape::setMargin` -- pure virtual upstream
    /// (`btCollisionShape.h:118`), and four of the five shapes here override
    /// it differently. It belongs on the trait for the same reason it is
    /// virtual there: `createShapePrimitive` calls it through the base pointer
    /// on whatever shape it has just built (`bullet_utils.cpp:577`, `:587`,
    /// `:599`), and the shape decides what that means.
    fn set_margin(&mut self, margin: Scalar);

    /// `btCollisionShape::getAabb` -- the shape's world AABB under `t`.
    fn get_aabb(&self, t: &Transform) -> (Vec3, Vec3);

    /// `btConvexInternalShape::localGetSupportingVertex`
    /// (`btConvexInternalShape.cpp:50-67`).
    ///
    /// `btSphereShape`, `btConeShape` and `btConvexHullShape` each override
    /// this with the same body; the sphere's drops the `margin != 0` guard,
    /// which cannot be observed -- with a zero radius its margin is zero and
    /// the term it adds is `0 * unit`, i.e. the vector the guard would have
    /// skipped adding.
    fn local_get_supporting_vertex(&self, vec: Vec3) -> Vec3 {
        let sup_vertex = self.local_get_supporting_vertex_without_margin(vec);
        if self.margin() != 0.0 {
            let mut vecnorm = vec;
            if vecnorm.length2() < (SIMD_EPSILON * SIMD_EPSILON) {
                vecnorm = Vec3::new(-1.0, -1.0, -1.0);
            }
            let vecnorm = vecnorm.normalize();
            return sup_vertex + self.margin() * vecnorm;
        }
        sup_vertex
    }

    /// `btConvexShape::localGetSupportVertexNonVirtual`
    /// (`btConvexShape.cpp:307-317`) -- what GJK actually calls.
    ///
    /// Not the same arithmetic as [`ConvexShape::local_get_supporting_vertex`]
    /// even though both add `margin * unit(dir)`: this one normalizes the
    /// direction *before* asking for the support point, so the cylinder's
    /// `radius / sqrt(v.x^2 + v.z^2)` and the hull's `maxDot` see a unit
    /// vector where the virtual path sees the caller's raw one. The results
    /// agree to within rounding, and rounding is the whole subject here.
    fn local_get_support_vertex_non_virtual(&self, local_dir: Vec3) -> Vec3 {
        let mut local_dir_norm = local_dir;
        if local_dir_norm.length2() < (SIMD_EPSILON * SIMD_EPSILON) {
            local_dir_norm = Vec3::new(-1.0, -1.0, -1.0);
        }
        let local_dir_norm = local_dir_norm.normalize();

        self.local_get_supporting_vertex_without_margin(local_dir_norm)
            + self.margin() * local_dir_norm
    }

    /// `btConvexInternalShape::getAabbSlow`
    /// (`btConvexInternalShape.cpp:29-48`) -- six support queries, one per
    /// axis direction, taken into the shape's frame by `vec * trans.getBasis()`.
    ///
    /// This is the base-class `getAabb` for any shape that does not override
    /// it, which among the five here is [`ConeShapeZ`] alone.
    fn get_aabb_slow(&self, trans: &Transform) -> (Vec3, Vec3) {
        let margin = self.margin();
        let mut min_aabb = Vec3::zero();
        let mut max_aabb = Vec3::zero();
        for i in 0..3 {
            let mut vec = Vec3::zero();
            vec[i] = 1.0;

            let sv = self.local_get_supporting_vertex(trans.basis.transposed_mul_vec(vec));
            let tmp = trans.transform_point(sv);
            max_aabb[i] = tmp[i] + margin;

            vec[i] = -1.0;
            let sv = self.local_get_supporting_vertex(trans.basis.transposed_mul_vec(vec));
            let tmp = trans.transform_point(sv);
            min_aabb[i] = tmp[i] - margin;
        }
        (min_aabb, max_aabb)
    }
}

/// `btBoxShape` (`btBoxShape.h`, `btBoxShape.cpp`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoxShape {
    /// `m_implicitShapeDimensions` -- the half extents **minus** the current
    /// margin.
    implicit_shape_dimensions: Vec3,
    /// `m_collisionMargin`.
    collision_margin: Scalar,
}

impl BoxShape {
    /// `btBoxShape(boxHalfExtents)` (`btBoxShape.cpp:17-26`), including the
    /// `setSafeMargin` clamp its constructor ends with.
    #[must_use]
    pub fn new(box_half_extents: Vec3) -> Self {
        let mut shape = Self {
            implicit_shape_dimensions: box_half_extents
                - Vec3::new(
                    CONVEX_DISTANCE_MARGIN,
                    CONVEX_DISTANCE_MARGIN,
                    CONVEX_DISTANCE_MARGIN,
                ),
            collision_margin: CONVEX_DISTANCE_MARGIN,
        };
        shape.set_safe_margin(box_half_extents);
        shape
    }

    /// `btConvexInternalShape::setSafeMargin(halfExtents, 0.1f)`
    /// (`btConvexInternalShape.h:66-81`) -- clamp the margin to a tenth of the
    /// smallest half extent, but only downward.
    fn set_safe_margin(&mut self, half_extents: Vec3) {
        let min_dimension = half_extents[half_extents.min_axis()];
        let safe_margin = 0.1 * min_dimension;
        if safe_margin < self.collision_margin {
            self.set_margin(safe_margin);
        }
    }

    /// `btBoxShape::getHalfExtentsWithoutMargin` (`btBoxShape.h:42-45`).
    #[must_use]
    pub fn half_extents_without_margin(&self) -> Vec3 {
        self.implicit_shape_dimensions
    }
}

impl ConvexShape for BoxShape {
    /// `btBoxShape::localGetSupportingVertexWithoutMargin`
    /// (`btBoxShape.h:58-65`).
    ///
    /// `btFsel`, not a sign test: a zero component of `vec` selects the `+`
    /// face, so `d = (0, 1, 0)` on a box returns all-positive components.
    fn local_get_supporting_vertex_without_margin(&self, vec: Vec3) -> Vec3 {
        let half_extents = self.half_extents_without_margin();
        Vec3::new(
            bt_fsel(vec.x, half_extents.x, -half_extents.x),
            bt_fsel(vec.y, half_extents.y, -half_extents.y),
            bt_fsel(vec.z, half_extents.z, -half_extents.z),
        )
    }

    fn margin(&self) -> Scalar {
        self.collision_margin
    }

    /// `btBoxShape::setMargin` (`btBoxShape.h:82-91`) -- re-expands the stored
    /// dimensions by the old margin and re-shrinks them by the new one, so the
    /// true half extents are invariant across margin changes.
    fn set_margin(&mut self, collision_margin: Scalar) {
        let old_margin = Vec3::new(
            self.collision_margin,
            self.collision_margin,
            self.collision_margin,
        );
        let with_margin = self.implicit_shape_dimensions + old_margin;
        self.collision_margin = collision_margin;
        let new_margin = Vec3::new(collision_margin, collision_margin, collision_margin);
        self.implicit_shape_dimensions = with_margin - new_margin;
    }

    /// `btBoxShape::getAabb` (`btBoxShape.cpp:28-31`).
    fn get_aabb(&self, t: &Transform) -> (Vec3, Vec3) {
        transform_aabb_half_extents(self.half_extents_without_margin(), self.margin(), t)
    }
}

/// `btSphereShape` (`btSphereShape.h`, `btSphereShape.cpp`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SphereShape {
    /// `m_implicitShapeDimensions.x()` -- the radius. Bullet keeps the other
    /// two components zeroed.
    radius: Scalar,
}

impl SphereShape {
    /// `btSphereShape(radius)` (`btSphereShape.h:29-37`).
    #[must_use]
    pub const fn new(radius: Scalar) -> Self {
        Self { radius }
    }

    /// `btSphereShape::getRadius` (`btSphereShape.h:47`).
    #[must_use]
    pub const fn radius(&self) -> Scalar {
        self.radius
    }
}

impl ConvexShape for SphereShape {
    /// `btSphereShape::localGetSupportingVertexWithoutMargin`
    /// (`btSphereShape.cpp:21-25`) -- the origin. A sphere is *all* margin.
    fn local_get_supporting_vertex_without_margin(&self, _vec: Vec3) -> Vec3 {
        Vec3::zero()
    }

    /// `btSphereShape::getMargin` (`btSphereShape.h:60-64`) -- the radius, so
    /// that GJK never has to enter the penetration case for a sphere.
    fn margin(&self) -> Scalar {
        self.radius
    }

    /// `btSphereShape::setMargin` (`btSphereShape.h:56-59`) -- stores the value
    /// in `m_collisionMargin`, which `getMargin` then never reads. Modelled as
    /// the no-op it is rather than kept as a write-only field, so that
    /// `set_margin(0.0)` on a sphere cannot be misread as shrinking it.
    fn set_margin(&mut self, _margin: Scalar) {}

    /// `btSphereShape::getAabb` (`btSphereShape.cpp:52-58`) -- the basis is
    /// ignored, which upstream's own comment calls "broken due to scaling".
    fn get_aabb(&self, t: &Transform) -> (Vec3, Vec3) {
        let center = t.origin;
        let extent = Vec3::new(self.margin(), self.margin(), self.margin());
        (center - extent, center + extent)
    }
}

/// `btCylinderShapeZ` -- radius in x/y, half height in z
/// (`btCylinderShape.cpp:36-40`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CylinderShapeZ {
    /// `m_implicitShapeDimensions` -- `(radius, radius, halfHeight)` minus the
    /// current margin, componentwise.
    implicit_shape_dimensions: Vec3,
    collision_margin: Scalar,
}

impl CylinderShapeZ {
    /// `btCylinderShapeZ(halfExtents)` (`btCylinderShape.cpp:19-40`).
    ///
    /// MoveIt passes `(radius, radius, length / 2)`
    /// (`bullet_utils.cpp:103-109`).
    #[must_use]
    pub fn new(half_extents: Vec3) -> Self {
        let mut shape = Self {
            implicit_shape_dimensions: half_extents
                - Vec3::new(
                    CONVEX_DISTANCE_MARGIN,
                    CONVEX_DISTANCE_MARGIN,
                    CONVEX_DISTANCE_MARGIN,
                ),
            collision_margin: CONVEX_DISTANCE_MARGIN,
        };
        shape.set_safe_margin(half_extents);
        shape
    }

    fn set_safe_margin(&mut self, half_extents: Vec3) {
        let min_dimension = half_extents[half_extents.min_axis()];
        let safe_margin = 0.1 * min_dimension;
        if safe_margin < self.collision_margin {
            self.set_margin(safe_margin);
        }
    }

    /// `btCylinderShape::getHalfExtentsWithoutMargin`.
    #[must_use]
    pub fn half_extents_without_margin(&self) -> Vec3 {
        self.implicit_shape_dimensions
    }
}

impl ConvexShape for CylinderShapeZ {
    /// `CylinderLocalSupportZ` (`btCylinderShape.cpp:183-220`).
    ///
    /// The index remapping is upstream's and is the trap in this function:
    /// `XX = 0, YY = 2, ZZ = 1`, so the *radial* plane is x/z of the direction
    /// mapped onto x/y of the result, and the axial component lands in `YY`,
    /// i.e. component 2. A `d = (0, 0, 1)` query therefore returns a point on
    /// the **rim**, `(radius, 0, halfHeight)`, not the centre of the cap.
    fn local_get_supporting_vertex_without_margin(&self, v: Vec3) -> Vec3 {
        const XX: usize = 0;
        const YY: usize = 2;
        const ZZ: usize = 1;
        const CYLINDER_UP_AXIS: usize = 2;

        let half_extents = self.half_extents_without_margin();
        let radius = half_extents[XX];
        let half_height = half_extents[CYLINDER_UP_AXIS];

        let mut tmp = Vec3::zero();
        let s = (v[XX] * v[XX] + v[ZZ] * v[ZZ]).sqrt();
        if s != 0.0 {
            let d = radius / s;
            tmp[XX] = v[XX] * d;
            tmp[YY] = if v[YY] < 0.0 {
                -half_height
            } else {
                half_height
            };
            tmp[ZZ] = v[ZZ] * d;
        } else {
            tmp[XX] = radius;
            tmp[YY] = if v[YY] < 0.0 {
                -half_height
            } else {
                half_height
            };
            tmp[ZZ] = 0.0;
        }
        tmp
    }

    fn margin(&self) -> Scalar {
        self.collision_margin
    }

    /// `btCylinderShape::setMargin` (`btCylinderShape.h:57-66`) -- the same
    /// re-expand/re-shrink `btBoxShape::setMargin` performs.
    fn set_margin(&mut self, collision_margin: Scalar) {
        let old_margin = Vec3::new(
            self.collision_margin,
            self.collision_margin,
            self.collision_margin,
        );
        let with_margin = self.implicit_shape_dimensions + old_margin;
        self.collision_margin = collision_margin;
        let new_margin = Vec3::new(collision_margin, collision_margin, collision_margin);
        self.implicit_shape_dimensions = with_margin - new_margin;
    }

    /// `btCylinderShape::getAabb` (`btCylinderShape.cpp:42-45`).
    fn get_aabb(&self, t: &Transform) -> (Vec3, Vec3) {
        transform_aabb_half_extents(self.half_extents_without_margin(), self.margin(), t)
    }
}

/// `btConeShapeZ` (`btConeShape.cpp:28-31`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConeShapeZ {
    /// `m_sinAngle` -- `radius / hypot(radius, height)`, the sine of the cone's
    /// half angle, cached at construction.
    sin_angle: Scalar,
    /// `m_radius`. Unlike the box and cylinder this is stored unshrunk: the
    /// cone's margin never enters its dimensions.
    radius: Scalar,
    /// `m_height` -- the **full** height. MoveIt passes `geom->length`
    /// (`bullet_utils.cpp:111-118`), and `coneLocalSupport` halves it itself.
    height: Scalar,
    collision_margin: Scalar,
}

/// `m_coneIndices` after `setConeUpIndex(2)` (`btConeShape.cpp:53-57`):
/// `[radial, axial, radial]` naming component indices, so the up axis is
/// component 2 and the second radial direction is component 1.
const CONE_Z_INDICES: [usize; 3] = [0, 2, 1];

impl ConeShapeZ {
    /// `btConeShapeZ(radius, height)` (`btConeShape.cpp:19-31`).
    ///
    /// `m_implicitShapeDimensions` is not carried: `setConeUpIndex` fills it
    /// (`btConeShape.cpp:62-64`), and nothing on the continuous-collision path
    /// reads it back -- the cone is absent from both non-virtual switches
    /// (`btConvexShape.cpp:133-300`, `:376`), and its `getAabb` is the base
    /// class's support-query one.
    #[must_use]
    pub fn new(radius: Scalar, height: Scalar) -> Self {
        Self {
            sin_angle: radius / (radius * radius + height * height).sqrt(),
            radius,
            height,
            collision_margin: CONVEX_DISTANCE_MARGIN,
        }
    }
}

impl ConvexShape for ConeShapeZ {
    /// `btConeShape::coneLocalSupport` (`btConeShape.cpp:67-102`).
    ///
    /// The apex test is `v[axial] > v.length() * m_sinAngle`, i.e. a
    /// comparison against the *unnormalized* length -- which is why callers
    /// that hand it a non-unit direction still get the right face.
    fn local_get_supporting_vertex_without_margin(&self, v: Vec3) -> Vec3 {
        let [r0, up, r1] = CONE_Z_INDICES;
        let half_height = self.height * 0.5;

        let mut tmp = Vec3::zero();
        if v[up] > v.length() * self.sin_angle {
            tmp[r0] = 0.0;
            tmp[up] = half_height;
            tmp[r1] = 0.0;
            return tmp;
        }

        let s = (v[r0] * v[r0] + v[r1] * v[r1]).sqrt();
        if s > SIMD_EPSILON {
            let d = self.radius / s;
            tmp[r0] = v[r0] * d;
            tmp[up] = -half_height;
            tmp[r1] = v[r1] * d;
        } else {
            tmp[r0] = 0.0;
            tmp[up] = -half_height;
            tmp[r1] = 0.0;
        }
        tmp
    }

    fn margin(&self) -> Scalar {
        self.collision_margin
    }

    /// `btConvexInternalShape::setMargin` (`btConvexInternalShape.h:102-105`).
    /// `btConeShape` does not override it, so this only assigns -- the cone's
    /// radius and height are already margin-free.
    fn set_margin(&mut self, margin: Scalar) {
        self.collision_margin = margin;
    }

    /// `btConvexInternalShape::getAabb` -- the cone overrides neither `getAabb`
    /// nor `getAabbNonVirtual`, so it pays for six support queries.
    fn get_aabb(&self, t: &Transform) -> (Vec3, Vec3) {
        self.get_aabb_slow(t)
    }
}

/// `btConvexHullShape` -- how every MoveIt mesh reaches Bullet
/// (`bullet_utils.cpp:131-153`).
#[derive(Clone, Debug, PartialEq)]
pub struct ConvexHullShape {
    /// `m_unscaledPoints`, in `addPoint` order. The order is not incidental:
    /// `maxDot` breaks ties toward the first vertex, so it decides which of
    /// several equally-extreme vertices a support query returns.
    unscaled_points: Vec<Vec3>,
    collision_margin: Scalar,
    local_aabb_min: Vec3,
    local_aabb_max: Vec3,
}

impl Default for ConvexHullShape {
    fn default() -> Self {
        Self::new()
    }
}

impl ConvexHullShape {
    /// `btConvexHullShape()` with no points -- what MoveIt constructs before
    /// feeding it `createConvexHull`'s output one vertex at a time
    /// (`bullet_utils.cpp:145-152`).
    ///
    /// `btPolyhedralConvexAabbCachingShape`'s constructor seeds the cached
    /// AABB inverted and invalid (`btPolyhedralConvexShape.cpp:492-498`); the
    /// zero-point `recalcLocalAabb` below writes `±m_collisionMargin` over it
    /// immediately, which is what the empty-hull state is here.
    #[must_use]
    pub fn new() -> Self {
        let mut shape = Self {
            unscaled_points: Vec::new(),
            collision_margin: CONVEX_DISTANCE_MARGIN,
            local_aabb_min: Vec3::zero(),
            local_aabb_max: Vec3::zero(),
        };
        shape.recalc_local_aabb();
        shape
    }

    /// `btConvexHullShape::addPoint(point, recalculateLocalAabb = true)`
    /// (`btConvexHullShape.cpp:50-55`).
    ///
    /// The recalculation uses the margin *as it stands now*, and nothing
    /// recomputes it later: `btConvexInternalShape::setMargin` writes
    /// `m_collisionMargin` and returns (`btConvexInternalShape.h:102-105`).
    /// MoveIt adds every vertex first and calls `setMargin(0)` afterwards
    /// (`bullet_utils.cpp:577`), so a MoveIt hull's cached AABB stays inflated
    /// by [`CONVEX_DISTANCE_MARGIN`] on each side even though its margin is
    /// zero. That stale AABB is what the broadphase sees, so it decides which
    /// pairs reach the narrow phase at all.
    pub fn add_point(&mut self, point: Vec3) {
        self.unscaled_points.push(point);
        self.recalc_local_aabb();
    }

    /// `btPolyhedralConvexAabbCachingShape::recalcLocalAabb`
    /// (`btPolyhedralConvexShape.cpp:505-534`).
    pub fn recalc_local_aabb(&mut self) {
        const DIRECTIONS: [Vec3; 6] = [
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(-1.0, 0.0, 0.0),
            Vec3::new(0.0, -1.0, 0.0),
            Vec3::new(0.0, 0.0, -1.0),
        ];

        // `batchedUnitVectorGetSupportingVertexWithoutMargin`
        // (`btConvexHullShape.cpp:75-96`) leaves the output vector untouched
        // for an empty point set, and upstream's caller seeded it at zero.
        let supporting =
            DIRECTIONS.map(|dir| self.local_get_supporting_vertex_without_margin_or_zero(dir));

        for i in 0..3 {
            self.local_aabb_max[i] = supporting[i][i] + self.collision_margin;
            self.local_aabb_min[i] = supporting[i + 3][i] - self.collision_margin;
        }
    }

    fn local_get_supporting_vertex_without_margin_or_zero(&self, vec: Vec3) -> Vec3 {
        vec.max_dot(&self.unscaled_points)
            .map_or_else(Vec3::zero, |(index, _)| self.unscaled_points[index])
    }

    /// The hull's vertices, in `addPoint` order.
    #[must_use]
    pub fn unscaled_points(&self) -> &[Vec3] {
        &self.unscaled_points
    }

    /// `m_localAabbMin`/`m_localAabbMax` -- the cached, possibly stale local
    /// AABB. See [`ConvexHullShape::add_point`].
    #[must_use]
    pub fn local_aabb(&self) -> (Vec3, Vec3) {
        (self.local_aabb_min, self.local_aabb_max)
    }
}

impl ConvexShape for ConvexHullShape {
    /// `btConvexHullShape::localGetSupportingVertexWithoutMargin`
    /// (`btConvexHullShape.cpp:57-70`).
    ///
    /// Upstream returns `(0, 0, 0)` for an empty hull, which is what
    /// `maxDot`'s `None` maps to here.
    fn local_get_supporting_vertex_without_margin(&self, vec: Vec3) -> Vec3 {
        self.local_get_supporting_vertex_without_margin_or_zero(vec)
    }

    fn margin(&self) -> Scalar {
        self.collision_margin
    }

    /// `btConvexInternalShape::setMargin` -- assignment only, and in
    /// particular no `recalcLocalAabb`. See [`ConvexHullShape::add_point`].
    fn set_margin(&mut self, margin: Scalar) {
        self.collision_margin = margin;
    }

    /// `btPolyhedralConvexAabbCachingShape::getAabb` ->
    /// `getNonvirtualAabb(trans, ..., getMargin())`
    /// (`btPolyhedralConvexShape.h:92-97`, `.cpp:500-503`).
    ///
    /// Note the two margins in play: the cached local AABB already carries
    /// whatever margin was current when the last point was added, and this
    /// adds `getMargin()` on top.
    fn get_aabb(&self, t: &Transform) -> (Vec3, Vec3) {
        transform_aabb(self.local_aabb_min, self.local_aabb_max, self.margin(), t)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::linear_math::Matrix3;

    /// The identity pose, which is the one the `bullet_support` oracle op
    /// reports its AABB at.
    const IDENTITY: Transform = Transform::new(Matrix3::identity(), Vec3::zero());

    /// The margin round trip MoveIt drives: `btBoxShape` shrinks by the
    /// default margin at construction, `setSafeMargin` may shrink the margin
    /// further, and `setMargin(0)` restores the half extents exactly.
    ///
    /// The expected values are the `bullet_support` oracle op's, for a
    /// `shapes::Box` of size 0.1 x 0.2 x 0.3: `margin` 0, support `±(0.05,
    /// 0.1, 0.15)`.
    #[test]
    fn box_setmargin_zero_restores_the_half_extents() {
        let mut shape = BoxShape::new(Vec3::new(0.05, 0.1, 0.15));
        // `setSafeMargin` clamps to a tenth of the smallest half extent, which
        // is below the 0.04 default. Spelled as the product rather than as
        // `0.005`: in f32 that product is 0.0050000004, and writing the decimal
        // would be asserting a value the code cannot produce.
        assert_eq!(shape.margin(), 0.1_f32 * 0.05);
        shape.set_margin(0.0);
        assert_eq!(shape.margin(), 0.0);
        assert_eq!(
            shape.half_extents_without_margin(),
            Vec3::new(0.05, 0.1, 0.15)
        );
    }

    /// `btFsel` at the boundary, on the shape that shows it: a direction with
    /// a zero component still picks the positive face.
    #[test]
    fn box_support_sends_a_zero_direction_component_to_the_positive_face() {
        let mut shape = BoxShape::new(Vec3::new(0.05, 0.1, 0.15));
        shape.set_margin(0.0);
        assert_eq!(
            shape.local_get_supporting_vertex_without_margin(Vec3::new(0.0, 1.0, 0.0)),
            Vec3::new(0.05, 0.1, 0.15)
        );
        assert_eq!(
            shape.local_get_supporting_vertex_without_margin(Vec3::new(-1.0, -1.0, -1.0)),
            Vec3::new(-0.05, -0.1, -0.15)
        );
    }

    /// A sphere is all margin, and `setMargin` cannot change that.
    #[test]
    fn sphere_margin_is_the_radius_and_setmargin_does_not_shrink_it() {
        let mut shape = SphereShape::new(0.05);
        assert_eq!(shape.margin(), 0.05);
        assert_eq!(
            shape.local_get_supporting_vertex_without_margin(Vec3::new(1.0, 0.0, 0.0)),
            Vec3::zero()
        );
        shape.set_margin(0.0);
        assert_eq!(shape.margin(), 0.05);
        assert_eq!(
            shape.local_get_supporting_vertex(Vec3::new(1.0, 0.0, 0.0)),
            Vec3::new(0.05, 0.0, 0.0)
        );
    }

    /// The oracle's cylinder case: radius 0.05, length 0.3, `d = (0, 0, 1)`
    /// returns the rim point `(0.05, 0, 0.15)` -- the index remap in
    /// `CylinderLocalSupportZ`, not the cap centre a reader expects.
    #[test]
    fn cylinder_z_support_along_the_axis_lands_on_the_rim() {
        let mut shape = CylinderShapeZ::new(Vec3::new(0.05, 0.05, 0.15));
        shape.set_margin(0.0);
        assert_eq!(shape.margin(), 0.0);
        assert_eq!(
            shape.local_get_supporting_vertex_without_margin(Vec3::new(0.0, 0.0, 1.0)),
            Vec3::new(0.05, 0.0, 0.15)
        );
        assert_eq!(
            shape.local_get_supporting_vertex_without_margin(Vec3::new(1.0, 0.0, 0.0)),
            Vec3::new(0.05, 0.0, 0.15)
        );
        // Radially along -y, and the axial component ties: `v[YY]` is `v.z`,
        // which is 0 here, and the test is `< 0`, so the tie goes to +z.
        assert_eq!(
            shape.local_get_supporting_vertex_without_margin(Vec3::new(0.0, -1.0, 0.0)),
            Vec3::new(0.0, -0.05, 0.15)
        );
        assert_eq!(
            shape.local_get_supporting_vertex_without_margin(Vec3::new(0.0, -1.0, -1.0)),
            Vec3::new(0.0, -0.05, -0.15)
        );
    }

    /// The oracle's cone case: radius 0.05, length 0.3, `d = (0, 0, 1)` gives
    /// the apex at `(0, 0, 0.15)` -- half the constructor's height.
    #[test]
    fn cone_z_support_along_the_axis_is_the_apex_at_half_the_height() {
        let mut shape = ConeShapeZ::new(0.05, 0.3);
        shape.set_margin(0.0);
        assert_eq!(
            shape.local_get_supporting_vertex_without_margin(Vec3::new(0.0, 0.0, 1.0)),
            Vec3::new(0.0, 0.0, 0.15)
        );
        assert_eq!(
            shape.local_get_supporting_vertex_without_margin(Vec3::new(0.0, 0.0, -1.0)),
            Vec3::new(0.0, 0.0, -0.15)
        );
        assert_eq!(
            shape.local_get_supporting_vertex_without_margin(Vec3::new(1.0, 0.0, 0.0)),
            Vec3::new(0.05, 0.0, -0.15)
        );
    }

    /// The apex test compares against `v.length() * sinAngle`, so a direction
    /// only slightly off-axis still picks the base rim. Both sides of the
    /// boundary, since a test on one side passes against `>=` as well as `>`.
    #[test]
    fn cone_z_apex_test_uses_the_half_angle() {
        let shape = ConeShapeZ::new(0.05, 0.3);
        // The cone's own surface direction, at the half angle: sin = r/hypot.
        let hypot = (0.05_f32 * 0.05 + 0.3 * 0.3).sqrt();
        let on_surface = Vec3::new(0.3 / hypot, 0.0, 0.05 / hypot);
        assert_eq!(
            shape
                .local_get_supporting_vertex_without_margin(on_surface)
                .z,
            -0.15
        );
        let steeper = Vec3::new(0.05 / hypot, 0.0, 0.3 / hypot);
        assert_eq!(
            shape.local_get_supporting_vertex_without_margin(steeper).z,
            0.15
        );
    }

    /// The stale-AABB quirk: a hull built the way MoveIt builds one reports an
    /// identity-pose AABB inflated by `CONVEX_DISTANCE_MARGIN` per side, even
    /// though its margin is zero by then. Written as a prediction the
    /// `bullet_support` oracle op could refute, and put to it -- on this exact
    /// point set the C++ answers `±0.14` with `margin: 0.0`, so it stands.
    #[test]
    fn hull_keeps_the_construction_margin_in_its_cached_aabb() {
        let mut shape = ConvexHullShape::new();
        for point in [
            Vec3::new(-0.1, -0.1, -0.1),
            Vec3::new(0.1, -0.1, -0.1),
            Vec3::new(0.0, 0.1, -0.1),
            Vec3::new(0.0, 0.0, 0.1),
        ] {
            shape.add_point(point);
        }
        shape.set_margin(0.0);
        assert_eq!(shape.margin(), 0.0);

        let (local_min, local_max) = shape.local_aabb();
        assert_eq!(
            local_max,
            Vec3::new(0.1, 0.1, 0.1) + Vec3::new(0.04, 0.04, 0.04)
        );
        assert_eq!(
            local_min,
            Vec3::new(-0.1, -0.1, -0.1) - Vec3::new(0.04, 0.04, 0.04)
        );

        let (world_min, world_max) = shape.get_aabb(&IDENTITY);
        assert_eq!((world_min, world_max), (local_min, local_max));
    }

    /// Tie-breaking is `addPoint` order, so two hulls over the same point set
    /// can return different support vertices. Not a defect to fix -- it is
    /// what `maxDot`'s strict `>` does, and reproducing it is the point.
    #[test]
    fn hull_support_ties_go_to_the_earlier_point() {
        let build = |points: [Vec3; 2]| {
            let mut shape = ConvexHullShape::new();
            for point in points {
                shape.add_point(point);
            }
            shape.set_margin(0.0);
            shape
        };
        let a = Vec3::new(1.0, 1.0, 0.0);
        let b = Vec3::new(1.0, -1.0, 0.0);
        let dir = Vec3::new(1.0, 0.0, 0.0);
        assert_eq!(
            build([a, b]).local_get_supporting_vertex_without_margin(dir),
            a
        );
        assert_eq!(
            build([b, a]).local_get_supporting_vertex_without_margin(dir),
            b
        );
    }

    /// A cone has no `getAabb` of its own, so it goes through six support
    /// queries. Checked against the closed form for the identity pose: a
    /// z-aligned cone spans `±radius` radially and `±height/2` axially.
    #[test]
    fn cone_aabb_comes_from_support_queries() {
        let mut shape = ConeShapeZ::new(0.05, 0.3);
        shape.set_margin(0.0);
        let (min, max) = shape.get_aabb(&IDENTITY);
        assert_eq!(max, Vec3::new(0.05, 0.05, 0.15));
        assert_eq!(min, Vec3::new(-0.05, -0.05, -0.15));
    }

    /// A sphere's AABB ignores the basis entirely.
    #[test]
    fn sphere_aabb_ignores_the_basis() {
        let shape = SphereShape::new(0.05);
        let s = core::f32::consts::FRAC_1_SQRT_2;
        let rotated = Transform::new(
            Matrix3::from_rows(
                Vec3::new(s, -s, 0.0),
                Vec3::new(s, s, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
            ),
            Vec3::new(1.0, 2.0, 3.0),
        );
        let (min, max) = shape.get_aabb(&rotated);
        assert_eq!(min, Vec3::new(0.95, 1.95, 2.95));
        assert_eq!(max, Vec3::new(1.05, 2.05, 3.05));
    }

    /// The non-virtual support path normalizes before querying; the virtual
    /// one does not. On a cylinder the two disagree in the last bits, which is
    /// why GJK's choice of path is not a detail.
    #[test]
    fn non_virtual_support_normalizes_the_direction_first() {
        let mut shape = CylinderShapeZ::new(Vec3::new(0.05, 0.05, 0.15));
        shape.set_margin(0.0);
        let dir = Vec3::new(3.0, 0.0, 4.0);
        let expected = shape.local_get_supporting_vertex_without_margin(dir.normalize());
        assert_eq!(shape.local_get_support_vertex_non_virtual(dir), expected);
    }

    /// A degenerate direction falls back to `(-1, -1, -1)` before
    /// normalization, on both support paths.
    #[test]
    fn a_zero_direction_falls_back_to_minus_ones() {
        let shape = SphereShape::new(0.05);
        let unit = Vec3::new(-1.0, -1.0, -1.0).normalize();
        assert_eq!(
            shape.local_get_support_vertex_non_virtual(Vec3::zero()),
            unit * 0.05
        );
        assert_eq!(shape.local_get_supporting_vertex(Vec3::zero()), unit * 0.05);
    }
}
