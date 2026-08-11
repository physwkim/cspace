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
//! geometry onto one of six convex shapes -- `btBoxShape`, `btSphereShape`,
//! `btCylinderShapeZ`, `btConeShapeZ`, `btConvexHullShape`,
//! `btTriangleShapeEx` -- or onto a compound of them. Those six are here;
//! nothing else in Bullet's shape library is.
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
//! `btCapsuleShape`, `btConvexPointCloudShape` and the
//! `btConvexInternalAabbCachingShape` family: `createShapePrimitive` builds
//! none of them for the collision-object types `CollisionEnvBullet` requests.
//!
//! [`TriangleShapeEx`] is here because that arm *is* requested.
//! `addAttachedObjects` fills its whole `collision_object_types` vector with
//! `USE_SHAPE_TYPE` (`collision_env_bullet.cpp:345-346`) rather than choosing
//! per shape the way `addToManager` (`:257-267`) and `addLinkAsCollisionObject`
//! (`:417-425`) do, so an attached body whose shape is a mesh reaches
//! `createShapePrimitive`'s triangle-soup branch and comes back as a compound
//! of `btTriangleShapeEx`.

use std::any::Any;
use std::borrow::Cow;

use crate::broadphase_proxy::BroadphaseNativeType;
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
///
/// # `Send + Sync`
///
/// A shape is immutable data behind a shared pointer -- `btCollisionShape*` in
/// C++, `Arc<dyn ConvexShape>` in [`crate::compound::Shape`] -- and a
/// collision environment is the sort of thing a planner holds behind an `Arc`
/// and queries from several threads. Without this bound `Arc<dyn ConvexShape>`
/// is neither `Send` nor `Sync` whatever the concrete shape is, which makes
/// every structure built over it thread-local by accident rather than by
/// decision. Every shape here is plain data and satisfies it; the bound is
/// what stops an implementor reaching for interior mutability to make a shape
/// re-poseable in place.
/// # `Any`
///
/// `addCastSingleResult` recovers the swept shape by
/// `static_cast<const CastHullShape*>(first_col_obj_wrap->getCollisionShape())`
/// (`bullet_utils.hpp:471`) -- a downcast to a type that is not in this crate,
/// on a pointer whose static type is the base. [`ConvexShape::as_any`] is that
/// downcast's only possible spelling here, and it is strictly the safer one:
/// upstream's `static_cast` reinterprets whatever it is given, while this one
/// fails when the shape is not the type asked for.
pub trait ConvexShape: Send + Sync + Any {
    /// The concrete shape behind the trait object, for the one downcast the
    /// continuous path performs; see the trait docs.
    ///
    /// Required rather than defaulted because a default body would need
    /// `Self: Sized`, which is exactly what a trait object is not.
    fn as_any(&self) -> &dyn Any;

    /// `btConvexShape::localGetSupportingVertexWithoutMargin`.
    fn local_get_supporting_vertex_without_margin(&self, vec: Vec3) -> Vec3;

    /// `btCollisionShape::getMargin`.
    fn margin(&self) -> Scalar;

    /// `btCollisionShape::getShapeType` -- the value each shape's constructor
    /// writes into `m_shapeType`, and the only thing
    /// `btCollisionDispatcher::findAlgorithm` looks at.
    ///
    /// On the trait rather than derived from the concrete type because that
    /// is where upstream keeps it: a shape this crate does not define -- and
    /// MoveIt's `CastHullShape` is one -- reports its own
    /// `CUSTOM_CONVEX_SHAPE_TYPE`, and the dispatch has to see that value and
    /// not a Rust type name.
    fn shape_type(&self) -> BroadphaseNativeType;

    /// `btCollisionShape::setMargin` -- pure virtual upstream
    /// (`btCollisionShape.h:118`), and three of the six shapes here override it
    /// differently: `btBoxShape`, `btSphereShape` and `btCylinderShape` each
    /// declare their own, while the cone, the hull and the triangle inherit
    /// `btConvexInternalShape`'s plain assignment. It belongs on the trait for
    /// the same reason it is virtual there: `createShapePrimitive` calls it
    /// through the base pointer on whatever shape it has just built
    /// (`bullet_utils.cpp:577`, `:587`, `:599`), and the shape decides what
    /// that means.
    fn set_margin(&mut self, margin: Scalar);

    /// `btCollisionShape::getAabb` -- the shape's world AABB under `t`.
    fn get_aabb(&self, t: &Transform) -> (Vec3, Vec3);

    /// `dynamic_cast<const btPolyhedralConvexShape*>` succeeding, together with
    /// the `getNumVertices`/`getVertex(i, pt)` pair the caller reads when it
    /// does (`btPolyhedralConvexShape.h:64-65`).
    ///
    /// One method rather than a cast test plus a count plus an indexed getter,
    /// because upstream's sole caller in this port's scope --
    /// `getAverageSupport` (`bullet_utils.hpp:351-377`) -- consumes all three
    /// in one loop, and three separately answerable questions can disagree
    /// where one cannot.
    ///
    /// `None` is a failed cast, and is what the sphere, the cylinder and the
    /// cone report: `getAverageSupport` then takes its `else` arm and asks for
    /// a single support point. `Some(&[])` is a *successful* cast onto a
    /// polyhedron with no vertices, which is a different state -- upstream
    /// enters the branch, loops zero times, and divides `pt_sum` by a zero
    /// `pt_count`. The two must stay distinguishable, so this is not
    /// `Option`-flattened into an empty slice.
    ///
    /// These are `getVertex`'s vertices, not the shape's support points:
    /// `btBoxShape` synthesises its eight corners from the half extents
    /// **with** margin (`btBoxShape.h:131-139`), so on a box whose margin is
    /// nonzero they lie outside what
    /// [`ConvexShape::local_get_supporting_vertex_without_margin`] returns.
    fn polyhedral_vertices(&self) -> Option<Cow<'_, [Vec3]>> {
        None
    }

    /// `btConvexInternalShape::localGetSupportingVertex`
    /// (`btConvexInternalShape.cpp:50-67`).
    ///
    /// `btCylinderShape` (`btCylinderShape.h:68-85`), `btConeShape`
    /// (`btConeShape.cpp:117-131`) and `btConvexHullShape`
    /// (`btConvexHullShape.cpp:99-114`) each override this with the same body,
    /// and `btSphereShape`'s (`btSphereShape.cpp:37-50`) differs only by
    /// dropping the `margin != 0` guard, which cannot be observed -- with a
    /// zero radius its margin is zero and the term it adds is `0 * unit`, i.e.
    /// the vector the guard would have skipped adding.
    ///
    /// [`BoxShape`] is the one that is genuinely different and therefore the
    /// one that overrides this here: it adds the margin to each half extent
    /// rather than along the query direction, which lands on a box corner
    /// instead of out along `vec`. On a zero-margin box the two agree, which is
    /// why `createShapePrimitive`'s `setMargin(0)` hides the difference on
    /// every shape MoveIt builds -- and why an enumeration of the overrides can
    /// skip it and still look complete.
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
    /// it, which among the six here is [`ConeShapeZ`] alone.
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

    /// `btBoxShape::getHalfExtentsWithMargin` (`btBoxShape.h:34-40`) -- the
    /// stored dimensions re-inflated by the current margin, which is the box a
    /// support query answers for once the margin term is added back.
    #[must_use]
    pub fn half_extents_with_margin(&self) -> Vec3 {
        let margin = Vec3::new(self.margin(), self.margin(), self.margin());
        self.half_extents_without_margin() + margin
    }
}

impl ConvexShape for BoxShape {
    fn as_any(&self) -> &dyn Any {
        self
    }

    /// `btBoxShape.cpp:20`.
    fn shape_type(&self) -> BroadphaseNativeType {
        BroadphaseNativeType::BOX_SHAPE
    }

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

    /// `btBoxShape::localGetSupportingVertex` (`btBoxShape.h:47-56`) -- the
    /// half extents **with** margin, selected per axis by [`bt_fsel`].
    ///
    /// Not the base class's `without_margin + margin * unit(vec)`: this puts
    /// the point at the corner of the inflated box, which is further out along
    /// every axis than the direction-aligned version and is a different vector
    /// unless `vec` happens to be an axis. The two agree only when the margin
    /// is zero.
    fn local_get_supporting_vertex(&self, vec: Vec3) -> Vec3 {
        let half_extents = self.half_extents_with_margin();
        Vec3::new(
            bt_fsel(vec.x, half_extents.x, -half_extents.x),
            bt_fsel(vec.y, half_extents.y, -half_extents.y),
            bt_fsel(vec.z, half_extents.z, -half_extents.z),
        )
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

    /// `btBoxShape::getNumVertices` = 8 and `getVertex` (`btBoxShape.h:121-139`).
    ///
    /// The index's low three bits pick one sign per axis. Written as upstream
    /// writes it -- `h * (1 - bit) - h * bit` rather than a sign flip -- because
    /// the two differ for a zero half extent: the subtraction yields `+0`, a
    /// negation yields `-0`.
    fn polyhedral_vertices(&self) -> Option<Cow<'_, [Vec3]>> {
        let half_extents = self.half_extents_with_margin();
        let component = |extent: Scalar, bit: usize| {
            let bit = bit as Scalar;
            extent * (1.0 - bit) - extent * bit
        };
        Some(Cow::Owned(
            (0..8)
                .map(|i| {
                    Vec3::new(
                        component(half_extents.x, i & 1),
                        component(half_extents.y, (i & 2) >> 1),
                        component(half_extents.z, (i & 4) >> 2),
                    )
                })
                .collect(),
        ))
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
    fn as_any(&self) -> &dyn Any {
        self
    }

    /// `btSphereShape.h:31`.
    fn shape_type(&self) -> BroadphaseNativeType {
        BroadphaseNativeType::SPHERE_SHAPE
    }

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
    fn as_any(&self) -> &dyn Any {
        self
    }

    /// `btCylinderShape.cpp:27` -- `btCylinderShapeZ`'s
    /// constructor only sets `m_upAxis` (`:36-40`), so the Z variant reports
    /// the same type as the base.
    fn shape_type(&self) -> BroadphaseNativeType {
        BroadphaseNativeType::CYLINDER_SHAPE
    }

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
    fn as_any(&self) -> &dyn Any {
        self
    }

    /// `btConeShape.cpp:22` -- `btConeShapeZ` only calls
    /// `setConeUpIndex(2)` (`:28-31`), so the Z variant reports the base's
    /// type.
    fn shape_type(&self) -> BroadphaseNativeType {
        BroadphaseNativeType::CONE_SHAPE
    }

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
    fn as_any(&self) -> &dyn Any {
        self
    }

    /// `btConvexHullShape.cpp:30`.
    fn shape_type(&self) -> BroadphaseNativeType {
        BroadphaseNativeType::CONVEX_HULL_SHAPE
    }

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

    /// `btConvexHullShape::getNumVertices` (`btConvexHullShape.cpp:130-133`)
    /// and `getVertex` (`:148-151`), which is `getScaledPoint(i)`.
    ///
    /// No scaling term: `btConvexInternalShape`'s constructor seeds
    /// `m_localScaling` at `(1,1,1)` (`btConvexInternalShape.cpp:19`), and
    /// MoveIt's only `setLocalScaling` is `CastHullShape`'s no-op override
    /// (`bullet_utils.hpp:291`), so `getScaledPoint` returns the unscaled point
    /// and the field is not carried here.
    fn polyhedral_vertices(&self) -> Option<Cow<'_, [Vec3]>> {
        Some(Cow::Borrowed(&self.unscaled_points))
    }
}

/// `btTriangleShapeEx` (`btTriangleShapeEx.h:126-167`) over its
/// `btTriangleShape` base (`btTriangleShape.h:23-173`).
///
/// The subclass, not the base, because the subclass is what
/// `createShapePrimitive` builds (`bullet_utils.cpp:175`) and it overrides
/// `getAabb`: `btTriangleShapeEx::getAabb` (`:140-149`) boxes the three
/// transformed corners, where `btTriangleShape::getAabb` (`btTriangleShape.h:
/// 60-64`) calls `getAabbSlow` and pays six support queries for it. Everything
/// else the narrow phase touches -- the support function, the margin, the
/// shape type, the polyhedral vertices -- is the base's, inherited unchanged.
///
/// The two `getAabb`s agree bit for bit at margin zero, which is the only
/// margin MoveIt runs: `getAabbSlow`'s support query on a polytope returns the
/// corner the subclass boxes. They part at a nonzero margin, where the base
/// adds it twice -- once inside `localGetSupportingVertex`, once in
/// `getAabbSlow`'s own `+ margin`. `shapeaabb_tri_*` against
/// `shapeaabb_tribase_*` in this module's `TRIANGLE_AABB_REFERENCE` is where
/// that is measured rather than assumed. This port carries the subclass's form because
/// that is the one on the path, not because a fixture here could tell.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TriangleShapeEx {
    /// `m_vertices1`.
    vertices: [Vec3; 3],
    /// `btConvexInternalShape::m_collisionMargin`, which the base constructor
    /// seeds at [`CONVEX_DISTANCE_MARGIN`] and `createShapePrimitive`
    /// immediately overwrites with `BULLET_MARGIN`.
    collision_margin: Scalar,
}

impl TriangleShapeEx {
    /// `btTriangleShapeEx(p0, p1, p2)` (`btTriangleShapeEx.h:133-135`),
    /// forwarding to `btTriangleShape(p0, p1, p2)` (`btTriangleShape.h:86-92`).
    #[must_use]
    pub const fn new(p0: Vec3, p1: Vec3, p2: Vec3) -> Self {
        Self {
            vertices: [p0, p1, p2],
            collision_margin: CONVEX_DISTANCE_MARGIN,
        }
    }

    /// `m_vertices1` -- the three corners, in construction order.
    #[must_use]
    pub const fn vertices(&self) -> &[Vec3; 3] {
        &self.vertices
    }
}

impl ConvexShape for TriangleShapeEx {
    fn as_any(&self) -> &dyn Any {
        self
    }

    /// `btTriangleShape.h:89` -- the subclass sets no type of its own.
    fn shape_type(&self) -> BroadphaseNativeType {
        BroadphaseNativeType::TRIANGLE_SHAPE
    }

    /// `btTriangleShape::localGetSupportingVertexWithoutMargin`
    /// (`btTriangleShape.h:66-70`).
    ///
    /// `maxAxis` breaks a tie toward the *lower* index (`btVector3.h:477-480`
    /// compares with `<`), so two corners at the same dot product resolve to
    /// the one the triangle was built from first. Not decoration: a mesh face
    /// seen edge-on has two corners at the same dot, and which one comes back
    /// is the witness point.
    fn local_get_supporting_vertex_without_margin(&self, vec: Vec3) -> Vec3 {
        let dots = vec.dot3(self.vertices[0], self.vertices[1], self.vertices[2]);
        self.vertices[dots.max_axis()]
    }

    fn margin(&self) -> Scalar {
        self.collision_margin
    }

    /// `btConvexInternalShape::setMargin` (`btConvexInternalShape.h:102-105`) --
    /// assignment, inherited unchanged. Unlike [`BoxShape`]'s there is no
    /// implicit dimension to re-derive: the corners are stored as given.
    fn set_margin(&mut self, margin: Scalar) {
        self.collision_margin = margin;
    }

    /// `btTriangleShapeEx::getAabb` (`btTriangleShapeEx.h:140-149`) --
    /// `btAABB(tv0, tv1, tv2, m_collisionMargin)`
    /// (`btBoxCollision.h:238-257`).
    ///
    /// `BT_MIN`/`BT_MAX` (`btBoxCollision.h:37-38`) are ternaries on `<` and
    /// `>`, which is not [`Scalar::min`]/[`Scalar::max`]: those return the
    /// non-`NaN` operand, where a ternary propagates whichever side the
    /// comparison lands on. A mesh with a `NaN` vertex is the case that tells
    /// them apart, so the comparisons are written out.
    fn get_aabb(&self, t: &Transform) -> (Vec3, Vec3) {
        // `BT_MAX(a, b)` is `(a < b ? b : a)` and `BT_MIN(a, b)` is
        // `(a > b ? b : a)`, nested as `BT_MAX3(a, b, c) = BT_MAX(a, BT_MAX(b, c))`.
        let bt_max = |a: Scalar, b: Scalar| if a < b { b } else { a };
        let bt_min = |a: Scalar, b: Scalar| if a > b { b } else { a };

        let tv = self.vertices.map(|v| t.transform_point(v));
        let mut min = Vec3::zero();
        let mut max = Vec3::zero();
        for i in 0..3 {
            min[i] = bt_min(tv[0][i], bt_min(tv[1][i], tv[2][i])) - self.collision_margin;
            max[i] = bt_max(tv[0][i], bt_max(tv[1][i], tv[2][i])) + self.collision_margin;
        }
        (min, max)
    }

    /// `btTriangleShape::getNumVertices` (`:30-33`) and `getVertex` (`:44-47`).
    ///
    /// A successful `dynamic_cast<const btPolyhedralConvexShape*>`, so
    /// `getAverageSupport` averages the corners that tie on the support value
    /// rather than taking a single support point -- which for a triangle lying
    /// flat against a face is two of them, and is what puts the swept contact
    /// mid-edge instead of on a corner.
    fn polyhedral_vertices(&self) -> Option<Cow<'_, [Vec3]>> {
        Some(Cow::Borrowed(&self.vertices))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::linear_math::Matrix3;
    use crate::probe_fixture::{diff_vec3, probe_shapes, probe_triangles, row};

    /// The `shapetype_*` rows of `tools/bullet-epa-reference/build.sh`'s
    /// stdout -- `getShapeType()` on the shapes `probe.cpp` builds.
    ///
    /// Separate from `broadphase_proxy`'s rows, which read the enum: these
    /// read what a *constructed shape* reports, and a shape wired to the
    /// wrong entry of a correctly-numbered enum is exactly what those rows
    /// cannot see.
    const SHAPE_TYPE_REFERENCE: &str = "\
shapetype_unit_box|0
shapetype_sphere|8
shapetype_cyl|13
shapetype_cone|11
shapetype_hull|4
shapetype_tri|1
";

    #[test]
    fn bullet_reference_shape_type() {
        let (unit_box, _, _, sphere, _, cyl, cone, hull) = probe_shapes();
        let (tri, _) = probe_triangles();
        let ports: [(&str, BroadphaseNativeType); 6] = [
            ("unit_box", unit_box.shape_type()),
            ("sphere", sphere.shape_type()),
            ("cyl", cyl.shape_type()),
            ("cone", cone.shape_type()),
            ("hull", hull.shape_type()),
            ("tri", tri.shape_type()),
        ];

        let mut bad = Vec::new();
        for (name, port) in ports {
            let prefix = format!("shapetype_{name}|");
            let line = SHAPE_TYPE_REFERENCE
                .lines()
                .find(|l| l.starts_with(&prefix))
                .unwrap_or_else(|| panic!("{name}: no such row"));
            let want: i32 = line.split('|').nth(1).unwrap().parse().unwrap();
            if port.0 != want {
                bad.push(format!("{name}: port {}, bullet {want}", port.0));
            }
        }
        assert!(bad.is_empty(), "{}", bad.join("\n"));
    }

    /// The `polycast_*` / `polyvert_*` rows of `build.sh`'s stdout -- which
    /// shapes `getAverageSupport`'s `dynamic_cast` accepts, and the vertices
    /// `getVertex` then hands it.
    ///
    /// `margin_box` earns its rows by reporting `±0.5` while its support
    /// function without margin returns `±0.46`: the cast's vertices are the
    /// half extents *with* margin, and no other row here separates the two.
    const POLYHEDRAL_REFERENCE: &str = "\
polycast_unit_box|1|8
polyvert_unit_box_0|0.5|0.5|0.5
polyvert_unit_box_1|-0.5|0.5|0.5
polyvert_unit_box_2|0.5|-0.5|0.5
polyvert_unit_box_3|-0.5|-0.5|0.5
polyvert_unit_box_4|0.5|0.5|-0.5
polyvert_unit_box_5|-0.5|0.5|-0.5
polyvert_unit_box_6|0.5|-0.5|-0.5
polyvert_unit_box_7|-0.5|-0.5|-0.5
polycast_flat_box|1|8
polyvert_flat_box_0|0.400000006|0.699999988|0.25
polyvert_flat_box_1|-0.400000006|0.699999988|0.25
polyvert_flat_box_2|0.400000006|-0.699999988|0.25
polyvert_flat_box_3|-0.400000006|-0.699999988|0.25
polyvert_flat_box_4|0.400000006|0.699999988|-0.25
polyvert_flat_box_5|-0.400000006|0.699999988|-0.25
polyvert_flat_box_6|0.400000006|-0.699999988|-0.25
polyvert_flat_box_7|-0.400000006|-0.699999988|-0.25
polycast_margin_box|1|8
polyvert_margin_box_0|0.5|0.5|0.5
polyvert_margin_box_1|-0.5|0.5|0.5
polyvert_margin_box_2|0.5|-0.5|0.5
polyvert_margin_box_3|-0.5|-0.5|0.5
polyvert_margin_box_4|0.5|0.5|-0.5
polyvert_margin_box_5|-0.5|0.5|-0.5
polyvert_margin_box_6|0.5|-0.5|-0.5
polyvert_margin_box_7|-0.5|-0.5|-0.5
polycast_sphere|0|0
polycast_cyl|0|0
polycast_cone|0|0
polycast_hull|1|8
polyvert_hull_0|0.300000012|0.200000003|0.100000001
polyvert_hull_1|-0.300000012|0.200000003|0.100000001
polyvert_hull_2|0.300000012|-0.200000003|0.100000001
polyvert_hull_3|-0.300000012|-0.200000003|0.100000001
polyvert_hull_4|0.300000012|0.200000003|-0.100000001
polyvert_hull_5|-0.300000012|0.200000003|-0.100000001
polyvert_hull_6|0.300000012|-0.200000003|-0.100000001
polyvert_hull_7|-0.300000012|-0.200000003|-0.100000001
polycast_tri|1|3
polyvert_tri_0|0|0|0
polyvert_tri_1|1|0|0
polyvert_tri_2|0|1|0
";

    /// The `support_*` rows -- the *virtual* `localGetSupportingVertex`, beside
    /// `localGetSupportingVertexWithoutMargin` and the margin itself.
    ///
    /// Every other fixture in this crate goes through GJK, which calls
    /// `localGetSupportVertexNonVirtual`; nothing reached the virtual with a
    /// nonzero margin, and that is how the box's override stayed unported.
    /// `unit_box_diag` sits beside `margin_box_diag` because a zero margin
    /// makes both formulas agree -- a fixture built only from MoveIt's
    /// zero-margin shapes cannot see the difference at all.
    const SUPPORT_REFERENCE: &str = "\
support_unit_box_diag|0.5|0.5|0.5|0.5|0.5|0.5|0
support_margin_box_diag|0.5|0.5|0.5|0.460000008|0.460000008|0.460000008|0.0399999991
support_margin_box_axis|0.5|0.5|0.5|0.460000008|0.460000008|0.460000008|0.0399999991
support_margin_box_diag_unit|0.5|0.5|0.5|0.460000008|0.460000008|0.460000008|0.0399999991
support_sphere_diag|0.288675129|0.288675129|0.288675129|0|0|0|0.5
support_cyl_diag|0.212132052|0.212132052|0.5|0.212132052|0.212132052|0.5|0
support_cone_diag|0|0|0.400000006|0|0|0.400000006|0
support_hull_diag|0.300000012|0.200000003|0.100000001|0.300000012|0.200000003|0.100000001|0
support_tri_diag|1|0|0|1|0|0|0
support_tri_tie_hi|1|0|0|1|0|0|0
support_tri_tie_lo|0|0|0|0|0|0|0
support_tri_margin_diag|1.02309406|0.0230940096|0.0230940096|1|0|0|0.0399999991
";

    #[test]
    fn bullet_reference_local_get_supporting_vertex() {
        let (unit_box, _, margin_box, sphere, _, cyl, cone, hull) = probe_shapes();
        let (tri, tri_margin) = probe_triangles();
        let pxyz = Vec3::new(1.0, 1.0, 1.0);
        let cases: [(&str, &dyn ConvexShape, Vec3); 12] = [
            ("unit_box_diag", &unit_box, pxyz),
            ("margin_box_diag", &margin_box, pxyz),
            ("margin_box_axis", &margin_box, Vec3::new(1.0, 0.0, 0.0)),
            ("margin_box_diag_unit", &margin_box, pxyz.normalize()),
            ("sphere_diag", &sphere, pxyz),
            ("cyl_diag", &cyl, pxyz),
            ("cone_diag", &cone, pxyz),
            ("hull_diag", &hull, pxyz),
            ("tri_diag", &tri, pxyz),
            ("tri_tie_hi", &tri, Vec3::new(1.0, 1.0, 0.0)),
            ("tri_tie_lo", &tri, Vec3::new(-1.0, -1.0, 0.0)),
            ("tri_margin_diag", &tri_margin, pxyz),
        ];

        let mut bad = Vec::new();
        let mut covered = Vec::new();
        for (name, shape, dir) in cases {
            let full = format!("support_{name}");
            covered.push(full.clone());
            let f = row(SUPPORT_REFERENCE, &full, 8);
            let n = |i: usize| -> Scalar { f[i].parse().unwrap() };

            diff_vec3(
                &mut bad,
                name,
                "support",
                shape.local_get_supporting_vertex(dir),
                Vec3::new(n(1), n(2), n(3)),
            );
            diff_vec3(
                &mut bad,
                name,
                "support_no_margin",
                shape.local_get_supporting_vertex_without_margin(dir),
                Vec3::new(n(4), n(5), n(6)),
            );
            crate::probe_fixture::diff(&mut bad, name, "margin", shape.margin(), n(7));
        }
        assert!(bad.is_empty(), "{}", bad.join("\n"));

        let mut want: Vec<String> = SUPPORT_REFERENCE
            .lines()
            .filter_map(|l| l.split('|').next())
            .map(str::to_string)
            .collect();
        want.sort();
        covered.sort();
        assert_eq!(
            covered, want,
            "the shapes checked and SUPPORT_REFERENCE disagree on which rows exist"
        );
    }

    #[test]
    fn bullet_reference_polyhedral_vertices() {
        let (unit_box, flat_box, margin_box, sphere, _, cyl, cone, hull) = probe_shapes();
        let (tri, _) = probe_triangles();
        let shapes: [(&str, &dyn ConvexShape); 8] = [
            ("unit_box", &unit_box),
            ("flat_box", &flat_box),
            ("margin_box", &margin_box),
            ("sphere", &sphere),
            ("cyl", &cyl),
            ("cone", &cone),
            ("hull", &hull),
            ("tri", &tri),
        ];

        let mut bad = Vec::new();
        let mut covered = Vec::new();
        for (name, shape) in shapes {
            let vertices = shape.polyhedral_vertices();
            covered.push(format!("polycast_{name}"));

            let f = row(POLYHEDRAL_REFERENCE, &format!("polycast_{name}"), 3);
            let want_cast = f[1] == "1";
            let want_len: usize = f[2].parse().unwrap();
            if vertices.is_some() != want_cast {
                bad.push(format!(
                    "{name}: port polyhedral {}, bullet {want_cast}",
                    vertices.is_some()
                ));
                continue;
            }
            let Some(vertices) = vertices else { continue };
            if vertices.len() != want_len {
                bad.push(format!(
                    "{name}: port {} vertices, bullet {want_len}",
                    vertices.len()
                ));
                continue;
            }

            for (i, &vertex) in vertices.iter().enumerate() {
                let vertex_row = format!("polyvert_{name}_{i}");
                covered.push(vertex_row.clone());
                let f = row(POLYHEDRAL_REFERENCE, &vertex_row, 4);
                let n = |k: usize| -> Scalar { f[k].parse().unwrap() };
                diff_vec3(
                    &mut bad,
                    &format!("{name}[{i}]"),
                    "vertex",
                    vertex,
                    Vec3::new(n(1), n(2), n(3)),
                );
            }
        }
        assert!(bad.is_empty(), "{}", bad.join("\n"));

        // A shape whose vertices this test never asked for would otherwise pass
        // it silently; comparing the row names covered against the row names
        // present makes the reference block check the loop back.
        let mut want: Vec<String> = POLYHEDRAL_REFERENCE
            .lines()
            .filter_map(|l| l.split('|').next())
            .map(str::to_string)
            .collect();
        want.sort();
        covered.sort();
        assert_eq!(
            covered, want,
            "the shapes checked and POLYHEDRAL_REFERENCE disagree on which rows exist"
        );
    }

    /// The `shapeaabb_*` rows -- `getAabb` on the triangle, at three poses,
    /// beside the same three poses run through a plain `btTriangleShape`.
    ///
    /// The `tribase_*` rows are the base class's inherited `getAabbSlow`, which
    /// the port does not carry: they are here as the *control* for the claim
    /// that overriding `getAabb` was necessary. Without them "the subclass
    /// boxes the corners" and "the base pays six support queries" are two
    /// descriptions of one number, and either implementation would pass.
    const TRIANGLE_AABB_REFERENCE: &str = "\
shapeaabb_tri_id|0|0|0|1|1|0
shapeaabb_tri_rot60|-0.0333333313|-0.400000006|-0.13333334|0.966666698|0.266666681|0.866666675
shapeaabb_tri_margin_rot60|-0.0733333305|-0.439999998|-0.173333347|1.00666666|0.306666672|0.906666696
shapeaabb_tribase_rot60|-0.0333333313|-0.400000006|-0.13333334|0.966666698|0.266666681|0.866666675
shapeaabb_tribase_margin_rot60|-0.113333322|-0.479999989|-0.213333338|1.04666662|0.346666634|0.946666658
";

    /// One `shapeaabb_*` row as `(min, max)`.
    fn aabb_row(name: &str) -> (Vec3, Vec3) {
        let f = row(TRIANGLE_AABB_REFERENCE, name, 7);
        let n = |i: usize| -> Scalar { f[i].parse().unwrap() };
        (Vec3::new(n(1), n(2), n(3)), Vec3::new(n(4), n(5), n(6)))
    }

    #[test]
    fn bullet_reference_triangle_aabb() {
        let (tri, tri_margin) = probe_triangles();
        let rot60 = crate::probe_fixture::rot60_at(0.3, -0.4, 0.2);
        let cases: [(&str, &TriangleShapeEx, Transform); 3] = [
            ("tri_id", &tri, IDENTITY),
            ("tri_rot60", &tri, rot60),
            ("tri_margin_rot60", &tri_margin, rot60),
        ];

        let mut bad = Vec::new();
        for (name, shape, pose) in cases {
            let (got_min, got_max) = shape.get_aabb(&pose);
            let (want_min, want_max) = aabb_row(&format!("shapeaabb_{name}"));
            diff_vec3(&mut bad, name, "min", got_min, want_min);
            diff_vec3(&mut bad, name, "max", got_max, want_max);
        }
        assert!(bad.is_empty(), "{}", bad.join("\n"));
    }

    /// The override earns itself only at a nonzero margin: at margin zero the
    /// subclass's corner box and the base's `getAabbSlow` are the same bits, and
    /// at 0.04 the base is wider by exactly the margin on every face because it
    /// adds it once inside `localGetSupportingVertex` and once again itself.
    ///
    /// This is the row pair that makes `get_aabb`'s body a decision rather than
    /// a restatement: delegate to `get_aabb_slow`, as an earlier draft of this
    /// port did, and `shapeaabb_tri_margin_rot60` fails while every other
    /// triangle row still passes.
    #[test]
    fn the_triangle_subclasss_aabb_differs_from_its_bases_only_at_a_nonzero_margin() {
        assert_eq!(
            aabb_row("shapeaabb_tri_rot60"),
            aabb_row("shapeaabb_tribase_rot60")
        );

        let (ex_min, ex_max) = aabb_row("shapeaabb_tri_margin_rot60");
        let (base_min, base_max) = aabb_row("shapeaabb_tribase_margin_rot60");
        assert_ne!((ex_min, ex_max), (base_min, base_max));
        for i in 0..3 {
            assert!((base_min[i] - (ex_min[i] - CONVEX_DISTANCE_MARGIN)).abs() < 1e-6);
            assert!((base_max[i] - (ex_max[i] + CONVEX_DISTANCE_MARGIN)).abs() < 1e-6);
        }
    }

    /// The vertices `getVertex` reports are not the support function's:
    /// `btBoxShape::getVertex` inflates by the margin and
    /// `localGetSupportingVertexWithoutMargin` does not, so on `margin_box`
    /// they differ by exactly the 0.04 default margin on each axis.
    ///
    /// Without this the port could route `polyhedral_vertices` through the
    /// support function and still match every row above, because the other two
    /// boxes and the hull all carry a zero margin.
    #[test]
    fn a_boxs_polyhedral_vertices_carry_the_margin_its_support_function_omits() {
        let (_, _, margin_box, ..) = probe_shapes();
        assert_eq!(margin_box.margin(), CONVEX_DISTANCE_MARGIN);

        let vertices = margin_box.polyhedral_vertices().expect("box is polyhedral");
        assert_eq!(vertices[0], Vec3::new(0.5, 0.5, 0.5));
        assert_eq!(
            margin_box.local_get_supporting_vertex_without_margin(Vec3::new(1.0, 1.0, 1.0)),
            Vec3::new(0.46, 0.46, 0.46)
        );
    }

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
