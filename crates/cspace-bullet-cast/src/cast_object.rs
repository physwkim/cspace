// Copyright (c) 2017, Southwest Research Institute
// Copyright (c) 2013, John Schulman
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-2-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection_bullet/src/bullet_integration/bullet_utils.cpp
//   moveit_core/collision_detection_bullet/src/bullet_integration/bullet_cast_bvh_manager.cpp

//! `makeCastCollisionObject` and the shape half of
//! `setCastCollisionObjectsTransform` -- turning a collision object into one
//! that sweeps, and re-aiming that sweep at a new pose pair.
//!
//! # Re-posing without a downcast
//!
//! Upstream reaches a swept child by index, `static_cast`s it back to
//! `CastHullShape` and writes the new delta into it
//! (`bullet_cast_bvh_manager.cpp:101`, `bullet_cast_bvh_manager.cpp:114`).
//! Neither half of that is available here: `compound::Shape`'s `Convex` arm is
//! a trait object, so "is this particular implementation" is exactly the
//! question the sum type removes, and a shape behind a shared `Arc` of a
//! `Send + Sync` trait cannot be written to at all.
//!
//! So [`CastCollisionObject`] keeps, in traversal order, the *inner* shape of
//! every swept child -- the expensive part, and the immutable one -- and
//! re-posing builds a fresh `CastHullShape` over it and puts that where the
//! old child was. The delta then exists in exactly one place, the child the
//! compound currently holds; there is no second copy to drift.
//!
//! Unqualified citations in this file are lines in
//! `bullet_utils.cpp`; a citation of any other file names that file.

use std::sync::Arc;

use cspace_bullet::broadphase_proxy::{BroadphaseNativeType, CollisionFilterGroup};
use cspace_bullet::compound::{CompoundShape, Shape as BulletShape};
use cspace_bullet::linear_math::Transform;
use cspace_bullet::shapes::ConvexShape;

use crate::cast_hull_shape::{
    ArcConvexShape, BULLET_COMPOUND_USE_DYNAMIC_AABB, BULLET_MARGIN, CastHullShape,
};
use crate::collision_object::CollisionObjectWrapper;

/// Why a collision object could not be made to sweep.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CastError {
    /// `assert(convex->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE)`
    /// (`bullet_utils.cpp:306`, `:324`, `:344`) -- the object already sweeps.
    /// `BulletCastBVHManager::clone` asserts the same thing about the objects
    /// it copies (`bullet_cast_bvh_manager.cpp:53`), because casting a cast
    /// would sweep a sweep.
    AlreadyCast,
    /// `assert(!btBroadphaseProxy::isCompound(second_compound->getChildShape(j)
    /// ->getShapeType()))` (`bullet_utils.cpp:341`) -- a compound three levels
    /// deep.
    ///
    /// Not reachable from `createShapePrimitive`, which nests exactly two:
    /// a multi-shape object is a compound, and an octree among its shapes is a
    /// compound child of it.
    CompoundTooDeep,
}

/// The shape swept by each `CastHullShape` of one cast object, in the order
/// `setCastCollisionObjectsTransform` walks them.
enum CastHulls {
    /// The object's whole collision shape is one swept shape -- the
    /// single-shape branch of `CollisionObjectWrapper`'s constructor.
    Single(ArcConvexShape),
    /// The object is a compound; each entry is one of its children.
    Compound(Vec<CastChildHulls>),
}

/// One child of a cast object's top-level compound.
enum CastChildHulls {
    /// A swept convex child.
    Hull(ArcConvexShape),
    /// A compound child, whose own children are all swept convex shapes.
    Compound(Vec<ArcConvexShape>),
}

/// A collision object that sweeps between two poses, and the handles that
/// re-aim the sweep.
pub struct CastCollisionObject {
    /// The cloned wrapper, with its shape replaced by the swept one.
    cow: CollisionObjectWrapper,
    /// The shapes the wrapper's swept children were built over.
    hulls: CastHulls,
}

impl CastCollisionObject {
    /// `makeCastCollisionObject` (`bullet_utils.cpp:293-385`).
    ///
    /// The delta starts at identity, which makes the swept shape's support
    /// function agree with the shape's own in every direction -- so a cast
    /// object that is never re-posed behaves as the discrete one it was built
    /// from.
    ///
    /// # Errors
    ///
    /// [`CastError`] for the two states upstream asserts against.
    pub fn new(cow: &CollisionObjectWrapper) -> Result<Self, CastError> {
        let (shape, hulls) = build(cow.collision_shape())?;
        let mut new_cow = cow.clone_object();
        // `new_cow->setWorldTransform(cow->getWorldTransform())`
        // (`bullet_utils.cpp:375`) is already done: `clone` copies it, and
        // upstream's compound branch repeats the assignment.
        *new_cow.collision_shape_mut() = shape;
        Ok(Self {
            cow: new_cow,
            hulls,
        })
    }

    /// The object as the broadphase and the narrow phase see it.
    #[must_use]
    pub fn object(&self) -> &CollisionObjectWrapper {
        &self.cow
    }

    /// The shape half of `setCastCollisionObjectsTransform`
    /// (`bullet_cast_bvh_manager.cpp:66-134`), less the map lookup that finds
    /// the object and the broadphase AABB update that follows -- both of them
    /// the manager's.
    ///
    /// The object's own world transform becomes `tf1`, so every delta below is
    /// expressed in the first pose's frame. For a compound child at `local_tf`
    /// that is `(tf1 * local_tf)⁻¹ (tf2 * local_tf)` and not `tf1⁻¹ tf2`: a
    /// rotation of the parent carries a child that sits off its origin through
    /// an arc the parent's own delta does not describe.
    ///
    /// # Panics
    ///
    /// If the object is not in the kinematic group -- `assert(cow->
    /// m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)`
    /// (`bullet_cast_bvh_manager.cpp:75`). Only an active link is ever cast; a
    /// static one is added to the manager as itself.
    pub fn set_cast_transforms(&mut self, tf1: Transform, tf2: Transform) {
        assert_eq!(
            self.cow.collision_filter_group,
            CollisionFilterGroup::KINEMATIC,
            "only an active object is ever cast"
        );

        self.cow.set_world_transform(tf1);

        // "If collision object is disabled don't proceed"
        // (`bullet_cast_bvh_manager.cpp:84`) -- and note the world transform
        // above is written either way.
        if !self.cow.enabled {
            return;
        }

        let hulls = &self.hulls;
        match (hulls, self.cow.collision_shape_mut()) {
            (CastHulls::Single(inner), shape) => {
                *shape = swept(inner, tf1.inverse_times(&tf2));
            }
            (CastHulls::Compound(children), BulletShape::Compound(compound)) => {
                for (i, child) in children.iter().enumerate() {
                    match child {
                        CastChildHulls::Hull(inner) => {
                            let local_tf = *compound.child_transform(i);
                            *compound.child_shape_mut(i) =
                                swept(inner, (tf1 * local_tf).inverse_times(&(tf2 * local_tf)));
                            // "This is required to update the BVH tree"
                            // (`bullet_cast_bvh_manager.cpp:102`): the child's
                            // AABB just changed under a transform that did not.
                            compound.update_child_transform(i, local_tf, false);
                        }
                        CastChildHulls::Compound(grandchildren) => {
                            let BulletShape::Compound(second) = compound.child_shape_mut(i) else {
                                unreachable!("a compound handle names a compound child")
                            };
                            for (j, inner) in grandchildren.iter().enumerate() {
                                let local_tf = *second.child_transform(j);
                                *second.child_shape_mut(j) =
                                    swept(inner, (tf1 * local_tf).inverse_times(&(tf2 * local_tf)));
                                second.update_child_transform(j, local_tf, false);
                            }
                            second.recalculate_local_aabb();
                        }
                    }
                }
                compound.recalculate_local_aabb();
            }
            (CastHulls::Compound(_), BulletShape::Convex(_)) => {
                unreachable!("a compound handle set is only built for a compound shape")
            }
        }
    }
}

/// One shape swept by one delta, as a compound child.
fn swept(inner: &ArcConvexShape, delta: Transform) -> BulletShape {
    let hull: Arc<dyn ConvexShape> = Arc::new(CastHullShape::new(Arc::clone(inner), delta));
    BulletShape::Convex(hull)
}

/// The shape swap itself, kept apart from the wrapper so the original is read
/// while the clone is written.
fn build(shape: &BulletShape) -> Result<(BulletShape, CastHulls), CastError> {
    let tf = Transform::identity();

    match shape {
        BulletShape::Convex(convex) => {
            if is_cast(shape) {
                return Err(CastError::AlreadyCast);
            }
            Ok((swept(convex, tf), CastHulls::Single(Arc::clone(convex))))
        }
        BulletShape::Compound(compound) => {
            let mut new_compound = CompoundShape::new(BULLET_COMPOUND_USE_DYNAMIC_AABB);
            let mut children = Vec::with_capacity(compound.num_child_shapes());

            for i in 0..compound.num_child_shapes() {
                let geom_trans = *compound.child_transform(i);
                match compound.child_shape(i) {
                    BulletShape::Convex(convex) => {
                        if is_cast(compound.child_shape(i)) {
                            return Err(CastError::AlreadyCast);
                        }
                        // `subshape->setMargin(BULLET_MARGIN)`
                        // (`bullet_utils.cpp:331`) is not written: a
                        // `CastHullShape`'s margin is zero by definition and
                        // its `setMargin` is a no-op.
                        new_compound.add_child_shape(geom_trans, swept(convex, tf));
                        children.push(CastChildHulls::Hull(Arc::clone(convex)));
                    }
                    BulletShape::Compound(second_compound) => {
                        let mut new_second = CompoundShape::new(BULLET_COMPOUND_USE_DYNAMIC_AABB);
                        let mut grandchildren =
                            Vec::with_capacity(second_compound.num_child_shapes());

                        for j in 0..second_compound.num_child_shapes() {
                            let BulletShape::Convex(convex) = second_compound.child_shape(j) else {
                                return Err(CastError::CompoundTooDeep);
                            };
                            if is_cast(second_compound.child_shape(j)) {
                                return Err(CastError::AlreadyCast);
                            }
                            new_second.add_child_shape(
                                *second_compound.child_transform(j),
                                swept(convex, tf),
                            );
                            grandchildren.push(Arc::clone(convex));
                        }

                        let mut second = BulletShape::Compound(new_second);
                        // margin on compound seems to have no effect when
                        // positive but has an effect when negative
                        // (`bullet_utils.cpp:359-360`)
                        second.set_margin(BULLET_MARGIN);
                        new_compound.add_child_shape(geom_trans, second);
                        children.push(CastChildHulls::Compound(grandchildren));
                    }
                }
            }

            new_compound.set_margin(BULLET_MARGIN);
            Ok((
                BulletShape::Compound(new_compound),
                CastHulls::Compound(children),
            ))
        }
    }
}

/// `getShapeType() == CUSTOM_CONVEX_SHAPE_TYPE` -- the test upstream's three
/// asserts make, and the only way a shape says it already sweeps.
fn is_cast(shape: &BulletShape) -> bool {
    shape.shape_type() == BroadphaseNativeType::CUSTOM_CONVEX_SHAPE
}

#[cfg(test)]
mod tests {
    use cspace_bullet::linear_math::{Matrix3, Vec3};
    use cspace_core::geometry::Isometry3;
    use cspace_core::geometry::shapes::{Cuboid, Shape};

    use super::*;
    use crate::contact_test_data::BodyType;
    use crate::shape_primitive::CollisionObjectType;

    fn cuboid(size: f64) -> Shape {
        Shape::Cuboid(Cuboid {
            size: [size, size, size],
        })
    }

    fn object(shapes: &[Shape], poses: &[Isometry3]) -> CollisionObjectWrapper {
        let types = vec![CollisionObjectType::UseShapeType; shapes.len()];
        CollisionObjectWrapper::new("link", BodyType::RobotLink, shapes, poses, &types, true)
            .unwrap()
    }

    fn at(x: f64, y: f64, z: f64) -> Isometry3 {
        Isometry3::translation(x, y, z)
    }

    fn shifted(x: f32) -> Transform {
        Transform::new(Matrix3::identity(), Vec3::new(x, 0.0, 0.0))
    }

    /// The swap is what makes the object sweep: the shape reports
    /// `CUSTOM_CONVEX_SHAPE`, which is how every dispatch downstream knows it
    /// is looking at a cast.
    #[test]
    fn a_single_shape_becomes_one_swept_shape() {
        let cow = object(&[cuboid(2.0)], &[at(0.0, 0.0, 0.0)]);
        let cast = CastCollisionObject::new(&cow).unwrap();

        assert_eq!(
            cast.object().collision_shape().shape_type(),
            BroadphaseNativeType::CUSTOM_CONVEX_SHAPE
        );
        assert_eq!(
            cow.collision_shape().shape_type(),
            BroadphaseNativeType::BOX_SHAPE,
            "the original is untouched"
        );
    }

    /// An already-cast object is refused rather than swept twice.
    #[test]
    fn casting_a_cast_object_is_refused() {
        let cow = object(&[cuboid(1.0)], &[at(0.0, 0.0, 0.0)]);
        let cast = CastCollisionObject::new(&cow).unwrap();
        assert_eq!(
            CastCollisionObject::new(cast.object()).err(),
            Some(CastError::AlreadyCast)
        );
    }

    /// Re-posing widens the swept support in the direction of travel and
    /// leaves it alone against it -- the whole point of the cast shape.
    #[test]
    fn a_re_posed_single_shape_sweeps_from_the_first_pose_to_the_second() {
        let cow = object(&[cuboid(2.0)], &[at(0.0, 0.0, 0.0)]);
        let mut cast = CastCollisionObject::new(&cow).unwrap();

        let support = |cast: &CastCollisionObject, dir: Vec3| {
            let BulletShape::Convex(swept) = cast.object().collision_shape() else {
                panic!("a single shape stays convex");
            };
            swept.local_get_supporting_vertex(dir).x
        };
        let plus_x = Vec3::new(1.0, 0.0, 0.0);

        assert_eq!(
            support(&cast, plus_x),
            1.0,
            "an identity delta is the shape itself"
        );

        cast.set_cast_transforms(shifted(0.0), shifted(5.0));

        assert_eq!(support(&cast, plus_x), 6.0);
        assert_eq!(support(&cast, -plus_x), -1.0);
        assert_eq!(
            cast.object().world_transform().origin,
            Vec3::new(0.0, 0.0, 0.0),
            "the object stays at the first pose"
        );
    }

    /// A compound child's delta is taken in the child's own frame, so a pure
    /// translation of the parent gives every child the same delta -- and the
    /// compound's AABB has to have followed, or the broadphase would still be
    /// offering the pre-sweep box.
    #[test]
    fn every_compound_child_sweeps_and_the_compound_aabb_follows() {
        let cow = object(
            &[cuboid(1.0), cuboid(1.0)],
            &[at(0.0, 0.0, 0.0), at(0.0, 4.0, 0.0)],
        );
        let mut cast = CastCollisionObject::new(&cow).unwrap();

        let (before_min, before_max) = cast.object().collision_shape().get_aabb(&shifted(0.0));
        cast.set_cast_transforms(shifted(0.0), shifted(5.0));
        let (after_min, after_max) = cast.object().collision_shape().get_aabb(&shifted(0.0));

        assert_eq!(before_min, Vec3::new(-0.5, -0.5, -0.5));
        assert_eq!(before_max, Vec3::new(0.5, 4.5, 0.5));
        assert_eq!(after_min, Vec3::new(-0.5, -0.5, -0.5));
        assert_eq!(after_max, Vec3::new(5.5, 4.5, 0.5));

        let BulletShape::Compound(compound) = cast.object().collision_shape() else {
            panic!("two shapes make a compound");
        };
        for i in 0..compound.num_child_shapes() {
            assert_eq!(
                compound.child_shape(i).shape_type(),
                BroadphaseNativeType::CUSTOM_CONVEX_SHAPE,
                "child {i} sweeps"
            );
        }
    }

    /// A disabled object keeps its new world transform but not a new sweep --
    /// upstream writes the transform before the `m_enabled` test and the
    /// deltas after it.
    #[test]
    fn a_disabled_object_moves_without_sweeping() {
        let cow = object(&[cuboid(2.0)], &[at(0.0, 0.0, 0.0)]);
        let mut cast = CastCollisionObject::new(&cow).unwrap();
        cast.cow.enabled = false;

        let BulletShape::Convex(swept) = cast.object().collision_shape() else {
            panic!("a single shape stays convex");
        };
        let swept = Arc::clone(swept);

        cast.set_cast_transforms(shifted(7.0), shifted(12.0));

        assert_eq!(
            cast.object().world_transform().origin,
            Vec3::new(7.0, 0.0, 0.0)
        );
        assert_eq!(
            swept
                .local_get_supporting_vertex(Vec3::new(1.0, 0.0, 0.0))
                .x,
            1.0,
            "the delta was never applied"
        );
    }
}
