// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: Zlib
//
// Ported from bullet3 @ 7dee3436e747958e7088dfdcea0e4ae031ce619e (tag 3.24):
//   bullet3/src/BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h
//   bullet3/src/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp

//! `btCollisionObjectWrapper` -- the (shape, transform) view of a collision
//! object that the dispatcher hands to an algorithm.
//!
//! # Why the object's transform is a field and not the wrapper's
//!
//! Upstream holds `m_collisionObject` and reaches `getWorldTransform()` through
//! it, so a wrapper answers two different transform questions: its own
//! `m_worldTransform`, which a compound child overrides with
//! `parent_world * child_local`, and the object's, which every child of the
//! same object shares. Both are read on the continuous path and they disagree
//! for exactly the case the path cares about -- a swept compound child --
//! `btCompoundLeafCallback::ProcessChildShape`
//! (`btCompoundCollisionAlgorithm.cpp:130-201`) building the first and
//! `btManifoldResult::addContactPoint` reading the second. Carrying only the
//! collision object's identity, as this type first did, made the second
//! unreachable from a child wrapper.

use crate::compound::Shape;
use crate::linear_math::Transform;

/// `btCollisionObjectWrapper` (`btCollisionObjectWrapper.h:17-46`), reduced to
/// what the traversal, the narrow phase and the cast result callback read.
///
/// No `Debug`: [`Shape`] holds a `dyn ConvexShape`, and a shape that a
/// downstream crate defines -- which is the whole reason that box is a trait
/// object -- cannot be required to implement it.
#[derive(Clone, Copy)]
pub struct CollisionObjectWrapper<'a> {
    /// `m_shape`.
    pub shape: &'a Shape,
    /// `m_worldTransform`.
    pub world_transform: Transform,
    /// Identity of `m_collisionObject`. Every child wrapper a compound builds
    /// inherits its parent's, which is what makes the swap detection in both
    /// leaf callbacks answer the same way at every depth.
    pub object_id: usize,
    /// `m_collisionObject->getWorldTransform()` -- inherited by every child
    /// wrapper for the same reason [`Self::object_id`] is, and equal to
    /// [`Self::world_transform`] only at the top level.
    pub object_transform: Transform,
}

impl<'a> CollisionObjectWrapper<'a> {
    /// A wrapper over a top-level collision object, whose own transform is the
    /// object's.
    #[must_use]
    pub fn new(shape: &'a Shape, world_transform: Transform, object_id: usize) -> Self {
        Self {
            shape,
            world_transform,
            object_id,
            object_transform: world_transform,
        }
    }

    /// The `btCollisionObjectWrapper compoundWrap(...)` both leaf callbacks
    /// build for a child: a new shape and a new world transform over the same
    /// collision object.
    #[must_use]
    pub fn child(&self, shape: &'a Shape, world_transform: Transform) -> Self {
        Self {
            shape,
            world_transform,
            object_id: self.object_id,
            object_transform: self.object_transform,
        }
    }
}
