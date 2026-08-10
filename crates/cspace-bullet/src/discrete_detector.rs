// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: Zlib
//
// Ported from bullet3 @ 7dee3436e747958e7088dfdcea0e4ae031ce619e (tag 3.24):
//   bullet3/src/BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h

//! `btDiscreteCollisionDetectorInterface` -- the input and output of a
//! narrow-phase query.
//!
//! The sign conventions here are the ones every caller downstream assumes, so
//! they are stated rather than left to the field names: the closest point is
//! on B, the normal points from B's surface towards A, and `depth` is negative
//! when the shapes overlap. Closest point on A is
//! `point_in_world + depth * normal_on_b_in_world`.
//!
//! `Result` deliberately keeps upstream's name even though it shadows the
//! prelude's `Result` inside a module that imports it. Nothing in this crate
//! returns `core::result::Result`, and renaming it would leave every call site
//! naming something upstream does not have.

use crate::linear_math::{BT_LARGE_FLOAT, Scalar, Transform, Vec3};

/// `btDiscreteCollisionDetectorInterface::Result`.
///
/// `setShapeIdentifiersA`/`B` are not here. They exist for per-triangle
/// material lookup in `btConvexConcaveCollisionAlgorithm`; the convex-convex
/// path this crate serves never calls them, and `btGjkPairDetector` does not
/// either.
pub trait Result {
    /// `addContactPoint(normalOnBInWorld, pointInWorld, depth)`.
    fn add_contact_point(
        &mut self,
        normal_on_b_in_world: Vec3,
        point_in_world: Vec3,
        depth: Scalar,
    );
}

/// `btDiscreteCollisionDetectorInterface::ClosestPointInput`.
#[derive(Clone, Copy, Debug)]
pub struct ClosestPointInput {
    /// `m_transformA`.
    pub transform_a: Transform,
    /// `m_transformB`.
    pub transform_b: Transform,
    /// `m_maximumDistanceSquared` -- the caller's cut-off, defaulted to
    /// `BT_LARGE_FLOAT` (which is `1e18`, not an infinity) so that no result
    /// is discarded unless the caller narrows it.
    pub maximum_distance_squared: Scalar,
}

impl ClosestPointInput {
    /// `ClosestPointInput()` -- identity transforms and no cut-off. Upstream's
    /// constructor only initializes `m_maximumDistanceSquared`; `btTransform`'s
    /// default constructor leaves the transforms uninitialized, and every
    /// caller assigns both before use.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            transform_a: Transform::identity(),
            transform_b: Transform::identity(),
            maximum_distance_squared: BT_LARGE_FLOAT,
        }
    }
}

impl Default for ClosestPointInput {
    fn default() -> Self {
        Self::new()
    }
}

/// `btStorageResult` -- keeps only the deepest contact it is given.
#[derive(Clone, Copy, Debug)]
pub struct StorageResult {
    /// `m_normalOnSurfaceB`.
    pub normal_on_surface_b: Vec3,
    /// `m_closestPointInB`.
    pub closest_point_in_b: Vec3,
    /// `m_distance` -- negative means penetration. Seeded to
    /// `BT_LARGE_FLOAT`, which is also the "nothing was added" marker: the
    /// first `add_contact_point` always beats it.
    pub distance: Scalar,
}

impl StorageResult {
    /// `btStorageResult()`.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            normal_on_surface_b: Vec3::zero(),
            closest_point_in_b: Vec3::zero(),
            distance: BT_LARGE_FLOAT,
        }
    }
}

impl Default for StorageResult {
    fn default() -> Self {
        Self::new()
    }
}

impl Result for StorageResult {
    fn add_contact_point(
        &mut self,
        normal_on_b_in_world: Vec3,
        point_in_world: Vec3,
        depth: Scalar,
    ) {
        if depth < self.distance {
            self.normal_on_surface_b = normal_on_b_in_world;
            self.closest_point_in_b = point_in_world;
            self.distance = depth;
        }
    }
}
