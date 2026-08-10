// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: Zlib
//
// Ported from bullet3 @ 7dee3436e747958e7088dfdcea0e4ae031ce619e (tag 3.24):
//   bullet3/src/BulletCollision/NarrowPhaseCollision/btManifoldPoint.h
//   bullet3/src/BulletCollision/NarrowPhaseCollision/btPersistentManifold.h
//   bullet3/src/BulletCollision/CollisionDispatch/btManifoldResult.h
//   bullet3/src/BulletCollision/CollisionDispatch/btManifoldResult.cpp

//! `btManifoldPoint`, `btPersistentManifold` and `btManifoldResult` -- the sink
//! the narrow phase writes contacts into, and the threshold it is asked for on
//! the way in.
//!
//! # What the CCD path uses this for, and what it does not
//!
//! Upstream this trio is a *cache*: `btManifoldResult::addContactPoint`
//! projects the contact into both bodies' local frames, looks for a matching
//! point already in `btPersistentManifold`, and either replaces it or adds a
//! new one -- reducing to four points by area when a fifth arrives, so a
//! solver has a stable contact set across frames.
//!
//! MoveIt's `TesseractBroadphaseBridgedManifoldResult` overrides
//! `addContactPoint` and never calls the base
//! (`bullet_utils.hpp:571-630`). It builds a `btManifoldPoint` and hands it
//! straight to its own callback. So in the continuous path *no contact ever
//! enters a manifold*: the point cache stays empty, `getNumContacts()` is
//! always zero, and `refreshContactPoints` returns on its first line. The
//! four-point reduction (`sortCachedPoints`, `getCacheEntry`,
//! `validContactDistance`) is therefore unreachable here and is not ported --
//! porting it would mean transcribing, and claiming fidelity for, arithmetic
//! that nothing in this crate's scope can execute.
//!
//! What does survive is narrow and load-bearing:
//!
//! - [`PersistentManifold::contact_breaking_threshold`], which
//!   `btConvexConvexAlgorithm::processCollision` adds into the GJK query's
//!   `m_maximumDistanceSquared` -- it decides how far apart a pair may be and
//!   still be asked for a contact at all.
//! - [`ManifoldResultState::closest_point_distance_threshold`], added to the
//!   same sum and used again to grow the compound traversal's AABBs.
//! - the body/shape identifiers, which the compound traversal rewrites per
//!   child and MoveIt's callback reads back out.

use crate::collision_object_wrapper::CollisionObjectWrapper;
use crate::discrete_detector::Result;
use crate::linear_math::{Scalar, Vec3};

/// `gContactBreakingThreshold` (`btPersistentManifold.cpp:26`).
pub const CONTACT_BREAKING_THRESHOLD: Scalar = 0.02;

/// `btManifoldPoint` (`btManifoldPoint.h:59-160`), less the solver's fields.
///
/// Everything dropped -- `m_combinedFriction`, `m_appliedImpulse`,
/// `m_lifeTime`, the lateral friction directions -- is written by the
/// constraint solver or read by it, and this crate contains no solver. What
/// remains is what `addSingleResult` reads: the two world positions, the
/// normal, the distance, and the shape identifiers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ManifoldPoint {
    /// `m_localPointA`.
    pub local_point_a: Vec3,
    /// `m_localPointB`.
    pub local_point_b: Vec3,
    /// `m_positionWorldOnB`.
    pub position_world_on_b: Vec3,
    /// `m_positionWorldOnA`.
    pub position_world_on_a: Vec3,
    /// `m_normalWorldOnB`.
    pub normal_world_on_b: Vec3,
    /// `m_distance1` -- negative when the shapes overlap.
    pub distance1: Scalar,
    /// `m_partId0`.
    pub part_id0: i32,
    /// `m_partId1`.
    pub part_id1: i32,
    /// `m_index0`.
    pub index0: i32,
    /// `m_index1`.
    pub index1: i32,
}

impl ManifoldPoint {
    /// `btManifoldPoint(pointA, pointB, normal, distance)`
    /// (`btManifoldPoint.h:70-101`).
    ///
    /// The two world positions are *not* derived from the local points here;
    /// upstream zeroes them and leaves the caller to fill them in, which
    /// MoveIt does immediately afterwards with the world-space values it
    /// already holds.
    #[must_use]
    pub fn new(point_a: Vec3, point_b: Vec3, normal: Vec3, distance: Scalar) -> Self {
        Self {
            local_point_a: point_a,
            local_point_b: point_b,
            position_world_on_b: Vec3::zero(),
            position_world_on_a: Vec3::zero(),
            normal_world_on_b: normal,
            distance1: distance,
            part_id0: -1,
            part_id1: -1,
            index0: -1,
            index1: -1,
        }
    }
}

/// `btPersistentManifold` (`btPersistentManifold.h:76-...`), reduced to the
/// fields the continuous path reads.
///
/// See the module docs for why the point cache is absent rather than empty:
/// nothing in this crate's scope can add to it, so a cache here would be an
/// array no code path writes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PersistentManifold {
    /// `m_contactBreakingThreshold`.
    pub contact_breaking_threshold: Scalar,
}

impl PersistentManifold {
    /// `btCollisionDispatcher::getNewManifold`
    /// (`btCollisionDispatcher.cpp:68-77`), for a dispatcher that has cleared
    /// `CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD`.
    ///
    /// MoveIt clears that flag (`bullet_bvh_manager.cpp:48-49`), which is what
    /// makes the breaking threshold the flat global rather than the smaller of
    /// the two shapes' own. The relative branch is not ported for the same
    /// reason the point cache is not: this path cannot reach it.
    ///
    /// The same line also sets `m_contactProcessingThreshold` to the smaller of
    /// the two objects' own (`:77`). That value has no reader in
    /// `BulletCollision`: both places that would consult it --
    /// `btManifoldResult::addContactPoint` (`:109`) and
    /// `btConvexConvexAlgorithm::processCollision`'s
    /// `m_maximumDistanceSquared` (`:386`) -- are commented out upstream, and
    /// every live read is a `BulletDynamics` constraint solver deciding whether
    /// to build a contact constraint. Carrying it here would be a field with a
    /// `btMin` behind it that no test in this crate could ever discriminate.
    #[must_use]
    pub fn new() -> Self {
        Self {
            contact_breaking_threshold: CONTACT_BREAKING_THRESHOLD,
        }
    }

    /// `getNumContacts` -- always zero here; see the module docs.
    #[must_use]
    pub fn num_contacts(&self) -> usize {
        0
    }
}

impl Default for PersistentManifold {
    /// The same manifold [`PersistentManifold::new`] builds: with the relative
    /// breaking threshold cleared there is only one manifold this path can
    /// produce.
    fn default() -> Self {
        Self::new()
    }
}

/// The fields `btManifoldResult` declares `protected`
/// (`btManifoldResult.h:50-58`) plus the public
/// `m_closestPointDistanceThreshold`.
///
/// Split out from the trait because upstream's subclasses inherit this state
/// and override only `addContactPoint`: an implementor of [`ManifoldResult`]
/// owns one of these and the trait's other methods are written against it
/// once, exactly as the C++ non-virtual accessors are.
#[derive(Clone, Copy)]
pub struct ManifoldResultState<'a> {
    /// `m_manifoldPtr`. `None` models the null upstream sets it to between
    /// compound children (`btCompoundCollisionAlgorithm.cpp:273`).
    pub manifold: Option<PersistentManifold>,
    /// `m_body0Wrap`.
    ///
    /// The whole wrapper, not the transform and identity read off it, because
    /// the compound traversal *replaces* it for each child
    /// (`btCompoundCollisionAlgorithm.cpp:170-201`) and the two things that
    /// replacement changes -- the shape and the wrapper's own transform -- are
    /// what `addCastSingleResult` reads out of it (`bullet_utils.hpp:470-473`).
    /// Storing only `getCollisionObject()->getWorldTransform()` here, as this
    /// type first did, left a child's swept shape unreachable from the result.
    pub body0_wrap: CollisionObjectWrapper<'a>,
    /// `m_body1Wrap`, as above.
    pub body1_wrap: CollisionObjectWrapper<'a>,
    /// `m_manifoldPtr->getBody0()` -- which of the two the manifold was
    /// created around, which is what decides `isSwapped`.
    pub manifold_body0_id: usize,
    /// `m_partId0`.
    pub part_id0: i32,
    /// `m_partId1`.
    pub part_id1: i32,
    /// `m_index0`.
    pub index0: i32,
    /// `m_index1`.
    pub index1: i32,
    /// `m_closestPointDistanceThreshold`.
    pub closest_point_distance_threshold: Scalar,
}

impl<'a> ManifoldResultState<'a> {
    /// `btManifoldResult(body0Wrap, body1Wrap)`
    /// (`btManifoldResult.cpp:44-52`).
    #[must_use]
    pub fn new(
        body0_wrap: CollisionObjectWrapper<'a>,
        body1_wrap: CollisionObjectWrapper<'a>,
    ) -> Self {
        Self {
            manifold: None,
            manifold_body0_id: body0_wrap.object_id,
            body0_wrap,
            body1_wrap,
            part_id0: -1,
            part_id1: -1,
            index0: -1,
            index1: -1,
            closest_point_distance_threshold: 0.0,
        }
    }

    /// `m_manifoldPtr->getBody0() != m_body0Wrap->getCollisionObject()`.
    #[must_use]
    pub fn is_swapped(&self) -> bool {
        self.manifold_body0_id != self.body0_wrap.object_id
    }
}

/// `btManifoldResult` (`btManifoldResult.h:47-...`) -- a
/// [`Result`] that also carries the manifold and the two bodies.
///
/// `add_contact_point` is inherited from `Result` and is the one method
/// upstream leaves virtual for subclasses to replace outright, which is what
/// MoveIt's bridge does.
pub trait ManifoldResult<'a>: Result {
    /// The state every implementor owns; see [`ManifoldResultState`].
    fn state(&mut self) -> &mut ManifoldResultState<'a>;

    /// `setPersistentManifold`.
    fn set_persistent_manifold(&mut self, manifold: Option<PersistentManifold>) {
        self.state().manifold = manifold;
    }

    /// `setBody0Wrap` (`btManifoldResult.h:132-135`), which the compound
    /// traversal calls with a child wrapper and again with the wrapper it
    /// displaced.
    ///
    /// Returns the wrapper it replaced, because every upstream caller saves
    /// that value first in order to restore it
    /// (`btCompoundCollisionAlgorithm.cpp:167-201`) and a caller that read it
    /// separately could read it after some other write.
    fn set_body0_wrap(&mut self, wrap: CollisionObjectWrapper<'a>) -> CollisionObjectWrapper<'a> {
        std::mem::replace(&mut self.state().body0_wrap, wrap)
    }

    /// `setBody1Wrap` (`btManifoldResult.h:137-140`), as above.
    fn set_body1_wrap(&mut self, wrap: CollisionObjectWrapper<'a>) -> CollisionObjectWrapper<'a> {
        std::mem::replace(&mut self.state().body1_wrap, wrap)
    }

    /// `setShapeIdentifiersA` (`btManifoldResult.h:90-94`).
    ///
    /// Declared pure virtual on `btDiscreteCollisionDetectorInterface::Result`
    /// and overridden here; it sits on this trait rather than on
    /// [`Result`] because the compound traversal is the only caller and it
    /// holds a `btManifoldResult*`.
    fn set_shape_identifiers_a(&mut self, part_id0: i32, index0: i32) {
        let state = self.state();
        state.part_id0 = part_id0;
        state.index0 = index0;
    }

    /// `setShapeIdentifiersB` (`btManifoldResult.h:95-99`).
    fn set_shape_identifiers_b(&mut self, part_id1: i32, index1: i32) {
        let state = self.state();
        state.part_id1 = part_id1;
        state.index1 = index1;
    }

    /// `refreshContactPoints` (`btManifoldResult.h:104-119`).
    ///
    /// The first line is `if (!m_manifoldPtr->getNumContacts()) return;`, and
    /// in this path the count is always zero -- see the module docs. The body
    /// below it is left unported rather than transcribed unreachable.
    fn refresh_contact_points(&mut self) {
        debug_assert!(
            self.state().manifold.is_some_and(|m| m.num_contacts() == 0),
            "the continuous path never adds a point to a manifold"
        );
    }
}
