// Copyright (c) 2017, Southwest Research Institute
// Copyright (c) 2013, John Schulman
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-2-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection_bullet/include/moveit/collision_detection_bullet/bullet_integration/bullet_utils.hpp
//   moveit_core/collision_detection_bullet/src/bullet_integration/bullet_utils.cpp

//! The callbacks a broadphase pair passes through on its way to a stored
//! contact: the pair filter, the pre-narrowphase check, the
//! `btManifoldResult` bridge, and the overlap callback that drives the narrow
//! phase.
//!
//! # The order they run in
//!
//! ```text
//! broadphase pair
//!   -> needBroadphaseCollision            masks, m_enabled, touch links
//!   -> processOverlap
//!        -> needsCollision                done, kinematic pairing, ACM
//!        -> the dispatched algorithm
//!             -> addContactPoint          -> addSingleResult
//!                  -> addCastSingleResult -> processResult
//! ```
//!
//! Which of the two filters rejects a pair is observable and they are not
//! interchangeable: `needBroadphaseCollision` runs once per pair-cache entry
//! and never sees the traversal's `done`, while `needsCollision` is asked
//! again for every overlap and short-circuits on it.
//!
//! # The two flags upstream carries that are constants here
//!
//! `BroadphaseContactResultCallback` has a `self_` and a `cast_`
//! (`bullet_utils.hpp:537-540`). `BulletCastBVHManager::contactTest` is the
//! only constructor on the continuous path and it passes `self = false,
//! cast = true` (`bullet_cast_bvh_manager.cpp:146`) -- the discrete managers
//! supply the other three combinations. So this port carries neither as a
//! field: [`BroadphaseContactResultCallback::needs_collision`] is upstream's
//! `cast_` branch and `addSingleResult` reaches `addCastSingleResult`
//! unconditionally. A flag whose other value
//! would select `addDiscreteSingleResult` -- which this crate does not carry
//! -- would be a switch with one working position.
//!
//! # Why the ACM arrives as a trait
//!
//! `acmCheck` reads a `collision_detection::AllowedCollisionMatrix`, which
//! this workspace ports in `cspace_collision::matrix` -- a BSD-3-Clause crate
//! that depends on this one. The dependency edge runs one way, so the matrix
//! reaches this crate as [`AllowedCollisions`] and `cspace_collision` supplies
//! the implementation. See [`crate::contact_test_data`] for the same seam and
//! the licence boundary that cuts it.

use cspace_bullet::broadphase_proxy::CollisionFilterGroup;
use cspace_bullet::collision_object_wrapper::CollisionObjectWrapper as BtObjectWrapper;
use cspace_bullet::compound::Shape as BulletShape;
use cspace_bullet::compound_algorithm::{UnportedAlgorithm, process_collision};
use cspace_bullet::discrete_detector::Result as DetectorResult;
use cspace_bullet::dispatch::DispatchTable;
use cspace_bullet::linear_math::{Scalar, Vec3};
use cspace_bullet::manifold::{ManifoldPoint, ManifoldResult, ManifoldResultState};

use crate::cast_contact::apply_cast_result;
use crate::cast_hull_shape::CastHullShape;
use crate::collision_object::{CollisionObjectWrapper, is_only_kinematic};
use crate::contact_test_data::{
    BodyType, CastRequest, CastResult, Contact, Stored, object_pair_key, process_result,
};

/// `collision_detection::AllowedCollision::Type`.
///
/// Re-declared here for the reason [`AllowedCollisions`] gives, and with all
/// three variants even though `acmCheck` folds the last two together: which
/// one an entry is is the matrix's answer, and an implementor that had to
/// collapse `CONDITIONAL` onto `ALWAYS` before answering would be deciding
/// part of `acmCheck` outside this port.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AllowedCollisionType {
    /// `NEVER` -- the pair must be checked.
    Never,
    /// `ALWAYS` -- the pair is allowed to collide.
    Always,
    /// `CONDITIONAL` -- allowed subject to a predicate. `acmCheck` does not
    /// evaluate the predicate; it treats the entry as allowed, so a
    /// conditional entry skips the continuous check outright.
    Conditional,
}

/// `collision_detection::AllowedCollisionMatrix::getAllowedCollision`, reduced
/// to the one query `acmCheck` makes.
///
/// A trait rather than the type itself because the type is in a crate that
/// depends on this one; see the module docs.
pub trait AllowedCollisions {
    /// The matrix's entry for the pair, or `None` when it has none -- which is
    /// upstream's `getAllowedCollision` returning false without writing its
    /// out-parameter.
    fn allowed_collision(&self, body_1: &str, body_2: &str) -> Option<AllowedCollisionType>;
}

/// `acmCheck` (`bullet_utils.cpp:49-83`) -- whether the pair is *allowed*, and
/// so may be skipped.
///
/// Note the polarity: true means "do not check this pair". No matrix and no
/// entry both answer false, so an unknown pair is checked.
#[must_use]
pub fn acm_check(body_1: &str, body_2: &str, acm: Option<&dyn AllowedCollisions>) -> bool {
    let Some(acm) = acm else {
        return false;
    };
    match acm.allowed_collision(body_1, body_2) {
        Some(AllowedCollisionType::Never) | None => false,
        Some(AllowedCollisionType::Always | AllowedCollisionType::Conditional) => true,
    }
}

/// `BroadphaseFilterCallback::needBroadphaseCollision`
/// (`bullet_utils.cpp:438-478`).
///
/// # The proxies' filters are the objects' filters
///
/// Upstream reads `proxy0->m_collisionFilterMask & proxy1->m_collisionFilterGroup`
/// off the broadphase proxies and only then reaches `m_clientObject` for the
/// rest. The proxy's copy is written once, by `createProxy`
/// (`bullet_utils.cpp:396-397`), out of the object's -- and the one path that
/// changes an object's filters afterwards destroys and recreates the proxy in
/// the same loop (`bullet_bvh_manager.cpp:129-137`). So the two cannot
/// disagree, and this port reads the objects rather than carrying a second
/// copy a future caller could let drift.
#[must_use]
pub fn need_broadphase_collision(
    cow0: &CollisionObjectWrapper,
    cow1: &CollisionObjectWrapper,
) -> bool {
    let cull = !cow0
        .collision_filter_mask
        .intersects(cow1.collision_filter_group)
        || !cow1
            .collision_filter_mask
            .intersects(cow0.collision_filter_group);
    if cull {
        return false;
    }

    if !cow0.enabled || !cow1.enabled {
        return false;
    }

    // An attached body never collides with the links it is attached to, in
    // whichever order the broadphase presents the pair.
    if cow0.type_id() == BodyType::RobotAttached
        && cow1.type_id() == BodyType::RobotLink
        && cow0.touch_links.contains(cow1.name())
    {
        return false;
    }
    if cow1.type_id() == BodyType::RobotAttached
        && cow0.type_id() == BodyType::RobotLink
        && cow1.touch_links.contains(cow0.name())
    {
        return false;
    }

    // Two attached bodies: equal *sets*, not an intersection, so two bodies
    // whose link sets merely overlap are still checked against each other.
    if cow0.type_id() == BodyType::RobotAttached
        && cow1.type_id() == BodyType::RobotAttached
        && cow0.touch_links == cow1.touch_links
    {
        return false;
    }

    true
}

/// `BroadphaseContactResultCallback` (`bullet_utils.hpp:530-569`), on the
/// `cast_ = true, self_ = false` setting that is the only one this crate can
/// be constructed in; see the module docs.
///
/// `collisions_` splits into [`Self::request`] and [`Self::result`] for the
/// reason [`crate::contact_test_data`] gives; `active` is not carried because
/// nothing this callback reaches reads it.
pub struct BroadphaseContactResultCallback<'a> {
    /// `collisions_.req`.
    pub request: &'a CastRequest,
    /// `collisions_.res`, plus the two traversal flags.
    pub result: &'a mut CastResult,
    /// `contact_distance_` -- the depth beyond which a contact is dropped.
    pub contact_distance: f64,
    /// `acm_`, `None` when the caller has no matrix.
    pub acm: Option<&'a dyn AllowedCollisions>,
}

impl BroadphaseContactResultCallback<'_> {
    /// `needsCollision`'s `cast_` branch (`bullet_utils.hpp:555-558`).
    ///
    /// `!isOnlyKinematic` rejects a pair of two *swept* objects: continuous
    /// against continuous is unsupported, which is stated where
    /// `updateCollisionObjectFilters` is declared (`bullet_utils.hpp:661-669`)
    /// and enforced here.
    #[must_use]
    pub fn needs_collision(
        &self,
        cow0: &CollisionObjectWrapper,
        cow1: &CollisionObjectWrapper,
    ) -> bool {
        !self.result.done
            && !is_only_kinematic(cow0, cow1)
            && !acm_check(cow0.name(), cow1.name(), self.acm)
    }

    /// `addSingleResult` (`bullet_utils.cpp:480-499`).
    ///
    /// Upstream's `btScalar` return is not carried: it is
    /// `btCollisionWorld::ContactResultCallback`'s signature, and the one
    /// caller on this path -- `addContactPoint` -- discards it.
    ///
    /// # Errors
    ///
    /// [`CastCallbackError::SweptSideIsNotAHull`] when the swept side did not
    /// present a [`CastHullShape`] to the narrow phase.
    fn add_single_result(
        &mut self,
        point: &ManifoldPoint,
        pair: &CastPair<'_, '_>,
    ) -> Result<(), CastCallbackError> {
        if f64::from(point.distance1) > self.contact_distance {
            return Ok(());
        }
        self.add_cast_single_result(point, pair)
    }

    /// `addCastSingleResult` (`bullet_utils.hpp:419-517`), less the tail
    /// [`apply_cast_result`] carries.
    fn add_cast_single_result(
        &mut self,
        point: &ManifoldPoint,
        pair: &CastPair<'_, '_>,
    ) -> Result<(), CastCallbackError> {
        let (cd0, cd1) = (pair.cow0, pair.cow1);
        let key = object_pair_key(cd0.name(), cd1.name());

        let contact = Contact {
            body_name_1: cd0.name().to_owned(),
            body_name_2: cd1.name().to_owned(),
            body_type_1: cd0.type_id(),
            body_type_2: cd1.type_id(),
            depth: point.distance1,
            normal: point.normal_world_on_b * -1.0,
            pos: point.position_world_on_a,
            percent_interpolation: 0.0,
        };

        let Stored::Yes { key } = process_result(self.result, self.request, contact, key) else {
            return Ok(());
        };

        // `assert(!(cd0 and cd1 are both kinematic))` (`:451-452`) -- which
        // `needsCollision` has already rejected, so the flag below names which
        // side is swept and exactly one side is.
        debug_assert!(
            !is_only_kinematic(cd0, cd1),
            "needsCollision rejects a pair of two swept objects"
        );
        let cast_shape_is_first = cd0.collision_filter_group == CollisionFilterGroup::KINEMATIC;

        let first = if cast_shape_is_first {
            pair.wrap0
        } else {
            pair.wrap1
        };
        let BulletShape::Convex(shape) = first.shape else {
            return Err(CastCallbackError::SweptSideIsNotAHull);
        };
        let Some(cast_shape) = shape.as_ref().as_any().downcast_ref::<CastHullShape>() else {
            return Err(CastCallbackError::SweptSideIsNotAHull);
        };

        let col = self
            .result
            .last_contact_mut(&key)
            .expect("processResult reported the contact it just appended");
        apply_cast_result(
            col,
            point,
            cast_shape_is_first,
            cast_shape,
            first.world_transform,
        );
        Ok(())
    }
}

/// The pair a contact was produced for: upstream's two
/// `btCollisionObjectWrapper*` and the `CollisionObjectWrapper` each one's
/// `getCollisionObject()` reaches.
///
/// The four travel together because `addCastSingleResult` reads names and
/// types off the objects and the shape and pose off the wrappers, and a
/// wrapper paired with the wrong object would produce a contact naming one
/// link and measured on another.
struct CastPair<'w, 'o> {
    /// `colObj0Wrap` -- the child wrapper when the object is a compound.
    wrap0: BtObjectWrapper<'w>,
    /// `colObj0Wrap->getCollisionObject()`.
    cow0: &'o CollisionObjectWrapper,
    /// `colObj1Wrap`.
    wrap1: BtObjectWrapper<'w>,
    /// `colObj1Wrap->getCollisionObject()`.
    cow1: &'o CollisionObjectWrapper,
}

/// A pair the continuous path cannot answer for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CastCallbackError {
    /// The side whose filter group is `KinematicFilter` did not present a
    /// [`CastHullShape`] to the narrow phase.
    ///
    /// Upstream `static_cast`s here and reads whatever it lands on. Reaching
    /// this means an object entered the manager as kinematic without
    /// `makeCastCollisionObject` having replaced its shapes.
    SweptSideIsNotAHull,
    /// The pair dispatched to an algorithm `cspace_bullet` does not carry.
    Unported(UnportedAlgorithm),
}

impl From<UnportedAlgorithm> for CastCallbackError {
    fn from(unported: UnportedAlgorithm) -> Self {
        Self::Unported(unported)
    }
}

/// `TesseractBroadphaseBridgedManifoldResult` (`bullet_utils.hpp:571-630`).
///
/// It replaces `btManifoldResult::addContactPoint` outright rather than
/// extending it, which is why no contact on this path ever enters a manifold
/// -- see `cspace_bullet::manifold`.
struct BridgedManifoldResult<'c, 'r, 'o, 'w> {
    state: ManifoldResultState<'w>,
    /// `m_body0Wrap->getCollisionObject()` for the wrapper this result was
    /// built around as body 0. Held rather than looked up: a child wrapper
    /// inherits its parent's collision object, so the pairing holds at every
    /// depth of a compound.
    cow0: &'o CollisionObjectWrapper,
    /// As above, for body 1.
    cow1: &'o CollisionObjectWrapper,
    callback: &'c mut BroadphaseContactResultCallback<'r>,
    /// The first error a contact hit. `addContactPoint` returns void upstream,
    /// so the failure is held here and reported by [`process_overlap`] rather
    /// than dropped.
    error: Option<CastCallbackError>,
}

impl DetectorResult for BridgedManifoldResult<'_, '_, '_, '_> {
    /// `addContactPoint(normalOnBInWorld, pointInWorld, depth)`
    /// (`bullet_utils.hpp:583-629`).
    fn add_contact_point(
        &mut self,
        normal_on_b_in_world: Vec3,
        point_in_world: Vec3,
        depth: Scalar,
    ) {
        if self.callback.result.done
            || self.callback.result.pair_done
            || f64::from(depth) > self.callback.contact_distance
        {
            return;
        }
        // Not upstream's: a pair that has already failed produces no further
        // contacts rather than a second failure over a result that is short.
        if self.error.is_some() {
            return;
        }

        let is_swapped = self.state.is_swapped();
        let point_a = point_in_world + normal_on_b_in_world * depth;

        // The two local points come from the *objects'* transforms, not the
        // wrappers': `m_bodyNWrap->getCollisionObject()->getWorldTransform()`
        // is the same at every depth of a compound.
        let (body_a, body_b) = if is_swapped {
            (&self.state.body1_wrap, &self.state.body0_wrap)
        } else {
            (&self.state.body0_wrap, &self.state.body1_wrap)
        };
        let local_a = body_a.object_transform.inv_xform(point_a);
        let local_b = body_b.object_transform.inv_xform(point_in_world);

        let mut new_pt = ManifoldPoint::new(local_a, local_b, normal_on_b_in_world, depth);
        new_pt.position_world_on_a = point_a;
        new_pt.position_world_on_b = point_in_world;

        // "BP mod, store contact triangles" (`:609`).
        if is_swapped {
            new_pt.part_id0 = self.state.part_id1;
            new_pt.part_id1 = self.state.part_id0;
            new_pt.index0 = self.state.index1;
            new_pt.index1 = self.state.index0;
        } else {
            new_pt.part_id0 = self.state.part_id0;
            new_pt.part_id1 = self.state.part_id1;
            new_pt.index0 = self.state.index0;
            new_pt.index1 = self.state.index1;
        }

        let pair = if is_swapped {
            CastPair {
                wrap0: self.state.body1_wrap,
                cow0: self.cow1,
                wrap1: self.state.body0_wrap,
                cow1: self.cow0,
            }
        } else {
            CastPair {
                wrap0: self.state.body0_wrap,
                cow0: self.cow0,
                wrap1: self.state.body1_wrap,
                cow1: self.cow1,
            }
        };

        if let Err(error) = self.callback.add_single_result(&new_pt, &pair) {
            self.error = Some(error);
        }
    }
}

impl<'w> ManifoldResult<'w> for BridgedManifoldResult<'_, '_, '_, 'w> {
    fn state(&mut self) -> &mut ManifoldResultState<'w> {
        &mut self.state
    }
}

/// `TesseractCollisionPairCallback::processOverlap`
/// (`bullet_utils.cpp:501-536`).
///
/// `id0`/`id1` are the two collision objects' identities, which is what
/// upstream compares pointers for; the manager supplies them and they must be
/// distinct.
///
/// Upstream returns `false` always -- the value `processAllOverlappingPairs`
/// reads to decide whether to delete the pair -- so there is nothing to return
/// but a failure.
///
/// `pair.m_algorithm`'s caching is not carried: it is a pool the dispatcher
/// owns, and this port's `findAlgorithm` is a pure function of the two shape
/// types.
///
/// # Errors
///
/// [`CastCallbackError`] if the pair, or any child pair below it, reaches an
/// algorithm this port does not carry, or if the swept side is not a hull.
/// Both mean the query produced fewer contacts than upstream would, and must
/// not be read as having found none.
pub fn process_overlap(
    callback: &mut BroadphaseContactResultCallback<'_>,
    cow0: &CollisionObjectWrapper,
    id0: usize,
    cow1: &CollisionObjectWrapper,
    id1: usize,
) -> Result<(), CastCallbackError> {
    debug_assert_ne!(id0, id1, "a broadphase pair is two distinct objects");
    callback.result.pair_done = false;

    if callback.result.done {
        return Ok(());
    }

    if !callback.needs_collision(cow0, cow1) {
        return Ok(());
    }

    let obj0_wrap = BtObjectWrapper::new(cow0.collision_shape(), cow0.world_transform(), id0);
    let obj1_wrap = BtObjectWrapper::new(cow1.collision_shape(), cow1.world_transform(), id1);

    let mut state = ManifoldResultState::new(obj0_wrap, obj1_wrap);
    // `contact_point_result.m_closestPointDistanceThreshold =
    // static_cast<btScalar>(results_callback_.contact_distance_)` (`:529`).
    state.closest_point_distance_threshold = callback.contact_distance as Scalar;
    let mut result = BridgedManifoldResult {
        state,
        cow0,
        cow1,
        callback,
        error: None,
    };

    let outcome = process_collision(
        &obj0_wrap,
        &obj1_wrap,
        DispatchTable::ClosestPoints,
        &mut result,
    );

    if let Some(error) = result.error {
        return Err(error);
    }
    outcome.map_err(CastCallbackError::from)
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;
    use std::sync::Arc;

    use cspace_bullet::linear_math::{Matrix3, Transform};
    use cspace_bullet::probe_fixture::{IDENTITY, at as probe_at, diff, diff_vec3, row};
    use cspace_core::geometry::Isometry3;
    use cspace_core::geometry::shapes::{Cuboid, Shape};

    use super::*;
    use crate::arc_probe::arc_probe_shapes;
    use crate::cast_object::CastCollisionObject;
    use crate::shape_primitive::CollisionObjectType;

    /// An ACM built from a list, so a test names only the pairs it cares about
    /// and every other pair answers "no entry".
    struct Acm(BTreeMap<(String, String), AllowedCollisionType>);

    impl Acm {
        fn of(entries: &[(&str, &str, AllowedCollisionType)]) -> Self {
            Self(
                entries
                    .iter()
                    .map(|(a, b, t)| (object_pair_key(a, b), *t))
                    .collect(),
            )
        }
    }

    impl AllowedCollisions for Acm {
        fn allowed_collision(&self, body_1: &str, body_2: &str) -> Option<AllowedCollisionType> {
            self.0.get(&object_pair_key(body_1, body_2)).copied()
        }
    }

    fn cuboid(size: f64) -> Shape {
        Shape::Cuboid(Cuboid {
            size: [size, size, size],
        })
    }

    fn at(x: f64) -> Isometry3 {
        Isometry3::translation(x, 0.0, 0.0)
    }

    fn shifted(x: Scalar) -> Transform {
        Transform::new(Matrix3::identity(), Vec3::new(x, 0.0, 0.0))
    }

    fn object(name: &str, type_id: BodyType, x: f64, active: bool) -> CollisionObjectWrapper {
        CollisionObjectWrapper::new(
            name,
            type_id,
            &[cuboid(1.0)],
            &[at(x)],
            &[CollisionObjectType::UseShapeType],
            active,
        )
        .expect("one cuboid at one pose is a valid object")
    }

    /// True means "skip this pair", so an unknown pair must answer false or a
    /// scene with no matrix would be checked for nothing.
    #[test]
    fn only_an_allowing_entry_skips_a_pair() {
        let acm = Acm::of(&[
            ("a", "never", AllowedCollisionType::Never),
            ("a", "always", AllowedCollisionType::Always),
            ("a", "cond", AllowedCollisionType::Conditional),
        ]);

        assert!(!acm_check("a", "never", Some(&acm)));
        assert!(acm_check("a", "always", Some(&acm)));
        assert!(
            acm_check("a", "cond", Some(&acm)),
            "acmCheck does not evaluate the predicate; a conditional entry is allowed"
        );
        assert!(
            !acm_check("a", "absent", Some(&acm)),
            "no entry, so checked"
        );
        assert!(!acm_check("a", "always", None), "no matrix, so checked");
    }

    /// The one direction `updateCollisionObjectFilters` never produces: two
    /// static objects, which the broadphase must not offer the narrow phase.
    #[test]
    fn a_static_pair_is_culled_and_an_active_pair_is_not() {
        let active_a = object("a", BodyType::RobotLink, 0.0, true);
        let active_b = object("b", BodyType::RobotLink, 0.0, true);
        let static_a = object("c", BodyType::WorldObject, 0.0, false);
        let static_b = object("d", BodyType::WorldObject, 0.0, false);

        assert!(need_broadphase_collision(&active_a, &static_a));
        assert!(need_broadphase_collision(&static_a, &active_a));
        assert!(need_broadphase_collision(&active_a, &active_b));
        assert!(!need_broadphase_collision(&static_a, &static_b));
    }

    /// Either side being disabled is enough, and the pair is asked in the
    /// order the broadphase happens to hold it.
    #[test]
    fn a_disabled_object_pairs_with_nothing() {
        let mut a = object("a", BodyType::RobotLink, 0.0, true);
        let b = object("b", BodyType::WorldObject, 0.0, false);
        a.enabled = false;

        assert!(!need_broadphase_collision(&a, &b));
        assert!(!need_broadphase_collision(&b, &a));
    }

    /// The touch-link test is asymmetric in the *types*: it is the attached
    /// body's list that is consulted, whichever side it arrives on.
    #[test]
    fn an_attached_body_never_pairs_with_a_link_it_touches() {
        let mut attached = object("gripped", BodyType::RobotAttached, 0.0, true);
        attached.touch_links.insert("hand".to_owned());
        let hand = object("hand", BodyType::RobotLink, 0.0, true);
        let forearm = object("forearm", BodyType::RobotLink, 0.0, true);

        assert!(!need_broadphase_collision(&attached, &hand));
        assert!(!need_broadphase_collision(&hand, &attached));
        assert!(
            need_broadphase_collision(&attached, &forearm),
            "a link the body does not touch is still checked"
        );
    }

    /// Equal sets, not an intersection: two bodies sharing one link but not
    /// all of them are still checked against each other.
    #[test]
    fn two_attached_bodies_pair_unless_their_link_sets_are_equal() {
        let links = |names: &[&str]| {
            let mut cow = object("attached", BodyType::RobotAttached, 0.0, true);
            cow.touch_links = names.iter().map(|n| (*n).to_owned()).collect();
            cow
        };

        assert!(!need_broadphase_collision(
            &links(&["hand"]),
            &links(&["hand"])
        ));
        assert!(need_broadphase_collision(
            &links(&["hand"]),
            &links(&["hand", "forearm"])
        ));
    }

    /// Continuous against continuous is unsupported, and this is where it is
    /// refused; `done` and the ACM are the other two ways a pair is dropped.
    #[test]
    fn needs_collision_drops_a_swept_pair_a_done_traversal_and_an_allowed_pair() {
        let swept_a = object("a", BodyType::RobotLink, 0.0, true);
        let swept_b = object("b", BodyType::RobotLink, 0.0, true);
        let world = object("w", BodyType::WorldObject, 0.0, false);
        let acm = Acm::of(&[("a", "w", AllowedCollisionType::Always)]);

        let request = CastRequest::default();
        let mut result = CastResult::default();
        let mut callback = BroadphaseContactResultCallback {
            request: &request,
            result: &mut result,
            contact_distance: 0.0,
            acm: None,
        };

        assert!(callback.needs_collision(&swept_a, &world));
        assert!(!callback.needs_collision(&swept_a, &swept_b));

        callback.result.done = true;
        assert!(!callback.needs_collision(&swept_a, &world));
        callback.result.done = false;

        callback.acm = Some(&acm);
        assert!(!callback.needs_collision(&swept_a, &world));
        assert!(
            callback.needs_collision(&world, &swept_b),
            "a pair the matrix has no entry for is still checked"
        );
    }

    /// One overlap of the swept link against the world box, at whatever
    /// state the caller has already put the result in.
    fn sweep(result: &mut CastResult, from: Scalar, to: Scalar, cast_first: bool) {
        let link = object("link", BodyType::RobotLink, 0.0, true);
        let mut cast = CastCollisionObject::new(&link).expect("a box is castable");
        cast.set_cast_transforms(shifted(from), shifted(to));
        let world = object("box", BodyType::WorldObject, 0.0, false);

        assert!(
            need_broadphase_collision(cast.object(), &world),
            "an active link against a world object is a broadphase pair"
        );

        let request = CastRequest {
            contacts: true,
            ..CastRequest::default()
        };
        let mut callback = BroadphaseContactResultCallback {
            request: &request,
            result,
            contact_distance: 0.0,
            acm: None,
        };

        if cast_first {
            process_overlap(&mut callback, cast.object(), 0, &world, 1)
        } else {
            process_overlap(&mut callback, &world, 1, cast.object(), 0)
        }
        .expect("two boxes dispatch to convex-convex");
    }

    /// The one contact the sweep stored.
    fn only_contact(result: &CastResult) -> &Contact {
        let contacts = &result.contacts[&object_pair_key("link", "box")];
        assert_eq!(contacts.len(), 1, "the default per-pair budget is one");
        &contacts[0]
    }

    /// The whole path, from the pair filter through the narrow phase to a
    /// stored contact -- and the swap `addCastSingleResult` makes so that the
    /// *non*-swept side is reported first, whichever side swept.
    #[test]
    fn a_sweep_into_a_box_reports_the_static_side_first() {
        let mut result = CastResult::default();
        sweep(&mut result, -3.0, -0.9, true);

        assert!(result.collision);
        let contact = only_contact(&result);
        assert_eq!(contact.body_name_1, "box");
        assert_eq!(contact.body_name_2, "link");
        assert_eq!(contact.body_type_1, BodyType::WorldObject);
        assert_eq!(contact.body_type_2, BodyType::RobotLink);

        // This sweep is the `cc_cast_box_approach` row's configuration, and
        // `depth` is the one field that crosses `addCastSingleResult`
        // untouched -- so it ties the whole callback path to a measured
        // narrow-phase answer rather than only to the reasoning above.
        let want: Scalar = row(BULLET_REFERENCE, "cc_cast_box_approach", 10)[8]
            .parse()
            .expect("the row's depth field is a float");
        assert_eq!(contact.depth, want);
    }

    /// The two ends of `addCastSingleResult`'s support comparison, which is
    /// what `percent_interpolation` is: not a time of impact, but which of the
    /// two poses reaches further along the contact normal.
    ///
    /// Both sweeps here are along the +/-x axis and end (or start) 0.1 inside
    /// a box the other pose is 2.0 clear of, so the normal is the sweep axis
    /// and one pose reaches further by the whole sweep length -- far outside
    /// `BULLET_SUPPORT_FUNC_TOLERANCE`, so the interpolating branch is not
    /// reached and the answer is exactly an endpoint.
    #[test]
    fn the_pose_that_reaches_further_along_the_normal_is_the_reported_end() {
        let mut approaching = CastResult::default();
        sweep(&mut approaching, -3.0, -0.9, true);
        assert_eq!(
            only_contact(&approaching).percent_interpolation,
            1.0,
            "the second pose is the one touching"
        );

        let mut retreating = CastResult::default();
        sweep(&mut retreating, -0.9, -3.0, true);
        assert_eq!(
            only_contact(&retreating).percent_interpolation,
            0.0,
            "the first pose is the one touching"
        );
    }

    /// A sweep that stops short reaches the narrow phase and produces nothing,
    /// so a contact is evidence of the sweep and not of the pair existing.
    #[test]
    fn a_sweep_that_stops_short_of_a_box_reports_no_collision() {
        let mut result = CastResult::default();
        sweep(&mut result, -3.0, -1.2, true);

        assert!(!result.collision);
        assert!(result.contacts.is_empty());
    }

    /// `pair_done` is cleared at the top of every overlap, or a pair that
    /// filled its budget would silence every pair after it.
    #[test]
    fn each_overlap_clears_the_previous_pairs_done_flag() {
        let mut result = CastResult {
            pair_done: true,
            ..CastResult::default()
        };
        sweep(&mut result, -3.0, -0.9, true);

        assert!(result.collision, "the stale pair_done did not silence it");
    }

    /// A finished traversal produces nothing, and stops before the narrow
    /// phase rather than by dropping what it returns.
    #[test]
    fn a_done_traversal_runs_no_further_pairs() {
        let mut result = CastResult {
            done: true,
            ..CastResult::default()
        };
        sweep(&mut result, -3.0, -0.9, true);

        assert!(!result.collision);
        assert!(result.contacts.is_empty());
    }

    /// The four `cc_cast_*` rows of `tools/bullet-epa-reference/build.sh`'s
    /// stdout: a `CastHullShape` against a static box through the dispatcher
    /// MoveIt configures, which is the narrow-phase query every continuous
    /// check is made of and the one no earlier fixture covers -- `cspace_bullet`
    /// cannot build these rows, because the swept shape is defined here.
    ///
    /// The corner normals are not a defect: `cc_cast_box_approach` puts the
    /// Minkowski difference's nearest face 0.1 away and its next-nearest 1.0,
    /// and Bullet's EPA still answers with a box corner at 0.0577. The pair
    /// handed over the other way round gets a *different* corner, which is why
    /// [`the_pair_is_filed_the_same_way_in_either_order`] asserts nothing about
    /// the geometry.
    ///
    /// Fields: `name|contacts|normalOnB xyz|pointOnB xyz|depth|maxDistSq`.
    const BULLET_REFERENCE: &str = "\
cc_cast_box_approach|1|-0.577350259|0.577350259|0.577350259|-0.433333397|0.466666698|0.5|-0.0577349737|0.00039999999
cc_cast_box_retreat|1|-0.222387701|0.6893996|0.6893996|-0.469337285|0.49999994|0.484668612|-0.0222386662|0.00039999999
cc_cast_box_through|1|0.07832627|0.704934478|0.704934359|0.5|0.223926395|0.5|-0.391630232|0.00039999999
cc_cast_box_approach_swapped|1|0.577350259|0.577350259|-0.577350259|-0.400000095|0.466666698|0.466666698|-0.0577349737|0.00039999999
";

    /// `RecordingResult` of `cspace_bullet`'s own convex-convex rows, which is
    /// the probe's `addContactPoint` override: the last contact, kept raw.
    struct RecordingResult<'a> {
        state: ManifoldResultState<'a>,
        count: usize,
        normal: Vec3,
        point: Vec3,
        depth: Scalar,
    }

    impl DetectorResult for RecordingResult<'_> {
        fn add_contact_point(
            &mut self,
            normal_on_b_in_world: Vec3,
            point_in_world: Vec3,
            depth: Scalar,
        ) {
            self.count += 1;
            self.normal = normal_on_b_in_world;
            self.point = point_in_world;
            self.depth = depth;
        }
    }

    impl<'a> ManifoldResult<'a> for RecordingResult<'a> {
        fn state(&mut self) -> &mut ManifoldResultState<'a> {
            &mut self.state
        }
    }

    /// Every swept-shape narrow-phase row, against the port.
    #[test]
    fn bullet_reference_cast_narrowphase() {
        let (unit_box, ..) = arc_probe_shapes();
        let swept = |dx: Scalar| {
            BulletShape::Convex(Arc::new(CastHullShape::new(
                Arc::clone(&unit_box),
                probe_at(dx, 0.0, 0.0),
            )))
        };
        let plain = BulletShape::Convex(Arc::clone(&unit_box));
        let mut bad = Vec::new();

        let mut case =
            |name: &str, a: &BulletShape, ta: Transform, b: &BulletShape, tb: Transform| {
                let f = row(BULLET_REFERENCE, name, 10);
                let n = |k: usize| -> Scalar {
                    f[k].parse()
                        .unwrap_or_else(|e| panic!("{name}: field {k} ({:?}): {e}", f[k]))
                };

                let (wrap_a, wrap_b) = (
                    BtObjectWrapper::new(a, ta, 0),
                    BtObjectWrapper::new(b, tb, 1),
                );
                let mut out = RecordingResult {
                    state: ManifoldResultState::new(wrap_a, wrap_b),
                    count: 0,
                    normal: Vec3::zero(),
                    point: Vec3::zero(),
                    depth: 0.0,
                };
                process_collision(&wrap_a, &wrap_b, DispatchTable::ClosestPoints, &mut out)
                    .unwrap_or_else(|e| panic!("{name}: {e:?}"));

                let want_count: usize = f[1]
                    .parse()
                    .unwrap_or_else(|e| panic!("{name}: field 1 ({:?}): {e}", f[1]));
                if out.count != want_count {
                    bad.push(format!(
                        "{name}.contacts: port {}, bullet {want_count}",
                        out.count
                    ));
                }
                diff_vec3(
                    &mut bad,
                    name,
                    "normal",
                    out.normal,
                    Vec3::new(n(2), n(3), n(4)),
                );
                diff_vec3(
                    &mut bad,
                    name,
                    "point",
                    out.point,
                    Vec3::new(n(5), n(6), n(7)),
                );
                diff(&mut bad, name, "depth", out.depth, n(8));
            };

        let fwd = swept(2.1);
        case(
            "cc_cast_box_approach",
            &fwd,
            shifted(-3.0),
            &plain,
            IDENTITY,
        );
        case(
            "cc_cast_box_retreat",
            &swept(-2.1),
            shifted(-0.9),
            &plain,
            IDENTITY,
        );
        case(
            "cc_cast_box_through",
            &swept(8.0),
            shifted(-4.0),
            &plain,
            IDENTITY,
        );
        case(
            "cc_cast_box_approach_swapped",
            &plain,
            IDENTITY,
            &fwd,
            shifted(-3.0),
        );

        assert!(bad.is_empty(), "{}", bad.join("\n"));
    }

    /// The swept side is found by its filter group, not by which side the
    /// broadphase presented -- so the pair the other way round is filed as the
    /// same contact, under the same key and with the same side first.
    ///
    /// What is *not* asserted is the contact's geometry. GJK and EPA are not
    /// symmetric in their two arguments: the `cc_cast_box_approach` and
    /// `cc_cast_box_approach_swapped` rows of the bullet probe are this exact
    /// pair both ways round and Bullet itself answers with two different
    /// corners of the same box. Requiring them to agree would be requiring the
    /// port to be better behaved than what it ports.
    #[test]
    fn the_pair_is_filed_the_same_way_in_either_order() {
        let mut cast_first = CastResult::default();
        sweep(&mut cast_first, -3.0, -0.9, true);
        let mut world_first = CastResult::default();
        sweep(&mut world_first, -3.0, -0.9, false);

        let (a, b) = (only_contact(&cast_first), only_contact(&world_first));
        assert_eq!(a.body_name_1, b.body_name_1);
        assert_eq!(a.body_name_2, b.body_name_2);
        assert_eq!(a.body_type_1, b.body_type_1);
        assert_eq!(a.body_type_2, b.body_type_2);
        assert_eq!(a.percent_interpolation, b.percent_interpolation);
    }
}
