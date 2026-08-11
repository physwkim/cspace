// Copyright (c) 2017, Southwest Research Institute
// Copyright (c) 2013, John Schulman
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-2-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection_bullet/src/bullet_integration/bullet_cast_bvh_manager.cpp
//   moveit_core/collision_detection_bullet/src/bullet_integration/bullet_bvh_manager.cpp
//   moveit_core/collision_detection_bullet/src/bullet_integration/bullet_utils.cpp
//   moveit_core/collision_detection_bullet/include/moveit/collision_detection_bullet/bullet_integration/bullet_utils.hpp

//! `BulletCastBVHManager` -- the collection of collision objects a continuous
//! check runs over, and the broadphase that decides which of them are worth
//! asking about.
//!
//! # What is here and what is not
//!
//! `BulletBVHManager` is a base class shared by the discrete and continuous
//! managers, and this port carries only the members
//! `checkRobotCollisionHelperCCD` reaches
//! (`collision_env_bullet.cpp:209-238`): [`BulletCastBvhManager::new`],
//! [`BulletCastBvhManager::add_collision_object`],
//! [`BulletCastBvhManager::remove_collision_object`],
//! [`BulletCastBvhManager::set_cast_collision_objects_transform`] and
//! [`BulletCastBvhManager::contact_test`]. The rest is deliberately absent:
//!
//! - `clone`, which exists so `CollisionEnvBullet`'s copy constructor can take
//!   a snapshot of a *persistent* manager. This one is built per query -- see
//!   below -- so there is no state to snapshot.
//! - `hasCollisionObject`, `getCollisionObjects`, `getActiveCollisionObjects`:
//!   read-only accessors nothing on this path reads.
//! - `enableCollisionObject` / `disableCollisionObject`. `m_enabled` is
//!   written by the object's constructor and read by the two filters; nothing
//!   between `addCollisionObject` and `contactTest` flips it.
//! - `setCollisionObjectsTransform`, the discrete re-pose.
//!   `checkRobotCollisionHelperCCD` re-poses through
//!   `setCastCollisionObjectsTransform` only, and the two are not
//!   interchangeable: the discrete one writes the world transform and leaves
//!   every swept child aimed where it was.
//! - `setActiveCollisionObjects`, which rewrites the filter groups of objects
//!   already added. On the cast manager that is a hole rather than a feature:
//!   `addCollisionObject` decides *at add time* whether an object is swept, by
//!   reading the very filter group this would change, so an object promoted to
//!   active afterwards would be a kinematic object with no `CastHullShape` --
//!   which is `CastCallbackError::SweptSideIsNotAHull` at query time.
//!   Activeness is settled one step earlier, by
//!   [`crate::collision_object::CollisionObjectWrapper::new`]'s `active`
//!   argument.
//!
//! `setContactDistanceThreshold` is absent for the reason the threshold itself
//! is a constant here: `BulletBVHManager`'s constructor seeds
//! `contact_distance_` to [`BULLET_DEFAULT_CONTACT_DISTANCE`] and the only two
//! calls that change it are on `CollisionEnvBullet`'s *discrete* `manager_`
//! (`collision_env_bullet.cpp:127,187`).
//!
//! # Per query, not persistent
//!
//! `CollisionEnvBullet` keeps one `manager_CCD_` for the lifetime of the
//! environment and adds attached bodies to it and removes them again around
//! each query. A caller of this port builds one, fills it, asks once and drops
//! it. The answers are the same -- the broadphase is a function of the AABBs
//! it is given, not of how many queries ago they arrived -- but the world and
//! link objects are re-added per query, so the cost is not.
//!
//! # Why the objects sit behind a shared cell
//!
//! Upstream's proxy carries `m_clientObject`, a `void*` back to the collision
//! object, and `BroadphaseFilterCallback::needBroadphaseCollision` follows it
//! while the broadphase is mid-insertion. `cspace_bullet`'s proxy carries a
//! `usize` it is documented not to be able to follow, so the callback needs
//! its own way back to the objects -- and it is installed *inside* the
//! broadphase, which the manager is mutating whenever the callback runs.
//!
//! Hence [`ObjectStore`] behind an `Rc<RefCell<..>>`, held by the manager and
//! by the filter callback. It is one store, not a copy: the filter reads
//! `m_enabled`, the two filter words and `touch_links` off the same objects
//! the query reads shapes off, so there is nothing that can drift. The
//! alternative -- dropping the filter callback and applying
//! [`need_broadphase_collision`] at query time instead -- is not equivalent:
//! a rejected pair is never *stored*, and the stored pair array's capacity is
//! the hash mask, so admitting rejected pairs reorders
//! `processAllOverlappingPairs` and hence which contacts a bounded
//! `max_contacts` keeps.
//!
//! Unqualified citations in this file are lines in
//! `bullet_cast_bvh_manager.cpp`; a citation of any other file names
//! that file.

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::fmt;
use std::rc::Rc;

use cspace_bullet::broadphase_proxy::CollisionFilterGroup;
use cspace_bullet::dbvt_broadphase::DbvtBroadphase;
use cspace_bullet::linear_math::Transform;
use cspace_bullet::overlapping_pair_cache::{OverlapFilterCallback, PairProxies, ProxyHandle};

use crate::cast_callback::{
    AllowedCollisions, BroadphaseContactResultCallback, CastCallbackError,
    need_broadphase_collision, process_overlap,
};
use crate::cast_hull_shape::BULLET_DEFAULT_CONTACT_DISTANCE;
use crate::cast_object::{CastCollisionObject, CastError};
use crate::collision_object::CollisionObjectWrapper;
use crate::contact_test_data::{CastRequest, CastResult};

/// Why an object could not be added to the manager.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum AddObjectError {
    /// The manager already holds an object of this name.
    ///
    /// Upstream has no such state to reproduce: `link2cow_[name] = cow`
    /// overwrites the entry and drops the last `shared_ptr` to the object that
    /// was there, while the broadphase keeps a proxy whose `m_clientObject`
    /// still points at it -- `addCollisionObject`
    /// (`bullet_cast_bvh_manager.cpp:151-171`) never removes what it
    /// displaces. Every pair that proxy takes part in afterwards reads freed
    /// memory.
    ///
    /// It does not arise in MoveIt: `CollisionEnvBullet` adds links and world
    /// objects once at construction, and adds each attached body around one
    /// query and removes it again (`collision_env_bullet.cpp:219-238`).
    DuplicateName(String),
    /// The object is active and could not be made to sweep.
    Cast(CastError),
}

impl From<CastError> for AddObjectError {
    fn from(error: CastError) -> Self {
        Self::Cast(error)
    }
}

impl fmt::Display for AddObjectError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DuplicateName(name) => {
                write!(f, "the manager already holds an object named {name:?}")
            }
            Self::Cast(error) => write!(f, "the object cannot be made to sweep: {error:?}"),
        }
    }
}

impl std::error::Error for AddObjectError {}

/// One object the manager holds, in whichever of the two forms
/// `addCollisionObject` put it in.
enum ManagedObject {
    /// An active link or attached body, replaced by its swept twin
    /// (`bullet_cast_bvh_manager.cpp:156-157`).
    Cast(CastCollisionObject),
    /// Everything else, held as it arrived (`:161`).
    Static(CollisionObjectWrapper),
}

impl ManagedObject {
    /// The object the broadphase and the narrow phase see.
    fn object(&self) -> &CollisionObjectWrapper {
        match self {
            Self::Cast(cast) => cast.object(),
            Self::Static(cow) => cow,
        }
    }
}

/// `link2cow_` plus the `m_clientObject` indirection, as one owner.
///
/// The slot index *is* the proxy's `client_object`, which is why a removed
/// object leaves a `None` behind rather than being swapped out: a stale proxy
/// pointing at a slot some later object had taken over would be
/// `needBroadphaseCollision` answering about the wrong pair. `ProxyArena` does
/// not reuse its own slots either, for the same reason.
#[derive(Default)]
pub struct ObjectStore {
    slots: Vec<Option<(ManagedObject, ProxyHandle)>>,
    by_name: BTreeMap<String, usize>,
}

impl ObjectStore {
    /// The object filed in `slot`.
    ///
    /// # Panics
    ///
    /// If `slot` holds no object, which means a proxy outlived the object it
    /// was created for.
    fn object(&self, slot: usize) -> &CollisionObjectWrapper {
        self.slots[slot]
            .as_ref()
            .expect("a live proxy names a live object")
            .0
            .object()
    }
}

/// `BroadphaseFilterCallback` (`bullet_utils.hpp:712-716`), holding the way
/// back to the objects that upstream gets from `m_clientObject`.
struct BroadphaseFilterCallback {
    store: Rc<RefCell<ObjectStore>>,
}

impl OverlapFilterCallback for BroadphaseFilterCallback {
    fn need_broadphase_collision(
        &self,
        proxies: &dyn PairProxies,
        proxy0: ProxyHandle,
        proxy1: ProxyHandle,
    ) -> bool {
        let store = self.store.borrow();
        need_broadphase_collision(
            store.object(proxies.proxy(proxy0).client_object),
            store.object(proxies.proxy(proxy1).client_object),
        )
    }
}

/// `BulletCastBVHManager` over `BulletBVHManager`'s broadphase and object map.
pub struct BulletCastBvhManager {
    store: Rc<RefCell<ObjectStore>>,
    broadphase: DbvtBroadphase,
}

impl Default for BulletCastBvhManager {
    fn default() -> Self {
        Self::new()
    }
}

impl BulletCastBvhManager {
    /// `BulletBVHManager::BulletBVHManager` (`bullet_bvh_manager.cpp:40-56`).
    ///
    /// The dispatcher upstream builds here is not a field, and both lines that
    /// configure it are already unconditional in `cspace_bullet`: registering
    /// the convex-convex create func for `BOX_SHAPE_PROXYTYPE` pairs is what
    /// `dispatch`'s shape-class match does without a table, and clearing
    /// `CD_USE_RELATIVE_CONTACT_BREAKING_THRESHOLD` is why `convex_convex`
    /// takes an absolute breaking threshold.
    #[must_use]
    pub fn new() -> Self {
        let store = Rc::new(RefCell::new(ObjectStore::default()));
        let mut broadphase = DbvtBroadphase::new();
        broadphase.set_overlap_filter_callback(Some(Box::new(BroadphaseFilterCallback {
            store: Rc::clone(&store),
        })));
        Self { store, broadphase }
    }

    /// `BulletCastBVHManager::addCollisionObject`
    /// (`bullet_cast_bvh_manager.cpp:151-171`), with
    /// `addCollisionObjectToBroadphase` (`bullet_utils.cpp:387-398`) inlined
    /// as upstream's is.
    ///
    /// # Errors
    ///
    /// [`AddObjectError::Cast`] when an active object cannot be made to sweep,
    /// and [`AddObjectError::DuplicateName`] for the state upstream leaves
    /// undefined.
    pub fn add_collision_object(
        &mut self,
        cow: CollisionObjectWrapper,
    ) -> Result<(), AddObjectError> {
        if self.store.borrow().by_name.contains_key(cow.name()) {
            return Err(AddObjectError::DuplicateName(cow.name().to_owned()));
        }

        let object = if cow.collision_filter_group == CollisionFilterGroup::KINEMATIC {
            ManagedObject::Cast(CastCollisionObject::new(&cow)?)
        } else {
            ManagedObject::Static(cow)
        };

        let name = object.object().name().to_owned();
        let (aabb_min, aabb_max) = object.object().get_aabb();
        let shape_type = object.object().collision_shape().shape_type();
        let group = object.object().collision_filter_group;
        let mask = object.object().collision_filter_mask;

        // The object has to be in the store before its proxy exists: upstream
        // passes `link2cow_[name].get()` to `createProxy`, and `createProxy`
        // runs the filter callback over every pair it announces -- a callback
        // that reads *this* slot. So the object goes in first, under a
        // placeholder handle, and the store is not borrowed across the call.
        let slot = {
            let mut store = self.store.borrow_mut();
            let slot = store.slots.len();
            store.slots.push(Some((object, ProxyHandle::NULL)));
            store.by_name.insert(name, slot);
            slot
        };

        let proxy = self
            .broadphase
            .create_proxy(aabb_min, aabb_max, shape_type, slot, group, mask);
        self.store.borrow_mut().slots[slot]
            .as_mut()
            .expect("the slot just filled")
            .1 = proxy;
        Ok(())
    }

    /// `BulletBVHManager::removeCollisionObject`
    /// (`bullet_bvh_manager.cpp:70-83`) with
    /// `removeCollisionObjectFromBroadphase` (`bullet_utils.hpp:689-702`)
    /// inlined.
    ///
    /// Returns whether an object of that name was there, which is upstream's
    /// return.
    pub fn remove_collision_object(&mut self, name: &str) -> bool {
        let Some(&slot) = self.store.borrow().by_name.get(name) else {
            return false;
        };
        let proxy = self.store.borrow().slots[slot]
            .as_ref()
            .expect("a named slot holds an object")
            .1;

        // "only clear the cached algorithms" and then `destroyProxy`, in that
        // order: the first frees the algorithm of every pair mentioning the
        // proxy and has to run while those pairs are still there.
        {
            let (cache, proxies) = self.broadphase.pair_cache_and_proxies();
            cache.clean_proxy_from_pairs(proxies, proxy);
        }
        self.broadphase.destroy_proxy(proxy);

        // `link2cow_.erase(name)` last, as upstream erases it last: nothing
        // above may reach a slot the broadphase can still name.
        let mut store = self.store.borrow_mut();
        store.slots[slot] = None;
        store.by_name.remove(name);
        true
    }

    /// `BulletCastBVHManager::setCastCollisionObjectsTransform`
    /// (`bullet_cast_bvh_manager.cpp:66-133`), less the shape walk, which is
    /// [`CastCollisionObject::set_cast_transforms`].
    ///
    /// A name the manager does not hold is ignored, as upstream's `find`
    /// against `end()` ignores it.
    ///
    /// # Panics
    ///
    /// If the named object is not one that sweeps -- upstream's
    /// `assert(cow->m_collisionFilterGroup == btBroadphaseProxy::KinematicFilter)`
    /// (`:75`). A static object reaches the manager as itself, so upstream's
    /// two states, wrong filter group and a shape that is neither convex nor
    /// compound, are one variant here.
    pub fn set_cast_collision_objects_transform(
        &mut self,
        name: &str,
        tf1: Transform,
        tf2: Transform,
    ) {
        let mut store = self.store.borrow_mut();
        let Some(&slot) = store.by_name.get(name) else {
            return;
        };
        let entry = store.slots[slot]
            .as_mut()
            .expect("a named slot holds an object");
        let proxy = entry.1;
        let ManagedObject::Cast(cast) = &mut entry.0 else {
            panic!(
                "only continuous collision check convex shapes and compound shapes made of convex shapes"
            );
        };
        cast.set_cast_transforms(tf1, tf2);

        // "Now update Broadphase AABB (See BulletWorld updateSingleAabb
        // function)" (`:130`) -- inside the `m_enabled` guard, so a disabled
        // object keeps the AABB it had while its world transform still moves.
        if !cast.object().enabled {
            return;
        }
        let (aabb_min, aabb_max) = cast.object().get_aabb();
        drop(store);
        self.broadphase.set_aabb(proxy, aabb_min, aabb_max);
    }

    /// `BulletCastBVHManager::contactTest`
    /// (`bullet_cast_bvh_manager.cpp:136-148`).
    ///
    /// `contact_distance_` is [`BULLET_DEFAULT_CONTACT_DISTANCE`]; see the
    /// module docs for why it is a constant here. The trailing `self` argument
    /// upstream takes and never reads is not carried, for the reason
    /// [`BroadphaseContactResultCallback`] does not carry `self_`.
    ///
    /// # Errors
    ///
    /// The first [`CastCallbackError`] any pair produced. No further pair is
    /// asked once one has failed: a result short an unknown number of contacts
    /// is not a result, and handing it back beside the error would invite
    /// reading it as one.
    pub fn contact_test(
        &mut self,
        request: &CastRequest,
        acm: Option<&dyn AllowedCollisions>,
    ) -> Result<CastResult, CastCallbackError> {
        self.broadphase.calculate_overlapping_pairs();

        let mut result = CastResult::default();
        let mut callback = BroadphaseContactResultCallback {
            request,
            result: &mut result,
            contact_distance: f64::from(BULLET_DEFAULT_CONTACT_DISTANCE),
            acm,
        };

        let store = Rc::clone(&self.store);
        let mut failure = None;
        let (cache, proxies) = self.broadphase.pair_cache_and_proxies();
        cache.process_all_overlapping_pairs(proxies, &mut |proxies, pair| {
            if failure.is_none() {
                let id0 = proxies.proxy(pair.proxy0).client_object;
                let id1 = proxies.proxy(pair.proxy1).client_object;
                let store = store.borrow();
                failure = process_overlap(
                    &mut callback,
                    store.object(id0),
                    id0,
                    store.object(id1),
                    id1,
                )
                .err();
            }
            // `processOverlap` returns false always: every pair it visited
            // stays in the cache.
            false
        });

        match failure {
            Some(error) => Err(error),
            None => Ok(result),
        }
    }
}

#[cfg(test)]
mod tests {
    use cspace_bullet::linear_math::{Matrix3, Scalar, Vec3};
    use cspace_core::geometry::Isometry3;
    use cspace_core::geometry::shapes::{Cuboid, Shape};

    use super::*;
    use crate::contact_test_data::{BodyType, Contact, object_pair_key};
    use crate::shape_primitive::CollisionObjectType;

    fn object(name: &str, type_id: BodyType, active: bool) -> CollisionObjectWrapper {
        CollisionObjectWrapper::new(
            name,
            type_id,
            &[Shape::Cuboid(Cuboid {
                size: [1.0, 1.0, 1.0],
            })],
            &[Isometry3::identity()],
            &[CollisionObjectType::UseShapeType],
            active,
        )
        .expect("one cuboid at one pose is a valid object")
    }

    fn shifted(x: Scalar) -> Transform {
        Transform::new(Matrix3::identity(), Vec3::new(x, 0.0, 0.0))
    }

    /// A manager holding one static box at the origin and one active link,
    /// swept from `from` to `to` along x.
    fn swept_scene(from: Scalar, to: Scalar) -> BulletCastBvhManager {
        let mut manager = BulletCastBvhManager::new();
        manager
            .add_collision_object(object("box", BodyType::WorldObject, false))
            .expect("a static box is added as itself");
        manager
            .add_collision_object(object("link", BodyType::RobotLink, true))
            .expect("a box is castable");
        manager.set_cast_collision_objects_transform("link", shifted(from), shifted(to));
        manager
    }

    fn contacts_of(result: &CastResult) -> &[Contact] {
        &result.contacts[&object_pair_key("box", "link")]
    }

    fn requested() -> CastRequest {
        CastRequest {
            contacts: true,
            ..CastRequest::default()
        }
    }

    /// The pair filter is installed *in* the broadphase, so a rejected pair is
    /// never stored -- which is the property the callback exists for and the
    /// one a query-time filter would not have.
    #[test]
    fn the_filter_runs_at_insert_time_so_a_rejected_pair_is_never_stored() {
        let mut two_static = BulletCastBvhManager::new();
        two_static
            .add_collision_object(object("a", BodyType::WorldObject, false))
            .expect("added");
        two_static
            .add_collision_object(object("b", BodyType::WorldObject, false))
            .expect("added");
        assert_eq!(
            two_static
                .broadphase
                .overlapping_pair_cache()
                .num_overlapping_pairs(),
            0,
            "two static objects are not a pair, however far they overlap"
        );

        let mut mixed = BulletCastBvhManager::new();
        mixed
            .add_collision_object(object("a", BodyType::WorldObject, false))
            .expect("added");
        mixed
            .add_collision_object(object("link", BodyType::RobotLink, true))
            .expect("added");
        assert_eq!(
            mixed
                .broadphase
                .overlapping_pair_cache()
                .num_overlapping_pairs(),
            1,
            "an active object against a static one is"
        );
    }

    /// The whole manager path: add, re-pose the sweep, query. The sweep ends
    /// 0.1 inside the box, and `addCastSingleResult` reports the *static* side
    /// first whichever side swept.
    #[test]
    fn a_sweep_into_a_box_is_reported_through_the_manager() {
        let mut manager = swept_scene(-3.0, -0.9);
        let result = manager
            .contact_test(&requested(), None)
            .expect("two boxes dispatch to convex-convex");

        assert!(result.collision);
        let contacts = contacts_of(&result);
        assert_eq!(contacts.len(), 1, "the default per-pair budget is one");
        assert_eq!(contacts[0].body_name_1, "box");
        assert_eq!(contacts[0].body_name_2, "link");
        assert_eq!(
            contacts[0].percent_interpolation, 1.0,
            "the second pose is the one touching"
        );
    }

    /// The broadphase's own answer: a sweep whose swept AABB never reaches the
    /// box is not offered to the narrow phase at all.
    #[test]
    fn a_sweep_that_stops_short_is_not_even_a_pair() {
        let mut manager = swept_scene(-5.0, -3.0);
        let result = manager
            .contact_test(&requested(), None)
            .expect("no pair is not a failure");

        assert!(!result.collision);
        assert!(result.contacts.is_empty());
        assert_eq!(
            manager
                .broadphase
                .overlapping_pair_cache()
                .num_overlapping_pairs(),
            0,
            "the pair created at insert time, when the link still sat at the \
             origin, is cleaned up once the sweep moves it away"
        );
    }

    /// Removal takes the object out of the pair cache as well as out of the
    /// map -- otherwise a proxy would outlive its slot and the store's
    /// `expect` would fire on the next query.
    #[test]
    fn a_removed_object_takes_its_pairs_with_it() {
        let mut manager = swept_scene(-3.0, -0.9);
        assert!(manager.remove_collision_object("link"));
        assert_eq!(
            manager
                .broadphase
                .overlapping_pair_cache()
                .num_overlapping_pairs(),
            0
        );

        let result = manager
            .contact_test(&requested(), None)
            .expect("one object is no pair");
        assert!(!result.collision);

        assert!(
            !manager.remove_collision_object("link"),
            "a name that is gone is not there to remove twice"
        );
        assert!(!manager.remove_collision_object("never added"));
    }

    /// A slot is never reused, so the object added after a removal must not
    /// land on the freed index -- a stale proxy would then name it.
    #[test]
    fn a_readded_name_takes_a_fresh_slot() {
        let mut manager = swept_scene(-3.0, -0.9);
        let first = manager.store.borrow().by_name["link"];
        assert!(manager.remove_collision_object("link"));
        manager
            .add_collision_object(object("link", BodyType::RobotLink, true))
            .expect("the name is free again");
        let second = manager.store.borrow().by_name["link"];

        assert_ne!(first, second);
        assert!(
            manager.store.borrow().slots[first].is_none(),
            "the removed object's slot stays empty"
        );
    }

    /// The state upstream leaves undefined is refused, and refusing it leaves
    /// the object already there untouched.
    #[test]
    fn a_duplicate_name_is_refused() {
        let mut manager = swept_scene(-3.0, -0.9);
        let error = manager
            .add_collision_object(object("link", BodyType::RobotLink, true))
            .expect_err("the name is taken");
        assert_eq!(error, AddObjectError::DuplicateName("link".to_owned()));

        let result = manager
            .contact_test(&requested(), None)
            .expect("the scene is unchanged");
        assert!(
            result.collision,
            "the sweep that was already there still reaches the box"
        );
        assert_eq!(
            contacts_of(&result).len(),
            1,
            "and it is one object, not two"
        );
    }

    /// `link2cow_.find(name) != link2cow_.end()` guards the whole body.
    #[test]
    fn re_posing_a_name_the_manager_does_not_hold_is_a_no_op() {
        let mut manager = swept_scene(-3.0, -0.9);
        manager.set_cast_collision_objects_transform("absent", shifted(0.0), shifted(100.0));

        let result = manager
            .contact_test(&requested(), None)
            .expect("the scene is unchanged");
        assert!(result.collision);
    }

    /// "If collision object is disabled don't proceed": the world transform is
    /// written either way, and the broadphase AABB is not.
    #[test]
    fn a_disabled_object_keeps_the_aabb_it_had() {
        let mut manager = BulletCastBvhManager::new();
        let mut link = object("link", BodyType::RobotLink, true);
        link.enabled = false;
        manager.add_collision_object(link).expect("added");

        let slot = manager.store.borrow().by_name["link"];
        let proxy = manager.store.borrow().slots[slot]
            .as_ref()
            .expect("just added")
            .1;
        let before = manager.broadphase.get_aabb(proxy);

        manager.set_cast_collision_objects_transform("link", shifted(50.0), shifted(60.0));

        assert_eq!(manager.broadphase.get_aabb(proxy), before);
        assert_eq!(
            manager.store.borrow().object(slot).world_transform().origin,
            Vec3::new(50.0, 0.0, 0.0),
            "the world transform moves even so"
        );
    }

    /// An allowed pair is dropped in `needsCollision`, after the broadphase
    /// has already stored it -- so the pair is there and the contact is not.
    #[test]
    fn an_allowed_pair_produces_no_contact() {
        struct AllowAll;
        impl AllowedCollisions for AllowAll {
            fn allowed_collision(
                &self,
                _body_1: &str,
                _body_2: &str,
            ) -> Option<crate::cast_callback::AllowedCollisionType> {
                Some(crate::cast_callback::AllowedCollisionType::Always)
            }
        }

        let mut manager = swept_scene(-3.0, -0.9);
        let result = manager
            .contact_test(&requested(), Some(&AllowAll))
            .expect("an allowed pair is not a failure");

        assert!(!result.collision);
        assert_eq!(
            manager
                .broadphase
                .overlapping_pair_cache()
                .num_overlapping_pairs(),
            1,
            "the ACM is not the broadphase filter; the pair is stored either way"
        );
    }

    /// A static object is added as itself, so there is no sweep to re-aim --
    /// upstream's `assert(cow->m_collisionFilterGroup == KinematicFilter)`.
    #[test]
    #[should_panic(expected = "only continuous collision check convex shapes")]
    fn re_posing_a_static_object_is_the_assertion_upstream_makes() {
        let mut manager = BulletCastBvhManager::new();
        manager
            .add_collision_object(object("box", BodyType::WorldObject, false))
            .expect("added");
        manager.set_cast_collision_objects_transform("box", shifted(0.0), shifted(1.0));
    }
}
