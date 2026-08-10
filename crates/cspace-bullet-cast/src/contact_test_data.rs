// Copyright (c) 2021, Southwest Research Institute
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-2-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection_bullet/src/bullet_integration/contact_checker_common.cpp

//! `processResult` -- the accumulation policy that decides whether a contact is
//! stored, and when the traversal may stop.
//!
//! # Why this crate carries its own request and result
//!
//! Upstream's `ContactTestData` holds references to
//! `collision_detection::CollisionRequest` and `CollisionResult`, which this
//! workspace ports in `cspace_collision::common` -- and which this crate cannot
//! name. `cspace_collision` is BSD-3-Clause and depends on this crate for the
//! continuous check; `tools/ci/check-license-matches-upstream.sh` requires one
//! SPDX identifier per crate, so the dependency edge can only run one way and
//! this BSD-2-Clause code cannot move into `cspace_collision`.
//!
//! That boundary is upstream's own: `contact_checker_common.cpp` and
//! `bullet_utils.cpp` are BSD-2-Clause, and `collision_env_bullet.cpp` -- the
//! caller, `checkRobotCollisionHelperCCD` -- is BSD-3-Clause. So [`CastRequest`]
//! and [`CastResult`] are not a second definition of a collision request and
//! result in general: they are exactly the fields `processResult` reads and
//! writes, and `cspace_collision` converts at the same seam upstream's licences
//! already cut.
//!
//! # The order the caller must not lose
//!
//! [`process_result`] sets [`CastResult::done`] partway through a traversal, and
//! [`CastRequest::max_contacts`] truncates. Which contacts survive therefore
//! depends on the order pairs are visited in, not only on which pairs collide.
//! That is why the broadphase's pair order is reproduced rather than
//! approximated -- see `cspace_bullet::dbvt_broadphase`.

use std::collections::BTreeMap;

use cspace_bullet::linear_math::{Scalar, Vec3};

/// `collision_detection::BodyType` (`collision_common.hpp:57-68`).
///
/// Re-declared here rather than taken from `cspace_collision` for the reason
/// the module docs give; `cspace_collision` maps this onto its own.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum BodyType {
    /// `ROBOT_LINK` -- a link of the robot.
    RobotLink,
    /// `ROBOT_ATTACHED` -- a body attached to a robot link.
    RobotAttached,
    /// `WORLD_OBJECT` -- an object in the collision world.
    WorldObject,
}

/// `collision_detection::Contact` (`collision_common.hpp:74-112`), reduced to
/// the fields the continuous path writes and reads.
///
/// This is the stored record, and every field on it is already in its final
/// form. In particular the two bodies are the way round the result reports
/// them: `addCastSingleResult` exchanges the names and the types in place
/// (`bullet_utils.hpp:464-465`) so the *non*-swept object is reported first,
/// and [`crate::cast_contact::apply_cast_result`] performs that exchange
/// rather than recording that one is owed.
///
/// `nearest_points` is absent. `addDiscreteSingleResult` fills it
/// (`bullet_utils.hpp:405-406`); the cast path never assigns it and
/// `collision_detection::Contact` gives it no default initialiser
/// (`collision_common.hpp:105`), so what the swap at `:462` exchanges is
/// whatever the stack held. There is no value to reproduce.
///
/// `cost_density`, `subframe_1`/`subframe_2` and the two `nearest_points`
/// members are likewise untouched by either bullet result callback.
#[derive(Clone, Debug, PartialEq)]
pub struct Contact {
    /// `body_name_1`.
    pub body_name_1: String,
    /// `body_name_2`.
    pub body_name_2: String,
    /// `body_type_1`.
    pub body_type_1: BodyType,
    /// `body_type_2`.
    pub body_type_2: BodyType,
    /// `normal` -- `-1 * cp.m_normalWorldOnB`, negated once more when the
    /// swept object was the first of the pair.
    pub normal: Vec3,
    /// `pos`.
    pub pos: Vec3,
    /// `depth` -- `cp.m_distance1`, negative when the shapes overlap.
    pub depth: Scalar,
    /// `percent_interpolation` -- 0 at the start pose, 1 at the end pose.
    /// Zero until `addCastSingleResult`'s tail computes it, which happens
    /// only for a contact this pair actually stored.
    pub percent_interpolation: Scalar,
}

/// The fields of `collision_detection::CollisionRequest` that `processResult`
/// and the cast callbacks read.
///
/// `group_name`, `cost`, `detailed_distance`, `max_cost_sources`, `is_done` and
/// `verbose` are absent because nothing on the continuous path reads them:
/// `checkRobotCollisionHelperCCD` selects its links before the manager is
/// entered, and Bullet's backend computes neither costs nor a detailed
/// distance.
#[derive(Clone, Copy, Debug)]
pub struct CastRequest {
    /// `distance` -- compute a proximity distance. Also the flag that stops
    /// `done` ever being set: with it, upstream keeps traversing after the
    /// contact budget is spent so the minimum distance is over every pair.
    pub distance: bool,
    /// `contacts` -- store contacts, rather than reporting a boolean.
    pub contacts: bool,
    /// `max_contacts` -- overall contact budget.
    pub max_contacts: usize,
    /// `max_contacts_per_pair` -- per-pair contact budget. At 1, upstream sets
    /// `pair_done` on the first contact of each pair.
    pub max_contacts_per_pair: usize,
}

impl Default for CastRequest {
    /// The defaults `collision_detection::CollisionRequest` declares
    /// (`collision_common.hpp:147-186`): no distance, no contacts, and budgets
    /// of one.
    fn default() -> Self {
        Self {
            distance: false,
            contacts: false,
            max_contacts: 1,
            max_contacts_per_pair: 1,
        }
    }
}

/// The fields of `collision_detection::CollisionResult` that `processResult`
/// writes, plus the two `ContactTestData` flags that steer the traversal.
///
/// `done` and `pair_done` live here rather than in a separate `ContactTestData`
/// because they are written by the same function that writes the rest and read
/// by the same traversal; upstream splits them only because `res` is a
/// reference into the caller's object.
#[derive(Clone, Debug)]
pub struct CastResult {
    /// `res.collision`.
    pub collision: bool,
    /// `res.distance` -- the smallest depth seen, `f64::MAX` until one is.
    ///
    /// Updated for every contact reaching `processResult`, including the ones
    /// it then declines to store.
    pub distance: f64,
    /// `res.contact_count` -- the number of contacts stored.
    pub contact_count: usize,
    /// `res.contacts`, keyed by `getObjectPairKey`: the two body names in
    /// ascending order, which is what makes a pair's key independent of which
    /// side the broadphase presented first.
    ///
    /// A [`BTreeMap`], because upstream's is a `std::map` and the iteration
    /// order of the finished result is part of what a caller sees.
    pub contacts: BTreeMap<(String, String), Vec<Contact>>,
    /// `cdata.done` -- stop the whole traversal.
    pub done: bool,
    /// `cdata.pair_done` -- stop this pair, cleared by the traversal at the
    /// start of each pair.
    pub pair_done: bool,
}

impl Default for CastResult {
    /// `CollisionResult::clear()` (`collision_common.hpp:339-347`), which is
    /// also the state a fresh one is constructed in.
    fn default() -> Self {
        Self {
            collision: false,
            distance: f64::MAX,
            contact_count: 0,
            contacts: BTreeMap::new(),
            done: false,
            pair_done: false,
        }
    }
}

/// `getObjectPairKey(obj1, obj2)` -- the two names in ascending order.
///
/// Stated rather than transcribed: the upstream definition is an `inline` in
/// `contact_checker_common.hpp`, which is Apache-2.0 and so not portable into
/// this BSD-2-Clause crate. The rule it states is one comparison, and
/// `the_pair_key_is_order_independent` in this module's tests is what holds
/// this port to it.
#[must_use]
pub fn object_pair_key(obj1: &str, obj2: &str) -> (String, String) {
    if obj1 < obj2 {
        (obj1.to_string(), obj2.to_string())
    } else {
        (obj2.to_string(), obj1.to_string())
    }
}

/// Whether `processResult` stored the contact, and so whether the caller may go
/// on to fill in its `percent_interpolation`.
///
/// Upstream returns `collision_detection::Contact*` -- null for "not stored",
/// and otherwise a pointer *into* `res.contacts` that `addCastSingleResult`
/// then writes through. A pointer into a `BTreeMap`'s value would borrow the
/// result for the rest of the call, so this port returns the key instead and
/// the caller writes back through [`CastResult::last_contact_mut`].
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Stored {
    /// `nullptr` -- the contact was counted but not kept.
    No,
    /// The contact was appended to this pair's vector, and is its last element.
    Yes {
        /// The `getObjectPairKey` the contact was filed under.
        key: (String, String),
    },
}

impl CastResult {
    /// The contact [`process_result`] most recently stored under `key`, which
    /// is that pair's last element.
    ///
    /// Upstream's `processResult` hands back `&(...->second.back())` and
    /// `addCastSingleResult` writes `col->percent_interpolation` and the two
    /// swaps through it; this is that pointer, re-acquired.
    pub fn last_contact_mut(&mut self, key: &(String, String)) -> Option<&mut Contact> {
        self.contacts.get_mut(key).and_then(|pair| pair.last_mut())
    }
}

/// `processResult(cdata, contact, key, found)`
/// (`contact_checker_common.cpp:44-118`).
///
/// `found` is upstream's "this pair already has a contact", which it computes
/// as a lookup that this port folds in: the map is consulted here rather than
/// by the caller, because the two must agree and a caller that checked a
/// different key would silently start a second vector.
///
/// # The `distance` flag suppresses stopping, it does not only add a number
///
/// Every `cdata.done = true` upstream is guarded by `if (!cdata.req.distance)`.
/// A distance request therefore visits every pair even after the contact budget
/// is spent -- which is what makes the reported minimum a minimum over the whole
/// scene rather than over a prefix of it.
pub fn process_result(
    result: &mut CastResult,
    request: &CastRequest,
    contact: Contact,
    key: (String, String),
) -> Stored {
    // add deepest penetration / smallest distance to result
    if request.distance {
        let depth = f64::from(contact.depth);
        if depth < result.distance {
            result.distance = depth;
        }
    }

    let found = result.contacts.contains_key(&key);

    // case if pair hasn't a contact yet
    if !found {
        if contact.depth <= 0.0 {
            result.collision = true;
        }

        // if we don't want contacts we are done here
        if !request.contacts {
            if !request.distance {
                result.done = true;
            }
            return Stored::No;
        }

        result.contacts.insert(key.clone(), vec![contact]);
        result.contact_count += 1;

        if result.contact_count >= request.max_contacts && !request.distance {
            result.done = true;
        }

        if request.max_contacts_per_pair == 1 {
            result.pair_done = true;
        }

        Stored::Yes { key }
    } else {
        let dr = result.contacts.entry(key.clone()).or_default();
        dr.push(contact);
        let pair_len = dr.len();
        result.contact_count += 1;

        if pair_len >= request.max_contacts_per_pair {
            result.pair_done = true;
        }

        if result.contact_count >= request.max_contacts && !request.distance {
            result.done = true;
        }

        Stored::Yes { key }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn contact(depth: Scalar) -> Contact {
        Contact {
            body_name_1: "a".to_owned(),
            body_name_2: "b".to_owned(),
            body_type_1: BodyType::RobotLink,
            body_type_2: BodyType::WorldObject,
            normal: Vec3::new(0.0, 0.0, 1.0),
            pos: Vec3::zero(),
            depth,
            percent_interpolation: 0.0,
        }
    }

    fn key(a: &str, b: &str) -> (String, String) {
        object_pair_key(a, b)
    }

    #[test]
    fn the_pair_key_is_order_independent() {
        assert_eq!(key("a", "b"), key("b", "a"));
        assert_eq!(key("a", "b"), ("a".to_string(), "b".to_string()));
        // Equal names are not a special case upstream: `obj1 < obj2` is false,
        // so the `else` arm runs and the pair is `(obj2, obj1)` -- the same
        // string twice either way.
        assert_eq!(key("a", "a"), ("a".to_string(), "a".to_string()));
    }

    /// With `contacts` unset the first contact ends the traversal and nothing
    /// is stored -- but `collision` is still set, which is the whole answer
    /// such a request asked for.
    #[test]
    fn a_boolean_request_stops_on_the_first_contact_without_storing_it() {
        let mut result = CastResult::default();
        let request = CastRequest::default();

        let stored = process_result(&mut result, &request, contact(-0.1), key("a", "b"));

        assert_eq!(stored, Stored::No);
        assert!(result.collision);
        assert!(result.done);
        assert_eq!(result.contact_count, 0);
        assert!(result.contacts.is_empty());
    }

    /// A distance request never sets `done`, so the traversal keeps running
    /// after the contact budget is spent.
    #[test]
    fn a_distance_request_never_stops_early() {
        let mut result = CastResult::default();
        let request = CastRequest {
            distance: true,
            ..CastRequest::default()
        };

        assert_eq!(
            process_result(&mut result, &request, contact(-0.1), key("a", "b")),
            Stored::No
        );
        assert!(!result.done, "req.distance suppresses every done");
        assert_eq!(result.distance, f64::from(-0.1_f32));

        // A shallower contact does not replace a deeper one.
        let _ = process_result(&mut result, &request, contact(0.5), key("c", "d"));
        assert_eq!(result.distance, f64::from(-0.1_f32));
        assert!(!result.done);
    }

    /// `depth <= 0` is a collision; a positive depth is a near miss that is
    /// still reported when it is within the contact distance.
    #[test]
    fn a_positive_depth_is_reported_without_setting_collision() {
        let mut result = CastResult::default();
        let request = CastRequest {
            contacts: true,
            max_contacts: 10,
            max_contacts_per_pair: 10,
            ..CastRequest::default()
        };

        let _ = process_result(&mut result, &request, contact(0.25), key("a", "b"));
        assert!(!result.collision);
        assert_eq!(result.contact_count, 1);

        let _ = process_result(&mut result, &request, contact(-0.0), key("c", "d"));
        assert!(
            result.collision,
            "a depth of exactly zero is a collision: the comparison is `<= 0`"
        );
    }

    /// The per-pair budget stops the pair; the overall budget stops everything.
    #[test]
    fn the_two_budgets_stop_different_things() {
        let mut result = CastResult::default();
        let request = CastRequest {
            contacts: true,
            max_contacts: 3,
            max_contacts_per_pair: 2,
            ..CastRequest::default()
        };

        let _ = process_result(&mut result, &request, contact(-0.1), key("a", "b"));
        assert!(!result.pair_done, "one of two is not the pair's budget");
        assert!(!result.done);

        let _ = process_result(&mut result, &request, contact(-0.2), key("a", "b"));
        assert!(result.pair_done, "two of two is");
        assert!(!result.done, "two of three is not the overall budget");

        result.pair_done = false;
        let _ = process_result(&mut result, &request, contact(-0.3), key("c", "d"));
        assert!(result.done, "three of three is");
        assert!(
            !result.pair_done,
            "a new pair's first contact does not reach its own budget of two"
        );
        assert_eq!(result.contact_count, 3);
        assert_eq!(result.contacts[&key("a", "b")].len(), 2);
        assert_eq!(result.contacts[&key("c", "d")].len(), 1);
    }

    /// A per-pair budget of exactly one is special-cased upstream: `pair_done`
    /// is set on the *first* contact, before any second one can be compared
    /// against the budget.
    #[test]
    fn a_per_pair_budget_of_one_stops_the_pair_on_its_first_contact() {
        let mut result = CastResult::default();
        let request = CastRequest {
            contacts: true,
            max_contacts: 10,
            max_contacts_per_pair: 1,
            ..CastRequest::default()
        };

        let _ = process_result(&mut result, &request, contact(-0.1), key("a", "b"));
        assert!(result.pair_done);
        assert_eq!(result.contact_count, 1);
    }

    /// The returned key names the contact the caller then writes
    /// `percent_interpolation` through, and it is the pair's *last* element --
    /// not its first.
    #[test]
    fn the_stored_key_reaches_the_contact_just_appended() {
        let mut result = CastResult::default();
        let request = CastRequest {
            contacts: true,
            max_contacts: 10,
            max_contacts_per_pair: 10,
            ..CastRequest::default()
        };

        let _ = process_result(&mut result, &request, contact(-0.1), key("a", "b"));
        let stored = process_result(&mut result, &request, contact(-0.2), key("a", "b"));

        let Stored::Yes { key: k } = stored else {
            panic!("the second contact of a pair is stored");
        };
        let last = result.last_contact_mut(&k).expect("just appended");
        assert_eq!(last.depth, -0.2);
        last.percent_interpolation = 0.75;
        assert_eq!(
            result.contacts[&key("a", "b")][0].percent_interpolation,
            0.0
        );
        assert_eq!(
            result.contacts[&key("a", "b")][1].percent_interpolation,
            0.75
        );
    }
}
