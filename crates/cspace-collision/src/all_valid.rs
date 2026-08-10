// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection/include/moveit/collision_detection/allvalid/collision_env_allvalid.hpp
//   moveit_core/collision_detection/src/allvalid/collision_env_allvalid.cpp

//! Upstream `collision_detection::CollisionEnvAllValid`: the backend that
//! reports no collision, ever, so a pipeline can be run with collision
//! checking off ("This can be used to save resources if collision checking
//! is not important", `collision_env_allvalid.hpp:43-45`).
//!
//! # How it is selected
//!
//! Upstream reaches this class through
//! `CollisionDetectorAllocatorAllValid`, whose `NAME` is `"ALL_VALID"`
//! (`collision_env_allvalid.cpp:53`) — a plugin `PlanningScene` looks up by
//! that string. This port has no allocator and no name lookup (see `env`'s
//! module doc and `PORTING-PLAN.md` §225.4): every
//! `cspace_scene::PlanningScene` collision method is generic over a
//! caller-supplied `E: CollisionEnv<Posed<'_, 'm>>`, so selecting this
//! backend means passing [`AllValidCollisionEnv`] where
//! [`crate::ParryCollisionEnv`] would otherwise go. That is the *whole*
//! selection path, and it is the one
//! `crates/cspace-scene/tests/all_valid_selection.rs` exercises — the same
//! scene, state and request answered two different ways depending only on
//! which `E` the caller named.
//!
//! # Not a `World`, not a `LinkPaddingScale`
//!
//! Upstream's three constructors (`collision_env_allvalid.cpp:55-70`) do
//! nothing but forward `(padding, scale)`, `(world, padding, scale)` or
//! `(other, world)` to the `CollisionEnv` base, whose padding/scale maps and
//! world this class then never reads — every method below ignores every
//! input it is given. This port's equivalent state does not live on the
//! backend trait at all ([`crate::LinkPaddingScale`] is a free-standing
//! struct, and [`crate::World`] is a value the caller owns; see `env`'s
//! module doc), so there is nothing for a constructor to forward and
//! [`AllValidCollisionEnv`] is a unit struct. A caller that wants
//! padding/scale bookkeeping alongside a disabled collision check holds its
//! own [`crate::LinkPaddingScale`], exactly as it would with any other
//! backend.
//!
//! # `distanceRobot`'s two upstream answers, and the one taken here
//!
//! Upstream declares `virtual double distanceRobot(state) const` returning
//! `0.0` (`collision_env_allvalid.cpp:114-123`), but `CollisionEnv`'s
//! same-named convenience overload is **non-virtual**
//! (`collision_env.hpp:202`), so that declaration overrides nothing — it
//! hides the base's, and a caller holding the `CollisionEnvPtr` the
//! allocator hands out gets the base version instead, which returns
//! `DistanceResultsData`'s default `std::numeric_limits<double>::max()`.
//! Two answers for one query, chosen by the static type of the expression;
//! see `doc/upstream-bugs.md`'s `all-valid-distance-robot-hides-base-overload`.
//! This port cannot express the split — [`CollisionEnv`] has no
//! `distance_robot(state)` convenience overload at all (`env`'s module doc:
//! both need `req.enableGroup(getRobotModel())`) — and lands on the value
//! the base version returns, [`crate::DistanceResultsData::default`]'s
//! `f64::MAX`. That is also the semantically right one: `0.0` is the
//! collision boundary this backend's whole purpose is to stay away from
//! (`DistanceResultsData::distance`'s own doc: "If two objects are in
//! collision, distance <= 0").

use cspace_error::Result;

use crate::common::{
    AttachedBodyGeometry, CollisionDistance, CollisionRequest, CollisionResult, ContactData,
    DistanceRequest, DistanceResult,
};
use crate::env::CollisionEnv;
use crate::matrix::AllowedCollisionMatrix;

/// A [`CollisionEnv`] that reports no collision for any state, any world and
/// any request.
///
/// Upstream `collision_detection::CollisionEnvAllValid`. Generic over
/// `State` rather than fixed to one, because nothing here reads the state:
/// this backend substitutes for any other at any call site, whatever state
/// type that site uses.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct AllValidCollisionEnv;

/// Upstream sets only `res.collision = false` and leaves every other field
/// of the freshly-constructed `CollisionResult` alone
/// (`collision_env_allvalid.cpp:75,84,93,103,128,137`). This port's
/// [`CollisionResult`] encodes "the caller asked for this" in the option
/// itself rather than in a separate request flag the reader has to
/// re-consult, so the three optional fields are filled in exactly when
/// `request` asked for them, each with what upstream's default-constructed
/// result holds: no contacts, no cost sources, and
/// `std::numeric_limits<double>::max()` for the distance.
fn nothing_collides(request: &CollisionRequest) -> CollisionResult {
    CollisionResult {
        collision: false,
        distance: request
            .distance
            .then_some(CollisionDistance::Closest(f64::MAX)),
        contacts: request.contacts.then(ContactData::default),
        cost_sources: request.cost.then(Vec::new),
    }
}

impl<State> CollisionEnv<State> for AllValidCollisionEnv {
    fn check_self_collision(
        &self,
        request: &CollisionRequest,
        _state: &State,
        _attached_bodies: &[AttachedBodyGeometry<'_>],
        _acm: Option<&AllowedCollisionMatrix>,
    ) -> CollisionResult {
        nothing_collides(request)
    }

    fn check_robot_collision(
        &self,
        request: &CollisionRequest,
        _state: &State,
        _attached_bodies: &[AttachedBodyGeometry<'_>],
        _acm: Option<&AllowedCollisionMatrix>,
    ) -> CollisionResult {
        nothing_collides(request)
    }

    /// Unlike [`crate::ParryCollisionEnv`], which returns `Err` here because
    /// it has no swept query and will not guess, this backend answers: its
    /// claim is that nothing collides anywhere, which covers a path as
    /// exactly as it covers a state. Upstream agrees — it overrides both
    /// two-state `checkRobotCollision` forms with the same `res.collision =
    /// false` (`collision_env_allvalid.cpp:89-106`) rather than leaving them
    /// to the FCL backend's log-an-error-and-return no-op.
    fn check_robot_collision_continuous(
        &self,
        request: &CollisionRequest,
        _state1: &State,
        _state2: &State,
        _attached_bodies: &[AttachedBodyGeometry<'_>],
        _acm: Option<&AllowedCollisionMatrix>,
    ) -> Result<CollisionResult> {
        Ok(nothing_collides(request))
    }

    /// `distanceSelf(req, res, state)`: upstream sets only `res.collision =
    /// false` (`collision_env_allvalid.cpp:142-146`), leaving
    /// `minimum_distance.distance` at the `DistanceResultsData` constructor's
    /// `std::numeric_limits<double>::max()` and `distances` empty — which is
    /// [`DistanceResult::default`] here, field for field.
    fn distance_self(
        &self,
        _request: &DistanceRequest<'_>,
        _state: &State,
        _attached_bodies: &[AttachedBodyGeometry<'_>],
    ) -> DistanceResult {
        DistanceResult::default()
    }

    /// `distanceRobot(req, res, state)`: same body as
    /// [`AllValidCollisionEnv::distance_self`] upstream
    /// (`collision_env_allvalid.cpp:108-112`). See this module's doc comment
    /// for the `0.0`-returning sibling overload that is not ported and why
    /// `f64::MAX` is the right answer.
    fn distance_robot(
        &self,
        _request: &DistanceRequest<'_>,
        _state: &State,
        _attached_bodies: &[AttachedBodyGeometry<'_>],
    ) -> DistanceResult {
        DistanceResult::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Stands in for a real robot state: nothing here reads it, which is the
    /// point of the blanket `impl<State>`.
    struct AnyState;

    /// The three `Option` fields track `request`, not the (empty) findings —
    /// so a caller that asked for contacts gets `Some(empty)`, the same
    /// "asked, found none" upstream expresses by leaving a
    /// default-constructed `CollisionResult` alone.
    #[test]
    fn optional_result_fields_follow_the_request_not_the_findings() {
        let env = AllValidCollisionEnv;
        let bare = CollisionRequest::default();
        let asked = CollisionRequest {
            distance: true,
            contacts: true,
            cost: true,
            ..CollisionRequest::default()
        };

        let r = env.check_self_collision(&bare, &AnyState, &[], None);
        assert!(!r.collision);
        assert!(r.distance.is_none());
        assert!(r.contacts.is_none());
        assert!(r.cost_sources.is_none());

        let r = env.check_self_collision(&asked, &AnyState, &[], None);
        assert!(!r.collision);
        assert_eq!(r.distance.map(|d| d.distance()), Some(f64::MAX));
        assert_eq!(r.contacts.map(|c| c.count()), Some(0));
        assert_eq!(r.cost_sources.map(|c| c.len()), Some(0));
    }

    /// `check_collision`'s default runs self- then robot-collision and merges;
    /// with both halves empty the merge must not invent a collision, and must
    /// keep the requested `distance` slot filled.
    #[test]
    fn check_collision_default_merges_two_empty_halves() {
        let env = AllValidCollisionEnv;
        let request = CollisionRequest {
            distance: true,
            ..CollisionRequest::default()
        };
        let r = env.check_collision(&request, &AnyState, &[], None);
        assert!(!r.collision);
        assert_eq!(r.distance.map(|d| d.distance()), Some(f64::MAX));
    }

    /// The continuous form answers instead of erroring — the one place this
    /// backend's contract differs from [`crate::ParryCollisionEnv`]'s.
    #[test]
    fn continuous_check_answers_rather_than_erroring() {
        let env = AllValidCollisionEnv;
        let r = env
            .check_robot_collision_continuous(
                &CollisionRequest::default(),
                &AnyState,
                &AnyState,
                &[],
                None,
            )
            .expect("the all-valid backend answers the continuous query");
        assert!(!r.collision);
    }

    /// `f64::MAX`, not `0.0` — see this module's doc comment on upstream's
    /// hidden overload.
    #[test]
    fn distance_queries_report_maximum_clearance() {
        let env = AllValidCollisionEnv;
        let request = DistanceRequest::default();
        assert_eq!(
            env.distance_self(&request, &AnyState, &[])
                .minimum_distance
                .distance,
            f64::MAX
        );
        assert_eq!(
            env.distance_robot(&request, &AnyState, &[])
                .minimum_distance
                .distance,
            f64::MAX
        );
    }
}
