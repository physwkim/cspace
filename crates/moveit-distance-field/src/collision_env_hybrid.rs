// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_distance_field/include/moveit/collision_distance_field/collision_env_hybrid.hpp
//   moveit_core/collision_distance_field/src/collision_env_hybrid.cpp

//! [`HybridCollisionEnv`]: upstream `CollisionEnvHybrid`, which combines a
//! general-purpose world-collision backend (upstream: FCL; this port:
//! [`moveit_collision::ParryCollisionEnv`]) with a distance-field
//! self-collision cache ([`DistanceFieldCollisionCache`], this crate's own
//! `CollisionEnvDistanceField` port) behind one type.
//!
//! # §186: why this file exists at all
//!
//! A previous round of this crate's own doc comment excluded this whole
//! upstream file with an inheritance-graph argument: `CollisionEnvHybrid`
//! "extends `collision_detection::CollisionEnvFCL` directly," and since
//! `CollisionEnvFCL` is never ported (D4.5's FCL/Bullet -> `parry3d-f64`
//! backend replacement), "nothing depending on it directly can be either."
//! That argument was measured false: of `CollisionEnvHybrid`'s 22 members,
//! only 4 touch the `CollisionEnvFCL` base at all -- 3 constructors'
//! base-init calls (passing state declared on `CollisionEnv` itself, not
//! FCL-specific) and one explicit `CollisionEnvFCL::setWorld` call inside
//! `CollisionEnvHybrid::setWorld` (`collision_env_hybrid.cpp:49,61,69,169`).
//! See this crate's `lib.rs` module doc for the full count and the
//! `setWorld` analysis. This is the third exclusion this session found false
//! by counting calls instead of trusting a relationship (§139, then
//! `plan_components_builder`, now this one).
//!
//! # Public shape
//!
//! [`HybridCollisionEnv`] holds a [`ParryCollisionEnv`] and a
//! [`DistanceFieldCollisionCache`] side by side -- composition, not
//! inheritance, matching every other multi-backend type in this workspace.
//! Two upstream behaviors, two different Rust answers:
//!
//! - Upstream's *unsuffixed* `checkCollision`/`checkSelfCollision`/
//!   `checkRobotCollision` are **not overridden** by `CollisionEnvHybrid` --
//!   they are inherited from `CollisionEnvFCL` unchanged, so a
//!   `CollisionEnvHybrid` used through its `CollisionEnv` interface is
//!   purely FCL-backed (self-collision included). This port's
//!   `impl `[`CollisionEnv`]` for `[`HybridCollisionEnv`] is pure delegation
//!   to the held [`ParryCollisionEnv`], preserving that same
//!   substitutability for any caller generic over `impl CollisionEnv<State>`
//!   -- the composition analog of upstream's inheritance-based one.
//! - The 12 `check{Self,Robot,}CollisionDistanceField` overloads
//!   (`collision_env_hybrid.hpp:88-133`) already collapsed to one method per
//!   operation when [`DistanceFieldCollisionCache`] itself was ported --
//!   `acm: Option<&AllowedCollisionMatrix>` uniformly, `gsr` always returned
//!   rather than taken in/out. Nothing new collapses here:
//!   [`HybridCollisionEnv::check_self_collision_distance_field`] and its
//!   siblings below are thin forwarders to that already-shipped,
//!   already-tested signature. No caller-visible semantic changes --  only
//!   the call shape differs from upstream's arity-per-overload C++.
//!
//! `getCollisionRobotDistanceField`/`getCollisionWorldDistanceField`
//! (`collision_env_hybrid.hpp:104-107,143-146`, both `return cenv_distance_`)
//! are ported as [`HybridCollisionEnv::distance_field`]/
//! [`HybridCollisionEnv::distance_field_mut`]. `initializeRobotDistanceField`
//! (`:79-86`, forwards to `cenv_distance_->initialize(...)`) has no separate
//! port: this crate already folded upstream's `CollisionEnvDistanceField::initialize`
//! into [`DistanceFieldCollisionCache::new`] itself (construction-time, not a
//! later call) when that type was ported, so there is no post-construction
//! initialize step left to wrap.

use moveit_collision::{
    AllowedCollisionMatrix, AttachedBodyGeometry, CollisionEnv, CollisionRequest, CollisionResult,
    DistanceRequest, DistanceResult, LinkPaddingScale, ParryCollisionEnv, World,
};
use moveit_error::Result;
use moveit_state::Posed;

use crate::collision_env_distance_field::LinkBodyDecompositions;
use crate::{DistanceFieldCollisionCache, DistanceFieldConfig, GroupStateRepresentation};

/// Upstream `CollisionEnvHybrid`. See this module's doc comment for the
/// public shape and the §186 measurement that unblocked porting it.
pub struct HybridCollisionEnv<'m> {
    parry: ParryCollisionEnv,
    distance_field: DistanceFieldCollisionCache<'m>,
}

impl<'m> HybridCollisionEnv<'m> {
    /// Upstream `CollisionEnvHybrid`'s two-`World`-taking constructor
    /// (`collision_env_hybrid.hpp:63-71`); the one-`World`-taking overload
    /// (`:53-61`, `World()`-default) is upstream's own convenience overload
    /// for "start with an empty world," which this port's caller gets by
    /// passing `World::new()` explicitly instead of a second constructor.
    /// The copy-with-new-world constructor (`:73`) is not ported: nothing in
    /// this crate's own [`DistanceFieldCollisionCache`] port has a copy
    /// constructor either (`Clone`-if-needed is the idiomatic Rust answer,
    /// and `DistanceFieldCollisionCache`'s `cache_entry` field is not
    /// `Clone` -- see that type's own doc).
    pub fn new(
        world: World,
        padding_scale: LinkPaddingScale,
        link_body_decompositions: LinkBodyDecompositions,
        distance_field_config: DistanceFieldConfig,
        collision_tolerance: f64,
    ) -> Self {
        Self {
            parry: ParryCollisionEnv::new(world, padding_scale),
            distance_field: DistanceFieldCollisionCache::new(
                link_body_decompositions,
                distance_field_config,
                collision_tolerance,
            ),
        }
    }

    /// Upstream's inherited (from `CollisionEnv`) `getWorld() const`.
    pub fn world(&self) -> &World {
        self.parry.world()
    }

    /// Upstream `CollisionEnvHybrid::setWorld` minus the
    /// `CollisionEnvFCL::setWorld` call: that call's entire purpose is
    /// rebuilding FCL's persistent broadphase cache
    /// (`collision_env_fcl.cpp:417-438`, `manager_`/`fcl_objs_`) and
    /// rewiring `World` observers on a world swap.
    /// [`ParryCollisionEnv`] has no such structure to invalidate -- every
    /// `check_*` call rebuilds its collision bodies fresh from `self.world`
    /// (`parry.rs:1840`, `world_bodies(&self.world, ...)`), so there is
    /// nothing here to rebuild; a caller mutates the `World` this returns
    /// (or replaces it wholesale) and the very next call already sees it.
    /// See [`Self::check_collision_distance_field`]'s doc for why the
    /// distance-field half has the identical property, and the
    /// `check_robot_collision_distance_field_reflects_a_world_swap_on_the_next_call`
    /// test in this module for the empirical check backing this claim.
    ///
    /// # §153.1: this claim expires if `ParryCollisionEnv` ever gains a
    /// persistent, world-derived cache
    ///
    /// This method's *absence* (there is no `set_world` here at all, only
    /// `world_mut` below) rests on "`ParryCollisionEnv` recomputes from
    /// `self.world` on every call" being true *today*. If a future round
    /// adds any such cache to `ParryCollisionEnv` for performance, this
    /// claim -- and the test named above -- must be revisited; grep this
    /// crate for `§153.1` before adding one.
    pub fn world_mut(&mut self) -> &mut World {
        self.parry.world_mut()
    }

    /// Upstream `getCollisionRobotDistanceField`/`getCollisionWorldDistanceField`
    /// (`collision_env_hybrid.hpp:104-107,143-146`) -- both `return cenv_distance_`,
    /// collapsed into one accessor since this port's [`DistanceFieldCollisionCache`]
    /// is the one thing both getters return.
    pub fn distance_field(&self) -> &DistanceFieldCollisionCache<'m> {
        &self.distance_field
    }

    /// `&mut` counterpart of [`Self::distance_field`]; upstream's own two
    /// getters are both `const`, but every one of
    /// [`DistanceFieldCollisionCache`]'s own check/gradient methods take
    /// `&mut self` (its cache write), so a caller driving it directly
    /// through this accessor needs `&mut` access too.
    pub fn distance_field_mut(&mut self) -> &mut DistanceFieldCollisionCache<'m> {
        &mut self.distance_field
    }

    /// Upstream `CollisionEnvHybrid::checkSelfCollisionDistanceField`'s four
    /// overloads (`collision_env_hybrid.cpp:75-105`), all `cenv_distance_->checkSelfCollision(...)`.
    /// See this module's doc comment for why this is a thin forward with no
    /// new arity-collapse decision, and
    /// [`DistanceFieldCollisionCache::check_self_collision`] for the real
    /// logic and error conditions.
    ///
    /// # Errors
    ///
    /// See [`DistanceFieldCollisionCache::check_self_collision`].
    pub fn check_self_collision_distance_field<'s>(
        &'s mut self,
        req: &CollisionRequest,
        state: &Posed<'_, 'm>,
        acm: Option<&AllowedCollisionMatrix>,
        current_attached_bodies: &[AttachedBodyGeometry<'_>],
    ) -> Result<(CollisionResult, GroupStateRepresentation<'s, 'm>)> {
        self.distance_field
            .check_self_collision(req, state, acm, current_attached_bodies)
    }
}

/// Upstream's inherited-not-overridden `checkCollision`/`checkSelfCollision`/
/// `checkRobotCollision`/`checkRobotCollision` (continuous)/`distanceSelf`/
/// `distanceRobot`: `CollisionEnvHybrid` does not declare any of these, so
/// they resolve to `CollisionEnvFCL`'s own bodies unchanged -- fully
/// FCL-backed, self-collision included. This impl is pure delegation to the
/// held [`ParryCollisionEnv`] for the same reason, preserving upstream's
/// inheritance-based substitutability (any caller holding
/// `impl CollisionEnv<State>` accepts a [`HybridCollisionEnv`] too) without
/// inheritance, which Rust does not have. `check_collision`'s default trait
/// body (calling `check_self_collision` then `check_robot_collision`) is not
/// overridden here either, for the identical reason it is not overridden by
/// upstream's `CollisionEnvFCL`.
impl<'s, 'm> CollisionEnv<Posed<'s, 'm>> for HybridCollisionEnv<'m> {
    fn check_self_collision(
        &self,
        request: &CollisionRequest,
        state: &Posed<'s, 'm>,
        attached_bodies: &[AttachedBodyGeometry<'_>],
        acm: Option<&AllowedCollisionMatrix>,
    ) -> CollisionResult {
        self.parry
            .check_self_collision(request, state, attached_bodies, acm)
    }

    fn check_robot_collision(
        &self,
        request: &CollisionRequest,
        state: &Posed<'s, 'm>,
        attached_bodies: &[AttachedBodyGeometry<'_>],
        acm: Option<&AllowedCollisionMatrix>,
    ) -> CollisionResult {
        self.parry
            .check_robot_collision(request, state, attached_bodies, acm)
    }

    fn check_robot_collision_continuous(
        &self,
        request: &CollisionRequest,
        state1: &Posed<'s, 'm>,
        state2: &Posed<'s, 'm>,
        attached_bodies: &[AttachedBodyGeometry<'_>],
        acm: Option<&AllowedCollisionMatrix>,
    ) -> Result<CollisionResult> {
        self.parry
            .check_robot_collision_continuous(request, state1, state2, attached_bodies, acm)
    }

    fn distance_self(
        &self,
        request: &DistanceRequest<'_>,
        state: &Posed<'s, 'm>,
        attached_bodies: &[AttachedBodyGeometry<'_>],
    ) -> DistanceResult {
        self.parry.distance_self(request, state, attached_bodies)
    }

    fn distance_robot(
        &self,
        request: &DistanceRequest<'_>,
        state: &Posed<'s, 'm>,
        attached_bodies: &[AttachedBodyGeometry<'_>],
    ) -> DistanceResult {
        self.parry.distance_robot(request, state, attached_bodies)
    }
}
