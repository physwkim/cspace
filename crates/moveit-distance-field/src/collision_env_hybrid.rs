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
use crate::{
    DistanceField, DistanceFieldCollisionCache, DistanceFieldConfig, GroupStateRepresentation,
    PropagationDistanceField, collision_object_point_decomposition,
};

/// Upstream `CollisionEnvHybrid`. See this module's doc comment for the
/// public shape and the §186 measurement that unblocked porting it.
pub struct HybridCollisionEnv<'m> {
    parry: ParryCollisionEnv,
    distance_field: DistanceFieldCollisionCache<'m>,
    /// Kept alongside `distance_field` (whose own copy is private to
    /// `collision_env_distance_field`) so `build_env_distance_field`
    /// can rebuild an environment field with the same geometry/propagation
    /// settings [`Self::distance_field`]'s own self-collision field uses --
    /// matching upstream, whose single `CollisionEnvDistanceField`
    /// constructor argument set builds both fields.
    distance_field_config: DistanceFieldConfig,
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
            distance_field_config,
        }
    }

    /// Upstream's inherited (from `CollisionEnv`) `getWorld() const`.
    pub fn world(&self) -> &World {
        self.parry.world()
    }

    /// Upstream `CollisionEnvHybrid::setWorld` (`collision_env_hybrid.cpp:163-170`)
    /// does two things: `cenv_distance_->setWorld(world)`, then
    /// `CollisionEnvFCL::setWorld(world)`. Both calls exist because upstream
    /// stores the world *twice* -- once as the `WorldPtr` `CollisionEnvFCL`
    /// (via the `CollisionEnv` base) rebuilds its FCL broadphase cache from,
    /// once as the `WorldPtr` `cenv_distance_` derives
    /// `distance_field_cache_entry_world_` from -- and nothing but that one
    /// override keeps the two in step. Anyone who reaches
    /// `CollisionEnvFCL::setWorld` directly on a `CollisionEnvHybrid` --
    /// through a `CollisionEnvFCL&`/`CollisionEnv&` reference, which C++
    /// permits since `setWorld` is a plain (non-`final`) virtual override,
    /// not a sealed one -- updates the FCL half's world and leaves
    /// `cenv_distance_`'s stale: the two halves would then disagree about
    /// what "the world" even is, not just about whether something in it
    /// collides.
    ///
    /// # §196.3: why this type has no `set_world` at all, not just a safe one
    ///
    /// [`HybridCollisionEnv`] holds exactly *one* `World` value -- inside
    /// `self.parry` (see this type's own field list). `self.distance_field`
    /// (a [`DistanceFieldCollisionCache`]) holds no `World` reference of its
    /// own; `build_env_distance_field` derives its environment field by
    /// reading `self.parry.world()` fresh on every call instead. So "the
    /// two halves disagree about what the world is" is not a state this
    /// type can reach -- not because a second world is kept carefully in
    /// sync by some guard, but because there is only ever one `World` value
    /// to begin with. `world_mut` below is the *only* way in or out for it,
    /// for both halves at once by construction; there is no second entry
    /// point for a caller to update selectively and no invariant to state,
    /// because there is nothing left for two updates to disagree about.
    ///
    /// The remaining question upstream's override also answers -- does a
    /// world swap actually take effect on the *next* call, or does some
    /// cache need invalidating first -- is a separate, narrower claim, since
    /// having one `World` value doesn't by itself guarantee every reader of
    /// it is cache-free. [`ParryCollisionEnv`] has no persistent structure
    /// to invalidate: every `check_*` call rebuilds its collision bodies
    /// fresh from `self.world` (`parry.rs:1884`,
    /// `world_bodies(&self.world, ...)`), and `build_env_distance_field`
    /// does the same for the distance-field half (see that method's own
    /// doc). See the
    /// `check_robot_collision_distance_field_reflects_a_world_swap_on_the_next_call`
    /// test in this module for the empirical check backing that narrower
    /// claim, including its own §153.1 expiry note -- that expiry is about
    /// *this* claim (no cache to invalidate), not the structural one above,
    /// which holds regardless of whether either backend ever grows a cache.
    ///
    /// # Proof: the reverse direction is a compile error, not a discipline
    ///
    /// The remaining question is the mirror of the one above: could
    /// [`Self::world_mut`] be called *while* a distance-field check's result
    /// is still alive and being read, mutating the world out from under it?
    /// No -- and provably so, not by convention: every
    /// `check_*_distance_field` method takes `&'s mut self` and returns a
    /// value borrowing that same `'s` ([`GroupStateRepresentation`]`<'s,
    /// 'm>`, via its `dfce: &'a DistanceFieldCacheEntry<'m>` field), so the
    /// exclusive borrow used to produce the result stays alive for as long
    /// as the result itself is in scope. A second `&mut self` call -- which
    /// [`Self::world_mut`] requires -- cannot coexist with that borrow; the
    /// borrow checker rejects it before the question of *correctness* even
    /// arises. This compiles:
    ///
    /// ```no_run
    /// # use moveit_collision::{CollisionRequest, LinkPaddingScale, World};
    /// # use moveit_distance_field::{DistanceFieldConfig, GridGeometry, HybridCollisionEnv, add_link_body_decompositions};
    /// # use moveit_model::{MeshSearchPaths, RobotModel};
    /// # use nalgebra::Vector3;
    /// # let urdf: urdf_rs::Robot =
    /// #     urdf_rs::read_from_string(r#"<robot name="r"><link name="l"/></robot>"#).unwrap();
    /// # let srdf = moveit_srdf::SrdfModel::parse_str(r#"<robot name="r"/>"#).unwrap();
    /// # let model =
    /// #     RobotModel::from_urdf_and_srdf(&urdf, "", &srdf, &MeshSearchPaths::none()).unwrap();
    /// # let padding = LinkPaddingScale::new();
    /// # let decompositions = add_link_body_decompositions(&model, 0.02, &padding, None).unwrap();
    /// # let size = Vector3::new(1.0, 1.0, 1.0);
    /// # let config = DistanceFieldConfig {
    /// #     geometry: GridGeometry::new(size, -0.5 * size, 0.05).unwrap(),
    /// #     max_propagation_distance: 0.5,
    /// #     use_signed_distance_field: false,
    /// # };
    /// # let mut env = HybridCollisionEnv::new(World::new(), padding, decompositions, config, 0.0);
    /// # let mut state = moveit_state::RobotState::new(&model);
    /// # state.set_to_default_values();
    /// # let posed = state.update();
    /// {
    ///     let (result, gsr) = env
    ///         .check_self_collision_distance_field(&CollisionRequest::default(), &posed, None, &[])
    ///         .unwrap();
    ///     let _ = (result, gsr); // both dropped at the end of this block
    /// }
    /// env.world_mut(); // fine: no live borrow from the check above remains
    /// ```
    ///
    /// This does not:
    ///
    /// ```compile_fail
    /// # use moveit_collision::{CollisionRequest, LinkPaddingScale, World};
    /// # use moveit_distance_field::{DistanceFieldConfig, GridGeometry, HybridCollisionEnv, add_link_body_decompositions};
    /// # use moveit_model::{MeshSearchPaths, RobotModel};
    /// # use nalgebra::Vector3;
    /// # let urdf: urdf_rs::Robot =
    /// #     urdf_rs::read_from_string(r#"<robot name="r"><link name="l"/></robot>"#).unwrap();
    /// # let srdf = moveit_srdf::SrdfModel::parse_str(r#"<robot name="r"/>"#).unwrap();
    /// # let model =
    /// #     RobotModel::from_urdf_and_srdf(&urdf, "", &srdf, &MeshSearchPaths::none()).unwrap();
    /// # let padding = LinkPaddingScale::new();
    /// # let decompositions = add_link_body_decompositions(&model, 0.02, &padding, None).unwrap();
    /// # let size = Vector3::new(1.0, 1.0, 1.0);
    /// # let config = DistanceFieldConfig {
    /// #     geometry: GridGeometry::new(size, -0.5 * size, 0.05).unwrap(),
    /// #     max_propagation_distance: 0.5,
    /// #     use_signed_distance_field: false,
    /// # };
    /// # let mut env = HybridCollisionEnv::new(World::new(), padding, decompositions, config, 0.0);
    /// # let mut state = moveit_state::RobotState::new(&model);
    /// # state.set_to_default_values();
    /// # let posed = state.update();
    /// let (result, gsr) = env
    ///     .check_self_collision_distance_field(&CollisionRequest::default(), &posed, None, &[])
    ///     .unwrap();
    /// env.world_mut(); // `gsr` below is still live -- must not compile
    /// let _ = (result, gsr);
    /// ```
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

    /// Builds a fresh environment [`PropagationDistanceField`] from every
    /// [`moveit_collision::Object`] currently in [`Self::world`], at the
    /// same geometry/propagation settings [`Self::distance_field`]'s own
    /// self-collision field uses -- matching upstream's single
    /// `CollisionEnvDistanceField` constructor argument set building both
    /// fields (`collision_env_hybrid.cpp:50-52`).
    ///
    /// Upstream's `cenv_distance_` maintains this same field incrementally,
    /// updated on every `World` change via `distance_field_cache_entry_world_`/
    /// `generateDistanceFieldCacheEntryWorld`/`updateDistanceObject`
    /// (`collision_env_distance_field.hpp:59-309`) -- machinery this port
    /// cannot replicate because [`moveit_collision::World`] deliberately has
    /// no observer/notify mechanism for a crate outside `moveit-collision`
    /// to hook (that type's own module doc, deviation list). Rebuilding
    /// fresh on every call instead matches [`ParryCollisionEnv`]'s own
    /// design, which recomputes its collision bodies fresh from `self.world`
    /// every `check_*` call with no persistent broadphase cache
    /// (`parry.rs:1884`, `world_bodies(&self.world, ...)`), and this crate's
    /// own established "recompute over cache-and-invalidate" precedent (see
    /// this module's doc comment). See
    /// `check_robot_collision_distance_field_reflects_a_world_swap_on_the_next_call`
    /// below for the empirical check that this actually stays in sync with
    /// [`Self::world_mut`].
    ///
    /// # Errors
    ///
    /// Returns an error if the configured grid geometry is invalid (see
    /// [`PropagationDistanceField::new`]) or if any world object's shape
    /// cannot be decomposed into collision points (see
    /// [`collision_object_point_decomposition`]).
    fn build_env_distance_field(&self) -> Result<PropagationDistanceField> {
        let mut field = PropagationDistanceField::new(
            self.distance_field_config.geometry,
            self.distance_field_config.max_propagation_distance,
            self.distance_field_config.use_signed_distance_field,
        )?;
        let resolution = self.distance_field_config.geometry.resolution;
        let mut points = Vec::new();
        for (_, object) in self.parry.world().iter() {
            let decomposition = collision_object_point_decomposition(object, resolution)?;
            points.extend(decomposition.collision_points());
        }
        field.add_points_to_field(&points);
        Ok(field)
    }

    /// Upstream `CollisionEnvHybrid::checkCollisionDistanceField`'s four
    /// overloads (`collision_env_hybrid.cpp:107-133`), all
    /// `cenv_distance_->checkCollision(...)`. See
    /// `build_env_distance_field` for why the environment field
    /// argument is built fresh here rather than read off a cache, and this
    /// module's doc comment for why that is not a new arity-collapse
    /// decision on top of [`DistanceFieldCollisionCache::check_collision`].
    ///
    /// # Errors
    ///
    /// See `build_env_distance_field` and
    /// [`DistanceFieldCollisionCache::check_collision`].
    pub fn check_collision_distance_field<'s>(
        &'s mut self,
        req: &CollisionRequest,
        state: &Posed<'_, 'm>,
        acm: Option<&AllowedCollisionMatrix>,
        current_attached_bodies: &[AttachedBodyGeometry<'_>],
    ) -> Result<(CollisionResult, GroupStateRepresentation<'s, 'm>)> {
        let env_distance_field = self.build_env_distance_field()?;
        self.distance_field.check_collision(
            req,
            state,
            acm,
            current_attached_bodies,
            &env_distance_field,
        )
    }

    /// Upstream `CollisionEnvHybrid::checkRobotCollisionDistanceField`'s
    /// four overloads (`collision_env_hybrid.cpp:135-161`), all
    /// `cenv_distance_->checkRobotCollision(...)`. See
    /// [`Self::check_collision_distance_field`]'s doc for the environment
    /// field rationale.
    ///
    /// # Errors
    ///
    /// See `build_env_distance_field` and
    /// [`DistanceFieldCollisionCache::check_robot_collision`].
    pub fn check_robot_collision_distance_field<'s>(
        &'s mut self,
        req: &CollisionRequest,
        state: &Posed<'_, 'm>,
        acm: Option<&AllowedCollisionMatrix>,
        current_attached_bodies: &[AttachedBodyGeometry<'_>],
    ) -> Result<(CollisionResult, GroupStateRepresentation<'s, 'm>)> {
        let env_distance_field = self.build_env_distance_field()?;
        self.distance_field.check_robot_collision(
            req,
            state,
            acm,
            current_attached_bodies,
            &env_distance_field,
        )
    }

    /// Upstream `CollisionEnvHybrid::getCollisionGradients`
    /// (`collision_env_hybrid.cpp:172-177`), `cenv_distance_->getCollisionGradients(...)`.
    /// See [`Self::check_collision_distance_field`]'s doc for the
    /// environment field rationale.
    ///
    /// # Errors
    ///
    /// See `build_env_distance_field` and
    /// [`DistanceFieldCollisionCache::get_collision_gradients`].
    pub fn get_collision_gradients<'s>(
        &'s mut self,
        req: &CollisionRequest,
        state: &Posed<'_, 'm>,
        acm: Option<&AllowedCollisionMatrix>,
        current_attached_bodies: &[AttachedBodyGeometry<'_>],
    ) -> Result<GroupStateRepresentation<'s, 'm>> {
        let env_distance_field = self.build_env_distance_field()?;
        self.distance_field.get_collision_gradients(
            req,
            state,
            acm,
            current_attached_bodies,
            &env_distance_field,
        )
    }

    /// Upstream `CollisionEnvHybrid::getAllCollisions`
    /// (`collision_env_hybrid.cpp:179-184`), `cenv_distance_->getAllCollisions(...)`.
    /// See [`Self::check_collision_distance_field`]'s doc for the
    /// environment field rationale.
    ///
    /// # Errors
    ///
    /// See `build_env_distance_field` and
    /// [`DistanceFieldCollisionCache::get_all_collisions`].
    pub fn get_all_collisions<'s>(
        &'s mut self,
        req: &CollisionRequest,
        state: &Posed<'_, 'm>,
        acm: Option<&AllowedCollisionMatrix>,
        current_attached_bodies: &[AttachedBodyGeometry<'_>],
    ) -> Result<(CollisionResult, GroupStateRepresentation<'s, 'm>)> {
        let env_distance_field = self.build_env_distance_field()?;
        self.distance_field.get_all_collisions(
            req,
            state,
            acm,
            current_attached_bodies,
            &env_distance_field,
        )
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

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use moveit_geometry::{Cuboid, Isometry3, Shape};
    use moveit_model::{MeshSearchPaths, RobotModel};
    use nalgebra::Vector3;

    use super::*;
    use crate::{GridGeometry, add_link_body_decompositions};

    /// A two-link robot, `mid` and `tip`, both 0.1 cubes, joined by a
    /// revolute joint whose origin places `tip`'s box `gap` away from
    /// `mid`'s along x (`gap` is the real, exact geometric separation
    /// between the two box surfaces). A fixed joint would express the same
    /// pose but produces an empty `"chain"` group -- `RobotModel`'s group
    /// resolution walks active joints, and a `fixed` one has none to walk
    /// (measured: switching this joint's `type` from `revolute` to `fixed`
    /// collapses `DistanceFieldCacheEntry::link_names` to `[]` and both
    /// tests below started passing vacuously, before the
    /// `moveit_test_support::assert_group_has_updated_links` call just below
    /// existed to catch it at construction time instead, §196). The joint's
    /// one variable defaults to 0, under which rotation about its own z axis
    /// is the identity, so [`moveit_state::RobotState::set_to_default_values`]
    /// alone still fully determines the state -- this is a fixed pose, not
    /// a swept one.
    fn two_link_gap_model() -> (RobotModel, f64) {
        const GAP: f64 = 0.05;
        let urdf_xml = format!(
            r#"<?xml version="1.0"?>
<robot name="two_link_gap">
  <link name="mid">
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <link name="tip">
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <joint name="mid_to_tip" type="revolute">
    <parent link="mid"/>
    <child link="tip"/>
    <origin xyz="{offset} 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"#,
            offset = 0.1 + GAP
        );
        let srdf_xml = r#"<?xml version="1.0"?>
<robot name="two_link_gap">
  <group name="chain">
    <chain base_link="mid" tip_link="tip"/>
  </group>
</robot>
"#;
        let urdf: urdf_rs::Robot = urdf_rs::read_from_string(&urdf_xml).unwrap();
        let srdf = moveit_srdf::SrdfModel::parse_str(srdf_xml).expect("srdf must parse");
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("two_link_gap model must build");
        moveit_test_support::assert_group_has_updated_links(&model, "chain");
        (model, GAP)
    }

    fn test_distance_field_config() -> DistanceFieldConfig {
        let size = Vector3::new(2.0, 2.0, 2.0);
        DistanceFieldConfig {
            geometry: GridGeometry::new(size, -0.5 * size, 0.05).unwrap(),
            max_propagation_distance: 0.6,
            use_signed_distance_field: false,
        }
    }

    /// Both halves see the *same* state -- `mid`/`tip` at their joint-origin
    /// poses (the one revolute variable between them defaults to 0, so this
    /// is a fixed pose, not a swept one), no attached bodies, the same
    /// `req.group_name` -- so any difference in verdict comes only from how
    /// each backend decides "colliding," not from checking different
    /// things.
    ///
    /// `mid` and `tip` have a real, exact 0.05 m gap (see
    /// [`two_link_gap_model`]): [`ParryCollisionEnv`]'s self-check does
    /// exact cuboid-cuboid intersection, so it must report clear.
    /// [`DistanceFieldCollisionCache::check_self_collision`]'s underlying
    /// [`get_collision_sphere_collisions`](crate::collision_distance_field_types::get_collision_sphere_collisions)
    /// flags a sphere as colliding when `sphere.radius - result.distance >
    /// collision_tolerance`; a *negative* `collision_tolerance` therefore
    /// flags a near miss up to `|collision_tolerance|` away as a collision
    /// too (measured empirically here, not assumed: `tip`'s one bounding
    /// sphere -- radius 0.0707, a resolution-0.02 decomposition of its 0.1 m
    /// box collapses to a single circumscribing sphere -- sits
    /// `0.0707 - 0.1 = -0.0293` short of `mid`'s surface). Built here with
    /// `collision_tolerance = -0.1`, comfortably past that -0.0293 margin,
    /// the distance-field self-check must report a collision for the
    /// identical geometry parry reports clear on. A combinator that
    /// silently forwarded one backend's self-collision answer as "the"
    /// answer, or that only ever tested states where both backends agree,
    /// would not catch a bug that broke this divergence.
    #[test]
    fn self_check_and_world_check_disagree_about_a_near_miss_within_tolerance() {
        let (model, gap) = two_link_gap_model();
        assert!(gap > 0.0, "the gap must be a real geometric separation");
        let padding = LinkPaddingScale::new();
        let link_body_decompositions =
            add_link_body_decompositions(&model, 0.02, &padding, None).unwrap();
        let mut env = HybridCollisionEnv::new(
            World::new(),
            padding,
            link_body_decompositions,
            test_distance_field_config(),
            -0.1,
        );
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let parry_result =
            env.check_self_collision(&CollisionRequest::default(), &posed, &[], None);
        assert!(
            !parry_result.collision,
            "mid and tip have a real 0.05 m gap; parry's exact-geometry self-check must \
             report no collision"
        );

        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            ..CollisionRequest::default()
        };
        let (df_result, _gsr) = env
            .check_self_collision_distance_field(&req, &posed, None, &[])
            .unwrap();
        assert!(
            df_result.collision,
            "collision_tolerance -0.1 pads past the ~0.0293 m margin between tip's bounding \
             sphere and mid's surface (see this test's doc comment for the measurement); the \
             distance-field self-check must flag it, proving the two backends genuinely \
             disagree about the identical state rather than one silently mirroring the other"
        );
    }

    /// # §153.1: this test's premise expires if `ParryCollisionEnv` ever
    /// gains a persistent, world-derived cache
    ///
    /// [`HybridCollisionEnv::world_mut`] has no override that rebuilds
    /// anything, on the measured claim that neither backend keeps a
    /// world-derived cache to invalidate: [`ParryCollisionEnv`] recomputes
    /// its collision bodies from `self.world` on every call
    /// (`parry.rs:1884`), and [`HybridCollisionEnv::build_env_distance_field`]
    /// does the same for the distance-field half. This test is the
    /// empirical check backing that claim for the distance-field half: if a
    /// future round adds a cache to either backend for performance, this
    /// test is what must start failing -- grep this crate for `§153.1`
    /// before adding one, and update this test's premise rather than
    /// patching around a new failure here.
    ///
    /// # Mutation-tested, not merely asserted
    ///
    /// This test's `res_after.collision` assertion is the discriminator for
    /// the "stale second world" bug upstream's `setWorld` override exists to
    /// prevent (see [`HybridCollisionEnv::world_mut`]'s §196.3 doc): if
    /// `build_env_distance_field` read from anything other than
    /// `self.parry.world()` fresh -- a snapshot taken at construction, a
    /// cached copy never updated by `world_mut` -- the environment field
    /// would never see the `add_shape` below, and `res_after.collision`
    /// would stay `false`. Confirmed by temporarily changing
    /// `build_env_distance_field`'s source loop from `self.parry.world()` to
    /// a fresh, permanently-empty `World::new()` (the shape a stale second
    /// world would take, since it would never observe the swap either) and
    /// re-running: this test failed, at exactly this assertion, with the
    /// expected message. Reverted before commit; `git diff` on that revert
    /// showed no residual change.
    #[test]
    fn check_robot_collision_distance_field_reflects_a_world_swap_on_the_next_call() {
        let (model, _gap) = two_link_gap_model();
        let padding = LinkPaddingScale::new();
        let link_body_decompositions =
            add_link_body_decompositions(&model, 0.02, &padding, None).unwrap();
        let mut env = HybridCollisionEnv::new(
            World::new(),
            padding,
            link_body_decompositions,
            test_distance_field_config(),
            0.0,
        );
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            ..CollisionRequest::default()
        };

        let (res_before, _gsr) = env
            .check_robot_collision_distance_field(&req, &posed, None, &[])
            .unwrap();
        assert!(
            !res_before.collision,
            "an empty World has no environment points to collide with"
        );

        env.world_mut().add_shape(
            "obstacle",
            Arc::new(Shape::Cuboid(Cuboid::new(0.3, 0.3, 0.3).unwrap())),
            Isometry3::identity(),
        );

        let (res_after, _gsr2) = env
            .check_robot_collision_distance_field(&req, &posed, None, &[])
            .unwrap();
        assert!(
            res_after.collision,
            "the very next check_robot_collision_distance_field call after world_mut() must \
             see the swap: a 0.3 m cube at the origin overlaps mid's own 0.1 m box there, and \
             build_env_distance_field rebuilds from the current World on every call, so there \
             is no stale cache to explain a miss here"
        );
    }
}
