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

use std::collections::{HashMap, HashSet};

use moveit_collision::{
    Action, AllowedCollisionMatrix, AttachedBodyGeometry, CollisionEnv, CollisionRequest,
    CollisionResult, DistanceRequest, DistanceResult, LinkPaddingScale, MoveObjectOutcome,
    Notification, ParryCollisionEnv, World,
};
use moveit_error::{Error, Result};
use moveit_state::Posed;
use nalgebra::Vector3;

use crate::collision_env_distance_field::LinkBodyDecompositions;
use crate::{
    DistanceField, DistanceFieldCollisionCache, DistanceFieldConfig, GroupStateRepresentation,
    PropagationDistanceField, collision_object_point_decomposition,
};

/// What [`HybridCollisionEnv::mutate_world`] can turn a `World` mutator's
/// return value into: zero or more [`Notification`]s to apply to
/// `env_field`. Every `World` mutator returns one of the four shapes
/// implemented below; this trait is the uniform interface
/// [`HybridCollisionEnv::mutate_world`] needs to stay generic over all of
/// them without a bespoke wrapper method per `World` mutator.
///
/// # `()` is deliberately not implemented — do not add it
///
/// It was, and it silently reopened the exact staleness this type exists to
/// prevent. `mutate_world` keeps `env_field` in step with the world by
/// applying whatever the closure *returns*, so a closure that mutates and
/// then discards its own [`Notification`] leaves the field describing a
/// world that no longer exists — and with `()` implemented as "zero
/// notifications", that closure compiled and reported success. The trigger
/// is a single semicolon: `|w| w.add_shape(..)` returns the notification,
/// `|w| { w.add_shape(..); }` returns `()`. Measured, not argued —
/// adding that one semicolon to this module's own
/// `check_robot_collision_distance_field_reflects_a_world_swap_on_the_next_call`
/// test compiled fine and reddened it, the field never having learned about
/// the obstacle.
///
/// Every `World` mutator returns something notification-carrying
/// ([`Notification`], `Option<Notification>`, `Vec<Notification>`,
/// [`MoveObjectOutcome`]), so there is no legitimate mutating closure that
/// needs `()`, and a closure that only *reads* should use
/// [`HybridCollisionEnv::world`] instead. Leaving `()` unimplemented turns
/// the discard from a silent desync into a compile error, which is the only
/// version of this invariant that does not depend on the caller
/// remembering.
pub trait AsNotifications {
    /// The notifications this value describes, borrowing rather than
    /// consuming so the caller of `mutate_world` still gets the original
    /// value back.
    fn as_notifications(&self) -> Vec<Notification>;
}

impl AsNotifications for Notification {
    fn as_notifications(&self) -> Vec<Notification> {
        vec![self.clone()]
    }
}

impl AsNotifications for Option<Notification> {
    fn as_notifications(&self) -> Vec<Notification> {
        self.iter().cloned().collect()
    }
}

impl AsNotifications for Vec<Notification> {
    fn as_notifications(&self) -> Vec<Notification> {
        self.clone()
    }
}

impl AsNotifications for MoveObjectOutcome {
    fn as_notifications(&self) -> Vec<Notification> {
        match self {
            MoveObjectOutcome::Moved(notification) => vec![notification.clone()],
            MoveObjectOutcome::NotFound | MoveObjectOutcome::NoChange => Vec::new(),
        }
    }
}

/// Upstream `CollisionEnvHybrid`. See this module's doc comment for the
/// public shape and the §186 measurement that unblocked porting it.
pub struct HybridCollisionEnv<'m> {
    parry: ParryCollisionEnv,
    distance_field: DistanceFieldCollisionCache<'m>,
    /// Kept alongside `distance_field` (whose own copy is private to
    /// `collision_env_distance_field`) so [`Self::apply_notification`] can
    /// build/extend `env_field` with the same geometry/propagation settings
    /// [`Self::distance_field`]'s own self-collision field uses -- matching
    /// upstream, whose single `CollisionEnvDistanceField` constructor
    /// argument set builds both fields.
    distance_field_config: DistanceFieldConfig,
    /// The environment [`PropagationDistanceField`], maintained
    /// incrementally. Upstream `cenv_distance_->distance_field_cache_entry_world_->distance_field_`.
    /// See [`Self::mutate_world`] for how this is kept in step with
    /// [`Self::world`].
    env_field: PropagationDistanceField,
    /// The points last synced into `env_field` for each object id, keyed by
    /// [`moveit_collision::World`] object id. Upstream
    /// `posed_body_point_decompositions_`, minus the intermediate
    /// `PosedBodyPointDecomposition` wrapper -- this port only ever needs
    /// the flat point list back out of it, never the wrapper's own methods.
    env_field_points: HashMap<String, Vec<Vector3<f64>>>,
    /// Object ids whose `env_field_points` entry does not reflect the
    /// object's current shapes, because
    /// [`collision_object_point_decomposition`] errored the last time
    /// [`Self::apply_notification`] tried to resync them. Upstream's
    /// `updateDistanceObject` has no equivalent: its point decomposition is
    /// infallible C++, so upstream never has anything to desync. This port's
    /// decomposition is fallible ([`collision_object_point_decomposition`]
    /// returns [`Result`]), and a mutation that already landed in
    /// [`Self::world`] cannot be undone just because its field-sync failed --
    /// so the id is recorded here instead of silently leaving `env_field`
    /// wrong. See [`Self::mutate_world`], the write-side half of the same
    /// invariant; the read side is the private sync check every
    /// `*_distance_field` method runs before touching `env_field`.
    ///
    /// # Deviation from upstream
    ///
    /// Every `check_*_distance_field`/`get_collision_gradients`/
    /// `get_all_collisions` method below returns [`Error::construct`] while
    /// this set is non-empty, rather than upstream's silent "the field is
    /// whatever the last successful sync left it as." This is the illegal
    /// state -- a field inconsistent with its world -- described in this
    /// crate's own house rules: erroring here makes it unrepresentable to a
    /// caller, rather than trusting every caller of [`Self::mutate_world`] to
    /// notice and handle a swallowed decomposition failure itself.
    desynced_objects: HashSet<String>,
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
    ///
    /// # Errors
    ///
    /// Returns an error if the configured grid geometry is invalid (see
    /// [`PropagationDistanceField::new`]) or if any object already in
    /// `world` cannot be decomposed into collision points -- unlike a later
    /// [`Self::mutate_world`] call hitting the same problem, a decomposition
    /// failure at construction time has no already-landed mutation to
    /// reconcile against, so it is reported here as a hard error rather than
    /// recorded in `desynced_objects`.
    pub fn new(
        world: World,
        padding_scale: LinkPaddingScale,
        link_body_decompositions: LinkBodyDecompositions,
        distance_field_config: DistanceFieldConfig,
        collision_tolerance: f64,
    ) -> Result<Self> {
        let mut env_field = PropagationDistanceField::new(
            distance_field_config.geometry,
            distance_field_config.max_propagation_distance,
            distance_field_config.use_signed_distance_field,
        )?;
        let resolution = distance_field_config.geometry.resolution;
        let mut env_field_points = HashMap::new();
        for (id, object) in world.iter() {
            let points =
                collision_object_point_decomposition(object, resolution)?.collision_points();
            env_field.add_points_to_field(&points);
            env_field_points.insert(id.clone(), points);
        }

        Ok(Self {
            parry: ParryCollisionEnv::new(world, padding_scale),
            distance_field: DistanceFieldCollisionCache::new(
                link_body_decompositions,
                distance_field_config,
                collision_tolerance,
            ),
            distance_field_config,
            env_field,
            env_field_points,
            desynced_objects: HashSet::new(),
        })
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
    /// # §196.3/§230: why this type has no `set_world` at all, not just a safe one
    ///
    /// [`HybridCollisionEnv`] holds exactly *one* `World` value -- inside
    /// `self.parry` (see this type's own field list). `self.distance_field`
    /// (a [`DistanceFieldCollisionCache`]) holds no `World` reference of its
    /// own; `self.env_field` is not a second `World`, it is a derived
    /// structure kept in step with the one `World` value by
    /// [`Self::mutate_world`] below. So "the two halves disagree about what
    /// the world is" is not a state this type can reach -- not because a
    /// second world is kept carefully in sync by some guard, but because
    /// there is only ever one `World` value to begin with. `mutate_world` is
    /// the *only* way in or out for it, for both halves at once by
    /// construction; there is no second entry point for a caller to update
    /// selectively and no invariant to state, because there is nothing left
    /// for two updates to disagree about.
    ///
    /// # §230: replaces upstream's `addObserver`, matches upstream's incrementality
    ///
    /// A previous round of this doc argued `env_field`'s predecessor
    /// (`build_env_distance_field`) had to rebuild from scratch every call
    /// because [`moveit_collision::World`] "deliberately has no
    /// observer/notify mechanism." That argument was measured false: `World`
    /// genuinely has no *callback* registration (that type's own module doc,
    /// deviation 4), but every mutator returns the
    /// [`moveit_collision::Notification`] describing what changed instead of
    /// pushing it to a registered observer, and
    /// [`moveit_collision::World::all_objects_as_notifications`] exists
    /// specifically to replay every current object to a newly-attached
    /// consumer -- the return-value equivalent of upstream's
    /// `notifyObserverAllObjects`. [`Self::mutate_world`] is that consumer:
    /// every mutator call it forwards feeds its `Notification`(s) straight
    /// into the private notification-applying helper, which applies the same
    /// `CREATE`/`ADD_SHAPE` (add only) vs. `MOVE_SHAPE`/`REMOVE_SHAPE`
    /// (remove old points, add current points) vs. `DESTROY` (remove only)
    /// branching upstream's own `notifyObjectChange`
    /// (`collision_env_distance_field.cpp:1704-1728`) uses. `env_field` is
    /// therefore maintained incrementally, matching upstream's own design,
    /// not rebuilt per call.
    ///
    /// # §230: not `OctreeCache`'s pattern, and concretely why not
    ///
    /// [`ParryCollisionEnv::world_mut`] safely returns a raw `&mut World`
    /// because [`ParryCollisionEnv`] keeps no persistent world-derived
    /// structure to invalidate -- every `check_*` call rebuilds its
    /// collision bodies fresh (`parry.rs:1902`,
    /// `world_bodies(&self.world, ...)`). Its `OctreeCache`
    /// (`parry.rs:1125-1236`) is a pure per-key memoization table -- one
    /// independent shape conversion (octree leaves -> parry `Compound`) per
    /// octree `Arc`, pruned by `Weak::strong_count() == 0` -- with no
    /// ordering or accumulation dependency between entries, so
    /// prune-and-recompute is sufficient on its own with no mediation of
    /// `World` mutation needed at all. `env_field` cannot use that pattern
    /// unchanged: a [`PropagationDistanceField`] is one shared aggregate
    /// whose cell distances depend jointly on every obstacle point
    /// currently in it via the propagation sweep, so there is no
    /// independent per-object fragment to memoize and no cheap way to union
    /// fragments after the fact -- removing object A's points must not
    /// disturb object B's, which only holds if removal and addition are
    /// applied to the *one* shared field precisely, not recomputed from a
    /// per-object cache. That is what the notification-applying helper does
    /// with [`PropagationDistanceField::add_points_to_field`]/
    /// [`PropagationDistanceField::remove_points_from_field`] instead --
    /// `env_field_points` is the per-object *tracking* map `OctreeCache`
    /// suggested (Arc/id identity as the signal for "this object changed"),
    /// applied to accumulate-into/retract-from one shared structure rather
    /// than to memoize independent ones.
    ///
    /// Note this per-object remove-then-add scheme has the same limit
    /// upstream's own `notifyObjectChange` has: [`PropagationDistanceField`]
    /// is a binary occupancy grid with no per-cell reference count
    /// (`propagation.rs`, `remove_obstacle_voxels`), so if two objects
    /// decompose to a point in the same voxel and one is later removed, that
    /// voxel is marked empty even though the other object still occupies
    /// it. This is not a new gap this port introduces -- upstream's
    /// `PropagationDistanceField` has the identical single flat grid with no
    /// occupancy count, and `updateDistanceObject` performs the identical
    /// per-object remove/add without checking for a shared voxel either.
    ///
    /// # Proof: the reverse direction is a compile error, not a discipline
    ///
    /// Could [`Self::mutate_world`] be called *while* a distance-field
    /// check's result is still alive and being read, mutating the world out
    /// from under it? No -- and provably so, not by convention: every
    /// `check_*_distance_field` method takes `&'s mut self` and returns a
    /// value borrowing that same `'s` ([`GroupStateRepresentation`]`<'s,
    /// 'm>`, via its `dfce: &'a DistanceFieldCacheEntry<'m>` field), so the
    /// exclusive borrow used to produce the result stays alive for as long
    /// as the result itself is in scope. A second `&mut self` call -- which
    /// [`Self::mutate_world`] requires -- cannot coexist with that borrow;
    /// the borrow checker rejects it before the question of *correctness*
    /// even arises. This compiles:
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
    /// # let mut env = HybridCollisionEnv::new(World::new(), padding, decompositions, config, 0.0).unwrap();
    /// # let mut state = moveit_state::RobotState::new(&model);
    /// # state.set_to_default_values();
    /// # let posed = state.update();
    /// {
    ///     let (result, gsr) = env
    ///         .check_self_collision_distance_field(&CollisionRequest::default(), &posed, None, &[])
    ///         .unwrap();
    ///     let _ = (result, gsr); // both dropped at the end of this block
    /// }
    /// // `clear_objects` rather than a no-op closure: `mutate_world` bounds its
    /// // closure's return type by `AsNotifications`, which `()` deliberately does
    /// // not implement (see that trait's doc), so a no-op closure would fail to
    /// // compile for a reason that has nothing to do with the borrow shown here.
    /// env.mutate_world(|w| w.clear_objects()).unwrap(); // fine: no live borrow remains
    /// ```
    ///
    /// This does not:
    ///
    /// ```compile_fail,E0499
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
    /// # let mut env = HybridCollisionEnv::new(World::new(), padding, decompositions, config, 0.0).unwrap();
    /// # let mut state = moveit_state::RobotState::new(&model);
    /// # state.set_to_default_values();
    /// # let posed = state.update();
    /// let (result, gsr) = env
    ///     .check_self_collision_distance_field(&CollisionRequest::default(), &posed, None, &[])
    ///     .unwrap();
    /// env.mutate_world(|w| w.clear_objects()).unwrap(); // `gsr` still live -- must not compile
    /// let _ = (result, gsr);
    /// ```
    ///
    /// # Errors
    ///
    /// Returns an error if `f` mutates or creates an object whose current
    /// shapes cannot be decomposed into collision points -- the world
    /// mutation itself always lands (there is no way to "undo" it once `f`
    /// has run), but the id is recorded internally and every subsequent
    /// `*_distance_field` / `get_collision_gradients` / `get_all_collisions`
    /// call on this env then fails naming it, rather than silently reading a
    /// field that no longer reflects the world.
    pub fn mutate_world<F, R>(&mut self, f: F) -> Result<R>
    where
        F: FnOnce(&mut World) -> R,
        R: AsNotifications,
    {
        let result = f(self.parry.world_mut());
        for notification in result.as_notifications() {
            self.apply_notification(&notification);
        }
        // `.min()`, not `.iter().next()`: `desynced_objects` is a `HashSet`,
        // whose iteration order depends on process-lifetime hash-seed
        // randomization, not insertion order -- `.next()` named a different
        // object across otherwise-identical runs whenever two or more were
        // desynced. `check_env_field_synced` below already sorts for the
        // same reason; `.min()` is the one-object equivalent of that same
        // fix.
        if let Some(id) = self.desynced_objects.iter().min() {
            return Err(Error::construct(format!(
                "HybridCollisionEnv::mutate_world: object '{id}' could not be decomposed into \
                 collision points; env_field no longer reflects it (see \
                 HybridCollisionEnv::desynced_objects)"
            )));
        }
        Ok(result)
    }

    /// Applies one [`Notification`] to `env_field`/`env_field_points`,
    /// mirroring upstream `CollisionEnvDistanceField::notifyObjectChange`
    /// (`collision_env_distance_field.cpp:1704-1728`). See
    /// [`Self::mutate_world`]'s doc comment for the branch-by-`Action`
    /// rationale and why this cannot use [`ParryCollisionEnv`]'s
    /// `OctreeCache` pattern unchanged.
    fn apply_notification(&mut self, notification: &Notification) {
        let id = notification.object.id().to_string();

        if notification.action.contains(Action::DESTROY) {
            if let Some(points) = self.env_field_points.remove(&id) {
                self.env_field.remove_points_from_field(&points);
            }
            self.desynced_objects.remove(&id);
            return;
        }

        let resolution = self.distance_field_config.geometry.resolution;
        let new_points =
            match collision_object_point_decomposition(&notification.object, resolution) {
                Ok(decomposition) => decomposition.collision_points(),
                Err(_) => {
                    self.desynced_objects.insert(id);
                    return;
                }
            };

        if notification.action.contains(Action::MOVE_SHAPE)
            || notification.action.contains(Action::REMOVE_SHAPE)
        {
            if let Some(old_points) = self.env_field_points.get(&id) {
                self.env_field.remove_points_from_field(old_points);
            }
        }
        self.env_field.add_points_to_field(&new_points);
        self.env_field_points.insert(id.clone(), new_points);
        self.desynced_objects.remove(&id);
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

    /// Guard called by every `check_*_distance_field`/
    /// `get_collision_gradients`/`get_all_collisions` method below before it
    /// reads `env_field`. See [`Self::desynced_objects`]'s doc for why a
    /// stale `env_field` must be reported here rather than silently used --
    /// this is the read-side half of that invariant; [`Self::mutate_world`]
    /// is the write-side half.
    ///
    /// # Errors
    ///
    /// Names every desynced object id if `env_field` does not currently
    /// reflect every object in [`Self::world`].
    fn check_env_field_synced(&self) -> Result<()> {
        if self.desynced_objects.is_empty() {
            return Ok(());
        }
        let mut ids: Vec<&str> = self.desynced_objects.iter().map(String::as_str).collect();
        ids.sort_unstable();
        Err(Error::construct(format!(
            "HybridCollisionEnv: env_field does not reflect object(s) {ids:?} -- the last \
             mutate_world call that touched them could not decompose their current shapes"
        )))
    }

    /// Upstream `CollisionEnvHybrid::checkCollisionDistanceField`'s four
    /// overloads (`collision_env_hybrid.cpp:107-133`), all
    /// `cenv_distance_->checkCollision(...)`. See [`Self::mutate_world`]'s
    /// doc for why `env_field` is read directly here rather than rebuilt,
    /// and this module's doc comment for why that is not a new
    /// arity-collapse decision on top of
    /// [`DistanceFieldCollisionCache::check_collision`].
    ///
    /// # Errors
    ///
    /// Returns an error naming every desynced object id if `env_field` does
    /// not currently reflect every object in [`Self::world`] -- see
    /// [`Self::mutate_world`] for how an object becomes desynced and why the
    /// stale field is reported rather than silently used. Also propagates
    /// [`DistanceFieldCollisionCache::check_collision`]'s own errors.
    pub fn check_collision_distance_field<'s>(
        &'s mut self,
        req: &CollisionRequest,
        state: &Posed<'_, 'm>,
        acm: Option<&AllowedCollisionMatrix>,
        current_attached_bodies: &[AttachedBodyGeometry<'_>],
    ) -> Result<(CollisionResult, GroupStateRepresentation<'s, 'm>)> {
        self.check_env_field_synced()?;
        self.distance_field.check_collision(
            req,
            state,
            acm,
            current_attached_bodies,
            &self.env_field,
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
    /// Returns an error naming every desynced object id if `env_field` does
    /// not currently reflect every object in [`Self::world`] -- see
    /// [`Self::mutate_world`] for how an object becomes desynced and why the
    /// stale field is reported rather than silently used. Also propagates
    /// [`DistanceFieldCollisionCache::check_robot_collision`]'s own errors.
    pub fn check_robot_collision_distance_field<'s>(
        &'s mut self,
        req: &CollisionRequest,
        state: &Posed<'_, 'm>,
        acm: Option<&AllowedCollisionMatrix>,
        current_attached_bodies: &[AttachedBodyGeometry<'_>],
    ) -> Result<(CollisionResult, GroupStateRepresentation<'s, 'm>)> {
        self.check_env_field_synced()?;
        self.distance_field.check_robot_collision(
            req,
            state,
            acm,
            current_attached_bodies,
            &self.env_field,
        )
    }

    /// Upstream `CollisionEnvHybrid::getCollisionGradients`
    /// (`collision_env_hybrid.cpp:172-177`), `cenv_distance_->getCollisionGradients(...)`.
    /// See [`Self::check_collision_distance_field`]'s doc for the
    /// environment field rationale.
    ///
    /// # Errors
    ///
    /// Returns an error naming every desynced object id if `env_field` does
    /// not currently reflect every object in [`Self::world`] -- see
    /// [`Self::mutate_world`] for how an object becomes desynced and why the
    /// stale field is reported rather than silently used. Also propagates
    /// [`DistanceFieldCollisionCache::get_collision_gradients`]'s own errors.
    pub fn get_collision_gradients<'s>(
        &'s mut self,
        req: &CollisionRequest,
        state: &Posed<'_, 'm>,
        acm: Option<&AllowedCollisionMatrix>,
        current_attached_bodies: &[AttachedBodyGeometry<'_>],
    ) -> Result<GroupStateRepresentation<'s, 'm>> {
        self.check_env_field_synced()?;
        self.distance_field.get_collision_gradients(
            req,
            state,
            acm,
            current_attached_bodies,
            &self.env_field,
        )
    }

    /// Upstream `CollisionEnvHybrid::getAllCollisions`
    /// (`collision_env_hybrid.cpp:179-184`), `cenv_distance_->getAllCollisions(...)`.
    /// See [`Self::check_collision_distance_field`]'s doc for the
    /// environment field rationale.
    ///
    /// # Errors
    ///
    /// Returns an error naming every desynced object id if `env_field` does
    /// not currently reflect every object in [`Self::world`] -- see
    /// [`Self::mutate_world`] for how an object becomes desynced and why the
    /// stale field is reported rather than silently used. Also propagates
    /// [`DistanceFieldCollisionCache::get_all_collisions`]'s own errors.
    pub fn get_all_collisions<'s>(
        &'s mut self,
        req: &CollisionRequest,
        state: &Posed<'_, 'm>,
        acm: Option<&AllowedCollisionMatrix>,
        current_attached_bodies: &[AttachedBodyGeometry<'_>],
    ) -> Result<(CollisionResult, GroupStateRepresentation<'s, 'm>)> {
        self.check_env_field_synced()?;
        self.distance_field.get_all_collisions(
            req,
            state,
            acm,
            current_attached_bodies,
            &self.env_field,
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
        )
        .unwrap();
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

    /// # Mutation-tested, not merely asserted
    ///
    /// This test's `res_after.collision` assertion is the discriminator for
    /// the "stale second world" bug upstream's `setWorld` override exists to
    /// prevent (see [`HybridCollisionEnv::mutate_world`]'s §196.3/§230 doc):
    /// if [`HybridCollisionEnv::apply_notification`] were never called, or
    /// were called with a stale `Notification`, `env_field` would never see
    /// the `add_shape` below, and `res_after.collision` would stay `false`.
    /// Confirmed by temporarily making `apply_notification`'s body a no-op
    /// and re-running: this test failed, at exactly this assertion, with the
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
        )
        .unwrap();
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

        env.mutate_world(|w| {
            w.add_shape(
                "obstacle",
                Arc::new(Shape::Cuboid(Cuboid::new(0.3, 0.3, 0.3).unwrap())),
                Isometry3::identity(),
            )
        })
        .unwrap();

        let (res_after, _gsr2) = env
            .check_robot_collision_distance_field(&req, &posed, None, &[])
            .unwrap();
        assert!(
            res_after.collision,
            "the very next check_robot_collision_distance_field call after mutate_world() must \
             see the swap: a 0.3 m cube at the origin overlaps mid's own 0.1 m box there, and \
             mutate_world's apply_notification call keeps env_field in step with every \
             mutation, so there is no stale field to explain a miss here"
        );
    }

    /// The strongest test this crate's own role brief names for the
    /// incremental primitives [`PropagationDistanceField::add_points_to_field`]/
    /// [`PropagationDistanceField::remove_points_from_field`] themselves,
    /// applied here to [`HybridCollisionEnv::mutate_world`]'s use of them:
    /// after a sequence of adds/moves/removes, `env_field` must be in the
    /// same state a fresh [`HybridCollisionEnv::new`] over the same final
    /// [`World`] would build directly -- proving the per-mutation
    /// remove-then-add bookkeeping in
    /// [`HybridCollisionEnv::apply_notification`] never drifts from "what a
    /// clean rebuild would produce," not just that *some* change is visible
    /// (that narrower claim is
    /// [`check_robot_collision_distance_field_reflects_a_world_swap_on_the_next_call`]'s
    /// job, above).
    #[test]
    fn env_field_after_incremental_churn_matches_a_fresh_rebuild_of_the_same_world() {
        let (model, _gap) = two_link_gap_model();
        let padding = LinkPaddingScale::new();
        let link_body_decompositions =
            add_link_body_decompositions(&model, 0.02, &padding, None).unwrap();
        let config = test_distance_field_config();

        let mut env = HybridCollisionEnv::new(
            World::new(),
            padding.clone(),
            link_body_decompositions.clone(),
            config,
            0.0,
        )
        .unwrap();

        let shape_a = Arc::new(Shape::Cuboid(Cuboid::new(0.2, 0.2, 0.2).unwrap()));
        env.mutate_world(|w| w.add_shape("a", shape_a, Isometry3::translation(0.5, 0.0, 0.0)))
            .unwrap();
        let shape_b = Arc::new(Shape::Cuboid(Cuboid::new(0.1, 0.1, 0.1).unwrap()));
        env.mutate_world(|w| w.add_shape("b", shape_b, Isometry3::translation(-0.5, 0.0, 0.0)))
            .unwrap();
        env.mutate_world(|w| w.move_object("a", Isometry3::translation(0.1, 0.0, 0.0)))
            .unwrap();
        env.mutate_world(|w| w.remove_object("b")).unwrap();
        let shape_c = Arc::new(Shape::Cuboid(Cuboid::new(0.15, 0.15, 0.15).unwrap()));
        env.mutate_world(|w| w.add_shape("c", shape_c, Isometry3::translation(0.0, 0.5, 0.0)))
            .unwrap();

        let fresh = HybridCollisionEnv::new(
            env.world().clone(),
            padding,
            link_body_decompositions,
            config,
            0.0,
        )
        .unwrap();

        assert_eq!(
            field_distances(&env.env_field),
            field_distances(&fresh.env_field),
            "env_field after incremental add/move/remove churn must equal a fresh rebuild over \
             the same final World -- a mismatch here means apply_notification's per-object \
             remove-then-add bookkeeping left env_field in a state no clean rebuild could reach"
        );
    }

    /// `desynced_objects` is a `HashSet<String>`; its iteration order
    /// depends on the process's randomized hash seed, not on id order or
    /// insertion order. Seeds four ids whose insertion order does not
    /// match their lexicographic order, calls `mutate_world` with a
    /// no-op mutation (`None::<Notification>` contributes nothing, so the
    /// pre-seeded set is exactly what `mutate_world` reports on), and pins
    /// that the named object is always `"alpha"`, the lexicographic
    /// minimum -- the invariant `.min()` guarantees regardless of hash
    /// seed. Before this fix (`.iter().next()`), which id got named
    /// depended on that seed: reverting to `.next()` and rerunning this
    /// exact test across repeated `cargo test` invocations (a fresh
    /// process, and so a fresh hash seed, each time) surfaced `"mango"`,
    /// `"zeta"`, `"delta"` and `"alpha"` itself as the reported id across
    /// different runs, confirming the message really does vary run to run
    /// rather than merely being theoretically capable of it.
    #[test]
    fn mutate_world_names_the_lexicographically_smallest_desynced_object() {
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
        )
        .unwrap();
        for id in ["zeta", "mango", "alpha", "delta"] {
            env.desynced_objects.insert(id.to_string());
        }

        let err = env.mutate_world(|_w| None::<Notification>).unwrap_err();

        assert!(
            err.to_string().contains("object 'alpha'"),
            "must always name the lexicographic minimum, not an arbitrary HashSet element: {err}"
        );
    }

    /// Every cell's positive distance, in `(x, y, z)` order -- enough to
    /// compare two [`PropagationDistanceField`]s for equality the way
    /// upstream's own `areDistanceFieldsDistancesEqual` test helper does
    /// (see [`PropagationDistanceField::update_points_in_field`]'s "Deviation
    /// from upstream" doc). `test_distance_field_config` sets
    /// `use_signed_distance_field: false`, so the negative field is never
    /// populated and comparing it would add nothing.
    fn field_distances(field: &PropagationDistanceField) -> Vec<f64> {
        let mut out = Vec::new();
        for x in 0..field.num_cells_x() {
            for y in 0..field.num_cells_y() {
                for z in 0..field.num_cells_z() {
                    out.push(field.distance_cell(x, y, z));
                }
            }
        }
        out
    }
}
