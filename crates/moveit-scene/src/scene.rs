// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/planning_scene/include/moveit/planning_scene/planning_scene.hpp
//   moveit_core/planning_scene/src/planning_scene.cpp

use std::collections::{BTreeMap, BTreeSet};
use std::sync::Arc;

use moveit_collision::{
    Action, AllowedCollisionMatrix, BodyType, CollisionEnv, CollisionRequest, CollisionResult,
    Contact, DistanceRequest, MoveObjectOutcome, Notification, World,
};
use moveit_error::{Error, Result};
use moveit_geometry::{Isometry3, Shape};
use moveit_model::RobotModel;
use moveit_srdf::SrdfModel;
use moveit_state::{Posed, RobotState};

use crate::attached_body::AttachedBody;
use crate::layered::Layered;
use crate::world_diff::WorldDiff;

/// The environment a planning instance reasons about: the world, the ACM,
/// the current [`RobotState`], attached bodies, and (for a diff scene) a
/// parent to fall back on. Upstream `planning_scene::PlanningScene`.
///
/// # Scope
///
/// Ported: the world (via [`PlanningScene::world`] and its mutators), the
/// ACM, the current state, attached bodies (see [`AttachedBody`]'s module
/// doc for why they live here rather than on [`RobotState`]), and the
/// parent/child diff relationship ([`PlanningScene::diff`],
/// [`PlanningScene::push_diffs`], [`PlanningScene::decouple_parent`]).
///
/// Deferred, not ported: anything that round-trips a `moveit_msgs` type
/// (`getPlanningSceneMsg`/`setPlanningSceneDiffMsg`/
/// `processCollisionObjectMsg`/octomap handling — D1, this is a
/// ROS-independent core crate), `scene_transforms_`/`getTransforms`
/// (upstream's separate named-frame lookup layer; no `Transforms`/
/// `SceneTransforms` exists in this port yet), object colors/types
/// (`object_colors_`/`object_types_`, cosmetic bookkeeping with no
/// collision-relevant behavior), `isState*` (constraint-checking
/// passthroughs; `moveit_constraints` is out of scope, see above) and cost
/// sources (`getCollisionEnv`/... never surface `CollisionRequest::cost` from
/// a `PlanningScene` entry point upstream either).
///
/// # Collision checking
///
/// [`PlanningScene::check_collision`]/[`PlanningScene::check_self_collision`]/
/// [`PlanningScene::check_robot_collision`]/[`PlanningScene::distance_to_collision`]/
/// [`PlanningScene::colliding_pairs`]/[`PlanningScene::colliding_links`] are
/// generic over a caller-supplied `E: CollisionEnv<Posed<'_, 'm>>` (in
/// practice [`moveit_collision::ParryCollisionEnv`]) rather than owning one —
/// upstream's `PlanningScene` owns *two* (`getCollisionEnv`/
/// `getCollisionEnvUnpadded`, one per `CollisionDetectorAllocator` plugin),
/// switched on `CollisionRequest::pad_environment_collisions`/
/// `pad_self_collisions`. D4's compile-time-registry redesign replaces that
/// plugin selection with the caller choosing (and owning) a concrete `E`
/// directly, so there is only ever one backend in play here; every method
/// below applies whatever padding `E` itself was built with (see
/// `ParryCollisionEnv`'s own doc) to both the self- and robot-collision
/// checks alike, rather than switching backends per flag.
///
/// Every method here also collapses upstream's separate const/non-const
/// overload pairs (`checkCollision(state) const` vs `checkCollision(state)`,
/// the latter calling `updateCollisionBodyTransforms()` first) into one
/// `&mut self` method that always calls [`RobotState::update`] — `update`
/// already no-ops when nothing is dirty (see its own doc comment), so this
/// reproduces both overloads' observable behavior without a separate
/// already-clean fast path to maintain.
///
/// [`PlanningScene::check_collision`] delegates to
/// [`moveit_collision::CollisionEnv::check_collision`]'s existing default
/// (self-collision first, then robot-collision with the *remaining* contact
/// budget, merged) rather than upstream's own `PlanningScene::checkCollision`
/// body, which checks robot-collision first and returns early once
/// `res.contacts.size() >= req.max_contacts` (a *pair* count, not a total
/// contact count) — a different order and a different early-exit condition.
/// This is a deliberate deviation, not an oversight: `check_collision`'s
/// default already has its own boundary tests (`max_contacts: 0` must not
/// suppress the collision flag) and is the one piece of budget-subtraction
/// logic this port maintains for every backend, so this method consumes it
/// rather than re-deriving upstream's dual-env order on top.
///
/// `E`'s own [`World`] (`ParryCollisionEnv::world`) is a value the caller
/// constructs, not a live view of [`PlanningScene::world`]: unlike
/// upstream's `CollisionEnv`, which upstream's own `PlanningScene`
/// constructs internally and keeps in sync via `World`'s observer callback
/// (`notifyObjectChange`), this crate's `E` is handed in by the caller at
/// every call. A caller wanting `E` to see this scene's world passes
/// `env: &ParryCollisionEnv::new(scene.world().clone(), padding_scale)` — a
/// cheap call thanks to [`World`]'s own copy-on-write [`Clone`] — and must
/// re-clone after any world mutation the caller wants reflected.
///
/// # The parent/child design
///
/// Upstream's child scene holds a `PlanningSceneConstPtr parent_`
/// (`shared_ptr` to a *const* view of the same object some other, mutable
/// `PlanningScenePtr` may still be updating) and every accessor for a
/// `std::optional`-backed field falls through to the parent when the
/// child's own optional is empty. Two things are true about that design at
/// once: it is genuinely *live* — a mutation the parent's owner makes after
/// the child was created is visible through the child's next read, for as
/// long as the child has not diverged — and it is the "implicit value plus
/// re-derived-every-read fallthrough" shape this project treats as a defect
/// source (see this crate's private `layered` module doc).
///
/// This port keeps the live fallthrough — the crate-private `Layered<T>`
/// exists to make that safe rather than to remove it, see its own doc — but
/// deliberately does **not** keep upstream's *mutable-parent-aliasing*
/// half: `parent` here is `Arc<PlanningScene<'m>>`, an immutable snapshot
/// captured at [`PlanningScene::diff`] time. If the scene that was diffed
/// is later mutated through some other handle, the child does **not**
/// observe it. That is a real, deliberate semantic deviation, not a gap:
/// reproducing upstream's version faithfully would need `Arc<Mutex<...>>`
/// (or `Rc<RefCell<...>>`) for every layered field, i.e. shared mutable
/// state visible through a nominally-`const` pointer — exactly the
/// aliased-mutability hazard Rust's ownership model exists to rule out, for
/// a usage pattern (a long-lived parent scene mutated *after* a child was
/// handed out, with the child expected to observe it) that is not the
/// primary way `PlanningScene::diff` is used in practice: the common
/// pattern is diff → read-only planning against the frozen child →
/// [`PlanningScene::push_diffs`] the result back, not concurrent mutation of
/// both. [`PlanningScene::decouple_parent`] remains the explicit,
/// documented way to freeze a child's inherited state and stop tracking a
/// parent at all — it does the same materialization this deviation would
/// otherwise force at `diff()` time, just deferred until the caller asks
/// for it.
pub struct PlanningScene<'m> {
    name: String,
    parent: Option<Arc<PlanningScene<'m>>>,
    robot_model: &'m RobotModel,
    robot_state: Layered<RobotState<'m>>,
    world: World,
    /// `Some` only for a diff (child) scene — upstream `world_diff_`,
    /// `nullptr unless this is a diff scene`.
    world_diff: Option<WorldDiff>,
    acm: Layered<AllowedCollisionMatrix>,
    /// Never layered: every scene, root or child, owns its own copy,
    /// seeded by cloning the parent's at [`PlanningScene::diff`] time — the
    /// same "always owned, seeded by clone" treatment upstream gives
    /// `world_` itself. See [`AttachedBody`]'s module doc for why this
    /// state lives here at all instead of on [`RobotState`].
    attached_bodies: BTreeMap<String, AttachedBody>,
}

impl<'m> PlanningScene<'m> {
    /// A root scene over an empty [`World`], with a [`RobotState`] at its
    /// default values and an ACM built from `srdf`. Upstream's
    /// `PlanningScene(robot_model, world)` constructor with `world`
    /// defaulted, followed by `initialize()`'s state/ACM setup.
    pub fn new(robot_model: &'m RobotModel, srdf: &SrdfModel) -> Self {
        Self::with_world(robot_model, srdf, World::new())
    }

    /// Like [`PlanningScene::new`], but starting from `world` instead of an
    /// empty one. Upstream `PlanningScene(robot_model, world)`.
    pub fn with_world(robot_model: &'m RobotModel, srdf: &SrdfModel, world: World) -> Self {
        let mut robot_state = RobotState::new(robot_model);
        robot_state.set_to_default_values();
        Self {
            name: String::new(),
            parent: None,
            robot_model,
            robot_state: Layered::Own(robot_state),
            world,
            world_diff: None,
            acm: Layered::Own(AllowedCollisionMatrix::from_srdf(srdf)),
            attached_bodies: BTreeMap::new(),
        }
    }

    /// This scene's name. Empty by default. Upstream `getName`.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Set this scene's name. Upstream `setName`.
    pub fn set_name(&mut self, name: impl Into<String>) {
        self.name = name.into();
    }

    /// The parent this scene diffs against, if it is a diff scene. Upstream
    /// `getParent` — see the type doc for why this is an immutable
    /// snapshot, not upstream's live-aliased const view.
    pub fn parent(&self) -> Option<&PlanningScene<'m>> {
        self.parent.as_deref()
    }

    /// The robot model this scene was built for. Upstream `getRobotModel`.
    pub fn robot_model(&self) -> &'m RobotModel {
        self.robot_model
    }

    // ---- current state --------------------------------------------------

    /// The state the robot is assumed to be in: this scene's own, or (via
    /// `Layered::resolve`) the parent's. Upstream `getCurrentState`.
    pub fn current_state(&self) -> &RobotState<'m> {
        self.robot_state.resolve(|| {
            self.parent
                .as_ref()
                .expect("Layered::Inherited robot_state requires a parent scene")
                .current_state()
        })
    }

    /// Mutable access to the current state, materializing this scene's own
    /// copy (cloned from the resolved value) first if it was inherited.
    /// Upstream `getCurrentStateNonConst`.
    pub fn current_state_mut(&mut self) -> &mut RobotState<'m> {
        if !self.robot_state.is_own() {
            let cloned = self.current_state().clone();
            self.robot_state = Layered::Own(cloned);
        }
        match &mut self.robot_state {
            Layered::Own(state) => state,
            Layered::Inherited => unreachable!("just materialized above"),
        }
    }

    /// Replace the current state outright. Upstream `setCurrentState`.
    pub fn set_current_state(&mut self, state: RobotState<'m>) {
        self.robot_state = Layered::Own(state);
    }

    // ---- allowed collision matrix ----------------------------------------

    /// This scene's ACM: its own, or the parent's. Upstream
    /// `getAllowedCollisionMatrix`.
    pub fn allowed_collision_matrix(&self) -> &AllowedCollisionMatrix {
        self.acm.resolve(|| {
            self.parent
                .as_ref()
                .expect("Layered::Inherited acm requires a parent scene")
                .allowed_collision_matrix()
        })
    }

    /// Mutable access to the ACM, materializing this scene's own copy first
    /// if it was inherited. Upstream `getAllowedCollisionMatrixNonConst`.
    pub fn allowed_collision_matrix_mut(&mut self) -> &mut AllowedCollisionMatrix {
        if !self.acm.is_own() {
            let cloned = self.allowed_collision_matrix().clone();
            self.acm = Layered::Own(cloned);
        }
        match &mut self.acm {
            Layered::Own(acm) => acm,
            Layered::Inherited => unreachable!("just materialized above"),
        }
    }

    /// Replace the ACM outright. Upstream `setAllowedCollisionMatrix`.
    pub fn set_allowed_collision_matrix(&mut self, acm: AllowedCollisionMatrix) {
        self.acm = Layered::Own(acm);
    }

    // ---- world ------------------------------------------------------------

    /// The world this scene sees. Upstream `getWorld`.
    pub fn world(&self) -> &World {
        &self.world
    }

    /// Feed a notification produced by a `self.world` mutation into
    /// `self.world_diff`, if this is a diff scene. The one funnel every
    /// world-mutating method below goes through, so "does this scene track
    /// a diff" is answered in exactly one place.
    fn track(&mut self, notification: Option<Notification>) {
        if let (Some(diff), Some(notification)) = (&mut self.world_diff, &notification) {
            diff.record(notification);
        }
    }

    fn track_all(&mut self, notifications: &[Notification]) {
        if let Some(diff) = &mut self.world_diff {
            diff.record_all(notifications);
        }
    }

    /// Add a single shape as a new (or augmented) world object. Upstream
    /// `World::addToObject`, reached through the scene so the change is
    /// tracked.
    pub fn add_shape(&mut self, id: &str, shape: Arc<Shape>, pose: Isometry3) {
        let notification = self.world.add_shape(id, shape, pose);
        self.track(notification);
    }

    /// Move an existing object by `transform` (world-frame composition).
    /// Upstream `World::moveObject`.
    pub fn move_object(&mut self, id: &str, transform: Isometry3) -> MoveObjectOutcome {
        let outcome = self.world.move_object(id, transform);
        if let MoveObjectOutcome::Moved(notification) = &outcome {
            self.track(Some(notification.clone()));
        }
        outcome
    }

    /// Remove a world object entirely. Upstream `processCollisionObjectRemove`:
    /// removes the object and — unlike attach, which never touches the
    /// ACM (see [`PlanningScene::attach`]'s doc) — prunes the ACM entry for
    /// `id` too, since this really is the object leaving the scene for
    /// good. Returns whether an object was actually removed.
    pub fn remove_object(&mut self, id: &str) -> bool {
        let Some(notification) = self.world.remove_object(id) else {
            return false;
        };
        self.track(Some(notification));
        self.allowed_collision_matrix_mut().remove_entries_for(id);
        true
    }

    /// Remove every world object. Upstream `removeAllCollisionObjects`,
    /// pruning each removed id's ACM entry the same way
    /// [`PlanningScene::remove_object`] does.
    pub fn remove_all_objects(&mut self) {
        let ids = self.world.object_ids();
        let notifications = self.world.clear_objects();
        self.track_all(&notifications);
        let acm = self.allowed_collision_matrix_mut();
        for id in &ids {
            acm.remove_entries_for(id);
        }
    }

    // ---- attached bodies ----------------------------------------------------

    /// Every attached body, by id.
    pub fn attached_bodies(&self) -> impl Iterator<Item = &AttachedBody> {
        self.attached_bodies.values()
    }

    /// The attached body named `id`, if any. Upstream
    /// `RobotState::getAttachedBody`.
    pub fn attached_body(&self, id: &str) -> Option<&AttachedBody> {
        self.attached_bodies.get(id)
    }

    /// Whether a body named `id` is currently attached. Upstream
    /// `RobotState::hasAttachedBody`.
    pub fn has_attached_body(&self, id: &str) -> bool {
        self.attached_bodies.contains_key(id)
    }

    /// Attach the world object `id` to `link_name`, removing it from the
    /// world. Upstream `processAttachedCollisionObjectMsg`'s ADD branch,
    /// restricted to the "object already exists in the world" case — see
    /// [`PlanningScene::attach_new`] for attaching geometry that is not
    /// already a world object.
    ///
    /// # Deviation from the task brief, confirmed against upstream
    ///
    /// Neither this method nor [`PlanningScene::detach`] touches the ACM.
    /// Reading `processAttachedCollisionObjectMsg` and
    /// `RobotState::attachBody` in full turned up no ACM mutation on
    /// attach/detach anywhere in `moveit_core` — upstream's own
    /// `UpdateACMAfterObjectRemoval` test (`test_planning_scene.cpp`) has a
    /// caller set an explicit ACM entry by hand (`acm.setEntry(object_name,
    /// hand_group_links, true)`) and then shows it survives an attach/detach
    /// round-trip untouched. The real, narrower invariant upstream protects
    /// — visible in `pushDiffs`' `"if object is attached, it should not be
    /// removed from the ACM"` guard — is that an id's ACM entry must survive
    /// as long as the id exists *either* in the world *or* as an attached
    /// body; only an outright deletion
    /// ([`PlanningScene::remove_object`]/[`PlanningScene::remove_all_objects`])
    /// may prune it. [`PlanningScene::push_diffs`] reproduces that exact
    /// guard.
    pub fn attach(
        &mut self,
        id: &str,
        link_name: &str,
        touch_links: BTreeSet<String>,
    ) -> Result<()> {
        if !self.robot_model.has_link_model(link_name) {
            return Err(Error::other(format!("no such link: {link_name}")));
        }
        let Some(object) = self.world.get_object(id) else {
            return Err(Error::other(format!(
                "attaching '{id}' requires it to already exist in the world; use attach_new for \
                 geometry that is not already a world object"
            )));
        };
        let link_transform = {
            let posed = self.current_state_mut().update();
            posed.global_link_transform(link_name)?
        };
        let object_pose = object.pose();
        let shapes: Vec<Arc<Shape>> = object
            .shapes()
            .iter()
            .map(|s| Arc::clone(s.shape()))
            .collect();
        let shape_poses: Vec<Isometry3> = object
            .shapes()
            .iter()
            .map(|s| link_transform.inverse() * object_pose * s.pose())
            .collect();

        let notification = self.world.remove_object(id);
        self.track(notification);

        self.attached_bodies.insert(
            id.to_owned(),
            AttachedBody::new(
                id.to_owned(),
                link_name.to_owned(),
                shapes,
                shape_poses,
                touch_links,
            ),
        );
        Ok(())
    }

    /// Attach geometry that is not already a world object. Upstream
    /// `processAttachedCollisionObjectMsg`'s ADD branch, message-shapes
    /// case. `shape_poses` are relative to `link_name`'s own frame (see
    /// [`AttachedBody`]'s module doc).
    pub fn attach_new(
        &mut self,
        id: &str,
        link_name: &str,
        shapes: Vec<Arc<Shape>>,
        shape_poses: Vec<Isometry3>,
        touch_links: BTreeSet<String>,
    ) -> Result<()> {
        if !self.robot_model.has_link_model(link_name) {
            return Err(Error::other(format!("no such link: {link_name}")));
        }
        if shapes.is_empty() || shapes.len() != shape_poses.len() {
            return Err(Error::other(
                "attach_new requires at least one shape, with one pose per shape",
            ));
        }
        self.attached_bodies.insert(
            id.to_owned(),
            AttachedBody::new(
                id.to_owned(),
                link_name.to_owned(),
                shapes,
                shape_poses,
                touch_links,
            ),
        );
        Ok(())
    }

    /// Detach `id`, adding its geometry back to the world at its current
    /// global pose. Upstream `processAttachedCollisionObjectMsg`'s REMOVE
    /// (detach) branch. See [`PlanningScene::attach`]'s doc for why this
    /// does not touch the ACM.
    ///
    /// Errors, leaving the body still attached, if the world already has an
    /// object named `id` — upstream instead warns and silently drops the
    /// detached geometry rather than overwrite it; this port surfaces that
    /// as an error instead of a silent geometry loss.
    pub fn detach(&mut self, id: &str) -> Result<AttachedBody> {
        let Some(body) = self.attached_bodies.get(id) else {
            return Err(Error::other(format!("no attached body named '{id}'")));
        };
        if self.world.has_object(id) {
            return Err(Error::other(format!(
                "cannot detach '{id}': the world already has an object with that name"
            )));
        }
        let link_name = body.link_name().to_owned();
        let link_transform = {
            let posed = self.current_state_mut().update();
            posed.global_link_transform(&link_name)?
        };
        let body = self
            .attached_bodies
            .remove(id)
            .expect("just confirmed present above");
        let notification =
            self.world
                .add_to_object(id, link_transform, body.shapes(), body.shape_poses());
        self.track(notification);
        Ok(body)
    }

    // ---- collision checking -----------------------------------------------
    //
    // See the type doc's "Collision checking" section for the overload
    // collapse (const/non-const, padded/unpadded env selection) and the
    // documented deviation in `check_collision`'s self-then-robot order.

    /// Check `self` for both self- and robot-collision against `env`, using
    /// this scene's own [`PlanningScene::allowed_collision_matrix`]. Upstream
    /// `checkCollision`/`checkCollisionUnpadded`, collapsed — see the type
    /// doc.
    pub fn check_collision<E>(&mut self, env: &E, request: &CollisionRequest) -> CollisionResult
    where
        E: for<'s> CollisionEnv<Posed<'s, 'm>>,
    {
        let acm = self.allowed_collision_matrix().clone();
        let posed = self.current_state_mut().update();
        env.check_collision(request, &posed, Some(&acm))
    }

    /// Check `self` for self-collision only against `env`. Upstream
    /// `checkSelfCollision`.
    pub fn check_self_collision<E>(
        &mut self,
        env: &E,
        request: &CollisionRequest,
    ) -> CollisionResult
    where
        E: for<'s> CollisionEnv<Posed<'s, 'm>>,
    {
        let acm = self.allowed_collision_matrix().clone();
        let posed = self.current_state_mut().update();
        env.check_self_collision(request, &posed, Some(&acm))
    }

    /// Check `self` against the world only (no self-collision) against
    /// `env`. Upstream's `checkCollision` family has no standalone
    /// robot-vs-world-only entry point at the `PlanningScene` level (only
    /// the combined `checkCollision`); this exposes
    /// [`moveit_collision::CollisionEnv::check_robot_collision`] directly
    /// through the scene's own state/ACM for a caller that wants that half
    /// alone, e.g. to build [`PlanningScene::colliding_pairs`]-style
    /// diagnostics without paying for a self-collision pass.
    pub fn check_robot_collision<E>(
        &mut self,
        env: &E,
        request: &CollisionRequest,
    ) -> CollisionResult
    where
        E: for<'s> CollisionEnv<Posed<'s, 'm>>,
    {
        let acm = self.allowed_collision_matrix().clone();
        let posed = self.current_state_mut().update();
        env.check_robot_collision(request, &posed, Some(&acm))
    }

    /// The distance between the robot at `self`'s current state and the
    /// nearest world collision, ignoring self-collisions. Upstream
    /// `distanceToCollision`/`distanceToCollisionUnpadded`, collapsed (see
    /// the type doc) — always against this scene's own
    /// [`PlanningScene::allowed_collision_matrix`], matching every upstream
    /// overload that does not take an explicit different `acm` (the ones
    /// that do are not ported: a caller wanting a one-off ACM already has
    /// `env.distance_robot` directly, with `DistanceRequest { acm:
    /// Some(&other_acm), .. }`).
    pub fn distance_to_collision<E>(&mut self, env: &E) -> f64
    where
        E: for<'s> CollisionEnv<Posed<'s, 'm>>,
    {
        let acm = self.allowed_collision_matrix().clone();
        let posed = self.current_state_mut().update();
        let request = DistanceRequest {
            acm: Some(&acm),
            ..Default::default()
        };
        env.distance_robot(&request, &posed)
            .minimum_distance
            .distance
    }

    /// Every colliding pair (self- and robot-collision alike) for `self`'s
    /// current state against `env`, keyed the same way
    /// [`moveit_collision::ContactData::by_pair`] is. Upstream
    /// `getCollidingPairs`.
    ///
    /// # Deviation from upstream
    ///
    /// `req.group_name` is not threaded through: `ParryCollisionEnv` never
    /// reads [`CollisionRequest::group_name`] at all (`parry`'s module doc,
    /// deviation 1 — group filtering needs a `RobotModel`-derived active-link
    /// set upstream's own FCL backend never wires up either), so a
    /// `group_name` parameter here would be inert. `req.max_contacts` is
    /// upstream's `getLinkModelsWithCollisionGeometry().size() + 1`; this
    /// port's [`RobotModel`] has no such query (see
    /// `moveit-model::robot_model`'s doc), so this uses every link with a
    /// non-empty [`moveit_model::LinkModel::shapes`] instead — a superset of
    /// links that actually convert to collision geometry (a link could still
    /// hold only [`Shape::OcTree`]/a degenerate [`Shape::Plane`], see
    /// `parry`'s module doc deviations 9–10), so this can only make the
    /// budget larger than upstream's, never smaller — the ceiling this
    /// exists to avoid hitting is never hit early.
    pub fn colliding_pairs<E>(&mut self, env: &E) -> BTreeMap<(String, String), Vec<Contact>>
    where
        E: for<'s> CollisionEnv<Posed<'s, 'm>>,
    {
        let max_contacts = self
            .robot_model
            .link_models()
            .iter()
            .filter(|link| !link.shapes().is_empty())
            .count()
            + 1;
        let request = CollisionRequest {
            contacts: true,
            max_contacts,
            max_contacts_per_pair: 1,
            ..Default::default()
        };
        self.check_collision(env, &request)
            .contacts
            .map(|contacts| contacts.by_pair)
            .unwrap_or_default()
    }

    /// Every robot link involved in a collision for `self`'s current state
    /// against `env`. Upstream `getCollidingLinks`.
    pub fn colliding_links<E>(&mut self, env: &E) -> Vec<String>
    where
        E: for<'s> CollisionEnv<Posed<'s, 'm>>,
    {
        let mut links = Vec::new();
        for contacts in self.colliding_pairs(env).values() {
            for contact in contacts {
                if contact.body_type_1 == BodyType::RobotLink {
                    links.push(contact.body_name_1.clone());
                }
                if contact.body_type_2 == BodyType::RobotLink {
                    links.push(contact.body_name_2.clone());
                }
            }
        }
        links
    }

    // ---- diff / decouple ----------------------------------------------------

    /// A new child scene that diffs against `self`: an empty
    /// [`WorldDiff`], a cloned [`World`] snapshot (cheap: see
    /// [`World`]'s own copy-on-write `Clone`), inherited state/ACM, and a
    /// cloned attached-body set. Upstream `diff()`.
    pub fn diff(self: &Arc<Self>) -> PlanningScene<'m> {
        PlanningScene {
            name: String::new(),
            parent: Some(Arc::clone(self)),
            robot_model: self.robot_model,
            robot_state: Layered::Inherited,
            world: self.world.clone(),
            world_diff: Some(WorldDiff::new()),
            acm: Layered::Inherited,
            attached_bodies: self.attached_bodies.clone(),
        }
    }

    /// A standalone copy of `scene`, decoupled from any parent even if
    /// `scene` itself has one. Upstream static `PlanningScene::clone`.
    pub fn cloned(scene: &Arc<PlanningScene<'m>>) -> PlanningScene<'m> {
        let mut result = scene.diff();
        result.decouple_parent();
        result.name = scene.name.clone();
        result
    }

    /// If this scene has a parent, apply what changed here — current state,
    /// ACM, and world changes — onto `target`. A no-op if this scene has no
    /// parent. Upstream `pushDiffs`.
    ///
    /// The world-change replay preserves the one ACM subtlety upstream is
    /// careful about: an id whose only recorded action here is a *pure*
    /// [`Action::DESTROY`] (exact equality, not "contains `DESTROY`" — an id
    /// destroyed and then recreated within this diff, per
    /// [`WorldDiff::record`](crate::WorldDiff::record)'s
    /// coalescing, is *not* pure `DESTROY` and takes the other branch) has
    /// its `target` ACM entry pruned too — *unless* `target` already
    /// considers that id an attached body, exactly mirroring upstream's
    /// `if (!scene->getCurrentState().hasAttachedBody(it.first))` guard via
    /// [`PlanningScene::has_attached_body`].
    pub fn push_diffs(&self, target: &mut PlanningScene<'m>) {
        if self.parent.is_none() {
            return;
        }
        if let Layered::Own(state) = &self.robot_state {
            target.set_current_state(state.clone());
        }
        if let Layered::Own(acm) = &self.acm {
            target.set_allowed_collision_matrix(acm.clone());
        }
        let Some(diff) = &self.world_diff else {
            return;
        };
        for (id, action) in diff.changes() {
            if *action == Action::DESTROY {
                let notification = target.world.remove_object(id);
                target.track(notification);
                if !target.has_attached_body(id) {
                    target.allowed_collision_matrix_mut().remove_entries_for(id);
                }
            } else if let Some(object) = self.world.get_object(id) {
                let notification = target.world.remove_object(id);
                target.track(notification);
                let shapes: Vec<Arc<Shape>> = object
                    .shapes()
                    .iter()
                    .map(|s| Arc::clone(s.shape()))
                    .collect();
                let shape_poses: Vec<Isometry3> =
                    object.shapes().iter().map(|s| s.pose()).collect();
                let notification =
                    target
                        .world
                        .add_to_object(id, object.pose(), &shapes, &shape_poses);
                target.track(notification);
                let subframes: BTreeMap<String, Isometry3> = object
                    .subframe_names()
                    .map(|name| {
                        (
                            name.to_owned(),
                            object
                                .subframe_pose(name)
                                .expect("name was just listed by subframe_names"),
                        )
                    })
                    .collect();
                target.world.set_subframes_of_object(id, subframes);
            }
        }
    }

    /// Materialize every inherited field locally, discard the world diff
    /// (nothing left to diff against), and drop the parent. A no-op if this
    /// scene has no parent. Upstream `decoupleParent`, scoped to the fields
    /// this port carries (`scene_transforms_`/`object_colors_`/
    /// `object_types_` are not ported — see the type's scope doc).
    pub fn decouple_parent(&mut self) {
        if self.parent.is_none() {
            return;
        }
        if !self.robot_state.is_own() {
            let cloned = self.current_state().clone();
            self.robot_state = Layered::Own(cloned);
        }
        if !self.acm.is_own() {
            let cloned = self.allowed_collision_matrix().clone();
            self.acm = Layered::Own(cloned);
        }
        self.world_diff = None;
        self.parent = None;
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use moveit_collision::{AllowedCollisionType, LinkPaddingScale, ParryCollisionEnv};
    use moveit_geometry::Cuboid;
    use moveit_srdf::SrdfModel;

    use super::*;

    // Fixture: a fixed-base robot with a shapeless `base` link and a `hand`
    // link one fixed joint away, so `attach`/`detach` have a real link to
    // attach to without needing a state to be posed first.
    const SRDF_XML: &str = r#"<robot name="test">
        <virtual_joint name="fixed_base" type="fixed" parent_frame="world" child_link="base"/>
    </robot>"#;

    const URDF_XML: &str = r#"<robot name="test">
        <link name="base"/>
        <link name="hand"/>
        <joint name="hand_joint" type="fixed">
            <parent link="base"/>
            <child link="hand"/>
            <origin xyz="0 0 1"/>
        </joint>
    </robot>"#;

    fn build_model() -> RobotModel {
        let urdf = urdf_rs::read_from_string(URDF_XML).expect("test URDF must parse");
        let srdf = SrdfModel::parse_str(SRDF_XML).expect("test SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, URDF_XML, &srdf)
            .expect("test fixture model must build")
    }

    fn srdf() -> SrdfModel {
        SrdfModel::parse_str(SRDF_XML).expect("test SRDF must parse")
    }

    fn cuboid_shape() -> Arc<Shape> {
        Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap()))
    }

    // ---- diff: added vs removed vs moved world objects ---------------------

    #[test]
    fn diff_scene_records_an_added_world_object_without_touching_the_parent() {
        let model = build_model();
        let root = Arc::new(PlanningScene::new(&model, &srdf()));
        let mut child = root.diff();

        child.add_shape("box", cuboid_shape(), Isometry3::identity());

        let diff = child
            .world_diff
            .as_ref()
            .expect("child scene must track a diff");
        assert!(diff.get("box").unwrap().contains(Action::CREATE));
        assert!(diff.get("box").unwrap().contains(Action::ADD_SHAPE));
        assert_eq!(child.world().object_ids(), vec!["box".to_owned()]);
        assert!(root.world().object_ids().is_empty());
    }

    #[test]
    fn diff_scene_records_a_removed_world_object_without_touching_the_parent() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.add_shape("box", cuboid_shape(), Isometry3::identity());
        let root = Arc::new(root);
        let mut child = root.diff();

        assert!(child.remove_object("box"));

        let diff = child.world_diff.as_ref().unwrap();
        assert_eq!(diff.get("box").unwrap(), Action::DESTROY);
        assert!(child.world().object_ids().is_empty());
        assert_eq!(root.world().object_ids(), vec!["box".to_owned()]);
    }

    #[test]
    fn diff_scene_records_a_move_only_change_for_an_existing_object() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.add_shape("box", cuboid_shape(), Isometry3::identity());
        let root = Arc::new(root);
        let mut child = root.diff();

        let outcome = child.move_object("box", Isometry3::translation(1.0, 0.0, 0.0));
        assert!(matches!(outcome, MoveObjectOutcome::Moved(_)));

        let diff = child.world_diff.as_ref().unwrap();
        assert_eq!(diff.get("box").unwrap(), Action::MOVE_SHAPE);
    }

    // ---- attach/detach: the ACM round-trips exactly, because neither touches it ----

    #[test]
    fn attach_then_detach_round_trips_the_acm_exactly() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene.add_shape("box", cuboid_shape(), Isometry3::identity());
        scene
            .allowed_collision_matrix_mut()
            .set_entry("box", "hand", true);
        let before = scene.allowed_collision_matrix().all_entry_names();

        scene.attach("box", "hand", BTreeSet::new()).unwrap();
        assert!(scene.has_attached_body("box"));
        assert!(!scene.world().has_object("box"));
        assert_eq!(scene.allowed_collision_matrix().all_entry_names(), before);

        scene.detach("box").unwrap();
        assert!(!scene.has_attached_body("box"));
        assert!(scene.world().has_object("box"));
        assert_eq!(scene.allowed_collision_matrix().all_entry_names(), before);
        assert_eq!(
            scene
                .allowed_collision_matrix()
                .entry("box", "hand")
                .unwrap()
                .kind(),
            AllowedCollisionType::Always
        );
    }

    #[test]
    fn remove_object_prunes_the_acm_entry_but_attach_leaves_it_alone() {
        let model = build_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene.add_shape("box", cuboid_shape(), Isometry3::identity());
        scene
            .allowed_collision_matrix_mut()
            .set_entry("box", "hand", true);
        assert!(scene.allowed_collision_matrix().has_entry("box"));

        scene.remove_object("box");

        assert!(!scene.allowed_collision_matrix().has_entry("box"));
    }

    // ---- push_diffs: the "attached, not deleted" ACM guard ------------------

    #[test]
    fn push_diffs_prunes_the_acm_entry_for_a_genuinely_destroyed_object() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.add_shape("box", cuboid_shape(), Isometry3::identity());
        root.allowed_collision_matrix_mut()
            .set_entry("box", "hand", true);
        let root = Arc::new(root);
        let mut child = root.diff();
        child.remove_object("box");

        let mut target = PlanningScene::new(&model, &srdf());
        target.add_shape("box", cuboid_shape(), Isometry3::identity());
        target
            .allowed_collision_matrix_mut()
            .set_entry("box", "hand", true);

        child.push_diffs(&mut target);

        assert!(!target.world().has_object("box"));
        assert!(!target.allowed_collision_matrix().has_entry("box"));
    }

    #[test]
    fn push_diffs_preserves_the_acm_entry_when_the_target_still_has_it_attached() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.add_shape("box", cuboid_shape(), Isometry3::identity());
        root.allowed_collision_matrix_mut()
            .set_entry("box", "hand", true);
        let root = Arc::new(root);
        let mut child = root.diff();
        // Attaching removes "box" from the child's world, producing the same
        // pure-DESTROY diff entry a real deletion would.
        child.attach("box", "hand", BTreeSet::new()).unwrap();

        let mut target = PlanningScene::new(&model, &srdf());
        target.add_shape("box", cuboid_shape(), Isometry3::identity());
        target
            .allowed_collision_matrix_mut()
            .set_entry("box", "hand", true);
        // The target independently attached "box" too — this is what the
        // guard actually reads.
        target.attach("box", "hand", BTreeSet::new()).unwrap();

        child.push_diffs(&mut target);

        assert!(target.allowed_collision_matrix().has_entry("box"));
    }

    #[test]
    fn push_diffs_is_a_no_op_for_a_root_scene() {
        let model = build_model();
        let root = PlanningScene::new(&model, &srdf());
        let mut target = PlanningScene::new(&model, &srdf());
        target.add_shape("box", cuboid_shape(), Isometry3::identity());

        root.push_diffs(&mut target);

        assert!(target.world().has_object("box"));
    }

    // ---- parent fallthrough vs override --------------------------------------

    #[test]
    fn child_falls_through_to_the_parent_acm_for_a_pair_it_never_touched() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.allowed_collision_matrix_mut()
            .set_entry("a", "b", true);
        let root = Arc::new(root);
        let child = root.diff();

        assert!(!child.acm.is_own());
        assert_eq!(
            child
                .allowed_collision_matrix()
                .entry("a", "b")
                .unwrap()
                .kind(),
            AllowedCollisionType::Always
        );
    }

    #[test]
    fn child_override_diverges_from_the_parent_without_mutating_it() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.allowed_collision_matrix_mut()
            .set_entry("c", "d", false);
        let root = Arc::new(root);
        let mut child = root.diff();

        child
            .allowed_collision_matrix_mut()
            .set_entry("c", "d", true);

        assert!(child.acm.is_own());
        assert_eq!(
            child
                .allowed_collision_matrix()
                .entry("c", "d")
                .unwrap()
                .kind(),
            AllowedCollisionType::Always
        );
        assert_eq!(
            root.allowed_collision_matrix()
                .entry("c", "d")
                .unwrap()
                .kind(),
            AllowedCollisionType::Never
        );
    }

    #[test]
    fn child_current_state_falls_through_until_mutated_then_materializes_its_own() {
        let model = build_model();
        let root = Arc::new(PlanningScene::new(&model, &srdf()));
        let mut child = root.diff();

        assert!(!child.robot_state.is_own());
        assert_eq!(
            child.current_state().positions(),
            root.current_state().positions()
        );

        child.current_state_mut();
        assert!(child.robot_state.is_own());
    }

    // ---- decouple_parent: isolation from later parent mutation --------------

    #[test]
    fn decouple_parent_then_mutating_the_former_parent_is_not_observed() {
        let model = build_model();
        let mut root = PlanningScene::new(&model, &srdf());
        root.allowed_collision_matrix_mut()
            .set_entry("a", "b", true);
        let mut root = Arc::new(root);
        let mut child = root.diff();

        child.decouple_parent();
        assert!(child.parent().is_none());

        Arc::get_mut(&mut root)
            .expect("sole owner: child dropped its Arc clone in decouple_parent")
            .allowed_collision_matrix_mut()
            .set_entry("a", "b", false);

        assert_eq!(
            child
                .allowed_collision_matrix()
                .entry("a", "b")
                .unwrap()
                .kind(),
            AllowedCollisionType::Always
        );
    }

    // ---- collision checking -----------------------------------------------

    // Fixture: a fixed-base robot with two independent floating-joint boxes,
    // `p`/`q`, so each can be posed to an arbitrary independent global
    // transform. Mirrors `moveit_collision::parry`'s own test fixture.

    fn box_link(name: &str) -> String {
        format!(
            r#"<link name="{name}">
                <collision><geometry><box size="1 1 1"/></geometry></collision>
            </link>"#
        )
    }

    fn floating_joint(name: &str, parent: &str, child: &str) -> String {
        format!(
            r#"<joint name="{name}" type="floating">
                <parent link="{parent}"/>
                <child link="{child}"/>
            </joint>"#
        )
    }

    fn build_collision_model() -> RobotModel {
        let urdf_xml = format!(
            r#"<robot name="test">
                <link name="base"/>
                {p}{joint_p}{q}{joint_q}
            </robot>"#,
            p = box_link("p"),
            joint_p = floating_joint("joint_p", "base", "p"),
            q = box_link("q"),
            joint_q = floating_joint("joint_q", "base", "q"),
        );
        let urdf = urdf_rs::read_from_string(&urdf_xml).expect("test URDF must parse");
        let srdf = SrdfModel::parse_str(SRDF_XML).expect("test SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf)
            .expect("test fixture model must build")
    }

    #[test]
    fn check_self_collision_reports_overlapping_links() {
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene
            .current_state_mut()
            .set_joint_transform("joint_q", &Isometry3::translation(0.5, 0.0, 0.0))
            .unwrap();
        let env = ParryCollisionEnv::default();

        let result = scene.check_self_collision(&env, &CollisionRequest::default());

        assert!(result.collision);
    }

    #[test]
    fn check_self_collision_reports_clear_when_links_are_apart() {
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene
            .current_state_mut()
            .set_joint_transform("joint_q", &Isometry3::translation(5.0, 0.0, 0.0))
            .unwrap();
        let env = ParryCollisionEnv::default();

        let result = scene.check_self_collision(&env, &CollisionRequest::default());

        assert!(!result.collision);
    }

    #[test]
    fn check_robot_collision_sees_the_scenes_world_once_cloned_into_env() {
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene.add_shape(
            "obstacle",
            Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
            Isometry3::identity(),
        );
        let env = ParryCollisionEnv::new(scene.world().clone(), LinkPaddingScale::default());

        let result = scene.check_robot_collision(&env, &CollisionRequest::default());

        assert!(result.collision);
    }

    #[test]
    fn check_robot_collision_does_not_see_a_world_object_added_after_the_env_was_built() {
        // Documents the "env's world is a value, not a live view" contract
        // from the type doc's "Collision checking" section.
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        let env = ParryCollisionEnv::new(scene.world().clone(), LinkPaddingScale::default());
        scene.add_shape(
            "obstacle",
            Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
            Isometry3::identity(),
        );

        let result = scene.check_robot_collision(&env, &CollisionRequest::default());

        assert!(!result.collision);
    }

    #[test]
    fn check_collision_finds_self_collision_even_when_the_world_is_clear() {
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene
            .current_state_mut()
            .set_joint_transform("joint_q", &Isometry3::translation(0.5, 0.0, 0.0))
            .unwrap();
        let env = ParryCollisionEnv::default();

        let result = scene.check_collision(&env, &CollisionRequest::default());

        assert!(result.collision);
    }

    #[test]
    fn distance_to_collision_reports_the_gap_to_a_world_object() {
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene.add_shape(
            "obstacle",
            Arc::new(Shape::Cuboid(Cuboid::new(1.0, 1.0, 1.0).unwrap())),
            Isometry3::translation(2.0, 0.0, 0.0),
        );
        let env = ParryCollisionEnv::new(scene.world().clone(), LinkPaddingScale::default());

        let distance = scene.distance_to_collision(&env);

        assert!((distance - 1.0).abs() < 1e-9);
    }

    #[test]
    fn colliding_pairs_and_colliding_links_report_the_overlapping_self_collision_pair() {
        let model = build_collision_model();
        let mut scene = PlanningScene::new(&model, &srdf());
        scene
            .current_state_mut()
            .set_joint_transform("joint_q", &Isometry3::translation(0.5, 0.0, 0.0))
            .unwrap();
        let env = ParryCollisionEnv::default();

        let pairs = scene.colliding_pairs(&env);
        assert_eq!(pairs.len(), 1);
        assert!(pairs.contains_key(&("p".to_string(), "q".to_string())));

        let mut links = scene.colliding_links(&env);
        links.sort();
        assert_eq!(links, vec!["p".to_string(), "q".to_string()]);
    }
}
