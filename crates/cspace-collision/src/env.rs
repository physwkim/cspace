// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection/include/moveit/collision_detection/collision_env.hpp
//   moveit_core/collision_detection/src/collision_env.cpp

//! The backend-facing interface of upstream `collision_detection::CollisionEnv`,
//! split into two pieces per this port's design (see PORTING-PLAN.md D4).
//!
//! Upstream `CollisionEnv` is one abstract class mixing:
//!
//! - a **virtual interface** a collision backend (FCL/Bullet/parry)
//!   implements — `checkSelfCollision`, `checkRobotCollision`, `distanceSelf`,
//!   `distanceRobot` — plus one **concrete, non-virtual** convenience method
//!   built from them, `checkCollision`;
//! - **concrete, non-virtual bookkeeping** every subclass inherits verbatim
//!   and never overrides: the link padding/scale maps and their accessors.
//!
//! This module keeps only the first piece: [`CollisionEnv`], a trait with
//! [`CollisionEnv::check_collision`] as its one default (non-required)
//! method, matching upstream's virtual/concrete split exactly. The second
//! piece — padding/scale bookkeeping — is [`LinkPaddingScale`], a plain
//! struct with no trait at all, since nothing about it is backend-specific;
//! a concrete backend (the parry implementation, next task) embeds one as a
//! field alongside a [`crate::World`], the same way upstream's subclasses
//! inherit `link_padding_`/`link_scale_`/`world_` from the base class.
//!
//! # Why the trait is generic, not tied to a robot-state type
//!
//! Every upstream method here takes a `const moveit::core::RobotState&`.
//! `crates/cspace-core` (owned by another worker) has not ported
//! `RobotState` yet, and this crate must not take on that dependency mid-port
//! — so [`CollisionEnv`] is generic over `State`, an entirely opaque type as
//! far as this trait is concerned. Once `RobotState` exists, a concrete
//! backend implements `CollisionEnv<cspace_core::model::RobotState>`; this trait's
//! shape does not need to change for that to happen.
//!
//! # No `CollisionDetectorAllocator`, and no `linkme` registry
//!
//! This section used to defer the question ("a compile-time registry needs
//! at least one registrant to be worth adding. This task ends with a trait
//! and no implementation (no parry backend yet)"). That condition expired:
//! three concrete backends now implement [`CollisionEnv`] —
//! [`crate::ParryCollisionEnv`], [`crate::AllValidCollisionEnv`] and
//! `crate::distance_field::HybridCollisionEnv`. `PORTING-PLAN.md` §225.4
//! re-decided it rather than carrying the deferral forward: upstream's
//! `collision_detector_allocator.hpp` is **not ported**, and no `linkme`
//! slice replaces it.
//!
//! Upstream's allocator exists to defer the *type* of the backend to a
//! runtime string: `CollisionDetectorAllocator::allocateEnv` hands back a
//! `CollisionEnvPtr` and `getName()` labels the pairing, so
//! `PlanningScene::allocateCollisionDetector` can key a `collision_detector_`
//! map by that name and `getCollisionEnv(name)` can look one up
//! (`planning_scene.cpp:255-311`). This port made that decision once and
//! permanently, in the other direction: `cspace_planning::scene::PlanningScene`'s
//! collision methods are generic over a caller-supplied
//! `E: CollisionEnv<Posed<'_, 'm>>`, so the backend is named as a *type* at
//! the call site (see `scene.rs`'s own "Collision checking" section and its
//! `allocateCollisionDetector` disposition). There is no name to look up
//! and no map to key it in — `rg -n -i 'collision_detector|detector_name'
//! crates/ ros/ tools/ --glob '*.rs'` finds no selection site at all, only
//! doc comments naming the upstream symbol.
//!
//! So a registry here would have registrants and no consumer, and would
//! import a hazard this workspace has already paid for once
//! (`PORTING-PLAN.md` §177: `linkme` order is linker-section order, and
//! adding one dependency anywhere in the workspace silently reordered
//! `KINEMATICS_SOLVERS`). The other thing the allocator supplies, a uniform
//! three-overload construction protocol, does not fit the backends that
//! exist either: [`crate::ParryCollisionEnv::new`] takes a [`crate::World`] and
//! a [`LinkPaddingScale`], `HybridCollisionEnv::new` additionally takes a
//! distance-field config, link body decompositions and a collision
//! tolerance, and [`crate::AllValidCollisionEnv`] takes nothing — no
//! `allocateEnv(world, robot_model)` signature can serve all three.
//!
//! What keeps a future FCL-FFI backend addable — §4.5's stated reason for
//! keeping the allocator trait — is [`CollisionEnv`] itself, which such a
//! backend would implement exactly as the three above do. §225.4 narrows
//! §4.5's wording accordingly.
//!
//! # Out of scope
//!
//! `collision_plugin_cache.*` (pluginlib-based FCL/Bullet/distance-field
//! selection by runtime string — the mechanism the section above declines),
//! `collision_octomap_filter.*` and `occupancy_map.*` (both need an octomap
//! dependency and a `RobotState`) are not touched here.

use std::collections::BTreeMap;

use cspace_core::error::Result;

use crate::common::{
    AttachedBodyGeometry, CollisionRequest, CollisionResult, ContactData, DistanceRequest,
    DistanceResult,
};
use crate::matrix::AllowedCollisionMatrix;

/// The virtual interface of upstream `collision_detection::CollisionEnv`: what
/// a collision-checking backend must implement.
///
/// # Overload collapse (upstream → here)
///
/// - `checkSelfCollision(req, res, state)` and
///   `checkSelfCollision(req, res, state, acm)` both become
///   [`check_self_collision`](CollisionEnv::check_self_collision), with
///   `acm: Option<&AllowedCollisionMatrix>` — `None` for the no-ACM overload.
/// - `checkRobotCollision(req, res, state)` and
///   `checkRobotCollision(req, res, state, acm)` (comparing a single state to
///   the world) both become
///   [`check_robot_collision`](CollisionEnv::check_robot_collision), same
///   `Option` collapse.
/// - `checkRobotCollision(req, res, state1, state2, acm)` and
///   `checkRobotCollision(req, res, state1, state2)` (continuous, between two
///   states) both become
///   [`check_robot_collision_continuous`](CollisionEnv::check_robot_collision_continuous).
/// - `checkCollision(req, res, state)` and `checkCollision(req, res, state,
///   acm)` — both concrete/non-virtual upstream, not part of the abstract
///   interface — become the one default method
///   [`check_collision`](CollisionEnv::check_collision), also
///   `Option`-collapsed.
/// - `distanceSelf(req, res, state)` (pure virtual) becomes
///   [`distance_self`](CollisionEnv::distance_self). Its two convenience
///   overloads, `distanceSelf(state)` and `distanceSelf(state, acm)`, are
///   **not ported**: both build their `DistanceRequest` by calling
///   `req.enableGroup(getRobotModel())`, which needs a `RobotModel` this
///   crate does not have. A caller with a real `RobotModel` builds the
///   equivalent `DistanceRequest` itself and calls `distance_self` directly.
/// - `distanceRobot(req, res, state)` (pure virtual) becomes
///   [`distance_robot`](CollisionEnv::distance_robot). Its two convenience
///   overloads, `distanceRobot(state, verbose)` and `distanceRobot(state,
///   acm, verbose)`, are **not ported**, for the same `enableGroup`/
///   `RobotModel` reason.
///
/// Nothing above is silently dropped: every overload either maps onto one of
/// the five methods below, or is named here as not portable yet and why.
///
/// # Attached-body geometry: the upstream answer, and why this trait deviates
///
/// Upstream's own answer to "where does attached-body geometry enter a
/// collision check" is: the backend asks the *state* for it.
/// `CollisionEnvFCL::constructFCLObjectRobot` calls `state.getAttachedBodies(ab)`
/// and folds every attached body's geometry into the same per-robot object
/// every one of `checkSelfCollision`/`checkRobotCollision`/`distanceSelf`/
/// `distanceRobot` builds from — attached bodies are not a side channel some
/// of those four consult and others don't; they are simply part of what
/// "the robot at this state" means, upstream-side.
///
/// This crate's `State` is generic and opaque (see this module's doc,
/// above) precisely so it does not have to depend on `cspace_core::state`; that
/// same genericity is why it cannot be the thing attached bodies live on
/// here — `cspace_core::state::RobotState` itself carries no attached-body concept
/// yet (`cspace_planning::scene::AttachedBody`'s own doc: `PlanningScene` is the sole
/// owner of that data for now, precisely because `RobotState` has nowhere to
/// put it). Each method below therefore takes `attached_bodies: &[AttachedBodyGeometry<'_>]`
/// as an explicit parameter standing in for what `state.getAttachedBodies()`
/// would return if it could — passed alongside `state`, to every one of the
/// same four methods upstream feeds it into, preserving upstream's real
/// structural answer ("attached bodies are part of the robot, not a
/// separate query") as closely as this port's crate boundaries allow. When
/// `RobotState` gains attached-body support, this parameter folds into
/// `State` itself and every signature below simplifies back to upstream's
/// shape.
pub trait CollisionEnv<State> {
    /// `checkSelfCollision`: check the robot against itself. Any collision
    /// between any pair of links (and any attached body — see the trait
    /// doc) is reported; `acm` filters which pairs are allowed to collide
    /// (`None` reports every pair, matching the no-ACM overload).
    fn check_self_collision(
        &self,
        request: &CollisionRequest,
        state: &State,
        attached_bodies: &[AttachedBodyGeometry<'_>],
        acm: Option<&AllowedCollisionMatrix>,
    ) -> CollisionResult;

    /// `checkRobotCollision` (single-state overloads): check the robot
    /// (including any attached body — see the trait doc) at `state` against
    /// the world. Self-collisions are not checked.
    fn check_robot_collision(
        &self,
        request: &CollisionRequest,
        state: &State,
        attached_bodies: &[AttachedBodyGeometry<'_>],
        acm: Option<&AllowedCollisionMatrix>,
    ) -> CollisionResult;

    /// `checkRobotCollision` (two-state overloads): check the world for
    /// collision along the continuous path from `state1` to `state2`.
    /// Self-collisions are not checked.
    ///
    /// # Errors
    ///
    /// Upstream's own FCL backend does not implement this either —
    /// `CollisionEnvFCL::checkRobotCollision(req, res, state1, state2[, acm])`
    /// logs an error and returns with `res` untouched, which is
    /// indistinguishable, at the call site, from "checked; found nothing." A
    /// backend that cannot do continuous collision checking faithfully
    /// returns `Err` here instead of that silent no-op, so a caller cannot
    /// mistake "not implemented" for "clear."
    ///
    /// The same reasoning covers a backend that does implement it but cannot
    /// build one of the bodies in this particular scene: `Err` again, rather
    /// than an answer about the geometry it did manage to look at.
    fn check_robot_collision_continuous(
        &self,
        request: &CollisionRequest,
        state1: &State,
        state2: &State,
        attached_bodies: &[AttachedBodyGeometry<'_>],
        acm: Option<&AllowedCollisionMatrix>,
    ) -> Result<CollisionResult>;

    /// `distanceSelf(req, res, state)`: the distance to self-collision at
    /// `state`, including any attached body (see the trait doc).
    fn distance_self(
        &self,
        request: &DistanceRequest<'_>,
        state: &State,
        attached_bodies: &[AttachedBodyGeometry<'_>],
    ) -> DistanceResult;

    /// `distanceRobot(req, res, state)`: the distance between the robot
    /// (including any attached body — see the trait doc) at `state` and the
    /// world.
    fn distance_robot(
        &self,
        request: &DistanceRequest<'_>,
        state: &State,
        attached_bodies: &[AttachedBodyGeometry<'_>],
    ) -> DistanceResult;

    /// `checkCollision`: self-collision, then robot-collision only if either
    /// no collision was found yet or `request.contacts` is set and there is
    /// still room for more (`request.max_contacts`) — upstream's exact guard
    /// in `CollisionEnv::checkCollision`.
    ///
    /// "Room for more" counts *pairs*, not contacts: upstream compares
    /// `res.contacts.size()` against `max_contacts`, and a pair can hold
    /// several contacts. See
    /// [`ContactData::pair_count`](crate::common::ContactData::pair_count).
    ///
    /// Upstream passes one `CollisionResult&` by reference into both calls
    /// and lets each backend accumulate into it in place, so the robot-check
    /// callback can see how many contacts the self-check callback already
    /// put in `res.contacts` and stop once their combined total reaches
    /// `req.max_contacts` (`collisionCallback`'s own `res_->contact_count`
    /// accounting, in `collision_common.cpp`). This port's
    /// [`check_self_collision`](CollisionEnv::check_self_collision) and
    /// [`check_robot_collision`](CollisionEnv::check_robot_collision) each
    /// return their own owned [`CollisionResult`] instead (this crate's
    /// structured-return idiom — see `world`'s module docs), so the robot
    /// check cannot see the self check's contacts by itself
    /// (PORTING-PLAN.md §10.5). This default implementation closes that gap
    /// by passing the *remaining* budget into the second call explicitly:
    /// `request.max_contacts` less however many contacts
    /// [`check_self_collision`](CollisionEnv::check_self_collision) already
    /// found ([`ContactData::count`](crate::common::ContactData::count), the
    /// total contact count upstream's `contact_count` tracks — not
    /// [`ContactData::pair_count`](crate::common::ContactData::pair_count),
    /// which is a different quantity the entry guard below reads for a
    /// different reason), then folds the second result into the first via
    /// [`CollisionResult::merge`]. A backend that also enforces
    /// `max_contacts` against its own request cannot then store more than
    /// `request.max_contacts` contacts in total across both calls.
    ///
    /// `cost_sources` is deliberately *not* rebudgeted like `max_contacts`
    /// above, even though upstream's `res_->cost_sources` is the same kind
    /// of shared, per-insertion-trimmed container as `res_->contacts`
    /// (`collision_detection_fcl/collision_common.cpp:285-287`, `:351-353`,
    /// `:388-390`). The difference is what the trim *selects*: `contacts`
    /// stops accumulating once a running count is reached (first-found,
    /// first-kept — no reordering, `collision_common.cpp:250-254`), so
    /// rebudgeting the second call's `max_contacts` down by however many the
    /// first call already found reproduces it exactly. `cost_sources` is a
    /// `std::set` insert-and-trim, i.e. a global top-`max_cost_sources`
    /// selection by `cost * getVolume()` — rebudgeting the second call the
    /// same way would let a more costly source the second phase finds lose
    /// to a less costly one the first phase already spent its (now-shrunk)
    /// budget keeping, since neither phase's own local trim ever compares
    /// its candidates against the other phase's. Both phases get the full,
    /// unmodified `request.max_cost_sources` instead, and
    /// [`CollisionResult::cap_cost_sources`] re-selects the global top-N
    /// once over the merged union below — see that method's doc for why
    /// this also reproduces the shared set's dedup and ordering, not just
    /// its count.
    fn check_collision(
        &self,
        request: &CollisionRequest,
        state: &State,
        attached_bodies: &[AttachedBodyGeometry<'_>],
        acm: Option<&AllowedCollisionMatrix>,
    ) -> CollisionResult {
        let mut result = self.check_self_collision(request, state, attached_bodies, acm);
        let contacts_have_room = result
            .contacts
            .as_ref()
            .is_some_and(|c| c.pair_count() < request.max_contacts);
        if !result.collision || contacts_have_room {
            let already_found = result.contacts.as_ref().map_or(0, ContactData::count);
            let remaining_request = CollisionRequest {
                max_contacts: request.max_contacts.saturating_sub(already_found),
                ..request.clone()
            };
            result.merge(self.check_robot_collision(
                &remaining_request,
                state,
                attached_bodies,
                acm,
            ));
        }
        result.cap_cost_sources(request.max_cost_sources);
        result
    }
}

fn validated_padding(padding: f64) -> f64 {
    if padding.is_finite() && padding >= 0.0 {
        padding
    } else {
        0.0
    }
}

/// Upstream's `scale < DBL_EPSILON` / `scale > DBL_MAX` checks never reject a
/// `NaN` scale (every comparison against `NaN` is `false`), so a `NaN` scale
/// silently passes upstream's `validateScale` as valid. `is_finite()` rejects
/// it too, matching the documented intent ("Scale must be positive"/"must be
/// finite") rather than the letter of a comparison that happens to miss one
/// case.
fn validated_scale(scale: f64) -> f64 {
    if scale.is_finite() && scale >= f64::EPSILON {
        scale
    } else {
        1.0
    }
}

/// Per-link collision padding and scale: upstream `CollisionEnv`'s
/// `link_padding_`/`link_scale_` maps and their accessors
/// (`setLinkPadding`/`getLinkPadding`/`setLinkScale`/`getLinkScale`, both
/// single- and map-argument forms, plus the set-every-known-link
/// `setPadding`/`setScale`).
///
/// # Deviation from upstream
///
/// Upstream's constructors seed every entry from
/// `robot_model_->getLinkModelsWithCollisionGeometry()` — a `RobotModel`
/// query this crate cannot make (see this module's doc). [`Self::with_links`]
/// takes that same link list as an explicit argument instead: a caller with
/// a real `RobotModel` passes
/// `model.link_models_with_collision_geometry().map(LinkModel::name)` (or
/// equivalent) itself. [`Self::set_padding_for_all_links`] and
/// [`Self::set_scale_for_all_links`] (`setPadding(double)`/`setScale(double)`)
/// likewise apply to every link *already tracked* — i.e. every link named in
/// [`Self::with_links`] or a prior `set_*` call — rather than re-querying a
/// `RobotModel` on every call.
///
/// That makes "tracked" load-bearing in a way it is not upstream, where the
/// bulk setters read the model rather than a map, so one link name owns one
/// entry holding both values here. Keeping padding and scale in separate
/// maps would give "tracked" two answers: a link named only to
/// `set_link_scale` would be invisible to `set_padding_for_all_links`, which
/// is the sort of gap upstream cannot have. A link tracked through either
/// setter therefore starts at the same padding `0.0` / scale `1.0` the
/// getters report for an untracked link, so tracking alone never changes
/// what is reported.
///
/// Every setter here funnels through the same private validating helpers
/// (upstream: `validatePadding`/`validateScale`, which log through `rclcpp`
/// — unavailable in a ROS-independent core crate, D1 — and are silently
/// invalid-but-ignored in one place: upstream's own `setLinkPadding(name,
/// padding)` calls `validatePadding(padding)` but discards its return value,
/// so an invalid *per-link* padding is stored verbatim while an invalid
/// *bulk* padding is caught and replaced with the default. That
/// inconsistency is closed here: every entry point — single, map, or
/// bulk — clamps through the same validating function, so an invalid value
/// can never reach the map by any path.
///
/// `updatedPaddingOrScaling`, upstream's protected virtual hook called
/// whenever a setter actually changes something, is not a callback here:
/// every setter returns the names of links that changed instead (this
/// crate's structured-return idiom — see `world`'s module docs), for a
/// caller (the concrete backend, once one exists) to act on directly.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct LinkPaddingScale {
    links: BTreeMap<String, LinkAdjustment>,
}

/// One tracked link's padding and scale. The defaults are what the getters
/// report for an untracked link, so tracking a link through one setter does
/// not change what the other one reports about it.
#[derive(Debug, Clone, Copy, PartialEq)]
struct LinkAdjustment {
    padding: f64,
    scale: f64,
}

impl Default for LinkAdjustment {
    fn default() -> Self {
        Self {
            padding: 0.0,
            scale: 1.0,
        }
    }
}

impl LinkPaddingScale {
    /// An empty padding/scale map, tracking no links yet.
    pub fn new() -> Self {
        Self::default()
    }

    /// The two `CollisionEnv` constructors that seed every link with a
    /// uniform `padding`/`scale`: `links` is the caller-supplied replacement
    /// for `robot_model_->getLinkModelsWithCollisionGeometry()` (see the
    /// type's deviation note).
    pub fn with_links(links: impl IntoIterator<Item = String>, padding: f64, scale: f64) -> Self {
        let adjustment = LinkAdjustment {
            padding: validated_padding(padding),
            scale: validated_scale(scale),
        };
        Self {
            links: links.into_iter().map(|link| (link, adjustment)).collect(),
        }
    }

    /// `getLinkPadding(link_name)`: `0.0` for an untracked link.
    pub fn link_padding(&self, link_name: &str) -> f64 {
        self.links
            .get(link_name)
            .map_or(LinkAdjustment::default().padding, |a| a.padding)
    }

    /// `getLinkScale(link_name)`: `1.0` for an untracked link.
    pub fn link_scale(&self, link_name: &str) -> f64 {
        self.links
            .get(link_name)
            .map_or(LinkAdjustment::default().scale, |a| a.scale)
    }

    /// `getLinkPadding()`: every tracked link's padding, in name order.
    pub fn link_paddings(&self) -> impl Iterator<Item = (&str, f64)> {
        self.links.iter().map(|(k, a)| (k.as_str(), a.padding))
    }

    /// `getLinkScale()`: every tracked link's scale, in name order.
    pub fn link_scales(&self) -> impl Iterator<Item = (&str, f64)> {
        self.links.iter().map(|(k, a)| (k.as_str(), a.scale))
    }

    /// `setLinkPadding(link_name, padding)`. Returns whether the resolved
    /// (post-validation) value actually changed.
    pub fn set_link_padding(&mut self, link_name: impl Into<String>, padding: f64) -> bool {
        let padding = validated_padding(padding);
        let entry = self.links.entry(link_name.into()).or_default();
        let changed = entry.padding != padding;
        entry.padding = padding;
        changed
    }

    /// `setLinkPadding(const std::map<std::string, double>&)`. Returns the
    /// names of links whose resolved padding actually changed.
    pub fn set_link_paddings(
        &mut self,
        paddings: impl IntoIterator<Item = (String, f64)>,
    ) -> Vec<String> {
        let mut changed = Vec::new();
        for (link_name, padding) in paddings {
            if self.set_link_padding(link_name.clone(), padding) {
                changed.push(link_name);
            }
        }
        changed
    }

    /// `setLinkScale(link_name, scale)`. Returns whether the resolved
    /// (post-validation) value actually changed.
    pub fn set_link_scale(&mut self, link_name: impl Into<String>, scale: f64) -> bool {
        let scale = validated_scale(scale);
        let entry = self.links.entry(link_name.into()).or_default();
        let changed = entry.scale != scale;
        entry.scale = scale;
        changed
    }

    /// `setLinkScale(const std::map<std::string, double>&)`. Returns the
    /// names of links whose resolved scale actually changed.
    pub fn set_link_scales(
        &mut self,
        scales: impl IntoIterator<Item = (String, f64)>,
    ) -> Vec<String> {
        let mut changed = Vec::new();
        for (link_name, scale) in scales {
            if self.set_link_scale(link_name.clone(), scale) {
                changed.push(link_name);
            }
        }
        changed
    }

    /// `setPadding(double)`: set every already-tracked link's padding to the
    /// same value (see the type's deviation note on what "every link" means
    /// here). Returns the names of links whose resolved padding actually
    /// changed.
    pub fn set_padding_for_all_links(&mut self, padding: f64) -> Vec<String> {
        let links: Vec<String> = self.links.keys().cloned().collect();
        self.set_link_paddings(links.into_iter().map(|link| (link, padding)))
    }

    /// `setScale(double)`: set every already-tracked link's scale to the same
    /// value (see the type's deviation note on what "every link" means
    /// here). Returns the names of links whose resolved scale actually
    /// changed.
    pub fn set_scale_for_all_links(&mut self, scale: f64) -> Vec<String> {
        let links: Vec<String> = self.links.keys().cloned().collect();
        self.set_link_scales(links.into_iter().map(|link| (link, scale)))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::common::{CollisionDistance, CostSource};
    use std::cell::Cell;
    use std::collections::{BTreeMap, BTreeSet};

    struct FakeRobotState;

    #[derive(Default)]
    struct StubEnv {
        self_result: CollisionResult,
        robot_result: CollisionResult,
        /// `max_contacts` of the request [`CollisionEnv::check_robot_collision`]
        /// was actually called with, for tests that check what budget
        /// [`CollisionEnv::check_collision`]'s default merge passed down.
        robot_seen_max_contacts: Cell<usize>,
        /// `max_cost_sources` of the request [`CollisionEnv::check_robot_collision`]
        /// was actually called with — the `cost_sources` analogue of
        /// `robot_seen_max_contacts` above.
        robot_seen_max_cost_sources: Cell<usize>,
        /// When non-empty, `check_robot_collision`'s mock draws from this
        /// pool instead of returning `robot_result` verbatim, trimming it to
        /// `request.max_cost_sources` via the same `BTreeSet` insert +
        /// pop-least-costly-while-over-budget mechanism a real backend
        /// (`parry.rs`'s `accumulate_collision`) already uses. `robot_result`
        /// above is a canned return value that ignores the budget it was
        /// asked for; tests that need to tell "budget respected" from
        /// "budget ignored" apart need a mock that actually respects it.
        robot_cost_source_candidates: Vec<CostSource>,
    }

    impl CollisionEnv<FakeRobotState> for StubEnv {
        fn check_self_collision(
            &self,
            _request: &CollisionRequest,
            _state: &FakeRobotState,
            _attached_bodies: &[AttachedBodyGeometry<'_>],
            _acm: Option<&AllowedCollisionMatrix>,
        ) -> CollisionResult {
            self.self_result.clone()
        }

        fn check_robot_collision(
            &self,
            request: &CollisionRequest,
            _state: &FakeRobotState,
            _attached_bodies: &[AttachedBodyGeometry<'_>],
            _acm: Option<&AllowedCollisionMatrix>,
        ) -> CollisionResult {
            self.robot_seen_max_contacts.set(request.max_contacts);
            self.robot_seen_max_cost_sources
                .set(request.max_cost_sources);
            if self.robot_cost_source_candidates.is_empty() {
                return self.robot_result.clone();
            }
            let mut set: BTreeSet<CostSource> =
                self.robot_cost_source_candidates.iter().copied().collect();
            while set.len() > request.max_cost_sources {
                set.pop_last();
            }
            CollisionResult {
                cost_sources: Some(set.into_iter().collect()),
                ..self.robot_result.clone()
            }
        }

        fn check_robot_collision_continuous(
            &self,
            _request: &CollisionRequest,
            _state1: &FakeRobotState,
            _state2: &FakeRobotState,
            _attached_bodies: &[AttachedBodyGeometry<'_>],
            _acm: Option<&AllowedCollisionMatrix>,
        ) -> Result<CollisionResult> {
            unimplemented!("not exercised by these tests")
        }

        fn distance_self(
            &self,
            _request: &DistanceRequest<'_>,
            _state: &FakeRobotState,
            _attached_bodies: &[AttachedBodyGeometry<'_>],
        ) -> DistanceResult {
            unimplemented!("not exercised by these tests")
        }

        fn distance_robot(
            &self,
            _request: &DistanceRequest<'_>,
            _state: &FakeRobotState,
            _attached_bodies: &[AttachedBodyGeometry<'_>],
        ) -> DistanceResult {
            unimplemented!("not exercised by these tests")
        }
    }

    fn contact_data(pair: (&str, &str), n: usize) -> ContactData {
        let mut by_pair = BTreeMap::new();
        by_pair.insert(
            (pair.0.to_string(), pair.1.to_string()),
            vec![crate::common::Contact::default(); n],
        );
        ContactData { by_pair }
    }

    fn cost_source(cost: f64) -> CostSource {
        CostSource {
            cost,
            ..Default::default()
        }
    }

    #[test]
    fn check_collision_skips_robot_check_when_self_collision_already_fills_max_contacts() {
        let env = StubEnv {
            self_result: CollisionResult {
                collision: true,
                contacts: Some(contact_data(("a", "b"), 1)),
                ..Default::default()
            },
            robot_result: CollisionResult {
                collision: true,
                contacts: Some(contact_data(("c", "d"), 1)),
                ..Default::default()
            },
            ..Default::default()
        };
        let request = CollisionRequest {
            contacts: true,
            max_contacts: 1,
            ..Default::default()
        };
        let result = env.check_collision(&request, &FakeRobotState, &[], None);
        // Only the self-collision contact is present: robot check was skipped.
        assert_eq!(result.contacts.unwrap().count(), 1);
    }

    #[test]
    fn check_collision_runs_robot_check_when_no_self_collision() {
        let env = StubEnv {
            self_result: CollisionResult::default(),
            robot_result: CollisionResult {
                collision: true,
                contacts: Some(contact_data(("c", "d"), 1)),
                ..Default::default()
            },
            ..Default::default()
        };
        let request = CollisionRequest {
            contacts: true,
            max_contacts: 1,
            ..Default::default()
        };
        let result = env.check_collision(&request, &FakeRobotState, &[], None);
        assert!(result.collision);
        assert_eq!(result.contacts.unwrap().count(), 1);
    }

    #[test]
    fn check_collision_measures_room_in_pairs_not_in_contacts() {
        // One pair holding three contacts, against max_contacts 2. Upstream
        // compares contacts.size() -- one pair -- so there is room and the
        // robot check runs; comparing the three contacts instead would skip
        // it. Every other test here puts one contact per pair, where the two
        // counts coincide and cannot tell the guards apart.
        let env = StubEnv {
            self_result: CollisionResult {
                collision: true,
                contacts: Some(contact_data(("a", "b"), 3)),
                ..Default::default()
            },
            robot_result: CollisionResult {
                collision: true,
                contacts: Some(contact_data(("c", "d"), 1)),
                ..Default::default()
            },
            ..Default::default()
        };
        let request = CollisionRequest {
            contacts: true,
            max_contacts: 2,
            ..Default::default()
        };
        let result = env.check_collision(&request, &FakeRobotState, &[], None);
        assert_eq!(result.contacts.unwrap().pair_count(), 2);
    }

    #[test]
    fn check_collision_runs_robot_check_when_contacts_have_room() {
        let env = StubEnv {
            self_result: CollisionResult {
                collision: true,
                contacts: Some(contact_data(("a", "b"), 1)),
                ..Default::default()
            },
            robot_result: CollisionResult {
                collision: true,
                contacts: Some(contact_data(("c", "d"), 1)),
                ..Default::default()
            },
            ..Default::default()
        };
        let request = CollisionRequest {
            contacts: true,
            max_contacts: 5,
            ..Default::default()
        };
        let result = env.check_collision(&request, &FakeRobotState, &[], None);
        assert_eq!(result.contacts.unwrap().count(), 2);
    }

    #[test]
    fn check_collision_passes_the_remaining_contact_budget_not_the_full_one() {
        // Self-collision already found one pair holding three contacts.
        // Upstream's shared `res_->contact_count` would already read 3 by the
        // time the robot check ran; this port must derive the same number
        // from the returned self-collision result and pass max_contacts - 3,
        // not the unmodified request (which would let the merged total
        // exceed max_contacts) and not max_contacts - pair_count (1), which
        // this test's 3-contacts-in-one-pair shape is built to tell apart
        // from the correct, count()-based subtraction.
        let env = StubEnv {
            self_result: CollisionResult {
                collision: true,
                contacts: Some(contact_data(("a", "b"), 3)),
                ..Default::default()
            },
            ..Default::default()
        };
        let request = CollisionRequest {
            contacts: true,
            max_contacts: 5,
            ..Default::default()
        };
        env.check_collision(&request, &FakeRobotState, &[], None);
        assert_eq!(env.robot_seen_max_contacts.get(), 2);
    }

    #[test]
    fn check_collision_passes_the_full_cost_source_budget_to_both_phases() {
        // Unlike max_contacts (see
        // check_collision_passes_the_remaining_contact_budget_not_the_full_one),
        // max_cost_sources is deliberately *not* rebudgeted down by however
        // many the self-collision phase already found: see
        // `check_collision`'s doc for why a per-phase rebudget breaks
        // cost_sources' global top-N selection (proven by
        // check_collision_keeps_the_globally_most_costly_source_not_whichever_phase_found_it_first
        // below). Both phases must see the same, full, unmodified
        // `request.max_cost_sources`.
        //
        // `self_result.collision` is `false` (rather than mirroring the
        // contacts test's `true`) so the robot check actually runs: the
        // `!result.collision || contacts_have_room` guard has no
        // `cost_sources`-based room check, only the `contacts`-based one,
        // and this test's `self_result` carries no `contacts`. Cost sources
        // are computed independent of the collision verdict upstream (see
        // `parry.rs`'s `accumulate_collision` doc), so a collision-free
        // self-check still reporting a cost source is a real, reachable
        // case, not a contrived one.
        let env = StubEnv {
            self_result: CollisionResult {
                collision: false,
                cost_sources: Some(vec![cost_source(1.0)]),
                ..Default::default()
            },
            ..Default::default()
        };
        let request = CollisionRequest {
            cost: true,
            max_cost_sources: 3,
            ..Default::default()
        };
        env.check_collision(&request, &FakeRobotState, &[], None);
        assert_eq!(env.robot_seen_max_cost_sources.get(), 3);
    }

    #[test]
    fn check_collision_keeps_the_globally_most_costly_source_not_whichever_phase_found_it_first() {
        // Upstream shares ONE std::set<CostSource> across both phases,
        // trimmed to max_cost_sources on every insertion
        // (collision_detection_fcl/collision_common.cpp:285-287, :351-353,
        // :388-390): the final set is the GLOBAL top-max_cost_sources by
        // `cost * getVolume()` across self- and robot-collision combined,
        // not whichever phase happens to run first. Self-collision here
        // yields a low-cost source (0.1); a compliant robot-collision
        // backend -- see `robot_cost_source_candidates`, which (unlike
        // `robot_result`) actually trims to the budget it is given, the way
        // a real backend does -- would yield a more costly one (1.0) if
        // given room. With max_cost_sources: 1, the merged result must keep
        // the 1.0 source and drop the 0.1 one, not the reverse: rebudgeting
        // the robot phase down by however many the self phase already found
        // (this crate's prior fix, before this test) leaves the robot phase
        // a budget of 1 - 1 = 0, so a compliant backend returns nothing for
        // it and the more costly source is lost outright.
        let env = StubEnv {
            self_result: CollisionResult {
                collision: false,
                cost_sources: Some(vec![cost_source(0.1)]),
                ..Default::default()
            },
            robot_result: CollisionResult {
                collision: true,
                ..Default::default()
            },
            robot_cost_source_candidates: vec![cost_source(1.0)],
            ..Default::default()
        };
        let request = CollisionRequest {
            cost: true,
            max_cost_sources: 1,
            ..Default::default()
        };

        let result = env.check_collision(&request, &FakeRobotState, &[], None);

        let sources = result.cost_sources.expect("cost was requested");
        assert_eq!(
            sources,
            vec![cost_source(1.0)],
            "the globally most costly source (1.0, found by the robot-collision phase) \
             must survive, not the less costly one (0.1) found first by the self-collision phase"
        );
    }

    #[test]
    fn check_collision_does_not_duplicate_a_cost_source_both_phases_report() {
        // Secondary consequence of the same defect: upstream's shared
        // std::set silently drops an Ord-Equal re-insert
        // (CostSource's deviation note), so a cost source both phases
        // report collapses to one entry, not two. `Vec::extend` (this
        // crate's prior merge) has no such dedup.
        let source = cost_source(1.0);
        let env = StubEnv {
            self_result: CollisionResult {
                collision: false,
                cost_sources: Some(vec![source]),
                ..Default::default()
            },
            robot_result: CollisionResult {
                collision: true,
                ..Default::default()
            },
            robot_cost_source_candidates: vec![source],
            ..Default::default()
        };
        let request = CollisionRequest {
            cost: true,
            max_cost_sources: 5,
            ..Default::default()
        };

        let result = env.check_collision(&request, &FakeRobotState, &[], None);

        assert_eq!(
            result.cost_sources.expect("cost was requested"),
            vec![source],
            "a cost source both phases report must collapse to one entry, matching \
             std::set::insert's silent drop of an Ord-Equal duplicate upstream"
        );
    }

    #[test]
    fn check_collision_sorts_the_merged_cost_sources_most_costly_first() {
        // Third consequence of the same defect: upstream's shared std::set
        // iterates most-costly-first as a whole, not self-phase-block then
        // robot-phase-block. Deliberately interleaved here (self finds the
        // middle-costliest, robot finds both the highest- and lowest-cost)
        // so a merge that is merely "each phase's own sorted run,
        // concatenated" would not happen to look sorted by accident.
        let env = StubEnv {
            self_result: CollisionResult {
                collision: false,
                cost_sources: Some(vec![cost_source(2.0)]),
                ..Default::default()
            },
            robot_result: CollisionResult {
                collision: true,
                ..Default::default()
            },
            robot_cost_source_candidates: vec![cost_source(3.0), cost_source(1.0)],
            ..Default::default()
        };
        let request = CollisionRequest {
            cost: true,
            max_cost_sources: 3,
            ..Default::default()
        };

        let result = env.check_collision(&request, &FakeRobotState, &[], None);

        assert_eq!(
            result.cost_sources.expect("cost was requested"),
            vec![cost_source(3.0), cost_source(2.0), cost_source(1.0)],
            "the merged cost_sources must be globally most-costly-first"
        );
    }

    #[test]
    fn merge_of_two_none_distances_is_none() {
        let mut a = CollisionResult::default();
        a.merge(CollisionResult::default());
        assert!(a.distance.is_none());
    }

    #[test]
    fn merge_picks_the_smaller_closest_distance() {
        let mut a = CollisionResult {
            distance: Some(CollisionDistance::Closest(2.0)),
            ..Default::default()
        };
        let b = CollisionResult {
            distance: Some(CollisionDistance::Closest(1.0)),
            ..Default::default()
        };
        a.merge(b);
        assert_eq!(a.distance.unwrap().distance(), 1.0);
    }

    #[test]
    fn link_padding_of_untracked_link_is_zero() {
        assert_eq!(LinkPaddingScale::new().link_padding("arm"), 0.0);
    }

    #[test]
    fn link_scale_of_untracked_link_is_one() {
        assert_eq!(LinkPaddingScale::new().link_scale("arm"), 1.0);
    }

    #[test]
    fn negative_padding_is_clamped_to_zero() {
        let mut p = LinkPaddingScale::new();
        p.set_link_padding("arm", -1.0);
        assert_eq!(p.link_padding("arm"), 0.0);
    }

    #[test]
    fn zero_padding_is_a_valid_value_not_clamped_away() {
        let mut p = LinkPaddingScale::new();
        p.set_link_padding("arm", 0.2);
        // 0.0 is within the valid range ([0, +inf)): this must actually take
        // effect, not be treated as invalid-and-reset-to-default (which
        // would coincidentally look the same, 0.0, since 0.0 is also the
        // default for an untracked link).
        assert!(p.set_link_padding("arm", 0.0));
        assert_eq!(p.link_padding("arm"), 0.0);
    }

    #[test]
    fn nan_padding_is_clamped_to_zero() {
        let mut p = LinkPaddingScale::new();
        p.set_link_padding("arm", f64::NAN);
        assert_eq!(p.link_padding("arm"), 0.0);
    }

    #[test]
    fn non_finite_padding_is_clamped_to_zero() {
        let mut p = LinkPaddingScale::new();
        p.set_link_padding("arm", f64::INFINITY);
        assert_eq!(p.link_padding("arm"), 0.0);
    }

    #[test]
    fn scale_below_epsilon_is_clamped_to_one() {
        let mut p = LinkPaddingScale::new();
        p.set_link_scale("arm", 0.0);
        assert_eq!(p.link_scale("arm"), 1.0);
    }

    #[test]
    fn scale_at_epsilon_is_valid() {
        let mut p = LinkPaddingScale::new();
        p.set_link_scale("arm", f64::EPSILON);
        assert_eq!(p.link_scale("arm"), f64::EPSILON);
    }

    #[test]
    fn nan_scale_is_clamped_to_one() {
        let mut p = LinkPaddingScale::new();
        p.set_link_scale("arm", f64::NAN);
        assert_eq!(p.link_scale("arm"), 1.0);
    }

    #[test]
    fn setting_the_same_padding_twice_reports_unchanged_the_second_time() {
        let mut p = LinkPaddingScale::new();
        assert!(p.set_link_padding("arm", 0.1));
        assert!(!p.set_link_padding("arm", 0.1));
    }

    #[test]
    fn with_links_seeds_every_link_with_the_same_values() {
        let p = LinkPaddingScale::with_links(["a".to_string(), "b".to_string()], 0.05, 2.0);
        assert_eq!(p.link_padding("a"), 0.05);
        assert_eq!(p.link_padding("b"), 0.05);
        assert_eq!(p.link_scale("a"), 2.0);
        assert_eq!(p.link_scale("b"), 2.0);
    }

    #[test]
    fn set_padding_for_all_links_only_touches_already_tracked_links() {
        let mut p = LinkPaddingScale::with_links(["a".to_string(), "b".to_string()], 0.0, 1.0);
        let changed = p.set_padding_for_all_links(0.2);
        assert_eq!(changed, vec!["a".to_string(), "b".to_string()]);
        assert_eq!(p.link_padding("a"), 0.2);
        assert_eq!(p.link_padding("c"), 0.0);
    }

    #[test]
    fn a_link_tracked_by_either_setter_is_tracked_by_both_bulk_setters() {
        // "c" is named only to set_link_scale. With padding and scale in
        // separate maps it would be absent from the padding map and so
        // skipped by set_padding_for_all_links, giving "tracked" two
        // different answers depending on which bulk setter asked.
        let mut p = LinkPaddingScale::new();
        p.set_link_scale("c", 2.0);
        assert_eq!(p.set_padding_for_all_links(0.3), vec!["c".to_string()]);
        assert_eq!(p.link_padding("c"), 0.3);
        assert_eq!(p.link_scale("c"), 2.0, "the scale must survive untouched");

        let mut q = LinkPaddingScale::new();
        q.set_link_padding("d", 0.4);
        assert_eq!(q.set_scale_for_all_links(3.0), vec!["d".to_string()]);
        assert_eq!(q.link_scale("d"), 3.0);
        assert_eq!(q.link_padding("d"), 0.4);
    }

    #[test]
    fn link_paddings_and_link_scales_list_the_same_links() {
        let mut p = LinkPaddingScale::with_links(["a".to_string()], 0.1, 1.5);
        p.set_link_scale("b", 2.0);
        p.set_link_padding("c", 0.2);
        let padded: Vec<&str> = p.link_paddings().map(|(name, _)| name).collect();
        let scaled: Vec<&str> = p.link_scales().map(|(name, _)| name).collect();
        assert_eq!(padded, vec!["a", "b", "c"]);
        assert_eq!(scaled, padded);
    }

    #[test]
    fn set_padding_for_all_links_reports_only_the_links_that_changed() {
        let mut p = LinkPaddingScale::with_links(["a".to_string(), "b".to_string()], 0.1, 1.0);
        p.set_link_padding("a", 0.2);
        let changed = p.set_padding_for_all_links(0.2);
        assert_eq!(changed, vec!["b".to_string()]);
    }
}
