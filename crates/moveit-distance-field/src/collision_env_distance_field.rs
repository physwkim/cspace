// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_distance_field/include/moveit/collision_distance_field/collision_env_distance_field.hpp
//   moveit_core/collision_distance_field/src/collision_env_distance_field.cpp

//! The slice of `CollisionEnvDistanceField` this port needs:
//! [`add_link_body_decompositions`] (upstream's two `addLinkBodyDecompositions`
//! overloads), which builds one [`BodyDecomposition`] per robot link with
//! collision geometry, unposed, ready for a `RobotState` to pose later;
//! [`generate_distance_field_cache_entry`] (upstream
//! `generateDistanceFieldCacheEntry`), which builds a
//! [`crate::DistanceFieldCacheEntry`] for one group;
//! [`DistanceFieldCollisionCache`], the persistent cache-owner
//! [`DistanceFieldCollisionCache::generate_collision_checking_structures`]
//! (upstream `generateCollisionCheckingStructures`) needs; and that same
//! type's five collision-check entry points -- `check_self_collision`,
//! `check_collision`, `check_robot_collision`, `get_collision_gradients`,
//! `get_all_collisions` -- see this module's doc comment for the cache
//! type's design and the "Round 21" section below for the entry points'
//! difference table against their seven upstream call sites.
//!
//! # Scope: what unblocked this round, and what is still blocked
//!
//! A previous round of this file's own doc comment recorded
//! `DistanceFieldCacheEntry::link_names_` (upstream
//! `getUpdatedLinkModelNames`) as a blocking dependency gap in
//! `moveit-model`. That gap is closed --
//! [`moveit_model::JointModelGroup::updated_link_names`]/
//! [`moveit_model::JointModelGroup::updated_link_with_geometry_names`] now
//! exist, oracle-verified -- which unblocks
//! [`generate_distance_field_cache_entry`] and the
//! [`crate::DistanceFieldCacheEntry`] struct itself (in
//! `collision_common_distance_field.rs`; see its module doc for the type).
//!
//! This round additionally lands [`compare_cache_entry_to_state`]/
//! [`compare_cache_entry_to_allowed_collision_matrix`] (upstream
//! `compareCacheEntryToState`/`compareCacheEntryToAllowedCollisionMatrix`):
//! a previous round grouped these with `getDistanceFieldCacheEntry` as all
//! alike blocked on the not-ported `CollisionEnvDistanceField` cache member,
//! but neither actually touches it -- both take only a
//! [`crate::DistanceFieldCacheEntry`] (already ported) plus a `RobotState`/
//! `AllowedCollisionMatrix` (already available), so both are free functions
//! here rather than methods needing that type's cache field or
//! construction-time state. `p1-fixtures` has since landed
//! `moveit-scene::AttachedBody`, but it remains unreachable from here: it is
//! tracked on `moveit_scene::PlanningScene`, not on `moveit_state::RobotState`
//! (that crate's own doc still lists "no attached bodies" under deferred
//! scope, deliberately, so `PlanningScene` stays the sole owner -- see
//! `moveit-scene`'s `attached_body` module doc), and
//! [`generate_distance_field_cache_entry`] takes a bare `RobotState`, not a
//! `PlanningScene`.
//!
//! **Stale as of a later round, corrected here:** the paragraph above used
//! to end by claiming upstream's attached-body comparison stays "vacuously
//! true" in this port, since a bare `RobotState` has no attached bodies to
//! compare. That was true only as long as [`crate::DistanceFieldCacheEntry`]'s
//! own attached-body fields stayed permanently empty. They no longer do:
//! [`compare_cache_entry_to_state`]'s own signature grew a
//! `current_attached_bodies: &[AttachedBodyGeometry<'_>]` parameter (see its
//! own doc comment) precisely so the comparison could become real, sourced
//! from the caller-supplied [`AttachedBodyGeometry`] slice this crate
//! threads through every collision-checking entry point instead of reading
//! attached bodies off a `RobotState`/`PlanningScene` this crate does not
//! carry them on -- see [`AttachedBodySnapshot`]'s own doc comment for why
//! that parameter closes a real cache-invalidation gap, not a cosmetic one.
//! Round 22 extends the same explicit-parameter pattern to
//! `getAttachedBodySphereDecomposition`/`getAttachedBodyPointDecomposition`
//! themselves (see "Still blocked, and why" below, now "Round 22" instead).
//!
//! # `generateCollisionCheckingStructures` and its cache-owner type
//!
//! This round lands [`DistanceFieldCollisionCache`] and its
//! [`DistanceFieldCollisionCache::generate_collision_checking_structures`]
//! (upstream `generateCollisionCheckingStructures`) -- the last unported
//! piece of this sub-package. Its body
//! (`collision_env_distance_field.cpp:158-175`) is short enough to quote in
//! full, read fresh against upstream's actual usage rather than assumed from
//! a previous round's "still blocked" grouping:
//!
//! ```text
//! DistanceFieldCacheEntryConstPtr dfce = getDistanceFieldCacheEntry(group_name, state, acm);
//! if (!dfce || (generate_distance_field && !dfce->distance_field_)) {
//!   DistanceFieldCacheEntryPtr new_dfce =
//!       generateDistanceFieldCacheEntry(group_name, state, acm, generate_distance_field);
//!   std::scoped_lock slock(update_cache_lock_);
//!   distance_field_cache_entry_ = new_dfce;
//!   dfce = new_dfce;
//! }
//! getGroupStateRepresentation(dfce, state, gsr);
//! ```
//!
//! Every read in that body -- `resolution_`, `link_body_decomposition_index_map_`,
//! and everything else [`generate_distance_field_cache_entry`]/
//! [`group_state_representation`] themselves read off
//! `CollisionEnvDistanceField` -- is already an explicit parameter of one of
//! those two already-ported free functions, or of [`get_distance_field_cache_entry`]
//! (the third function this body calls). Exactly one thing in this body is
//! *not* already a parameter anywhere: `distance_field_cache_entry_` itself,
//! read at the top and conditionally overwritten at the bottom. That is the
//! whole of "owns state across calls" this function needs a type for; every
//! other read is "is merely where upstream put the function", already
//! solved by this file's existing free functions taking that state as an
//! argument instead of a `self` field. `update_cache_lock_` is a
//! `const`-method workaround for mutating `distance_field_cache_entry_`
//! through a `const_cast` (upstream's `checkCollision`/`checkSelfCollision`
//! need to stay `const` while still caching); a `&mut self` method gives the
//! same single-writer guarantee at compile time, so there is no mutex field
//! to port.
//!
//! [`DistanceFieldCollisionCache`] is that one field, plus the two
//! construction-time inputs every call needs regardless of a cache hit or
//! miss (the return value of [`add_link_body_decompositions`], and a
//! [`DistanceFieldConfig`]) -- not a port of `CollisionEnvDistanceField`
//! itself. Upstream's own call sites confirm the narrower type is correct,
//! not merely convenient: `checkSelfCollisionHelper` and the six other
//! `check*`/`distance*` callers (`collision_env_distance_field.cpp:185`,
//! `1395`, `1428`, `1461`, `1490`, `1526`, `1547`) all guard the call the
//! same way -- `if (!gsr) generateCollisionCheckingStructures(...); else
//! updateGroupStateRepresentationState(...)` -- so the *caller*, not this
//! function, decides whether the cache is even consulted, and
//! `updateGroupStateRepresentationState` (already ported, and itself
//! stateless) needs nothing from `CollisionEnvDistanceField` when that
//! branch is taken instead. `in_group_update_map_` and
//! `pregenerated_group_state_representation_map_` -- upstream members this
//! function's own body never touches -- correspondingly have no field here
//! either: the first is computed inline per call by
//! [`generate_distance_field_cache_entry`] rather than cached (see that
//! function's own doc comment), and the second cannot be read by any
//! [`group_state_representation`] call this port can reach (see that
//! function's "Deviations from upstream").
//!
//! Still blocked, and why:
//!
//! - **`getPosedLinkBodySphereDecomposition`/`getPosedLinkBodyPointDecomposition`.**
//!   Trivial one-line wrappers; now that their only upstream callers
//!   ([`group_state_representation`] and this file's own
//!   `build_non_group_distance_field`) are ported, both inline the
//!   equivalent `PosedBodySphereDecomposition`/`PosedBodyPointDecomposition`
//!   constructor call directly rather than through a same-crate,
//!   single-call-site wrapper -- see those functions' own doc comments.
//!
//! **Round 22: `AttachedBody`-dependent methods, no longer blocked.**
//! `getAttachedBodySphereDecomposition`/`getAttachedBodyPointDecomposition`
//! (in `collision_common_distance_field.cpp`) used to be listed above as
//! blocked on the same "`AttachedBody` lives on `PlanningScene`, unreachable
//! from a bare `RobotState`" premise as [`compare_cache_entry_to_state`]'s
//! own attached-body comparison (see the "Scope" section above) -- true, but
//! it turned out irrelevant: this crate does not need a `RobotState` to
//! *see* attached bodies at all, since [`AttachedBodyGeometry`] already
//! threads them through every collision-checking entry point as an explicit
//! caller-supplied parameter, the same pattern
//! [`compare_cache_entry_to_state`] already used. Round 22 ports both
//! functions as [`attached_body_sphere_decomposition`]/
//! [`attached_body_point_decomposition`] in
//! `collision_common_distance_field.rs` against that same parameter, and
//! wires their sole upstream callers -- `getGroupStateRepresentation`
//! (`:1239`) and `generateDistanceFieldCacheEntry`'s non-group-link loop
//! (`:928`) -- into [`group_state_representation`] and the private
//! `build_non_group_distance_field` respectively, plus the
//! [`generate_distance_field_cache_entry`] population loop that decides
//! which attached bodies belong to which (`collision_env_distance_field.cpp:775`
//! `if (acm)`, population at `:798`, its `else` at `:825` -- round 26: the
//! upstream-absence audit found this citation previously pointed at
//! `:844-919`, an unrelated later block; corrected here and at every other
//! site in this file citing the same wrong range).
//! See `collision_common_distance_field.rs`'s own "Round 22" doc section for
//! the two functions themselves.
//!
//! # Round 21: the seven collision-check entry points
//!
//! `checkSelfCollisionHelper` (`:185`) and the six `check*`/`getCollisionGradients`/
//! `getAllCollisions` bodies that guard a `gsr` in/out parameter the same way
//! (`if (!gsr) generateCollisionCheckingStructures(...); else
//! updateGroupStateRepresentationState(...)`, at the `if (!gsr)` lines
//! `:1395`, `:1428`, `:1461`, `:1490`, `:1526`, `:1547`) are now ported, as
//! [`DistanceFieldCollisionCache::check_self_collision`]/
//! [`DistanceFieldCollisionCache::check_collision`]/
//! [`DistanceFieldCollisionCache::check_robot_collision`]/
//! [`DistanceFieldCollisionCache::get_collision_gradients`]/
//! [`DistanceFieldCollisionCache::get_all_collisions`]. Sharing a common
//! guard shape is not the same claim as sharing a body -- each of the seven
//! was read and ported individually; the table below is the difference,
//! not the resemblance.
//!
//! | Upstream site | Port | `generate_distance_field` | Collision phases run | Difference from a plain read of the shared guard |
//! |---|---|---|---|---|
//! | `checkSelfCollisionHelper` (`:177-198`) | [`check_self_collision`](DistanceFieldCollisionCache::check_self_collision) | always `true` | self, then (if `!done`) intra-group | 차이 없음 -- matches the guard shape exactly; no environment phase exists for self-collision. |
//! | `checkCollision(req, res, state[, gsr])` (`:1389-1411`) | [`check_collision`](DistanceFieldCollisionCache::check_collision) | always `true` | self -> (if `!done`) intra-group -> (if `!done`) environment | 차이 없음 -- both the no-acm and acm-taking `gsr` bodies (`:1395`, `:1428`) pass `true`; no asymmetry between them, unlike `checkRobotCollision` below. |
//! | `checkCollision(req, res, state, acm[, gsr])` (`:1414-1443`) | [`check_collision`](DistanceFieldCollisionCache::check_collision) (`acm: Some(..)`) | always `true` | self -> (if `!done`) intra-group -> (if `!done`) environment | 차이 없음 -- identical body to the no-acm overload above except the `acm` argument threaded into `generateCollisionCheckingStructures`; this port already collapses that into one `acm: Option<&AllowedCollisionMatrix>` parameter. |
//! | `checkRobotCollision(req, res, state[, gsr])` (`:1447-1470`) | [`check_robot_collision`](DistanceFieldCollisionCache::check_robot_collision) (`acm: None`) | `false` | environment only | **Real difference, preserved:** no self/intra-group phase at all, and `generate_distance_field = false` here vs. `true` on the acm overload -- see that method's own "Deviation from upstream" doc for why this is kept observable only in what gets cached, not in this call's own result. |
//! | `checkRobotCollision(req, res, state, acm[, gsr])` (`:1473-1499`) | [`check_robot_collision`](DistanceFieldCollisionCache::check_robot_collision) (`acm: Some(..)`) | `true` | environment only | Same phase set as the no-acm overload; only `generate_distance_field` differs, preserved via `acm.is_some()`. |
//! | `getCollisionGradients` (`:1517-1538`) | [`get_collision_gradients`](DistanceFieldCollisionCache::get_collision_gradients) | always `true` | self, intra-group, environment **proximity gradients**, unconditionally (no `done`/early-exit at all -- gradient computation, not a yes/no check) | **Real difference:** computes gradients, not collisions; upstream's own `res` parameter is `/*res*/`, never read or written, so this port has no `res` in/out at all -- see that method's own doc. |
//! | `getAllCollisions` (`:1540-1559`) | [`get_all_collisions`](DistanceFieldCollisionCache::get_all_collisions) | always `true` | self, intra-group, environment, **all three unconditionally** | **Real difference:** unlike `checkCollision`, upstream's own body has no `if (!done)` guard between phases here -- every phase runs and every return value is discarded. Ported as-is; see that method's own doc. |
//!
//! Two upstream `checkRobotCollision` overloads (`:1502-1515`) take a second
//! `RobotState` for continuous checking; both are stubbed upstream to
//! `RCLCPP_ERROR(logger_, "Continuous collision checking not implemented")`
//! and return without checking anything. Not ported -- see
//! [`DistanceFieldCollisionCache::check_robot_collision`]'s own doc comment.
//!
//! **Combinations that do not exist upstream, so are not invented here:**
//! there is no self-collision overload that takes an `env_distance_field`
//! (self-collision never touches the environment); there is no
//! `checkRobotCollision` overload that runs a self or intra-group phase
//! (`checkRobotCollision` only ever calls `getEnvironmentCollisions`); and
//! the `distance` axis (`distanceSelf`/`distanceRobot`, four overloads plus
//! two `DistanceRequest`-taking overrides, all header-inline in
//! `collision_env_distance_field.hpp:109-199`) never reaches
//! `generateCollisionCheckingStructures`/`gsr` at all -- every `distanceSelf`/
//! `distanceRobot` overload either unconditionally returns `0.0` or logs
//! `RCLCPP_ERROR(logger_, "Not implemented")`, with no cache read, no
//! self/intra-group/environment phase, nothing this port's `gsr`-shaped
//! functions would have anything to share with. None of the seven ported
//! entry points is a "distance" check in that sense; the "check*/distance*"
//! grouping in this section's own name is upstream's naming convention, not
//! a claim that a `distance*` counterpart to `check_self_collision`/
//! `check_collision`/`check_robot_collision` exists to port.
//!
//! The `gsr: GroupStateRepresentationPtr&` in/out parameter every one of the
//! seven upstream bodies threads through is not ported as
//! `&mut Option<GroupStateRepresentation>` -- every one of the five new
//! methods above returns an owned `(CollisionResult,
//! GroupStateRepresentation)` (or bare `GroupStateRepresentation` for
//! [`get_collision_gradients`](DistanceFieldCollisionCache::get_collision_gradients),
//! whose upstream `res` is vestigial) instead. See
//! [`DistanceFieldCollisionCache::check_self_collision`]'s own doc comment
//! for the full reasoning: [`GroupStateRepresentation::dfce`] is a genuine
//! borrow into `self.cache_entry`, which makes a caller holding a `gsr` from
//! one call across a *second* `&mut self` call a hard borrow conflict, not a
//! stylistic one -- so the cross-call reuse half of upstream's nullable
//! `gsr` cannot be honored by any signature shaped
//! `fn(&mut self, ..., &mut Option<GroupStateRepresentation<'s, 'm>>)`, and a
//! plain owned return is what survives of it.
//!
//! One upstream nuance with no analog here: `checkSelfCollision`'s
//! fourth overload (`:260-272`, taking both `acm` and a caller-owned `gsr`)
//! logs `RCLCPP_WARN(logger_, "Shouldn't be calling this function with
//! initialized gsr - ACM will be ignored")` when `gsr` is already
//! populated on entry, because `checkSelfCollisionHelper`'s `acm` argument
//! only reaches `generateCollisionCheckingStructures`'s rebuild branch, not
//! the `updateGroupStateRepresentationState` reuse branch a non-null `gsr`
//! takes instead. Since this port has no caller-owned `gsr` parameter for a
//! second call to arrive already-populated, this warning's situation cannot
//! occur here at all -- not preserved, because there is nothing left for it
//! to warn about.
//!
//! The rest of `CollisionEnvDistanceField` -- `createCollisionModelMarker`
//! (draws `visualization_msgs::msg::MarkerArray`, out of scope under
//! PORTING-PLAN.md D1 same as `getBodySphereVisualizationMarkers`), the
//! `distanceSelf`/`distanceRobot` stubs described above, `updateDistanceObject`,
//! `generateDistanceFieldCacheEntryWorld`, `notifyObjectChange`, and the
//! `CollisionEnvDistanceField` type itself (a `CollisionEnv` implementor
//! wrapping a `World` observer this crate still has no counterpart for --
//! `moveit-collision`'s own `World` doc explicitly documents "No
//! addObserver/removeObserver/notify" as a deliberate, still-current
//! omission) -- remains out of scope regardless. Round 26: the
//! upstream-absence audit found the prior text here also claimed a
//! `planning_scene::PlanningScene` counterpart didn't exist in this
//! workspace either; `moveit_scene::PlanningScene` (referenced two
//! paragraphs up) landed before this file's own round-22 commits and does
//! exist -- `CollisionEnvDistanceField` staying unported is a real,
//! separate gap (the `CollisionEnv`/`World`-observer wiring, not
//! `PlanningScene` itself), not evidence the workspace lacks the type.
//! ("do not try to land all of it in one round") and is not addressed here.
//! [`DistanceFieldCollisionCache`] is not that type and does not become it:
//! it owns only the one cache slot `generateCollisionCheckingStructures`
//! itself reads and writes, none of the World/`PlanningScene`/observer state
//! the excluded methods above need.
//!
//! Upstream's own `test_collision_distance_field.cpp` -- read in full to
//! look for a ground truth for this round's new function too -- does not
//! help narrow this further: every one of its `TEST_F` cases calls
//! `checkSelfCollision` or `checkRobotCollision`, none exercise
//! `addLinkBodyDecompositions` or `DistanceFieldCacheEntry` construction in
//! isolation. The oracle op `distance_field_cache_entry` drives
//! `generateDistanceFieldCacheEntry` indirectly instead, through the same
//! public `checkSelfCollision` -> `getLastDistanceFieldEntry()` path those
//! upstream tests use (see the oracle's own doc comment on that op for
//! why); the tests below are oracle-driven only, matching this crate's
//! practice for the other two files in this sub-package
//! (`collision_distance_field_types`, `collision_common_distance_field`)
//! where upstream's own suite does not reach the ported slice directly.
//!
//! # `addLinkBodyDecompositions`'s two overloads, as one function
//!
//! Upstream declares two independent, near-duplicate overloads:
//! `addLinkBodyDecompositions(resolution)` and
//! `addLinkBodyDecompositions(resolution, link_spheres)`, the second
//! additionally calling `BodyDecomposition::replaceCollisionSpheres` per
//! link found in `link_spheres`. Rust has no overloading; this port
//! collapses them into [`add_link_body_decompositions`] with
//! `link_spheres_override: Option<&HashMap<String, Vec<CollisionSphere>>>`
//! -- `None` behaves exactly like the first overload, `Some` exactly like
//! the second. This changes nothing observable, matching how this crate
//! already collapsed `BodyDecomposition`'s own two constructors behind one
//! `padding` parameter.
//!
//! `getLinkModelsWithCollisionGeometry()` (the robot-wide link set both
//! overloads iterate) is not a `RobotModel` method in this workspace either
//! -- `moveit-collision`'s `LinkPaddingScale` documents the same absence and
//! takes that link list as a caller-supplied argument for the same reason
//! (see its own deviation note). Unlike `getUpdatedLinkModelNames`, this is
//! a one-line filter over already-fully-public data
//! (`RobotModel::link_models()` where `!LinkModel::shapes().is_empty()`,
//! confirmed against upstream's own construction-time filter in
//! `robot_model.cpp`: `if (!link->getShapes().empty()) { ... }`), not a
//! nontrivial graph traversal another crate's own doc comment claims
//! ownership of -- so this port computes it locally rather than reporting
//! it as a blocker.
//!
//! That local filter now reaches byte-exact parity with the live oracle on
//! `pr2.urdf`: pr2's 18 `<collision>` mesh files are committed under
//! `fixtures/meshes/pr2_description/` (landed by p3-acm; see
//! `tools/ci/verify-fixture-provenance.sh`), the same way panda's and
//! fanuc's are, so `collision_env_distance_field_parity.rs`'s
//! `link_models_with_collision_geometry_matches_the_oracle` builds its model
//! with real mesh geometry and compares the computed link set against the
//! oracle's by plain equality, no per-link mesh-gap narrowing -- the earlier
//! version of this paragraph described a fixture-copy gap that closed before
//! this round and has been removed rather than left to go stale again.
//!
//! This file's own `pr2_model()` test helper below still builds with
//! [`moveit_model::MeshSearchPaths::none`], but for an unrelated reason, not
//! a residual fixture gap: every test that uses it
//! (`only_links_with_shapes_get_a_decomposition`,
//! `link_spheres_override_replaces_the_computed_spheres_for_that_link_only`)
//! checks a structural correlation ("a link has a decomposition iff it has
//! shapes", "overriding one link's spheres leaves every other link's alone")
//! that holds regardless of which links carry geometry, so loading real
//! meshes would add test cost (real STL parsing) without changing what
//! either test can prove.

use std::collections::{HashMap, HashSet};
use std::sync::Arc;

use moveit_collision::{
    AllowedCollisionMatrix, AllowedCollisionType, AttachedBodyGeometry, BodyType, CollisionRequest,
    CollisionResult, Contact, ContactData, LinkPaddingScale,
};
use moveit_error::{Error, Result};
use moveit_geometry::Isometry3;
use moveit_model::RobotModel;
use moveit_state::Posed;
use nalgebra::Vector3;

use crate::collision_common_distance_field::{
    AttachedBodySnapshot, DistanceFieldCacheEntry, GroupStateRepresentation,
    attached_body_point_decomposition, attached_body_sphere_decomposition,
};
use crate::collision_distance_field_types::{
    BodyDecomposition, CollisionSphere, CollisionType, GradientInfo, PosedBodyPointDecomposition,
    PosedBodySphereDecomposition, PosedDistanceField, SphereGradientQuery,
    do_bounding_spheres_intersect, get_collision_sphere_collision, get_collision_sphere_collisions,
    get_collision_sphere_gradients,
};
use crate::{DistanceField, GridGeometry, PropagationDistanceField};

/// Upstream's `link_body_decomposition_vector_` paired with
/// `link_body_decomposition_index_map_`: every link's [`BodyDecomposition`],
/// in `RobotModel::link_models()` order, and a name-to-index lookup into it.
///
/// `pub(crate)`, not private: [`crate::HybridCollisionEnv::new`] takes this
/// same type as a constructor parameter, matching upstream
/// `CollisionEnvHybrid`'s constructor, which takes the identical
/// `link_body_decompositions` map upstream `CollisionEnvDistanceField`'s own
/// constructor does.
pub(crate) type LinkBodyDecompositions = (Vec<Arc<BodyDecomposition>>, HashMap<String, usize>);

/// Upstream's two `CollisionEnvDistanceField::addLinkBodyDecompositions`
/// overloads (see this module's doc comment for why they are one function
/// here). Builds one [`BodyDecomposition`] per link in `robot_model` that
/// has collision geometry, from *all* of that link's shapes together (not
/// per-shape, unlike [`crate::get_body_decomposition_cache_entry`]), padded
/// per [`LinkPaddingScale::link_padding`] (`0.0` for an untracked link,
/// matching upstream's `getLinkPadding` default) rather than
/// [`BodyDecomposition::DEFAULT_PADDING`].
///
/// Returns the decompositions in the same order as `robot_model.link_models()`
/// (matching upstream's `link_body_decomposition_vector_`, itself built in
/// `RobotModel`'s own construction-time link order -- see this module's doc
/// comment) alongside a name-to-index map (upstream's
/// `link_body_decomposition_index_map_`).
///
/// # Errors
///
/// See [`BodyDecomposition::from_shapes`].
pub fn add_link_body_decompositions(
    robot_model: &RobotModel,
    resolution: f64,
    link_padding: &LinkPaddingScale,
    link_spheres_override: Option<&HashMap<String, Vec<CollisionSphere>>>,
) -> Result<LinkBodyDecompositions> {
    let mut link_body_decomposition_vector = Vec::new();
    let mut link_body_decomposition_index_map = HashMap::new();

    for link_model in robot_model.link_models() {
        if link_model.shapes().is_empty() {
            continue;
        }

        let shapes: Vec<_> = link_model
            .shapes()
            .iter()
            .map(|link_shape| link_shape.shape.clone())
            .collect();
        let poses: Vec<_> = link_model
            .shapes()
            .iter()
            .map(|link_shape| link_shape.origin_transform)
            .collect();
        let mut decomposition = BodyDecomposition::from_shapes(
            &shapes,
            &poses,
            resolution,
            link_padding.link_padding(link_model.name()),
        )?;

        if let Some(overrides) = link_spheres_override {
            if let Some(spheres) = overrides.get(link_model.name()) {
                decomposition.replace_collision_spheres(spheres.clone(), Isometry3::identity());
            }
        }

        link_body_decomposition_vector.push(Arc::new(decomposition));
        link_body_decomposition_index_map.insert(
            link_model.name().to_string(),
            link_body_decomposition_vector.len() - 1,
        );
    }

    Ok((
        link_body_decomposition_vector,
        link_body_decomposition_index_map,
    ))
}

/// The distance-field-construction parameters
/// [`generate_distance_field_cache_entry`] needs only when its caller wants
/// a real [`PropagationDistanceField`] built -- upstream's
/// `generate_distance_field` `bool` plus the `size_`/`origin_`/`resolution_`/
/// `max_propogation_distance_`/`use_signed_distance_field_` members
/// `CollisionEnvDistanceField` would otherwise already carry from its own
/// construction. Bundled into one struct (matching how
/// [`PropagationDistanceField::new`] itself already bundles `size`/`origin`/
/// `resolution` into [`GridGeometry`]) both to stay under this crate's
/// `clippy::too_many_arguments` budget and because these five values are
/// only ever meaningful together. `None` in
/// [`generate_distance_field_cache_entry`]'s own `generate_distance_field`
/// parameter plays the role of upstream's `generate_distance_field = false`.
/// `Copy` so [`DistanceFieldCollisionCache`] can hand a value out to
/// [`generate_distance_field_cache_entry`] on every cache-miss call without
/// forcing that call to consume the cache's own copy.
#[derive(Debug, Clone, Copy)]
pub struct DistanceFieldConfig {
    /// Corner-origin grid geometry, matching [`PropagationDistanceField::new`].
    /// Upstream performs a center-to-corner shift (`origin_.x() - 0.5 *
    /// size_.x()`, and the same for `y`/`z`) inline at this exact call site,
    /// sourced from `CollisionEnvDistanceField::origin_`/`size_` (which are
    /// center-origin). Since that struct is not ported (see this module's
    /// doc comment), the shift becomes this field's caller's responsibility
    /// instead of this function's.
    pub geometry: GridGeometry,
    /// `max_propogation_distance_`.
    pub max_propagation_distance: f64,
    /// `use_signed_distance_field_`.
    pub use_signed_distance_field: bool,
}

/// Upstream `CollisionEnvDistanceField::generateDistanceFieldCacheEntry`.
/// Builds a [`DistanceFieldCacheEntry`] for `group_name`: which of the
/// group's updated links have collision geometry, which pairs are exempt
/// from self/intra-group collision checking per `acm`, the joint-variable
/// cache key (`state_check_indices`/`state_values`), and -- when
/// `generate_distance_field` is `Some` -- a real distance field of every
/// *other* link's collision geometry, posed at `state`'s current transforms.
///
/// Upstream reads `robot_model_`/`resolution_`/`link_body_decomposition_index_map_`/
/// `in_group_update_map_` from `CollisionEnvDistanceField`'s own
/// construction-time state; since that type is not ported (see this
/// module's doc comment), every one of those becomes an explicit parameter
/// here instead: `state.model()` for `robot_model_`, `link_body_decompositions`
/// for `link_body_decomposition_index_map_` (see [`add_link_body_decompositions`]),
/// and `state.model().joint_model_group(group_name)?.updated_link_with_geometry_names()`
/// computed inline for `in_group_update_map_.find(group_name)->second` (the
/// two are the exact same set -- `in_group_update_map_` is built once per
/// group, at `CollisionEnvDistanceField` construction, from precisely that
/// query; see `collision_env_distance_field.cpp:141-155`).
///
/// # Deviations from upstream
///
/// - **`link_state_indices` computed directly, not searched.** Upstream
///   searches `state.getJointModelGroup(group_name)->getUpdatedLinkModels()`
///   for each `link_names_[i]` and stores the matching position. Both
///   `getUpdatedLinkModels()` and `getUpdatedLinkModelNames()` are built
///   from the *same* sorted `updated_link_model_vector_`
///   (`joint_model_group.cpp:261-278`), so that search always finds
///   `link_name` at position `i` itself -- there is no input on which the
///   search and a direct `i` could disagree. This port computes
///   `link_state_indices[i] = i` directly instead of re-deriving an
///   invariant result via a linear search.
/// - **The "already has a field" skip is not ported.** Upstream's
///   `generate_distance_field` branch checks `if (dfce->distance_field_) {
///   skip } else { build }`, but `dfce` is a freshly-`make_shared`d struct a
///   few lines above with `distance_field_` still null -- that check can
///   never take the "skip" branch from within this function. This port
///   builds the field unconditionally when `generate_distance_field` is
///   `Some`, matching the only behavior upstream's own dead branch allows.
/// - **The "no link state found" skip is not ported**, for the same reason:
///   it guards against `getUpdatedLinkModels()` not containing a name from
///   `link_names_`, which -- per the same-sorted-vector fact above -- cannot
///   happen.
/// - **`attached_bodies` is a new field with no upstream member behind it.**
///   Upstream re-derives its own attached-body comparison in
///   `compareCacheEntryToState` on demand from `dfce->state_->getAttachedBodies()`,
///   which needs no capture step here (`state_` is a full `RobotStatePtr`
///   snapshot upstream). This port's [`DistanceFieldCacheEntry::state`]
///   cannot answer that query at all -- see [`AttachedBodySnapshot`]'s doc
///   comment -- so `attached_bodies` captures an owned snapshot of `state`'s
///   attached bodies at generation time instead, from the `attached_bodies`
///   parameter below.
/// - **`attached_body_names`/`attached_body_link_state_indices` (round 22)
///   are real, not always-empty** -- ported faithfully from upstream's own
///   per-link loop, including the upstream quirk that this population only
///   runs when `acm` is `Some` (see this function's body for the `rg`-cited
///   line numbers): an `acm: None` call never sees any attached body, by
///   upstream's own construction, not a gap this port introduces.
///
/// # Errors
///
/// [`moveit_error::Error::UnknownName`] if `group_name` does not name a
/// group in `state`'s model. See [`PropagationDistanceField::new`] for
/// errors from building the distance field.
pub fn generate_distance_field_cache_entry<'m>(
    group_name: &str,
    state: &Posed<'_, 'm>,
    acm: Option<&AllowedCollisionMatrix>,
    link_body_decompositions: &LinkBodyDecompositions,
    generate_distance_field: Option<DistanceFieldConfig>,
    attached_bodies: &[AttachedBodyGeometry<'_>],
) -> Result<DistanceFieldCacheEntry<'m>> {
    let model = state.model();
    let group = model.joint_model_group(group_name)?;
    let (link_body_decomposition_vector, link_body_decomposition_index_map) =
        link_body_decompositions;

    let link_indices = group.updated_link_indices();
    let link_names: Vec<String> = group.updated_link_names().to_vec();
    let total = link_names.len() + attached_bodies.len();

    let mut link_has_geometry = Vec::with_capacity(link_names.len());
    let mut link_body_indices = Vec::with_capacity(link_names.len());
    let mut link_state_indices = Vec::with_capacity(link_names.len());
    let mut self_collision_enabled = vec![true; total];
    let mut intra_group_collision_enabled = vec![Vec::new(); total];
    let mut attached_body_names: Vec<String> = Vec::new();
    let mut attached_body_link_state_indices: Vec<usize> = Vec::new();

    for (i, (&link_index, link_name)) in link_indices.iter().zip(link_names.iter()).enumerate() {
        let link = model.link_model_at(link_index);
        // See this function's "Deviations from upstream": always `i` itself.
        link_state_indices.push(i);

        if !link.shapes().is_empty() {
            link_has_geometry.push(true);
            let body_index = *link_body_decomposition_index_map
                .get(link_name)
                .expect("an updated link with geometry always has a body decomposition");
            link_body_indices.push(body_index);

            let mut row = vec![true; total];
            if let Some(acm) = acm {
                if is_always_allowed(acm, link_name, link_name) {
                    self_collision_enabled[i] = false;
                }
                for (j, other) in link_names.iter().enumerate().skip(i + 1) {
                    if other == link_name {
                        row[j] = false;
                        continue;
                    }
                    if is_always_allowed(acm, link_name, other) {
                        row[j] = false;
                    }
                }
                // Upstream populates `attached_body_names_` here too, but
                // only inside this `if (acm)` branch
                // (`collision_env_distance_field.cpp:775` vs its `else` at
                // `:825`) -- when no ACM is supplied, upstream never
                // enumerates attached bodies at all, so
                // `attached_body_names_` (and every attached body's
                // decomposition downstream) stays empty whenever `acm` is
                // `None`. Faithfully reproduced, not "fixed": see this
                // function's own "Deviations from upstream".
                for attached in attached_bodies
                    .iter()
                    .filter(|ab| ab.link_name == link_name.as_str())
                {
                    attached_body_names.push(attached.id.to_string());
                    attached_body_link_state_indices.push(i);
                    let att_index = link_names.len() + attached_body_names.len() - 1;
                    if is_always_allowed(acm, link_name, attached.id) {
                        row[att_index] = false;
                    }
                    // Touch links take priority over the ACM entry.
                    if attached.touch_links.contains(link_name) {
                        row[att_index] = false;
                    }
                }
            }
            intra_group_collision_enabled[i] = row;
        } else {
            link_has_geometry.push(false);
            link_body_indices.push(0);
            self_collision_enabled[i] = false;
            intra_group_collision_enabled[i] = vec![false; total];
        }
    }

    for i in 0..attached_body_names.len() {
        let row_index = link_names.len() + i;
        let mut row = vec![true; total];
        if let Some(acm) = acm {
            if is_always_allowed(acm, &attached_body_names[i], &attached_body_names[i]) {
                self_collision_enabled[row_index] = false;
            }
            for (j, other) in attached_body_names.iter().enumerate().skip(i + 1) {
                if is_always_allowed(acm, &attached_body_names[i], other) {
                    row[link_names.len() + j] = false;
                }
            }
        }
        intra_group_collision_enabled[row_index] = row;
    }

    let active_variable_names: HashSet<&str> = group
        .active_joint_indices()
        .iter()
        .flat_map(|&joint_index| model.joint_model_at(joint_index).variable_names())
        .map(String::as_str)
        .collect();

    let mut state_values = Vec::with_capacity(model.variable_names().len());
    let mut state_check_indices = Vec::new();
    for name in model.variable_names() {
        state_values.push(state.variable_position(name)?);
        if !active_variable_names.contains(name.as_str()) {
            state_check_indices.push(state_values.len() - 1);
        }
    }

    let distance_field = generate_distance_field
        .map(|config| {
            build_non_group_distance_field(
                model,
                state,
                link_names.iter().map(String::as_str),
                link_body_decomposition_vector,
                link_body_decomposition_index_map,
                attached_bodies,
                config,
            )
        })
        .transpose()?;

    Ok(DistanceFieldCacheEntry {
        group_name: group_name.to_string(),
        state: (**state).clone(),
        state_check_indices,
        state_values,
        acm: acm.cloned().unwrap_or_default(),
        distance_field,
        link_names,
        link_has_geometry,
        link_body_indices,
        link_state_indices,
        attached_body_names,
        attached_body_link_state_indices,
        attached_bodies: attached_bodies
            .iter()
            .map(AttachedBodySnapshot::from_geometry)
            .collect(),
        self_collision_enabled,
        intra_group_collision_enabled,
    })
}

/// `acm.getEntry(a, b, type) && type == AllowedCollision::ALWAYS`.
fn is_always_allowed(acm: &AllowedCollisionMatrix, a: &str, b: &str) -> bool {
    matches!(
        acm.allowed_collision(a, b).map(|entry| entry.kind()),
        Some(AllowedCollisionType::Always)
    )
}

/// Upstream file-local `const double EPSILON = 0.001f;`
/// (`collision_env_distance_field.cpp:55`), the tolerance
/// [`compare_cache_entry_to_state`] allows an out-of-group joint variable to
/// have drifted by before invalidating a cached entry.
const STATE_CHECK_EPSILON: f64 = 0.001;

/// Upstream `CollisionEnvDistanceField::compareCacheEntryToState`. `false`
/// ("regenerate the cache") when `state` carries a different number of
/// variables than `dfce` was generated with, or any variable *outside* the
/// group (`dfce.state_check_indices`) has moved by more than
/// `STATE_CHECK_EPSILON` (`0.001`, matching upstream's own file-local
/// `EPSILON`) since `dfce` was generated. See
/// `collision_common_distance_field.rs`'s "`compareCacheEntryToState`'s
/// cache-key semantics" doc section for what this actually checks and why.
///
/// Upstream also compares `dfce->state_->getAttachedBodies()` against
/// `state.getAttachedBodies()` (count, then name/touch-links/shapes per
/// body, in order), invalidating the cache on any difference --
/// `current_attached_bodies` plays that role here, compared against
/// [`DistanceFieldCacheEntry::attached_bodies`] (see [`AttachedBodySnapshot`]'s
/// doc comment for why this port needed a real signature change, not a
/// documented deviation, to close this).
pub fn compare_cache_entry_to_state(
    dfce: &DistanceFieldCacheEntry<'_>,
    state: &Posed<'_, '_>,
    current_attached_bodies: &[AttachedBodyGeometry<'_>],
) -> bool {
    let new_state_values = state.positions();
    if dfce.state_values.len() != new_state_values.len() {
        return false;
    }
    if !dfce
        .state_check_indices
        .iter()
        .all(|&i| (dfce.state_values[i] - new_state_values[i]).abs() <= STATE_CHECK_EPSILON)
    {
        return false;
    }
    if dfce.attached_bodies.len() != current_attached_bodies.len() {
        return false;
    }
    dfce.attached_bodies
        .iter()
        .zip(current_attached_bodies)
        .all(|(snapshot, current)| snapshot.matches(current))
}

/// Upstream `CollisionEnvDistanceField::compareCacheEntryToAllowedCollisionMatrix`.
/// `false` ("regenerate the cache") when `acm` has a different number of
/// rows than `dfce.acm`, or when `acm` would compute a different
/// self-collision-enabled or intra-group-collision-enabled bit than `dfce`
/// already cached, for any pair of geometry-bearing links in
/// `dfce.link_names`.
///
/// # Deviation from upstream
///
/// Upstream also collects `dfce->state_->getAttachedBodies()` into a local
/// `attached_bodies` -- but never reads it again afterward in this function
/// (`collision_env_distance_field.cpp:1322-1369`: assigned once, never
/// used). This port omits the equivalent fetch rather than port a call with
/// no observable effect: unlike [`compare_cache_entry_to_state`] (whose
/// attached-body comparison is real, hence takes `current_attached_bodies`
/// -- see [`AttachedBodySnapshot`]'s doc comment), this function's own
/// upstream attached-body fetch has nothing reading it, so there is no
/// signature to change here.
pub fn compare_cache_entry_to_allowed_collision_matrix(
    dfce: &DistanceFieldCacheEntry<'_>,
    acm: &AllowedCollisionMatrix,
) -> bool {
    if dfce.acm.len() != acm.len() {
        return false;
    }
    for (i, link_name) in dfce.link_names.iter().enumerate() {
        if !dfce.link_has_geometry[i] {
            continue;
        }
        let self_collision_enabled = !is_always_allowed(acm, link_name, link_name);
        if self_collision_enabled != dfce.self_collision_enabled[i] {
            return false;
        }
        for (j, other) in dfce.link_names.iter().enumerate().skip(i + 1) {
            if !dfce.link_has_geometry[j] {
                continue;
            }
            let intra_collision_enabled = !is_always_allowed(acm, link_name, other);
            if dfce.intra_group_collision_enabled[i][j] != intra_collision_enabled {
                return false;
            }
        }
    }
    true
}

/// Upstream `CollisionEnvDistanceField::getDistanceFieldCacheEntry`. A pure
/// decision function: reads no persistent state and writes none, so unlike
/// upstream's method (a `const` method that still reads
/// `distance_field_cache_entry_`, a member this crate's not-yet-designed
/// cache-owner type would hold -- see this module's doc comment) this port
/// takes the candidate entry directly as `current` rather than reading it off
/// `self`.
///
/// `None` ("regenerate") when `current` is `None`, names a different group
/// than `group_name`, [`compare_cache_entry_to_state`] rejects `state`/
/// `current_attached_bodies`, or -- when `acm` is `Some` --
/// [`compare_cache_entry_to_allowed_collision_matrix`] rejects it.
/// `Some(current)` (unchanged) otherwise, matching upstream's `return cur;`
/// on every accepting path.
pub fn get_distance_field_cache_entry<'e, 'm>(
    current: Option<&'e DistanceFieldCacheEntry<'m>>,
    group_name: &str,
    state: &Posed<'_, '_>,
    acm: Option<&AllowedCollisionMatrix>,
    current_attached_bodies: &[AttachedBodyGeometry<'_>],
) -> Option<&'e DistanceFieldCacheEntry<'m>> {
    let cur = current?;
    if group_name != cur.group_name {
        return None;
    }
    if !compare_cache_entry_to_state(cur, state, current_attached_bodies) {
        return None;
    }
    if let Some(acm) = acm {
        if !compare_cache_entry_to_allowed_collision_matrix(cur, acm) {
            return None;
        }
    }
    Some(cur)
}

/// Upstream `CollisionEnvDistanceField::getGroupStateRepresentation`'s
/// fresh-build *construction* branch (`!dfce->pregenerated_group_state_representation_`),
/// with the pregenerated branch's `sphere_locations` output reproduced
/// directly rather than through its cache-reuse mechanism -- see this
/// function's "Deviations from upstream" for why the two branches agree on
/// every field value despite that mechanism staying unported. Builds one
/// [`PosedBodySphereDecomposition`]/[`PosedDistanceField`] pair per
/// geometry-bearing link in `dfce.link_names`, posed at `state`'s current
/// global transform for that link, plus a [`GradientInfo`] slot sized to
/// that link's own sphere count.
///
/// Upstream reads `resolution_`/`max_propogation_distance_`/
/// `use_signed_distance_field_` off `CollisionEnvDistanceField`'s own
/// construction-time state (not ported; see this module's doc comment), so
/// this port takes them as explicit parameters instead.
/// `link_body_decomposition_vector` plays the same role for
/// `getPosedLinkBodySphereDecomposition`'s own
/// `link_body_decomposition_vector_.at(ind)` lookup (that helper is a
/// one-line `PosedBodySphereDecomposition::new` wrapper -- see this module's
/// doc comment on why it stays inlined rather than becoming its own
/// function).
///
/// # Deviations from upstream
///
/// - **The "pregenerated" branch's *cache-reuse mechanism* is not ported,
///   because it is provably unreachable here -- but its *observable output*
///   is (round 25).** Upstream populates
///   `dfce->pregenerated_group_state_representation_` in exactly one place,
///   `CollisionEnvDistanceField::initialize` (its constructor path,
///   `collision_env_distance_field.cpp:126-156`): for every joint group, it
///   builds a `DistanceFieldCacheEntry` and immediately calls this same
///   function -- which at that moment still takes the fresh-build branch,
///   since the field is not yet populated -- to seed
///   `pregenerated_group_state_representation_map_`, which a *later*
///   `generateDistanceFieldCacheEntry` call then copies onto the field this
///   branch checks. Consequence: **every call to this function upstream
///   makes *after* construction takes the pregenerated branch**
///   (`:1212-1227`), not the fresh-build branch -- the fresh-build branch
///   only ever runs once per group, inside `initialize()` itself. Earlier
///   revisions of this doc comment treated "fresh-build branch only" as this
///   port's scope and concluded the *value* differences between the two
///   branches were therefore moot; PORTING-PLAN.md §154 measures that
///   conclusion to be wrong for `sphere_locations` specifically -- an oracle
///   fixture committed before the `gradients` field even existed already
///   showed non-empty per-link `sphere_locations`, which only the
///   pregenerated branch's extra line (`:1224`,
///   `gradients_[i].sphere_locations = link_body_decompositions_[i]->getSphereCenters()`,
///   evaluated *after* that call's `updatePose`) can produce.
///
///   `DistanceFieldCacheEntry` in this port (see its own doc comment) still
///   has no `pregenerated_group_state_representation`-equivalent field, and
///   there is still no `initialize`-equivalent eager per-group construction
///   step in this crate's scope to populate one -- see this module's
///   [`DistanceFieldCollisionCache::new`] doc for exactly why that
///   *mechanism* remains unported. But `:1224`'s value does not depend on
///   *which* branch computed it, only on the current pose and the
///   underlying decomposition geometry: both branches source their link
///   decomposition from the same `link_body_decomposition_vector_.at(ind)`
///   (`dfce.link_body_indices[i]` here) and pose it to the same
///   `state.getFrameTransform`/[`Posed::global_link_transform`] before
///   reading centers back out. This port's link loop below already builds
///   that decomposition and poses it in the same statement sequence (there
///   is no separate "already built earlier" object to distinguish it from),
///   so reading `link_bd.sphere_centers()` right after `link_bd.update_pose`
///   yields the identical value the pregenerated branch's own read would --
///   closing the *value* gap directly at its one construction site, without
///   needing the cache-reuse machinery that produces the same value
///   upstream. What upstream's pregenerated branch buys that this still does
///   not reproduce is purely a *performance* difference (reusing an
///   already-built `PosedDistanceField` instead of rebuilding one every
///   call) -- see [`DistanceFieldCollisionCache::new`]'s doc comment and
///   `lib.rs`'s module doc for that remaining, still-open mechanism gap and
///   its own falsifier condition.
/// - **Attached bodies (round 22): upstream's trailing attached-body loop
///   (`collision_env_distance_field.cpp:1229-1251`) is now ported**, using
///   [`attached_body_sphere_decomposition`] to build each attached body's
///   posed decomposition. Upstream runs this loop *unconditionally*, after
///   the fresh-build/pregenerated `if`/`else` closes, so `sphere_locations`
///   (`:1246`) is always set here regardless of branch -- matching the link
///   loop above, which as of round 25 also sets `sphere_locations`
///   unconditionally (see this doc's first bullet); there is no longer a
///   link/attached-body asymmetry to preserve. Each
///   `dfce.attached_body_names[i]` is looked up by id in the caller-supplied
///   `attached_bodies` slice (see this function's own signature) -- the same
///   explicit-parameter pattern this crate already uses everywhere a
///   `RobotState` this crate does not carry attached bodies on would
///   otherwise be needed (see [`AttachedBodyGeometry`]'s own doc comment).
/// - **`gradients[i].gradients` is seeded with `Vector3::zeros()`, not left
///   uninitialized.** Upstream's fresh-build branch writes
///   `gradients_[i].gradients.resize(n)` with no fill argument --
///   `std::vector<Eigen::Vector3d>::resize` default-constructs each new
///   element, and unlike `std::vector<double>` (whose `resize` zero-fills),
///   `Eigen::Vector3d` has no default initializer, so every element is
///   genuinely uninitialized memory. (Confirmed against this crate's own
///   `group_state_representation` oracle fixture: every `distances[i]` reads
///   back as `DBL_MAX`, matching the sibling `resize(n, DBL_MAX)` call one
///   line above it, but the oracle op does not -- cannot -- serialize
///   `gradients_[i]` itself, since there is no defined value to serialize.)
///   Compare [`update_group_state_representation_state`], whose equivalent
///   reset uses `.assign(n, Eigen::Vector3d(0, 0, 0))` -- a real,
///   deterministic zero-fill -- so only this *fresh-build* path has the
///   defect, not the *update* path. Same defect class as
///   [`BodyDecomposition`]'s own `relative_cylinder_pose_`-for-Sphere case
///   (see that type's doc comment): this port seeds `Vector3::zeros()`
///   deterministically rather than reproduce nondeterministic garbage, which
///   is *more* defined than upstream, not less, and therefore not "fixed" to
///   match -- there is no defined upstream value to match.
///
/// # Errors
///
/// [`moveit_error::Error::UnknownName`] if `state`'s model has no link named
/// by some entry of `dfce.link_names` (propagated from
/// [`Posed::global_link_transform`]), or if `attached_bodies` has no entry
/// matching some name in `dfce.attached_body_names`. Upstream's own
/// equivalent lookup, `state.getAttachedBody(dfce->attached_body_names_[i])`
/// (`collision_env_distance_field.cpp:1239`), has *no* null check at all in
/// this fresh-build path -- unlike
/// [`update_group_state_representation_state`]'s own attached-body loop,
/// which does check and log-then-`continue` -- so a name mismatch here
/// upstream dereferences a null pointer, undefined behaviour this port
/// cannot reproduce in safe Rust. A hard error is the closest safe
/// equivalent, not a deviation invented for convenience. See
/// [`PosedDistanceField::new`] for errors building a link's own distance
/// field.
pub fn group_state_representation<'a, 'm>(
    dfce: &'a DistanceFieldCacheEntry<'m>,
    state: &Posed<'_, 'm>,
    link_body_decomposition_vector: &[Arc<BodyDecomposition>],
    resolution: f64,
    max_propagation_distance: f64,
    use_signed_distance_field: bool,
    attached_bodies: &[AttachedBodyGeometry<'_>],
) -> Result<GroupStateRepresentation<'a, 'm>> {
    let mut link_body_decompositions = Vec::with_capacity(dfce.link_names.len());
    let mut link_distance_fields = Vec::with_capacity(dfce.link_names.len());
    let mut gradients = Vec::with_capacity(dfce.link_names.len());

    for (i, link_name) in dfce.link_names.iter().enumerate() {
        if !dfce.link_has_geometry[i] {
            link_body_decompositions.push(None);
            link_distance_fields.push(None);
            gradients.push(GradientInfo::default());
            continue;
        }

        let mut link_bd = PosedBodySphereDecomposition::new(Arc::clone(
            &link_body_decomposition_vector[dfce.link_body_indices[i]],
        ));

        let diameter = 2.0 * link_bd.bounding_sphere_radius();
        let link_size = Vector3::new(diameter, diameter, diameter);
        let link_origin = link_bd.bounding_sphere_center() - 0.5 * link_size;

        let mut field = PosedDistanceField::new(
            link_size,
            link_origin,
            resolution,
            max_propagation_distance,
            use_signed_distance_field,
        )?;
        field
            .field_mut()
            .add_points_to_field(link_bd.collision_points());

        let transform = state.global_link_transform(link_name)?;
        link_bd.update_pose(transform);
        field.update_pose(transform);

        let sphere_count = link_bd.collision_spheres().len();
        let joint_index = state.model().link_model(link_name)?.parent_joint_index();
        gradients.push(GradientInfo {
            types: vec![CollisionType::None; sphere_count],
            distances: vec![f64::MAX; sphere_count],
            gradients: vec![Vector3::zeros(); sphere_count],
            sphere_locations: link_bd.sphere_centers().to_vec(),
            sphere_radii: link_bd.sphere_radii().to_vec(),
            joint_name: state.model().joint_model_at(joint_index).name().to_string(),
            ..GradientInfo::default()
        });
        link_body_decompositions.push(Some(link_bd));
        link_distance_fields.push(Some(field));
    }

    let mut attached_body_decompositions = Vec::with_capacity(dfce.attached_body_names.len());
    for (i, name) in dfce.attached_body_names.iter().enumerate() {
        let link_index = dfce.attached_body_link_state_indices[i];
        let link_name = &dfce.link_names[link_index];
        let attached = attached_bodies
            .iter()
            .find(|ab| ab.id == name.as_str())
            .ok_or_else(|| Error::unknown_name("attached body", name.clone()))?;
        let link_transform = state.global_link_transform(link_name)?;
        let decomposition =
            attached_body_sphere_decomposition(attached, link_transform, resolution)?;

        let sphere_count = decomposition.collision_spheres().len();
        let joint_index = state.model().link_model(link_name)?.parent_joint_index();
        gradients.push(GradientInfo {
            types: vec![CollisionType::None; sphere_count],
            distances: vec![f64::MAX; sphere_count],
            gradients: vec![Vector3::zeros(); sphere_count],
            sphere_locations: decomposition.sphere_centers().to_vec(),
            sphere_radii: decomposition.sphere_radii().to_vec(),
            joint_name: state.model().joint_model_at(joint_index).name().to_string(),
            ..GradientInfo::default()
        });
        attached_body_decompositions.push(decomposition);
    }

    Ok(GroupStateRepresentation {
        dfce,
        link_body_decompositions,
        attached_body_decompositions,
        link_distance_fields,
        gradients,
    })
}

/// Upstream `CollisionEnvDistanceField::updateGroupStateRepresentationState`'s
/// link loop only. Re-poses every geometry-bearing link's decomposition and
/// distance field to `state`'s current global transform, and resets that
/// link's [`GradientInfo`] slot to a fresh, unset state sized to its own
/// sphere count.
///
/// # Deviation from upstream
///
/// Upstream's trailing attached-body loop (round 22) is now ported too --
/// re-posing each attached-body decomposition's shapes and resetting its
/// [`GradientInfo`] slot, mirroring [`group_state_representation`]'s own
/// attached-body build. It also faithfully reproduces upstream's own
/// suspicious count check (`collision_env_distance_field.cpp:1132-1137`,
/// upstream's own comment: `TODO: This logic for checking attached body
/// count might be incorrect`) rather than "fixing" it -- see this function's
/// body for exactly what it compares.
///
/// Unlike [`group_state_representation`]'s fresh-build path, this reset
/// *is* upstream-deterministic: `gradients_[i].gradients.assign(n,
/// Eigen::Vector3d(0.0, 0.0, 0.0))` is a real zero-fill, not a bare `resize`,
/// so `Vector3::zeros()` here matches upstream's actual value rather than
/// substituting for an undefined one.
///
/// # Errors
///
/// [`moveit_error::Error::UnknownName`] if `state`'s model has no link named
/// by some entry of `gsr.dfce.link_names` (propagated from
/// [`Posed::global_link_transform`]), or if `attached_bodies` has no entry
/// matching some name in `gsr.dfce.attached_body_names` -- unlike upstream,
/// whose own null check here logs and `continue`s (this is the one
/// attached-body loop in this module that *does* have a defensive check;
/// compare [`group_state_representation`]'s own "Errors" doc). This port
/// still prefers a hard error over silently skipping, for the same
/// "caller's slice and the cache entry disagree" reasoning, and to keep one
/// error-handling style across this module's two attached-body loops rather
/// than one strict and one lenient for no functional reason.
pub fn update_group_state_representation_state(
    state: &Posed<'_, '_>,
    gsr: &mut GroupStateRepresentation<'_, '_>,
    attached_bodies: &[AttachedBodyGeometry<'_>],
) -> Result<()> {
    for (i, link_name) in gsr.dfce.link_names.iter().enumerate() {
        if !gsr.dfce.link_has_geometry[i] {
            continue;
        }
        let transform = state.global_link_transform(link_name)?;

        let link_bd = gsr.link_body_decompositions[i]
            .as_mut()
            .expect("link_has_geometry[i] implies link_body_decompositions[i] is Some");
        link_bd.update_pose(transform);
        let field = gsr.link_distance_fields[i]
            .as_mut()
            .expect("link_has_geometry[i] implies link_distance_fields[i] is Some");
        field.update_pose(transform);

        let sphere_count = link_bd.collision_spheres().len();
        gsr.gradients[i] = GradientInfo {
            closest_distance: f64::MAX,
            collision: false,
            types: vec![CollisionType::None; sphere_count],
            distances: vec![f64::MAX; sphere_count],
            gradients: vec![Vector3::zeros(); sphere_count],
            sphere_radii: gsr.gradients[i].sphere_radii.clone(),
            joint_name: gsr.gradients[i].joint_name.clone(),
            sphere_locations: link_bd.sphere_centers().to_vec(),
        };
    }

    for (i, name) in gsr.dfce.attached_body_names.iter().enumerate() {
        let link_index = gsr.dfce.attached_body_link_state_indices[i];
        let link_name = &gsr.dfce.link_names[link_index];
        let attached = attached_bodies
            .iter()
            .find(|ab| ab.id == name.as_str())
            .ok_or_else(|| Error::unknown_name("attached body", name.clone()))?;

        // Upstream's own suspicious count check
        // (`collision_env_distance_field.cpp:1132-1137`): compares the
        // *outer* vector's length (total attached body count for this
        // group) against *this one* attached body's shape count, not this
        // decomposition's own shape count against itself. Faithfully
        // reproduced, not fixed -- see this function's doc comment.
        if gsr.attached_body_decompositions.len() != attached.shapes.len() {
            continue;
        }

        let link_transform = state.global_link_transform(link_name)?;
        for (j, shape_pose) in attached.shape_poses.iter().enumerate() {
            gsr.attached_body_decompositions[i].update_pose(j, link_transform * *shape_pose);
        }

        let decomposition = &gsr.attached_body_decompositions[i];
        let sphere_count = decomposition.collision_spheres().len();
        gsr.gradients[i + gsr.dfce.link_names.len()] = GradientInfo {
            closest_distance: f64::MAX,
            collision: false,
            types: vec![CollisionType::None; sphere_count],
            distances: vec![f64::MAX; sphere_count],
            gradients: vec![Vector3::zeros(); sphere_count],
            sphere_locations: decomposition.sphere_centers().to_vec(),
            sphere_radii: decomposition.sphere_radii().to_vec(),
            joint_name: gsr.gradients[i + gsr.dfce.link_names.len()]
                .joint_name
                .clone(),
        };
    }
    Ok(())
}

/// Upstream `CollisionEnvDistanceField`'s persistent cache slot
/// (`distance_field_cache_entry_`) plus the two construction-time inputs
/// every [`DistanceFieldCollisionCache::generate_collision_checking_structures`]
/// call needs regardless of a cache hit or miss -- not
/// `CollisionEnvDistanceField` itself. See this module's doc comment for why
/// the narrower scope is the one upstream's own call sites actually need.
pub struct DistanceFieldCollisionCache<'m> {
    link_body_decompositions: LinkBodyDecompositions,
    distance_field_config: DistanceFieldConfig,
    /// `collision_tolerance_`. Upstream `DEFAULT_COLLISION_TOLERANCE = 0.0`
    /// (`collision_env_distance_field.hpp:54`). Unlike
    /// [`DistanceFieldConfig`]'s five fields (only ever meaningful together,
    /// at distance-field *construction* time -- see that type's own doc),
    /// this is a collision-*checking* parameter read only by
    /// [`get_self_collisions`] and its siblings below, so it does not belong
    /// on that type.
    collision_tolerance: f64,
    /// `distance_field_cache_entry_`. The only field here with no
    /// already-ported free-function equivalent -- see this module's doc
    /// comment.
    cache_entry: Option<DistanceFieldCacheEntry<'m>>,
}

impl<'m> DistanceFieldCollisionCache<'m> {
    /// Upstream `CollisionEnvDistanceField::initialize`'s config-storage
    /// half only. Upstream's other half -- a loop over every joint group
    /// building a `DistanceFieldCacheEntry` and pregenerating a
    /// `GroupStateRepresentation` for it -- exists to populate
    /// `pregenerated_group_state_representation_map_`, which lets later
    /// [`Self::generate_collision_checking_structures`] calls reuse an
    /// already-built `PosedDistanceField` per link instead of rebuilding one
    /// from scratch every call (see
    /// [`group_state_representation`]'s own "Deviations from upstream" for
    /// the *value*-level half of this story: as of round 25, that function
    /// reproduces the pregenerated branch's `sphere_locations` output
    /// directly, so this loop is no longer needed for output correctness --
    /// only for the caching optimization).
    ///
    /// That optimization itself stays unported, and is a genuine type-level
    /// obstacle, not a missing convenience: upstream's per-group cache entry
    /// is `pregenerated_group_state_representation_map_[group] : GroupStateRepresentation`,
    /// and this port's [`GroupStateRepresentation<'a, 'm>`] borrows its
    /// `dfce: &'a DistanceFieldCacheEntry<'m>` rather than owning/sharing it.
    /// To hold N pregenerated entries (one per joint model group) alive
    /// across calls, `Self` would need to *also* own N independent
    /// `DistanceFieldCacheEntry<'m>` values (not the single, call-overwritten
    /// `cache_entry` slot below) with addresses stable for `Self`'s own
    /// lifetime, while simultaneously storing `GroupStateRepresentation`
    /// values borrowing from them -- a struct holding both owned data and a
    /// reference into a sibling field of the same struct. Safe Rust cannot
    /// express that directly; it requires either pinning/unsafe or an
    /// external self-referential-struct crate (e.g. `ouroboros`/`self_cell`),
    /// neither of which this round adds for what is now, post-round-25, a
    /// pure performance optimization with no output-correctness gap left to
    /// close. **Falsifier:** this becomes worth reconsidering only if a
    /// caller-observable *behavior* gap (not merely speed) is measured
    /// between rebuild-every-call and upstream's cache-reuse -- e.g. a field
    /// this port's fresh rebuild computes differently from what a reused,
    /// once-built decomposition would retain across calls. No such gap is
    /// known; §154's own gap (`sphere_locations`) is closed without it.
    pub fn new(
        link_body_decompositions: LinkBodyDecompositions,
        distance_field_config: DistanceFieldConfig,
        collision_tolerance: f64,
    ) -> Self {
        Self {
            link_body_decompositions,
            distance_field_config,
            collision_tolerance,
            cache_entry: None,
        }
    }

    /// Upstream `CollisionEnvDistanceField::generateCollisionCheckingStructures`.
    /// Reuses the cached entry when [`get_distance_field_cache_entry`]
    /// accepts it for `group_name`/`state`/`acm` *and* it already carries a
    /// distance field or none was asked for; otherwise rebuilds one via
    /// [`generate_distance_field_cache_entry`] and replaces the cache before
    /// building this call's [`GroupStateRepresentation`] via
    /// [`group_state_representation`] -- the exact three-step sequence
    /// upstream's own body performs (`collision_env_distance_field.cpp:158-175`).
    ///
    /// # Deviation from upstream
    ///
    /// Upstream serializes the cache write with `update_cache_lock_`
    /// (`std::scoped_lock`) because its method is `const` and mutates
    /// `distance_field_cache_entry_` through a `const_cast` -- a workaround
    /// for `CollisionEnvDistanceField`'s own `checkCollision`/
    /// `checkSelfCollision` methods needing to stay `const` while still
    /// caching. This port's `&mut self` gives the same single-writer
    /// exclusivity at compile time, so there is no mutex field to port.
    ///
    /// # Errors
    ///
    /// See [`generate_distance_field_cache_entry`] and
    /// [`group_state_representation`].
    pub fn generate_collision_checking_structures<'s>(
        &'s mut self,
        group_name: &str,
        state: &Posed<'_, 'm>,
        acm: Option<&AllowedCollisionMatrix>,
        current_attached_bodies: &[AttachedBodyGeometry<'_>],
        generate_distance_field: bool,
    ) -> Result<GroupStateRepresentation<'s, 'm>> {
        let needs_rebuild = match get_distance_field_cache_entry(
            self.cache_entry.as_ref(),
            group_name,
            state,
            acm,
            current_attached_bodies,
        ) {
            None => true,
            Some(dfce) => generate_distance_field && dfce.distance_field.is_none(),
        };

        if needs_rebuild {
            let config = generate_distance_field.then_some(self.distance_field_config);
            self.cache_entry = Some(generate_distance_field_cache_entry(
                group_name,
                state,
                acm,
                &self.link_body_decompositions,
                config,
                current_attached_bodies,
            )?);
        }

        let dfce = self
            .cache_entry
            .as_ref()
            .expect("just populated above whenever it was absent or stale");
        group_state_representation(
            dfce,
            state,
            &self.link_body_decompositions.0,
            self.distance_field_config.geometry.resolution,
            self.distance_field_config.max_propagation_distance,
            self.distance_field_config.use_signed_distance_field,
            current_attached_bodies,
        )
    }

    /// Upstream `CollisionEnvDistanceField::checkSelfCollisionHelper`
    /// (`collision_env_distance_field.cpp:177-200`), collapsing its four
    /// `checkSelfCollision` overload callers (`:235-272`) into the `acm`
    /// parameter below -- see this module's doc comment's difference table
    /// for exactly which upstream overload each `acm` value reproduces.
    ///
    /// # Why this returns `(CollisionResult, GroupStateRepresentation)`, not `&mut Option<GroupStateRepresentation>`
    ///
    /// Every upstream overload takes `GroupStateRepresentationPtr& gsr` as an
    /// in/out reference: `gsr == nullptr` on entry means "no cached state,
    /// build one"; a non-null `gsr` means "reuse and cheaply re-pose the one
    /// I already built on an earlier call" via
    /// `updateGroupStateRepresentationState` instead of a full rebuild. That
    /// second half -- a caller holding one `gsr` alive *across several
    /// separate top-level calls* to reuse it -- cannot be reproduced with a
    /// plain `&mut Option<GroupStateRepresentation<'s, 'm>>` parameter here,
    /// and this is a hard fact about this port's existing types, not a
    /// stylistic choice: [`GroupStateRepresentation::dfce`] is a genuine
    /// `&'a DistanceFieldCacheEntry<'m>` borrow into `self.cache_entry`
    /// (round 20's [`DistanceFieldCollisionCache`] design, kept as-is here --
    /// changing it to an owned/cloned `dfce` would be a real semantic change
    /// to a type this file and its tests already depend on throughout, and
    /// is not what this round asked for). Because [`Self::generate_collision_checking_structures`]
    /// takes `&'s mut self` to produce it, any `GroupStateRepresentation<'s, 'm>`
    /// a caller is still holding keeps `self` mutably borrowed for that same
    /// `'s` -- so a *second* call needing `&'s mut self` again (to check
    /// `gsr.is_none()` and potentially rebuild) cannot start until the first
    /// call's `gsr` is no longer live. A caller cannot simultaneously "still
    /// be holding `gsr` from call 1" and "start call 2, which needs `&mut
    /// self` again" -- the two requirements contradict outright, for *any*
    /// signature shaped `fn(&mut self, ..., &mut Option<GroupStateRepresentation<'s, 'm>>)`.
    /// gcc/upstream do not hit this because `GroupStateRepresentationPtr` is
    /// a `shared_ptr` with no borrow checker watching `distance_field_cache_entry_`.
    ///
    /// Given that, upstream's own `Option`-like (nullable-pointer) `gsr`
    /// varies only on *entry* -- every successful call leaves it populated on
    /// *exit* -- and the entry-side variation is exactly the half this port
    /// cannot honor. What survives is a plain owned return: this method
    /// always performs the "build fresh, or reuse via the `dfce` cache" path
    /// (matching upstream's `checkSelfCollision(req, res, state[, acm])`
    /// overloads -- the two that pass a *local*, always-null `gsr` into the
    /// helper -- not the two that thread a caller-owned `gsr` through), and
    /// hands the resulting [`GroupStateRepresentation`] back so a caller can
    /// still inspect gradients/decompositions afterward, matching the
    /// informational value the two `gsr`-out overloads provide. A caller
    /// that genuinely needs the cheap, `self`-free re-pose-without-rebuild
    /// path upstream's caller-owned-`gsr` overloads exist for can still get
    /// it directly: hold the returned `GroupStateRepresentation` and call
    /// [`update_group_state_representation_state`]/`get_self_collisions`/
    /// `get_intra_group_collisions` on it themselves for a new state --
    /// none of those free functions take `self` at all, so nothing about
    /// *that* reuse is blocked, only the convenience of doing it through one
    /// [`DistanceFieldCollisionCache`] method call.
    ///
    /// # Errors
    ///
    /// See [`Self::generate_collision_checking_structures`].
    pub fn check_self_collision<'s>(
        &'s mut self,
        req: &CollisionRequest,
        state: &Posed<'_, 'm>,
        acm: Option<&AllowedCollisionMatrix>,
        current_attached_bodies: &[AttachedBodyGeometry<'_>],
    ) -> Result<(CollisionResult, GroupStateRepresentation<'s, 'm>)> {
        let max_propagation_distance = self.distance_field_config.max_propagation_distance;
        let collision_tolerance = self.collision_tolerance;
        let group_name = req.group_name.as_deref().unwrap_or_default();
        let mut gsr = self.generate_collision_checking_structures(
            group_name,
            state,
            acm,
            current_attached_bodies,
            true,
        )?;

        let mut res = CollisionResult {
            contacts: req.contacts.then(ContactData::default),
            ..CollisionResult::default()
        };
        let done = get_self_collisions(
            req,
            &mut res,
            &mut gsr,
            max_propagation_distance,
            collision_tolerance,
        );
        if !done {
            get_intra_group_collisions(req, &mut res, &mut gsr);
        }
        Ok((res, gsr))
    }

    /// Upstream `CollisionEnvDistanceField::checkCollision`'s four overloads
    /// (`collision_env_distance_field.cpp:1382-1442`), collapsed the same way
    /// as [`Self::check_self_collision`] -- see that method's doc for why
    /// this returns `(CollisionResult, GroupStateRepresentation)` rather than
    /// taking a caller-owned `gsr` in/out.
    ///
    /// Unlike [`Self::check_self_collision`], every `checkCollision` overload
    /// also checks environment collisions, via
    /// `distance_field_cache_entry_world_->distance_field_` -- a `World`-
    /// sourced field this crate does not own (see this module's doc
    /// comment). `env_distance_field` below is an explicit caller-supplied
    /// parameter instead, the same way [`crate::PropagationDistanceField`]
    /// is threaded through this crate wherever upstream reads it off a
    /// `World` this port has no type for.
    ///
    /// # Errors
    ///
    /// See [`Self::generate_collision_checking_structures`].
    pub fn check_collision<'s>(
        &'s mut self,
        req: &CollisionRequest,
        state: &Posed<'_, 'm>,
        acm: Option<&AllowedCollisionMatrix>,
        current_attached_bodies: &[AttachedBodyGeometry<'_>],
        env_distance_field: &dyn DistanceField,
    ) -> Result<(CollisionResult, GroupStateRepresentation<'s, 'm>)> {
        let max_propagation_distance = self.distance_field_config.max_propagation_distance;
        let collision_tolerance = self.collision_tolerance;
        let group_name = req.group_name.as_deref().unwrap_or_default();
        let mut gsr = self.generate_collision_checking_structures(
            group_name,
            state,
            acm,
            current_attached_bodies,
            true,
        )?;

        let mut res = CollisionResult {
            contacts: req.contacts.then(ContactData::default),
            ..CollisionResult::default()
        };
        let mut done = get_self_collisions(
            req,
            &mut res,
            &mut gsr,
            max_propagation_distance,
            collision_tolerance,
        );
        if !done {
            done = get_intra_group_collisions(req, &mut res, &mut gsr);
        }
        if !done {
            get_environment_collisions(
                req,
                &mut res,
                env_distance_field,
                &mut gsr,
                max_propagation_distance,
                collision_tolerance,
            );
        }
        Ok((res, gsr))
    }

    /// Upstream `CollisionEnvDistanceField::checkRobotCollision`'s two real
    /// (non-continuous) overloads (`collision_env_distance_field.cpp:1447-1500`).
    /// The two continuous-state overloads (`:1502`, `:1510`) are not ported:
    /// upstream itself stubs both to `RCLCPP_ERROR(logger_, "Continuous
    /// collision checking not implemented")` and returns without checking
    /// anything, matching
    /// `moveit_collision::CollisionEnv::check_robot_collision_continuous`'s
    /// `Err`-returning convention rather than silently reporting "no
    /// collision" for a query that was never actually run.
    ///
    /// # Deviation from upstream: the `generate_distance_field` asymmetry, preserved
    ///
    /// The no-`acm` overload (`:1447`) passes `generate_distance_field =
    /// false` into `generateCollisionCheckingStructures`; the `acm` overload
    /// (`:1473`) passes `true`. Neither overload's own body reads
    /// `dfce.distance_field` -- both only ever touch the separately-sourced
    /// `env_distance_field` -- so this makes *no difference to this call's
    /// own observable result* either way. It changes what ends up cached in
    /// `self.cache_entry` for a *later* call that reuses it (a subsequent
    /// [`Self::check_self_collision`]/[`Self::check_collision`] on the same
    /// cache would otherwise find a stale `distance_field: None` and have to
    /// rebuild). Preserved exactly rather than unified to one value in either
    /// direction, via `acm.is_some()` below.
    ///
    /// # Errors
    ///
    /// See [`Self::generate_collision_checking_structures`].
    pub fn check_robot_collision<'s>(
        &'s mut self,
        req: &CollisionRequest,
        state: &Posed<'_, 'm>,
        acm: Option<&AllowedCollisionMatrix>,
        current_attached_bodies: &[AttachedBodyGeometry<'_>],
        env_distance_field: &dyn DistanceField,
    ) -> Result<(CollisionResult, GroupStateRepresentation<'s, 'm>)> {
        let max_propagation_distance = self.distance_field_config.max_propagation_distance;
        let collision_tolerance = self.collision_tolerance;
        let group_name = req.group_name.as_deref().unwrap_or_default();
        let mut gsr = self.generate_collision_checking_structures(
            group_name,
            state,
            acm,
            current_attached_bodies,
            acm.is_some(),
        )?;

        let mut res = CollisionResult {
            contacts: req.contacts.then(ContactData::default),
            ..CollisionResult::default()
        };
        get_environment_collisions(
            req,
            &mut res,
            env_distance_field,
            &mut gsr,
            max_propagation_distance,
            collision_tolerance,
        );
        Ok((res, gsr))
    }

    /// Upstream `CollisionEnvDistanceField::getCollisionGradients`
    /// (`collision_env_distance_field.cpp:1517-1538`).
    ///
    /// # Deviation from upstream: no `res` parameter
    ///
    /// Upstream's `res` parameter is commented `/*res*/` in its own
    /// signature -- never read or written anywhere in the body. Not ported;
    /// a caller reads gradients back out of the returned
    /// [`GroupStateRepresentation`] itself, the same way upstream's own
    /// caller (`getCollisionGradients`'s only caller outside this file) does.
    ///
    /// # Errors
    ///
    /// See [`Self::generate_collision_checking_structures`].
    pub fn get_collision_gradients<'s>(
        &'s mut self,
        req: &CollisionRequest,
        state: &Posed<'_, 'm>,
        acm: Option<&AllowedCollisionMatrix>,
        current_attached_bodies: &[AttachedBodyGeometry<'_>],
        env_distance_field: &dyn DistanceField,
    ) -> Result<GroupStateRepresentation<'s, 'm>> {
        let max_propagation_distance = self.distance_field_config.max_propagation_distance;
        let collision_tolerance = self.collision_tolerance;
        let group_name = req.group_name.as_deref().unwrap_or_default();
        let mut gsr = self.generate_collision_checking_structures(
            group_name,
            state,
            acm,
            current_attached_bodies,
            true,
        )?;

        get_self_proximity_gradients(&mut gsr, collision_tolerance, max_propagation_distance);
        get_intra_group_proximity_gradients(&mut gsr);
        get_environment_proximity_gradients(
            env_distance_field,
            &mut gsr,
            collision_tolerance,
            max_propagation_distance,
        );
        Ok(gsr)
    }

    /// Upstream `CollisionEnvDistanceField::getAllCollisions`
    /// (`collision_env_distance_field.cpp:1540-1559`).
    ///
    /// # Deviation from upstream: none, but note the shape versus `check_collision`
    ///
    /// Unlike [`Self::check_collision`], this calls
    /// `get_self_collisions`/`get_intra_group_collisions`/
    /// `get_environment_collisions` unconditionally: upstream's own body
    /// has no `if (!done)` guard between them at all here (unlike
    /// `checkCollision`), discarding every one of their return values. Ported
    /// as-is: a caller that wants *every* collision found, not just whichever
    /// of the three phases hits first, uses this instead of
    /// [`Self::check_collision`].
    ///
    /// # Errors
    ///
    /// See [`Self::generate_collision_checking_structures`].
    pub fn get_all_collisions<'s>(
        &'s mut self,
        req: &CollisionRequest,
        state: &Posed<'_, 'm>,
        acm: Option<&AllowedCollisionMatrix>,
        current_attached_bodies: &[AttachedBodyGeometry<'_>],
        env_distance_field: &dyn DistanceField,
    ) -> Result<(CollisionResult, GroupStateRepresentation<'s, 'm>)> {
        let max_propagation_distance = self.distance_field_config.max_propagation_distance;
        let collision_tolerance = self.collision_tolerance;
        let group_name = req.group_name.as_deref().unwrap_or_default();
        let mut gsr = self.generate_collision_checking_structures(
            group_name,
            state,
            acm,
            current_attached_bodies,
            true,
        )?;

        let mut res = CollisionResult {
            contacts: req.contacts.then(ContactData::default),
            ..CollisionResult::default()
        };
        get_self_collisions(
            req,
            &mut res,
            &mut gsr,
            max_propagation_distance,
            collision_tolerance,
        );
        get_intra_group_collisions(req, &mut res, &mut gsr);
        get_environment_collisions(
            req,
            &mut res,
            env_distance_field,
            &mut gsr,
            max_propagation_distance,
            collision_tolerance,
        );
        Ok((res, gsr))
    }
}

/// Upstream `CollisionEnvDistanceField::getSelfCollisions`
/// (`collision_env_distance_field.cpp:274-353`). Checks every
/// geometry-bearing, self-collision-enabled link's collision spheres against
/// `gsr.dfce.distance_field` -- the group's own aggregate distance field,
/// built by [`generate_distance_field_cache_entry`] from every *other*
/// link's points (see [`build_non_group_distance_field`]) -- stopping as
/// soon as either a collision is found with `req.contacts` unset, or
/// `req.max_contacts` is reached with it set.
///
/// # Deviation from upstream
///
/// None functionally. Upstream's loop bound is `link_names_.size() +
/// attached_body_names_.size()` (`:278`), with an `is_link` branch (`:279`)
/// selecting `link_body_decompositions_`/`link_names_` or
/// `attached_body_decompositions_`/`attached_body_names_` per index
/// (`:284-295`) and reporting attached-body self-collisions with
/// `body_type_1 = BodyTypes::ROBOT_ATTACHED` instead of `ROBOT_LINK`
/// (`:312-320`) -- ported as the `is_link` `if`/`else` below, matching that
/// indexing exactly (round 23; the attached-body half was omitted through
/// round 22 on the mistaken belief `attached_body_names_` stayed
/// permanently empty in this port, corrected once round 22 itself falsified
/// that premise).
///
/// # Panics
///
/// If `gsr.dfce.distance_field` is `None`. Every caller in this file builds
/// `gsr` through
/// [`DistanceFieldCollisionCache::generate_collision_checking_structures`]
/// with `generate_distance_field: true` before calling this, so the field is
/// always present in practice -- matching upstream, which never null-checks
/// `dfce_->distance_field_` here either (`getCollisionSphereCollision` is
/// called directly on `.get()`).
fn get_self_collisions(
    req: &CollisionRequest,
    res: &mut CollisionResult,
    gsr: &mut GroupStateRepresentation<'_, '_>,
    max_propagation_distance: f64,
    collision_tolerance: f64,
) -> bool {
    let distance_field: &dyn DistanceField = gsr.dfce.distance_field.as_ref().expect(
        "generate_collision_checking_structures always requests a distance field before \
         self-collision checks",
    );
    let num_links = gsr.dfce.link_names.len();
    let total = num_links + gsr.dfce.attached_body_names.len();

    for i in 0..total {
        let is_link = i < num_links;
        if (is_link && !gsr.dfce.link_has_geometry[i]) || !gsr.dfce.self_collision_enabled[i] {
            continue;
        }
        let (spheres, centers): (&[CollisionSphere], &[Vector3<f64>]) = if is_link {
            let bd = gsr.link_body_decompositions[i]
                .as_ref()
                .expect("link_has_geometry[i] implies link_body_decompositions[i] is Some");
            (bd.collision_spheres(), bd.sphere_centers())
        } else {
            let bd = &gsr.attached_body_decompositions[i - num_links];
            (bd.collision_spheres(), bd.sphere_centers())
        };

        if req.contacts {
            let already = res.contacts.as_ref().map_or(0, ContactData::count);
            let limit = req
                .max_contacts_per_pair
                .min(req.max_contacts.saturating_sub(already));
            let num_coll = u32::try_from(limit).unwrap_or(u32::MAX);
            let mut colls = Vec::new();
            let coll = get_collision_sphere_collisions(
                distance_field,
                spheres,
                centers,
                max_propagation_distance,
                collision_tolerance,
                num_coll,
                &mut colls,
            );
            if coll {
                res.collision = true;
                let (body_name_1, body_type_1) = if is_link {
                    (gsr.dfce.link_names[i].clone(), BodyType::RobotLink)
                } else {
                    (
                        gsr.dfce.attached_body_names[i - num_links].clone(),
                        BodyType::RobotAttached,
                    )
                };
                let contacts = res.contacts.get_or_insert_with(ContactData::default);
                for &col in &colls {
                    let con = Contact {
                        pos: centers[col as usize],
                        body_name_1: body_name_1.clone(),
                        body_type_1,
                        body_name_2: "self".to_string(),
                        body_type_2: BodyType::RobotLink,
                        ..Contact::default()
                    };
                    contacts
                        .by_pair
                        .entry((body_name_1.clone(), "self".to_string()))
                        .or_default()
                        .push(con);
                    gsr.gradients[i].types[col as usize] = CollisionType::SelfCollision;
                }
                gsr.gradients[i].collision = true;
                if contacts.count() >= req.max_contacts {
                    return true;
                }
            }
        } else {
            let coll = get_collision_sphere_collision(
                distance_field,
                spheres,
                centers,
                max_propagation_distance,
                collision_tolerance,
            );
            if coll {
                res.collision = true;
                return true;
            }
        }
    }
    res.contacts
        .as_ref()
        .is_some_and(|c| c.count() >= req.max_contacts)
}

/// Upstream `CollisionEnvDistanceField::getSelfProximityGradients`
/// (`collision_env_distance_field.cpp:355-428`). For every geometry-bearing,
/// self-collision-enabled link, folds in gradients from every *other* link's
/// own [`PosedDistanceField`] not ruled out by the ACM (`Never` or absent --
/// see [`AllowedCollisionType`]), then from the group's aggregate
/// `gsr.dfce.distance_field`.
///
/// # Deviation from upstream
///
/// Unlike [`get_self_collisions`], this one is a *faithful* port with no
/// attached-body gap: upstream's own loop condition here is `i <
/// link_names_.size()` (`:359`), not `i < link_names_.size() +
/// attached_body_names_.size()` like [`get_self_collisions`] (`:278`) --
/// so the `is_link` computed at `:362` is always `true` and upstream's own
/// `is_link`-false branch (`:373-377`) is unreachable dead code in the C++
/// itself, correctly omitted here rather than ported unreachable. (Round 22
/// mistakenly grouped this function with [`get_self_collisions`]'s real
/// gap; round 23's fresh read of the loop bound found the two are not
/// alike -- see [`get_environment_proximity_gradients`] for the same
/// pattern on the environment side.)
///
/// # Panics
///
/// Same as [`get_self_collisions`].
fn get_self_proximity_gradients(
    gsr: &mut GroupStateRepresentation<'_, '_>,
    collision_tolerance: f64,
    max_propagation_distance: f64,
) -> bool {
    let distance_field: &dyn DistanceField = gsr.dfce.distance_field.as_ref().expect(
        "generate_collision_checking_structures always requests a distance field before \
         gradient queries",
    );
    let mut in_collision = false;

    for i in 0..gsr.dfce.link_names.len() {
        if !gsr.dfce.link_has_geometry[i] || !gsr.dfce.self_collision_enabled[i] {
            continue;
        }
        let link_name = gsr.dfce.link_names[i].clone();

        if !gsr.dfce.acm.is_empty() {
            for j in 0..gsr.dfce.link_names.len() {
                if link_name == gsr.dfce.link_names[j] {
                    continue;
                }
                let allowed = gsr
                    .dfce
                    .acm
                    .allowed_collision(&link_name, &gsr.dfce.link_names[j]);
                if let Some(entry) = allowed {
                    if entry.kind() != AllowedCollisionType::Never {
                        continue;
                    }
                }
                let Some(field_j) = gsr.link_distance_fields[j].as_ref() else {
                    continue;
                };
                let bd_i = gsr.link_body_decompositions[i]
                    .as_ref()
                    .expect("link_has_geometry[i] implies link_body_decompositions[i] is Some");
                let query = SphereGradientQuery {
                    collision_type: CollisionType::SelfCollision,
                    tolerance: collision_tolerance,
                    subtract_radii: false,
                    maximum_value: max_propagation_distance,
                    stop_at_first_collision: false,
                };
                let coll = field_j.get_collision_sphere_gradients(
                    bd_i.collision_spheres(),
                    bd_i.sphere_centers(),
                    &mut gsr.gradients[i],
                    &query,
                );
                if coll {
                    in_collision = true;
                }
            }
        }

        let bd_i = gsr.link_body_decompositions[i]
            .as_ref()
            .expect("link_has_geometry[i] implies link_body_decompositions[i] is Some");
        let query = SphereGradientQuery {
            collision_type: CollisionType::SelfCollision,
            tolerance: collision_tolerance,
            subtract_radii: false,
            maximum_value: max_propagation_distance,
            stop_at_first_collision: false,
        };
        let coll = get_collision_sphere_gradients(
            distance_field,
            bd_i.collision_spheres(),
            bd_i.sphere_centers(),
            &mut gsr.gradients[i],
            &query,
        );
        if coll {
            in_collision = true;
        }
    }
    in_collision
}

/// Upstream `CollisionEnvDistanceField::getIntraGroupCollisions`
/// (`collision_env_distance_field.cpp:430-642`). Checks every pair of
/// geometry-bearing, intra-group-collision-enabled links whose bounding
/// spheres intersect ([`do_bounding_spheres_intersect`]), the same
/// contacts/early-exit shape as [`get_self_collisions`].
///
/// # Deviation from upstream
///
/// Upstream's loop covers `link_names_.size() + attached_body_names_.size()`
/// indices (`:437`) with `i_is_link`/`j_is_link` branches (`:442-443`)
/// selecting between `link_body_decompositions_`/`attached_body_decompositions_`
/// per side of the pair, plus an `i == j` guard (`:440-441`) that can never
/// trigger (the inner loop already starts at `j = i + 1`, so that guard is
/// dead in upstream regardless of attached bodies and is correctly omitted
/// here, as it was before round 23). Ported below (round 23) with the same
/// three-way bounding-sphere pre-filter upstream uses (`:451-499`): both
/// sides link uses [`do_bounding_spheres_intersect`] directly; either side
/// attached iterates that attached body's own sub-decompositions
/// (`PosedBodySphereDecompositionVector::getPosedBodySphereDecomposition`,
/// `:462-497`) checking each against the other side's (possibly also
/// per-sub-decomposition) bounding sphere.
///
/// One upstream bug is *not* reproduced: its contact-reporting branch
/// (`:601`) unconditionally reads `con.pos =
/// gsr->link_body_decompositions_[i]->getSphereCenters()[k]`, never
/// branching on `i_is_link` the way `body_type_1` immediately below it does
/// (`:604-611`, round 26: corrected from a prior citation of `:534`/`:537-544`,
/// which point at the unrelated `req.contacts` header a few lines earlier --
/// the underlying bug claim itself was always correct) -- when `i` is an
/// attached-body index this indexes
/// `link_body_decompositions_` (sized `num_links`) out of bounds, undefined
/// behaviour in C++ that safe Rust cannot reproduce. `centers_1[k]` (already
/// correctly sourced from whichever side `i` actually is, matching every
/// other read of sphere position in this same block) is the value the
/// surrounding code's own intent requires and what `body_type_1`'s sibling
/// branch shows was meant to be conditional; using it is the closest safe
/// equivalent, not an invented deviation.
fn get_intra_group_collisions(
    req: &CollisionRequest,
    res: &mut CollisionResult,
    gsr: &mut GroupStateRepresentation<'_, '_>,
) -> bool {
    let num_links = gsr.dfce.link_names.len();
    let total = num_links + gsr.dfce.attached_body_names.len();
    for i in 0..total {
        for j in (i + 1)..total {
            let i_is_link = i < num_links;
            let j_is_link = j < num_links;

            if (i_is_link && !gsr.dfce.link_has_geometry[i])
                || (j_is_link && !gsr.dfce.link_has_geometry[j])
            {
                continue;
            }
            if !gsr.dfce.intra_group_collision_enabled[i][j] {
                continue;
            }

            let bounding_spheres_disjoint = if i_is_link && j_is_link {
                let bd_i = gsr.link_body_decompositions[i]
                    .as_ref()
                    .expect("link_has_geometry[i] implies link_body_decompositions[i] is Some");
                let bd_j = gsr.link_body_decompositions[j]
                    .as_ref()
                    .expect("link_has_geometry[j] implies link_body_decompositions[j] is Some");
                !do_bounding_spheres_intersect(bd_i, bd_j)
            } else if !i_is_link && j_is_link {
                let bd_j = gsr.link_body_decompositions[j]
                    .as_ref()
                    .expect("link_has_geometry[j] implies link_body_decompositions[j] is Some");
                let attached_i = &gsr.attached_body_decompositions[i - num_links];
                !(0..attached_i.len()).any(|k| {
                    let sub = attached_i.get(k).expect("k < attached_i.len()");
                    do_bounding_spheres_intersect(bd_j, sub)
                })
            } else if i_is_link && !j_is_link {
                let bd_i = gsr.link_body_decompositions[i]
                    .as_ref()
                    .expect("link_has_geometry[i] implies link_body_decompositions[i] is Some");
                let attached_j = &gsr.attached_body_decompositions[j - num_links];
                !(0..attached_j.len()).any(|l| {
                    let sub = attached_j.get(l).expect("l < attached_j.len()");
                    do_bounding_spheres_intersect(bd_i, sub)
                })
            } else {
                let attached_i = &gsr.attached_body_decompositions[i - num_links];
                let attached_j = &gsr.attached_body_decompositions[j - num_links];
                !(0..attached_i.len()).any(|k| {
                    let sub_i = attached_i.get(k).expect("k < attached_i.len()");
                    (0..attached_j.len()).any(|l| {
                        let sub_j = attached_j.get(l).expect("l < attached_j.len()");
                        do_bounding_spheres_intersect(sub_i, sub_j)
                    })
                })
            };
            if bounding_spheres_disjoint {
                continue;
            }

            let (name_1, body_type_1) = if i_is_link {
                (gsr.dfce.link_names[i].clone(), BodyType::RobotLink)
            } else {
                (
                    gsr.dfce.attached_body_names[i - num_links].clone(),
                    BodyType::RobotAttached,
                )
            };
            let (name_2, body_type_2) = if j_is_link {
                (gsr.dfce.link_names[j].clone(), BodyType::RobotLink)
            } else {
                (
                    gsr.dfce.attached_body_names[j - num_links].clone(),
                    BodyType::RobotAttached,
                )
            };
            let mut num_pair = res
                .contacts
                .as_ref()
                .and_then(|c| c.by_pair.get(&(name_1.clone(), name_2.clone())))
                .map_or(0usize, Vec::len);

            let (spheres_1, centers_1): (&[CollisionSphere], &[Vector3<f64>]) = if i_is_link {
                let bd = gsr.link_body_decompositions[i]
                    .as_ref()
                    .expect("link_has_geometry[i] implies link_body_decompositions[i] is Some");
                (bd.collision_spheres(), bd.sphere_centers())
            } else {
                let bd = &gsr.attached_body_decompositions[i - num_links];
                (bd.collision_spheres(), bd.sphere_centers())
            };
            let (spheres_2, centers_2): (&[CollisionSphere], &[Vector3<f64>]) = if j_is_link {
                let bd = gsr.link_body_decompositions[j]
                    .as_ref()
                    .expect("link_has_geometry[j] implies link_body_decompositions[j] is Some");
                (bd.collision_spheres(), bd.sphere_centers())
            } else {
                let bd = &gsr.attached_body_decompositions[j - num_links];
                (bd.collision_spheres(), bd.sphere_centers())
            };

            let mut k = 0;
            while k < spheres_1.len() && (!req.contacts || num_pair < req.max_contacts_per_pair) {
                let mut l = 0;
                while l < spheres_2.len() && (!req.contacts || num_pair < req.max_contacts_per_pair)
                {
                    let dist = (centers_1[k] - centers_2[l]).norm();
                    if dist < spheres_1[k].radius + spheres_2[l].radius {
                        res.collision = true;
                        if req.contacts {
                            let con = Contact {
                                pos: centers_1[k],
                                body_name_1: name_1.clone(),
                                body_type_1,
                                body_name_2: name_2.clone(),
                                body_type_2,
                                ..Contact::default()
                            };
                            let contacts = res.contacts.get_or_insert_with(ContactData::default);
                            contacts
                                .by_pair
                                .entry((name_1.clone(), name_2.clone()))
                                .or_default()
                                .push(con);
                            num_pair += 1;
                            gsr.gradients[i].types[k] = CollisionType::Intra;
                            gsr.gradients[i].collision = true;
                            gsr.gradients[j].types[l] = CollisionType::Intra;
                            gsr.gradients[j].collision = true;
                            if contacts.count() >= req.max_contacts {
                                return true;
                            }
                        } else {
                            return true;
                        }
                    }
                    l += 1;
                }
                k += 1;
            }
        }
    }
    false
}

/// Upstream `CollisionEnvDistanceField::getIntraGroupProximityGradients`
/// (`collision_env_distance_field.cpp:644-709`). For every pair of
/// geometry-bearing, intra-group-collision-enabled links, folds each
/// sphere's nearest opposing-sphere distance into that sphere's
/// [`GradientInfo`] slot whenever it improves on what is already there.
///
/// # Deviation from upstream
///
/// Upstream's loop bound and `i_is_link`/`j_is_link` sphere-source selection
/// (`:650-680`) match [`get_intra_group_collisions`]'s exactly; ported the
/// same way (round 23), minus the bounding-sphere pre-filter -- upstream
/// itself has none here, going straight to the full pairwise sphere loop
/// (`:681-703`). The unreachable `i == j` guard is correctly omitted, same
/// reasoning as [`get_intra_group_collisions`]. Upstream's own
/// `in_collision` local is declared, never written, and returned as-is --
/// always `false`; ported faithfully rather than changed to `-> ()`, since
/// upstream's own caller (`getCollisionGradients`) discards this return
/// value too, and keeping the `bool` shape matches this function's siblings
/// ([`get_self_proximity_gradients`]/[`get_environment_proximity_gradients`]).
fn get_intra_group_proximity_gradients(gsr: &mut GroupStateRepresentation<'_, '_>) -> bool {
    let num_links = gsr.dfce.link_names.len();
    let total = num_links + gsr.dfce.attached_body_names.len();
    for i in 0..total {
        for j in (i + 1)..total {
            let i_is_link = i < num_links;
            let j_is_link = j < num_links;

            if (i_is_link && !gsr.dfce.link_has_geometry[i])
                || (j_is_link && !gsr.dfce.link_has_geometry[j])
            {
                continue;
            }
            if !gsr.dfce.intra_group_collision_enabled[i][j] {
                continue;
            }
            let centers_1: &[Vector3<f64>] = if i_is_link {
                gsr.link_body_decompositions[i]
                    .as_ref()
                    .expect("link_has_geometry[i] implies link_body_decompositions[i] is Some")
                    .sphere_centers()
            } else {
                gsr.attached_body_decompositions[i - num_links].sphere_centers()
            };
            let centers_2: &[Vector3<f64>] = if j_is_link {
                gsr.link_body_decompositions[j]
                    .as_ref()
                    .expect("link_has_geometry[j] implies link_body_decompositions[j] is Some")
                    .sphere_centers()
            } else {
                gsr.attached_body_decompositions[j - num_links].sphere_centers()
            };

            for (k, &c1) in centers_1.iter().enumerate() {
                for (l, &c2) in centers_2.iter().enumerate() {
                    let gradient = c1 - c2;
                    let dist = gradient.norm();
                    if dist < gsr.gradients[i].distances[k] {
                        gsr.gradients[i].distances[k] = dist;
                        gsr.gradients[i].gradients[k] = gradient;
                        gsr.gradients[i].types[k] = CollisionType::Intra;
                    }
                    if dist < gsr.gradients[j].distances[l] {
                        gsr.gradients[j].distances[l] = dist;
                        gsr.gradients[j].gradients[l] = -gradient;
                        gsr.gradients[j].types[l] = CollisionType::Intra;
                    }
                }
            }
        }
    }
    false
}

/// Upstream `CollisionEnvDistanceField::getEnvironmentCollisions`
/// (`collision_env_distance_field.cpp:1561-1643`). Same shape as
/// [`get_self_collisions`], checking every geometry-bearing link's collision
/// spheres against `env_distance_field` instead of the group's own aggregate
/// field, and reporting contacts against a synthetic `"environment"` /
/// [`BodyType::WorldObject`] body rather than `"self"`.
///
/// `env_distance_field` is an explicit parameter rather than read off `self`
/// because it is upstream's `distance_field_cache_entry_world_->distance_field_`
/// -- `World`-sourced state this crate does not own (see this module's doc
/// comment).
///
/// # Deviation from upstream
///
/// None functionally, same shape as [`get_self_collisions`]'s own (round
/// 23) deviation note: upstream's loop bound is `link_names_.size() +
/// attached_body_names_.size()` (`:1565`) with an `is_link` branch
/// (`:1567`) selecting `link_body_decompositions_`/`attached_body_decompositions_`
/// per index (`:1576-1587`) and reporting attached-body environment
/// collisions with `body_type_1 = BodyTypes::ROBOT_ATTACHED` (`:1599-1607`)
/// -- ported below. Upstream also declares a `link_name` local
/// (`:1568`, `"attached"` placeholder for non-link indices) that is never
/// read anywhere in the function body; not ported, matching this crate's
/// standing practice of not carrying forward genuinely dead upstream state.
fn get_environment_collisions(
    req: &CollisionRequest,
    res: &mut CollisionResult,
    env_distance_field: &dyn DistanceField,
    gsr: &mut GroupStateRepresentation<'_, '_>,
    max_propagation_distance: f64,
    collision_tolerance: f64,
) -> bool {
    let num_links = gsr.dfce.link_names.len();
    let total = num_links + gsr.dfce.attached_body_names.len();
    for i in 0..total {
        let is_link = i < num_links;
        if is_link && !gsr.dfce.link_has_geometry[i] {
            continue;
        }
        let (spheres, centers): (&[CollisionSphere], &[Vector3<f64>]) = if is_link {
            let bd = gsr.link_body_decompositions[i]
                .as_ref()
                .expect("link_has_geometry[i] implies link_body_decompositions[i] is Some");
            (bd.collision_spheres(), bd.sphere_centers())
        } else {
            let bd = &gsr.attached_body_decompositions[i - num_links];
            (bd.collision_spheres(), bd.sphere_centers())
        };

        if req.contacts {
            let already = res.contacts.as_ref().map_or(0, ContactData::count);
            let limit = req
                .max_contacts_per_pair
                .min(req.max_contacts.saturating_sub(already));
            let num_coll = u32::try_from(limit).unwrap_or(u32::MAX);
            let mut colls = Vec::new();
            let coll = get_collision_sphere_collisions(
                env_distance_field,
                spheres,
                centers,
                max_propagation_distance,
                collision_tolerance,
                num_coll,
                &mut colls,
            );
            if coll {
                res.collision = true;
                let (body_name_1, body_type_1) = if is_link {
                    (gsr.dfce.link_names[i].clone(), BodyType::RobotLink)
                } else {
                    (
                        gsr.dfce.attached_body_names[i - num_links].clone(),
                        BodyType::RobotAttached,
                    )
                };
                let contacts = res.contacts.get_or_insert_with(ContactData::default);
                for &col in &colls {
                    let con = Contact {
                        pos: centers[col as usize],
                        body_name_1: body_name_1.clone(),
                        body_type_1,
                        body_name_2: "environment".to_string(),
                        body_type_2: BodyType::WorldObject,
                        ..Contact::default()
                    };
                    contacts
                        .by_pair
                        .entry((body_name_1.clone(), "environment".to_string()))
                        .or_default()
                        .push(con);
                    gsr.gradients[i].types[col as usize] = CollisionType::Environment;
                }
                gsr.gradients[i].collision = true;
                if contacts.count() >= req.max_contacts {
                    return true;
                }
            }
        } else {
            let coll = get_collision_sphere_collision(
                env_distance_field,
                spheres,
                centers,
                max_propagation_distance,
                collision_tolerance,
            );
            if coll {
                res.collision = true;
                return true;
            }
        }
    }
    res.contacts
        .as_ref()
        .is_some_and(|c| c.count() >= req.max_contacts)
}

/// Upstream `CollisionEnvDistanceField::getEnvironmentProximityGradients`
/// (`collision_env_distance_field.cpp:1645-1681`). Same shape as
/// [`get_self_proximity_gradients`]'s final (non-ACM) half: folds
/// `env_distance_field` gradients into every geometry-bearing link's
/// [`GradientInfo`] slot.
///
/// # Deviation from upstream
///
/// Unlike its five siblings ([`get_self_collisions`],
/// [`get_self_proximity_gradients`], [`get_intra_group_collisions`],
/// [`get_intra_group_proximity_gradients`], [`get_environment_collisions`]),
/// this one is a *faithful* port with no attached-body gap: upstream's own
/// loop condition here is `i < link_names_.size()` (`:1649`), not `i <
/// link_names_.size() + attached_body_names_.size()` like the other five --
/// so upstream's own `is_link`-false branch is unreachable dead code in the
/// C++ too, correctly omitted here rather than ported unreachable.
fn get_environment_proximity_gradients(
    env_distance_field: &dyn DistanceField,
    gsr: &mut GroupStateRepresentation<'_, '_>,
    collision_tolerance: f64,
    max_propagation_distance: f64,
) -> bool {
    let mut in_collision = false;
    for i in 0..gsr.dfce.link_names.len() {
        if !gsr.dfce.link_has_geometry[i] {
            continue;
        }
        let bd = gsr.link_body_decompositions[i]
            .as_ref()
            .expect("link_has_geometry[i] implies link_body_decompositions[i] is Some");
        let query = SphereGradientQuery {
            collision_type: CollisionType::Environment,
            tolerance: collision_tolerance,
            subtract_radii: false,
            maximum_value: max_propagation_distance,
            stop_at_first_collision: false,
        };
        let coll = get_collision_sphere_gradients(
            env_distance_field,
            bd.collision_spheres(),
            bd.sphere_centers(),
            &mut gsr.gradients[i],
            &query,
        );
        if coll {
            in_collision = true;
        }
    }
    in_collision
}

/// The `generate_distance_field: true` branch of upstream
/// `generateDistanceFieldCacheEntry`: a fresh [`PropagationDistanceField`]
/// seeded with every collision-bearing link's points *outside* the group
/// (`in_group_names`), posed at `state`'s current global link transforms,
/// plus (round 22) every such link's attached bodies' points, via
/// [`attached_body_point_decomposition`] -- upstream's own
/// `non_group_attached_body_decompositions` loop
/// (`collision_env_distance_field.cpp:945-950`).
///
/// # Deviation from upstream
///
/// Upstream iterates `robot_model_->getLinkModelsWithCollisionGeometry()`
/// (a pre-filtered list), not every link; this port folds that same filter
/// into the `link.shapes().is_empty()` half of the combined skip condition
/// below instead of pre-filtering `robot_model.link_models()` first. Same
/// observable set, but as a consequence -- upstream's own, not introduced
/// here -- an attached body on a non-group link with *no* collision
/// geometry of its own is invisible to this field too: such a link is
/// excluded from `getLinkModelsWithCollisionGeometry()` itself
/// (`collision_env_distance_field.cpp:910`), so upstream's own loop body
/// (and its attached-body sub-loop, `:927`) never runs for it at all --
/// the exclusion is the pre-filtered iteration source, not an in-loop
/// `continue`.
fn build_non_group_distance_field<'a>(
    robot_model: &RobotModel,
    state: &Posed<'_, '_>,
    in_group_names: impl Iterator<Item = &'a str>,
    link_body_decomposition_vector: &[Arc<BodyDecomposition>],
    link_body_decomposition_index_map: &HashMap<String, usize>,
    attached_bodies: &[AttachedBodyGeometry<'_>],
    config: DistanceFieldConfig,
) -> Result<PropagationDistanceField> {
    let in_group: HashSet<&str> = in_group_names.collect();

    let mut all_points: Vec<Vector3<f64>> = Vec::new();
    for link in robot_model.link_models() {
        if link.shapes().is_empty() || in_group.contains(link.name()) {
            continue;
        }
        let body_index = link_body_decomposition_index_map[link.name()];
        let pose = state.global_link_transform_at(link.link_index());
        let posed = PosedBodyPointDecomposition::with_pose(
            Arc::clone(&link_body_decomposition_vector[body_index]),
            pose,
        );
        all_points.extend_from_slice(posed.collision_points());

        for attached in attached_bodies
            .iter()
            .filter(|ab| ab.link_name == link.name())
        {
            let decomposition =
                attached_body_point_decomposition(attached, pose, config.geometry.resolution)?;
            all_points.extend(decomposition.collision_points());
        }
    }

    let mut field = PropagationDistanceField::new(
        config.geometry,
        config.max_propagation_distance,
        config.use_signed_distance_field,
    )?;
    field.add_points_to_field(&all_points);
    Ok(field)
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use moveit_geometry::{Shape, Sphere};
    use moveit_model::MeshSearchPaths;

    use super::*;

    fn pr2_model() -> RobotModel {
        let urdf_path = format!("{}/tests/fixtures/pr2.urdf", env!("CARGO_MANIFEST_DIR"));
        let srdf_path = format!("{}/tests/fixtures/pr2.srdf", env!("CARGO_MANIFEST_DIR"));
        let urdf_xml =
            std::fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
        let urdf = urdf_rs::read_file(&urdf_path).expect("pr2.urdf must parse");
        let srdf = moveit_srdf::SrdfModel::parse_file(&srdf_path).expect("pr2.srdf must parse");
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("pr2 model must build")
    }

    #[test]
    fn only_links_with_shapes_get_a_decomposition() {
        let model = pr2_model();
        let padding = LinkPaddingScale::new();
        let (vector, index_map) =
            add_link_body_decompositions(&model, 0.05, &padding, None).unwrap();

        assert_eq!(vector.len(), index_map.len());
        for link_model in model.link_models() {
            assert_eq!(
                index_map.contains_key(link_model.name()),
                !link_model.shapes().is_empty(),
                "link {} disagreed on whether it has a decomposition",
                link_model.name()
            );
        }
    }

    #[test]
    fn link_spheres_override_replaces_the_computed_spheres_for_that_link_only() {
        let model = pr2_model();
        let padding = LinkPaddingScale::new();
        let (baseline, index_map) =
            add_link_body_decompositions(&model, 0.05, &padding, None).unwrap();
        let target_link = model
            .link_models()
            .iter()
            .find(|l| !l.shapes().is_empty())
            .unwrap()
            .name()
            .to_string();

        let overridden_spheres = vec![CollisionSphere::new(
            nalgebra::Vector3::new(1.0, 2.0, 3.0),
            9.9,
        )];
        let mut overrides = HashMap::new();
        overrides.insert(target_link.clone(), overridden_spheres.clone());

        let (with_override, _) =
            add_link_body_decompositions(&model, 0.05, &padding, Some(&overrides)).unwrap();

        let target_index = index_map[&target_link];
        assert_eq!(
            with_override[target_index].collision_spheres(),
            overridden_spheres.as_slice()
        );
        // Every other link's decomposition must be untouched.
        for (name, &index) in &index_map {
            if name == &target_link {
                continue;
            }
            assert_eq!(
                with_override[index].collision_spheres(),
                baseline[index].collision_spheres(),
                "link {name} was not overridden but its spheres changed anyway"
            );
        }
    }

    fn pr2_srdf() -> moveit_srdf::SrdfModel {
        let srdf_path = format!("{}/tests/fixtures/pr2.srdf", env!("CARGO_MANIFEST_DIR"));
        moveit_srdf::SrdfModel::parse_file(&srdf_path).expect("pr2.srdf must parse")
    }

    /// One in-group variable name (moving it must never affect
    /// [`compare_cache_entry_to_state`], since it is excluded from
    /// `state_check_indices` by construction) and one out-of-group variable
    /// name (moving it is exactly what `state_check_indices` exists to
    /// detect), derived from the model/group directly rather than
    /// hardcoded, so this stays correct if the fixture's joint set changes.
    fn one_in_group_and_one_out_of_group_variable(
        model: &RobotModel,
        group_name: &str,
    ) -> (String, String) {
        let group = model.joint_model_group(group_name).unwrap();
        let active: HashSet<&str> = group
            .active_joint_indices()
            .iter()
            .flat_map(|&i| model.joint_model_at(i).variable_names())
            .map(String::as_str)
            .collect();
        let in_group = active
            .iter()
            .next()
            .expect("group must have at least one active variable")
            .to_string();
        let out_of_group = model
            .variable_names()
            .iter()
            .find(|name| !active.contains(name.as_str()))
            .expect("model must have at least one variable outside the group")
            .to_string();
        (in_group, out_of_group)
    }

    fn right_arm_cache_entry(model: &RobotModel) -> DistanceFieldCacheEntry<'_> {
        let padding = LinkPaddingScale::new();
        let link_body_decompositions =
            add_link_body_decompositions(model, 0.05, &padding, None).unwrap();
        let srdf = pr2_srdf();
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(model);
        state.set_to_default_values();
        let posed = state.update();
        generate_distance_field_cache_entry(
            "right_arm",
            &posed,
            Some(&acm),
            &link_body_decompositions,
            None,
            &[],
        )
        .unwrap()
    }

    #[test]
    fn compare_cache_entry_to_state_ignores_in_group_joint_movement() {
        let model = pr2_model();
        let dfce = right_arm_cache_entry(&model);
        let (in_group_var, _out_of_group_var) =
            one_in_group_and_one_out_of_group_variable(&model, "right_arm");

        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        state.set_variable_position(&in_group_var, 1.0).unwrap();
        let posed = state.update();

        assert!(
            compare_cache_entry_to_state(&dfce, &posed, &[]),
            "moving {in_group_var} (in-group) must not invalidate the cache entry"
        );
    }

    #[test]
    fn compare_cache_entry_to_state_detects_out_of_group_joint_movement_past_epsilon() {
        let model = pr2_model();
        let dfce = right_arm_cache_entry(&model);
        let (_in_group_var, out_of_group_var) =
            one_in_group_and_one_out_of_group_variable(&model, "right_arm");

        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        state.set_variable_position(&out_of_group_var, 1.0).unwrap();
        let posed = state.update();

        assert!(
            !compare_cache_entry_to_state(&dfce, &posed, &[]),
            "moving {out_of_group_var} (out-of-group) by 1.0 rad must invalidate the cache entry"
        );
    }

    #[test]
    fn compare_cache_entry_to_state_tolerates_out_of_group_movement_within_epsilon() {
        let model = pr2_model();
        let dfce = right_arm_cache_entry(&model);
        let (_in_group_var, out_of_group_var) =
            one_in_group_and_one_out_of_group_variable(&model, "right_arm");
        let baseline = dfce
            .state
            .variable_position(&out_of_group_var)
            .expect("variable exists");

        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        state
            .set_variable_position(&out_of_group_var, baseline + STATE_CHECK_EPSILON * 0.5)
            .unwrap();
        let posed = state.update();

        assert!(
            compare_cache_entry_to_state(&dfce, &posed, &[]),
            "moving {out_of_group_var} by half of EPSILON must not invalidate the cache entry"
        );
    }

    #[test]
    fn compare_cache_entry_to_state_rejects_a_state_from_a_different_sized_model() {
        let model = pr2_model();
        let dfce = right_arm_cache_entry(&model);

        // A minimal single-joint model has a different variable count than
        // pr2's, exercising the size-mismatch branch directly rather than
        // only ever reaching it through a shared model this test can't
        // otherwise desync.
        let urdf_xml = r#"<?xml version="1.0"?>
<robot name="one_joint">
  <link name="base"/>
  <link name="tip"/>
  <joint name="j" type="revolute">
    <parent link="base"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"#;
        let urdf: urdf_rs::Robot = urdf_rs::read_from_string(urdf_xml).unwrap();
        let srdf = moveit_srdf::SrdfModel::default();
        let other_model =
            RobotModel::from_urdf_and_srdf(&urdf, urdf_xml, &srdf, &MeshSearchPaths::none())
                .unwrap();
        let mut other_state = moveit_state::RobotState::new(&other_model);
        other_state.set_to_default_values();
        let other_posed = other_state.update();

        assert!(
            !compare_cache_entry_to_state(&dfce, &other_posed, &[]),
            "a state with a different variable count must invalidate the cache entry"
        );
    }

    #[test]
    fn compare_cache_entry_to_allowed_collision_matrix_agrees_with_its_own_generating_acm() {
        let model = pr2_model();
        let dfce = right_arm_cache_entry(&model);
        let acm = AllowedCollisionMatrix::from_srdf(&pr2_srdf());

        assert!(compare_cache_entry_to_allowed_collision_matrix(&dfce, &acm));
    }

    #[test]
    fn compare_cache_entry_to_allowed_collision_matrix_detects_a_new_disabled_self_collision() {
        let model = pr2_model();
        let dfce = right_arm_cache_entry(&model);
        let link = dfce
            .link_names
            .iter()
            .zip(&dfce.link_has_geometry)
            .find(|&(_, &has_geometry)| has_geometry)
            .map(|(name, _)| name.clone())
            .expect("right_arm must have at least one geometry-bearing link");

        let mut acm = AllowedCollisionMatrix::from_srdf(&pr2_srdf());
        acm.set_entry(&link, &link, true);

        assert!(
            !compare_cache_entry_to_allowed_collision_matrix(&dfce, &acm),
            "disabling {link}'s self-collision in a new acm must invalidate the cache entry"
        );
    }

    /// A minimal two-joint chain with a `<box>` collision shape on every
    /// link, so its own updated-link set has two geometry-bearing links
    /// without depending on `pr2.urdf`'s mesh gap (`right_arm`'s own
    /// updated-link set has only one geometry-bearing link under
    /// [`MeshSearchPaths::none`] -- see this module's doc comment -- so it
    /// cannot exercise the intra-group-pair branch at all).
    fn two_link_model_and_srdf() -> (RobotModel, moveit_srdf::SrdfModel) {
        let urdf_xml = r#"<?xml version="1.0"?>
<robot name="two_link">
  <link name="base">
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <link name="mid">
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <link name="tip">
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <joint name="j1" type="revolute">
    <parent link="base"/>
    <child link="mid"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="j2" type="revolute">
    <parent link="mid"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"#;
        let srdf_xml = r#"<?xml version="1.0"?>
<robot name="two_link">
  <group name="chain">
    <chain base_link="base" tip_link="tip"/>
  </group>
</robot>
"#;
        let urdf: urdf_rs::Robot = urdf_rs::read_from_string(urdf_xml).unwrap();
        let srdf = moveit_srdf::SrdfModel::parse_str(srdf_xml).expect("srdf must parse");
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("two_link model must build");
        moveit_test_support::assert_group_has_updated_links(&model, "chain");
        (model, srdf)
    }

    #[test]
    fn compare_cache_entry_to_allowed_collision_matrix_detects_a_new_disabled_intra_group_pair() {
        let (model, srdf) = two_link_model_and_srdf();
        let padding = LinkPaddingScale::new();
        let link_body_decompositions =
            add_link_body_decompositions(&model, 0.05, &padding, None).unwrap();
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let dfce = generate_distance_field_cache_entry(
            "chain",
            &posed,
            Some(&acm),
            &link_body_decompositions,
            None,
            &[],
        )
        .unwrap();

        let geometry_links: Vec<&String> = dfce
            .link_names
            .iter()
            .zip(&dfce.link_has_geometry)
            .filter(|&(_, &has_geometry)| has_geometry)
            .map(|(name, _)| name)
            .collect();
        assert_eq!(
            geometry_links.len(),
            2,
            "the group's updated-link set is its two non-base links (\"mid\", \"tip\"), \
             both with their own box collision shape -- \"base\" is the group's fixed \
             reference frame, not an updated link"
        );
        let (a, b) = (geometry_links[0].clone(), geometry_links[1].clone());

        let mut new_acm = AllowedCollisionMatrix::from_srdf(&srdf);
        new_acm.set_entry(&a, &b, true);

        assert!(
            !compare_cache_entry_to_allowed_collision_matrix(&dfce, &new_acm),
            "disabling collision between {a} and {b} in a new acm must invalidate the cache entry"
        );
    }

    #[test]
    fn compare_cache_entry_to_allowed_collision_matrix_rejects_a_different_size() {
        let model = pr2_model();
        let dfce = right_arm_cache_entry(&model);
        let mut acm = AllowedCollisionMatrix::from_srdf(&pr2_srdf());
        acm.set_entry("base_link", "base_footprint", true);

        // `AllowedCollisionMatrix::len` counts distinct rows, so this only
        // grows `acm`'s size if pr2.srdf's own entries didn't already cover
        // this pair -- assert that precondition rather than assume it.
        assert_ne!(
            dfce.acm.len(),
            acm.len(),
            "test setup must pick a pair that changes the row count"
        );

        assert!(!compare_cache_entry_to_allowed_collision_matrix(
            &dfce, &acm
        ));
    }

    // --- get_distance_field_cache_entry ---

    #[test]
    fn get_distance_field_cache_entry_returns_none_when_current_is_none() {
        let model = pr2_model();
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        assert!(get_distance_field_cache_entry(None, "right_arm", &posed, None, &[]).is_none());
    }

    #[test]
    fn get_distance_field_cache_entry_returns_none_on_group_name_mismatch() {
        let model = pr2_model();
        let dfce = right_arm_cache_entry(&model);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        assert!(
            get_distance_field_cache_entry(Some(&dfce), "left_arm", &posed, None, &[]).is_none(),
            "a different group name must invalidate the cache entry"
        );
    }

    #[test]
    fn get_distance_field_cache_entry_returns_none_when_the_state_check_fails() {
        let model = pr2_model();
        let dfce = right_arm_cache_entry(&model);
        let (_in_group_var, out_of_group_var) =
            one_in_group_and_one_out_of_group_variable(&model, "right_arm");

        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        state.set_variable_position(&out_of_group_var, 1.0).unwrap();
        let posed = state.update();

        assert!(
            get_distance_field_cache_entry(Some(&dfce), "right_arm", &posed, None, &[]).is_none(),
            "an out-of-group joint moved past epsilon must invalidate the cache entry"
        );
    }

    #[test]
    fn get_distance_field_cache_entry_returns_none_when_the_acm_check_fails() {
        let model = pr2_model();
        let dfce = right_arm_cache_entry(&model);
        let link = dfce
            .link_names
            .iter()
            .zip(&dfce.link_has_geometry)
            .find(|&(_, &has_geometry)| has_geometry)
            .map(|(name, _)| name.clone())
            .expect("right_arm must have at least one geometry-bearing link");
        let mut acm = AllowedCollisionMatrix::from_srdf(&pr2_srdf());
        acm.set_entry(&link, &link, true);

        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        assert!(
            get_distance_field_cache_entry(Some(&dfce), "right_arm", &posed, Some(&acm), &[])
                .is_none(),
            "an acm that disables {link}'s self-collision must invalidate the cache entry"
        );
    }

    #[test]
    fn get_distance_field_cache_entry_accepts_with_no_acm_check() {
        let model = pr2_model();
        let dfce = right_arm_cache_entry(&model);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let result = get_distance_field_cache_entry(Some(&dfce), "right_arm", &posed, None, &[]);
        assert!(
            matches!(result, Some(entry) if std::ptr::eq(entry, &dfce)),
            "acm: None must skip the acm check and still accept an otherwise-agreeing state"
        );
    }

    #[test]
    fn get_distance_field_cache_entry_returns_the_entry_when_everything_agrees() {
        let model = pr2_model();
        let dfce = right_arm_cache_entry(&model);
        let acm = AllowedCollisionMatrix::from_srdf(&pr2_srdf());
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let result =
            get_distance_field_cache_entry(Some(&dfce), "right_arm", &posed, Some(&acm), &[]);
        assert!(
            matches!(result, Some(entry) if std::ptr::eq(entry, &dfce)),
            "an agreeing state and acm must return the same cache entry unchanged"
        );
    }

    // --- compare_cache_entry_to_state: attached-body comparison ---

    fn sample_shape() -> Arc<Shape> {
        Arc::new(Shape::Sphere(Sphere::new(0.1).unwrap()))
    }

    fn right_arm_cache_entry_with_attached<'m>(
        model: &'m RobotModel,
        attached_bodies: &[AttachedBodyGeometry<'_>],
    ) -> DistanceFieldCacheEntry<'m> {
        let padding = LinkPaddingScale::new();
        let link_body_decompositions =
            add_link_body_decompositions(model, 0.05, &padding, None).unwrap();
        let srdf = pr2_srdf();
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(model);
        state.set_to_default_values();
        let posed = state.update();
        generate_distance_field_cache_entry(
            "right_arm",
            &posed,
            Some(&acm),
            &link_body_decompositions,
            None,
            attached_bodies,
        )
        .unwrap()
    }

    #[test]
    fn compare_cache_entry_to_state_accepts_an_identical_attached_body() {
        let model = pr2_model();
        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::identity()];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "gripped_box",
            link_name: "r_gripper_palm_link",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };
        let dfce = right_arm_cache_entry_with_attached(&model, &[attached]);

        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        assert!(
            compare_cache_entry_to_state(&dfce, &posed, &[attached]),
            "an unchanged attached-body slice must not invalidate the cache entry"
        );
    }

    #[test]
    fn compare_cache_entry_to_state_rejects_an_attached_body_count_mismatch() {
        let model = pr2_model();
        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::identity()];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "gripped_box",
            link_name: "r_gripper_palm_link",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };
        let dfce = right_arm_cache_entry_with_attached(&model, &[attached]);

        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        assert!(
            !compare_cache_entry_to_state(&dfce, &posed, &[]),
            "a different attached-body count must invalidate the cache entry"
        );
    }

    #[test]
    fn compare_cache_entry_to_state_rejects_an_attached_body_id_mismatch() {
        let model = pr2_model();
        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::identity()];
        let touch_links = BTreeSet::new();
        let generating = AttachedBodyGeometry {
            id: "gripped_box",
            link_name: "r_gripper_palm_link",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };
        let dfce = right_arm_cache_entry_with_attached(&model, &[generating]);
        let renamed = AttachedBodyGeometry {
            id: "a_different_id",
            ..generating
        };

        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        assert!(
            !compare_cache_entry_to_state(&dfce, &posed, &[renamed]),
            "a different attached-body id must invalidate the cache entry"
        );
    }

    #[test]
    fn compare_cache_entry_to_state_rejects_an_attached_body_touch_links_mismatch() {
        let model = pr2_model();
        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::identity()];
        let no_touch_links = BTreeSet::new();
        let generating = AttachedBodyGeometry {
            id: "gripped_box",
            link_name: "r_gripper_palm_link",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &no_touch_links,
        };
        let dfce = right_arm_cache_entry_with_attached(&model, &[generating]);

        let mut new_touch_links = BTreeSet::new();
        new_touch_links.insert("r_gripper_palm_link".to_string());
        let retouched = AttachedBodyGeometry {
            touch_links: &new_touch_links,
            ..generating
        };

        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        assert!(
            !compare_cache_entry_to_state(&dfce, &posed, &[retouched]),
            "a different attached-body touch-links set must invalidate the cache entry"
        );
    }

    #[test]
    fn compare_cache_entry_to_state_rejects_an_attached_body_shape_identity_mismatch() {
        let model = pr2_model();
        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::identity()];
        let touch_links = BTreeSet::new();
        let generating = AttachedBodyGeometry {
            id: "gripped_box",
            link_name: "r_gripper_palm_link",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };
        let dfce = right_arm_cache_entry_with_attached(&model, &[generating]);

        // Same shape value, a distinct `Arc` allocation: the comparison is
        // pointer identity (`Arc::ptr_eq`, see `AttachedBodySnapshot::matches`'s
        // doc comment), not value equality, so this must still invalidate.
        let different_shapes = vec![sample_shape()];
        let different_arc = AttachedBodyGeometry {
            shapes: &different_shapes,
            ..generating
        };

        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        assert!(
            !compare_cache_entry_to_state(&dfce, &posed, &[different_arc]),
            "an equal-valued but distinct Arc<Shape> must still invalidate the cache entry"
        );
    }

    // --- generate_distance_field_cache_entry: attached-body population (round 22) ---

    #[test]
    fn generate_distance_field_cache_entry_populates_attached_body_names_when_acm_is_some() {
        let model = pr2_model();
        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::identity()];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "gripped_box",
            // Population only runs inside the `if !link.shapes().is_empty()`
            // branch (see this function's own doc comment) -- under this
            // fixture's `MeshSearchPaths::none()`, every right_arm link but
            // this one resolves to zero shapes (mesh-only collision
            // geometry, unresolved), so this is the one link that can
            // exercise the attached-body population path at all.
            link_name: "r_gripper_motor_accelerometer_link",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };
        let dfce = right_arm_cache_entry_with_attached(&model, &[attached]);

        assert_eq!(dfce.attached_body_names, vec!["gripped_box".to_string()]);
        let link_index = dfce
            .link_names
            .iter()
            .position(|n| n == "r_gripper_motor_accelerometer_link")
            .expect("r_gripper_motor_accelerometer_link must be one of right_arm's updated links");
        assert_eq!(dfce.attached_body_link_state_indices, vec![link_index]);
    }

    #[test]
    fn generate_distance_field_cache_entry_never_populates_attached_bodies_when_acm_is_none() {
        let model = pr2_model();
        let padding = LinkPaddingScale::new();
        let link_body_decompositions =
            add_link_body_decompositions(&model, 0.05, &padding, None).unwrap();
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::identity()];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "gripped_box",
            link_name: "r_gripper_palm_link",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };

        let dfce = generate_distance_field_cache_entry(
            "right_arm",
            &posed,
            None,
            &link_body_decompositions,
            None,
            &[attached],
        )
        .unwrap();

        assert!(
            dfce.attached_body_names.is_empty(),
            "upstream only enumerates attached bodies inside the `if (acm)` \
             branch (collision_env_distance_field.cpp:775 vs the `else` at \
             :825) -- acm: None must leave attached_body_names empty even \
             though an attached body was supplied"
        );
    }

    #[test]
    fn generate_distance_field_cache_entry_excludes_an_attached_body_on_a_non_group_link() {
        let model = pr2_model();
        let padding = LinkPaddingScale::new();
        let link_body_decompositions =
            add_link_body_decompositions(&model, 0.05, &padding, None).unwrap();
        let srdf = pr2_srdf();
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::identity()];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "left_hand_box",
            link_name: "l_gripper_palm_link",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };

        let dfce = generate_distance_field_cache_entry(
            "right_arm",
            &posed,
            Some(&acm),
            &link_body_decompositions,
            None,
            &[attached],
        )
        .unwrap();

        assert!(
            !dfce.link_names.iter().any(|n| n == "l_gripper_palm_link"),
            "test precondition: l_gripper_palm_link must not be one of \
             right_arm's updated links"
        );
        assert!(
            dfce.attached_body_names.is_empty(),
            "an attached body on a link outside the group's own updated-link \
             set must never be enumerated into attached_body_names"
        );
    }

    // --- build_non_group_distance_field: attached-body obstacle points (round 22) ---

    #[test]
    fn build_non_group_distance_field_includes_a_non_group_attached_body_as_an_obstacle() {
        let model = pr2_model();
        let padding = LinkPaddingScale::new();
        let (link_body_decomposition_vector, link_body_decomposition_index_map) =
            add_link_body_decompositions(&model, 0.05, &padding, None).unwrap();
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let group = model.joint_model_group("right_arm").unwrap();
        let in_group_names: Vec<String> = group.updated_link_names().to_vec();

        // Upstream's own equivalent loop iterates
        // `getLinkModelsWithCollisionGeometry()`, not every link -- an
        // attached body on a link with no resolved collision geometry of
        // its own (most of pr2.urdf's arm links, under this fixture's
        // `MeshSearchPaths::none()`) is never visited at all, faithfully
        // reproduced by this port's own combined `link.shapes().is_empty()
        // || in_group.contains(...)` skip (collision_env_distance_field.cpp:910-925).
        // `l_gripper_motor_accelerometer_link` is one of the few left-arm
        // links with real (non-mesh) collision geometry, so it is the one
        // that can exercise this path.
        assert!(
            !model
                .link_model("l_gripper_motor_accelerometer_link")
                .unwrap()
                .shapes()
                .is_empty(),
            "test precondition: l_gripper_motor_accelerometer_link must have \
             resolved collision geometry of its own, or this test cannot \
             reach the attached-body sub-loop at all"
        );
        let link_transform = posed
            .global_link_transform("l_gripper_motor_accelerometer_link")
            .unwrap();
        // Offset well clear of the link's own origin, so the probe measures
        // the attached body's own contribution, not the link's own
        // (already-included-in-baseline) collision geometry.
        let shape_pose = Isometry3::from_parts(
            nalgebra::Translation3::new(0.3, 0.3, 0.3),
            nalgebra::UnitQuaternion::identity(),
        );
        let probe = (link_transform * shape_pose).translation.vector;
        // A grid centered on the probe point itself, so coverage does not
        // depend on where l_gripper_palm_link happens to sit relative to the
        // robot's own origin.
        let size = Vector3::new(1.0, 1.0, 1.0);
        let config = DistanceFieldConfig {
            geometry: GridGeometry::new(size, probe - 0.5 * size, 0.05).unwrap(),
            max_propagation_distance: 0.25,
            use_signed_distance_field: false,
        };

        let baseline = build_non_group_distance_field(
            &model,
            &posed,
            in_group_names.iter().map(String::as_str),
            &link_body_decomposition_vector,
            &link_body_decomposition_index_map,
            &[],
            config,
        )
        .unwrap();
        let baseline_distance = baseline.distance(probe.x, probe.y, probe.z);
        assert_eq!(
            baseline_distance, config.max_propagation_distance,
            "test precondition: with no attached body, `probe` must read back \
             as the field's own uninitialized/max distance -- otherwise some \
             other non-group link's own geometry already occupies this \
             point, and this test cannot isolate the attached body's own \
             contribution"
        );

        let shapes = vec![sample_shape()];
        let shape_poses = vec![shape_pose];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "left_hand_box",
            link_name: "l_gripper_motor_accelerometer_link",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };
        let with_attached = build_non_group_distance_field(
            &model,
            &posed,
            in_group_names.iter().map(String::as_str),
            &link_body_decomposition_vector,
            &link_body_decomposition_index_map,
            &[attached],
            config,
        )
        .unwrap();
        let with_attached_distance = with_attached.distance(probe.x, probe.y, probe.z);

        assert!(
            with_attached_distance < baseline_distance,
            "an attached body on a non-group link must add obstacle points \
             at its own posed location, shrinking the distance there \
             relative to a field built with no attached bodies at all \
             (baseline {baseline_distance}, with attached body \
             {with_attached_distance})"
        );
    }

    // --- group_state_representation / update_group_state_representation_state ---

    #[test]
    fn update_group_state_representation_state_skips_links_without_geometry() {
        let model = pr2_model();
        let dfce = right_arm_cache_entry(&model);
        assert!(
            dfce.link_has_geometry.iter().any(|&has| !has),
            "test precondition: right_arm's updated-link set must include at least one link \
             without geometry (pr2.urdf's mesh gap under MeshSearchPaths::none, per this \
             module's doc comment)"
        );
        let padding = LinkPaddingScale::new();
        let (link_body_decomposition_vector, _) =
            add_link_body_decompositions(&model, 0.05, &padding, None).unwrap();

        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let mut gsr = group_state_representation(
            &dfce,
            &posed,
            &link_body_decomposition_vector,
            0.05,
            0.25,
            false,
            &[],
        )
        .unwrap();

        update_group_state_representation_state(&posed, &mut gsr, &[]).unwrap();

        for (i, &has_geometry) in dfce.link_has_geometry.iter().enumerate() {
            if !has_geometry {
                assert!(
                    gsr.link_body_decompositions[i].is_none(),
                    "link {} has no geometry but got a decomposition after update",
                    dfce.link_names[i]
                );
                assert!(
                    gsr.link_distance_fields[i].is_none(),
                    "link {} has no geometry but got a distance field after update",
                    dfce.link_names[i]
                );
            }
        }
    }

    #[test]
    fn update_group_state_representation_state_agrees_with_a_fresh_rebuild_at_the_same_pose() {
        let model = pr2_model();
        let dfce = right_arm_cache_entry(&model);
        let padding = LinkPaddingScale::new();
        let (link_body_decomposition_vector, _) =
            add_link_body_decompositions(&model, 0.05, &padding, None).unwrap();
        let (in_group_var, _out_of_group_var) =
            one_in_group_and_one_out_of_group_variable(&model, "right_arm");

        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let default_posed = state.update();

        let mut gsr = group_state_representation(
            &dfce,
            &default_posed,
            &link_body_decomposition_vector,
            0.05,
            0.25,
            false,
            &[],
        )
        .unwrap();

        // Move an in-group joint away, then update back to the exact same
        // (default) pose the group_state_representation above was already
        // built at -- an update-to-and-fro must land on the same sphere
        // centers a fresh build at that pose computes, since `update_pose`
        // recomputes every sphere center from the link-relative geometry on
        // each call rather than accumulating a delta (see
        // `PosedBodySphereDecomposition::update_pose`).
        let mut moved = moveit_state::RobotState::new(&model);
        moved.set_to_default_values();
        moved.set_variable_position(&in_group_var, 0.3).unwrap();
        let moved_posed = moved.update();
        update_group_state_representation_state(&moved_posed, &mut gsr, &[]).unwrap();
        update_group_state_representation_state(&default_posed, &mut gsr, &[]).unwrap();

        let rebuilt = group_state_representation(
            &dfce,
            &default_posed,
            &link_body_decomposition_vector,
            0.05,
            0.25,
            false,
            &[],
        )
        .unwrap();

        for i in 0..dfce.link_names.len() {
            match (
                &gsr.link_body_decompositions[i],
                &rebuilt.link_body_decompositions[i],
            ) {
                (Some(updated), Some(fresh)) => {
                    assert_eq!(
                        updated.sphere_centers(),
                        fresh.sphere_centers(),
                        "link {} disagreed between update-to-and-fro and a fresh rebuild",
                        dfce.link_names[i]
                    );
                }
                (None, None) => {}
                _ => panic!(
                    "link {} disagreed on whether it has geometry between update and rebuild",
                    dfce.link_names[i]
                ),
            }
        }
    }

    // --- group_state_representation / update_group_state_representation_state: attached bodies (round 22) ---

    #[test]
    fn group_state_representation_builds_a_gradient_slot_for_an_attached_body() {
        let model = pr2_model();
        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::identity()];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "gripped_box",
            // See `generate_distance_field_cache_entry_populates_attached_body_names_when_acm_is_some`
            // for why this must be a geometry-bearing link.
            link_name: "r_gripper_motor_accelerometer_link",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };
        let dfce = right_arm_cache_entry_with_attached(&model, &[attached]);
        let padding = LinkPaddingScale::new();
        let (link_body_decomposition_vector, _) =
            add_link_body_decompositions(&model, 0.05, &padding, None).unwrap();
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let gsr = group_state_representation(
            &dfce,
            &posed,
            &link_body_decomposition_vector,
            0.05,
            0.25,
            false,
            &[attached],
        )
        .unwrap();

        assert_eq!(gsr.attached_body_decompositions.len(), 1);
        assert_eq!(
            gsr.gradients.len(),
            dfce.link_names.len() + 1,
            "the attached body's GradientInfo slot must be appended after \
             every link's own slot"
        );
        let attached_gradient = gsr.gradients.last().unwrap();
        assert!(
            !attached_gradient.sphere_locations.is_empty(),
            "unlike the link loop above it, group_state_representation's \
             attached-body loop must set sphere_locations at fresh-build \
             time (collision_env_distance_field.cpp:1246-1247) -- a real \
             upstream asymmetry this port preserves"
        );
        assert_eq!(
            attached_gradient.sphere_locations.len(),
            gsr.attached_body_decompositions[0].sphere_centers().len()
        );
    }

    #[test]
    fn group_state_representation_attached_body_with_multiple_shapes_at_distinct_poses() {
        let model = pr2_model();
        let shapes = vec![sample_shape(), sample_shape()];
        let shape_poses = vec![
            Isometry3::identity(),
            Isometry3::from_parts(
                nalgebra::Translation3::new(0.2, 0.0, 0.0),
                nalgebra::UnitQuaternion::identity(),
            ),
        ];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "two_part_tool",
            link_name: "r_gripper_motor_accelerometer_link",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };
        let dfce = right_arm_cache_entry_with_attached(&model, &[attached]);
        let padding = LinkPaddingScale::new();
        let (link_body_decomposition_vector, _) =
            add_link_body_decompositions(&model, 0.05, &padding, None).unwrap();
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let gsr = group_state_representation(
            &dfce,
            &posed,
            &link_body_decomposition_vector,
            0.05,
            0.25,
            false,
            &[attached],
        )
        .unwrap();

        let decomposition = &gsr.attached_body_decompositions[0];
        assert_eq!(
            decomposition.collision_spheres().len(),
            gsr.gradients.last().unwrap().sphere_locations.len()
        );
        // Both shapes are identical spheres a resolution apart; a
        // decomposition that dropped the second shape's pose (e.g. by
        // reusing the first shape's pose for both) would still produce
        // exactly one shape's worth of spheres, all clustered at the first
        // pose.
        let centers = decomposition.sphere_centers();
        assert!(
            centers.iter().any(|c| (c - centers[0]).norm() > 0.05),
            "a two-shape attached body posed a resolution apart must \
             produce spheres spread across both poses, not all clustered at \
             one"
        );
    }

    #[test]
    fn group_state_representation_errors_when_attached_bodies_slice_is_missing_a_tracked_id() {
        let model = pr2_model();
        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::identity()];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "gripped_box",
            link_name: "r_gripper_motor_accelerometer_link",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };
        let dfce = right_arm_cache_entry_with_attached(&model, &[attached]);
        assert_eq!(
            dfce.attached_body_names.len(),
            1,
            "test precondition: dfce must actually track the attached body, \
             or the lookup this test targets never runs"
        );
        let padding = LinkPaddingScale::new();
        let (link_body_decomposition_vector, _) =
            add_link_body_decompositions(&model, 0.05, &padding, None).unwrap();
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let result = group_state_representation(
            &dfce,
            &posed,
            &link_body_decomposition_vector,
            0.05,
            0.25,
            false,
            &[],
        );

        assert!(
            result.is_err(),
            "dfce.attached_body_names names a body the caller-supplied slice \
             no longer carries -- upstream's own equivalent lookup has no \
             null check at all here (collision_env_distance_field.cpp:1239) \
             and would dereference null; this port's closest safe equivalent \
             is a hard error, not a silent skip"
        );
    }

    #[test]
    fn update_group_state_representation_state_agrees_with_a_fresh_rebuild_for_an_attached_body() {
        let model = pr2_model();
        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::identity()];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "gripped_box",
            link_name: "r_gripper_motor_accelerometer_link",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };
        let dfce = right_arm_cache_entry_with_attached(&model, &[attached]);
        let padding = LinkPaddingScale::new();
        let (link_body_decomposition_vector, _) =
            add_link_body_decompositions(&model, 0.05, &padding, None).unwrap();
        let (in_group_var, _out_of_group_var) =
            one_in_group_and_one_out_of_group_variable(&model, "right_arm");

        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let default_posed = state.update();

        let mut gsr = group_state_representation(
            &dfce,
            &default_posed,
            &link_body_decomposition_vector,
            0.05,
            0.25,
            false,
            &[attached],
        )
        .unwrap();

        let mut moved = moveit_state::RobotState::new(&model);
        moved.set_to_default_values();
        moved.set_variable_position(&in_group_var, 0.3).unwrap();
        let moved_posed = moved.update();
        update_group_state_representation_state(&moved_posed, &mut gsr, &[attached]).unwrap();
        update_group_state_representation_state(&default_posed, &mut gsr, &[attached]).unwrap();

        let rebuilt = group_state_representation(
            &dfce,
            &default_posed,
            &link_body_decomposition_vector,
            0.05,
            0.25,
            false,
            &[attached],
        )
        .unwrap();

        assert_eq!(
            gsr.attached_body_decompositions[0].sphere_centers(),
            rebuilt.attached_body_decompositions[0].sphere_centers(),
            "the attached body disagreed between update-to-and-fro and a \
             fresh rebuild"
        );
    }

    #[test]
    fn update_group_state_representation_state_reproduces_upstreams_suspicious_attached_body_count_check()
     {
        let model = pr2_model();
        let shapes = vec![sample_shape(), sample_shape()];
        let shape_poses = vec![
            Isometry3::identity(),
            Isometry3::from_parts(
                nalgebra::Translation3::new(0.2, 0.0, 0.0),
                nalgebra::UnitQuaternion::identity(),
            ),
        ];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "two_part_tool",
            link_name: "r_gripper_motor_accelerometer_link",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };
        let dfce = right_arm_cache_entry_with_attached(&model, &[attached]);
        // Precondition for upstream's own bug to trip: exactly one attached
        // body (the outer vector's length, `gsr->attached_body_decompositions_.size()`)
        // whose own shape count (2) differs from that outer length --
        // upstream's own suspicious check
        // (`collision_env_distance_field.cpp:1132-1137`, upstream's own
        // comment: "TODO: This logic for checking attached body count might
        // be incorrect") compares those two unrelated counts, not this
        // decomposition's own shape count against itself.
        assert_eq!(dfce.attached_body_names.len(), 1);
        assert_eq!(attached.shapes.len(), 2);

        let padding = LinkPaddingScale::new();
        let (link_body_decomposition_vector, _) =
            add_link_body_decompositions(&model, 0.05, &padding, None).unwrap();
        let (in_group_var, _out_of_group_var) =
            one_in_group_and_one_out_of_group_variable(&model, "right_arm");

        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let default_posed = state.update();

        let mut gsr = group_state_representation(
            &dfce,
            &default_posed,
            &link_body_decomposition_vector,
            0.05,
            0.25,
            false,
            &[attached],
        )
        .unwrap();
        let before = gsr.attached_body_decompositions[0]
            .sphere_centers()
            .to_vec();

        let mut moved = moveit_state::RobotState::new(&model);
        moved.set_to_default_values();
        moved.set_variable_position(&in_group_var, 0.5).unwrap();
        let moved_posed = moved.update();

        let fresh_at_moved = group_state_representation(
            &dfce,
            &moved_posed,
            &link_body_decomposition_vector,
            0.05,
            0.25,
            false,
            &[attached],
        )
        .unwrap();
        assert_ne!(
            fresh_at_moved.attached_body_decompositions[0].sphere_centers(),
            before.as_slice(),
            "test precondition: moving {in_group_var} must actually change \
             r_gripper_motor_accelerometer_link's transform, or this test \
             cannot distinguish 'the buggy check skipped the update' from \
             'the update was a no-op anyway'"
        );

        update_group_state_representation_state(&moved_posed, &mut gsr, &[attached]).unwrap();

        assert_eq!(
            gsr.attached_body_decompositions[0].sphere_centers(),
            before.as_slice(),
            "1 (outer attached-body count) != 2 (this attached body's own \
             shape count) must trip upstream's suspicious `continue` and \
             skip the re-pose entirely, even though the joint actually moved"
        );
    }

    // --- DistanceFieldCollisionCache::generate_collision_checking_structures ---

    /// Matching `oracle_default_distance_field_config` in
    /// `collision_env_distance_field_parity.rs`, at a coarser resolution for
    /// unit-test speed -- these tests only exercise cache-invalidation
    /// decisions, not distance-field accuracy.
    fn small_distance_field_config() -> DistanceFieldConfig {
        let size = Vector3::new(3.0, 3.0, 4.0);
        let origin_center = Vector3::new(0.0, 0.0, 0.0);
        DistanceFieldConfig {
            geometry: GridGeometry::new(size, origin_center - 0.5 * size, 0.05).unwrap(),
            max_propagation_distance: 0.25,
            use_signed_distance_field: false,
        }
    }

    fn right_arm_collision_cache(model: &RobotModel) -> DistanceFieldCollisionCache<'_> {
        let padding = LinkPaddingScale::new();
        let link_body_decompositions =
            add_link_body_decompositions(model, 0.05, &padding, None).unwrap();
        DistanceFieldCollisionCache::new(
            link_body_decompositions,
            small_distance_field_config(),
            0.0,
        )
    }

    #[test]
    fn generate_collision_checking_structures_builds_a_fresh_entry_on_first_call() {
        let model = pr2_model();
        let mut cache = right_arm_collision_cache(&model);
        let srdf = pr2_srdf();
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let gsr = cache
            .generate_collision_checking_structures("right_arm", &posed, Some(&acm), &[], false)
            .unwrap();

        assert_eq!(gsr.dfce.group_name, "right_arm");
        assert!(
            gsr.dfce.distance_field.is_none(),
            "generate_distance_field: false must not build a field"
        );
    }

    #[test]
    fn generate_collision_checking_structures_rebuilds_when_a_distance_field_becomes_required() {
        let model = pr2_model();
        let mut cache = right_arm_collision_cache(&model);
        let srdf = pr2_srdf();
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let first = cache
            .generate_collision_checking_structures("right_arm", &posed, Some(&acm), &[], false)
            .unwrap();
        assert!(first.dfce.distance_field.is_none());

        let second = cache
            .generate_collision_checking_structures("right_arm", &posed, Some(&acm), &[], true)
            .unwrap();
        assert!(
            second.dfce.distance_field.is_some(),
            "a cached entry with no distance field must be rebuilt when one is required"
        );
    }

    #[test]
    fn generate_collision_checking_structures_switches_group_without_stale_data() {
        let model = pr2_model();
        let mut cache = right_arm_collision_cache(&model);
        let srdf = pr2_srdf();
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        cache
            .generate_collision_checking_structures("right_arm", &posed, Some(&acm), &[], false)
            .unwrap();
        let gsr = cache
            .generate_collision_checking_structures("left_arm", &posed, Some(&acm), &[], false)
            .unwrap();

        assert_eq!(gsr.dfce.group_name, "left_arm");
        assert!(
            gsr.dfce
                .link_names
                .iter()
                .all(|name| !name.starts_with('r')),
            "a left_arm cache entry must not carry right_arm's own link names: {:?}",
            gsr.dfce.link_names
        );
    }

    #[test]
    fn generate_collision_checking_structures_reflects_a_changed_acm_rather_than_the_stale_cache() {
        let model = pr2_model();
        let mut cache = right_arm_collision_cache(&model);
        let srdf = pr2_srdf();
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let permissive_acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let baseline = cache
            .generate_collision_checking_structures(
                "right_arm",
                &posed,
                Some(&permissive_acm),
                &[],
                false,
            )
            .unwrap();
        let link = baseline
            .dfce
            .link_names
            .iter()
            .zip(&baseline.dfce.link_has_geometry)
            .find(|&(_, &has_geometry)| has_geometry)
            .map(|(name, _)| name.clone())
            .expect("right_arm must have at least one geometry-bearing link");
        let link_index = baseline
            .dfce
            .link_names
            .iter()
            .position(|n| n == &link)
            .unwrap();
        let baseline_self_collision = baseline.dfce.self_collision_enabled[link_index];

        let mut restrictive_acm = AllowedCollisionMatrix::from_srdf(&srdf);
        restrictive_acm.set_entry(&link, &link, true);

        let updated = cache
            .generate_collision_checking_structures(
                "right_arm",
                &posed,
                Some(&restrictive_acm),
                &[],
                false,
            )
            .unwrap();

        assert!(
            baseline_self_collision,
            "the permissive acm must leave {link}'s self-collision enabled"
        );
        assert!(
            !updated.dfce.self_collision_enabled[link_index],
            "a new acm disabling {link}'s self-collision must invalidate the cached entry, \
             not serve the permissive acm's stale bit"
        );
    }

    #[test]
    fn generate_collision_checking_structures_agrees_with_a_fresh_generate_and_represent_call() {
        let model = pr2_model();
        let padding = LinkPaddingScale::new();
        let link_body_decompositions =
            add_link_body_decompositions(&model, 0.05, &padding, None).unwrap();
        let srdf = pr2_srdf();
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let config = small_distance_field_config();

        let mut cache =
            DistanceFieldCollisionCache::new(link_body_decompositions.clone(), config, 0.0);
        let via_cache = cache
            .generate_collision_checking_structures("right_arm", &posed, Some(&acm), &[], false)
            .unwrap();

        let direct_dfce = generate_distance_field_cache_entry(
            "right_arm",
            &posed,
            Some(&acm),
            &link_body_decompositions,
            None,
            &[],
        )
        .unwrap();
        let direct_gsr = group_state_representation(
            &direct_dfce,
            &posed,
            &link_body_decompositions.0,
            config.geometry.resolution,
            config.max_propagation_distance,
            config.use_signed_distance_field,
            &[],
        )
        .unwrap();

        assert_eq!(via_cache.dfce.link_names, direct_gsr.dfce.link_names);
        assert_eq!(
            via_cache.dfce.link_has_geometry,
            direct_gsr.dfce.link_has_geometry
        );
        for i in 0..via_cache.dfce.link_names.len() {
            match (
                &via_cache.link_body_decompositions[i],
                &direct_gsr.link_body_decompositions[i],
            ) {
                (Some(a), Some(b)) => {
                    assert_eq!(
                        a.sphere_centers(),
                        b.sphere_centers(),
                        "link {} disagreed between the cache path and a fresh direct call",
                        via_cache.dfce.link_names[i]
                    );
                }
                (None, None) => {}
                _ => panic!(
                    "link {} disagreed on whether it has geometry",
                    via_cache.dfce.link_names[i]
                ),
            }
        }
    }

    #[test]
    fn generate_collision_checking_structures_rebuilds_on_out_of_group_movement_past_epsilon() {
        let model = pr2_model();
        let mut cache = right_arm_collision_cache(&model);
        let srdf = pr2_srdf();
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let (_in_group_var, out_of_group_var) =
            one_in_group_and_one_out_of_group_variable(&model, "right_arm");

        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let baseline = state.update();
        let baseline_gsr = cache
            .generate_collision_checking_structures("right_arm", &baseline, Some(&acm), &[], false)
            .unwrap();
        let baseline_state_values = baseline_gsr.dfce.state_values.clone();

        let mut moved = moveit_state::RobotState::new(&model);
        moved.set_to_default_values();
        moved
            .set_variable_position(&out_of_group_var, STATE_CHECK_EPSILON * 2.0)
            .unwrap();
        let moved_posed = moved.update();
        let moved_gsr = cache
            .generate_collision_checking_structures(
                "right_arm",
                &moved_posed,
                Some(&acm),
                &[],
                false,
            )
            .unwrap();

        assert_ne!(
            baseline_state_values, moved_gsr.dfce.state_values,
            "moving {out_of_group_var} past STATE_CHECK_EPSILON must invalidate and rebuild \
             the cached entry rather than keep serving the pre-move state_values"
        );
    }

    // --- DistanceFieldCollisionCache::check_self_collision / check_collision /
    //     check_robot_collision / get_collision_gradients / get_all_collisions ---

    /// `two_link_model_and_srdf`'s "mid"/"tip" links carry no `<origin>` on
    /// either joint, so at the all-zero default pose "base", "mid", and
    /// "tip" are all exactly coincident -- a cheap, deterministic collision
    /// fixture, used throughout the tests below instead of trying to pose a
    /// real PR2 arm into self-collision.
    fn chain_collision_cache(model: &RobotModel) -> DistanceFieldCollisionCache<'_> {
        let padding = LinkPaddingScale::new();
        let link_body_decompositions =
            add_link_body_decompositions(model, 0.02, &padding, None).unwrap();
        DistanceFieldCollisionCache::new(
            link_body_decompositions,
            small_distance_field_config(),
            0.0,
        )
    }

    /// A [`PropagationDistanceField`] seeded with exactly one point, built
    /// from the same [`small_distance_field_config`] every test below
    /// checks the robot against.
    fn point_environment_distance_field(point: Vector3<f64>) -> PropagationDistanceField {
        let config = small_distance_field_config();
        let mut field = PropagationDistanceField::new(
            config.geometry,
            config.max_propagation_distance,
            config.use_signed_distance_field,
        )
        .unwrap();
        field.add_points_to_field(&[point]);
        field
    }

    #[test]
    fn check_self_collision_reports_no_collision_for_a_well_separated_group() {
        let model = pr2_model();
        let mut cache = right_arm_collision_cache(&model);
        let srdf = pr2_srdf();
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let req = CollisionRequest {
            group_name: Some("right_arm".to_string()),
            ..CollisionRequest::default()
        };

        let (res, _gsr) = cache
            .check_self_collision(&req, &posed, Some(&acm), &[])
            .unwrap();
        assert!(
            !res.collision,
            "the PR2 right arm's default pose has no overlapping links, and pr2.srdf's own \
             disable_collisions entries cover every pair that would otherwise be flagged"
        );
    }

    #[test]
    fn check_self_collision_intra_group_collision_survives_when_the_self_check_is_disabled() {
        let (model, srdf) = two_link_model_and_srdf();
        let mut cache = chain_collision_cache(&model);
        let mut acm = AllowedCollisionMatrix::from_srdf(&srdf);
        // Disable each link's own self_collision_enabled bit (link vs
        // itself), isolating this assertion to `get_intra_group_collisions`
        // (mid vs tip) rather than `get_self_collisions` (mid/tip vs the
        // group's own aggregate field, built from "base" -- also coincident
        // at the default pose, and would otherwise leave it ambiguous which
        // function actually found the collision).
        acm.set_entry("mid", "mid", true);
        acm.set_entry("tip", "tip", true);

        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            ..CollisionRequest::default()
        };

        let (res, _gsr) = cache
            .check_self_collision(&req, &posed, Some(&acm), &[])
            .unwrap();
        assert!(
            res.collision,
            "mid and tip are exactly coincident at the default pose; disabling each link's own \
             self_collision_enabled bit must not also suppress the intra-group check between \
             them"
        );
    }

    #[test]
    fn check_self_collision_max_contacts_caps_the_total_recorded_across_pairs() {
        let (model, srdf) = two_link_model_and_srdf();
        let mut cache = chain_collision_cache(&model);
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            contacts: true,
            max_contacts: 1,
            max_contacts_per_pair: 100,
            ..CollisionRequest::default()
        };

        let (res, _gsr) = cache
            .check_self_collision(&req, &posed, Some(&acm), &[])
            .unwrap();
        assert!(res.collision);
        let contacts = res.contacts.expect("contacts requested");
        assert!(
            contacts.count() <= 1,
            "req.max_contacts == 1 must cap the total number of contacts recorded, even though \
             mid/tip being exactly coincident gives every sphere pair a contact to report"
        );
    }

    #[test]
    fn check_collision_short_circuits_before_checking_the_environment_once_self_collision_is_found()
    {
        let (model, srdf) = two_link_model_and_srdf();
        let mut cache = chain_collision_cache(&model);
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let env = point_environment_distance_field(Vector3::new(0.0, 0.0, 0.0));

        // `done` (both here and upstream) means "the contact budget is
        // spent", not merely "a collision was found" -- with `max_contacts`
        // set generously, `get_self_collisions` keeps accumulating contacts
        // without becoming `done`, and `get_environment_collisions` would
        // still run. `max_contacts: 1` makes the very first self-collision
        // contact spend the whole budget, so `done` really does go true
        // after the self-collision phase alone.
        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            contacts: true,
            max_contacts: 1,
            max_contacts_per_pair: 10,
            ..CollisionRequest::default()
        };

        let (res, _gsr) = cache
            .check_collision(&req, &posed, Some(&acm), &[], &env)
            .unwrap();
        assert!(res.collision);
        let contacts = res.contacts.expect("contacts requested");
        assert_eq!(
            contacts.count(),
            1,
            "max_contacts == 1 must cap the total recorded, whichever phase reports it"
        );
        assert!(
            !contacts.by_pair.keys().any(|(_, b)| b == "environment"),
            "get_self_collisions already spends the whole max_contacts budget, so \
             check_collision's `if !done` guard must skip get_environment_collisions entirely \
             -- no \"environment\" contact pair, even though the seeded env field would also \
             collide"
        );
    }

    #[test]
    fn get_all_collisions_checks_the_environment_even_when_self_collision_already_found() {
        let (model, srdf) = two_link_model_and_srdf();
        let mut cache = chain_collision_cache(&model);
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let env = point_environment_distance_field(Vector3::new(0.0, 0.0, 0.0));

        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            contacts: true,
            max_contacts: 10,
            max_contacts_per_pair: 10,
            ..CollisionRequest::default()
        };

        let (res, _gsr) = cache
            .get_all_collisions(&req, &posed, Some(&acm), &[], &env)
            .unwrap();
        assert!(res.collision);
        let contacts = res.contacts.expect("contacts requested");
        assert!(
            contacts.by_pair.keys().any(|(_, b)| b == "environment"),
            "unlike check_collision, get_all_collisions has no `if !done` guard between \
             phases -- it must still record the environment contact even though a self-collision \
             was already found"
        );
    }

    #[test]
    fn check_robot_collision_ignores_a_self_collision_and_only_checks_the_environment() {
        let (model, srdf) = two_link_model_and_srdf();
        let mut cache = chain_collision_cache(&model);
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let config = small_distance_field_config();
        let empty_env = PropagationDistanceField::new(
            config.geometry,
            config.max_propagation_distance,
            config.use_signed_distance_field,
        )
        .unwrap();

        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            ..CollisionRequest::default()
        };

        let (res, _gsr) = cache
            .check_robot_collision(&req, &posed, Some(&acm), &[], &empty_env)
            .unwrap();
        assert!(
            !res.collision,
            "mid and tip are coincident (a genuine self-collision), but check_robot_collision \
             must only consult the environment field -- with no points in it, this must report \
             no collision at all"
        );
    }

    #[test]
    fn check_robot_collision_detects_an_environment_collision() {
        let (model, srdf) = two_link_model_and_srdf();
        let mut cache = chain_collision_cache(&model);
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let env = point_environment_distance_field(Vector3::new(0.0, 0.0, 0.0));

        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            ..CollisionRequest::default()
        };

        let (res, _gsr) = cache
            .check_robot_collision(&req, &posed, Some(&acm), &[], &env)
            .unwrap();
        assert!(
            res.collision,
            "an environment point placed exactly at the coincident mid/tip origin must \
             register as an environment collision"
        );
    }

    #[test]
    fn get_collision_gradients_reports_a_finite_distance_for_a_nearby_environment_point() {
        let (model, srdf) = two_link_model_and_srdf();
        let mut cache = chain_collision_cache(&model);
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let env = point_environment_distance_field(Vector3::new(0.0, 0.0, 0.0));

        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            ..CollisionRequest::default()
        };

        let gsr = cache
            .get_collision_gradients(&req, &posed, Some(&acm), &[], &env)
            .unwrap();
        let mid_index = gsr
            .dfce
            .link_names
            .iter()
            .position(|n| n == "mid")
            .expect("\"mid\" is one of chain's updated links");
        assert!(
            gsr.gradients[mid_index].closest_distance < f64::MAX,
            "an environment point placed at mid's own coincident pose must produce a finite \
             closest_distance, not the default-initialized f64::MAX"
        );
    }

    // --- round 23: attached-body paths through get_self_collisions /
    //     get_intra_group_collisions / get_intra_group_proximity_gradients /
    //     get_environment_collisions. Zero attached bodies is already
    //     exercised throughout the tests above (every `&[]` call); these
    //     cover one, many, in-group vs out-of-group, acm None vs Some,
    //     link-vs-attached and attached-vs-attached pairs, touch_links, and
    //     colliding vs not, per boundary rather than per narrative. ---

    #[test]
    fn get_self_collisions_detects_an_attached_body_on_an_in_group_link() {
        let (model, srdf) = two_link_model_and_srdf();
        let mut cache = chain_collision_cache(&model);
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::identity()];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "gripped",
            link_name: "mid",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };

        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            contacts: true,
            max_contacts: 100,
            max_contacts_per_pair: 100,
            ..CollisionRequest::default()
        };

        let (res, _gsr) = cache
            .check_self_collision(&req, &posed, Some(&acm), &[attached])
            .unwrap();
        assert!(res.collision);
        let contacts = res.contacts.expect("contacts requested");
        let pair = contacts
            .by_pair
            .get(&("gripped".to_string(), "self".to_string()))
            .expect(
                "an attached body coincident with the group's own aggregate field must be \
                 reported by get_self_collisions against \"self\", same as a link would be, now \
                 that its loop bound covers attached_body_names too",
            );
        assert_eq!(pair[0].body_type_1, BodyType::RobotAttached);
    }

    #[test]
    fn get_self_collisions_ignores_an_attached_body_placed_away_from_the_self_field() {
        let (model, srdf) = two_link_model_and_srdf();
        let mut cache = chain_collision_cache(&model);
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::translation(10.0, 0.0, 0.0)];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "gripped",
            link_name: "mid",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };

        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            contacts: true,
            max_contacts: 100,
            max_contacts_per_pair: 100,
            ..CollisionRequest::default()
        };

        let (res, _gsr) = cache
            .check_self_collision(&req, &posed, Some(&acm), &[attached])
            .unwrap();
        if let Some(contacts) = &res.contacts {
            assert!(
                !contacts
                    .by_pair
                    .contains_key(&("gripped".to_string(), "self".to_string())),
                "an attached body 10m from the self field's only obstacle (\"base\") must not \
                 be reported as a self-collision"
            );
        }
    }

    #[test]
    fn attached_body_on_an_out_of_group_link_is_invisible_to_collision_checks() {
        let (model, srdf) = two_link_model_and_srdf();
        let mut cache = chain_collision_cache(&model);
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::identity()];
        let touch_links = BTreeSet::new();
        // "base" is "chain"'s fixed reference link, not one of its two
        // updated links ("mid", "tip") -- out-of-group, even though it is
        // exactly coincident with mid/tip at the default pose.
        let attached = AttachedBodyGeometry {
            id: "gripped",
            link_name: "base",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };

        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            contacts: true,
            max_contacts: 100,
            max_contacts_per_pair: 100,
            ..CollisionRequest::default()
        };

        let (res, gsr) = cache
            .check_self_collision(&req, &posed, Some(&acm), &[attached])
            .unwrap();
        assert!(
            gsr.dfce.attached_body_names.is_empty(),
            "generate_distance_field_cache_entry's per-link loop only ever visits \"chain\"'s \
             own updated links (:775-825) -- \"base\" is never one of them, so \"gripped\" must \
             never be enumerated into attached_body_names"
        );
        if let Some(contacts) = &res.contacts {
            assert!(
                contacts
                    .by_pair
                    .keys()
                    .all(|(a, b)| a != "gripped" && b != "gripped"),
                "an attached body on an out-of-group link must never appear in a contact, even \
                 though it is geometrically coincident with mid/tip/base and mid/tip do \
                 genuinely self-collide"
            );
        }
    }

    #[test]
    fn attached_body_is_invisible_when_acm_is_none() {
        let (model, _srdf) = two_link_model_and_srdf();
        let mut cache = chain_collision_cache(&model);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::identity()];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "gripped",
            link_name: "mid",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };

        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            contacts: true,
            max_contacts: 100,
            max_contacts_per_pair: 100,
            ..CollisionRequest::default()
        };

        let (res, gsr) = cache
            .check_self_collision(&req, &posed, None, &[attached])
            .unwrap();
        assert!(
            gsr.dfce.attached_body_names.is_empty(),
            "upstream only enumerates attached bodies inside the `if (acm)` branch \
             (collision_env_distance_field.cpp:775 vs its `else` at :825) -- acm: None must \
             leave attached_body_names empty even for an attached body on an in-group link"
        );
        if let Some(contacts) = &res.contacts {
            assert!(
                contacts
                    .by_pair
                    .keys()
                    .all(|(a, b)| a != "gripped" && b != "gripped"),
                "with acm: None, \"gripped\" never enters attached_body_names, so it must never \
                 appear in a contact even though it is on an in-group link and geometrically \
                 coincident with mid"
            );
        }
    }

    #[test]
    fn get_intra_group_collisions_detects_a_link_vs_attached_pair() {
        let (model, srdf) = two_link_model_and_srdf();
        let mut cache = chain_collision_cache(&model);
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::identity()];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "gripped",
            link_name: "tip",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };

        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            contacts: true,
            max_contacts: 100,
            max_contacts_per_pair: 100,
            ..CollisionRequest::default()
        };

        let (res, _gsr) = cache
            .check_self_collision(&req, &posed, Some(&acm), &[attached])
            .unwrap();
        assert!(res.collision);
        let contacts = res.contacts.expect("contacts requested");
        let pair = contacts
            .by_pair
            .get(&("mid".to_string(), "gripped".to_string()))
            .expect(
                "\"gripped\" is attached to \"tip\", not \"mid\" -- a contact between \"mid\" \
                 (a link) and \"gripped\" (an attached body on a different link) can only come \
                 from get_intra_group_collisions's link-vs-attached branch",
            );
        assert_eq!(pair[0].body_type_1, BodyType::RobotLink);
        assert_eq!(pair[0].body_type_2, BodyType::RobotAttached);
    }

    #[test]
    fn get_intra_group_collisions_detects_an_attached_vs_attached_pair() {
        let (model, srdf) = two_link_model_and_srdf();
        let mut cache = chain_collision_cache(&model);
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let shapes_a = vec![sample_shape()];
        let shape_poses_a = vec![Isometry3::identity()];
        let touch_links_a = BTreeSet::new();
        let attached_a = AttachedBodyGeometry {
            id: "gripped_a",
            link_name: "mid",
            shapes: &shapes_a,
            shape_poses: &shape_poses_a,
            touch_links: &touch_links_a,
        };
        let shapes_b = vec![sample_shape()];
        let shape_poses_b = vec![Isometry3::identity()];
        let touch_links_b = BTreeSet::new();
        let attached_b = AttachedBodyGeometry {
            id: "gripped_b",
            link_name: "tip",
            shapes: &shapes_b,
            shape_poses: &shape_poses_b,
            touch_links: &touch_links_b,
        };

        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            contacts: true,
            max_contacts: 100,
            max_contacts_per_pair: 100,
            ..CollisionRequest::default()
        };

        let (res, _gsr) = cache
            .check_self_collision(&req, &posed, Some(&acm), &[attached_a, attached_b])
            .unwrap();
        assert!(res.collision);
        let contacts = res.contacts.expect("contacts requested");
        let pair = contacts
            .by_pair
            .get(&("gripped_a".to_string(), "gripped_b".to_string()))
            .expect(
                "two attached bodies on different in-group links, both coincident at the \
                 origin, can only be reported by get_intra_group_collisions's \
                 attached-vs-attached branch",
            );
        assert_eq!(pair[0].body_type_1, BodyType::RobotAttached);
        assert_eq!(pair[0].body_type_2, BodyType::RobotAttached);
    }

    #[test]
    fn attached_body_touch_links_disables_collision_only_with_its_own_attaching_link() {
        let (model, srdf) = two_link_model_and_srdf();
        let mut cache = chain_collision_cache(&model);
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::identity()];
        let mut touch_links = BTreeSet::new();
        touch_links.insert("tip".to_string());
        let attached = AttachedBodyGeometry {
            id: "gripped",
            link_name: "tip",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };

        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            contacts: true,
            max_contacts: 100,
            max_contacts_per_pair: 100,
            ..CollisionRequest::default()
        };

        let (res, _gsr) = cache
            .check_self_collision(&req, &posed, Some(&acm), &[attached])
            .unwrap();
        let contacts = res.contacts.expect("contacts requested");
        assert!(
            !contacts
                .by_pair
                .contains_key(&("tip".to_string(), "gripped".to_string())),
            "touch_links containing \"tip\" (gripped's own attaching link) must disable \
             intra_group_collision_enabled between them -- generate_distance_field_cache_entry's \
             `if attached.touch_links.contains(link_name)` check is only ever reached while \
             iterating the attaching link itself"
        );
        assert!(
            contacts
                .by_pair
                .contains_key(&("mid".to_string(), "gripped".to_string())),
            "touch_links only disables the pair against the attaching link itself -- \"mid\" is \
             a different link and must still collide with the coincident attached body"
        );
    }

    #[test]
    fn get_intra_group_collisions_ignores_an_attached_body_placed_away_from_its_group() {
        let (model, srdf) = two_link_model_and_srdf();
        let mut cache = chain_collision_cache(&model);
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::translation(10.0, 0.0, 0.0)];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "gripped",
            link_name: "tip",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };

        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            contacts: true,
            max_contacts: 100,
            max_contacts_per_pair: 100,
            ..CollisionRequest::default()
        };

        let (res, _gsr) = cache
            .check_self_collision(&req, &posed, Some(&acm), &[attached])
            .unwrap();
        if let Some(contacts) = &res.contacts {
            assert!(
                contacts
                    .by_pair
                    .keys()
                    .all(|(a, b)| a != "gripped" && b != "gripped"),
                "an attached body 10m from mid/tip's coincident origin must not pass the \
                 bounding-sphere pre-filter, so get_intra_group_collisions must report no pair \
                 involving it"
            );
        }
    }

    #[test]
    fn get_environment_collisions_detects_an_attached_body_at_an_environment_point() {
        let (model, srdf) = two_link_model_and_srdf();
        let mut cache = chain_collision_cache(&model);
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let env = point_environment_distance_field(Vector3::new(0.0, 0.0, 0.0));

        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::identity()];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "gripped",
            link_name: "tip",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };

        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            contacts: true,
            max_contacts: 100,
            max_contacts_per_pair: 100,
            ..CollisionRequest::default()
        };

        // check_robot_collision only ever runs get_environment_collisions
        // (no self/intra-group phase), isolating this assertion to that one
        // function even though mid/tip/gripped are all also coincident with
        // each other.
        let (res, _gsr) = cache
            .check_robot_collision(&req, &posed, Some(&acm), &[attached], &env)
            .unwrap();
        assert!(res.collision);
        let contacts = res.contacts.expect("contacts requested");
        let pair = contacts
            .by_pair
            .get(&("gripped".to_string(), "environment".to_string()))
            .expect(
                "an environment point at the coincident tip/gripped origin must register as an \
                 environment collision for the attached body, now that \
                 get_environment_collisions's loop bound covers attached_body_names too",
            );
        assert_eq!(pair[0].body_type_1, BodyType::RobotAttached);
        assert_eq!(pair[0].body_type_2, BodyType::WorldObject);
    }

    #[test]
    fn get_environment_collisions_ignores_an_attached_body_with_no_nearby_environment_point() {
        let (model, srdf) = two_link_model_and_srdf();
        let mut cache = chain_collision_cache(&model);
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let config = small_distance_field_config();
        let empty_env = PropagationDistanceField::new(
            config.geometry,
            config.max_propagation_distance,
            config.use_signed_distance_field,
        )
        .unwrap();

        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::identity()];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "gripped",
            link_name: "tip",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };

        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            ..CollisionRequest::default()
        };

        let (res, _gsr) = cache
            .check_robot_collision(&req, &posed, Some(&acm), &[attached], &empty_env)
            .unwrap();
        assert!(
            !res.collision,
            "check_robot_collision only consults env_distance_field -- the attached body is \
             coincident with tip, but with no points in the field this must report no collision"
        );
    }

    /// Round 27 item 2: measures whether this crate's own cache
    /// (`DistanceFieldCollisionCache::cache_entry`) can serve a stale
    /// environment-collision answer after the *same* `env_distance_field`
    /// value is mutated in place between two calls on the *same* cache --
    /// the Rust analog of upstream's `World`-observer concern (a
    /// `CollisionEnvDistanceField` whose `distance_field_cache_entry_world_`
    /// must not survive a `World` mutation stale). Unlike upstream, this
    /// port never stores `env_distance_field` as a struct field anywhere
    /// (see [`DistanceFieldCollisionCache::check_collision`]'s own doc: it
    /// is threaded through as an explicit `&dyn DistanceField` parameter on
    /// every call, "the same way [`crate::PropagationDistanceField`] is
    /// threaded through this crate wherever upstream reads it off a `World`
    /// this port has no type for"), so [`DistanceFieldCollisionCache`]'s own
    /// cache-key comparison (`get_distance_field_cache_entry`) never
    /// includes it and cannot serve stale environment data -- confirmed
    /// here empirically, not just by re-reading that doc comment.
    #[test]
    fn check_robot_collision_reflects_a_field_mutated_in_place_between_calls() {
        let (model, srdf) = two_link_model_and_srdf();
        let mut cache = chain_collision_cache(&model);
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let config = small_distance_field_config();
        let mut env = PropagationDistanceField::new(
            config.geometry,
            config.max_propagation_distance,
            config.use_signed_distance_field,
        )
        .unwrap();

        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::identity()];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "gripped",
            link_name: "tip",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };

        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            ..CollisionRequest::default()
        };

        // World state 1: an object occupies the point coincident with
        // `gripped`/`tip`'s origin.
        let point = Vector3::new(0.0, 0.0, 0.0);
        env.add_points_to_field(&[point]);
        let (res, _gsr) = cache
            .check_robot_collision(&req, &posed, Some(&acm), &[attached], &env)
            .unwrap();
        assert!(
            res.collision,
            "first query: an environment point at the coincident tip/gripped origin must \
             register a collision"
        );

        // World state 2: the object is removed -- same `cache`, same `env`
        // value, mutated in place, no field rebuilt from scratch and no
        // explicit invalidation call made to `cache`.
        env.remove_points_from_field(&[point]);
        let (res, _gsr) = cache
            .check_robot_collision(&req, &posed, Some(&acm), &[attached], &env)
            .unwrap();
        assert!(
            !res.collision,
            "second query, after removing the point from the same env field instance: must \
             reflect the mutation, not a stale collision cached from the first query -- \
             DistanceFieldCollisionCache does not store env_distance_field, so there is no \
             cache to go stale here"
        );
    }

    #[test]
    fn get_intra_group_proximity_gradients_updates_an_attached_bodys_gradient_slot() {
        let (model, srdf) = two_link_model_and_srdf();
        let mut cache = chain_collision_cache(&model);
        let acm = AllowedCollisionMatrix::from_srdf(&srdf);
        let mut state = moveit_state::RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let env = point_environment_distance_field(Vector3::new(0.0, 0.0, 0.0));

        let shapes = vec![sample_shape()];
        let shape_poses = vec![Isometry3::identity()];
        let touch_links = BTreeSet::new();
        let attached = AttachedBodyGeometry {
            id: "gripped",
            link_name: "tip",
            shapes: &shapes,
            shape_poses: &shape_poses,
            touch_links: &touch_links,
        };

        let req = CollisionRequest {
            group_name: Some("chain".to_string()),
            ..CollisionRequest::default()
        };

        let gsr = cache
            .get_collision_gradients(&req, &posed, Some(&acm), &[attached], &env)
            .unwrap();
        let attached_index = gsr.dfce.link_names.len()
            + gsr
                .dfce
                .attached_body_names
                .iter()
                .position(|n| n == "gripped")
                .expect("\"gripped\" is on \"tip\", an in-group link, with acm: Some");
        assert!(
            gsr.gradients[attached_index]
                .types
                .contains(&CollisionType::Intra),
            "get_intra_group_proximity_gradients must fold a finite mid-vs-gripped distance \
             into the attached body's own gradient slot, marked CollisionType::Intra -- \
             get_self_proximity_gradients/get_environment_proximity_gradients are the faithful \
             no-gap siblings and never touch an attached-body index, so this slot can only be \
             written by the intra-group phase"
        );
    }
}
