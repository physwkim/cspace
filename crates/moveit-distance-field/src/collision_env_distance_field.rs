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
//! [`crate::DistanceFieldCacheEntry`] for one group; and
//! [`DistanceFieldCollisionCache`], the persistent cache-owner
//! [`DistanceFieldCollisionCache::generate_collision_checking_structures`]
//! (upstream `generateCollisionCheckingStructures`) needs -- see this
//! module's doc comment for its design.
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
//! `PlanningScene`. Both new functions' own doc comments cover the resulting
//! deviation: upstream's attached-body comparison is vacuously true here,
//! since a bare `RobotState` genuinely has none to compare, matching
//! [`crate::DistanceFieldCacheEntry`]'s own "always empty" attached-body
//! fields.
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
//! - **`AttachedBody`-dependent methods** (`getAttachedBodySphereDecomposition`/
//!   `getAttachedBodyPointDecomposition`, in `collision_common_distance_field.cpp`).
//!   `AttachedBody` now exists (`moveit-scene`), but is unreachable from a
//!   bare `RobotState` -- see the paragraph above -- and these two take a
//!   `moveit::core::AttachedBody*` directly, which this workspace has no
//!   equivalent standalone type for outside `moveit-scene`'s ownership of it.
//!
//! The rest of `CollisionEnvDistanceField` -- `checkSelfCollision`,
//! `checkCollision`, `checkRobotCollision`, the `distance*` methods,
//! `createCollisionModelMarker` (draws `visualization_msgs::msg::MarkerArray`,
//! out of scope under PORTING-PLAN.md D1 same as
//! `getBodySphereVisualizationMarkers`), `getEnvironmentCollisions`/
//! `getEnvironmentProximityGradients`, `updateDistanceObject`,
//! `generateDistanceFieldCacheEntryWorld`, `notifyObjectChange`, and the
//! `CollisionEnvDistanceField` type itself (a `CollisionEnv` implementor
//! wrapping a `World` observer and a `planning_scene::PlanningScene` this
//! workspace does not have yet either) -- remains out of scope regardless
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
    AllowedCollisionMatrix, AllowedCollisionType, AttachedBodyGeometry, LinkPaddingScale,
};
use moveit_error::Result;
use moveit_geometry::Isometry3;
use moveit_model::RobotModel;
use moveit_state::Posed;
use nalgebra::Vector3;

use crate::collision_common_distance_field::{
    AttachedBodySnapshot, DistanceFieldCacheEntry, GroupStateRepresentation,
};
use crate::collision_distance_field_types::{
    BodyDecomposition, CollisionSphere, CollisionType, GradientInfo, PosedBodyPointDecomposition,
    PosedBodySphereDecomposition, PosedDistanceField,
};
use crate::{DistanceField, GridGeometry, PropagationDistanceField};

/// Upstream's `link_body_decomposition_vector_` paired with
/// `link_body_decomposition_index_map_`: every link's [`BodyDecomposition`],
/// in `RobotModel::link_models()` order, and a name-to-index lookup into it.
type LinkBodyDecompositions = (Vec<Arc<BodyDecomposition>>, HashMap<String, usize>);

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
    let total = link_names.len(); // + attached_body_names.len(), always 0 here.

    let mut link_has_geometry = Vec::with_capacity(link_names.len());
    let mut link_body_indices = Vec::with_capacity(link_names.len());
    let mut link_state_indices = Vec::with_capacity(link_names.len());
    let mut self_collision_enabled = vec![true; total];
    let mut intra_group_collision_enabled = vec![Vec::new(); total];

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
                // No attached bodies exist in this workspace (see
                // `DistanceFieldCacheEntry`'s "Deviations from upstream"),
                // so upstream's per-attached-body loop here is a no-op.
            }
            intra_group_collision_enabled[i] = row;
        } else {
            link_has_geometry.push(false);
            link_body_indices.push(0);
            self_collision_enabled[i] = false;
            intra_group_collision_enabled[i] = vec![false; total];
        }
    }
    // Upstream's second loop, over `attached_body_names_`, is a no-op here
    // for the same reason: this port never populates that vector.

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
        attached_body_names: Vec::new(),
        attached_body_link_state_indices: Vec::new(),
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
/// fresh-build branch only (`!dfce->pregenerated_group_state_representation_`
/// -- see this function's "Deviations from upstream" for why the other
/// branch cannot be reached here). Builds one
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
/// - **The "pregenerated" branch is not ported, because it is provably
///   unreachable here.** Upstream populates
///   `dfce->pregenerated_group_state_representation_` in exactly one place,
///   `CollisionEnvDistanceField::initialize` (its constructor path,
///   `collision_env_distance_field.cpp:126-156`): for every joint group, it
///   builds a `DistanceFieldCacheEntry` and immediately calls this same
///   function to populate `pregenerated_group_state_representation_map_`,
///   which a *later* `generateDistanceFieldCacheEntry` call then copies onto
///   the field this branch checks. `DistanceFieldCacheEntry` in this port
///   (see its own doc comment) has no such field at all -- there is no
///   `initialize`-equivalent constructor in this crate's scope to populate
///   it from, so every `DistanceFieldCacheEntry` this port can build takes
///   the fresh-build branch, unconditionally.
/// - **Attached bodies are always skipped**, same reasoning as this module's
///   own `build_non_group_distance_field`: this workspace has no `AttachedBody`
///   reachable from a bare `RobotState`. Upstream's own trailing
///   attached-body loop (`collision_env_distance_field.cpp:1229-` onward) is
///   accordingly not ported; `dfce.attached_body_names` is always empty (see
///   [`DistanceFieldCacheEntry`]'s "Deviations from upstream"), so that loop
///   would iterate zero times regardless.
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
/// [`Posed::global_link_transform`]). See [`PosedDistanceField::new`] for
/// errors building a link's own distance field.
pub fn group_state_representation<'a, 'm>(
    dfce: &'a DistanceFieldCacheEntry<'m>,
    state: &Posed<'_, 'm>,
    link_body_decomposition_vector: &[Arc<BodyDecomposition>],
    resolution: f64,
    max_propagation_distance: f64,
    use_signed_distance_field: bool,
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
            sphere_radii: link_bd.sphere_radii().to_vec(),
            joint_name: state.model().joint_model_at(joint_index).name().to_string(),
            ..GradientInfo::default()
        });
        link_body_decompositions.push(Some(link_bd));
        link_distance_fields.push(Some(field));
    }

    Ok(GroupStateRepresentation {
        dfce,
        link_body_decompositions,
        attached_body_decompositions: Vec::new(),
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
/// Upstream's trailing attached-body loop is not ported: `gsr.dfce.attached_body_names`
/// is always empty (see [`DistanceFieldCacheEntry`]'s "Deviations from
/// upstream"), so that loop would iterate zero times regardless -- same
/// reasoning as [`group_state_representation`]'s own attached-body
/// deviation.
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
/// [`Posed::global_link_transform`]).
pub fn update_group_state_representation_state(
    state: &Posed<'_, '_>,
    gsr: &mut GroupStateRepresentation<'_, '_>,
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
    /// `distance_field_cache_entry_`. The only field here with no
    /// already-ported free-function equivalent -- see this module's doc
    /// comment.
    cache_entry: Option<DistanceFieldCacheEntry<'m>>,
}

impl<'m> DistanceFieldCollisionCache<'m> {
    /// Upstream `CollisionEnvDistanceField::initialize`'s config-storage
    /// half only. Upstream's other half -- a loop over every joint group
    /// building a `DistanceFieldCacheEntry` and pregenerating a
    /// `GroupStateRepresentation` for it -- exists solely to populate
    /// `pregenerated_group_state_representation_map_`, a map
    /// [`group_state_representation`] proves no call reachable from this
    /// port can ever read (see that function's own "Deviations from
    /// upstream"); there is nothing here for that loop to do.
    pub fn new(
        link_body_decompositions: LinkBodyDecompositions,
        distance_field_config: DistanceFieldConfig,
    ) -> Self {
        Self {
            link_body_decompositions,
            distance_field_config,
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
        )
    }
}

/// The `generate_distance_field: true` branch of upstream
/// `generateDistanceFieldCacheEntry`: a fresh [`PropagationDistanceField`]
/// seeded with every collision-bearing link's points *outside* the group
/// (`in_group_names`), posed at `state`'s current global link transforms.
/// Attached-body points are always skipped -- this workspace has no
/// `AttachedBody` to enumerate (see [`DistanceFieldCacheEntry`]'s
/// "Deviations from upstream").
fn build_non_group_distance_field<'a>(
    robot_model: &RobotModel,
    state: &Posed<'_, '_>,
    in_group_names: impl Iterator<Item = &'a str>,
    link_body_decomposition_vector: &[Arc<BodyDecomposition>],
    link_body_decomposition_index_map: &HashMap<String, usize>,
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
        // Attached bodies on this link: always none, see this function's
        // doc comment.
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
        )
        .unwrap();

        update_group_state_representation_state(&posed, &mut gsr).unwrap();

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
        update_group_state_representation_state(&moved_posed, &mut gsr).unwrap();
        update_group_state_representation_state(&default_posed, &mut gsr).unwrap();

        let rebuilt = group_state_representation(
            &dfce,
            &default_posed,
            &link_body_decomposition_vector,
            0.05,
            0.25,
            false,
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
        DistanceFieldCollisionCache::new(link_body_decompositions, small_distance_field_config())
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

        let mut cache = DistanceFieldCollisionCache::new(link_body_decompositions.clone(), config);
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
}
