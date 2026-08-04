// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_distance_field/include/moveit/collision_distance_field/collision_common_distance_field.hpp
//   moveit_core/collision_distance_field/src/collision_common_distance_field.cpp

//! The `RobotState`/`RobotModel`-dependent slice of `collision_common_distance_field`:
//! turning a [`Shape`] or a [`moveit_collision::Object`] into the posed
//! sphere/point decompositions [`crate::collision_distance_field_types`]
//! defines, with the same identity-keyed shape cache upstream uses.
//!
//! # Scope
//!
//! Ported: [`get_body_decomposition_cache_entry`] (upstream
//! `getBodyDecompositionCacheEntry`), [`collision_object_point_decomposition`]
//! (upstream `getCollisionObjectPointDecomposition`),
//! [`attached_body_sphere_decomposition`]/[`attached_body_point_decomposition`]
//! (upstream `getAttachedBodySphereDecomposition`/
//! `getAttachedBodyPointDecomposition` -- round 22, see below), and
//! [`DistanceFieldCacheEntry`] itself, now that `moveit-model`'s
//! `JointModelGroup::updated_link_names`/`updated_link_with_geometry_names`
//! close the dependency gap this file's previous doc comment recorded here
//! (upstream `getUpdatedLinkModelNames`/`getUpdatedLinkModelsWithGeometryNames`).
//! [`crate::generate_distance_field_cache_entry`], the function that
//! populates it, lives in `collision_env_distance_field.rs` — see its own
//! doc comment for the construction logic and for
//! `compareCacheEntryToState`'s cache-key semantics (read but not ported;
//! see below).
//!
//! Round 6 additionally ports [`GroupStateRepresentation`] itself (upstream
//! `collision_common_distance_field.hpp:56-103`) -- re-reading its
//! definition (per this round's own instruction to re-test rather than
//! inherit a blocker list) found it owns only already-ported types
//! ([`DistanceFieldCacheEntry`] plus [`PosedBodySphereDecomposition`]/
//! [`PosedDistanceField`]/[`GradientInfo`]), so the previous round's
//! "unported cache type" framing was itself another over-grouping, the same
//! failure mode round 5 found for `compareCacheEntryToState`/
//! `compareCacheEntryToAllowedCollisionMatrix`. The three functions that
//! build/refresh it (`getGroupStateRepresentation`/
//! `updateGroupStateRepresentationState`/`getDistanceFieldCacheEntry`) live
//! in `collision_env_distance_field.rs`, matching upstream's own file
//! split; see that module's doc for what unblocked them and for
//! `generateCollisionCheckingStructures`'s remaining, more precisely
//! restated blocker.
//!
//! This round also closes a real correctness gap
//! [`crate::compare_cache_entry_to_state`] had left open: see
//! [`AttachedBodySnapshot`]'s doc comment.
//!
//! # Round 22: `getAttachedBodySphereDecomposition`/`getAttachedBodyPointDecomposition`
//!
//! Ported as [`attached_body_sphere_decomposition`]/
//! [`attached_body_point_decomposition`]. Previously deferred (see prior
//! revisions of this doc comment) as blocked on "no `AttachedBody` reachable
//! from a bare `RobotState`" -- the same premise [`AttachedBodySnapshot`]
//! already disproves for cache-key comparison: [`AttachedBodyGeometry`] is
//! an explicit, caller-supplied parameter (not read off `RobotState`), and
//! every one of this crate's collision-checking entry points already
//! threads a `&[AttachedBodyGeometry<'_>]` slice through for exactly that
//! reason. The blocker was stale, not structural -- these two functions
//! only needed the same parameter their sibling already had.
//!
//! Real, in-scope caller confirmed by `rg` before porting (not a
//! caller-less symbol; see this crate's completion rollup in `lib.rs`):
//! upstream `getAttachedBodySphereDecomposition`'s only caller is
//! `CollisionEnvDistanceField::getGroupStateRepresentation`
//! (`collision_env_distance_field.cpp:1239`), ported as
//! [`crate::group_state_representation`], which now calls
//! [`attached_body_sphere_decomposition`] for real (see that function's own
//! doc comment). `getAttachedBodyPointDecomposition`'s only caller is
//! `generateDistanceFieldCacheEntry`'s non-group-link loop
//! (`collision_env_distance_field.cpp:928`), ported as
//! `build_non_group_distance_field` in `collision_env_distance_field.rs`,
//! which now calls [`attached_body_point_decomposition`] the same way.
//!
//! Deferred, and why:
//!
//! - **`getBodySphereVisualizationMarkers`.** Builds a
//!   `visualization_msgs::msg::MarkerArray` for RViz. PORTING-PLAN.md D1
//!   keeps ROS message types out of every crate but the optional
//!   `moveit-ros`.
//!
//! # `compareCacheEntryToState`'s cache-key semantics, read but not ported
//!
//! `generateDistanceFieldCacheEntry` builds `state_values_` from
//! `state.getVariableNames()`, i.e. every non-fixed joint's variables in
//! joint order -- mimic joints included, despite `DistanceFieldCacheEntry::state_values_`'s
//! own header comment claiming they are excluded; the actual loop
//! (`collision_env_distance_field.cpp:886-896`) iterates `getVariableNames()`
//! with no mimic filter, so the header comment is stale relative to the
//! code it documents. This port follows the code. `state_check_indices_`
//! holds indices into `state_values_` for variables *not* in the group's
//! active joints -- i.e. the joints a cached entry promises did not move.
//! `compareCacheEntryToState` then re-reads the current `RobotState`'s
//! values at exactly those indices and invalidates the cache entry if any
//! differs from the stored value by more than `EPSILON = 0.001`. That is
//! the actual "cache key": a `DistanceFieldCacheEntry` is valid for a new
//! `RobotState` iff every joint variable *outside* the group is unchanged
//! to within `EPSILON`. Note this ties the key's shape to a specific group
//! -- a general, group-independent cache would need a different design.
//! Not ported this round -- see the deferred list above.
//!
//! # Known upstream defect, ported byte-for-byte: the shape cache is resolution-blind
//!
//! [`get_body_decomposition_cache_entry`] preserves a genuine, upstream-
//! acknowledged defect: the cache is keyed *solely* on shape identity, but
//! the cached [`BodyDecomposition`] value also depends on `resolution` (it
//! is threaded into [`crate::find_internal_points::find_internal_points_convex`] via
//! [`BodyDecomposition::from_shapes`], which changes the interior point
//! sampling). Upstream's own comment in `getBodyDecompositionCacheEntry`
//! is literally `// TODO - deal with changing resolution?` -- the gap is
//! not inferred here, it is upstream's own admission. Calling this function
//! twice for the same shape with two different resolutions returns the
//! *first* call's decomposition both times; nothing about the mismatch is
//! observable from a single query in isolation, only from calling it twice
//! with different `resolution` arguments for the same shape and comparing
//! against a fresh, uncached rebuild -- see this module's tests.
//!
//! This is exactly the "a cache whose key omits something the value depends
//! on" defect class this file's task description flagged. It is preserved
//! rather than fixed for the same reason
//! [`crate::do_bounding_spheres_intersect`]'s squared-vs-unsquared defect
//! is preserved: this task's mandate is matching upstream's actual
//! behaviour, not upstream's intent, and "fixing" a cache key is a
//! behaviour change that needs its own sign-off, not a silent parity
//! deviation.
//!
//! In current upstream usage (per `collision_env_distance_field.cpp`, read
//! but not ported) this cache is reached only for world objects and
//! attached bodies -- single, reused shapes queried repeatedly at a fixed
//! resolution in practice. Robot links go through `addLinkBodyDecompositions`
//! instead, which builds each link's `BodyDecomposition` directly, once,
//! from *all* of that link's shapes together, and is itself in the
//! out-of-scope file.
//!
//! # Cache key: `Arc` identity via raw address, pinned by a stored `Weak`
//!
//! Upstream keys `BodyDecompositionCache::map_` on `shapes::ShapeConstWeakPtr`
//! compared with `std::owner_less` -- ownership-based identity, not shape
//! *value* equality. A `HashMap` needs `Hash`/`Eq`, which `std::sync::Weak`
//! does not implement in Rust, so this port keys on `Arc::as_ptr(shape) as
//! usize` instead: the pointer's numeric address. Two live `Arc`s alias the
//! same allocation iff `Arc::as_ptr` agrees, exactly matching `owner_less`.
//!
//! A bare address is not enough, though: once every `Arc<Shape>` pointing at
//! an allocation is dropped, Rust's allocator is free to hand that same
//! address to a later, *unrelated* `Arc<Shape>` allocation. A first version
//! of this cache stored only `Arc<BodyDecomposition>` values against that
//! bare address and hit exactly this -- confirmed concretely by
//! `collision_object_point_decomposition_matches_the_oracle` in
//! `collision_common_distance_field_parity.rs`, where a sphere's `Arc<Shape>`
//! was built, cached, and dropped, and a box's `Arc<Shape>` built immediately
//! after landed at the same freed address, so the box's lookup silently
//! returned the sphere's stale `BodyDecomposition` (a wildly different
//! interior point count). Upstream does not have this hazard: storing a
//! `std::weak_ptr` as a `std::map` key keeps that pointer's *control block*
//! allocated for as long as the map entry exists, even after the pointee
//! itself is destroyed, so a later, unrelated `shared_ptr`'s control block
//! can never land at that address while the map key survives.
//!
//! This port closes the gap the same way: each entry stores a
//! `Weak<Shape>` alongside the cached value. Holding any `Weak<T>` keeps
//! `T`'s `ArcInner` allocation (and therefore its address) alive even after
//! `T` itself is dropped in place -- Rust deallocates only once *both* the
//! strong and weak counts reach zero. Since this cache never evicts
//! (matching upstream's own unimplemented `// TODO - clean cache`), every
//! address it has ever cached stays pinned for the rest of the process, so
//! a later shape can never reuse it. Only the numeric address is ever
//! dereferenced-free-compared as the `HashMap` key; the `Weak` exists
//! purely to pin the allocation, not to be upgraded on lookup.
//!
//! # Locking pattern matches upstream, redundant-computation race included
//!
//! Upstream's `getBodyDecompositionCacheEntry` takes the lock, checks for
//! an existing entry, and *releases the lock* before constructing a fresh
//! `BodyDecomposition` (an allocation-heavy, unbounded-cost operation upstream
//! evidently did not want to hold a mutex through), then re-locks only to
//! insert. Two threads racing on the same not-yet-cached shape can both
//! miss the first check, both build a fresh decomposition, and both insert
//! -- upstream's `cache.map_[wptr] = bdcp;` on the second lock
//! unconditionally overwrites, so the second inserter's result simply wins,
//! with the first thread's work discarded and returned to its own caller
//! (both callers still get a valid, self-consistent -- if not
//! object-identical -- `BodyDecomposition`). This port reproduces that
//! exact lock/unlock/build/lock/insert shape (including the unconditional
//! overwrite) rather than closing the race with a single held lock, since
//! closing it would be a straightforward correctness improvement upstream
//! itself does not have and this task's mandate is matching upstream
//! behaviour.

use std::collections::{BTreeSet, HashMap};
use std::sync::{Arc, Mutex, OnceLock, Weak};

use moveit_collision::{AllowedCollisionMatrix, AttachedBodyGeometry, Object};
use moveit_error::Result;
use moveit_geometry::{Isometry3, Shape};
use moveit_state::RobotState;

use crate::PropagationDistanceField;
use crate::collision_distance_field_types::{
    BodyDecomposition, GradientInfo, PosedBodyPointDecomposition,
    PosedBodyPointDecompositionVector, PosedBodySphereDecomposition,
    PosedBodySphereDecompositionVector, PosedDistanceField,
};

/// An owned snapshot of one [`moveit_collision::AttachedBodyGeometry`],
/// captured by [`crate::generate_distance_field_cache_entry`] at generation
/// time and stored on [`DistanceFieldCacheEntry::attached_bodies`].
///
/// # Closes a real gap left open through round 5
///
/// Upstream's `compareCacheEntryToState` keeps `dfce->state_` (a full
/// `RobotStatePtr` snapshot) and re-derives its attached bodies on demand
/// via `dfce->state_->getAttachedBodies()` at comparison time --
/// `moveit::core::RobotState` owns that concept upstream, so the snapshot
/// alone is enough. `moveit_state::RobotState` does not carry attached
/// bodies at all in this port (see `moveit_scene::AttachedBody`'s own
/// module doc for why that concept lives on `PlanningScene` here instead),
/// so [`DistanceFieldCacheEntry::state`] structurally *cannot* answer the
/// same query -- there is nothing to re-derive from. Round 5 (see this
/// module's earlier doc revisions in `PORTING-PLAN.md` §27.2) reported this
/// as a permanent, vacuously-true deviation in both
/// [`crate::compare_cache_entry_to_state`] and
/// [`crate::compare_cache_entry_to_allowed_collision_matrix`] and moved on.
///
/// Re-reading upstream's actual comparison this round (see
/// `collision_env_distance_field.cpp:1280-1310`) found only
/// `compareCacheEntryToState` genuinely needs it --
/// `compareCacheEntryToAllowedCollisionMatrix`'s own `attached_bodies` fetch
/// is dead code upstream, assigned and never read (still correctly
/// documented as omitted on that function, not fixed here). And the fix for
/// `compareCacheEntryToState` does not need `moveit_state::RobotState` to
/// grow attached-body support at all: [`moveit_collision::AttachedBodyGeometry`]
/// is a *borrowed* view already defined in `moveit-collision` (a crate this
/// one already depends on) specifically so a lower crate can consume
/// attached-body data without depending back on `moveit-scene` --
/// `moveit_scene::AttachedBody::as_geometry` already builds one this exact
/// way for `moveit_collision::CollisionEnv` callers. This snapshot type
/// applies the same pattern one layer further down: an *owned* copy of
/// exactly the fields upstream's comparison reads (`getName`/
/// `getTouchLinks`/`getShapes`), since a [`DistanceFieldCacheEntry`] outlives
/// the single call that captures it, unlike a borrowed
/// [`moveit_collision::AttachedBodyGeometry`].
///
/// `shapes` clones the `Arc<Shape>` pointers, not the shape values: upstream
/// compares `getShapes()[j] != getShapes()[j]`, a `shared_ptr` `operator!=`
/// -- pointer identity, not shape equality -- so
/// [`crate::compare_cache_entry_to_state`] compares via `Arc::ptr_eq` too,
/// matching [`get_body_decomposition_cache_entry`]'s own identity-not-value
/// cache-key rationale.
#[derive(Debug, Clone)]
pub struct AttachedBodySnapshot {
    id: String,
    touch_links: BTreeSet<String>,
    shapes: Vec<Arc<Shape>>,
}

impl AttachedBodySnapshot {
    pub(crate) fn from_geometry(geometry: &AttachedBodyGeometry<'_>) -> Self {
        Self {
            id: geometry.id.to_string(),
            touch_links: geometry.touch_links.clone(),
            shapes: geometry.shapes.to_vec(),
        }
    }

    pub(crate) fn matches(&self, other: &AttachedBodyGeometry<'_>) -> bool {
        self.id == other.id
            && &self.touch_links == other.touch_links
            && self.shapes.len() == other.shapes.len()
            && self
                .shapes
                .iter()
                .zip(other.shapes)
                .all(|(a, b)| Arc::ptr_eq(a, b))
    }
}

/// Upstream `DistanceFieldCacheEntry`: a group-, ACM-, and robot-state-
/// specific cache entry pairing a static distance field of "the rest of the
/// robot" with the bookkeeping needed to check the group's own links against
/// it. Populated by [`crate::generate_distance_field_cache_entry`] (upstream
/// `CollisionEnvDistanceField::generateDistanceFieldCacheEntry`); see this
/// module's doc comment for what stays deferred.
///
/// # Deviations from upstream
///
/// - `pregenerated_group_state_representation_` does not exist here: no
///   [`crate::group_state_representation`] call this port can reach ever
///   populates it (see that function's own "Deviations from upstream" for
///   why upstream's pregenerated-reuse branch is provably unreachable), so
///   there is nothing for this field to ever hold.
/// - `attached_body_names_`/`attached_body_link_state_indices_` are plain
///   `Vec`s, not an `Option`/sentinel encoding "no attached bodies in the
///   group": empty [`Vec`]s already represent that case exactly. As of round
///   22, [`crate::generate_distance_field_cache_entry`] populates both
///   fields for real, one entry per attached body found on a geometry-
///   bearing, ACM-tracked link (see this function's own doc comment) --
///   they are no longer permanently empty. Distinct from
///   [`DistanceFieldCacheEntry::attached_bodies`] below, which tracks a
///   different set: these two track only the bodies attached within the
///   cached group, `attached_bodies` tracks the entire robot state's
///   attached bodies (what upstream's cache-invalidation check actually
///   needs, regardless of group membership).
/// - `acm_` is a plain [`AllowedCollisionMatrix`], not the
///   default-constructed (i.e. empty, permit-nothing-restricted-yet) one
///   upstream's `acm_` member is left at when `generateDistanceFieldCacheEntry`'s
///   `acm` parameter is null. [`AllowedCollisionMatrix::default`] gives the
///   exact same "no entries, no defaults" value, so [`generate_distance_field_cache_entry`](crate::generate_distance_field_cache_entry)
///   uses it directly rather than wrapping this field in an `Option` that
///   would only ever disagree with upstream's actual (always-present, just
///   possibly-empty) member.
///
/// [`generate_distance_field_cache_entry`]: crate::generate_distance_field_cache_entry
pub struct DistanceFieldCacheEntry<'m> {
    /// `group_name_`.
    pub group_name: String,
    /// `state_`. Upstream stores a `RobotStatePtr` (a copy captured at
    /// generation time); this port stores the owned [`RobotState`] value
    /// directly rather than wrapping it in an `Arc`, since nothing in this
    /// crate's scope needs to share ownership of it.
    pub state: RobotState<'m>,
    /// `state_check_indices_`. See this module's "`compareCacheEntryToState`'s
    /// cache-key semantics" doc section.
    pub state_check_indices: Vec<usize>,
    /// `state_values_`. See this module's "`compareCacheEntryToState`'s
    /// cache-key semantics" doc section for why mimic joints are included
    /// despite upstream's own header comment claiming otherwise.
    pub state_values: Vec<f64>,
    /// `acm_`. See this type's "Deviations from upstream" for why this is a
    /// plain value rather than an `Option`.
    pub acm: AllowedCollisionMatrix,
    /// `distance_field_`: the distance field of every link *not* in the
    /// group, including (as of round 22) the points of any attached body on
    /// such a link -- see `build_non_group_distance_field`'s own (module-
    /// private, `collision_env_distance_field.rs`) "Deviation from upstream"
    /// for the geometry-bearing-link precondition this inherits. `None` when
    /// [`generate_distance_field_cache_entry`](crate::generate_distance_field_cache_entry)
    /// was called with `generate_distance_field: None`, matching upstream's
    /// null `distance_field_` in that case.
    pub distance_field: Option<PropagationDistanceField>,
    /// `link_names_`: every link that moves if any joint in the group
    /// moves, index-ordered (upstream `getUpdatedLinkModelNames`, now
    /// [`moveit_model::JointModelGroup::updated_link_names`]).
    pub link_names: Vec<String>,
    /// `link_has_geometry_`, one entry per [`DistanceFieldCacheEntry::link_names`].
    pub link_has_geometry: Vec<bool>,
    /// `link_body_indices_`, one entry per [`DistanceFieldCacheEntry::link_names`]
    /// -- an index into the caller's link-body-decomposition table (see
    /// [`crate::add_link_body_decompositions`]) for links with geometry, `0`
    /// (matching upstream's own placeholder) for links without.
    pub link_body_indices: Vec<usize>,
    /// `link_state_indices_`, one entry per [`DistanceFieldCacheEntry::link_names`].
    /// Upstream searches `state.getJointModelGroup(group_name)->getUpdatedLinkModels()`
    /// for the matching link and stores its position there; see
    /// [`crate::generate_distance_field_cache_entry`]'s doc comment for why
    /// that search always lands on `link_state_indices[i] == i` and this
    /// port computes it directly instead of re-deriving it with a search
    /// that can never disagree.
    pub link_state_indices: Vec<usize>,
    /// `attached_body_names_`. See this type's "Deviations from upstream"
    /// for what populates this and when it is empty.
    pub attached_body_names: Vec<String>,
    /// `attached_body_link_state_indices_`. See this type's "Deviations from
    /// upstream" for what populates this and when it is empty.
    pub attached_body_link_state_indices: Vec<usize>,
    /// Not an upstream field by this name -- see [`AttachedBodySnapshot`]'s
    /// doc comment for what this closes and why it exists as a field here
    /// rather than being re-derived from [`DistanceFieldCacheEntry::state`]
    /// the way upstream re-derives it from `state_`. Every attached body on
    /// the `RobotState` [`crate::generate_distance_field_cache_entry`] was
    /// called with, regardless of group membership -- see this type's
    /// "Deviations from upstream" for how this differs from
    /// [`DistanceFieldCacheEntry::attached_body_names`].
    pub attached_bodies: Vec<AttachedBodySnapshot>,
    /// `self_collision_enabled_`, one entry per
    /// [`DistanceFieldCacheEntry::link_names`] followed by one per
    /// [`DistanceFieldCacheEntry::attached_body_names`] (populated for real
    /// as of round 22, per-ACM-entry, when `attached_body_names` is
    /// non-empty). Note: this data is computed correctly, but as of round 22
    /// the attached-body slots it describes are not yet read by
    /// `get_self_collisions`/`get_self_proximity_gradients` in
    /// `collision_env_distance_field.rs` -- see that module's doc comment
    /// for the known gap.
    pub self_collision_enabled: Vec<bool>,
    /// `intra_group_collision_enabled_`: a square matrix, same indexing as
    /// [`DistanceFieldCacheEntry::self_collision_enabled`] on both axes.
    pub intra_group_collision_enabled: Vec<Vec<bool>>,
}

/// Upstream `CollisionEnvDistanceField::GroupStateRepresentation`
/// (`collision_common_distance_field.hpp:56-103`): a [`DistanceFieldCacheEntry`]
/// paired with one posed sphere decomposition and one small per-link
/// [`PosedDistanceField`] per group link, plus a [`GradientInfo`] slot per
/// link and attached body. Built by [`crate::group_state_representation`]
/// (upstream `getGroupStateRepresentation`), refreshed in place for a new
/// pose by [`crate::update_group_state_representation_state`] (upstream
/// `updateGroupStateRepresentationState`).
///
/// # Deviations from upstream
///
/// - `dfce_` is a `DistanceFieldCacheEntryConstPtr` (shared, reference-
///   counted) upstream, so the same cache entry can back more than one
///   `GroupStateRepresentation` at once (e.g. across repeated
///   `checkCollision` calls that reuse `CollisionEnvDistanceField`'s own
///   `distance_field_cache_entry_` member). That member is out of this
///   crate's scope this round (see `collision_env_distance_field.rs`'s
///   module doc), and nothing else in this crate's scope needs to share a
///   [`DistanceFieldCacheEntry`] across multiple `GroupStateRepresentation`s
///   -- so this port borrows it instead (`&'a DistanceFieldCacheEntry<'m>`),
///   matching the same "no in-scope aliasing need" reasoning
///   [`crate::PosedBodySphereDecompositionVector`]'s own doc comment already
///   applies to its elements.
/// - `attached_body_decompositions_`: as of round 22, populated by
///   [`crate::group_state_representation`] via
///   [`crate::attached_body_sphere_decomposition`] (ported
///   `getAttachedBodySphereDecomposition`) -- one entry per
///   `dfce.attached_body_names`, distinct from [`AttachedBodySnapshot`],
///   which only identifies an attached body for cache invalidation, not its
///   geometry.
/// - Upstream's custom copy constructor (deep-cloning
///   `link_body_decompositions_`/`attached_body_decompositions_`, shallow-
///   copying `link_distance_fields_` via `assign`) exists only to support the
///   "pregenerated" reuse path in `getGroupStateRepresentation`
///   (`dfce->pregenerated_group_state_representation_`), which this port
///   proves unreachable -- see [`crate::group_state_representation`]'s own
///   doc comment -- so there is nothing here that needs that copy behaviour.
pub struct GroupStateRepresentation<'a, 'm> {
    /// `dfce_`. See this type's "Deviations from upstream" for why this is
    /// borrowed rather than a shared pointer.
    pub dfce: &'a DistanceFieldCacheEntry<'m>,
    /// `link_body_decompositions_`, one entry per `dfce.link_names`. `None`
    /// for a link without geometry, matching upstream's null
    /// `PosedBodySphereDecompositionPtr` for the same case.
    pub link_body_decompositions: Vec<Option<PosedBodySphereDecomposition>>,
    /// `attached_body_decompositions_`, one entry per `dfce.attached_body_names`.
    /// See this type's "Deviations from upstream".
    pub attached_body_decompositions: Vec<PosedBodySphereDecompositionVector>,
    /// `link_distance_fields_`, one entry per `dfce.link_names`. `None` for
    /// a link without geometry, matching
    /// [`GroupStateRepresentation::link_body_decompositions`].
    pub link_distance_fields: Vec<Option<PosedDistanceField>>,
    /// `gradients_`, one entry per `dfce.link_names` followed by one per
    /// `dfce.attached_body_names`. Note: the attached-body slots are
    /// allocated and kept in sync by [`crate::group_state_representation`]/
    /// [`crate::update_group_state_representation_state`], but as of round
    /// 22 are not yet written to by the collision-checking functions in
    /// `collision_env_distance_field.rs` -- see that module's doc comment
    /// for the known gap.
    pub gradients: Vec<GradientInfo>,
}

/// The process-wide shape decomposition cache. Upstream's file-local
/// `getBodyDecompositionCache()` function-local `static BodyDecompositionCache
/// cache;`. Upstream's `clean_count_`/`MAX_CLEAN_COUNT` bookkeeping exists
/// only to gate a cache-eviction pass upstream itself never implemented
/// (`// TODO - clean cache`, right where `clean_count_` would be read) --
/// there is no behaviour to port, so this cache never evicts, matching
/// upstream's actual (not intended) behaviour exactly.
/// `Weak<Shape>` pins the cached shape's allocation address so it can never
/// be reused by a later, unrelated shape; see this module's doc comment.
type BodyDecompositionCacheEntry = (Weak<Shape>, Arc<BodyDecomposition>);

fn body_decomposition_cache() -> &'static Mutex<HashMap<usize, BodyDecompositionCacheEntry>> {
    static CACHE: OnceLock<Mutex<HashMap<usize, BodyDecompositionCacheEntry>>> = OnceLock::new();
    CACHE.get_or_init(|| Mutex::new(HashMap::new()))
}

/// Upstream free function `getBodyDecompositionCacheEntry`.
///
/// See this module's doc comment for the resolution-blind cache-key defect
/// this preserves, and for why the key is `Arc::as_ptr(shape) as usize`
/// rather than a `Weak`-based port of `shapes::ShapeConstWeakPtr`.
///
/// `padding` is always [`BodyDecomposition::DEFAULT_PADDING`], matching
/// upstream's `BodyDecomposition(shape, resolution)` two-argument
/// constructor call, which relies on that header's `padding = 0.01` default
/// argument -- Rust has no default parameters, so this port hardcodes the
/// same value at this one call site rather than accepting a `padding`
/// parameter this function's own upstream signature does not have.
///
/// # Errors
///
/// See [`BodyDecomposition::new`].
pub fn get_body_decomposition_cache_entry(
    shape: &Arc<Shape>,
    resolution: f64,
) -> Result<Arc<BodyDecomposition>> {
    let key = Arc::as_ptr(shape) as usize;

    {
        let cache = body_decomposition_cache()
            .lock()
            .expect("body decomposition cache mutex poisoned");
        if let Some((_, existing)) = cache.get(&key) {
            return Ok(Arc::clone(existing));
        }
    }

    let decomposition = Arc::new(BodyDecomposition::new(
        shape,
        resolution,
        BodyDecomposition::DEFAULT_PADDING,
    )?);

    let mut cache = body_decomposition_cache()
        .lock()
        .expect("body decomposition cache mutex poisoned");
    cache.insert(key, (Arc::downgrade(shape), Arc::clone(&decomposition)));
    Ok(decomposition)
}

/// Upstream free function `getCollisionObjectPointDecomposition`.
///
/// Upstream builds each [`PosedBodyPointDecomposition`] at identity via the
/// one-argument constructor, adds it to the result vector, then re-poses it
/// in place through the vector's own `updatePose`. This port calls
/// [`PosedBodyPointDecomposition::with_pose`] directly instead -- posing
/// once at construction rather than once at identity and then again at the
/// real pose -- which lands on the same final `posed_collision_points` for
/// every point (`transform_as_point` at identity is the identity function,
/// so upstream's extra step is redundant, not semantically different; see
/// [`PosedBodyPointDecomposition::new`]'s own doc comment for the same
/// observation).
///
/// # Errors
///
/// [`moveit_error::Error::Construct`] if any of `obj`'s shapes has no
/// `bodies::` counterpart -- see [`get_body_decomposition_cache_entry`]/
/// [`BodyDecomposition::new`].
pub fn collision_object_point_decomposition(
    obj: &Object,
    resolution: f64,
) -> Result<PosedBodyPointDecompositionVector> {
    let mut result = PosedBodyPointDecompositionVector::new();
    for shape_entry in obj.shapes() {
        let body_decomposition =
            get_body_decomposition_cache_entry(shape_entry.shape(), resolution)?;
        result.add_to_vector(PosedBodyPointDecomposition::with_pose(
            body_decomposition,
            shape_entry.global_pose(),
        ));
    }
    Ok(result)
}

/// Upstream free function `getAttachedBodySphereDecomposition`: one
/// [`PosedBodySphereDecomposition`] per shape in `attached.shapes`, each
/// posed into the frame `link_transform` names.
///
/// Upstream reads `att->getGlobalCollisionBodyTransforms()[i]` -- each
/// shape's pose already resolved into the world frame by the `RobotState`
/// this crate does not carry attached bodies on. This port's
/// [`AttachedBodyGeometry::shape_poses`] is link-relative instead (see that
/// field's own doc comment), so the caller supplies `link_transform` (the
/// attached-to link's own global transform) explicitly and this function
/// composes `link_transform * shape_poses[i]` to land on the same global
/// pose upstream reads directly.
///
/// # Errors
///
/// [`moveit_error::Error::Construct`] if any of `attached.shapes` has no
/// `bodies::` counterpart -- see [`get_body_decomposition_cache_entry`]/
/// [`BodyDecomposition::new`].
pub fn attached_body_sphere_decomposition(
    attached: &AttachedBodyGeometry<'_>,
    link_transform: Isometry3,
    resolution: f64,
) -> Result<PosedBodySphereDecompositionVector> {
    let mut result = PosedBodySphereDecompositionVector::new();
    for (shape, shape_pose) in attached.shapes.iter().zip(attached.shape_poses) {
        let body_decomposition = get_body_decomposition_cache_entry(shape, resolution)?;
        let mut pbd = PosedBodySphereDecomposition::new(body_decomposition);
        pbd.update_pose(link_transform * *shape_pose);
        result.add_to_vector(pbd);
    }
    Ok(result)
}

/// Upstream free function `getAttachedBodyPointDecomposition`. See
/// [`attached_body_sphere_decomposition`]'s doc comment for why this port
/// takes `link_transform` explicitly to compose the global pose upstream
/// reads directly off `att->getGlobalCollisionBodyTransforms()`.
///
/// # Errors
///
/// Same as [`attached_body_sphere_decomposition`].
pub fn attached_body_point_decomposition(
    attached: &AttachedBodyGeometry<'_>,
    link_transform: Isometry3,
    resolution: f64,
) -> Result<PosedBodyPointDecompositionVector> {
    let mut result = PosedBodyPointDecompositionVector::new();
    for (shape, shape_pose) in attached.shapes.iter().zip(attached.shape_poses) {
        let body_decomposition = get_body_decomposition_cache_entry(shape, resolution)?;
        result.add_to_vector(PosedBodyPointDecomposition::with_pose(
            body_decomposition,
            link_transform * *shape_pose,
        ));
    }
    Ok(result)
}

#[cfg(test)]
mod tests {
    use super::*;
    use moveit_geometry::{Isometry3, Sphere};
    use nalgebra::Translation3;

    fn sphere_shape(radius: f64) -> Arc<Shape> {
        Arc::new(Shape::Sphere(Sphere::new(radius).unwrap()))
    }

    /// Invariant boundary: two different `Arc<Shape>` instances with
    /// *identical* shape values must get independent cache entries --
    /// proving the cache is keyed on identity, not on shape value, matching
    /// upstream's `owner_less`-on-`ShapeConstWeakPtr` semantics (see this
    /// module's doc comment).
    #[test]
    fn cache_entry_is_keyed_on_shape_identity_not_shape_value() {
        let shape_a = sphere_shape(0.2);
        let shape_b = sphere_shape(0.2);

        let entry_a = get_body_decomposition_cache_entry(&shape_a, 0.05).unwrap();
        let entry_b = get_body_decomposition_cache_entry(&shape_b, 0.05).unwrap();

        assert!(
            !Arc::ptr_eq(&entry_a, &entry_b),
            "two distinct Arc<Shape> with equal values must not share a cache entry"
        );
    }

    /// The same shape queried twice returns the exact same cached `Arc`,
    /// not merely an equal one -- upstream's whole point in having a cache
    /// at all.
    #[test]
    fn cache_entry_is_reused_for_the_same_shape() {
        let shape = sphere_shape(0.2);

        let first = get_body_decomposition_cache_entry(&shape, 0.05).unwrap();
        let second = get_body_decomposition_cache_entry(&shape, 0.05).unwrap();

        assert!(
            Arc::ptr_eq(&first, &second),
            "the same Arc<Shape> queried twice must hit the cache"
        );
    }

    /// The documented defect, pinned concretely: the *same* `Arc<Shape>`
    /// queried at two different resolutions returns the *first* call's
    /// decomposition both times, even though a fresh, uncached rebuild at
    /// the second resolution produces a different interior point count.
    /// This is the exact shape of bug the module doc warns will not show up
    /// in any single-query test -- it only appears by calling the cached
    /// entry point twice and comparing against an uncached rebuild.
    #[test]
    fn cache_entry_ignores_a_changed_resolution_for_the_same_shape() {
        // A sphere large enough that resolution actually changes the
        // interior point count `find_internal_points_convex` produces.
        let shape = sphere_shape(1.0);

        let coarse = get_body_decomposition_cache_entry(&shape, 0.5).unwrap();
        let cached_at_fine_request = get_body_decomposition_cache_entry(&shape, 0.01).unwrap();

        assert!(
            Arc::ptr_eq(&coarse, &cached_at_fine_request),
            "documented defect: a second call with a different resolution still \
             returns the first call's cached decomposition"
        );

        let fresh_fine =
            BodyDecomposition::new(&shape, 0.01, BodyDecomposition::DEFAULT_PADDING).unwrap();
        assert_ne!(
            cached_at_fine_request.collision_points().len(),
            fresh_fine.collision_points().len(),
            "test setup must pick resolutions that actually produce different point counts, \
             otherwise this test cannot distinguish the cached-stale value from a correct one"
        );
    }

    /// Regression test for a real defect this port introduced (not an
    /// upstream one): the cache used to key on a bare
    /// `Arc::as_ptr(shape) as usize` with nothing pinning that address, so
    /// once the original `Arc<Shape>` was dropped, Rust's allocator was
    /// free to hand the same address to a later, unrelated `Arc<Shape>`
    /// allocation -- which then silently received the *first* shape's
    /// cached `BodyDecomposition`. This reproduced concretely via
    /// `collision_object_point_decomposition_matches_the_oracle` in
    /// `collision_common_distance_field_parity.rs`: a sphere and a box
    /// built and dropped back-to-back landed at the same address on this
    /// allocator, and the box's decomposition silently came back as the
    /// sphere's.
    ///
    /// The fix stores a `Weak<Shape>` alongside the cached value, which
    /// pins the allocation for as long as the (never-evicted) cache entry
    /// exists. This test allocates and drops many differently-sized shapes
    /// in a tight loop -- the same shape/drop churn that triggered the bug
    /// -- and checks every returned decomposition against an independent,
    /// freshly rebuilt one, so a mixed-up cache entry is caught regardless
    /// of whether this particular run actually reuses an address.
    #[test]
    fn cache_entry_survives_the_original_arc_shape_being_dropped() {
        let mut decompositions = Vec::new();
        for i in 0..64_u32 {
            let radius = 0.05 + f64::from(i) * 0.01;
            let shape = sphere_shape(radius);
            let decomposition = get_body_decomposition_cache_entry(&shape, 0.05).unwrap();
            decompositions.push((radius, decomposition));
            // `shape`'s only strong reference is dropped here.
        }
        for (radius, decomposition) in &decompositions {
            let expected = BodyDecomposition::new(
                &sphere_shape(*radius),
                0.05,
                BodyDecomposition::DEFAULT_PADDING,
            )
            .unwrap();
            assert_eq!(
                decomposition.collision_points().len(),
                expected.collision_points().len(),
                "cached decomposition for radius {radius} does not match a fresh rebuild -- \
                 a later shape's allocation appears to have reused an earlier, dropped \
                 shape's address"
            );
        }
    }

    #[test]
    fn collision_object_point_decomposition_poses_every_shape_by_its_global_pose() {
        let mut world = moveit_collision::World::new();
        let object_pose = Isometry3::from_parts(
            Translation3::new(1.0, 0.0, 0.0),
            nalgebra::UnitQuaternion::identity(),
        );
        let shape_pose = Isometry3::from_parts(
            Translation3::new(0.0, 2.0, 0.0),
            nalgebra::UnitQuaternion::identity(),
        );
        world
            .add_to_object(
                "obj",
                object_pose,
                &[sphere_shape(0.3)],
                std::slice::from_ref(&shape_pose),
            )
            .unwrap();
        let obj = world.get_object("obj").unwrap();

        let result = collision_object_point_decomposition(&obj, 0.05).unwrap();

        assert_eq!(result.len(), 1);
        let global_pose = object_pose * shape_pose;
        let expected = result.get(0).unwrap().collision_points().to_vec();
        // Every posed point must equal the unposed point transformed by the
        // shape's global pose, not merely the object's own pose.
        let direct = PosedBodyPointDecomposition::with_pose(
            get_body_decomposition_cache_entry(obj.shapes()[0].shape(), 0.05).unwrap(),
            global_pose,
        );
        assert_eq!(expected, direct.collision_points());
    }
}
