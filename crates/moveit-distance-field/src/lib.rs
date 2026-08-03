// Copyright (c) 2009, Willow Garage, Inc.
// Copyright (c) 2010, Willow Garage, Inc.
// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/distance_field/include/moveit/distance_field/voxel_grid.hpp
//   moveit_core/distance_field/include/moveit/distance_field/distance_field.hpp
//   moveit_core/distance_field/include/moveit/distance_field/propagation_distance_field.hpp
//   moveit_core/distance_field/include/moveit/distance_field/find_internal_points.hpp
//   moveit_core/distance_field/src/distance_field.cpp
//   moveit_core/distance_field/src/propagation_distance_field.cpp
//   moveit_core/distance_field/src/find_internal_points.cpp
//   moveit_core/collision_distance_field/include/moveit/collision_distance_field/collision_distance_field_types.hpp
//   moveit_core/collision_distance_field/src/collision_distance_field_types.cpp

//! Voxel distance fields for moveit-rs: dense 3D grids holding the distance
//! from every cell to the nearest obstacle.
//!
//! # Scope
//!
//! This crate ports `moveit_core/distance_field` in full, plus
//! `moveit_core/collision_distance_field`'s body-decomposition,
//! cache-entry-construction, and per-group-state machinery:
//! `collision_distance_field_types` (no `RobotModel` dependency),
//! `collision_common_distance_field`'s `RobotState`/`RobotModel`-dependent
//! half plus its [`DistanceFieldCacheEntry`]/[`GroupStateRepresentation`]
//! structs, and `collision_env_distance_field`'s construction/query slice
//! (`addLinkBodyDecompositions`, `generateDistanceFieldCacheEntry`,
//! `getDistanceFieldCacheEntry`, `getGroupStateRepresentation`,
//! `updateGroupStateRepresentationState`). The collision *checker* itself
//! (`CollisionEnvDistanceField::checkCollision` and friends, plus its own
//! persistent cache-owner role -- `generateCollisionCheckingStructures`)
//! belongs to a later phase; see `PORTING-PLAN.md` §3 and
//! `collision_env_distance_field.rs`'s own module doc comment for specifics.
//!
//! - [`VoxelGrid`] — the generic dense grid, with the world↔cell coordinate
//!   conversion whose rounding convention is load-bearing (see its `impl`
//!   doc on [`VoxelGrid::cell_from_location`]).
//! - [`DistanceField`] — the query interface. Per `PORTING-PLAN.md` D4 this
//!   is a trait rather than upstream's abstract base class.
//! - [`PropagationDistanceField`] — the (currently only) implementer,
//!   propagating distances outward via bucketed-queue wavefront expansion.
//! - [`find_internal_points_convex`] / [`ConvexBody`] — the point-sampling
//!   helper used to turn a shape into obstacle points.
//! - [`PosedDistanceField`], [`BodyDecomposition`] and the `Posed*` sphere/
//!   point decomposition types — see [`PosedDistanceField`]'s own doc
//!   comment for the composition-over-inheritance design note.
//! - [`get_body_decomposition_cache_entry`] / [`collision_object_point_decomposition`]
//!   — the `RobotState`/`RobotModel`-dependent slice of
//!   `collision_common_distance_field`; see that function's own doc comment
//!   for what is deferred and why.
//! - [`add_link_body_decompositions`] / [`generate_distance_field_cache_entry`] /
//!   [`DistanceFieldConfig`] — `collision_env_distance_field`'s
//!   construction-only slice; see [`add_link_body_decompositions`]'s doc
//!   comment for the remaining dependency gap and
//!   [`generate_distance_field_cache_entry`]'s own doc comment for what it
//!   builds.
//! - [`compare_cache_entry_to_state`] / [`compare_cache_entry_to_allowed_collision_matrix`]
//!   / [`get_distance_field_cache_entry`] — decide whether a
//!   [`DistanceFieldCacheEntry`] is still valid for a new `RobotState`/
//!   `AllowedCollisionMatrix`; see `collision_env_distance_field`'s module
//!   doc for what is still deferred around them.
//! - [`DistanceFieldCacheEntry`] — the group-, ACM-, and robot-state-specific
//!   cache entry [`generate_distance_field_cache_entry`] populates; see its
//!   own doc comment for what upstream field it deliberately leaves unset.
//! - [`GroupStateRepresentation`] / [`group_state_representation`] /
//!   [`update_group_state_representation_state`] — the per-group posed
//!   sphere-decomposition-plus-distance-field bundle a collision check
//!   queries against; see [`group_state_representation`]'s own doc comment
//!   for the uninitialized-gradient defect it preserves (more defined, not
//!   less) from upstream's fresh-build path.
//! - [`AttachedBodySnapshot`] — closes a real cache-invalidation gap in
//!   [`compare_cache_entry_to_state`]; see its own doc comment.
//!
//! See [`DistanceField`]'s doc comment for what upstream's abstract base
//! class carries that is deliberately *not* ported here, and why.
//!
//! # Symbol audit: every public symbol under `collision_distance_field/include/`
//!
//! Re-run by re-reading the headers fresh, not by inferring from what is
//! already ported: `collision_common_distance_field.h`,
//! `collision_distance_field_types.h`, `collision_env_distance_field.h`,
//! `collision_env_hybrid.h`, `collision_detector_allocator_distance_field.h`
//! and `collision_detector_allocator_hybrid.h` are all deprecated
//! auto-generated forwarding shims to the `.hpp` of the same stem
//! (`#pragma message(".h header is obsolete...")`, then one `#include`); no
//! independent content, so only the six `.hpp` files carry real symbols.
//! `ported as <symbol>` gives the Rust name; `D-decision excludes it` names
//! the decision; `unported` gives the reason it is not (yet, or ever) ported.
//!
//! ## Whole-file exclusions
//!
//! - `collision_env_hybrid.hpp` (`CollisionEnvHybrid`) — extends
//!   `collision_detection::CollisionEnvFCL` directly. D-decision:
//!   `PORTING-PLAN.md`'s FCL/Bullet → `parry3d-f64` backend replacement
//!   (lines 232–233) means `CollisionEnvFCL` itself is never ported, so
//!   nothing depending on it directly can be either.
//! - `collision_detector_allocator_distance_field.hpp`
//!   (`CollisionDetectorAllocatorDistanceField`) and
//!   `collision_detector_allocator_hybrid.hpp`
//!   (`CollisionDetectorAllocatorHybrid`) — both
//!   `CollisionDetectorAllocatorTemplate<...>` ROS-pluginlib-style runtime
//!   plugin registrations. D-decision: D4 (this port's plugin model is a
//!   compile-time trait + `linkme` registry, not a runtime allocator
//!   class). Each also depends on its (separately excluded) `CollisionEnv*`
//!   type.
//!
//! ## `collision_common_distance_field.hpp`
//!
//! - `GroupStateRepresentation` (struct) — ported as [`GroupStateRepresentation`].
//! - `DistanceFieldCacheEntry` (struct) — ported as [`DistanceFieldCacheEntry`].
//! - `getBodyDecompositionCacheEntry` — ported as [`get_body_decomposition_cache_entry`].
//! - `getCollisionObjectPointDecomposition` — ported as [`collision_object_point_decomposition`].
//! - `getAttachedBodySphereDecomposition` — unported: takes a
//!   `moveit::core::AttachedBody*` and builds a real posed decomposition of
//!   its geometry; unreachable from a bare `RobotState` (see
//!   `collision_common_distance_field.rs`'s module doc, "Deferred, and why").
//! - `getAttachedBodyPointDecomposition` — unported, same reason.
//! - `getBodySphereVisualizationMarkers` — D-decision excludes it: D1 (no
//!   ROS message types / renderer outside the optional `moveit-ros` crate).
//!
//! ## `collision_distance_field_types.hpp`
//!
//! - `CollisionType` (enum) — ported as [`CollisionType`].
//! - `CollisionSphere` (struct) — ported as [`CollisionSphere`].
//! - `GradientInfo` (struct, incl. `clear()`) — ported as [`GradientInfo`]
//!   (incl. [`GradientInfo::clear`]).
//! - `PosedDistanceField` (class) — ported as [`PosedDistanceField`]:
//!   `updatePose`/`getPose` as [`PosedDistanceField::update_pose`]/
//!   [`PosedDistanceField::pose`]; the *member* `getDistanceGradient` as
//!   [`PosedDistanceField::distance_gradient`]; the *member*
//!   `getCollisionSphereGradients` as the method
//!   [`PosedDistanceField::get_collision_sphere_gradients`] — distinct from,
//!   and independently ported alongside, the free function below.
//! - `determineCollisionSpheres` — ported as [`determine_collision_spheres`].
//! - `getCollisionSphereGradients` (free function, takes an explicit
//!   `distance_field::DistanceField*`) — ported as the free function
//!   [`get_collision_sphere_gradients`].
//! - `getCollisionSphereCollision` (bool-only overload, no output param) —
//!   ported as [`get_collision_sphere_collision`].
//! - `getCollisionSphereCollision` (`num_coll`/`colls` output-param overload)
//!   — ported as [`get_collision_sphere_collisions`] (plural, distinguishing
//!   it from the overload above since Rust has no overloading).
//! - `BodyDecompositionVector` — unported: forward-declared and friended by
//!   `BodyDecomposition` (`collision_distance_field_types.hpp:226,230`) but
//!   never defined anywhere in the upstream tree — `grep -rn "class
//!   BodyDecompositionVector\|BodyDecompositionVector::"` against the full
//!   `/home/stevek/work/moveit2/` checkout returns only those two lines.
//!   Phantom upstream code; there is nothing to port.
//! - `BodyDecomposition` (class, 2 constructor overloads) — ported as
//!   [`BodyDecomposition`], the overloads collapsed to
//!   [`BodyDecomposition::new`] (single shape) and
//!   [`BodyDecomposition::from_shapes`] (multiple shapes + poses).
//! - `PosedBodySphereDecomposition` (class) — ported as
//!   [`PosedBodySphereDecomposition`].
//! - `PosedBodyPointDecomposition` (class, 3 constructor overloads) — ported
//!   for 2 of 3: `PosedBodyPointDecomposition(body_decomposition)`/
//!   `PosedBodyPointDecomposition(body_decomposition, pose)` as
//!   [`PosedBodyPointDecomposition::new`]/[`PosedBodyPointDecomposition::with_pose`].
//!   The third, `PosedBodyPointDecomposition(const std::shared_ptr<const
//!   octomap::OcTree>&)`, is unported: no crate in this workspace ports
//!   `octomap::OcTree` or an equivalent octree type, so there is no input
//!   type to build it from.
//! - `PosedBodySphereDecompositionVector` (class) — ported as
//!   [`PosedBodySphereDecompositionVector`] (`getSize`/
//!   `getPosedBodySphereDecomposition` as
//!   [`PosedBodySphereDecompositionVector::len`]/
//!   [`PosedBodySphereDecompositionVector::is_empty`]/
//!   [`PosedBodySphereDecompositionVector::get`]).
//! - `PosedBodyPointDecompositionVector` (class) — ported as
//!   [`PosedBodyPointDecompositionVector`], same renaming pattern.
//! - `ProximityInfo` (struct) — ported as [`ProximityInfo`].
//! - `doBoundingSpheresIntersect` — ported as [`do_bounding_spheres_intersect`].
//! - `getCollisionSphereMarkers` — D-decision excludes it: D1.
//! - `getProximityGradientMarkers` — D-decision excludes it: D1.
//! - `getCollisionMarkers` — D-decision excludes it: D1.
//!
//! ## `collision_env_distance_field.hpp`
//!
//! `DEFAULT_SIZE_X`/`_Y`/`_Z`, `DEFAULT_USE_SIGNED_DISTANCE_FIELD`,
//! `DEFAULT_RESOLUTION`, `DEFAULT_COLLISION_TOLERANCE`,
//! `DEFAULT_MAX_PROPOGATION_DISTANCE` — unported: every one is a default
//! constructor argument of `CollisionEnvDistanceField` itself (unported,
//! below); `DEFAULT_COLLISION_TOLERANCE` specifically backs
//! `collision_tolerance_`, read only by checker-level methods
//! (`checkSelfCollision`, `getSelfProximityGradients`, ...), none of which
//! are ported — not a gap in [`DistanceFieldConfig`], which already carries
//! every field the functions this crate *does* port actually consume.
//!
//! `CollisionEnvDistanceField` (class) — unported in its entirety: the
//! collision *checker* itself, a later phase (see "Still blocked, and why"
//! in `collision_env_distance_field.rs`'s module doc). This covers every
//! public method not listed as ported below (3 constructors, `initialize`,
//! `checkSelfCollision` ×4, `checkCollision` ×4, `checkRobotCollision` ×6,
//! `distanceSelf` ×3, `distanceRobot` ×3 — the `DistanceRequest` overloads
//! of both are themselves stubbed upstream to
//! `RCLCPP_ERROR("Not implemented")` — `setWorld`, `getDistanceField`,
//! `getLastGroupStateRepresentation`, `getCollisionGradients`,
//! `getAllCollisions`, `getLastDistanceFieldEntry`, the nested
//! `DistanceFieldCacheEntryWorld` struct, and the destructor;
//! `createCollisionModelMarker` additionally falls under D1) and every
//! protected method not listed as ported below (`getSelfProximityGradients`,
//! `getIntraGroupProximityGradients`, `getSelfCollisions`,
//! `getIntraGroupCollisions`, `checkSelfCollisionHelper`,
//! `updatedPaddingOrScaling` (a no-op override of the `CollisionEnv`
//! interface), `generateDistanceFieldCacheEntryWorld`, `updateDistanceObject`,
//! `getEnvironmentCollisions`, `getEnvironmentProximityGradients`,
//! `notifyObjectChange` — the last six specifically `World`-dependent, a
//! dependency this crate deliberately does not take).
//!
//! Of `CollisionEnvDistanceField`'s protected methods, these ARE ported —
//! as free functions or a [`DistanceFieldCollisionCache`] method rather than
//! staying methods of the unported class; see
//! `collision_env_distance_field.rs`'s own module doc for why each is a free
//! function/narrow type instead:
//!
//! - `updateGroupStateRepresentationState` — ported as
//!   [`update_group_state_representation_state`].
//! - `generateCollisionCheckingStructures` — ported as
//!   [`DistanceFieldCollisionCache::generate_collision_checking_structures`].
//! - `getDistanceFieldCacheEntry` — ported as [`get_distance_field_cache_entry`].
//! - `generateDistanceFieldCacheEntry` — ported as
//!   [`generate_distance_field_cache_entry`].
//! - `addLinkBodyDecompositions` (2 overloads) — ported as
//!   [`add_link_body_decompositions`] (collapsed to one function).
//! - `getPosedLinkBodySphereDecomposition`/`getPosedLinkBodyPointDecomposition`
//!   — unported: trivial one-line wrappers whose only callers
//!   ([`group_state_representation`] and this crate's own
//!   `build_non_group_distance_field`) inline the equivalent
//!   `PosedBodySphereDecomposition`/`PosedBodyPointDecomposition`
//!   constructor call directly instead.
//! - `getGroupStateRepresentation` — ported as [`group_state_representation`].
//! - `compareCacheEntryToState` — ported as [`compare_cache_entry_to_state`].
//! - `compareCacheEntryToAllowedCollisionMatrix` — ported as
//!   [`compare_cache_entry_to_allowed_collision_matrix`].
//!
//! Member fields:
//!
//! - `size_`, `origin_`, `use_signed_distance_field_`, `resolution_`,
//!   `max_propogation_distance_` — ported as [`DistanceFieldConfig`]'s
//!   fields (`origin_`/`size_` additionally require the center-to-corner
//!   shift documented on [`DistanceFieldConfig::geometry`]).
//! - `collision_tolerance_` — unported; see the `DEFAULT_COLLISION_TOLERANCE`
//!   note above.
//! - `link_body_decomposition_vector_`/`link_body_decomposition_index_map_`
//!   — ported as the `LinkBodyDecompositions` pair
//!   [`add_link_body_decompositions`] returns.
//! - `update_cache_lock_` — unported by design: exists only as a
//!   `const`-method workaround (`const_cast<CollisionEnvDistanceField*>(this)`
//!   in `generateCollisionCheckingStructures`'s body, needed because
//!   `checkCollision`/`checkSelfCollision` must stay `const`); a `&mut self`
//!   method gives the same single-writer guarantee at compile time, so there
//!   is no mutex field to port.
//! - `distance_field_cache_entry_` — ported as
//!   [`DistanceFieldCollisionCache`]'s private `cache_entry` field.
//! - `in_group_update_map_` — **unported as a field**: `generateCollisionCheckingStructures`'s
//!   own body never touches it as a cached value; the same information is
//!   computed inline, once per call, by [`generate_distance_field_cache_entry`]
//!   (`state.model().joint_model_group(group_name)?.updated_link_with_geometry_names()`)
//!   rather than cached across calls the way upstream's `initialize()`
//!   precomputes it for every group up front. Same information, computed
//!   fresh instead of cached — not a missing map, a different recomputation
//!   strategy for the one caller this crate has.
//! - `pregenerated_group_state_representation_map_` — **unported as a
//!   field, and provably unreachable, not merely unimplemented**: populated
//!   only inside `CollisionEnvDistanceField::initialize()` (unported,
//!   checker-level, above), which eagerly builds one
//!   `DistanceFieldCacheEntry` + `GroupStateRepresentation` pair per joint
//!   model group at construction time. It is read in exactly one place,
//!   `generateDistanceFieldCacheEntry`'s
//!   `dfce->pregenerated_group_state_representation_ = it->second`, and that
//!   field is read only by `getGroupStateRepresentation`'s "already
//!   pregenerated" branch — see [`group_state_representation`]'s own doc
//!   comment, "Deviations from upstream", for the proof that branch cannot
//!   be reached: [`DistanceFieldCacheEntry`] in this port has no such field
//!   at all, since there is no `initialize`-equivalent constructor in this
//!   crate's scope to populate it from, so every `DistanceFieldCacheEntry`
//!   this port builds takes the fresh-build branch, unconditionally.
//! - `planning_scene_` — unported; `PlanningScene`-dependent, checker-level
//!   construction state (built once in `initialize()`, used only to source
//!   a default-empty `AllowedCollisionMatrix` for the pregeneration loop
//!   above).
//! - `update_cache_lock_world_`/`distance_field_cache_entry_world_`/
//!   `last_gsr_`/`observer_handle_` — unported; `World`-dependent
//!   checker-level state (the environment-object half of the checker, as
//!   opposed to the robot-link half [`DistanceFieldCollisionCache`] covers).
//! - `logger_` — unported; ROS logging, not carried by this crate's
//!   ROS-independent scope (PORTING-PLAN.md D1).

mod collision_common_distance_field;
mod collision_distance_field_types;
mod collision_env_distance_field;
mod distance_field;
mod find_internal_points;
mod propagation;
mod voxel_grid;

pub use collision_common_distance_field::{
    AttachedBodySnapshot, DistanceFieldCacheEntry, GroupStateRepresentation,
    collision_object_point_decomposition, get_body_decomposition_cache_entry,
};
pub use collision_distance_field_types::{
    BodyDecomposition, CollisionSphere, CollisionType, GradientInfo, PosedBodyPointDecomposition,
    PosedBodyPointDecompositionVector, PosedBodySphereDecomposition,
    PosedBodySphereDecompositionVector, PosedDistanceField, ProximityInfo, SphereGradientQuery,
    determine_collision_spheres, do_bounding_spheres_intersect, get_collision_sphere_collision,
    get_collision_sphere_collisions, get_collision_sphere_gradients,
};
pub use collision_env_distance_field::{
    DistanceFieldCollisionCache, DistanceFieldConfig, add_link_body_decompositions,
    compare_cache_entry_to_allowed_collision_matrix, compare_cache_entry_to_state,
    generate_distance_field_cache_entry, get_distance_field_cache_entry,
    group_state_representation, update_group_state_representation_state,
};
pub use distance_field::{DistanceField, DistanceGradient};
pub use find_internal_points::{ConvexBody, find_internal_points_convex};
pub use propagation::{NearestCell, PropDistanceFieldVoxel, PropagationDistanceField};
pub use voxel_grid::{Dimension, GridGeometry, VoxelGrid};
