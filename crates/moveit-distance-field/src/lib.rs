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
