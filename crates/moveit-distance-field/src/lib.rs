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
//! # Completion condition
//!
//! PORTING-PLAN.md previously claimed a gap this crate had already closed
//! (§65/§71) — a claim nobody had a way to check against the code, only to
//! trust. This section is that check: it names exactly what "done" means for
//! this crate's current scope, so plan and code can be compared directly
//! instead of re-diverging silently.
//!
//! **Headers, fully audited (read in full against the pinned SHA, not
//! inferred from what is already ported):**
//!
//! - `moveit_core/distance_field/include/moveit/distance_field/{distance_field,propagation_distance_field,voxel_grid,find_internal_points}.hpp`
//!   plus their four `.h` deprecated-forwarding-shim siblings — see the
//!   "Symbol audit: every public symbol under `moveit_core/distance_field/`"
//!   section below for the per-symbol table.
//! - `moveit_core/collision_distance_field/include/moveit/collision_distance_field/*.hpp`
//!   (six headers: `collision_common_distance_field`,
//!   `collision_distance_field_types`, `collision_env_distance_field`,
//!   `collision_env_hybrid`, `collision_detector_allocator_distance_field`,
//!   `collision_detector_allocator_hybrid`) plus their six `.h` shims — see
//!   the "Symbol audit: every public symbol under
//!   `collision_distance_field/include/`" section below.
//!
//! Every symbol in both packages is classified in those two sections as
//! ported (with its Rust name), D-decision-excluded (with the decision), or
//! unported (with the specific reason) — there is no symbol from either
//! package left unclassified.
//!
//! **Fixtures, and what they cover:**
//!
//! - `tests/upstream_parity.rs` — every case upstream's own
//!   `test_distance_field.cpp` gtest suite carries, minus `TestOcTree`
//!   (needs the unported `DistanceField::addOcTreeToField`) and
//!   `TestPerformance` (a benchmark, not an assertion) — see that file's own
//!   module doc for the two exclusions' reasoning.
//! - `tests/boundaries.rs` — invariant-boundary cases upstream's suite does
//!   not carry: a point exactly on a cell boundary, a point outside the
//!   grid, add-then-remove returning to the reset state (signed and
//!   unsigned), and incremental update vs. full rebuild agreement.
//! - `tests/oracle_parity.rs` — [`PropagationDistanceField`]'s own
//!   `grid_to_world`/`distance`/`distance_cell`/`distance_gradient`/
//!   `nearest_cell` against the oracle's `distance_field` op, both
//!   `propagate_negative` settings, exact comparison (see that file's
//!   "Exactness" doc section for why a tolerance does not apply here).
//! - `tests/shape_points_parity.rs` — [`find_internal_points_convex`]
//!   against the oracle's `shape_points` op, for all four `bodies::` shape
//!   kinds (Sphere, Box, Cylinder, Mesh).
//! - `tests/collision_distance_field_types_parity.rs` — `BodyDecomposition`'s
//!   sphere decomposition/bounding sphere, [`PosedDistanceField::distance_gradient`]
//!   and [`PosedDistanceField::get_collision_sphere_gradients`] (the member
//!   overload) against the oracle, all four shape kinds at two resolutions
//!   each.
//! - `tests/collision_sphere_free_functions_parity.rs` — the free
//!   [`get_collision_sphere_gradients`]/[`get_collision_sphere_collision`]/
//!   [`get_collision_sphere_collisions`] functions against the oracle,
//!   hand-built to cover every reachable branch (see that file's module doc
//!   for the one upstream guard no fixture can reach, and why).
//! - `tests/collision_common_distance_field_parity.rs` —
//!   [`collision_object_point_decomposition`] and
//!   `BodyDecomposition::from_shapes`/`PosedBodySphereDecomposition`
//!   composed the way [`add_link_body_decompositions`] does, against the
//!   oracle, for a real PR2 link across a group state.
//! - `tests/collision_env_distance_field_parity.rs` — link-set selection
//!   (`link_models_with_collision_geometry`), [`add_link_body_decompositions`]
//!   against the per-link oracle fixture above, and
//!   [`generate_distance_field_cache_entry`]/[`group_state_representation`]
//!   indirectly through the oracle's `checkSelfCollision` ->
//!   `getLastDistanceFieldEntry()` path, across three PR2 groups/ACM
//!   configurations.
//! - Everything under `src/*.rs`'s own `#[cfg(test)]` modules — unit tests
//!   for behavior with no oracle op to compare against (cache invalidation,
//!   ACM comparison, attached-body handling; see
//!   `collision_env_distance_field.rs`'s and `collision_common_distance_field.rs`'s
//!   own module docs).
//!
//! **What is still missing, and why it is not a gap in the above:** every
//! item is already named individually in the two symbol-audit sections below
//! with its own reason; this is the roll-up. `CollisionEnvDistanceField`
//! itself (the collision *checker* — `checkCollision`/`checkSelfCollision`/
//! `checkRobotCollision`/`distanceSelf`/`distanceRobot` and its persistent
//! cache-owner role) is entirely unported, a later phase per
//! `PORTING-PLAN.md` §3 — see `collision_env_distance_field.rs`'s module doc,
//! "Still blocked, and why". `CollisionEnvHybrid` and both
//! `CollisionDetectorAllocator*` classes are D-decision-excluded (D1's
//! FCL/Bullet replacement, D4's compile-time-trait plugin model). The
//! `AttachedBody`-dependent decomposition functions
//! (`getAttachedBodySphereDecomposition`/`getAttachedBodyPointDecomposition`)
//! are unported because a bare `moveit_state::State` structurally cannot see
//! attached bodies (they live on `moveit_scene::PlanningScene`, which this
//! crate does not depend on — see `moveit-state`'s `State::frame_transform`
//! doc). The octree-backed `PosedBodyPointDecomposition` constructor is
//! unported because this crate has no dependency on `moveit-octomap` — that
//! crate now ports an `octomap::OcTree` equivalent, but nothing in this
//! crate's own dependency graph reaches it. Nothing above depends on
//! ROS message types, a renderer, or `World` in a way this crate's
//! `DistanceFieldCollisionCache`/link-decomposition scope does not already
//! account for.
//!
//! This crate's completion condition, stated as a check rather than a claim:
//! every symbol in both audited packages is classified above; every
//! classified-as-ported symbol has either an upstream-gtest-derived fixture
//! (`upstream_parity.rs`), an oracle-driven fixture (the `*_parity.rs` files
//! above), or a boundary/unit test with a documented reason no oracle op
//! covers it; and every classified-as-unported symbol names the specific
//! missing dependency or later-phase boundary, not "not yet". If a future
//! symbol or fixture cannot be placed in one of those buckets, this section
//! is stale and needs re-auditing before the plan is updated to match it.
//!
//! ## The `max_relative` trap (PORTING-PLAN.md §79)
//!
//! Diagnosed in this crate first (§71.2/§78.2), then found to be
//! workspace-wide (§79): `approx`'s `assert_relative_eq!(a, b, epsilon =
//! X)` does not compare against `X` alone. Its real pass condition is
//! `|a - b| <= epsilon` **or** `|a - b| <= max_relative * max(|a|, |b|)`,
//! and any call that omits `max_relative` gets `max_relative =
//! f64::EPSILON` (~2.22e-16) silently. Bisecting `epsilon` alone toward
//! `0.0` to find a tolerance's true binding point is unreliable for exactly
//! this reason: once `epsilon` drops below `f64::EPSILON * max(|a|, |b|)`,
//! the implicit `max_relative` term takes over and the assertion keeps
//! passing regardless of how low `epsilon` goes — not because the values
//! agree to that precision, but because the second, unnamed tolerance term
//! is still covering the real difference. This crate's own `RADIUS_TOL`
//! (see `collision_env_distance_field_parity.rs`'s doc comment) bisected
//! all the way to `0.0` before this was understood, and only revealed its
//! real floor (between `1e-18` and `1e-17`) once `max_relative` was pinned
//! to the same named constant explicitly.
//!
//! Two exits, per call site: pin `max_relative` to the same named constant
//! as `epsilon` (closes the trap, keeps the comparison a real gate at
//! whatever value the constant is bisected to), or — if bisecting `epsilon`
//! together with an explicit `max_relative` still finds no binding point
//! above `0.0` — drop the tolerance entirely and use `assert_eq!` instead,
//! as `oracle_parity.rs` and `collision_sphere_free_functions_parity.rs` do
//! (see the "Exactness" section of `oracle_parity.rs`'s own module doc). A
//! call site is *not* automatically defective for lacking `max_relative`:
//! `upstream_parity.rs`'s 7 calls use upstream's own literal `EXPECT_NEAR`
//! values (`0.0001`, `0.1`), which stay 12+ orders of magnitude above any
//! magnitude this file's 1x1x1 meter grid can produce, so the implicit term
//! can never dominate there — checked by bisecting all 7 to `0.0` together,
//! which fails immediately, confirming the named epsilon is what is
//! actually gating those assertions, not `approx`'s default.
//!
//! # Symbol audit: every public symbol under `collision_distance_field/include/`
//!
//! Re-run by re-reading the headers fresh, not by inferring from what is
//! already ported: `collision_common_distance_field.h`,
//! `collision_distance_field_types.h`, `collision_env_distance_field.h`,
//! `collision_env_hybrid.h`, `collision_detector_allocator_distance_field.h`
//! and `collision_detector_allocator_hybrid.h` are all deprecated
//! auto-generated forwarding shims to the `.hpp` of the same stem; no
//! independent content, so only the six `.hpp` files carry real symbols.
//! Established by directly reading all six `.h` files in full (not
//! inspecting one and assuming the rest match): each one's *entire* content
//! after its BSD license block is a doc comment naming the generator by name
//! (*"`.h` headers are now autogenerated via `create_deprecated_headers.py`,
//! and will import the corresponding `.hpp` with a deprecation warning"*,
//! citing `moveit/moveit2#3113`), then exactly three lines of code:
//! `#pragma once`, `#pragma message(".h header is obsolete. Please use the
//! .hpp header instead.")`, and one `#include` of the `.hpp` of the same
//! stem. No macros, no re-declarations, no symbols of their own.
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
//!   octomap::OcTree>&)`, is unported: `moveit-octomap` now ports an
//!   `octomap::OcTree` equivalent, but this crate has no dependency on it,
//!   so there is no input type in this crate's own scope to build it from.
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
//!   pregenerated" branch. The unreachability is a *type-level* guarantee
//!   here, not a call-graph argument that a new caller could invalidate:
//!   [`DistanceFieldCacheEntry`] has no `pregenerated_group_state_representation`
//!   field at all (see its field list below), and this port's
//!   [`group_state_representation`] has no corresponding early-return branch
//!   to read one — there is nothing to reach regardless of which function
//!   constructs the entry.
//!
//!   Re-derived against
//!   [`DistanceFieldCollisionCache::generate_collision_checking_structures`]
//!   (added in a later round, after this reasoning was first written): it is
//!   a *new caller* of the *same* [`generate_distance_field_cache_entry`]
//!   this note already accounts for, layering cache-reuse
//!   (`get_distance_field_cache_entry`) around it — it does not construct a
//!   `DistanceFieldCacheEntry` any other way, and does not touch
//!   `group_state_representation`'s branching. It does not change this
//!   conclusion.
//!
//!   **Falsifier** — this becomes reachable only if *all three* land
//!   together: (1) [`DistanceFieldCacheEntry`] gains a
//!   `pregenerated_group_state_representation` field; (2) some construction
//!   path populates it *before* first use — this crate's scope has no
//!   `initialize()`-equivalent eager per-group precomputation step to do
//!   that from (the `planning_scene_`/checker-construction state below is
//!   unported for the same reason); and (3) [`group_state_representation`]
//!   gains an early-return branch that reads it. Adding only (1) is inert
//!   (an unread field); adding only (3) cannot compile against today's
//!   [`DistanceFieldCacheEntry`] (the field it would read does not exist).
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
//!
//! # Symbol audit: every public symbol under `moveit_core/distance_field/`
//!
//! The audit above covers `collision_distance_field/include/`; this one
//! covers the *other* upstream package this crate carries —
//! `distance_field.hpp`, `propagation_distance_field.hpp`, `voxel_grid.hpp`
//! and `find_internal_points.hpp` — established by directly reading all
//! four `.hpp` files, their `.cpp`s, and their `.h` shims in full, not by
//! inferring from what is already ported. Same shim pattern as the six
//! headers above: `distance_field.h`, `propagation_distance_field.h`,
//! `voxel_grid.h` and `find_internal_points.h` are each the identical
//! three-line deprecated forwarding shim, confirmed the same way (reading
//! all four in full), so only the `.hpp` files carry real symbols. Same
//! classification key as above.
//!
//! ## `voxel_grid.hpp`
//!
//! - `Dimension` (enum) — ported as [`Dimension`].
//! - `MOVEIT_DECLARE_PTR_MEMBER(VoxelGrid)` — unported: a C++
//!   smart-pointer-typedef convenience macro, no Rust equivalent needed.
//! - `VoxelGrid(size_x, ..., default_object)` (sized constructor) — ported
//!   as [`VoxelGrid::new`], the six size/origin doubles bundled into
//!   [`GridGeometry`] (see that type's doc comment for why).
//! - `virtual ~VoxelGrid()` — unported: Rust ownership needs no destructor
//!   for what this type never allocates outside its own `Vec`.
//! - `VoxelGrid()` (default constructor) / `resize` — unported: see
//!   [`VoxelGrid`]'s own "Deviations from upstream" doc, first bullet.
//! - `operator()(double,double,double)` — ported as [`VoxelGrid::get`].
//! - `operator()(const Eigen::Vector3d&)`, `getCell(const Eigen::Vector3i&)`
//!   (const and non-const), `setCell(const Eigen::Vector3i&, const T&)` and
//!   `isCellValid(const Eigen::Vector3i&)` — unported: each is a one-line
//!   forward to the scalar overload beside it, and every call site in this
//!   crate already holds three separate components rather than an
//!   `Eigen::Vector3`-shaped value at the point it calls in — see
//!   [`VoxelGrid`]'s own "Deviations from upstream" doc for the `rg`
//!   evidence.
//! - `getCell(int,int,int)` (const and non-const) — ported as
//!   [`VoxelGrid::get_cell`]/[`VoxelGrid::get_cell_mut`].
//! - `setCell(int,int,int,const T&)` — ported as [`VoxelGrid::set_cell`].
//! - `reset(const T&)` — ported as [`VoxelGrid::reset`].
//! - `getSize`/`getResolution`/`getOrigin`/`getNumCells` — ported as
//!   [`VoxelGrid::size`]/[`VoxelGrid::resolution`]/[`VoxelGrid::origin`]/
//!   [`VoxelGrid::num_cells`].
//! - `gridToWorld(int,int,int,double&,double&,double&)` and
//!   `gridToWorld(const Eigen::Vector3i&, Eigen::Vector3d&)` — collapsed
//!   into [`VoxelGrid::grid_to_world`]; see that method's own doc comment
//!   for the deliberately asymmetric shape versus `world_to_grid` below.
//! - `worldToGrid(double,double,double,int&,int&,int&)` and
//!   `worldToGrid(const Eigen::Vector3d&, Eigen::Vector3i&)` — collapsed
//!   into [`VoxelGrid::world_to_grid`]; see that method's own doc comment.
//! - `isCellValid(int,int,int)` — ported as [`VoxelGrid::is_cell_valid`].
//! - `isCellValid(Dimension,int)` — ported as
//!   [`VoxelGrid::is_cell_valid_dim`].
//! - protected `ref` — ported as the private `VoxelGrid::index`, kept
//!   private since nothing outside this module needs a raw 1D index.
//! - protected `getCellFromLocation` — ported as
//!   [`VoxelGrid::cell_from_location`], made `pub` (this crate's rounding
//!   convention at cell boundaries lives here; see that method's own doc).
//! - protected `getLocationFromCell` — ported as
//!   [`VoxelGrid::location_from_cell`], made `pub`.
//! - fields `data_`/`default_object_`/`size_`/`resolution_`/
//!   `oo_resolution_`/`origin_`/`origin_minus_`/`num_cells_`/`stride1_`/
//!   `stride2_` — ported as [`VoxelGrid`]'s private fields of the same
//!   names minus the trailing underscore; `data_` becomes an owned `Vec<T>`
//!   rather than a raw `T*`.
//! - field `num_cells_total_` — not stored: this port reads `data.len()`
//!   wherever upstream would read `num_cells_total_`.
//! - field `data_ptrs_` (`T*** data_ptrs_`) — unported, and dead in the
//!   *upstream* source, not merely unneeded in Rust: `rg -n "data_ptrs_"`
//!   against the full `moveit_core/distance_field` tree returns only its
//!   own declaration line — never initialized, assigned, or read anywhere,
//!   including by the constructor/destructor that own `data_`. Nothing to
//!   port.
//!
//! ## `distance_field.hpp`
//!
//! - `PlaneVisualizationType` (enum) — D-decision excludes it: D1 (used
//!   only by `getPlaneMarkers`, see the marker methods below).
//! - `DistanceField` constructor / destructor — N/A: see [`DistanceField`]'s
//!   own doc comment, "The rest is not ported", last bullet.
//! - `addPointsToField`/`removePointsFromField`/`updatePointsInField`/
//!   `reset` (pure virtual) — ported as the required trait methods
//!   [`DistanceField::add_points_to_field`]/
//!   [`DistanceField::remove_points_from_field`]/
//!   [`DistanceField::update_points_in_field`]/[`DistanceField::reset`].
//! - `getShapePoints`/`addShapeToField`/`moveShapeInField`/
//!   `removeShapeFromField` — ported as
//!   [`DistanceField::add_shape_to_field`]/
//!   [`DistanceField::move_shape_in_field`]/
//!   [`DistanceField::remove_shape_from_field`] (`getShapePoints` has no
//!   separate Rust name — its one job, converting a shape+pose into
//!   obstacle points, is inlined as this crate's private `posed_body` plus
//!   [`find_internal_points_convex`], called from all three).
//! - `addOcTreeToField` — unported; see [`DistanceField`]'s own doc.
//! - `getOcTreePoints` (protected) — unported: upstream's only caller is
//!   `addOcTreeToField`, itself unported above.
//! - `getDistance(double,double,double)`/`getDistance(int,int,int)`/
//!   `isCellValid`/`getXNumCells`/`getYNumCells`/`getZNumCells`/
//!   `gridToWorld`/`worldToGrid`/`getUninitializedDistance` (pure virtual)
//!   — ported as the required trait methods [`DistanceField::distance`]/
//!   [`DistanceField::distance_cell`]/[`DistanceField::is_cell_valid`]/
//!   [`DistanceField::num_cells_x`]/[`DistanceField::num_cells_y`]/
//!   [`DistanceField::num_cells_z`]/[`DistanceField::grid_to_world`]/
//!   [`DistanceField::world_to_grid`]/
//!   [`DistanceField::uninitialized_distance`].
//! - `getDistanceGradient` — ported as the default trait method
//!   [`DistanceField::distance_gradient`]; see that method's own doc for the
//!   `inv_twice_resolution_` truncation bug this port cannot reintroduce,
//!   and [`PosedDistanceField::get_collision_sphere_gradients`]'s
//!   "Decision" doc section for the downstream consequence of this method's
//!   zero-on-out-of-bounds behaviour.
//! - `writeToStream`/`readFromStream` — unported; see [`DistanceField`]'s
//!   own doc.
//! - `getIsoSurfaceMarkers`/`getGradientMarkers`/`getPlaneMarkers`/
//!   `getProjectionPlanes` — D-decision excludes them: D1.
//! - `setPoint` (protected) — unported: upstream's only caller is
//!   `getProjectionPlanes`, itself excluded above.
//! - `getSizeX`/`getSizeY`/`getSizeZ`/`getOriginX`/`getOriginY`/
//!   `getOriginZ`/`getResolution` (inline getters) — ported as the required
//!   trait methods [`DistanceField::size_x`]/[`DistanceField::size_y`]/
//!   [`DistanceField::size_z`]/[`DistanceField::origin_x`]/
//!   [`DistanceField::origin_y`]/[`DistanceField::origin_z`]/
//!   [`DistanceField::resolution`].
//! - fields `size_x_`/`size_y_`/`size_z_`/`origin_x_`/`origin_y_`/
//!   `origin_z_`/`resolution_` — ported as the state each implementer
//!   ([`PropagationDistanceField`]) stores in its own [`VoxelGrid`], read
//!   back through the getter methods above rather than duplicated on the
//!   trait.
//! - field `inv_twice_resolution_` (mistyped `int`, silently truncating) —
//!   unported as a stored field; see [`DistanceField::distance_gradient`]'s
//!   own doc for why this port recomputes it from `resolution()` instead of
//!   caching the upstream bug.
//!
//! ## `propagation_distance_field.hpp`
//!
//! - `CompareEigenVector3i` (struct) — unported: a `std::set` ordering
//!   comparator for cache locality only, not correctness — see
//!   [`PropagationDistanceField::update_points_in_field`]'s own "Deviation
//!   from upstream" doc for why a plain `(x, y, z)`-ordered `BTreeSet`
//!   replaces it.
//! - `PropDistanceFieldVoxel` (struct, 2 constructors) — ported as
//!   [`PropDistanceFieldVoxel`]; the field-uninitialized default
//!   constructor has no caller (see that type's own "Deviation from
//!   upstream" doc) so only the 2-argument constructor is ported, as
//!   [`PropDistanceFieldVoxel::new`].
//! - `PropagationDistanceField(size_x, ..., propagate_negative_distances)`
//!   (primary constructor) — ported as [`PropagationDistanceField::new`].
//! - `PropagationDistanceField(octree, bbx_min, bbx_max, ...)` /
//!   `PropagationDistanceField(istream&, ...)` — unported; see
//!   [`PropagationDistanceField`]'s own "Deviations from upstream" doc,
//!   first bullet.
//! - `~PropagationDistanceField()` (empty override) — N/A, same as the base
//!   class destructor above.
//! - `addPointsToField`/`removePointsFromField`/`updatePointsInField`/
//!   `reset`/`getDistance` (×2)/`isCellValid`/`getXNumCells`/
//!   `getYNumCells`/`getZNumCells`/`gridToWorld`/`worldToGrid`/
//!   `getUninitializedDistance` (overrides) — ported as this type's
//!   [`DistanceField`] impl.
//! - `writeToStream`/`readFromStream` (overrides) — unported; same reason
//!   as the base class.
//! - `getCell` — ported as [`PropagationDistanceField::cell`].
//! - `getNearestCell` — ported as [`PropagationDistanceField::nearest_cell`];
//!   see that method's own doc comment for a real upstream defect
//!   (undefined-behaviour-in-practice pointer aliasing) this port closes
//!   rather than reproduces.
//! - `getMaximumDistanceSquared` — ported as
//!   [`PropagationDistanceField::max_distance_squared`].
//! - private `initialize` — ported inline in
//!   [`PropagationDistanceField::new`] (no separate method; nothing else
//!   calls it, matching upstream, where every constructor also calls it
//!   exactly once).
//! - private `addNewObstacleVoxels`/`removeObstacleVoxels`/
//!   `propagatePositive`/`propagateNegative` — ported as the private
//!   `add_new_obstacle_voxels`/`remove_obstacle_voxels`/
//!   `propagate_positive`/`propagate_negative` methods.
//! - private `getDistance(const PropDistanceFieldVoxel&)` — ported as the
//!   private `distance_of`.
//! - private `getDirectionNumber`/`getLocationDifference`/
//!   `initNeighborhoods` — ported as the free functions
//!   `direction_number`/`build_neighborhoods` (the latter two merged:
//!   `build_neighborhoods` returns both the neighborhoods table and the
//!   direction-number-to-direction lookup `getLocationDifference` reads
//!   from, since nothing else needs them built separately).
//! - private `print` (×2, debug-only `RCLCPP_DEBUG` dumps) — unported;
//!   `PORTING-PLAN.md` D1's ROS-independence applies to logging macros the
//!   same as message types, and nothing in the ported test suite exercises
//!   these debug-only paths.
//! - field `propagate_negative_` — ported as the private `propagate_negative`
//!   field. **Genuinely threaded, not merely fixture-pinned**: see
//!   [`PropagationDistanceField::new`]'s own doc comment for the `rg`
//!   evidence that it gates the same call sites upstream's
//!   `propagate_negative_` does, line for line.
//! - field `voxel_grid_` — ported as the private `voxel_grid` field
//!   (`VoxelGrid<PropDistanceFieldVoxel>` owned directly rather than behind
//!   a `shared_ptr`).
//! - fields `bucket_queue_`/`negative_bucket_queue_` — ported as the
//!   private `bucket_queue`/`negative_bucket_queue` fields.
//! - fields `max_distance_`/`max_distance_sq_` — ported as the private
//!   `max_distance`/`max_distance_sq` fields.
//! - field `sqrt_table_` — ported as the private `sqrt_table` field.
//! - field `neighborhoods_` — ported as the private `neighborhoods` field.
//! - field `direction_number_to_direction_` — ported as the private
//!   `direction_number_to_direction` field.
//! - `VoxelSet` (private typedef) — unported as a named type: this port's
//!   `BTreeSet<(i32, i32, i32)>` needs no comparator typedef (see
//!   `CompareEigenVector3i` above).
//!
//! **There is no Euclidean-vs-Manhattan (or any other) distance-metric mode
//! to port.** All three constructors above take exactly one boolean mode
//! parameter, `propagate_negative_distances`; `rg -n
//! "PropagationDistanceField\("
//! moveit_core/distance_field/include/moveit/distance_field/propagation_distance_field.hpp`
//! against the pinned tree shows this for every overload, and `rg -ni
//! "manhattan|chebyshev|euclidean" moveit_core/distance_field/` returns zero
//! hits anywhere under the package. `PropagationDistanceField` is also the
//! package's only subclass of `DistanceField` (`rg -n "public DistanceField"
//! moveit_core/distance_field/` returns exactly this one hit), so there is
//! no second class carrying an alternate metric either. The wavefront
//! propagation itself (`propagatePositive`/`propagateNegative`, 26-connected
//! `neighborhoods_`, squared-distance bucket ordering) is single and
//! non-switchable on both sides of this port.
//!
//! ## `find_internal_points.hpp`
//!
//! - `findInternalPointsConvex` — ported as [`find_internal_points_convex`],
//!   generic over [`ConvexBody`] rather than upstream's concrete
//!   `bodies::Body` — see that trait's own doc comment for the narrowed
//!   dependency and why.

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
