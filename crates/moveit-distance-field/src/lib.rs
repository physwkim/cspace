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
//! cache-entry-construction, per-group-state, and collision-checking
//! machinery: `collision_distance_field_types` (no `RobotModel`
//! dependency), `collision_common_distance_field`'s `RobotState`/
//! `RobotModel`-dependent half plus its [`DistanceFieldCacheEntry`]/
//! [`GroupStateRepresentation`] structs, and `collision_env_distance_field`'s
//! construction/query/checking slice (`addLinkBodyDecompositions`,
//! `generateDistanceFieldCacheEntry`, `getDistanceFieldCacheEntry`,
//! `getGroupStateRepresentation`, `updateGroupStateRepresentationState`,
//! its persistent cache-owner role -- `generateCollisionCheckingStructures`
//! -- and, as of round 21, `checkSelfCollision`/`checkCollision`/
//! `checkRobotCollision`/`getCollisionGradients`/`getAllCollisions` as
//! [`DistanceFieldCollisionCache`] methods). What remains out of scope is
//! narrower than "the collision checker": `CollisionEnvDistanceField` as a
//! `World`-observing type (its constructors, `initialize`, `setWorld`,
//! `notifyObjectChange`, `updateDistanceObject`,
//! `generateDistanceFieldCacheEntryWorld`), the two continuous-state
//! `checkRobotCollision` overloads (upstream itself stubs both to "not
//! implemented"), the header-inline `distanceSelf`/`distanceRobot` stubs,
//! and `createCollisionModelMarker` (out of scope under D1). See
//! `PORTING-PLAN.md` §3 and `collision_env_distance_field.rs`'s own module
//! doc comment (its "Round 21" section carries the full difference table
//! against upstream's seven `check*`/`getCollisionGradients`/
//! `getAllCollisions` call sites) for specifics.
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
//!   `test_distance_field.cpp` gtest suite carries, minus `TestOcTree` (needs
//!   two other still-unported items — the octree-and-bounding-box-taking
//!   `PropagationDistanceField` constructor overload and
//!   `addShapeToField`'s `shapes::OCTREE` dispatch, not
//!   [`DistanceField::add_octree_to_field`] itself, which is now ported) and
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
//! itself is no longer entirely unported: round 21 ports its five real
//! (non-continuous) collision-checking entry points --
//! `checkSelfCollision`/`checkCollision`/`checkRobotCollision`/
//! `getCollisionGradients`/`getAllCollisions` -- as
//! [`DistanceFieldCollisionCache`] methods (see the `collision_env_distance_field.hpp`
//! symbol-audit section below for the full accounting). What remains
//! unported is narrower: the class's construction/`World`-observation surface
//! (3 constructors, `initialize`, `setWorld`, `getDistanceField`,
//! `notifyObjectChange`, `updateDistanceObject`,
//! `generateDistanceFieldCacheEntryWorld`, the nested
//! `DistanceFieldCacheEntryWorld` struct), `checkRobotCollision`'s 2
//! continuous-state overloads (upstream itself stubs both to "not
//! implemented"), the header-inline `distanceSelf`/`distanceRobot` stubs
//! (never reach the `gsr`-cache machinery at all), and
//! `createCollisionModelMarker` (D1) -- see
//! `collision_env_distance_field.rs`'s module doc, "Round 21" section, for
//! the difference table against all seven upstream call sites this round
//! reads and ports individually. `CollisionEnvHybrid` and both
//! `CollisionDetectorAllocator*` classes are D-decision-excluded (D1's
//! FCL/Bullet replacement, D4's compile-time-trait plugin model). The
//! `AttachedBody`-dependent decomposition functions
//! (`getAttachedBodySphereDecomposition`/`getAttachedBodyPointDecomposition`)
//! are now ported too, as of round 22, as
//! [`attached_body_sphere_decomposition`]/[`attached_body_point_decomposition`]
//! — the premise this bullet used to state (a bare `moveit_state::State`
//! structurally cannot see attached bodies, since they live on
//! `moveit_scene::PlanningScene`, which this crate does not depend on) is
//! still true, but it turned out irrelevant: this crate had already solved
//! the same problem a different way for a different symbol
//! ([`AttachedBodySnapshot`]'s explicit `AttachedBodyGeometry` parameter,
//! threaded through every collision-checking entry point below), and round
//! 22 extended that existing pattern to these two decomposition functions
//! rather than adding a new dependency. Their sole upstream callers —
//! `CollisionEnvDistanceField::getGroupStateRepresentation`
//! (`collision_env_distance_field.cpp:1239`) and
//! `generateDistanceFieldCacheEntry`'s non-group-link loop (`:928`) — are
//! [`group_state_representation`] and the private `build_non_group_distance_field`
//! respectively, both already ported and both now threading
//! `attached_bodies: &[AttachedBodyGeometry<'_>]` through to these two
//! functions. See `collision_common_distance_field.rs`'s own "Round 22"
//! doc section for the full account. The octree-backed
//! `PosedBodyPointDecomposition` constructor is now
//! ported, as [`PosedBodyPointDecomposition::from_octree`], against a
//! `moveit-octomap` dependency this crate added for it — see that method's
//! own doc comment for the faithfully-reproduced upstream behaviour
//! (every tree node, not just occupied leaves). Nothing above depends on
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
//! **Argued-only-claim sweep (round 19): 0 found, with the anchor and
//! method so "0" does not have to be re-derived from scratch next round.**
//! Anchor: every `.rs` file under `crates/moveit-distance-field/`
//! (`src/*.rs` and `tests/*.rs`), grepped case-insensitively for hedge and
//! open-question language in both English and Korean —
//! `unverified|not verified|presumably|likely|assumed|assume[ds]?|no
//! evidence|open (item|question|gap)|not yet
//! (measured|verified|checked)|in principle|theoretically|by
//! inspection|hand-wave|without (measuring|verifying|checking)|no dedicated
//! (test|fixture)|no test (covers|exercises|checks)|not directly
//! (tested|verified|checked)|believed|expect(ed)? to|probably|추정|짐작` —
//! plus a second pass for `TODO|FIXME|remains open|still open|cross-language
//! question|not (yet )?(pinned|covered|tested|confirmed|closed)|left
//! unported|open gap`. Every hit resolved to one of three outcomes: already
//! measured and cross-referenced (e.g. `octree_points`'s own former "open
//! cross-language question", closed by the `octree_points` oracle op),
//! proven unreachable by construction (e.g.
//! `collision_sphere_free_functions_parity.rs`'s out-of-bounds early-return
//! guard — no fixture can trigger it, and the doc proves why, not just
//! asserts it), or a deliberately-scoped-out deviation with a named reason
//! (e.g. the unported `writeToStream`/`readFromStream` round trip).
//! None were open measurement claims. The one item that did not resolve
//! this way — `LeavesInBbx`'s unread fields and unpinned cross-leaf order —
//! is not a claim in this crate's own doc at all; it is a gap in a
//! *different* crate's primitive that this crate's own doc now names
//! precisely (see `octree_points`'s own doc comment) because no fixture
//! here could ever close it.
//!
//! **Numeric coverage audit (PORTING-PLAN.md §90.3).** Derived by re-reading
//! both upstream headers directly and counting every `public:`-section
//! member, not by trusting the classification bullets above at face value —
//! this is the independent count that checks they add up.
//!
//! **Counting criteria (PORTING-PLAN.md §96.5), stated so a re-count can be
//! checked against the same rule rather than re-derived from scratch:**
//! constructors and the destructor are *not* counted (upstream's own
//! per-class "N/A" bullets above call those out separately, and a
//! move/copy-constructor-free C++ class has at most one real constructor
//! shape to port regardless of how many overloads it takes); every
//! **inline-body accessor counts as one method**, `{ return size_x_; }`
//! one-liners included — this is the category a naive brace-depth counter
//! silently drops once it stops tracking after the first inline body, which
//! is exactly the discrepancy between this section's 32 and an independent
//! recount landing on 27 (`getSizeY`/`getSizeZ`/`getOriginX`/... lost);
//! **overloads count separately, one per distinct signature**, never merged
//! into their base name — `getDistance(double,double,double)` and
//! `getDistance(int,int,int)` are 2 of the 32, matching this port's 1
//! upstream-signature-to-1-Rust-name mapping (`distance`/`distance_cell`),
//! and the same rule gives `collision_distance_field_types.hpp`'s
//! `getCollisionSphereCollision` 2 overloads as 2 of its 5 free functions
//! (`get_collision_sphere_collision`/`get_collision_sphere_collisions`);
//! there are **no `virtual` re-declarations to reconcile** in either header
//! counted here — `distance_field.hpp`'s pure-virtual methods are declared
//! once, and `PropagationDistanceField`'s overrides live in
//! `propagation_distance_field.hpp`, a different header handled in its own
//! (non-numeric) symbol-audit section below, not double-counted against
//! this one.
//!
//! `distance_field.hpp` (`DistanceField`, abstract base): 32 public methods
//! — 26 ported, 2 unported, 4 D-excluded. 2 protected methods — 1 ported, 1
//! unported. 8 protected fields — 7 ported as implementer state, 1
//! deliberately unported. One bullet per name below, so a re-count can diff
//! against this list directly rather than this section's prose summary of
//! it — `awk '/## `distance_field.hpp` \(by name\)/,/## `collision_distance_field_types.hpp` \(by name\)/'
//! crates/moveit-distance-field/src/lib.rs | rg -c '^//! - '` gives **42**
//! (= 32 + 2 + 8), heading-anchored so the command stays correct across
//! future edits instead of hardcoding line numbers that would drift:
//! ## `distance_field.hpp` (by name)
//!
//! - `addPointsToField` — ported, [`DistanceField::add_points_to_field`].
//! - `removePointsFromField` — ported, [`DistanceField::remove_points_from_field`].
//! - `updatePointsInField` — ported, [`DistanceField::update_points_in_field`].
//! - `getShapePoints` — ported, inlined into the private `posed_body` plus
//!   [`find_internal_points_convex`] (no separate Rust name).
//! - `addShapeToField` — ported, [`DistanceField::add_shape_to_field`].
//! - `addOcTreeToField` — ported, [`DistanceField::add_octree_to_field`].
//! - `moveShapeInField` — ported, [`DistanceField::move_shape_in_field`].
//! - `removeShapeFromField` — ported, [`DistanceField::remove_shape_from_field`].
//! - `reset` — ported, [`DistanceField::reset`].
//! - `getDistance(double,double,double)` — ported, [`DistanceField::distance`].
//! - `getDistanceGradient` — ported, [`DistanceField::distance_gradient`].
//! - `getDistance(int,int,int)` — ported, [`DistanceField::distance_cell`].
//! - `isCellValid` — ported, [`DistanceField::is_cell_valid`].
//! - `getXNumCells` — ported, [`DistanceField::num_cells_x`].
//! - `getYNumCells` — ported, [`DistanceField::num_cells_y`].
//! - `getZNumCells` — ported, [`DistanceField::num_cells_z`].
//! - `gridToWorld` — ported, [`DistanceField::grid_to_world`].
//! - `worldToGrid` — ported, [`DistanceField::world_to_grid`].
//! - `writeToStream` — unported.
//! - `readFromStream` — unported.
//! - `getIsoSurfaceMarkers` — D1-excluded.
//! - `getGradientMarkers` — D1-excluded.
//! - `getPlaneMarkers` — D1-excluded.
//! - `getProjectionPlanes` — D1-excluded.
//! - `getSizeX` — ported, [`DistanceField::size_x`].
//! - `getSizeY` — ported, [`DistanceField::size_y`].
//! - `getSizeZ` — ported, [`DistanceField::size_z`].
//! - `getOriginX` — ported, [`DistanceField::origin_x`].
//! - `getOriginY` — ported, [`DistanceField::origin_y`].
//! - `getOriginZ` — ported, [`DistanceField::origin_z`].
//! - `getResolution` — ported, [`DistanceField::resolution`].
//! - `getUninitializedDistance` — ported, [`DistanceField::uninitialized_distance`].
//! - *(protected)* `getOcTreePoints` — ported, the private free function
//!   `octree_points`.
//! - *(protected)* `setPoint` — unported.
//! - *(protected field)* `size_x_` — ported, implementer state.
//! - *(protected field)* `size_y_` — ported, implementer state.
//! - *(protected field)* `size_z_` — ported, implementer state.
//! - *(protected field)* `origin_x_` — ported, implementer state.
//! - *(protected field)* `origin_y_` — ported, implementer state.
//! - *(protected field)* `origin_z_` — ported, implementer state.
//! - *(protected field)* `resolution_` — ported, implementer state.
//! - *(protected field)* `inv_twice_resolution_` — deliberately unported;
//!   see [`DistanceField::distance_gradient`]'s own doc.
//!
//! `collision_distance_field_types.hpp` (11 top-level types, `grep -n
//! "^class \|^struct \|^enum "` against the pinned tree): under the same
//! ctor-excluded criteria as above, this header has **34** public methods,
//! not the 44 an earlier round of this section reported. That 44 was wrong
//! two ways, not one: it folded the 11 constructors into a "members"
//! headline the criteria above says not to count, *and*, independently of
//! that, its own per-type tally undercounted `BodyDecomposition` at 7
//! methods when the class has 8 (`replaceCollisionSpheres`/
//! `getCollisionSpheres`/`getSphereRadii`/`getCollisionPoints`/`getBody`/
//! `getBodiesCount`/`getRelativeCylinderPose`/`getRelativeBoundingSphere`,
//! re-read directly from `collision_distance_field_types.hpp:228-299`).
//! Restated consistently with `distance_field.hpp`'s own count above, which
//! never counted `DistanceField`'s constructor either: **34 methods + 11
//! constructors = 45** total ported symbols across the 8 types that have
//! either, plus 5 free functions and 3 D-excluded free functions at
//! namespace scope. `CollisionType` (enum, 0 members), `ProximityInfo`
//! (fields only, 0 methods) and `BodyDecompositionVector`
//! (forward-declared at line 226, never defined in this header — 0 members
//! to port, confirmed by reading the full 536-line file) contribute
//! nothing to either count. All 34 methods and all 11 constructors are
//! ported — cross-checked name-for-name against
//! `collision_distance_field_types.rs`'s `pub fn` list (`grep -n
//! "pub fn "`). One bullet per name below; `awk '/## `collision_distance_field_types.hpp`
//! \(by name\)/,/## The `max_relative` trap/'
//! crates/moveit-distance-field/src/lib.rs | rg -c '^//! - '` gives **53**
//! (= 34 + 11 + 5 + 3), heading-anchored for the same reason as above:
//!
//! ## `collision_distance_field_types.hpp` (by name)
//!
//! - `GradientInfo::clear` — ported, [`GradientInfo::clear`].
//! - `PosedDistanceField::updatePose` — ported, [`PosedDistanceField::update_pose`].
//! - `PosedDistanceField::getPose` — ported, [`PosedDistanceField::pose`].
//! - `PosedDistanceField::getDistanceGradient` — ported, [`PosedDistanceField::distance_gradient`].
//! - `PosedDistanceField::getCollisionSphereGradients` (member overload) —
//!   ported, [`PosedDistanceField::get_collision_sphere_gradients`].
//! - `BodyDecomposition::replaceCollisionSpheres` — ported, [`BodyDecomposition::replace_collision_spheres`].
//! - `BodyDecomposition::getCollisionSpheres` — ported, [`BodyDecomposition::collision_spheres`].
//! - `BodyDecomposition::getSphereRadii` — ported, [`BodyDecomposition::sphere_radii`].
//! - `BodyDecomposition::getCollisionPoints` — ported, [`BodyDecomposition::collision_points`].
//! - `BodyDecomposition::getBody` — ported, [`BodyDecomposition::body`].
//! - `BodyDecomposition::getBodiesCount` — ported, [`BodyDecomposition::bodies_count`].
//! - `BodyDecomposition::getRelativeCylinderPose` — ported, [`BodyDecomposition::relative_cylinder_pose`].
//! - `BodyDecomposition::getRelativeBoundingSphere` — ported, [`BodyDecomposition::relative_bounding_sphere`].
//! - `PosedBodySphereDecomposition::getCollisionSpheres` — ported, [`PosedBodySphereDecomposition::collision_spheres`].
//! - `PosedBodySphereDecomposition::getSphereCenters` — ported, [`PosedBodySphereDecomposition::sphere_centers`].
//! - `PosedBodySphereDecomposition::getCollisionPoints` — ported, [`PosedBodySphereDecomposition::collision_points`].
//! - `PosedBodySphereDecomposition::getSphereRadii` — ported, [`PosedBodySphereDecomposition::sphere_radii`].
//! - `PosedBodySphereDecomposition::getBoundingSphereCenter` — ported, [`PosedBodySphereDecomposition::bounding_sphere_center`].
//! - `PosedBodySphereDecomposition::getBoundingSphereRadius` — ported, [`PosedBodySphereDecomposition::bounding_sphere_radius`].
//! - `PosedBodySphereDecomposition::updatePose` — ported, [`PosedBodySphereDecomposition::update_pose`].
//! - `PosedBodyPointDecomposition::getCollisionPoints` — ported, [`PosedBodyPointDecomposition::collision_points`].
//! - `PosedBodyPointDecomposition::updatePose` — ported, [`PosedBodyPointDecomposition::update_pose`].
//! - `PosedBodySphereDecompositionVector::getCollisionSpheres` — ported, [`PosedBodySphereDecompositionVector::collision_spheres`].
//! - `PosedBodySphereDecompositionVector::getSphereCenters` — ported, [`PosedBodySphereDecompositionVector::sphere_centers`].
//! - `PosedBodySphereDecompositionVector::getSphereRadii` — ported, [`PosedBodySphereDecompositionVector::sphere_radii`].
//! - `PosedBodySphereDecompositionVector::addToVector` — ported, [`PosedBodySphereDecompositionVector::add_to_vector`].
//! - `PosedBodySphereDecompositionVector::getSize` — ported, [`PosedBodySphereDecompositionVector::len`]
//!   (plus a Rust-idiom `is_empty`, not an upstream symbol).
//! - `PosedBodySphereDecompositionVector::getPosedBodySphereDecomposition` — ported, [`PosedBodySphereDecompositionVector::get`].
//! - `PosedBodySphereDecompositionVector::updatePose` — ported, [`PosedBodySphereDecompositionVector::update_pose`].
//! - `PosedBodyPointDecompositionVector::getCollisionPoints` — ported, [`PosedBodyPointDecompositionVector::collision_points`].
//! - `PosedBodyPointDecompositionVector::addToVector` — ported, [`PosedBodyPointDecompositionVector::add_to_vector`].
//! - `PosedBodyPointDecompositionVector::getSize` — ported, [`PosedBodyPointDecompositionVector::len`]
//!   (plus a Rust-idiom `is_empty`, not an upstream symbol).
//! - `PosedBodyPointDecompositionVector::getPosedBodyDecomposition` — ported, [`PosedBodyPointDecompositionVector::get`].
//! - `PosedBodyPointDecompositionVector::updatePose` — ported, [`PosedBodyPointDecompositionVector::update_pose`].
//! - *(constructor)* `CollisionSphere(rel, radius)` — ported, [`CollisionSphere::new`].
//! - *(constructor)* `GradientInfo()` — ported, `GradientInfo`'s `Default` impl.
//! - *(constructor)* `PosedDistanceField(size, origin, resolution, max_distance, propagate_negative_distances)`
//!   — ported, [`PosedDistanceField::new`].
//! - *(constructor)* `BodyDecomposition(shape, resolution, padding)` — ported, [`BodyDecomposition::new`].
//! - *(constructor)* `BodyDecomposition(shapes, poses, resolution, padding)` — ported, [`BodyDecomposition::from_shapes`].
//! - *(constructor)* `PosedBodySphereDecomposition(body_decomposition)` — ported, [`PosedBodySphereDecomposition::new`].
//! - *(constructor)* `PosedBodyPointDecomposition(body_decomposition)` — ported, [`PosedBodyPointDecomposition::new`].
//! - *(constructor)* `PosedBodyPointDecomposition(body_decomposition, pose)` — ported, [`PosedBodyPointDecomposition::with_pose`].
//! - *(constructor)* `PosedBodyPointDecomposition(octree)` — ported, [`PosedBodyPointDecomposition::from_octree`].
//! - *(constructor)* `PosedBodySphereDecompositionVector()` — ported, [`PosedBodySphereDecompositionVector::new`].
//! - *(constructor)* `PosedBodyPointDecompositionVector()` — ported, [`PosedBodyPointDecompositionVector::new`].
//! - *(free function)* `determineCollisionSpheres` — ported, [`determine_collision_spheres`].
//! - *(free function)* `getCollisionSphereGradients` — ported, [`get_collision_sphere_gradients`].
//! - *(free function)* `getCollisionSphereCollision` (maximum_value/tolerance overload)
//!   — ported, [`get_collision_sphere_collision`].
//! - *(free function)* `getCollisionSphereCollision` (num_coll/colls overload)
//!   — ported, [`get_collision_sphere_collisions`].
//! - *(free function)* `doBoundingSpheresIntersect` — ported, [`do_bounding_spheres_intersect`].
//! - *(free function)* `getCollisionSphereMarkers` — D1-excluded.
//! - *(free function)* `getProximityGradientMarkers` — D1-excluded.
//! - *(free function)* `getCollisionMarkers` — D1-excluded.
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
//! (see the "Exactness" section of `oracle_parity.rs`'s own module doc).
//!
//! **Bisect by constant, not by file (PORTING-PLAN.md §85.3).** A call site
//! is not automatically defective for lacking `max_relative` — but checking
//! that by lowering every `assert_relative_eq!` in a file to `epsilon = 0.0`
//! as one group is unreliable in the *other* direction: one still-passing
//! real gate is indistinguishable, at the group level, from "nothing in this
//! group is trap-caught". `upstream_parity.rs`'s own first pass made exactly
//! this mistake — bisecting its 7 calls together showed a pass-then-fail at
//! `0.0`, read as "the named epsilons dominate", when 4 of the 7 were in
//! fact trap-caught and only the remaining 3 (a different named constant)
//! were the real gate (see that file's own module doc for the corrected
//! measurement). Bisect per constant, or per constant-group sharing one
//! name, not per file — and when a lowered group fails, confirm by name
//! which assertion actually tripped, since a file can mix a real gate with
//! trapped ones behind the same `epsilon = 0.0` run.
//!
//! **Recounted (round 21), with the same reproducible command
//! `moveit-geometry`'s audit script now uses**:
//!
//! ```text
//! perl tools/ci/count-relative-eq.pl \
//!   $(find crates/moveit-distance-field -name '*.rs' | sort)
//! both=45 epsilon_only=3 max_relative_only=0 neither=0
//! ```
//!
//! `both` moved twice since the round 19 count (`27`), for two unrelated
//! reasons, neither a real behavior change: first to `44`, a pure
//! measurement fix — `count-relative-eq.pl` mishandled a Rust
//! line-continuation string literal (`"...\` + newline) elsewhere in the
//! workspace, undercounting real calls that happen to follow one; the fix
//! (`c9780c7`) changed the *script*, not this crate, and the corrected
//! count already reflected this file's state before round 21 started. Then
//! to `45`, this round's own addition:
//! `check_self_collision_reuses_the_distance_field_cache_entry_fixture` in
//! `collision_env_distance_field_parity.rs` adds one more
//! `assert_relative_eq!(epsilon = TOL, max_relative = TOL)` call, the same
//! shape as every other `both` site in that file. `epsilon_only=3` and
//! `neither=0` are unchanged by both the fix and this round's addition.
//!
//! The 3 `epsilon_only` sites are `upstream_parity.rs`'s three
//! `epsilon = RESOLUTION` comparisons — already bisected and shown immune
//! by construction (each compares against an exact-zero component, where
//! `max_relative`'s implicit term reduces to `|a| <= max_relative * |a|`,
//! false for any `max_relative < 1`), see that file's own module doc, "The
//! 3 `epsilon = RESOLUTION` sites are a real gate". Nothing to dispose of
//! this round for that group either — recount confirms the prior disposal,
//! it does not change it.
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
//! - `collision_env_hybrid.hpp`/`.cpp` (`CollisionEnvHybrid`) — **not
//!   excluded.** Ported as [`HybridCollisionEnv`] (round 29); see that
//!   type's own module doc for the shape and the §186 measurement that
//!   found the previous "extends `CollisionEnvFCL`, so unportable"
//!   exclusion false (the third exclusion this session justified by a
//!   relationship rather than a call count, after §139 and
//!   `plan_components_builder`).
//! - `collision_detector_allocator_distance_field.hpp`
//!   (`CollisionDetectorAllocatorDistanceField`) and
//!   `collision_detector_allocator_hybrid.hpp`
//!   (`CollisionDetectorAllocatorHybrid`) — both
//!   `CollisionDetectorAllocatorTemplate<...>` ROS-pluginlib-style runtime
//!   plugin registrations. D-decision: D4 (this port's plugin model is a
//!   compile-time trait + `linkme` registry, not a runtime allocator
//!   class) -- independent of whether either type's own `CollisionEnv*` is
//!   ported (`CollisionDetectorAllocatorDistanceField`'s
//!   `CollisionEnvDistanceField` is not ported as a `World`-owning type at
//!   all, see this module's doc on that; `CollisionDetectorAllocatorHybrid`'s
//!   `CollisionEnvHybrid` is ported, as [`HybridCollisionEnv`] above,
//!   but the pluginlib-style allocator wrapping it still is not, for this
//!   D4 reason alone).
//!
//! ## `collision_common_distance_field.hpp`
//!
//! - `GroupStateRepresentation` (struct) — ported as [`GroupStateRepresentation`].
//! - `DistanceFieldCacheEntry` (struct) — ported as [`DistanceFieldCacheEntry`].
//! - `getBodyDecompositionCacheEntry` — ported as [`get_body_decomposition_cache_entry`].
//! - `getCollisionObjectPointDecomposition` — ported as [`collision_object_point_decomposition`].
//! - `getAttachedBodySphereDecomposition` — ported as
//!   [`attached_body_sphere_decomposition`] (round 22). Its sole upstream
//!   caller, `getGroupStateRepresentation` (`collision_env_distance_field.cpp:1239`),
//!   is [`group_state_representation`], threading an explicit
//!   `attached_bodies: &[AttachedBodyGeometry<'_>]` parameter rather than
//!   reading attached bodies off a `RobotState` this crate does not carry
//!   them on — see `collision_common_distance_field.rs`'s "Round 22" doc
//!   section.
//! - `getAttachedBodyPointDecomposition` — ported as
//!   [`attached_body_point_decomposition`] (round 22), same reasoning. Its
//!   sole upstream caller, `generateDistanceFieldCacheEntry`'s non-group-link
//!   loop (`collision_env_distance_field.cpp:928`), is the private
//!   `build_non_group_distance_field`.
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
//!   for all 3: `PosedBodyPointDecomposition(body_decomposition)`/
//!   `PosedBodyPointDecomposition(body_decomposition, pose)`/
//!   `PosedBodyPointDecomposition(const std::shared_ptr<const
//!   octomap::OcTree>&)` as [`PosedBodyPointDecomposition::new`]/
//!   [`PosedBodyPointDecomposition::with_pose`]/
//!   [`PosedBodyPointDecomposition::from_octree`] — the last against a
//!   `moveit-octomap` dependency added for it; see that method's own doc
//!   comment for the faithfully-reproduced upstream behaviour.
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
//! `DEFAULT_RESOLUTION`, `DEFAULT_MAX_PROPOGATION_DISTANCE` — unported: every
//! one is a default constructor argument of `CollisionEnvDistanceField`
//! itself (unported, below), a type this crate has no constructor for.
//! `DEFAULT_COLLISION_TOLERANCE` differs as of round 21: its value now backs
//! `collision_tolerance`, [`DistanceFieldCollisionCache`]'s field, read by
//! [`DistanceFieldCollisionCache::check_self_collision`] and its four
//! siblings below — but it is not ported as a *named* constant either, any
//! more than the other four `DEFAULT_*` are; every
//! [`DistanceFieldCollisionCache::new`] call site supplies the value as an
//! explicit `f64` argument, the same way [`DistanceFieldConfig`]'s own
//! fields are always explicit rather than defaulted.
//!
//! `CollisionEnvDistanceField` (class) — construction, `World`-observation,
//! and continuous-collision checking remain unported; round 21 ports its
//! five real (non-continuous) collision-checking entry points as
//! [`DistanceFieldCollisionCache`] methods (below), narrowing what
//! "unported" covers from the whole class to specifically its construction/
//! `World`/continuous-checking surface. Still unported: 3 constructors,
//! `initialize`, `setWorld`, `getDistanceField`,
//! `getLastGroupStateRepresentation`, `getLastDistanceFieldEntry`, the
//! nested `DistanceFieldCacheEntryWorld` struct, the destructor,
//! `checkRobotCollision`'s 2 continuous-state overloads (upstream itself
//! stubs both to `RCLCPP_ERROR("Continuous collision checking not
//! implemented")` — see [`DistanceFieldCollisionCache::check_robot_collision`]'s
//! own doc comment), `distanceSelf` ×3/`distanceRobot` ×3 (header-inline
//! stubs that unconditionally return `0.0` or log
//! `RCLCPP_ERROR("Not implemented")`, never reaching the `gsr`-cache
//! machinery at all — see `collision_env_distance_field.rs`'s "Round 21"
//! module doc section for why no `distance*` counterpart to the five ported
//! methods below exists to port), and `createCollisionModelMarker` (D1).
//!
//! Now ported, as [`DistanceFieldCollisionCache`] methods rather than
//! methods of the still-unported class — see that module's own "Round 21"
//! doc section for the full difference table against all seven upstream
//! call sites, including why none threads a caller-owned
//! `GroupStateRepresentationPtr& gsr` the way every one of the seven does:
//!
//! - `checkSelfCollisionHelper` (all 4 `checkSelfCollision` overloads
//!   collapsed into it via `acm: Option<&AllowedCollisionMatrix>`) — ported
//!   as [`DistanceFieldCollisionCache::check_self_collision`].
//! - `checkCollision`'s 2 real (`gsr`-threading-collapsed) overloads — ported
//!   as [`DistanceFieldCollisionCache::check_collision`].
//! - `checkRobotCollision`'s 2 real (non-continuous) overloads — ported as
//!   [`DistanceFieldCollisionCache::check_robot_collision`].
//! - `getCollisionGradients` — ported as
//!   [`DistanceFieldCollisionCache::get_collision_gradients`] (its upstream
//!   `res` parameter is commented `/*res*/`, never read or written, so not
//!   ported).
//! - `getAllCollisions` — ported as
//!   [`DistanceFieldCollisionCache::get_all_collisions`].
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
//! - `getSelfCollisions` — ported (round 21) as the module-private free
//!   function `get_self_collisions` in `collision_env_distance_field.rs`,
//!   called from `check_self_collision`/`check_collision`/
//!   `get_all_collisions`. Round 22 exposed a real gap here (its loop never
//!   read `attached_body_names`/`attached_body_decompositions`), and round 23
//!   (`ebd7ebc`) closed it: the loop bound now covers
//!   `link_names.len() + attached_body_names.len()`, matching upstream's own
//!   indexing (`collision_env_distance_field.cpp:278-320`) — see that
//!   function's own "Deviation from upstream" for the full account, and
//!   `bde1dcb` for the oracle-verified attached-body fixtures.
//! - `getSelfProximityGradients` — ported (round 21) as the module-private
//!   free function `get_self_proximity_gradients`. Round 22 mistakenly
//!   grouped this with `getSelfCollisions`'s real gap; round 23's fresh read
//!   of the loop bound found this one has **no** attached-body gap at all:
//!   upstream's own loop condition here is `i < link_names_.size()`
//!   (`:359`), never extending to `attached_body_names_`, so the
//!   attached-body branch is dead code in the C++ itself and correctly
//!   omitted rather than ported unreachable.
//! - `getIntraGroupCollisions` — ported (round 21) as the module-private
//!   free function `get_intra_group_collisions`. Same real attached-body gap
//!   as `getSelfCollisions` (no link-vs-attached or attached-vs-attached pair
//!   was ever checked), closed the same way in round 23 (`ebd7ebc`).
//! - `getIntraGroupProximityGradients` — ported (round 21) as the
//!   module-private free function `get_intra_group_proximity_gradients`.
//!   Same real attached-body gap as `getIntraGroupCollisions`, closed in
//!   round 23 (`ebd7ebc`).
//! - `getEnvironmentCollisions` — ported (round 21) as the module-private
//!   free function `get_environment_collisions`, taking
//!   `env_distance_field: &dyn DistanceField` as an explicit parameter in
//!   place of reading `distance_field_cache_entry_world_->distance_field_`
//!   off the `World` this crate does not depend on. Same real attached-body
//!   gap as `getSelfCollisions`, closed in round 23 (`ebd7ebc`).
//! - `getEnvironmentProximityGradients` — ported (round 21) the same way, as
//!   the module-private free function `get_environment_proximity_gradients`.
//!   Like `getSelfProximityGradients` above, this one never had an
//!   attached-body gap: upstream's own loop bound here
//!   (`collision_env_distance_field.cpp:1649`) never advances past
//!   `link_names_.size()`, so its attached-body branch is dead code in the
//!   C++ itself. Of the six `get*`/`get*ProximityGradients` functions this
//!   section lists, these two were always faithful; the other four carried a
//!   real gap that round 23 closed — see each function's own doc comment for
//!   its exact upstream loop-bound citation.
//! - `updatedPaddingOrScaling` — unported: a no-op override of the
//!   `CollisionEnv` interface upstream itself (`collision_env_distance_field.hpp:270`:
//!   `void updatedPaddingOrScaling(...) override{};`, an empty `{}` body,
//!   round 26: corrected from a prior misquote of it as `return;`).
//! - `generateDistanceFieldCacheEntryWorld`, `updateDistanceObject`,
//!   `notifyObjectChange` — unported: specifically `World`-dependent, a
//!   dependency this crate deliberately does not take.
//!
//! Member fields:
//!
//! - `size_`, `origin_`, `use_signed_distance_field_`, `resolution_`,
//!   `max_propogation_distance_` — ported as [`DistanceFieldConfig`]'s
//!   fields (`origin_`/`size_` additionally require the center-to-corner
//!   shift documented on [`DistanceFieldConfig::geometry`]).
//! - `collision_tolerance_` — ported (round 21) as
//!   [`DistanceFieldCollisionCache`]'s `collision_tolerance` field; see the
//!   `DEFAULT_COLLISION_TOLERANCE` note above.
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
//! - `pregenerated_group_state_representation_map_` — **unported as a field
//!   (round 25 revision: still true, but *not* because it is unreachable —
//!   see below).** Populated only inside
//!   `CollisionEnvDistanceField::initialize()` (unported, checker-level,
//!   above), which eagerly builds one `DistanceFieldCacheEntry` +
//!   `GroupStateRepresentation` pair per joint model group at construction
//!   time. It is read in exactly one place, `generateDistanceFieldCacheEntry`'s
//!   `dfce->pregenerated_group_state_representation_ = it->second`, and that
//!   field is read only by `getGroupStateRepresentation`'s "already
//!   pregenerated" branch (`collision_env_distance_field.cpp:1212-1227`).
//!
//!   Earlier revisions of this note argued the field's *absence* was inert
//!   because nothing here could ever read it. PORTING-PLAN.md §154 measures
//!   that argument to have been incomplete: since `initialize()`'s
//!   fresh-build call only runs once per group, essentially *every* real
//!   upstream call site takes the pregenerated branch, not the fresh-build
//!   one — so the two branches' field-by-field differences are not moot,
//!   they are what every caller actually observes. The one field that
//!   differs, `sphere_locations` on link entries, is closed directly in
//!   [`group_state_representation`] as of round 25 (see that function's own
//!   "Deviations from upstream") — not by adding this field/map/branch, but
//!   by recognizing the pregenerated branch's extra line reads a value this
//!   port's existing fresh-build loop already has in hand at the same point
//!   in its own control flow, since both branches source the same
//!   `link_body_decomposition_vector_.at(ind)` posed to the same transform.
//!
//!   What remains genuinely unported is the *cache-reuse mechanism* itself
//!   (reusing an already-built `PosedDistanceField` per link across calls,
//!   instead of rebuilding one every call) — a performance difference with
//!   no known remaining output-correctness gap. See
//!   [`DistanceFieldCollisionCache::new`]'s doc comment for the type-level
//!   reason that mechanism is not added anyway: it would require this port's
//!   [`GroupStateRepresentation<'a, 'm>`] (which *borrows* its `dfce`, unlike
//!   upstream's `shared_ptr`) to be stored self-referentially alongside its
//!   own owned `DistanceFieldCacheEntry`, which safe Rust cannot express
//!   without pinning/unsafe or an external self-referential-struct
//!   dependency.
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
//!   **Falsifier** — the *mechanism* (not the value, already closed) becomes
//!   worth adding only if a caller-observable *behavior* gap, not merely a
//!   speed difference, is measured between this port's rebuild-every-call
//!   and upstream's cache-reuse — e.g. some field a reused, once-built
//!   decomposition would retain across calls that a fresh rebuild computes
//!   differently. None is known as of round 25; §154's own gap
//!   (`sphere_locations`) is closed without it (see above).
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
//! - `addOcTreeToField` — ported as the default trait method
//!   [`DistanceField::add_octree_to_field`]; see that method's own doc for
//!   the occupancy-filter/subdivision algorithm and its mutation-verified
//!   pins.
//! - `getOcTreePoints` (protected) — ported as the private free function
//!   `octree_points`, upstream's only caller
//!   ([`DistanceField::add_octree_to_field`]) above.
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
//!   `inv_twice_resolution_` truncation bug (round 26: reproduced
//!   bit-for-bit via an `as i32` cast rather than left to silently diverge
//!   at resolutions where it is not a no-op), and
//!   [`PosedDistanceField::get_collision_sphere_gradients`]'s "Decision" doc
//!   section for the downstream consequence of this method's
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
//!   unported as a *stored* field (this port recomputes it from
//!   `resolution()` each call instead of caching it), but its truncation is
//!   ported: see [`DistanceField::distance_gradient`]'s own doc for the
//!   round-26 fix that casts the recomputed value through `i32` to
//!   reproduce upstream's narrowing bug bit-for-bit rather than only
//!   matching it by coincidence at the two resolutions this crate's own
//!   fixtures happen to use.
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
mod collision_env_hybrid;
mod distance_field;
mod find_internal_points;
mod propagation;
mod voxel_grid;

pub use collision_common_distance_field::{
    AttachedBodySnapshot, DistanceFieldCacheEntry, GroupStateRepresentation,
    attached_body_point_decomposition, attached_body_sphere_decomposition,
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
pub use collision_env_hybrid::HybridCollisionEnv;
pub use distance_field::{DistanceField, DistanceGradient};
pub use find_internal_points::{ConvexBody, find_internal_points_convex};
pub use propagation::{NearestCell, PropDistanceFieldVoxel, PropagationDistanceField};
pub use voxel_grid::{Dimension, GridGeometry, VoxelGrid};
