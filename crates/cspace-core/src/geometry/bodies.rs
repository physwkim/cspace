// Copyright 2008 Willow Garage, Inc.
// Copyright 2013 Willow Garage, Inc.
// Copyright 2019 Bielefeld University
// Copyright 2019 Open Robotics
// Copyright 2024 Open Robotics
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from geometric_shapes 2.3.3 (tag `192801cebacc07d0e9f719576cdd1c9b36d0bc28`,
// same package/version verified in shapes.rs's provenance comment — see that
// comment for how the source tree was obtained and matched against the
// installed `ros-rolling-geometric-shapes` 2.3.3-1noble.20260113.113114
// package).
//
// Files read in full:
//   geometric_shapes/include/geometric_shapes/bodies.h
//   geometric_shapes/src/bodies.cpp
//   geometric_shapes/include/geometric_shapes/aabb.h
//   geometric_shapes/src/aabb.cpp
//   geometric_shapes/include/geometric_shapes/obb.h
//   geometric_shapes/src/obb.cpp
//   geometric_shapes/include/geometric_shapes/body_operations.h
//   geometric_shapes/src/body_operations.cpp
//     (only `mergeBoundingSpheres`, `mergeBoundingBoxes`,
//     `mergeBoundingBoxesApprox` and `computeBoundingSphere(vector)`; the
//     rest of that file converts to/from `shape_msgs`/`visualization_msgs`
//     bodies (`createBodyFromShape`'s message-facing siblings,
//     `constructShapeFromBody`, `constructMarkerFromBody`,
//     `constructBodyFromMsg`), which PORTING-PLAN.md D1 keeps out of the
//     core crates entirely)
//
// Also read, to confirm what upstream's own test coverage actually is (see
// "Test file discrepancy" below):
//   geometric_shapes/test/test_point_inclusion.cpp
//   geometric_shapes/test/test_ray_intersection.cpp
//   geometric_shapes/test/test_bounding_cylinder.cpp
//   geometric_shapes/test/test_bounding_box.cpp
//   geometric_shapes/test/test_bounding_sphere.cpp
//     (confirmed to test the already-ported, unposed
//     `computeShapeBoundingSphere` from shapes.rs — no new ground truth here)
//   geometric_shapes/test/test_body_operations.cpp
//     (confirmed entirely message/marker-based — out of scope per D1, no
//     portable ground truth here)
//
// Test file discrepancy: the task that requested this port named
// `test_bodies.cpp` as the file to port test cases from. No such file exists
// in the 2.3.3 tree — `bodies::`'s coverage is split across the five files
// above instead. This comment records that discrepancy so it is not lost.
//
// FCL provenance: `bodies::OBB` is a PIMPL wrapping FCL's
// `fcl::OBB<double>` (`obb.cpp` `#include <fcl/math/bv/OBB.h>`). FCL is a
// separate upstream package from `geometric_shapes`, but it ships as its
// own Debian package (`libfcl-dev`) installed in the same oracle
// container, at version 0.7.0 (`/usr/lib/x86_64-linux-gnu/libfcl.so.0.7.0`,
// matching what `libgeometric_shapes.so` itself links against) — and
// because `fcl::OBB<S>` is a template class, its member functions are
// defined inline in headers, not a `.cpp`: `operator+`/`operator+=`,
// `merge_largedist`, `merge_smalldist` and `computeVertices` were read in
// full from `/usr/include/fcl/math/bv/OBB-inl.h`; `eigen_old`,
// `getCovariance` and `getExtentAndCenter` (the point-cloud, `ts ==
// nullptr` branches, the only ones FCL's OBB merge ever calls) from
// `/usr/include/fcl/math/geometry-inl.h`. [`OBB::contains_point`] and
// [`OBB::overlaps`] remain this port's own implementations of the
// documented behavior (see the module docs on [`OBB`]) — the probe in
// `tests/probe_parity.rs` confirms both already agree with the shipped
// binary, so there was no reason to displace them with FCL's own
// `contain`/`overlap`. [`OBB::extend_approx`]'s general-merge branch,
// which the same probe caught disagreeing with the binary, is now a
// literal port of `operator+`/`merge_largedist`/`merge_smalldist` instead.
//
// Round 8 re-verification: the cached source tree this file (and the round-8
// symbol audit below) reads from is a tarball matching the shipped package
// exactly by content (`geometric_shapes-2.3.3/` prefix, file mtimes
// `2025-06-06 20:41`, matching `CHANGELOG.rst`'s own `2.3.3 (2025-06-06)`
// entry — consistent with GitHub's per-tag source archive, not a hand-edited
// tree), and the cached `libgeometric_shapes.so.2.3.3` copy checked against
// it is byte-identical (`sha256sum`, `547881ff...`) to the one inside this
// round's freshly rebuilt oracle image, so the check below applies to the
// current tree. Re-ran shapes.rs's own string-table method for two files its
// original six-literal check did not cover, since this round's audit draws
// conclusions from both: `body_operations.cpp`'s "Creating body from shape:
// Unknown shape type %d" and `shape_operations.cpp`'s "Unable to save shape
// of type %d" / "Unknown shape type: '%s'" each appear in `strings
// libgeometric_shapes.so.2.3.3` exactly once. For the one round-8 finding
// that depends on a *negative* fact (`ConvexMesh::computeScaledVerticesFromPlaneProjections`
// is never called, module docs below) — a fact no string literal can prove
// — went one level past a source grep: `objdump -d`'d the shipped `.so`
// directly and confirmed zero `call` instructions anywhere in the library
// target that function's address, including inside
// `bodies::ConvexMesh::updateInternalData()`'s own disassembly. That holds
// regardless of whether the source tarball is letter-perfect, since it reads
// the compiled binary's actual call graph, not the tarball's text.

//! The posed, algorithmic half of `geometric_shapes`: `bodies::Body` and its
//! four concrete kinds, plus the bounding-volume types they return. Upstream
//! `namespace bodies` from the `geometric_shapes` package — see the
//! provenance comment above for how the source was obtained and verified,
//! and [`crate::geometry::shapes`] for the unposed shape data layer this module builds
//! on.
//!
//! # Scope
//!
//! This module ports `bodies::Body`'s four concrete subclasses — [`Sphere`],
//! [`Cylinder`], [`Cuboid`] (upstream `bodies::Box`, renamed for the same
//! reason as [`crate::geometry::Cuboid`]) and [`ConvexMesh`] — along with
//! `setPose`/`setDimensions`/`setScale`/`setPadding`, `containsPoint`,
//! `intersectsRay`, `computeVolume`, `computeBoundingSphere` (here
//! [`Body::compute_bounding_sphere`]), `computeBoundingCylinder`,
//! `samplePointInside`, [`AABB`], [`OBB`], and the free function
//! `mergeBoundingSpheres` (here [`merge_bounding_spheres`]). The free
//! function `computeBoundingSphere(vector<Body*>)` (`body_operations.h`) is
//! unported, and — an earlier round of this doc's own claim, corrected here
//! — is **not** equivalent to composing [`Body::compute_bounding_sphere`]
//! and [`merge_bounding_spheres`]. Reading `body_operations.cpp` shows its
//! real algorithm only considers `bodies::MESH` bodies at all (every other
//! body type is silently skipped, upstream's own comment above the loop
//! reading `// TODO - expand to all body types`), and even restricted to
//! mesh bodies it computes one sphere from the centroid of every mesh's
//! *raw* scaled/posed vertices pooled together, not by merging each body's
//! own precomputed [`BoundingSphere`] pairwise the way
//! [`merge_bounding_spheres`] does — a materially different result in
//! general, not a reformulation of the same one. It has zero callers
//! anywhere in the pinned `moveit2` tree (`rg -n
//! 'bodies::computeBoundingSphere\('` finds none outside `geometric_shapes`
//! itself), so there is no in-scope caller to port it for; the function is
//! listed here as unported rather than as composed from parts this port
//! already has. `mergeBoundingBoxes`/`mergeBoundingBoxesApprox` (here
//! [`merge_bounding_boxes`]/[`merge_bounding_boxes_approx`]) are included too
//! — they are one-line loops over [`AABB::extend_aabb`]/[`OBB::extend_approx`]
//! once those types exist, and upstream's own test coverage for [`OBB`]
//! (`test_bounding_box.cpp`'s `MergeBoundingBoxes` suite) runs through them.
//!
//! # Who actually calls this, in this workspace (round 10)
//!
//! This module's algorithms were originally scoped as "deferred to Phase 3
//! collision" (this crate's own round-1 brief) before being ported in round
//! 3 (`PORTING-PLAN.md` §16). That phrase implied `cspace-collision` would be
//! the consumer; re-checked this round against the tree as it now stands,
//! that was never the shape of it. `cspace-collision` explicitly declines
//! [`Body`] (its own `lib.rs`/`world.rs` module docs: "the `bodies::`
//! posed-geometry layer is ... out of scope for `World`") — its
//! `ParryCollisionEnv` backend builds directly on `parry3d-f64` shapes
//! through its own `PosedBody`, never on this module. The real, current
//! consumers are elsewhere:
//!
//! - [`Body::from_shape`]/[`Body::contains_point`] — `cspace_planning::constraints`'s
//!   `PositionConstraint` (`position.rs`), for constraint-region membership.
//! - [`Body::compute_bounding_sphere`] — `cspace_collision::distance_field`'s
//!   `distance_field.rs`.
//! - [`Body::compute_bounding_cylinder`] — `cspace_collision::distance_field`'s
//!   `collision_distance_field_types.rs`.
//! - [`Body::contains_point`] again — `cspace_collision::distance_field`'s
//!   `find_internal_points.rs`.
//! - [`Body::intersects_ray`], [`Body::compute_volume`] — exercised only by
//!   this module's own tests and [`crate`]'s `body_query_parity`/
//!   `probe_parity` right now; no *Rust* caller outside `cspace_core::geometry`
//!   (checked by `rg` for each method name across `crates/*/src`, excluding
//!   this file and `shapes.rs`). Their *upstream* consumers, or absence
//!   thereof, are decided per-method below (round 13 item 3, corrected round
//!   14 item 0a for `intersects_ray`) rather than left as one
//!   undifferentiated "no caller" line.
//! - [`Body::sample_point_inside`] — **has** a caller outside this crate:
//!   `cspace_planning::constraints`'s `ik_sampler.rs:254` (round 14 item 0b corrects
//!   round 13's "once ported" framing, which was already stale when
//!   written — see below).
//!
//!   Checked (round 11) whether "no caller" understates
//!   [`ConvexMesh::intersects_ray`] specifically, since §9.1's probe already
//!   found [`ConvexMesh::ray_intersections`] disagreeing with the C++ oracle
//!   under scale+padding (deviations 7 and 8, documented on
//!   [`ConvexMesh::ray_intersections`] itself). It does not: `intersects_ray`
//!   is a one-line wrapper, `!self.ray_intersections(origin, dir,
//!   None).is_empty()`, so it is not a *different*, unverified surface —
//!   `probe_parity.rs` calling `ray_intersections` directly already exercises
//!   the exact code `intersects_ray` runs. Also checked whether that probe
//!   work itself ever landed: branch `probe-parity`'s tip and commit
//!   `dbf50a7` both name a "probe bodies:: against the shipped
//!   libgeometric_shapes" change, but neither is an ancestor of `HEAD`
//!   (`git merge-base --is-ancestor dbf50a7 HEAD` fails). `git diff dbf50a7
//!   0032889 -- crates/cspace-core/tests/probe_parity.rs` is empty —
//!   `dbf50a7` is a byte-identical orphaned duplicate of `0032889`, which
//!   *is* an ancestor of `HEAD` and was extended by four follow-up commits
//!   already on `main` (`10b1909`, `16cf87b`, `aa80496`, `9ca1dd3`). Nothing
//!   is stranded.
//!
//! So "deferred to Phase 3 collision" is not a live UNFIXED condition: the
//! condition it named (cspace-collision existing) was met and then the
//! premise under it turned out false (cspace-collision does not need this
//! module at all). The narrower, still-accurate unported items are the ones
//! enumerated in the symbol audits directly below, each already qualified by
//! its own caller check rather than by scope-phase.
//!
//! ## [`Body::intersects_ray`] / [`Body::sample_point_inside`]: owners (round 13, item 3)
//!
//! "No caller outside `cspace_core::geometry` yet" (above, since round 10) is a
//! report line, not a state — closed here per-method rather than repeated.
//!
//! **[`Body::sample_point_inside`] — has a consumer: `cspace_planning::constraints`.**
//! Upstream, `moveit_core/constraint_samplers/src/
//! default_constraint_samplers.cpp:461`'s `IKConstraintSampler::samplePose`
//! calls `b[(i + k) % b.size()]->samplePointInside(random_number_generator_,
//! max_attempts, pos)` on `b`, the `std::vector<bodies::BodyPtr>` returned by
//! `PositionConstraint::getConstraintRegions()` — a real, indexed, per-body
//! call, not a `BodyVector`-mediated one. Round 13 recorded this as a future
//! consumer ("once ported"); it already was ported by then —
//! `cspace-constraints/src/ik_sampler.rs:254`'s `IkSampler::sample_pose`
//! (p1-robotmodel round 9, merged after this crate's round-13 base, so the
//! round-13 report was stale the moment it was written) calls exactly
//! [`Body::sample_point_inside`] on the clone-at-pose'd constraint-region
//! body: `body.sample_point_inside(max_attempts, &mut |lo, hi|
//! rng.random_range(lo..hi))`. That caller fixes the two things this port's
//! signature had to get right to be usable: `max_attempts: u32` is a plain
//! retry budget (not upstream's `RandomNumberGenerator&` bundled with it —
//! this port takes attempts and a sampler as two separate parameters), and
//! `uniform: &mut dyn FnMut(f64, f64) -> f64` is the `(lo, hi) -> sample`
//! closure every concrete body's `sample_point_inside` calls one or more
//! times per attempt (see [`Sphere::sample_point_inside`],
//! [`Cylinder::sample_point_inside`], etc.) — `ik_sampler.rs:254` supplies
//! `rng.random_range(lo..hi)` for it, which is exactly what upstream's
//! bundled `RandomNumberGenerator::uniformReal(lo, hi)` would have produced
//! through the same call sites.
//!
//! **[`Body::intersects_ray`] — ported (`Sphere`/`Cylinder`/`Cuboid`/
//! `ConvexMesh`/[`Body`]'s own enum dispatch, five `intersects_ray` methods
//! in this file), no consumer outside this crate.** This method is *not*
//! missing an implementation; what is missing is an external caller. Round
//! 13's wording ("stays unported", "decided unported for now") read as if
//! the method itself were absent, which it is not and never has been —
//! corrected here (round 14, item 0a) rather than repeated. Re-checked this
//! round against both trees: `rg -n intersectsRay` across the full
//! `geometric_shapes` 2.3.3 source (`src/`, `include/`, `test/`) finds
//! exactly two non-test call sites, both internal to `bodies.cpp` itself and
//! already accounted for above — `ConvexMesh::intersectsRay`'s bounding-box
//! pre-check (`bodies.cpp:1241`, mirrored by this port's `ray_intersections`
//! short-circuit, lines 156-164 above) and `BodyVector::intersectsRay`'s
//! first-hit loop over child bodies (`bodies.cpp:1414`, the wrapper decided
//! not-worth-porting at lines 263-278 above). Neither is an external
//! consumer; both are the same already-ported algorithm calling itself.
//! `rg -n intersectsRay` across all of `/home/stevek/work/moveit2`
//! (`moveit_core` and `moveit_ros`) returns zero hits — no moveit2-layer
//! caller exists either, unlike `sample_point_inside`. So this is upstream's
//! own answer, not a gap in what this port's search covered: nothing
//! anywhere in the pinned upstream tree calls a `Body`'s `intersectsRay`
//! except another `Body` method already covered by an existing decision.
//! Reopens if a future consumer appears — most plausibly Phase 3 collision's
//! `ParryCollisionEnv`/`PosedBody` path, if it ever needs a body-level ray
//! test this module could serve instead of `parry3d-f64`'s own ray-cast, or
//! if `BodyVector` gets a real caller and its decided-unported status (lines
//! 263-278) is itself reopened.
//!
//! ## `bodies.h` `Body`-base and `ConvexMesh`-extra symbol audit (round 8)
//!
//! **Counting convention.** Unlike `shapes.rs`'s and `cspace_core::octomap`'s
//! `tree.rs`'s single audit list, this crate documents `bodies.h` symbols
//! at their point of definition throughout this file (one doc comment per
//! ported method) rather than in one monolithic table; this section
//! supplements that with only the members not already named at their own
//! definition (`Body`'s dirty-setter half and the members [`Body`]'s own
//! variant dispatch subsumes) or in `body_operations.h`'s three sections
//! above.
//!
//! **Reproducible raw counts, spot-check (round 18, item 1).** Per-class
//! raw `public:` declaration counts from
//! `tools/ci/count-public-declarations.sh` against a
//! fresh oracle fetch of `bodies.h`:
//!
//! ```text
//! $ sg docker -c "docker run --rm --entrypoint bash moveit-rs/oracle:e7d32225310d3278 -c 'cat /opt/ros/rolling/include/geometric_shapes/geometric_shapes/bodies.h'" > /tmp/bodies.h
//! $ for c in Body Sphere Cylinder Box ConvexMesh BodyVector; do
//! >   echo "$c: $(tools/ci/count-public-declarations.sh /tmp/bodies.h "$c")"
//! > done
//! Body: 28
//! Sphere: 16
//! Cylinder: 16
//! Box: 16
//! ConvexMesh: 20
//! BodyVector: 12
//! ```
//!
//! Every raw declaration in every one of the six lists was matched by
//! name against a method doc comment somewhere in this file (`getType`
//! through `updateInternalData`, all four `Body::` accessor pairs, every
//! `Sphere`/`Cylinder`/`Box`/`ConvexMesh` override, `BodyVector`'s own
//! twelve) except `Body()`/`~Body()` and each concrete class's own
//! constructor/destructor pair, all covered by the D4 design section
//! below the same way `shapes.rs`'s D4 section covers `Shape`'s base
//! ctor/dtor -- with one difference this round confirmed rather than
//! assumed: unlike `shapes::Mesh::~Mesh()`, every `bodies::` destructor
//! (`Sphere`/`Cylinder`/`Box`/`ConvexMesh`) is `= default` upstream, doing
//! no real cleanup work a `Vec`-based port would need to account for
//! separately -- the destructor gap this round found and fixed in
//! `shapes.rs` does not recur here. `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`
//! (a bare macro invocation, not a declaration) is excluded by the
//! counting script itself, the same way a `using`/preprocessor line is.
//! No gap found; this is the deliverable for this header, per "맞으면
//! 표 자체가 결과물이다."
//!
//! Members not already named above, classified:
//!
//! - `Body::getType()` — **subsumed by D4.** A caller matches on [`Body`]'s
//!   variant directly (`matches!(body, Body::Sphere(_))`, or the `match`
//!   arms every dispatch method here already uses) instead of comparing a
//!   `ShapeType` tag that D4 makes impossible to desync from the real type
//!   in the first place. `collision_distance_field_types.cpp:63`'s
//!   `if (body->getType() == shapes::ShapeType::SPHERE)` is the one in-scope
//!   caller (`cspace_collision::distance_field`'s port this round, not this crate's).
//! - `setScaleDirty`/`setPaddingDirty`/`setPoseDirty`/`setDimensionsDirty`
//!   (the batch-then-`updateInternalData()` half of each setter pair) —
//!   **subsumed by the "no dirty/clean setter pair" design** (see below):
//!   every setter here recomputes eagerly, so there is no separate `*Dirty`
//!   entry point to port.
//! - `containsPoint(double x, double y, double z, bool verbose = false)`
//!   (the raw-coordinate convenience overload) and the `verbose` parameter
//!   on both `containsPoint` overloads — **unported.** The 3-`double`
//!   overload is a one-line wrapper that builds an `Eigen::Vector3d` and
//!   forwards; callers here already pass `&Vector3` directly, so there is
//!   nothing the wrapper would save. `verbose` only ever
//!   `CONSOLE_BRIDGE_logDebug`s inside `containsPoint`'s per-body
//!   implementations; this crate does not log (the same reasoning already
//!   given for `print()`), so there is no destination for the message that
//!   parameter would enable.
//! - `useDimensions` (protected) — **subsumed**, folded into each concrete
//!   body's own constructor/`recompute`, since this port never constructs an
//!   empty [`Body`] and fills its dimensions in a second step the way
//!   `createEmptyBodyFromShapeType` + `setDimensionsDirty` does.
//! - `cloneAt` (both overloads) — **ported** as [`Sphere::clone_at`]/
//!   [`Cylinder::clone_at`]/[`Cuboid::clone_at`]/[`ConvexMesh::clone_at`]
//!   (the 1-arg, current-padding-and-scale overload) and each type's
//!   `clone_at_with` (the 3-arg overload).
//! - `ConvexMesh::getTriangles`/`getVertices`/`getScaledVertices`/`getPlanes`
//!   — **ported** as [`ConvexMesh::triangles`]/[`ConvexMesh::vertices`]/
//!   [`ConvexMesh::scaled_vertices`]/[`ConvexMesh::planes`] (deviation 2
//!   above covers where the last two cannot match 1:1).
//! - `ConvexMesh::computeScaledVerticesFromPlaneProjections` — **unported,
//!   dead code upstream.** See the provenance comment at the top of this
//!   file for the disassembly-level proof that it is never called from
//!   anywhere in the shipped binary; [`ConvexMesh`]'s own `recompute` already
//!   matches `updateInternalData()`'s actual (simpler, radial-scaling) inline
//!   logic instead.
//! - `ConvexMesh::correctVertexOrderFromPlanes` — **not needed**, per
//!   deviation 2 above: `parry3d-f64`'s `try_convex_hull` already guarantees
//!   CCW winding, so there is no per-facet vertex-order correction to make.
//! - `ConvexMesh::countVerticesBehindPlane` — **unported.** Its own doc
//!   comment upstream calls it "used mainly for debugging"; `rg -n
//!   'countVerticesBehindPlane' /home/stevek/work/moveit2` returns no hits —
//!   zero callers anywhere in the pinned tree.
//!
//! ## `body_operations.h` symbol audit (round 8)
//!
//! **Reproducible raw count (round 18, item 1).** Free functions at
//! namespace scope; verified with a signature-line grep against a fresh
//! oracle fetch, the same recipe `shapes.rs` uses for `shape_operations.h`:
//!
//! ```text
//! $ sg docker -c "docker run --rm --entrypoint bash moveit-rs/oracle:e7d32225310d3278 -c 'cat /opt/ros/rolling/include/geometric_shapes/geometric_shapes/body_operations.h'" > /tmp/body_operations.h
//! $ grep -c '^[A-Za-z].*(\|^  .*(' /tmp/body_operations.h
//! 11
//! ```
//!
//! Matches this section's stated 11 exactly.
//!
//! The remaining 4 of `body_operations.h`'s 11 declarations, classified:
//!
//! - `createEmptyBodyFromShapeType(ShapeType)` — **subsumed by
//!   [`Body::from_shape`].** Upstream's own callers never call this alone;
//!   every in-scope one (`kinematic_constraint.cpp:411,438`,
//!   `distance_field.cpp:223,301,316`) immediately follows it with
//!   `setDimensionsDirty(shape)`/`setPoseDirty(pose)`/`updateInternalData()`
//!   — the exact composition [`Body::from_shape`] performs in one call under
//!   this port's no-dirty-flag design (see below), followed by
//!   [`Body::set_pose`] for the pose half.
//! - `createBodyFromShape(const Shape*)` — **ported as
//!   [`Body::from_shape`]**, already documented on that method as its direct
//!   upstream counterpart. `body_operations.cpp` confirms the two are the
//!   same composition (`createEmptyBodyFromShapeType(shape->type)` then
//!   `body->setDimensions(shape)`), so this is not a fresh equivalence claim,
//!   only cross-referencing one already made.
//! - `constructShapeFromBody(const Body*)`, `constructMarkerFromBody(const
//!   Body*, Marker&)`, and all three `constructBodyFromMsg` overloads —
//!   **unported.** `rg -n
//!   'constructShapeFromBody|constructMarkerFromBody|constructBodyFromMsg'
//!   /home/stevek/work/moveit2` returns no hits for any of the four —  zero
//!   callers anywhere in the pinned tree, not merely "not requested". The
//!   marker/message ones would be D1-excluded regardless
//!   (`visualization_msgs`/`shape_msgs`), matching `shapes.rs`'s message
//!   conversions; `constructShapeFromBody` takes no message type and could in
//!   principle be ported, but has no caller to port it for either.
//!
//! It deliberately does **not** port `bodies::BodyVector` — declared in
//! `bodies.h`, not `body_operations.h`, included here because it is the one
//! other unported symbol in the `bodies` namespace. Checking this claim
//! against the pinned tree this round (`rg -n 'BodyVector'
//! /home/stevek/work/moveit2`) found it does have a real in-scope caller —
//! `collision_distance_field_types.hpp:293`'s `bodies::BodyVector bodies_;`
//! member (`moveit_core/collision_distance_field`, `cspace_collision::distance_field`'s
//! port this round, not this crate's) — so "not in the requested scope" from
//! an earlier round of this doc was an unverified guess, not a checked fact;
//! corrected here rather than repeated. It is still not ported, but for a
//! narrower and now-verified reason: `BodyVector` itself is a thin
//! `std::vector<Body*>` plus loop-based `containsPoint`/first-hit
//! `intersectsRay`/indexed `getBody`, entirely composable from a plain
//! `Vec<Body>` and the per-body methods this crate already exposes
//! ([`Body::contains_point`], [`Body::intersects_ray`]) — there is no
//! algorithm here beyond the loop itself.
//!
//! **Decided (round 11): a wrapper buys nothing concrete, checked against
//! `cspace_collision::distance_field`'s actual usage rather than left as that crate's
//! call.** `cspace_collision::distance_field`'s only composition of `Vec<Body>` is
//! `BodyDecomposition::from_shapes` (`collision_distance_field_types.rs:711-721`),
//! which builds the vector by a plain `Vec::with_capacity` + `push` loop, and
//! every consumer of the resulting field is whole-vector iteration or
//! indexed access, never `BodyVector`'s first-hit query:
//! `collision_distance_field_types.rs:726`'s `for body in &bodies` runs to
//! completion three times over (collision spheres, internal points via
//! [`Body::contains_point`], bounding spheres via
//! `Body::compute_bounding_sphere`), and `BodyDecomposition::body`/
//! `bodies_count` (`:781-786`) are a bounds-checked index and a length, not a
//! search. Upstream `BodyVector::containsPoint`/`intersectsRay` return on the
//! *first* body that matches; nothing in this workspace ever needs that
//! short-circuit — every call site needs the full set. A wrapper here would
//! duplicate `Vec<Body>` plus re-derive the loops
//! `BodyDecomposition::from_shapes` already writes directly, for a query
//! shape (first-hit) that has no caller. `cspace_collision::distance_field`'s own doc
//! independently reaches the same conclusion for the sibling
//! `BodyDecompositionVector` (`lib.rs:155-160`: phantom upstream type,
//! forward-declared and never defined, so "unported" there is not even a
//! design choice) — the pattern in that crate is plain `Vec`/`&[T]`
//! throughout, with no vector-wrapper type anywhere in its `Body`/
//! `BodyDecomposition` handling.
//!
//! # Design: enum, not a trait-object hierarchy (D4)
//!
//! Upstream `bodies::Body` is `geometric_shapes`'s second abstract base
//! class (after `shapes::Shape`), with `Sphere`, `Cylinder`, `Box`,
//! `ConvexMesh` as concrete subclasses carrying a `shapes::ShapeType type_`
//! tag alongside the `static_cast<const Sphere*>(body)`-style downcasts that
//! tag licenses — the exact "value's meaning depends on a side tag" pattern
//! [`crate::geometry::shapes`]'s module docs already argue against for `shapes::Shape`.
//! The same argument applies here unchanged, so [`Body`] is a closed enum
//! for the same reason.
//!
//! # Design: cached derived fields, no dirty/clean setter pair
//!
//! Upstream's `Body` exposes two setters per mutable property —
//! `setScaleDirty`/`setScale`, `setPaddingDirty`/`setPadding`,
//! `setPoseDirty`/`setPose`, `setDimensionsDirty`/`setDimensions` — where the
//! `Dirty` half writes the raw field only, and the plain half additionally
//! calls the virtual `updateInternalData()` that recomputes every cached
//! derived quantity (`radius2_`, `center_`, `normalH_`, ...). The `Dirty`
//! half exists purely so a caller changing several properties at once can
//! defer that recomputation to a single trailing `updateInternalData()` call
//! instead of paying for it after each setter.
//!
//! This port keeps the cache (each body variant's fields split into
//! "shape-dependent" and "pose/scale/padding-dependent" sections, mirroring
//! `bodies.h`'s own comments) but drops the `Dirty` half: every setter here
//! recomputes eagerly, so there is exactly one way to mutate a body and it
//! is always self-consistent — no state exists where the cache and the
//! source fields disagree. A caller batching several changes pays for one
//! extra recomputation per change instead of one total; `computeVolume` and
//! friends stay infallible reads of already-validated fields either way, and
//! recomputation is cheap arithmetic, not the qhull/mesh work that would
//! justify optimizing it. [`Body::set_pose`] is infallible (no upstream
//! validation depends on the pose alone); [`Body::set_scale`] and
//! [`Body::set_padding`] return [`Result`] because — exactly as upstream —
//! a large enough negative padding can drive a radius or extent below zero.
//!
//! # Deviations from upstream
//!
//! 1. **`intersectsRay`'s nullable `intersections` out-param becomes two
//!    methods.** Upstream's single `intersectsRay(origin, dir, intersections
//!    = nullptr, count = 0)` has a real fast path when `intersections ==
//!    nullptr`: several branches `return true` immediately on the first hit
//!    without computing every intersection point. [`Body::intersects_ray`]
//!    is that fast path (bool only); [`Body::ray_intersections`] is the full
//!    computation, returning the ordered points directly instead of writing
//!    through a pointer. `count: Option<usize>` replaces the `0 = unlimited`
//!    magic value — `None` unlimited, `Some(n)` capped at `n` — matching
//!    upstream's `filterIntersections` semantics without a sentinel that
//!    means something different from every other `count`.
//! 2. **The convex hull comes from `parry3d-f64`'s quickhull, not qhull.**
//!    Upstream links `libqhull_r`, which is not a dependency of this
//!    workspace and was not added unilaterally — `parry3d-f64` is already a
//!    pinned workspace dependency (`PORTING-PLAN.md` §3) and its
//!    `transformation::try_convex_hull` was surveyed this session
//!    specifically for this use (`utils::obb`'s PCA-fitting OBB and
//!    `bounding_volume::Aabb` were surveyed too and rejected — the former
//!    computes the wrong thing, a best-fit box, not a caller-posed one; the
//!    latter doesn't match `AABB`'s `Eigen::AlignedBox3d`-derived API
//!    closely enough to be worth adapting instead of the ~15-line hand
//!    roll). `try_convex_hull`'s CCW-winding guarantee on its output
//!    triangles means [`ConvexMesh`] computes each triangle's own plane
//!    directly from its own three vertices, rather than porting
//!    `correctVertexOrderFromPlanes` (upstream needs that pass because
//!    qhull's own per-facet vertex order isn't guaranteed to agree with the
//!    facet's plane normal) or `plane_for_triangle_`/`triangle_for_plane_`
//!    (upstream's facet-plane-merging maps, which exist only to reuse one
//!    plane across every triangle of a multi-triangle facet — an
//!    optimization, not a behavior: a convex region bounded by `N` planes
//!    and the same region bounded by `N` redundant co-planar
//!    half-spaces are the same region). [`ConvexMesh`] exposes
//!    [`ConvexMesh::vertices`] and [`ConvexMesh::scaled_vertices`] as exact
//!    matches for upstream's `getVertices`/`getScaledVertices` (both are
//!    hull-vertex sets, and a probe confirms the same vertex set — up to
//!    permutation — for meshes where every input point is a true hull
//!    vertex). [`ConvexMesh::triangles`] matches upstream's `getTriangles`
//!    in what it enumerates (one hull triangle per entry) but not
//!    necessarily in its exact triangulation: `parry3d-f64`'s quickhull and
//!    qhull can choose different face diagonals across a coplanar patch, so
//!    the *topology* is not guaranteed to agree, only vertex membership and
//!    hull volume. [`ConvexMesh::planes`] cannot match upstream's
//!    `getPlanes` 1:1 for the reason above: this port keeps one entry per
//!    *triangle*, not per *facet*, so its length is a superset of
//!    upstream's plane-merged output whenever the mesh has a coplanar
//!    patch (e.g. a box: 12 triangles here against upstream's 6 merged
//!    facets) — see that method's own doc for what it returns instead.
//!
//!    **Falsifier for "does the topology difference matter" (round 8):**
//!    for two of [`ConvexMesh`]'s three triangle-consuming methods, it
//!    provably cannot, by construction rather than by testing.
//!    [`ConvexMesh::compute_volume`] sums each triangle's signed
//!    origin-tetrahedron volume; that sum is a standard consequence of the
//!    divergence theorem and is invariant to how a closed convex surface is
//!    triangulated, so any diagonal choice across a coplanar patch gives the
//!    identical total. [`ConvexMesh::contains_point`] (and
//!    `samplePointInside`'s rejection sampling, which is built on it) ANDs
//!    over every triangle's plane inequality; `convex_mesh_planes_accessor_dedups_to_libgeometric_shapes_box`
//!    (`tests/probe_parity.rs`) already proves this port's per-triangle
//!    plane set dedups to upstream's per-facet set exactly, and ANDing extra
//!    duplicate constraints changes nothing, so the result cannot depend on
//!    which diagonal either hull library picked. The one method where
//!    triangulation is not merely cosmetic is
//!    [`ConvexMesh::ray_intersections`]/`intersects_ray`: each candidate hit
//!    is a genuine point-in-*triangle* test (barycentric cross-product signs
//!    against that triangle's own three vertices, not just its plane), so a
//!    ray whose hit point lands exactly on the shared edge between a
//!    coplanar patch's two triangles depends on which triangle each
//!    library's own floating-point tie-break assigns that point to — the
//!    entry/exit *coordinate* is still identical (both triangles of a facet
//!    share one plane equation, so `t` is the same), but whether the ray is
//!    reported as a hit at all can differ at that exact measure-zero
//!    boundary. That is not otherwise observable: it requires a ray
//!    constructed to land precisely on the specific diagonal each library
//!    happened to choose, which is a property of quickhull's and qhull's own
//!    internal tie-breaking, not something this port controls or could match
//!    without adopting qhull's numerics wholesale — the thing this deviation
//!    already gives a reasoned decision not to do.
//! 3. **`bodies::OBB`'s `contains_point`/`overlaps` are this port's own
//!    implementation, not a literal port; `extend_approx`'s general-merge
//!    case is.** See the provenance comment above. [`OBB::contains_point`]
//!    is the one unambiguous case (inverse-transform the point into the
//!    box's local frame, compare against the half-extents componentwise —
//!    there is only one reasonable meaning for "an oriented box contains a
//!    point"). [`OBB::overlaps`] implements the standard 15-axis
//!    separating-axis test for two oriented boxes (3 face normals of each
//!    box, plus the 9 pairwise cross products — Gottschalk et al. 1996;
//!    Ericson, *Real-Time Collision Detection* §4.4.1), a textbook
//!    algorithm independent of FCL's implementation; a binary-ground-truth
//!    probe (`tests/probe_parity.rs`) confirms both agree with the shipped
//!    `.so` exactly. [`OBB::extend_approx`]'s two shortcut cases (this box
//!    has zero extent; one box wholly contains the other) are ported
//!    byte-for-byte from `obb.cpp`, which spells them out before
//!    delegating the general case to FCL's `OBB::operator+=`; that general
//!    case dispatches on the two boxes' center distance to `merge_largedist`
//!    (PCA-fit over both boxes' 16 vertices projected perpendicular to the
//!    center-to-center axis; needs `geometry-inl.h`'s `eigen_old` classical
//!    Jacobi eigendecomposition, `getCovariance` and `getExtentAndCenter`) or
//!    `merge_smalldist` (hemisphere-corrected quaternion-average orientation,
//!    arithmetic-mean center, then componentwise min/max of both boxes'
//!    vertices projected onto the merged axes) — a literal port of
//!    `OBB-inl.h`'s `operator+` **except** `merge_smalldist`'s own min/max
//!    tracking, which is not: see deviation 10. A probe (`tests/probe_parity.rs`)
//!    pins the rest of both branches against the shipped binary; its one
//!    `merge_smalldist` fixture does not happen to land in the input region
//!    deviation 10 covers, which is why nothing caught that one line's gap
//!    from "literal port" until this audit.
//! 4. **`samplePointInside` takes a caller-supplied uniform-sampler closure,
//!    not a `random_numbers::RandomNumberGenerator`.** That type has no Rust
//!    port (PORTING-PLAN.md records no mature substitute was pulled in for
//!    this crate), and hard-coding a specific RNG crate as a runtime
//!    dependency of a geometry-primitives crate is a heavier commitment than
//!    the port needs — nothing here requires a *specific* RNG, only *a*
//!    source of uniform reals on a range, which is exactly what upstream's
//!    `rng.uniformReal(lo, hi)` calls are. [`Body::sample_point_inside`]
//!    takes `impl FnMut(f64, f64) -> f64` instead; this module's own tests
//!    supply a small inline deterministic generator (not `rand`, to avoid
//!    adding a dependency for tests alone) and check the same invariants
//!    upstream's `random_numbers`-driven property tests check (the sampled
//!    point satisfies `contains_point`, lies within the computed bounding
//!    sphere, ...) rather than porting those tests' exact iteration count or
//!    RNG sequence, which is inherently tied to an RNG this port does not
//!    have.
//! 5. **`bodies::Body::cloneAt`/`Sphere`/`Cylinder`/`Box`'s
//!    `BoundingSphere`/`BoundingCylinder`/`AABB` constructors become
//!    [`Body::clone_at`]/[`Sphere::from_bounding_sphere`]/
//!    [`Cylinder::from_bounding_cylinder`]/[`Cuboid::from_aabb`].** Same
//!    shape, upstream just spells them as C++ constructor overloads and this
//!    port has closed enum variants, not classes, to attach a constructor
//!    to.
//! 6. **Mesh-loading infrastructure (`.dae`/`.stl` via
//!    `shapes::createMeshFromResource`) is out of scope**, exactly as noted
//!    in `shapes.rs`. Every upstream test that builds a [`ConvexMesh`] from
//!    a loaded resource (`MeshPointContainment::Basic`/`Pr2Forearm` in
//!    `test_point_inclusion.cpp`, the loaded-mesh cases in
//!    `test_ray_intersection.cpp`) has no ground truth this port can use.
//!    This module's [`ConvexMesh`] tests instead build meshes by hand from
//!    explicit vertex/triangle lists — the same `createBoxMesh(min, max)`
//!    pattern `test_bounding_cylinder.cpp`/`test_bounding_box.cpp` use for
//!    their own mesh cases (8 vertices, 12 triangles) — and cross-check
//!    against the equivalent [`Cuboid`] body, since a box-shaped convex mesh
//!    and a [`Cuboid`] describe the same geometry.
//! 7. **`ConvexMesh::ray_intersections` keeps a sign convention that
//!    disagrees with the shipped `.so` on 3 of the probe's 5 rays, because
//!    the binary's own two relevant methods disagree with each other.** A
//!    binary-ground-truth probe found `intersectsRay` on a padded, scaled
//!    tetrahedron reporting a hit count on `ray[0]`, `ray[2]` and `ray[4]`
//!    that is topologically impossible given the *same binary's* own
//!    `containsPoint` answers (the probe originally cited only `ray[0]`;
//!    enabling the probe's other rays past it surfaced that the same defect
//!    also hits `ray[2]` and `ray[4]`). Upstream's `isPointInsidePlanes`
//!    recomputes each plane's padded/scaled offset as `-plane_normal.dot(
//!    scaled_vertex)`, while `intersectsRay` independently recomputes the
//!    *same* quantity as `+plane_normal.dot(scaled_vertex)` (`bodies.cpp`,
//!    read in full — confirmed byte-for-byte, not a transcription slip).
//!    This port caches one signed `plane_offsets` value per triangle
//!    (`ConvexMesh::recompute`) and reuses it in both
//!    `ConvexMesh::is_point_inside_planes` and
//!    [`ConvexMesh::ray_intersections`], so it cannot reproduce this
//!    inconsistency by construction. Proof the binary, not this port, is
//!    wrong, ray by ray (each argument needs only boundedness, not
//!    convexity, and no reference beyond the fixture's own other outputs):
//!    - `ray[0]`: an exterior origin, running through the posed body's
//!      probe-origin point (confirmed interior by the fixture's own
//!      `containsPoint`) and beyond. A bounded region can only be entered
//!      and later exited around a point it truly contains, so the two hits
//!      must bracket that point; the binary reports 1 (unbracketed) hit.
//!    - `ray[2]`: a second, orthogonal ray through the same confirmed-
//!      interior point. The binary reports 1 hit, and that hit falls
//!      *after* the interior point in the ray's parametrization — i.e. by
//!      the binary's own account, zero boundary crossings occur before
//!      reaching a point its own `containsPoint` calls interior, the same
//!      contradiction as `ray[0]`.
//!    - `ray[4]`: origin *is* the confirmed-interior point itself, so
//!      escaping a bounded region to infinity requires an odd number of
//!      crossings. This port reports 1 (odd, correct); the binary reports 2
//!      (even, impossible from an interior start).
//!
//!    This port keeps its single, self-consistent sign convention on all
//!    three; `tests/probe_parity.rs`'s `convex_mesh_sign_bug_upstream_defect`
//!    documents the fixture keys this disagreement lands on and asserts
//!    this port's own (internally consistent) answers, plus the bracket/
//!    parity check proving each one, instead of the fixture's values for
//!    those three rays.
//! 8. **`ConvexMesh::ray_intersections` disagrees with the shipped `.so` on
//!    `ray[1]` for an unrelated reason: anchor-choice non-uniqueness, not
//!    the deviation-7 sign bug.** Both sides report the same, topologically
//!    consistent 2-hit count on `ray[1]` (unlike deviation 7's three rays),
//!    but at genuinely different positions. `ConvexMesh::recompute`
//!    applies scale and padding radially, per vertex, from the mesh center
//!    (`updateInternalData` upstream), so under non-zero padding a
//!    triangle's 3 padded vertices are in general no longer coplanar; each
//!    triangle's plane offset is then only well-defined relative to
//!    whichever of its 3 vertices is used to compute it, and that choice is
//!    an artifact of the convex hull library's internal vertex-enumeration
//!    order for that facet, not a geometric property of the face. Deviation
//!    2 already documents that this port's hull comes from `parry3d-f64`'s
//!    quickhull rather than upstream's qhull; the two libraries pick
//!    different anchor vertices for the face `ray[1]` grazes, and hand
//!    verification against the shipped binary's own qhull-ordered vertex
//!    data confirmed the two anchors give measurably different offsets
//!    (this port's anchor: ≈ -0.0433; qhull's own anchor for the identical
//!    face: ≈ -0.0405) — a real difference in which real number is correct
//!    under an anchor-dependent construction, not floating-point noise.
//!    Nothing about this is fixable within this port without matching
//!    qhull's own facet vertex order exactly, which deviation 2 already
//!    declined to depend on. `tests/probe_parity.rs`'s
//!    `convex_mesh_ray1_anchor_choice_deviation` asserts the hit count and
//!    the bracket invariant against the fixture's `containsPoint`, plus
//!    this port's own regression-pinned positions — not the fixture's
//!    exact positions, which have no more claim to being "correct" than
//!    this port's under an anchor-dependent formula.
//! 9. **A zero-vertex mesh is an error here; upstream builds a body with an
//!    infinite bounding-cylinder radius and returns normally.**
//!    `build_mesh_data`'s `mesh.vertices.is_empty()` guard has no upstream
//!    counterpart at all. Upstream's `useDimensions` (`bodies.cpp:825-944`)
//!    returns `void` and has no early exit for `vertex_count == 0`: the
//!    min/max loop never runs, the `if (maxX < minX) maxX = minX = 0.0;`
//!    clamps turn the untouched infinities into a zero-size box at the
//!    origin, the cylinder-axis selection falls through both `>` tests to
//!    the `else` arm giving `cyl_length = 0`, and then — the part that
//!    matters — `maxdist` is *only* lowered inside the per-vertex loop, so
//!    with no vertices it keeps its `-std::numeric_limits<double>::infinity()`
//!    initializer and is assigned straight to
//!    `mesh_data_->bounding_cylinder_.radius`. `qh_new_qhull` is then called
//!    with 0 points, fails, logs `"Convex hull creation failed"` at warn
//!    level, and `useDimensions` returns. The caller gets a fully
//!    constructed `ConvexMesh` whose vertices, triangles and planes are
//!    empty and whose bounding-cylinder radius is `-inf`, with nothing in
//!    the return type to say so.
//!
//!    Erroring instead is this port's D6 policy (untrustworthy input returns
//!    `Err` rather than silently falling back) applied to a case where the
//!    silent fallback is not merely lossy but poisoned: `-inf` propagates
//!    through every downstream bounding-volume comparison without ever
//!    tripping a NaN check. Note the consequence for the deviation-2
//!    quickhull swap — because the guard returns first, this port never
//!    reaches `try_convex_hull` on an empty vertex set, so it has no
//!    observation of how `parry3d-f64` behaves there and does not need one.
//!    `convex_mesh_zero_vertex_is_an_error` (this crate) pins the guard, and
//!    `new_rejects_a_mesh_whose_body_construction_fails`
//!    (`cspace-constraints/tests/decide.rs`) pins that the `Err` survives out
//!    through `PositionConstraint::new` rather than being swallowed on the
//!    way.
//!
//! 10. **`merge_smalldist`'s per-axis min/max tracking is two independent
//!     `if`s; upstream's is `if`/`else if`, and that is an upstream bug this
//!     port does not reproduce.** `OBB-inl.h:341-345,355-359`'s loop is
//!     `if (dot > pmax[j]) pmax[j] = dot; else if (dot < pmin[j]) pmin[j] =
//!     dot;` — mutually exclusive, so a vertex that raises `pmax[j]` is
//!     never also checked against `pmin[j]`. Depending on vertex-visitation
//!     order this can leave `pmin[j]` at its `real_max` sentinel, or
//!     otherwise above the true minimum. `merge_smalldist` uses two
//!     independent `if`s, which always compute the true componentwise
//!     min/max regardless of order. Despite the surrounding claim ("a
//!     literal port of `OBB-inl.h`'s `operator+`") and the deviation-7
//!     cross-reference above, this one line was never a byte-for-byte port
//!     and the round-12 probe below does not happen to exercise the input
//!     region where the two disagree (its one `merge_smalldist` fixture is
//!     not one of them) — so nothing previously caught or documented this.
//!     `merge_smalldist_matches_true_min_max_not_fcls_order_dependent_replica`
//!     (this module) pins the port's output against an independent
//!     min/max oracle *and* confirms FCL's own `else if` shape disagrees on
//!     the same input, so the fixture cannot silently stop exercising the
//!     divergence.
//!
//! # Bounding-volume methods, checked against an external reference (round 12)
//!
//! The defect class this audit checks for: a bounding volume that is
//! uniformly wrong in a way self-consistency tests cannot catch — a sphere
//! radius twice too large still "contains every vertex" and "grows
//! monotonically" and passes every containment/overlap test built from the
//! method's own output. The one thing that catches it is a value computed
//! by a *different* implementation. For every method named for this round
//! ([`Sphere::compute_bounding_sphere`]/[`Cylinder::compute_bounding_sphere`]/
//! [`Cuboid::compute_bounding_sphere`]/[`ConvexMesh::compute_bounding_sphere`],
//! the four types' `compute_bounding_cylinder`, `compute_volume`, and
//! [`OBB::extend_approx`]), that different implementation already exists and
//! is already wired in:
//!
//! - **`tests/probe_parity.rs`**: `bodies_probe.json` is the `%.17g` stdout
//!   of a standalone C++ program linked directly against the real,
//!   installed `libgeometric_shapes.so.2.3.3` (PORTING-PLAN.md §9.1) — not
//!   a value this port computed and echoed back, and not upstream's own
//!   gtest literals either (those live in `test/*.cpp`, never compiled into
//!   the `.so`). The `check_body!` macro applies `compute_volume`,
//!   `compute_bounding_sphere`, `compute_bounding_cylinder`,
//!   `compute_bounding_aabb`, and `compute_bounding_obb` identically to all
//!   four body types (`sphere_matches_libgeometric_shapes`,
//!   `cylinder_matches_libgeometric_shapes`, `box_matches_libgeometric_shapes`,
//!   `convex_mesh_matches_libgeometric_shapes`) at a pose that is both
//!   translated and rotated, under nonzero scale and padding — every one of
//!   the six methods this round names is covered for every body type,
//!   including [`ConvexMesh`], which has no simple closed-form value to
//!   check by hand and so has no ground truth *other* than a second,
//!   independent implementation. [`OBB::extend_approx`]'s two branches are
//!   both separately pinned: `obb_predicates_match_libgeometric_shapes`
//!   forces the `merge_smalldist` branch, and
//!   `obb_extend_approx_merge_largedist_matches_libgeometric_shapes` forces
//!   `merge_largedist` (see deviation 3 above for why the fixture needed a
//!   second, far-apart OBB pair to reach that branch at all).
//! - **The oracle's `body_query` op** (`tools/moveit-oracle/src/oracle.cpp`,
//!   `bodyQuery`) independently corroborates three of the four body types —
//!   sphere, cylinder, box (`tests/fixtures/body_query_{request,response}.json`,
//!   4 cases: 2 sphere, 1 cylinder, 1 box; no mesh case) — computed by
//!   `bodies::createEmptyBodyFromShapeType`/`computeBoundingSphere`/
//!   `computeBoundingCylinder`/`computeVolume` running inside the oracle
//!   container against the *same* shipped library the standalone probe
//!   links, but through a completely different harness (the oracle's JSON
//!   request/response protocol, `body_posed_algorithms_match_the_oracle` in
//!   `tests/body_query_parity.rs`) — a second source, not a duplicate of
//!   the first.
//!
//! Conclusion: nothing named this round needs pinning: every one of
//! `compute_bounding_sphere`, `compute_bounding_cylinder`, `compute_volume`,
//! and `OBB::extend_approx` is already checked against a value this port
//! did not itself compute, for every body type this crate has, by two
//! independently-harnessed routes to the same real upstream binary.

use crate::error::{Error, Result};

use crate::geometry::numeric::{cxx_max, cxx_min};
use crate::geometry::shapes::{Mesh as ShapeMesh, Shape};
use crate::geometry::{BoundingSphere, Isometry3, Vector3};

const ZERO: f64 = 1e-9;

/// Normalize `dir`, unless it is already (very nearly) unit length. Upstream
/// `bodies::normalize` — the guard avoids paying for a square root on the
/// overwhelmingly common case of a caller who already normalized.
///
/// A zero vector's squared norm is `0.0`, so `(0.0 - 1.0) > 1e-9` is false
/// and this returns the zero vector unchanged — matching upstream exactly,
/// which does the same (the `norm - 1 > 1e-9` guard is false for `norm ==
/// 0.0` too). See [`Body::ray_intersections`]'s docs for what a zero
/// direction does downstream.
fn normalize_dir(dir: &Vector3) -> Vector3 {
    let norm_sqr = dir.norm_squared();
    if (norm_sqr - 1.0) > ZERO {
        dir / norm_sqr.sqrt()
    } else {
        *dir
    }
}

/// Transform `point` — a *position*, not a free vector — by `pose`,
/// applying both rotation and translation.
///
/// This crate represents both positions and free vectors as bare
/// [`Vector3`], but nalgebra's `Isometry3: Mul<Vector3>` gives *vector*
/// semantics (rotation only, no translation — correct for directions and
/// normals). Multiplying a pose directly by a `Vector3` that actually holds
/// a position silently drops the isometry's translation. Every site in this
/// module that carries a position across frames (a local corner becoming a
/// world vertex, a world ray origin becoming a local one, ...) must go
/// through this helper instead.
fn transform_point(pose: &Isometry3, point: &Vector3) -> Vector3 {
    (pose * nalgebra::Point3::from(*point)).coords
}

/// The squared distance between a ray (through `origin`, along the
/// already-normalized `dir`) and a point. Upstream `detail::distanceSQR`.
fn distance_sqr(p: &Vector3, origin: &Vector3, dir: &Vector3) -> f64 {
    let a = p - origin;
    let d = dir.dot(&a);
    a.norm_squared() - d * d
}

/// A candidate ray/body intersection, carrying the ray parameter it was
/// found at so a batch of candidates can be ordered and deduplicated.
/// Upstream `detail::intersc`.
struct Intersc {
    pt: Vector3,
    time: f64,
}

/// Sort `candidates` by ray parameter, drop near-duplicates (within `ZERO`
/// of the previous kept point, in `Vector3d::isApprox`'s sense — the case
/// where a ray grazes exactly the shared boundary between two primitives,
/// e.g. a cylinder's side and base, and is reported once per primitive),
/// and cap the result at `count` points (`None` keeps them all). Upstream
/// `detail::filterIntersections` (`bodies.cpp:105`): `p.pt.isApprox(
/// intersections->back(), ZERO)`.
///
/// `Vector3d::isApprox` is Eigen `Fuzzy.h:27` —
/// `(this-other).squaredNorm() <= prec*prec *
/// numext::mini(this.squaredNorm(), other.squaredNorm())` —
/// `numext::mini` is `std::min` under another name, and the scale is the
/// **smaller** of the two squared norms, not the larger: use [`cxx_min`],
/// not `f64::min`, to reproduce it exactly (`f64::min` differs from
/// `std::min` only on NaN, but `cxx_min`'s name is the one that documents
/// which upstream function this is standing in for).
fn filter_intersections(mut candidates: Vec<Intersc>, count: Option<usize>) -> Vec<Vector3> {
    candidates.sort_by(|a, b| a.time.total_cmp(&b.time));
    let n = match count {
        Some(n) => n.min(candidates.len()),
        None => candidates.len(),
    };
    let mut out: Vec<Vector3> = Vec::with_capacity(n);
    for c in candidates {
        if out.len() == n {
            break;
        }
        if let Some(last) = out.last() {
            let diff_norm_sqr = (c.pt - last).norm_squared();
            let scale = cxx_min(last.norm_squared(), c.pt.norm_squared());
            if diff_norm_sqr <= ZERO * ZERO * scale {
                continue;
            }
        }
        out.push(c.pt);
    }
    out
}

/// A cylinder bounding a posed body. Upstream `bodies::BoundingCylinder`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundingCylinder {
    /// The bounding cylinder's pose.
    pub pose: Isometry3,
    /// The bounding cylinder's radius.
    pub radius: f64,
    /// The bounding cylinder's length.
    pub length: f64,
}

/// An axis-aligned bounding box. Upstream `bodies::AABB`, a thin extension of
/// `Eigen::AlignedBox3d` — this port carries only the subset of
/// `AlignedBox3d`'s API `bodies.cpp`/`body_operations.cpp` actually use.
///
/// An empty box (as built by [`AABB::empty`]) has `min` componentwise
/// greater than `max`, matching `Eigen::AlignedBox3d::setEmpty`'s sentinel
/// (`min = +inf`, `max = -inf` per component) rather than adding an `Option`
/// wrapper upstream's type does not have.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AABB {
    min: Vector3,
    max: Vector3,
}

impl Default for AABB {
    fn default() -> Self {
        Self::empty()
    }
}

impl AABB {
    /// An empty box: [`AABB::extend`] on it behaves as if starting from
    /// nothing. Upstream `Eigen::AlignedBox3d`'s default constructor (which
    /// upstream's `Body::computeBoundingBox` overrides call `setEmpty()`
    /// after, being explicit about depending on this exact state).
    pub fn empty() -> Self {
        Self {
            min: Vector3::from_element(f64::INFINITY),
            max: Vector3::from_element(f64::NEG_INFINITY),
        }
    }

    /// Build a box directly from its corners. Upstream
    /// `Eigen::AlignedBox3d(min, max)`, inherited via `using
    /// Eigen::AlignedBox3d::AlignedBox;`.
    pub const fn new(min: Vector3, max: Vector3) -> Self {
        Self { min, max }
    }

    /// Whether this box is empty (see [`AABB::empty`]).
    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x || self.min.y > self.max.y || self.min.z > self.max.z
    }

    /// This box's minimum corner.
    pub const fn min(&self) -> Vector3 {
        self.min
    }

    /// This box's maximum corner.
    pub const fn max(&self) -> Vector3 {
        self.max
    }

    /// This box's center. Meaningless on an empty box, matching upstream
    /// (`(min + max) / 2` with `min = +inf`, `max = -inf` is `NaN`).
    pub fn center(&self) -> Vector3 {
        (self.min + self.max) * 0.5
    }

    /// This box's extents along x, y, z. Meaningless on an empty box, for
    /// the same reason as [`AABB::center`].
    pub fn sizes(&self) -> Vector3 {
        self.max - self.min
    }

    /// Grow this box to include `point`. Upstream
    /// `Eigen::AlignedBox3d::extend(const Vector3d&)`.
    pub fn extend(&mut self, point: Vector3) {
        self.min = self.min.inf(&point);
        self.max = self.max.sup(&point);
    }

    /// Grow this box to include `other`. Upstream
    /// `Eigen::AlignedBox3d::extend(const AlignedBox3d&)`, used by
    /// [`merge_bounding_boxes`].
    pub fn extend_aabb(&mut self, other: &AABB) {
        self.min = self.min.inf(&other.min);
        self.max = self.max.sup(&other.max);
    }

    /// Grow this box to include an oriented `box_size`-by-`box_size`-by-
    /// `box_size` box (full extents, not half) posed at `transform`.
    /// Upstream `AABB::extendWithTransformedBox`, which delegates to FCL's
    /// `computeBV<AABBd>(Boxd, transform, aabb)`. That function's formula —
    /// each world-axis half-extent is the row-sum of the absolute values of
    /// the rotation matrix times the local half-extents — is the standard,
    /// well-known "AABB of a rotated box" identity (not FCL-specific
    /// behavior; safe to reimplement without FCL's source), and this port's
    /// output matches upstream's literal test values
    /// (`test_bounding_box.cpp`'s rotated-box cases) to the tests'
    /// `1e-4` tolerance — see this module's tests.
    pub fn extend_with_transformed_box(&mut self, transform: &Isometry3, box_size: Vector3) {
        let half = box_size * 0.5;
        let abs_rot = transform.rotation.to_rotation_matrix().matrix().abs();
        let world_half = abs_rot * half;
        let center = transform.translation.vector;
        self.extend(center - world_half);
        self.extend(center + world_half);
    }

    /// Whether `point` lies in or on this box.
    pub fn contains(&self, point: &Vector3) -> bool {
        point.x >= self.min.x
            && point.x <= self.max.x
            && point.y >= self.min.y
            && point.y <= self.max.y
            && point.z >= self.min.z
            && point.z <= self.max.z
    }
}

/// An oriented bounding box. Upstream `bodies::OBB`, a PIMPL wrapper around
/// FCL's `fcl::OBB<double>` — see the module docs ("Design" and "Deviations
/// from upstream" 3) for which of this type's methods are a literal port and
/// which are this port's own implementation of the documented behavior.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OBB {
    pose: Isometry3,
    half_extents: Vector3,
}

impl Default for OBB {
    /// Position zero, zero extents, identity orientation. Upstream `OBB()`.
    fn default() -> Self {
        Self {
            pose: Isometry3::identity(),
            half_extents: Vector3::zeros(),
        }
    }
}

impl OBB {
    /// Build an OBB from its pose and full (not half) extents. Upstream
    /// `OBB(pose, extents)`.
    pub fn new(pose: Isometry3, extents: Vector3) -> Self {
        let mut obb = Self::default();
        obb.set_pose_and_extents(pose, extents);
        obb
    }

    /// Set this OBB's pose and full extents. Upstream `setPoseAndExtents`.
    pub fn set_pose_and_extents(&mut self, pose: Isometry3, extents: Vector3) {
        self.pose = pose;
        self.half_extents = extents * 0.5;
    }

    /// This OBB's full (not half) extents. Upstream `getExtents`.
    pub fn extents(&self) -> Vector3 {
        self.half_extents * 2.0
    }

    /// This OBB's pose.
    pub const fn pose(&self) -> Isometry3 {
        self.pose
    }

    /// The axis-aligned box bounding this OBB. Upstream `toAABB`.
    pub fn to_aabb(&self) -> AABB {
        let mut aabb = AABB::empty();
        aabb.extend_with_transformed_box(&self.pose, self.extents());
        aabb
    }

    /// This OBB's 8 vertices, in world coordinates. Upstream
    /// `computeVertices`.
    pub fn compute_vertices(&self) -> Vec<Vector3> {
        let e = self.half_extents;
        [
            Vector3::new(-e.x, -e.y, -e.z),
            Vector3::new(-e.x, -e.y, e.z),
            Vector3::new(-e.x, e.y, -e.z),
            Vector3::new(-e.x, e.y, e.z),
            Vector3::new(e.x, -e.y, -e.z),
            Vector3::new(e.x, -e.y, e.z),
            Vector3::new(e.x, e.y, -e.z),
            Vector3::new(e.x, e.y, e.z),
        ]
        .into_iter()
        .map(|v| transform_point(&self.pose, &v))
        .collect()
    }

    /// Whether `point` lies in or on this OBB. See the module docs,
    /// deviation 3, for why this is this port's own implementation.
    pub fn contains_point(&self, point: &Vector3) -> bool {
        let local = self.pose.inverse_transform_point(&(*point).into());
        local.x.abs() <= self.half_extents.x
            && local.y.abs() <= self.half_extents.y
            && local.z.abs() <= self.half_extents.z
    }

    /// Whether this OBB wholly contains `other` (every vertex of `other`
    /// lies in or on this OBB). Upstream `contains(const OBB&)`.
    pub fn contains_obb(&self, other: &OBB) -> bool {
        other
            .compute_vertices()
            .iter()
            .all(|v| self.contains_point(v))
    }

    /// Whether this and `other` have nonempty intersection. See the module
    /// docs, deviation 3, for why this is this port's own implementation
    /// (the standard 15-axis SAT test for two oriented boxes, not FCL's
    /// `OBB::overlap`).
    pub fn overlaps(&self, other: &OBB) -> bool {
        // Ericson, Real-Time Collision Detection §4.4.1. `ra`/`rb` are this
        // and `other`'s rotation matrices (columns are their local axes);
        // `r`/`abs_r` are `other`'s axes expressed in this box's frame.
        const EPS: f64 = 1e-9;
        let ra = self.pose.rotation.to_rotation_matrix();
        let rb = other.pose.rotation.to_rotation_matrix();
        let r = ra.matrix().transpose() * rb.matrix();
        let abs_r = r.map(|v| v.abs() + EPS);

        let t_world = other.pose.translation.vector - self.pose.translation.vector;
        let t = ra.matrix().transpose() * t_world;

        let ea = self.half_extents;
        let eb = other.half_extents;

        // This box's 3 axes.
        for i in 0..3 {
            let ra_i = ea[i];
            let rb_i = eb[0] * abs_r[(i, 0)] + eb[1] * abs_r[(i, 1)] + eb[2] * abs_r[(i, 2)];
            if t[i].abs() > ra_i + rb_i {
                return false;
            }
        }

        // Other box's 3 axes.
        for j in 0..3 {
            let ra_j = ea[0] * abs_r[(0, j)] + ea[1] * abs_r[(1, j)] + ea[2] * abs_r[(2, j)];
            let rb_j = eb[j];
            let t_j = t[0] * r[(0, j)] + t[1] * r[(1, j)] + t[2] * r[(2, j)];
            if t_j.abs() > ra_j + rb_j {
                return false;
            }
        }

        // 9 cross-product axes, Ai x Bj.
        let cases: [(usize, usize); 9] = [
            (0, 0),
            (0, 1),
            (0, 2),
            (1, 0),
            (1, 1),
            (1, 2),
            (2, 0),
            (2, 1),
            (2, 2),
        ];
        for (i, j) in cases {
            let (i1, i2) = ((i + 1) % 3, (i + 2) % 3);
            let ra_ij = ea[i1] * abs_r[(i2, j)] + ea[i2] * abs_r[(i1, j)];
            let rb_ij = eb[(j + 1) % 3] * abs_r[(i, (j + 2) % 3)]
                + eb[(j + 2) % 3] * abs_r[(i, (j + 1) % 3)];
            let t_ij = t[i2] * r[(i1, j)] - t[i1] * r[(i2, j)];
            if t_ij.abs() > ra_ij + rb_ij {
                return false;
            }
        }

        true
    }

    /// Grow this OBB to (approximately) enclose `other`. Upstream
    /// `extendApprox`; see the module docs, deviation 3, for the general
    /// case's behavior.
    pub fn extend_approx(&mut self, other: &OBB) {
        if self.half_extents == Vector3::zeros() {
            *self = *other;
            return;
        }
        if self.contains_obb(other) {
            return;
        }
        if other.contains_obb(self) {
            *self = *other;
            return;
        }

        // Neither box contains the other: this is FCL's `OBB::operator+=`
        // (`obb.cpp`'s `extendApprox` calls `*this->obb_ += *box.obb_`) —
        // ported literally from `OBB-inl.h`'s `operator+`, which dispatches
        // on how far apart the two boxes' centers are. See the module
        // docs, deviation 3, and the provenance comment at the top of this
        // file for where the FCL source came from.
        //
        // `max_extent`/`other_max_extent` use `cxx_max`, not `.max()`:
        // upstream is `std::max(std::max(extent[0], extent[1]), extent[2])`
        // (`OBB-inl.h:164-165`), which propagates a NaN `extent[0]`
        // (`.x`) out through both calls but silently discards a NaN
        // `extent[1]`/`extent[2]`. `.x.max(y).max(z)` discards NaN
        // regardless of position, so it can pick a different
        // `merge_largedist`/`merge_smalldist` branch than upstream's
        // NaN-forced `merge_smalldist` for the same input — see
        // `crate::geometry::numeric`.
        let max_extent = cxx_max(
            cxx_max(self.half_extents.x, self.half_extents.y),
            self.half_extents.z,
        );
        let other_max_extent = cxx_max(
            cxx_max(other.half_extents.x, other.half_extents.y),
            other.half_extents.z,
        );
        let center_diff = self.pose.translation.vector - other.pose.translation.vector;
        *self = if center_diff.norm() > 2.0 * (max_extent + other_max_extent) {
            merge_largedist(self, other)
        } else {
            merge_smalldist(self, other)
        };
    }
}

/// FCL's `OBB::operator+`, `center_diff.norm() <= 2*(max_extent +
/// max_extent2)` branch: arithmetic-mean center, hemisphere-corrected
/// quaternion-average orientation, then the componentwise min/max of both
/// boxes' vertices projected onto the merged axes. `OBB-inl.h`'s
/// `merge_smalldist`; the only caller is [`OBB::extend_approx`].
fn merge_smalldist(a: &OBB, b: &OBB) -> OBB {
    let center = (a.pose.translation.vector + b.pose.translation.vector) * 0.5;

    let q0 = a.pose.rotation;
    let q1 = b.pose.rotation;
    let q1_coords = if q0.coords.dot(&q1.coords) < 0.0 {
        -q1.coords
    } else {
        q1.coords
    };
    let merged_rotation = nalgebra::UnitQuaternion::from_quaternion(
        nalgebra::Quaternion::from_vector(q0.coords + q1_coords),
    );
    let axis = merged_rotation.to_rotation_matrix();
    let axis_cols = [
        axis.matrix().column(0).into_owned(),
        axis.matrix().column(1).into_owned(),
        axis.matrix().column(2).into_owned(),
    ];

    let mut pmin = Vector3::from_element(f64::MAX);
    let mut pmax = Vector3::from_element(-f64::MAX);
    for obb in [a, b] {
        for v in obb.compute_vertices() {
            let diff = v - center;
            for (j, axis_j) in axis_cols.iter().enumerate() {
                let dot = diff.dot(axis_j);
                if dot > pmax[j] {
                    pmax[j] = dot;
                }
                if dot < pmin[j] {
                    pmin[j] = dot;
                }
            }
        }
    }

    let mut new_center = center;
    let mut new_half_extents = Vector3::zeros();
    for (j, axis_j) in axis_cols.iter().enumerate() {
        new_center += axis_j * (0.5 * (pmax[j] + pmin[j]));
        new_half_extents[j] = 0.5 * (pmax[j] - pmin[j]);
    }

    OBB {
        pose: Isometry3::from_parts(new_center.into(), merged_rotation),
        half_extents: new_half_extents,
    }
}

/// `Vector3::normalize()`, guarded against a zero-norm input to match
/// Eigen. Upstream `OBB-inl.h:263-264` is `b.axis.col(0) = b1.To - b2.To;
/// b.axis.col(0).normalize();` — Eigen's in-place `normalize()` (and its
/// `normalized()`) are guarded: `if (z > 0) *this /= sqrt(z); ` else the
/// vector is left unchanged. nalgebra's `.normalize()` is
/// `self.unscale(self.norm())`, unguarded, and gives `[NaN, NaN, NaN]` for
/// a zero vector (confirmed empirically: `Vector3::zeros().normalize() ==
/// [NaN, NaN, NaN]`, since `0.0 / 0.0` is NaN).
///
/// `v.try_normalize(0.0)` returns `None` exactly when `v.norm() <= 0.0`
/// (i.e. `v` is the zero vector) and `Some(unit vector)` otherwise;
/// `.unwrap_or(v)` on `None` reproduces Eigen's "leave unchanged" branch
/// precisely. `.unwrap_or_else(Vector3::zeros)` would NOT reproduce it: it
/// assumes `v` itself is the zero vector whenever normalization fails, but
/// the only failure case here *is* `v` already being the zero vector, so
/// the two spellings coincide today — the distinction matters the moment
/// this helper gains a second caller whose input can be zero for a
/// different reason.
fn guarded_normalize(v: Vector3) -> Vector3 {
    v.try_normalize(0.0).unwrap_or(v)
}

/// FCL's `OBB::operator+`, `center_diff.norm() > 2*(max_extent +
/// max_extent2)` branch: a PCA-fit box over both boxes' 16 vertices
/// projected onto the plane perpendicular to the center-to-center axis.
/// `OBB-inl.h`'s `merge_largedist`; the only caller is
/// [`OBB::extend_approx`].
fn merge_largedist(a: &OBB, b: &OBB) -> OBB {
    let vertices: Vec<Vector3> = a
        .compute_vertices()
        .into_iter()
        .chain(b.compute_vertices())
        .collect();

    // `guarded_normalize`, not `.normalize()`: nothing in this crate's
    // public API rejects a negative `half_extents` component
    // (`OBB::new`/`set_pose_and_extents` just multiply by 0.5), and a box
    // built with all-negative half-extents has a negative `max_extent`
    // (see `OBB::extend_approx`) — negative enough that
    // `center_diff.norm() > 2*(max_extent+other_max_extent)` can hold even
    // when the two centers are identical, forcing this branch with a
    // genuinely zero `a.To - b.To`. See `guarded_normalize`'s doc for why
    // upstream doesn't produce NaN there and this port, unguarded, did.
    let axis0 = guarded_normalize(a.pose.translation.vector - b.pose.translation.vector);
    let projected: Vec<Vector3> = vertices.iter().map(|v| v - axis0 * v.dot(&axis0)).collect();

    let cov = fcl_covariance(&projected);
    let (eigenvalues, eigenvectors) = fcl_eigen_old(&cov);
    let (_min, mid, max) = min_mid_max(&eigenvalues);

    let axis = nalgebra::Matrix3::from_columns(&[axis0, eigenvectors[max], eigenvectors[mid]]);
    let (center, half_extents) = fcl_extent_and_center(&vertices, &axis);
    let rotation = nalgebra::UnitQuaternion::from_rotation_matrix(
        &nalgebra::Rotation3::from_matrix_unchecked(axis),
    );

    OBB {
        pose: Isometry3::from_parts(center.into(), rotation),
        half_extents,
    }
}

/// The `min`/`mid`/`max` eigenvalue-index selection shared by
/// [`merge_largedist`] and FCL's own `axisFromEigen`. `geometry-inl.h`.
fn min_mid_max(s: &Vector3) -> (usize, usize, usize) {
    let (mut min, mut max);
    if s[0] > s[1] {
        max = 0;
        min = 1;
    } else {
        min = 0;
        max = 1;
    }
    let mid;
    if s[2] < s[min] {
        mid = min;
        min = 2;
    } else if s[2] > s[max] {
        mid = max;
        max = 2;
    } else {
        mid = 2;
    }
    (min, mid, max)
}

/// FCL's `getCovariance` (`geometry-inl.h`), specialized to the
/// point-cloud, single-frame case (`ts == nullptr`, `ps2 == nullptr`,
/// `indices == nullptr`) — the only case [`merge_largedist`] ever calls it
/// with.
fn fcl_covariance(points: &[Vector3]) -> nalgebra::Matrix3<f64> {
    let mut s1 = Vector3::zeros();
    let (mut s2_00, mut s2_11, mut s2_22, mut s2_01, mut s2_02, mut s2_12) =
        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for p in points {
        s1 += p;
        s2_00 += p.x * p.x;
        s2_11 += p.y * p.y;
        s2_22 += p.z * p.z;
        s2_01 += p.x * p.y;
        s2_02 += p.x * p.z;
        s2_12 += p.y * p.z;
    }
    let n_points = points.len() as f64;
    let m00 = s2_00 - s1.x * s1.x / n_points;
    let m11 = s2_11 - s1.y * s1.y / n_points;
    let m22 = s2_22 - s1.z * s1.z / n_points;
    let m01 = s2_01 - s1.x * s1.y / n_points;
    let m12 = s2_12 - s1.y * s1.z / n_points;
    let m02 = s2_02 - s1.x * s1.z / n_points;
    nalgebra::Matrix3::new(m00, m01, m02, m01, m11, m12, m02, m12, m22)
}

/// FCL's `getExtentAndCenter` (`geometry-inl.h`), specialized to the
/// point-cloud, single-frame case — the only case [`merge_largedist`]
/// ever calls it with.
fn fcl_extent_and_center(points: &[Vector3], axis: &nalgebra::Matrix3<f64>) -> (Vector3, Vector3) {
    let mut min_coord = Vector3::from_element(f64::MAX);
    let mut max_coord = Vector3::from_element(-f64::MAX);
    for p in points {
        let proj = Vector3::new(
            axis.column(0).dot(p),
            axis.column(1).dot(p),
            axis.column(2).dot(p),
        );
        for j in 0..3 {
            if proj[j] > max_coord[j] {
                max_coord[j] = proj[j];
            }
            if proj[j] < min_coord[j] {
                min_coord[j] = proj[j];
            }
        }
    }
    let o = (max_coord + min_coord) * 0.5;
    let center = axis * o;
    let extent = (max_coord - min_coord) * 0.5;
    (center, extent)
}

/// FCL's `eigen_old` (`geometry-inl.h`): a fixed 50-sweep classical Jacobi
/// eigenvalue algorithm for a symmetric 3x3 matrix. Returns the
/// eigenvalues and, for each, its eigenvector (`eigenvectors[k]`
/// corresponds to `eigenvalues[k]`) — [`merge_largedist`] is the only
/// caller.
fn fcl_eigen_old(m: &nalgebra::Matrix3<f64>) -> (Vector3, [Vector3; 3]) {
    fn assemble(d: [f64; 3], v: [[f64; 3]; 3]) -> (Vector3, [Vector3; 3]) {
        let eigenvectors = [
            Vector3::new(v[0][0], v[1][0], v[2][0]),
            Vector3::new(v[0][1], v[1][1], v[2][1]),
            Vector3::new(v[0][2], v[1][2], v[2][2]),
        ];
        (Vector3::new(d[0], d[1], d[2]), eigenvectors)
    }

    let mut r = *m;
    let mut v = [[0.0_f64; 3]; 3];
    for (i, row) in v.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    let mut b = [0.0_f64; 3];
    let mut d = [0.0_f64; 3];
    let mut z = [0.0_f64; 3];
    for ip in 0..3 {
        b[ip] = r[(ip, ip)];
        d[ip] = r[(ip, ip)];
    }

    for sweep in 0..50 {
        let mut sm = 0.0;
        for ip in 0..3 {
            for iq in (ip + 1)..3 {
                sm += r[(ip, iq)].abs();
            }
        }
        if sm == 0.0 {
            return assemble(d, v);
        }

        let tresh = if sweep < 3 { 0.2 * sm / 9.0 } else { 0.0 };

        for ip in 0..3 {
            for iq in (ip + 1)..3 {
                let g = 100.0 * r[(ip, iq)].abs();
                if sweep > 3 && d[ip].abs() + g == d[ip].abs() && d[iq].abs() + g == d[iq].abs() {
                    r[(ip, iq)] = 0.0;
                    continue;
                }
                if r[(ip, iq)].abs() <= tresh {
                    continue;
                }
                let h = d[iq] - d[ip];
                let t = if h.abs() + g == h.abs() {
                    r[(ip, iq)] / h
                } else {
                    let theta = 0.5 * h / r[(ip, iq)];
                    let t = 1.0 / (theta.abs() + (1.0 + theta * theta).sqrt());
                    if theta < 0.0 { -t } else { t }
                };
                let c = 1.0 / (1.0 + t * t).sqrt();
                let s = t * c;
                let tau = s / (1.0 + c);
                let h = t * r[(ip, iq)];
                z[ip] -= h;
                z[iq] += h;
                d[ip] -= h;
                d[iq] += h;
                r[(ip, iq)] = 0.0;
                for j in 0..ip {
                    let g = r[(j, ip)];
                    let h = r[(j, iq)];
                    r[(j, ip)] = g - s * (h + g * tau);
                    r[(j, iq)] = h + s * (g - h * tau);
                }
                for j in (ip + 1)..iq {
                    let g = r[(ip, j)];
                    let h = r[(j, iq)];
                    r[(ip, j)] = g - s * (h + g * tau);
                    r[(j, iq)] = h + s * (g - h * tau);
                }
                for j in (iq + 1)..3 {
                    let g = r[(ip, j)];
                    let h = r[(iq, j)];
                    r[(ip, j)] = g - s * (h + g * tau);
                    r[(iq, j)] = h + s * (g - h * tau);
                }
                for row in &mut v {
                    let g = row[ip];
                    let h = row[iq];
                    row[ip] = g - s * (h + g * tau);
                    row[iq] = h + s * (g - h * tau);
                }
            }
        }
        for ip in 0..3 {
            b[ip] += z[ip];
            d[ip] = b[ip];
            z[ip] = 0.0;
        }
    }

    assemble(d, v)
}

/// Merge several bounding spheres into one that contains them all. Upstream
/// `mergeBoundingSpheres`.
///
/// Spheres with non-positive radius are skipped (upstream `if
/// (spheres[i].radius <= 0.0) continue;`) — after the first sphere seeds
/// `mergedSphere`, a later degenerate sphere cannot shrink or move it.
pub fn merge_bounding_spheres(spheres: &[BoundingSphere]) -> BoundingSphere {
    let Some((first, rest)) = spheres.split_first() else {
        return BoundingSphere {
            center: Vector3::zeros(),
            radius: 0.0,
        };
    };
    let mut merged = *first;
    for s in rest {
        if s.radius <= 0.0 {
            continue;
        }
        let diff = s.center - merged.center;
        let d = diff.norm();
        if d + merged.radius <= s.radius {
            merged = *s;
        } else if d + s.radius > merged.radius {
            let delta = merged.center - s.center;
            let delta_norm = delta.norm();
            merged.radius = (delta_norm + s.radius + merged.radius) * 0.5;
            let dir = if delta_norm > 0.0 {
                delta / delta_norm
            } else {
                Vector3::zeros()
            };
            merged.center = dir * (merged.radius - s.radius) + s.center;
        }
    }
    merged
}

/// Merge several axis-aligned boxes into one that contains them all.
/// Upstream `mergeBoundingBoxes`.
pub fn merge_bounding_boxes(boxes: &[AABB]) -> AABB {
    let mut merged = AABB::empty();
    for b in boxes {
        merged.extend_aabb(b);
    }
    merged
}

/// Merge several oriented boxes into one that approximately contains them
/// all. Upstream `mergeBoundingBoxesApprox`.
pub fn merge_bounding_boxes_approx(boxes: &[OBB]) -> OBB {
    let mut merged = OBB::default();
    for b in boxes {
        merged.extend_approx(b);
    }
    merged
}

/// A sphere body: a posed, scaled, padded [`crate::geometry::shapes::Sphere`].
/// Upstream `bodies::Sphere`.
///
/// Fields are split into shape-dependent (`radius`) and
/// pose/scale/padding-dependent cached fields, matching `bodies.h`'s own
/// grouping comments — see the module docs, "Design: cached derived
/// fields".
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Sphere {
    radius: f64,
    pose: Isometry3,
    scale: f64,
    padding: f64,
    // cached
    radius_scaled: f64,
    radius_scaled_sqr: f64,
    center: Vector3,
}

impl Sphere {
    /// Build a sphere body from a raw radius, identity pose, scale 1.0, no
    /// padding. Upstream `Sphere(shape)` immediately followed by
    /// `setDimensions`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when `radius` is negative.
    pub fn new(radius: f64) -> Result<Self> {
        let mut s = Self {
            radius: 0.0,
            pose: Isometry3::identity(),
            scale: 1.0,
            padding: 0.0,
            radius_scaled: 0.0,
            radius_scaled_sqr: 0.0,
            center: Vector3::zeros(),
        };
        s.set_dimensions(radius)?;
        Ok(s)
    }

    /// Build a sphere body directly from a bounding sphere. Upstream
    /// `explicit Sphere(const BoundingSphere&)`.
    pub fn from_bounding_sphere(sphere: &BoundingSphere) -> Result<Self> {
        let mut s = Self::new(sphere.radius)?;
        s.set_pose(Isometry3::translation(
            sphere.center.x,
            sphere.center.y,
            sphere.center.z,
        ));
        Ok(s)
    }

    /// This body's raw (unscaled, unpadded) radius. Upstream
    /// `getDimensions`.
    pub fn dimensions(&self) -> Vec<f64> {
        vec![self.radius]
    }

    /// This body's scaled and padded radius. Upstream `getScaledDimensions`.
    pub fn scaled_dimensions(&self) -> Vec<f64> {
        vec![self.radius_scaled]
    }

    /// This body's pose.
    pub const fn pose(&self) -> Isometry3 {
        self.pose
    }

    /// This body's scale factor.
    pub const fn scale(&self) -> f64 {
        self.scale
    }

    /// This body's padding.
    pub const fn padding(&self) -> f64 {
        self.padding
    }

    /// Set this body's raw (unscaled, unpadded) radius. Upstream
    /// `setDimensions`/`useDimensions`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when the resulting scaled radius would be
    /// negative.
    pub fn set_dimensions(&mut self, radius: f64) -> Result<()> {
        let radius_scaled = radius * self.scale + self.padding;
        if radius_scaled < 0.0 {
            return Err(Error::construct("Sphere radius must be non-negative."));
        }
        self.radius = radius;
        self.radius_scaled = radius_scaled;
        self.radius_scaled_sqr = radius_scaled * radius_scaled;
        Ok(())
    }

    /// Set this body's pose. Upstream `setPose`.
    pub fn set_pose(&mut self, pose: Isometry3) {
        self.pose = pose;
        self.center = pose.translation.vector;
    }

    /// Set this body's scale factor. Upstream `setScale`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when the resulting scaled radius would be
    /// negative. On error, this body is left unchanged.
    pub fn set_scale(&mut self, scale: f64) -> Result<()> {
        let radius_scaled = self.radius * scale + self.padding;
        if radius_scaled < 0.0 {
            return Err(Error::construct("Sphere radius must be non-negative."));
        }
        self.scale = scale;
        self.radius_scaled = radius_scaled;
        self.radius_scaled_sqr = radius_scaled * radius_scaled;
        Ok(())
    }

    /// Set this body's padding. Upstream `setPadding`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when the resulting scaled radius would be
    /// negative. On error, this body is left unchanged.
    pub fn set_padding(&mut self, padding: f64) -> Result<()> {
        let radius_scaled = self.radius * self.scale + padding;
        if radius_scaled < 0.0 {
            return Err(Error::construct("Sphere radius must be non-negative."));
        }
        self.padding = padding;
        self.radius_scaled = radius_scaled;
        self.radius_scaled_sqr = radius_scaled * radius_scaled;
        Ok(())
    }

    /// Whether `p` lies in or on this sphere. Upstream `containsPoint`.
    pub fn contains_point(&self, p: &Vector3) -> bool {
        (self.center - p).norm_squared() <= self.radius_scaled_sqr
    }

    /// This body's volume. Upstream `computeVolume`.
    pub fn compute_volume(&self) -> f64 {
        4.0 / 3.0 * std::f64::consts::PI * self.radius_scaled.powi(3)
    }

    /// This body's bounding sphere (itself). Upstream `computeBoundingSphere`.
    pub fn compute_bounding_sphere(&self) -> BoundingSphere {
        BoundingSphere {
            center: self.center,
            radius: self.radius_scaled,
        }
    }

    /// This body's bounding cylinder. Upstream `computeBoundingCylinder`.
    pub fn compute_bounding_cylinder(&self) -> BoundingCylinder {
        BoundingCylinder {
            pose: self.pose,
            radius: self.radius_scaled,
            length: 2.0 * self.radius_scaled,
        }
    }

    /// This body's axis-aligned bounding box. Upstream
    /// `computeBoundingBox(AABB&)`. A sphere is never rotated for this
    /// purpose, matching upstream's explicit comment.
    pub fn compute_bounding_aabb(&self) -> AABB {
        let mut bbox = AABB::empty();
        let transform = Isometry3::translation(self.center.x, self.center.y, self.center.z);
        let d = 2.0 * self.radius_scaled;
        bbox.extend_with_transformed_box(&transform, Vector3::new(d, d, d));
        bbox
    }

    /// This body's oriented bounding box. Upstream
    /// `computeBoundingBox(OBB&)`.
    pub fn compute_bounding_obb(&self) -> OBB {
        let transform = Isometry3::translation(self.center.x, self.center.y, self.center.z);
        OBB::new(
            transform,
            2.0 * Vector3::new(self.radius_scaled, self.radius_scaled, self.radius_scaled),
        )
    }

    /// Every intersection of the ray (through `origin`, along `dir`) with
    /// this body, ordered along the ray and capped at `count` points
    /// (`None` for unlimited). Upstream `intersectsRay`; see the module
    /// docs, deviation 1.
    pub fn ray_intersections(
        &self,
        origin: &Vector3,
        dir: &Vector3,
        count: Option<usize>,
    ) -> Vec<Vector3> {
        let dir_norm = normalize_dir(dir);
        if distance_sqr(&self.center, origin, &dir_norm) > self.radius_scaled_sqr {
            return Vec::new();
        }

        let cp = origin - self.center;
        let dpcpv = cp.dot(&dir_norm);
        let w = cp - dpcpv * dir_norm;
        let q = self.center + w;
        let x = self.radius_scaled_sqr - w.norm_squared();

        let mut out = Vec::new();
        if x.abs() < ZERO {
            let w = q - origin;
            let dp_qv = w.dot(&dir_norm);
            if dp_qv > ZERO {
                out.push(q);
            }
        } else if x > 0.0 {
            let x = x.sqrt();
            let w = dir_norm * x;
            let a = q - w;
            let b = q + w;
            let dp_av = (a - origin).dot(&dir_norm);
            let dp_bv = (b - origin).dot(&dir_norm);

            if dp_av > ZERO {
                out.push(a);
                if count == Some(1) {
                    return out;
                }
            }
            if dp_bv > ZERO {
                out.push(b);
            }
        }
        out
    }

    /// Whether the ray (through `origin`, along `dir`) intersects this
    /// body. Upstream `intersectsRay` called with a null `intersections`
    /// out-param; see the module docs, deviation 1.
    pub fn intersects_ray(&self, origin: &Vector3, dir: &Vector3) -> bool {
        !self.ray_intersections(origin, dir, Some(1)).is_empty()
    }

    /// Sample a point inside this body, trying up to `max_attempts * 20`
    /// times (see upstream's own comment: with 20 inner tries, the failure
    /// probability of the enclosing-box rejection sampler is under
    /// `0.00004%`). Upstream `Sphere::samplePointInside` — note this
    /// overrides the generic `Body::samplePointInside` with a different
    /// (nested-loop) structure; see the module docs, deviation 4, for why
    /// this takes a closure instead of a `random_numbers::RandomNumberGenerator`.
    pub fn sample_point_inside(
        &self,
        max_attempts: u32,
        uniform: &mut dyn FnMut(f64, f64) -> f64,
    ) -> Option<Vector3> {
        let min = self.center - Vector3::from_element(self.radius_scaled);
        let max = self.center + Vector3::from_element(self.radius_scaled);
        for _ in 0..max_attempts {
            for _ in 0..20 {
                let candidate = Vector3::new(
                    uniform(min.x, max.x),
                    uniform(min.y, max.y),
                    uniform(min.z, max.z),
                );
                if self.contains_point(&candidate) {
                    return Some(candidate);
                }
            }
        }
        None
    }

    /// Clone this body at a new pose, keeping this body's padding and
    /// scale. Upstream `Body::cloneAt(pose)`.
    pub fn clone_at(&self, pose: Isometry3) -> Self {
        self.clone_at_with(pose, self.padding, self.scale)
            .expect("keeping this body's own valid padding and scale cannot fail")
    }

    /// Clone this body at a new pose, padding and scale. Upstream
    /// `Sphere::cloneAt(pose, padding, scale)`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] under the same condition as [`Sphere::new`].
    pub fn clone_at_with(&self, pose: Isometry3, padding: f64, scale: f64) -> Result<Self> {
        let mut s = Self::new(self.radius)?;
        s.padding = padding;
        s.scale = scale;
        s.set_dimensions(self.radius)?;
        s.set_pose(pose);
        Ok(s)
    }
}

/// A cylinder body: a posed, scaled, padded [`crate::geometry::shapes::Cylinder`].
/// Upstream `bodies::Cylinder`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cylinder {
    radius: f64,
    length: f64,
    pose: Isometry3,
    scale: f64,
    padding: f64,
    // cached
    center: Vector3,
    normal_h: Vector3,
    normal_b1: Vector3,
    normal_b2: Vector3,
    half_length: f64,
    radius_scaled: f64,
    radius_scaled_sqr: f64,
    radius_bounding: f64,
    radius_bounding_sqr: f64,
    d1: f64,
    d2: f64,
}

impl Cylinder {
    /// Build a cylinder body from raw dimensions, identity pose, scale 1.0,
    /// no padding.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when either dimension is negative.
    pub fn new(radius: f64, length: f64) -> Result<Self> {
        let mut c = Self {
            radius: 0.0,
            length: 0.0,
            pose: Isometry3::identity(),
            scale: 1.0,
            padding: 0.0,
            center: Vector3::zeros(),
            normal_h: Vector3::z(),
            normal_b1: Vector3::x(),
            normal_b2: Vector3::y(),
            half_length: 0.0,
            radius_scaled: 0.0,
            radius_scaled_sqr: 0.0,
            radius_bounding: 0.0,
            radius_bounding_sqr: 0.0,
            d1: 0.0,
            d2: 0.0,
        };
        c.set_dimensions(radius, length)?;
        Ok(c)
    }

    /// Build a cylinder body directly from a bounding cylinder. Upstream
    /// `explicit Cylinder(const BoundingCylinder&)`.
    pub fn from_bounding_cylinder(cylinder: &BoundingCylinder) -> Result<Self> {
        let mut c = Self::new(cylinder.radius, cylinder.length)?;
        c.set_pose(cylinder.pose);
        Ok(c)
    }

    fn recompute(&mut self, radius: f64, length: f64, scale: f64, padding: f64) -> Result<()> {
        let radius_scaled = radius * scale + padding;
        if radius_scaled < 0.0 {
            return Err(Error::construct("Cylinder radius must be non-negative."));
        }
        let half_length = scale * length / 2.0 + padding;
        if half_length < 0.0 {
            return Err(Error::construct("Cylinder length must be non-negative."));
        }
        self.radius = radius;
        self.length = length;
        self.scale = scale;
        self.padding = padding;
        self.radius_scaled = radius_scaled;
        self.radius_scaled_sqr = radius_scaled * radius_scaled;
        self.half_length = half_length;
        self.center = self.pose.translation.vector;
        self.radius_bounding_sqr = half_length * half_length + self.radius_scaled_sqr;
        self.radius_bounding = self.radius_bounding_sqr.sqrt();

        let basis = self.pose.rotation.to_rotation_matrix();
        self.normal_b1 = basis.matrix().column(0).into();
        self.normal_b2 = basis.matrix().column(1).into();
        self.normal_h = basis.matrix().column(2).into();

        let tmp = -self.normal_h.dot(&self.center);
        self.d1 = tmp + self.half_length;
        self.d2 = tmp - self.half_length;
        Ok(())
    }

    /// This body's raw (unscaled, unpadded) radius and length, in that
    /// order. Upstream `getDimensions`.
    pub fn dimensions(&self) -> Vec<f64> {
        vec![self.radius, self.length]
    }

    /// This body's scaled and padded radius and length, in that order.
    /// Upstream `getScaledDimensions`.
    pub fn scaled_dimensions(&self) -> Vec<f64> {
        vec![self.radius_scaled, 2.0 * self.half_length]
    }

    /// This body's pose.
    pub const fn pose(&self) -> Isometry3 {
        self.pose
    }

    /// This body's scale factor.
    pub const fn scale(&self) -> f64 {
        self.scale
    }

    /// This body's padding.
    pub const fn padding(&self) -> f64 {
        self.padding
    }

    /// Set this body's raw (unscaled, unpadded) radius and length. Upstream
    /// `setDimensions`/`useDimensions`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when a resulting scaled dimension would be
    /// negative. On error, this body is left unchanged.
    pub fn set_dimensions(&mut self, radius: f64, length: f64) -> Result<()> {
        self.recompute(radius, length, self.scale, self.padding)
    }

    /// Set this body's pose. Upstream `setPose`.
    pub fn set_pose(&mut self, pose: Isometry3) {
        self.pose = pose;
        self.recompute(self.radius, self.length, self.scale, self.padding)
            .expect("pose change alone cannot invalidate an already-valid cylinder body");
    }

    /// Set this body's scale factor. Upstream `setScale`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when a resulting scaled dimension would be
    /// negative. On error, this body is left unchanged.
    pub fn set_scale(&mut self, scale: f64) -> Result<()> {
        self.recompute(self.radius, self.length, scale, self.padding)
    }

    /// Set this body's padding. Upstream `setPadding`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when a resulting scaled dimension would be
    /// negative. On error, this body is left unchanged.
    pub fn set_padding(&mut self, padding: f64) -> Result<()> {
        self.recompute(self.radius, self.length, self.scale, padding)
    }

    /// Whether `p` lies in or on this cylinder. Upstream `containsPoint`.
    pub fn contains_point(&self, p: &Vector3) -> bool {
        let v = p - self.center;
        let p_h = v.dot(&self.normal_h);
        if p_h.abs() > self.half_length {
            return false;
        }
        let p_b1 = v.dot(&self.normal_b1);
        let remaining = self.radius_scaled_sqr - p_b1 * p_b1;
        if remaining < 0.0 {
            return false;
        }
        let p_b2 = v.dot(&self.normal_b2);
        p_b2 * p_b2 <= remaining
    }

    /// This body's volume. Upstream `computeVolume`.
    pub fn compute_volume(&self) -> f64 {
        2.0 * std::f64::consts::PI * self.radius_scaled_sqr * self.half_length
    }

    /// This body's bounding sphere. Upstream `computeBoundingSphere`.
    pub fn compute_bounding_sphere(&self) -> BoundingSphere {
        BoundingSphere {
            center: self.center,
            radius: self.radius_bounding,
        }
    }

    /// This body's bounding cylinder (itself). Upstream
    /// `computeBoundingCylinder`.
    pub fn compute_bounding_cylinder(&self) -> BoundingCylinder {
        BoundingCylinder {
            pose: self.pose,
            radius: self.radius_scaled,
            length: 2.0 * self.half_length,
        }
    }

    /// This body's axis-aligned bounding box. Upstream
    /// `computeBoundingBox(AABB&)`, via the disk-bounding-box method
    /// (<http://www.iquilezles.org/www/articles/diskbbox/diskbbox.htm>).
    pub fn compute_bounding_aabb(&self) -> AABB {
        let mut bbox = AABB::empty();
        let a = self.normal_h;
        let e = self.radius_scaled
            * Vector3::new(
                (1.0 - a.x * a.x / a.dot(&a)).sqrt(),
                (1.0 - a.y * a.y / a.dot(&a)).sqrt(),
                (1.0 - a.z * a.z / a.dot(&a)).sqrt(),
            );
        let pa = self.center + self.half_length * self.normal_h;
        let pb = self.center - self.half_length * self.normal_h;
        bbox.extend(pa - e);
        bbox.extend(pa + e);
        bbox.extend(pb - e);
        bbox.extend(pb + e);
        bbox
    }

    /// This body's oriented bounding box. Upstream
    /// `computeBoundingBox(OBB&)`.
    pub fn compute_bounding_obb(&self) -> OBB {
        OBB::new(
            self.pose,
            2.0 * Vector3::new(self.radius_scaled, self.radius_scaled, self.half_length),
        )
    }

    /// Every intersection of the ray (through `origin`, along `dir`) with
    /// this body, ordered along the ray and capped at `count` points
    /// (`None` for unlimited). Upstream `intersectsRay`; see the module
    /// docs, deviation 1.
    pub fn ray_intersections(
        &self,
        origin: &Vector3,
        dir: &Vector3,
        count: Option<usize>,
    ) -> Vec<Vector3> {
        let dir_norm = normalize_dir(dir);
        if distance_sqr(&self.center, origin, &dir_norm) > self.radius_bounding_sqr {
            return Vec::new();
        }

        let mut ipts: Vec<Intersc> = Vec::new();

        let tmp = self.normal_h.dot(&dir_norm);
        if tmp.abs() > ZERO {
            let tmp2 = -self.normal_h.dot(origin);
            let t1 = (tmp2 - self.d1) / tmp;
            if t1 > 0.0 {
                let p1 = origin + dir_norm * t1;
                let mut v1 = p1 - self.center;
                v1 -= self.normal_h.dot(&v1) * self.normal_h;
                if v1.norm_squared() < self.radius_scaled_sqr + ZERO {
                    ipts.push(Intersc { pt: p1, time: t1 });
                }
            }
            let t2 = (tmp2 - self.d2) / tmp;
            if t2 > 0.0 {
                let p2 = origin + dir_norm * t2;
                let mut v2 = p2 - self.center;
                v2 -= self.normal_h.dot(&v2) * self.normal_h;
                if v2.norm_squared() < self.radius_scaled_sqr + ZERO {
                    ipts.push(Intersc { pt: p2, time: t2 });
                }
            }
        }

        if ipts.len() < 2 {
            let vd = self.normal_h.cross(&dir_norm);
            let rod = self.normal_h.cross(&(origin - self.center));
            let a = vd.norm_squared();
            let b = 2.0 * rod.dot(&vd);
            let c = rod.norm_squared() - self.radius_scaled_sqr;
            let d = b * b - 4.0 * a * c;
            if d >= 0.0 && a.abs() > ZERO {
                let d = d.sqrt();
                let e = -a * 2.0;
                let t1 = (b + d) / e;
                let t2 = (b - d) / e;

                if t1 > 0.0 {
                    let p1 = origin + dir_norm * t1;
                    let v1 = self.center - p1;
                    if self.normal_h.dot(&v1).abs() < self.half_length + ZERO {
                        ipts.push(Intersc { pt: p1, time: t1 });
                    }
                }
                if t2 > 0.0 {
                    let p2 = origin + dir_norm * t2;
                    let v2 = self.center - p2;
                    if self.normal_h.dot(&v2).abs() < self.half_length + ZERO {
                        ipts.push(Intersc { pt: p2, time: t2 });
                    }
                }
            }
        }

        if ipts.is_empty() {
            return Vec::new();
        }
        filter_intersections(ipts, count)
    }

    /// Whether the ray (through `origin`, along `dir`) intersects this
    /// body. Upstream `intersectsRay` called with a null `intersections`
    /// out-param; see the module docs, deviation 1.
    pub fn intersects_ray(&self, origin: &Vector3, dir: &Vector3) -> bool {
        !self.ray_intersections(origin, dir, None).is_empty()
    }

    /// Sample a point inside this body by sampling directly in cylindrical
    /// coordinates (always succeeds; `max_attempts` is accepted for
    /// interface parity with [`Body::sample_point_inside`] but unused,
    /// matching upstream exactly). Upstream `Cylinder::samplePointInside` —
    /// note the `r` term ranges over `[-radiusU_, radiusU_]`, not `[0,
    /// radiusU_]`, so it is not an area-uniform disk sample; ported as-is.
    /// See the module docs, deviation 4.
    pub fn sample_point_inside(
        &self,
        _max_attempts: u32,
        uniform: &mut dyn FnMut(f64, f64) -> f64,
    ) -> Option<Vector3> {
        let a = uniform(-std::f64::consts::PI, std::f64::consts::PI);
        let r = uniform(-self.radius_scaled, self.radius_scaled);
        let x = a.cos() * r;
        let y = a.sin() * r;
        let z = uniform(-self.half_length, self.half_length);
        Some(transform_point(&self.pose, &Vector3::new(x, y, z)))
    }

    /// Clone this body at a new pose, keeping this body's padding and
    /// scale. Upstream `Body::cloneAt(pose)`.
    pub fn clone_at(&self, pose: Isometry3) -> Self {
        self.clone_at_with(pose, self.padding, self.scale)
            .expect("keeping this body's own valid padding and scale cannot fail")
    }

    /// Clone this body at a new pose, padding and scale. Upstream
    /// `Cylinder::cloneAt(pose, padding, scale)`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] under the same condition as [`Cylinder::new`].
    pub fn clone_at_with(&self, pose: Isometry3, padding: f64, scale: f64) -> Result<Self> {
        let mut c = *self;
        c.pose = pose;
        c.recompute(self.radius, self.length, scale, padding)?;
        Ok(c)
    }
}

/// A box body: a posed, scaled, padded [`crate::geometry::shapes::Cuboid`]. Upstream
/// `bodies::Box`, renamed to avoid shadowing [`std::boxed::Box`] — the same
/// reason as [`crate::geometry::Cuboid`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Cuboid {
    length: f64,
    width: f64,
    height: f64,
    pose: Isometry3,
    scale: f64,
    padding: f64,
    // cached
    center: Vector3,
    inv_rot: nalgebra::Matrix3<f64>,
    min_corner: Vector3,
    max_corner: Vector3,
    half_length: f64,
    half_width: f64,
    half_height: f64,
    radius_scaled_sqr: f64,
    radius_bounding: f64,
}

impl Cuboid {
    /// Build a box body from raw dimensions (length, width, height along x,
    /// y, z), identity pose, scale 1.0, no padding.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when any resulting scaled dimension would be
    /// negative.
    pub fn new(length: f64, width: f64, height: f64) -> Result<Self> {
        let mut b = Self {
            length: 0.0,
            width: 0.0,
            height: 0.0,
            pose: Isometry3::identity(),
            scale: 1.0,
            padding: 0.0,
            center: Vector3::zeros(),
            inv_rot: nalgebra::Matrix3::identity(),
            min_corner: Vector3::zeros(),
            max_corner: Vector3::zeros(),
            half_length: 0.0,
            half_width: 0.0,
            half_height: 0.0,
            radius_scaled_sqr: 0.0,
            radius_bounding: 0.0,
        };
        b.set_dimensions(length, width, height)?;
        Ok(b)
    }

    /// Build a box body directly from an axis-aligned bounding box. Upstream
    /// `explicit Box(const AABB&)`.
    pub fn from_aabb(aabb: &AABB) -> Result<Self> {
        let sizes = aabb.sizes();
        let mut b = Self::new(sizes.x, sizes.y, sizes.z)?;
        let center = aabb.center();
        b.set_pose(Isometry3::translation(center.x, center.y, center.z));
        Ok(b)
    }

    fn recompute(
        &mut self,
        length: f64,
        width: f64,
        height: f64,
        scale: f64,
        padding: f64,
    ) -> Result<()> {
        let s2 = scale / 2.0;
        let half_length = length * s2 + padding;
        let half_width = width * s2 + padding;
        let half_height = height * s2 + padding;
        if half_length < 0.0 || half_width < 0.0 || half_height < 0.0 {
            return Err(Error::construct("Box dimensions must be non-negative."));
        }

        self.length = length;
        self.width = width;
        self.height = height;
        self.scale = scale;
        self.padding = padding;
        self.half_length = half_length;
        self.half_width = half_width;
        self.half_height = half_height;

        self.center = self.pose.translation.vector;
        self.radius_scaled_sqr =
            half_length * half_length + half_width * half_width + half_height * half_height;
        self.radius_bounding = self.radius_scaled_sqr.sqrt();

        self.inv_rot = self.pose.rotation.to_rotation_matrix().matrix().transpose();

        let half = Vector3::new(half_length, half_width, half_height);
        self.min_corner = self.center - half;
        self.max_corner = self.center + half;
        Ok(())
    }

    /// This body's raw (unscaled, unpadded) length, width, height, in that
    /// order. Upstream `getDimensions`.
    pub fn dimensions(&self) -> Vec<f64> {
        vec![self.length, self.width, self.height]
    }

    /// This body's scaled and padded length, width, height, in that order.
    /// Upstream `getScaledDimensions`.
    pub fn scaled_dimensions(&self) -> Vec<f64> {
        vec![
            2.0 * self.half_length,
            2.0 * self.half_width,
            2.0 * self.half_height,
        ]
    }

    /// This body's pose.
    pub const fn pose(&self) -> Isometry3 {
        self.pose
    }

    /// This body's scale factor.
    pub const fn scale(&self) -> f64 {
        self.scale
    }

    /// This body's padding.
    pub const fn padding(&self) -> f64 {
        self.padding
    }

    /// Set this body's raw (unscaled, unpadded) length, width, height.
    /// Upstream `setDimensions`/`useDimensions`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when a resulting scaled dimension would be
    /// negative. On error, this body is left unchanged.
    pub fn set_dimensions(&mut self, length: f64, width: f64, height: f64) -> Result<()> {
        self.recompute(length, width, height, self.scale, self.padding)
    }

    /// Set this body's pose. Upstream `setPose`.
    pub fn set_pose(&mut self, pose: Isometry3) {
        self.pose = pose;
        self.recompute(
            self.length,
            self.width,
            self.height,
            self.scale,
            self.padding,
        )
        .expect("pose change alone cannot invalidate an already-valid box body");
    }

    /// Set this body's scale factor. Upstream `setScale`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when a resulting scaled dimension would be
    /// negative. On error, this body is left unchanged.
    pub fn set_scale(&mut self, scale: f64) -> Result<()> {
        self.recompute(self.length, self.width, self.height, scale, self.padding)
    }

    /// Set this body's padding. Upstream `setPadding`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when a resulting scaled dimension would be
    /// negative. On error, this body is left unchanged.
    pub fn set_padding(&mut self, padding: f64) -> Result<()> {
        self.recompute(self.length, self.width, self.height, self.scale, padding)
    }

    /// Whether `p` lies in or on this box. Upstream `containsPoint`.
    pub fn contains_point(&self, p: &Vector3) -> bool {
        let aligned = (self.inv_rot * (p - self.center)).abs();
        aligned.x <= self.half_length
            && aligned.y <= self.half_width
            && aligned.z <= self.half_height
    }

    /// This body's volume. Upstream `computeVolume`.
    pub fn compute_volume(&self) -> f64 {
        8.0 * self.half_length * self.half_width * self.half_height
    }

    /// This body's bounding sphere. Upstream `computeBoundingSphere`.
    pub fn compute_bounding_sphere(&self) -> BoundingSphere {
        BoundingSphere {
            center: self.center,
            radius: self.radius_bounding,
        }
    }

    /// This body's bounding cylinder. Upstream `computeBoundingCylinder`:
    /// picks the longest of the three half-extents as the cylinder's axis,
    /// and rotates the pose accordingly (90 degrees about y or x) so the
    /// cylinder's own z-axis matches that longest extent.
    pub fn compute_bounding_cylinder(&self) -> BoundingCylinder {
        let (length, a, b, pose);
        if self.half_length > self.half_width && self.half_length > self.half_height {
            length = self.half_length * 2.0;
            a = self.half_width;
            b = self.half_height;
            let rot = nalgebra::UnitQuaternion::from_axis_angle(
                &Vector3::y_axis(),
                std::f64::consts::FRAC_PI_2,
            );
            pose = self.pose * Isometry3::from_parts(nalgebra::Translation3::identity(), rot);
        } else if self.half_width > self.half_height {
            length = self.half_width * 2.0;
            a = self.half_height;
            b = self.half_length;
            let rot = nalgebra::UnitQuaternion::from_axis_angle(
                &Vector3::x_axis(),
                std::f64::consts::FRAC_PI_2,
            );
            pose = self.pose * Isometry3::from_parts(nalgebra::Translation3::identity(), rot);
        } else {
            length = self.half_height * 2.0;
            a = self.half_width;
            b = self.half_length;
            pose = self.pose;
        }
        BoundingCylinder {
            pose,
            radius: (a * a + b * b).sqrt(),
            length,
        }
    }

    /// This body's axis-aligned bounding box. Upstream
    /// `computeBoundingBox(AABB&)`.
    pub fn compute_bounding_aabb(&self) -> AABB {
        let mut bbox = AABB::empty();
        bbox.extend_with_transformed_box(
            &self.pose,
            2.0 * Vector3::new(self.half_length, self.half_width, self.half_height),
        );
        bbox
    }

    /// This body's oriented bounding box. Upstream
    /// `computeBoundingBox(OBB&)`.
    pub fn compute_bounding_obb(&self) -> OBB {
        OBB::new(
            self.pose,
            2.0 * Vector3::new(self.half_length, self.half_width, self.half_height),
        )
    }

    /// Every intersection of the ray (through `origin`, along `dir`) with
    /// this body, ordered along the ray and capped at `count` points
    /// (`None` for unlimited). Upstream `intersectsRay` (Brian Smits,
    /// "Efficient bounding box intersection", Ray Tracing News 15(1), 2002);
    /// see the module docs, deviation 1.
    pub fn ray_intersections(
        &self,
        origin: &Vector3,
        dir: &Vector3,
        count: Option<usize>,
    ) -> Vec<Vector3> {
        let dir_norm = normalize_dir(dir);

        let o = self.inv_rot * (origin - self.center) + self.center;
        let d = self.inv_rot * dir_norm;

        let mut tmp_tmin = (self.min_corner - o).component_div(&d);
        let mut tmp_tmax = (self.max_corner - o).component_div(&d);
        for i in 0..3 {
            if d[i] < 0.0 {
                std::mem::swap(&mut tmp_tmin[i], &mut tmp_tmax[i]);
            }
        }

        let tmin = tmp_tmin.x.max(tmp_tmin.y.max(tmp_tmin.z));
        let tmax = tmp_tmax.x.min(tmp_tmax.y.min(tmp_tmax.z));

        if tmax - tmin < -ZERO {
            return Vec::new();
        }
        if tmax < 0.0 {
            return Vec::new();
        }

        let mut out = Vec::new();
        if tmax - tmin > ZERO {
            if tmin > ZERO {
                out.push(tmin * dir_norm + origin);
                if count.is_none_or(|c| c > 1) {
                    out.push(tmax * dir_norm + origin);
                }
            } else {
                out.push(tmax * dir_norm + origin);
            }
        } else {
            out.push(tmax * dir_norm + origin);
        }
        out
    }

    /// Whether the ray (through `origin`, along `dir`) intersects this
    /// body. Upstream `intersectsRay` called with a null `intersections`
    /// out-param; see the module docs, deviation 1.
    pub fn intersects_ray(&self, origin: &Vector3, dir: &Vector3) -> bool {
        !self.ray_intersections(origin, dir, None).is_empty()
    }

    /// Sample a point inside this body by sampling directly in local box
    /// coordinates (always succeeds; `max_attempts` is accepted for
    /// interface parity with [`Body::sample_point_inside`] but unused,
    /// matching upstream exactly). Upstream `Box::samplePointInside`. See
    /// the module docs, deviation 4.
    pub fn sample_point_inside(
        &self,
        _max_attempts: u32,
        uniform: &mut dyn FnMut(f64, f64) -> f64,
    ) -> Option<Vector3> {
        let local = Vector3::new(
            uniform(-self.half_length, self.half_length),
            uniform(-self.half_width, self.half_width),
            uniform(-self.half_height, self.half_height),
        );
        Some(transform_point(&self.pose, &local))
    }

    /// Clone this body at a new pose, keeping this body's padding and
    /// scale. Upstream `Body::cloneAt(pose)`.
    pub fn clone_at(&self, pose: Isometry3) -> Self {
        self.clone_at_with(pose, self.padding, self.scale)
            .expect("keeping this body's own valid padding and scale cannot fail")
    }

    /// Clone this body at a new pose, padding and scale. Upstream
    /// `Box::cloneAt(pose, padding, scale)`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] under the same condition as [`Cuboid::new`].
    pub fn clone_at_with(&self, pose: Isometry3, padding: f64, scale: f64) -> Result<Self> {
        let mut b = *self;
        b.pose = pose;
        b.recompute(self.length, self.width, self.height, scale, padding)?;
        Ok(b)
    }
}

fn to_parry(v: Vector3) -> parry3d_f64::math::Vector3 {
    parry3d_f64::math::Vector3::new(v.x, v.y, v.z)
}

fn from_parry(v: parry3d_f64::math::Vector3) -> Vector3 {
    Vector3::new(v.x, v.y, v.z)
}

/// The convex-hull data a [`ConvexMesh`] is built from: the shape-dependent
/// half of upstream `ConvexMesh::MeshData`, shared behind an [`std::sync::Arc`] so
/// [`ConvexMesh::clone_at`] is a cheap pointer copy — upstream's own reason
/// for keeping this in one PIMPL struct (`bodies.h`'s comment on
/// `mesh_data_`).
///
/// See the module docs, deviation 2, for why this holds one outward unit
/// normal per *triangle* rather than upstream's per-*facet* (possibly
/// multi-triangle) plane list.
#[derive(Debug, Clone, PartialEq)]
struct MeshData {
    /// Hull vertices, in the mesh's own (unposed, unscaled) frame.
    vertices: Vec<Vector3>,
    /// Hull triangles, CCW when viewed from outside (`parry3d-f64`'s
    /// `try_convex_hull` guarantee) — indices into `vertices`.
    triangles: Vec<[u32; 3]>,
    /// Each triangle's outward unit normal, computed once from `vertices`
    /// (fixed; scale and padding never rotate a plane, only translate it —
    /// matching upstream's own comment that its per-facet planes
    /// "correspond to the unscaled mesh").
    normals: Vec<Vector3>,
    /// Each triangle's plane offset in the mesh's own (unposed, unscaled)
    /// frame — `-normal.dot(vertices[tri[0]])`, so the plane is `{x :
    /// normal.dot(x) + offset == 0}`. Backs [`ConvexMesh::planes`] only;
    /// `ConvexMesh`'s live containment test uses the scaled/padded
    /// `plane_offsets` field instead (see [`ConvexMesh::recompute`]) —
    /// upstream's own comment on `planes_` notes the stored offset
    /// "corresponds to the unscaled mesh" and that callers needing the
    /// scaled offset must recompute it themselves.
    unscaled_plane_offsets: Vec<f64>,
    /// Centroid of `vertices`.
    mesh_center: Vector3,
    /// The farthest any hull vertex is from `mesh_center`.
    mesh_radius_bounding: f64,
    /// Center of the *original* (pre-hull) mesh's own axis-aligned bounding
    /// box.
    box_offset: Vector3,
    /// Size of the *original* (pre-hull) mesh's own axis-aligned bounding
    /// box.
    box_size: Vector3,
    /// The local (unscaled, unpadded) bounding cylinder's radius, computed
    /// from the *original* mesh vertices around the box's longest axis.
    bounding_cylinder_radius: f64,
    /// The local (unscaled, unpadded) bounding cylinder's length — the
    /// original box's longest extent.
    bounding_cylinder_length: f64,
}

fn build_mesh_data(mesh: &ShapeMesh) -> Result<MeshData> {
    if mesh.vertices.is_empty() {
        return Err(Error::construct(
            "convex mesh body requires at least one vertex",
        ));
    }

    let mut min = Vector3::from_element(f64::INFINITY);
    let mut max = Vector3::from_element(f64::NEG_INFINITY);
    for v in &mesh.vertices {
        min = min.inf(v);
        max = max.sup(v);
    }
    let box_size = max - min;
    let box_offset = (min + max) * 0.5;

    let (off1, off2, cyl_length) = if box_size.x > box_size.y && box_size.x > box_size.z {
        (1usize, 2usize, box_size.x)
    } else if box_size.y > box_size.z {
        (0usize, 2usize, box_size.y)
    } else {
        (0usize, 1usize, box_size.z)
    };
    let pose1 = box_offset[off1];
    let pose2 = box_offset[off2];
    let bounding_cylinder_radius = mesh
        .vertices
        .iter()
        .map(|v| {
            let a = v[off1] - pose1;
            let b = v[off2] - pose2;
            (a * a + b * b).sqrt()
        })
        .fold(f64::NEG_INFINITY, f64::max);

    let parry_points: Vec<parry3d_f64::math::Vector3> =
        mesh.vertices.iter().map(|v| to_parry(*v)).collect();
    let (hull_vertices, triangles) = parry3d_f64::transformation::try_convex_hull(&parry_points)
        .map_err(|e| Error::construct(format!("convex hull computation failed: {e}")))?;
    let vertices: Vec<Vector3> = hull_vertices.into_iter().map(from_parry).collect();

    let mut normals = Vec::with_capacity(triangles.len());
    let mut unscaled_plane_offsets = Vec::with_capacity(triangles.len());
    for tri in &triangles {
        let v0 = vertices[tri[0] as usize];
        let v1 = vertices[tri[1] as usize];
        let v2 = vertices[tri[2] as usize];
        let normal = (v1 - v0).cross(&(v2 - v0));
        let normal = normal.try_normalize(0.0).unwrap_or_else(Vector3::zeros);
        unscaled_plane_offsets.push(-normal.dot(&v0));
        normals.push(normal);
    }

    let mesh_center = vertices.iter().sum::<Vector3>() / vertices.len() as f64;
    let mesh_radius_bounding = vertices
        .iter()
        .map(|v| (v - mesh_center).norm_squared())
        .fold(0.0, f64::max)
        .sqrt();

    Ok(MeshData {
        vertices,
        triangles,
        normals,
        unscaled_plane_offsets,
        mesh_center,
        mesh_radius_bounding,
        box_offset,
        box_size,
        bounding_cylinder_radius,
        bounding_cylinder_length: cyl_length,
    })
}

/// A convex mesh body: the convex hull of a [`crate::geometry::shapes::Mesh`], posed,
/// scaled and padded. Upstream `bodies::ConvexMesh`.
#[derive(Debug, Clone, PartialEq)]
pub struct ConvexMesh {
    mesh_data: std::sync::Arc<MeshData>,
    pose: Isometry3,
    scale: f64,
    padding: f64,
    // cached
    i_pose: Isometry3,
    center: Vector3,
    radius_bounding: f64,
    radius_bounding_sqr: f64,
    bounding_box: Cuboid,
    /// Each hull vertex, scaled and padded along its own line to the mesh
    /// center. Parallel to `mesh_data.vertices`.
    scaled_vertices: Vec<Vector3>,
    /// Each triangle's plane offset, recomputed from `scaled_vertices`
    /// whenever scale or padding changes. Parallel to `mesh_data.normals`;
    /// paired as `(normal, offset)` the plane is `{x : normal.dot(x) +
    /// offset == 0}`, with `normal` pointing outward — see
    /// [`ConvexMesh::is_point_inside_planes`].
    plane_offsets: Vec<f64>,
}

impl ConvexMesh {
    /// Build a convex mesh body as the convex hull of `mesh`, with identity
    /// pose, scale 1.0, no padding.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when `mesh` has no vertices, or when computing
    /// the convex hull fails (upstream logs a warning and silently
    /// continues with an empty, always-non-containing, always-non-
    /// intersecting body in this case — see the module docs, deviation 2;
    /// this port surfaces the failure instead of building a body that can
    /// never contain anything).
    pub fn new(mesh: &ShapeMesh) -> Result<Self> {
        let mesh_data = std::sync::Arc::new(build_mesh_data(mesh)?);
        let bounding_box = Cuboid::new(
            mesh_data.box_size.x,
            mesh_data.box_size.y,
            mesh_data.box_size.z,
        )?;
        let mut m = Self {
            mesh_data,
            pose: Isometry3::identity(),
            scale: 1.0,
            padding: 0.0,
            i_pose: Isometry3::identity(),
            center: Vector3::zeros(),
            radius_bounding: 0.0,
            radius_bounding_sqr: 0.0,
            bounding_box,
            scaled_vertices: Vec::new(),
            plane_offsets: Vec::new(),
        };
        m.recompute(1.0, 0.0)?;
        Ok(m)
    }

    fn recompute(&mut self, scale: f64, padding: f64) -> Result<()> {
        self.bounding_box.set_scale(scale)?;
        self.bounding_box.set_padding(padding)?;
        self.bounding_box.set_dimensions(
            self.mesh_data.box_size.x,
            self.mesh_data.box_size.y,
            self.mesh_data.box_size.z,
        )?;
        let bbox_pose_translation = transform_point(&self.pose, &self.mesh_data.box_offset);
        self.bounding_box.set_pose(Isometry3::from_parts(
            bbox_pose_translation.into(),
            self.pose.rotation,
        ));

        self.scale = scale;
        self.padding = padding;
        self.i_pose = self.pose.inverse();
        self.center = transform_point(&self.pose, &self.mesh_data.mesh_center);
        self.radius_bounding = self.mesh_data.mesh_radius_bounding * scale + padding;
        self.radius_bounding_sqr = self.radius_bounding * self.radius_bounding;

        self.scaled_vertices = if padding == 0.0 && scale == 1.0 {
            self.mesh_data.vertices.clone()
        } else {
            self.mesh_data
                .vertices
                .iter()
                .map(|v| {
                    let d = v - self.mesh_data.mesh_center;
                    let l = d.norm();
                    self.mesh_data.mesh_center
                        + d * (scale + if l > ZERO { padding / l } else { 0.0 })
                })
                .collect()
        };

        self.plane_offsets = self
            .mesh_data
            .triangles
            .iter()
            .zip(self.mesh_data.normals.iter())
            .map(|(tri, normal)| -normal.dot(&self.scaled_vertices[tri[0] as usize]))
            .collect();
        Ok(())
    }

    /// The convex hull's vertices, in the mesh's own unposed, unscaled
    /// frame. Upstream `getVertices`. Exact: both this port's quickhull and
    /// upstream's qhull compute the convex hull of the same input point
    /// set, so the vertex *set* matches (up to order and, for a mesh with
    /// no coplanar-with-hull interior points, exactly) — a probe against
    /// the shipped `.so` confirms this for both a mesh with no discarded
    /// interior points (a tetrahedron) and one with coplanar faces (a box).
    pub fn vertices(&self) -> &[Vector3] {
        &self.mesh_data.vertices
    }

    /// The convex hull's vertices, scaled and padded along their own line
    /// to the mesh center — i.e. `vertices()` after this body's current
    /// [`ConvexMesh::set_scale`]/[`ConvexMesh::set_padding`]. Upstream
    /// `getScaledVertices`. Exact, for the same reason `vertices` is.
    pub fn scaled_vertices(&self) -> &[Vector3] {
        &self.scaled_vertices
    }

    /// The convex hull's triangles, as vertex-index triples into
    /// [`ConvexMesh::vertices`]. Upstream `getTriangles`, which returns the
    /// same triangles flattened into one contiguous index list rather than
    /// grouped in triples — this port's `[u32; 3]`-per-triangle shape is a
    /// representation choice, not a semantic difference.
    ///
    /// Not expected to match upstream triangle-for-triangle: this port's
    /// quickhull and upstream's qhull can split the same coplanar patch
    /// along different face diagonals, so the triangulation *topology* can
    /// legitimately differ between the two even when every vertex and the
    /// overall hull volume agree. A probe confirms vertex-set and
    /// volume/count-of-triangles-per-planar-patch equivalence, not
    /// index-for-index equality.
    pub fn triangles(&self) -> &[[u32; 3]] {
        &self.mesh_data.triangles
    }

    /// One outward-facing plane per hull *triangle*, as `(normal, offset)`
    /// pairs in the mesh's own unposed, unscaled frame, where the plane is
    /// `{x : normal.dot(x) + offset == 0}`. Upstream `getPlanes` returns
    /// one plane per hull *facet* instead, merging the planes of adjacent
    /// triangles that are coplanar to within its own tolerance (see the
    /// module docs, deviation 2, for why this port does not reconstruct
    /// that merge) — so this method's length is a superset of upstream's
    /// whenever the mesh has a coplanar patch (a box: this port reports 12
    /// entries, six duplicated pairs, where upstream reports 6), and an
    /// exact match only when no two triangles share a plane (e.g. a
    /// tetrahedron, whose 4 triangles are all on distinct planes). Grouping
    /// this method's output by `(normal, offset)` within a small tolerance
    /// recovers upstream's per-facet count and values; a probe against the
    /// shipped `.so` confirms both the tetrahedron's exact match and the
    /// box's over-count-that-dedups-to-the-same-values case.
    pub fn planes(&self) -> Vec<(Vector3, f64)> {
        self.mesh_data
            .normals
            .iter()
            .copied()
            .zip(self.mesh_data.unscaled_plane_offsets.iter().copied())
            .collect()
    }

    /// Returns an empty vector. Upstream `getDimensions` — a convex mesh has
    /// no scalar dimensions to report.
    pub fn dimensions(&self) -> Vec<f64> {
        Vec::new()
    }

    /// Returns an empty vector. Upstream `getScaledDimensions`.
    pub fn scaled_dimensions(&self) -> Vec<f64> {
        Vec::new()
    }

    /// This body's pose.
    pub const fn pose(&self) -> Isometry3 {
        self.pose
    }

    /// This body's scale factor.
    pub const fn scale(&self) -> f64 {
        self.scale
    }

    /// This body's padding.
    pub const fn padding(&self) -> f64 {
        self.padding
    }

    /// Set this body's pose. Upstream `setPose`.
    pub fn set_pose(&mut self, pose: Isometry3) {
        self.pose = pose;
        self.recompute(self.scale, self.padding)
            .expect("pose change alone cannot invalidate an already-valid convex mesh body");
    }

    /// Set this body's scale factor. Upstream `setScale`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] under the same condition as the embedded
    /// bounding box's `set_scale`. On error, this body is left unchanged.
    pub fn set_scale(&mut self, scale: f64) -> Result<()> {
        self.recompute(scale, self.padding)
    }

    /// Set this body's padding. Upstream `setPadding`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] under the same condition as the embedded
    /// bounding box's `set_padding`. On error, this body is left unchanged.
    pub fn set_padding(&mut self, padding: f64) -> Result<()> {
        self.recompute(self.scale, padding)
    }

    /// Whether `point` (already transformed into this mesh's own, unposed
    /// frame) lies on the inner side of every hull plane, with a `ZERO`
    /// margin outside each plane still counted as inside. Upstream
    /// `isPointInsidePlanes`.
    fn is_point_inside_planes(&self, point: &Vector3) -> bool {
        self.mesh_data
            .normals
            .iter()
            .zip(self.plane_offsets.iter())
            .all(|(normal, offset)| normal.dot(point) + offset - ZERO <= 0.0)
    }

    /// Whether `p` lies in or on this convex mesh. Upstream `containsPoint`.
    pub fn contains_point(&self, p: &Vector3) -> bool {
        if !self.bounding_box.contains_point(p) {
            return false;
        }
        let local = transform_point(&self.i_pose, p);
        self.is_point_inside_planes(&local)
    }

    /// This body's volume (of the convex hull). Upstream `computeVolume` —
    /// note this is the hull's *raw* volume, not scaled/padded (matching
    /// upstream, which computes it directly from `mesh_data_->vertices_`,
    /// never `scaled_vertices_`).
    pub fn compute_volume(&self) -> f64 {
        let mut volume = 0.0;
        for tri in &self.mesh_data.triangles {
            let v1 = self.mesh_data.vertices[tri[0] as usize];
            let v2 = self.mesh_data.vertices[tri[1] as usize];
            let v3 = self.mesh_data.vertices[tri[2] as usize];
            volume += v1.x * v2.y * v3.z + v2.x * v3.y * v1.z + v3.x * v1.y * v2.z
                - v1.x * v3.y * v2.z
                - v2.x * v1.y * v3.z
                - v3.x * v2.y * v1.z;
        }
        volume.abs() / 6.0
    }

    /// This body's bounding sphere. Upstream `computeBoundingSphere`.
    pub fn compute_bounding_sphere(&self) -> BoundingSphere {
        BoundingSphere {
            center: self.center,
            radius: self.radius_bounding,
        }
    }

    /// This body's bounding cylinder. Upstream `computeBoundingCylinder` —
    /// the pose comes from the embedded bounding box's own bounding
    /// cylinder (upstream's comment: "need to do rotation correctly to get
    /// pose, which bounding box does").
    pub fn compute_bounding_cylinder(&self) -> BoundingCylinder {
        let boxed = self.bounding_box.compute_bounding_cylinder();
        BoundingCylinder {
            pose: boxed.pose,
            radius: self.mesh_data.bounding_cylinder_radius * self.scale + self.padding,
            length: self.mesh_data.bounding_cylinder_length * self.scale + 2.0 * self.padding,
        }
    }

    /// This body's axis-aligned bounding box. Upstream
    /// `computeBoundingBox(AABB&)` — delegates to the embedded bounding box.
    pub fn compute_bounding_aabb(&self) -> AABB {
        self.bounding_box.compute_bounding_aabb()
    }

    /// This body's oriented bounding box. Upstream
    /// `computeBoundingBox(OBB&)` — delegates to the embedded bounding box.
    pub fn compute_bounding_obb(&self) -> OBB {
        self.bounding_box.compute_bounding_obb()
    }

    /// Every intersection of the ray (through `origin`, along `dir`) with
    /// this body, ordered along the ray and capped at `count` points
    /// (`None` for unlimited). Upstream `intersectsRay`; see the module
    /// docs, deviations 1 and 2.
    ///
    /// # Deviation: `tmp`/`t`'s guards were negated into a NaN-unsafe form
    ///
    /// Upstream (`bodies.cpp:1260-1266`) gates the per-triangle work with
    /// two *positive* conditions: `fabs(tmp) > detail::ZERO`, then `t >
    /// 0.0`. Both read `false` for a NaN `tmp`/`t`, so a NaN correctly
    /// skips the triangle. This method used to spell the same gate as an
    /// early `continue` on the naively negated condition — `if tmp.abs()
    /// <= ZERO { continue; }` — which is not upstream's true negation:
    /// `<=` is *also* false for NaN, so a NaN `tmp`/`t` fell through into
    /// `p = orig + dr * t`, carrying the NaN into a returned intersection
    /// point. The explicit `is_nan()` checks below are the true negation
    /// of upstream's positive condition, correct for NaN where the naive
    /// `<=`/`<=` flip was not (`clippy::neg_cmp_op_on_partial_ord` also
    /// rejects spelling this as `!(tmp.abs() > ZERO)`, for the same
    /// reason: it reads as `<=`).
    ///
    /// # Corrected: a non-finite ray poisons every triangle, it is not "zeroed out"
    ///
    /// An earlier draft of this comment justified the above by a NaN
    /// confined to one axis of `origin`/`dir` reaching `tmp`/`t` as a
    /// *finite* value whenever a triangle's axis-aligned `normal` happens
    /// to be zero on that axis — e.g. `normal = (0, 0, 1)` "zeroing out" a
    /// NaN-x ray's x-component in `normal.dot(&dr)`. That is false: IEEE
    /// 754 gives `0.0 * NaN == NaN`, not `0.0`, so the NaN survives the
    /// multiply and then the sum — `Vector3::new(0.0, 1.0,
    /// 0.0).dot(&Vector3::new(f64::NAN, 1.0, 0.0))` is `NaN`, not the `1.0`
    /// the "zeroed out" claim implied. The dot product is NaN on *every*
    /// triangle a non-finite ray touches, regardless of which normal
    /// component is zero; the `is_nan()` checks below are not narrowly
    /// catching a value that escaped a coincidental zeroing, a non-finite
    /// input poisons them unconditionally.
    ///
    /// That also settles whether `origin`/`dir` need an up-front finite
    /// check ahead of the per-triangle loop, rather than relying on the
    /// per-triangle `is_nan()` guards alone: they do not. `±inf` reaches
    /// this method through two paths, and both are already intercepted
    /// before `tmp`/`t` by the same `0.0 * x == NaN` fact, one step
    /// removed. An infinite `dir` makes `dir_norm` (`dir / dir.norm()`, an
    /// `inf / inf` division) carry a NaN before it is ever dotted with a
    /// normal. An infinite `origin` survives `transform_point`'s
    /// quaternion-based rotation as a clean infinity on at most one axis,
    /// while the cross-product terms mixing in the *other* axes each hit a
    /// `0.0 * inf` and turn to NaN — at the identity pose, `origin = (0.0,
    /// 0.0, f64::INFINITY)` transforms to `(NaN, NaN, NaN)`, not `(0.0,
    /// 0.0, f64::INFINITY)` unchanged. Once any axis of the transformed
    /// origin is NaN, `normal.dot(&orig)` is NaN regardless of which axis
    /// a given triangle's normal is nonzero on, by the same fact above.
    /// See `convex_mesh_ray_rejects_positive_infinity_in_origin` and its
    /// siblings below for the regression coverage.
    pub fn ray_intersections(
        &self,
        origin: &Vector3,
        dir: &Vector3,
        count: Option<usize>,
    ) -> Vec<Vector3> {
        let dir_norm = normalize_dir(dir);
        if distance_sqr(&self.center, origin, &dir_norm) > self.radius_bounding_sqr {
            return Vec::new();
        }
        if !self.bounding_box.intersects_ray(origin, &dir_norm) {
            return Vec::new();
        }

        let orig = transform_point(&self.i_pose, origin);
        let dr = self.i_pose.rotation * dir_norm;

        let mut ipts: Vec<Intersc> = Vec::new();
        for ((tri, normal), offset) in self
            .mesh_data
            .triangles
            .iter()
            .zip(self.mesh_data.normals.iter())
            .zip(self.plane_offsets.iter())
        {
            let tmp = normal.dot(&dr);
            if tmp.is_nan() || tmp.abs() <= ZERO {
                continue;
            }
            let t = -(normal.dot(&orig) + offset) / tmp;
            if t.is_nan() || t <= 0.0 {
                continue;
            }

            let a = self.scaled_vertices[tri[0] as usize];
            let b = self.scaled_vertices[tri[1] as usize];
            let c = self.scaled_vertices[tri[2] as usize];
            let cb = c - b;
            let ab = a - b;
            let p = orig + dr * t;

            let pb = p - b;
            let c1 = cb.cross(&pb);
            let c2 = cb.cross(&ab);
            if c1.dot(&c2) < 0.0 {
                continue;
            }

            let ca = c - a;
            let pa = p - a;
            let ba = -ab;
            let c1 = ca.cross(&pa);
            let c2 = ca.cross(&ba);
            if c1.dot(&c2) < 0.0 {
                continue;
            }

            let c1 = ba.cross(&pa);
            let c2 = ba.cross(&ca);
            if c1.dot(&c2) < 0.0 {
                continue;
            }

            ipts.push(Intersc {
                pt: origin + dir_norm * t,
                time: t,
            });
        }

        if ipts.is_empty() {
            return Vec::new();
        }
        filter_intersections(ipts, count)
    }

    /// Whether the ray (through `origin`, along `dir`) intersects this
    /// body. Upstream `intersectsRay` called with a null `intersections`
    /// out-param; see the module docs, deviation 1.
    pub fn intersects_ray(&self, origin: &Vector3, dir: &Vector3) -> bool {
        !self.ray_intersections(origin, dir, None).is_empty()
    }

    /// Sample a point inside this body. Upstream has no `ConvexMesh`
    /// override for `samplePointInside`, so it falls back to the generic
    /// `Body::samplePointInside` (rejection sampling within the computed
    /// bounding sphere). See the module docs, deviation 4.
    pub fn sample_point_inside(
        &self,
        max_attempts: u32,
        uniform: &mut dyn FnMut(f64, f64) -> f64,
    ) -> Option<Vector3> {
        let bs = self.compute_bounding_sphere();
        for _ in 0..max_attempts {
            let candidate = Vector3::new(
                uniform(bs.center.x - bs.radius, bs.center.x + bs.radius),
                uniform(bs.center.y - bs.radius, bs.center.y + bs.radius),
                uniform(bs.center.z - bs.radius, bs.center.z + bs.radius),
            );
            if self.contains_point(&candidate) {
                return Some(candidate);
            }
        }
        None
    }

    /// Clone this body at a new pose, keeping this body's padding and
    /// scale. Upstream `Body::cloneAt(pose)`. Cheap: the hull data is
    /// shared via `Arc`, matching upstream's `shared_ptr` reuse.
    pub fn clone_at(&self, pose: Isometry3) -> Self {
        self.clone_at_with(pose, self.padding, self.scale)
            .expect("keeping this body's own valid padding and scale cannot fail")
    }

    /// Clone this body at a new pose, padding and scale. Upstream
    /// `ConvexMesh::cloneAt(pose, padding, scale)`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] under the same condition as [`ConvexMesh::set_scale`]/
    /// [`ConvexMesh::set_padding`].
    pub fn clone_at_with(&self, pose: Isometry3, padding: f64, scale: f64) -> Result<Self> {
        let mut m = self.clone();
        m.pose = pose;
        m.recompute(scale, padding)?;
        Ok(m)
    }
}

/// A posed, scaled, padded solid body derived from a [`Shape`]. Upstream
/// `bodies::Body` and its `BodyType` tag — see the module docs, design
/// note 1, for why this is a closed enum rather than a trait-object
/// hierarchy.
#[derive(Debug, Clone, PartialEq)]
pub enum Body {
    /// Upstream `bodies::Sphere`.
    Sphere(Sphere),
    /// Upstream `bodies::Cylinder`.
    Cylinder(Cylinder),
    /// Upstream `bodies::Box`.
    Cuboid(Cuboid),
    /// Upstream `bodies::ConvexMesh`. Boxed: at 500+ bytes (a shared
    /// [`std::sync::Arc`] plus an embedded [`Cuboid`] plus two `Vec`s), it
    /// otherwise dwarfs the other three variants (each under 300 bytes) and
    /// would pad every [`Body`] value to its size.
    ConvexMesh(Box<ConvexMesh>),
}

impl From<Sphere> for Body {
    fn from(value: Sphere) -> Self {
        Self::Sphere(value)
    }
}

impl From<Cylinder> for Body {
    fn from(value: Cylinder) -> Self {
        Self::Cylinder(value)
    }
}

impl From<Cuboid> for Body {
    fn from(value: Cuboid) -> Self {
        Self::Cuboid(value)
    }
}

impl From<ConvexMesh> for Body {
    fn from(value: ConvexMesh) -> Self {
        Self::ConvexMesh(Box::new(value))
    }
}

impl Body {
    /// Build the body corresponding to `shape`, with identity pose, scale
    /// 1.0, no padding. Upstream `bodies::createBodyFromShape`.
    ///
    /// Returns `Ok(None)` for [`Shape::Cone`], [`Shape::Plane`] and
    /// [`Shape::OcTree`], which have no `bodies::` counterpart upstream:
    /// `createEmptyBodyFromShapeType` (`body_operations.cpp:37-60`) has no
    /// arm for any of the three, so it logs an error and returns `nullptr`.
    ///
    /// Upstream's `createBodyFromShape` does **not** hand that `nullptr`
    /// back to its caller — it calls `body->setDimensions(shape)` on it
    /// first (`body_operations.cpp:68-69`), unguarded. Same shape one level
    /// up in `kinematic_constraints`: `kinematic_constraint.cpp:412-413`
    /// takes `createEmptyBodyFromShapeType(shape->type)` straight into
    /// `body->setDimensionsDirty(shape.get())`. So the upstream behaviour
    /// this `None` stands in for is a null dereference, not a returned
    /// `nullptr`, and it is reachable: `constraint_region.primitives` is a
    /// `shape_msgs/SolidPrimitive[]`, `SolidPrimitive::CONE` is one of the
    /// four types `constructShapeFromMsg` builds
    /// (`shape_operations.cpp:101-106`), and nothing between the two filters
    /// it out.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] if building the underlying body fails — e.g. a
    /// [`Shape::Mesh`] with no vertices, or whose convex hull cannot be
    /// computed (see [`ConvexMesh::new`]).
    pub fn from_shape(shape: &Shape) -> Result<Option<Self>> {
        Ok(match shape {
            Shape::Sphere(s) => Some(Sphere::new(s.radius)?.into()),
            Shape::Cylinder(c) => Some(Cylinder::new(c.radius, c.length)?.into()),
            Shape::Cuboid(b) => Some(Cuboid::new(b.size[0], b.size[1], b.size[2])?.into()),
            Shape::Mesh(m) => Some(ConvexMesh::new(m)?.into()),
            Shape::Cone(_) | Shape::Plane(_) | Shape::OcTree(_) => None,
        })
    }

    /// This body's dimensions, before scale/padding. Upstream
    /// `getDimensions`.
    pub fn dimensions(&self) -> Vec<f64> {
        match self {
            Self::Sphere(s) => s.dimensions(),
            Self::Cylinder(c) => c.dimensions(),
            Self::Cuboid(b) => b.dimensions(),
            Self::ConvexMesh(m) => m.dimensions(),
        }
    }

    /// This body's dimensions, after scale/padding. Upstream
    /// `getScaledDimensions`.
    pub fn scaled_dimensions(&self) -> Vec<f64> {
        match self {
            Self::Sphere(s) => s.scaled_dimensions(),
            Self::Cylinder(c) => c.scaled_dimensions(),
            Self::Cuboid(b) => b.scaled_dimensions(),
            Self::ConvexMesh(m) => m.scaled_dimensions(),
        }
    }

    /// This body's pose. Upstream `getPose`.
    pub fn pose(&self) -> Isometry3 {
        match self {
            Self::Sphere(s) => s.pose(),
            Self::Cylinder(c) => c.pose(),
            Self::Cuboid(b) => b.pose(),
            Self::ConvexMesh(m) => m.pose(),
        }
    }

    /// This body's scale factor. Upstream `getScale`.
    pub fn scale(&self) -> f64 {
        match self {
            Self::Sphere(s) => s.scale(),
            Self::Cylinder(c) => c.scale(),
            Self::Cuboid(b) => b.scale(),
            Self::ConvexMesh(m) => m.scale(),
        }
    }

    /// This body's padding. Upstream `getPadding`.
    pub fn padding(&self) -> f64 {
        match self {
            Self::Sphere(s) => s.padding(),
            Self::Cylinder(c) => c.padding(),
            Self::Cuboid(b) => b.padding(),
            Self::ConvexMesh(m) => m.padding(),
        }
    }

    /// Set this body's pose. Upstream `setPose`.
    pub fn set_pose(&mut self, pose: Isometry3) {
        match self {
            Self::Sphere(s) => s.set_pose(pose),
            Self::Cylinder(c) => c.set_pose(pose),
            Self::Cuboid(b) => b.set_pose(pose),
            Self::ConvexMesh(m) => m.set_pose(pose),
        }
    }

    /// Set this body's scale factor. Upstream `setScale`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] under the same condition as the wrapped body's
    /// own `set_scale`. On error, this body is left unchanged.
    pub fn set_scale(&mut self, scale: f64) -> Result<()> {
        match self {
            Self::Sphere(s) => s.set_scale(scale),
            Self::Cylinder(c) => c.set_scale(scale),
            Self::Cuboid(b) => b.set_scale(scale),
            Self::ConvexMesh(m) => m.set_scale(scale),
        }
    }

    /// Set this body's padding. Upstream `setPadding`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] under the same condition as the wrapped body's
    /// own `set_padding`. On error, this body is left unchanged.
    pub fn set_padding(&mut self, padding: f64) -> Result<()> {
        match self {
            Self::Sphere(s) => s.set_padding(padding),
            Self::Cylinder(c) => c.set_padding(padding),
            Self::Cuboid(b) => b.set_padding(padding),
            Self::ConvexMesh(m) => m.set_padding(padding),
        }
    }

    /// Whether `p` lies in or on this body. Upstream `containsPoint`
    /// (the `Vector3d&` overload, with no `verbose` out-param).
    pub fn contains_point(&self, p: &Vector3) -> bool {
        match self {
            Self::Sphere(s) => s.contains_point(p),
            Self::Cylinder(c) => c.contains_point(p),
            Self::Cuboid(b) => b.contains_point(p),
            Self::ConvexMesh(m) => m.contains_point(p),
        }
    }

    /// This body's volume. Upstream `computeVolume`.
    pub fn compute_volume(&self) -> f64 {
        match self {
            Self::Sphere(s) => s.compute_volume(),
            Self::Cylinder(c) => c.compute_volume(),
            Self::Cuboid(b) => b.compute_volume(),
            Self::ConvexMesh(m) => m.compute_volume(),
        }
    }

    /// This body's bounding sphere. Upstream `computeBoundingSphere`.
    pub fn compute_bounding_sphere(&self) -> BoundingSphere {
        match self {
            Self::Sphere(s) => s.compute_bounding_sphere(),
            Self::Cylinder(c) => c.compute_bounding_sphere(),
            Self::Cuboid(b) => b.compute_bounding_sphere(),
            Self::ConvexMesh(m) => m.compute_bounding_sphere(),
        }
    }

    /// This body's bounding cylinder. Upstream `computeBoundingCylinder`.
    pub fn compute_bounding_cylinder(&self) -> BoundingCylinder {
        match self {
            Self::Sphere(s) => s.compute_bounding_cylinder(),
            Self::Cylinder(c) => c.compute_bounding_cylinder(),
            Self::Cuboid(b) => b.compute_bounding_cylinder(),
            Self::ConvexMesh(m) => m.compute_bounding_cylinder(),
        }
    }

    /// This body's axis-aligned bounding box. Upstream
    /// `computeBoundingBox(AABB&)`.
    pub fn compute_bounding_aabb(&self) -> AABB {
        match self {
            Self::Sphere(s) => s.compute_bounding_aabb(),
            Self::Cylinder(c) => c.compute_bounding_aabb(),
            Self::Cuboid(b) => b.compute_bounding_aabb(),
            Self::ConvexMesh(m) => m.compute_bounding_aabb(),
        }
    }

    /// This body's oriented bounding box. Upstream
    /// `computeBoundingBox(OBB&)`.
    pub fn compute_bounding_obb(&self) -> OBB {
        match self {
            Self::Sphere(s) => s.compute_bounding_obb(),
            Self::Cylinder(c) => c.compute_bounding_obb(),
            Self::Cuboid(b) => b.compute_bounding_obb(),
            Self::ConvexMesh(m) => m.compute_bounding_obb(),
        }
    }

    /// Every intersection of the ray (through `origin`, along `dir`) with
    /// this body, ordered along the ray and capped at `count` points
    /// (`None` for unlimited). Upstream `intersectsRay`.
    pub fn ray_intersections(
        &self,
        origin: &Vector3,
        dir: &Vector3,
        count: Option<usize>,
    ) -> Vec<Vector3> {
        match self {
            Self::Sphere(s) => s.ray_intersections(origin, dir, count),
            Self::Cylinder(c) => c.ray_intersections(origin, dir, count),
            Self::Cuboid(b) => b.ray_intersections(origin, dir, count),
            Self::ConvexMesh(m) => m.ray_intersections(origin, dir, count),
        }
    }

    /// Whether the ray (through `origin`, along `dir`) intersects this
    /// body. Upstream `intersectsRay` called with a null `intersections`
    /// out-param.
    pub fn intersects_ray(&self, origin: &Vector3, dir: &Vector3) -> bool {
        match self {
            Self::Sphere(s) => s.intersects_ray(origin, dir),
            Self::Cylinder(c) => c.intersects_ray(origin, dir),
            Self::Cuboid(b) => b.intersects_ray(origin, dir),
            Self::ConvexMesh(m) => m.intersects_ray(origin, dir),
        }
    }

    /// Sample a point inside this body, using `uniform(lo, hi)` as the
    /// analog of upstream's `RandomNumberGenerator::uniformReal(lo, hi)` —
    /// see the module docs, deviation 5, for why this port takes a
    /// sampler closure instead of depending on a RNG crate. Upstream
    /// `Body::samplePointInside` and its per-type overrides.
    pub fn sample_point_inside(
        &self,
        max_attempts: u32,
        uniform: &mut dyn FnMut(f64, f64) -> f64,
    ) -> Option<Vector3> {
        match self {
            Self::Sphere(s) => s.sample_point_inside(max_attempts, uniform),
            Self::Cylinder(c) => c.sample_point_inside(max_attempts, uniform),
            Self::Cuboid(b) => b.sample_point_inside(max_attempts, uniform),
            Self::ConvexMesh(m) => m.sample_point_inside(max_attempts, uniform),
        }
    }

    /// Clone this body at a new pose, keeping this body's padding and
    /// scale. Upstream `Body::cloneAt(pose)`.
    pub fn clone_at(&self, pose: Isometry3) -> Self {
        match self {
            Self::Sphere(s) => s.clone_at(pose).into(),
            Self::Cylinder(c) => c.clone_at(pose).into(),
            Self::Cuboid(b) => b.clone_at(pose).into(),
            Self::ConvexMesh(m) => m.clone_at(pose).into(),
        }
    }

    /// Clone this body at a new pose, padding and scale. Upstream
    /// `Body::cloneAt(pose, padding, scale)`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] under the same condition as the wrapped body's
    /// own `clone_at_with`.
    pub fn clone_at_with(&self, pose: Isometry3, padding: f64, scale: f64) -> Result<Self> {
        Ok(match self {
            Self::Sphere(s) => s.clone_at_with(pose, padding, scale)?.into(),
            Self::Cylinder(c) => c.clone_at_with(pose, padding, scale)?.into(),
            Self::Cuboid(b) => b.clone_at_with(pose, padding, scale)?.into(),
            Self::ConvexMesh(m) => m.clone_at_with(pose, padding, scale)?.into(),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::UnitQuaternion;
    use approx::assert_relative_eq;

    /// A tiny deterministic xorshift64 PRNG, used only by this module's own
    /// tests as the `uniform(lo, hi)` sampler [`Sphere::sample_point_inside`]
    /// et al. take — see the module docs, deviation 5, for why no `rand`
    /// dependency was added and upstream's exact
    /// `RandomNumberGenerator`/iteration-count sequences are not
    /// reproduced.
    /// `{:?}`-based equality: unlike `assert_eq!`, two NaN fields compare
    /// equal here (both print `NaN`), which a couple of tests below need --
    /// see their own comments for why a genuinely-NaN field is expected on
    /// both sides of the comparison, not a bug being masked.
    fn debug_string(obb: &OBB) -> String {
        format!("{obb:?}")
    }

    fn uniform_test_rng(seed: u64) -> impl FnMut(f64, f64) -> f64 {
        let mut state = seed | 1;
        move |lo: f64, hi: f64| {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            let unit = (state >> 11) as f64 / (1u64 << 53) as f64;
            lo + unit * (hi - lo)
        }
    }

    /// An axis-aligned box mesh (8 corners, no input triangles — matching
    /// upstream's own `ConvexMeshRayIntersection` tests, which build their
    /// mesh via `shapes::createMeshFromShape(&box)` and let `ConvexMesh`
    /// recompute its own hull) with full extents `(lx, ly, lz)` centered at
    /// the origin.
    fn box_mesh(lx: f64, ly: f64, lz: f64) -> ShapeMesh {
        let (hx, hy, hz) = (lx / 2.0, ly / 2.0, lz / 2.0);
        ShapeMesh {
            vertices: vec![
                Vector3::new(-hx, -hy, -hz),
                Vector3::new(-hx, -hy, hz),
                Vector3::new(-hx, hy, -hz),
                Vector3::new(-hx, hy, hz),
                Vector3::new(hx, -hy, -hz),
                Vector3::new(hx, -hy, hz),
                Vector3::new(hx, hy, -hz),
                Vector3::new(hx, hy, hz),
            ],
            ..Default::default()
        }
    }

    // --- containsPoint: literal ground truth from geometric_shapes 2.3.3
    // --- test/test_point_inclusion.cpp ---

    #[test]
    fn sphere_contains_point_basic() {
        // SpherePointContainment::Basic
        let sphere = Sphere::new(1.0).unwrap();
        assert!(sphere.contains_point(&Vector3::new(0.0, 0.0, 0.0)));
        assert!(!sphere.contains_point(&Vector3::new(1.0, 1.0, 1.0)));

        assert!(sphere.contains_point(&Vector3::new(0.99, 0.0, 0.0)));
        assert!(sphere.contains_point(&Vector3::new(1.00, 0.0, 0.0))); // surface counts as inside
        assert!(!sphere.contains_point(&Vector3::new(1.01, 0.0, 0.0)));

        let sq3 = 3f64.sqrt() / 3.0;
        assert!(sphere.contains_point(&Vector3::new(0.57, 0.57, 0.57)));
        assert!(sphere.contains_point(&Vector3::new(sq3, sq3, sq3)));
        assert!(!sphere.contains_point(&Vector3::new(0.58, 0.58, 0.58)));
    }

    #[test]
    fn sphere_contains_point_translated() {
        // SpherePointContainment::Basic, "near three-axis maximum with translation"
        let mut sphere = Sphere::new(1.0).unwrap();
        let sq3 = 3f64.sqrt() / 3.0;
        sphere.set_pose(Isometry3::translation(1.0, 0.0, 0.0));
        assert!(sphere.contains_point(&Vector3::new(1.57, 0.57, 0.57)));
        assert!(sphere.contains_point(&Vector3::new(1.0 + sq3, sq3, sq3)));
        assert!(!sphere.contains_point(&Vector3::new(1.58, 0.58, 0.58)));
    }

    #[test]
    fn cuboid_contains_point_basic() {
        // BoxPointContainment::Basic
        let cuboid = Cuboid::new(2.0, 2.0, 2.0).unwrap();
        assert!(cuboid.contains_point(&Vector3::new(0.0, 0.0, 0.0)));
        assert!(!cuboid.contains_point(&Vector3::new(2.0, 2.0, 2.0)));

        assert!(cuboid.contains_point(&Vector3::new(0.99, 0.99, 0.99)));
        assert!(cuboid.contains_point(&Vector3::new(1.00, 1.00, 1.00))); // corner counts as inside
        assert!(!cuboid.contains_point(&Vector3::new(1.01, 1.01, 1.01)));
    }

    #[test]
    fn cuboid_contains_point_translated() {
        // BoxPointContainment::Basic, "near three-axis maximum with translation"
        let mut cuboid = Cuboid::new(2.0, 2.0, 2.0).unwrap();
        cuboid.set_pose(Isometry3::translation(1.0, 0.0, 0.0));
        assert!(cuboid.contains_point(&Vector3::new(1.99, 0.99, 0.99)));
        assert!(cuboid.contains_point(&Vector3::new(2.00, 1.00, 1.00)));
        assert!(!cuboid.contains_point(&Vector3::new(2.01, 1.01, 1.01)));
    }

    #[test]
    fn cylinder_contains_point_basic() {
        // CylinderPointContainment::Basic
        let cylinder = Cylinder::new(1.0, 4.0).unwrap();
        assert!(cylinder.contains_point(&Vector3::new(0.0, 0.0, 0.0)));
        assert!(!cylinder.contains_point(&Vector3::new(1.0, 1.0, 4.0)));

        assert!(cylinder.contains_point(&Vector3::new(0.99, 0.0, 0.0)));
        assert!(cylinder.contains_point(&Vector3::new(1.00, 0.0, 0.0)));
        assert!(!cylinder.contains_point(&Vector3::new(1.01, 0.0, 0.0)));

        assert!(cylinder.contains_point(&Vector3::new(0.0, 0.0, 1.99)));
        assert!(cylinder.contains_point(&Vector3::new(0.0, 0.0, 2.00)));
        assert!(!cylinder.contains_point(&Vector3::new(0.0, 0.0, 2.01)));
    }

    #[test]
    fn cylinder_padding_increases_bounding_sphere() {
        // CylinderPointContainment::CylinderPadding
        let mut cylinder = Cylinder::new(1.0, 4.0).unwrap();
        assert!(!cylinder.contains_point(&Vector3::new(0.0, 1.01, 0.0)));
        cylinder.set_padding(0.02).unwrap();
        assert!(cylinder.contains_point(&Vector3::new(0.0, 1.01, 0.0)));
        cylinder.set_padding(0.0).unwrap();
        assert!(cylinder.compute_bounding_sphere().radius > 2.0);
    }

    // --- ray intersection: literal ground truth from
    // --- test/test_ray_intersection.cpp ---

    #[test]
    fn sphere_ray_origin_inside_basic_axes() {
        // SphereRayIntersection::OriginInside
        let sphere = Sphere::new(1.0).unwrap();
        let origin = Vector3::zeros();
        for (dir, expected) in [
            (Vector3::new(1.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)),
            (Vector3::new(-1.0, 0.0, 0.0), Vector3::new(-1.0, 0.0, 0.0)),
            (Vector3::new(0.0, 1.0, 0.0), Vector3::new(0.0, 1.0, 0.0)),
            (Vector3::new(0.0, 0.0, -1.0), Vector3::new(0.0, 0.0, -1.0)),
        ] {
            let hits = sphere.ray_intersections(&origin, &dir, Some(2));
            assert_eq!(hits.len(), 1);
            assert_eq!(hits[0], expected);
            assert!(sphere.intersects_ray(&origin, &dir));
        }
    }

    #[test]
    fn sphere_ray_origin_inside_scaled() {
        // SphereRayIntersection::OriginInside, "scaling"
        let mut sphere = Sphere::new(1.0).unwrap();
        sphere.set_scale(1.1).unwrap();
        let origin = Vector3::zeros();
        let hits = sphere.ray_intersections(&origin, &Vector3::new(1.0, 0.0, 0.0), Some(2));
        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0], Vector3::new(1.1, 0.0, 0.0));
    }

    #[test]
    fn sphere_ray_origin_inside_moved_sphere() {
        // SphereRayIntersection::OriginInside, "move sphere" — upstream
        // reaches this section with scale still 1.1 from the earlier
        // "scaling" section in the same TEST (never reset), hence 1.6/-0.6
        // rather than 1.5/-0.5.
        let mut sphere = Sphere::new(1.0).unwrap();
        sphere.set_scale(1.1).unwrap();
        sphere.set_pose(Isometry3::translation(0.5, 0.0, 0.0));
        let origin = Vector3::zeros();
        let hits = sphere.ray_intersections(&origin, &Vector3::new(1.0, 0.0, 0.0), Some(2));
        assert_eq!(hits.len(), 1);
        // Bit-exact (round 14, §79): bisected alone (not grouped with the
        // -0.6 assertion below) to epsilon = 0.0, max_relative = 0.0 and
        // still passed.
        assert_eq!(hits[0], Vector3::new(1.6, 0.0, 0.0));
        let hits = sphere.ray_intersections(&origin, &Vector3::new(-1.0, 0.0, 0.0), Some(2));
        assert_eq!(hits.len(), 1);
        // NOT bit-exact (round 14, §79): bisected alone, unlike the +1.6
        // hit above -- fails at epsilon = 0.0 (measured left =
        // -0.6000000000000001, right = -0.6, diff ~1.11e-16, 1 ULP),
        // passes at 1e-15, fails at 1e-16. epsilon = 1e-13 below is real,
        // measured headroom (~1e3x the observed diff), not the file's old
        // 1e-6 carried over unmeasured.
        assert_relative_eq!(
            hits[0],
            Vector3::new(-0.6, 0.0, 0.0),
            epsilon = 1e-13,
            max_relative = 0.0
        );
    }

    #[test]
    fn sphere_ray_origin_outside_twice_axes() {
        // SphereRayIntersection::OriginOutside
        let sphere = Sphere::new(1.0).unwrap();
        let hits = sphere.ray_intersections(
            &Vector3::new(-2.0, 0.0, 0.0),
            &Vector3::new(1.0, 0.0, 0.0),
            Some(2),
        );
        assert_eq!(hits.len(), 2);
        let mut norms: Vec<f64> = hits.iter().map(|h| h.x).collect();
        norms.sort_by(f64::total_cmp);
        assert_eq!(norms[0], -1.0);
        assert_eq!(norms[1], 1.0);
    }

    /// `filter_intersections`' dedup threshold must be Eigen's *relative*
    /// `prec*prec * min(|a|^2, |b|^2)`, not an *absolute* one.
    ///
    /// A sphere's own two-root path can't exercise this: its half-chord
    /// `x = radius_scaled_sqr - w.norm_squared()` snaps to a single-point
    /// tangent hit whenever `x.abs() < ZERO`, so any two points it *does*
    /// return are already at least `2*sqrt(ZERO) ~ 6.3e-5` apart — always
    /// past the old code's worst-case (`.max(1.0)`-floored) `ZERO` = `1e-9`
    /// threshold. Same shape in `Cuboid`'s `tmax - tmin > ZERO` slab test.
    /// A cylinder's two *cap* hits don't share that coupling: `d1`'s `t1`
    /// and `d2`'s `t2` (`Cylinder::recompute`) are each validated only
    /// against their own disk, with no joint tangent check between them —
    /// the side quadratic that has one only runs at all when `ipts.len() <
    /// 2`, i.e. once both caps already matched.
    ///
    /// A wafer-thin cylinder (`length == 1e-12`, `half_length == 5e-13`)
    /// centered at `(0, 0, 1.5e-12)`, hit head-on along its own axis, has
    /// its two cap intersections at exactly `(0, 0, 1e-12)` and `(0, 0,
    /// 2e-12)` — 1e-12 apart, both well inside the unit ball. Eigen's
    /// threshold there is `(1e-9)^2 * min((1e-12)^2, (2e-12)^2) = 1e-42`,
    /// and `1e-12 > 1e-42`, so upstream keeps both. The version this
    /// replaces used `ZERO * max(|a|, |b|).max(1.0)` — the `.max(1.0)`
    /// floor turns that into an absolute `1e-9` threshold whenever both
    /// points are within the unit ball, and `1e-12 <= 1e-9` wrongly deduped
    /// the second point away.
    #[test]
    fn ray_intersections_keeps_two_cap_hits_within_the_unit_ball_that_the_old_absolute_floor_deduped()
     {
        let mut cylinder = Cylinder::new(1.0, 1e-12).unwrap();
        cylinder.set_pose(Isometry3::translation(0.0, 0.0, 1.5e-12));
        let hits = cylinder.ray_intersections(
            &Vector3::new(0.0, 0.0, 0.0),
            &Vector3::new(0.0, 0.0, 1.0),
            None,
        );
        assert_eq!(
            hits.len(),
            2,
            "both cap hits are within Eigen's relative tolerance of being distinct: {hits:?}"
        );
        assert_relative_eq!(hits[0], Vector3::new(0.0, 0.0, 1e-12), epsilon = 1e-19);
        assert_relative_eq!(hits[1], Vector3::new(0.0, 0.0, 2e-12), epsilon = 1e-19);
    }

    /// The demonstrated opposite of the test above: the same two-independent-
    /// cap-hits mechanism, but scaled up so both points sit at `|z| ~ 10`,
    /// `1e-9` apart. There `min(|a|^2, |b|^2) ~ 100` is far above the old
    /// code's `.max(1.0)` floor, so both the old and new formulas agree —
    /// this is not the case the fix changes behavior on, and that is the
    /// point: it proves `filter_intersections` still dedups a real
    /// near-duplicate rather than the fix having turned it into "keep every
    /// candidate, always."
    #[test]
    fn ray_intersections_still_dedups_a_genuine_near_duplicate_at_ordinary_scale() {
        let mut cylinder = Cylinder::new(1.0, 1e-9).unwrap();
        cylinder.set_pose(Isometry3::translation(0.0, 0.0, 10.0));
        let hits = cylinder.ray_intersections(
            &Vector3::new(0.0, 0.0, 0.0),
            &Vector3::new(0.0, 0.0, 1.0),
            None,
        );
        assert_eq!(
            hits.len(),
            1,
            "a 1e-9 separation at |z| ~ 10 is a near-duplicate under both formulas: {hits:?}"
        );
    }

    /// Boundary: a ray tangent to the sphere's surface — hits exactly once,
    /// not twice, not zero times.
    #[test]
    fn sphere_ray_tangent_hits_surface_once() {
        // SphereRayIntersection::OriginOutside, "test hitting the surface"
        let sphere = Sphere::new(1.0).unwrap();
        let hits = sphere.ray_intersections(
            &Vector3::new(-1.0, -1.0, 0.0),
            &Vector3::new(0.0, 1.0, 0.0),
            Some(2),
        );
        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0], Vector3::new(-1.0, 0.0, 0.0));
    }

    /// Boundary: a ray that just misses the sphere's surface.
    #[test]
    fn sphere_ray_just_misses_surface_is_no_intersection() {
        // SphereRayIntersection::OriginOutside, "test missing the surface"
        let sphere = Sphere::new(1.0).unwrap();
        assert!(
            sphere
                .ray_intersections(
                    &Vector3::new(-1.1, -1.0, 0.0),
                    &Vector3::new(0.0, 1.0, 0.0),
                    Some(2),
                )
                .is_empty()
        );
        assert!(
            !sphere.intersects_ray(&Vector3::new(-1.1, -1.0, 0.0), &Vector3::new(0.0, 1.0, 0.0))
        );
    }

    #[test]
    fn sphere_ray_simple() {
        let mut sphere = Sphere::new(1.0).unwrap();
        sphere.set_scale(1.05).unwrap();
        let hits = sphere.ray_intersections(
            &Vector3::new(5.0, 0.0, 0.0),
            &Vector3::new(-1.0, 0.0, 0.0),
            Some(2),
        );
        assert_eq!(hits.len(), 2);
        assert!(!sphere.intersects_ray(&Vector3::new(5.0, 0.0, 0.0), &Vector3::new(1.0, 0.0, 0.0)));
    }

    #[test]
    fn cylinder_ray_origin_inside_basic_axes() {
        // CylinderRayIntersection::OriginInside
        let cylinder = Cylinder::new(1.0, 2.0).unwrap();
        let origin = Vector3::zeros();
        for (dir, expected) in [
            (Vector3::new(1.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)),
            (Vector3::new(0.0, 0.0, 1.0), Vector3::new(0.0, 0.0, 1.0)),
            (Vector3::new(0.0, 0.0, -1.0), Vector3::new(0.0, 0.0, -1.0)),
        ] {
            let hits = cylinder.ray_intersections(&origin, &dir, Some(2));
            assert_eq!(hits.len(), 1);
            assert_eq!(hits[0], expected);
        }
    }

    #[test]
    fn cylinder_ray_origin_outside_twice_axes() {
        // CylinderRayIntersection::OriginOutside, scale 1.5 padding 0.5
        let mut cylinder = Cylinder::new(1.0, 2.0).unwrap();
        cylinder.set_scale(1.5).unwrap();
        cylinder.set_padding(0.5).unwrap();
        let hits = cylinder.ray_intersections(
            &Vector3::new(-4.0, 0.0, 0.0),
            &Vector3::new(1.0, 0.0, 0.0),
            Some(2),
        );
        assert_eq!(hits.len(), 2);
        let mut xs: Vec<f64> = hits.iter().map(|h| h.x).collect();
        xs.sort_by(f64::total_cmp);
        assert_eq!(xs[0], -2.0);
        assert_eq!(xs[1], 2.0);
    }

    /// Boundary: a ray tangent to the cylinder's curved surface.
    #[test]
    fn cylinder_ray_tangent_hits_surface_once() {
        // CylinderRayIntersection::OriginOutside, "test hitting the surface"
        let mut cylinder = Cylinder::new(1.0, 2.0).unwrap();
        cylinder.set_scale(1.5).unwrap();
        cylinder.set_padding(0.5).unwrap();
        let hits = cylinder.ray_intersections(
            &Vector3::new(-2.0, -2.0, 0.0),
            &Vector3::new(0.0, 1.0, 0.0),
            Some(2),
        );
        assert_eq!(hits.len(), 1);
        assert_eq!(hits[0], Vector3::new(-2.0, 0.0, 0.0));
    }

    /// Boundary: a ray that just misses the cylinder's curved surface.
    #[test]
    fn cylinder_ray_just_misses_surface_is_no_intersection() {
        // CylinderRayIntersection::OriginOutside, "test missing the surface"
        let mut cylinder = Cylinder::new(1.0, 2.0).unwrap();
        cylinder.set_scale(1.5).unwrap();
        cylinder.set_padding(0.5).unwrap();
        assert!(
            !cylinder.intersects_ray(&Vector3::new(-2.1, -1.0, 0.0), &Vector3::new(0.0, 1.0, 0.0))
        );
    }

    #[test]
    fn cylinder_ray_simple() {
        let mut cylinder = Cylinder::new(1.0, 2.0).unwrap();
        cylinder.set_scale(1.05).unwrap();
        let hits = cylinder.ray_intersections(
            &Vector3::new(5.0, 0.0, 0.0),
            &Vector3::new(-1.0, 0.0, 0.0),
            Some(2),
        );
        assert_eq!(hits.len(), 2);
        assert!(
            !cylinder.intersects_ray(&Vector3::new(5.0, 0.0, 0.0), &Vector3::new(1.0, 0.0, 0.0))
        );
    }

    #[test]
    fn cuboid_ray_simple1() {
        // BoxRayIntersection::SimpleRay1
        let mut cuboid = Cuboid::new(1.0, 1.0, 3.0).unwrap();
        cuboid.set_scale(0.95).unwrap();
        let hits = cuboid.ray_intersections(
            &Vector3::new(10.0, 0.449, 0.0),
            &Vector3::new(-1.0, 0.0, 0.0),
            Some(2),
        );
        assert_eq!(hits.len(), 2);
        let mut xs: Vec<f64> = hits.iter().map(|h| h.x).collect();
        xs.sort_by(f64::total_cmp);
        // NOT bit-exact (round 14, §79): each bisected alone. xs[0] fails
        // at epsilon = 0.0 (measured left = -0.47499999999999964, right =
        // -0.475, diff ~3.6e-16); xs[1] fails the same way (left =
        // 0.47499999999999964, right = 0.475). Both pass at 1e-15, fail at
        // 1e-16. epsilon = 1e-13 below is real, measured headroom, not the
        // file's old 1e-4 carried over unmeasured.
        assert_relative_eq!(xs[0], -0.475, epsilon = 1e-13, max_relative = 0.0);
        assert_relative_eq!(xs[1], 0.475, epsilon = 1e-13, max_relative = 0.0);
    }

    #[test]
    fn cuboid_ray_simple2() {
        // BoxRayIntersection::SimpleRay2
        let cuboid_shape = (0.9, 0.01, 1.2);
        let mut cuboid = Cuboid::new(cuboid_shape.0, cuboid_shape.1, cuboid_shape.2).unwrap();
        cuboid.set_pose(Isometry3::translation(0.0, 0.005, 0.6));
        let dir = Vector3::new(0.0, -5.195, -0.77).normalize();
        let hits = cuboid.ray_intersections(&Vector3::new(0.0, 5.0, 1.6), &dir, Some(2));
        assert_eq!(hits.len(), 2);
    }

    #[test]
    fn cuboid_ray_simple3_no_intersection() {
        // BoxRayIntersection::SimpleRay3
        let mut cuboid = Cuboid::new(0.02, 0.4, 1.2).unwrap();
        cuboid.set_pose(Isometry3::translation(0.45, -0.195, 0.6));
        let dir = Vector3::new(0.0, 1.8, -0.669).normalize();
        assert!(!cuboid.intersects_ray(&Vector3::new(0.0, -2.0, 1.11), &dir));
    }

    #[test]
    fn cuboid_ray_regression109_rotated_corner() {
        // BoxRayIntersection::Regression109 — a rotated box so the
        // original (0.5,0.5,0.5) corner is no longer the max corner.
        let mut cuboid = Cuboid::new(1.0, 1.0, 1.0).unwrap();
        let axis = Vector3::new(1.0, -1.0, 1.0).normalize();
        let rot = nalgebra::UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(axis),
            std::f64::consts::PI * 2.0 / 3.0,
        );
        cuboid.set_pose(Isometry3::from_parts(
            nalgebra::Translation3::identity(),
            rot,
        ));

        let hits = cuboid.ray_intersections(
            &Vector3::new(-2.0, 0.0, 0.0),
            &Vector3::new(1.0, 0.0, 0.0),
            Some(2),
        );
        assert_eq!(hits.len(), 2);
        let mut xs: Vec<f64> = hits.iter().map(|h| h.x).collect();
        xs.sort_by(f64::total_cmp);
        assert_eq!(xs[0], -0.5);
        assert_eq!(xs[1], 0.5);
    }

    #[test]
    fn cuboid_ray_origin_inside_basic_axes() {
        // BoxRayIntersection::OriginInside
        let cuboid = Cuboid::new(2.0, 2.0, 2.0).unwrap();
        let origin = Vector3::zeros();
        for (dir, expected) in [
            (Vector3::new(1.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)),
            (Vector3::new(0.0, -1.0, 0.0), Vector3::new(0.0, -1.0, 0.0)),
        ] {
            let hits = cuboid.ray_intersections(&origin, &dir, Some(2));
            assert_eq!(hits.len(), 1);
            assert_eq!(hits[0], expected);
        }
    }

    #[test]
    fn cuboid_ray_origin_outside_twice_axes() {
        // BoxRayIntersection::OriginOutsideIntersects
        let cuboid = Cuboid::new(2.0, 2.0, 2.0).unwrap();
        let hits = cuboid.ray_intersections(
            &Vector3::new(-2.0, 0.0, 0.0),
            &Vector3::new(1.0, 0.0, 0.0),
            Some(2),
        );
        assert_eq!(hits.len(), 2);
    }

    /// Boundary: ray hits exactly a shared edge (two coordinates pinned to
    /// the box's extent simultaneously), traveling along that edge.
    #[test]
    fn cuboid_ray_hits_exact_edge_twice() {
        let cuboid = Cuboid::new(2.0, 2.0, 2.0).unwrap();
        let hits = cuboid.ray_intersections(
            &Vector3::new(-4.0, 1.0, 1.0),
            &Vector3::new(1.0, 0.0, 0.0),
            Some(2),
        );
        assert_eq!(hits.len(), 2);
        let mut xs: Vec<f64> = hits.iter().map(|h| h.x).collect();
        xs.sort_by(f64::total_cmp);
        assert_eq!(xs[0], -1.0);
        assert_eq!(xs[1], 1.0);
        for h in &hits {
            assert_eq!(h.y, 1.0);
            assert_eq!(h.z, 1.0);
        }
    }

    /// Boundary: ray direction lies exactly in the plane of a face (one
    /// coordinate pinned to the box's extent, ray otherwise crossing the
    /// face at its own boundary).
    #[test]
    fn cuboid_ray_parallel_to_face() {
        let cuboid = Cuboid::new(2.0, 2.0, 2.0).unwrap();
        let hits = cuboid.ray_intersections(
            &Vector3::new(0.0, -4.0, 1.0),
            &Vector3::new(0.0, 1.0, 0.0),
            Some(2),
        );
        assert_eq!(hits.len(), 2);
        let mut ys: Vec<f64> = hits.iter().map(|h| h.y).collect();
        ys.sort_by(f64::total_cmp);
        assert_eq!(ys[0], -1.0);
        assert_eq!(ys[1], 1.0);
        for h in &hits {
            assert_eq!(h.z, 1.0);
        }
    }

    /// Boundary: ray hits exactly a vertex (all three coordinates pinned
    /// simultaneously), passing straight through to the opposite vertex.
    #[test]
    fn cuboid_ray_hits_exact_vertex_twice() {
        let cuboid = Cuboid::new(2.0, 2.0, 2.0).unwrap();
        let dir = Vector3::new(1.0, 1.0, 1.0).normalize();
        let hits = cuboid.ray_intersections(&Vector3::new(-4.0, -4.0, -4.0), &dir, Some(2));
        assert_eq!(hits.len(), 2);
        assert_eq!(hits[0], Vector3::new(-1.0, -1.0, -1.0));
        assert_eq!(hits[1], Vector3::new(1.0, 1.0, 1.0));
    }

    /// Boundary: zero-length ray direction from outside the box must not
    /// report a false intersection. Upstream's own algorithm (Brian
    /// Smits' slab method) hits the same IEEE-754 `inf - inf = NaN`/`0 *
    /// inf = NaN` arithmetic for a zero direction in both languages; from
    /// outside the box this resolves cleanly to "no intersection" because
    /// the `tmax < 0.0` check short-circuits before any NaN can reach an
    /// output point (see [`Cuboid::ray_intersections`]).
    #[test]
    fn cuboid_ray_zero_length_direction_from_outside_is_no_intersection() {
        let cuboid = Cuboid::new(2.0, 2.0, 2.0).unwrap();
        assert!(!cuboid.intersects_ray(&Vector3::new(5.0, 5.0, 5.0), &Vector3::zeros()));
    }

    // --- ConvexMesh: cross-checked against the equivalent Cuboid (see the
    // --- module docs' note on the sign-convention this port had to choose
    // --- for the ray/plane intersection formula) and against upstream's
    // --- own literal `ConvexMeshRayIntersection` numbers, which reuse the
    // --- exact `BoxRayIntersection` values for a box-shaped mesh. ---

    #[test]
    fn convex_mesh_ray_matches_cuboid_simple1() {
        // ConvexMeshRayIntersection::SimpleRay1 == BoxRayIntersection::SimpleRay1
        let mesh = ConvexMesh::new(&box_mesh(1.0, 1.0, 3.0)).unwrap();
        let mut mesh = mesh;
        mesh.set_scale(0.95).unwrap();
        let hits = mesh.ray_intersections(
            &Vector3::new(10.0, 0.449, 0.0),
            &Vector3::new(-1.0, 0.0, 0.0),
            Some(2),
        );
        assert_eq!(hits.len(), 2);
        let mut xs: Vec<f64> = hits.iter().map(|h| h.x).collect();
        xs.sort_by(f64::total_cmp);
        // NOT bit-exact (round 14, §79): each bisected alone. xs[0] fails
        // at epsilon = 0.0 (measured left = -0.47499999999999964, right =
        // -0.475, diff ~3.6e-16); xs[1] fails the same way (left =
        // 0.47499999999999964, right = 0.475). Both pass at 1e-15, fail at
        // 1e-16. epsilon = 1e-13 below is real, measured headroom, not the
        // file's old 1e-4 carried over unmeasured.
        assert_relative_eq!(xs[0], -0.475, epsilon = 1e-13, max_relative = 0.0);
        assert_relative_eq!(xs[1], 0.475, epsilon = 1e-13, max_relative = 0.0);
    }

    #[test]
    fn convex_mesh_ray_matches_cuboid_simple2() {
        // ConvexMeshRayIntersection::SimpleRay2 == BoxRayIntersection::SimpleRay2
        let mut mesh = ConvexMesh::new(&box_mesh(0.9, 0.01, 1.2)).unwrap();
        mesh.set_pose(Isometry3::translation(0.0, 0.005, 0.6));
        let dir = Vector3::new(0.0, -5.195, -0.77).normalize();
        let hits = mesh.ray_intersections(&Vector3::new(0.0, 5.0, 1.6), &dir, Some(2));
        assert_eq!(hits.len(), 2);
    }

    #[test]
    fn convex_mesh_ray_matches_cuboid_simple3_no_intersection() {
        // ConvexMeshRayIntersection::SimpleRay3 == BoxRayIntersection::SimpleRay3
        let mut mesh = ConvexMesh::new(&box_mesh(0.02, 0.4, 1.2)).unwrap();
        mesh.set_pose(Isometry3::translation(0.45, -0.195, 0.6));
        let dir = Vector3::new(0.0, 1.8, -0.669).normalize();
        assert!(!mesh.intersects_ray(&Vector3::new(0.0, -2.0, 1.11), &dir));
    }

    #[test]
    fn convex_mesh_ray_origin_inside_basic_axes() {
        // ConvexMeshRayIntersection::OriginInside
        let mesh = ConvexMesh::new(&box_mesh(2.0, 2.0, 2.0)).unwrap();
        let origin = Vector3::zeros();
        for (dir, expected) in [
            (Vector3::new(1.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)),
            (Vector3::new(0.0, 0.0, -1.0), Vector3::new(0.0, 0.0, -1.0)),
        ] {
            let hits = mesh.ray_intersections(&origin, &dir, Some(2));
            assert_eq!(hits.len(), 1);
            assert_eq!(hits[0], expected);
        }
    }

    #[test]
    fn convex_mesh_ray_origin_outside_twice_axes() {
        // ConvexMeshRayIntersection::OriginOutsideIntersects
        let mesh = ConvexMesh::new(&box_mesh(2.0, 2.0, 2.0)).unwrap();
        let hits = mesh.ray_intersections(
            &Vector3::new(-2.0, 0.0, 0.0),
            &Vector3::new(1.0, 0.0, 0.0),
            Some(2),
        );
        assert_eq!(hits.len(), 2);
    }

    /// A NaN confined to `origin`'s y-component, against every face
    /// (including the top/bottom, z-normal faces, whose `normal` is zero
    /// on the y-axis): `normal.dot(&orig)` is NaN on all of them —
    /// `0.0 * NaN == NaN`, so a zero normal component does not zero the
    /// NaN out — and the per-triangle `t.is_nan()` guard rejects every
    /// triangle before any `p = orig + dr * t` is computed. See
    /// `ConvexMesh::ray_intersections`'s doc comment, "a non-finite ray
    /// poisons every triangle".
    #[test]
    fn convex_mesh_ray_rejects_a_nan_confined_to_one_axis() {
        let mesh = ConvexMesh::new(&box_mesh(2.0, 2.0, 2.0)).unwrap();
        let hits = mesh.ray_intersections(
            &Vector3::new(0.0, f64::NAN, -2.0),
            &Vector3::new(0.0, 0.0, 1.0),
            None,
        );
        assert_eq!(
            hits,
            Vec::<Vector3>::new(),
            "a non-finite ray must yield no intersections, not a NaN-carrying point: {hits:?}"
        );
    }

    /// Demonstrated opposite for this test and the three infinity
    /// regressions below: the same ray with only one component replaced
    /// still hits the top and bottom faces normally, so none of the four
    /// can pass by rejecting everything.
    #[test]
    fn convex_mesh_ray_still_hits_the_same_axis_when_finite() {
        let mesh = ConvexMesh::new(&box_mesh(2.0, 2.0, 2.0)).unwrap();
        let hits = mesh.ray_intersections(
            &Vector3::new(0.0, 0.0, -2.0),
            &Vector3::new(0.0, 0.0, 1.0),
            None,
        );
        assert_eq!(hits.len(), 2, "{hits:?}");
    }

    /// `origin` at `+inf` on the axis `dir` travels along (a hit face's
    /// normal is nonzero there) — the infinite mirror of
    /// `convex_mesh_ray_still_hits_the_same_axis_when_finite`'s finite
    /// `origin.z`, `dir` sign flipped so the ray still points at the box.
    /// [`transform_point`] turns this into an all-NaN local origin (see
    /// `ConvexMesh::ray_intersections`'s doc comment), so `t.is_nan()`
    /// rejects every triangle; no up-front finite check is needed to close
    /// this.
    #[test]
    fn convex_mesh_ray_rejects_positive_infinity_in_origin() {
        let mesh = ConvexMesh::new(&box_mesh(2.0, 2.0, 2.0)).unwrap();
        let hits = mesh.ray_intersections(
            &Vector3::new(0.0, 0.0, f64::INFINITY),
            &Vector3::new(0.0, 0.0, -1.0),
            None,
        );
        assert_eq!(
            hits,
            Vec::<Vector3>::new(),
            "a non-finite ray must yield no intersections: {hits:?}"
        );
    }

    /// `-inf`, mirrored: same baseline as
    /// `convex_mesh_ray_still_hits_the_same_axis_when_finite`, `origin.z`
    /// replaced by `-inf` instead of the finite `-2.0`.
    #[test]
    fn convex_mesh_ray_rejects_negative_infinity_in_origin() {
        let mesh = ConvexMesh::new(&box_mesh(2.0, 2.0, 2.0)).unwrap();
        let hits = mesh.ray_intersections(
            &Vector3::new(0.0, 0.0, f64::NEG_INFINITY),
            &Vector3::new(0.0, 0.0, 1.0),
            None,
        );
        assert_eq!(
            hits,
            Vec::<Vector3>::new(),
            "a non-finite ray must yield no intersections: {hits:?}"
        );
    }

    /// `dir.z` at `+inf` instead of `origin.z` — same baseline as
    /// `convex_mesh_ray_still_hits_the_same_axis_when_finite`, `dir`'s
    /// finite `1.0` replaced by `+inf`. A different mechanism than the two
    /// `origin` cases above: `normalize_dir`'s `dir / dir.norm()` is an
    /// `inf / inf` division, so `dir_norm` (and then `dr`) is already NaN
    /// before any triangle's `tmp = normal.dot(&dr)`, and `tmp.is_nan()`
    /// rejects every triangle before `t` is even computed.
    #[test]
    fn convex_mesh_ray_rejects_positive_infinity_in_dir() {
        let mesh = ConvexMesh::new(&box_mesh(2.0, 2.0, 2.0)).unwrap();
        let hits = mesh.ray_intersections(
            &Vector3::new(0.0, 0.0, -2.0),
            &Vector3::new(0.0, 0.0, f64::INFINITY),
            None,
        );
        assert_eq!(
            hits,
            Vec::<Vector3>::new(),
            "a non-finite ray must yield no intersections: {hits:?}"
        );
    }

    #[test]
    fn convex_mesh_contains_point_basic() {
        // MeshPointContainment::Basic — box.dae is a half-extent-1 cube
        // (derived from the test's own scale=1.5/padding=0.5*sqrt(3)
        // literals: scaled corner = h*(1.5 + 0.5/h) = 2.0 => h = 1.0).
        let mut mesh = ConvexMesh::new(&box_mesh(2.0, 2.0, 2.0)).unwrap();
        mesh.set_scale(1.5).unwrap();
        mesh.set_padding(0.5 * 3f64.sqrt()).unwrap();

        assert!(mesh.contains_point(&Vector3::new(0.0, 0.0, 0.0)));
        assert!(!mesh.contains_point(&Vector3::new(3.0, 3.0, 3.0)));

        assert!(mesh.contains_point(&Vector3::new(1.99, 0.0, 0.0)));
        assert!(mesh.contains_point(&Vector3::new(2.00, 0.0, 0.0)));
        assert!(!mesh.contains_point(&Vector3::new(2.01, 0.0, 0.0)));

        assert!(mesh.contains_point(&Vector3::new(1.99, 1.99, 1.99)));
        assert!(mesh.contains_point(&Vector3::new(2.00, 2.00, 2.00)));
        assert!(!mesh.contains_point(&Vector3::new(2.01, 2.01, 2.01)));
    }

    /// Boundary: an empty vertex list must be rejected, not silently
    /// produce a body that never contains or intersects anything (see the
    /// module docs' note on this being a deliberate improvement over
    /// upstream's own "zombie" empty-`mesh_data_` behavior).
    ///
    /// `build_mesh_data` has a second, distinct `Error::Construct` site --
    /// `try_convex_hull` itself failing -- and `try_convex_hull` on zero
    /// points also errors, so a bare `.is_err()` here would still pass even
    /// with the dedicated vertex-count guard deleted entirely (bite-checked:
    /// removing the guard leaves this test green). Matching on the message
    /// is what proves the guard fired rather than the hull call falling
    /// through and failing on its own.
    #[test]
    fn convex_mesh_zero_vertex_is_an_error() {
        let mesh = ShapeMesh::default();
        let err = ConvexMesh::new(&mesh).unwrap_err();
        assert!(
            err.to_string().contains("requires at least one vertex"),
            "expected the dedicated vertex-count guard, got: {err}"
        );
    }

    /// Boundary: a mesh with vertices but zero *input* triangles is still
    /// constructible — `ConvexMesh` always recomputes its own hull from
    /// the vertex point cloud and never reads the input mesh's own
    /// triangulation (matching upstream `useDimensions`, which calls qhull
    /// on the vertex array regardless of what triangles were supplied).
    #[test]
    fn mesh_with_zero_triangles_is_constructible() {
        let mesh = box_mesh(2.0, 2.0, 2.0);
        assert!(mesh.triangles.is_empty());
        let body = ConvexMesh::new(&mesh).unwrap();
        assert!(body.contains_point(&Vector3::zeros()));
    }

    // --- volume / dimensions: invariant boundaries, not narrative
    // --- scenarios ---

    #[test]
    fn sphere_volume_matches_four_thirds_pi_r_cubed() {
        let sphere = Sphere::new(2.0).unwrap();
        // Bit-exact (round 14, §79): bisected to epsilon = 0.0, max_relative
        // = 0.0 and still passed. `radius * radius * radius` for
        // radius = 2.0 is `2.0*2.0*2.0 = 8.0`, an exact power-of-two chain,
        // matching this literal's own `* 8.0` bit for bit.
        assert_eq!(
            sphere.compute_volume(),
            4.0 / 3.0 * std::f64::consts::PI * 8.0
        );
    }

    #[test]
    fn cylinder_volume_matches_pi_r_squared_h() {
        let cylinder = Cylinder::new(2.0, 3.0).unwrap();
        // Bit-exact (round 14, §79): bisected to epsilon = 0.0, max_relative
        // = 0.0 and still passed. `PI * radius * radius * length` is
        // `PI * 2.0 * 2.0 * 3.0`; `2.0 * 2.0` is an exact power-of-two
        // multiply, matching this literal's `PI * 4.0 * 3.0` bit for bit.
        assert_eq!(cylinder.compute_volume(), std::f64::consts::PI * 4.0 * 3.0);
    }

    /// Boundary: zero-length cylinder has zero volume, not NaN or a
    /// negative value.
    #[test]
    fn degenerate_cylinder_zero_length_volume_is_zero() {
        let cylinder = Cylinder::new(1.0, 0.0).unwrap();
        // Bit-exact (round 14, §79): bisected to epsilon = 0.0, max_relative
        // = 0.0 and still passed. `length = 0.0` makes the volume product
        // exactly 0.0, no rounding possible.
        assert_eq!(cylinder.compute_volume(), 0.0);
    }

    #[test]
    fn cuboid_volume_matches_l_w_h() {
        let cuboid = Cuboid::new(2.0, 3.0, 4.0).unwrap();
        // Bit-exact (round 14, §79): bisected to epsilon = 0.0, max_relative
        // = 0.0 and still passed. `2.0 * 3.0 * 4.0` is exact integer
        // arithmetic in f64.
        assert_eq!(cuboid.compute_volume(), 24.0);
    }

    #[test]
    fn convex_mesh_volume_of_box_matches_l_w_h() {
        let mesh = ConvexMesh::new(&box_mesh(2.0, 3.0, 4.0)).unwrap();
        // Bit-exact (round 14, §79): bisected to epsilon = 0.0, max_relative
        // = 0.0 and still passed. Convex-hull volume of an axis-aligned
        // 2x3x4 box decomposes into tetrahedra whose signed volumes are
        // sums of exact-integer products; the exact decomposition sums to
        // 24.0 for this mesh with no rounding surviving.
        assert_eq!(mesh.compute_volume(), 24.0);
    }

    // --- negative/zero dimensions and padding inversion: invariant
    // --- boundaries ---

    // Assertion-discrimination sweep (round 2): `Sphere::new` ->
    // `set_dimensions` has exactly one `Err` site -- verdict
    // `single-branch`, one `Error::` hit over the function body.
    #[test]
    fn sphere_negative_radius_is_an_error() {
        assert!(Sphere::new(-1.0).is_err());
    }

    #[test]
    fn sphere_zero_radius_is_valid() {
        assert!(Sphere::new(0.0).is_ok());
    }

    /// Distinct from `cylinder_negative_length_is_an_error` below:
    /// `Cylinder::recompute` has two separate sequential guards with two
    /// different messages ("radius"/"length"), unlike `shapes::Cylinder`'s
    /// single combined `||` guard -- a bare `.is_err()` cannot tell the two
    /// apart, so a message swapped onto the wrong guard would still pass
    /// both tests (bite-checked: swapping the two messages left both green
    /// under the old assertion).
    #[test]
    fn cylinder_negative_radius_is_an_error() {
        let err = Cylinder::new(-1.0, 1.0).unwrap_err();
        assert!(
            err.to_string().contains("radius"),
            "expected the radius guard, got: {err}"
        );
    }

    /// Distinct from `cylinder_negative_radius_is_an_error` above: see that
    /// test's doc comment.
    #[test]
    fn cylinder_negative_length_is_an_error() {
        let err = Cylinder::new(1.0, -1.0).unwrap_err();
        assert!(
            err.to_string().contains("length"),
            "expected the length guard, got: {err}"
        );
    }

    // Assertion-discrimination sweep (round 2, verdict corrected this
    // round): unlike `Cylinder`'s two sequential guards (see the doc
    // comments above), `Cuboid::recompute` rejects all three axes
    // through one combined `half_length < 0.0 || half_width < 0.0 ||
    // half_height < 0.0` check with a single shared message -- an
    // earlier revision of this comment called that `single-branch`
    // because there is no per-axis message to confuse. "One `Error::`
    // token" is not "one cause" (brief section 3, `search_position_ik`).
    // Bite-checked: neutralizing the `half_length` clause alone fails
    // this test's first assertion (length) and leaves the third
    // (height) green; the same shape as `shapes::Cuboid::new`, corrected
    // above. Verdict `multi-branch`, discriminating -- each axis's
    // assertion is its own site.
    #[test]
    fn cuboid_negative_dimension_is_an_error_per_axis() {
        assert!(Cuboid::new(-1.0, 1.0, 1.0).is_err());
        assert!(Cuboid::new(1.0, -1.0, 1.0).is_err());
        assert!(Cuboid::new(1.0, 1.0, -1.0).is_err());
    }

    /// Boundary: padding negative enough to invert the sphere's scaled
    /// radius is rejected, and the sphere is left in its previous valid
    /// state (see the module docs' "no dirty/clean setter pair" design
    /// note).
    // Assertion-discrimination sweep (round 2): `Sphere::set_padding`
    // has exactly one `Err` site -- verdict `single-branch`, one
    // `Error::` hit over the function body.
    #[test]
    fn sphere_padding_inversion_is_rejected_and_state_preserved() {
        let mut sphere = Sphere::new(1.0).unwrap();
        assert!(sphere.set_padding(-2.0).is_err());
        assert_eq!(sphere.padding(), 0.0);
        assert!(sphere.contains_point(&Vector3::new(1.0, 0.0, 0.0)));
    }

    // Assertion-discrimination sweep (round 2): `Cylinder::recompute` has
    // *two* sequential guards, "radius must be non-negative" and "length
    // must be non-negative", each with its own message. `radius = length
    // = 1.0` and `padding = -2.0` drives both scaled dimensions negative
    // at once, so a bare `.is_err()` doesn't prove the radius guard is
    // what fired -- confirmed by disabling the radius guard alone
    // (`if false && radius_scaled < 0.0`) and re-running: the test still
    // passed, silently covered by the length guard instead. Fixed by
    // asserting the message names "radius" specifically, which fails
    // correctly under that same mutation (the length guard's message
    // does not contain "radius") and also under the discrimination bite
    // (radius guard's message swapped to the length guard's text,
    // condition left intact): both mutations reddened the fixed
    // assertion. Reverted before this comment was written.
    #[test]
    fn cylinder_padding_inversion_is_rejected_and_state_preserved() {
        let mut cylinder = Cylinder::new(1.0, 1.0).unwrap();
        let err = cylinder.set_padding(-2.0).unwrap_err();
        assert!(
            err.to_string().contains("radius"),
            "expected the radius guard to name itself, got: {err}"
        );
        assert_eq!(cylinder.padding(), 0.0);
        assert!(cylinder.contains_point(&Vector3::new(1.0, 0.0, 0.0)));
    }

    // Assertion-discrimination sweep (round 2): `Cuboid::set_padding` ->
    // `recompute` has the same single combined guard as
    // `cuboid_negative_dimension_is_an_error_per_axis` above (no
    // per-axis message to confuse) -- verdict `single-branch`.
    #[test]
    fn cuboid_padding_inversion_is_rejected_and_state_preserved() {
        let mut cuboid = Cuboid::new(1.0, 1.0, 1.0).unwrap();
        assert!(cuboid.set_padding(-2.0).is_err());
        assert_eq!(cuboid.padding(), 0.0);
        assert!(cuboid.contains_point(&Vector3::new(0.5, 0.0, 0.0)));
    }

    // --- AABB / OBB merges: literal ground truth from
    // --- test/test_bounding_box.cpp ---

    #[test]
    fn merge_bounding_boxes_two_unit_aabbs() {
        // MergeBoundingBoxes::Merge1
        let boxes = [
            AABB::new(Vector3::new(-1.0, -1.0, -1.0), Vector3::new(0.0, 0.0, 0.0)),
            AABB::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 1.0, 1.0)),
        ];
        let merged = merge_bounding_boxes(&boxes);
        // Bit-exact (round 14, §79): bisected to epsilon = 0.0, max_relative
        // = 0.0 and still passed. Axis-aligned min/max of unit boxes at
        // integer-valued corners is a min/max reduction over exact ±1.0
        // literals, with no arithmetic to round.
        assert_eq!(merged.min(), Vector3::new(-1.0, -1.0, -1.0));
        assert_eq!(merged.max(), Vector3::new(1.0, 1.0, 1.0));
    }

    #[test]
    fn obb_extend_approx_bootstraps_from_zero_extent() {
        // MergeBoundingBoxes::OBBInvalid
        let rot = nalgebra::UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::new(1.0, 1.0, 1.0)),
            std::f64::consts::FRAC_PI_2,
        );
        let pose = Isometry3::from_parts(nalgebra::Translation3::new(-0.6, -0.6, -0.6), rot);
        let mut b1 = OBB::default();
        let b2 = OBB::new(pose, Vector3::new(0.1, 0.1, 0.1));

        b1.extend_approx(&b2);

        assert!(b1.overlaps(&b2));
        assert!(b2.overlaps(&b1));
        // Bit-exact (round 14, §79): bisected to epsilon = 0.0, max_relative
        // = 0.0 and still passed. Bootstrapping from a zero-extent `b1`
        // means `extend_approx` just copies `b2`'s own extents, pose
        // translation, and rotation matrix, so every field compares a
        // freshly-copied value against its own source, not a recomputed one.
        assert_eq!(b1.extents(), Vector3::new(0.1, 0.1, 0.1));
        assert_eq!(b1.pose().translation.vector, Vector3::new(-0.6, -0.6, -0.6));
        assert_eq!(
            b1.pose().rotation.to_rotation_matrix().matrix(),
            pose.rotation.to_rotation_matrix().matrix()
        );
    }

    #[test]
    fn obb_extend_approx_noop_when_self_contains_other() {
        // MergeBoundingBoxes::OBBContains1
        let mut b1 = OBB::new(
            Isometry3::translation(-0.5, -0.5, -0.5),
            Vector3::new(1.0, 1.0, 1.0),
        );
        let rot = nalgebra::UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::new(1.0, 1.0, 1.0)),
            std::f64::consts::FRAC_PI_2,
        );
        let b2 = OBB::new(
            Isometry3::from_parts(nalgebra::Translation3::new(-0.6, -0.6, -0.6), rot),
            Vector3::new(0.1, 0.1, 0.1),
        );

        assert!(b1.contains_obb(&b2));
        assert!(!b2.contains_obb(&b1));

        b1.extend_approx(&b2);

        assert!(b1.contains_obb(&b2));
        // Bit-exact (round 14, §79): bisected to epsilon = 0.0, max_relative
        // = 0.0 and still passed. `self.contains_obb(other)` takes the
        // early `return` in `extend_approx`, leaving `b1` byte-for-byte as
        // constructed above -- nothing here was recomputed.
        assert_eq!(b1.extents(), Vector3::new(1.0, 1.0, 1.0));
        assert_eq!(b1.pose().translation.vector, Vector3::new(-0.5, -0.5, -0.5));
    }

    #[test]
    fn obb_extend_approx_becomes_other_when_other_contains_self() {
        // MergeBoundingBoxes::OBBContains2
        let b1 = OBB::new(
            Isometry3::translation(-0.5, -0.5, -0.5),
            Vector3::new(1.0, 1.0, 1.0),
        );
        let rot = nalgebra::UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(Vector3::new(1.0, 1.0, 1.0)),
            std::f64::consts::FRAC_PI_2,
        );
        let mut b2 = OBB::new(
            Isometry3::from_parts(nalgebra::Translation3::new(-0.6, -0.6, -0.6), rot),
            Vector3::new(0.1, 0.1, 0.1),
        );

        b2.extend_approx(&b1);

        // Bit-exact (round 14, §79): bisected to epsilon = 0.0, max_relative
        // = 0.0 and still passed. `other.contains_obb(self)` takes the
        // `*self = *other` branch in `extend_approx`, an exact struct copy
        // of `b1` into `b2` -- nothing here was recomputed.
        assert_eq!(b2.extents(), Vector3::new(1.0, 1.0, 1.0));
        assert_eq!(b2.pose().translation.vector, Vector3::new(-0.5, -0.5, -0.5));
    }

    /// Upstream's own test for this branch only asserts loose sanity
    /// bounds (`test_bounding_box.cpp`'s `OBBApprox1`) — see the module
    /// docs' FCL source-availability note for why this port's own
    /// general-merge formula is checked against those loose bounds
    /// rather than an exact literal.
    #[test]
    fn obb_extend_approx_general_merge_loose_bounds() {
        // MergeBoundingBoxes::OBBApprox1
        let boxes = [
            OBB::new(
                Isometry3::translation(-0.5, -0.5, -0.5),
                Vector3::new(1.0, 1.0, 1.0),
            ),
            OBB::new(
                Isometry3::translation(0.5, 0.5, 0.5),
                Vector3::new(1.0, 1.0, 1.0),
            ),
        ];
        let merged = merge_bounding_boxes_approx(&boxes);

        for axis in 0..3 {
            assert!(merged.extents()[axis] <= 2.1);
            assert!(merged.extents()[axis] >= 2.0);
            assert!(merged.pose().translation.vector[axis] <= 0.1);
            assert!(merged.pose().translation.vector[axis] >= -0.1);
        }
        assert!(merged.contains_point(&boxes[0].pose().translation.vector));
        assert!(merged.contains_point(&boxes[1].pose().translation.vector));
        assert!(merged.overlaps(&boxes[0]));
        assert!(merged.overlaps(&boxes[1]));
    }

    /// Upstream `OBB::operator+`'s `max_extent = std::max(std::max(extent[0],
    /// extent[1]), extent[2])` propagates a NaN `extent[0]` (`.x`) out
    /// through both nested calls (it is the first operand of each), which
    /// then makes `center_diff.norm() > 2*(max_extent+max_extent2)` false --
    /// every comparison against NaN is false -- so upstream always takes
    /// `merge_smalldist` once `.x` is NaN, regardless of how far apart the
    /// centers actually are. `b1`/`b2`'s centers below are 1.0 apart, well
    /// past `2*(0.1+0.1) = 0.4`: far enough that a correctly-propagated NaN
    /// forces `merge_smalldist`, but a *silently discarded* one (the
    /// pre-fix `.x.max(y).max(z)`, which picks `max(0.1, 0.1) = 0.1` instead
    /// of NaN) clears that threshold and takes `merge_largedist` instead --
    /// asserting the chosen branch, not just `max_extent`'s value, is what
    /// this test is for.
    #[test]
    fn extend_approx_takes_merge_smalldist_when_a_half_extent_x_is_nan() {
        let b1 = OBB::new(
            Isometry3::translation(0.0, 0.0, 0.0),
            Vector3::new(f64::NAN, 0.2, 0.2),
        );
        let b2 = OBB::new(
            Isometry3::translation(1.0, 0.0, 0.0),
            Vector3::new(0.2, 0.2, 0.2),
        );

        let mut merged = b1;
        merged.extend_approx(&b2);

        let smalldist = merge_smalldist(&b1, &b2);
        let largedist = merge_largedist(&b1, &b2);
        assert_ne!(
            smalldist, largedist,
            "the two branches must be distinguishable for this test to mean anything"
        );
        assert_eq!(merged, smalldist);
    }

    /// The demonstrated opposite of
    /// `extend_approx_takes_merge_smalldist_when_a_half_extent_x_is_nan`:
    /// the same NaN moved to `.y` (or `.z`) is the position upstream's
    /// chained `std::max` *discards* (`std::max(a, b) = a<b?b:a`; NaN as
    /// the second operand makes the comparison false and returns `a`), so
    /// upstream and the port agree here even before the fix -- without this
    /// case, the test above would also pass on a port that propagated NaN
    /// through every position, which is not the bug that was fixed.
    #[test]
    fn extend_approx_agrees_with_upstream_when_the_nan_is_at_a_discarded_position() {
        let b1 = OBB::new(
            Isometry3::translation(0.0, 0.0, 0.0),
            Vector3::new(0.2, f64::NAN, 0.2),
        );
        let b2 = OBB::new(
            Isometry3::translation(1.0, 0.0, 0.0),
            Vector3::new(0.2, 0.2, 0.2),
        );

        let mut merged = b1;
        merged.extend_approx(&b2);

        // Upstream's max_extent = std::max(std::max(NaN, 0.1), 0.1):
        // NaN discarded at the first (inner) call since it's the *second*
        // operand there, leaving std::max(0.1, 0.1) = 0.1 -- a finite
        // max_extent, same as `.x.max(y).max(z)` computes. Both sides take
        // merge_largedist: center_diff.norm() = 1.0 > 2*(0.1+0.1) = 0.4.
        let largedist = merge_largedist(&b1, &b2);
        let smalldist = merge_smalldist(&b1, &b2);
        assert_ne!(
            debug_string(&smalldist),
            debug_string(&largedist),
            "the two branches must be distinguishable for this test to mean anything"
        );
        // `b1.half_extents.y` being NaN also feeds `compute_vertices`
        // directly (every corner has a `+-e.y` component), which poisons
        // `merge_largedist`'s covariance/PCA sum into an all-NaN pose --
        // unrelated to the branch-selection bug this test is for, and NaN
        // != NaN under plain `assert_eq!`, so this compares the `Debug`
        // strings instead: both sides genuinely agree here, NaNs and all.
        assert_eq!(debug_string(&merged), debug_string(&largedist));
    }

    /// `guarded_normalize` unit tests: the zero case `merge_largedist`
    /// exists for, and its demonstrated opposite (a nonzero input, where
    /// `try_normalize` and the unguarded `.normalize()` agree).
    #[test]
    fn guarded_normalize_leaves_a_zero_vector_unchanged_instead_of_producing_nan() {
        assert_eq!(guarded_normalize(Vector3::zeros()), Vector3::zeros());
    }

    #[test]
    fn guarded_normalize_still_normalizes_a_nonzero_vector() {
        let v = guarded_normalize(Vector3::new(3.0, 0.0, 4.0));
        assert!((v.norm() - 1.0).abs() < 1e-12);
        assert_eq!(v, Vector3::new(3.0, 0.0, 4.0).normalize());
    }

    /// Nothing in this crate's public API rejects a negative half-extent:
    /// `OBB::new`/`set_pose_and_extents` just multiply the given extents by
    /// 0.5 (see both). A box built with `Vector3::new(-2.0, -2.0, -2.0)`
    /// has `half_extents == (-1.0, -1.0, -1.0)`, so `max_extent ==
    /// cxx_max(cxx_max(-1.0, -1.0), -1.0) == -1.0` (finite values fold
    /// exactly like `f64::max` there -- only NaN makes `cxx_max` diverge
    /// from it). Two such boxes give `max_extent + other_max_extent ==
    /// -2.0`, and `center_diff.norm() > 2.0 * -2.0` is `0.0 > -4.0`: true
    /// even for identical centers. `contains_obb` doesn't screen this
    /// first: `contains_point`'s `local.x.abs() <= self.half_extents.x` is
    /// `<= a negative number`, which is false for every point, so both
    /// `self.contains_obb(other)` and `other.contains_obb(self)` are false
    /// and execution reaches the merge dispatch. This is the reachable
    /// case `guarded_normalize` exists for: `a.To - b.To` is the genuine
    /// zero vector here, not merely a very small one.
    #[test]
    fn extend_approx_avoids_a_nan_axis_when_negative_extents_zero_the_merge_largedist_threshold() {
        let b1 = OBB::new(
            Isometry3::translation(0.0, 0.0, 0.0),
            Vector3::new(-2.0, -2.0, -2.0),
        );
        let b2 = OBB::new(
            Isometry3::translation(0.0, 0.0, 0.0),
            Vector3::new(-2.0, -2.0, -2.0),
        );

        let smalldist = merge_smalldist(&b1, &b2);
        let largedist = merge_largedist(&b1, &b2);
        assert_ne!(
            debug_string(&smalldist),
            debug_string(&largedist),
            "the two branches must be distinguishable for this test to mean anything"
        );

        let mut merged = b1;
        merged.extend_approx(&b2);

        assert_eq!(
            debug_string(&merged),
            debug_string(&largedist),
            "must actually take merge_largedist for this test to exercise the fix"
        );
        assert!(
            !merged.pose().translation.vector.iter().any(|c| c.is_nan()),
            "a zero-norm axis0 must not leak NaN into the merged pose: {merged:?}"
        );
        assert!(
            !merged.extents().iter().any(|c| c.is_nan()),
            "a zero-norm axis0 must not leak NaN into the merged extents: {merged:?}"
        );
    }

    /// The demonstrated opposite of the test above: distinct centers give
    /// `guarded_normalize` a nonzero input, where it behaves exactly like
    /// the unguarded `.normalize()` (both produce a proper unit vector) --
    /// so this scenario's result is unaffected by the fix. Without this
    /// case, the test above would also pass on a `guarded_normalize` that
    /// always returned the zero vector regardless of input, which is not
    /// what the fix does.
    #[test]
    fn extend_approx_still_takes_a_unit_axis0_when_centers_are_genuinely_distinct() {
        let b1 = OBB::new(
            Isometry3::translation(0.0, 0.0, 0.0),
            Vector3::new(0.2, 0.2, 0.2),
        );
        let b2 = OBB::new(
            Isometry3::translation(10.0, 0.0, 0.0),
            Vector3::new(0.2, 0.2, 0.2),
        );

        let smalldist = merge_smalldist(&b1, &b2);
        let largedist = merge_largedist(&b1, &b2);
        assert_ne!(
            debug_string(&smalldist),
            debug_string(&largedist),
            "the two branches must be distinguishable for this test to mean anything"
        );

        let mut merged = b1;
        merged.extend_approx(&b2);

        assert_eq!(debug_string(&merged), debug_string(&largedist));
        assert!(!merged.pose().translation.vector.iter().any(|c| c.is_nan()));
        assert!(!merged.extents().iter().any(|c| c.is_nan()));
    }

    #[test]
    fn merge_smalldist_matches_true_min_max_not_fcls_order_dependent_replica() {
        // Module docs, deviation 10: FCL's own `merge_smalldist`
        // (`OBB-inl.h:341-345,355-359`) tracks the per-axis running min/max
        // with `if (dot > pmax[j]) pmax[j] = dot; else if (dot < pmin[j])
        // pmin[j] = dot;` -- mutually exclusive, so a vertex that updates
        // `pmax[j]` is never also considered for `pmin[j]`. Depending on
        // vertex-visitation order this can leave `pmin[j]` at its
        // `real_max` sentinel (never set) or otherwise too large, which is
        // an upstream bug, not upstream's intended semantics. This port
        // uses two independent `if`s, which always compute the true
        // componentwise min/max regardless of order. These fixed
        // (arbitrary but reproducible) box configurations are one case
        // where the two disagree; found by a deterministic xorshift64
        // search over ~200k random OBB pairs (seed 12345, first hit at
        // trial 3) filtered to the `merge_smalldist`-selecting distance
        // condition `OBB::extend_approx` itself uses.
        let a = OBB {
            pose: Isometry3::from_parts(
                Vector3::new(
                    -0.1777397387377112,
                    -0.0026174820415298394,
                    -0.07073199985140266,
                )
                .into(),
                UnitQuaternion::from_euler_angles(
                    3.195825488004906,
                    3.524268228416687,
                    5.145759698588444,
                ),
            ),
            half_extents: Vector3::new(1.9409112600385845, 1.731226599362296, 0.42932324056205884),
        };
        let b = OBB {
            pose: Isometry3::from_parts(
                Vector3::new(
                    0.007680239521112575,
                    -0.0022048760613254115,
                    0.1928448311361619,
                )
                .into(),
                UnitQuaternion::from_euler_angles(
                    4.090495798141434,
                    3.211304474174798,
                    3.3319936050087446,
                ),
            ),
            half_extents: Vector3::new(1.7458628505727363, 1.5533248412342824, 1.9171102999768692),
        };

        let port_result = merge_smalldist(&a, &b);

        // Independent oracle: the true componentwise min/max of all 16
        // vertices' projections onto the merged axes, via `Iterator::min`/
        // `max` -- not a replica of either `if`-shape under test, so it
        // cannot silently agree with either one by construction.
        let center = (a.pose.translation.vector + b.pose.translation.vector) * 0.5;
        let axis = port_result.pose.rotation.to_rotation_matrix();
        let axis_cols = [
            axis.matrix().column(0).into_owned(),
            axis.matrix().column(1).into_owned(),
            axis.matrix().column(2).into_owned(),
        ];
        let all_vertices: Vec<Vector3> = a
            .compute_vertices()
            .into_iter()
            .chain(b.compute_vertices())
            .collect();
        for (j, axis_j) in axis_cols.iter().enumerate() {
            let projections: Vec<f64> = all_vertices
                .iter()
                .map(|v| (v - center).dot(axis_j))
                .collect();
            let true_min = projections.iter().copied().fold(f64::MAX, f64::min);
            let true_max = projections.iter().copied().fold(f64::MIN, f64::max);
            assert_relative_eq!(
                port_result.half_extents[j],
                0.5 * (true_max - true_min),
                epsilon = 1e-9
            );
        }

        // And confirm FCL's own `else if` shape really does disagree on
        // this input, so this test cannot pass by both sides coincidentally
        // producing the same answer.
        fn fcl_merge_smalldist_replica(a: &OBB, b: &OBB) -> Vector3 {
            let center = (a.pose.translation.vector + b.pose.translation.vector) * 0.5;
            let q0 = a.pose.rotation;
            let q1 = b.pose.rotation;
            let q1_coords = if q0.coords.dot(&q1.coords) < 0.0 {
                -q1.coords
            } else {
                q1.coords
            };
            let merged_rotation = UnitQuaternion::from_quaternion(
                nalgebra::Quaternion::from_vector(q0.coords + q1_coords),
            );
            let axis = merged_rotation.to_rotation_matrix();
            let axis_cols = [
                axis.matrix().column(0).into_owned(),
                axis.matrix().column(1).into_owned(),
                axis.matrix().column(2).into_owned(),
            ];
            let mut pmin = Vector3::from_element(f64::MAX);
            let mut pmax = Vector3::from_element(-f64::MAX);
            for obb in [a, b] {
                for v in obb.compute_vertices() {
                    let diff = v - center;
                    for (j, axis_j) in axis_cols.iter().enumerate() {
                        let dot = diff.dot(axis_j);
                        if dot > pmax[j] {
                            pmax[j] = dot;
                        } else if dot < pmin[j] {
                            pmin[j] = dot;
                        }
                    }
                }
            }
            Vector3::new(
                0.5 * (pmax[0] - pmin[0]),
                0.5 * (pmax[1] - pmin[1]),
                0.5 * (pmax[2] - pmin[2]),
            )
        }
        let fcl_extents = fcl_merge_smalldist_replica(&a, &b);
        assert!(
            (port_result.half_extents - fcl_extents).abs().max() > 1e-6,
            "fixture no longer exercises the known FCL merge_smalldist divergence -- \
             replace it with a new one found the same way, do not delete this test"
        );
    }

    #[test]
    fn merge_bounding_spheres_two_spheres() {
        // MergeBoundingSpheres::MergeTwoSpheres
        let spheres = [
            BoundingSphere {
                center: Vector3::new(5.0, 0.0, 0.0),
                radius: 1.0,
            },
            BoundingSphere {
                center: Vector3::new(-5.1, 0.0, 0.0),
                radius: 1.0,
            },
        ];
        let merged = merge_bounding_spheres(&spheres);
        // NOT bit-exact (round 14, §79): bisected alone, unlike
        // `merged.radius` below -- fails at epsilon = 0.0 (measured left =
        // -0.04999999999999982, right = -0.05, diff ~1.8e-16), passes at
        // 1e-15, fails at 1e-16. epsilon = 1e-13 is real, measured
        // headroom, not the file's old 1e-5 carried over unmeasured.
        assert_relative_eq!(merged.center.x, -0.05, epsilon = 1e-13, max_relative = 0.0);
        // Bit-exact (round 14, §79): bisected alone (not grouped with
        // `merged.center.x` above) to epsilon = 0.0, max_relative = 0.0 and
        // still passed.
        assert_eq!(merged.radius, 6.05);
    }

    // --- Body::from_shape ---

    // Assertion-discrimination sweep (round 2): each of this test's four
    // `assert!(matches!(...))` calls names a different Some(Body::_)-producing
    // arm of `from_shape`'s `match shape { ... }` (lines 3109-3114) --
    // Sphere/Cylinder/Cuboid/Mesh each get their own arm, each building a
    // different concrete `Body` variant.
    //
    // Verdict `multi-branch`, discriminating. An earlier revision of this
    // comment recorded `single-branch`, reasoning that Rust's exhaustive
    // match on a known, concrete input variant already proves only the
    // matching arm executes, leaving nothing for an isolating mutation to
    // separate. That conflates which arm's *pattern* matches -- fixed by the
    // type system, and indeed unmutatable -- with what that arm's *body*
    // computes, which is ordinary code fixed by nothing. Bite-checked:
    // rewriting the `Shape::Sphere` arm to build a `Cylinder` compiles, and
    // fails both this test and `body_posed_algorithms_match_the_oracle`. The
    // same correction applies to `from_shape_returns_none_for_cone_plane_octree`
    // below, where splitting `Cone` out of the combined `None` arm and
    // returning `Some` is likewise caught.
    #[test]
    fn from_shape_builds_matching_body_variant() {
        assert!(matches!(
            Body::from_shape(&Shape::Sphere(crate::geometry::shapes::Sphere {
                radius: 1.0
            }))
            .unwrap(),
            Some(Body::Sphere(_))
        ));
        assert!(matches!(
            Body::from_shape(&Shape::Cylinder(crate::geometry::shapes::Cylinder {
                radius: 1.0,
                length: 1.0
            }))
            .unwrap(),
            Some(Body::Cylinder(_))
        ));
        assert!(matches!(
            Body::from_shape(&Shape::Cuboid(crate::geometry::shapes::Cuboid {
                size: [1.0, 1.0, 1.0]
            }))
            .unwrap(),
            Some(Body::Cuboid(_))
        ));
        assert!(matches!(
            Body::from_shape(&Shape::Mesh(box_mesh(1.0, 1.0, 1.0))).unwrap(),
            Some(Body::ConvexMesh(_))
        ));
    }

    // Assertion-discrimination sweep (round 2, D6 check per brief section
    // 3a added; `single-branch` verdict below corrected this round):
    // `Body::from_shape` has exactly one `None`-producing arm,
    // `Shape::Cone(_) | Shape::Plane(_) | Shape::OcTree(_) => None`. The
    // original verdict read Rust's exhaustive `match` on the *input*
    // variant ("only that arm's pattern can match") as proof there was
    // "nothing for an isolating mutation to separate" -- but the type
    // system only fixes which arm's *pattern* matches a given input, not
    // what that arm's *body* computes. Splitting any one of
    // Cone/Plane/OcTree out of the combined arm into its own
    // `Shape::X(_) => Some(Sphere::new(1.0)?.into())` still compiles
    // (each pattern is independently constructible) and fails exactly
    // that variant's assertion below while leaving the other two green
    // -- confirmed live for all three (Cone: line 4356 fails; Plane:
    // 4364; OcTree: 4374). Verdict `multi-branch`, discriminating: this
    // is a real, passing test, not an unreachable assertion. D6 check,
    // from the actual call sites, not the signature, still stands:
    // every in-tree caller (`cspace_planning::constraints::position::
    // PositionConstraint::new`, `cspace_collision::distance_field::distance_field::
    // posed_body`, `cspace_collision::distance_field::collision_distance_field_types
    // ::BodyDecomposition::from_shapes`) uses `Body::from_shape(shape)?
    // .ok_or_else(|| Error::construct(format!("... {shape:?}")))` --
    // they format the *caller's own copy* of `shape` into the error
    // message, not anything read back out of the `None`, so none of them
    // needs `from_shape` itself to say which of Cone/Plane/OcTree fired.
    // `cspace-constraints/tests/decide.rs` independently confirms this
    // for `PositionConstraint::new`, testing `Cone` and `Plane` as
    // interchangeable members of one `bodyless` list against the same
    // `"has no bodies:: counterpart"` message. Not a D6 finding.
    #[test]
    fn from_shape_returns_none_for_cone_plane_octree() {
        assert!(
            Body::from_shape(&Shape::Cone(crate::geometry::shapes::Cone {
                radius: 1.0,
                length: 1.0
            }))
            .unwrap()
            .is_none()
        );
        assert!(
            Body::from_shape(&Shape::Plane(crate::geometry::shapes::Plane {
                a: 0.0,
                b: 0.0,
                c: 1.0,
                d: 0.0
            }))
            .unwrap()
            .is_none()
        );
        assert!(
            Body::from_shape(&Shape::OcTree(crate::geometry::shapes::OcTree::new()))
                .unwrap()
                .is_none()
        );
    }

    // --- samplePointInside: this port's own property tests (see the
    // --- module docs, deviation 5) — not upstream's exact RNG sequence,
    // --- but the same invariant upstream checks: a sampled point is
    // --- always contained. ---

    #[test]
    fn sphere_sample_point_inside_is_contained() {
        let mut uniform = uniform_test_rng(1);
        for _ in 0..200 {
            let mut sphere = Sphere::new(1.0).unwrap();
            sphere.set_scale(uniform(0.1, 10.0)).unwrap();
            sphere.set_padding(uniform(-0.05, 5.0)).unwrap();
            let p = sphere
                .sample_point_inside(100, &mut uniform)
                .expect("sampling should find a point within 100 attempts");
            assert!(sphere.contains_point(&p));
        }
    }

    #[test]
    fn cylinder_sample_point_inside_is_contained() {
        let mut uniform = uniform_test_rng(2);
        for _ in 0..200 {
            let mut cylinder = Cylinder::new(1.0, 2.0).unwrap();
            cylinder.set_scale(uniform(0.1, 10.0)).unwrap();
            cylinder.set_padding(uniform(-0.05, 5.0)).unwrap();
            let p = cylinder
                .sample_point_inside(100, &mut uniform)
                .expect("sampling should find a point within 100 attempts");
            assert!(cylinder.contains_point(&p));
        }
    }

    #[test]
    fn cuboid_sample_point_inside_is_contained() {
        let mut uniform = uniform_test_rng(3);
        for _ in 0..200 {
            let mut cuboid = Cuboid::new(1.0, 2.0, 3.0).unwrap();
            cuboid.set_scale(uniform(0.1, 10.0)).unwrap();
            cuboid.set_padding(uniform(-0.05, 5.0)).unwrap();
            let p = cuboid
                .sample_point_inside(100, &mut uniform)
                .expect("sampling should find a point within 100 attempts");
            assert!(cuboid.contains_point(&p));
        }
    }

    #[test]
    fn convex_mesh_sample_point_inside_via_body_is_contained() {
        // ConvexMesh has no samplePointInside override upstream, so this
        // exercises the generic Body::sample_point_inside fallback.
        let mut uniform = uniform_test_rng(4);
        for _ in 0..50 {
            let body: Body = ConvexMesh::new(&box_mesh(1.0, 2.0, 3.0)).unwrap().into();
            let p = body
                .sample_point_inside(1000, &mut uniform)
                .expect("sampling should find a point within 1000 attempts");
            assert!(body.contains_point(&p));
        }
    }

    #[test]
    fn cylinder_ray_hits_are_symmetric_with_intersects_ray() {
        // Spot-check the intersects_ray = !ray_intersections(...).is_empty()
        // invariant this port relies on for every body kind (see the module
        // docs) rather than hand-duplicating each type's fast path.
        let cylinder = Cylinder::new(1.0, 2.0).unwrap();
        assert_eq!(
            cylinder.intersects_ray(&Vector3::new(5.0, 0.0, 0.0), &Vector3::new(-1.0, 0.0, 0.0)),
            !cylinder
                .ray_intersections(
                    &Vector3::new(5.0, 0.0, 0.0),
                    &Vector3::new(-1.0, 0.0, 0.0),
                    None
                )
                .is_empty()
        );
    }
}
