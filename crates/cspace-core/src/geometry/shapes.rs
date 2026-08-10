// Copyright 2008, 2011, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from geometric_shapes 2.3.3 — a separate upstream package that
// moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf depends on (PORTING-PLAN.md
// §1.1: 68 `#include <geometric_shapes/...>` occurrences) and that is not
// vendored anywhere on this machine (not in /home/stevek/work/moveit2, no
// source/-dbgsym package in the oracle image). Two sources, both verified:
//
// - Headers came from the oracle image's Debian package
//   (`ros-rolling-geometric-shapes` 2.3.3-1noble.20260113.113114) at
//   `/opt/ros/rolling/include/geometric_shapes/geometric_shapes/*.h`, and are
//   byte-identical (`diff`) to the `geometric_shapes` GitHub tag `2.3.3`
//   (commit `192801cebacc07d0e9f719576cdd1c9b36d0bc28`), which is the tag
//   matching the installed package version exactly.
// - `.cpp` implementations came from that same tag's source tarball, since
//   the Debian package ships headers and a compiled `.so` only. Verified
//   against `/opt/ros/rolling/lib/libgeometric_shapes.so.2.3.3`'s string
//   table: six distinct `std::runtime_error`/`CONSOLE_BRIDGE_logWarn`
//   literals unique to `shapes.cpp` ("Sphere radius must be non-negative.",
//   "Cylinder dimensions must be non-negative.", "Cone dimensions must be
//   non-negative.", "Box dimensions must be non-negative.", "OcTrees cannot
//   be scaled or padded", "Planes cannot be scaled or padded") each appear
//   exactly once in `strings libgeometric_shapes.so.2.3.3`.
//
// Files read in full:
//   geometric_shapes/include/geometric_shapes/shapes.h
//   geometric_shapes/src/shapes.cpp
//   geometric_shapes/include/geometric_shapes/shape_operations.h
//   geometric_shapes/src/shape_operations.cpp
//     (only `computeShapeExtents` and `computeShapeBoundingSphere`; the rest
//     of that file converts to/from `shape_msgs`/`visualization_msgs`, which
//     PORTING-PLAN.md D1 keeps out of the core crates entirely)
//   geometric_shapes/include/geometric_shapes/bodies.h
//     (read to confirm which of `computeVolume`/`getDimensions` have a
//     `shapes::Shape`-level upstream counterpart at all — see the module
//     docs below)

//! The `geometric_shapes` shape layer: dimensioned, unposed geometric
//! primitives. Upstream `namespace shapes` from the `geometric_shapes`
//! package (not `moveit_core`) — see the provenance comment above for how
//! the source was obtained and verified.
//!
//! # Scope
//!
//! This module ports the **shape** data layer only: [`Shape`] and its seven
//! variants, [`ShapeType`], each variant's `scaleAndPadd`, and the two
//! unposed geometry helpers `computeShapeExtents`/`computeShapeBoundingSphere`
//! (here [`Shape::extents`]/[`Shape::bounding_sphere`]).
//!
//! The posed-body algorithms upstream's `namespace bodies` layers on top
//! of these shapes — `containsPoint`, `intersectsRay`, and
//! `computeBoundingBox`/`computeBoundingCylinder` on a *posed* body, plus
//! the `bodies::AABB`/`bodies::OBB` types those return — live in the
//! sibling [`crate::geometry::bodies`] module, not here; see its module docs for
//! scope and provenance.
//!
//! [`Shape::compute_volume`] and [`Shape::get_dimensions`] are a partial
//! exception worth explaining, because upstream does not define either on
//! `shapes::Shape` at all — `bodies.h` shows `computeVolume`/`getDimensions`
//! are pure-virtual members of `bodies::Body`, not `shapes::Shape`. Reading
//! `bodies.cpp` shows why porting the dimension-only part is still faithful:
//! `bodies::Sphere::computeVolume`/`getDimensions` depend only on the
//! shape's own dimensions plus `Body::scale_`/`padding_` (never `pose_`), and
//! `shapes::Shape::scaleAndPadd` already mutates a shape's dimensions in
//! place — so calling [`Shape::scale_and_padd`] then [`Shape::compute_volume`]
//! reproduces `Body::setScale`/`setPadding` then `Body::computeVolume()`
//! exactly, without needing `Body`'s pose or its scale/padding cache. Only
//! three of the eight `ShapeType` variants have a `bodies::` counterpart at
//! all upstream — `Sphere`, `Cylinder`, `Box` (here [`Cuboid`], see below).
//! There is no `bodies::Cone`; [`Shape::compute_volume`] and
//! [`Shape::get_dimensions`] return `None` for [`Shape::Cone`] rather than
//! guess a formula upstream never wrote. `bodies::ConvexMesh` exists but its
//! `computeVolume`/`getDimensions` are posed-body algorithms (qhull-backed
//! convex hull, `useDimensions`/`computeScaledVerticesFromPlaneProjections`)
//! that belong to [`crate::geometry::bodies::ConvexMesh`], not this module;
//! [`Shape::Mesh`] still returns `None` here. `Plane` and `OcTree` have no
//! `bodies::` counterpart either.
//!
//! # Design: enum, not a trait-object hierarchy (D4)
//!
//! Upstream `shapes::Shape` is an abstract base class with `Sphere`,
//! `Cylinder`, `Cone`, `Box`, `Mesh`, `Plane`, `OcTree` as concrete
//! subclasses, dispatched through `virtual` calls and `dynamic_cast`/
//! `static_cast<const Sphere*>(shape)` on a runtime `type` tag. PORTING-PLAN.md
//! D4 asks for a sum type here instead of a trait-object hierarchy, and there
//! is a concrete reason beyond D4's default: upstream's `ShapeType type`
//! field and the concrete class are two separate facts the compiler does not
//! relate — `static_cast<const Sphere*>(shape)` after checking
//! `shape->type == SPHERE` is exactly the "value's meaning depends on a
//! side tag" pattern the dirty-flag redesign in PORTING-PLAN.md §8.2
//! replaced for `RobotState`. A closed `enum Shape { Sphere(Sphere), ... }`
//! makes the tag-matches-payload invariant hold by construction: matching on
//! the enum *is* the type check, so there is no cast that can be wrong.
//!
//! # Deviations from upstream
//!
//! 1. **`Box` is renamed [`Cuboid`].** `shapes::Box` would shadow
//!    `std::boxed::Box` at every use site in this crate; the type is
//!    otherwise a direct field-for-field port. [`ShapeType::Box`] keeps
//!    upstream's name since enum variants live in their own namespace and
//!    there is no collision.
//! 2. **`Mesh` owns `Vec`s, not `count` + raw pointer pairs.** Upstream
//!    `Mesh::vertex_count` and `Mesh::vertices` (`double*`) are two facts a
//!    caller must keep in sync by convention; nothing stops
//!    `vertex_count > vertices.len()`-worth-of-allocation once the mesh
//!    exists. [`Mesh::vertices`] is a `Vec<Vector3>`, so the count is always
//!    `vertices.len()` — the same "explicit state over implicit value plus a
//!    side counter" move as §8.2. [`Mesh::triangles`] is `Vec<[u32; 3]>`
//!    rather than a flat `Vec<u32>` of length `3 * triangle_count`, for the
//!    same reason.
//! 3. **[`Mesh::new`] rejects out-of-range triangle indices.** Upstream never
//!    checks that a triangle's vertex indices are `< vertex_count`; an
//!    out-of-range index is a read past the end of the `vertices` heap
//!    allocation the first time any per-triangle method runs. This port
//!    validates once at construction (this crate's established boundary for
//!    "reject malformed input outright" — see [`Error::Construct`] on
//!    [`crate::geometry::Transforms::new`]) instead of leaving a `Vec` index that can
//!    panic deep inside [`Mesh::compute_triangle_normals`] or
//!    [`Mesh::merge_vertices`].
//! 4. **Mesh scale/padding requires vertex normals up front, as an
//!    error, not a null-pointer read.** `Mesh::scaleAndPadd` in `shapes.cpp`
//!    unconditionally reads `vertex_normals[i3]` — for *every* call,
//!    including scale-only convenience wrappers with padding `0.0` — because
//!    the per-vertex padding direction comes from the vertex normal, not the
//!    coordinate axes (see the type docs on [`Mesh`]). A `Mesh` built by
//!    [`Mesh::new`] has `vertex_normals: None` until
//!    [`Mesh::compute_vertex_normals`] is called (mirroring that upstream
//!    leaves the array genuinely uninitialized — `new double[n]` without
//!    `()` — until `computeVertexNormals()` runs), so the equivalent
//!    read in upstream is a null-pointer dereference on a fresh mesh, not
//!    merely a logic bug. [`Mesh::scale_and_padd_axes`] turns that into
//!    [`Error::Construct`].
//! 5. **The `OcTree` payload is `Arc<crate::octomap::OcTree>`, not
//!    `Rc`/an owned value.** Round 1 left [`OcTree`] a unit struct
//!    (PORTING-PLAN.md §2's "성숙도 미달" gap); `cspace-octomap` (this
//!    crate's sibling, also this round) closes it. Upstream's field is
//!    `std::shared_ptr<const octomap::OcTree>` — shared, read-only ownership
//!    — and `OcTree::clone()` (`shapes.cpp`) returns `new OcTree(octree)`,
//!    reusing the same `shared_ptr` rather than deep-copying the tree.
//!    `Arc<crate::octomap::OcTree>` reproduces both facts at once: cloning a
//!    [`Shape::OcTree`] is an `O(1)` refcount bump sharing the same tree
//!    (matching upstream's shallow `clone()`), and nothing in this crate ever
//!    calls `Arc::get_mut` (matching upstream's `const`). See [`OcTree`]'s
//!    own doc comment for the corresponding [`PartialEq`] deviation.
//!
//!    A former item 5 here claimed degenerate triangles were a deviation —
//!    "`Eigen`'s `Vector3d::normalize()` on a zero-length cross product
//!    divides by zero, producing `NaN`" — measured false against real Eigen
//!    3.4.0 (`moveit-rs/oracle:ccc22ff0287a603f`, `/usr/include/eigen3`):
//!    an in-place `.normalize()` call on a zero vector, unguarded at the
//!    call site exactly as `Mesh::computeTriangleNormals` (`shapes.cpp:503-504`)
//!    writes it, itself stays `[0, 0, 0]`, not `NaN` — `normalize()`'s own
//!    internal guard leaves a zero-norm input unchanged, same as its
//!    documented `normalized()` counterpart. [`Mesh::compute_triangle_normals`]'s
//!    `try_normalize(0.0)` was never deviating from upstream; both sides
//!    agree. See that function's doc comment for the corrected citation.
//!
//! # `shapes.h` / `shape_operations.h` symbol audit (round 8)
//!
//! Every public declaration in the oracle container's installed
//! `geometric_shapes/include/geometric_shapes/shapes.h` and
//! `shape_operations.h`, classified so the next audit re-runs this instead of
//! re-deriving it.
//!
//! **Counting convention (stated explicitly, round 18 item 1; the D4
//! design section and the numbered deviations above already implied this
//! but never named it as a counting rule the way `cspace-octomap`'s
//! `tree.rs` does).** One bullet per raw `public:` declaration in each of
//! `shapes.h`'s eight classes, with the following collapses, each
//! justified by [D4](#design-enum-not-a-trait-object-hierarchy-d4) rather
//! than by convenience: every concrete class's own `scaleAndPadd(double,
//! double)` override (the pure-virtual `scaleAndPadd`'s uniform 2-arg
//! signature) collapses into the one base-class bullet, since D4 makes
//! this literally one `match` statement, not eight separate functions;
//! same for every concrete `clone()` override (one `Clone` impl, not
//! eight); `STRING_NAME` across all seven concrete classes collapses into
//! one bullet (one `match` in [`ShapeType::as_str`]); the seven
//! concrete-class constructors collapse into one bullet (each is a direct
//! field-for-field `new`, no shared logic to separately explain); same for
//! the concrete classes' own data fields. Anything upstream expresses as a
//! per-subclass virtual override this port expresses as one `match` arm
//! set, so audited *once* against the base-class or free-function bullet
//! that names the whole match, not once per overriding subclass -- the
//! opposite of `tree.rs`'s convention, where each override gets its own
//! bullet because C++ virtual dispatch keeps them as distinct callable
//! declarations upstream. `using Base::method;` visibility declarations
//! and namespace-scope type aliases are not counted (no new symbol).
//!
//! **Reproducible raw counts (round 18, item 1).** Per-class raw
//! `public:` declaration counts, from
//! `tools/ci/count-public-declarations.sh <header>
//! <ClassName>` against a fresh oracle fetch (same script and comment/
//! brace-depth handling `cspace-octomap`'s `tree.rs` uses, copied into
//! this crate so a `cspace-geometry`-only audit doesn't need to reach into
//! a sibling crate's directory):
//!
//! ```text
//! $ sg docker -c "docker run --rm --entrypoint bash moveit-rs/oracle:e7d32225310d3278 -c 'cat /opt/ros/rolling/include/geometric_shapes/geometric_shapes/shapes.h'" > /tmp/shapes.h
//! $ for c in Shape Sphere Cylinder Cone Box Mesh Plane OcTree; do
//! >   echo "$c: $(tools/ci/count-public-declarations.sh /tmp/shapes.h "$c")"
//! > done
//! Shape: 9
//! Sphere: 7
//! Cylinder: 13
//! Cone: 13
//! Box: 12
//! Mesh: 21
//! Plane: 8
//! OcTree: 8
//! ```
//!
//! 91 raw declarations across the 8 classes, plus 4 at namespace scope
//! (`ShapeType`, `operator<<`, `ShapePtr`, `ShapeConstPtr`) = 95 total.
//! Every one of the 95 is accounted for by a bullet below except the one
//! genuine gap this round found and fixed -- `Mesh::~Mesh() override`, see
//! its own bullet -- confirmed by listing each class's raw declarations
//! (same script, no `<ClassName>` argument collapsing) and matching each
//! by name against this section's prose rather than trusting the bullet
//! count alone (bullets here bundle multiple raw declarations by design,
//! per the convention above, so a bullet-count-vs-raw-count arithmetic
//! check the way `tree.rs` uses would not by itself prove completeness).
//!
//! `shapes.h`:
//!
//! - `ShapeType` enum and its `operator<<` — **ported** as [`ShapeType`]/
//!   [`ShapeType::as_str`]/`Display`.
//! - `Shape` base class: constructor/destructor, the pure-virtual
//!   `clone`/`scaleAndPadd`, and the `ShapeType type` field — **subsumed by
//!   D4.** An enum has no separate construct/destruct step to port, `#[derive(Clone)]`
//!   is exact (every variant here is plain data — no raw pointers to
//!   deep-copy, unlike upstream's `Mesh`), `scaleAndPadd` is
//!   [`Shape::scale_and_padd`]'s per-variant match arms, and `type` is
//!   subsumed by [`Shape::shape_type`] — there is no separate field that
//!   could disagree with the enum discriminant, which is D4's whole point.
//! - `Shape::scale(double)`/`Shape::padd(double)` (the non-virtual uniform
//!   convenience wrappers) — **ported** as [`Shape::scale`]/[`Shape::padd`],
//!   both defined in terms of [`Shape::scale_and_padd`] exactly as upstream
//!   defines them in terms of the virtual `scaleAndPadd`.
//! - `Shape::isFixed()` (default `false`, overridden `true` in `Plane`/
//!   `OcTree`) — **ported** as [`Shape::is_fixed`].
//! - `Shape::print(ostream&)`, and every concrete class's override of it
//!   (`Sphere::print` through `OcTree::print`) — **unported.** Nothing in
//!   this crate writes to a debug stream; see [`OcTree`]'s own doc,
//!   "Deviations from upstream", for the one case — the null-`shared_ptr`
//!   state `OcTree::print` has to branch on — where this port's `Option`
//!   already makes the state explicit instead of needing a printed message.
//! - Each concrete class's `STRING_NAME` (`Sphere::STRING_NAME` through
//!   `OcTree::STRING_NAME`) — **ported** as [`ShapeType::as_str`]'s match
//!   arms, already probe-verified per value.
//! - `Cylinder`/`Cone`/`Box`/`Mesh`'s non-uniform overloads (2-, 3- or 6-arg
//!   `scale`/`padd`/`scaleAndPadd` taking a separate factor per axis) —
//!   **ported** as each type's `scale_axes`/`padd_axes`/`scale_and_padd_axes`.
//!   `Sphere`/`Plane`/`OcTree` have no non-uniform overloads upstream either
//!   (a sphere/plane/octree has no per-axis dimension to scale
//!   independently), so none exist here.
//! - `Mesh::computeTriangleNormals`/`computeVertexNormals`/`mergeVertices` —
//!   **ported** as [`Mesh::compute_triangle_normals`]/
//!   [`Mesh::compute_vertex_normals`]/[`Mesh::merge_vertices`].
//! - `Mesh(unsigned int v_count, unsigned int t_count)` (the
//!   allocate-then-fill-by-index constructor `constructShapeFromText` and
//!   `mesh_operations.cpp`'s loading pipeline use internally) — **unported.**
//!   [`Mesh::new`] takes fully-built `Vec`s instead (deviation 2 above), so
//!   nothing in this port ever needs to allocate an empty mesh and fill it
//!   by index; its only upstream caller besides internal pipeline plumbing
//!   already ported elsewhere ([`Mesh::new`] itself) is the deferred
//!   `constructShapeFromText` below.
//! - `ShapePtr`/`ShapeConstPtr` (`shared_ptr<Shape>`/`shared_ptr<const Shape>`
//!   typedefs) — **subsumed by Rust ownership.** A caller that needs shared
//!   ownership of a [`Shape`] uses `Arc<Shape>` directly (as [`OcTree`]'s own
//!   field does one level down, for the `octomap::OcTree` it wraps); there is no
//!   separate named type to port for a type alias whose only job upstream is
//!   working around the lack of a language-level ownership model.
//! - Each concrete class's own constructors (`Sphere()`/`Sphere(double)`,
//!   `Cylinder()`/`Cylinder(double, double)`, `Cone()`/`Cone(double, double)`,
//!   `Box()`/`Box(double, double, double)`, `Mesh()`, `Plane()`/`Plane(double,
//!   double, double, double)`, `OcTree()`/`OcTree(const shared_ptr<const
//!   octomap::OcTree>&)` — 13 declarations, `Mesh(unsigned int, unsigned
//!   int)` already covered above) and each concrete class's own data fields
//!   (`Sphere::radius`, `Cylinder`/`Cone`'s `length`/`radius`, `Box::size`,
//!   `Mesh`'s six count/pointer fields, `Plane`'s `a`/`b`/`c`/`d`, `OcTree`'s
//!   `octree`, already named in deviation 5 above) — **ported, direct
//!   constructor-for-constructor and field-for-field**, one Rust field or
//!   `new`/`try_new` parameter per upstream field or constructor parameter,
//!   per variant's own struct definition. **Addition, round 17 item 3:**
//!   absent from every walk through round 8 as its own accounted-for bullet
//!   — the general "direct field-for-field port" statement in the D4 design
//!   section above (in the `Cuboid` rename paragraph) was never restated
//!   here as covering the other six variants' constructors and fields too.
//! - The `using Shape::padd;`/`using Shape::scale;` using-declarations in
//!   `Cylinder`/`Cone`/`Box`/`Mesh` — not a declaration (no new symbol; a
//!   C++ visibility mechanism reintroducing the base overload set alongside
//!   each subclass's own overloads), skipped. Rust has no equivalent
//!   shadowing rule to work around: [`Shape::scale`]/[`Shape::padd`] and
//!   each variant's `scale_axes`/`padd_axes` are just differently-named
//!   methods, not overloads competing for the same name.
//! - `Mesh::~Mesh() override` — **subsumed by Rust ownership.** The only
//!   concrete class that declares its own destructor at all (every other
//!   variant relies on the base's trivial virtual `~Shape()`, already
//!   covered above); upstream's version frees the four raw
//!   `new double[]`/`new unsigned int[]` allocations deviation 2 replaces
//!   with `Vec`s, so there is no manual free left to port — `Mesh`'s
//!   `#[derive(Clone)]` field-by-field `Drop` already does exactly this
//!   work. **Addition, round 18 item 1:** absent from every walk through
//!   round 8/17 as its own bullet, found by the same raw-declaration-count
//!   reconciliation as the constructors/fields bullet above (`Mesh`'s own
//!   21 raw `public:` declarations minus the 20 already named elsewhere in
//!   this section left exactly this one unaccounted for).
//!
//! **Reproducible raw count, `shape_operations.h` (round 18, item 1).**
//! Free functions at namespace scope, not class members, so
//! `count_public_declarations.sh` (a `class`-body counter) does not apply;
//! verified instead with a signature-line grep, run against a fresh
//! oracle fetch:
//!
//! ```text
//! $ sg docker -c "docker run --rm --entrypoint bash moveit-rs/oracle:e7d32225310d3278 -c 'cat /opt/ros/rolling/include/geometric_shapes/geometric_shapes/shape_operations.h'" > /tmp/shape_operations.h
//! $ grep -c '^[A-Za-z].*(\|^  .*(' /tmp/shape_operations.h
//! 12
//! ```
//!
//! `shape_operations.h`, the 12 free functions:
//!
//! - `constructShapeFromMsg` (3 overloads: `SolidPrimitive`, `Plane`, `Mesh`,
//!   plus the `ShapeMsg` variant dispatcher — 4 declarations total),
//!   `constructMsgFromShape`, `constructMarkerFromShape`,
//!   `computeShapeExtents(const ShapeMsg&)` — **D-decision excludes (D1).**
//!   All seven take or produce `shape_msgs`/`visualization_msgs` ROS message
//!   types (4 + 1 + 1 + 1 = 7, not 6 — **correction, round 17 item 3**: the
//!   prior wording undercounted `constructShapeFromMsg`'s own 4 overloads by
//!   one against the other three named declarations); PORTING-PLAN.md D1
//!   keeps ROS-message (de)serialization and the rviz marker layer out of
//!   the core crates entirely (see the provenance comment at the top of
//!   this file).
//! - `computeShapeExtents(const Shape*)` — **ported as [`Shape::extents`].**
//! - `computeShapeBoundingSphere(const Shape*, center, radius)` — **ported as
//!   [`Shape::bounding_sphere`]**, an owned [`BoundingSphere`] return instead
//!   of two out-params.
//! - `shapeStringName(const Shape*)` — **ported, subsumed by
//!   [`Shape::shape_type`]`().`[`as_str`](ShapeType::as_str).** No dedicated
//!   `shape_string_name` free function exists; `shape.shape_type().as_str()`
//!   returns the identical `STRING_NAME` string with one fewer null-check
//!   (this port's `Shape` is never null — an enum value, not a pointer —
//!   which is the same "illegal state unrepresentable by construction"
//!   argument the module doc's D4 section makes for the type as a whole).
//! - `saveAsText(const Shape*, ostream&)`, `constructShapeFromText(istream&)`
//!   — **distinct, decided, not deferred (round 12).** Rounds 7-11 carried
//!   this as an open falsifier ("closes when a consumer names this exact
//!   format as the one it needs"), with `planning_scene.cpp`'s
//!   `PlanningScene::saveGeometryToStream`/`loadGeometryFromStream`
//!   (`:1062`/`:1152`) as the one candidate consumer — owned by
//!   `cspace-scene`, not this crate. Two deferrals each waiting on the
//!   other's silence never closes on its own; `cspace-scene` answered the
//!   real question this round instead of naming the format
//!   (`crates/cspace-scene/src/scene.rs`, commit `86f102c`): does this port
//!   intend `.scene` file interop at all? No. Every real upstream caller of
//!   `saveGeometryToStream`/`loadGeometryFromStream` — `move_group`'s
//!   `{load,save}_geometry_to_file_service_capability.cpp`, warehouse
//!   import/export, the RViz Scene tab, `publish_scene_from_text.cpp` — is
//!   `moveit_ros` tooling wrapped around a live ROS node; `moveit_py` is the
//!   out-of-scope rewrite one layer removed. `cspace-ros`, the one crate
//!   that could ever carry ROS-coupled tooling, does not exist and has no
//!   plan naming this workflow. That answer settles this crate's half too:
//!   `saveAsText`/`constructShapeFromText` has no reason to exist here
//!   beyond serving a caller that is now a positive out-of-scope decision,
//!   not an unmet falsifier. Not ported, and not reopened by a future
//!   `cspace-scene` round without a new, different consumer naming this
//!   exact text format for a reason unrelated to `.scene` file interop.
//!
//!   The algorithm remains recorded in case that ever happens: write/read
//!   `STRING_NAME` then each shape's raw numeric fields whitespace-separated
//!   (`Sphere`: radius; `Box`: 3 sizes; `Cylinder`/`Cone`: radius, length;
//!   `Plane`: a b c d; `Mesh`: vertex/triangle counts then each vertex and
//!   triangle, followed by `computeTriangleNormals()`/
//!   `computeVertexNormals()`) via the default `ostream`/`istream` `<<`/`>>`
//!   operators (no `setprecision` anywhere in the file — a round trip is
//!   lossy to the default ~6 significant digits). `OcTree` has no case in
//!   either function, matching this port's own D4 boundary — that shape was
//!   never round-trippable upstream either.
//!
//! # Who consumes `Shape::OcTree`, and what they will need from it
//!
//! Read from `moveit2`'s two call sites (`collision_common.cpp`,
//! `collision_env_distance_field.cpp`) so the eventual consumer does not have
//! to re-derive this; neither is implemented here — this crate is the shape
//! layer, not a collision backend, and `cspace-distance-field` owns the
//! second file this round.
//!
//! - **`collision_detection_fcl`'s conversion**
//!   (`collision_common.cpp::createCollisionGeometry`, the `shapes::OCTREE`
//!   case) is a one-line wrap: `new fcl::OcTreed(g->octree)`. FCL has a
//!   native octree collision geometry that operates directly on an
//!   arbitrary-resolution, arbitrary-depth `octomap::OcTree`, including
//!   pruned (coarse) leaves, at whatever depth `prune()` left them.
//!   `cspace-collision`'s `parry3d-f64` backend
//!   (`crates/cspace-collision/src/parry.rs`, `convert_shape`) has no
//!   FCL-`OcTreed`-shaped counterpart to convert into: `parry3d-f64` 0.30.0's
//!   closest shape, `parry3d_f64::shape::Voxels`, is a **uniform-resolution**
//!   sparse voxel grid (one fixed `voxel_size` for every filled cell) with no
//!   notion of a coarser leaf, and feeding it from a pruned [`OcTree`] would
//!   mean re-inflating every collapsed leaf back into up to `8^k`
//!   finest-resolution cells (`k` = depth deficit) — undoing the exact
//!   space saving `prune()` exists to provide. [`crate::geometry::compound_from_octree`]
//!   (this crate's `octree_collision` module) takes the other option instead:
//!   one `parry3d_f64::shape::Cuboid` per occupied leaf, sized to that leaf's
//!   own (possibly coarse) extent, collected into one
//!   `parry3d_f64::shape::Compound` — no uniform-resolution constraint, so a
//!   pruned coarse leaf costs one `Cuboid`, not `8^k` of them.
//!   `cspace-collision`'s `ParryCollisionEnv::convert_shape` calls it,
//!   memoized per-tree by an `OctreeCache` (see that module's doc) so the
//!   `Compound` is not rebuilt on every collision/distance query, and the
//!   wired path is oracle-verified against a real `CollisionEnvFCL` in
//!   `crates/cspace-collision/tests/octree_world_collision_parity.rs`, and
//!   `crates/cspace-collision/tests/octree_leaf_count_scaling_parity.rs`
//!   measures `robot_distance` agreement across leaf counts 0–216 rather than
//!   just asserting the structural difference (round 7 item 2; no divergence
//!   at any count tested).
//!
//!   **Not a live correctness gap — no consumer can observe it (round 11).**
//!   `parry3d_f64::shape::Voxels` is never actually used anywhere in this
//!   workspace: [`crate::geometry::compound_from_octree`] is the *only* `Shape::OcTree`
//!   conversion path `cspace-collision`'s `parry.rs` calls, and it has no
//!   uniform-size constraint (one [`Cuboid`](parry3d_f64::shape::Cuboid) per
//!   occupied leaf, sized to that leaf's own extent). So the gap this section
//!   describes is "`Voxels` cannot be *adopted* as an alternative
//!   representation", not "the shipped representation disagrees with
//!   upstream" — the latter is what
//!   `octree_leaf_count_scaling_parity.rs`/`octree_world_collision_parity.rs`
//!   (`cspace-collision`) already rule out at leaf counts 0–216, oracle-
//!   verified. `octree_shape_query`/`octree_in_world` (this crate's own
//!   oracle ops) are the capture path that would surface a divergence if one
//!   existed; nothing in three rounds of using them has.
//!
//!   **Falsifier (round 8, re-verified round 11 against the still-pinned
//!   `parry3d-f64 = "0.30.0"` in `Cargo.lock` — unchanged since round 8):**
//!   the item closes when `parry3d-f64` ships a shape that accepts *per-node*
//!   size, not one uniform `voxel_size` for the whole shape. Re-read the
//!   vendored source directly this round, not just `Voxels::new`: every
//!   `pub fn` in
//!   `~/.cargo/registry/src/index.crates.io-*/parry3d-f64-0.30.0/src/shape/voxels/voxels.rs`
//!   (18 methods) checked for a second constructor — `Voxels::new`
//!   (`:574`, `pub fn new(voxel_size: Vector, grid_coordinates: &[IVector])
//!   -> Self`) and `Voxels::from_points` (`:675`, same single `voxel_size`
//!   parameter, delegates to `new`) are the only two, and `voxel_size` is a
//!   single struct field (`:509`), not a per-cell array. Still cannot take a
//!   pruned, variable-depth [`crate::octomap::OcTree`] leaf directly, for the
//!   same re-inflation reason given above. `rg -n 'pub fn new' .../voxels.rs`
//!   returning a signature with a per-node size parameter (or a new
//!   `parry3d_f64::shape` module implementing a sparse/hierarchical volume)
//!   would close it; a version bump alone does not, unless that signature
//!   changed. It has not fired.
//! - **`collision_env_distance_field`'s treatment**
//!   (`collision_env_distance_field.cpp`, `~line 1753`) only ever reads: it
//!   builds `PosedBodyPointDecomposition(octree)` directly from the
//!   `shared_ptr<const octomap::OcTree>`, without a body/shape round trip at
//!   all. The interface it needs is exactly what [`crate::octomap::OcTree`]
//!   already exposes for this purpose: [`crate::octomap::OcTree::leaves`]
//!   filtered by [`crate::octomap::Leaf::is_occupied`], reading
//!   [`crate::octomap::Leaf::coordinate`] (and, if a decomposition wants to
//!   record cell size, [`crate::octomap::Leaf::size`]) per occupied leaf.
//!   Nothing further is needed on the `cspace-octomap` side for this
//!   consumer; the point-decomposition type itself belongs to
//!   `cspace-distance-field`.
//! - Neither consumer calls anything that inserts new data into a tree
//!   (`insertPointCloud`/`computeDiscreteUpdate`) — both only read an
//!   already-built `OcTree` handed to them through a `World::Object`'s
//!   shapes. The sensor updaters that populate a tree from a point cloud
//!   (`pointcloud_octomap_updater.cpp`,
//!   `occupancy_map_monitor.cpp`) are a separate, ROS-node-shaped ingestion
//!   subsystem the round-1 Step 1 survey already found and deferred; this
//!   analysis does not change that — `insertPointCloud` stays deferred.
//! - What both consumers *do* need, in common, is the composition of a
//!   `World::Object`'s pose with a shape's own sub-pose, then mapping a
//!   world-frame query back into the octree's local frame before a query
//!   means anything — `collision_common.cpp`'s `fcl::OcTreed` and
//!   `collision_env_distance_field.cpp`'s `PosedBodyPointDecomposition`
//!   each perform that step through their own backend-specific mechanism,
//!   but neither special-cases the *pose* half of it. That half is verified
//!   directly against the real oracle (`octree_in_world` op, exercising
//!   `collision_detection::World::addToObject`/`getGlobalShapeTransforms`
//!   with a `shapes::OcTree`) in
//!   `crates/cspace-core/tests/octree_in_world_parity.rs`, using only
//!   this crate's own [`OcTree`]/[`crate::geometry::Isometry3`] and
//!   [`crate::octomap::OcTree`]'s existing point-query API — nothing further
//!   is needed from this crate for either backend to build on.
//!
//! ## Transfer boundary, symbol by symbol (round 15, item 2)
//!
//! `cspace-octomap`'s own module docs used to describe `Shape::OcTree` as
//! "already stubbed, deliberately deferred to Phase 3/5 collision" — stale,
//! fixed this round (see that crate's `lib.rs`). Re-checked against the tree
//! as it stands after the round-15 rebase, not assumed:
//!
//! - **`Shape::OcTree` (this crate's data layer) → `cspace-collision`.**
//!   Already receiving, nothing further owed: `crates/cspace-collision/src/
//!   parry.rs`'s `ParryCollisionEnv::convert_shape` calls
//!   [`crate::geometry::compound_from_octree`] directly (see "Who consumes
//!   `Shape::OcTree`" above for the full path and its oracle coverage).
//!   `Cargo.toml` already carries `cspace-octomap.workspace = true` for
//!   this.
//! - **`crate::octomap::OcTree`'s raw leaf payload → `cspace-distance-field`,
//!   *not yet* receiving.** `collision_env_distance_field.cpp`'s
//!   `PosedBodyPointDecomposition(shared_ptr<const octomap::OcTree>)`
//!   constructor is still unported there (`crates/cspace-collision/src/distance_field/
//!   lib.rs`'s own module docs list it, under `PosedBodyPointDecomposition`,
//!   as the one of three constructor overloads not yet done); confirmed by
//!   `Cargo.toml` too — `cspace-distance-field` names no `cspace-octomap`
//!   dependency at all, unlike `cspace-collision`'s. What it needs to
//!   receive this: add that workspace dependency, then implement the
//!   constructor over [`crate::octomap::OcTree::leaves`] filtered by
//!   [`crate::octomap::Leaf::is_occupied`] (see "Who consumes
//!   `Shape::OcTree`" above for the exact field mapping). Nothing on this
//!   crate's or `cspace-octomap`'s side blocks that — the API this
//!   constructor needs already exists and is public.
//! - **`bodies::` posed-body algorithms (`containsPoint`/`intersectsRay`/
//!   the bounding-volume methods) → stay in `cspace-geometry`, not
//!   transferred anywhere.** The original task brief for this crate assumed
//!   these belonged with Phase 3 collision; they do not, and re-checking
//!   this round confirms that is still true: `cspace-collision`'s `lib.rs`/
//!   `world.rs` still explicitly declines the `bodies::` posed-geometry
//!   layer as out of scope for `World`, and its `ParryCollisionEnv` backend
//!   still builds directly on `parry3d-f64` shapes from `Shape`, never
//!   from [`crate::geometry::bodies::Body`]. The real consumers remain
//!   `cspace-constraints` and `cspace-distance-field`, already receiving —
//!   see `bodies.rs`'s own "Who actually calls this" section for the
//!   method-by-method breakdown, which this confirmation does not
//!   duplicate.
//! - **[`crate::geometry::bodies::Body::intersects_ray`] → stays in `cspace-geometry`,
//!   still no consumer.** `bodies.rs` already documents this (round 13-14)
//!   with its reopening condition — `cspace-collision`'s `ParryCollisionEnv`/
//!   `PosedBody` path needing a body-level ray test, or `BodyVector` getting
//!   a real caller. Re-checked this round: neither has happened
//!   (`ParryCollisionEnv` still does not reference `bodies::Body` at all,
//!   confirmed above). No change from round 14's decision.

use std::fmt;
use std::sync::Arc;

use crate::error::{Error, Result};

use crate::geometry::Vector3;

/// A list of known shape types. Upstream `shapes::ShapeType`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ShapeType {
    /// `shapes::UNKNOWN_SHAPE`. Never returned by [`Shape::shape_type`] —
    /// every [`Shape`] variant is a concrete, known type — but kept for
    /// fidelity with upstream's enum, which uses this value as
    /// `bodies::Body`'s pre-`setDimensions` sentinel.
    Unknown,
    /// `shapes::SPHERE`.
    Sphere,
    /// `shapes::CYLINDER`.
    Cylinder,
    /// `shapes::CONE`.
    Cone,
    /// `shapes::BOX`.
    Box,
    /// `shapes::PLANE`.
    Plane,
    /// `shapes::MESH`.
    Mesh,
    /// `shapes::OCTREE`.
    OcTree,
}

impl ShapeType {
    /// The upstream `STRING_NAME` for this type (`operator<<(ostream&, ShapeType)`
    /// in `shapes.cpp`).
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Unknown => "unknown",
            Self::Sphere => "sphere",
            Self::Cylinder => "cylinder",
            Self::Cone => "cone",
            Self::Box => "box",
            Self::Plane => "plane",
            Self::Mesh => "mesh",
            Self::OcTree => "octree",
        }
    }
}

impl fmt::Display for ShapeType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// A sphere bounding a shape, centered at the shape's own origin.
///
/// Upstream `bodies::BoundingSphere`, restricted to the unposed case
/// `shapes::computeShapeBoundingSphere` produces; reused as-is by
/// [`crate::geometry::bodies`] for the posed case (`bodies::Body::computeBoundingSphere`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BoundingSphere {
    /// The sphere's center, relative to the shape's own origin.
    pub center: Vector3,
    /// The sphere's radius.
    pub radius: f64,
}

/// A sphere, given by its radius. Upstream `shapes::Sphere`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Sphere {
    /// The radius of the sphere.
    pub radius: f64,
}

impl Sphere {
    /// Build a sphere with the given radius.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when `radius` is negative or non-finite — see
    /// [`Cone::bounding_sphere`]'s doc for why a bare `< 0.0` check alone
    /// (upstream's own form, `if (r < 0)`) is not enough: `NaN < 0.0` is
    /// `false`, so it silently accepts a `NaN` radius as "non-negative".
    pub fn new(radius: f64) -> Result<Self> {
        if !radius.is_finite() || radius < 0.0 {
            return Err(Error::construct("Sphere radius must be non-negative."));
        }
        Ok(Self { radius })
    }

    /// Uniformly scale and pad this sphere. Upstream `Sphere::scaleAndPadd`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when the resulting radius would be negative or
    /// non-finite (`scale`/`padding` themselves are not validated upstream
    /// either — see [`Sphere::new`]).
    pub fn scale_and_padd(&mut self, scale: f64, padding: f64) -> Result<()> {
        let radius = self.radius * scale + padding;
        if !radius.is_finite() || radius < 0.0 {
            return Err(Error::construct("Sphere radius must be non-negative."));
        }
        self.radius = radius;
        Ok(())
    }

    /// Uniformly scale this sphere. Upstream `Shape::scale` as inherited by
    /// `Sphere`.
    pub fn scale(&mut self, scale: f64) -> Result<()> {
        self.scale_and_padd(scale, 0.0)
    }

    /// Add uniform padding to this sphere. Upstream `Shape::padd` as
    /// inherited by `Sphere`.
    pub fn padd(&mut self, padding: f64) -> Result<()> {
        self.scale_and_padd(1.0, padding)
    }

    /// The volume of this sphere. Upstream `bodies::Sphere::computeVolume`
    /// (dimension-only part — see the module docs).
    pub fn compute_volume(&self) -> f64 {
        4.0 / 3.0 * std::f64::consts::PI * self.radius * self.radius * self.radius
    }

    /// The axis-aligned extents of this sphere's bounding box. Upstream
    /// `computeShapeExtents`'s `SPHERE` branch.
    pub fn extents(&self) -> Vector3 {
        let d = self.radius * 2.0;
        Vector3::new(d, d, d)
    }

    /// This sphere's own bounding sphere (itself). Upstream
    /// `computeShapeBoundingSphere`'s `SPHERE` branch.
    pub fn bounding_sphere(&self) -> BoundingSphere {
        BoundingSphere {
            center: Vector3::zeros(),
            radius: self.radius,
        }
    }
}

/// A cylinder, given by its radius and length. The length runs along the
/// z-axis; the origin is the center of mass. Upstream `shapes::Cylinder`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Cylinder {
    /// The radius of the cylinder.
    pub radius: f64,
    /// The length of the cylinder, along z.
    pub length: f64,
}

impl Cylinder {
    /// Build a cylinder with the given radius and length.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when either dimension is negative or non-finite
    /// — see [`Cone::bounding_sphere`]'s doc: `NaN < 0.0` is `false`.
    pub fn new(radius: f64, length: f64) -> Result<Self> {
        if !radius.is_finite() || !length.is_finite() || radius < 0.0 || length < 0.0 {
            return Err(Error::construct(
                "Cylinder dimensions must be non-negative.",
            ));
        }
        Ok(Self { radius, length })
    }

    /// Scale and pad this cylinder's radius and length independently.
    /// Upstream `Cylinder::scaleAndPadd(scaleRadius, scaleLength, paddRadius,
    /// paddLength)`.
    ///
    /// Padding is applied to the length on both ends, so `padd_length` is
    /// doubled — matching upstream exactly.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when the resulting radius or length would be
    /// negative or non-finite.
    pub fn scale_and_padd_axes(
        &mut self,
        scale_radius: f64,
        scale_length: f64,
        padd_radius: f64,
        padd_length: f64,
    ) -> Result<()> {
        let radius = self.radius * scale_radius + padd_radius;
        let length = self.length * scale_length + 2.0 * padd_length;
        if !radius.is_finite() || !length.is_finite() || radius < 0.0 || length < 0.0 {
            return Err(Error::construct(
                "Cylinder dimensions must be non-negative.",
            ));
        }
        self.radius = radius;
        self.length = length;
        Ok(())
    }

    /// Uniformly scale and pad this cylinder. Upstream
    /// `Cylinder::scaleAndPadd(scale, padd)` (the `Shape` override).
    pub fn scale_and_padd(&mut self, scale: f64, padding: f64) -> Result<()> {
        self.scale_and_padd_axes(scale, scale, padding, padding)
    }

    /// Scale this cylinder's radius and length independently. Upstream
    /// `Cylinder::scale(scaleRadius, scaleLength)`.
    pub fn scale_axes(&mut self, scale_radius: f64, scale_length: f64) -> Result<()> {
        self.scale_and_padd_axes(scale_radius, scale_length, 0.0, 0.0)
    }

    /// Pad this cylinder's radius and length independently. Upstream
    /// `Cylinder::padd(paddRadius, paddLength)`.
    pub fn padd_axes(&mut self, padd_radius: f64, padd_length: f64) -> Result<()> {
        self.scale_and_padd_axes(1.0, 1.0, padd_radius, padd_length)
    }

    /// Uniformly scale this cylinder. Upstream `Shape::scale` as inherited
    /// (via `using Shape::scale;`) by `Cylinder`.
    pub fn scale(&mut self, scale: f64) -> Result<()> {
        self.scale_and_padd(scale, 0.0)
    }

    /// Add uniform padding to this cylinder. Upstream `Shape::padd` as
    /// inherited (via `using Shape::padd;`) by `Cylinder`.
    pub fn padd(&mut self, padding: f64) -> Result<()> {
        self.scale_and_padd(1.0, padding)
    }

    /// The volume of this cylinder. Upstream
    /// `bodies::Cylinder::computeVolume` (dimension-only part).
    pub fn compute_volume(&self) -> f64 {
        std::f64::consts::PI * self.radius * self.radius * self.length
    }

    /// The axis-aligned extents of this cylinder's bounding box. Upstream
    /// `computeShapeExtents`'s `CYLINDER` branch.
    pub fn extents(&self) -> Vector3 {
        let d = self.radius * 2.0;
        Vector3::new(d, d, self.length)
    }

    /// This cylinder's bounding sphere. Upstream `computeShapeBoundingSphere`'s
    /// `CYLINDER` branch.
    pub fn bounding_sphere(&self) -> BoundingSphere {
        let half_len = self.length * 0.5;
        BoundingSphere {
            center: Vector3::zeros(),
            radius: (self.radius * self.radius + half_len * half_len).sqrt(),
        }
    }
}

/// A cone. The tip is on the positive z-axis, the center of the base on the
/// negative z-axis; the origin is halfway between. Upstream `shapes::Cone`.
///
/// # Deviation from upstream
///
/// Unlike [`Sphere`], [`Cylinder`] and [`Cuboid`], `Cone` has no
/// `compute_volume`/`get_dimensions` — see the module docs: upstream has no
/// `bodies::Cone` to port either from.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Cone {
    /// The radius of the cone's base.
    pub radius: f64,
    /// The length (height) of the cone.
    pub length: f64,
}

impl Cone {
    /// Build a cone with the given radius and length.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when either dimension is negative or non-finite
    /// — see [`Cone::bounding_sphere`]'s doc: `NaN < 0.0` is `false`, so a
    /// bare `< 0.0` check (upstream's own form, `if (r < 0 || l < 0)`)
    /// silently accepts a `NaN` dimension as "non-negative".
    pub fn new(radius: f64, length: f64) -> Result<Self> {
        if !radius.is_finite() || !length.is_finite() || radius < 0.0 || length < 0.0 {
            return Err(Error::construct("Cone dimensions must be non-negative."));
        }
        Ok(Self { radius, length })
    }

    /// Scale and pad this cone's radius and length independently. Upstream
    /// `Cone::scaleAndPadd(scaleRadius, scaleLength, paddRadius, paddLength)`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when the resulting radius or length would be
    /// negative or non-finite.
    pub fn scale_and_padd_axes(
        &mut self,
        scale_radius: f64,
        scale_length: f64,
        padd_radius: f64,
        padd_length: f64,
    ) -> Result<()> {
        let radius = self.radius * scale_radius + padd_radius;
        let length = self.length * scale_length + 2.0 * padd_length;
        if !radius.is_finite() || !length.is_finite() || radius < 0.0 || length < 0.0 {
            return Err(Error::construct("Cone dimensions must be non-negative."));
        }
        self.radius = radius;
        self.length = length;
        Ok(())
    }

    /// Uniformly scale and pad this cone. Upstream `Cone::scaleAndPadd(scale,
    /// padd)` (the `Shape` override).
    pub fn scale_and_padd(&mut self, scale: f64, padding: f64) -> Result<()> {
        self.scale_and_padd_axes(scale, scale, padding, padding)
    }

    /// Scale this cone's radius and length independently. Upstream
    /// `Cone::scale(scaleRadius, scaleLength)`.
    pub fn scale_axes(&mut self, scale_radius: f64, scale_length: f64) -> Result<()> {
        self.scale_and_padd_axes(scale_radius, scale_length, 0.0, 0.0)
    }

    /// Pad this cone's radius and length independently. Upstream
    /// `Cone::padd(paddRadius, paddLength)`.
    pub fn padd_axes(&mut self, padd_radius: f64, padd_length: f64) -> Result<()> {
        self.scale_and_padd_axes(1.0, 1.0, padd_radius, padd_length)
    }

    /// Uniformly scale this cone. Upstream `Shape::scale` as inherited (via
    /// `using Shape::scale;`) by `Cone`.
    pub fn scale(&mut self, scale: f64) -> Result<()> {
        self.scale_and_padd(scale, 0.0)
    }

    /// Add uniform padding to this cone. Upstream `Shape::padd` as inherited
    /// (via `using Shape::padd;`) by `Cone`.
    pub fn padd(&mut self, padding: f64) -> Result<()> {
        self.scale_and_padd(1.0, padding)
    }

    /// The axis-aligned extents of this cone's bounding box. Upstream
    /// `computeShapeExtents`'s `CONE` branch.
    pub fn extents(&self) -> Vector3 {
        let d = self.radius * 2.0;
        Vector3::new(d, d, self.length)
    }

    /// This cone's bounding sphere. Upstream `computeShapeBoundingSphere`'s
    /// `CONE` branch: a tall cone's bounding sphere touches the base rim and
    /// the tip; a short, wide cone's bounding sphere touches the base rim
    /// only and is centered on the base.
    ///
    /// # Deviation: a non-finite or non-positive `length` cannot reach the
    /// division
    ///
    /// Upstream (`geometric_shapes/src/shape_operations.cpp`, `CONE` branch
    /// of `computeShapeBoundingSphere`) has the identical `cone_height >
    /// cone_radius` branch and `cone_radius * cone_radius / cone_height`
    /// division, unguarded the same way. For a `Cone` built through
    /// [`Cone::new`]/[`Cone::scale_and_padd_axes`] alone this can't divide
    /// by zero: `length > radius >= 0` already forces `length > 0`. But
    /// `radius`/`length` are public fields — this crate has no way to stop
    /// a caller writing `self.length = 0.0` directly, and even with
    /// [`Cone::new`] fixed to reject `NaN`, a `Cone` can still carry one via
    /// direct field assignment. `self.length > self.radius` reads `false`
    /// for any `NaN` operand (every `NaN` comparison is), which does not
    /// route around the problem: the `else` branch below assigns
    /// `self.radius`/`self.length` straight through too. So this method
    /// checks both fields itself, up front, the same way [`Shape::
    /// bounding_sphere`] already falls back to a zero-radius sphere for
    /// [`Shape::Plane`]/[`Shape::OcTree`] when it has nothing real to
    /// report — extending that existing convention rather than adding a
    /// special case. The `length > 0.0` clause on the `if` (redundant for
    /// any `Cone` [`Cone::new`] would accept, since that already forces
    /// `length > radius >= 0`) is what actually keeps the division's
    /// denominator off zero if `radius` itself is negative by direct field
    /// assignment.
    pub fn bounding_sphere(&self) -> BoundingSphere {
        if !self.radius.is_finite() || !self.length.is_finite() {
            return BoundingSphere {
                center: Vector3::zeros(),
                radius: 0.0,
            };
        }
        if self.length > self.radius && self.length > 0.0 {
            let z = (self.length - (self.radius * self.radius / self.length)) * 0.5;
            BoundingSphere {
                center: Vector3::new(0.0, 0.0, z - self.length * 0.5),
                radius: self.length - z,
            }
        } else {
            BoundingSphere {
                center: Vector3::new(0.0, 0.0, -(self.length * 0.5)),
                radius: self.radius,
            }
        }
    }
}

/// An axis-aligned box, given by its extents along x, y and z. Upstream
/// `shapes::Box`, renamed to avoid shadowing [`std::boxed::Box`] — see the
/// module docs, deviation 1.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Cuboid {
    /// Extents along x, y, z. Upstream `bodies::Box::getDimensions` labels
    /// these length, width, height respectively.
    pub size: [f64; 3],
}

impl Cuboid {
    /// Build a cuboid with the given x, y, z extents.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when any dimension is negative or non-finite —
    /// see [`Cone::bounding_sphere`]'s doc: `NaN < 0.0` is `false`.
    pub fn new(x: f64, y: f64, z: f64) -> Result<Self> {
        if !x.is_finite() || !y.is_finite() || !z.is_finite() || x < 0.0 || y < 0.0 || z < 0.0 {
            return Err(Error::construct("Box dimensions must be non-negative."));
        }
        Ok(Self { size: [x, y, z] })
    }

    /// Scale and pad this cuboid's x, y, z extents independently. Upstream
    /// `Box::scaleAndPadd(scaleX, scaleY, scaleZ, paddX, paddY, paddZ)`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when a resulting dimension would be negative or
    /// non-finite.
    pub fn scale_and_padd_axes(
        &mut self,
        scale_x: f64,
        scale_y: f64,
        scale_z: f64,
        padd_x: f64,
        padd_y: f64,
        padd_z: f64,
    ) -> Result<()> {
        let x = self.size[0] * scale_x + padd_x * 2.0;
        let y = self.size[1] * scale_y + padd_y * 2.0;
        let z = self.size[2] * scale_z + padd_z * 2.0;
        if !x.is_finite() || !y.is_finite() || !z.is_finite() || x < 0.0 || y < 0.0 || z < 0.0 {
            return Err(Error::construct("Box dimensions must be non-negative."));
        }
        self.size = [x, y, z];
        Ok(())
    }

    /// Uniformly scale and pad this cuboid. Upstream `Box::scaleAndPadd(scale,
    /// padd)` (the `Shape` override).
    pub fn scale_and_padd(&mut self, scale: f64, padding: f64) -> Result<()> {
        self.scale_and_padd_axes(scale, scale, scale, padding, padding, padding)
    }

    /// Scale this cuboid's x, y, z extents independently. Upstream
    /// `Box::scale(scaleX, scaleY, scaleZ)`.
    pub fn scale_axes(&mut self, scale_x: f64, scale_y: f64, scale_z: f64) -> Result<()> {
        self.scale_and_padd_axes(scale_x, scale_y, scale_z, 0.0, 0.0, 0.0)
    }

    /// Pad this cuboid's x, y, z extents independently. Upstream
    /// `Box::padd(paddX, paddY, paddZ)`.
    pub fn padd_axes(&mut self, padd_x: f64, padd_y: f64, padd_z: f64) -> Result<()> {
        self.scale_and_padd_axes(1.0, 1.0, 1.0, padd_x, padd_y, padd_z)
    }

    /// Uniformly scale this cuboid. Upstream `Shape::scale` as inherited
    /// (via `using Shape::scale;`) by `Box`.
    pub fn scale(&mut self, scale: f64) -> Result<()> {
        self.scale_and_padd(scale, 0.0)
    }

    /// Add uniform padding to this cuboid. Upstream `Shape::padd` as
    /// inherited (via `using Shape::padd;`) by `Box`.
    pub fn padd(&mut self, padding: f64) -> Result<()> {
        self.scale_and_padd(1.0, padding)
    }

    /// The volume of this cuboid. Upstream `bodies::Box::computeVolume`
    /// (dimension-only part).
    pub fn compute_volume(&self) -> f64 {
        self.size[0] * self.size[1] * self.size[2]
    }

    /// The axis-aligned extents of this cuboid — itself. Upstream
    /// `computeShapeExtents`'s `BOX` branch.
    pub fn extents(&self) -> Vector3 {
        Vector3::new(self.size[0], self.size[1], self.size[2])
    }

    /// This cuboid's bounding sphere. Upstream `computeShapeBoundingSphere`'s
    /// `BOX` branch.
    pub fn bounding_sphere(&self) -> BoundingSphere {
        let half = Vector3::new(self.size[0], self.size[1], self.size[2]) * 0.5;
        BoundingSphere {
            center: Vector3::zeros(),
            radius: half.norm(),
        }
    }
}

/// A plane, with equation `ax + by + cz + d = 0`. Upstream `shapes::Plane`.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Plane {
    /// The `a` coefficient.
    pub a: f64,
    /// The `b` coefficient.
    pub b: f64,
    /// The `c` coefficient.
    pub c: f64,
    /// The `d` coefficient.
    pub d: f64,
}

impl Plane {
    /// Build a plane from its equation coefficients. Upstream's constructor
    /// does not validate — an infinite plane has no notion of a negative
    /// dimension — so this does not either.
    pub const fn new(a: f64, b: f64, c: f64, d: f64) -> Self {
        Self { a, b, c, d }
    }

    /// A plane cannot be scaled or padded. Upstream `Plane::scaleAndPadd`
    /// logs a warning (`CONSOLE_BRIDGE_logWarn("Planes cannot be scaled or
    /// padded")`) and otherwise does nothing; this port has no logging
    /// framework wired up yet, so it is simply a no-op.
    pub const fn scale_and_padd(&mut self, _scale: f64, _padding: f64) {}

    /// A no-op, like [`Plane::scale_and_padd`]. Upstream `Shape::scale` as
    /// inherited (via `using Shape::scale;`) by `Plane`.
    pub const fn scale(&mut self, _scale: f64) {}

    /// A no-op, like [`Plane::scale_and_padd`]. Upstream `Shape::padd` as
    /// inherited (via `using Shape::padd;`) by `Plane`.
    pub const fn padd(&mut self, _padding: f64) {}
}

/// Representation of an octree as a shape. Upstream `shapes::OcTree`.
///
/// # Deviations from upstream
///
/// - **`octree` is `Option<Arc<crate::octomap::OcTree>>`, not a raw
///   `shared_ptr`.** See the module docs, deviation 5, for why `Arc`; `Option`
///   reproduces upstream's default-constructed (`OcTree::OcTree()`) null
///   `shared_ptr` state, which upstream's own `OcTree::print()`
///   (`if (octree) ... else "OcTree[NULL]"`) explicitly handles rather than
///   assumes away. `print()` itself is not ported — nothing in this crate
///   logs — so the null case only shows up here as `octree: None`.
/// - **[`PartialEq`] compares by [`Arc::ptr_eq`], not tree contents.**
///   `std::shared_ptr`'s own `operator==` compares the two pointers, not the
///   pointees, so two `shapes::OcTree`s wrapping separately-built but
///   identical trees are upstream-`!=`; a per-node structural comparison here
///   would be both more expensive than upstream ever pays and answer a
///   different question than upstream's `==` does. Two `None` trees (both
///   the default-constructed null state) are equal.
#[derive(Debug, Clone, Default)]
pub struct OcTree {
    /// The wrapped tree, or `None` for upstream's default-constructed
    /// (null-`shared_ptr`) state. Upstream `shapes::OcTree::octree`.
    pub octree: Option<Arc<crate::octomap::OcTree>>,
}

impl OcTree {
    /// Upstream `OcTree::OcTree()`: no tree yet.
    pub const fn new() -> Self {
        Self { octree: None }
    }

    /// Upstream `OcTree::OcTree(const std::shared_ptr<const octomap::OcTree>&)`.
    pub const fn from_tree(tree: Arc<crate::octomap::OcTree>) -> Self {
        Self { octree: Some(tree) }
    }

    /// An octree cannot be scaled or padded. Upstream `OcTree::scaleAndPadd`
    /// logs a warning (`CONSOLE_BRIDGE_logWarn("OcTrees cannot be scaled or
    /// padded")`) and otherwise does nothing; see [`Plane::scale_and_padd`]
    /// for why this port drops the log.
    pub const fn scale_and_padd(&mut self, _scale: f64, _padding: f64) {}

    /// A no-op, like [`OcTree::scale_and_padd`]. Upstream `Shape::scale` as
    /// inherited (via `using Shape::scale;`) by `OcTree`.
    pub const fn scale(&mut self, _scale: f64) {}

    /// A no-op, like [`OcTree::scale_and_padd`]. Upstream `Shape::padd` as
    /// inherited (via `using Shape::padd;`) by `OcTree`.
    pub const fn padd(&mut self, _padding: f64) {}
}

impl PartialEq for OcTree {
    /// See the type doc's deviation on [`PartialEq`]: identity, not tree
    /// content, matching upstream `shared_ptr::operator==`.
    fn eq(&self, other: &Self) -> bool {
        match (&self.octree, &other.octree) {
            (None, None) => true,
            (Some(a), Some(b)) => Arc::ptr_eq(a, b),
            _ => false,
        }
    }
}

/// A triangle mesh. By convention the mesh's own AABB is centered at the
/// origin; methods that assume this (padding, extents, bounding sphere) may
/// not behave sensibly otherwise — this matches upstream. Upstream
/// `shapes::Mesh`.
///
/// # Deviations from upstream
///
/// See the module docs, deviations 2–5: `vertices`/`triangles` are owned
/// `Vec`s instead of `count` fields paired with raw pointers,
/// [`Mesh::new`] rejects out-of-range triangle indices, padding requires
/// vertex normals to already be computed, and degenerate triangles produce a
/// zero normal instead of `NaN`.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Mesh {
    /// The mesh's vertices.
    pub vertices: Vec<Vector3>,
    /// Each triangle's three vertex indices, into [`Mesh::vertices`].
    pub triangles: Vec<[u32; 3]>,
    /// The unit normal of each triangle, parallel to [`Mesh::triangles`].
    /// `None` until [`Mesh::compute_triangle_normals`] runs — upstream
    /// leaves the equivalent array allocated but uninitialized until then.
    pub triangle_normals: Option<Vec<Vector3>>,
    /// The unit normal at each vertex, parallel to [`Mesh::vertices`]. `None`
    /// until [`Mesh::compute_vertex_normals`] runs.
    pub vertex_normals: Option<Vec<Vector3>>,
}

impl Mesh {
    /// Build a mesh from its vertices and triangles (as vertex-index
    /// triples). Normals start unset; call [`Mesh::compute_triangle_normals`]
    /// and/or [`Mesh::compute_vertex_normals`] to populate them.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when a triangle references a vertex index that
    /// is out of range for `vertices`. Upstream never checks this — an
    /// out-of-range index is a read past the end of the `vertices`
    /// allocation the first time any per-triangle method runs. See the
    /// module docs, deviation 3.
    pub fn new(vertices: Vec<Vector3>, triangles: Vec<[u32; 3]>) -> Result<Self> {
        let n = vertices.len();
        for tri in &triangles {
            for &idx in tri {
                if idx as usize >= n {
                    return Err(Error::construct(format!(
                        "mesh triangle references vertex {idx}, but only {n} vertices exist"
                    )));
                }
            }
        }
        Ok(Self {
            vertices,
            triangles,
            triangle_normals: None,
            vertex_normals: None,
        })
    }

    /// Scale and pad this mesh's x, y, z extents independently. Upstream
    /// `Mesh::scaleAndPadd(scaleX, scaleY, scaleZ, paddX, paddY, paddZ)`:
    /// finds the vertex centroid, scales each vertex toward/away from it,
    /// then pads along that vertex's own normal direction (not the
    /// coordinate axes) by `padd_x`/`padd_y`/`padd_z` per component.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when [`Mesh::vertex_normals`] is `None`. Upstream
    /// reads the vertex normal array unconditionally, for every call
    /// regardless of the padding values — see the module docs, deviation 4.
    /// A mesh with no vertices is a no-op, matching upstream (the loop
    /// that would read `vertex_normals` never runs).
    pub fn scale_and_padd_axes(
        &mut self,
        scale_x: f64,
        scale_y: f64,
        scale_z: f64,
        padd_x: f64,
        padd_y: f64,
        padd_z: f64,
    ) -> Result<()> {
        if self.vertices.is_empty() {
            return Ok(());
        }
        let vertex_normals = self.vertex_normals.as_ref().ok_or_else(|| {
            Error::construct(
                "mesh padding requires vertex normals; call compute_vertex_normals() first",
            )
        })?;

        let n = self.vertices.len() as f64;
        let mut centroid = Vector3::zeros();
        for v in &self.vertices {
            centroid += v;
        }
        centroid /= n;

        for (v, vn) in self.vertices.iter_mut().zip(vertex_normals.iter()) {
            let d = *v - centroid;
            let scaled = centroid + Vector3::new(d.x * scale_x, d.y * scale_y, d.z * scale_z);
            *v = scaled + Vector3::new(vn.x * padd_x, vn.y * padd_y, vn.z * padd_z);
        }
        Ok(())
    }

    /// Uniformly scale and pad this mesh. Upstream `Mesh::scaleAndPadd(scale,
    /// padd)` (the `Shape` override).
    pub fn scale_and_padd(&mut self, scale: f64, padding: f64) -> Result<()> {
        self.scale_and_padd_axes(scale, scale, scale, padding, padding, padding)
    }

    /// Scale this mesh's x, y, z extents independently, about the vertex
    /// centroid. Upstream `Mesh::scale(scaleX, scaleY, scaleZ)`.
    pub fn scale_axes(&mut self, scale_x: f64, scale_y: f64, scale_z: f64) -> Result<()> {
        self.scale_and_padd_axes(scale_x, scale_y, scale_z, 0.0, 0.0, 0.0)
    }

    /// Pad this mesh's x, y, z extents independently, along each vertex's
    /// own normal. Upstream `Mesh::padd(paddX, paddY, paddZ)`.
    pub fn padd_axes(&mut self, padd_x: f64, padd_y: f64, padd_z: f64) -> Result<()> {
        self.scale_and_padd_axes(1.0, 1.0, 1.0, padd_x, padd_y, padd_z)
    }

    /// Uniformly scale this mesh, about the vertex centroid. Upstream
    /// `Shape::scale` as inherited (via `using Shape::scale;`) by `Mesh`.
    pub fn scale(&mut self, scale: f64) -> Result<()> {
        self.scale_and_padd(scale, 0.0)
    }

    /// Add uniform padding to this mesh, along each vertex's own normal.
    /// Upstream `Shape::padd` as inherited (via `using Shape::padd;`) by
    /// `Mesh`.
    pub fn padd(&mut self, padding: f64) -> Result<()> {
        self.scale_and_padd(1.0, padding)
    }

    /// Compute each triangle's unit normal from its three vertices, via
    /// cross product. Upstream `Mesh::computeTriangleNormals` (`shapes.cpp:
    /// 488-509`): `Eigen::Vector3d normal = s1.cross(s2); normal.normalize();`
    /// — no explicit guard at the call site.
    ///
    /// A degenerate triangle (zero-length cross product — two coincident or
    /// colinear vertices) gets the zero vector here, matching upstream: this
    /// is not a deviation. Measured against real Eigen 3.4.0
    /// (`moveit-rs/oracle:ccc22ff0287a603f`): an unguarded in-place
    /// `.normalize()` on a zero vector stays `[0, 0, 0]` — `normalize()`
    /// carries the same internal zero-norm guard as its `normalized()`
    /// counterpart. `try_normalize(0.0)`'s `None` branch (substituting the
    /// zero vector) is exactly that guard, not a divergence from it.
    ///
    /// # Panics
    ///
    /// If [`Mesh::triangles`] was mutated after construction to reference an
    /// out-of-range vertex index. [`Mesh::new`] cannot enforce this after the
    /// fact because [`Mesh::triangles`] is a public field.
    pub fn compute_triangle_normals(&mut self) {
        let mut normals = Vec::with_capacity(self.triangles.len());
        for tri in &self.triangles {
            let p0 = self.vertices[tri[0] as usize];
            let p1 = self.vertices[tri[1] as usize];
            let p2 = self.vertices[tri[2] as usize];
            let s1 = p0 - p1;
            let s2 = p1 - p2;
            let normal = s1.cross(&s2);
            normals.push(normal.try_normalize(0.0).unwrap_or_else(Vector3::zeros));
        }
        self.triangle_normals = Some(normals);
    }

    /// Compute each vertex's unit normal, by averaging the normals of
    /// adjacent triangles weighted by the angle they subtend at that vertex.
    /// Calls [`Mesh::compute_triangle_normals`] first if needed. Upstream
    /// `Mesh::computeVertexNormals`.
    ///
    /// A vertex touched by no triangle (or only degenerate ones whose
    /// weighted-normal sum is exactly zero) keeps a zero normal rather than
    /// being normalized — matching upstream's explicit `squaredNorm() != 0.0`
    /// guard.
    pub fn compute_vertex_normals(&mut self) {
        if self.triangle_normals.is_none() {
            self.compute_triangle_normals();
        }
        let triangle_normals = self
            .triangle_normals
            .as_ref()
            .expect("populated above if it was None");

        let mut mapped_normals = vec![Vector3::zeros(); self.vertices.len()];
        for (tri, &triangle_normal) in self.triangles.iter().zip(triangle_normals.iter()) {
            let v1 = tri[0] as usize;
            let v2 = tri[1] as usize;
            let v3 = tri[2] as usize;
            let p1 = self.vertices[v1];
            let p2 = self.vertices[v2];
            let p3 = self.vertices[v3];

            let ang1 = (p2 - p1).angle(&(p3 - p1));
            let ang2 = (p1 - p2).angle(&(p3 - p2));
            let ang3 = (p1 - p3).angle(&(p2 - p3));

            mapped_normals[v1] += triangle_normal * ang1;
            mapped_normals[v2] += triangle_normal * ang2;
            mapped_normals[v3] += triangle_normal * ang3;
        }

        for normal in &mut mapped_normals {
            if normal.norm_squared() != 0.0 {
                normal.normalize_mut();
            }
        }
        self.vertex_normals = Some(mapped_normals);
    }

    /// Merge vertices that are within `threshold` of each other, remapping
    /// triangle indices and recomputing whichever normals were already
    /// present. Upstream `Mesh::mergeVertices`.
    pub fn merge_vertices(&mut self, threshold: f64) {
        let threshold_sqr = threshold * threshold;
        let n = self.vertices.len();
        let orig_vertices = self.vertices.clone();
        let mut vertex_map: Vec<usize> = (0..n).collect();
        let mut compressed_vertices: Vec<Vector3> = Vec::new();

        for v1 in 0..n {
            if vertex_map[v1] != v1 {
                continue;
            }
            vertex_map[v1] = compressed_vertices.len();
            compressed_vertices.push(orig_vertices[v1]);

            for v2 in (v1 + 1)..n {
                let distance_sqr = (orig_vertices[v1] - orig_vertices[v2]).norm_squared();
                if distance_sqr <= threshold_sqr {
                    vertex_map[v2] = vertex_map[v1];
                }
            }
        }

        if compressed_vertices.len() == orig_vertices.len() {
            return;
        }

        for tri in &mut self.triangles {
            tri[0] = vertex_map[tri[0] as usize] as u32;
            tri[1] = vertex_map[tri[1] as usize] as u32;
            tri[2] = vertex_map[tri[2] as usize] as u32;
        }
        self.vertices = compressed_vertices;

        if self.triangle_normals.is_some() {
            self.compute_triangle_normals();
        }
        if self.vertex_normals.is_some() {
            self.compute_vertex_normals();
        }
    }

    /// The axis-aligned extents of this mesh's bounding box. Upstream
    /// `computeShapeExtents`'s `MESH` branch: zero for a mesh with fewer
    /// than two vertices.
    pub fn extents(&self) -> Vector3 {
        if self.vertices.len() <= 1 {
            return Vector3::zeros();
        }
        let mut min = Vector3::from_element(f64::MAX);
        let mut max = Vector3::from_element(-f64::MAX);
        for v in &self.vertices {
            min = min.inf(v);
            max = max.sup(v);
        }
        max - min
    }

    /// This mesh's bounding sphere. Upstream `computeShapeBoundingSphere`'s
    /// `MESH` branch: zero for a mesh with fewer than two vertices.
    ///
    /// This is a **loose** bound (the sphere through the AABB's opposite
    /// corners, centered at the AABB's center), matching upstream exactly —
    /// not the minimal enclosing sphere.
    pub fn bounding_sphere(&self) -> BoundingSphere {
        if self.vertices.len() <= 1 {
            return BoundingSphere {
                center: Vector3::zeros(),
                radius: 0.0,
            };
        }
        let mut min = Vector3::from_element(f64::MAX);
        let mut max = Vector3::from_element(-f64::MAX);
        for v in &self.vertices {
            min = min.inf(v);
            max = max.sup(v);
        }
        BoundingSphere {
            center: (min + max) * 0.5,
            radius: (max - min).norm() * 0.5,
        }
    }
}

/// A basic geometric shape: a sphere, cylinder, cone, box, mesh, plane or
/// octree, centered at its own origin. Upstream `shapes::Shape` and its
/// concrete subclasses — see the module docs for why this is a sum type
/// rather than a trait-object hierarchy (D4), and for the scope of what this
/// port of `geometric_shapes` covers.
#[derive(Debug, Clone, PartialEq)]
pub enum Shape {
    /// Upstream `shapes::Sphere`.
    Sphere(Sphere),
    /// Upstream `shapes::Cylinder`.
    Cylinder(Cylinder),
    /// Upstream `shapes::Cone`.
    Cone(Cone),
    /// Upstream `shapes::Box` — see [`Cuboid`] for the rename.
    Cuboid(Cuboid),
    /// Upstream `shapes::Plane`.
    Plane(Plane),
    /// Upstream `shapes::Mesh`.
    Mesh(Mesh),
    /// Upstream `shapes::OcTree`.
    OcTree(OcTree),
}

impl From<Sphere> for Shape {
    fn from(value: Sphere) -> Self {
        Self::Sphere(value)
    }
}

impl From<Cylinder> for Shape {
    fn from(value: Cylinder) -> Self {
        Self::Cylinder(value)
    }
}

impl From<Cone> for Shape {
    fn from(value: Cone) -> Self {
        Self::Cone(value)
    }
}

impl From<Cuboid> for Shape {
    fn from(value: Cuboid) -> Self {
        Self::Cuboid(value)
    }
}

impl From<Plane> for Shape {
    fn from(value: Plane) -> Self {
        Self::Plane(value)
    }
}

impl From<Mesh> for Shape {
    fn from(value: Mesh) -> Self {
        Self::Mesh(value)
    }
}

impl From<OcTree> for Shape {
    fn from(value: OcTree) -> Self {
        Self::OcTree(value)
    }
}

impl Shape {
    /// This shape's [`ShapeType`]. Upstream `Shape::type`.
    pub const fn shape_type(&self) -> ShapeType {
        match self {
            Self::Sphere(_) => ShapeType::Sphere,
            Self::Cylinder(_) => ShapeType::Cylinder,
            Self::Cone(_) => ShapeType::Cone,
            Self::Cuboid(_) => ShapeType::Box,
            Self::Plane(_) => ShapeType::Plane,
            Self::Mesh(_) => ShapeType::Mesh,
            Self::OcTree(_) => ShapeType::OcTree,
        }
    }

    /// Whether this shape can be scaled and/or padded. Upstream
    /// `Shape::isFixed`, overridden `true` for `Plane` and `OcTree`.
    pub const fn is_fixed(&self) -> bool {
        matches!(self, Self::Plane(_) | Self::OcTree(_))
    }

    /// Uniformly scale and pad this shape in place. Upstream the virtual
    /// `Shape::scaleAndPadd(scale, padd)`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when the result would have a negative dimension,
    /// or (for [`Shape::Mesh`]) when vertex normals have not been computed.
    /// [`Shape::Plane`] and [`Shape::OcTree`] can never fail — scaling and
    /// padding them is a no-op.
    pub fn scale_and_padd(&mut self, scale: f64, padding: f64) -> Result<()> {
        match self {
            Self::Sphere(s) => s.scale_and_padd(scale, padding),
            Self::Cylinder(c) => c.scale_and_padd(scale, padding),
            Self::Cone(c) => c.scale_and_padd(scale, padding),
            Self::Cuboid(b) => b.scale_and_padd(scale, padding),
            Self::Mesh(m) => m.scale_and_padd(scale, padding),
            Self::Plane(p) => {
                p.scale_and_padd(scale, padding);
                Ok(())
            }
            Self::OcTree(o) => {
                o.scale_and_padd(scale, padding);
                Ok(())
            }
        }
    }

    /// Uniformly scale this shape in place. Upstream `Shape::scale`.
    pub fn scale(&mut self, scale: f64) -> Result<()> {
        self.scale_and_padd(scale, 0.0)
    }

    /// Add uniform padding to this shape in place. Upstream `Shape::padd`.
    pub fn padd(&mut self, padding: f64) -> Result<()> {
        self.scale_and_padd(1.0, padding)
    }

    /// The axis-aligned extents of this shape's bounding box, centered at
    /// the shape's own origin. Upstream `computeShapeExtents(const Shape*)`.
    /// Zero for [`Shape::Plane`] and [`Shape::OcTree`], matching upstream's
    /// fall-through default.
    pub fn extents(&self) -> Vector3 {
        match self {
            Self::Sphere(s) => s.extents(),
            Self::Cylinder(c) => c.extents(),
            Self::Cone(c) => c.extents(),
            Self::Cuboid(b) => b.extents(),
            Self::Mesh(m) => m.extents(),
            Self::Plane(_) | Self::OcTree(_) => Vector3::zeros(),
        }
    }

    /// A sphere bounding this shape, centered at the shape's own origin.
    /// Upstream `computeShapeBoundingSphere`. Zero-radius, centered at the
    /// origin, for [`Shape::Plane`] and [`Shape::OcTree`], matching
    /// upstream's fall-through default.
    pub fn bounding_sphere(&self) -> BoundingSphere {
        match self {
            Self::Sphere(s) => s.bounding_sphere(),
            Self::Cylinder(c) => c.bounding_sphere(),
            Self::Cone(c) => c.bounding_sphere(),
            Self::Cuboid(b) => b.bounding_sphere(),
            Self::Mesh(m) => m.bounding_sphere(),
            Self::Plane(_) | Self::OcTree(_) => BoundingSphere {
                center: Vector3::zeros(),
                radius: 0.0,
            },
        }
    }

    /// This shape's volume, if upstream `geometric_shapes` defines one.
    ///
    /// `None` for [`Shape::Cone`], [`Shape::Plane`], [`Shape::Mesh`] and
    /// [`Shape::OcTree`] — upstream has no `bodies::` counterpart for any of
    /// them to port a volume formula from. See the module docs.
    pub fn compute_volume(&self) -> Option<f64> {
        match self {
            Self::Sphere(s) => Some(s.compute_volume()),
            Self::Cylinder(c) => Some(c.compute_volume()),
            Self::Cuboid(b) => Some(b.compute_volume()),
            Self::Cone(_) | Self::Plane(_) | Self::Mesh(_) | Self::OcTree(_) => None,
        }
    }

    /// This shape's raw (unscaled, unpadded) dimensions, in the same order
    /// as upstream `bodies::*::getDimensions()`: `[radius]` for a sphere,
    /// `[radius, length]` for a cylinder, `[x, y, z]` for a cuboid.
    ///
    /// `None` for [`Shape::Cone`], [`Shape::Plane`], [`Shape::Mesh`] and
    /// [`Shape::OcTree`] — see [`Shape::compute_volume`] and the module
    /// docs; the same absence of an upstream `bodies::` counterpart applies.
    pub fn get_dimensions(&self) -> Option<Vec<f64>> {
        match self {
            Self::Sphere(s) => Some(vec![s.radius]),
            Self::Cylinder(c) => Some(vec![c.radius, c.length]),
            Self::Cuboid(b) => Some(b.size.to_vec()),
            Self::Cone(_) | Self::Plane(_) | Self::Mesh(_) | Self::OcTree(_) => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Round 14's §79 sweep: all 45 of this file's `assert_relative_eq!`
    // calls converted to `assert_eq!` below, none left approximate.
    // `assert_relative_eq!` with neither `epsilon` nor `max_relative` given
    // defaults both to `f64::EPSILON`; the 7 remaining calls gave an
    // explicit `epsilon = 1e-12` but no `max_relative`. Every one was
    // bisected to a literal `epsilon = 0.0, max_relative = 0.0` (confirmed,
    // not assumed) and still passed, then reconfirmed passing as
    // `assert_eq!` here. Two reasons cover all 45: (1) most compare a
    // scale/padd/volume result built from small, exactly-representable
    // literals (halves, quarters, small integers) against a hand-computed
    // literal of the same arithmetic, with no rounding step for the two
    // sides to disagree on; (2) the handful involving `PI`,
    // cross-product/`normalize`, or `1.0/3.0_f64.sqrt()` still land
    // bit-exact for these specific inputs -- axis-aligned cross products,
    // `2.0 * 2.0`-style power-of-two products, and additions of exact
    // sign-symmetric pairs -- each says why at its own call site.

    // --- Construction: zero and negative dimensions ---

    #[test]
    fn sphere_zero_radius_is_valid() {
        assert_eq!(Sphere::new(0.0).unwrap().radius, 0.0);
    }

    // Assertion-discrimination sweep (round 2, verdict corrected this
    // round): `Sphere::new` has one operand and one guard -- nothing for
    // an isolating mutation to separate, so it is genuinely
    // `single-branch`. `Cylinder::new`, `Cone::new`, and `Cuboid::new`
    // do NOT share that verdict, despite each rejecting every axis
    // through one combined `< 0.0 || ...` check with a single shared
    // message (unlike `bodies::Cylinder::recompute`'s two *sequential*
    // guards with distinct "radius"/"length" messages, see `bodies.rs`).
    // "One `Error::` token" is not "one cause" -- see brief section 3's
    // `search_position_ik` example. Bite-checked, both directions
    // (comment out the sibling assertion(s) first, since `assert!`
    // short-circuits within one test fn): neutralizing one operand's
    // clause alone fails only the assertion whose fixture drives that
    // operand negative and leaves the sibling assertion green.
    // Cylinder::new confirmed both directions (radius clause neutralized
    // -> the radius assertion fails, the length assertion stays green;
    // length clause neutralized -> mirrors); Cone::new confirmed the
    // radius direction (radius assertion fails, length assertion green);
    // Cuboid::new confirmed x vs. z (x assertion fails, z assertion
    // green). Verdict `multi-branch`, discriminating, for
    // the `cylinder_negative_dimension_is_an_error`,
    // `cone_negative_dimension_is_an_error`, and
    // `cuboid_negative_dimension_is_an_error` tests below -- each
    // per-axis assertion is its own discriminating site.
    #[test]
    fn sphere_negative_radius_is_an_error() {
        assert!(Sphere::new(-1.0).is_err());
    }

    #[test]
    fn cylinder_negative_dimension_is_an_error() {
        assert!(Cylinder::new(-1.0, 1.0).is_err());
        assert!(Cylinder::new(1.0, -1.0).is_err());
        assert!(Cylinder::new(0.0, 0.0).is_ok());
    }

    #[test]
    fn cone_negative_dimension_is_an_error() {
        assert!(Cone::new(-1.0, 1.0).is_err());
        assert!(Cone::new(1.0, -1.0).is_err());
    }

    #[test]
    fn cuboid_negative_dimension_is_an_error() {
        assert!(Cuboid::new(-1.0, 1.0, 1.0).is_err());
        assert!(Cuboid::new(1.0, -1.0, 1.0).is_err());
        assert!(Cuboid::new(1.0, 1.0, -1.0).is_err());
    }

    #[test]
    fn plane_never_rejects_a_dimension() {
        // Upstream's constructor does not validate — a, b, c, d are plane
        // equation coefficients, not lengths.
        let p = Plane::new(-1.0, -2.0, -3.0, -4.0);
        assert_eq!((p.a, p.b, p.c, p.d), (-1.0, -2.0, -3.0, -4.0));
    }

    // --- scaleAndPadd: padding that would invert a dimension ---

    // Assertion-discrimination sweep (round 2): `Sphere::scale_and_padd`
    // has exactly one `Err` site -- verdict `single-branch`.
    #[test]
    fn sphere_padding_past_negative_is_an_error() {
        let mut s = Sphere::new(1.0).unwrap();
        assert!(s.scale_and_padd(1.0, -2.0).is_err());
        // The field is untouched on error.
        assert_eq!(s.radius, 1.0);
    }

    // Assertion-discrimination sweep (round 2, verdict corrected this
    // round): `Cylinder::scale_and_padd_axes` has the same combined
    // `radius < 0.0 || length < 0.0` guard as `Cylinder::new` above (no
    // per-axis message) -- an earlier revision of this comment called
    // that `single-branch` "regardless of this test's own care to
    // isolate radius from length per case", reasoning the isolation only
    // mattered for the padding arithmetic. That repeats the same "one
    // token, so one cause" error corrected above `Cylinder::new`.
    // Bite-checked: neutralizing the radius clause alone fails
    // `radius_case`'s assertion and leaves `length_case`'s green;
    // neutralizing length mirrors it. Verdict `multi-branch`,
    // discriminating -- each case's assertion is its own site.
    #[test]
    fn cylinder_padding_past_negative_is_an_error_per_axis() {
        let mut radius_case = Cylinder::new(1.0, 10.0).unwrap();
        assert!(
            radius_case
                .scale_and_padd_axes(1.0, 1.0, -2.0, 0.0)
                .is_err()
        );

        let mut length_case = Cylinder::new(10.0, 1.0).unwrap();
        // length is halved-then-doubled via 2*padd, so -0.6 already inverts a
        // length of 1.0 (1.0 + 2*(-0.6) = -0.2).
        assert!(
            length_case
                .scale_and_padd_axes(1.0, 1.0, 0.0, -0.6)
                .is_err()
        );
    }

    #[test]
    fn cylinder_scale_and_padd_matches_manual_axes_call() {
        let mut a = Cylinder::new(2.0, 4.0).unwrap();
        let mut b = Cylinder::new(2.0, 4.0).unwrap();
        a.scale_and_padd(1.5, 0.25).unwrap();
        b.scale_and_padd_axes(1.5, 1.5, 0.25, 0.25).unwrap();
        assert_eq!(a, b);
        // radius: 2*1.5+0.25 = 3.25; length: 4*1.5 + 2*0.25 = 6.5
        assert_eq!(a.radius, 3.25);
        assert_eq!(a.length, 6.5);
    }

    #[test]
    fn cuboid_scale_and_padd_matches_manual_axes_call() {
        let mut a = Cuboid::new(1.0, 2.0, 3.0).unwrap();
        let mut b = Cuboid::new(1.0, 2.0, 3.0).unwrap();
        a.scale_and_padd(2.0, 0.5).unwrap();
        b.scale_and_padd_axes(2.0, 2.0, 2.0, 0.5, 0.5, 0.5).unwrap();
        assert_eq!(a, b);
        assert_eq!(a.size, [3.0, 5.0, 7.0]);
    }

    #[test]
    fn plane_and_octree_scale_and_padd_are_a_no_op() {
        let mut p = Plane::new(1.0, 2.0, 3.0, 4.0);
        p.scale_and_padd(100.0, 100.0);
        assert_eq!((p.a, p.b, p.c, p.d), (1.0, 2.0, 3.0, 4.0));

        let mut shape = Shape::Plane(p);
        assert!(shape.scale_and_padd(100.0, 100.0).is_ok());

        let mut o = Shape::OcTree(OcTree::new());
        assert!(o.scale_and_padd(100.0, 100.0).is_ok());
        assert!(o.is_fixed());
    }

    // --- computeVolume: degenerate cylinder, and the "no upstream Body" cases ---

    #[test]
    fn sphere_volume() {
        let s = Sphere::new(1.0).unwrap();
        // Bit-exact (round 14, §79): bisected to epsilon = 0.0, max_relative
        // = 0.0 and still passed. `compute_volume` multiplies radius = 1.0
        // by itself twice (no-ops) then by `4.0 / 3.0 * PI`, matching this
        // literal's own operand order and grouping exactly.
        assert_eq!(s.compute_volume(), 4.0 / 3.0 * std::f64::consts::PI);
    }

    #[test]
    fn degenerate_cylinder_volume_is_zero() {
        // Zero length: a disc, zero volume.
        assert_eq!(Cylinder::new(1.0, 0.0).unwrap().compute_volume(), 0.0);
        // Zero radius: a line, zero volume.
        assert_eq!(Cylinder::new(0.0, 1.0).unwrap().compute_volume(), 0.0);
    }

    #[test]
    fn cylinder_volume_matches_pi_r2_h() {
        let c = Cylinder::new(2.0, 3.0).unwrap();
        // Bit-exact (round 14, §79): bisected to epsilon = 0.0, max_relative
        // = 0.0 and still passed. `compute_volume` is `PI * radius *
        // radius * length` = `PI * 2.0 * 2.0 * 3.0`; `2.0 * 2.0` is an exact
        // power-of-two multiply, so the running product matches this
        // literal's `PI * 4.0 * 3.0` bit for bit.
        assert_eq!(c.compute_volume(), std::f64::consts::PI * 4.0 * 3.0);
    }

    #[test]
    fn cuboid_volume() {
        let b = Cuboid::new(2.0, 3.0, 4.0).unwrap();
        assert_eq!(b.compute_volume(), 24.0);
    }

    // Assertion-discrimination sweep (round 2, D6 check per brief section
    // 3a added; `single-branch` verdict below corrected this round):
    // `Shape::compute_volume` and `Shape::get_dimensions` each have
    // exactly one `None`-producing arm, `Self::Cone(_) | Self::Plane(_) |
    // Self::Mesh(_) | Self::OcTree(_) => None`. The original verdict
    // argued Rust's exhaustive `match` on the loop variable's *input*
    // variant ("only that variant's arm can execute") meant "there is
    // nothing to isolate" -- but that only fixes which arm's *pattern*
    // matches a given call, not what that arm's *body* returns. Splitting
    // `Self::Cone(_)` (or `Self::Plane(_)`, confirmed separately) out of
    // the combined arm into its own `Self::X(_) => Some(999.0)` still
    // compiles and fails exactly that shape's loop iteration for both
    // `compute_volume` and `get_dimensions`, leaving the other three
    // variants' iterations green -- confirmed live. Verdict `multi-branch`,
    // discriminating: this is a real, passing test, not an unreachable
    // assertion. D6 check, from the actual call sites, not the signature,
    // still stands: `rg` for `\.compute_volume\(\)|\.get_dimensions\(\)`
    // workspace-wide shows no caller of *this* `Shape`-level method
    // outside its own test module -- `bodies.rs`'s `Body::compute_volume`
    // (called from `body_query_parity.rs`/`probe_parity.rs`) is a
    // different, non-Option method reached only through
    // `Body::from_shape`'s already-narrowed `Sphere`/`Cylinder`/`Cuboid`/
    // `ConvexMesh` variants. No caller needs to distinguish the four
    // `None` reasons, so this is not a D6 finding.
    #[test]
    fn shapes_with_no_upstream_body_have_no_volume_or_dimensions() {
        for shape in [
            Shape::Cone(Cone::new(1.0, 1.0).unwrap()),
            Shape::Plane(Plane::new(0.0, 0.0, 1.0, 0.0)),
            Shape::Mesh(Mesh::default()),
            Shape::OcTree(OcTree::new()),
        ] {
            assert!(shape.compute_volume().is_none());
            assert!(shape.get_dimensions().is_none());
        }
    }

    #[test]
    fn shape_get_dimensions_matches_bodies_ordering() {
        let sphere = Shape::Sphere(Sphere::new(1.5).unwrap());
        assert_eq!(sphere.get_dimensions(), Some(vec![1.5]));

        let cylinder = Shape::Cylinder(Cylinder::new(1.5, 3.0).unwrap());
        assert_eq!(cylinder.get_dimensions(), Some(vec![1.5, 3.0]));

        let cuboid = Shape::Cuboid(Cuboid::new(1.0, 2.0, 3.0).unwrap());
        assert_eq!(cuboid.get_dimensions(), Some(vec![1.0, 2.0, 3.0]));
    }

    // --- extents / bounding_sphere ---

    #[test]
    fn cone_bounding_sphere_tall_vs_short() {
        // length (10) > radius (1): tall cone, sphere touches tip and base rim.
        let tall = Cone::new(1.0, 10.0).unwrap();
        let bs = tall.bounding_sphere();
        assert!(bs.radius > 0.0);
        assert_ne!(bs.center.z, 0.0);

        // radius (10) > length (1): short cone, sphere touches base rim only,
        // centered on the base.
        let short = Cone::new(10.0, 1.0).unwrap();
        let bs = short.bounding_sphere();
        assert_eq!(bs.radius, 10.0);
        assert_eq!(bs.center.z, -0.5);
    }

    #[test]
    fn sphere_extents_and_bounding_sphere() {
        let s = Sphere::new(2.0).unwrap();
        assert_eq!(s.extents(), Vector3::new(4.0, 4.0, 4.0));
        let bs = s.bounding_sphere();
        assert_eq!(bs.center, Vector3::zeros());
        assert_eq!(bs.radius, 2.0);
    }

    // --- Mesh: zero triangles, degenerate triangles, missing normals ---

    #[test]
    fn mesh_with_zero_triangles_is_constructible() {
        let vertices = vec![Vector3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0)];
        let mesh = Mesh::new(vertices, vec![]).unwrap();
        assert!(mesh.triangles.is_empty());
        // 2 vertices is enough for a real AABB even with no triangles.
        assert_eq!(mesh.extents(), Vector3::new(1.0, 0.0, 0.0));
    }

    // Assertion-discrimination sweep (round 2): `Mesh::new` has exactly
    // one `Error::construct(..)` call site (inside the nested
    // vertex-index-range loop) -- verdict `single-branch` for the
    // `matches!(err, Error::Construct(_))` check, one `Error::` hit over
    // the function body, even though that one call site can fire for
    // different (triangle, index) pairs depending on input.
    #[test]
    fn mesh_rejects_out_of_range_triangle_index() {
        let vertices = vec![Vector3::zeros(), Vector3::new(1.0, 0.0, 0.0)];
        let err = Mesh::new(vertices, vec![[0, 1, 2]]).unwrap_err();
        assert!(matches!(err, Error::Construct(_)));
    }

    #[test]
    fn mesh_with_one_vertex_has_zero_extents_and_bounding_sphere() {
        let mesh = Mesh::new(vec![Vector3::new(5.0, 5.0, 5.0)], vec![]).unwrap();
        assert_eq!(mesh.extents(), Vector3::zeros());
        let bs = mesh.bounding_sphere();
        assert_eq!(bs.center, Vector3::zeros());
        assert_eq!(bs.radius, 0.0);
    }

    #[test]
    fn mesh_zero_vertex_scale_and_padd_is_a_no_op() {
        // Upstream: the loop that divides by vertex_count and reads
        // vertex_normals never runs, so this must not panic or error even
        // though vertex_normals is None.
        let mut mesh = Mesh::default();
        assert!(mesh.scale_and_padd(2.0, 1.0).is_ok());
    }

    // Assertion-discrimination sweep (round 2): `scale_axes`/`padd_axes`
    // both funnel into `scale_and_padd_axes`, whose only `Err` site is
    // the `vertex_normals.as_ref().ok_or_else(..)` (the
    // `vertices.is_empty()` branch returns `Ok(())`, not an error) --
    // verdict `single-branch` for both assertions.
    #[test]
    fn mesh_padding_without_vertex_normals_is_an_error() {
        let mesh_vertices = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ];
        let mut mesh = Mesh::new(mesh_vertices, vec![[0, 1, 2]]).unwrap();
        // Even scale-only (padding 0.0) must error: upstream dereferences
        // vertex_normals unconditionally.
        assert!(mesh.scale_axes(2.0, 2.0, 2.0).is_err());
        assert!(mesh.padd_axes(0.1, 0.1, 0.1).is_err());
    }

    #[test]
    fn mesh_scale_and_padd_succeeds_once_normals_are_computed() {
        let vertices = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ];
        let mut mesh = Mesh::new(vertices, vec![[0, 1, 2]]).unwrap();
        mesh.compute_vertex_normals();
        assert!(mesh.scale_and_padd(1.0, 0.0).is_ok());
    }

    #[test]
    fn degenerate_triangle_normal_is_zero_not_nan() {
        // Two coincident vertices: zero-area triangle, zero cross product.
        let vertices = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
        ];
        let mut mesh = Mesh::new(vertices, vec![[0, 1, 2]]).unwrap();
        mesh.compute_triangle_normals();
        let normal = mesh.triangle_normals.unwrap()[0];
        assert_eq!(normal, Vector3::zeros());
        assert!(!normal.x.is_nan());
    }

    #[test]
    fn compute_triangle_normal_matches_right_hand_rule() {
        // Right triangle in the xy-plane, CCW when viewed from +z.
        let vertices = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ];
        let mut mesh = Mesh::new(vertices, vec![[0, 1, 2]]).unwrap();
        mesh.compute_triangle_normals();
        // Bit-exact (round 14, §79): bisected to epsilon = 0.0, max_relative
        // = 0.0 and still passed. `(1,0,0) x (0,1,0) = (0,0,1)` exactly (each
        // component is a sum of exact products of 0.0/1.0), and its norm is
        // exactly 1.0, so normalizing is a no-op divide-by-one -- no `sqrt`
        // rounding for this specific right-triangle input.
        assert_eq!(
            mesh.triangle_normals.unwrap()[0],
            Vector3::new(0.0, 0.0, 1.0)
        );
    }

    #[test]
    fn compute_vertex_normals_calls_triangle_normals_when_needed() {
        let vertices = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ];
        let mut mesh = Mesh::new(vertices, vec![[0, 1, 2]]).unwrap();
        // Assertion-discrimination sweep (round 2): reads a struct field
        // set to a literal `None` in `Mesh::new` -- not a computed
        // branch at all, so there is nothing to discriminate.
        assert!(mesh.triangle_normals.is_none());
        mesh.compute_vertex_normals();
        assert!(mesh.triangle_normals.is_some());
        let vertex_normals = mesh.vertex_normals.unwrap();
        assert_eq!(vertex_normals.len(), 3);
        for n in vertex_normals {
            // Bit-exact (round 14, §79): bisected to epsilon = 0.0,
            // max_relative = 0.0 and still passed. Single-triangle mesh, so
            // each vertex normal is that one triangle's normal with no
            // averaging across triangles -- same (0,0,1)-exact reasoning as
            // `compute_triangle_normal_matches_right_hand_rule` above.
            assert_eq!(n, Vector3::new(0.0, 0.0, 1.0));
        }
    }

    #[test]
    fn compute_vertex_normals_leaves_unreferenced_vertex_at_zero() {
        // A vertex with no incident triangle keeps a zero (not NaN) normal.
        let vertices = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(9.0, 9.0, 9.0),
        ];
        let mut mesh = Mesh::new(vertices, vec![[0, 1, 2]]).unwrap();
        mesh.compute_vertex_normals();
        let vertex_normals = mesh.vertex_normals.unwrap();
        assert_eq!(vertex_normals[3], Vector3::zeros());
    }

    #[test]
    fn merge_vertices_collapses_within_threshold_and_remaps_triangles() {
        let vertices = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0001, 0.0, 0.0), // within threshold of vertex 0
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ];
        let mut mesh = Mesh::new(vertices, vec![[0, 2, 3], [1, 2, 3]]).unwrap();
        mesh.merge_vertices(0.01);
        assert_eq!(mesh.vertices.len(), 3);
        // Both triangles' first index now refer to the same merged vertex.
        assert_eq!(mesh.triangles[0][0], mesh.triangles[1][0]);
    }

    #[test]
    fn merge_vertices_is_a_no_op_when_nothing_is_within_threshold() {
        let vertices = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ];
        let mut mesh = Mesh::new(vertices.clone(), vec![[0, 1, 2]]).unwrap();
        mesh.merge_vertices(1e-9);
        assert_eq!(mesh.vertices, vertices);
    }

    // --- ShapeType / Shape::shape_type round trip ---

    #[test]
    fn shape_type_round_trips_through_display() {
        let cases = [
            (Shape::Sphere(Sphere::default()), "sphere"),
            (Shape::Cylinder(Cylinder::default()), "cylinder"),
            (Shape::Cone(Cone::default()), "cone"),
            (Shape::Cuboid(Cuboid::default()), "box"),
            (Shape::Plane(Plane::default()), "plane"),
            (Shape::Mesh(Mesh::default()), "mesh"),
            (Shape::OcTree(OcTree::new()), "octree"),
        ];
        for (shape, expected) in cases {
            assert_eq!(shape.shape_type().to_string(), expected);
        }
    }

    #[test]
    fn from_impls_wrap_into_shape() {
        let s: Shape = Sphere::new(1.0).unwrap().into();
        assert_eq!(s.shape_type(), ShapeType::Sphere);
    }

    // --- Ground truth: geometric_shapes 2.3.3 test/test_shapes.cpp ---
    //
    // These reproduce upstream's `TEST(<Type>, ScaleAndPadd)` cases exactly,
    // call for call, using the same literal expected values, as ground truth
    // for the scaleAndPadd formulas.

    #[test]
    fn ground_truth_plane_scale_and_padd() {
        let plane = Plane::new(1.0, 1.0, 1.0, 1.0);
        let mut plane2 = plane;
        assert_eq!(
            (plane2.a, plane2.b, plane2.c, plane2.d),
            (1.0, 1.0, 1.0, 1.0)
        );

        plane2.scale(2.0);
        assert_eq!(
            (plane2.a, plane2.b, plane2.c, plane2.d),
            (1.0, 1.0, 1.0, 1.0)
        );

        plane2.padd(1.0);
        assert_eq!(
            (plane2.a, plane2.b, plane2.c, plane2.d),
            (1.0, 1.0, 1.0, 1.0)
        );

        plane2.scale_and_padd(2.0, 1.0);
        assert_eq!(
            (plane2.a, plane2.b, plane2.c, plane2.d),
            (1.0, 1.0, 1.0, 1.0)
        );
    }

    #[test]
    fn ground_truth_octree_scale_and_padd_empty() {
        let mut octree = OcTree::new();
        octree.scale(2.0);
        octree.padd(1.0);
        octree.scale_and_padd(2.0, 1.0);
    }

    #[test]
    fn ground_truth_sphere_scale_and_padd() {
        let sphere = Sphere::new(1.0).unwrap();
        let mut sphere2 = sphere;
        assert_eq!(sphere2.radius, sphere.radius);

        sphere2.scale(2.0).unwrap();
        assert_eq!(sphere2.radius, 2.0);

        sphere2.padd(1.0).unwrap();
        assert_eq!(sphere2.radius, 3.0);

        sphere2.scale_and_padd(2.0, 1.0).unwrap();
        assert_eq!(sphere2.radius, 7.0);
    }

    #[test]
    fn ground_truth_cylinder_scale_and_padd() {
        let mut c = Cylinder::new(1.0, 2.0).unwrap();

        c.scale(2.0).unwrap();
        assert_eq!(c.radius, 2.0);
        assert_eq!(c.length, 4.0);

        c.padd(1.0).unwrap();
        assert_eq!(c.radius, 3.0);
        assert_eq!(c.length, 6.0);

        c.scale_and_padd(2.0, 1.0).unwrap();
        assert_eq!(c.radius, 7.0);
        assert_eq!(c.length, 14.0);

        c.scale_and_padd_axes(1.0, 3.0, 1.0, 3.0).unwrap();
        assert_eq!(c.radius, 8.0);
        assert_eq!(c.length, 48.0);

        c.scale_axes(2.0, 1.5).unwrap();
        assert_eq!(c.radius, 16.0);
        assert_eq!(c.length, 72.0);

        c.padd_axes(2.0, 3.0).unwrap();
        assert_eq!(c.radius, 18.0);
        assert_eq!(c.length, 78.0);
    }

    #[test]
    fn ground_truth_cone_scale_and_padd() {
        // Identical numeric chain to the cylinder case above — upstream's
        // Cone::scaleAndPadd is the same formula as Cylinder's.
        let mut c = Cone::new(1.0, 2.0).unwrap();

        c.scale(2.0).unwrap();
        assert_eq!(c.radius, 2.0);
        assert_eq!(c.length, 4.0);

        c.padd(1.0).unwrap();
        assert_eq!(c.radius, 3.0);
        assert_eq!(c.length, 6.0);

        c.scale_and_padd(2.0, 1.0).unwrap();
        assert_eq!(c.radius, 7.0);
        assert_eq!(c.length, 14.0);

        c.scale_and_padd_axes(1.0, 3.0, 1.0, 3.0).unwrap();
        assert_eq!(c.radius, 8.0);
        assert_eq!(c.length, 48.0);

        c.scale_axes(2.0, 1.5).unwrap();
        assert_eq!(c.radius, 16.0);
        assert_eq!(c.length, 72.0);

        c.padd_axes(2.0, 3.0).unwrap();
        assert_eq!(c.radius, 18.0);
        assert_eq!(c.length, 78.0);
    }

    #[test]
    fn ground_truth_cuboid_scale_and_padd() {
        fn assert_size(size: [f64; 3], expected: [f64; 3]) {
            assert_eq!(size[0], expected[0]);
            assert_eq!(size[1], expected[1]);
            assert_eq!(size[2], expected[2]);
        }

        let mut b = Cuboid::new(1.0, 2.0, 3.0).unwrap();

        b.scale(2.0).unwrap();
        assert_size(b.size, [2.0, 4.0, 6.0]);

        b.padd(1.0).unwrap();
        assert_size(b.size, [4.0, 6.0, 8.0]);

        b.scale_and_padd(2.0, 1.0).unwrap();
        assert_size(b.size, [10.0, 14.0, 18.0]);

        b.scale_and_padd_axes(1.0, 2.0, 3.0, 1.0, 2.0, 3.0).unwrap();
        assert_size(b.size, [12.0, 32.0, 60.0]);

        b.scale_axes(1.0, 2.0, 3.0).unwrap();
        assert_size(b.size, [12.0, 64.0, 180.0]);

        b.padd_axes(1.0, 2.0, 3.0).unwrap();
        assert_size(b.size, [14.0, 68.0, 186.0]);
    }

    #[test]
    fn ground_truth_mesh_scale_and_padd() {
        // Reproduces `TEST(Mesh, ScaleAndPadd)` through its first
        // scale/padd/scaleAndPadd chain. Upstream loads a unit cube from
        // `box.dae` via assimp (no Rust equivalent is in scope here — see
        // the module docs); this builds the same 8 corner vertices directly.
        // Each corner's vertex normal is the normalized corner position: for
        // a cube, the three axis-aligned faces meeting at a corner are
        // mutually perpendicular, so the angle-weighted average in
        // `compute_vertex_normals` (verified separately, see
        // `compute_vertex_normals_calls_triangle_normals_when_needed`)
        // reduces to the normalized corner direction by symmetry. Setting it
        // directly here isolates the scale/padd formula under test from mesh
        // topology/triangulation, which isn't part of the ground truth being
        // checked.
        let corners = [
            Vector3::new(1.0, 1.0, -1.0),
            Vector3::new(1.0, -1.0, -1.0),
            Vector3::new(-1.0, -1.0, -1.0),
            Vector3::new(-1.0, 1.0, -1.0),
            Vector3::new(1.0, 1.0, 1.0),
            Vector3::new(-1.0, 1.0, 1.0),
            Vector3::new(-1.0, -1.0, 1.0),
            Vector3::new(1.0, -1.0, 1.0),
        ];
        let mut mesh = Mesh::new(corners.to_vec(), vec![]).unwrap();
        mesh.vertex_normals = Some(corners.iter().map(|v| v.normalize()).collect());

        // Bit-exact throughout (round 14, §79): all three loops below
        // bisected to epsilon = 0.0, max_relative = 0.0 and still passed.
        // The 8 corners are symmetric about the origin in ±1.0 per axis, so
        // the centroid `scale_and_padd_axes` computes sums to exactly 0.0
        // (each addition either combines equal magnitudes with opposite
        // sign, which IEEE 754 gives back exactly, or doubles a value,
        // which is an exact power-of-two multiply) -- so `d = v - centroid`
        // is `v` itself, unrounded. `c * 2.0` is then an exact power-of-two
        // scale, matching this loop's own `c * 2.0` literal bit for bit.
        mesh.scale(2.0).unwrap();
        for (v, c) in mesh.vertices.iter().zip(&corners) {
            assert_eq!(*v, c * 2.0);
        }

        // For a right-angled corner, the vertex normal points away equally
        // from the three sides, so padding of 1.0 moves each vertex by
        // 1.0 total, split equally (1/sqrt(3)) across x, y, z. Per corner
        // component, `v.x = 2.0*c.x + vn.x` where `vn.x = normalize(c).x`
        // is exactly `c.x * (1.0 / 3.0_f64.sqrt())` (a single sign flip,
        // exact, off the same `sqrt(3.0)` every corner shares) -- so this
        // is `±2.0 ± (1.0/sqrt(3))`, which IEEE 754 addition/negation gives
        // back identically to `c * (2.0 + 1.0/3.0_f64.sqrt())` regardless
        // of which term is negated.
        mesh.padd(1.0).unwrap();
        let pos = 2.0 + 1.0 / 3.0_f64.sqrt();
        for (v, c) in mesh.vertices.iter().zip(&corners) {
            assert_eq!(*v, c * pos);
        }

        mesh.scale_and_padd(2.0, 1.0).unwrap();
        let pos2 = pos * 2.0 + 1.0 / 3.0_f64.sqrt();
        for (v, c) in mesh.vertices.iter().zip(&corners) {
            assert_eq!(*v, c * pos2);
        }
    }
}
