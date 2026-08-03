// Copyright 2008, 2019, 2024 Willow Garage, Inc. / Open Robotics
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
// FCL source-availability gap: `bodies::OBB` is a PIMPL wrapping FCL's
// `fcl::OBB<double>` (`obb.cpp` `#include <fcl/math/bv/OBB.h>`). FCL is a
// separate upstream package from `geometric_shapes`; it is not vendored
// anywhere on this machine, not in the oracle container's
// `geometric_shapes` source tree, and fetching+verifying FCL's own source
// was out of the time this port had. `OBB::contains`/`overlaps`/
// `extendApprox`'s fallback merge are therefore this port's own
// implementations of the documented behavior (see the module docs on
// [`OBB`]), not literal ports — this is called out explicitly rather than
// silently presented as a port of code that was never read.

//! The posed, algorithmic half of `geometric_shapes`: `bodies::Body` and its
//! four concrete kinds, plus the bounding-volume types they return. Upstream
//! `namespace bodies` from the `geometric_shapes` package — see the
//! provenance comment above for how the source was obtained and verified,
//! and [`crate::shapes`] for the unposed shape data layer this module builds
//! on.
//!
//! # Scope
//!
//! This module ports `bodies::Body`'s four concrete subclasses — [`Sphere`],
//! [`Cylinder`], [`Cuboid`] (upstream `bodies::Box`, renamed for the same
//! reason as [`crate::Cuboid`]) and [`ConvexMesh`] — along with
//! `setPose`/`setDimensions`/`setScale`/`setPadding`, `containsPoint`,
//! `intersectsRay`, `computeVolume`, `computeBoundingSphere` (here
//! [`Body::compute_bounding_sphere`]), `computeBoundingCylinder`,
//! `samplePointInside`, [`AABB`], [`OBB`], and the free function
//! `mergeBoundingSpheres` (here [`merge_bounding_spheres`]). The free
//! function `computeBoundingSphere(vector<Body*>)` (`body_operations.h`)
//! has no dedicated port — a caller composes it from
//! [`Body::compute_bounding_sphere`] and [`merge_bounding_spheres`] directly.
//! `mergeBoundingBoxes`/`mergeBoundingBoxesApprox` (here
//! [`merge_bounding_boxes`]/[`merge_bounding_boxes_approx`]) are included too
//! — they are one-line loops over [`AABB::extend_aabb`]/[`OBB::extend_approx`]
//! once those types exist, and upstream's own test coverage for [`OBB`]
//! (`test_bounding_box.cpp`'s `MergeBoundingBoxes` suite) runs through them.
//!
//! It deliberately does **not** port `bodies::BodyVector` (a thin
//! `Vec<Body>`-plus-first-hit-query convenience wrapper), or the
//! message/marker-facing free functions in `body_operations.h`
//! (`createEmptyBodyFromShapeType`'s `shape_msgs`/`visualization_msgs`
//! siblings, `constructShapeFromBody`, `constructMarkerFromBody`,
//! `constructBodyFromMsg`) — none of these were in the requested scope, and
//! the message-facing ones are out of scope for the same reason as
//! `shapes.rs`'s message conversions (PORTING-PLAN.md D1).
//!
//! # Design: enum, not a trait-object hierarchy (D4)
//!
//! Upstream `bodies::Body` is `geometric_shapes`'s second abstract base
//! class (after `shapes::Shape`), with `Sphere`, `Cylinder`, `Box`,
//! `ConvexMesh` as concrete subclasses carrying a `shapes::ShapeType type_`
//! tag alongside the `static_cast<const Sphere*>(body)`-style downcasts that
//! tag licenses — the exact "value's meaning depends on a side tag" pattern
//! [`crate::shapes`]'s module docs already argue against for `shapes::Shape`.
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
//!    half-spaces are the same region). [`ConvexMesh`] does not expose
//!    `getTriangles`/`getVertices`/`getScaledVertices`/`getPlanes` — they
//!    were not in the requested scope, and dropping the plane-merge maps
//!    means this port's plane count does not match upstream's 1:1 anyway.
//! 3. **`bodies::OBB`'s FCL-backed methods are this port's own
//!    implementation, not a literal port.** See the provenance comment
//!    above. [`OBB::contains_point`] is the one unambiguous case (inverse-
//!    transform the point into the box's local frame, compare against the
//!    half-extents componentwise — there is only one reasonable meaning for
//!    "an oriented box contains a point"). [`OBB::overlaps`] implements the
//!    standard 15-axis separating-axis test for two oriented boxes (3 face
//!    normals of each box, plus the 9 pairwise cross products — Gottschalk
//!    et al. 1996; Ericson, *Real-Time Collision Detection* §4.4.1), a
//!    textbook algorithm independent of FCL's implementation.
//!    [`OBB::extend_approx`]'s two shortcut cases (this box has zero
//!    extent; one box wholly contains the other) are ported byte-for-byte
//!    from `obb.cpp`, which spells them out before delegating the general
//!    case to FCL's `OBB::operator+=`. For that general case (neither box
//!    contains the other), this port computes the tightest box that shares
//!    this box's orientation and enclosses both boxes' vertices — a
//!    different, and generally less tight, approximation than FCL's, but a
//!    valid enclosing OBB. `test_bounding_box.cpp`'s `OBBApprox1` is the
//!    only upstream test that reaches this branch, and it asserts only
//!    loose sanity bounds (`EXPECT_GE`/`EXPECT_LE` on extent and
//!    translation ranges, plus `contains`/`overlaps` on the inputs) rather
//!    than exact literals — this port's implementation satisfies all of
//!    them (verified in this module's tests, transcribed from that test).
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

use moveit_error::{Error, Result};

use crate::shapes::{Mesh as ShapeMesh, Shape};
use crate::{BoundingSphere, Isometry3, Vector3};

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
/// of the previous kept point — the case where a ray grazes exactly the
/// shared boundary between two primitives, e.g. a cylinder's side and base,
/// and is reported once per primitive), and cap the result at `count` points
/// (`None` keeps them all). Upstream `detail::filterIntersections`.
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
            if (c.pt - last).norm() <= ZERO * last.norm().max(c.pt.norm()).max(1.0) {
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

        // Neither box contains the other: build the tightest box sharing
        // this box's orientation that encloses both boxes' vertices — see
        // the module docs, deviation 3.
        let inv = self.pose.inverse();
        let mut local_min = Vector3::from_element(f64::INFINITY);
        let mut local_max = Vector3::from_element(f64::NEG_INFINITY);
        for v in self
            .compute_vertices()
            .into_iter()
            .chain(other.compute_vertices())
        {
            let local = transform_point(&inv, &v);
            local_min = local_min.inf(&local);
            local_max = local_max.sup(&local);
        }
        let local_center = (local_min + local_max) * 0.5;
        let new_extents = local_max - local_min;
        let new_pose =
            self.pose * Isometry3::translation(local_center.x, local_center.y, local_center.z);
        self.set_pose_and_extents(new_pose, new_extents);
    }
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

/// A sphere body: a posed, scaled, padded [`crate::shapes::Sphere`].
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

/// A cylinder body: a posed, scaled, padded [`crate::shapes::Cylinder`].
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

/// A box body: a posed, scaled, padded [`crate::shapes::Cuboid`]. Upstream
/// `bodies::Box`, renamed to avoid shadowing [`std::boxed::Box`] — the same
/// reason as [`crate::Cuboid`].
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
/// half of upstream `ConvexMesh::MeshData`, shared behind an [`Arc`] so
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
    for tri in &triangles {
        let v0 = vertices[tri[0] as usize];
        let v1 = vertices[tri[1] as usize];
        let v2 = vertices[tri[2] as usize];
        let normal = (v1 - v0).cross(&(v2 - v0));
        normals.push(normal.try_normalize(0.0).unwrap_or_else(Vector3::zeros));
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
        mesh_center,
        mesh_radius_bounding,
        box_offset,
        box_size,
        bounding_cylinder_radius,
        bounding_cylinder_length: cyl_length,
    })
}

/// A convex mesh body: the convex hull of a [`crate::shapes::Mesh`], posed,
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
            if tmp.abs() <= ZERO {
                continue;
            }
            let t = -(normal.dot(&orig) + offset) / tmp;
            if t <= 0.0 {
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
    /// [`Shape::OcTree`], which have no `bodies::` counterpart upstream
    /// (upstream's `createBodyFromShape` returns `nullptr` for these, after
    /// logging an error).
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
    use approx::assert_relative_eq;

    /// A tiny deterministic xorshift64 PRNG, used only by this module's own
    /// tests as the `uniform(lo, hi)` sampler [`Sphere::sample_point_inside`]
    /// et al. take — see the module docs, deviation 5, for why no `rand`
    /// dependency was added and upstream's exact
    /// `RandomNumberGenerator`/iteration-count sequences are not
    /// reproduced.
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
            assert_relative_eq!(hits[0], expected, epsilon = 1e-6);
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
        assert_relative_eq!(hits[0], Vector3::new(1.1, 0.0, 0.0), epsilon = 1e-6);
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
        assert_relative_eq!(hits[0], Vector3::new(1.6, 0.0, 0.0), epsilon = 1e-6);
        let hits = sphere.ray_intersections(&origin, &Vector3::new(-1.0, 0.0, 0.0), Some(2));
        assert_eq!(hits.len(), 1);
        assert_relative_eq!(hits[0], Vector3::new(-0.6, 0.0, 0.0), epsilon = 1e-6);
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
        assert_relative_eq!(norms[0], -1.0, epsilon = 1e-6);
        assert_relative_eq!(norms[1], 1.0, epsilon = 1e-6);
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
        assert_relative_eq!(hits[0], Vector3::new(-1.0, 0.0, 0.0), epsilon = 1e-6);
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
            assert_relative_eq!(hits[0], expected, epsilon = 1e-6);
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
        assert_relative_eq!(xs[0], -2.0, epsilon = 1e-6);
        assert_relative_eq!(xs[1], 2.0, epsilon = 1e-6);
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
        assert_relative_eq!(hits[0], Vector3::new(-2.0, 0.0, 0.0), epsilon = 1e-6);
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
        assert_relative_eq!(xs[0], -0.475, epsilon = 1e-4);
        assert_relative_eq!(xs[1], 0.475, epsilon = 1e-4);
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
        assert_relative_eq!(xs[0], -0.5, epsilon = 1e-6);
        assert_relative_eq!(xs[1], 0.5, epsilon = 1e-6);
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
            assert_relative_eq!(hits[0], expected, epsilon = 1e-6);
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
        assert_relative_eq!(xs[0], -1.0, epsilon = 1e-6);
        assert_relative_eq!(xs[1], 1.0, epsilon = 1e-6);
        for h in &hits {
            assert_relative_eq!(h.y, 1.0, epsilon = 1e-6);
            assert_relative_eq!(h.z, 1.0, epsilon = 1e-6);
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
        assert_relative_eq!(ys[0], -1.0, epsilon = 1e-6);
        assert_relative_eq!(ys[1], 1.0, epsilon = 1e-6);
        for h in &hits {
            assert_relative_eq!(h.z, 1.0, epsilon = 1e-6);
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
        assert_relative_eq!(hits[0], Vector3::new(-1.0, -1.0, -1.0), epsilon = 1e-6);
        assert_relative_eq!(hits[1], Vector3::new(1.0, 1.0, 1.0), epsilon = 1e-6);
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
        assert_relative_eq!(xs[0], -0.475, epsilon = 1e-4);
        assert_relative_eq!(xs[1], 0.475, epsilon = 1e-4);
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
            assert_relative_eq!(hits[0], expected, epsilon = 1e-6);
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
    #[test]
    fn convex_mesh_zero_vertex_is_an_error() {
        let mesh = ShapeMesh::default();
        assert!(ConvexMesh::new(&mesh).is_err());
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
        assert_relative_eq!(
            sphere.compute_volume(),
            4.0 / 3.0 * std::f64::consts::PI * 8.0,
            epsilon = 1e-9
        );
    }

    #[test]
    fn cylinder_volume_matches_pi_r_squared_h() {
        let cylinder = Cylinder::new(2.0, 3.0).unwrap();
        assert_relative_eq!(
            cylinder.compute_volume(),
            std::f64::consts::PI * 4.0 * 3.0,
            epsilon = 1e-9
        );
    }

    /// Boundary: zero-length cylinder has zero volume, not NaN or a
    /// negative value.
    #[test]
    fn degenerate_cylinder_zero_length_volume_is_zero() {
        let cylinder = Cylinder::new(1.0, 0.0).unwrap();
        assert_relative_eq!(cylinder.compute_volume(), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn cuboid_volume_matches_l_w_h() {
        let cuboid = Cuboid::new(2.0, 3.0, 4.0).unwrap();
        assert_relative_eq!(cuboid.compute_volume(), 24.0, epsilon = 1e-9);
    }

    #[test]
    fn convex_mesh_volume_of_box_matches_l_w_h() {
        let mesh = ConvexMesh::new(&box_mesh(2.0, 3.0, 4.0)).unwrap();
        assert_relative_eq!(mesh.compute_volume(), 24.0, epsilon = 1e-6);
    }

    // --- negative/zero dimensions and padding inversion: invariant
    // --- boundaries ---

    #[test]
    fn sphere_negative_radius_is_an_error() {
        assert!(Sphere::new(-1.0).is_err());
    }

    #[test]
    fn sphere_zero_radius_is_valid() {
        assert!(Sphere::new(0.0).is_ok());
    }

    #[test]
    fn cylinder_negative_radius_is_an_error() {
        assert!(Cylinder::new(-1.0, 1.0).is_err());
    }

    #[test]
    fn cylinder_negative_length_is_an_error() {
        assert!(Cylinder::new(1.0, -1.0).is_err());
    }

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
    #[test]
    fn sphere_padding_inversion_is_rejected_and_state_preserved() {
        let mut sphere = Sphere::new(1.0).unwrap();
        assert!(sphere.set_padding(-2.0).is_err());
        assert_relative_eq!(sphere.padding(), 0.0, epsilon = 1e-12);
        assert!(sphere.contains_point(&Vector3::new(1.0, 0.0, 0.0)));
    }

    #[test]
    fn cylinder_padding_inversion_is_rejected_and_state_preserved() {
        let mut cylinder = Cylinder::new(1.0, 1.0).unwrap();
        assert!(cylinder.set_padding(-2.0).is_err());
        assert_relative_eq!(cylinder.padding(), 0.0, epsilon = 1e-12);
        assert!(cylinder.contains_point(&Vector3::new(1.0, 0.0, 0.0)));
    }

    #[test]
    fn cuboid_padding_inversion_is_rejected_and_state_preserved() {
        let mut cuboid = Cuboid::new(1.0, 1.0, 1.0).unwrap();
        assert!(cuboid.set_padding(-2.0).is_err());
        assert_relative_eq!(cuboid.padding(), 0.0, epsilon = 1e-12);
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
        assert_relative_eq!(merged.min(), Vector3::new(-1.0, -1.0, -1.0), epsilon = 1e-4);
        assert_relative_eq!(merged.max(), Vector3::new(1.0, 1.0, 1.0), epsilon = 1e-4);
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
        assert_relative_eq!(b1.extents(), Vector3::new(0.1, 0.1, 0.1), epsilon = 1e-12);
        assert_relative_eq!(
            b1.pose().translation.vector,
            Vector3::new(-0.6, -0.6, -0.6),
            epsilon = 1e-12
        );
        assert_relative_eq!(
            b1.pose().rotation.to_rotation_matrix().matrix(),
            pose.rotation.to_rotation_matrix().matrix(),
            epsilon = 1e-12
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
        assert_relative_eq!(b1.extents(), Vector3::new(1.0, 1.0, 1.0), epsilon = 1e-12);
        assert_relative_eq!(
            b1.pose().translation.vector,
            Vector3::new(-0.5, -0.5, -0.5),
            epsilon = 1e-12
        );
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

        assert_relative_eq!(b2.extents(), Vector3::new(1.0, 1.0, 1.0), epsilon = 1e-12);
        assert_relative_eq!(
            b2.pose().translation.vector,
            Vector3::new(-0.5, -0.5, -0.5),
            epsilon = 1e-12
        );
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
        assert_relative_eq!(merged.center.x, -0.05, epsilon = 1e-5);
        assert_relative_eq!(merged.radius, 6.05, epsilon = 1e-12);
    }

    // --- Body::from_shape ---

    #[test]
    fn from_shape_builds_matching_body_variant() {
        assert!(matches!(
            Body::from_shape(&Shape::Sphere(crate::shapes::Sphere { radius: 1.0 })).unwrap(),
            Some(Body::Sphere(_))
        ));
        assert!(matches!(
            Body::from_shape(&Shape::Cylinder(crate::shapes::Cylinder {
                radius: 1.0,
                length: 1.0
            }))
            .unwrap(),
            Some(Body::Cylinder(_))
        ));
        assert!(matches!(
            Body::from_shape(&Shape::Cuboid(crate::shapes::Cuboid {
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

    #[test]
    fn from_shape_returns_none_for_cone_plane_octree() {
        assert!(
            Body::from_shape(&Shape::Cone(crate::shapes::Cone {
                radius: 1.0,
                length: 1.0
            }))
            .unwrap()
            .is_none()
        );
        assert!(
            Body::from_shape(&Shape::Plane(crate::shapes::Plane {
                a: 0.0,
                b: 0.0,
                c: 1.0,
                d: 0.0
            }))
            .unwrap()
            .is_none()
        );
        assert!(
            Body::from_shape(&Shape::OcTree(crate::shapes::OcTree))
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
