// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: Zlib

//! The shape set and transforms `tools/bullet-epa-reference/probe.cpp` builds,
//! spelled once for every module that asserts against that probe's output.
//!
//! Three modules pin themselves to the same C++ run -- `epa`, `pen_depth` and
//! `gjk` -- and a fixture row only means anything if the Rust side fed the
//! solver the identical shapes. Divergence between per-module copies would not
//! fail as a mismatch; it would fail as a bit difference somewhere in the
//! result, blamed on the algorithm. One definition, shared, so a change to
//! `probe.cpp`'s setup has exactly one place to land.

use crate::linear_math::{Matrix3, Scalar, Transform, Vec3};
use crate::shapes::{
    BoxShape, ConeShapeZ, ConvexHullShape, ConvexShape, CylinderShapeZ, SphereShape,
    TriangleShapeEx,
};

/// `const btTransform id = at(0, 0, 0);`
pub const IDENTITY: Transform = Transform::new(Matrix3::identity(), Vec3::zero());

/// `at(x, y, z)` -- a pure translation.
pub fn at(x: Scalar, y: Scalar, z: Scalar) -> Transform {
    Transform::new(Matrix3::identity(), Vec3::new(x, y, z))
}

/// `rot60_at(x, y, z)` -- 60 degrees about `(1,1,1)/sqrt(3)`, translated.
///
/// Every entry is the quotient of two small integers, so this is the same
/// basis `probe.cpp` builds without either side going through a quaternion or
/// a trigonometric function whose last bit could differ.
pub fn rot60_at(x: Scalar, y: Scalar, z: Scalar) -> Transform {
    let p = 2.0 / 3.0;
    let m = -1.0 / 3.0;
    Transform::new(
        Matrix3::from_rows(Vec3::new(p, m, p), Vec3::new(p, p, m), Vec3::new(m, p, p)),
        Vec3::new(x, y, z),
    )
}

/// The eight shapes `probe.cpp` builds, in its order:
/// `(unit_box, flat_box, margin_box, sphere, small_sphere, cyl, cone, hull)`.
///
/// `margin_box`, `sphere` and `small_sphere` keep their default margins on
/// purpose -- the 0.04 box margin and the sphere's radius-as-margin are what
/// the margin-carrying rows exercise.
///
/// MoveIt's hull order -- every vertex added first, `setMargin(0)` after -- is
/// reproduced because `addPoint` order decides which of several equally
/// extreme vertices `maxDot` returns.
pub fn probe_shapes() -> (
    BoxShape,
    BoxShape,
    BoxShape,
    SphereShape,
    SphereShape,
    CylinderShapeZ,
    ConeShapeZ,
    ConvexHullShape,
) {
    let mut unit_box = BoxShape::new(Vec3::new(0.5, 0.5, 0.5));
    unit_box.set_margin(0.0);
    let mut flat_box = BoxShape::new(Vec3::new(0.4, 0.7, 0.25));
    flat_box.set_margin(0.0);
    let margin_box = BoxShape::new(Vec3::new(0.5, 0.5, 0.5));
    let sphere = SphereShape::new(0.5);
    let small_sphere = SphereShape::new(0.3);
    let mut cyl = CylinderShapeZ::new(Vec3::new(0.3, 0.3, 0.5));
    cyl.set_margin(0.0);
    let mut cone = ConeShapeZ::new(0.25, 0.8);
    cone.set_margin(0.0);

    let mut hull = ConvexHullShape::new();
    for p in [
        Vec3::new(0.3, 0.2, 0.1),
        Vec3::new(-0.3, 0.2, 0.1),
        Vec3::new(0.3, -0.2, 0.1),
        Vec3::new(-0.3, -0.2, 0.1),
        Vec3::new(0.3, 0.2, -0.1),
        Vec3::new(-0.3, 0.2, -0.1),
        Vec3::new(0.3, -0.2, -0.1),
        Vec3::new(-0.3, -0.2, -0.1),
    ] {
        hull.add_point(p);
    }
    hull.set_margin(0.0);

    (
        unit_box,
        flat_box,
        margin_box,
        sphere,
        small_sphere,
        cyl,
        cone,
        hull,
    )
}

/// The two triangles `probe.cpp` builds, in its order: `(tri, tri_margin)`.
///
/// The same three corners twice, differing only in margin. `tri` is what
/// `createShapePrimitive` produces -- `setMargin(BULLET_MARGIN)`, which is zero
/// -- and `tri_margin` keeps the constructed 0.04, which is the only setting
/// under which the shape's `getAabb` and the `getAabbSlow` it overrides give
/// different answers.
///
/// Separate from [`probe_shapes`] rather than a ninth and tenth element of its
/// tuple: nothing that consumes that tuple wants a triangle, and widening it
/// would edit every destructuring in this crate to add two holes.
pub fn probe_triangles() -> (TriangleShapeEx, TriangleShapeEx) {
    let corners = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
    ];
    let mut tri = TriangleShapeEx::new(corners[0], corners[1], corners[2]);
    tri.set_margin(0.0);
    let tri_margin = TriangleShapeEx::new(corners[0], corners[1], corners[2]);
    (tri, tri_margin)
}

/// Bit-exact comparison against a probe field, with `+0.0 == -0.0` the one
/// admitted difference: the sign of a zero survives `printf` but says nothing
/// about the arithmetic, and `f32::to_bits` alone would fail on it.
///
/// Differences are accumulated rather than asserted, so one run reports every
/// field that moved instead of the first.
pub fn diff(into: &mut Vec<String>, name: &str, field: &str, got: Scalar, want: Scalar) {
    if got.to_bits() != want.to_bits() && got != want {
        into.push(format!(
            "{name}.{field}: port {got:e} ({:#010x}), bullet {want:e} ({:#010x})",
            got.to_bits(),
            want.to_bits()
        ));
    }
}

/// [`diff`] over all three components of a vector field.
pub fn diff_vec3(into: &mut Vec<String>, name: &str, field: &str, got: Vec3, want: Vec3) {
    diff(into, name, &format!("{field}.x"), got.x, want.x);
    diff(into, name, &format!("{field}.y"), got.y, want.y);
    diff(into, name, &format!("{field}.z"), got.z, want.z);
}

/// Splits one `|`-separated probe row out of a `BULLET_REFERENCE` block and
/// checks its arity, so a row that gained or lost a field fails as a row-shape
/// error rather than as a silently shifted value.
pub fn row<'a>(reference: &'a str, name: &str, fields: usize) -> Vec<&'a str> {
    let line = reference
        .lines()
        .find(|l| l.split('|').next() == Some(name))
        .unwrap_or_else(|| panic!("{name}: no such row in BULLET_REFERENCE"));
    let f: Vec<&str> = line.split('|').collect();
    assert_eq!(
        f.len(),
        fields,
        "{name}: {} fields, expected {fields}",
        f.len()
    );
    f
}
