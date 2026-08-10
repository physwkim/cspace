// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Regression coverage for the NaN-blind dimension validation in
//! `shapes.rs`'s `Sphere`/`Cylinder`/`Cone`/`Cuboid` constructors, and for
//! `Cone::bounding_sphere`'s own NaN-blind branch and division -- see
//! `Cone::bounding_sphere`'s doc comment for the full mechanism.
//!
//! `Cone::new`/`Cylinder::new`/etc. validated non-negativity with a bare `x
//! < 0.0` check (matching upstream `geometric_shapes` exactly). Every NaN
//! comparison is `false`, so `NaN < 0.0` reads as "not negative" and the
//! constructor silently accepted a NaN dimension instead of rejecting it --
//! confirmed reachable from real ROS wire messages via
//! `ros/cspace-ros/src/constraints/position.rs`'s
//! `TryFrom<SolidPrimitiveMsg>`, which feeds a `PositionConstraint`
//! message's raw, externally-supplied `dimensions` floats straight into
//! these constructors.
//!
//! Before this fix, `Sphere::new(f64::NAN)` and friends returned `Ok` (a
//! changed, directly observable returned answer: `.is_err()` flips from
//! `false` to `true`). Separately, because `radius`/`length` are public
//! fields, a `Cone` can carry a non-finite or invalid dimension without
//! going through `Cone::new` at all -- `Cone::bounding_sphere` is fixed to
//! defend itself independently, and the tests below construct a `Cone`
//! directly (not through `Cone::new`) to demonstrate that.

use cspace_core::geometry::{Cone, Cuboid, Cylinder, Sphere};

#[test]
fn sphere_new_rejects_a_nan_radius_the_same_way_it_rejects_a_negative_one() {
    assert!(Sphere::new(f64::NAN).is_err());
    assert!(Sphere::new(-1.0).is_err(), "sanity: still rejects negative");
    assert!(
        Sphere::new(1.0).is_ok(),
        "sanity: still accepts valid input"
    );
}

#[test]
fn cylinder_new_rejects_a_nan_radius_or_length() {
    assert!(Cylinder::new(f64::NAN, 1.0).is_err());
    assert!(Cylinder::new(1.0, f64::NAN).is_err());
    assert!(Cylinder::new(1.0, 1.0).is_ok(), "sanity");
}

#[test]
fn cone_new_rejects_a_nan_radius_or_length() {
    assert!(Cone::new(f64::NAN, 1.0).is_err());
    assert!(Cone::new(1.0, f64::NAN).is_err());
    assert!(Cone::new(1.0, 1.0).is_ok(), "sanity");
}

#[test]
fn cuboid_new_rejects_a_nan_dimension_on_any_axis() {
    assert!(Cuboid::new(f64::NAN, 1.0, 1.0).is_err());
    assert!(Cuboid::new(1.0, f64::NAN, 1.0).is_err());
    assert!(Cuboid::new(1.0, 1.0, f64::NAN).is_err());
    assert!(Cuboid::new(1.0, 1.0, 1.0).is_ok(), "sanity");
}

#[test]
fn cone_scale_and_padd_axes_rejects_a_nan_result_the_same_way_it_rejects_a_negative_one() {
    let mut c = Cone::new(1.0, 1.0).unwrap();
    // A NaN scale factor (e.g. from a corrupted padding config) makes the
    // scaled radius NaN; `NaN < 0.0` must not let that through silently.
    assert!(c.scale_and_padd_axes(f64::NAN, 1.0, 0.0, 0.0).is_err());
}

/// `radius`/`length` are public fields, so a `Cone` can carry a NaN without
/// ever going through `Cone::new` -- direct field construction, exactly as
/// any external caller of this crate could write. Before the fix,
/// `self.length > self.radius` read `false` for the NaN radius (every NaN
/// comparison does), took the "short cone" branch, and returned that NaN
/// straight through as `BoundingSphere.radius` -- a corrupted answer, not
/// merely a NaN observed mid-computation. After the fix it returns the
/// documented zero-radius fallback instead.
#[test]
fn cone_bounding_sphere_does_not_leak_a_nan_radius_into_the_returned_sphere() {
    let cone = Cone {
        radius: f64::NAN,
        length: 5.0,
    };

    let bs = cone.bounding_sphere();

    assert!(
        bs.radius.is_finite() && bs.center.iter().all(|c| c.is_finite()),
        "a NaN Cone dimension must not reach the returned BoundingSphere: got {bs:?}"
    );
}

/// Same defect, `length` carrying the NaN instead -- a different field read
/// directly by the `else` branch's `center.z` computation.
#[test]
fn cone_bounding_sphere_does_not_leak_a_nan_length_into_the_returned_sphere() {
    let cone = Cone {
        radius: 5.0,
        length: f64::NAN,
    };

    let bs = cone.bounding_sphere();

    assert!(
        bs.radius.is_finite() && bs.center.iter().all(|c| c.is_finite()),
        "a NaN Cone dimension must not reach the returned BoundingSphere: got {bs:?}"
    );
}

/// A `Cone` built by bypassing `Cone::new` with a negative radius and an
/// exactly-zero length can drive `bounding_sphere`'s division denominator
/// to a literal `0.0` (not NaN): `length (0.0) > radius (-1.0)` is
/// genuinely true, so before this fix the divide branch ran with
/// `self.length == 0.0`, producing `radius * radius / 0.0 = +inf` and an
/// infinite returned `BoundingSphere`. The `length > 0.0` clause this fix
/// adds to the branch condition keeps that degenerate case in the
/// (division-free) `else` branch instead.
#[test]
fn cone_bounding_sphere_does_not_divide_by_a_bypassed_zero_length() {
    let cone = Cone {
        radius: -1.0,
        length: 0.0,
    };

    let bs = cone.bounding_sphere();

    assert!(
        bs.radius.is_finite() && bs.center.iter().all(|c| c.is_finite()),
        "an exactly-zero length must not reach the division: got {bs:?}"
    );
}

/// The fix must not change behavior for a `Cone` built the honest way:
/// `Cone::new`'s own validation already guarantees `length > radius` (the
/// `if` branch) implies `length > 0`, so the tall-cone geometry must be
/// unchanged from before this fix.
#[test]
fn cone_bounding_sphere_tall_cone_geometry_is_unchanged() {
    let tall = Cone::new(1.0, 10.0).unwrap();
    let bs = tall.bounding_sphere();
    assert!(bs.radius > 0.0);
    assert_ne!(bs.center.z, 0.0);

    let short = Cone::new(10.0, 1.0).unwrap();
    let bs = short.bounding_sphere();
    assert_eq!(bs.radius, 10.0);
    assert_eq!(bs.center.z, -0.5);
}
