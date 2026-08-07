// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No upstream file: an investigation probe for PORTING-PLAN.md §288.5's
// positive-gap band (`cylinder x box` 600/3,067, `box x box` 253/1,515).
// Does NOT touch `crates/moveit-collision/src/parry.rs` -- this calls
// `parry3d_f64::query::contact` directly, bypassing this crate's own
// `accumulate_collision`, to isolate whether the positive-gap `Some` is a
// property of the vendored `parry3d-f64` dependency itself.

//! Binary-searches the exact gap at which `parry3d_f64::query::contact(_,
//! 0.0)` flips from `Some` (a "contact" the caller reads as collision) to
//! `None`, for a `Cylinder` vs `Cuboid` pair and several `Cuboid` vs `Cuboid`
//! configurations, at multiple shape scales and orientations.
//!
//! `crates/moveit-collision/src/parry.rs`'s own comment on
//! `accumulate_collision` states the boundary was measured once, by hand, at
//! `~5e-8 m` for prbt's base cylinder against a `4x4x0.1` box. This probe
//! re-derives that number by binary search (not by hand-picked samples).
//!
//! # Source-level claim this probe is checking
//!
//! `parry3d-f64-0.30.0`'s `query::gjk::closest_points` (`src/query/gjk/gjk.rs:415`)
//! terminates on `max_bound - min_bound <= _eps_rel * max_bound`, where
//! `_eps_rel = sqrt(10.0 * f64::EPSILON)` (`gjk.rs:141-143`, `gjk.rs:366-367`)
//! -- a RELATIVE tolerance tied to `max_bound`, an intermediate distance-bound
//! estimate. This part is confirmed directly from source and is not what the
//! scale sweep below tests.
//!
//! `Cuboid`-`Cuboid` is included because `default_query_dispatcher.rs`'s
//! `fn contact` (the function `query::contact` calls) has its
//! `contact_cuboid_cuboid` closed-form branch commented out
//! (`default_query_dispatcher.rs:317-320`) -- `Cuboid`-`Cuboid` falls through
//! to the exact same generic `contact_support_map_support_map` /
//! `gjk::closest_points` path as `Cylinder`-`Cuboid`. That is the structural
//! reason §288.5 measured the SAME defect shape on both pairs (`cylinder x
//! box` 600/3,067, `box x box` 253/1,515): one shared mechanism, not two.
//!
//! # What the scale sweep below actually shows (corrects an earlier revision
//! # of this file)
//!
//! An earlier revision of this probe asserted "the boundary must move with
//! shape scale, not sit at one fixed number" -- reasoning that `max_bound`
//! being an intermediate value tied to shape/query scale implies
//! `eps_rel * max_bound` should scale linearly with a uniform rescaling of
//! the whole configuration. Measuring it refutes that prediction: scaling
//! the prbt cylinder-vs-box configuration by x100 or x0.01 does NOT move the
//! boundary to ~100x or ~1/100x of the unscaled 3.03e-8 m value -- both
//! collapse to ~2.2e-15 m (the machine-epsilon noise floor), matching the
//! degenerate axis-aligned `Cuboid`-`Cuboid` case rather than scaling.
//!
//! The scale sweep further down (`scale` in
//! `{1, 1.5, 2, 3, 5, 10, 20, 50, 100, 0.5, 0.2, 0.1, 0.05, 0.01}`) isolates
//! why: scale factors that are exact powers of two (`1`, `2`, `0.5`) --
//! which only shift the f64 exponent and touch no mantissa bit, so they
//! introduce zero new rounding relative to the unscaled case -- reproduce
//! the *exact* `boundary / base` ratio at every one of those three scales.
//! Every non-power-of-two factor (`1.5`, `3`, `5`, `10`, `20`, `50`, `100`,
//! `0.2`, `0.1`, `0.05`, `0.01`) -- each of which perturbs the decimal shape
//! parameters (`0.09275`, `0.13`, ...) by a different, new sub-ULP rounding
//! pattern when multiplied -- collapses to the ~2.2e-15 noise floor instead.
//!
//! So the presence/magnitude of a measurable band is NOT a smooth function
//! of shape/query scale (that specific claim is refuted), and it is also not
//! reducible to contact-feature type alone: the rotated `Cuboid`-`Cuboid`
//! sweep (vertex-vs-face, non-power-of-two axis-angle parameters) still
//! lands at ~2.1e-8 m, the same order of magnitude as the unscaled
//! `Cylinder`-`Cuboid` case, despite sharing the "collapses for non-power-of-
//! two parameters" pattern's precondition. What determines whether a given
//! (shapes, pose) tuple's floating-point evaluation takes the GJK iteration
//! path that surfaces the ~1e-8 `eps_rel`-driven band, versus a path that
//! converges to near machine-epsilon precision instead, is NOT characterized
//! by this probe -- only that it is not simply "feature type" and not simply
//! "shape scale". This is additional (not weaker) evidence for this task's
//! deliverable (2): the phenomenon is too rounding-path-sensitive for any
//! caller-side geometric renormalization to characterize or route around.
//!
//! Run: `cargo run -p moveit-collision --example probe_gjk_positive_gap_boundary --release`

use parry3d_f64::math::Pose;
use parry3d_f64::query;
use parry3d_f64::shape::{Cuboid, Cylinder};

/// Binary-searches the boundary gap in `[lo, hi]` at which `query::contact`
/// with `prediction = 0.0` flips from `Some` to `None`, placing shape `b`
/// directly above shape `a` by `gap` along whichever axis the caller's
/// `contact_at` closure offsets on (both shapes' relevant half-extent along
/// that axis already subtracted by the caller, so `gap` is the true
/// clear-air separation).
fn find_boundary(mut lo: f64, mut hi: f64, contact_at: impl Fn(f64) -> bool) -> f64 {
    assert!(contact_at(lo), "lo={lo} must still be Some (a control)");
    // `hi` is a starting guess, not a hard requirement: double it until the
    // query actually reports `None`, so a boundary the caller under-guessed
    // is still found rather than panicking on a bad initial bracket.
    while contact_at(hi) {
        hi *= 2.0;
        assert!(
            hi.is_finite() && hi < 1.0,
            "boundary did not resolve below 1.0 m"
        );
    }
    // 60 bisections resolves an f64 mantissa's worth of precision within
    // any [lo, hi] this probe calls with.
    for _ in 0..60 {
        let mid = 0.5 * (lo + hi);
        if contact_at(mid) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    lo
}

fn eps_rel() -> f64 {
    (10.0 * f64::EPSILON).sqrt()
}

fn main() {
    println!("f64::EPSILON              = {:e}", f64::EPSILON);
    println!(
        "sqrt(10 * f64::EPSILON)   = {:e}  (parry's gjk::eps_rel)",
        eps_rel()
    );
    println!();

    // --- Cylinder x Cuboid, at the exact scale the parry.rs comment names:
    // prbt's base cylinder (radius 0.09275, length 0.13 -- fixtures/prbt.urdf's
    // first <cylinder>) against a 4x4x0.1 box. `parry3d_f64::shape::Cylinder`
    // is canonically aligned along Y, not Z (its own doc comment: "Creates a
    // new cylinder aligned with the Y axis"), which is exactly why
    // `crates/moveit-collision/src/parry.rs`'s `convert_shape` applies
    // `axis_fix()` when converting a URDF (Z-axis) cylinder into parry's
    // (Y-axis) `Cylinder`. This probe leaves the cylinder unrotated at its
    // own local frame and offsets the box along Y instead, so `gap` is the
    // true clear-air separation between the cylinder's flat cap and the
    // box's face rather than a distance to its curved lateral surface. The
    // box's own thin half-extent (0.05) must move to the Y slot too -- it
    // names which face is thin, not which axis index happens to hold it. ---
    {
        let cyl = Cylinder::new(0.13 * 0.5, 0.09275);
        let cuboid = Cuboid::new(parry3d_f64::math::Vector::new(2.0, 0.05, 2.0));
        let cyl_half_len = 0.13 * 0.5;
        let box_half_y = 0.05;
        let pos_cyl = Pose::IDENTITY;
        let contact_at = |gap: f64| {
            // box top face at y=0; cylinder bottom cap placed `gap` above it.
            let pos_box_y = -(box_half_y + cyl_half_len + gap);
            let pos_box = Pose::translation(0.0, pos_box_y, 0.0);
            query::contact(&pos_cyl, &cyl, &pos_box, &cuboid, 0.0)
                .expect("supported pair")
                .is_some()
        };
        let b = find_boundary(0.0, 1e-4, contact_at);
        println!(
            "cylinder(r=0.09275,l=0.13) x box(4x4x0.1): boundary gap = {:e} m  (ratio to sqrt(10*EPS) = {:.3})",
            b,
            b / eps_rel()
        );
        // Reproduce the exact two hand-picked samples the parry.rs comment cites.
        println!(
            "  spot check: gap=3e-8 -> {}   gap=1e-7 -> {}   (comment: Some, None)",
            if contact_at(3e-8) { "Some" } else { "None" },
            if contact_at(1e-7) { "Some" } else { "None" }
        );
    }

    println!();

    // --- Scale sweep over the SAME cylinder-vs-box configuration: does the
    // boundary scale linearly with a uniform rescaling of shape+query, as
    // `eps_rel * max_bound` (max_bound being scale-dependent) would predict?
    // Power-of-two scales (1, 2, 0.5) introduce no new mantissa rounding when
    // multiplied into the shape parameters -- only an exponent shift -- so
    // they isolate "scale" from "incidental floating-point rounding of these
    // specific decimal parameters". Every other scale mixes both effects. ---
    {
        println!("scale sweep (same cylinder x box configuration, uniformly rescaled):");
        for &scale in &[
            1.0, 1.5, 2.0, 3.0, 5.0, 10.0, 20.0, 50.0, 100.0, 0.5, 0.2, 0.1, 0.05, 0.01,
        ] {
            let cyl = Cylinder::new(0.13 * 0.5 * scale, 0.09275 * scale);
            let cuboid = Cuboid::new(parry3d_f64::math::Vector::new(
                2.0 * scale,
                0.05 * scale,
                2.0 * scale,
            ));
            let cyl_half_len = 0.13 * 0.5 * scale;
            let box_half_y = 0.05 * scale;
            let base = box_half_y + cyl_half_len;
            let pos_cyl = Pose::IDENTITY;
            let contact_at = |gap: f64| {
                let pos_box_y = -(base + gap);
                let pos_box = Pose::translation(0.0, pos_box_y, 0.0);
                query::contact(&pos_cyl, &cyl, &pos_box, &cuboid, 0.0)
                    .expect("supported pair")
                    .is_some()
            };
            // `1e-9` as the doubling seed is small enough that, for the
            // scales that collapse, the loop never needs to double at all
            // (contact_at(1e-9) is already `false`) -- it is not load-bearing
            // for the scales that don't collapse either, since `find_boundary`
            // doubles its bracket until it brackets the true crossing.
            let b = find_boundary(0.0, 1e-9, contact_at);
            let is_pow2 = scale.log2().fract() == 0.0;
            println!(
                "  scale={scale:>6}  {}  boundary={b:e}  boundary/base={:e}",
                if is_pow2 {
                    "(power of 2)"
                } else {
                    "            "
                },
                b / base
            );
        }
        println!(
            "  -> power-of-2 scales (1, 2, 0.5) reproduce the identical boundary/base ratio;\n     \
             every other scale collapses to the ~2.2e-15 machine-epsilon noise floor.\n     \
             The boundary is NOT a smooth function of shape/query scale; it is sensitive\n     \
             to incidental sub-ULP rounding of the specific decimal shape parameters."
        );
    }

    println!();

    // --- Cuboid x Cuboid, unit cube vs unit cube (the pair whose closed
    // form is commented out of default_query_dispatcher.rs's `fn contact`). ---
    {
        let a = Cuboid::new(parry3d_f64::math::Vector::new(0.5, 0.5, 0.5));
        let b_shape = Cuboid::new(parry3d_f64::math::Vector::new(0.5, 0.5, 0.5));
        let pos_a = Pose::IDENTITY;
        let contact_at = |gap: f64| {
            let pos_b = Pose::translation(0.0, 0.0, -(1.0 + gap));
            query::contact(&pos_a, &a, &pos_b, &b_shape, 0.0)
                .expect("supported pair")
                .is_some()
        };
        let b = find_boundary(0.0, 1e-4, contact_at);
        println!(
            "cuboid(1x1x1) x cuboid(1x1x1): boundary gap = {:e} m  (ratio to sqrt(10*EPS) = {:.3})",
            b,
            b / eps_rel()
        );
    }

    // --- Cuboid x Cuboid at prbt_pg70's own scale (box x box control pair's
    // band was 2.371e-9 to 7.844e-9 in §288.5's prbt_pg70 row; the exact
    // shapes there are not reproduced here, only representative small boxes,
    // to see whether the boundary lands in the same order of magnitude for
    // small shapes as it did for the unit cube above). ---
    {
        let a = Cuboid::new(parry3d_f64::math::Vector::new(0.05, 0.05, 0.05));
        let b_shape = Cuboid::new(parry3d_f64::math::Vector::new(0.05, 0.05, 0.05));
        let pos_a = Pose::IDENTITY;
        let contact_at = |gap: f64| {
            let pos_b = Pose::translation(0.0, 0.0, -(0.1 + gap));
            query::contact(&pos_a, &a, &pos_b, &b_shape, 0.0)
                .expect("supported pair")
                .is_some()
        };
        let b = find_boundary(0.0, 1e-6, contact_at);
        println!(
            "cuboid(0.1x0.1x0.1) x cuboid(0.1x0.1x0.1): boundary gap = {:e} m",
            b
        );
    }

    println!();

    // --- Cuboid x Cuboid, GENERIC orientation: the two sweeps above are
    // face-to-face and axis-aligned, a degenerate configuration real corpus
    // pairs essentially never hit -- their result (~2.3e-15 m, i.e. no
    // measurable positive-gap band) must not be read as "box x box has no
    // positive-gap band" without checking a generic pose too. Here `b`'s
    // corner (its vertex support function has no continuous/curved degree of
    // freedom like Cylinder's cap, but a rotated cuboid's closest feature to
    // an axis-aligned face is a single vertex, not the flat-vs-flat case
    // above) is rotated by an arbitrary non-axis-aligned axis-angle so the
    // contact is vertex-vs-face, then translated so that vertex sits exactly
    // `gap` above `a`'s top face. `min_corner_z` is that vertex's z-offset
    // under rotation alone (translation=0), so translating `b`'s center to
    // `a_half + gap - min_corner_z` places the vertex at exactly `a_half + gap`. ---
    {
        let axisangle = parry3d_f64::math::Vector::new(0.4, 0.5, 0.3);
        let a_half = 2.0;
        let b_half = 0.5;
        let a = Cuboid::new(parry3d_f64::math::Vector::new(a_half, a_half, a_half));
        let b_shape = Cuboid::new(parry3d_f64::math::Vector::new(b_half, b_half, b_half));
        let pos_a = Pose::IDENTITY;
        let rot_only = Pose::rotation(axisangle);
        let corners: Vec<f64> = [-1.0, 1.0]
            .iter()
            .flat_map(|&sx| {
                [-1.0, 1.0]
                    .iter()
                    .flat_map(move |&sy| [-1.0, 1.0].iter().map(move |&sz| (sx, sy, sz)))
            })
            .map(|(sx, sy, sz)| {
                rot_only
                    .transform_point(parry3d_f64::math::Vector::new(
                        sx * b_half,
                        sy * b_half,
                        sz * b_half,
                    ))
                    .z
            })
            .collect();
        let min_corner_z = corners.iter().cloned().fold(f64::INFINITY, f64::min);
        // Sanity check this is a genuine vertex contact, not degenerately
        // close to a face/edge contact (which would need a different
        // horizontal-footprint argument to guarantee `a`'s top face, not one
        // of its edges, is the true closest feature).
        let n_near_min = corners
            .iter()
            .filter(|&&z| (z - min_corner_z).abs() < 1e-9)
            .count();
        assert_eq!(
            n_near_min, 1,
            "axis-angle produced a degenerate (non-vertex) contact"
        );
        let contact_at = |gap: f64| {
            let z_center = a_half + gap - min_corner_z;
            let pos_b = Pose::new(
                parry3d_f64::math::Vector::new(0.0, 0.0, z_center),
                axisangle,
            );
            query::contact(&pos_a, &a, &pos_b, &b_shape, 0.0)
                .expect("supported pair")
                .is_some()
        };
        let b = find_boundary(0.0, 1e-4, contact_at);
        println!(
            "cuboid(4x4x4) x cuboid(1x1x1) ROTATED (vertex-vs-face): boundary gap = {:e} m  (ratio to sqrt(10*EPS) = {:.3})",
            b,
            b / eps_rel()
        );
    }
}
