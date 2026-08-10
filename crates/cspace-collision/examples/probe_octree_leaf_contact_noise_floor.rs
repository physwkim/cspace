// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No upstream file: a follow-up investigation to
// `probe_gjk_positive_gap_boundary.rs`. Does NOT touch
// `crates/cspace-collision/src/parry.rs` -- this calls
// `cspace_core::geometry::compound_from_octree` and `parry3d_f64::query::contact`
// directly, exercising the real octree-leaf-to-`Compound` conversion
// production code uses, without going through `accumulate_collision`.

//! `accumulate_collision`'s doc comment relies on octree "case 4" --
//! `octree_world_collision_response.json`'s id 4, an octree leaf's -x face
//! flush against the robot collision box's +x face -- as the reason
//! `dist > 0.0` must still count as collision: parry's `Cuboid`-`Cuboid`
//! contact for that exact configuration returns `dist = +4.129349354679189e-17`
//! (`parry.rs:5109`, `:5187`), and upstream fcl calls the same configuration
//! a genuine collision (`robot_distance: -0.0`).
//!
//! `probe_gjk_positive_gap_boundary.rs` found that the analogous positive-gap
//! `Some` for `Cylinder`-`Cuboid` and rotated `Cuboid`-`Cuboid` pairs sits at
//! `~1e-8 m` (order of `sqrt(10*f64::EPSILON)`, gjk.rs:415's relative
//! convergence tolerance) for SOME parameter choices, and collapses to the
//! `~1e-15` machine-epsilon noise floor for others, with no clean predictor
//! found (not shape scale alone, not contact-feature type alone) --
//! including a demonstrated case where changing only whether a scale factor
//! is an exact power of two flips a configuration between the two regimes.
//!
//! Given that, this probe asks: is octree case 4's `+4.129e-17` reliably a
//! member of the "collapsed, near machine epsilon" population, or could a
//! *different but equally reachable* octree leaf / robot box combination
//! land in the `~1e-8` "wide" population instead -- which would mean
//! `accumulate_collision`'s `dist > 0.0 => collision` behavior for octree
//! pairs is not merely "tolerating a ~1e-17 rounding residual" but
//! "tolerating an up-to-~1e-8 m false-collision band that happens not to
//! have been hit by this fixture's specific numbers"?
//!
//! # Method
//!
//! Builds a real [`cspace_core::octomap::OcTree`] at each swept `resolution`,
//! occupies one leaf via [`cspace_core::octomap::OcTree::update_node`] (so the
//! leaf's `coordinate()`/`size()` come from the tree's own
//! `key_to_coord_axis`/`node_size`, not a hand-approximation), converts it
//! through the real [`cspace_core::geometry::compound_from_octree`], and places a
//! robot [`Cuboid`] so its face is exactly flush (by construction, using the
//! leaf's own reported center/size) against the leaf's face along the swept
//! `axis`. Calls `parry3d_f64::query::contact` directly on the resulting
//! `Compound`/`Cuboid` pair -- the same dispatch
//! (`contact_composite_shape_shape` iterating `dispatcher.contact` per leaf,
//! `default_query_dispatcher.rs:317-320`'s commented-out `Cuboid`-`Cuboid`
//! closed form) that `accumulate_collision` itself exercises.
//!
//! `resolution` and `box_half` are each swept over BOTH power-of-two values
//! (`0.0625, 0.125, 0.25, 0.5, 1.0, 2.0` / `0.25, 0.5, 1.0`) and
//! non-power-of-two values (`0.1, 0.05, 0.025, 0.15, 0.033, 0.2, 0.075,
//! 0.01, 0.3, 0.045` / `0.3, 0.7, 0.45`) -- the exact axis
//! `probe_gjk_positive_gap_boundary.rs`'s scale sweep showed drove that
//! probe's own regime split. `axis` is swept over all three coordinate axes.
//! The literal case-4 fixture configuration (`resolution = 0.1`, `box_half =
//! 0.5`, `axis = X`) is run first, standalone, and asserted to reproduce the
//! known `+4.129349354679189e-17` bit for bit -- if it doesn't, the harness
//! itself does not faithfully reproduce case 4's real computation and
//! nothing else it measures can be trusted.
//!
//! A separate, final sweep holds `resolution`/`box_half` at case 4's own
//! real values and instead varies WHICH leaf is occupied -- from the one
//! immediately adjacent to the robot box out to near
//! `OcTree::TREE_MAX_VAL = 32768`'s addressable edge (`tree.rs:646`), i.e.
//! leaf coordinates from `~0.55 m` to `~3,200 m` -- to isolate absolute leaf
//! *position* (as opposed to leaf *size*/box size, already swept above) as
//! its own variable.
//!
//! # Result
//!
//! All 288 (axis x resolution x box_half) configurations stayed at
//! `|dist| <= 1.07e-16` -- roughly 8-9 orders of magnitude below the
//! `~1e-8 m` "wide" regime, and even below the `~1e-15` collapsed-but-still-
//! measurable floor `probe_gjk_positive_gap_boundary.rs`'s degenerate
//! face-to-face `Cuboid`-`Cuboid` sweeps landed on. No power-of-two
//! sensitivity, no regime switch: octree leaf x robot box face contact
//! appears to be a structurally more stable computation than the curved or
//! generically-rotated pairs that probe characterized.
//!
//! The leaf-origin sweep found the residual grows SMOOTHLY (not chaotically)
//! with the leaf's absolute coordinate magnitude -- ordinary IEEE-754 ULP
//! scaling (`ulp(x) ~ x * 2^-52`), not the `eps_rel`-driven regime-switch --
//! crossing `1e-14` only at a `~1,000 m` leaf coordinate (`dist =
//! -4.55e-14`, still ~1,000x under the `~1e-8` "wide" regime), and the
//! exact-touch construction itself breaking down (`dist = None`, no longer
//! reporting a contact at all) at `~3,200 m`, right at `TREE_MAX_VAL`'s own
//! addressable edge. `1,000 m`-`3,200 m` leaf coordinates ARE representable
//! by `OcTree`'s key space (not rejected as invalid), but are not reachable
//! by any realistic MoveIt robot/world model -- typical workspaces are under
//! `10 m`, where every measured residual stays at `~1e-16` to `~1e-17`,
//! matching case 4's own real value.
//!
//! Run: `cargo run -p cspace-collision --example probe_octree_leaf_contact_noise_floor --release`

use cspace_core::geometry::compound_from_octree;
use cspace_core::octomap::OcTree;
use nalgebra::Point3;
use parry3d_f64::math::Pose;
use parry3d_f64::query;
use parry3d_f64::shape::Cuboid;

#[derive(Clone, Copy)]
enum Axis {
    X,
    Y,
    Z,
}

impl Axis {
    fn name(self) -> &'static str {
        match self {
            Axis::X => "x",
            Axis::Y => "y",
            Axis::Z => "z",
        }
    }

    fn point(self, along: f64) -> Point3<f64> {
        match self {
            Axis::X => Point3::new(along, 0.0, 0.0),
            Axis::Y => Point3::new(0.0, along, 0.0),
            Axis::Z => Point3::new(0.0, 0.0, along),
        }
    }

    fn translation(self, along: f64) -> (f64, f64, f64) {
        match self {
            Axis::X => (along, 0.0, 0.0),
            Axis::Y => (0.0, along, 0.0),
            Axis::Z => (0.0, 0.0, along),
        }
    }

    fn coordinate(self, p: Point3<f64>) -> f64 {
        match self {
            Axis::X => p.x,
            Axis::Y => p.y,
            Axis::Z => p.z,
        }
    }
}

/// Occupies one leaf at `resolution` so its face on the `-axis` side sits
/// (by construction, using the leaf's own real reported center/size) exactly
/// `0.0` away from a `box_half`-half-extent robot cuboid's `+axis` face.
/// Returns the raw `dist` parry's `query::contact` reports for that pair, or
/// `None` if parry reports no contact at all (itself a finding worth
/// surfacing, not an error).
fn touching_dist(axis: Axis, resolution: f64, box_half: f64) -> Option<f64> {
    touching_dist_at_leaf_offset(axis, resolution, box_half, 0)
}

/// As [`touching_dist`], but the occupied leaf is `leaf_offset` leaf-widths
/// further out along `axis` than the leaf immediately adjacent to the robot
/// box -- the box is repositioned to keep touching THAT (farther-out) leaf's
/// actual reported face exactly, so this isolates "which leaf, i.e. which
/// absolute-magnitude world coordinate, is occupied" as its own variable,
/// independent of `resolution`/`box_half`. `leaf_offset` is in units of
/// `resolution`, not raw metres, so every value stays leaf-aligned (a
/// non-aligned point would still snap to a real leaf via the octree's own
/// key rounding, but would no longer isolate origin from resolution).
fn touching_dist_at_leaf_offset(
    axis: Axis,
    resolution: f64,
    box_half: f64,
    leaf_offset: i64,
) -> Option<f64> {
    let mut tree = OcTree::new(resolution);
    // Request a point inside the leaf whose -axis face should land exactly
    // on the robot box's +axis face: leaf center (along axis) = box_half +
    // leaf_half + leaf_offset*resolution, where leaf_half = resolution / 2.0
    // (the exact halving `compound_from_octree` itself performs,
    // `octree_collision.rs`'s `leaf.size() / 2.0`).
    let leaf_half_guess = resolution / 2.0;
    let request_point = axis.point(box_half + leaf_half_guess + (leaf_offset as f64) * resolution);
    tree.update_node(request_point, true, false);

    let leaf = tree
        .leaves()
        .find(cspace_core::octomap::Leaf::is_occupied)
        .expect("update_node just occupied one leaf");
    let leaf_center = axis.coordinate(leaf.coordinate());
    let leaf_half = leaf.size() / 2.0;

    let compound = compound_from_octree(&tree).expect("one occupied leaf");
    let cuboid = Cuboid::new(parry3d_f64::math::Vector::new(box_half, box_half, box_half));
    // Box center placed so its +axis face sits exactly at the leaf's own
    // -axis face: box_center = leaf_center - leaf_half - box_half.
    let box_center_axis = leaf_center - leaf_half - box_half;
    let (bx, by, bz) = axis.translation(box_center_axis);
    let pose_box = Pose::translation(bx, by, bz);

    query::contact(&Pose::IDENTITY, &compound, &pose_box, &cuboid, 0.0)
        .expect("supported pair")
        .map(|c| c.dist)
}

fn main() {
    // --- Sanity check: reproduce the literal case-4 fixture bit for bit. ---
    let case4 = touching_dist(Axis::X, 0.1, 0.5);
    println!(
        "case 4 reproduction (resolution=0.1, box_half=0.5, axis=x): dist = {:?}",
        case4
    );
    const CASE4_KNOWN: f64 = 4.129349354679189e-17;
    match case4 {
        Some(d) if d == CASE4_KNOWN => {
            println!("  MATCHES parry.rs:5109/:5187's pinned value bit for bit.\n");
        }
        Some(d) => {
            panic!(
                "harness does not reproduce case 4: got {d:e}, expected {CASE4_KNOWN:e} -- \
                 nothing else this probe measures can be trusted until this matches."
            );
        }
        None => panic!("harness got None for case 4, expected Some({CASE4_KNOWN:e})"),
    }

    let resolutions_pow2 = [0.0625, 0.125, 0.25, 0.5, 1.0, 2.0];
    let resolutions_non_pow2 = [0.1, 0.05, 0.025, 0.15, 0.033, 0.2, 0.075, 0.01, 0.3, 0.045];
    let box_halves_pow2 = [0.25, 0.5, 1.0];
    let box_halves_non_pow2 = [0.3, 0.7, 0.45];
    let axes = [Axis::X, Axis::Y, Axis::Z];

    let resolutions: Vec<(f64, bool)> = resolutions_pow2
        .iter()
        .map(|&r| (r, true))
        .chain(resolutions_non_pow2.iter().map(|&r| (r, false)))
        .collect();
    let box_halves: Vec<(f64, bool)> = box_halves_pow2
        .iter()
        .map(|&b| (b, true))
        .chain(box_halves_non_pow2.iter().map(|&b| (b, false)))
        .collect();

    // Noise-floor threshold: probe_gjk_positive_gap_boundary.rs's collapsed
    // regime measured ~2.2e-15 to ~2.7e-15 across every degenerate
    // configuration it tried; case 4 itself is ~4.1e-17, well under that.
    // 1e-14 is a generous margin above both, so "exceeds this" means
    // "clearly NOT the collapsed regime", not a borderline call.
    const NOISE_FLOOR_THRESHOLD: f64 = 1e-14;

    let mut results = Vec::new();
    for &axis in &axes {
        for &(resolution, res_pow2) in &resolutions {
            for &(box_half, box_pow2) in &box_halves {
                let dist = touching_dist(axis, resolution, box_half);
                results.push((axis, resolution, res_pow2, box_half, box_pow2, dist));
            }
        }
    }

    println!(
        "{} configurations swept (axis x resolution x box_half, {} resolutions x {} box_halves x 3 axes)\n",
        results.len(),
        resolutions.len(),
        box_halves.len()
    );

    let mut none_count = 0usize;
    let mut over_floor = Vec::new();
    let mut max_abs = 0.0_f64;
    for &(axis, resolution, res_pow2, box_half, box_pow2, dist) in &results {
        match dist {
            None => none_count += 1,
            Some(d) => {
                max_abs = max_abs.max(d.abs());
                if d.abs() > NOISE_FLOOR_THRESHOLD {
                    over_floor.push((axis, resolution, res_pow2, box_half, box_pow2, d));
                }
            }
        }
    }

    println!("dist == None (parry reported no contact at all): {none_count}");
    println!(
        "max |dist| across all {} Some(..) results: {max_abs:e}",
        results.len() - none_count
    );
    println!(
        "configurations with |dist| > {NOISE_FLOOR_THRESHOLD:e} (clearly outside the collapsed/noise-floor regime): {}",
        over_floor.len()
    );
    if over_floor.is_empty() {
        println!(
            "  -> NONE. Every swept (axis, resolution, box_half) combination -- power-of-two\n     \
             and non-power-of-two alike -- stayed at or under the noise floor. Octree leaf x\n     \
             robot box face contact, for this axis-aligned face-to-face configuration family,\n     \
             appears to be a structurally different (and more stable) computational case than\n     \
             the curved (Cylinder) or generically-rotated (Cuboid vertex-vs-face) contacts\n     \
             probe_gjk_positive_gap_boundary.rs found to be power-of-two-sensitive."
        );
    } else {
        println!("  -> OUTLIERS FOUND (latent defect candidate):");
        for (axis, resolution, res_pow2, box_half, box_pow2, d) in &over_floor {
            println!(
                "     axis={}  resolution={:e}{}  box_half={:e}{}  dist={:e}",
                axis.name(),
                resolution,
                if *res_pow2 { " (pow2)" } else { "" },
                box_half,
                if *box_pow2 { " (pow2)" } else { "" },
                d
            );
        }
    }

    println!();
    println!("full table (axis, resolution, box_half, dist):");
    for (axis, resolution, res_pow2, box_half, box_pow2, dist) in &results {
        println!(
            "  axis={}  resolution={:e}{}  box_half={:e}{}  dist={}",
            axis.name(),
            resolution,
            if *res_pow2 { " (pow2)" } else { "     " },
            box_half,
            if *box_pow2 { " (pow2)" } else { "     " },
            dist.map(|d| format!("{d:e}"))
                .unwrap_or_else(|| "None".to_string())
        );
    }

    // --- Leaf ORIGIN sweep: the sweep above varies resolution and box_half,
    // but the occupied leaf's own absolute world coordinate is a side effect
    // of those, not an independently controlled variable. Hold resolution
    // and box_half fixed at case 4's own real values (0.1, 0.5) and instead
    // vary WHICH leaf is occupied, from the one immediately adjacent to the
    // robot box out to near the octree's addressable edge
    // (`OcTree::TREE_MAX_VAL = 32768`, `cspace-octomap/src/tree.rs:646`) --
    // covering leaf coordinates from ~0.55 m out to ~3,200 m. This directly
    // tests whether the absolute magnitude of a leaf's own coordinate (as
    // opposed to the resolution/box_half scale already swept above) can push
    // the residual out of the noise floor. ---
    {
        println!();
        println!(
            "leaf ORIGIN sweep (resolution=0.1, box_half=0.5 fixed, axis=x, leaf_offset in units of resolution):"
        );
        let offsets: [i64; 15] = [
            0, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89, 144, 1_000, 10_000, 32_000,
        ];
        let mut origin_max_abs = 0.0_f64;
        let mut origin_over_floor = 0usize;
        for &leaf_offset in &offsets {
            let dist = touching_dist_at_leaf_offset(Axis::X, 0.1, 0.5, leaf_offset);
            let leaf_coord_approx = 0.5 + 0.05 + (leaf_offset as f64) * 0.1;
            match dist {
                Some(d) => {
                    origin_max_abs = origin_max_abs.max(d.abs());
                    if d.abs() > NOISE_FLOOR_THRESHOLD {
                        origin_over_floor += 1;
                    }
                    println!(
                        "  leaf_offset={leaf_offset:>6}  leaf coord~{leaf_coord_approx:>8.2} m  dist={d:e}"
                    );
                }
                None => println!(
                    "  leaf_offset={leaf_offset:>6}  leaf coord~{leaf_coord_approx:>8.2} m  dist=None"
                ),
            }
        }
        println!(
            "  max |dist| across leaf-origin sweep: {origin_max_abs:e}  ({origin_over_floor} over {NOISE_FLOOR_THRESHOLD:e} threshold)"
        );
    }
}
