// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Separates the two populations behind `PORTING-PLAN.md` §5 Phase 3's last
//! `collision: bool` carve-out -- the positive-gap band §229.2 measured
//! (`3e-8` colliding, `1e-7` not, on prbt's cylinder against a floor box) and
//! left `UNFIXED` -- and, per this round's follow-up brief, compares
//! `crates/moveit-collision/src/parry.rs`'s two sibling accumulators
//! (`accumulate_collision` and `accumulate_distance`) on the identical
//! configurations, since they turn out to gate the same
//! `parry3d_f64::query::contact` result by two different rules.
//!
//! No oracle, no docker: every synthetic scene is constructed, so the
//! ground-truth gap is known by construction, and the only question is what
//! this port's own `parry3d_f64::query::contact` (reached via
//! `contact_support_map_support_map`,
//! `parry3d-f64-0.30.0/src/query/contact/contact_support_map_support_map.rs:20-24`)
//! reports for it, and how `parry.rs`'s two call sites interpret that report.
//!
//! # The two call sites (current tree, `main` at `e2639964`)
//!
//! - `accumulate_collision` (`parry.rs:2245`): `query::contact(.., 0.0)`.
//!   Off the exact tie (`contact.dist != 0.0`) *any* `Some` is unconditionally
//!   a touch -- no sign check. At the exact tie (`contact.dist == 0.0`
//!   bit-for-bit) `fcl_tangency_verdict`/`is_mesh_pair` decide instead, a
//!   dispatch table this round's `residual-triage` merge (`e2639964`) added.
//! - `accumulate_distance` (`parry.rs:2464`): `query::contact(..,
//!   bounded_prediction(threshold))`, then `if contact.dist >= threshold {
//!   continue }` (`:2473`), then, independently, `if data.distance <= 0.0 {
//!   result.collision = true }` (`:2502`).
//!
//! `bounded_prediction`'s own doc (`parry.rs:1243-1257`) says clamping to
//! `0.0` "match[es] `accumulate_collision`'s own prediction-`0.0` convention
//! for a touching-or-penetrating-only query" -- but `accumulate_collision`'s
//! *own* doc, two functions down, says the opposite of that convention holds
//! ("NOT gated on `contact.dist <= 0.0`... `parry` returns a contact across a
//! small positive gap too"). Both cannot be the operative rule; §1 below
//! measures which one actually governs `accumulate_distance`'s correctness.
//!
//! # What this binary measures
//!
//! 1. Whether `accumulate_distance`'s `contact.dist >= threshold` skip is
//!    what keeps it correct on the band, or whether it is vacuous there
//!    (`DistanceRequestType::Global`'s threshold starts at
//!    `DistanceResultsData::default().distance == f64::MAX`, so the skip
//!    cannot fire until some *other* pair has already reported something
//!    smaller) -- and, separately, whether `data.distance <= 0.0` is doing
//!    the actual work.
//! 2. The two committed anchors -- prbt's cylinder-on-box exact tangency
//!    (`exact_tangency_boundary.rs`) and `octree_world_collision_response.
//!    json` case id 4 -- reproduced here directly, each run through *both*
//!    sibling accumulators, not just `accumulate_collision`.
//! 3. Population A's own magnitude: the raw `dist` `accumulate_distance`
//!    reports for shape pairs placed at *exact* geometric tangency (gap 0.0
//!    by construction), across a size/position sweep -- this is rounding
//!    noise, not a measurement of real separation, and its scale is the
//!    other half of the separability question.
//! 4. Population B's width on each accumulator separately: the largest gap
//!    at which `accumulate_collision` still (wrongly) reports collision, and
//!    the largest gap at which `accumulate_distance`'s own `data.distance <=
//!    0.0` rule does the same (expected near-zero, if finding 1 holds) --
//!    across the same sweep, plus kind pairs `fcl_tangency_table`
//!    classifies (`Box`/`Sphere`/`Cylinder`/`Cone`), not only the
//!    `cylinder x box`/`cylinder x cylinder` pair the brief names, since
//!    `contact_support_map_support_map` is generic over all of them except
//!    `Ball/Ball`.
//!
//! Run: `cargo run --release --bin positive_gap_band_separation -p
//! moveit-diff`. Prints one line per measurement, machine-parseable, and a
//! verdict at the end. Exits `0` always -- this is a measurement, not a gate.

use std::sync::Arc;

use moveit_collision::{
    AllowedCollisionMatrix, AttachedBodyGeometry, CollisionEnv, CollisionRequest, DistanceRequest,
    DistanceRequestType, LinkPaddingScale, ParryCollisionEnv, World,
};
use moveit_geometry::{Cuboid, Cylinder, Isometry3, Shape};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_octomap::OcTree;
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;
use nalgebra::Point3;

/// `sqrt(10 * f64::EPSILON)` -- `parry3d-f64-0.30.0/src/query/gjk/gjk.rs`'s
/// own `eps_rel`, recomputed here rather than hardcoded so a `parry3d-f64`
/// upgrade that changes `eps_tol()` changes this too.
fn eps_rel() -> f64 {
    (10.0 * f64::EPSILON).sqrt()
}

/// Both accumulators' verdicts on the identical configuration, plus the raw
/// distance both are built from -- `accumulate_distance`'s `contact.dist` is
/// read out through `DistanceRequest::enable_signed_distance`, since
/// `accumulate_collision` never exposes its own `contact.dist` through any
/// public field (only the `bool` it derives).
#[derive(Clone, Copy, Debug)]
struct Verdict {
    /// `accumulate_collision`'s verdict (`ParryCollisionEnv::check_robot_collision`).
    bool_collision: bool,
    /// `accumulate_distance`'s own verdict (`DistanceResult::collision`,
    /// `parry.rs:2502-2504`'s `data.distance <= 0.0`), read independently of
    /// `bool_collision` -- the two are computed by different code paths even
    /// though both ultimately call `query::contact` on the same shapes.
    dist_collision: bool,
    /// `DistanceResult::minimum_distance.distance`, signed.
    dist_value: f64,
}

impl Verdict {
    fn agree(&self) -> bool {
        self.bool_collision == self.dist_collision
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Kind {
    Box,
    Sphere,
    Cylinder,
    Cone,
}

impl Kind {
    const fn name(self) -> &'static str {
        match self {
            Self::Box => "box",
            Self::Sphere => "sphere",
            Self::Cylinder => "cylinder",
            Self::Cone => "cone",
        }
    }

    /// A shape of this kind with local half-extent exactly `half` along every
    /// axis -- same convention as `exact_tangency_is_decided_per_shape_pair.
    /// rs`'s `Kind::shape`. `Sphere`'s "half" is its radius.
    fn shape(self, half: f64) -> Arc<Shape> {
        Arc::new(match self {
            Self::Box => Shape::Cuboid(
                Cuboid::new(2.0 * half, 2.0 * half, 2.0 * half).expect("positive cuboid"),
            ),
            Self::Sphere => {
                Shape::Sphere(moveit_geometry::Sphere::new(half).expect("positive sphere"))
            }
            Self::Cylinder => {
                Shape::Cylinder(Cylinder::new(half, 2.0 * half).expect("positive cylinder"))
            }
            Self::Cone => {
                Shape::Cone(moveit_geometry::Cone::new(half, 2.0 * half).expect("positive cone"))
            }
        })
    }
}

fn build_prbt() -> RobotModel {
    let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/prbt.urdf");
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/prbt.srdf");
    let urdf_xml = std::fs::read_to_string(urdf_path).expect("fixture URDF must be readable");
    let urdf = urdf_rs::read_file(urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

/// An ACM that allows every pair *except* `"upper"` x `"lower"` -- so a huge
/// synthetic shape at either id cannot register a spurious hit against
/// prbt's own geometry and corrupt the one pair under test. Built from every
/// real link name plus the two synthetic ids, all defaulted `true`
/// (allowed), then the one pair set back to `false` -- `tangency_subset.rs`'s
/// `probe_hits` isolates its probe the same way.
fn isolated_acm(model: &RobotModel) -> AllowedCollisionMatrix {
    let mut names: Vec<String> = model
        .link_models()
        .iter()
        .map(|l| l.name().to_owned())
        .collect();
    names.push("upper".to_owned());
    names.push("lower".to_owned());
    let mut acm = AllowedCollisionMatrix::from_names(&names, true);
    acm.set_entry("upper", "lower", false);
    acm
}

/// Both sibling accumulators' verdicts for `upper` (attached to
/// `prbt_base_link`, whose default-state world transform is the identity --
/// see `exact_tangency_is_decided_per_shape_pair.rs`'s
/// `the_attached_frame_is_the_world_frame`) stacked `2 * half + gap` above
/// `lower` (a world object), both centred at `(position_x, 0, ...)`.
///
/// Single isolated pair, `DistanceRequestType::Global` (the default): the
/// running-minimum threshold `accumulate_distance` computes for this one
/// pair starts at `DistanceResultsData::default().distance == f64::MAX`
/// (`common.rs:401`) and there is no earlier pair to have improved it --
/// `contact.dist >= threshold` (`parry.rs:2473`) is therefore checking `dist
/// >= f64::MAX`, which cannot fire for any real geometry. That is a fact
/// about this harness's isolation, not an assumption about the mechanism
/// under test: it is what makes finding 1 (whether the skip or the sign
/// check is what governs `accumulate_distance` here) measurable at all --
/// with only one candidate pair, the skip is provably inert, so whatever
/// `dist_collision` reports below is `data.distance <= 0.0` alone.
fn query(
    model: &RobotModel,
    acm: &AllowedCollisionMatrix,
    upper: Kind,
    lower: Kind,
    half: f64,
    position_x: f64,
    gap: f64,
) -> Verdict {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    let posed = state.update();
    let touch_links = std::collections::BTreeSet::new();

    let upper_shapes = [upper.shape(half)];
    let upper_poses = [Isometry3::translation(position_x, 0.0, half + gap)];
    let attached = AttachedBodyGeometry {
        id: "upper",
        link_name: "prbt_base_link",
        shapes: &upper_shapes,
        shape_poses: &upper_poses,
        touch_links: &touch_links,
    };

    let mut world = World::new();
    world.add_shape(
        "lower",
        lower.shape(half),
        Isometry3::translation(position_x, 0.0, -half),
    );
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
    let attached_slice = std::slice::from_ref(&attached);
    let bool_collision = env
        .check_robot_collision(
            &CollisionRequest::default(),
            &posed,
            attached_slice,
            Some(acm),
        )
        .collision;
    let distance_request = DistanceRequest {
        enable_signed_distance: true,
        request_type: DistanceRequestType::Global,
        acm: Some(acm),
        ..DistanceRequest::default()
    };
    let distance_result = env.distance_robot(&distance_request, &posed, attached_slice);
    Verdict {
        bool_collision,
        dist_collision: distance_result.collision,
        dist_value: distance_result.minimum_distance.distance,
    }
}

/// Population B's width on the `bool_collision` channel: the largest gap
/// this binary can still find with `bool_collision == true`, found by
/// exponential search for a bracket then 200 bisections (double precision
/// exhausts long before that -- cheap and avoids tuning a step count to the
/// scale of `half`).
fn find_bool_boundary(
    model: &RobotModel,
    acm: &AllowedCollisionMatrix,
    upper: Kind,
    lower: Kind,
    half: f64,
    position_x: f64,
) -> Result<f64, &'static str> {
    find_boundary(model, acm, upper, lower, half, position_x, |v| {
        v.bool_collision
    })
}

/// Same search, but on `dist_collision` -- `accumulate_distance`'s own,
/// independently-derived verdict. If finding 1 (the skip at `parry.rs:2473`
/// is vacuous here, `data.distance <= 0.0` alone governs) is right, this
/// should come back essentially at `gap == 0` (bounded by population A's own
/// rounding noise, not by GJK's much coarser relative-convergence
/// tolerance) rather than tracking `find_bool_boundary`.
fn find_dist_boundary(
    model: &RobotModel,
    acm: &AllowedCollisionMatrix,
    upper: Kind,
    lower: Kind,
    half: f64,
    position_x: f64,
) -> Result<f64, &'static str> {
    find_boundary(model, acm, upper, lower, half, position_x, |v| {
        v.dist_collision
    })
}

/// Log-spaced scan from `1e-20 * scale` up to `half.max(1.0)`, recording
/// the *largest* probed gap where `collides_at` holds, then a local
/// bisection refining its outer edge against the smallest probed gap above
/// it where `collides_at` does not.
///
/// Deliberately not a doubling search seeded from `lo = 0.0` (tried first,
/// reverted): `contact.dist == 0.0` bit-for-bit routes through
/// `fcl_tangency_verdict`, which answers `false` for any kind pair
/// `fcl_tangency_table::SPECIALISED` does not mark -- `cylinder x cylinder`
/// among them (fcl's own cylinder-cylinder narrowphase is unspecialised
/// libccd MPR, not a closed form). A synthetic pair placed at gap `0.0`
/// often lands `contact.dist` at bit-exact `0.0` too (measured: every
/// `cylinder x cylinder` row up to `half = 1.0` in this file's sweep prints
/// `tie_dist=0.000000e0`), so a search that assumes "gap `0.0` is a known
/// `true`" silently breaks its own precondition for exactly the pairs where
/// that matters, and reports a meaningless near-zero "boundary" instead of
/// scanning for the real (possibly non-adjacent-to-zero) band the off-tie
/// `Some ⟹ true` catch-all still produces at a genuinely-nonzero gap.
/// Scanning the full range and keeping the *largest* true (rather than
/// stopping at the first true-to-false transition) also does not assume
/// the band is a single contiguous island adjacent to zero -- it finds the
/// outermost edge even if the tangency-table exception carves a false
/// pocket out of the middle of it.
fn find_boundary(
    model: &RobotModel,
    acm: &AllowedCollisionMatrix,
    upper: Kind,
    lower: Kind,
    half: f64,
    position_x: f64,
    collides_at: impl Fn(&Verdict) -> bool,
) -> Result<f64, &'static str> {
    let ceiling = half.max(1.0);
    let mut best_true: Option<f64> = None;
    let mut smallest_false_above_best_true: Option<f64> = None;
    let mut exponent = -20.0f64;
    while exponent <= 0.0 {
        let gap = ceiling * 10f64.powf(exponent);
        let v = query(model, acm, upper, lower, half, position_x, gap);
        if collides_at(&v) {
            best_true = Some(gap);
            smallest_false_above_best_true = None; // a later false may now be closer
        } else if best_true.is_some() && smallest_false_above_best_true.is_none() {
            smallest_false_above_best_true = Some(gap);
        }
        exponent += 0.1;
    }
    let (mut lo, mut hi) = match (best_true, smallest_false_above_best_true) {
        (Some(lo), Some(hi)) => (lo, hi),
        (Some(lo), None) => return Ok(lo), // true all the way to `ceiling`
        (None, _) => return Err("no true gap found in the scanned range"),
    };
    for _ in 0..200 {
        let mid = lo + (hi - lo) / 2.0;
        if mid == lo || mid == hi {
            break; // double precision exhausted
        }
        let v = query(model, acm, upper, lower, half, position_x, mid);
        if collides_at(&v) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    Ok(hi)
}

/// Population A's magnitude at *exact* geometric tangency (`gap == 0.0` by
/// construction): the raw signed `dist` `accumulate_distance` reports there.
/// Whatever comes back is this port's own rounding of composing poses and
/// the GJK solve, not a measurement of real separation.
fn tie_noise(
    model: &RobotModel,
    acm: &AllowedCollisionMatrix,
    upper: Kind,
    lower: Kind,
    half: f64,
    position_x: f64,
) -> Verdict {
    query(model, acm, upper, lower, half, position_x, 0.0)
}

const PAIRS: [(Kind, Kind); 6] = [
    (Kind::Cylinder, Kind::Box),
    (Kind::Box, Kind::Cylinder),
    (Kind::Cylinder, Kind::Cylinder),
    (Kind::Box, Kind::Box),
    (Kind::Cone, Kind::Box),
    (Kind::Sphere, Kind::Cylinder),
];

/// One printed row of `main`'s per-pair sweep -- the shape-kind pair and
/// sweep-point coordinates under test, plus the `Verdict` at exact tie and
/// both accumulators' boundary-search results. `report_row` used to take
/// each of these positionally: `half`/`position_x` (both `f64`), `upper`/
/// `lower` (both `Kind`) and `bool_boundary`/`dist_boundary` (both
/// `Result<f64, &'static str>`) are each an interchangeable same-typed pair
/// a positional call could transpose silently, the same hazard
/// `clippy::too_many_arguments` was flagging in `GridGeometry`'s
/// `size_*`/`origin_*` precedent. A struct literal forces every field to be
/// named at its construction site instead.
struct SweepRow {
    half: f64,
    position_x: f64,
    axis: &'static str,
    upper: Kind,
    lower: Kind,
    tie: Verdict,
    bool_boundary: Result<f64, &'static str>,
    dist_boundary: Result<f64, &'static str>,
}

fn report_row(row: SweepRow) {
    let SweepRow {
        half,
        position_x,
        axis,
        upper,
        lower,
        tie,
        bool_boundary: bnd_bool,
        dist_boundary: bnd_dist,
    } = row;
    let scale = half.max(position_x.abs()).max(1e-300);
    let bnd_bool_str = match bnd_bool {
        Ok(b) => format!("{b:.6e}"),
        Err(e) => format!("NOT_FOUND({e})"),
    };
    let bnd_dist_str = match bnd_dist {
        Ok(b) => format!("{b:.6e}"),
        Err(e) => format!("NOT_FOUND({e})"),
    };
    let bnd_bool_norm_eps = bnd_bool
        .map(|b| b / (f64::EPSILON * scale))
        .unwrap_or(f64::NAN);
    let bnd_bool_norm_epsrel = bnd_bool
        .map(|b| b / (eps_rel() * scale))
        .unwrap_or(f64::NAN);
    let tie_norm_eps = tie.dist_value.abs() / (f64::EPSILON * scale);
    println!(
        "sweep={axis:<8} pair={:<8}x{:<8} half={half:>10.3e} pos={position_x:>10.3e} \
         tie_dist={:>14.6e} tie_agree={} tie/EPS*scale={:>10.3e} \
         bool_boundary={bnd_bool_str:<22} dist_boundary={bnd_dist_str:<22} \
         boolB/EPS*scale={bnd_bool_norm_eps:>10.3e} boolB/eps_rel*scale={bnd_bool_norm_epsrel:>10.3e}",
        upper.name(),
        lower.name(),
        tie.dist_value,
        tie.agree(),
        tie_norm_eps,
    );
}

/// Reproduces `exact_tangency_boundary.rs`'s prbt cylinder-on-box scene
/// directly (not by quoting the file), so this binary's own anchor numbers
/// come from a run, not a copy -- extended to also read
/// `distance_robot(...).collision`, which that test file never asserts on.
fn prbt_anchor() {
    let model = build_prbt();
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/prbt.srdf");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    let acm = AllowedCollisionMatrix::from_srdf(&srdf);

    let floor_env = |top_z: f64| {
        let mut world = World::new();
        world.add_shape(
            "floor",
            Arc::new(Shape::Cuboid(
                Cuboid::new(4.0, 4.0, 0.1).expect("positive cuboid"),
            )),
            Isometry3::translation(0.0, 0.0, top_z - 0.05),
        );
        ParryCollisionEnv::new(world, LinkPaddingScale::default())
    };
    let measure = |top_z: f64| -> Verdict {
        let env = floor_env(top_z);
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();
        let bool_collision = env
            .check_robot_collision(&CollisionRequest::default(), &posed, &[], Some(&acm))
            .collision;
        let distance_result = env.distance_robot(
            &DistanceRequest {
                enable_signed_distance: true,
                request_type: DistanceRequestType::Global,
                acm: Some(&acm),
                ..DistanceRequest::default()
            },
            &posed,
            &[],
        );
        Verdict {
            bool_collision,
            dist_collision: distance_result.collision,
            dist_value: distance_result.minimum_distance.distance,
        }
    };

    let tie = measure(0.0);
    println!(
        "anchor=prbt_exact_tangency population=A bool_collision={} dist_collision={} \
         dist_value={:e} agree={} (population A: gap is exactly 0.0 by construction; upstream \
         reports the -1.0 sentinel here, not a real value to compare against; local scale is the \
         cylinder radius 0.065)",
        tie.bool_collision,
        tie.dist_collision,
        tie.dist_value,
        tie.agree(),
    );

    // Bisect prbt's own boundary on both channels rather than re-quoting the
    // 3e-8/1e-7 bracket exact_tangency_boundary.rs already pins.
    let bisect = |collides_at: &dyn Fn(&Verdict) -> bool| -> f64 {
        let mut lo = 0.0f64;
        let mut hi = 1e-9;
        loop {
            let v = measure(-hi);
            if !collides_at(&v) {
                break;
            }
            lo = hi;
            hi *= 4.0;
            if hi > 1e-3 {
                break;
            }
        }
        for _ in 0..200 {
            let mid = lo + (hi - lo) / 2.0;
            if mid == lo || mid == hi {
                break;
            }
            let v = measure(-mid);
            if collides_at(&v) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        hi
    };
    let local_half = 0.065_f64;
    let bool_boundary = bisect(&|v| v.bool_collision);
    let dist_boundary = bisect(&|v| v.dist_collision);
    println!(
        "anchor=prbt_boundary_bisected population=B bool_boundary={bool_boundary:e} \
         dist_boundary={dist_boundary:e} bool/eps_rel*local_half={:e} \
         bool/EPS*local_half={:e} dist/EPS*local_half={:e} (local_half={local_half}, \
         position~0 -- prbt_base_link sits at the world origin)",
        bool_boundary / (eps_rel() * local_half),
        bool_boundary / (f64::EPSILON * local_half),
        dist_boundary / (f64::EPSILON * local_half),
    );
}

/// Reproduces `octree_world_collision_parity.rs`'s case id 4 directly: a
/// `0.1`-resolution leaf at `(0.55, 0, 0)` against the fixture robot's own
/// `1x1x1` box link `"p"` at the identity pose, exact face-on-face contact
/// at `x = 0.5` -- extended to also read `distance_robot(...).collision`,
/// which that test file never asserts on (only `minimum_distance.distance`
/// against the oracle's `-0.0`, within `1e-4`).
fn octree_anchor() {
    let urdf_path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../crates/moveit-collision/tests/fixtures/octree_world_robot.urdf"
    );
    let srdf_path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../crates/moveit-collision/tests/fixtures/octree_world_robot.srdf"
    );
    let urdf_xml = std::fs::read_to_string(urdf_path).expect("octree fixture URDF readable");
    let urdf = urdf_rs::read_from_string(&urdf_xml).expect("octree fixture URDF parses");
    let srdf = SrdfModel::parse_file(srdf_path).expect("octree fixture SRDF parses");
    let model = RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("octree fixture model builds");

    let mut tree = OcTree::new(0.1);
    tree.update_node(Point3::new(0.55, 0.0, 0.0), true, false);
    let mut world = World::new();
    world.add_shape(
        "octree_object",
        Arc::new(Shape::OcTree(moveit_geometry::OcTree::from_tree(Arc::new(
            tree,
        )))),
        Isometry3::identity(),
    );
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();

    let bool_collision = env
        .check_robot_collision(&CollisionRequest::default(), &posed, &[], None)
        .collision;
    let distance_result = env.distance_robot(
        &DistanceRequest {
            enable_signed_distance: true,
            request_type: DistanceRequestType::Global,
            ..DistanceRequest::default()
        },
        &posed,
        &[],
    );
    let v = Verdict {
        bool_collision,
        dist_collision: distance_result.collision,
        dist_value: distance_result.minimum_distance.distance,
    };
    println!(
        "anchor=octree_case_4 population=A bool_collision={} dist_collision={} dist_value={:e} \
         agree={} (population A: leaf face flush on box face, gap exactly 0.0 by construction; \
         upstream reports robot_collision=true, robot_distance=-0.0; leaf half=0.05, box \
         half=0.5, position~0.55; NEITHER octree_world_collision_parity.rs NOR any other \
         committed test asserts dist_collision for this case -- only minimum_distance.distance \
         against the oracle within 1e-4, which a wrong-sign near-zero value would still pass)",
        v.bool_collision,
        v.dist_collision,
        v.dist_value,
        v.agree(),
    );
}

fn main() {
    println!("# anchors (exact constructions, reproduced not quoted)");
    prbt_anchor();
    octree_anchor();

    println!();
    println!(
        "# per-pair sweep: tie noise (population A, gap=0 by construction) and both \
         accumulators' boundaries (population B), across size and position"
    );
    let model = build_prbt();
    let acm = isolated_acm(&model);
    const SIZES: [f64; 5] = [1e-6, 1e-4, 1e-2, 1e0, 1e2];
    const POSITIONS: [f64; 5] = [0.0, 1e-3, 1e-1, 1e1, 1e3];

    for (upper, lower) in PAIRS {
        for half in SIZES {
            let position_x = 0.3;
            let tie = tie_noise(&model, &acm, upper, lower, half, position_x);
            let bnd_bool = find_bool_boundary(&model, &acm, upper, lower, half, position_x);
            let bnd_dist = find_dist_boundary(&model, &acm, upper, lower, half, position_x);
            report_row(SweepRow {
                half,
                position_x,
                axis: "size",
                upper,
                lower,
                tie,
                bool_boundary: bnd_bool,
                dist_boundary: bnd_dist,
            });
        }
    }
    for (upper, lower) in PAIRS {
        for position_x in POSITIONS {
            let half = 0.065;
            let tie = tie_noise(&model, &acm, upper, lower, half, position_x);
            let bnd_bool = find_bool_boundary(&model, &acm, upper, lower, half, position_x);
            let bnd_dist = find_dist_boundary(&model, &acm, upper, lower, half, position_x);
            report_row(SweepRow {
                half,
                position_x,
                axis: "position",
                upper,
                lower,
                tie,
                bool_boundary: bnd_bool,
                dist_boundary: bnd_dist,
            });
        }
    }
    for (upper, lower) in PAIRS {
        for half in SIZES {
            let position_x = 3.0 * half;
            let tie = tie_noise(&model, &acm, upper, lower, half, position_x);
            let bnd_bool = find_bool_boundary(&model, &acm, upper, lower, half, position_x);
            let bnd_dist = find_dist_boundary(&model, &acm, upper, lower, half, position_x);
            report_row(SweepRow {
                half,
                position_x,
                axis: "joint",
                upper,
                lower,
                tie,
                bool_boundary: bnd_bool,
                dist_boundary: bnd_dist,
            });
        }
    }

    println!();
    println!("eps_rel = {:e}", eps_rel());
    println!("f64::EPSILON = {:e}", f64::EPSILON);
}
