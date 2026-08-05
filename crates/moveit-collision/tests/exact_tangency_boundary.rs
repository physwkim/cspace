// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The exact-tangency boundary, which is where PORTING-PLAN.md §5 Phase 3's
//! `collision: bool` clause fails on prbt (6,854/10,000 states) -- and the
//! measurements that settle what kind of failure it is.
//!
//! # The configuration
//!
//! `fixtures/prbt.urdf`'s `prbt_base_link` collision cylinder is `length
//! 0.13` at `origin z 0.065`, so its bottom face is at exactly `z = 0.0`.
//! `tools/moveit-diff`'s floor box is `4 x 4 x 0.1` centred at `z = -0.05`, so
//! its top face is at exactly `z = 0.0`. Both coordinates are exact in binary,
//! the base link is fixed to the world, and no joint value moves it -- so
//! every one of Phase 3's 10,000 sampled states presents the identical
//! exactly-tangent pair, gap exactly zero. That is why the clause fails at a
//! *rate* (68.54%) rather than on scattered states: one degenerate pair,
//! resampled.
//!
//! # What the oracle does there (measured at `e017c91ee`, seed-free)
//!
//! Sweeping the floor's top face through the tie:
//!
//! | floor top `z` | true gap | `robot_collision` | `robot_distance` |
//! |---|---|---|---|
//! | `-1e-3`  | `+1e-3`  | `false` | `+1.000000000000001e-3` |
//! | `-1e-7`  | `+1e-7`  | `false` | `+1.000000000028756e-7` |
//! | `-3e-8`  | `+3e-8`  | `false` | `+2.999999999808711e-8` |
//! | `-1e-9`  | `+1e-9`  | `false` | `+9.999999994736568e-10` |
//! | `-1e-15` | `+1e-15` | `false` | `+1.038912551220369e-15` |
//! | `0`      | `0`      | `false` | **`-1.000000000000000e0`** |
//! | `+1e-15` | `-1e-15` | `true`  | `-1.129411566063279e-15` |
//! | `+1e-9`  | `-1e-9`  | `true`  | `-9.999999994737827e-10` |
//! | `+1e-3`  | `-1e-3`  | `true`  | `-1.000000000000001e-3` |
//!
//! The `distance` column is a defect and is recorded as one: continuous at
//! `~1e-15` on both sides, `-1.0` at the single point between them, a `1e15`-fold
//! discontinuity where the function is otherwise smooth. That is
//! `doc/upstream-bugs.md`'s `fcl-distance-sentinel-survives-zero-contacts`, and
//! this table is its reproducer.
//!
//! # The `bool` column is not a convention the port can adopt
//!
//! Read alone, that column looks like a rule -- touching is not colliding,
//! monotone across the boundary. It is not one. `fcl::collide` dispatches per
//! shape pair, and this workspace's two exactly-touching fixtures come back
//! opposite ways:
//!
//! | pair | gap | oracle `collision` | oracle `distance` |
//! |---|---|---|---|
//! | prbt cylinder on a box (table above) | exactly `0` | `false` | `-1.0` |
//! | octree leaf face on a box face (`octree_world_collision_response.json` case 4) | exactly `0` | `true` | `-0.0` |
//!
//! Both are exact face-on-face contact and upstream answers each differently.
//! The `-1.0` is the tell: prbt's pair reaches `fcl::distance`'s sentinel
//! because `fcl::collide` found *zero* contacts there, while the octree pair
//! found one and reported its (negative-zero) depth. So there is no single
//! upstream answer at exact contact to match -- there is a per-narrowphase
//! outcome.
//!
//! # Consequence for Phase 3
//!
//! No tolerance closes the prbt `bool` clause: `bool` has no tolerance. Nor
//! does adopting a convention, since upstream has none here and this backend's
//! answer at the tie is set by its own rounding
//! ([`the_tie_is_decided_below_one_ulp`]). Moving `fixtures/prbt.urdf` or the
//! floor would turn the clause green by deleting the configuration that
//! measures it.

use std::fs;
use std::sync::Arc;

use moveit_collision::{
    AllowedCollisionMatrix, CollisionEnv, CollisionRequest, DistanceRequest, LinkPaddingScale,
    ParryCollisionEnv, World,
};
use moveit_geometry::{Cuboid, Isometry3, Shape};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;

const FLOOR_THICKNESS: f64 = 0.1;

/// Offsets applied to the floor's top face, straddling the exact tie. The
/// `1e-15` pair is the tightest offset that still resolves cleanly on both
/// sides; `0.0` is the configuration Phase 3's sweep actually presents.
const TOP_OFFSETS: [f64; 5] = [-1e-9, -1e-15, 0.0, 1e-15, 1e-9];

fn build_prbt() -> RobotModel {
    let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/prbt.urdf");
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/prbt.srdf");
    let urdf_xml = fs::read_to_string(urdf_path).expect("fixture URDF must be readable");
    let urdf = urdf_rs::read_file(urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    // prbt's collision geometry is entirely primitives -- no mesh search path
    // is needed, and passing none proves that rather than assuming it.
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

fn build_acm() -> AllowedCollisionMatrix {
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/prbt.srdf");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    AllowedCollisionMatrix::from_srdf(&srdf)
}

fn floor_env(top_z: f64) -> ParryCollisionEnv {
    let mut world = World::new();
    world.add_shape(
        "floor",
        Arc::new(Shape::Cuboid(
            Cuboid::new(4.0, 4.0, FLOOR_THICKNESS).expect("positive cuboid dimensions"),
        )),
        Isometry3::translation(0.0, 0.0, top_z - FLOOR_THICKNESS / 2.0),
    );
    ParryCollisionEnv::new(world, LinkPaddingScale::default())
}

/// `(robot_collision, robot_distance)` for prbt's default state against a floor
/// whose top face sits at `top_z`. A negative `top_z` leaves a gap of `-top_z`.
fn measure(top_z: f64) -> (bool, f64) {
    let model = build_prbt();
    let acm = build_acm();
    let env = floor_env(top_z);
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();

    let collision = env
        .check_robot_collision(&CollisionRequest::default(), &posed, &[], Some(&acm))
        .collision;
    let distance = env
        .distance_robot(
            &DistanceRequest {
                enable_signed_distance: true,
                acm: Some(&acm),
                ..DistanceRequest::default()
            },
            &posed,
            &[],
        )
        .minimum_distance
        .distance;
    (collision, distance)
}

/// The fixture geometry the whole file rests on: the tie is real, not
/// approximate. If either coordinate ever drifts, every test below would still
/// pass while measuring an ordinary gap or overlap instead of a tie.
#[test]
fn the_base_link_and_the_floor_are_exactly_tangent() {
    let model = build_prbt();
    let link = model
        .link_model("prbt_base_link")
        .expect("prbt_base_link must resolve");
    let shapes = link.shapes();
    assert_eq!(shapes.len(), 1, "prbt_base_link has one collision shape");
    let Shape::Cylinder(cylinder) = &shapes[0].shape else {
        panic!("prbt_base_link's collision geometry is a cylinder");
    };
    let centre_z = shapes[0].origin_transform.translation.vector.z;
    let bottom = centre_z - cylinder.length / 2.0;
    assert_eq!(
        bottom, 0.0,
        "prbt_base_link's cylinder bottom face must be exactly z=0 (centre {centre_z}, length {})",
        cylinder.length
    );
    // And the floor's top face, built the same way `collision_scene` builds it.
    let floor_top = -0.05 + FLOOR_THICKNESS / 2.0;
    assert_eq!(floor_top, 0.0, "the floor's top face must be exactly z=0");
}

/// Well clear of the boundary the two backends agree, so the prbt disagreement
/// is confined to the tie and this backend's margin rather than being a general
/// defect in its world-collision path.
///
/// Oracle at these two offsets, from the module doc's table: `false` /
/// `+1.000000000000001e-3` and `true` / `-1.000000000000001e-3`.
#[test]
fn a_millimetre_either_side_matches_the_oracle() {
    let (gap_collides, gap_distance) = measure(-1e-3);
    assert!(!gap_collides, "a 1mm gap must not count as a collision");
    assert!(
        (gap_distance - 1.000_000_000_000_001e-3).abs() < 1e-12,
        "a 1mm gap must match the oracle's +1.000000000000001e-3, got {gap_distance}"
    );

    let (overlap_collides, overlap_distance) = measure(1e-3);
    assert!(overlap_collides, "a 1mm overlap must count as a collision");
    assert!(
        (overlap_distance + 1.000_000_000_000_001e-3).abs() < 1e-12,
        "a 1mm overlap must match the oracle's -1.000000000000001e-3, got {overlap_distance}"
    );
}

/// The distance stays continuous through the tie -- this backend produces no
/// sentinel, which is the whole of its deviation from
/// `fcl-distance-sentinel-survives-zero-contacts`.
///
/// Bounds the magnitude rather than pinning a literal: the exact rounding at
/// the tie is not a contract, but "stays within a nanometre of zero" is, and
/// it is what upstream's `-1.0` violates by nine orders of magnitude.
#[test]
fn no_sentinel_escapes_at_the_tie() {
    for offset in TOP_OFFSETS {
        let (_, distance) = measure(offset);
        assert!(
            distance.abs() < 1e-8,
            "floor top {offset:e}: distance {distance} is not within 1e-8 of zero -- upstream \
             reports -1.0 at offset 0.0, and this test exists to catch this backend acquiring \
             the same sentinel"
        );
    }
}

/// The magnitude behind "the tie is decided below one ulp", measured rather
/// than asserted, so §218's claim that no tolerance closes the prbt `bool`
/// clause rests on a number.
///
/// The true gap at offset `0.0` is exactly zero. Whatever this backend reports
/// there is its own rounding in composing the link pose; pinning that it is
/// tiny (rather than pinning its value) is what makes this a statement about
/// the arithmetic instead of a snapshot of one parry release.
#[test]
fn the_tie_is_decided_below_one_ulp() {
    let (collides, distance) = measure(0.0);
    assert!(
        distance != 0.0 && distance.abs() < 1e-15,
        "at the exact tie this backend reports {distance}; the claim that the prbt bool \
         disagreement is a sub-ulp tie rather than a modelling error needs it nonzero and far \
         below any tolerance either backend could agree on"
    );
    assert!(
        collides,
        "the tie falls on the colliding side, which is *why* prbt's bool clause disagrees with \
         the oracle's false -- if this flips, §218's 6,854/10,000 is stale"
    );
}
