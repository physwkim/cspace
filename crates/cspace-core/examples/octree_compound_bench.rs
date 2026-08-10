// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Measures `parry3d_f64::shape::Compound::new` construction time for
//! `compound_from_octree` on a room-scale scene, at both resolutions
//! PORTING-PLAN.md §4.8 measured leaf/cell expansion ratios on (0.05 m and
//! 0.02 m) -- the two numbers §4.8 says would give option 1 (a hand-written
//! depth-adaptive `parry` shape) a basis, or retire it.
//!
//! The scene: a 4x4x2.4m room (floor, ceiling, four walls, each one cell
//! thick) with a 0.6m cube table resting on the floor, its remaining
//! interior filled with free space at finest resolution, then pruned --
//! matching PORTING-PLAN.md §4.8's prose description of the scene the
//! round-3 expansion-ratio measurement used. That measurement's own scene
//! generator was a throwaway, never committed (nothing under this crate's
//! history adds it) -- this is a reconstruction from the spec recorded in
//! §4.8's prose, not a byte-for-byte replay, so it is run once here and its
//! own leaf/cell counts are printed alongside the timing so a reader can
//! judge how closely it lands (see this change's commit body for the
//! comparison against round 3's 46,978/305,989 leaf counts).
//!
//! Run with `cargo run --release --example octree_compound_bench -p
//! cspace-geometry`. Debug builds are unusably slow for the 0.02m scene (on
//! the order of 5 million individual octree updates).

use std::time::Instant;

use cspace_core::octomap::OcTree;
use nalgebra::Point3;
use parry3d_f64::math::{Pose, Vector as ParryVector};
use parry3d_f64::shape::{Compound, Cuboid, SharedShape};

const ROOM_X: f64 = 4.0;
const ROOM_Y: f64 = 4.0;
const ROOM_Z: f64 = 2.4;
const TABLE_SIZE: f64 = 0.6;
/// Table's min corner, resting on the floor, off-center so it does not sit
/// symmetrically astride any wall.
const TABLE_MIN: [f64; 3] = [1.7, 1.7, 0.0];

fn in_table(p: [f64; 3]) -> bool {
    (0..3).all(|i| p[i] >= TABLE_MIN[i] && p[i] < TABLE_MIN[i] + TABLE_SIZE)
}

fn in_wall(p: [f64; 3], resolution: f64) -> bool {
    let [x, y, z] = p;
    x < resolution
        || x > ROOM_X - resolution
        || y < resolution
        || y > ROOM_Y - resolution
        || z < resolution
        || z > ROOM_Z - resolution
}

/// Builds the room-scale scene at `resolution`: walls, floor and ceiling
/// occupied; the table block occupied; every other interior cell explicitly
/// free at finest resolution -- then a single `update_inner_occupancy` +
/// `prune` pass, matching how a real batch sensor integration would be
/// followed by a maintenance prune rather than pruning after every insert.
fn build_room_scene(resolution: f64) -> OcTree {
    let mut tree = OcTree::new(resolution);
    let half = resolution / 2.0;
    let nx = (ROOM_X / resolution).round() as i64;
    let ny = (ROOM_Y / resolution).round() as i64;
    let nz = (ROOM_Z / resolution).round() as i64;

    for xi in 0..nx {
        for yi in 0..ny {
            for zi in 0..nz {
                let p = [
                    half + resolution * xi as f64,
                    half + resolution * yi as f64,
                    half + resolution * zi as f64,
                ];
                let occupied = in_wall(p, resolution) || in_table(p);
                tree.update_node(Point3::from(p), occupied, true);
            }
        }
    }
    tree.update_inner_occupancy();
    tree.prune();
    tree
}

/// `8^(finest_depth - leaf_depth)`, matching PORTING-PLAN.md §4.8's own
/// definition of the cell count a single-resolution `Voxels` shape would
/// need to represent the same leaf.
fn finest_resolution_cell_count(tree: &OcTree) -> u64 {
    tree.leaves()
        .map(|leaf| 8u64.pow(OcTree::TREE_DEPTH - leaf.depth()))
        .sum()
}

fn bench_one(resolution: f64) {
    let build_start = Instant::now();
    let tree = build_room_scene(resolution);
    let build_elapsed = build_start.elapsed();

    let leaf_count = tree.leaves().count();
    let occupied_leaf_count = tree
        .leaves()
        .filter(cspace_core::octomap::Leaf::is_occupied)
        .count();
    let finest_cells = finest_resolution_cell_count(&tree);

    println!("resolution {resolution}m:");
    println!("  scene build (update_node x{leaf_count}-ish + prune): {build_elapsed:?}");
    println!("  leaves total: {leaf_count}, occupied: {occupied_leaf_count}");
    println!(
        "  finest-resolution-equivalent cells: {finest_cells} (x{:.2} vs actual leaf count)",
        finest_cells as f64 / leaf_count as f64
    );

    // Mirrors `compound_from_octree`'s own leaf-to-Cuboid mapping, but keeps
    // it out of the timed section: the task asks for `Compound::new`'s own
    // construction (BVH build) cost specifically, not the cost of walking
    // the tree's leaves, which a real sensor-update path would do anyway to
    // find what changed.
    let leaf_shapes: Vec<(Pose, SharedShape)> = tree
        .leaves()
        .filter(cspace_core::octomap::Leaf::is_occupied)
        .map(|leaf| {
            let half_extent = leaf.size() / 2.0;
            let cuboid = Cuboid::new(ParryVector::new(half_extent, half_extent, half_extent));
            let center = leaf.coordinate();
            let pose = cspace_core::geometry::Isometry3::translation(center.x, center.y, center.z);
            (pose.into(), SharedShape::new(cuboid))
        })
        .collect();

    const ITERS: u32 = 10;
    let mut total = std::time::Duration::ZERO;
    let mut last_len = 0usize;
    for _ in 0..ITERS {
        let compound_start = Instant::now();
        let compound = Compound::new(leaf_shapes.clone());
        total += compound_start.elapsed();
        last_len = compound.shapes().len();
    }
    println!(
        "  Compound::new over {last_len} occupied-leaf Cuboids: {:?} average over {ITERS} runs (total {:?})",
        total / ITERS,
        total
    );
    println!();
}

fn main() {
    bench_one(0.05);
    bench_one(0.02);
}
