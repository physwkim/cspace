// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Leaf-`Cuboid` [`parry3d_f64::shape::Compound`] approximation of an
//! octree collision shape.
//!
//! `geometric_shapes` has no equivalent of this module: upstream never
//! decomposes an octree into a `parry`-style shape hierarchy, because each
//! of upstream's collision backends carries its own native octree wrap
//! instead (`fcl::OcTreed` in `collision_detection_fcl`,
//! `PosedBodyPointDecomposition` in `collision_env_distance_field`). `parry`
//! itself has no multi-resolution octree shape (PORTING-PLAN.md §4.8's
//! investigation), so [`compound_from_octree`] is new code this port
//! introduces under decision D4's sum-type shape layer, not a port of any
//! upstream function.
//!
//! # Why a `Compound` of leaf `Cuboid`s
//!
//! Decision recorded in PORTING-PLAN.md §4.8: one [`Cuboid`] per *occupied*
//! leaf, sized and posed at that leaf's own depth (never expanded to the
//! tree's finest resolution), keeps FCL's real traversal gain intact —
//! `Compound` builds its own BVH over the leaf shapes, so one AABB overlap
//! test on a coarse leaf's `Cuboid` prunes everything that leaf covers,
//! exactly as `fcl::OcTreed`'s own depth-first descent does, without
//! hand-writing that descent or a `parry` `Shape`/`QueryDispatcher`
//! implementation (the alternative the same section rejected for now).
//!
//! Only *occupied* leaves become `Cuboid`s — matching
//! `octomap::AbstractOccupancyOcTree::isNodeOccupied`
//! ([`cspace_octomap::Leaf::is_occupied`]), the same predicate FCL's own
//! octree collision traversal gates on. A free or unknown leaf, no matter
//! how coarse, carries no obstacle and must not appear in the `Compound`, or
//! this approximation would report collisions FCL does not.

use cspace_octomap::OcTree;
use parry3d_f64::shape::{Compound, Cuboid, SharedShape};

use crate::Isometry3;

/// Builds a leaf-`Cuboid` [`Compound`] approximating `tree`'s occupied
/// space, or `None` if `tree` has no occupied leaves.
///
/// `Compound::new` panics on an empty shape list; this guards that
/// precondition itself so a caller with an all-free (or brand new) octree
/// gets `None` — the same "no shape" convention
/// [`crate::shapes::OcTree::octree`] already uses for "no tree at all" —
/// rather than a panic.
pub fn compound_from_octree(tree: &OcTree) -> Option<Compound> {
    let leaf_shapes: Vec<(Isometry3, SharedShape)> = tree
        .leaves()
        .filter(cspace_octomap::Leaf::is_occupied)
        .map(|leaf| {
            let half_extent = leaf.size() / 2.0;
            let cuboid = Cuboid::new(parry3d_f64::math::Vector::new(
                half_extent,
                half_extent,
                half_extent,
            ));
            let center = leaf.coordinate();
            let pose = Isometry3::translation(center.x, center.y, center.z);
            (pose, SharedShape::new(cuboid))
        })
        .collect();

    if leaf_shapes.is_empty() {
        return None;
    }

    Some(Compound::new(
        leaf_shapes
            .into_iter()
            .map(|(pose, shape)| (pose.into(), shape))
            .collect(),
    ))
}

#[cfg(test)]
mod tests {
    use nalgebra::Point3;

    use super::*;

    #[test]
    fn a_single_leaf_bounding_box_lands_exactly_on_the_leafs_own_corner() {
        // The tree's own bounding box IS this single leaf's box: the
        // Compound's local_aabb must land exactly on the leaf's own corner,
        // neither clipped short of it (an off-by-one in the half-extent)
        // nor padded past it.
        //
        // Bit-exact, not approximate: `assert_relative_eq!(..., epsilon =
        // 1e-9)` bisected to `epsilon = 0.0, max_relative = 0.0` and still
        // passed (round 14, §79 sweep) -- for this test's literal 0.1
        // resolution and 0.05 point, `half_extent = leaf.size() / 2.0` is an
        // exact halving (no rounding, per IEEE 754) of the same `0.1`
        // literal `node_size` returns at the finest depth, and
        // `leaf.coordinate() +/- half_extent` reproduces `0.0`/`0.1` bit for
        // bit. `assert_relative_eq!` was silently testing to a tolerance
        // nothing in this computation ever approaches.
        let mut tree = OcTree::new(0.1);
        tree.update_node(Point3::new(0.05, 0.05, 0.05), true, false);
        let compound = compound_from_octree(&tree).expect("one occupied leaf");
        let aabb = compound.local_aabb();
        assert_eq!(aabb.mins.x, 0.0);
        assert_eq!(aabb.mins.y, 0.0);
        assert_eq!(aabb.mins.z, 0.0);
        assert_eq!(aabb.maxs.x, 0.1);
        assert_eq!(aabb.maxs.y, 0.1);
        assert_eq!(aabb.maxs.z, 0.1);
    }

    // Assertion-discrimination sweep (round 2): `compound_from_octree`
    // has exactly one `None`-producing site (`if leaf_shapes.is_empty()
    // { return None; }`) -- verdict `single-branch`, established by
    // reading the whole function body (lines 48-74): the only other
    // return is the unconditional trailing `Some(..)`. Both tests below
    // share that verdict.
    #[test]
    fn empty_tree_has_no_occupied_leaves() {
        let tree = OcTree::new(0.1);
        assert!(compound_from_octree(&tree).is_none());
    }

    #[test]
    fn all_free_tree_has_no_occupied_leaves() {
        let mut tree = OcTree::new(0.1);
        tree.update_node(Point3::new(1.0, 1.0, 1.0), false, false);
        assert!(compound_from_octree(&tree).is_none());
    }

    #[test]
    fn one_occupied_leaf_becomes_one_cuboid_at_finest_resolution() {
        let mut tree = OcTree::new(0.1);
        tree.update_node(Point3::new(0.05, 0.05, 0.05), true, false);
        let compound = compound_from_octree(&tree).expect("one occupied leaf");
        assert_eq!(compound.shapes().len(), 1);
        let (pose, shape) = &compound.shapes()[0];
        let cuboid = shape.as_cuboid().expect("shape is a Cuboid");
        assert!((cuboid.half_extents.x - 0.05).abs() < 1e-12);
        assert!((pose.translation.x - 0.05).abs() < 1e-9);
    }

    #[test]
    fn a_pruned_solid_block_becomes_one_coarse_cuboid_not_many_fine_ones() {
        // An 8x8x8 block of occupied 0.1m cells, all sharing the same
        // occupancy value, prunes into a single leaf at depth
        // TREE_DEPTH - 3 (one Cuboid of size 0.8m), not 512 finest-resolution
        // leaves -- exercising the same coarse-leaf collapse the
        // "against a pruned-away subtree" oracle boundary case checks.
        let mut tree = OcTree::new(0.1);
        for xi in 0..8 {
            for yi in 0..8 {
                for zi in 0..8 {
                    let p = Point3::new(
                        0.05 + 0.1 * f64::from(xi),
                        0.05 + 0.1 * f64::from(yi),
                        0.05 + 0.1 * f64::from(zi),
                    );
                    tree.update_node(p, true, true);
                }
            }
        }
        tree.update_inner_occupancy();
        tree.prune();

        let compound = compound_from_octree(&tree).expect("occupied block");
        assert_eq!(compound.shapes().len(), 1);
        let (_, shape) = &compound.shapes()[0];
        let cuboid = shape.as_cuboid().expect("shape is a Cuboid");
        assert!((cuboid.half_extents.x - 0.4).abs() < 1e-9);
    }

    #[test]
    fn local_aabb_covers_every_occupied_leaf() {
        let mut tree = OcTree::new(0.1);
        tree.update_node(Point3::new(0.05, 0.05, 0.05), true, false);
        tree.update_node(Point3::new(5.05, 5.05, 5.05), true, false);
        let compound = compound_from_octree(&tree).expect("two occupied leaves");
        let aabb = compound.local_aabb();
        assert!(aabb.mins.x <= 0.0 && aabb.maxs.x >= 5.1);
    }
}
