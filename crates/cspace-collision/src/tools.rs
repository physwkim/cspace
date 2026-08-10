// Copyright (c) 2011, Willow Garage, Inc.
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection/src/collision_tools.cpp

//! Pure computations over [`CostSource`] sets.
//!
//! Upstream `collision_tools.cpp` is mostly ROS message/visualization
//! conversions (`getCostMarkers`, `getCollisionMarkersFromContacts`,
//! `costSourceToMsg`, `contactToMsg`, all building
//! `visualization_msgs`/`moveit_msgs` types) — out of scope for a
//! ROS-independent core crate (PORTING-PLAN.md D1); that conversion layer
//! belongs in the optional `cspace-ros` crate, once one exists to convert
//! into. What is ported here is the pure geometric/numeric logic that upstream
//! happens to define in the same file: [`total_cost`], [`intersect_cost_sources`],
//! [`remove_overlapping`], [`remove_cost_sources`] and [`sensor_positioning`].
//!
//! `getSensorPositioning`'s `geometry_msgs::msg::Point` out-param becomes a
//! plain [`Vector3`] return value here, for the same D1 reason.

use std::collections::BTreeSet;

use cspace_core::geometry::Vector3;

use crate::common::CostSource;
use crate::numeric::{cxx_max, cxx_min};

/// `getTotalCost`: the sum of each cost source's cost density times its AABB
/// volume.
pub fn total_cost(cost_sources: &BTreeSet<CostSource>) -> f64 {
    cost_sources.iter().map(|c| c.cost * c.volume()).sum()
}

/// `getSensorPositioning`: a point near the 80th-percentile cost source in
/// `cost_sources`'s upstream `operator<` order (most-costly-first), or `None`
/// if `cost_sources` is empty.
///
/// Upstream computes `4 * cost_sources.size() / 5` (integer division) steps
/// from `begin()` and returns the AABB center there; the index — not the
/// specific box chosen — is the point of the function (biasing sensor
/// placement toward, but not exactly at, the costliest region).
pub fn sensor_positioning(cost_sources: &BTreeSet<CostSource>) -> Option<Vector3> {
    let index = 4 * cost_sources.len() / 5;
    let source = cost_sources.iter().nth(index)?;
    Some(aabb_center(source))
}

fn aabb_center(source: &CostSource) -> Vector3 {
    Vector3::new(
        (source.aabb_max[0] + source.aabb_min[0]) / 2.0,
        (source.aabb_max[1] + source.aabb_min[1]) / 2.0,
        (source.aabb_max[2] + source.aabb_min[2]) / 2.0,
    )
}

fn aabb_intersection(a: &CostSource, b: &CostSource) -> Option<([f64; 3], [f64; 3])> {
    // `std::max(source_a.aabb_min[i], source_b.aabb_min[i])` /
    // `std::min(source_a.aabb_max[i], source_b.aabb_max[i])`
    // (`collision_tools.cpp:170-176`) keep a NaN `a` bound as NaN; `f64::max`/
    // `f64::min` would silently discard it in favor of `b`'s finite bound.
    // See `crate::numeric`.
    let min = [
        cxx_max(a.aabb_min[0], b.aabb_min[0]),
        cxx_max(a.aabb_min[1], b.aabb_min[1]),
        cxx_max(a.aabb_min[2], b.aabb_min[2]),
    ];
    let max = [
        cxx_min(a.aabb_max[0], b.aabb_max[0]),
        cxx_min(a.aabb_max[1], b.aabb_max[1]),
        cxx_min(a.aabb_max[2], b.aabb_max[2]),
    ];
    if min[0] >= max[0] || min[1] >= max[1] || min[2] >= max[2] {
        None
    } else {
        Some((min, max))
    }
}

fn aabb_volume(min: [f64; 3], max: [f64; 3]) -> f64 {
    (max[0] - min[0]) * (max[1] - min[1]) * (max[2] - min[2])
}

/// `intersectCostSources`: every pairwise intersection of a box from `a` with
/// a box from `b`, each costed at `max(source_a.cost, source_b.cost)`. Pairs
/// whose AABBs do not overlap contribute nothing (matching upstream's
/// `continue` on a zero- or negative-volume intersection, which excludes
/// boxes that merely touch at a boundary as well as disjoint ones).
pub fn intersect_cost_sources(
    a: &BTreeSet<CostSource>,
    b: &BTreeSet<CostSource>,
) -> BTreeSet<CostSource> {
    let mut result = BTreeSet::new();
    for source_a in a {
        for source_b in b {
            let Some((aabb_min, aabb_max)) = aabb_intersection(source_a, source_b) else {
                continue;
            };
            result.insert(CostSource {
                aabb_min,
                aabb_max,
                // `std::max(source_a.cost, source_b.cost)`
                // (`collision_tools.cpp:181`); same NaN-first-operand-wins
                // rule as `aabb_intersection` above.
                cost: cxx_max(source_a.cost, source_b.cost),
            });
        }
    }
    result
}

/// `removeOverlapping`: drop any cost source whose intersection with an
/// earlier (in `operator<` order) one covers at least `overlap_fraction` of
/// its own volume.
///
/// A removed source never goes on to remove anything itself: upstream erases
/// it from the set before the outer iterator can reach it, so it is gone as a
/// remover and not only as a result. That asymmetry is load-bearing, and it
/// is why removal is expressed here by erasing from the set being built
/// rather than by a parallel `removed` flag array: a flag the outer loop
/// forgets to consult silently turns a dropped box into a remover, dropping a
/// box upstream keeps (see
/// `remove_overlapping_does_not_let_a_removed_source_remove_another`).
///
/// Iterating an ordered snapshot rather than the live set is safe because the
/// only mutation is removal, and the outer loop re-checks liveness.
pub fn remove_overlapping(
    cost_sources: &BTreeSet<CostSource>,
    overlap_fraction: f64,
) -> BTreeSet<CostSource> {
    let ordered: Vec<CostSource> = cost_sources.iter().copied().collect();
    let mut live = cost_sources.clone();
    for (i, source) in ordered.iter().enumerate() {
        if !live.contains(source) {
            continue;
        }
        let threshold = source.volume() * overlap_fraction;
        for other in &ordered[i + 1..] {
            let Some((min, max)) = aabb_intersection(source, other) else {
                continue;
            };
            if aabb_volume(min, max) >= threshold {
                live.remove(other);
            }
        }
    }
    live
}

/// `removeCostSources`: for each source in `cost_sources_to_remove`, drop any
/// `cost_sources` box overlapping it by at least `overlap_fraction` of the
/// box's own volume.
///
/// A box overlapping by less is not dropped: upstream inserts the parts of
/// it that fall outside the intersection (one per axis where the box extends
/// past the intersection bound) *in addition to*, not instead of, the
/// original box — the original stays in the result even though it still
/// overlaps `source_remove`. That is upstream's literal behavior (see
/// `collision_tools.cpp`, the `else` branch never pushes `it` onto its own
/// `remove` list), reproduced here as-is rather than "fixed", since nothing
/// in this port's scope calls for a semantic change to it.
pub fn remove_cost_sources(
    cost_sources: &BTreeSet<CostSource>,
    cost_sources_to_remove: &BTreeSet<CostSource>,
    overlap_fraction: f64,
) -> BTreeSet<CostSource> {
    let mut cost_sources = cost_sources.clone();
    for source_remove in cost_sources_to_remove {
        let mut remove = Vec::new();
        let mut add = BTreeSet::new();
        for source in &cost_sources {
            let Some((min, max)) = aabb_intersection(source, source_remove) else {
                continue;
            };
            if aabb_volume(min, max) >= source.volume() * overlap_fraction {
                remove.push(*source);
                continue;
            }
            for axis in 0..3 {
                if source.aabb_max[axis] > max[axis] {
                    let mut split = *source;
                    split.aabb_min[axis] = max[axis];
                    add.insert(split);
                }
                if source.aabb_min[axis] < min[axis] {
                    let mut split = *source;
                    split.aabb_max[axis] = min[axis];
                    add.insert(split);
                }
            }
        }
        for source in remove {
            cost_sources.remove(&source);
        }
        cost_sources.extend(add);
    }
    cost_sources
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cost_source(aabb_min: [f64; 3], aabb_max: [f64; 3], cost: f64) -> CostSource {
        CostSource {
            aabb_min,
            aabb_max,
            cost,
        }
    }

    #[test]
    fn total_cost_of_empty_set_is_zero() {
        assert_eq!(total_cost(&BTreeSet::new()), 0.0);
    }

    #[test]
    fn total_cost_sums_cost_times_volume() {
        let mut sources = BTreeSet::new();
        sources.insert(cost_source([0.0, 0.0, 0.0], [1.0, 1.0, 1.0], 2.0));
        sources.insert(cost_source([0.0, 0.0, 0.0], [2.0, 1.0, 1.0], 0.5));
        assert_eq!(total_cost(&sources), 2.0 * 1.0 + 0.5 * 2.0);
    }

    #[test]
    fn sensor_positioning_of_empty_set_is_none() {
        assert!(sensor_positioning(&BTreeSet::new()).is_none());
    }

    #[test]
    fn sensor_positioning_returns_aabb_center_at_the_documented_index() {
        let mut sources = BTreeSet::new();
        for i in 0..5 {
            sources.insert(cost_source(
                [i as f64, 0.0, 0.0],
                [i as f64 + 1.0, 1.0, 1.0],
                5.0 - i as f64,
            ));
        }
        // 4 * 5 / 5 == 4: the 5th element in most-costly-first order, i.e.
        // the least costly (i == 4) since cost descends with i.
        let expected = aabb_center(&cost_source([4.0, 0.0, 0.0], [5.0, 1.0, 1.0], 1.0));
        assert_eq!(sensor_positioning(&sources), Some(expected));
    }

    #[test]
    fn intersecting_boxes_produce_the_max_cost_intersection() {
        let mut a = BTreeSet::new();
        a.insert(cost_source([0.0, 0.0, 0.0], [2.0, 2.0, 2.0], 1.0));
        let mut b = BTreeSet::new();
        b.insert(cost_source([1.0, 1.0, 1.0], [3.0, 3.0, 3.0], 2.0));

        let result = intersect_cost_sources(&a, &b);
        assert_eq!(result.len(), 1);
        let only = result.iter().next().unwrap();
        assert_eq!(only.aabb_min, [1.0, 1.0, 1.0]);
        assert_eq!(only.aabb_max, [2.0, 2.0, 2.0]);
        assert_eq!(only.cost, 2.0);
    }

    #[test]
    fn boxes_that_only_touch_do_not_intersect() {
        let mut a = BTreeSet::new();
        a.insert(cost_source([0.0, 0.0, 0.0], [1.0, 1.0, 1.0], 1.0));
        let mut b = BTreeSet::new();
        b.insert(cost_source([1.0, 0.0, 0.0], [2.0, 1.0, 1.0], 1.0));
        assert!(intersect_cost_sources(&a, &b).is_empty());
    }

    #[test]
    fn boxes_that_only_touch_on_y_do_not_intersect() {
        // Overlaps on x and z, touches only at y == 1: isolates the
        // `min[1] >= max[1]` operand the way the sibling test above
        // isolates `min[0] >= max[0]`.
        let mut a = BTreeSet::new();
        a.insert(cost_source([0.0, 0.0, 0.0], [1.0, 1.0, 1.0], 1.0));
        let mut b = BTreeSet::new();
        b.insert(cost_source([0.0, 1.0, 0.0], [1.0, 2.0, 1.0], 1.0));
        assert!(intersect_cost_sources(&a, &b).is_empty());
    }

    #[test]
    fn boxes_that_only_touch_on_z_do_not_intersect() {
        // Overlaps on x and y, touches only at z == 1: isolates the
        // `min[2] >= max[2]` operand the way the sibling tests above
        // isolate `min[0] >= max[0]` and `min[1] >= max[1]`.
        let mut a = BTreeSet::new();
        a.insert(cost_source([0.0, 0.0, 0.0], [1.0, 1.0, 1.0], 1.0));
        let mut b = BTreeSet::new();
        b.insert(cost_source([0.0, 0.0, 1.0], [1.0, 1.0, 2.0], 1.0));
        assert!(intersect_cost_sources(&a, &b).is_empty());
    }

    #[test]
    fn disjoint_boxes_do_not_intersect() {
        let mut a = BTreeSet::new();
        a.insert(cost_source([0.0, 0.0, 0.0], [1.0, 1.0, 1.0], 1.0));
        let mut b = BTreeSet::new();
        b.insert(cost_source([5.0, 5.0, 5.0], [6.0, 6.0, 6.0], 1.0));
        assert!(intersect_cost_sources(&a, &b).is_empty());
    }

    // `std::max`/`std::min` return a NaN *first* operand and discard a NaN
    // *second* one; `f64::max`/`f64::min` discard NaN wherever it sits. The
    // pairs below put the NaN on `a` (the diverging position) to prove the
    // fix, then on `b` (the non-diverging position) to prove the fix didn't
    // just start propagating NaN everywhere.

    #[test]
    fn aabb_min_nan_on_a_is_returned_not_discarded() {
        let a = cost_source([f64::NAN, 0.0, 0.0], [2.0, 2.0, 2.0], 1.0);
        let b = cost_source([1.0, 1.0, 1.0], [3.0, 3.0, 3.0], 1.0);
        let (min, _) = aabb_intersection(&a, &b).expect("boxes overlap on y/z");
        assert!(min[0].is_nan());
    }

    #[test]
    fn aabb_min_nan_on_b_is_discarded_in_favor_of_a() {
        let a = cost_source([1.0, 0.0, 0.0], [2.0, 2.0, 2.0], 1.0);
        let b = cost_source([f64::NAN, 1.0, 1.0], [3.0, 3.0, 3.0], 1.0);
        let (min, _) = aabb_intersection(&a, &b).expect("boxes overlap on y/z");
        assert_eq!(min[0], 1.0);
    }

    #[test]
    fn aabb_max_nan_on_a_is_returned_not_discarded() {
        let a = cost_source([0.0, 0.0, 0.0], [f64::NAN, 2.0, 2.0], 1.0);
        let b = cost_source([1.0, 1.0, 1.0], [3.0, 3.0, 3.0], 1.0);
        let (_, max) = aabb_intersection(&a, &b).expect("boxes overlap on y/z");
        assert!(max[0].is_nan());
    }

    #[test]
    fn aabb_max_nan_on_b_is_discarded_in_favor_of_a() {
        let a = cost_source([0.0, 0.0, 0.0], [2.0, 2.0, 2.0], 1.0);
        let b = cost_source([1.0, 1.0, 1.0], [f64::NAN, 3.0, 3.0], 1.0);
        let (_, max) = aabb_intersection(&a, &b).expect("boxes overlap on y/z");
        assert_eq!(max[0], 2.0);
    }

    #[test]
    fn intersection_cost_nan_on_a_is_returned_not_discarded() {
        let mut a = BTreeSet::new();
        a.insert(cost_source([0.0, 0.0, 0.0], [2.0, 2.0, 2.0], f64::NAN));
        let mut b = BTreeSet::new();
        b.insert(cost_source([1.0, 1.0, 1.0], [3.0, 3.0, 3.0], 1.0));
        let result = intersect_cost_sources(&a, &b);
        assert_eq!(result.len(), 1);
        assert!(result.iter().next().unwrap().cost.is_nan());
    }

    #[test]
    fn intersection_cost_nan_on_b_is_discarded_in_favor_of_a() {
        let mut a = BTreeSet::new();
        a.insert(cost_source([0.0, 0.0, 0.0], [2.0, 2.0, 2.0], 5.0));
        let mut b = BTreeSet::new();
        b.insert(cost_source([1.0, 1.0, 1.0], [3.0, 3.0, 3.0], f64::NAN));
        let result = intersect_cost_sources(&a, &b);
        assert_eq!(result.len(), 1);
        assert_eq!(result.iter().next().unwrap().cost, 5.0);
    }

    #[test]
    fn remove_overlapping_drops_a_box_overlapping_at_the_threshold() {
        // Upstream's threshold is `it->getVolume() * overlap_fraction`,
        // where `it` is the earlier (more costly) box in sorted order, not
        // the box being considered for removal — same identical AABB on
        // both makes the two volumes equal so the boundary is exact.
        let mut sources = BTreeSet::new();
        sources.insert(cost_source([0.0, 0.0, 0.0], [1.0, 1.0, 1.0], 10.0));
        sources.insert(cost_source([0.0, 0.0, 0.0], [1.0, 1.0, 1.0], 1.0));

        let result = remove_overlapping(&sources, 1.0);
        assert_eq!(result.len(), 1);
        assert_eq!(result.iter().next().unwrap().cost, 10.0);
    }

    #[test]
    fn remove_overlapping_keeps_a_box_overlapping_below_the_threshold() {
        let mut sources = BTreeSet::new();
        // The earlier (more costly) box has volume 8; threshold at
        // overlap_fraction 0.5 is 4. The two boxes overlap in only a
        // 0.5x1x1 slab (volume 0.5), well below that threshold.
        sources.insert(cost_source([0.0, 0.0, 0.0], [2.0, 2.0, 2.0], 10.0));
        sources.insert(cost_source([1.5, 0.0, 0.0], [2.5, 1.0, 1.0], 1.0));

        let result = remove_overlapping(&sources, 0.5);
        assert_eq!(result.len(), 2);
    }

    #[test]
    fn remove_overlapping_does_not_let_a_removed_source_remove_another() {
        // a removes b (b covers all of a, threshold 0.5 * vol(a) = 0.5).
        // b would remove c (c covers half of b, threshold 0.5 * vol(b) = 1.0),
        // but upstream has already erased b before its outer iterator gets
        // there, so c survives. a and c do not intersect at all -- they meet
        // exactly at x = 1, and a zero-volume intersection is not an overlap.
        let a = cost_source([0.0, 0.0, 0.0], [1.0, 1.0, 1.0], 10.0);
        let b = cost_source([0.0, 0.0, 0.0], [2.0, 1.0, 1.0], 4.0);
        let c = cost_source([1.0, 0.0, 0.0], [3.0, 1.0, 1.0], 3.0);
        let sources: BTreeSet<CostSource> = [a, b, c].into_iter().collect();
        // cost * volume is 10, 8, 6: the set iterates a, b, c in that order,
        // which is what makes b an outer source after it has been removed.
        assert_eq!(
            sources.iter().copied().collect::<Vec<_>>(),
            vec![a, b, c],
            "the case depends on this iteration order"
        );

        let result = remove_overlapping(&sources, 0.5);
        assert_eq!(result, [a, c].into_iter().collect::<BTreeSet<_>>());
    }

    #[test]
    fn remove_cost_sources_drops_a_fully_overlapped_box() {
        let mut sources = BTreeSet::new();
        sources.insert(cost_source([0.0, 0.0, 0.0], [1.0, 1.0, 1.0], 1.0));
        let mut to_remove = BTreeSet::new();
        to_remove.insert(cost_source([-1.0, -1.0, -1.0], [2.0, 2.0, 2.0], 1.0));

        let result = remove_cost_sources(&sources, &to_remove, 1.0);
        assert!(result.is_empty());
    }

    #[test]
    fn remove_cost_sources_below_threshold_adds_the_remainder_but_keeps_the_original() {
        let source = cost_source([0.0, 0.0, 0.0], [2.0, 1.0, 1.0], 1.0);
        let mut sources = BTreeSet::new();
        sources.insert(source);
        let mut to_remove = BTreeSet::new();
        // Overlaps only x in [0, 1]: half the box's volume, below threshold
        // 1.0, so upstream adds the x-in-[1,2] remainder without dropping
        // the original (see this function's doc comment).
        to_remove.insert(cost_source([0.0, 0.0, 0.0], [1.0, 1.0, 1.0], 1.0));

        let result = remove_cost_sources(&sources, &to_remove, 1.0);
        assert!(result.contains(&source), "original box must survive");
        assert!(
            result.contains(&cost_source([1.0, 0.0, 0.0], [2.0, 1.0, 1.0], 1.0)),
            "remainder outside the intersection must be added"
        );
        assert_eq!(result.len(), 2);
    }

    #[test]
    fn remove_cost_sources_of_disjoint_removal_is_a_no_op() {
        let mut sources = BTreeSet::new();
        sources.insert(cost_source([0.0, 0.0, 0.0], [1.0, 1.0, 1.0], 1.0));
        let mut to_remove = BTreeSet::new();
        to_remove.insert(cost_source([5.0, 5.0, 5.0], [6.0, 6.0, 6.0], 1.0));

        let result = remove_cost_sources(&sources, &to_remove, 1.0);
        assert_eq!(result, sources);
    }
}
