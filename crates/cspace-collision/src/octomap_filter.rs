// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/collision_detection/include/moveit/collision_detection/collision_octomap_filter.hpp
//   moveit_core/collision_detection/src/collision_octomap_filter.cpp

//! Re-estimates contact normals (and, optionally, depths) against an octomap
//! by fitting a Wyvill metaball implicit surface to the occupied cells near
//! each contact point.
//!
//! Upstream `collision_detection::refineContactNormals`, from A. Leeper,
//! S. Chan, K. Salisbury, "Point Clouds Can Be Represented as Implicit
//! Surfaces for Constraint-Based Haptic Rendering" (ICRA 2012). Upstream's
//! `RCLCPP_ERROR`/`RCLCPP_WARN` calls (invalid-argument and no-contacts
//! diagnostics) are not ported: a ROS-independent core crate has no logger
//! to hand them to (PORTING-PLAN.md D1), and neither call gates any
//! behavior — [`refine_contact_normals`] returns `0` in both cases either
//! way, so no information is lost by dropping the message.

use nalgebra::Point3;

use crate::common::CollisionResult;
use crate::world::Object;
use cspace_core::geometry::{Shape, Vector3};

/// Upstream `refineContactNormals`. Walks every contact already recorded in
/// `result` whose pair name mentions `"octomap"` (upstream's own
/// substring-match convention for the octomap side of a contact pair) and,
/// for each one, refits a metaball surface from `object`'s octree cells
/// within `cell_bbx_search_distance` cells of the contact point. A contact's
/// normal is only overwritten when the refit's normal diverges from the
/// existing one by more than `allowed_angle_divergence` radians; its depth
/// is overwritten whenever `estimate_depth` is set and the refit succeeds,
/// independent of that divergence check (matching upstream's own two
/// separate `if`s). Returns the number of normals actually rewritten.
///
/// `object` is expected to carry an `OcTree` shape as its first shape,
/// matching upstream's `object->shapes_[0]` (upstream only ever looks at
/// index `0`, never the rest of `shapes_`). Returns `0` without touching
/// `result` if `object` has no first shape, that shape is not an `OcTree`,
/// the `OcTree` wraps no tree, or `result` was not built with
/// `CollisionRequest::contacts` set (upstream's `res.contact_count < 1`
/// guard — this port's equivalent absence, `result.contacts` being `None`,
/// gets the same early return).
///
/// Upstream defaults: `cell_bbx_search_distance = 1.0`,
/// `allowed_angle_divergence = 0.0`, `estimate_depth = false`,
/// `iso_value = 0.5`, `metaball_radius_multiple = 1.5`.
pub fn refine_contact_normals(
    object: &Object,
    result: &mut CollisionResult,
    cell_bbx_search_distance: f64,
    allowed_angle_divergence: f64,
    estimate_depth: bool,
    iso_value: f64,
    metaball_radius_multiple: f64,
) -> usize {
    let Some(contacts) = result.contacts.as_mut() else {
        return 0;
    };
    if contacts.count() < 1 {
        return 0;
    }

    let Some(entry) = object.shapes().first() else {
        return 0;
    };
    let Shape::OcTree(octree_shape) = entry.shape().as_ref() else {
        return 0;
    };
    let Some(octree) = &octree_shape.octree else {
        return 0;
    };
    let cell_size = octree.resolution();

    let mut modified = 0usize;
    for ((body_1, body_2), contact_vector) in &mut contacts.by_pair {
        if !body_1.contains("octomap") && !body_2.contains("octomap") {
            continue;
        }
        for contact in contact_vector {
            let contact_point = Point3::from(contact.pos);
            let half_diagonal =
                Vector3::new(1.0, 1.0, 1.0) * (cell_size * cell_bbx_search_distance);
            let bbx_min = contact_point - half_diagonal;
            let bbx_max = contact_point + half_diagonal;

            let node_centers: Vec<Vector3> = match octree.leaves_in_bbx(bbx_min, bbx_max) {
                Some(leaves) => leaves
                    .filter(cspace_core::octomap::Leaf::is_occupied)
                    .map(|leaf| leaf.coordinate().coords)
                    .collect(),
                None => Vec::new(),
            };

            let Some((normal, depth)) = metaball_surface_properties(
                &node_centers,
                cell_size,
                iso_value,
                metaball_radius_multiple,
                contact.pos,
                estimate_depth,
            ) else {
                continue;
            };

            // Only overwrite the normal if the refit predicts a
            // sufficiently different result.
            if contact.normal.angle(&normal) > allowed_angle_divergence {
                modified += 1;
                contact.normal = normal;
            }
            if let Some(depth) = depth {
                contact.depth = depth;
            }
        }
    }
    modified
}

/// Upstream `getMetaballSurfaceProperties`. `estimate_depth == false` only
/// samples the field gradient at `contact_point` (upstream: "just get
/// normals, no depth"); `true` also walks to the surface via
/// [`find_surface`] and reports a signed depth along that surface's normal.
/// `None` when the underlying [`sample_cloud`]/[`find_surface`] call fails
/// (empty `cloud`, or -- for `estimate_depth` -- no convergence within 10
/// iterations).
fn metaball_surface_properties(
    cloud: &[Vector3],
    spacing: f64,
    iso_value: f64,
    r_multiple: f64,
    contact_point: Vector3,
    estimate_depth: bool,
) -> Option<(Vector3, Option<f64>)> {
    if estimate_depth {
        let (surface_point, normal) =
            find_surface(cloud, spacing, iso_value, r_multiple, contact_point)?;
        let depth = normal.dot(&(surface_point - contact_point));
        Some((normal, Some(depth)))
    } else {
        let (_, gradient) = sample_cloud(cloud, spacing, r_multiple, contact_point)?;
        // octomath's `normalize()` (`third_party/octomap/octomap/include/
        // octomap/math/Vector3.h:270-276`) leaves the vector unchanged when
        // `len > 0` is false; nalgebra's `.normalize()` divides
        // unconditionally, so a symmetric cloud whose gradient sums to
        // exactly zero produced an all-NaN normal instead of upstream's
        // zero one.
        Some((gradient.try_normalize(0.0).unwrap_or(gradient), None))
    }
}

/// Upstream `findSurface`, from Salisbury & Tarr's 1997 paper: starting at
/// `seed`, follows the implicit field's gradient to the `iso_value`
/// iso-surface. Returns `(surface_point, normal)`, or `None` if it fails to
/// converge within upstream's fixed 10 iterations (or the underlying
/// [`sample_cloud`] call fails).
fn find_surface(
    cloud: &[Vector3],
    spacing: f64,
    iso_value: f64,
    r_multiple: f64,
    seed: Vector3,
) -> Option<(Vector3, Vector3)> {
    const ITERATIONS: usize = 10;
    const EPSILON: f64 = 1e-10;

    let mut p = seed;
    for _ in 0..ITERATIONS {
        let (intensity, gradient) = sample_cloud(cloud, spacing, r_multiple, p)?;
        let s = iso_value - intensity;
        // `std::max(gs.dot(gs), epsilon)` upstream (`collision_octomap_filter.cpp:226`)
        // is the same std::min/std::max-vs-f64::min/f64::max family as
        // `crate::numeric`, but NOT a fix here: `sample_cloud`'s only
        // reachable NaN source (`r == 0`, a query position exactly at a
        // cloud point) sets `pos = delta / r` with `delta` the zero vector,
        // so every component of that term -- and hence of `gradient` -- goes
        // NaN together, never a subset. `gradient * -s` above is then NaN in
        // every component regardless of what this `.max` evaluates to, so
        // `dp` is `(NaN, NaN, NaN)` whether the divergent `f64::max` or a
        // faithful `cxx_max` runs here -- measured directly: `find_surface`
        // returns `None` identically either way for `cloud = [Vector3::
        // zeros()], seed = Vector3::zeros()`. (A mixed NaN/non-NaN gradient
        // is constructible in principle, through `0.0 * f64::INFINITY` in
        // the per-point scalar/direction product, but only at a per-axis
        // coordinate difference and query distance around 1e61 -- outside
        // any real octree leaf's coordinate range, so not a production
        // concern.)
        let dp = (gradient * -s) / gradient.dot(&gradient).max(EPSILON);
        p += dp;
        if dp.dot(&dp) < EPSILON {
            // Same octomath-vs-nalgebra zero-vector divergence as
            // `metaball_surface_properties` above: a symmetric cloud makes
            // `gradient` sum to exactly zero on the converging iteration.
            return Some((p, gradient.try_normalize(0.0).unwrap_or(gradient)));
        }
    }
    None
}

/// Upstream `sampleCloud`: samples a Wyvill metaball implicit field (each
/// occupied cell in `cloud` contributing its own metaball of radius
/// `r_multiple * spacing`) at `position`, returning `(intensity, gradient)`.
/// `None` when `cloud` is empty (upstream's `nn == 0` early return).
///
/// # Deviations from upstream
///
/// Upstream also compares each point's distance `r` against a bounds
/// variable also named `r` (`if (r > r) continue;`) before including it —
/// but that outer `r` is shadowed by the very `const double r = pos.norm();`
/// declared two lines above, so both operands of the comparison resolve to
/// the same local, and the check is always false. Every point in `cloud`
/// contributes regardless of `r_multiple * spacing`, unconditionally, in
/// upstream as compiled. This port reproduces that behavior by simply not
/// filtering — `clippy::eq_op` correctly refuses to compile a literal
/// `r > r`, and there is no other bound to apply once the shadowing is
/// named.
///
/// Upstream also does not guard the `1.0 / r` division inside the loop
/// against `r == 0.0` (a query position exactly at a cell center), unlike
/// [`find_surface`]'s own `max(_, EPSILON)` guard on its unrelated
/// denominator; a `NaN`/`inf` contribution from that case propagates into
/// `intensity`/`gradient` here exactly as it would upstream.
fn sample_cloud(
    cloud: &[Vector3],
    spacing: f64,
    r_multiple: f64,
    position: Vector3,
) -> Option<(f64, Vector3)> {
    if cloud.is_empty() {
        return None;
    }

    let r = r_multiple * spacing;
    let r2 = r * r;
    let r4 = r2 * r2;
    let r6 = r4 * r2;
    let a1 = (-4.0 / 9.0) / r6;
    let b1 = (17.0 / 9.0) / r4;
    let c1 = (-22.0 / 9.0) / r2;
    let a2 = 6.0 * a1;
    let b2 = 4.0 * b1;
    let c2 = 2.0 * c1;

    let mut intensity = 0.0;
    let mut gradient = Vector3::zeros();
    for v in cloud {
        let delta = position - v;
        let r = delta.norm();
        let pos = delta / r;
        let r2 = r * r;
        let r3 = r * r2;
        let r4 = r2 * r2;
        let r5 = r3 * r2;
        let r6 = r3 * r3;

        let f_val = a1 * r6 + b1 * r4 + c1 * r2 + 1.0;
        let f_grad = pos * (a2 * r5 + b2 * r3 + c2 * r);

        intensity += f_val;
        gradient += f_grad;
    }
    // "implicit surface gradient convention points out, so we flip it."
    Some((intensity, -gradient))
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeMap;
    use std::sync::Arc;

    use approx::assert_relative_eq;
    use cspace_core::geometry::{Isometry3, OcTree as OcTreeShape, Shape, Sphere};
    use nalgebra::Point3;

    use super::*;
    use crate::common::{Contact, ContactData};
    use crate::world::World;

    // `refine_contact_normals` has no upstream caller anywhere in moveit2
    // (confirmed via `rg -rn refineContactNormals` across the whole tree,
    // matching only its own declaration/definition), so there is no oracle
    // fixture to differentially test against. These are self-contained
    // property tests of the ported algorithm instead.

    fn octree_with_cells(cells: &[Point3<f64>]) -> cspace_core::octomap::OcTree {
        let mut tree = cspace_core::octomap::OcTree::new(0.5);
        for cell in cells {
            tree.update_node(*cell, true, false);
        }
        tree
    }

    fn object_with_octree(tree: Option<cspace_core::octomap::OcTree>) -> Arc<crate::world::Object> {
        let mut world = World::new();
        let shape = Shape::OcTree(OcTreeShape::from_tree(Arc::new(
            tree.unwrap_or_else(|| cspace_core::octomap::OcTree::new(0.5)),
        )));
        world
            .add_shape("octomap", Arc::new(shape), Isometry3::identity())
            .expect("non-empty shape list");
        world.get_object("octomap").expect("just inserted")
    }

    fn result_with_contact(pos: Vector3, normal: Vector3) -> CollisionResult {
        let mut by_pair = BTreeMap::new();
        by_pair.insert(
            ("robot_link".to_owned(), "octomap".to_owned()),
            vec![Contact {
                pos,
                normal,
                depth: 0.01,
                ..Default::default()
            }],
        );
        CollisionResult {
            contacts: Some(ContactData { by_pair }),
            ..Default::default()
        }
    }

    #[test]
    fn sample_cloud_empty_returns_none() {
        assert!(sample_cloud(&[], 0.1, 1.5, Vector3::zeros()).is_none());
    }

    #[test]
    fn sample_cloud_symmetric_pair_cancels_gradient() {
        let cloud = [Vector3::new(1.0, 0.0, 0.0), Vector3::new(-1.0, 0.0, 0.0)];
        let (intensity, gradient) =
            sample_cloud(&cloud, 1.0, 1.5, Vector3::zeros()).expect("non-empty cloud");
        // Both cloud points sit at the same distance from the origin with
        // exactly opposite unit directions, so their gradient contributions
        // are exact negatives of each other -- this is a property of the
        // field, not a tolerance question.
        assert_eq!(gradient, Vector3::zeros());
        assert!(
            intensity > 0.0,
            "a point strictly inside its own metaball radius contributes positive intensity"
        );
    }

    #[test]
    fn sample_cloud_position_at_cell_center_is_the_preserved_upstream_quirk() {
        // Regression test for the deviation documented on `sample_cloud`:
        // `position == cloud[0]` divides `0.0 / 0.0` when normalizing the
        // (zero-length) delta vector, poisoning the gradient with `NaN`
        // exactly as upstream's unguarded `1.0 / r` does.
        let cloud = [Vector3::zeros()];
        let (_, gradient) =
            sample_cloud(&cloud, 1.0, 1.5, Vector3::zeros()).expect("non-empty cloud");
        assert!(gradient.x.is_nan() && gradient.y.is_nan() && gradient.z.is_nan());
    }

    #[test]
    fn find_surface_converges_to_the_iso_value_for_a_single_point_source() {
        let cloud = [Vector3::zeros()];
        let seed = Vector3::new(1.0, 0.0, 0.0);
        let (surface_point, normal) =
            find_surface(&cloud, 1.0, 0.5, 1.5, seed).expect("single-point field converges");
        let (intensity_at_surface, _) =
            sample_cloud(&cloud, 1.0, 1.5, surface_point).expect("non-empty cloud");
        assert_relative_eq!(intensity_at_surface, 0.5, epsilon = 1e-6);
        // The field is radially symmetric around the single cloud point and
        // the seed lies exactly on the x-axis, so the whole iteration stays
        // on that axis: the surface point and its outward normal both point
        // in +x.
        assert!(surface_point.x > 0.0);
        assert_relative_eq!(normal, Vector3::new(1.0, 0.0, 0.0), epsilon = 1e-6);
    }

    #[test]
    fn find_surface_zero_gradient_seed_returns_zero_normal_not_nan() {
        // octomath's `normalize()` leaves a zero vector unchanged; a plain
        // `.normalize()` divides `0.0 / 0.0` instead. Same symmetric cloud
        // as `sample_cloud_symmetric_pair_cancels_gradient`, sampled at the
        // seed itself so `gradient` is exactly zero on iteration 1: `dp`
        // collapses to the zero vector too (`.max(EPSILON)` guards only the
        // division, not the numerator), so `dp.dot(&dp) < EPSILON` fires
        // immediately and returns the zero gradient un-advanced.
        let cloud = [Vector3::new(1.0, 0.0, 0.0), Vector3::new(-1.0, 0.0, 0.0)];
        let seed = Vector3::zeros();
        let (surface_point, normal) =
            find_surface(&cloud, 1.0, 0.5, 1.5, seed).expect("non-empty cloud");
        assert_eq!(surface_point, Vector3::zeros());
        assert_eq!(normal, Vector3::zeros());
    }

    #[test]
    fn metaball_surface_properties_without_depth_returns_unit_normal_only() {
        let cloud = [Vector3::new(1.0, 0.0, 0.0)];
        let (normal, depth) =
            metaball_surface_properties(&cloud, 1.0, 0.5, 1.5, Vector3::new(0.5, 0.0, 0.0), false)
                .expect("non-empty cloud");
        assert_relative_eq!(normal.norm(), 1.0, epsilon = 1e-12);
        assert!(depth.is_none());
    }

    #[test]
    fn metaball_surface_properties_without_depth_zero_gradient_returns_zero_normal_not_nan() {
        // Same zero-gradient construction as `find_surface_zero_gradient_
        // seed_returns_zero_normal_not_nan`, through the other call site
        // (`metaball_surface_properties`'s `estimate_depth == false`
        // branch) that shares the same `.normalize()` fix.
        let cloud = [Vector3::new(1.0, 0.0, 0.0), Vector3::new(-1.0, 0.0, 0.0)];
        let (normal, depth) =
            metaball_surface_properties(&cloud, 1.0, 0.5, 1.5, Vector3::zeros(), false)
                .expect("non-empty cloud");
        assert_eq!(normal, Vector3::zeros());
        assert!(depth.is_none());
    }

    #[test]
    fn metaball_surface_properties_with_depth_reports_signed_depth() {
        let cloud = [Vector3::zeros()];
        let (_, depth) =
            metaball_surface_properties(&cloud, 1.0, 0.5, 1.5, Vector3::new(1.0, 0.0, 0.0), true)
                .expect("single-point field converges");
        assert!(depth.is_some());
    }

    #[test]
    fn metaball_surface_properties_empty_cloud_is_none_in_both_modes() {
        assert!(metaball_surface_properties(&[], 1.0, 0.5, 1.5, Vector3::zeros(), false).is_none());
        assert!(metaball_surface_properties(&[], 1.0, 0.5, 1.5, Vector3::zeros(), true).is_none());
    }

    #[test]
    fn refine_contact_normals_no_contacts_requested_is_a_noop() {
        let object = object_with_octree(Some(octree_with_cells(&[Point3::new(0.0, 0.0, 0.0)])));
        let mut result = CollisionResult::default();
        assert_eq!(
            refine_contact_normals(&object, &mut result, 1.0, 0.0, false, 0.5, 1.5),
            0
        );
        assert!(result.contacts.is_none());
    }

    #[test]
    fn refine_contact_normals_empty_contact_map_is_a_noop() {
        let object = object_with_octree(Some(octree_with_cells(&[Point3::new(0.0, 0.0, 0.0)])));
        let mut result = CollisionResult {
            contacts: Some(ContactData::default()),
            ..Default::default()
        };
        assert_eq!(
            refine_contact_normals(&object, &mut result, 1.0, 0.0, false, 0.5, 1.5),
            0
        );
    }

    #[test]
    fn refine_contact_normals_first_shape_not_octree_is_a_noop() {
        let mut world = World::new();
        world
            .add_shape(
                "obstacle",
                Arc::new(Shape::Sphere(Sphere { radius: 1.0 })),
                Isometry3::identity(),
            )
            .expect("non-empty shape list");
        let object = world.get_object("obstacle").expect("just inserted");
        let mut result =
            result_with_contact(Vector3::new(0.1, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        assert_eq!(
            refine_contact_normals(&object, &mut result, 1.0, 0.0, false, 0.5, 1.5),
            0
        );
    }

    #[test]
    fn refine_contact_normals_octree_shape_with_no_tree_is_a_noop() {
        let object = object_with_octree(None);
        let mut result =
            result_with_contact(Vector3::new(0.1, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        assert_eq!(
            refine_contact_normals(&object, &mut result, 1.0, 0.0, false, 0.5, 1.5),
            0
        );
    }

    #[test]
    fn refine_contact_normals_skips_pairs_without_octomap_in_the_name() {
        let object = object_with_octree(Some(octree_with_cells(&[Point3::new(0.0, 0.0, 0.0)])));
        let mut by_pair = BTreeMap::new();
        by_pair.insert(
            ("robot_link_a".to_owned(), "robot_link_b".to_owned()),
            vec![Contact {
                pos: Vector3::new(0.1, 0.0, 0.0),
                normal: Vector3::new(0.0, 0.0, 1.0),
                depth: 0.01,
                ..Default::default()
            }],
        );
        let mut result = CollisionResult {
            contacts: Some(ContactData { by_pair }),
            ..Default::default()
        };
        assert_eq!(
            refine_contact_normals(&object, &mut result, 1.0, 0.0, false, 0.5, 1.5),
            0
        );
        // Untouched: no refit was ever attempted on this pair.
        assert_relative_eq!(
            result.contacts.unwrap().by_pair
                [&("robot_link_a".to_owned(), "robot_link_b".to_owned())][0]
                .normal,
            Vector3::new(0.0, 0.0, 1.0)
        );
    }

    // A voxel *center* for a resolution-0.5 tree, not a boundary: octomap
    // keys snap by `floor(coord / resolution)`, so an insertion exactly at
    // `(0.0, 0.0, 0.0)` lands on a cell boundary and comes back out of
    // `leaves_in_bbx` centered at `(0.25, 0.25, 0.25)`, not the origin
    // (confirmed empirically). Inserting at an already-aligned center avoids
    // that snap so the single-point field stays exactly radially symmetric
    // around a coordinate the test also uses directly.
    const CELL_CENTER: Vector3 = Vector3::new(0.25, 0.25, 0.25);

    #[test]
    fn refine_contact_normals_rewrites_a_sufficiently_diverging_normal_and_leaves_depth_alone_by_default()
     {
        let object = object_with_octree(Some(octree_with_cells(&[Point3::from(CELL_CENTER)])));
        let original_depth = 0.01;
        // Search distance in cells * cell_size (0.5) must reach the cell
        // center from 1.0 away: 3.0 * 0.5 = 1.5 half-diagonal.
        let contact_pos = CELL_CENTER + Vector3::new(1.0, 0.0, 0.0);
        let mut result = result_with_contact(contact_pos, Vector3::new(0.0, 1.0, 0.0));
        let modified = refine_contact_normals(&object, &mut result, 3.0, 0.01, false, 0.5, 1.5);
        assert_eq!(modified, 1);
        let contacts = result.contacts.unwrap();
        let contact = &contacts.by_pair[&("robot_link".to_owned(), "octomap".to_owned())][0];
        // Points from the contact back toward the occupied cell -- upstream's
        // own sign convention (see `sample_cloud`'s doc), not the "outward
        // from solid" direction a naive reading of "surface normal" suggests.
        assert_relative_eq!(contact.normal, Vector3::new(-1.0, 0.0, 0.0), epsilon = 1e-9);
        // `estimate_depth` was left `false`, so depth must be untouched.
        assert_relative_eq!(contact.depth, original_depth);
    }

    #[test]
    fn refine_contact_normals_estimate_depth_overwrites_depth() {
        let object = object_with_octree(Some(octree_with_cells(&[Point3::from(CELL_CENTER)])));
        // Closer than the normal-rewrite test above (0.5 vs 1.0), so the
        // query point sits inside this field's nominal radius
        // (`metaball_radius_multiple * spacing` = 1.5 * 0.5 = 0.75) and
        // `find_surface`'s 10-iteration Newton walk actually converges.
        let contact_pos = CELL_CENTER + Vector3::new(0.5, 0.0, 0.0);
        let mut result = result_with_contact(contact_pos, Vector3::new(0.0, 1.0, 0.0));
        let modified = refine_contact_normals(&object, &mut result, 3.0, 0.01, true, 0.5, 1.5);
        assert_eq!(
            modified, 1,
            "find_surface must have converged for depth to be set at all"
        );
        let contacts = result.contacts.unwrap();
        let contact = &contacts.by_pair[&("robot_link".to_owned(), "octomap".to_owned())][0];
        assert_ne!(
            contact.depth, 0.01,
            "estimate_depth = true must overwrite depth on a successful refit"
        );
    }
}
