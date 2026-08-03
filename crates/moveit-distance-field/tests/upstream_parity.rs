// Copyright (c) 2009, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/distance_field/test/test_distance_field.cpp

//! Ported cases from upstream's own gtest suite for this subsystem — the
//! ground truth for this crate (see `test_distance_field.cpp`'s
//! `TEST(TestPropagationDistanceField, ...)` / `TEST(TestSignedPropagationDistanceField,
//! ...)`), plus the two `TestVoxelGrid.TestReadWrite` dimension assertions
//! already folded into `voxel_grid.rs`'s `dimensions_match_upstream_test_read_write`.
//!
//! Deliberately **not** ported, with reasons:
//!
//! - `TestOcTree` — needs `octomap::OcTree` and
//!   `DistanceField::addOcTreeToField`; neither exists in this workspace (see
//!   [`moveit_distance_field::DistanceField`]'s "Deviations from upstream").
//! - `TestPerformance` — a benchmark (timing printed to stdout), not a
//!   correctness assertion.
//! - The file-I/O half of `TestReadWrite` (`writeToStream`/`readFromStream`
//!   round trip) — that API isn't ported; see the same doc section.
//! - The shape/gradient portion of `TestSignedAddRemovePoints` (everything
//!   from `gradient_df.addShapeToField` onward) — `addShapeToField` itself
//!   isn't ported (no `geometric_shapes::Shape` in this workspace), and
//!   `getNearestCell`'s gradient-direction assertions are exercised more
//!   directly by this crate's own boundary tests. The points-only portion
//!   (add a cube of points, remove the center, assert equality against a
//!   fresh rebuild) is ported below as
//!   [`signed_add_remove_points_matches_rebuild_without_the_removed_point`] —
//!   this is exactly the add/remove-vs-rebuild invariant our verification
//!   brief asks for independent of the shape/gradient plumbing.
//! - `TestShape` is ported with `addShapeToField`/`moveShapeInField` inlined
//!   to their upstream definitions (`getShapePoints` +
//!   `addPointsToField`/`updatePointsInField` — see `distance_field.cpp`),
//!   since a sphere shape is available in this crate's test-only
//!   [`ConvexBody`] the same way `find_internal_points.rs`'s own unit tests
//!   use one.

use approx::assert_relative_eq;
use nalgebra::Vector3;

use moveit_distance_field::{
    ConvexBody, DistanceField, GridGeometry, PropagationDistanceField, find_internal_points_convex,
};

const WIDTH: f64 = 1.0;
const HEIGHT: f64 = 1.0;
const DEPTH: f64 = 1.0;
const RESOLUTION: f64 = 0.1;
const ORIGIN_X: f64 = 0.0;
const ORIGIN_Y: f64 = 0.0;
const ORIGIN_Z: f64 = 0.0;
const MAX_DIST: f64 = 0.3;

fn geometry() -> GridGeometry {
    GridGeometry::new(
        Vector3::new(WIDTH, HEIGHT, DEPTH),
        Vector3::new(ORIGIN_X, ORIGIN_Y, ORIGIN_Z),
        RESOLUTION,
    )
    .unwrap()
}

fn point1() -> Vector3<f64> {
    Vector3::new(0.1, 0.0, 0.0)
}
fn point2() -> Vector3<f64> {
    Vector3::new(0.0, 0.1, 0.2)
}
fn point3() -> Vector3<f64> {
    Vector3::new(0.4, 0.0, 0.0)
}

/// Upstream `checkDistanceField`: every obstacle cell (`distance_square_ ==
/// 0`) in the field must be one of `points`, and (when `do_negs`) must carry
/// a positive `negative_distance_square_`.
fn check_distance_field(
    df: &PropagationDistanceField,
    points: &[Vector3<f64>],
    num_x: i32,
    num_y: i32,
    num_z: i32,
    do_negs: bool,
) {
    let points_ind: Vec<Vector3<i32>> = points
        .iter()
        .map(|p| {
            let (_valid, x, y, z) = df.world_to_grid(p);
            Vector3::new(x, y, z)
        })
        .collect();

    for x in 0..num_x {
        for y in 0..num_y {
            for z in 0..num_z {
                let voxel = df.cell(x, y, z);
                if voxel.distance_square == 0 {
                    let found = points_ind.iter().any(|p| p.x == x && p.y == y && p.z == z);
                    if do_negs {
                        assert!(
                            voxel.negative_distance_square > 0,
                            "Obstacle point {x} {y} {z} has zero negative value"
                        );
                    }
                    assert!(found, "Obstacle point {x} {y} {z} not found");
                }
            }
        }
    }
}

/// Upstream `areDistanceFieldsDistancesEqual`.
fn distance_fields_have_equal_distances(
    a: &PropagationDistanceField,
    b: &PropagationDistanceField,
) -> bool {
    if a.num_cells_x() != b.num_cells_x()
        || a.num_cells_y() != b.num_cells_y()
        || a.num_cells_z() != b.num_cells_z()
    {
        return false;
    }
    for z in 0..a.num_cells_z() {
        for x in 0..a.num_cells_x() {
            for y in 0..a.num_cells_y() {
                let va = a.cell(x, y, z);
                let vb = b.cell(x, y, z);
                if va.distance_square != vb.distance_square {
                    return false;
                }
                if va.negative_distance_square != vb.negative_distance_square {
                    return false;
                }
            }
        }
    }
    true
}

/// Upstream `TEST(TestPropagationDistanceField, TestAddRemovePoints)`.
#[test]
fn add_remove_points_matches_upstream_test_propagation_distance_field() {
    let mut df = PropagationDistanceField::new(geometry(), MAX_DIST, false).unwrap();

    let num_x = df.num_cells_x();
    let num_y = df.num_cells_y();
    let num_z = df.num_cells_z();

    assert_eq!(num_x, (WIDTH / RESOLUTION + 0.5) as i32);
    assert_eq!(num_y, (HEIGHT / RESOLUTION + 0.5) as i32);
    assert_eq!(num_z, (DEPTH / RESOLUTION + 0.5) as i32);

    assert_relative_eq!(
        df.distance(1000.0, 1000.0, 1000.0),
        MAX_DIST,
        epsilon = 0.0001
    );
    let grad = df.distance_gradient(1000.0, 1000.0, 1000.0);
    assert_relative_eq!(grad.distance, MAX_DIST, epsilon = 0.0001);
    assert!(!grad.in_bounds);

    df.add_points_to_field(&[point1(), point2()]);

    df.update_points_in_field(&[point1()], &[point2(), point3()]);
    check_distance_field(&df, &[point2(), point3()], num_x, num_y, num_z, false);

    df.remove_points_from_field(&[point2()]);
    check_distance_field(&df, &[point3()], num_x, num_y, num_z, false);

    df.reset();
    df.add_points_to_field(&[point1()]);
    let mut first = true;
    for z in 1..df.num_cells_z() - 1 {
        for x in 1..df.num_cells_x() - 1 {
            for y in 1..df.num_cells_y() - 1 {
                let dist = df.distance_cell(x, y, z);
                let world = df.grid_to_world(x, y, z);
                let grad = df.distance_gradient(world.x, world.y, world.z);
                assert!(grad.in_bounds, "{x} {y} {z}");
                assert_relative_eq!(dist, grad.distance, epsilon = 0.0001);
                if dist > 0.0 && dist < MAX_DIST {
                    let norm = grad.gradient.norm();
                    let xscale = grad.gradient.x / norm;
                    let yscale = grad.gradient.y / norm;
                    let zscale = grad.gradient.z / norm;

                    let comp_x = world.x - xscale * dist;
                    let comp_y = world.y - yscale * dist;
                    let comp_z = world.z - zscale * dist;
                    first = false;
                    assert_relative_eq!(comp_x, point1().x, epsilon = RESOLUTION);
                    assert_relative_eq!(comp_y, point1().y, epsilon = RESOLUTION);
                    assert_relative_eq!(comp_z, point1().z, epsilon = RESOLUTION);
                }
            }
        }
    }
    assert!(!first);
}

/// Upstream `TEST(TestSignedPropagationDistanceField, TestSignedAddRemovePoints)`,
/// points-only portion — see this file's module doc for the shape/gradient
/// portion this omits.
#[test]
fn signed_add_remove_points_matches_rebuild_without_the_removed_point() {
    let mut df = PropagationDistanceField::new(geometry(), MAX_DIST, true).unwrap();

    let num_x = df.num_cells_x();
    let num_y = df.num_cells_y();
    let num_z = df.num_cells_z();
    assert_eq!(num_x, (WIDTH / RESOLUTION + 0.5) as i32);
    assert_eq!(num_y, (HEIGHT / RESOLUTION + 0.5) as i32);
    assert_eq!(num_z, (DEPTH / RESOLUTION + 0.5) as i32);

    let low = df.grid_to_world(1, 1, 1);
    let high = df.grid_to_world(8, 8, 8);

    let mut points = Vec::new();
    let mut x = low.x;
    while x <= high.x {
        let mut y = low.y;
        while y <= high.y {
            let mut z = low.z;
            while z <= high.z {
                points.push(Vector3::new(x, y, z));
                z += 0.1;
            }
            y += 0.1;
        }
        x += 0.1;
    }

    df.reset();
    df.add_points_to_field(&points);

    let center_point = df.grid_to_world(5, 5, 5);
    df.remove_points_from_field(&[center_point]);

    let mut test_df = PropagationDistanceField::new(geometry(), MAX_DIST, true).unwrap();
    let test_points: Vec<Vector3<f64>> = points
        .iter()
        .copied()
        .filter(|&p| p != center_point)
        .collect();
    test_df.add_points_to_field(&test_points);

    assert!(distance_fields_have_equal_distances(&df, &test_df));
}

struct Sphere {
    center: Vector3<f64>,
    radius: f64,
}

impl ConvexBody for Sphere {
    fn bounding_sphere(&self) -> (Vector3<f64>, f64) {
        (self.center, self.radius)
    }

    fn contains_point(&self, point: &Vector3<f64>) -> bool {
        (point - self.center).norm() <= self.radius
    }
}

/// Upstream `TEST(TestSignedPropagationDistanceField, TestShape)`, with
/// `addShapeToField(shape, pose)` inlined to `findInternalPointsConvex` +
/// `addPointsToField`, and `moveShapeInField(shape, old, new)` inlined to
/// `findInternalPointsConvex` at both poses + `updatePointsInField` — see
/// `distance_field.cpp`'s definitions of both, and this file's module doc.
#[test]
fn moving_a_shape_in_field_matches_rebuild_at_the_new_pose() {
    let mut df = PropagationDistanceField::new(geometry(), MAX_DIST, true).unwrap();

    let num_x = df.num_cells_x();
    let num_y = df.num_cells_y();
    let num_z = df.num_cells_z();

    let sphere_p = Sphere {
        center: Vector3::new(0.5, 0.5, 0.5),
        radius: 0.25,
    };
    let sphere_np = Sphere {
        center: Vector3::new(0.7, 0.7, 0.7),
        radius: 0.25,
    };

    let mut point_vec = Vec::new();
    find_internal_points_convex(&sphere_p, RESOLUTION, &mut point_vec);
    df.add_points_to_field(&point_vec);
    check_distance_field(&df, &point_vec, num_x, num_y, num_z, true);

    let mut point_vec_2 = Vec::new();
    find_internal_points_convex(&sphere_np, RESOLUTION, &mut point_vec_2);
    df.add_points_to_field(&point_vec_2);
    let mut point_vec_union = point_vec_2.clone();
    point_vec_union.extend(point_vec.iter().copied());
    check_distance_field(&df, &point_vec_union, num_x, num_y, num_z, true);

    // "should get rid of old pose"
    df.update_points_in_field(&point_vec, &point_vec_2);
    check_distance_field(&df, &point_vec_2, num_x, num_y, num_z, true);

    // "should be equivalent to just adding second shape"
    let mut test_df = PropagationDistanceField::new(geometry(), MAX_DIST, true).unwrap();
    test_df.add_points_to_field(&point_vec_2);
    assert!(distance_fields_have_equal_distances(&df, &test_df));
}
