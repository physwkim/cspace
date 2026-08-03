// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/distance_field/include/moveit/distance_field/find_internal_points.hpp
//   moveit_core/distance_field/src/find_internal_points.cpp

use nalgebra::Vector3;

/// The minimal shape interface [`find_internal_points_convex`] needs.
///
/// Upstream takes a `const bodies::Body&` from `geometric_shapes` directly.
/// This port narrows the dependency to exactly the two operations
/// `findInternalPointsConvex` actually calls on a body: `computeBoundingSphere`
/// and `containsPoint` — implemented for [`moveit_geometry::bodies::Body`] in
/// [`crate::DistanceField`]'s module (see that module for which shape
/// variants are supported). Convex correctness is the caller's
/// responsibility, as upstream documents ("If the body is not convex then
/// its convex hull is used").
pub trait ConvexBody {
    /// Upstream `bodies::Body::computeBoundingSphere`: a sphere (center,
    /// radius) that contains the whole body.
    fn bounding_sphere(&self) -> (Vector3<f64>, f64);
    /// Upstream `bodies::Body::containsPoint`.
    fn contains_point(&self, point: &Vector3<f64>) -> bool;
}

/// Find every point on a `resolution`-spaced grid that lies inside `body`.
///
/// Upstream `distance_field::findInternalPointsConvex`.
pub fn find_internal_points_convex<B: ConvexBody>(
    body: &B,
    resolution: f64,
    points: &mut Vec<Vector3<f64>>,
) {
    let (center, radius) = body.bounding_sphere();

    let start_x = ((center.x - radius - resolution) / resolution).floor() * resolution;
    let start_y = ((center.y - radius - resolution) / resolution).floor() * resolution;
    let start_z = ((center.z - radius - resolution) / resolution).floor() * resolution;
    let end_x = center.x + radius + resolution;
    let end_y = center.y + radius + resolution;
    let end_z = center.z + radius + resolution;

    let mut x = start_x;
    while x <= end_x {
        let mut y = start_y;
        while y <= end_y {
            let mut z = start_z;
            while z <= end_z {
                let point = Vector3::new(x, y, z);
                if body.contains_point(&point) {
                    points.push(point);
                }
                z += resolution;
            }
            y += resolution;
        }
        x += resolution;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

    #[test]
    fn every_returned_point_is_inside_the_body() {
        let sphere = Sphere {
            center: Vector3::new(0.5, 0.5, 0.5),
            radius: 0.25,
        };
        let mut points = Vec::new();
        find_internal_points_convex(&sphere, 0.1, &mut points);
        assert!(!points.is_empty());
        for p in &points {
            assert!(sphere.contains_point(p));
        }
    }

    #[test]
    fn the_center_point_is_found_on_a_grid_aligned_with_the_origin() {
        let sphere = Sphere {
            center: Vector3::new(0.5, 0.5, 0.5),
            radius: 0.25,
        };
        let mut points = Vec::new();
        find_internal_points_convex(&sphere, 0.1, &mut points);
        assert!(
            points
                .iter()
                .any(|p| (p - Vector3::new(0.5, 0.5, 0.5)).norm() < 1e-9)
        );
    }
}
