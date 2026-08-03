// Copyright (c) 2013, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from geometric_shapes 2.3.3 (not part of moveit2 itself — see
// below): include/geometric_shapes/bodies.h, src/bodies.cpp
//   (bodies::Sphere, bodies::Cylinder, bodies::Box: containsPoint,
//   updateInternalData, computeBoundingSphere)

//! A [`Shape`] posed in the world, implementing [`ConvexBody`] so it can
//! feed [`crate::find_internal_points_convex`].
//!
//! [`crate::DistanceField::add_shape_to_field`] and its siblings need the
//! containment/bounding-sphere half of upstream `bodies::Body` —
//! `geometric_shapes`'s posed algorithmic layer over `shapes::Shape`. That
//! type is a separate upstream package from `moveit2` and is not yet ported
//! into any crate in this workspace: `moveit-geometry` carries the unposed
//! `shapes::Shape` data layer only (its own module doc explicitly defers
//! `bodies::Body`), and `moveit-collision` doesn't have it either. This
//! module ports exactly the two operations
//! [`crate::find_internal_points_convex`] needs
//! (`containsPoint`/`computeBoundingSphere`) directly from real upstream
//! `geometric_shapes` 2.3.3 `bodies.{h,cpp}`, scoped `pub(crate)` since the
//! full shape hierarchy belongs to whichever crate eventually owns it, not
//! to this one.
//!
//! # Deviations from upstream
//!
//! - Upstream's `bodies::Body` carries `padding_`/`scale_` fields, set via
//!   `setPadding`/`setScale` and defaulting to `0.0`/`1.0`. Every call site
//!   in `distance_field.cpp` (`getShapePoints`, `moveShapeInField`,
//!   `removeShapeFromField`) constructs a body and calls
//!   `updateInternalData()` without ever touching either, so this port
//!   hard-codes the same defaults (padding 0, scale 1) rather than carry
//!   fields nothing here sets.
//! - Only `Sphere`, `Cylinder`, `Cuboid` are supported — [`PosedShape::new`]
//!   returns [`Error::Construct`] for the rest. This matches upstream
//!   itself: `bodies::createEmptyBodyFromShapeType` (`body_operations.cpp`)
//!   has no case for `CONE`/`PLANE`/`OCTREE` (falls through to a null body,
//!   so a real caller null-derefs); `MESH` maps to `bodies::ConvexMesh`,
//!   which needs `qhull` — not a workspace dependency, and untested here. A
//!   documented construction error is strictly safer than upstream's
//!   null-deref, matching this crate's established "panic/error instead of
//!   upstream's documented corruption/SEGFAULT" pattern (see
//!   [`crate::VoxelGrid`]'s doc comment).

use moveit_error::{Error, Result};
use moveit_geometry::{Isometry3, Shape};
use nalgebra::Vector3;

use crate::find_internal_points::ConvexBody;

/// A [`Shape`] paired with the [`Isometry3`] pose it sits at in the world.
pub(crate) struct PosedShape<'a> {
    shape: &'a Shape,
    pose: &'a Isometry3,
}

impl<'a> PosedShape<'a> {
    /// # Errors
    ///
    /// [`Error::Construct`] when `shape` is not a `Sphere`, `Cylinder`, or
    /// `Cuboid` — see this module's "Deviations from upstream".
    pub(crate) fn new(shape: &'a Shape, pose: &'a Isometry3) -> Result<Self> {
        match shape {
            Shape::Sphere(_) | Shape::Cylinder(_) | Shape::Cuboid(_) => Ok(Self { shape, pose }),
            other => Err(Error::construct(format!(
                "PosedShape only supports Sphere, Cylinder, and Cuboid shapes, got {other:?}"
            ))),
        }
    }
}

impl ConvexBody for PosedShape<'_> {
    /// Upstream `bodies::{Sphere,Cylinder,Box}::computeBoundingSphere`. The
    /// bounding-sphere center of all three unposed bodies is the origin, so
    /// the posed center is always exactly `pose.translation()` regardless of
    /// shape.
    fn bounding_sphere(&self) -> (Vector3<f64>, f64) {
        let center = self.pose.translation.vector;
        let radius = match self.shape {
            Shape::Sphere(sphere) => sphere.radius,
            Shape::Cylinder(cylinder) => {
                let length2 = cylinder.length / 2.0;
                (length2 * length2 + cylinder.radius * cylinder.radius).sqrt()
            }
            Shape::Cuboid(cuboid) => {
                Vector3::new(cuboid.size[0], cuboid.size[1], cuboid.size[2]).norm() * 0.5
            }
            _ => unreachable!("shape variant validated in PosedShape::new"),
        };
        (center, radius)
    }

    fn contains_point(&self, point: &Vector3<f64>) -> bool {
        let center = self.pose.translation.vector;
        match self.shape {
            // Upstream `bodies::Sphere::containsPoint`.
            Shape::Sphere(sphere) => {
                (point - center).norm_squared() <= sphere.radius * sphere.radius
            }
            // Upstream `bodies::Cylinder::containsPoint`.
            Shape::Cylinder(cylinder) => {
                let rotation = self.pose.rotation;
                let v = point - center;

                let normal_h = rotation * Vector3::z();
                let length2 = cylinder.length / 2.0;
                if v.dot(&normal_h).abs() > length2 {
                    return false;
                }

                let normal_b1 = rotation * Vector3::x();
                let p_b1 = v.dot(&normal_b1);
                let remaining = cylinder.radius * cylinder.radius - p_b1 * p_b1;
                if remaining < 0.0 {
                    return false;
                }

                let normal_b2 = rotation * Vector3::y();
                let p_b2 = v.dot(&normal_b2);
                p_b2 * p_b2 <= remaining
            }
            // Upstream `bodies::Box::containsPoint`.
            Shape::Cuboid(cuboid) => {
                let aligned = (self.pose.rotation.inverse() * (point - center)).abs();
                aligned.x <= cuboid.size[0] / 2.0
                    && aligned.y <= cuboid.size[1] / 2.0
                    && aligned.z <= cuboid.size[2] / 2.0
            }
            _ => unreachable!("shape variant validated in PosedShape::new"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use moveit_geometry::{Cuboid, Cylinder, Sphere};

    #[test]
    fn rejects_unsupported_shape_variants() {
        let plane = Shape::Plane(moveit_geometry::Plane::new(0.0, 0.0, 1.0, 0.0));
        let pose = Isometry3::identity();
        assert!(PosedShape::new(&plane, &pose).is_err());
    }

    #[test]
    fn sphere_bounding_sphere_and_containment_are_pose_translated() {
        let sphere = Shape::Sphere(Sphere::new(0.5).unwrap());
        let pose = Isometry3::translation(1.0, 2.0, 3.0);
        let posed = PosedShape::new(&sphere, &pose).unwrap();

        let (center, radius) = posed.bounding_sphere();
        assert_relative_eq!(center, Vector3::new(1.0, 2.0, 3.0));
        assert_relative_eq!(radius, 0.5);

        assert!(posed.contains_point(&Vector3::new(1.0, 2.0, 3.0)));
        assert!(posed.contains_point(&Vector3::new(1.5, 2.0, 3.0)));
        assert!(!posed.contains_point(&Vector3::new(1.51, 2.0, 3.0)));
    }

    #[test]
    fn cylinder_containment_respects_rotation() {
        let cylinder = Shape::Cylinder(Cylinder::new(0.5, 2.0).unwrap());
        // Rotate the cylinder's axis (locally z) onto the world x axis.
        let pose = Isometry3::from_parts(
            nalgebra::Translation3::identity(),
            nalgebra::UnitQuaternion::from_axis_angle(
                &nalgebra::Vector3::y_axis(),
                std::f64::consts::FRAC_PI_2,
            ),
        );
        let posed = PosedShape::new(&cylinder, &pose).unwrap();

        // Now long along world x (length2 = 1.0), radius 0.5 in y/z.
        assert!(posed.contains_point(&Vector3::new(0.9, 0.0, 0.0)));
        assert!(!posed.contains_point(&Vector3::new(1.1, 0.0, 0.0)));
        assert!(posed.contains_point(&Vector3::new(0.0, 0.4, 0.0)));
        assert!(!posed.contains_point(&Vector3::new(0.0, 0.6, 0.0)));
    }

    #[test]
    fn cuboid_bounding_sphere_and_containment() {
        let cuboid = Shape::Cuboid(Cuboid::new(1.0, 2.0, 3.0).unwrap());
        let pose = Isometry3::identity();
        let posed = PosedShape::new(&cuboid, &pose).unwrap();

        let (center, radius) = posed.bounding_sphere();
        assert_relative_eq!(center, Vector3::zeros());
        assert_relative_eq!(
            radius,
            (0.5_f64.powi(2) + 1.0_f64.powi(2) + 1.5_f64.powi(2)).sqrt()
        );

        assert!(posed.contains_point(&Vector3::new(0.49, 0.99, 1.49)));
        assert!(!posed.contains_point(&Vector3::new(0.51, 0.0, 0.0)));
    }
}
