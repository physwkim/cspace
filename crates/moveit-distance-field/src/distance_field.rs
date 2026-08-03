// Copyright (c) 2009, Willow Garage, Inc.
// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/distance_field/include/moveit/distance_field/distance_field.hpp
//   moveit_core/distance_field/src/distance_field.cpp

use moveit_error::{Error, Result};
use moveit_geometry::bodies::Body;
use moveit_geometry::{Isometry3, Shape};
use nalgebra::Vector3;

use crate::find_internal_points::{ConvexBody, find_internal_points_convex};

/// Upstream `bodies::Body::computeBoundingSphere`/`containsPoint`, dispatched
/// per body kind by [`Body`] itself.
impl ConvexBody for Body {
    fn bounding_sphere(&self) -> (Vector3<f64>, f64) {
        let sphere = self.compute_bounding_sphere();
        (sphere.center, sphere.radius)
    }

    fn contains_point(&self, point: &Vector3<f64>) -> bool {
        Body::contains_point(self, point)
    }
}

/// Build the posed [`Body`] for `shape`, at identity scale and zero padding
/// — matching `distance_field.cpp`'s `getShapePoints`/`addShapeToField`/
/// `moveShapeInField`, none of which ever call `setScale`/`setPadding` on
/// the body they construct.
///
/// # Errors
///
/// [`Error::Construct`] for shape kinds with no `bodies::` counterpart —
/// see [`DistanceField`]'s "Deviations from upstream".
fn posed_body(shape: &Shape, pose: &Isometry3) -> Result<Body> {
    let mut body = Body::from_shape(shape)?.ok_or_else(|| {
        Error::construct(format!(
            "distance field shapes must be Sphere, Cylinder, Cuboid, or Mesh, got {shape:?}"
        ))
    })?;
    body.set_pose(*pose);
    Ok(body)
}

/// Distance and gradient at a queried world point. Return value of
/// [`DistanceField::distance_gradient`], upstream
/// `DistanceField::getDistanceGradient`'s out-parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DistanceGradient {
    /// The distance value at the queried cell (same value
    /// [`DistanceField::distance_cell`] would report for that cell).
    pub distance: f64,
    /// Points out of the obstacle. Zero magnitude where the cell is
    /// entirely surrounded by cells of the same distance (no local gradient
    /// to detect), which callers distinguish by checking the norm.
    pub gradient: Vector3<f64>,
    /// `false` for cells within one cell of the grid boundary — gradients
    /// need a 1-cell padding on every side and are not computed there.
    pub in_bounds: bool,
}

/// Computes distances from sets of 3D obstacle points to the nearest
/// obstacle.
///
/// Upstream `distance_field::DistanceField`, an abstract base class. Per
/// `PORTING-PLAN.md` D4, this port uses a trait instead of virtual
/// inheritance — [`crate::PropagationDistanceField`] is the (currently only)
/// implementer, matching upstream's `PropagationDistanceField`.
///
/// # Deviations from upstream
///
/// Upstream's base class also declares `addShapeToField`,
/// `removeShapeFromField`, `moveShapeInField`, `addOcTreeToField` and the
/// RViz marker-generating methods (`getIsoSurfaceMarkers`,
/// `getGradientMarkers`, `getPlaneMarkers`, `getProjectionPlanes`).
///
/// [`DistanceField::add_shape_to_field`], [`DistanceField::remove_shape_from_field`]
/// and [`DistanceField::move_shape_in_field`] *are* ported, as default trait
/// methods built from the required methods above plus
/// [`crate::find_internal_points_convex`] — matching upstream's own
/// placement of `getShapePoints`/`addShapeToField`/`moveShapeInField` as
/// non-virtual methods on the `DistanceField` base class. They accept
/// exactly the shape variants [`moveit_geometry::bodies::Body::from_shape`]
/// supports (`Sphere`/`Cylinder`/`Cuboid`/`Mesh`), returning
/// [`moveit_error::Error::Construct`] for [`Shape::Cone`], [`Shape::Plane`]
/// and [`Shape::OcTree`] rather than upstream's null-deref on those —
/// matching upstream's own `createEmptyBodyFromShapeType`, which has no
/// case for them.
///
/// The rest is not ported:
///
/// - `addOcTreeToField` needs an `octomap::OcTree` equivalent, which exists
///   nowhere in this workspace; this crate owns none of that and cannot
///   invent it without guessing at a design another phase owns.
/// - The marker methods build `visualization_msgs::msg::Marker` /
///   `MarkerArray` for RViz. `PORTING-PLAN.md` D1 keeps every crate outside
///   the optional `moveit-ros` free of ROS message types; there is nothing
///   for this crate to build them into.
/// - `writeToStream`/`readFromStream` zlib-compress the occupancy grid via
///   `boost::iostreams`. No workspace dependency provides that, and nothing
///   in the ported test suite exercises it beyond a round-trip already
///   covered by the add/rebuild-equivalence tests, so it is left unported.
pub trait DistanceField {
    /// Upstream `DistanceField::getSizeX`.
    fn size_x(&self) -> f64;
    /// Upstream `DistanceField::getSizeY`.
    fn size_y(&self) -> f64;
    /// Upstream `DistanceField::getSizeZ`.
    fn size_z(&self) -> f64;
    /// Upstream `DistanceField::getOriginX`.
    fn origin_x(&self) -> f64;
    /// Upstream `DistanceField::getOriginY`.
    fn origin_y(&self) -> f64;
    /// Upstream `DistanceField::getOriginZ`.
    fn origin_z(&self) -> f64;
    /// Upstream `DistanceField::getResolution`.
    fn resolution(&self) -> f64;
    /// Upstream `DistanceField::getUninitializedDistance`.
    fn uninitialized_distance(&self) -> f64;

    /// Upstream `DistanceField::addPointsToField`.
    fn add_points_to_field(&mut self, points: &[Vector3<f64>]);
    /// Upstream `DistanceField::removePointsFromField`.
    fn remove_points_from_field(&mut self, points: &[Vector3<f64>]);
    /// Upstream `DistanceField::updatePointsInField`.
    fn update_points_in_field(&mut self, old_points: &[Vector3<f64>], new_points: &[Vector3<f64>]);
    /// Upstream `DistanceField::reset`.
    fn reset(&mut self);

    /// Upstream `DistanceField::getDistance(double,double,double)`: the
    /// distance to the closest obstacle at a world location. Returns
    /// [`DistanceField::uninitialized_distance`] (not a panic) for a
    /// location outside the grid.
    fn distance(&self, x: f64, y: f64, z: f64) -> f64;
    /// Upstream `DistanceField::getDistance(int,int,int)`. `x`, `y`, `z`
    /// must be a valid cell — implementations may panic otherwise, matching
    /// upstream's documented "must be valid or corruption occurs" contract
    /// modulo panic-instead-of-UB (see [`crate::VoxelGrid`]'s doc comment).
    fn distance_cell(&self, x: i32, y: i32, z: i32) -> f64;
    /// Upstream `DistanceField::isCellValid`.
    fn is_cell_valid(&self, x: i32, y: i32, z: i32) -> bool;
    /// Upstream `DistanceField::getXNumCells`.
    fn num_cells_x(&self) -> i32;
    /// Upstream `DistanceField::getYNumCells`.
    fn num_cells_y(&self) -> i32;
    /// Upstream `DistanceField::getZNumCells`.
    fn num_cells_z(&self) -> i32;
    /// Upstream `DistanceField::gridToWorld`.
    ///
    /// Upstream returns a `bool` here that every known implementation
    /// (`PropagationDistanceField::gridToWorld`) hard-codes to `true`
    /// regardless of input; this port drops the always-true return value
    /// rather than carry a signal no implementer produces.
    fn grid_to_world(&self, x: i32, y: i32, z: i32) -> Vector3<f64>;
    /// Upstream `DistanceField::worldToGrid`. The `bool` reports whether the
    /// computed cell is actually valid, and the indices are still returned
    /// when it is not.
    fn world_to_grid(&self, world: &Vector3<f64>) -> (bool, i32, i32, i32);

    /// Upstream `DistanceField::getDistanceGradient`.
    ///
    /// This has a shared, non-virtual implementation upstream (defined once
    /// in `distance_field.cpp` against the abstract interface); this port
    /// mirrors that with a default trait method built only from the other
    /// required methods above.
    fn distance_gradient(&self, x: f64, y: f64, z: f64) -> DistanceGradient {
        let (_valid, gx, gy, gz) = self.world_to_grid(&Vector3::new(x, y, z));

        // Gradients need a cell of padding on every side.
        if gx < 1
            || gy < 1
            || gz < 1
            || gx >= self.num_cells_x() - 1
            || gy >= self.num_cells_y() - 1
            || gz >= self.num_cells_z() - 1
        {
            return DistanceGradient {
                distance: self.uninitialized_distance(),
                gradient: Vector3::zeros(),
                in_bounds: false,
            };
        }

        // Upstream stores this scale factor in a field mistyped as `int`
        // (`inv_twice_resolution_` in distance_field.hpp), silently
        // truncating it. Both of the ported upstream tests use resolutions
        // where 1.0/(2*resolution) happens to be an exact integer (0.1 and
        // 0.02), so the truncation is a no-op there and invisible either
        // way. This port never stores the intermediate at all — it is
        // trivially derivable from `resolution()` — so the bug has no
        // opportunity to reappear for other resolutions.
        let inv_twice_resolution = 1.0 / (2.0 * self.resolution());

        let gradient = Vector3::new(
            (self.distance_cell(gx + 1, gy, gz) - self.distance_cell(gx - 1, gy, gz))
                * inv_twice_resolution,
            (self.distance_cell(gx, gy + 1, gz) - self.distance_cell(gx, gy - 1, gz))
                * inv_twice_resolution,
            (self.distance_cell(gx, gy, gz + 1) - self.distance_cell(gx, gy, gz - 1))
                * inv_twice_resolution,
        );

        DistanceGradient {
            distance: self.distance_cell(gx, gy, gz),
            gradient,
            in_bounds: true,
        }
    }

    /// Upstream `DistanceField::addShapeToField`: sample `shape` at `pose`
    /// onto the field's resolution grid and add every point found inside it
    /// as an obstacle.
    ///
    /// # Errors
    ///
    /// See this trait's "Deviations from upstream" for the shape variants
    /// this supports.
    fn add_shape_to_field(&mut self, shape: &Shape, pose: &Isometry3) -> Result<()> {
        let body = posed_body(shape, pose)?;
        let mut points = Vec::new();
        find_internal_points_convex(&body, self.resolution(), &mut points);
        self.add_points_to_field(&points);
        Ok(())
    }

    /// Upstream `DistanceField::removeShapeFromField`.
    ///
    /// # Errors
    ///
    /// See this trait's "Deviations from upstream" for the shape variants
    /// this supports.
    fn remove_shape_from_field(&mut self, shape: &Shape, pose: &Isometry3) -> Result<()> {
        let body = posed_body(shape, pose)?;
        let mut points = Vec::new();
        find_internal_points_convex(&body, self.resolution(), &mut points);
        self.remove_points_from_field(&points);
        Ok(())
    }

    /// Upstream `DistanceField::moveShapeInField`: remove the obstacle
    /// points `shape` occupies at `old_pose` and add the ones it occupies at
    /// `new_pose`, via a single [`DistanceField::update_points_in_field`]
    /// call rather than a separate remove-then-add pass, matching upstream.
    ///
    /// # Errors
    ///
    /// See this trait's "Deviations from upstream" for the shape variants
    /// this supports.
    fn move_shape_in_field(
        &mut self,
        shape: &Shape,
        old_pose: &Isometry3,
        new_pose: &Isometry3,
    ) -> Result<()> {
        let old_body = posed_body(shape, old_pose)?;
        let mut old_points = Vec::new();
        find_internal_points_convex(&old_body, self.resolution(), &mut old_points);

        let new_body = posed_body(shape, new_pose)?;
        let mut new_points = Vec::new();
        find_internal_points_convex(&new_body, self.resolution(), &mut new_points);

        self.update_points_in_field(&old_points, &new_points);
        Ok(())
    }
}
