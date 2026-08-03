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
use moveit_octomap::OcTree;
use nalgebra::{Point3, Vector3};

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

/// Upstream `DistanceField::getOcTreePoints` (protected — no caller outside
/// [`DistanceField::add_octree_to_field`], so this stays a private free
/// function rather than a trait method).
///
/// `bbx_min`/`bbx_max` are `grid_to_world(0, 0, 0)` and
/// `grid_to_world(num_cells_x, num_cells_y, num_cells_z)` — the latter one
/// cell past the last valid index, matching upstream's own
/// `gridToWorld(num_x, num_y, num_z, ...)` call, a pure extrapolation
/// [`crate::VoxelGrid::grid_to_world`] computes with no bounds check, same
/// as upstream's `VoxelGrid::gridToWorld`.
///
/// `None` from [`OcTree::leaves_in_bbx`] (the field's own grid extent is
/// outside the octree's representable coordinate range) yields zero points,
/// matching upstream's `coordToKeyChecked` failure case, which upstream
/// handles by producing an immediately-empty iterator rather than an error.
///
/// **The subdivision loop's `<=` upper bound does not reliably include the
/// last face** (PORTING-PLAN.md §96.3) — faithfully so, upstream's own
/// `for (double x = ...; x <= ...; x += resolution_)` has the identical
/// susceptibility. `ceil(leaf.size() / resolution)` only guarantees the
/// swept interval is a multiple of `resolution` in exact arithmetic; it
/// does not guarantee that accumulating `resolution` by repeated `+=`
/// reproduces the independently-rounded upper bound bit-for-bit. Measured
/// directly: sweeping realistic `(field_resolution, octree_resolution,
/// insert_point)` triples through real octree leaves, the actual per-axis
/// point count fell short of the intended `ceil(size / resolution) + 1` in
/// 176 of 448 cases (39%) — common, and per-axis (one axis of a leaf can
/// drop its last face while the other two do not), not a function of `k`'s
/// parity or any other predictable property. See this module's
/// `octree_points_subdivision_drops_the_last_face_for_an_even_k_boundary`/
/// `octree_points_subdivision_boundary_outcome_is_per_axis_not_per_k` tests
/// for two concrete, deterministic instances, plus a third power-of-two
/// case in `octree_points_subdivision_le_boundary_keeps_the_last_face_that_lt_would_drop`.
/// Whether upstream's C++ produces the identical per-instance count *and*
/// emission order for the same inputs was an open cross-language question
/// — floating-point accumulation order is the same source-language
/// construct on both sides, but compiler-level reassociation (FMA
/// contraction, `-ffast-math`-class flags) could in principle diverge from
/// Rust's strict left-to-right `+=`. It no longer is: PORTING-PLAN.md
/// §102's `octree_points` oracle op ran all three cases against the real
/// `moveit2` C++ and every point matched, bit-for-bit and in emission
/// order, not just the count (`octree_points_matches_the_oracle_for_all_three_pinned_boundary_cases`,
/// backed by `tests/fixtures/octree_points_request.json`/`_response.json`).
/// The 39% last-face-drop rate is a property of the loop both
/// implementations run, not a divergence this port introduced.
fn octree_points(
    bbx_min: Vector3<f64>,
    bbx_max: Vector3<f64>,
    resolution: f64,
    octree: &OcTree,
) -> Vec<Vector3<f64>> {
    let mut points = Vec::new();
    let Some(leaves) = octree.leaves_in_bbx(Point3::from(bbx_min), Point3::from(bbx_max)) else {
        return points;
    };
    for leaf in leaves {
        if !leaf.is_occupied() {
            continue;
        }
        let coord = leaf.coordinate();
        if leaf.size() <= resolution {
            points.push(Vector3::new(coord.x, coord.y, coord.z));
            continue;
        }
        let ceil_val = (leaf.size() / resolution).ceil() * resolution / 2.0;
        let mut x = coord.x - ceil_val;
        while x <= coord.x + ceil_val {
            let mut y = coord.y - ceil_val;
            while y <= coord.y + ceil_val {
                let mut z = coord.z - ceil_val;
                while z <= coord.z + ceil_val {
                    points.push(Vector3::new(x, y, z));
                    z += resolution;
                }
                y += resolution;
            }
            x += resolution;
        }
    }
    points
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
/// [`DistanceField::add_shape_to_field`], [`DistanceField::remove_shape_from_field`],
/// [`DistanceField::move_shape_in_field`] and [`DistanceField::add_octree_to_field`]
/// *are* ported, as default trait methods built from the required methods
/// above plus [`crate::find_internal_points_convex`]/`octree_points` —
/// matching upstream's own placement of
/// `getShapePoints`/`addShapeToField`/`moveShapeInField`/`addOcTreeToField`
/// as non-virtual methods on the `DistanceField` base class. The shape
/// methods accept exactly the shape variants
/// [`moveit_geometry::bodies::Body::from_shape`] supports
/// (`Sphere`/`Cylinder`/`Cuboid`/`Mesh`), returning
/// [`moveit_error::Error::Construct`] for [`Shape::Cone`], [`Shape::Plane`]
/// and [`Shape::OcTree`] rather than upstream's null-deref on those —
/// matching upstream's own `createEmptyBodyFromShapeType`, which has no
/// case for them. [`DistanceField::add_octree_to_field`] takes a
/// [`moveit_octomap::OcTree`] directly instead, against the
/// `moveit-octomap` dependency added for
/// [`crate::PosedBodyPointDecomposition::from_octree`] — a different,
/// simpler point-collection algorithm from that method (see its own doc):
/// occupied leaves only, bounding-box-clipped to this field's own grid
/// extent, oversized leaves subdivided to `resolution` spacing rather than
/// every tree node unfiltered.
///
/// The rest is not ported:
///
/// - The marker methods build `visualization_msgs::msg::Marker` /
///   `MarkerArray` for RViz. `PORTING-PLAN.md` D1 keeps every crate outside
///   the optional `moveit-ros` free of ROS message types; there is nothing
///   for this crate to build them into.
/// - `writeToStream`/`readFromStream` zlib-compress the occupancy grid via
///   `boost::iostreams`. No workspace dependency provides that, and nothing
///   in the ported test suite exercises it beyond a round-trip already
///   covered by the add/rebuild-equivalence tests, so it is left unported.
/// - The octree-and-bounding-box-taking `PropagationDistanceField`
///   constructor overload (`propagation_distance_field.hpp`) is a separate
///   item from `addOcTreeToField` — see
///   [`crate::PropagationDistanceField`]'s "Deviations from upstream" — and
///   stays unported.
/// - `setPoint` (protected) has no caller left once the marker methods above
///   are unported — upstream's only caller of it is `getProjectionPlanes`.
/// - The base class constructor and destructor have no Rust counterpart:
///   this trait carries no state of its own. The seven size/origin/
///   resolution constructor arguments upstream's base class stores become
///   [`crate::GridGeometry`] plus `max_distance`/`propagate_negative_distances`
///   on [`crate::PropagationDistanceField`], the implementer that actually
///   owns them; Rust's ownership model needs no destructor for what a trait
///   itself never allocates.
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

    /// Upstream `DistanceField::addOcTreeToField`: every occupied leaf of
    /// `octree` that overlaps this field's own grid extent becomes one or
    /// more obstacle points, via `octree_points`. A leaf no larger than
    /// [`DistanceField::resolution`] contributes its own center; a larger
    /// leaf is subdivided into a sub-grid of points at `resolution` spacing
    /// covering the leaf's full extent, rather than collapsing it to one
    /// point — so a coarse octree still yields obstacle density comparable
    /// to a fine one at this field's resolution.
    fn add_octree_to_field(&mut self, octree: &OcTree) {
        let bbx_min = self.grid_to_world(0, 0, 0);
        let bbx_max =
            self.grid_to_world(self.num_cells_x(), self.num_cells_y(), self.num_cells_z());
        let points = octree_points(bbx_min, bbx_max, self.resolution(), octree);
        self.add_points_to_field(&points);
    }
}

/// The first three tests below each pin one piece of [`octree_points`]'s
/// behavior (occupancy filter, subdivision, bounding-box clip) and were each
/// checked against a mutation that removes exactly that piece: dropping the
/// occupancy check made `octree_points_excludes_unoccupied_leaves` fail
/// (`2` points instead of `1`); collapsing subdivision to a single center
/// point made `octree_points_subdivides_a_leaf_larger_than_resolution` fail
/// (`1` instead of `64`); replacing the bbox-clipped iterator with an
/// unfiltered one made `octree_points_excludes_leaves_outside_the_bounding_box`
/// fail (`2` instead of `1`) while leaving the fourth test
/// (`add_octree_to_field_wires_the_fields_own_extent_through`) passing --
/// confirming that test alone would not have caught a missing bbox clip,
/// since [`crate::propagation::PropagationDistanceField::add_points_to_field`]'s
/// own out-of-grid check silently absorbs the difference at that level. Each
/// mutation was reverted after confirming.
#[cfg(test)]
mod tests {
    use serde::Deserialize;

    use super::*;
    use crate::propagation::PropagationDistanceField;
    use crate::voxel_grid::GridGeometry;

    const RESOLUTION: f64 = 0.1;
    const BBX_MIN: Vector3<f64> = Vector3::new(0.0, 0.0, 0.0);
    const BBX_MAX: Vector3<f64> = Vector3::new(1.0, 1.0, 1.0);

    fn field() -> PropagationDistanceField {
        let geometry = GridGeometry::new(
            Vector3::new(1.0, 1.0, 1.0),
            Vector3::new(0.0, 0.0, 0.0),
            RESOLUTION,
        )
        .unwrap();
        PropagationDistanceField::new(geometry, 0.3, false).unwrap()
    }

    /// Pin for the occupancy filter in [`octree_points`]: an explicitly-free
    /// leaf (`update_node(_, false, _)`) must not contribute a point, only
    /// the occupied one does. Both leaves are sized to match `RESOLUTION`
    /// exactly, so neither triggers subdivision — isolates the occupancy
    /// check from the subdivision behavior covered separately below.
    #[test]
    fn octree_points_excludes_unoccupied_leaves() {
        let mut tree = OcTree::new(RESOLUTION);
        tree.update_node(Point3::new(0.35, 0.35, 0.35), true, false);
        tree.update_node(Point3::new(0.75, 0.75, 0.75), false, false);

        let points = octree_points(BBX_MIN, BBX_MAX, RESOLUTION, &tree);

        assert_eq!(points.len(), 1, "the free leaf must not contribute a point");
    }

    /// Pin for the subdivision step in [`octree_points`]: a leaf larger than
    /// `RESOLUTION` must contribute more than one point (its full extent
    /// sub-sampled at `RESOLUTION` spacing), not just its own center. Reads
    /// the octree's own leaf back rather than assuming octomap's exact
    /// rounding of the inserted point, so the expected count is derived from
    /// what the tree actually built.
    #[test]
    fn octree_points_subdivides_a_leaf_larger_than_resolution() {
        let octree_resolution = 0.3;
        let mut tree = OcTree::new(octree_resolution);
        tree.update_node(Point3::new(0.5, 0.5, 0.5), true, false);

        let leaf = tree.leaves().next().expect("one leaf was inserted");
        assert!(
            leaf.size() > RESOLUTION,
            "test setup must produce a leaf larger than the field resolution \
             for this to actually exercise subdivision, got size {}",
            leaf.size()
        );
        let half_width = (leaf.size() / RESOLUTION).ceil() * RESOLUTION / 2.0;
        let steps_per_axis = ((2.0 * half_width) / RESOLUTION).round() as usize + 1;
        let expected_points = steps_per_axis.pow(3);
        assert!(
            expected_points > 1,
            "test setup must predict more than one point for this pin to mean anything"
        );

        let points = octree_points(BBX_MIN, BBX_MAX, RESOLUTION, &tree);

        assert_eq!(points.len(), expected_points);
    }

    /// Pin for the bounding-box clip in [`octree_points`]: an occupied leaf
    /// entirely outside `[bbx_min, bbx_max]` must not contribute a point,
    /// even though it is occupied. Checked at the `octree_points` level
    /// specifically (not through [`DistanceField::add_octree_to_field`]),
    /// since [`crate::propagation::PropagationDistanceField::add_points_to_field`]
    /// has its own redundant out-of-grid check that would mask a missing
    /// bbox clip here.
    #[test]
    fn octree_points_excludes_leaves_outside_the_bounding_box() {
        let mut tree = OcTree::new(RESOLUTION);
        tree.update_node(Point3::new(5.0, 5.0, 5.0), true, false);
        tree.update_node(Point3::new(0.35, 0.35, 0.35), true, false);

        let points = octree_points(BBX_MIN, BBX_MAX, RESOLUTION, &tree);

        assert_eq!(
            points.len(),
            1,
            "the out-of-bbox leaf must not contribute a point"
        );
    }

    /// [`DistanceField::add_octree_to_field`] must actually wire the field's
    /// own grid extent and resolution through to [`octree_points`] — the
    /// three tests above cover `octree_points` in isolation with
    /// hand-supplied bounds, this confirms the trait method computes and
    /// passes the right ones. An out-of-bbox occupied leaf combined with an
    /// in-bbox one must leave exactly one obstacle cell in the field.
    #[test]
    fn add_octree_to_field_wires_the_fields_own_extent_through() {
        let mut df = field();
        let mut tree = OcTree::new(RESOLUTION);
        tree.update_node(Point3::new(5.0, 5.0, 5.0), true, false);
        tree.update_node(Point3::new(0.35, 0.35, 0.35), true, false);

        df.add_octree_to_field(&tree);

        let mut occupied = 0;
        for x in 0..df.num_cells_x() {
            for y in 0..df.num_cells_y() {
                for z in 0..df.num_cells_z() {
                    if df.cell(x, y, z).distance_square == 0 {
                        occupied += 1;
                    }
                }
            }
        }
        assert_eq!(occupied, 1);
    }

    /// Pin for the subdivision loop's `<=` termination boundary
    /// (PORTING-PLAN.md §96.3). Both upstream (`distance_field.cpp:268-278`,
    /// `for (double x = ...; x <= ...; x += resolution_)`) and this port
    /// accumulate the loop variable by repeated floating-point addition,
    /// then compare against a separately-computed `end = coord + ceil_val`.
    /// `ceil(size / resolution)` guarantees the interval width is a
    /// multiple of `resolution` only in exact real-number arithmetic; it
    /// says nothing about whether `k` repeated `+= resolution` additions
    /// from `start` reproduce `end` bit-for-bit, because `start`/`end` are
    /// each computed with one rounding step while the accumulated value
    /// carries the compounded rounding of every addition along the way.
    /// Whichever side rounds up costs the comparison the last face.
    ///
    /// Measured directly (not assumed): sweeping realistic
    /// `(field_resolution, octree_resolution, insert_point)` combinations
    /// through real [`OcTree`] leaves and comparing the actual per-axis
    /// point count against the intended `ceil(size / resolution) + 1`
    /// (both ends included) found the boundary dropped in 176 of 448
    /// combinations (39%) -- common, not a rare corner. This test pins one
    /// fully deterministic instance with `ceil(size / resolution) = 10`
    /// (an even count): a `0.1`-sized leaf at `octree_resolution = 0.1`,
    /// field `resolution = 0.01`, centered at `(0.35, 0.35, 0.35)`. All
    /// three axes independently drop their last face -- the naive
    /// `(k + 1)^3 = 1331` is not what the loop produces; `10^3 = 1000` is.
    /// The companion test below pins an odd-`k` case where only one axis
    /// drops, showing the outcome is per-axis and value-specific, not a
    /// function of `k`'s parity.
    ///
    /// **This test and its companion below do not distinguish `<=` from
    /// `<`.** Both pick inputs where the accumulated last iterate already
    /// lands strictly past `end`, so `<=` and `<` agree and either
    /// operator reproduces the same (dropped) count -- flipping the
    /// operator here would not fail either test. That is a real gap: it
    /// means neither test is evidence the loop actually uses `<=` rather
    /// than `<`, only evidence of what the accumulation does at these
    /// particular inputs.
    /// [`octree_points_subdivision_le_boundary_keeps_the_last_face_that_lt_would_drop`]
    /// below closes it, with a power-of-two-resolution input chosen so the
    /// accumulation is exact and `<=`/`<` genuinely disagree.
    #[test]
    fn octree_points_subdivision_drops_the_last_face_for_an_even_k_boundary() {
        let field_resolution = 0.01;
        let mut tree = OcTree::new(0.1);
        tree.update_node(Point3::new(0.35, 0.35, 0.35), true, false);
        let leaf = tree.leaves().next().expect("one leaf was inserted");
        assert_eq!(
            leaf.size(),
            0.1,
            "test setup must produce a leaf exactly matching octree_resolution \
             for k = ceil(size / resolution) = 10 to hold"
        );

        let bbx_min = Vector3::new(-10.0, -10.0, -10.0);
        let bbx_max = Vector3::new(10.0, 10.0, 10.0);
        let points = octree_points(bbx_min, bbx_max, field_resolution, &tree);

        assert_eq!(
            points.len(),
            1000,
            "observed floating-point boundary behavior -- 10 points per axis, \
             not the mathematically-intended 11 (see this test's own doc)"
        );
    }

    /// Companion to the even-`k` pin above: `octree_resolution = 0.05`,
    /// field `resolution = 0.01` gives `k = ceil(0.05 / 0.01) = 5` (odd).
    /// At the leaf centered on `(-1.234567, 7.654321, 10.0001)`, the `x`
    /// axis drops its last face (5 points, not 6) while `y` and `z` both
    /// keep theirs (6 points each) -- `5 * 6 * 6 = 180`, not `6^3 = 216`.
    /// The same `octree_resolution`/`resolution` pair at a different leaf
    /// center (e.g. `(0.0501, 0.0501, 0.0501)`, not asserted here) keeps
    /// all three axes at 6. This is the concrete evidence that the boundary
    /// outcome is a per-axis function of the specific `f64` bit pattern of
    /// `coord ± ceil_val` versus the accumulated sum, not a predictable
    /// property of `k`, `k`'s parity, or the leaf/field resolution alone --
    /// no general "safe by construction" argument closes this, only
    /// per-instance measurement does.
    #[test]
    fn octree_points_subdivision_boundary_outcome_is_per_axis_not_per_k() {
        let field_resolution = 0.01;
        let mut tree = OcTree::new(0.05);
        tree.update_node(Point3::new(-1.234567, 7.654321, 10.0001), true, false);
        let leaf = tree.leaves().next().expect("one leaf was inserted");
        assert_eq!(
            leaf.size(),
            0.05,
            "test setup must produce a leaf exactly matching octree_resolution \
             for k = ceil(size / resolution) = 5 to hold"
        );

        let bbx_min = Vector3::new(-10.0, -10.0, -10.0);
        let bbx_max = Vector3::new(10.0, 10.0, 10.0);
        let points = octree_points(bbx_min, bbx_max, field_resolution, &tree);

        assert_eq!(
            points.len(),
            180,
            "observed floating-point boundary behavior -- x drops to 5 points, \
             y and z each keep 6 (see this test's own doc)"
        );
    }

    /// The operator pin the two tests above cannot provide (PORTING-PLAN.md
    /// §96.3): an input where `octree_resolution`/`field_resolution` are
    /// both exact powers of two, so `start`/`end`/every accumulated step
    /// are all exactly representable in `f64` and the loop's last iterate
    /// lands exactly on `end` rather than past it. At that point `<=` and
    /// `<` genuinely disagree: `octree_resolution = 0.125`, `field_resolution
    /// = 0.0625` gives `k = ceil(0.125 / 0.0625) = 2` per axis, so `<=`
    /// must visit 3 positions per axis (`3^3 = 27` points, both faces
    /// included) while `<` visits only 2 (`2^3 = 8`, the last face
    /// dropped). Flipping the loop's `<=` to `<` locally and re-running
    /// confirmed exactly this: 27 with `<=`, 8 with `<`. This is also the
    /// numeric confirmation, for a case picked to be exact rather than
    /// accumulation-dependent, that the loop's intent really is `ceil(size
    /// / resolution) + 1` points per axis, both ends included -- the same
    /// intent the two tests above show does not always survive the
    /// accumulation.
    #[test]
    fn octree_points_subdivision_le_boundary_keeps_the_last_face_that_lt_would_drop() {
        let field_resolution = 0.0625;
        let mut tree = OcTree::new(0.125);
        tree.update_node(Point3::new(0.3125, 0.3125, 0.3125), true, false);
        let leaf = tree.leaves().next().expect("one leaf was inserted");
        assert_eq!(
            leaf.size(),
            0.125,
            "test setup must produce a leaf exactly matching octree_resolution \
             for k = ceil(size / resolution) = 2 to hold"
        );

        let bbx_min = Vector3::new(-10.0, -10.0, -10.0);
        let bbx_max = Vector3::new(10.0, 10.0, 10.0);
        let points = octree_points(bbx_min, bbx_max, field_resolution, &tree);

        assert_eq!(
            points.len(),
            27,
            "power-of-two resolutions make the accumulation exact -- 3 points \
             per axis (both ends included), not 2 (see this test's own doc)"
        );
    }

    /// `serde_json` 1.0.151's direct `f64`-typed deserialization (whether
    /// into a bare `f64` field, a `[f64; N]` array, `serde_json::Value`, or
    /// a tuple struct) is not correctly rounded for some 17-significant-digit
    /// literals -- confirmed directly:
    /// `serde_json::from_str::<[f64; 3]>("[0,0,10.049999999999999]")`
    /// yields the `f64` for `10.05`, a different, wrong neighbor. This
    /// crate's fixtures compare floating-point coordinates bit-exactly, so
    /// a silently mis-rounded expected value is exactly the false result
    /// that discipline exists to catch. The fix is to never let
    /// `serde_json` parse these literals into a float itself: deserialize
    /// the source text verbatim via `serde_json::value::RawValue` (this
    /// crate's `raw_value` feature, see `Cargo.toml`) and hand it to
    /// Rust's own correctly-rounded `str::parse::<f64>` -- confirmed
    /// against both `str::parse::<f64>` directly and Python's `json`
    /// module. (`arbitrary_precision` also fixes `serde_json::Value`'s
    /// rounding, but was rejected: it breaks `#[serde(tag = "...")]`
    /// internally-tagged enum deserialization crate-wide, which broke
    /// `tests/shape_points_parity.rs`'s `ShapeSpec` and
    /// `tests/collision_distance_field_types_parity.rs`'s equivalent.)
    /// Every float-bearing field in these fixture structs therefore goes
    /// through [`de_f64`]/[`de_f64_3`]/[`de_f64_3_vec`] rather than
    /// deriving `Deserialize` directly on `f64`/`[f64; 3]`/`Vec<[f64; 3]>`.
    fn de_f64<'de, D: serde::Deserializer<'de>>(
        deserializer: D,
    ) -> std::result::Result<f64, D::Error> {
        let raw = <Box<serde_json::value::RawValue>>::deserialize(deserializer)?;
        raw.get()
            .parse()
            .map_err(|_| serde::de::Error::custom("expected a JSON number"))
    }

    fn de_f64_3<'de, D: serde::Deserializer<'de>>(
        deserializer: D,
    ) -> std::result::Result<[f64; 3], D::Error> {
        let raw: [Box<serde_json::value::RawValue>; 3] = Deserialize::deserialize(deserializer)?;
        let mut out = [0.0; 3];
        for (slot, value) in out.iter_mut().zip(raw.iter()) {
            *slot = value
                .get()
                .parse()
                .map_err(|_| serde::de::Error::custom("expected a JSON number"))?;
        }
        Ok(out)
    }

    fn de_f64_3_vec<'de, D: serde::Deserializer<'de>>(
        deserializer: D,
    ) -> std::result::Result<Vec<[f64; 3]>, D::Error> {
        let raw: Vec<[Box<serde_json::value::RawValue>; 3]> =
            Deserialize::deserialize(deserializer)?;
        raw.iter()
            .map(|triple| {
                let mut out = [0.0; 3];
                for (slot, value) in out.iter_mut().zip(triple.iter()) {
                    *slot = value
                        .get()
                        .parse()
                        .map_err(|_| serde::de::Error::custom("expected a JSON number"))?;
                }
                Ok(out)
            })
            .collect()
    }

    #[derive(Deserialize)]
    struct OracleGeometry {
        #[serde(deserialize_with = "de_f64_3")]
        size: [f64; 3],
        #[serde(deserialize_with = "de_f64_3")]
        origin: [f64; 3],
        #[serde(deserialize_with = "de_f64")]
        resolution: f64,
    }

    #[derive(Deserialize)]
    struct OracleAction {
        #[serde(rename = "type")]
        action_type: String,
        #[serde(deserialize_with = "de_f64_3")]
        point: [f64; 3],
        occupied: bool,
    }

    #[derive(Deserialize)]
    struct OracleRequest {
        id: u64,
        geometry: OracleGeometry,
        #[serde(deserialize_with = "de_f64")]
        octree_resolution: f64,
        actions: Vec<OracleAction>,
    }

    #[derive(Deserialize)]
    struct OracleLeaf {
        #[serde(deserialize_with = "de_f64_3")]
        coordinate: [f64; 3],
        #[serde(deserialize_with = "de_f64")]
        size: f64,
        occupied: bool,
    }

    #[derive(Deserialize)]
    struct OracleResult {
        count: usize,
        leaves: Vec<OracleLeaf>,
        #[serde(deserialize_with = "de_f64_3_vec")]
        points: Vec<[f64; 3]>,
    }

    #[derive(Deserialize)]
    struct OracleResponse {
        id: u64,
        ok: bool,
        result: OracleResult,
    }

    fn load_fixture<T: for<'de> Deserialize<'de>>(name: &str) -> T {
        let path = format!("{}/tests/fixtures/{name}", env!("CARGO_MANIFEST_DIR"));
        let raw = std::fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
        serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
    }

    /// Closes the cross-language gap [`octree_points`]'s own doc used to
    /// flag as unverified: whether upstream's C++ produces the identical
    /// per-instance point count and emission order for the same inputs, or
    /// whether compiler-level FMA contraction makes its accumulated `x
    /// += resolution_` diverge from Rust's strict left-to-right `+=`.
    /// `tests/fixtures/octree_points_request.json`/`_response.json` capture
    /// the real `moveit2` C++ oracle's answer (via its new `octree_points`
    /// op, PORTING-PLAN.md §102) for the three deterministic cases this
    /// module already pins by count alone: the even-`k` boundary
    /// (`octree_points_subdivision_drops_the_last_face_for_an_even_k_boundary`),
    /// the odd-`k`/per-axis boundary
    /// (`octree_points_subdivision_boundary_outcome_is_per_axis_not_per_k`),
    /// and the power-of-two `<=` pin
    /// (`octree_points_subdivision_le_boundary_keeps_the_last_face_that_lt_would_drop`).
    /// Every point is compared bit-for-bit, in emission order, not just the
    /// count -- upstream and this port agreeing on "1000 points" says
    /// nothing about whether they are the *same* 1000 points; a grid
    /// shifted by one cell would pass a count-only check and fail this one.
    /// **Result: all three cases match upstream exactly, count and every
    /// coordinate** (PORTING-PLAN.md §102.2) -- the 39% last-face-drop rate
    /// this module measured is a property of the loop upstream and this
    /// port both implement, not a divergence this port introduced.
    #[test]
    fn octree_points_matches_the_oracle_for_all_three_pinned_boundary_cases() {
        let requests: Vec<OracleRequest> = load_fixture("octree_points_request.json");
        let responses: Vec<OracleResponse> = load_fixture("octree_points_response.json");
        assert_eq!(
            requests.len(),
            3,
            "fixture must cover all three pinned cases"
        );
        assert_eq!(requests.len(), responses.len());

        for (request, response) in requests.iter().zip(&responses) {
            assert_eq!(
                request.id, response.id,
                "request/response fixtures must stay paired by id"
            );
            assert!(response.ok, "id {}: oracle reported failure", request.id);
            assert_eq!(
                request.actions.len(),
                1,
                "id {}: this test only builds a single-leaf octree",
                request.id
            );
            let action = &request.actions[0];
            assert_eq!(action.action_type, "update_point");

            let mut tree = OcTree::new(request.octree_resolution);
            tree.update_node(Point3::from(action.point), action.occupied, false);

            let bbx_min = Vector3::from(request.geometry.origin);
            let bbx_max = bbx_min + Vector3::from(request.geometry.size);
            let points = octree_points(bbx_min, bbx_max, request.geometry.resolution, &tree);

            assert_eq!(
                points.len(),
                response.result.count,
                "id {}: point count",
                request.id
            );
            let actual: Vec<[f64; 3]> = points.iter().map(|p| [p.x, p.y, p.z]).collect();
            assert_eq!(
                actual, response.result.points,
                "id {}: emission order and coordinates must match the oracle exactly, \
                 not just the count",
                request.id
            );

            assert_eq!(
                response.result.leaves.len(),
                1,
                "id {}: fixture must contain exactly the one leaf this test built",
                request.id
            );
            let leaf = tree.leaves().next().expect("one leaf was inserted");
            let expected_leaf = &response.result.leaves[0];
            assert_eq!(
                leaf.size(),
                expected_leaf.size,
                "id {}: leaf size",
                request.id
            );
            assert_eq!(
                leaf.is_occupied(),
                expected_leaf.occupied,
                "id {}: leaf occupancy",
                request.id
            );
            let coord = leaf.coordinate();
            assert_eq!(
                [coord.x, coord.y, coord.z],
                expected_leaf.coordinate,
                "id {}: leaf coordinate",
                request.id
            );
        }
    }
}
