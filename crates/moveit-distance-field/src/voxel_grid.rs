// Copyright (c) 2009, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/distance_field/include/moveit/distance_field/voxel_grid.hpp

use moveit_error::{Error, Result};
use nalgebra::Vector3;

/// Which axis a [`VoxelGrid`] query or dimension refers to.
///
/// Upstream `distance_field::Dimension` (`DIM_X`/`DIM_Y`/`DIM_Z`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Dimension {
    /// `DIM_X`
    X = 0,
    /// `DIM_Y`
    Y = 1,
    /// `DIM_Z`
    Z = 2,
}

/// The size, origin and resolution of a [`VoxelGrid`] or
/// [`crate::PropagationDistanceField`], bundled into one type.
///
/// Upstream spells these as six separate `size_x`/`size_y`/`size_z`/
/// `origin_x`/`origin_y`/`origin_z` `f64` constructor arguments (plus
/// `resolution`), which the type checker cannot tell apart — a caller that
/// transposes a size and an origin compiles silently into a corrupt grid.
/// Bundling `size` and `origin` into [`Vector3`] fields makes them
/// distinguishable by type-shape at the call site instead.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridGeometry {
    /// World-space extent along x, y, z.
    pub size: Vector3<f64>,
    /// World-space location of cell `(0, 0, 0)`'s corner.
    pub origin: Vector3<f64>,
    /// The edge length of one (cubic) cell.
    pub resolution: f64,
}

impl GridGeometry {
    /// Build a grid geometry, validating what
    /// `VoxelGrid`/`PropagationDistanceField`'s upstream constructors do not
    /// (see [`VoxelGrid::new`]'s former "Deviations from upstream" note,
    /// folded into this type since the checks moved here): a non-positive
    /// `resolution` or a negative `size` divides/multiplies silently into
    /// infinite or NaN cell counts upstream.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when `resolution` is not finite and positive, or
    /// any `size` component is not finite and non-negative.
    pub fn new(size: Vector3<f64>, origin: Vector3<f64>, resolution: f64) -> Result<Self> {
        if !(resolution.is_finite() && resolution > 0.0) {
            return Err(Error::construct(format!(
                "resolution must be finite and positive, got {resolution}"
            )));
        }
        for (name, value) in [("size.x", size.x), ("size.y", size.y), ("size.z", size.z)] {
            if !(value.is_finite() && value >= 0.0) {
                return Err(Error::construct(format!(
                    "{name} must be finite and non-negative, got {value}"
                )));
            }
        }
        Ok(Self {
            size,
            origin,
            resolution,
        })
    }
}

/// A dense, axis-aligned 3D grid of `T`, addressable by either integer cell
/// index or world coordinate.
///
/// Upstream `distance_field::VoxelGrid<T>`.
///
/// # Deviations from upstream
///
/// - Upstream's default constructor plus `resize()` exists so a `VoxelGrid`
///   can be built before its size is known; nothing in this crate's only
///   consumer ([`crate::PropagationDistanceField`]) ever does that, so this
///   port carries only the sized constructor ([`VoxelGrid::new`]) and drops
///   `resize`/the default constructor as unused surface.
/// - Upstream's constructor allocates `new T[n]` and leaves every cell
///   default-constructed (for `PropDistanceFieldVoxel`, explicitly
///   *uninitialized* — see that type's doc comment); the sole real caller
///   ([`crate::PropagationDistanceField::new`]) immediately overwrites every
///   cell via `reset()` regardless. Safe Rust has no uninitialized-memory
///   escape hatch without `unsafe` (forbidden workspace-wide), so this port
///   fills every cell with `default_object.clone()` at construction. This is
///   unobservable given the immediate-`reset()` calling convention every
///   known caller follows.
/// - `getCell`/`setCell` document "corruption and/or SEGFAULTS" for
///   out-of-bounds indices; this port panics instead (Rust's `Vec` indexing
///   already does this — no `unsafe` is used to intentionally skip the
///   bounds check).
/// - Upstream declares five `Eigen::Vector3`-taking convenience overloads
///   next to their scalar equivalents — `operator()(const Eigen::Vector3d&)`,
///   `getCell(const Eigen::Vector3i&)` (const and non-const),
///   `setCell(const Eigen::Vector3i&, const T&)` and
///   `isCellValid(const Eigen::Vector3i&)` — each a one-line forward to the
///   scalar overload beside it. This port drops all five: `rg` over every
///   call site of the scalar equivalents in this crate shows each one
///   already holding three separate `i32`/`f64` components (a loop index, a
///   destructured tuple) at the point it calls in, never an
///   `Eigen::Vector3`-shaped value, so there is nothing for a Vector3-taking
///   overload to save. [`VoxelGrid::grid_to_world`] and
///   [`VoxelGrid::world_to_grid`] below are the two exceptions — see their
///   own doc comments for why those two *do* each carry one Vector3 shape.
/// - `data_ptrs_` (`T*** data_ptrs_`, a protected field) is dead code in the
///   *upstream* source, not merely unneeded in Rust: `rg -n "data_ptrs_"`
///   against the full `moveit_core/distance_field` tree returns only its own
///   declaration line — never initialized, assigned, or read anywhere,
///   including by the constructor/destructor that own `data_`. Nothing to
///   port. `num_cells_total_` is dropped for the ordinary reason instead:
///   this port reads `data.len()` wherever upstream would read it.
#[derive(Debug, Clone)]
pub struct VoxelGrid<T> {
    data: Vec<T>,
    default_object: T,
    size: [f64; 3],
    resolution: f64,
    oo_resolution: f64,
    origin: [f64; 3],
    origin_minus: [f64; 3],
    num_cells: [i32; 3],
    stride1: i32,
    stride2: i32,
}

impl<T: Clone> VoxelGrid<T> {
    /// Upstream `VoxelGrid::VoxelGrid(size_x, size_y, size_z, resolution,
    /// origin_x, origin_y, origin_z, default_object)`. Infallible: `geometry`
    /// is already validated by [`GridGeometry::new`] — see that type's doc
    /// comment for what upstream fails to check here.
    pub fn new(geometry: GridGeometry, default_object: T) -> Self {
        let GridGeometry {
            size,
            origin,
            resolution,
        } = geometry;

        let oo_resolution = 1.0 / resolution;
        let size = [size.x, size.y, size.z];
        let origin = [origin.x, origin.y, origin.z];
        let origin_minus = [
            origin[0] - 0.5 * resolution,
            origin[1] - 0.5 * resolution,
            origin[2] - 0.5 * resolution,
        ];
        // Matches upstream exactly: truncating multiplication, not the
        // rounded `getCellFromLocation` formula.
        let num_cells = [
            (size[0] * oo_resolution) as i32,
            (size[1] * oo_resolution) as i32,
            (size[2] * oo_resolution) as i32,
        ];
        let num_cells_total = (num_cells[0] as i64) * (num_cells[1] as i64) * (num_cells[2] as i64);
        let stride1 = num_cells[1] * num_cells[2];
        let stride2 = num_cells[2];

        Self {
            data: vec![default_object.clone(); num_cells_total.max(0) as usize],
            default_object,
            size,
            resolution,
            oo_resolution,
            origin,
            origin_minus,
            num_cells,
            stride1,
            stride2,
        }
    }

    /// Upstream `VoxelGrid::reset`: sets every cell to `initial`.
    pub fn reset(&mut self, initial: T) {
        self.data.fill(initial);
    }

    /// Upstream `VoxelGrid::getSize`.
    pub fn size(&self, dim: Dimension) -> f64 {
        self.size[dim as usize]
    }

    /// Upstream `VoxelGrid::getResolution`.
    pub fn resolution(&self) -> f64 {
        self.resolution
    }

    /// Upstream `VoxelGrid::getOrigin`.
    pub fn origin(&self, dim: Dimension) -> f64 {
        self.origin[dim as usize]
    }

    /// Upstream `VoxelGrid::getNumCells`.
    pub fn num_cells(&self, dim: Dimension) -> i32 {
        self.num_cells[dim as usize]
    }

    /// Upstream `VoxelGrid::isCellValid(int,int,int)`.
    pub fn is_cell_valid(&self, x: i32, y: i32, z: i32) -> bool {
        x >= 0
            && x < self.num_cells[0]
            && y >= 0
            && y < self.num_cells[1]
            && z >= 0
            && z < self.num_cells[2]
    }

    /// Upstream `VoxelGrid::isCellValid(Dimension,int)`.
    pub fn is_cell_valid_dim(&self, dim: Dimension, cell: i32) -> bool {
        cell >= 0 && cell < self.num_cells[dim as usize]
    }

    fn index(&self, x: i32, y: i32, z: i32) -> usize {
        (x * self.stride1 + y * self.stride2 + z) as usize
    }

    /// Upstream `VoxelGrid::getCell(int,int,int) const`. Panics (in place of
    /// upstream's documented "corruption and/or SEGFAULTS") if the cell is
    /// invalid.
    pub fn get_cell(&self, x: i32, y: i32, z: i32) -> &T {
        &self.data[self.index(x, y, z)]
    }

    /// Mutable counterpart of [`VoxelGrid::get_cell`]. Upstream
    /// `VoxelGrid::getCell(int,int,int)` (non-const overload).
    pub fn get_cell_mut(&mut self, x: i32, y: i32, z: i32) -> &mut T {
        let idx = self.index(x, y, z);
        &mut self.data[idx]
    }

    /// Upstream `VoxelGrid::setCell`.
    pub fn set_cell(&mut self, x: i32, y: i32, z: i32, value: T) {
        let idx = self.index(x, y, z);
        self.data[idx] = value;
    }

    /// Upstream `VoxelGrid::operator()(double,double,double)`: returns the
    /// default object for an out-of-bounds world location instead of
    /// panicking.
    pub fn get(&self, x: f64, y: f64, z: f64) -> &T {
        let cx = self.cell_from_location(Dimension::X, x);
        let cy = self.cell_from_location(Dimension::Y, y);
        let cz = self.cell_from_location(Dimension::Z, z);
        if !self.is_cell_valid(cx, cy, cz) {
            return &self.default_object;
        }
        self.get_cell(cx, cy, cz)
    }

    /// Upstream `VoxelGrid::getCellFromLocation`.
    ///
    /// This is the rounding convention that matters at cell boundaries: it
    /// is *not* a plain `floor((loc - origin) / resolution)`. Upstream
    /// documents it as `floor((loc - origin) / resolution + 0.5)` — the
    /// nearest-cell-center rounding — implemented via a pre-shifted origin
    /// (`origin_minus = origin - 0.5 * resolution`) for speed. Both forms are
    /// algebraically identical; this port keeps upstream's pre-shifted-origin
    /// form rather than the documented formula so the two can never drift.
    pub fn cell_from_location(&self, dim: Dimension, loc: f64) -> i32 {
        ((loc - self.origin_minus[dim as usize]) * self.oo_resolution).floor() as i32
    }

    /// Upstream `VoxelGrid::getLocationFromCell`: the world-space center of
    /// cell `cell` along `dim`.
    pub fn location_from_cell(&self, dim: Dimension, cell: i32) -> f64 {
        self.origin[dim as usize] + self.resolution * f64::from(cell)
    }

    /// Upstream `VoxelGrid::gridToWorld(int,int,int,...)`, collapsing that
    /// overload and its `gridToWorld(const Eigen::Vector3i&, Eigen::Vector3d&)`
    /// sibling into one method that takes scalar `i32`s in and returns an
    /// owned `Vector3<f64>` out — the opposite input/output shape from
    /// [`VoxelGrid::world_to_grid`] below, deliberately, not an
    /// inconsistency: every call site of this method in this crate already
    /// holds `x`/`y`/`z` as three separate loop indices or a destructured
    /// tuple, never an `Eigen::Vector3i`-shaped value, so a
    /// `Vector3<i32>`-taking overload would only force a repack the caller
    /// does not need; the `Vector3<f64>` return, in turn, is what every
    /// caller immediately wants a world point *as*.
    pub fn grid_to_world(&self, x: i32, y: i32, z: i32) -> Vector3<f64> {
        Vector3::new(
            self.location_from_cell(Dimension::X, x),
            self.location_from_cell(Dimension::Y, y),
            self.location_from_cell(Dimension::Z, z),
        )
    }

    /// Upstream `VoxelGrid::worldToGrid`, collapsing that overload and its
    /// `worldToGrid(const Eigen::Vector3d&, Eigen::Vector3i&)` sibling into
    /// one method that takes a `&Vector3<f64>` in — the shape every call
    /// site of this method in this crate already holds a world point in
    /// (constructed fresh or read out of a points slice), unlike
    /// [`VoxelGrid::grid_to_world`] above (see that method's own doc for the
    /// matching rationale on its side). Returns the computed indices
    /// alongside whether they are valid as a plain tuple rather than a named
    /// struct, since every caller destructures it immediately — matching
    /// upstream's "the returned indices will be computed even if they are
    /// invalid" contract.
    pub fn world_to_grid(&self, world: &Vector3<f64>) -> (bool, i32, i32, i32) {
        let x = self.cell_from_location(Dimension::X, world.x);
        let y = self.cell_from_location(Dimension::Y, world.y);
        let z = self.cell_from_location(Dimension::Z, world.z);
        (self.is_cell_valid(x, y, z), x, y, z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A cube grid of the given size and resolution, origin at zero — every
    /// test below except [`rejects_non_positive_resolution`] only varies
    /// these two.
    fn cube_geometry(size: f64, resolution: f64) -> GridGeometry {
        GridGeometry::new(Vector3::new(size, size, size), Vector3::zeros(), resolution).unwrap()
    }

    #[test]
    fn dimensions_match_upstream_test_read_write() {
        let vg = VoxelGrid::new(cube_geometry(0.02, 0.01), -100);
        assert_eq!(vg.num_cells(Dimension::X), 2);
        assert_eq!(vg.num_cells(Dimension::Y), 2);
        assert_eq!(vg.num_cells(Dimension::Z), 2);
    }

    #[test]
    fn reset_then_set_round_trips_every_cell() {
        let mut vg = VoxelGrid::new(cube_geometry(0.02, 0.01), -100);
        vg.reset(0);
        for x in 0..2 {
            for y in 0..2 {
                for z in 0..2 {
                    assert_eq!(*vg.get_cell(x, y, z), 0);
                }
            }
        }
        let mut i = 0;
        for x in 0..2 {
            for y in 0..2 {
                for z in 0..2 {
                    vg.set_cell(x, y, z, i);
                    i += 1;
                }
            }
        }
        i = 0;
        for x in 0..2 {
            for y in 0..2 {
                for z in 0..2 {
                    assert_eq!(*vg.get_cell(x, y, z), i);
                    i += 1;
                }
            }
        }
    }

    #[test]
    fn rejects_non_positive_resolution() {
        let size = Vector3::new(1.0, 1.0, 1.0);
        assert!(GridGeometry::new(size, Vector3::zeros(), 0.0).is_err());
        assert!(GridGeometry::new(size, Vector3::zeros(), -0.1).is_err());
    }

    #[test]
    fn cell_from_location_rounds_to_nearest_cell_center() {
        // resolution 1.0, origin 0.0: cell k covers world [k-0.5, k+0.5).
        let vg = VoxelGrid::new(cube_geometry(10.0, 1.0), 0);
        assert_eq!(vg.cell_from_location(Dimension::X, 0.0), 0);
        assert_eq!(vg.cell_from_location(Dimension::X, 0.49), 0);
        // exactly on the upper boundary rounds up to the next cell: floor
        // matches upstream bit-for-bit since both use the same expression.
        assert_eq!(vg.cell_from_location(Dimension::X, 0.5), 1);
        assert_eq!(vg.cell_from_location(Dimension::X, -0.5), 0);
        assert_eq!(vg.cell_from_location(Dimension::X, -0.51), -1);
    }

    #[test]
    fn world_to_grid_reports_invalid_outside_the_grid_but_still_computes_indices() {
        let vg = VoxelGrid::new(cube_geometry(1.0, 0.1), 0);
        let (valid, x, y, z) = vg.world_to_grid(&Vector3::new(1000.0, 1000.0, 1000.0));
        assert!(!valid);
        assert!(x > 0 && y > 0 && z > 0);
    }

    #[test]
    fn grid_to_world_round_trips_cell_centers() {
        let vg = VoxelGrid::new(cube_geometry(1.0, 0.1), 0);
        for cell in [0, 3, 9] {
            let world = vg.grid_to_world(cell, cell, cell);
            let (valid, x, y, z) = vg.world_to_grid(&world);
            assert!(valid);
            assert_eq!((x, y, z), (cell, cell, cell));
        }
    }
}
