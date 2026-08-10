// Copyright (c) 2009, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/distance_field/include/moveit/distance_field/voxel_grid.hpp

use cspace_error::{Error, Result};
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
///
/// # Fields are `pub(crate)`, not `pub`
///
/// PORTING-PLAN.md §172.3's rule 2: a validating constructor is only a real
/// guarantee if nothing can construct the value without going through it.
/// Every field here is numeric and every one of [`GridGeometry::new`]'s
/// checks exists to keep [`VoxelGrid::new`] (below) infallible -- if the
/// fields were `pub`, any crate could build a `GridGeometry { .. }` struct
/// literal directly, skip `new`'s validation entirely, and hand
/// `VoxelGrid::new` exactly the unchecked input its own doc comment claims
/// cannot reach it. `rg -n 'GridGeometry\s*\{'` against this crate and its
/// two external consumers (`cspace-planners-chomp`) turns up only this
/// definition and `VoxelGrid::new`'s own destructuring pattern match --
/// nothing constructs one by struct literal today -- so narrowing to
/// `pub(crate)` (still freely readable/destructurable anywhere in this
/// crate) breaks no call site while closing the gap for good.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GridGeometry {
    /// World-space extent along x, y, z.
    pub(crate) size: Vector3<f64>,
    /// World-space location of cell `(0, 0, 0)`'s corner.
    pub(crate) origin: Vector3<f64>,
    /// The edge length of one (cubic) cell.
    pub(crate) resolution: f64,
}

impl GridGeometry {
    /// Build a grid geometry, validating what
    /// `VoxelGrid`/`PropagationDistanceField`'s upstream constructors do not
    /// (see [`VoxelGrid::new`]'s former "Deviations from upstream" note,
    /// folded into this type since the checks moved here): a non-positive
    /// `resolution` or a negative `size` divides/multiplies silently into
    /// infinite or NaN cell counts upstream.
    ///
    /// # §153.1: the `size / resolution > i32::MAX` check
    ///
    /// PORTING-PLAN.md §172 (float-to-integer narrowing family):
    /// [`VoxelGrid::new`] computes each axis of `num_cells` by multiplying
    /// that axis of `size` by `1.0 / resolution` and casting the result to
    /// `i32` -- upstream's own per-axis assignment (`int num_cells_ =
    /// size_ * oo_resolution_;`), a `double` expression narrowed into an
    /// `int` field, transcribed exactly. A finite, positive `size` and a
    /// finite, positive `resolution` (both already required above) do not
    /// bound that ratio: `size = 1.0, resolution = 1e-10` passes every
    /// check above and yields `1e10`, past `i32::MAX` (`2147483647`). Past
    /// that point Rust's `as i32` **saturates**; C++'s `(int)(double)`
    /// narrowing of an out-of-range value is **UB**, so there is no
    /// upstream value to compare against. `DistanceField::distance_gradient`
    /// used to carry a saturation boundary of the same shape and no longer
    /// does (it stopped reproducing upstream's narrowing at all); an
    /// ordinary `size`/`resolution` ratio reaches this one, and
    /// `VoxelGrid::new`
    /// sizes its cell storage (via `num_cells_total`) from the saturated
    /// result -- a resource exhaustion, not merely a wrong value, the same
    /// shape as §172.1's `max_distance_sq` case. This section expires if
    /// upstream adds its own bound check to `VoxelGrid::initialize`.
    ///
    /// # Errors
    ///
    /// [`Error::Construct`] when `resolution` is not finite and positive,
    /// when any `size` component is not finite and non-negative, or when
    /// `size[i] / resolution` would exceed `i32::MAX` along any axis.
    pub fn new(size: Vector3<f64>, origin: Vector3<f64>, resolution: f64) -> Result<Self> {
        if !(resolution.is_finite() && resolution > 0.0) {
            return Err(Error::construct(format!(
                "resolution must be finite and positive, got {resolution}"
            )));
        }
        let oo_resolution = 1.0 / resolution;
        for (name, value) in [("size.x", size.x), ("size.y", size.y), ("size.z", size.z)] {
            if !(value.is_finite() && value >= 0.0) {
                return Err(Error::construct(format!(
                    "{name} must be finite and non-negative, got {value}"
                )));
            }
            let num_cells = value * oo_resolution;
            if num_cells > f64::from(i32::MAX) {
                return Err(Error::construct(format!(
                    "{name} / resolution must fit in i32, got {name}={value}, \
                     resolution={resolution} ({num_cells} cells)"
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
    ///
    /// # §153.1: a non-finite `loc` returns a deliberately-invalid cell
    ///
    /// PORTING-PLAN.md §172 (float-to-integer narrowing family): the
    /// computed cell coordinate is a `double` expression narrowed into
    /// `i32` by `.floor() as i32`, upstream's own
    /// `static_cast<int>(floor(...))` transcribed exactly. Without a guard,
    /// `loc == f64::INFINITY`/`f64::NEG_INFINITY` are safe *by accident*
    /// (Rust's `as i32` saturates those to `i32::MAX`/`i32::MIN`, and every
    /// caller's `is_cell_valid` already rejects both), but `NaN` is not:
    /// `f64::NAN as i32` is `0` in Rust, a valid cell index along any axis
    /// with at least one cell — a `NaN` world coordinate would silently
    /// resolve to the grid's origin cell and pass `is_cell_valid` instead
    /// of being rejected. C++'s `(int)floor(NaN)` is UB, so there is no
    /// upstream value to match here either. This port rejects the general
    /// case (`!value.is_finite()`, covering `NaN` and both infinities
    /// uniformly, ahead of the `.floor() as i32` that would otherwise treat
    /// them differently) rather than special-casing only `NaN` — one
    /// predicate instead of an infinity-shaped carve-out — and returns
    /// `i32::MIN` for all of them: a sentinel every caller's
    /// `is_cell_valid` already rejects, matching upstream's own "the
    /// returned indices will be computed even if they are invalid"
    /// contract (this still always returns *some* index, just one
    /// guaranteed invalid). This section expires if upstream adds its own
    /// `isfinite` check to `getCellFromLocation`.
    pub fn cell_from_location(&self, dim: Dimension, loc: f64) -> i32 {
        let value = (loc - self.origin_minus[dim as usize]) * self.oo_resolution;
        if !value.is_finite() {
            return i32::MIN;
        }
        value.floor() as i32
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

    /// A bare `.is_err()` does not discriminate here: with the resolution
    /// guard no-opped, `resolution = 0.0` still errors, but via the
    /// unrelated `size / resolution` overflow guard (`1.0 / 0.0 =
    /// f64::INFINITY`, which trips `num_cells > i32::MAX`), so the
    /// `resolution = 0.0` case previously passed for the wrong reason.
    /// `resolution = -0.1` does not share that accident (`1.0 / -0.1 = -10.0`
    /// never overflows), so it alone would have caught the guard's removal.
    /// Both bites confirmed by mutation: no-opping the resolution guard
    /// fails only the `-0.1` assertion under the old `.is_err()` form; the
    /// message check below fails both, since the overflow guard's message
    /// never contains "must be finite and positive".
    #[test]
    fn rejects_non_positive_resolution() {
        let size = Vector3::new(1.0, 1.0, 1.0);
        let zero_err = GridGeometry::new(size, Vector3::zeros(), 0.0).unwrap_err();
        assert!(zero_err.to_string().contains("must be finite and positive"));
        let negative_err = GridGeometry::new(size, Vector3::zeros(), -0.1).unwrap_err();
        assert!(
            negative_err
                .to_string()
                .contains("must be finite and positive")
        );
    }

    /// PORTING-PLAN.md §172 boundary: `size / resolution` at exactly
    /// `i32::MAX` cells must still be accepted -- this is the last value
    /// [`VoxelGrid::new`]'s `as i32` cast represents without saturating, and
    /// upstream's own `int` narrowing is still well-defined here too.
    #[test]
    fn new_accepts_size_over_resolution_at_the_i32_boundary() {
        let size = Vector3::new(f64::from(i32::MAX), 1.0, 1.0);
        assert!(GridGeometry::new(size, Vector3::zeros(), 1.0).is_ok());
    }

    /// The immediately adjacent boundary: one cell past `i32::MAX` must be
    /// rejected. Past this point Rust's `as i32` saturates while C++'s
    /// `(int)(double)` narrowing of the same out-of-range value is UB, so
    /// there is no upstream value this port could match by letting it
    /// through -- see [`GridGeometry::new`]'s own §153.1 doc section.
    ///
    /// ASSERTION-DISCRIMINATION AUDIT (round 2): `single-branch` for this
    /// specific input -- `size.y`/`size.z` are `1.0` at `resolution = 1.0`,
    /// well inside the boundary, and the per-axis loop checks `x` first, so
    /// only `size.x`'s overflow guard can ever fire here; verified by an
    /// isolating mutation that disabled only that guard, which broke this
    /// test alone and left the boundary-accept test above green (see the
    /// sibling y-axis test below for the discrimination gap this same loop
    /// structure caused when the guard was instead reached via a uniform
    /// resolution).
    #[test]
    fn new_rejects_size_over_resolution_one_past_the_i32_boundary() {
        let size = Vector3::new(f64::from(i32::MAX) + 1.0, 1.0, 1.0);
        assert!(GridGeometry::new(size, Vector3::zeros(), 1.0).is_err());
    }

    /// The same boundary reached via `resolution` shrinking rather than
    /// `size` growing, and on a different axis than the two tests above --
    /// pins that the guard applies per-axis, not just to `size.x`.
    ///
    /// A uniform `size = (1.0, 1.0, 1.0)` does not isolate this: `new`'s
    /// per-axis loop shares one `oo_resolution` and checks `x` first, so a
    /// resolution fine enough to overflow `y` also overflows `x`, and a bare
    /// `.is_err()` cannot tell the two apart -- bite-checked by disabling
    /// only the `x`-axis overflow branch, which left the original
    /// `size = (1.0, 1.0, 1.0)` version of this test green, proving it had
    /// been isolating `x`, not `y`, all along. `size.x`/`size.z` are kept
    /// small enough here to stay within the boundary at this resolution so
    /// only `y`'s guard can fire, and the message is matched by name since
    /// `Error::construct` carries no structured field to match on instead.
    #[test]
    fn new_rejects_a_pathologically_fine_resolution_on_the_y_axis() {
        let size = Vector3::new(1e-10, 1.0, 1e-10);
        let err = GridGeometry::new(size, Vector3::zeros(), 1e-10).unwrap_err();
        assert!(
            err.to_string().contains("size.y"),
            "must name size.y specifically, not size.x or size.z; got {err}"
        );
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

    /// PORTING-PLAN.md §172 boundary: every non-finite `loc` -- both
    /// infinities included, not just `NaN` -- returns the same `i32::MIN`
    /// sentinel via the `!value.is_finite()` guard, ahead of the
    /// `.floor() as i32` that would otherwise treat them differently
    /// (`as i32` would saturate the infinities to `i32::MAX`/`i32::MIN` but
    /// collapse `NaN` to `0`, a valid cell index -- see
    /// [`VoxelGrid::cell_from_location`]'s own doc comment). Pinned as one
    /// uniform boundary rather than three separate cases so a future
    /// refactor can't accidentally special-case one non-finite value
    /// differently from the others.
    #[test]
    fn cell_from_location_of_any_non_finite_value_is_the_invalid_sentinel() {
        let vg = VoxelGrid::new(cube_geometry(10.0, 1.0), 0);
        for loc in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert_eq!(vg.cell_from_location(Dimension::X, loc), i32::MIN);
        }
    }

    /// The end-to-end consequence of the sentinel above: a `NaN` component
    /// anywhere in a [`VoxelGrid::world_to_grid`] query must report
    /// `valid == false`, not silently resolve to a real cell.
    #[test]
    fn world_to_grid_of_a_nan_component_is_invalid() {
        let vg = VoxelGrid::new(cube_geometry(10.0, 1.0), 0);
        let (valid, ..) = vg.world_to_grid(&Vector3::new(f64::NAN, 1.0, 1.0));
        assert!(!valid);
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
