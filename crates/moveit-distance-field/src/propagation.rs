// Copyright (c) 2009, Willow Garage, Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/distance_field/include/moveit/distance_field/propagation_distance_field.hpp
//   moveit_core/distance_field/src/propagation_distance_field.cpp

use std::collections::BTreeSet;

use moveit_error::{Error, Result};
use nalgebra::Vector3;

use crate::distance_field::DistanceField;
use crate::voxel_grid::{Dimension, VoxelGrid};

/// Per-voxel bookkeeping for [`PropagationDistanceField`]: distances (stored
/// squared, in cells — see the field docs) plus enough state to resume
/// propagation incrementally.
///
/// Upstream `distance_field::PropDistanceFieldVoxel`.
///
/// # Deviation from upstream
///
/// Upstream's default constructor leaves every field genuinely
/// uninitialized ("All fields left uninitialized", by design — voxels are
/// always overwritten before being read). Safe Rust has no such escape
/// hatch, so [`PropDistanceFieldVoxel::new`] (the only constructor this port
/// carries — the field-uninitialized default has no caller) initializes
/// `update_direction`/`negative_update_direction` to
/// [`PropDistanceFieldVoxel::UNINITIALIZED`] rather than leaving them
/// genuinely undefined. No algorithm path reads either field before writing
/// it, so this is unobservable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PropDistanceFieldVoxel {
    /// `distance_square_`: squared distance in cells to the closest obstacle.
    /// Squared and in cells, not meters — see [`crate::PropagationDistanceField`]'s
    /// module doc for why this is never silently normalised to a `f64`
    /// meters value except at read time ([`DistanceField::distance`] /
    /// [`DistanceField::distance_cell`]).
    pub distance_square: i32,
    /// `negative_distance_square_`: squared distance in cells to the
    /// nearest unoccupied cell. Only meaningful when the field propagates
    /// negative distances.
    pub negative_distance_square: i32,
    /// `closest_point_`.
    pub closest_point: Vector3<i32>,
    /// `closest_negative_point_`.
    pub closest_negative_point: Vector3<i32>,
    update_direction: i32,
    negative_update_direction: i32,
}

impl PropDistanceFieldVoxel {
    /// `PropDistanceFieldVoxel::UNINITIALIZED`.
    pub const UNINITIALIZED: i32 = -1;

    /// Upstream `PropDistanceFieldVoxel(int, int)`.
    pub fn new(distance_square: i32, negative_distance_square: i32) -> Self {
        let uninitialized = Vector3::new(
            Self::UNINITIALIZED,
            Self::UNINITIALIZED,
            Self::UNINITIALIZED,
        );
        Self {
            distance_square,
            negative_distance_square,
            closest_point: uninitialized,
            closest_negative_point: uninitialized,
            update_direction: Self::UNINITIALIZED,
            negative_update_direction: Self::UNINITIALIZED,
        }
    }
}

fn direction_number(dx: i32, dy: i32, dz: i32) -> i32 {
    (dx + 1) * 9 + (dy + 1) * 3 + dz + 1
}

/// `neighborhoods[n][direction_number]` is the list of `(dx, dy, dz)`
/// offsets to propagate to. See [`build_neighborhoods`].
type Neighborhoods = [Vec<Vec<Vector3<i32>>>; 2];

/// Builds the `neighborhoods_` table and the direction-number lookup.
/// Upstream `PropagationDistanceField::initNeighborhoods`.
///
/// `neighborhoods[0]` is the unrestricted 26-connected neighborhood, used
/// for the initial (`d = 0`) expansion of a bucket. `neighborhoods[1]` is
/// restricted to the 6 face directions that do not oppose the direction a
/// voxel was itself last updated from — the optimization that keeps
/// propagation from revisiting cells behind the wavefront. Both are indexed
/// by `direction_number(dx, dy, dz)` of the *source* direction.
fn build_neighborhoods() -> (Neighborhoods, [Vector3<i32>; 27]) {
    let mut direction_number_to_direction = [Vector3::new(0, 0, 0); 27];
    for dx in -1..=1 {
        for dy in -1..=1 {
            for dz in -1..=1 {
                let dn = direction_number(dx, dy, dz) as usize;
                direction_number_to_direction[dn] = Vector3::new(dx, dy, dz);
            }
        }
    }

    let mut neighborhoods: Neighborhoods = [vec![Vec::new(); 27], vec![Vec::new(); 27]];
    for (n, neighborhood) in neighborhoods.iter_mut().enumerate() {
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let dn = direction_number(dx, dy, dz) as usize;
                    for tdx in -1_i32..=1 {
                        for tdy in -1_i32..=1 {
                            for tdz in -1_i32..=1 {
                                if tdx == 0 && tdy == 0 && tdz == 0 {
                                    continue;
                                }
                                if n >= 1 {
                                    if tdx.abs() + tdy.abs() + tdz.abs() != 1 {
                                        continue;
                                    }
                                    if dx * tdx < 0 || dy * tdy < 0 || dz * tdz < 0 {
                                        continue;
                                    }
                                }
                                neighborhood[dn].push(Vector3::new(tdx, tdy, tdz));
                            }
                        }
                    }
                }
            }
        }
    }
    (neighborhoods, direction_number_to_direction)
}

/// Squared Euclidean distance between two cell coordinates, in cells.
/// Upstream `(a - b).squaredNorm()` on `Eigen::Vector3i`. Computed in `i64`
/// so the subtraction and the sum of squares cannot overflow `i32` for grids
/// upstream's own `int` arithmetic would already have been unsafe on.
fn squared_distance(a: Vector3<i32>, b: Vector3<i32>) -> i64 {
    let dx = i64::from(a.x) - i64::from(b.x);
    let dy = i64::from(a.y) - i64::from(b.y);
    let dz = i64::from(a.z) - i64::from(b.z);
    dx * dx + dy * dy + dz * dz
}

/// The nearest occupied (or, if the queried cell is itself occupied, nearest
/// free) cell to a queried cell. Return value of
/// [`PropagationDistanceField::nearest_cell`], upstream
/// `PropagationDistanceField::getNearestCell`'s out-parameters plus return
/// value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NearestCell<'a> {
    /// The neighboring voxel at [`NearestCell::position`], or `None` if the
    /// queried cell has no distance information yet (matching upstream's
    /// "if nearest cell is unknown, return nullptr") or if the nearest cell
    /// *is* the queried cell itself.
    pub voxel: Option<&'a PropDistanceFieldVoxel>,
    /// Negative if the queried cell is inside an obstacle (distance to the
    /// nearest free cell), positive if outside (distance to the nearest
    /// obstacle), zero if unknown.
    pub distance: f64,
    /// The position of the nearest cell.
    pub position: Vector3<i32>,
}

/// A [`DistanceField`] that propagates distances outward from occupied
/// cells (and, optionally, inward from unoccupied cells) via bucketed-queue
/// wavefront expansion, to a configurable maximum distance.
///
/// Upstream `distance_field::PropagationDistanceField`.
///
/// # Deviations from upstream
///
/// - The octree-bounding-box and istream constructors, and
///   `writeToStream`/`readFromStream`, are not ported — see
///   [`DistanceField`]'s module doc for why.
/// - Upstream's `initialize()` computes `max_distance_sq_` and divides by
///   `resolution_` without checking either is finite or positive, which for
///   `resolution <= 0` produces `inf`/`NaN` cell counts that are then
///   silently truncated into the `int` fields. [`PropagationDistanceField::new`]
///   returns [`moveit_error::Error::Construct`] instead.
pub struct PropagationDistanceField {
    propagate_negative: bool,
    voxel_grid: VoxelGrid<PropDistanceFieldVoxel>,
    bucket_queue: Vec<Vec<Vector3<i32>>>,
    negative_bucket_queue: Vec<Vec<Vector3<i32>>>,
    max_distance: f64,
    max_distance_sq: i32,
    sqrt_table: Vec<f64>,
    neighborhoods: Neighborhoods,
    direction_number_to_direction: [Vector3<i32>; 27],
}

impl PropagationDistanceField {
    /// Upstream `PropagationDistanceField(size_x, size_y, size_z,
    /// resolution, origin_x, origin_y, origin_z, max_distance,
    /// propagate_negative_distances = false)`.
    ///
    /// # Errors
    ///
    /// See this type's "Deviations from upstream" doc section.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        size_x: f64,
        size_y: f64,
        size_z: f64,
        resolution: f64,
        origin_x: f64,
        origin_y: f64,
        origin_z: f64,
        max_distance: f64,
        propagate_negative_distances: bool,
    ) -> Result<Self> {
        if !(resolution.is_finite() && resolution > 0.0) {
            return Err(Error::construct(format!(
                "resolution must be finite and positive, got {resolution}"
            )));
        }
        // Upstream: `max_distance_sq_ = ceil(max_distance_ / resolution_) *
        // ceil(max_distance_ / resolution_)`, assigned into an `int` field.
        let n = (max_distance / resolution).ceil();
        let max_distance_sq_f = n * n;
        if !(max_distance_sq_f.is_finite() && max_distance_sq_f >= 0.0) {
            return Err(Error::construct(format!(
                "max_distance_sq computed as {max_distance_sq_f} from max_distance={max_distance}, \
                 resolution={resolution}; must be finite and non-negative"
            )));
        }
        let max_distance_sq = max_distance_sq_f as i32;

        let voxel_grid = VoxelGrid::new(
            size_x,
            size_y,
            size_z,
            resolution,
            origin_x,
            origin_y,
            origin_z,
            PropDistanceFieldVoxel::new(max_distance_sq, 0),
        )?;

        let (neighborhoods, direction_number_to_direction) = build_neighborhoods();
        let bucket_len = (max_distance_sq + 1) as usize;

        let mut field = Self {
            propagate_negative: propagate_negative_distances,
            voxel_grid,
            bucket_queue: vec![Vec::new(); bucket_len],
            negative_bucket_queue: vec![Vec::new(); bucket_len],
            max_distance,
            max_distance_sq,
            sqrt_table: (0..bucket_len)
                .map(|i| (i as f64).sqrt() * resolution)
                .collect(),
            neighborhoods,
            direction_number_to_direction,
        };
        field.reset();
        Ok(field)
    }

    /// Upstream `PropagationDistanceField::getCell`. Panics if the cell is
    /// invalid, matching upstream's documented contract (see
    /// [`crate::VoxelGrid`]'s doc comment for the panic-instead-of-UB
    /// rationale).
    pub fn cell(&self, x: i32, y: i32, z: i32) -> &PropDistanceFieldVoxel {
        self.voxel_grid.get_cell(x, y, z)
    }

    /// Upstream `PropagationDistanceField::getMaximumDistanceSquared`.
    pub fn max_distance_squared(&self) -> i32 {
        self.max_distance_sq
    }

    /// Upstream `PropagationDistanceField::getNearestCell`.
    pub fn nearest_cell(&self, x: i32, y: i32, z: i32) -> NearestCell<'_> {
        let queried = Vector3::new(x, y, z);
        let cell = self.voxel_grid.get_cell(x, y, z);
        if cell.distance_square > 0 {
            let pos = cell.closest_point;
            let neighbor = self.voxel_grid.get_cell(pos.x, pos.y, pos.z);
            return NearestCell {
                voxel: (pos != queried).then_some(neighbor),
                distance: self.sqrt_table[cell.distance_square as usize],
                position: pos,
            };
        }
        if cell.negative_distance_square > 0 {
            let pos = cell.closest_negative_point;
            let neighbor = self.voxel_grid.get_cell(pos.x, pos.y, pos.z);
            return NearestCell {
                voxel: (pos != queried).then_some(neighbor),
                distance: -self.sqrt_table[cell.negative_distance_square as usize],
                position: pos,
            };
        }
        NearestCell {
            voxel: None,
            distance: 0.0,
            position: queried,
        }
    }

    fn distance_of(&self, voxel: &PropDistanceFieldVoxel) -> f64 {
        self.sqrt_table[voxel.distance_square as usize]
            - self.sqrt_table[voxel.negative_distance_square as usize]
    }

    /// Upstream `PropagationDistanceField::addNewObstacleVoxels`.
    fn add_new_obstacle_voxels(&mut self, voxel_points: &[Vector3<i32>]) {
        let initial_update_direction = direction_number(0, 0, 0);
        self.bucket_queue[0].reserve(voxel_points.len());
        let mut negative_stack: Vec<Vector3<i32>> = Vec::new();
        if self.propagate_negative {
            negative_stack.reserve(self.grid_cell_count());
            self.negative_bucket_queue[0].reserve(voxel_points.len());
        }

        for &loc in voxel_points {
            let mut voxel = *self.voxel_grid.get_cell(loc.x, loc.y, loc.z);
            voxel.distance_square = 0;
            voxel.closest_point = loc;
            voxel.update_direction = initial_update_direction;
            if self.propagate_negative {
                voxel.negative_distance_square = self.max_distance_sq;
                voxel.closest_negative_point = Vector3::new(
                    PropDistanceFieldVoxel::UNINITIALIZED,
                    PropDistanceFieldVoxel::UNINITIALIZED,
                    PropDistanceFieldVoxel::UNINITIALIZED,
                );
                negative_stack.push(loc);
            }
            self.voxel_grid.set_cell(loc.x, loc.y, loc.z, voxel);
            self.bucket_queue[0].push(loc);
        }
        self.propagate_positive();

        if self.propagate_negative {
            // Find every neighbor whose closest non-obstacle cell just
            // became an obstacle, and re-seed those for negative
            // propagation. See this function's `nvoxel` aliasing note below.
            while let Some(loc) = negative_stack.pop() {
                for neighbor_idx in 0..27 {
                    let diff = self.direction_number_to_direction[neighbor_idx];
                    let nloc = loc + diff;
                    if !self.voxel_grid.is_cell_valid(nloc.x, nloc.y, nloc.z) {
                        continue;
                    }
                    // Upstream mutates `nvoxel` (a live reference into the
                    // grid) directly, including through a `close_point`
                    // reference aliased into `nvoxel.closest_negative_point_`.
                    // This port instead copies the cell, mutates the copy,
                    // and writes it back once at the end; `same_cell` below
                    // replicates the one place the aliasing is observable —
                    // reading `closest_point_voxel` when `close_point` has
                    // just been reset to `nloc` itself must see this
                    // iteration's own (not-yet-written-back) mutation.
                    let mut nvoxel = *self.voxel_grid.get_cell(nloc.x, nloc.y, nloc.z);
                    let mut close_point = nvoxel.closest_negative_point;
                    if !self
                        .voxel_grid
                        .is_cell_valid(close_point.x, close_point.y, close_point.z)
                    {
                        close_point = nloc;
                        nvoxel.closest_negative_point = nloc;
                    }
                    let same_cell = close_point == nloc;
                    let closest_point_voxel = if same_cell {
                        nvoxel
                    } else {
                        *self
                            .voxel_grid
                            .get_cell(close_point.x, close_point.y, close_point.z)
                    };

                    if closest_point_voxel.negative_distance_square != 0 {
                        // Our closest non-obstacle cell has itself become an
                        // obstacle; force re-propagation with a new one.
                        if nvoxel.negative_distance_square != self.max_distance_sq {
                            nvoxel.negative_distance_square = self.max_distance_sq;
                            nvoxel.closest_negative_point = Vector3::new(
                                PropDistanceFieldVoxel::UNINITIALIZED,
                                PropDistanceFieldVoxel::UNINITIALIZED,
                                PropDistanceFieldVoxel::UNINITIALIZED,
                            );
                            self.voxel_grid.set_cell(nloc.x, nloc.y, nloc.z, nvoxel);
                            negative_stack.push(nloc);
                        }
                    } else {
                        nvoxel.negative_update_direction = initial_update_direction;
                        self.voxel_grid.set_cell(nloc.x, nloc.y, nloc.z, nvoxel);
                        self.negative_bucket_queue[0].push(nloc);
                    }
                }
            }
            self.propagate_negative();
        }
    }

    /// Upstream `PropagationDistanceField::removeObstacleVoxels`.
    ///
    /// # Deviation from upstream
    ///
    /// Upstream declares and `reserve()`s a `negative_stack` vector here
    /// that is never read or written anywhere else in the function — dead
    /// code in the C++ source (`propagation_distance_field.cpp`,
    /// `removeObstacleVoxels`). This port omits it entirely rather than
    /// port an unused variable.
    fn remove_obstacle_voxels(&mut self, voxel_points: &[Vector3<i32>]) {
        let initial_update_direction = direction_number(0, 0, 0);
        let mut stack: Vec<Vector3<i32>> = Vec::with_capacity(self.grid_cell_count());
        self.bucket_queue[0].reserve(voxel_points.len());
        if self.propagate_negative {
            self.negative_bucket_queue[0].reserve(voxel_points.len());
        }

        for &loc in voxel_points {
            let mut voxel = *self.voxel_grid.get_cell(loc.x, loc.y, loc.z);
            voxel.distance_square = self.max_distance_sq;
            voxel.closest_point = loc;
            voxel.update_direction = initial_update_direction;
            if self.propagate_negative {
                voxel.negative_distance_square = 0;
                voxel.closest_negative_point = loc;
                voxel.negative_update_direction = initial_update_direction;
                self.negative_bucket_queue[0].push(loc);
            }
            self.voxel_grid.set_cell(loc.x, loc.y, loc.z, voxel);
            stack.push(loc);
        }

        // Reset every neighbor whose closest occupied cell is now gone. See
        // `add_new_obstacle_voxels`'s doc comment for the aliasing note this
        // mirrors.
        while let Some(loc) = stack.pop() {
            for neighbor_idx in 0..27 {
                let diff = self.direction_number_to_direction[neighbor_idx];
                let nloc = loc + diff;
                if !self.voxel_grid.is_cell_valid(nloc.x, nloc.y, nloc.z) {
                    continue;
                }
                let mut nvoxel = *self.voxel_grid.get_cell(nloc.x, nloc.y, nloc.z);
                let mut close_point = nvoxel.closest_point;
                if !self
                    .voxel_grid
                    .is_cell_valid(close_point.x, close_point.y, close_point.z)
                {
                    close_point = nloc;
                    nvoxel.closest_point = nloc;
                }
                let same_cell = close_point == nloc;
                let closest_point_voxel = if same_cell {
                    nvoxel
                } else {
                    *self
                        .voxel_grid
                        .get_cell(close_point.x, close_point.y, close_point.z)
                };

                if closest_point_voxel.distance_square != 0 {
                    if nvoxel.distance_square != self.max_distance_sq {
                        nvoxel.distance_square = self.max_distance_sq;
                        nvoxel.closest_point = nloc;
                        nvoxel.update_direction = initial_update_direction;
                        self.voxel_grid.set_cell(nloc.x, nloc.y, nloc.z, nvoxel);
                        stack.push(nloc);
                    }
                } else {
                    nvoxel.update_direction = initial_update_direction;
                    self.voxel_grid.set_cell(nloc.x, nloc.y, nloc.z, nvoxel);
                    self.bucket_queue[0].push(nloc);
                }
            }
        }
        self.propagate_positive();

        if self.propagate_negative {
            self.propagate_negative();
        }
    }

    /// Upstream `PropagationDistanceField::propagatePositive`.
    fn propagate_positive(&mut self) {
        for i in 0..self.bucket_queue.len() {
            let list = std::mem::take(&mut self.bucket_queue[i]);
            let d = i.min(1);
            for loc in list {
                let voxel = *self.voxel_grid.get_cell(loc.x, loc.y, loc.z);
                let dir = voxel.update_direction as usize;
                let count = self.neighborhoods[d][dir].len();
                for k in 0..count {
                    let diff = self.neighborhoods[d][dir][k];
                    let nloc = loc + diff;
                    if !self.voxel_grid.is_cell_valid(nloc.x, nloc.y, nloc.z) {
                        continue;
                    }
                    let dist = squared_distance(voxel.closest_point, nloc);
                    if dist > i64::from(self.max_distance_sq) {
                        continue;
                    }
                    let dist = dist as i32;
                    let mut neighbor = *self.voxel_grid.get_cell(nloc.x, nloc.y, nloc.z);
                    if dist < neighbor.distance_square {
                        neighbor.distance_square = dist;
                        neighbor.closest_point = voxel.closest_point;
                        neighbor.update_direction = direction_number(diff.x, diff.y, diff.z);
                        self.voxel_grid.set_cell(nloc.x, nloc.y, nloc.z, neighbor);
                        self.bucket_queue[dist as usize].push(nloc);
                    }
                }
            }
        }
    }

    /// Upstream `PropagationDistanceField::propagateNegative`.
    fn propagate_negative(&mut self) {
        for i in 0..self.negative_bucket_queue.len() {
            let list = std::mem::take(&mut self.negative_bucket_queue[i]);
            let d = i.min(1);
            for loc in list {
                let voxel = *self.voxel_grid.get_cell(loc.x, loc.y, loc.z);
                let dir = voxel.negative_update_direction as usize;
                let count = self.neighborhoods[d][dir].len();
                for k in 0..count {
                    let diff = self.neighborhoods[d][dir][k];
                    let nloc = loc + diff;
                    if !self.voxel_grid.is_cell_valid(nloc.x, nloc.y, nloc.z) {
                        continue;
                    }
                    let dist = squared_distance(voxel.closest_negative_point, nloc);
                    if dist > i64::from(self.max_distance_sq) {
                        continue;
                    }
                    let dist = dist as i32;
                    let mut neighbor = *self.voxel_grid.get_cell(nloc.x, nloc.y, nloc.z);
                    if dist < neighbor.negative_distance_square {
                        neighbor.negative_distance_square = dist;
                        neighbor.closest_negative_point = voxel.closest_negative_point;
                        neighbor.negative_update_direction =
                            direction_number(diff.x, diff.y, diff.z);
                        self.voxel_grid.set_cell(nloc.x, nloc.y, nloc.z, neighbor);
                        self.negative_bucket_queue[dist as usize].push(nloc);
                    }
                }
            }
        }
    }

    fn grid_cell_count(&self) -> usize {
        self.num_cells_x() as usize * self.num_cells_y() as usize * self.num_cells_z() as usize
    }
}

impl DistanceField for PropagationDistanceField {
    fn size_x(&self) -> f64 {
        self.voxel_grid.size(Dimension::X)
    }

    fn size_y(&self) -> f64 {
        self.voxel_grid.size(Dimension::Y)
    }

    fn size_z(&self) -> f64 {
        self.voxel_grid.size(Dimension::Z)
    }

    fn origin_x(&self) -> f64 {
        self.voxel_grid.origin(Dimension::X)
    }

    fn origin_y(&self) -> f64 {
        self.voxel_grid.origin(Dimension::Y)
    }

    fn origin_z(&self) -> f64 {
        self.voxel_grid.origin(Dimension::Z)
    }

    fn resolution(&self) -> f64 {
        self.voxel_grid.resolution()
    }

    fn uninitialized_distance(&self) -> f64 {
        self.max_distance
    }

    /// Upstream `PropagationDistanceField::addPointsToField`.
    fn add_points_to_field(&mut self, points: &[Vector3<f64>]) {
        let mut voxel_points = Vec::new();
        for point in points {
            let (valid, x, y, z) = self.voxel_grid.world_to_grid(point);
            if valid && self.voxel_grid.get_cell(x, y, z).distance_square > 0 {
                voxel_points.push(Vector3::new(x, y, z));
            }
        }
        self.add_new_obstacle_voxels(&voxel_points);
    }

    /// Upstream `PropagationDistanceField::removePointsFromField`.
    fn remove_points_from_field(&mut self, points: &[Vector3<f64>]) {
        let mut voxel_points = Vec::new();
        for point in points {
            let (valid, x, y, z) = self.voxel_grid.world_to_grid(point);
            if valid {
                voxel_points.push(Vector3::new(x, y, z));
            }
        }
        self.remove_obstacle_voxels(&voxel_points);
    }

    /// Upstream `PropagationDistanceField::updatePointsInField`.
    ///
    /// # Deviation from upstream
    ///
    /// Upstream sorts the old/new point sets with a comparator ordering by
    /// `(z, y, x)` (`CompareEigenVector3i`), for cache locality of the
    /// `std::set` it builds — not for correctness: the resulting
    /// `old_not_new`/`new_not_old` sets feed `addNewObstacleVoxels`, whose
    /// output is a fixed point over the whole batch regardless of the order
    /// voxels are seeded in (ties in propagation distance only affect which
    /// of several equidistant obstacle points is recorded as
    /// `closest_point_`, never the recorded *distance* — exactly the
    /// invariant upstream's own `areDistanceFieldsDistancesEqual` test
    /// helper checks, by comparing `distance_square_` and
    /// `negative_distance_square_` only). This port uses a plain
    /// `(x, y, z)`-ordered `BTreeSet` instead.
    fn update_points_in_field(&mut self, old_points: &[Vector3<f64>], new_points: &[Vector3<f64>]) {
        let mut old_set = BTreeSet::new();
        for point in old_points {
            let (valid, x, y, z) = self.voxel_grid.world_to_grid(point);
            if valid {
                old_set.insert((x, y, z));
            }
        }
        let mut new_set = BTreeSet::new();
        for point in new_points {
            let (valid, x, y, z) = self.voxel_grid.world_to_grid(point);
            if valid {
                new_set.insert((x, y, z));
            }
        }

        let old_not_new: Vec<Vector3<i32>> = old_set
            .difference(&new_set)
            .map(|&(x, y, z)| Vector3::new(x, y, z))
            .collect();
        let new_not_in_current: Vec<Vector3<i32>> = new_set
            .difference(&old_set)
            .filter(|&&(x, y, z)| self.voxel_grid.get_cell(x, y, z).distance_square != 0)
            .map(|&(x, y, z)| Vector3::new(x, y, z))
            .collect();

        self.remove_obstacle_voxels(&old_not_new);
        self.add_new_obstacle_voxels(&new_not_in_current);
    }

    /// Upstream `PropagationDistanceField::reset`.
    fn reset(&mut self) {
        let filler = PropDistanceFieldVoxel::new(self.max_distance_sq, 0);
        self.voxel_grid.reset(filler);
        for x in 0..self.num_cells_x() {
            for y in 0..self.num_cells_y() {
                for z in 0..self.num_cells_z() {
                    let mut voxel = *self.voxel_grid.get_cell(x, y, z);
                    voxel.closest_negative_point = Vector3::new(x, y, z);
                    voxel.negative_distance_square = 0;
                    self.voxel_grid.set_cell(x, y, z, voxel);
                }
            }
        }
    }

    fn distance(&self, x: f64, y: f64, z: f64) -> f64 {
        self.distance_of(self.voxel_grid.get(x, y, z))
    }

    fn distance_cell(&self, x: i32, y: i32, z: i32) -> f64 {
        self.distance_of(self.voxel_grid.get_cell(x, y, z))
    }

    fn is_cell_valid(&self, x: i32, y: i32, z: i32) -> bool {
        self.voxel_grid.is_cell_valid(x, y, z)
    }

    fn num_cells_x(&self) -> i32 {
        self.voxel_grid.num_cells(Dimension::X)
    }

    fn num_cells_y(&self) -> i32 {
        self.voxel_grid.num_cells(Dimension::Y)
    }

    fn num_cells_z(&self) -> i32 {
        self.voxel_grid.num_cells(Dimension::Z)
    }

    fn grid_to_world(&self, x: i32, y: i32, z: i32) -> Vector3<f64> {
        self.voxel_grid.grid_to_world(x, y, z)
    }

    fn world_to_grid(&self, world: &Vector3<f64>) -> (bool, i32, i32, i32) {
        self.voxel_grid.world_to_grid(world)
    }
}
