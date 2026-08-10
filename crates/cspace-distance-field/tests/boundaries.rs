// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! One case per invariant boundary of [`PropagationDistanceField`], beyond
//! the ported upstream cases in `tests/upstream_parity.rs`.
//!
//! The boundaries covered, and why each is a boundary rather than a scenario:
//!
//! - a point exactly on a cell boundary — the pre-shifted-origin rounding in
//!   [`VoxelGrid::cell_from_location`] is unit-tested directly in
//!   `voxel_grid.rs`; this file checks the same rounding is what
//!   `add_points_to_field` actually uses to decide which cell becomes the
//!   obstacle.
//! - a point outside the grid fed to `add_points_to_field` /
//!   `remove_points_from_field` — both silently drop it (no panic, no
//!   effect), matching upstream's "returned indices computed even when
//!   invalid" contract.
//! - add-then-remove of the same point set returns to the freshly-reset
//!   state, for both the unsigned and the negative-field-enabled cases.
//! - an incremental `update_points_in_field` with partial overlap between
//!   the old and new point sets agrees with a full rebuild from the new set
//!   alone.
//! - the negative field is tracked only when `propagate_negative_distances`
//!   is enabled at construction; positive distances are identical either
//!   way.

use cspace_distance_field::{DistanceField, GridGeometry, PropagationDistanceField};
use nalgebra::Vector3;

const WIDTH: f64 = 1.0;
const HEIGHT: f64 = 1.0;
const DEPTH: f64 = 1.0;
const RESOLUTION: f64 = 0.1;
const ORIGIN: f64 = 0.0;
const MAX_DIST: f64 = 0.3;

fn new_field(propagate_negative: bool) -> PropagationDistanceField {
    let geometry = GridGeometry::new(
        Vector3::new(WIDTH, HEIGHT, DEPTH),
        Vector3::new(ORIGIN, ORIGIN, ORIGIN),
        RESOLUTION,
    )
    .unwrap();
    PropagationDistanceField::new(geometry, MAX_DIST, propagate_negative).unwrap()
}

fn fields_equal(a: &PropagationDistanceField, b: &PropagationDistanceField) -> bool {
    if a.num_cells_x() != b.num_cells_x()
        || a.num_cells_y() != b.num_cells_y()
        || a.num_cells_z() != b.num_cells_z()
    {
        return false;
    }
    for x in 0..a.num_cells_x() {
        for y in 0..a.num_cells_y() {
            for z in 0..a.num_cells_z() {
                let va = a.cell(x, y, z);
                let vb = b.cell(x, y, z);
                if va.distance_square != vb.distance_square
                    || va.negative_distance_square != vb.negative_distance_square
                {
                    return false;
                }
            }
        }
    }
    true
}

// ---------------------------------------------------- cell boundary ----

/// Resolution 0.1, origin 0.0: cell 0 covers world `[-0.05, 0.05)`, cell 1
/// covers `[0.05, 0.15)`. A point exactly at `0.05` is on the boundary and,
/// per `cell_from_location`'s `floor` convention, rounds up into cell 1 —
/// never cell 0.
#[test]
fn a_point_exactly_on_a_cell_boundary_becomes_an_obstacle_in_the_upper_cell() {
    let mut df = new_field(false);
    df.add_points_to_field(&[Vector3::new(0.05, 0.05, 0.05)]);

    assert_eq!(
        df.cell(1, 1, 1).distance_square,
        0,
        "the upper cell is the obstacle"
    );
    assert_ne!(
        df.cell(0, 0, 0).distance_square,
        0,
        "the lower cell must not be the obstacle"
    );
}

// ------------------------------------------------ outside the grid ----

#[test]
fn a_point_outside_the_grid_is_dropped_by_add_and_remove_without_panicking() {
    let mut df = new_field(true);
    let baseline = new_field(true);
    let outside = Vector3::new(1000.0, 1000.0, 1000.0);

    df.add_points_to_field(&[outside]);
    assert!(
        fields_equal(&df, &baseline),
        "an out-of-grid point must not become an obstacle"
    );

    df.remove_points_from_field(&[outside]);
    assert!(fields_equal(&df, &baseline));
}

// -------------------------------------------- add-then-remove == reset ----

#[test]
fn add_then_remove_the_same_points_returns_to_the_reset_state() {
    let mut df = new_field(false);
    let fresh = new_field(false);

    let points: Vec<Vector3<f64>> = [(1, 1, 1), (5, 5, 5), (8, 2, 6)]
        .into_iter()
        .map(|(x, y, z)| df.grid_to_world(x, y, z))
        .collect();

    df.add_points_to_field(&points);
    df.remove_points_from_field(&points);

    assert!(fields_equal(&df, &fresh));
}

#[test]
fn add_then_remove_the_same_points_returns_to_the_reset_state_with_negative_field_enabled() {
    let mut df = new_field(true);
    let fresh = new_field(true);

    let points: Vec<Vector3<f64>> = [(3, 3, 3), (4, 3, 3), (3, 4, 3), (3, 3, 4)]
        .into_iter()
        .map(|(x, y, z)| df.grid_to_world(x, y, z))
        .collect();

    df.add_points_to_field(&points);
    df.remove_points_from_field(&points);

    assert!(fields_equal(&df, &fresh));
}

// -------------------------------------- incremental update == rebuild ----

/// `old = {A, B, C}`, `new = {B, C, D}` — a partial-overlap update (one point
/// dropped, one added, two retained), not the all-different or
/// all-in-common cases the ported upstream tests already cover.
#[test]
fn incremental_update_with_partial_overlap_agrees_with_a_full_rebuild() {
    let mut df = new_field(false);

    let cell_points =
        |df: &PropagationDistanceField, cells: &[(i32, i32, i32)]| -> Vec<Vector3<f64>> {
            cells
                .iter()
                .map(|&(x, y, z)| df.grid_to_world(x, y, z))
                .collect()
        };

    let old_points = cell_points(&df, &[(1, 1, 1), (2, 2, 2), (3, 3, 3)]);
    df.add_points_to_field(&old_points);

    let new_points = cell_points(&df, &[(2, 2, 2), (3, 3, 3), (7, 7, 7)]);
    df.update_points_in_field(&old_points, &new_points);

    let mut rebuilt = new_field(false);
    rebuilt.add_points_to_field(&new_points);

    assert!(fields_equal(&df, &rebuilt));
}

// ---------------------------------- negative field enabled vs disabled ----

/// Positive distances must be identical whether or not the negative field is
/// tracked; only the negative field itself differs.
#[test]
fn negative_field_is_tracked_only_when_enabled_at_construction() {
    let mut unsigned = new_field(false);
    let mut signed = new_field(true);

    // A solid 3x3x3 block of obstacle cells so its center has no free
    // face-neighbor and a meaningful (non-1-cell) negative distance can
    // propagate to it.
    let mut points = Vec::new();
    for x in 3..=5 {
        for y in 3..=5 {
            for z in 3..=5 {
                points.push(unsigned.grid_to_world(x, y, z));
            }
        }
    }

    unsigned.add_points_to_field(&points);
    signed.add_points_to_field(&points);

    for x in 0..unsigned.num_cells_x() {
        for y in 0..unsigned.num_cells_y() {
            for z in 0..unsigned.num_cells_z() {
                assert_eq!(
                    unsigned.cell(x, y, z).distance_square,
                    signed.cell(x, y, z).distance_square,
                    "positive distance must not depend on negative-field tracking"
                );
                assert_eq!(
                    unsigned.cell(x, y, z).negative_distance_square,
                    0,
                    "unsigned field must never carry a negative distance"
                );
            }
        }
    }

    // (4, 4, 4) is the fully-enclosed center of the block.
    assert!(signed.cell(4, 4, 4).negative_distance_square > 0);
    assert!(signed.distance_cell(4, 4, 4) < 0.0);
}
