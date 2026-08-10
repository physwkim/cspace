// Copyright (c) 2023, PickNik Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/stomp/include/stomp_moveit/filter_functions.hpp

//! Trajectory-update filters STOMP applies to its waypoint matrix each
//! iteration.
//!
//! # `simpleSmoothingMatrix`, round 22
//!
//! Round 21's note here said `ros-industrial/stomp` (the separate upstream
//! `stomp::generateSmoothingMatrix` lives in) was absent from this machine.
//! It is now vendored at `/home/stevek/work/stomp` and ported as
//! [`cspace_stomp_core::generate_smoothing_matrix`]; [`simple_smoothing_matrix`]
//! below calls it, closing this gap.
//!
//! # `FilterFn`'s home
//!
//! Upstream declares `FilterFn` in `stomp_moveit_task.hpp` (not ported --
//! ROS/task-engine layer, see `lib.rs`), alongside `NoiseGeneratorFn` and
//! `CostFn`. This module carries the one piece of that header every filter
//! function here actually needs: the `FilterFn` signature itself.

use cspace_core::model::{JointModelGroup, RobotModel};
use nalgebra::DMatrix;

use cspace_core::error::{Error, Result};

use crate::stomp::require_single_variable;

/// `using FilterFn = std::function<bool(const Eigen::MatrixXd&,
/// Eigen::MatrixXd&)>`. Parametrized over a lifetime because, unlike
/// upstream's raw-pointer `[=]` captures, a filter built from borrowed data
/// (a [`JointModelGroup`], a [`RobotModel`]) must not outlive what it
/// borrows -- the borrow checker's translation of "the caller must keep the
/// captured pointer's target alive", which upstream leaves unchecked.
pub type FilterFn<'a> = Box<dyn Fn(&DMatrix<f64>, &mut DMatrix<f64>) -> bool + 'a>;

/// `NO_FILTER`. Upstream is a `static const FilterFn` value; Rust has no
/// equally ergonomic static boxed-closure value, so this port uses a
/// zero-capture factory function instead -- the natural replacement, since
/// the returned closure captures nothing and needs no lifetime bound.
pub fn no_filter() -> FilterFn<'static> {
    Box::new(|_values, _filtered_values| true)
}

/// `simpleSmoothingMatrix`: builds `stomp::generateSmoothingMatrix` once,
/// for `dt = 1.0` (upstream's own hardcoded placeholder -- a finite-
/// difference step for approximating acceleration, unrelated to waypoint
/// timing; see `conversion_functions`' module doc, "Deviation:
/// unparameterized-by-construction"), and returns a filter that overwrites
/// `filtered_values` in place with that matrix applied to each of its rows.
///
/// Upstream's closure signature is `(const Eigen::MatrixXd& /*values*/,
/// Eigen::MatrixXd& filtered_values)` -- the first argument is unused, and
/// upstream applies the smoothing matrix to whatever `filtered_values`
/// already holds when the filter runs, not to `values`. This port reproduces
/// that exactly: `values` is ignored here too. Per row `r` (one joint's
/// values across all timesteps), upstream computes `r^T := M * r^T`; over
/// the whole matrix that is `filtered_values := filtered_values * M^T`,
/// which this port computes directly rather than looping per row.
///
/// # Errors
///
/// [`Error::Other`] if [`cspace_stomp_core::generate_smoothing_matrix`]'s
/// control-cost matrix `R` is not invertible for `num_timesteps` -- upstream
/// has no such check (`generateSmoothingMatrix` calls C++'s unchecked
/// `FullPivLU::inverse()`); see `cspace_stomp_core`'s own module doc for why
/// this port makes that failure explicit instead of propagating a garbage
/// matrix.
pub fn simple_smoothing_matrix(num_timesteps: usize) -> Result<FilterFn<'static>> {
    let smoothing_matrix = cspace_stomp_core::generate_smoothing_matrix(num_timesteps, 1.0)
        .ok_or_else(|| {
            Error::Other(format!(
                "generate_smoothing_matrix({num_timesteps}, 1.0): control cost matrix R is \
                 not invertible"
            ))
        })?;
    let smoothing_matrix_transpose = smoothing_matrix.transpose();
    Ok(Box::new(move |_values, filtered_values| {
        *filtered_values = &*filtered_values * &smoothing_matrix_transpose;
        true
    }))
}

/// `enforcePositionBounds`: a filter that clips every waypoint's active-joint
/// positions to `group`'s joint bounds in place.
///
/// # Errors
///
/// [`cspace_core::error::Error::Other`] up front if any of `group`'s active joints
/// is not single-variable -- see `conversion_functions`' module doc,
/// "Single-variable-joint precondition"; the same precondition applies here,
/// since row `i` of the filtered matrix is assumed to be joint `i`'s one
/// scalar value.
pub fn enforce_position_bounds<'a>(
    robot_model: &'a RobotModel,
    group: &'a JointModelGroup,
) -> Result<FilterFn<'a>> {
    for name in group.active_joint_names() {
        let variable_count = robot_model.joint_model(name)?.variable_count();
        require_single_variable(name, variable_count)?;
    }
    Ok(Box::new(move |values, filtered_values| {
        *filtered_values = values.clone();
        for (i, name) in group.active_joint_names().iter().enumerate() {
            let joint = robot_model
                .joint_model(name)
                .expect("checked in enforce_position_bounds's own constructor");
            for j in 0..filtered_values.ncols() {
                let mut var = [filtered_values[(i, j)]];
                joint.enforce_position_bounds(&mut var);
                filtered_values[(i, j)] = var[0];
            }
        }
        true
    }))
}

/// `chain`: applies each of `filter_functions` in order, each one's output
/// feeding the next one's input.
pub fn chain<'a>(filter_functions: Vec<FilterFn<'a>>) -> FilterFn<'a> {
    Box::new(move |values, filtered_values| {
        let mut values_in = values.clone();
        for filter_fn in &filter_functions {
            filter_fn(&values_in, filtered_values);
            values_in = filtered_values.clone();
        }
        true
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use cspace_core::model::MeshSearchPaths;
    use cspace_core::srdf::SrdfModel;
    use std::fs;

    // `R` (the control cost matrix `generate_smoothing_matrix` inverts) is
    // `dt * A^T * A` for a finite-difference matrix `A` -- positive
    // semi-definite by construction, and STOMP's literature assumes it is
    // positive *definite* (invertible) for any `num_timesteps >= 1, dt >
    // 0`; that is the premise the whole algorithm relies on, which is
    // exactly why upstream never checks it. There is no realistic
    // `(num_timesteps, dt)` input through this public API that makes it
    // singular, so unlike `MultivariateGaussian::new` (which takes a
    // caller-supplied covariance and so can be handed a genuinely
    // indefinite one directly), `simple_smoothing_matrix`'s `Err` branch
    // has no reachable test input and is not exercised here.

    #[test]
    fn simple_smoothing_matrix_applies_the_smoothing_matrix_to_each_row() {
        let num_timesteps = 4;
        let filter = simple_smoothing_matrix(num_timesteps).unwrap();
        let expected_m = cspace_stomp_core::generate_smoothing_matrix(num_timesteps, 1.0)
            .expect("control_cost_matrix_R is invertible for num_timesteps=4, dt=1.0");

        // Two joint rows, num_timesteps columns each -- distinct values per
        // row so a row-transposition bug would show up as a mismatch.
        let filtered_before = DMatrix::from_row_slice(
            2,
            num_timesteps,
            &[1.0, 2.0, 3.0, 4.0, -1.0, 0.5, 2.5, -3.0],
        );
        let values = DMatrix::zeros(2, num_timesteps); // must be ignored, see next test
        let mut filtered_values = filtered_before.clone();
        assert!(filter(&values, &mut filtered_values));

        let expected = &filtered_before * expected_m.transpose();
        assert_eq!(filtered_values, expected);
    }

    #[test]
    fn simple_smoothing_matrix_ignores_the_values_argument() {
        // Upstream's own closure signature marks its first parameter
        // `/*values*/` -- unread. The filter must transform whatever
        // `filtered_values` already holds, regardless of what `values` is.
        let filter = simple_smoothing_matrix(3).unwrap();
        let filtered_before = DMatrix::from_row_slice(1, 3, &[1.0, 2.0, 3.0]);

        let mut via_zero_values = filtered_before.clone();
        assert!(filter(&DMatrix::zeros(1, 3), &mut via_zero_values));

        let mut via_other_values = filtered_before.clone();
        assert!(filter(
            &DMatrix::from_row_slice(1, 3, &[100.0, -50.0, 7.0]),
            &mut via_other_values
        ));

        assert_eq!(via_zero_values, via_other_values);
    }

    #[test]
    fn simple_smoothing_matrix_for_zero_timesteps_matches_the_dimension_not_a_panic() {
        let filter = simple_smoothing_matrix(0).unwrap();
        let mut filtered_values = DMatrix::<f64>::zeros(2, 0);
        assert!(filter(&DMatrix::zeros(2, 0), &mut filtered_values));
        assert_eq!(filtered_values.shape(), (2, 0));
    }

    fn fixture_path(file_name: &str) -> String {
        format!(
            concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
            file_name
        )
    }

    fn panda_model() -> RobotModel {
        let urdf_path = fixture_path("panda.urdf");
        let srdf_path = fixture_path("panda.srdf");
        let urdf_xml =
            fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
        let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("fixture model must build")
    }

    fn pr2_model() -> RobotModel {
        let urdf_path = fixture_path("pr2.urdf");
        let srdf_path = fixture_path("pr2.srdf");
        let urdf_xml =
            fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
        let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
        let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
            .expect("fixture model must build")
    }

    #[test]
    fn no_filter_leaves_filtered_values_untouched_and_returns_true() {
        let filter = no_filter();
        let values = DMatrix::from_element(2, 2, 1.0);
        let mut filtered = DMatrix::zeros(2, 2);
        assert!(filter(&values, &mut filtered));
        // NO_FILTER never writes `filtered_values` at all upstream -- this
        // is the exact reproduction: `filtered` stays whatever the caller
        // had in it.
        assert_eq!(filtered, DMatrix::zeros(2, 2));
    }

    #[test]
    fn enforce_position_bounds_clips_an_out_of_range_value_and_leaves_an_in_range_one_alone() {
        let model = panda_model();
        let group = model.joint_model_group("panda_arm").unwrap();
        let n = group.active_joint_names().len();
        let filter = enforce_position_bounds(&model, group).unwrap();

        // Column 0: every joint driven far past its max bound (every panda
        // arm joint's position bound is well within +/-100 rad). Column 1:
        // left at 0.0, in bounds for every panda arm joint, unperturbed.
        let mut values = DMatrix::zeros(n, 2);
        for i in 0..n {
            values[(i, 0)] = 100.0;
        }
        let mut filtered = DMatrix::zeros(n, 2);
        assert!(filter(&values, &mut filtered));

        for i in 0..n {
            let joint = model.joint_model(&group.active_joint_names()[i]).unwrap();
            // `JointModel::enforce_position_bounds` returns `true`
            // unconditionally for a revolute joint -- upstream
            // `RevoluteJointModel::enforcePositionBounds` does too
            // (verified against moveit2 @
            // e017c91ee12984393a28ba246075c65f69cde3bf's
            // `revolute_joint_model.cpp`, a faithfully-ported quirk, not a
            // bug in this port) -- so idempotency is checked via
            // `satisfies_position_bounds`, not the return value.
            assert!(
                joint.satisfies_position_bounds(&[filtered[(i, 0)]], 0.0),
                "column 0, joint {i} was not clipped into its own bounds"
            );
            assert_ne!(
                filtered[(i, 0)],
                100.0,
                "column 0, joint {i} should have been clipped away from 100.0"
            );
            assert_eq!(
                filtered[(i, 1)],
                values[(i, 1)],
                "column 1, joint {i} was already in bounds and should be unchanged"
            );
        }
    }

    // Assertion-discrimination sweep (round 2): the previous version of
    // this test searched every fixture group at runtime for one containing
    // a multi-variable joint, and returned early -- skipping its own
    // assertion -- if none was found. Probing with `eprintln!` showed the
    // panda fixture (the only one this module loaded) never has such a
    // group, so the search always failed and the `assert!` at the bottom
    // never ran: the test passed vacuously on every run, asserting
    // nothing. `pr2_model` (above) loads a fixture that does have one:
    // PR2's SRDF "base" group's sole active joint is `world_joint`, the
    // planar virtual joint from `pr2.srdf`'s `<virtual_joint type="planar">`
    // (3 variables). The assertion now names both the joint and the guard
    // that must have fired -- `require_single_variable`'s only Err site --
    // rather than a bare `.is_err()`. Reachability bite: no-op
    // `require_single_variable`'s `variable_count != 1` check -> the
    // now-Ok result fails `.unwrap_err()`. There is only one guard in
    // `enforce_position_bounds`'s loop (one `require_single_variable` call
    // per joint, no sibling branch), so no separate discrimination bite
    // applies.
    #[test]
    fn enforce_position_bounds_rejects_a_multi_variable_joint() {
        let model = pr2_model();
        let group = model.joint_model_group("base").unwrap();
        let Err(err) = enforce_position_bounds(&model, group) else {
            panic!("expected enforce_position_bounds to reject world_joint's 3 variables");
        };
        assert!(
            err.to_string().contains("world_joint") && err.to_string().contains("3 variables"),
            "expected the single-variable guard to name world_joint's 3 variables, got: {err}"
        );
    }

    #[test]
    fn chain_of_two_filters_applies_both_in_order() {
        // First filter doubles every value; second filter adds 1. Applied in
        // order, `chain` must produce `2*x + 1`, not `2*(x+1)`.
        let double: FilterFn<'static> = Box::new(|values, filtered| {
            *filtered = values * 2.0;
            true
        });
        let add_one: FilterFn<'static> = Box::new(|values, filtered| {
            *filtered = values.add_scalar(1.0);
            true
        });
        let chained = chain(vec![double, add_one]);

        let values = DMatrix::from_element(1, 1, 3.0);
        let mut filtered = DMatrix::zeros(1, 1);
        assert!(chained(&values, &mut filtered));
        assert_eq!(filtered[(0, 0)], 7.0);
    }

    #[test]
    fn chain_of_zero_filters_is_a_no_op_that_still_returns_true() {
        let chained: FilterFn<'static> = chain(Vec::new());
        let values = DMatrix::from_element(1, 1, 5.0);
        let mut filtered = DMatrix::zeros(1, 1);
        assert!(chained(&values, &mut filtered));
    }
}
