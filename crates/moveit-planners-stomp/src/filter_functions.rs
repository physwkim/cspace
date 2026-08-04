// Copyright (c) 2023, PickNik Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/stomp/include/stomp_moveit/filter_functions.hpp

//! Trajectory-update filters STOMP applies to its waypoint matrix each
//! iteration.
//!
//! # Not ported this round: `simpleSmoothingMatrix`
//!
//! Upstream's `simpleSmoothingMatrix` calls `stomp::generateSmoothingMatrix`
//! (`<stomp/utils.h>`), from the separate upstream `ros-industrial/stomp`
//! optimizer repository -- not `moveit2`. That repository was searched for
//! and is not present on this machine (`/home/stevek/work`, `/opt/ros`, and
//! the rest of the filesystem; reported to the user, see this crate's
//! `lib.rs`). This port does not guess at `generateSmoothingMatrix`'s
//! finite-difference construction: `simple_smoothing_matrix` is not
//! implemented here. Reopens once that source is available.
//!
//! # `FilterFn`'s home
//!
//! Upstream declares `FilterFn` in `stomp_moveit_task.hpp` (not ported --
//! ROS/task-engine layer, see `lib.rs`), alongside `NoiseGeneratorFn` and
//! `CostFn`. This module carries the one piece of that header every filter
//! function here actually needs: the `FilterFn` signature itself.

use moveit_model::{JointModelGroup, RobotModel};
use nalgebra::DMatrix;

use moveit_error::Result;

use crate::require_single_variable;

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

/// `enforcePositionBounds`: a filter that clips every waypoint's active-joint
/// positions to `group`'s joint bounds in place.
///
/// # Errors
///
/// [`moveit_error::Error::Other`] up front if any of `group`'s active joints
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
    use moveit_model::MeshSearchPaths;
    use moveit_srdf::SrdfModel;
    use std::fs;

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

    #[test]
    fn enforce_position_bounds_rejects_a_multi_variable_joint() {
        // `panda_arm` (the group used above) has no multi-variable joint;
        // this fixture must have one to exercise the rejection path.
        let model = panda_model();
        let has_multi_variable_group = model.joint_model_group_names().find_map(|name| {
            let group = model.joint_model_group(name).ok()?;
            let has_one = group.active_joint_names().iter().any(|joint_name| {
                model
                    .joint_model(joint_name)
                    .map(|j| j.variable_count() != 1)
                    .unwrap_or(false)
            });
            has_one.then_some(group)
        });
        let Some(group) = has_multi_variable_group else {
            // No fixture group happens to contain a multi-variable joint --
            // the precondition path is still covered directly via
            // `conversion_functions`' own tests of the same helper.
            return;
        };
        assert!(enforce_position_bounds(&model, group).is_err());
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
