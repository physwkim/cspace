// Copyright (c) 2023, PickNik Inc.
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_planners/stomp/include/stomp_moveit/conversion_functions.hpp

//! Conversions between [`RobotState`]/[`RobotTrajectory`] waypoints and
//! STOMP's own `Eigen::MatrixXd` representation: rows are a
//! [`JointModelGroup`]'s active-joint values, columns are timesteps.
//!
//! # Single-variable-joint precondition
//!
//! Upstream's matrix has exactly one row per active joint
//! (`group->getActiveJointModels().size()` rows), and every read or write of
//! a row touches exactly one `double` per joint: `getPositions` dereferences
//! `state.getJointPositions(joint)` once, `setJointPositions` writes through
//! `&values[joint_index]`, a pointer to one `double`. That is only correct
//! when every active joint has exactly one variable (revolute or
//! prismatic) -- a multi-variable joint (planar: 3, floating: 7) in the
//! group would make upstream's raw-pointer C++ read or write past a
//! one-`double` slot, undefined behaviour either way. Every real STOMP call
//! site (`stomp_moveit_planning_context.cpp`, not ported -- see this
//! crate's `lib.rs`) only ever plans for arm-like single-DOF-joint groups,
//! so this has never been observed to matter upstream. This port does not
//! reproduce the UB: [`positions`] and [`set_positions`] return
//! [`moveit_error::Error::Other`] up front for any active joint whose
//! variable count is not `1`, naming the offending joint, instead of
//! silently reading or writing the first variable and discarding the rest.
//!
//! # Deviation: a concrete group, not an optional one
//!
//! Upstream's four functions take `reference_state`/`trajectory` alone and
//! fall back to `trajectory.getRobotModel()->getActiveJointModels()` (the
//! *whole* robot) when no group is set. Every real call site in
//! `stomp_moveit_planning_context.cpp` passes a concrete group; this port
//! requires `&JointModelGroup` explicitly rather than reproducing a
//! whole-robot fallback branch STOMP itself never exercises.

use moveit_model::JointModelGroup;
use moveit_state::RobotState;
use moveit_trajectory::RobotTrajectory;
use nalgebra::{DMatrix, DVector};

use moveit_error::Result;

use crate::require_single_variable;

/// `getPositions`: `group`'s active-joint position vector, read from
/// `state`.
pub fn positions(state: &RobotState<'_>, group: &JointModelGroup) -> Result<DVector<f64>> {
    let names = group.active_joint_names();
    let mut out = DVector::zeros(names.len());
    for (i, name) in names.iter().enumerate() {
        let slice = state.joint_position(name)?;
        require_single_variable(name, slice.len())?;
        out[i] = slice[0];
    }
    Ok(out)
}

/// `setJointPositions`: writes `values` into `state`, one entry per
/// `group`'s active joint.
///
/// # Panics
///
/// If `values.len()` does not equal `group.active_joint_names().len()`.
pub fn set_positions(
    values: &DVector<f64>,
    group: &JointModelGroup,
    state: &mut RobotState<'_>,
) -> Result<()> {
    let names = group.active_joint_names();
    assert_eq!(
        values.len(),
        names.len(),
        "values.len() must equal the group's active joint count"
    );
    for (i, name) in names.iter().enumerate() {
        let variable_count = state.model().joint_model(name)?.variable_count();
        require_single_variable(name, variable_count)?;
        state.set_joint_positions(name, &[values[i]])?;
    }
    Ok(())
}

/// `fillRobotTrajectory`: overwrites `trajectory` with one waypoint per
/// column of `trajectory_values`, cloning `reference_state` for every
/// waypoint's non-`group` joint values.
///
/// # Deviation: waypoint 0's duration is `0.0`, not upstream's placeholder `0.1`
///
/// Upstream pushes a placeholder `dt = 0.1` for every waypoint, including
/// the first, with a comment noting "the actual timestep duration will be
/// computed by a planner adapter after solving" -- i.e. this placeholder is
/// always overwritten downstream and its value is not load-bearing.
/// `moveit-trajectory`'s own [`RobotTrajectory::add_suffix_way_point`]
/// already establishes a stricter invariant than upstream ever enforced --
/// `duration_from_previous[0]` is structurally `0.0`, since waypoint 0 has
/// no previous waypoint to measure a gap from -- and rejects a nonzero
/// value there. This port satisfies that invariant instead of fighting it:
/// waypoint 0's `dt` is `0.0`, every later waypoint's is upstream's `0.1`.
///
/// # Errors
///
/// See [`positions`]/[`set_positions`]'s "Single-variable-joint
/// precondition".
///
/// # Panics
///
/// If `trajectory_values.nrows()` does not equal
/// `group.active_joint_names().len()`.
pub fn fill_robot_trajectory<'m>(
    trajectory_values: &DMatrix<f64>,
    reference_state: &RobotState<'m>,
    group: &JointModelGroup,
    trajectory: &mut RobotTrajectory<'m>,
) -> Result<()> {
    trajectory.clear();
    let names = group.active_joint_names();
    assert_eq!(
        trajectory_values.nrows(),
        names.len(),
        "trajectory_values must have one row per active joint"
    );
    for timestep in 0..trajectory_values.ncols() {
        let mut waypoint = reference_state.clone();
        let column = DVector::from_iterator(
            names.len(),
            (0..names.len()).map(|i| trajectory_values[(i, timestep)]),
        );
        set_positions(&column, group, &mut waypoint)?;
        let dt = if timestep == 0 { 0.0 } else { 0.1 };
        trajectory.add_suffix_way_point(waypoint, dt)?;
    }
    Ok(())
}

/// `matrixToRobotTrajectory`: builds a fresh [`RobotTrajectory`] for `group`
/// from `trajectory_values`.
pub fn matrix_to_robot_trajectory<'m>(
    trajectory_values: &DMatrix<f64>,
    reference_state: &RobotState<'m>,
    group: &'m JointModelGroup,
) -> Result<RobotTrajectory<'m>> {
    let mut trajectory = RobotTrajectory::for_group(reference_state.model(), Some(group));
    fill_robot_trajectory(trajectory_values, reference_state, group, &mut trajectory)?;
    Ok(trajectory)
}

/// `robotTrajectoryToMatrix`: the inverse of [`fill_robot_trajectory`].
pub fn robot_trajectory_to_matrix(
    trajectory: &RobotTrajectory<'_>,
    group: &JointModelGroup,
) -> Result<DMatrix<f64>> {
    let names = group.active_joint_names();
    let mut out = DMatrix::zeros(names.len(), trajectory.way_point_count());
    for timestep in 0..trajectory.way_point_count() {
        let waypoint = trajectory.way_point(timestep)?;
        let column = positions(waypoint, group)?;
        for (i, value) in column.iter().enumerate() {
            out[(i, timestep)] = *value;
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use moveit_model::{MeshSearchPaths, RobotModel};
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
    fn positions_then_set_positions_round_trips_through_the_matrix_representation() {
        let model = panda_model();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();

        let before = positions(&state, group).unwrap();
        assert_eq!(before.len(), group.active_joint_names().len());

        let perturbed = &before + DVector::from_element(before.len(), 0.01);
        set_positions(&perturbed, group, &mut state).unwrap();
        let after = positions(&state, group).unwrap();
        assert_eq!(after, perturbed);
    }

    #[test]
    fn set_positions_panics_on_a_length_mismatch() {
        let model = panda_model();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let wrong_length = DVector::from_element(group.active_joint_names().len() + 1, 0.0);
        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            set_positions(&wrong_length, group, &mut state)
        }));
        assert!(result.is_err());
    }

    #[test]
    fn fill_then_convert_back_round_trips_a_multi_waypoint_trajectory() {
        let model = panda_model();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut reference = RobotState::new(&model);
        reference.set_to_default_values();

        let n = group.active_joint_names().len();
        let mut values = DMatrix::zeros(n, 3);
        for j in 0..3 {
            for i in 0..n {
                values[(i, j)] = 0.01 * (i as f64) + 0.1 * (j as f64);
            }
        }

        let trajectory = matrix_to_robot_trajectory(&values, &reference, group).unwrap();
        assert_eq!(trajectory.way_point_count(), 3);
        assert_eq!(trajectory.way_point_duration_from_previous(0), 0.0);
        assert_eq!(trajectory.way_point_duration_from_previous(1), 0.1);
        assert_eq!(trajectory.way_point_duration_from_previous(2), 0.1);

        let round_tripped = robot_trajectory_to_matrix(&trajectory, group).unwrap();
        assert_eq!(round_tripped, values);
    }

    #[test]
    fn fill_robot_trajectory_clears_a_previously_populated_trajectory() {
        let model = panda_model();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut reference = RobotState::new(&model);
        reference.set_to_default_values();
        let n = group.active_joint_names().len();

        let first = DMatrix::zeros(n, 5);
        let mut trajectory = matrix_to_robot_trajectory(&first, &reference, group).unwrap();
        assert_eq!(trajectory.way_point_count(), 5);

        let second = DMatrix::zeros(n, 2);
        fill_robot_trajectory(&second, &reference, group, &mut trajectory).unwrap();
        assert_eq!(trajectory.way_point_count(), 2);
    }
}
