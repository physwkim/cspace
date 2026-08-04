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
//!
//! # Deviation: unparameterized-by-construction (round 21 correction)
//!
//! **A round-20 mistake, corrected here.** This module's doc previously
//! claimed upstream's `fillRobotTrajectory` placeholder `dt = 0.1` carries a
//! comment "the actual timestep duration will be computed by a planner
//! adapter after solving". That sentence does not exist at
//! `conversion_functions.hpp`'s `addSuffixWayPoint(waypoint, 0.1 /*
//! placeholder dt */)` call -- the comment there is only `/* placeholder dt
//! */`. The quoted sentence is real, but it is attached to a *different*
//! `dt` in a different file: `filter_functions.hpp`'s `simpleSmoothingMatrix`
//! passes `dt = 1.0` to `stomp::generateSmoothingMatrix` as the finite-
//! difference step for approximating waypoint *acceleration*, not a
//! waypoint duration. Confirmed by reading both sites directly, moveit2 @
//! `e017c91ee12984393a28ba246075c65f69cde3bf`. The two `0.1`/`1.0` values
//! are unrelated in both meaning and in the code path that touches them.
//!
//! Fixing the citation is not sufficient by itself. What the (wrongly
//! quoted) sentence would have guaranteed -- "some later stage computes the
//! real timestep duration, so the placeholder is harmless" -- has no
//! equivalent anywhere in this port. Upstream's own guarantee, even where
//! it does apply (to `simpleSmoothingMatrix`'s `dt`, not this function's),
//! comes from a "planner adapter" that lives in the ROS integration layer
//! (`move_group`'s planning-response-adapter pipeline), which this
//! workspace does not port (D1/D2). Read `stomp_moveit_planning_context.cpp`
//! directly to confirm: `solveWithStomp` calls `fillRobotTrajectory` and
//! hands the result straight to `res.trajectory` with no time-
//! parameterization step in between anywhere in that file. So even
//! upstream's real behaviour is "whatever ROS pipeline the caller has
//! configured might fix this timing, or might not" -- and this port has no
//! such pipeline at all. A `0.1`-per-waypoint duration silently leaving
//! [`fill_robot_trajectory`] as if it were real timing is exactly the
//! "wrong value flows out uncaught" shape this project's structural-fix
//! doctrine targets; a doc comment saying "don't trust this" is not
//! enforcement.
//!
//! [`fill_robot_trajectory`] and [`matrix_to_robot_trajectory`] therefore
//! return [`UnparameterizedTrajectory`], not a bare [`RobotTrajectory`]. The
//! wrapper exposes waypoint *positions* ([`UnparameterizedTrajectory::way_point_count`])
//! but no duration accessor, so there is no way to read a placeholder
//! duration off of it by mistake. The only way to obtain a real
//! [`RobotTrajectory`] is [`UnparameterizedTrajectory::into_uniformly_timed`],
//! which requires the caller to name an explicit `dt` -- the caller is
//! visibly asserting "a uniform discretization at this rate is acceptable
//! for my use", instead of silently inheriting a value this port picked for
//! an unrelated reason. (Depending on `moveit-trajectory`/`moveit-smoothing`'s
//! TOTG to compute a real time parameterization here was considered and
//! rejected for this round: STOMP's own upstream never calls a time-
//! parameterization algorithm itself either, so pulling one in here would
//! be adding behaviour upstream never had, not porting behaviour that
//! exists. Type-enforcing the caller's choice is the smaller, faithful
//! fix.) Every waypoint's duration is set to an inert `0.0` internally
//! during construction, not upstream's `0.1` -- nothing can observe that
//! value before [`UnparameterizedTrajectory::into_uniformly_timed`]
//! overwrites it, so there is nothing for it to faithfully reproduce.

use moveit_model::{JointModelGroup, RobotModel};
use moveit_state::RobotState;
use moveit_trajectory::RobotTrajectory;
use nalgebra::{DMatrix, DVector};

use moveit_error::Result;

use crate::require_single_variable;

/// The output of [`fill_robot_trajectory`]/[`matrix_to_robot_trajectory`]: a
/// [`RobotTrajectory`] whose waypoint positions are STOMP's solved matrix,
/// but whose per-waypoint durations are not yet real timing. See this
/// module's "Deviation: unparameterized-by-construction".
pub struct UnparameterizedTrajectory<'m>(RobotTrajectory<'m>);

impl<'m> UnparameterizedTrajectory<'m> {
    fn for_group(robot_model: &'m RobotModel, group: Option<&'m JointModelGroup>) -> Self {
        Self(RobotTrajectory::for_group(robot_model, group))
    }

    /// The number of waypoints. Position data only -- no duration accessor
    /// is exposed by this type.
    pub fn way_point_count(&self) -> usize {
        self.0.way_point_count()
    }

    /// Consumes this trajectory, assigning every waypoint after the first a
    /// uniform `dt` (waypoint 0's duration stays structurally `0.0`, per
    /// [`RobotTrajectory`]'s own invariant), and returns the now
    /// genuinely-timed [`RobotTrajectory`]. See this module's "Deviation:
    /// unparameterized-by-construction" for why this must be an explicit
    /// call rather than a default this port picks silently.
    pub fn into_uniformly_timed(mut self, dt: f64) -> Result<RobotTrajectory<'m>> {
        for i in 1..self.0.way_point_count() {
            self.0.set_way_point_duration_from_previous(i, dt)?;
        }
        Ok(self.0)
    }
}

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
/// waypoint's non-`group` joint values. See this module's "Deviation:
/// unparameterized-by-construction" for why `trajectory`'s type carries no
/// duration guarantee until [`UnparameterizedTrajectory::into_uniformly_timed`]
/// is called.
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
    trajectory: &mut UnparameterizedTrajectory<'m>,
) -> Result<()> {
    trajectory.0.clear();
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
        // Inert: no accessor on `UnparameterizedTrajectory` exposes this
        // value before `into_uniformly_timed` overwrites it.
        trajectory.0.add_suffix_way_point(waypoint, 0.0)?;
    }
    Ok(())
}

/// `matrixToRobotTrajectory`: builds a fresh [`UnparameterizedTrajectory`]
/// for `group` from `trajectory_values`.
pub fn matrix_to_robot_trajectory<'m>(
    trajectory_values: &DMatrix<f64>,
    reference_state: &RobotState<'m>,
    group: &'m JointModelGroup,
) -> Result<UnparameterizedTrajectory<'m>> {
    let mut trajectory = UnparameterizedTrajectory::for_group(reference_state.model(), Some(group));
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

        let unparameterized = matrix_to_robot_trajectory(&values, &reference, group).unwrap();
        assert_eq!(unparameterized.way_point_count(), 3);

        let trajectory = unparameterized.into_uniformly_timed(0.1).unwrap();
        assert_eq!(trajectory.way_point_duration_from_previous(0), 0.0);
        assert_eq!(trajectory.way_point_duration_from_previous(1), 0.1);
        assert_eq!(trajectory.way_point_duration_from_previous(2), 0.1);

        let round_tripped = robot_trajectory_to_matrix(&trajectory, group).unwrap();
        assert_eq!(round_tripped, values);
    }

    #[test]
    fn into_uniformly_timed_leaves_way_point_zero_at_structurally_zero() {
        // `set_way_point_duration_from_previous` skips index 0 entirely --
        // `RobotTrajectory`'s own invariant (`duration_from_previous[0]` is
        // always `0.0`) is preserved, not fought, by never touching it.
        let model = panda_model();
        let group = model.joint_model_group("panda_arm").unwrap();
        let mut reference = RobotState::new(&model);
        reference.set_to_default_values();
        let n = group.active_joint_names().len();

        let values = DMatrix::zeros(n, 1);
        let unparameterized = matrix_to_robot_trajectory(&values, &reference, group).unwrap();
        let trajectory = unparameterized.into_uniformly_timed(0.25).unwrap();
        assert_eq!(trajectory.way_point_duration_from_previous(0), 0.0);
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
