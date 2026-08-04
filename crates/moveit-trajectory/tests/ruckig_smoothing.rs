// Copyright (c) 2021, PickNik Robotics
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/trajectory_processing/test/test_ruckig_traj_smoothing.cpp
//
// Every upstream `RuckigTests` case is ported. `zeroVelocities()`/
// `zeroAccelerations()` are not called anywhere: a freshly constructed
// `RobotState` already starts with all-zero velocity/acceleration storage
// (see `moveit-state`'s `RobotState::new`), so there is nothing to zero.
// `single_waypoint`'s lone waypoint is added with duration `0.0`, not
// upstream's `DEFAULT_TIMESTEP`: this port's `duration_from_previous[0] ==
// 0.0` invariant (see `robot_trajectory.rs`'s "Deviations from upstream")
// makes upstream's value an `Err` here.

//! Ported `test_ruckig_traj_smoothing.cpp` cases, plus boundary tests for
//! `apply_smoothing`/`apply_smoothing_with_limits`'s own invariants (missing
//! group, empty/single-waypoint trajectories, duplicate waypoints).

use std::collections::HashMap;
use std::fs;

use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;
use moveit_trajectory::RobotTrajectory;
use moveit_trajectory::ruckig_smoothing::{
    SmoothingOptions, apply_smoothing, apply_smoothing_with_limits,
};

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
        file_name
    )
}

fn panda() -> RobotModel {
    let urdf_path = fixture_path("panda.urdf");
    let srdf_path = fixture_path("panda.srdf");
    let urdf_xml =
        fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

const DEFAULT_TIMESTEP: f64 = 0.1;
const JOINT_GROUP: &str = "panda_arm";

// ---- basic_trajectory -------------------------------------------------

#[test]
fn basic_trajectory_smooths_successfully() {
    let model = panda();
    let mut trajectory = RobotTrajectory::for_group_name(&model, JOINT_GROUP).unwrap();

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    // First waypoint is default joint positions.
    trajectory.add_suffix_way_point(state.clone(), 0.0).unwrap();

    // Second waypoint has slightly-different joint positions.
    let value = state.variable_position("panda_joint1").unwrap();
    state
        .set_variable_position("panda_joint1", value + 0.05)
        .unwrap();
    trajectory
        .add_suffix_way_point(state, DEFAULT_TIMESTEP)
        .unwrap();

    assert!(apply_smoothing(&mut trajectory, &SmoothingOptions::default()).is_ok());
}

// ---- basic_trajectory_with_custom_limits -------------------------------

#[test]
fn basic_trajectory_with_custom_limits_smooths_successfully() {
    let model = panda();
    let mut trajectory = RobotTrajectory::for_group_name(&model, JOINT_GROUP).unwrap();

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    trajectory.add_suffix_way_point(state.clone(), 0.0).unwrap();

    let value = state.variable_position("panda_joint1").unwrap();
    state
        .set_variable_position("panda_joint1", value + 0.05)
        .unwrap();
    trajectory
        .add_suffix_way_point(state, DEFAULT_TIMESTEP)
        .unwrap();

    let velocity_limits = HashMap::from([("panda_joint1".to_string(), 1.3)]);
    let acceleration_limits = HashMap::from([
        ("panda_joint2".to_string(), 2.3),
        ("panda_joint3".to_string(), 3.3),
    ]);
    let jerk_limits = HashMap::from([("panda_joint5".to_string(), 100.0)]);

    assert!(
        apply_smoothing_with_limits(
            &mut trajectory,
            &velocity_limits,
            &acceleration_limits,
            &jerk_limits,
            &SmoothingOptions::default(),
        )
        .is_ok()
    );
}

// ---- trajectory_duration -----------------------------------------------

#[test]
fn trajectory_duration_is_within_ten_percent_of_the_analytical_solution() {
    // Compare against the OJET online trajectory generator. Ruckig applies
    // defaults when the RobotModel has none.
    const IDEAL_DURATION: f64 = 0.210;

    let model = panda();
    let mut trajectory = RobotTrajectory::for_group_name(&model, JOINT_GROUP).unwrap();

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    // Special attention to joint 0: it is the only joint to move in this
    // test, with zero velocity/acceleration at both endpoints.
    state.set_variable_position("panda_joint1", 0.0).unwrap();
    trajectory.add_suffix_way_point(state.clone(), 0.0).unwrap();

    state.set_variable_position("panda_joint1", 0.1).unwrap();
    trajectory
        .add_suffix_way_point(state, DEFAULT_TIMESTEP)
        .unwrap();

    assert!(apply_smoothing(&mut trajectory, &SmoothingOptions::default()).is_ok());

    // No waypoint durations of zero except the first.
    for waypoint_idx in 1..trajectory.way_point_count() - 1 {
        assert_ne!(
            trajectory.way_point_duration_from_previous(waypoint_idx),
            0.0
        );
    }

    // The trajectory duration should be within 10% of the analytical solution,
    // since the retry loop extends the duration by 10% at every iteration.
    let last = trajectory.way_point_count() - 1;
    let actual = trajectory.way_point_duration_from_start(last);
    assert!(
        actual > 0.9999 * IDEAL_DURATION,
        "actual = {actual}, ideal = {IDEAL_DURATION}"
    );
    assert!(
        actual < 1.11 * IDEAL_DURATION,
        "actual = {actual}, ideal = {IDEAL_DURATION}"
    );
}

// ---- single_waypoint -----------------------------------------------------

#[test]
fn single_waypoint_trajectory_is_unmodified() {
    let model = panda();
    let mut trajectory = RobotTrajectory::for_group_name(&model, JOINT_GROUP).unwrap();

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    trajectory.add_suffix_way_point(state.clone(), 0.0).unwrap();

    // With only one waypoint, Ruckig cannot smooth the trajectory: it should
    // pass the trajectory through unmodified and return `Ok`.
    assert!(apply_smoothing(&mut trajectory, &SmoothingOptions::default()).is_ok());

    let new_first_waypoint = trajectory.first_way_point().unwrap();
    for variable_name in model.variable_names() {
        assert_eq!(
            state.variable_position(variable_name).unwrap(),
            new_first_waypoint.variable_position(variable_name).unwrap(),
        );
    }
}

// ---- Boundary tests -----------------------------------------------------

#[test]
fn no_group_set_is_an_error() {
    let model = panda();
    let mut trajectory = RobotTrajectory::new(&model);
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    trajectory.add_suffix_way_point(state.clone(), 0.0).unwrap();
    trajectory
        .add_suffix_way_point(state, DEFAULT_TIMESTEP)
        .unwrap();

    assert!(trajectory.group().is_none());
    // `apply_smoothing` has 3 reachable `Error::other` sites (missing
    // group, ruckig calculate failure, ruckig smoothing-result failure); a
    // bare `.is_err()` cannot say which fired
    // (assertion-discrimination-round2.md sec. 3).
    assert!(
        apply_smoothing(&mut trajectory, &SmoothingOptions::default())
            .unwrap_err()
            .to_string()
            .contains("did not set the group")
    );
}

#[test]
fn empty_trajectory_with_a_group_is_a_no_op() {
    let model = panda();
    let mut trajectory = RobotTrajectory::for_group_name(&model, JOINT_GROUP).unwrap();
    assert!(trajectory.is_empty());

    assert!(apply_smoothing(&mut trajectory, &SmoothingOptions::default()).is_ok());
    assert!(trajectory.is_empty());
}

#[test]
fn duplicate_consecutive_waypoints_do_not_hang() {
    let model = panda();
    let mut trajectory = RobotTrajectory::for_group_name(&model, JOINT_GROUP).unwrap();

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    trajectory.add_suffix_way_point(state.clone(), 0.0).unwrap();
    trajectory
        .add_suffix_way_point(state, DEFAULT_TIMESTEP)
        .unwrap();

    assert!(apply_smoothing(&mut trajectory, &SmoothingOptions::default()).is_ok());
}

#[test]
fn mitigate_overshoot_option_does_not_hang_or_error_on_an_ordinary_trajectory() {
    let model = panda();
    let mut trajectory = RobotTrajectory::for_group_name(&model, JOINT_GROUP).unwrap();

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    trajectory.add_suffix_way_point(state.clone(), 0.0).unwrap();
    let value = state.variable_position("panda_joint1").unwrap();
    state
        .set_variable_position("panda_joint1", value + 0.05)
        .unwrap();
    trajectory
        .add_suffix_way_point(state, DEFAULT_TIMESTEP)
        .unwrap();

    let options = SmoothingOptions {
        mitigate_overshoot: true,
        ..SmoothingOptions::default()
    };
    assert!(apply_smoothing(&mut trajectory, &options).is_ok());
}
