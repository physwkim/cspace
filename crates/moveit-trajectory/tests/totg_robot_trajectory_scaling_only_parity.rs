// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `totg` op, group-driven
//! branch, covering `time_optimal_trajectory_generation.rs`'s
//! [`compute_time_stamps`] (the scaling-only overload) end to end.
//!
//! `totg_robot_trajectory_parity.rs` exercises `compute_time_stamps_with_limits`
//! exclusively, because no fixture model in this workspace has
//! `acceleration_bounded` set: `moveit-model`'s URDF loader never reads an
//! acceleration limit (URDF has no such field), and until
//! `RobotModel::joint_model_mut` landed, nothing outside `moveit-model`
//! could set one programmatically either. That accessor now exists, so this
//! file closes the gap `time_optimal_trajectory_generation.rs`'s doc comment
//! used to record under "Known gap".
//!
//! `panda.urdf` already gives every `panda_arm` joint `velocity_bounded =
//! true` (from its `<limit velocity="...">` elements) — only acceleration
//! was missing. This test adds it the same way upstream's own
//! `joint_limits.yaml` loaders do: read the joint's current bounds via
//! [`JointModel::variable_bounds_msg`], overwrite only the acceleration
//! fields, write back via [`JointModel::set_variable_bounds_from_limits`].
//! `oracle.cpp`'s `totgRobotTrajectoryCase` does the oracle-side equivalent
//! (`JointModel::getVariableBoundsMsg`/`setVariableBounds`) when a case
//! carries an `"acceleration_bounds"` field — see that function's doc
//! comment. Both sides start from the same `panda.urdf`, so the two
//! mutations produce the same model.
//!
//! # Tolerance
//!
//! Same `TOL` as `totg_robot_trajectory_parity.rs`, for the same reason:
//! both sides run the identical Kunz & Stilman numerics
//! `totg_parity.rs` already validates bit-for-bit against the core-only
//! oracle path.

use std::collections::HashMap;
use std::fs;

use approx::assert_relative_eq;
use serde::Deserialize;

use moveit_model::joint::JointModel;
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;
use moveit_trajectory::RobotTrajectory;
use moveit_trajectory::time_optimal_trajectory_generation::{
    TotgOptions, compute_time_stamps, has_mixed_joint_types,
};

const TOL: f64 = 1e-6;

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        file_name
    )
}

fn read_fixture(file_name: &str) -> String {
    let path = fixture_path(file_name);
    fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

fn panda() -> RobotModel {
    let urdf_path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
        "panda.urdf"
    );
    let srdf_path = format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/{}"),
        "panda.srdf"
    );
    let urdf_xml =
        fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

/// Mirrors `oracle.cpp`'s `totgRobotTrajectoryCase` handling of a case's
/// `"acceleration_bounds"` field: read-modify-write through
/// [`JointModel::variable_bounds_msg`]/[`JointModel::set_variable_bounds_from_limits`]
/// so URDF-sourced position/velocity bounds are preserved.
fn set_acceleration_bound(joint: &mut JointModel, max_acceleration: f64) {
    let mut limits = joint.variable_bounds_msg();
    for limit in &mut limits {
        limit.has_acceleration_limits = true;
        limit.max_acceleration = max_acceleration;
    }
    joint.set_variable_bounds_from_limits(&limits);
}

#[derive(Deserialize)]
struct TotgRtScalingOnlyRequestCase {
    acceleration_bounds: HashMap<String, f64>,
    waypoints: Vec<HashMap<String, f64>>,
    durations_from_previous: Vec<f64>,
}

#[derive(Deserialize)]
struct TotgRtScalingOnlyRequest {
    group: String,
    cases: Vec<TotgRtScalingOnlyRequestCase>,
}

#[derive(Deserialize)]
struct TotgRtResultWaypoint {
    positions: HashMap<String, f64>,
    velocities: HashMap<String, f64>,
    accelerations: HashMap<String, f64>,
}

#[derive(Deserialize)]
struct TotgRtResultCase {
    ok: bool,
    has_mixed_joint_types: bool,
    #[serde(default)]
    durations_from_previous: Vec<f64>,
    #[serde(default)]
    waypoints: Vec<TotgRtResultWaypoint>,
}

#[derive(Deserialize)]
struct TotgRtResult {
    cases: Vec<TotgRtResultCase>,
}

#[derive(Deserialize)]
struct TotgRtResponseEntry {
    result: TotgRtResult,
}

#[test]
fn totg_robot_trajectory_scaling_only_matches_the_oracle() {
    let mut model = panda();

    let requests: Vec<TotgRtScalingOnlyRequest> = serde_json::from_str(&read_fixture(
        "totg_robot_trajectory_scaling_only_request.json",
    ))
    .expect("parse totg_robot_trajectory_scaling_only_request.json");
    let responses: Vec<TotgRtResponseEntry> = serde_json::from_str(&read_fixture(
        "totg_robot_trajectory_scaling_only_response.json",
    ))
    .expect("parse totg_robot_trajectory_scaling_only_response.json");
    assert_eq!(requests.len(), responses.len());
    let request = &requests[0];
    let response = &responses[0];
    assert_eq!(request.cases.len(), response.result.cases.len());

    for (case_index, (case, expected)) in
        request.cases.iter().zip(&response.result.cases).enumerate()
    {
        for (joint_name, &max_acceleration) in &case.acceleration_bounds {
            let joint = model
                .joint_model_mut(joint_name)
                .unwrap_or_else(|e| panic!("case {case_index}: joint_model_mut: {e}"));
            set_acceleration_bound(joint, max_acceleration);
        }

        let mut trajectory = RobotTrajectory::for_group_name(&model, &request.group)
            .unwrap_or_else(|e| panic!("case {case_index}: for_group_name: {e}"));
        let group = model
            .joint_model_group(&request.group)
            .unwrap_or_else(|e| panic!("case {case_index}: joint_model_group: {e}"));
        assert_eq!(
            has_mixed_joint_types(&trajectory, group),
            expected.has_mixed_joint_types,
            "case {case_index}: has_mixed_joint_types mismatch"
        );

        for (values, &dt) in case.waypoints.iter().zip(&case.durations_from_previous) {
            let mut state = RobotState::new(&model);
            state.set_to_default_values();
            for (name, &value) in values {
                state
                    .set_variable_position(name, value)
                    .unwrap_or_else(|e| panic!("case {case_index}: set_variable_position: {e}"));
            }
            trajectory
                .add_suffix_way_point(state, dt)
                .unwrap_or_else(|e| panic!("case {case_index}: add_suffix_way_point: {e}"));
        }

        let result = compute_time_stamps(&mut trajectory, &TotgOptions::default());

        assert_eq!(
            result.is_ok(),
            expected.ok,
            "case {case_index}: compute_time_stamps ok mismatch ({result:?})"
        );
        if !expected.ok {
            continue;
        }

        assert_eq!(
            trajectory.way_point_count(),
            expected.waypoints.len(),
            "case {case_index}: waypoint count mismatch"
        );
        assert_eq!(
            trajectory.way_point_count(),
            expected.durations_from_previous.len(),
            "case {case_index}: duration count mismatch"
        );

        for waypoint_idx in 0..trajectory.way_point_count() {
            assert_relative_eq!(
                trajectory.way_point_duration_from_previous(waypoint_idx),
                expected.durations_from_previous[waypoint_idx],
                epsilon = TOL
            );

            let waypoint = trajectory
                .way_point(waypoint_idx)
                .unwrap_or_else(|e| panic!("case {case_index}: way_point({waypoint_idx}): {e}"));
            let expected_waypoint = &expected.waypoints[waypoint_idx];

            for (name, &expected_position) in &expected_waypoint.positions {
                assert_relative_eq!(
                    waypoint.variable_position(name).unwrap_or_else(|e| panic!(
                        "case {case_index}: waypoint {waypoint_idx}: variable_position({name}): {e}"
                    )),
                    expected_position,
                    epsilon = TOL
                );
            }
            for (name, &expected_velocity) in &expected_waypoint.velocities {
                assert_relative_eq!(
                    waypoint.variable_velocity(name).unwrap_or_else(|e| panic!(
                        "case {case_index}: waypoint {waypoint_idx}: variable_velocity({name}): {e}"
                    )),
                    expected_velocity,
                    epsilon = TOL
                );
            }
            for (name, &expected_acceleration) in &expected_waypoint.accelerations {
                assert_relative_eq!(
                    waypoint.variable_acceleration(name).unwrap_or_else(|e| panic!(
                        "case {case_index}: waypoint {waypoint_idx}: variable_acceleration({name}): {e}"
                    )),
                    expected_acceleration,
                    epsilon = TOL
                );
            }
        }
    }
}
