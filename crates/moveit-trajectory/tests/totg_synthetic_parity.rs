// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `totg` op, group-driven
//! branch, against the crate-local `totg_synthetic.urdf`/`.srdf` fixture
//! (see that file's doc comment). Closes the two coverage holes round 4's
//! report named as verified by code inspection only:
//!
//! - **Multi-DOF active joints.** `planar_group` is a single `planar`
//!   joint (`planar_joint/x`, `planar_joint/y`, `planar_joint/theta`),
//!   the first fixture in this workspace with a multi-DOF joint reachable
//!   by a group driven through the `TimeOptimalTrajectoryGeneration`
//!   adapter. Two cases: a plain two-waypoint move, and a three-waypoint
//!   move crossing `theta`'s `[-pi, pi]` seam (`-3.0` -> `2.5` -> `0.0`,
//!   with asymmetric per-variable limits) to exercise
//!   `active_joint_variables`'s per-joint `variable_names()` expansion
//!   against more than the trivial single-variable case.
//! - **`hasMixedJointTypes`.** `mixed_group` chains one prismatic and one
//!   revolute active joint, no mimics. `oracle.cpp`'s
//!   `hasMixedJointTypesForGroup` (re-implementing the private
//!   `TimeOptimalTrajectoryGeneration::hasMixedJointTypes`, cpp:1273-1288,
//!   which the oracle cannot call directly) returns `true` for it, and so
//!   does this crate's `has_mixed_joint_types` -- both computed from the
//!   group's own `getActiveJointModels()`/`active_joint_indices()`, not
//!   from scraping the `RCLCPP_WARN` upstream logs at the one real call
//!   site (cpp:1176-1180) that actually uses the predicate. The case
//!   still succeeds on both sides (`ok: true`): upstream never gates
//!   `computeTimeStamps` on this check, only warns that `path_tolerance`
//!   is unreliable for a mixed-type group -- see this round's report for
//!   why "rejects" in the round 5 task description means the predicate
//!   flags the group, not that the call fails.
//!
//! No fixture-side velocity/acceleration limit exists for either group
//! (`planar_joint` because moveit-model never reads multi-DOF joint
//! bounds from URDF at all; `mixed_prismatic_joint`/`mixed_revolute_joint`
//! because this test deliberately doesn't lean on URDF-sourced bounds
//! either, for the same reason `totg_robot_trajectory_parity.rs` doesn't),
//! so every case goes through `compute_time_stamps_with_limits`.
//!
//! # Tolerance
//!
//! Same numerics as `totg_robot_trajectory_parity.rs`; `TOL` matches.

use std::collections::HashMap;
use std::fs;

use approx::assert_relative_eq;
use serde::Deserialize;

use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;
use moveit_trajectory::RobotTrajectory;
use moveit_trajectory::time_optimal_trajectory_generation::{
    TotgOptions, compute_time_stamps_with_limits, has_mixed_joint_types,
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

fn totg_synthetic() -> RobotModel {
    let urdf_path = fixture_path("totg_synthetic.urdf");
    let srdf_path = fixture_path("totg_synthetic.srdf");
    let urdf_xml =
        fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

#[derive(Deserialize)]
struct TotgSynRequestCase {
    waypoints: Vec<HashMap<String, f64>>,
    durations_from_previous: Vec<f64>,
    velocity_limits: HashMap<String, f64>,
    #[serde(default)]
    acceleration_limits: HashMap<String, f64>,
}

#[derive(Deserialize)]
struct TotgSynRequest {
    group: String,
    cases: Vec<TotgSynRequestCase>,
}

#[derive(Deserialize)]
struct TotgSynResultWaypoint {
    positions: HashMap<String, f64>,
    velocities: HashMap<String, f64>,
    accelerations: HashMap<String, f64>,
}

#[derive(Deserialize)]
struct TotgSynResultCase {
    ok: bool,
    has_mixed_joint_types: bool,
    #[serde(default)]
    durations_from_previous: Vec<f64>,
    #[serde(default)]
    waypoints: Vec<TotgSynResultWaypoint>,
}

#[derive(Deserialize)]
struct TotgSynResult {
    cases: Vec<TotgSynResultCase>,
}

#[derive(Deserialize)]
struct TotgSynResponseEntry {
    result: TotgSynResult,
}

#[test]
fn totg_synthetic_matches_the_oracle() {
    let model = totg_synthetic();

    let requests: Vec<TotgSynRequest> =
        serde_json::from_str(&read_fixture("totg_synthetic_request.json"))
            .expect("parse totg_synthetic_request.json");
    let responses: Vec<TotgSynResponseEntry> =
        serde_json::from_str(&read_fixture("totg_synthetic_response.json"))
            .expect("parse totg_synthetic_response.json");
    assert_eq!(requests.len(), responses.len());

    for (request, response) in requests.iter().zip(&responses) {
        assert_eq!(request.cases.len(), response.result.cases.len());

        let group = model
            .joint_model_group(&request.group)
            .unwrap_or_else(|e| panic!("group {}: joint_model_group: {e}", request.group));

        for (case_index, (case, expected)) in
            request.cases.iter().zip(&response.result.cases).enumerate()
        {
            let mut trajectory = RobotTrajectory::for_group_name(&model, &request.group)
                .unwrap_or_else(|e| {
                    panic!(
                        "group {}: case {case_index}: for_group_name: {e}",
                        request.group
                    )
                });

            assert_eq!(
                has_mixed_joint_types(&trajectory, group),
                expected.has_mixed_joint_types,
                "group {}: case {case_index}: has_mixed_joint_types mismatch",
                request.group
            );

            for (values, &dt) in case.waypoints.iter().zip(&case.durations_from_previous) {
                let mut state = RobotState::new(&model);
                state.set_to_default_values();
                for (name, &value) in values {
                    state
                        .set_variable_position(name, value)
                        .unwrap_or_else(|e| {
                            panic!(
                                "group {}: case {case_index}: set_variable_position: {e}",
                                request.group
                            )
                        });
                }
                trajectory
                    .add_suffix_way_point(state, dt)
                    .unwrap_or_else(|e| {
                        panic!(
                            "group {}: case {case_index}: add_suffix_way_point: {e}",
                            request.group
                        )
                    });
            }

            let options = TotgOptions::default();
            let result = compute_time_stamps_with_limits(
                &mut trajectory,
                &case.velocity_limits,
                &case.acceleration_limits,
                &options,
            );

            assert_eq!(
                result.is_ok(),
                expected.ok,
                "group {}: case {case_index}: compute_time_stamps_with_limits ok mismatch ({result:?})",
                request.group
            );
            if !expected.ok {
                continue;
            }

            assert_eq!(
                trajectory.way_point_count(),
                expected.waypoints.len(),
                "group {}: case {case_index}: waypoint count mismatch",
                request.group
            );
            assert_eq!(
                trajectory.way_point_count(),
                expected.durations_from_previous.len(),
                "group {}: case {case_index}: duration count mismatch",
                request.group
            );

            for waypoint_idx in 0..trajectory.way_point_count() {
                assert_relative_eq!(
                    trajectory.way_point_duration_from_previous(waypoint_idx),
                    expected.durations_from_previous[waypoint_idx],
                    epsilon = TOL
                );

                let waypoint = trajectory.way_point(waypoint_idx).unwrap_or_else(|e| {
                    panic!(
                        "group {}: case {case_index}: way_point({waypoint_idx}): {e}",
                        request.group
                    )
                });
                let expected_waypoint = &expected.waypoints[waypoint_idx];

                for (name, &expected_position) in &expected_waypoint.positions {
                    assert_relative_eq!(
                        waypoint.variable_position(name).unwrap_or_else(|e| panic!(
                            "group {}: case {case_index}: waypoint {waypoint_idx}: variable_position({name}): {e}",
                            request.group
                        )),
                        expected_position,
                        epsilon = TOL
                    );
                }
                for (name, &expected_velocity) in &expected_waypoint.velocities {
                    assert_relative_eq!(
                        waypoint.variable_velocity(name).unwrap_or_else(|e| panic!(
                            "group {}: case {case_index}: waypoint {waypoint_idx}: variable_velocity({name}): {e}",
                            request.group
                        )),
                        expected_velocity,
                        epsilon = TOL
                    );
                }
                for (name, &expected_acceleration) in &expected_waypoint.accelerations {
                    assert_relative_eq!(
                        waypoint.variable_acceleration(name).unwrap_or_else(|e| panic!(
                            "group {}: case {case_index}: waypoint {waypoint_idx}: variable_acceleration({name}): {e}",
                            request.group
                        )),
                        expected_acceleration,
                        epsilon = TOL
                    );
                }
            }
        }
    }
}
