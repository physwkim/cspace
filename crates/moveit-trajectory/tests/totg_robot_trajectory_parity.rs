// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `totg` op, group-driven
//! branch, covering `time_optimal_trajectory_generation.rs`'s
//! `RobotTrajectory` adapter (`compute_time_stamps_with_limits`).
//!
//! `totg_parity.rs` already cross-checks the model-independent
//! `Path`/`Trajectory` core against the oracle's core-only `totg` cases
//! (no `"group"` key). This file exercises the adapter layer on top of
//! that core: waypoint construction from a `RobotTrajectory`, scaling-
//! factor clamping, and write-back, via the same `totg` op with a
//! `"group"` key set (see `oracle.cpp`'s `totgRobotTrajectoryCase`).
//!
//! All three fixture cases go through `compute_time_stamps_with_limits`
//! (the explicit-limits overload) with the exact `panda_arm` per-joint
//! limits from upstream's `testCustomLimits` (already ported as a unit
//! test in `time_optimal_trajectory_generation.rs`). The scaling-only
//! overload (`compute_time_stamps`) is not exercised here: this port's
//! `moveit-model` URDF loader never sets `acceleration_bounded`, so that
//! overload fails validation against every fixture in this workspace
//! before it can reach any numeric comparison (see this round's report).
//! Custom limits bypass that gap entirely, matching upstream's own
//! `setAccelerationLimits` test workaround.
//!
//! Case 0 is the baseline (`testCustomLimits`'s own two waypoints,
//! oracle-side, as a sanity cross-check of the already-ported unit test).
//! Case 1 is identical except `max_velocity_scaling_factor: 2.0` and
//! `max_acceleration_scaling_factor: -1.0` (both outside `(0, 1]`) --
//! `verify_scaling_factor`/`verifyScalingFactor` must clamp both to the
//! default `1.0`, so this case's oracle output is bit-identical to case
//! 0's (confirmed when the fixture was captured; the oracle logged
//! `Invalid max_velocity_scaling_factor 2.000000 specified, defaulting to
//! 1.000000 instead.` and the equivalent for acceleration). Case 2
//! repeats waypoint 0 before waypoint 1 -- the duplicate must be dropped
//! by the `min_angle_change` dedup pass, producing the same waypoint
//! count and durations as case 0/1 (matching this crate's own
//! `a_middle_duplicate_waypoint_is_dropped_not_double_counted` unit
//! test).
//!
//! # Tolerance
//!
//! Both sides run the identical Kunz & Stilman numerics this crate's
//! `totg_parity.rs` already validates bit-for-bit against the core-only
//! oracle path; `TOL` matches that file's.

use std::collections::HashMap;
use std::fs;

use approx::assert_relative_eq;
use serde::Deserialize;

use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;
use moveit_trajectory::RobotTrajectory;
use moveit_trajectory::time_optimal_trajectory_generation::{
    TotgOptions, compute_time_stamps_with_limits,
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

#[derive(Deserialize)]
struct TotgRtRequestCase {
    waypoints: Vec<HashMap<String, f64>>,
    durations_from_previous: Vec<f64>,
    #[serde(default)]
    max_velocity_scaling_factor: Option<f64>,
    #[serde(default)]
    max_acceleration_scaling_factor: Option<f64>,
    velocity_limits: HashMap<String, f64>,
    #[serde(default)]
    acceleration_limits: HashMap<String, f64>,
}

#[derive(Deserialize)]
struct TotgRtRequest {
    group: String,
    cases: Vec<TotgRtRequestCase>,
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
fn totg_robot_trajectory_matches_the_oracle() {
    let model = panda();

    let requests: Vec<TotgRtRequest> =
        serde_json::from_str(&read_fixture("totg_robot_trajectory_request.json"))
            .expect("parse totg_robot_trajectory_request.json");
    let responses: Vec<TotgRtResponseEntry> =
        serde_json::from_str(&read_fixture("totg_robot_trajectory_response.json"))
            .expect("parse totg_robot_trajectory_response.json");
    assert_eq!(requests.len(), responses.len());
    let request = &requests[0];
    let response = &responses[0];
    assert_eq!(request.cases.len(), response.result.cases.len());

    for (case_index, (case, expected)) in
        request.cases.iter().zip(&response.result.cases).enumerate()
    {
        let mut trajectory = RobotTrajectory::for_group_name(&model, &request.group)
            .unwrap_or_else(|e| panic!("case {case_index}: for_group_name: {e}"));

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

        let options = TotgOptions {
            max_velocity_scaling_factor: case.max_velocity_scaling_factor.unwrap_or(1.0),
            max_acceleration_scaling_factor: case.max_acceleration_scaling_factor.unwrap_or(1.0),
            ..Default::default()
        };

        let result = compute_time_stamps_with_limits(
            &mut trajectory,
            &case.velocity_limits,
            &case.acceleration_limits,
            &options,
        );

        assert_eq!(
            result.is_ok(),
            expected.ok,
            "case {case_index}: compute_time_stamps_with_limits ok mismatch ({result:?})"
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
