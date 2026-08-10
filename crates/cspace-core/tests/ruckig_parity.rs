// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `ruckig` op, covering
//! `ruckig_smoothing.rs`.
//!
//! `test_ruckig_traj_smoothing.cpp`'s four cases are already ported as
//! direct unit tests in `tests/ruckig_smoothing.rs` (they only assert
//! success/duration-bounds, which upstream's own test does too). This file
//! is the numeric cross-check those tests cannot be: it runs the same
//! `panda_arm` scenarios through both `apply_smoothing`/
//! `apply_smoothing_with_limits` and the real `trajectory_processing::
//! RuckigSmoothing::applySmoothing` (via the oracle's `ruckig` op) and
//! compares every waypoint's position/velocity/acceleration and every
//! `duration_from_previous`, not just pass/fail.
//!
//! The fixture's 7 cases: `basic_trajectory` and `trajectory_duration`
//! (upstream scenarios, scaling-factor-only overload), a custom-limits case
//! (`apply_smoothing_with_limits`, matching `basic_trajectory_with_custom_
//! limits`), `single_waypoint` and an empty trajectory (both the `num_
//! waypoints < 2` no-op path), a `mitigate_overshoot: true` case, and a
//! duplicate-consecutive-waypoints case.
//!
//! # Exactness, not tolerance
//!
//! This file used `assert_relative_eq!` with an unspecified `max_relative`
//! (silently `f64::EPSILON`, ~2.22e-16) until PORTING-PLAN.md §78.1/§79
//! found that trap workspace-wide: bisecting `epsilon` alone can never
//! reach a real biting point once the implicit relative branch covers the
//! diff on its own. Here it does more than cover it -- pinning
//! `epsilon = max_relative = 0.0` and printing every non-bit-identical pair
//! (not just asserting) showed **zero** diffs across all 7 cases' waypoints:
//! `run_ruckig` never rewrites a waypoint's position (only
//! `duration_from_previous`, and -- on the duration-extension path only --
//! velocity/acceleration, which this fixture's inputs start at `0.0` and
//! `0.0 / duration_extension_factor` stays `0.0`), so `positions`/
//! `velocities`/`accelerations` are literal echoes of what the test set up,
//! not values this port computed. `duration_from_previous` *is* computed
//! (by `rsruckig`'s `Ruckig::calculate`/`get_duration`) and still came back
//! bit-identical to the oracle's `ruckig` for every case -- for these
//! non-degenerate single-segment trapezoidal profiles both independent
//! implementations evidently walk the same floating-point operations in the
//! same order. All four fields are therefore `assert_eq!`, not
//! `assert_relative_eq!`: this is measured exactness on the current
//! fixture, not an a priori claim that `rsruckig` always bit-matches
//! `ruckig` (see `ruckig_filter_parity.rs`'s streaming fixture, whose
//! multi-tick cases do diverge at the ULP level). Confirmed still
//! discriminating: multiplying `run_ruckig`'s
//! `ruckig_output.get_duration()` writeback by `1.0001` makes case 0's
//! `duration_from_previous` assertion fail.

use std::collections::HashMap;
use std::fs;

use serde::Deserialize;

use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;
use cspace_core::trajectory::RobotTrajectory;
use cspace_core::trajectory::ruckig_smoothing::{
    SmoothingOptions, apply_smoothing, apply_smoothing_with_limits,
};

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/trajectory/{}"),
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
struct RuckigRequestCase {
    waypoints: Vec<HashMap<String, f64>>,
    durations_from_previous: Vec<f64>,
    #[serde(default)]
    max_velocity_scaling_factor: Option<f64>,
    #[serde(default)]
    max_acceleration_scaling_factor: Option<f64>,
    #[serde(default)]
    mitigate_overshoot: Option<bool>,
    #[serde(default)]
    overshoot_threshold: Option<f64>,
    #[serde(default)]
    velocity_limits: Option<HashMap<String, f64>>,
    #[serde(default)]
    acceleration_limits: Option<HashMap<String, f64>>,
    #[serde(default)]
    jerk_limits: Option<HashMap<String, f64>>,
}

#[derive(Deserialize)]
struct RuckigRequest {
    group: String,
    cases: Vec<RuckigRequestCase>,
}

#[derive(Deserialize)]
struct RuckigResultWaypoint {
    positions: HashMap<String, f64>,
    velocities: HashMap<String, f64>,
    accelerations: HashMap<String, f64>,
}

#[derive(Deserialize)]
struct RuckigResultCase {
    ok: bool,
    #[serde(default)]
    durations_from_previous: Vec<f64>,
    #[serde(default)]
    waypoints: Vec<RuckigResultWaypoint>,
}

#[derive(Deserialize)]
struct RuckigResult {
    cases: Vec<RuckigResultCase>,
}

#[derive(Deserialize)]
struct RuckigResponseEntry {
    result: RuckigResult,
}

#[test]
fn ruckig_smoothing_matches_the_oracle() {
    let model = panda();

    let requests: Vec<RuckigRequest> = serde_json::from_str(&read_fixture("ruckig_request.json"))
        .expect("parse ruckig_request.json");
    let responses: Vec<RuckigResponseEntry> =
        serde_json::from_str(&read_fixture("ruckig_response.json"))
            .expect("parse ruckig_response.json");
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

        let options = SmoothingOptions {
            max_velocity_scaling_factor: case.max_velocity_scaling_factor.unwrap_or(1.0),
            max_acceleration_scaling_factor: case.max_acceleration_scaling_factor.unwrap_or(1.0),
            mitigate_overshoot: case.mitigate_overshoot.unwrap_or(false),
            overshoot_threshold: case.overshoot_threshold.unwrap_or(0.01),
        };

        let result = match &case.velocity_limits {
            Some(velocity_limits) => apply_smoothing_with_limits(
                &mut trajectory,
                velocity_limits,
                case.acceleration_limits.as_ref().unwrap_or(&HashMap::new()),
                case.jerk_limits.as_ref().unwrap_or(&HashMap::new()),
                &options,
            ),
            None => apply_smoothing(&mut trajectory, &options),
        };

        assert_eq!(
            result.is_ok(),
            expected.ok,
            "case {case_index}: apply_smoothing ok mismatch ({result:?})"
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
            assert_eq!(
                trajectory.way_point_duration_from_previous(waypoint_idx),
                expected.durations_from_previous[waypoint_idx],
                "case {case_index} waypoint {waypoint_idx}: duration_from_previous"
            );

            let waypoint = trajectory
                .way_point(waypoint_idx)
                .unwrap_or_else(|e| panic!("case {case_index}: way_point({waypoint_idx}): {e}"));
            let expected_waypoint = &expected.waypoints[waypoint_idx];

            for (name, &expected_position) in &expected_waypoint.positions {
                assert_eq!(
                    waypoint.variable_position(name).unwrap_or_else(|e| panic!(
                        "case {case_index}: waypoint {waypoint_idx}: variable_position({name}): {e}"
                    )),
                    expected_position,
                    "case {case_index} waypoint {waypoint_idx}: position {name}"
                );
            }
            for (name, &expected_velocity) in &expected_waypoint.velocities {
                assert_eq!(
                    waypoint.variable_velocity(name).unwrap_or_else(|e| panic!(
                        "case {case_index}: waypoint {waypoint_idx}: variable_velocity({name}): {e}"
                    )),
                    expected_velocity,
                    "case {case_index} waypoint {waypoint_idx}: velocity {name}"
                );
            }
            for (name, &expected_acceleration) in &expected_waypoint.accelerations {
                assert_eq!(
                    waypoint.variable_acceleration(name).unwrap_or_else(|e| panic!(
                        "case {case_index}: waypoint {waypoint_idx}: variable_acceleration({name}): {e}"
                    )),
                    expected_acceleration,
                    "case {case_index} waypoint {waypoint_idx}: acceleration {name}"
                );
            }
        }
    }
}
