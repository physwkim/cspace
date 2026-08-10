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
//! test in `time_optimal_trajectory_generation.rs`), matching upstream's
//! own `setAccelerationLimits` test workaround for a URDF with no
//! acceleration field. The scaling-only overload (`compute_time_stamps`)
//! is not exercised here — see
//! `totg_robot_trajectory_scaling_only_parity.rs`, a separate fixture and
//! test file, and `time_optimal_trajectory_generation.rs`'s "Closed gap"
//! doc section for why it needed its own model setup rather than a fourth
//! case here.
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
//! Every case also cross-checks `has_mixed_joint_types` against
//! `oracle.cpp`'s `hasMixedJointTypesForGroup` (a re-implementation of
//! the private `TimeOptimalTrajectoryGeneration::hasMixedJointTypes`,
//! cpp:1273-1288, over the same public `getActiveJointModels()` API --
//! see that function's doc comment for why a re-implementation was
//! necessary rather than a direct call). `panda_arm` is revolute-only, so
//! every case here asserts `false`; `totg_synthetic_parity.rs` is where
//! this comes back `true`.
//!
//! # Tolerance
//!
//! `assert_relative_eq!` without an explicit `max_relative` silently gets
//! `f64::EPSILON` (~2.22e-16), so bisecting `epsilon` alone can plateau
//! before a real biting point (PORTING-PLAN.md §78.1/§79) -- and it did
//! here: the old unqualified `TOL` passed all the way to `0.0`. Every call
//! below now pins `max_relative = TOL`, coupling both branches to the same
//! constant. A whole-test bisection found `2.5e-16` and above pass, `2e-16`
//! and below fail, first stopping (fail-fast, at the earliest failing
//! assertion in loop order) on a `positions` comparison, e.g.
//! `-1.0781250000000013` vs `-1.078125000000001`, diff ~2.3e-16 -- an
//! earlier draft of this doc mislabeled that pair as
//! `duration_from_previous` from the magnitude alone; it is a joint
//! position (durations in this fixture are all < 1s, never ~1.08).
//!
//! All four comparison groups (`duration_from_previous`, `positions`,
//! `velocities`, `accelerations`) share this one `TOL`, so a bisection that
//! only watches the first fail-fast failure can report the wrong group as
//! tightest and never notice a looser (or tighter) one hiding behind it
//! (PORTING-PLAN.md's correction to §79's method, citing
//! `distance-field/tests/upstream_parity.rs`: 4 of 7 bundled assertions
//! there only bit 12 orders below the named epsilon once re-bisected per
//! group). Re-verified per group with a non-panicking max-diff sweep
//! (temporarily printing every group's largest `|actual - expected|`
//! across every case/waypoint/joint instead of asserting, so fail-fast
//! can't hide one group's true floor behind another's): `duration_from_previous`
//! maxes at `1.39e-17`, `positions` at `4.44e-16` (case 0, waypoint 5,
//! `panda_joint2` -- the true tightest group, not `duration_from_previous`
//! as the whole-test bisection's first failure suggested), `velocities` at
//! `2.78e-17`, `accelerations` at `2.22e-16` (`= f64::EPSILON` exactly) --
//! all four groups are genuinely nonzero, so none collapses to
//! `assert_eq!`. `TOL` is `1e-12`, ~3.35 orders of magnitude of headroom
//! over the loosest group (`positions`). Confirmed still discriminating:
//! multiplying `do_time_parameterization_calculations`'s
//! `position[j]` writeback by `1.0001` fails the fixture.

use std::collections::HashMap;
use std::fs;

use approx::assert_relative_eq;
use serde::Deserialize;

use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;
use cspace_core::trajectory::RobotTrajectory;
use cspace_core::trajectory::time_optimal_trajectory_generation::{
    TotgOptions, compute_time_stamps_with_limits, has_mixed_joint_types,
};

const TOL: f64 = 1e-12;

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

        // Not struct-update syntax: `TotgOptions::resample_dt` is
        // `pub(crate)` (see its doc comment), and functional update syntax
        // requires every field to be nameable from the call site even when
        // not explicitly listed — which an integration test, compiled as
        // its own external crate, cannot do. Direct assignment to these two
        // still-`pub` fields sidesteps that.
        let mut options = TotgOptions::default();
        options.max_velocity_scaling_factor = case.max_velocity_scaling_factor.unwrap_or(1.0);
        options.max_acceleration_scaling_factor =
            case.max_acceleration_scaling_factor.unwrap_or(1.0);

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
                epsilon = TOL,
                max_relative = TOL
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
                    epsilon = TOL,
                    max_relative = TOL
                );
            }
            for (name, &expected_velocity) in &expected_waypoint.velocities {
                assert_relative_eq!(
                    waypoint.variable_velocity(name).unwrap_or_else(|e| panic!(
                        "case {case_index}: waypoint {waypoint_idx}: variable_velocity({name}): {e}"
                    )),
                    expected_velocity,
                    epsilon = TOL,
                    max_relative = TOL
                );
            }
            for (name, &expected_acceleration) in &expected_waypoint.accelerations {
                assert_relative_eq!(
                    waypoint.variable_acceleration(name).unwrap_or_else(|e| panic!(
                        "case {case_index}: waypoint {waypoint_idx}: variable_acceleration({name}): {e}"
                    )),
                    expected_acceleration,
                    epsilon = TOL,
                max_relative = TOL
                );
            }
        }
    }
}
