// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `ruckig_filter` op,
//! covering `ruckig_filter.rs`. Closes the asymmetry
//! `ruckig_filter.rs`'s module doc used to describe: unlike
//! `acceleration_filter.rs`, this port previously had no oracle op wrapping
//! `online_signal_smoothing::RuckigFilterPlugin` (real `rsruckig::Ruckig::
//! update` streaming path), only unit tests checking bound compliance and
//! qualitative direction.
//!
//! The fixture's 6 cases: an `initialize` failure (one active joint given a
//! `velocity_bounds`/`acceleration_bounds` but no `jerk_bounds` entry), a
//! 5-tick streaming trajectory from rest toward a fixed target (exercising
//! `doSmoothing`'s internal `pass_to_input` state threading across calls), a
//! single tick already at the target (zero displacement), a 25-tick
//! streaming trajectory toward a fixed target long enough to reach and
//! settle at it, a 15-tick streaming trajectory toward a continuously
//! moving target (the commanded position changes every tick), and a single
//! tick with `panda_joint1`'s `jerk_bounds` set to `0.0` (triggering a
//! `rsruckig::RuckigResult` outside `{Working, Finished,
//! ErrorSynchronizationCalculation}`).
//!
//! # What each case discriminates
//!
//! Verified by deletion/perturbation testing (temporarily breaking one
//! computation in `RuckigFilter::do_smoothing`/`reset`, confirming this
//! fixture then fails, and reverting):
//!
//! - `pass_to_input` under `have_initial_output`, and both `do_smoothing`'s
//!   and `reset`'s three-line output writebacks, are all already killed by
//!   the original 3-case fixture (case 1, the 5-tick streaming case).
//! - The `target_velocity` extrapolation
//!   (`target_velocity[i] = current_velocity[i] + current_acceleration[i] *
//!   delta_time`) was **not** killed by the original 3-case fixture at any
//!   perturbation (`×1.000001`, `×2.0`, `×0.0`, or `= 0.0`) — case 1's only
//!   multi-step case never left the opening jerk ramp. The fixed-target
//!   25-tick case (index 3) and the moving-target 15-tick case (index 4)
//!   both now kill `target_velocity = 0.0`: index 3 first diverges at step
//!   5 (`panda_joint1` position off by ~4.7e-7), index 4 at step 4 (~4.7e-8),
//!   both growing every subsequent step as the wrong extrapolated velocity
//!   compounds.
//! - The `RuckigResult` early-return branch (`if !matches!(result, Working |
//!   Finished | ErrorSynchronizationCalculation) { return Ok(()); }`) was
//!   unreachable through the original 3-case fixture — none of those cases
//!   ever produce a `RuckigResult` outside that set. The zero-jerk-bound
//!   case (index 5) does: `getVelAccelJerkBounds`/[`joint_vel_accel_jerk_bounds`]
//!   only checks that a bound is *present*, not that it is positive, so a
//!   `jerk_bounds` entry of `0.0` on a joint with a real displacement to
//!   cover reaches `Ruckig::update` and returns an error variant outside the
//!   allowed set. Deleting the branch (always falling through to the
//!   writeback loop) makes `positions[0]` come back `0.0` (the never-written
//!   `OutputParameter::new` default) instead of the fixture's expected
//!   `0.3` (the commanded target, left untouched by the early return).
//!
//! # Tolerance
//!
//! `rsruckig` is an independent Rust reimplementation of the same published
//! algorithm as upstream's C++ `ruckig`, not a binding to it -- its
//! block/step-2 root-finding does not walk identical floating-point
//! operations in identical order, so exact bit-parity is not expected.
//! `TOL` is set from what this fixture actually produces, matching
//! `ruckig_parity.rs`'s own precedent for the same crate pairing.

use std::collections::HashMap;
use std::fs;

use approx::assert_relative_eq;
use serde::Deserialize;

use moveit_model::joint::JointLimits;
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_smoothing::ruckig_filter::{RuckigFilter, joint_vel_accel_jerk_bounds};
use moveit_srdf::SrdfModel;

const TOL: f64 = 1e-9;

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
    let urdf_path = fixture_path("panda.urdf");
    let srdf_path = fixture_path("panda.srdf");
    let urdf_xml =
        fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_file(&urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(&srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

fn set_bound(
    model: &mut RobotModel,
    bounds: &HashMap<String, f64>,
    apply: impl Fn(&mut JointLimits, f64),
) {
    for (name, &value) in bounds {
        let joint = model
            .joint_model_mut(name)
            .unwrap_or_else(|e| panic!("joint_model_mut({name}): {e}"));
        let mut limits = joint.variable_bounds_msg();
        for limit in &mut limits {
            apply(limit, value);
        }
        joint.set_variable_bounds_from_limits(&limits);
    }
}

#[derive(Deserialize)]
struct Command {
    positions: HashMap<String, f64>,
}

#[derive(Deserialize)]
struct Reset {
    positions: HashMap<String, f64>,
    velocities: HashMap<String, f64>,
    accelerations: HashMap<String, f64>,
}

#[derive(Deserialize)]
struct RequestCase {
    group: String,
    update_period: f64,
    #[serde(default)]
    velocity_bounds: HashMap<String, f64>,
    #[serde(default)]
    acceleration_bounds: HashMap<String, f64>,
    #[serde(default)]
    jerk_bounds: HashMap<String, f64>,
    #[serde(default)]
    reset: Option<Reset>,
    #[serde(default)]
    commands: Vec<Command>,
}

#[derive(Deserialize)]
struct Request {
    cases: Vec<RequestCase>,
}

#[derive(Deserialize)]
struct Step {
    ok: bool,
    positions: HashMap<String, f64>,
    velocities: HashMap<String, f64>,
    accelerations: HashMap<String, f64>,
}

#[derive(Deserialize)]
struct ResultCase {
    initialize_ok: bool,
    #[serde(default)]
    reset_ok: bool,
    #[serde(default)]
    steps: Vec<Step>,
}

#[derive(Deserialize)]
struct RequestResult {
    cases: Vec<ResultCase>,
}

#[derive(Deserialize)]
struct ResponseEntry {
    result: RequestResult,
}

#[test]
fn ruckig_filter_matches_the_oracle() {
    let requests: Vec<Request> = serde_json::from_str(&read_fixture("ruckig_filter_request.json"))
        .expect("parse ruckig_filter_request.json");
    let responses: Vec<ResponseEntry> =
        serde_json::from_str(&read_fixture("ruckig_filter_response.json"))
            .expect("parse ruckig_filter_response.json");
    assert_eq!(requests.len(), responses.len());
    let request = &requests[0];
    let response = &responses[0];
    assert_eq!(request.cases.len(), response.result.cases.len());

    for (case_index, (case, expected)) in
        request.cases.iter().zip(&response.result.cases).enumerate()
    {
        let mut model = panda();
        set_bound(&mut model, &case.velocity_bounds, |l, v| {
            l.has_velocity_limits = true;
            l.max_velocity = v;
        });
        set_bound(&mut model, &case.acceleration_bounds, |l, v| {
            l.has_acceleration_limits = true;
            l.max_acceleration = v;
        });
        set_bound(&mut model, &case.jerk_bounds, |l, v| {
            l.has_jerk_limits = true;
            l.max_jerk = v;
        });
        let group = model
            .joint_model_group(&case.group)
            .unwrap_or_else(|e| panic!("case {case_index}: joint_model_group: {e}"));
        let bounds = joint_vel_accel_jerk_bounds(&model, group);

        let Ok((velocity_bounds, acceleration_bounds, jerk_bounds)) = bounds else {
            assert!(
                !expected.initialize_ok,
                "case {case_index}: expected initialize_ok"
            );
            continue;
        };
        assert!(
            expected.initialize_ok,
            "case {case_index}: expected initialize to fail"
        );

        let joint_names = group.active_joint_names().to_vec();
        let mut filter = RuckigFilter::new(
            &velocity_bounds,
            &acceleration_bounds,
            &jerk_bounds,
            case.update_period,
        );

        let named = |values: &HashMap<String, f64>| -> Vec<f64> {
            joint_names
                .iter()
                .map(|name| {
                    *values
                        .get(name)
                        .unwrap_or_else(|| panic!("case {case_index}: missing {name}"))
                })
                .collect()
        };

        let reset = case
            .reset
            .as_ref()
            .unwrap_or_else(|| panic!("case {case_index}: missing reset"));
        let reset_positions = named(&reset.positions);
        let reset_velocities = named(&reset.velocities);
        let reset_accelerations = named(&reset.accelerations);
        let reset_result = filter.reset(&reset_positions, &reset_velocities, &reset_accelerations);
        assert_eq!(
            reset_result.is_ok(),
            expected.reset_ok,
            "case {case_index}: reset ok mismatch ({reset_result:?})"
        );

        assert_eq!(
            case.commands.len(),
            expected.steps.len(),
            "case {case_index}: step count mismatch"
        );

        for (step_index, (command, expected_step)) in
            case.commands.iter().zip(&expected.steps).enumerate()
        {
            let mut positions = named(&command.positions);
            let mut velocities = vec![0.0; joint_names.len()];
            let mut accelerations = vec![0.0; joint_names.len()];
            let result = filter.do_smoothing(&mut positions, &mut velocities, &mut accelerations);
            assert_eq!(
                result.is_ok(),
                expected_step.ok,
                "case {case_index} step {step_index}: do_smoothing ok mismatch ({result:?})"
            );
            if !expected_step.ok {
                continue;
            }

            for (idx, name) in joint_names.iter().enumerate() {
                assert_relative_eq!(positions[idx], expected_step.positions[name], epsilon = TOL);
                assert_relative_eq!(
                    velocities[idx],
                    expected_step.velocities[name],
                    epsilon = TOL
                );
                assert_relative_eq!(
                    accelerations[idx],
                    expected_step.accelerations[name],
                    epsilon = TOL
                );
            }
        }
    }
}
