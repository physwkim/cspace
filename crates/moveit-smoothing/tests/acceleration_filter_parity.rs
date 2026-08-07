// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `acceleration_filter` op,
//! covering `acceleration_filter.rs`.
//!
//! The fixture's 6 cases are the boundary set `acceleration_filter.rs`'s own
//! unit tests already hand-derive independently: unconstrained (no bound
//! binds, two chained steps), a single bound binding alone on two different
//! joint indices, the interval intersection collapsing to a single point,
//! the interval intersection going empty (fallback branch), and a
//! below-`COMMAND_DIFFERENCE_THRESHOLD` hold. This file is the numeric
//! cross-check those hand-derivations cannot be on their own: it runs the
//! same six cases through both `AccelerationLimitedFilter::do_smoothing` and
//! the real `online_signal_smoothing::AccelerationLimitedPlugin::doSmoothing`
//! (via the oracle's `acceleration_filter` op, which loads the real plugin
//! through `pluginlib` and solves the real QP through `osqp` — see that op's
//! own comment in `tools/moveit-oracle/src/oracle.cpp`).
//!
//! # Tolerance
//!
//! Every case except the third uses `TOL`, matching `osqp`'s exact
//! convergence away from a degenerate constraint. The third case
//! deliberately gives one joint a zero-width acceleration interval, which
//! forces this port's exact closed-form optimum to `alpha == 1.0` while
//! `osqp`'s own default `eps_abs`/`eps_rel` stop its iterative solve only
//! *close to* that point — see `acceleration_filter.rs`'s module doc for the
//! full derivation and the measured magnitude `DEGENERATE_CASE_TOL` is set
//! from.
//!
//! Both `assert_relative_eq!` calls now pin `max_relative = tol`: left
//! unspecified it silently defaults to `f64::EPSILON` (~2.22e-16)
//! regardless of `epsilon`, so bisecting `epsilon` alone can plateau before
//! a real biting point once `max(|a|, |b|)` is large enough for that
//! implicit relative branch to cover the diff on its own (PORTING-PLAN.md
//! §78.1/§79). Bisecting the coupled `TOL`: `1.2e-15` and above pass,
//! `1.1e-15` and below fail (first divergence is `positions[idx]` in case 0,
//! `0.9999999999999998` vs `1.0000000000000009`, diff ~1.1e-15 — this was
//! never actually masked by the implicit relative branch, which for
//! `max(|a|,|b|) ≈ 1.0` would only cover ~2.22e-16). `TOL` is `1e-11`, ~4
//! orders of magnitude of headroom over that floor.
//!
//! `positions` and `velocities` share this one `tol`, so a bisection that
//! only watches the first `assert_relative_eq!` failure could report the
//! loosest group's floor and never notice a tighter group behind it
//! (PORTING-PLAN.md's correction to §79's method, citing
//! `distance-field/tests/upstream_parity.rs`: 4 of 7 bundled assertions
//! there only bit 12 orders below the named epsilon once re-bisected per
//! group). Re-verified with a non-panicking max-diff sweep across every
//! case/step/joint: `positions` and `velocities` both max at exactly
//! `1.11e-15` for the non-degenerate cases (case 0's `positions[idx]`
//! error propagates straight into `velocities` through `do_smoothing`'s
//! `(p - last_p) / dt`), and both max at exactly `8.29e-4` for case 2 --
//! consistent with a single shared floor per group of cases, not one group
//! masking a tighter other.
//!
//! `DEGENERATE_CASE_TOL` was independently confirmed to still bite at its
//! documented measured value (fails at `8e-4`, passes at `2e-3`) — it was
//! never at risk of the `max_relative` trap since it is derived from an
//! actually-observed diff, not guessed. Perturbing the `alpha`-blend
//! writeback in
//! `AccelerationLimitedFilter::do_smoothing` (`*p = alpha * last_p + (1.0 -
//! alpha) * *p`) by a `1.0001` factor makes this fixture fail, confirming
//! the assertions still discriminate.
//!
//! ## Round 14: re-measured under `float_roundtrip`, unchanged
//!
//! The standalone (non-workspace, non-`float_roundtrip`) checker built for
//! item 1 of this round found `0` of `109` literals in
//! `acceleration_filter_request.json` and `0` of `98` in
//! `acceleration_filter_response.json` misparsed by the old default
//! `serde_json` parser -- unlike `moveit-trajectory`'s and
//! `ruckig_filter_response.json`'s fixtures, this pair has no corrupted
//! literal to begin with, so the fix could not have moved either floor. The
//! non-panicking max-diff sweep confirms this directly: re-run under the
//! now-fixed workspace parser, the non-degenerate group still maxes at
//! `1.11e-15` (case 0 `positions[panda_joint1]`,
//! `0.9999999999999998`/`1.0000000000000009`, bit-identical to the figures
//! above) and the degenerate case still maxes at `8.29e-4` (case 2
//! `positions[panda_joint1]`, `0`/`0.0008294991991130152`, likewise
//! bit-identical). The `1.1e-15`-fails/`1.2e-15`-passes bisection boundary
//! for `TOL` was re-run against the fixed parser and reproduced exactly.

use std::collections::HashMap;
use std::fs;

use approx::assert_relative_eq;
use serde::Deserialize;

use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_smoothing::acceleration_filter::{AccelerationLimitedFilter, joint_acceleration_bounds};
use moveit_srdf::SrdfModel;

const TOL: f64 = 1e-11;

/// Measured, not guessed: `acceleration_filter_response.json`'s third case
/// puts `osqp`'s answer `0.0008294991991130152` away from this port's exact
/// `0.0` — see `acceleration_filter.rs`'s module doc.
const DEGENERATE_CASE_TOL: f64 = 2e-3;

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

fn set_acceleration_bounds(model: &mut RobotModel, bounds: &HashMap<String, f64>) {
    for (name, &max_acceleration) in bounds {
        let joint = model
            .joint_model_mut(name)
            .unwrap_or_else(|e| panic!("joint_model_mut({name}): {e}"));
        let mut limits = joint.variable_bounds_msg();
        for limit in &mut limits {
            limit.has_acceleration_limits = true;
            limit.max_acceleration = max_acceleration;
        }
        joint.set_variable_bounds_from_limits(&limits);
    }
}

#[derive(Deserialize)]
struct Command {
    positions: HashMap<String, f64>,
    velocities: HashMap<String, f64>,
}

#[derive(Deserialize)]
struct Reset {
    positions: HashMap<String, f64>,
    velocities: HashMap<String, f64>,
}

#[derive(Deserialize)]
struct RequestCase {
    group: String,
    update_period: f64,
    acceleration_bounds: HashMap<String, f64>,
    reset: Reset,
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
}

#[derive(Deserialize)]
struct ResultCase {
    initialize_ok: bool,
    reset_ok: bool,
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
fn acceleration_filter_matches_the_oracle() {
    let requests: Vec<Request> =
        serde_json::from_str(&read_fixture("acceleration_filter_request.json"))
            .expect("parse acceleration_filter_request.json");
    let responses: Vec<ResponseEntry> =
        serde_json::from_str(&read_fixture("acceleration_filter_response.json"))
            .expect("parse acceleration_filter_response.json");
    assert_eq!(requests.len(), responses.len());
    let request = &requests[0];
    let response = &responses[0];
    assert_eq!(request.cases.len(), response.result.cases.len());

    for (case_index, (case, expected)) in
        request.cases.iter().zip(&response.result.cases).enumerate()
    {
        let mut model = panda();
        set_acceleration_bounds(&mut model, &case.acceleration_bounds);
        let group = model
            .joint_model_group(&case.group)
            .unwrap_or_else(|e| panic!("case {case_index}: joint_model_group: {e}"));
        let (min_acceleration_limits, max_acceleration_limits) =
            joint_acceleration_bounds(&model, group)
                .unwrap_or_else(|e| panic!("case {case_index}: joint_acceleration_bounds: {e}"));
        let joint_names = group.active_joint_names().to_vec();

        assert!(
            expected.initialize_ok,
            "case {case_index}: expected initialize_ok"
        );
        let mut filter = AccelerationLimitedFilter::new(
            &min_acceleration_limits,
            &max_acceleration_limits,
            case.update_period,
        )
        .unwrap_or_else(|e| panic!("case {case_index}: AccelerationLimitedFilter::new: {e}"));

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

        let reset_positions = named(&case.reset.positions);
        let reset_velocities = named(&case.reset.velocities);
        let reset_result = filter.reset(&reset_positions, &reset_velocities);
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
            let mut velocities = named(&command.velocities);
            let result = filter.do_smoothing(&mut positions, &mut velocities);
            assert_eq!(
                result.is_ok(),
                expected_step.ok,
                "case {case_index} step {step_index}: do_smoothing ok mismatch ({result:?})"
            );
            if !expected_step.ok {
                continue;
            }

            let tol = if case_index == 2 {
                DEGENERATE_CASE_TOL
            } else {
                TOL
            };
            for (name, &expected_position) in &expected_step.positions {
                let idx = joint_names
                    .iter()
                    .position(|n| n == name)
                    .unwrap_or_else(|| {
                        panic!("case {case_index} step {step_index}: unknown joint {name}")
                    });
                assert_relative_eq!(
                    positions[idx],
                    expected_position,
                    epsilon = tol,
                    max_relative = tol
                );
            }
            for (name, &expected_velocity) in &expected_step.velocities {
                let idx = joint_names
                    .iter()
                    .position(|n| n == name)
                    .unwrap_or_else(|| {
                        panic!("case {case_index} step {step_index}: unknown joint {name}")
                    });
                assert_relative_eq!(
                    velocities[idx],
                    expected_velocity,
                    epsilon = tol,
                    max_relative = tol
                );
            }
        }
    }
}
