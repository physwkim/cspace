// Copyright (c) 2026, cspace contributors
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
//!
//! `assert_relative_eq!` without an explicit `max_relative` silently gets
//! `f64::EPSILON` (~2.22e-16) for it, so bisecting `epsilon` alone can never
//! reach a real biting point once `max(|a|, |b|)` is large enough for that
//! implicit relative branch to cover the diff on its own (PORTING-PLAN.md
//! §78.1/§79). Every call below pins `max_relative = TOL` so the same
//! constant gates both branches. Bisecting that coupled constant on this
//! fixture: `1e-15` through `2.3e-16` pass, `2.2e-16` and below fail (the
//! first divergence is `accelerations[idx]` in the 15-tick moving-target
//! case, e.g. `0.28633902198848005` vs `0.2863390219884798`, diff ~2.5e-16 --
//! larger than the ~6.4e-17 the implicit relative branch alone would have
//! given it, so this assertion was never actually masked, just far looser
//! than it needed to be).
//!
//! `positions`/`velocities`/`accelerations` share this one `TOL`, so a
//! bisection that only watches the first `assert_relative_eq!` failure
//! could report the loosest group's floor and never notice a tighter group
//! hiding behind it (PORTING-PLAN.md's correction to §79's method, citing
//! `distance-field/tests/upstream_parity.rs`: 4 of 7 bundled assertions
//! there only bit 12 orders below the named epsilon once re-bisected per
//! group). Re-verified with a non-panicking max-diff sweep across every
//! case/step/joint instead of asserting: `positions` maxes at `5.55e-17`,
//! `velocities` at `5.55e-17`, `accelerations` at `2.22e-16` -- the
//! `accelerations` group above (`~2.5e-16`) genuinely is the tightest of
//! the three, so the first-failure bisection was not masking a tighter
//! group.
//!
//! `TOL` is `1e-12`, giving ~3.6 orders of magnitude of headroom over the
//! measured floor -- matching the headroom `distance-field` settled on in
//! PORTING-PLAN.md §78.2 for the same independent-reimplementation
//! situation. That the diffs bottom out at the f64 ULP floor rather than
//! growing with trajectory length is itself evidence `rsruckig`'s
//! root-finding agrees with the oracle's `ruckig` to within rounding, not
//! that this assertion has lost discriminating power; the
//! deletion/perturbation testing in "What each case discriminates" above is
//! what establishes that it still does.
//!
//! ## Round 14: `velocities`' `1.11e-16` figure above was a parser artifact
//!
//! `ruckig_filter_response.json` has 14 of 987 float literals the
//! pre-`float_roundtrip` `serde_json` default parser misparsed by 1 ULP
//! (found with the same standalone, non-workspace checker item 1 of this
//! round used against `cspace_core::trajectory`'s fixtures). Re-running the
//! max-diff sweep above under the now-fixed parser reproduces `positions`
//! (`5.55e-17`, case 4 step 13 `panda_joint1`) and `accelerations`
//! (`2.22e-16`, case 4 step 14 `panda_joint1`, values
//! `0.28633902198848005`/`0.2863390219884798` -- bit-identical to the
//! figures above) unchanged, but `velocities` drops to `5.55e-17` (case 4
//! step 9 `panda_joint1`), not the `1.11e-16` this section previously
//! reported. The old figure was traced to its source: the max-diff site
//! under the buggy parser was case 4 step 13's `velocities.panda_joint1`,
//! whose fixture literal `0.49700578537928997` the old default parser read
//! as `4.97005785379290022e-1` instead of the correct
//! `4.97005785379289966e-1`. This port's own computed value there,
//! `0.4970057853792899`, diffs from the *correct* expected value by
//! `5.551e-17` (matching the current true floor) but from the *buggy*
//! misparsed one by `1.110e-16` -- exactly the figure this section used to
//! cite. Fixing the parser did not change this port's arithmetic at all;
//! it only stopped corrupting the oracle value being compared against, so
//! the comparison at that site tightened to what it should have measured
//! all along. `positions` and `accelerations`' max-diff sites
//! (`0.31304484669850374` and `0.2863390219884798`) are not among the 14
//! corrupted literals, which is why re-measuring left them untouched.
//!
//! `TOL` is unaffected either way -- `1e-12` still has ~3.6 orders of
//! headroom over the loosest group (`accelerations`, unchanged at
//! `2.22e-16`). The coupled-constant bisection boundary this section
//! documents (`2.2e-16` fails, `2.3e-16` passes) was re-run against the
//! fixed parser and reproduced exactly: `2.2e-16` fails on the same
//! `accelerations[idx]` case (`0.28633902198848005` vs
//! `0.2863390219884798`), `2.3e-16` passes.

use std::collections::HashMap;
use std::fs;

use approx::assert_relative_eq;
use serde::Deserialize;

use cspace_core::model::joint::JointLimits;
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::smoothing::ruckig_filter::{RuckigFilter, joint_vel_accel_jerk_bounds};
use cspace_core::srdf::SrdfModel;

const TOL: f64 = 1e-12;

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/smoothing/{}"),
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
        )
        .unwrap_or_else(|e| panic!("case {case_index}: RuckigFilter::new: {e}"));

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
                assert_relative_eq!(
                    positions[idx],
                    expected_step.positions[name],
                    epsilon = TOL,
                    max_relative = TOL
                );
                assert_relative_eq!(
                    velocities[idx],
                    expected_step.velocities[name],
                    epsilon = TOL,
                    max_relative = TOL
                );
                assert_relative_eq!(
                    accelerations[idx],
                    expected_step.accelerations[name],
                    epsilon = TOL,
                    max_relative = TOL
                );
            }
        }
    }
}
