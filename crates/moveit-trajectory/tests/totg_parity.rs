// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `totg` op, core-only
//! branch (a request with no top-level `"group"` key), covering
//! [`moveit_trajectory::Path`]/[`moveit_trajectory::Trajectory`] (the
//! model-independent numeric core of `time_optimal_trajectory_generation.
//! hpp` lines 62-192) -- *not* the `TimeOptimalTrajectoryGeneration`
//! adapter class (header line 193 on) built on top of that core. The
//! adapter has its own parity fixture and test,
//! `totg_robot_trajectory_parity.rs`, against the same op's
//! group-driven branch.
//!
//! Fixture cases, one per invariant boundary:
//!
//! 1. Below the two-waypoint minimum -- `Path::create` returns `Err`
//!    (`stage: "path_create"` on the oracle side).
//! 2. Duplicate consecutive waypoints inside an otherwise ordinary path
//!    (`Path::create` succeeds; the duplicate collapses to a zero-length
//!    segment blended away like any other).
//! 3. A zero-length path (two identical waypoints) -- `Trajectory::create`
//!    still succeeds, but `duration`/velocity/acceleration are NaN (wire
//!    `null`) wherever upstream's own 0/0 divide lands on NaN, while
//!    position stays well-defined; see
//!    `a_zero_length_path_produces_a_nan_duration_trajectory` in
//!    `trajectory.rs`.
//! 4. A straight-line move that saturates `max_velocity`.
//! 5. `upstream_test2`'s ordinary multi-segment path (reused verbatim from
//!    `trajectory.rs`), the general case.
//!
//! Branches reached (recorded here rather than re-derived from the JSON):
//! case 1 exercises `Path::create`'s `waypoints.len() < 2` error path only.
//! Cases 2-5 all reach `Trajectory::create`'s `Ok` path; case 3 additionally
//! exercises the zero-length/NaN-propagation path documented above, and
//! case 4 exercises a velocity-limited (as opposed to purely
//! acceleration-limited) plateau. None of the five cases exercises
//! `Path::create`'s `max_deviation <= 0.0` error or its 180-degree-turn
//! error, or `Trajectory::create`'s `time_step <= 0.0` error -- those are
//! already covered as direct unit tests in `path.rs`/`trajectory.rs` and
//! need no oracle comparison since upstream rejects them before any
//! numerics run.
//!
//! # Tolerance
//!
//! Both sides run the same published algorithm (Kunz & Stilman) with the
//! same switching-point search, transcribed instruction-for-instruction
//! from upstream (see `trajectory.rs`'s module doc comment) -- unlike
//! `ruckig_parity.rs`'s independent reimplementation, this is close to a
//! line-for-line port, so tight agreement is expected. Comparisons here
//! don't go through `assert_relative_eq!` at all (this file's one grep hit
//! for that name, in `assert_matches_nullable`'s own doc comment below, is
//! prose, not an invocation) -- `assert_matches_nullable` is a hand-rolled
//! single-branch absolute `diff <= tol` check, so the implicit
//! `max_relative == f64::EPSILON` masking trap PORTING-PLAN.md §78.1/§79
//! describes cannot occur here structurally: there is no second branch to
//! silently take over.
//!
//! What *can* still happen with one shared constant across quantities of
//! different natural noise floors is the mirror problem: a constant sized
//! to clear the loosest floor is far looser than it needs to be for the
//! others, diluting their power to catch a real regression. Measured with
//! a non-panicking max-diff sweep (temporarily printing each quantity's
//! largest `|actual - expected|` across every case/sample/index instead of
//! asserting): `duration` maxes at `8.893039193935692e-9`, in case 5 --
//! re-measured directly in round 12 after two earlier drafts of this doc
//! disagreed with each other about which case (`4` vs `5`) and which bound
//! (`2e-9` vs `8.89e-9`); case 5 is correct and is now cross-checked
//! against `Trajectory::create`'s round-12 root-cause investigation in
//! `trajectory.rs`'s `upstream_test2` doc comment, which traces this exact
//! number to case 5 being the only one of the five that builds a
//! `CircularPathSegment` -- cases 1/3/4 (straight lines, or blends too
//! shallow to matter) are bit-exact (`0e0`) against the oracle. While
//! `position`/`velocity`/`acceleration` max at `2.27e-13`, `2.22e-16`,
//! `1.78e-15` respectively -- duration's floor is about 4 orders of
//! magnitude looser than the others', because it comes out of the
//! iterative switching-point/time-step integration (`trajectory.rs`'s
//! `Trajectory::create`) rather than a direct coordinate read. A single
//! `TOL` sized for duration would leave position/velocity/acceleration a
//! real regression could hide inside; a single `TOL` sized for them would
//! spuriously fail on duration's honest noise. Split: `TOL = 1e-9` covers
//! position/velocity/acceleration with ~3.6 orders of headroom over
//! `2.27e-13`; `DURATION_TOL = 2e-5` covers duration with ~3.35 orders of
//! headroom over `8.893e-9`. Both still
//! absolute, not relative -- several expected values (e.g. case 2's
//! terminal velocity `3.19e-17`) are legitimately near zero, where a
//! relative bound is meaningless, matching `ruckig_parity.rs`'s original
//! convention. Confirmed both constants still discriminate: multiplying
//! `Trajectory::duration`'s returned `time` by `1.0001` fails case 1's
//! duration comparison (diff `~3e-4`, far past `DURATION_TOL`), and
//! multiplying `Trajectory::position`'s returned config by `1.0000001`
//! fails case 1 sample 1's `position[0]` comparison (diff `~2.9e-8`, far
//! past `TOL`).

use std::fs;

use nalgebra::DVector;
use serde::Deserialize;

use moveit_trajectory::{Path, Trajectory};

const TOL: f64 = 1e-9;

/// `duration` comes out of `Trajectory::create`'s iterative switching-point
/// search, whose accumulated floating-point noise floor is measurably
/// looser than a direct position/velocity/acceleration coordinate read --
/// see this module's "Tolerance" doc section.
const DURATION_TOL: f64 = 2e-5;

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

#[derive(Deserialize)]
struct TotgRequestCase {
    waypoints: Vec<Vec<f64>>,
    max_deviation: f64,
    max_velocity: Vec<f64>,
    max_acceleration: Vec<f64>,
    time_step: f64,
    sample_times: Vec<f64>,
}

#[derive(Deserialize)]
struct TotgRequest {
    cases: Vec<TotgRequestCase>,
}

#[derive(Deserialize)]
struct TotgResultSample {
    time: f64,
    position: Vec<Option<f64>>,
    velocity: Vec<Option<f64>>,
    acceleration: Vec<Option<f64>>,
}

#[derive(Deserialize)]
struct TotgResultCase {
    ok: bool,
    #[serde(default)]
    stage: Option<String>,
    #[serde(default)]
    duration: Option<f64>,
    #[serde(default)]
    samples: Vec<TotgResultSample>,
}

#[derive(Deserialize)]
struct TotgResult {
    cases: Vec<TotgResultCase>,
}

#[derive(Deserialize)]
struct TotgResponseEntry {
    result: TotgResult,
}

/// Asserts `actual` matches `expected` -- `None` means the oracle's `null`,
/// i.e. the C++ side observed a NaN, so `actual` must be NaN too rather
/// than compared numerically (NaN vs NaN would fail `assert_relative_eq!`
/// even though it is exactly the agreement being asserted).
fn assert_matches_nullable(actual: f64, expected: Option<f64>, tol: f64, context: &str) {
    match expected {
        Some(expected) => {
            let diff = (actual - expected).abs();
            assert!(
                diff <= tol,
                "{context}: actual={actual}, expected={expected}, abs diff={diff}"
            );
        }
        None => assert!(actual.is_nan(), "{context}: expected NaN, got {actual}"),
    }
}

#[test]
fn totg_matches_the_oracle() {
    let requests: Vec<TotgRequest> =
        serde_json::from_str(&read_fixture("totg_request.json")).expect("parse totg_request.json");
    let responses: Vec<TotgResponseEntry> =
        serde_json::from_str(&read_fixture("totg_response.json"))
            .expect("parse totg_response.json");
    assert_eq!(requests.len(), responses.len());
    let request = &requests[0];
    let response = &responses[0];
    assert_eq!(request.cases.len(), response.result.cases.len());

    for (case_index, (case, expected)) in
        request.cases.iter().zip(&response.result.cases).enumerate()
    {
        let waypoints: Vec<DVector<f64>> = case
            .waypoints
            .iter()
            .map(|wp| DVector::from_vec(wp.clone()))
            .collect();
        let max_velocity = DVector::from_vec(case.max_velocity.clone());
        let max_acceleration = DVector::from_vec(case.max_acceleration.clone());

        let path_result = Path::create(&waypoints, case.max_deviation);

        let Ok(path) = path_result else {
            assert!(
                !expected.ok,
                "case {case_index}: Path::create failed unexpectedly"
            );
            assert_eq!(
                expected.stage.as_deref(),
                Some("path_create"),
                "case {case_index}: stage mismatch"
            );
            continue;
        };

        let trajectory_result =
            Trajectory::create(path, &max_velocity, &max_acceleration, case.time_step);

        let Ok(trajectory) = trajectory_result else {
            assert!(
                !expected.ok,
                "case {case_index}: Trajectory::create failed unexpectedly"
            );
            assert_eq!(
                expected.stage.as_deref(),
                Some("trajectory_create"),
                "case {case_index}: stage mismatch"
            );
            continue;
        };

        assert!(
            expected.ok,
            "case {case_index}: Trajectory::create succeeded but the oracle failed at {:?}",
            expected.stage
        );

        assert_matches_nullable(
            trajectory.duration(),
            expected.duration,
            DURATION_TOL,
            &format!("case {case_index}: duration"),
        );

        assert_eq!(
            case.sample_times.len(),
            expected.samples.len(),
            "case {case_index}: sample count mismatch"
        );

        for (sample_idx, (&t, expected_sample)) in
            case.sample_times.iter().zip(&expected.samples).enumerate()
        {
            assert_eq!(
                t, expected_sample.time,
                "case {case_index} sample {sample_idx}: sample_times/response time mismatch"
            );

            let position = trajectory.position(t);
            let velocity = trajectory.velocity(t);
            let acceleration = trajectory.acceleration(t);

            for i in 0..expected_sample.position.len() {
                assert_matches_nullable(
                    position[i],
                    expected_sample.position[i],
                    TOL,
                    &format!("case {case_index} sample {sample_idx}: position[{i}]"),
                );
                assert_matches_nullable(
                    velocity[i],
                    expected_sample.velocity[i],
                    TOL,
                    &format!("case {case_index} sample {sample_idx}: velocity[{i}]"),
                );
                assert_matches_nullable(
                    acceleration[i],
                    expected_sample.acceleration[i],
                    TOL,
                    &format!("case {case_index} sample {sample_idx}: acceleration[{i}]"),
                );
            }
        }
    }
}
