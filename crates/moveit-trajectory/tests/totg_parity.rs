// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `totg` op, covering
//! [`moveit_trajectory::Path`]/[`moveit_trajectory::Trajectory`] (the
//! model-independent numeric core of `time_optimal_trajectory_generation.
//! hpp` lines 62-192) -- *not* the `TimeOptimalTrajectoryGeneration`
//! adapter class (header line 193 on), which is out of this crate's scope
//! (see `PORTING-PLAN.md` and `crate::trajectory`'s module doc comment).
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
//! line-for-line port, so tight agreement is expected. `TOL` is set from
//! what this fixture actually produces: the observed case 5 (the longest,
//! most-integrated case) position/velocity/acceleration diffs are all
//! below `2e-9` in absolute terms against magnitudes up to ~1900, i.e.
//! relative diffs on the order of `1e-12`; `1e-6` is used as `epsilon` to
//! leave headroom without masking a real disagreement, matching
//! `ruckig_parity.rs`'s existing convention of an absolute `epsilon` rather
//! than a relative bound (several expected values, e.g. case 2's terminal
//! velocity `3.19e-17`, are legitimately near zero, where a relative bound
//! is meaningless).

use std::fs;

use nalgebra::DVector;
use serde::Deserialize;

use moveit_trajectory::{Path, Trajectory};

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
fn assert_matches_nullable(actual: f64, expected: Option<f64>, context: &str) {
    match expected {
        Some(expected) => {
            let diff = (actual - expected).abs();
            assert!(
                diff <= TOL,
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
                    &format!("case {case_index} sample {sample_idx}: position[{i}]"),
                );
                assert_matches_nullable(
                    velocity[i],
                    expected_sample.velocity[i],
                    &format!("case {case_index} sample {sample_idx}: velocity[{i}]"),
                );
                assert_matches_nullable(
                    acceleration[i],
                    expected_sample.acceleration[i],
                    &format!("case {case_index} sample {sample_idx}: acceleration[{i}]"),
                );
            }
        }
    }
}
