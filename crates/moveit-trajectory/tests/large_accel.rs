// Copyright (c) 2011, Georgia Tech Research Corporation
// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// Ported from moveit2 @ e017c91ee12984393a28ba246075c65f69cde3bf:
//   moveit_core/trajectory_processing/test/test_time_optimal_trajectory_generation.cpp
//   (`TEST(time_optimal_trajectory_generation, testLargeAccel)`)

//! Upstream `testLargeAccel`: samples `Trajectory::getAcceleration` across
//! the whole duration of a 6-DOF trajectory and asserts every component
//! stays within a generous bound, guarding against the algorithm producing
//! an unbounded acceleration spike.
//!
//! The fixture (`tests/fixtures/large_accel_waypoints.json`) holds
//! upstream's own test data verbatim, extracted mechanically from the
//! `.cpp` (see the fixture's own `source` field for the exact line range)
//! rather than retyped by hand — this crate's git history has a fixed case
//! of manual transcription silently dropping trailing digits from these
//! exact literals. Loading the data from JSON instead of writing it as Rust
//! float literals also sidesteps a `clippy::approx_constant` false
//! positive: one of upstream's `max_acceleration` components,
//! `0.78539816339699997`, is close enough to `FRAC_PI_4` for the lint to
//! flag it (verified: the two are 4038 ULPs apart, not equal and not one
//! ULP apart — this really is a coincidence in upstream's fixture data, not
//! a near-miss worth aliasing to the constant). A `serde_json`-deserialized
//! `f64` isn't a float literal in this crate's source at all, so the lint
//! has nothing to match; no `#[allow(...)]` or literal-hiding trick needed.

use std::fs;

use serde::Deserialize;

use moveit_trajectory::{Path, Trajectory};

#[derive(Deserialize)]
struct Fixture {
    max_acceleration: Vec<f64>,
    max_velocity: Vec<f64>,
    path_tolerance: f64,
    resample_dt: f64,
    time_step: f64,
    waypoints: Vec<Vec<f64>>,
}

fn load_fixture() -> Fixture {
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/fixtures/large_accel_waypoints.json"
    );
    let raw = fs::read_to_string(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
}

#[test]
fn upstream_test_large_accel() {
    let fixture = load_fixture();
    let waypoints: Vec<_> = fixture
        .waypoints
        .iter()
        .map(|w| nalgebra::DVector::from_vec(w.clone()))
        .collect();
    let max_velocity = nalgebra::DVector::from_vec(fixture.max_velocity);
    let max_acceleration = nalgebra::DVector::from_vec(fixture.max_acceleration);

    let path = Path::create(&waypoints, fixture.path_tolerance).unwrap();
    let trajectory =
        Trajectory::create(path, &max_velocity, &max_acceleration, fixture.time_step).unwrap();

    let sample_count = (trajectory.duration() / fixture.resample_dt).ceil() as u64;
    for sample in 0..=sample_count {
        // Always sample the end of the trajectory as well.
        let t = trajectory
            .duration()
            .min(sample as f64 * fixture.resample_dt);
        let acceleration = trajectory.acceleration(t);
        assert_eq!(acceleration.len(), 6);
        for i in 0..6 {
            assert!(
                acceleration[i].abs() < 100.0,
                "sample {sample}, joint {i}: {}",
                acceleration[i]
            );
        }
    }
}
