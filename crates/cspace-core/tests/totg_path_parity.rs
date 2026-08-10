// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `totg_path` op, which
//! measures [`cspace_core::trajectory::Path`] geometry alone -- `length`,
//! `getSwitchingPoints`, `getConfig`/`getTangent`/`getCurvature` -- with no
//! [`cspace_core::trajectory::Trajectory`] timing/integration involved. This
//! exists to isolate `Path::create`'s circular-blend construction from
//! `Trajectory::create`'s switching-point search: `totg_parity.rs`'s
//! `DURATION_TOL` is roughly four orders of magnitude looser than its
//! position/velocity/acceleration `TOL` (see that file's "Tolerance" doc
//! section), and PORTING-PLAN.md's round-12/round-13 oracle measurements
//! traced that gap to `Trajectory`'s integration rather than `Path`'s
//! geometry. A single fixture asserting only `Path` output, at tolerances
//! sized from `Path` output alone, keeps that isolation enforced by the
//! test suite rather than by a doc claim someone has to re-verify by hand.
//!
//! The single fixture case reuses `upstream_test2`'s five waypoints (also
//! used by `trajectory.rs`'s own `upstream_test2` unit test and by
//! `totg_parity.rs`'s case 5) with `max_deviation = 100.0`, and samples each
//! of the three circular blend segments at its 25%/50%/75% arc-length
//! points -- deliberately *not* a uniform `[0, length]` grid, which risks
//! landing a sample exactly on a switching point (`Path::config`/`tangent`/
//! `curvature` are only guaranteed continuous strictly inside a segment).
//! The blend boundaries this reuses (`Path::switching_points()`, not
//! independently re-derived): `[50, 140.23806829245763]`,
//! `[380.71934373703334, 879.8507114774787]`,
//! `[1084.8016572321708, 1163.3414735719157]`, total `length`
//! `1213.3414735719157`.
//!
//! `Path::switching_points()` is `pub(crate)` (see `path.rs`) and so is not
//! reachable from this file, which `cargo` compiles as a separate crate;
//! the fixture still records `switching_points` (for replay/documentation
//! and so a future in-crate unit test can compare against it directly), but
//! this test does not assert on that field.
//!
//! # Tolerance
//!
//! Measured with a throwaway diagnostic comparing this crate's own
//! `Path::config`/`tangent`/`curvature` at the fixture's nine sample points
//! against the captured oracle response (not asserted, then deleted before
//! committing, per this crate's established measure-then-delete
//! convention): `length` is bit-exact (`0` diff, matching PORTING-PLAN.md's
//! own `length` ULP-0 measurement); `config` maxes at
//! `2.2737367544323206e-13`; `tangent` maxes at `1.0547118733938987e-15`;
//! `curvature` (including `curvature_norm`, same magnitude class) maxes at
//! `2.168404344971009e-17`. These three floors span roughly four orders of
//! magnitude from each other because `config` carries the waypoints' own
//! ~1000-scale magnitude while `tangent` is unit-scale and `curvature` is
//! `O(1/radius)` at radii of tens to hundreds -- one shared constant would
//! either be too loose for `curvature` or spuriously fail on `config`'s
//! honest floating-point noise (the same "mirror problem" `totg_parity.rs`
//! documents for its own three constants). Split, each with roughly 3.6-4.0
//! orders of headroom over its measured floor: `CONFIG_TOL = 1e-9`
//! (~3.64 orders over `2.27e-13`), `TANGENT_TOL = 1e-11` (~3.98 orders over
//! `1.05e-15`), `CURVATURE_TOL = 1e-13` (~3.66 orders over `2.17e-17`).
//! All three are looser than PORTING-PLAN.md's cited ULP ceiling for this
//! case (config max 1 ULP, tangent max 19 ULP, curvature |k| max 3 ULP) --
//! at these magnitudes 1e-9/1e-11/1e-13 correspond to roughly 4500/9500/
//! 4600 ULP respectively, i.e. deliberately far looser than the ULP figures
//! themselves, which are a precision *report*, not a claim that this test
//! should assert at ULP granularity across machines/toolchains. Confirmed
//! by temporarily perturbing each comparison (not left in the committed
//! test) that both directions still discriminate: multiplying sample 0's
//! expected `config[0]` by `1.000001` and by `0.999999` both fail with
//! diff `~1.424e-3`, far past `CONFIG_TOL`; the same two perturbations
//! against `tangent[0]` fail with diff `~2.623e-7`, far past
//! `TANGENT_TOL`; against `curvature_norm`, diff `~1.188e-8`, far past
//! `CURVATURE_TOL`.
//!
//! ## Round 14: re-measured under `float_roundtrip`, unchanged
//!
//! `serde_json`'s default f64 parser is not correctly rounded (round 13
//! measured the floors above with it); the workspace now resolves
//! `serde_json` with the `float_roundtrip` feature (`Cargo.toml`), so every
//! fixture literal deserializes to the exact value the oracle wrote. Re-
//! running the same throwaway diagnostic under the fixed parser reproduces
//! the identical floors bit-for-bit -- `2.27373675443232059e-13` /
//! `1.05471187339389871e-15` / `2.16840434497100887e-17` /
//! `1.04083408558608426e-17` -- so `CONFIG_TOL`/`TANGENT_TOL`/
//! `CURVATURE_TOL` are unchanged. This is not assumed from the floors
//! matching; a standalone checker (outside this workspace, built without
//! `float_roundtrip`) directly confirmed which literals in
//! `totg_path_request.json`/`totg_path_response.json` the old parser got
//! wrong -- 2 of 30 and 8 of 138 respectively, all `sample_arc_lengths`/`s`
//! echoes or `config`/`tangent`/`curvature` components at samples other than
//! the ones that set each category's max -- and the diagnostic was
//! instrumented to print which `(sample, component)` sets each reported
//! max: `config` at sample 5's `config[2]` (`1043.2228678362944`, not among
//! the corrupted literals), `tangent` at sample 8's `tangent[0]`
//! (`-0.3752519769224667`, not corrupted), `curvature` at sample 6's
//! `curvature[0]` (`0.007505039538449353`, not corrupted), `curvature_norm`
//! at sample 7 (`0.019999999999999997`, not corrupted). None of the eight
//! corrupted response literals -- `-0.0009300842487151479`,
//! `1020.0139966538143`, `-0.00010177853025357365`,
//! `-0.0009923757125245853`, `0.013867504840522621`,
//! `-0.9238795325112871`, plus the two `s` echoes -- is the term that sets
//! any of the four reported maxima, which is why fixing the parser left
//! this fixture's floors unmoved even though the fixture itself does
//! contain corrupted-under-the-old-parser literals. The six bidirectional
//! perturbation checks from round 13 (`config[0]`/`tangent[0]`/
//! `curvature_norm`, `*1.000001` and `*0.999999`) were re-run against the
//! fixed parser and still fail with the same diffs (`~1.424e-3` /
//! `~2.623e-7` / `~1.188e-8`), confirming discriminating power is likewise
//! unaffected.

use std::fs;

use nalgebra::DVector;
use serde::Deserialize;

use cspace_core::trajectory::Path;

/// See this module's "Tolerance" doc section: `length` is bit-exact, so it
/// is compared with `assert_eq!` rather than a tolerance constant.
const CONFIG_TOL: f64 = 1e-9;

/// See this module's "Tolerance" doc section.
const TANGENT_TOL: f64 = 1e-11;

/// See this module's "Tolerance" doc section. Shared by `curvature` and
/// `curvature_norm` -- both are the same magnitude class (`O(1/radius)`).
const CURVATURE_TOL: f64 = 1e-13;

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

#[derive(Deserialize)]
struct TotgPathRequestCase {
    waypoints: Vec<Vec<f64>>,
    max_deviation: f64,
    sample_arc_lengths: Vec<f64>,
}

#[derive(Deserialize)]
struct TotgPathRequest {
    cases: Vec<TotgPathRequestCase>,
}

#[derive(Deserialize)]
struct TotgPathResultSample {
    s: f64,
    config: Vec<f64>,
    tangent: Vec<f64>,
    curvature: Vec<f64>,
    curvature_norm: f64,
}

#[derive(Deserialize)]
struct TotgPathResultCase {
    ok: bool,
    length: f64,
    samples: Vec<TotgPathResultSample>,
}

#[derive(Deserialize)]
struct TotgPathResult {
    cases: Vec<TotgPathResultCase>,
}

#[derive(Deserialize)]
struct TotgPathResponseEntry {
    result: TotgPathResult,
}

#[test]
fn totg_path_matches_the_oracle() {
    let requests: Vec<TotgPathRequest> =
        serde_json::from_str(&read_fixture("totg_path_request.json"))
            .expect("parse totg_path_request.json");
    let responses: Vec<TotgPathResponseEntry> =
        serde_json::from_str(&read_fixture("totg_path_response.json"))
            .expect("parse totg_path_response.json");
    assert_eq!(requests.len(), responses.len());
    let request = &requests[0];
    let response = &responses[0];
    assert_eq!(request.cases.len(), response.result.cases.len());

    for (case_index, (case, expected)) in
        request.cases.iter().zip(&response.result.cases).enumerate()
    {
        assert!(
            expected.ok,
            "case {case_index}: oracle reported failure but this fixture only carries a \
             successful case"
        );

        let waypoints: Vec<DVector<f64>> = case
            .waypoints
            .iter()
            .map(|wp| DVector::from_vec(wp.clone()))
            .collect();

        let path = Path::create(&waypoints, case.max_deviation)
            .unwrap_or_else(|e| panic!("case {case_index}: Path::create failed: {e}"));

        assert_eq!(path.length(), expected.length, "case {case_index}: length");

        assert_eq!(
            case.sample_arc_lengths.len(),
            expected.samples.len(),
            "case {case_index}: sample count mismatch"
        );

        for (sample_idx, (&s, expected_sample)) in case
            .sample_arc_lengths
            .iter()
            .zip(&expected.samples)
            .enumerate()
        {
            assert_eq!(
                s, expected_sample.s,
                "case {case_index} sample {sample_idx}: sample_arc_lengths/response s mismatch"
            );

            let config = path.config(s);
            let tangent = path.tangent(s);
            let curvature = path.curvature(s);

            assert_eq!(
                config.len(),
                expected_sample.config.len(),
                "case {case_index} sample {sample_idx}: config dimension mismatch"
            );
            for i in 0..expected_sample.config.len() {
                let diff = (config[i] - expected_sample.config[i]).abs();
                assert!(
                    diff <= CONFIG_TOL,
                    "case {case_index} sample {sample_idx}: config[{i}] actual={}, \
                     expected={}, abs diff={diff}",
                    config[i],
                    expected_sample.config[i]
                );
            }

            for i in 0..expected_sample.tangent.len() {
                let diff = (tangent[i] - expected_sample.tangent[i]).abs();
                assert!(
                    diff <= TANGENT_TOL,
                    "case {case_index} sample {sample_idx}: tangent[{i}] actual={}, \
                     expected={}, abs diff={diff}",
                    tangent[i],
                    expected_sample.tangent[i]
                );
            }

            for i in 0..expected_sample.curvature.len() {
                let diff = (curvature[i] - expected_sample.curvature[i]).abs();
                assert!(
                    diff <= CURVATURE_TOL,
                    "case {case_index} sample {sample_idx}: curvature[{i}] actual={}, \
                     expected={}, abs diff={diff}",
                    curvature[i],
                    expected_sample.curvature[i]
                );
            }

            let curvature_norm = curvature.norm();
            let diff = (curvature_norm - expected_sample.curvature_norm).abs();
            assert!(
                diff <= CURVATURE_TOL,
                "case {case_index} sample {sample_idx}: curvature_norm actual={curvature_norm}, \
                 expected={}, abs diff={diff}",
                expected_sample.curvature_norm
            );
        }
    }
}
