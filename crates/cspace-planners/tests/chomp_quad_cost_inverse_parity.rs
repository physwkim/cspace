// Copyright (c) 2026, cspace contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `chomp_quad_cost_inverse`
//! op, ground truth for [`cspace_planners::chomp::ChompCost::quadratic_cost_inverse`]'s
//! decomposition-family gap left open by round 16 and requested from the
//! oracle in round 17 (`doc/oracle-request-quad-cost-inv.md`).
//!
//! # Why this needs the real upstream, not a residual
//!
//! `Eigen::MatrixXd::inverse()` is always `PartialPivLU` (its row count is
//! `Dynamic` at compile time); `nalgebra::DMatrix::try_inverse()` dispatches
//! on the matrix's *runtime* size instead -- closed-form cofactor for
//! `1..=4`, `lu::try_invert_to` (partial-pivoted LU, the same strategy as
//! Eigen) for `>=5` (verified against the pinned `nalgebra` 0.35.0 source in
//! round 18, not assumed from its docs -- see `doc/oracle-request-quad-cost-
//! inv.md`'s "Verified" section). A residual check (round 16's
//! `quad_cost_inv_stays_a_true_inverse_across_both_algorithm_branches`)
//! proves the nalgebra result is *a* correct inverse, not that it is
//! *Eigen's* inverse -- two different decompositions can each have a
//! vanishing residual and still disagree in the low bits, which is
//! precisely the question this test settles.
//!
//! # Fixture
//!
//! `chomp_quad_cost_inverse_request.json`/`_response.json` are the oracle's
//! own wire request/response lines (`{"id", "op", ...}` /
//! `{"id", "ok", "result"}`), captured directly against oracle stamp
//! `6797447ac4dc46e9` -- the op links the real upstream `ChompCost`
//! rather than a transcription of its constructor (`oracle.cpp`'s own doc
//! comment on `chompQuadCostInverse`), so nothing about this comparison's
//! request-building can itself be a source of disagreement.
//!
//! Five cases, one per side of the algorithm-branch boundary, exactly the
//! oracle request document's table: `num_points` 13/14/15/16/20 ->
//! `num_vars_free` 1/2/3/4/8. All five share `discretization = 0.1`,
//! `derivative_costs = [0.0, 1.0, 0.0]` (acceleration only, matching this
//! crate's own `cost.rs`/`optimizer.rs` test fixtures), `ridge_factor =
//! 1e-6`; only `num_points` varies, so the algorithm-branch boundary is the
//! one thing under test. `group_name` is omitted from the request (the
//! oracle's own default, `panda_arm`) since `ChompCost`'s constructor never
//! reads it.
//!
//! Ground truth was re-run directly against the live oracle when this test
//! was written (not copied from the request document's worked example) --
//! `num_vars_free` matched the table for all five cases, which is itself
//! confirmation that `DIFF_RULE_LENGTH` and the free-point boundary formula
//! agree between the two ports; a mismatch there would have been the more
//! interesting finding, per the oracle op's own doc comment.
//!
//! # Tolerance
//!
//! Measured, not guessed: printing each case's actual
//! `max |actual - expected|` before picking a bound (temporarily, via
//! `eprintln!`, since removed) found agreement, but not the bit-exact
//! agreement this test's own gap-closing question might suggest --
//! `1..=4` (cofactor) and `>=5` (LU) are still two different code paths on
//! nalgebra's side, evaluated in a different operation order than Eigen's
//! `PartialPivLU` even where the underlying strategy matches (the `>=5`
//! case), and floating-point addition is not associative. Measured maxima,
//! smallest case to largest: `num_points` 13 -> `1.78e-15`, 14 ->
//! `1.95e-14`, 15 -> `1.78e-13`, 16 -> `5.97e-13`, 20 -> `2.68e-11`. The
//! trend (each case's max roughly an order of magnitude looser than the
//! previous) tracks matrix size and entry magnitude, not a step change at
//! the `1..=4`/`>=5` boundary itself -- exactly what accumulated rounding
//! noise from two independently-ordered decompositions looks like, not a
//! real disagreement. `TOL = 1e-7` gives the loosest case (`2.68e-11`) about
//! 3.6 orders of magnitude of headroom, matching this port's established
//! convention for a two-independent-implementations comparison (see
//! `totg_parity.rs`'s "Tolerance" section) -- loose enough not to flake on a
//! future nalgebra/Eigen point release nudging pivot order, tight enough
//! that a real decomposition disagreement (which round 16's residual check
//! could not have ruled out) would still fail it by three-plus orders of
//! magnitude.

use std::fs;

use nalgebra::DMatrix;
use serde::Deserialize;

use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_planners::chomp::{ChompCost, ChompTrajectory};

const TOL: f64 = 1e-7;
const GROUP: &str = "panda_arm";

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/chomp/{}"),
        file_name
    )
}

fn read_fixture(file_name: &str) -> String {
    let path = fixture_path(file_name);
    fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

fn panda_model() -> RobotModel {
    let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.urdf");
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.srdf");
    let urdf_xml =
        fs::read_to_string(urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_file(urdf_path).expect("panda.urdf parses");
    let srdf = SrdfModel::parse_file(srdf_path).expect("panda.srdf parses");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("panda model builds")
}

#[derive(Deserialize)]
struct RequestLine {
    id: u64,
    num_points: usize,
    discretization: f64,
    derivative_costs: Vec<f64>,
    ridge_factor: f64,
}

#[derive(Deserialize)]
struct ResponseResult {
    num_vars_free: usize,
    quad_cost_inverse: Vec<Vec<f64>>,
}

#[derive(Deserialize)]
struct ResponseLine {
    id: u64,
    ok: bool,
    result: ResponseResult,
}

#[test]
fn quad_cost_inverse_matches_the_oracle() {
    let requests: Vec<RequestLine> =
        serde_json::from_str(&read_fixture("chomp_quad_cost_inverse_request.json"))
            .expect("parse chomp_quad_cost_inverse_request.json");
    let responses: Vec<ResponseLine> =
        serde_json::from_str(&read_fixture("chomp_quad_cost_inverse_response.json"))
            .expect("parse chomp_quad_cost_inverse_response.json");
    assert_eq!(requests.len(), responses.len());
    assert_eq!(requests.len(), 5, "expected all 5 boundary cases");

    let model = panda_model();

    for (request, response) in requests.iter().zip(&responses) {
        assert_eq!(
            request.id, response.id,
            "request/response fixture id mismatch"
        );
        assert!(
            response.ok,
            "case id {}: oracle reported ok=false",
            request.id
        );

        let trajectory = ChompTrajectory::from_num_points(
            &model,
            request.num_points,
            request.discretization,
            GROUP,
        )
        .unwrap_or_else(|e| panic!("case id {}: build trajectory: {e}", request.id));

        let cost = ChompCost::new(&trajectory, &request.derivative_costs, request.ridge_factor)
            .unwrap_or_else(|e| panic!("case id {}: ChompCost::new: {e}", request.id));

        let actual = cost.quadratic_cost_inverse();
        assert_eq!(
            actual.nrows(),
            response.result.num_vars_free,
            "case id {}: num_vars_free mismatch",
            request.id
        );

        let expected = DMatrix::from_row_iterator(
            response.result.num_vars_free,
            response.result.num_vars_free,
            response.result.quad_cost_inverse.iter().flatten().copied(),
        );

        let mut max_abs_diff = 0.0_f64;
        for r in 0..actual.nrows() {
            for c in 0..actual.ncols() {
                let diff = (actual[(r, c)] - expected[(r, c)]).abs();
                max_abs_diff = max_abs_diff.max(diff);
            }
        }
        assert!(
            max_abs_diff <= TOL,
            "case id {} (num_points={}, num_vars_free={}): max |actual - expected| = {max_abs_diff}, expected <= {TOL}",
            request.id,
            request.num_points,
            response.result.num_vars_free,
        );
    }
}
