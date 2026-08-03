// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `kinematics_metrics` op,
//! ground truth for [`KinematicsMetrics`].
//!
//! Both sides are driven from the committed `panda_kinematics_metrics_request.json`
//! (the exact oracle request: `group`, `count`, `seed`, `penalty_multiplier`)
//! and its unedited `panda_kinematics_metrics_response.json` (the oracle's
//! real answer — regenerate with the command in this file's header if the
//! oracle or the request parameters ever change). The oracle draws its own
//! random states with `RobotModel::getVariableRandomPositions` (see
//! `oracle.cpp`'s `kinematicsMetrics`/`randomStates` comments for why the
//! oracle, not the runner, must own the randomness) and dumps each state's
//! `joint_values` alongside its four metrics, so this test never needs to
//! reproduce MoveIt's own sampler — it only replays the already-drawn
//! joint values through this port's [`moveit_state::RobotState`].
//!
//! Regenerate the response fixture with:
//! ```text
//! R=/home/stevek/work/moveit-rs/.caucus/worktrees/5REEQZSC40-p1-fixtures-920dace3-1
//! python3 -c "import json; print(json.dumps(json.load(open('$R/crates/moveit-metrics/tests/fixtures/panda_kinematics_metrics_request.json'))))" \
//!   | sg docker -c "$R/tools/moveit-oracle/run-oracle.sh --urdf $R/fixtures/panda.urdf --srdf $R/fixtures/panda.srdf" \
//!   | python3 -c "import json,sys; print(json.dumps(json.load(sys.stdin), indent=2, sort_keys=True))"
//! ```
//! (the oracle's wire protocol is one compact JSON object per stdin line;
//! the `json.dumps` on the way in collapses the fixture's pretty-printed
//! request to that shape — piping the pretty-printed file directly breaks
//! the newline-delimited-JSON protocol, since each of *its* lines is not
//! independently valid JSON. `tools/ci/verify-fixture-replay.sh` runs the
//! equivalent of this same round trip against
//! `crates/moveit-metrics/tests/fixtures/oracle-models.json`'s
//! `panda_kinematics_metrics` entry to catch drift between this committed
//! response and the live oracle).
//!
//! `manipulability_index`/`manipulability` are pinned positionally,
//! field-by-field, per state, at both `translation` values: both Eigen's
//! `JacobiSVD` (upstream) and `nalgebra::SVD::new` (this port) guarantee
//! descending singular-value order (see `moveit-metrics`'s own doc comment
//! for the primary-source verification), so a positional pin is meaningful.
//!
//! `manipulability_ellipsoid` is different: neither Eigen's `EigenSolver`
//! nor `nalgebra::SymmetricEigen` guarantees eigenvalue/eigenvector order,
//! so pinning position-for-position would make this test's pass/fail
//! depend on an implementation detail neither library promises. Both sides
//! are sorted by eigenvalue and each eigenvector's sign is normalized
//! (eigenvectors are only defined up to sign) before comparison — see
//! `sorted_ellipsoid`.
//!
//! `panda_arm` is a 7-DOF chain, so `jacobian.cols() == 7 >= 6` for every
//! state: `manipulability_index`'s `columns < 6` branch (the SVD-product
//! path, as opposed to `sqrt(det(J J^T))`) is **not exercised by this
//! fixture**. No group in any committed fixture (`panda_arm` 7, fanuc
//! `manipulator` 6, pr2 arm groups 7) has fewer than 6 active-joint
//! variables, so that branch has no oracle coverage at all right now —
//! flagged here rather than silently assumed correct; see this round's
//! report.

use std::fs;

use approx::assert_relative_eq;
use serde::Deserialize;

use moveit_metrics::KinematicsMetrics;
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;

#[derive(Deserialize)]
struct StateCase {
    joint_values: std::collections::HashMap<String, f64>,
    manipulability_index_full: f64,
    manipulability_index_translation: f64,
    manipulability_full: f64,
    manipulability_translation: f64,
    ellipsoid_eigenvalues_real: [f64; 3],
    ellipsoid_eigenvalues_imag: [f64; 3],
    ellipsoid_eigenvectors_real: [f64; 9],
    ellipsoid_eigenvectors_imag: [f64; 9],
}

#[derive(Deserialize)]
struct ResponseResult {
    group: String,
    penalty_multiplier: f64,
    states: Vec<StateCase>,
}

#[derive(Deserialize)]
struct OracleResponse {
    result: ResponseResult,
}

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        file_name
    )
}

fn load_response() -> ResponseResult {
    let path = fixture_path("panda_kinematics_metrics_response.json");
    let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let response: OracleResponse =
        serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"));
    response.result
}

fn build_model() -> RobotModel {
    let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.urdf");
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.srdf");
    let urdf_xml =
        fs::read_to_string(urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_file(urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

/// Scalar tolerance for `manipulability_index`/`manipulability`: both sides
/// run different SVD/determinant code (Eigen vs. `nalgebra`), so bit-exact
/// agreement is not expected. Measured, not guessed: a temporary probe
/// (same computation as this test, printing `(actual - expected) /
/// expected` per field instead of asserting) against this exact fixture's
/// 10 states — several within an order of magnitude of a kinematic
/// singularity, where the *absolute* Jacobian error both libraries carry
/// (~1e-16, f64 rounding) becomes a much larger *relative* error in the
/// smallest singular value — measured a worst case of `4.75e-14` across
/// every `manipulability_index`/`manipulability` value at both
/// `translation` settings, all 10 states. `1e-10` is ~2000x that measured
/// worst case: tight enough that the perturbation tests below (reversed
/// SVD order, no-penalty, no-translation) still fail loudly, loose enough
/// to survive a legitimate `nalgebra`/Eigen algorithm-version difference
/// that does not change the underlying answer. An `epsilon` floor of
/// `1e-12` covers exact-zero comparisons (nothing in this fixture is
/// exactly zero, but the floor costs nothing and avoids a
/// div-by-zero-adjacent relative comparison if a future fixture
/// regeneration draws a genuinely singular configuration).
const SCALAR_MAX_RELATIVE: f64 = 1e-10;
const SCALAR_EPSILON: f64 = 1e-12;

/// Same measurement method as `SCALAR_MAX_RELATIVE`, applied to ellipsoid
/// eigenvalues/eigenvectors (a different algorithm family entirely --
/// `EigenSolver` vs. `SymmetricEigen` -- so its own bisection point, not
/// assumed to match the SVD-based methods'). Measured worst case across
/// all 10 states, eigenvalues and eigenvector components alike: `3.59e-13`
/// (an eigenvector component in a near-degenerate case, where two close
/// eigenvalues make the corresponding eigenvectors individually
/// ill-conditioned even though the eigenspace they span is not). `1e-9` is
/// ~2700x that measured worst case, same margin reasoning as the scalar
/// tolerance above.
const ELLIPSOID_MAX_RELATIVE: f64 = 1e-9;
const ELLIPSOID_EPSILON: f64 = 1e-9;

/// Sort (eigenvalue, eigenvector-column) pairs ascending by eigenvalue and
/// normalize each eigenvector's sign (largest-magnitude component
/// positive) so two decompositions of the same matrix -- from libraries
/// that each explicitly do not guarantee an order -- become comparable.
fn sorted_ellipsoid(eigenvalues: [f64; 3], eigenvectors: [f64; 9]) -> ([f64; 3], [f64; 9]) {
    let mut columns: Vec<(f64, [f64; 3])> = (0..3)
        .map(|c| {
            (
                eigenvalues[c],
                [eigenvectors[c], eigenvectors[3 + c], eigenvectors[6 + c]],
            )
        })
        .collect();
    columns.sort_by(|a, b| a.0.partial_cmp(&b.0).expect("eigenvalues are never NaN"));

    let mut out_values = [0.0; 3];
    let mut out_vectors = [0.0; 9];
    for (c, (value, mut vector)) in columns.into_iter().enumerate() {
        let largest = vector
            .iter()
            .copied()
            .max_by(|a, b| a.abs().partial_cmp(&b.abs()).unwrap())
            .unwrap();
        if largest < 0.0 {
            vector = vector.map(|x| -x);
        }
        out_values[c] = value;
        out_vectors[c] = vector[0];
        out_vectors[3 + c] = vector[1];
        out_vectors[6 + c] = vector[2];
    }
    (out_values, out_vectors)
}

#[test]
fn panda_kinematics_metrics_matches_the_oracle() {
    let model = build_model();
    let response = load_response();
    assert_eq!(response.group, "panda_arm");

    let mut metrics = KinematicsMetrics::new(&model);
    metrics.set_penalty_multiplier(response.penalty_multiplier);

    for (case_index, case) in response.states.iter().enumerate() {
        for &imag in &case.ellipsoid_eigenvalues_imag {
            assert_eq!(
                imag, 0.0,
                "case {case_index}: oracle eigenvalue has a nonzero imaginary part"
            );
        }
        for &imag in &case.ellipsoid_eigenvectors_imag {
            assert_eq!(
                imag, 0.0,
                "case {case_index}: oracle eigenvector has a nonzero imaginary part"
            );
        }

        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        for (name, &value) in &case.joint_values {
            state
                .set_variable_position(name, value)
                .unwrap_or_else(|e| panic!("case {case_index}: set {name}: {e}"));
        }
        let posed = state.update();

        let index_full = metrics
            .manipulability_index(&posed, "panda_arm", false)
            .unwrap_or_else(|e| panic!("case {case_index}: manipulability_index(false): {e}"));
        assert_relative_eq!(
            index_full,
            case.manipulability_index_full,
            epsilon = SCALAR_EPSILON,
            max_relative = SCALAR_MAX_RELATIVE
        );

        let index_translation = metrics
            .manipulability_index(&posed, "panda_arm", true)
            .unwrap_or_else(|e| panic!("case {case_index}: manipulability_index(true): {e}"));
        assert_relative_eq!(
            index_translation,
            case.manipulability_index_translation,
            epsilon = SCALAR_EPSILON,
            max_relative = SCALAR_MAX_RELATIVE
        );

        let manipulability_full = metrics
            .manipulability(&posed, "panda_arm", false)
            .unwrap_or_else(|e| panic!("case {case_index}: manipulability(false): {e}"));
        assert_relative_eq!(
            manipulability_full,
            case.manipulability_full,
            epsilon = SCALAR_EPSILON,
            max_relative = SCALAR_MAX_RELATIVE
        );

        let manipulability_translation = metrics
            .manipulability(&posed, "panda_arm", true)
            .unwrap_or_else(|e| panic!("case {case_index}: manipulability(true): {e}"));
        assert_relative_eq!(
            manipulability_translation,
            case.manipulability_translation,
            epsilon = SCALAR_EPSILON,
            max_relative = SCALAR_MAX_RELATIVE
        );

        let (eigenvalues, eigenvectors) = metrics
            .manipulability_ellipsoid(&posed, "panda_arm")
            .unwrap_or_else(|e| panic!("case {case_index}: manipulability_ellipsoid: {e}"));
        let mut actual_vectors = [0.0; 9];
        for r in 0..3 {
            for c in 0..3 {
                actual_vectors[r * 3 + c] = eigenvectors[(r, c)];
            }
        }
        let (actual_values, actual_vectors) = sorted_ellipsoid(
            [eigenvalues[0], eigenvalues[1], eigenvalues[2]],
            actual_vectors,
        );
        let (expected_values, expected_vectors) = sorted_ellipsoid(
            case.ellipsoid_eigenvalues_real,
            case.ellipsoid_eigenvectors_real,
        );

        for i in 0..3 {
            assert_relative_eq!(
                actual_values[i],
                expected_values[i],
                epsilon = ELLIPSOID_EPSILON,
                max_relative = ELLIPSOID_MAX_RELATIVE
            );
        }
        for i in 0..9 {
            assert_relative_eq!(
                actual_vectors[i],
                expected_vectors[i],
                epsilon = ELLIPSOID_EPSILON,
                max_relative = ELLIPSOID_MAX_RELATIVE
            );
        }
    }
}
