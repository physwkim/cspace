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
//! path, as opposed to `sqrt(det(J J^T))`) is not exercised by this
//! fixture. That branch is pinned instead by
//! `panda_arm_5dof_kinematics_metrics_matches_the_oracle`, against a second
//! group (`panda_arm_5dof`, `panda_link0` to `panda_link5`, 5 active
//! joints) that exists only in the crate-local, deliberately-divergent
//! `crates/moveit-metrics/tests/fixtures/panda.srdf` (see that file's
//! trailing comment, and `tools/ci/verify-fixture-provenance.sh`'s
//! `DIVERGENT` table entry for it) — no committed *upstream* fixture
//! (`panda_arm` 7, fanuc `manipulator` 6, pr2 arm groups 7) has fewer than
//! 6 active-joint variables.
//!
//! This pins the branch's *narrowing* direction (the threshold literal
//! shrinking, e.g. `columns < 6` -> `columns < 5` or `< 4`): confirmed by
//! deliberately making that change and re-running both parity tests —
//! `panda_arm_5dof_kinematics_metrics_matches_the_oracle` fails at both
//! `< 5` and `< 4` (`panda_kinematics_metrics_matches_the_oracle` does not,
//! since `panda_arm`'s 7 columns stay on the determinant path either way).
//! Shrinking the threshold below `panda_arm_5dof`'s column count routes it
//! onto the `sqrt(det(J J^T))` path with a `J` that has only 5 columns —
//! `J J^T` is then a 6x6 (or 3x3, `translation=true`) matrix built from a
//! rank-<=5 outer product, so its determinant is exactly (up to rounding)
//! `0.0`, diverging hard from the oracle's genuinely nonzero SVD-product
//! answer. The branch's *widening* direction (`columns < 6` -> `columns <
//! 8`, routing a `>= 6`-column group like `panda_arm` onto the SVD-product
//! path instead) stays unobservable through any oracle-diff fixture,
//! confirmed the same way: product of singular values equals `sqrt(det(J
//! J^T))` exactly for any full row-rank `J`, so widening the threshold
//! only ever *adds* full-row-rank groups to the SVD-product side, where
//! both formulas already agree. The two directions are not symmetric —
//! only narrowing below an *exercised* group's own column count can expose
//! a real divergence, and only `panda_arm_5dof` (once its own group falls
//! below the threshold) can expose it in this fixture set.
//!
//! Regenerate that response fixture with:
//! ```text
//! R=/home/stevek/work/moveit-rs/.caucus/worktrees/5REEQZSC40-p1-fixtures-920dace3-1
//! python3 -c "import json; print(json.dumps(json.load(open('$R/crates/moveit-metrics/tests/fixtures/panda_arm_5dof_kinematics_metrics_request.json'))))" \
//!   | sg docker -c "$R/tools/moveit-oracle/run-oracle.sh --urdf $R/fixtures/panda.urdf --srdf $R/crates/moveit-metrics/tests/fixtures/panda.srdf" \
//!   | python3 -c "import json,sys; print(json.dumps(json.load(sys.stdin), indent=2, sort_keys=True))"
//! ```
//! (same protocol note as above: the request must go in as compact,
//! newline-delimited JSON; note the SRDF path is the crate-local divergent
//! copy, not `fixtures/panda.srdf`).

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

fn load_response(file_name: &str) -> ResponseResult {
    let path = fixture_path(file_name);
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

/// `panda.srdf` here is the crate-local, deliberately-divergent copy (see
/// its trailing comment): it adds a `panda_arm_5dof` group that no
/// upstream panda SRDF group provides, needed to exercise
/// `manipulability_index`/`manipulability`'s `columns < 6` branch.
/// `panda.urdf` is still the shared, unmodified root fixture.
fn build_model_with_5dof_group() -> RobotModel {
    let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.urdf");
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/panda.srdf");
    let urdf_xml =
        fs::read_to_string(urdf_path).unwrap_or_else(|e| panic!("read {urdf_path}: {e}"));
    let urdf = urdf_rs::read_file(urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(srdf_path).expect("divergent fixture SRDF must parse");
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
/// smallest singular value — measured a worst case of `4.7503e-14` across
/// every `manipulability_index`/`manipulability` value at both
/// `translation` settings, all 10 states.
///
/// `SCALAR_EPSILON` is `0.0`, not a small-but-nonzero floor. `approx`'s
/// `relative_eq` passes if `|a - b| <= epsilon` **or**
/// `|a - b| <= max_relative * max(|a|, |b|)` — an *or*, not an *and* — so
/// any nonzero `epsilon` sets a hard floor under the whole comparison that
/// `max_relative` can never tighten below. This fixture's 40 scalar values
/// range from `2.94616e-11` to `7.03754e-06` (`joint_limits_penalty`
/// pushes `panda_arm`'s default-ish sampled poses close to a limit).
/// A former `epsilon = 1e-12` therefore dominated *every* comparison: at
/// the fixture's largest value, `max_relative * |expected|` = `1e-10 *
/// 7.03754e-6` ≈ `7.0e-16`, three orders of magnitude below that `epsilon`
/// floor — so the floor, not `max_relative`, decided every pass/fail, and
/// at the fixture's *smallest* value the floor alone permitted a relative
/// error up to `1e-12 / 2.94616e-11` ≈ `3.4%`, a hundred million times
/// looser than the actual measured worst case. `"1e-10 is ~2000x the
/// measured worst case"` and `"the floor costs nothing"` were both false
/// under that regime: the floor was not incidental, it was the entire
/// check.
///
/// With `epsilon = 0.0`, `max_relative` alone decides every comparison, so
/// its own headroom is real: re-bisected one constant at a time (`approx`'s
/// OR means bisecting both together re-hides this exact bug), `1e-13`
/// passes every case, `1e-14` fails on `manipulability_index_full` at a
/// measured `4.7503e-14` relative error — reproducing the probe's global
/// worst case exactly, which confirms that failure *is* the binding case,
/// not a second, larger one the probe missed. `1e-10` is `1e-10 /
/// 4.7503e-14` ≈ **2105x** that floor: this time actually the applied
/// margin, not a claim `epsilon` silently overrode — tight enough that the
/// perturbation tests below (reversed SVD order, no-penalty,
/// no-translation) still fail loudly, loose enough to survive a
/// legitimate `nalgebra`/Eigen algorithm-version difference that does not
/// change the underlying answer.
///
/// `epsilon = 0.0` is only safe because nothing in *this* fixture's
/// expected values is exactly zero (`2.94616e-11` is the smallest). A
/// future regeneration that draws a genuinely singular configuration
/// (an exact-zero expected value, where any nonzero `max_relative *
/// max(|a|, |b|)` is also zero) would need its own `epsilon`, sized to
/// *that* value's magnitude when it is measured — not a floor set larger
/// than every value already in the fixture on the chance one might
/// someday be smaller.
const SCALAR_MAX_RELATIVE: f64 = 1e-10;
const SCALAR_EPSILON: f64 = 0.0;

/// Same measurement method and same `epsilon`-as-hard-floor bug as
/// `SCALAR_MAX_RELATIVE`, applied to ellipsoid eigenvalues/eigenvectors (a
/// different algorithm family entirely — `EigenSolver` vs.
/// `SymmetricEigen` — so its own bisection point, not assumed to match the
/// SVD-based methods'). Measured worst case across all 10 states,
/// eigenvalues and eigenvector components alike: `3.5860e-13` (an
/// eigenvector component in a near-degenerate case, where two close
/// eigenvalues make the corresponding eigenvectors individually
/// ill-conditioned even though the eigenspace they span is not). This
/// fixture's smallest eigenvalue is `3.29669e-03` and smallest eigenvector
/// component `1.62356e-03`; a former `epsilon = 1e-9` floor permitted a
/// relative error up to `1e-9 / 1.62356e-03` ≈ `6.16e-7` there — a real
/// margin, but one `epsilon`, not `max_relative`, was granting.
///
/// With `epsilon = 0.0`, re-bisected the same one-constant-at-a-time way:
/// `1e-12` passes every case, `1e-13` fails on a
/// `manipulability_ellipsoid` eigenvector component at a measured
/// `3.586047e-13` relative error — again reproducing the probe's global
/// worst case exactly. `1e-9` is `1e-9 / 3.5860e-13` ≈ **2789x** that
/// floor, this time as the actually-applied margin.
const ELLIPSOID_MAX_RELATIVE: f64 = 1e-9;
const ELLIPSOID_EPSILON: f64 = 0.0;

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

/// Shared body for both parity tests: replays every state in `response`
/// through `model` via [`KinematicsMetrics`] and asserts all four metrics
/// against the oracle's recorded values. `response.group` names the group
/// to query, so this works unchanged for `panda_arm` (`columns >= 6`) and
/// `panda_arm_5dof` (`columns < 6`).
fn assert_matches_oracle(model: &RobotModel, response: &ResponseResult) {
    let mut metrics = KinematicsMetrics::new(model);
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

        let mut state = RobotState::new(model);
        state.set_to_default_values();
        for (name, &value) in &case.joint_values {
            state
                .set_variable_position(name, value)
                .unwrap_or_else(|e| panic!("case {case_index}: set {name}: {e}"));
        }
        let posed = state.update();

        let index_full = metrics
            .manipulability_index(&posed, &response.group, false)
            .unwrap_or_else(|e| panic!("case {case_index}: manipulability_index(false): {e}"));
        assert_relative_eq!(
            index_full,
            case.manipulability_index_full,
            epsilon = SCALAR_EPSILON,
            max_relative = SCALAR_MAX_RELATIVE
        );

        let index_translation = metrics
            .manipulability_index(&posed, &response.group, true)
            .unwrap_or_else(|e| panic!("case {case_index}: manipulability_index(true): {e}"));
        assert_relative_eq!(
            index_translation,
            case.manipulability_index_translation,
            epsilon = SCALAR_EPSILON,
            max_relative = SCALAR_MAX_RELATIVE
        );

        let manipulability_full = metrics
            .manipulability(&posed, &response.group, false)
            .unwrap_or_else(|e| panic!("case {case_index}: manipulability(false): {e}"));
        assert_relative_eq!(
            manipulability_full,
            case.manipulability_full,
            epsilon = SCALAR_EPSILON,
            max_relative = SCALAR_MAX_RELATIVE
        );

        let manipulability_translation = metrics
            .manipulability(&posed, &response.group, true)
            .unwrap_or_else(|e| panic!("case {case_index}: manipulability(true): {e}"));
        assert_relative_eq!(
            manipulability_translation,
            case.manipulability_translation,
            epsilon = SCALAR_EPSILON,
            max_relative = SCALAR_MAX_RELATIVE
        );

        let (eigenvalues, eigenvectors) = metrics
            .manipulability_ellipsoid(&posed, &response.group)
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

#[test]
fn panda_kinematics_metrics_matches_the_oracle() {
    let model = build_model();
    let response = load_response("panda_kinematics_metrics_response.json");
    assert_eq!(response.group, "panda_arm");
    assert_matches_oracle(&model, &response);
}

/// Pins `manipulability_index`/`manipulability`'s `columns < 6` branch
/// (P4, round 13 brief): `panda_arm_5dof` has 5 active joints, so
/// `jacobian.cols() == 5 < 6` for every state, forcing the SVD-product
/// path that `panda_arm` (7 DOF) never exercises.
#[test]
fn panda_arm_5dof_kinematics_metrics_matches_the_oracle() {
    let model = build_model_with_5dof_group();
    let response = load_response("panda_arm_5dof_kinematics_metrics_response.json");
    assert_eq!(response.group, "panda_arm_5dof");
    assert_matches_oracle(&model, &response);
}
