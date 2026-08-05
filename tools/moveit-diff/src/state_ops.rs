// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! PORTING-PLAN.md §5 Phase 2's third completion condition — "관절 한계
//! 클램핑, mimic 전파, floating/planar 조인트 보간이 일치" — as a comparison
//! against the C++ oracle.
//!
//! The first two conditions of that phase (FK at `1e-9`, jacobian at `1e-7`)
//! are swept by `tools/ci/verify-oracle-sweep.sh` over random states. This
//! one deliberately is not random-first: clamping and wrap are *boundary*
//! behaviours, and a uniform sample over a joint's range hits `min`, `max`
//! and `±π` with probability zero. Every case here is an enumerated boundary
//! — exactly at a limit, one ULP inside, one ULP outside, a macroscopic step
//! outside, and the wrap point for continuous joints — one case per boundary
//! rather than one case per narrative.
//!
//! # The three clauses, and why each is driven where it is
//!
//! **Clamping** drives `RobotState::enforce_bounds` against
//! `RobotState::enforceBounds()`, over a complete variable vector installed
//! raw on both sides. Raw matters: the vector reaches `enforceBounds` without
//! any mimic propagation having run, so a case can hand in a mimic variable
//! that disagrees with its master and observe what `enforceBounds` does about
//! it.
//!
//! **Mimic propagation** drives `RobotState::set_joint_positions` against
//! `RobotState::setJointPositions`, from the model defaults, writing only
//! joints that mimic nothing. Every mimic variable in the answer therefore
//! got there through `updateMimicJoint` and nothing else.
//!
//! **Interpolation** drives `JointModel::interpolate` against
//! `JointModel::interpolate`, per joint. That is where every type-specific
//! rule lives — the continuous-revolute and planar angle wrap, the floating
//! slerp and its near-identical shortcut. Upstream's `RobotState::interpolate`
//! adds an active-joint loop and a mimic pass on top; this port has no public
//! equivalent of it (`RobotTrajectory::interpolate_into` is a private copy of
//! the same loop), which is reported as a gap rather than papered over by
//! reconstructing the loop here — a harness that reimplements the thing under
//! test is not a comparison.
//!
//! # The clamping/mimic interaction
//!
//! `enforceBounds()` iterates `getActiveJointModels()`, which **excludes**
//! mimic joints, so nothing ever consults a mimic joint's own bounds. The
//! only thing that writes a mimic variable is `updateMimicJoint`, and
//! `enforcePositionBounds(joint)` calls it *only when* the master's own
//! `enforcePositionBounds` returned true. That return is unconditional for
//! revolute (`revolute_joint_model.cpp:218`, `return true` on every path)
//! and change-gated for prismatic (`prismatic_joint_model.cpp:99`, `return
//! false` when already inside). So the same case has opposite outcomes on
//! panda (prismatic master) and pr2 (revolute masters), and a mimic joint can
//! be left outside its own limits by a call named "enforce bounds". Both are
//! measured here rather than assumed: see [`clamp_mimic_cases`] and
//! [`EnforceBoundsResult::out_of_bounds`](crate::protocol::EnforceBoundsResult::out_of_bounds).
//!
//! # Quaternion double cover
//!
//! A floating joint's interpolated rotation comes back as four stored
//! variables, and `q` and `-q` are the same rotation. Comparing only the
//! rotation would accept a port that stores the opposite representative;
//! comparing only the components would report such a port as if it had a
//! numeric error of magnitude 2. Both are computed: the componentwise
//! deviation decides the case (the stored variables are what a `RobotState`
//! keeps, and they feed `distance`, `satisfiesPositionBounds` and the next
//! interpolation), and a case whose rotation blocks are exact negatives is
//! additionally counted and reported under its own name, so a sign-convention
//! divergence can never be read as either agreement or arithmetic noise.

use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::PI;

use moveit_model::RobotModel;
use moveit_state::RobotState;

use crate::Oracle;
use crate::protocol::{JointDetail, ModelInfo, Op, OracleResult};

/// One clause's measured outcome.
pub(crate) struct ClauseResult {
    /// `clamping`, `mimic` or `interpolation`.
    pub(crate) name: &'static str,
    /// How many cases ran.
    pub(crate) cases: usize,
    /// How many disagreed.
    pub(crate) disagreements: usize,
    /// Largest componentwise `|rust - oracle|` seen, over every case.
    pub(crate) worst_deviation: f64,
    /// The case that produced `worst_deviation`.
    pub(crate) worst_label: String,
    /// Interpolation only: disagreements whose rotation blocks are exact
    /// negatives of each other — the same rotation stored with the opposite
    /// quaternion representative. Counted inside `disagreements`, named
    /// separately so it is not read as arithmetic error.
    pub(crate) double_cover: usize,
    /// Formatted disagreements, capped — see [`MAX_REPORTED`].
    pub(crate) failures: Vec<String>,
    /// Cases the enumerator could not build, with the reason. A skipped
    /// case is not a passing case: printed alongside the counts so a clause
    /// that measured less than it looks like says so.
    pub(crate) skipped: Vec<String>,
}

/// How many disagreements are formatted per clause. The count is always
/// exact; only the printed list is capped.
const MAX_REPORTED: usize = 20;

impl ClauseResult {
    fn new(name: &'static str) -> Self {
        Self {
            name,
            cases: 0,
            disagreements: 0,
            worst_deviation: 0.0,
            worst_label: String::new(),
            double_cover: 0,
            failures: Vec::new(),
            skipped: Vec::new(),
        }
    }

    /// Record one case's deviation, and its failure text when it disagreed.
    fn record(&mut self, label: &str, deviation: f64, failure: Option<String>) {
        self.cases += 1;
        if deviation > self.worst_deviation || self.worst_label.is_empty() {
            self.worst_deviation = deviation;
            self.worst_label = label.to_owned();
        }
        if let Some(text) = failure {
            self.disagreements += 1;
            if self.failures.len() < MAX_REPORTED {
                self.failures.push(text);
            }
        }
    }
}

/// Everything the three clauses measured on one robot.
pub(crate) struct Report {
    pub(crate) clauses: Vec<ClauseResult>,
}

impl Report {
    /// Whether every clause agreed.
    pub(crate) fn met(&self) -> bool {
        self.clauses.iter().all(|c| c.disagreements == 0)
    }
}

/// Run all three clauses against `oracle`, comparing with `rust_model`.
///
/// `info` is the oracle's own `model_info`, and every case value is built
/// from it rather than from `rust_model` — the bounds a case is a boundary
/// *of* have to come from the reference, or a port with wrong limits would
/// generate cases at its own wrong limits and agree with itself. (Those
/// limits are separately compared: `verify-oracle-sweep.sh`'s `joint_limits`
/// clause is exactly that check.)
pub(crate) fn run(
    oracle: &mut Oracle,
    rust_model: &RobotModel,
    info: &ModelInfo,
    tol_interpolate: f64,
) -> Result<Report, String> {
    let base = base_positions(info);
    if base.len() != rust_model.variable_count() {
        return Err(format!(
            "built a {}-variable base vector, this port's model has {} variables",
            base.len(),
            rust_model.variable_count()
        ));
    }

    Ok(Report {
        clauses: vec![
            run_clamping(oracle, rust_model, info, &base)?,
            run_mimic(oracle, rust_model, info)?,
            run_interpolation(oracle, rust_model, info, tol_interpolate)?,
        ],
    })
}

// ---- Clause 1: clamping ---------------------------------------------------

/// One `enforce_bounds` case: a complete variable vector, and what it is.
struct ClampCase {
    label: String,
    positions: BTreeMap<String, f64>,
}

fn run_clamping(
    oracle: &mut Oracle,
    rust_model: &RobotModel,
    info: &ModelInfo,
    base: &BTreeMap<String, f64>,
) -> Result<ClauseResult, String> {
    let mut result = ClauseResult::new("clamping");
    let mut cases = clamp_cases(info, base, &mut result.skipped);
    cases.extend(clamp_mimic_cases(info, base, &mut result.skipped));

    let names: Vec<String> = rust_model.variable_names().to_vec();
    for case in cases {
        let expected = match oracle.ask(Op::EnforceBounds {
            positions: case.positions.clone(),
        })? {
            OracleResult::EnforceBounds(r) => r,
            other => return Err(format!("expected enforce_bounds, got {other:?}")),
        };

        let mut state = RobotState::new(rust_model);
        let ordered: Vec<f64> = names.iter().map(|n| case.positions[n]).collect();
        state.set_variable_positions(&ordered);
        state.enforce_bounds();

        let actual: BTreeMap<String, f64> = names
            .iter()
            .cloned()
            .zip(state.positions().iter().copied())
            .collect();
        let (deviation, mismatch) = compare_named(&actual, &expected.enforced);

        // Every joint of the model, active and mimic alike, whose own
        // `satisfies_position_bounds` is false afterwards -- the same set the
        // oracle reports, computed from this port's own predicate.
        let mut out_of_bounds: BTreeSet<String> = BTreeSet::new();
        for joint in rust_model.joint_models() {
            if joint.variable_count() == 0 {
                continue;
            }
            let values = state
                .joint_position(joint.name())
                .map_err(|e| format!("{}: {e}", case.label))?;
            if !joint.satisfies_position_bounds(values, 0.0) {
                out_of_bounds.insert(joint.name().to_owned());
            }
        }
        let expected_oob: BTreeSet<String> = expected.out_of_bounds.iter().cloned().collect();

        let mut failure = None;
        if let Some(text) = mismatch {
            failure = Some(format!("{}: {text}", case.label));
        } else if out_of_bounds != expected_oob {
            failure = Some(format!(
                "{}: positions agree but out-of-bounds joints differ: rust {:?}, oracle {:?}",
                case.label, out_of_bounds, expected_oob
            ));
        }
        result.record(&case.label, deviation, failure);
    }
    Ok(result)
}

/// One case per boundary of every joint variable, each a one-joint edit of
/// `base`.
///
/// Mimic joints are not enumerated here: their variable is only ever written
/// by propagation from a master, so a boundary case on a mimic variable in
/// isolation would be a case about the raw setter. [`clamp_mimic_cases`] is
/// where they are covered, in combination with their master.
fn clamp_cases(
    info: &ModelInfo,
    base: &BTreeMap<String, f64>,
    skipped: &mut Vec<String>,
) -> Vec<ClampCase> {
    let mut cases = Vec::new();
    for detail in &info.joint_details {
        if detail.variable_names.is_empty() || detail.mimic.is_some() {
            continue;
        }
        if detail.type_name == "Floating" {
            cases.extend(floating_clamp_cases(detail, base, skipped));
            continue;
        }
        for (index, variable) in detail.variable_names.iter().enumerate() {
            for (what, value) in variable_boundaries(detail, index, skipped) {
                let mut positions = base.clone();
                positions.insert(variable.clone(), value);
                cases.push(ClampCase {
                    label: format!("clamp/{variable}/{what}"),
                    positions,
                });
            }
        }
    }
    cases
}

/// The boundary values for one scalar variable, chosen by what kind of bound
/// it actually has rather than by its joint's type name.
///
/// Three shapes, and they are not interchangeable:
///
/// - **A finite clamp** (`position_bounded`, both limits finite): revolute
///   and prismatic joints. `enforcePositionBounds` copies the violated limit.
/// - **A wrap** (`!position_bounded`, limits `±π`): a continuous revolute, or
///   a planar joint's `theta`. Both wrap, and they disagree about the
///   endpoints: revolute wraps at `v <= -π || v > π`
///   (`revolute_joint_model.cpp:220`), planar at `!(v >= -π && v <= π)`
///   (`planar_joint_model.cpp:310`) — so `-π` is rewritten to `+π` by one and
///   left alone by the other. That asymmetry is why `at-minus-pi` and
///   `at-plus-pi` are separate cases rather than one "at the wrap point".
/// - **An infinite bound** (`position_bounded` with a non-finite limit): a
///   planar joint's `x`/`y`, a floating joint's translation. There is no
///   boundary to enumerate, so these get magnitude cases only, and that
///   limitation is recorded in `skipped` rather than passed over.
fn variable_boundaries(
    detail: &JointDetail,
    index: usize,
    skipped: &mut Vec<String>,
) -> Vec<(&'static str, f64)> {
    let variable = &detail.variable_names[index];
    let (lo, hi) = detail.bounds[index];
    let bounded = detail.position_bounded[index];

    match (bounded, lo, hi) {
        (true, Some(lo), Some(hi)) if lo.is_finite() && hi.is_finite() => {
            let step = (0.5 * (hi - lo)).max(1e-3);
            vec![
                ("at-min", lo),
                ("min-1ulp-below", next_down(lo)),
                ("min-1ulp-above", next_up(lo)),
                ("below-min", lo - step),
                ("at-max", hi),
                ("max-1ulp-above", next_up(hi)),
                ("max-1ulp-below", next_down(hi)),
                ("above-max", hi + step),
                ("midpoint", 0.5 * (lo + hi)),
            ]
        }
        (false, _, _) => vec![
            ("at-minus-pi", -PI),
            ("at-plus-pi", PI),
            ("minus-pi-1ulp-below", next_down(-PI)),
            ("minus-pi-1ulp-above", next_up(-PI)),
            ("plus-pi-1ulp-below", next_down(PI)),
            ("plus-pi-1ulp-above", next_up(PI)),
            ("two-pi", 2.0 * PI),
            ("minus-two-pi", -2.0 * PI),
            ("three-pi", 3.0 * PI),
            ("minus-three-pi", -3.0 * PI),
            ("zero", 0.0),
        ],
        _ => {
            skipped.push(format!(
                "clamp/{variable}: bounds are ({lo:?}, {hi:?}) — no finite boundary to enumerate, \
                 magnitude cases only"
            ));
            vec![
                ("large-positive", 1e6),
                ("large-negative", -1e6),
                ("zero", 0.0),
            ]
        }
    }
}

/// A floating joint's four rotation variables move together:
/// `enforcePositionBounds` calls `normalizeRotation` on the block as a whole
/// (`floating_joint_model.cpp:212`), so a per-variable boundary would be
/// meaningless. Its translation variables are enumerated normally.
///
/// The cases bracket **two** different epsilons, which is the point of the
/// list: `normalizeRotation` renormalizes once `|‖q‖² − 1| > 100·ε`
/// (`:180`), while `satisfiesPositionBounds` demands `|‖q‖² − 1| ≤ 10·ε`
/// (`:175`). A quaternion between those two is left alone by `enforceBounds`
/// *and* reported out of bounds immediately afterwards — the one case where
/// "enforce" provably does not produce a satisfying state.
fn floating_clamp_cases(
    detail: &JointDetail,
    base: &BTreeMap<String, f64>,
    skipped: &mut Vec<String>,
) -> Vec<ClampCase> {
    let mut cases = Vec::new();
    if detail.variable_names.len() != 7 {
        skipped.push(format!(
            "clamp/{}: floating joint has {} variables, expected 7",
            detail.name,
            detail.variable_names.len()
        ));
        return cases;
    }
    for index in 0..3 {
        for (what, value) in variable_boundaries(detail, index, skipped) {
            let mut positions = base.clone();
            positions.insert(detail.variable_names[index].clone(), value);
            cases.push(ClampCase {
                label: format!("clamp/{}/{what}", detail.variable_names[index]),
                positions,
            });
        }
    }

    let eps = f64::EPSILON;
    let with_norm_sqr = |k: f64| {
        let w = (1.0 + k * eps).sqrt();
        [0.0, 0.0, 0.0, w]
    };
    let half = 0.5f64;
    let quaternions: Vec<(&'static str, [f64; 4])> = vec![
        ("unit", [0.0, 0.0, 0.0, 1.0]),
        ("unit-off-axis", [half, half, half, half]),
        ("scaled-2x", [0.0, 0.0, 0.0, 2.0]),
        ("scaled-half", [0.0, 0.0, 0.0, 0.5]),
        ("all-ones", [1.0, 1.0, 1.0, 1.0]),
        ("zero", [0.0, 0.0, 0.0, 0.0]),
        ("underflowing", [0.0, 0.0, 0.0, 1e-200]),
        ("norm-sqr-plus-5eps", with_norm_sqr(5.0)),
        ("norm-sqr-plus-10eps", with_norm_sqr(10.0)),
        ("norm-sqr-plus-50eps", with_norm_sqr(50.0)),
        ("norm-sqr-plus-100eps", with_norm_sqr(100.0)),
        ("norm-sqr-plus-200eps", with_norm_sqr(200.0)),
        ("norm-sqr-minus-50eps", with_norm_sqr(-50.0)),
        ("norm-sqr-minus-200eps", with_norm_sqr(-200.0)),
    ];
    for (what, q) in quaternions {
        let mut positions = base.clone();
        for (offset, value) in q.iter().enumerate() {
            positions.insert(detail.variable_names[3 + offset].clone(), *value);
        }
        cases.push(ClampCase {
            label: format!("clamp/{}/rotation/{what}", detail.name),
            positions,
        });
    }
    cases
}

/// The clamping × mimic interaction, one case per combination of a master
/// boundary and a mimic-variable state.
///
/// Neither half in isolation covers it. A mimic joint is not active, so
/// `enforceBounds` never clamps it; its variable is rewritten only when the
/// master's `enforcePositionBounds` reports a change, and *that* is what
/// differs by master type. So the grid is (master at/inside/outside each of
/// its limits) × (mimic consistent / inconsistent but within its own bounds /
/// outside its own bounds), and the interesting cell is the one where the
/// master needs no clamping and the mimic is inconsistent: the mimic survives
/// unrepaired behind a prismatic master and is overwritten behind a revolute
/// one.
fn clamp_mimic_cases(
    info: &ModelInfo,
    base: &BTreeMap<String, f64>,
    skipped: &mut Vec<String>,
) -> Vec<ClampCase> {
    let mut cases = Vec::new();
    for detail in &info.joint_details {
        let Some(mimic) = &detail.mimic else { continue };
        let Some(master) = info.joint_details.iter().find(|d| d.name == mimic.joint) else {
            skipped.push(format!(
                "clamp-mimic/{}: mimics {}, which model_info does not list",
                detail.name, mimic.joint
            ));
            continue;
        };
        if master.variable_names.len() != 1 || detail.variable_names.len() != 1 {
            // `updateMimicJoint` writes `getFirstVariableIndex()` only, so a
            // multi-variable mimic is not a relationship this grid describes.
            skipped.push(format!(
                "clamp-mimic/{}: {} master variables and {} mimic variables; \
                 updateMimicJoint writes only the first",
                detail.name,
                master.variable_names.len(),
                detail.variable_names.len()
            ));
            continue;
        }
        let master_variable = &master.variable_names[0];
        let mimic_variable = &detail.variable_names[0];
        let master_values = variable_boundaries(master, 0, skipped);
        let (mimic_lo, mimic_hi) = detail.bounds[0];
        let outside = match mimic_hi {
            Some(hi) if hi.is_finite() => hi + 1.0,
            _ => 1e6,
        };

        for (what, master_value) in master_values {
            let consistent = mimic.multiplier * master_value + mimic.offset;
            // Inconsistent but still inside the mimic's own bounds: a quarter
            // of the way across its range from whichever end is further from
            // `consistent`, so the two are never accidentally equal.
            let inconsistent = match (mimic_lo, mimic_hi) {
                (Some(lo), Some(hi)) if lo.is_finite() && hi.is_finite() => {
                    let quarter = lo + 0.25 * (hi - lo);
                    let three_quarter = lo + 0.75 * (hi - lo);
                    if (quarter - consistent).abs() > (three_quarter - consistent).abs() {
                        quarter
                    } else {
                        three_quarter
                    }
                }
                _ => consistent + 1.0,
            };
            for (state, mimic_value) in [
                ("mimic-consistent", consistent),
                ("mimic-inconsistent-in-bounds", inconsistent),
                ("mimic-outside-own-bounds", outside),
            ] {
                let mut positions = base.clone();
                positions.insert(master_variable.clone(), master_value);
                positions.insert(mimic_variable.clone(), mimic_value);
                cases.push(ClampCase {
                    label: format!("clamp-mimic/{mimic_variable}/master-{what}/{state}"),
                    positions,
                });
            }
        }
    }
    cases
}

// ---- Clause 2: mimic propagation ------------------------------------------

fn run_mimic(
    oracle: &mut Oracle,
    rust_model: &RobotModel,
    info: &ModelInfo,
) -> Result<ClauseResult, String> {
    let mut result = ClauseResult::new("mimic");
    let cases = mimic_cases(info, &mut result.skipped);
    if cases.is_empty() {
        result
            .skipped
            .push("mimic: this robot has no mimic joints — nothing to propagate".to_owned());
        return Ok(result);
    }

    let names: Vec<String> = rust_model.variable_names().to_vec();
    for (label, joint_positions) in cases {
        let expected = match oracle.ask(Op::MimicPropagate {
            joint_positions: joint_positions.clone(),
        })? {
            OracleResult::MimicPropagate(r) => r,
            other => return Err(format!("expected mimic_propagate, got {other:?}")),
        };

        let mut state = RobotState::new(rust_model);
        state.set_to_default_values();
        for (joint, values) in &joint_positions {
            state
                .set_joint_positions(joint, values)
                .map_err(|e| format!("{label}: {e}"))?;
        }
        let actual: BTreeMap<String, f64> = names
            .iter()
            .cloned()
            .zip(state.positions().iter().copied())
            .collect();

        let (deviation, mismatch) = compare_named(&actual, &expected.propagated);
        result.record(&label, deviation, mismatch.map(|t| format!("{label}: {t}")));
    }
    Ok(result)
}

/// One case per boundary of every joint that at least one mimic joint
/// follows, plus one case that writes them all at once.
///
/// The out-of-range master values are deliberate: `setJointPositions` does no
/// clamping, so a master outside its limits must propagate a mimic value
/// outside *its* limits. That is the behaviour, and a port that quietly
/// clamped on the way through would look identical on the in-range cases.
fn mimic_cases(
    info: &ModelInfo,
    skipped: &mut Vec<String>,
) -> Vec<(String, BTreeMap<String, Vec<f64>>)> {
    let masters: BTreeSet<&str> = info
        .joint_details
        .iter()
        .filter_map(|d| d.mimic.as_ref())
        .map(|m| m.joint.as_str())
        .collect();

    let mut cases = Vec::new();
    let mut all: BTreeMap<String, Vec<f64>> = BTreeMap::new();
    for name in &masters {
        let Some(master) = info.joint_details.iter().find(|d| &d.name == name) else {
            skipped.push(format!("mimic/{name}: model_info does not list this joint"));
            continue;
        };
        if master.variable_names.len() != 1 {
            skipped.push(format!(
                "mimic/{name}: master has {} variables; updateMimicJoint writes only the first",
                master.variable_names.len()
            ));
            continue;
        }
        for (what, value) in variable_boundaries(master, 0, skipped) {
            cases.push((
                format!("mimic/{name}/{what}"),
                BTreeMap::from([((*name).to_owned(), vec![value])]),
            ));
            if what == "at-max" || what == "at-plus-pi" {
                all.insert((*name).to_owned(), vec![value]);
            }
        }
    }
    if all.len() > 1 {
        cases.push(("mimic/all-masters-at-max".to_owned(), all));
    }
    cases
}

// ---- Clause 3: interpolation ----------------------------------------------

/// One `interpolate` case.
struct InterpCase {
    label: String,
    joint: String,
    from: Vec<f64>,
    to: Vec<f64>,
}

/// The `t` grid every from/to pair is evaluated on.
///
/// `0` and `1` are the endpoint identities; `1e-9` and `1 - 1e-9` are just
/// inside them, where a branch keyed on `t` (the diff-drive turn/drive/turn
/// split) changes behaviour without the endpoint value changing; the rest
/// sample the interior including the slerp midpoint.
const T_GRID: [f64; 7] = [0.0, 1e-9, 0.25, 0.5, 0.75, 1.0 - 1e-9, 1.0];

fn run_interpolation(
    oracle: &mut Oracle,
    rust_model: &RobotModel,
    info: &ModelInfo,
    tol: f64,
) -> Result<ClauseResult, String> {
    let mut result = ClauseResult::new("interpolation");
    let cases = interpolation_cases(info, &mut result.skipped);

    for case in cases {
        let joint = rust_model
            .joint_model(&case.joint)
            .map_err(|e| format!("{}: {e}", case.label))?;
        let rotation_block = (joint.type_name() == "Floating").then_some(3..7);

        for t in T_GRID {
            let label = format!("{}/t={t}", case.label);
            let expected = match oracle.ask(Op::Interpolate {
                joint: case.joint.clone(),
                from: case.from.clone(),
                to: case.to.clone(),
                t,
            })? {
                OracleResult::Interpolate(r) => r,
                other => return Err(format!("expected interpolate, got {other:?}")),
            };

            let mut actual = vec![0.0; joint.variable_count()];
            joint.interpolate(&case.from, &case.to, t, &mut actual);

            if actual.len() != expected.interpolated.len() {
                return Err(format!(
                    "{label}: this port produced {} values, the oracle {}",
                    actual.len(),
                    expected.interpolated.len()
                ));
            }
            let deviation = actual
                .iter()
                .zip(&expected.interpolated)
                .map(|(a, b)| (a - b).abs())
                .fold(0.0f64, f64::max);

            // NaN named rather than caught by negating the comparison: a NaN
            // deviation means one side produced a NaN coefficient, which is a
            // disagreement of the most interesting kind, and `deviation > tol`
            // alone would report it as agreement.
            let mut failure = None;
            if deviation.is_nan() || deviation > tol {
                let antipodal = rotation_block.clone().is_some_and(|block| {
                    is_antipodal(&actual[block.clone()], &expected.interpolated[block], tol)
                });
                if antipodal {
                    result.double_cover += 1;
                }
                let angle = rotation_block.clone().map(|block| {
                    rotation_angle(&actual[block.clone()], &expected.interpolated[block])
                });
                failure = Some(format!(
                    "{label}: max|Δ| {deviation:.6e}{}{}\n      rust   {:?}\n      oracle {:?}",
                    angle.map_or(String::new(), |a| format!(", rotation angle {a:.6e} rad")),
                    if antipodal {
                        " [DOUBLE COVER: rotation blocks are exact negatives]"
                    } else {
                        ""
                    },
                    actual,
                    expected.interpolated,
                ));
            }
            result.record(&label, deviation, failure);
        }
    }
    Ok(result)
}

/// One from/to pair per interpolation branch of every joint type present.
///
/// Mimic joints are included: `JointModel::interpolate` is a property of the
/// joint's own type and knows nothing about mimicking, so excluding them
/// would drop real revolute and prismatic coverage for no reason.
fn interpolation_cases(info: &ModelInfo, skipped: &mut Vec<String>) -> Vec<InterpCase> {
    let mut cases = Vec::new();
    for detail in &info.joint_details {
        if detail.variable_names.is_empty() {
            continue;
        }
        let name = &detail.name;
        let pairs: Vec<(&'static str, Vec<f64>, Vec<f64>)> = match detail.type_name.as_str() {
            "Revolute" | "Prismatic" => {
                let (lo, hi) = detail.bounds[0];
                match (detail.position_bounded[0], lo, hi) {
                    (true, Some(lo), Some(hi)) if lo.is_finite() && hi.is_finite() => {
                        let mid = 0.5 * (lo + hi);
                        vec![
                            ("min-to-max", vec![lo], vec![hi]),
                            ("max-to-min", vec![hi], vec![lo]),
                            ("same", vec![mid], vec![mid]),
                            ("mid-to-max", vec![mid], vec![hi]),
                        ]
                    }
                    // A continuous revolute. Every pair is chosen by where
                    // `to - from` falls relative to `±π`, because that is the
                    // branch `revolute_joint_model.cpp:139` keys on.
                    _ => vec![
                        ("diff-eq-pi", vec![-PI / 2.0], vec![PI / 2.0]),
                        (
                            "diff-1ulp-over-pi",
                            vec![-PI / 2.0],
                            vec![next_up(PI / 2.0)],
                        ),
                        (
                            "diff-1ulp-under-pi",
                            vec![-PI / 2.0],
                            vec![next_down(PI / 2.0)],
                        ),
                        ("diff-eq-minus-pi", vec![PI / 2.0], vec![-PI / 2.0]),
                        (
                            "diff-1ulp-under-minus-pi",
                            vec![PI / 2.0],
                            vec![next_down(-PI / 2.0)],
                        ),
                        ("endpoints", vec![-PI], vec![PI]),
                        ("wrap-positive", vec![3.0], vec![-3.0]),
                        ("wrap-negative", vec![-3.0], vec![3.0]),
                        ("same", vec![0.7], vec![0.7]),
                    ],
                }
            }
            "Planar" => vec![
                (
                    "theta-diff-eq-pi",
                    vec![0.0, 0.0, -PI / 2.0],
                    vec![1.0, 2.0, PI / 2.0],
                ),
                (
                    "theta-diff-1ulp-over-pi",
                    vec![0.0, 0.0, -PI / 2.0],
                    vec![1.0, 2.0, next_up(PI / 2.0)],
                ),
                (
                    "theta-wrap-positive",
                    vec![-1.0, -2.0, 3.0],
                    vec![1.0, 2.0, -3.0],
                ),
                ("theta-endpoints", vec![0.0, 0.0, -PI], vec![0.0, 0.0, PI]),
                ("same", vec![1.0, 2.0, 0.5], vec![1.0, 2.0, 0.5]),
                (
                    "translation-only",
                    vec![-1e3, 1e3, 0.25],
                    vec![1e3, -1e3, 0.25],
                ),
            ],
            "Floating" => floating_pairs(),
            "Fixed" => continue,
            other => {
                skipped.push(format!(
                    "interpolate/{name}: unrecognized joint type {other:?}"
                ));
                continue;
            }
        };
        for (what, from, to) in pairs {
            if from.len() != detail.variable_names.len() {
                skipped.push(format!(
                    "interpolate/{name}/{what}: built {} values for a {}-variable joint",
                    from.len(),
                    detail.variable_names.len()
                ));
                continue;
            }
            cases.push(InterpCase {
                label: format!("interpolate/{name}/{what}"),
                joint: name.clone(),
                from,
                to,
            });
        }
    }
    cases
}

/// From/to pairs for a floating joint, chosen by which branch of
/// `floating_joint_model.cpp:136` they take.
///
/// Upstream skips the slerp entirely when `Σ|from_i − to_i|` over the four
/// rotation components is at most `f64::EPSILON`, and slerps the *raw*
/// quaternions otherwise — no normalization on the way in. So the branch
/// boundary is a sum of absolute differences, not an angle, and
/// `within-eps`/`just-over-eps` sit on either side of it. `antipodal` and
/// `double-cover-long-way` are where Eigen's own sign handling (`if (d < 0)
/// scale1 = -scale1`) and nalgebra's (`if dot < 0 { slerp against -other }`)
/// have to agree about which of two identical rotations comes back.
fn floating_pairs() -> Vec<(&'static str, Vec<f64>, Vec<f64>)> {
    let identity = vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0];
    let quarter = (PI / 4.0).sin();
    let z90 = vec![0.0, 0.0, 0.0, 0.0, 0.0, quarter, quarter];
    let z180 = vec![0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0];
    let negate = |q: &[f64]| -> Vec<f64> {
        let mut out = q.to_vec();
        for value in &mut out[3..7] {
            *value = -*value;
        }
        out
    };
    let scale = |q: &[f64], k: f64| -> Vec<f64> {
        let mut out = q.to_vec();
        for value in &mut out[3..7] {
            *value *= k;
        }
        out
    };
    let nudge_w = |q: &[f64], d: f64| -> Vec<f64> {
        let mut out = q.to_vec();
        out[6] += d;
        out
    };

    vec![
        ("same", identity.clone(), identity.clone()),
        (
            "within-eps",
            identity.clone(),
            nudge_w(&identity, f64::EPSILON / 4.0),
        ),
        (
            "just-over-eps",
            identity.clone(),
            nudge_w(&identity, 4.0 * f64::EPSILON),
        ),
        ("ninety-deg", identity.clone(), z90.clone()),
        ("one-eighty-deg", identity.clone(), z180.clone()),
        ("antipodal", z90.clone(), negate(&z90)),
        ("double-cover-long-way", identity.clone(), negate(&z90)),
        ("double-cover-one-eighty", identity.clone(), negate(&z180)),
        ("unnormalized-both", scale(&identity, 2.0), scale(&z90, 2.0)),
        ("near-antipodal", z90.clone(), nudge_w(&negate(&z90), 1e-12)),
        (
            "translation",
            vec![-1.0, 2.0, -3.0, 0.0, 0.0, 0.0, 1.0],
            vec![4.0, -5.0, 6.0, 0.0, 0.0, 0.0, 1.0],
        ),
    ]
}

// ---- Comparison helpers ---------------------------------------------------

/// Componentwise comparison of two name-keyed vectors.
///
/// Returns the largest `|actual − expected|` and, when the two disagree
/// anywhere, the text naming the worst variable. The threshold is exact
/// equality on purpose: clamping copies a limit, wrapping is `fmod` plus one
/// conditional add, and mimic propagation is `factor · v + offset`. Both
/// sides run the identical IEEE operations on identical inputs, so any
/// difference at all is a difference in *what* was computed rather than in
/// how precisely. A non-zero deviation here is a finding, and the number is
/// reported so it can be read as one.
fn compare_named(
    actual: &BTreeMap<String, f64>,
    expected: &BTreeMap<String, f64>,
) -> (f64, Option<String>) {
    let mut worst = 0.0f64;
    let mut worst_variable = String::new();
    for (name, expected_value) in expected {
        let Some(actual_value) = actual.get(name) else {
            return (
                f64::INFINITY,
                Some(format!("this port has no variable named {name:?}")),
            );
        };
        let deviation = (actual_value - expected_value).abs();
        if deviation > worst || (deviation > 0.0 && worst_variable.is_empty()) {
            worst = deviation;
            worst_variable = name.clone();
        }
        if actual_value != expected_value && worst_variable.is_empty() {
            worst_variable = name.clone();
        }
    }
    if actual.len() != expected.len() {
        return (
            worst,
            Some(format!(
                "this port reported {} variables, the oracle {}",
                actual.len(),
                expected.len()
            )),
        );
    }
    if worst_variable.is_empty() {
        return (worst, None);
    }
    let text = format!(
        "{worst_variable}: rust {:?}, oracle {:?} (Δ {worst:.6e})",
        actual[&worst_variable], expected[&worst_variable]
    );
    (worst, Some(text))
}

/// Whether two quaternion blocks are the same rotation stored with opposite
/// signs — the double-cover case, which is a disagreement about the stored
/// variables and *not* a disagreement about the rotation.
fn is_antipodal(a: &[f64], b: &[f64], tol: f64) -> bool {
    a.len() == b.len()
        && a.iter()
            .zip(b)
            .all(|(x, y)| (x + y).abs() <= tol.max(1e-12))
        && a.iter().any(|x| x.abs() > 1e-12)
}

/// The angle between the rotations two quaternion blocks represent, in
/// radians, insensitive to sign. `NaN`-free for a zero-norm block: that
/// returns `0.0`, since neither side then represents a rotation at all.
fn rotation_angle(a: &[f64], b: &[f64]) -> f64 {
    let norm = |q: &[f64]| q.iter().map(|v| v * v).sum::<f64>().sqrt();
    let (na, nb) = (norm(a), norm(b));
    if na == 0.0 || nb == 0.0 {
        return 0.0;
    }
    let dot: f64 = a.iter().zip(b).map(|(x, y)| x * y).sum::<f64>() / (na * nb);
    2.0 * dot.abs().min(1.0).acos()
}

/// The base variable vector every clamping case is a one-joint edit of:
/// each variable at the midpoint of its own bounds, so the vector satisfies
/// every joint's limits before a case moves one of them.
///
/// Built from the oracle's `model_info`, not from this port's model — see
/// [`run`]. Mimic variables are derived from their master's base value with
/// the wire's own multiplier and offset, and only for the joint's first
/// variable, which is all `updateMimicJoint` writes.
fn base_positions(info: &ModelInfo) -> BTreeMap<String, f64> {
    let mut base = BTreeMap::new();
    for detail in &info.joint_details {
        for (index, variable) in detail.variable_names.iter().enumerate() {
            base.insert(variable.clone(), base_value(detail, index));
        }
    }
    for detail in &info.joint_details {
        let Some(mimic) = &detail.mimic else { continue };
        let Some(master) = info.joint_details.iter().find(|d| d.name == mimic.joint) else {
            continue;
        };
        if master.variable_names.is_empty() || detail.variable_names.is_empty() {
            continue;
        }
        let master_value = base[&master.variable_names[0]];
        base.insert(
            detail.variable_names[0].clone(),
            mimic.multiplier * master_value + mimic.offset,
        );
    }
    base
}

/// One variable's in-bounds starting value.
///
/// A floating joint's rotation block is special-cased to the identity
/// quaternion rather than taken per variable: the per-variable midpoint of
/// `[-1, 1]` is zero on all four components, which is the zero quaternion —
/// a state `satisfiesPositionBounds` rejects, so every case built on it would
/// start out of bounds for a reason unrelated to the case.
fn base_value(detail: &JointDetail, index: usize) -> f64 {
    if detail.type_name == "Floating" && detail.variable_names.len() == 7 {
        return if index == 6 { 1.0 } else { 0.0 };
    }
    match detail.bounds[index] {
        (Some(lo), Some(hi)) if lo.is_finite() && hi.is_finite() => 0.5 * (lo + hi),
        _ => 0.0,
    }
}

/// The next representable `f64` toward `+∞`.
///
/// Hand-rolled rather than `f64::next_up`, which this workspace's MSRV does
/// not have. Both helpers exist so "just outside a limit" can mean *one ULP*
/// outside — the tightest case a clamp can be asked to get right, and the one
/// a scale-relative epsilon would miss on a limit like `0.0`.
fn next_up(v: f64) -> f64 {
    if v.is_nan() || v == f64::INFINITY {
        return v;
    }
    if v == 0.0 {
        return f64::from_bits(1);
    }
    if v > 0.0 {
        f64::from_bits(v.to_bits() + 1)
    } else {
        f64::from_bits(v.to_bits() - 1)
    }
}

/// The next representable `f64` toward `−∞`. See [`next_up`].
fn next_down(v: f64) -> f64 {
    if v.is_nan() || v == f64::NEG_INFINITY {
        return v;
    }
    if v == 0.0 {
        return -f64::from_bits(1);
    }
    if v > 0.0 {
        f64::from_bits(v.to_bits() - 1)
    } else {
        f64::from_bits(v.to_bits() + 1)
    }
}
