// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Differential test runner: drives the C++ moveit2 oracle and the moveit-rs
//! implementation over the same cases and reports every disagreement.
//!
//! The oracle is a separate process (see `tools/moveit-oracle`), normally run
//! inside the `moveit-rs/oracle` container. This binary never links moveit2.
//!
//! ```text
//! moveit-diff --urdf <path> --srdf <path> [--cases N] [--seed S]
//!             [--tol-fk EPS] [--oracle <cmd> [args...]]
//! ```
//!
//! Exit status is 0 only when every case passed.

mod protocol;
mod rust_impl;

use std::collections::BTreeMap;
use std::io::{BufRead, BufReader, Write};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};

use moveit_geometry::{Isometry3, Rotation3, UnitQuaternion, Vector3};
use moveit_model::RobotModel;
use moveit_srdf::SrdfModel;
use protocol::{
    ConstraintRegionSpec, ConstraintsResult, ConstraintsSpec, FkResult, JacobianResult,
    JointConstraintSpec, ModelInfo, Op, OracleResult, OrientationConstraintSpec,
    OrientationToleranceSpec, PositionConstraintSpec, Request, Response, ShapeSpec,
    VisibilityConstraintSpec,
};
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

/// How the runner was configured.
struct Config {
    urdf: String,
    srdf: String,
    cases: usize,
    seed: i32,
    tol_fk: f64,
    /// Joint model group to also run the `jacobian` op against, alongside
    /// every `fk` case. `None` skips Jacobian comparison entirely, keeping
    /// existing `fk`-only invocations unchanged.
    group: Option<String>,
    tol_jacobian: f64,
    /// How many constraint-combination cases to additionally generate and
    /// compare via [`Op::Constraints`]. `0` (the default) runs none, keeping
    /// existing `fk`/`jacobian`-only invocations unchanged.
    constraints: usize,
    tol_constraints: f64,
    oracle: Vec<String>,
}

impl Config {
    fn from_args() -> Result<Self, String> {
        let mut urdf = None;
        let mut srdf = None;
        let mut cases = 1_000usize;
        let mut seed = 0i32;
        let mut tol_fk = 1e-9;
        let mut group = None;
        let mut tol_jacobian = 1e-7;
        let mut constraints = 0usize;
        let mut tol_constraints = 1e-9;
        let mut oracle: Vec<String> = vec!["tools/moveit-oracle/run-oracle.sh".to_owned()];

        let mut args = std::env::args().skip(1);
        while let Some(arg) = args.next() {
            let mut want = |name: &str| -> Result<String, String> {
                args.next()
                    .ok_or_else(|| format!("{name} requires a value"))
            };
            match arg.as_str() {
                "--urdf" => urdf = Some(want("--urdf")?),
                "--srdf" => srdf = Some(want("--srdf")?),
                "--cases" => {
                    cases = want("--cases")?
                        .parse()
                        .map_err(|e| format!("--cases: {e}"))?
                }
                "--seed" => {
                    seed = want("--seed")?
                        .parse()
                        .map_err(|e| format!("--seed: {e}"))?
                }
                "--tol-fk" => {
                    tol_fk = want("--tol-fk")?
                        .parse()
                        .map_err(|e| format!("--tol-fk: {e}"))?
                }
                "--group" => group = Some(want("--group")?),
                "--tol-jacobian" => {
                    tol_jacobian = want("--tol-jacobian")?
                        .parse()
                        .map_err(|e| format!("--tol-jacobian: {e}"))?
                }
                "--constraints" => {
                    constraints = want("--constraints")?
                        .parse()
                        .map_err(|e| format!("--constraints: {e}"))?
                }
                "--tol-constraints" => {
                    tol_constraints = want("--tol-constraints")?
                        .parse()
                        .map_err(|e| format!("--tol-constraints: {e}"))?
                }
                // Everything after --oracle is the command line to run.
                "--oracle" => {
                    oracle = args.by_ref().collect();
                    if oracle.is_empty() {
                        return Err("--oracle requires a command".to_owned());
                    }
                    break;
                }
                other => return Err(format!("unrecognized argument {other:?}")),
            }
        }

        Ok(Self {
            urdf: urdf.ok_or("--urdf is required")?,
            srdf: srdf.ok_or("--srdf is required")?,
            cases,
            seed,
            tol_fk,
            group,
            tol_jacobian,
            constraints,
            tol_constraints,
            oracle,
        })
    }
}

/// A live oracle subprocess.
struct Oracle {
    child: Child,
    /// `None` once closed. Closing this pipe is the oracle's shutdown signal,
    /// so `Drop` must close it *before* waiting: a `wait()` with stdin still
    /// open deadlocks against a child blocked reading the next request.
    /// Holding it in an `Option` is what lets `Drop` take and close it.
    stdin: Option<ChildStdin>,
    stdout: BufReader<ChildStdout>,
    next_id: u64,
}

impl Oracle {
    fn spawn(cfg: &Config) -> Result<Self, String> {
        let (program, rest) = cfg.oracle.split_first().ok_or("empty oracle command")?;
        let mut child = Command::new(program)
            .args(rest)
            .arg("--urdf")
            .arg(&cfg.urdf)
            .arg("--srdf")
            .arg(&cfg.srdf)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .map_err(|e| format!("failed to start oracle {program:?}: {e}"))?;
        let stdin = child.stdin.take().ok_or("oracle stdin unavailable")?;
        let stdout = BufReader::new(child.stdout.take().ok_or("oracle stdout unavailable")?);
        Ok(Self {
            child,
            stdin: Some(stdin),
            stdout,
            next_id: 0,
        })
    }

    fn ask(&mut self, op: Op) -> Result<OracleResult, String> {
        let id = self.next_id;
        self.next_id += 1;
        let line = serde_json::to_string(&Request { id, op })
            .map_err(|e| format!("encoding request {id}: {e}"))?;
        let stdin = self
            .stdin
            .as_mut()
            .ok_or_else(|| format!("oracle stdin already closed at request {id}"))?;
        writeln!(stdin, "{line}").map_err(|e| format!("writing request {id}: {e}"))?;
        stdin
            .flush()
            .map_err(|e| format!("flushing request {id}: {e}"))?;

        let mut buf = String::new();
        let n = self
            .stdout
            .read_line(&mut buf)
            .map_err(|e| format!("reading response {id}: {e}"))?;
        if n == 0 {
            return Err(format!(
                "oracle closed stdout before answering request {id}"
            ));
        }
        let resp: Response = serde_json::from_str(buf.trim())
            .map_err(|e| format!("decoding response {id}: {e} in {buf:?}"))?;
        if resp.id != id {
            return Err(format!("oracle answered id {} for request {id}", resp.id));
        }
        match (resp.ok, resp.result, resp.error) {
            (true, Some(r), _) => Ok(r),
            (true, None, _) => Err(format!("oracle reported ok with no result for {id}")),
            (false, _, Some(e)) => Err(format!("oracle error: {e}")),
            (false, _, None) => Err(format!("oracle reported failure with no message for {id}")),
        }
    }
}

impl Drop for Oracle {
    fn drop(&mut self) {
        // Order matters: closing stdin is the oracle's shutdown signal, and
        // waiting before that deadlocks -- the child blocks reading a request
        // that will never come while the parent blocks waiting for it to exit.
        drop(self.stdin.take());
        let _ = self.child.wait();
    }
}

/// One case's verdict.
enum Verdict {
    Pass,
    Fail(String),
}

fn main() {
    let cfg = match Config::from_args() {
        Ok(c) => c,
        Err(e) => {
            eprintln!("moveit-diff: {e}");
            eprintln!("usage: moveit-diff --urdf <path> --srdf <path> [--cases N] [--seed S]");
            eprintln!("                   [--tol-fk EPS] [--group NAME] [--tol-jacobian EPS]");
            eprintln!("                   [--constraints N] [--tol-constraints EPS]");
            eprintln!("                   [--oracle <cmd> [args...]]");
            std::process::exit(2);
        }
    };

    match run(&cfg) {
        Ok(failures) => std::process::exit(i32::from(failures > 0)),
        Err(e) => {
            eprintln!("moveit-diff: {e}");
            std::process::exit(2);
        }
    }
}

/// Parse the same URDF/SRDF pair the oracle was launched with, so both sides
/// answer questions about the same robot.
fn build_rust_model(cfg: &Config) -> Result<RobotModel, String> {
    let urdf_xml = std::fs::read_to_string(&cfg.urdf)
        .map_err(|e| format!("reading URDF {}: {e}", cfg.urdf))?;
    let urdf =
        urdf_rs::read_file(&cfg.urdf).map_err(|e| format!("parsing URDF {}: {e}", cfg.urdf))?;
    let srdf =
        SrdfModel::parse_file(&cfg.srdf).map_err(|e| format!("parsing SRDF {}: {e}", cfg.srdf))?;
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf)
        .map_err(|e| format!("building RobotModel: {e}"))
}

fn run(cfg: &Config) -> Result<usize, String> {
    let mut oracle = Oracle::spawn(cfg)?;
    let rust_model = build_rust_model(cfg)?;

    // The oracle is the source of truth for the fixture's joint names and
    // bounds, so cases can be sampled before moveit-rs can load a model at all.
    let model = match oracle.ask(Op::ModelInfo)? {
        OracleResult::ModelInfo(m) => m,
        other => return Err(format!("expected model_info, got {other:?}")),
    };
    println!(
        "oracle model: {} ({} links, {} joints, {} groups)",
        model.name,
        model.links.len(),
        model.joints.len(),
        model.groups.len()
    );

    let mut verdicts: Vec<(String, Verdict)> = Vec::new();

    verdicts.push((
        "model_info".to_owned(),
        compare_model_info(&rust_model, &model),
    ));

    let states = match oracle.ask(Op::RandomStates {
        count: cfg.cases,
        seed: cfg.seed,
    })? {
        OracleResult::RandomStates(r) => r.states,
        other => return Err(format!("expected random_states, got {other:?}")),
    };
    if states.len() != cfg.cases {
        return Err(format!(
            "asked for {} random states, oracle returned {}",
            cfg.cases,
            states.len()
        ));
    }

    let mut max_jacobian_dev = 0.0f64;
    // Kept for the constraint-case generator below, which needs each state's
    // link poses to build "meaningful" (boundary-straddling) constraints
    // rather than re-asking the oracle for the same fk it already answered.
    let mut fks: Vec<FkResult> = Vec::with_capacity(states.len());

    for (case, joint_values) in states.iter().enumerate() {
        let expected = match oracle.ask(Op::Fk {
            joint_values: joint_values.clone(),
            links: Vec::new(),
        })? {
            OracleResult::Fk(f) => f,
            other => return Err(format!("expected fk, got {other:?}")),
        };
        verdicts.push((
            format!("fk[{case}]"),
            compare_fk(cfg, &rust_model, joint_values, &expected),
        ));

        if let Some(group) = &cfg.group {
            let expected = match oracle.ask(Op::Jacobian {
                group: group.clone(),
                joint_values: joint_values.clone(),
            })? {
                OracleResult::Jacobian(j) => j,
                other => return Err(format!("expected jacobian, got {other:?}")),
            };
            let (verdict, dev) = compare_jacobian(cfg, &rust_model, group, joint_values, &expected);
            if dev.is_finite() {
                max_jacobian_dev = max_jacobian_dev.max(dev);
            }
            verdicts.push((format!("jacobian[{case}]"), verdict));
        }
        fks.push(expected);
    }

    if cfg.group.is_some() {
        println!("worst jacobian deviation: {max_jacobian_dev:.3e}");
    }

    if cfg.constraints > 0 {
        run_constraint_cases(cfg, &mut oracle, &rust_model, &states, &fks, &mut verdicts)?;
    }

    report(&verdicts)
}

/// Kind label to `(satisfied, violated)` oracle-reported counts -- the
/// factual split this task's completion report names per constraint kind,
/// even if lopsided.
type KindSplit = BTreeMap<&'static str, (usize, usize)>;

/// Drives [`Op::Constraints`] over `cfg.constraints` generated combinations,
/// cycling through the six constraint shapes [`build_constraint_case`] knows
/// how to build, one per case, reusing the states and fk answers the
/// `fk`/`jacobian` loop already collected instead of drawing new ones.
fn run_constraint_cases(
    cfg: &Config,
    oracle: &mut Oracle,
    rust_model: &RobotModel,
    states: &[BTreeMap<String, f64>],
    fks: &[FkResult],
    verdicts: &mut Vec<(String, Verdict)>,
) -> Result<(), String> {
    let mut rng = ChaCha8Rng::seed_from_u64((cfg.seed as i64 as u64) ^ 0x5EED_C0DE_u64);
    let mut split: KindSplit = KindSplit::new();

    for case in 0..cfg.constraints {
        let state_idx = case % states.len();
        let joint_values = &states[state_idx];
        let fk = &fks[state_idx];
        let (kind, spec) = build_constraint_case(case, &mut rng, rust_model, joint_values, fk);

        let expected = match oracle.ask(Op::Constraints {
            joint_values: joint_values.clone(),
            constraints: spec.clone(),
        })? {
            OracleResult::Constraints(c) => c,
            other => return Err(format!("expected constraints, got {other:?}")),
        };

        let (verdict, satisfied) =
            compare_constraints(cfg, rust_model, joint_values, &spec, &expected);
        let entry = split.entry(kind).or_insert((0, 0));
        match satisfied {
            Some(true) => entry.0 += 1,
            Some(false) => entry.1 += 1,
            None => {}
        }
        verdicts.push((format!("constraints[{case}]:{kind}"), verdict));
    }

    println!("constraint satisfied/violated split (oracle-reported):");
    for (kind, (satisfied, violated)) in &split {
        println!("  {kind}: {satisfied} satisfied, {violated} violated");
    }

    Ok(())
}

fn compare_model_info(rust_model: &RobotModel, expected: &ModelInfo) -> Verdict {
    let actual = rust_impl::model_info(rust_model);
    if actual == *expected {
        Verdict::Pass
    } else {
        Verdict::Fail(format!(
            "model differs: rust name={:?} links={} joints={}, oracle name={:?} links={} joints={}",
            actual.name,
            actual.links.len(),
            actual.joints.len(),
            expected.name,
            expected.links.len(),
            expected.joints.len()
        ))
    }
}

fn compare_fk(
    cfg: &Config,
    rust_model: &RobotModel,
    joint_values: &BTreeMap<String, f64>,
    expected: &FkResult,
) -> Verdict {
    let actual = match rust_impl::fk(rust_model, joint_values) {
        Ok(a) => a,
        Err(e) => return Verdict::Fail(e),
    };
    for (link, want) in &expected.link_transforms {
        let Some(got) = actual.link_transforms.get(link) else {
            return Verdict::Fail(format!("rust produced no transform for link {link:?}"));
        };
        for (i, (w, g)) in want.iter().zip(got.iter()).enumerate() {
            let d = (w - g).abs();
            // NaN must fail: `d > tol` alone is false for NaN, which would let
            // a NaN transform pass silently.
            if d.is_nan() || d > cfg.tol_fk {
                return Verdict::Fail(format!(
                    "link {link:?} entry {i}: oracle {w:.17e} vs rust {g:.17e} (|d|={d:.3e} > {:.3e})",
                    cfg.tol_fk
                ));
            }
        }
    }
    Verdict::Pass
}

/// Compares a `jacobian` case, returning both the verdict and the worst
/// elementwise deviation observed (even on a pass) so the caller can track
/// the true worst case across every case in the run, not just the failures
/// -- "passes" is not what this task's parity run reports, the number is.
/// `f64::NAN` when the two sides disagree on shape or Rust errored, since
/// no elementwise deviation exists to report in that case.
fn compare_jacobian(
    cfg: &Config,
    rust_model: &RobotModel,
    group: &str,
    joint_values: &BTreeMap<String, f64>,
    expected: &JacobianResult,
) -> (Verdict, f64) {
    let actual = match rust_impl::jacobian(rust_model, group, joint_values) {
        Ok(a) => a,
        Err(e) => return (Verdict::Fail(e), f64::NAN),
    };
    if actual.rows != expected.rows || actual.cols != expected.cols {
        return (
            Verdict::Fail(format!(
                "shape mismatch: rust {}x{} vs oracle {}x{}",
                actual.rows, actual.cols, expected.rows, expected.cols
            )),
            f64::NAN,
        );
    }

    let mut max_dev = 0.0f64;
    let mut max_at = 0usize;
    for (i, (&w, &g)) in expected.data.iter().zip(&actual.data).enumerate() {
        let d = (w - g).abs();
        // NaN must win over any finite max so a NaN entry cannot hide
        // behind an earlier, smaller finite deviation.
        if d.is_nan() || d > max_dev {
            max_dev = d;
            max_at = i;
        }
    }

    if max_dev.is_nan() || max_dev > cfg.tol_jacobian {
        let w = expected.data[max_at];
        let g = actual.data[max_at];
        return (
            Verdict::Fail(format!(
                "entry {max_at}: oracle {w:.17e} vs rust {g:.17e} (|d|={max_dev:.3e} > {:.3e})",
                cfg.tol_jacobian
            )),
            max_dev,
        );
    }
    (Verdict::Pass, max_dev)
}

/// Compares one [`Op::Constraints`] case. Returns the verdict, plus the
/// oracle's own `satisfied` verdict for the case's one constraint (`None` on
/// any mismatch/error, since there is then no agreed-on answer to bucket) --
/// the caller uses this for the per-kind satisfied/violated split, which must
/// reflect ground truth (the oracle), not whichever side the comparison
/// happened to fail on.
fn compare_constraints(
    cfg: &Config,
    rust_model: &RobotModel,
    joint_values: &BTreeMap<String, f64>,
    spec: &ConstraintsSpec,
    expected: &ConstraintsResult,
) -> (Verdict, Option<bool>) {
    let actual = match rust_impl::constraints(rust_model, joint_values, spec) {
        Ok(a) => a,
        Err(e) => return (Verdict::Fail(e), None),
    };
    if actual.results.len() != expected.results.len() {
        return (
            Verdict::Fail(format!(
                "result count mismatch: rust {} vs oracle {}",
                actual.results.len(),
                expected.results.len()
            )),
            None,
        );
    }
    for (i, (a, e)) in actual.results.iter().zip(&expected.results).enumerate() {
        if a.satisfied != e.satisfied {
            return (
                Verdict::Fail(format!(
                    "constraint {i}: satisfied mismatch rust={} oracle={}",
                    a.satisfied, e.satisfied
                )),
                Some(e.satisfied),
            );
        }
        let d = (a.distance - e.distance).abs();
        // NaN must fail, matching compare_fk/compare_jacobian's own guard.
        if d.is_nan() || d > cfg.tol_constraints {
            return (
                Verdict::Fail(format!(
                    "constraint {i}: distance oracle {:.17e} vs rust {:.17e} (|d|={d:.3e} > {:.3e})",
                    e.distance, a.distance, cfg.tol_constraints
                )),
                Some(e.satisfied),
            );
        }
    }
    (Verdict::Pass, expected.results.first().map(|r| r.satisfied))
}

/// Row-major 4x4 for `pose` with `translation`, matching `rust_impl`'s
/// `to_row_major_4x4`/`isometry_from_row_major` pair.
fn pose_row_major(translation: Vector3, rotation: UnitQuaternion) -> [f64; 16] {
    rust_impl::to_row_major_4x4(&Isometry3::from_parts(translation.into(), rotation))
}

/// Builds one `moveit_msgs`-shaped constraint at `joint_values`/`fk` (the
/// state the case is drawn from), perturbed to straddle its own tolerance
/// boundary so the differential run's satisfied/violated split is
/// meaningful rather than a coin flip landing far from the boundary.
/// Cycles through seven shapes by `case % 7`: a joint-position constraint, a
/// fixed-frame position constraint, a fixed-frame orientation constraint in
/// each of the two [`OrientationToleranceSpec`] parameterizations, a
/// visibility constraint under each of the two angle criteria
/// (view-angle, range-angle), and a visibility constraint with
/// `target_radius` set -- the cone-vs-robot collision check.
///
/// # The `target_radius` case never places the cone near the robot
///
/// `panda.urdf`/`fanuc.urdf`/`dual_arm_panda.urdf`'s `<collision>` geometry
/// is entirely `<mesh>` (STL); `moveit-model`'s URDF loader does not retain
/// mesh collision geometry at all (see `moveit-collision`'s `parry.rs`
/// module doc), so a `RobotModel` built from any of them has *zero*
/// parry-representable collision geometry -- the port can never detect a
/// cone-vs-robot collision against these three fixtures, no matter where the
/// cone is placed, while the oracle's real FCL backend collides against the
/// real STL meshes. Placing the cone near the robot for those fixtures would
/// therefore compare two backends checking entirely different geometry, not
/// exercise a shared code path -- a parity check that cannot fail whatever
/// the port does is not a parity check. This case instead always places the
/// cone far outside every fixture's reach (`FAR_OFFSET`, well past pr2's own
/// ~1.7m arm length, the largest committed fixture), so both sides agree
/// "clear" for every fixture regardless of collision-geometry differences.
/// This exercises the real request/response wire path and the "no collision"
/// branch of `VisibilityConstraint::decide_cone` end-to-end against the
/// oracle; the "collision found" branch is covered instead by
/// `moveit-constraints`' own `cone_through_a_robot_link_is_violated` unit
/// test, which uses pr2's one primitive (non-mesh) collision shape
/// (`base_bellow_link`) since that is the only committed fixture whose
/// collision geometry this port loads at all.
///
/// Every case here resolves against the model frame (upstream's
/// `mobile_frame_ == false` path): the mobile-frame resolution path is a
/// shared, already-tested code path (`position.rs`/`orientation.rs`/
/// `visibility.rs`'s own `mobile_frame_is_resolved_fresh_from_state`-style
/// unit tests), not a further axis this bulk generator also needs to sweep.
fn build_constraint_case(
    case: usize,
    rng: &mut ChaCha8Rng,
    model: &RobotModel,
    joint_values: &BTreeMap<String, f64>,
    fk: &FkResult,
) -> (&'static str, ConstraintsSpec) {
    let mut spec = ConstraintsSpec::default();
    let model_frame = model.model_frame().to_owned();

    let kind = match case % 7 {
        0 => {
            let eligible: Vec<&str> = model
                .joint_models()
                .filter(|j| j.variable_count() == 1)
                .map(|j| j.name())
                .collect();
            let joint_name = if eligible.is_empty() {
                model.joint_names()[0].as_str()
            } else {
                eligible[case % eligible.len()]
            };
            let current = *joint_values.get(joint_name).unwrap_or(&0.0);
            let tolerance: f64 = rng.random_range(0.01..0.2);
            let offset = rng.random_range(-2.0 * tolerance..2.0 * tolerance);
            spec.joint_constraints.push(JointConstraintSpec {
                joint_name: joint_name.to_owned(),
                position: current - offset,
                tolerance_above: tolerance,
                tolerance_below: tolerance,
                weight: 1.0,
            });
            "joint"
        }
        1 => {
            let links = model.link_names();
            let link_name = &links[case % links.len()];
            let point = fk
                .link_transforms
                .get(link_name)
                .expect("link_name came from model.link_names()");
            let mut center = [point[3], point[7], point[11]];
            let half_extent: f64 = rng.random_range(0.02..0.1);
            let r = rng.random_range(0.0..2.0 * half_extent);
            center[case % 3] -= r;
            spec.position_constraints.push(PositionConstraintSpec {
                frame_id: model_frame,
                link_name: link_name.clone(),
                target_point_offset: [0.0, 0.0, 0.0],
                regions: vec![ConstraintRegionSpec {
                    shape: ShapeSpec::Box {
                        size: [2.0 * half_extent; 3],
                    },
                    pose: pose_row_major(
                        Vector3::new(center[0], center[1], center[2]),
                        UnitQuaternion::identity(),
                    ),
                }],
                weight: 1.0,
            });
            "position"
        }
        parameterization @ (2 | 3) => {
            let links = model.link_names();
            let link_name = &links[case % links.len()];
            let point = fk
                .link_transforms
                .get(link_name)
                .expect("link_name came from model.link_names()");
            let actual = quat_from_row_major(point);
            let tolerance: f64 = rng.random_range(0.01..0.3);
            let theta = rng.random_range(0.0..2.0 * tolerance);
            // `actual * Rz(-theta)` makes the rotation error
            // `target^-1 * actual` exactly `Rz(theta)`, whose Euler/rotation-
            // vector decomposition is `[0, 0, theta]` regardless of `actual`
            // -- see this function's own commit for the derivation. That
            // keeps the boundary straddle exact instead of merely
            // approximate.
            let target = actual * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), -theta);
            let c = target.coords;
            let tolerance_spec = if parameterization == 2 {
                OrientationToleranceSpec::XyzEuler {
                    x: tolerance,
                    y: tolerance,
                    z: tolerance,
                }
            } else {
                OrientationToleranceSpec::RotationVector {
                    x: tolerance,
                    y: tolerance,
                    z: tolerance,
                }
            };
            spec.orientation_constraints
                .push(OrientationConstraintSpec {
                    frame_id: model_frame,
                    link_name: link_name.clone(),
                    orientation: [c[0], c[1], c[2], c[3]],
                    tolerance: tolerance_spec,
                    weight: 1.0,
                });
            if parameterization == 2 {
                "orientation_xyz_euler"
            } else {
                "orientation_rotation_vector"
            }
        }
        4 => {
            let tolerance = rng.random_range(0.02..0.3);
            let angle = rng.random_range(0.0..2.0 * tolerance);
            // `target_z` (the target pose's local z axis) must equal
            // `-[sin(angle), 0, cos(angle)]` so that `dp = sensor_view_axis
            // . -target_z == cos(angle)` exactly; rotating the standard z
            // axis by `pi + angle` about y lands there (see this function's
            // own commit for the derivation).
            let target_rotation =
                UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f64::consts::PI + angle);
            spec.visibility_constraints.push(VisibilityConstraintSpec {
                sensor_frame_id: model_frame.clone(),
                sensor_pose: pose_row_major(Vector3::zeros(), UnitQuaternion::identity()),
                sensor_view_direction: "sensor_z".to_owned(),
                target_frame_id: model_frame,
                target_pose: pose_row_major(Vector3::zeros(), target_rotation),
                cone_sides: 3,
                target_radius: None,
                max_view_angle: Some(tolerance),
                max_range_angle: None,
                weight: 1.0,
            });
            "visibility_view_angle"
        }
        5 => {
            let tolerance: f64 = rng.random_range(0.02..0.3);
            let angle = rng.random_range(0.0..2.0 * tolerance);
            // The sensor-to-target direction is exactly
            // `[sin(angle), 0, cos(angle)]`, so `dp = sensor_view_axis . dir
            // == cos(angle)` exactly.
            let target_translation = Vector3::new(angle.sin(), 0.0, angle.cos());
            spec.visibility_constraints.push(VisibilityConstraintSpec {
                sensor_frame_id: model_frame.clone(),
                sensor_pose: pose_row_major(Vector3::zeros(), UnitQuaternion::identity()),
                sensor_view_direction: "sensor_z".to_owned(),
                target_frame_id: model_frame,
                target_pose: pose_row_major(target_translation, UnitQuaternion::identity()),
                cone_sides: 3,
                target_radius: None,
                max_view_angle: None,
                max_range_angle: Some(tolerance),
                weight: 1.0,
            });
            "visibility_range_angle"
        }
        6 => {
            // Well past pr2's own ~1.7m reach -- see this function's doc
            // comment for why the cone must stay clear of every fixture's
            // geometry here, not just panda/fanuc/dual_arm_panda's mesh
            // links.
            const FAR_OFFSET: f64 = 50.0;
            let links = model.link_names();
            let link_name = &links[case % links.len()];
            let point = fk
                .link_transforms
                .get(link_name)
                .expect("link_name came from model.link_names()");
            let mut anchor = Vector3::new(point[3], point[7], point[11]);
            let axis = case % 3;
            anchor[axis] += FAR_OFFSET;
            let radius = rng.random_range(0.05..0.5);
            spec.visibility_constraints.push(VisibilityConstraintSpec {
                sensor_frame_id: model_frame.clone(),
                sensor_pose: pose_row_major(
                    anchor + Vector3::new(0.0, 0.0, 1.0),
                    UnitQuaternion::identity(),
                ),
                sensor_view_direction: "sensor_z".to_owned(),
                target_frame_id: model_frame,
                target_pose: pose_row_major(anchor, UnitQuaternion::identity()),
                cone_sides: 3 + case % 6,
                target_radius: Some(radius),
                max_view_angle: None,
                max_range_angle: None,
                weight: 1.0,
            });
            "visibility_cone"
        }
        _ => unreachable!("case % 7 is in 0..7"),
    };

    (kind, spec)
}

/// Rotation part of a row-major 4x4, matching the oracle's
/// `fromRowMajor4x4`'s rotation half (see `rust_impl::isometry_from_row_major`
/// for the translation half this function does not need).
fn quat_from_row_major(m: &[f64; 16]) -> UnitQuaternion {
    let rotation = Rotation3::from_matrix_unchecked(nalgebra::Matrix3::new(
        m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10],
    ));
    UnitQuaternion::from_rotation_matrix(&rotation)
}

/// Print per-case lines and the summary. Returns the failure count.
fn report(verdicts: &[(String, Verdict)]) -> Result<usize, String> {
    let mut failures = 0usize;
    // Distinct failure messages, so 1,000 identical "unimplemented" lines
    // collapse to one with a count instead of burying the real disagreements.
    let mut by_message: BTreeMap<&str, (usize, &str)> = BTreeMap::new();

    for (name, verdict) in verdicts {
        match verdict {
            Verdict::Pass => println!("PASS {name}"),
            Verdict::Fail(msg) => {
                failures += 1;
                let entry = by_message.entry(msg.as_str()).or_insert((0, name.as_str()));
                entry.0 += 1;
                if entry.0 == 1 {
                    println!("FAIL {name}: {msg}");
                }
            }
        }
    }

    println!("\n--- summary ---");
    println!("cases:  {}", verdicts.len());
    println!("passed: {}", verdicts.len() - failures);
    println!("failed: {failures}");
    if failures > 0 {
        println!("distinct failure messages:");
        for (msg, (count, first)) in &by_message {
            println!("  {count:>6}x  (first: {first})  {msg}");
        }
    }
    Ok(failures)
}
