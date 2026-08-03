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

use moveit_model::RobotModel;
use moveit_srdf::SrdfModel;
use protocol::{FkResult, JacobianResult, ModelInfo, Op, OracleResult, Request, Response};

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

    for (case, joint_values) in states.into_iter().enumerate() {
        let expected = match oracle.ask(Op::Fk {
            joint_values: joint_values.clone(),
            links: Vec::new(),
        })? {
            OracleResult::Fk(f) => f,
            other => return Err(format!("expected fk, got {other:?}")),
        };
        verdicts.push((
            format!("fk[{case}]"),
            compare_fk(cfg, &rust_model, &joint_values, &expected),
        ));

        if let Some(group) = &cfg.group {
            let expected = match oracle.ask(Op::Jacobian {
                group: group.clone(),
                joint_values: joint_values.clone(),
            })? {
                OracleResult::Jacobian(j) => j,
                other => return Err(format!("expected jacobian, got {other:?}")),
            };
            let (verdict, dev) =
                compare_jacobian(cfg, &rust_model, group, &joint_values, &expected);
            if dev.is_finite() {
                max_jacobian_dev = max_jacobian_dev.max(dev);
            }
            verdicts.push((format!("jacobian[{case}]"), verdict));
        }
    }

    if cfg.group.is_some() {
        println!("worst jacobian deviation: {max_jacobian_dev:.3e}");
    }

    report(&verdicts)
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
