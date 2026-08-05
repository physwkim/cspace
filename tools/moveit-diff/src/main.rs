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
//!             [--tol-fk EPS] [--group NAME] [--tol-jacobian EPS]
//!             [--collision] [--tol-distance EPS] [--stats-json <path>]
//!             [--oracle <cmd> [args...]]
//! ```
//!
//! Exit status is 0 only when every case passed.

mod protocol;
mod rust_impl;

use std::collections::BTreeMap;
use std::io::{BufRead, BufReader, Write};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::sync::Arc;

use moveit_collision::{AllowedCollisionMatrix, LinkPaddingScale, ParryCollisionEnv, World};
use moveit_geometry::{Cuboid, Isometry3, Rotation3, Shape, UnitQuaternion, Vector3};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use protocol::{
    CollisionCheckResult, CollisionObjectSpec, ConstraintRegionSpec, ConstraintsResult,
    ConstraintsSpec, DistancePair, FkResult, IkResult, JacobianResult, JointConstraintSpec,
    ModelInfo, Op, OracleResult, OrientationConstraintSpec, OrientationToleranceSpec,
    PositionConstraintSpec, Request, Response, ShapeSpec, VisibilityConstraintSpec,
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
    /// Whether to also run the `collision` op against a fixed collision
    /// scene, alongside every `fk` case. Off by default, mirroring `group`:
    /// existing `fk`-only invocations are unaffected.
    collision: bool,
    tol_distance: f64,
    /// Also run the `ik` op against every `fk` case, using `group` (this
    /// flag is meaningless without `--group`). Success is a statistic, not
    /// a per-case verdict -- see `run`'s ik block -- but a converged
    /// solution's own FK is still checked to `tol_ik`, which IS a per-case
    /// correctness verdict.
    ///
    /// `tol_ik` must stay looser than `SolverParams::default().epsilon`
    /// (`1e-5`): `rust_impl::ik` always solves with that default, matching
    /// the oracle's own `kEpsilon` for a fair success-rate comparison, and
    /// `CartToJnt`'s convergence check accepts any step whose twist norm is
    /// `<= epsilon` -- so a converged solution's FK error can legitimately
    /// land anywhere in `(0, epsilon]`, not just near `0`. A 5,000-case
    /// panda_arm sweep measured this directly: with `tol_ik = 1e-6`, 2930
    /// cases "failed" with translation errors between `7e-6` and `9.9e-6`,
    /// none above `epsilon`'s `1e-5` bound -- expected epsilon-bounded
    /// slack, not a solver defect. `tol_ik` only catches a real correctness
    /// bug (branch-wrong solution, sign error, etc.) once it is set above
    /// that bound.
    ik: bool,
    tol_ik: f64,
    ik_position_only: bool,
    /// `Op::Ik::max_restarts` / `SolverParams::max_restarts` on this side.
    /// Defaults to `20`, matching `SolverParams::default()`, so an
    /// unqualified `--ik` invocation keeps its established behaviour.
    /// `--ik-max-restarts 0` is round 2's decisive run: with no restart
    /// randomness on either side, a surviving success-rate gap cannot be
    /// restart-RNG divergence -- see `Op::Ik::max_restarts`'s own doc
    /// comment.
    ik_max_restarts: u32,
    /// Fraction of each active joint's own `(max - min)` range used as its
    /// `Op::Ik::consistency_limits` bound. `None` (the default) sends no
    /// consistency limits at all -- unqualified `--ik` invocations keep
    /// their established behaviour. Deliberately a *fraction of range*
    /// rather than a fixed radian/metre bound: `panda_arm`'s revolute
    /// joints and `manipulator`'s prismatic-free arm and a gripper's
    /// prismatic finger do not share units or scale, so a single absolute
    /// bound could be meaninglessly loose on one and always-rejecting on
    /// another. See `Op::Ik::consistency_limits`'s doc comment for why this
    /// is oracle-comparable at all.
    ik_consistency_fraction: Option<f64>,
    /// Where to write the run's counts as JSON, alongside the existing
    /// human-readable stdout report. `None` (the default) writes nothing,
    /// keeping every existing invocation's behaviour unchanged.
    ///
    /// PORTING-PLAN.md §60.3 is why this exists: two of round 9's reported
    /// denominators were wrong because they were hand-parsed out of this
    /// tool's own prose (once double-counting an inline `FAIL` line against
    /// the end-of-run aggregate that restates it, once from a still-unclosed
    /// arithmetic error) instead of read from a number this binary already
    /// computed. `--stats-json` makes every count [`RunStats`] carries
    /// re-derivable by construction -- `serde_json::from_str` on this file,
    /// not a regex over stdout -- instead of re-parsed by every reader
    /// (this round's worker, the next round's reviewer, p3-acm) hitting the
    /// same class of mistake independently.
    stats_json: Option<String>,
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
        let mut collision = false;
        let mut tol_distance = 1e-4;
        let mut ik = false;
        // See `Config::tol_ik`'s own doc comment: this must stay looser than
        // `SolverParams::default().epsilon` (1e-5), which is what actually
        // bounds a converged solution's FK error.
        let mut tol_ik = 2e-5;
        let mut ik_position_only = false;
        let mut ik_max_restarts = 20u32;
        let mut ik_consistency_fraction: Option<f64> = None;
        let mut stats_json: Option<String> = None;
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
                "--collision" => collision = true,
                "--tol-distance" => {
                    tol_distance = want("--tol-distance")?
                        .parse()
                        .map_err(|e| format!("--tol-distance: {e}"))?
                }
                "--ik" => ik = true,
                "--tol-ik" => {
                    tol_ik = want("--tol-ik")?
                        .parse()
                        .map_err(|e| format!("--tol-ik: {e}"))?
                }
                "--ik-position-only" => ik_position_only = true,
                "--ik-max-restarts" => {
                    ik_max_restarts = want("--ik-max-restarts")?
                        .parse()
                        .map_err(|e| format!("--ik-max-restarts: {e}"))?
                }
                "--ik-consistency-limit" => {
                    ik_consistency_fraction = Some(
                        want("--ik-consistency-limit")?
                            .parse()
                            .map_err(|e| format!("--ik-consistency-limit: {e}"))?,
                    )
                }
                "--stats-json" => stats_json = Some(want("--stats-json")?),
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

        // `--cases` sizes the pool of random states that every other op reads:
        // the fk/jacobian/collision/ik loop walks it, and the constraint loop
        // cycles through it. So `--cases 0` alongside any of them is a
        // contradictory request -- it asks to compare something against no
        // states at all. Rejecting it here, at the one point a `Config` is
        // built, is what lets every consumer below index the pool without a
        // guard; `run_constraint_cases`' `case % states.len()` divided by zero
        // before this check existed.
        if cases == 0 {
            let mut asked: Vec<&str> = Vec::new();
            if constraints > 0 {
                asked.push("--constraints");
            }
            if collision {
                asked.push("--collision");
            }
            if ik {
                asked.push("--ik");
            }
            if group.is_some() {
                asked.push("--group");
            }
            if !asked.is_empty() {
                return Err(format!(
                    "--cases 0 leaves no states for {} to run against",
                    asked.join(", ")
                ));
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
            collision,
            tol_distance,
            ik,
            tol_ik,
            ik_position_only,
            ik_max_restarts,
            ik_consistency_fraction,
            stats_json,
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
    /// A real gate ran, found nothing above its threshold, but had too few
    /// paired disagreements for that to mean agreement -- see
    /// [`MINIMUM_USABLE_B_PLUS_C`]. Printed distinctly from `Pass` so a
    /// reader scanning for `PASS`/`FAIL` lines cannot mistake "not enough
    /// signal to judge" for "judged and found fine", the exact confusion
    /// `ik_paired_divergence` itself was added to prevent one level up.
    Underpowered(String),
}

fn main() {
    let cfg = match Config::from_args() {
        Ok(c) => c,
        Err(e) => {
            eprintln!("moveit-diff: {e}");
            eprintln!("usage: moveit-diff --urdf <path> --srdf <path> [--cases N] [--seed S]");
            eprintln!("                   [--tol-fk EPS] [--group NAME] [--tol-jacobian EPS]");
            eprintln!("                   [--constraints N] [--tol-constraints EPS]");
            eprintln!("                   [--collision] [--tol-distance EPS]");
            eprintln!(
                "                   [--ik] [--tol-ik EPS] [--ik-position-only] [--ik-max-restarts N]"
            );
            eprintln!("                   [--ik-consistency-limit FRACTION]");
            eprintln!("                   [--stats-json <path>]");
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

/// The `moveit_resources_*_description` packages every committed fixture
/// robot's `<mesh>` URIs name, mapped to their real (gitignored) vendored
/// checkouts under `third_party/moveit_resources/` -- see that directory's
/// own `README.md` for provenance. Unlike `collision_parity.rs`'s
/// `fixture_mesh_search_paths` (which points at the small, committed subset
/// under `fixtures/meshes/` so `cargo test` needs no submodule), this tool
/// already requires `third_party/` for `tools/ci/verify-oracle-sweep.sh`'s own
/// fixture-provenance check, and is never run in CI (see that script's own
/// module doc), so pointing directly at the full vendored tree costs nothing
/// extra here and additionally covers pr2, which `fixtures/meshes/` does not.
fn mesh_search_paths() -> MeshSearchPaths {
    let resources_root = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../third_party/moveit_resources"
    );
    MeshSearchPaths::new([
        (
            "moveit_resources_panda_description",
            format!("{resources_root}/panda_description"),
        ),
        (
            "moveit_resources_fanuc_description",
            format!("{resources_root}/fanuc_description"),
        ),
        (
            "moveit_resources_pr2_description",
            format!("{resources_root}/pr2_description"),
        ),
    ])
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
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &mesh_search_paths())
        .map_err(|e| format!("building RobotModel: {e}"))
}

/// The fixed collision scene both sides check the robot against: a single
/// scale-invariant floor box just below `z = 0`, sized to be robot-agnostic
/// across panda/fanuc/pr2's very different reach envelopes (unlike a
/// fixed-radius sphere, which would need per-robot tuning).
///
/// Built once as `(id, shape, pose)` triples and converted into both the
/// wire [`CollisionObjectSpec`]s and the local [`World`] from that single
/// list, so a disagreement can never come from the two sides being handed
/// different geometry.
fn collision_scene() -> Vec<(String, Arc<Shape>, Isometry3)> {
    vec![(
        "floor".to_owned(),
        Arc::new(Shape::Cuboid(
            Cuboid::new(4.0, 4.0, 0.1).expect("4x4x0.1 are valid, positive cuboid dimensions"),
        )),
        Isometry3::translation(0.0, 0.0, -0.05),
    )]
}

/// The wire [`ShapeSpec`] for a [`Shape`] built by [`collision_scene`].
/// Covers only the variants [`collision_scene`] can produce -- not a general
/// `Shape` -> `ShapeSpec` converter, since [`ShapeSpec`] itself has no
/// `Cone`/`Plane`/`OcTree` variant to convert into (see its own doc comment).
fn shape_to_wire(shape: &Shape) -> Result<ShapeSpec, String> {
    match shape {
        Shape::Sphere(s) => Ok(ShapeSpec::Sphere { radius: s.radius }),
        Shape::Cuboid(b) => Ok(ShapeSpec::Box { size: b.size }),
        Shape::Cylinder(c) => Ok(ShapeSpec::Cylinder {
            radius: c.radius,
            length: c.length,
        }),
        Shape::Mesh(m) => Ok(ShapeSpec::Mesh {
            vertices: m.vertices.iter().map(|v| [v.x, v.y, v.z]).collect(),
            triangles: m.triangles.clone(),
        }),
        other => Err(format!(
            "collision scene shape {other:?} has no ShapeSpec wire form"
        )),
    }
}

/// Everything the `collision` op comparison needs, built once: the local
/// [`ParryCollisionEnv`] (holding the [`collision_scene`] world objects), the
/// [`AllowedCollisionMatrix`] built independently from the same SRDF the
/// oracle loaded (proven to agree with the oracle's own
/// `AllowedCollisionMatrix(*model_->getSRDF())` construction by
/// `crates/moveit-collision/tests/acm_parity.rs`, so it need not be sent over
/// the wire), and the wire form of the scene to send with every request.
struct CollisionFixture {
    env: ParryCollisionEnv,
    acm: AllowedCollisionMatrix,
    wire_objects: Vec<CollisionObjectSpec>,
}

fn build_collision_fixture(cfg: &Config) -> Result<CollisionFixture, String> {
    let srdf =
        SrdfModel::parse_file(&cfg.srdf).map_err(|e| format!("parsing SRDF {}: {e}", cfg.srdf))?;
    let acm = AllowedCollisionMatrix::from_srdf(&srdf);

    let scene = collision_scene();
    let mut world = World::new();
    let mut wire_objects = Vec::with_capacity(scene.len());
    for (id, shape, pose) in &scene {
        world.add_shape(id, shape.clone(), *pose);
        wire_objects.push(CollisionObjectSpec {
            id: id.clone(),
            pose: rust_impl::to_row_major_4x4(pose),
            shape: shape_to_wire(shape)?,
        });
    }

    Ok(CollisionFixture {
        env: ParryCollisionEnv::new(world, LinkPaddingScale::default()),
        acm,
        wire_objects,
    })
}

/// Every count this run computed, in one machine-readable place -- see
/// [`Config::stats_json`]'s doc comment for why this exists. Mirrors exactly
/// what the stdout report already prints; nothing here is derived
/// specially for this struct.
#[derive(serde::Serialize)]
struct RunStats {
    cases: usize,
    passed: usize,
    failed: usize,
    underpowered: usize,
    /// `Some` only when `--group` ran a jacobian comparison.
    worst_jacobian_deviation: Option<f64>,
    /// `Some` only when `--collision` ran.
    worst_distance_deviation: Option<f64>,
    /// `Some` only when `--collision` ran.
    distance_pairs: Option<DistancePairStats>,
    /// `Some` only when `--ik` ran.
    ik: Option<IkStats>,
}

fn run(cfg: &Config) -> Result<usize, String> {
    if cfg.ik && cfg.group.is_none() {
        return Err("--ik requires --group".to_owned());
    }
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

    let collision_fixture = if cfg.collision {
        Some(build_collision_fixture(cfg)?)
    } else {
        None
    };

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
    let mut pair_stats = DistancePairStats::default();
    // Kept for the constraint-case generator below, which needs each state's
    // link poses to build "meaningful" (boundary-straddling) constraints
    // rather than re-asking the oracle for the same fk it already answered.
    let mut fks: Vec<FkResult> = Vec::with_capacity(states.len());
    let mut max_distance_dev = 0.0f64;
    let mut ik_stats = IkStats::default();
    // Built once, outside the loop: `NewtonRaphsonSolver` owns the RNG its
    // random restarts draw from, so a fresh solver per case would replay the
    // same `max_restarts` draws on every case. See `rust_impl::IkSolver`.
    let mut ik_solver = match (cfg.ik, &cfg.group) {
        (true, Some(group)) => Some(rust_impl::IkSolver::new(
            &rust_model,
            group,
            cfg.ik_position_only,
            cfg.ik_max_restarts,
        )?),
        _ => None,
    };
    // Full-space (active + mimic), one bound per chain joint -- see
    // `Config::ik_consistency_fraction`'s doc comment for why this is a
    // fraction of each joint's own range rather than one fixed bound, and
    // `Op::Ik::consistency_limits`'s doc comment for why it is keyed by
    // name. Computed once: bounds (and so the range each fraction scales)
    // are group-constant, exactly like `IkSolver`'s own bounds-midpoint seed.
    let ik_consistency_limits: BTreeMap<String, f64> =
        match (&ik_solver, cfg.ik_consistency_fraction) {
            (Some(solver), Some(fraction)) => solver
                .chain_joint_names()
                .into_iter()
                .map(|name| {
                    let bounds = &rust_model
                        .joint_model(&name)
                        .expect("chain_joint_names only ever names real model joints")
                        .variable_bounds()[0];
                    (name, fraction * (bounds.max_position - bounds.min_position))
                })
                .collect(),
            _ => BTreeMap::new(),
        };

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

        if let Some(fixture) = &collision_fixture {
            let expected = match oracle.ask(Op::Collision {
                joint_values: joint_values.clone(),
                objects: fixture.wire_objects.clone(),
                attached_bodies: Vec::new(),
            })? {
                OracleResult::Collision(c) => c,
                other => return Err(format!("expected collision, got {other:?}")),
            };
            let (verdict, dev) = compare_collision(
                cfg,
                &rust_model,
                fixture,
                joint_values,
                &expected,
                &mut pair_stats,
            );
            if dev.is_finite() {
                max_distance_dev = max_distance_dev.max(dev);
            }
            verdicts.push((format!("collision[{case}]"), verdict));
        }

        if cfg.ik {
            // Validated once in `run` before the loop starts.
            let group = cfg.group.as_ref().expect("--ik requires --group");
            let expected = match oracle.ask(Op::Ik {
                group: group.clone(),
                joint_values: joint_values.clone(),
                position_only: cfg.ik_position_only,
                max_restarts: cfg.ik_max_restarts,
                consistency_limits: ik_consistency_limits.clone(),
            })? {
                OracleResult::Ik(r) => r,
                other => return Err(format!("expected ik, got {other:?}")),
            };
            let verdict = compare_ik(
                cfg,
                ik_solver
                    .as_mut()
                    .expect("--ik built the solver before the loop"),
                joint_values,
                &ik_consistency_limits,
                &expected,
                &mut ik_stats,
            );
            verdicts.push((format!("ik[{case}]"), verdict));
        }
    }

    if cfg.group.is_some() {
        println!("worst jacobian deviation: {max_jacobian_dev:.3e}");
    }
    if cfg.collision {
        println!("worst distance deviation: {max_distance_dev:.3e}");
        pair_stats.report();
    }

    if cfg.constraints > 0 {
        run_constraint_cases(cfg, &mut oracle, &rust_model, &states, &fks, &mut verdicts)?;
    }
    if cfg.ik {
        println!(
            "\n--- ik summary ({}) ---",
            cfg.group.as_deref().unwrap_or("")
        );
        match cfg.ik_consistency_fraction {
            Some(fraction) => println!(
                "consistency limits: {:.1}% of range per active joint ({} joints)",
                100.0 * fraction,
                ik_consistency_limits.len()
            ),
            None => println!("consistency limits: none"),
        }
        println!(
            "oracle success rate: {}/{} ({:.1}%)",
            ik_stats.oracle_success,
            ik_stats.total,
            100.0 * ik_stats.oracle_success as f64 / ik_stats.total.max(1) as f64
        );
        println!(
            "rust   success rate: {}/{} ({:.1}%)",
            ik_stats.rust_success,
            ik_stats.total,
            100.0 * ik_stats.rust_success as f64 / ik_stats.total.max(1) as f64
        );
        println!(
            "rust   degenerate (solution == seed): {}/{}",
            ik_stats.rust_degenerate, ik_stats.rust_success
        );
        println!(
            "oracle degenerate (solution == seed): {}/{}",
            ik_stats.oracle_degenerate, ik_stats.oracle_success
        );
        println!(
            "paired: b (oracle only) = {}, c (rust only) = {} (McNemar; b≈c means noise, b>>c means real)",
            ik_stats.oracle_only, ik_stats.rust_only
        );
        let z = paired_divergence_z(ik_stats.oracle_only, ik_stats.rust_only);
        let b_plus_c = ik_stats.oracle_only + ik_stats.rust_only;
        let verdict = if z > PAIRED_DIVERGENCE_Z_THRESHOLD {
            println!(
                "VERDICT: paired divergence is not noise (|z| = {z:.2} > {PAIRED_DIVERGENCE_Z_THRESHOLD}) -- b = {}, c = {} likely reflects a real algorithmic gap, not restart-RNG variance",
                ik_stats.oracle_only, ik_stats.rust_only
            );
            Verdict::Fail(format!(
                "paired ik divergence |z| = {z:.2} exceeds {PAIRED_DIVERGENCE_Z_THRESHOLD} (b = {}, c = {}) -- see PAIRED_DIVERGENCE_Z_THRESHOLD's doc comment",
                ik_stats.oracle_only, ik_stats.rust_only
            ))
        } else if b_plus_c < MINIMUM_USABLE_B_PLUS_C {
            println!(
                "VERDICT: underpowered -- b + c = {b_plus_c} < {MINIMUM_USABLE_B_PLUS_C}, so |z| = {z:.2} not clearing {PAIRED_DIVERGENCE_Z_THRESHOLD} does not mean agreement; see MINIMUM_USABLE_B_PLUS_C's doc comment"
            );
            Verdict::Underpowered(format!(
                "b + c = {b_plus_c} is below {MINIMUM_USABLE_B_PLUS_C}, the minimum this gate needs to resolve a real divergence as small as the one it is calibrated against -- see MINIMUM_USABLE_B_PLUS_C's doc comment"
            ))
        } else {
            Verdict::Pass
        };
        verdicts.push(("ik_paired_divergence".to_string(), verdict));
    }

    let (failures, underpowered) = report(&verdicts)?;

    if let Some(path) = &cfg.stats_json {
        let stats = RunStats {
            cases: verdicts.len(),
            passed: verdicts.len() - failures - underpowered,
            failed: failures,
            underpowered,
            worst_jacobian_deviation: cfg.group.is_some().then_some(max_jacobian_dev),
            worst_distance_deviation: cfg.collision.then_some(max_distance_dev),
            distance_pairs: cfg.collision.then_some(pair_stats),
            ik: cfg.ik.then_some(ik_stats),
        };
        let json = serde_json::to_string_pretty(&stats)
            .map_err(|e| format!("serializing --stats-json: {e}"))?;
        std::fs::write(path, json).map_err(|e| format!("writing --stats-json {path}: {e}"))?;
    }

    Ok(failures)
}

/// Kind label to `(satisfied, violated)` oracle-reported counts -- the
/// factual split this task's completion report names per constraint kind,
/// even if lopsided. Counted per *constraint*, not per case: a combination
/// case contributes one entry to each kind it contains, so the totals here
/// sum to the number of individual `decide()` answers compared rather than
/// to the number of cases.
type KindSplit = BTreeMap<&'static str, (usize, usize)>;

/// Case-label to case-count -- how many of `cfg.constraints` combinations
/// took each of [`SHAPE_CYCLE`]'s slots. Reported alongside [`KindSplit`]
/// because the two answer different questions: this one says how many
/// *combinations* of each composition ran (which is what `PORTING-PLAN.md`
/// §5's Phase 5 first clause counts), the other how many individual
/// constraints of each kind were decided inside them.
type CaseSplit = BTreeMap<String, usize>;

/// Drives [`Op::Constraints`] over `cfg.constraints` generated combinations,
/// cycling through the [`SHAPE_CYCLE`] compositions [`build_constraint_case`]
/// knows how to build -- seven single-kind shapes and five multi-kind
/// combinations -- one per case, reusing the states and fk answers the
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
    let mut cases: CaseSplit = CaseSplit::new();

    for case in 0..cfg.constraints {
        let state_idx = case % states.len();
        let joint_values = &states[state_idx];
        let fk = &fks[state_idx];
        let spec = build_constraint_case(case, &mut rng, rust_model, joint_values, fk);
        let kinds = constraint_kinds(&spec);
        let label = kinds.join("+");

        let expected = match oracle.ask(Op::Constraints {
            joint_values: joint_values.clone(),
            constraints: spec.clone(),
        })? {
            OracleResult::Constraints(c) => c,
            other => return Err(format!("expected constraints, got {other:?}")),
        };

        let (verdict, satisfied) =
            compare_constraints(cfg, rust_model, joint_values, &kinds, &spec, &expected);
        for (kind, satisfied) in kinds.iter().zip(&satisfied) {
            let entry = split.entry(kind).or_insert((0, 0));
            if *satisfied {
                entry.0 += 1;
            } else {
                entry.1 += 1;
            }
        }
        *cases.entry(label.clone()).or_insert(0) += 1;
        verdicts.push((format!("constraints[{case}]:{label}"), verdict));
    }

    println!("constraint combinations by composition:");
    for (label, count) in &cases {
        println!("  {label}: {count} cases");
    }
    println!("constraint satisfied/violated split (oracle-reported), per constraint:");
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
/// oracle's own `satisfied` verdict for each of the case's constraints, in
/// `kinds` order -- the caller buckets these into the per-kind
/// satisfied/violated split, which must reflect ground truth (the oracle),
/// not whichever side the comparison happened to fail on. Empty when there
/// is no per-constraint oracle answer to bucket at all (the port errored, or
/// the two sides disagree on how many constraints the case even has); a
/// failing *comparison* still yields the full oracle split, because the
/// oracle answered every constraint whether or not the port matched it.
///
/// `kinds[i] == "visibility_cone"` skips constraint `i`'s `distance`
/// comparison below (still comparing its `satisfied`, and still comparing
/// both fields for every *other* constraint in the same case -- which is why
/// this is indexed per constraint rather than applied to the whole case:
/// a combination case pairs a cone with kinds whose `distance` is compared
/// exactly). Round-16 measurement (`PORTING-PLAN.md`'s
/// round-16 report; live sweep, seed 4, `--group right_arm --cases 100
/// --constraints 2000` against oracle stamp `cd8ee2c1bdcf7148`) captured all
/// 115/2201 distance mismatches this comparison has ever produced for this
/// kind: max |diff| 5.42e-2, median 3.57e-3, min 3.93e-5, 25/115 sign flips
/// at near-zero depths -- three decades of spread with no floor separating
/// "backend noise" from a hypothetical real defect, so no tolerance short of
/// ~5.4e-2 (larger than many of the depths themselves) would silence every
/// case, and one that large would also silence a genuine regression. `git
/// log` across every commit that has touched `visibility_cone`/`decide_cone`
/// since it was introduced (`7d99473`) turns up zero cases where this
/// specific distance comparison caught a real defect: the one deliberately
/// injected regression on record (`7d99473`'s commit message, flipping
/// `decide_cone`'s final `!result.collision`) was caught by the `satisfied`
/// mismatch it produced, not by `distance`. `distance` disagreeing here is
/// `moveit-collision`'s already-documented, already-accepted deviation 6
/// (`crates/moveit-collision/src/parry.rs`'s module doc): FCL's non-convex
/// penetration depth is itself an approximation, `parry3d_f64` computes an
/// independent approximation of the same ill-posed quantity, and the two
/// need not agree even on an unambiguous single contact (this module's own
/// `visibility_cone_ambiguity_diagnostic` rules out an ambiguous scene as an
/// alternative explanation). Narrowing here, rather than widening
/// `tol_constraints`, keeps this comparison meaningful for the six other
/// constraint kinds (none of which reach collision detection, and none of
/// which have ever produced a mismatch) while giving up exactly one class of
/// coverage: a real bug in the port's visibility-cone depth/distance
/// *value* that does not also flip which side of `decide_cone`'s near/far
/// threshold the case lands on. A bug big enough to change `satisfied` is
/// still caught below.
fn compare_constraints(
    cfg: &Config,
    rust_model: &RobotModel,
    joint_values: &BTreeMap<String, f64>,
    kinds: &[&'static str],
    spec: &ConstraintsSpec,
    expected: &ConstraintsResult,
) -> (Verdict, Vec<bool>) {
    let actual = match rust_impl::constraints(rust_model, joint_values, spec) {
        Ok(a) => a,
        Err(e) => return (Verdict::Fail(e), Vec::new()),
    };
    if actual.results.len() != expected.results.len() || actual.results.len() != kinds.len() {
        return (
            Verdict::Fail(format!(
                "result count mismatch: rust {} vs oracle {} vs generated kinds {}",
                actual.results.len(),
                expected.results.len(),
                kinds.len()
            )),
            Vec::new(),
        );
    }
    let split: Vec<bool> = expected.results.iter().map(|r| r.satisfied).collect();
    for (i, (a, e)) in actual.results.iter().zip(&expected.results).enumerate() {
        if a.satisfied != e.satisfied {
            return (
                Verdict::Fail(format!(
                    "constraint {i} ({}): satisfied mismatch rust={} oracle={}",
                    kinds[i], a.satisfied, e.satisfied
                )),
                split,
            );
        }
        if kinds[i] == "visibility_cone" {
            continue;
        }
        let d = (a.distance - e.distance).abs();
        // NaN must fail, matching compare_fk/compare_jacobian's own guard.
        if d.is_nan() || d > cfg.tol_constraints {
            return (
                Verdict::Fail(format!(
                    "constraint {i} ({}): distance oracle {:.17e} vs rust {:.17e} (|d|={d:.3e} > {:.3e})",
                    kinds[i], e.distance, a.distance, cfg.tol_constraints
                )),
                split,
            );
        }
    }
    (Verdict::Pass, split)
}

/// Row-major 4x4 for `pose` with `translation`, matching `rust_impl`'s
/// `to_row_major_4x4`/`isometry_from_row_major` pair.
fn pose_row_major(translation: Vector3, rotation: UnitQuaternion) -> [f64; 16] {
    rust_impl::to_row_major_4x4(&Isometry3::from_parts(translation.into(), rotation))
}

/// Whether `moveit-collision`'s parry backend can represent this shape at
/// all -- excludes `Mesh` (never retained by `moveit-model`'s URDF loader in
/// the first place, so never reachable from a `RobotModel`'s links, but
/// checked here too for robustness) and `OcTree` (this port's `OcTree`
/// carries no tree payload, see `moveit-collision`'s `parry.rs` module doc,
/// deviation 10).
fn is_parry_representable(shape: &Shape) -> bool {
    !matches!(shape, Shape::Mesh(_) | Shape::OcTree(_))
}

/// Every link name whose collision geometry `moveit-collision`'s parry
/// backend actually represents -- see [`build_constraint_case`]'s doc
/// comment for why the `visibility_cone` case needs this to place a
/// genuine, oracle-comparable collision.
fn parry_representable_link_names(model: &RobotModel) -> Vec<&str> {
    model
        .link_models()
        .iter()
        .filter(|link| {
            link.shapes()
                .iter()
                .any(|s| is_parry_representable(&s.shape))
        })
        .map(moveit_model::LinkModel::name)
        .collect()
}

/// Builds one `moveit_msgs`-shaped constraint *combination* at
/// `joint_values`/`fk` (the state the case is drawn from), each constituent
/// constraint perturbed to straddle its own tolerance boundary so the
/// differential run's satisfied/violated split is meaningful rather than a
/// coin flip landing far from the boundary.
///
/// Cycles through [`SHAPE_CYCLE`] compositions by `case % SHAPE_CYCLE`: the
/// seven single-kind shapes of [`ConstraintShape`] (a joint-position
/// constraint, a fixed-frame position constraint, a fixed-frame orientation
/// constraint in each of the two [`OrientationToleranceSpec`]
/// parameterizations, a visibility constraint under each of the two angle
/// criteria, and a visibility constraint with `target_radius` set -- the
/// cone-vs-robot collision check), then the five multi-kind
/// [`COMBINATIONS`].
///
/// # Why combinations, not only singles
///
/// `PORTING-PLAN.md` §5's Phase 5 first clause is about "제약 조합" --
/// constraint *combinations*. Seven single-constraint shapes sweep each
/// kind's own `decide()` but never reach `KinematicConstraintSet::decide`'s
/// job of running several constraints over one state and reporting one
/// result per constraint in a fixed order. That order is the part a
/// single-constraint case cannot check: both sides push joint, then
/// position, then orientation, then visibility (`rust_impl::constraints`
/// and `oracle.cpp`'s `constraints()`), and with one constraint per case any
/// permutation of those four loops produces an identical result vector.
///
/// # The `target_radius` case is fixture-aware about where it can place the cone
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
/// the port does is not a parity check. For these three, this case always
/// places the cone far outside the model's reach (`FAR_OFFSET`), so both
/// sides agree "clear" regardless of collision-geometry differences.
///
/// `pr2.urdf` is different: `parry_representable_link_names` finds 17 links
/// whose collision geometry is a primitive (5 box, 8 cylinder, 4 sphere) --
/// `moveit-model`'s loader retains those, so both the port and the oracle's
/// FCL backend see the same geometry there. Whenever the model has such a
/// link, half of this shape's occurrences (`(case / SHAPE_CYCLE) % 2 == 0`) place the
/// target exactly at one such link's global collision-shape center (cycling
/// through the eligible links by case) with a small radius and sensor
/// offset -- see the `Some(link_name)` arm below for why both are kept
/// small -- so the cone's filled base cap (see `VisibilityConstraint::
/// cone_mesh`'s own doc for why the cap, not the hollow lateral shell, is
/// what must overlap) reaches the target link's surface without also
/// reaching a neighbor's. The other half keep the `FAR_OFFSET` placement,
/// so the split covers both branches of `decide_cone` against the real
/// oracle instead of only the "no collision" one. Fixtures with no eligible
/// link keep the `FAR_OFFSET` placement for every occurrence, unchanged.
///
/// This closes the coverage gap -- `decide_cone`'s `satisfied`/`violated`
/// verdict matches the oracle on every one of pr2's near- and far-placed
/// cases -- but does not close a second, narrower gap: the reported
/// *distance* (the constraint's diagnostic depth, not its verdict) still
/// disagrees on most near-placed cases. `decide_cone` builds a throwaway
/// local environment containing only the cone plus whatever robot links the
/// port can represent (`max_contacts: 1`, so only the first contact found is
/// kept); the oracle's equivalent environment also contains pr2's mesh
/// links, which sit immediately adjacent to nearly every primitive link on
/// this densely-packed fixture. When the cone also touches one of those,
/// upstream's traversal order can surface that contact instead of the
/// intended one, and the two sides then report different depths for the
/// same verdict. This is the mesh-visibility gap already on record (see
/// `PORTING-PLAN.md`), now visible in a second place; it is not a
/// `decide_cone` logic defect (its collision-decision logic matches
/// upstream's `VisibilityConstraint::decide()` exactly), and closing it
/// requires mesh collision geometry this port does not carry, not a change
/// to this generator or to `decide_cone` itself.
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
) -> ConstraintsSpec {
    let mut spec = ConstraintsSpec::default();
    let slot = case % SHAPE_CYCLE;
    let shapes: &[ConstraintShape] = match SINGLE_SHAPES.get(slot) {
        Some(single) => std::slice::from_ref(single),
        None => COMBINATIONS[slot - SINGLE_SHAPES.len()],
    };
    for shape in shapes {
        push_shape(*shape, &mut spec, case, rng, model, joint_values, fk);
    }
    spec
}

/// One of the seven single-kind constraint shapes [`push_shape`] builds.
/// [`SINGLE_SHAPES`] runs each on its own; [`COMBINATIONS`] pairs them up.
#[derive(Clone, Copy)]
enum ConstraintShape {
    Joint,
    Position,
    OrientationXyzEuler,
    OrientationRotationVector,
    VisibilityViewAngle,
    VisibilityRangeAngle,
    VisibilityCone,
}

/// The single-kind shapes, one per `case % SHAPE_CYCLE` slot, in the order
/// this generator has always produced them.
const SINGLE_SHAPES: [ConstraintShape; 7] = [
    ConstraintShape::Joint,
    ConstraintShape::Position,
    ConstraintShape::OrientationXyzEuler,
    ConstraintShape::OrientationRotationVector,
    ConstraintShape::VisibilityViewAngle,
    ConstraintShape::VisibilityRangeAngle,
    ConstraintShape::VisibilityCone,
];

/// The multi-kind combinations, taking the `case % SHAPE_CYCLE` slots after
/// [`SINGLE_SHAPES`]. Chosen so that every one of the four `moveit_msgs`
/// constraint kinds appears in a combination, every pair of adjacent kinds
/// in the result-vector order (joint→position→orientation→visibility) is
/// spanned by at least one, and the last one carries all four kinds *and*
/// two visibility constraints -- the only composition where a single case's
/// result vector can be permuted within one kind as well as across kinds.
const COMBINATIONS: [&[ConstraintShape]; 5] = [
    &[ConstraintShape::Joint, ConstraintShape::Position],
    &[ConstraintShape::Joint, ConstraintShape::OrientationXyzEuler],
    &[
        ConstraintShape::Position,
        ConstraintShape::OrientationRotationVector,
    ],
    &[
        ConstraintShape::Joint,
        ConstraintShape::Position,
        ConstraintShape::OrientationXyzEuler,
        ConstraintShape::VisibilityViewAngle,
    ],
    &[
        ConstraintShape::Joint,
        ConstraintShape::Position,
        ConstraintShape::OrientationRotationVector,
        ConstraintShape::VisibilityRangeAngle,
        ConstraintShape::VisibilityCone,
    ],
];

/// How many `case % SHAPE_CYCLE` slots there are: one per single-kind shape,
/// then one per combination.
const SHAPE_CYCLE: usize = SINGLE_SHAPES.len() + COMBINATIONS.len();

/// The kind label of every constraint in `spec`, in the order both
/// `rust_impl::constraints` and `oracle.cpp`'s `constraints()` push them
/// into their `KinematicConstraintSet` -- so `constraint_kinds(spec)[i]`
/// names the constraint that produced `results[i]` on either side.
///
/// Derived from the spec rather than reported alongside it by
/// [`build_constraint_case`]: a returned list is a second description of the
/// same thing and can disagree with what was actually built, and the
/// disagreement would show up as a `visibility_cone` distance skip landing
/// on the wrong constraint -- silently, since skipping a comparison cannot
/// fail. Every label here is a function of the one field that decides the
/// port's own branch: an orientation constraint's parameterization is its
/// `tolerance` variant, and a visibility constraint's criterion is which of
/// `target_radius`/`max_view_angle`/`max_range_angle` it carries.
fn constraint_kinds(spec: &ConstraintsSpec) -> Vec<&'static str> {
    let mut kinds: Vec<&'static str> = Vec::new();
    kinds.extend(spec.joint_constraints.iter().map(|_| "joint"));
    kinds.extend(spec.position_constraints.iter().map(|_| "position"));
    kinds.extend(
        spec.orientation_constraints
            .iter()
            .map(|oc| match oc.tolerance {
                OrientationToleranceSpec::XyzEuler { .. } => "orientation_xyz_euler",
                OrientationToleranceSpec::RotationVector { .. } => "orientation_rotation_vector",
            }),
    );
    kinds.extend(spec.visibility_constraints.iter().map(|vc| {
        if vc.target_radius.is_some() {
            "visibility_cone"
        } else if vc.max_view_angle.is_some() {
            "visibility_view_angle"
        } else {
            "visibility_range_angle"
        }
    }));
    kinds
}

/// Pushes one [`ConstraintShape`] into `spec`. See
/// [`build_constraint_case`]'s doc comment for what each shape is and why it
/// is built the way it is.
fn push_shape(
    shape: ConstraintShape,
    spec: &mut ConstraintsSpec,
    case: usize,
    rng: &mut ChaCha8Rng,
    model: &RobotModel,
    joint_values: &BTreeMap<String, f64>,
    fk: &FkResult,
) {
    let model_frame = model.model_frame().to_owned();

    match shape {
        ConstraintShape::Joint => {
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
        }
        ConstraintShape::Position => {
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
        }
        ConstraintShape::OrientationXyzEuler | ConstraintShape::OrientationRotationVector => {
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
            let tolerance_spec = if matches!(shape, ConstraintShape::OrientationXyzEuler) {
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
        }
        ConstraintShape::VisibilityViewAngle => {
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
        }
        ConstraintShape::VisibilityRangeAngle => {
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
        }
        ConstraintShape::VisibilityCone => {
            let eligible = parry_representable_link_names(model);
            // See this function's doc comment: fixtures with no
            // parry-representable link always take the far branch, and
            // fixtures that have one (pr2) alternate so the split covers
            // both branches of `decide_cone` against the oracle.
            let hit_link = (!eligible.is_empty() && (case / SHAPE_CYCLE) % 2 == 0)
                .then(|| eligible[case % eligible.len()]);

            let (anchor, radius, sensor_offset) = match hit_link {
                Some(link_name) => {
                    let link_model = model
                        .link_model(link_name)
                        .expect("link_name came from parry_representable_link_names(model)");
                    let shape = link_model
                        .shapes()
                        .iter()
                        .find(|s| is_parry_representable(&s.shape))
                        .expect("link_name is eligible because it has such a shape");
                    let link_fk = rust_impl::isometry_from_row_major(
                        fk.link_transforms
                            .get(link_name)
                            .expect("link_name came from model.link_models()"),
                    );
                    let center = (link_fk * shape.origin_transform).translation.vector;
                    // Small on both axes, and matched to each other, not
                    // to the far branch's radius/1m offset: pr2's
                    // smallest primitive (the head-mount spheres, r =
                    // 0.0005m) sits centimeters from its largest
                    // (base_bellow_link's box, half-extent ~0.185m), and
                    // every one of them sits right up against the rest
                    // of pr2's body -- much of it mesh, invisible to this
                    // port. A radius or sensor offset sized for the big
                    // shapes would swallow or sweep through that
                    // neighboring geometry; keeping both small enough for
                    // even the smallest shape (but still positive, so the
                    // filled base cap always reaches the target's own
                    // surface -- see this function's doc comment) keeps
                    // the cone local to the one link this case intends.
                    (center, rng.random_range(0.005..0.015), 0.005)
                }
                None => {
                    // Well past pr2's own ~1.7m reach -- see this
                    // function's doc comment for why the cone must stay
                    // clear of every fixture's geometry here, not just
                    // panda/fanuc/dual_arm_panda's mesh links.
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
                    (anchor, rng.random_range(0.05..0.5), 1.0)
                }
            };
            spec.visibility_constraints.push(VisibilityConstraintSpec {
                sensor_frame_id: model_frame.clone(),
                sensor_pose: pose_row_major(
                    anchor + Vector3::new(0.0, 0.0, sensor_offset),
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
        }
    }
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

/// Accumulated across every `collision[case]` whose booleans agreed (so a
/// pair comparison was even possible) -- counts how often the two sides
/// *name a different pair* for `self_distance`/`robot_distance`, split by
/// whether that disagreement also moved the scalar past `cfg.tol_distance`.
///
/// This exists because a pair disagreement and a value disagreement are
/// different observations that [`compare_collision`] used to conflate: the
/// `DistancePair`s only ever reached the operator inside a `distance
/// differs` `Verdict::Fail` message, so a case where the two sides pick
/// different pairs but land on the same number -- a tie, since a case with
/// more than one equidistant pair has no unique right answer -- was
/// invisible. On pr2 that is not rare: most `robot_distance` pair
/// disagreements are exactly this (any of pr2's eight equidistant caster
/// wheels is a correct nearest-to-`floor` answer), and folding a tie into
/// FAIL would make the sweep fail on cases that are not defects. Recording
/// the split instead -- disagreements, and how many of those also exceed
/// tolerance -- gives p3-acm the denominator their pair-ranking-flip
/// diagnosis (PORTING-PLAN.md §53.3) needs, without changing what makes a
/// case pass or fail.
///
/// PORTING-PLAN.md §60.2 found a second, distinct species this struct did
/// not separate out: a case can exceed `cfg.tol_distance` with the pair
/// *agreeing* on both sides -- no ranking involved at all, just a magnitude
/// disagreement on the one pair both sides already picked. A pair flip and
/// a same-pair magnitude disagreement have different causes (a ranking bug
/// cannot explain a case where both sides ranked identically), so
/// `*_pair_flip_and_value_diverges` (subset of `*_pair_disagrees`) and
/// `*_same_pair_and_value_diverges` (pair matched, value still exceeded
/// tolerance) are tracked and reported separately instead of one combined
/// "also exceeded tol" count that used to fold both species together.
/// PORTING-PLAN.md §67.2 found 62/3000 self-side same-pair divergences and
/// left their pair composition unsplit: how much is the known
/// `base_bellow_link`/`torso_lift_link` plateau (§56, §63.2) versus an
/// unexplained remainder is exactly the number that decides whether §56's
/// ranking-only account still covers the self side. Keyed by
/// [`pair_key`] (order-independent, matching [`distance_pair_matches`]) so
/// one physical pair cannot appear as two histogram entries depending on
/// which side named `body_name_1` first.
type PairHistogram = BTreeMap<String, usize>;

#[derive(Debug, Default, Clone, serde::Serialize)]
struct DistancePairStats {
    self_total: usize,
    self_pair_disagrees: usize,
    self_pair_flip_and_value_diverges: usize,
    self_same_pair_and_value_diverges: usize,
    self_same_pair_histogram: PairHistogram,
    robot_total: usize,
    robot_pair_disagrees: usize,
    robot_pair_flip_and_value_diverges: usize,
    robot_same_pair_and_value_diverges: usize,
    robot_same_pair_histogram: PairHistogram,
}

impl DistancePairStats {
    /// `"ranking flipped in N of M, of which K also moved the value past
    /// tol"` plus the same-pair-only divergence count, once for
    /// `self_distance` and once for `robot_distance`.
    fn report(&self) {
        println!(
            "self  pair disagreement: {}/{} ({:.1}%), of which {} also exceeded tol (pair-flip)",
            self.self_pair_disagrees,
            self.self_total,
            100.0 * self.self_pair_disagrees as f64 / self.self_total.max(1) as f64,
            self.self_pair_flip_and_value_diverges
        );
        println!(
            "self  same-pair value divergence: {}/{} ({:.1}%)",
            self.self_same_pair_and_value_diverges,
            self.self_total,
            100.0 * self.self_same_pair_and_value_diverges as f64 / self.self_total.max(1) as f64,
        );
        println!(
            "robot pair disagreement: {}/{} ({:.1}%), of which {} also exceeded tol (pair-flip)",
            self.robot_pair_disagrees,
            self.robot_total,
            100.0 * self.robot_pair_disagrees as f64 / self.robot_total.max(1) as f64,
            self.robot_pair_flip_and_value_diverges
        );
        println!(
            "robot same-pair value divergence: {}/{} ({:.1}%)",
            self.robot_same_pair_and_value_diverges,
            self.robot_total,
            100.0 * self.robot_same_pair_and_value_diverges as f64 / self.robot_total.max(1) as f64,
        );
    }
}

/// Whether two [`DistancePair`]s name the same pair of bodies, order-
/// independent: the two sides do not agree on which of `link_names[0]`/`[1]`
/// is "first" (seen directly in round 8's sweep, e.g. oracle's
/// `floor/bl_caster_r_wheel_link` against this port's
/// `br_caster_l_wheel_link/floor`), so a positional comparison would count
/// every such case as a disagreement even when the named pair is identical.
/// `None` on both sides (upstream's `DistanceResultsData::clear()` state,
/// never overwritten) counts as agreement.
/// Canonical, order-independent key for a matched [`DistancePair`], e.g.
/// `"base_bellow_link/torso_lift_link"`. Must agree with
/// [`distance_pair_matches`]'s order-independence -- both sides can name
/// `body_name_1`/`body_name_2` in either order (see that function's own doc
/// comment) -- or a single physical pair would split across two histogram
/// entries depending only on which side happened to report first.
fn pair_key(pair: &DistancePair) -> String {
    let mut names = [pair.body_name_1.as_str(), pair.body_name_2.as_str()];
    names.sort_unstable();
    format!("{}/{}", names[0], names[1])
}

fn distance_pair_matches(a: &Option<DistancePair>, b: &Option<DistancePair>) -> bool {
    match (a, b) {
        (None, None) => true,
        (Some(a), Some(b)) => {
            let a_bodies = [
                (&a.body_name_1, &a.body_type_1),
                (&a.body_name_2, &a.body_type_2),
            ];
            let b_bodies = [
                (&b.body_name_1, &b.body_type_1),
                (&b.body_name_2, &b.body_type_2),
            ];
            (a_bodies[0] == b_bodies[0] && a_bodies[1] == b_bodies[1])
                || (a_bodies[0] == b_bodies[1] && a_bodies[1] == b_bodies[0])
        }
        _ => false,
    }
}

/// Compares a `collision` case: `self_collision`/`robot_collision` exactly,
/// `self_distance`/`robot_distance` at `cfg.tol_distance`. Contact/nearest-
/// point coordinates are never compared -- PORTING-PLAN.md §4.5 records that
/// exclusion as Phase 3's recorded verification limit, not an oversight; the
/// two sides' contact geometry differs by construction (module doc,
/// `crates/moveit-collision/src/parry.rs`, deviations 4 and 6) in ways that
/// would never converge under any tolerance.
///
/// Also tallies `pair_stats` (see [`DistancePairStats`]): a pair
/// disagreement never affects the returned [`Verdict`] by itself, only the
/// scalar does, exactly as before this parameter existed.
///
/// Returns both the verdict and the worst of the two distance deviations
/// (even on a pass), mirroring [`compare_jacobian`]'s reporting: the number
/// this task's parity run reports is the worst deviation across every case,
/// not just the failures. `f64::NAN` when a boolean disagrees or Rust
/// errored, since no distance deviation is meaningful to report then.
fn compare_collision(
    cfg: &Config,
    rust_model: &RobotModel,
    fixture: &CollisionFixture,
    joint_values: &BTreeMap<String, f64>,
    expected: &CollisionCheckResult,
    pair_stats: &mut DistancePairStats,
) -> (Verdict, f64) {
    let actual = match rust_impl::collision(rust_model, &fixture.env, &fixture.acm, joint_values) {
        Ok(a) => a,
        Err(e) => return (Verdict::Fail(e), f64::NAN),
    };

    if actual.self_collision != expected.self_collision {
        return (
            Verdict::Fail(format!(
                "self_collision differs: oracle {} (distance {:.17e}) vs rust {} (distance {:.17e})",
                expected.self_collision,
                expected.self_distance,
                actual.self_collision,
                actual.self_distance
            )),
            f64::NAN,
        );
    }
    if actual.robot_collision != expected.robot_collision {
        return (
            Verdict::Fail(format!(
                "robot_collision differs: oracle {} (distance {:.17e}) vs rust {} (distance {:.17e})",
                expected.robot_collision,
                expected.robot_distance,
                actual.robot_collision,
                actual.robot_distance
            )),
            f64::NAN,
        );
    }

    let self_dev = (expected.self_distance - actual.self_distance).abs();
    let robot_dev = (expected.robot_distance - actual.robot_distance).abs();
    // NaN must win over any finite max so a NaN deviation cannot hide behind
    // an earlier, smaller finite one.
    let max_dev = if self_dev.is_nan() || robot_dev.is_nan() {
        f64::NAN
    } else {
        self_dev.max(robot_dev)
    };

    pair_stats.self_total += 1;
    if distance_pair_matches(&expected.self_distance_pair, &actual.self_distance_pair) {
        if self_dev.is_nan() || self_dev > cfg.tol_distance {
            pair_stats.self_same_pair_and_value_diverges += 1;
            // The pair matched, so either side's name identifies the same
            // physical pair; `actual` is used only because it is always
            // present when `expected` is (never the reverse in practice).
            if let Some(pair) = &actual.self_distance_pair {
                *pair_stats
                    .self_same_pair_histogram
                    .entry(pair_key(pair))
                    .or_insert(0) += 1;
            }
        }
    } else {
        pair_stats.self_pair_disagrees += 1;
        if self_dev.is_nan() || self_dev > cfg.tol_distance {
            pair_stats.self_pair_flip_and_value_diverges += 1;
        }
    }
    pair_stats.robot_total += 1;
    if distance_pair_matches(&expected.robot_distance_pair, &actual.robot_distance_pair) {
        if robot_dev.is_nan() || robot_dev > cfg.tol_distance {
            pair_stats.robot_same_pair_and_value_diverges += 1;
            if let Some(pair) = &actual.robot_distance_pair {
                *pair_stats
                    .robot_same_pair_histogram
                    .entry(pair_key(pair))
                    .or_insert(0) += 1;
            }
        }
    } else {
        pair_stats.robot_pair_disagrees += 1;
        if robot_dev.is_nan() || robot_dev > cfg.tol_distance {
            pair_stats.robot_pair_flip_and_value_diverges += 1;
        }
    }

    if max_dev.is_nan() || max_dev > cfg.tol_distance {
        return (
            Verdict::Fail(format!(
                "distance differs: self oracle {:.17e} [{}] vs rust {:.17e} [{}] (|d|={self_dev:.3e}), \
                 robot oracle {:.17e} [{}] vs rust {:.17e} [{}] (|d|={robot_dev:.3e}), tol {:.3e}",
                expected.self_distance,
                format_distance_pair(&expected.self_distance_pair),
                actual.self_distance,
                format_distance_pair(&actual.self_distance_pair),
                expected.robot_distance,
                format_distance_pair(&expected.robot_distance_pair),
                actual.robot_distance,
                format_distance_pair(&actual.robot_distance_pair),
                cfg.tol_distance
            )),
            max_dev,
        );
    }
    (Verdict::Pass, max_dev)
}

/// Formats a [`DistancePair`] for `compare_collision`'s `distance differs`
/// message -- `"link_a(robot_link)/link_b(world_object)"`, or `"none"` when
/// upstream's `DistanceResultsData::clear()` state was never overwritten. A
/// message that reports two disagreeing scalars without saying which pair
/// produced them is what left pr2 case 7552 unnamed for three rounds (see
/// `protocol.rs`'s [`DistancePair`] doc).
fn format_distance_pair(pair: &Option<DistancePair>) -> String {
    match pair {
        Some(p) => format!(
            "{}({})/{}({})",
            p.body_name_1, p.body_type_1, p.body_name_2, p.body_type_2
        ),
        None => "none".to_owned(),
    }
}

/// Accumulated across every `ik[case]` in a run -- Phase 4's own completion
/// condition is a rate over many cases, not a per-case verdict (see
/// `Op::Ik`'s doc comment), so these counts are tallied alongside, not
/// instead of, `compare_ik`'s per-case correctness verdict.
#[derive(Debug, Default, serde::Serialize)]
struct IkStats {
    total: usize,
    oracle_success: usize,
    rust_success: usize,
    /// A converged solution within this of the seed, elementwise max-norm --
    /// catches a solver that "succeeds" by returning its seed unmoved.
    ///
    /// Confirmed reading `0` on a real 300-case `panda_arm` sweep
    /// (`--group panda_arm --ik --cases 300 --seed 1`, 297 rust successes,
    /// 294 oracle successes): `--stats-json` reported
    /// `"rust_degenerate": 0, "oracle_degenerate": 0`. Expected, given
    /// [`IK_DEGENERATE_EPS`]'s own measured basis -- every real convergence
    /// observed there left `1e-6` at least `5.5` decades of headroom, so a
    /// normal sweep should never trip this counter. That this field can
    /// still register non-zero on a real (not stubbed) solve is pinned by
    /// `degenerate_counter_reachability_tests::
    /// a_case_already_at_the_seed_pose_converges_to_the_seed_unmoved`: a
    /// case whose target pose already equals `FK(seed)` converges on
    /// `cart_to_jnt`'s very first iteration, before `q_full` is perturbed,
    /// so the returned solution is the seed's own values read straight
    /// back. `rust_impl::IkSolver` is a concrete struct, not a trait object
    /// (`compare_ik` takes `&mut rust_impl::IkSolver<'_>` by name), so there
    /// is no seam to substitute a stub solver without adding a test-only
    /// abstraction the tool does not otherwise have -- this
    /// already-real path made that seam unnecessary.
    rust_degenerate: usize,
    oracle_degenerate: usize,
    /// McNemar's `b`: oracle solved this case, rust did not. Marginal totals
    /// (`oracle_success` vs `rust_success`) cannot distinguish a real gap
    /// from restart-RNG noise on the *same* 5,000 targets -- two solvers
    /// disagreeing on different subsets of cases in each direction can
    /// produce an identical marginal gap to one where every disagreement
    /// runs the same way. `b`/`c` are the paired counts that do
    /// distinguish them: `b ≈ c` means noise regardless of the marginal
    /// gap's size, `b >> c` means a real effect.
    oracle_only: usize,
    /// McNemar's `c`: rust solved this case, oracle did not.
    rust_only: usize,
}

/// McNemar's normal-approximation test statistic for the paired counts
/// `b`/`c`: `0.0` when both are zero (no disagreement to judge), otherwise
/// `|b - c| / sqrt(b + c)`.
fn paired_divergence_z(oracle_only: usize, rust_only: usize) -> f64 {
    let b = oracle_only as f64;
    let c = rust_only as f64;
    if b + c == 0.0 {
        0.0
    } else {
        (b - c).abs() / (b + c).sqrt()
    }
}

/// Above this, `b`/`c` are lopsided enough that `paired_divergence_z` calls
/// it a real divergence rather than restart-RNG noise, and [`run`] turns the
/// IK summary into a failing case instead of a line nobody has to read.
///
/// A round-5 `p1-joints` regression is why this exists: the oracle's own IK
/// reseed loop sampled full-range regardless of `consistency_limits`, so
/// under a tight limit almost every reseed landed too far from the seed to
/// pass its own consistency check even when it converged. Three of the four
/// fixtures showed it clearly (panda `b=15, c=69` -> `z=5.89`; fanuc
/// `b=18, c=92` -> `z=7.06`; dual_arm_panda `b=20, c=67` -> `z=5.04`); pr2
/// (`b=8, c=18` -> `z=1.96`) had too few total disagreements at 500 cases to
/// clear a `3.0` bar even with the bug present, since its baseline success
/// rate was high enough (99.6-99.8%) that few cases ever needed a reseed at
/// all -- a real limitation of this statistic at low `b + c`, not evidence
/// pr2 was unaffected. After fixing that reseed to match
/// `searchPositionIK`'s own near-by-when-limited branch
/// (`kdl_kinematics_plugin.cpp:373-382`), the same four fixtures gave `z`
/// between `0.0` and `1.21` at the same seed. `3.0` sits comfortably above
/// the three clear pre-fix values and comfortably above every post-fix one,
/// so a genuine recurrence of this class of bug on a fixture with enough
/// disagreements to measure cannot be reported as agreement again by a
/// report that only reads success-rate lines or `failed: 0`.
const PAIRED_DIVERGENCE_Z_THRESHOLD: f64 = 3.0;

/// Minimum `b + c` for a `z` at or under [`PAIRED_DIVERGENCE_Z_THRESHOLD`] to
/// be read as "no divergence found" rather than "not enough disagreements to
/// tell". Below this, `z <= 3.0` is exactly the failure mode
/// `ik_paired_divergence` exists to catch one level up: a printed pass that
/// a reader takes for agreement.
///
/// `z = |b - c| / sqrt(b + c)`, so for a fixed true skew `p` (the fraction of
/// disagreements landing on the larger side, `p > 0.5`), its expectation
/// grows with `sqrt(b + c)`: `E[z] ~= (2p - 1) * sqrt(b + c)`. Solving
/// `(2p - 1) * sqrt(n) = 3.0` for `n` gives the `b + c` this gate needs to
/// expect a `z` right at the threshold for a skew of `p`.
///
/// `p` has to come from somewhere real, not an assumed number: the smallest
/// skew this project has directly confirmed as a genuine (not noise) bug is
/// pr2's own round-5 pre-fix pair, `b = 8, c = 18` (`p = 18/26 = 9/13`,
/// `2p - 1 = 5/13`) -- the same reseed bug that panda/fanuc/dual_arm_panda
/// showed at `z` between `5.04` and `7.06`, just with fewer of pr2's cases
/// ever needing a reseed at all (see `PAIRED_DIVERGENCE_Z_THRESHOLD`'s doc
/// comment). Calibrating against that, rather than an arbitrary split, means
/// this floor is "big enough to have caught the smallest real bug seen so
/// far", not a guess:
///
/// `n = (3.0 / (5/13))^2 = (39/5)^2 = 60.84`, rounded up to `61`.
///
/// pr2's own pre-fix run had `b + c = 26 < 61` -- exactly why its `z = 1.96`
/// stayed under `3.0` despite the bug being real, and exactly the case this
/// constant now flags as `Verdict::Underpowered` instead of `Verdict::Pass`.
const MINIMUM_USABLE_B_PLUS_C: usize = 61;

/// How close counts as "did not move from the seed" for the degenerate
/// check -- far looser than `tol_ik`, since this is about catching a solver
/// that took no real step at all, not measuring numerical precision. This is
/// not a ported constant -- upstream's `KDLKinematicsPlugin`/
/// `ChainIkSolverVelMimicSVD` have no "did not move" concept at all
/// (confirmed by `rg -n "degenerate"` over `kdl_kinematics_plugin.cpp`: no
/// hits), so there is no oracle value to match. This value, and the
/// diagnostic itself, are this tool's own invention for catching a bug class
/// (a solver that returns its seed rather than actually iterating) that
/// upstream's own test suite has no name for either.
///
/// `IK_DEGENERATE_EPS` gates `stats.rust_degenerate`/`stats.oracle_degenerate`
/// (`IkStats`, `#[derive(Serialize)]`), which land in `--stats-json` verbatim
/// -- a number read by whoever runs a sweep, not just a print statement, so
/// its value has to be defensible, not just "small".
///
/// Measured against 2952 successful `panda_arm` solves (`--urdf
/// fixtures/panda.urdf --group panda_arm --ik --cases 3000 --seed 1`,
/// instrumented to print each case's own elementwise max-`|solved - seed|` --
/// exactly the quantity `IK_DEGENERATE_EPS` gates, since "every joint moved
/// less than `IK_DEGENERATE_EPS`" is equivalent to "the worst-moved joint
/// moved less than `IK_DEGENERATE_EPS`"): the smallest max-per-case value
/// across all 2952 cases was `3.414642e-01`, roughly `5.5` decades above
/// `1e-6`. Every real convergence this sweep produced left `1e-6` with wide
/// headroom on both sides -- loose enough that `NewtonRaphsonSolver`'s own
/// floating-point iteration noise near a genuine near-seed convergence would
/// not spuriously clear `>= IK_DEGENERATE_EPS`, tight enough that no
/// legitimate solve observed here came remotely close to tripping it by
/// coincidence. Not a claim that `1e-6` is the *only* value with this
/// property -- anything between roughly `1e-6` and `1e-2` would show the
/// same headroom against this sweep -- only that `1e-6` is inside that band,
/// not an arbitrary guess outside it.
const IK_DEGENERATE_EPS: f64 = 1e-6;

/// The shared "did not move from the seed" test behind both
/// `stats.rust_degenerate` and `stats.oracle_degenerate`: true only when
/// *every* joint's solved value sits within [`IK_DEGENERATE_EPS`] of its
/// seed value (elementwise max-norm, not Euclidean -- one joint that moved
/// enough is sufficient to call the whole solution non-degenerate,
/// regardless of how many others did not move at all).
fn is_degenerate_from_seed(solution: &[f64], seed: &[f64]) -> bool {
    solution
        .iter()
        .zip(seed)
        .all(|(&v, &s)| (v - s).abs() < IK_DEGENERATE_EPS)
}

/// Compares one `ik[case]`. Per-case failure is reserved for what upstream
/// itself would call a bug: a converged solution whose own FK misses the
/// target by more than `cfg.tol_ik`. A side simply not converging is not a
/// failure -- that is the success-rate statistic `stats` accumulates
/// instead, reported once at the end of the run, as numbers rather than a
/// verdict (see `Op::Ik`'s doc comment).
fn compare_ik(
    cfg: &Config,
    solver: &mut rust_impl::IkSolver<'_>,
    joint_values: &BTreeMap<String, f64>,
    consistency_limits: &BTreeMap<String, f64>,
    expected: &IkResult,
    stats: &mut IkStats,
) -> Verdict {
    stats.total += 1;

    let outcome = match solver.solve_case(joint_values, consistency_limits) {
        Ok(o) => o,
        Err(e) => return Verdict::Fail(e),
    };

    match (expected.success, outcome.solution.is_some()) {
        (true, false) => stats.oracle_only += 1,
        (false, true) => stats.rust_only += 1,
        (true, true) | (false, false) => {}
    }

    if expected.success {
        stats.oracle_success += 1;
        if let Some(solution) = &expected.solution {
            // `solution` crossed the wire from the oracle process, keyed by
            // name rather than position, so a name it omits is a real
            // possibility to handle, not just a same-process invariant.
            // `f64::INFINITY` keeps that joint from ever reading as "did
            // not move", matching the original `is_some_and(...)` -> false
            // behaviour this replaces.
            let positional: Vec<f64> = outcome
                .joint_names
                .iter()
                .map(|name| solution.get(name).copied().unwrap_or(f64::INFINITY))
                .collect();
            if is_degenerate_from_seed(&positional, &outcome.seed) {
                stats.oracle_degenerate += 1;
            }
        }
    }

    let Some(solution) = &outcome.solution else {
        return Verdict::Pass;
    };
    stats.rust_success += 1;

    if is_degenerate_from_seed(solution, &outcome.seed) {
        stats.rust_degenerate += 1;
    }

    let (translation_error, rotation_error) = outcome
        .errors
        .expect("IkOutcome::errors is Some whenever IkOutcome::solution is");
    if translation_error.is_nan() || translation_error > cfg.tol_ik {
        return Verdict::Fail(format!(
            "rust converged but FK(solution) translation error {translation_error:e} > {:e}",
            cfg.tol_ik
        ));
    }
    if rotation_error.is_nan() || rotation_error > cfg.tol_ik {
        return Verdict::Fail(format!(
            "rust converged but FK(solution) rotation error {rotation_error:e} rad > {:e}",
            cfg.tol_ik
        ));
    }
    Verdict::Pass
}

/// Print per-case lines and the summary. Returns the failure count.
///
/// `Verdict::Underpowered` is not counted in `failed` -- it asserts nothing
/// wrong was found, only that this run could not have told either way -- but
/// it prints as its own `UNDERPOWERED` line rather than folding into `PASS`,
/// so it survives being read at a glance.
/// Prints the standard PASS/FAIL/UNDERPOWERED report and returns
/// `(failures, underpowered)`, the two counts [`RunStats`] cannot derive
/// from `verdicts.len()` alone.
fn report(verdicts: &[(String, Verdict)]) -> Result<(usize, usize), String> {
    let mut failures = 0usize;
    let mut underpowered = 0usize;
    // Distinct failure messages, so 1,000 identical "unimplemented" lines
    // collapse to one with a count instead of burying the real disagreements.
    let mut by_message: BTreeMap<&str, (usize, &str)> = BTreeMap::new();

    for (name, verdict) in verdicts {
        match verdict {
            Verdict::Pass => println!("PASS {name}"),
            Verdict::Underpowered(msg) => {
                underpowered += 1;
                println!("UNDERPOWERED {name}: {msg}");
            }
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
    println!("passed: {}", verdicts.len() - failures - underpowered);
    println!("failed: {failures}");
    if underpowered > 0 {
        println!("underpowered: {underpowered}");
    }
    if failures > 0 {
        println!("distinct failure messages:");
        for (msg, (count, first)) in &by_message {
            println!("  {count:>6}x  (first: {first})  {msg}");
        }
    }
    Ok((failures, underpowered))
}

#[cfg(test)]
mod paired_divergence_tests {
    use super::*;

    /// `b == c == 0` is "nothing to judge", not `0.0 / 0.0`.
    #[test]
    fn zero_paired_counts_give_a_zero_statistic() {
        assert_eq!(paired_divergence_z(0, 0), 0.0);
    }

    /// The threshold's own boundary: the round-5 post-fix measurements
    /// (panda 31/36, fanuc 32/32, dual_arm_panda 35/45, pr2 17/12) all land
    /// under `PAIRED_DIVERGENCE_Z_THRESHOLD`; the round-5 pre-fix ones
    /// (panda 15/69, fanuc 18/92, dual_arm_panda 20/67) all land over it.
    /// pr2's pre-fix pair (8/18) is deliberately not asserted here -- it did
    /// not clear the threshold either (see `PAIRED_DIVERGENCE_Z_THRESHOLD`'s
    /// doc comment on why a low `b + c` limits this statistic's power).
    #[test]
    fn measured_noise_stays_under_threshold_measured_bug_clears_it() {
        for (b, c) in [(31, 36), (32, 32), (35, 45), (17, 12)] {
            let z = paired_divergence_z(b, c);
            assert!(
                z <= PAIRED_DIVERGENCE_Z_THRESHOLD,
                "b={b}, c={c}: z={z} should be noise (<= threshold)"
            );
        }
        for (b, c) in [(15, 69), (18, 92), (20, 67)] {
            let z = paired_divergence_z(b, c);
            assert!(
                z > PAIRED_DIVERGENCE_Z_THRESHOLD,
                "b={b}, c={c}: z={z} should clear the threshold (real divergence)"
            );
        }
    }

    /// `b`/`c` symmetric contribution: swapping which side is `b` and which
    /// is `c` must not change the statistic, since the test is two-sided.
    #[test]
    fn statistic_is_symmetric_in_b_and_c() {
        assert_eq!(paired_divergence_z(15, 69), paired_divergence_z(69, 15));
    }

    /// The exact case `MINIMUM_USABLE_B_PLUS_C` was calibrated against:
    /// pr2's round-5 pre-fix pair sits under both the z threshold and the
    /// power floor, which is precisely why it must read as `Underpowered`
    /// rather than `Pass`.
    #[test]
    fn pr2_pre_fix_pair_is_below_both_the_threshold_and_the_power_floor() {
        let (b, c) = (8, 18);
        assert!(paired_divergence_z(b, c) <= PAIRED_DIVERGENCE_Z_THRESHOLD);
        assert!(b + c < MINIMUM_USABLE_B_PLUS_C);
    }

    /// Locks the constant to its own derivation: `n` such that a skew of
    /// `p = 9/13` (pr2's round-5 pre-fix `b=8, c=18`) gives an expected `z`
    /// of exactly `PAIRED_DIVERGENCE_Z_THRESHOLD`, rounded up. If this drifts
    /// from `MINIMUM_USABLE_B_PLUS_C`, the doc comment's derivation and the
    /// constant have gone out of sync.
    #[test]
    fn minimum_usable_b_plus_c_matches_its_own_derivation() {
        let p = 9.0 / 13.0;
        let n_min = (PAIRED_DIVERGENCE_Z_THRESHOLD / (2.0 * p - 1.0)).powi(2);
        assert_eq!(MINIMUM_USABLE_B_PLUS_C, n_min.ceil() as usize);
    }

    /// The three round-5 post-fix pairs with enough disagreements clear the
    /// floor and must not be misread as underpowered. pr2's own post-fix
    /// pair (`b=17, c=12`, `n=29`) is deliberately excluded and asserted the
    /// other way: `n=29 < 61` means this gate now reports pr2's post-fix run
    /// itself as `Underpowered`, not `Pass` -- an honest consequence of a
    /// floor calibrated against a real effect size, not a reason to lower
    /// the floor until pr2 clears it.
    #[test]
    fn post_fix_pairs_with_enough_n_clear_the_power_floor_pr2_does_not() {
        for (b, c) in [(31, 36), (32, 32), (35, 45)] {
            assert!(
                b + c >= MINIMUM_USABLE_B_PLUS_C,
                "b={b}, c={c}: b+c={} should clear the power floor",
                b + c
            );
        }
        let (b, c) = (17, 12);
        assert!(b + c < MINIMUM_USABLE_B_PLUS_C);
    }
}

#[cfg(test)]
mod is_degenerate_from_seed_tests {
    use super::*;

    /// The measured basis for `IK_DEGENERATE_EPS`: the smallest observed
    /// elementwise max-`|solved - seed|` across 2952 real `panda_arm` solves
    /// was `3.414642e-01` (see `IK_DEGENERATE_EPS`'s own doc comment) --
    /// roughly `5.5` decades above the threshold. A solution that moved by
    /// that much, the smallest real movement this sweep produced, must not
    /// read as degenerate; this is the floor `IK_DEGENERATE_EPS` has to stay
    /// under to keep matching that measurement.
    #[test]
    fn the_smallest_measured_real_movement_does_not_read_as_degenerate() {
        let seed = [0.0, 0.0];
        let smallest_measured_movement = [3.414_642e-1, 0.0];
        assert!(!is_degenerate_from_seed(&smallest_measured_movement, &seed));
    }

    /// A solution identical to its seed in every joint is the textbook
    /// degenerate case this diagnostic exists to catch.
    #[test]
    fn a_solution_identical_to_its_seed_is_degenerate() {
        let seed = [0.1, -0.2, 0.3];
        assert!(is_degenerate_from_seed(&seed, &seed));
    }

    /// Elementwise max-norm, not Euclidean or mean: one joint moved far
    /// enough is sufficient to call the whole solution non-degenerate, no
    /// matter how many other joints sit exactly on their seed value. A mean-
    /// or sum-based check would still read this as degenerate, since one
    /// large diff among many zeros can average back under the threshold.
    #[test]
    fn one_joint_moving_past_the_threshold_is_enough_to_disqualify_the_whole_solution() {
        let seed = [0.0; 8];
        let mut solution = [0.0; 8];
        solution[3] = 1.0;
        assert!(!is_degenerate_from_seed(&solution, &seed));
    }

    /// `IK_DEGENERATE_EPS` itself is strict `<`, matching upstream `Pose::
    /// distance`/`configDistance2`'s own strict comparisons ported in
    /// `ik_cache.rs` (`1eabb2b`): exactly at the threshold does not count as
    /// "did not move", only strictly under it does.
    #[test]
    fn exactly_at_the_threshold_is_not_degenerate() {
        let seed = [0.0];
        let solution = [IK_DEGENERATE_EPS];
        assert!(!is_degenerate_from_seed(&solution, &seed));
    }
}

/// Whether `stats.rust_degenerate`/`stats.oracle_degenerate` can ever read
/// non-zero outside of a hand-built unit input: a real sweep's own solver is
/// `rust_impl::IkSolver`, a concrete struct hard-wired to
/// `NewtonRaphsonSolver` (`rust_impl.rs:487`), not a trait object --
/// `compare_ik` takes `&mut rust_impl::IkSolver<'_>` by concrete type, so
/// there is no seam to substitute a stub "returns its seed unmoved" solver
/// without adding a test-only abstraction that does not otherwise exist.
///
/// A stub is unnecessary, though: `cart_to_jnt`'s own convergence check
/// (`cart_to_jnt.rs:181`, `delta_twist_norm <= params.epsilon`) can pass on
/// attempt 0's very first iteration, before `q_full` is perturbed at all --
/// if the case's target pose is already exactly `FK(seed)`, the returned
/// solution is read straight back from the seed's own values
/// (`cart_to_jnt.rs:182-192`), bit-for-bit unmodified. That is a real,
/// unmodified solver path, reachable by choosing the case itself rather
/// than the solver.
#[cfg(test)]
mod degenerate_counter_reachability_tests {
    use super::*;

    /// Builds panda_arm's `IkSolver` (bounds-midpoint seed, computed the
    /// same way `IkSolver::new` computes it -- `rust_impl.rs:512-521`) and
    /// hands `solve_case` a case whose `joint_values` are that same
    /// bounds-midpoint configuration by name, so `chain_relative_pose`
    /// computes the identical target the solver's own seed already sits at.
    /// panda_arm has no mimic joints, so `chain_joint_names` (every
    /// single-DOF joint, active and mimic) and the solver's own reduced
    /// active-joint set name exactly the same joints.
    ///
    /// Reads `moveit-kinematics/tests/fixtures/panda.urdf`/`.srdf` directly
    /// rather than duplicating them into this crate's own fixtures.
    #[test]
    fn a_case_already_at_the_seed_pose_converges_to_the_seed_unmoved() {
        let fixtures_dir = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../crates/moveit-kinematics/tests/fixtures"
        );
        let urdf_path = format!("{fixtures_dir}/panda.urdf");
        let srdf_path = format!("{fixtures_dir}/panda.srdf");
        let urdf_xml =
            std::fs::read_to_string(&urdf_path).unwrap_or_else(|e| panic!("read panda.urdf: {e}"));
        let urdf =
            urdf_rs::read_file(&urdf_path).unwrap_or_else(|e| panic!("parse panda.urdf: {e}"));
        let srdf = SrdfModel::parse_file(&srdf_path).expect("parse panda.srdf");
        let model =
            RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
                .expect("build panda RobotModel");

        let mut solver =
            rust_impl::IkSolver::new(&model, "panda_arm", false, 0).expect("construct IkSolver");

        let joint_values: BTreeMap<String, f64> = solver
            .chain_joint_names()
            .into_iter()
            .map(|name| {
                let bounds = &model
                    .joint_model(&name)
                    .expect("chain_joint_names names a real model joint")
                    .variable_bounds()[0];
                (name, (bounds.min_position + bounds.max_position) / 2.0)
            })
            .collect();

        let outcome = solver
            .solve_case(&joint_values, &BTreeMap::new())
            .expect("solve_case");
        let solution = outcome
            .solution
            .expect("target already at the seed's own pose must converge on attempt 0");

        assert_eq!(solution, outcome.seed);
        assert!(is_degenerate_from_seed(&solution, &outcome.seed));
    }
}

#[cfg(test)]
mod distance_pair_tests {
    use super::*;

    /// Each body carries its own `(name, type)` pair -- `pair_key`/
    /// `distance_pair_matches` compare `(name, type)` tuples, not names
    /// alone, so a helper that swapped only names while leaving
    /// `body_type_1`/`body_type_2` fixed to their old positions would build
    /// a self-contradictory `DistancePair` (a body that is simultaneously
    /// `robot_link` at one position and `world_object` at the other)
    /// instead of a genuinely reordered one.
    fn pair(name1: &str, type1: &str, name2: &str, type2: &str) -> DistancePair {
        DistancePair {
            body_name_1: name1.to_owned(),
            body_type_1: type1.to_owned(),
            body_name_2: name2.to_owned(),
            body_type_2: type2.to_owned(),
        }
    }

    const BELLOW: (&str, &str) = ("base_bellow_link", "robot_link");
    const TORSO: (&str, &str) = ("torso_lift_link", "robot_link");

    /// The boundary [`pair_key`] exists to close: the same physical pair
    /// named in opposite `body_name_1`/`body_name_2` order must hash to one
    /// histogram entry, not two.
    #[test]
    fn pair_key_is_order_independent() {
        assert_eq!(
            pair_key(&pair(BELLOW.0, BELLOW.1, TORSO.0, TORSO.1)),
            pair_key(&pair(TORSO.0, TORSO.1, BELLOW.0, BELLOW.1))
        );
    }

    /// Two genuinely different pairs must not collide onto the same key.
    #[test]
    fn pair_key_distinguishes_different_pairs() {
        assert_ne!(
            pair_key(&pair(BELLOW.0, BELLOW.1, TORSO.0, TORSO.1)),
            pair_key(&pair(
                "base_link",
                "robot_link",
                "fl_caster_l_wheel_link",
                "robot_link"
            ))
        );
    }

    /// `distance_pair_matches`'s own order-independence -- the property
    /// [`pair_key`] must stay consistent with, or a same-pair case could be
    /// tallied under `*_same_pair_and_value_diverges` yet land in the
    /// histogram under two different keys depending on which side is `a`.
    #[test]
    fn distance_pair_matches_ignores_body_order() {
        let forward = pair(BELLOW.0, BELLOW.1, TORSO.0, TORSO.1);
        let reversed = pair(TORSO.0, TORSO.1, BELLOW.0, BELLOW.1);
        assert!(distance_pair_matches(
            &Some(forward.clone()),
            &Some(forward)
        ));
        assert!(distance_pair_matches(
            &Some(pair(BELLOW.0, BELLOW.1, TORSO.0, TORSO.1)),
            &Some(reversed)
        ));
    }

    #[test]
    fn distance_pair_matches_rejects_a_different_pair() {
        assert!(!distance_pair_matches(
            &Some(pair(BELLOW.0, BELLOW.1, TORSO.0, TORSO.1)),
            &Some(pair(
                "base_link",
                "robot_link",
                "fl_caster_l_wheel_link",
                "robot_link"
            ))
        ));
    }

    /// Same names, but the wrong body carries `world_object`: this must NOT
    /// match, even though every name lines up -- type is part of body
    /// identity, not a spectator field.
    #[test]
    fn distance_pair_matches_rejects_type_only_mismatch() {
        let robot_vs_world = pair(BELLOW.0, "robot_link", TORSO.0, "world_object");
        let both_robot = pair(BELLOW.0, "robot_link", TORSO.0, "robot_link");
        assert!(!distance_pair_matches(
            &Some(robot_vs_world),
            &Some(both_robot)
        ));
    }

    /// Upstream's `DistanceResultsData::clear()` state on both sides (no
    /// other body existed to measure against) is agreement, not a mismatch.
    #[test]
    fn distance_pair_matches_treats_both_none_as_agreement() {
        assert!(distance_pair_matches(&None, &None));
    }

    /// One side reporting a pair and the other reporting none can never be a
    /// tie -- this is the `_ => false` arm `(None, None)`/`(Some, Some)`
    /// leave uncovered.
    #[test]
    fn distance_pair_matches_rejects_one_sided_none() {
        let p = Some(pair(BELLOW.0, BELLOW.1, TORSO.0, TORSO.1));
        assert!(!distance_pair_matches(&p, &None));
        assert!(!distance_pair_matches(&None, &p));
    }
}

/// Round-15 Task 1: pr2's `visibility_cone` distance mismatch (115/2201,
/// `PORTING-PLAN.md` §37/§38.3/§75.4; re-measured this round, still 115/2201
/// against the current oracle stamp) is **neither a case-generation bug in
/// this crate nor a `decide_cone` judgment bug in `moveit-constraints`.**
/// Both were suspects; both are cleared by direct comparison below. It is
/// `moveit-collision`'s already-documented, already-accepted "deviation 6"
/// (`crates/moveit-collision/src/parry.rs`'s module doc, presentations
/// (a)/(b)/(c)): FCL's non-convex penetration depth is itself an
/// approximation, this backend (`parry3d_f64`) computes an independent
/// approximation of the same ill-posed quantity, and the two need not agree
/// -- surfacing here through `decide_cone`'s single-contact visibility
/// check.
///
/// Two hypotheses were tested and rejected in order:
///
/// 1. **Ambiguous-scene / `BTreeMap` tie-break.** `decide_cone`'s
///    `by_pair.values().next()` (`moveit-collision`'s
///    `BTreeMap<(String, String), Vec<Contact>>`) picks whichever colliding
///    link name sorts alphabetically first, not the nearest or deepest --
///    and upstream's own FCL manager, asked for the same `max_contacts: 1`,
///    likewise just returns whichever pair its own broad-phase happens to
///    visit first. If `build_constraint_case`'s near-placement ever handed
///    the collision check more than one simultaneously-touching candidate,
///    the two arbitrary tie-break policies could pick different pairs with
///    no defect on either side. **Rejected**: both tests below (raising
///    `max_contacts` to 64 well past `decide_cone`'s real `1`) show the
///    real failing scenes are never ambiguous -- exactly one link touches
///    the cone, every time. There is nothing to tie-break.
/// 2. **A construction bug in case-generation or `decide_cone`.** Checked
///    the entire input chain against upstream line by line: `build_
///    constraint_case`'s `Some(link_name)` arm's radius/offset sampling
///    against `VisibilityConstraint::configure`'s `points_` formula
///    (`kinematic_constraint.cpp:834-843`); `cone_mesh`'s vertex/triangle
///    indices against `getVisibilityCone` (`kinematic_constraint.cpp:
///    922-993`, vertex 0 sensor / 1 target-center / `2..cone_sides+2` rim,
///    same closing-triangle pair); `decide_cone`'s `World`/`ACM`/
///    `CollisionRequest` construction against `decide`'s collision-check
///    section (`kinematic_constraint.cpp:1138-1179` -- `CollisionEnvFCL`
///    single-arg ctor, default-constructed ACM, `req.contacts=true;
///    req.max_contacts=1`); `LinkPaddingScale::new()` against
///    `CollisionEnvFCL`'s untracked-link default (`env.rs`'s
///    `LinkAdjustment::default()`: `padding: 0.0, scale: 1.0`, matching
///    upstream); `CollisionRequest::default()`'s
///    `pad_environment_collisions: true` against upstream's own default
///    (`collision_common.hpp:154`) -- every one of these matches upstream
///    exactly. **Rejected**: nothing in `tools/moveit-diff` or
///    `moveit-constraints` differs from upstream anywhere in this chain.
///
/// With both cleared, a live sweep (seed 4, `--cases 100 --group right_arm
/// --constraints 2000`, matching `PORTING-PLAN.md` §37/§38.3's own
/// reproduction, run 2026-08-04 against oracle stamp `e7d32225310d3278`)
/// gives the actual numeric shape of the 115 mismatches, all in the
/// `distance` field (`satisfied`/`violated` is bit-perfect, 0 mismatches,
/// same as every prior measurement):
///
/// - Case 104 (this module's real-scene test, below): oracle
///   `7.47914550966356367e-2`, rust `2.08696987934593702e-2`. The touched
///   link is `bl_caster_l_wheel_link`, whose collision geometry is a
///   `<cylinder length="0.034" radius="0.074792"/>` (`fixtures/pr2.urdf:
///   501`) -- oracle's reported depth matches that cylinder's own radius to
///   within `5.4e-7` (about 7ppm), while rust's independently-computed
///   depth for the identical single contact does not. Several other
///   mismatches in the same sweep cluster near that same `0.07479...`
///   value (cases 76/174/244/314/398/552/636/1770), consistent with the
///   same wheel-radius-shaped disagreement recurring across pr2's four
///   caster wheels.
/// - Several near-zero cases show a sign disagreement at FCL's own
///   near-touching numerical boundary, not just a magnitude one: e.g. case
///   48, oracle `-5.00000000000000010e-4` vs rust
///   `4.99999999999555921e-4` -- same magnitude, opposite sign, for the
///   same `satisfied=false` verdict.
///
/// Both presentations (a large magnitude gap tied to real shape geometry,
/// and a sign flip at a near-zero depth) are exactly what "two independent
/// approximations of an ill-posed quantity" produces, and neither traces to
/// a formula this crate or `moveit-constraints` controls -- both chains were
/// verified bit-identical to upstream above. This round's brief's premise
/// (p3-acm excluded `moveit-collision` from scope, leaving only
/// case-generation or judgment as the two candidates) does not hold up
/// under this evidence: this is `moveit-collision`'s deviation 6, not a new
/// defect in either crate this module can fix. Not touching
/// `moveit-collision` -- reporting only, per this round's ownership rule.
#[cfg(test)]
mod visibility_cone_ambiguity_diagnostic {
    use std::sync::Arc;

    use moveit_collision::{BodyType, CollisionEnv, CollisionRequest, Contact, DecideContactFn};
    use moveit_geometry::{Mesh, Transforms};
    use moveit_state::RobotState;
    use nalgebra::Point3;

    use super::*;

    /// A read-only copy of `VisibilityConstraint::cone_mesh`'s triangulation
    /// (`moveit-constraints/src/visibility.rs:311-350`, private to that
    /// crate) -- reproduced here only to diagnose the scene this generator
    /// hands it, not called into moveit-constraints.
    fn cone_mesh(
        world_to_sensor: Isometry3,
        world_to_target: Isometry3,
        target_radius: f64,
        cone_sides: usize,
    ) -> Mesh {
        let mut vertices = Vec::with_capacity(cone_sides + 2);
        vertices.push(world_to_sensor.translation.vector);
        vertices.push(world_to_target.translation.vector);
        let delta = 2.0 * std::f64::consts::PI / cone_sides as f64;
        for i in 0..cone_sides {
            let a = delta * i as f64;
            let rim_point_in_target =
                Vector3::new(a.sin() * target_radius, a.cos() * target_radius, 0.0);
            vertices.push((world_to_target * Point3::from(rim_point_in_target)).coords);
        }
        let mut triangles = Vec::with_capacity(cone_sides * 2);
        for i in 1..cone_sides {
            triangles.push([(i + 1) as u32, 0, (i + 2) as u32]);
            triangles.push([(i + 1) as u32, 1, (i + 2) as u32]);
        }
        triangles.push([(cone_sides + 1) as u32, 0, 2]);
        triangles.push([(cone_sides + 1) as u32, 1, 2]);
        Mesh::new(vertices, triangles).expect("same construction as visibility.rs's own cone_mesh")
    }

    /// A read-only copy of `allow_sensor_or_target_contact`
    /// (`moveit-constraints/src/visibility.rs:426-449`, private) -- the ACM
    /// policy that excludes the sensor/target link's own necessary touch at
    /// the cone's apex/base-center vertices, so this diagnostic counts only
    /// genuine *extra* candidates, not that expected one.
    fn allow_sensor_or_target_contact(
        sensor_frame: String,
        target_frame: String,
    ) -> DecideContactFn {
        Arc::new(move |contact: &mut Contact| {
            if contact.body_type_1 == BodyType::RobotAttached
                || contact.body_type_2 == BodyType::RobotAttached
            {
                return true;
            }
            if contact.body_type_1 == BodyType::RobotLink
                && contact.body_type_2 == BodyType::WorldObject
                && (Transforms::same_frame(&contact.body_name_1, &sensor_frame)
                    || Transforms::same_frame(&contact.body_name_1, &target_frame))
            {
                return true;
            }
            if contact.body_type_2 == BodyType::RobotLink
                && contact.body_type_1 == BodyType::WorldObject
                && (Transforms::same_frame(&contact.body_name_2, &sensor_frame)
                    || Transforms::same_frame(&contact.body_name_2, &target_frame))
            {
                return true;
            }
            false
        })
    }

    /// Reproduces `decide_cone`'s exact scene (`visibility.rs:381-416`) at
    /// pr2's `base_footprint_joint` default pose, cone anchored at the first
    /// `parry_representable_link_names` link, same radius/offset bounds this
    /// generator uses (`main.rs`'s `Some(link_name)` arm: radius in
    /// `0.005..0.015`, sensor offset `0.005`) -- but with `max_contacts`
    /// raised far past 1, to see every link the cone actually touches, not
    /// just whichever one `max_contacts: 1` reports.
    ///
    /// Pins the module doc's hypothesis-1 rejection: every one of pr2's 17
    /// parry-representable links touches the cone at most once at this
    /// pose, so `decide_cone`'s `max_contacts: 1` pick is never a tie-break
    /// among several candidates here. A future change that made this
    /// generator's near-placement genuinely ambiguous would flip this
    /// assertion, which is exactly the regression this pins against.
    ///
    /// Needs `third_party/moveit_resources` for pr2's real mesh geometry
    /// (see `mesh_search_paths`'s own doc comment for why this tool already
    /// requires it) -- `#[ignore]`d so `cargo nextest run --workspace`
    /// (which has no vendored checkout in CI) does not depend on it.
    #[test]
    #[ignore = "needs third_party/moveit_resources; see this module's doc comment"]
    fn near_placement_never_touches_more_than_one_link_at_once() {
        let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/pr2.urdf");
        let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/pr2.srdf");
        let urdf_xml = std::fs::read_to_string(urdf_path).expect("read pr2.urdf");
        let urdf = urdf_rs::read_file(urdf_path).expect("parse pr2.urdf");
        let srdf = SrdfModel::parse_file(srdf_path).expect("parse pr2.srdf");
        let model = RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &mesh_search_paths())
            .expect("build pr2 RobotModel with real mesh collision geometry");

        let eligible = parry_representable_link_names(&model);
        assert!(
            !eligible.is_empty(),
            "pr2 must have at least one parry-representable link for this diagnostic to mean anything"
        );

        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let posed = state.update();

        let mut touched_link_counts = Vec::new();
        for &link_name in &eligible {
            let link_model = model
                .link_model(link_name)
                .expect("link_name came from parry_representable_link_names(&model)");
            let shape = link_model
                .shapes()
                .iter()
                .find(|s| is_parry_representable(&s.shape))
                .expect("link_name is eligible because it has such a shape");
            let link_fk = posed
                .global_link_transform(link_name)
                .expect("link_name came from model.link_models()");
            let center = (link_fk * shape.origin_transform).translation.vector;

            // Mid of this generator's own `0.005..0.015` radius range and its
            // fixed `0.005` sensor offset -- see `build_constraint_case`'s
            // `Some(link_name)` arm.
            let radius = 0.01;
            let sensor_offset = 0.005;
            let world_to_target = Isometry3::from_parts(center.into(), UnitQuaternion::identity());
            let world_to_sensor = Isometry3::from_parts(
                (center + Vector3::new(0.0, 0.0, sensor_offset)).into(),
                UnitQuaternion::identity(),
            );
            let cone = cone_mesh(world_to_sensor, world_to_target, radius, 6);

            let mut world = World::new();
            world.add_shape("cone", Arc::new(Shape::Mesh(cone)), Isometry3::identity());
            let env = ParryCollisionEnv::new(world, LinkPaddingScale::new());

            let mut acm = AllowedCollisionMatrix::new();
            acm.set_default_conditional_entry(
                "cone",
                allow_sensor_or_target_contact(
                    model.model_frame().to_owned(),
                    model.model_frame().to_owned(),
                ),
            );

            let request = CollisionRequest {
                contacts: true,
                max_contacts: 64,
                max_contacts_per_pair: 1,
                ..Default::default()
            };
            let result = env.check_robot_collision(&request, &posed, &[], Some(&acm));
            let touched = result
                .contacts
                .as_ref()
                .map_or(0, moveit_collision::ContactData::pair_count);
            // Asserted here, per link, rather than as a post-hoc check on the
            // collected `touched_link_counts`: this cone is built directly
            // around `link_name`'s own collision shape, so it touching
            // nothing means collision detection itself has regressed to
            // returning no contacts. Filtering `touched == 0` entries out of
            // the aggregate below and then only checking the aggregate for
            // ambiguity would let exactly that regression go undetected --
            // an empty `touched_link_counts` makes `ambiguous` empty too,
            // and the test would pass vacuously instead of catching it.
            assert!(
                touched > 0,
                "expected pr2 link {link_name}'s own near-placement cone to touch it at least \
                 once -- touched 0, which would otherwise silently empty out this diagnostic's \
                 ambiguous-link check below"
            );
            touched_link_counts.push((link_name, touched));
        }

        let ambiguous: Vec<_> = touched_link_counts
            .iter()
            .filter(|(_, count)| *count > 1)
            .collect();
        eprintln!("links checked: {}", eligible.len());
        eprintln!(
            "links whose near-placement touched >=1 other link: {}",
            touched_link_counts.len()
        );
        eprintln!("links whose near-placement touched >1 other link (ambiguous): {ambiguous:?}");
        assert!(
            ambiguous.is_empty(),
            "expected every near-placement at pr2's default pose to touch at most one link, \
             ruling out decide_cone's max_contacts: 1 tie-break as the source of the 115-case \
             distance mismatch (see this module's doc comment) -- got ambiguous links {ambiguous:?}"
        );
    }

    /// Same question as
    /// [`near_placement_never_touches_more_than_one_link_at_once`], but
    /// against an actual failing case instead of a default-pose stand-in:
    /// `pr2` `--seed 4 --group right_arm --cases 100 --constraints 2000`
    /// case 104 (`joint_values`/`spec` captured verbatim via a temporary
    /// `MOVEIT_DIFF_DEBUG_VISIBILITY_CONE=1` eprintln in
    /// `run_constraint_cases`, then copied here -- oracle reported distance
    /// `7.47914550966356367e-2` there, rust `2.08696987934593702e-2`; see
    /// this module's doc comment for why that gap is not this case being
    /// ambiguous).
    #[test]
    #[ignore = "needs third_party/moveit_resources; see this module's doc comment"]
    fn a_real_mismatching_case_touches_exactly_one_link() {
        let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/pr2.urdf");
        let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/pr2.srdf");
        let urdf_xml = std::fs::read_to_string(urdf_path).expect("read pr2.urdf");
        let urdf = urdf_rs::read_file(urdf_path).expect("parse pr2.urdf");
        let srdf = SrdfModel::parse_file(srdf_path).expect("parse pr2.srdf");
        let model = RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &mesh_search_paths())
            .expect("build pr2 RobotModel with real mesh collision geometry");

        let joint_values: BTreeMap<String, f64> = serde_json::from_str(
            r#"{"bl_caster_l_wheel_joint": -2.451585059798172, "bl_caster_r_wheel_joint": -1.2125751462448606, "bl_caster_rotation_joint": 0.129901095290601, "br_caster_l_wheel_joint": 2.093234081841553, "br_caster_r_wheel_joint": 0.0920718799633682, "br_caster_rotation_joint": 1.156251961941016, "fl_caster_l_wheel_joint": 0.4501360411272022, "fl_caster_r_wheel_joint": -2.331468058637221, "fl_caster_rotation_joint": 2.6978024804506067, "fr_caster_l_wheel_joint": 2.0805852854369835, "fr_caster_r_wheel_joint": -0.07704670772749234, "fr_caster_rotation_joint": 2.1595746971716094, "head_pan_joint": 1.772165230598301, "head_tilt_joint": 0.7787539671446244, "l_elbow_flex_joint": -0.2736173052095341, "l_forearm_roll_joint": 1.0488381119058694, "l_gripper_joint": 0.07618819281458854, "l_gripper_l_finger_joint": 0.2638400489529595, "l_gripper_l_finger_tip_joint": 0.2638400489529595, "l_gripper_motor_screw_joint": 2.685487501470873, "l_gripper_motor_slider_joint": -0.02708936585113407, "l_gripper_r_finger_joint": 0.2638400489529595, "l_gripper_r_finger_tip_joint": 0.2638400489529595, "l_shoulder_lift_joint": 0.7195627860737034, "l_shoulder_pan_joint": 0.8130559688981515, "l_upper_arm_roll_joint": 3.427571022603661, "l_wrist_flex_joint": -1.5513118343194947, "l_wrist_roll_joint": -2.2071143516290372, "laser_tilt_mount_joint": -0.35061450647910364, "r_elbow_flex_joint": -0.21371314669155983, "r_forearm_roll_joint": -0.17202537080433045, "r_gripper_joint": 0.03843543348833919, "r_gripper_l_finger_joint": 0.4854168069222942, "r_gripper_l_finger_tip_joint": 0.4854168069222942, "r_gripper_motor_screw_joint": 0.28511155988088, "r_gripper_motor_slider_joint": 0.023785984655842186, "r_gripper_r_finger_joint": 0.4854168069222942, "r_gripper_r_finger_tip_joint": 0.4854168069222942, "r_shoulder_lift_joint": -0.10111868691151032, "r_shoulder_pan_joint": -1.189628085223248, "r_upper_arm_roll_joint": -2.918286682944745, "r_wrist_flex_joint": -1.3908302708994598, "r_wrist_roll_joint": -0.6736757665340329, "torso_lift_joint": 0.2581471112640574, "torso_lift_motor_screw_joint": -0.8098637376411038, "world_joint/theta": -2.836643659765878, "world_joint/x": 0.0, "world_joint/y": 0.0}"#,
        )
        .expect("captured joint_values must parse");

        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        for (name, &value) in &joint_values {
            state
                .set_variable_position(name, value)
                .unwrap_or_else(|e| panic!("setting {name}: {e}"));
        }
        let posed = state.update();

        // Captured verbatim from case 104's spec.
        let sensor_pose = rust_impl::isometry_from_row_major(&[
            1.0,
            0.0,
            0.0,
            0.30231483312872937,
            0.0,
            1.0,
            0.0,
            -0.1912422727165995,
            0.0,
            0.0,
            1.0,
            0.0842,
            0.0,
            0.0,
            0.0,
            1.0,
        ]);
        let target_pose = rust_impl::isometry_from_row_major(&[
            1.0,
            0.0,
            0.0,
            0.30231483312872937,
            0.0,
            1.0,
            0.0,
            -0.1912422727165995,
            0.0,
            0.0,
            1.0,
            0.07919999999999999,
            0.0,
            0.0,
            0.0,
            1.0,
        ]);
        let radius = 0.007960615621068475;
        let cone_sides = 5;

        let cone = cone_mesh(sensor_pose, target_pose, radius, cone_sides);
        let mut world = World::new();
        world.add_shape("cone", Arc::new(Shape::Mesh(cone)), Isometry3::identity());
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::new());

        let mut acm = AllowedCollisionMatrix::new();
        acm.set_default_conditional_entry(
            "cone",
            allow_sensor_or_target_contact(
                model.model_frame().to_owned(),
                model.model_frame().to_owned(),
            ),
        );

        let request = CollisionRequest {
            contacts: true,
            max_contacts: 64,
            max_contacts_per_pair: 1,
            ..Default::default()
        };
        let result = env.check_robot_collision(&request, &posed, &[], Some(&acm));
        let pairs: Vec<_> = result
            .contacts
            .as_ref()
            .map(|c| c.by_pair.keys().cloned().collect())
            .unwrap_or_default();
        eprintln!(
            "case 104: {} pair(s) touched the cone: {pairs:?}",
            pairs.len()
        );
        assert_eq!(
            pairs.len(),
            1,
            "expected case 104's own real scene to touch exactly one link, ruling out an \
             ambiguous multi-candidate scene as the source of its distance mismatch (see this \
             module's doc comment) -- got {pairs:?}"
        );
    }
}
