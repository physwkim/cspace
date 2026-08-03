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
//!             [--collision] [--tol-distance EPS] [--oracle <cmd> [args...]]
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
    ConstraintsSpec, FkResult, IkResult, JacobianResult, JointConstraintSpec, ModelInfo, Op,
    OracleResult, OrientationConstraintSpec, OrientationToleranceSpec, PositionConstraintSpec,
    Request, Response, ShapeSpec, VisibilityConstraintSpec,
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
            eprintln!("                   [--collision] [--tol-distance EPS]");
            eprintln!(
                "                   [--ik] [--tol-ik EPS] [--ik-position-only] [--ik-max-restarts N]"
            );
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
/// already requires `third_party/` for `tools/ci/run-oracle-sweep.sh`'s own
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
            let (verdict, dev) =
                compare_collision(cfg, &rust_model, fixture, joint_values, &expected);
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
    }

    if cfg.constraints > 0 {
        run_constraint_cases(cfg, &mut oracle, &rust_model, &states, &fks, &mut verdicts)?;
    }
    if cfg.ik {
        println!(
            "\n--- ik summary ({}) ---",
            cfg.group.as_deref().unwrap_or("")
        );
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
/// link, half of this case's occurrences (`(case / 7) % 2 == 0`) place the
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
            let eligible = parry_representable_link_names(model);
            // See this function's doc comment: fixtures with no
            // parry-representable link always take the far branch, and
            // fixtures that have one (pr2) alternate so the split covers
            // both branches of `decide_cone` against the oracle.
            let hit_link = (!eligible.is_empty() && (case / 7) % 2 == 0)
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

/// Compares a `collision` case: `self_collision`/`robot_collision` exactly,
/// `self_distance`/`robot_distance` at `cfg.tol_distance`. Contact/nearest-
/// point coordinates are never compared -- PORTING-PLAN.md §4.5 records that
/// exclusion as Phase 3's recorded verification limit, not an oversight; the
/// two sides' contact geometry differs by construction (module doc,
/// `crates/moveit-collision/src/parry.rs`, deviations 4 and 6) in ways that
/// would never converge under any tolerance.
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

    if max_dev.is_nan() || max_dev > cfg.tol_distance {
        return (
            Verdict::Fail(format!(
                "distance differs: self oracle {:.17e} vs rust {:.17e} (|d|={self_dev:.3e}), \
                 robot oracle {:.17e} vs rust {:.17e} (|d|={robot_dev:.3e}), tol {:.3e}",
                expected.self_distance,
                actual.self_distance,
                expected.robot_distance,
                actual.robot_distance,
                cfg.tol_distance
            )),
            max_dev,
        );
    }
    (Verdict::Pass, max_dev)
}

/// Accumulated across every `ik[case]` in a run -- Phase 4's own completion
/// condition is a rate over many cases, not a per-case verdict (see
/// `Op::Ik`'s doc comment), so these counts are tallied alongside, not
/// instead of, `compare_ik`'s per-case correctness verdict.
#[derive(Default)]
struct IkStats {
    total: usize,
    oracle_success: usize,
    rust_success: usize,
    /// A converged solution within this of the seed, elementwise max-norm --
    /// catches a solver that "succeeds" by returning its seed unmoved.
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

/// How close counts as "did not move from the seed" for the degenerate
/// check -- far looser than `tol_ik`, since this is about catching a solver
/// that took no real step at all, not measuring numerical precision.
const IK_DEGENERATE_EPS: f64 = 1e-6;

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
    expected: &IkResult,
    stats: &mut IkStats,
) -> Verdict {
    stats.total += 1;

    let outcome = match solver.solve_case(joint_values) {
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
            let degenerate =
                outcome
                    .joint_names
                    .iter()
                    .zip(&outcome.seed)
                    .all(|(name, &seed_value)| {
                        solution
                            .get(name)
                            .is_some_and(|&v| (v - seed_value).abs() < IK_DEGENERATE_EPS)
                    });
            if degenerate {
                stats.oracle_degenerate += 1;
            }
        }
    }

    let Some(solution) = &outcome.solution else {
        return Verdict::Pass;
    };
    stats.rust_success += 1;

    let degenerate = solution
        .iter()
        .zip(&outcome.seed)
        .all(|(&v, &s)| (v - s).abs() < IK_DEGENERATE_EPS);
    if degenerate {
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
