// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Extends `penetration_subset.rs`'s defect-free corpus to the two
//! subpopulations PORTING-PLAN.md §5 Phase 3's `distance: f64` row (closed
//! via §283, `distance-callback-threshold-suppresses-deeper-pairs`,
//! `fcl-distance-sentinel-survives-zero-contacts`,
//! `distance-callback-max-contact-depth`) names as unmeasured and requiring
//! an oracle patch to reach: states with **two or more simultaneously
//! penetrating pairs**, and **`box x box`**. Mesh is not covered by either
//! measurement here; see "What this does not cover" below.
//!
//! # Two or more pairs, no oracle change needed
//!
//! `penetration_subset.rs`'s ACM mask (`tools/moveit-diff/src/bin/
//! penetration_subset.rs:591-601`, `mask_diff`) excludes
//! `distance-callback-threshold-suppresses-deeper-pairs` by construction --
//! it leaves exactly one (probe, target) pair visible, so there is never a
//! second pair for the running-minimum threshold to suppress. That same
//! defect is *only* observable with two or more pairs. Reusing the mask
//! twice against one scene -- once per pair, each in isolation -- gives a
//! true per-pair distance neither call's own defect exposure can corrupt
//! (each call still visits exactly one pair), and the minimum of the two is
//! the scene's true multi-pair minimum: the two probes sit on different
//! links with reach bounded well short of overlapping each other's target,
//! and every other pair in the scene is masked away in both calls, so no
//! third pair exists to contend.
//!
//! What is new relative to `penetration_subset.rs` is the query on **this**
//! side: [`multi_pair_port_distance`] builds one ACM that leaves *both*
//! pairs visible and asks `distance_robot` once, the same call
//! `penetration_subset.rs` never exercises with more than one pair
//! unmasked. That is the port's own multi-pair minimum-of-pairs logic, and
//! it is what this measurement is actually testing -- the oracle side is
//! reused exactly as `penetration_subset.rs` already established it,
//! twice per sample instead of once.
//!
//! # `box x box`, via a new oracle op
//!
//! `box x box` is excluded from `penetration_subset.rs`'s corpus because
//! `boxBoxIntersect` can emit more than one contact
//! (`box_box-inl.h:857-874`), which reopens
//! `distance-callback-max-contact-depth`, and because that number of
//! contacts is not bounded the way the three closed-form single-contact
//! pairs are, so a defect-free subset cannot be carved out of it the same
//! way. That is a property of `distanceCallback`'s own workaround
//! (`collision_common.cpp:636-680`), not of FCL's distance machinery in
//! general: `distanceCallback` builds its FCL request as
//! `fcl::DistanceRequestd(cdata->req->enable_nearest_points)`
//! (`collision_common.cpp:603`) -- one positional argument, so FCL's own
//! `DistanceRequest::enable_signed_distance` is left at its default
//! `false` regardless of MoveIt's separately-scoped
//! `cdata->req->enable_signed_distance`, and FCL's native signed-distance
//! path (`ShapeDistanceTraversalNode::leafTesting` dispatching to
//! `nsolver->shapeSignedDistance`, `shape_distance_traversal_node-inl.h:
//! 85-92`) is never reached. That native path -- fcl's own docs call the
//! combination "SD_1", exact signed distance via GJK/EPA
//! (`distance_request.h:68`) -- is real for any primitive pair under
//! `GST_LIBCCD`, `box x box` included: `box_box-inl.h` registers no
//! closed-form *distance* routine, only the `boxBoxIntersect` collision
//! test, so `box x box` distance already falls through to the generic GJK
//! dispatch both with and without this measurement's op.
//!
//! `tools/moveit-oracle/src/oracle.cpp`'s `pair_signed_distance` op calls
//! that native path directly: it builds both FCL objects with MoveIt's own
//! unmodified `createCollisionGeometry`/`transform2fcl` (the same factories
//! `distanceCallback` itself uses) and calls `fcl::distance` with FCL's
//! `enable_signed_distance = true`, bypassing `distanceCallback` and its
//! workaround entirely. Nothing here reimplements FCL's geometry math --
//! see that op's own doc comment for the full citation trail.
//!
//! # What this does not cover
//!
//! Mesh. `pair_signed_distance` does not extend to it: FCL's mesh leaf test,
//! `ShapeMeshDistanceTraversalNode::leafTesting`, calls
//! `nsolver->shapeTriangleDistance` unconditionally
//! (`shape_mesh_distance_traversal_node-inl.h:88`) with no
//! `enable_signed_distance` branch to take, and fcl's own request-flag
//! documentation classifies "mesh and octree" as "SD_2" (positive distance
//! only; negative distance by an external penetration workaround) even when
//! signed distance is requested (`distance_request.h:69`). There is no
//! upstream call this op -- or any other -- can make to get a genuine
//! signed distance for a penetrating mesh pair; reproducing that value from
//! this side of the wire would be deriving it independently rather than
//! comparing against the reference, so mesh stays excluded rather than
//! measured by a second implementation. This is a fact about FCL 0.7.0, not
//! about the oracle's serialization.
//!
//! # Usage
//!
//! ```text
//! penetration-extended --urdf F.urdf --srdf F.srdf \
//!     --oracle tools/moveit-oracle/run-oracle.sh [--states N] [--seed S] [--tol T]
//! ```
//!
//! Each of the two measurements is independently MET or NOT MET; the exit
//! code is nonzero if either disagrees, and a robot offering no target for
//! a given measurement (fewer than two single-primitive links for the
//! multi-pair measurement, no `box` target for `box x box`) is reported and
//! skipped for that measurement rather than silently counted as a pass.

use std::cmp::Ordering;
use std::io::{BufRead, BufReader, Write};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::sync::Arc;

use moveit_collision::{
    AllowedCollisionMatrix, CollisionEnv, DistanceRequest, LinkPaddingScale, ParryCollisionEnv,
    World,
};
use moveit_geometry::{Cuboid, Isometry3, Shape, Sphere};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;
use serde_json::{Value, json};

/// Command line, parsed by hand the way `penetration_subset.rs`'s own
/// `Args` is.
struct Args {
    urdf: String,
    srdf: String,
    oracle: Vec<String>,
    /// Samples drawn per (target pair | box target), matching
    /// `penetration_subset.rs`'s `--states` in spirit though not in scale --
    /// each sample here costs two oracle round trips, not one.
    states: usize,
    seed: i64,
    tol: f64,
}

fn parse_args() -> Result<Args, String> {
    let mut urdf = None;
    let mut srdf = None;
    let mut oracle: Vec<String> = Vec::new();
    let mut states = 40usize;
    let mut seed = 1i64;
    let mut tol = 1e-4f64;

    let mut argv = std::env::args().skip(1);
    while let Some(arg) = argv.next() {
        let want = |name: &str, argv: &mut dyn Iterator<Item = String>| {
            argv.next().ok_or_else(|| format!("{name} needs a value"))
        };
        match arg.as_str() {
            "--urdf" => urdf = Some(want("--urdf", &mut argv)?),
            "--srdf" => srdf = Some(want("--srdf", &mut argv)?),
            "--oracle" => oracle.push(want("--oracle", &mut argv)?),
            "--states" => {
                states = want("--states", &mut argv)?
                    .parse()
                    .map_err(|e| format!("--states: {e}"))?;
            }
            "--seed" => {
                seed = want("--seed", &mut argv)?
                    .parse()
                    .map_err(|e| format!("--seed: {e}"))?;
            }
            "--tol" => {
                tol = want("--tol", &mut argv)?
                    .parse()
                    .map_err(|e| format!("--tol: {e}"))?;
            }
            other => return Err(format!("unknown argument {other}")),
        }
    }

    Ok(Args {
        urdf: urdf.ok_or("--urdf is required")?,
        srdf: srdf.ok_or("--srdf is required")?,
        oracle: if oracle.is_empty() {
            vec!["tools/moveit-oracle/run-oracle.sh".to_owned()]
        } else {
            oracle
        },
        states,
        seed,
        tol,
    })
}

/// The oracle subprocess, as a JSON-lines filter. Identical to
/// `penetration_subset.rs`'s `Oracle`; not shared because each `bin/*.rs`
/// in this crate is self-contained (see `tangency_subset.rs`).
struct Oracle {
    child: Child,
    stdin: Option<ChildStdin>,
    stdout: BufReader<ChildStdout>,
    next_id: u64,
}

impl Oracle {
    fn spawn(args: &Args) -> Result<Self, String> {
        let (program, rest) = args.oracle.split_first().ok_or("empty oracle command")?;
        let mut child = Command::new(program)
            .args(rest)
            .arg("--urdf")
            .arg(&args.urdf)
            .arg("--srdf")
            .arg(&args.srdf)
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

    fn ask(&mut self, mut request: Value) -> Result<Value, String> {
        let id = self.next_id;
        self.next_id += 1;
        request["id"] = json!(id);
        let stdin = self
            .stdin
            .as_mut()
            .ok_or_else(|| format!("oracle stdin already closed at request {id}"))?;
        writeln!(stdin, "{request}").map_err(|e| format!("writing request {id}: {e}"))?;
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
        let resp: Value = serde_json::from_str(buf.trim())
            .map_err(|e| format!("decoding {id}: {e} in {buf:?}"))?;
        if resp["ok"] != json!(true) {
            return Err(format!("oracle error for request {id}: {}", resp["error"]));
        }
        Ok(resp["result"].clone())
    }
}

impl Drop for Oracle {
    fn drop(&mut self) {
        drop(self.stdin.take());
        let _ = self.child.wait();
    }
}

/// One link this corpus can use: exactly one collision element, and that
/// element is a primitive. Identical selection to `penetration_subset.rs`'s
/// `Target`/`targets`, kept separate for the same reason `Oracle` is.
struct Target {
    link: String,
    origin: Isometry3,
    half_extent: [f64; 3],
    kind: &'static str,
}

fn targets(model: &RobotModel) -> Vec<Target> {
    let mut out = Vec::new();
    for link in model.link_models() {
        let shapes = link.shapes();
        if shapes.len() != 1 {
            continue;
        }
        let (kind, half_extent) = match &shapes[0].shape {
            Shape::Sphere(s) => ("sphere", [s.radius, s.radius, s.radius]),
            Shape::Cuboid(b) => ("box", [b.size[0] / 2.0, b.size[1] / 2.0, b.size[2] / 2.0]),
            Shape::Cylinder(c) => ("cylinder", [c.radius, c.radius, c.length / 2.0]),
            _ => continue,
        };
        if half_extent
            .iter()
            .any(|e| e.partial_cmp(&0.0) != Some(Ordering::Greater))
        {
            continue;
        }
        out.push(Target {
            link: link.name().to_owned(),
            origin: shapes[0].origin_transform,
            half_extent,
            kind,
        });
    }
    out
}

fn row_major(pose: &Isometry3) -> Value {
    let m = pose.to_homogeneous();
    let mut out = Vec::with_capacity(16);
    for r in 0..4 {
        for c in 0..4 {
            out.push(m[(r, c)]);
        }
    }
    json!(out)
}

fn apply_joint_values(state: &mut RobotState<'_>, joint_values: &Value) -> Result<(), String> {
    for (name, value) in joint_values
        .as_object()
        .ok_or("joint_values is not an object")?
    {
        let v = value.as_f64().ok_or("joint value is not a number")?;
        state
            .set_variable_position(name, v)
            .map_err(|e| format!("setting {name}: {e}"))?;
    }
    Ok(())
}

fn build_model(args: &Args) -> Result<RobotModel, String> {
    let urdf_xml =
        std::fs::read_to_string(&args.urdf).map_err(|e| format!("reading {}: {e}", args.urdf))?;
    let urdf = urdf_rs::read_file(&args.urdf).map_err(|e| format!("parsing {}: {e}", args.urdf))?;
    let srdf =
        SrdfModel::parse_file(&args.srdf).map_err(|e| format!("parsing {}: {e}", args.srdf))?;
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::default())
        .map_err(|e| format!("building RobotModel: {e}"))
}

/// ACM diff leaving visible exactly the (probe id, link) pairs in `keep`,
/// across every probe id in `probe_ids`. `penetration_subset.rs`'s
/// `mask_diff` is this function specialised to one probe and one kept pair.
fn mask_diff_pairs(link_names: &[String], probe_ids: &[&str], keep: &[(&str, &str)]) -> Value {
    let mut actions = Vec::new();
    for &probe in probe_ids {
        for name in link_names {
            let is_kept = keep.iter().any(|&(p, t)| p == probe && t == name);
            if !is_kept {
                actions.push(
                    json!({"action": "set_acm_entry", "first": probe, "second": name, "allowed": true}),
                );
            }
        }
    }
    json!(actions)
}

/// Upstream leaves `DistanceResultsData::distance` at `DBL_MAX` when no pair
/// is considered; see `penetration_subset.rs`'s `NO_PAIR` for why this only
/// has to separate that from a real distance.
const NO_PAIR: f64 = 1e30;

/// Per-shape-pair or per-measurement tallies, mirroring
/// `penetration_subset.rs`'s `Stats`.
#[derive(Default)]
struct Stats {
    requested: usize,
    separated: usize,
    kept: usize,
    disagrees: usize,
    worst: f64,
    le_1e12: usize,
    le_1e9: usize,
    le_1e6: usize,
    le_tol: usize,
    outliers: Vec<String>,
}

impl Stats {
    const OUTLIERS: usize = 8;

    fn record(&mut self, deviation: f64, tol: f64, describe: impl FnOnce() -> String) {
        self.kept += 1;
        if deviation.is_nan() || deviation > tol {
            self.disagrees += 1;
            if self.outliers.len() < Self::OUTLIERS {
                self.outliers.push(describe());
            }
        } else if deviation <= 1e-12 {
            self.le_1e12 += 1;
        } else if deviation <= 1e-9 {
            self.le_1e9 += 1;
        } else if deviation <= 1e-6 {
            self.le_1e6 += 1;
        } else {
            self.le_tol += 1;
        }
        if deviation.is_nan() || deviation > self.worst {
            self.worst = deviation;
        }
    }

    fn report(&self, label: &str) {
        println!(
            "{label}: requested {}, separated (oracle > 0 on at least one pair) {}, kept {}",
            self.requested, self.separated, self.kept
        );
        println!(
            "{label}: deviation buckets: <=1e-12 {}, <=1e-9 {}, <=1e-6 {}, <=tol {}, >tol {}",
            self.le_1e12, self.le_1e9, self.le_1e6, self.le_tol, self.disagrees
        );
        println!("{label}: worst |oracle - port| {:.6e}", self.worst);
        for line in &self.outliers {
            println!("  {label} OUTLIER {line}");
        }
    }

    /// `Ok(())` when this measurement is MET (something was kept and nothing
    /// disagreed), matching `penetration_subset.rs`'s exit convention: a run
    /// that compared nothing is not a pass either.
    fn verdict(&self, label: &str, tol: f64) -> Result<(), String> {
        if self.kept == 0 {
            return Err(format!("{label}: nothing was measured"));
        }
        if self.disagrees != 0 {
            return Err(format!(
                "{label}: NOT MET at tol {tol:.0e}: {} of {} samples disagree",
                self.disagrees, self.kept
            ));
        }
        println!("{label}: MET at tol {tol:.0e} on {} samples", self.kept);
        Ok(())
    }
}

/// Draws a log-uniform probe half-extent and an offset within `target`'s
/// reach, exactly `penetration_subset.rs`'s per-sample draw (see that
/// file's module doc for why the range is `[-2.7, -0.52]` decades).
fn draw_probe(rng: &mut ChaCha8Rng, target: &Target) -> (f64, [f64; 3]) {
    let radius: f64 = 10f64.powf(rng.random_range(-2.7..-0.52));
    let offset: [f64; 3] = std::array::from_fn(|i| {
        let reach = target.half_extent[i] + radius;
        rng.random_range(-reach..reach)
    });
    (radius, offset)
}

fn link_pose(
    model: &RobotModel,
    joint_values: &Value,
    target: &Target,
) -> Result<Isometry3, String> {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    apply_joint_values(&mut state, joint_values)?;
    let fk = state
        .update()
        .global_link_transform(&target.link)
        .map_err(|e| format!("FK for {}: {e}", target.link))?;
    Ok(fk * target.origin)
}

// ---------------------------------------------------------------------
// Measurement 1: two or more simultaneously penetrating pairs.
// ---------------------------------------------------------------------

/// One probe of `multi_pair_port_distance`'s two -- the target link it must
/// stay unmasked against, its shape and its world pose. Bundled so the
/// function takes one of these per probe instead of three loose parameters,
/// which is what made it `too_many_arguments` in the first place.
struct PairProbe<'a> {
    target: &'a str,
    shape: &'a Shape,
    pose: Isometry3,
}

/// This side's answer for a scene with two probes, each unmasked against
/// its own target only -- the port's own multi-pair minimum, exercised with
/// more than one pair visible at once for the first time in this corpus.
fn multi_pair_port_distance(
    model: &RobotModel,
    link_names: &[String],
    joint_values: &Value,
    pair_a: PairProbe,
    pair_b: PairProbe,
) -> Result<f64, String> {
    let mut world = World::new();
    world.add_shape("probe_a", Arc::new(pair_a.shape.clone()), pair_a.pose);
    world.add_shape("probe_b", Arc::new(pair_b.shape.clone()), pair_b.pose);
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

    let mut names: Vec<String> = link_names.to_vec();
    names.push("probe_a".to_owned());
    names.push("probe_b".to_owned());
    let mut acm = AllowedCollisionMatrix::from_names(&names, true);
    acm.set_entry("probe_a", pair_a.target, false);
    acm.set_entry("probe_b", pair_b.target, false);

    let mut state = RobotState::new(model);
    state.set_to_default_values();
    apply_joint_values(&mut state, joint_values)?;
    let posed = state.update();

    let result = env.distance_robot(
        &DistanceRequest {
            enable_signed_distance: true,
            acm: Some(&acm),
            ..DistanceRequest::default()
        },
        &posed,
        &[],
    );
    Ok(result.minimum_distance.distance)
}

fn measure_multi_pair(
    oracle: &mut Oracle,
    model: &RobotModel,
    link_names: &[String],
    targets: &[Target],
    states: &[Value],
    args: &Args,
) -> Result<Stats, String> {
    let mut stats = Stats::default();
    let n = targets.len();
    if n < 2 {
        println!(
            "multi-pair: {n} single-primitive target(s), need at least 2 -- skipped for this robot"
        );
        return Ok(stats);
    }

    // Adjacent, circular pairing: (targets[0], targets[1]), (targets[1],
    // targets[2]), ..., (targets[n-1], targets[0]) -- n pairs, so every
    // target link is exercised in two of them, at a cost comparable to the
    // single-pair corpus's per-robot request count rather than the
    // O(n^2) full cross product.
    let mut rng = ChaCha8Rng::seed_from_u64((args.seed as u64) ^ 0x1357_9BDF_2468_ACE0);

    for i in 0..n {
        let target_a = &targets[i];
        let target_b = &targets[(i + 1) % n];

        for (index, joint_values) in states.iter().enumerate() {
            let (radius_a, offset_a) = draw_probe(&mut rng, target_a);
            let (radius_b, offset_b) = draw_probe(&mut rng, target_b);

            let base_a = link_pose(model, joint_values, target_a)?;
            let base_b = link_pose(model, joint_values, target_b)?;
            let pose_a = base_a * Isometry3::translation(offset_a[0], offset_a[1], offset_a[2]);
            let pose_b = base_b * Isometry3::translation(offset_b[0], offset_b[1], offset_b[2]);

            stats.requested += 1;

            let scene_objects = json!([
                {"id": "probe_a", "pose": row_major(&pose_a), "shape": {"type": "sphere", "radius": radius_a}},
                {"id": "probe_b", "pose": row_major(&pose_b), "shape": {"type": "sphere", "radius": radius_b}},
            ]);

            let answer_a = oracle.ask(json!({
                "op": "scene_diff_collision",
                "joint_values": joint_values,
                "objects": scene_objects,
                "diff": mask_diff_pairs(link_names, &["probe_a", "probe_b"], &[("probe_a", &target_a.link)]),
            }))?;
            let dist_a = answer_a["child"]["robot_distance"]
                .as_f64()
                .ok_or("child.robot_distance is not a number")?;

            let answer_b = oracle.ask(json!({
                "op": "scene_diff_collision",
                "joint_values": joint_values,
                "objects": scene_objects,
                "diff": mask_diff_pairs(link_names, &["probe_a", "probe_b"], &[("probe_b", &target_b.link)]),
            }))?;
            let dist_b = answer_b["child"]["robot_distance"]
                .as_f64()
                .ok_or("child.robot_distance is not a number")?;

            // Both isolated queries fully masked the other probe away
            // (`mask_diff_pairs` with a `keep` list that excludes it), so an
            // isolated answer at `NO_PAIR` means the ACM diff did not reach
            // the query -- the same failure `prove_mask_applies` catches in
            // `penetration_subset.rs`, checked here on every sample instead
            // of once per target since there is no separate proof step.
            if dist_a >= NO_PAIR || dist_b >= NO_PAIR {
                return Err(format!(
                    "{}/{} state {index}: isolated query answered at NO_PAIR ({dist_a:.3e}, {dist_b:.3e}) -- \
                     the ACM diff did not reach the query",
                    target_a.link, target_b.link
                ));
            }

            // Both pairs must be in the penetration branch for this to be a
            // two-pair penetrating sample; a probe placement that missed its
            // own target (possible only at the extreme edge of the
            // reach-bounded offset draw) is counted and dropped, exactly
            // `penetration_subset.rs`'s `separated`.
            if dist_a > 0.0 || dist_b > 0.0 {
                stats.separated += 1;
                continue;
            }

            let true_min = dist_a.min(dist_b);

            let shape_a =
                Shape::Sphere(Sphere::new(radius_a).map_err(|e| format!("probe a radius: {e}"))?);
            let shape_b =
                Shape::Sphere(Sphere::new(radius_b).map_err(|e| format!("probe b radius: {e}"))?);
            let port_answer = multi_pair_port_distance(
                model,
                link_names,
                joint_values,
                PairProbe {
                    target: &target_a.link,
                    shape: &shape_a,
                    pose: pose_a,
                },
                PairProbe {
                    target: &target_b.link,
                    shape: &shape_b,
                    pose: pose_b,
                },
            )?;

            let deviation = (true_min - port_answer).abs();
            let (link_a, link_b) = (target_a.link.clone(), target_b.link.clone());
            stats.record(deviation, args.tol, move || {
                format!(
                    "{link_a}+{link_b} state {index}: oracle min(a,b) {true_min:.17e} vs port \
                     {port_answer:.17e} (|d|={deviation:.3e})"
                )
            });
        }
    }

    Ok(stats)
}

// ---------------------------------------------------------------------
// Measurement 2: `box x box`.
// ---------------------------------------------------------------------

/// This side's answer for one `box`-target link against a `box`-shaped
/// probe, through the same single-pair ACM mask `penetration_subset.rs`'s
/// `port_distance` uses -- only the probe's shape differs.
fn box_port_distance(
    model: &RobotModel,
    link_names: &[String],
    joint_values: &Value,
    target: &str,
    probe: &Shape,
    probe_pose: Isometry3,
) -> Result<f64, String> {
    let mut world = World::new();
    world.add_shape("probe", Arc::new(probe.clone()), probe_pose);
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

    let mut names: Vec<String> = link_names.to_vec();
    names.push("probe".to_owned());
    let mut acm = AllowedCollisionMatrix::from_names(&names, true);
    acm.set_entry("probe", target, false);

    let mut state = RobotState::new(model);
    state.set_to_default_values();
    apply_joint_values(&mut state, joint_values)?;
    let posed = state.update();

    let result = env.distance_robot(
        &DistanceRequest {
            enable_signed_distance: true,
            acm: Some(&acm),
            ..DistanceRequest::default()
        },
        &posed,
        &[],
    );
    Ok(result.minimum_distance.distance)
}

fn measure_box_box(
    oracle: &mut Oracle,
    model: &RobotModel,
    link_names: &[String],
    targets: &[Target],
    states: &[Value],
    args: &Args,
) -> Result<Stats, String> {
    let mut stats = Stats::default();
    let box_targets: Vec<&Target> = targets.iter().filter(|t| t.kind == "box").collect();
    if box_targets.is_empty() {
        println!("box x box: no `box` target on this robot -- skipped");
        return Ok(stats);
    }

    let mut rng = ChaCha8Rng::seed_from_u64((args.seed as u64) ^ 0xC3D2_E1F0_A5B4_9687);

    for target in &box_targets {
        for (index, joint_values) in states.iter().enumerate() {
            let (radius, offset) = draw_probe(&mut rng, target);
            let base = link_pose(model, joint_values, target)?;
            let probe_pose = base * Isometry3::translation(offset[0], offset[1], offset[2]);

            stats.requested += 1;
            let size = [2.0 * radius, 2.0 * radius, 2.0 * radius];
            let fcl_answer = oracle.ask(json!({
                "op": "pair_signed_distance",
                "joint_values": joint_values,
                "link": target.link,
                "objects": [{
                    "id": "probe",
                    "pose": row_major(&probe_pose),
                    "shape": {"type": "box", "size": size},
                }],
            }))?;
            let expected = fcl_answer["fcl_signed_distance"]
                .as_f64()
                .ok_or("fcl_signed_distance is not a number")?;

            // `> 0` and not `>= 0`, matching `penetration_subset.rs`'s own
            // branch test (`collision_common.cpp:636` is `distance <= 0`).
            if expected > 0.0 {
                stats.separated += 1;
                continue;
            }

            let probe = Shape::Cuboid(
                Cuboid::new(size[0], size[1], size[2])
                    .map_err(|e| format!("probe box {size:?}: {e}"))?,
            );
            let actual = box_port_distance(
                model,
                link_names,
                joint_values,
                &target.link,
                &probe,
                probe_pose,
            )?;
            let deviation = (expected - actual).abs();

            let link = target.link.clone();
            stats.record(deviation, args.tol, move || {
                format!(
                    "{link} state {index} half-extent={radius:.6e}: fcl {expected:.17e} vs port \
                     {actual:.17e} (|d|={deviation:.3e})"
                )
            });
        }
    }

    Ok(stats)
}

fn run() -> Result<i32, String> {
    let args = parse_args()?;
    let model = build_model(&args)?;
    let mut oracle = Oracle::spawn(&args)?;

    let info = oracle.ask(json!({"op": "model_info"}))?;
    let link_names: Vec<String> = info["links"]
        .as_array()
        .ok_or("model_info.links is not an array")?
        .iter()
        .map(|v| v.as_str().unwrap_or_default().to_owned())
        .collect();

    let targets = targets(&model);
    println!(
        "targets: {} link(s) with exactly one primitive collision shape ({} box)",
        targets.len(),
        targets.iter().filter(|t| t.kind == "box").count()
    );

    let states = oracle.ask(json!({
        "op": "random_states", "count": args.states, "seed": args.seed
    }))?;
    let states = states["states"]
        .as_array()
        .ok_or("random_states.states is not an array")?
        .clone();

    println!();
    println!("=== multi-pair (>= 2 simultaneously penetrating pairs) ===");
    let multi = measure_multi_pair(&mut oracle, &model, &link_names, &targets, &states, &args)?;
    multi.report("multi-pair");

    println!();
    println!("=== box x box ===");
    let boxbox = measure_box_box(&mut oracle, &model, &link_names, &targets, &states, &args)?;
    boxbox.report("box x box");

    println!();
    let mut failed = false;
    for (label, stats) in [("multi-pair", &multi), ("box x box", &boxbox)] {
        if stats.kept == 0 {
            println!("{label}: SKIPPED (nothing to measure on this robot)");
            continue;
        }
        match stats.verdict(label, args.tol) {
            Ok(()) => {}
            Err(message) => {
                println!("{message}");
                failed = true;
            }
        }
    }

    Ok(if failed { 1 } else { 0 })
}

fn main() {
    match run() {
        Ok(code) => std::process::exit(code),
        Err(message) => {
            eprintln!("penetration-extended: {message}");
            std::process::exit(2);
        }
    }
}
