// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Measures `PORTING-PLAN.md` §5 Phase 3's `distance: f64` clause on the
//! *penetration* branch -- the branch the sweep prints but does not score,
//! because all three of this repo's filed `distanceCallback` defects live
//! there.
//!
//! The sweep cannot score that branch against the oracle for a reason that is
//! about the *reference*, not about the branch: on an arbitrary state the
//! oracle's published penetration value may have come out of a defective path.
//! It does not follow that no penetrating query is comparable. This binary
//! builds the sub-population on which none of the three can fire, derived from
//! upstream's own source rather than from sweep output, and compares the two
//! sides there at the clause's own `1e-4`.
//!
//! # The three exclusions, and what each one costs
//!
//! Line numbers are `moveit_core/collision_detection_fcl/src/collision_common.cpp`
//! at moveit2 `e017c91ee12984393a28ba246075c65f69cde3bf`.
//!
//! Every fcl line number below is read at **tag `0.7.0`**
//! (`df2702ca5e703dec98ebd725782ce13862e87fc8`), not at the local checkout's
//! `e5efcc4` HEAD, per PORTING-PLAN.md §135: the oracle image links
//! `libfcl-dev 0.7.0-3build2`, which that section establishes is pure upstream
//! 0.7.0. The distinction is not bookkeeping here. Eight of the ten headers
//! cited below are byte-identical between the tag and the checkout, but
//! `box_box-inl.h` is one line off and `gjk_libccd-inl.h` is 95 -- its
//! `-CCD_ONE` return is `:2256` at the tag and `:2351` at HEAD. Both were
//! checked against the image's own `/usr/include/fcl` with `cmp`, not assumed.
//!
//! fcl headers are cited by basename, the spelling
//! `tools/ci/upstream-citation-exemptions.json` declares them under; their
//! directories, all under `include/fcl`, are:
//! `narrowphase/distance_request-inl.h`,
//! `narrowphase/collision_request.h`,
//! `narrowphase/detail/distance_func_matrix-inl.h`,
//! `narrowphase/detail/gjk_solver_libccd-inl.h`,
//! `narrowphase/detail/convexity_based_algorithm/gjk_libccd-inl.h`,
//! `narrowphase/detail/primitive_shape_algorithm/{sphere_sphere,sphere_box,sphere_cylinder,box_box}-inl.h`
//! and `narrowphase/detail/traversal/collision/shape_collision_traversal_node-inl.h`.
//!
//! **`distance-callback-threshold-suppresses-deeper-pairs`.** For a GLOBAL
//! query `:574` seeds `dist_threshold` from the running
//! `res->minimum_distance.distance` and `:594` copies it into
//! `fcl_result.min_distance`. `DistanceRequest::isSatisfied` is literally
//! `return (result.min_distance <= 0);`
//! (`distance_request-inl.h:76`), and every distance entry point
//! opens with `if(request.isSatisfied(result)) return result.min_distance;`
//! -- ten occurrences in `distance_func_matrix-inl.h`,
//! covering every entry point in the matrix, of which
//! `distance_func_matrix-inl.h:217` is `ShapeShapeDistance`'s. So once the
//! running minimum is non-positive, `fcl::distance`
//! at `:603` returns the threshold *without computing anything* and `:608`'s
//! `if (distance < dist_threshold)` is `x < x`. Every later pair is dropped
//! whole. The defect is shape-pair independent; what bounds it is *ordering*,
//! and it can only change an answer when a pair visited after the minimum
//! first went non-positive would have been deeper. **Exclusion: one pair.**
//! Every query below allows away every robot-link/world-object pair but one
//! through the ACM, so there is no later pair to lose.
//!
//! **`fcl-distance-sentinel-survives-zero-contacts`.** `:636` enters the
//! penetration block on `distance <= 0`, `:647` re-runs the pair through
//! `fcl::collide`, and `:648`'s `if (contacts > 0)` has no `else` -- with zero
//! contacts `dist_result.distance` keeps `fcl_result.min_distance` from
//! `:613`, which for a penetrating pair is libccd's `-1` sentinel
//! (`gjk_libccd-inl.h:2255-2256`, or the
//! closed-form routines' own `*dist = -1`). It fires exactly where the
//! distance routine and the intersect routine disagree about whether the pair
//! touches. **Exclusion: shape pairs whose two routines share one predicate.**
//!
//! **`distance-callback-max-contact-depth`.** `:646` asks for up to 200
//! contacts and `:650-659` keeps the one with the **largest**
//! `penetration_depth`, which for a contact set spanning a large flat body is
//! a lateral escape distance rather than the pair's depth. It needs a set with
//! at least two members to pick the wrong one from. **Exclusion: shape pairs
//! whose intersect routine emits exactly one contact.**
//!
//! # The pairs that satisfy both shape-pair exclusions
//!
//! `sphere x sphere`, `sphere x box` and `sphere x cylinder`, in either order.
//! Upstream builds `fcl::Sphered`/`fcl::Boxd`/`fcl::Cylinderd` directly for
//! URDF primitives (`:875-893`), fcl's default
//! `gjk_solver_type` is `GST_LIBCCD` (`collision_request.h:102`),
//! and libccd registers a closed-form intersect routine for each of these
//! three -- `gjk_solver_libccd-inl.h:245` (sphere/sphere),
//! `gjk_solver_libccd-inl.h:250` (sphere/box) and
//! `gjk_solver_libccd-inl.h:252` (sphere/cylinder), each macro expanding to
//! both argument orders -- so neither the generic GJK/MPR path nor any BVH
//! path is reached.
//!
//! Each routine has exactly one `contacts->emplace_back` and it is not in a
//! loop -- `sphere_sphere-inl.h:82`,
//! `sphere_box-inl.h:175`, `sphere_cylinder-inl.h:218` -- and
//! `ShapeCollisionTraversalNode::leafTesting` adds the solver's contacts
//! verbatim (`shape_collision_traversal_node-inl.h:99-100`).
//! So `fcl::collide` returns exactly 1, `:650-659` has one candidate, and
//! `dist_result.distance` is `-depth` for the single closed-form contact.
//!
//! And the two routines of each pair test the *same* predicate:
//! `sphereBoxIntersect` returns false iff `squared_distance > r * r`
//! (`sphere_box-inl.h:119-120`), and `sphereBoxDistance` writes `*dist = -1`
//! on the complement of that same test (`sphere_box-inl.h:205`,
//! `sphere_box-inl.h:224`); likewise `sphere_cylinder-inl.h:136-137` against
//! `sphere_cylinder-inl.h:250` and `sphere_cylinder-inl.h:269`, and
//! `sphere_sphere-inl.h:72-73` against `sphere_sphere-inl.h:99` and
//! `sphere_sphere-inl.h:107`. A pair that reaches `:636` therefore always
//! produces a contact, and the sentinel can never survive.
//!
//! `box x box` is deliberately excluded: `boxBoxIntersect` copies out a
//! `boxBox2(..., 4, contacts)` set (`box_box-inl.h:857-874`,
//! the call at `box_box-inl.h:865-868`) and can emit several contacts. Mesh
//! links are excluded for the same reason and more so.
//!
//! # What the corpus is
//!
//! One world object -- a sphere, so every pair below is one of the three --
//! placed on a link whose collision geometry is exactly one primitive. Links
//! carrying several collision elements are skipped: the ACM keys on the link
//! *name*, so unmasking such a link would expose all of its shapes as separate
//! pairs and re-open the ordering defect.
//!
//! The probe's radius and its offset from the link shape's own frame are drawn
//! from a seeded `ChaCha8Rng`, and the pose that goes on the wire is absolute,
//! so both sides are handed identical geometry whatever either one thinks the
//! link's collision origin is. Samples whose oracle value comes back positive
//! are not in the penetration branch at all; they are counted and dropped, and
//! that count is printed -- it is the branch's own definition (`:636` tests
//! `distance <= 0`), not a filter on the result.
//!
//! # Usage
//!
//! ```text
//! penetration-subset --urdf F.urdf --srdf F.srdf \
//!     --oracle tools/moveit-oracle/run-oracle.sh [--states N] [--seed S] [--tol T]
//! ```
//!
//! Exits non-zero when any kept sample misses `--tol`, or when the robot
//! offers no target link at all -- a run that compared nothing must not read
//! as a pass.

use std::cmp::Ordering;
use std::io::{BufRead, BufReader, Write};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::sync::Arc;

use moveit_collision::{
    AllowedCollisionMatrix, CollisionEnv, DistanceRequest, LinkPaddingScale, ParryCollisionEnv,
    World,
};
use moveit_geometry::{Isometry3, Shape, Sphere};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;
use serde_json::{Value, json};

/// The world object's id. Not a link name in any committed fixture, which is
/// what lets the ACM address it.
const PROBE_ID: &str = "probe";

/// Command line, parsed by hand the way `moveit-diff`'s own `Config` is.
struct Args {
    urdf: String,
    srdf: String,
    oracle: Vec<String>,
    /// Probe placements drawn per target link.
    states: usize,
    /// Seed for the oracle's `random_states` *and* (xor-folded) for this
    /// side's probe draws, so one number replays the whole corpus.
    seed: i64,
    /// The clause's own tolerance. Not defaulted to anything else: this
    /// binary exists to answer §5 Phase 3's `distance: f64` row.
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

/// The oracle subprocess, as a JSON-lines filter.
struct Oracle {
    child: Child,
    /// `None` once closed; closing it is the shutdown signal, so `Drop` must
    /// close before waiting or the wait deadlocks against a blocked read.
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
/// element is a primitive whose pair against a sphere reaches a closed-form
/// libccd routine.
struct Target {
    link: String,
    /// The shape's pose in the link's own frame.
    origin: Isometry3,
    /// Half-extents in the shape's own frame, used only to size the region
    /// the probe centre is drawn from.
    half_extent: [f64; 3],
    /// `"sphere"`, `"box"` or `"cylinder"` -- the other half of the pair name
    /// in the report, the probe always being a sphere.
    kind: &'static str,
}

/// Every link of `model` that qualifies, in model order.
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

/// `Isometry3` as the oracle's row-major 4x4 wire form.
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

/// Per-shape-pair tallies, so the verdict names the pairs it covered rather
/// than a single count that could be one pair repeated.
#[derive(Default)]
struct Tally {
    kept: usize,
    worst: f64,
    /// Every kept sample's oracle value. A subset measurement that only ever
    /// saw depths near zero would agree for a reason that has nothing to do
    /// with the penetration branch, so the depths it covered are reported
    /// beside the agreement rather than left to be assumed.
    depths: Vec<f64>,
}

impl Tally {
    /// `(shallowest, median, deepest)` of `depths` as magnitudes.
    fn depth_span(&self) -> (f64, f64, f64) {
        let mut d: Vec<f64> = self.depths.iter().map(|v| v.abs()).collect();
        d.sort_by(f64::total_cmp);
        match d.len() {
            0 => (f64::NAN, f64::NAN, f64::NAN),
            n => (d[0], d[n / 2], d[n - 1]),
        }
    }
}

/// The measurement, accumulated across every (target, placement).
#[derive(Default)]
struct Stats {
    requested: usize,
    /// Oracle value came back `> 0`: not the penetration branch, so not this
    /// row's population.
    separated: usize,
    kept: usize,
    disagrees: usize,
    worst: f64,
    /// Kept samples whose masked answer differs from the same response's
    /// `parent_before` -- the identical query with the ACM diff *not* applied.
    ///
    /// Reported, not asserted on: with the probe placed on its own target the
    /// target pair is usually the unmasked minimum too, so this is zero on a
    /// healthy corpus and cannot serve as the mask's proof. What proves the
    /// mask is [`prove_mask_applies`]. This number says something else, and it
    /// is worth its line: every sample it counts is one where the unmasked
    /// GLOBAL query -- the shape every sweep state has -- published a
    /// different number for geometry that did not move.
    mask_changed: usize,
    /// Largest `|masked - unmasked|` over the kept samples.
    mask_effect: f64,
    /// `|oracle - port|` partitioned by first matching bound; the columns sum
    /// to `kept`.
    le_1e12: usize,
    le_1e9: usize,
    le_1e6: usize,
    le_tol: usize,
    /// Worst-first, capped, so a failure is diagnosable without a re-run.
    outliers: Vec<String>,
}

impl Stats {
    /// How many outliers are kept for the report.
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

/// This side's answer for one placement: `distance_robot` through an ACM that
/// allows every pair away except `target` against [`PROBE_ID`].
fn port_distance(
    model: &RobotModel,
    link_names: &[String],
    joint_values: &Value,
    target: &str,
    probe: &Shape,
    probe_pose: Isometry3,
) -> Result<f64, String> {
    let mut world = World::new();
    world.add_shape(PROBE_ID, Arc::new(probe.clone()), probe_pose);
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

    let mut names: Vec<String> = link_names.to_vec();
    names.push(PROBE_ID.to_owned());
    let mut acm = AllowedCollisionMatrix::from_names(&names, true);
    acm.set_entry(PROBE_ID, target, false);

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

/// Poses `state` at one `random_states` entry. Every variable the oracle drew
/// is set, so the two sides start from the same configuration and not from
/// this side's defaults for anything the draw happened to omit.
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

/// Upstream leaves `DistanceResultsData::distance` at `DBL_MAX` when no pair
/// is considered, so a masked-out query answers above any real geometry. The
/// threshold only has to separate that from a distance, not to approximate it.
const NO_PAIR: f64 = 1e30;

/// Proves the ACM diff reaches the oracle's query, once per target link.
///
/// The corpus's whole claim to being ordering-defect-free is that exactly one
/// pair is visited, and that rests entirely on `set_acm_entry` being applied.
/// Comparing masked against unmasked answers cannot show it -- with the probe
/// sitting on its own target, the target pair is the unmasked minimum too, so
/// a diff that silently stopped applying would leave every measured number
/// identical and the run would still print `MET`.
///
/// This does show it, from one response per target: the probe is placed at the
/// link shape's own origin (an overlap by construction, whatever the shape),
/// every link *including* the target is allowed away, and the two summaries in
/// the same answer must disagree in the one way only an applied diff can
/// produce -- `parent_before` a real penetrating value because the pair is
/// there, `child` at [`NO_PAIR`] because the diff took it away.
fn prove_mask_applies(
    oracle: &mut Oracle,
    model: &RobotModel,
    link_names: &[String],
    joint_values: &Value,
    target: &Target,
) -> Result<(), String> {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    apply_joint_values(&mut state, joint_values)?;
    let probe_pose = state
        .update()
        .global_link_transform(&target.link)
        .map_err(|e| format!("FK for {}: {e}", target.link))?
        * target.origin;
    let radius = target
        .half_extent
        .iter()
        .copied()
        .fold(f64::INFINITY, f64::min);

    // `""` is no link's name, so nothing is filtered out and the mask is
    // total. Asserted rather than assumed: a filter that dropped the target
    // would make the check below pass for the wrong reason.
    let all_masked = mask_diff(link_names, "");
    let covered = all_masked.as_array().map_or(0, Vec::len);
    if covered != link_names.len() {
        return Err(format!(
            "full mask covers {covered} of {} links",
            link_names.len()
        ));
    }

    let answer = oracle.ask(json!({
        "op": "scene_diff_collision",
        "joint_values": joint_values,
        "objects": [{
            "id": PROBE_ID,
            "pose": row_major(&probe_pose),
            "shape": {"type": "sphere", "radius": radius},
        }],
        "diff": all_masked,
    }))?;
    let unmasked = answer["parent_before"]["robot_distance"]
        .as_f64()
        .ok_or("parent_before.robot_distance is not a number")?;
    let masked = answer["child"]["robot_distance"]
        .as_f64()
        .ok_or("child.robot_distance is not a number")?;

    if unmasked > 0.0 {
        return Err(format!(
            "{}: a probe at the link shape's own origin did not penetrate it \
             (unmasked robot_distance {unmasked:.17e}) -- the mask proof needs a pair to remove",
            target.link
        ));
    }
    if masked < NO_PAIR {
        return Err(format!(
            "{}: allowing every link away still answered {masked:.17e} -- the ACM diff did not \
             reach the query, so no run here may claim a single-pair corpus",
            target.link
        ));
    }
    Ok(())
}

/// The `diff` array that leaves exactly `target` visible to the probe.
fn mask_diff(link_names: &[String], target: &str) -> Value {
    let actions: Vec<Value> = link_names
        .iter()
        .filter(|name| name.as_str() != target)
        .map(|name| {
            json!({"action": "set_acm_entry", "first": PROBE_ID, "second": name, "allowed": true})
        })
        .collect();
    json!(actions)
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
    if targets.is_empty() {
        return Err(format!(
            "{}: no link carries exactly one primitive collision shape -- \
             this robot offers no defect-free penetrating pair, and a run that \
             compared nothing is not a pass",
            args.urdf
        ));
    }

    let states = oracle.ask(json!({
        "op": "random_states", "count": args.states, "seed": args.seed
    }))?;
    let states = states["states"]
        .as_array()
        .ok_or("random_states.states is not an array")?
        .clone();

    // One stream for the whole corpus, folded off `--seed` the way
    // `moveit-diff` folds its own, so `(--states, --seed)` replays it.
    let mut rng = ChaCha8Rng::seed_from_u64((args.seed as u64) ^ 0x9E37_79B9_7F4A_7C15);

    println!(
        "targets: {} link(s) with exactly one primitive collision shape",
        targets.len()
    );
    for t in &targets {
        println!(
            "  {:<40} sphere x {:<9} half-extent [{:.4}, {:.4}, {:.4}]",
            t.link, t.kind, t.half_extent[0], t.half_extent[1], t.half_extent[2]
        );
    }

    let mut stats = Stats::default();
    let mut per_pair: std::collections::BTreeMap<&'static str, Tally> =
        std::collections::BTreeMap::new();

    let first_state = states.first().ok_or("random_states returned no state")?;
    for target in &targets {
        prove_mask_applies(&mut oracle, &model, &link_names, first_state, target)?;
    }
    println!(
        "ACM diff proven to reach the oracle's query on all {} target(s)",
        targets.len()
    );

    for target in &targets {
        for (index, joint_values) in states.iter().enumerate() {
            // Log-uniform over 2.18 decades, and it has to reach the top of
            // that range: the corpus is only as strong as the deepest
            // penetration in it. A relative error of `e` planted in this
            // port's signed distance deviates by `e * |depth|`, so a corpus
            // whose deepest sample is `d` cannot separate any error below
            // `tol / d` from agreement. Measured -- with the radius capped at
            // `0.05` the deepest sample was `6.8e-2`, and scaling
            // `parry.rs`'s `accumulate_distance` by `1.001` (0.1%) moved the
            // worst deviation only to `6.8e-5`, inside the clause's `1e-4`,
            // so the run still printed MET. At this range the same mutation
            // is caught, and the two numbers obey that identity exactly: see
            // the round section cited by §5 Phase 3's row.
            let radius: f64 = 10f64.powf(rng.random_range(-2.7..-0.52));
            let offset: [f64; 3] = std::array::from_fn(|i| {
                let reach = target.half_extent[i] + radius;
                rng.random_range(-reach..reach)
            });

            let mut state = RobotState::new(&model);
            state.set_to_default_values();
            apply_joint_values(&mut state, joint_values)?;
            let link_pose = state
                .update()
                .global_link_transform(&target.link)
                .map_err(|e| format!("FK for {}: {e}", target.link))?;
            let probe_pose =
                link_pose * target.origin * Isometry3::translation(offset[0], offset[1], offset[2]);

            stats.requested += 1;
            let answer = oracle.ask(json!({
                "op": "scene_diff_collision",
                "joint_values": joint_values,
                "objects": [{
                    "id": PROBE_ID,
                    "pose": row_major(&probe_pose),
                    "shape": {"type": "sphere", "radius": radius},
                }],
                "diff": mask_diff(&link_names, &target.link),
            }))?;
            let expected = answer["child"]["robot_distance"]
                .as_f64()
                .ok_or("child.robot_distance is not a number")?;
            // The same response's unmasked answer, free: `parent_before` is
            // the identical scene with the ACM diff not yet applied.
            let unmasked = answer["parent_before"]["robot_distance"]
                .as_f64()
                .ok_or("parent_before.robot_distance is not a number")?;

            // `> 0` and not `>= 0`, because `:636` tests `distance <= 0`: a
            // published exact zero is the penetration branch's own answer.
            if expected > 0.0 {
                stats.separated += 1;
                continue;
            }

            let probe = Shape::Sphere(
                Sphere::new(radius).map_err(|e| format!("probe radius {radius}: {e}"))?,
            );
            let actual = port_distance(
                &model,
                &link_names,
                joint_values,
                &target.link,
                &probe,
                probe_pose,
            )?;
            let deviation = (expected - actual).abs();

            let mask_effect = (expected - unmasked).abs();
            if mask_effect > 0.0 {
                stats.mask_changed += 1;
                if mask_effect > stats.mask_effect {
                    stats.mask_effect = mask_effect;
                }
            }

            let entry = per_pair.entry(target.kind).or_default();
            entry.kept += 1;
            entry.depths.push(expected);
            if deviation.is_nan() || deviation > entry.worst {
                entry.worst = deviation;
            }
            let link = target.link.clone();
            stats.record(deviation, args.tol, move || {
                format!(
                    "{link} state {index} r={radius:.6e}: oracle {expected:.17e} vs port \
                     {actual:.17e} (|d|={deviation:.3e})"
                )
            });
        }
    }

    println!();
    println!(
        "requests {}, separated (oracle > 0, not this branch) {}, kept {}",
        stats.requested, stats.separated, stats.kept
    );
    for (kind, tally) in &per_pair {
        let (shallow, median, deep) = tally.depth_span();
        println!(
            "  sphere x {:<9} kept {:>6}  worst |oracle - port| {:.6e}  \
             depth |oracle| min {shallow:.3e} median {median:.3e} max {deep:.3e}",
            kind, tally.kept, tally.worst
        );
    }
    println!(
        "deviation buckets: <=1e-12 {}, <=1e-9 {}, <=1e-6 {}, <=tol {}, >tol {}",
        stats.le_1e12, stats.le_1e9, stats.le_1e6, stats.le_tol, stats.disagrees
    );
    println!("worst |oracle - port|: {:.6e}", stats.worst);
    for line in &stats.outliers {
        println!("  OUTLIER {line}");
    }

    println!(
        "ACM mask changed the oracle's answer on {} of {} kept samples, by up to {:.6e}",
        stats.mask_changed, stats.kept, stats.mask_effect
    );

    if stats.kept == 0 {
        return Err("no placement landed in the penetration branch -- nothing was measured".into());
    }
    if stats.disagrees == 0 {
        println!(
            "MET at tol {:.0e} on the defect-free penetration subset, {} samples",
            args.tol, stats.kept
        );
        Ok(0)
    } else {
        println!(
            "NOT MET at tol {:.0e}: {} of {} samples disagree",
            args.tol, stats.disagrees, stats.kept
        );
        Ok(1)
    }
}

fn main() {
    match run() {
        Ok(code) => std::process::exit(code),
        Err(message) => {
            eprintln!("penetration-subset: {message}");
            std::process::exit(2);
        }
    }
}
