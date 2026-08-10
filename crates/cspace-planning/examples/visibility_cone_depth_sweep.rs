// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No upstream file: PORTING-PLAN.md §148's decisive experiment for the
// residual pr2 `visibility_cone` `touching >= 2` cases -- diagnostic
// infrastructure for this crate's own `VisibilityConstraint`, not a port.

//! §148's decisive test (see `VisibilityConstraint::decide`'s own doc
//! comment for the full round-15/16/121.2 history this continues): for
//! every case where multiple robot links touch the visibility cone at once,
//! pull every candidate contact's depth
//! (`VisibilityConstraint::cone_contact_depths`) and compare each to the
//! oracle's own reported depth for that case. A match (within a tolerance
//! measured from this same run's own `touching == 1` population, where no
//! traversal-order ambiguity is even possible) means traversal order --
//! picking a different "first" contact than upstream's FCL would have -- is
//! a live explanation for that case; no match across every candidate
//! excludes it, leaving `cspace-collision`'s deviation 6 as the sole
//! explanation.
//!
//! # Why this drives the oracle itself, unlike this crate's other tests
//!
//! `tools/moveit-diff`'s own `compare_constraints` deliberately stopped
//! comparing `visibility_cone` distances at all (see that function's own
//! doc comment) -- a reasoned decision for its job (an unqualified
//! pass/fail parity gate), but it means the tool that already drives the
//! oracle over `Op::Constraints` never surfaces the raw depths this test
//! needs, and nothing under `tools/` is this crate's to change this round.
//! This binary is a second, independent oracle client, built from
//! `tools/moveit-diff/src/protocol.rs`'s published wire shapes (that file's
//! own header: "Keep this file and `oracle.cpp` in step" -- reproduced here
//! rather than imported, since `moveit-diff` is a `[[bin]]`-only crate with
//! no library target this crate could depend on).
//!
//! # Usage
//!
//! ```text
//! sg docker -c 'cargo run --release --example visibility_cone_depth_sweep \
//!   -p cspace-constraints -- \
//!   --urdf <abs>/fixtures/pr2.urdf --srdf <abs>/fixtures/pr2.srdf \
//!   --seed <N> --cases <N>'
//! ```
//!
//! Absolute paths only -- relative paths fail inside the oracle container.
//! `--cases` is the number of *visibility_cone*-style cases this binary
//! generates itself (its own case-6-only re-derivation of
//! `tools/moveit-diff`'s `build_constraint_case`, since that function is
//! private to a bin-only crate this one cannot import): half placed at a
//! real pr2 link's collision-shape center (may touch), half 50m away (never
//! touches), alternating by parity -- not `--constraints`-many mixed-kind
//! cases like `moveit-diff` generates, since this binary needs no coverage
//! of the other six constraint kinds.
//!
//! Prints one line per `touching >= 1` case, then a summary: touching==1/
//! touching>=2 counts, the measured tolerance, and the traversal-order
//! match verdict with exact counts.
//!
//! `--geometry-gaps` switches to a second, independent diagnostic instead
//! of running the sweep: for every pair of pr2's parry-representable
//! links, print `center_dist - far_link_extent` (the cone reach a
//! near-placed case anchored at the *other* link of the pair would need,
//! at minimum, to touch it) sorted ascending, then the generator's own
//! maximum possible cone reach (`target_radius`'s upper bound, since the
//! cone's farthest point from its own base center is at most
//! `max(target_radius, sensor_offset)` -- see this file's own
//! `max_cone_reach` doc). This is the evidence backing
//! `PORTING-PLAN.md`'s §148 closure: it is what makes "0 touching >= 2
//! cases" a provable current fact rather than a sampling accident.

use std::collections::{BTreeMap, HashMap};
use std::io::{BufRead, BufReader, Write};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};

use cspace_core::geometry::{Isometry3, Shape, Transforms, UnitQuaternion, Vector3};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;
use cspace_planning::constraints::{
    SensorSpec, SensorViewDirection, TargetSpec, VisibilityConstraint, VisibilityCriteria,
};
use nalgebra::{Matrix3, Rotation3, Translation3};
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};

// --- Wire protocol, mirrored from tools/moveit-diff/src/protocol.rs (see
// this file's own module doc for why it is reproduced rather than
// imported). Only the ops/results this binary actually uses. ---

#[derive(Serialize)]
struct Request {
    id: u64,
    #[serde(flatten)]
    op: Op,
}

#[derive(Serialize)]
#[serde(tag = "op", rename_all = "snake_case")]
enum Op {
    ModelInfo,
    RandomStates {
        count: usize,
        seed: i32,
    },
    Fk {
        joint_values: BTreeMap<String, f64>,
        links: Vec<String>,
    },
    Constraints {
        joint_values: BTreeMap<String, f64>,
        constraints: ConstraintsSpecWire,
    },
}

#[derive(Serialize, Default)]
struct ConstraintsSpecWire {
    joint_constraints: Vec<serde_json::Value>,
    position_constraints: Vec<serde_json::Value>,
    orientation_constraints: Vec<serde_json::Value>,
    visibility_constraints: Vec<VisibilityConstraintSpecWire>,
}

#[derive(Serialize)]
struct VisibilityConstraintSpecWire {
    sensor_frame_id: String,
    sensor_pose: [f64; 16],
    sensor_view_direction: String,
    target_frame_id: String,
    target_pose: [f64; 16],
    cone_sides: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    target_radius: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    max_view_angle: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    max_range_angle: Option<f64>,
    weight: f64,
}

#[derive(Deserialize)]
struct Response {
    id: u64,
    ok: bool,
    #[serde(default)]
    result: Option<OracleResult>,
    #[serde(default)]
    error: Option<String>,
}

#[derive(Deserialize)]
#[serde(untagged)]
enum OracleResult {
    ModelInfo(ModelInfoWire),
    RandomStates(RandomStatesResultWire),
    Fk(FkResultWire),
    Constraints(ConstraintsResultWire),
}

#[derive(Deserialize)]
struct ModelInfoWire {
    model_frame: String,
}

#[derive(Deserialize)]
struct RandomStatesResultWire {
    states: Vec<BTreeMap<String, f64>>,
}

#[derive(Deserialize)]
struct FkResultWire {
    link_transforms: BTreeMap<String, [f64; 16]>,
}

#[derive(Deserialize)]
struct ConstraintResultWire {
    satisfied: bool,
    distance: f64,
}

#[derive(Deserialize)]
struct ConstraintsResultWire {
    results: Vec<ConstraintResultWire>,
}

// --- Oracle client (same request/response-line protocol as
// tools/moveit-diff/src/main.rs's own `Oracle`). ---

struct Oracle {
    child: Child,
    stdin: Option<ChildStdin>,
    stdout: BufReader<ChildStdout>,
    next_id: u64,
}

impl Oracle {
    fn spawn(oracle_cmd: &[String], urdf: &str, srdf: &str) -> Result<Self, String> {
        let (program, rest) = oracle_cmd.split_first().ok_or("empty oracle command")?;
        let mut child = Command::new(program)
            .args(rest)
            .arg("--urdf")
            .arg(urdf)
            .arg("--srdf")
            .arg(srdf)
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
            (false, _, Some(e)) => Err(format!("oracle error for request {id}: {e}")),
            _ => Err(format!("malformed response for request {id}: {buf:?}")),
        }
    }

    fn close(&mut self) {
        self.stdin.take();
    }
}

impl Drop for Oracle {
    fn drop(&mut self) {
        self.close();
        let _ = self.child.wait();
    }
}

fn isometry_from_row_major(m: &[f64; 16]) -> Isometry3 {
    let rotation = Rotation3::from_matrix_unchecked(Matrix3::new(
        m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10],
    ));
    let translation = Translation3::new(m[3], m[7], m[11]);
    Isometry3::from_parts(translation, UnitQuaternion::from_rotation_matrix(&rotation))
}

fn to_row_major_4x4(iso: &Isometry3) -> [f64; 16] {
    let m = iso.to_homogeneous();
    let mut out = [0.0; 16];
    for r in 0..4 {
        for c in 0..4 {
            out[r * 4 + c] = m[(r, c)];
        }
    }
    out
}

fn pose_row_major(translation: Vector3, rotation: UnitQuaternion) -> [f64; 16] {
    to_row_major_4x4(&Isometry3::from_parts(translation.into(), rotation))
}

fn is_parry_representable(shape: &Shape) -> bool {
    !matches!(shape, Shape::Mesh(_) | Shape::OcTree(_))
}

/// Same selection `tools/moveit-diff`'s `parry_representable_link_names`
/// makes: every link whose collision geometry the parry backend actually
/// represents -- see this binary's own module doc for why a real `pr2`
/// link is what makes a `touching >= 1` case possible at all.
fn parry_representable_link_names(model: &RobotModel) -> Vec<String> {
    model
        .link_models()
        .iter()
        .filter(|link| {
            link.shapes()
                .iter()
                .any(|s| is_parry_representable(&s.shape))
        })
        .map(|link| link.name().to_owned())
        .collect()
}

/// One visibility_cone-style case, re-deriving `tools/moveit-diff`'s
/// `build_constraint_case` case-6 branch (see that function's own doc
/// comment for why the near/far split exists and why both radius/offset
/// magnitudes are what they are). `idx` alternates near (even) / far (odd)
/// so the sweep covers both branches of `decide_cone` against the oracle,
/// same as upstream's own `(case / 7) % 2 == 0` alternation.
fn build_case(
    rng: &mut ChaCha8Rng,
    idx: usize,
    model: &RobotModel,
    model_frame: &str,
    eligible: &[String],
    fk: &BTreeMap<String, [f64; 16]>,
) -> VisibilityConstraintSpecWire {
    let hit_link =
        (!eligible.is_empty() && idx % 2 == 0).then(|| eligible[idx % eligible.len()].as_str());

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
            let link_fk = isometry_from_row_major(
                fk.get(link_name)
                    .expect("link_name came from model.link_models()"),
            );
            let center = (link_fk * shape.origin_transform).translation.vector;
            (center, rng.random_range(0.005..0.015), 0.005)
        }
        None => {
            const FAR_OFFSET: f64 = 50.0;
            let links = model.link_names();
            let link_name = &links[idx % links.len()];
            let point = fk
                .get(link_name)
                .expect("link_name came from model.link_names()");
            let mut anchor = Vector3::new(point[3], point[7], point[11]);
            let axis = idx % 3;
            anchor[axis] += FAR_OFFSET;
            (anchor, rng.random_range(0.05..0.5), 1.0)
        }
    };

    VisibilityConstraintSpecWire {
        sensor_frame_id: model_frame.to_owned(),
        sensor_pose: pose_row_major(
            anchor + Vector3::new(0.0, 0.0, sensor_offset),
            UnitQuaternion::identity(),
        ),
        sensor_view_direction: "sensor_z".to_owned(),
        target_frame_id: model_frame.to_owned(),
        target_pose: pose_row_major(anchor, UnitQuaternion::identity()),
        cone_sides: 3 + idx % 6,
        target_radius: Some(radius),
        max_view_angle: None,
        max_range_angle: None,
        weight: 1.0,
    }
}

struct Args {
    urdf: String,
    srdf: String,
    seed: i32,
    cases: usize,
    oracle: Vec<String>,
    geometry_gaps: bool,
}

fn parse_args() -> Result<Args, String> {
    let mut urdf = None;
    let mut srdf = None;
    let mut seed = 0i32;
    let mut cases = 400usize;
    let mut oracle: Vec<String> = vec!["tools/moveit-oracle/run-oracle.sh".to_owned()];
    let mut geometry_gaps = false;

    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut want = |name: &str| -> Result<String, String> {
            args.next()
                .ok_or_else(|| format!("{name} requires a value"))
        };
        match arg.as_str() {
            "--urdf" => urdf = Some(want("--urdf")?),
            "--srdf" => srdf = Some(want("--srdf")?),
            "--seed" => {
                seed = want("--seed")?
                    .parse()
                    .map_err(|e| format!("--seed: {e}"))?
            }
            "--cases" => {
                cases = want("--cases")?
                    .parse()
                    .map_err(|e| format!("--cases: {e}"))?
            }
            "--geometry-gaps" => geometry_gaps = true,
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

    Ok(Args {
        urdf: urdf.ok_or("--urdf is required")?,
        srdf: srdf.ok_or("--srdf is required")?,
        seed,
        cases,
        oracle,
        geometry_gaps,
    })
}

/// A generator-produced cone's own maximum possible distance from its
/// base center (the near-branch's `anchor`, i.e. the hit link's shape
/// center -- see `build_case`'s `Some(link_name)` arm): the cone is the
/// convex hull of its apex (`sensor_offset` above the base center) and
/// its base rim (radius `target_radius` around the base center), and the
/// squared distance from a fixed point to any point on a line segment is
/// a convex function of the segment parameter, so its maximum over the
/// segment is attained at one of the two endpoints -- giving
/// `max(sensor_offset, target_radius)`, not the two legs' hypotenuse.
/// `target_radius`'s upper bound (0.015) exceeds `sensor_offset` (fixed
/// at 0.005) for every case this generator can produce, so the bound is
/// always `target_radius`'s own maximum.
fn max_cone_reach(radius_range: std::ops::Range<f64>, sensor_offset: f64) -> f64 {
    radius_range.end.max(sensor_offset)
}

fn extent(shape: &Shape) -> f64 {
    match shape {
        Shape::Sphere(s) => s.radius,
        Shape::Cylinder(c) => c.radius.max(c.length / 2.0),
        Shape::Cuboid(b) => {
            let [x, y, z] = b.size;
            (x * x + y * y + z * z).sqrt() / 2.0
        }
        _ => 0.0,
    }
}

/// `--geometry-gaps`: for every pair of `eligible` links, the minimum
/// cone reach (from a near-placement anchored at one link's center) that
/// would be needed to touch the other link's own collision shape --
/// `center_dist - far_link_extent`. Sorted ascending so the tightest,
/// most touching-prone pair is first. `fk` must be a full forward-
/// kinematics result covering every link in `eligible`.
fn print_geometry_gaps(model: &RobotModel, eligible: &[String], fk: &BTreeMap<String, [f64; 16]>) {
    let mut centers = Vec::new();
    for name in eligible {
        let link_model = model
            .link_model(name)
            .expect("name came from parry_representable_link_names(model)");
        let shape = link_model
            .shapes()
            .iter()
            .find(|s| is_parry_representable(&s.shape))
            .expect("name is eligible because it has such a shape");
        let link_fk = isometry_from_row_major(
            fk.get(name)
                .expect("name came from parry_representable_link_names(model)"),
        );
        let center = (link_fk * shape.origin_transform).translation.vector;
        centers.push((name.clone(), center, extent(&shape.shape)));
    }

    let mut gaps = Vec::new();
    for i in 0..centers.len() {
        for j in (i + 1)..centers.len() {
            let center_dist = (centers[i].1 - centers[j].1).norm();
            // Reach needed from a case anchored at i to touch j's shape,
            // and vice versa -- the smaller of the two is what a
            // near-placement at either link would need.
            let reach_i_to_j = center_dist - centers[j].2;
            let reach_j_to_i = center_dist - centers[i].2;
            gaps.push((
                reach_i_to_j.min(reach_j_to_i),
                centers[i].0.clone(),
                centers[j].0.clone(),
                center_dist,
            ));
        }
    }
    gaps.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

    let max_reach = max_cone_reach(0.005..0.015, 0.005);
    println!("generator max cone reach: {max_reach:.4}");
    println!("reach-needed  center-dist  pair");
    for (reach_needed, a, b, center_dist) in &gaps {
        println!("{reach_needed:.4}  {center_dist:.4}  {a} <-> {b}");
    }
    let min_reach_needed = gaps
        .first()
        .map(|(g, ..)| *g)
        .expect("pr2 has more than one eligible link");
    println!(
        "verdict: tightest pair needs {min_reach_needed:.4}, generator's max reach is {max_reach:.4} -- touching >= 2 is {} under this generator + fixture",
        if min_reach_needed > max_reach {
            "GEOMETRICALLY UNREACHABLE"
        } else {
            "reachable"
        }
    );
}

fn build_rust_model(urdf_path: &str, srdf_path: &str) -> Result<RobotModel, String> {
    let urdf_xml =
        std::fs::read_to_string(urdf_path).map_err(|e| format!("reading URDF {urdf_path}: {e}"))?;
    let urdf =
        urdf_rs::read_file(urdf_path).map_err(|e| format!("parsing URDF {urdf_path}: {e}"))?;
    let srdf =
        SrdfModel::parse_file(srdf_path).map_err(|e| format!("parsing SRDF {srdf_path}: {e}"))?;
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .map_err(|e| format!("building RobotModel: {e}"))
}

fn main() -> Result<(), String> {
    let args = parse_args()?;
    let model = build_rust_model(&args.urdf, &args.srdf)?;
    let eligible = parry_representable_link_names(&model);
    eprintln!(
        "seed={} cases={} parry-representable links={}",
        args.seed,
        args.cases,
        eligible.len()
    );

    let mut oracle = Oracle::spawn(&args.oracle, &args.urdf, &args.srdf)?;

    let model_frame = match oracle.ask(Op::ModelInfo)? {
        OracleResult::ModelInfo(m) => m.model_frame,
        _ => return Err("expected model_info".to_owned()),
    };

    if args.geometry_gaps {
        let fk = match oracle.ask(Op::Fk {
            joint_values: BTreeMap::new(),
            links: Vec::new(),
        })? {
            OracleResult::Fk(f) => f.link_transforms,
            _ => return Err("expected fk".to_owned()),
        };
        print_geometry_gaps(&model, &eligible, &fk);
        return Ok(());
    }

    let states = match oracle.ask(Op::RandomStates {
        count: args.cases,
        seed: args.seed,
    })? {
        OracleResult::RandomStates(r) => r.states,
        _ => return Err("expected random_states".to_owned()),
    };

    let mut rng = ChaCha8Rng::seed_from_u64((args.seed as i64 as u64) ^ 0x5EED_C0DE_u64);
    let tf = Transforms::new(model.model_frame()).map_err(|e| format!("{e}"))?;

    // (oracle_distance, local_single_depth) for every touching == 1 case --
    // no traversal-order ambiguity is even possible here (one candidate),
    // so this is a direct, unambiguous measurement of deviation-6's own
    // noise floor for this sweep.
    let mut touching1_diffs: Vec<f64> = Vec::new();
    // Per touching >= 2 case: (oracle_distance, all local contact depths).
    let mut touching2plus: Vec<(f64, Vec<f64>)> = Vec::new();
    let mut touching0 = 0usize;
    let mut touching1 = 0usize;

    for (idx, joint_values) in states.iter().enumerate() {
        let expected_fk = match oracle.ask(Op::Fk {
            joint_values: joint_values.clone(),
            links: Vec::new(),
        })? {
            OracleResult::Fk(f) => f.link_transforms,
            _ => return Err("expected fk".to_owned()),
        };

        let spec = build_case(&mut rng, idx, &model, &model_frame, &eligible, &expected_fk);

        let sensor_pose = isometry_from_row_major(&spec.sensor_pose);
        let target_pose = isometry_from_row_major(&spec.target_pose);
        let constraint = VisibilityConstraint::new(
            &model,
            &tf,
            SensorSpec {
                frame_id: &spec.sensor_frame_id,
                pose: sensor_pose,
                view_direction: SensorViewDirection::SensorZ,
            },
            TargetSpec {
                frame_id: &spec.target_frame_id,
                pose: target_pose,
            },
            spec.cone_sides,
            VisibilityCriteria {
                target_radius: spec.target_radius,
                max_view_angle: spec.max_view_angle,
                max_range_angle: spec.max_range_angle,
            },
            spec.weight,
        )
        .map_err(|e| format!("VisibilityConstraint::new: {e}"))?;

        let expected = match oracle.ask(Op::Constraints {
            joint_values: joint_values.clone(),
            constraints: ConstraintsSpecWire {
                visibility_constraints: vec![spec],
                ..Default::default()
            },
        })? {
            OracleResult::Constraints(c) => c,
            _ => return Err("expected constraints".to_owned()),
        };
        let oracle_result = expected
            .results
            .first()
            .expect("exactly one visibility constraint was sent");

        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        let mut map: HashMap<String, f64> = HashMap::new();
        for (k, v) in joint_values {
            map.insert(k.clone(), *v);
        }
        state
            .set_variable_positions_by_name(&map)
            .map_err(|e| format!("set_variable_positions_by_name: {e}"))?;
        let posed = state.update();

        let touching = constraint.cone_touching_link_count(&posed);
        match touching {
            0 => touching0 += 1,
            1 => {
                touching1 += 1;
                let depths = constraint.cone_contact_depths(&posed);
                let local = depths.first().copied().unwrap_or(0.0);
                let diff = (oracle_result.distance - local).abs();
                touching1_diffs.push(diff);
                println!(
                    "case[{idx}] touching=1 oracle_satisfied={} oracle_distance={:.6e} local_depth={:.6e} |diff|={:.3e}",
                    oracle_result.satisfied, oracle_result.distance, local, diff
                );
            }
            n => {
                let depths = constraint.cone_contact_depths(&posed);
                println!(
                    "case[{idx}] touching={n} oracle_satisfied={} oracle_distance={:.6e} depths={depths:?}",
                    oracle_result.satisfied, oracle_result.distance
                );
                touching2plus.push((oracle_result.distance, depths));
            }
        }
    }

    let tolerance = touching1_diffs.iter().cloned().fold(0.0f64, f64::max);

    println!();
    println!("=== summary ===");
    println!("seed={} cases={}", args.seed, args.cases);
    println!("touching==0: {touching0}");
    println!(
        "touching==1: {touching1} (max |oracle - local| = {:.6e}, this run's measured tolerance)",
        tolerance
    );
    println!("touching>=2: {}", touching2plus.len());

    let mut matched = 0usize;
    for (oracle_distance, depths) in &touching2plus {
        let min_diff = depths
            .iter()
            .map(|d| (d - oracle_distance).abs())
            .fold(f64::INFINITY, f64::min);
        let is_match = min_diff <= tolerance;
        if is_match {
            matched += 1;
        }
        println!(
            "  touching>=2 case: oracle={oracle_distance:.6e} depths={depths:?} min|diff|={min_diff:.3e} match={is_match}"
        );
    }
    println!(
        "verdict: {matched} of {} touching>=2 cases have a contact depth within tolerance ({:.3e}) of the oracle's reported depth",
        touching2plus.len(),
        tolerance
    );
    if touching2plus.is_empty() {
        println!(
            "verdict: 0 touching>=2 cases produced by this sweep -- see this binary's own module doc; report this as a fact, not chase a different seed to force a nonzero count"
        );
    } else if matched == touching2plus.len() {
        println!(
            "verdict: traversal order IS a live explanation -- every touching>=2 case has a matching contact"
        );
    } else if matched == 0 {
        println!(
            "verdict: traversal order is EXCLUDED for every touching>=2 case -- deviation 6 alone explains all of them"
        );
    } else {
        println!(
            "verdict: MIXED -- {matched} of {} touching>=2 cases match (traversal order), {} do not (deviation 6 alone)",
            touching2plus.len(),
            touching2plus.len() - matched
        );
    }

    Ok(())
}
