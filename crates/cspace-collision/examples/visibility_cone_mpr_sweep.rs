// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No upstream file: `PORTING-PLAN.md`'s deviation-6(b) closure -- extends
// `case104_mpr_input.rs`'s single-case libccd-MPR-vs-parry-EPA comparison to
// N cases, per the coordinator's own round-27 charge: "One case cannot
// establish a class... feed it every `visibility_cone` case where the port
// and the oracle disagree on depth, and report the MPR-vs-EPA gap for
// each."

//! Generates `--cases` near-placement `visibility_cone` cases, each anchored
//! at a real pr2 cylinder link (matching `case104_mpr_input.rs`'s own
//! `bl_caster_l_wheel_link` case, generalized to every wheel), asks the real
//! oracle for its own ground-truth contact depth for the exact (cone,
//! target link) pair, computes this backend's own EPA depth via the
//! identical reconstruction `case104_mpr_input.rs` uses, and -- for every
//! case where the two disagree beyond [`MISMATCH_THRESHOLD`] -- feeds the
//! winning triangle and cylinder to the already-committed, already-generic
//! `tools/mpr-vs-epa/build/mpr_case104` binary to get libccd's own real MPR
//! depth for that exact pair.
//!
//! # Why the oracle's `collision` op, not `constraints`, is the ground
//! truth (closing PORTING-PLAN.md §188 for this harness)
//!
//! `case104_mpr_input.rs` self-checks its reconstruction against
//! `CAPTURED_REFERENCE_DEPTH`, a constant derived by that same
//! reconstruction the first time it ran -- that check only catches a later
//! *regression* in the formula, not a bug that was already present when the
//! constant was captured. A wrong reconstruction (wrong link, wrong joint,
//! a sign or scale error) that still lands inside the touched link's own
//! inscribed sphere would still interpenetrate (round 25's own 15/15
//! measurement: near-placement interpenetrates through the link's own
//! centroid *by construction*, independent of which link or radius), and
//! MPR reads deeper than EPA for most interpenetrating triangle-vs-cylinder
//! pairs regardless of whether the pair is the *right* one -- so comparing
//! this program's own EPA output only to libccd's own MPR output, or only
//! to a frozen constant, would not catch a structurally wrong
//! reconstruction; a plausible-looking, self-consistent, wrong pair of
//! depths is exactly the failure §188 warns about.
//!
//! This binary's own first working version sent `Op::Constraints` (a real
//! `VisibilityConstraint`, same as `visibility_cone_depth_sweep.rs`) and
//! read back the resulting `distance`. `Op::Collision` (`oracle.cpp`'s
//! `collision`) is used instead: it adds the identical cone mesh as a world
//! object and returns *every* contacting pair (`max_contacts = 100`), so
//! this program looks up the *specific* (cone, target link) pair by name
//! rather than trusting `VisibilityConstraint::decide`'s own single
//! `req.max_contacts = 1` collision check
//! (`kinematic_constraint.cpp:1160-1178`). Every sampled case in this
//! binary's own runs has `touching_count == 1` (no sibling link is ever
//! also in contact for this generator -- see `touching_count` in the
//! per-case output), so pair selection is not the source of the finding
//! below; `Op::Collision` is used anyway because it is the more direct
//! question ("what does the target link's own contact say") and because
//! `CollisionRequest::max_contacts_per_pair` (`collision_detection/collision_common.hpp:176`)
//! defaults to `1` even for `Op::Collision` -- MoveIt caps *which single
//! triangle's* contact a mesh-vs-shape pair reports, not just how many
//! pairs are reported, and this program has no way to ask the oracle for
//! every candidate triangle's own depth the way it asks for every pair.
//! Every case run here compares this reconstruction's EPA depth against
//! that pair-specific oracle depth -- the oracle's C++ FCL-based collision
//! computation shares no code with this file's Rust reconstruction, so a
//! structural error here would show up as this backend disagreeing with
//! the oracle on nearly every case, not just producing a plausible-looking
//! gap.
//!
//! # The sign flip: `ccdMPRPenetration`'s own end-cap plateau
//!
//! 9 of 945 measured mismatches (seed 4, 1000 cases) have `mpr_depth <
//! epa_depth` by more than [`GAP_NOISE_FLOOR`] -- a real falsification of
//! deviation 6(b)'s "MPR is deeper by construction" framing, not float
//! noise. Every one of those 9 has `mpr_depth` reading `1.700000e-2` to six
//! significant figures, and pr2's wheel collision cylinders
//! (`fixtures/pr2.urdf`, e.g. `fl_caster_l_wheel_link`) are declared
//! `<cylinder length="0.034" radius="0.074792"/>` -- `0.034 / 2 ==
//! 0.017` exactly. `mpr_case104` is fed the *same* winning triangle this
//! program's own EPA search already identified as deepest (not a
//! different, shallower one FCL might have picked -- see the paragraph
//! above), so this is not a wrong-triangle artifact: `ccdMPRPenetration`
//! itself, called on the true deepest triangle, sometimes converges to the
//! cylinder's own half-length -- consistent with the portal-refinement
//! search finding a witness on the cylinder's flat end-cap edge (a real
//! geometric feature at exactly that offset from the cylinder's own
//! center) instead of on the curved side where the true deepest
//! penetration actually is. This is the same class of caveat
//! `parry.rs`'s existing deviation-6(b) doc already raises for MPR ("not
//! guaranteed to converge to the true minimum-depth witness the way EPA
//! does"), measured here to also run in the *shallow* direction, not only
//! the deep one that motivated the doc originally.
//!
//! # Usage
//!
//! ```text
//! sg docker -c 'cargo run --release --example visibility_cone_mpr_sweep \
//!   -p cspace-collision -- \
//!   --urdf <abs>/fixtures/pr2.urdf --srdf <abs>/fixtures/pr2.srdf \
//!   --seed <N> --cases <N> --mpr-binary <abs>/tools/mpr-vs-epa/build/mpr_case104'
//! ```
//!
//! Absolute paths only -- relative paths fail inside the oracle container
//! (same requirement as `visibility_cone_depth_sweep.rs`). `--mpr-binary`
//! defaults to `tools/mpr-vs-epa/build/mpr_case104` relative to the current
//! directory; build it first with `tools/mpr-vs-epa/build.sh`. `--dump-case
//! <idx>` prints one case's exact fed geometry (the bytes this program
//! would otherwise pipe to `mpr_case104`) and exits; `--dump-contacts
//! <idx>`, usually paired with `--max-contacts-per-pair <N>`, prints every
//! contact the oracle returned for that case's touched pair and exits --
//! see `tools/mpr-vs-epa/README.md`'s own sections on both.

use std::collections::BTreeMap;
use std::io::{BufRead, BufReader, Write};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};

use cspace_core::geometry::{Isometry3, Shape, UnitQuaternion, Vector3};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use nalgebra::{Matrix3, Rotation3, Translation3};
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;
use serde::{Deserialize, Serialize};

/// Four-plus orders of magnitude above this reconstruction's own measured
/// self-consistency noise (`case104_mpr_input.rs`'s `REFERENCE_TOLERANCE`,
/// `1e-9`), comfortably below deviation 6(b)'s own established magnitude
/// floor (`PORTING-PLAN.md` cites gaps in `0.0312..0.1042`m) -- a case whose
/// oracle/backend depths differ by more than this is a real mismatch, not
/// float noise or a borderline touch.
const MISMATCH_THRESHOLD: f64 = 1e-4;

/// `case104_mpr_input.rs`'s own `REFERENCE_TOLERANCE` -- a `gap` inside
/// this band of zero is float noise (the two sides computed the same
/// depth via different code paths), not a real sign difference.
const GAP_NOISE_FLOOR: f64 = 1e-9;

// --- Wire protocol, mirrored from tools/moveit-diff/src/protocol.rs (same
// reproduction this crate's sibling `visibility_cone_depth_sweep.rs`
// already does, and for the same reason -- see that file's own module doc:
// `moveit-diff` is a `[[bin]]`-only crate with no library target this crate
// could depend on, and `cspace-collision` cannot depend on
// `cspace-constraints` either, since the dependency edge runs the other
// way). Only the ops/results this binary actually uses. ---

#[derive(Serialize)]
struct Request {
    id: u64,
    #[serde(flatten)]
    op: Op,
}

#[derive(Serialize)]
#[serde(tag = "op", rename_all = "snake_case")]
enum Op {
    RandomStates {
        count: usize,
        seed: i32,
    },
    Fk {
        joint_values: BTreeMap<String, f64>,
        links: Vec<String>,
    },
    Collision {
        joint_values: BTreeMap<String, f64>,
        objects: Vec<ObjectWire>,
        #[serde(skip_serializing_if = "Option::is_none")]
        max_contacts_per_pair: Option<usize>,
    },
}

#[derive(Serialize)]
struct ObjectWire {
    id: String,
    pose: [f64; 16],
    shape: ShapeWire,
}

#[derive(Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ShapeWire {
    Mesh {
        vertices: Vec<[f64; 3]>,
        triangles: Vec<[u32; 3]>,
    },
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
    RandomStates(RandomStatesResultWire),
    Fk(FkResultWire),
    Collision(CollisionResultWire),
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
struct ContactWire {
    body_name_1: String,
    body_name_2: String,
    depth: f64,
}

#[derive(Deserialize)]
struct CollisionResultWire {
    robot_contacts: Vec<ContactWire>,
}

// --- Oracle client (identical protocol handling to
// `visibility_cone_depth_sweep.rs`'s own `Oracle`). ---

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

/// Every pr2 link whose own first collision shape is a cylinder -- the
/// shape `mpr_case104.c`'s triangle-vs-cylinder comparison supports, and
/// deviation 6(b)'s own established scope (`*_caster_*_wheel_link`).
fn cylinder_link_names(model: &RobotModel) -> Vec<String> {
    model
        .link_models()
        .iter()
        .filter(|link| {
            link.shapes()
                .first()
                .is_some_and(|s| matches!(s.shape, Shape::Cylinder(_)))
        })
        .map(|link| link.name().to_owned())
        .collect()
}

/// `VisibilityConstraint::cone_mesh`'s exact vertex/triangle formula --
/// reproduced here for the same reason `case104_mpr_input.rs` reproduces
/// it (that file's own module doc): `cspace-constraints` depends on this
/// crate, not the other way around.
fn cone_mesh_world(
    world_to_sensor: &Isometry3,
    world_to_target: &Isometry3,
    target_radius: f64,
    cone_sides: usize,
) -> (Vec<Vector3>, Vec<[u32; 3]>) {
    let mut vertices = Vec::with_capacity(cone_sides + 2);
    vertices.push(world_to_sensor.translation.vector);
    vertices.push(world_to_target.translation.vector);
    let delta = 2.0 * std::f64::consts::PI / cone_sides as f64;
    for i in 0..cone_sides {
        let a = delta * i as f64;
        let rim_point_in_target =
            Vector3::new(a.sin() * target_radius, a.cos() * target_radius, 0.0);
        vertices.push((world_to_target * nalgebra::Point3::from(rim_point_in_target)).coords);
    }

    let mut triangles = Vec::with_capacity(cone_sides * 2);
    for i in 1..cone_sides {
        triangles.push([(i + 1) as u32, 0, (i + 2) as u32]);
        triangles.push([(i + 1) as u32, 1, (i + 2) as u32]);
    }
    triangles.push([(cone_sides + 1) as u32, 0, 2]);
    triangles.push([(cone_sides + 1) as u32, 1, 2]);
    (vertices, triangles)
}

/// Same triangle-vs-cylinder search `case104_mpr_input.rs` runs, returning
/// the winning triangle's raw `parry3d_f64::query::contact` distance
/// (negative when penetrating -- see `parry.rs:1443`'s
/// `depth: (-pc.dist).max(0.0)`, the conversion to this crate's own
/// positive-when-penetrating `Contact::depth` convention, applied by the
/// caller, not here) plus every vertex expressed in the cylinder's own
/// local (Z-native) frame, so the caller can hand the winning triangle's
/// three vertices to `mpr_case104` without a second coordinate transform.
fn deepest_triangle_vs_cylinder(
    cyl_frame: &Isometry3,
    cylinder: &cspace_core::geometry::Cylinder,
    vertices: &[Vector3],
    triangles: &[[u32; 3]],
) -> (f64, [u32; 3], Vec<parry3d_f64::math::Vector>) {
    let to_cyl = cyl_frame.inverse();
    let local_vertices: Vec<parry3d_f64::math::Vector> = vertices
        .iter()
        .map(|v| {
            let p = to_cyl.transform_point(&nalgebra::Point3::from(*v));
            parry3d_f64::math::Vector::new(p.x, p.y, p.z)
        })
        .collect();

    let parry_cylinder = parry3d_f64::shape::Cylinder::new(cylinder.length * 0.5, cylinder.radius);
    // See `case104_mpr_input.rs`'s own comment: parry's own canonical
    // Cylinder axis is Y, so this query needs the same Y-onto-Z rotation
    // `convert_shape`'s `axis_fix` applies. Do NOT apply this when calling
    // `mpr_case104` -- libccd's own cylinder support function is already
    // Z-native.
    let axis_fix: parry3d_f64::math::Pose =
        nalgebra::Isometry3::rotation(nalgebra::Vector3::x() * std::f64::consts::FRAC_PI_2).into();
    let identity: parry3d_f64::math::Pose = nalgebra::Isometry3::identity().into();

    let mut best = f64::INFINITY;
    let mut best_tri = [0u32; 3];
    for tri in triangles {
        let p0 = local_vertices[tri[0] as usize];
        let p1 = local_vertices[tri[1] as usize];
        let p2 = local_vertices[tri[2] as usize];
        let triangle = parry3d_f64::shape::Triangle::new(p0, p1, p2);
        let Ok(Some(contact)) =
            parry3d_f64::query::contact(&identity, &triangle, &axis_fix, &parry_cylinder, 0.0)
        else {
            continue;
        };
        if contact.dist < best {
            best = contact.dist;
            best_tri = *tri;
        }
    }
    (best, best_tri, local_vertices)
}

/// The 11-number stdin format `mpr_case104.c` (and `case104_mpr_input.rs`)
/// reads -- factored out of [`run_mpr`] so [`Args::dump_case`] prints the
/// exact bytes this program would otherwise pipe to the MPR binary itself,
/// not a second, possibly-drifting copy of the same format string.
fn mpr_stdin(
    p0: parry3d_f64::math::Vector,
    p1: parry3d_f64::math::Vector,
    p2: parry3d_f64::math::Vector,
    radius: f64,
    length: f64,
) -> String {
    format!(
        "{:.17e} {:.17e} {:.17e}\n{:.17e} {:.17e} {:.17e}\n{:.17e} {:.17e} {:.17e}\n{:.17e} {:.17e}",
        p0.x, p0.y, p0.z, p1.x, p1.y, p1.z, p2.x, p2.y, p2.z, radius, length
    )
}

/// Runs the already-committed, already-generic
/// `tools/mpr-vs-epa/build/mpr_case104` binary once on `(p0, p1, p2,
/// radius, length)` -- the same 11-number stdin format
/// `case104_mpr_input.rs` produces -- and returns `Some(depth)` for
/// `mpr_depth=...`, `None` for `collision=0` (MPR found no overlap; a
/// finding in itself for a case this reconstruction reports as
/// penetrating).
fn run_mpr(
    binary: &str,
    p0: parry3d_f64::math::Vector,
    p1: parry3d_f64::math::Vector,
    p2: parry3d_f64::math::Vector,
    radius: f64,
    length: f64,
) -> Result<Option<f64>, String> {
    let mut child = Command::new(binary)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .map_err(|e| format!("failed to start {binary}: {e}"))?;
    {
        let stdin = child
            .stdin
            .as_mut()
            .ok_or_else(|| format!("{binary} stdin unavailable"))?;
        writeln!(stdin, "{}", mpr_stdin(p0, p1, p2, radius, length))
            .map_err(|e| format!("writing {binary} stdin: {e}"))?;
    }
    let output = child
        .wait_with_output()
        .map_err(|e| format!("waiting for {binary}: {e}"))?;
    if !output.status.success() {
        return Err(format!("{binary} exited with {}", output.status));
    }
    let text = String::from_utf8_lossy(&output.stdout);
    let line = text.trim();
    if let Some(rest) = line.strip_prefix("mpr_depth=") {
        rest.trim()
            .parse::<f64>()
            .map(Some)
            .map_err(|e| format!("parsing mpr_depth {rest:?}: {e}"))
    } else if line == "collision=0" {
        Ok(None)
    } else {
        Err(format!("unrecognized {binary} output: {line:?}"))
    }
}

struct Args {
    urdf: String,
    srdf: String,
    seed: i32,
    cases: usize,
    oracle: Vec<String>,
    mpr_binary: String,
    dump_case: Option<usize>,
    dump_contacts: Option<usize>,
    max_contacts_per_pair: Option<usize>,
}

fn parse_args() -> Result<Args, String> {
    let mut urdf = None;
    let mut srdf = None;
    let mut seed = 0i32;
    let mut cases = 300usize;
    let mut oracle: Vec<String> = vec!["tools/moveit-oracle/run-oracle.sh".to_owned()];
    let mut mpr_binary = "tools/mpr-vs-epa/build/mpr_case104".to_owned();
    let mut dump_case = None;
    let mut dump_contacts = None;
    let mut max_contacts_per_pair = None;

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
            "--mpr-binary" => mpr_binary = want("--mpr-binary")?,
            "--dump-case" => {
                dump_case = Some(
                    want("--dump-case")?
                        .parse()
                        .map_err(|e| format!("--dump-case: {e}"))?,
                )
            }
            "--dump-contacts" => {
                dump_contacts = Some(
                    want("--dump-contacts")?
                        .parse()
                        .map_err(|e| format!("--dump-contacts: {e}"))?,
                )
            }
            "--max-contacts-per-pair" => {
                max_contacts_per_pair = Some(
                    want("--max-contacts-per-pair")?
                        .parse()
                        .map_err(|e| format!("--max-contacts-per-pair: {e}"))?,
                )
            }
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
        mpr_binary,
        dump_case,
        dump_contacts,
        max_contacts_per_pair,
    })
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

fn max_edge(
    p0: parry3d_f64::math::Vector,
    p1: parry3d_f64::math::Vector,
    p2: parry3d_f64::math::Vector,
) -> f64 {
    (p0 - p1)
        .length()
        .max((p1 - p2).length())
        .max((p2 - p0).length())
}

/// Pearson correlation coefficient, `NaN` if either series has zero
/// variance or fewer than two points -- no upstream/crate dependency
/// needed for a two-column linear correlation over at most a few hundred
/// points.
fn pearson(xs: &[f64], ys: &[f64]) -> f64 {
    let n = xs.len();
    if n < 2 || n != ys.len() {
        return f64::NAN;
    }
    let n_f = n as f64;
    let mean_x = xs.iter().sum::<f64>() / n_f;
    let mean_y = ys.iter().sum::<f64>() / n_f;
    let mut cov = 0.0;
    let mut var_x = 0.0;
    let mut var_y = 0.0;
    for (&x, &y) in xs.iter().zip(ys) {
        let dx = x - mean_x;
        let dy = y - mean_y;
        cov += dx * dy;
        var_x += dx * dx;
        var_y += dy * dy;
    }
    if var_x == 0.0 || var_y == 0.0 {
        return f64::NAN;
    }
    cov / (var_x.sqrt() * var_y.sqrt())
}

struct MprRecord {
    idx: usize,
    link: String,
    oracle_distance: f64,
    epa_depth: f64,
    mpr_depth: f64,
    triangle_size: f64,
}

/// One numeric depth reading (oracle, EPA, or MPR), carried with the
/// touched cylinder's own dimensions so [`plateau_kind`] can classify it
/// against *that reading's own* link, not a hardcoded constant -- pr2's 8
/// wheel links happen to share one radius/length (`fixtures/pr2.urdf`),
/// but nothing here assumes that stays true of every fixture this binary
/// is ever pointed at.
#[derive(Clone, Copy)]
struct Reading {
    value: f64,
    radius: f64,
    half_length: f64,
}

/// Round 29's own correction (`f8cbacd`) found that a "9 of 945 shallow"
/// framing hid a much larger population of readings sitting on one of the
/// cylinder's own two characteristic dimensions rather than a genuine
/// contact depth -- not by counting three sampled cases, but by counting
/// every reading. `REL_TOL` is far tighter than the two dimensions could
/// ever coincidentally collide at (pr2's own `radius=0.074792` and
/// `length/2=0.017` differ by more than 4x), so a reading landing inside
/// it is the plateau, not noise.
const PLATEAU_REL_TOL: f64 = 1e-6;

fn plateau_kind(r: Reading) -> &'static str {
    if (r.value - r.radius).abs() <= PLATEAU_REL_TOL * r.radius {
        "radial(radius)"
    } else if (r.value - r.half_length).abs() <= PLATEAU_REL_TOL * r.half_length {
        "axial(length/2)"
    } else {
        "other"
    }
}

/// Prints the exact counts a distributional claim about `readings` should
/// cite -- this is the histogram item 4 of round 29's own charge asked
/// for, so the next round reads a count off this program's own output
/// rather than grepping and tallying printed lines by hand.
fn print_plateau_histogram(label: &str, readings: &[Reading]) {
    let n = readings.len();
    if n == 0 {
        println!("{label} plateau histogram: n=0, nothing to classify");
        return;
    }
    let axial = readings
        .iter()
        .filter(|r| plateau_kind(**r) == "axial(length/2)")
        .count();
    let radial = readings
        .iter()
        .filter(|r| plateau_kind(**r) == "radial(radius)")
        .count();
    let other = n - axial - radial;
    println!(
        "{label} plateau histogram (n={n}): axial(length/2)={axial} ({:.1}%)  radial(radius)={radial} ({:.1}%)  other={other} ({:.1}%)",
        100.0 * axial as f64 / n as f64,
        100.0 * radial as f64 / n as f64,
        100.0 * other as f64 / n as f64,
    );
}

fn main() -> Result<(), String> {
    let args = parse_args()?;
    let model = build_rust_model(&args.urdf, &args.srdf)?;
    let eligible = cylinder_link_names(&model);
    if eligible.is_empty() {
        return Err("no cylinder-shaped links in this model".to_owned());
    }
    eprintln!(
        "seed={} cases={} threshold={:.3e} cylinder links={}",
        args.seed,
        args.cases,
        MISMATCH_THRESHOLD,
        eligible.len()
    );

    let mut oracle = Oracle::spawn(&args.oracle, &args.urdf, &args.srdf)?;

    let states = match oracle.ask(Op::RandomStates {
        count: args.cases,
        seed: args.seed,
    })? {
        OracleResult::RandomStates(r) => r.states,
        _ => return Err("expected random_states".to_owned()),
    };

    let mut rng = ChaCha8Rng::seed_from_u64((args.seed as i64 as u64) ^ 0x5EED_C0DE_u64);

    let mut no_epa_contact = 0usize;
    let mut agreeing = 0usize;
    let mut collision_zero: Vec<usize> = Vec::new();
    let mut mpr_errors: Vec<usize> = Vec::new();
    let mut records: Vec<MprRecord> = Vec::new();
    // Every case that prints an `oracle=`/`epa=` reading (AGREE or
    // MISMATCH, including MPR collision=0) feeds these -- see
    // `print_plateau_histogram`'s own doc for why this is a population
    // count, not a sample.
    let mut oracle_readings: Vec<Reading> = Vec::new();
    let mut epa_readings: Vec<Reading> = Vec::new();
    let mut mpr_readings: Vec<Reading> = Vec::new();

    for (idx, joint_values) in states.iter().enumerate() {
        let expected_fk = match oracle.ask(Op::Fk {
            joint_values: joint_values.clone(),
            links: Vec::new(),
        })? {
            OracleResult::Fk(f) => f.link_transforms,
            _ => return Err("expected fk".to_owned()),
        };

        let link_name = &eligible[idx % eligible.len()];
        let link_model = model
            .link_model(link_name)
            .map_err(|e| format!("{link_name}: {e}"))?;
        let shape = &link_model.shapes()[0];
        let Shape::Cylinder(cylinder) = &shape.shape else {
            return Err(format!("{link_name} shape[0] is not a cylinder"));
        };
        let link_fk = isometry_from_row_major(
            expected_fk
                .get(link_name)
                .ok_or_else(|| format!("fk result missing {link_name}"))?,
        );
        let cyl_frame = link_fk * shape.origin_transform;
        let anchor = cyl_frame.translation.vector;

        let radius = rng.random_range(0.005..0.015);
        let cone_sides = 3 + idx % 6;
        const SENSOR_OFFSET: f64 = 0.005;

        let sensor_pose = Isometry3::from_parts(
            (anchor + Vector3::new(0.0, 0.0, SENSOR_OFFSET)).into(),
            UnitQuaternion::identity(),
        );
        let target_pose = Isometry3::from_parts(anchor.into(), UnitQuaternion::identity());

        let (vertices, triangles) = cone_mesh_world(&sensor_pose, &target_pose, radius, cone_sides);

        // Ground truth: add the identical cone mesh as a world object and
        // ask the oracle's own `collision` op for every contacting pair --
        // see this file's own module doc for why `Op::Constraints`'
        // `max_contacts = 1` first-pair-in-map-order result cannot be
        // trusted here.
        let collision_result = match oracle.ask(Op::Collision {
            joint_values: joint_values.clone(),
            objects: vec![ObjectWire {
                id: "cone".to_owned(),
                pose: pose_row_major(Vector3::new(0.0, 0.0, 0.0), UnitQuaternion::identity()),
                shape: ShapeWire::Mesh {
                    vertices: vertices.iter().map(|v| [v.x, v.y, v.z]).collect(),
                    triangles: triangles.clone(),
                },
            }],
            max_contacts_per_pair: args.max_contacts_per_pair,
        })? {
            OracleResult::Collision(c) => c,
            _ => return Err("expected collision".to_owned()),
        };
        let cone_contacts: Vec<(&str, f64)> = collision_result
            .robot_contacts
            .iter()
            .filter_map(|c| {
                if c.body_name_1 == "cone" {
                    Some((c.body_name_2.as_str(), c.depth))
                } else if c.body_name_2 == "cone" {
                    Some((c.body_name_1.as_str(), c.depth))
                } else {
                    None
                }
            })
            .collect();

        if args.dump_contacts == Some(idx) {
            let named: Vec<(&str, f64)> = cone_contacts
                .iter()
                .filter(|(name, _)| *name == link_name.as_str())
                .copied()
                .collect();
            println!(
                "case[{idx}] link={link_name} max_contacts_per_pair={:?} contacts={named:?}",
                args.max_contacts_per_pair
            );
            return Ok(());
        }

        let touching_count = cone_contacts.len();
        // Upstream's own "not colliding" sentinel (`kinematic_constraint.cpp`
        // `ConstraintEvaluationResult(true, 0.0)` path) when the target link
        // is not itself among the contacting pairs.
        let oracle_distance = cone_contacts
            .iter()
            .find(|(name, _)| *name == link_name.as_str())
            .map_or(0.0, |&(_, depth)| depth);

        let (best, best_tri, local_vertices) =
            deepest_triangle_vs_cylinder(&cyl_frame, cylinder, &vertices, &triangles);

        if args.dump_case == Some(idx) {
            if !best.is_finite() {
                return Err(format!("case[{idx}] has no EPA contact, nothing to dump"));
            }
            let p0 = local_vertices[best_tri[0] as usize];
            let p1 = local_vertices[best_tri[1] as usize];
            let p2 = local_vertices[best_tri[2] as usize];
            print!(
                "{}",
                mpr_stdin(p0, p1, p2, cylinder.radius, cylinder.length)
            );
            return Ok(());
        }

        if !best.is_finite() {
            no_epa_contact += 1;
            println!(
                "case[{idx}] link={link_name} NO EPA CONTACT (reconstruction found no interpenetrating triangle) touching_count={touching_count} oracle_distance={oracle_distance:.6e}"
            );
            continue;
        }

        // parry's raw `contact.dist` is negative when penetrating; convert
        // to this crate's own positive-when-penetrating `Contact::depth`
        // convention (`parry.rs:1443`: `depth: (-pc.dist).max(0.0)`) before
        // comparing against the oracle's own positive-convention distance.
        let epa_depth = (-best).max(0.0);
        let diff = (oracle_distance - epa_depth).abs();
        let half_length = cylinder.length / 2.0;
        oracle_readings.push(Reading {
            value: oracle_distance,
            radius: cylinder.radius,
            half_length,
        });
        epa_readings.push(Reading {
            value: epa_depth,
            radius: cylinder.radius,
            half_length,
        });

        if diff <= MISMATCH_THRESHOLD {
            agreeing += 1;
            println!(
                "case[{idx}] link={link_name} AGREE oracle={oracle_distance:.6e} epa={epa_depth:.6e} |diff|={diff:.3e} touching_count={touching_count}"
            );
            continue;
        }

        let p0 = local_vertices[best_tri[0] as usize];
        let p1 = local_vertices[best_tri[1] as usize];
        let p2 = local_vertices[best_tri[2] as usize];
        match run_mpr(
            &args.mpr_binary,
            p0,
            p1,
            p2,
            cylinder.radius,
            cylinder.length,
        ) {
            Ok(Some(mpr_depth)) => {
                let triangle_size = max_edge(p0, p1, p2);
                println!(
                    "case[{idx}] link={link_name} MISMATCH oracle={oracle_distance:.6e} epa={epa_depth:.6e} mpr={mpr_depth:.6e} gap(mpr-epa)={:.6e} tri_size={triangle_size:.4e} touching_count={touching_count}",
                    mpr_depth - epa_depth
                );
                mpr_readings.push(Reading {
                    value: mpr_depth,
                    radius: cylinder.radius,
                    half_length,
                });
                records.push(MprRecord {
                    idx,
                    link: link_name.clone(),
                    oracle_distance,
                    epa_depth,
                    mpr_depth,
                    triangle_size,
                });
            }
            Ok(None) => {
                collision_zero.push(idx);
                println!(
                    "case[{idx}] link={link_name} MISMATCH oracle={oracle_distance:.6e} epa={epa_depth:.6e} mpr=collision=0 touching_count={touching_count} (libccd found NO overlap for a pair this reconstruction reports as penetrating)"
                );
            }
            Err(e) => {
                mpr_errors.push(idx);
                eprintln!("case[{idx}] link={link_name} mpr_case104 error: {e}");
            }
        }
    }

    println!();
    println!("=== summary ===");
    println!("seed={} cases={}", args.seed, args.cases);
    println!("no_epa_contact: {no_epa_contact}");
    println!("agreeing (|oracle - epa| <= {MISMATCH_THRESHOLD:.3e}): {agreeing}");
    println!("mismatches with a real MPR reading: {}", records.len());
    println!(
        "mismatches where MPR reported collision=0: {}",
        collision_zero.len()
    );
    if !collision_zero.is_empty() {
        println!("  case indices: {collision_zero:?}");
    }
    println!(
        "mismatches where mpr_case104 itself errored: {}",
        mpr_errors.len()
    );
    if !mpr_errors.is_empty() {
        println!("  case indices: {mpr_errors:?}");
    }

    println!();
    println!(
        "plateau histograms -- does this reading sit on one of the cylinder's own two dimensions rather than a real contact depth? (round 29's own correction, f8cbacd)"
    );
    print_plateau_histogram("oracle", &oracle_readings);
    print_plateau_histogram("epa", &epa_readings);
    print_plateau_histogram("mpr", &mpr_readings);

    if records.is_empty() {
        println!(
            "verdict: 0 mismatches produced a real MPR reading -- report this as a fact, not chase a different seed to force a nonzero count"
        );
        return Ok(());
    }

    let gaps: Vec<f64> = records.iter().map(|r| r.mpr_depth - r.epa_depth).collect();
    let epa_depths: Vec<f64> = records.iter().map(|r| r.epa_depth).collect();
    let triangle_sizes: Vec<f64> = records.iter().map(|r| r.triangle_size).collect();

    let mpr_deeper = gaps.iter().filter(|&&g| g > GAP_NOISE_FLOOR).count();
    let mpr_shallower = gaps.iter().filter(|&&g| g < -GAP_NOISE_FLOOR).count();
    let mpr_equal = gaps.len() - mpr_deeper - mpr_shallower;
    let min_gap = gaps.iter().cloned().fold(f64::INFINITY, f64::min);
    let max_gap = gaps.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let mean_gap = gaps.iter().sum::<f64>() / gaps.len() as f64;

    println!();
    println!(
        "gap = mpr_depth - epa_depth, both in this crate's positive-when-penetrating convention"
    );
    println!(
        "mpr deeper than epa (gap > {GAP_NOISE_FLOOR:.0e}): {mpr_deeper} of {}",
        gaps.len()
    );
    println!(
        "mpr shallower than epa (gap < -{GAP_NOISE_FLOOR:.0e}): {mpr_shallower} of {}",
        gaps.len()
    );
    println!(
        "mpr == epa within float noise (|gap| <= {GAP_NOISE_FLOOR:.0e}): {mpr_equal} of {}",
        gaps.len()
    );
    println!("gap min={min_gap:.6e} max={max_gap:.6e} mean={mean_gap:.6e}");
    println!(
        "pearson(gap, epa_depth)={:.4}  pearson(gap, triangle_size)={:.4}  (n={})",
        pearson(&gaps, &epa_depths),
        pearson(&gaps, &triangle_sizes),
        gaps.len()
    );

    if mpr_shallower > 0 {
        println!();
        println!(
            "SIGN FLIP -- {mpr_shallower} case(s) where MPR reported a SHALLOWER depth than EPA:"
        );
        for r in &records {
            let gap = r.mpr_depth - r.epa_depth;
            if gap < -GAP_NOISE_FLOOR {
                println!(
                    "  case[{}] link={} oracle={:.6e} epa={:.6e} mpr={:.6e} gap={:.6e}",
                    r.idx, r.link, r.oracle_distance, r.epa_depth, r.mpr_depth, gap
                );
            }
        }
        println!(
            "verdict: deviation 6(b)'s \"by construction\" framing is FALSIFIED by the case(s) above -- MPR is not always deeper than EPA for this pair shape"
        );
    } else {
        println!(
            "verdict: MPR was deeper than (or equal to) EPA in every one of {} mismatch cases with a real MPR reading -- no sign flip found in this sample",
            gaps.len()
        );
    }

    Ok(())
}
