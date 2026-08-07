// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Measures `PORTING-PLAN.md` §5 Phase 3's `collision: bool` clause on the
//! sub-population that row's own MET note names as unmeasured: pairs where
//! *either* side is a mesh.
//!
//! `exact_tangency_is_decided_per_shape_pair.rs` already pins this port's
//! answer for the full 5x5 `{box, sphere, cylinder, cone, mesh}` grid at
//! three offsets, and `tangency_subset.rs`'s module doc already states that
//! upstream maps `shapes::MESH` to `fcl::BVHModel` -- "a third traversal
//! that is neither specialisation nor MPR" -- and derives nothing further.
//! Neither file puts a mesh cell in front of the oracle. This binary does,
//! reusing exactly §251.2's method: shapes go into `Op::Collision`'s
//! `objects` (world) and `attached_bodies` (robot) fields, on prbt, where
//! `prbt_base_link`'s default-state world transform is the identity, so an
//! attached shape's `shape_pose` *is* its world pose
//! (`the_attached_frame_is_the_world_frame`, same file). No oracle source
//! change, no image rebuild.
//!
//! # Geometry
//!
//! Byte-identical to `exact_tangency_is_decided_per_shape_pair.rs`: the same
//! `HALF = 0.5`, the same `LOWER_CENTRE`, the same synthetic unit-cube mesh
//! (8 vertices, 12 triangles). Built once here rather than imported -- a
//! `tests/` file is not a library this binary can depend on -- so a change to
//! either must be carried to both by hand; a future edit that lets them
//! diverge would read as a collision-backend difference instead of what it
//! actually is.
//!
//! # Coverage, and the one gap this method cannot close
//!
//! `oracle.cpp`'s `parseShape` (`tools/moveit-oracle/src/oracle.cpp:340-378`)
//! has branches for `"sphere"`, `"box"`, `"cylinder"` and `"mesh"` and no
//! `"cone"` branch -- it throws `unsupported shape type cone` for one. That
//! is a wire-protocol ceiling, not a bug to route around: the task that asked
//! for this measurement also forbids editing the oracle source, and the
//! oracle image is not rebuilt for this round. The 9 of 25 grid cells that
//! involve `cone` (`cone x {box, sphere, cylinder, cone, mesh}` and its
//! mirror) are therefore reported from this port alone, each row printed with
//! `"oracle": null` and the reason, rather than silently dropped.
//!
//! # Ladder
//!
//! 11 positive decades `1e-12` through `1e-2` (the same span
//! `tangency_subset.rs`'s `GAP_DECADE_MIN`/`GAP_DECADE_MAX` log-uniformly
//! samples, walked here at one point per decade instead of drawn at random,
//! so a run is exactly reproducible), their 11 negation as constructed
//! overlaps, and exact tangency at `0.0`.
//!
//! # Usage
//!
//! ```text
//! mesh-collision-bool-ladder --urdf F.urdf --srdf F.srdf \
//!     --oracle tools/moveit-oracle/run-oracle.sh
//! ```
//!
//! One NDJSON line per `(upper, lower, delta)` cell on stdout:
//! `{"upper","lower","delta","port_collision","port_distance","oracle_collision","oracle_distance","match"}`,
//! `oracle_*` and `match` are `null` for the 9 cone-involving cells. Exits
//! non-zero iff any non-null `match` is `false`, an oracle request itself
//! errored, or nothing was scored against the oracle at all -- the last of
//! those is not a hypothetical: the whole 5x5 grid this measures has no
//! per-robot decomposition to skip a sub-population legitimately (contrast
//! `penetration_extended.rs`, where a robot with no `box` target is an
//! expected empty run), so zero scored here can only mean the oracle
//! rejected more shape kinds than the 9 cone-involving cells this binary
//! already excludes, or the ladder produced nothing.

use std::io::{BufRead, BufReader, Write};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::sync::Arc;

use moveit_collision::{
    AllowedCollisionMatrix, AttachedBodyGeometry, CollisionEnv, CollisionRequest, DistanceRequest,
    LinkPaddingScale, ParryCollisionEnv, World,
};
use moveit_geometry::{Cone, Cuboid, Cylinder, Isometry3, Mesh, Shape, Sphere, Vector3};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;
use serde_json::{Value, json};

/// Half the extent of every shape below, along every axis. Exactly
/// representable in binary, so the gap at `delta == 0.0` is exactly zero.
const HALF: f64 = 0.5;

/// The lower (world) shape's centre. `x = 5.0` keeps it clear of prbt's own
/// links, mirroring `exact_tangency_is_decided_per_shape_pair.rs`.
const LOWER_CENTRE: (f64, f64, f64) = (5.0, 0.0, -HALF);

/// The gap ladder: 11 negative (constructed overlap), exact zero, 11
/// positive (`1e-12`..`1e-2`, one point per decade).
fn ladder() -> Vec<f64> {
    let mut out = Vec::with_capacity(23);
    for exp in (-12..=-2).rev() {
        out.push(-(10f64.powi(exp)));
    }
    out.push(0.0);
    for exp in -12..=-2 {
        out.push(10f64.powi(exp));
    }
    out
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Kind {
    Box,
    Sphere,
    Cylinder,
    Cone,
    Mesh,
}

const KINDS: [Kind; 5] = [
    Kind::Box,
    Kind::Sphere,
    Kind::Cylinder,
    Kind::Cone,
    Kind::Mesh,
];

impl Kind {
    const fn name(self) -> &'static str {
        match self {
            Self::Box => "box",
            Self::Sphere => "sphere",
            Self::Cylinder => "cylinder",
            Self::Cone => "cone",
            Self::Mesh => "mesh",
        }
    }

    /// `oracle.cpp`'s `parseShape` has no `"cone"` branch -- the one shape
    /// kind this wire protocol cannot build.
    const fn oracle_supported(self) -> bool {
        !matches!(self, Self::Cone)
    }

    /// A shape of this kind whose extent is exactly `HALF` from its own
    /// origin along every axis, so two of them stacked `2 * HALF` apart touch
    /// with a gap of exactly zero. Same construction as
    /// `exact_tangency_is_decided_per_shape_pair.rs`'s `Kind::shape`.
    fn shape(self) -> Arc<Shape> {
        Arc::new(match self {
            Self::Box => Shape::Cuboid(
                Cuboid::new(2.0 * HALF, 2.0 * HALF, 2.0 * HALF).expect("positive cuboid"),
            ),
            Self::Sphere => Shape::Sphere(Sphere::new(HALF).expect("positive sphere")),
            Self::Cylinder => {
                Shape::Cylinder(Cylinder::new(HALF, 2.0 * HALF).expect("positive cylinder"))
            }
            Self::Cone => Shape::Cone(Cone::new(HALF, 2.0 * HALF).expect("positive cone")),
            Self::Mesh => Shape::Mesh(unit_cube_mesh()),
        })
    }

    /// `oracle.cpp`'s `parseShape` JSON for this kind. Never called for
    /// `Cone` -- guarded by [`Kind::oracle_supported`] at every call site.
    fn oracle_shape_json(self) -> Value {
        match self {
            Self::Box => json!({"type": "box", "size": [2.0 * HALF, 2.0 * HALF, 2.0 * HALF]}),
            Self::Sphere => json!({"type": "sphere", "radius": HALF}),
            Self::Cylinder => json!({"type": "cylinder", "radius": HALF, "length": 2.0 * HALF}),
            Self::Cone => unreachable!("cone has no oracle wire encoding"),
            Self::Mesh => {
                let mesh = unit_cube_mesh();
                let vertices: Vec<[f64; 3]> =
                    mesh.vertices.iter().map(|v| [v.x, v.y, v.z]).collect();
                let triangles: Vec<[u32; 3]> = mesh.triangles.clone();
                json!({"type": "mesh", "vertices": vertices, "triangles": triangles})
            }
        }
    }
}

/// A cube spanning `[-HALF, HALF]` on every axis, as 8 vertices and 12
/// triangles. Must stay byte-identical to
/// `exact_tangency_is_decided_per_shape_pair.rs`'s `unit_cube_mesh`.
fn unit_cube_mesh() -> Mesh {
    let mut vertices = Vec::with_capacity(8);
    for &z in &[-HALF, HALF] {
        for &y in &[-HALF, HALF] {
            for &x in &[-HALF, HALF] {
                vertices.push(Vector3::new(x, y, z));
            }
        }
    }
    // Vertex i has bit 0 = +x, bit 1 = +y, bit 2 = +z.
    let triangles = vec![
        [0u32, 2, 1],
        [1, 2, 3], // z = -HALF
        [4, 5, 6],
        [5, 7, 6], // z = +HALF
        [0, 1, 4],
        [1, 5, 4], // y = -HALF
        [2, 6, 3],
        [3, 6, 7], // y = +HALF
        [0, 4, 2],
        [2, 4, 6], // x = -HALF
        [1, 3, 5],
        [3, 7, 5], // x = +HALF
    ];
    Mesh::new(vertices, triangles).expect("cube mesh indices are in range")
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

/// Command line, parsed by hand the way `tangency_subset.rs`'s `Args` is.
struct Args {
    urdf: String,
    srdf: String,
    oracle: Vec<String>,
}

fn parse_args() -> Result<Args, String> {
    let mut urdf = None;
    let mut srdf = None;
    let mut oracle: Vec<String> = Vec::new();

    let mut argv = std::env::args().skip(1);
    while let Some(arg) = argv.next() {
        let want = |name: &str, argv: &mut dyn Iterator<Item = String>| {
            argv.next().ok_or_else(|| format!("{name} needs a value"))
        };
        match arg.as_str() {
            "--urdf" => urdf = Some(want("--urdf", &mut argv)?),
            "--srdf" => srdf = Some(want("--srdf", &mut argv)?),
            "--oracle" => oracle.push(want("--oracle", &mut argv)?),
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
    })
}

/// The oracle subprocess, as a JSON-lines filter. Same shape as
/// `tangency_subset.rs`'s `Oracle`.
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

fn build_model(args: &Args) -> Result<RobotModel, String> {
    let urdf_xml =
        std::fs::read_to_string(&args.urdf).map_err(|e| format!("reading {}: {e}", args.urdf))?;
    let urdf = urdf_rs::read_file(&args.urdf).map_err(|e| format!("parsing {}: {e}", args.urdf))?;
    let srdf =
        SrdfModel::parse_file(&args.srdf).map_err(|e| format!("parsing {}: {e}", args.srdf))?;
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .map_err(|e| format!("building model: {e}"))
}

fn build_acm(args: &Args) -> Result<AllowedCollisionMatrix, String> {
    let srdf =
        SrdfModel::parse_file(&args.srdf).map_err(|e| format!("parsing {}: {e}", args.srdf))?;
    Ok(AllowedCollisionMatrix::from_srdf(&srdf))
}

struct Cell {
    upper: Kind,
    lower: Kind,
    delta: f64,
    port_collision: bool,
    port_distance: f64,
    oracle: Option<(bool, f64)>,
}

fn port_answer(
    model: &RobotModel,
    acm: &AllowedCollisionMatrix,
    upper: Kind,
    lower: Kind,
    delta: f64,
) -> (bool, f64) {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    let posed = state.update();
    let touch_links = std::collections::BTreeSet::new();

    let upper_shapes = [upper.shape()];
    let upper_poses = [Isometry3::translation(
        LOWER_CENTRE.0,
        LOWER_CENTRE.1,
        LOWER_CENTRE.2 + 2.0 * HALF + delta,
    )];
    let attached = AttachedBodyGeometry {
        id: "upper",
        link_name: "prbt_base_link",
        shapes: &upper_shapes,
        shape_poses: &upper_poses,
        touch_links: &touch_links,
    };

    let mut world = World::new();
    world.add_shape(
        "lower",
        lower.shape(),
        Isometry3::translation(LOWER_CENTRE.0, LOWER_CENTRE.1, LOWER_CENTRE.2),
    );
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
    let attached_slice = std::slice::from_ref(&attached);
    let collision = env
        .check_robot_collision(
            &CollisionRequest::default(),
            &posed,
            attached_slice,
            Some(acm),
        )
        .collision;
    let distance_request = DistanceRequest {
        enable_signed_distance: true,
        acm: Some(acm),
        ..DistanceRequest::default()
    };
    let distance = env
        .distance_robot(&distance_request, &posed, attached_slice)
        .minimum_distance
        .distance;
    (collision, distance)
}

fn oracle_answer(
    oracle: &mut Oracle,
    upper: Kind,
    lower: Kind,
    delta: f64,
) -> Result<(bool, f64), String> {
    let upper_pose = row_major(&Isometry3::translation(
        LOWER_CENTRE.0,
        LOWER_CENTRE.1,
        LOWER_CENTRE.2 + 2.0 * HALF + delta,
    ));
    let lower_pose = row_major(&Isometry3::translation(
        LOWER_CENTRE.0,
        LOWER_CENTRE.1,
        LOWER_CENTRE.2,
    ));
    let answer = oracle.ask(json!({
        "op": "collision",
        "joint_values": {},
        "attached_bodies": [{
            "id": "upper",
            "link_name": "prbt_base_link",
            "shapes": [upper.oracle_shape_json()],
            "shape_poses": [upper_pose],
        }],
        "objects": [{
            "id": "lower",
            "pose": lower_pose,
            "shape": lower.oracle_shape_json(),
        }],
    }))?;
    let collision = answer["robot_collision"]
        .as_bool()
        .ok_or("robot_collision is not a boolean")?;
    let distance = answer["robot_distance"]
        .as_f64()
        .ok_or("robot_distance is not a number")?;
    Ok((collision, distance))
}

fn run() -> Result<i32, String> {
    let args = parse_args()?;
    let model = build_model(&args)?;
    let acm = build_acm(&args)?;
    let mut oracle = Oracle::spawn(&args)?;

    let mut cells = Vec::new();
    let mut mismatches = 0usize;
    let mut oracle_errors = Vec::new();

    for delta in ladder() {
        for upper in KINDS {
            for lower in KINDS {
                // Score only the pairs the brief asked for: mesh must appear
                // on at least one side. The rest of the 5x5 grid is already
                // pinned by `exact_tangency_is_decided_per_shape_pair.rs` at
                // three offsets and re-measuring it here would not add
                // coverage.
                if upper != Kind::Mesh && lower != Kind::Mesh {
                    continue;
                }
                let (port_collision, port_distance) =
                    port_answer(&model, &acm, upper, lower, delta);
                let oracle_result = if upper.oracle_supported() && lower.oracle_supported() {
                    match oracle_answer(&mut oracle, upper, lower, delta) {
                        Ok(pair) => Some(pair),
                        Err(e) => {
                            oracle_errors.push(format!(
                                "{} x {} @ {delta:e}: {e}",
                                upper.name(),
                                lower.name()
                            ));
                            None
                        }
                    }
                } else {
                    None
                };
                if let Some((oracle_collision, _)) = oracle_result {
                    if oracle_collision != port_collision {
                        mismatches += 1;
                    }
                }
                cells.push(Cell {
                    upper,
                    lower,
                    delta,
                    port_collision,
                    port_distance,
                    oracle: oracle_result,
                });
            }
        }
    }

    for cell in &cells {
        let (oracle_collision, oracle_distance, matches) = match cell.oracle {
            Some((c, d)) => (json!(c), json!(d), json!(c == cell.port_collision)),
            None => (Value::Null, Value::Null, Value::Null),
        };
        println!(
            "{}",
            json!({
                "upper": cell.upper.name(),
                "lower": cell.lower.name(),
                "delta": cell.delta,
                "port_collision": cell.port_collision,
                "port_distance": cell.port_distance,
                "oracle_collision": oracle_collision,
                "oracle_distance": oracle_distance,
                "match": matches,
            })
        );
    }

    for err in &oracle_errors {
        eprintln!("oracle error: {err}");
    }
    let scored = cells.iter().filter(|c| c.oracle.is_some()).count();
    eprintln!(
        "{} cells, {} scored against the oracle, {} mismatch(es), {} oracle error(s)",
        cells.len(),
        scored,
        mismatches,
        oracle_errors.len(),
    );

    // A run that compared nothing is not a pass: `mismatches` and
    // `oracle_errors` are both vacuously empty when every cell was skipped
    // (an oracle rejecting more kinds than the 9 already-excluded
    // cone-involving cells, or `ladder()` returning nothing), and this
    // binary has no per-robot decomposition -- unlike
    // `penetration_extended.rs`, where a robot legitimately has no `box`
    // target and the empty-population guard belongs in the shell wrapper
    // that sums across robots, every run of this binary measures the same
    // fixed grid on the same fixture, so there is no legitimate reason for
    // it to score zero. The guard therefore belongs here, not in a future
    // wrapper.
    if scored == 0 {
        eprintln!(
            "0 cells were scored against the oracle -- nothing was measured, this is not a pass"
        );
        return Ok(1);
    }

    Ok(if mismatches > 0 || !oracle_errors.is_empty() {
        1
    } else {
        0
    })
}

fn main() {
    match run() {
        Ok(code) => std::process::exit(code),
        Err(e) => {
            eprintln!("error: {e}");
            std::process::exit(2);
        }
    }
}
