// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Measures `PORTING-PLAN.md` §5 Phase 3's `collision: bool` clause on the
//! sub-population where neither narrow phase's boundary convention can decide
//! the answer -- the shape pairs on which upstream and this port compute the
//! *same* geometric predicate with the *same* inclusive boundary.
//!
//! The committed sweep leaves that clause UNMET for one reason: `fcl::collide`
//! dispatches per shape pair, and the pairs it leaves to libccd answer `false`
//! at exact contact while the pairs it specialises answer `true`. prbt's
//! `cylinder x box` is one of the unspecialised ones, so all 6,854 of that
//! sweep's disagreements are one blank cell being sampled 10,000 times. That
//! is a fact about *which pairs* the reference is uniform on, not about the
//! clause: the pairs where both sides run the same closed form are comparable,
//! and this binary is that measurement.
//!
//! # The two dispatch tables, and the four cells that survive both
//!
//! Every fcl line number below is read at **tag `0.7.0`**
//! (`df2702ca5e703dec98ebd725782ce13862e87fc8`), not at the local checkout's
//! `e5efcc4` HEAD, per PORTING-PLAN.md §135: the oracle image links
//! `libfcl-dev 0.7.0-3build2`, which that section establishes is pure upstream
//! 0.7.0. §283.7 measured what the distinction costs -- `gjk_libccd-inl.h` is
//! 95 lines apart between the two revisions.
//!
//! fcl headers are cited by basename; their directories, all under
//! `include/fcl`, are
//! `narrowphase/detail/gjk_solver_libccd-inl.h`,
//! `narrowphase/collision_request.h` and
//! `narrowphase/detail/primitive_shape_algorithm/{sphere_sphere,sphere_box,sphere_cylinder,box_box}-inl.h`.
//!
//! **Upstream.** `GJKSolver_libccd<S>::shapeIntersect` forwards to
//! `ShapeIntersectLibccdImpl<S, Shape1, Shape2>::run`, whose generic body calls
//! `detail::GJKCollide` -- libccd's MPR, which tests *interior* overlap
//! strictly and so finds nothing when the clearance is exactly zero. The pairs
//! that escape it are the explicit template specialisations registered by
//! `FCL_GJK_LIBCCD_SHAPE_INTERSECT` / `FCL_GJK_LIBCCD_SHAPE_SHAPE_INTERSECT`
//! at `gjk_solver_libccd-inl.h:245-267`, and those reject only on a *strict*
//! separation test, so exact contact falls through to contact generation.
//! Restricted to the seven shape types §251.1 enumerates, the registrations are
//! `Sphere/Sphere`, `Box/Box`, `Sphere/Capsule`, `Sphere/Box` and
//! `Sphere/Cylinder`, the last three expanding to both argument orders: eight
//! of forty-nine cells. MoveIt never overrides the solver -- `GST_LIBCCD` is
//! `CollisionRequest`'s default (`collision_request.h:102`) and the fixed
//! moveit2 checkout names no `gjk_solver_type` anywhere (§251.1).
//!
//! **This port.** `parry3d_f64`'s `DefaultQueryDispatcher::contact` has the
//! same shape and a *different* blank set
//! (`parry3d-f64-0.30.0/src/query/default_query_dispatcher.rs:305-359`):
//! `Ball`/`Ball` goes to `contact_ball_ball`, a ball against anything convex
//! goes to `contact_ball_convex_polyhedron` /
//! `contact_convex_polyhedron_ball`, and everything else -- including
//! `Cuboid`/`Cuboid`, whose closed form sits commented out at
//! `parry3d-f64-0.30.0/src/query/default_query_dispatcher.rs:317-320` --
//! falls through to `contact_support_map_support_map`, a GJK whose boundary is
//! set by an iteration tolerance rather than by the surfaces. §229.2 measured
//! that tolerance: on prbt's base cylinder against a `4x4x0.1` box the
//! effective boundary sits near `5e-8 m` of *clear air*.
//!
//! A pair is comparable only if it is specialised on **both** sides, and the
//! two specialisations must take the same side of the boundary:
//!
//! | pair | fcl | parry | comparable |
//! |---|---|---|---|
//! | `sphere x box`      | `sphereBoxIntersect` | `contact_*_ball` | yes |
//! | `sphere x cylinder` | `sphereCylinderIntersect` | `contact_*_ball` | yes |
//! | `sphere x sphere`   | `sphereSphereIntersect` | `contact_ball_ball` | **no** |
//! | `box x box`         | `boxBoxIntersect` | GJK | **no** |
//! | `box x cylinder`    | libccd MPR | GJK | **no** |
//! | `cylinder x cylinder` | libccd MPR | GJK | **no** |
//!
//! `sphere x sphere` is specialised on both sides and still excluded, because
//! the two closed forms disagree about the boundary itself: fcl rejects on
//! `if(len > s1.radius + s2.radius) return false;`
//! (`sphere_sphere-inl.h:72-73`) -- contact included -- while parry accepts on
//! `if distance_squared < sum_radius_with_error * sum_radius_with_error`
//! (`parry3d-f64-0.30.0/src/query/contact/contact_ball_ball.rs:16`) -- contact
//! excluded. §251.3 measured that cell as this port's one non-uniform tangency
//! answer.
//!
//! The two that survive share a predicate the way §283's three did. fcl finds
//! the nearest point *inside* the other body and rejects on
//! `if (squared_distance > r * r) return false;`
//! (`sphere_box-inl.h:119-120`, `sphere_cylinder-inl.h:136-137`); parry
//! projects the ball centre onto the same body with the same closed-form clamp
//! (`parry3d-f64-0.30.0/src/query/point/point_cuboid.rs:8-12`,
//! `parry3d-f64-0.30.0/src/query/point/point_cylinder.rs:7-70`) and accepts on
//! `if dist <= prediction`
//! (`parry3d-f64-0.30.0/src/query/contact/contact_ball_convex_polyhedron.rs:52`)
//! with `prediction` `0.0`. Same quantity, same inclusive side, no iteration
//! tolerance on either.
//!
//! # What the corpus is
//!
//! One world object -- the probe -- placed against a link whose collision
//! geometry is exactly one primitive, with every other robot-link/probe pair
//! allowed away through the ACM so the query has exactly one pair and its
//! shape pair is therefore well defined. Links carrying several collision
//! elements are skipped: the ACM keys on the link *name*, so unmasking such a
//! link would expose all of its shapes as separate pairs.
//!
//! The placement is *constructed*, not drawn from a box around the link: a
//! point is sampled on the target's surface, the probe is placed along the
//! outward normal there so that the clearance between the two surfaces is
//! exactly a drawn `gap`, and `gap` is drawn log-uniformly over
//! `[GAP_DECADE_MIN, GAP_DECADE_MAX]` decades with a random sign, plus a rung
//! at exactly zero. A corpus of random offsets would never come within a
//! decade of the boundary and would agree for a reason that has nothing to do
//! with this clause.
//!
//! `gap` is a parameter of the query, fixed before either side is asked. It is
//! not a distance either implementation reported.
//!
//! # What is not scored, and why each one is named
//!
//! - **The zero rung.** At `gap == 0` the clearance each side actually sees is
//!   its own rounding of `link_transform * shape_origin * probe_pose`, and the
//!   two roundings are independent. Nothing about either boundary convention
//!   is being measured there. It is sampled and reported per pair anyway,
//!   because the reader must be able to see what it does rather than be told.
//! - **The four incomparable pairs.** Measured and reported as control arms,
//!   never scored. They are what shows the corpus has power: the two libccd
//!   cells disagree over the whole positive band below parry's GJK boundary,
//!   at the same gaps where the scored pairs agree.
//! - **Meshes, and links with several collision elements.** Upstream maps
//!   `shapes::MESH` to `fcl::BVHModel`, a third traversal that is neither
//!   specialisation nor MPR; this section derives nothing about it.
//!
//! # Usage
//!
//! ```text
//! tangency-subset --urdf F.urdf --srdf F.srdf \
//!     --oracle tools/moveit-oracle/run-oracle.sh [--states N] [--seed S]
//! ```
//!
//! Exits non-zero when a scored pair disagrees, when the robot offers no
//! target link, when too few samples were kept, when no control arm from a
//! libccd cell disagreed (the corpus lost its power to separate), or when the
//! two sides' forward kinematics for a target link differ by more than
//! [`FK_FLOOR`] -- past which the finest gap rung would be measuring FK rather
//! than either narrow phase.

use std::io::{BufRead, BufReader, Write};
use std::process::{Child, ChildStdin, ChildStdout, Command, Stdio};
use std::sync::Arc;

use cspace_collision::{
    AllowedCollisionMatrix, CollisionEnv, CollisionRequest, LinkPaddingScale, ParryCollisionEnv,
    World,
};
use cspace_geometry::{Cuboid, Cylinder, Isometry3, Shape, Sphere, UnitQuaternion, Vector3};
use cspace_model::{MeshSearchPaths, RobotModel};
use cspace_srdf::SrdfModel;
use cspace_state::RobotState;
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;
use serde_json::{Value, json};

/// The world object's id. Not a link name in any committed fixture, which is
/// what lets the ACM address it.
const PROBE_ID: &str = "probe";

/// Finest gap decade the corpus samples, as a base-ten exponent.
///
/// Four decades above the largest FK divergence [`FK_FLOOR`] admits, and four
/// decades *below* the `5e-8` clear-air boundary §229.2 measured for parry's
/// GJK path -- so a gap at this rung is unambiguous geometry for the scored
/// pairs and is inside the divergent band for the control arms.
const GAP_DECADE_MIN: f64 = -12.0;

/// Coarsest gap decade the corpus samples, as a base-ten exponent. Well clear
/// of every boundary either library has; these samples are the corpus's own
/// control that a plain non-contact is called the same by both sides.
const GAP_DECADE_MAX: f64 = -2.0;

/// One in this many placements is drawn at `gap == 0` exactly.
const TANGENT_IN: u32 = 6;

/// Largest element-wise difference tolerated between the two sides' global
/// transform for a target link.
///
/// Not a tolerance on the answer -- the answer is a boolean and has none. It
/// bounds the *input*: the corpus asserts a clearance of `10^GAP_DECADE_MIN`,
/// and if the two sides disagreed about where the link is by more than that,
/// the finest rung would be measuring forward kinematics. Sized from
/// measurement, not from Phase 2's `1e-9` clause, which is five decades too
/// loose to protect anything here: the worst element-wise divergence measured
/// over the four robots is `1.110223e-15`, so this floor is two decades under
/// the finest gap rung and nine times the observed worst.
const FK_FLOOR: f64 = 1e-14;

/// Fewest scored samples a robot's run may keep and still be a measurement.
const MIN_SCORED: usize = 100;

/// The three primitives this corpus can put on either side of a pair.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Debug)]
enum Kind {
    Box,
    Cylinder,
    Sphere,
}

impl Kind {
    fn name(self) -> &'static str {
        match self {
            Self::Box => "box",
            Self::Cylinder => "cylinder",
            Self::Sphere => "sphere",
        }
    }
}

/// Every probe kind, run against every target, so the control arms come out of
/// the same corpus as the scored ones rather than a second construction.
const PROBES: [Kind; 3] = [Kind::Sphere, Kind::Box, Kind::Cylinder];

/// The unordered pair name, which is how both dispatch tables key.
fn pair_name(a: Kind, b: Kind) -> String {
    let (lo, hi) = if a <= b { (a, b) } else { (b, a) };
    format!("{} x {}", lo.name(), hi.name())
}

/// A pair is scored iff exactly one side is a sphere.
///
/// That is the derivation above read as a predicate, not a list: `sphere x
/// {box, cylinder}` is where fcl's registration table and parry's dispatcher
/// both leave the generic path, on the same side of the boundary. Two spheres
/// land on opposite sides (`contact_ball_ball`'s strict `<`), and no sphere at
/// all means parry's GJK decides it.
fn is_scored(a: Kind, b: Kind) -> bool {
    (a == Kind::Sphere) != (b == Kind::Sphere)
}

/// A pair fcl leaves to libccd MPR: neither side a sphere, and not `box x box`.
///
/// These are the blank cells of §251.1's table restricted to what a URDF can
/// build, and they are the arms that must fire for the corpus to have power.
fn is_libccd_cell(a: Kind, b: Kind) -> bool {
    a != Kind::Sphere && b != Kind::Sphere && !(a == Kind::Box && b == Kind::Box)
}

/// Command line, parsed by hand the way `moveit-diff`'s own `Config` is.
struct Args {
    urdf: String,
    srdf: String,
    oracle: Vec<String>,
    /// Placements drawn per target link per probe kind.
    states: usize,
    /// Seed for the oracle's `random_states` *and* (xor-folded) for this
    /// side's placement draws, so one number replays the whole corpus.
    seed: i64,
}

fn parse_args() -> Result<Args, String> {
    let mut urdf = None;
    let mut srdf = None;
    let mut oracle: Vec<String> = Vec::new();
    let mut states = 40usize;
    let mut seed = 1i64;

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
        moveit_diff::wait_or_kill(&mut self.child, moveit_diff::ORACLE_SHUTDOWN_TIMEOUT);
    }
}

/// One link this corpus can use: exactly one collision element, and that
/// element is one of the three primitives whose pairs the derivation covers.
struct Target {
    link: String,
    /// The shape's pose in the link's own frame.
    origin: Isometry3,
    kind: Kind,
    /// `box`: half extents. `cylinder`: `[radius, radius, length / 2]`.
    /// `sphere`: `[radius; 3]`.
    half: [f64; 3],
}

impl Target {
    /// Smallest feature of the target, which caps how far a placement may
    /// penetrate before the contact stops being the one face this corpus
    /// constructed.
    fn min_half(&self) -> f64 {
        self.half.iter().copied().fold(f64::INFINITY, f64::min)
    }
}

/// Every link of `model` that qualifies, in model order.
fn targets(model: &RobotModel) -> Vec<Target> {
    let mut out = Vec::new();
    for link in model.link_models() {
        let shapes = link.shapes();
        if shapes.len() != 1 {
            continue;
        }
        let (kind, half) = match &shapes[0].shape {
            Shape::Sphere(s) => (Kind::Sphere, [s.radius, s.radius, s.radius]),
            Shape::Cuboid(b) => (
                Kind::Box,
                [b.size[0] / 2.0, b.size[1] / 2.0, b.size[2] / 2.0],
            ),
            Shape::Cylinder(c) => (Kind::Cylinder, [c.radius, c.radius, c.length / 2.0]),
            _ => continue,
        };
        // `all(> 0.0)` rather than `any(<= 0.0)`: a NaN half-extent fails the
        // strict-positive test and passes the negated one, and it would reach
        // `surface_point` as a placement this corpus cannot score.
        if !half.iter().all(|e| *e > 0.0) {
            continue;
        }
        out.push(Target {
            link: link.name().to_owned(),
            origin: shapes[0].origin_transform,
            kind,
            half,
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

/// A point on the target's surface and the outward unit normal there, both in
/// the shape's own frame.
///
/// Sampled away from edges and rims -- `EDGE_MARGIN` of the way in on every
/// tangential axis -- so the nearest feature is the one face the placement
/// below assumes, and the constructed clearance is exactly the drawn gap
/// rather than a corner distance.
fn surface_point(target: &Target, rng: &mut ChaCha8Rng) -> (Vector3, Vector3) {
    /// Fraction of a face's half-extent a sampled point may reach.
    const EDGE_MARGIN: f64 = 0.6;

    let [hx, _, hz] = target.half;
    match target.kind {
        Kind::Box => {
            let axis = rng.random_range(0..3usize);
            let sign = if rng.random_range(0..2u8) == 0 {
                1.0
            } else {
                -1.0
            };
            let mut p = Vector3::zeros();
            for i in 0..3 {
                let reach = EDGE_MARGIN * target.half[i];
                p[i] = if i == axis {
                    sign * target.half[i]
                } else {
                    rng.random_range(-reach..reach)
                };
            }
            let mut n = Vector3::zeros();
            n[axis] = sign;
            (p, n)
        }
        Kind::Cylinder => {
            let theta = rng.random_range(0.0..std::f64::consts::TAU);
            let (s, c) = theta.sin_cos();
            if rng.random_range(0..2u8) == 0 {
                // A cap: the flat disc at +-half length, normal along z.
                let sign = if rng.random_range(0..2u8) == 0 {
                    1.0
                } else {
                    -1.0
                };
                let rho = rng.random_range(0.0..EDGE_MARGIN * hx);
                (
                    Vector3::new(rho * c, rho * s, sign * hz),
                    Vector3::new(0.0, 0.0, sign),
                )
            } else {
                // The lateral surface, normal radial.
                let z = rng.random_range(-EDGE_MARGIN * hz..EDGE_MARGIN * hz);
                (Vector3::new(hx * c, hx * s, z), Vector3::new(c, s, 0.0))
            }
        }
        Kind::Sphere => {
            let n = unit_vector(rng);
            (n * hx, n)
        }
    }
}

/// A uniformly distributed unit vector, by rejection in the cube.
fn unit_vector(rng: &mut ChaCha8Rng) -> Vector3 {
    loop {
        let v = Vector3::new(
            rng.random_range(-1.0..1.0),
            rng.random_range(-1.0..1.0),
            rng.random_range(-1.0..1.0),
        );
        let n = v.norm();
        if n > 0.25 {
            return v / n;
        }
    }
}

/// The probe: a shape, its support extent back along the approach normal from
/// its own centre, and the rotation that turns its local `+z` into that normal.
struct Probe {
    shape: Shape,
    extent: f64,
    rotation: UnitQuaternion,
    json: Value,
}

/// Draws a probe of `kind` approaching along `normal`.
///
/// Sizes are log-uniform over the same 2.18 decades §283 settled on
/// (`10^-2.7` to `10^-0.52`), which keeps every probe three decades clear of
/// `parry`'s `DEFAULT_EPSILON` degenerate-projection branch
/// (`parry3d-f64-0.30.0/src/query/contact/contact_ball_convex_polyhedron.rs:37`
/// tests `len >= DEFAULT_EPSILON`, and `DEFAULT_EPSILON` is `Real::EPSILON` at
/// `parry3d-f64-0.30.0/src/math/mod.rs:43`), so no scored sample is decided by
/// that fallback instead of by the projection.
fn probe(kind: Kind, normal: Vector3, rng: &mut ChaCha8Rng) -> Result<Probe, String> {
    let mut size = || 10f64.powf(rng.random_range(-2.7..-0.52));
    // `+z` onto the outward normal, so a flat probe presents a face to the
    // target's face rather than an edge. Antiparallel has no unique rotation;
    // a half turn about `x` is the one that maps `+z` to `-z`.
    let rotation = UnitQuaternion::rotation_between(&Vector3::z(), &normal).unwrap_or_else(|| {
        UnitQuaternion::from_axis_angle(&Vector3::x_axis(), std::f64::consts::PI)
    });
    let (shape, extent, json) = match kind {
        Kind::Sphere => {
            let r = size();
            (
                Shape::Sphere(Sphere::new(r).map_err(|e| format!("probe sphere {r}: {e}"))?),
                r,
                json!({"type": "sphere", "radius": r}),
            )
        }
        Kind::Box => {
            let (x, y, z) = (size(), size(), size());
            (
                Shape::Cuboid(
                    Cuboid::new(2.0 * x, 2.0 * y, 2.0 * z)
                        .map_err(|e| format!("probe box {x} {y} {z}: {e}"))?,
                ),
                z,
                json!({"type": "box", "size": [2.0 * x, 2.0 * y, 2.0 * z]}),
            )
        }
        Kind::Cylinder => {
            let (r, h) = (size(), size());
            (
                Shape::Cylinder(
                    Cylinder::new(r, 2.0 * h).map_err(|e| format!("probe cylinder {r}: {e}"))?,
                ),
                h,
                json!({"type": "cylinder", "radius": r, "length": 2.0 * h}),
            )
        }
    };
    Ok(Probe {
        shape,
        extent,
        rotation,
        json,
    })
}

/// Per-pair tallies, so a verdict names the pairs it covered rather than a
/// single count that could be one pair repeated.
#[derive(Default)]
struct Tally {
    kept: usize,
    disagrees: usize,
    /// Samples drawn at `gap == 0`, and how many of those disagreed. Held
    /// apart from `kept`/`disagrees` because the zero rung measures the two
    /// sides' transform rounding, not their boundary conventions.
    tangent: usize,
    tangent_disagrees: usize,
    /// Smallest `|gap|` at which the two sides agreed, and largest at which
    /// they did not. Together they bracket the boundary this pair actually
    /// has: a control arm's `disagree_max` is the width of its divergent band.
    agree_min: f64,
    disagree_max: f64,
    /// Worst-first, capped, so a failure is diagnosable without a re-run.
    outliers: Vec<String>,
}

impl Tally {
    const OUTLIERS: usize = 6;

    fn record(&mut self, gap: f64, agreed: bool, describe: impl FnOnce() -> String) {
        if gap == 0.0 {
            self.tangent += 1;
            if !agreed {
                self.tangent_disagrees += 1;
            }
            return;
        }
        self.kept += 1;
        if agreed {
            if self.agree_min == 0.0 || gap.abs() < self.agree_min {
                self.agree_min = gap.abs();
            }
        } else {
            self.disagrees += 1;
            if gap.abs() > self.disagree_max {
                self.disagree_max = gap.abs();
            }
            if self.outliers.len() < Self::OUTLIERS {
                self.outliers.push(describe());
            }
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

/// This side's answer for one placement: `check_robot_collision` through an
/// ACM that allows every pair away except `target` against [`PROBE_ID`].
///
/// `contacts`/`max_contacts` mirror the oracle's `sceneCollisionSummary`,
/// whose `robot_req.contacts = true; robot_req.max_contacts = 100;` selects
/// the `collisionCallback` branch that sets `res.collision` inside
/// `if (num_contacts > 0)`.
fn port_collision(
    model: &RobotModel,
    link_names: &[String],
    joint_values: &Value,
    target: &str,
    shape: &Shape,
    pose: Isometry3,
) -> Result<bool, String> {
    let mut world = World::new();
    world.add_shape(PROBE_ID, Arc::new(shape.clone()), pose);
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());

    let mut names: Vec<String> = link_names.to_vec();
    names.push(PROBE_ID.to_owned());
    let mut acm = AllowedCollisionMatrix::from_names(&names, true);
    acm.set_entry(PROBE_ID, target, false);

    let mut state = RobotState::new(model);
    state.set_to_default_values();
    apply_joint_values(&mut state, joint_values)?;
    let posed = state.update();

    let request = CollisionRequest {
        contacts: true,
        max_contacts: 100,
        max_contacts_per_pair: 1,
        ..CollisionRequest::default()
    };
    Ok(env
        .check_robot_collision(&request, &posed, &[], Some(&acm))
        .collision)
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

/// Proves the ACM diff reaches the oracle's query, once per target link.
///
/// The corpus's claim to a well-defined shape pair is that exactly one pair is
/// visited, and that rests entirely on `set_acm_entry` being applied. This
/// shows it from one response per target: a probe sphere is placed at the link
/// shape's own origin -- an overlap by construction, whatever the shape --
/// every link *including* the target is allowed away, and the two summaries in
/// the same answer must disagree in the one way only an applied diff can
/// produce: `parent_before` colliding because the pair is there, `child` clear
/// because the diff took it away.
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
    let pose = state
        .update()
        .global_link_transform(&target.link)
        .map_err(|e| format!("FK for {}: {e}", target.link))?
        * target.origin;

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
            "pose": row_major(&pose),
            "shape": {"type": "sphere", "radius": target.min_half()},
        }],
        "diff": all_masked,
    }))?;
    let unmasked = answer["parent_before"]["robot_collision"]
        .as_bool()
        .ok_or("parent_before.robot_collision is not a boolean")?;
    let masked = answer["child"]["robot_collision"]
        .as_bool()
        .ok_or("child.robot_collision is not a boolean")?;

    if !unmasked {
        return Err(format!(
            "{}: a probe at the link shape's own origin did not collide with it -- \
             the mask proof needs a pair to remove",
            target.link
        ));
    }
    if masked {
        return Err(format!(
            "{}: allowing every link away still reported a collision -- the ACM diff did not \
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

/// The largest `|a[i] - b[i]|` over paired slices, NaN-propagating.
///
/// `.fold(0.0, f64::max)` looks like the obvious way to write this, but
/// `f64::max` is IEEE `maxNum`: it discards a NaN operand wherever it
/// appears rather than propagating it, so seeded from a non-NaN `0.0` that
/// fold can never actually return NaN -- a NaN transform entry (a Rust-side
/// FK bug; the oracle's JSON answer can't produce one, `serde_json` rejects
/// NaN as a number token) would silently read as "every element agrees"
/// instead of failing [`fk_divergence`]'s caller's `fk > FK_FLOOR` gate.
/// Same helper, same reason as `state_ops.rs::worst_abs_diff` in the other
/// binary.
fn worst_abs_diff(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y).abs())
        .fold(0.0f64, |acc, d| if d.is_nan() || d > acc { d } else { acc })
}

/// Largest element-wise difference between this side's global transform for
/// each target link and the oracle's, at one state.
fn fk_divergence(
    oracle: &mut Oracle,
    joint_values: &Value,
    targets: &[Target],
    poses: &[Isometry3],
) -> Result<f64, String> {
    let links: Vec<&str> = targets.iter().map(|t| t.link.as_str()).collect();
    let answer = oracle.ask(json!({
        "op": "fk", "joint_values": joint_values, "links": links
    }))?;
    let transforms = answer["link_transforms"]
        .as_object()
        .ok_or("fk.link_transforms is not an object")?;

    let mut worst = 0.0f64;
    for (target, pose) in targets.iter().zip(poses) {
        let flat = transforms
            .get(&target.link)
            .and_then(Value::as_array)
            .ok_or_else(|| format!("fk did not answer for {}", target.link))?;
        if flat.len() != 16 {
            return Err(format!("fk for {} is not 4x4", target.link));
        }
        let want: Vec<f64> = flat
            .iter()
            .map(|v| {
                v.as_f64()
                    .ok_or_else(|| format!("fk for {} is not numeric", target.link))
            })
            .collect::<Result<_, _>>()?;
        let m = pose.to_homogeneous();
        let got: Vec<f64> = (0..4)
            .flat_map(|r| (0..4).map(move |c| m[(r, c)]))
            .collect();
        let d = worst_abs_diff(&want, &got);
        if d.is_nan() || d > worst {
            worst = d;
        }
    }
    Ok(worst)
}

/// Everything one run accumulates, keyed by unordered pair name.
#[derive(Default)]
struct Stats {
    requested: usize,
    per_pair: std::collections::BTreeMap<String, Tally>,
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
             this robot offers no single-pair corpus, and a run that compared \
             nothing is not a pass",
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
            "  {:<40} {:<9} half [{:.4}, {:.4}, {:.4}]",
            t.link,
            t.kind.name(),
            t.half[0],
            t.half[1],
            t.half[2]
        );
    }

    let first_state = states.first().ok_or("random_states returned no state")?;
    for target in &targets {
        prove_mask_applies(&mut oracle, &model, &link_names, first_state, target)?;
    }
    println!(
        "ACM diff proven to reach the oracle's query on all {} target(s)",
        targets.len()
    );

    let mut stats = Stats::default();
    let mut worst_fk = 0.0f64;

    for (index, joint_values) in states.iter().enumerate() {
        let mut state = RobotState::new(&model);
        state.set_to_default_values();
        apply_joint_values(&mut state, joint_values)?;
        let posed = state.update();
        let link_poses: Vec<Isometry3> = targets
            .iter()
            .map(|t| {
                posed
                    .global_link_transform(&t.link)
                    .map_err(|e| format!("FK for {}: {e}", t.link))
            })
            .collect::<Result<_, _>>()?;

        // The corpus asserts a clearance; if the two sides disagreed about
        // where the link is by more than that, the finest rung would be
        // measuring FK. Checked every state, not once: a divergence that only
        // some configurations produce is exactly the one a single check misses.
        let fk = fk_divergence(&mut oracle, joint_values, &targets, &link_poses)?;
        // NaN must win both the running worst and the gate below: plain `>`
        // is always false against NaN, which would let a NaN transform
        // element read as "no divergence" on both counts.
        if fk.is_nan() || fk > worst_fk {
            worst_fk = fk;
        }
        if fk.is_nan() || fk > FK_FLOOR {
            return Err(format!(
                "state {index}: the two sides' global link transforms differ by {fk:.6e}, \
                 above the {FK_FLOOR:.0e} floor -- at that size the corpus's finest gap rung \
                 (1e{GAP_DECADE_MIN:.0}) measures forward kinematics, not either narrow phase"
            ));
        }

        for (target, link_pose) in targets.iter().zip(&link_poses) {
            let shape_pose = link_pose * target.origin;
            for probe_kind in PROBES {
                let (point, normal) = surface_point(target, &mut rng);
                let probe = probe(probe_kind, normal, &mut rng)?;

                // Log-uniform magnitude, random sign, and one rung in
                // `TANGENT_IN` at exactly zero. The magnitude is capped a
                // quarter below the smaller of the two bodies so a
                // penetrating placement still meets the face it was built
                // against rather than passing through it.
                let cap = 0.25 * probe.extent.min(target.min_half());
                let hi = cap.log10().min(GAP_DECADE_MAX);
                if hi <= GAP_DECADE_MIN {
                    continue;
                }
                let gap = if rng.random_range(0..TANGENT_IN) == 0 {
                    0.0
                } else {
                    let mag = 10f64.powf(rng.random_range(GAP_DECADE_MIN..hi));
                    if rng.random_range(0..2u8) == 0 {
                        mag
                    } else {
                        -mag
                    }
                };

                let local = Isometry3::from_parts(
                    (point + normal * (probe.extent + gap)).into(),
                    probe.rotation,
                );
                let pose = shape_pose * local;

                stats.requested += 1;
                let answer = oracle.ask(json!({
                    "op": "scene_diff_collision",
                    "joint_values": joint_values,
                    "objects": [{
                        "id": PROBE_ID,
                        "pose": row_major(&pose),
                        "shape": probe.json,
                    }],
                    "diff": mask_diff(&link_names, &target.link),
                }))?;
                let expected = answer["child"]["robot_collision"]
                    .as_bool()
                    .ok_or("child.robot_collision is not a boolean")?;
                let actual = port_collision(
                    &model,
                    &link_names,
                    joint_values,
                    &target.link,
                    &probe.shape,
                    pose,
                )?;

                let name = pair_name(target.kind, probe_kind);
                let link = target.link.clone();
                stats
                    .per_pair
                    .entry(name)
                    .or_default()
                    .record(gap, expected == actual, move || {
                        format!(
                            "{link} state {index} gap {gap:+.6e}: oracle {expected} vs port {actual}"
                        )
                    });
            }
        }
    }

    report(&stats, worst_fk)
}

/// Prints the run and returns its exit code.
fn report(stats: &Stats, worst_fk: f64) -> Result<i32, String> {
    println!();
    println!(
        "requests {}, worst |oracle - port| forward kinematics {worst_fk:.6e} (floor {FK_FLOOR:.0e})",
        stats.requested
    );
    println!(
        "gap magnitudes log-uniform over 1e{GAP_DECADE_MIN:.0}..1e{GAP_DECADE_MAX:.0}, \
         one placement in {TANGENT_IN} at exactly zero"
    );
    println!();
    println!(
        "  {:<22} {:>6} {:>6} {:>13} {:>14}  {:>7} {:>6}",
        "pair", "kept", "differ", "agree |gap|>=", "differ |gap|<=", "tangent", "differ"
    );

    let mut scored_kept = 0usize;
    let mut scored_disagrees = 0usize;
    let mut libccd_samples = 0usize;
    let mut libccd_disagrees = 0usize;
    let mut outliers: Vec<&String> = Vec::new();

    for (name, tally) in &stats.per_pair {
        let (a, b) = split_pair(name)?;
        let role = if is_scored(a, b) {
            scored_kept += tally.kept;
            scored_disagrees += tally.disagrees;
            outliers.extend(tally.outliers.iter());
            "SCORED"
        } else {
            if is_libccd_cell(a, b) {
                libccd_samples += tally.kept;
                libccd_disagrees += tally.disagrees;
            }
            "control"
        };
        println!(
            "  {:<22} {:>6} {:>6} {:>13} {:>14}  {:>7} {:>6}  {role}",
            name,
            tally.kept,
            tally.disagrees,
            magnitude(tally.agree_min),
            magnitude(tally.disagree_max),
            tally.tangent,
            tally.tangent_disagrees,
        );
    }
    for line in &outliers {
        println!("  OUTLIER {line}");
    }

    println!();
    println!(
        "scored {scored_kept} samples on sphere x {{box, cylinder}}, {scored_disagrees} disagree"
    );
    println!(
        "libccd control arms {libccd_samples} samples, {libccd_disagrees} disagree \
         (the corpus's power to separate)"
    );

    if scored_kept < MIN_SCORED {
        return Err(format!(
            "kept {scored_kept} scored samples, below the {MIN_SCORED} floor -- \
             a corpus this thin cannot carry the row"
        ));
    }
    if libccd_samples == 0 {
        return Err(
            "no control arm landed on a pair fcl leaves to libccd -- nothing in this run \
             shows the corpus can separate the two dispatch tables"
                .into(),
        );
    }
    if libccd_disagrees == 0 {
        return Err(format!(
            "{libccd_samples} samples on pairs fcl leaves to libccd and not one disagreed -- \
             the corpus has lost its power to separate, so its agreement elsewhere means nothing"
        ));
    }
    if scored_disagrees == 0 {
        println!("MET on the comparable-dispatch subset, {scored_kept} samples");
        Ok(0)
    } else {
        println!("NOT MET: {scored_disagrees} of {scored_kept} scored samples disagree");
        Ok(1)
    }
}

/// A bracket magnitude, or `--` when the bracket is empty.
///
/// Printing `0.000e0` for "this pair never disagreed" would read as a
/// disagreement at zero gap, which is the one gap class this row does not
/// score.
fn magnitude(value: f64) -> String {
    if value == 0.0 {
        "--".to_owned()
    } else {
        format!("{value:.3e}")
    }
}

/// Splits `"a x b"` back into its two kinds.
fn split_pair(name: &str) -> Result<(Kind, Kind), String> {
    let mut parts = name.split(" x ");
    let mut kind = || match parts.next() {
        Some("box") => Ok(Kind::Box),
        Some("cylinder") => Ok(Kind::Cylinder),
        Some("sphere") => Ok(Kind::Sphere),
        other => Err(format!("unknown shape kind in pair {name:?}: {other:?}")),
    };
    Ok((kind()?, kind()?))
}

fn main() {
    match run() {
        Ok(code) => std::process::exit(code),
        Err(message) => {
            eprintln!("tangency-subset: {message}");
            std::process::exit(2);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The defect: a NaN component used to be silently outranked by an
    /// ordinary `0.0` (the fold's seed) instead of winning outright, which
    /// would have let a Rust-side FK bug pass `fk_divergence`'s caller's
    /// `fk > FK_FLOOR` gate unnoticed.
    #[test]
    fn worst_abs_diff_does_not_discard_a_lone_nan_component() {
        let a = [0.0; 16];
        let mut b = [0.0; 16];
        b[9] = f64::NAN;
        assert!(
            worst_abs_diff(&a, &b).is_nan(),
            "a NaN transform element must make the whole aggregate NaN, not \
             silently read as \"every element agrees\""
        );
    }

    /// The demonstrated opposite: ordinary, non-degenerate deviations are
    /// still aggregated correctly -- the fix must not be a neutered no-op
    /// that also breaks the normal case.
    #[test]
    fn worst_abs_diff_matches_plain_max_when_nothing_is_nan() {
        let mut a = [0.0; 16];
        let mut b = [0.0; 16];
        a[3] = 1.0;
        b[3] = 1.3;
        a[11] = 2.0;
        b[11] = 1.5;
        assert_eq!(worst_abs_diff(&a, &b), 0.5);
    }
}
