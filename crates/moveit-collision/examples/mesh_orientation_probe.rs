// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No upstream file: this is a port-side self-consistency probe, not a
// parity measurement against fcl.

//! Probe: does `moveit_collision`'s production `check_robot_collision` path
//! ever report `false` for a mesh pair that is, by construction, exactly
//! tangent -- at orientations other than the single axis-aligned offset
//! `exact_tangency_is_decided_per_shape_pair.rs` measures?
//!
//! # Why this question was still open
//!
//! `exact_tangency_is_decided_per_shape_pair.rs`'s `TANGENT` table measures
//! one geometry per mesh cell: an axis-aligned unit cube, offset along a
//! single world axis from an axis-aligned box/sphere/cylinder/cone/mesh.
//! `residual-triage`'s enumeration of `accumulate_collision`'s rescue branch
//! (`crates/moveit-collision/src/parry.rs:2429`) found it structurally inert
//! for `TriMesh`: `fcl_tangency_verdict` (`parry.rs:2151-2158`) returns
//! `None` whenever either shape is `HalfSpace`, `TriMesh` or `Compound`, so
//! the branch's `== Some(true)` guard can never pass for a mesh pair,
//! regardless of shape or orientation -- there is no `is_mesh_pair`-style
//! explicit exclusion, the equality already excludes it by construction.
//! That makes the mesh row/column of `TANGENT` a direct read of
//! `query::contact`'s own raw output with no rescue underneath it, and
//! `triage-2429-enumeration.md` states the open question outright: whether
//! `query::contact` can miss for a mesh pair at some other orientation, "not
//! benign" but unmeasured.
//!
//! # Method
//!
//! `evidence-retention-1e69c0a3-1`'s `tools/mpr-cone-orientation` (sibling
//! branch, read for method only, not merged here) answers the equivalent
//! question against fcl's own generic MPR by constructing exact tangency at
//! an arbitrary tilt from the touching shape's own support function, then
//! confirming the construction numerically rather than trusting the algebra
//! alone. This probe applies the same method to this port's own mesh
//! fixture and its own `query::contact` dispatch:
//!
//! - [`cube_geometry`] is bit-for-bit
//!   `exact_tangency_is_decided_per_shape_pair.rs::unit_cube_mesh` -- same 8
//!   vertices, same 12 triangles, same indices.
//! - For a rotation `rot`, [`extremal_vertex`] finds the rotated cube's own
//!   support point along +/-z: since the cube is a convex polytope, the
//!   vertex with the extreme `z` after rotation *is* the support function's
//!   exact value in that direction, not a sampled approximation. Every other
//!   rotated vertex is provably on the far side of the plane through that
//!   point, no matter how the cube is turned.
//! - [`rotated_mesh_pose`] translates the mesh so that vertex lands exactly
//!   on [`TOUCH`]; [`fixed_pose`] places the other (unrotated) shape's own
//!   touching feature -- a face centre, a pole, a disc centre, all of which
//!   coincide with `TOUCH` for a `HALF`-extent shape centred `HALF` away --
//!   on the same point from the other side. The two shapes are tangent by
//!   construction for any `rot`.
//! - [`sanity_check`] confirms each `delta == 0.0` case that comes back
//!   `false` is not an artefact of this construction: if pushing the same
//!   pose towards more overlap never produces a collision either, the pose
//!   is not a genuine near-tie (floating-point rounding of `rot * vertex`
//!   the likely cause, or a footprint/curvature mismatch this construction
//!   did not anticipate) and is excluded from the miss count, reported
//!   separately instead of silently folded into "no miss found".
//!
//! Two argument orders run for every rotation and every other kind: mesh as
//! the attached/upper body against a fixed world/lower shape
//! ([`Role::Upper`]), and mesh as the world/lower body against a fixed
//! attached/upper shape ([`Role::Lower`]) -- this port has already measured
//! order-sensitivity in fcl's own generic MPR for `mesh x cone`
//! (`exact_tangency_is_decided_per_shape_pair.rs`'s module doc), so this
//! probe does not assume this backend's own dispatch is order-symmetric
//! without checking both orders itself.
//!
//! Every check goes through the same public `ParryCollisionEnv::
//! check_robot_collision` path `exact_tangency_is_decided_per_shape_pair.rs`
//! uses, not a hand-rolled call into `query::contact` -- this measures the
//! shape actually shipped, and since the rescue branch is proven inert for
//! every mesh pair, a `false` here is `query::contact` missing with nothing
//! underneath it, by the same reasoning that makes the axis-aligned
//! `TANGENT` cells a direct read of it.
//!
//! Every *systematic* (not random) config also prints a `CSV,...` line --
//! `other,role,axis=..,angle=..deg,collision` -- true or false, not just the
//! misses: this is the row-per-config grid an fcl-side probe over the same
//! 497 orientations needs to join against, to answer whether this backend's
//! own tilt-dependent misses are a divergence from fcl or a case where fcl
//! is itself unstable under tilt (`c04d5640`, sibling branch
//! `evidence-retention-1e69c0a3-1`, already measured that instability for
//! fcl's own generic MPR at cone and cylinder).
//!
//! # Coverage and result
//!
//! 497 systematic rotations (7 axes: `x`, `y`, `z`, the three face
//! diagonals, the body diagonal; 5-degree resolution from 5 to 355 degrees)
//! plus 2,000 uniformly-random rotations (Shoemake's method, `ChaCha8Rng`
//! seeded `0x5EED_C0DE`, fixed for reproducibility) -- 2,497 orientations x
//! 5 other kinds x 2 argument orders = 24,970 configurations, each an exact
//! zero-gap tie by construction.
//!
//! Found, not "not found": 6,083 of 24,970 (24.4%) report `false` at
//! `delta == 0.0`. All 6,083 are genuine near-ties, not construction
//! failures -- [`sanity_check`] never excluded one. Every single one flips
//! to `true` with between `1e-16` and `1e-14` of added overlap
//! ([`MISS_DEPTH_PROBES`]'s own resolution below `1e-16`, `1e-17`, never
//! flipped any of them, so the true depth for most is somewhere in
//! `(1e-17, 1e-16]` and a minority in `(1e-15, 1e-14]`), the same order of
//! magnitude `exact_tangency_is_decided_per_shape_pair.rs`'s own module doc
//! already measures as GJK's floating-point rounding at an axis-aligned tie.
//! By other kind: `box` 6, `cylinder` 13, `cone` 1521, `mesh` 1949, `sphere`
//! 2594. By role: 4221 with the mesh as attached/upper, 1862 as world/lower
//! -- both orders miss, so this is not resolved by picking one argument
//! order over the other.
//!
//! Combined with the rescue branch's structural inertness for `TriMesh`
//! (this file's own module doc, above): this is an unrescued false negative
//! reaching `check_robot_collision`'s boolean result for roughly a quarter
//! of the orientations swept here, at the same floating-point-rounding
//! magnitude every non-mesh pair already gets forgiven for. The smallest
//! reproducer found (`box`, mesh attached, 5 degrees about `z`) is pinned as
//! a regression fixture in
//! `tests/mesh_orientation_tangency_can_miss.rs`. Whether to extend
//! `accumulate_collision`'s rescue to cover mesh pairs is not decided here --
//! that requires an fcl target to converge to, not just a `false` this side.
//!
//! `tools/fcl-mesh-orientation-probe` answers that for the 497 systematic
//! (non-random) orientations above: fcl has no single stable answer for most
//! `mesh x cone` tilts (82.1% of the 497 poses), a partial answer for `mesh x
//! mesh`, and a clean, orientation-independent `true` for every `mesh x
//! sphere` tilt this port misses (29.2% of the 497, zero exceptions) -- see
//! that tool's own README for the full per-kind pose-level table.
//!
//! # Resolution -- partial
//!
//! `mesh x sphere` had an fcl target to converge to and nothing else did, so
//! it is the one pair with an identified fix: `crate::mesh_tangency_table`
//! replaces the deleted `is_mesh_pair`'s single blanket boolean with a
//! measured, per-paired-kind verdict (`MeshVerdict::AlwaysTouching` for
//! `Sphere` alone; `NoStableTarget` for `cone`, measured unresolvable;
//! `Undiagnosed` for `box`/`cylinder`/`mesh`, measured but not safely
//! reducible to one verdict per pair -- `crate::mesh_tangency_table`'s own
//! module doc has the full accounting), and `fcl_tangency_verdict` now
//! dispatches mesh pairs to it instead of returning `None` unconditionally.
//!
//! That alone does not close the 145 `mesh x sphere` misses. "The branch's
//! `== Some(true)` guard can never pass for a mesh pair, regardless of shape
//! or orientation" (above) is no longer true -- the guard passes for
//! `mesh x sphere` now -- but the branch's own confirmation call,
//! `query::intersection_test`, was measured on 10 sampled miss poses to
//! answer `false` at every one: the same near-degenerate rounding this probe
//! measures, one geometric query deeper (`Ball`-vs-`Triangle`'s
//! `PointQuery::project_local_point`, not the GJK `contact` path
//! `touches_at_tie` already knows how to round through). A widened-
//! prediction second `query::contact` call finds `Some` at all 10 of the
//! same poses instead, which is the fix `accumulate_collision`'s branch body
//! still needs -- outside `crate::mesh_tangency_table`'s own confinement, not
//! made here. `MeshVerdict::AlwaysTouching`'s own doc has the measurement.

use std::collections::BTreeSet;
use std::sync::Arc;

use moveit_collision::{
    AllowedCollisionMatrix, AttachedBodyGeometry, CollisionEnv, CollisionRequest, LinkPaddingScale,
    ParryCollisionEnv, World,
};
use moveit_geometry::{Cone, Cuboid, Cylinder, Isometry3, Mesh, Shape, UnitQuaternion, Vector3};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::{Posed, RobotState};
use nalgebra::{Quaternion, Translation3, Unit};
use rand::{RngExt, SeedableRng};
use rand_chacha::ChaCha8Rng;

/// Half the extent of every shape below, along every axis -- matches
/// `exact_tangency_is_decided_per_shape_pair.rs::HALF`.
const HALF: f64 = 0.5;

/// The single point every fixed shape's own touching face/pole/disc centre
/// sits at, and every rotated mesh's own extremal support vertex is
/// translated onto. `(5.0, 0.0, 0.0)` for the same reason
/// `exact_tangency_is_decided_per_shape_pair.rs::LOWER_CENTRE` uses `x =
/// 5.0`: clear of prbt's own links.
const TOUCH: (f64, f64, f64) = (5.0, 0.0, 0.0);

/// Fixed seed: this is a probe meant to be re-run and checked, not a
/// statistical sample that should vary between runs.
const RANDOM_SEED: u64 = 0x5EED_C0DE;
const RANDOM_ROTATIONS: usize = 2000;
const SYSTEMATIC_STEP_DEGREES: i32 = 5;

/// Offsets tried, smallest magnitude first, once a `delta == 0.0` case comes
/// back `false` and passes [`sanity_check`] -- the smallest one that flips
/// the result to `true` is reported as this pose's own miss depth.
const MISS_DEPTH_PROBES: [f64; 11] = [
    -1e-17, -1e-16, -1e-15, -1e-14, -1e-13, -1e-12, -1e-9, -1e-6, -1e-3, -1e-1, -1.0,
];

/// Every kind whose `moveit_geometry::Shape` variant this backend can convert
/// to a `parry` shape *and* place at an exact tangency -- same set as
/// `exact_tangency_is_decided_per_shape_pair.rs::KINDS`, mesh included: the
/// mesh-vs-mesh cell keeps the *other* mesh unrotated, so it is still one
/// rotated party against one fixed party.
const OTHER_KINDS: [Kind; 5] = [
    Kind::Box,
    Kind::Sphere,
    Kind::Cylinder,
    Kind::Cone,
    Kind::Mesh,
];

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Kind {
    Box,
    Sphere,
    Cylinder,
    Cone,
    Mesh,
}

impl Kind {
    fn shape(self) -> Arc<Shape> {
        Arc::new(match self {
            Self::Box => Shape::Cuboid(
                Cuboid::new(2.0 * HALF, 2.0 * HALF, 2.0 * HALF).expect("positive cuboid"),
            ),
            Self::Sphere => {
                Shape::Sphere(moveit_geometry::Sphere::new(HALF).expect("positive sphere"))
            }
            Self::Cylinder => {
                Shape::Cylinder(Cylinder::new(HALF, 2.0 * HALF).expect("positive cylinder"))
            }
            Self::Cone => Shape::Cone(Cone::new(HALF, 2.0 * HALF).expect("positive cone")),
            Self::Mesh => Shape::Mesh(unit_cube_mesh()),
        })
    }
}

/// Bit-for-bit `exact_tangency_is_decided_per_shape_pair.rs::unit_cube_mesh`.
fn unit_cube_mesh() -> Mesh {
    let (vertices, triangles) = cube_geometry();
    Mesh::new(vertices, triangles).expect("cube mesh indices are in range")
}

/// A cube spanning `[-HALF, HALF]` on every axis: 8 vertices, vertex `i` has
/// bit 0 = +x, bit 1 = +y, bit 2 = +z, and 12 triangles -- the same fixture
/// `exact_tangency_is_decided_per_shape_pair.rs` builds, split out here
/// because [`extremal_vertex`] needs the raw vertex list, not just the
/// assembled `Mesh`.
fn cube_geometry() -> (Vec<Vector3>, Vec<[u32; 3]>) {
    let mut vertices = Vec::with_capacity(8);
    for &z in &[-HALF, HALF] {
        for &y in &[-HALF, HALF] {
            for &x in &[-HALF, HALF] {
                vertices.push(Vector3::new(x, y, z));
            }
        }
    }
    let triangles = vec![
        [0u32, 2, 1],
        [1, 2, 3],
        [4, 5, 6],
        [5, 7, 6],
        [0, 1, 4],
        [1, 5, 4],
        [2, 6, 3],
        [3, 6, 7],
        [0, 4, 2],
        [2, 4, 6],
        [1, 3, 5],
        [3, 7, 5],
    ];
    (vertices, triangles)
}

/// The rotated cube's own support vertex along +/-z: the vertex (of a known
/// convex polytope, so this is the support function's exact value, not a
/// sample) with the lowest rotated `z` (`want_min == true`) or the highest
/// (`want_min == false`).
fn extremal_vertex(rot: UnitQuaternion, want_min: bool) -> Vector3 {
    let (vertices, _) = cube_geometry();
    let mut best = rot * vertices[0];
    for &v in &vertices[1..] {
        let r = rot * v;
        if (want_min && r.z < best.z) || (!want_min && r.z > best.z) {
            best = r;
        }
    }
    best
}

#[derive(Clone, Copy)]
enum Role {
    /// The rotated mesh is the attached/upper body; the other kind is the
    /// unrotated world/lower shape.
    Upper,
    /// The rotated mesh is the world/lower body; the other kind is the
    /// unrotated attached/upper shape.
    Lower,
}

/// The rotated mesh's own pose: translated so its own extremal rotated
/// vertex (lowest for [`Role::Upper`], highest for [`Role::Lower`]) sits
/// exactly on [`TOUCH`], offset by `delta` along the touch normal. Negative
/// `delta` moves the mesh toward more overlap, positive toward more
/// clearance -- the same sign convention
/// `exact_tangency_is_decided_per_shape_pair.rs::sweep`'s `delta` uses.
fn rotated_mesh_pose(rot: UnitQuaternion, role: Role, delta: f64) -> Isometry3 {
    let want_min = matches!(role, Role::Upper);
    let v = extremal_vertex(rot, want_min);
    let sign = if want_min { 1.0 } else { -1.0 };
    let translation = Translation3::new(TOUCH.0 - v.x, TOUCH.1 - v.y, TOUCH.2 - v.z + sign * delta);
    Isometry3::from_parts(translation, rot)
}

/// The fixed (unrotated) shape's own pose: its own touching feature -- a
/// `HALF`-extent shape's top face centre, pole or disc centre, all of which
/// sit exactly `HALF` from its own origin along z -- lands on [`TOUCH`] from
/// the side opposite the rotated mesh.
fn fixed_pose(role: Role) -> Isometry3 {
    let z = match role {
        Role::Upper => TOUCH.2 - HALF,
        Role::Lower => TOUCH.2 + HALF,
    };
    Isometry3::translation(TOUCH.0, TOUCH.1, z)
}

/// Runs one configuration through the full production path and returns
/// whether `check_robot_collision` reports a collision.
fn check(
    posed: &Posed<'_, '_>,
    acm: &AllowedCollisionMatrix,
    other: Kind,
    role: Role,
    mesh_pose: Isometry3,
) -> bool {
    let touch_links = BTreeSet::new();
    let (upper_shape, upper_pose, lower_shape, lower_pose) = match role {
        Role::Upper => (
            Kind::Mesh.shape(),
            mesh_pose,
            other.shape(),
            fixed_pose(role),
        ),
        Role::Lower => (
            other.shape(),
            fixed_pose(role),
            Kind::Mesh.shape(),
            mesh_pose,
        ),
    };
    let upper_shapes = [upper_shape];
    let upper_poses = [upper_pose];
    let attached = AttachedBodyGeometry {
        id: "upper",
        link_name: "prbt_base_link",
        shapes: &upper_shapes,
        shape_poses: &upper_poses,
        touch_links: &touch_links,
    };
    let mut world = World::new();
    world.add_shape("lower", lower_shape, lower_pose);
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
    env.check_robot_collision(
        &CollisionRequest::default(),
        posed,
        std::slice::from_ref(&attached),
        Some(acm),
    )
    .collision
}

/// For a `delta == 0.0` case that came back `false`: the smallest-magnitude
/// entry in [`MISS_DEPTH_PROBES`] that flips the result to `true`, or `None`
/// if even `-1.0` (half the shapes' own extent) does not -- meaning this
/// pose is not a genuine near-tie at all (a construction failure, not a
/// dispatch miss) and should be excluded from the reported count.
fn sanity_check(
    posed: &Posed<'_, '_>,
    acm: &AllowedCollisionMatrix,
    other: Kind,
    role: Role,
    rot: UnitQuaternion,
) -> Option<f64> {
    for &delta in &MISS_DEPTH_PROBES {
        let pose = rotated_mesh_pose(rot, role, delta);
        if check(posed, acm, other, role, pose) {
            return Some(delta);
        }
    }
    None
}

fn build_prbt() -> RobotModel {
    let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/prbt.urdf");
    let urdf_xml = std::fs::read_to_string(urdf_path).expect("fixture URDF must be readable");
    let urdf = urdf_rs::read_file(urdf_path).expect("fixture URDF must parse");
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/prbt.srdf");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

fn build_acm() -> AllowedCollisionMatrix {
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/prbt.srdf");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    AllowedCollisionMatrix::from_srdf(&srdf)
}

/// 7 axes (3 cardinal, 3 face-diagonal, 1 body-diagonal of the cube) at
/// `SYSTEMATIC_STEP_DEGREES` resolution from that step to 360 minus that
/// step -- 0 and 360 are the already-measured axis-aligned case.
fn systematic_rotations() -> Vec<(String, UnitQuaternion)> {
    let axes: [(&str, Vector3); 7] = [
        ("x", Vector3::new(1.0, 0.0, 0.0)),
        ("y", Vector3::new(0.0, 1.0, 0.0)),
        ("z", Vector3::new(0.0, 0.0, 1.0)),
        ("xy", Vector3::new(1.0, 1.0, 0.0)),
        ("xz", Vector3::new(1.0, 0.0, 1.0)),
        ("yz", Vector3::new(0.0, 1.0, 1.0)),
        ("xyz", Vector3::new(1.0, 1.0, 1.0)),
    ];
    let mut out = Vec::new();
    let mut deg = SYSTEMATIC_STEP_DEGREES;
    while deg < 360 {
        for (name, axis) in &axes {
            let unit_axis = Unit::new_normalize(*axis);
            let rot = UnitQuaternion::from_axis_angle(&unit_axis, (deg as f64).to_radians());
            out.push((format!("axis={name},angle={deg}deg"), rot));
        }
        deg += SYSTEMATIC_STEP_DEGREES;
    }
    out
}

/// A uniformly-random unit quaternion via Shoemake's method (three uniform
/// `[0,1)` draws, no Gaussian sampling needed).
fn random_rotation(rng: &mut ChaCha8Rng) -> UnitQuaternion {
    let u1: f64 = rng.random_range(0.0..1.0);
    let u2: f64 = rng.random_range(0.0..std::f64::consts::TAU);
    let u3: f64 = rng.random_range(0.0..std::f64::consts::TAU);
    let a = (1.0 - u1).sqrt();
    let b = u1.sqrt();
    UnitQuaternion::new_normalize(Quaternion::new(
        b * u3.cos(),
        a * u2.sin(),
        a * u2.cos(),
        b * u3.sin(),
    ))
}

fn random_rotations(count: usize, seed: u64) -> Vec<(String, UnitQuaternion)> {
    let mut rng = ChaCha8Rng::seed_from_u64(seed);
    (0..count)
        .map(|i| (format!("random#{i}"), random_rotation(&mut rng)))
        .collect()
}

struct Miss {
    label: String,
    other: Kind,
    role: &'static str,
    depth: Option<f64>,
}

fn main() {
    let model = build_prbt();
    let acm = build_acm();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();

    let mut rotations = systematic_rotations();
    let systematic_count = rotations.len();
    rotations.extend(random_rotations(RANDOM_ROTATIONS, RANDOM_SEED));
    let total_rotations = rotations.len();

    let roles = [
        (Role::Upper, "mesh=upper/attached"),
        (Role::Lower, "mesh=lower/world"),
    ];

    let mut configs = 0usize;
    let mut misses: Vec<Miss> = Vec::new();
    let mut excluded_bad_construction: Vec<String> = Vec::new();

    for other in OTHER_KINDS {
        for (role, role_name) in roles {
            for (label, rot) in &rotations {
                configs += 1;
                let pose = rotated_mesh_pose(*rot, role, 0.0);
                let collided = check(&posed, &acm, other, role, pose);
                // One row per *systematic* config (not random -- see
                // `main`'s own doc), true or false: the raw grid this port
                // measured, joinable against an fcl-side probe run over the
                // identical 497 orientations to build a confusion matrix
                // rather than re-deriving it from the miss list alone.
                if label.starts_with("axis=") {
                    println!("CSV,{other:?},{role_name},{label},{collided}");
                }
                if collided {
                    continue;
                }
                match sanity_check(&posed, &acm, other, role, *rot) {
                    Some(depth) => misses.push(Miss {
                        label: format!("{other:?} {role_name} {label}"),
                        other,
                        role: role_name,
                        depth: Some(depth),
                    }),
                    None => {
                        excluded_bad_construction.push(format!("{other:?} {role_name} {label}"))
                    }
                }
            }
        }
    }

    println!(
        "mesh_orientation_probe: {total_rotations} rotations ({systematic_count} systematic \
         at {SYSTEMATIC_STEP_DEGREES} deg resolution over 7 axes, {RANDOM_ROTATIONS} random \
         seed={RANDOM_SEED:#x}) x {} other kinds x 2 argument orders = {configs} configurations \
         checked through check_robot_collision at delta=0.0",
        OTHER_KINDS.len()
    );
    println!(
        "excluded as not a genuine near-tie (construction failure, not a dispatch question): {}",
        excluded_bad_construction.len()
    );
    for e in &excluded_bad_construction {
        println!("  excluded: {e}");
    }

    if misses.is_empty() {
        println!(
            "RESULT: no miss found in {configs} configurations sweeping {total_rotations} \
             orientations x {} other kinds x 2 argument orders. Not found within this measured \
             range -- not proven absent beyond it.",
            OTHER_KINDS.len()
        );
        std::process::exit(0);
    }

    println!("RESULT: {} miss(es) found:", misses.len());
    for m in &misses {
        match m.depth {
            Some(depth) => println!(
                "  MISS {} ({:?}/{}): query::contact-backed result is false at delta=0.0, \
                 flips true at delta={depth:e}",
                m.label, m.other, m.role
            ),
            None => println!("  {} (unreachable: depth already reported)", m.label),
        }
    }
    std::process::exit(1);
}
