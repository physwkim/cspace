// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! What this backend answers when two shapes touch with a gap of exactly
//! zero, for every shape pair this workspace can build -- and the pin that
//! makes a change to it loud.
//!
//! `exact_tangency_boundary.rs` measured one such pair (prbt's base cylinder
//! on `tools/moveit-diff`'s floor box) and concluded that upstream has no
//! convention to match, from two data points that disagreed. Two points
//! cannot say *what* the disagreement is a property of. This file is the
//! enumeration that can, and its answer is: the shape pair, via which
//! narrowphase routine the library dispatches to. Both libraries do it, in
//! opposite directions.
//!
//! # Upstream: the pair decides, and fcl's own header says which pairs
//!
//! Line numbers below are `/home/stevek/work/fcl` at `e5efcc4`
//! (`git describe --tags`: `0.7.0-17-ge5efcc4`), the readable checkout of
//! the library the oracle image links. The image itself ships `libfcl-dev
//! 0.7.0-3build2`; `gjk_solver_libccd-inl.h` and `collision_request.h` are
//! byte-identical between the two (`cmp`), and `box_box-inl.h` differs by
//! one line added above the cited region, so its six numbers are each one
//! lower there.
//!
//! `fcl::collide` is reached with `gjk_solver_type` at its default
//! `GST_LIBCCD` (`include/fcl/narrowphase/collision_request.h:102`; MoveIt
//! never sets the field -- `rg -n gjk_solver_type` over the pinned `moveit2`
//! checkout returns nothing outside comments).
//! `GJKSolver_libccd::shapeIntersect` forwards to
//! `ShapeIntersectLibccdImpl<S, Shape1, Shape2>::run`
//! (`include/fcl/narrowphase/detail/gjk_solver_libccd-inl.h:174` to the
//! generic template at `:112`), whose body calls `detail::GJKCollide` --
//! libccd MPR, which tests strict interior overlap and reports no contact
//! for a zero gap. A *specialisation* is registered for some pairs and not
//! others, and fcl draws the set itself as an ASCII table headed "Shape
//! intersect algorithms not using libccd" (`:178-201`), followed by the
//! registrations that implement it (`:245-267`:
//! `FCL_GJK_LIBCCD_SHAPE_INTERSECT(Sphere, …)`, `(Box, …)`, and the
//! `SHAPE_SHAPE` pairs). The specialised routines separate on a *strict*
//! inequality, so a zero gap falls through to contact generation --
//! `boxBox2`'s six separating-axis rejections are each `if(s2 > 0) {
//! *return_code = 0; return 0; }`
//! (`include/fcl/narrowphase/detail/primitive_shape_algorithm/box_box-inl.h`
//! `:302`, `:314`, `:326`, `:339`, `:351`, `:363`), and `s2 == 0` is not
//! `s2 > 0`.
//!
//! Measured, not inferred. A 7-kind x 7-kind x 3-offset probe compiled and run
//! inside the pinned oracle image (`libfcl-dev 0.7.0-3build2`), calling
//! `fcl::collide` directly on exactly-tangent pairs, against the specialised
//! set parsed out of the header above:
//!
//! | | box | sphere | ellipsoid | capsule | cone | cylinder | convex |
//! |---|---|---|---|---|---|---|---|
//! | **box**       | T/spec | T/spec | F/gjk | F/gjk  | F/gjk | F/gjk  | F/gjk |
//! | **sphere**    | T/spec | T/spec | F/gjk | T/spec | F/gjk | T/spec | F/gjk |
//! | **ellipsoid** | F/gjk  | F/gjk  | F/gjk | F/gjk  | F/gjk | F/gjk  | F/gjk |
//! | **capsule**   | F/gjk  | T/spec | F/gjk | F/gjk  | F/gjk | F/gjk  | F/gjk |
//! | **cone**      | F/gjk  | F/gjk  | F/gjk | F/gjk  | F/gjk | F/gjk  | F/gjk |
//! | **cylinder**  | F/gjk  | T/spec | F/gjk | F/gjk  | F/gjk | F/gjk  | F/gjk |
//! | **convex**    | F/gjk  | F/gjk  | F/gjk | F/gjk  | F/gjk | F/gjk  | F/gjk |
//!
//! 49 of 49 cells: specialised iff colliding-at-tangency, no exception. Three
//! cells break the confounds that a two-point sample leaves open. `convex x
//! convex` is a unit cube -- the same eight vertices as `box` -- and is
//! `false` where `box x box` is `true`, so it is not the geometry, not the
//! contact dimensionality and not curvature; only the C++ type differs.
//! `capsule x sphere` is `true` while `capsule x box` is `false`, so it is not
//! a property of one broken shape. `sphere x box` is `true` at a single point
//! of contact while `cylinder x box` is `false` on a whole face, so it is not
//! contact area.
//!
//! At the MoveIt level the same split survives the wrapper. 4 kinds x 4 kinds
//! x 3 offsets through `collision_detection::CollisionEnvFCL` on prbt, exact
//! tangency: `box x cylinder`, `cylinder x box` and `cylinder x cylinder` are
//! `false` and the other 13 are `true`. Those are exactly the generic-libccd
//! cells. `mesh` was not part of that 4x4 sweep, and the sentence that used to
//! stand here in its place -- "`mesh` is `true` against everything because
//! MoveIt maps `shapes::MESH` to `fcl::BVHModel`
//! (`collision_common.cpp:900-923`), a third traversal that is neither
//! specialisation nor libccd MPR" -- was an inference, not a measurement, and
//! it is wrong: `fcl::BVHModel` is only the broad-phase. Its leaf test against
//! a candidate triangle still calls `nsolver->shapeTriangleIntersect(shape,
//! ...)` (`mesh_shape_collision_traversal_node-inl.h:85,98,117,226,239,258`;
//! `shape_mesh_collision_traversal_node-inl.h:89,102,121`, both symmetric),
//! and that function has closed-form specialisations for exactly `Sphere<S>`
//! (`gjk_solver_libccd-inl.h:403-419`, `:480-498`), `Halfspace<S>`
//! (`:502-520`) and `Plane<S>` (`:524-542`) -- the complete list, confirmed by
//! grepping every `ShapeTriangleIntersectLibccdImpl<S, _>`/
//! `ShapeTransformedTriangleIntersectLibccdImpl<S, _>` specialisation in the
//! file, four total. `Box<S>`, `Cylinder<S>`, `Cone<S>` have none, so they
//! fall to the same generic template (`:349-383`, `:423-458`) every non-mesh
//! generic pair uses, which calls `detail::GJKCollide<S>` -- libccd MPR, whose
//! `discoverPortal` (`/home/stevek/work/libccd` v2.1/`7931e76`,
//! `src/mpr.c:189,209,232`) rejects an exact-zero support-direction dot as
//! outside the portal, and `ccdMPRIntersect`/`ccdMPRPenetration` (`:99-115`,
//! `:117-148`) both read that rejection as no collision -- the same mechanism
//! as the `box x cylinder` cell already measured above, not a fourth kind
//! exempt from it. Measured (not inferred): `mesh` is `true` at exact
//! tangency only against `sphere` (`sphere_triangle-inl.h:146,156`'s
//! `radius_with_threshold = radius + epsilon` pads the boundary inclusive,
//! the same admitting convention as the specialised cells above), and `false`
//! against `box`, `cylinder`, `cone`.
//!
//! # This backend: the pair decides here too, and one cell disagrees
//!
//! `parry3d-f64` 0.30.0 has the same structure.
//! `DefaultQueryDispatcher::contact` routes `Ball`/`Ball` to
//! `contact_ball_ball` (`parry3d-f64-0.30.0/src/query/default_query_dispatcher.rs:316`), which
//! admits a pair on `if distance_squared < sum_radius_with_error *
//! sum_radius_with_error` (`parry3d-f64-0.30.0/src/query/contact/contact_ball_ball.rs:16`) -- a
//! *strict* `<`, so two spheres at a gap of exactly zero give `1.0 < 1.0`,
//! `None`, no collision. Its generic support-map path is the other way round:
//! `gjk::closest_points` rejects on `if min_bound > max_dist`
//! (`parry3d-f64-0.30.0/src/query/gjk/gjk.rs:411`), also strict, so a distance of exactly
//! `max_dist` *is* admitted. One library's specialisation is inclusive at the boundary
//! and its generic path exclusive; the other's is exclusive and its generic
//! path inclusive. Neither states a convention, and the sign of the
//! disagreement is an accident of which side of the comparison the boundary
//! landed on.
//!
//! # What this backend picks today
//!
//! `accumulate_collision` now ports fcl's own dispatch table
//! (`crates/moveit-collision/src/fcl_tangency_table.rs`, generated from the
//! oracle image's `gjk_solver_libccd-inl.h` by
//! `tools/ci/verify-fcl-tangency-dispatch.sh --emit`) and consults it,
//! per shape pair, whenever `query::contact`'s `dist` lands at exactly
//! `0.0` -- mesh decided separately as an unconditional `true`. That choice
//! was reasoned from the same wrong premise the corrected section above used
//! to state ("`BVHModel` is a third dispatch, exempt from libccd MPR"): it
//! is not, for three of the four non-mesh-mesh pairs. `mesh x sphere`'s
//! `true` here matches upstream's own closed form; `mesh x box`, `mesh x
//! cylinder` and `mesh x cone`'s `true` here do not -- upstream measures
//! `false` for those at exact tangency, by the same generic-libccd mechanism
//! as `box x cylinder` above. This is a known, currently undecided
//! divergence from upstream on those six cells (both argument orders),
//! carried by `is_mesh_pair`'s unconditional `true`; it is not fixed by this
//! file. Off that exact tie, the pre-existing rule is unchanged: any `Some` is a
//! touch. `sphere x sphere`'s own tie is a `None` from `query::contact`
//! (`contact_ball_ball`'s strict `<`) that `query::intersection_test`
//! (`intersection_test_ball_ball`'s `<=`) still confirms is touching; see
//! `parry.rs`'s `parry_boolean_queries_disagree_in_both_directions_at_the_tie`
//! for why that recovery is gated to table-true, non-`Conditional` pairs
//! rather than a blanket substitution.
//!
//! Measured before `touches_at_tie` existed, and still true of the raw
//! `query::contact` output it now sits in front of: `box x cylinder`/
//! `cylinder x box` and `box x cone`/`cone x box` measure a tiny nonzero
//! `dist` at this exact construction (`3.661568089044567e-33` for `box x
//! cylinder`, `-1.1102230246251565e-16` for `cylinder x box`,
//! `7.850462293418875e-17` for `box x cone`, `-1.1102230246251565e-16` for
//! `cone x box` -- opposite signs for the same pair in opposite argument
//! order), not binary `0.0`, even though every input length here is exactly
//! representable in binary. That is GJK's own rounding during iteration, not
//! the geometry, and it is the same phenomenon `accumulate_collision`'s own
//! comment already documents for prbt's cylinder-on-box tie
//! (`-2.775558e-17`) -- this file independently confirms it is not
//! particular to that one fixture, nor to one shape-kind pair, nor even to
//! these four: `parry.rs`'s `TIE_ROUNDING_MARGIN` doc has the full sweep,
//! including a `cylinder x cylinder` tie that rounds nonzero too.
//!
//! `accumulate_collision`/`accumulate_distance` no longer key this decision
//! on the literal bit pattern `dist == 0.0` -- `touches_at_tie`
//! (`crates/moveit-collision/src/parry.rs`) treats any `dist` within
//! `TIE_ROUNDING_MARGIN * f64::EPSILON * scale` of zero as this same
//! rounding, not a real answer, and consults the dispatch table there
//! instead of trusting the sign. All 25 cells below are decided by the
//! table now, matching upstream exactly for the 20 non-mesh cells and for
//! `mesh x mesh`: the naive "restrict fcl's table to this crate's five
//! kinds" prediction was correct there, and the rounding above no longer
//! keeps four pairs from reaching it. The remaining 6 cells -- `mesh x
//! {box, cylinder, cone}`, both orders -- do not match upstream (see above);
//! this file pins the port's own current, undecided answer for them, not
//! upstream's.
//!
//! # Cost
//!
//! 4 tests, 81 `check_robot_collision` calls, no oracle and no docker.
//! `cargo nextest run -p moveit-collision -E
//! 'binary(exact_tangency_is_decided_per_shape_pair)'` summarises at
//! `0.008s`, so this needs no opt-in gate.

use std::collections::BTreeSet;
use std::fs;
use std::sync::Arc;

use moveit_collision::{
    AllowedCollisionMatrix, AttachedBodyGeometry, CollisionEnv, CollisionRequest, LinkPaddingScale,
    ParryCollisionEnv, World,
};
use moveit_geometry::{Cone, Cuboid, Cylinder, Isometry3, Mesh, Shape, Sphere, Vector3};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;

/// Every kind whose `moveit_geometry::Shape` variant this backend can convert
/// to a `parry` shape *and* place at an exact tangency. `Plane` and
/// `Halfspace` are unbounded and `OcTree` is a compound of cuboids already
/// covered by `Box`, so the constructible-and-tangent set is these five.
const KINDS: [Kind; 5] = [
    Kind::Box,
    Kind::Sphere,
    Kind::Cylinder,
    Kind::Cone,
    Kind::Mesh,
];

/// The pinned answer at a gap of exactly zero, rows indexed by the upper
/// (attached) shape and columns by the lower (world) shape, both in [`KINDS`]
/// order. Measured, not predicted, but now equal to fcl's table transcribed
/// (`fcl_tangency_table::SPECIALISED`, extended with mesh's own unconditional
/// `true` row/column): `accumulate_collision`/`accumulate_distance` no longer
/// key the dispatch-table lookup on the literal bit pattern `dist == 0.0`.
/// Four of the twenty-five pairs built here -- `box x cylinder`, `cylinder x
/// box`, `box x cone`, `cone x box` -- land at a nonzero-but-tiny `dist`
/// instead of exact `0.0` (measured `3.661568089044567e-33` for `box x
/// cylinder`, `-1.1102230246251565e-16` for `cylinder x box`,
/// `7.850462293418875e-17` for `box x cone`, `-1.1102230246251565e-16` for
/// `cone x box` -- opposite signs for the same pair in opposite argument
/// order, GJK's own rounding rather than anything about the input geometry,
/// which is exactly representable in binary on both sides), but
/// `touches_at_tie`'s rounding band (`parry.rs`, `TIE_ROUNDING_MARGIN`)
/// reaches all four, so every cell here is decided by the table now.
const TANGENT: [[bool; 5]; 5] = [
    //        box   sphere  cyl    cone   mesh
    [true, true, false, false, true],   // box
    [true, true, true, false, true],    // sphere
    [false, true, false, false, true],  // cylinder
    [false, false, false, false, true], // cone
    [true, true, true, true, true],     // mesh
];

/// The pinned answer across a `1e-9` gap of clear air: `false` for all 25,
/// matching upstream exactly. It was not always -- `box x cylinder`,
/// `cylinder x box` and `cone x box` used to read `true` here, this
/// backend's positive margin (`exact_tangency_boundary.rs`,
/// `the_collision_boundary_sits_in_a_positive_gap`) reaching a `Some`
/// `query::contact` result that the pre-`touches_at_tie` code took as an
/// unconditional touch. `touches_at_tie` still lets that `Some` through
/// (`1e-9` is many orders of magnitude past `TIE_ROUNDING_MARGIN * EPS *
/// scale`, so `dist`'s own sign decides, not the table), but the measured
/// `dist` at a real `1e-9` gap is itself positive for all three, so the sign
/// now agrees with upstream instead of being discarded.
const CLEARANCE: [[bool; 5]; 5] = [[false; 5]; 5];

/// Half the extent of every shape below, along every axis. Exactly
/// representable in binary, so `LOWER_CENTRE_Z + HALF` and `HALF` are exact
/// and the gap at `delta == 0.0` is exactly zero rather than nearly zero.
const HALF: f64 = 0.5;

/// The lower (world) shape's centre. `x = 5.0` keeps it clear of prbt's own
/// links, which [`only_the_pair_under_test_is_in_contact`] checks rather than
/// assumes.
const LOWER_CENTRE: (f64, f64, f64) = (5.0, 0.0, -HALF);

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Kind {
    Box,
    Sphere,
    Cylinder,
    Cone,
    Mesh,
}

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

    /// A shape of this kind whose extent is exactly `HALF` from its own
    /// origin along every axis, so two of them stacked `2 * HALF` apart touch
    /// with a gap of exactly zero.
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
            // Built through the public `Mesh::new`, which leaves
            // `vertex_normals: None` -- the state an attached body's mesh
            // actually arrives in on this side.
            Self::Mesh => Shape::Mesh(unit_cube_mesh()),
        })
    }
}

/// A cube spanning `[-HALF, HALF]` on every axis, as 8 vertices and 12
/// triangles.
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

fn build_prbt() -> RobotModel {
    let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/prbt.urdf");
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/prbt.srdf");
    let urdf_xml = fs::read_to_string(urdf_path).expect("fixture URDF must be readable");
    let urdf = urdf_rs::read_file(urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())
        .expect("fixture model must build")
}

fn build_acm() -> AllowedCollisionMatrix {
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/prbt.srdf");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    AllowedCollisionMatrix::from_srdf(&srdf)
}

/// The whole 5x5 table at one vertical offset. `delta` is the signed gap:
/// negative overlaps, zero is the exact tie, positive is clear air.
fn sweep(delta: f64) -> [[bool; 5]; 5] {
    let model = build_prbt();
    let acm = build_acm();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();
    let touch_links = BTreeSet::new();

    let mut table = [[false; 5]; 5];
    for (row, upper) in KINDS.iter().enumerate() {
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
        for (col, lower) in KINDS.iter().enumerate() {
            let mut world = World::new();
            world.add_shape(
                "lower",
                lower.shape(),
                Isometry3::translation(LOWER_CENTRE.0, LOWER_CENTRE.1, LOWER_CENTRE.2),
            );
            let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
            table[row][col] = env
                .check_robot_collision(
                    &CollisionRequest::default(),
                    &posed,
                    std::slice::from_ref(&attached),
                    Some(&acm),
                )
                .collision;
        }
    }
    table
}

fn report(measured: &[[bool; 5]; 5], expected: &[[bool; 5]; 5], what: &str) {
    let mut wrong = Vec::new();
    for (row, upper) in KINDS.iter().enumerate() {
        for (col, lower) in KINDS.iter().enumerate() {
            if measured[row][col] != expected[row][col] {
                wrong.push(format!(
                    "{} x {}: expected {}, got {}",
                    upper.name(),
                    lower.name(),
                    expected[row][col],
                    measured[row][col]
                ));
            }
        }
    }
    assert!(
        wrong.is_empty(),
        "{what} changed in {} of 25 cells:\n  {}",
        wrong.len(),
        wrong.join("\n  ")
    );
}

/// The fixture the other three rest on: `prbt_base_link` is at the world
/// origin at the default state, so an attached shape's `shape_pose` *is* its
/// world pose and the offsets below mean what they say. If the link ever
/// moves, every table would still compare cleanly while measuring some other
/// separation.
#[test]
fn the_attached_frame_is_the_world_frame() {
    let model = build_prbt();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();
    let pose = posed
        .global_link_transform("prbt_base_link")
        .expect("prbt_base_link must resolve");
    assert_eq!(
        pose.translation.vector,
        Vector3::zeros(),
        "prbt_base_link must sit at the world origin"
    );
    assert_eq!(
        pose.rotation,
        Isometry3::identity().rotation,
        "prbt_base_link must be unrotated"
    );
}

/// The control. With no attached body the lower object touches nothing, so
/// every `true` in the tables below is the pair under test and not prbt's own
/// geometry reaching 5 m sideways.
#[test]
fn only_the_pair_under_test_is_in_contact() {
    let model = build_prbt();
    let acm = build_acm();
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();
    for kind in KINDS {
        let mut world = World::new();
        world.add_shape(
            "lower",
            kind.shape(),
            Isometry3::translation(LOWER_CENTRE.0, LOWER_CENTRE.1, LOWER_CENTRE.2),
        );
        let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
        let hit = env
            .check_robot_collision(&CollisionRequest::default(), &posed, &[], Some(&acm))
            .collision;
        assert!(!hit, "the lower {} must not touch prbt itself", kind.name());
    }
}

/// The subject. A gap of exactly zero is decided per shape pair -- by
/// `fcl_tangency_table` where `dist` lands at exactly `0.0`, unconditionally
/// for `box x cylinder`/`cylinder x box` and `box x cone`/`cone x box` where
/// it never does (see the module doc). A `parry` upgrade that changes a
/// comparison or a dispatch for any pair, or an `accumulate_collision`
/// change to the table lookup itself, reddens this and names the cell.
#[test]
fn exact_tangency_is_decided_by_fcls_dispatch_table() {
    report(&sweep(0.0), &TANGENT, "the exact-tangency table");
}

/// The two controls that bracket it: a nanometre of overlap collides
/// everywhere, and a nanometre of clearance collides only where this
/// backend's positive margin reaches -- which is itself per-pair, and
/// asymmetric in the pair's order.
#[test]
fn a_nanometre_either_side_of_the_tie_brackets_it() {
    report(&sweep(-1e-9), &[[true; 5]; 5], "the overlap table");
    report(&sweep(1e-9), &CLEARANCE, "the clearance table");
}
