// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The assertions of upstream's `test_collision_common_panda.hpp`, restated
//! against this backend.
//!
//! `doc/port-coverage.md` classifies that header `decided-non-port`: it is a
//! GoogleTest `TYPED_TEST_P` fixture whose whole reason to be a shared header
//! is the `CollisionAllocatorType` type parameter, and this port has one
//! collision backend for that parameter to range over. What the header
//! *asserts* is a different thing from the machinery it asserts it with, and
//! the assertions are what this file keeps. Every configuration below --
//! joint values, box sizes, box poses, padding values, expected magnitudes
//! and their tolerances -- is upstream's, read from that header at
//! `e017c91ee12984393a28ba246075c65f69cde3bf`, with the upstream test name
//! named on each case.
//!
//! # Why these are not oracle-backed
//!
//! `tools/moveit-oracle`'s `collision` op constructs `CollisionEnvFCL(model,
//! world)` and never calls `setLinkPadding`, so [`padding_test`]'s
//! configuration cannot be posed to it at all. The rest could be, but there
//! would be little point: upstream's own constants *are* the expectation
//! here, and re-deriving them from the oracle would replace a fixed target
//! with a moving one. `collision_parity.rs` is where oracle agreement is
//! measured; this file is where upstream's hand-written expectations are.
//!
//! Two of them are magnitudes rather than booleans (`DistanceSelf`'s `0.022`
//! and `DistanceWorld`'s `0.029`), and upstream states its own tolerance for
//! each (`0.001` and `0.01`). Those tolerances are kept verbatim rather than
//! re-measured: widening one would only hide the divergence it exists to
//! catch, and this backend's measured values are recorded on each test so a
//! future drift is visible as a number, not just as a pass.
//!
//! # What is here, against the header's ten cases
//!
//! The header registers ten tests across three suites
//! (`REGISTER_TYPED_TEST_SUITE_P`, `:378-383`). Nine are below, one by one:
//!
//! | upstream | here |
//! |---|---|
//! | `InitOK` (`:109-112`) | not a case: it asserts the fixture's own `robot_model_ok_`, which is [`build_panda`]'s `.expect` |
//! | `DefaultNotInCollision` (`:115-121`) | [`default_not_in_collision`] |
//! | `LinksInCollision` (`:124-137`) | [`links_in_collision`] |
//! | `RobotWorldCollision_1` (`:140-180`) | [`robot_world_collision_1`] |
//! | `RobotWorldCollision_2` (`:183-202`) | [`robot_world_collision_2`] |
//! | `PaddingTest` (`:205-234`) | [`padding_test`] |
//! | `DistanceSelf` (`:237-245`) | [`distance_self`] |
//! | `DistanceWorld` (`:247-267`) | [`distance_world`] |
//! | `DistanceSingle` (`:276-322`) | [`distance_single`] |
//! | `DistancePoints` (`:325-376`) | [`distance_points`] |
//!
//! # Two defects this found, both fixed at source
//!
//! Neither was reachable from the tests this workspace already had, which is
//! the argument for restating a header's assertions rather than reasoning
//! that they must be covered:
//!
//! - `CollisionRequest::distance` populated nothing. Upstream's two collision
//!   helpers each end with an `if (req.distance)` block
//!   (`collision_env_fcl.cpp:283-297`, `:340-354`); the parry backend ran
//!   neither, so [`distance_self`] and [`distance_world`] had no field to
//!   read. Fixed by `parry.rs`'s `attach_requested_distance`.
//! - Both per-pair maps were keyed in iteration order rather than by sorted
//!   name (`collision_common.cpp:240-242`, `:564-567`), so [`distance_single`]
//!   looked up `("collection", "panda_hand")` in a map that held
//!   `("panda_hand", "collection")`. Fixed by `parry.rs`'s `pair_key`.

use std::fs;
use std::sync::Arc;

use moveit_collision::{
    AllowedCollisionMatrix, CollisionEnv, CollisionRequest, DistanceRequest, DistanceRequestType,
    LinkPaddingScale, ParryCollisionEnv, World,
};
use moveit_geometry::{Cuboid, Cylinder, Isometry3, Shape};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;

fn fixture_mesh_search_paths() -> MeshSearchPaths {
    let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
    MeshSearchPaths::new([(
        "moveit_resources_panda_description",
        format!("{meshes_root}/panda_description"),
    )])
}

fn build_panda() -> RobotModel {
    let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.urdf");
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.srdf");
    let urdf_xml = fs::read_to_string(urdf_path).expect("fixture URDF must be readable");
    let urdf = urdf_rs::read_file(urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &fixture_mesh_search_paths())
        .expect("fixture model must build")
}

fn build_acm() -> AllowedCollisionMatrix {
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.srdf");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    AllowedCollisionMatrix::from_srdf(&srdf)
}

/// `setToHome` (`test_collision_common_panda.hpp:56-68`): the default state
/// with four joints overridden. Every case below starts here, exactly as
/// upstream's `SetUp` does.
fn state_at_home(model: &RobotModel) -> RobotState<'_> {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    for (joint, value) in [
        ("panda_joint2", -0.785),
        ("panda_joint4", -2.356),
        ("panda_joint6", 1.571),
        ("panda_joint7", 0.785),
    ] {
        state
            .set_joint_positions(joint, &[value])
            .expect("panda's revolute joints take one position each");
    }
    state
}

/// A world holding one axis-aligned box of the given full extents, centred at
/// `(x, y, z)` -- upstream's `shapes::Box(a, b, c)` is full extents, and its
/// `addToObject(id, pose, shape, Identity())` puts the object pose on the
/// object and the identity on the shape.
fn world_with_box(id: &str, extents: [f64; 3], centre: [f64; 3]) -> World {
    let mut world = World::new();
    world.add_shape(
        id,
        Arc::new(Shape::Cuboid(
            Cuboid::new(extents[0], extents[1], extents[2]).expect("positive cuboid dimensions"),
        )),
        Isometry3::translation(centre[0], centre[1], centre[2]),
    );
    world
}

/// `DefaultNotInCollision` (`:115-121`): the SRDF-derived home position is
/// self-collision free.
#[test]
fn default_not_in_collision() {
    let model = build_panda();
    let acm = build_acm();
    let env = ParryCollisionEnv::new(World::new(), LinkPaddingScale::default());
    let mut state = state_at_home(&model);
    let posed = state.update();

    let result = env.check_self_collision(&CollisionRequest::default(), &posed, &[], Some(&acm));
    assert!(
        !result.collision,
        "panda's home position must be self-collision free"
    );
}

/// `LinksInCollision` (`:124-137`): `panda_joint2 = 0.15`, `panda_joint4 =
/// -3.0` folds the arm into itself.
///
/// The companion to [`default_not_in_collision`]: on its own, a
/// self-collision check that answered `false` unconditionally would pass that
/// test, so the pair is what makes either mean anything.
#[test]
fn links_in_collision() {
    let model = build_panda();
    let acm = build_acm();
    let env = ParryCollisionEnv::new(World::new(), LinkPaddingScale::default());
    let mut state = state_at_home(&model);
    state
        .set_joint_positions("panda_joint2", &[0.15])
        .expect("panda_joint2 takes one position");
    state
        .set_joint_positions("panda_joint4", &[-3.0])
        .expect("panda_joint4 takes one position");
    let posed = state.update();

    let result = env.check_self_collision(&CollisionRequest::default(), &posed, &[], Some(&acm));
    assert!(
        result.collision,
        "panda_joint2=0.15 / panda_joint4=-3.0 must self-collide"
    );
}

/// `RobotWorldCollision_1` (`:140-180`): a `0.1 m` box moved up the `z` axis
/// past the arm, in and out of collision.
///
/// The first check is a *self*-collision check with the box present, and it
/// must stay `false`: a world object must not leak into the self-collision
/// pair set.
///
/// Upstream's `moveObject(id, transform)` composes -- `setObjectPose(id,
/// transform * pose_)` at `world.cpp:291` -- so its four `z` literals
/// `0.3, 0.25, 0.05, 0.25` put the box at `0.3, 0.55, 0.60, 0.85`, not at the
/// literals. That is transcribed here as the same *relative* moves rather
/// than as the absolute heights, so the composing semantics
/// ([`moveit_collision::World::move_object`], `world.rs:593`) is part of what
/// this case checks rather than something the test quietly assumes.
#[test]
fn robot_world_collision_1() {
    let model = build_panda();
    let acm = build_acm();
    let mut state = state_at_home(&model);
    let posed = state.update();
    let request = CollisionRequest::default();

    let mut env = ParryCollisionEnv::new(
        world_with_box("box", [0.1, 0.1, 0.1], [0.0, 0.0, 0.3]),
        LinkPaddingScale::default(),
    );
    assert!(
        !env.check_self_collision(&request, &posed, &[], Some(&acm))
            .collision,
        "a world box must not appear in the self-collision pair set"
    );
    assert!(
        env.check_robot_collision(&request, &posed, &[], Some(&acm))
            .collision,
        "the box at z=0.3 must collide with the robot"
    );

    // (relative move, resulting absolute height, expected collision)
    for (step, height, expected) in [(0.25, 0.55, false), (0.05, 0.60, true), (0.25, 0.85, false)] {
        env.world_mut()
            .move_object("box", Isometry3::translation(0.0, 0.0, step));
        let collision = env
            .check_robot_collision(&request, &posed, &[], Some(&acm))
            .collision;
        assert_eq!(
            collision, expected,
            "after moving the 0.1 box up by {step} (to z={height}) it must report \
             collision={expected}"
        );
    }
}

/// `RobotWorldCollision_2` (`:183-202`): a `0.4 m` box at `z = 0.3` collides,
/// and reports at least three contacts.
///
/// Upstream sets `max_contacts = 10` and `contacts = true`; the `>= 3` is its
/// own bound, kept as-is. This backend reports the count recorded in the
/// assertion message below, so the margin over upstream's floor is visible
/// rather than implied.
#[test]
fn robot_world_collision_2() {
    let model = build_panda();
    let acm = build_acm();
    let env = ParryCollisionEnv::new(
        world_with_box("box", [0.4, 0.4, 0.4], [0.0, 0.0, 0.3]),
        LinkPaddingScale::default(),
    );
    let mut state = state_at_home(&model);
    let posed = state.update();

    let request = CollisionRequest {
        contacts: true,
        max_contacts: 10,
        verbose: true,
        ..CollisionRequest::default()
    };
    let result = env.check_robot_collision(&request, &posed, &[], Some(&acm));
    assert!(result.collision, "the 0.4 box at z=0.3 must collide");
    let count = result.contacts.as_ref().map_or(0, |c| c.count());
    assert!(
        count >= 3,
        "upstream asserts at least 3 contacts for this configuration, got {count}"
    );
}

/// `PaddingTest` (`:205-234`): a box that clears the hand collides once
/// `panda_hand` is padded by `0.08 m`, and clears again at padding `0.0`.
///
/// This is the only end-to-end check in this workspace that link padding
/// reaches a collision answer at all. `parry.rs`'s `scaled_padded_shape` is
/// unit-tested on geometry and is called from the two build sites that pose
/// robot links, but nothing else asserts that the value set through
/// `set_link_padding` changes what `check_robot_collision` returns -- so this
/// case is not redundant with anything in `env.rs`'s padding tests, all of
/// which stop at the padding map.
#[test]
fn padding_test() {
    let model = build_panda();
    let acm = build_acm();
    let mut state = state_at_home(&model);
    let posed = state.update();
    let request = CollisionRequest {
        contacts: true,
        max_contacts: 10,
        ..CollisionRequest::default()
    };

    // Upstream checks the empty world first; a box added later is what makes
    // the padding step mean something.
    let empty = ParryCollisionEnv::new(World::new(), LinkPaddingScale::default());
    assert!(
        !empty
            .check_robot_collision(&request, &posed, &[], Some(&acm))
            .collision,
        "the home position must be world-collision free before the box is added"
    );

    let box_world = || world_with_box("box", [0.1, 0.1, 0.1], [0.43, 0.0, 0.55]);

    let mut padded = LinkPaddingScale::default();
    padded.set_link_padding("panda_hand", 0.08);
    let env = ParryCollisionEnv::new(box_world(), padded);
    assert!(
        env.check_robot_collision(&request, &posed, &[], Some(&acm))
            .collision,
        "padding panda_hand by 0.08 must bring the box into collision"
    );

    let mut unpadded = LinkPaddingScale::default();
    unpadded.set_link_padding("panda_hand", 0.0);
    let env = ParryCollisionEnv::new(box_world(), unpadded);
    assert!(
        !env.check_robot_collision(&request, &posed, &[], Some(&acm))
            .collision,
        "returning panda_hand's padding to 0.0 must clear the collision again"
    );
}

/// `DistanceSelf` (`:237-245`): the home position's closest self-pair is
/// `0.022 m` apart, to `0.001`.
///
/// Upstream's tolerance, not one measured here. This backend's value is in
/// the assertion message so a drift inside the tolerance is still legible.
#[test]
fn distance_self() {
    let model = build_panda();
    let acm = build_acm();
    let env = ParryCollisionEnv::new(World::new(), LinkPaddingScale::default());
    let mut state = state_at_home(&model);
    let posed = state.update();

    let request = CollisionRequest {
        distance: true,
        ..CollisionRequest::default()
    };
    let result = env.check_self_collision(&request, &posed, &[], Some(&acm));
    assert!(
        !result.collision,
        "the home position is self-collision free"
    );
    let distance = result
        .distance
        .as_ref()
        .expect("`distance: true` must populate the distance field")
        .distance();
    assert!(
        (distance - 0.022).abs() < 0.001,
        "upstream expects 0.022 +/- 0.001 for the home position's closest self-pair, got {distance}"
    );
}

/// `DistanceWorld` (`:247-267`): with the box in front of the hand and no
/// padding, the closest robot-world pair is `0.029 m` apart, to `0.01`.
#[test]
fn distance_world() {
    let model = build_panda();
    let acm = build_acm();
    let mut unpadded = LinkPaddingScale::default();
    unpadded.set_link_padding("panda_hand", 0.0);
    let env = ParryCollisionEnv::new(
        world_with_box("box", [0.1, 0.1, 0.1], [0.43, 0.0, 0.55]),
        unpadded,
    );
    let mut state = state_at_home(&model);
    let posed = state.update();

    let request = CollisionRequest {
        distance: true,
        ..CollisionRequest::default()
    };
    let result = env.check_robot_collision(&request, &posed, &[], Some(&acm));
    assert!(!result.collision, "the unpadded hand clears the box");
    let distance = result
        .distance
        .as_ref()
        .expect("`distance: true` must populate the distance field")
        .distance();
    assert!(
        (distance - 0.029).abs() < 0.01,
        "upstream expects 0.029 +/- 0.01 for the gap to the box, got {distance}"
    );
}

/// `DistanceSingle` (`:276-322`): a `Single` distance query against an object
/// holding many shapes reports the minimum over those shapes.
///
/// Upstream accumulates ten cylinders into one object `collection` while
/// re-creating a single-shape object `object` at the same place each round,
/// and asserts that `collection`'s reported distance equals the running
/// minimum of `object`'s. Each round also poses the two identically by
/// *different decompositions* -- `collection` gets the identity object pose
/// and the pose on the shape, `object` gets the pose on the object and the
/// identity on the shape -- so the equality is also the check that the two
/// compose to the same global placement.
///
/// # Two deliberate departures, both stated rather than hidden
///
/// Upstream's poses come from `random_numbers::RandomNumberGenerator
/// rng(0x47110815)`. That generator is not ported and its exact stream is not
/// what the assertion is about -- the invariant holds for any sequence -- so
/// the ten poses are a fixed table here. They are chosen inside upstream's
/// own sampling box (`x, y` in `[0.1, 2.0]`, `z` in `[1.2, 1.7]`, radius and
/// length in `(0, 1)`) and, unlike a random draw, are ordered so the running
/// minimum actually moves: the test asserts that it moves, since a sequence
/// whose minimum is set at round 0 and never beaten would satisfy the
/// equality while checking nothing.
///
/// Upstream also sets `active_components_only = {panda_hand}`. That field is
/// not ported (`common.rs:327-332`: it needs a `RobotModel`, out of scope for
/// this crate), and it is a restriction on which pairs are *computed*, not on
/// what is asserted -- both assertions read one named pair out of the map, and
/// those two entries are present either way.
#[test]
fn distance_single() {
    // (radius, length, x, y, z), inside upstream's sampling box. Ordered so
    // rounds 0, 2 and 5 each set a new minimum.
    const ROUNDS: [(f64, f64, f64, f64, f64); 10] = [
        (0.10, 0.20, 1.60, 0.90, 1.60),
        (0.15, 0.30, 1.80, 1.40, 1.70),
        (0.20, 0.40, 0.90, 0.40, 1.35),
        (0.12, 0.25, 1.95, 1.90, 1.65),
        (0.18, 0.35, 1.70, 1.60, 1.55),
        (0.30, 0.60, 0.30, 0.15, 1.25),
        (0.10, 0.50, 1.20, 1.10, 1.45),
        (0.25, 0.20, 1.50, 0.60, 1.70),
        (0.14, 0.45, 1.85, 0.30, 1.30),
        (0.22, 0.15, 0.60, 1.75, 1.20),
    ];

    let model = build_panda();
    let mut env = ParryCollisionEnv::new(World::new(), LinkPaddingScale::default());
    let mut state = state_at_home(&model);
    let posed = state.update();

    let request = DistanceRequest {
        request_type: DistanceRequestType::Single,
        enable_signed_distance: true,
        ..DistanceRequest::default()
    };

    let mut running_min = f64::MAX;
    let mut times_the_minimum_moved = 0_usize;

    for (round, &(radius, length, x, y, z)) in ROUNDS.iter().enumerate() {
        let shape = Arc::new(Shape::Cylinder(
            Cylinder::new(radius, length).expect("positive cylinder dimensions"),
        ));
        let pose = Isometry3::translation(x, y, z);

        // Same global placement, opposite decomposition.
        env.world_mut().add_shape_to_object(
            "collection",
            Isometry3::identity(),
            shape.clone(),
            pose,
        );
        env.world_mut().remove_object("object");
        env.world_mut()
            .add_shape_to_object("object", pose, shape, Isometry3::identity());

        let result = env.distance_robot(&request, &posed, &[]);
        let single = |object: &str| {
            let key = (object.to_string(), "panda_hand".to_string());
            let data = result.distances.get(&key).unwrap_or_else(|| {
                // The keys are printed rather than just named: this map is
                // keyed by a *pair*, so "not found" is ambiguous between "the
                // pair was never computed" and "it is filed under the other
                // ordering", and those want opposite fixes.
                let keys: Vec<_> = result.distances.keys().collect();
                panic!(
                    "round {round}: no distance reported for {object}/panda_hand; keys are {keys:?}"
                )
            });
            assert_eq!(
                data.len(),
                1,
                "round {round}: a Single request must report one datum for {object}/panda_hand"
            );
            data[0].distance
        };

        let individual = single("object");
        if individual < running_min {
            if running_min != f64::MAX {
                times_the_minimum_moved += 1;
            }
            running_min = individual;
        }
        let collection = single("collection");
        assert!(
            (collection - running_min).abs() < 1e-5,
            "round {round}: the distance to the accumulated object is {collection}, not the \
             running minimum {running_min} of the individually placed ones"
        );
    }

    assert!(
        times_the_minimum_moved >= 2,
        "the pose table must beat its own first minimum at least twice or the equality above is \
         satisfied trivially, but it moved {times_the_minimum_moved} times"
    );
}

/// `DistancePoints` (`:325-376`): every nearest point reported *on* the box
/// lies inside the box.
///
/// Upstream's own check is `|p_local| <= size + eps` per axis, comparing the
/// box-local coordinate against the **full** extent rather than the half
/// extent -- a bound twice as loose as the box. That is kept, because
/// tightening it would be inventing an assertion upstream does not make; the
/// half-extent bound is asserted separately below so the loose one cannot be
/// the only thing standing.
#[test]
fn distance_points() {
    const EXTENT: f64 = 0.1;
    const CENTRE: [f64; 3] = [0.43, 0.0, 0.55];
    const EPS: f64 = 1e-5;

    let model = build_panda();
    let acm = build_acm();
    let env = ParryCollisionEnv::new(
        world_with_box("box", [EXTENT, EXTENT, EXTENT], CENTRE),
        LinkPaddingScale::default(),
    );
    let mut state = state_at_home(&model);
    let posed = state.update();

    let request = DistanceRequest {
        acm: Some(&acm),
        request_type: DistanceRequestType::Single,
        enable_nearest_points: true,
        max_contacts_per_body: 1,
        ..DistanceRequest::default()
    };
    let result = env.distance_robot(&request, &posed, &[]);

    let mut checked = 0_usize;
    for ((first, second), data) in &result.distances {
        for datum in data {
            let point = if datum.link_names[0] == "box" {
                datum.nearest_points[0]
            } else if datum.link_names[1] == "box" {
                datum.nearest_points[1]
            } else {
                panic!("unrecognized link names {first}/{second}");
            };
            for axis in 0..3 {
                let local = (point[axis] - CENTRE[axis]).abs();
                assert!(
                    local <= EXTENT + EPS,
                    "nearest point {point:?} on the box is outside it on axis {axis}: {local}"
                );
                // Upstream compares against the full extent; the box only
                // reaches half of it. Asserted here so the pair of bounds is
                // explicit rather than the loose one silently standing alone.
                assert!(
                    local <= EXTENT / 2.0 + EPS,
                    "nearest point {point:?} is outside the box's actual half extent on axis \
                     {axis}: {local}"
                );
            }
            checked += 1;
        }
    }
    assert!(
        checked > 0,
        "no nearest point was reported on the box, so this test asserted nothing"
    );
}
