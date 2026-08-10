// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! This port's counterpart to upstream's `CollisionDetectorPandaTest.PaddingTest`
//! and `.DistanceWorld`
//! (`moveit_core/collision_detection/include/moveit/collision_detection/test_collision_common_panda.hpp:205-267`,
//! moveit2 @ `e017c91e`).
//!
//! Those two upstream cases are the only assertions in either
//! `test_collision_common_{panda,pr2}.hpp` that `crates/cspace-collision`'s
//! oracle-parity suite cannot reach, and the reason is structural rather than
//! an oversight: the oracle's `collision` op takes no padding argument
//! (`oracle.cpp`'s `json collision(const json&)` reads `joint_values`,
//! `attached_bodies`, world objects and `max_contacts_per_pair`, and nothing
//! else), so every case in `tests/fixtures/{panda,fanuc,pr2}_collision.json`
//! was captured at the `CollisionEnv` constructor default of padding `0.0`
//! and scale `1.0`. `parry.rs`'s module doc reaches the same conclusion from
//! the other side ("both backends apply exactly zero padding and unit scale
//! to every pr2 link on that op's path"). A differential fixture therefore
//! cannot exercise [`LinkPaddingScale`] at all, and before this file the
//! workspace had no test that put a non-zero padding in front of a collision
//! query: every `set_link_padding`/`LinkPaddingScale::with_links` call site
//! lived in `env.rs`'s own unit tests, which assert the map's bookkeeping
//! (clamping, change reporting, untracked-link defaults) and never a verdict.
//!
//! So this is ground truth of the third kind this crate uses -- not the C++
//! oracle's recorded answer, and not a hand-picked constant, but upstream's
//! own published scenario replayed with its own numbers: the same `0.1`-cube
//! obstacle at the same `(0.43, 0, 0.55)`, the same `panda_hand` padded by
//! the same `0.08`, from the same home pose `setToHome` defines.
//!
//! # The measured numbers, and why the flip is not an accident of the value
//!
//! Swept on this backend at the pose below (`padding -> min robot distance`,
//! nearest pair in parentheses):
//!
//! ```text
//! 0.00  ->  +0.029119199  (panda_link7 / box)
//! 0.02  ->  +0.021490125  (panda_hand  / box)
//! 0.04  ->  +0.001589386  (panda_hand  / box)
//! 0.06  ->  -0.018311353  (panda_hand  / box)
//! 0.08  ->  -0.038212093  (panda_hand  / box)
//! ```
//!
//! Two things fall out of that sweep, and the test asserts both rather than
//! just the endpoint upstream happens to name.
//!
//! The unpadded figure, `+0.029119199` against `panda_link7`, is upstream's
//! `DistanceWorld` constant: that case asserts `EXPECT_NEAR(res.distance,
//! 0.029, 0.01)` for exactly this scene. This backend lands `0.000119` from
//! upstream's nominal `0.029`, i.e. inside a tolerance ~84x tighter than the
//! `0.01` upstream itself allows -- so the scene is reproduced, not merely
//! similar.
//!
//! The verdict flips between `0.04` and `0.06`, because `panda_hand`'s own
//! unpadded clearance is `~0.0416`. Upstream's `0.08` is therefore not a
//! knife-edge: it clears the flip point by roughly `2x`, which is what makes
//! it a stable regression target rather than a tolerance to be tuned.
//!
//! # What each assertion discriminates
//!
//! A test that only checked "collision is true at `0.08`" would still pass if
//! padding had gone wrong in ways that matter, so three properties are
//! asserted separately:
//!
//! - the nearest pair at `0.08` is `panda_hand`, the link that was padded --
//!   without this, padding some *other* link, or growing the world box, would
//!   satisfy the verdict just as well;
//! - the depth at `0.08` tracks the padding rather than merely being
//!   negative -- a mesh grown by `0.08` along its vertex normals must lose
//!   about `0.08` of clearance, and asserting the magnitude rejects a
//!   "padding makes everything collide" failure;
//! - restoring padding to `0.0` restores the *exact* original distance --
//!   this is what proves padding is applied per query from
//!   [`LinkPaddingScale`] rather than baked irreversibly into the link
//!   geometry at construction, which is the property upstream's third step
//!   (`setLinkPadding("panda_hand", 0.0)` then re-check) exists to pin.
//!
//! # The self half
//!
//! Upstream's `PaddingTest` pads a link and then checks
//! `checkRobotCollision` only, so a backend that applied
//! [`LinkPaddingScale`] to the robot-vs-world query and silently skipped it
//! on the self query would satisfy every line above. `CollisionEnvFCL` does
//! not have that freedom -- both queries read the one padded `robot_geoms_`
//! its constructor built -- and neither does this backend, but nothing
//! asserted it. The second test below is that boundary's other side, with
//! the same three-step shape and its own discriminator: padding a
//! *different* link by the same `0.05` leaves the scene clear, so the
//! verdict is attributable to the link that was padded rather than to
//! padding at all.
//!
//! (Which environment a *`PlanningScene`* self-check runs against is a
//! separate question, and this port answers it differently from upstream --
//! see `PlanningScene`'s type doc in `cspace_planning::scene`. This file is about the
//! backend, where upstream pads both queries too.)

use std::sync::Arc;

use cspace_collision::{
    AllowedCollisionMatrix, CollisionEnv, CollisionRequest, DistanceRequest, LinkPaddingScale,
    ParryCollisionEnv, World,
};
use cspace_core::geometry::{Cuboid, Isometry3, Shape};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;

const PANDA_URDF: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.urdf");
const PANDA_SRDF: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.srdf");

/// Upstream's `setToHome` (`test_collision_common_panda.hpp:56-68`): the SRDF
/// defaults with four joints overridden.
const HOME: [(&str, f64); 4] = [
    ("panda_joint2", -0.785),
    ("panda_joint4", -2.356),
    ("panda_joint6", 1.571),
    ("panda_joint7", 0.785),
];

/// Upstream pads `panda_hand` by this much (`test_collision_common_panda.hpp:227`).
const PADDING: f64 = 0.08;

/// `panda_link7` vs. the box with nothing padded. Upstream's `DistanceWorld`
/// asserts `EXPECT_NEAR(res.distance, 0.029, 0.01)` for this scene; this is
/// the same quantity measured here, kept at full precision so the
/// restore-to-zero step can compare against it exactly.
const UNPADDED_DISTANCE: f64 = 0.029_119_199;

/// `panda_hand`'s own clearance to the box before padding, read off the sweep
/// in the module doc (`0.02 -> +0.021490125`, so `0.021490125 + 0.02`). Only
/// used to state what the depth at [`PADDING`] should be.
const HAND_UNPADDED_CLEARANCE: f64 = 0.041_490_125;

/// The home pose's closest self-pair, `panda_link5` vs. `panda_link7`, with
/// nothing padded. Upstream's `DistanceSelf`
/// (`test_collision_common_panda.hpp:236-244`) asserts `EXPECT_NEAR(
/// res.distance, 0.022, 0.001)` for exactly this quantity; this backend
/// lands `0.000135` from that nominal, so the self scene is reproduced the
/// same way [`UNPADDED_DISTANCE`] reproduces the world one.
const SELF_UNPADDED_DISTANCE: f64 = 0.022_134_891;

/// Padding applied to one link for the self-collision half. Larger than
/// [`PADDING`] because a self-pair grows on one side only here: padding
/// `panda_link5` alone shrinks the pair's clearance from `0.0221` to
/// `-0.0102`, so `0.05` clears the flip point (measured between `0.0223`
/// and `0.03`) by roughly the same `2x` margin upstream's `0.08` leaves in
/// the world half.
const SELF_PADDING: f64 = 0.05;

/// Distances here are separations between rigid bodies at a fixed pose, so
/// the only spread to absorb is the mesh-padding arithmetic itself: growing a
/// mesh along its vertex normals is not exactly a uniform surface offset, and
/// over the sweep above `panda_hand`'s implied unpadded clearance drifts from
/// `0.041490` at padding `0.02` to `0.041788` at `0.08` -- about `3e-4`.
/// `1e-3` covers that with room, and is still `80x` tighter than the
/// `0.08` effect being measured.
const TOL: f64 = 1e-3;

fn panda() -> RobotModel {
    let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
    let search_paths = MeshSearchPaths::new([(
        "moveit_resources_panda_description",
        format!("{meshes_root}/panda_description"),
    )]);
    let urdf_xml = std::fs::read_to_string(PANDA_URDF).expect("fixture URDF must be readable");
    let urdf = urdf_rs::read_file(PANDA_URDF).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(PANDA_SRDF).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &search_paths)
        .expect("fixture model must build")
}

/// The `0.1`-cube obstacle upstream places right in front of the hand
/// (`test_collision_common_panda.hpp:216-224`).
fn box_in_front_of_the_hand() -> World {
    let mut world = World::new();
    world.add_shape(
        "box",
        Arc::new(Shape::Cuboid(
            Cuboid::new(0.1, 0.1, 0.1).expect("0.1 is a valid, positive cuboid dimension"),
        )),
        Isometry3::translation(0.43, 0.0, 0.55),
    );
    world
}

#[test]
fn padding_panda_hand_turns_a_clear_scene_into_a_collision_and_back() {
    let model = panda();
    let acm = AllowedCollisionMatrix::from_srdf(
        &SrdfModel::parse_file(PANDA_SRDF).expect("fixture SRDF must parse"),
    );

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    for (name, value) in HOME {
        state
            .set_variable_position(name, value)
            .unwrap_or_else(|e| panic!("setting {name}: {e}"));
    }
    let posed = state.update();

    let mut env = ParryCollisionEnv::new(box_in_front_of_the_hand(), LinkPaddingScale::default());

    let distance_request = || DistanceRequest {
        enable_signed_distance: true,
        acm: Some(&acm),
        ..DistanceRequest::default()
    };
    let collides = |env: &ParryCollisionEnv| {
        env.check_robot_collision(&CollisionRequest::default(), &posed, &[], Some(&acm))
            .collision
    };

    // Step 1 -- upstream's `ASSERT_FALSE(res.collision)` before any padding,
    // plus its `DistanceWorld` constant.
    assert!(
        !collides(&env),
        "unpadded panda_hand must clear the box at the home pose"
    );
    let unpadded = env.distance_robot(&distance_request(), &posed, &[]);
    assert!(
        (unpadded.minimum_distance.distance - UNPADDED_DISTANCE).abs() < TOL,
        "unpadded minimum robot distance {} != {UNPADDED_DISTANCE}",
        unpadded.minimum_distance.distance
    );

    // Step 2 -- upstream's `setLinkPadding("panda_hand", 0.08)` then
    // `ASSERT_TRUE(res.collision)`.
    assert!(
        env.padding_scale_mut()
            .set_link_padding("panda_hand", PADDING),
        "setting panda_hand's padding to {PADDING} must report a change"
    );
    assert!(
        collides(&env),
        "panda_hand padded by {PADDING} must reach the box"
    );

    let padded = env.distance_robot(&distance_request(), &posed, &[]);
    assert!(
        padded
            .minimum_distance
            .link_names
            .iter()
            .any(|name| name == "panda_hand"),
        "the padded link must be the one now nearest the box, got {:?}",
        padded.minimum_distance.link_names
    );
    let expected_depth = HAND_UNPADDED_CLEARANCE - PADDING;
    assert!(
        (padded.minimum_distance.distance - expected_depth).abs() < TOL,
        "padded depth {} does not track the padding (expected ~{expected_depth})",
        padded.minimum_distance.distance
    );

    // Step 3 -- upstream's `setLinkPadding("panda_hand", 0.0)` then
    // `ASSERT_FALSE(res.collision)`. Asserting the distance too, not just the
    // verdict, is what makes this a statement about reversibility rather than
    // about the sign.
    assert!(
        env.padding_scale_mut().set_link_padding("panda_hand", 0.0),
        "restoring panda_hand's padding to 0.0 must report a change"
    );
    assert!(
        !collides(&env),
        "restoring padding to 0.0 must restore the clear verdict"
    );
    let restored = env.distance_robot(&distance_request(), &posed, &[]);
    assert_eq!(
        restored.minimum_distance.distance, unpadded.minimum_distance.distance,
        "restoring padding to 0.0 must restore the exact unpadded distance, \
         not merely a clear verdict"
    );
}

#[test]
fn padding_panda_link5_turns_a_clear_self_scene_into_a_collision_and_back() {
    let model = panda();
    let acm = AllowedCollisionMatrix::from_srdf(
        &SrdfModel::parse_file(PANDA_SRDF).expect("fixture SRDF must parse"),
    );

    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    for (name, value) in HOME {
        state
            .set_variable_position(name, value)
            .unwrap_or_else(|e| panic!("setting {name}: {e}"));
    }
    let posed = state.update();

    // No world at all, so nothing but the self query can produce a verdict.
    let mut env = ParryCollisionEnv::new(World::new(), LinkPaddingScale::default());

    let distance_request = || DistanceRequest {
        enable_signed_distance: true,
        acm: Some(&acm),
        ..DistanceRequest::default()
    };
    let self_collides = |env: &ParryCollisionEnv| {
        env.check_self_collision(&CollisionRequest::default(), &posed, &[], Some(&acm))
            .collision
    };

    // Step 1 -- the home pose is self-collision free, at upstream's own
    // `DistanceSelf` separation.
    assert!(
        !self_collides(&env),
        "the home pose must be self-collision free before anything is padded"
    );
    let unpadded = env.distance_self(&distance_request(), &posed, &[]);
    assert!(
        (unpadded.minimum_distance.distance - SELF_UNPADDED_DISTANCE).abs() < TOL,
        "unpadded minimum self distance {} != {SELF_UNPADDED_DISTANCE}",
        unpadded.minimum_distance.distance
    );

    // Step 2 -- padding one side of that pair closes it. `check_self_collision`
    // reading `LinkPaddingScale` at all is what this asserts.
    assert!(
        env.padding_scale_mut()
            .set_link_padding("panda_link5", SELF_PADDING),
        "setting panda_link5's padding to {SELF_PADDING} must report a change"
    );
    assert!(
        self_collides(&env),
        "panda_link5 padded by {SELF_PADDING} must reach panda_link7"
    );
    let padded = env.distance_self(&distance_request(), &posed, &[]);
    assert!(
        padded
            .minimum_distance
            .link_names
            .iter()
            .any(|name| name == "panda_link5"),
        "the padded link must be in the pair that now collides, got {:?}",
        padded.minimum_distance.link_names
    );

    // Step 3 -- the same magnitude on a link that is not in that pair leaves
    // the scene clear. Without this the verdict above would also be satisfied
    // by a backend that collides whenever any padding is non-zero.
    let mut elsewhere = LinkPaddingScale::default();
    elsewhere.set_link_padding("panda_hand", SELF_PADDING);
    let elsewhere = ParryCollisionEnv::new(World::new(), elsewhere);
    assert!(
        !self_collides(&elsewhere),
        "padding panda_hand by {SELF_PADDING} instead must leave the pose clear"
    );

    // Step 4 -- reversibility, as in the world half: padding is read per
    // query, not baked into the link geometry.
    assert!(
        env.padding_scale_mut().set_link_padding("panda_link5", 0.0),
        "restoring panda_link5's padding to 0.0 must report a change"
    );
    assert!(
        !self_collides(&env),
        "restoring padding to 0.0 must restore the clear self verdict"
    );
    let restored = env.distance_self(&distance_request(), &posed, &[]);
    assert_eq!(
        restored.minimum_distance.distance, unpadded.minimum_distance.distance,
        "restoring padding to 0.0 must restore the exact unpadded self distance"
    );
}
