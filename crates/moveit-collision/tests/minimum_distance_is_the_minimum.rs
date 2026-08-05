// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! `distance_robot`'s reported minimum must be the smallest value over the
//! pairs it considered -- and in particular a pair that merely *touches* must
//! not hide a pair that is deeply inside.
//!
//! This is the invariant behind `doc/upstream-bugs.md`'s
//! `distance-callback-threshold-suppresses-deeper-pairs`, and it is what makes
//! `PORTING-PLAN.md` §5 Phase 3's `distance: f64` clause fail on fanuc. The
//! configuration is fanuc's own: `fixtures/fanuc.urdf`'s `base_link` mesh
//! reaches exactly `z = 0` and `tools/moveit-diff`'s floor box is `4 x 4 x 0.1`
//! centred at `z = -0.05`, so its top face is exactly `z = 0` too. `base_link`
//! is fixed to the world, so *every* sampled state presents that same
//! exactly-tangent pair alongside whatever else has swung into the floor.
//!
//! # What upstream does with it (measured, `e017c91ee`)
//!
//! State `collision[9651]` of the seed-1 sweep, one `scene_diff_collision`
//! request per row, the only thing changed between rows being one
//! `set_acm_entry` -- no geometry moves:
//!
//! | pairs allowed away | oracle `robot_collision` | oracle `robot_distance` |
//! |---|---|---|
//! | (none)                      | `true`  | `-9.93013661298909247e-16` |
//! | `base_link/floor`           | `true`  | `-1.39489505627222377e0` |
//! | `+ link_4/floor`            | `false` | `+1.12707096491874714e-1` |
//!
//! Upstream reports `-9.9e-16` while its own distance query, with nothing but
//! that one tangent pair taken out of consideration, reports `-1.39` for a
//! pair that was there the whole time -- 1.39 below what it called the
//! minimum. Its own `robot_contacts` for the unmasked request names both pairs
//! (`base_link/floor` depth `4.440892098500626e-16`, `link_4/floor` depth
//! `0.018272381609339305`), so the deeper pair is not merely reachable, it is
//! already reported by the collision call on the same state.
//!
//! The mechanism is in `distanceCallback`
//! (`moveit_core/collision_detection_fcl/src/collision_common.cpp`): `:574`
//! sets `dist_threshold` to the running `res->minimum_distance.distance`, and
//! `:608`'s `if (distance < dist_threshold)` gates the whole signed-distance
//! block on the *raw* `fcl::distance` return, while `:663`'s
//! `dist_result.distance = -contact.penetration_depth` -- the value that
//! becomes the running minimum at `:694` -- is a penetration depth. The two
//! sides of `:608` are different quantities: once any pair makes the running
//! minimum negative, a later penetrating pair whose raw `fcl::distance` does
//! not come back below it is dropped before its depth is ever computed. There
//! is no tie-break rule to port; `:694` is a plain strict `<`.
//!
//! # Why "near-tied candidates" was the wrong reading
//!
//! `PORTING-PLAN.md` §218.4 recorded fanuc's robot side as 2,302 pair-flips
//! and read that as the minimum changing hands between two nearly equal
//! candidates. That is a claim about the two *values*, and nothing had
//! measured the second one. `tools/moveit-diff --pair-probe-json` measures it:
//! for each failing state it re-runs this port's own `distance_robot` with
//! every pair masked out but the oracle's, so both candidates are in one
//! metric. No tie exists -- over all 2,302 of fanuc's robot-side flips the
//! smallest gap is `2.264e-4`, 2.26x the clause's own `1e-4`. `PORTING-PLAN.md`
//! §247 carries the full distribution.
//!
//! # What makes these tests bite
//!
//! Porting the defect in is a one-token edit, and it was run: in
//! `crates/moveit-collision/src/parry.rs`'s `accumulate_distance`, gating on
//! the raw separation distance the way `:608` does (gating on
//! `contact.dist.max(0.0)` where the port gates on `contact.dist`) makes this
//! port answer `base_link/floor`, upstream's own answer, and reddens
//! `the_tangent_pair_does_not_win_over_the_deep_one` and
//! `masking_the_tangent_pair_does_not_move_the_answer` while leaving
//! `the_state_presents_a_tangent_pair_and_a_deep_one` green. That split is
//! the point: the premise test queries each pair on its own, where nothing
//! can be suppressed, so only the other two carry the invariant.

use std::fs;
use std::sync::Arc;

use moveit_collision::{
    AllowedCollisionMatrix, CollisionEnv, DistanceRequest, LinkPaddingScale, ParryCollisionEnv,
    World,
};
use moveit_geometry::{Cuboid, Isometry3, Shape};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::{Posed, RobotState};

/// Seed-1 state 9651 of Phase 3's fanuc sweep -- the state `PORTING-PLAN.md`
/// §218.4 named as fanuc's worst `distance` deviation, so the numbers here are
/// the ones the clause is failing on rather than a configuration invented to
/// make the point.
const JOINTS: [(&str, f64); 6] = [
    ("joint_1", 2.3781846893113108),
    ("joint_2", 1.6345888887066395),
    ("joint_3", -0.20823032780084771),
    ("joint_4", 2.0400704372860488),
    ("joint_5", -0.49419655353762204),
    ("joint_6", 0.01724972829222704),
];

const FLOOR_THICKNESS: f64 = 0.1;

/// Tolerance for the two penetration values this file pins. Measured before
/// it was set: on this build every one of them reproduces its constant bit
/// for bit -- the masked, the two-pair and the SRDF-ACM queries all return
/// `-2.89703002516375319e-1` with a residual of exactly `0`, and masking the
/// tangent pair moves the answer by exactly `0`. So `1e-9` is margin against
/// a rebuild reassociating the mesh-against-box contact, not a fitted
/// residual, and what constrains it is its ceiling: it sits `8.5` orders
/// below the `2.897e-1` it brackets and `5.3` orders below `2.264e-4`, the
/// smallest candidate gap the fanuc robot-side sweep measured. It cannot
/// absorb either quantity this file is about.
const TOL: f64 = 1e-9;

fn build_fanuc() -> RobotModel {
    let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/fanuc.urdf");
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/fanuc.srdf");
    let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
    let urdf_xml = fs::read_to_string(urdf_path).expect("fixture URDF must be readable");
    let urdf = urdf_rs::read_file(urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    let paths = MeshSearchPaths::new([(
        "moveit_resources_fanuc_description",
        format!("{meshes_root}/fanuc_description"),
    )]);
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &paths)
        .expect("fixture model must build")
}

fn floor_env() -> ParryCollisionEnv {
    let mut world = World::new();
    world.add_shape(
        "floor",
        Arc::new(Shape::Cuboid(
            Cuboid::new(4.0, 4.0, FLOOR_THICKNESS).expect("positive cuboid dimensions"),
        )),
        Isometry3::translation(0.0, 0.0, -FLOOR_THICKNESS / 2.0),
    );
    ParryCollisionEnv::new(world, LinkPaddingScale::default())
}

/// Runs `body` with state 9651 posed, the way `tools/moveit-diff` poses it.
fn with_state_9651<T>(model: &RobotModel, body: impl FnOnce(Posed<'_, '_>) -> T) -> T {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    for (name, value) in JOINTS {
        state
            .set_variable_position(name, value)
            .expect("fanuc joint name must resolve");
    }
    body(state.update())
}

/// An ACM over every link plus `floor`, everything allowed except the listed
/// pairs -- so a query through it sees exactly those pairs and nothing else.
fn only(model: &RobotModel, pairs: &[(&str, &str)]) -> AllowedCollisionMatrix {
    let mut names: Vec<String> = model
        .link_models()
        .iter()
        .map(|link| link.name().to_owned())
        .collect();
    names.push("floor".to_owned());
    let mut acm = AllowedCollisionMatrix::from_names(&names, true);
    for (first, second) in pairs {
        acm.set_entry(first, second, false);
    }
    acm
}

/// The SRDF ACM, i.e. what the sweep itself queries through.
fn srdf_acm() -> AllowedCollisionMatrix {
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/fanuc.srdf");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    AllowedCollisionMatrix::from_srdf(&srdf)
}

/// `(pair, distance)` from `distance_robot` through `acm`.
fn minimum(
    env: &ParryCollisionEnv,
    acm: &AllowedCollisionMatrix,
    posed: Posed<'_, '_>,
) -> (String, f64) {
    let result = env.distance_robot(
        &DistanceRequest {
            enable_signed_distance: true,
            acm: Some(acm),
            ..DistanceRequest::default()
        },
        &posed,
        &[],
    );
    let names = &result.minimum_distance.link_names;
    (
        format!("{}/{}", names[0], names[1]),
        result.minimum_distance.distance,
    )
}

/// The premise: `base_link` is exactly tangent to the floor and `link_4` is
/// `0.29 m` inside it. Without this the three tests below would pass while
/// measuring a configuration in which nothing is suppressed.
#[test]
fn the_state_presents_a_tangent_pair_and_a_deep_one() {
    let model = build_fanuc();
    let env = floor_env();
    with_state_9651(&model, |posed| {
        let (pair, base) = minimum(&env, &only(&model, &[("base_link", "floor")]), posed);
        assert_eq!(pair, "base_link/floor");
        assert_eq!(
            base, 0.0,
            "base_link's mesh reaches exactly z=0 and the floor's top face is exactly z=0"
        );

        let (pair, deep) = minimum(&env, &only(&model, &[("link_4", "floor")]), posed);
        assert_eq!(pair, "link_4/floor");
        assert!(
            (deep - -2.897030025163753e-1).abs() < TOL,
            "link_4 is deep inside the floor, measured {deep:.17e}"
        );
    });
}

/// The invariant. With both pairs in play the answer must be the deeper one.
///
/// Upstream answers `-9.93013661298909247e-16` here (module doc, row 1) --
/// the tangent pair -- because the tangent pair is visited first and its
/// signed value then gates every later pair out at `collision_common.cpp:608`.
#[test]
fn the_tangent_pair_does_not_win_over_the_deep_one() {
    let model = build_fanuc();
    let env = floor_env();
    with_state_9651(&model, |posed| {
        let (pair, both) = minimum(
            &env,
            &only(&model, &[("base_link", "floor"), ("link_4", "floor")]),
            posed,
        );
        assert_eq!(
            pair, "link_4/floor",
            "the minimum over two pairs must be the smaller of the two"
        );
        assert!(
            (both - -2.897030025163753e-1).abs() < TOL,
            "measured {both:.17e}"
        );
    });
}

/// The oracle's own falsifying experiment, run against this port: taking the
/// tangent pair out of consideration must not move the answer. Upstream's
/// answer moves by `1.39` under exactly this edit (module doc, rows 1 and 2).
#[test]
fn masking_the_tangent_pair_does_not_move_the_answer() {
    let model = build_fanuc();
    let env = floor_env();
    with_state_9651(&model, |posed| {
        let (with_pair, with_value) = minimum(&env, &srdf_acm(), posed);
        let mut masked = srdf_acm();
        masked.set_entry("base_link", "floor", true);
        let (without_pair, without_value) = minimum(&env, &masked, posed);
        assert_eq!(with_pair, without_pair, "the winning pair must not change");
        assert_eq!(
            with_value, without_value,
            "the reported minimum must not change when a pair above it is masked out"
        );
        assert!(
            (with_value - -2.897030025163753e-1).abs() < TOL,
            "measured {with_value:.17e}"
        );
    });
}
