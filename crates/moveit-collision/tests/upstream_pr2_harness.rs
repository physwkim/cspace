// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! The reachable assertions of upstream's `test_collision_common_pr2.hpp`,
//! restated against this backend -- and, for each one that is not reachable,
//! the measured reason.
//!
//! Same split as `upstream_panda_harness.rs`: `doc/port-coverage.md`
//! classifies the header `decided-non-port` because it is a GoogleTest
//! `TYPED_TEST_P` fixture parameterised over `CollisionAllocatorType`, which
//! this port has one backend for; what it *asserts* is a separate thing, and
//! that is what is kept. Unlike the panda header, most of this one is not
//! reachable, so the accounting below is the substance of the row.
//!
//! # The 41 assertion sites, and where each one goes
//!
//! | upstream test | sites | here |
//! |---|---|---|
//! | `InitOK` (`:100-103`) | 1 | [`build_pr2`]'s `.expect` |
//! | `DefaultNotInCollision` (`:105-115`) | 1 | [`default_not_in_collision`], plus the measurement that it is vacuous |
//! | `LinksInCollision` (`:117-157`) | 3 | **not reachable**: `updateStateWithLinkAt` |
//! | `ContactReporting` (`:159-212`) | 9 | **not reachable**: `updateStateWithLinkAt` |
//! | `ContactPositions` (`:214-283`) | 9 | **not reachable**: `updateStateWithLinkAt` (and `:282` asserts on an unwritten result) |
//! | `AttachedBodyTester` (`:285-352`) | 6 | **not reachable**: `updateStateWithLinkAt` |
//! | `DiffSceneTester` (`:354-406`) | 3 | **no substance**: all three are `EXPECT_TIME_LT` |
//! | `ConvertObjectToAttached` (`:408-474`) | 4 | 3 are `EXPECT_TIME_LT`; the 1 real one needs `kinect.dae` |
//! | `TestCollisionMapAdditionSpeed` (`:476-493`) | 1 | `EXPECT_TIME_LT`; its substance is [`collision_map_addition_lands_every_shape_in_one_object`] |
//! | `MoveMesh` (`:495-517`) | 0 | the test makes no assertion at all |
//! | `TestChangingShapeSize` (`:519-567`) | 4 | [`changing_shape_size_keeps_the_collision`]; `:528` asserts on an unwritten result, `:553`/`:565` need `kinect.dae` |
//!
//! The two "unwritten result" sites are an upstream defect, recorded as
//! `pr2-collision-test-asserts-unwritten-result` in `doc/upstream-bugs.md`:
//! both read a default-constructed `CollisionResult`, whose `collision` member
//! is initialised `false` in-class (`collision_common.hpp:353`), so they hold
//! however the checker behaves. They are the reason the accounting above
//! cannot be read as "38 assertions this port does not make" -- two of the 41
//! are assertions upstream does not make either.
//!
//! # The three reasons, each measured rather than asserted
//!
//! **`updateStateWithLinkAt` is not ported.** It writes a link's global
//! transform directly and then re-derives only its descendants
//! (`robot_state.cpp:850-871`: `global_link_transforms_[link->getLinkIndex()]
//! = transform`, then `updateLinkTransformsInternal` per child joint), so the
//! state deliberately stops matching its own joint values -- upstream's
//! declaration says so (`robot_state.hpp:1213-1220`, "neglecting the joint
//! values of its parent joint ... although they do not match the joint values
//! anymore"). That is how these tests place two specific links in contact
//! without solving for a pose. It appears 15 times in the header, in 4 of the
//! 11 tests, and has no counterpart here:
//!
//! ```console
//! $ rg -n 'update_state_with_link_at' crates/ ros/ --glob '*.rs' \
//!     --glob '!**/upstream_pr2_harness.rs'
//! $ echo $?
//! 1
//! ```
//!
//! (the exclusion is this file: the sentence above names the symbol, so an
//! unfiltered search matches its own transcript.)
//!
//! It belongs to `moveit-state`'s `RobotState`, not to this crate, so this
//! file records the blockage rather than working around it -- the workaround
//! (a URDF with floating joints, as `parry.rs`'s own tests use) would be
//! testing a different robot than the one upstream's numbers describe.
//!
//! **`kinect.dae` is not a committed fixture.** The file exists on this
//! machine at
//! `third_party/moveit_resources/pr2_description/urdf/meshes/sensors/kinect_v0/kinect.dae`
//! (164,201 bytes), but `third_party/` is a gitignored external checkout, and
//! this repository's convention -- stated at
//! `crates/moveit-geometry/tests/mesh_parity.rs:19-23` -- is that tests read
//! copies under `fixtures/meshes/`, "not from `third_party/` directly, so this
//! test runs under a plain `cargo nextest run --workspace` with no vendored
//! checkout required". That tree holds 18 files for `pr2_description`, all
//! `.stl`:
//!
//! ```console
//! $ find fixtures/meshes/pr2_description -type f | sed 's/.*\.//' | sort | uniq -c
//!      18 stl
//! ```
//!
//! Copying `kinect.dae` in is therefore not just a copy. Two things would have
//! to land with it: a COLLADA reader (this port parses STL --
//! `moveit-geometry`'s `stl.rs`), and a fix to the provenance gate, which
//! globs STL only (`verify-fixture-provenance.sh:196`,
//! `mesh_fixtures=(fixtures/meshes/**/*.stl)`) and would leave a `.dae`
//! sitting unchecked by the gate that exists to check exactly that.
//!
//! **`EXPECT_TIME_LT` cannot fail below one second.** The macro
//! (`:92-96`) is `EXPECT_LT` under `NDEBUG` and a no-op otherwise, and every
//! value it compares is built with
//! `duration_cast<std::chrono::seconds>(..).count()` -- an integer count of
//! whole seconds widened to `double`. So `EXPECT_TIME_LT(x, .05)` holds
//! whenever `x == 0`, i.e. for anything under a second, and
//! `EXPECT_TIME_LT(fabs(a - b), .05)` holds whenever the two truncate equal.
//! Restating those bounds here would copy a tolerance that no measurement
//! chose; where a timing case has substance underneath, this file asserts the
//! substance and reports the time as a number instead.

use std::fs;
use std::sync::Arc;
use std::time::Instant;

use moveit_collision::{
    AllowedCollisionMatrix, CollisionEnv, CollisionRequest, LinkPaddingScale, ParryCollisionEnv,
    World,
};
use moveit_geometry::{Cuboid, Isometry3, Shape};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;

fn fixture_mesh_search_paths() -> MeshSearchPaths {
    let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
    MeshSearchPaths::new([(
        "moveit_resources_pr2_description",
        format!("{meshes_root}/pr2_description"),
    )])
}

/// `loadTestingRobotModel("pr2")` (`:68`). Upstream's `InitOK` asserts the
/// resulting `robot_model_ok_`; here that is this `.expect`, reached by every
/// case below.
fn build_pr2() -> RobotModel {
    let urdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/pr2.urdf");
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/pr2.srdf");
    let urdf_xml = fs::read_to_string(urdf_path).expect("fixture URDF must be readable");
    let urdf = urdf_rs::read_file(urdf_path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &fixture_mesh_search_paths())
        .expect("fixture model must build")
}

/// `AllowedCollisionMatrix(robot_model_->getLinkModelNames(), true)` (`:71`)
/// -- every pair of link names entered as *allowed*. Note the `true`: this is
/// not the SRDF-derived matrix the panda header uses, and what it does to
/// self-collision checking is the subject of [`default_not_in_collision`].
fn all_allowed_acm(model: &RobotModel) -> AllowedCollisionMatrix {
    let names: Vec<String> = model
        .link_models()
        .iter()
        .map(|link| link.name().to_string())
        .collect();
    AllowedCollisionMatrix::from_names(&names, true)
}

/// `DefaultNotInCollision` (`:105-115`): the default state is self-collision
/// free under the fixture's ACM.
///
/// It passes here, and it is worth almost nothing, which is why the second
/// half of this test exists. The fixture's ACM allows *every* link pair, so
/// `checkSelfCollision` skips every pair before any geometry is consulted and
/// cannot answer anything but `false` -- for this state or any other. The
/// same state under an empty ACM is what shows whether the `false` was ever
/// about the robot's pose, and the assertion below records which of the two
/// it turned out to be.
#[test]
fn default_not_in_collision() {
    let model = build_pr2();
    let acm = all_allowed_acm(&model);
    let env = ParryCollisionEnv::new(World::new(), LinkPaddingScale::default());
    let mut state = RobotState::new(&model);
    state.set_to_default_values();
    let posed = state.update();

    let with_fixture_acm = env
        .check_self_collision(&CollisionRequest::default(), &posed, &[], Some(&acm))
        .collision;
    assert!(
        !with_fixture_acm,
        "upstream's DefaultNotInCollision: pr2's default state is self-collision free"
    );

    let with_no_acm = env
        .check_self_collision(&CollisionRequest::default(), &posed, &[], None)
        .collision;
    assert!(
        with_no_acm,
        "the assertion above is only meaningful if the all-allowed ACM is not what produced it; \
         with no ACM this same default state must report a collision, and it did not -- if this \
         ever fires, upstream's case has become a real one and this note is wrong"
    );
}

/// `TestChangingShapeSize`'s box half (`:530-546`): five times, drop the world
/// object and re-add it one notch larger, and check it still collides.
///
/// Upstream's `res1` assertion (`:528`) is on a default-constructed
/// `CollisionResult` that no call ever writes to, so it asserts nothing and is
/// not restated -- `doc/upstream-bugs.md`,
/// `pr2-collision-test-asserts-unwritten-result`. The kinect half (`:548-566`)
/// needs a mesh fixture this repository does not carry -- see the module doc.
#[test]
fn changing_shape_size_keeps_the_collision() {
    let model = build_pr2();
    let acm = all_allowed_acm(&model);
    let mut state = RobotState::new(&model);
    let posed = state.update();
    let mut env = ParryCollisionEnv::new(World::new(), LinkPaddingScale::default());

    for i in 0..5 {
        let side = 1.0 + f64::from(i) * 0.0001;
        env.world_mut().remove_object("shape");
        env.world_mut().add_shapes_to_object(
            "shape",
            &[Arc::new(Shape::Cuboid(
                Cuboid::new(side, side, side).expect("positive cuboid dimensions"),
            ))],
            &[Isometry3::identity()],
        );

        // Upstream calls `checkCollision` (self + robot). The self half is
        // inert under this ACM (see `default_not_in_collision`), so what this
        // asserts is the robot-world half: the box is not a link name, so no
        // ACM entry covers those pairs.
        let result =
            env.check_robot_collision(&CollisionRequest::default(), &posed, &[], Some(&acm));
        assert!(
            result.collision,
            "a {side} m box at the origin must collide with pr2's default state (iteration {i})"
        );
    }
}

/// `TestCollisionMapAdditionSpeed` (`:476-493`): 10,000 boxes into one object.
///
/// Upstream's only assertion is `EXPECT_TIME_LT(t, 5.0)`, which under the
/// second-truncation described in the module doc holds for anything up to
/// four whole seconds. Rather than restate a bound that measures nothing,
/// this asserts what the call is actually for -- that all 10,000 shapes land
/// in one object -- and prints the elapsed time.
///
/// Cost: the `add_shapes_to_object` call itself measured 1.113 ms / 1.339 ms /
/// 1.066 ms over three runs (forced-failure runs printing `elapsed`), and the
/// whole case runs in 0.009 s under `cargo nextest run`, so it is not gated
/// behind an opt-in the way the long sweeps are. It does *not* run a collision
/// check over those 10,000 shapes; upstream does not either, and doing so
/// would put a quadratic pair walk in a default test run.
#[test]
fn collision_map_addition_lands_every_shape_in_one_object() {
    const COUNT: usize = 10_000;

    let mut world = World::new();
    let shapes: Vec<Arc<Shape>> = (0..COUNT)
        .map(|_| {
            Arc::new(Shape::Cuboid(
                Cuboid::new(0.01, 0.01, 0.01).expect("positive cuboid dimensions"),
            ))
        })
        .collect();
    let poses = vec![Isometry3::identity(); COUNT];

    let start = Instant::now();
    world.add_shapes_to_object("map", &shapes, &poses);
    let elapsed = start.elapsed();

    let object = world.get_object("map").expect("the object was just added");
    assert_eq!(
        object.shapes().len(),
        COUNT,
        "all {COUNT} shapes must land in the one object, took {elapsed:?}"
    );
}
