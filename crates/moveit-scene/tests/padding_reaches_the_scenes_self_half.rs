// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Pins the one place this port's single-environment collision design gives a
//! different answer from upstream's two-environment one.
//!
//! Upstream `PlanningScene::checkCollision` (`moveit_core/planning_scene/src/
//! planning_scene.cpp:436-455`, moveit2 @ `e017c91e`) picks a *different*
//! environment per half:
//!
//! ```cpp
//! req.pad_environment_collisions ? getCollisionEnv()->checkRobotCollision(...)
//!                                : getCollisionEnvUnpadded()->checkRobotCollision(...);
//! // ... early return ...
//! req.pad_self_collisions ? getCollisionEnv()->checkSelfCollision(...)
//!                         : getCollisionEnvUnpadded()->checkSelfCollision(...);
//! ```
//!
//! The two defaults are not symmetric -- `pad_environment_collisions = true`,
//! `pad_self_collisions = false` (`collision_common.hpp:154`, `:157`) -- and
//! nothing in the whole `moveit2` tree ever assigns the second one (the three
//! sites that assign the first are `planning_scene.cpp`'s own
//! `checkCollisionUnpadded` overloads, `plan_execution.cpp:285` and
//! `BenchmarkExecutor.cpp:1012`, all of them setting it to `false`). So
//! upstream's effective rule is: the robot-vs-world half is padded, the self
//! half never is.
//!
//! D4 gives [`PlanningScene`] one caller-owned environment, so this port's
//! rule is uniform -- whatever padding that environment carries reaches both
//! halves. The two rules agree on every unpadded environment and diverge on
//! every padded one, and that is what the two cases below fix in place: same
//! scene, same pose, empty world so only the self half can produce a verdict,
//! one padded environment and one not.
//!
//! # What this does not claim
//!
//! Nothing here says the port's answer is wrong. Padding self-collision is
//! what `CollisionEnvFCL` itself does at the backend level -- both of its
//! queries read the one padded `robot_geoms_` -- and upstream's asymmetry
//! lives one layer up, in which of two environments `PlanningScene` hands the
//! query to. What the divergence needs is a caller that (a) builds a padded
//! environment and (b) goes through [`PlanningScene::check_collision`] rather
//! than the backend directly. This workspace has none today; upstream's one
//! `move_group`-reachable unpadded caller,
//! `PlanExecution::isRemainingPathValid` (`plan_execution.cpp:268-353`), is
//! not ported. If that caller arrives, the fix is for *it* to pass its own
//! unpadded environment -- `let mut u = env.clone(); *u.padding_scale_mut() =
//! LinkPaddingScale::default();`, which shares the world's `Arc<Object>`
//! contents and the octree cache -- not for [`PlanningScene`] to grow a
//! second environment parameter.
//!
//! # The numbers
//!
//! Measured on this backend at the pose below, padding `panda_link5` alone
//! (`padding -> min self distance`, nearest pair `panda_link5`/`panda_link7`
//! throughout):
//!
//! ```text
//! 0.00   ->  +0.022134891
//! 0.02   ->  +0.004424666
//! 0.0223 ->  +0.002464738
//! 0.03   ->  -0.004126921
//! 0.05   ->  -0.010166264
//! ```
//!
//! `0.05` is used below: it clears the flip point (between `0.0223` and
//! `0.03`) by roughly `2x`, the same margin upstream's own `PaddingTest`
//! leaves. The unpadded `+0.022134891` is upstream's `DistanceSelf` constant
//! (`test_collision_common_panda.hpp:236-244` asserts `EXPECT_NEAR(
//! res.distance, 0.022, 0.001)`), so the pose is upstream's, not one invented
//! to make the flip happen.
//!
//! The backend-level half of this boundary --  that
//! `CollisionEnv::check_self_collision` reads [`LinkPaddingScale`] at all --
//! is `moveit-collision/tests/link_padding_changes_collision_verdict.rs`.
//! This file is only about which of the two halves a *scene-level*
//! `check_collision` applies it to.

use moveit_collision::{CollisionRequest, LinkPaddingScale, ParryCollisionEnv, World};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_scene::PlanningScene;
use moveit_srdf::SrdfModel;

const PANDA_URDF: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.urdf");
const PANDA_SRDF: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.srdf");

/// Upstream's `setToHome` (`test_collision_common_panda.hpp:56-68`).
const HOME: [(&str, f64); 4] = [
    ("panda_joint2", -0.785),
    ("panda_joint4", -2.356),
    ("panda_joint6", 1.571),
    ("panda_joint7", 0.785),
];

/// Enough padding on `panda_link5` to close its `0.0221` clearance to
/// `panda_link7`; see the module doc's sweep.
const PADDING: f64 = 0.05;

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

/// `true` when [`PlanningScene::check_collision`] reports a collision at the
/// home pose against `env`. The world is empty, so the robot-vs-world half
/// can never contribute and the answer is the self half's alone.
fn collides_at_home(model: &RobotModel, srdf: &SrdfModel, env: &ParryCollisionEnv) -> bool {
    let mut scene = PlanningScene::new(model, srdf);
    for (name, value) in HOME {
        scene
            .current_state_mut()
            .set_variable_position(name, value)
            .unwrap_or_else(|e| panic!("setting {name}: {e}"));
    }
    scene
        .check_collision(env, &CollisionRequest::default())
        .collision
}

#[test]
fn an_unpadded_env_gives_the_scene_upstreams_own_self_verdict() {
    let model = panda();
    let srdf = SrdfModel::parse_file(PANDA_SRDF).expect("fixture SRDF must parse");
    let env = ParryCollisionEnv::new(World::new(), LinkPaddingScale::default());

    assert!(
        !collides_at_home(&model, &srdf, &env),
        "the home pose is self-collision free, and with no padding both this \
         port and upstream's unpadded self half must say so"
    );
}

#[test]
fn a_padded_env_reaches_the_scenes_self_half_where_upstreams_default_would_not() {
    let model = panda();
    let srdf = SrdfModel::parse_file(PANDA_SRDF).expect("fixture SRDF must parse");
    let mut padding = LinkPaddingScale::default();
    padding.set_link_padding("panda_link5", PADDING);
    let env = ParryCollisionEnv::new(World::new(), padding);

    assert!(
        collides_at_home(&model, &srdf, &env),
        "PlanningScene::check_collision applies the env's padding to the self \
         half too; upstream's pad_self_collisions default of false would run \
         this half against getCollisionEnvUnpadded() and report no collision"
    );
}
