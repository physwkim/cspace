// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Measures [`moveit_planners_sbp::PlanningSceneValidityChecker::is_valid`]'s
//! per-call cost on panda's real, mesh-loaded collision geometry (empty
//! world, no constraints) -- the number
//! `planning_scene_validity.rs`'s own "No state pooling" doc section cites.
//!
//! Run with `cargo run --example planning_scene_validity_bench -p
//! moveit-planners-sbp` for the debug-profile figure, or add `--release` for
//! the optimized one; the binary reports which profile it was built under so
//! the two are never confused after the fact.
//!
//! No `assert!`: machine speed varies too much across hosts for a hard bound
//! to be meaningful, and `nextest` swallows stdout on a passing `#[test]`
//! anyway, so a number asserted only there is invisible in a normal run
//! while still costing every suite run its 50 real-mesh collision checks --
//! see `crates/moveit-geometry/examples/octree_compound_bench.rs` for the
//! established precedent of moving a printing-only measurement here instead.

use std::fs;
use std::time::Instant;

use moveit_collision::{CollisionRequest, LinkPaddingScale, ParryCollisionEnv};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_planners_sbp::{
    JointModelGroupSpace, PlanningSceneValidityChecker, StateSpace, StateValidityChecker,
};
use moveit_scene::PlanningScene;
use moveit_srdf::SrdfModel;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

/// The `moveit_resources_panda_description` package committed under
/// `fixtures/meshes/` -- see `collision_parity.rs`'s
/// `fixture_mesh_search_paths` for the pattern this copies.
fn fixture_mesh_search_paths() -> MeshSearchPaths {
    let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
    MeshSearchPaths::new([(
        "moveit_resources_panda_description",
        format!("{meshes_root}/panda_description"),
    )])
}

fn load_panda() -> (RobotModel, SrdfModel) {
    let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
    let urdf_xml = fs::read_to_string(format!("{root}/panda.urdf")).unwrap();
    let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(format!("{root}/panda.srdf")).unwrap();
    let model =
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &fixture_mesh_search_paths())
            .expect("fixture model must build");
    (model, srdf)
}

fn main() {
    let (model, srdf) = load_panda();
    let space = JointModelGroupSpace::new(&model, "panda_arm").unwrap();
    let mut scene = PlanningScene::new(&model, &srdf);
    let env = ParryCollisionEnv::new(moveit_collision::World::new(), LinkPaddingScale::default());
    let checker = PlanningSceneValidityChecker::new(
        &mut scene,
        &env,
        CollisionRequest::default(),
        None,
        &space,
    );

    let mut rng = ChaCha8Rng::seed_from_u64(0);
    const SAMPLES: usize = 50;
    let samples: Vec<_> = (0..SAMPLES)
        .map(|_| space.sample_uniform(&mut rng))
        .collect();

    let mut durations = Vec::with_capacity(samples.len());
    for sample in &samples {
        let start = Instant::now();
        std::hint::black_box(checker.is_valid(sample));
        durations.push(start.elapsed());
    }
    let total: std::time::Duration = durations.iter().sum();

    let profile = if cfg!(debug_assertions) {
        "debug"
    } else {
        "release"
    };
    println!(
        "PlanningSceneValidityChecker::is_valid [{profile}]: mean {:?}/call, min {:?}, max {:?}, over {} calls (panda_arm, real mesh-loaded self-collision geometry, empty world, no constraints)",
        total / durations.len() as u32,
        durations.iter().min().unwrap(),
        durations.iter().max().unwrap(),
        durations.len()
    );
}
