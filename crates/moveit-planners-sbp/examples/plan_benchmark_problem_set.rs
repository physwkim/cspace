// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause
//
// No upstream file: this is Phase 7 benchmark infrastructure (PORTING-PLAN.md
// §5, §118), not a port -- see `lib.rs`'s own top-of-file comment for why
// this crate has no OMPL C++ counterpart to transcribe.

//! Endpoint-valid `(start, goal)` pair sampler for one obstacle
//! configuration, emitting an oracle `plan`-op request on stdout.
//!
//! # Why this is docker-free
//!
//! This binary never talks to the oracle itself -- every other live-oracle
//! call in this workspace goes through a shell script wrapping
//! `tools/moveit-oracle/run-oracle.sh` (`sg docker -c ...`), never from
//! inside committed Rust source, and this keeps that boundary: it only
//! samples pairs and prints the request JSON those wrappers can pipe in.
//! `benches/sweep_baseline.sh` is the companion script that actually invokes
//! the oracle, one fresh process per config (see that script's own doc
//! comment for why one process per config, not one for the whole sweep).
//!
//! # Why an endpoint filter at all
//!
//! A `(start, goal)` pair sampled uniformly at random is frequently invalid
//! on one or both ends (self-collision or obstacle penetration) once real
//! obstacles are in the scene. An unfiltered problem set would measure how
//! often the *sampler* lands on a valid pair, not how well either planner
//! solves real problems -- Phase 7's completion conditions (§5: success
//! rate, path validity, path length) are only meaningful over problems both
//! endpoints of which are already known-valid. This binary counts and
//! reports the rejection rate (`bad_ep`) rather than hiding it.
//!
//! # Usage
//!
//! `cargo run --release --example plan_benchmark_problem_set -p
//! moveit-planners-sbp -- <config> <count> <seed>`
//!
//! `config` is one of [`CONFIGS`]. `count` is the number of valid pairs to
//! collect. `seed` seeds this run's `ChaCha8Rng` (both the pair sampling
//! *and* the request's own `seed` field, so the oracle's OMPL RNG and this
//! generator's endpoint sampling are both pinned by the one number a caller
//! passes in -- there is no independent oracle-side seed to reconcile).
//! Prints one NDJSON `plan` request line to stdout; per-run sampling stats
//! to stderr.

use std::collections::BTreeMap;
use std::env;
use std::fs;
use std::sync::Arc;

use moveit_collision::{CollisionRequest, LinkPaddingScale, ParryCollisionEnv, World};
use moveit_geometry::{Cuboid, Isometry3, Shape};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_planners_sbp::{
    CompoundValue, JointModelGroupSpace, PlanningSceneValidityChecker, StateSpace,
    StateValidityChecker,
};
use moveit_scene::PlanningScene;
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

/// Every obstacle configuration this generator knows how to build. Kept as a
/// flat list (rather than an enum) so `benches/sweep_baseline.sh` can drive
/// it by name without a matching Rust-side enum to keep in sync.
const CONFIGS: &[&str] = &["empty", "floor", "floor_wall", "slot", "corridor", "cage"];

/// The `moveit_resources_panda_description` package committed under
/// `fixtures/meshes/` -- same pattern as
/// `examples/planning_scene_validity_bench.rs`.
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

/// One `objects[]` entry the oracle's `plan` op reads: a box, by id, full
/// x/y/z extents (matching `shapes::Box`'s own constructor, and
/// [`Cuboid::new`]'s), and world pose.
struct Obstacle {
    id: &'static str,
    size: (f64, f64, f64),
    pose: Isometry3,
}

/// A table-top surface just below `panda_link0`'s own mounting origin: a
/// first attempt with the top face flush at `z = 0` made every sample
/// self-collide unconditionally (panda_link0's own collision mesh extends
/// slightly below its origin), which is not a benchmark difficulty lever, it
/// is a bug -- so this leaves a 3 cm clearance instead. Every non-`"empty"`
/// config includes this; a sample that dips the arm itself below the table
/// still self-obstructs, which *is* a real difficulty lever.
fn floor() -> Obstacle {
    Obstacle {
        id: "floor",
        size: (2.0, 2.0, 0.5),
        pose: Isometry3::translation(0.0, 0.0, -0.28),
    }
}

/// One thin wall on the `+x` side of panda's reach (panda_arm's own extent
/// is roughly 0.85 m from `panda_link0`; 0.45 m sits inside that so the wall
/// is actually reachable, not decorative).
fn wall() -> Obstacle {
    Obstacle {
        id: "wall",
        size: (0.05, 1.6, 1.6),
        pose: Isometry3::translation(0.45, 0.0, 0.8),
    }
}

/// Two wall segments on the `+x` side leaving a 0.45 m gap in `y` at
/// `y = 0` -- wide enough to be routinely passable, narrow enough to rule
/// out most of the reachable `+x` volume outright.
fn slot_walls() -> [Obstacle; 2] {
    [
        Obstacle {
            id: "slot_left",
            size: (0.05, 0.65, 1.6),
            pose: Isometry3::translation(0.45, -0.55, 0.8),
        },
        Obstacle {
            id: "slot_right",
            size: (0.05, 0.65, 1.6),
            pose: Isometry3::translation(0.45, 0.55, 0.8),
        },
    ]
}

/// Like [`slot_walls`] but the `y` gap is narrower (0.2 m, not 0.45 m) *and*
/// capped by a ceiling segment -- a true tunnel, not just a doorway, so a
/// solution has to stay both narrow in `y` and low in `z` at once.
fn corridor_walls() -> [Obstacle; 3] {
    [
        Obstacle {
            id: "corridor_left",
            size: (0.05, 0.75, 1.6),
            pose: Isometry3::translation(0.45, -0.475, 0.8),
        },
        Obstacle {
            id: "corridor_right",
            size: (0.05, 0.75, 1.6),
            pose: Isometry3::translation(0.45, 0.475, 0.8),
        },
        Obstacle {
            id: "corridor_ceiling",
            size: (0.9, 1.7, 0.05),
            pose: Isometry3::translation(0.45, 0.0, 1.4),
        },
    ]
}

/// Four side walls plus a ceiling, all with a 0.6 m clearance from
/// `panda_link0`, and exactly one 0.3 m door (in the `+x` wall). This is
/// deliberately near-enclosing: most of the reachable volume is walled off,
/// so most random pairs are invalid on at least one end
/// (`benches/sweep_baseline.sh`'s own measurement: 76.5% at n=20). Do not
/// assume what that does to the *surviving* pairs' planning difficulty --
/// measured here, this config's median `ptc_evaluations` is the *highest*
/// of all six, not the lowest, unlike a tightness-implies-easier-survivors
/// effect another panel measured for a different, untransferable obstacle
/// geometry (see `benches/sweep_baseline.sh`'s doc comment).
fn cage_walls() -> [Obstacle; 6] {
    [
        Obstacle {
            id: "cage_east_lower",
            size: (0.05, 1.2, 0.9),
            pose: Isometry3::translation(0.6, -0.45, 0.45),
        },
        Obstacle {
            id: "cage_east_upper",
            size: (0.05, 1.2, 0.4),
            pose: Isometry3::translation(0.6, -0.45, 1.4),
        },
        Obstacle {
            id: "cage_west",
            size: (0.05, 1.2, 1.6),
            pose: Isometry3::translation(-0.6, 0.0, 0.8),
        },
        Obstacle {
            id: "cage_north",
            size: (1.2, 0.05, 1.6),
            pose: Isometry3::translation(0.0, 0.6, 0.8),
        },
        Obstacle {
            id: "cage_south",
            size: (1.2, 0.05, 1.6),
            pose: Isometry3::translation(0.0, -0.6, 0.8),
        },
        Obstacle {
            id: "cage_ceiling",
            size: (1.2, 1.2, 0.05),
            pose: Isometry3::translation(0.0, 0.0, 1.6),
        },
    ]
}

/// Every obstacle for `config`, floor included where applicable. Panics on
/// an unknown name (this binary is not a public library API, so an argument
/// error surfacing as a panic with the bad value in the message is
/// sufficient -- there is no caller to hand a typed error back to).
fn obstacles_for(config: &str) -> Vec<Obstacle> {
    match config {
        "empty" => vec![],
        "floor" => vec![floor()],
        "floor_wall" => {
            let mut v = vec![floor()];
            v.push(wall());
            v
        }
        "slot" => {
            let mut v = vec![floor()];
            v.extend(slot_walls());
            v
        }
        "corridor" => {
            let mut v = vec![floor()];
            v.extend(corridor_walls());
            v
        }
        "cage" => {
            let mut v = vec![floor()];
            v.extend(cage_walls());
            v
        }
        other => panic!("unknown config {other:?}, expected one of {CONFIGS:?}"),
    }
}

/// Row-major 4x4, matching the oracle's `fromRowMajor4x4`/`toRowMajor4x4`
/// (`tools/moveit-oracle/src/oracle.cpp`) -- the same encoding
/// `tools/moveit-diff/src/rust_impl.rs::to_row_major_4x4` uses, reimplemented
/// here rather than imported since that tool is a separate binary crate this
/// one does not depend on.
fn to_row_major_4x4(pose: &Isometry3) -> [f64; 16] {
    let m = pose.to_homogeneous();
    let mut out = [0.0; 16];
    for r in 0..4 {
        for c in 0..4 {
            out[r * 4 + c] = m[(r, c)];
        }
    }
    out
}

/// Reads `state` (a [`JointModelGroupSpace`] sample) back out as a
/// joint-name -> value map, the shape the oracle's `plan` op's
/// `problems[].start`/`.goal` (and `distance_probes[].a`/`.b`) expect.
fn state_to_joint_map(
    space: &JointModelGroupSpace,
    model: &RobotModel,
    variable_names: &[String],
    state: &[CompoundValue],
) -> BTreeMap<String, f64> {
    let mut robot_state = RobotState::new(model);
    robot_state.set_to_default_values();
    space.write_robot_state(&state.to_vec(), &mut robot_state);
    variable_names
        .iter()
        .map(|name| {
            let value = robot_state
                .variable_position(name)
                .unwrap_or_else(|e| panic!("variable_position({name}): {e}"));
            (name.clone(), value)
        })
        .collect()
}

/// Sampling-attempt ceiling for one config's endpoint filter: high enough
/// that `cage` (the tightest configuration this file builds) still reaches
/// its target count in practice, but finite so a config with genuinely too
/// little valid volume fails loudly instead of hanging.
const MAX_SAMPLE_ATTEMPTS: usize = 500_000;

fn main() {
    let args: Vec<String> = env::args().collect();
    let config = args
        .get(1)
        .unwrap_or_else(|| panic!("usage: <config> <count> <seed>; got {args:?}"));
    let count: usize = args
        .get(2)
        .unwrap_or_else(|| panic!("usage: <config> <count> <seed>; got {args:?}"))
        .parse()
        .expect("count must be a non-negative integer");
    let seed: u64 = args
        .get(3)
        .unwrap_or_else(|| panic!("usage: <config> <count> <seed>; got {args:?}"))
        .parse()
        .expect("seed must be a u64");

    let (model, srdf) = load_panda();
    let group_name = "panda_arm";
    let space = JointModelGroupSpace::new(&model, group_name)
        .unwrap_or_else(|e| panic!("JointModelGroupSpace::new({group_name}): {e}"));
    let variable_names = model
        .joint_model_group(group_name)
        .unwrap_or_else(|e| panic!("joint_model_group({group_name}): {e}"))
        .variable_names()
        .to_vec();

    let obstacles = obstacles_for(config);
    let mut world = World::new();
    for obstacle in &obstacles {
        world.add_shape(
            obstacle.id,
            Arc::new(Shape::Cuboid(
                Cuboid::new(obstacle.size.0, obstacle.size.1, obstacle.size.2)
                    .unwrap_or_else(|e| panic!("Cuboid::new({:?}): {e}", obstacle.size)),
            )),
            obstacle.pose,
        );
    }
    let env = ParryCollisionEnv::new(world, LinkPaddingScale::default());
    let mut scene = PlanningScene::new(&model, &srdf);
    let checker = PlanningSceneValidityChecker::new(
        &mut scene,
        &env,
        CollisionRequest::default(),
        None,
        &space,
    );

    let mut rng = ChaCha8Rng::seed_from_u64(seed);
    let mut problems: Vec<(Vec<CompoundValue>, Vec<CompoundValue>)> = Vec::with_capacity(count);
    let mut attempts = 0usize;
    let mut bad_ep = 0usize;
    while problems.len() < count {
        attempts += 1;
        if attempts > MAX_SAMPLE_ATTEMPTS {
            panic!(
                "config {config}: exceeded {MAX_SAMPLE_ATTEMPTS} sampling attempts with only \
                 {}/{count} valid pairs found ({bad_ep} rejected) -- this config may not have \
                 enough valid volume for the requested count",
                problems.len()
            );
        }
        let start = space.sample_uniform(&mut rng);
        let goal = space.sample_uniform(&mut rng);
        if checker.is_valid(&start) && checker.is_valid(&goal) {
            problems.push((start, goal));
        } else {
            bad_ep += 1;
        }
    }

    eprintln!(
        "config={config} count={count} attempts={attempts} bad_ep={bad_ep} bad_ep_rate={:.1}%",
        100.0 * bad_ep as f64 / attempts as f64
    );

    let objects_json: Vec<serde_json::Value> = obstacles
        .iter()
        .map(|o| {
            serde_json::json!({
                "id": o.id,
                "pose": to_row_major_4x4(&o.pose),
                "shape": {"type": "box", "size": [o.size.0, o.size.1, o.size.2]},
            })
        })
        .collect();

    let problems_json: Vec<serde_json::Value> = problems
        .iter()
        .enumerate()
        .map(|(i, (start, goal))| {
            serde_json::json!({
                "id": i,
                "start": state_to_joint_map(&space, &model, &variable_names, start),
                "goal": state_to_joint_map(&space, &model, &variable_names, goal),
            })
        })
        .collect();

    let request = serde_json::json!({
        "id": 0,
        "op": "plan",
        "group": group_name,
        "seed": seed,
        "range": 0.05,
        "motion_resolution": 0.01,
        "max_iterations": 2000,
        "objects": objects_json,
        "problems": problems_json,
    });

    println!("{}", serde_json::to_string(&request).unwrap());
}
