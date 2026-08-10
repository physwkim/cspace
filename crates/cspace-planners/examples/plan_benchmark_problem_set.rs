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
//! cspace-planners-sbp -- <config> <count> <seed> [robot] [joint_constraint]`
//!
//! `config` is one of [`CONFIGS`]. `count` is the number of valid pairs to
//! collect. `seed` seeds this run's `ChaCha8Rng` (both the pair sampling
//! *and* the request's own `seed` field, so the oracle's OMPL RNG and this
//! generator's endpoint sampling are both pinned by the one number a caller
//! passes in -- there is no independent oracle-side seed to reconcile).
//! `robot` defaults to `panda` and selects one of [`ROBOTS`].
//! `joint_constraint` is optional and takes the form
//! `<joint_name>:<position>:<tolerance>`; see `# Path constraints` below.
//! Prints one NDJSON `plan` request line to stdout; per-run sampling stats
//! to stderr.
//!
//! # Path constraints
//!
//! Phase 7 condition 2 is "every produced path passes `cspace-scene`'s
//! collision check *and constraints*". With no constraint in the problem
//! the constraint half of that condition is vacuously true -- it reports a
//! pass having checked nothing, which is exactly the failure mode that
//! condition is worth testing for. The optional `joint_constraint` argument
//! puts a real [`JointConstraint`] on the problem: it is applied to the
//! endpoint filter here (both endpoints must satisfy it, or the pair is
//! rejected -- a pair that violates it is unsolvable under it, not hard),
//! emitted into the request JSON, enforced during planning by
//! `plan_benchmark_port`, and re-checked on every dense waypoint by
//! `PlanningScene::is_path_valid`.
//!
//! Constrained sets are **port-side only**. The oracle's `plan` op builds
//! its `PlanValidityChecker` from a `planning_scene::PlanningScene`
//! collision check alone and has no constraint input at all (see
//! `oracle.cpp`'s `plan()`), so a constrained problem has no C++ RRTConnect
//! counterpart to compare a success rate or a path length against.
//! Conditions 1 and 3 are therefore measured on the unconstrained set only,
//! and the constrained set carries condition 2's constraint half.

use std::collections::BTreeMap;
use std::env;
use std::fs;
use std::sync::Arc;

use cspace_collision::{CollisionRequest, LinkPaddingScale, ParryCollisionEnv, World};
use cspace_core::geometry::{Cuboid, Isometry3, Shape};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;
use cspace_planners_sbp::{
    CompoundValue, JointModelGroupSpace, PlanningSceneValidityChecker, StateSpace,
    StateValidityChecker,
};
use cspace_planning::constraints::{Constraint, JointConstraint, KinematicConstraintSet};
use cspace_planning::scene::PlanningScene;
use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

/// Every obstacle configuration this generator knows how to build. Kept as a
/// flat list (rather than an enum) so `benches/sweep_baseline.sh` can drive
/// it by name without a matching Rust-side enum to keep in sync.
const CONFIGS: &[&str] = &["empty", "floor", "floor_wall", "slot", "corridor", "cage"];

/// One benchmark robot: the fixture stem, the planning group, its mesh
/// package/directory, and the workspace radius every obstacle box below is
/// scaled against.
struct Robot {
    /// `fixtures/<name>.urdf` / `.srdf`.
    name: &'static str,
    /// The SRDF group planned for.
    group: &'static str,
    /// `package://` name used by this robot's URDF.
    mesh_package: &'static str,
    /// Directory under `fixtures/meshes/` holding that package.
    mesh_dir: &'static str,
    /// Maximum cylindrical radius (`sqrt(x^2 + y^2)`) reached by any link
    /// origin over uniform samples of `group`. **Measured**, not taken from
    /// a datasheet: `panda_arm` 0.9025 and `manipulator` 1.4912, over 4000
    /// uniform samples at seed 7 of every link origin in each model. The
    /// obstacle geometry below is expressed as multiples of this number so
    /// one config name means the same *relative* difficulty on a 7-DoF
    /// 0.9 m arm and a 6-DoF 1.5 m one, rather than the same absolute box
    /// that would be unreachable decoration on the larger robot.
    reach: f64,
}

/// The two benchmark robots. `fixtures/` also holds `pr2` and
/// `dual_arm_panda`; neither is here, and for a stated reason rather than
/// by omission. `dual_arm_panda`'s `left_panda_arm` measures a 2.398 m
/// radius because the group's base is offset from the world origin, so a
/// radius-scaled box centred on the origin does not describe its workspace
/// at all -- the scaling rule this struct encodes is wrong for it, and
/// inventing a second rule for one robot is not worth a third data point.
/// `pr2`'s `right_arm` is likewise shoulder-mounted on a torso rather than
/// origin-mounted. Both are honest exclusions of robots this geometry rule
/// does not fit, not of robots that failed.
const ROBOTS: &[Robot] = &[
    Robot {
        name: "panda",
        group: "panda_arm",
        mesh_package: "moveit_resources_panda_description",
        mesh_dir: "panda_description",
        reach: 0.9025,
    },
    Robot {
        name: "fanuc",
        group: "manipulator",
        mesh_package: "moveit_resources_fanuc_description",
        mesh_dir: "fanuc_description",
        reach: 1.4912,
    },
];

fn robot_by_name(name: &str) -> &'static Robot {
    ROBOTS.iter().find(|r| r.name == name).unwrap_or_else(|| {
        panic!(
            "unknown robot {name:?}, expected one of {:?}",
            robot_names()
        )
    })
}

fn robot_names() -> Vec<&'static str> {
    ROBOTS.iter().map(|r| r.name).collect()
}

/// The mesh package committed under `fixtures/meshes/` -- same pattern as
/// `examples/planning_scene_validity_bench.rs`.
fn fixture_mesh_search_paths(robot: &Robot) -> MeshSearchPaths {
    let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
    MeshSearchPaths::new([(
        robot.mesh_package,
        format!("{meshes_root}/{}", robot.mesh_dir),
    )])
}

fn load_robot(robot: &Robot) -> (RobotModel, SrdfModel) {
    let root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures");
    let name = robot.name;
    let urdf_xml = fs::read_to_string(format!("{root}/{name}.urdf")).unwrap();
    let urdf = urdf_rs::read_from_string(&urdf_xml).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(format!("{root}/{name}.srdf")).unwrap();
    let model =
        RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &fixture_mesh_search_paths(robot))
            .expect("fixture model must build");
    (model, srdf)
}

/// Parses a `<joint_name>:<position>:<tolerance>` argument into a one-member
/// constraint set, symmetric above and below at unit weight. Returns the
/// resolved set alongside the argument string, which is echoed into the
/// request JSON so the port side rebuilds exactly this constraint rather
/// than a re-derived one.
fn parse_joint_constraint(model: &RobotModel, spec: &str) -> KinematicConstraintSet {
    let parts: Vec<&str> = spec.split(':').collect();
    assert_eq!(
        parts.len(),
        3,
        "joint_constraint must be <joint_name>:<position>:<tolerance>, got {spec:?}"
    );
    let position: f64 = parts[1]
        .parse()
        .unwrap_or_else(|e| panic!("joint_constraint position {:?}: {e}", parts[1]));
    let tolerance: f64 = parts[2]
        .parse()
        .unwrap_or_else(|e| panic!("joint_constraint tolerance {:?}: {e}", parts[2]));
    let constraint = JointConstraint::new(model, parts[0], position, tolerance, tolerance, 1.0)
        .unwrap_or_else(|e| panic!("JointConstraint::new({:?}): {e}", parts[0]));
    let mut set = KinematicConstraintSet::new();
    set.push(Constraint::Joint(constraint));
    set
}

/// One `objects[]` entry the oracle's `plan` op reads: a box, by id, full
/// x/y/z extents (matching `shapes::Box`'s own constructor, and
/// [`Cuboid::new`]'s), and world pose.
struct Obstacle {
    id: &'static str,
    size: (f64, f64, f64),
    pose: Isometry3,
}

/// Scales one obstacle's extents and pose by `s`. See [`Robot::reach`] for
/// where `s` comes from and why the geometry is relative at all.
///
/// `s` is `robot.reach / ROBOTS[0].reach`, so for panda -- `ROBOTS[0]` --
/// it is a value divided by itself, which IEEE-754 makes *exactly* `1.0`,
/// and every `x * s` below is exactly `x`. That is the point of scaling by
/// a ratio rather than rewriting the literals: panda's boxes stay
/// bit-identical to the ones the committed C++ baseline in
/// `benches/sweep_baseline.sh` was measured against, so that measurement
/// stays valid, while fanuc's are derived from a measurement rather than
/// guessed.
fn scaled(id: &'static str, size: (f64, f64, f64), xyz: (f64, f64, f64), s: f64) -> Obstacle {
    Obstacle {
        id,
        size: (size.0 * s, size.1 * s, size.2 * s),
        pose: Isometry3::translation(xyz.0 * s, xyz.1 * s, xyz.2 * s),
    }
}

/// A table-top surface just below `panda_link0`'s own mounting origin: a
/// first attempt with the top face flush at `z = 0` made every sample
/// self-collide unconditionally (panda_link0's own collision mesh extends
/// slightly below its origin), which is not a benchmark difficulty lever, it
/// is a bug -- so this leaves a 3 cm clearance instead. Every non-`"empty"`
/// config includes this; a sample that dips the arm itself below the table
/// still self-obstructs, which *is* a real difficulty lever.
fn floor(s: f64) -> Obstacle {
    scaled("floor", (2.0, 2.0, 0.5), (0.0, 0.0, -0.28), s)
}

/// One thin wall on the `+x` side of the robot's reach (panda_arm's own
/// measured extent is 0.9025 m from `panda_link0`; 0.45 m sits inside that
/// so the wall is actually reachable, not decorative).
fn wall(s: f64) -> Obstacle {
    scaled("wall", (0.05, 1.6, 1.6), (0.45, 0.0, 0.8), s)
}

/// Two wall segments on the `+x` side leaving a 0.45 m gap in `y` at
/// `y = 0` -- wide enough to be routinely passable, narrow enough to rule
/// out most of the reachable `+x` volume outright.
fn slot_walls(s: f64) -> [Obstacle; 2] {
    [
        scaled("slot_left", (0.05, 0.65, 1.6), (0.45, -0.55, 0.8), s),
        scaled("slot_right", (0.05, 0.65, 1.6), (0.45, 0.55, 0.8), s),
    ]
}

/// Like [`slot_walls`] but the `y` gap is narrower (0.2 m, not 0.45 m) *and*
/// capped by a ceiling segment -- a true tunnel, not just a doorway, so a
/// solution has to stay both narrow in `y` and low in `z` at once.
fn corridor_walls(s: f64) -> [Obstacle; 3] {
    [
        scaled("corridor_left", (0.05, 0.75, 1.6), (0.45, -0.475, 0.8), s),
        scaled("corridor_right", (0.05, 0.75, 1.6), (0.45, 0.475, 0.8), s),
        scaled("corridor_ceiling", (0.9, 1.7, 0.05), (0.45, 0.0, 1.4), s),
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
fn cage_walls(s: f64) -> [Obstacle; 6] {
    [
        scaled("cage_east_lower", (0.05, 1.2, 0.9), (0.6, -0.45, 0.45), s),
        scaled("cage_east_upper", (0.05, 1.2, 0.4), (0.6, -0.45, 1.4), s),
        scaled("cage_west", (0.05, 1.2, 1.6), (-0.6, 0.0, 0.8), s),
        scaled("cage_north", (1.2, 0.05, 1.6), (0.0, 0.6, 0.8), s),
        scaled("cage_south", (1.2, 0.05, 1.6), (0.0, -0.6, 0.8), s),
        scaled("cage_ceiling", (1.2, 1.2, 0.05), (0.0, 0.0, 1.6), s),
    ]
}

/// Every obstacle for `config` at scale `s`, floor included where
/// applicable. Panics on an unknown name (this binary is not a public
/// library API, so an argument error surfacing as a panic with the bad
/// value in the message is sufficient -- there is no caller to hand a typed
/// error back to).
fn obstacles_for(config: &str, s: f64) -> Vec<Obstacle> {
    match config {
        "empty" => vec![],
        "floor" => vec![floor(s)],
        "floor_wall" => vec![floor(s), wall(s)],
        "slot" => {
            let mut v = vec![floor(s)];
            v.extend(slot_walls(s));
            v
        }
        "corridor" => {
            let mut v = vec![floor(s)];
            v.extend(corridor_walls(s));
            v
        }
        "cage" => {
            let mut v = vec![floor(s)];
            v.extend(cage_walls(s));
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
    let usage = "usage: <config> <count> <seed> [robot] [joint_constraint]";
    let config = args
        .get(1)
        .unwrap_or_else(|| panic!("{usage}; got {args:?}"));
    let count: usize = args
        .get(2)
        .unwrap_or_else(|| panic!("{usage}; got {args:?}"))
        .parse()
        .expect("count must be a non-negative integer");
    let seed: u64 = args
        .get(3)
        .unwrap_or_else(|| panic!("{usage}; got {args:?}"))
        .parse()
        .expect("seed must be a u64");
    let robot = robot_by_name(args.get(4).map_or("panda", String::as_str));
    let constraint_spec = args.get(5).filter(|s| !s.is_empty()).cloned();

    let (model, srdf) = load_robot(robot);
    let group_name = robot.group;
    let space = JointModelGroupSpace::new(&model, group_name)
        .unwrap_or_else(|e| panic!("JointModelGroupSpace::new({group_name}): {e}"));
    let variable_names = model
        .joint_model_group(group_name)
        .unwrap_or_else(|e| panic!("joint_model_group({group_name}): {e}"))
        .variable_names()
        .to_vec();

    // Exactly 1.0 for panda -- see `scaled`'s doc comment.
    let scale = robot.reach / ROBOTS[0].reach;
    let constraints = constraint_spec
        .as_deref()
        .map(|spec| parse_joint_constraint(&model, spec));

    let obstacles = obstacles_for(config, scale);
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
    // The endpoint filter enforces `constraints` too, not just collision: a
    // pair violating the path constraint is unsolvable *under that
    // constraint*, so counting it as a planning failure would measure the
    // sampler, not the planner (this file's own `# Why an endpoint filter
    // at all`, applied to the constraint half).
    let checker = PlanningSceneValidityChecker::new(
        &mut scene,
        &env,
        CollisionRequest::default(),
        constraints.as_ref(),
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
        "robot={} group={group_name} config={config} scale={scale} count={count} \
         attempts={attempts} bad_ep={bad_ep} bad_ep_rate={:.1}% constraint={}",
        robot.name,
        100.0 * bad_ep as f64 / attempts as f64,
        constraint_spec.as_deref().unwrap_or("<none>"),
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

    // `range` (RRT step size) and `motion_resolution` are absolute distances
    // in the space's own metric, so they scale with the robot the same way
    // the obstacles do -- leaving them at panda's absolute values would give
    // fanuc a step size 1.65x finer relative to its own workspace, which is
    // a different search, not the same benchmark on a second robot.
    //
    // `robot` and `joint_constraint` are extra fields the oracle ignores
    // (its `plan` op reads only the keys it knows) and `plan_benchmark_port`
    // reads: the port side must rebuild *this* robot and *this* constraint,
    // not re-derive them from a flag a caller could pass inconsistently.
    let request = serde_json::json!({
        "id": 0,
        "op": "plan",
        "robot": robot.name,
        "group": group_name,
        "seed": seed,
        "config": config,
        "scale": scale,
        "range": 0.05 * scale,
        "motion_resolution": 0.01 * scale,
        "max_iterations": 2000,
        "joint_constraint": constraint_spec,
        "objects": objects_json,
        "problems": problems_json,
    });

    println!("{}", serde_json::to_string(&request).unwrap());
}
