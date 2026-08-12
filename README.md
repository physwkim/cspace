# cspace

A pure-Rust port of [MoveIt 2](https://github.com/moveit/moveit2)'s motion
planning core, with no ROS dependency. Robot model, collision checking,
constraints, planning scene, IK and four planners are ordinary Rust libraries:
they load URDF/SRDF from files, are driven by function calls, and are tested
with `cargo test` on a machine with no ROS 2 installed.

ROS 2 interop exists, but only as one optional crate outside the workspace
(`ros/cspace-ros`), so nothing in the core can reach `r2r` — a CI gate enforces
the direction.

Correctness is not asserted against hand-written expectations. Every phase is
measured differentially against upstream C++: `tools/moveit-oracle` is a C++
binary linked against MoveIt 2 itself, and `tools/moveit-diff` feeds both
implementations the same inputs and compares the answers.

## Crates

| Crate | Contents |
|---|---|
| `cspace-core` | `error`, `geometry`, `octomap`, `srdf`, `model`, `state`, `kinematics`, `sampling`, `trajectory`, `smoothing`, `metrics`, `test_support` |
| `cspace-collision` | collision environment — `parry` for the discrete check, `bullet_ccd` for the two-state continuous one — allowed-collision matrix, `distance_field` |
| `cspace-bullet` | Bullet's convex narrow phase: support functions, Voronoi simplex, GJK, EPA, the `dbvt` broadphase, manifold |
| `cspace-bullet-cast` | MoveIt's Bullet continuous-collision layer: `cast_hull_shape`, the cast broadphase manager, cast contact conversion |
| `cspace-planning` | planning request/response, pipeline and adapters, `constraints`, `scene`, `planner_registry` |
| `cspace-planners` | `sbp` (RRT-Connect and state spaces), `chomp`, `pilz`, `stomp` |
| `cspace-stomp-core` | the ROS-independent STOMP optimizer loop |

Twenty of those module names were separate packages at one point; the split was
a build-time boundary rather than a design one, and the module names are what
the former crate names became. The two Bullet crates are not from that fold —
they arrived later, and each tracks an upstream of its own.

`ros/cspace-ros` is its own workspace: it needs `r2r` and a local ROS 2 install
at build time, which neither this repo's CI nor a plain `cargo build --workspace`
has.

## Scope

Ported: `moveit_core`, `moveit_kinematics`, and the `pilz`, `chomp` and `stomp`
planners from `moveit_planners`. Sampling-based planning is not a port —
upstream delegates it to OMPL, which has no Rust equivalent, so `cspace-planners`'
`sbp` module is original work following the published RRT-Connect algorithm.

Not ported: `moveit_ros` (partially covered by `ros/cspace-ros`),
`moveit_setup_assistant`, `moveit_py`, `moveit_plugins`. FCL is replaced by
[`parry`](https://github.com/dimforge/parry) rather than ported;
`collision_detection_bullet` is the other way round, ported rather than
replaced, so a discrete check is parry's answer and the two-state continuous
check is Bullet's own GJK/EPA.

Plugins are resolved at compile time through a `linkme` registry rather than
`pluginlib`, so a planner is linked in by depending on the crate that declares
it.

## Using it

Abridged from `cspace-planning`'s crate-level doctest, which is the compiled
version of this:

```rust
use cspace_collision::ParryCollisionEnv;
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;
use cspace_planning::constraints::utils::construct_goal_joint_constraints;
use cspace_planning::planner_registry::resolve_planner;
use cspace_planning::scene::PlanningScene;
use cspace_planning::{PlannerConfigurationMap, PlanningRequest, generate_plan};

// Linked for its side effect: `RrtConnectManager` registers itself into
// `PLANNER_MANAGERS` through a `linkme` static, and nothing below names a
// `cspace_planners` item. Without this the linker drops the registration.
use cspace_planners as _;

let urdf_xml = std::fs::read_to_string("fixtures/panda.urdf")?;
let urdf = urdf_rs::read_from_string(&urdf_xml)?;
let srdf = SrdfModel::parse_file("fixtures/panda.srdf")?;
let model = RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &MeshSearchPaths::none())?;

let mut scene = PlanningScene::new(&model, &srdf);
let env = ParryCollisionEnv::default();

let mut goal_state = RobotState::new(&model);
goal_state.set_to_default_values();
goal_state.set_joint_positions("panda_joint1", &[0.4])?;
let goal =
    construct_goal_joint_constraints(&model, &goal_state.update(), "panda_arm", 0.0, 0.0)?;

let request = PlanningRequest {
    group_name: "panda_arm".to_string(),
    goal_constraints: vec![goal],
    ..PlanningRequest::default()
};

let planner = resolve_planner("rrt_connect", &PlannerConfigurationMap::new())?;
let response = generate_plan(&mut scene, &env, &[], &[planner], &[], request)?;
```

## Status

Every condition in the phase table in [`GOALS.md`](GOALS.md) is met as of the
2026-08-11 measurement: `doc/phase7-benchmark-results.json` 39/39 and
`doc/phase8-optimizer-properties.json` 140/140. Those record the CHOMP and
STOMP optimizers measured against their own upstream C++ implementations.

A second Phase 8 instrument, `tools/ci/verify-phase8-benchmark.sh`, measures the
same three properties against the C++ OMPL RRTConnect baseline instead and ends
QUALIFIED: four of its six conditions are UNMET, because success rate compares a
trajectory optimizer to a tree-growing sampler (CHOMP 380/500, STOMP 441/500,
against a 89.64% bar). Each UNMET has its reasoning in that script's header.

`cargo nextest run --workspace` is 2,596 tests, measured on `9cd8eaba`.

## Building and testing

Rust 1.85, edition 2024. No ROS 2, no system MoveIt, no C++ toolchain needed
for the workspace itself.

```sh
cargo fmt --all
cargo clippy --workspace --all-targets -- -D warnings
cargo nextest run --workspace
cargo test --doc --workspace

tools/ci/check-*.sh     # behaviour gates; no docker, run by CI on every push
tools/ci/verify-*.sh    # differential sweeps; most need the C++ oracle image
```

The oracle image is built by `tools/moveit-oracle/build.sh` and is not part of
the per-push CI job, because building it takes long enough to be a nightly.

## Upstream and licensing

Upstream reference: `moveit2` @ `e017c91ee12984393a28ba246075c65f69cde3bf`.
Every ported file keeps its original copyright header and names the upstream
path it came from, at the top of the file.

BSD-3-Clause, matching MoveIt 2, except where a crate ports an upstream under
other terms — each of those sets its license explicitly rather than inheriting
the workspace's, and carries the text beside its manifest:

| crate | license | upstream |
| --- | --- | --- |
| `cspace-core`, `cspace-collision`, `cspace-planning` | BSD-3-Clause | moveit2 |
| `cspace-planners` | BSD-3-Clause AND Apache-2.0 | moveit2; `pilz_industrial_motion_planner`'s vendored `joint_limits_copy/joint_limits.hpp` is PAL Robotics' and Apache-2.0 |
| `cspace-bullet` | Zlib | [bullet3](https://github.com/bulletphysics/bullet3) @ `7dee3436e747958e7088dfdcea0e4ae031ce619e` (tag 3.24) |
| `cspace-bullet-cast` | BSD-2-Clause | moveit2's `collision_detection_bullet/bullet_integration/` |
| `cspace-stomp-core` | Apache-2.0 | [ros-industrial/stomp](https://github.com/ros-industrial/stomp) @ `b1a87c80f7338caae25a5c689b876da15492aa75` |

`tools/ci/check-license-matches-upstream.sh` derives that table from the tree
on every CI run and fails if a manifest names fewer terms than its own sources
impose, so it cannot drift from this file silently.
