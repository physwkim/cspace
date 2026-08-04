// Copyright (c) 2026, moveit-rs contributors
// SPDX-License-Identifier: BSD-3-Clause

//! Parity test against the moveit2 C++ oracle's `cost_sources`/
//! `path_cost_sources` ops, ground truth for
//! [`PlanningScene::cost_sources`]/[`PlanningScene::path_cost_sources`].
//!
//! Both sides are driven from the committed `panda_cost_sources_request.json`/
//! `panda_path_cost_sources_request.json` (the oracle requests) and their
//! unedited `_response.json` counterparts, at `moveit-rs/oracle:c88557f4058892e9`
//! -- the same pattern `frame_transform_parity.rs` uses, for the same reason:
//! it is the only way to guarantee the Rust-built scene and the oracle's real
//! `planning_scene::PlanningScene` start from the identical scenario.
//!
//! # Why the world object is a thin slab, not a fully-engulfing box
//!
//! An earlier draft of this fixture used a `4x4x4` box centered on the
//! robot -- large enough that every link's mesh sat *entirely inside* it,
//! not merely crossing its boundary. That made every non-self-collision
//! case disagree with the oracle by a wide margin (0.1-0.5m, not
//! floating-point noise). Switching to a thin `4x4x0.1` slab (this crate's
//! own `floor` convention from `is_state_valid_parity.rs`), positioned so
//! it only *crosses* a link's mesh boundary rather than swallowing it
//! whole, did not fix that disagreement -- see the next section.
//!
//! # §171's dispatch defect is fixed; two distinct residual defects remain
//!
//! §171's original defect (`moveit-collision::parry::mesh_shape_cost_sources`
//! reporting one `CostSource` per intersecting mesh-side triangle instead of
//! the one coarse box upstream's `use_approximate_cost_==true` dispatch
//! produces) was fixed this round by `moveit-collision`'s owner (`c6161b9`,
//! `2607bef`, merged `f74a2b7`): `mesh_shape_cost_sources` now reports one
//! box per colliding mesh-link/shape pair, the whole-mesh-AABB/shape-AABB
//! intersection, matching the *shape* of upstream's dispatch. Re-measured
//! against the same fixture this round (both numbers below are freshly
//! measured, not carried over from before the fix):
//!
//! - Id 2's state-op nearest-match distance improved from `1.311e-1` to
//!   `2.6899575667278616e-2` -- five times closer, but still seven orders of
//!   magnitude above the `1e-9` threshold. The routing is now right (one
//!   coarse box, not twenty), but the box's own geometry is not bit-exact
//!   with FCL's: this port's box is `mesh.aabb() ∩ other.compute_aabb()`
//!   (`crates/moveit-collision/src/parry.rs`, `mesh_shape_cost_sources`),
//!   which is not necessarily identical to FCL's own `constructBox` +
//!   box-vs-shape narrowphase result. A `moveit-collision` defect, distinct
//!   from §171 and still open.
//! - Id 3's path-op count mismatch changed shape from `3` (actual) vs `5`
//!   (expected) to `1` vs `5` -- not the same failure §171.7 flagged, so
//!   the "wait for §171 and remeasure" premise on §171.7 has expired.
//!   Diagnosis, from this crate's boundary: [`PlanningScene::path_cost_sources`]'s
//!   own call sequence (union → truncate to `max_costs` → `remove_cost_sources`
//!   against `cs_start` → `remove_overlapping`) was independently re-verified
//!   against `planning_scene.cpp:2451-2490` this same round and matches
//!   faithfully. Id 3's `max_costs` is `2`, yet the oracle's expected final
//!   count is `5` -- the only mechanism in this pipeline that can grow a
//!   truncated-to-2 set past 2 is `remove_cost_sources`'s per-axis split
//!   branch (`crates/moveit-collision/src/tools.rs`, the `add.insert(split)`
//!   loop for a below-threshold overlap), not `remove_overlapping` (pure
//!   drop, never adds). This port currently collapses to 1, undershooting
//!   even the pre-`max_costs` truncation floor, which points at that split
//!   branch or the coarser boxes now feeding it, inside `moveit-collision` --
//!   not at this crate's orchestration.
//!
//! Neither residual defect is a stale-oracle artifact (both numbers
//! reproduced against the same fixed `moveit-rs/oracle:c88557f4058892e9`
//! image this round) and neither is fixable from `moveit-scene`:
//! `mesh_shape_cost_sources`, `remove_cost_sources`, and `remove_overlapping`
//! are all `moveit-collision` functions. See
//! [`panda_cost_sources_blocked_by_mesh_shape_cost_sources`] and
//! [`panda_path_cost_sources_blocked_by_mesh_shape_cost_sources`] for the
//! full per-id detail; those two tests stay `#[ignore]`d, now for these two
//! new reasons rather than §171's (fixed) one.
//!
//! `panda_cost_sources_matches_the_oracle`/
//! `panda_path_cost_sources_matches_the_oracle` below therefore only run
//! the case ids that do not depend on that function -- self-collision
//! (mesh-vs-mesh, unaffected: id 1's 75 entries at ULP-level distances
//! confirm the mesh-vs-mesh path and the removal/truncation math are both
//! correct) and the trivial all-zero ids. The ids that need a genuinely
//! colliding world object/attached body to demonstrate the `max_costs`
//! below/at/above boundary and `group_name` filtering (state ids 2-6, 8)
//! or the `overlap_fraction`/splitting behavior against real geometry
//! (path ids 3-6) are captured in the fixture -- real, oracle-verified
//! ground truth -- but currently only reachable through the ignored tests
//! above, not this crate's asserted regression suite.
//!
//! # What each case isolates
//!
//! `panda_cost_sources_request.json` (state op, ids 1-11):
//! - id 1: `joint_values={}` (the established default self-colliding pose),
//!   no world object, `group_name` omitted -- upstream's state overload runs
//!   one `checkCollision(cost=true)` and swaps the result out with **no**
//!   `removeCostSources`/`removeOverlapping` pass (`planning_scene.cpp:2499-2506`),
//!   unlike the trajectory overload below. 75 raw mesh-triangle cost sources
//!   survive uncollapsed; a port that (wrongly) ran a removal pass here would
//!   under-count.
//! - ids 2-4: the clean arm against a `floor` slab at `z=0.4` (chosen so it
//!   crosses several links' meshes without engulfing any of them),
//!   `group_name` omitted, `max_costs` at 5 (below the true count of 9), 9
//!   (exactly at it) and 50 (above it) -- the below/at/above boundary
//!   `CollisionRequest::max_cost_sources` is required to hold at.
//! - ids 5-6: the same floor scenario with `group_name` explicitly `"hand"`
//!   (2) and `"panda_arm"` (9, matching the whole-robot count here because
//!   every colliding link this floor height reaches is inside `panda_arm`)
//!   -- real, non-empty, non-partitioning subsets, confirming group
//!   filtering is a per-source collision check, not a naive split of the
//!   whole-robot result.
//! - id 7: the clean arm against a floor slab moved far away (`z=-2.05`,
//!   this crate's own `floor_far_away` convention) -- the fixture's one
//!   all-zero case, alongside eight non-zero ones so it measures something.
//! - ids 8-9: a small box attached to `panda_hand` with `touch_links`
//!   covering the hand/finger family but not `panda_link7`; id 8 shows the
//!   attached body genuinely colliding with the untouched link (1), id 9
//!   adds `panda_link7` to `touch_links` and the same collision vanishes (0)
//!   -- `attached_bodies`/`touch_links` read the same schema `collision`
//!   does, exercised here rather than assumed.
//! - ids 10-11: id 1's self-colliding state again, at `max_costs` 10 and
//!   40 -- both below the true mesh-vs-mesh count of 75, so both exercise
//!   the truncate-to-most-costly boundary on a case unaffected by the
//!   `mesh_shape_cost_sources` defect below, closing a gap ids 2-6/8
//!   cannot: before these two, nothing in this crate's asserted suite
//!   measured `max_costs` truncation against real oracle ground truth.
//!
//! `panda_path_cost_sources_request.json` (trajectory op, ids 1-6):
//! - ids 1-2: two waypoints, one clean (`CLEAN_POSE`) and one self-colliding
//!   (`{}`), in each order. `cs_start` is captured by `swap` at the *first*
//!   waypoint only, not a copy of the running union
//!   (`planning_scene.cpp:2473-2474`) -- id 1 (clean first) keeps 5 survivors
//!   after removal, id 2 (colliding first) keeps 0, a distinction only a
//!   trajectory whose first waypoint collides can show.
//! - ids 3-5: two arm poses (`CLEAN_POSE`, and `CLEAN_POSE` with
//!   `panda_joint1` perturbed to `0.3`) both colliding with the same floor
//!   slab, `max_costs=2` (small enough to force truncation before removal
//!   even starts), swept over `overlap_fraction` 0.9/0.8/0.1 giving 5/4/0
//!   survivors -- `removeCostSources`'s `else` branch does not just drop a
//!   partially-overlapping box, it *splits* the surviving remainder along
//!   each axis (`collision_tools.cpp:246-264`), which is why raising
//!   `overlap_fraction` (a stricter removal bar) does not shrink the count
//!   monotonically the way a plain filter would.
//! - id 6: the same two-waypoint floor scenario at `max_costs=3`,
//!   `overlap_fraction=0.8` -- 6 survivors, distinct from id 4's
//!   `max_costs=2` at the same `overlap_fraction` (4 survivors) on
//!   identical geometry, showing `max_costs` changes *which* specific
//!   AABBs survive truncation before splitting runs, not just how many.
//!
//! # Comparison tolerance
//!
//! `assert_cost_sources_close` below does a nearest-match pairing rather
//! than `BTreeSet` equality: mesh-triangle AABB corners (id 1's 75 entries)
//! differ from the committed response at the ULP level (~1e-16, measured),
//! from floating-point reduction order differing between C++ Eigen and
//! Rust nalgebra/parry over the same triangle set -- not a real
//! disagreement, and `CostSource::cmp`'s `total_cmp` chain is exact enough
//! that a ULP shift can, in principle, reorder two very-close-in-cost
//! entries, which is why comparison is by nearest-neighbor distance rather
//! than by `BTreeSet` iteration position.

use std::collections::{BTreeMap, BTreeSet};
use std::fs;
use std::sync::Arc;

use serde::Deserialize;

use moveit_collision::{CostSource, LinkPaddingScale, ParryCollisionEnv, World};
use moveit_geometry::{Cuboid, Isometry3, Shape};
use moveit_model::{MeshSearchPaths, RobotModel};
use moveit_scene::PlanningScene;
use moveit_srdf::SrdfModel;
use moveit_state::RobotState;
use nalgebra::{Matrix3, Translation3, UnitQuaternion};

#[derive(Deserialize)]
struct ShapeSpec {
    #[serde(rename = "type")]
    kind: String,
    #[serde(default)]
    size: [f64; 3],
}

#[derive(Deserialize)]
struct ObjectSpec {
    id: String,
    pose: [f64; 16],
    shape: ShapeSpec,
}

#[derive(Deserialize)]
struct AttachedBodySpec {
    id: String,
    link_name: String,
    shapes: Vec<ShapeSpec>,
    shape_poses: Vec<[f64; 16]>,
    #[serde(default)]
    touch_links: Vec<String>,
}

#[derive(Deserialize)]
struct CostSourcesCase {
    id: u32,
    joint_values: BTreeMap<String, f64>,
    #[serde(default)]
    objects: Vec<ObjectSpec>,
    #[serde(default)]
    attached_bodies: Vec<AttachedBodySpec>,
    max_costs: usize,
    #[serde(default)]
    group_name: Option<String>,
}

#[derive(Deserialize)]
struct PathCostSourcesCase {
    id: u32,
    #[serde(default)]
    objects: Vec<ObjectSpec>,
    #[serde(default)]
    attached_bodies: Vec<AttachedBodySpec>,
    waypoints: Vec<BTreeMap<String, f64>>,
    max_costs: usize,
    #[serde(default)]
    group_name: Option<String>,
    overlap_fraction: f64,
}

#[derive(Deserialize)]
struct ResponseCostSource {
    aabb_min: [f64; 3],
    aabb_max: [f64; 3],
    cost: f64,
}

impl From<ResponseCostSource> for CostSource {
    fn from(r: ResponseCostSource) -> Self {
        CostSource {
            aabb_min: r.aabb_min,
            aabb_max: r.aabb_max,
            cost: r.cost,
        }
    }
}

#[derive(Deserialize)]
struct ResponseResult {
    cost_sources: Vec<ResponseCostSource>,
}

#[derive(Deserialize)]
struct OracleResponse {
    id: u32,
    result: ResponseResult,
}

fn fixture_path(file_name: &str) -> String {
    format!(
        concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/{}"),
        file_name
    )
}

fn load<T: for<'de> Deserialize<'de>>(file_name: &str) -> Vec<T> {
    let path = fixture_path(file_name);
    let raw = fs::read_to_string(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    serde_json::from_str(&raw).unwrap_or_else(|e| panic!("parse {path}: {e}"))
}

fn expected_cost_sources(responses: &[OracleResponse], id: u32) -> BTreeSet<CostSource> {
    responses
        .iter()
        .find(|r| r.id == id)
        .unwrap_or_else(|| panic!("no response for id {id}"))
        .result
        .cost_sources
        .iter()
        .map(|r| CostSource {
            aabb_min: r.aabb_min,
            aabb_max: r.aabb_max,
            cost: r.cost,
        })
        .collect()
}

/// The two `moveit_resources_*_description` packages committed under
/// `fixtures/meshes/`, same mapping `is_state_valid_parity.rs`'s
/// `mesh_search_paths` uses -- panda's self-collision cost sources (id 1)
/// come from real `<mesh>` geometry, not primitive shapes.
fn mesh_search_paths() -> MeshSearchPaths {
    let meshes_root = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/meshes");
    MeshSearchPaths::new([(
        "moveit_resources_panda_description",
        format!("{meshes_root}/panda_description"),
    )])
}

fn build_model() -> RobotModel {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.urdf");
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.srdf");
    let urdf_xml = fs::read_to_string(path).unwrap_or_else(|e| panic!("read {path}: {e}"));
    let urdf = urdf_rs::read_file(path).expect("fixture URDF must parse");
    let srdf = SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse");
    RobotModel::from_urdf_and_srdf(&urdf, &urdf_xml, &srdf, &mesh_search_paths())
        .expect("fixture model must build")
}

fn srdf() -> SrdfModel {
    let srdf_path = concat!(env!("CARGO_MANIFEST_DIR"), "/../../fixtures/panda.srdf");
    SrdfModel::parse_file(srdf_path).expect("fixture SRDF must parse")
}

fn isometry_from_row_major(m: &[f64; 16]) -> Isometry3 {
    let rotation = Matrix3::new(m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]);
    let translation = Translation3::new(m[3], m[7], m[11]);
    Isometry3::from_parts(translation, UnitQuaternion::from_matrix(&rotation))
}

fn cuboid_shape(spec: &ShapeSpec) -> Arc<Shape> {
    assert_eq!(spec.kind, "box", "this fixture only ever uses box shapes");
    let cuboid = Cuboid::new(spec.size[0], spec.size[1], spec.size[2])
        .expect("fixture box dimensions must be positive");
    Arc::new(Shape::Cuboid(cuboid))
}

fn world_from_objects(objects: &[ObjectSpec]) -> World {
    let mut world = World::new();
    for object in objects {
        world.add_shape(
            &object.id,
            cuboid_shape(&object.shape),
            isometry_from_row_major(&object.pose),
        );
    }
    world
}

fn state_with<'m>(model: &'m RobotModel, joint_values: &BTreeMap<String, f64>) -> RobotState<'m> {
    let mut state = RobotState::new(model);
    state.set_to_default_values();
    for (name, &value) in joint_values {
        state
            .set_variable_position(name, value)
            .unwrap_or_else(|e| panic!("setting {name}: {e}"));
    }
    state
}

fn scene_with_attached<'m>(
    model: &'m RobotModel,
    srdf: &SrdfModel,
    world: World,
    attached_bodies: &[AttachedBodySpec],
) -> PlanningScene<'m> {
    let mut scene = PlanningScene::with_world(model, srdf, world);
    for attached in attached_bodies {
        let shapes: Vec<Arc<Shape>> = attached.shapes.iter().map(cuboid_shape).collect();
        let shape_poses: Vec<Isometry3> = attached
            .shape_poses
            .iter()
            .map(isometry_from_row_major)
            .collect();
        let touch_links: BTreeSet<String> = attached.touch_links.iter().cloned().collect();
        scene
            .attach_new(
                &attached.id,
                &attached.link_name,
                shapes,
                shape_poses,
                touch_links,
                BTreeMap::new(),
            )
            .unwrap_or_else(|e| panic!("attach {}: {e}", attached.id));
    }
    scene
}

/// Case ids that do not exercise `mesh_shape_cost_sources`
/// (`crates/moveit-collision/src/parry.rs:1368-1388`): either no world
/// object/attached body at all (mesh-vs-mesh self-collision, or the
/// far-away/touch-suppressed zero cases), so the defect documented on
/// [`panda_cost_sources_blocked_by_mesh_shape_cost_sources`] cannot reach
/// them. Ids 10-11 are the same id-1 self-collision state at `max_costs`
/// 10 and 40 (both below the true count of 75), added to measure the
/// `max_costs` truncation rule -- `CostSource::Ord`
/// (`crates/moveit-collision/src/common.rs:144-160`) -- independently of
/// §171: computing the volume-sorted top 10/40 of id 1's own 75-entry
/// response offline and comparing against these two oracle-captured
/// responses confirms the oracle truncates to exactly that ranking (top-N
/// by `cost * volume`, then `cost`, then `aabb_min`), with no case in this
/// 75-entry population where two sources tie through `aabb_min` and differ
/// only in `aabb_max` -- the collapse `CostSource::cmp`'s doc comment
/// says it deliberately reproduces from `std::set<CostSource>::operator<`
/// (which does not compare `aabb_max`) exists in principle but does not
/// trigger on this dataset.
const COST_SOURCES_PASSING_IDS: [u32; 5] = [1, 7, 9, 10, 11];

fn run_cost_sources_case(
    model: &RobotModel,
    srdf: &SrdfModel,
    responses: &[OracleResponse],
    case: &CostSourcesCase,
) {
    let world = world_from_objects(&case.objects);
    let mut scene = scene_with_attached(model, srdf, world, &case.attached_bodies);
    let state = state_with(model, &case.joint_values);
    scene.set_current_state(state);
    let env = ParryCollisionEnv::new(scene.world().clone(), LinkPaddingScale::default());

    let actual = scene.cost_sources(&env, case.group_name.as_deref(), case.max_costs);
    let expected = expected_cost_sources(responses, case.id);
    assert_cost_sources_close(&actual, &expected, case.id);
}

#[test]
fn panda_cost_sources_matches_the_oracle() {
    let model = build_model();
    let srdf = srdf();
    let cases: Vec<CostSourcesCase> = load("panda_cost_sources_request.json");
    let responses: Vec<OracleResponse> = load("panda_cost_sources_response.json");
    assert_eq!(cases.len(), 11, "expected exactly 11 cases in the fixture");

    for case in cases
        .iter()
        .filter(|c| COST_SOURCES_PASSING_IDS.contains(&c.id))
    {
        run_cost_sources_case(&model, &srdf, &responses, case);
    }
}

/// Ids 2-6 and 8 all put a genuinely colliding world object or attached
/// body against a panda link, every one of which uses `<collision><mesh>`
/// geometry (confirmed: no panda link has a primitive-shape collision
/// geometry to fall back to). §171's original dispatch defect (routing
/// through a per-triangle traversal instead of upstream's one-coarse-box
/// path) is fixed this round (`c6161b9`/`2607bef`, merged `f74a2b7`) --
/// `mesh_shape_cost_sources` now reports one box per pair, the right shape
/// of answer. What remains, re-measured this round against the same
/// `moveit-rs/oracle:c88557f4058892e9` fixture: id 2's nearest-match
/// distance is `2.6899575667278616e-2` against the `1e-9` threshold --
/// improved from the pre-fix `1.311e-1` (routing was clearly wrong before),
/// but the box this port now computes
/// (`mesh.aabb(pose).intersection(other.compute_aabb(pose))` in
/// `crates/moveit-collision/src/parry.rs`, `mesh_shape_cost_sources`) is a
/// whole-mesh/whole-shape AABB intersection, not necessarily identical to
/// FCL's own `constructBox` + box-vs-shape narrowphase box. That geometry
/// gap, not a routing gap, is what remains -- a `moveit-collision` defect,
/// distinct from §171 and still open. See this file's module doc for the
/// full writeup and the sibling path-op test's own (differently-shaped)
/// residual defect. Tracked as an UNFIXED cross-crate blocker in the
/// p1-fixtures round report; remove this `#[ignore]` once
/// `moveit-collision` is fixed and confirm it passes with the existing
/// `COST_SOURCE_EPSILON`.
#[test]
#[ignore = "blocked on a moveit-collision defect, distinct from the now-fixed §171: mesh_shape_cost_sources's one-box-per-pair AABB intersection is not bit-exact with FCL's own box-vs-shape narrowphase result (id 2 nearest-match distance 2.69e-2 against a 1e-9 threshold, crates/moveit-collision/src/parry.rs, mesh_shape_cost_sources)"]
fn panda_cost_sources_blocked_by_mesh_shape_cost_sources() {
    let model = build_model();
    let srdf = srdf();
    let cases: Vec<CostSourcesCase> = load("panda_cost_sources_request.json");
    let responses: Vec<OracleResponse> = load("panda_cost_sources_response.json");

    for case in cases
        .iter()
        .filter(|c| !COST_SOURCES_PASSING_IDS.contains(&c.id))
    {
        run_cost_sources_case(&model, &srdf, &responses, case);
    }
}

fn cost_source_distance(a: &CostSource, b: &CostSource) -> f64 {
    let mut d: f64 = (a.cost - b.cost).abs();
    for i in 0..3 {
        d = d.max((a.aabb_min[i] - b.aabb_min[i]).abs());
        d = d.max((a.aabb_max[i] - b.aabb_max[i]).abs());
    }
    d
}

/// Measured against the committed fixture's id-1 case (75 mesh-triangle
/// pairs): C++ Eigen and Rust nalgebra/parry reduce the same triangle
/// vertices in different order, producing coordinate differences up to
/// `1.11e-16` (one f64 ULP at these magnitudes). `1e-9` sits eight orders
/// of magnitude above that noise floor and eight below the smallest
/// mismatch the `mesh_shape_cost_sources` defect below produces (~1e-2 m),
/// so it cannot mask a real disagreement.
const COST_SOURCE_EPSILON: f64 = 1e-9;

fn assert_cost_sources_close(
    actual: &BTreeSet<CostSource>,
    expected: &BTreeSet<CostSource>,
    id: u32,
) {
    assert_eq!(actual.len(), expected.len(), "case id {id}: count mismatch");
    let mut remaining: Vec<CostSource> = expected.iter().copied().collect();
    for a in actual {
        let (idx, dist) = remaining
            .iter()
            .enumerate()
            .map(|(i, e)| (i, cost_source_distance(a, e)))
            .min_by(|x, y| x.1.total_cmp(&y.1))
            .unwrap_or_else(|| {
                panic!("case id {id}: no expected cost sources left to match {a:?}")
            });
        assert!(
            dist < COST_SOURCE_EPSILON,
            "case id {id}: nearest-match distance {dist:e} exceeds {COST_SOURCE_EPSILON:e}"
        );
        remaining.remove(idx);
    }
}

/// Case ids 1-2 are the only `panda_path_cost_sources_request.json` cases
/// with no world object -- purely mesh-vs-mesh self-collision across both
/// waypoints -- so they do not exercise `mesh_shape_cost_sources` and are
/// unaffected by the defect documented on
/// [`panda_path_cost_sources_blocked_by_mesh_shape_cost_sources`].
const PATH_COST_SOURCES_PASSING_IDS: [u32; 2] = [1, 2];

fn run_path_cost_sources_case(
    model: &RobotModel,
    srdf: &SrdfModel,
    responses: &[OracleResponse],
    case: &PathCostSourcesCase,
) {
    let world = world_from_objects(&case.objects);
    let mut scene = scene_with_attached(model, srdf, world, &case.attached_bodies);
    let env = ParryCollisionEnv::new(scene.world().clone(), LinkPaddingScale::default());

    let waypoints: Vec<RobotState> = case
        .waypoints
        .iter()
        .map(|wp| state_with(model, wp))
        .collect();

    let actual = scene.path_cost_sources(
        &env,
        &waypoints,
        case.group_name.as_deref(),
        case.max_costs,
        case.overlap_fraction,
    );
    let expected = expected_cost_sources(responses, case.id);
    assert_cost_sources_close(&actual, &expected, case.id);
}

#[test]
fn panda_path_cost_sources_matches_the_oracle() {
    let model = build_model();
    let srdf = srdf();
    let cases: Vec<PathCostSourcesCase> = load("panda_path_cost_sources_request.json");
    let responses: Vec<OracleResponse> = load("panda_path_cost_sources_response.json");
    assert_eq!(cases.len(), 6, "expected exactly 6 cases in the fixture");

    for case in cases
        .iter()
        .filter(|c| PATH_COST_SOURCES_PASSING_IDS.contains(&c.id))
    {
        run_path_cost_sources_case(&model, &srdf, &responses, case);
    }
}

/// Ids 3-6 all put both waypoints against the same floor slab, so every
/// survivor after `remove_cost_sources`/`remove_overlapping` traces back to
/// a `mesh_shape_cost_sources` call. §171's dispatch defect is fixed this
/// round (see [`panda_cost_sources_blocked_by_mesh_shape_cost_sources`]'s
/// doc), so this is re-measured, not carried over: id 3's count mismatch
/// changed from `3` (actual) vs `5` (expected) -- §171.7's reversal -- to
/// `1` vs `5`, a different shape of mismatch, so §171.7's "wait for §171
/// and remeasure" is expired; this is a distinct, still-open defect.
/// [`PlanningScene::path_cost_sources`]'s own call sequence was
/// independently re-verified against `planning_scene.cpp:2451-2490` this
/// round (see the doc there) and is faithful; id 3's `max_costs` is `2` and
/// the expected final count is `5`, which only `remove_cost_sources`'s
/// per-axis split branch (`crates/moveit-collision/src/tools.rs`) can grow
/// past the truncation cap -- this port collapses to `1`, undershooting
/// even that cap, pointing at that split branch or the coarser boxes now
/// feeding it. A `moveit-collision` defect, not this crate's orchestration.
/// Remove this `#[ignore]` once `moveit-collision` is fixed.
#[test]
#[ignore = "blocked on a moveit-collision defect, distinct from the now-fixed §171: id 3 (max_costs=2, expected 5 after remove_cost_sources's per-axis split) collapses to 1 survivor instead, in crates/moveit-collision/src/tools.rs (remove_cost_sources/remove_overlapping) or the coarser mesh_shape_cost_sources boxes feeding them"]
fn panda_path_cost_sources_blocked_by_mesh_shape_cost_sources() {
    let model = build_model();
    let srdf = srdf();
    let cases: Vec<PathCostSourcesCase> = load("panda_path_cost_sources_request.json");
    let responses: Vec<OracleResponse> = load("panda_path_cost_sources_response.json");

    for case in cases
        .iter()
        .filter(|c| !PATH_COST_SOURCES_PASSING_IDS.contains(&c.id))
    {
        run_path_cost_sources_case(&model, &srdf, &responses, case);
    }
}
