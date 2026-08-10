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
//! # §171's dispatch defect is fixed; the path-op split-branch residual has closed too
//!
//! §171's original defect (`cspace-collision::parry::mesh_shape_cost_sources`
//! reporting one `CostSource` per intersecting mesh-side triangle instead of
//! the one coarse box upstream's `use_approximate_cost_==true` dispatch
//! produces) was fixed by `cspace-collision`'s owner (`c6161b9`, `2607bef`,
//! merged `f74a2b7`): `mesh_shape_cost_sources` now reports one box per
//! colliding mesh-link/shape pair, matching the *shape* of upstream's
//! dispatch. That fix left two residual defects on record (id 2's state-op
//! box-geometry gap at `2.6899575667278616e-2` against the `1e-9`
//! threshold; id 3's path-op count mismatch, `1` actual vs `5` expected). A
//! second, distinct `cspace-collision` fix has since closed both: fitting
//! the mesh cost-source root box as an oriented OBB rather than an
//! axis-aligned AABB (`54250b1`). Re-measured this round, not carried over
//! from that fix's own report:
//!
//! - State-op id 2's nearest-match distance is now
//!   `6.661338147750939e-16` (measured directly: `cargo nextest run -p
//!   cspace-scene --test cost_sources_parity
//!   panda_cost_sources_blocked_by_mesh_shape_cost_sources --run-ignored all
//!   --no-capture` with a temporary `eprintln!`, output discarded, not
//!   committed) -- an ULP-level gap indistinguishable from id 1's
//!   already-passing mesh-vs-mesh noise floor, eight orders of magnitude
//!   under the `1e-9` threshold, not a real disagreement.
//! - Path-op ids 3-6 (the split-branch cases) now pass outright, measured
//!   the same way with `--run-ignored all`:
//!   [`panda_path_cost_sources_blocked_by_mesh_shape_cost_sources`] is
//!   green, so its `#[ignore]` is removed below -- see "Oracle-verified
//!   coverage for the split branch" further down for what ids 4/6 prove,
//!   now exercised rather than only reasoned about.
//!
//! A third state-op case, id 5 (`group_name="hand"`, `9` actual vs `2`
//! expected -- a count mismatch the OBB fit does not touch, since that fit
//! changes a box's *geometry*, not how many boxes group filtering keeps),
//! is closed too, but not by this crate: `group_name` filtering was
//! entirely unimplemented in `cspace-collision`'s
//! `check_self_collision`/`check_robot_collision`/`distance_self`/
//! `distance_robot`, which received the field and never read it, against a
//! module doc there that claimed the omission matched upstream. It did
//! not -- `cspace-collision`'s owner fixed it in `585a79e`. See
//! [`panda_cost_sources_blocked_by_mesh_shape_cost_sources`] for the
//! isolation measurement and `doc/claim-audit/cspace-scene.md` for why the
//! earlier attribution of this defect to `PlanningScene::cost_sources` was
//! wrong.
//!
//! # Oracle-verified coverage for the split branch (path ids 4, 6)
//!
//! [`PlanningScene::path_cost_sources`] truncates `cs` to `max_costs`
//! *before* calling `remove_cost_sources`, and `remove_overlapping` (the
//! only stage after it) never adds -- it only drops a later entry that
//! overlaps an earlier one past `overlap_fraction`. So a final count
//! *larger* than `max_costs` is only reachable if `remove_cost_sources`'s
//! per-axis split branch fired and added split pieces. Two ids in this same
//! fixture are exactly that, oracle-sourced and unambiguous: id 4
//! (`max_costs=2`, oracle's expected final count `4`) and id 6
//! (`max_costs=3`, oracle's expected final count `6`) -- both mathematically
//! impossible under a pure-truncate-then-drop pipeline, so both are direct
//! proof the split branch is real and expected to grow the set past the
//! cap. Previously reasoned about while gated behind an `#[ignore]` this
//! fixture already carried; now exercised directly, not merely reasoned
//! about, by [`panda_path_cost_sources_blocked_by_mesh_shape_cost_sources`]
//! passing.
//!
//! `panda_cost_sources_matches_the_oracle` below only runs the state-op
//! case ids that were already passing before this round's fixes --
//! self-collision (mesh-vs-mesh, unaffected: id 1's 75 entries at
//! ULP-level distances confirm the mesh-vs-mesh path and the
//! removal/truncation math are both correct) and the trivial all-zero
//! ids. `panda_path_cost_sources_matches_the_oracle` covers ids 1-2 the
//! same way; every other path id (3-6, the split-branch cases) is now
//! exercised, unconditionally, by
//! [`panda_path_cost_sources_blocked_by_mesh_shape_cost_sources`] instead
//! -- both functions run in this crate's regular (non-ignored) suite.
//! State ids 2-4, 6, 8 also pass now and stay routed through
//! [`panda_cost_sources_blocked_by_mesh_shape_cost_sources`] the same way,
//! now joined by the formerly-failing id 5 (see above) -- not moved to the
//! passing-ids allowlist, matching the path-op split's own precedent of
//! exercising the group together rather than folding each id in as it
//! closes.
//!
//! # What each case isolates
//!
//! `panda_cost_sources_request.json` (state op, ids 1-11):
//! - id 1: `joint_values={}` (the established default self-colliding pose),
//!   no world object, `group_name` omitted -- upstream's state overload runs
//!   one `checkCollision(cost=true)` and swaps the result out with **no**
//!   `removeCostSources`/`removeOverlapping` pass (`planning_scene.cpp:2499-2510`),
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

use cspace_collision::{CostSource, LinkPaddingScale, ParryCollisionEnv, World};
use cspace_core::geometry::{Cuboid, Isometry3, Shape};
use cspace_core::model::{MeshSearchPaths, RobotModel};
use cspace_core::srdf::SrdfModel;
use cspace_core::state::RobotState;
use cspace_scene::PlanningScene;
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
/// (`crates/cspace-collision/src/parry.rs:1368-1388`): either no world
/// object/attached body at all (mesh-vs-mesh self-collision, or the
/// far-away/touch-suppressed zero cases), so the defect documented on
/// [`panda_cost_sources_blocked_by_mesh_shape_cost_sources`] cannot reach
/// them. Ids 10-11 are the same id-1 self-collision state at `max_costs`
/// 10 and 40 (both below the true count of 75), added to measure the
/// `max_costs` truncation rule -- `CostSource::Ord`
/// (`crates/cspace-collision/src/common.rs:144-160`) -- independently of
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
/// geometry to fall back to). §171's original dispatch defect and the
/// state-op box-geometry gap that followed it (id 2, nearest-match distance
/// `2.6899575667278616e-2` against the `1e-9` threshold) are both fixed:
/// `mesh_shape_cost_sources` now reports one box per pair
/// (`c6161b9`/`2607bef`, merged `f74a2b7`), and fitting the mesh
/// cost-source root box as an oriented OBB rather than an axis-aligned AABB
/// (`54250b1`) closed the geometry gap -- id 2 now measures
/// `6.661338147750939e-16`, see this file's module doc. Ids 3, 4, 6 pass
/// too, unaffected by either fix.
///
/// Id 5 (`group_name="hand"`) used to return `9` cost sources against an
/// oracle-expected `2` -- a count mismatch, not a nearest-match distance
/// gap either fix above could touch. It was never a `cspace-scene` defect:
/// `group_name` filtering was entirely unimplemented in `cspace-collision`'s
/// `check_self_collision`/`check_robot_collision`/`distance_self`/
/// `distance_robot` (all four received the field and never read it) while
/// that crate's own module doc claimed the omission matched upstream. It
/// does not -- upstream's `checkSelfCollisionHelper`/`checkRobotCollisionHelper`
/// call `cd.enableGroup(getRobotModel())` unconditionally
/// (`collision_env_fcl.cpp:281,336`), resolving through
/// `getUpdatedLinkModelsSet()`, and `collisionCallback`/`distanceCallback`
/// (`collision_detection_fcl/collision_common.cpp:79-94,482-500`) drop a pair only when neither side
/// is in the active set. Fixed in `cspace-collision` by `585a79e`. Measured
/// directly this round with `cargo nextest run -p cspace-scene
/// --run-ignored all`: passes, 86/86. Isolated by reverting `585a79e`'s
/// `parry.rs` hunk alone and rerunning this test: fails with the identical
/// `count mismatch left: 9 right: 2`; restoring it passes again. No longer
/// blocked, so the `#[ignore]` is removed -- see
/// `doc/claim-audit/cspace-scene.md`'s row for this case, now EXPIRED, for
/// why the earlier attribution to this crate was wrong twice over (wrong
/// crate, and reasoned from which layer *should* own group filtering
/// rather than from a count or a probe).
#[test]
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
/// a `mesh_shape_cost_sources` call. §171's dispatch defect and a later,
/// distinct box-geometry defect (id 3's count mismatch, `1` actual vs `5`
/// expected) are both fixed now -- the second by fitting the mesh
/// cost-source root box as an oriented OBB rather than an axis-aligned AABB
/// (`54250b1`), per the module doc's "§171's dispatch defect is fixed" and
/// "Oracle-verified coverage for the split branch" sections. Measured
/// directly this round with `cargo nextest run -p cspace-scene --test
/// cost_sources_parity panda_path_cost_sources_blocked_by_mesh_shape_cost_sources
/// --run-ignored all`: passes. No longer blocked, so the `#[ignore]` is
/// removed -- a passing test left `#[ignore]`d never runs again, and a
/// regression here would go unnoticed.
///
/// Ids 4 (`max_costs=2` -> expected `4`) and 6 (`max_costs=3` -> expected
/// `6`) in this same loop are oracle-sourced proof `remove_cost_sources`'s
/// per-axis split branch is real and does grow the set past `max_costs` --
/// see the module doc's "Oracle-verified coverage for the split branch"
/// section; this test now exercises that proof directly rather than only
/// reasoning about it while gated behind an `#[ignore]`.
#[test]
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
